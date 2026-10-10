#include "PCH.h"
#include "SlotSolver.h"

namespace logger = SKSE::log;

namespace standpoint {
namespace {

constexpr float kPi  = 3.14159265358979f;
constexpr float k2Pi = 6.28318530717959f;

// Entry-property bits (NIF entryProperties AND record NAM0 share this layout).
constexpr std::uint16_t kEntryFront  = 1 << 0;
constexpr std::uint16_t kEntryBehind = 1 << 1;
constexpr std::uint16_t kEntryRight  = 1 << 2;
constexpr std::uint16_t kEntryLeft   = 1 << 3;
constexpr std::uint16_t kEntrySideMask = kEntryFront | kEntryBehind | kEntryRight | kEntryLeft;

float Norm2Pi(float a)
{
    while (a >= k2Pi) a -= k2Pi;
    while (a < 0.f) a += k2Pi;
    return a;
}

float NormPi(float a)
{
    while (a > kPi) a -= k2Pi;
    while (a < -kPi) a += k2Pi;
    return a;
}

// Compass: 0 = +Y (north), clockwise. forward(t) = (sin t, cos t, 0).
RE::NiPoint3 Dir(float a_heading)
{
    return { std::sin(a_heading), std::cos(a_heading), 0.f };
}

float HeadingOf(const RE::NiPoint3& a_v)
{
    return Norm2Pi(std::atan2(a_v.x, a_v.y));
}

float Dist2D(const RE::NiPoint3& a, const RE::NiPoint3& b)
{
    const float dx = a.x - b.x, dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

struct Candidate {
    RE::NiPoint3 pos{};
    RE::NiPoint3 anchor{};            // the marker / side point this slot serves
    float        faceHeading{ 0.f };  // where the ARRIVED actor should look
    float        sideAngle{ 0.f };    // candidate bearing in the target's facing frame
    std::int32_t markerIndex{ -1 };
    float        penalty{ 0.f };
};

// The agent's capsule radius; char-controller when available, else 32 u.
float AgentRadius(RE::Actor* a_agent)
{
    if (a_agent) {
        if (auto* cc = a_agent->GetCharController()) {
            // collisionBound.extents.x is the capsule half-width in game units.
            const float r = cc->collisionBound.extents.x;
            if (r > 4.f && r < 96.f) return r;
        }
    }
    return 32.f;
}

// ---- furniture candidates -------------------------------------------------

void FurnitureCandidates(RE::TESObjectREFR* a_target, RE::NiAVObject* a_root,
                         float a_approach, std::vector<Candidate>& a_out)
{
    auto* fmn = RE::BSFurnitureMarkerNode::FindBSFurnitureMarkerNode(a_root);
    if (!fmn || fmn->markers.empty()) return;

    auto* furn = a_target->GetObjectReference() ? a_target->GetObjectReference()->As<RE::TESFurniture>() : nullptr;

    for (std::uint32_t i = 0; i < fmn->markers.size(); ++i) {
        const auto& m = fmn->markers[i];

        // Gate 1: record enable bit (furnFlags kSit0..kSit23). i>=24: treat enabled.
        if (furn && i < 24) {
            if (!(furn->furnFlags.underlying() & (1u << i))) continue;
        }
        // Gate 2: the NIF marker's own entry points.
        std::uint16_t points = m.entryProperties.underlying() & kEntrySideMask;
        // Gate 3: NAM0 subtraction — ONLY from the element whose entryPoint == i.
        if (furn) {
            for (const auto& ep : furn->entryPointDataArray) {
                if (ep.entryPoint == i) {
                    points &= static_cast<std::uint16_t>(~ep.disabledEntryPoints.disabledPoints.underlying());
                    break;
                }
            }
        }
        if (!points) continue;

        // World transform of the marker (full 3x3 — handles sloped refs and the
        // baked root rotations; we read the engine's own frame, never the file).
        const RE::NiPoint3 world = a_root->world * m.offset;
        const RE::NiPoint3 localFwd = Dir(m.heading);
        const RE::NiPoint3 worldFwd = a_root->world.rotate * localFwd;
        const float markerHeading = HeadingOf(worldFwd);

        // One candidate per allowed entry direction, marker-local frame:
        // Front = heading, Behind = +pi, Right = +pi/2, Left = -pi/2.
        const struct { std::uint16_t bit; float off; } dirs[] = {
            { kEntryFront, 0.f }, { kEntryBehind, kPi },
            { kEntryRight, kPi * 0.5f }, { kEntryLeft, -kPi * 0.5f },
        };
        for (const auto& d : dirs) {
            if (!(points & d.bit)) continue;
            const float entryHeading = Norm2Pi(markerHeading + d.off);
            Candidate c;
            c.pos = world + Dir(entryHeading) * a_approach;
            c.pos.z = world.z;
            c.faceHeading = Norm2Pi(entryHeading + kPi);  // face back at the marker
            c.markerIndex = static_cast<std::int32_t>(i);
            c.anchor = world;
            logger::info("  cand m{}: world=({:.0f},{:.0f},{:.0f}) off=({:.1f},{:.1f},{:.1f}) mh={:.1f} eh={:.1f} app={:.0f} -> pos=({:.0f},{:.0f})",
                         i, world.x, world.y, world.z, m.offset.x, m.offset.y, m.offset.z,
                         markerHeading * 57.2958f, entryHeading * 57.2958f, a_approach, c.pos.x, c.pos.y);
            a_out.push_back(c);
        }
    }
}

// ---- geometric candidates (OBND box sides; actors get a ring) -------------

void GeometricCandidates(RE::TESObjectREFR* a_target, RE::NiAVObject* a_root,
                         float a_pad, std::vector<Candidate>& a_out)
{
    RE::NiPoint3 c{ 0.f, 0.f, 0.f };
    RE::NiPoint3 h{ 24.f, 24.f, 0.f };

    auto* base = a_target->GetObjectReference();
    if (auto* bound = base ? base->As<RE::TESBoundObject>() : nullptr) {
        const float scale = a_target->GetScale();
        const auto& bd = bound->boundData;
        c = { (bd.boundMin.x + bd.boundMax.x) * 0.5f * scale,
              (bd.boundMin.y + bd.boundMax.y) * 0.5f * scale, 0.f };
        h = { (bd.boundMax.x - bd.boundMin.x) * 0.5f * scale,
              (bd.boundMax.y - bd.boundMin.y) * 0.5f * scale, 0.f };
        if (h.x < 8.f) h.x = 8.f;
        if (h.y < 8.f) h.y = 8.f;
    }

    // Model-space side points: FRONT +Y, BACK -Y, RIGHT +X, LEFT -X.
    const RE::NiPoint3 sides[4] = {
        { c.x, c.y + h.y + a_pad, 0.f },   // front
        { c.x + h.x + a_pad, c.y, 0.f },   // right
        { c.x, c.y - h.y - a_pad, 0.f },   // back
        { c.x - h.x - a_pad, c.y, 0.f },   // left
    };

    const RE::NiPoint3 tpos = a_target->GetPosition();
    const float tz = a_target->GetAngle().z;  // compass rad
    const bool flat = std::fabs(a_target->GetAngle().x) < 0.05f && std::fabs(a_target->GetAngle().y) < 0.05f;

    for (int s = 0; s < 4; ++s) {
        RE::NiPoint3 w;
        if (!flat && a_root) {
            w = a_root->world * sides[s];
        } else {
            // rotate model (x,y) by compass angle tz: the TRANSPOSE of textbook CCW.
            const float ct = std::cos(tz), st = std::sin(tz);
            w = { tpos.x + sides[s].x * ct + sides[s].y * st,
                  tpos.y - sides[s].x * st + sides[s].y * ct,
                  tpos.z };
        }
        w.z = tpos.z;
        Candidate cd;
        cd.pos = w;
        cd.anchor = w;
        // face the target's centre
        RE::NiPoint3 centreW = flat || !a_root ? RE::NiPoint3{ tpos.x, tpos.y, tpos.z } : a_root->world * c;
        cd.faceHeading = HeadingOf(RE::NiPoint3{ centreW.x - w.x, centreW.y - w.y, 0.f });
        cd.markerIndex = -1;
        a_out.push_back(cd);
    }
}

// ---- occupancy ------------------------------------------------------------

bool MarkerTaken(RE::TESObjectREFR* a_target, std::int32_t a_marker, RE::Actor* a_agent)
{
    if (a_marker < 0) return false;
    const auto check = [&](auto* a_extra) -> bool {
        if (!a_extra) return false;
        for (const auto& u : a_extra->usedMarkers) {
            if (static_cast<std::int32_t>(u.markerID) != a_marker) continue;
            auto held = u.actorinMarker.get();
            if (held && held.get() != a_agent) return true;
        }
        return false;
    };
    if (check(a_target->extraList.GetByType<RE::ExtraUsedMarkers>())) return true;
    if (auto* res = a_target->extraList.GetByType<RE::ExtraReservedMarkers>()) {
        for (const auto& u : res->reservedMarkerArray) {
            if (static_cast<std::int32_t>(u.markerID) != a_marker) continue;
            auto held = u.actorinMarker.get();
            if (held && held.get() != a_agent) return true;
        }
    }
    return false;
}

// ---- navmesh snap ---------------------------------------------------------

bool SnapToNavmesh(RE::TESObjectREFR* a_target, RE::NiPoint3& a_pos)
{
    auto* pathing = RE::Pathing::GetSingleton();
    if (!pathing) return false;
    RE::BSTSmartPointer<RE::BSPathingCell> cell;
    if (!pathing->GetPathingCell(a_pos, a_target->GetParentCell(), a_target->GetWorldspace(), cell) || !cell) {
        return false;
    }
    RE::FindTriangleForLocationFilterCheckDeltaZ filter;
    filter.maxDistAbove = 96.f;    // floor-relative window (NOT storey-derived; report 04)
    filter.maxDistBelow = 160.f;
    RE::NiPoint3 out;
    if (!pathing->FindClosestPointOnNavmesh(cell, a_pos, filter, out)) {
        return false;
    }
    a_pos = out;
    return true;
}

}  // namespace

// ---------------------------------------------------------------------------

SlotResult Solve(RE::TESObjectREFR* a_target, Relation a_rel, RE::Actor* a_agent, float a_standoff)
{
    SlotResult r;
    if (!a_target) { r.fail = kNoTarget; return r; }
    auto* root = a_target->Get3D();
    if (!root) { r.fail = kNo3D; return r; }

    const float agentR = AgentRadius(a_agent);
    const bool targetIsActor = a_target->As<RE::Actor>() != nullptr;

    // approach distance from the anchor point
    float approach = a_standoff;
    if (approach <= 0.f) approach = targetIsActor ? 85.f : 0.f;
    approach = (std::max)(approach, agentR + 16.f);

    std::vector<Candidate> cands;
    if (!targetIsActor) {
        FurnitureCandidates(a_target, root, (std::max)(48.f, approach), cands);
    }
    const bool usedMarkers = !cands.empty();

    // Geometric fallback with the B1 retreat/expansion ladder.
    const float pads[3] = { agentR + 8.f, agentR + 32.f, agentR + 56.f };
    int padTier = 0;
    if (!usedMarkers) {
        GeometricCandidates(a_target, root, targetIsActor ? approach : pads[0], cands);
    }
    if (cands.empty()) { r.fail = kNoCandidates; return r; }

    // Side classification in the TARGET's own facing frame.
    const float targetFacing = Norm2Pi(a_target->GetAngle().z);
    const RE::NiPoint3 tpos = a_target->GetPosition();
    for (auto& c : cands) {
        c.sideAngle = NormPi(HeadingOf(RE::NiPoint3{ c.pos.x - tpos.x, c.pos.y - tpos.y, 0.f }) - targetFacing);
    }

    const auto sideWanted = [&](const Candidate& c) -> float {
        // penalty by angular distance from the requested side's bearing
        float want;
        switch (a_rel) {
        case Relation::kFront: want = 0.f; break;
        case Relation::kRight: want = kPi * 0.5f; break;
        case Relation::kBack:  want = kPi; break;
        case Relation::kLeft:  want = -kPi * 0.5f; break;
        default: return 0.f;  // kNearest: no side preference
        }
        return std::fabs(NormPi(c.sideAngle - want));
    };

    // Rank: requested side first, then nearest to the agent (or to the target front).
    const RE::NiPoint3 ref = a_agent ? a_agent->GetPosition() : tpos;
    std::sort(cands.begin(), cands.end(), [&](const Candidate& a, const Candidate& b) {
        const float sa = sideWanted(a), sb = sideWanted(b);
        if (std::fabs(sa - sb) > 0.35f) return sa < sb;   // ~20 deg bucket tolerance
        return Dist2D(a.pos, ref) < Dist2D(b.pos, ref);
    });

    // v0.7.2 (2026-10-01, VRCD handover 79 §13 - the user: "standpoint is supposed to scan and notice the 'front not
    // accessible' and chose another position"): for an ACTOR target, a side whose ideal point is not standable - the navmesh
    // snap had to SLIDE it more than kBlockedSlide - is BLOCKED: the floor is there, but no room for the walker's body.
    // Measured 2026-10-01 (Carmella facing a table): her front point slid 50 u along the table edge and was kept as "front";
    // the walker stuck 16 u short, wedged between her and the table, and was never turned (93 deg off). At the closest
    // standoff the slide pulled the point 20 u INTO her (27 u from her centre). On open floor every side snapped with 0 u.
    // Pass 0 skips blocked sides, so the next side in rank (the nearest other side) is used. Pass 1 is the pre-0.7.2 rule
    // (any standable point) - used only when EVERY side is blocked, so no target that solved before fails now.
    // Furniture and objects keep their own rules (markers / the retreat ladder).
    constexpr float kBlockedSlide = 16.f;
    const int passes = targetIsActor ? 2 : 1;

    // Validation ladder over the ranked list (occupancy -> snap; expand pads on
    // total failure for the geometric path).
    for (int attempt = 0; attempt <= 2; ++attempt) {
      for (int pass = 0; pass < passes; ++pass) {
        if (pass == 1) logger::info("  every side of the actor is blocked - taking the least-bad standable point (the pre-0.7.2 rule)");
        for (auto& c : cands) {
            if (MarkerTaken(a_target, c.markerIndex, a_agent)) { c.penalty += 10.f; logger::info("  rej m{}: occupied", c.markerIndex); continue; }
            const RE::NiPoint3 preSnap = c.pos;  // untouched copy — guard vs any callee mutation
            RE::NiPoint3 snapped = preSnap;
            if (!SnapToNavmesh(a_target, snapped)) { logger::info("  rej m{}: no navmesh at ({:.0f},{:.0f})", c.markerIndex, preSnap.x, preSnap.y); continue; }
            const float moved = Dist2D(snapped, preSnap);
            if (moved > 96.f) { logger::info("  rej m{}: snap moved {:.0f} u", c.markerIndex, moved); continue; }
            // sanity: the slot must still serve its anchor
            const float fromAnchor = Dist2D(snapped, c.anchor);
            if (fromAnchor > (std::max)(96.f, approach + 64.f)) { logger::info("  rej m{}: {:.0f} u from anchor", c.markerIndex, fromAnchor); continue; }
            if (pass == 0 && targetIsActor && moved > kBlockedSlide) {
                logger::info("  blocked: the side at {:.0f} deg of the actor's facing slid {:.0f} u on the navmesh (> {:.0f}) - no room "
                             "for the walker there, trying the next side", c.sideAngle * 57.2958f, moved, kBlockedSlide);
                continue;
            }

            // v0.3.2: Actor::CanNavigateToPosition is REMOVED from the ladder. It shares
            // its address with UpdateNavPos (RELOCATION_ID 46050/47314, batch-1 warning)
            // and was caught IN THE FIELD mutating its second argument — it overwrote a
            // validated slot with a navmesh point near the AGENT (bed candidate
            // (116,178,293) became (106,53,-20) after the call; "moved" guard had passed
            // at 0). Its bool is also false for a SITTING agent, which vetoed every
            // destination for a parked walker. The navmesh snap above already proves the
            // slot is standable; transit feasibility is SeverActions' job.
            r.pos = snapped;
            r.headingRad = c.faceHeading >= 6.2822f ? 0.f : c.faceHeading;
            r.markerIndex = c.markerIndex;
            r.quality = (std::max)(0.f, 1.f - sideWanted(c) * 0.3f - moved / 96.f * 0.4f - c.penalty);
            r.fail = kOK;
            logger::info("Solve: ok side={} marker={} pos=({:.0f},{:.0f},{:.0f}) face={:.1f}deg moved={:.0f} q={:.2f} tier={} at={:.0f}deg{}",
                         static_cast<int>(a_rel), r.markerIndex, r.pos.x, r.pos.y, r.pos.z,
                         r.headingRad * 180.f / kPi, moved, r.quality, attempt, c.sideAngle * 57.2958f,
                         pass == 1 ? " (all sides blocked)" : "");
            return r;
        }
      }
        // expand only the geometric path (markers do not move)
        if (usedMarkers || targetIsActor) break;
        ++padTier;
        if (padTier > 2) break;
        cands.clear();
        GeometricCandidates(a_target, root, pads[padTier], cands);
        for (auto& c : cands) {
            c.sideAngle = NormPi(HeadingOf(RE::NiPoint3{ c.pos.x - tpos.x, c.pos.y - tpos.y, 0.f }) - targetFacing);
        }
        std::sort(cands.begin(), cands.end(), [&](const Candidate& a, const Candidate& b) {
            const float sa = sideWanted(a), sb = sideWanted(b);
            if (std::fabs(sa - sb) > 0.35f) return sa < sb;
            return Dist2D(a.pos, ref) < Dist2D(b.pos, ref);
        });
    }

    if (r.fail == kOK || r.fail == 0) r.fail = kSnapFailed;
    logger::info("Solve: FAIL code={} target={:X} rel={}", r.fail, a_target->GetFormID(), static_cast<int>(a_rel));
    return r;
}

std::int32_t GetMarkerCount(RE::TESObjectREFR* a_target)
{
    if (!a_target) return -1;
    auto* root = a_target->Get3D();
    if (!root) return -1;
    auto* fmn = RE::BSFurnitureMarkerNode::FindBSFurnitureMarkerNode(root);
    return fmn ? static_cast<std::int32_t>(fmn->markers.size()) : 0;
}

bool GetMarkerInfo(RE::TESObjectREFR* a_target, std::int32_t a_index, float (&a_out)[7])
{
    if (!a_target) return false;
    auto* root = a_target->Get3D();
    if (!root) return false;
    auto* fmn = RE::BSFurnitureMarkerNode::FindBSFurnitureMarkerNode(root);
    if (!fmn || a_index < 0 || a_index >= static_cast<std::int32_t>(fmn->markers.size())) return false;

    const auto& m = fmn->markers[static_cast<std::uint32_t>(a_index)];
    auto* furn = a_target->GetObjectReference() ? a_target->GetObjectReference()->As<RE::TESFurniture>() : nullptr;

    std::uint16_t points = m.entryProperties.underlying() & kEntrySideMask;
    bool enabled = true;
    if (furn && a_index < 24) enabled = (furn->furnFlags.underlying() & (1u << a_index)) != 0;
    if (furn) {
        for (const auto& ep : furn->entryPointDataArray) {
            if (ep.entryPoint == static_cast<std::uint32_t>(a_index)) {
                points &= static_cast<std::uint16_t>(~ep.disabledEntryPoints.disabledPoints.underlying());
                break;
            }
        }
    }

    const RE::NiPoint3 world = root->world * m.offset;
    const float heading = HeadingOf(root->world.rotate * Dir(m.heading));
    a_out[0] = world.x; a_out[1] = world.y; a_out[2] = world.z;
    a_out[3] = heading;
    a_out[4] = static_cast<float>(points);
    a_out[5] = static_cast<float>(m.animationType.underlying());
    a_out[6] = enabled ? 1.f : 0.f;
    return true;
}

#include "SlotSolverClock.inc"   // v0.8.0: SolveClock / TargetCentre / MeasureClock (the move API)

}  // namespace standpoint
