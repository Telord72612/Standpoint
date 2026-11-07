// ─────────────────────────────────────────────────────────────────────────────
//  RelativeSolve.cpp — 1.6.0 the physics solve of a relative spot (see RelativeSolve.h; report "1.6.0 Plan" S1 §2.1).
//  Main thread only (SteerSession's Frame, inside the walk pump's frame step). One log line per candidate (condensed) so a
//  run's analysis sees every hour swap and why; the planner's probes are asked QUIET (their answer comes in a ProbeNote).
// ─────────────────────────────────────────────────────────────────────────────
#include "PCH.h"
#include "RelativeSolve.h"
#include "PathPlanner.h"

#include <chrono>
#include <format>

namespace logger = SKSE::log;

namespace standpoint_relsolve {
namespace {

    constexpr float kPi          = 3.14159265358979f;
    constexpr float k2Pi         = 6.28318530717959f;
    constexpr float kHour        = k2Pi / 12.f;
    constexpr float kRadToDeg    = 57.2957795f;
    constexpr float kApproachOut = 12.f;     // E lies this far outside the keep-out circle (S1 §2.1: R + 12)
    constexpr float kReachPad    = 4.f;      // the reach leg starts her clearance + 4 beyond the target's edge
    constexpr float kReachMinLeg = 16.f;     // a reach leg shorter than this is not probed (S1 §2.1)
    constexpr float kActorPass1  = 16.f;     // an actor's pass 1: the ring 16 u further out (S1 §2.1)
    constexpr float kAvoidRel    = 2.f * kHour - 0.02f;   // a failed side: no side within 2 hours of it is planned again
    constexpr float kPreferRel   = 0.27f;    // a re-solve's own side: candidates within ~15 deg of it are tried first
    constexpr float kNavFlat     = 16.f;     // "on the navmesh": the solver's snap moved the spot at most this far flat ...
    constexpr float kNavHigh     = 24.f;     // ... and this far up / down
    constexpr float kDiscPad     = 8.f;      // a standing actor's disc = her radius + the walker's clearance + 8 (the planner's)
    constexpr float kDiscRange   = 700.f;    // standing actors farther than this from the target cannot touch a candidate
    constexpr int   kPassBoxSide = 2;        // 1.6.1 (C3): a furniture target's box side, tried after all its entries

    float Norm2Pi(float a_a)
    {
        while (a_a >= k2Pi) a_a -= k2Pi;
        while (a_a < 0.f) a_a += k2Pi;
        return a_a;
    }

    float NormPi(float a_a)
    {
        while (a_a > kPi) a_a -= k2Pi;
        while (a_a < -kPi) a_a += k2Pi;
        return a_a;
    }

    RE::NiPoint3 Dir(float a_heading) { return { std::sin(a_heading), std::cos(a_heading), 0.f }; }
    float        HeadingOf(float a_dx, float a_dy) { return Norm2Pi(std::atan2(a_dx, a_dy)); }
    float        Dist2D(const RE::NiPoint3& a_a, const RE::NiPoint3& a_b) { return std::hypot(a_a.x - a_b.x, a_a.y - a_b.y); }

    float ControllerRadius(RE::Actor* a_actor, float a_default)
    {
        if (a_actor) {
            if (auto* cc = a_actor->GetCharController()) {
                const float r = cc->collisionBound.extents.x;
                if (r > 4.f && r < 96.f) return r;
            }
        }
        return a_default;
    }

    bool Avoided(const std::vector<float>& a_avoid, float a_rel)
    {
        for (const float a : a_avoid) {
            if (std::fabs(NormPi(a_rel - a)) < kAvoidRel) return true;
        }
        return false;
    }

    // 1.6.1 (R3 §2: "the log cannot show which side failed or the heights"): a stand-spot refusal with the planner's numbers when
    // one of its 4 side floor rays refused (PathPlanner ProbeNote 1.6.1), and the hits on the target's own body it stepped through
    std::string StandWhy(const standpoint_path::ProbeNote& a_n)
    {
        std::string s = a_n.why.empty() ? std::string("(no reason given)") : a_n.why;
        if (a_n.side >= 0 && a_n.side <= 3) {
            static constexpr const char* kSide[] = { "N", "E", "S", "W" };
            static constexpr const char* kWhat[] = { "a floor out of her step", "blocked", "no floor", "deep water", "no loaded cell" };
            const int w = (a_n.sideWhat >= 0 && a_n.sideWhat <= 4) ? a_n.sideWhat : 0;
            s += std::format(" [side ray {} at {:.0f} u: {}", kSide[a_n.side], a_n.rc, kWhat[w]);
            if (w == 0 || w == 1) s += std::format(" at z {:.0f}, the spot's floor {:.0f} (window +-{:.0f})", a_n.sideZ, a_n.centreZ, a_n.window);
            if (w == 3) s += std::format(" {:.0f} u deep", a_n.sideDepth);
            if (a_n.sideLayer) s += std::format(", layer {}", a_n.sideLayer);
            if (a_n.sideRef) s += std::format(", [{:08X}]", a_n.sideRef);
            s += "]";
        }
        if (a_n.ignored > 0) s += std::format(" ({} hit(s) on the target's own body stepped through)", a_n.ignored);
        return s;
    }

}  // namespace

float WalkerRadius(RE::Actor* a_walker)
{
    return ControllerRadius(a_walker, 20.f);
}

float KeepOutRadius(RE::TESObjectREFR* a_target, RE::Actor* a_walker)
{
    float tr = 24.f;
    if (auto* ta = a_target ? a_target->As<RE::Actor>() : nullptr) tr = ControllerRadius(ta, 24.f);
    return tr + WalkerRadius(a_walker) + 8.f;
}

bool StandingActorAt(RE::Actor* a_walker, RE::FormID a_target, const RE::NiPoint3& a_p, RE::FormID& a_who, float& a_r)
{
    a_who = 0;
    a_r = 0.f;
    if (!a_walker) return false;
    auto* cellW = a_walker->GetParentCell();
    if (!cellW) return false;
    const float rc = WalkerRadius(a_walker) + 4.f;
    bool        hit = false;
    const auto  consider = [&](RE::Actor* a) {
        if (hit || !a || a == a_walker || !a->Is3DLoaded() || a->GetFormID() == a_target) return;
        auto* cellA = a->GetParentCell();
        if (!cellA) return;
        if (cellW->IsInteriorCell() || cellA->IsInteriorCell()) {
            if (cellW != cellA) return;
        } else if (a->GetWorldspace() != a_walker->GetWorldspace()) {
            return;
        }
        if (a->IsDead() || a->IsDisabled() || a->IsMoving()) return;
        const RE::NiPoint3 p = a->GetPosition();
        const float        r = ControllerRadius(a, 20.f) + rc + kDiscPad;
        if (std::hypot(a_p.x - p.x, a_p.y - p.y) < r) {
            hit = true;
            a_who = a->GetFormID();
            a_r = r;
        }
    };
    if (auto* lists = RE::ProcessLists::GetSingleton()) {
        for (const auto* arr : { &lists->highActorHandles, &lists->middleHighActorHandles }) {
            for (const auto& h : *arr) {
                auto ptr = h.get();
                consider(ptr.get());
            }
        }
    }
    consider(RE::PlayerCharacter::GetSingleton());
    return hit;
}

bool Job::Begin(const Ask& a_ask, std::string& a_why)
{
    m_ask = a_ask;
    m_cands.clear();
    m_discs.clear();
    m_next = 0;
    m_sub = 0;
    m_done = false;
    m_spot = Spot{};
    m_lastWhy.clear();
    m_ms = 0.0;
    m_steps = 0;
    m_ignoredHits = 0;
    auto* walker = m_ask.who ? RE::TESForm::LookupByID<RE::Actor>(m_ask.who) : nullptr;
    auto* target = m_ask.target ? RE::TESForm::LookupByID<RE::TESObjectREFR>(m_ask.target) : nullptr;
    if (!walker || !walker->Is3DLoaded()) {
        a_why = "the walker is not a loaded NPC";
        return false;
    }
    if (!target) {
        a_why = "the target does not resolve";
        return false;
    }
    const bool        clock = m_ask.kind == Kind::kClock;
    standpoint::ClockFrame f;
    standpoint::ClockFrameOf(target, walker, clock ? m_ask.facingH : 0.f, clock ? m_ask.orientH : 0.f, f);
    // FIX ROUND (review R1 #7): a FIXED frame yaw - the player as target: a re-solve keeps the first solve's frame (his headset's
    // live yaw would turn the spot with every glance) and follows only his position. GoTo's candidates are turned by the same angle.
    float frameDelta = 0.f;
    if (m_ask.frameYawSet) {
        const float yaw = Norm2Pi(m_ask.frameYaw);
        frameDelta = NormPi(yaw - f.targetFacing);
        f.targetFacing = yaw;
        if (f.nearest) {
            const RE::NiPoint3 ap = walker->GetPosition();
            if (Dist2D(ap, f.centre) > 1.f) f.wantRel = Norm2Pi(HeadingOf(ap.x - f.centre.x, ap.y - f.centre.y) - yaw);
        }
    }
    m_tpos = target->GetPosition();
    m_centre = f.centre;
    m_targetFacing = f.targetFacing;
    m_floorZ = f.floorZ;
    m_orientOff = clock ? f.orientOff : 0.f;
    m_targetActor = f.targetIsActor;
    m_keepR = f.bare ? 0.f : KeepOutRadius(target, walker);   // a model-less marker has no body: no keep-out circle (no E)
    m_rc = WalkerRadius(walker) + 4.f;
    const float targetR = m_targetActor ? ControllerRadius(target->As<RE::Actor>(), 24.f) : 0.f;

    const auto add = [&](const RE::NiPoint3& a_pos, float a_rel, float a_face, float a_reach, std::int32_t a_marker, int a_pass) {
        Cand c;
        c.pos = a_pos;
        c.rel = Norm2Pi(a_rel);
        c.distU = Dist2D(a_pos, m_centre);
        c.faceRad = Norm2Pi(a_face);
        c.reachU = a_reach;
        c.marker = a_marker;
        c.pass = a_pass;
        m_cands.push_back(c);
    };

    int skippedAvoid = 0, skippedTaken = 0;
    int boxWithheld = 0;     // 1.6.1 FIX-UP (C3b): furniture box sides left out - k entr(ies) taken
    if (clock) {
        if (f.bare && m_ask.distU < 1.f) {
            // a model-less marker at distance 0: stand exactly ON it, looking the way it points (+ the orientation)
            RE::NiPoint3 p = m_tpos;
            p.z = m_floorZ;
            add(p, 0.f, f.targetFacing + f.orientOff, 0.f, -1, 0);
        } else {
            float rels[12];
            standpoint::ClockHourOrder(f.wantRel, rels);
            std::vector<float> order;
            order.reserve(13);
            if (m_ask.preferSet) order.push_back(m_ask.preferRel);
            for (const float r : rels) {
                if (m_ask.preferSet && std::fabs(NormPi(r - m_ask.preferRel)) < 0.02f) continue;
                order.push_back(r);
            }
            const bool closeActor = m_ask.close && m_targetActor;
            for (int pass = 0; pass < 2; ++pass) {
                // pass 0: the asked distance (with the solver's floors); pass 1: an object's retreat tiers (+24, +48), an actor
                // 16 u further out; a bare marker and a close request to an actor have no pass 1 (exactly as asked)
                std::vector<int> tiers;
                if (pass == 0) {
                    tiers.push_back(0);
                } else if (!f.bare && !closeActor) {
                    if (m_targetActor) {
                        tiers.push_back(0);
                    } else {
                        tiers.push_back(1);
                        tiers.push_back(2);
                    }
                }
                for (const int tier : tiers) {
                    for (const float rel : order) {
                        if (Avoided(m_ask.avoidRels, rel)) {
                            if (pass == 0 && tier == 0) ++skippedAvoid;
                            continue;
                        }
                        float d = standpoint::ClockRingDistance(f, rel, m_ask.distU, m_ask.close, tier);
                        if (pass == 1 && m_targetActor) d += kActorPass1;
                        const float  bearing = f.targetFacing + rel;
                        RE::NiPoint3 p{ m_centre.x + std::sin(bearing) * d, m_centre.y + std::cos(bearing) * d, m_floorZ };
                        const float  face = d < 1.f ? f.targetFacing + f.orientOff
                                                    : HeadingOf(m_centre.x - p.x, m_centre.y - p.y) + f.orientOff;
                        add(p, rel, face, m_targetActor ? targetR : standpoint::BoxReachAt(f, rel), -1, pass);
                    }
                }
            }
        }
        m_spot.faceLive = m_targetActor;
        m_spot.faceOffsetRad = f.orientOff;
    } else {
        // GoTo: the solver's own candidates (furniture entries, box sides, an actor's four sides) in its own rank
        const auto          relation = static_cast<standpoint::Relation>(std::clamp(m_ask.relation, 0, 4));
        standpoint::GoToFrame gf;
        std::vector<standpoint::SolverCand> list;
        const auto addGoTo = [&](const standpoint::SolverCand& a_c, bool a_marker, int a_pass) {
            if (a_c.occupied) {
                ++skippedTaken;
                return;
            }
            const float rel = Norm2Pi(a_c.sideAngle);
            if (Avoided(m_ask.avoidRels, rel)) {
                ++skippedAvoid;
                return;
            }
            RE::NiPoint3 p = a_c.pos;
            if (!a_marker) p.z = m_floorZ;             // box sides / actor sides: the target's floor (markers keep theirs)
            if (frameDelta != 0.f) {
                // (the fixed frame: the candidate turned round the target's position by the frame's turn, its facing too)
                const float dx = p.x - m_tpos.x, dy = p.y - m_tpos.y;
                const float cs = std::cos(frameDelta), sn = std::sin(frameDelta);
                p.x = m_tpos.x + dx * cs + dy * sn;
                p.y = m_tpos.y - dx * sn + dy * cs;
            }
            const float reach = m_targetActor ? targetR : standpoint::BoxReachAt(f, rel);
            add(p, rel, a_c.faceHeading + frameDelta, reach, a_c.markerIndex, a_pass);
        };
        int  tiers = 1;
        bool markers = false;
        for (int tier = 0; tier < tiers && tier < 3; ++tier) {
            if (!standpoint::GoToCandidates(target, relation, walker, m_ask.standoffU, tier, list, gf)) break;
            tiers = gf.tiers;
            markers = gf.usedMarkers;
            for (const auto& c : list) addGoTo(c, gf.usedMarkers, tier > 0 ? 1 : 0);
        }
        // 1.6.1 (C3; run R3 §2: GoTo FRONT of a chair - its one entry refused, SPOT none, O 1 in 0.6 s): furniture with markers gets
        // its BOX SIDES after its entries (Solve()'s pads, 3 tiers; tried in order, so only when every entry is refused - a brink
        // beside it, no floor, a side that had no route): she stands at a side of it instead of O 1.
        // FIX-UP (review R1 § VERIFY 1.6.1 NIT-a / NIT-b): NOT when an entry is TAKEN - someone sits there, and she is never stood
        // beside an occupied chair (every entry taken answers O 1 at once, as 1.5.1 did); the sides are ranked by SIDE across the
        // tiers - the asked side at all three distances before the next side (Solve()'s 20-deg buckets; within one: tier, then the
        // solver's own order)
        if (markers && skippedTaken == 0) {
            std::vector<std::pair<standpoint::SolverCand, int>> box;   // (a box side, its tier)
            int                                                 boxTiers = 3;
            for (int tier = 0; tier < boxTiers && tier < 3; ++tier) {
                if (!standpoint::GoToCandidates(target, relation, walker, m_ask.standoffU, tier, list, gf, true)) break;
                boxTiers = gf.tiers;
                for (const auto& c : list) box.emplace_back(c, tier);
            }
            std::stable_sort(box.begin(), box.end(), [](const auto& a_a, const auto& a_b) {
                return std::lround(a_a.first.sideMiss / 0.35f) < std::lround(a_b.first.sideMiss / 0.35f);
            });
            for (const auto& b : box) addGoTo(b.first, false, kPassBoxSide);
        } else if (markers) {
            boxWithheld = skippedTaken;
        }
        m_spot.faceLive = m_targetActor;
        m_spot.faceOffsetRad = 0.f;
    }
    if (m_ask.preferSet && m_ask.kind == Kind::kGoTo) {
        // a re-solve: her current side first (she does not flip sides on a moving target)
        std::stable_partition(m_cands.begin(), m_cands.end(),
                              [&](const Cand& a_c) { return std::fabs(NormPi(a_c.rel - m_ask.preferRel)) < kPreferRel; });
    }
    if (m_cands.empty()) {
        a_why = std::format("no candidate to test ({} side(s) near a failed one, {} furniture marker(s) taken)", skippedAvoid, skippedTaken);
        return false;
    }

    // the standing actors (the planner's own eligibility - PathPlanner ScanActors): their discs are refused (the probes scan no
    // actors). The walker and the target are left out; moving actors are the walk's business.
    if (auto* cellW = walker->GetParentCell()) {
        std::vector<RE::FormID> seen;
        const auto consider = [&](RE::Actor* a) {
            if (!a || a == walker || !a->Is3DLoaded()) return;
            const RE::FormID id = a->GetFormID();
            if (id == m_ask.target || std::find(seen.begin(), seen.end(), id) != seen.end()) return;
            seen.push_back(id);
            auto* cellA = a->GetParentCell();
            if (!cellA) return;
            if (cellW->IsInteriorCell() || cellA->IsInteriorCell()) {
                if (cellW != cellA) return;
            } else if (a->GetWorldspace() != walker->GetWorldspace()) {
                return;
            }
            if (a->IsDead() || a->IsDisabled() || a->IsMoving()) return;
            const RE::NiPoint3 p = a->GetPosition();
            if (Dist2D(p, m_centre) > kDiscRange) return;
            Disc d;
            d.id = id;
            d.x = p.x;
            d.y = p.y;
            d.r = ControllerRadius(a, 20.f) + m_rc + kDiscPad;
            m_discs.push_back(d);
        };
        if (auto* lists = RE::ProcessLists::GetSingleton()) {
            for (const auto* arr : { &lists->highActorHandles, &lists->middleHighActorHandles }) {
                for (const auto& h : *arr) {
                    auto ptr = h.get();
                    consider(ptr.get());
                }
            }
        }
        consider(RE::PlayerCharacter::GetSingleton());
    }

    const char* what = clock ? "the clock" : "GoTo";
    const auto  boxSides = std::count_if(m_cands.begin(), m_cands.end(), [](const Cand& a_c) { return a_c.pass == kPassBoxSide; });
    logger::info("STEER #{} SOLVE {}: {} candidate(s){} around [{:08X}] ({}{}){} - keep-out circle {:.0f} u, her clearance {:.0f} u, {} standing "
                 "actor(s) kept out, the target's floor z {:.0f}{}",
                 m_ask.steerId, what, m_cands.size(),
                 boxSides      ? std::format(" ({} of them the box sides after the furniture's entries)", boxSides)
                 : boxWithheld ? std::format(" (no box sides: {} entr(ies) taken - someone sits there)", boxWithheld)
                               : std::string(),
                 m_ask.target,
                 clock ? std::format("asked {:.0f} u at hour {:.1f}, orientation {:.1f}h", m_ask.distU, m_ask.facingH, m_ask.orientH)
                       : std::format("relation {}, standoff {:.0f} u", m_ask.relation, m_ask.standoffU),
                 m_ask.close ? ", CLOSE" : "", m_ask.preferSet ? std::format(" - a RE-SOLVE, hour {:.1f} first", standpoint::ClockHours(m_ask.preferRel)) : std::string(),
                 m_keepR, m_rc, m_discs.size(), m_floorZ,
                 skippedAvoid || skippedTaken ? std::format(" ({} side(s) near a failed one left out, {} marker(s) taken)", skippedAvoid, skippedTaken)
                                              : std::string());
    m_t0 = std::chrono::steady_clock::now();
    return true;
}

void Job::LogCand(const Cand& a_c, const std::string& a_verdict) const
{
    logger::info("STEER #{} SOLVE {}/{} hour {:.1f} at {:.0f} u{}{}: {}", m_ask.steerId, m_next + 1, m_cands.size(),
                 standpoint::ClockHours(a_c.rel), a_c.distU,
                 a_c.pass == kPassBoxSide ? " (a box side - every entry before it was refused)" : a_c.pass ? " (pass 1)" : "",
                 a_c.marker >= 0 ? std::format(" marker {}", a_c.marker) : std::string(), a_verdict);
}

void Job::Finish(bool a_found)
{
    m_done = true;
    m_spot.found = a_found;
    if (!a_found && m_spot.why.empty()) m_spot.why = m_lastWhy.empty() ? std::string("no candidate passed") : m_lastWhy;
    m_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - m_t0).count();
    if (a_found) {
        logger::info("STEER #{} SPOT hour {:.1f} at {:.0f} u - ({:.0f},{:.0f},{:.0f}), heading {:.0f} deg{}{}{} | {} candidate(s), {} probe(s) in {} "
                     "frame(s), {:.0f} ms wall",
                     m_ask.steerId, m_spot.hours, m_spot.distU, m_spot.pos.x, m_spot.pos.y, m_spot.pos.z, m_spot.headingRad * kRadToDeg,
                     m_spot.onNavmesh ? ", on the navmesh" : ", OFF the navmesh",
                     m_spot.inCircle ? std::format(", CLOSE: inside the keep-out circle - in from E ({:.0f},{:.0f},{:.0f})", m_spot.approach.x,
                                                   m_spot.approach.y, m_spot.approach.z)
                                     : std::string(),
                     m_spot.faceLive ? std::format(", faces the target live + {:.0f} deg", m_spot.faceOffsetRad * kRadToDeg) : std::string(),
                     m_spot.tested, m_spot.probes, m_steps, m_ms);
    } else {
        logger::info("STEER #{} SPOT none: no standable spot ({} candidate(s), {} probe(s) in {} frame(s), {:.0f} ms) - last: {}", m_ask.steerId,
                     m_spot.tested, m_spot.probes, m_steps, m_ms, m_spot.why);
    }
}

bool Job::Step(double a_budgetMs)
{
    if (m_done) return true;
    auto* walker = m_ask.who ? RE::TESForm::LookupByID<RE::Actor>(m_ask.who) : nullptr;
    auto* target = m_ask.target ? RE::TESForm::LookupByID<RE::TESObjectREFR>(m_ask.target) : nullptr;
    if (!walker || !walker->Is3DLoaded() || !target) {
        m_spot.why = "the walker or the target is gone";
        Finish(false);
        return true;
    }
    ++m_steps;
    const auto t0 = std::chrono::steady_clock::now();
    const auto next = [&](const std::string& a_verdict) {
        LogCand(m_cands[m_next], a_verdict);
        m_lastWhy = std::format("hour {:.1f}: {}", standpoint::ClockHours(m_cands[m_next].rel), a_verdict);
        ++m_next;
        m_sub = 0;
    };
    const auto accept = [&](bool a_inCircle) {
        const Cand& c = m_cands[m_next];
        m_spot.pos = m_cur;
        m_spot.headingRad = c.faceRad >= 6.2822f ? 0.f : c.faceRad;
        m_spot.rel = c.rel;
        m_spot.hours = standpoint::ClockHours(c.rel);
        m_spot.distU = Dist2D(m_cur, m_centre);
        m_spot.inCircle = a_inCircle;
        m_spot.approach = a_inCircle ? m_e : RE::NiPoint3{};
        m_spot.markerIndex = c.marker;
        RE::NiPoint3 sn = m_cur;
        m_spot.onNavmesh = standpoint::SnapToNavmeshNear(target, sn) && Dist2D(sn, m_cur) <= kNavFlat && std::fabs(sn.z - m_cur.z) <= kNavHigh;
        std::string ok = a_inCircle ? "OK (close: the approach point and the way in are clear)" : "OK";
        if (m_ignoredHits > 0) ok += std::format(" - its side rays stepped through the target's own body ({} hit(s))", m_ignoredHits);
        LogCand(c, ok);
        Finish(true);
    };
    do {
        if (m_next >= m_cands.size()) {
            Finish(false);
            return true;
        }
        const Cand& c = m_cands[m_next];
        if (m_sub == 0) {
            ++m_spot.tested;
            const Disc* hit = nullptr;
            for (const auto& d : m_discs) {
                if (std::hypot(c.pos.x - d.x, c.pos.y - d.y) < d.r) {
                    hit = &d;
                    break;
                }
            }
            if (hit) {
                next(std::format("refused: a standing actor's place ([{:08X}], disc {:.0f} u)", hit->id, hit->r));
                continue;
            }
            float                     floorZ = c.pos.z, depth = 0.f;
            standpoint_path::ProbeNote note;
            ++m_spot.probes;
            // 1.6.1 (C3; R3 §2): the side rays step through the TARGET's own body - a spot beside a chair is not "a brink" because
            // the chair's seat stands 35 u up next to it (the centre ray, the headroom and the plans keep their rules)
            if (!standpoint_path::ProbeStandSpot(walker, c.pos, m_ask.depthFrac, floorZ, depth, true, &note, standpoint_path::kStandNzTerrain,
                                                 m_ask.target)) {
                next(std::format("refused: not a place to stand - {}", StandWhy(note)));
                continue;
            }
            m_ignoredHits = note.ignored;
            m_cur = RE::NiPoint3{ c.pos.x, c.pos.y, floorZ };
            const float flat = Dist2D(m_cur, m_centre);
            const float allowZ = standpoint::ClockLevelAllowance(flat);
            const float dz = floorZ - m_floorZ;
            if (std::fabs(dz) > allowZ) {
                next(std::format("refused: other level - {:.0f} u {} the target's floor (allowed {:.0f})", std::fabs(dz), dz < 0.f ? "below" : "above",
                                 allowZ));
                continue;
            }
            if (Dist2D(m_cur, m_tpos) < m_keepR) {
                // a CLOSE spot: the approach point E on the same bearing, just outside the keep-out circle
                const float h = HeadingOf(m_cur.x - m_tpos.x, m_cur.y - m_tpos.y);
                const RE::NiPoint3 dir = Dir(h);
                m_e = RE::NiPoint3{ m_tpos.x + dir.x * (m_keepR + kApproachOut), m_tpos.y + dir.y * (m_keepR + kApproachOut), floorZ };
                m_sub = 1;
            } else {
                m_sub = 2;
            }
        } else if (m_sub == 1) {
            float                     ez = m_e.z, edepth = 0.f;
            standpoint_path::ProbeNote note;
            ++m_spot.probes;
            if (!standpoint_path::ProbeStandSpot(walker, m_e, m_ask.depthFrac, ez, edepth, true, &note, standpoint_path::kStandNzTerrain,
                                                 m_ask.target)) {
                next(std::format("refused: the approach point E ({:.0f},{:.0f}) is not a place to stand - {}", m_e.x, m_e.y, StandWhy(note)));
                continue;
            }
            m_e.z = ez;
            m_sub = 3;
        } else if (m_sub == 3) {
            standpoint_path::ProbeNote note;
            ++m_spot.probes;
            if (!standpoint_path::ProbeSegment(walker, m_e, m_cur, m_ask.depthFrac, true, &note)) {
                next(std::format("refused: body blocked on the way in from E at {:.0f} u - {}", note.failAt,
                                 note.why.empty() ? std::string("(no reason given)") : note.why));
                continue;
            }
            accept(true);
            return true;
        } else {
            // a FAR spot: the reach leg from just beyond the target's edge toward it (a spot behind a wall from the target fails)
            const float out = c.reachU + m_rc + kReachPad;
            // FIX ROUND (review R1 #6): no reach leg when the spot lies ON the centre or within the leg's start + 16 u (ON a bare
            // marker at distance 0 its bearing was "north": the leg started 28 u north of the marker and a wall there refused it)
            if (Dist2D(m_cur, m_centre) <= out + kReachMinLeg) {
                accept(false);
                return true;
            }
            const float  h = HeadingOf(m_cur.x - m_centre.x, m_cur.y - m_centre.y);
            const RE::NiPoint3 dir = Dir(h);
            const RE::NiPoint3 k{ m_centre.x + dir.x * out, m_centre.y + dir.y * out, m_cur.z };
            standpoint_path::ProbeNote note;
            ++m_spot.probes;
            if (!standpoint_path::ProbeSegment(walker, k, m_cur, m_ask.depthFrac, true, &note)) {
                next(std::format("refused: body blocked - the reach leg from the target's edge fails at {:.0f} u - {}", note.failAt,
                                 note.why.empty() ? std::string("(no reason given)") : note.why));
                continue;
            }
            accept(false);
            return true;
        }
    } while (std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() < a_budgetMs);
    return m_done;
}

}  // namespace standpoint_relsolve
