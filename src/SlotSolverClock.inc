// ===========================================================================
// v0.8.0 - the clock solve (the move API). Included at the end of SlotSolver.cpp, inside namespace standpoint, so it
// shares that file's helpers. Everything in SlotSolver.cpp itself is untouched: the existing Solve() serves GoTo /
// Solve exactly as before (Bathing in Skyrim Renewed, VRCD).
// ===========================================================================
namespace {

constexpr float kHour = k2Pi / 12.f;   // one clock hour = 30 deg

// The bounds box in model space (centre + half extents, scaled). false = no bounds object.
bool BoxOf(RE::TESObjectREFR* a_target, RE::NiPoint3& a_c, RE::NiPoint3& a_h)
{
    a_c = { 0.f, 0.f, 0.f };
    a_h = { 24.f, 24.f, 0.f };
    auto* base = a_target->GetObjectReference();
    auto* bound = base ? base->As<RE::TESBoundObject>() : nullptr;
    if (!bound) return false;
    const float scale = a_target->GetScale();
    const auto& bd = bound->boundData;
    a_c = { (bd.boundMin.x + bd.boundMax.x) * 0.5f * scale, (bd.boundMin.y + bd.boundMax.y) * 0.5f * scale, 0.f };
    a_h = { (bd.boundMax.x - bd.boundMin.x) * 0.5f * scale, (bd.boundMax.y - bd.boundMin.y) * 0.5f * scale, 0.f };
    if (a_h.x < 8.f) a_h.x = 8.f;
    if (a_h.y < 8.f) a_h.y = 8.f;
    return true;
}

// How far the box reaches from its centre along the model-frame direction at a_rel (clockwise from the front, +Y).
float BoxReach(const RE::NiPoint3& a_h, float a_rel)
{
    const float sx = std::fabs(std::sin(a_rel)), cy = std::fabs(std::cos(a_rel));
    float reach = (std::numeric_limits<float>::max)();
    if (sx > 1e-4f) reach = (std::min)(reach, a_h.x / sx);
    if (cy > 1e-4f) reach = (std::min)(reach, a_h.y / cy);
    return reach;
}

float HoursOf(float a_rad)
{
    float h = Norm2Pi(a_rad) / kHour;
    if (h >= 11.9995f) h = 0.f;
    return h;
}

// v1.2.0 (F1, the BiSR letter of 2026-10-30, report 11 §3): a ring point must stand on the TARGET's level. The navmesh
// snap looks 96 u up and 160 u down for the nearest triangle, so a point over water or off a platform edge snapped onto
// the floor BELOW with a tiny flat slide - Breezehome: the walker was sent to the main floor at the foot of the bath's
// rock wall, about 60 u under the woman in the raised bath, and the engine walked her there and never took the step.
// "Same level" = within one step (40 u) or a walkable slope (30 deg) over the flat distance, whichever is more.
constexpr float kLevelStep  = 40.f;
constexpr float kLevelSlope = 0.58f;    // tan(30 deg)

float LevelAllowance(float a_flatDist)
{
    return (std::max)(kLevelStep, kLevelSlope * a_flatDist);
}

// The height the target stands on. An actor: her feet (her position). Anything else (furniture, an object on a table,
// a model-less marker): the navmesh under it when one is found close by, else its own position.
float TargetFloorZ(RE::TESObjectREFR* a_target, const RE::NiPoint3& a_tpos)
{
    if (a_target->As<RE::Actor>()) return a_tpos.z;
    RE::NiPoint3 fp = a_tpos;
    if (SnapToNavmesh(a_target, fp) && Dist2D(fp, a_tpos) <= 96.f) return fp.z;
    return a_tpos.z;
}

}  // namespace

RE::NiPoint3 TargetCentre(RE::TESObjectREFR* a_target)
{
    if (!a_target) return {};
    const RE::NiPoint3 tpos = a_target->GetPosition();
    if (a_target->As<RE::Actor>() || !a_target->Get3D()) return tpos;
    RE::NiPoint3 c, h;
    if (!BoxOf(a_target, c, h)) return tpos;
    const float tz = a_target->GetAngle().z;
    const float ct = std::cos(tz), st = std::sin(tz);
    return { tpos.x + c.x * ct + c.y * st, tpos.y - c.x * st + c.y * ct, tpos.z };
}

void MeasureClock(RE::TESObjectREFR* a_target, RE::Actor* a_walker, float& a_distUnits, float& a_facingHours,
                  float& a_orientHours)
{
    a_distUnits = a_facingHours = a_orientHours = 0.f;
    if (!a_target || !a_walker) return;
    const RE::NiPoint3 c = TargetCentre(a_target);
    const RE::NiPoint3 w = a_walker->GetPosition();
    const float targetFacing = Norm2Pi(a_target->GetAngle().z);
    const float walkerFacing = Norm2Pi(a_walker->GetAngle().z);
    a_distUnits = Dist2D(c, w);
    if (a_distUnits < 1.f) {
        // standing ON the target (a marker at distance 0): no side; her facing is read against the target's own
        a_orientHours = HoursOf(walkerFacing - targetFacing);
        return;
    }
    a_facingHours = HoursOf(HeadingOf(RE::NiPoint3{ w.x - c.x, w.y - c.y, 0.f }) - targetFacing);
    a_orientHours = HoursOf(walkerFacing - HeadingOf(RE::NiPoint3{ c.x - w.x, c.y - w.y, 0.f }));
}

ClockResult SolveClock(RE::TESObjectREFR* a_target, RE::Actor* a_agent, float a_distUnits, float a_facingHours,
                       float a_orientHours, bool a_close)
{
    ClockResult out;
    SlotResult& r = out.slot;
    if (!a_target) { r.fail = kNoTarget; return out; }

    auto* root = a_target->Get3D();                 // null = a model-less marker: solved from position + angle
    const bool targetIsActor = a_target->As<RE::Actor>() != nullptr;
    const float agentR = AgentRadius(a_agent);
    const RE::NiPoint3 tpos = a_target->GetPosition();
    const float targetFacing = Norm2Pi(a_target->GetAngle().z);
    const RE::NiPoint3 centre = TargetCentre(a_target);
    const float orientOff = Norm2Pi(a_orientHours * kHour);
    out.orientOffsetRad = orientOff;

    // the asked side, clockwise from the target's front; "nearest" = the side the agent already stands on
    const bool nearest = a_facingHours < 0.f;
    float wantRel = 0.f;
    if (!nearest) {
        wantRel = Norm2Pi(a_facingHours * kHour);
    } else if (a_agent) {
        const RE::NiPoint3 ap = a_agent->GetPosition();
        if (Dist2D(ap, centre) > 1.f) {
            wantRel = Norm2Pi(HeadingOf(RE::NiPoint3{ ap.x - centre.x, ap.y - centre.y, 0.f }) - targetFacing);
        }
    }

    const auto finish = [&](const RE::NiPoint3& a_pos, float a_face, std::int32_t a_marker, float a_moved, const char* a_how) {
        r.pos = a_pos;
        const float face = Norm2Pi(a_face + orientOff);
        r.headingRad = face >= 6.2822f ? 0.f : face;
        r.markerIndex = a_marker;
        r.quality = (std::max)(0.f, 1.f - a_moved / 96.f * 0.4f);
        r.fail = kOK;
        const float d = Dist2D(a_pos, centre);
        out.gotHours = d < 1.f ? 0.f : HoursOf(HeadingOf(RE::NiPoint3{ a_pos.x - centre.x, a_pos.y - centre.y, 0.f }) - targetFacing);
        logger::info("SolveClock: ok ({}) target={:X} asked hour={:.1f} dist={:.0f}u -> hour={:.1f} dist={:.0f}u pos=({:.0f},{:.0f},{:.0f}) "
                     "face={:.1f}deg (orient {:.1f}h) marker={} moved={:.0f}{}",
                     a_how, a_target->GetFormID(), a_facingHours, a_distUnits, out.gotHours, d, a_pos.x, a_pos.y, a_pos.z,
                     r.headingRad * 57.2958f, a_orientHours, a_marker, a_moved,
                     a_close && targetIsActor ? " CLOSE (no floor)" : "");
    };

    // v0.8.1 (2026-10-08, the first VR run of the move API): furniture is an OBJECT here. 0.8.0 sent her to the legal sit /
    // sleep entry closest to the asked hour and ignored the distance - a chair has ONE entry, so all six test requests
    // (hours 12, 3, 6, 9, nearest) were solved to the same spot at hour 8.1, 64 cm out. The user asked for "this hour, this
    // far" around whatever the target is. The marker gate stays where it belongs: Solve() / GoTo.

    // ---- the ring: an actor, an object (furniture too), a model-less marker ---------------------------------
    RE::NiPoint3 bc, bh;
    const bool bare = !root;
    if (!bare) BoxOf(a_target, bc, bh);

    // a bare marker at distance 0 = "stand exactly here, facing the way it points"
    if (bare && a_distUnits < 1.f) {
        RE::NiPoint3 p = tpos;
        if (!SnapToNavmesh(a_target, p) || Dist2D(p, tpos) > 96.f) {
            r.fail = kSnapFailed;
            logger::info("SolveClock: FAIL - no navmesh at the marker {:X}", a_target->GetFormID());
            return out;
        }
        finish(p, targetFacing, -1, Dist2D(p, tpos), "on the marker");
        return out;
    }

    // hour order: the asked hour, then alternating to both sides (+1, -1, +2, -2, ... 6)
    float rels[12];
    rels[0] = wantRel;
    for (int k = 1; k <= 5; ++k) {
        rels[2 * k - 1] = wantRel + k * kHour;
        rels[2 * k]     = wantRel - k * kHour;
    }
    rels[11] = wantRel + 6.f * kHour;

    // Distance floors (the existing rules, expressed centre to centre): an actor = her box + the walker's radius + 16
    // (bodies would overlap closer); an object = its box + the radius + 8, then the retreat ladder (+24, +48) when
    // nothing is standable (the bowl in the middle of a big table). A model-less marker has no floor.
    const int tiers = (bare || targetIsActor) ? 1 : 3;
    constexpr float kBlockedSlide = 16.f;
    const float floorZ = TargetFloorZ(a_target, tpos);   // v1.2.0 (F1): the level every ring point must stand on
    for (int pass = 0; pass < 2; ++pass) {              // 0 = the point must be standable where asked; 1 = least-bad
        for (int tier = 0; tier < tiers; ++tier) {
            for (float rel : rels) {
                float d = a_distUnits;
                // v0.9.0 close contact: an ACTOR target has no floor - the caller asked for a body-to-body distance and
                // gets exactly that point; what she can physically reach is the result line's business.
                if (!bare && !(a_close && targetIsActor)) {
                    const float floorD = BoxReach(bh, rel) + agentR + (targetIsActor ? 16.f : 8.f);
                    d = (std::max)(d, floorD) + 24.f * tier;
                }
                if (d < 0.f) d = 0.f;
                const float bearing = targetFacing + rel;
                const RE::NiPoint3 preSnap{ centre.x + std::sin(bearing) * d, centre.y + std::cos(bearing) * d, tpos.z };
                RE::NiPoint3 snapped = preSnap;
                if (!SnapToNavmesh(a_target, snapped)) continue;
                const float moved = Dist2D(snapped, preSnap);
                if (moved > 96.f) continue;
                // v1.2.0 (F1): another level is never a point, in either pass - no point is better than a wrong floor
                const float dz = snapped.z - floorZ;
                const float allowZ = LevelAllowance(Dist2D(snapped, centre));
                if (std::fabs(dz) > allowZ) {
                    logger::info("  clock other level: hour {:.1f} at {:.0f} u snapped {:.0f} u {} the target's floor (allowed {:.0f})",
                                 HoursOf(rel), d, std::fabs(dz), dz < 0.f ? "below" : "above", allowZ);
                    continue;
                }
                if (pass == 0 && moved > kBlockedSlide) {
                    logger::info("  clock blocked: hour {:.1f} at {:.0f} u slid {:.0f} u on the navmesh", HoursOf(rel), d, moved);
                    continue;
                }
                const float dc = Dist2D(snapped, centre);
                const float face = dc < 1.f ? targetFacing : HeadingOf(RE::NiPoint3{ centre.x - snapped.x, centre.y - snapped.y, 0.f });
                finish(snapped, face, -1, moved,
                       pass == 1 ? "least-bad standable point" : bare ? "marker ring" : targetIsActor ? "actor ring" : "object ring");
                return out;
            }
        }
    }
    r.fail = kSnapFailed;
    logger::info("SolveClock: FAIL - no standable point around {:X}", a_target->GetFormID());
    return out;
}

// ===========================================================================
// 1.6.0 - EXPORTS for the steered session's physics solve (RelativeSolve.cpp; report "1.6.0 Plan" S1 §2.1). Solve() and
// SolveClock() above are untouched: these functions only hand out the same candidates, ranking and rules (unsnapped) through
// SlotSolver.h, built from the same helpers.
// ===========================================================================

bool GoToCandidates(RE::TESObjectREFR* a_target, Relation a_rel, RE::Actor* a_agent, float a_standoff, int a_tier,
                    std::vector<SolverCand>& a_out, GoToFrame& a_frame, bool a_boxSides)
{
    a_out.clear();
    a_frame = GoToFrame{};
    if (!a_target) return false;
    auto* root = a_target->Get3D();
    if (!root) return false;

    // Solve()'s own approach rule
    const float agentR = AgentRadius(a_agent);
    const bool  targetIsActor = a_target->As<RE::Actor>() != nullptr;
    float       approach = a_standoff;
    if (approach <= 0.f) approach = targetIsActor ? 85.f : 0.f;
    approach = (std::max)(approach, agentR + 16.f);
    a_frame.targetIsActor = targetIsActor;
    a_frame.approach = approach;

    std::vector<Candidate> cands;
    if (!targetIsActor && !a_boxSides) {      // (1.6.1 C3: a_boxSides = the box sides only - the steered GoTo's fallback)
        FurnitureCandidates(a_target, root, (std::max)(48.f, approach), cands);
    }
    const bool usedMarkers = !cands.empty();
    a_frame.usedMarkers = usedMarkers;
    a_frame.tiers = usedMarkers ? 1 : (targetIsActor ? 2 : 3);
    if (a_tier < 0 || a_tier >= a_frame.tiers) return false;
    if (!usedMarkers) {
        const float pads[3] = { agentR + 8.f, agentR + 32.f, agentR + 56.f };   // Solve()'s retreat ladder (objects)
        const float pad = targetIsActor ? approach + 16.f * static_cast<float>(a_tier) : pads[a_tier];
        GeometricCandidates(a_target, root, pad, cands);
    }
    if (cands.empty()) return false;

    // Solve()'s side classification and ranking, unchanged
    const float        targetFacing = Norm2Pi(a_target->GetAngle().z);
    const RE::NiPoint3 tpos = a_target->GetPosition();
    for (auto& c : cands) {
        c.sideAngle = NormPi(HeadingOf(RE::NiPoint3{ c.pos.x - tpos.x, c.pos.y - tpos.y, 0.f }) - targetFacing);
    }
    const auto sideWanted = [&](const Candidate& c) -> float {
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
    const RE::NiPoint3 ref = a_agent ? a_agent->GetPosition() : tpos;
    std::sort(cands.begin(), cands.end(), [&](const Candidate& a, const Candidate& b) {
        const float sa = sideWanted(a), sb = sideWanted(b);
        if (std::fabs(sa - sb) > 0.35f) return sa < sb;   // ~20 deg bucket tolerance
        return Dist2D(a.pos, ref) < Dist2D(b.pos, ref);
    });

    a_out.reserve(cands.size());
    for (const auto& c : cands) {
        SolverCand o;
        o.pos = c.pos;
        o.anchor = c.anchor;
        o.faceHeading = c.faceHeading >= 6.2822f ? 0.f : c.faceHeading;
        o.sideAngle = c.sideAngle;
        o.sideMiss = sideWanted(c);
        o.markerIndex = c.markerIndex;
        o.occupied = MarkerTaken(a_target, c.markerIndex, a_agent);
        a_out.push_back(o);
    }
    return true;
}

bool ClockFrameOf(RE::TESObjectREFR* a_target, RE::Actor* a_agent, float a_facingHours, float a_orientHours, ClockFrame& a_out)
{
    a_out = ClockFrame{};
    if (!a_target) return false;
    auto* root = a_target->Get3D();                 // null = a model-less marker (SolveClock's "bare")
    a_out.bare = !root;
    a_out.targetIsActor = a_target->As<RE::Actor>() != nullptr;
    a_out.agentR = AgentRadius(a_agent);
    a_out.tpos = a_target->GetPosition();
    a_out.targetFacing = Norm2Pi(a_target->GetAngle().z);
    a_out.centre = TargetCentre(a_target);
    a_out.orientOff = Norm2Pi(a_orientHours * kHour);
    a_out.nearest = a_facingHours < 0.f;
    if (!a_out.nearest) {
        a_out.wantRel = Norm2Pi(a_facingHours * kHour);
    } else if (a_agent) {
        const RE::NiPoint3 ap = a_agent->GetPosition();
        if (Dist2D(ap, a_out.centre) > 1.f) {
            a_out.wantRel = Norm2Pi(HeadingOf(RE::NiPoint3{ ap.x - a_out.centre.x, ap.y - a_out.centre.y, 0.f }) - a_out.targetFacing);
        }
    }
    RE::NiPoint3 bc;
    a_out.boxH = RE::NiPoint3{ 24.f, 24.f, 0.f };
    if (!a_out.bare) BoxOf(a_target, bc, a_out.boxH);
    a_out.floorZ = TargetFloorZ(a_target, a_out.tpos);
    return true;
}

void ClockHourOrder(float a_wantRel, float (&a_rels)[12])
{
    a_rels[0] = a_wantRel;
    for (int k = 1; k <= 5; ++k) {
        a_rels[2 * k - 1] = a_wantRel + static_cast<float>(k) * kHour;
        a_rels[2 * k]     = a_wantRel - static_cast<float>(k) * kHour;
    }
    a_rels[11] = a_wantRel + 6.f * kHour;
}

float ClockRingDistance(const ClockFrame& a_f, float a_rel, float a_distUnits, bool a_close, int a_tier)
{
    float d = a_distUnits;
    if (!a_f.bare && !(a_close && a_f.targetIsActor)) {
        const float floorD = BoxReach(a_f.boxH, a_rel) + a_f.agentR + (a_f.targetIsActor ? 16.f : 8.f);
        d = (std::max)(d, floorD) + 24.f * static_cast<float>(a_tier);
    }
    return d < 0.f ? 0.f : d;
}

float BoxReachAt(const ClockFrame& a_f, float a_rel)
{
    if (a_f.bare) return 0.f;
    return BoxReach(a_f.boxH, a_rel);
}

float ClockHours(float a_rad)
{
    return HoursOf(a_rad);
}

float ClockLevelAllowance(float a_flatDist)
{
    return LevelAllowance(a_flatDist);
}

bool SnapToNavmeshNear(RE::TESObjectREFR* a_context, RE::NiPoint3& a_pos)
{
    if (!a_context) return false;
    return SnapToNavmesh(a_context, a_pos);
}

float AgentRadiusOf(RE::Actor* a_agent)
{
    return AgentRadius(a_agent);
}
