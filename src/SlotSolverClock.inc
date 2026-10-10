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
