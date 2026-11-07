#pragma once

// ---------------------------------------------------------------------------
// Standpoint SlotSolver (S2) — pure, read-only, synchronous.
// Given (target ref, relation, agent) -> a validated world stand-point +
// compass heading, or a coded failure. NEVER moves anything.
//
// Implements the corrected slot gate from Report 04 §A1:
//   allowed_i = NIF marker[i].entryProperties
//               AND enable bit i of TESFurniture::furnFlags (kSit0..kSit23)
//               MINUS the NAM0 disabledPoints of the entryPointDataArray
//                     element whose `entryPoint` field == i  (NEVER by array index)
// Angles: radians + compass convention internally (0=+Y north, clockwise);
// converted to DEGREES only at the Papyrus seam (Report 04 §C3).
// ---------------------------------------------------------------------------

namespace standpoint {

enum class Relation : std::int32_t {
    kNearest = 0,  // nearest valid slot (to agent if given, else to target front)
    kFront   = 1,  // sides are in the TARGET's own facing frame
    kRight   = 2,
    kBack    = 3,
    kLeft    = 4,
};

enum FailCode : std::int32_t {
    kOK             = 0,
    kNoTarget       = 1,
    kNo3D           = 2,
    kNoCandidates   = 3,  // every marker gated off / no bounds
    kSnapFailed     = 4,  // no navmesh under any candidate
    kUnreachable    = 5,  // CanNavigateToPosition refused the winner
    kAllOccupied    = 6,
};

struct SlotResult {
    RE::NiPoint3 pos{};            // world, navmesh-snapped
    float        headingRad{ 0.f };// compass, [0, 2pi)
    std::int32_t markerIndex{ -1 };// FRN marker that produced it; -1 = geometric
    std::int32_t fail{ kOK };
    float        quality{ 0.f };   // 1.0 clean; penalties applied
};

SlotResult Solve(RE::TESObjectREFR* a_target, Relation a_rel, RE::Actor* a_agent, float a_standoff);

// Introspection for the test harness / consumers.
std::int32_t          GetMarkerCount(RE::TESObjectREFR* a_target);
// out: [wx, wy, wz, headingRad, effectivePoints, animTypes, enabled]
bool                  GetMarkerInfo(RE::TESObjectREFR* a_target, std::int32_t a_index, float (&a_out)[7]);

// ---- v0.8.0: the clock solve (the move API, StandpointAPI.h IStandpoint2) -------------------------------
// Where around the target (a clock hour in the target's own facing: 0 / 12 = in front, 3 = its right, 6 = behind,
// 9 = its left; < 0 = the side nearest the agent) and how far (centre of the target to the stand-point, game units).
// The arrival heading = "facing the target" turned clockwise by a_orientHours * 30 deg (6 = her back to the target).
// A target WITHOUT loaded 3D (a custom marker) is solved from its position and Z angle alone, with no distance floor.
// Furniture is an object like any other here (v0.8.1): its sit / sleep markers are NOT consulted.
// A blocked hour (the navmesh snap slid the point > 16 u) is skipped for the next hour, alternating to both sides.
struct ClockResult {
    SlotResult slot{};
    float      gotHours{ 0.f };        // the hour she is actually sent to, [0, 12)
    float      orientOffsetRad{ 0.f }; // clockwise turn from "facing the target" (the driver's short-arrival turn adds it)
};
// a_close (v0.9.0, close contact): an ACTOR target gets no distance floor - the point is exactly as far as asked.
ClockResult SolveClock(RE::TESObjectREFR* a_target, RE::Actor* a_agent, float a_distUnits, float a_facingHours,
                       float a_orientHours, bool a_close = false);

// The point distances and clock hours are measured from: an actor's or a model-less marker's position, else the centre
// of the object's bounds box (a table's reference point is rarely its middle).
RE::NiPoint3 TargetCentre(RE::TESObjectREFR* a_target);

// The live result line: centre-to-centre distance (game units), the walker's clock hour around the target and the clock
// hour of her own facing (0 = facing the target), both [0, 12).
void MeasureClock(RE::TESObjectREFR* a_target, RE::Actor* a_walker, float& a_distUnits, float& a_facingHours,
                  float& a_orientHours);

// ---- 1.6.0: EXPORTS for the steered session's physics solve (RelativeSolve.cpp; report "1.6.0 Plan" S1 §2.1) ----------
// Solve() and SolveClock() above are UNCHANGED (the engine sessions, SE / AE, the Papyrus Solve / SolveClock natives). These
// hand out the SAME candidates, ranking and rules, UNSNAPPED, so that the steered session can test each one by physics
// (the planner's probes) instead of the navmesh snap. Pure reads; the snap helper reads the navmesh (main thread, or a VM
// thread as Solve() does today).

// One candidate exactly as Solve() builds it (before its snap / occupancy ladder).
struct SolverCand {
    RE::NiPoint3 pos{};              // where she would stand (z = the marker's / the target's z: unsnapped, no floor test)
    RE::NiPoint3 anchor{};           // the marker / side point it serves
    float        faceHeading{ 0.f }; // where she looks once there (compass rad)
    float        sideAngle{ 0.f };   // its bearing in the TARGET's facing frame (rad, -pi..pi)
    float        sideMiss{ 0.f };    // how far that bearing is from the asked side (rad; 0 = on it, or kNearest)
    std::int32_t markerIndex{ -1 };  // the furniture marker it serves; -1 = geometric
    bool         occupied{ false };  // that marker is used / reserved by another actor (Solve() skips it)
};
struct GoToFrame {
    bool  targetIsActor{ false };
    bool  usedMarkers{ false };      // furniture entry points (true), else box sides / the actor's four sides
    int   tiers{ 1 };                // tiers worth trying: an object's box sides 3 (Solve()'s retreat pads), an actor 2 (+16 u),
                                     // furniture markers 1
    float approach{ 0.f };           // Solve()'s approach distance (u) for tier 0
};
// Solve()'s candidates for a_tier (0 .. a_frame.tiers - 1), ranked as Solve() ranks them: the asked side first (20-deg
// buckets), then nearest to the agent. Tier 1-2 of an object = Solve()'s wider pads; tier 1 of an actor = the approach + 16 u.
// false = none (no target, no 3D, no candidates).
// 1.6.1 (C3): a_boxSides = an object's BOX SIDES (3 tiers) even when it has furniture markers - the steered GoTo's fallback when
// every entry is refused (Solve() itself never falls back: a furniture target there is its entries only).
bool GoToCandidates(RE::TESObjectREFR* a_target, Relation a_rel, RE::Actor* a_agent, float a_standoff, int a_tier,
                    std::vector<SolverCand>& a_out, GoToFrame& a_frame, bool a_boxSides = false);

// SolveClock()'s frame for one request.
struct ClockFrame {
    RE::NiPoint3 centre{};           // TargetCentre: hours and distances are measured from it
    RE::NiPoint3 tpos{};             // the target's own position
    RE::NiPoint3 boxH{};             // the bounds' half extents (model frame, scaled; 24 x 24 without bounds)
    float        targetFacing{ 0.f };// compass rad
    float        wantRel{ 0.f };     // the asked side, clockwise from the target's front (rad); a_facingHours < 0 = her side
    float        orientOff{ 0.f };   // her facing, clockwise from "toward the target" (rad)
    float        agentR{ 32.f };     // the walker's radius (AgentRadius)
    float        floorZ{ 0.f };      // the level every ring point must stand on (TargetFloorZ)
    bool         bare{ false };      // a model-less marker (no 3D): no floor
    bool         targetIsActor{ false };
    bool         nearest{ false };   // a_facingHours < 0
};
bool  ClockFrameOf(RE::TESObjectREFR* a_target, RE::Actor* a_agent, float a_facingHours, float a_orientHours, ClockFrame& a_out);
// SolveClock()'s hour order: the asked hour, then +1, -1, +2, -2 ... +5, -5, then +6 (rad, target frame)
void  ClockHourOrder(float a_wantRel, float (&a_rels)[12]);
// SolveClock()'s distance for hour a_rel: as asked, or its floor (box reach + her radius + 16 for an actor / + 8 for an
// object) whichever is more, + 24 x a_tier. A bare marker, or an actor with a_close: exactly as asked (no floor, no tier).
float ClockRingDistance(const ClockFrame& a_f, float a_rel, float a_distUnits, bool a_close, int a_tier);
float BoxReachAt(const ClockFrame& a_f, float a_rel);         // the box's reach from its centre toward a_rel (model frame)
float ClockHours(float a_rad);                                // an angle -> clock hours [0, 12)
float ClockLevelAllowance(float a_flatDist);                  // the F1 level rule: max(40, 0.58 x flat distance)
// The solver's navmesh snap (96 u up, 160 u down from a_pos, in a_context's cell / worldspace). false = no navmesh there.
bool  SnapToNavmeshNear(RE::TESObjectREFR* a_context, RE::NiPoint3& a_pos);
float AgentRadiusOf(RE::Actor* a_agent);                      // the walker's radius as the solver reads it (else 32)

}  // namespace standpoint
