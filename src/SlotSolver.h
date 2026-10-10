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

}  // namespace standpoint
