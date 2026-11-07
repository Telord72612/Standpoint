// ─────────────────────────────────────────────────────────────────────────────
//  PathPlanner.cpp — v1.3.0: the obstacle-aware route for a walk's last stretch (see PathPlanner.h).
//
//  THE QUESTIONS IT ASKS THE ENGINE (and nothing else - it never moves anyone):
//    * the navmesh: Pathing::FindClosestPointOnNavmesh with a tight height window (the floor of a grid cell; a cell
//      without navmesh is not walkable - the engine walks every leg, so every leg must end on navmesh)
//    * Havok rays: bhkWorld::PickObject (the pattern PPB runs in VR: the world's read lock around each pick, the answer
//      sanity-checked). A hit on a static / anim static / transparent / tree / prop / terrain / ground / invisible-wall
//      body BLOCKS; any other hit (an actor, loose clutter, a trigger, water) is stepped over.
//  THE BODY TEST, per piece of a straight line (pieces of at most 64 u): level rays at knee height on the centre line and
//  0.8 x her radius to each side, one at chest height, and two DIAGONAL rays on the centre line that cross every height
//  from 16 u to 112 u - so a flat obstacle (a table top, a shelf) cannot slip between the level rays (the review of
//  2026-10-31). A keep-out circle round the walk's own target: the way round never runs through her.
//  THE SEARCH: lazy weighted A* on a 32 u grid (a step is ray-checked only when the search pops it), then the cell path
//  is pulled straight into waypoints >= 64 u apart (a short corner point is pushed out to 64 u when it can be).
//  THE COST: slices; all running plans together ask at most kSliceQueries questions per frame (one SKSE task per plan
//  per frame); each plan logs its totals.
//
//  v1.4.0 THE EX MODE (StartEx; design B_Hybrid_Planner_Design.md, decisions D_Standpoint_140_Build_Spec.md): every grid
//  cell gets a kind - NAV (the navmesh, as 1.3.0), FLOOR (no navmesh; ONE down ray found standable ground) or WATER (NAV
//  or FLOOR under a water surface, depth <= depthFrac x her height). Steps between cells are relative (|dz| <= one step),
//  so a drop bigger than a step is never an edge - the search goes round it. Water = placed water activators first,
//  then the exterior water height per point, then the interior cell plane only where the cell has no placed water. The
//  cell path is split (a navmesh prefix / a direct suffix), pulled, spaced, and its corners chord-tested. Shared budget:
//  the same 96 questions per frame + a 1.0 ms per-frame time cap. A 1.3.0 Start plan never enters any of it.
//
//  v1.5.0 (D_Standpoint_150_Build_Spec.md §2; the body numbers B_Body_Limits_And_Raycast_Routes.md §7 C1-C17): a StartEx
//  plan with ANY of kNavGuide / kWadeSpot / kDoors / kActorsKeepOut runs the 1.5.0 rules (x.v15) - everything marked
//  "1.5.0" below. Without them every line runs exactly as 1.4.1 did (the 1.4.1 functions are kept and called as before).
//    * the BODY: one rise / drop <= 26 u (x s for s > 1) between floor samples <= ~11 u apart (two per 32 u step); cell to
//      cell <= 26 + L u; clearance rc = extents.x + 4 for every side ray; two side-column up-rays (floor + 30 .. Htop) per
//      edge and per <= 64 u piece; headroom to Htop = her controller bound's top + 2; the C12 lists, up to 6 step-overs; a step
//      that fails only beside her is retried 8 / 16 u aside (the cell remembers where it went)
//    * up to 4 FLOORS per map square (a level per distinct floor surface; each its own z, kind, depth, blocked directions)
//    * 3D ends: a straight line must end on the end point's floor (+- one step + 8 u); a partial route ends on the goal's floor
//    * the BOX: margin max(448, 0.75 d) <= 1024, a second pass 1.5 x as wide; ~10 questions per box cell (8,000-20,000)
//    * kNavGuide (the MIX): the navmesh's own corridor first, every leg tested with the rules above, a failed leg repaired
//      by a LOCAL ray search round it; no corridor -> the full ray search. A navmesh leg that fails = one CARPET line.
//    * kWadeSpot: the search ends at the first reachable wading spot (depth window + the 5-of-8 patch + the stand spot)
//    * kDoors: a closed, unlocked, ordinary door on a leg is passed (the point after it carries the door's FormID)
//    * kActorsKeepOut: actors standing still at plan start are discs to keep out of
//    * a CALIBRATION line per walker per game run (her hull cone, ring radius, slope limit, bounds - read only)
//
//  v1.5.1 (D_Standpoint_151_Build_Spec.md §4; the 1.5.0 VR run's hills: 15-54 k questions, 4 x out of budget, navmesh legs
//  "too steep at 57 / 72 u") - for a 1.5.0-flag plan only (x.v15); 1.3.0 and old-flag plans are untouched:
//    * TWO RISERS (review fix #2, the session's decision): 1.5.0's 26 u x scale (>= 24) on every floor - STATIC, furniture,
//      clutter, off the navmesh (a 30 u bench is never a step, a bench top never "on the navmesh") - and her body's
//      clamp(0.9 x her measured hull CONE, 24, 40) u only between land floors on the navmesh (unread: = the first)
//    * TERRAIN on the navmesh: a step whose floor samples are terrain / ground (layers 13 / 17) with the navmesh under them
//      may be as steep as the engine walks there - slope n.z >= 0.45, rise <= horizontal x tan 63 deg + 8 u (cells, steps,
//      the cell rule, line samples, brinks, lips); a floor beside her past the terrain riser only when the floor HALFWAY out
//      confirms a hillside (review fix #1: never the foot of a sheer drop); counted per plan
//    * the CELL CACHE (60 s; per physics world - another interior, going in / out, or a load clears it; exterior answers only
//      over loaded cells): the down rays of the search (by
//      world square + the window they answer), the navmesh and the water over each floor found, the cells' own tests and the
//      step tests (per walker size class) - only answers that touched nothing but fixed world layers (never a door, an
//      actor, clutter); the next plans in one place (stpf, BiSR) reuse them. A reuse costs no question (1/4 of one against
//      the search's own budget); hits are logged per plan
//    * the RANGE: 2400 u (the far phase hands over at 2100 u); the navmesh corridor's own cap grows with the distance
//
//  v1.6.0 (D_Standpoint_160_Build_Spec.md §1 items 6-10; design S2_Long_Path_Speed_Stairs_Wading.md §2-§6) - x.v15 plans and
//  the probes only; 1.3.0 Start and old-flag plans are untouched:
//    * THE SKELETON (NavGraph.cpp): the MIX's corridor is NavGraph's triangle path (stepped inside this plan's slices, or given by
//      the caller) instead of the 32 u grid corridor; its self-check failing -> the grid corridor; CHUNK MODE (a long skeleton
//      verified a chunk at a time: the walk appends each chunk while she walks); a goal outside the loaded cells = O 26's reason
//    * FASTER: slices bounded by TIME (1.0 ms a frame shared, a 2,000-question safety cap); water lookups 1/4 question; the second
//      pass CONTINUES the first: in the wider box (the grid made at the wider margin) after an edge contact, else - pass 1 out of
//      budget - ONCE in the same box with a fresh budget (fix round, R2 #5); no second pass only when pass 1's open list ran empty
//      inside its box; side-steps deferred until they are the open list's best (away from the side that hit first, then the
//      other side once - R2 #13); a MIX repair <= 2,000 q
//    * STAIRS: a fixed floor with the navmesh under it may be as steep as n.z 0.66 (cells, steps, line samples, corridor points,
//      spots aside); stand spots stay 0.85 (0.75 on navmesh terrain) - never on a stair; a goal that is no place to stand moves
//      by a RING search (16-160 u) instead of the 16 u march: on its own floor (+ 0.5 x the radius on navmesh terrain), or - its
//      floor refused only for the SLOPE of a stair / static ramp - at the ASKED height (stpf: his floor); nothing on the ring ->
//      the 1.5.1 march toward her (fix round, R2 #3 / #4)
//    * CORNERS: the +-128 u chord of every corner turning 15-60 deg (the run's look-ahead): a failure = kPtNoCut
//    * WADING: the entry (shin) -> scan 1 (2 body lengths: thigh ends it, else the nearest knee) -> scan 2 round the knee spot for
//      thigh - ONE route; a floor flood first aims the search at the nearest reachable shin water (a seed whose WATER SURFACE lies
//      more than a riser below her floor - a ledge; fix round, R2 #10)
//  v1.6.1 (the 17:12 VR run: R3 / R4; NavGraph's bit-5 rule = A1, there):
//    * A2 the flood is DIRECTED toward the seed (4,000 squares / 10,000 questions), keeps its own steps, and once it finds the water
//      a way-back pass (no question) makes its floor distance to that water the search's heuristic: the way round a ledge first
//    * A3 ProbeStandSpot: a_ignoreRef (the side floor rays step through the target's own body) and the side refusal in the note
//    * A4 a ring spot must FIT her body (the cell clearance + the 4 side columns at rc): no hopeless squeeze between a table and chairs
// ─────────────────────────────────────────────────────────────────────────────
#include "PCH.h"
#include "PathPlanner.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <format>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <unordered_map>
#include <unordered_set>

namespace logger = SKSE::log;

namespace standpoint_path {
namespace {

    constexpr float kToHavok      = 0.0142875f;   // game units -> Havok metres
    constexpr float kCell         = 32.f;         // grid step, game units
    constexpr float kKnee         = 28.f;         // ray heights above the floor (x her scale)
    constexpr float kChest        = 80.f;
    constexpr float kSweepLow     = 16.f;         // the two diagonal rays sweep this whole band
    constexpr float kSweepHigh    = 112.f;
    constexpr float kPiece        = 64.f;         // a long line is tested in pieces of at most this length
    constexpr float kSideFrac     = 0.8f;         // the side rays run at 0.8 x her radius (a doorway's frame must not stop her)
    constexpr float kLevelBand    = 48.f;         // a neighbour's floor is looked for this far above / below the current one
    constexpr float kMaxStep      = 40.f;         // the most height between two neighbouring cells
    constexpr float kMinLeg       = 64.f;         // waypoints at least this far apart (nearer, the engine poses her)
    constexpr int   kMaxWaypoints = 16;
    constexpr float kMaxDistance  = 1600.f;       // a local route only; further, the engine's own pathing
    constexpr int   kSliceQueries = 96;           // physics questions per frame, ALL running plans together
    constexpr int   kMinSlice     = 24;           // ... but every running plan gets at least this many
    constexpr int   kMaxQueries   = 5000;         // per plan
    constexpr float kHeurWeight   = 1.4f;         // weighted A*: far fewer expansions, a route at most ~40 % longer
    constexpr int   kRunning      = -9;
    constexpr int   kMaxJobs      = 16;

    // ---- 1.4.0 the Ex mode (B §0 table; H = her height, r = her radius, s = her scale) --------------------------------
    constexpr float  kXStepFrac      = 0.30f;    // one step up / down between cells = clamp(0.30 H, 24, 40); NAV<->NAV keeps kMaxStep
    constexpr float  kXStepMin       = 24.f;
    constexpr float  kXStepMax       = 40.f;
    constexpr float  kXHMin          = 48.f;     // GetHeight() is used when inside 48-400 u, else 128 x s
    constexpr float  kXHMax          = 400.f;
    constexpr float  kXHDefault      = 128.f;
    constexpr float  kXDepthFracDef  = 0.45f;    // the water depth cap = depthFrac x H ...
    constexpr float  kXDepthFracMin  = 0.05f;
    constexpr float  kXDepthFracMax  = 0.65f;    // ... never >= 0.65 H (the engine swims from a submerged level of 0.70)
    constexpr float  kXWet           = 4.f;      // deeper than this = wet
    // 1.4.1 A START IN DEEP WATER (the user's VR run 2026-11-03: Zora pulled into the Lakeview bath, 80 u deep, submerged
    // 0.69, cap 54 u -> "no route"). She may START deeper than her cap: she is already there. Her START REGION (the water
    // deeper than the cap joined to her spot, cell to cell) may be walked up to her own depth + kXDeepStartPad, strictly
    // under the engine's swim level - Actor::GetSubmergedLevel (36452) = (water - root z) / ((boundMax.z - boundMin.z) x
    // scale), swimming from 0.70 (KNOWLEDGEBASE / Bathing report 13 F §4: read from SkyrimVR.exe 1.4.15).
    constexpr float  kXSwimLevel     = 0.70f;
    constexpr float  kXDeepStartPad  = 8.f;      // the start region's limit = max(cap, her floor depth + 8 u) ...
    // ... and (review R #5) at most 6 u under the swim depth, measured as the engine does: from her ROOT, which sits under the
    // floor ray's hit (1-1.4 u in the 11-03 log) - that offset is subtracted too. Never under the water she already stands in
    // (she does not swim there), + kXOwnDepthTol for ray noise while that still leaves her under the swim level.
    constexpr float  kXSwimMargin    = 6.f;
    constexpr float  kXOwnDepthTol   = 0.5f;
    constexpr float  kXSameSurface   = 2.f;      // (review R #6) a goal's water is HER start's when its surface is within this
    constexpr float  kXMinNz         = 0.70f;    // a floor: the down ray's normal z >= 0.70 (<= 45.6 deg)
    constexpr float  kXStandNz       = 0.85f;    // a stand spot: >= 0.85 (<= 32 deg)
    constexpr float  kXHeadFoot      = 8.f;      // headroom: clear from the floor + 8 s ...
    constexpr float  kXHeadFrac      = 0.85f;    // ... to the floor + 0.85 H
    constexpr float  kXFloorDown     = 160.f;    // a cell's down ray: from the parent's floor + step + 16 to - max(160, 1.25 H)
    constexpr float  kXFloorDownH    = 1.25f;
    constexpr float  kXTopPad        = 16.f;
    constexpr float  kXGoalTopPad    = 48.f;     // the goal's down ray: from its z + step + 48 to its z - 160
    constexpr float  kXSample        = 16.f;     // a floor line test: a centre down ray every 16 u, side rays every 32 u
    constexpr float  kXMinDirect     = 24.f;     // direct points at least this far apart (engine legs: kMinLeg)
    constexpr float  kXPrefixMin     = 384.f;    // a navmesh prefix shorter than this buys nothing: all direct
    constexpr int    kXMaxPrefix     = 16;       // engine legs at most (the Papyrus waypoint store)
    constexpr int    kXMaxPoints     = 48;       // more points = "too many turns"
    constexpr float  kXMarginFrac    = 0.75f;    // search box margin = clamp(0.75 d, 320, 640): the way round a ledge may lie aside
    constexpr float  kXMarginMin     = 320.f;
    constexpr float  kXMarginMax     = 640.f;
    constexpr int    kXMaxSearchQ    = 8000;     // questions per plan: the start, the goal, the straight line and the search ...
    constexpr int    kXMaxFinishQ    = 4000;     // ... + the pull, the spacing and the corners
    constexpr double kXFrameUs       = 1000.0;   // + a time cap: all running plans together at most 1.0 ms of slices per frame
    constexpr float  kXCostFloor     = 1.25f;    // cost per unit: NAV dry 1.0 · FLOOR dry 1.25 · WATER 1.25 + 1.5 depth / H ...
    constexpr float  kXCostWater     = 1.25f;
    constexpr float  kXCostDepth     = 1.5f;
    constexpr float  kXCostEdge      = 0.5f;     // ... + 0.5 out of an edge cell · + 0.5 x |dz|
    constexpr float  kXCostDz        = 0.5f;
    // (review G, RISK 7) the corner test spans the driver's own pure-pursuit look-ahead: kXCornerChord MUST equal
    // DirectWalk.cpp kLookAhead (48). A vertex whose +-48 u arc turns more than 15 deg gets a chord test (SHARP on failure).
    constexpr float  kXCornerCos     = 0.9659258f;   // cos 15 deg
    constexpr float  kXCornerChord   = 48.f;
    constexpr float  kXMarchStep     = 16.f;     // GOAL_MAY_MOVE: from the goal toward her in 16 u steps ...
    constexpr float  kXMarchMax      = 320.f;    // ... up to 320 u
    constexpr float  kXBadPad        = 24.f;     // a bad spot refuses cells within r + 24 u (2D) and |dz| <= 64 ...
    constexpr float  kXBadDz         = 64.f;
    // (review G, BUG 3) ... and only a spot UNDER her (within kXBadUnderHer of the plan's start) gets B's start exemption
    // (r + 48 u round her start: she must be able to leave where she stands). A spot ahead of her (the driver marks a stall
    // 32 u ahead) is never exempted, so the one re-plan really goes round it.
    constexpr float  kXBadUnderHer   = 16.f;
    constexpr float  kXBadStartPad   = 48.f;
    constexpr int    kXPartialTries  = 4;        // (review G, BUG 4) a partial route's end steps back at most 4 cells to a stand spot
    constexpr float  kXBadLifeDef    = 60.f;     // seconds; the caller may ask 5-300
    constexpr float  kXBadLifeMin    = 5.f;
    constexpr float  kXBadLifeMax    = 300.f;
    constexpr std::size_t kXBadMax   = 32;       // per walker
    constexpr std::size_t kXMaxWaters = 16;      // placed waters kept per plan (the nearest to her line)
    constexpr float  kXWaterBoxPad   = 256.f;    // ... from the search box + 256 u
    constexpr int    kXMaxRefsLooked = 20000;    // the placed-water scan stops after this many references
    constexpr float  kXFootPad       = 4.f;      // in a placed water: its footprint + 4 u, the floor from 400 u under to 4 u over
    constexpr float  kXWaterUnder    = 400.f;
    constexpr float  kXWaterOver     = 4.f;
    constexpr float  kXInteriorDown  = 10.f;     // an interior cell plane counts from floor - 10 to floor + 200 (Water Finder)
    constexpr float  kXInteriorUp    = 200.f;
    constexpr float  kXPartialMin    = 96.f;     // a partial route must end >= max(96 u, 25 % of her distance) nearer the goal
    constexpr float  kXPartialFrac   = 0.25f;
    constexpr int    kXLedgeLines    = 4;        // LEDGE lines per plan (kVerbose: kXLedgeLinesV)
    constexpr int    kXLedgeLinesV   = 16;

    // ---- 1.5.0 (D §2; B §7 C1-C17) - used only by a plan with a 1.5.0 flag (x.v15) and by the probes ---------------------
    constexpr std::uint32_t kV15Flags = kNavGuide | kWadeSpot | kDoors | kActorsKeepOut;
    constexpr float  kVRiser         = 26.f;     // C1: one rise / drop between floor samples (x scale for s > 1, never < 24)
    constexpr float  kVRiserMin      = 24.f;
    constexpr float  kVSample        = 32.f / 3.f;   // C1/C3: floor samples <= ~11 u apart (two per 32 u step, at 1/3 and 2/3)
    constexpr float  kVClearPad      = 4.f;      // C5: rc = collisionBound.extents.x + 4 (every side ray)
    // C7 (review R1 #2, the coordinator's rule): Htop = the TOP of her own controller bound (collisionBound.center.z +
    // extents.z, read from her controller: 66.6 + 64.9 = 131.5 u in the 11-03 VR log) + 2 u - never a fixed 134. A top outside
    // 0.6-1.6 H is not trusted: then H (GetHeight) + 2.
    constexpr float  kVHeadPad       = 2.f;
    constexpr float  kVHeadLogFrac   = 0.6f;     // a headroom refusal with at least 0.6 Htop of clearance (a lintel, a beam) is logged ...
    constexpr int    kVHeadLogMax    = 8;        // ... once per 64 u spot, at most 8 lines a plan (the VR log then shows real door heights)
    constexpr float  kVColumnPad     = 4.f;      // C6: the side columns run from the floor + riser + 4 up to the floor + Htop
    constexpr float  kVEdgeDropPad   = 4.f;      // (review R1 #7) `edge` = a neighbour more than riser + 4 u lower (or no floor)
    constexpr std::array<float, 4> kVNudge{ 8.f, -8.f, 16.f, -16.f };   // C11: a step failing only beside her, retried aside
    constexpr int    kVMaxLevels     = 4;        // C9: floors per 32 u map square
    constexpr float  kVLevelTol      = 6.f;      // two floor hits at one x,y within this = the same surface (one level)
    constexpr int    kVExtraLevels   = 4096;     // the pool of extra levels a plan may add (beyond the first per square)
    constexpr float  kVMarginMin     = 448.f;    // C10 / item 4: margin max(448, 0.75 d) ...
    constexpr float  kVMarginMax     = 1024.f;   // ... at most 1024; the second pass 1.5 x wider (<= 1024)
    constexpr float  kVPass2Mul      = 1.5f;
    constexpr float  kVPass2Gain     = 16.f;     // a second pass only when its box really is wider
    constexpr int    kVQPerCell      = 10;       // the search budget per box cell ...
    constexpr int    kVSearchQMin    = 8000;     // ... never less than 1.4.1's ...
    constexpr int    kVSearchQMax    = 20000;    // ... never more than this (C10)
    constexpr int    kVFinishQ       = 6000;     // pull + spacing + corners (1.4.1: 4000; the 1.5.0 line tests ask ~1.5 x more)
    constexpr float  kVLevelBand     = 8.f;      // item 1: "the same floor" = |dz| <= one step + 8 u
    constexpr int    kVCasts         = 7;        // C12: a ray steps over at most 6 hits that are neither floor nor blocking
    constexpr float  kVActorPad      = 8.f;      // C13: a standing actor's disc = her extents.x + rc + 8
    constexpr float  kVWadeNbrFrac   = 0.10f;    // item 8 (b): a patch neighbour's depth lies in [0.10 H, the cap]
    constexpr float  kVCorrGoalBand  = 96.f;     // the corridor's goal on the navmesh: 1.3.0's own band and slide
    constexpr int    kVCorrMaxStraight = 8;      // a corridor leg is cut at least every 8 cells (256 u): repairs stay local
    constexpr int    kVDoorMemo      = 32;       // doors remembered per plan (FindCollidableRef once per door body)
    constexpr int    kVLocalShare    = 3;        // (review R1 #6) one MIX repair may use at most 1/3 of the MIX budget ...
    constexpr int    kVLocalMinQ     = 1500;     // ... but never less than this
    constexpr int    kVNgLegLog      = 12;       // legs named in the NAV-GUIDE line (then "...")
    constexpr std::int32_t kNbrUnset = -2;       // an expansion's neighbour level not looked up yet

    // ---- 1.5.1 (D_Standpoint_151_Build_Spec.md §4) - x.v15 plans and the probes only ------------------------------------
    constexpr float  kVConeFrac      = 0.9f;     // the TERRAIN riser = 0.9 x her hull's CONE (the 1.5.0 VR run: Sofia 38.0 u -> 34 u)
    constexpr float  kVRiserMax      = 40.f;     // ... clamped 24-40 u - only on navmesh terrain / ground (review fix #2: every other
                                                 // floor keeps 1.5.0's 26 u x scale, so a 30-31 u bench / chair seat is never a step)
    constexpr float  kVConeMin       = 8.f;      // a cone read outside 8-120 u is not trusted (then 1.5.0's 26 u x scale)
    constexpr float  kVConeMax       = 120.f;
    constexpr float  kVConeRingFlat  = 2.f;      // (review #9) ... nor one whose bottom ring of 8 is not flat (a non-humanoid hull)
    constexpr float  kVTerrainNz     = 0.45f;    // TERRAIN on the navmesh: slope n.z >= 0.45 (<= 63 deg) ...
    constexpr float  kVTerrainTan    = 1.9626105f;   // ... rise <= horizontal x tan 63 deg ...
    constexpr float  kVTerrainPad    = 8.f;      // ... + 8 u
    constexpr float  kVSideTerrNz    = 0.40f;    // (review #4) a ray BESIDE her steps over land only where its face is ground (|n.z| >= 0.40)
    constexpr float  kVMaxDistance   = kRangeV15;    // the direct planner's range (1.3.0 / old-flag plans: kMaxDistance 1600)
    constexpr float  kVCorrQPerU     = 3.f;      // the navmesh corridor's own cap: 3 questions per unit of distance ...
    constexpr int    kVCorrQMax      = 8000;     // ... never under 1.3.0's 5,000 (kMaxQueries), never over this
    constexpr long long kVCacheLifeMs = 60000;   // the cell cache: an answer is reused for 60 s ...
    constexpr std::size_t kVCacheSweepSome = 2048;   // ... expired ones are swept at most this many per table per call (each plan's
                                                     // start, and a full table at most once a slice): no sweep ever pauses a frame
    constexpr std::size_t kVCacheBucketWalk = 8192;  // (review #10) ... walking at most this many (empty) buckets per call
    constexpr std::size_t kVCacheSquares = 16384;    // memory bound: squares (down-ray answers), cells (their own tests),
    constexpr std::size_t kVCacheCells   = 16384;    // step tests - a table still full after its bounded sweep keeps nothing new
    constexpr std::size_t kVCacheSteps   = 32768;    // until answers expire (worst ~7 MB in all: ~3 + 1 + 3; an stpf session < 1 MB)
    constexpr int    kVCacheRecs     = 3;        // down-ray answers kept per world square (each: one surface or "nothing")
    constexpr int    kVCacheHitDiv   = 4;        // a reused answer counts 1/4 question against the search's own budget
    constexpr float  kVCacheSameZ    = 0.25f;    // two hits within this at one x,y = the same surface (its first z is kept)
    constexpr float  kVCacheEps      = 0.01f;

    // ---- 1.6.0 (D_Standpoint_160_Build_Spec.md §1 items 6-10) - x.v15 plans and the probes only --------------------------
    constexpr int    kVSliceSafetyQ  = 2000;     // §4 #1: a slice is bounded by TIME (its share of kXFrameUs); this is only a safety cap
    constexpr double kVNavSliceUs    = 250.0;    // ... all running NavGraph jobs TOGETHER keep 0.25 ms of the frame (NavGraph's
    constexpr double kVNavJobMinUs   = 50.0;     //     kJobFrameUs; each job >= 0.05 ms - its kJobMinUs; fix round, R2 #14) ...
    constexpr double kVMinSliceUs    = 200.0;    // ... and a plan's slice never gets less than this
    constexpr int    kVLocalCapQ     = 2000;     // §4 #8: one MIX repair's own cap (1.5.1: a third of the MIX budget, >= 1,500)
    constexpr float  kVStairTan      = 1.1383f;  // §5: tan(acos 0.66) - the stair allowance's rise over a horizontal distance
    constexpr float  kVStairPad      = 4.f;      // ... + 4 u: a stair cell's lip / brink floors beside her (24 u x 1.14 + 4 = 31 u)
    constexpr float  kVStairSideNz   = 0.55f;    // a ray BESIDE her on a stair step passes the stair's own sloped face (n.z >= 0.55
                                                 // UP - signed: a face looking down, the underside of a flight above, blocks; R2 #9)
    constexpr float  kVRingMin       = 16.f;     // §5.3 #3 the goal ring: radii 16-160 u in 16 u steps, 8 bearings, nearest first ...
    constexpr float  kVRingMax       = 160.f;
    constexpr float  kVRingStep      = 16.f;
    constexpr float  kVRingLevel     = 8.f;      // ... on the goal's own floor: |dz| <= riser + 8 ...
    constexpr float  kVRingSlope     = 0.5f;     // ... + 0.5 x the ring's radius on navmesh terrain (a hillside; fix round, R2 #4)
    constexpr float  kVCornerChordRun = 128.f;   // §3.3 the run's chord: +-128 u ...
    constexpr float  kVCornerRunCos  = 0.5f;     // ... for a corner turning 15-60 deg over it (cos 60)
    constexpr float  kVCornerRunOff  = 16.f;     // ... (or lying more than 16 u off that chord)
    constexpr float  kVSkelDupe      = 16.f;     // a skeleton point this close to the one before it is not a leg of its own
    constexpr int    kVWadeScanQ     = 4000;     // §6.3: one wade scan's cap (questions)
    constexpr int    kVFloodSquares  = 4000;     // §6.4: the floor flood: squares at most (1.6.1 A2: 2,500 -> 4,000) ...
    constexpr int    kVFloodQ        = 10000;    // ... and questions at most (1.6.1 A2: 6,000 -> 10,000 - the ledge pond hit 6,000 twice)
    constexpr float  kVFloodAim      = 1.0f;     // (1.6.1 A2) the flood is DIRECTED: its order = floor distance + 1.0 x the flat distance to the seed
    constexpr int    kVFloodBackBatch = 256;     // (1.6.1 A2) the way-back pass: squares per unit (no question: the flood's own steps)
    constexpr int    kVFloodOpp[8]   = { 1, 0, 3, 2, 7, 6, 5, 4 };   // kDir's opposite directions
    constexpr float  kXPi            = 3.14159265f;

    constexpr int kDir[8][2] = { { 1, 0 }, { -1, 0 }, { 0, 1 }, { 0, -1 }, { 1, 1 }, { 1, -1 }, { -1, 1 }, { -1, -1 } };

    struct Node {
        float         x{ 0.f }, y{ 0.f }, z{ 0.f };
        float         g{ 0.f };
        float         bestG{ 3.0e38f };
        std::int32_t  parent{ -1 };
        std::uint8_t  floor{ 0 };         // 0 unknown, 1 navmesh, 2 none (or inside the keep-out circle)
        std::uint8_t  rejectedFrom{ 0 };  // the directions whose step INTO this cell was found blocked
        bool          closed{ false };
    };

    struct OpenEntry {
        float        f;
        float        g;
        std::int32_t idx;
        std::int32_t parent;
        bool operator>(const OpenEntry& a_o) const { return f > a_o.f; }
    };

    // ---- 1.4.0 the Ex mode's memory -------------------------------------------------------------------------------------
    constexpr std::uint8_t kKUnknown = 0, kKNav = 1, kKFloor = 2, kKWater = 3, kKNone = 4;    // = Kind's values + 0 / 4

    // (1.5.0: kWhyActor - a standing actor; kWhyTight - a wall or a lip within rc of the cell, and no spot 8 / 16 u aside fits)
    enum XWhy : std::uint8_t { kWhyNone = 0, kWhyNoFloor, kWhySteep, kWhyBlocked, kWhyHeadroom, kWhyDeep, kWhyKeepOut,
                               kWhyBad, kWhyDryOnly, kWhyNoCell, kWhyActor, kWhyTight, kWhyCount };

    // (1.5.0: + kXNavCorr / kXNavLegs - the MIX's navmesh corridor and its legs; the value order is internal)
    // (1.6.0: + kXSkeleton - NavGraph's triangle path stepped here; kXRing - the goal's ring search; kXFlood - the wade flood)
    enum XPhase : int { kXMeasure = 0, kXWaters, kXStart, kXGoal, kXMarch, kXGate, kXLegacy, kXStraight, kXGrid, kXSearch,
                        kXPartialEnd, kXSplit, kXPull, kXSpace, kXCorners, kXAssemble, kXNavCorr, kXNavLegs, kXSkeleton, kXRing, kXFlood,
                        kXDone };

    struct XNode {
        float         x{ 0.f }, y{ 0.f }, z{ 0.f };
        float         g{ 0.f };
        float         bestG{ 3.0e38f };
        float         depth{ 0.f };       // water over the floor (0 = dry)
        float         zRef{ 0.f };        // the level it was classified from (its first parent's floor)
        std::int32_t  parent{ -1 };
        std::uint8_t  kind{ kKUnknown };
        std::uint8_t  why{ kWhyNone };    // a NONE cell's reason
        std::uint8_t  rejectedFrom{ 0 };  // the directions whose step INTO this cell was found blocked
        std::uint8_t  head{ 0 };          // headroom: 0 not asked, 1 clear, 2 blocked
        bool          onNav{ false };
        bool          edge{ false };      // a neighbour lies more than a step lower, or has no floor (a brink)
        bool          closed{ false };
        bool          hasZ{ false };      // a floor z was found (also for a NONE cell: too deep, too steep)
        bool          deepOk{ false };    // 1.4.1: a cell of her START REGION (deeper than the cap, joined to her deep start)
        // 1.5.0 (several floors per map square): a node is ONE level of its square; the square's first level is the node at
        // the square's own index, further levels hang off it through `next`
        std::int32_t  col{ -1 };          // its map square (nx * iy + ix)
        std::int32_t  next{ -1 };         // the next level of the same square (-1 = none)
        RE::FormID    door{ 0 };          // the closed door the accepted step INTO this cell passes (kDoors)
        float         nudge{ 0.f };       // C11: how far it was moved aside (across its step) so her body fits (0 = not)
        bool          pushed{ false };    // it was ever put on the open list (a never-pushed dead level may be re-used)
        std::uint8_t  lip{ 0 };           // (review R1 #4) the floor at +-rc round it: 0 not asked, 1 fine, 2 a brink / a rise beside it
        bool          navTerr{ false };   // 1.5.1: its floor is terrain / ground (13 / 17) AND on the navmesh (the terrain allowance)
        bool          navStair{ false };  // 1.6.0: its floor is a fixed floor on the navmesh steeper than 0.70 (the stair allowance)
        bool          frontier{ false };  // 1.6.0: expanded with a neighbour outside the pass-1 box (pass 2 re-opens it)
    };

    struct PCell {                        // a point of the cell path / of the route being finished
        RE::NiPoint3 pos{};
        std::uint8_t kind{ kKNav };
        float        depth{ 0.f };
        bool         onNav{ false };
        bool         edge{ false };
        bool         goal{ false };
        bool         deep{ false };       // 1.4.1: in her start region (a line from here may use its water until it leaves it)
        RE::FormID   door{ 0 };           // 1.5.0: the closed door the leg INTO this point passes (kPtDoor)
        bool         wade{ false };       // 1.5.0: the wading spot a kWadeSpot plan chose (kPtWade)
        bool         navTerr{ false };    // 1.5.1: terrain / ground under it AND the navmesh there (the terrain allowance)
        bool         stair{ false };      // 1.6.0: the leg INTO it uses the stair allowance (kPtStair); a corridor point: on a stair
        bool         chunkEnd{ false };   // 1.6.0: the end of a chunk that is not the goal (kPtChunkEnd)
        bool         virt{ false };       // 1.6.0: the leg INTO it is a skeleton's virtual link (kPtVirtual once it passed)
    };

    // A placed water activator's footprint: OBND x/y x scale, turned by angle.z; surface = position.z + boundMax.z x scale
    // (Water Finder 1.5.0's math, the user's public-domain code, re-implemented here - no link to its DLL).
    struct WaterFP {
        float cx{ 0.f }, cy{ 0.f }, surf{ 0.f }, hx{ 8.f }, hy{ 8.f }, c{ 1.f }, s{ 0.f };
        bool Contains(float a_x, float a_y, float a_pad) const
        {
            const float wx = a_x - cx, wy = a_y - cy;
            const float lx = c * wx - s * wy, ly = s * wx + c * wy;
            return std::fabs(lx) <= hx + a_pad && std::fabs(ly) <= hy + a_pad;
        }
    };

    struct BadSpot {
        float                                 x{ 0.f }, y{ 0.f }, z{ 0.f };
        std::chrono::steady_clock::time_point until{};
    };

    // One straight-line test, resumable across slices (a long line never spikes a frame).
    struct SegTest {
        bool         active{ false };
        bool         nav{ false };        // 1.3.0's navmesh-leg rules (+ dry) instead of the floor rules
        bool         sides{ true };       // floor rules: the 32 u side rays (off for the re-test of one cell step, B §5.2)
        RE::NiPoint3 a{}, b{};
        float        len{ 0.f }, px{ 0.f }, py{ 0.f };
        int          n{ 0 }, k{ 0 };      // samples (navmesh: pieces), the next one
        RE::NiPoint3 last{};              // the last sample found on the floor
        RE::NiPoint3 pieceA{};            // the body test's current piece starts here
        float        pieceLen{ 0.f };
        float        depthLimit{ 0.f };
        bool         deepRun{ false };    // 1.4.1: the line starts in her start region and has not left it yet (limit = startLimit)
        int          result{ 0 };         // 0 running, 1 clear, -1 blocked
        const char*  why{ "" };
        float        failAt{ 0.f };       // how far along the line it failed
        // 1.5.0 (x.v15, floor rules: SegRunV)
        bool          v15{ false };
        bool          doorsOk{ false };   // kDoors: a closed unlocked ordinary door on the line is passed (one door per line)
        RE::FormID    door{ 0 };          // ... the door it passed
        bool          capture{ false };   // the first blocking hit is resolved and kept (a navmesh leg's CARPET line)
        bool          capHave{ false };
        std::uint32_t capLayer{ 0 };
        RE::FormID    capRef{ 0 };
        RE::NiPoint3  capAt{};
        RE::NiPoint3  failPos{};          // where it failed
        // (review R1 #1) the side columns of the current piece wait until its CENTRE rays ran: a door is passed only where her
        // centre line goes through it, never because a column beside her grazed its leaf
        std::array<RE::NiPoint3, 4> colQ{};
        int           colN{ 0 };
        bool          flushOwed{ false };  // (verifier N2) a piece's centre rays passed; its queued columns are the next unit
        RE::NiPoint3  flushAt{};           // ... the sample where that piece ended
        std::uint32_t lastLayer{ 0 };      // 1.5.1: the layer of the floor the last sample found (0 = none yet: the line's start)
        bool          pieceTerr{ true };   // 1.5.1: every sample of the current piece stood on terrain / ground ...
        bool          pieceNav{ false };   // ... and the navmesh was found under one of them: its rays beside her pass the hillside
        bool          flushTerr{ false };  // ... the same for the piece whose columns are owed
        bool          stairUsed{ false };  // 1.6.0: a sample of the line used the stair allowance (the point it reaches: kPtStair)
        bool          pieceStair{ false }; // 1.6.0: ... a sample of the current piece did: its rays beside her pass the stair's face
        bool          flushStair{ false }; // ... the same for the piece whose columns are owed
    };

    struct ExStats {                      // kept for the log after the search memory is freed
        int         rF{ 0 }, rO{ 0 }, rM{ 0 }, rR{ 0 }, rS{ 0 }, rC{ 0 }, nW{ 0 };   // rays: floor, body, mid, headroom, side, classify; water questions
        int         wet{ 0 };             // water answers deeper than kXWet
        int         waterSeen{ 0 }, waterSeenNoOurs{ 0 };
        float       waterDeltaMax{ 0.f };
        int         normBad{ 0 };
        int         ledges{ 0 };
        std::string straight;             // why the straight line failed
        std::string g1;                   // the navmesh-first try
        // 1.5.0
        int         rK{ 0 }, rN{ 0 }, rG{ 0 };    // rays: side columns; nudge retries; the navmesh corridor's step rays
        int         nudgeTries{ 0 }, nudged{ 0 };
        int         levelsExtra{ 0 }, levelFull{ 0 };
        int         doorsPassed{ 0 };
        int         actorsKept{ 0 }, actorsMoving{ 0 };
        int         passes{ 0 };          // grid passes run (1 or 2; local repairs not counted)
        int         headFails{ 0 };       // (review fixes) headroom refusals, lip refusals, nudges refused at the jog / the walked line
        int         lipFails{ 0 };
        int         jogFails{ 0 };
        std::string pass2;                // why the second, wider pass ran
        std::string mix;                  // the NAV-GUIDE summary for the PATH line
        std::string wade;                 // the WADE summary for the PATH line
        // 1.5.1: the terrain allowance used (cells whose slope only it allowed; edge / goal steps; line samples and brinks) and
        // the cell cache (floor look-ups answered of those asked, cells' own tests, step tests reused; answers stored)
        int         terrCells{ 0 }, terrSteps{ 0 }, terrLines{ 0 };
        int         cacheFloorAsks{ 0 }, cacheFloorHits{ 0 }, cacheCellHits{ 0 }, cacheStepHits{ 0 }, cacheStores{ 0 };
        int         CacheHits() const { return cacheFloorHits + cacheCellHits + cacheStepHits; }
        // 1.6.0: the stair allowance used (cells, steps, line samples); the goal's ring search; the run chords; deferred side-steps;
        // the skeleton; the wade stages and the flood
        int         stairCells{ 0 }, stairSteps{ 0 }, stairLines{ 0 };
        int         ringTried{ 0 }, ringRefused{ 0 }, ringOtherFloor{ 0 };
        int         runChords{ 0 }, noCut{ 0 };
        int         deferQueued{ 0 }, deferRun{ 0 }, deferSkipped{ 0 };
        int         pass2Continued{ 0 }, frontierReopened{ 0 };
        int         pass2Same{ 0 };       // (fix round, R2 #5) pass 2 continued pass 1 in the SAME box with a fresh budget
        int         ringMarch{ 0 };       // (fix round, R2 #4) the ring found nothing: the 1.5.1 march ran after it
        int         ringTight{ 0 };       // (1.6.1 A4) ring spots refused as too tight for her body (a table edge, a chair, a wall)
        int         nudgeFlips{ 0 };      // (fix round, R2 #13) side-steps that tried the other side once
        std::string skeleton;             // the SKELETON summary for the PATH line
        std::string flood;                // the flood's summary
        std::string scans;                // the wade scans' summary
        std::string chunk;                // the chunk's summary
    };

    struct ExState {
        int          xphase{ kXMeasure };
        bool         yield{ false };      // end this slice now (a heavy one-off step had it to itself)
        // the walker (measured in the first slice, on the main thread)
        float        H{ 128.f }, Hraw{ 0.f }, step{ 38.f }, depthCap{ 57.6f };
        float        swimDiv{ 128.f }, swimDepth{ 89.6f };   // 1.4.1: the engine's submerge divisor; 0.70 x it = she swims
        bool         deepStart{ false };  // 1.4.1: she starts in water deeper than the cap (her start region exists) ...
        float        startLimit{ 0.f };   // ... and may walk it up to this depth (XPhaseStart: < swimDepth, >= her own depth)
        int          startWater{ -1 };    // (review R #6) the placed water she starts in (an index into waters; -1 = none) ...
        float        startSurf{ -1.0e9f };// ... and its surface: only a goal in THIS water may use startLimit
        bool         interior{ false };
        bool         placedInCell{ false };    // an interior with placed water: its cell plane is never trusted
        std::vector<WaterFP> waters;
        std::vector<BadSpot> bad;
        // per slice
        int          qEnd{ 0 };
        std::chrono::steady_clock::time_point deadline{};
        int          qBase{ 0 };          // questions when the hybrid search began (its own budget after a navmesh-first try)
        int          qFinish{ 0 };        // ... when the finishing phases began
        // start and goal
        PCell        start{}, goal{};
        RE::NiPoint3 asked{};             // the goal as asked
        std::string  goalWhy;
        int          marchK{ 0 };
        RE::NiPoint3 marchPrev{};
        float        marchRefZ{ 0.f };
        bool         marchBody{ false };
        float        marchSurf{ -1.0e9f };    // the asked goal's water surface (the march stays at the water's level)
        bool         g1{ false };
        // the grid search
        float        minX{ 0.f }, minY{ 0.f };
        int          nx{ 0 }, ny{ 0 };
        std::vector<XNode> nodes;
        std::priority_queue<OpenEntry, std::vector<OpenEntry>, std::greater<OpenEntry>> open;
        std::int32_t best{ -1 };          // the closed node (not an edge cell) nearest the goal (a partial route ends there)
        float        bestD{ 3.0e38f };
        bool         partial{ false };
        std::string  noWayWhy;
        std::string  partialWhy;
        int          partialTries{ 0 };
        std::int32_t expIdx{ -1 };        // the cell being expanded (an expansion is resumable, neighbour by neighbour)
        int          expD{ 0 };           // its next neighbour
        int          expProbe{ -1 };      // a LEDGE probe owed toward this neighbour (its own unit of work)
        bool         pullOver{ false };   // the pull ran out of budget: the rest is the tested cell path as it is
        int          rawLegs{ 0 };        // cell steps taken unpulled (each re-tested with the floor rules)
        // finishing
        std::vector<PCell> cells;         // the cell path: start ... end (the goal last when it was reached)
        std::size_t  sw{ 0 };             // the split: cells[0 .. sw-1] = the navmesh prefix (0 = all direct)
        bool         swChosen{ false };
        int          swFails{ 0 };
        std::vector<PCell> pulled;        // after the pull (no start; the end last)
        std::size_t  prefixPulled{ 0 };   // the first ones belong to the prefix (S = the last of them)
        int          pullSeg{ 0 };        // 0 the prefix, 1 the suffix
        std::size_t  pFrom{ 0 }, pEnd{ 0 }, pLo{ 0 }, pHi{ 0 }, pCand{ 0 };
        int          pStage{ 0 };         // 0 choose, 1 testing the farthest, 2 bisecting, 3 testing a bisection
        std::vector<PCell> spaced;
        std::size_t  prefixSpaced{ 0 };
        std::size_t  spaceK{ 0 };
        PCell        spacePrev{};
        std::vector<std::uint8_t> sharp;
        std::size_t  cornerK{ 0 };
        SegTest      seg{};
        std::size_t  pLoDoorK{ 0 };       // 1.5.0: the bisection's best passed candidate and the door its line passed
        RE::FormID   pLoDoor{ 0 };
        bool         pLoStair{ false };   // 1.6.0: ... and whether that line used the stair allowance

        // ---- 1.5.0 (x.v15: a plan with a 1.5.0 flag, or a probe) -------------------------------------------------------
        bool         v15{ false };
        float        riser{ kVRiser };    // C1 (x.step = riser too: every "one step" rule)
        float        rc{ 24.f };          // C5 clearance
        float        htop{ 134.f };       // C7 headroom
        bool         htopFromBound{ false };
        // the test running now (rays read these): doors may be passed / the door passed / the first blocking hit kept
        bool          doorsOk{ false };
        bool          doorSide{ false };  // (review R1 #1) the ray now cast is BESIDE her: it passes only the door her centre line passed
        bool          doorPeek{ false };  // a cell's clearance probe: sees through a passable door, records nothing
        RE::FormID    curDoor{ 0 };
        int           headLogged{ 0 };    // (review R1 #2) headroom refusals logged in this plan, and their 64 u spots
        std::vector<std::uint64_t> headSpots;
        int           localCap{ 0 };      // (review R1 #6) the current MIX repair's own budget
        bool          capOn{ false };
        bool          capHave{ false };
        std::uint32_t capLayer{ 0 };
        RE::FormID    capRef{ 0 };
        RE::NiPoint3  capAt{};
        struct DoorMemo {
            const void* col{ nullptr };   // the door body's collidable (compared, never read)
            RE::FormID  id{ 0 };
            bool        pass{ false };    // closed, unlocked, not a load door
        };
        std::vector<DoorMemo> doorMemo;
        struct Disc {                     // C13: a standing actor to keep out of
            float      x{ 0.f }, y{ 0.f }, z{ 0.f }, r{ 0.f };
            RE::FormID id{ 0 };
        };
        std::vector<Disc> discs;
        // the grid passes (item 4)
        int          pass{ 1 };
        float        margin{ 0.f };
        int          searchQ{ kXMaxSearchQ };
        int          edgeCells{ 0 };      // expanded cells whose neighbour lay outside the box (a wider pass can only help then)
        std::array<std::int32_t, 8> expNbr{};    // the level reached in each direction from the cell being expanded
        // C11: a step being retried aside - (verifier N2) each offset in three units of work: 0 the moved end's own tests, 1 the
        // line she walks, 2 the sideways jog (each <= ~17 rays, the slice's share checked between them)
        bool         nudgeOn{ false };
        int          nudgeK{ 0 };
        OpenEntry    nudgeE{};
        int          nudgeD{ -1 };
        int          nudgeStage{ 0 };
        float        nudgeOff{ 0.f };
        RE::NiPoint3 nudgePs{}, nudgeMs{};
        float        nudgeDepth{ 0.f };
        RE::FormID   nudgeDoor{ 0 };
        // (verifier N1) the MIX corridor points' own clearance: the next spot aside to try for the point being judged (0 = none
        // yet), and how many points were kept / moved aside / dropped
        int          ngSpot{ 0 };
        int          ngMoved{ 0 }, ngDropped{ 0 };
        // (review fixes) a CELL whose own clearance failed, moved once to one of 8 spots aside, one spot per unit of work
        bool         cellNudgeOn{ false };
        int          cellK{ 0 };
        OpenEntry    cellE{};
        // kWadeSpot (item 8)
        bool         wade{ false };
        float        wadeMin{ 0.f };      // (a) the least depth: wadeMinFrac x H (>= kXWet)
        float        wadeNbrMin{ 0.f };   // (b) a patch neighbour's least depth: 0.10 H
        int          wadePatchMin{ 5 };
        std::int32_t wadePend{ -1 };      // (c) a cell whose stand-spot test is owed (its own unit of work)
        int          wadePendPatch{ 0 };
        int          wadeTested{ 0 }, wadeShallow{ 0 }, wadePatchFail{ 0 }, wadeStandFail{ 0 };
        std::string  wadeLastWhy;
        // kNavGuide (item 5, the MIX)
        int          ngStage{ 0 };        // the corridor: 0 set-up, 1 the navmesh A*, 2 its points' floors
        int          qNav0{ 0 };
        int          ngBudget{ 0 };
        std::vector<PCell> ngPts;         // the corridor's points (her start not included; the goal last)
        std::size_t  ngK{ 0 };            // the next point (floors) / the next leg
        std::vector<PCell> ngOut;         // the route so far (her start not included)
        bool         local{ false };      // a local repair search runs: x.start / x.goal = that leg's ends
        bool         localDone{ false };
        PCell        realStart{}, realGoal{};
        int          ngNav{ 0 }, ngRay{ 0 }, ngLegs{ 0 };
        int          ngLocalQ0{ 0 };
        std::string  ngLog;
        bool         ngDone{ false };     // the cell path came from the MIX (all direct)
        int          corrCells{ 0 }, corrQ{ 0 };

        // ---- 1.5.1 (x.v15) -------------------------------------------------------------------------------------------
        float        riserCone{ 0.f };    // her hull cone the TERRAIN riser came from (0 = not read: the terrain riser = the riser)
        float        riserT{ kVRiser };   // (review fix #2) the TERRAIN riser: navmesh terrain / ground only; x.riser everywhere else
        bool         nudgeTerr{ false };  // (review #5) the moved end of a step nudge stands on land (it keeps navTerr only then)
        bool         asideTerr{ false };  // (review #5) the spot XSpotAside found stands on land
        bool         watersComplete{ false };   // (review #6) the placed-water scan was complete (not capped, <= 16): depths may be kept
        int          corrCap{ 0 };        // the navmesh corridor's own question cap (grows with the distance)
        bool         terrUsed{ false };   // the step being tested needed the terrain allowance
        bool         terrSide{ false };   // a terrain step on the navmesh is tested: a ray BESIDE her that meets terrain / ground
                                          // meets the hillside she walks on, not a wall - it is stepped over (centre rays still block)
        // the cell cache: this plan uses it (a 1.5.0-flag plan; never a probe), the physics world it belongs to, the slice's clock
        bool          cacheOn{ false };
        std::uint64_t cacheWorld{ 0 };
        RE::FormID    cacheSpace{ 0 };
        std::uint64_t cacheClass{ 0 };    // her size class (clearance / headroom / both risers / scale buckets) for cells and steps
        std::chrono::steady_clock::time_point cacheNow{};
        int          cacheDirty{ 0 };     // + every ray hit that is not a fixed world layer (a door, an actor, clutter, a bad answer)
        bool         planFail{ false };   // the step test failed on THIS plan's own rules (a bad spot, the water limit, no cell)
        std::array<float, 2> profZ{}, profD{};   // the step test's two floor samples (z, water depth) - a reused step re-checks them
        int          hitBase{ 0 };        // reused answers when the current search pass began (its budget)

        // ---- 1.6.0 (x.v15) -------------------------------------------------------------------------------------------
        float        standNzTerr{ kStandNzTerrain };   // a stand spot's least n.z on navmesh-backed terrain (a probe may ask kStandNz)
        float        standSeenNz{ 1.f };               // the last stand-spot test's floor slope ...
        bool         standSeenTerr{ false };           // ... and whether it was terrain with the navmesh under it
        // (fix round, R2 #3) the last stand-spot test refused its floor for the SLOPE only (StandSpotV); the last XGoalSpot's
        // refusal was that (spotSteep) on a floor of this layer (spotLayer)
        bool          standSteep{ false };
        bool          spotSteep{ false };
        std::uint32_t spotLayer{ 0 };
        // the goal ring's reference height (the goal's floor, or the ASKED height for a stair / static ramp refused for its slope)
        float         ringRefZ{ 0.f };
        bool          ringAsked{ false };
        bool          nudgeFlipped{ false };          // (R2 #13) this side-step tried the other side already
        float         seedSurf{ -1.0e9f };            // (R2 #10) a wade seed's water surface (-1e9 = none known)
        // (1.6.1 A3) ProbeStandSpot's options and its side note: the reference the side floor rays step through (ignoreOn while they
        // run), the hits they stepped through, refs asked of floor hits (wantRef: a probe only); the last stand-spot test's side refusal
        RE::FormID    ignoreRef{ 0 };
        bool          ignoreOn{ false };
        bool          wantRef{ false };
        bool          probeNotes{ false };
        int           ignoredHits{ 0 };
        int           standSide{ -1 }, standSideWhat{ 0 };
        float         standCentreZ{ 0.f }, standSideZ{ 0.f }, standSideDepth{ 0.f };
        std::uint32_t standSideLayer{ 0 };
        RE::FormID    standSideRef{ 0 };
        std::string   fitWhy;                         // (1.6.1 A4) why a ring spot was too tight for her
        // §5 the stair allowance: the step / line being tested used it; a stair step is tested - a ray BESIDE her meeting the
        // stair's own sloped face steps over it (centre rays and headroom still block)
        bool         stairUsed{ false };
        bool         stairSide{ false };
        bool         asideStair{ false };   // the spot XSpotAside found stands on the stair (it keeps navStair only then)
        bool         nudgeStair{ false };   // the moved end of a step nudge stands on the stair
        // the side that blocked a step "beside her" (+1 her left, -1 her right, 0 not known): §4 #7 its side-steps go the other way
        int          besideSide{ 0 };
        int          nudgeSide{ 0 };
        // §4 #7 DEFERRED side-steps: an entry whose step failed only beside her (3) or whose cell's own clearance failed (4) waits
        // until it is the open list's best - a cheaper way round found first never pays for them
        struct Deferred {
            OpenEntry    e{};
            int          d{ -1 };
            std::uint8_t kind{ 3 };
            std::int8_t  side{ 0 };
            bool operator>(const Deferred& a_o) const { return e.f > a_o.e.f; }
        };
        std::priority_queue<Deferred, std::vector<Deferred>, std::greater<Deferred>> deferred;
        // §4 #5 the CONTINUED pass: the grid is made at the wider margin; pass 1 searches its box [ax0..ax1] x [ay0..ay1]
        int          ax0{ 0 }, ay0{ 0 }, ax1{ 0 }, ay1{ 0 };
        bool         canGrow{ false };    // the grid is wider than pass 1's box: a pass 2 continues it
        float        margin2{ 0.f };
        std::vector<std::int32_t> frontier;   // pass 1's cells with a neighbour outside its box (pass 2 expands them again)
        std::size_t  frontierK{ 0 };
        bool         reexp{ false };      // the cell being expanded is such a frontier cell (its goal test was done)
        // §2 the SKELETON (MIX part 0) and CHUNK MODE
        standpoint_nav::SearchPtr               skel;     // NavGraph's search, stepped inside this plan's slices
        std::vector<standpoint_nav::SkelPoint>  skelIn;   // the caller's skeleton - this plan's part of it (a chunk: to its end)
        bool         chunk{ false };      // CHUNK MODE ...
        bool         chunkFinal{ false }; // ... and its chunk reaches the goal
        int          skelTotal{ 0 };      // the caller's skeleton's points in all
        std::string  corrNote;            // where the corridor came from (the NAV-GUIDE line)
        int          cornerStage{ 0 };    // §3.3 the corner being tested: 0 its +-48 u chord, 1 its +-128 u chord
        std::string  ringLastWhy;         // §5.3 #3 the goal ring's last refusal
        // §6.3 the WADE SCANS: a scan runs (h = 0, distance only, cells within scanR of its root)
        bool         wadeScan{ false };
        int          wadeStage{ 0 };      // 0 the entry search, 1 scan 1 (round the entry E), 2 scan 2 (round the knee spot K)
        std::int32_t scanRoot{ -1 };
        float        scanRX{ 0.f }, scanRY{ 0.f }, scanR{ 0.f };
        int          scanQ0{ 0 };
        std::int32_t scanKnee{ -1 };      // scan 1's nearest knee spot (its stand spot passed), its depth and patch
        float        scanKneeDepth{ 0.f };
        int          scanKneePatch{ 0 };
        float        wadeKnee{ 0.f }, wadeThigh{ 0.f };   // the bands (u)
        std::vector<PCell> wadePrefix;    // start -> E
        std::vector<PCell> wadeChain1;    // E -> K
        float        wadeEntryDepth{ 0.f };
        int          wadeEntryPatch{ 0 };
        float        wadeKneeDepthUsed{ 0.f };
        // §6.4 the FLOOR FLOOD (a wade plan, her start dry, the seed lower than her floor): a cheap spread from her to the nearest
        // reachable shin-deep square; the search then aims there
        bool         floodWanted{ false };
        std::vector<float>        floodZ, floodG;
        std::vector<std::uint8_t> floodSt;    // 0 unseen, 1 open, 2 done, 3 refused
        std::priority_queue<std::pair<float, std::int32_t>, std::vector<std::pair<float, std::int32_t>>, std::greater<>> floodOpen;
        int          floodQ0{ 0 }, floodSquares{ 0 };
        std::int32_t floodCur{ -1 };      // the square being spread from, and its next direction (one neighbour per unit)
        int          floodDir{ 0 };
        // (1.6.1 A2) the flood keeps its own steps (both ends: the 8 direction bits that passed its tests); once it finds the water,
        // the WAY BACK pass (no question) gives every square it reached its floor distance to that water along those steps - the
        // search's heuristic then (the way round a ledge, not across it); floodZ stays with it (the level the flood stood on)
        std::vector<std::uint8_t> floodEdge;
        std::vector<float>        floodToE;
        std::int32_t              floodFound{ -1 };
        bool                      floodBack{ false };
        int                       floodBackSquares{ 0 };
        std::int32_t startIdx{ -1 };      // her square in the grid (XGridV)
    };

    struct Job {
        std::int32_t  handle{ 0 };
        RE::FormID    walker{ 0 };
        RE::NiPoint3  start{}, goal{};
        float         radius{ 20.f }, scale{ 1.f };
        bool          avoidOn{ false };      // a keep-out circle: the walk's own target
        float         avoidX{ 0.f }, avoidY{ 0.f }, avoidR{ 0.f };
        std::uint32_t rayFilter{ 30u };
        int           status{ kRunning };
        int           phase{ 0 };            // 0 the straight line, 1 the search, 2 the pull, 3 the spacing, 4 done
        bool          taskQueued{ false };
        float         minX{ 0.f }, minY{ 0.f };
        int           nx{ 0 }, ny{ 0 };
        std::vector<Node> nodes;
        std::priority_queue<OpenEntry, std::vector<OpenEntry>, std::greater<OpenEntry>> open;
        std::vector<RE::NiPoint3> cells;     // the found cell path: start ... goal
        std::size_t   pullFrom{ 0 }, pullTo{ 1 };
        std::vector<RE::NiPoint3> pulled;    // after the pull (goal last)
        std::size_t   spaceK{ 0 };           // phase 3 (resumable): the next pulled point, and the last one kept
        RE::NiPoint3  spacePrev{};
        std::vector<RE::NiPoint3> out;       // the waypoints handed out (goal last)
        int           queries{ 0 }, rays{ 0 }, snaps{ 0 }, slices{ 0 }, expansions{ 0 }, rejected{ 0 }, pushed{ 0 };
        double        usTotal{ 0.0 }, usMax{ 0.0 };
        std::string   why;
        std::uint64_t order{ 0 };
        // ---- 1.4.0 (StartEx only; a 1.3.0 Start plan leaves all of it as it is here) ----
        bool          ex{ false };
        bool          legacyOnly{ false };   // StartEx without kFloor and kWater: the 1.3.0 search, its answer as route points
        std::uint32_t xflags{ 0 };
        float         depthFrac{ kXDepthFracDef };
        std::unique_ptr<ExState> x;          // the Ex search's memory - freed when the plan ends
        ExStats       xs;
        std::vector<RoutePoint> route;       // the Ex answer (kept until Cancel / ClearAll)
        int           switchIndex{ 0 };
        float         goalMovedBy{ 0.f }, depthCap{ 0.f }, height{ 0.f }, routeLen{ 0.f };
        bool          swimStart{ false };    // 1.4.1: status 0 because she stands at the swim level / swims (PlanInfo::atSwimLevel)
        // ---- 1.5.0 ----
        RE::FormID    avoidId{ 0 };          // the walk's own target (never a keep-out disc of its own: it has the circle)
        float         halfZ{ 0.f };          // her collisionBound.extents.z (logged)
        float         boundTop{ 0.f };       // her collisionBound.center.z + extents.z (C7: Htop = it + 2; 0 = not read)
        float         wadeMinFrac{ 0.15f };  // PlanOptions (kWadeSpot)
        int           wadePatchMin{ 5 };
        bool          wadeNone{ false };     // PlanInfo: no reachable wading spot
        float         wadeDepth{ 0.f };
        int           wadePatch{ 0 };
        // ---- 1.5.1 ----
        float         cone{ 0.f };           // her hull cone (u), read on the main thread before MeasureV (0 = not read)
        bool          coneRemembered{ false };   // (review #9) a failed read took her last good cone
        float         riserUsed{ 0.f };      // PlanInfo::riser (0 = an old-flag plan)
        float         riserTUsed{ 0.f };     // PlanInfo::riserTerrain
        // ---- 1.6.0 (PlanOptions / PlanInfo) ----
        std::uint32_t tag{ 0 };              // the caller's id (WALK / STEER #n)
        bool          outsideLoaded{ false };
        bool          chunked{ false }, chunkFinal{ false };
        int           skelTaken{ 0 };
        float         remainAfterU{ 0.f };
        int           skelSource{ 0 };       // 0 none, 1 given, 2 built here, 3 the 1.5.x grid corridor
        float         skelLength{ 0.f };
        int           noCutPoints{ 0 };
        int           wadeStage{ 0 };
        float         wadeEntryDepth{ 0.f };
        float         wadeKneeFrac{ kWadeKneeFrac }, wadeThighFrac{ kWadeThighFrac }, wadeScanBodies{ 2.f };
        bool          wadeFlood{ true };
    };

    std::mutex                                             g_mtx;    // jobs: natives + the main-thread slices + the pump threads
    std::unordered_map<std::int32_t, std::unique_ptr<Job>> g_jobs;
    std::int32_t                                           g_nextHandle = 1;
    std::uint64_t                                          g_order = 0;
    std::unordered_map<RE::FormID, std::vector<BadSpot>>   g_bad;    // 1.4.0: per walker, the spots she got stuck at (g_mtx)
    // 1.5.0 once per game run (g_mtx; never cleared by a load - "per game run"): the walkers whose CALIBRATION line was
    // written, and the CARPET lines written (by the blocking reference, or by the 64 u spot when nothing was resolved)
    std::unordered_set<RE::FormID>                         g_calibrated;
    std::unordered_set<std::uint64_t>                      g_carpetSeen;
    // 1.5.1 (g_mtx; ClearAll): each walker's last good hull cone - a read that fails once (a controller being rebuilt) takes it,
    // so a plan and the driver's probes of the same walk never run on two different risers
    std::unordered_map<RE::FormID, float>                  g_cone;

    // ---- 1.5.1 THE CELL CACHE (spec §4; g_mtx - only the planner's main-thread slices touch it, never a probe) -------------
    // Physical answers only, each kept 60 s: what a DOWN RAY of the search found at a world square (32 u lattice point) - a
    // floor or a blocker at z with nothing between `top` and z, or nothing at all from `top` down to `bottom` - plus the
    // navmesh and the water over that floor; a cell's own tests (headroom, clearance) and a step's tests, per walker size
    // class. Stored only when every hit of the rays behind it was a fixed world layer (static, terrain, ground, trees, water,
    // invisible wall, collision box ...): a door, an actor, clutter, an animated body or a bad answer is never kept.
    using CacheClock = std::chrono::steady_clock;
    struct SurfRec {
        float        z{ 0.f }, nz{ 1.f };
        float        top{ 0.f }, bottom{ 0.f };   // a surface: nothing from `top` down to z; nothing: nothing from `top` to `bottom`
        float        navZ{ 0.f }, depth{ 0.f }, surf{ 0.f };
        CacheClock::time_point t{};
        std::uint8_t layer{ 0 };
        std::uint8_t what{ 0 };                   // 0 free, 1 a floor, 2 a blocker (a tree / an invisible wall), 3 nothing
        std::uint8_t nav{ 0 };                    // 0 not asked, 1 no navmesh within 19 u / 48 u, 2 the navmesh at navZ
        std::uint8_t dep{ 0 };                    // 0 not asked, 2 depth / surf known (an unloaded cell is never kept)
    };
    struct SquareRec { std::array<SurfRec, kVCacheRecs> r{}; };
    struct CellKey {
        std::int32_t  ix{ 0 }, iy{ 0 }, zq{ 0 };
        std::uint64_t cls{ 0 };
        bool operator==(const CellKey&) const = default;
    };
    struct CellRec { std::uint8_t head{ 0 }, lip{ 0 }; CacheClock::time_point t{}; };
    struct StepKey {
        std::int32_t  pix{ 0 }, piy{ 0 }, pzq{ 0 }, mix{ 0 }, miy{ 0 }, mzq{ 0 };
        std::uint64_t cls{ 0 };
        bool operator==(const StepKey&) const = default;
    };
    struct StepRec {
        std::uint8_t r{ 0 };                       // XStepRules' answer: 0 clear, 1 blocked, 2 blocked only beside her
        std::uint8_t terr{ 0 };                    // it needed the terrain allowance
        std::uint8_t stair{ 0 };                   // 1.6.0: it used the stair allowance
        std::int8_t  side{ 0 };                    // 1.6.0: a "beside her" answer's side (+1 left, -1 right, 0 not known)
        std::array<float, 2> z{}, d{};             // its two floor samples (1/3, 2/3): z and the water over them
        CacheClock::time_point t{};
    };
    struct CacheHash {
        static std::size_t Mix(std::uint64_t a_v)
        {
            a_v ^= a_v >> 33;
            a_v *= 0xff51afd7ed558ccdULL;
            a_v ^= a_v >> 33;
            a_v *= 0xc4ceb9fe1a85ec53ULL;
            a_v ^= a_v >> 33;
            return static_cast<std::size_t>(a_v);
        }
        static std::uint64_t Pair(std::int32_t a_a, std::int32_t a_b)
        {
            return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(a_a)) << 32) | static_cast<std::uint32_t>(a_b);
        }
        std::size_t operator()(const CellKey& a_k) const noexcept
        {
            return Mix(Pair(a_k.ix, a_k.iy)) ^ (Mix(Pair(a_k.zq, 0)) * 31u) ^ (Mix(a_k.cls) * 131u);
        }
        std::size_t operator()(const StepKey& a_k) const noexcept
        {
            return Mix(Pair(a_k.pix, a_k.piy)) ^ (Mix(Pair(a_k.pzq, a_k.mix)) * 31u) ^ (Mix(Pair(a_k.miy, a_k.mzq)) * 131u) ^
                   (Mix(a_k.cls) * 1031u);
        }
    };
    struct CellCache {
        std::uint64_t world{ 0 };                  // the physics world (bhkWorld) and the interior cell / worldspace it serves
        RE::FormID    space{ 0 };
        std::unordered_map<std::uint64_t, SquareRec>     squares;
        std::unordered_map<CellKey, CellRec, CacheHash>  cells;
        std::unordered_map<StepKey, StepRec, CacheHash>  steps;
        std::size_t   curSq{ 0 }, curCell{ 0 }, curStep{ 0 };   // the bounded sweeps' bucket cursors
        int           full{ 0 };                   // answers not kept because their table was full of live ones
        CacheClock::time_point fullSweepAt{};      // (review #10) a full table's extra sweep: at most once per slice
        std::size_t Size() const { return squares.size() + cells.size() + steps.size(); }
        void Clear()
        {
            decltype(squares)().swap(squares);
            decltype(cells)().swap(cells);
            decltype(steps)().swap(steps);
            world = 0;
            space = 0;
            curSq = curCell = curStep = 0;
            full = 0;
            fullSweepAt = {};
        }
    };
    CellCache g_cache;
    CellCache g_cacheOld;      // (review #10) a previous world's answers - never used, freed a bounded part per plan start

    float Dist2D(const RE::NiPoint3& a_a, const RE::NiPoint3& a_b)
    {
        return std::hypot(a_a.x - a_b.x, a_a.y - a_b.y);
    }

    bool IsBlockingLayer(std::uint32_t a_layer)
    {
        switch (a_layer) {
        case 1:   // static
        case 2:   // anim static
        case 3:   // transparent
        case 9:   // trees
        case 10:  // props
        case 13:  // terrain
        case 17:  // ground
        case 27:  // invisible wall
            return true;
        default:
            return false;
        }
    }

    struct WorldRead {
        RE::BSReadWriteLock& lock;
        explicit WorldRead(RE::bhkWorld* a_bw) : lock(a_bw->worldLock) { lock.LockForRead(); }
        ~WorldRead() { lock.UnlockForRead(); }
        WorldRead(const WorldRead&) = delete;
        WorldRead& operator=(const WorldRead&) = delete;
    };

    // One straight ray a -> b (game units). true = a BLOCKING hit; any other hit is stepped over (cast again from 1 u past
    // it, at most 3 times). An answer that fails its sanity test counts as blocked (the safe side).
    bool RayBlocked(Job& j, RE::bhkWorld* a_bw, const RE::NiPoint3& a_a, const RE::NiPoint3& a_b)
    {
        const RE::NiPoint3 total = a_b - a_a;
        const float        totalLen = total.Length();
        if (totalLen < 0.5f) return false;
        RE::NiPoint3 from = a_a;
        for (int k = 0; k < 4; ++k) {
            float         f = 1.f;
            std::uint32_t layer = 0;
            bool          hit = false, bad = false;
            {
                RE::bhkPickData pd;
                pd.rayInput.from = { from.x * kToHavok, from.y * kToHavok, from.z * kToHavok, 0.f };
                pd.rayInput.to   = { a_b.x * kToHavok, a_b.y * kToHavok, a_b.z * kToHavok, 0.f };
                pd.rayInput.enableShapeCollectionFilter = false;
                pd.rayInput.filterInfo.filter           = j.rayFilter;
                ++j.queries;
                ++j.rays;
                const WorldRead lock(a_bw);
                if (a_bw->PickObject(pd) && pd.rayOutput.HasHit()) {
                    hit = true;
                    f = pd.rayOutput.hitFraction;
                    const RE::hkpCollidable* c = pd.rayOutput.rootCollidable;
                    if (!(f >= 0.f && f <= 1.f) || !c) {
                        bad = true;
                    } else {
                        layer = c->broadPhaseHandle.collisionFilterInfo.filter & 0x7Fu;
                    }
                }
            }
            if (!hit) return false;
            if (bad || IsBlockingLayer(layer)) return true;
            const RE::NiPoint3 seg = a_b - from;
            const float        segLen = seg.Length();
            if (segLen < 1.f) return false;
            from = from + seg * f + seg * (1.f / segLen);
            if ((from - a_a).Length() >= totalLen) return false;
        }
        return false;
    }

    // The navmesh under (x, y), looked for a_band above and below a_z. false = none there.
    bool SnapNav(Job& j, RE::Actor* a_w, float a_x, float a_y, float a_z, float a_band, RE::NiPoint3& a_out)
    {
        ++j.queries;
        ++j.snaps;
        auto* pathing = RE::Pathing::GetSingleton();
        if (!pathing) return false;
        const RE::NiPoint3 in{ a_x, a_y, a_z };
        RE::BSTSmartPointer<RE::BSPathingCell> cell;
        if (!pathing->GetPathingCell(in, a_w->GetParentCell(), a_w->GetWorldspace(), cell) || !cell) return false;
        RE::FindTriangleForLocationFilterCheckDeltaZ filter;
        filter.maxDistAbove = a_band;
        filter.maxDistBelow = a_band;
        RE::NiPoint3 got;
        if (!pathing->FindClosestPointOnNavmesh(cell, in, filter, got)) return false;
        a_out = got;
        return true;
    }

    bool InKeepOut(const Job& j, float a_x, float a_y)
    {
        if (!j.avoidOn) return false;
        const float dx = a_x - j.avoidX, dy = a_y - j.avoidY;
        return dx * dx + dy * dy < j.avoidR * j.avoidR;
    }

    // Does the flat segment a-b come within the keep-out circle?
    bool CrossesKeepOut(const Job& j, const RE::NiPoint3& a_a, const RE::NiPoint3& a_b)
    {
        if (!j.avoidOn) return false;
        const float abx = a_b.x - a_a.x, aby = a_b.y - a_a.y;
        const float len2 = abx * abx + aby * aby;
        float       t = len2 > 1e-4f ? ((j.avoidX - a_a.x) * abx + (j.avoidY - a_a.y) * aby) / len2 : 0.f;
        t = std::clamp(t, 0.f, 1.f);
        return InKeepOut(j, a_a.x + abx * t, a_a.y + aby * t);
    }

    // One piece (at most kPiece long) of a straight walk, both ends on the floor (see the file head).
    bool PieceClear(Job& j, RE::bhkWorld* a_bw, const RE::NiPoint3& a_a, const RE::NiPoint3& a_b, float a_px, float a_py)
    {
        const float s = j.scale;
        const float knee = kKnee * s, chest = kChest * s, lo = kSweepLow * s, hi = kSweepHigh * s;
        if (RayBlocked(j, a_bw, RE::NiPoint3{ a_a.x, a_a.y, a_a.z + knee }, RE::NiPoint3{ a_b.x, a_b.y, a_b.z + knee })) return false;
        if (RayBlocked(j, a_bw, RE::NiPoint3{ a_a.x + a_px, a_a.y + a_py, a_a.z + knee }, RE::NiPoint3{ a_b.x + a_px, a_b.y + a_py, a_b.z + knee })) return false;
        if (RayBlocked(j, a_bw, RE::NiPoint3{ a_a.x - a_px, a_a.y - a_py, a_a.z + knee }, RE::NiPoint3{ a_b.x - a_px, a_b.y - a_py, a_b.z + knee })) return false;
        if (RayBlocked(j, a_bw, RE::NiPoint3{ a_a.x, a_a.y, a_a.z + chest }, RE::NiPoint3{ a_b.x, a_b.y, a_b.z + chest })) return false;
        if (RayBlocked(j, a_bw, RE::NiPoint3{ a_a.x, a_a.y, a_a.z + lo }, RE::NiPoint3{ a_b.x, a_b.y, a_b.z + hi })) return false;
        if (RayBlocked(j, a_bw, RE::NiPoint3{ a_a.x, a_a.y, a_a.z + hi }, RE::NiPoint3{ a_b.x, a_b.y, a_b.z + lo })) return false;
        return true;
    }

    // Her body fits the straight walk a -> b (both on the floor). The line is tested in pieces of at most 64 u; with
    // a_sample the navmesh is also looked for under every piece's end (no gap, no other level), and the next piece starts
    // from the floor found there.
    bool SegmentClear(Job& j, RE::Actor* a_w, RE::bhkWorld* a_bw, const RE::NiPoint3& a_a, const RE::NiPoint3& a_b, bool a_sample)
    {
        if (CrossesKeepOut(j, a_a, a_b)) return false;
        float       dx = a_b.x - a_a.x, dy = a_b.y - a_a.y;
        const float len = std::sqrt(dx * dx + dy * dy);
        if (len < 1.f) return true;
        dx /= len;
        dy /= len;
        const float side = j.radius * kSideFrac;
        const float px = -dy * side, py = dx * side;
        const int   pieces = (std::max)(1, static_cast<int>(std::ceil(len / kPiece)));
        RE::NiPoint3 pa = a_a;
        for (int k = 1; k <= pieces; ++k) {
            RE::NiPoint3 pb = a_b;
            if (k < pieces) {
                const float t = static_cast<float>(k) / static_cast<float>(pieces);
                pb = RE::NiPoint3{ a_a.x + (a_b.x - a_a.x) * t, a_a.y + (a_b.y - a_a.y) * t, a_a.z + (a_b.z - a_a.z) * t };
                if (a_sample) {
                    RE::NiPoint3 got;
                    if (!SnapNav(j, a_w, pb.x, pb.y, pb.z, kLevelBand, got)) return false;
                    if (std::hypot(got.x - pb.x, got.y - pb.y) > kCell * 0.75f) return false;    // no navmesh there
                    if (std::fabs(got.z - pa.z) > kMaxStep + kPiece * 0.6f) return false;        // another level
                    pb.z = got.z;
                }
            }
            if (!PieceClear(j, a_bw, pa, pb, px, py)) return false;
            pa = pb;
        }
        return true;
    }

    RE::NiPoint3 Pos(const Node& a_n) { return { a_n.x, a_n.y, a_n.z }; }

    void Finish(Job& j, int a_status, std::string a_why)
    {
        j.status = a_status;
        j.why = std::move(a_why);
        j.phase = 4;
        // the search's memory goes now (a finished plan waits for its caller with only its answer)
        std::vector<Node>().swap(j.nodes);
        decltype(j.open)().swap(j.open);
        std::vector<RE::NiPoint3>().swap(j.cells);
        std::vector<RE::NiPoint3>().swap(j.pulled);
    }

    void InitGrid(Job& j)
    {
        const float dist = Dist2D(j.start, j.goal);
        const float margin = std::clamp(dist * 0.6f, 160.f, 480.f);
        j.minX = (std::min)(j.start.x, j.goal.x) - margin;
        j.minY = (std::min)(j.start.y, j.goal.y) - margin;
        const float maxX = (std::max)(j.start.x, j.goal.x) + margin;
        const float maxY = (std::max)(j.start.y, j.goal.y) + margin;
        j.nx = static_cast<int>((maxX - j.minX) / kCell) + 2;
        j.ny = static_cast<int>((maxY - j.minY) / kCell) + 2;
        j.nodes.assign(static_cast<std::size_t>(j.nx) * static_cast<std::size_t>(j.ny), Node{});
        for (int iy = 0; iy < j.ny; ++iy) {
            for (int ix = 0; ix < j.nx; ++ix) {
                Node& n = j.nodes[static_cast<std::size_t>(iy) * j.nx + ix];
                n.x = j.minX + ix * kCell;
                n.y = j.minY + iy * kCell;
            }
        }
        const int sx = std::clamp(static_cast<int>(std::lround((j.start.x - j.minX) / kCell)), 0, j.nx - 1);
        const int sy = std::clamp(static_cast<int>(std::lround((j.start.y - j.minY) / kCell)), 0, j.ny - 1);
        const std::int32_t s = sy * j.nx + sx;
        Node& sn = j.nodes[s];
        sn.x = j.start.x;
        sn.y = j.start.y;
        sn.z = j.start.z;
        sn.floor = 1;
        sn.bestG = 0.f;
        j.open.push({ kHeurWeight * Dist2D(j.start, j.goal), 0.f, s, -1 });
    }

    // the step from one neighbour INTO cell m was blocked: m may still be reached from its other closed neighbours
    void ReopenFromOthers(Job& j, std::int32_t a_m)
    {
        Node&     m = j.nodes[a_m];
        const int mx = a_m % j.nx, my = a_m / j.nx;
        m.bestG = 3.0e38f;
        for (int d = 0; d < 8; ++d) {
            if (m.rejectedFrom & (1u << d)) continue;
            const int qx = mx - kDir[d][0], qy = my - kDir[d][1];      // q --(direction d)--> m
            if (qx < 0 || qy < 0 || qx >= j.nx || qy >= j.ny) continue;
            const std::int32_t qi = qy * j.nx + qx;
            const Node&        q = j.nodes[qi];
            if (!q.closed || q.floor != 1) continue;
            const float ng = q.g + (Pos(q) - Pos(m)).Length();
            if (ng < m.bestG) m.bestG = ng;
            j.open.push({ ng + kHeurWeight * Dist2D(Pos(m), j.goal), ng, a_m, qi });
        }
    }

    int DirIndex(std::int32_t a_from, std::int32_t a_to, int a_nx)
    {
        const int dx = (a_to % a_nx) - (a_from % a_nx), dy = (a_to / a_nx) - (a_from / a_nx);
        for (int d = 0; d < 8; ++d) {
            if (kDir[d][0] == dx && kDir[d][1] == dy) return d;
        }
        return -1;
    }

    // ONE slice: until the slice's budget is used or the plan is done.
    void RunSlice(Job& j, RE::Actor* a_w, RE::bhkWorld* a_bw, int a_budgetEnd)
    {
        // ---- phase 0: the goal on the navmesh, the distance, then the straight line --------------------------------
        if (j.phase == 0) {
            RE::NiPoint3 g2;
            if (!SnapNav(j, a_w, j.goal.x, j.goal.y, j.goal.z, 96.f, g2) || Dist2D(g2, j.goal) > kCell) {
                Finish(j, 0, "no navmesh at the goal (water, a bath, a raised spot) - not a walkable leg");
                return;
            }
            j.goal = g2;
            if (Dist2D(j.start, j.goal) > kMaxDistance) {
                Finish(j, 0, "too far for a local route (the engine's own pathing walks this)");
                return;
            }
            if (SegmentClear(j, a_w, a_bw, j.start, j.goal, true)) {
                Finish(j, 1, "the straight line is clear");
                return;
            }
            InitGrid(j);
            j.phase = 1;
        }
        // ---- phase 1: lazy weighted A* ------------------------------------------------------------------------------
        while (j.phase == 1) {
            if (j.queries >= a_budgetEnd) return;
            if (j.queries >= kMaxQueries) {
                Finish(j, 0, "out of budget before a way was found");
                return;
            }
            if (j.open.empty()) {
                Finish(j, 0, "no way round inside the searched area");
                return;
            }
            const OpenEntry e = j.open.top();
            j.open.pop();
            Node& n = j.nodes[e.idx];
            if (n.closed || e.g > n.bestG + 0.01f) continue;     // done already, or a stale entry
            if (e.parent >= 0) {
                const int d = DirIndex(e.parent, e.idx, j.nx);
                if (d >= 0 && (n.rejectedFrom & (1u << d))) continue;   // this step was found blocked already (the review)
                const Node& p = j.nodes[e.parent];
                if (!SegmentClear(j, a_w, a_bw, Pos(p), Pos(n), false)) {
                    ++j.rejected;
                    if (d >= 0) n.rejectedFrom |= static_cast<std::uint8_t>(1u << d);
                    ReopenFromOthers(j, e.idx);
                    continue;
                }
            }
            n.closed = true;
            n.g = e.g;
            n.parent = e.parent;
            ++j.expansions;
            if (Dist2D(Pos(n), j.goal) <= kCell * 1.5f && SegmentClear(j, a_w, a_bw, Pos(n), j.goal, false)) {
                j.cells.clear();
                j.cells.push_back(j.goal);
                for (std::int32_t k = e.idx; k >= 0; k = j.nodes[k].parent) j.cells.push_back(Pos(j.nodes[k]));
                std::reverse(j.cells.begin(), j.cells.end());
                j.cells.front() = j.start;
                j.pullFrom = 0;
                j.pullTo = 1;
                j.phase = 2;
                break;
            }
            const int ix = e.idx % j.nx, iy = e.idx / j.nx;
            for (int d = 0; d < 8; ++d) {
                const int mx = ix + kDir[d][0], my = iy + kDir[d][1];
                if (mx < 0 || my < 0 || mx >= j.nx || my >= j.ny) continue;
                const std::int32_t mi = my * j.nx + mx;
                Node&              m = j.nodes[mi];
                if (m.closed || (m.rejectedFrom & (1u << d))) continue;
                if (m.floor == 0) {
                    RE::NiPoint3 got;
                    if (!InKeepOut(j, m.x, m.y) && SnapNav(j, a_w, m.x, m.y, n.z, kLevelBand, got) &&
                        std::hypot(got.x - m.x, got.y - m.y) <= kCell * 0.6f && std::fabs(got.z - n.z) <= kMaxStep) {
                        m.x = got.x;
                        m.y = got.y;
                        m.z = got.z;
                        m.floor = 1;
                    } else {
                        m.floor = 2;
                    }
                }
                if (m.floor != 1 || std::fabs(m.z - n.z) > kMaxStep) continue;
                const float ng = n.g + (Pos(m) - Pos(n)).Length();
                if (ng < m.bestG) {
                    m.bestG = ng;
                    j.open.push({ ng + kHeurWeight * Dist2D(Pos(m), j.goal), ng, mi, e.idx });
                }
            }
        }
        // ---- phase 2: pull the cell path straight ---------------------------------------------------------------------
        while (j.phase == 2) {
            if (j.queries >= a_budgetEnd) return;
            if (j.queries >= kMaxQueries + kMaxQueries / 2) {
                Finish(j, 0, "out of budget while straightening the way");
                return;
            }
            if (j.pullTo + 1 < j.cells.size() && SegmentClear(j, a_w, a_bw, j.cells[j.pullFrom], j.cells[j.pullTo + 1], true)) {
                ++j.pullTo;
                continue;
            }
            j.pulled.push_back(j.cells[j.pullTo]);
            if (j.pullTo + 1 >= j.cells.size()) {
                j.phase = 3;
                j.spaceK = 0;
                j.spacePrev = j.start;
                break;
            }
            j.pullFrom = j.pullTo;
            j.pullTo = j.pullFrom + 1;
        }
        // ---- phase 3: waypoints at least kMinLeg apart (resumable) --------------------------------------------------------
        while (j.phase == 3) {
            if (j.queries >= a_budgetEnd) return;
            if (j.spaceK >= j.pulled.size()) {
                if (j.out.size() > static_cast<std::size_t>(kMaxWaypoints)) {
                    Finish(j, 0, "too many turns for a local way round");
                } else if (j.out.size() <= 1) {
                    Finish(j, 1, "the straight line is clear (after the pull)");
                } else {
                    Finish(j, 2, "a way round");
                }
                return;
            }
            const std::size_t  k = j.spaceK++;
            const RE::NiPoint3 p = j.pulled[k];
            const bool         last = k + 1 == j.pulled.size();
            if (!last && Dist2D(j.spacePrev, p) < kMinLeg) {
                const RE::NiPoint3 next = j.pulled[k + 1];
                if (SegmentClear(j, a_w, a_bw, j.spacePrev, next, true)) {
                    continue;      // too short a leg, and she can walk past it straight to the next one
                }
                // (the review) a short corner point she would only pose at: pushed out along her line to kMinLeg, when
                // that point is on the floor and both lines from it are clear
                const float  dl = (std::max)(1.f, Dist2D(j.spacePrev, p));
                const float  ex = j.spacePrev.x + (p.x - j.spacePrev.x) / dl * kMinLeg;
                const float  ey = j.spacePrev.y + (p.y - j.spacePrev.y) / dl * kMinLeg;
                RE::NiPoint3 e;
                if (!InKeepOut(j, ex, ey) && SnapNav(j, a_w, ex, ey, p.z, kLevelBand, e) && Dist2D(e, RE::NiPoint3{ ex, ey, p.z }) <= kCell * 0.6f &&
                    SegmentClear(j, a_w, a_bw, j.spacePrev, e, true) && SegmentClear(j, a_w, a_bw, e, next, true)) {
                    ++j.pushed;
                    j.out.push_back(e);
                    j.spacePrev = e;
                    continue;
                }
            }
            j.out.push_back(p);
            j.spacePrev = p;
        }
    }

    void LogDone(const Job& j)
    {
        std::string pts;
        for (const auto& p : j.out) pts += std::format(" ({:.0f},{:.0f},{:.0f})", p.x, p.y, p.z);
        logger::info("PATH #{}: {} [{}] - {} waypoint(s){} | {} questions ({} rays, {} navmesh), {} cells opened, {} steps "
                     "blocked, {} pushed out, {} slice(s), {:.2f} ms in all, longest slice {:.2f} ms",
                     j.handle, j.status == 2 ? "ROUTE" : j.status == 1 ? "DIRECT" : j.status == 0 ? "NO ROUTE" : "FAILED", j.why,
                     j.status == 2 ? j.out.size() : 0, j.status == 2 ? pts : std::string(), j.queries, j.rays, j.snaps,
                     j.expansions, j.rejected, j.pushed, j.slices, j.usTotal / 1000.0, j.usMax / 1000.0);
    }

    // =================================================================================================================
    //  1.4.0 THE EX MODE - everything from here to Step runs only for a StartEx plan (B §2-§7). Main thread only.
    // =================================================================================================================

    bool IsFiniteHeight(float a_h) { return std::isfinite(a_h) && a_h > -1.0e7f; }    // -NI_INFINITY = "no water"

    RE::NiPoint3 XPos(const XNode& a_n) { return { a_n.x, a_n.y, a_n.z }; }

    // what a floor ray stands on (B §3.3): static, anim static, transparent, props, terrain, ground, stair helper
    bool IsFloorLayer(std::uint32_t a_layer)
    {
        switch (a_layer) {
        case 1:   // static
        case 2:   // anim static
        case 3:   // transparent
        case 10:  // props
        case 13:  // terrain
        case 17:  // ground
        case 31:  // stair helper (what a character controller walks on; whether a layer-30 ray reports it is UNVERIFIED)
            return true;
        default:
            return false;
        }
    }

    // this slice's share is used: its questions (96 / running plans, >= 24) or its time (1.0 ms / running plans)
    bool Spent(const Job& j)
    {
        const ExState& x = *j.x;
        return j.queries >= x.qEnd || std::chrono::steady_clock::now() >= x.deadline;
    }

    void ExFinish(Job& j, int a_status, std::string a_why)
    {
        j.status = a_status;
        j.why = std::move(a_why);
        if (j.x) {
            j.x->xphase = kXDone;
            j.x->seg.active = false;
        }
        if (a_status != 1 && a_status != 2 && a_status != 3) {
            j.route.clear();
            j.switchIndex = 0;
            j.routeLen = 0.f;
        }
    }

    // an Ex plan's search memory goes once its answer is final: its own state, and the 1.3.0 grid a navmesh-first try or the
    // 1.5.0 MIX's corridor may have used (the answer, Job::route, stays until Cancel / ClearAll)
    void ExFree(Job& j)
    {
        j.x.reset();
        std::vector<Node>().swap(j.nodes);
        decltype(j.open)().swap(j.open);
    }

    struct FloorHit {
        bool  found{ false };     // a floor layer at z, slope normal z nz
        bool  blocked{ false };   // a tree / an invisible wall stands there
        float z{ 0.f }, nz{ 1.f };
        int   water{ 0 };         // water (layer 11) hits stepped through ...
        float waterZ{ 0.f };      // ... the first one's z
        std::uint32_t layer{ 0 }; // 1.5.1: the layer of the floor (or blocker) found - set by XFloorRay only (terrain = 13 / 17)
        RE::FormID    ref{ 0 };   // 1.6.1: the reference hit - XFloorRay, only while x.wantRef (a probe's side rays)
    };

    // ONE down ray at (x, y) from a_top to a_bottom (B §3.3). Up to 4 casts: past each hit that is not floor it casts again
    // from 1 u under it. Floor layers = found; trees / invisible walls = blocked; water is never floor (its z is kept for
    // the WATER SEEN cross-check); no hit / a bad answer = no floor (the safe side). The slope = the normal's z (unsigned,
    // normalised); a hit without a usable normal counts as flat and is counted (normBad) so the log shows it.
    FloorHit FloorRay(Job& j, RE::bhkWorld* a_bw, float a_x, float a_y, float a_top, float a_bottom, int& a_bucket)
    {
        FloorHit h;
        float    from = a_top;
        for (int k = 0; k < 4 && from - a_bottom >= 1.f; ++k) {
            float             f = 1.f;
            std::uint32_t     layer = 0;
            bool              hit = false, bad = false;
            alignas(16) float nv[4]{};
            {
                RE::bhkPickData pd;
                pd.rayInput.from = { a_x * kToHavok, a_y * kToHavok, from * kToHavok, 0.f };
                pd.rayInput.to   = { a_x * kToHavok, a_y * kToHavok, a_bottom * kToHavok, 0.f };
                pd.rayInput.enableShapeCollectionFilter = false;
                pd.rayInput.filterInfo.filter           = j.rayFilter;
                ++j.queries;
                ++j.rays;
                ++a_bucket;
                const WorldRead lock(a_bw);
                if (a_bw->PickObject(pd) && pd.rayOutput.HasHit()) {
                    hit = true;
                    f = pd.rayOutput.hitFraction;
                    const RE::hkpCollidable* c = pd.rayOutput.rootCollidable;
                    if (!(f >= 0.f && f <= 1.f) || !c) {
                        bad = true;
                    } else {
                        layer = c->broadPhaseHandle.collisionFilterInfo.filter & 0x7Fu;
                        _mm_store_ps(nv, pd.rayOutput.normal.quad);
                    }
                }
            }
            if (!hit || bad) return h;
            const float z = from + (a_bottom - from) * f;
            if (IsFloorLayer(layer)) {
                h.found = true;
                h.z = z;
                const float nl = std::sqrt(nv[0] * nv[0] + nv[1] * nv[1] + nv[2] * nv[2]);
                if (nl > 0.5f) {
                    h.nz = std::fabs(nv[2]) / nl;
                } else {
                    h.nz = 1.f;
                    ++j.xs.normBad;
                }
                return h;
            }
            if (layer == 9 || layer == 27) {     // trees, invisible wall
                h.blocked = true;
                h.z = z;
                return h;
            }
            if (layer == 11) {
                if (h.water == 0) h.waterZ = z;
                ++h.water;
            }
            from = z - 1.f;
        }
        return h;
    }

    struct XHit {
        bool  hit{ false };
        float frac{ 1.f };        // along the whole a -> b
        float nx{ 0.f }, ny{ 0.f }, nz{ 0.f };
    };

    // RayBlocked's twin that also says WHERE it hit and the surface's normal (the classifier's wall rays; log only).
    XHit RayFirstBlock(Job& j, RE::bhkWorld* a_bw, const RE::NiPoint3& a_a, const RE::NiPoint3& a_b, int& a_bucket)
    {
        XHit               out;
        const RE::NiPoint3 total = a_b - a_a;
        const float        totalLen = total.Length();
        if (totalLen < 0.5f) return out;
        RE::NiPoint3 from = a_a;
        for (int k = 0; k < 4; ++k) {
            float             f = 1.f;
            std::uint32_t     layer = 0;
            bool              hit = false, bad = false;
            alignas(16) float nv[4]{};
            {
                RE::bhkPickData pd;
                pd.rayInput.from = { from.x * kToHavok, from.y * kToHavok, from.z * kToHavok, 0.f };
                pd.rayInput.to   = { a_b.x * kToHavok, a_b.y * kToHavok, a_b.z * kToHavok, 0.f };
                pd.rayInput.enableShapeCollectionFilter = false;
                pd.rayInput.filterInfo.filter           = j.rayFilter;
                ++j.queries;
                ++j.rays;
                ++a_bucket;
                const WorldRead lock(a_bw);
                if (a_bw->PickObject(pd) && pd.rayOutput.HasHit()) {
                    hit = true;
                    f = pd.rayOutput.hitFraction;
                    const RE::hkpCollidable* c = pd.rayOutput.rootCollidable;
                    if (!(f >= 0.f && f <= 1.f) || !c) {
                        bad = true;
                    } else {
                        layer = c->broadPhaseHandle.collisionFilterInfo.filter & 0x7Fu;
                        _mm_store_ps(nv, pd.rayOutput.normal.quad);
                    }
                }
            }
            if (!hit) return out;
            const RE::NiPoint3 seg = a_b - from;
            const float        segLen = seg.Length();
            const RE::NiPoint3 at = from + seg * f;
            if (bad || IsBlockingLayer(layer)) {
                out.hit = true;
                out.frac = (at - a_a).Length() / totalLen;
                const float nl = std::sqrt(nv[0] * nv[0] + nv[1] * nv[1] + nv[2] * nv[2]);
                if (nl > 0.5f) {
                    out.nx = nv[0] / nl;
                    out.ny = nv[1] / nl;
                    out.nz = nv[2] / nl;
                }
                return out;
            }
            if (segLen < 1.f) return out;
            from = at + seg * (1.f / segLen);
            if ((from - a_a).Length() >= totalLen) return out;
        }
        return out;
    }

    // The water over a floor point (B §4.1), no Water Finder needed: (1) a placed water activator whose footprint holds
    // (x, y) with the floor from 400 u under to 4 u over its surface (several: the lowest surface); (2) outdoors the water
    // height at that point (TES::GetWaterHeight - per point, never one plane for the whole route); (3) indoors the cell
    // plane, ONLY when the cell has no placed water (the TNF Breezehome flags HasWater at height 0: KB 8920-8924), within
    // floor - 10 .. floor + 200. depth = surface - floor (0 when not above it). false = no loaded cell there (unknown is not
    // dry). One question.
    bool DepthAt(Job& j, RE::Actor* a_w, float a_x, float a_y, float a_z, float& a_depth, float& a_surface)
    {
        ExState& x = *j.x;
        // (1.6.0 §4 #9) a water lookup asks no physics question: on a 1.5.0-flag plan it counts 1/4 question (every 4th one counts
        // one) against every budget, as a cache answer does; 1.4.1 plans count each one in full as before
        if (x.v15) {
            if ((++j.xs.nW & 3) == 0) ++j.queries;
        } else {
            ++j.queries;
            ++j.xs.nW;
        }
        a_depth = 0.f;
        a_surface = -1.0e9f;
        // several placed waters: the lowest surface ABOVE the floor wins (one up to 4 u under the floor only says "dry here")
        bool  placed = false;
        float above = 3.0e38f, under = -3.0e38f;
        for (const auto& wf : x.waters) {
            if (a_z <= wf.surf + kXWaterOver && a_z >= wf.surf - kXWaterUnder && wf.Contains(a_x, a_y, kXFootPad)) {
                placed = true;
                if (wf.surf > a_z) {
                    above = (std::min)(above, wf.surf);
                } else {
                    under = (std::max)(under, wf.surf);
                }
            }
        }
        if (placed) {
            a_surface = above < 1.0e38f ? above : under;
        } else {
            auto* tes = RE::TES::GetSingleton();
            if (!tes) return false;
            const RE::NiPoint3 p{ a_x, a_y, a_z };
            if (x.interior) {
                auto* cell = a_w->GetParentCell();
                if (!cell) return false;                       // unknown is not dry
                if (!x.placedInCell) {
                    float cw = 0.f;
                    if (cell->GetWaterHeight(p, cw) && IsFiniteHeight(cw) && cw >= a_z - kXInteriorDown && cw <= a_z + kXInteriorUp) {
                        a_surface = cw;
                    }
                }
            } else {
                auto* cell = tes->GetCell(p);
                if (!cell) return false;
                const float wz = tes->GetWaterHeight(p, cell);
                if (IsFiniteHeight(wz)) a_surface = wz;
            }
        }
        if (a_surface > a_z) a_depth = a_surface - a_z;
        if (a_depth > kXWet) ++j.xs.wet;
        return true;
    }

    // (review R #6) WHICH placed water DepthAt would use over a floor point: its index in x.waters, or -1 (none: exterior water
    // height / interior cell plane / dry). The same choice as DepthAt's: the lowest surface above the floor, else the highest
    // one up to kXWaterOver under it. No question asked.
    int PlacedWaterAt(const ExState& x, float a_x, float a_y, float a_z)
    {
        int   above = -1, under = -1;
        float aboveZ = 3.0e38f, underZ = -3.0e38f;
        for (std::size_t i = 0; i < x.waters.size(); ++i) {
            const WaterFP& wf = x.waters[i];
            if (a_z > wf.surf + kXWaterOver || a_z < wf.surf - kXWaterUnder || !wf.Contains(a_x, a_y, kXFootPad)) continue;
            if (wf.surf > a_z) {
                if (wf.surf < aboveZ) {
                    aboveZ = wf.surf;
                    above = static_cast<int>(i);
                }
            } else if (wf.surf > underZ) {
                underZ = wf.surf;
                under = static_cast<int>(i);
            }
        }
        return above >= 0 ? above : under;
    }

    // B §4.2: does a layer-30 down ray report water at all? Its water hits vs our surface, for the WATER SEEN line.
    void NoteWater(Job& j, const FloorHit& a_h, float a_surface)
    {
        if (!a_h.water) return;
        ++j.xs.waterSeen;
        if (IsFiniteHeight(a_surface)) {
            j.xs.waterDeltaMax = (std::max)(j.xs.waterDeltaMax, std::fabs(a_h.waterZ - a_surface));
        } else {
            ++j.xs.waterSeenNoOurs;
        }
    }

    // B §7.7: a cell near a spot she got stuck at is refused. (Review G, BUG 3) B's start exemption (r + 48 u round her
    // start) applies only to a spot UNDER her (within kXBadUnderHer of the start: she must be able to leave where she
    // stands); a spot ahead of her - the driver marks a stall 32 u ahead - is refused right up to her, or the one re-plan
    // would walk the same line into the same blocker.
    bool IsBadSpot(const Job& j, float a_x, float a_y, float a_z)
    {
        const ExState& x = *j.x;
        if (x.bad.empty()) return false;
        const bool nearStart = std::hypot(a_x - j.start.x, a_y - j.start.y) < j.radius + kXBadStartPad;
        for (const auto& b : x.bad) {
            if (nearStart && std::hypot(b.x - j.start.x, b.y - j.start.y) < kXBadUnderHer) continue;
            if (std::hypot(a_x - b.x, a_y - b.y) < j.radius + kXBadPad && std::fabs(a_z - b.z) <= kXBadDz) return true;
        }
        return false;
    }

    // her live bad spots, copied into the plan (g_mtx held by the caller)
    void SnapshotBad(Job& j)
    {
        const auto it = g_bad.find(j.walker);
        if (it == g_bad.end()) return;
        const auto now = std::chrono::steady_clock::now();
        for (const auto& b : it->second) {
            if (b.until > now) j.x->bad.push_back(b);
        }
    }

    // =================================================================================================================
    //  1.5.0 THE BODY RULES' RAYS (x.v15 only; B §7 C5-C12). The 1.4.1 rays above are untouched and still used by 1.4.1.
    // =================================================================================================================

    // C12: what stops her body: unidentified, static, anim static, transparent, trees, props, terrain, ground, large debris,
    // small transparent (+ anim), invisible wall, collision box - what her controller AND the engine's own path pick collide with
    bool IsBlockingLayerV(std::uint32_t a_layer)
    {
        switch (a_layer) {
        case 0: case 1: case 2: case 3: case 9: case 10: case 13: case 17: case 20: case 26: case 27: case 28: case 35:
            return true;
        default:
            return false;
        }
    }

    // C12: what a down ray stands on: unidentified, static, anim static, transparent, props, terrain, ground, small
    // transparent (+ anim), stair helper
    bool IsFloorLayerV(std::uint32_t a_layer)
    {
        switch (a_layer) {
        case 0: case 1: case 2: case 3: case 10: case 13: case 17: case 26: case 28: case 31:
            return true;
        default:
            return false;
        }
    }

    // C12: a down ray that meets one of these finds no place to stand there: trees, large debris, invisible wall, collision box
    bool IsBlockedFloorLayerV(std::uint32_t a_layer) { return a_layer == 9 || a_layer == 20 || a_layer == 27 || a_layer == 35; }

    // 1.5.1 TERRAIN / ground (13 / 17): with the navmesh on it, a step may be as steep as the engine walks there
    bool IsTerrainLayer(std::uint32_t a_layer) { return a_layer == 13 || a_layer == 17; }

    // 1.5.1 the terrain allowance's rise over a horizontal distance a_h: h x tan 63 deg + 8 u
    float TerrainRise(float a_h) { return a_h * kVTerrainTan + kVTerrainPad; }

    // 1.6.0 §5 THE STAIR ALLOWANCE: a FIXED floor (static 1 / transparent 3 / stair helper 31) steeper than the floor rule (0.70) but
    // n.z >= 0.66 (48.7 deg) is walkable where the navmesh lies under it - the evidence that the engine walks her there (her own AI
    // took Lakeview Manor's ~46 deg ramp stairs both ways; they are part of the upper floor's STATIC mesh, so the 1.5.1 terrain rule
    // never applied). Never a stand spot (0.85). The caller asks the navmesh (NavUnder) where it is needed.
    bool IsStairLayer(std::uint32_t a_layer) { return a_layer == 1 || a_layer == 3 || a_layer == 31; }
    bool StairSlope(float a_nz, std::uint32_t a_layer) { return a_nz >= kStairNz && a_nz < kXMinNz && IsStairLayer(a_layer); }
    // the rise the stair allowance allows over a horizontal distance a_h (the floors beside her on a stair: 24 u -> 31 u)
    float StairRise(float a_h) { return a_h * kVStairTan + kVStairPad; }

    // 1.5.1 the cell cache keeps only answers of FIXED world layers: static, transparent, trees, water (a marker), terrain,
    // ground, small transparent, invisible wall, stair helper, collision box. Anything else (a door / anim static 2, clutter,
    // props, debris, actors, the unidentified 0, animated small transparent 28 ...) may move or be judged per plan.
    bool IsFixedLayerV(std::uint32_t a_layer)
    {
        switch (a_layer) {
        case 1: case 3: case 9: case 11: case 13: case 17: case 26: case 27: case 31: case 35:
            return true;
        default:
            return false;
        }
    }

    // (review R1 #9) is the hit body KEYFRAMED - a door's swinging leaf? A door model's fixed frame / lintel is not, and stays a
    // wall even when the door may be passed. Read under the world lock (the collidable is the world's), PPB's VR-run way: an
    // entity's owner is its rigid body, checked to own this very collidable. Unreadable = not a leaf (the safe side).
    bool IsKeyframedBody(const RE::hkpCollidable* a_c)
    {
        if (!a_c || a_c->broadPhaseHandle.type != 1) return false;      // 1 = an entity
        const auto* rb = static_cast<const RE::hkpRigidBody*>(a_c->GetOwner());
        if (!rb || static_cast<const RE::hkpCollidable*>(&rb->collidable) != a_c) return false;
        return rb->motion.type.get() == RE::hkpMotion::MotionType::kKeyframed;
    }

    const ExState::DoorMemo* DoorMemoFind(const ExState& x, const void* a_col)
    {
        for (const auto& m : x.doorMemo) {
            if (m.col == a_col) return &m;
        }
        return nullptr;
    }

    // C14 / item 7 + reviews R2 S1/S3: may she pass this layer-2 body? Only a door that IsOpenableDoor() accepts - the ONE rule
    // the driver also applies before it opens one (a Door with an open / close state, not locked by the lock FLAG, not a load
    // door, no activation links) - AND closed / closing at plan time (an open leaf stands where it stands: a wall there).
    // a_ref was resolved under the world read lock (FindCollidableRef, the VR-proven way) when the body was new to this plan's
    // memo; judged once, one log line per door (with the raw open state). Returns the door's FormID when she may pass it, else 0.
    // (Review R1 #9) a_leaf: the hit body is keyframed (the leaf); a fixed part of the door's model (a frame) is a wall.
    RE::FormID DoorPassFor(Job& j, const void* a_col, bool a_asked, RE::TESObjectREFR* a_ref, bool a_leaf)
    {
        ExState& x = *j.x;
        if (const auto* m = DoorMemoFind(x, a_col)) return m->pass ? m->id : 0;
        if (!a_asked) return 0;
        const bool        remember = x.doorMemo.size() < static_cast<std::size_t>(kVDoorMemo);   // full: judged again, quietly
        ExState::DoorMemo memo;
        memo.col = a_col;
        auto* base = a_ref ? a_ref->GetBaseObject() : nullptr;
        if (a_ref && base && base->Is(RE::FormType::Door)) {
            using OS = RE::BGSOpenCloseForm::OPEN_STATE;
            memo.id = a_ref->GetFormID();
            const OS   st = RE::BGSOpenCloseForm::GetOpenState(a_ref);
            const bool closed = st == OS::kClosed || st == OS::kClosing;
            memo.pass = a_leaf && IsOpenableDoor(a_ref) && closed;
            if (remember) {
                const char* wall = "";
                if (!memo.pass) {
                    const auto* lk = a_ref->GetLock();
                    wall = !a_leaf ? "a fixed part of the door's model (a frame / a lintel), not its leaf" :
                           a_ref->extraList.HasType<RE::ExtraTeleport>() ? "a load door" :
                           (a_ref->extraList.HasType<RE::ExtraActivateRef>() || a_ref->extraList.HasType<RE::ExtraActivateRefChildren>()) ?
                                                                            "wired to other references (a trap or a script) - never opened by Standpoint" :
                           (lk && lk->IsLocked())                         ? "locked" :
                           st == OS::kNone                                ? "it has no open / close state" :
                                                                            "open (its leaf stands where it is)";
                }
                if (memo.pass) ++j.xs.doorsPassed;
                logger::info("PATH {} door {:08X} (base {:08X}, open state {}): {}",
                             j.handle ? std::format("#{}", j.handle) : std::string("probe"), memo.id, base->GetFormID(), static_cast<int>(st),
                             memo.pass ? "closed, unlocked, not a load door, no activation links - her route may pass it (it is opened before that leg)" :
                                         std::format("a wall here: {}", wall));
            }
        } else {
            memo.id = a_ref ? a_ref->GetFormID() : 0;     // a moving platform, a lift ...: a wall, remembered
        }
        if (remember) x.doorMemo.push_back(memo);
        return memo.pass ? memo.id : 0;
    }

    // a navmesh leg's first blocker (the CARPET line); a_ref resolved under the lock
    void CaptureHit(ExState& x, std::uint32_t a_layer, RE::TESObjectREFR* a_ref, const RE::NiPoint3& a_at)
    {
        if (!x.capOn || x.capHave) return;
        x.capHave = true;
        x.capLayer = a_layer;
        x.capRef = a_ref ? a_ref->GetFormID() : 0;
        x.capAt = a_at;
    }

    // RayBlocked's 1.5.0 twin: the C12 list, up to 6 step-overs, a passable door (kDoors, one per test: x.curDoor) is stepped
    // through, and the first blocking hit is kept when the test asks for it (x.capOn). (Review R1 #1) only a CENTRE-line ray
    // may start passing a door; a ray beside her (x.doorSide) passes only the door her centre line already passed, any other
    // door leaf is a wall to it. (Review R1 #12) a ray that runs out of its step-overs before the end is BLOCKED (the safe side).
    // a_frac (may be null): where along a -> b it was blocked (0..1).
    bool XRayBlocked(Job& j, RE::bhkWorld* a_bw, const RE::NiPoint3& a_a, const RE::NiPoint3& a_b, int& a_bucket, float* a_frac = nullptr)
    {
        ExState&           x = *j.x;
        const RE::NiPoint3 total = a_b - a_a;
        const float        totalLen = total.Length();
        if (totalLen < 0.5f) return false;
        const auto blockedAt = [&](const RE::NiPoint3& a_at) {
            if (a_frac) *a_frac = std::clamp((a_at - a_a).Length() / totalLen, 0.f, 1.f);
            return true;
        };
        RE::NiPoint3 from = a_a;
        for (int k = 0; k < kVCasts; ++k) {
            float                    f = 1.f;
            std::uint32_t            layer = 0;
            bool                     hit = false, bad = false, doorAsk = false, capAsk = false, leaf = false;
            const RE::hkpCollidable* col = nullptr;
            RE::TESObjectREFR*       ref = nullptr;
            alignas(16) float        nv[4]{};      // (review #4) the hit's normal: a side ray steps over land only where it is ground
            {
                RE::bhkPickData pd;
                pd.rayInput.from = { from.x * kToHavok, from.y * kToHavok, from.z * kToHavok, 0.f };
                pd.rayInput.to   = { a_b.x * kToHavok, a_b.y * kToHavok, a_b.z * kToHavok, 0.f };
                pd.rayInput.enableShapeCollectionFilter = false;
                pd.rayInput.filterInfo.filter           = j.rayFilter;
                ++j.queries;
                ++j.rays;
                ++a_bucket;
                const WorldRead lock(a_bw);
                if (a_bw->PickObject(pd) && pd.rayOutput.HasHit()) {
                    hit = true;
                    f = pd.rayOutput.hitFraction;
                    col = pd.rayOutput.rootCollidable;
                    if (!(f >= 0.f && f <= 1.f) || !col) {
                        bad = true;
                    } else {
                        layer = col->broadPhaseHandle.collisionFilterInfo.filter & 0x7Fu;
                        _mm_store_ps(nv, pd.rayOutput.normal.quad);
                        if (IsBlockingLayerV(layer)) {
                            // resolved while the world is still locked (the collidable is the world's)
                            doorAsk = layer == 2 && x.doorsOk && !DoorMemoFind(x, col);
                            capAsk = x.capOn && !x.capHave;
                            if (doorAsk || capAsk) ref = RE::TESHavokUtilities::FindCollidableRef(*col);
                            if (doorAsk) leaf = IsKeyframedBody(col);
                        }
                    }
                }
            }
            if (!hit) return false;
            const RE::NiPoint3 seg = a_b - from;
            const float        segLen = seg.Length();
            const RE::NiPoint3 at = from + seg * f;
            if (bad || !IsFixedLayerV(layer)) ++x.cacheDirty;     // 1.5.1: not a fixed world answer - never cached
            if (bad) {
                CaptureHit(x, 0u, nullptr, at);
                return blockedAt(at);
            }
            if (IsBlockingLayerV(layer)) {
                if (layer == 2 && x.doorsOk) {
                    const RE::FormID d = DoorPassFor(j, col, doorAsk, ref, leaf);
                    // a second door on one test stays a wall; a ray beside her never STARTS a door pass (R1 #1); a cell's own
                    // clearance probe sees through a passable door without recording it (x.doorPeek)
                    if (d && (x.doorPeek || x.curDoor == d || (x.curDoor == 0 && !x.doorSide))) {
                        if (!x.doorPeek) x.curDoor = d;
                        if (segLen < 1.f) return false;
                        from = at + seg * (1.f / segLen);
                        if ((from - a_a).Length() >= totalLen) return false;
                        continue;
                    }
                }
                // (1.5.1) on a terrain step on the navmesh a ray BESIDE her (a side ray, a clearance probe, a side column, a cell's
                // knee probe) that meets terrain / ground meets the hillside she walks on: stepped over. The floor tests beside
                // her (brinks, the cell's lip) still refuse a rise past the terrain allowance - a cliff stays a wall.
                // (review #4) only where the face it meets is GROUND (|n.z| >= 0.40, a slope she could walk): a steep layer-13 / 17
                // face (a mod's landscape mesh, a cut bank) is an obstacle; a hit without a usable normal stays one too
                if (x.terrSide && (x.doorSide || x.doorPeek) && IsTerrainLayer(layer)) {
                    const float nl = std::sqrt(nv[0] * nv[0] + nv[1] * nv[1] + nv[2] * nv[2]);
                    if (nl > 0.5f && std::fabs(nv[2]) / nl >= kVSideTerrNz) {
                        if (segLen < 1.f) return false;
                        from = at + seg * (1.f / segLen);
                        if ((from - a_a).Length() >= totalLen) return false;
                        continue;
                    }
                }
                // (1.6.0 §5) on a STAIR step (the stair allowance) a ray BESIDE her that meets the stair's own sloped face (a fixed
                // floor layer whose normal is at least 0.55 up - a ramp, never a wall or a riser) meets the floor she climbs: stepped
                // over. Centre rays and headroom still block. (fix round, R2 #9) the SIGNED n.z: a face looking down (the underside
                // of a flight above her on a switchback stair) is an obstacle
                if (x.stairSide && (x.doorSide || x.doorPeek) && IsStairLayer(layer)) {
                    const float nl = std::sqrt(nv[0] * nv[0] + nv[1] * nv[1] + nv[2] * nv[2]);
                    if (nl > 0.5f && nv[2] / nl >= kVStairSideNz) {
                        if (segLen < 1.f) return false;
                        from = at + seg * (1.f / segLen);
                        if ((from - a_a).Length() >= totalLen) return false;
                        continue;
                    }
                }
                if (capAsk) CaptureHit(x, layer, ref, at);
                return blockedAt(at);
            }
            if (segLen < 1.f) return false;
            from = at + seg * (1.f / segLen);
            if ((from - a_a).Length() >= totalLen) return false;
        }
        ++x.cacheDirty;              // (1.5.1) not proven either way
        return blockedAt(from);      // (R1 #12) the step-overs ran out before the end: not proven clear
    }

    // FloorRay's 1.5.0 twin (C12): floor = 0, 1, 2, 3, 10, 13, 17, 26, 28, 31; no place to stand = 9, 20, 27, 35; water (11) is
    // a marker; anything else (and a passable door leaf, kDoors) is stepped through, up to 6 times.
    FloorHit XFloorRay(Job& j, RE::bhkWorld* a_bw, float a_x, float a_y, float a_top, float a_bottom, int& a_bucket)
    {
        ExState& x = *j.x;
        FloorHit h;
        float    from = a_top;
        for (int k = 0; k < kVCasts && from - a_bottom >= 1.f; ++k) {
            float                    f = 1.f;
            std::uint32_t            layer = 0;
            bool                     hit = false, bad = false, doorAsk = false, capAsk = false, leaf = false;
            const RE::hkpCollidable* col = nullptr;
            RE::TESObjectREFR*       ref = nullptr;
            RE::FormID               refId = 0;    // (1.6.1 A3) the hit's reference, read under the lock (ignoreOn / wantRef only)
            alignas(16) float        nv[4]{};
            {
                RE::bhkPickData pd;
                pd.rayInput.from = { a_x * kToHavok, a_y * kToHavok, from * kToHavok, 0.f };
                pd.rayInput.to   = { a_x * kToHavok, a_y * kToHavok, a_bottom * kToHavok, 0.f };
                pd.rayInput.enableShapeCollectionFilter = false;
                pd.rayInput.filterInfo.filter           = j.rayFilter;
                ++j.queries;
                ++j.rays;
                ++a_bucket;
                const WorldRead lock(a_bw);
                if (a_bw->PickObject(pd) && pd.rayOutput.HasHit()) {
                    hit = true;
                    f = pd.rayOutput.hitFraction;
                    col = pd.rayOutput.rootCollidable;
                    if (!(f >= 0.f && f <= 1.f) || !col) {
                        bad = true;
                    } else {
                        layer = col->broadPhaseHandle.collisionFilterInfo.filter & 0x7Fu;
                        _mm_store_ps(nv, pd.rayOutput.normal.quad);
                        doorAsk = layer == 2 && x.doorsOk && !DoorMemoFind(x, col);
                        capAsk = IsBlockedFloorLayerV(layer) && x.capOn && !x.capHave;
                        const bool refAsk = (x.ignoreOn || x.wantRef) && (IsFloorLayerV(layer) || IsBlockedFloorLayerV(layer));
                        if (doorAsk || capAsk || refAsk) ref = RE::TESHavokUtilities::FindCollidableRef(*col);
                        if (refAsk && ref) refId = ref->GetFormID();
                        if (doorAsk) leaf = IsKeyframedBody(col);
                    }
                }
            }
            if (bad) ++x.cacheDirty;                                // 1.5.1: never cached
            if (!hit || bad) return h;
            if (!IsFixedLayerV(layer)) ++x.cacheDirty;              // 1.5.1: a door, clutter, an actor ... - never cached
            const float z = from + (a_bottom - from) * f;
            if (layer == 2 && x.doorsOk && DoorPassFor(j, col, doorAsk, ref, leaf)) {     // the top of a closed door leaf: not a floor
                from = z - 1.f;
                continue;
            }
            // (1.6.1 A3) the probe's target: its own body is no floor and no wall for these rays (stepped through, never cached)
            if (x.ignoreOn && refId != 0 && refId == x.ignoreRef) {
                ++x.ignoredHits;
                ++x.cacheDirty;
                from = z - 1.f;
                continue;
            }
            h.layer = layer;
            h.ref = refId;
            if (IsFloorLayerV(layer)) {
                h.found = true;
                h.z = z;
                const float nl = std::sqrt(nv[0] * nv[0] + nv[1] * nv[1] + nv[2] * nv[2]);
                if (nl > 0.5f) {
                    h.nz = std::fabs(nv[2]) / nl;
                } else {
                    h.nz = 1.f;
                    ++j.xs.normBad;
                }
                return h;
            }
            if (IsBlockedFloorLayerV(layer)) {
                h.blocked = true;
                h.z = z;
                if (capAsk) CaptureHit(x, layer, ref, RE::NiPoint3{ a_x, a_y, z });
                return h;
            }
            if (layer == 11) {
                if (h.water == 0) h.waterZ = z;
                ++h.water;
            }
            from = z - 1.f;
        }
        if (from - a_bottom >= 1.f) ++x.cacheDirty;    // (1.5.1) the step-overs ran out before the window's end: not proven
        return h;
    }

    // C13: inside a standing actor's disc (her floor within the disc's height band: another floor above or below is not hers)
    bool InActorDisc(const ExState& x, float a_x, float a_y, float a_z)
    {
        for (const auto& d : x.discs) {
            const float dx = a_x - d.x, dy = a_y - d.y;
            if (dx * dx + dy * dy < d.r * d.r && std::fabs(a_z - d.z) <= x.htop) return true;
        }
        return false;
    }

    bool CrossesActorDisc(const ExState& x, const RE::NiPoint3& a_a, const RE::NiPoint3& a_b)
    {
        if (x.discs.empty()) return false;
        const float abx = a_b.x - a_a.x, aby = a_b.y - a_a.y;
        const float len2 = abx * abx + aby * aby;
        for (const auto& d : x.discs) {
            float t = len2 > 1e-4f ? ((d.x - a_a.x) * abx + (d.y - a_a.y) * aby) / len2 : 0.f;
            t = std::clamp(t, 0.f, 1.f);
            const float px = a_a.x + abx * t - d.x, py = a_a.y + aby * t - d.y, pz = a_a.z + (a_b.z - a_a.z) * t;
            if (px * px + py * py < d.r * d.r && std::fabs(pz - d.z) <= x.htop) return true;
        }
        return false;
    }

    // C6: the two side columns at a point p of her line (unit direction a_ux, a_uy): up-rays at +-rc across it from the floor +
    // riser + 4 to the floor + Htop - a table top, a counter, a shelf, a beam beside her. true = one is blocked. (Rays beside
    // her: they pass only a door her centre line passed - R1 #1.)
    bool XSideColumns(Job& j, RE::bhkWorld* a_bw, float a_x, float a_y, float a_floorZ, float a_ux, float a_uy)
    {
        ExState&    x = *j.x;
        const float px = -a_uy * x.rc, py = a_ux * x.rc;
        const bool  side0 = x.doorSide;
        x.doorSide = true;
        bool blocked = false;
        for (const float sg : { 1.f, -1.f }) {
            const float cx = a_x + sg * px, cy = a_y + sg * py;
            if (XRayBlocked(j, a_bw, RE::NiPoint3{ cx, cy, a_floorZ + x.riser + kVColumnPad }, RE::NiPoint3{ cx, cy, a_floorZ + x.htop }, j.xs.rK)) {
                blocked = true;
                x.besideSide = sg > 0.f ? 1 : -1;     // (1.6.0 §4 #7) her left (+1) or her right (-1)
                break;
            }
        }
        x.doorSide = side0;
        return blocked;
    }

    // C7 (review R1 #2): the headroom over a floor point - from the floor + 8 s up to the floor + Htop (her own bound top + 2).
    // A refusal with a doorway-like clearance (>= 0.6 Htop: a lintel, a beam, a low ceiling - not a table) is logged once per
    // 64 u spot, at most kVHeadLogMax lines a plan, with the clearance MEASURED there - the VR log then shows real door heights.
    // true = blocked.
    bool HeadBlocked(Job& j, RE::bhkWorld* a_bw, float a_x, float a_y, float a_floorZ, const char* a_where)
    {
        ExState&           x = *j.x;
        const RE::NiPoint3 a{ a_x, a_y, a_floorZ + kXHeadFoot * j.scale }, b{ a_x, a_y, a_floorZ + x.htop };
        float              frac = 1.f;
        if (!XRayBlocked(j, a_bw, a, b, j.xs.rR, &frac)) return false;
        ++j.xs.headFails;
        const float clear = a.z + (b.z - a.z) * frac - a_floorZ;
        if (clear >= kVHeadLogFrac * x.htop && x.headLogged < kVHeadLogMax) {
            const auto          q = [](float a_v) { return static_cast<std::uint64_t>(static_cast<std::int64_t>(std::floor(a_v / 64.f)) & 0xFFFFF); };
            const std::uint64_t key = (q(a_x) << 40) | (q(a_y) << 20) | q(a_floorZ);
            if (std::find(x.headSpots.begin(), x.headSpots.end(), key) == x.headSpots.end()) {
                x.headSpots.push_back(key);
                ++x.headLogged;
                logger::info("PATH {} headroom: ({:.0f},{:.0f},{:.0f}) {} has only {:.0f} u of clearance over the floor - her bound top needs {:.0f} u "
                             "(a lintel, a beam or a low ceiling)",
                             j.handle ? std::format("#{}", j.handle) : std::string("probe"), a_x, a_y, a_floorZ, a_where, clear, x.htop);
            }
        }
        return true;
    }

    // 1.5.0 her body along a straight piece a -> b (both on the floor; a_zMid = the floor at its middle): 0 clear; 1 blocked on
    // her centre line (a standing actor's disc, the knee, the chest, the two diagonals 16 <-> 112 u; with a_midHead the headroom
    // at the middle); 2 blocked only BESIDE her - C5 the knee rays at +-rc along the piece; (review R1 #3) the CLEARANCE probes:
    // 4 short rays from her centre line at the middle straight out to +-rc at knee and chest height, which cross a wall, a counter
    // or a bench running PARALLEL to her line closer than rc (a ray along it never would); with a_columns C6 the side columns
    // at the middle - such a step may be retried aside (C11). 10-13 rays. (The line test casts its side columns every ~32 u
    // itself, after the piece's centre rays.) The rays beside her pass only a door her centre line passed (R1 #1).
    int XBody(Job& j, RE::bhkWorld* a_bw, const RE::NiPoint3& a_a, const RE::NiPoint3& a_b, float a_zMid, bool a_midHead, bool a_columns = true)
    {
        ExState&    x = *j.x;
        const float s = j.scale;
        const float knee = kKnee * s, chest = kChest * s, lo = kSweepLow * s, hi = kSweepHigh * s;
        if (CrossesActorDisc(x, a_a, a_b)) return 1;
        if (XRayBlocked(j, a_bw, RE::NiPoint3{ a_a.x, a_a.y, a_a.z + knee }, RE::NiPoint3{ a_b.x, a_b.y, a_b.z + knee }, j.xs.rO)) return 1;
        if (XRayBlocked(j, a_bw, RE::NiPoint3{ a_a.x, a_a.y, a_a.z + chest }, RE::NiPoint3{ a_b.x, a_b.y, a_b.z + chest }, j.xs.rO)) return 1;
        if (XRayBlocked(j, a_bw, RE::NiPoint3{ a_a.x, a_a.y, a_a.z + lo }, RE::NiPoint3{ a_b.x, a_b.y, a_b.z + hi }, j.xs.rO)) return 1;
        if (XRayBlocked(j, a_bw, RE::NiPoint3{ a_a.x, a_a.y, a_a.z + hi }, RE::NiPoint3{ a_b.x, a_b.y, a_b.z + lo }, j.xs.rO)) return 1;
        const float mx = (a_a.x + a_b.x) * 0.5f, my = (a_a.y + a_b.y) * 0.5f;
        if (a_midHead && HeadBlocked(j, a_bw, mx, my, a_zMid, "on her line")) return 1;
        const float dx = a_b.x - a_a.x, dy = a_b.y - a_a.y;
        const float len = std::sqrt(dx * dx + dy * dy);
        if (len < 1.f) return 0;                    // no direction: nothing "beside" her to test
        const float px = -dy / len * x.rc, py = dx / len * x.rc;
        const float mz = (a_a.z + a_b.z) * 0.5f;    // the centre line's own height at the middle (the rays above ran from it)
        const bool  side0 = x.doorSide;
        x.doorSide = true;
        int side = 0;
        x.besideSide = 0;                           // (1.6.0 §4 #7) which side blocked: its side-steps go the other way
        for (const float sg : { 1.f, -1.f }) {
            if (XRayBlocked(j, a_bw, RE::NiPoint3{ a_a.x + sg * px, a_a.y + sg * py, a_a.z + knee },
                            RE::NiPoint3{ a_b.x + sg * px, a_b.y + sg * py, a_b.z + knee }, j.xs.rO)) {
                side = 2;
                x.besideSide = sg > 0.f ? 1 : -1;
                break;
            }
        }
        for (int k = 0; k < 4 && side == 0; ++k) {     // the clearance probes: knee / chest x left / right
            const float h = (k < 2 ? knee : chest), sg = (k % 2) == 0 ? 1.f : -1.f;
            if (XRayBlocked(j, a_bw, RE::NiPoint3{ mx, my, mz + h }, RE::NiPoint3{ mx + sg * px, my + sg * py, mz + h }, j.xs.rO)) {
                side = 2;
                x.besideSide = sg > 0.f ? 1 : -1;
            }
        }
        x.doorSide = side0;
        if (side) return side;
        if (a_columns && XSideColumns(j, a_bw, mx, my, a_zMid, dx / len, dy / len)) return 2;
        return 0;
    }

    // (review R1 #3 #4) a route cell's own clearance, N / E / S / W at rc: the floor there within a riser of its own (no brink
    // under her shoulder, no rise beside her - the letter-18 lip) and a knee-height probe from the cell out to it (a wall or a
    // post closer than rc: her body would overlap it at a turn). A passable closed door is seen THROUGH by these probes and
    // never recorded (the cell in front of a door stays usable; only her centre line passes a door). 8 rays. true = fine.
    // (review fix #1) the terrain allowance may accept a LAND floor beside her more than her terrain riser away (up to rc x tan
    // 63 + 8 = 55 u) only when ONE more floor ray halfway out (a_hx, a_hy) finds land too and NEITHER half changes height by more
    // than (a_off / 2) x tan 63 + 8 u (31.6 u at rc 24 - the allowance's own formula over half the offset): a 63 deg hillside
    // passes (23.6 u a half), the foot of a sheer 35-55 u drop beside her (a retaining wall, a rock step, a lake bank) does not -
    // its whole height falls in one half. The 1.5.0 lip fix (R1 #4) holds on hillsides. One ray, only where it is needed.
    bool HalfwayLand(Job& j, RE::bhkWorld* a_bw, float a_hx, float a_hy, float a_z0, float a_zSide, float a_off, float a_window, int& a_bucket)
    {
        const FloorHit h = XFloorRay(j, a_bw, a_hx, a_hy, a_z0 + a_window + kXTopPad, a_z0 - a_window - 1.f, a_bucket);
        if (h.blocked || !h.found || !IsTerrainLayer(h.layer)) return false;
        const float lim = TerrainRise(a_off * 0.5f);
        return std::fabs(h.z - a_z0) <= lim && std::fabs(a_zSide - h.z) <= lim;
    }

    // (1.5.1) a_terr: the cell is terrain on the navmesh - a side floor that is terrain too may lie up to her TERRAIN riser from it,
    // and up to rc x tan 63 + 8 u (a hillside, not a lip) when the floor halfway out confirms it (review fix #1, HalfwayLand); the
    // window reaches that far. Every other floor keeps 1.5.0's riser (review fix #2).
    // (1.6.0 §5) a_stair: the cell lies on a stair (the stair allowance) - a fixed floor beside it may lie up to rc x tan(acos 0.66)
    // + 4 u (31 u at rc 24) off it (the stair's own slope over rc: the along-slope tolerance), and its knee probes pass the stair's
    // own sloped face
    bool XCellClear(Job& j, RE::bhkWorld* a_bw, float a_x, float a_y, float a_z, bool a_terr = false, bool a_stair = false)
    {
        ExState&        x = *j.x;
        constexpr float kSides[4][2] = { { 0.f, 1.f }, { 1.f, 0.f }, { 0.f, -1.f }, { -1.f, 0.f } };
        const float     sRise = a_stair ? (std::max)(x.riser, StairRise(x.rc)) : x.riser;
        const float     tRise = (std::max)(a_terr ? (std::max)(x.riserT, TerrainRise(x.rc)) : x.riser, sRise);
        for (const auto& d : kSides) {
            const FloorHit h = XFloorRay(j, a_bw, a_x + d[0] * x.rc, a_y + d[1] * x.rc, a_z + tRise + kXTopPad, a_z - tRise - 1.f, j.xs.rS);
            if (h.blocked || !h.found) return false;
            const float dz = std::fabs(h.z - a_z);
            if (dz <= x.riser) continue;
            if (a_stair && IsStairLayer(h.layer) && dz <= sRise) continue;    // (1.6.0) the stair itself beside a stair cell
            if (!a_terr || !IsTerrainLayer(h.layer) || dz > (std::max)(x.riserT, TerrainRise(x.rc))) return false;
            if (dz <= x.riserT) continue;         // land beside land within her terrain riser: a step she takes there
            if (!HalfwayLand(j, a_bw, a_x + d[0] * x.rc * 0.5f, a_y + d[1] * x.rc * 0.5f, a_z, h.z, x.rc, tRise, j.xs.rS)) return false;
        }
        const float knee = kKnee * j.scale;
        const bool  peek0 = x.doorPeek, terrSide0 = x.terrSide, stairSide0 = x.stairSide;
        x.doorPeek = true;
        x.terrSide = a_terr;                      // (1.5.1) the hillside beside a terrain cell is no wall
        x.stairSide = a_stair;                    // (1.6.0) nor the stair's own face beside a stair cell
        bool clear = true;
        for (const auto& d : kSides) {
            if (XRayBlocked(j, a_bw, RE::NiPoint3{ a_x, a_y, a_z + knee }, RE::NiPoint3{ a_x + d[0] * x.rc, a_y + d[1] * x.rc, a_z + knee }, j.xs.rO)) {
                clear = false;
                break;
            }
        }
        x.doorPeek = peek0;
        x.terrSide = terrSide0;
        x.stairSide = stairSide0;
        return clear;
    }

    // (review R1 #4) the line test's brink pair at the middle of a step: the floor at +-rc across it within a riser. 2 floor rays.
    // (1.5.1) a_terr: a terrain step on the navmesh - a terrain side floor may lie up to her terrain riser from the middle, and up
    // to rc x tan 63 + 8 u when the floor halfway out confirms a hillside (review fix #1).
    // (1.6.0 §5) a_stair: a stair step - a fixed floor beside her may lie up to StairRise(rc) off the middle (the stair's own slope);
    // (§4 #7) a refused side is remembered (x.besideSide) so a side-step goes the other way
    bool XBrinkClear(Job& j, RE::bhkWorld* a_bw, const RE::NiPoint3& a_a, const RE::NiPoint3& a_b, float a_zMid, bool a_terr = false,
                     bool a_stair = false)
    {
        ExState&    x = *j.x;
        const float dx = a_b.x - a_a.x, dy = a_b.y - a_a.y;
        const float len = std::sqrt(dx * dx + dy * dy);
        if (len < 1.f) return true;
        const float px = -dy / len * x.rc, py = dx / len * x.rc;
        const float mx = (a_a.x + a_b.x) * 0.5f, my = (a_a.y + a_b.y) * 0.5f;
        const float terrRise = a_terr ? (std::max)(x.riserT, TerrainRise(x.rc)) : x.riser;
        const float sRise = a_stair ? (std::max)(x.riser, StairRise(x.rc)) : x.riser;
        const float tRise = (std::max)(terrRise, sRise);
        for (const float sg : { 1.f, -1.f }) {
            const FloorHit h = XFloorRay(j, a_bw, mx + sg * px, my + sg * py, a_zMid + tRise + kXTopPad, a_zMid - tRise - 1.f, j.xs.rS);
            const int      side = sg > 0.f ? 1 : -1;
            if (h.blocked || !h.found) {
                x.besideSide = side;
                return false;
            }
            const float dz = std::fabs(h.z - a_zMid);
            if (dz <= x.riser) continue;
            if (a_stair && IsStairLayer(h.layer) && dz <= sRise) {     // (1.6.0) the stair itself beside her
                x.stairUsed = true;
                continue;
            }
            if (!(a_terr && IsTerrainLayer(h.layer) && dz <= terrRise)) {
                x.besideSide = side;
                return false;
            }
            if (dz > x.riserT && !HalfwayLand(j, a_bw, mx + sg * px * 0.5f, my + sg * py * 0.5f, a_zMid, h.z, x.rc, terrRise, j.xs.rS)) {
                x.besideSide = side;
                return false;
            }
            x.terrUsed = true;
        }
        return true;
    }

    // 1.5.0 C1/C3: the floor of the step p -> m at 1/3 and 2/3 - each sample a floor (n.z >= 0.70) within one riser of the one
    // before it (p, then the first sample), not a bad spot, the water over it within a_lim (none on a dry-only plan), and m
    // within one riser of the second. a_zMid = the floor at the step's middle. 4 questions (+ step-overs).
    // (1.5.1) a_terr: p and m are both terrain on the navmesh - a sample that is terrain / ground too may be as steep as n.z
    // >= 0.45 and rise up to max(her TERRAIN riser, (a third of the step) x tan 63 + 8 u) from the one before it (when that one
    // was terrain too; review fix #2: between any other floors 1.5.0's riser); the samples are kept for the cell cache (x.profZ /
    // x.profD) and a failure on THIS plan's own rules (a bad spot, the water limit, an unloaded cell) is marked (x.planFail: never
    // cached). The rays' window is 1.5.0's (her riser + 16 >= 40 u covers every terrain rise <= 40 u).
    bool NavUnder(Job& j, RE::Actor* a_w, float a_x, float a_y, float a_z);     // (defined below)

    // (1.6.0 §5) a sample steeper than 0.70 that is not terrain passes on the STAIR ALLOWANCE (a fixed floor, n.z >= 0.66) when the
    // navmesh lies under it (one navmesh question, only for such a sample): x.stairUsed
    bool XProfile(Job& j, RE::Actor* a_w, RE::bhkWorld* a_bw, const RE::NiPoint3& a_p, const RE::NiPoint3& a_m, float a_lim, float& a_zMid,
                  bool a_terr = false)
    {
        ExState&    x = *j.x;
        float       zPrev = a_p.z, z1 = a_p.z;
        const float tRise = a_terr ? (std::max)(x.riserT, TerrainRise(Dist2D(a_p, a_m) / 3.f)) : x.riser;
        bool        prevTerr = a_terr;
        for (int k = 1; k <= 2; ++k) {
            const float    t = static_cast<float>(k) / 3.f;
            const float    sx = a_p.x + (a_m.x - a_p.x) * t, sy = a_p.y + (a_m.y - a_p.y) * t;
            const FloorHit h = XFloorRay(j, a_bw, sx, sy, zPrev + x.riser + kXTopPad, zPrev - x.riser - kXTopPad, j.xs.rM);
            if (h.blocked || !h.found) return false;
            const bool  terr = a_terr && IsTerrainLayer(h.layer);
            const float rise = std::fabs(h.z - zPrev);
            if (rise > x.riser) {
                if (!(terr && prevTerr && rise <= tRise)) return false;
                x.terrUsed = true;
            }
            if (h.nz < kXMinNz) {
                if (terr && h.nz >= kVTerrainNz) {
                    x.terrUsed = true;
                } else if (StairSlope(h.nz, h.layer) && NavUnder(j, a_w, sx, sy, h.z)) {
                    x.stairUsed = true;           // (1.6.0) the stair allowance
                } else {
                    return false;
                }
            }
            if (IsBadSpot(j, sx, sy, h.z)) {
                x.planFail = true;
                return false;
            }
            float depth = 0.f, surf = 0.f;
            if (!DepthAt(j, a_w, sx, sy, h.z, depth, surf)) {
                x.planFail = true;
                return false;
            }
            NoteWater(j, h, surf);
            if (depth > kXWet && (!(j.xflags & kWater) || depth > a_lim)) {
                x.planFail = true;
                return false;
            }
            x.profZ[static_cast<std::size_t>(k - 1)] = h.z;
            x.profD[static_cast<std::size_t>(k - 1)] = depth;
            if (k == 1) z1 = h.z;
            zPrev = h.z;
            prevTerr = terr;
        }
        a_zMid = (z1 + zPrev) * 0.5f;
        const float endRise = std::fabs(a_m.z - zPrev);
        if (endRise <= x.riser) return true;
        if (!(a_terr && prevTerr && endRise <= tRise)) return false;
        x.terrUsed = true;
        return true;
    }

    // 1.5.1 is the navmesh under this floor point (within 19 u of it, and its height within one step of the floor)? One
    // navmesh question - asked only where the terrain allowance is needed. The classification asks the same (FloorNav).
    bool NavUnder(Job& j, RE::Actor* a_w, float a_x, float a_y, float a_z)
    {
        const ExState& x = *j.x;
        RE::NiPoint3   got{};
        return SnapNav(j, a_w, a_x, a_y, a_z, kLevelBand, got) && std::hypot(got.x - a_x, got.y - a_y) <= kCell * 0.6f &&
               std::fabs(got.z - a_z) <= x.step;
    }

    // her scale, radius and ray filter - Start's own lines, for StartEx and the probes
    void FillWalker(Job& j, RE::Actor* a_w)
    {
        j.walker = a_w->GetFormID();
        j.scale = (std::max)(0.5f, a_w->GetScale());
        float         radius = 20.f;
        std::uint32_t group = 0;
        if (auto* cc = a_w->GetCharController()) {
            const float r = cc->collisionBound.extents.x;
            if (r > 4.f && r < 96.f) radius = r;
            if (auto* rb = cc->GetRigidBody()) group = rb->collidable.broadPhaseHandle.collisionFilterInfo.filter >> 16;
            const float hz = cc->collisionBound.extents.z;           // 1.5.0 C7: her bound's half height and its TOP (centre z +
            const float top = cc->collisionBound.center.z + hz;      // half height; checked against H in MeasureV)
            if (std::isfinite(hz) && hz > 8.f && hz < 400.f) j.halfZ = hz;
            if (std::isfinite(top) && top > 16.f && top < 800.f) j.boundTop = top;
        }
        j.radius = radius;
        j.rayFilter = 30u | (group << 16);       // her own controller's layer and group: the rays skip her own body
    }

    // B §2: H = GetHeight() - the engine's own submerge divisor, so our depth cap and its swim rule speak one unit -
    // accepted 48-400 u, else 128 x s; step = clamp(0.30 H, 24, 40); depthCap = depthFrac (0.05-0.65, <= 0 -> 0.45) x H.
    // Main thread (GetHeight touches her process's cached height).
    void MeasureH(Job& j, RE::Actor* a_w, float& a_H, float& a_raw, float& a_step)
    {
        a_raw = a_w->GetHeight();
        a_H = (a_raw >= kXHMin && a_raw <= kXHMax) ? a_raw : kXHDefault * j.scale;
        a_step = std::clamp(kXStepFrac * a_H, kXStepMin, kXStepMax);
        j.depthFrac = std::clamp(j.depthFrac > 0.f ? j.depthFrac : kXDepthFracDef, kXDepthFracMin, kXDepthFracMax);
        j.height = a_H;
        j.depthCap = j.depthFrac * a_H;
    }

    // 1.4.1: the engine's own submerge divisor for her - (boundMax.z - boundMin.z) x GetScale(), Actor::GetSubmergedLevel's
    // (36452) - and the swim depth = 0.70 x it. Outside 48-400 u: H. Main thread (after MeasureH).
    void MeasureSwim(ExState& x, RE::Actor* a_w)
    {
        const RE::NiPoint3 lo = a_w->GetBoundMin();
        const RE::NiPoint3 hi = a_w->GetBoundMax();
        const float        div = (hi.z - lo.z) * a_w->GetScale();
        x.swimDiv = (std::isfinite(div) && div >= kXHMin && div <= kXHMax) ? div : x.H;
        x.swimDepth = kXSwimLevel * x.swimDiv;
    }

    // 1.5.0 her body numbers (after MeasureH): C1 riser = 26 u x scale for s > 1 (never under 24) - also every "one step"
    // rule (x.step); C5 rc = collisionBound.extents.x + 4; C7 (review R1 #2) Htop = the TOP of her own controller bound
    // (collisionBound.center.z + extents.z) + 2 u - a top outside 0.6-1.6 H is not trusted: then H + 2. Main thread.
    // (1.5.1, spec §4 + the session's decision on review fix #2) TWO risers: x.riser = 1.5.0's (26 u x scale for s > 1, never
    // under 24) on EVERY floor - STATIC, furniture, clutter, anything off the navmesh: a 30 u bench or a 31 u chair seat is never
    // a step - and also every "one step" rule (x.step: the navmesh-on-this-floor tolerance, so a bench top never counts as on the
    // navmesh); x.riserT = the TERRAIN riser from HER BODY, clamp(0.9 x her hull's cone, 24, 40) u (j.cone 8-120 u; the hull
    // already carries her scale - the 1.5.0 VR log: 38.0 u -> 34 u), used only between land floors on the navmesh (the terrain
    // allowance's own test); never under x.riser; cone not read -> x.riser.
    void MeasureV(Job& j, ExState& x)
    {
        x.v15 = true;
        x.riserCone = (j.cone >= kVConeMin && j.cone <= kVConeMax) ? j.cone : 0.f;
        x.riser = (std::max)(kVRiserMin, kVRiser * (std::max)(1.f, j.scale));
        x.riserT = x.riserCone > 0.f ? (std::max)(x.riser, std::clamp(kVConeFrac * x.riserCone, kVRiserMin, kVRiserMax)) : x.riser;
        x.step = x.riser;
        x.rc = j.radius + kVClearPad;
        x.htopFromBound = j.boundTop >= 0.6f * x.H && j.boundTop <= 1.6f * x.H;
        x.htop = (x.htopFromBound ? j.boundTop : x.H) + kVHeadPad;
        j.riserUsed = x.riser;
        j.riserTUsed = x.riserT;
    }

    // 1.5.1 (g_mtx held) a cone read now is remembered for her; a failed read (0 / out of 8-120 u / not a humanoid hull) takes her
    // last good one (review #9: and says so in the log)
    void ConeMemo(Job& j)
    {
        j.coneRemembered = false;
        if (j.cone >= kVConeMin && j.cone <= kVConeMax) {
            g_cone[j.walker] = j.cone;
        } else if (const auto it = g_cone.find(j.walker); it != g_cone.end()) {
            j.cone = it->second;
            j.coneRemembered = true;
        }
    }

    // 1.5.1 both risers and where each comes from, for the measured, the calibration and the PATH lines
    std::string RiserNote(const Job& j, const ExState& x)
    {
        return std::format("riser {:.0f} u (26 u x scale {:.2f}, >= 24: every floor) / terrain riser {:.0f} u ({}: land on the navmesh only)",
                           x.riser, j.scale, x.riserT,
                           x.riserCone > 0.f ? std::format("0.9 x her hull cone {:.1f} u{}, 24-40", x.riserCone, j.coneRemembered ? " (remembered)" : "") :
                                               std::string("her hull cone not read: = the riser"));
    }

    // 1.5.1 her character-controller hull, read only (1.5.0's Calibrate pointers, moved here so the riser can use the cone on
    // EVERY 1.5.0-flag plan and probe - the line itself stays once per walker per run): controller -> rigid body -> its list
    // shape -> the convex hull; the cone = the bottom ring's height over the lowest point (the slope that lifts her over an
    // edge). a_text: the CALIBRATION line's words. 0 = not read. Main thread.
    float ReadHull(RE::Actor* a_w, std::string* a_text)
    {
        constexpr float kU = 1.f / kToHavok;      // Havok metres -> game units
        float           cone = 0.f;
        if (a_text) *a_text = "hull: not read";
        auto* cc = a_w ? a_w->GetCharController() : nullptr;
        if (!cc) return 0.f;
        const RE::hkpShape* top = nullptr;
        if (auto* rb = cc->GetRigidBody()) top = rb->collidable.shape;
        const RE::hkpConvexVerticesShape* cv = nullptr;
        if (top && top->type == RE::hkpShapeType::kConvexVertices) {
            cv = static_cast<const RE::hkpConvexVerticesShape*>(top);
        } else if (top && top->type == RE::hkpShapeType::kList) {
            const auto*        list = static_cast<const RE::hkpListShape*>(top);
            const std::int32_t cn = list->childInfo.size();
            for (std::int32_t i = 0; i < cn && i < 8 && !cv; ++i) {
                const RE::hkpShape* ch = list->childInfo[i].shape;
                if (ch && ch->type == RE::hkpShapeType::kConvexVertices) cv = static_cast<const RE::hkpConvexVerticesShape*>(ch);
            }
        }
        if (cv) {
            const std::int32_t nv = cv->numVertices;
            const std::int32_t blocks = cv->rotatedVertices.size();
            if (nv >= 4 && nv <= 64 && blocks >= (nv + 3) / 4 && cv->rotatedVertices.data()) {
                std::vector<RE::NiPoint3> v;
                v.reserve(static_cast<std::size_t>(nv));
                for (std::int32_t i = 0; i < nv; ++i) {
                    const auto&       b = cv->rotatedVertices[i / 4];
                    alignas(16) float bx[4]{}, by[4]{}, bz[4]{};
                    _mm_store_ps(bx, b.x.quad);
                    _mm_store_ps(by, b.y.quad);
                    _mm_store_ps(bz, b.z.quad);
                    v.emplace_back(bx[i % 4] * kU, by[i % 4] * kU, bz[i % 4] * kU);
                }
                std::sort(v.begin(), v.end(), [](const RE::NiPoint3& l, const RE::NiPoint3& r) { return l.z < r.z; });
                // a column with a pointed cone under it: the lowest point, then the bottom ring (B §1.1: 1 + 8 + 8 + 1)
                const RE::NiPoint3 bot = v.front();
                const std::size_t  ringN = (std::min)(static_cast<std::size_t>(8), v.size() - 1);
                float              ringZ = 0.f, ringR = 0.f, ringLo = 3.0e38f, ringHi = -3.0e38f;
                for (std::size_t i = 1; i <= ringN; ++i) {
                    ringZ += v[i].z;
                    ringR += std::hypot(v[i].x - bot.x, v[i].y - bot.y);
                    ringLo = (std::min)(ringLo, v[i].z);
                    ringHi = (std::max)(ringHi, v[i].z);
                }
                ringZ /= static_cast<float>(ringN);
                ringR /= static_cast<float>(ringN);
                // (review #9) only a humanoid controller hull gives her step: ONE lowest point under a FLAT ring of 8 (1 + 8 + 8 + 1);
                // any other shape (a creature's, a horse's) is logged but not used for the riser
                const bool humanoid = nv >= 10 && ringN == 8 && ringHi - ringLo <= kVConeRingFlat && ringLo - bot.z > 1.f;
                cone = humanoid ? ringZ - bot.z : 0.f;
                if (!std::isfinite(cone)) cone = 0.f;
                if (a_text) {
                    *a_text = std::format("hull: {} vertices, lowest point z {:.1f}, bottom ring z {:.1f} (spread {:.1f} u) -> CONE {:.1f} u (her step){}, "
                                          "ring radius {:.1f} u, top point z {:.1f}, convex radius {:.1f} u",
                                          nv, bot.z, ringZ, ringHi - ringLo, ringZ - bot.z,
                                          humanoid ? "" : " - NOT a humanoid hull (one lowest point, a flat ring of 8): not used for the riser", ringR,
                                          v.back().z, cv->radius * kU);
                }
            } else if (a_text) {
                *a_text = std::format("hull: a convex shape with {} vertices in {} blocks - not read", nv, blocks);
            }
        } else if (top && a_text) {
            *a_text = std::format("hull: none found (top shape type {})", static_cast<int>(top->type));
        }
        return cone;
    }

    // 1.5.0 C17 / B §1.4: ONE calibration line per walker per game run - what her collision body really is, read only (the
    // same pointers TestCmd's close-contact program walks in VR: controller -> rigid body -> its list shape -> the convex hull;
    // the proxy's tunables the way FollowerBumpGuard reads them). Nothing is written. Main thread, g_mtx held (Step).
    // (1.5.1: the hull is read by ReadHull; the line adds the riser chosen from it)
    void Calibrate(const Job& j, RE::Actor* a_w, const ExState& x)
    {
        if (!g_calibrated.insert(j.walker).second) return;
        auto* cc = a_w->GetCharController();
        if (!cc) {
            logger::info("PATH CALIBRATION {:08X}: no character controller - nothing read", j.walker);
            return;
        }
        constexpr float kU = 1.f / kToHavok;      // Havok metres -> game units
        const auto&     cb = cc->collisionBound;
        std::string     hull;
        ReadHull(a_w, &hull);
        // the slope limit and the soft shell: the proxy controller (the player) or the rigid-body one (NPCs) - each read only
        // after RTTI says what it is; the rigid body's character object only when its embedded wrapper carries the game's
        // own vtable (so a wrong offset can never be followed)
        std::string ctl = "controller: neither proxy nor rigid body (RTTI) - slope limit not read";
        if (auto* pxc = skyrim_cast<RE::bhkCharProxyController*>(cc)) {
            if (const auto* px = pxc->GetCharacterProxy()) {
                ctl = std::format("proxy controller: keepDistance {:.1f} u, maxSlopeCosine {:.3f}", px->keepDistance * kU, px->maxSlopeCosine);
            } else {
                ctl = "proxy controller without a proxy";
            }
        } else if (auto* rbc = skyrim_cast<RE::bhkCharRigidBodyController*>(cc)) {
            const auto vt = *reinterpret_cast<const std::uintptr_t*>(&rbc->charRigidBody);
            if (vt == RE::VTABLE_bhkCharacterRigidBody[0].address()) {
                const auto* crb = static_cast<const RE::hkpCharacterRigidBody*>(rbc->charRigidBody.referencedObject.get());
                const auto  vt2 = crb ? *reinterpret_cast<const std::uintptr_t*>(crb) : 0;
                if (crb && vt2 == RE::VTABLE_hkpCharacterRigidBody[0].address()) {
                    ctl = std::format("rigid-body controller: maxSlopeCosine {:.3f}, supportDistance {:.1f} u, hardSupportDistance {:.1f} u "
                                      "(no keepDistance: a proxy field)",
                                      crb->maxSlopeCosine, crb->supportDistance * kU, crb->hardSupportDistance * kU);
                } else {
                    ctl = "rigid-body controller: its character body did not carry the expected vtable - not read";
                }
            } else {
                ctl = "rigid-body controller: the wrapper did not carry the expected vtable - not read";
            }
        }
        logger::info("PATH CALIBRATION {:08X} (once per game run, read only): GetHeight {:.1f} u, scale {:.2f} (controller {:.2f}), "
                     "collisionBound centre ({:.1f},{:.1f},{:.1f}) half-extents ({:.1f},{:.1f},{:.1f}) u | {} | {} | the planner uses: {}, "
                     "clearance {:.0f} u, headroom {:.0f} u ({}), slope {:.2f} / stand {:.2f}; terrain on the navmesh {:.2f}, rise <= h x tan 63 + {:.0f} u",
                     j.walker, x.Hraw, a_w->GetScale(), cc->scale, cb.center.x, cb.center.y, cb.center.z, cb.extents.x, cb.extents.y,
                     cb.extents.z, hull, ctl, RiserNote(j, x), x.rc, x.htop,
                     x.htopFromBound ? "her bound top + 2" : "H + 2: the bound top not trusted", kXMinNz, kXStandNz, kVTerrainNz, kVTerrainPad);
    }

    // The pull's free first test (no questions): every 16 u sample of a -> b lies in a cell the search already classified
    // walkable (NAV / FLOOR / WATER; a navmesh leg: dry NAV only), with no step over the limit between neighbouring samples'
    // cells. Samples within one cell of either end are not looked up (the ends are known points). No grid: passes.
    // 1.5.1 C2, the cell rule between two floor points a_L u apart: one riser + a_L; a_terr (both terrain on the navmesh): her
    // TERRAIN riser + a_L, or a_L x tan 63 deg + 8 u when that is more
    float CellRise(const ExState& x, float a_L, bool a_terr)
    {
        return a_terr ? (std::max)(x.riserT + a_L, TerrainRise(a_L)) : x.riser + a_L;
    }

    // 1.5.0: Precheck with several floors per square - each sample takes the level of its square nearest the line's own
    // height (none within a riser + a cell of it = unexplored there), and neighbouring samples' levels obey the cell rule
    // (C2: one riser + their distance). (1.5.1: the cell rule with the terrain allowance between two terrain cells)
    bool PrecheckV(const Job& j, const RE::NiPoint3& a_a, const RE::NiPoint3& a_b, bool a_navOnly, float& a_deepest)
    {
        const ExState& x = *j.x;
        a_deepest = 0.f;
        if (x.nodes.empty()) return true;
        const float  len = Dist2D(a_a, a_b);
        const int    n = (std::max)(1, static_cast<int>(std::ceil(len / kXSample)));
        std::int32_t prev = -1;
        for (int k = 0; k <= n; ++k) {
            const float t = static_cast<float>(k) / static_cast<float>(n);
            const float along = t * len;
            if (along < kCell || len - along < kCell) continue;
            const float px = a_a.x + (a_b.x - a_a.x) * t, py = a_a.y + (a_b.y - a_a.y) * t, pz = a_a.z + (a_b.z - a_a.z) * t;
            const int   ix = static_cast<int>(std::lround((px - x.minX) / kCell));
            const int   iy = static_cast<int>(std::lround((py - x.minY) / kCell));
            if (ix < 0 || iy < 0 || ix >= x.nx || iy >= x.ny) return false;
            std::int32_t best = -1;
            float        bestDz = 3.0e38f;
            for (std::int32_t lv = iy * x.nx + ix; lv >= 0; lv = x.nodes[static_cast<std::size_t>(lv)].next) {
                const XNode& c = x.nodes[static_cast<std::size_t>(lv)];
                if (!c.hasZ) continue;
                const float dz = std::fabs(c.z - pz);
                if (dz < bestDz) {
                    bestDz = dz;
                    best = lv;
                }
            }
            if (best < 0 || bestDz > x.riser + kCell) return false;
            const XNode& c = x.nodes[static_cast<std::size_t>(best)];
            if (a_navOnly ? c.kind != kKNav : (c.kind < kKNav || c.kind > kKWater)) return false;
            if (prev >= 0 && prev != best) {
                const XNode& pc = x.nodes[static_cast<std::size_t>(prev)];
                if (std::fabs(c.z - pc.z) > CellRise(x, (std::max)(kCell, std::hypot(c.x - pc.x, c.y - pc.y)), c.navTerr && pc.navTerr)) return false;
            }
            a_deepest = (std::max)(a_deepest, c.depth);
            prev = best;
        }
        return true;
    }

    bool Precheck(const Job& j, const RE::NiPoint3& a_a, const RE::NiPoint3& a_b, bool a_navOnly, float& a_deepest)
    {
        const ExState& x = *j.x;
        if (x.v15) return PrecheckV(j, a_a, a_b, a_navOnly, a_deepest);
        a_deepest = 0.f;
        if (x.nodes.empty()) return true;
        const float  len = Dist2D(a_a, a_b);
        const int    n = (std::max)(1, static_cast<int>(std::ceil(len / kXSample)));
        std::int32_t prev = -1;
        for (int k = 0; k <= n; ++k) {
            const float t = static_cast<float>(k) / static_cast<float>(n);
            const float along = t * len;
            if (along < kCell || len - along < kCell) continue;
            const float px = a_a.x + (a_b.x - a_a.x) * t, py = a_a.y + (a_b.y - a_a.y) * t;
            const int   ix = static_cast<int>(std::lround((px - x.minX) / kCell));
            const int   iy = static_cast<int>(std::lround((py - x.minY) / kCell));
            if (ix < 0 || iy < 0 || ix >= x.nx || iy >= x.ny) return false;
            const std::int32_t i = iy * x.nx + ix;
            const XNode&       c = x.nodes[static_cast<std::size_t>(i)];
            if (a_navOnly ? c.kind != kKNav : (c.kind < kKNav || c.kind > kKWater)) return false;
            if (prev >= 0 && prev != i) {
                const XNode& pc = x.nodes[static_cast<std::size_t>(prev)];
                if (std::fabs(c.z - pc.z) > x.step) return false;      // physics floors both (review G: one step rule)
            }
            a_deepest = (std::max)(a_deepest, c.depth);
            prev = i;
        }
        return true;
    }

    // Start a line test a -> b (both on the floor). a_nav: a navmesh leg (1.3.0's SegmentClear(sample = true) per <= 64 u
    // piece, + dry); else the floor rules (B §7.4.3). a_precheck: the grid pre-check first, and the depth limit becomes
    // min(depthCap, the deepest cell it replaces + 16). (1.4.1) a_fromDeep: a starts in her start region - the line may
    // use water up to x.startLimit until its first sample at or under the cap; from there on the cap rules (it never
    // re-enters deeper water: the cell rule of the search).
    // (1.5.0, x.v15 floor rules: samples every <= ~11 u, side rays at +-rc, the standing actors' discs, a_doors = a closed
    // unlocked ordinary door may be passed (kDoors), a_capture = keep the first blocking hit (a navmesh leg's CARPET line))
    void SegBegin(Job& j, const RE::NiPoint3& a_a, const RE::NiPoint3& a_b, bool a_nav, bool a_precheck, float a_endDepth, bool a_sides = true,
                  bool a_fromDeep = false, bool a_doors = false, bool a_capture = false)
    {
        ExState& x = *j.x;
        SegTest& s = x.seg;
        s = SegTest{};
        s.active = true;
        s.nav = a_nav;
        s.sides = a_sides;
        s.a = a_a;
        s.b = a_b;
        s.last = a_a;
        s.pieceA = a_a;
        s.deepRun = a_fromDeep && x.deepStart && !a_nav;
        s.v15 = x.v15 && !a_nav;
        s.doorsOk = s.v15 && a_doors;
        s.capture = s.v15 && a_capture;
        const float capFor = s.deepRun ? (std::max)(x.depthCap, x.startLimit) : x.depthCap;
        s.depthLimit = capFor;
        const float dx = a_b.x - a_a.x, dy = a_b.y - a_a.y;
        s.len = std::sqrt(dx * dx + dy * dy);
        if (s.len < 1.f) {
            // (review R1 #8, 1.5.0) 3D like every end test: the same x,y on ANOTHER floor is no clear line
            if (s.v15 && std::fabs(a_b.z - a_a.z) > x.riser + kVLevelBand) {
                s.result = -1;
                s.why = "another floor at the end (the line ran on a floor above or under it)";
                s.failPos = a_b;
                return;
            }
            s.result = 1;
            return;
        }
        const float side = s.v15 ? x.rc : j.radius * kSideFrac;
        s.px = -dy / s.len * side;
        s.py = dx / s.len * side;
        s.n = (std::max)(1, static_cast<int>(std::ceil(s.len / (a_nav ? kPiece : s.v15 ? kVSample : kXSample))));
        s.k = 1;
        if (CrossesKeepOut(j, a_a, a_b)) {
            s.result = -1;
            s.why = "the keep-out circle";
            return;
        }
        if (s.v15 && CrossesActorDisc(x, a_a, a_b)) {
            s.result = -1;
            s.why = "a standing actor (kept out)";
            s.failPos = a_a;
            return;
        }
        if (a_precheck) {
            float deepest = 0.f;
            if (!Precheck(j, a_a, a_b, a_nav, deepest)) {
                s.result = -1;
                s.why = "unexplored or refused ground (grid pre-check)";
                return;
            }
            s.depthLimit = (std::min)(capFor, (std::max)(deepest, a_endDepth) + 16.f);
        }
    }

    // 1.5.0 the floor-rule line test (B §7; item 1): a sample every <= ~11 u - a floor (C12) within ONE RISER of the sample
    // before it (C1), n.z >= 0.70, the water rules of 1.4.1; every 3rd sample (~32 u) and at the end the floor at +-rc beside
    // her within a riser (no brink under her shoulder - a riser at 24 u allows the slopes 1.4.1's half step at 16 u did);
    // every <= 64 u piece her 1.5.0 body (XBody: centre, sides, side columns, the headroom at its middle); and the LAST
    // sample must find the end point's own floor (+- a riser + 8 u): never "clear" along the floor above or under the end.
    // While it runs the rays pass a door when the test allows it (x.doorsOk) and keep the first blocker when asked (x.capOn).
    void SegRunV(Job& j, RE::Actor* a_w, RE::bhkWorld* a_bw)
    {
        ExState& x = *j.x;
        SegTest& s = x.seg;
        x.doorsOk = s.doorsOk;
        x.curDoor = s.door;
        x.capOn = s.capture;
        x.capHave = s.capHave;
        x.capLayer = s.capLayer;
        x.capRef = s.capRef;
        x.capAt = s.capAt;
        const auto fail = [&s](const char* a_why, const RE::NiPoint3& a_at) {
            s.result = -1;
            s.why = a_why;
            s.failAt = s.len * static_cast<float>(s.k) / static_cast<float>(s.n);
            s.failPos = a_at;
        };
        while (s.result == 0) {
            if (Spent(j)) break;
            if (s.flushOwed) {                    // (verifier N2) the side columns of the piece whose centre rays passed: own unit
                s.flushOwed = false;
                bool       beside = false;
                const bool terrSide0 = x.terrSide, stairSide0 = x.stairSide;
                x.terrSide = s.flushTerr;         // (1.5.1) a terrain piece on the navmesh: the hillside beside her is no table top
                x.stairSide = s.flushStair;       // (1.6.0) nor a stair's own face beside a stair piece
                for (int c = 0; c < s.colN && !beside; ++c) {
                    const RE::NiPoint3& cp = s.colQ[static_cast<std::size_t>(c)];
                    beside = XSideColumns(j, a_bw, cp.x, cp.y, cp.z, (s.b.x - s.a.x) / s.len, (s.b.y - s.a.y) / s.len);
                }
                x.terrSide = terrSide0;
                x.stairSide = stairSide0;
                s.colN = 0;
                if (beside) {
                    fail("something beside her (a table top, a counter, a shelf)", s.flushAt);
                    break;
                }
                if (s.k > s.n) s.result = 1;
                continue;
            }
            const float  t = static_cast<float>(s.k) / static_cast<float>(s.n);
            RE::NiPoint3 q{ s.a.x + (s.b.x - s.a.x) * t, s.a.y + (s.b.y - s.a.y) * t, s.a.z + (s.b.z - s.a.z) * t };
            if (IsBadSpot(j, q.x, q.y, s.last.z)) {
                fail("a spot she got stuck at", q);
                break;
            }
            const FloorHit h = XFloorRay(j, a_bw, q.x, q.y, s.last.z + x.riser + kXTopPad, s.last.z - x.riser - kXTopPad, j.xs.rF);
            if (h.blocked) {
                fail("a tree or an invisible wall", q);
                break;
            }
            if (!h.found) {
                fail("no floor within a riser (a drop, a hole or deep water)", q);
                break;
            }
            // (1.5.1) TERRAIN / ground with the navmesh under it may be as steep as the engine walks there: n.z >= 0.45, and a rise
            // up to max(her TERRAIN riser, (the sample spacing) x tan 63 + 8 u) from a terrain sample before it. The navmesh is asked
            // (one question) only for a sample that needs the allowance; STATIC / furniture / clutter floors keep 1.5.0's riser.
            const float rise = std::fabs(h.z - s.last.z);
            const bool  high = rise > x.riser, steep = h.nz < kXMinNz;
            int         navHere = -1;             // the navmesh under this sample: -1 not asked yet, 0 no, 1 yes
            const auto  navUnderHere = [&]() {
                if (navHere < 0) navHere = NavUnder(j, a_w, q.x, q.y, h.z) ? 1 : 0;
                return navHere == 1;
            };
            bool stairHere = false;               // (1.6.0 §5) this sample passed on the stair allowance
            if (high || steep) {
                const float landRise = (std::max)(x.riserT, TerrainRise(s.len / static_cast<float>(s.n)));
                const bool  terr = IsTerrainLayer(h.layer) && h.nz >= kVTerrainNz && (!high || (IsTerrainLayer(s.lastLayer) && rise <= landRise)) &&
                                  navUnderHere();
                if (terr) {
                    ++j.xs.terrLines;
                } else if (!high && StairSlope(h.nz, h.layer) && navUnderHere()) {
                    // (1.6.0 §5) THE STAIR ALLOWANCE: a fixed floor up to 48.7 deg with the navmesh under it, within a riser of the last
                    stairHere = true;
                    s.stairUsed = true;
                    s.pieceStair = true;
                    ++j.xs.stairLines;
                } else {
                    fail(high ? "a step higher than she can take" : "too steep", q);
                    break;
                }
            }
            q.z = h.z;
            if (s.k == s.n && std::fabs(q.z - s.b.z) > x.riser + kVLevelBand) {
                fail("another floor at the end (the line ran on a floor above or under it)", q);
                break;
            }
            float depth = 0.f, surf = 0.f;
            if (!DepthAt(j, a_w, q.x, q.y, q.z, depth, surf)) {
                fail("no loaded cell under the line", q);
                break;
            }
            NoteWater(j, h, surf);
            if (s.deepRun && depth <= x.depthCap) {
                s.deepRun = false;              // (1.4.1) the line has left her start region - the cap rules from here on
                s.depthLimit = (std::min)(s.depthLimit, x.depthCap);
            }
            if (depth > kXWet) {
                if (!(j.xflags & kWater)) {
                    fail("water on a dry-only route", q);
                    break;
                }
                if (depth > s.depthLimit) {
                    fail(s.deepRun ? (depth > x.startLimit ? "deeper than her deep-start limit" : "deeper water than the cells it replaces")
                                   : (depth > x.depthCap ? "deeper than her depth cap" : "deeper water than the cells it replaces"),
                         q);
                    break;
                }
            }
            if ((s.k % 3) == 0 || s.k == s.n) {      // every ~32 u (and at the end): beside her
                if (s.sides) {
                    bool brink = false;
                    // (1.5.1) beside a terrain sample on the navmesh a terrain floor may lie up to her terrain riser off, and up to
                    // rc x tan 63 + 8 u when the floor halfway out confirms a hillside (review fix #1: never the foot of a sheer drop)
                    const bool  terrHere = IsTerrainLayer(h.layer);
                    const float stairSideRise = (std::max)(x.riser, StairRise(x.rc));     // (1.6.0) a stair's own slope beside her
                    const float sideRise = (std::max)(terrHere ? (std::max)(x.riserT, TerrainRise(x.rc)) : x.riser, stairHere ? stairSideRise : 0.f);
                    for (const float sg : { 1.f, -1.f }) {
                        const FloorHit hs = XFloorRay(j, a_bw, q.x + sg * s.px, q.y + sg * s.py, q.z + sideRise + kXTopPad, q.z - sideRise - 1.f, j.xs.rS);
                        if (hs.blocked || !hs.found) {
                            brink = true;
                            break;
                        }
                        const float dz = std::fabs(hs.z - q.z);
                        if (dz <= x.riser) continue;
                        // (1.6.0 §5) the stair beside her (she is on it, or the floor beside her is a stair's own sloped surface)
                        if ((stairHere || StairSlope(hs.nz, hs.layer)) && IsStairLayer(hs.layer) && dz <= stairSideRise) {
                            s.stairUsed = true;
                            ++j.xs.stairLines;
                            continue;
                        }
                        if (terrHere && IsTerrainLayer(hs.layer) && dz <= sideRise && navUnderHere() &&
                            (dz <= x.riserT ||
                             HalfwayLand(j, a_bw, q.x + sg * s.px * 0.5f, q.y + sg * s.py * 0.5f, q.z, hs.z, x.rc, sideRise, j.xs.rS))) {
                            ++j.xs.terrLines;
                            continue;
                        }
                        brink = true;
                        break;
                    }
                    if (brink) {
                        fail("a brink or a rise beside her", q);
                        break;
                    }
                }
                // C6 the side columns at the edge test's own density (one pair per ~32 u, not per 64 u piece: a table corner
                // cannot slip between them). (Review R1 #1) they are QUEUED and cast after the piece's centre rays, so a column
                // beside her can pass only a door her centre line went through - never be the first to meet a door leaf.
                if (s.colN >= static_cast<int>(s.colQ.size())) {    // (never: a piece queues at most 3) cast early - doors stay walls
                    bool early = false;
                    for (const auto& c : s.colQ) early = early || XSideColumns(j, a_bw, c.x, c.y, c.z, (s.b.x - s.a.x) / s.len, (s.b.y - s.a.y) / s.len);
                    s.colN = 0;
                    if (early) {
                        fail("something beside her (a table top, a counter, a shelf)", q);
                        break;
                    }
                }
                s.colQ[static_cast<std::size_t>(s.colN++)] = q;
            }
            s.pieceLen += s.len / static_cast<float>(s.n);
            s.pieceTerr = s.pieceTerr && IsTerrainLayer(h.layer);     // (1.5.1) the piece's ground, and the navmesh seen under it
            if (navHere == 1) s.pieceNav = true;
            if (s.pieceLen >= kPiece - 0.5f || s.k == s.n) {
                const bool pieceSide = s.pieceTerr && s.pieceNav;     // (1.5.1) a terrain piece on the navmesh: its hillside is no wall
                const bool terrSide0 = x.terrSide, stairSide0 = x.stairSide;
                x.terrSide = pieceSide;
                x.stairSide = s.pieceStair;       // (1.6.0) a stair piece: the stair's own face beside her is no wall
                const int body = XBody(j, a_bw, s.pieceA, q, (s.pieceA.z + q.z) * 0.5f, true, false);
                x.terrSide = terrSide0;
                x.stairSide = stairSide0;
                if (body != 0) {
                    fail(body == 1 ? "an obstacle in the way" : "something beside her (a wall, a counter, a narrow gap)", q);
                    break;
                }
                s.flushOwed = s.colN > 0;         // its queued side columns: the next unit (after its centre rays, R1 #1)
                s.flushAt = q;
                s.flushTerr = pieceSide;
                s.flushStair = s.pieceStair;
                s.pieceA = q;
                s.pieceLen = 0.f;
                s.pieceTerr = true;
                s.pieceNav = false;
                s.pieceStair = false;
            }
            s.last = q;
            s.lastLayer = h.layer;                // (1.5.1) the next sample's rise allowance needs terrain here too
            if (++s.k > s.n && !s.flushOwed) s.result = 1;
        }
        // the test's door / blocker back into it (it may go on next slice)
        s.door = x.curDoor;
        s.capHave = x.capHave;
        s.capLayer = x.capLayer;
        s.capRef = x.capRef;
        s.capAt = x.capAt;
        x.doorsOk = false;
        x.capOn = false;
    }

    // Run the line test until it is decided or this slice's share is used (the next slice continues it).
    void SegRun(Job& j, RE::Actor* a_w, RE::bhkWorld* a_bw)
    {
        ExState&   x = *j.x;
        SegTest&   s = x.seg;
        if (s.v15) {              // 1.5.0 (x.v15, floor rules)
            SegRunV(j, a_w, a_bw);
            return;
        }
        const auto fail = [&s](const char* a_why) {
            s.result = -1;
            s.why = a_why;
            s.failAt = s.len * static_cast<float>(s.k) / static_cast<float>(s.n);
        };
        while (s.result == 0) {
            if (Spent(j)) return;
            const float  t = static_cast<float>(s.k) / static_cast<float>(s.n);
            RE::NiPoint3 q{ s.a.x + (s.b.x - s.a.x) * t, s.a.y + (s.b.y - s.a.y) * t, s.a.z + (s.b.z - s.a.z) * t };
            if (s.nav) {
                if (s.k < s.n) {
                    RE::NiPoint3 got;
                    if (!SnapNav(j, a_w, q.x, q.y, q.z, kLevelBand, got) || std::hypot(got.x - q.x, got.y - q.y) > kCell * 0.75f) {
                        fail("no navmesh under the line");
                        return;
                    }
                    if (std::fabs(got.z - s.last.z) > kMaxStep + kPiece * 0.6f) {
                        fail("another level under the line");
                        return;
                    }
                    q.z = got.z;
                    if (IsBadSpot(j, q.x, q.y, q.z)) {
                        fail("a spot she got stuck at");
                        return;
                    }
                    float depth = 0.f, surf = 0.f;
                    if (!DepthAt(j, a_w, q.x, q.y, q.z, depth, surf)) {
                        fail("no loaded cell under the line");
                        return;
                    }
                    if (depth > kXWet) {
                        fail("water under a navmesh leg");
                        return;
                    }
                }
                const int  r0 = j.rays;
                const bool clear = PieceClear(j, a_bw, s.last, q, s.px, s.py);
                j.xs.rO += j.rays - r0;
                if (!clear) {
                    fail("an obstacle in the way");
                    return;
                }
                s.last = q;
            } else {
                // floor first (ledges fail there), then obstacles
                if (IsBadSpot(j, q.x, q.y, s.last.z)) {
                    fail("a spot she got stuck at");
                    return;
                }
                const FloorHit h = FloorRay(j, a_bw, q.x, q.y, s.last.z + x.step + kXTopPad, s.last.z - x.step - kXTopPad, j.xs.rF);
                if (h.blocked) {
                    fail("a tree or an invisible wall");
                    return;
                }
                if (!h.found) {
                    fail("no floor (a drop, a hole or deep water)");
                    return;
                }
                if (std::fabs(h.z - s.last.z) > x.step) {
                    fail("a step higher than she can take");
                    return;
                }
                if (h.nz < kXMinNz) {
                    fail("too steep");
                    return;
                }
                q.z = h.z;
                float depth = 0.f, surf = 0.f;
                if (!DepthAt(j, a_w, q.x, q.y, q.z, depth, surf)) {
                    fail("no loaded cell under the line");
                    return;
                }
                NoteWater(j, h, surf);
                if (s.deepRun && depth <= x.depthCap) {
                    // 1.4.1: the line has left her start region - the cap rules from here on
                    s.deepRun = false;
                    s.depthLimit = (std::min)(s.depthLimit, x.depthCap);
                }
                if (depth > kXWet) {
                    if (!(j.xflags & kWater)) {
                        fail("water on a dry-only route");
                        return;
                    }
                    if (depth > s.depthLimit) {
                        fail(s.deepRun ? (depth > x.startLimit ? "deeper than her deep-start limit" : "deeper water than the cells it replaces")
                                       : (depth > x.depthCap ? "deeper than her depth cap" : "deeper water than the cells it replaces"));
                        return;
                    }
                }
                if (s.sides && ((s.k % 2) == 0 || s.k == s.n)) {    // every 32 u: no brink under her shoulders
                    for (const float sg : { 1.f, -1.f }) {
                        const FloorHit hs = FloorRay(j, a_bw, q.x + sg * s.px, q.y + sg * s.py, q.z + x.step + kXTopPad,
                                                     q.z - x.step * 0.5f - 1.f, j.xs.rS);
                        if (hs.blocked || !hs.found || std::fabs(hs.z - q.z) > x.step * 0.5f) {
                            fail("a brink or a rise beside her");
                            return;
                        }
                    }
                }
                s.pieceLen += s.len / static_cast<float>(s.n);
                if (s.pieceLen >= kPiece - 0.5f || s.k == s.n) {   // every <= 64 u: her body (1.3.0's 6 rays) and her head
                    const int  r0 = j.rays;
                    const bool clear = PieceClear(j, a_bw, s.pieceA, q, s.px, s.py);
                    j.xs.rO += j.rays - r0;
                    if (!clear) {
                        fail("an obstacle in the way");
                        return;
                    }
                    const float mx = (s.pieceA.x + q.x) * 0.5f, my = (s.pieceA.y + q.y) * 0.5f, mz = (s.pieceA.z + q.z) * 0.5f;
                    const int   r1 = j.rays;
                    const bool  low = RayBlocked(j, a_bw, RE::NiPoint3{ mx, my, mz + kXHeadFoot * j.scale }, RE::NiPoint3{ mx, my, mz + kXHeadFrac * x.H });
                    j.xs.rR += j.rays - r1;
                    if (low) {
                        fail("a low ceiling");
                        return;
                    }
                    s.pieceA = q;
                    s.pieceLen = 0.f;
                }
                s.last = q;
            }
            if (++s.k > s.n) s.result = 1;
        }
    }

    // B §5.3 StandSpot: the centre floor (n.z >= 0.85), the water there <= the cap (dry-only: none), headroom to 0.85 H,
    // 4 side floors at 0.8 r (N, E, S, W) within half a step of the centre, no side deeper than the cap + 8. ~11 questions.
    // (1.4.1) a_limit > 0: that depth limit instead of the cap (a spot in her start region: x.startLimit).
    // 1.5.0 StandSpot (C5/C7/C8/C12/C13): the centre floor (n.z >= 0.85), the water there <= the cap (dry-only: none), not in a
    // standing actor's disc, headroom to Htop, 4 side floors at rc (N, E, S, W) within a riser of the centre (B §5.2's slope
    // allowance at the wider radius), no side deeper than the cap + 8. ~11 questions.
    bool StandSpotV(Job& j, RE::Actor* a_w, RE::bhkWorld* a_bw, float a_x, float a_y, float a_top, float a_bottom, float& a_floorZ,
                    float& a_depth, bool& a_hasFloor, std::string& a_why, float* a_surface, float a_limit, std::uint32_t* a_layer)
    {
        ExState&    x = *j.x;
        const float cap = a_limit > 0.f ? a_limit : x.depthCap;
        a_hasFloor = false;
        a_depth = 0.f;
        x.standSteep = false;                     // (fix round, R2 #3) set only by the slope refusal below
        x.standSide = -1;                         // (1.6.1 A3) set only by a side refusal below
        x.standSideWhat = 0;
        if (a_surface) *a_surface = -1.0e9f;
        const FloorHit c = XFloorRay(j, a_bw, a_x, a_y, a_top, a_bottom, j.xs.rF);
        if (c.blocked) {
            a_why = "a tree or an invisible wall stands there";
            return false;
        }
        if (!c.found) {
            a_why = "no floor under it (a hole, a drop or deep water)";
            return false;
        }
        a_hasFloor = true;
        a_floorZ = c.z;
        x.standCentreZ = c.z;                     // (1.6.1 A3) the probe's note
        if (a_layer) *a_layer = c.layer;          // (1.5.1) the floor's layer: terrain + the navmesh = the terrain allowance
        float depth = 0.f, surf = 0.f;
        if (!DepthAt(j, a_w, a_x, a_y, c.z, depth, surf)) {
            a_why = "no loaded cell there";
            return false;
        }
        NoteWater(j, c, surf);
        if (a_surface) *a_surface = surf;
        a_depth = depth > kXWet ? depth : 0.f;
        // (1.6.0, D §1 items 4 / 9) n.z >= 0.85 - or >= 0.75 (x.standNzTerr) on terrain / ground with the navmesh under it (a hillside
        // the engine walks; one navmesh question, only for such a floor). A stair (0.66-0.70) is never a stand spot.
        x.standSeenNz = c.nz;
        x.standSeenTerr = false;
        if (c.nz < kXStandNz) {
            const bool terr = x.standNzTerr < kXStandNz && IsTerrainLayer(c.layer) && c.nz >= x.standNzTerr && NavUnder(j, a_w, a_x, a_y, c.z);
            if (!terr) {
                a_why = std::format("too steep to stand on (n.z {:.2f}, a stand spot needs {:.2f}{})", c.nz, kXStandNz,
                                    x.standNzTerr < kXStandNz ? std::format(" - {:.2f} on terrain with the navmesh under it", x.standNzTerr) : std::string());
                x.standSteep = true;
                return false;
            }
            x.standSeenTerr = true;
        }
        if (a_depth > 0.f && !(j.xflags & kWater)) {
            a_why = std::format("in water ({:.0f} u) and the route is dry-only", a_depth);
            return false;
        }
        if (a_depth > cap) {
            a_why = std::format("too deep: {:.0f} u of water, her cap is {:.0f} u{}", a_depth, cap,
                                cap > x.depthCap + 0.5f ? " (her deep-start limit)" : "");
            return false;
        }
        if (InActorDisc(x, a_x, a_y, c.z)) {
            a_why = "a standing actor is there (kept out)";
            return false;
        }
        if (HeadBlocked(j, a_bw, a_x, a_y, c.z, "at a stand spot")) {
            a_why = std::format("no headroom (to {:.0f} u, her bound top + 2)", x.htop);
            return false;
        }
        // (1.6.1 A3) the 4 side rays step through the probe's target (x.ignoreRef: a chair beside its own marker spot) and, in a probe,
        // name what they hit; a side refusal is recorded (side, heights, layer, reference) for the probe's note
        constexpr float kSides[4][2] = { { 0.f, 1.f }, { 1.f, 0.f }, { 0.f, -1.f }, { -1.f, 0.f } };
        const bool      ignore0 = x.ignoreOn, want0 = x.wantRef;
        x.ignoreOn = x.ignoreRef != 0;
        x.wantRef = x.probeNotes;
        const auto sideFail = [&](int a_sideK, int a_what, const FloorHit& a_hit, float a_sideDepth, const char* a_text) {
            x.ignoreOn = ignore0;
            x.wantRef = want0;
            x.standSide = a_sideK;
            x.standSideWhat = a_what;
            x.standSideZ = a_hit.z;
            x.standSideDepth = a_sideDepth;
            x.standSideLayer = a_hit.layer;
            x.standSideRef = a_hit.ref;
            a_why = a_text;
            return false;
        };
        for (int k = 0; k < 4; ++k) {
            const float    sx = a_x + kSides[k][0] * x.rc, sy = a_y + kSides[k][1] * x.rc;
            const FloorHit h = XFloorRay(j, a_bw, sx, sy, c.z + x.riser + kXTopPad, c.z - x.riser - 1.f, j.xs.rS);
            if (h.blocked || !h.found || std::fabs(h.z - c.z) > x.riser) {
                return sideFail(k, h.blocked ? 1 : !h.found ? 2 : 0, h, 0.f, "uneven ground or a brink beside the spot");
            }
            float sd = 0.f, ss = 0.f;
            if (!DepthAt(j, a_w, sx, sy, h.z, sd, ss)) return sideFail(k, 4, h, 0.f, "no loaded cell beside the spot");
            if (sd > kXWet && sd > cap + 8.f) return sideFail(k, 3, h, sd, "deep water beside the spot");
        }
        x.ignoreOn = ignore0;
        x.wantRef = want0;
        return true;
    }

    bool StandSpot(Job& j, RE::Actor* a_w, RE::bhkWorld* a_bw, float a_x, float a_y, float a_top, float a_bottom, float& a_floorZ,
                   float& a_depth, bool& a_hasFloor, std::string& a_why, float* a_surface = nullptr, float a_limit = 0.f,
                   std::uint32_t* a_layer = nullptr)
    {
        ExState&    x = *j.x;
        if (x.v15) return StandSpotV(j, a_w, a_bw, a_x, a_y, a_top, a_bottom, a_floorZ, a_depth, a_hasFloor, a_why, a_surface, a_limit, a_layer);
        const float cap = a_limit > 0.f ? a_limit : x.depthCap;
        a_hasFloor = false;
        a_depth = 0.f;
        if (a_surface) *a_surface = -1.0e9f;
        const FloorHit c = FloorRay(j, a_bw, a_x, a_y, a_top, a_bottom, j.xs.rF);
        if (c.blocked) {
            a_why = "a tree or an invisible wall stands there";
            return false;
        }
        if (!c.found) {
            a_why = "no floor under it (a hole, a drop or deep water)";
            return false;
        }
        a_hasFloor = true;
        a_floorZ = c.z;
        float depth = 0.f, surf = 0.f;
        if (!DepthAt(j, a_w, a_x, a_y, c.z, depth, surf)) {
            a_why = "no loaded cell there";
            return false;
        }
        NoteWater(j, c, surf);
        if (a_surface) *a_surface = surf;
        a_depth = depth > kXWet ? depth : 0.f;
        if (c.nz < kXStandNz) {
            a_why = std::format("too steep to stand on (n.z {:.2f}, a stand spot needs {:.2f})", c.nz, kXStandNz);
            return false;
        }
        if (a_depth > 0.f && !(j.xflags & kWater)) {
            a_why = std::format("in water ({:.0f} u) and the route is dry-only", a_depth);
            return false;
        }
        if (a_depth > cap) {
            a_why = std::format("too deep: {:.0f} u of water, her cap is {:.0f} u{}", a_depth, cap,
                                cap > x.depthCap + 0.5f ? " (her deep-start limit)" : "");
            return false;
        }
        const int  r0 = j.rays;
        const bool low = RayBlocked(j, a_bw, RE::NiPoint3{ a_x, a_y, c.z + kXHeadFoot * j.scale }, RE::NiPoint3{ a_x, a_y, c.z + kXHeadFrac * x.H });
        j.xs.rR += j.rays - r0;
        if (low) {
            a_why = "no headroom";
            return false;
        }
        const float           sr = j.radius * kSideFrac;
        constexpr float       kSides[4][2] = { { 0.f, 1.f }, { 1.f, 0.f }, { 0.f, -1.f }, { -1.f, 0.f } };
        for (const auto& d : kSides) {
            const float    sx = a_x + d[0] * sr, sy = a_y + d[1] * sr;
            const FloorHit h = FloorRay(j, a_bw, sx, sy, c.z + x.step + kXTopPad, c.z - x.step * 0.5f - 1.f, j.xs.rS);
            if (h.blocked || !h.found || std::fabs(h.z - c.z) > x.step * 0.5f) {
                a_why = "uneven ground or a brink beside the spot";
                return false;
            }
            float sd = 0.f, ss = 0.f;
            if (!DepthAt(j, a_w, sx, sy, h.z, sd, ss)) {
                a_why = "no loaded cell beside the spot";
                return false;
            }
            if (sd > kXWet && sd > cap + 8.f) {
                a_why = "deep water beside the spot";
                return false;
            }
        }
        return true;
    }

    // B §5.3 ClassifyStep - it only NAMES what lies ahead of p along (dx, dy) (for the log and a partial route's end):
    // 3 level rays (0.30 / 0.50 / 0.70 H, 48 u) for a wall facing her, then its top; else one down ray 40 u ahead.
    // LEDGE_UP is reported, never used: 1.4.0 has no climb and no jump edges.
    std::string ClassifyStep(Job& j, RE::Actor* a_w, RE::bhkWorld* a_bw, const RE::NiPoint3& a_p, float a_dx, float a_dy)
    {
        ExState& x = *j.x;
        float    dw = 1.0e9f;
        for (const float hf : { 0.30f, 0.50f, 0.70f }) {
            const RE::NiPoint3 a{ a_p.x, a_p.y, a_p.z + hf * x.H };
            const RE::NiPoint3 b{ a.x + a_dx * 48.f, a.y + a_dy * 48.f, a.z };
            const XHit         h = RayFirstBlock(j, a_bw, a, b, j.xs.rC);
            if (h.hit && std::fabs(h.nz) < 0.5f && (-h.nx * a_dx - h.ny * a_dy) >= 0.35f) dw = (std::min)(dw, h.frac * 48.f);
        }
        if (dw < 1.0e8f) {
            const float    tx = a_p.x + a_dx * (dw + 14.f), ty = a_p.y + a_dy * (dw + 14.f);
            const FloorHit t = FloorRay(j, a_bw, tx, ty, a_p.z + 1.6f * x.H, a_p.z + x.step, j.xs.rC);
            if (t.found && t.nz >= kXMinNz && t.z - a_p.z <= 0.8f * x.H) {
                return std::format("LEDGE_UP {:.0f} u, {:.0f} u ahead (no climbing in 1.4.0)", t.z - a_p.z, dw);
            }
            return std::format("WALL {:.0f} u ahead", dw);
        }
        const float    qx = a_p.x + a_dx * 40.f, qy = a_p.y + a_dy * 40.f;
        const float    down = (std::max)(kXFloorDown, kXFloorDownH * x.H);
        const FloorHit d = FloorRay(j, a_bw, qx, qy, a_p.z + x.step + kXTopPad, a_p.z - down, j.xs.rC);
        if (d.blocked) return std::format("BLOCKED (a tree or an invisible wall at ({:.0f},{:.0f}))", qx, qy);
        float depth = 0.f, surf = 0.f;
        if (!d.found) {
            if (DepthAt(j, a_w, qx, qy, a_p.z - down, depth, surf) && IsFiniteHeight(surf) && surf < a_p.z) {
                return std::format("CLIFF: no floor within {:.0f} u down at ({:.0f},{:.0f}); a water surface {:.0f} u down", down, qx, qy, a_p.z - surf);
            }
            return std::format("CLIFF: no floor within {:.0f} u down at ({:.0f},{:.0f})", down, qx, qy);
        }
        const float dz = d.z - a_p.z;
        if (std::fabs(dz) <= kXWet) return "FLAT";
        if (dz > 0.f) return std::format("STEP_UP {:.0f} u", dz);
        if (-dz <= x.step) return std::format("STEP_DOWN {:.0f} u", -dz);
        if (DepthAt(j, a_w, qx, qy, d.z, depth, surf) && depth > kXWet) {
            return std::format("DROP_INTO_WATER: drop {:.0f} u at ({:.0f},{:.0f}) into water (surface {:.0f} u down, {:.0f} u deep)", -dz, qx, qy,
                               a_p.z - surf, depth);
        }
        return std::format("DROP {:.0f} u at ({:.0f},{:.0f})", -dz, qx, qy);
    }

    // B §3.2 Classify, from the level a_zRef of the parent that first touched m. (Review G, BUG 2 / RISK 5 / RISK 9: in
    // 1.4.0 the driver walks every leg, so EVERY cell stands on its physics floor.) The navmesh only says whether the cell
    // is on it - the cell stays on its lattice point (no 19 u slide onto another dock / rock) - then ONE down ray gives the
    // floor and the slope for every cell (a navmesh draped under a rock shelf or lying on a water surface no longer passes
    // for the floor), then the water over THAT floor. Without kFloor a cell must also be on the navmesh.
    // (1.4.1) a_fromDeep: the classifying parent lies in her START REGION - water deeper than the cap is still walkable here
    // up to x.startLimit (the cell joins the region); from any other parent it is refused (kWhyDeep) as before.
    void XClassify(Job& j, RE::Actor* a_w, RE::bhkWorld* a_bw, XNode& m, float a_zRef, bool a_fromDeep)
    {
        ExState& x = *j.x;
        m.zRef = a_zRef;
        m.why = kWhyNone;
        m.hasZ = false;
        m.onNav = false;
        m.depth = 0.f;
        m.head = 0;
        m.deepOk = false;
        const auto none = [&m](std::uint8_t a_why) {
            m.kind = kKNone;
            m.why = a_why;
        };
        if (InKeepOut(j, m.x, m.y)) {
            none(kWhyKeepOut);
            return;
        }
        if (IsBadSpot(j, m.x, m.y, a_zRef)) {
            none(kWhyBad);
            return;
        }
        RE::NiPoint3 got;
        const bool   nav = SnapNav(j, a_w, m.x, m.y, a_zRef, kLevelBand, got) && std::hypot(got.x - m.x, got.y - m.y) <= kCell * 0.6f &&
                         std::fabs(got.z - a_zRef) <= kMaxStep;
        if (!nav && !(j.xflags & kFloor)) {
            none(kWhyNoFloor);
            return;
        }
        const FloorHit h = FloorRay(j, a_bw, m.x, m.y, a_zRef + x.step + kXTopPad, a_zRef - (std::max)(kXFloorDown, kXFloorDownH * x.H), j.xs.rF);
        if (h.blocked) {
            none(kWhyBlocked);
            return;
        }
        if (!h.found) {
            none(kWhyNoFloor);
            return;
        }
        m.z = h.z;
        m.hasZ = true;
        if (h.nz < kXMinNz) {
            none(kWhySteep);
            return;
        }
        m.onNav = nav && std::fabs(got.z - h.z) <= x.step;      // the navmesh lies on THIS floor
        if (!m.onNav && !(j.xflags & kFloor)) {
            none(kWhyNoFloor);
            return;
        }
        m.kind = m.onNav ? kKNav : kKFloor;
        float depth = 0.f, surf = 0.f;
        if (!DepthAt(j, a_w, m.x, m.y, m.z, depth, surf)) {
            none(kWhyNoCell);
            return;
        }
        NoteWater(j, h, surf);
        if (depth > kXWet) {
            m.depth = depth;
            if (!(j.xflags & kWater)) {
                none(kWhyDryOnly);
                return;
            }
            if (depth > x.depthCap) {
                if (a_fromDeep && depth <= x.startLimit) {     // 1.4.1: her start region goes on here
                    m.kind = kKWater;
                    m.deepOk = true;
                    return;
                }
                none(kWhyDeep);
                return;
            }
            m.kind = kKWater;
        }
    }

    // B §3.4 the step p -> m, at pop. (Review G, BUG 2: the driver walks this step as a straight line, so EVERY step is
    // tested like a FLOOR step, navmesh or not:) m's headroom (once per cell), the floor at the step's middle (within half a
    // step of the ends' band, slope n.z >= 0.70, and the water over THAT floor <= the cap / dry when dry-only), then
    // 1.3.0's body rays + the keep-out. 0 = clear, 1 = this step is blocked, 2 = m itself has no headroom.
    int XEdgeTest(Job& j, RE::Actor* a_w, RE::bhkWorld* a_bw, const XNode& a_p, XNode& a_m)
    {
        ExState& x = *j.x;
        if (a_m.head == 0) {
            const int  r0 = j.rays;
            const bool low = RayBlocked(j, a_bw, RE::NiPoint3{ a_m.x, a_m.y, a_m.z + kXHeadFoot * j.scale }, RE::NiPoint3{ a_m.x, a_m.y, a_m.z + kXHeadFrac * x.H });
            j.xs.rR += j.rays - r0;
            a_m.head = low ? 2 : 1;
        }
        if (a_m.head == 2) return 2;
        {
            const float    lo = (std::min)(a_p.z, a_m.z), hi = (std::max)(a_p.z, a_m.z);
            const float    mx = (a_p.x + a_m.x) * 0.5f, my = (a_p.y + a_m.y) * 0.5f;
            const FloorHit h = FloorRay(j, a_bw, mx, my, hi + x.step + kXTopPad, lo - x.step - kXTopPad, j.xs.rM);
            if (h.blocked || !h.found || h.z < lo - x.step * 0.5f || h.z > hi + x.step * 0.5f || h.nz < kXMinNz) return 1;
            float depth = 0.f, surf = 0.f;
            if (!DepthAt(j, a_w, mx, my, h.z, depth, surf)) return 1;
            NoteWater(j, h, surf);
            // (1.4.1) a step out of her start region may cross its water at the middle too
            const float lim = a_p.deepOk ? (std::max)(x.depthCap, x.startLimit) : x.depthCap;
            if (depth > kXWet && (!(j.xflags & kWater) || depth > lim)) return 1;
        }
        const int  r0 = j.rays;
        const bool clear = SegmentClear(j, a_w, a_bw, XPos(a_p), XPos(a_m), false);
        j.xs.rO += j.rays - r0;
        return clear ? 0 : 1;
    }

    // (review G: one step rule - every cell's z is its physics floor now, navmesh or not; 1.3.0's 40 u NAV<->NAV allowance
    // was for legs the ENGINE walked)
    float XStepLimit(const ExState& a_x, const XNode&, const XNode&) { return a_x.step; }

    // B §7.3: per unit of 3D length NAV dry 1.0 · FLOOR dry 1.25 · WATER 1.25 + 1.5 depth / H; + 0.5 out of an edge cell;
    // + 0.5 x |dz|
    float XStepCost(const ExState& a_x, const XNode& a_p, const XNode& a_m)
    {
        const float dx = a_m.x - a_p.x, dy = a_m.y - a_p.y, dz = a_m.z - a_p.z;
        const float len = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (a_x.wadeScan) return len;     // (1.6.0 §6.3) a wade scan is distance only: the NEAREST deeper spot wins
        float       mult = a_m.kind == kKNav ? 1.f : a_m.kind == kKFloor ? kXCostFloor : kXCostWater + kXCostDepth * a_m.depth / a_x.H;
        if (a_p.edge) mult += kXCostEdge;
        return len * mult + kXCostDz * std::fabs(dz);
    }

    // the step from one neighbour INTO cell m was blocked: m may still be reached from its other closed neighbours
    void XReopen(Job& j, std::int32_t a_m)
    {
        ExState&  x = *j.x;
        XNode&    m = x.nodes[static_cast<std::size_t>(a_m)];
        const int mx = a_m % x.nx, my = a_m / x.nx;
        m.bestG = 3.0e38f;
        for (int d = 0; d < 8; ++d) {
            if (m.rejectedFrom & (1u << d)) continue;
            const int qx = mx - kDir[d][0], qy = my - kDir[d][1];      // q --(direction d)--> m
            if (qx < 0 || qy < 0 || qx >= x.nx || qy >= x.ny) continue;
            const std::int32_t qi = qy * x.nx + qx;
            const XNode&       q = x.nodes[static_cast<std::size_t>(qi)];
            if (!q.closed || std::fabs(m.z - q.z) > XStepLimit(x, q, m)) continue;
            if (m.deepOk && !q.deepOk) continue;     // 1.4.1: her start region is never entered from outside it
            const float ng = q.g + XStepCost(x, q, m);
            if (ng < m.bestG) m.bestG = ng;
            x.open.push({ ng + kHeurWeight * Dist2D(XPos(m), x.goal.pos), ng, a_m, qi });
        }
    }

    // Expand the closed cell x.expIdx, resumable neighbour by neighbour (review G, RISK 6: each classification and each
    // LEDGE probe is its own unit of work, checked against this slice's share): classify the new neighbours from its
    // level, tag it `edge` when one lies over a drop (the first few refused steps are named in LEDGE lines), then push the
    // neighbours within one step. true = done; false = the slice's share is used (x.expD / x.expProbe keep the place).
    bool XExpandStep(Job& j, RE::Actor* a_w, RE::bhkWorld* a_bw)
    {
        ExState&           x = *j.x;
        const std::int32_t idx = x.expIdx;
        XNode&             n = x.nodes[static_cast<std::size_t>(idx)];
        const int          ix = idx % x.nx, iy = idx / x.nx;
        const int          ledgeMax = (j.xflags & kVerbose) ? kXLedgeLinesV : kXLedgeLines;
        while (x.expD < 8) {
            const int d = x.expD;
            const int mx = ix + kDir[d][0], my = iy + kDir[d][1];
            if (mx < 0 || my < 0 || mx >= x.nx || my >= x.ny) {
                ++x.expD;
                continue;
            }
            XNode& m = x.nodes[static_cast<std::size_t>(my * x.nx + mx)];
            if (x.expProbe == d) {                // the LEDGE probe owed toward this neighbour
                if (Spent(j)) return false;
                x.expProbe = -1;
                const float dl = std::sqrt(static_cast<float>(kDir[d][0] * kDir[d][0] + kDir[d][1] * kDir[d][1]));
                const float dx = static_cast<float>(kDir[d][0]) / dl, dy = static_cast<float>(kDir[d][1]) / dl;
                const std::string what = ClassifyStep(j, a_w, a_bw, XPos(n), dx, dy);
                logger::info("PATH #{} LEDGE: from ({:.0f},{:.0f},{:.0f}) toward ({:.2f},{:.2f}): {}", j.handle, n.x, n.y, n.z, dx, dy, what);
                ++x.expD;
                continue;
            }
            if (!m.closed) {
                // a cell with no floor seen from another level gets one more look from this one (a cliff's foot)
                if (m.kind == kKNone && m.why == kWhyNoFloor && std::fabs(n.z - m.zRef) > x.step) m.kind = kKUnknown;
                // (1.4.1) a cell refused as too deep from outside her start region joins it when touched from inside it (its
                // floor and depth are known: no new question)
                if (m.kind == kKNone && m.why == kWhyDeep && n.deepOk && m.hasZ && m.depth <= x.startLimit) {
                    m.kind = kKWater;
                    m.why = kWhyNone;
                    m.deepOk = true;
                }
                if (m.kind == kKUnknown) {
                    if (Spent(j)) return false;
                    XClassify(j, a_w, a_bw, m, n.z, n.deepOk);
                }
            }
            const bool drop = (m.kind == kKNone && m.why == kWhyNoFloor) || (m.hasZ && m.z < n.z - XStepLimit(x, n, m));
            if (drop) n.edge = true;
            const bool refused = drop || (m.hasZ && m.z > n.z + XStepLimit(x, n, m));
            if (refused && j.xs.ledges < ledgeMax) {
                ++j.xs.ledges;
                x.expProbe = d;                   // its probe runs as the next unit
                continue;
            }
            ++x.expD;
        }
        for (int d = 0; d < 8; ++d) {
            const int mx = ix + kDir[d][0], my = iy + kDir[d][1];
            if (mx < 0 || my < 0 || mx >= x.nx || my >= x.ny) continue;
            const std::int32_t mi = my * x.nx + mx;
            XNode&             m = x.nodes[static_cast<std::size_t>(mi)];
            if (m.closed || m.kind == kKNone || m.kind == kKUnknown || (m.rejectedFrom & (1u << d))) continue;
            if (std::fabs(m.z - n.z) > XStepLimit(x, n, m)) continue;     // the ledge rule: never an edge over a step
            if (m.deepOk && !n.deepOk) continue;     // 1.4.1: her start region is never re-entered once the route has left it
            const float ng = n.g + XStepCost(x, n, m);
            if (ng < m.bestG) {
                m.bestG = ng;
                x.open.push({ ng + kHeurWeight * Dist2D(XPos(m), x.goal.pos), ng, mi, idx });
            }
        }
        return true;
    }

    // =================================================================================================================
    //  1.5.0 THE SEARCH (x.v15): several floors per map square (C9), the body rules at every step (C1-C8, C12-C13), nudging
    //  (C11). The 1.4.1 functions above stay as they were for a 1.4.1 plan.
    // =================================================================================================================

    // =================================================================================================================
    //  1.5.1 THE CELL CACHE (spec §4; g_cache, g_mtx held by Step) - reused only by a plan whose physics world is the cache's
    // =================================================================================================================

    bool CacheLive(const ExState& x) { return x.cacheOn && g_cache.world == x.cacheWorld && g_cache.space == x.cacheSpace; }

    bool CacheFresh(const ExState& x, CacheClock::time_point a_t) { return x.cacheNow - a_t < std::chrono::milliseconds(kVCacheLifeMs); }

    // a world lattice index: true when a_v is an exact multiple of the 32 u grid (the 1.5.1 search's squares lie on it)
    bool LatticeIndex(float a_v, std::int32_t& a_i)
    {
        const float k = std::round(a_v / kCell);
        if (!(std::fabs(k) < 1.0e6f) || k * kCell != a_v) return false;
        a_i = static_cast<std::int32_t>(k);
        return true;
    }

    std::int32_t ZKey(float a_z) { return static_cast<std::int32_t>(std::lround(a_z * 4.f)); }    // a floor's level, to 0.25 u

    // her size class for the cells' and the steps' tests: clearance (1 u), headroom (2 u), both risers (1 u), scale (0.02) buckets
    // (bits 0-35; bit 40 = the terrain flag of a key)
    std::uint64_t SizeClass(const Job& j, const ExState& x)
    {
        const auto q = [](float a_v, float a_unit, long a_max) {
            return static_cast<std::uint64_t>(std::clamp(std::lround(a_v / a_unit), 0L, a_max));
        };
        return q(x.rc, 1.f, 127) | (q(x.htop, 2.f, 511) << 7) | (q(x.riser, 1.f, 63) << 16) | (q(j.scale, 0.02f, 255) << 22) |
               (q(x.riserT, 1.f, 63) << 30);
    }

    // a BOUNDED sweep of the squares: at most kVCacheSweepSome of them (and kVCacheBucketWalk buckets) per call, from the bucket where
    // the last call stopped (a rehash only shifts what the next calls visit); expired answers go, a square with none left goes
    void CacheSweepSquares(const ExState& x)
    {
        auto&             m = g_cache.squares;
        const std::size_t nb = m.bucket_count();
        if (m.empty() || nb == 0) return;
        std::vector<std::uint64_t> dead;
        std::size_t                seen = 0;
        for (std::size_t b = 0; b < nb && b < kVCacheBucketWalk && seen < kVCacheSweepSome; ++b) {
            if (g_cache.curSq >= nb) g_cache.curSq = 0;
            for (auto it = m.begin(g_cache.curSq); it != m.end(g_cache.curSq); ++it) {
                ++seen;
                bool any = false;
                for (auto& r : it->second.r) {
                    if (r.what && !CacheFresh(x, r.t)) r.what = 0;
                    any = any || r.what != 0;
                }
                if (!any) dead.push_back(it->first);
            }
            ++g_cache.curSq;
        }
        for (const auto k : dead) m.erase(k);
    }

    // the same bounded sweep for the cells' and the steps' tables
    template <class M>
    void CacheSweepSome(const ExState& x, M& a_map, std::size_t& a_cursor)
    {
        const std::size_t nb = a_map.bucket_count();
        if (a_map.empty() || nb == 0) return;
        std::vector<typename M::key_type> dead;
        std::size_t                       seen = 0;
        for (std::size_t b = 0; b < nb && b < kVCacheBucketWalk && seen < kVCacheSweepSome; ++b) {
            if (a_cursor >= nb) a_cursor = 0;
            for (auto it = a_map.cbegin(a_cursor); it != a_map.cend(a_cursor); ++it) {
                ++seen;
                if (!CacheFresh(x, it->second.t)) dead.push_back(it->first);
            }
            ++a_cursor;
        }
        for (const auto& k : dead) a_map.erase(k);
    }

    void CacheSweep(const ExState& x)
    {
        CacheSweepSquares(x);
        CacheSweepSome(x, g_cache.cells, g_cache.curCell);
        CacheSweepSome(x, g_cache.steps, g_cache.curStep);
    }

    // room for one more answer: a full table sweeps a bounded part of itself (review #10: at most once a slice); still full of live
    // answers -> this one is not kept (the memory bound; nothing is ever emptied in one go during play)
    bool CacheRoomSquares(const ExState& x)
    {
        if (g_cache.squares.size() < kVCacheSquares) return true;
        if (g_cache.fullSweepAt != x.cacheNow) {
            g_cache.fullSweepAt = x.cacheNow;
            CacheSweepSquares(x);
            if (g_cache.squares.size() < kVCacheSquares) return true;
        }
        ++g_cache.full;
        return false;
    }

    template <class M>
    bool CacheRoom(const ExState& x, M& a_map, std::size_t a_cap, std::size_t& a_cursor)
    {
        if (a_map.size() < a_cap) return true;
        if (g_cache.fullSweepAt != x.cacheNow) {
            g_cache.fullSweepAt = x.cacheNow;
            CacheSweepSome(x, a_map, a_cursor);
            if (a_map.size() < a_cap) return true;
        }
        ++g_cache.full;
        return false;
    }

    // (review #10) a previous world's answers are never used; they are FREED a bounded part per plan start (no slice frees a full
    // cache in one go)
    void CacheDrainOld()
    {
        if (g_cacheOld.Size() == 0) return;
        std::size_t n = 0;
        const auto  drain = [&n](auto& a_map) {
            while (n < kVCacheSweepSome && !a_map.empty()) {
                a_map.erase(a_map.begin());
                ++n;
            }
        };
        drain(g_cacheOld.squares);
        drain(g_cacheOld.cells);
        drain(g_cacheOld.steps);
        if (g_cacheOld.Size() == 0) g_cacheOld.Clear();     // the last of it: its bucket arrays go too
    }

    // a plan's cache set-up (main thread, g_mtx): her physics world (an interior cell's own, or her worldspace's) - another one
    // than the cache's (another interior, going in or out, a world rebuilt) takes the cache out of use at once ("cleared on a cell
    // change"; its memory is freed a bounded part per plan start, review #10); a load clears it in ClearAll. Each plan's start
    // sweeps a bounded part of every table. Exterior answers are reused only over loaded cells (FloorFacts, review #3). Never
    // for a probe.
    void CacheBegin(Job& j, RE::Actor* a_w, ExState& x)
    {
        auto* cell = a_w->GetParentCell();
        auto* bw = cell ? cell->GetbhkWorld() : nullptr;
        if (!cell || !bw) return;
        auto* ws = a_w->GetWorldspace();
        x.cacheNow = CacheClock::now();
        x.cacheWorld = static_cast<std::uint64_t>(reinterpret_cast<std::uintptr_t>(bw));
        x.cacheSpace = cell->IsInteriorCell() ? cell->GetFormID() : (ws ? ws->GetFormID() : 0u);
        x.cacheClass = SizeClass(j, x);
        CacheDrainOld();
        if (g_cache.world != x.cacheWorld || g_cache.space != x.cacheSpace) {
            const std::size_t n = g_cache.Size();
            if (n) {
                if (g_cacheOld.Size()) g_cacheOld.Clear();     // (two world changes before the last drain ended: rare)
                g_cacheOld.squares.swap(g_cache.squares);
                g_cacheOld.cells.swap(g_cache.cells);
                g_cacheOld.steps.swap(g_cache.steps);
                logger::info("PATH #{} cell cache: {} kept answer(s) dropped - she plans in another physics world now ({} {:08X})", j.handle, n,
                             cell->IsInteriorCell() ? "interior cell" : "worldspace", x.cacheSpace);
            }
            g_cache.Clear();                                   // (empty maps now: nothing heavy freed here)
            g_cache.world = x.cacheWorld;
            g_cache.space = x.cacheSpace;
        } else {
            CacheSweep(x);
        }
        x.cacheOn = true;
    }

    // one classification's down ray, from the cache or cast
    struct VFloor {
        bool          found{ false }, blocked{ false };
        float         z{ 0.f }, nz{ 1.f };
        std::uint32_t layer{ 0 };
        SurfRec*      rec{ nullptr };     // its kept record (the navmesh and the water over that floor are kept there too)
        bool          fresh{ false };     // cast now (its water hits go to the WATER SEEN numbers)
        FloorHit      h{};
    };

    // The classification's down ray at a lattice square (a_x, a_y) from a_top to a_bottom. A kept answer of that square covers
    // this window when it found a surface S first from a top at or over a_top while S lies under a_top (then S is this ray's
    // first hit too - or, S under a_bottom, nothing is in this window), or when it found nothing from a top at or over a_top to a
    // bottom at or under a_bottom. Else the ray is cast (one question) and kept when every hit was a fixed world layer; a kept
    // answer it contradicts goes (the world changed); the same surface found from higher keeps ITS first z (one level, one z).
    // (review fix #3) an EXTERIOR square is reused or kept only while the cell under it is loaded and attached: a ray over an
    // unloaded / still-streaming cell finds "nothing" for the wrong reason, and a kept floor (with its depth) must not outlive its
    // cell - one grid look-up per question (an interior is her own loaded cell)
    bool SquareLoaded(const ExState& x, float a_x, float a_y, float a_z)
    {
        if (x.interior) return true;
        auto* tes = RE::TES::GetSingleton();
        auto* c = tes ? tes->GetCell(RE::NiPoint3{ a_x, a_y, a_z }) : nullptr;
        return c && !c->IsInteriorCell() && c->IsAttached();
    }

    VFloor FloorFacts(Job& j, RE::bhkWorld* a_bw, float a_x, float a_y, float a_top, float a_bottom)
    {
        ExState&      x = *j.x;
        VFloor        out;
        std::int32_t  ix = 0, iy = 0;
        const bool    keyed = CacheLive(x) && LatticeIndex(a_x, ix) && LatticeIndex(a_y, iy) && SquareLoaded(x, a_x, a_y, a_top);
        const auto    key = CacheHash::Pair(ix, iy);
        if (keyed) {
            ++j.xs.cacheFloorAsks;
            const auto it = g_cache.squares.find(key);
            if (it != g_cache.squares.end()) {
                for (auto& r : it->second.r) {
                    if (!r.what || !CacheFresh(x, r.t)) continue;
                    bool answers = false, nothing = false;
                    if (r.what == 3) {
                        answers = nothing = a_top <= r.top + kVCacheEps && a_bottom >= r.bottom - kVCacheEps;
                    } else if (r.z <= a_top + kVCacheEps && a_top <= r.top + kVCacheEps) {
                        answers = true;
                        nothing = r.z < a_bottom - kVCacheEps;
                    }
                    if (!answers) continue;
                    ++j.xs.cacheFloorHits;
                    if (!nothing) {
                        out.found = r.what == 1;
                        out.blocked = r.what == 2;
                        out.z = r.z;
                        out.nz = r.nz;
                        out.layer = r.layer;
                        out.rec = &r;
                    }
                    return out;
                }
            }
        }
        const int      dirty0 = x.cacheDirty;
        const FloorHit h = XFloorRay(j, a_bw, a_x, a_y, a_top, a_bottom, j.xs.rF);
        out.found = h.found;
        out.blocked = h.blocked;
        out.z = h.z;
        out.nz = h.nz;
        out.layer = h.layer;
        out.fresh = true;
        out.h = h;
        if (!keyed || x.cacheDirty != dirty0) return out;
        auto known = g_cache.squares.find(key);
        if (known == g_cache.squares.end()) {
            if (!CacheRoomSquares(x)) return out;     // full of live answers: this one is not kept (the memory bound)
            known = g_cache.squares.try_emplace(key).first;
        }
        SquareRec&         sq = known->second;
        const bool         surfHit = h.found || h.blocked;
        const std::uint8_t what = static_cast<std::uint8_t>(h.found ? 1 : h.blocked ? 2 : 3);
        for (auto& r : sq.r) {                    // what this answer contradicts (or what expired) goes
            if (!r.what) continue;
            if (!CacheFresh(x, r.t)) {
                r.what = 0;
                continue;
            }
            if (surfHit) {
                if (r.what == 3) {
                    if (h.z <= r.top + kVCacheEps && h.z >= r.bottom - kVCacheEps) r.what = 0;           // "nothing" where S is
                } else if (std::fabs(r.z - h.z) <= kVCacheSameZ) {
                    if (r.what != what) r.what = 0;                                                       // the same place, another kind
                } else if ((h.z > r.z && h.z <= r.top + kVCacheEps) || (r.z > h.z && r.z <= a_top + kVCacheEps)) {
                    r.what = 0;                   // one says nothing where the other found a surface
                }
            } else if (r.what != 3 && r.z <= a_top + kVCacheEps && r.z >= a_bottom - kVCacheEps) {
                r.what = 0;                       // a surface where this ray found nothing
            }
        }
        if (surfHit) {                            // the same surface kept already: its "nothing above" now reaches a_top
            for (auto& r : sq.r) {
                if (r.what == what && std::fabs(r.z - h.z) <= kVCacheSameZ) {
                    r.top = (std::max)(r.top, a_top);
                    out.z = r.z;
                    out.nz = r.nz;
                    out.layer = r.layer;
                    out.rec = &r;
                    return out;
                }
            }
        }
        SurfRec* slot = nullptr;
        for (auto& r : sq.r) {
            if (!r.what) {
                slot = &r;
                break;
            }
        }
        if (!slot) {
            slot = &sq.r[0];
            for (auto& r : sq.r) {
                if (r.t < slot->t) slot = &r;
            }
        }
        *slot = SurfRec{};
        slot->what = what;
        slot->z = h.z;
        slot->nz = h.nz;
        slot->layer = static_cast<std::uint8_t>(h.layer & 0x7Fu);
        slot->top = a_top;
        slot->bottom = a_bottom;
        slot->t = x.cacheNow;
        ++j.xs.cacheStores;
        if (surfHit) out.rec = slot;
        return out;
    }

    // the navmesh ON the floor a_f found at (a_x, a_y): within 19 u of it and one step of its height - NavUnder's question (the
    // navmesh within 48 u of that floor), kept with the floor's record
    bool FloorNav(Job& j, RE::Actor* a_w, const VFloor& a_f, float a_x, float a_y)
    {
        const ExState& x = *j.x;
        bool           ok = false;
        float          navZ = 0.f;
        if (a_f.rec && a_f.rec->nav) {
            ok = a_f.rec->nav == 2;
            navZ = a_f.rec->navZ;
        } else {
            RE::NiPoint3 got{};
            ok = SnapNav(j, a_w, a_x, a_y, a_f.z, kLevelBand, got) && std::hypot(got.x - a_x, got.y - a_y) <= kCell * 0.6f;
            navZ = got.z;
            if (a_f.rec) {
                a_f.rec->nav = static_cast<std::uint8_t>(ok ? 2 : 1);
                a_f.rec->navZ = navZ;
            }
        }
        return ok && std::fabs(navZ - a_f.z) <= x.step;
    }

    // the water over the floor a_f (DepthAt), kept with the floor's record (an unloaded cell is never kept; review #6: only from a
    // COMPLETE placed-water scan - a capped or truncated list could miss the pool another plan would see)
    bool FloorDepth(Job& j, RE::Actor* a_w, const VFloor& a_f, float a_x, float a_y, float& a_depth, float& a_surf)
    {
        if (a_f.rec && a_f.rec->dep == 2) {
            a_depth = a_f.rec->depth;
            a_surf = a_f.rec->surf;
            return true;
        }
        const bool known = DepthAt(j, a_w, a_x, a_y, a_f.z, a_depth, a_surf);
        if (known && a_f.rec && j.x->watersComplete) {
            a_f.rec->dep = 2;
            a_f.rec->depth = a_depth;
            a_f.rec->surf = a_surf;
        }
        return known;
    }

    // 1.5.0 Classify (C4 / C12 / C13) - 1.4.1's XClassify with: the navmesh looked for within a riser + L of the parent's floor,
    // the floor ray's window up to the parent's floor + riser + L + 16 (L = the step's length: it reaches the next tread of a
    // 37 deg stair, B §2.3), the C12 layers, and the standing actors' discs (kWhyActor). From her start region (1.4.1) over-cap
    // water up to x.startLimit stays walkable.
    // (1.5.1) the window's top reaches L x tan 63 + 8 u over the parent too when that is more (the terrain allowance); the down
    // ray's answer may come from the cell cache; the navmesh is asked AT the floor found (within 48 u; on it = within 19 u and
    // one step: the same question as the line test's NavUnder, kept with the floor) - before 1.5.1 it was asked at the parent's
    // height; a floor that is terrain / ground with the navmesh on it may be as steep as n.z 0.45 (navTerr, counted).
    void XClassifyV(Job& j, RE::Actor* a_w, RE::bhkWorld* a_bw, XNode& m, const XNode& a_n)
    {
        ExState&    x = *j.x;
        const float zRef = a_n.z;
        m.zRef = zRef;
        m.why = kWhyNone;
        m.hasZ = false;
        m.onNav = false;
        m.navTerr = false;
        m.navStair = false;
        m.depth = 0.f;
        m.head = 0;
        m.deepOk = false;
        m.kind = kKUnknown;
        const auto none = [&m](std::uint8_t a_why) {
            m.kind = kKNone;
            m.why = a_why;
        };
        if (InKeepOut(j, m.x, m.y)) {
            none(kWhyKeepOut);
            return;
        }
        if (IsBadSpot(j, m.x, m.y, zRef)) {
            none(kWhyBad);
            return;
        }
        const float  L = (std::max)(kCell, Dist2D(XPos(a_n), XPos(m)));
        const float  band = (std::max)(x.riserT + L, TerrainRise(L));     // (x.riserT >= x.riser: covers 1.5.0's riser + L)
        const VFloor f = FloorFacts(j, a_bw, m.x, m.y, zRef + band + kXTopPad, zRef - (std::max)(kXFloorDown, kXFloorDownH * x.H));
        if (f.blocked) {
            none(kWhyBlocked);
            return;
        }
        if (!f.found) {
            none(kWhyNoFloor);
            return;
        }
        m.z = f.z;
        m.hasZ = true;
        m.onNav = FloorNav(j, a_w, f, m.x, m.y);       // the navmesh lies on THIS floor
        if (!m.onNav && !(j.xflags & kFloor)) {
            none(kWhyNoFloor);
            return;
        }
        m.navTerr = m.onNav && IsTerrainLayer(f.layer);
        if (f.nz < kXMinNz) {
            if (m.navTerr && f.nz >= kVTerrainNz) {
                ++j.xs.terrCells;                   // only the terrain allowance keeps this cell
            } else if (m.onNav && StairSlope(f.nz, f.layer)) {
                m.navStair = true;                  // (1.6.0 §5) only the stair allowance keeps it (the navmesh is on THIS floor)
                ++j.xs.stairCells;
            } else {
                none(kWhySteep);
                return;
            }
        }
        if (InActorDisc(x, m.x, m.y, m.z)) {
            none(kWhyActor);
            return;
        }
        m.kind = m.onNav ? kKNav : kKFloor;
        float depth = 0.f, surf = 0.f;
        if (!FloorDepth(j, a_w, f, m.x, m.y, depth, surf)) {
            none(kWhyNoCell);
            return;
        }
        if (f.fresh) NoteWater(j, f.h, surf);
        if (depth > kXWet) {
            m.depth = depth;
            if (!(j.xflags & kWater)) {
                none(kWhyDryOnly);
                return;
            }
            if (depth > x.depthCap) {
                if (a_n.deepOk && depth <= x.startLimit) {     // (1.4.1) her start region goes on here
                    m.kind = kKWater;
                    m.deepOk = true;
                    return;
                }
                none(kWhyDeep);
                return;
            }
            m.kind = kKWater;
        }
    }

    // 1.5.0 the step p -> m at pop - the same rules as the line test (audit C fix 2): m's headroom to Htop (once per cell), the
    // cell rule C2 (|dz| <= riser + L), the keep-out circle and the standing actors, m's own clearance (once per cell), the
    // floor at 1/3 and 2/3 (C1/C3), her 1.5.0 body (XBody), the brink pair. Doors as the plan allows. 0 clear (m.door = the
    // door it passed), 1 blocked, 2 m has no headroom, 3 blocked only BESIDE the step (a_allowNudge: C11 tries the step aside),
    // 4 m's own clearance failed (a_allowNudge: the cell is tried aside once, XCellNudgeTry).
    // (review fixes R1 #2 #3 #4, the coordinator's "one rule set": every step that is KEPT passes the full 1.5.0 rules - this
    // edge test now carries everything the line test does: the headroom (her bound top + 2, refusals with a doorway-like
    // clearance logged), the brink pair at the step's middle, the middle headroom, the clearance probes, and (once per cell,
    // cached) the floor at +-rc round m - a lip within rc of a drop is refused, or retried aside)
    // (1.5.1) a_terr: both ends are terrain on the navmesh - the profile and the brink pair may use the terrain allowance
    // (1.6.0) a_stair: an end of the step lies on a stair (or its profile used the stair allowance) - the rays beside her pass the
    // stair's own face and its brink floors may follow the stair's slope; x.stairUsed tells the caller
    int XStepRules(Job& j, RE::Actor* a_w, RE::bhkWorld* a_bw, const RE::NiPoint3& a_p, const RE::NiPoint3& a_m, float a_lim, bool a_terr = false,
                   bool a_stair = false)
    {
        ExState& x = *j.x;
        float    zMid = 0.f;
        x.stairUsed = false;
        if (!XProfile(j, a_w, a_bw, a_p, a_m, a_lim, zMid, a_terr)) return 1;
        const bool stair = a_stair || x.stairUsed;
        const bool terrSide0 = x.terrSide, stairSide0 = x.stairSide;
        x.terrSide = a_terr;                      // (1.5.1) the rays beside her step over the hillside of a terrain step
        x.stairSide = stair;                      // (1.6.0) ... and over a stair's own sloped face on a stair step
        const int  body = XBody(j, a_bw, a_p, a_m, zMid, true);
        x.terrSide = terrSide0;
        x.stairSide = stairSide0;
        if (body != 0) return body;
        const bool brinkOk = XBrinkClear(j, a_bw, a_p, a_m, zMid, a_terr, stair);
        if (stair) x.stairUsed = true;
        return brinkOk ? 0 : 2;
    }

    // 1.5.1 the cell cache's keys: a cell (its lattice square, its floor to 0.25 u, her size class, its terrain flag) and a step
    // (both ends' squares and floors, her size class, the step's terrain flag). Off the lattice (her start, a cell moved
    // aside, the goal) there is no key - such a test is never kept.
    bool CellKeyFor(const ExState& x, const XNode& a_m, CellKey& a_k)
    {
        if (!CacheLive(x) || !LatticeIndex(a_m.x, a_k.ix) || !LatticeIndex(a_m.y, a_k.iy)) return false;
        a_k.zq = ZKey(a_m.z);
        a_k.cls = x.cacheClass | (a_m.navTerr ? (1ull << 40) : 0ull) | (a_m.navStair ? (1ull << 41) : 0ull);   // (1.6.0: + stair)
        return true;
    }

    bool StepKeyFor(const ExState& x, const XNode& a_p, const XNode& a_m, bool a_terr, bool a_stair, StepKey& a_k)
    {
        if (!CacheLive(x) || !LatticeIndex(a_p.x, a_k.pix) || !LatticeIndex(a_p.y, a_k.piy) || !LatticeIndex(a_m.x, a_k.mix) ||
            !LatticeIndex(a_m.y, a_k.miy)) {
            return false;
        }
        a_k.pzq = ZKey(a_p.z);
        a_k.mzq = ZKey(a_m.z);
        a_k.cls = x.cacheClass | (a_terr ? (1ull << 40) : 0ull) | (a_stair ? (1ull << 41) : 0ull);   // (1.6.0: + stair)
        return true;
    }

    // (verifier N2) a cell's own tests - its headroom and its clearance - as a unit of their own (<= 9 rays, once per cell): the
    // step into it is then tested as the next unit (XEdgeTestV finds both cached)
    // (1.5.1) the answers come from the cell cache when kept (no question), and are kept when every hit was a fixed world layer
    // (a door seen through, an actor, clutter: never); the lip test uses the terrain allowance on a terrain cell (navTerr)
    void XCellTests(Job& j, RE::bhkWorld* a_bw, XNode& a_m)
    {
        ExState&   x = *j.x;
        CellKey    key;
        const bool keyed = CellKeyFor(x, a_m, key);
        if (keyed) {
            const auto it = g_cache.cells.find(key);
            if (it != g_cache.cells.end() && CacheFresh(x, it->second.t)) {
                const CellRec& e = it->second;
                if (a_m.head == 0) a_m.head = e.head;
                if (a_m.head == 1 && a_m.lip == 0) a_m.lip = e.lip;
                if (a_m.head == 2 || (a_m.head == 1 && a_m.lip != 0)) {
                    ++j.xs.cacheCellHits;
                    if (a_m.lip == 2) ++j.xs.lipFails;
                    return;
                }
            }
        }
        const int dirty0 = x.cacheDirty;
        x.doorsOk = (j.xflags & kDoors) != 0u;
        x.doorSide = false;
        x.curDoor = 0;
        if (a_m.head == 0) a_m.head = HeadBlocked(j, a_bw, a_m.x, a_m.y, a_m.z, "over a cell") ? 2 : 1;
        if (a_m.head == 1 && a_m.lip == 0) {
            a_m.lip = XCellClear(j, a_bw, a_m.x, a_m.y, a_m.z, a_m.navTerr, a_m.navStair) ? 1 : 2;
            if (a_m.lip == 2) ++j.xs.lipFails;
        }
        x.doorsOk = false;
        if (keyed && x.cacheDirty == dirty0 && x.curDoor == 0 && (a_m.head == 2 || a_m.lip != 0) &&
            CacheRoom(x, g_cache.cells, kVCacheCells, g_cache.curCell)) {
            CellRec e;
            e.head = a_m.head;
            e.lip = a_m.head == 1 ? a_m.lip : static_cast<std::uint8_t>(0);
            e.t = x.cacheNow;
            g_cache.cells.insert_or_assign(key, e);
            ++j.xs.cacheStores;
        }
    }

    // 1.5.1 XStepRules for a search step, through the cell cache: a kept answer for this very step (both ends' squares and floors,
    // her size class, the terrain flag) is reused - "blocked" as it is; "clear" / "blocked beside her" only after THIS plan's
    // own rules on its two floor samples (a spot she got stuck at, the water limit; the keep-out circle and the standing actors
    // were checked before, in XEdgeTestV). A test is kept when every hit was a fixed world layer, it passed no door and failed
    // on no plan rule.
    // (1.6.0: + a_stair - the step's stair flag is part of its key; a reused answer brings back its stair use and its side)
    int XStepRulesCached(Job& j, RE::Actor* a_w, RE::bhkWorld* a_bw, const XNode& a_p, const XNode& a_m, float a_lim, bool a_terr, bool a_stair)
    {
        ExState&           x = *j.x;
        const RE::NiPoint3 pp = XPos(a_p), mp = XPos(a_m);
        StepKey            key;
        const bool         keyed = StepKeyFor(x, a_p, a_m, a_terr, a_stair, key);
        if (keyed) {
            const auto it = g_cache.steps.find(key);
            if (it != g_cache.steps.end() && CacheFresh(x, it->second.t)) {
                const StepRec& e = it->second;
                ++j.xs.cacheStepHits;
                x.terrUsed = e.terr != 0;
                x.stairUsed = e.stair != 0;
                x.besideSide = e.side;
                if (e.r == 1) return 1;
                for (std::size_t k = 0; k < 2; ++k) {
                    const float t = static_cast<float>(k + 1) / 3.f;
                    if (IsBadSpot(j, pp.x + (mp.x - pp.x) * t, pp.y + (mp.y - pp.y) * t, e.z[k])) return 1;
                    if (e.d[k] > kXWet && (!(j.xflags & kWater) || e.d[k] > a_lim)) return 1;
                }
                return e.r;
            }
        }
        const int dirty0 = x.cacheDirty;
        x.planFail = false;
        x.terrUsed = false;
        x.besideSide = 0;
        const int r = XStepRules(j, a_w, a_bw, pp, mp, a_lim, a_terr, a_stair);
        if (keyed && x.cacheDirty == dirty0 && !x.planFail && x.curDoor == 0 && CacheRoom(x, g_cache.steps, kVCacheSteps, g_cache.curStep)) {
            StepRec e;
            e.r = static_cast<std::uint8_t>(r);
            e.terr = static_cast<std::uint8_t>(x.terrUsed ? 1 : 0);
            e.stair = static_cast<std::uint8_t>(x.stairUsed ? 1 : 0);
            e.side = static_cast<std::int8_t>(r == 2 ? x.besideSide : 0);
            e.z = x.profZ;
            e.d = x.profD;
            e.t = x.cacheNow;
            g_cache.steps.insert_or_assign(key, e);
            ++j.xs.cacheStores;
        }
        return r;
    }

    int XEdgeTestV(Job& j, RE::Actor* a_w, RE::bhkWorld* a_bw, const XNode& a_p, XNode& a_m, bool a_allowNudge)
    {
        ExState& x = *j.x;
        x.doorsOk = (j.xflags & kDoors) != 0u;
        x.doorSide = false;
        x.curDoor = 0;
        x.terrUsed = false;
        const auto out = [&x](int a_r) {
            x.doorsOk = false;
            return a_r;
        };
        if (a_m.head == 0) a_m.head = HeadBlocked(j, a_bw, a_m.x, a_m.y, a_m.z, "over a cell") ? 2 : 1;
        if (a_m.head == 2) return out(2);
        const RE::NiPoint3 pp = XPos(a_p), mp = XPos(a_m);
        const bool         terr = a_p.navTerr && a_m.navTerr;     // (1.5.1) both ends terrain on the navmesh
        const bool         stair = a_p.navStair || a_m.navStair;  // (1.6.0) an end on a stair (the stair allowance)
        const float        stepL = Dist2D(pp, mp), dz = std::fabs(a_m.z - a_p.z);
        if (dz > CellRise(x, stepL, terr)) return out(1);
        const bool c2Terr = dz > x.riser + stepL;                 // only the terrain allowance let the cell rule pass
        if (CrossesKeepOut(j, pp, mp) || CrossesActorDisc(x, pp, mp)) return out(1);
        if (a_m.lip == 0) {                       // the cell's own clearance (lip + walls), once (8 rays)
            a_m.lip = XCellClear(j, a_bw, a_m.x, a_m.y, a_m.z, a_m.navTerr, a_m.navStair) ? 1 : 2;
            if (a_m.lip == 2) ++j.xs.lipFails;
        }
        // 4 = the cell itself wants moving aside (ONE cell nudge, its own units, whatever the direction it is reached from)
        if (a_m.lip == 2) return out(a_allowNudge ? 4 : 1);
        const float lim = a_p.deepOk ? (std::max)(x.depthCap, x.startLimit) : x.depthCap;
        const int   rules = XStepRulesCached(j, a_w, a_bw, a_p, a_m, lim, terr, stair);
        if (rules == 1) return out(1);
        if (rules == 2) return out(a_allowNudge ? 3 : 1);
        if (c2Terr || x.terrUsed) ++j.xs.terrSteps;
        if (x.stairUsed) ++j.xs.stairSteps;
        a_m.door = x.curDoor;
        return out(0);
    }

    // C11 (verifier N2: the work is split into three units, the slice's share checked between them) - the step p -> m failed only
    // beside her; m is tried a_off u aside (across the step, B §5.4: the line shifted the same way at p too).
    // Unit 1 (XNudgeEnd, ~12 rays): the moved end stands on m's own floor (within half a riser) and the shifted start on p's; the
    // moved end passes the cell's own tests (keep-out, bad spots, the actors, the water, its headroom, its clearance). Both are
    // kept in x.nudgePs / x.nudgeMs. Unit 2 (XNudgeLine p -> ms): the line she walks, full rules. Unit 3 (XNudgeLine p -> ps):
    // the sideways jog, full rules (review R1 #5). All three pass -> XNudgeApply.
    // (1.5.1) a floor hit a_off u aside from a floor point at a_z0 is "the same floor" there: within half a riser of it and n.z >=
    // 0.70 - or, terrain / ground beside a terrain point on the navmesh (a_terr), within a_off x tan 63 + 8 u and n.z >= 0.45
    // (1.6.0 §5) a_stair: the point lies on a stair - a stair floor aside may lie up to StairRise(off) off it and be as steep as 0.66
    bool AsideFloorOk(const ExState& x, const FloorHit& a_h, float a_z0, float a_off, bool a_terr, bool a_stair = false)
    {
        if (a_h.blocked || !a_h.found) return false;
        const bool  t = a_terr && IsTerrainLayer(a_h.layer);
        const bool  st = a_stair && IsStairLayer(a_h.layer);
        float       tol = t ? (std::max)(x.riserT * 0.5f, TerrainRise(std::fabs(a_off))) : x.riser * 0.5f;
        if (st) tol = (std::max)(tol, StairRise(std::fabs(a_off)));
        if (std::fabs(a_h.z - a_z0) > tol) return false;
        return a_h.nz >= kXMinNz || (t && a_h.nz >= kVTerrainNz) || (st && a_h.nz >= kStairNz);
    }

    bool XNudgeEnd(Job& j, RE::Actor* a_w, RE::bhkWorld* a_bw, const XNode& a_p, const XNode& a_m, float a_off)
    {
        ExState&    x = *j.x;
        const float dx = a_m.x - a_p.x, dy = a_m.y - a_p.y;
        const float len = std::sqrt(dx * dx + dy * dy);
        if (len < 1.f) return false;
        const float ox = -dy / len * a_off, oy = dx / len * a_off;
        x.doorsOk = (j.xflags & kDoors) != 0u;
        x.doorSide = false;
        x.curDoor = 0;
        const auto out = [&x](bool a_r) {
            x.doorsOk = false;
            return a_r;
        };
        const FloorHit hp = XFloorRay(j, a_bw, a_p.x + ox, a_p.y + oy, a_p.z + x.riser + kXTopPad, a_p.z - x.riser - kXTopPad, j.xs.rN);
        if (!AsideFloorOk(x, hp, a_p.z, a_off, a_p.navTerr, a_p.navStair)) return out(false);
        const FloorHit hm = XFloorRay(j, a_bw, a_m.x + ox, a_m.y + oy, a_m.z + x.riser + kXTopPad, a_m.z - x.riser - kXTopPad, j.xs.rN);
        if (!AsideFloorOk(x, hm, a_m.z, a_off, a_m.navTerr, a_m.navStair)) return out(false);
        const RE::NiPoint3 ps{ a_p.x + ox, a_p.y + oy, hp.z }, ms{ a_m.x + ox, a_m.y + oy, hm.z };
        if (InKeepOut(j, ms.x, ms.y) || IsBadSpot(j, ms.x, ms.y, ms.z) || InActorDisc(x, ms.x, ms.y, ms.z)) return out(false);
        float depth = 0.f, surf = 0.f;
        if (!DepthAt(j, a_w, ms.x, ms.y, ms.z, depth, surf)) return out(false);
        if (depth > kXWet && (!(j.xflags & kWater) || depth > x.depthCap)) return out(false);
        if (HeadBlocked(j, a_bw, ms.x, ms.y, ms.z, "over a cell moved aside")) return out(false);
        if (!XCellClear(j, a_bw, ms.x, ms.y, ms.z, a_m.navTerr && IsTerrainLayer(hm.layer), a_m.navStair && IsStairLayer(hm.layer))) return out(false);
        x.nudgePs = ps;
        x.nudgeMs = ms;
        x.nudgeDepth = depth;
        x.nudgeOff = a_off;
        x.nudgeTerr = a_m.navTerr && IsTerrainLayer(hm.layer);   // (review #5) the moved end keeps navTerr only on land
        x.nudgeStair = a_m.navStair && IsStairLayer(hm.layer) && hm.nz < kXMinNz;    // (1.6.0) ... and navStair only on the stair
        return out(true);
    }

    // units 2 and 3 of a step nudge: ONE line p -> a_to with the full rules (doors as the plan allows; a_door = the one door this
    // line may pass too). <= ~17 rays + 2 water questions. true = clear (x.curDoor = the door it passed).
    // (1.5.1) a_terr: both ends terrain on the navmesh (the terrain allowance); never cached (off the lattice)
    bool XNudgeLine(Job& j, RE::Actor* a_w, RE::bhkWorld* a_bw, const XNode& a_p, const RE::NiPoint3& a_to, RE::FormID a_door, bool a_terr,
                    bool a_stair = false)
    {
        ExState& x = *j.x;
        x.doorsOk = (j.xflags & kDoors) != 0u;
        x.doorSide = false;
        x.curDoor = a_door;
        x.terrUsed = false;
        const float lim = a_p.deepOk ? (std::max)(x.depthCap, x.startLimit) : x.depthCap;
        const bool  ok = XStepRules(j, a_w, a_bw, XPos(a_p), a_to, lim, a_terr, a_stair) == 0;
        x.doorsOk = false;
        return ok;
    }

    // the step nudge fitted: m moves to x.nudgeMs and remembers how far; its door = the walked line's
    void XNudgeApply(const ExState& x, XNode& a_m)
    {
        a_m.x = x.nudgeMs.x;
        a_m.y = x.nudgeMs.y;
        a_m.z = x.nudgeMs.z;
        a_m.lip = 1;
        a_m.nudge = x.nudgeOff;
        a_m.navTerr = x.nudgeTerr;        // (1.5.1, review #5) its new floor decides the terrain allowance
        a_m.navStair = x.nudgeStair;      // (1.6.0) ... and the stair allowance
        if (x.nudgeDepth > kXWet) {
            a_m.depth = x.nudgeDepth;
            a_m.kind = kKWater;
        } else {
            a_m.depth = 0.f;
            a_m.kind = a_m.onNav ? kKNav : kKFloor;
        }
        a_m.door = x.nudgeDoor;
    }

    // (review fixes) spot k (0..7: 8 u then 16 u east, west, north, south) beside a floor point a_p: its floor (within half a riser
    // of a_p, n.z >= 0.70), outside keep-out / bad spots / actors, its water within the cap (none when dry-only), its headroom and
    // its own clearance. ~10 rays. true = fine (a_out on its floor, a_depth = the water there). Used to move a route cell whose
    // own clearance failed (XCellNudgeTry) and (verifier N1) a MIX corridor point.
    // (1.5.1) a_terr: the point is terrain on the navmesh - a terrain spot aside may lie up to 8 / 16 u x tan 63 + 8 u off it
    // (1.6.0 §5) a_stair: the point lies on a stair - a stair floor aside (n.z >= 0.66, within the stair's slope) is the same floor;
    // x.asideStair says whether the spot found keeps the stair allowance
    bool XSpotAside(Job& j, RE::Actor* a_w, RE::bhkWorld* a_bw, const RE::NiPoint3& a_p, int a_k, RE::NiPoint3& a_out, float& a_depth,
                    bool a_terr, bool a_stair = false)
    {
        ExState&        x = *j.x;
        constexpr float kSpots[8][2] = { { 8.f, 0.f }, { -8.f, 0.f }, { 0.f, 8.f }, { 0.f, -8.f }, { 16.f, 0.f }, { -16.f, 0.f }, { 0.f, 16.f }, { 0.f, -16.f } };
        const float     cx = a_p.x + kSpots[a_k][0], cy = a_p.y + kSpots[a_k][1];
        x.doorsOk = (j.xflags & kDoors) != 0u;
        x.doorSide = false;
        const auto out = [&x](bool a_r) {
            x.doorsOk = false;
            return a_r;
        };
        const float    off = std::fabs(kSpots[a_k][0]) + std::fabs(kSpots[a_k][1]);
        const float    win = a_stair ? (std::max)(x.riser, StairRise(off)) : x.riser;
        const FloorHit h = XFloorRay(j, a_bw, cx, cy, a_p.z + win + kXTopPad, a_p.z - win - kXTopPad, j.xs.rN);
        if (!AsideFloorOk(x, h, a_p.z, off, a_terr, a_stair)) return out(false);
        if (InKeepOut(j, cx, cy) || IsBadSpot(j, cx, cy, h.z) || InActorDisc(x, cx, cy, h.z)) return out(false);
        float depth = 0.f, surf = 0.f;
        if (!DepthAt(j, a_w, cx, cy, h.z, depth, surf)) return out(false);
        if (depth > kXWet && (!(j.xflags & kWater) || depth > x.depthCap)) return out(false);
        if (HeadBlocked(j, a_bw, cx, cy, h.z, "over a spot moved aside")) return out(false);
        const bool stairHere = a_stair && IsStairLayer(h.layer) && h.nz < kXMinNz;
        if (!XCellClear(j, a_bw, cx, cy, h.z, a_terr && IsTerrainLayer(h.layer), stairHere)) return out(false);
        a_out = RE::NiPoint3{ cx, cy, h.z };
        a_depth = depth > kXWet ? depth : 0.f;
        x.asideTerr = a_terr && IsTerrainLayer(h.layer);   // (review #5) the spot keeps the terrain allowance only on land
        x.asideStair = stairHere;                          // (1.6.0) ... and the stair allowance only on the stair
        return out(true);
    }

    // (review fixes) a cell whose OWN clearance failed (a wall or a lip within rc) is tried at one of its 8 spots aside, once for
    // the cell, whatever direction it is reached from (one spot per unit of work). true = the cell moves there (the step into it
    // is then tested again, from its pop).
    bool XCellNudgeTry(Job& j, RE::Actor* a_w, RE::bhkWorld* a_bw, XNode& a_m, int a_k)
    {
        RE::NiPoint3 sp;
        float        depth = 0.f;
        if (!XSpotAside(j, a_w, a_bw, XPos(a_m), a_k, sp, depth, a_m.navTerr, a_m.navStair)) return false;
        a_m.nudge = Dist2D(XPos(a_m), sp);
        a_m.x = sp.x;
        a_m.y = sp.y;
        a_m.z = sp.z;
        a_m.navTerr = j.x->asideTerr;     // (1.5.1, review #5)
        a_m.navStair = j.x->asideStair;   // (1.6.0)
        a_m.lip = 1;
        a_m.head = 1;
        if (depth > 0.f) {
            a_m.depth = depth;
            a_m.kind = kKWater;
        } else {
            a_m.depth = 0.f;
            a_m.kind = a_m.onNav ? kKNav : kKFloor;
        }
        return true;
    }

    // C9: the level of square a_col that cell n reaches without a question - one classified from about n's height (its zRef
    // within a riser of n's floor). -1 = none yet.
    std::int32_t XLevelFind(const ExState& x, std::int32_t a_col, const XNode& a_n)
    {
        for (std::int32_t k = a_col; k >= 0; k = x.nodes[static_cast<std::size_t>(k)].next) {
            const XNode& c = x.nodes[static_cast<std::size_t>(k)];
            if (c.kind != kKUnknown && std::fabs(c.zRef - a_n.z) <= x.riser) return k;
        }
        return -1;
    }

    // C9: classify square a_col from cell n's level - ONE floor ray. The floor found is the square's level already known for
    // that surface (within kVLevelTol), else a NEW level (up to 4 per square, from the plan's pool); a level that died before it
    // was ever put on the open list may give its place. A floor under another one closer than her height is kept too: its own
    // headroom ray refuses it, and it is never asked again. -1 = no room left (counted, the step is not taken).
    std::int32_t XLevelClassify(Job& j, RE::Actor* a_w, RE::bhkWorld* a_bw, std::int32_t a_col, const XNode& a_n)
    {
        ExState& x = *j.x;
        XNode&   head = x.nodes[static_cast<std::size_t>(a_col)];
        if (head.kind == kKUnknown) {            // the square's first look
            XClassifyV(j, a_w, a_bw, head, a_n);
            return a_col;
        }
        XNode t;
        t.col = a_col;
        t.x = x.minX + static_cast<float>(a_col % x.nx) * kCell;
        t.y = x.minY + static_cast<float>(a_col / x.nx) * kCell;
        XClassifyV(j, a_w, a_bw, t, a_n);
        int          levels = 0;
        std::int32_t last = -1, dead = -1;
        for (std::int32_t k = a_col; k >= 0; k = x.nodes[static_cast<std::size_t>(k)].next) {
            const XNode& c = x.nodes[static_cast<std::size_t>(k)];
            ++levels;
            last = k;
            if (t.hasZ && c.hasZ && std::fabs(c.z - t.z) <= kVLevelTol) return k;     // the same surface: that level
            if (dead < 0 && c.kind == kKNone && !c.pushed && !c.closed) dead = k;
        }
        if (levels < kVMaxLevels && x.nodes.size() < x.nodes.capacity()) {      // never past the reserve: no reference moves
            const auto ni = static_cast<std::int32_t>(x.nodes.size());
            x.nodes.push_back(t);
            x.nodes[static_cast<std::size_t>(last)].next = ni;
            ++j.xs.levelsExtra;
            return ni;
        }
        if (dead >= 0) {
            XNode&             d = x.nodes[static_cast<std::size_t>(dead)];
            const std::int32_t keepNext = d.next;
            d = t;
            d.next = keepNext;
            return dead;
        }
        ++j.xs.levelFull;
        return -1;
    }

    // the lattice direction of the step a_from -> a_to (their squares)
    int DirIndexV(const ExState& x, std::int32_t a_from, std::int32_t a_to)
    {
        const std::int32_t fc = x.nodes[static_cast<std::size_t>(a_from)].col, tc = x.nodes[static_cast<std::size_t>(a_to)].col;
        const int          dx = (tc % x.nx) - (fc % x.nx), dy = (tc / x.nx) - (fc / x.nx);
        for (int d = 0; d < 8; ++d) {
            if (kDir[d][0] == dx && kDir[d][1] == dy) return d;
        }
        return -1;
    }

    // (1.6.0) the 1.5.0 search's heuristic: weighted A* toward the goal - a wade scan is a plain Dijkstra (0: the nearest wins)
    float XHeur(const ExState& x, const XNode& a_m)
    {
        if (x.wadeScan) return 0.f;
        // (1.6.1 A2) after the flood found the water: the floor distance from this square to it along the flood's own steps (round
        // a ledge, never across it), where the flood stood on this square's level; elsewhere the flat distance as before
        if (!x.floodToE.empty() && !x.local) {
            const auto c = static_cast<std::size_t>(a_m.col);
            if (a_m.col >= 0 && c < x.floodToE.size() && c < x.floodZ.size() && x.floodToE[c] < 1.0e37f &&
                std::fabs(a_m.z - x.floodZ[c]) <= x.riser + kVRingLevel) {
                return kHeurWeight * x.floodToE[c];
            }
        }
        return kHeurWeight * Dist2D(XPos(a_m), x.goal.pos);
    }
    // (1.6.0 §4 #5) pass 1 searches only its own box of the (wider) grid
    bool XInBox(const ExState& x, int a_ix, int a_iy) { return a_ix >= x.ax0 && a_iy >= x.ay0 && a_ix <= x.ax1 && a_iy <= x.ay1; }
    // (1.6.0 §6.3) a wade scan keeps to its disc round its root
    bool XInScan(const ExState& x, float a_x, float a_y) { return !x.wadeScan || std::hypot(a_x - x.scanRX, a_y - x.scanRY) <= x.scanR; }

    // 1.5.0 XReopen: m may still be reached from the closed levels of its neighbouring squares that are within the cell rule
    void XReopenV(Job& j, std::int32_t a_m)
    {
        ExState&  x = *j.x;
        XNode&    m = x.nodes[static_cast<std::size_t>(a_m)];
        const int mx = m.col % x.nx, my = m.col / x.nx;
        m.bestG = 3.0e38f;
        for (int d = 0; d < 8; ++d) {
            if (m.rejectedFrom & (1u << d)) continue;
            const int qx = mx - kDir[d][0], qy = my - kDir[d][1];      // q --(direction d)--> m
            if (qx < 0 || qy < 0 || qx >= x.nx || qy >= x.ny) continue;
            for (std::int32_t qi = qy * x.nx + qx; qi >= 0; qi = x.nodes[static_cast<std::size_t>(qi)].next) {
                const XNode& q = x.nodes[static_cast<std::size_t>(qi)];
                if (!q.closed || std::fabs(m.z - q.z) > CellRise(x, Dist2D(XPos(q), XPos(m)), q.navTerr && m.navTerr)) continue;
                if (m.deepOk && !q.deepOk) continue;     // (1.4.1) her start region is never entered from outside it
                const float ng = q.g + XStepCost(x, q, m);
                if (ng < m.bestG) m.bestG = ng;
                x.open.push({ ng + XHeur(x, m), ng, a_m, qi });
            }
        }
    }

    // 1.5.0 XExpandStep with several floors per square: each neighbour square gives the level n reaches (XLevelFind, else ONE
    // classification - its own unit of work), recorded per direction (x.expNbr: the wade patch reads it); n is tagged `edge`
    // when a neighbour lies lower than the cell rule allows (C2) or has no floor; LEDGE lines as 1.4.1; then the neighbours
    // within the cell rule are pushed. true = done; false = this slice's share is used (x.expD / x.expProbe keep the place).
    bool XExpandStepV(Job& j, RE::Actor* a_w, RE::bhkWorld* a_bw)
    {
        ExState&           x = *j.x;
        const std::int32_t idx = x.expIdx;
        XNode&             n = x.nodes[static_cast<std::size_t>(idx)];
        const int          ix = n.col % x.nx, iy = n.col / x.nx;
        const int          ledgeMax = (j.xflags & kVerbose) ? kXLedgeLinesV : kXLedgeLines;
        while (x.expD < 8) {
            const int  d = x.expD;
            const int  mx = ix + kDir[d][0], my = iy + kDir[d][1];
            const bool inGrid = mx >= 0 && my >= 0 && mx < x.nx && my < x.ny;
            if (!inGrid || !XInBox(x, mx, my)) {
                if (x.expNbr[static_cast<std::size_t>(d)] == kNbrUnset) {
                    ++x.edgeCells;                // the search reached its box's edge
                    // (1.6.0 §4 #5) a neighbour in the grid but outside pass 1's box: pass 2 expands this cell again
                    if (inGrid && x.canGrow && !n.frontier) {
                        n.frontier = true;
                        x.frontier.push_back(idx);
                    }
                }
                x.expNbr[static_cast<std::size_t>(d)] = -1;
                ++x.expD;
                continue;
            }
            if (x.expProbe == d) {                // the LEDGE probe owed toward this neighbour
                if (Spent(j)) return false;
                x.expProbe = -1;
                const float       dl = std::sqrt(static_cast<float>(kDir[d][0] * kDir[d][0] + kDir[d][1] * kDir[d][1]));
                const float       ux = static_cast<float>(kDir[d][0]) / dl, uy = static_cast<float>(kDir[d][1]) / dl;
                const std::string what = ClassifyStep(j, a_w, a_bw, XPos(n), ux, uy);
                logger::info("PATH #{} LEDGE: from ({:.0f},{:.0f},{:.0f}) toward ({:.2f},{:.2f}): {}", j.handle, n.x, n.y, n.z, ux, uy, what);
                ++x.expD;
                continue;
            }
            if (x.expNbr[static_cast<std::size_t>(d)] == kNbrUnset) {
                const std::int32_t cm = my * x.nx + mx;
                std::int32_t       mi = XLevelFind(x, cm, n);
                if (mi < 0) {
                    if (Spent(j)) return false;
                    mi = XLevelClassify(j, a_w, a_bw, cm, n);
                }
                x.expNbr[static_cast<std::size_t>(d)] = mi;
                if (mi >= 0) {
                    XNode& m = x.nodes[static_cast<std::size_t>(mi)];
                    // (1.4.1) a cell refused as too deep from outside her start region joins it when touched from inside it
                    if (!m.closed && m.kind == kKNone && m.why == kWhyDeep && n.deepOk && m.hasZ && m.depth <= x.startLimit) {
                        m.kind = kKWater;
                        m.why = kWhyNone;
                        m.deepOk = true;
                    }
                    // (1.5.1) the cell rule with the terrain allowance between two terrain cells on the navmesh
                    const bool  terr = n.navTerr && m.navTerr;
                    const float lim = CellRise(x, Dist2D(XPos(n), XPos(m)), terr);
                    const bool  noFloor = m.kind == kKNone && m.why == kWhyNoFloor;
                    // (review R1 #7) `edge` = a neighbour more than riser + 4 u lower (30 u: a porch, a terrace wall, a lip) or
                    // no floor - not only past the cell rule (58 / 71 u); the LEDGE probes still name only what the cell rule refuses
                    // (1.5.1: between two terrain cells on the navmesh a slope within the allowance is no brink - only past it)
                    if (noFloor || (m.hasZ && m.z < n.z - (terr ? lim : x.riser + kVEdgeDropPad))) n.edge = true;
                    const bool refused = noFloor || (m.hasZ && (m.z < n.z - lim || m.z > n.z + lim));
                    if (refused && j.xs.ledges < ledgeMax) {
                        ++j.xs.ledges;
                        x.expProbe = d;           // its probe runs as the next unit
                        continue;
                    }
                }
            }
            ++x.expD;
        }
        for (int d = 0; d < 8; ++d) {
            const std::int32_t mi = x.expNbr[static_cast<std::size_t>(d)];
            if (mi < 0) continue;
            XNode& m = x.nodes[static_cast<std::size_t>(mi)];
            if (m.closed || m.kind == kKNone || m.kind == kKUnknown || (m.rejectedFrom & (1u << d))) continue;
            if (std::fabs(m.z - n.z) > CellRise(x, Dist2D(XPos(n), XPos(m)), n.navTerr && m.navTerr)) continue;     // C2 (1.5.1: terrain)
            if (m.deepOk && !n.deepOk) continue;     // (1.4.1) her start region is never re-entered once the route has left it
            if (!XInScan(x, m.x, m.y)) continue;     // (1.6.0 §6.3) a wade scan stays within its disc
            const float ng = n.g + XStepCost(x, n, m);
            if (ng < m.bestG) {
                m.bestG = ng;
                m.pushed = true;
                x.open.push({ ng + XHeur(x, m), ng, mi, idx });
            }
        }
        return true;
    }

    // the cell path start ... a_end (+ the goal); the start keeps her own point
    void XBuildPath(Job& j, std::int32_t a_end, bool a_toGoal)
    {
        ExState& x = *j.x;
        x.cells.clear();
        for (std::int32_t k = a_end; k >= 0; k = x.nodes[static_cast<std::size_t>(k)].parent) {
            const XNode& c = x.nodes[static_cast<std::size_t>(k)];
            PCell        pc;
            pc.pos = XPos(c);
            pc.kind = c.kind;
            pc.depth = c.depth;
            pc.onNav = c.onNav;
            pc.edge = c.edge;
            pc.deep = c.deepOk;
            pc.door = c.door;             // 1.5.0 (0 for a 1.4.1 plan)
            pc.navTerr = c.navTerr;       // 1.5.1 (false for a 1.4.1 plan)
            pc.stair = c.navStair;        // 1.6.0: a cell on a stair - the step into it is a stair step
            x.cells.push_back(pc);
        }
        std::reverse(x.cells.begin(), x.cells.end());
        const bool startEdge = x.cells.front().edge;
        x.cells.front() = x.start;
        x.cells.front().edge = startEdge;
        if (a_toGoal) x.cells.push_back(x.goal);
    }

    // 1.6.0 §6.3 a wade scan's chain: its root ... a_end as cells (the root kept as it is - XBuildPath puts her start there)
    std::vector<PCell> XChain(const ExState& x, std::int32_t a_end)
    {
        std::vector<PCell> out;
        for (std::int32_t k = a_end; k >= 0 && out.size() < x.nodes.size(); k = x.nodes[static_cast<std::size_t>(k)].parent) {
            const XNode& c = x.nodes[static_cast<std::size_t>(k)];
            PCell        pc;
            pc.pos = XPos(c);
            pc.kind = c.kind;
            pc.depth = c.depth;
            pc.onNav = c.onNav;
            pc.edge = c.edge;
            pc.deep = c.deepOk;
            pc.door = c.door;
            pc.navTerr = c.navTerr;
            pc.stair = c.navStair;
            out.push_back(pc);
        }
        std::reverse(out.begin(), out.end());
        return out;
    }

    // no way to the goal: a partial route (B §7.6) when asked for and worth it, else no route. Its end is stand-spot tested
    // next (kXPartialEnd, review G BUG 4).
    void XNoWay(Job& j, const char* a_why)
    {
        ExState&    x = *j.x;
        const float d0 = Dist2D(x.start.pos, x.goal.pos);
        if ((j.xflags & kPartial) && x.best >= 0 && x.nodes[static_cast<std::size_t>(x.best)].parent >= 0 &&
            d0 - x.bestD >= (std::max)(kXPartialMin, kXPartialFrac * d0)) {
            x.partial = true;
            x.noWayWhy = a_why;
            XBuildPath(j, x.best, false);
            x.partialTries = 0;
            x.xphase = kXPartialEnd;
            return;
        }
        if (x.v15 && (j.xflags & kPartial)) {     // 1.5.0 item 1: a partial end only on the goal's own floor
            ExFinish(j, 0, std::format("{}; no partial route: nothing she reached on the goal's floor (+-{:.0f} u) is nearer it by {:.0f} u or more",
                                       a_why, x.riser + kVLevelBand, (std::max)(kXPartialMin, kXPartialFrac * d0)));
            return;
        }
        ExFinish(j, 0, a_why);
    }

    // (review G, BUG 4) B §5.3 / §7.6: a partial route ends at a PLACE TO STAND. Its last cell is stand-spot tested; when
    // it fails the route steps back one cell (at most kXPartialTries times). It must still end >= max(96 u, 25 %) nearer the
    // goal than she is. Nothing passes -> no route, with the reason.
    void XPhasePartialEnd(Job& j, RE::Actor* a_w, RE::bhkWorld* a_bw)
    {
        ExState&    x = *j.x;
        const float d0 = Dist2D(x.start.pos, x.goal.pos);
        while (x.xphase == kXPartialEnd) {
            if (Spent(j)) return;
            const PCell& e = x.cells.back();
            // (1.5.0 item 1: and on the goal's own floor - a cell stepped back off it is no end)
            if (x.cells.size() < 2 || d0 - Dist2D(e.pos, x.goal.pos) < (std::max)(kXPartialMin, kXPartialFrac * d0) ||
                (x.v15 && std::fabs(e.pos.z - x.goal.pos.z) > x.riser + kVLevelBand)) {
                ExFinish(j, 0, std::format("{}; no partial route: {}", x.noWayWhy,
                                           x.partialWhy.empty() ? std::string("no place to stand near enough the goal") :
                                                                  "its nearest ends are not places to stand (" + x.partialWhy + ")"));
                return;
            }
            float       fz = 0.f, dep = 0.f;
            bool        hf = false;
            std::string why;
            // (1.4.1) an end in her start region is a place she can stand at its own limit (she stands in such water already)
            if (StandSpot(j, a_w, a_bw, e.pos.x, e.pos.y, e.pos.z + x.step + kXTopPad, e.pos.z - x.step - kXTopPad, fz, dep, hf, why, nullptr,
                          e.deep ? x.startLimit : 0.f)) {
                if (x.partialTries > 0) {
                    logger::info("PATH #{} partial end stepped back {} cell(s) to a place to stand (the last one failed: {})", j.handle,
                                 x.partialTries, x.partialWhy);
                }
                x.xphase = kXSplit;
                return;
            }
            x.partialWhy = why;
            if (++x.partialTries >= kXPartialTries) {
                ExFinish(j, 0, std::format("{}; no partial route: its {} nearest ends are not places to stand (last: {})", x.noWayWhy,
                                           kXPartialTries, why));
                return;
            }
            x.cells.pop_back();
        }
    }

    // A navmesh-only answer as route points (1.3.0 waypoints, or the goal of a clear straight line): NAV, dry, on the
    // navmesh; engine legs from the start while each leg is >= 64 u (at most kXMaxPrefix), the switch after them
    // (ALL_DIRECT: the switch is 0).
    void NavPoints(const RE::NiPoint3& a_start, const std::vector<RE::NiPoint3>& a_pts, bool a_goalMoved, bool a_allDirect,
                   std::vector<RoutePoint>& a_out, int& a_switch, float& a_len)
    {
        a_out.clear();
        a_len = 0.f;
        std::size_t  k = 0;
        RE::NiPoint3 prev = a_start;
        while (!a_allDirect && k < a_pts.size() && k < static_cast<std::size_t>(kXMaxPrefix) && Dist2D(prev, a_pts[k]) >= kMinLeg) prev = a_pts[k++];
        a_switch = static_cast<int>(k);
        prev = a_start;
        for (std::size_t i = 0; i < a_pts.size(); ++i) {
            std::uint32_t f = kPtOnNavmesh;
            if (i < k) f |= kPtEngineLeg;
            if (i + 1 == a_pts.size()) {
                f |= kPtGoal;
                if (a_goalMoved) f |= kPtGoalMoved;
            }
            a_out.push_back(RoutePoint{ a_pts[i], Kind::kNav, 0.f, f });
            a_len += Dist2D(prev, a_pts[i]);
            prev = a_pts[i];
        }
    }

    // a finished plan of the 1.3.0 search (Start, or StartEx without kFloor / kWater) as route points
    void LegacyPoints(const Job& j, std::vector<RoutePoint>& a_out, int& a_switch, float& a_len)
    {
        std::vector<RE::NiPoint3> src;
        if (j.status == 2) {
            src = j.out;
        } else if (j.status == 1) {
            src.push_back(j.goal);
        }
        NavPoints(j.start, src, false, j.ex && (j.xflags & kAllDirect) != 0u, a_out, a_switch, a_len);
    }

    // ---- the phases (B §7.2) -------------------------------------------------------------------------------------------

    // 0a: her numbers, her bad spots; swimming = refused
    void XPhaseMeasure(Job& j, RE::Actor* a_w)
    {
        ExState& x = *j.x;
        MeasureH(j, a_w, x.H, x.Hraw, x.step);
        x.depthCap = j.depthCap;
        MeasureSwim(x, a_w);
        std::string v15note;
        if (j.xflags & kV15Flags) {
            // 1.5.0: her body numbers (the "step" printed below is then the riser), the wade numbers, the calibration line
            // (1.5.1: the riser from her hull's cone, read here on the main thread for every such plan; then the cell cache)
            j.cone = ReadHull(a_w, nullptr);
            ConeMemo(j);
            MeasureV(j, x);
            CacheBegin(j, a_w, x);
            x.wade = (j.xflags & kWadeSpot) != 0u;
            if (x.wade) {
                const float mf = std::clamp(j.wadeMinFrac > 0.f ? j.wadeMinFrac : 0.15f, kXDepthFracMin, j.depthFrac);
                x.wadeMin = (std::min)(x.depthCap, (std::max)(kXWet + 0.5f, mf * x.H));
                x.wadeNbrMin = (std::max)(kXWet, kVWadeNbrFrac * x.H);
                x.wadePatchMin = std::clamp(j.wadePatchMin, 0, 8);
                // (1.6.0 §6.2) the depth bands x her height: knee = scan 1's least, thigh reached; the thigh MAX = the cap
                x.wadeKnee = std::clamp(j.wadeKneeFrac, 0.f, 1.f) * x.H;
                x.wadeThigh = std::clamp(j.wadeThighFrac, 0.f, 1.f) * x.H;
            }
            v15note = std::format(" | 1.5.0 rules (flags{}{}{}{}): {}, clearance {:.0f} u, headroom {:.0f} u ({}){}", (j.xflags & kNavGuide) ? " NAVGUIDE" : "",
                                  x.wade ? " WADE" : "", (j.xflags & kDoors) ? " DOORS" : "", (j.xflags & kActorsKeepOut) ? " ACTORS" : "",
                                  RiserNote(j, x), x.rc, x.htop, x.htopFromBound ? "her bound top + 2" : "H + 2",
                                  x.wade ? std::format(", wade depth {:.0f}-{:.0f} u, patch >= {} of 8 at >= {:.0f} u; 1.6.0 bands: knee {:.0f} u, thigh "
                                                       "{:.0f}-{:.0f} u, scans {}",
                                                       x.wadeMin, x.depthCap, x.wadePatchMin, x.wadeNbrMin, x.wadeKnee, x.wadeThigh, x.depthCap,
                                                       (j.wadeScanBodies > 0.f && x.depthCap >= x.wadeKnee) ?
                                                           std::format("within {:.0f} u ({:.1f} body lengths)", j.wadeScanBodies * x.H, j.wadeScanBodies) :
                                                           std::string("off (the cap is under knee depth, or none asked)")) :
                                           std::string());
            v15note += std::format(" | 1.5.1: terrain on the navmesh n.z >= {:.2f}, rise <= h x tan 63 + {:.0f} u; range {:.0f} u; cell cache {} ({} kept answer(s))",
                                   kVTerrainNz, kVTerrainPad, kVMaxDistance, x.cacheOn ? "on" : "off (no physics world)", g_cache.Size());
            v15note += std::format(" | 1.6.0: stair allowance n.z >= {:.2f} (a fixed floor with the navmesh under it); stand spots {:.2f} ({:.2f} on "
                                   "navmesh terrain); slices by time ({:.1f} ms a frame shared)",
                                   kStairNz, kStandNz, x.standNzTerr, kXFrameUs / 1000.0);
            Calibrate(j, a_w, x);
        }
        SnapshotBad(j);
        auto*      st = a_w->AsActorState();
        const bool swimming = st && st->IsSwimming();
        logger::info("PATH #{} measured: H {:.0f} u (GetHeight {:.1f}{}), scale {:.2f}, radius {:.0f} u, step {:.0f} u, depthCap {:.0f} u "
                     "({:.2f} x H), swims from {:.0f} u of water (0.70 x {:.1f} u), {} live bad spot(s){}{}",
                     j.handle, x.H, x.Hraw, (x.Hraw >= kXHMin && x.Hraw <= kXHMax) ? "" : " - outside 48-400, 128 x scale used", j.scale,
                     j.radius, x.step, x.depthCap, j.depthFrac, x.swimDepth, x.swimDiv, x.bad.size(), swimming ? ", SHE IS SWIMMING" : "", v15note);
        if (swimming) {
            j.swimStart = true;   // 1.4.1: the walk answers O 21 (she would swim), not O 1
            ExFinish(j, 0, "she is swimming (a route must never ask for swimming)");
            return;
        }
        x.xphase = kXWaters;
    }

    bool IsWaterActivator(RE::TESObjectREFR* a_ref)
    {
        if (!a_ref || a_ref->IsDisabled() || a_ref->IsDeleted()) return false;
        auto* base = a_ref->GetBaseObject();
        auto* acti = base ? base->As<RE::TESObjectACTI>() : nullptr;
        return acti && acti->waterForm;
    }

    bool Footprint(RE::TESObjectREFR* a_ref, WaterFP& a_fp)
    {
        auto* base = a_ref ? a_ref->GetBaseObject() : nullptr;
        if (!base) return false;
        const float sc = a_ref->GetScale();
        const auto& bd = base->boundData;
        a_fp.hx = (std::max)(8.f, static_cast<float>(bd.boundMax.x - bd.boundMin.x) * 0.5f * sc);
        a_fp.hy = (std::max)(8.f, static_cast<float>(bd.boundMax.y - bd.boundMin.y) * 0.5f * sc);
        const float ang = a_ref->GetAngleZ();
        a_fp.c = std::cos(ang);
        a_fp.s = std::sin(ang);
        const float lx = static_cast<float>(bd.boundMin.x + bd.boundMax.x) * 0.5f * sc;
        const float ly = static_cast<float>(bd.boundMin.y + bd.boundMax.y) * 0.5f * sc;
        const auto  p = a_ref->GetPosition();
        a_fp.cx = p.x + a_fp.c * lx + a_fp.s * ly;      // world = [[c, s], [-s, c]] x model (Water Finder's ToWorld)
        a_fp.cy = p.y - a_fp.s * lx + a_fp.c * ly;
        a_fp.surf = p.z + static_cast<float>(bd.boundMax.z) * sc;   // vanilla water planes: max z = 0
        return true;
    }

    // 1.5.0 C13 / item 6 (kActorsKeepOut): actors standing still at plan start, in her loaded space and in the box, become
    // discs (their extents.x + rc + 8); the walker and the walk's own target (it has the keep-out circle) are left out. A disc
    // never holds her start or the goal: it shrinks, and under her radius it is dropped (she must be able to leave and to
    // arrive). Eligibility as Threats.cpp (VR-run): the high and middle-high process lists + the player, 3D loaded, her cell
    // (interior) or her worldspace (exterior), not dead, not disabled; "standing" = Actor::IsMoving() false. Main thread.
    void ScanActors(Job& j, RE::Actor* a_w, float a_x0, float a_x1, float a_y0, float a_y1)
    {
        ExState& x = *j.x;
        auto*    cellW = a_w->GetParentCell();
        if (!cellW) return;
        std::vector<RE::FormID> seen;
        std::string             ids;
        const auto consider = [&](RE::Actor* a) {
            if (!a || a == a_w || !a->Is3DLoaded()) return;
            const RE::FormID id = a->GetFormID();
            if (id == j.avoidId || std::find(seen.begin(), seen.end(), id) != seen.end()) return;
            seen.push_back(id);
            auto* cellA = a->GetParentCell();
            if (!cellA) return;
            if (cellW->IsInteriorCell() || cellA->IsInteriorCell()) {
                if (cellW != cellA) return;
            } else if (a->GetWorldspace() != a_w->GetWorldspace()) {
                return;
            }
            if (a->IsDead() || a->IsDisabled()) return;
            const RE::NiPoint3 p = a->GetPosition();
            if (p.x < a_x0 || p.x > a_x1 || p.y < a_y0 || p.y > a_y1) return;
            if (a->IsMoving()) {
                ++j.xs.actorsMoving;          // a moving one is the driver's business (stall -> bad spot -> re-plan)
                return;
            }
            float ar = 20.f;
            if (auto* acc = a->GetCharController()) {
                const float e = acc->collisionBound.extents.x;
                if (e > 4.f && e < 96.f) ar = e;
            }
            float r = ar + x.rc + kVActorPad;
            r = (std::min)(r, Dist2D(j.start, p) - 1.f);
            r = (std::min)(r, Dist2D(x.asked, p) - 1.f);
            if (r < j.radius) return;
            ExState::Disc d;
            d.x = p.x;
            d.y = p.y;
            d.z = p.z;
            d.r = r;
            d.id = id;
            x.discs.push_back(d);
            if (x.discs.size() <= 8) ids += std::format(" {:08X} r {:.0f} u", id, r);
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
        j.xs.actorsKept = static_cast<int>(x.discs.size());
        logger::info("PATH #{} actors: {} standing still kept out{}{}, {} moving one(s) left to the walk", j.handle, x.discs.size(),
                     x.discs.empty() ? "" : " -", ids, j.xs.actorsMoving);
    }

    // 0b: the placed water activators near her line, collected ONCE (pointers inside the cells' locks, footprints after;
    // nothing kept but numbers). Interior: her cell (any placed water there = its cell plane is never trusted). Exterior:
    // the loaded cells under the search box (9 points of the box: it is smaller than one 4096 u cell, so they cover it).
    // (1.5.0: the box is the widest any of its searches can open - a second pass, a local repair round a corridor leg - and
    // the standing actors are scanned in the same box. 9 points still cover it: they lie at most 4096 u apart.)
    void XPhaseWaters(Job& j, RE::Actor* a_w, bool a_log)
    {
        ExState&    x = *j.x;
        auto*       pcell = a_w->GetParentCell();
        x.interior = pcell && pcell->IsInteriorCell();
        const float d = Dist2D(j.start, x.asked);
        const float pad = x.v15 ? kVMarginMax + kVMarginMin + kXWaterBoxPad : std::clamp(d * kXMarginFrac, kXMarginMin, kXMarginMax) + kXWaterBoxPad;
        const float bx0 = (std::min)(j.start.x, x.asked.x) - pad, bx1 = (std::max)(j.start.x, x.asked.x) + pad;
        const float by0 = (std::min)(j.start.y, x.asked.y) - pad, by1 = (std::max)(j.start.y, x.asked.y) + pad;
        std::vector<RE::TESObjectREFR*> found;
        int                             looked = 0, cellsLooked = 0;
        bool                            capped = false;
        // (review G) an interior is scanned whole: a scan stopped before its placed water would trust the cell plane (the
        // Breezehome trap); only the exterior scan has the reference cap
        const auto visit = [&](RE::TESObjectREFR* a_ref) {
            if (++looked > kXMaxRefsLooked && !x.interior) {
                capped = true;
                return RE::BSContainer::ForEachResult::kStop;
            }
            if (IsWaterActivator(a_ref)) found.push_back(a_ref);
            return RE::BSContainer::ForEachResult::kContinue;
        };
        if (x.interior) {
            pcell->ForEachReference(visit);
            ++cellsLooked;
            x.placedInCell = !found.empty();
        } else if (auto* tes = RE::TES::GetSingleton()) {
            std::vector<RE::TESObjectCELL*> seen;
            const float                     xs[3] = { bx0, (bx0 + bx1) * 0.5f, bx1 }, ys[3] = { by0, (by0 + by1) * 0.5f, by1 };
            for (const float px : xs) {
                for (const float py : ys) {
                    if (capped) break;
                    auto* c = tes->GetCell(RE::NiPoint3{ px, py, j.start.z });
                    if (!c || c->IsInteriorCell() || !c->IsAttached() || std::find(seen.begin(), seen.end(), c) != seen.end()) continue;
                    seen.push_back(c);
                    c->ForEachReference(visit);
                    ++cellsLooked;
                }
            }
        }
        // their footprints (outside the cells' locks): those over the box, the nearest to her line first
        std::vector<std::pair<float, WaterFP>> byDist;
        const RE::NiPoint3                     a = j.start, b = x.asked;
        const float                            abx = b.x - a.x, aby = b.y - a.y, ab2 = abx * abx + aby * aby;
        for (auto* ref : found) {
            WaterFP fp;
            if (!Footprint(ref, fp)) continue;
            const float rr = std::hypot(fp.hx, fp.hy);
            if (fp.cx + rr < bx0 || fp.cx - rr > bx1 || fp.cy + rr < by0 || fp.cy - rr > by1) continue;
            const float t = ab2 > 1e-4f ? std::clamp(((fp.cx - a.x) * abx + (fp.cy - a.y) * aby) / ab2, 0.f, 1.f) : 0.f;
            byDist.emplace_back(std::hypot(fp.cx - (a.x + abx * t), fp.cy - (a.y + aby * t)), fp);
        }
        std::sort(byDist.begin(), byDist.end(), [](const auto& l, const auto& r) { return l.first < r.first; });
        x.watersComplete = !capped && byDist.size() <= kXMaxWaters;     // (1.5.1 review #6) the cache keeps depths only then
        if (byDist.size() > kXMaxWaters) byDist.resize(kXMaxWaters);
        x.waters.clear();
        for (const auto& e : byDist) x.waters.push_back(e.second);
        if (a_log) {
            logger::info("PATH #{} water: {} placed water(s) seen, {} kept, {} reference(s) looked at in {} cell(s){} - {}", j.handle, found.size(),
                         x.waters.size(), looked, cellsLooked, capped ? " (STOPPED at the reference cap)" : "",
                         !x.interior    ? "exterior: placed water first, then the water height per point" :
                         x.placedInCell ? "interior WITH placed water: its cell water plane is NOT trusted" :
                                          "interior without placed water: the cell plane is trusted from floor - 10 to floor + 200");
            if (j.xflags & kVerbose) {
                for (const auto& e : byDist) {
                    logger::info("PATH #{} water: footprint centre ({:.0f},{:.0f}), surface z {:.0f}, {:.0f} x {:.0f} u, {:.0f} u from her line",
                                 j.handle, e.second.cx, e.second.cy, e.second.surf, e.second.hx * 2.f, e.second.hy * 2.f, e.first);
                }
            }
        }
        if (x.v15 && (j.xflags & kActorsKeepOut) && a_log) ScanActors(j, a_w, bx0, bx1, by0, by1);
        x.xphase = kXStart;
    }

    // 0a: where she stands - the navmesh, else the floor under her feet (else her feet: she IS standing there); the
    // water there (deeper than the cap = refused); and the engine's own water reading next to ours (B §4.2)
    void XPhaseStart(Job& j, RE::Actor* a_w, RE::bhkWorld* a_bw)
    {
        ExState&           x = *j.x;
        const RE::NiPoint3 p = j.start;
        PCell&             s = x.start;
        s = PCell{};
        s.pos = p;
        // (review G: every z is a physics floor) the floor under her feet; the navmesh only says whether she is on it
        const char*  how = "her feet (no floor found under her)";
        RE::NiPoint3 got;
        const bool   nav = SnapNav(j, a_w, p.x, p.y, p.z, kLevelBand, got) && Dist2D(got, p) <= kCell * 0.6f && std::fabs(got.z - p.z) <= kMaxStep;
        const float    sTop = p.z + x.step + kXTopPad, sBot = p.z - (std::max)(kXFloorDown, kXFloorDownH * x.H);
        const FloorHit h = x.v15 ? XFloorRay(j, a_bw, p.x, p.y, sTop, sBot, j.xs.rF) : FloorRay(j, a_bw, p.x, p.y, sTop, sBot, j.xs.rF);
        if (h.found && std::fabs(h.z - p.z) <= x.step) {
            s.pos.z = h.z;
            how = "the physics floor";
        }
        s.onNav = nav && std::fabs(got.z - s.pos.z) <= x.step;
        s.navTerr = x.v15 && s.onNav && h.found && s.pos.z == h.z && IsTerrainLayer(h.layer);    // 1.5.1 (XFloorRay's layer)
        s.kind = s.onNav ? kKNav : kKFloor;
        float depth = 0.f, surf = 0.f;
        const bool  known = DepthAt(j, a_w, p.x, p.y, s.pos.z, depth, surf);
        const float ew = a_w->GetWaterHeight();
        const bool  engWet = IsFiniteHeight(ew) && ew > a_w->GetPositionZ();
        if (engWet || depth > kXWet) {
            logger::info("PATH #{} WATER CHECK: engine {} (her feet z {:.0f}), ours {} (her floor z {:.0f}, depth {:.0f} u)", j.handle,
                         IsFiniteHeight(ew) ? std::format("{:.1f}", ew) : std::string("none"), a_w->GetPositionZ(),
                         IsFiniteHeight(surf) ? std::format("{:.1f}", surf) : std::string("none"), s.pos.z, depth);
        }
        std::string deepNote;
        if (known && depth > kXWet) {
            if (depth > x.depthCap) {
                // 1.4.1 THE DEEP START: she stands deeper than her cap (pulled into a bath, waded in on her own) - she is
                // already there, so it is no reason to refuse. Her way out (her START REGION) may use water up to her own
                // depth + 8 u, strictly under the swim level. At the swim level already: refused (the walk answers O 21).
                const float rootDepth = IsFiniteHeight(surf) ? surf - p.z : depth;    // the engine measures from her root
                const float here = (std::max)(depth, rootDepth);
                if (here >= x.swimDepth) {
                    j.swimStart = true;
                    ExFinish(j, 0, std::format("she is at the swim level: {:.0f} u of water over her feet, the engine swims her from {:.0f} u "
                                               "(0.70 x {:.1f} u) - a route never asks for swimming", here, x.swimDepth, x.swimDiv));
                    return;
                }
                x.deepStart = true;
                // (review R #5) measured as the engine does - from her ROOT: a cell of floor depth D puts her root at about
                // D + rootUnder; the limit keeps that kXSwimMargin under the swim depth, and never falls under her own depth
                const float rootUnder = (std::max)(0.f, rootDepth - depth);
                const float bySwim = x.swimDepth - kXSwimMargin - rootUnder;
                const float own = (std::max)(depth, (std::min)(depth + kXOwnDepthTol, x.swimDepth - rootUnder - kXOwnDepthTol));
                x.startLimit = (std::max)((std::min)((std::max)(x.depthCap, depth + kXDeepStartPad), bySwim), own);
                x.startWater = PlacedWaterAt(x, p.x, p.y, s.pos.z);     // (review R #6) her own water: only it gets the limit
                x.startSurf = surf;
                deepNote = std::format(" (deeper than the cap {:.0f} u: allowed, she is already there) - her way out may use water up to {:.0f} u "
                                       "({:.0f} u over her root, {:.1f} u under the floor hit; the swim level is {:.0f} u, kept {:.0f} u under "
                                       "where room allows; {} water)",
                                       x.depthCap, x.startLimit, rootDepth, rootUnder, x.swimDepth, kXSwimMargin,
                                       x.startWater >= 0 ? "placed" : "cell / exterior");
            }
            s.kind = kKWater;
            s.depth = depth;
            s.deep = x.deepStart;
        }
        logger::info("PATH #{} start: ({:.0f},{:.0f},{:.0f}) on {}{}, {}{}", j.handle, s.pos.x, s.pos.y, s.pos.z, how, s.onNav ? " (on the navmesh)" : "",
                     s.kind == kKWater ? std::format("in water {:.0f} u deep", s.depth) : std::string("dry"), deepNote);
        x.xphase = kXGoal;
    }

    // The goal (or a GOAL_MAY_MOVE candidate): the navmesh or the physics floor, the water there, the stand-spot test.
    // a_out.pos.z = the floor found, also when the test fails (the march carries that level); *a_surface = the water
    // surface seen over that floor (-1e9 = none).
    bool XGoalSpot(Job& j, RE::Actor* a_w, RE::bhkWorld* a_bw, const RE::NiPoint3& a_p, PCell& a_out, bool& a_hasFloor, std::string& a_why,
                   float* a_surface = nullptr)
    {
        ExState& x = *j.x;
        a_out = PCell{};
        a_out.pos = a_p;
        a_out.goal = true;
        a_hasFloor = false;
        if (a_surface) *a_surface = -1.0e9f;
        RE::NiPoint3 got;
        const bool   nav = SnapNav(j, a_w, a_p.x, a_p.y, a_p.z, kLevelBand, got) && std::hypot(got.x - a_p.x, got.y - a_p.y) <= kCell * 0.6f;
        if (!nav && !(j.xflags & kFloor)) {
            a_why = "no navmesh there (a floor route was not asked)";
            return false;
        }
        if (InKeepOut(j, a_p.x, a_p.y)) {
            a_why = "inside the keep-out circle";
            return false;
        }
        // the floor is searched from the ASKED point's level (never from a navmesh found above it)
        // (1.4.1) with a deep start, a goal may lie in her start region: its depth limit is the region's (the search reaches
        // such a goal only from inside the region - XPhaseSearch's goal test). (Review R #6) Only in HER OWN water - the same
        // placed water, or (none placed) the same surface: a goal in other deep water keeps the cap and GOAL_MAY_MOVE moves it
        // toward her as in 1.4.0. One question (the water over the asked point).
        float lim = 0.f;
        if (x.deepStart) {
            float      d0 = 0.f, s0 = -1.0e9f;
            const bool known = DepthAt(j, a_w, a_p.x, a_p.y, a_p.z, d0, s0);
            const int  gw = PlacedWaterAt(x, a_p.x, a_p.y, a_p.z);
            const bool same = x.startWater >= 0 ? gw == x.startWater
                                                : (known && gw < 0 && IsFiniteHeight(s0) && std::fabs(s0 - x.startSurf) <= kXSameSurface);
            if (same) lim = x.startLimit;
        }
        float         floorZ = a_p.z, depth = 0.f, surf = -1.0e9f;
        std::uint32_t layer = 0;              // (1.5.1) the floor's layer (the terrain allowance)
        bool          ok = StandSpot(j, a_w, a_bw, a_p.x, a_p.y, a_p.z + x.step + kXGoalTopPad, a_p.z - kXFloorDown, floorZ, depth, a_hasFloor, a_why,
                                     &surf, lim, &layer);
        bool          steep = x.v15 && x.standSteep;     // (fix round, R2 #3) THIS test's refusal was the slope only
        // (review G, RISK 11) a floor more than a step ABOVE the asked point (a jetty / plank over a water target): look
        // under it once, from just over the asked level; a lower floor found there wins
        if (a_hasFloor && floorZ > a_p.z + x.step) {
            float         fz2 = 0.f, dep2 = 0.f, surf2 = -1.0e9f;
            bool          hf2 = false;
            std::string   why2;
            std::uint32_t layer2 = 0;
            const bool    ok2 = StandSpot(j, a_w, a_bw, a_p.x, a_p.y, a_p.z + x.step + kXTopPad, a_p.z - kXFloorDown, fz2, dep2, hf2, why2, &surf2,
                                          lim, &layer2);
            const bool    steep2 = x.v15 && x.standSteep;
            if (hf2 && fz2 < floorZ - x.step) {
                ok = ok2;
                floorZ = fz2;
                depth = dep2;
                surf = surf2;
                a_why = why2;
                layer = layer2;
                steep = steep2;
            }
        }
        x.spotSteep = !ok && steep && a_hasFloor;      // the refusal used is the slope only ...
        x.spotLayer = layer;                           // ... of a floor of this layer (terrain = 13 / 17)
        if (a_hasFloor) a_out.pos.z = floorZ;
        a_out.onNav = nav && a_hasFloor && std::fabs(got.z - floorZ) <= x.step;     // the navmesh lies on THIS floor
        a_out.navTerr = x.v15 && a_out.onNav && IsTerrainLayer(layer);             // 1.5.1
        if (ok && !a_out.onNav && !(j.xflags & kFloor)) {
            ok = false;
            a_why = "the navmesh there is not on its floor (a floor route was not asked)";
        }
        a_out.depth = depth;
        a_out.kind = depth > 0.f ? kKWater : a_out.onNav ? kKNav : kKFloor;
        a_out.deep = lim > 0.f && depth > x.depthCap;       // 1.4.1: a goal in her start region's water (her own water only)
        if (a_surface) *a_surface = surf;
        return ok;
    }

    // how far the goal moved: flat distance, or the 3D one when its floor is more than a step from the asked z (review G)
    float GoalMove(const RE::NiPoint3& a_asked, const RE::NiPoint3& a_goal, float a_step)
    {
        const float flat = Dist2D(a_asked, a_goal), dz = std::fabs(a_goal.z - a_asked.z);
        return dz > a_step ? std::hypot(flat, dz) : flat;
    }

    // 0c: the goal; not a place to stand -> GOAL_MAY_MOVE's march, else no route with the reason
    void XPhaseGoal(Job& j, RE::Actor* a_w, RE::bhkWorld* a_bw)
    {
        ExState& x = *j.x;
        // (1.6.0 §2.6, D §1 item 6) a goal outside the loaded cells is answered at once: the walk answers O 26 (outsideLoaded)
        if (x.v15) {
            std::string why;
            if (!standpoint_nav::GoalLoaded(a_w, x.asked, &why)) {
                j.outsideLoaded = true;
                logger::info("PATH #{} goal ({:.0f},{:.0f},{:.0f}): {}", j.handle, x.asked.x, x.asked.y, x.asked.z, why);
                ExFinish(j, 0, why);
                return;
            }
        }
        if (Dist2D(j.start, x.asked) > (x.v15 ? kVMaxDistance : kMaxDistance)) {     // (1.5.1: 2400 u for a 1.5.0-flag plan)
            ExFinish(j, 0, "too far for a local route (the engine's own pathing walks this)");
            return;
        }
        // (1.6.0) CHUNK MODE: a chunk's end that is not the goal is a pass-through point - no stand-spot test (its floor, slope, water,
        // headroom and clearance are a corridor point's own tests: XPhaseNavCorr stage 2, which may move it aside or drop it)
        if (x.chunk && !x.chunkFinal) {
            PCell g;
            g.pos = x.asked;
            g.kind = kKNav;
            g.onNav = true;
            g.chunkEnd = true;
            x.goal = g;
            j.goalMovedBy = 0.f;
            logger::info("PATH #{} chunk: ends at ({:.0f},{:.0f},{:.0f}), skeleton point {} of {} - a pass-through point; {:.0f} u of skeleton after it",
                         j.handle, g.pos.x, g.pos.y, g.pos.z, j.skelTaken, x.skelTotal, j.remainAfterU);
            x.xphase = kXGate;
            return;
        }
        if (x.wade) {
            // 1.5.0 item 8 (kWadeSpot): the seed (Water Finder's target) is no place to stand on - it only aims the ONE search
            // from her and sizes its box; the search ends at the first reachable wading spot. No stand-spot test here and no
            // march toward her (a march up a cliff was the letter-18 failure).
            PCell sg;
            sg.pos = x.asked;
            sg.kind = kKWater;
            x.goal = sg;
            float      dep = 0.f, srf = -1.0e9f;
            const bool known = DepthAt(j, a_w, x.asked.x, x.asked.y, x.asked.z, dep, srf);
            x.seedSurf = (known && IsFiniteHeight(srf) && srf > x.asked.z) ? srf : -1.0e9f;     // (fix round, R2 #10) the flood's trigger
            logger::info("PATH #{} wade: seed ({:.0f},{:.0f},{:.0f}), water there {} - ONE search from her toward it ends at the first reachable "
                         "water cell {:.0f}-{:.0f} u deep with >= {} of its 8 neighbours wading water ({:.0f} u or more) and a place to stand",
                         j.handle, x.asked.x, x.asked.y, x.asked.z,
                         !known ? std::string("unknown (no loaded cell)") :
                         IsFiniteHeight(srf) && srf > x.asked.z ? std::format("{:.0f} u over the seed's z (surface {:.0f})", dep, srf) :
                                                                  std::string("none over the seed's z"),
                         x.wadeMin, x.depthCap, x.wadePatchMin, x.wadeNbrMin);
            x.xphase = kXGate;
            return;
        }
        PCell       g;
        bool        hasFloor = false;
        std::string why;
        float       surf = -1.0e9f;
        if (XGoalSpot(j, a_w, a_bw, x.asked, g, hasFloor, why, &surf)) {
            x.goal = g;
            j.goalMovedBy = GoalMove(x.asked, g.pos, x.step);      // its floor more than a step from the asked z counts as moved
            logger::info("PATH #{} goal: ({:.0f},{:.0f},{:.0f}) {}{}, a place to stand{}", j.handle, g.pos.x, g.pos.y, g.pos.z,
                         g.kind == kKWater ? std::format("in water {:.0f} u deep", g.depth) : std::string("dry"), g.onNav ? ", on the navmesh" : "",
                         j.goalMovedBy > 0.5f ? std::format(" (its floor is {:.0f} u from the asked z)", g.pos.z - x.asked.z) : std::string());
            x.xphase = kXGate;
            return;
        }
        // a goal IN water: the march must stay within a step over that water's surface (never up onto a dock / a bank);
        // a dry goal has no such band
        const float floorRef = hasFloor ? g.pos.z : x.asked.z;
        if (!IsFiniteHeight(surf)) {
            float depth = 0.f;
            if (!DepthAt(j, a_w, x.asked.x, x.asked.y, floorRef, depth, surf)) surf = -1.0e9f;
        }
        x.marchSurf = (IsFiniteHeight(surf) && surf > floorRef + kXWet) ? surf : -1.0e9f;
        x.goalWhy = why;
        logger::info("PATH #{} goal ({:.0f},{:.0f},{:.0f}) is not a place to stand: {}{}", j.handle, x.asked.x, x.asked.y, x.asked.z, why,
                     (j.xflags & kGoalMayMove) ? " - moving it toward her" : "");
        if (!(j.xflags & kGoalMayMove)) {
            ExFinish(j, 0, "the goal is not a place to stand: " + why);
            return;
        }
        x.marchK = 0;
        x.marchPrev = RE::NiPoint3{ x.asked.x, x.asked.y, hasFloor ? g.pos.z : x.asked.z };
        x.marchRefZ = x.marchPrev.z;
        x.marchBody = hasFloor;
        // (fix round, R2 #3) the ring's reference height: the goal's own floor - but a floor refused ONLY for its slope that is no
        // terrain (a stair / a ramp of a static floor: run #10's "too steep", n.z 0.70 on Lakeview's stair) is no floor to stand
        // on at any height: the ring then looks at the ASKED height (stpf asks at his own floor), so the floor he stands on beside
        // the stair - above it or below it - is the candidates' floor. Stand spots stay 0.85: never a spot on the stair itself.
        x.ringAsked = x.v15 && hasFloor && x.spotSteep && !IsTerrainLayer(x.spotLayer);
        x.ringRefZ = x.ringAsked ? x.asked.z : x.marchRefZ;
        if (x.ringAsked) {
            logger::info("PATH #{} goal ring: the goal's floor ({:.0f}, layer {}) was refused for its slope only - a stair / a ramp, no terrain: "
                         "the ring looks for a spot at the ASKED height ({:.0f}, {:.0f} u from that floor)",
                         j.handle, g.pos.z, x.spotLayer, x.asked.z, x.asked.z - g.pos.z);
        }
        x.xphase = x.v15 ? kXRing : kXMarch;      // (1.6.0 §5.3 #3) the ring search on the goal's own floor; 1.4.1: the march
    }

    // 1.6.0 §5.3 #3 THE GOAL RING (a 1.5.0-flag plan with kGoalMayMove; the asked goal is no place to stand - under a table, on a
    // stair, by a brink): candidates on rings 16-160 u round the asked point (16 u apart), 8 bearings each - the bearing toward her
    // first, then alternating round - nearest ring first; each gets the goal's own tests (XGoalSpot: the stand spot n.z >= 0.85 /
    // 0.75 on navmesh terrain, headroom, no brink) on the GOAL'S FLOOR (|dz| <= riser + 8 of the floor under the asked point, else
    // its z); a goal in water stays at the water's level. The refused spot is never the start of a body line (the 1.5.x march
    // tested her body FROM it and died under a table or on a ramp). One candidate per unit (~12 questions).
    // (fix round, R2 #3) a goal floor refused only for the slope of a stair / static ramp: the reference is the ASKED height
    // (x.ringRefZ - XPhaseGoal). (R2 #4) on navmesh terrain a candidate may lie riser + 8 + 0.5 x r off (a 20 deg hillside rises
    // 58 u over 160 u); none on the ring -> the 1.5.1 march toward her (following the floor <= 320 u), never a new O 1.
    // (1.6.1 A4 - R3 Q3: 0.5 s spent on a ring spot 8 u from a table's edge between two chairs) her body must FIT at a moved goal: the
    // search's own cell clearance there (the floor at rc N / E / S / W within a riser - no brink, no table top or chair seat under her
    // shoulder - and knee probes out to rc) and the 4 side columns at rc (a table top, a counter, a chair back beside her). ~12
    // questions. A spot it refuses is one the search could reach only by squeezing in - the ring takes the next one.
    bool XSpotFits(Job& j, RE::bhkWorld* a_bw, const PCell& a_g)
    {
        ExState& x = *j.x;
        if (!XCellClear(j, a_bw, a_g.pos.x, a_g.pos.y, a_g.pos.z, a_g.navTerr, false)) {
            x.fitWhy = "a brink, a rise (a table top, a seat) or a wall within her clearance";
            return false;
        }
        const int  side0 = x.besideSide;
        const bool cols = XSideColumns(j, a_bw, a_g.pos.x, a_g.pos.y, a_g.pos.z, 1.f, 0.f) ||     // columns at +-rc north / south
                          XSideColumns(j, a_bw, a_g.pos.x, a_g.pos.y, a_g.pos.z, 0.f, 1.f);       // ... east / west
        x.besideSide = side0;
        if (cols) {
            x.fitWhy = "a table top, a counter or furniture beside her body (the side columns)";
            return false;
        }
        return true;
    }

    void XPhaseRing(Job& j, RE::Actor* a_w, RE::bhkWorld* a_bw)
    {
        ExState&            x = *j.x;
        constexpr int       kPerRing = 8;
        constexpr int       kOrder[kPerRing] = { 0, 1, -1, 2, -2, 3, -3, 4 };
        const int           rings = static_cast<int>((kVRingMax - kVRingMin) / kVRingStep + 0.5f) + 1;
        const float         toHer = std::atan2(j.start.y - x.asked.y, j.start.x - x.asked.x);
        const char*         refName = x.ringAsked ? "the asked height" : "the goal's floor";
        while (x.xphase == kXRing) {
            if (Spent(j)) return;
            if (x.marchK >= rings * kPerRing) {
                // (fix round, R2 #4) nothing on the ring: the 1.5.1 march toward her from the refused spot (no regression); its
                // answers carry the ring's count
                x.goalWhy = std::format("{}; no spot within {:.0f} u of {} passed the ring ({} tried: {} not a place to stand, {} on another "
                                        "floor, {} too tight for her; last: {})",
                                        x.goalWhy, kVRingMax, refName, j.xs.ringTried, j.xs.ringRefused, j.xs.ringOtherFloor, j.xs.ringTight,
                                        x.ringLastWhy.empty() ? std::string("-") : x.ringLastWhy);
                logger::info("PATH #{} goal ring: no spot within {:.0f} u of {} ({:.0f}) passed ({} tried) - the 1.5.1 march toward her follows "
                             "the floor instead (<= {:.0f} u)",
                             j.handle, kVRingMax, refName, x.ringRefZ, j.xs.ringTried, kXMarchMax);
                ++j.xs.ringMarch;
                x.marchK = 0;                     // the march's own count (marchPrev / marchRefZ / marchBody: XPhaseGoal's, untouched)
                x.xphase = kXMarch;
                return;
            }
            const int   k = x.marchK++;
            const float r = kVRingMin + static_cast<float>(k / kPerRing) * kVRingStep;
            const float ang = toHer + static_cast<float>(kOrder[k % kPerRing]) * (kXPi * 0.25f);
            const float cx = x.asked.x + r * std::cos(ang), cy = x.asked.y + r * std::sin(ang);
            if (InKeepOut(j, cx, cy)) continue;
            ++j.xs.ringTried;
            PCell       g;
            bool        hasFloor = false;
            std::string why;
            if (!XGoalSpot(j, a_w, a_bw, RE::NiPoint3{ cx, cy, x.ringRefZ }, g, hasFloor, why)) {
                ++j.xs.ringRefused;
                x.ringLastWhy = why;
                continue;
            }
            const float tol = x.riser + kVRingLevel + (g.navTerr ? kVRingSlope * r : 0.f);     // (R2 #4) the hillside allowance
            if (std::fabs(g.pos.z - x.ringRefZ) > tol || (IsFiniteHeight(x.marchSurf) && g.pos.z > x.marchSurf + x.step)) {
                ++j.xs.ringOtherFloor;
                x.ringLastWhy = std::format("({:.0f},{:.0f}) stands {:.0f} u off {} (at most {:.0f} u)", cx, cy, g.pos.z - x.ringRefZ, refName, tol);
                continue;
            }
            if (!XSpotFits(j, a_bw, g)) {         // (1.6.1 A4) her body must fit there
                ++j.xs.ringTight;
                x.ringLastWhy = std::format("({:.0f},{:.0f}) is too tight for her: {}", cx, cy, x.fitWhy);
                continue;
            }
            x.goal = g;
            j.goalMovedBy = (std::max)(Dist2D(x.asked, g.pos), GoalMove(x.asked, g.pos, x.step));
            logger::info("PATH #{} goal moved {:.0f} u by the ring search ({:.0f} u ring, candidate {} of {}): ({:.0f},{:.0f},{:.0f}) {}", j.handle,
                         j.goalMovedBy, r, k + 1, rings * kPerRing, g.pos.x, g.pos.y, g.pos.z,
                         g.kind == kKWater ? std::format("in water {:.0f} u deep", g.depth) : std::string("dry"));
            x.xphase = kXGate;
            return;
        }
    }

    // B §4.4 GOAL_MAY_MOVE: from the goal toward her in 16 u steps up to 320 u; the first standable point is the goal.
    // The way between the steps must be free for her body (a step must never pass into a rock or a wall). (Review G,
    // RISK 11) It follows the floor by at most a step per 16 u (a dock / plank above the water is never stepped onto: its
    // down ray starts under it) and, for a goal in water, never rises more than a step over the water's surface.
    void XPhaseMarch(Job& j, RE::Actor* a_w, RE::bhkWorld* a_bw)
    {
        ExState&    x = *j.x;
        const float total = Dist2D(x.asked, j.start);
        const float ux = total > 1.f ? (j.start.x - x.asked.x) / total : 0.f, uy = total > 1.f ? (j.start.y - x.asked.y) / total : 0.f;
        while (x.xphase == kXMarch) {
            if (Spent(j)) return;
            const float d = static_cast<float>(++x.marchK) * kXMarchStep;
            if (d > kXMarchMax || d >= total - 1.f) {
                ExFinish(j, 0, std::format("the goal is not a place to stand: {}; nothing standable within {:.0f} u toward her", x.goalWhy,
                                           (std::min)(kXMarchMax, total)));
                return;
            }
            const float    cx = x.asked.x + ux * d, cy = x.asked.y + uy * d;
            const FloorHit h = x.v15 ? XFloorRay(j, a_bw, cx, cy, x.marchRefZ + x.step + kXTopPad, x.marchRefZ - kXFloorDown, j.xs.rF) :
                                       FloorRay(j, a_bw, cx, cy, x.marchRefZ + x.step + kXTopPad, x.marchRefZ - kXFloorDown, j.xs.rF);
            if (h.blocked) {
                ExFinish(j, 0, std::format("the goal is not a place to stand: {}; a tree or an invisible wall {:.0f} u toward her", x.goalWhy, d));
                return;
            }
            if (!h.found || std::fabs(h.z - x.marchRefZ) > x.step) continue;    // a hole, deep water, a drop: march on at the same level
            if (IsFiniteHeight(x.marchSurf) && h.z > x.marchSurf + x.step) {
                ExFinish(j, 0, std::format("the goal is not a place to stand: {}; nothing standable at the water's level within {:.0f} u toward her",
                                           x.goalWhy, d));
                return;
            }
            const RE::NiPoint3 c{ cx, cy, h.z };
            if (x.marchBody) {
                const float side = j.radius * kSideFrac;
                const int   r0 = j.rays;
                // (1.5.0: her 1.5.0 body - beside her too: a march step must fit her whole)
                const bool  clear = x.v15 ? XBody(j, a_bw, x.marchPrev, c, (x.marchPrev.z + c.z) * 0.5f, false) == 0 :
                                            PieceClear(j, a_bw, x.marchPrev, c, -uy * side, ux * side);
                if (!x.v15) j.xs.rO += j.rays - r0;
                if (!clear) {
                    ExFinish(j, 0, std::format("the goal is not a place to stand: {}; something stands between it and her {:.0f} u toward her",
                                               x.goalWhy, d));
                    return;
                }
            }
            x.marchPrev = c;
            x.marchRefZ = h.z;
            x.marchBody = true;
            PCell       g;
            bool        hasFloor = false;
            std::string why;
            if (XGoalSpot(j, a_w, a_bw, c, g, hasFloor, why) && !(IsFiniteHeight(x.marchSurf) && g.pos.z > x.marchSurf + x.step)) {
                x.goal = g;
                j.goalMovedBy = (std::max)(Dist2D(x.asked, g.pos), GoalMove(x.asked, g.pos, x.step));
                logger::info("PATH #{} goal moved {:.0f} u toward her: ({:.0f},{:.0f},{:.0f}) {}", j.handle, j.goalMovedBy, g.pos.x, g.pos.y, g.pos.z,
                             g.kind == kKWater ? std::format("in water {:.0f} u deep", g.depth) : std::string("dry"));
                x.xphase = kXGate;
                return;
            }
        }
    }

    // 0d B §7.8 G1 (navmesh first): start AND goal on the dry navmesh, no live bad spots -> the 1.3.0 search first.
    // (G2 "not worth it" and G3 "asked again within 15 s" are NOT built: unasked limits / cooldowns - the user's rule.)
    // (Review G, BUG 1) NOT under kAllDirect: the 1.3.0 search's legs were tested for the ENGINE (navmesh at piece ends
    // <= 64 u apart, 78 u of height a piece, no physics floor, water at the line's height) - a straight walk over them can
    // leave a dock or a terrace. The hybrid search already prefers navmesh cells (cost 1.0) and tests every leg for the
    // driver, so skipping G1 loses nothing and also removes 1.3.0's untimed slices from the 1.4.0 walk.
    void XPhaseGate(Job& j)
    {
        ExState& x = *j.x;
        if ((j.xflags & kNavFirst) && !(j.xflags & kAllDirect) && x.start.kind == kKNav && x.goal.kind == kKNav && x.bad.empty()) {
            x.g1 = true;
            j.goal = x.goal.pos;        // the 1.3.0 search's own goal (its phase 0 snaps it again)
            j.phase = 0;
            j.out.clear();
            x.xphase = kXLegacy;
            return;
        }
        x.xphase = kXStraight;
    }

    // the 1.3.0 search, untouched, on this plan; a DIRECT / ROUTE answer is kept when every 32 u of it is dry (that
    // search does not know water), else - and on NO ROUTE - the hybrid search runs with its own budget
    void XPhaseLegacy(Job& j, RE::Actor* a_w, RE::bhkWorld* a_bw)
    {
        ExState& x = *j.x;
        RunSlice(j, a_w, a_bw, x.qEnd);
        if (j.status == kRunning) return;
        const int         st = j.status;
        const std::string why = j.why;
        if (st == -1) {
            ExFinish(j, -1, why);
            return;
        }
        if (st == 1 || st == 2) {
            std::vector<RE::NiPoint3> pts;
            if (st == 2) {
                pts = j.out;
            } else {
                pts.push_back(j.goal);
            }
            RE::NiPoint3 prev = j.start, wetAt{};
            float        wetDepth = 0.f;
            bool         dry = true;
            for (const auto& p : pts) {
                const int n = (std::max)(1, static_cast<int>(std::ceil(Dist2D(prev, p) / kCell)));
                for (int k = 1; k <= n && dry; ++k) {
                    const float        t = static_cast<float>(k) / static_cast<float>(n);
                    const RE::NiPoint3 q{ prev.x + (p.x - prev.x) * t, prev.y + (p.y - prev.y) * t, prev.z + (p.z - prev.z) * t };
                    float              depth = 0.f, surf = 0.f;
                    if (!DepthAt(j, a_w, q.x, q.y, q.z, depth, surf) || depth > kXWet) {
                        dry = false;
                        wetAt = q;
                        wetDepth = depth;
                    }
                }
                if (!dry) break;
                prev = p;
            }
            if (dry) {
                NavPoints(j.start, pts, j.goalMovedBy > 0.5f, (j.xflags & kAllDirect) != 0u, j.route, j.switchIndex, j.routeLen);
                j.xs.g1 = std::format("{} - kept (dry, all on the navmesh)", st == 1 ? "DIRECT" : "ROUTE");
                ExFinish(j, st, "navmesh first: " + why);
                return;
            }
            j.xs.g1 = std::format("{} [{}] but wet or unknown at ({:.0f},{:.0f}) ({:.0f} u) - the hybrid search instead", st == 1 ? "DIRECT" : "ROUTE",
                                  why, wetAt.x, wetAt.y, wetDepth);
        } else {
            j.xs.g1 = std::format("NO ROUTE [{}] - the hybrid search instead", why);
        }
        logger::info("PATH #{} navmesh first: {}", j.handle, j.xs.g1);
        j.status = kRunning;
        j.why.clear();
        j.out.clear();
        x.qBase = j.queries;
        x.xphase = kXStraight;
    }

    // 1.5.0 item 4 / C10: a box's search budget - ~10 questions per 32 u square of the box a -> b with the pass-1 margin
    // max(448, 0.75 d) <= 1024, never under 8,000 nor over 20,000
    int VBoxBudget(const RE::NiPoint3& a_a, const RE::NiPoint3& a_b)
    {
        const float m = std::clamp(Dist2D(a_a, a_b) * kXMarginFrac, kVMarginMin, kVMarginMax);
        const float w = std::fabs(a_b.x - a_a.x) + 2.f * m, h = std::fabs(a_b.y - a_a.y) + 2.f * m;
        const int   cols = (static_cast<int>(w / kCell) + 2) * (static_cast<int>(h / kCell) + 2);
        return std::clamp(cols * kVQPerCell, kVSearchQMin, kVSearchQMax);
    }

    // 1.6.0 §2 THE SKELETON AS THE CORRIDOR: its points (the caller's - a chunk's part of them - or NavGraph's, built here) become the
    // MIX's corridor points, her start not included; the last is x.goal (the goal as XPhaseGoal / the ring tested it, or a chunk's
    // pass-through end). A point within 16 u of the one before it is no leg of its own (a virtual link's flag moves on). Their floors
    // are measured next (XPhaseNavCorr stage 2: a point with no floor is moved aside or dropped), then every leg is tested with the
    // full rules (XPhaseNavLegs) - the navmesh suggests, the rays decide. a_source: PlanInfo::skelSource (1 given, 2 built here).
    void XSkelToCorridor(Job& j, const std::vector<standpoint_nav::SkelPoint>& a_pts, int a_source)
    {
        ExState& x = *j.x;
        x.realStart = x.start;
        x.realGoal = x.goal;
        x.ngPts.clear();
        RE::NiPoint3 prev = x.start.pos;
        bool         virtOwed = false;
        float        len = 0.f;
        RE::NiPoint3 lp = x.start.pos;
        int          virt = 0;
        for (std::size_t i = 0; i + 1 < a_pts.size(); ++i) {      // the last skeleton point = the goal / the chunk's end: x.goal
            const auto& sp = a_pts[i];
            len += (sp.pos - lp).Length();
            lp = sp.pos;
            virtOwed = virtOwed || (sp.flags & standpoint_nav::kSkVirtual) != 0u;
            if ((sp.pos - prev).Length() < kVSkelDupe || (sp.pos - x.goal.pos).Length() < kVSkelDupe) continue;
            PCell c;
            c.pos = sp.pos;
            c.kind = kKNav;
            c.onNav = true;
            c.virt = virtOwed;
            if (c.virt) ++virt;
            virtOwed = false;
            x.ngPts.push_back(c);
            prev = sp.pos;
        }
        if (!a_pts.empty()) len += (a_pts.back().pos - lp).Length();
        PCell g = x.goal;
        g.virt = virtOwed || (!a_pts.empty() && (a_pts.back().flags & standpoint_nav::kSkVirtual) != 0u);
        if (g.virt) ++virt;
        x.ngPts.push_back(g);
        j.skelSource = a_source;
        j.skelLength = len;
        x.corrNote = std::format("the SKELETON ({}: {} corridor point(s) of {} skeleton point(s), {:.0f} u{})", a_source == 1 ? "given" : "built here",
                                 x.ngPts.size(), a_pts.size(), len, virt ? std::format(", {} virtual link(s) to prove", virt) : std::string());
        x.ngBudget = VBoxBudget(x.start.pos, x.goal.pos);
        x.qNav0 = j.queries;
        x.ngSpot = 0;
        x.ngMoved = 0;
        x.ngDropped = 0;
        x.corrCells = static_cast<int>(x.ngPts.size());
        x.corrQ = 0;
        x.ngK = 0;
        x.ngStage = 2;                            // their floors (XPhaseNavCorr stage 2), then the legs
        x.xphase = kXNavCorr;
        x.yield = true;
    }

    // 0e: the straight line with the floor rules -> clear = status 1 (one point: the goal)
    // (1.5.0: the 3D line test of SegRunV - it must end on the goal's own floor - with doors as the plan allows; a wade-spot
    // plan has no straight line (the seed is no goal); a failed line with kNavGuide goes to the MIX's corridor when her start
    // and the goal are both on the navmesh)
    // (1.6.0: the corridor is the SKELETON - the caller's, else NavGraph's built here (kXSkeleton) - whose ends may lie up to 256 u
    // off the navmesh; CHUNK MODE skips the straight line: the chunk's skeleton is the corridor at once)
    void XPhaseStraight(Job& j, RE::Actor* a_w, RE::bhkWorld* a_bw)
    {
        ExState& x = *j.x;
        if (!(j.xflags & kFloor)) {
            j.xs.straight = "not tried (no kFloor: every point must be a navmesh cell)";
            x.xphase = kXGrid;
            x.yield = true;
            return;
        }
        if (x.wade) {
            j.xs.straight = "not tried (a wade-spot search: the seed only aims it)";
            x.xphase = kXGrid;
            x.yield = true;
            return;
        }
        if (x.chunk) {
            j.xs.straight = "not tried (chunk mode: the chunk's skeleton is the corridor)";
            XSkelToCorridor(j, x.skelIn, 1);
            return;
        }
        if (!x.seg.active) SegBegin(j, x.start.pos, x.goal.pos, false, false, 0.f, true, x.start.deep, (j.xflags & kDoors) != 0u);
        SegRun(j, a_w, a_bw);
        if (x.seg.result == 0) return;
        x.seg.active = false;
        if (x.seg.result == 1) {
            j.route.clear();
            std::uint32_t f = kPtGoal;
            if (j.goalMovedBy > 0.5f) f |= kPtGoalMoved;
            if (x.goal.onNav) f |= kPtOnNavmesh;
            if (x.seg.door) f |= kPtDoor;
            j.route.push_back(RoutePoint{ x.goal.pos, static_cast<Kind>(x.goal.kind), x.goal.depth, f, x.seg.door });
            j.switchIndex = 0;
            j.routeLen = Dist2D(x.start.pos, x.goal.pos);
            ExFinish(j, 1, "the straight line is clear");
            return;
        }
        j.xs.straight = std::format("{} at {:.0f} u", x.seg.why, x.seg.failAt);
        if (x.v15 && (j.xflags & kNavGuide)) {
            // (1.6.0) the caller's skeleton is the corridor; else NavGraph's, stepped here (its ends join the navmesh within 256 u)
            if (!x.skelIn.empty()) {
                XSkelToCorridor(j, x.skelIn, 1);
                return;
            }
            x.realStart = x.start;
            x.realGoal = x.goal;
            x.xphase = kXSkeleton;
            x.yield = true;
            return;
        }
        x.xphase = kXGrid;
        x.yield = true;           // the grid gets a slice to itself
    }

    // 1.5.0 the grid (item 4 / C10): margin max(448, 0.75 d) <= 1024 (the second pass 1.5 x as wide, <= 1024); one node per
    // square to start with + a reserved pool for further levels (C9; nothing ever reallocates, so no reference moves); budget
    // ~10 questions per square (8,000-20,000), fresh for each pass; a local repair round a corridor leg draws on the MIX's.
    void XGridV(Job& j, float a_dist)
    {
        ExState& x = *j.x;
        float    margin = std::clamp(a_dist * kXMarginFrac, kVMarginMin, kVMarginMax);
        if (x.pass == 2) margin = (std::min)(kVMarginMax, margin * kVPass2Mul);
        x.margin = margin;
        // (1.6.0 §4 #5) a plan that may need a second pass gets its grid at the pass-2 margin at once; pass 1 searches its own box of
        // it, and a pass 2 CONTINUES it (nothing re-classified, nothing re-tested) instead of rebuilding a wider grid
        const float m2 = (std::min)(kVMarginMax, margin * kVPass2Mul);
        x.canGrow = x.pass == 1 && !x.local && m2 > margin + kVPass2Gain;
        x.margin2 = x.canGrow ? m2 : margin;
        const float alloc = x.margin2;
        // (1.5.1) the lattice lies on WORLD multiples of 32 u (the box's corner rounded down to one): every plan's squares are the
        // same points, so the cell cache's answers fit the next plan in the same place
        x.minX = std::floor(((std::min)(x.start.pos.x, x.goal.pos.x) - alloc) / kCell) * kCell;
        x.minY = std::floor(((std::min)(x.start.pos.y, x.goal.pos.y) - alloc) / kCell) * kCell;
        const float maxX = (std::max)(x.start.pos.x, x.goal.pos.x) + alloc;
        const float maxY = (std::max)(x.start.pos.y, x.goal.pos.y) + alloc;
        x.nx = static_cast<int>((maxX - x.minX) / kCell) + 2;
        x.ny = static_cast<int>((maxY - x.minY) / kCell) + 2;
        // pass 1's box in squares (the whole grid when no pass 2 can follow)
        x.ax0 = 0;
        x.ay0 = 0;
        x.ax1 = x.nx - 1;
        x.ay1 = x.ny - 1;
        if (x.canGrow) {
            const auto sq = [](float a_v, float a_min, int a_n) {
                return std::clamp(static_cast<int>(std::floor((a_v - a_min) / kCell)), 0, a_n - 1);
            };
            x.ax0 = sq((std::min)(x.start.pos.x, x.goal.pos.x) - margin, x.minX, x.nx);
            x.ay0 = sq((std::min)(x.start.pos.y, x.goal.pos.y) - margin, x.minY, x.ny);
            x.ax1 = (std::min)(x.nx - 1, sq((std::max)(x.start.pos.x, x.goal.pos.x) + margin, x.minX, x.nx) + 1);
            x.ay1 = (std::min)(x.ny - 1, sq((std::max)(x.start.pos.y, x.goal.pos.y) + margin, x.minY, x.ny) + 1);
        }
        x.frontier.clear();
        x.frontierK = 0;
        x.reexp = false;
        decltype(x.deferred)().swap(x.deferred);
        x.wadeScan = false;
        const std::size_t cols = static_cast<std::size_t>(x.nx) * static_cast<std::size_t>(x.ny);
        std::vector<XNode>().swap(x.nodes);
        x.nodes.reserve(cols + (std::min)(cols * 3, static_cast<std::size_t>(kVExtraLevels)));
        x.nodes.resize(cols);
        for (int iy = 0; iy < x.ny; ++iy) {
            for (int ix = 0; ix < x.nx; ++ix) {
                const std::size_t i = static_cast<std::size_t>(iy) * static_cast<std::size_t>(x.nx) + static_cast<std::size_t>(ix);
                XNode&            n = x.nodes[i];
                n.x = x.minX + static_cast<float>(ix) * kCell;
                n.y = x.minY + static_cast<float>(iy) * kCell;
                n.col = static_cast<std::int32_t>(i);
            }
        }
        decltype(x.open)().swap(x.open);
        const int          sx = std::clamp(static_cast<int>(std::lround((x.start.pos.x - x.minX) / kCell)), 0, x.nx - 1);
        const int          sy = std::clamp(static_cast<int>(std::lround((x.start.pos.y - x.minY) / kCell)), 0, x.ny - 1);
        const std::int32_t s = sy * x.nx + sx;
        XNode&             sn = x.nodes[static_cast<std::size_t>(s)];
        sn.x = x.start.pos.x;
        sn.y = x.start.pos.y;
        sn.z = x.start.pos.z;
        sn.zRef = sn.z;
        sn.kind = x.start.kind;
        sn.onNav = x.start.onNav;
        sn.navTerr = x.start.navTerr;     // 1.5.1
        sn.depth = x.start.depth;
        sn.deepOk = x.start.deep;
        sn.hasZ = true;
        sn.head = 1;
        sn.bestG = 0.f;
        sn.pushed = true;
        x.hitBase = j.xs.CacheHits();     // 1.5.1 this pass's budget counts its own reused answers (1/4 question each)
        x.best = -1;                      // item 1: a partial end is looked for on the goal's floor only (XPhaseSearchV)
        x.bestD = 3.0e38f;
        x.expIdx = -1;
        x.expD = 0;
        x.expProbe = -1;
        x.nudgeOn = false;
        x.cellNudgeOn = false;
        x.wadePend = -1;
        x.edgeCells = 0;
        x.qBase = j.queries;
        x.startIdx = s;
        // the pass's budget: ~10 questions per square of its OWN box (1.6.0: pass 1's box, not the wider grid it lies in)
        const int boxCols = (x.ax1 - x.ax0 + 1) * (x.ay1 - x.ay0 + 1);
        x.searchQ = std::clamp((std::min)(boxCols, static_cast<int>(cols)) * kVQPerCell, kVSearchQMin, kVSearchQMax);
        if (x.local) {      // (review R1 #6) a repair's own share of the MIX budget (XPhaseNavLegs set it)
            x.searchQ = (std::min)(x.searchQ, (std::max)(0, x.localCap));
        } else {
            ++j.xs.passes;
        }
        x.open.push({ XHeur(x, sn), 0.f, s, -1 });
        // (1.6.0 §6.4) a wade plan from DRY ground toward a seed lower than her floor first floods the floor for the nearest
        // reachable shin-deep water (the seed may lie below a ledge she cannot step down). (fix round, R2 #10) "lower" = the seed's
        // WATER SURFACE more than a riser under her floor (a bank / a ledge), not its bed: almost every seed's bed lies that low
        const float seedTop = IsFiniteHeight(x.seedSurf) ? x.seedSurf : x.asked.z;
        x.floodWanted = x.wade && !x.local && j.wadeFlood && x.start.kind != kKWater && seedTop < x.start.pos.z - x.riser;
        x.floodSt.clear();
        std::vector<float>().swap(x.floodToE);    // (1.6.1 A2) an earlier flood's way back belongs to an earlier grid
        std::vector<float>().swap(x.floodZ);
        x.xphase = x.floodWanted ? kXFlood : kXSearch;
        x.yield = true;
    }

    // the grid (its own slice): B §7.1 box margin clamp(0.75 d, 320, 640)
    void XPhaseGrid(Job& j)
    {
        ExState&    x = *j.x;
        const float dist = Dist2D(x.start.pos, x.goal.pos);
        if (x.v15) {
            XGridV(j, dist);
            return;
        }
        const float margin = std::clamp(dist * kXMarginFrac, kXMarginMin, kXMarginMax);
        x.minX = (std::min)(x.start.pos.x, x.goal.pos.x) - margin;
        x.minY = (std::min)(x.start.pos.y, x.goal.pos.y) - margin;
        const float maxX = (std::max)(x.start.pos.x, x.goal.pos.x) + margin;
        const float maxY = (std::max)(x.start.pos.y, x.goal.pos.y) + margin;
        x.nx = static_cast<int>((maxX - x.minX) / kCell) + 2;
        x.ny = static_cast<int>((maxY - x.minY) / kCell) + 2;
        x.nodes.assign(static_cast<std::size_t>(x.nx) * static_cast<std::size_t>(x.ny), XNode{});
        for (int iy = 0; iy < x.ny; ++iy) {
            for (int ix = 0; ix < x.nx; ++ix) {
                XNode& n = x.nodes[static_cast<std::size_t>(iy) * static_cast<std::size_t>(x.nx) + static_cast<std::size_t>(ix)];
                n.x = x.minX + static_cast<float>(ix) * kCell;
                n.y = x.minY + static_cast<float>(iy) * kCell;
            }
        }
        const int          sx = std::clamp(static_cast<int>(std::lround((x.start.pos.x - x.minX) / kCell)), 0, x.nx - 1);
        const int          sy = std::clamp(static_cast<int>(std::lround((x.start.pos.y - x.minY) / kCell)), 0, x.ny - 1);
        const std::int32_t s = sy * x.nx + sx;
        XNode&             sn = x.nodes[static_cast<std::size_t>(s)];
        sn.x = x.start.pos.x;
        sn.y = x.start.pos.y;
        sn.z = x.start.pos.z;
        sn.zRef = sn.z;
        sn.kind = x.start.kind;
        sn.onNav = x.start.onNav;
        sn.depth = x.start.depth;
        sn.deepOk = x.start.deep;          // 1.4.1: her start region grows from here
        sn.hasZ = true;
        sn.head = 1;
        sn.bestG = 0.f;
        x.best = s;
        x.bestD = Dist2D(x.start.pos, x.goal.pos);
        x.open.push({ kHeurWeight * x.bestD, 0.f, s, -1 });
        x.xphase = kXSearch;
        x.yield = true;
    }

    // 1.6.0 §6.4 THE FLOOR FLOOD (a wade plan from dry ground toward a seed lower than her floor - the 10:32 case: Water Finder's
    // seed below an 80-87 u ledge pulled the search across the ledge top until its budget ran out). From her square, nearest squares
    // first, inside pass 1's box: ONE down ray each (the classification's own window, through the cell cache), the water over it
    // (1/4 question), the slope (0.70; terrain 0.45; a fixed floor 0.66) and the cell rule (a riser + L; terrain h x tan 63 + 8), and
    // ONE knee-height ray between the two squares (a wall; a door is a wall here) - about 2.3 questions a square, no body rules: it
    // only AIMS. The first square with shin-deep water (the entry depth .. the cap) becomes the search's aim (its heuristic) instead
    // of the seed. Bounded (1.6.0: 2,500 squares / 6,000 questions - 1.6.1 below); none found -> the seed as before. One neighbour
    // per unit of work.
    // (1.6.1 A2 - the ledge pond hit 6,000 questions twice, 4.2 s + 3.0 s, R4 §5) the flood is DIRECTED (its order = floor distance +
    // the flat distance to the seed: it reaches the water round a ledge in about a third of the squares a round spread needs), bounded
    // at 4,000 squares / 10,000 questions, and keeps its own steps; once it finds the water, a WAY BACK pass (no question, 256 squares
    // a unit) gives every square it reached its floor distance to that water along those steps - the search's heuristic (XHeur), so
    // the search walks the flood's way round the ledge instead of searching the ledge top first.
    void XPhaseFlood(Job& j, RE::Actor* a_w, RE::bhkWorld* a_bw)
    {
        ExState&          x = *j.x;
        const std::size_t cols = static_cast<std::size_t>(x.nx) * static_cast<std::size_t>(x.ny);
        const auto        aim = [&](std::size_t a_i) {
            const XNode& n = x.nodes[a_i];
            return kVFloodAim * std::hypot(n.x - x.asked.x, n.y - x.asked.y);
        };
        if (x.floodSt.size() != cols) {           // the set-up (its own unit)
            x.floodZ.assign(cols, 0.f);
            x.floodG.assign(cols, 3.0e38f);
            x.floodSt.assign(cols, 0);
            x.floodEdge.assign(cols, 0);
            std::vector<float>().swap(x.floodToE);
            x.floodFound = -1;
            x.floodBack = false;
            x.floodBackSquares = 0;
            decltype(x.floodOpen)().swap(x.floodOpen);
            const std::size_t s = static_cast<std::size_t>(x.startIdx);
            x.floodZ[s] = x.start.pos.z;
            x.floodG[s] = 0.f;
            x.floodSt[s] = 1;
            x.floodOpen.push({ aim(s), x.startIdx });
            x.floodQ0 = j.queries;
            x.floodSquares = 0;
            x.floodCur = -1;
            x.floodDir = 0;
            return;
        }
        const float knee = kKnee * j.scale;
        const auto  finish = [&](std::int32_t a_found) {
            if (a_found >= 0) {
                const XNode& fn = x.nodes[static_cast<std::size_t>(a_found)];
                x.goal.pos = RE::NiPoint3{ fn.x, fn.y, x.floodZ[static_cast<std::size_t>(a_found)] };
                j.xs.flood = std::format("the nearest reachable shin-deep water ({:.0f},{:.0f},{:.0f}) lies {:.0f} u along the floor - the search aims "
                                         "there, not at the seed, and follows the flood's way to it ({} square(s), {} questions; the way back: "
                                         "{} square(s))",
                                         x.goal.pos.x, x.goal.pos.y, x.goal.pos.z, x.floodG[static_cast<std::size_t>(a_found)], x.floodSquares,
                                         j.queries - x.floodQ0, x.floodBackSquares);
            } else {
                j.xs.flood = std::format("no reachable shin-deep water found ({} square(s), {} questions) - the search aims at the seed", x.floodSquares,
                                         j.queries - x.floodQ0);
                std::vector<float>().swap(x.floodToE);
                std::vector<float>().swap(x.floodZ);
            }
            // (fix round, R2 #10) why it ran: the seed's water surface (else its z) lies more than a riser under her floor
            j.xs.flood += std::format(" (run as the seed's {} {:.0f} lies {:.0f} u under her floor)", IsFiniteHeight(x.seedSurf) ? "water surface" : "z",
                                      IsFiniteHeight(x.seedSurf) ? x.seedSurf : x.asked.z,
                                      x.start.pos.z - (IsFiniteHeight(x.seedSurf) ? x.seedSurf : x.asked.z));
            logger::info("PATH #{} flood: {}", j.handle, j.xs.flood);
            // (1.6.1 A2) floodZ stays with floodToE (the search's heuristic; XGridV frees both at the next grid)
            std::vector<float>().swap(x.floodG);
            std::vector<std::uint8_t>().swap(x.floodSt);
            std::vector<std::uint8_t>().swap(x.floodEdge);
            decltype(x.floodOpen)().swap(x.floodOpen);
            // the search starts toward its aim: her square again with the new heuristic, its budget from now
            decltype(x.open)().swap(x.open);
            const XNode& sn = x.nodes[static_cast<std::size_t>(x.startIdx)];
            x.open.push({ XHeur(x, sn), 0.f, x.startIdx, -1 });
            x.qBase = j.queries;
            x.hitBase = j.xs.CacheHits();
            x.xphase = kXSearch;
            x.yield = true;
        };
        while (x.xphase == kXFlood) {
            if (Spent(j)) return;
            // (1.6.1 A2) THE WAY BACK: from the water found, along the flood's own steps (both ends were marked) - a plain Dijkstra on
            // numbers, no question; kVFloodBackBatch squares a unit
            if (x.floodBack) {
                for (int b = 0; b < kVFloodBackBatch; ++b) {
                    if (x.floodOpen.empty()) {
                        finish(x.floodFound);
                        return;
                    }
                    const auto top = x.floodOpen.top();
                    x.floodOpen.pop();
                    const std::size_t ci = static_cast<std::size_t>(top.second);
                    if (top.first > x.floodToE[ci] + 0.01f) continue;
                    ++x.floodBackSquares;
                    const std::uint8_t eb = x.floodEdge[ci];
                    const XNode&       cn = x.nodes[ci];
                    const int          cx = static_cast<int>(ci % static_cast<std::size_t>(x.nx));
                    const int          cy = static_cast<int>(ci / static_cast<std::size_t>(x.nx));
                    for (int d = 0; d < 8; ++d) {
                        if (!(eb & (1u << d))) continue;
                        const int bx = cx + kDir[d][0], by = cy + kDir[d][1];
                        if (bx < 0 || by < 0 || bx >= x.nx || by >= x.ny) continue;
                        const std::size_t bi = static_cast<std::size_t>(by) * static_cast<std::size_t>(x.nx) + static_cast<std::size_t>(bx);
                        const XNode&      bn = x.nodes[bi];
                        const float       L = (std::max)(kCell, std::hypot(bn.x - cn.x, bn.y - cn.y));
                        const float       nd = x.floodToE[ci] + std::hypot(L, x.floodZ[bi] - x.floodZ[ci]);
                        if (nd < x.floodToE[bi]) {
                            x.floodToE[bi] = nd;
                            x.floodOpen.push({ nd, static_cast<std::int32_t>(bi) });
                        }
                    }
                }
                continue;
            }
            if (x.floodCur < 0) {
                if (x.floodOpen.empty() || x.floodSquares >= kVFloodSquares || j.queries - x.floodQ0 >= kVFloodQ) {
                    finish(-1);
                    return;
                }
                const auto top = x.floodOpen.top();
                x.floodOpen.pop();
                const std::size_t ci = static_cast<std::size_t>(top.second);
                if (x.floodSt[ci] == 2 || top.first > x.floodG[ci] + aim(ci) + 0.5f) continue;     // (1.6.1 A2) the order = g + aim
                x.floodSt[ci] = 2;
                x.floodCur = top.second;
                x.floodDir = 0;
                continue;
            }
            if (x.floodDir >= 8) {
                x.floodCur = -1;
                continue;
            }
            const int          d = x.floodDir++;
            const std::size_t  ci = static_cast<std::size_t>(x.floodCur);
            const XNode&       cn = x.nodes[ci];
            const int          mx = static_cast<int>(ci % static_cast<std::size_t>(x.nx)) + kDir[d][0];
            const int          my = static_cast<int>(ci / static_cast<std::size_t>(x.nx)) + kDir[d][1];
            if (mx < 0 || my < 0 || mx >= x.nx || my >= x.ny || !XInBox(x, mx, my)) continue;
            const std::size_t mi = static_cast<std::size_t>(my) * static_cast<std::size_t>(x.nx) + static_cast<std::size_t>(mx);
            if (x.floodSt[mi] >= 2) continue;
            const XNode& mn = x.nodes[mi];
            if (InKeepOut(j, mn.x, mn.y)) {
                x.floodSt[mi] = 3;
                continue;
            }
            const float  z = x.floodZ[ci];
            const float  L = (std::max)(kCell, std::hypot(mn.x - cn.x, mn.y - cn.y));
            const float  band = (std::max)(x.riserT + L, TerrainRise(L));
            const VFloor f = FloorFacts(j, a_bw, mn.x, mn.y, z + band + kXTopPad, z - (std::max)(kXFloorDown, kXFloorDownH * x.H));
            if (f.blocked || !f.found) {
                x.floodSt[mi] = 3;
                continue;
            }
            const bool terr = IsTerrainLayer(f.layer);
            if (!(f.nz >= kXMinNz || (terr && f.nz >= kVTerrainNz) || StairSlope(f.nz, f.layer))) {
                x.floodSt[mi] = 3;
                continue;
            }
            const float dz = std::fabs(f.z - z);
            if (dz > (terr ? (std::max)(x.riser + L, TerrainRise(L)) : x.riser + L)) continue;     // not from here (maybe from another square)
            float depth = 0.f, surf = 0.f;
            if (!FloorDepth(j, a_w, f, mn.x, mn.y, depth, surf) || depth > x.depthCap) {
                x.floodSt[mi] = 3;                // unknown, or deeper than she may wade: the flood does not cross it
                continue;
            }
            const bool doors0 = x.doorsOk;
            x.doorsOk = false;
            const bool wall = XRayBlocked(j, a_bw, RE::NiPoint3{ cn.x, cn.y, z + knee }, RE::NiPoint3{ mn.x, mn.y, f.z + knee }, j.xs.rO);
            x.doorsOk = doors0;
            if (wall) continue;                   // a wall between them (the square may still be reached round it)
            x.floodEdge[ci] |= static_cast<std::uint8_t>(1u << d);              // (1.6.1 A2) a step the flood took (both ends)
            x.floodEdge[mi] |= static_cast<std::uint8_t>(1u << kVFloodOpp[d]);
            const float ng = x.floodG[ci] + std::hypot(L, f.z - z);
            if (ng >= x.floodG[mi]) continue;
            x.floodG[mi] = ng;
            x.floodZ[mi] = f.z;
            x.floodSt[mi] = 1;
            ++x.floodSquares;
            if (depth >= x.wadeMin) {
                // (1.6.1 A2) found: the way back from here runs next (its own units), then the search
                x.floodFound = static_cast<std::int32_t>(mi);
                x.floodBack = true;
                x.floodToE.assign(cols, 3.0e38f);
                x.floodToE[mi] = 0.f;
                decltype(x.floodOpen)().swap(x.floodOpen);
                x.floodOpen.push({ 0.f, static_cast<std::int32_t>(mi) });
                x.floodCur = -1;
                continue;
            }
            x.floodOpen.push({ ng + aim(mi), static_cast<std::int32_t>(mi) });
        }
    }

    // 1.5.0 the MIX gives up (no corridor, a repair found no way, its budget spent): her real start and goal back, the
    // corridor's memory freed, and the full ray search (pass 1, its own fresh budget) runs instead - the 1.4.x way
    // (1.6.0 S2 §2.4) a CHUNK that is not the last never falls back to the full ray search (a partial route there would skip the
    // skeleton between its end and the chunk's end): it answers "no route" at once - the leg is remembered as bad for 60 s (NoteBadLeg,
    // where it failed) and the walk runs the skeleton again from its last checked point
    void XNavGuideFail(Job& j, const std::string& a_why)
    {
        ExState& x = *j.x;
        x.start = x.realStart;
        x.goal = x.realGoal;
        x.local = false;
        x.localDone = false;
        if (x.chunk && !x.chunkFinal) {
            j.xs.mix = std::format("the chunk's corridor failed ({}){}{} - no full ray search for a chunk: the walk runs the skeleton again from its "
                                   "last checked point",
                                   a_why,
                                   (x.ngMoved || x.ngDropped) ? std::format(" (corridor points: {} moved aside, {} dropped)", x.ngMoved, x.ngDropped) :
                                                                std::string(),
                                   x.ngLog.empty() ? std::string() : " - legs:" + x.ngLog);
            logger::info("PATH #{} NAV-GUIDE: {}", j.handle, j.xs.mix);
            std::vector<PCell>().swap(x.ngPts);
            std::vector<PCell>().swap(x.ngOut);
            x.skel.reset();
            x.seg.active = false;
            ExFinish(j, 0, "the chunk's corridor could not be walked: " + a_why);
            return;
        }
        j.xs.mix = std::format("the full ray search ({}){}{}", a_why,
                               (x.ngMoved || x.ngDropped) ? std::format(" (corridor points: {} moved aside, {} dropped)", x.ngMoved, x.ngDropped) : std::string(),
                               x.ngLog.empty() ? std::string() : " - legs:" + x.ngLog);
        logger::info("PATH #{} NAV-GUIDE: {}", j.handle, j.xs.mix);
        std::vector<PCell>().swap(x.ngPts);
        std::vector<PCell>().swap(x.ngOut);
        std::vector<Node>().swap(j.nodes);
        decltype(j.open)().swap(j.open);
        x.skel.reset();
        x.pass = 1;
        x.seg.active = false;
        x.xphase = kXGrid;
        x.yield = true;
    }

    // 1.6.0 §2.2 MIX part 0: THE SKELETON - NavGraph's A* over the loaded navmesh triangles + its funnel, from her start to the goal
    // (each may lie up to 256 u off the navmesh: joined), stepped inside THIS plan's slices (its share of the frame; the search
    // re-resolves the navmeshes at every step). Its points become the corridor (XSkelToCorridor). Its self-check failing (the
    // navmesh data is not what we read it as) or no navmesh read -> the 1.5.x grid corridor when both ends lie on the navmesh, else
    // the full ray search; no triangle path (even with virtual links) -> the full ray search.
    void XPhaseSkeleton(Job& j, RE::Actor* a_w)
    {
        ExState& x = *j.x;
        if (!x.skel) {
            standpoint_nav::SkelOptions so;
            so.tag = static_cast<std::uint32_t>(j.handle);
            so.water = (j.xflags & kWater) != 0u && x.goal.kind == kKWater;     // a goal in water: its water triangles are no detour
            so.clearance = x.rc;
            so.joinMax = 256.f;
            so.links = true;
            for (const auto& d : x.discs) so.discs.push_back(standpoint_nav::Disc{ d.x, d.y, d.z, d.r });
            for (const auto& b : x.bad) so.discs.push_back(standpoint_nav::Disc{ b.x, b.y, b.z, j.radius + kXBadPad });
            x.skel = standpoint_nav::SearchBegin(a_w, x.start.pos, x.goal.pos, so);
        }
        if (!standpoint_nav::SearchStep(*x.skel, x.deadline)) return;    // this slice's share is used: the next slice goes on
        standpoint_nav::SkelInfo               info;
        std::vector<standpoint_nav::SkelPoint> pts;
        standpoint_nav::SearchResult(*x.skel, info, pts);
        x.skel.reset();
        j.xs.skeleton = std::format("{} - {} point(s), {:.0f} u, {} triangle(s) in {} navmesh(es){}{}{}, {} restart(s), {} step(s), {:.2f} ms{}",
                                    info.status == 1 ? "BUILT" : info.status == -1 ? "FAILED" : "NONE", pts.size(), info.length, info.expanded, info.meshes,
                                    info.links ? std::format(", {} virtual link(s)", info.links) : std::string(),
                                    info.startJoined ? ", her start joined" : "", info.goalJoined ? ", the goal joined" : "", info.restarts,
                                    info.slices, info.ms, info.status == 1 ? std::string() : " [" + info.why + "]");
        logger::info("PATH #{} SKELETON: {}", j.handle, j.xs.skeleton);
        if (info.status == 1 && !pts.empty()) {
            x.skelIn = std::move(pts);
            x.skelTotal = static_cast<int>(x.skelIn.size());
            XSkelToCorridor(j, x.skelIn, 2);
            return;
        }
        if (info.status == -1 && x.start.onNav && x.goal.onNav) {    // the navmesh could not be read as expected: the 1.5.x corridor
            j.skelSource = 3;
            x.corrNote = std::format("the 1.5.x grid corridor (the skeleton failed: {})", info.why);
            x.ngStage = 0;
            x.ngBudget = VBoxBudget(x.start.pos, x.goal.pos);
            x.xphase = kXNavCorr;
            x.yield = true;
            return;
        }
        XNavGuideFail(j, std::format("the skeleton: {}", info.why));
    }

    // 1.5.0: the popped step is taken - the cell is closed; its goal test, then its expansion, are the next units
    void XAcceptPop(Job& j, const OpenEntry& a_e)
    {
        ExState& x = *j.x;
        XNode&   n = x.nodes[static_cast<std::size_t>(a_e.idx)];
        n.closed = true;
        n.g = a_e.g;
        n.parent = a_e.parent;
        ++j.expansions;
        x.expIdx = a_e.idx;
        x.expD = -1;
        x.expProbe = -1;
        x.expNbr.fill(kNbrUnset);
    }

    // item 8 (a)+(b), no question: a closed, expanded water cell of the right depth -> how many of its 8 neighbours (the levels
    // it reached) are wading water, [0.10 H, the cap]; -1 = the cell itself is not a candidate
    // (1.6.0 §6.3: in a wade scan the least depth is the scan's band - scan 1: knee until a knee spot is held, then thigh; scan 2:
    // thigh and deeper than the knee spot; a scan's own root is never a candidate)
    int XWadePatch(const ExState& x, const XNode& a_n)
    {
        float dmin = x.wadeMin;
        if (x.wadeScan) dmin = (x.wadeStage == 1 && x.scanKnee < 0) ? x.wadeKnee : (std::max)(x.wadeThigh, x.scanKneeDepth + 0.5f);
        if (a_n.kind != kKWater || a_n.deepOk || a_n.depth < dmin || a_n.depth > x.depthCap) return -1;
        int count = 0;
        for (const std::int32_t mi : x.expNbr) {
            if (mi < 0) continue;
            const XNode& m = x.nodes[static_cast<std::size_t>(mi)];
            if (m.kind == kKWater && !m.deepOk && m.depth >= x.wadeNbrMin && m.depth <= x.depthCap) ++count;
        }
        return count;
    }

    // 1.6.0 §6.3 a wade SCAN round a_root (stage 1: the entry E; stage 2: the knee spot K): the cells keep everything they learnt
    // (kind, floor, depth, headroom, clearance, refused steps), only the search's own marks are reset; a plain Dijkstra (h = 0, the
    // cost = distance) over the cells within wadeScanBodies x H of the root, the same edge tests, the same patch rule and stand-spot
    // test, the cap on every cell; its own budget (4,000 questions). Each pop is one unit, as in the entry search.
    void XWadeScanBegin(Job& j, std::int32_t a_root, int a_stage)
    {
        ExState& x = *j.x;
        XNode&   r = x.nodes[static_cast<std::size_t>(a_root)];
        x.wadeScan = true;
        x.wadeStage = a_stage;
        x.scanRoot = a_root;
        x.scanRX = r.x;
        x.scanRY = r.y;
        x.scanR = j.wadeScanBodies * x.H;
        for (auto& c : x.nodes) {
            c.closed = false;
            c.g = 0.f;
            c.bestG = 3.0e38f;
            c.parent = -1;
        }
        decltype(x.open)().swap(x.open);
        decltype(x.deferred)().swap(x.deferred);
        x.nudgeOn = false;
        x.cellNudgeOn = false;
        x.wadePend = -1;
        x.expIdx = -1;
        x.frontier.clear();
        x.frontierK = 0;
        x.ax0 = 0;                                // a scan may reach past pass 1's box (the whole grid)
        x.ay0 = 0;
        x.ax1 = x.nx - 1;
        x.ay1 = x.ny - 1;
        r.bestG = 0.f;
        x.open.push({ 0.f, 0.f, a_root, -1 });
        x.scanQ0 = j.queries;
        x.qBase = j.queries;
        x.hitBase = j.xs.CacheHits();
        x.searchQ = kVWadeScanQ;
        logger::info("PATH #{} wade: scan {} round ({:.0f},{:.0f},{:.0f}) {:.0f} u deep - within {:.0f} u, {} wins",
                     j.handle, a_stage, r.x, r.y, r.z, r.depth, x.scanR,
                     a_stage == 1 ? std::format("the nearest thigh-deep spot ({:.0f}-{:.0f} u), else the nearest knee-deep one (>= {:.0f} u)", x.wadeThigh,
                                                x.depthCap, x.wadeKnee) :
                                    std::format("the nearest thigh-deep spot ({:.0f}-{:.0f} u)", (std::max)(x.wadeThigh, r.depth + 0.5f), x.depthCap));
    }

    // 1.6.0 §6.3 the wade plan's ONE route: start -> E (the prefix) [-> K (scan 1's chain)] [-> the final spot (the last scan's
    // chain)]; only the final spot is the goal / the wading spot - E and K are ordinary points (no intermediate end, no stop).
    // a_tail: the last scan's chain (its root first; empty = the final spot is the end of what is already joined).
    void XWadeAssemble(Job& j, const std::vector<PCell>& a_tail, float a_depth, int a_patch, int a_stage)
    {
        ExState& x = *j.x;
        x.cells = x.wadePrefix;
        if (a_stage == 2 || (a_stage == 1 && !x.wadeChain1.empty() && a_tail.empty())) {
            for (std::size_t i = 1; i < x.wadeChain1.size(); ++i) x.cells.push_back(x.wadeChain1[i]);
        }
        for (std::size_t i = 1; i < a_tail.size(); ++i) x.cells.push_back(a_tail[i]);
        for (auto& c : x.cells) {
            c.goal = false;
            c.wade = false;
        }
        PCell& e = x.cells.back();
        e.goal = true;
        e.wade = true;
        x.goal = e;
        x.wadeScan = false;
        x.expIdx = -1;
        j.wadeDepth = a_depth;
        j.wadePatch = a_patch;
        j.wadeStage = a_stage;
        j.wadeEntryDepth = x.wadeEntryDepth;
        const PCell& ent = x.wadePrefix.back();
        j.xs.wade = std::format("entry ({:.0f},{:.0f},{:.0f}) {:.0f} u deep (patch {}) {}; final ({:.0f},{:.0f},{:.0f}) {:.0f} u deep, patch {} of 8, {:.0f} u "
                                "from the seed - ONE route of {} cell(s); {} water cell(s) checked - {} too shallow, {} without the patch, {} not a "
                                "place to stand",
                                ent.pos.x, ent.pos.y, ent.pos.z, x.wadeEntryDepth, x.wadeEntryPatch,
                                a_stage == 0 ? std::string("-> nothing deeper within reach") :
                                a_stage == 1 ? std::string("-> scan 1 chose a deeper spot") :
                                               std::format("-> scan 1: knee {:.0f} u -> scan 2 chose a thigh-deep spot", x.wadeKneeDepthUsed),
                                e.pos.x, e.pos.y, e.pos.z, a_depth, a_patch, Dist2D(e.pos, x.asked), x.cells.size(),
                                x.wadeTested + x.wadeShallow + x.wadePatchFail, x.wadeShallow, x.wadePatchFail, x.wadeStandFail);
        if (!j.xs.scans.empty()) j.xs.wade += " | scans: " + j.xs.scans;
        if (x.cells.size() < 2) {
            j.route.clear();
            const std::uint32_t f = kPtGoal | kPtWade | (e.onNav ? static_cast<std::uint32_t>(kPtOnNavmesh) : 0u);
            j.route.push_back(RoutePoint{ e.pos, Kind::kWater, a_depth, f, 0 });
            j.switchIndex = 0;
            j.routeLen = 0.f;
            ExFinish(j, 1, "she already stands in a wading spot");
            return;
        }
        x.xphase = kXSplit;
    }

    // 1.6.0 §6.3 a scan's candidate passed its stand-spot test: a THIGH-deep spot is the answer (the scan pops nearest first, so it is
    // the nearest one - "if thigh deep is found on the first scan, no need for the second one"); scan 1's first KNEE spot is kept
    // (K) and the scan goes on (a thigh spot anywhere in its disc still wins)
    void XWadeScanHit(Job& j, std::int32_t a_i, float a_depth, int a_patch)
    {
        ExState& x = *j.x;
        if (a_depth >= x.wadeThigh) {
            j.xs.scans += std::format("{}scan {}: thigh {:.0f} u at {:.0f} u", j.xs.scans.empty() ? "" : "; ", x.wadeStage, a_depth,
                                      Dist2D(XPos(x.nodes[static_cast<std::size_t>(a_i)]), RE::NiPoint3{ x.scanRX, x.scanRY, 0.f }));
            if (x.wadeStage == 2) x.wadeKneeDepthUsed = x.scanKneeDepth;
            XWadeAssemble(j, XChain(x, a_i), a_depth, a_patch, x.wadeStage);
            return;
        }
        if (x.wadeStage == 1 && x.scanKnee < 0) {
            x.scanKnee = a_i;
            x.scanKneeDepth = a_depth;
            x.scanKneePatch = a_patch;
            x.wadeChain1 = XChain(x, a_i);        // E -> K: closed cells, their parents are final
            j.xs.scans += std::format("{}scan 1: knee {:.0f} u at {:.0f} u", j.xs.scans.empty() ? "" : "; ", a_depth,
                                      Dist2D(XPos(x.nodes[static_cast<std::size_t>(a_i)]), RE::NiPoint3{ x.scanRX, x.scanRY, 0.f }));
            if (x.wadeThigh > x.depthCap) {       // her cap is under thigh depth: nothing deeper may come - K is the answer now
                x.wadeKneeDepthUsed = a_depth;
                XWadeAssemble(j, {}, a_depth, a_patch, 1);
            }
        }
    }

    // 1.6.0 §6.3 a scan ran out (its disc searched, or its budget used): scan 1 with a knee spot K -> scan 2 round K; scan 2 -> K
    // is the answer; scan 1 without one -> the entry E is the answer
    void XWadeScanEnd(Job& j, bool a_budget)
    {
        ExState& x = *j.x;
        if (a_budget) j.xs.scans += std::format("{}scan {} out of its budget", j.xs.scans.empty() ? "" : "; ", x.wadeStage);
        if (x.wadeStage == 1 && x.scanKnee >= 0 && x.wadeThigh <= x.depthCap) {
            XWadeScanBegin(j, x.scanKnee, 2);
            return;
        }
        if (x.wadeStage == 2 || (x.wadeStage == 1 && x.scanKnee >= 0)) {
            if (x.wadeStage == 2) j.xs.scans += "; scan 2: nothing thigh-deep";
            x.wadeKneeDepthUsed = x.scanKneeDepth;
            XWadeAssemble(j, {}, x.scanKneeDepth, x.scanKneePatch, 1);
            return;
        }
        j.xs.scans += std::format("{}scan 1: nothing knee-deep", j.xs.scans.empty() ? "" : "; ");
        XWadeAssemble(j, {}, x.wadeEntryDepth, x.wadeEntryPatch, 0);
    }

    // item 8: the first reachable wading spot - the route ends there (kPtWade). She already stands in one: status 1, her spot.
    // (1.6.0 §6.3, the user's priority rule: it is the ENTRY E; when the cap reaches knee depth and E is not thigh-deep yet, scan 1
    // looks round E for deeper water, then scan 2 round the knee spot - all before she moves, ONE route)
    void XWadeFound(Job& j, std::int32_t a_i, float a_depth, int a_patch)
    {
        ExState& x = *j.x;
        j.wadeDepth = a_depth;
        j.wadePatch = a_patch;
        XBuildPath(j, a_i, false);
        PCell& e = x.cells.back();
        e.goal = true;
        e.wade = true;
        x.goal = e;
        x.expIdx = -1;
        x.wadeEntryDepth = a_depth;
        x.wadeEntryPatch = a_patch;
        j.wadeEntryDepth = a_depth;
        if (j.wadeScanBodies > 0.f && x.depthCap >= x.wadeKnee && a_depth < x.wadeThigh) {
            x.wadePrefix = x.cells;
            x.wadeChain1.clear();
            x.scanKnee = -1;
            x.scanKneeDepth = 0.f;
            XWadeScanBegin(j, a_i, 1);
            return;
        }
        j.xs.wade = std::format("the first reachable wading spot ({:.0f},{:.0f},{:.0f}): {:.0f} u deep, patch {} of 8, {:.0f} u from the seed; "
                                "{} water cell(s) checked - {} too shallow, {} without the patch, {} not a place to stand",
                                e.pos.x, e.pos.y, e.pos.z, a_depth, a_patch, Dist2D(e.pos, x.asked), x.wadeTested + x.wadeShallow + x.wadePatchFail,
                                x.wadeShallow, x.wadePatchFail, x.wadeStandFail);
        if (x.cells.size() < 2) {
            j.route.clear();
            const std::uint32_t f = kPtGoal | kPtWade | (e.onNav ? static_cast<std::uint32_t>(kPtOnNavmesh) : 0u);
            j.route.push_back(RoutePoint{ e.pos, Kind::kWater, a_depth, f, 0 });
            j.switchIndex = 0;
            j.routeLen = 0.f;
            ExFinish(j, 1, "she already stands in a wading spot");
            return;
        }
        x.xphase = kXSplit;
    }

    // 1.5.0 the search found no way (its area or its budget): a second, wider pass when its box can still grow AND the first
    // one reached the box's edge or ran out of budget (a closed room is not helped by a wider box) (item 4); else a local
    // repair's failure gives the MIX up, a wade plan answers "none" (wadeNone), any other a partial route / none
    // (1.6.0 §4 #4 / #5) the second pass CONTINUES the first: after an edge contact in the wider grid made for it (cells, steps,
    // the open list and the partial end kept; pass 1's cells at the edge expanded again), with a fresh budget for the wider box;
    // (fix round, R2 #5) out of budget without an edge contact: ONCE in the same box with a fresh budget; no pass 2 only when the
    // open list ran empty inside the box (the early "no way"). A wade scan's end goes
    // to XWadeScanEnd; a failed local repair tells NavGraph that leg cannot be walked (60 s) before the MIX gives up.
    void XSearchNoWay(Job& j, const char* a_why, bool a_budget)
    {
        ExState& x = *j.x;
        if (x.wadeScan) {
            XWadeScanEnd(j, a_budget);
            return;
        }
        if (x.local) {
            standpoint_nav::NoteBadLeg(x.start.pos, x.goal.pos, 60.f);
            XNavGuideFail(j, std::format("leg {}: the local ray search round it found no way - {}", x.ngK + 1, a_why));
            return;
        }
        if (x.pass == 1 && x.edgeCells > 0 && x.canGrow) {
            j.xs.pass2 = std::format("{} at margin {:.0f} u after {} questions, {} cell(s) at the box's edge", a_why, x.margin, j.queries - x.qBase,
                                     x.edgeCells);
            logger::info("PATH #{} pass 1 found no way ({}): the search goes on in its wider box, margin {:.0f} u, a fresh budget ({} cell(s) at "
                         "the edge expanded again)",
                         j.handle, j.xs.pass2, x.margin2, x.frontier.size());
            x.pass = 2;
            x.ax0 = 0;
            x.ay0 = 0;
            x.ax1 = x.nx - 1;
            x.ay1 = x.ny - 1;
            x.margin = x.margin2;
            x.frontierK = 0;
            x.edgeCells = 0;
            x.qBase = j.queries;
            x.hitBase = j.xs.CacheHits();
            x.searchQ = std::clamp(x.nx * x.ny * kVQPerCell, kVSearchQMin, kVSearchQMax);
            ++j.xs.passes;
            ++j.xs.pass2Continued;
            return;                               // the search goes on (kXSearch)
        }
        // (fix round, R2 #5) out of budget WITHOUT an edge contact (or the grid is the box already): pass 2 continues ONCE in the
        // SAME box with a fresh budget - the open list, the cells, the steps and the partial end kept, nothing replayed. On 11-06
        // all 7 hard plans ended pass 1 so, and the routes of #34 / #38 / #43 came from pass 2's fresh budget (1.5.1's outcome for
        // about half its cost). Only an EMPTY open list inside the box skips pass 2.
        if (x.pass == 1 && a_budget) {
            const int boxCols = (x.ax1 - x.ax0 + 1) * (x.ay1 - x.ay0 + 1);
            j.xs.pass2 = std::format("{} at margin {:.0f} u after {} questions, {} cell(s) at the box's edge - continued in the same box", a_why,
                                     x.margin, j.queries - x.qBase, x.edgeCells);
            logger::info("PATH #{} pass 1 found no way ({}): the search goes on ONCE in the same box with a fresh budget ({} open entr(ies) kept)",
                         j.handle, j.xs.pass2, x.open.size());
            x.pass = 2;
            x.canGrow = false;                    // the box stays: no cell is kept for a wider pass, none expanded again
            x.frontierK = x.frontier.size();
            x.edgeCells = 0;
            x.qBase = j.queries;
            x.hitBase = j.xs.CacheHits();
            x.searchQ = std::clamp(boxCols * kVQPerCell, kVSearchQMin, kVSearchQMax);
            ++j.xs.passes;
            ++j.xs.pass2Same;
            return;                               // the search goes on (kXSearch)
        }
        if (x.pass == 1) {                        // the open list ran empty inside the box: a fresh budget cannot help
            j.xs.pass2 = std::format("skipped - {} ({})", a_why,
                                     x.edgeCells > 0 ? "the grid is the box: no wider pass" : "no cell at the box's edge: a wider box cannot help");
        }
        if (x.wade) {
            j.wadeNone = true;
            j.xs.wade = std::format("none: {} water cell(s) checked - {} too shallow, {} without a patch of {} / 8, {} not a place to stand{}",
                                    x.wadeTested + x.wadeShallow + x.wadePatchFail, x.wadeShallow, x.wadePatchFail, x.wadePatchMin, x.wadeStandFail,
                                    x.wadeLastWhy.empty() ? std::string() : " (last: " + x.wadeLastWhy + ")");
            ExFinish(j, 0, std::format("no reachable wading spot near this water ({}; depth {:.0f}-{:.0f} u, patch >= {} of 8)", a_why, x.wadeMin,
                                       x.depthCap, x.wadePatchMin));
            return;
        }
        XNoWay(j, a_why);
    }

    // 1.5.0 the search (x.v15): 1.4.1's lazy weighted A* with the 1.5.0 units of work - one pop + its edge test (XEdgeTestV);
    // one nudge offset (C11); the goal test of a closed cell (3D: the cell rule + the profile + her body); one neighbour's
    // level look-up / classification; one LEDGE probe; one wade stand-spot test. The partial end candidate lies on the goal's
    // own floor (item 1). A wade plan has no goal test: the first popped water cell passing (a) depth, (b) the patch and
    // (c) the stand spot ends it (item 8).
    void XPhaseSearchV(Job& j, RE::Actor* a_w, RE::bhkWorld* a_bw)
    {
        ExState& x = *j.x;
        while (x.xphase == kXSearch) {
            if (Spent(j)) return;
            if (x.nudgeOn) {                      // C11: a step that failed only beside her, tried aside - one unit per stage (N2)
                XNode&       n = x.nodes[static_cast<std::size_t>(x.nudgeE.idx)];
                const XNode& p = x.nodes[static_cast<std::size_t>(x.nudgeE.parent)];
                if (x.nudgeStage == 1) {          // unit 2: the line she walks, p -> ms
                    if (XNudgeLine(j, a_w, a_bw, p, x.nudgeMs, 0, p.navTerr && n.navTerr, p.navStair || n.navStair)) {
                        x.nudgeDoor = x.curDoor;
                        x.nudgeStage = 2;
                    } else {
                        ++j.xs.jogFails;
                        x.nudgeStage = 0;
                    }
                    continue;
                }
                if (x.nudgeStage == 2) {          // unit 3: the sideways jog, p -> ps (it may pass only the walked line's door)
                    x.nudgeStage = 0;
                    if (XNudgeLine(j, a_w, a_bw, p, x.nudgePs, x.nudgeDoor, p.navTerr, p.navStair)) {
                        XNudgeApply(x, n);
                        ++j.xs.nudged;
                        x.nudgeOn = false;
                        XAcceptPop(j, x.nudgeE);
                    } else {
                        ++j.xs.jogFails;
                    }
                    continue;
                }
                // (1.6.0 §4 #7) the offsets AWAY from the side that blocked first (+1 her left: to her right, negative offsets)
                while (x.nudgeK < static_cast<int>(kVNudge.size())) {
                    const float off = kVNudge[static_cast<std::size_t>(x.nudgeK)];
                    if ((x.nudgeSide > 0 && off > 0.f) || (x.nudgeSide < 0 && off < 0.f)) {
                        ++x.nudgeK;
                        continue;
                    }
                    break;
                }
                if (x.nudgeK < static_cast<int>(kVNudge.size())) {     // unit 1: the next offset's moved end
                    ++j.xs.nudgeTries;
                    if (XNudgeEnd(j, a_w, a_bw, p, n, kVNudge[static_cast<std::size_t>(x.nudgeK++)])) x.nudgeStage = 1;
                    continue;
                }
                // (fix round, R2 #13) the side away from the hit gave nothing: the OTHER side once (the side guess may be wrong -
                // both sides tight, a brink pair); its offsets are the ones skipped above
                if (x.nudgeSide != 0 && !x.nudgeFlipped) {
                    x.nudgeSide = -x.nudgeSide;
                    x.nudgeFlipped = true;
                    x.nudgeK = 0;
                    ++j.xs.nudgeFlips;
                    continue;
                }
                x.nudgeOn = false;
                ++j.rejected;
                if (x.nudgeD >= 0) n.rejectedFrom |= static_cast<std::uint8_t>(1u << x.nudgeD);
                XReopenV(j, x.nudgeE.idx);
                continue;
            }
            if (x.cellNudgeOn) {                  // (review fixes) a cell's own clearance failed: the next of its 8 spots aside
                XNode& n = x.nodes[static_cast<std::size_t>(x.cellE.idx)];
                if (x.cellK < 8) {
                    ++j.xs.nudgeTries;
                    if (XCellNudgeTry(j, a_w, a_bw, n, x.cellK++)) {
                        ++j.xs.nudged;
                        x.cellNudgeOn = false;
                        x.open.push(x.cellE);     // its step is tested again, against the cell's new spot
                    }
                    continue;
                }
                x.cellNudgeOn = false;            // nowhere aside fits: the cell is out, from every side
                n.lip = 3;
                n.kind = kKNone;
                n.why = kWhyTight;
                continue;
            }
            if (x.wadePend >= 0) {                // item 8 (c): the stand-spot test of a wading candidate
                const std::int32_t wi = x.wadePend;
                x.wadePend = -1;
                const XNode& c = x.nodes[static_cast<std::size_t>(wi)];
                float        fz = 0.f, dep = 0.f;
                bool         hf = false;
                std::string  why;
                ++x.wadeTested;
                if (StandSpot(j, a_w, a_bw, c.x, c.y, c.z + x.riser + kXTopPad, c.z - x.riser - kXTopPad, fz, dep, hf, why)) {
                    // (1.6.0 §6.3) in a scan: a thigh spot ends it, a knee spot is kept; else the ENTRY was found (its scans may follow)
                    if (x.wadeScan) {
                        XWadeScanHit(j, wi, dep > 0.f ? dep : c.depth, x.wadePendPatch);
                    } else {
                        XWadeFound(j, wi, dep > 0.f ? dep : c.depth, x.wadePendPatch);
                    }
                    continue;
                }
                ++x.wadeStandFail;
                x.wadeLastWhy = why;
                continue;
            }
            if (x.expIdx >= 0) {
                XNode& n = x.nodes[static_cast<std::size_t>(x.expIdx)];
                if (x.expD < 0) {                 // the goal test owed for this closed cell
                    x.expD = 0;
                    if (!x.wade && Dist2D(XPos(n), x.goal.pos) <= kCell * 1.5f) {
                        XNode g;
                        g.x = x.goal.pos.x;
                        g.y = x.goal.pos.y;
                        g.z = x.goal.pos.z;
                        g.kind = x.goal.kind;
                        g.onNav = x.goal.onNav;
                        g.navTerr = x.goal.navTerr;   // 1.5.1 the terrain allowance for the goal's step
                        g.head = 1;               // the stand-spot test cleared its headroom
                        // the asked goal passed its stand-spot test (the floor at rc round it): a spot by a wall or in a corner
                        // stays reachable - her body stops where it touches. (Verifier N1) a repair's goal is a corridor point
                        // that passed the cells' headroom + clearance rule up front (XPhaseNavCorr) - never held to a new test here.
                        g.lip = 1;
                        if (std::fabs(g.z - n.z) <= CellRise(x, Dist2D(XPos(n), XPos(g)), n.navTerr && g.navTerr) && (!x.goal.deep || n.deepOk) &&
                            XEdgeTestV(j, a_w, a_bw, n, g, false) == 0) {
                            x.goal.door = g.door;
                            XBuildPath(j, x.expIdx, true);
                            x.expIdx = -1;
                            if (x.local) {
                                x.localDone = true;
                                x.xphase = kXNavLegs;
                            } else {
                                x.xphase = kXSplit;
                            }
                            return;
                        }
                    }
                    continue;
                }
                if (!XExpandStepV(j, a_w, a_bw)) return;
                if (!x.wade && !x.local) {        // item 1: a partial end only on the goal's own floor, never an edge cell
                    const float dg = Dist2D(XPos(n), x.goal.pos);
                    if (!n.edge && std::fabs(n.z - x.goal.pos.z) <= x.riser + kVLevelBand &&
                        (x.best < 0 || dg < x.bestD - 0.01f ||
                         (std::fabs(dg - x.bestD) <= 0.01f && n.g < x.nodes[static_cast<std::size_t>(x.best)].g))) {
                        x.best = x.expIdx;
                        x.bestD = dg;
                    }
                }
                if (x.wade) {
                    // (1.6.0 §6.3) a scan's own root is never its candidate
                    const int patch = (x.wadeScan && x.expIdx == x.scanRoot) ? -1 : XWadePatch(x, n);
                    if (patch >= x.wadePatchMin) {
                        x.wadePend = x.expIdx;    // its stand-spot test is the next unit
                        x.wadePendPatch = patch;
                    } else if (patch >= 0) {
                        ++x.wadePatchFail;
                    } else if (!x.wadeScan && n.kind == kKWater && !n.deepOk && n.depth < x.wadeMin) {
                        ++x.wadeShallow;
                    }
                }
                x.expIdx = -1;
                x.reexp = false;
                continue;
            }
            // (1.6.0 §4 #5) a continued pass 2: pass 1's cells at its box's edge are expanded again first (no goal test: done)
            if (x.pass == 2 && x.frontierK < x.frontier.size()) {
                const std::int32_t fi = x.frontier[x.frontierK++];
                if (!x.nodes[static_cast<std::size_t>(fi)].closed) continue;
                ++j.xs.frontierReopened;
                x.expIdx = fi;
                x.expD = 0;
                x.expProbe = -1;
                x.expNbr.fill(kNbrUnset);
                x.reexp = true;
                continue;
            }
            // (1.5.1) the pass's budget: its questions + 1/4 question per answer the cell cache gave (a reused answer is not free
            // for ever: a box the cache knows by heart is still searched within bounds)
            if (j.queries - x.qBase + (j.xs.CacheHits() - x.hitBase) / kVCacheHitDiv >= x.searchQ) {
                XSearchNoWay(j, "out of budget before a way was found", true);
                return;
            }
            // (1.6.0 §4 #7) a DEFERRED side-step runs once it is the best the open list has (or nothing else is left): a cheaper way
            // round found first never pays for it
            if (!x.deferred.empty() && (x.open.empty() || x.deferred.top().e.f <= x.open.top().f)) {
                const ExState::Deferred df = x.deferred.top();
                x.deferred.pop();
                XNode& dn = x.nodes[static_cast<std::size_t>(df.e.idx)];
                if (!dn.closed && dn.kind != kKNone && df.kind == 4 && dn.lip != 2) {
                    x.open.push(df.e);            // the cell was moved aside already: this step is tested against its new spot
                    continue;
                }
                if (dn.closed || dn.kind == kKNone || df.e.g > dn.bestG + 0.01f || (df.d >= 0 && (dn.rejectedFrom & (1u << df.d)))) {
                    ++j.xs.deferSkipped;
                    continue;
                }
                ++j.xs.deferRun;
                if (df.kind == 3) {
                    x.nudgeOn = true;
                    x.nudgeK = 0;
                    x.nudgeStage = 0;
                    x.nudgeE = df.e;
                    x.nudgeD = df.d;
                    x.nudgeSide = df.side;
                    x.nudgeFlipped = false;
                } else {
                    x.cellNudgeOn = true;
                    x.cellK = 0;
                    x.cellE = df.e;
                }
                continue;
            }
            if (x.open.empty()) {
                XSearchNoWay(j, "no way round inside the searched area", false);
                return;
            }
            const OpenEntry e = x.open.top();
            x.open.pop();
            XNode& n = x.nodes[static_cast<std::size_t>(e.idx)];
            if (n.closed || n.kind == kKNone || e.g > n.bestG + 0.01f) continue;     // done already, refused, or a stale entry
            if (e.parent >= 0) {
                const int d = DirIndexV(x, e.parent, e.idx);
                if (d >= 0 && (n.rejectedFrom & (1u << d))) continue;
                if (n.head == 0 || (n.head == 1 && n.lip == 0)) {     // (N2) the cell's own tests: their own unit, once per cell
                    XCellTests(j, a_bw, n);
                    if (n.head == 2) {
                        n.kind = kKNone;
                        n.why = kWhyHeadroom;
                        continue;
                    }
                    x.open.push(e);               // its step is the next unit
                    continue;
                }
                const int r = XEdgeTestV(j, a_w, a_bw, x.nodes[static_cast<std::size_t>(e.parent)], n, !n.deepOk);
                if (r == 2) {                     // no headroom: the cell itself is out, from every side
                    n.kind = kKNone;
                    n.why = kWhyHeadroom;
                    continue;
                }
                if (r == 3 || r == 4) {
                    // C11 (3): only beside her - tried aside (each offset three units); (4) the cell's own clearance - the cell is
                    // tried aside, once, one spot per unit. (1.6.0 §4 #7) both wait in the DEFERRED queue at this entry's f
                    ExState::Deferred df;
                    df.e = e;
                    df.d = d;
                    df.kind = static_cast<std::uint8_t>(r);
                    df.side = static_cast<std::int8_t>(r == 3 ? x.besideSide : 0);
                    x.deferred.push(df);
                    ++j.xs.deferQueued;
                    continue;
                }
                if (r != 0) {
                    ++j.rejected;
                    if (d >= 0) n.rejectedFrom |= static_cast<std::uint8_t>(1u << d);
                    XReopenV(j, e.idx);
                    continue;
                }
            }
            XAcceptPop(j, e);
        }
    }

    // 1: lazy weighted A* (w 1.4): Classify on expansion, the edge test on pop; the closed node nearest the goal is kept.
    // (Review G, RISK 6) Units of work, each checked against this slice's share: one pop + its edge test; the goal test of
    // a closed cell; each neighbour's classification; each LEDGE probe.
    void XPhaseSearch(Job& j, RE::Actor* a_w, RE::bhkWorld* a_bw)
    {
        ExState& x = *j.x;
        if (x.v15) {              // 1.5.0
            XPhaseSearchV(j, a_w, a_bw);
            return;
        }
        while (x.xphase == kXSearch) {
            if (Spent(j)) return;
            if (x.expIdx >= 0) {
                XNode& n = x.nodes[static_cast<std::size_t>(x.expIdx)];
                if (x.expD < 0) {         // the goal test owed for this closed cell
                    x.expD = 0;
                    if (Dist2D(XPos(n), x.goal.pos) <= kCell * 1.5f) {
                        XNode g;
                        g.x = x.goal.pos.x;
                        g.y = x.goal.pos.y;
                        g.z = x.goal.pos.z;
                        g.kind = x.goal.kind;
                        g.onNav = x.goal.onNav;
                        g.head = 1;       // the stand-spot test cleared its headroom
                        // (1.4.1) a goal in her start region's water is reached only from inside the region
                        if (std::fabs(g.z - n.z) <= XStepLimit(x, n, g) && (!x.goal.deep || n.deepOk) && XEdgeTest(j, a_w, a_bw, n, g) == 0) {
                            XBuildPath(j, x.expIdx, true);
                            x.expIdx = -1;
                            x.xphase = kXSplit;
                            return;
                        }
                    }
                    continue;
                }
                if (!XExpandStep(j, a_w, a_bw)) return;
                // (review G, BUG 4) a partial route's end candidate: the closed cell nearest the goal that is NOT an edge
                // cell (its `edge` is known only now) - a cell back from a brink, not the lip itself
                const float dg = Dist2D(XPos(n), x.goal.pos);
                if (!n.edge && (dg < x.bestD - 0.01f || (std::fabs(dg - x.bestD) <= 0.01f && n.g < x.nodes[static_cast<std::size_t>(x.best)].g))) {
                    x.best = x.expIdx;
                    x.bestD = dg;
                }
                x.expIdx = -1;
                continue;
            }
            if (j.queries - x.qBase >= kXMaxSearchQ) {
                XNoWay(j, "out of budget before a way was found");
                return;
            }
            if (x.open.empty()) {
                XNoWay(j, "no way round inside the searched area");
                return;
            }
            const OpenEntry e = x.open.top();
            x.open.pop();
            XNode& n = x.nodes[static_cast<std::size_t>(e.idx)];
            if (n.closed || n.kind == kKNone || e.g > n.bestG + 0.01f) continue;     // done already, refused, or a stale entry
            if (e.parent >= 0) {
                const int d = DirIndex(e.parent, e.idx, x.nx);
                if (d >= 0 && (n.rejectedFrom & (1u << d))) continue;
                const int r = XEdgeTest(j, a_w, a_bw, x.nodes[static_cast<std::size_t>(e.parent)], n);
                if (r == 2) {             // no headroom: the cell itself is out, from every side
                    n.kind = kKNone;
                    n.why = kWhyHeadroom;
                    continue;
                }
                if (r != 0) {
                    ++j.rejected;
                    if (d >= 0) n.rejectedFrom |= static_cast<std::uint8_t>(1u << d);
                    XReopen(j, e.idx);
                    continue;
                }
            }
            n.closed = true;
            n.g = e.g;
            n.parent = e.parent;
            ++j.expansions;
            x.expIdx = e.idx;             // next units: its goal test, then its neighbours
            x.expD = -1;
            x.expProbe = -1;
        }
    }

    // =================================================================================================================
    //  1.5.0 THE MIX (kNavGuide, item 5 / C16): the navmesh says WHICH WAY, the rays decide WHETHER HER BODY FITS
    // =================================================================================================================

    // the corridor's step test: the 4 centre rays (knee, chest, the two diagonals), door-aware - only "which way" (doors,
    // stairs, floors); every leg of it is tested with the full 1.5.0 rules afterwards
    bool NavStepClear(Job& j, RE::bhkWorld* a_bw, const RE::NiPoint3& a_a, const RE::NiPoint3& a_b)
    {
        ExState&    x = *j.x;
        const float s = j.scale;
        const float knee = kKnee * s, chest = kChest * s, lo = kSweepLow * s, hi = kSweepHigh * s;
        x.doorsOk = (j.xflags & kDoors) != 0u;
        x.curDoor = 0;
        const bool blocked =
            XRayBlocked(j, a_bw, RE::NiPoint3{ a_a.x, a_a.y, a_a.z + knee }, RE::NiPoint3{ a_b.x, a_b.y, a_b.z + knee }, j.xs.rG) ||
            XRayBlocked(j, a_bw, RE::NiPoint3{ a_a.x, a_a.y, a_a.z + chest }, RE::NiPoint3{ a_b.x, a_b.y, a_b.z + chest }, j.xs.rG) ||
            XRayBlocked(j, a_bw, RE::NiPoint3{ a_a.x, a_a.y, a_a.z + lo }, RE::NiPoint3{ a_b.x, a_b.y, a_b.z + hi }, j.xs.rG) ||
            XRayBlocked(j, a_bw, RE::NiPoint3{ a_a.x, a_a.y, a_a.z + hi }, RE::NiPoint3{ a_b.x, a_b.y, a_b.z + lo }, j.xs.rG);
        x.doorsOk = false;
        return !blocked;
    }

    // the corridor's grid: 1.3.0's navmesh grid (Job::nodes / open, unused otherwise by a plan with kAllDirect) with the
    // 1.5.0 margin; j.goal = the goal on the navmesh (its heuristic)
    void NavCorrInit(Job& j, const RE::NiPoint3& a_start, const RE::NiPoint3& a_goal)
    {
        j.goal = a_goal;
        const float margin = std::clamp(Dist2D(a_start, a_goal) * kXMarginFrac, kVMarginMin, kVMarginMax);
        j.minX = (std::min)(a_start.x, a_goal.x) - margin;
        j.minY = (std::min)(a_start.y, a_goal.y) - margin;
        const float maxX = (std::max)(a_start.x, a_goal.x) + margin;
        const float maxY = (std::max)(a_start.y, a_goal.y) + margin;
        j.nx = static_cast<int>((maxX - j.minX) / kCell) + 2;
        j.ny = static_cast<int>((maxY - j.minY) / kCell) + 2;
        std::vector<Node>().swap(j.nodes);
        j.nodes.assign(static_cast<std::size_t>(j.nx) * static_cast<std::size_t>(j.ny), Node{});
        decltype(j.open)().swap(j.open);
        for (int iy = 0; iy < j.ny; ++iy) {
            for (int ix = 0; ix < j.nx; ++ix) {
                Node& n = j.nodes[static_cast<std::size_t>(iy) * static_cast<std::size_t>(j.nx) + static_cast<std::size_t>(ix)];
                n.x = j.minX + static_cast<float>(ix) * kCell;
                n.y = j.minY + static_cast<float>(iy) * kCell;
            }
        }
        const int          sx = std::clamp(static_cast<int>(std::lround((a_start.x - j.minX) / kCell)), 0, j.nx - 1);
        const int          sy = std::clamp(static_cast<int>(std::lround((a_start.y - j.minY) / kCell)), 0, j.ny - 1);
        const std::int32_t s = sy * j.nx + sx;
        Node&              sn = j.nodes[static_cast<std::size_t>(s)];
        sn.x = a_start.x;
        sn.y = a_start.y;
        sn.z = a_start.z;
        sn.floor = 1;
        sn.bestG = 0.f;
        j.open.push({ kHeurWeight * Dist2D(a_start, a_goal), 0.f, s, -1 });
    }

    // the corridor's cells (node indices, her start first) cut into legs: at every turn of the lattice path and at least every
    // kVCorrMaxStraight cells (a repair stays local); each cut a point on the navmesh (its floor is measured next); the last
    // point is her goal itself (its stand spot, not the navmesh point under it)
    void NavCorrLegs(Job& j, const std::vector<std::int32_t>& a_path)
    {
        ExState& x = *j.x;
        x.ngPts.clear();
        std::size_t legStart = 0;
        for (std::size_t i = 1; i + 1 < a_path.size(); ++i) {
            const int dPrev = DirIndex(a_path[i - 1], a_path[i], j.nx), dNext = DirIndex(a_path[i], a_path[i + 1], j.nx);
            if (dPrev != dNext || i - legStart >= static_cast<std::size_t>(kVCorrMaxStraight)) {
                PCell c;
                c.pos = Pos(j.nodes[static_cast<std::size_t>(a_path[i])]);
                c.kind = kKNav;
                c.onNav = true;
                x.ngPts.push_back(c);
                legStart = i;
            }
        }
        x.ngPts.push_back(x.goal);
    }

    // MIX part 1: the corridor. 1.3.0's navmesh A* (each cell snapped to the navmesh within 48 u of its parent, 40 u a step,
    // its own 5,000-question cap) from her to the goal's navmesh point, then the leg ends put on their physics floors (one
    // floor ray + the water each; a point with no floor there is dropped - its neighbours' legs join). No navmesh under the
    // goal, no way, out of its budget -> the full ray search. Units: the set-up; one pop; one leg end.
    void XPhaseNavCorr(Job& j, RE::Actor* a_w, RE::bhkWorld* a_bw)
    {
        ExState& x = *j.x;
        if (x.ngStage == 0) {
            x.qNav0 = j.queries;
            x.ngSpot = 0;
            x.ngMoved = 0;
            x.ngDropped = 0;
            RE::NiPoint3 g2;
            if (!SnapNav(j, a_w, x.goal.pos.x, x.goal.pos.y, x.goal.pos.z, kVCorrGoalBand, g2) || Dist2D(g2, x.goal.pos) > kCell) {
                XNavGuideFail(j, "no navmesh within 32 u under the goal");
                return;
            }
            NavCorrInit(j, x.start.pos, g2);
            // (1.5.1) its own cap: 1.3.0's 5,000 questions up to ~1,670 u, then 3 per unit of distance (the 2,400 u range), <= 8,000
            x.corrCap = std::clamp(static_cast<int>(kVCorrQPerU * Dist2D(x.start.pos, g2)), kMaxQueries, kVCorrQMax);
            x.ngStage = 1;
            x.yield = true;               // the grid had this slice
            return;
        }
        while (x.ngStage == 1) {
            if (Spent(j)) return;
            if (j.queries - x.qNav0 >= x.corrCap) {
                XNavGuideFail(j, "the navmesh corridor search ran out of its budget");
                return;
            }
            if (j.open.empty()) {
                XNavGuideFail(j, "the navmesh has no way there");
                return;
            }
            const OpenEntry e = j.open.top();
            j.open.pop();
            Node& n = j.nodes[static_cast<std::size_t>(e.idx)];
            if (n.closed || e.g > n.bestG + 0.01f) continue;
            if (e.parent >= 0) {
                const int d = DirIndex(e.parent, e.idx, j.nx);
                if (d >= 0 && (n.rejectedFrom & (1u << d))) continue;
                if (!NavStepClear(j, a_bw, Pos(j.nodes[static_cast<std::size_t>(e.parent)]), Pos(n))) {
                    if (d >= 0) n.rejectedFrom |= static_cast<std::uint8_t>(1u << d);
                    ReopenFromOthers(j, e.idx);
                    continue;
                }
            }
            n.closed = true;
            n.g = e.g;
            n.parent = e.parent;
            if (Dist2D(Pos(n), j.goal) <= kCell * 1.5f && NavStepClear(j, a_bw, Pos(n), j.goal)) {
                std::vector<std::int32_t> path;
                for (std::int32_t k = e.idx; k >= 0; k = j.nodes[static_cast<std::size_t>(k)].parent) path.push_back(k);
                std::reverse(path.begin(), path.end());
                NavCorrLegs(j, path);
                x.corrCells = static_cast<int>(path.size());
                x.corrQ = j.queries - x.qNav0;
                std::vector<Node>().swap(j.nodes);
                decltype(j.open)().swap(j.open);
                x.ngStage = 2;
                x.ngK = 0;
                break;
            }
            const int ix = e.idx % j.nx, iy = e.idx / j.nx;
            for (int d = 0; d < 8; ++d) {
                const int mx = ix + kDir[d][0], my = iy + kDir[d][1];
                if (mx < 0 || my < 0 || mx >= j.nx || my >= j.ny) continue;
                const std::int32_t mi = my * j.nx + mx;
                Node&              m = j.nodes[static_cast<std::size_t>(mi)];
                if (m.closed || (m.rejectedFrom & (1u << d))) continue;
                if (m.floor == 0) {
                    RE::NiPoint3 got;
                    if (!InKeepOut(j, m.x, m.y) && SnapNav(j, a_w, m.x, m.y, n.z, kLevelBand, got) &&
                        std::hypot(got.x - m.x, got.y - m.y) <= kCell * 0.6f && std::fabs(got.z - n.z) <= kMaxStep) {
                        m.x = got.x;
                        m.y = got.y;
                        m.z = got.z;
                        m.floor = 1;
                    } else {
                        m.floor = 2;
                    }
                }
                if (m.floor != 1 || std::fabs(m.z - n.z) > kMaxStep) continue;
                const float ng = n.g + (Pos(m) - Pos(n)).Length();
                if (ng < m.bestG) {
                    m.bestG = ng;
                    j.open.push({ ng + kHeurWeight * Dist2D(Pos(m), j.goal), ng, mi, e.idx });
                }
            }
        }
        while (x.ngStage == 2) {
            if (x.ngK >= x.ngPts.size()) {
                // (1.6.0) a CHUNK's end (not the goal): it is never a stair (she must never stop on one if the next chunk fails) - the
                // chunk ends at the last corridor point off the stair; that point is the chunk's end (kPtChunkEnd) and this plan's goal
                if (x.chunk && !x.chunkFinal) {
                    while (x.ngPts.size() > 1 && x.ngPts.back().stair) {
                        x.ngPts.pop_back();
                        ++x.ngDropped;
                    }
                    if (x.ngPts.empty()) {
                        XNavGuideFail(j, "no point of the chunk's corridor is a floor she can walk");
                        return;
                    }
                    x.ngPts.back().chunkEnd = true;
                    x.ngPts.back().goal = false;
                    x.goal = x.ngPts.back();
                    x.realGoal = x.goal;
                }
                x.ngOut.clear();
                x.ngK = 0;
                x.ngLog.clear();
                x.xphase = kXNavLegs;
                return;
            }
            if (Spent(j)) return;
            PCell& c = x.ngPts[x.ngK];
            if (c.goal) {                         // her goal: its stand spot as XGoalSpot measured it
                ++x.ngK;
                continue;
            }
            if (x.ngSpot > 0) {                   // (verifier N1) its spots aside, one per unit: the first that fits moves it
                if (x.ngSpot > 8) {               // none fits: not a point she can be at - dropped (its neighbours' legs join)
                    x.ngSpot = 0;
                    ++x.ngDropped;
                    x.ngPts.erase(x.ngPts.begin() + static_cast<std::ptrdiff_t>(x.ngK));
                    continue;
                }
                RE::NiPoint3 sp;
                float        dep = 0.f;
                if (XSpotAside(j, a_w, a_bw, c.pos, x.ngSpot++ - 1, sp, dep, c.navTerr, c.stair)) {
                    c.pos = sp;
                    c.depth = dep;
                    c.kind = dep > 0.f ? kKWater : kKNav;
                    c.navTerr = x.asideTerr;      // (1.5.1, review #5) its new floor decides the terrain allowance
                    c.stair = x.asideStair;       // (1.6.0) ... and the stair allowance
                    x.ngSpot = 0;
                    ++x.ngMoved;
                    ++x.ngK;
                }
                continue;
            }
            const FloorHit h = XFloorRay(j, a_bw, c.pos.x, c.pos.y, c.pos.z + x.riser + kXTopPad, c.pos.z - x.riser - kXTopPad, j.xs.rF);
            // (1.5.1) a corridor point is ON the navmesh (its z is the navmesh's): terrain / ground within a step of it gets the
            // terrain allowance (n.z >= 0.45) - the hills' corridors are no longer dropped point by point
            c.navTerr = h.found && !h.blocked && IsTerrainLayer(h.layer) && std::fabs(h.z - c.pos.z) <= x.step;
            // (1.6.0 §5) ... and a fixed floor within a step of it the stair allowance (n.z >= 0.66): a corridor point on a stair
            c.stair = h.found && !h.blocked && !c.navTerr && StairSlope(h.nz, h.layer) && std::fabs(h.z - c.pos.z) <= x.step;
            float      depth = 0.f, surf = 0.f;
            const bool ok = !h.blocked && h.found && (h.nz >= kXMinNz || (c.navTerr && h.nz >= kVTerrainNz) || c.stair) &&
                            DepthAt(j, a_w, c.pos.x, c.pos.y, h.z, depth, surf) && (depth <= kXWet || ((j.xflags & kWater) && depth <= x.depthCap));
            if (!ok) {
                ++x.ngDropped;
                x.ngPts.erase(x.ngPts.begin() + static_cast<std::ptrdiff_t>(x.ngK));
                continue;
            }
            if (h.nz < kXMinNz) {
                if (c.stair) {
                    ++j.xs.stairCells;
                } else {
                    ++j.xs.terrCells;
                }
            }
            c.pos.z = h.z;
            c.depth = depth > kXWet ? depth : 0.f;
            c.kind = depth > kXWet ? kKWater : kKNav;
            // (verifier N1) every corridor point may become a repair's GOAL: it gets the route cells' own rule up front - its
            // headroom and its clearance (a wall or a lip within rc); failing, its 8 spots aside are tried, else it is dropped.
            // So a repair never aims at a point no cell could be - it is never impossible from the start.
            x.doorsOk = (j.xflags & kDoors) != 0u;
            x.doorSide = false;
            const bool fine = !HeadBlocked(j, a_bw, c.pos.x, c.pos.y, c.pos.z, "at a navmesh corridor point") &&
                              XCellClear(j, a_bw, c.pos.x, c.pos.y, c.pos.z, c.navTerr, c.stair);
            x.doorsOk = false;
            if (fine) {
                ++x.ngK;
            } else {
                x.ngSpot = 1;
            }
        }
    }

    // 1.5.0 item 10 (Runtime Navmesh B §4.3): a navmesh leg her body cannot walk = "the carpet is wrong here". ONE line per
    // blocker per game run (by the blocking reference, else by the 64 u spot): the cell, where, what blocked (the layer, the
    // reference + its plugin) and a suggested navcut box (centre, half-extents, yaw). A closed door is the navmesh's own
    // portal, not an error: no line for it.
    void CarpetLine(Job& j, RE::Actor* a_w, const SegTest& s, const RE::NiPoint3& a_a, const RE::NiPoint3& a_b)
    {
        const ExState& x = *j.x;
        // (review R1 #11) not the carpet's fault: a standing actor, a spot she got stuck at, the keep-out circle, an unloaded cell;
        // (verifier N1) nor her own clearance beside the line (a wall, a narrow gap, a brink) - a navcut would not change it. A
        // side COLUMN hit (a table top, a counter, a shelf the navmesh walks under) keeps its line: that is furniture.
        const std::string_view why = s.why ? s.why : "";
        if (why == "a standing actor (kept out)" || why == "a spot she got stuck at" || why == "the keep-out circle" ||
            why == "no loaded cell under the line" || why == "something beside her (a wall, a counter, a narrow gap)" ||
            why == "a brink or a rise beside her") {
            return;
        }
        RE::TESObjectREFR*  ref = (s.capHave && s.capRef) ? RE::TESForm::LookupByID<RE::TESObjectREFR>(s.capRef) : nullptr;
        RE::TESBoundObject* base = ref ? ref->GetBaseObject() : nullptr;
        if (base && base->Is(RE::FormType::Door)) return;
        const RE::NiPoint3 at = s.capHave ? s.capAt : s.failPos;
        std::uint64_t      key = 0;
        if (ref) {
            key = ref->GetFormID();
        } else {
            const auto q = [](float a_v) { return static_cast<std::uint64_t>(static_cast<std::int64_t>(std::floor(a_v / 64.f)) & 0xFFFFF); };
            key = (1ull << 63) | (q(at.x) << 40) | (q(at.y) << 20) | q(at.z);
        }
        if (!g_carpetSeen.insert(key).second) return;
        RE::TESObjectCELL* cell = x.interior ? a_w->GetParentCell() : nullptr;
        if (!cell) {
            if (auto* tes = RE::TES::GetSingleton()) cell = tes->GetCell(at);
        }
        RE::NiPoint3 c{ at.x, at.y, at.z + x.htop * 0.5f }, h{ x.rc + 8.f, x.rc + 8.f, x.htop * 0.5f };
        float        yaw = 0.f;
        std::string  what = s.capHave ? std::format("layer {} ", s.capLayer) : std::string();
        if (ref && base) {
            const auto& bd = base->boundData;
            const float sc = ref->GetScale(), ang = ref->GetAngleZ();
            const float lx = static_cast<float>(bd.boundMin.x + bd.boundMax.x) * 0.5f * sc;
            const float ly = static_cast<float>(bd.boundMin.y + bd.boundMax.y) * 0.5f * sc;
            const float lz = static_cast<float>(bd.boundMin.z + bd.boundMax.z) * 0.5f * sc;
            const auto  p = ref->GetPosition();
            const float cs = std::cos(ang), sn = std::sin(ang);
            c = RE::NiPoint3{ p.x + cs * lx + sn * ly, p.y - sn * lx + cs * ly, p.z + lz };
            h = RE::NiPoint3{ (std::max)(8.f, static_cast<float>(bd.boundMax.x - bd.boundMin.x) * 0.5f * sc),
                              (std::max)(8.f, static_cast<float>(bd.boundMax.y - bd.boundMin.y) * 0.5f * sc),
                              (std::max)(8.f, static_cast<float>(bd.boundMax.z - bd.boundMin.z) * 0.5f * sc) };
            yaw = ang * 57.2957795f;
            const auto* file = ref->GetFile(0);
            what += std::format("ref {:08X} (base {:08X}) from {}", ref->GetFormID(), base->GetFormID(),
                                file ? std::string(file->GetFilename()) : std::string("(created in game)"));
        } else if (s.capHave) {
            what += "(no reference: terrain / a mesh of the cell)";
        } else {
            what = "the floor itself";
        }
        const char* edid = cell ? cell->GetFormEditorID() : nullptr;
        logger::info("PATH CARPET: cell {:08X}{} at ({:.0f},{:.0f},{:.0f}): the navmesh leg ({:.0f},{:.0f},{:.0f})->({:.0f},{:.0f},{:.0f}) is not walkable "
                     "for her body - {} at {:.0f} u; blocked by {} | suggested navcut box: centre ({:.0f},{:.0f},{:.0f}), half-extents ({:.0f},{:.0f},{:.0f}) "
                     "u, yaw {:.0f} deg",
                     cell ? cell->GetFormID() : 0u, (edid && edid[0]) ? std::format(" '{}'", edid) : std::string(), at.x, at.y, at.z, a_a.x, a_a.y, a_a.z,
                     a_b.x, a_b.y, a_b.z, s.why, s.failAt, what, c.x, c.y, c.z, h.x, h.y, h.z, yaw);
    }

    // MIX part 2: every leg of the corridor tested with the full 1.5.0 rules at floor height (SegRunV: doors as the plan
    // allows, the first blocker kept). A failed leg: its CARPET line, then a LOCAL ray search in a box round that leg
    // (margin max(448, 0.75 d) u) from its start to its end, spliced in. A repair finding no way, or the MIX's budget (the
    // full box's) spent -> the full ray search. All legs good: the route goes on to the pull, the spacing and the corners.
    void XPhaseNavLegs(Job& j, RE::Actor* a_w, RE::bhkWorld* a_bw)
    {
        ExState& x = *j.x;
        if (x.localDone) {                        // a local repair came back: x.cells = that leg's start ... its end
            x.localDone = false;
            x.local = false;
            for (std::size_t i = 1; i < x.cells.size(); ++i) x.ngOut.push_back(x.cells[i]);
            x.start = x.realStart;
            x.goal = x.realGoal;
            ++x.ngRay;
            if (x.ngLegs <= kVNgLegLog) x.ngLog += std::format(" -> repaired ({} q used, {} cell(s))", j.queries - x.ngLocalQ0, x.cells.size());
            std::vector<XNode>().swap(x.nodes);
            decltype(x.open)().swap(x.open);
            ++x.ngK;
        }
        while (x.xphase == kXNavLegs) {
            if (Spent(j)) return;
            if (x.seg.active) {
                SegRun(j, a_w, a_bw);
                if (x.seg.result == 0) return;
                x.seg.active = false;
                PCell& b = x.ngPts[x.ngK];
                ++x.ngLegs;
                if (x.seg.result == 1) {
                    b.door = x.seg.door;
                    b.stair = x.seg.stairUsed;    // (1.6.0) the leg INTO b used the stair allowance (kPtStair)
                    x.ngOut.push_back(b);
                    ++x.ngNav;
                    if (x.ngLegs <= kVNgLegLog) x.ngLog += std::format("{} {}:navmesh{}", x.ngLegs > 1 ? ";" : "", x.ngK + 1, b.door ? " (a door)" : "");
                    ++x.ngK;
                    continue;
                }
                const PCell a = x.ngOut.empty() ? x.realStart : x.ngOut.back();
                CarpetLine(j, a_w, x.seg, a.pos, b.pos);
                // (review R1 #6) a LOCAL repair with its own share of the MIX budget: one leg never eats it all
                // (1.6.0 §4 #8: at most 2,000 questions - 1.5.1's third of the MIX budget, >= 1,500, let one repair spend 6,666)
                const int left = x.ngBudget - (j.queries - x.qNav0);
                x.localCap = (std::min)(left, kVLocalCapQ);
                if (x.ngLegs <= kVNgLegLog) {
                    x.ngLog += std::format("{} {}:navmesh FAILED ({} at {:.0f} u) -> a local ray search (its budget {} q)", x.ngLegs > 1 ? ";" : "", x.ngK + 1,
                                           x.seg.why, x.seg.failAt, x.localCap);
                } else if (x.ngLegs == kVNgLegLog + 1) {
                    x.ngLog += "; ...";
                }
                x.start = a;
                x.start.goal = false;
                x.goal = b;
                x.local = true;
                x.localDone = false;
                x.ngLocalQ0 = j.queries;
                x.pass = 1;
                x.xphase = kXGrid;
                x.yield = true;
                return;
            }
            if (j.queries - x.qNav0 >= x.ngBudget) {
                XNavGuideFail(j, "the MIX's budget is spent");
                return;
            }
            if (x.ngK < x.ngPts.size()) {
                // (review R1 #6) a leg that ENDS in a standing actor's disc, a bad spot or the keep-out circle cannot be walked
                // nor repaired toward that end: the full search at once (no local repair, no leg test)
                const PCell& e = x.ngPts[x.ngK];
                const char*  in = InActorDisc(x, e.pos.x, e.pos.y, e.pos.z) ? "in a standing actor's disc" :
                                  IsBadSpot(j, e.pos.x, e.pos.y, e.pos.z) ? "at a spot she got stuck at" :
                                  InKeepOut(j, e.pos.x, e.pos.y)          ? "in the keep-out circle" :
                                                                            nullptr;
                if (in) {
                    // (1.6.0) a re-run skeleton keeps away from it for 60 s
                    standpoint_nav::NoteBadLeg(x.ngOut.empty() ? x.realStart.pos : x.ngOut.back().pos, e.pos, 60.f);
                    XNavGuideFail(j, std::format("leg {} ends {} at ({:.0f},{:.0f},{:.0f}) - no local repair", x.ngK + 1, in, e.pos.x, e.pos.y, e.pos.z));
                    return;
                }
            }
            if (x.ngK >= x.ngPts.size()) {        // every leg is walkable: the cell path for the finishing phases (all direct)
                x.cells.clear();
                x.cells.push_back(x.realStart);
                x.cells.insert(x.cells.end(), x.ngOut.begin(), x.ngOut.end());
                x.ngDone = true;
                std::vector<XNode>().swap(x.nodes);       // no grid: the pull's pre-check passes, its line tests decide
                j.xs.mix = std::format("the MIX{}: {} corridor cell(s) ({} q) -> {} leg(s): {} navmesh, {} ray-repaired; corridor points {} moved aside, "
                                       "{} dropped (no floor, or a wall / a lip within {:.0f} u and no spot aside) -{}",
                                       x.corrNote.empty() ? std::string() : " on " + x.corrNote, x.corrCells, x.corrQ, x.ngLegs, x.ngNav, x.ngRay,
                                       x.ngMoved, x.ngDropped, x.rc, x.ngLog);
                logger::info("PATH #{} NAV-GUIDE: {}", j.handle, j.xs.mix);
                std::vector<PCell>().swap(x.ngPts);
                std::vector<PCell>().swap(x.ngOut);
                x.xphase = kXSplit;
                return;
            }
            const PCell& a = x.ngOut.empty() ? x.realStart : x.ngOut.back();
            const PCell& b = x.ngPts[x.ngK];
            SegBegin(j, a.pos, b.pos, false, false, (std::max)(a.depth, b.depth), true, a.deep, (j.xflags & kDoors) != 0u, true);
        }
    }

    // 2a B §7.4.1: split at the first cell off the dry navmesh. All direct when ALL_DIRECT, when she starts off it, or when
    // the navmesh prefix is shorter than 384 u. The prefix's last point S must be a place to stand (3 tries back, else
    // all direct).
    void XPhaseSplit(Job& j, RE::Actor* a_w, RE::bhkWorld* a_bw)
    {
        ExState& x = *j.x;
        if (!x.swChosen) {
            x.swChosen = true;
            std::size_t s = 0;
            while (s < x.cells.size() && x.cells[s].kind == kKNav) ++s;
            float plen = 0.f;
            for (std::size_t i = 1; i < s; ++i) plen += Dist2D(x.cells[i - 1].pos, x.cells[i].pos);
            // (1.5.0: a route of the MIX is all direct - every leg of it was tested for her body)
            x.sw = ((j.xflags & kAllDirect) || s < 2 || plen < kXPrefixMin || x.ngDone) ? 0 : s;
            x.qFinish = j.queries;
        }
        while (x.sw >= 2 && x.sw < x.cells.size()) {      // (S = the goal itself was tested already)
            if (Spent(j)) return;
            const PCell& sp = x.cells[x.sw - 1];
            float        fz = 0.f, dep = 0.f;
            bool         hf = false;
            std::string  why;
            if (StandSpot(j, a_w, a_bw, sp.pos.x, sp.pos.y, sp.pos.z + x.step + kXTopPad, sp.pos.z - x.step - kXTopPad, fz, dep, hf, why)) break;
            if (++x.swFails >= 3) {
                x.sw = 0;
                break;
            }
            --x.sw;
        }
        if (x.sw == 1) x.sw = 0;
        x.pulled.clear();
        x.prefixPulled = 0;
        x.pStage = 0;
        x.pFrom = 0;
        if (x.sw >= 2) {
            x.pullSeg = 0;
            x.pEnd = x.sw - 1;
        } else {
            x.pullSeg = 1;
            x.pEnd = x.cells.size() - 1;
        }
        x.xphase = kXPull;
    }

    void XPullTake(ExState& x, std::size_t a_c)
    {
        x.pulled.push_back(x.cells[a_c]);
        x.pFrom = a_c;
        x.pStage = 0;
    }

    // 2 B §7.4.2-3: pull each stretch straight - the prefix with 1.3.0's navmesh-leg test (+ dry), the suffix with the floor
    // test. From each point: the farthest cell the free grid pre-check allows is tested first; when it fails, a bisection
    // between the next cell and it. (Review G, BUG 2) A suffix step taken UNPULLED (one cell to the next) is re-tested with
    // the floor rules - every 16 u: floor, step, slope, depth; her body; headroom - without the side rays (B §5.2: the cell
    // path one cell from a brink stays); it fails -> no route, with where and why. (Review G, RISK 8) Out of its budget the
    // pull does not throw the plan away: the rest is the cell path as the search tested it.
    // (1.5.0: a pulled line passes a door when the plan allows it - the point it reaches carries that door (and a pulled line
    // that passes none clears the door its cell had); an unpulled step is taken as the search tested it - the 1.5.0 edge test
    // IS the line rule set (audit C fix 2: a found path never dies at a re-test); the finishing budget is kVFinishQ)
    void XPhasePull(Job& j, RE::Actor* a_w, RE::bhkWorld* a_bw)
    {
        ExState&   x = *j.x;
        const bool doors = (j.xflags & kDoors) != 0u;
        const int  finishQ = x.v15 ? kVFinishQ : kXMaxFinishQ;
        const auto takeRaw = [&](std::size_t a_c) {
            if (x.pullSeg == 1 && !x.pullOver && !x.v15) {
                ++x.rawLegs;
                x.pCand = a_c;
                x.pStage = 4;
                SegBegin(j, x.cells[x.pFrom].pos, x.cells[a_c].pos, false, false, (std::max)(x.cells[x.pFrom].depth, x.cells[a_c].depth), false,
                         x.cells[x.pFrom].deep);
            } else {
                if (x.v15 && x.pullSeg == 1) ++x.rawLegs;
                XPullTake(x, a_c);
            }
        };
        while (x.xphase == kXPull) {
            if (Spent(j)) return;
            if (!x.pullOver && j.queries - x.qFinish >= finishQ) {
                x.pullOver = true;
                x.seg.active = false;
                x.pStage = 0;
                logger::info("PATH #{} the pull ran out of its budget at cell {} of {}: the rest is the cell path as the search tested it",
                             j.handle, x.pFrom, x.cells.size());
            }
            const bool nav = x.pullSeg == 0;
            if (x.seg.active) {
                SegRun(j, a_w, a_bw);
                if (x.seg.result == 0) return;
                x.seg.active = false;
                const bool pass = x.seg.result == 1;
                if (x.pStage == 4) {             // the re-test of one unpulled cell step
                    if (pass) {
                        XPullTake(x, x.pCand);
                        continue;
                    }
                    const RE::NiPoint3& a = x.cells[x.pFrom].pos;
                    const RE::NiPoint3& b = x.cells[x.pCand].pos;
                    ExFinish(j, 0, std::format("a step of the way failed its floor re-test: ({:.0f},{:.0f},{:.0f}) -> ({:.0f},{:.0f},{:.0f}): {} at {:.0f} u",
                                               a.x, a.y, a.z, b.x, b.y, b.z, x.seg.why, x.seg.failAt));
                    return;
                }
                if (x.pStage == 1) {
                    if (pass) {
                        if (x.v15) {
                            x.cells[x.pCand].door = x.seg.door;
                            x.cells[x.pCand].stair = x.seg.stairUsed;     // (1.6.0) the pulled leg INTO it used the stair allowance
                        }
                        XPullTake(x, x.pCand);
                        continue;
                    }
                    x.pLo = x.pFrom + 1;
                    x.pHi = x.pCand - 1;
                } else if (pass) {
                    x.pLo = x.pCand;
                    x.pLoDoorK = x.pCand;         // (1.5.0) the door its line passed
                    x.pLoDoor = x.seg.door;
                    x.pLoStair = x.seg.stairUsed; // (1.6.0) and its stair use
                } else {
                    x.pHi = x.pCand - 1;
                }
                x.pStage = 2;
                continue;
            }
            if (x.pStage == 0) {
                if (x.pFrom >= x.pEnd) {
                    if (x.pullSeg == 0) {        // the prefix is done: the suffix starts at S
                        x.prefixPulled = x.pulled.size();
                        x.pullSeg = 1;
                        x.pFrom = x.sw - 1;
                        x.pEnd = x.cells.size() - 1;
                        continue;
                    }
                    x.spaced.clear();
                    x.prefixSpaced = 0;
                    x.spaceK = 0;
                    x.spacePrev = x.start;
                    x.xphase = kXSpace;
                    return;
                }
                if (x.pullOver) {                // out of budget: the next cell, as the search tested it
                    XPullTake(x, x.pFrom + 1);
                    continue;
                }
                if (x.pFrom + 1 == x.pEnd) {
                    takeRaw(x.pEnd);
                    continue;
                }
                std::size_t farthest = x.pFrom + 1;
                for (std::size_t c = x.pEnd; c >= x.pFrom + 2; --c) {
                    float deepest = 0.f;
                    if (Precheck(j, x.cells[x.pFrom].pos, x.cells[c].pos, nav, deepest)) {
                        farthest = c;
                        break;
                    }
                }
                if (farthest == x.pFrom + 1) {
                    takeRaw(farthest);
                    continue;
                }
                x.pCand = farthest;
                x.pStage = 1;
                SegBegin(j, x.cells[x.pFrom].pos, x.cells[farthest].pos, nav, true, (std::max)(x.cells[x.pFrom].depth, x.cells[farthest].depth),
                         true, x.cells[x.pFrom].deep, doors);
                continue;
            }
            if (x.pLo >= x.pHi) {
                if (x.pLo == x.pFrom + 1) {
                    takeRaw(x.pLo);              // never tested by the pull: one cell step
                } else {
                    if (x.v15 && x.pLoDoorK == x.pLo) {
                        x.cells[x.pLo].door = x.pLoDoor;
                        x.cells[x.pLo].stair = x.pLoStair;
                    }
                    XPullTake(x, x.pLo);         // a line the pull tested
                }
                continue;
            }
            const std::size_t mid = (x.pLo + x.pHi + 1) / 2;
            float             deepest = 0.f;
            if (!Precheck(j, x.cells[x.pFrom].pos, x.cells[mid].pos, nav, deepest)) {
                x.pHi = mid - 1;
                continue;
            }
            x.pCand = mid;
            x.pStage = 3;
            SegBegin(j, x.cells[x.pFrom].pos, x.cells[mid].pos, nav, true, (std::max)(x.cells[x.pFrom].depth, x.cells[mid].depth), true,
                     x.cells[x.pFrom].deep, doors);
        }
    }

    void XSpaceKeep(ExState& x, std::size_t a_k)
    {
        x.spaced.push_back(x.pulled[a_k]);
        if (a_k < x.prefixPulled) ++x.prefixSpaced;
        x.spacePrev = x.pulled[a_k];
        x.spaceK = a_k + 1;
    }

    // 3 B §7.4.4: the prefix's points >= 64 u apart, the suffix's >= 24 u; a nearer point goes when the line past it to
    // the next one is clear. S and the end always stay. Out of budget: the rest are kept as they are.
    // (1.5.0: the line past a dropped point passes a door when the plan allows it - the next point then carries that door)
    void XPhaseSpace(Job& j, RE::Actor* a_w, RE::bhkWorld* a_bw)
    {
        ExState&  x = *j.x;
        const int finishQ = x.v15 ? kVFinishQ : kXMaxFinishQ;
        while (x.xphase == kXSpace) {
            if (Spent(j)) return;
            if (x.seg.active) {
                SegRun(j, a_w, a_bw);
                if (x.seg.result == 0) return;
                x.seg.active = false;
                if (x.seg.result == 1) {
                    if (x.v15) {
                        x.pulled[x.spaceK + 1].door = x.seg.door;
                        x.pulled[x.spaceK + 1].stair = x.seg.stairUsed;   // (1.6.0) the new leg's stair use
                    }
                    ++x.spaceK;          // she walks past it straight to the next one
                } else {
                    XSpaceKeep(x, x.spaceK);
                }
                continue;
            }
            if (x.spaceK >= x.pulled.size()) {
                x.sharp.assign(x.spaced.size(), 0);
                x.cornerK = 0;
                x.cornerStage = 0;
                x.xphase = kXCorners;
                return;
            }
            const std::size_t k = x.spaceK;
            const PCell&      p = x.pulled[k];
            const bool        last = k + 1 == x.pulled.size();
            const bool        isS = x.prefixPulled > 0 && k + 1 == x.prefixPulled;
            const bool        inPrefix = k < x.prefixPulled;
            const float       minLeg = inPrefix ? kMinLeg : kXMinDirect;
            // too near the point before it - or (review G, RISK 10) too near the END after it: the end always stays
            const bool        crowded = Dist2D(x.spacePrev.pos, p.pos) < minLeg ||
                                 (!inPrefix && k + 2 == x.pulled.size() && Dist2D(p.pos, x.pulled.back().pos) < kXMinDirect);
            if (!last && !isS && j.queries - x.qFinish < finishQ && crowded) {
                const PCell& nx = x.pulled[k + 1];
                SegBegin(j, x.spacePrev.pos, nx.pos, inPrefix, true, (std::max)(x.spacePrev.depth, nx.depth), true, x.spacePrev.deep,
                         (j.xflags & kDoors) != 0u);
                continue;
            }
            XSpaceKeep(x, k);
        }
    }

    // the point at arc length a_s along her route (her start, then the spaced points), clamped to its ends
    RE::NiPoint3 ArcPoint(const ExState& x, float a_s)
    {
        RE::NiPoint3 prev = x.start.pos;
        if (a_s <= 0.f) return prev;
        float acc = 0.f;
        for (const auto& c : x.spaced) {
            const float l = Dist2D(prev, c.pos);
            if (l > 1e-3f && acc + l >= a_s) {
                const float t = (a_s - acc) / l;
                return { prev.x + (c.pos.x - prev.x) * t, prev.y + (c.pos.y - prev.y) * t, prev.z + (c.pos.z - prev.z) * t };
            }
            acc += l;
            prev = c.pos;
        }
        return prev;
    }

    // 1.4.1: does the leg that holds arc length a_s lie in her start region - (review R nit 11) BOTH its ends, so a corner chord
    // that starts after the route left the region never gets the region's limit back
    bool ArcDeep(const ExState& x, float a_s)
    {
        const PCell* prev = &x.start;
        if (a_s <= 0.f) return prev->deep;
        float acc = 0.f;
        for (const auto& c : x.spaced) {
            const float l = Dist2D(prev->pos, c.pos);
            if (acc + l >= a_s) return prev->deep && c.deep;
            acc += l;
            prev = &c;
        }
        return prev->deep;
    }

    // 4 B §7.4.5, made to match the driver (review G, RISK 7): the driver aims kLookAhead (48 u) ahead ALONG the route, so
    // near a vertex P she drives inside the route over the arc from 48 u before P to 48 u after it (spanning other points).
    // When that arc turns more than 15 deg (or P lies more than 6 u off its chord), the chord from 48 u before P to 48 u
    // after it - the far side of all she can cut - is tested with the floor rules; it fails = SHARP (the driver walks to P,
    // then turns). Out of budget: the rest are marked SHARP (the safe side).
    // (1.6.0 §3.3, D §1 item 7) then THE RUN'S CHORD: at a run the driver looks ~126 u ahead (0.35 s x her speed), so it cuts a
    // corner over +-128 u; every corner whose +-128 u arc turns 15-60 deg (or lies more than 16 u off that chord) is chord-tested
    // over +-128 u too - a failure = kPtNoCut (the driver keeps 48 u there and is at jog or slower from the lead before it). A SHARP
    // corner needs no run chord (she walks it); a turn past 60 deg is walked anyway (the turn caps). Out of budget: kPtNoCut.
    void XPhaseCorners(Job& j, RE::Actor* a_w, RE::bhkWorld* a_bw)
    {
        ExState& x = *j.x;
        while (x.xphase == kXCorners) {
            if (Spent(j)) return;
            if (x.seg.active) {
                SegRun(j, a_w, a_bw);
                if (x.seg.result == 0) return;
                x.seg.active = false;
                if (x.cornerStage == 0) {         // the +-48 u chord
                    if (x.seg.result != 1) {
                        x.sharp[x.cornerK] = 1;   // SHARP: she walks to it and turns - no run chord
                        ++x.cornerK;
                    } else {
                        x.cornerStage = 1;        // passed: the run's chord next (when the corner wants one)
                    }
                    continue;
                }
                if (x.seg.result != 1) {          // (1.6.0) the +-128 u chord
                    x.sharp[x.cornerK] = static_cast<std::uint8_t>(x.sharp[x.cornerK] | 2u);
                    ++j.xs.noCut;
                }
                x.cornerStage = 0;
                ++x.cornerK;
                continue;
            }
            if (x.cornerK + 1 >= x.spaced.size()) {
                x.xphase = kXAssemble;
                return;
            }
            const std::size_t i = x.cornerK;
            float             s = 0.f;
            RE::NiPoint3      prev = x.start.pos;
            for (std::size_t k = 0; k <= i; ++k) {
                s += Dist2D(prev, x.spaced[k].pos);
                prev = x.spaced[k].pos;
            }
            const RE::NiPoint3 p = x.spaced[i].pos;
            if (x.cornerStage == 1) {             // (1.6.0) the run's chord for this corner?
                if (!x.v15) {
                    x.cornerStage = 0;
                    ++x.cornerK;
                    continue;
                }
                const RE::NiPoint3 ra = ArcPoint(x, s - kVCornerChordRun), rb = ArcPoint(x, s + kVCornerChordRun);
                const float        rla = Dist2D(ra, p), rlb = Dist2D(p, rb), rab = Dist2D(ra, rb);
                bool               want = rla >= 1.f && rlb >= 1.f && rab >= 1.f;
                if (want) {
                    const float rcos = ((p.x - ra.x) * (rb.x - p.x) + (p.y - ra.y) * (rb.y - p.y)) / (rla * rlb);
                    const float roff = std::fabs((rb.x - ra.x) * (p.y - ra.y) - (rb.y - ra.y) * (p.x - ra.x)) / rab;
                    want = (rcos < kXCornerCos || roff > kVCornerRunOff) && rcos >= kVCornerRunCos;
                }
                if (!want) {
                    x.cornerStage = 0;
                    ++x.cornerK;
                    continue;
                }
                if (j.queries - x.qFinish >= kVFinishQ) {     // out of budget: the safe side
                    x.sharp[i] = static_cast<std::uint8_t>(x.sharp[i] | 2u);
                    ++j.xs.noCut;
                    x.cornerStage = 0;
                    ++x.cornerK;
                    continue;
                }
                ++j.xs.runChords;
                float rdeep = x.spaced[i].depth;
                if (i > 0) rdeep = (std::max)(rdeep, x.spaced[i - 1].depth);
                rdeep = (std::max)(rdeep, x.spaced[i + 1].depth);
                SegBegin(j, ra, rb, false, true, rdeep, true, ArcDeep(x, s - kVCornerChordRun));     // (stage 1: its answer above)
                continue;
            }
            const RE::NiPoint3 a = ArcPoint(x, s - kXCornerChord), b = ArcPoint(x, s + kXCornerChord);
            const float        la = Dist2D(a, p), lb = Dist2D(p, b), ab = Dist2D(a, b);
            if (la < 1.f || lb < 1.f || ab < 1.f) {
                x.cornerStage = 1;                // (1.6.0) no +-48 u test: the run's chord may still be wanted
                continue;
            }
            const float cosT = ((p.x - a.x) * (b.x - p.x) + (p.y - a.y) * (b.y - p.y)) / (la * lb);
            const float off = std::fabs((b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x)) / ab;     // P from the chord
            if (cosT >= kXCornerCos && off <= 6.f) {
                x.cornerStage = 1;
                continue;
            }
            // (1.5.0: the finishing budget is kVFinishQ; a chord never passes a door - a corner cut through a door is SHARP)
            if (j.queries - x.qFinish >= (x.v15 ? kVFinishQ : kXMaxFinishQ)) {
                x.sharp[i] = 1;
                ++x.cornerK;
                continue;
            }
            float deep = x.spaced[i].depth;
            if (i > 0) deep = (std::max)(deep, x.spaced[i - 1].depth);
            deep = (std::max)(deep, x.spaced[i + 1].depth);
            SegBegin(j, a, b, false, true, deep, true, ArcDeep(x, s - kXCornerChord));
        }
    }

    // 5: the answer - flags, the switch index (the prefix's legs that are engine-walkable, from the start on), status
    void XPhaseAssemble(Job& j, RE::Actor* a_w, RE::bhkWorld* a_bw)
    {
        ExState&          x = *j.x;
        const std::size_t n = x.spaced.size();
        if (n == 0) {
            ExFinish(j, 0, "no points after the pull");
            return;
        }
        if (n > static_cast<std::size_t>(kXMaxPoints) && !x.pullOver) {    // (an unpulled cell path is many points by nature)
            ExFinish(j, 0, std::format("too many turns for a local way round ({} points)", n));
            return;
        }
        std::size_t  k = 0;
        RE::NiPoint3 prev = x.start.pos;
        bool         prevNav = x.start.kind == kKNav;
        while (k < x.prefixSpaced && k < static_cast<std::size_t>(kXMaxPrefix)) {
            const PCell& p = x.spaced[k];
            if (!(prevNav && p.kind == kKNav && Dist2D(prev, p.pos) >= kMinLeg)) break;
            prev = p.pos;
            ++k;
        }
        j.switchIndex = static_cast<int>(k);
        j.route.clear();
        j.routeLen = 0.f;
        prev = x.start.pos;
        for (std::size_t i = 0; i < n; ++i) {
            const PCell&  c = x.spaced[i];
            std::uint32_t f = 0;
            if (i < k) f |= kPtEngineLeg;
            if (i < x.sharp.size() && (x.sharp[i] & 1u)) f |= kPtSharp;
            if (i < x.sharp.size() && (x.sharp[i] & 2u)) {     // (1.6.0) the run's chord failed here
                f |= kPtNoCut;
                ++j.noCutPoints;
            }
            if (c.stair) f |= kPtStair;           // (1.6.0) the leg into it uses the stair allowance
            if (c.chunkEnd) f |= kPtChunkEnd;     // (1.6.0) a chunk's pass-through end
            if (c.virt) f |= kPtVirtual;          // (1.6.0) the leg into it proved a virtual link
            if (c.edge) f |= kPtEdge;
            if (c.goal) {
                f |= kPtGoal;
                if (j.goalMovedBy > 0.5f) f |= kPtGoalMoved;
            }
            if (c.onNav) f |= kPtOnNavmesh;
            if (c.door) f |= kPtDoor;     // 1.5.0: the leg INTO this point passes that closed door (the driver opens it first)
            if (c.wade) f |= kPtWade;     // 1.5.0: the wading spot (the end of a kWadeSpot plan)
            j.route.push_back(RoutePoint{ c.pos, static_cast<Kind>(c.kind), c.depth, f, c.door });
            j.routeLen += Dist2D(prev, c.pos);
            prev = c.pos;
        }
        if (x.partial) {                  // its end passed the stand-spot test (kXPartialEnd); the classifier names what is beyond
            const PCell&      e = x.spaced.back();
            const float       dl = Dist2D(e.pos, x.goal.pos);
            const std::string verdict = dl > 1.f ? ClassifyStep(j, a_w, a_bw, e.pos, (x.goal.pos.x - e.pos.x) / dl, (x.goal.pos.y - e.pos.y) / dl) :
                                                   std::string("at the goal");
            ExFinish(j, 3, std::format("partial ({}): ends {:.0f} u from the goal at a place to stand, toward it: {}", x.noWayWhy, dl, verdict));
            return;
        }
        ExFinish(j, n == 1 ? 1 : 2, n == 1 ? "the straight line is clear (after the pull)" : "a way round");
    }

    // ONE slice of an Ex plan: the phases in order until this slice's share is used or the plan is decided
    void ExSlice(Job& j, RE::Actor* a_w, RE::bhkWorld* a_bw)
    {
        ExState& x = *j.x;
        x.yield = false;
        while (j.status == kRunning && !x.yield) {
            const int before = x.xphase;
            switch (x.xphase) {
            case kXMeasure:  XPhaseMeasure(j, a_w); break;
            case kXWaters:   XPhaseWaters(j, a_w, true); x.yield = true; break;     // the scan has a slice to itself
            case kXStart:    XPhaseStart(j, a_w, a_bw); break;
            case kXGoal:     XPhaseGoal(j, a_w, a_bw); break;
            case kXMarch:    XPhaseMarch(j, a_w, a_bw); break;
            case kXGate:     XPhaseGate(j); break;
            case kXLegacy:   XPhaseLegacy(j, a_w, a_bw); break;
            case kXStraight: XPhaseStraight(j, a_w, a_bw); break;
            case kXGrid:     XPhaseGrid(j); break;
            case kXSearch:   XPhaseSearch(j, a_w, a_bw); break;
            case kXPartialEnd: XPhasePartialEnd(j, a_w, a_bw); break;
            case kXSplit:    XPhaseSplit(j, a_w, a_bw); break;
            case kXPull:     XPhasePull(j, a_w, a_bw); break;
            case kXSpace:    XPhaseSpace(j, a_w, a_bw); break;
            case kXCorners:  XPhaseCorners(j, a_w, a_bw); break;
            case kXAssemble: XPhaseAssemble(j, a_w, a_bw); break;
            case kXNavCorr:  XPhaseNavCorr(j, a_w, a_bw); break;      // 1.5.0 the MIX
            case kXNavLegs:  XPhaseNavLegs(j, a_w, a_bw); break;
            case kXSkeleton: XPhaseSkeleton(j, a_w); break;           // 1.6.0 NavGraph's triangle path, stepped here
            case kXRing:     XPhaseRing(j, a_w, a_bw); break;         // 1.6.0 the goal's ring search
            case kXFlood:    XPhaseFlood(j, a_w, a_bw); break;        // 1.6.0 the wade plan's floor flood
            default:         ExFinish(j, -1, "internal: an unknown phase"); break;
            }
            if (x.xphase == before && j.status == kRunning) return;     // the phase used this slice's share
            if (Spent(j)) return;
        }
    }

    // the extended PATH line (B §7.9) + WATER SEEN
    void ExLogDone(const Job& j)
    {
        const ExState& x = *j.x;
        int            nav = 0, flo = 0, wat = 0, none = 0, deepCells = 0;
        int            why[kWhyCount]{};
        for (const auto& c : x.nodes) {
            if (c.deepOk) ++deepCells;
            switch (c.kind) {
            case kKNav:   ++nav; break;
            case kKFloor: ++flo; break;
            case kKWater: ++wat; break;
            case kKNone:
                ++none;
                if (c.why < kWhyCount) ++why[c.why];
                break;
            default: break;
            }
        }
        std::string pts;
        for (const auto& p : j.route) {
            pts += std::format(" ({:.0f},{:.0f},{:.0f} {}", p.pos.x, p.pos.y, p.pos.z, p.kind == Kind::kNav ? 'N' : p.kind == Kind::kFloor ? 'F' : 'W');
            if (p.kind == Kind::kWater) pts += std::format(" d{:.0f}", p.depth);
            if (p.flags) {
                pts += ' ';
                if (p.flags & kPtEngineLeg) pts += 'e';
                if (p.flags & kPtSharp) pts += 's';
                if (p.flags & kPtEdge) pts += 'b';
                if (p.flags & kPtGoal) pts += 'g';
                if (p.flags & kPtGoalMoved) pts += 'm';
                if (p.flags & kPtOnNavmesh) pts += 'n';
                if (p.flags & kPtWade) pts += 'w';          // 1.5.0 (never set by a 1.4.1 plan)
                if (p.flags & kPtNoCut) pts += 'x';         // 1.6.0: no cut at speed (the +-128 u chord failed)
                if (p.flags & kPtStair) pts += 't';         // 1.6.0: a stair leg
                if (p.flags & kPtChunkEnd) pts += 'k';      // 1.6.0: a chunk's pass-through end
                if (p.flags & kPtVirtual) pts += 'v';       // 1.6.0: a proven virtual link
                if (p.flags & kPtDoor) pts += std::format(" door {:08X}", p.door);
            }
            pts += ')';
        }
        const std::size_t n = j.route.size();
        std::string       mode;
        if (n) {
            mode = j.switchIndex == 0 ? std::string(" (all direct)") :
                   static_cast<std::size_t>(j.switchIndex) >= n ? std::string(" (all engine legs)") :
                                                                  std::format(" (hybrid, switch at {} of {})", j.switchIndex, n);
        }
        const char* st = j.status == 2 ? "ROUTE" : j.status == 1 ? "DIRECT" : j.status == 3 ? "PARTIAL" : j.status == 0 ? "NO ROUTE" : "FAILED";
        // (1.5.0's side-column / nudge / corridor rays are 0 on a 1.4.1 plan)
        const int   sorted = j.xs.rF + j.xs.rO + j.xs.rM + j.xs.rR + j.xs.rS + j.xs.rC + j.xs.rK + j.xs.rN + j.xs.rG;
        std::string line = std::format("PATH #{}: {}{} [{}] - {} point(s){}", j.handle, st, mode, j.why, n, pts);
        line += std::format(" | H {:.0f} u, step {:.0f} u, depthCap {:.0f} u, length {:.0f} u", x.H, x.step, x.depthCap, j.routeLen);
        line += std::format(" | {} questions (rays: {} floor, {} body, {} mid, {} headroom, {} side, {} classify, {} navmesh-first; {} navmesh; {} water)",
                            j.queries, j.xs.rF, j.xs.rO, j.xs.rM, j.xs.rR, j.xs.rS, j.xs.rC, j.rays - sorted, j.snaps, j.xs.nW);
        line += std::format(", cells nav {} / floor {} / water {} / none {} (steep {}, nofloor {}, deep {}, blocked {}, headroom {}, keepout {}, "
                            "bad {}, dry-only {}, no cell {})",
                            nav, flo, wat, none, why[kWhySteep], why[kWhyNoFloor], why[kWhyDeep], why[kWhyBlocked], why[kWhyHeadroom],
                            why[kWhyKeepOut], why[kWhyBad], why[kWhyDryOnly], why[kWhyNoCell]);
        line += std::format(", {} cells opened, {} steps blocked, goal moved {:.0f} u, {} slice(s), {:.2f} ms in all, longest slice {:.2f} ms",
                            j.expansions, j.rejected, j.goalMovedBy, j.slices, j.usTotal / 1000.0, j.usMax / 1000.0);
        if (x.v15) {
            // 1.5.0: the body numbers, the passes, the floors per square, the nudges, the doors, the actors, the MIX, the wade
            line += std::format(" | 1.5.0: riser {:.0f} u, clearance {:.0f} u, headroom {:.0f} u; {} pass(es){}; {} extra floor level(s){}; {} of {} "
                                "nudge(s) fitted ({} refused at the jog / the walked line); {} cell(s) with a wall or a lip within {:.0f} u ({} of them "
                                "nowhere 8-16 u aside: tight); {} headroom refusal(s); {} door(s) passable; {} standing actor(s) kept out ({} cell(s)), "
                                "{} moving; rays +{} side-column, +{} nudge, +{} corridor",
                                x.riser, x.rc, x.htop, j.xs.passes, j.xs.pass2.empty() ? std::string() : " (pass 2 after: " + j.xs.pass2 + ")",
                                j.xs.levelsExtra, j.xs.levelFull ? std::format(" ({} square(s) had no room left)", j.xs.levelFull) : std::string(), j.xs.nudged,
                                j.xs.nudgeTries, j.xs.jogFails, j.xs.lipFails, x.rc, why[kWhyTight], j.xs.headFails, j.xs.doorsPassed, j.xs.actorsKept,
                                why[kWhyActor], j.xs.actorsMoving, j.xs.rK, j.xs.rN, j.xs.rG);
            // 1.5.1: the riser's origin, the terrain allowance used, the cell cache's reuse
            line += std::format(" | 1.5.1: {}; terrain allowance (navmesh + terrain / ground: n.z >= {:.2f}, rise <= h x tan 63 + {:.0f} u) "
                                "used by {} cell(s), {} step(s), {} line sample(s) / brink(s); cell cache (60 s): {} of {} down ray(s) answered, {} cell "
                                "test(s), {} step test(s) reused ({} answer(s) in all, each 1/{} question of the search budget), {} kept now, {} held{}",
                                RiserNote(j, x), kVTerrainNz, kVTerrainPad, j.xs.terrCells, j.xs.terrSteps, j.xs.terrLines, j.xs.cacheFloorHits,
                                j.xs.cacheFloorAsks, j.xs.cacheCellHits, j.xs.cacheStepHits, j.xs.CacheHits(), kVCacheHitDiv, j.xs.cacheStores,
                                g_cache.Size(), g_cache.full ? std::format(" ({} answer(s) not kept: a table full of live ones)", g_cache.full) : std::string());
            if (x.corrCap > kMaxQueries) line += std::format("; the navmesh corridor's cap {} questions", x.corrCap);
            // 1.6.0: the stair allowance, the run chords, the deferred side-steps, the continued pass, the ring, the chunk, the skeleton
            line += std::format(" | 1.6.0: stair allowance (a fixed floor on the navmesh, n.z >= {:.2f}) used by {} cell(s), {} step(s), {} line "
                                "sample(s) / brink(s); run chords (+-{:.0f} u) {} tested, {} failed (kPtNoCut); side-steps deferred {} - run {}, "
                                "dropped {}{}{}",
                                kStairNz, j.xs.stairCells, j.xs.stairSteps, j.xs.stairLines, kVCornerChordRun, j.xs.runChords, j.xs.noCut,
                                j.xs.deferQueued, j.xs.deferRun, j.xs.deferSkipped,
                                j.xs.pass2Continued ? std::format("; pass 2 continued pass 1 ({} edge cell(s) expanded again)", j.xs.frontierReopened) :
                                j.xs.pass2Same      ? std::string("; pass 2 continued pass 1 in the same box (a fresh budget)") :
                                                      std::string(),
                                j.xs.ringTried ? std::format("; goal ring{}: {} spot(s) tried, {} not a place to stand, {} on another floor, {} too "
                                                             "tight for her{}",
                                                             x.ringAsked ? " (at the asked height: a stair's slope refused)" : "", j.xs.ringTried,
                                                             j.xs.ringRefused, j.xs.ringOtherFloor, j.xs.ringTight,
                                                             j.xs.ringMarch ? " - then the 1.5.1 march" : "") :
                                                 std::string());
            if (j.xs.nudgeFlips) line += std::format("; {} side-step(s) tried the other side too", j.xs.nudgeFlips);
            if (j.chunked) {
                line += std::format("; CHUNK {} of the skeleton: {} point(s) taken, {:.0f} u left after it", j.chunkFinal ? "(the last)" : "(more follow)",
                                    j.skelTaken, j.remainAfterU);
            }
            if (!j.xs.skeleton.empty()) line += " | skeleton: " + j.xs.skeleton;
            if (!j.xs.flood.empty()) line += " | flood: " + j.xs.flood;
            if (!j.xs.mix.empty()) line += " | mix: " + j.xs.mix;
            if (!j.xs.wade.empty()) line += " | wade: " + j.xs.wade;
        }
        if (x.rawLegs || x.pullOver) {
            line += std::format(" | {} unpulled cell step(s) {}{}", x.rawLegs,
                                x.v15 ? "taken as the search's full 1.5.0 edge test passed them (the line test's rule set: floor, brinks, body, "
                                        "clearance, columns, headroom)" :
                                        "re-tested with the floor rules",
                                x.pullOver ? "; the pull ran out of budget - the rest is the cell path as the search tested it" : "");
        }
        if (x.deepStart) {
            line += std::format(" | DEEP START: she started {:.0f} u deep (cap {:.0f} u) - her start region may use water up to {:.0f} u "
                                "(swim level {:.0f} u): {} cell(s) joined it",
                                x.start.depth, x.depthCap, x.startLimit, x.swimDepth, deepCells);
        }
        if (!j.xs.straight.empty()) line += " | straight line: " + j.xs.straight;
        if (!j.xs.g1.empty()) line += " | navmesh first: " + j.xs.g1;
        if (j.xs.normBad) line += std::format(" | {} floor hit(s) without a usable normal (slope not tested there)", j.xs.normBad);
        logger::info("{}", line);
        if (j.xs.waterSeen || j.xs.wet) {
            logger::info("PATH #{} WATER SEEN {} (layer-11 hits on floor rays; {} wet answer(s) from our surface; physics surface vs ours: max "
                         "delta {:.1f} u; {} hit(s) where ours saw none)",
                         j.handle, j.xs.waterSeen, j.xs.wet, j.xs.waterDeltaMax, j.xs.waterSeenNoOurs);
        }
    }

    // Main thread (an SKSE task): one slice of one plan. The per-frame budget is shared by all running plans.
    void Step(std::int32_t a_handle)
    {
        std::scoped_lock lk(g_mtx);
        const auto it = g_jobs.find(a_handle);
        if (it == g_jobs.end()) return;
        Job& j = *it->second;
        j.taskQueued = false;
        if (j.status != kRunning) return;
        int running = 0;
        for (const auto& [k, v] : g_jobs) {
            if (v->status == kRunning) ++running;
        }
        const int budget = (std::max)(kMinSlice, kSliceQueries / (std::max)(1, running));
        const auto t0 = std::chrono::steady_clock::now();
        ++j.slices;
        auto* w = RE::TESForm::LookupByID<RE::Actor>(j.walker);
        auto* cell = w ? w->GetParentCell() : nullptr;
        auto* bw = cell ? cell->GetbhkWorld() : nullptr;
        if (!w || !bw || !w->Is3DLoaded()) {
            if (j.x) {
                ExFinish(j, -1, "the walker or her physics world is gone");
            } else {
                Finish(j, -1, "the walker or her physics world is gone");
            }
        } else if (j.x) {
            if (j.xflags & kV15Flags) {
                // (1.6.0 §4 #1) a 1.5.0-flag plan's slice is bounded by TIME: its share of the 1.0 ms a frame (the running NavGraph
                // jobs keep 0.25 ms of it TOGETHER - fix round, R2 #14; each job >= 0.05 ms, so 6+ jobs keep 0.05 ms each) - no
                // 96-question share any more (1.5.1's slices all ended on it after 0.15-0.38 ms); a 2,000-question safety cap only
                const int    navJobs = standpoint_nav::RunningJobs();
                const double navUs = navJobs > 0 ? (std::max)(kVNavSliceUs, kVNavJobMinUs * static_cast<double>(navJobs)) : 0.0;
                const double share = (std::max)(kVMinSliceUs, (kXFrameUs - navUs) / static_cast<double>((std::max)(1, running)));
                j.x->qEnd = j.queries + kVSliceSafetyQ;
                j.x->deadline = t0 + std::chrono::microseconds(static_cast<long long>(share));
            } else {
                // 1.4.0 an Ex plan: the same question share + its share of the 1.0 ms per-frame time cap
                j.x->qEnd = j.queries + budget;
                j.x->deadline = t0 + std::chrono::microseconds(static_cast<long long>(kXFrameUs / static_cast<double>((std::max)(1, running))));
            }
            j.x->cacheNow = t0;               // 1.5.1 the cell cache's clock for this slice (its 60 s)
            ExSlice(j, w, bw);
        } else {
            if (j.ex && j.slices == 1) {      // 1.4.0 a StartEx plan on the 1.3.0 search: her H for PlanInfo (main thread)
                float h = 0.f, raw = 0.f, step = 0.f;
                MeasureH(j, w, h, raw, step);
            }
            RunSlice(j, w, bw, j.queries + budget);
        }
        const double us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
        j.usTotal += us;
        j.usMax = (std::max)(j.usMax, us);
        if (j.status != kRunning) {
            if (j.x) {
                ExLogDone(j);
                ExFree(j);                    // the Ex search's memory goes now; the answer (route) stays until Cancel
            } else {
                LogDone(j);
                if (j.ex) LegacyPoints(j, j.route, j.switchIndex, j.routeLen);
            }
        }
    }

    // A small thread per plan: about once a frame (~8 ms) it hands the main thread ONE slice task - never a second one
    // before the first has run (a task that queues itself again would make SKSE drain its queue forever: a freeze).
    bool Pump(std::int32_t a_handle)
    {
        try {
            std::thread([a_handle]() {
                for (int n = 0; n < 7500; ++n) {          // a hard stop after ~60 s
                    std::this_thread::sleep_for(std::chrono::milliseconds(8));
                    bool queue = false;
                    {
                        std::scoped_lock lk(g_mtx);
                        const auto it = g_jobs.find(a_handle);
                        if (it == g_jobs.end() || it->second->status != kRunning) return;
                        if (!it->second->taskQueued) {
                            it->second->taskQueued = true;
                            queue = true;
                        }
                    }
                    if (queue) {
                        auto* task = SKSE::GetTaskInterface();
                        if (!task) return;
                        task->AddTask([a_handle]() { Step(a_handle); });
                    }
                }
                // 1.4.0: an Ex plan still running when its pump stops is ended and its search memory freed (never left
                // "planning" for ever); a 1.3.0 plan is left exactly as before
                std::scoped_lock lk(g_mtx);
                const auto       it = g_jobs.find(a_handle);
                if (it != g_jobs.end() && it->second->x && it->second->status == kRunning) {
                    Job& j = *it->second;
                    ExFinish(j, -1, "the plan outlived its pump (~60 s)");
                    ExLogDone(j);
                    ExFree(j);
                }
            }).detach();
            return true;
        } catch (...) {
            return false;     // (the review) a thread that cannot be made must not take the game down from a native
        }
    }

    // ---- natives (class Standpoint) ----------------------------------------------------------------------------------
    std::int32_t PathStartNative(RE::StaticFunctionTag*, RE::Actor* a_walker, float a_x, float a_y, float a_z, RE::TESObjectREFR* a_avoid)
    {
        return Start(a_walker, RE::NiPoint3{ a_x, a_y, a_z }, a_avoid);
    }

    std::vector<float> PathResultNative(RE::StaticFunctionTag*, std::int32_t a_handle)
    {
        return Result(a_handle);
    }

    void PathCancelNative(RE::StaticFunctionTag*, std::int32_t a_handle)
    {
        Cancel(a_handle);
    }

}  // namespace

std::int32_t Start(RE::Actor* a_walker, const RE::NiPoint3& a_goal, RE::TESObjectREFR* a_avoid)
{
    if (!a_walker || !a_walker->Is3DLoaded() || !a_walker->GetParentCell()) return 0;
    auto job = std::make_unique<Job>();
    job->walker = a_walker->GetFormID();
    job->start = a_walker->GetPosition();
    job->goal = a_goal;
    job->scale = (std::max)(0.5f, a_walker->GetScale());
    float         radius = 20.f;
    std::uint32_t group = 0;
    if (auto* cc = a_walker->GetCharController()) {
        const float r = cc->collisionBound.extents.x;
        if (r > 4.f && r < 96.f) radius = r;
        if (auto* rb = cc->GetRigidBody()) group = rb->collidable.broadPhaseHandle.collisionFilterInfo.filter >> 16;
    }
    job->radius = radius;
    job->rayFilter = 30u | (group << 16);       // her own controller's layer and group: the rays skip her own body
    // the keep-out circle round the walk's own target (her body + the walker's + a margin); never so big that the start or
    // the goal lies inside it (the walk in to a close point ends right at her)
    if (a_avoid && a_avoid != a_walker) {
        float tr = 24.f;
        if (auto* ta = a_avoid->As<RE::Actor>()) {
            if (auto* tcc = ta->GetCharController()) {
                const float r = tcc->collisionBound.extents.x;
                if (r > 4.f && r < 96.f) tr = r;
            }
        }
        const RE::NiPoint3 ap = a_avoid->GetPosition();
        float r = tr + radius + 8.f;
        r = (std::min)(r, std::hypot(job->start.x - ap.x, job->start.y - ap.y) - 1.f);
        r = (std::min)(r, std::hypot(a_goal.x - ap.x, a_goal.y - ap.y) - 1.f);
        if (r >= radius) {
            job->avoidOn = true;
            job->avoidX = ap.x;
            job->avoidY = ap.y;
            job->avoidR = r;
        }
    }
    std::int32_t h = 0;
    {
        std::scoped_lock lk(g_mtx);
        if (g_jobs.size() >= static_cast<std::size_t>(kMaxJobs)) {
            // make room: the oldest FINISHED plan goes (a caller that never fetched it has given up on it)
            std::int32_t  oldest = 0;
            std::uint64_t oldestOrder = UINT64_MAX;
            for (const auto& [k, v] : g_jobs) {
                if (v->status != kRunning && v->order < oldestOrder) {
                    oldest = k;
                    oldestOrder = v->order;
                }
            }
            if (!oldest) return 0;
            g_jobs.erase(oldest);
        }
        h = g_nextHandle++;
        if (g_nextHandle > 0x00FFFFFF) g_nextHandle = 1;
        job->handle = h;
        job->order = ++g_order;
        g_jobs[h] = std::move(job);
    }
    logger::info("PATH #{} asked: {:08X} from ({:.0f},{:.0f},{:.0f}) to ({:.0f},{:.0f},{:.0f}), radius {:.0f} u{}",
                 h, a_walker->GetFormID(), a_walker->GetPositionX(), a_walker->GetPositionY(), a_walker->GetPositionZ(),
                 a_goal.x, a_goal.y, a_goal.z, radius,
                 a_avoid ? std::format(", keeping clear of {:08X}", a_avoid->GetFormID()) : std::string());
    if (!Pump(h)) {
        std::scoped_lock lk(g_mtx);
        const auto it = g_jobs.find(h);
        if (it != g_jobs.end()) {
            Finish(*it->second, -1, "no thread could be started for the plan");
            LogDone(*it->second);
        }
    }
    return h;
}

std::int32_t StartEx(RE::Actor* a_walker, const RE::NiPoint3& a_goal, RE::TESObjectREFR* a_avoid, const PlanOptions& a_o)
{
    if (!a_walker || !a_walker->Is3DLoaded() || !a_walker->GetParentCell()) return 0;
    if (!std::isfinite(a_goal.x) || !std::isfinite(a_goal.y) || !std::isfinite(a_goal.z)) return 0;
    auto job = std::make_unique<Job>();
    // (1.6.0) a later chunk starts at the last point of the route she already walks (PlanOptions::useFrom)
    const bool useFrom = a_o.useFrom && std::isfinite(a_o.from.x) && std::isfinite(a_o.from.y) && std::isfinite(a_o.from.z);
    job->start = useFrom ? a_o.from : a_walker->GetPosition();
    job->goal = a_goal;
    FillWalker(*job, a_walker);
    job->ex = true;
    // (1.5.0: a wade-spot plan walks the physics floor into water by its nature - kFloor and kWater come with kWadeSpot)
    job->xflags = a_o.flags | ((a_o.flags & kWadeSpot) ? static_cast<std::uint32_t>(kFloor | kWater) : 0u);
    job->depthFrac = a_o.depthFrac;
    job->wadeMinFrac = a_o.wadeMinFrac;      // 1.5.0 (kWadeSpot)
    job->wadePatchMin = a_o.wadePatchMin;
    job->avoidId = (a_avoid && a_avoid != a_walker) ? a_avoid->GetFormID() : 0;
    job->tag = a_o.tag;                      // 1.6.0
    job->wadeKneeFrac = a_o.wadeKneeFrac;
    job->wadeThighFrac = a_o.wadeThighFrac;
    job->wadeScanBodies = (std::max)(0.f, a_o.wadeScanBodies);
    job->wadeFlood = a_o.wadeFlood;
    job->legacyOnly = (job->xflags & (kFloor | kWater)) == 0u;
    // (1.6.0) a GIVEN skeleton is the MIX's corridor (kNavGuide implied); not for a wade plan (its search is its own)
    const bool skel = !a_o.skeleton.empty() && !job->legacyOnly && !(job->xflags & kWadeSpot);
    if (skel) job->xflags |= kNavGuide;
    std::string skelNote;
    if (!job->legacyOnly) {
        job->x = std::make_unique<ExState>();
        job->x->asked = a_goal;
    }
    if (skel) {
        // CHUNK MODE (chunkMinU / chunkMaxU): the chunk ends at the first skeleton point at least chunkMinU along the skeleton from
        // the plan's start, then on to the last one still within chunkMaxU along (never past the planner's range); the goal (the
        // skeleton's last point) ends the last chunk. Without chunk mode the whole skeleton is the corridor.
        ExState&           x = *job->x;
        const auto&        P = a_o.skeleton;
        const bool         chunk = a_o.chunkMinU > 0.f || a_o.chunkMaxU > 0.f;
        std::vector<float> arc(P.size(), 0.f);
        float              acc = 0.f;
        RE::NiPoint3       prev = job->start;
        for (std::size_t i = 0; i < P.size(); ++i) {
            acc += (P[i].pos - prev).Length();
            arc[i] = acc;
            prev = P[i].pos;
        }
        std::size_t end = P.size() - 1;
        if (chunk) {
            const float maxU = (std::max)(a_o.chunkMaxU, a_o.chunkMinU);
            std::size_t k = 0;
            while (k + 1 < P.size() && arc[k] < a_o.chunkMinU) ++k;
            while (k + 1 < P.size() && arc[k + 1] <= maxU) ++k;
            while (k > 0 && Dist2D(job->start, P[k].pos) > kRangeV15) --k;
            end = k;
        }
        x.skelTotal = static_cast<int>(P.size());
        x.chunk = chunk;
        x.chunkFinal = end + 1 == P.size();
        x.skelIn.assign(P.begin(), P.begin() + static_cast<std::ptrdiff_t>(end + 1));
        job->chunked = chunk;
        job->chunkFinal = x.chunkFinal;
        job->skelTaken = static_cast<int>(end + 1);
        job->remainAfterU = arc.back() - arc[end];
        if (chunk && !x.chunkFinal) x.asked = P[end].pos;    // the chunk's pass-through end is this plan's goal
        skelNote = chunk ? std::format(", CHUNK MODE ({:.0f}-{:.0f} u): skeleton points 1-{} of {} ({:.0f} u), {:.0f} u after it{}", a_o.chunkMinU,
                                       a_o.chunkMaxU, end + 1, P.size(), arc[end], job->remainAfterU, x.chunkFinal ? " - the last chunk" : "") :
                           std::format(", the skeleton given: {} point(s), {:.0f} u", P.size(), arc.back());
    }
    // the keep-out circle round the walk's own target - Start's rule exactly
    const float radius = job->radius;
    if (a_avoid && a_avoid != a_walker) {
        float tr = 24.f;
        if (auto* ta = a_avoid->As<RE::Actor>()) {
            if (auto* tcc = ta->GetCharController()) {
                const float r = tcc->collisionBound.extents.x;
                if (r > 4.f && r < 96.f) tr = r;
            }
        }
        const RE::NiPoint3 ap = a_avoid->GetPosition();
        float              r = tr + radius + 8.f;
        r = (std::min)(r, std::hypot(job->start.x - ap.x, job->start.y - ap.y) - 1.f);
        r = (std::min)(r, std::hypot(a_goal.x - ap.x, a_goal.y - ap.y) - 1.f);
        if (r >= radius) {
            job->avoidOn = true;
            job->avoidX = ap.x;
            job->avoidY = ap.y;
            job->avoidR = r;
        }
    }
    std::int32_t h = 0;
    {
        std::scoped_lock lk(g_mtx);
        if (g_jobs.size() >= static_cast<std::size_t>(kMaxJobs)) {
            std::int32_t  oldest = 0;
            std::uint64_t oldestOrder = UINT64_MAX;
            for (const auto& [k, v] : g_jobs) {
                if (v->status != kRunning && v->order < oldestOrder) {
                    oldest = k;
                    oldestOrder = v->order;
                }
            }
            if (!oldest) return 0;
            g_jobs.erase(oldest);
        }
        h = g_nextHandle++;
        if (g_nextHandle > 0x00FFFFFF) g_nextHandle = 1;
        job->handle = h;
        job->order = ++g_order;
        g_jobs[h] = std::move(job);
    }
    logger::info("PATH #{} asked (Ex, flags 0x{:02X}{}{}): {:08X} from ({:.0f},{:.0f},{:.0f}){} to ({:.0f},{:.0f},{:.0f}), radius {:.0f} u, "
                 "depthFrac {:.2f}{}{}",
                 h, a_o.flags, (a_o.flags & (kFloor | kWater)) == 0u ? " = the 1.3.0 navmesh-only search" : "",
                 a_o.tag ? std::format(", for #{}", a_o.tag) : std::string(), a_walker->GetFormID(), a_walker->GetPositionX(), a_walker->GetPositionY(),
                 a_walker->GetPositionZ(), useFrom ? std::format(" - the plan starts at ({:.0f},{:.0f},{:.0f})", a_o.from.x, a_o.from.y, a_o.from.z) : std::string(),
                 a_goal.x, a_goal.y, a_goal.z, radius, a_o.depthFrac,
                 a_avoid ? std::format(", keeping clear of {:08X}", a_avoid->GetFormID()) : std::string(), skelNote);
    if (!Pump(h)) {
        std::scoped_lock lk(g_mtx);
        const auto       it = g_jobs.find(h);
        if (it != g_jobs.end()) {
            Job& j = *it->second;
            if (j.x) {
                ExFinish(j, -1, "no thread could be started for the plan");
                ExLogDone(j);
                ExFree(j);
            } else {
                Finish(j, -1, "no thread could be started for the plan");
                LogDone(j);
            }
        }
    }
    return h;
}

bool GetRoute(std::int32_t a_handle, PlanInfo& a_info, std::vector<RoutePoint>& a_pts)
{
    std::scoped_lock lk(g_mtx);
    const auto       it = g_jobs.find(a_handle);
    if (it == g_jobs.end()) {                     // (review G, RISK 13) unknown / dropped (a load, Cancel, the 16-plan eviction)
        a_info.status = -2;                       // 1.3.0's -2 meaning; never "still planning" for ever
        a_info.switchIndex = 0;
        a_info.goalMovedBy = 0.f;
        a_info.depthCap = 0.f;
        a_info.height = 0.f;
        a_info.length = 0.f;
        a_info.questions = 0;
        a_info.why = "no such plan (unknown handle, dropped by a load, Cancel or the 16-plan limit)";
        a_info.atSwimLevel = false;
        a_info.wadeNone = false;
        a_info.wadeDepth = 0.f;
        a_info.wadePatch = 0;
        a_info.riser = 0.f;
        a_info.terrainSteps = 0;
        a_info.cacheHits = 0;
        a_info.riserTerrain = 0.f;
        a_info.outsideLoaded = false;             // 1.6.0
        a_info.chunked = false;
        a_info.chunkFinal = false;
        a_info.skelTaken = 0;
        a_info.remainAfterU = 0.f;
        a_info.skelSource = 0;
        a_info.skelLength = 0.f;
        a_info.stairSteps = 0;
        a_info.noCutPoints = 0;
        a_info.wadeStage = 0;
        a_info.wadeEntryDepth = 0.f;
        a_pts.clear();
        return true;
    }
    if (it->second->status == kRunning) return false;
    const Job& j = *it->second;
    a_info.status = j.status;
    a_info.goalMovedBy = j.goalMovedBy;
    a_info.depthCap = j.depthCap;
    a_info.height = j.height;
    a_info.questions = j.queries;
    a_info.why = j.why;
    a_info.atSwimLevel = j.swimStart;     // 1.4.1
    a_info.wadeNone = j.wadeNone;         // 1.5.0 (kWadeSpot)
    a_info.wadeDepth = j.wadeDepth;
    a_info.wadePatch = j.wadePatch;
    a_info.riser = j.riserUsed;           // 1.5.1 (0 = an old-flag plan): 1.5.0's riser, every floor
    a_info.terrainSteps = j.xs.terrCells + j.xs.terrSteps + j.xs.terrLines;
    a_info.cacheHits = j.xs.CacheHits();
    a_info.riserTerrain = j.riserTUsed;   // 1.5.1 (review fix #2): the terrain riser, land on the navmesh only
    a_info.outsideLoaded = j.outsideLoaded;   // 1.6.0
    a_info.chunked = j.chunked;
    a_info.chunkFinal = j.chunkFinal;
    a_info.skelTaken = j.skelTaken;
    a_info.remainAfterU = j.remainAfterU;
    a_info.skelSource = j.skelSource;
    a_info.skelLength = j.skelLength;
    a_info.stairSteps = j.xs.stairCells + j.xs.stairSteps + j.xs.stairLines;
    a_info.noCutPoints = j.noCutPoints;
    a_info.wadeStage = j.wadeStage;
    a_info.wadeEntryDepth = j.wadeEntryDepth;
    if (j.ex) {
        a_pts = j.route;
        a_info.switchIndex = j.switchIndex;
        a_info.length = j.routeLen;
    } else {                                      // a 1.3.0 Start plan: its waypoints as route points
        int   sw = 0;
        float len = 0.f;
        LegacyPoints(j, a_pts, sw, len);
        a_info.switchIndex = sw;
        a_info.length = len;
    }
    return true;
}

void MarkBad(RE::FormID a_walker, const RE::NiPoint3& a_spot, float a_seconds)
{
    if (!a_walker) return;
    const float life = std::clamp(a_seconds > 0.f ? a_seconds : kXBadLifeDef, kXBadLifeMin, kXBadLifeMax);
    const auto  now = std::chrono::steady_clock::now();
    std::size_t live = 0;
    {
        std::scoped_lock lk(g_mtx);
        auto&            v = g_bad[a_walker];
        v.erase(std::remove_if(v.begin(), v.end(), [&now](const BadSpot& b) { return b.until <= now; }), v.end());
        if (v.size() >= kXBadMax) {
            v.erase(std::min_element(v.begin(), v.end(), [](const BadSpot& l, const BadSpot& r) { return l.until < r.until; }));
        }
        BadSpot b;
        b.x = a_spot.x;
        b.y = a_spot.y;
        b.z = a_spot.z;
        b.until = now + std::chrono::milliseconds(static_cast<long long>(life * 1000.f));
        v.push_back(b);
        live = v.size();
    }
    logger::info("PATH bad spot for {:08X} at ({:.0f},{:.0f},{:.0f}) for {:.0f} s ({} live)", a_walker, a_spot.x, a_spot.y, a_spot.z, life, live);
}

namespace {
    // a plan that is never queued: the probes run it to the end at once (main thread)
    bool ProbeJob(Job& j, RE::Actor* a_walker, const RE::NiPoint3& a_a, const RE::NiPoint3& a_b, float a_depthFrac, RE::bhkWorld*& a_bw)
    {
        if (!a_walker || !a_walker->Is3DLoaded()) return false;
        auto* cell = a_walker->GetParentCell();
        a_bw = cell ? cell->GetbhkWorld() : nullptr;
        if (!a_bw) return false;
        FillWalker(j, a_walker);
        j.ex = true;
        j.xflags = kFloor | kWater | kDoors;     // 1.5.0: the walk's own rules (a closed ordinary door on the leg is passed)
        j.depthFrac = a_depthFrac;
        j.start = a_a;
        j.goal = a_b;
        j.x = std::make_unique<ExState>();
        ExState& x = *j.x;
        x.asked = a_b;
        x.qEnd = (std::numeric_limits<int>::max)();
        x.deadline = (std::chrono::steady_clock::time_point::max)();
        MeasureH(j, a_walker, x.H, x.Hraw, x.step);
        x.depthCap = j.depthCap;
        j.cone = ReadHull(a_walker, nullptr);     // 1.5.1 the riser from her hull's cone - the walk's own (main thread)
        {
            std::scoped_lock lk(g_mtx);
            SnapshotBad(j);
            ConeMemo(j);                          // 1.5.1 a failed read: her last good cone (the plan's riser)
        }
        MeasureV(j, x);                           // 1.5.0 the body rules (x.v15, riser, clearance, headroom)
        XPhaseWaters(j, a_walker, false);
        return true;
    }
}

bool ProbeSegment(RE::Actor* a_walker, const RE::NiPoint3& a_a, const RE::NiPoint3& a_b, float a_depthFrac, bool a_quiet, ProbeNote* a_note)
{
    const auto    t0 = std::chrono::steady_clock::now();
    Job           j;
    RE::bhkWorld* bw = nullptr;
    if (a_note) *a_note = ProbeNote{};
    if (!ProbeJob(j, a_walker, a_a, a_b, a_depthFrac, bw)) {
        if (a_note) a_note->why = "the walker or her physics world is gone";
        return false;
    }
    ExState& x = *j.x;
    SegBegin(j, a_a, a_b, false, false, 0.f, true, false, (j.xflags & kDoors) != 0u);
    SegRun(j, a_walker, bw);
    const bool   ok = x.seg.result == 1;
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    if (a_note) {
        a_note->why = ok ? std::string() : std::string(x.seg.why ? x.seg.why : "blocked");
        a_note->failAt = ok ? 0.f : x.seg.failAt;
        a_note->questions = j.queries;
        a_note->ms = static_cast<float>(ms);
    }
    if (!a_quiet) {
        logger::info("PATH probe {:08X}: segment ({:.0f},{:.0f},{:.0f}) -> ({:.0f},{:.0f},{:.0f}) {}{} | {} questions, {:.2f} ms", j.walker, a_a.x, a_a.y,
                     a_a.z, a_b.x, a_b.y, a_b.z, ok ? "CLEAR" : "BLOCKED: ", ok ? std::string() : std::format("{} at {:.0f} u", x.seg.why, x.seg.failAt),
                     j.queries, ms);
    }
    return ok;
}

bool ProbeStandSpot(RE::Actor* a_walker, const RE::NiPoint3& a_p, float a_depthFrac, float& a_floorZ, float& a_depth, bool a_quiet, ProbeNote* a_note,
                    float a_terrainStandNz, RE::FormID a_ignoreRef)
{
    const auto    t0 = std::chrono::steady_clock::now();
    Job           j;
    RE::bhkWorld* bw = nullptr;
    a_floorZ = a_p.z;
    a_depth = 0.f;
    if (a_note) *a_note = ProbeNote{};
    if (!ProbeJob(j, a_walker, a_walker ? a_walker->GetPosition() : a_p, a_p, a_depthFrac, bw)) {
        if (a_note) a_note->why = "the walker or her physics world is gone";
        return false;
    }
    ExState&    x = *j.x;
    x.standNzTerr = std::clamp(a_terrainStandNz, kStandNzTerrain - 0.25f, kStandNz);
    x.ignoreRef = a_ignoreRef;                    // (1.6.1 A3) the side rays step through the target's own body
    x.probeNotes = true;                          // ... and name what they hit
    bool        hasFloor = false;
    std::string why;
    const bool  ok = StandSpot(j, a_walker, bw, a_p.x, a_p.y, a_p.z + x.step + kXGoalTopPad, a_p.z - kXFloorDown, a_floorZ, a_depth, hasFloor, why);
    if (!hasFloor) a_floorZ = a_p.z;
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    const bool   sideRefusal = !ok && x.standSide >= 0;
    if (a_note) {
        a_note->why = ok ? std::string() : why;
        a_note->nz = x.standSeenNz;
        a_note->terrainNav = x.standSeenTerr;
        a_note->questions = j.queries;
        a_note->ms = static_cast<float>(ms);
        // (1.6.1 A3) the side that refused and what it measured
        a_note->centreZ = hasFloor ? x.standCentreZ : a_p.z;
        a_note->window = x.riser;
        a_note->rc = x.rc;
        a_note->ignored = x.ignoredHits;
        if (sideRefusal) {
            a_note->side = x.standSide;
            a_note->sideWhat = x.standSideWhat;
            a_note->sideZ = x.standSideZ;
            a_note->sideDepth = x.standSideDepth;
            a_note->sideLayer = x.standSideLayer;
            a_note->sideRef = x.standSideRef;
        }
    }
    if (!a_quiet) {
        static constexpr const char* kSideName[4] = { "N", "E", "S", "W" };
        std::string side;
        if (sideRefusal && x.standSide < 4) {
            const char* what = x.standSideWhat == 1 ? "blocked (a tree / an invisible wall)" :
                               x.standSideWhat == 2 ? "no floor in its window" :
                               x.standSideWhat == 3 ? "deep water" :
                               x.standSideWhat == 4 ? "no loaded cell" : "a floor";
            side = std::format(" [side {} at {:.0f} u: {} at z {:.0f} = {:+.0f} u from the centre floor {:.0f} (the window +-{:.0f} u), layer {}{}]",
                               kSideName[x.standSide], x.rc, what, x.standSideZ, x.standSideZ - x.standCentreZ, x.standCentreZ, x.riser,
                               x.standSideLayer, x.standSideRef ? std::format(", ref {:08X}", x.standSideRef) : std::string());
        }
        logger::info("PATH probe {:08X}: stand spot ({:.0f},{:.0f},{:.0f}) {}{} - floor z {:.0f}, depth {:.0f} u (cap {:.0f} u){} | {} questions, {:.2f} ms",
                     j.walker, a_p.x, a_p.y, a_p.z, ok ? "OK" : "REFUSED: " + why, side, a_floorZ, a_depth, x.depthCap,
                     a_ignoreRef ? std::format(" | its side rays stepped through {:08X} {} time(s)", a_ignoreRef, x.ignoredHits) : std::string(),
                     j.queries, ms);
    }
    return ok;
}

// v1.5.0 the ONE door rule of the planner and the driver (reviews R2 S1 + S3) - see PathPlanner.h. Main thread.
bool IsOpenableDoor(RE::TESObjectREFR* a_ref)
{
    if (!a_ref || a_ref->IsDeleted() || a_ref->IsDisabled()) return false;
    auto* base = a_ref->GetBaseObject();
    if (!base || !base->Is(RE::FormType::Door)) return false;                    // a TESObjectDOOR (a BGSOpenCloseForm)
    if (a_ref->extraList.HasType<RE::ExtraTeleport>()) return false;            // a load door
    // wired: it activates something (ExtraActivateRefChildren - a trap trigger, a puzzle, a quest chain) or something
    // activates through it (ExtraActivateRef - it is someone's activate child, e.g. a hinge trap trigger, itself a Door form)
    if (a_ref->extraList.HasType<RE::ExtraActivateRef>() || a_ref->extraList.HasType<RE::ExtraActivateRefChildren>()) return false;
    if (const auto* lk = a_ref->GetLock(); lk && lk->IsLocked()) return false;   // the lock FLAG (REFR_LOCK::kLocked), not the level
    return RE::BGSOpenCloseForm::GetOpenState(a_ref) != RE::BGSOpenCloseForm::OPEN_STATE::kNone;   // it has an open / close state
}

std::vector<float> Result(std::int32_t a_handle)
{
    std::vector<float> out;
    std::scoped_lock   lk(g_mtx);
    const auto         it = g_jobs.find(a_handle);
    if (it == g_jobs.end()) return { -2.f, 0.f };     // (the review) an unknown / dropped plan, not "still planning"
    if (it->second->status == kRunning) return out;
    const Job& j = *it->second;
    if (j.ex && !j.legacyOnly) {                       // 1.4.0 an Ex plan, in the old stride-3 shape (status 3 = partial)
        out.push_back(static_cast<float>(j.status));
        const bool pts = j.status == 2 || j.status == 3;
        out.push_back(pts ? static_cast<float>(j.route.size()) : 0.f);
        if (pts) {
            for (const auto& p : j.route) {
                out.push_back(p.pos.x);
                out.push_back(p.pos.y);
                out.push_back(p.pos.z);
            }
        }
        return out;
    }
    out.push_back(static_cast<float>(j.status));
    out.push_back(j.status == 2 ? static_cast<float>(j.out.size()) : 0.f);
    if (j.status == 2) {
        for (const auto& p : j.out) {
            out.push_back(p.x);
            out.push_back(p.y);
            out.push_back(p.z);
        }
    }
    return out;
}

void Cancel(std::int32_t a_handle)
{
    std::scoped_lock lk(g_mtx);
    g_jobs.erase(a_handle);
}

void ClearAll()
{
    {
        std::scoped_lock lk(g_mtx);
        g_jobs.clear();
        g_bad.clear();         // 1.4.0: the bad-spot memory belongs to the old game too
        g_cache.Clear();       // 1.5.1: the cell cache too (a load: its memory is freed) ...
        g_cacheOld.Clear();    // ... a previous world's still being freed ...
        g_cone.clear();        // ... and the walkers' remembered hull cones
    }
    standpoint_nav::ClearAll();    // 1.6.0: NavGraph's jobs and its 60 s memories (its own lock, never under the planner's)
}

bool RegisterNatives(RE::BSScript::IVirtualMachine* a_vm)
{
    a_vm->RegisterFunction("PathStart", "Standpoint", PathStartNative);
    a_vm->RegisterFunction("PathResult", "Standpoint", PathResultNative);
    a_vm->RegisterFunction("PathCancel", "Standpoint", PathCancelNative);
    logger::info("route planner natives registered (class Standpoint, 3 functions)");
    return true;
}

}  // namespace standpoint_path
