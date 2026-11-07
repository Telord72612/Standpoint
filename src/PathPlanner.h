#pragma once
#include "NavGraph.h"     // v1.6.0: the skeleton's point type (PlanOptions::skeleton)
// ─────────────────────────────────────────────────────────────────────────────
//  PathPlanner.h — v1.3.0: an obstacle-aware route for the LAST stretch of a walk (the user, 2026-10-30: "If we have a
//  route and create waypoint on that route, we can travel the NPC on that route "manualy" ... with check to confirm they
//  arrive"). The engine still walks every leg (travel package, navmesh); this finds WHERE the legs go when something the
//  navmesh does not know about stands in the straight line (furniture, a house mod's walls).
//
//  Floor = the navmesh (a cell is walkable only where the navmesh is, so the engine can walk each leg). Obstacles = Havok
//  rays at knee and chest height, two of them her radius to each side (her whole body must fit). Lazy weighted A* on a
//  32 u grid, then the cell path is pulled straight into a few waypoints at least 64 u apart (nearer, the engine poses
//  her instead of walking - the 0.9.8 measurement).
//  COST: the work runs in SLICES on the main thread (an SKSE task per frame, at most kSliceQueries physics questions per
//  slice), so a plan never spikes one frame; each plan logs its query count and time.
//  v1.4.0: + the Ex mode (StartEx / GetRoute / MarkBad / ProbeSegment / ProbeStandSpot, below) - the 1.3.0 functions
//  above keep their exact behaviour.
// ─────────────────────────────────────────────────────────────────────────────

namespace standpoint_path {

    // Start planning for a_walker from where she stands to a_goal (game units). a_avoid (may be null): a reference whose
    // body the way round keeps clear of (the walk's own target). A handle >= 1, or 0 = refused.
    std::int32_t Start(RE::Actor* a_walker, const RE::NiPoint3& a_goal, RE::TESObjectREFR* a_avoid);

    // [] while planning. Done: [status, n, x1, y1, z1, ..., xn, yn, zn]
    //   status 1 = the straight line is clear (n = 0)   2 = a route: n waypoints, the goal last
    //          0 = no route found (blocked, too far, out of budget, no navmesh at the goal, too many turns)
    //         -1 = failed (walker gone, no physics world)   -2 = no such plan (unknown handle, dropped, a load)
    std::vector<float> Result(std::int32_t a_handle);

    void Cancel(std::int32_t a_handle);   // frees a 1.3.0 plan and a 1.4.0 Ex plan alike (one job table)
    void ClearAll();     // a game load: every plan is dropped (1.4.0: and every walker's bad-spot memory)

    bool RegisterNatives(RE::BSScript::IVirtualMachine* a_vm);

    // ─────────────────────────────────────────────────────────────────────────────────────────────────────────────────
    //  1.4.0 THE EX MODE (C++ only in 1.4.0 - no Papyrus natives): a route over the navmesh, bare ground ("the physics
    //  floor") and shallow water, for the direct-control walk driver. Design: Report/StandPoint Module/1.4.0 Water Walk/
    //  B_Hybrid_Planner_Design.md §2-§8.3 (+ D's decisions). Same job table, pump thread and per-frame budget as Start.
    //
    //  Flags: kFloor   cells off the navmesh may stand on the physics floor (one down ray)
    //         kWater   wet cells (depth <= depthFrac x her height) are allowed; a goal in the water is legal
    //         kPartial no way to the goal: a route to the nearest standable point reached (status 3)
    //         kAllDirect  the switch index is 0 (no engine prefix)
    //         kGoalMayMove  a goal that is not a place to stand moves toward her (16 u steps, up to 320 u)
    //         kNavFirst  start AND goal on the dry navmesh: the 1.3.0 search first (a dry, all-navmesh answer wins).
    //                  IGNORED with kAllDirect (review G): that search's legs are tested for the ENGINE, not for a
    //                  straight direct walk; the hybrid search prefers navmesh cells anyway.
    //         kForce   accepted, no effect in 1.4.0 (the repeat guard G3 is not built: no unasked cooldowns)
    //         kVerbose more LEDGE lines (16) and the placed waters listed in the log
    //  flags without kFloor and kWater = the 1.3.0 navmesh-only search exactly, its answer as route points.
    //
    //  status: 2 a route · 1 the straight line (one point: the goal) · 3 partial · 0 none (why says) · -1 failed
    //  pts: where her feet go, the start NOT included, the end last; z = the floor (the navmesh, or the bed in water).
    //  switchIndex k: points 0..k-1 are reached by engine-walkable legs, k.. by direct control (0 = all direct,
    //  n = all engine). 1.4.0 walks all of it directly (D §1.1); k is filled for a later hybrid.
    //  ! The default PlanOptions (B's) has no kAllDirect: then a navmesh PREFIX is pulled with 1.3.0's ENGINE-leg rules
    //    (navmesh at piece ends, up to 78 u of height a 64 u piece). A caller that walks every point directly (the 1.4.0
    //    walk) MUST add kAllDirect - then every leg is tested with the physics rules.
    //  v1.4.1 A START IN DEEP WATER (the user's VR run 2026-11-03: pulled into a bath 80 u deep, cap 54 u -> "no route"):
    //    she may START deeper than the cap - she is already there. The START REGION = the water deeper than the cap that
    //    joins her spot, cell to cell; there the limit is max(cap, her depth + 8 u), strictly under the engine's swim level
    //    (submerged 0.70 = (water - root z) / ((boundMax.z - boundMin.z) x scale)). A route never re-enters the start region
    //    once it has left it; a goal (or a partial route's end) inside it is legal up to the same limit. Every other rule
    //    is unchanged. Already at the swim level: status 0 with atSwimLevel (the walk answers O 21).
    //
    //  v1.5.0 (Report/StandPoint Module/1.5.0 Plan/D_Standpoint_150_Build_Spec.md §2) - four new flags. ANY of them switches
    //  the plan to the 1.5.0 BODY RULES (B_Body_Limits_And_Raycast_Routes.md §7): one rise / drop <= 26 u (x scale for s > 1)
    //  between floor samples <= ~11 u apart; cell to cell <= 26 + L u; clearance her collisionBound.extents.x + 4 for every
    //  side ray + two side-column rays (table tops); headroom to her controller bound's top + 2; the C12 layer lists; a step that fails
    //  only beside her retried 8 / 16 u aside; up to 4 floors per map square; 3D end tests (never "clear" on the floor above
    //  or under the end); a box margin max(448, 0.75 d) <= 1024, a second wider pass, ~10 questions per box cell (<= 20,000).
    //  Without any of the four flags StartEx is exactly 1.4.1. The probes below always use the 1.5.0 rules (+ kDoors).
    //  (v1.5.1 - the riser from her hull, the terrain allowance, the cell cache, the 2400 u range: see the end of this file.)
    //         kNavGuide  start and goal on the navmesh: the navmesh's corridor first, every leg tested with the physics rules,
    //                    a failed leg repaired by a local ray search; no navmesh / no corridor -> the full ray search
    //         kWadeSpot  `goal` = Water Finder's seed: ONE search from her toward it that ends at the first reachable water
    //                    cell of the right depth (wadeMinFrac x H .. the cap) whose 8 neighbours are mostly water too
    //                    (>= wadePatchMin of 8) and which passes the stand-spot test; none -> status 0 + wadeNone
    //         kDoors     an unlocked, closed, ordinary (not load) door in the way is passed: the point after it gets kPtDoor
    //                    and RoutePoint::door (the driver opens it before that leg); locked / load / open doors are walls
    //         kActorsKeepOut  actors standing still at plan start are kept out (discs of their radius + her clearance + 8)
    //  Threads: as 1.4.x - every ray in an SKSE task (main-thread slice) under the world read lock; never from a VM thread.
    // ─────────────────────────────────────────────────────────────────────────────────────────────────────────────────
    enum Flag : std::uint32_t { kFloor = 1, kWater = 2, kPartial = 4, kAllDirect = 8, kGoalMayMove = 16,
                                kNavFirst = 32, kForce = 64, kVerbose = 128,
                                kNavGuide = 256, kWadeSpot = 512, kDoors = 1024, kActorsKeepOut = 2048 };
    enum class Kind : std::uint8_t { kNav = 1, kFloor = 2, kWater = 3 };
    // (v1.5.0: + door - the door the leg INTO this point passes (kPtDoor), 0 = none)
    struct RoutePoint { RE::NiPoint3 pos; Kind kind; float depth; std::uint32_t flags; RE::FormID door = 0; };
    // (v1.5.0: + wadeMinFrac / wadePatchMin - the kWadeSpot depth floor (x her height) and the patch minimum (of 8))
    // (v1.6.0, appended - see the v1.6.0 block at the end of this file: the skeleton input, CHUNK MODE, the start override, the
    //  wade-depth bands, the caller's tag)
    struct PlanOptions { std::uint32_t flags = kFloor | kWater | kPartial | kNavFirst; float depthFrac = 0.45f;
                         float wadeMinFrac = 0.15f; int wadePatchMin = 5;
                         std::vector<standpoint_nav::SkelPoint> skeleton;   // NavGraph's points (start / `from` NOT included, the end last)
                         bool          useFrom = false;     // the plan starts at `from` instead of her position (a later chunk) ...
                         RE::NiPoint3  from{};              // ... = the last point of the route she already walks
                         float         chunkMinU = 0.f;     // > 0 (with a skeleton) = CHUNK MODE: only the skeleton's first chunk ...
                         float         chunkMaxU = 0.f;     // ... is verified (see below)
                         float         wadeKneeFrac = 0.28f;      // the wade-depth bands (x her height): knee (scan 1's least) ...
                         float         wadeThighFrac = 0.38f;     // ... thigh reached (the thigh MAX = depthFrac)
                         float         wadeScanBodies = 2.f;      // each scan reaches this many body lengths (x H) round its spot; 0 = no scans
                         bool          wadeFlood = true;          // the floor flood before the wade search (her start dry)
                         std::uint32_t tag = 0; };          // the caller's id (WALK / STEER #n) - on the PATH asked line
    // (v1.4.1: + atSwimLevel - status 0 because she stands at the swim level or swims; the walk answers O 21 for it)
    // (v1.5.0: + wadeNone - a kWadeSpot plan found no reachable wading spot; wadeDepth / wadePatch - the chosen spot's water
    //  depth (u) and how many of its 8 neighbours were wading water)
    // (v1.5.1, appended: riser - 1.5.0's riser this plan used on EVERY floor (u; 0 = an old-flag plan); terrainSteps - how many
    //  cells / steps / line samples only the terrain allowance let pass; cacheHits - answers the cell cache gave this plan;
    //  riserTerrain - the riser from her hull cone, used only between land floors on the navmesh (log / diagnostics only))
    // (v1.6.0, appended - see the v1.6.0 block at the end of this file)
    struct PlanInfo { int status; int switchIndex; float goalMovedBy, depthCap, height, length; int questions; std::string why;
                      bool atSwimLevel; bool wadeNone = false; float wadeDepth = 0.f; int wadePatch = 0;
                      float riser = 0.f; int terrainSteps = 0; int cacheHits = 0; float riserTerrain = 0.f;
                      bool  outsideLoaded = false;   // status 0: the goal lies outside the loaded cells - the walk answers O 26
                      bool  chunked = false;         // a CHUNK MODE answer: one chunk of the skeleton ...
                      bool  chunkFinal = false;      // ... the chunk that reaches the goal (else its last point is kPtChunkEnd)
                      int   skelTaken = 0;           // ... skeleton points it covered: the next chunk gets the skeleton after them
                      float remainAfterU = 0.f;      // ... the skeleton's length from this chunk's end to the goal (u)
                      int   skelSource = 0;          // the corridor: 0 none, 1 given (PlanOptions::skeleton), 2 built here (NavGraph),
                                                     // 3 the 1.5.x grid corridor (NavGraph's self-check failed / no navmesh read)
                      float skelLength = 0.f;        // the skeleton's length (u) when one was used
                      int   stairSteps = 0;          // cells / steps / line samples only the stair allowance let pass
                      int   noCutPoints = 0;         // route points that got kPtNoCut
                      int   wadeStage = 0;           // a wade plan's last stage: 0 the entry, 1 scan 1 chose a deeper spot, 2 scan 2 did
                      float wadeEntryDepth = 0.f; }; // the entry spot's depth (u) - wadeDepth is the final spot's

    // RoutePoint::flags bits (B §6.1) - names only, the values are B's
    enum PointFlag : std::uint32_t {
        kPtEngineLeg  = 1,    // the leg INTO this point is engine-walkable (dry navmesh both ends, >= 64 u, before the switch)
        kPtSharp      = 2,    // a sharp corner: the chord round it failed - reach the point, slow, then turn
        kPtEdge       = 4,    // on an edge cell: a drop / a hole within one cell
        kPtGoal       = 8,    // the goal
        kPtGoalMoved  = 16,   // the goal was moved (GOAL_MAY_MOVE)
        kPtOnNavmesh  = 32,   // on the navmesh
        kPtDoor       = 64,   // v1.5.0: the leg INTO this point passes the closed door RoutePoint::door (open it first)
        kPtWade       = 128,  // v1.5.0: the wading spot a kWadeSpot plan chose (the end)
        kPtNoCut      = 256,  // v1.6.0: the chord +-128 u round this corner failed (the +-48 u one passed): not cut at speed -
                              //         the driver's look-ahead stays 48 u and she is at jog or slower from the lead before it
        kPtStair      = 512,  // v1.6.0: the leg INTO this point uses the STAIR ALLOWANCE (a navmesh-backed fixed floor, n.z 0.66-0.70):
                              //         walk it (and never stop on it - no stand spot is ever on a stair)
        kPtChunkEnd   = 1024, // v1.6.0: the last point of a CHUNK that is not the goal: a pass-through point (never an arrival) -
                              //         the next chunk is appended from here
        kPtVirtual    = 2048  // v1.6.0: the leg INTO this point follows a VIRTUAL LINK between two navmesh islands (its leg test passed)
    };

    // A handle >= 1, or 0 = refused (as Start). Any thread (reads her position / controller only, as Start).
    std::int32_t StartEx(RE::Actor* walker, const RE::NiPoint3& goal, RE::TESObjectREFR* avoid, const PlanOptions& o);
    // false = still planning. An UNKNOWN handle (never made, dropped by a load / Cancel / the 16-plan limit) answers true
    // with info.status = -2 (1.3.0's meaning) and no points. Copies under the planner's lock. Also answers a 1.3.0 Start handle.
    bool GetRoute(std::int32_t handle, PlanInfo& info, std::vector<RoutePoint>& pts);
    // The bad-spot memory: a spot she got stuck at; cells within her radius + 24 u of it (|dz| <= 64) are refused for
    // `seconds` (<= 0 = 60; clamped 5-300). Up to 32 per walker. Any thread. Only a spot UNDER her (within 16 u of a new
    // plan's start) leaves her a way out (radius + 48 u round her start); mark a stall AHEAD of her (>= 16 u - the driver
    // uses 32 u), never at her feet, or the re-plan cannot avoid it.
    void MarkBad(RE::FormID walker, const RE::NiPoint3& spot, float seconds);
    // v1.6.0 the stand-spot slope rule of EVERY 1.5.0-flag plan and of the probes (D §1 items 4 / 9): a stand spot needs n.z >= 0.85
    // (<= 32 deg) - or >= 0.75 (<= 41 deg) on terrain / ground (layers 13 / 17) with the navmesh under it. Stairs (the stair
    // allowance, 0.66) are never a stand spot.
    inline constexpr float kStandNz        = 0.85f;
    inline constexpr float kStandNzTerrain = 0.75f;
    inline constexpr float kStairNz        = 0.66f;   // the stair allowance: a fixed floor (static / transparent / stair helper)
                                                      // with the navmesh under it may be walked up to this slope (48.7 deg)

    // v1.6.0 what a probe found (the caller's own log line, with quiet = true)
    struct ProbeNote {
        std::string why;               // empty = passed; else the refusal ("too steep to stand on ...", "an obstacle in the way" ...)
        float       failAt{ 0.f };     // ProbeSegment: how far along a -> b it failed (u)
        float       nz{ 1.f };         // ProbeStandSpot: the floor's slope (normal z)
        bool        terrainNav{ false };   // ProbeStandSpot: the floor is terrain / ground with the navmesh under it (the 0.75 rule)
        int         questions{ 0 };
        float       ms{ 0.f };
        // v1.6.1 ProbeStandSpot: a refusal by one of the 4 SIDE floor rays at her clearance rc ("uneven ground or a brink beside the
        // spot", "deep water beside the spot", "no loaded cell beside the spot") - which side and what it measured
        int           side{ -1 };      // 0 N (+y), 1 E (+x), 2 S (-y), 3 W (-x); -1 = not a side refusal
        int           sideWhat{ 0 };   // 0 a floor at sideZ more than `window` off centreZ, 1 blocked (a tree / an invisible wall),
                                       // 2 no floor in its window, 3 deep water there (sideDepth), 4 no loaded cell there
        float         centreZ{ 0.f };  // the spot's own floor z (every refusal after the centre floor was found)
        float         sideZ{ 0.f };    // the side ray's floor (or blocking hit) z
        float         sideDepth{ 0.f };
        float         window{ 0.f };   // the side floor had to lie within +- this of centreZ (her riser)
        float         rc{ 0.f };       // how far out the side rays were (her clearance)
        std::uint32_t sideLayer{ 0 };  // the collision layer the side ray met (1 static, 13 terrain ...)
        RE::FormID    sideRef{ 0 };    // the reference it met (0 = none / not known)
        int           ignored{ 0 };    // hits on a_ignoreRef stepped through (the target's own body)
    };

    // MAIN THREAD ONLY (rays under the world lock; Papyrus natives run on VM threads - never call these from one).
    // Each call logs one line (v1.6.0: unless a_quiet - then a_note carries the answer for the caller's own line).
    // (v1.5.0: both use the 1.5.0 body rules - the walk's own - and ProbeSegment passes an unlocked closed ordinary door.)
    // (v1.6.0: both share the plans' 1.6.0 rules - the stair allowance on the line; the stand spot's slope rule above:
    //  a_terrainStandNz = the least n.z on navmesh-backed terrain (kStandNzTerrain = the plans' own; pass kStandNz for 0.85
    //  everywhere, the 1.5.x rule) - so a spot the probe accepts is a goal StartEx accepts too.)
    bool ProbeSegment(RE::Actor* walker, const RE::NiPoint3& a, const RE::NiPoint3& b, float depthFrac, bool a_quiet = false,
                      ProbeNote* a_note = nullptr);   // the driver's re-check before a leg
    // v1.6.1 (R3 Q1): a_ignoreRef = a reference whose OWN collision the 4 side floor rays step through (a furniture marker's spot
    // beside its chair: the side ray toward the chair landed on its seat ~35 u up -> "a brink"). 0 = none (as before). The centre
    // floor ray, the headroom ray and every plan keep their rules.
    bool ProbeStandSpot(RE::Actor* walker, const RE::NiPoint3& p, float depthFrac, float& floorZ, float& depth, bool a_quiet = false,
                        ProbeNote* a_note = nullptr, float a_terrainStandNz = kStandNzTerrain, RE::FormID a_ignoreRef = 0);

    // v1.5.0 ONE door rule for the planner AND the driver (review R2 S1 + S3): may Standpoint open this door for her?
    // true only when ALL hold: the reference is not deleted and not disabled; the base form is a Door (TESObjectDOOR, a
    // BGSOpenCloseForm) AND it has an open / close state
    // (BGSOpenCloseForm::GetOpenState != kNone); it is NOT locked - the lock FLAG: GetLock() non-null and REFR_LOCK::IsLocked()
    // (never TESObjectREFR::IsLocked(), which goes through the lock LEVEL); it is NOT a load door (no ExtraTeleport); it has NO
    // activation links (no ExtraActivateRef / ExtraActivateRefChildren: a trapped or scripted door fires its trap / chain when
    // activated). Whether it is open NOW is not part of it - check that live. The planner passes a door only when this is
    // true AND the door is closed / closing at plan time; any other door is a wall. MAIN THREAD (reads the reference's extra
    // data, its lock and its open state). nullptr -> false.
    bool IsOpenableDoor(RE::TESObjectREFR* ref);

    // ─────────────────────────────────────────────────────────────────────────────────────────────────────────────────
    //  v1.5.1 (Report/StandPoint Module/1.5.1 Plan/D_Standpoint_151_Build_Spec.md §4) - for a plan with a 1.5.0 flag only
    //  (every walk); 1.3.0 Start and old-flag StartEx plans are unchanged:
    //   * TWO risers (the session's decision on review fix #2): 1.5.0's 26 u x scale (>= 24) on EVERY floor - STATIC, furniture,
    //     clutter, off the navmesh: a 30 u bench is never a step (PlanInfo::riser) - and her body's clamp(0.9 x her hull CONE, 24,
    //     40) u (Sofia: 34) only between land floors on the navmesh (PlanInfo::riserTerrain); the probes use the same two
    //   * TERRAIN on the navmesh: a step whose floor samples are terrain / ground (layers 13 / 17) with the navmesh under them may
    //     be as steep as the engine walks there - slope n.z >= 0.45, rise <= horizontal x tan 63 deg + 8 u (STATIC / furniture /
    //     clutter floors keep the riser rules); a floor beside her past the terrain riser only when the floor halfway out confirms
    //     a hillside (never the foot of a sheer drop). The probes apply it too (ProbeSegment asks the navmesh where it is needed).
    //   * the CELL CACHE (60 s; cleared when a plan runs in another physics world - another interior, going in or out - and by
    //     ClearAll; exterior answers only over loaded cells): the search's down rays, the navmesh and water over each floor, the
    //     cells' own tests and the step tests are reused by the next plans in one place (stpf, BiSR); doors, actors and anything
    //     that moves are never kept
    //   * the RANGE (straight 2D distance her -> the goal): kRangeV15; further = status 0 "too far for a local route (the
    //     engine's own pathing walks this)". 1.3.0 Start and old-flag StartEx keep 1600 u.
    // ─────────────────────────────────────────────────────────────────────────────────────────────────────────────────
    inline constexpr float kRangeV15 = 2400.f;

    // ─────────────────────────────────────────────────────────────────────────────────────────────────────────────────
    //  v1.6.0 (Report/StandPoint Module/1.6.0 Plan/D_Standpoint_160_Build_Spec.md §1 items 6-10; design S2 §2-§6) - for a plan with a
    //  1.5.0 flag (every walk) and the probes; 1.3.0 Start and old-flag StartEx plans are unchanged:
    //   * THE SKELETON (NavGraph.h): with kNavGuide, when the straight line fails, the corridor is NavGraph's triangle path (her start
    //     and the goal may lie up to 256 u off the navmesh: the joining legs are tested / repaired like any other) instead of the
    //     32 u grid corridor; NavGraph's self-check failing (or no navmesh read) -> the 1.5.x grid corridor (PlanInfo::skelSource 3).
    //     PlanOptions::skeleton GIVEN: it is the corridor as it is (no search; kNavGuide implied).
    //   * CHUNK MODE (PlanOptions::chunkMinU / chunkMaxU > 0, a skeleton given, the plan starts at `from` when useFrom): the chunk
    //     ends at the first skeleton point at least chunkMinU along the skeleton from the start - then on to the last point still
    //     within chunkMaxU along; when the goal (the skeleton's last point) lies within that, it is the last chunk (chunkFinal: the
    //     goal's own tests, as any plan). Use chunkMinU = chunkMaxU = kChunkFirstU for chunk 1 (she starts soon) and chunkMinU 0,
    //     chunkMaxU = kChunkNextU for the next ones. A chunk's end that is not the goal is a pass-through point (its floor, slope,
    //     water, headroom and clearance - no stand-spot test; moved <= 16 u aside, else the chunk ends at the last corridor point
    //     before it): kPtChunkEnd. No straight-line try in chunk mode. PlanInfo: chunked, chunkFinal, skelTaken (next chunk:
    //     skeleton.erase(begin, begin + skelTaken), from = the route's last point), remainAfterU.
    //   * the LOADED AREA: a goal outside the loaded cells (NavGraph's GoalLoaded) = status 0 at once with outsideLoaded (O 26).
    //   * SLICES limited by TIME: 1.0 ms a frame shared by all running plans (NavGraph's jobs keep 0.25 ms of it together), no
    //     96-question cap (a 2,000-question safety cap a slice); 1.3.0 Start keeps its 96. A water lookup counts 1/4 question. The
    //     second pass CONTINUES the first: in the wider box (the grid is made at the wider margin) after an edge contact, else once
    //     in the same box with a fresh budget when pass 1 ran out of budget; none when pass 1's open list ran empty in its box;
    //     side-steps (C11 nudges, a cell moved aside) are deferred until they are the open list's best; a MIX repair <= 2,000 q.
    //   * STAIRS: the stair allowance (kStairNz) on fixed floors with the navmesh under them - cells, steps, line samples, corridor
    //     points, spots aside (counted: PlanInfo::stairSteps; route points kPtStair); a goal that is not a place to stand moves by a
    //     RING search (16-160 u, 8 bearings, nearest first, on the goal's floor: |dz| <= riser + 8, + 0.5 x the radius on navmesh
    //     terrain; a goal refused only for the slope of a stair / static ramp: at the ASKED height; v1.6.1: her body must fit
    //     there - the cell clearance and the 4 side columns at rc) instead of the 16 u march; nothing on the ring -> the 1.5.1 march
    //     toward her.
    //   * CORNERS: a corner whose +-128 u arc turns 15-60 deg is chord-tested over +-128 u too; a failure = kPtNoCut.
    //   * WADING (kWadeSpot; the user's priority rule, all in ONE plan): the first reachable SHIN-deep wading spot E (wadeMinFrac,
    //     as before) -> scan 1 within wadeScanBodies x H of E: the nearest THIGH spot (>= wadeThighFrac x H) ends it, else the nearest
    //     KNEE spot K (>= wadeKneeFrac x H) -> scan 2 within wadeScanBodies x H of K for the nearest thigh spot; the cap (depthFrac,
    //     the thigh MAX) holds on every cell of the route; start -> E -> K -> the final spot is ONE route (no stop). A floor flood
    //     first aims the search at the nearest reachable shin-deep water (the seed's water surface more than a riser below her
    //     floor: a ledge; v1.6.1: directed toward the seed, 4,000 squares / 10,000 questions, and once it finds the water the
    //     search follows the flood's own way to it - its floor distance there is the search's heuristic). PlanInfo: wadeStage,
    //     wadeEntryDepth, wadeDepth (final).
    // ─────────────────────────────────────────────────────────────────────────────────────────────────────────────────
    inline constexpr float kChunkFirstU      = 1400.f;   // chunk 1: ends at the first skeleton point at least this far along
    inline constexpr float kChunkNextU       = 2400.f;   // the next chunks: at most this far along (= kRangeV15)
    inline constexpr float kWadeShinFrac     = 0.15f;    // the wade-depth bands x her height H: shin = the entry ...
    inline constexpr float kWadeKneeFrac     = 0.28f;    // ... knee (scan 1's least) ...
    inline constexpr float kWadeThighFrac    = 0.38f;    // ... thigh reached ...
    inline constexpr float kWadeThighMaxFrac = 0.45f;    // ... thigh MAX = the route's cap (the API's default depthFrac)

}  // namespace standpoint_path
