Scriptname Standpoint Native Hidden
{ Standpoint SKSE plugin natives (v2.0.0). SHIPS with the mod. }

; Write a line to SKSE/Standpoint.log (pure Papyrus cannot reach that file).
Function Log(String asMsg) Global Native

; Read a live engine setting (GMST collection first, then INI, then INIPref).
; INI-class names must include their section, e.g. "fAITurnSpeedScale:Pathfinding".
; Returns -999999.0 when the name resolves nowhere.
Float Function GetEngineSettingFloat(String asName) Global Native

; Runtime-only write to a live engine setting (same lookup). Nothing is saved.
Bool Function SetEngineSettingFloat(String asName, Float afValue) Global Native

; The engine's live cursor into akActor's current package procedure tree.
; MEASURED S1 (2026-08-15): the runtime index counts PROCEDURES, not tree nodes.
; For Standpoint_TravelAndPose: 0 = Travel running, 1 = UseIdleMarker running;
; ARRIVAL SIGNAL = the 0 -> 1 transition. -2 = no actor/AI process.
; A run-once package can shadow this — always pair with GetCurrentPackageFormID.
Int Function GetCurrentProcedureIndex(Actor akActor) Global Native

; FormID of the package the AI is currently executing (0 = none/no process).
Int Function GetCurrentPackageFormID(Actor akActor) Global Native

; ---- S2 (v0.3.0) -----------------------------------------------------------

; Solve a validated stand-slot + facing for a destination.
; aiRelation: 0=nearest 1=front 2=right 3=back 4=left (in the TARGET's facing frame)
; Returns [] on failure (reason in Standpoint.log), else
; [x, y, z, headingDEGREES, markerIndex(-1=geometric), quality 0..1].
Float[] Function SolveSlot(ObjectReference akTarget, Int aiRelation, Actor akAgent, Float afStandoff) Global Native

; Number of furniture (FRN) markers on the ref's loaded 3D. -1 = no 3D, 0 = none.
Int Function GetFurnitureMarkerCount(ObjectReference akTarget) Global Native

; Per-marker world data: [x, y, z, headingDEGREES, effectiveEntryPoints, animType, enabled].
; effectiveEntryPoints bits: 1=Front 2=Behind 4=Right 8=Left (after the full three-term gate).
Float[] Function GetFurnitureMarkerInfo(ObjectReference akTarget, Int aiIndex) Global Native

; Plugin version, e.g. "0.4.0". The soft-dep contract: call via StandpointAPI.GetVersion(),
; which returns "" when the mod is absent.
String Function GetPluginVersion() Global Native

; ---- v0.5.0 interrupt recognition (observation only) -----------------------

; True while either of the player's HIGGS hands is grabbing this actor (VR;
; always False on flat editions or without HIGGS).
Bool Function IsPlayerGrabbing(Actor akActor) Global Native

; The packType in the actor's run-once package slot, -1 if none. The engine's
; bump reaction = 32 (FollowerBumpGuard's in-VR measurement; recognition
; technique ported from FBG, observation-only - nothing is suppressed).
Int Function GetRunOncePackageTypeId(Actor akActor) Global Native

; ---- v0.6.0 danger recognition (observation only) --------------------------

; Scan the CORRIDOR from akWalker to (destX,destY,destZ) for hostiles: a threat
; qualifies when it lies within afCorridor of the route line and no further than
; afLookAhead along it (anything within 320 units of the walker qualifies in any
; direction). Returns [] with no walker, else 10 floats:
;   [0] count   - the gating union (declared | viaPlayer)
;   [1] nearest - distance walker -> nearest unioned threat, -1 if none
;   [2][3][4]   - centroid of the unioned threats: the danger blob
;   [5] spread  - blob radius (this is what a Tier-3 avoid node would be sized to)
;   [6] declared      - engine says hostile to the WALKER
;   [7] viaPlayer     - engine says hostile to the PLAYER (fires earlier)
;   [8] inCombat      - actively fighting (unambiguous but late)
;   [9] aggro         - AIDATA aggression >= Aggressive (predictive: bandits,
;                       wildlife and draugr read hostile BEFORE they notice anyone.
;                       MEASURED: wolves are Aggressive(1), NOT VeryAggressive(2))
; 6..9 are reported separately on purpose - which one actually catches a sleeping
; bandit camp is a field question, not a theory question.
Float[] Function ScanThreats(Actor akWalker, Float afDestX, Float afDestY, Float afDestZ, Float afCorridor, Float afLookAhead) Global Native

; ---- v0.8.0 the move API (C++ interface revision 2, API/StandpointAPI.h) -----
; The DLL's mailbox: a C++ plugin hands in a 10-slot request line; the driver reads it with these, walks her with its
; own session engine and names the ending. DRIVER USE ONLY - a Papyrus consumer calls StandpointAPI.GoTo / GoToPoint.

; The NPC / the target of the open request aiId (None = no such request, or a release line has no target).
Actor Function ApiReqWho(Int aiId) Global Native
ObjectReference Function ApiReqWhere(Int aiId) Global Native

; [distanceCm, facingHour, orientationHour, pose, holdSeconds, isRelease(0/1), close(0/1)]; [] = no such request.
; (the 7th value since v0.9.0)
Float[] Function ApiReqValues(Int aiId) Global Native

; Close the request: asLetter = C completed / P player / O obstacle / D danger / L leash / X released / B refused;
; aiDetail = the reason code (StandpointAPI.h MoveDetail). The DLL measures the result line and calls the plugin back.
; Answering a request twice is harmless (the second is ignored).
Function ApiPostResult(Int aiId, String asLetter, Int aiDetail) Global Native

; The clock solve. afFacingHour: where around the target, in the TARGET's facing (0 / 12 front, 3 its right, 6 behind,
; 9 its left; < 0 = the side nearest akAgent). afDistCm: centre of the target to the stand-point. afOrientHour: her
; own facing once there (0 / 12 = at the target, 6 = her back to it). A target without loaded 3D (a marker) works.
; Returns [] on failure (reason in Standpoint.log), else
; [x, y, z, headingDEGREES, orientOffsetDEGREES, gotHour, quality 0..1].
; aiClose (v0.9.0): 1 = close contact - an ACTOR target has no distance floor (the point is exactly as far as asked).
Float[] Function SolveClock(ObjectReference akTarget, Actor akAgent, Float afDistCm, Float afFacingHour, Float afOrientHour, Int aiClose = 0) Global Native

; ---- v0.9.4 the pair rule (part of close mode) - DRIVER USE ONLY ---------------
; "These two NPCs' character controllers ignore each other": their bodies can overlap; walls, furniture and every
; other actor collide as always. The DLL keeps a pool of 8 pairs and counts who uses each.
; (0.9.3, never run in VR, had `Bool PairArm(Int aiSlot, ...)` / `PairRelease(Int aiSlot)`: a pair per session slot.)

; Arm the rule for these two. Returns a handle (>= 1) to give back to PairRelease, or 0 = not armed: a controller that
; could not be read, both controllers in one collision group, or the pool is full. (v1.2.0: without HIGGS - SE, AE -
; Standpoint hooks the game's collision filter itself; before 1.2.0 "no HIGGS" also meant 0.)
; A pair that is armed already (or whose release is still pending) keeps its entry: the pending release is cancelled,
; the use is counted, the rule never drops.
; It only bites for a pair whose bodies start to overlap AFTER it is armed.
Int Function PairArm(Actor akA, Actor akB) Global Native

; Give a handle back: one use less. At 0 the pair is released "when apart": the DLL switches the rule off once the two
; are 90 u or more apart, or one of them is gone - never while their bodies still overlap. Harmless with a handle
; that names no live pair.
Function PairRelease(Int aiHandle) Global Native

; ---- v1.2.0 the Papyrus door to the move request line - API PLUMBING: call StandpointAPI.RequestMove /
; GetMoveOutcome / GetMoveResult, not these ---------------------------------------------------------------------
; Open a request line exactly like the C++ IStandpoint2::RequestMove (same mailbox, same driver path, same measured
; answer). Returns the handle (>= 1), or 0 when akWho is None. The answer comes as the "Standpoint_MoveResult" event.
Int Function ApiMoveSubmit(Actor akWho, ObjectReference akWhere, Float afDistCm, Float afFacingHour, Float afOrientHour, Int aiPose, Float afHoldSeconds, Int aiClose) Global Native
; "" = no such handle (unknown, too old, or made before the last load); "-" = still open; else the answer's letter.
String Function ApiMoveOutcome(Int aiHandle) Global Native
; [distanceCm, facingHour, orientationHour, detail] once answered; [] while open or unknown.
Float[] Function ApiMoveResult(Int aiHandle) Global Native

; ---- v1.3.0 the route planner (PathPlanner.cpp) - DRIVER USE (the detour); safe for anyone, it never moves anyone ----
; Plan a walkable way for akWalker from where she stands to (afX, afY, afZ): the navmesh for the floor, Havok rays at
; knee and chest height plus two diagonal rays for what stands in the way. The work runs in slices, all plans together
; at most ~96 physics questions per frame. akAvoid (None allowed): a reference whose body the way keeps clear of (the
; walk's own target). Returns a handle (>= 1), or 0 = refused.
Int Function PathStart(Actor akWalker, Float afX, Float afY, Float afZ, ObjectReference akAvoid) Global Native
; [] while planning. Done: [status, n, x1, y1, z1, ...] - status 1 the straight line is clear (n 0), 2 a way round (n
; points, the goal last), 0 no way found (blocked, too far, out of budget, no navmesh at the goal, too many turns),
; -1 could not run, -2 no such plan (unknown, dropped, or a game load came in between).
Float[] Function PathResult(Int aiHandle) Global Native
; Drop a plan (read or not). Harmless with an unknown handle.
Function PathCancel(Int aiHandle) Global Native

; ---- v1.4.0 the walk off the navmesh (WalkApi.cpp + DirectWalk.cpp) - API PLUMBING: call StandpointAPI.WalkTo /
; GetWalkOutcome / GetWalkResult / CanWalkOffNavmesh, not these ------------------------------------------------------
; Plan a route for akWho (navmesh, bare ground, shallow water up to afDepthFrac x her height) and walk her along it by
; planner direct control: real steps, nothing slides her. Returns the handle (>= 1), or 0 when akWho is None. Her older
; walk / stand answers X 6. The answer comes ONCE as the "Standpoint_WalkResult" event (StandpointAPI.ApiWalkDone).
Int Function WalkStart(Actor akWho, Float afX, Float afY, Float afZ, Float afFacingDeg, Float afHoldSeconds, Float afDepthFrac, Float afStopSubmerged) Global Native
; Her walk / stand ends now: an open walk answers X aiDetail (3 released, 6 replaced); a stand after the answer just ends.
; True = she had one.
Bool Function WalkStop(Actor akWho, Int aiDetail) Global Native
; "" = no such handle (unknown, too old, or made before the last load); "-" = still open; else the answer's letter.
String Function WalkOutcome(Int aiHandle) Global Native
; [distLeft u, submergedLevel, detail, x, y, z] once answered; [] while open or unknown.
Float[] Function WalkResultValues(Int aiHandle) Global Native
; True on Skyrim VR with the driver's hook installed (the walk off the navmesh is available). (v1.5.0: also on SE / AE once
; the start-up self-test says READY and Data/SKSE/Plugins/Standpoint.ini has [DirectWalk] SEAE = 1.)
Bool Function WalkSupported() Global Native

; ---- v1.5.0 the walk into water (WalkApi.cpp) - API PLUMBING: call StandpointAPI.WalkToWater / CanWalkToWater, not these ----
; Water Finder (1.6.0+) finds water within afMaxMetres of akWho; the planner's wade-spot search ends at the first wading spot
; she can reach (water afDepthMinFrac..afDepthMaxFrac x her height, with water round it); she walks there and faces the
; water. Returns the handle (>= 1; the same numbers as WalkStart), or 0 when akWho is None. The answer comes ONCE as the
; "Standpoint_WalkResult" event (StandpointAPI.ApiWalkDone), and through WalkOutcome / WalkResultValues.
Int Function WalkToWaterStart(Actor akWho, Float afMaxMetres, Float afDepthMinFrac, Float afDepthMaxFrac, Float afHoldSeconds) Global Native
; True when WalkSupported() AND Water Finder 1.6.0+ answered Standpoint's C++ handshake (the water walk can run).
Bool Function WaterWalkSupported() Global Native

; ---- v1.6.0 THE STEERED SESSION (SteerSession.cpp, RelativeSolve.cpp) - DRIVER USE ONLY (Standpoint_PlayerAlias) -------------
; Skyrim VR with Data/SKSE/Plugins/Standpoint.ini [Session] Steered = 1 (the default on VR; 0 on SE / AE): a GoTo / GoToPoint /
; RequestMove session keeps its slot and its whole contract in the driver, but its legs are walked by Standpoint's own steering:
; the DLL solves the spot by physics, plans ONE route, walks her there and keeps a state line the driver polls.

; True = the driver's sessions walk steered (WalkSupported() and [Session] Steered = 1).
Bool Function SteerSupported() Global Native
; True = this request can be walked steered: akWho is a loaded NPC and akTarget (None = the point afX, afY, afZ) lies in her own
; interior / her worldspace and is loaded. False = keep the engine session (it walks through load doors). The reason is logged.
Bool Function SteerCheck(Actor akWho, ObjectReference akTarget, Float afX, Float afY, Float afZ) Global Native
; Start the steered session for slot aiSlot (0-3). akTarget: the relative target (None for a point). afAsk:
;   [0] kind 1 = RequestMove (the clock), 2 = GoTo (a relation), 3 = GoToPoint (a point)
;   [1] holdSeconds  [2] pose 0-3  [3] interrupts 0-2  [4] close 0 / 1
;   kind 1: [5] distanceCm  [6] facingHour (< 0 = her side)  [7] orientationHour
;   kind 2: [5] relation 0-4  [6] standoff (u, <= 0 = the default)
;   kind 3: [5] x  [6] y  [7] z  [8] heading (compass degrees)
; Returns the steer id (>= 1); 0 = refused (no walker, or SteerSupported() is False). Her older steered session (a retask) is
; replaced; her older walk is replaced too and keeps her held until the new one takes her.
Int Function SteerStart(Actor akWho, Int aiSlot, ObjectReference akTarget, Float[] afAsk) Global Native
; [] = no such steer (a load, never made). Else 23 values (19 until 1.6.1's fix-up; the driver reads [19..22] only when present):
;   [0] phase 0 solving, 1 planning, 2 walking, 3 arrived (held by the walk's stand, or let go: [16]), 4 ended, 5 handed off,
;       6 waiting (a grab / combat - [15]; she walks on when it is over)
;   [1] letter 0 none, 1 C, 2 P, 3 O, 4 D, 5 L, 6 X, 7 B, 8 F = no route: the driver walks ONE engine leg to the spot,
;       9 H = her stand after the arrival ended (a grab, combat, an animation): freed silently
;   [2] detail (MoveDetail)  [3] her flat distance to the spot (u)  [4] her facing error (deg)  [5..7] where she is
;   [8..10] the solved spot  [11] the final heading (compass deg)  [12] the solved clock hour  [13] the spot is on the navmesh
;   [14] the walk's handle  [15] the wait: 0 none, 1 a grab, 2 combat  [16] she was let go at the point (hold 0, pose 0)
;   [17] her facing offset (deg)  [18] the spot is known (1)
;   [19] 1 = F for a POINT off the navmesh: the ONE engine leg goes to its nearest navmesh point [20..22] (x, y, z) - the arrival is
;        measured to the asked point [8..10]
Float[] Function SteerState(Int aiSteer) Global Native
; The pin is in place (marker at her feet, alias, package on): her stand ends quietly (the walk hands her back and re-evaluates
; her package). No answer, no event.
Function SteerHandOff(Int aiSteer) Global Native
; The session ends: the DLL forgets it; its walk / stand ends quietly (she is handed back). Harmless with an unknown id.
Function SteerStop(Int aiSteer) Global Native

; ---- v1.6.0 BUG 2: the marker litter (MarkerSweep.cpp) - DRIVER USE ONLY (Setup, once per load) ------------------------------
; Counts the persistent Standpoint idle markers (0x802 / 0x80F, created ones) that are NOT in akKeep (the driver's own) - one
; "MARKERS:" log line. Only with Standpoint.ini [Cleanup] SweepMarkers = 1 they are disabled and marked deleted.
Function MarkerSweep(ObjectReference[] akKeep) Global Native

; ---- v1.6.0 the stpm console test (TestCmd.cpp) - TEST USE ONLY ---------------------------------------------------------------
; The driver's test ear hands every Standpoint_* event it hears to the DLL while stpm runs (one [TEST] MIX event line each).
Function TestEvent(String asEvent, Form akWho, Int aiHandle, String asOutcome, Int aiDetail, Float afA, Float afB, Float afC) Global Native

; ---- v1.5.1 the far phase of a walk (WalkApi.cpp) - DRIVER USE ONLY (Standpoint_PlayerAlias) -------------------------------------
; (1.6.0: the far phase is REMOVED from the walks - the DLL never asks for a far session any more; these four stay registered
; with neutral answers, so the driver's far code below stays bound and simply dormant.)
; A walk beyond 30 m is first run by Standpoint's own travel session (the engine's walking / running) until the direct walk takes
; over. Right before that session puts its package on her, the DLL sets the slot's package (akPackage) to RUN: aiWalk = the walk's
; handle, aiSlot = the session slot (0-3).
Function WalkFarPackage(Int aiWalk, Int aiSlot, Package akPackage) Global Native
; True while walk aiWalk still wants its far session (or the DLL has already asked for its end); False = the driver frees the
; session itself (the walk ended, a load, a lost message).
Bool Function WalkFarAlive(Int aiWalk) Global Native
; HOW to free walk aiWalk's far session now (aiAsked = what the DLL asked; the driver's own poll asks 0): 0 the let-go
; (IdleForceDefaultState + EvaluatePackage), 1 the override off only (a walk of hers goes on: she keeps moving), 2 the override
; off + EvaluatePackage, no IdleForceDefaultState (she is in a reaction / held by a hand, or a quiet end came late).
Int Function WalkFarEndMode(Int aiWalk, Actor akWho, Int aiAsked) Global Native
; A consumer's Release of a far session: 2 (override off + EvaluatePackage, no IdleForceDefaultState) when akWho is in a reaction
; (ragdoll, knocked, kill move, bleedout, dead, combat, furniture, mounted) or a HIGGS hand holds her; else 0 (the let-go).
Int Function WalkReleaseMode(Actor akWho) Global Native

; Tier 3 reconnaissance: log the engine's avoid-node machinery for this actor.
; READ-ONLY - every dereference is page-checked and objects are found by vtable
; identity, never by struct offset. Diagnostic only; safe to omit.
Function ProbeAvoidance(Actor akActor) Global Native
