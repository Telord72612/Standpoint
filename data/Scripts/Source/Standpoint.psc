Scriptname Standpoint Native Hidden
{ Standpoint SKSE plugin natives (v1.0.0). SHIPS with the mod. }

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

; Arm the rule for these two. Returns a handle (>= 1) to give back to PairRelease, or 0 = not armed: no HIGGS (flat
; edition), a controller that could not be read, both controllers in one collision group, or the pool is full.
; A pair that is armed already (or whose release is still pending) keeps its entry: the pending release is cancelled,
; the use is counted, the rule never drops.
; It only bites for a pair whose bodies start to overlap AFTER it is armed.
Int Function PairArm(Actor akA, Actor akB) Global Native

; Give a handle back: one use less. At 0 the pair is released "when apart": the DLL switches the rule off once the two
; are 90 u or more apart, or one of them is gone - never while their bodies still overlap. Harmless with a handle
; that names no live pair.
Function PairRelease(Int aiHandle) Global Native

; Tier 3 reconnaissance: log the engine's avoid-node machinery for this actor.
; READ-ONLY - every dereference is page-checked and objects are found by vtable
; identity, never by struct offset. Diagnostic only; safe to omit.
Function ProbeAvoidance(Actor akActor) Global Native
