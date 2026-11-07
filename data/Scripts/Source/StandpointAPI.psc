Scriptname StandpointAPI Hidden
{ ============================================================================
  Standpoint - public Papyrus API (v2.0.0). SHIPS with the mod.

  "Walk this NPC to a precise stand-slot at a target and have them arrive
  FACING the right way, then hold there." Slots come from real furniture
  markers (full legality gate + live occupancy) or from the target's bounds;
  every slot is navmesh-snapped. Arrival facing is engine-driven (IdleMarker,
  ±1 degree). Since v0.9.2 NOTHING moves a stuck walker by force (SeverActions'
  stuck recovery is no longer started): she fails after 90 s, or with code 6.

  RELATIONS (in the TARGET's own facing frame):
    0 = nearest    1 = front    2 = right    3 = back    4 = left

  EVENTS (RegisterForModEvent on your own form):
    "Standpoint_Arrived"     handler(Form akWalker, Float afDist, Float afAngErr)
    "Standpoint_Failed"      handler(Form akWalker, Float afCode)
        code: 1 no-slot  2 timeout  3 released-before-arrival
              4 danger (dropped)  5 leash (strayed too far from the player)
              6 (v0.9.2) she stopped moving far from the point (100 u or more) - not an arrival
              7 (v0.9.9, a close move request to an NPC only - C++, or RequestMove since v1.2.0) NO WAY:
                she could not get to the asked point - the way is blocked and no other way worked; released
    "Standpoint_Interrupted" handler(Form akWalker, Float afCode)
        code: 1 combat  2 player bump/body-block  3 player hand-grab (VR)
              4 danger on or near the route
    "Standpoint_MoveResult"  (v1.2.0, the answer to RequestMove - ONE per handle, never after a game load)
        handler(Form akWho, Int aiHandle, String asOutcome, Int aiDetail, Float afDistCm,
                Float afFacingHour, Float afOrientHour, Form akWhere)
        asOutcome: C completed  P the player stopped her  O obstacle / no spot / no way
                   D danger  L leash  X released / replaced  B refused (busy, or not ready)
        aiDetail: the reason code (MoveDetail in API/StandpointAPI.h; 0 = on the point, 9 = stopped short
                  and near, 17 = no way). afDistCm / hours: MEASURED where she ended up.
        The walker's own events (Arrived / Failed / Interrupted) still fire for the same walk.
    "Standpoint_WalkResult"  (v1.4.0, the answer to WalkTo / v1.5.0 WalkToWater - ONE per handle, never after a load)
        handler(Form akWho, Int aiHandle, String asOutcome, Int aiDetail, Float afDistLeft, Float afSubmerged)
        asOutcome / aiDetail: the table in the ";" comment below this block.
    "Standpoint_DirectWalk"  (v1.4.0, for other mods: sender = her) strArg "start" when Standpoint takes her movement,
        "stop|<why>" when it gives it back. handler(String asEventName, String asWhat, Float afWalkHandle, Form akWho)

  SOFT-DEP CHECK:  StandpointAPI.GetVersion() returns "" when absent.
  Up to 4 concurrent walkers. A walker holds their slot (engine-pinned)
  until Release() or a new GoTo() retasks them.
  ============================================================================ }

; (v1.5.0: kept OUT of the doc block above - a "{...}" doc comment longer than ~4 KB makes Caprica fail with "Failed while
; allocating a Heap!")
; "Standpoint_WalkResult" asOutcome / aiDetail:
;   C 0 arrived (within 32 u), C 9 more than 32 u away (a partial route, the point was no place to stand and moved toward her,
;   or something in the way at the end), C 20 the asked depth reached; O 1 no route, O 2 not there in 90 s, O 10 not a loaded
;   NPC, O 15 pushed off the route (and no new route from there), O 18 blocked (no new route got her through), O 21 she would
;   swim; P 8 grabbed (VR); X 3 released, X 6 replaced, X 14 another end (combat, ragdoll, death, ... - Standpoint.log names
;   it); B 11 busy, B 19 not on this runtime (SE / AE without the switch), B 22 another mod drives her.
;   v1.5.0 WalkToWater only: O 23 no water within afMaxMetres, O 24 water but no wading spot she can reach, B 25 Water Finder
;   absent or older than 1.6.0.
;   v1.5.1: O 1 also "too far (beyond 200 m)"; O 2 for a walk beyond 30 m = not there in 90 s + (her distance / 150 u a second).
; afDistLeft: flat distance from her to the asked point (game units; WalkToWater: to the wading spot it chose);
; afSubmerged: her submerged level 0..1
;   v1.6.0: O 26 the point is outside the loaded area (answered at once - Standpoint walks only where the game has the ground
;   loaded; GoToPoint can travel there). The engine's far phase is gone: every WalkTo is walked by Standpoint's own path.

; v1.6.0 - THE STEERED SESSIONS (Skyrim VR; Data/SKSE/Plugins/Standpoint.ini [Session] Steered = 1, the default on VR - 0 on
; SE / AE keeps the engine's walk exactly as before, and switches the steered sessions off on VR too):
;   GoTo, GoToPoint and RequestMove keep their calls, their handles and their whole contract (IsBusy, GetArrivalState, the
;   Arrived / Failed / Interrupted events, Standpoint_MoveResult, hold -1 / 0 / N, the poses, Release, the leash, interrupts,
;   danger). What changed is HOW she walks: Standpoint finds the spot itself with physics tests (furniture, walls, other
;   NPCs, water at a safe depth), plans one route and walks her there with its own steering (as WalkTo) - no navmesh-only
;   spots, no stop-and-go between legs. After she arrives she is held where she stopped (the pose / hold as before).
;   Changes you can see: the answer comes sooner after she stops; O 17 (Failed 7) can also mean "no route" for a request that
;   is not close; IsDirectWalking is true and "Standpoint_DirectWalk" start / stop fire during a session's walk; a RequestMove
;   target that moves 32 u (or an NPC target that turns 30 deg) is followed (she re-aims while she walks); with interrupts 0 a
;   hand-grab (or with interrupts 0-1 combat) makes the session WAIT - she walks on afterwards (Interrupted 3 is reported).
;   When her route cannot be found, a request that is not close contact walks ONE engine leg to the spot (the old way); a
;   close request answers O 17 at once. A target in another interior / worldspace keeps the engine's walk (load doors).
; v1.6.1 (after the first 1.6.0 VR run): a GoToPoint with no route whose point is NOT on the navmesh walks its ONE engine leg to
;   the nearest navmesh point (within 128 u - a shore point on a rock or in the water: she ends at the shore edge) and arrives
;   SHORT, Arrived's distance measured to your point; Failed 1 at once only when there is no navmesh within 128 u of it (a point
;   outside the loaded area keeps its engine leg); an engine leg ends with Failed 7 (O 17) as soon as it is stuck - the engine
;   reports her movement blocked, or she has not moved for 6 s, 80 u or more from its point - instead of after 90 s; a GoTo on
;   furniture whose entries are all refused (no room to stand beside them) stands her at a side of it instead of Failed 1 (not
;   when an entry is TAKEN - she is never stood beside an occupied seat; every entry taken answers Failed 1, as before); a close RequestMove ends
;   on the asked distance (1.6.0 stopped about 5 u short of it); a steered GoToPoint that ends off the navmesh is held by the
;   walk's own stand (no marker).

String Function GetVersion() Global
    If Game.GetModByName("Standpoint.esp") == 255
        Return ""
    EndIf
    Return Standpoint.GetPluginVersion()
EndFunction

; Pure solve - never moves anyone. Returns [] on failure, else
; [x, y, z, headingDegrees, markerIndex(-1=geometric), quality 0..1].
Float[] Function Solve(ObjectReference akTarget, Int aiRelation = 0, Actor akAgent = None, Float afStandoff = 0.0) Global
    Return Standpoint.SolveSlot(akTarget, aiRelation, akAgent, afStandoff)
EndFunction

; Solve + walk + hold. Returns a session handle (>0) or 0 on failure.
; Re-calling for a walker who already has a session retasks them.
; afHoldSeconds: how long to hold the slot AFTER arrival before the walker is
; automatically released back to their normal AI. -1 (default) = hold until
; Release()/a new GoTo(). 0 = release immediately after the Arrived event.
; aiPose: interaction pose played ON ARRIVAL, aimed at the target object:
;   0 = none (default)   1 = auto by target height (kneel below ~40 u, bend above)
;   2 = kneel to floor (IdlePickup_Ground)   3 = standing bend (table pickup)
; afLeash: maximum walker-to-player distance in game units (default 3500 =
; ~50 m). Standpoint is a SHORT-RANGE framework: if the errand carries the
; walker further than this from the player, the session cancels (Failed code 5
; while walking; a silent release while holding) and the walker reverts to her
; normal AI - so no follower ever vanishes across the map because of a GoTo.
; Pass 0.0 to disable. Ignored when the player is the destination.
; aiDanger: what happens when hostiles are found on or near the route between
; the walker and the destination (a "Standpoint_Interrupted" code 4 fires either
; way). Standpoint sells ACCURACY, not guaranteed arrival - so the default is to
; give up cleanly and hand the decision back to YOU:
;   0 = ignore danger entirely
;   1 = report only, keep walking into it
;   2 = (default) DROP the session: package removed, walker reverts to her own
;       AI, "Standpoint_Failed" code 4. There is no pausing and no auto-resume -
;       destinations are short-range, so just call GoTo again when it suits you.
; A threat qualifies when it is within ~320 u of the walker in ANY direction
; ("upon her"), or inside the 600 u-wide corridor ahead of her along the route.
; Hostiles well off the route never drop a session.
; Detection covers only LOADED actors, which is the same horizon the engine
; itself has: an unloaded camp cannot fight anyone.
; aiInterrupts: what stops the travel early (the "Standpoint_Interrupted" event
; fires either way; codes: 1 combat, 2 player bump/body-block, 3 player hand-grab):
;   0 = report only, never auto-cancel
;   1 = (default) cancel when the PLAYER physically intercepts the walker
;       (VR hand-grab, bump-capsule contact, sustained body-block) - unless the
;       player is the destination
;   2 = as 1, plus cancel on combat (default combat behaviour: the engine
;       suspends travel, she fights, travel resumes after)
Int Function GoTo(Actor akWalker, ObjectReference akTarget, Int aiRelation = 0, Float afStandoff = 0.0, Float afHoldSeconds = -1.0, Int aiPose = 0, Int aiInterrupts = 1, Int aiDanger = 2, Float afLeash = 3500.0) Global
    Standpoint_PlayerAlias d = _Driver()
    If !d || !akWalker || !akTarget
        Return 0
    EndIf
    Return d.ApiGoTo(akWalker, akTarget, aiRelation, afStandoff, afHoldSeconds, aiPose, aiInterrupts, aiDanger, afLeash)
EndFunction

; Walk to explicit coordinates + facing (degrees, compass). Same contract.
Int Function GoToPoint(Actor akWalker, Float afX, Float afY, Float afZ, Float afHeadingDeg, Float afHoldSeconds = -1.0, Int aiInterrupts = 1, Int aiDanger = 2, Float afLeash = 3500.0) Global
    Standpoint_PlayerAlias d = _Driver()
    If !d || !akWalker
        Return 0
    EndIf
    Return d.ApiGoToPoint(akWalker, afX, afY, afZ, afHeadingDeg, afHoldSeconds, aiInterrupts, aiDanger, afLeash)
EndFunction

; v0.7.3 (2026-10-01): whom the walker turns to if she stops SHORT of her point and cannot get closer (body contact with the
; NPC she walked to, a table in the way). Without one she is turned to the heading you asked for. GoTo sets it by itself for
; an ACTOR target; after GoToPoint call this yourself. Call it right AFTER GoTo / GoToPoint - every new session (a retask
; too) clears it. None clears it. Returns False when the walker has no session. Older Standpoint: the function is missing
; (check GetVersion() >= "0.7.3" before calling, or the call fails with a Papyrus log line and nothing else).
Bool Function SetFaceTarget(Actor akWalker, ObjectReference akFace) Global
    Standpoint_PlayerAlias d = _Driver()
    If !d || !akWalker
        Return False
    EndIf
    Return d.ApiSetFaceTarget(akWalker, akFace)
EndFunction

; End the walker's session: package removed, pose reset, slot freed.
; v1.4.0: also ends her walk off the navmesh (WalkTo) or the stand after it - an open walk answers X 3.
; True = she had a session or a walk.
Bool Function Release(Actor akWalker) Global
    If !akWalker
        Return False
    EndIf
    Bool hadWalk = Standpoint.WalkStop(akWalker, 3)
    Standpoint_PlayerAlias d = _Driver()
    If !d
        Return hadWalk
    EndIf
    Bool hadSession = d.ApiRelease(akWalker)
    If hadSession
        Return True
    EndIf
    Return hadWalk
EndFunction

; Read-only danger check on the corridor from akWalker to a destination ref.
; Returns the number of hostiles found; -1 if the call could not run. Use it to
; decide whether to send someone at all, without starting a session.
Int Function CountThreatsOnRoute(Actor akWalker, ObjectReference akTarget, Float afCorridor = 600.0) Global
    If !akWalker || !akTarget
        Return -1
    EndIf
    Float[] th = Standpoint.ScanThreats(akWalker, akTarget.GetPositionX(), akTarget.GetPositionY(), akTarget.GetPositionZ(), afCorridor, 4096.0)
    If th.Length < 10
        Return -1
    EndIf
    Return th[0] as Int
EndFunction

; True while the walker has an active session (walking or holding).
Bool Function IsBusy(Actor akWalker) Global
    Standpoint_PlayerAlias d = _Driver()
    If !d || !akWalker
        Return False
    EndIf
    Return d.ApiIsBusy(akWalker)
EndFunction

; v0.7.4 (2026-10-03): where the walker's session stands - for a consumer that wants to ASK instead of waiting for an event.
;   0 = no session    1 = walking (or Standpoint's arrival handling is still in progress)    2 = arrived and holding
; With hold 0 the session ends at arrival, so the state goes 1 -> 0 (use the "Standpoint_Arrived" event or your own check
; then). Returns 0 when Standpoint is absent or akWalker is None. New in v0.7.4: on an older Standpoint the function is
; missing (check GetVersion() >= "0.7.4" before calling, or the call fails with a Papyrus log line and nothing else).
Int Function GetArrivalState(Actor akWalker) Global
    Standpoint_PlayerAlias d = _Driver()
    If !d || !akWalker
        Return 0
    EndIf
    Return d.ApiGetArrivalState(akWalker)
EndFunction

; ---- v1.2.0 (2026-10-30): THE MOVE REQUEST FROM PAPYRUS - the same 8 values as the C++ request line ----------
; (IStandpoint2::RequestMove, API/StandpointAPI.h; the integration guide's "The move request"), so CLOSE mode is open
; to Papyrus mods: two NPCs walked body to body (down to about 30 cm with HIGGS in VR), standing still near the point
; counts as arriving, she steps back and walks around by herself, the player standing near does not cancel, and a
; request that cannot be walked answers O 17.
;   akWho         the NPC who walks (not the player)
;   akWhere       the target: an NPC, furniture, an object, a marker. None = release akWho (answers X 3).
;                 akWhere = akWho = she holds her place.
;   afDistCm      centre of the target to her, centimetres
;   afFacingHour  where around the target, as a clock hour in the TARGET's facing (0 / 12 front, 3 its right,
;                 6 behind, 9 its left; decimals allowed); -1 = the side she is nearest to
;   afOrientHour  her own facing once there (0 = toward the target, 6 = her back to it)
;   aiPose        0 plain stand   1 standard wait   2 kneel   3 bend
;   afHoldSeconds how long she holds the spot after arriving (0 = let go at once, -1 = until released)
;   aiClose       0 normal   1 CLOSE contact (to an NPC: body to body; the two bodies ignore each other - through
;                 HIGGS in VR, through Standpoint's own collision hook without HIGGS on SE / AE)
; Returns a handle (>= 1), or 0 when Standpoint is absent or akWho is None. The answer comes ONCE as the
; "Standpoint_MoveResult" event (header above); or ask GetMoveOutcome / GetMoveResult with the handle.
; A game load answers nothing: forget your open handles in OnPlayerLoadGame.
; New in v1.2.0: check GetVersion() first (on an older Standpoint the call fails with a Papyrus log line and returns 0).
Int Function RequestMove(Actor akWho, ObjectReference akWhere, Float afDistCm, Float afFacingHour, Float afOrientHour, Int aiPose, Float afHoldSeconds, Int aiClose) Global
    If !akWho || Game.GetModByName("Standpoint.esp") == 255
        Return 0
    EndIf
    Return Standpoint.ApiMoveSubmit(akWho, akWhere, afDistCm, afFacingHour, afOrientHour, aiPose, afHoldSeconds, aiClose)
EndFunction

; v1.2.0: where a RequestMove stands. "" = no such handle (never made, one of more than 128 newer ones, or made before
; the last game load); "-" = still open; else the answer's letter (C, P, O, D, L, X, B).
String Function GetMoveOutcome(Int aiHandle) Global
    Return Standpoint.ApiMoveOutcome(aiHandle)
EndFunction

; v1.2.0: the answer's measured values, once there is one: [distanceCm, facingHour, orientationHour, detail];
; [] while the request is open, or for an unknown handle.
Float[] Function GetMoveResult(Int aiHandle) Global
    Return Standpoint.ApiMoveResult(aiHandle)
EndFunction

; ---- v1.4.0 (2026-11-02): THE WALK OFF THE NAVMESH (Skyrim VR) --------------------------------------------------------
; Walk akWho to (afX, afY, afZ) along a planned route - the navmesh, bare ground (rocks, a bath platform) and shallow
; water up to afDepthFrac x her height (0.05-0.65; <= 0 = 0.45). She walks every step with her own legs: Standpoint
; steers her movement the way a pull on a chain at her hips would (planner direct control); nothing slides her.
;   afFacingDeg     her facing at the end, compass degrees (0 = north, clockwise); < 0 = keep her last travel direction
;   afHoldSeconds   -1 = she stands there (held, not posed) until Release / a new WalkTo / a GoTo; 0 = let go at arrival;
;                   N = N seconds
;   afStopSubmerged > 0 = she arrives as soon as her submerged level reaches it (0..1, e.g. 0.35 = to the waist-ish)
; If she has a Standpoint session (GoTo / GoToPoint / RequestMove) it is released first; her older walk answers X 6.
; Returns a handle (>= 1), or 0 when Standpoint is absent or akWho is None. The answer comes ONCE as the
; "Standpoint_WalkResult" event (header above); or ask GetWalkOutcome / GetWalkResult with the handle. A game load answers
; nothing. On SE / AE the walk answers B 19 at once - check CanWalkOffNavmesh() first and keep your old way there.
; New in v1.4.0: check GetVersion() first (an older Standpoint has no WalkTo: the call fails with a Papyrus log line).
; v1.5.0: the route follows the navmesh where there is one (every leg still checked by rays), she opens unlocked ordinary
; doors on the way (locked and load doors stay walls), keeps clear of NPCs standing still, and gets a new route after
; every stall or push in a new place (O 18 when none helps).
; v1.5.1 - UP TO 200 m. A walk beyond 30 m has 90 s + 1 s per 150 u of her distance. A standing NPC's idle is broken first.
; v1.6.0 - Standpoint's OWN path all the way (the engine's far travel is gone): a route over the loaded navmesh, checked ahead
; of her while she walks; she RUNS while more than 15 m of route are left, JOGS down to 3 m and WALKS the last 3 m (a pace that
; would last under a second is skipped). A point outside the loaded area answers O 26 at once (GoToPoint can travel there).
; Stairs both ways where the navmesh confirms them; she never stops on a stair.
Int Function WalkTo(Actor akWho, Float afX, Float afY, Float afZ, Float afFacingDeg = -1.0, Float afHoldSeconds = -1.0, Float afDepthFrac = 0.45, Float afStopSubmerged = 0.0) Global
    If !akWho || Game.GetModByName("Standpoint.esp") == 255
        Return 0
    EndIf
    ; Her session is released only where the walk can run (the 1.4.0 review): on SE / AE WalkStart answers B 19 (once) and
    ; her GoTo / hold goes on untouched. Her older WALK is not stopped here: WalkStart replaces it itself (X 6) and keeps her
    ; held until the new walk takes her. (v1.5.1: nor is the far session of her older walk: that walk is replaced, X 6.)
    Bool canWalk = Standpoint.WalkSupported()
    If canWalk
        If IsBusy(akWho)
            _ReleaseUnlessFar(akWho)
        EndIf
    EndIf
    Return Standpoint.WalkStart(akWho, afX, afY, afZ, afFacingDeg, afHoldSeconds, afDepthFrac, afStopSubmerged)
EndFunction

; v1.4.0: where a WalkTo stands. "" = no such handle (never made, one of more than 128 newer ones, or made before the
; last game load); "-" = still open; else the answer's letter (C, O, P, X, B).
String Function GetWalkOutcome(Int aiHandle) Global
    Return Standpoint.WalkOutcome(aiHandle)
EndFunction

; v1.4.0: the answer's values, once there is one: [distLeft u, submergedLevel, detail, x, y, z]; [] while open or unknown.
Float[] Function GetWalkResult(Int aiHandle) Global
    Return Standpoint.WalkResultValues(aiHandle)
EndFunction

; v1.4.0: True when WalkTo can walk off the navmesh here (Skyrim VR, the driver's hook installed). False on SE / AE
; (WalkTo would answer B 19) and when Standpoint is absent. (v1.5.0: True on SE / AE too once the start-up self-test says
; READY and Data/SKSE/Plugins/Standpoint.ini has [DirectWalk] SEAE = 1.)
Bool Function CanWalkOffNavmesh() Global
    If Game.GetModByName("Standpoint.esp") == 255
        Return False
    EndIf
    Return Standpoint.WalkSupported()
EndFunction

; ---- v1.5.0 (2026-11-05): THE WALK INTO WATER ------------------------------------------------------------------------------
; "Walk her into the nearest water she can really reach, and let her stand in it." Standpoint asks Water Finder (1.6.0+)
; for the nearest water within afMaxMetres of her, then plans from HER toward it and stops at the first WADING SPOT she can
; walk to: water between afDepthMinFrac and afDepthMaxFrac x her height (0.05-0.65), with water round it, a place to stand.
; Deeper water is never crossed; a dock, a ledge top or the foot of a wall is never the answer. None reachable near that
; water: the next water Water Finder 1.6.2 offers (inside the same 90 s). She walks there with her own legs (as WalkTo) and
; ends FACING THE WATER (from where her route entered the water to the spot).
;   afHoldSeconds  -1 = she stands there until Release / a new WalkTo / WalkToWater / GoTo; 0 = let go at arrival; N seconds
; Returns a handle (>= 1; the same numbers as WalkTo), or 0 when Standpoint is absent or akWho is None. The answer comes ONCE
; as the "Standpoint_WalkResult" event (header above; afDistLeft = to the wading spot); or ask GetWalkOutcome / GetWalkResult.
; New answers: O 23 no water within afMaxMetres, O 24 water but no wading spot she can reach, B 25 Water Finder absent / too
; old. Her Standpoint session is released first only where the water walk can run (CanWalkToWater).
; New in v1.5.0: check GetVersion() first (an older Standpoint has no WalkToWater: the call fails with a Papyrus log line).
; v1.6.0 - DEEPER, BY PRIORITY, IN ONE WALK: the entry is shin-deep water (afDepthMinFrac); from there Standpoint looks two body
; lengths further for knee-deep water, and from that spot two body lengths more for thigh-deep (afDepthMaxFrac, 0.45 = the
; thigh, is the deepest) - all joined into one route, so she never stops between the steps.
Int Function WalkToWater(Actor akWho, Float afMaxMetres = 20.0, Float afDepthMinFrac = 0.15, Float afDepthMaxFrac = 0.45, Float afHoldSeconds = -1.0) Global
    If !akWho || Game.GetModByName("Standpoint.esp") == 255
        Return 0
    EndIf
    Bool canWade = Standpoint.WaterWalkSupported()
    If canWade
        If IsBusy(akWho)
            _ReleaseUnlessFar(akWho)
        EndIf
    EndIf
    Return Standpoint.WalkToWaterStart(akWho, afMaxMetres, afDepthMinFrac, afDepthMaxFrac, afHoldSeconds)
EndFunction

; v1.5.0: True when WalkToWater can run here: the walk off the navmesh is available (CanWalkOffNavmesh) AND Water Finder
; 1.6.0+ is installed. False when Standpoint is absent.
Bool Function CanWalkToWater() Global
    If Game.GetModByName("Standpoint.esp") == 255
        Return False
    EndIf
    Return Standpoint.WaterWalkSupported()
EndFunction

; v0.8.0 (2026-10-08): NOT for consumers. The DLL calls this to hand an open move request (the C++ interface revision 2,
; IStandpoint2::RequestMove - API/StandpointAPI.h) to the driver. A request the driver cannot take is answered here.
Function ApiWake(Int aiId) Global
    Standpoint_PlayerAlias d = _Driver()
    If !d
        Standpoint.ApiPostResult(aiId, "B", 11)
        Return
    EndIf
    d.ApiRunRequest(aiId)
EndFunction

; v1.2.0: NOT for consumers. The DLL calls this with the answer to a RequestMove; it sends "Standpoint_MoveResult".
; An answer that is no longer the one the DLL keeps for the handle (a load came in between) is dropped.
Function ApiMoveDone(Int aiHandle, Int aiWho, Int aiWhere, String asOutcome, Int aiDetail, Float afDistCm, Float afFacingHour, Float afOrientHour) Global
    If Standpoint.ApiMoveOutcome(aiHandle) != asOutcome
        Return
    EndIf
    Int ev = ModEvent.Create("Standpoint_MoveResult")
    If ev
        ModEvent.PushForm(ev, Game.GetForm(aiWho))
        ModEvent.PushInt(ev, aiHandle)
        ModEvent.PushString(ev, asOutcome)
        ModEvent.PushInt(ev, aiDetail)
        ModEvent.PushFloat(ev, afDistCm)
        ModEvent.PushFloat(ev, afFacingHour)
        ModEvent.PushFloat(ev, afOrientHour)
        ModEvent.PushForm(ev, Game.GetForm(aiWhere))
        ModEvent.Send(ev)
    EndIf
EndFunction

; v1.4.0: NOT for consumers. The DLL calls this with the answer to a WalkTo; it sends "Standpoint_WalkResult".
; An answer that is no longer the one the DLL keeps for the handle (a load came in between) is dropped.
Function ApiWalkDone(Int aiHandle, Int aiWho, String asOutcome, Int aiDetail, Float afDistLeft, Float afSubmerged) Global
    If Standpoint.WalkOutcome(aiHandle) != asOutcome
        Return
    EndIf
    Int ev = ModEvent.Create("Standpoint_WalkResult")
    If ev
        ModEvent.PushForm(ev, Game.GetForm(aiWho))
        ModEvent.PushInt(ev, aiHandle)
        ModEvent.PushString(ev, asOutcome)
        ModEvent.PushInt(ev, aiDetail)
        ModEvent.PushFloat(ev, afDistLeft)
        ModEvent.PushFloat(ev, afSubmerged)
        ModEvent.Send(ev)
    EndIf
EndFunction

; v1.4.0: NOT for consumers. A C++ walk request (IStandpoint3::RequestWalk) for an NPC who has a Standpoint session: the DLL
; calls this first - the session is released (one owner per NPC) - and starts the walk when it has returned.
Function ApiWalkFreeSession(Actor akWho) Global
    Standpoint_PlayerAlias d = _Driver()
    If !d || !akWho
        Return
    EndIf
    If d.ApiIsBusy(akWho)
        If !d.ApiIsFarSession(akWho)      ; v1.5.1: the far session of her older walk - that walk is replaced (X 6) and frees it
            d.ApiRelease(akWho)
        EndIf
    EndIf
EndFunction

; v1.6.0: NOT for consumers. The stpm console test (Standpoint's own TestCmd) switches the driver's event ear on / off: while it
; is on, every Standpoint_* event is handed to the DLL's test log ("[TEST] MIX event ..."). A game load switches it off.
Function ApiTestListen(Bool abOn) Global
    Standpoint_PlayerAlias d = _Driver()
    If d
        d.ApiTestListen(abOn)
    EndIf
EndFunction

; v1.5.1: NOT for consumers. Her session is released (Release) - unless it is the FAR phase of her own older walk: the new walk
; replaces that walk itself (X 6), and the walk frees its session. (1.6.0: the far phase is gone - this is a plain Release.)
Function _ReleaseUnlessFar(Actor akWho) Global
    Standpoint_PlayerAlias d = _Driver()
    If d
        If d.ApiIsFarSession(akWho)
            Return
        EndIf
    EndIf
    Release(akWho)
EndFunction

; v1.5.1: NOT for consumers. THE FAR PHASE of a walk beyond 30 m: the DLL asks for Standpoint's own travel session toward
; (afX, afY, afZ) - a plain session (the engine's walking / running) that the DLL watches and ends itself. Returns the session
; handle (the slot + 1), 0 = refused (all 4 slots busy, or the engine is not ready).
Int Function ApiWalkFarStart(Actor akWho, Float afX, Float afY, Float afZ, Float afFaceDeg, Int aiWalk) Global
    Standpoint_PlayerAlias d = _Driver()
    If !d || !akWho
        Return 0
    EndIf
    Return d.ApiFarStart(akWho, afX, afY, afZ, afFaceDeg, aiWalk)
EndFunction

; v1.5.1: NOT for consumers. The DLL ends walk aiWalk's far session. aiMode (decided again by the DLL when this runs -
; Standpoint.WalkFarEndMode): 0 the let-go, 1 the override off only (the direct walk / a newer walk has her: she keeps moving),
; 2 the override off + EvaluatePackage. Returns the mode applied; -1 = nothing (the slot holds another session).
Int Function ApiWalkFarEnd(Actor akWho, Int aiWalk, Int aiMode) Global
    Standpoint_PlayerAlias d = _Driver()
    If !d || !akWho
        Return -1
    EndIf
    Return d.ApiFarEnd(akWho, aiWalk, aiMode)
EndFunction

; ---------------------------------------------------------------------------

Standpoint_PlayerAlias Function _Driver() Global
    Quest q = Game.GetFormFromFile(0x800, "Standpoint.esp") as Quest
    If !q
        Return None
    EndIf
    Return q.GetAlias(0) as Standpoint_PlayerAlias
EndFunction
