Scriptname StandpointAPI Hidden
{ ============================================================================
  Standpoint - public Papyrus API (v0.9.9). SHIPS with the mod.

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
              7 (v0.9.9, a C++ close move request to an NPC only) NO WAY: she could not get to the
                asked point - the way is blocked and no other way worked; released
    "Standpoint_Interrupted" handler(Form akWalker, Float afCode)
        code: 1 combat  2 player bump/body-block  3 player hand-grab (VR)
              4 danger on or near the route

  SOFT-DEP CHECK:  StandpointAPI.GetVersion() returns "" when absent.
  Up to 4 concurrent walkers. A walker holds their slot (engine-pinned)
  until Release() or a new GoTo() retasks them.
  ============================================================================ }

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
Bool Function Release(Actor akWalker) Global
    Standpoint_PlayerAlias d = _Driver()
    If !d || !akWalker
        Return False
    EndIf
    Return d.ApiRelease(akWalker)
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

; ---------------------------------------------------------------------------

Standpoint_PlayerAlias Function _Driver() Global
    Quest q = Game.GetFormFromFile(0x800, "Standpoint.esp") as Quest
    If !q
        Return None
    EndIf
    Return q.GetAlias(0) as Standpoint_PlayerAlias
EndFunction
