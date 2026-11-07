Scriptname Standpoint_PlayerAlias extends ReferenceAlias
{ Standpoint driver (v2.0.0; the slot engine dates from v0.4.0). Lives on alias 0 (player) of Standpoint_Quest so it is
  permanently bound (the WFTest lesson: never drive from a fire-and-forget magic effect).

  S3 SLOT ENGINE: 4 concurrent walker sessions. Slot i uses marker alias (i+1) and its
  own TravelAndUseIdle clone package. Public surface = StandpointAPI.psc; consumers
  never touch this script directly.

  Also carries the S1 park-and-face measurement test (legacy, shares slot 0's alias)
  and the S2 spell harness (walker select + relation cycling), now routed through the
  slot engine. Do not run the S1 test while API sessions are active. }

; ---- slot engine state (4 slots; slot i <-> alias i+1) ----
Actor[] slotWalker
ObjectReference[] slotMarker
Package[] slotPkg
Bool[] slotActive          ; polling toward arrival
Bool[] slotHolding         ; arrived, package still pinning
Float[] slotX
Float[] slotY
Float[] slotFaceDeg
Float[] slotStart
Int[] slotLastIdx
Bool[] slotSawTravel
Float[] slotHoldSeconds
Float[] slotHoldUntil
Int[] slotPose
Float[] slotDestZ
Int[] slotInterrupts
Bool[] slotDestIsPlayer
Bool[] slotCombatSeen
Int[] slotContactPolls
; ---- v0.6.0 Tier 1: danger recognition ----
Float[] slotZ              ; destination Z (the threat scan needs the full point)
Int[] slotDanger           ; policy 0 off / 1 report only / 2 DROP (default)
Float[] slotLeash          ; v0.6.1: max walker-player distance; <=0 disables
ObjectReference[] slotFaceRef ; v0.7.3: whom she turns to when she stops SHORT (GoTo sets it for an actor target; SetFaceTarget)
; ---- v0.8.0 the move API (C++ IStandpoint2::RequestMove through the DLL's mailbox) ----
Int[] slotApiId            ; the open request this session answers (0 = a Papyrus GoTo session: nothing to answer)
Float[] slotFaceOffset     ; degrees clockwise from "facing the face target" (the request's orientation hour * 30)
ObjectReference[] slotMarkerPlain ; the slot's second IdleMarker: base 0x80F, NO idle animation (pose 0 = a plain stand)
Bool[] slotPlain           ; this session parks her on the plain marker
; ---- v0.9.0 close contact (slot 8 of the request line) ----
Int[] slotClose            ; 1 = close contact: a stall near her target is her arrival; the player near does not cancel
Float[] slotStallX         ; where she stood when she last moved 4 u ...
Float[] slotStallY
Float[] slotStallT         ; ... and when (the stall clock)
; ---- v0.9.3 close contact, second round (report 05 §12 / §13) ----
Bool[] slotSeenOurs        ; OUR package has been seen running (whatever its procedure): the stall clock runs from that poll
Int[] slotPairH            ; v0.9.4: the handle of the pair "ignore each other" rule this slot's session holds in the DLL (Standpoint.PairArm); 0 = none
Float[] slotArrivalBusy    ; v0.9.4: = slotStart[i] while ONE stack runs the arrival handling of that session (a second stack skips it)
Bool[] slotHasMoved        ; v0.9.5: she has moved 4 u from where the first our-package poll found her (the stall rule: 3 s still once moved, 8 s if never)
Bool[] slotFreeing         ; v0.9.5: FreeSession(i) is running right now - SameSession answers False meanwhile
; ---- v0.9.7 step back and around; v0.9.8 no glide, real stops (the user's rulings of 2026-10-10; report 05 §15-§17) ----
Int[] slotRoute            ; 0 = no stop on the way: she walks to the ASKED point under the normal arrival rules; she is walking to a STOP first: 1 = her step back, 2 = a point on the way around, 3 = the approach point
Int[] slotRouteLegs        ; how many stops this session has been sent to (a loop guard: at most 6)
Float[] slotFinX           ; the ASKED point and heading, kept while slotX / slotY / slotZ / slotFaceDeg hold a waypoint
Float[] slotFinY
Float[] slotFinZ
Float[] slotFinFace
Bool[] slotFinPlain        ; v0.9.8: the marker the session asked for at the ASKED point (True = the plain one, pose 0); every stop on the way uses the plain one
Int[] slotRetry            ; v0.9.8: 1 = the ONE second walk in of this session has been used (or she did not walk at all on a leg)
Int[] slotRouteFails       ; v0.9.9: how many legs on the way round (around / approach) have FAILED in this session (blocked, or she stood still); the second one = NO WAY (O 17)
Int[] slotBlockPolls       ; v0.9.9: how many polls in a row found the engine's "movement blocked" (run-once type 36) with her not moving; 2 = blocked
Bool[] slotRouteOk         ; v0.9.8: a close session with an NPC target: slotFin* and slotFaceRef are set, stops and a second walk in are possible
; ---- v1.2.0 the BiSR letter (report 11, 2026-10-30) ----
Float[] slotLastPollT      ; when this session was last polled - a gap over 5 s = the game was paused (a menu, the VR dashboard)
Float[] slotPausedT        ; paused time inside this session: the 90 s limit does not count it
Float[] slotLastPX         ; where she stood at the last poll (the player-contact rule: a walker who is still moving is not blocked)
Float[] slotLastPY
Int[] slotLegKicks         ; v1.3.0: how far a leg that has not STARTED was kicked: 0 none, 1 package evaluated again, 2 sent once more
; ---- v1.3.0 THE DETOUR (the user's waypoint plan, 2026-10-30): a blocked leg gets a planned way round (PathPlanner.cpp) ----
Int[] slotPathH            ; the planner's handle while it plans (0 = none)
Int[] slotDetour           ; 0 none, 1 planning, 2 walking the way round
Int[] slotDetourTries      ; ways round started in this session (at most 2)
Float[] slotPlanT          ; when the planning started (given up after 5 s)
Float[] slotWpX            ; the way round's points: slot i uses [i * 16 ... i * 16 + 15]
Float[] slotWpY
Float[] slotWpZ
Int[] slotWpN              ; how many points
Int[] slotWpK              ; the next one
Int[] slotDefState         ; the leg the way round leads back to (its route state and point)
Float[] slotDefX
Float[] slotDefY
Float[] slotDefZ
Float[] slotDefFace
; ---- v1.5.1 THE FAR PHASE of a walk (a WalkTo / WalkToWater / RequestWalk beyond 30 m: the engine's travel, watched by the DLL) ----
Int[] slotFar              ; the walk whose far session this is (0 = an ordinary session); the DLL ends it (ApiFarEnd)
                           ; (1.6.0: the far phase is removed from the walks - the DLL never starts one: dormant)
; ---- v1.6.0 THE STEERED SESSION (report "1.6.0 Plan" S1 §2-§3; SteerSession.cpp) and BUG 1 ----
Int[] slotSteer            ; the DLL's steer id: > 0 a steered session, -1 starting (SteerBegin has not asked the DLL yet), 0 an engine session
Int[] slotSteerHold        ; after a steered arrival: 1 pinned by the package (the engine's hold), 2 held by the walk's own stand
Int[] slotSteerWait        ; the wait already reported for this episode (1 a grab, 2 combat), 0 none
Int[] slotSteerOld         ; a steered START in progress (slotSteer -1) replaces this live steered session of hers (0 none): the DLL
                           ; supersedes it at SteerStart; a release before that stops it (FreeSession), so none is ever orphaned
Int[] slotSteerPhase       ; the DLL's phase at the last poll (FIX ROUND R1 #9: the contact rule waits while she plans / waits)
Int[] slotSteerKind        ; v1.6.1 (review R1 § VERIFY N1): the steered session's kind - 1 the clock (RequestMove), 2 GoTo, 3 a POINT; 0 none
Int[] slotEngineLeg        ; v1.6.1 (C4): this session walks ONE engine leg after a steered try (EngineFallback) - 1 no route, 2 she sat,
                           ; 3 no route and the POINT is off the navmesh: the leg goes to its nearest navmesh point (FIX-UP C4a-2); 0 none
Float[] slotEngAskX        ; v1.6.1 FIX-UP (C4a-2): the asked point of an engine leg 3 - its arrival is measured to it (ArrivalTail)
Float[] slotEngAskY
Float[] slotSteerLimit     ; the backstop time limit of a steered session, s (the walk's own limit answers first)
ObjectReference[] slotAnchor ; BUG 1: the reference the slot's marker is placed relative to (a relative session's target); None = the walker
Bool avoidProbed = False   ; Tier 3 Phase A2 fires once per game session
Bool engineReady = False
Bool saAvailable = False

; ---- S1 legacy test state ----
Actor targetNpc = None
ObjectReference marker = None
Package parkPkg = None
Bool running = False
Int phase = 1
Int cycle = 0
Int lastIdx = -99
Int timeouts = 0
Float faceTarget = 0.0
Float cycleStart = 0.0
Float settleStart = 0.0
Float savedAvoid = -1.0
Int Property CyclesPhase1 = 10 AutoReadOnly
Int Property CyclesPhase2 = 6 AutoReadOnly

; ---- S2 spell-harness state ----
; wade: the scripted last-wet-step after a shore arrival (navmesh ends at the
; waterline - the WF consumer rule: package to the SHORE, glide the rest)
Bool s2WadePending = False
Float s2WadeX = 0.0
Float s2WadeY = 0.0
Float s2WadeZ = 0.0
Actor s2Walker = None
ObjectReference s2Dest = None
Int s2Rel = 0

Event OnInit()
    Setup()
EndEvent

Event OnPlayerLoadGame()
    Setup()
EndEvent

Function Setup()
    parkPkg = Game.GetFormFromFile(0x803, "Standpoint.esp") as Package

    ; ---- v1.6.0 BUG 2 (report "1.6.0 Plan" S1 §5): THE ORPHAN CURE, before anything is reset. No session survives a load (the
    ; DLL answers its open requests X 12), but a package override does - PapyrusUtil keeps it in the save - and so does a marker
    ; alias: after a load an NPC could walk on to an old session's marker with no owner (handover 33 M8 / H7). The four marker
    ; aliases are cleared and every override of the four slot packages is removed, on every actor.
    Package setupPkg1 = Game.GetFormFromFile(0x80A, "Standpoint.esp") as Package
    Package setupPkg2 = Game.GetFormFromFile(0x80B, "Standpoint.esp") as Package
    Package setupPkg3 = Game.GetFormFromFile(0x80C, "Standpoint.esp") as Package
    Int setupK = 1
    While setupK <= 4
        ReferenceAlias setupAlias = GetOwningQuest().GetAlias(setupK) as ReferenceAlias
        If setupAlias
            setupAlias.Clear()
        EndIf
        setupK += 1
    EndWhile
    Int setupRemoved = 0
    If parkPkg
        setupRemoved += ActorUtil.RemoveAllPackageOverride(parkPkg)
    EndIf
    If setupPkg1
        setupRemoved += ActorUtil.RemoveAllPackageOverride(setupPkg1)
    EndIf
    If setupPkg2
        setupRemoved += ActorUtil.RemoveAllPackageOverride(setupPkg2)
    EndIf
    If setupPkg3
        setupRemoved += ActorUtil.RemoveAllPackageOverride(setupPkg3)
    EndIf
    Standpoint.Log("Setup: the 4 marker aliases cleared, " + setupRemoved + " slot package override(s) removed (no session survives a load)")

    slotWalker = new Actor[4]
    ; v1.6.0 BUG 2: the slot MARKERS are KEPT across loads (saved script variables - after a load they hold the same references).
    ; Until 1.5.1 both arrays were made new here at every load, and each load could leave up to 8 more persistent markers in the
    ; save, never deleted. A reference that is not one of our marker bases any more is dropped (made again at its next use).
    If !slotMarker || slotMarker.Length != 4
        slotMarker = new ObjectReference[4]
    EndIf
    If !slotMarkerPlain || slotMarkerPlain.Length != 4
        slotMarkerPlain = new ObjectReference[4]
    EndIf
    Form setupBaseStd = Game.GetFormFromFile(0x802, "Standpoint.esp")
    Form setupBasePlain = Game.GetFormFromFile(0x80F, "Standpoint.esp")
    setupK = 0
    While setupK < 4
        ObjectReference setupMk = slotMarker[setupK]
        If setupMk
            If setupMk.GetBaseObject() != setupBaseStd
                slotMarker[setupK] = None
            EndIf
        EndIf
        ObjectReference setupMkP = slotMarkerPlain[setupK]
        If setupMkP
            If setupMkP.GetBaseObject() != setupBasePlain
                slotMarkerPlain[setupK] = None
            EndIf
        EndIf
        setupK += 1
    EndWhile
    slotPkg = new Package[4]
    slotActive = new Bool[4]
    slotHolding = new Bool[4]
    slotX = new Float[4]
    slotY = new Float[4]
    slotFaceDeg = new Float[4]
    slotStart = new Float[4]
    slotLastIdx = new Int[4]
    slotSawTravel = new Bool[4]
    slotHoldSeconds = new Float[4]
    slotHoldUntil = new Float[4]
    slotPose = new Int[4]
    slotDestZ = new Float[4]
    slotInterrupts = new Int[4]
    slotDestIsPlayer = new Bool[4]
    slotCombatSeen = new Bool[4]
    slotContactPolls = new Int[4]
    slotZ = new Float[4]
    slotDanger = new Int[4]
    slotLeash = new Float[4]
    slotFaceRef = new ObjectReference[4]
    slotApiId = new Int[4]
    slotFaceOffset = new Float[4]
    slotPlain = new Bool[4]                ; (v1.6.0: slotMarkerPlain is kept - see above)
    slotClose = new Int[4]
    slotStallX = new Float[4]
    slotStallY = new Float[4]
    slotStallT = new Float[4]
    slotSeenOurs = new Bool[4]
    slotPairH = new Int[4]         ; (a load: the DLL cleared its whole pair pool at kPreLoadGame)
    slotArrivalBusy = new Float[4]
    slotHasMoved = new Bool[4]
    slotFreeing = new Bool[4]
    slotRoute = new Int[4]
    slotRouteLegs = new Int[4]
    slotFinX = new Float[4]
    slotFinY = new Float[4]
    slotFinZ = new Float[4]
    slotFinFace = new Float[4]
    slotFinPlain = new Bool[4]
    slotRetry = new Int[4]
    slotRouteOk = new Bool[4]
    slotRouteFails = new Int[4]
    slotBlockPolls = new Int[4]
    EnsureV12Arrays(True)
    slotFar = new Int[4]           ; v1.5.1 (a load: the DLL forgot its walks' far phases at kPreLoadGame)
    EnsureSteerArrays(True)        ; v1.6.0 (a load: the DLL forgot its steered sessions at kPreLoadGame)
    slotPkg[0] = parkPkg
    slotPkg[1] = setupPkg1
    slotPkg[2] = setupPkg2
    slotPkg[3] = setupPkg3
    engineReady = slotPkg[0] && slotPkg[1] && slotPkg[2] && slotPkg[3]

    ; v1.2.0 (the user, 2026-10-30: "remove the test spells"): the four developer test spells (Park Test 0x805, Slot
    ; Test 0x807, Send Pointed 0x809, Send to Water 0x80E) are no longer given to the player - Standpoint ships inside
    ; other mods now - and a player who has them from an older version loses them here. The records stay in the ESP
    ; (unused); the console test `stpt` is the test tool.
    RemoveTestSpell(0x805)
    RemoveTestSpell(0x807)
    RemoveTestSpell(0x809)
    RemoveTestSpell(0x80E)

    saAvailable = Game.GetModByName("SeverActions.esp") != 255
    ; v1.2.0: no longer listens to SeverActions' travel event. Standpoint has started no SeverActions journey since 0.9.1,
    ; the handler only logged, and SeverActions sends FOUR values to a three-value handler - the VM refused every call
    ; with an argument-count error in the Papyrus log (report 11 §5). A registration kept in an older save is removed.
    UnregisterForModEvent("SeverActions_TravelComplete")
    ApiTestListen(False)            ; v1.6.0: the stpm test's ear is off after every load (a registration is kept in the save)
    Bool setupSteered = Standpoint.SteerSupported()
    Standpoint.Log("driver v2.0.0 ready (engine=" + engineReady + " sa=" + saAvailable + " steered=" + setupSteered + ")")
    ; v1.6.0 BUG 2: the markers older Standpoints left in this save are COUNTED (one MARKERS: line; deleted only with
    ; Standpoint.ini [Cleanup] SweepMarkers = 1). The slot markers kept above are never counted as litter.
    ObjectReference[] setupKeep = new ObjectReference[8]
    setupK = 0
    While setupK < 4
        setupKeep[setupK] = slotMarker[setupK]
        setupKeep[setupK + 4] = slotMarkerPlain[setupK]
        setupK += 1
    EndWhile
    Standpoint.MarkerSweep(setupKeep)
EndFunction

; =========================== THE SLOT ENGINE ================================

Int Function ApiGoTo(Actor akWalker, ObjectReference akTarget, Int aiRelation, Float afStandoff, Float afHoldSeconds = -1.0, Int aiPose = 0, Int aiInterrupts = 1, Int aiDanger = 2, Float afLeash = 3500.0)
    ; v1.6.0 (report "1.6.0 Plan" S1 §2; spec D item 4): Skyrim VR with [Session] Steered = 1 - THE STEERED SESSION. The DLL
    ; solves the spot by physics (the solver's own candidates: furniture entries, box sides, the actor's sides), plans ONE route
    ; and walks her there with Standpoint's own steering; this slot keeps the whole contract (IsBusy, the events, the hold, the
    ; pose, Release). A target the steered walk cannot reach (another interior / worldspace) keeps the engine session below.
    If SteeredOn() && akTarget
        Bool goSteer = Standpoint.SteerCheck(akWalker, akTarget, 0.0, 0.0, 0.0)
        If goSteer
            Return GoToSteered(akWalker, akTarget, aiRelation, afStandoff, afHoldSeconds, aiPose, aiInterrupts, aiDanger, afLeash)
        EndIf
    EndIf
    Float[] slot = Standpoint.SolveSlot(akTarget, aiRelation, akWalker, afStandoff)
    If slot.Length == 0
        SendFailed(akWalker, 1.0)
        Return 0
    EndIf
    ; (v1.6.0 BUG 1: the marker is placed relative to the TARGET - its point lives in the target's cell)
    Int h = StartSession(akWalker, slot[0], slot[1], slot[2], slot[3], afHoldSeconds, False, False, 0, 0.0, 0.0, 0.0, 0.0, None, 0, akTarget)
    If h > 0
        slotPose[h - 1] = aiPose
        slotDestZ[h - 1] = akTarget.GetPositionZ()
        slotInterrupts[h - 1] = aiInterrupts
        slotDestIsPlayer[h - 1] = (akTarget as Actor) == Game.GetPlayer()
        slotDanger[h - 1] = aiDanger
        slotLeash[h - 1] = afLeash
        If akTarget as Actor
            slotFaceRef[h - 1] = akTarget      ; v0.7.3: an actor target is whom she faces if she ends short
        EndIf
    EndIf
    Return h
EndFunction

; v0.7.3: the walker's session turns toward akFace (instead of the asked heading) when she stops SHORT of her point and
; cannot get closer - body contact, a blocked approach. Call right AFTER GoTo / GoToPoint (a new session clears it).
; None clears it. False = she has no session.
Bool Function ApiSetFaceTarget(Actor akWalker, ObjectReference akFace)
    Int i = FindSlot(akWalker)
    If i < 0
        Return False
    EndIf
    slotFaceRef[i] = akFace
    Return True
EndFunction

Int Function ApiGoToPoint(Actor akWalker, Float afX, Float afY, Float afZ, Float afHeadingDeg, Float afHoldSeconds = -1.0, Int aiInterrupts = 1, Int aiDanger = 2, Float afLeash = 3500.0)
    ; v1.6.0 (S1 §3; spec D item 5 - the user said yes): a POINT session is walked steered too, on the same switch. Her legs keep
    ; "a point that is no place to stand moves toward her" and the 100 u strayed rule; no route (e.g. behind a load door) = ONE
    ; engine leg (the user's rule). interrupts 0 + a hand-grab / combat = the session waits and walks on afterwards.
    If SteeredOn() && akWalker
        Bool ptSteer = Standpoint.SteerCheck(akWalker, None, afX, afY, afZ)
        If ptSteer
            Return GoToPointSteered(akWalker, afX, afY, afZ, afHeadingDeg, afHoldSeconds, aiInterrupts, aiDanger, afLeash)
        EndIf
    EndIf
    Int h = StartSession(akWalker, afX, afY, afZ, afHeadingDeg, afHoldSeconds)
    If h > 0
        slotInterrupts[h - 1] = aiInterrupts
        slotDanger[h - 1] = aiDanger
        slotLeash[h - 1] = afLeash
    EndIf
    Return h
EndFunction

; ====================== v1.6.0: THE STEERED SESSION (driver side; report "1.6.0 Plan" S1 §2.0-§2.9, §3) ======================
; The consumer's door is unchanged (GoTo / GoToPoint / RequestMove). The driver keeps the SLOT and the whole contract: IsBusy,
; GetArrivalState, the three per-walker events, ApiReport + the measured result line, Release, hold -1 / 0 / N, the poses, the
; pair rule, leash / interrupts / danger (PollSlot, unchanged). The session's legs are NOT the travel package any more: the DLL
; (SteerSession.cpp) solves the spot by physics, plans ONE route and walks her there with Standpoint's own steering; PollSlot
; reads its state line (SteerState) every 0.5 s:
;   walking  -> nothing (the slot stays "1 = walking")
;   arrived  -> THE PIN: the marker moved to her feet with the final heading, the alias, the package on, her stand ended quietly
;               (SteerHandOff), then the existing arrival tail (Standpoint_Arrived, ApiReport C, the pose, the hold). Off the
;               navmesh with pose 0 the walk's own stand keeps her instead; hold 0 + pose 0 = she was let go at the point.
;   ended    -> the letter -> Failed / Interrupted + ApiReport + FreeSession (S1 §2.7); F = ONE engine leg to the solved spot.
; SE / AE, and [Session] Steered = 0: the engine session exactly as before (the switch is the VR rollback without a rebuild).

; True = sessions are walked steered here (Skyrim VR - or SE / AE with the walk on - and Standpoint.ini [Session] Steered = 1)
Bool Function SteeredOn()
    Return Standpoint.SteerSupported()
EndFunction

; ApiGoTo's steered branch. Every native call is made BEFORE StartSession, so the slot's fields are written right after its
; one-piece block with no call out of the script in between.
Int Function GoToSteered(Actor akWalker, ObjectReference akTarget, Int aiRelation, Float afStandoff, Float afHoldSeconds, Int aiPose, Int aiInterrupts, Int aiDanger, Float afLeash)
    Float gsX = akTarget.GetPositionX()
    Float gsY = akTarget.GetPositionY()
    Float gsZ = akTarget.GetPositionZ()
    Float gsFace = akTarget.GetAngleZ()
    Actor gsActor = akTarget as Actor
    Bool gsToPlayer = False
    Actor gsPlayer = Game.GetPlayer()
    If gsActor == gsPlayer
        gsToPlayer = True                  ; (None == the player is False: an object target is never the player)
    EndIf
    Int h = StartSession(akWalker, gsX, gsY, gsZ, gsFace, afHoldSeconds, False, False, 0, 0.0, 0.0, 0.0, 0.0, None, 0, akTarget, True)
    If h <= 0
        Return h
    EndIf
    Int i = h - 1
    Float gsStamp = slotStart[i]
    slotPose[i] = aiPose
    slotDestZ[i] = gsZ
    slotInterrupts[i] = aiInterrupts
    slotDestIsPlayer[i] = gsToPlayer
    slotDanger[i] = aiDanger
    slotLeash[i] = afLeash
    If gsActor
        slotFaceRef[i] = akTarget          ; (an actor target is whom she faces if she ends short - as the engine path)
    EndIf
    slotSteerKind[i] = 2                   ; v1.6.1 (N1): GoTo
    Float[] gsAsk = new Float[7]
    gsAsk[0] = 2.0                         ; kind 2: GoTo (a relation)
    gsAsk[1] = afHoldSeconds
    gsAsk[2] = aiPose as Float
    gsAsk[3] = aiInterrupts as Float
    gsAsk[4] = 0.0
    gsAsk[5] = aiRelation as Float
    gsAsk[6] = afStandoff
    SteerBegin(i, akWalker, gsStamp, akTarget, gsAsk)
    Return h
EndFunction

; ApiGoToPoint's steered branch (kind 3: a point).
Int Function GoToPointSteered(Actor akWalker, Float afX, Float afY, Float afZ, Float afHeadingDeg, Float afHoldSeconds, Int aiInterrupts, Int aiDanger, Float afLeash)
    Int h = StartSession(akWalker, afX, afY, afZ, afHeadingDeg, afHoldSeconds, False, False, 0, 0.0, 0.0, 0.0, 0.0, None, 0, None, True)
    If h <= 0
        Return h
    EndIf
    Int i = h - 1
    Float gpStamp = slotStart[i]
    slotInterrupts[i] = aiInterrupts
    slotDanger[i] = aiDanger
    slotLeash[i] = afLeash
    slotSteerKind[i] = 3                   ; v1.6.1 (N1): a POINT - off the navmesh the walk's own stand keeps her (SteerArrive)
    Float[] gpAsk = new Float[9]
    gpAsk[0] = 3.0                         ; kind 3: GoToPoint
    gpAsk[1] = afHoldSeconds
    gpAsk[2] = 0.0
    gpAsk[3] = aiInterrupts as Float
    gpAsk[4] = 0.0
    gpAsk[5] = afX
    gpAsk[6] = afY
    gpAsk[7] = afZ
    gpAsk[8] = afHeadingDeg
    SteerBegin(i, akWalker, gpStamp, None, gpAsk)
    Return h
EndFunction

; ApiRunRequest's steered branch (kind 1: the clock - a C++ RequestMove line or Papyrus RequestMove). The pair rule is armed
; BEFORE the DLL is asked (S1 §2.4: it bites only for bodies that start to overlap after it is armed).
Function RunSteered(Int aiId, Actor w, ObjectReference t, Float[] v, Int aiClose)
    Int pose = v[3] as Int
    Bool plain = False
    If pose == 0
        plain = True
    EndIf
    Bool closeB = False
    If aiClose == 1
        closeB = True
    EndIf
    Float hold = v[4]
    Float rsX = t.GetPositionX()
    Float rsY = t.GetPositionY()
    Float rsZ = t.GetPositionZ()
    Float rsFace = t.GetAngleZ()
    Bool rsToPlayer = False
    If (t as Actor) == Game.GetPlayer()
        rsToPlayer = True
    EndIf
    Int h = StartSession(w, rsX, rsY, rsZ, rsFace, hold, plain, closeB, 0, 0.0, 0.0, 0.0, 0.0, None, 0, t, True)
    If h == 0
        Standpoint.ApiPostResult(aiId, "B", 11)
        Return
    EndIf
    Int i = h - 1
    Float runStamp = slotStart[i]
    If pose == 2 || pose == 3
        slotPose[i] = pose
    EndIf
    slotDestZ[i] = rsZ
    slotDestIsPlayer[i] = rsToPlayer
    slotFaceRef[i] = t                     ; stopped short -> she turns to the target ...
    slotFaceOffset[i] = v[2] * 30.0        ; ... plus the asked orientation (degrees, clockwise)
    slotApiId[i] = aiId
    slotClose[i] = aiClose
    slotSteerKind[i] = 1                   ; v1.6.1 (N1): the clock
    Standpoint.Log("session " + i + " answers move request #" + aiId + " (steered; asked " + v[0] + " cm at hour " + v[1] + ", orientation " + v[2] + ", pose " + pose + ", hold " + hold + " s, close " + aiClose + ")")
    If aiClose == 1
        PairArmFor(i, w, t, runStamp)
    EndIf
    Float[] rsAsk = new Float[8]
    rsAsk[0] = 1.0                         ; kind 1: the clock
    rsAsk[1] = hold
    rsAsk[2] = pose as Float
    rsAsk[3] = slotInterrupts[i] as Float
    rsAsk[4] = aiClose as Float
    rsAsk[5] = v[0]
    rsAsk[6] = v[1]
    rsAsk[7] = v[2]
    SteerBegin(i, w, runStamp, t, rsAsk)
EndFunction

; Hand slot i's steered session to the DLL (after StartSession's one-piece block and the caller's field writes). False = the slot
; changed meanwhile (released, or retasked: that start owns it now) - nothing more is done here.
Bool Function SteerBegin(Int i, Actor akW, Float afStamp, ObjectReference akTarget, Float[] afAsk)
    If !StillThisStart(i, akW, afStamp) || slotSteer[i] != -1
        Return False
    EndIf
    Int sid = Standpoint.SteerStart(akW, i, akTarget, afAsk)
    If !StillThisStart(i, akW, afStamp) || slotSteer[i] != -1
        If sid > 0
            Standpoint.SteerStop(sid)      ; (this start lost the slot during the call: its steered session goes too)
        EndIf
        Return False
    EndIf
    If sid <= 0
        ; FIX ROUND (review R1 BLOCKER 2): a GoTo / GoToPoint caller already holds h > 0 - it is told the session ended (Failed 1)
        Standpoint.Log("session " + i + ": the DLL refused the steered session - answered B 11 (Failed 1), released")
        SendFailed(akW, 1.0)
        ApiReport(i, "B", 11)
        FreeSession(i)
        Return False
    EndIf
    slotSteer[i] = sid
    slotSteerOld[i] = 0                    ; (SteerStart superseded her older steered session in the DLL)
    Standpoint.Log("session " + i + ": steered (STEER #" + sid + ") - the DLL solves the spot, plans one route and walks her")
    Return True
EndFunction

; One poll of a steered session - PollSlot calls it after the leash, the interrupts and the danger scan (the session's own
; rules, unchanged); nothing else of PollSlot runs for it (no procedure watch, kicks, detours or stall rule: the DLL walks her).
Function SteerPoll(Int i, Actor w, Float afNow)
    Int sid = slotSteer[i]
    If sid < 0
        slotSteerPhase[i] = 0              ; (starting = planning: the contact rule waits - FIX ROUND R1 #9)
        SteerBackstop(i, w, afNow)         ; (SteerBegin has not asked the DLL yet)
        Return
    EndIf
    Float[] st = Standpoint.SteerState(sid)
    If slotSteer[i] != sid || slotWalker[i] != w || !slotActive[i]
        Return                             ; (released or retasked during the call)
    EndIf
    If slotFreeing
        If slotFreeing[i]
            Return                         ; (FIX ROUND R1 NIT 12: a release in flight sends its own event - no second one)
        EndIf
    EndIf
    If st.Length < 19
        Standpoint.Log("  s" + i + " STEER #" + sid + " is unknown to the DLL (a load came in between, or it was lost) - released, X 14")
        FreeSession(i)
        Return
    EndIf
    Int stPhase = st[0] as Int
    slotSteerPhase[i] = stPhase            ; (FIX ROUND R1 #9: the contact rule reads it - no P 7 while planning or waiting)
    If st[18] > 0.5
        ; the solved spot is the session's point from now on (the danger scan, the strayed rule, DistToSlot, the log)
        slotX[i] = st[8]
        slotY[i] = st[9]
        slotZ[i] = st[10]
        slotFaceDeg[i] = st[11]
    EndIf
    If stPhase == 6
        Int why = st[15] as Int
        If why > 0 && slotSteerWait[i] != why
            slotSteerWait[i] = why
            If why == 1
                ; interrupts 0 (or the player as target) + a hand-grab: the session waits and walks on when the hand lets go
                ; (S1 §2.7, D5) - reported, not ended
                Standpoint.Log("  s" + i + " STEER #" + sid + " WAITS: the player's hand holds her - she walks on when it lets go (Interrupted 3, report only)")
                SendInterrupted(w, 3.0)
            ElseIf why == 2
                Standpoint.Log("  s" + i + " STEER #" + sid + " WAITS: she is in combat - she walks on when it is over (interrupts " + slotInterrupts[i] + ")")
            Else
                ; FIX ROUND (R1 #8): a reaction (ragdoll / knocked / kill move) is waited out like combat, as 1.5.1's package did
                Standpoint.Log("  s" + i + " STEER #" + sid + " WAITS: a reaction (ragdoll / knocked / kill move) - she walks on when it is over")
            EndIf
        EndIf
        SteerBackstop(i, w, afNow)
        Return
    EndIf
    If slotSteerWait[i] > 0
        Standpoint.Log("  s" + i + " STEER #" + sid + ": the wait is over - she walks on")
        slotSteerWait[i] = 0
    EndIf
    If stPhase == 3 || stPhase == 7
        SteerArrive(i, w, sid, st, afNow)  ; (7 = she arrived, her stand ended before the pin: still an arrival - FIX ROUND R1 NIT 14)
        Return
    EndIf
    If stPhase == 4
        SteerEnd(i, w, sid, st)
        Return
    EndIf
    SteerBackstop(i, w, afNow)
EndFunction

; The backstop: the walk's own limit (90 s; beyond 30 m 90 s + d / 150 u/s) answers O 2 first; this catches a session that the
; DLL never ended (S1 §2.7: PollSlot's own 90 s -> a backstop only). A paused game is not counted.
Function SteerBackstop(Int i, Actor w, Float afNow)
    If afNow - slotStart[i] - slotPausedT[i] > slotSteerLimit[i]
        Standpoint.Log("  s" + i + " TIMEOUT (steered, the backstop " + (slotSteerLimit[i] as Int) + " s) at " + PosText(w))
        SendFailed(w, 2.0)
        ApiReport(i, "O", 2)
        FreeSession(i)
    EndIf
EndFunction

; A steered session has ended without an arrival (S1 §2.7): the letter -> the walker's event + the request's answer, then freed.
Function SteerEnd(Int i, Actor w, Int sid, Float[] st)
    If slotFreeing
        If slotFreeing[i]
            Return                         ; (FIX ROUND R1 NIT 12: a release is already freeing her - it sends its own event)
        EndIf
    EndIf
    Int code = st[1] as Int
    Int detail = st[2] as Int
    If code == 8
        EngineFallback(i, w, sid, st)
        Return
    EndIf
    String letter = LetterOf(code)
    Standpoint.Log("  s" + i + " STEER #" + sid + " ENDED " + letter + " " + detail + " - " + (st[3] as Int) + " u from the spot at " + PosText(w))
    ; FIX ROUND (review R1 BLOCKER 2): while the slot is active, EVERY ending without an arrival sends ONE event - a GoTo /
    ; GoToPoint caller (BiSR's MoveToWater, VRCD) listens for nothing else. The move request's letter is unchanged.
    If letter == "O"
        If detail == 1
            SendFailed(w, 1.0)             ; no standable spot
        ElseIf detail == 2
            SendFailed(w, 2.0)             ; not there in time
        ElseIf detail == 15 || detail == 21
            SendFailed(w, 6.0)             ; pushed off / would swim: not an arrival
        Else
            SendFailed(w, 7.0)             ; no way (17; also 18 / 26 when no engine leg was possible)
        EndIf
    ElseIf letter == "P"
        SendInterrupted(w, 3.0)            ; a hand-grab (interrupts 1-2, not toward the player)
    ElseIf slotActive[i]
        SendFailed(w, 3.0)                 ; X (any: released, replaced, her gate - mounted, bleedout, an animation lock ...), B, H:
                                           ; "ended without an arrival" (BiSR maps it to EndSeek; X 6 keeps its meaning on MoveResult)
    EndIf
    If code == 9
        letter = "X"                       ; (H - kept for safety: a stand that ended is answered X 14, as before)
        detail = 14
    EndIf
    ApiReport(i, letter, detail)
    FreeSession(i)
EndFunction

; The planner found no way (S1 §2.8; the user's rule, spec D §0): ONE engine leg to the solved spot - this slot becomes a plain
; engine session from here (its own arrival rules, stall rule and - FIX ROUND - its own fresh 90 s). Not for a close request with no
; route (the DLL answers O 17 then) - but FIX ROUND (review R1 BLOCKER 1): ALSO when she SITS (a chair, a bench, a bed, getting up:
; detail 14), for every kind, close contact too - the steered walk cannot take a sitting NPC; the game's walk stands her up and walks her.
Function EngineFallback(Int i, Actor w, Int sid, Float[] st)
    Float efStamp = slotStart[i]
    Float efX = slotX[i]
    Float efY = slotY[i]
    Float efZ = slotZ[i]
    Float efFace = slotFaceDeg[i]
    If st[18] > 0.5
        efX = st[8]
        efY = st[9]
        efZ = st[10]
        efFace = st[11]
    EndIf
    Int detail = st[2] as Int
    String efWhy = "no route"
    If detail == 18
        efWhy = "blocked, no new route"
    ElseIf detail == 26
        efWhy = "the spot is outside the loaded area"
    ElseIf detail == 14
        efWhy = "she sits - the game's walk stands her up"     ; (FIX ROUND R1 BLOCKER 1: a chair, a bench, a bed, getting up)
    EndIf
    ; v1.6.1 FIX-UP (C4a-2, review R1 § VERIFY 1.6.1): a POINT off the navmesh with no route - the DLL names its NEAREST NAVMESH POINT
    ; ([19] 1, [20..22]): the leg goes there (BiSR's shore point on a rock / in the water: she ends at the shore edge) and its arrival is
    ; measured to the asked point (ArrivalTail: a SHORT arrival)
    Bool efNav = False
    Float efLegX = efX
    Float efLegY = efY
    Float efLegZ = efZ
    If st.Length >= 23
        If st[19] > 0.5
            efNav = True
            efLegX = st[20]
            efLegY = st[21]
            efLegZ = st[22]
        EndIf
    EndIf
    If efNav
        Float efNdx = efLegX - efX
        Float efNdy = efLegY - efY
        Float efNd = Math.Sqrt(efNdx * efNdx + efNdy * efNdy)
        Standpoint.Log("  s" + i + " ENGINE FALLBACK (STEER #" + sid + "): " + efWhy + " and the point (" + (efX as Int) + "," + (efY as Int) + "," + (efZ as Int) + ") is off the navmesh - ONE engine leg to its nearest navmesh point (" + (efLegX as Int) + "," + (efLegY as Int) + "," + (efLegZ as Int) + "), " + (efNd as Int) + " u from it, face=" + (efFace as Int))
    Else
        Standpoint.Log("  s" + i + " ENGINE FALLBACK (STEER #" + sid + "): " + efWhy + " - ONE engine leg to the solved spot (" + (efX as Int) + "," + (efY as Int) + "," + (efZ as Int) + ") face=" + (efFace as Int))
    EndIf
    slotSteer[i] = 0                       ; an engine session from here on (FreeSession / PollSlot treat it as one)
    slotSteerHold[i] = 0
    slotSteerWait[i] = 0
    slotSteerPhase[i] = 2
    ; v1.6.1 (C4; run R3 §3 / §7): the leg is marked - PollSlot ends it when it does not get on (MOVEMENT BLOCKED, or 6 s still) instead
    ; of the 90 s; after NO ROUTE no way round is planned for it (the steered planner has just found none); a sitting walker's leg
    ; keeps its two
    If detail == 14
        slotEngineLeg[i] = 2
    Else
        slotEngineLeg[i] = 1
        slotDetourTries[i] = 2
        If efNav
            slotEngineLeg[i] = 3           ; (FIX-UP C4a-2: to the nearest navmesh point - the arrival is measured to the asked point)
            slotEngAskX[i] = efX
            slotEngAskY[i] = efY
        EndIf
    EndIf
    ; FIX ROUND (R1 NIT 13): the ONE engine leg gets its own 90 s (the steered try's time is not counted against it) - the session's
    ; stamp stays (SameSession), the paused-time account absorbs what has passed
    slotPausedT[i] = Utility.GetCurrentRealTime() - slotStart[i]
    Bool efPlain = slotFinPlain[i]
    EnsureMarker(i, w, efPlain)
    If !SameSession(i, w, efStamp)
        Return
    EndIf
    RouteSend(i, w, efStamp, efLegX, efLegY, efLegZ, efFace, 0)
EndFunction

; A steered arrival (phase 3). S1 §2.5: THE PIN by default; the walk's own stand off the navmesh with pose 0; hold 0 + pose 0:
; she was let go at the point. Then the arrival tail every session shares.
Function SteerArrive(Int i, Actor w, Int sid, Float[] st, Float afNow)
    Float stamp = slotStart[i]
    If slotArrivalBusy[i] == stamp
        Return                             ; (another stack runs this arrival)
    EndIf
    slotArrivalBusy[i] = stamp
    Int detail = st[2] as Int
    Bool isShort = False
    If detail == 9
        isShort = True
    EndIf
    Float d = st[3]
    Float wantDeg = st[11]
    Bool released = st[16] > 0.5
    Bool onNav = st[13] > 0.5
    Bool standEnded = (st[0] as Int) == 7  ; (FIX ROUND R1 NIT 14: she arrived but her stand ended before the pin - still an arrival)
    ; a face target (GoToPoint + SetFaceTarget, VRCD) and she stopped SHORT: toward it (+ the request's orientation), as the
    ; engine path turns a short walker
    If isShort
        ObjectReference fr = slotFaceRef[i]
        If fr && fr != w
            Float fdx = fr.GetPositionX() - w.GetPositionX()
            Float fdy = fr.GetPositionY() - w.GetPositionY()
            If fdx * fdx + fdy * fdy > 1.0
                wantDeg = NormAng360(CompassBearing(fdx, fdy) + slotFaceOffset[i])
            EndIf
        EndIf
    EndIf
    If wantDeg >= 359.95
        wantDeg = 0.0
    EndIf
    If released
        ; hold 0 + pose 0: the walk let her go at the point - nothing to pin (FreeSession: no let-go mid-stride)
        slotSteerHold[i] = 0
        Float angR = NormAng180(w.GetAngleZ() - wantDeg)
        ArrivalTail(i, w, d, angR, isShort, stamp, afNow, " (steered STEER #" + sid + ": let go at the point - hold 0)")
        Return
    EndIf
    ; FIX ROUND (R1 NIT 15): "no pose" = the PLAIN marker of the request (pose 0) and no interaction pose - slotPose holds only 2 / 3,
    ; so a RequestMove pose 1 (the standard wait) off the navmesh is pinned with its wait idle like everywhere else
    Bool noPose = slotPlain[i] && slotPose[i] == 0
    ; v1.6.1 (review R1 § VERIFY N1): a steered POINT (GoToPoint) has no pose either - off the navmesh (BiSR's shore on a rock, in
    ; the water) the walk's own stand keeps her, never a travel package aimed at a marker off the navmesh (S1 §2.5)
    If slotSteerKind[i] == 3
        noPose = True
    EndIf
    If noPose && !onNav && slotHoldSeconds[i] != 0.0
        If standEnded
            ; (her stand already ended: nothing holds her off the navmesh - she arrived and is let go, as with hold 0)
            slotSteerHold[i] = 0
            Float angE = NormAng180(w.GetAngleZ() - wantDeg)
            ArrivalTail(i, w, d, angE, isShort, stamp, afNow, " (steered STEER #" + sid + ": off the navmesh, her stand ended before the pin - let go)")
            If slotWalker[i] == w && slotStart[i] == stamp && slotHolding[i]
                FreeSession(i)
            EndIf
            Return
        EndIf
        ; off the navmesh (water, rock) with no pose: the walk's own stand keeps her (VR-proven: the bathhouse hold) - a travel
        ; package aimed at a marker off the navmesh is untested (S1 §2.5)
        slotSteerHold[i] = 2
        Float angS = NormAng180(w.GetAngleZ() - wantDeg)
        ArrivalTail(i, w, d, angS, isShort, stamp, afNow, " (steered STEER #" + sid + ": held by the walk's stand - off the navmesh)")
        Return
    EndIf
    ; THE PIN (S1 §2.5): the proven short-arrival re-anchor - the marker at her feet with the final heading, the alias, our
    ; package on; then her stand ends quietly and the package holds her (a pose or her own AI never walks her off)
    Bool pinPlain = slotPlain[i]
    ObjectReference mk = EnsureMarker(i, w, pinPlain)
    If !SameSession(i, w, stamp)
        Standpoint.Log("  s" + i + " arrival handling abandoned - the session changed while it waited")
        Return
    EndIf
    If !mk
        slotSteerHold[i] = 2
        Float angM = NormAng180(w.GetAngleZ() - wantDeg)
        ArrivalTail(i, w, d, angM, isShort, stamp, afNow, " (steered STEER #" + sid + ": no marker could be made - held by the walk's stand)")
        Return
    EndIf
    mk.MoveTo(w, 0.0, 0.0, 0.0, False)
    mk.SetAngle(0.0, 0.0, wantDeg)
    If !SameSession(i, w, stamp)
        Standpoint.Log("  s" + i + " arrival handling abandoned - the session changed while it waited")
        Return
    EndIf
    ReferenceAlias pa = GetOwningQuest().GetAlias(i + 1) as ReferenceAlias
    If pa
        pa.ForceRefTo(mk)
    EndIf
    ; FIX ROUND (review R1 #3): ForceRefTo yields - a release on another stack (a grab / leash / danger rule, a consumer's Release)
    ; must not be followed by our package override on a freed NPC
    If !SameSession(i, w, stamp)
        Standpoint.Log("  s" + i + " arrival handling abandoned - the session changed while it waited")
        Return
    EndIf
    ActorUtil.AddPackageOverride(w, slotPkg[i], 95, 1)
    If !SameSession(i, w, stamp)
        ; the session changed during the call: released (the slot is not hers any more) -> our override comes off again; a retask of
        ; hers in this slot owns the slot's package from now on (its own start set it up) -> left alone. v1.6.1 (review R1 § VERIFY
        ; #3): a release IN FLIGHT (FreeSession running on another stack - slotWalker still her) is a release too: its own removal may
        ; already be behind it, so the override comes off here as well (twice is harmless)
        Bool pinOff = slotWalker[i] != w
        If slotFreeing
            If slotFreeing[i]
                pinOff = True
            EndIf
        EndIf
        If pinOff
            ActorUtil.RemovePackageOverride(w, slotPkg[i])
            Standpoint.Log("  s" + i + " arrival handling abandoned - she was released while the pin was set: the override is taken off again")
        Else
            Standpoint.Log("  s" + i + " arrival handling abandoned - the session changed while it waited")
        EndIf
        Return
    EndIf
    slotSteerHold[i] = 1
    Standpoint.SteerHandOff(sid)
    Utility.Wait(0.1)
    If !SameSession(i, w, stamp)
        Standpoint.Log("  s" + i + " arrival handling abandoned - the session changed while it waited")
        Return
    EndIf
    w.EvaluatePackage()
    Bool turnOk = TurnWait(i, w, stamp, 2.5, wantDeg)
    If !turnOk
        Standpoint.Log("  s" + i + " arrival handling abandoned - the session changed while it waited")
        Return
    EndIf
    Float angErr = NormAng180(w.GetAngleZ() - wantDeg)
    String how = ""
    If angErr > 5.0 || angErr < -5.0
        w.SetAngle(0.0, 0.0, wantDeg)
        Utility.Wait(0.5)
        If !SameSession(i, w, stamp)
            Standpoint.Log("  s" + i + " arrival handling abandoned - the session changed while it waited")
            Return
        EndIf
        angErr = NormAng180(w.GetAngleZ() - wantDeg)
        how = ", turned directly (the engine did not)"
    EndIf
    d = DistToSlot(w, i)
    ArrivalTail(i, w, d, angErr, isShort, stamp, afNow, " (steered STEER #" + sid + ": pinned where she stopped" + how + ")")
EndFunction

; The end of an arrival - shared by the engine session's arrival block and the steered arrival (S1 §2.9): the ARRIVED line,
; the hold, Standpoint_Arrived, the move request's C answer, the pose, the hold policy. asHow is appended to the log line.
Function ArrivalTail(Int i, Actor w, Float afDist, Float afAngErr, Bool abShort, Float afStamp, Float afNow, String asHow)
    ; v1.6.1 FIX-UP (C4a-2): her engine leg went to the NEAREST NAVMESH POINT of an asked point off the navmesh - she stands where the
    ; game's walk could take her: a SHORT arrival (C 9), its distance measured to the point the caller asked for
    If slotEngineLeg && slotEngAskX
        If slotEngineLeg[i] == 3
            Float taX = w.GetPositionX() - slotEngAskX[i]
            Float taY = w.GetPositionY() - slotEngAskY[i]
            afDist = Math.Sqrt(taX * taX + taY * taY)
            abShort = True
            asHow = asHow + " (her engine leg went to the nearest navmesh point - the distance is to the asked point)"
        EndIf
    EndIf
    If abShort
        Standpoint.Log("  s" + i + " ARRIVED (short): dist=" + afDist + " angErr=" + afAngErr + " t=" + ((afNow - slotStart[i]) as Int) + "s at " + PosText(w) + asHow)
    Else
        Standpoint.Log("  s" + i + " ARRIVED: dist=" + afDist + " angErr=" + afAngErr + " t=" + ((afNow - slotStart[i]) as Int) + "s at " + PosText(w) + asHow)
    EndIf
    slotActive[i] = False
    slotHolding[i] = True
    Int h = ModEvent.Create("Standpoint_Arrived")
    If h
        ModEvent.PushForm(h, w)
        ModEvent.PushFloat(h, afDist)
        ModEvent.PushFloat(h, afAngErr)
        ModEvent.Send(h)
    EndIf
    ; v0.8.0: a move request is answered AT ARRIVAL (the hold then runs out by itself)
    If abShort
        ApiReport(i, "C", 9)
    Else
        ApiReport(i, "C", 0)
    EndIf
    If s2WadePending && slotWalker[i] == s2Walker
        ExecuteWade(i)
        Return
    EndIf
    If slotPose[i] > 0
        ExecutePose(i)
        ; v0.7.4: ExecutePose waits too (3.2 s). She has arrived, so slotActive is already False here and SameSession
        ; cannot be asked: the walker and the stamp alone say whether the hold policy below still belongs to this session.
        If slotWalker[i] != w || slotStart[i] != afStamp
            Standpoint.Log("  s" + i + " arrival handling abandoned - the session changed while it waited")
            Return
        EndIf
    EndIf
    ; hold policy: -1 = until Release/retask; 0 = free now; N = free after N s
    If slotHoldSeconds[i] >= 0.0
        If slotHoldSeconds[i] < 0.5
            FreeSession(i)
        Else
            slotHoldUntil[i] = Utility.GetCurrentRealTime() + slotHoldSeconds[i]
        EndIf
    EndIf
EndFunction

; SteerState's letter code -> the letter
String Function LetterOf(Int aiCode)
    If aiCode == 1
        Return "C"
    ElseIf aiCode == 2
        Return "P"
    ElseIf aiCode == 3
        Return "O"
    ElseIf aiCode == 4
        Return "D"
    ElseIf aiCode == 5
        Return "L"
    ElseIf aiCode == 7
        Return "B"
    ElseIf aiCode == 8
        Return "F"
    ElseIf aiCode == 9
        Return "H"
    EndIf
    Return "X"
EndFunction

; v1.6.0 BUG 1: slot i's marker of the kind asked (abPlain: base 0x80F, no idle animation; else 0x802), made AT THE WALKER when
; the slot has none yet (until 1.5.1 at the player). None = an older Standpoint.esp without 0x80F gives the standard one.
ObjectReference Function EnsureMarker(Int i, Actor akW, Bool abPlain)
    If abPlain
        If !slotMarkerPlain[i]
            Form emPlain = Game.GetFormFromFile(0x80F, "Standpoint.esp")
            If emPlain
                ObjectReference emNewP = akW.PlaceAtMe(emPlain, 1, True, False)
                If !slotMarkerPlain[i]
                    slotMarkerPlain[i] = emNewP
                EndIf
            EndIf
        EndIf
        If slotMarkerPlain[i]
            Return slotMarkerPlain[i]
        EndIf
    EndIf
    If !slotMarker[i]
        Form emStd = Game.GetFormFromFile(0x802, "Standpoint.esp")
        ObjectReference emNew = akW.PlaceAtMe(emStd, 1, True, False)
        If !slotMarker[i]
            slotMarker[i] = emNew
        EndIf
    EndIf
    Return slotMarker[i]
EndFunction

; v1.6.0 BUG 1: the reference slot i's marker is placed relative to - the session's target (its point lives in the target's
; cell), else the walker
ObjectReference Function AnchorFor(Int i, Actor akW)
    If slotAnchor
        ObjectReference an = slotAnchor[i]
        If an
            If an.GetParentCell()
                Return an
            EndIf
        EndIf
    EndIf
    Return akW
EndFunction

; v1.6.0 BUG 1 (S1 §4): ONE cross-cell move (the engine's own MoveTo, offsets from the anchor) + the heading. SetPosition never
; changes a reference's cell: the 2026-11-06 far runs walked to a marker left inside Lakeview Manor. One call, so the marker
; never stands at her feet for a frame.
Function PlaceMarker(ObjectReference akMk, ObjectReference akAnchor, Float afX, Float afY, Float afZ, Float afFaceDeg)
    akMk.MoveTo(akAnchor, afX - akAnchor.GetPositionX(), afY - akAnchor.GetPositionY(), afZ - akAnchor.GetPositionZ(), False)
    akMk.SetAngle(0.0, 0.0, afFaceDeg)
EndFunction

; v1.6.0 BUG 1: the session line's marker part - where the marker is (cell / worldspace) beside where she is; a [W] line when the
; marker is not in its anchor's space after the move
Function MarkerCellLog(Int i, ObjectReference akMk, ObjectReference akAnchor, Actor akW)
    Cell mc = akMk.GetParentCell()
    Cell wc = akW.GetParentCell()
    Standpoint.Log("session " + i + ": marker " + akMk + " in " + mc + " / " + akMk.GetWorldSpace() + "; she is in " + wc + " / " + akW.GetWorldSpace())
    Cell ac = akAnchor.GetParentCell()
    Bool sameSpace = False
    If mc && ac
        If mc.IsInterior() || ac.IsInterior()
            sameSpace = mc == ac
        Else
            sameSpace = akMk.GetWorldSpace() == akAnchor.GetWorldSpace()
        EndIf
    EndIf
    If !sameSpace
        Standpoint.Log("[W] session " + i + ": the marker " + akMk + " is NOT in the space of " + akAnchor + " after the move (" + mc + " / " + ac + ")")
    EndIf
EndFunction

; v1.6.0 the steered-session arrays (Setup makes them new at every load: abNew; on an older save a call can come in before
; OnPlayerLoadGame's Setup ran - then they are made here, once)
Function EnsureSteerArrays(Bool abNew = False)
    If abNew || !slotSteer
        slotSteer = new Int[4]
        slotSteerHold = new Int[4]
        slotSteerWait = new Int[4]
        slotSteerLimit = new Float[4]
        slotAnchor = new ObjectReference[4]
        slotSteerOld = new Int[4]
        slotSteerPhase = new Int[4]
        slotSteerKind = new Int[4]
        slotEngineLeg = new Int[4]
        slotEngAskX = new Float[4]
        slotEngAskY = new Float[4]
    EndIf
    If !slotSteerPhase
        slotSteerPhase = new Int[4]        ; (FIX ROUND: a save made with the first 1.6.0 build has the arrays above but not this one)
    EndIf
    If !slotSteerKind
        slotSteerKind = new Int[4]         ; (v1.6.1: a save made with 1.6.0 has the arrays above but not these)
    EndIf
    If !slotEngineLeg
        slotEngineLeg = new Int[4]
    EndIf
    If !slotEngAskX
        slotEngAskX = new Float[4]
        slotEngAskY = new Float[4]
    EndIf
EndFunction

; v0.9.3 THE PAIR RULE for a close request to an NPC (refactored in 1.6.0 so the engine path and the steered path share it - the
; code is unchanged). For an NPC target (not the player, not herself) the walker's and the target's character controllers ignore
; each other for as long as this session lives, so her body can overlap the target's (measured in the second run: 41 cm front,
; 46 cm right, the target not moved - report 05 §12.2). FreeSession / a retask release it; the DLL switches it off only once the
; two are 90 u apart. KNOWN LIMIT: the rule only bites for a pair whose bodies START to overlap after it is armed.
Function PairArmFor(Int i, Actor w, ObjectReference t, Float afStamp)
    Actor tA = t as Actor
    Bool pairWanted = False
    If tA
        If tA != Game.GetPlayer() && tA != w
            pairWanted = True
        EndIf
    EndIf
    If pairWanted
        Int newPairH = Standpoint.PairArm(w, tA)
        If newPairH > 0
            ; v0.9.4: the rule must have an owner. If this slot no longer holds the session this request started
            ; (released or retasked while the rule was being armed), nobody would ever release the handle:
            ; it is given back at once and not stored.
            ; v0.9.5: a release IN FLIGHT counts as "not mine any more" too (slotFreeing - see FreeSession): its
            ; PairRelease step may already be behind it.
            Bool stillMine = False
            If slotWalker[i] == w && slotStart[i] == afStamp
                stillMine = True
                If slotFreeing
                    If slotFreeing[i]
                        stillMine = False
                    EndIf
                EndIf
            EndIf
            If stillMine
                Int stalePairH = slotPairH[i]
                slotPairH[i] = newPairH
                If stalePairH > 0
                    Standpoint.PairRelease(stalePairH)     ; (never expected: StartSession emptied it)
                EndIf
                Standpoint.Log("session " + i + ": close - the PAIR RULE is armed for " + w + " and " + tA + " (their controllers ignore each other; handle " + newPairH + ")")
            Else
                Standpoint.PairRelease(newPairH)
                Standpoint.Log("session " + i + ": close - the session changed while the pair rule was being armed: the rule is given back at once (handle " + newPairH + ")")
            EndIf
        Else
            Standpoint.Log("session " + i + ": close - the pair rule is NOT armed (no HIGGS, a controller could not be read, or the pool is full): normal collision")
        EndIf
    Else
        Standpoint.Log("session " + i + ": close - no pair rule (the target is the player, or not an actor): normal collision, no stops, no second walk in")
    EndIf
EndFunction

; ====================== v1.6.0: THE stpm TEST's EAR (TEST USE ONLY) ======================
; The stpm console test (TestCmd.cpp) drives the PUBLIC Papyrus doors and must hear every Standpoint_* event a consumer would
; hear. While it runs this alias is registered for them (StandpointAPI.ApiTestListen) and hands each one to the DLL
; (Standpoint.TestEvent: one "[TEST] MIX event" line, and the test's own bookkeeping). Setup switches it off at every load.
Function ApiTestListen(Bool abOn)
    If abOn
        RegisterForModEvent("Standpoint_Arrived", "OnStpmArrived")
        RegisterForModEvent("Standpoint_Failed", "OnStpmFailed")
        RegisterForModEvent("Standpoint_Interrupted", "OnStpmInterrupted")
        RegisterForModEvent("Standpoint_MoveResult", "OnStpmMoveResult")
        RegisterForModEvent("Standpoint_WalkResult", "OnStpmWalkResult")
        RegisterForModEvent("Standpoint_DirectWalk", "OnStpmDirectWalk")
    Else
        UnregisterForModEvent("Standpoint_Arrived")
        UnregisterForModEvent("Standpoint_Failed")
        UnregisterForModEvent("Standpoint_Interrupted")
        UnregisterForModEvent("Standpoint_MoveResult")
        UnregisterForModEvent("Standpoint_WalkResult")
        UnregisterForModEvent("Standpoint_DirectWalk")
    EndIf
EndFunction

Event OnStpmArrived(Form akWalker, Float afDist, Float afAngErr)
    Standpoint.TestEvent("Arrived", akWalker, 0, "", 0, afDist, afAngErr, 0.0)
EndEvent

Event OnStpmFailed(Form akWalker, Float afCode)
    Int fCode = afCode as Int
    Standpoint.TestEvent("Failed", akWalker, 0, "", fCode, afCode, 0.0, 0.0)
EndEvent

Event OnStpmInterrupted(Form akWalker, Float afCode)
    Int iCode = afCode as Int
    Standpoint.TestEvent("Interrupted", akWalker, 0, "", iCode, afCode, 0.0, 0.0)
EndEvent

Event OnStpmMoveResult(Form akWho, Int aiHandle, String asOutcome, Int aiDetail, Float afDistCm, Float afFacingHour, Float afOrientHour, Form akWhere)
    Standpoint.TestEvent("MoveResult", akWho, aiHandle, asOutcome, aiDetail, afDistCm, afFacingHour, afOrientHour)
EndEvent

Event OnStpmWalkResult(Form akWho, Int aiHandle, String asOutcome, Int aiDetail, Float afDistLeft, Float afSubmerged)
    Standpoint.TestEvent("WalkResult", akWho, aiHandle, asOutcome, aiDetail, afDistLeft, afSubmerged, 0.0)
EndEvent

Event OnStpmDirectWalk(String asEventName, String asWhat, Float afWalkHandle, Form akWho)
    Int dwHandle = afWalkHandle as Int
    Standpoint.TestEvent("DirectWalk", akWho, dwHandle, asWhat, 0, afWalkHandle, 0.0, 0.0)
EndEvent

; v0.9.7 / v0.9.8: the last six parameters are a close move request's ROUTE (ApiRunRequest): akRouteTarget = the NPC
; she walks to, (afFinX, afFinY, afFinZ, afFinFace) the asked point; aiRoute 1 / 2 / 3 = (afX, afY, afZ) is a STOP on the
; way (her step back, a point on the way around, the approach point), aiRoute 0 = (afX, afY, afZ) is the asked point
; itself. Left at their defaults (every other caller) the session is exactly what it was.
; v1.5.1: aiFarWalk > 0 = THE FAR PHASE of that walk (ApiFarStart): her walk is NOT stopped (the session is part of it), the DLL
; sets the slot's package to RUN right before she takes it up (Standpoint.WalkFarPackage), and PollSlot leaves the session to
; the DLL. 0 (every other caller) = exactly as before; any start resets the slot's far mark (a GoTo retasking a far session
; makes it an ordinary one - that walk answers X 6, as any walk of hers would).
; v1.6.0: akAnchor (BUG 1) = the reference the marker is placed relative to - a relative session's TARGET (None = the walker);
; abSteered = a STEERED session (S1 §2.9): no marker, no alias, no package and no WalkStop here - her older walk is replaced by the
; DLL's own walk (which keeps her held until it takes her); the slot is marked "starting" (slotSteer -1) and the CALLER asks the
; DLL (SteerBegin) once it has written the slot's fields. A held walker's pin (our package) comes off without the let-go wait.
Int Function StartSession(Actor akWalker, Float afX, Float afY, Float afZ, Float afFaceDeg, Float afHoldSeconds = -1.0, Bool abPlain = False, Bool abClose = False, Int aiRoute = 0, Float afFinX = 0.0, Float afFinY = 0.0, Float afFinZ = 0.0, Float afFinFace = 0.0, ObjectReference akRouteTarget = None, Int aiFarWalk = 0, ObjectReference akAnchor = None, Bool abSteered = False)
    If !engineReady
        Setup()
        If !engineReady
            Return 0
        EndIf
    EndIf
    Int i = FindSlot(akWalker)      ; retask an existing session
    Bool wasHeld = False            ; v0.9.6: the retask of a walker who is HELD on her marker (read before anything is reset)
    If i < 0
        i = FreeSlot()
    ElseIf slotHolding[i]
        wasHeld = True
    EndIf
    If i < 0
        Standpoint.Log("GoTo: all 4 slots busy")
        Return 0
    EndIf

    slotWalker[i] = akWalker
    ; v0.7.4: stamp the session FIRST. A retask WITHOUT Release (VRCD re-solves when its target moves) reuses this slot while the
    ; previous session's arrival handling may still sit in one of its waits; from this line on its SameSession check fails, before
    ; the marker is moved below. (The real start time is written again at the end, as before.)
    slotStart[i] = Utility.GetCurrentRealTime()
    Float startStamp = slotStart[i]     ; v0.9.6: this start's own stamp, asked again after the let-go wait below
    ; v0.9.8 (the review): the slot is NOT ACTIVE while it is being re-started (it goes active again further down, in
    ; one piece with the new point). A retask of a walker who is still walking used to leave it active under the first
    ; stamp with the OLD session's close / route / retry fields: a poll in that window could enter the arrival handling
    ; and take her package off after the new one had been put on. Now no poll starts, and every stale check fails.
    slotActive[i] = False
    ; v0.8.0: a move request this slot still owed an answer to is REPLACED by this session (a retask) -> it answers X
    EnsureApiArrays()
    ; v0.9.7: a new session starts with no route (a retask of a walker who was on one: her old route ends here, before
    ; anything below lets another stack run). A new route (aiRoute) is written further down, in one piece with her point.
    slotRoute[i] = 0
    slotRouteLegs[i] = 0
    ; v1.4.0: a walk off the navmesh of hers (StandpointAPI.WalkTo, C++ RequestWalk) ends here - it answers X 6 (replaced);
    ; the DLL hands her movement back at once. A session and a walk never own her at the same time. (The 1.4.0 review: HERE,
    ; not right after the stamp - a native call lets another stack run, and the slot must be inactive with its route reset
    ; before that can happen.)
    ; v1.5.1: the far mark goes down first (still no native call since slotActive above); a far start's own walk is not stopped
    EnsureFarArray()
    slotFar[i] = 0
    ; v1.6.0: the steered mark too, in the same piece (-1 = starting: PollSlot waits for SteerBegin). Her live steered session
    ; (or the one a start still in progress was replacing) is carried in slotSteerOld until the DLL has superseded it
    EnsureSteerArrays()
    Int oldSteer = slotSteer[i]
    Int carriedSteer = 0
    If oldSteer > 0
        carriedSteer = oldSteer
    ElseIf oldSteer < 0
        carriedSteer = slotSteerOld[i]
    EndIf
    slotSteer[i] = 0
    slotSteerOld[i] = 0
    slotSteerHold[i] = 0
    slotSteerWait[i] = 0
    slotSteerPhase[i] = 0
    slotSteerKind[i] = 0                    ; v1.6.1 (N1: the steered caller writes its kind; C4: no engine leg yet)
    slotEngineLeg[i] = 0
    If abSteered
        slotSteer[i] = -1
        slotSteerOld[i] = carriedSteer      ; (SteerStart replaces it in the DLL - her walk keeps her held meanwhile)
    EndIf
    If aiFarWalk <= 0 && !abSteered
        Standpoint.WalkStop(akWalker, 6)
    EndIf
    If carriedSteer > 0 && !abSteered
        Standpoint.SteerStop(carriedSteer)  ; v1.6.0: an engine session takes the slot - the old steered session ends in the DLL
    EndIf
    ApiReport(i, "X", 6)
    ; v0.9.3: a retask of a close session - the old session's pair rule is released first ("off once the two are apart").
    ; If the new session is a close one for the same two, ApiRunRequest arms it again and the DLL simply keeps the entry
    ; (v0.9.4: a pool with use counts - the release only orders a look one second later, the new arm cancels it: no gap).
    Int oldPairH = slotPairH[i]
    If oldPairH > 0
        slotPairH[i] = 0
        Standpoint.PairRelease(oldPairH)
    EndIf
    ; v0.9.6: A HELD WALKER IS LET GO BEFORE SHE IS RETASKED.
    ; Measured 2026-10-10 00:24-00:29 (the third close-contact run, report 05 §14): a walker pinned on her idle marker
    ; (arrived, holding) did NOT walk when the marker was moved and the same package override was removed and added
    ; again in one go. She stood in her pose - package ours, procedure 1 - for 90 s (step 3) or until the stall rule took
    ; her where she stood (5 more steps). "Calling again retasks her" was true only for a walker who was still walking.
    ; So, ONLY for a slot that was HOLDING when this retask came in: what FreeSession does to end the pose - the package
    ; override off, IdleForceDefaultState, EvaluatePackage - then one second for the engine to really drop the pose, and
    ; only then the usual sequence below (marker, alias, override on, evaluate).
    ; The 1.0 s is a FIRST VALUE, not a measurement. The alias and the slot are NOT cleared here (the slot stays hers);
    ; a retask of a walker who is still WALKING does not come through here at all.
    ; The stamp was written above, so a handler of the old session gives up during this wait. If the slot is released
    ; or taken by yet another start during the wait, THIS start is abandoned (0): it must not go on building a session
    ; in a slot that is no longer its own.
    ; v0.9.7: the let-go itself is now the function LetGo (below) - the same three calls and the same 1.0 s, plus up to
    ; 2 s more for as long as the engine still reports OUR package as her current one. RoutePoll uses it too, for a
    ; walker who stands still on her route.
    If wasHeld && !abSteered
        Standpoint.Log("session " + i + ": retask from a held pose: let go first")
        slotHolding[i] = False          ; the hold is over: no hold-expiry / leash release of this slot during the wait
        Bool letGoOk = LetGo(i, akWalker, startStamp)
        If !letGoOk
            Standpoint.Log("session " + i + ": released or taken over while she was being let go - this start is abandoned")
            Return 0
        EndIf
    EndIf
    If afFaceDeg >= 359.95
        afFaceDeg = 0.0
    EndIf
    Float steerDist = 0.0                   ; v1.6.0: her flat distance to the point (a steered session's backstop limit)
    If abSteered
        ; v1.6.0 (S1 §2.9): NO marker, alias or package - the DLL walks her. An engine session of hers in this slot (a retask, a
        ; held pin) loses its package and alias here, WITHOUT the let-go wait or IdleForceDefaultState: the DLL's walk takes her
        ; (and breaks an idle pose itself) and holds her while her route is planned.
        If wasHeld
            Standpoint.Log("session " + i + ": steered retask from a held pose - the pin comes off, the DLL's walk takes her")
            slotHolding[i] = False
        EndIf
        ActorUtil.RemovePackageOverride(akWalker, slotPkg[i])
        ReferenceAlias sa = GetOwningQuest().GetAlias(i + 1) as ReferenceAlias
        If sa
            sa.Clear()
        EndIf
        If !StillThisStart(i, akWalker, startStamp)
            Standpoint.Log("session " + i + ": released or taken over while it was being started - this start is abandoned")
            Return 0
        EndIf
        Float steerDx = afX - akWalker.GetPositionX()
        Float steerDy = afY - akWalker.GetPositionY()
        steerDist = Math.Sqrt(steerDx * steerDx + steerDy * steerDy)
        slotPlain[i] = abPlain              ; (the pin's marker after the arrival: EnsureMarker makes it then)
    Else
        ; v1.6.0 BUG 1 (S1 §4): the marker is placed by ONE cross-cell move relative to its anchor - the session's TARGET for a
        ; relative session (its point lives in the target's cell; a walker in another cell is walked there through the door, as
        ; the engine does), else the walker. Markers are made AT THE WALKER (until 1.5.1 at the player).
        ObjectReference ssAnchor = akWalker
        If akAnchor
            If akAnchor.GetParentCell()
                ssAnchor = akAnchor
            EndIf
        EndIf
        If !slotMarker[i]
            Form idlmBase = Game.GetFormFromFile(0x802, "Standpoint.esp")
            slotMarker[i] = akWalker.PlaceAtMe(idlmBase, 1, True, False)
        EndIf
        ; v0.8.0: pose 0 (a plain stand) parks her on the slot's SECOND marker, whose base (0x80F) carries no idle animation.
        ; An older Standpoint.esp has no 0x80F: she then gets the standard marker (hands behind the back), and the log says so.
        slotPlain[i] = False
        ; v0.9.8: a STOP on the way (aiRoute > 0) is always walked to on the plain marker - no wait pose there
        If abPlain || aiRoute > 0
            If !slotMarkerPlain[i]
                Form idlmPlain = Game.GetFormFromFile(0x80F, "Standpoint.esp")
                If idlmPlain
                    slotMarkerPlain[i] = akWalker.PlaceAtMe(idlmPlain, 1, True, False)
                EndIf
            EndIf
            If slotMarkerPlain[i]
                slotPlain[i] = True
            Else
                Standpoint.Log("session " + i + ": no plain marker (Standpoint.esp has no 0x80F) - the standard wait marker is used")
            EndIf
        EndIf
        ObjectReference mk = CurMarker(i)
        PlaceMarker(mk, ssAnchor, afX, afY, afZ, afFaceDeg)
        MarkerCellLog(i, mk, ssAnchor, akWalker)

        ReferenceAlias a = GetOwningQuest().GetAlias(i + 1) as ReferenceAlias
        a.ForceRefTo(mk)
        ActorUtil.RemovePackageOverride(akWalker, slotPkg[i])
        If aiFarWalk > 0
            ; v1.5.1: the DLL sets this slot's package to RUN (its preferred speed) BEFORE she takes it up below
            Standpoint.WalkFarPackage(aiFarWalk, i, slotPkg[i])
        EndIf
        ActorUtil.AddPackageOverride(akWalker, slotPkg[i], 95, 1)
        Utility.Wait(0.1)
        akWalker.EvaluatePackage()
    EndIf

    If saAvailable
        SeverActionsNativeExt.Travel_CancelByActor(akWalker)
        ; v0.9.1: a CLOSE session does NOT start SeverActions' stuck recovery. Measured 2026-10-09 19:14 (report 05 §10): a
        ; close walker stood still in body contact with her target, 40.6 u from her point - just outside SA's 40 u
        ; "arrived" radius. SA called her stuck: a nudge after 4.5 s, then "Level 2 leapfrog ... dist=500 moved=496u"
        ; after 7.5 s - 500 u straight ahead, through the target and the wall, into the void under the cell.
        ; Standing still next to the target is how a close walk ENDS; the driver's own stall rule handles it.
        ; v0.9.2 - the user's ruling, 2026-10-09: "drop it, teleportation is never good. Yes, NPC get stuck, but
        ; teleporting is a world of bug, including what just happen just now". NO Standpoint walk starts SeverActions'
        ; stuck recovery any more (it was `Travel_BeginXY(akWalker, afX, afY, afZ, None, 40.0, "standpoint_" + i, 12, 90)`
        ; right here). A walker who gets stuck is moved by nothing: she stays where she is until the 90 s limit
        ; (Standpoint_Failed 2) or, if she stands still near the point, until the short-arrival handling takes her.
        ; The cancel above stays: a SeverActions journey that somebody else started for her would call her "stuck"
        ; while she walks for us - and leap her.
    EndIf

    If !slotFaceRef
        slotFaceRef = new ObjectReference[4]   ; v0.7.3 on an older save, before OnPlayerLoadGame's Setup ran
    EndIf
    Float startReal = Utility.GetCurrentRealTime()     ; the session's final stamp, read BEFORE the one-piece block below
    ; v0.9.7: the route is written in ONE piece with the point itself AND the final stamp - no native call from here to
    ; slotStart below, so no other stack can see the waypoint without its route, or the route under the first stamp
    ; (a retask of a walker who is still walking is polled all the time). For a new or a held walker this is also
    ; before the slot becomes active (slotActive, below). The two stall flags go down here too, so a poll that slips
    ; in before the rest of the resets cannot read the OLD session's stall clock.
    slotFinPlain[i] = abPlain
    slotRetry[i] = 0
    slotRouteFails[i] = 0
    slotBlockPolls[i] = 0
    slotRouteOk[i] = False
    slotRoute[i] = aiRoute                  ; v1.3.0 (the review): for EVERY session, in this one piece (a retask must never
                                            ; inherit a detour's state 4 written by a stack that ran during its early part)
    If akRouteTarget
        slotFinX[i] = afFinX
        slotFinY[i] = afFinY
        slotFinZ[i] = afFinZ
        slotFinFace[i] = afFinFace
        slotFaceRef[i] = akRouteTarget
        slotRouteOk[i] = True
        slotRouteLegs[i] = 0
        If aiRoute > 0
            slotRouteLegs[i] = 1
        EndIf
        slotRoute[i] = aiRoute
    EndIf
    slotSeenOurs[i] = False
    slotHasMoved[i] = False
    slotX[i] = afX
    slotY[i] = afY
    slotZ[i] = afZ
    slotFaceDeg[i] = afFaceDeg
    slotStart[i] = startReal
    slotLastIdx[i] = -99
    slotSawTravel[i] = False
    slotFar[i] = aiFarWalk                  ; v1.5.1: in the one piece too (before the slot goes active)
    slotAnchor[i] = akAnchor                ; v1.6.0 BUG 1: later legs (RouteSend) place the marker relative to it too
    slotSteerLimit[i] = 95.0                ; v1.6.0: a steered session's backstop - the walk's own 90 s answers first ...
    If steerDist > 2100.0
        slotSteerLimit[i] = 95.0 + steerDist / 150.0     ; ... beyond 30 m 90 s + d / 150 u/s (the walk's own rule)
    EndIf
    slotActive[i] = True
    slotHolding[i] = False
    slotHoldSeconds[i] = afHoldSeconds
    slotHoldUntil[i] = 0.0
    slotPose[i] = 0
    slotDestZ[i] = 0.0
    slotInterrupts[i] = 1
    slotDestIsPlayer[i] = False
    slotCombatSeen[i] = False
    slotContactPolls[i] = 0
    slotDanger[i] = 2
    slotLeash[i] = 3500.0
    slotFaceRef[i] = akRouteTarget         ; None for every session without a route (v0.9.7: a route's target was written above)
    slotFaceOffset[i] = 0.0
    slotClose[i] = 0
    slotStallX[i] = akWalker.GetPositionX()
    slotStallY[i] = akWalker.GetPositionY()
    slotStallT[i] = slotStart[i]
    slotSeenOurs[i] = False
    slotHasMoved[i] = False
    EnsureV12Arrays(False)                   ; v1.2.0 - an older save, before OnPlayerLoadGame's Setup ran
    slotLegKicks[i] = 0                      ; v1.3.0
    Int oldPathH = slotPathH[i]              ; v1.3.0: the slot's old session may have left a plan running
    slotPathH[i] = 0
    If oldPathH > 0
        Standpoint.PathCancel(oldPathH)
    EndIf
    slotDetour[i] = 0
    slotDetourTries[i] = 0
    slotWpN[i] = 0
    slotWpK[i] = 0
    slotLastPollT[i] = 0.0
    slotPausedT[i] = 0.0
    slotLastPX[i] = akWalker.GetPositionX()
    slotLastPY[i] = akWalker.GetPositionY()
    ; v1.2.0: where she starts from is logged too (report 11 §10: the stuck positions had to be guessed)
    String steerTxt = ""
    If abSteered
        steerTxt = " - STEERED (the DLL solves the spot and walks her; the point above is the target / the asked point)"
    EndIf
    Standpoint.Log("session " + i + ": " + akWalker + " -> (" + (afX as Int) + "," + (afY as Int) + "," + (afZ as Int) + ") face=" + (afFaceDeg as Int) + " from " + PosText(akWalker) + steerTxt)
    RegisterForSingleUpdate(0.5)
    Return i + 1
EndFunction

Bool Function ApiRelease(Actor akWalker)
    Int i = FindSlot(akWalker)
    If i < 0
        Return False
    EndIf
    ; FIX ROUND (review R1 NIT 12): a release already in flight (another stack's FreeSession - SteerEnd's, PollSlot's - yields before
    ; slotWalker is cleared) has sent its event: no second Failed 3, no second FreeSession
    If slotFreeing
        If slotFreeing[i]
            Return True
        EndIf
    EndIf
    ; v1.5.1: a walk's FAR session is let go the DLL's way - never IdleForceDefaultState on a body that is ragdolled, knocked,
    ; in a kill move, bleeding out, fighting, in furniture, mounted, or held by a HIGGS hand ("the moonwalk"): EvaluatePackage only
    Int relMode = 0
    If FarWalkOf(i) > 0
        relMode = Standpoint.WalkReleaseMode(akWalker)
        If slotWalker[i] != akWalker
            Return True                 ; (another stack freed or retasked the slot during the native call: hers is gone)
        EndIf
    EndIf
    If slotActive[i] && FarWalkOf(i) == 0      ; v1.5.1: a walk's far session sends no Standpoint_Failed (its walk answers)
        SendFailed(akWalker, 3.0)
    EndIf
    ApiReport(i, "X", 3)
    FreeSession(i, relMode)
    Return True
EndFunction

; ====================== v1.5.1: THE FAR PHASE OF A WALK (driver side) ==================
; A walk beyond 30 m (StandpointAPI.WalkTo / WalkToWater, C++ RequestWalk / RequestWalkToWater, stpf) is first run by the
; engine's own travel: the DLL asks for a PLAIN session toward the point (or its nearest navmesh point) through
; StandpointAPI.ApiWalkFarStart; StartSession lets the DLL set the slot's package to RUN before she takes it up
; (Standpoint.WalkFarPackage). The DLL watches her (distance, progress, the gate, the walk's limit) and ends the session itself
; through ApiWalkFarEnd - at the hand-over to the direct walk WITHOUT the let-go (she keeps moving). PollSlot leaves such a
; session alone, except that it frees one whose walk does not want it any more (Standpoint.WalkFarAlive).
Int Function ApiFarStart(Actor akWho, Float afX, Float afY, Float afZ, Float afFaceDeg, Int aiWalk)
    Int h = StartSession(akWho, afX, afY, afZ, afFaceDeg, -1.0, True, False, 0, 0.0, 0.0, 0.0, 0.0, None, aiWalk)
    If h > 0
        Standpoint.Log("session " + (h - 1) + ": the FAR phase of walk #" + aiWalk + " - the DLL watches her and ends this session itself")
    Else
        Standpoint.Log("the FAR phase of walk #" + aiWalk + ": no session (all 4 slots busy, or the engine is not ready)")
    EndIf
    Return h
EndFunction

; The DLL ends walk aiWalk's far session. The mode is decided again HERE, when it runs (Standpoint.WalkFarEndMode - review R2
; B2 / R2 / R3): 1 = a walk of hers goes on (the direct walk after the hand-over, or a newer walk): the override comes off WITHOUT
; IdleForceDefaultState / EvaluatePackage (she keeps moving; that walk evaluates her package later); 2 = override off +
; EvaluatePackage (no walk of hers any more - a late end never leaves a stale travel package - or she is in a reaction / a hand
; holds her); 0 = the full let-go. Returns the mode applied; -1 = nothing (the slot holds another session by now: a GoTo /
; RequestMove retasked it, or it was freed).
Int Function ApiFarEnd(Actor akWho, Int aiWalk, Int aiMode)
    Int i = FindSlot(akWho)
    If i < 0
        Return -1
    EndIf
    If FarWalkOf(i) != aiWalk
        Return -1
    EndIf
    Int endMode = Standpoint.WalkFarEndMode(aiWalk, akWho, aiMode)
    If FarWalkOf(i) != aiWalk
        Return -1                       ; (another stack took or freed the slot during the native call)
    EndIf
    If endMode == 1
        Standpoint.Log("session " + i + ": the FAR phase of walk #" + aiWalk + " is over - a walk of hers has her: the package override comes off WITHOUT the let-go")
    EndIf
    FreeSession(i, endMode)
    Return endMode
EndFunction

; True when her session is the far phase of one of her walks (StandpointAPI's WalkTo / WalkToWater / ApiWalkFreeSession leave it
; to that walk: a new walk replaces the old one, X 6, and the old one frees it).
Bool Function ApiIsFarSession(Actor akWho)
    If !akWho
        Return False
    EndIf
    Int i = FindSlot(akWho)
    If i < 0
        Return False
    EndIf
    Return FarWalkOf(i) > 0
EndFunction

; the walk whose far session slot i is (0 = none; None-safe on an older save before OnPlayerLoadGame's Setup ran)
Int Function FarWalkOf(Int i)
    If !slotFar
        Return 0
    EndIf
    Return slotFar[i]
EndFunction

Function EnsureFarArray()
    If !slotFar
        slotFar = new Int[4]
    EndIf
EndFunction

; ====================== v0.8.0: THE MOVE API (driver side) ==================
; A C++ plugin's 10-slot request line (IStandpoint2::RequestMove, API/StandpointAPI.h) arrives here through
; StandpointAPI.ApiWake. The line is read from the DLL's mailbox, solved by the clock solver and walked by the SAME
; session engine as GoTo; every ending of the session names its letter through ApiReport. Detail codes = MoveDetail.
Function ApiRunRequest(Int aiId)
    Actor w = Standpoint.ApiReqWho(aiId)
    Float[] v = Standpoint.ApiReqValues(aiId)
    If !w || v.Length < 6
        Standpoint.ApiPostResult(aiId, "O", 10)
        Return
    EndIf
    If v[5] > 0.5
        ; a release line (where = 0): her running request answers X (ApiRelease), then the release line itself
        If ApiRelease(w)
            Standpoint.ApiPostResult(aiId, "X", 3)
        Else
            Standpoint.ApiPostResult(aiId, "X", 13)
        EndIf
        Return
    EndIf
    ObjectReference t = Standpoint.ApiReqWhere(aiId)
    If !t
        Standpoint.ApiPostResult(aiId, "O", 10)
        Return
    EndIf
    ; v0.9.3: where = who -> HOLD YOUR PLACE. She is pinned where she stands, facing the way she faces, for the asked
    ; hold time. No solve, never close mode, no pair rule, no face target. She "arrives" at once through the normal
    ; arrival block (C, detail 0, about 3 s later). Why: a close request is solved against where its target stands at
    ; that moment, and a free target walks away (Carmella went 214 u during the second run - report 05 §12.4).
    Actor tSelf = t as Actor
    If tSelf == w
        Int poseHold = v[3] as Int
        Bool plainHold = False
        If poseHold == 0
            plainHold = True
        EndIf
        Float holdHold = v[4]
        Int hHold = StartSession(w, w.GetPositionX(), w.GetPositionY(), w.GetPositionZ(), w.GetAngleZ(), holdHold, plainHold, False)
        If hHold == 0
            Standpoint.ApiPostResult(aiId, "B", 11)
            Return
        EndIf
        Int iHold = hHold - 1
        If poseHold == 2 || poseHold == 3
            slotPose[iHold] = poseHold
        EndIf
        slotDestZ[iHold] = w.GetPositionZ()
        slotApiId[iHold] = aiId
        Standpoint.Log("session " + iHold + " answers move request #" + aiId + " (where = who: she HOLDS her place, pose " + poseHold + ", hold " + holdHold + " s)")
        Return
    EndIf
    ; v0.9.0: slot 8 of the request line. 1 = CLOSE CONTACT (an actor target has no distance floor; a stall near her is
    ; her arrival; the player standing near does not cancel the walk). An older DLL sends six values: close stays 0.
    Int close = 0
    If v.Length >= 7
        If v[6] > 0.5
            close = 1
        EndIf
    EndIf
    ; v1.6.0 (S1 §2; spec D item 4): Skyrim VR with [Session] Steered = 1 - THE STEERED SESSION: the DLL solves the hour by
    ; physics (the clock solver's own ring), plans one route (round the target, the last stretch straight in) and walks her; no
    ; SolveClock / RouteStage here. A target the steered walk cannot reach (another interior / worldspace) keeps the engine path.
    If SteeredOn()
        Bool rqSteer = Standpoint.SteerCheck(w, t, 0.0, 0.0, 0.0)
        If rqSteer
            RunSteered(aiId, w, t, v, close)
            Return
        EndIf
    EndIf
    Float[] s = Standpoint.SolveClock(t, w, v[0], v[1], v[2], close)
    If s.Length < 7
        SendFailed(w, 1.0)
        Standpoint.ApiPostResult(aiId, "O", 1)
        Return
    EndIf
    Int pose = v[3] as Int
    Bool plain = False
    If pose == 0
        plain = True
    EndIf
    Float hold = v[4]
    Bool closeB = False
    If close == 1
        closeB = True
    EndIf
    ; ---- v0.9.7 / v0.9.8: STEP BACK AND AROUND, and NO GLIDE (the user's rulings of 2026-10-10: "yes, have standpoint
    ; do the step back and around itself" and, of the glide, "Let's try not to do that again"; report 05 §14.4, §15-§17) ----
    ; A close request to an NPC (not the player, not herself) is walked - every step of it. RouteStage (below) says
    ; whether she can walk to the asked point from where she stands, or needs a STOP first: her step back from the
    ; target, a point on the way around it, or the approach point the walk in starts from. The session then starts
    ; toward that stop; the asked point waits in slotFin*, and RoutePoll (called by PollSlot) chooses every later leg
    ; from where she and her target really stand. The request still answers once, at the end.
    ; She is asked nothing when she already stands on the asked point (within 8 u - the arrival block's own tolerance).
    Bool rtNpc = False
    If close == 1
        Actor rtActor = t as Actor
        If rtActor
            If rtActor != Game.GetPlayer() && rtActor != w
                rtNpc = True
            EndIf
        EndIf
    EndIf
    Float[] rtFirst                 ; her first stop, when she needs one
    Int rtState = 0                 ; its kind (1 step back, 2 around, 3 approach), 0 = none
    If rtNpc
        Float rtGapX = w.GetPositionX() - s[0]
        Float rtGapY = w.GetPositionY() - s[1]
        Float rtGap = Math.Sqrt(rtGapX * rtGapX + rtGapY * rtGapY)
        If rtGap > 8.0
            rtFirst = RouteStage(t, w, s[0], s[1], False)
            If rtFirst.Length >= 5
                rtState = rtFirst[4] as Int
                Standpoint.Log("ROUTE for move request #" + aiId + ": she stands " + (rtGap as Int) + " u from the asked point (" + (s[0] as Int) + "," + (s[1] as Int) + ") - first " + RouteKindName(rtState) + ", to (" + (rtFirst[0] as Int) + "," + (rtFirst[1] as Int) + ")")
            EndIf
        EndIf
    EndIf
    ; (the route is handed to StartSession, which writes it in one piece with her first point - see there)
    ; (v1.6.0 BUG 1: every marker of a relative session is placed relative to its TARGET - the last argument)
    Int h = 0
    If rtState > 0
        h = StartSession(w, rtFirst[0], rtFirst[1], rtFirst[2], rtFirst[3], hold, plain, closeB, rtState, s[0], s[1], s[2], s[3], t, 0, t)
    ElseIf rtNpc
        h = StartSession(w, s[0], s[1], s[2], s[3], hold, plain, closeB, 0, s[0], s[1], s[2], s[3], t, 0, t)
    Else
        h = StartSession(w, s[0], s[1], s[2], s[3], hold, plain, closeB, 0, 0.0, 0.0, 0.0, 0.0, None, 0, t)
    EndIf
    If h == 0
        Standpoint.ApiPostResult(aiId, "B", 11)
        Return
    EndIf
    Int i = h - 1
    ; v0.9.4: the stamp of the session StartSession just made - asked again after PairArm below (a native call lets
    ; another stack in: a release or a retask of this slot may have happened by the time it returns)
    Float runStamp = slotStart[i]
    If pose == 2 || pose == 3
        slotPose[i] = pose
    EndIf
    slotDestZ[i] = t.GetPositionZ()
    Bool toPlayer = False
    If (t as Actor) == Game.GetPlayer()
        toPlayer = True
    EndIf
    slotDestIsPlayer[i] = toPlayer
    slotFaceRef[i] = t             ; stopped short -> she turns to the target ...
    slotFaceOffset[i] = s[4]       ; ... plus the asked orientation
    slotApiId[i] = aiId
    slotClose[i] = close
    Standpoint.Log("session " + i + " answers move request #" + aiId + " (hour " + s[5] + ", pose " + pose + ", hold " + hold + " s, close " + close + ")")
    ; v0.9.3: THE PAIR RULE is part of close mode. For an NPC target (not the player, not herself) the walker's and the
    ; target's character controllers ignore each other for as long as this session lives, so her body can overlap the
    ; target's (measured in the second run: 41 cm front, 46 cm right, the target not moved - report 05 §12.2).
    ; FreeSession / a retask release it; the DLL switches it off only once the two are 90 u apart.
    ; KNOWN LIMIT: the rule only bites for a pair whose bodies START to overlap after it is armed - a walker sent from
    ; body contact keeps the old contact until the two have separated once.
    ; (v1.6.0: the block itself moved, unchanged, into PairArmFor - the steered path arms it the same way)
    If close == 1
        PairArmFor(i, w, t, runStamp)
    EndIf
EndFunction

; Name the ending of slot i's move request (if it has one): the DLL measures the result line and calls the plugin back.
Function ApiReport(Int i, String asLetter, Int aiDetail)
    If !slotApiId
        Return
    EndIf
    Int id = slotApiId[i]
    If id > 0
        slotApiId[i] = 0
        Standpoint.ApiPostResult(id, asLetter, aiDetail)
    EndIf
EndFunction

; v0.8.0 on a save made with an older Standpoint, before OnPlayerLoadGame's Setup ran.
Function EnsureApiArrays()
    If !slotApiId
        slotApiId = new Int[4]
    EndIf
    If !slotFaceOffset
        slotFaceOffset = new Float[4]
    EndIf
    If !slotMarkerPlain
        slotMarkerPlain = new ObjectReference[4]
    EndIf
    If !slotPlain
        slotPlain = new Bool[4]
    EndIf
    If !slotClose
        slotClose = new Int[4]
    EndIf
    If !slotStallX
        slotStallX = new Float[4]
    EndIf
    If !slotStallY
        slotStallY = new Float[4]
    EndIf
    If !slotStallT
        slotStallT = new Float[4]
    EndIf
    If !slotSeenOurs
        slotSeenOurs = new Bool[4]     ; v0.9.3
    EndIf
    If !slotPairH
        slotPairH = new Int[4]         ; v0.9.4
    EndIf
    If !slotArrivalBusy
        slotArrivalBusy = new Float[4] ; v0.9.4
    EndIf
    If !slotHasMoved
        slotHasMoved = new Bool[4]     ; v0.9.5
    EndIf
    If !slotFreeing
        slotFreeing = new Bool[4]      ; v0.9.5
    EndIf
    If !slotRoute
        slotRoute = new Int[4]         ; v0.9.7
    EndIf
    If !slotRouteLegs
        slotRouteLegs = new Int[4]     ; v0.9.7
    EndIf
    If !slotFinX
        slotFinX = new Float[4]        ; v0.9.7
    EndIf
    If !slotFinY
        slotFinY = new Float[4]        ; v0.9.7
    EndIf
    If !slotFinZ
        slotFinZ = new Float[4]        ; v0.9.7
    EndIf
    If !slotFinFace
        slotFinFace = new Float[4]     ; v0.9.7
    EndIf
    If !slotFinPlain
        slotFinPlain = new Bool[4]     ; v0.9.8
    EndIf
    If !slotRetry
        slotRetry = new Int[4]         ; v0.9.8
    EndIf
    If !slotRouteOk
        slotRouteOk = new Bool[4]      ; v0.9.8
    EndIf
    If !slotRouteFails
        slotRouteFails = new Int[4]    ; v0.9.9
    EndIf
    If !slotBlockPolls
        slotBlockPolls = new Int[4]    ; v0.9.9
    EndIf
EndFunction

; The IdleMarker slot i's session parks her on: the standard one, or the plain one (pose 0).
ObjectReference Function CurMarker(Int i)
    If slotPlain && slotMarkerPlain
        If slotPlain[i] && slotMarkerPlain[i]
            Return slotMarkerPlain[i]
        EndIf
    EndIf
    Return slotMarker[i]
EndFunction

Bool Function ApiIsBusy(Actor akWalker)
    Return FindSlot(akWalker) >= 0
EndFunction

; v0.7.4: where her session stands, for a consumer that wants to ASK instead of waiting for an event.
;   0 = no session for her    1 = still walking, or the arrival handling is in progress (the settle / short-arrival waits)
;   2 = arrived: Standpoint's own arrival handling is finished and she is held
Int Function ApiGetArrivalState(Actor akWalker)
    If !akWalker
        Return 0
    EndIf
    Int i = FindSlot(akWalker)
    If i < 0
        Return 0
    EndIf
    If slotHolding[i]
        Return 2
    EndIf
    Return 1
EndFunction

; v0.9.5: slotFreeing[i] is True from the first statement to the last. This function makes several native calls before it
; clears slotWalker / slotActive (the order is deliberate: clearing the slot early would let a new session take it while
; the old alias is still being cleared), and every native call lets another stack run. A handler that asked SameSession
; during those calls was told "yes, still yours" for a session that was already being released. SameSession now answers
; False while the flag is up; nothing else reads it except the owner check after PairArm (ApiRunRequest).
; v1.5.1: aiMode (a walk's far session only - ApiFarEnd and the far poll; every other caller passes nothing: 0, the full release
; as before): 1 = the package override comes off WITHOUT IdleForceDefaultState / EvaluatePackage (a walk of hers has her,
; moving); 2 = override off + EvaluatePackage, no IdleForceDefaultState (never on a body in a reaction - "the moonwalk").
Function FreeSession(Int i, Int aiMode = 0)
    If slotFreeing             ; (None only on a save older than 0.9.5, before OnPlayerLoadGame's Setup ran)
        slotFreeing[i] = True
    EndIf
    If slotFar                 ; v1.5.1: no longer a walk's far session (a watchdog poll or a late ApiFarEnd leaves it alone)
        slotFar[i] = 0
    EndIf
    ApiReport(i, "X", 14)      ; v0.8.0 catch-all: an ending that named no letter of its own (every named one answered before this)
    ; v0.9.3: the session's pair rule is released "when apart" - the DLL switches it off once the two are 90 u apart
    ; (or one of them is gone), never while their bodies still overlap. (v0.9.4: by its handle.)
    If slotPairH
        Int freePairH = slotPairH[i]
        If freePairH > 0
            slotPairH[i] = 0
            Standpoint.PairRelease(freePairH)
        EndIf
    EndIf
    If slotPathH                ; v1.3.0: a way round still being planned for her is dropped
        Int freePathH = slotPathH[i]
        If freePathH > 0
            slotPathH[i] = 0
            Standpoint.PathCancel(freePathH)
        EndIf
        slotDetour[i] = 0
    EndIf
    ; v1.6.0 (S1 §2.9): a STEERED session ends in the DLL first - its walk / stand hands her back (the walk's own hand-back
    ; re-evaluates her package). While the walk or its stand has her, the driver does NOT let her go itself (mode 1: no
    ; IdleForceDefaultState mid-stride); pinned by our package after her arrival, the usual let-go.
    Bool freeSteered = False
    If slotSteer
        Int freeSteer = slotSteer[i]
        Int freeOld = slotSteerOld[i]
        slotSteerOld[i] = 0
        If freeSteer != 0
            slotSteer[i] = 0
            freeSteered = True
            If freeSteer > 0
                Standpoint.SteerStop(freeSteer)
            EndIf
            If slotSteerHold[i] != 1 && aiMode == 0
                aiMode = 1
            EndIf
        EndIf
        If freeOld > 0
            Standpoint.SteerStop(freeOld)  ; (released while a steered retask was starting: the session it replaced ends too)
        EndIf
        slotSteerHold[i] = 0
        slotSteerWait[i] = 0
    EndIf
    Actor w = slotWalker[i]
    If w
        If saAvailable
            SeverActionsNativeExt.Travel_CancelByActor(w)
        EndIf
        ActorUtil.RemovePackageOverride(w, slotPkg[i])
        If aiMode == 0
            Debug.SendAnimationEvent(w, "IdleForceDefaultState")
            w.EvaluatePackage()
        ElseIf aiMode == 2
            w.EvaluatePackage()
        EndIf
    EndIf
    ReferenceAlias a = GetOwningQuest().GetAlias(i + 1) as ReferenceAlias
    If a
        a.Clear()
    EndIf
    slotWalker[i] = None
    slotActive[i] = False
    slotHolding[i] = False
    If freeSteered && aiMode == 1
        Standpoint.Log("session " + i + " released (steered: the walk hands her back - no let-go mid-stride)")
    ElseIf aiMode == 1
        Standpoint.Log("session " + i + " released (no let-go: a walk of hers has her)")
    ElseIf aiMode == 2
        Standpoint.Log("session " + i + " released (EvaluatePackage only, no IdleForceDefaultState)")
    Else
        Standpoint.Log("session " + i + " released")
    EndIf
    If slotFreeing
        slotFreeing[i] = False
    EndIf
EndFunction

Int Function FindSlot(Actor akWalker)
    Int i = 0
    While i < 4
        If slotWalker[i] == akWalker
            Return i
        EndIf
        i += 1
    EndWhile
    Return -1
EndFunction

Int Function FreeSlot()
    Int i = 0
    While i < 4
        If slotWalker[i] == None
            Return i
        EndIf
        i += 1
    EndWhile
    Return -1
EndFunction

Function PollSlot(Int i)
    Actor w = slotWalker[i]
    If !w
        slotActive[i] = False
        Int gonePairH = slotPairH[i]   ; v0.9.3: her session's pair rule must not outlive her (the DLL finds her gone: off)
        If gonePairH > 0
            slotPairH[i] = 0
            Standpoint.PairRelease(gonePairH)
        EndIf
        If slotPathH                   ; v1.3.0 (the review): nor a way round still being planned for her
            Int gonePathH = slotPathH[i]
            If gonePathH > 0
                slotPathH[i] = 0
                Standpoint.PathCancel(gonePathH)
            EndIf
        EndIf
        Return
    EndIf
    Float now = Utility.GetCurrentRealTime()

    ; ---- v1.2.0 (report 11 §7.2): a PAUSED game does not count -------------
    ; The polls stop while the game is paused (a menu, the VR dashboard) but the real-time clock runs on, so a walk that
    ; was paused for 2.5 min timed out on the first poll after it (BiSR test, 2026-10-30 18:56: "TIMEOUT" after 173 s).
    ; A gap of more than 5 s since this session's last poll is taken as a pause: the 90 s limit and the stall clock do
    ; not count it. (5 s, not less: the arrival handling can hold one poll for about 11 s, and the two ways out of it
    ; that keep her walking refresh slotLastPollT themselves.)
    EnsureV12Arrays(False)
    If slotLastPollT[i] > 0.0 && now - slotLastPollT[i] > 5.0
        Float pausedFor = now - slotLastPollT[i] - 0.5
        slotPausedT[i] = slotPausedT[i] + pausedFor
        slotStallT[i] = slotStallT[i] + pausedFor
        Standpoint.Log("  s" + i + " no poll for " + (pausedFor as Int) + " s (the game was paused) - not counted")
    EndIf
    slotLastPollT[i] = now

    ; ---- v1.5.1 THE FAR PHASE of a walk (a walk beyond 30 m: the engine's travel, watched by the DLL) ----
    ; The DLL polls her (distance, progress, the gate, the walk's time limit) and ends this session itself (ApiFarEnd); none of
    ; the session rules below (leash, interrupts, danger, kicks, detours, stalls, arrival, the 90 s) apply to it. The one check
    ; here: the walk still wants it - else it is freed now (a far session never outlives its walk: a lost message, a load).
    Int farWalk = FarWalkOf(i)
    If farWalk > 0
        Bool farAlive = Standpoint.WalkFarAlive(farWalk)
        If !farAlive
            ; how it is freed is the DLL's word (review R2 B2 / R2): quietly while a walk of hers goes on, never the let-go on a
            ; body in a reaction
            Int farMode = Standpoint.WalkFarEndMode(farWalk, w, 0)
            If FarWalkOf(i) == farWalk
                Standpoint.Log("  s" + i + " FAR session of walk #" + farWalk + ": the walk has no far phase any more - released (mode " + farMode + ")")
                FreeSession(i, farMode)
            EndIf
        EndIf
        Return
    EndIf

    ; ---- v0.6.1 leash: Standpoint is SHORT-RANGE. If the errand carries the
    ; walker too far from the player, cancel it - no follower ever disappears
    ; across the map because of a Standpoint call. Skipped when the player IS
    ; the destination (walking to the player from afar is legitimate).
    If slotLeash[i] > 0.0 && !slotDestIsPlayer[i]
        Float leashD = w.GetDistance(Game.GetPlayer())
        If leashD > slotLeash[i]
            Standpoint.Log("  s" + i + " LEASH exceeded (" + (leashD as Int) + " > " + (slotLeash[i] as Int) + " u) -> cancel")
            SendFailed(w, 5.0)
            ApiReport(i, "L", 5)
            FreeSession(i)
            Return
        EndIf
    EndIf
    ; v1.6.0: a STEERED session (slotSteer -1 starting, > 0 the DLL's) has no travel package to watch: the procedure / package
    ; reads below are skipped; after the leash, the interrupts and the danger scan SteerPoll takes over (see there)
    EnsureSteerArrays()
    Bool steeredPoll = slotSteer[i] != 0
    Int idx = -2
    Bool ourPkg = False
    If !steeredPoll
        idx = Standpoint.GetCurrentProcedureIndex(w)
        ourPkg = Standpoint.GetCurrentPackageFormID(w) == slotPkg[i].GetFormID()
        If idx != slotLastIdx[i]
            Standpoint.Log("  s" + i + " idx " + slotLastIdx[i] + " -> " + idx + " our=" + ourPkg + " t=" + ((now - slotStart[i]) as Int) + "s")
            slotLastIdx[i] = idx
        EndIf
        ; v0.9.3: close mode's stall clock starts at the FIRST poll in which OUR package is the current one, whatever its
        ; procedure. (0.9.0-0.9.2 waited for a procedure-0 poll. Measured 2026-10-09 21:19, report 05 §12.4: a walk ran in
        ; procedure 1 from its first poll, the clock never started, and she stood 42 u short for 105 s.)
        If ourPkg
            If !slotSeenOurs[i]
                slotSeenOurs[i] = True
                slotStallX[i] = w.GetPositionX()
                slotStallY[i] = w.GetPositionY()
                slotStallT[i] = now
            EndIf
        EndIf
        ; v1.2.0 (F2, report 11 §4): OUR package in procedure 1 counts as "sent" too. Within about 50 u of her marker the
        ; engine does not walk her - it poses her where she stands, in procedure 1, from the first poll (KNOWLEDGEBASE: the
        ; 0.9.8 run). A walker re-sent 25-80 u away therefore never showed procedure 0, the arrival handling never opened for
        ; her, and she stood still until the 90 s limit (BiSR test 2026-10-30: three times, about 520 s in all). The arrival
        ; handling's own "still closing in" check keeps a walker who really is moving in procedure 1 walking.
        If idx == 1 && ourPkg
            slotSawTravel[i] = True
        EndIf
        If idx == 0 && ourPkg
            slotSawTravel[i] = True
            If !avoidProbed && !w.IsInCombat()
                ; diagnostics only - never probe a controller mid-fight, that is
                ; when its agent arrays churn hardest (CTD lesson 2026-08-18)
                avoidProbed = True
                Standpoint.ProbeAvoidance(w)   ; Tier 3 Phase A2: read-only reconnaissance
            EndIf
        EndIf
    EndIf
    Float ddx = w.GetPositionX() - slotX[i]
    Float ddy = w.GetPositionY() - slotY[i]
    Float d = Math.Sqrt(ddx * ddx + ddy * ddy)

    ; ---- interrupt recognition (v0.5.0, observation only) ------------------
    If !slotCombatSeen[i] && w.IsInCombat()
        slotCombatSeen[i] = True
        Standpoint.Log("  s" + i + " INTERRUPT combat")
        SendInterrupted(w, 1.0)
        If slotInterrupts[i] >= 2
            FreeSession(i)
            Return
        EndIf
    EndIf
    If slotInterrupts[i] >= 1 && !slotDestIsPlayer[i]
        If Standpoint.IsPlayerGrabbing(w)
            Standpoint.Log("  s" + i + " INTERRUPT player grab")
            SendInterrupted(w, 3.0)
            ApiReport(i, "P", 8)
            FreeSession(i)
            Return
        EndIf
        Actor player = Game.GetPlayer()
        Float pdx = w.GetPositionX() - player.GetPositionX()
        Float pdy = w.GetPositionY() - player.GetPositionY()
        Float pd = Math.Sqrt(pdx * pdx + pdy * pdy)
        Bool bumpNow = Standpoint.GetRunOncePackageTypeId(w) == 32 && pd < 80.0
        ; v1.2.0 (report 11 §7.3): in a small room any path brushes past the player. A walker who moved 8 u or more
        ; since the last poll (half a second) is walking past him, not blocked by him - the contact count starts again.
        ; (The engine's own bump reaction above, and a grab, still cancel as before.)
        Float cmx = w.GetPositionX() - slotLastPX[i]
        Float cmy = w.GetPositionY() - slotLastPY[i]
        ; FIX ROUND (review R1 #9): a STEERED session that is starting, planning (she stands HELD while her route is worked out -
        ; outdoors for seconds) or waiting stands still on purpose: the player touching her then is no block, and no bump stops it
        Bool steerStill = False
        If steeredPoll
            Int stillPhase = slotSteerPhase[i]
            If slotSteer[i] < 0 || stillPhase == 0 || stillPhase == 1 || stillPhase == 6
                steerStill = True
            EndIf
        EndIf
        If pd < 42.0 && cmx * cmx + cmy * cmy < 64.0 && !steerStill
            slotContactPolls[i] = slotContactPolls[i] + 1
        Else
            slotContactPolls[i] = 0
        EndIf
        ; v0.9.0: in CLOSE mode the player standing near, or a bump next to him, does not cancel the walk - close work
        ; happens next to the player, and the bump is the walker meeting her TARGET. Grabbing her (above) still does.
        If slotClose[i] == 0 && !steerStill && (bumpNow || slotContactPolls[i] >= 2)
            Standpoint.Log("  s" + i + " INTERRUPT player bump/block (bump=" + bumpNow + " pd=" + (pd as Int) + ") at " + PosText(w))
            SendInterrupted(w, 2.0)
            ApiReport(i, "P", 7)
            FreeSession(i)
            Return
        EndIf
    EndIf
    slotLastPX[i] = w.GetPositionX()       ; v1.2.0 - for the next poll's contact rule
    slotLastPY[i] = w.GetPositionY()
    ; ------------------------------------------------------------------------

    ; ---- danger recognition (v0.7.0) ---------------------------------------
    ; USER RULE (2026-08-18): Standpoint sells ACCURACY, not 100% arrival. If
    ; anything hostile is upon the walker or standing in the route, DROP the
    ; session outright - no pausing, no waiting for a corridor to clear, no
    ; resume. The destination is always short-range, so the CONSUMER can simply
    ; ask again later if it still wants it. That is a cleaner contract than a
    ; walker frozen mid-errand, and it is why there is no pause state here.
    If slotDanger[i] >= 1
        Float[] th = Standpoint.ScanThreats(w, slotX[i], slotY[i], slotZ[i], 600.0, 4096.0)
        If th.Length >= 10
            Int n = th[0] as Int
            If n > 0 || th[8] > 0.0 || th[9] > 0.0
                ; all four raw signals whenever ANY is live - this is how we learn
                ; which predicate catches a camp before it aggros
                Standpoint.Log("  s" + i + " threats: union=" + n + " near=" + (th[1] as Int) + " declared=" + (th[6] as Int) + " viaPlayer=" + (th[7] as Int) + " combat=" + (th[8] as Int) + " vAggro=" + (th[9] as Int))
            EndIf
            If n > 0
                Standpoint.Log("  s" + i + " INTERRUPT danger (n=" + n + " near=" + (th[1] as Int) + " blob=" + (th[2] as Int) + "," + (th[3] as Int) + " r=" + (th[5] as Int) + ")")
                SendInterrupted(w, 4.0)
                If slotDanger[i] >= 2
                    Standpoint.Log("  s" + i + " DROPPED (danger) - consumer decides what happens next")
                    SendFailed(w, 4.0)
                    ApiReport(i, "D", 4)
                    FreeSession(i)
                    Return
                EndIf
            EndIf
        EndIf
    EndIf
    ; ------------------------------------------------------------------------

    ; ---- v1.6.0: a STEERED session - the DLL walks her; the rules above (leash, interrupts, danger) applied as to any session ----
    If steeredPoll
        If slotWalker[i] == w && slotActive[i]
            SteerPoll(i, w, now)
        EndIf
        Return
    EndIf

    ; ---- v0.9.0: CLOSE mode - standing still near her target IS her arrival ----
    ; A close request asks for a point she may not be able to reach (the target's body is in the way). Without this she
    ; would stand against her target, 25 u or more from the point, until the 90 s limit. Rule: OUR package has been
    ; seen running (v0.9.3: in any procedure - slotSeenOurs), she has not moved 4 u for the needed time, is still 25 u
    ; or more from the point (nearer, the normal arrival block takes over) and is within 100 u of her target -> the
    ; arrival block runs at once, without its waits, as a SHORT arrival: the marker moves to where she stands, she is
    ; turned to her target, the result says C detail 9 with the real distance (v0.9.3: with the pair rule armed and
    ; 8-64 u to go she was first GLIDED onto the point; v0.9.8: no glide - ONE more walk in, see the arrival block).
    ; (The 100 u keeps a walker stuck far away - a closed door - on the normal path: the 90 s limit and O.)
    ; v0.9.5 - THE NEEDED TIME: "moved, then stopped" is not "has not started yet". A walker who is retasked while she
    ; is HELD on her marker (the test's steps since 0.9.4; any consumer that re-asks) stands in her pose when the new
    ; session starts: our package is already the current one at the first poll, the clock starts there, and with one
    ; flat 3 s a "go to 150 cm" step ended as a short arrival where she stood, before she had taken a step.
    ;   slotHasMoved[i] is False from StartSession until she is 4 u from where the first our-package poll found her.
    ;   never moved in this session -> 8 s still (a walker who is blocked from the very start is still caught, later)
    ;   has moved                   -> 3 s still, as before
    ; ---- v0.9.7 / v0.9.8: STEP BACK AND AROUND - she is walking to a STOP of her route (her step back, a point on the way around, the approach point) ----
    ; RoutePoll looks at where she is and, when it is time, points her session at the next leg. On her route there is
    ; no arrival: the stall rule and the arrival block below are both skipped for this poll (also in the poll that
    ; hands her over to the normal rules - d above may still measure an old waypoint). Leash, grab, danger (above)
    ; and the 90 s limit (below) apply on every leg; the 90 s run from the start of the request, not of the leg.
    Bool routePolled = False
    If slotRoute
        If slotRoute[i] > 0
            routePolled = True
            Bool routeAlive = RoutePoll(i, w, now)
            If !routeAlive
                Return                  ; released or replaced while her next leg was being set
            EndIf
        EndIf
    EndIf
    ; v1.3.0 - LEG START for the session's own leg (the asked point, or a plain walk) - see LegKick. "Started" = she is
    ; 4 u from where the first our-package poll found her; tracked here too, because far from her point a plain walk
    ; never reaches the stall rule below.
    If !routePolled && slotSeenOurs[i] && !slotHasMoved[i] && slotArrivalBusy[i] != slotStart[i]
        Float lsx = w.GetPositionX() - slotStallX[i]
        Float lsy = w.GetPositionY() - slotStallY[i]
        If lsx * lsx + lsy * lsy >= 16.0
            slotHasMoved[i] = True
        ElseIf slotBlockPolls[i] < 2
            Int kicksBefore = slotLegKicks[i]
            Float kickStamp = slotStart[i]
            slotArrivalBusy[i] = kickStamp      ; (the review) one stack only while she may be let go and sent again
            Bool legKickOk = LegKick(i, w, kickStamp, now)
            If slotArrivalBusy[i] == kickStamp
                slotArrivalBusy[i] = 0.0
            EndIf
            If !legKickOk
                Return                  ; released or replaced while she was being kicked
            EndIf
            If kicksBefore == 1 && slotLegKicks[i] == 2
                Return                  ; her leg was just sent again: this poll's readings are stale
            EndIf
        EndIf
    EndIf
    ; v1.3.0 - THE DETOUR for a PLAIN walk's own leg (GoTo / GoToPoint; a close session keeps its own rules near its
    ; target - the review): 80 u to 1500 u from her point (nearer, the rules below decide; further, the engine's own
    ; pathing), she has walked and now stands still - the engine reports MOVEMENT BLOCKED on two polls in a row with her
    ; 1 s still, or 3 s still - a way round is planned first (DetourStart; at most 2 per session).
    If !routePolled && slotClose[i] == 0 && d >= 80.0 && d <= 1500.0 && slotSeenOurs[i] && slotHasMoved[i] && slotArrivalBusy[i] != slotStart[i]
        If slotDetourTries[i] < 2
            Float dtx = w.GetPositionX() - slotStallX[i]
            Float dty = w.GetPositionY() - slotStallY[i]
            If dtx * dtx + dty * dty >= 16.0
                slotStallX[i] = w.GetPositionX()
                slotStallY[i] = w.GetPositionY()
                slotStallT[i] = now
                slotBlockPolls[i] = 0
            Else
                Float dtStill = now - slotStallT[i]
                If Standpoint.GetRunOncePackageTypeId(w) == 36
                    slotBlockPolls[i] = slotBlockPolls[i] + 1
                Else
                    slotBlockPolls[i] = 0
                EndIf
                Bool dtBlocked = slotBlockPolls[i] >= 2 && dtStill >= 1.0
                If dtBlocked || dtStill >= 3.0
                    String dtWhy = "she has stood still for 3 s, " + (d as Int) + " u from her point, at " + PosText(w)
                    If dtBlocked
                        dtWhy = "the engine reports MOVEMENT BLOCKED, " + (d as Int) + " u from her point, at " + PosText(w)
                    EndIf
                    Float dtStamp = slotStart[i]
                    slotArrivalBusy[i] = dtStamp
                    Bool dtOk = DetourStart(i, w, dtStamp, dtWhy)
                    If dtOk && slotArrivalBusy[i] == dtStamp
                        slotArrivalBusy[i] = 0.0
                    EndIf
                    Return
                EndIf
            EndIf
        EndIf
    EndIf
    ; v1.6.1 (C4; run R3 §3 / §7 - A1 under the dining table: the engine leg stalled 110-151 u short, MOVEMENT BLOCKED, and nothing
    ; ended it for 56-62 s): an ENGINE FALLBACK leg (slotEngineLeg: the steered walk had no route, or she sat) that does not get on
    ; ends NOW instead of at the 90 s - 80 u or more from her point (nearer, the stall rule below makes it an arrival, as for every
    ; plain session), no way round left to plan (none after no route; a sitting walker's leg has used its two), and the engine
    ; reports MOVEMENT BLOCKED on two polls in a row with her 1 s still, or she has not moved 4 u for 6 s (once she has walked, or
    ; after both leg-start kicks) -> NO WAY: Standpoint_Failed 7, the request O 17, released.
    If !routePolled && slotClose[i] == 0 && slotEngineLeg[i] > 0 && d >= 80.0 && slotDetourTries[i] >= 2 && slotSeenOurs[i] && slotArrivalBusy[i] != slotStart[i]
        Float elx = w.GetPositionX() - slotStallX[i]
        Float ely = w.GetPositionY() - slotStallY[i]
        If elx * elx + ely * ely >= 16.0
            slotHasMoved[i] = True
            slotStallX[i] = w.GetPositionX()
            slotStallY[i] = w.GetPositionY()
            slotStallT[i] = now
            slotBlockPolls[i] = 0
        Else
            Float elStamp = slotStart[i]
            Float elStill = now - slotStallT[i]
            Int elRunOnce = Standpoint.GetRunOncePackageTypeId(w)
            If !SameSession(i, w, elStamp)
                Return                  ; (released or retasked during the call)
            EndIf
            If elRunOnce == 36
                slotBlockPolls[i] = slotBlockPolls[i] + 1
            Else
                slotBlockPolls[i] = 0
            EndIf
            Bool elBlocked = slotBlockPolls[i] >= 2 && elStill >= 1.0
            Bool elStuck = False
            If elStill >= 6.0
                If slotHasMoved[i] || slotLegKicks[i] >= 2
                    elStuck = True
                EndIf
            EndIf
            If elBlocked || elStuck
                String elWhy = "she has not moved 4 u for " + (elStill as Int) + " s"
                If elBlocked
                    elWhy = "the engine reports MOVEMENT BLOCKED and she has not moved for 1 s"
                EndIf
                String elLeg = "after no route"
                If slotEngineLeg[i] == 2
                    elLeg = "she sat"
                ElseIf slotEngineLeg[i] == 3
                    elLeg = "after no route, to the point's nearest navmesh point"
                EndIf
                Standpoint.Log("  s" + i + " ENGINE LEG ENDED (" + elLeg + "): " + elWhy + ", " + (d as Int) + " u from her point at " + PosText(w) + " - no way: Failed 7, O 17")
                SendFailed(w, 7.0)
                ApiReport(i, "O", 17)
                FreeSession(i)
                Return
            EndIf
        EndIf
    EndIf
    Bool stalled = False
    ; (v0.9.4: not asked while another stack already runs this session's arrival handling - see slotArrivalBusy below)
    ; v1.2.0 (F2, report 11 §4): a PLAIN session (GoTo / GoToPoint) gets the stall rule too, when she stands within
    ; 80 u of her point - 3 s still once she has moved, 8 s if she never moved. Before, only the 90 s limit ended a
    ; plain walker who had stopped 25-80 u short. Further out a plain walker keeps the 90 s limit (a door, a crowd).
    If !routePolled && (slotClose[i] > 0 || d < 80.0) && slotSeenOurs[i] && slotArrivalBusy[i] != slotStart[i]
        Float sdx = w.GetPositionX() - slotStallX[i]
        Float sdy = w.GetPositionY() - slotStallY[i]
        Float needStill = 8.0
        If slotHasMoved[i]
            needStill = 3.0
        EndIf
        ; v0.9.9 - BLOCKED: REACT AT ONCE (the user, 2026-10-10: "if the path is blocked, making the NPCs actually react
        ; right away is probably the best thing to do"). The engine names it itself: run-once package type 36 =
        ; "movement blocked" (run 4 step 7 and run 5 step 9 stood 8 s under it). For a close session with an NPC
        ; target (slotRouteOk): type 36 AND no movement for 1 s = the walk has ended here, now - no waiting out the
        ; 3 s / 8 s. What follows is the arrival block's business: ONE more walk in, then C 9 or O 17.
        ; "Blocked" needs the reading on TWO polls in a row (slotBlockPolls), so one passing reading cannot end a
        ; walk. In the three logged runs type 36 showed in three episodes: each a real block with no movement at
        ; all, and gone the moment she was let go.
        Bool blockedNow = False
        Float stillFor = now - slotStallT[i]
        Bool routeSession = False
        If slotRouteOk
            If slotRouteOk[i]
                routeSession = True
            EndIf
        EndIf
        If routeSession
            If sdx * sdx + sdy * sdy >= 16.0
                slotBlockPolls[i] = 0
            ElseIf Standpoint.GetRunOncePackageTypeId(w) == 36
                slotBlockPolls[i] = slotBlockPolls[i] + 1
            Else
                slotBlockPolls[i] = 0
            EndIf
            If slotBlockPolls[i] >= 2 && stillFor >= 1.0 && stillFor < needStill && d >= 25.0
                blockedNow = True
            EndIf
        EndIf
        If sdx * sdx + sdy * sdy >= 16.0
            slotHasMoved[i] = True
            slotStallX[i] = w.GetPositionX()
            slotStallY[i] = w.GetPositionY()
            slotStallT[i] = now
        ElseIf (now - slotStallT[i] >= needStill || blockedNow) && d >= 25.0 && slotClose[i] == 0
            ; v1.2.0 (F2): a plain session that stands still within 80 u of her point - her walk has ended here
            stalled = True
            String plainHow = "8 s and never moved in this session"
            If slotHasMoved[i]
                plainHow = "3 s after she had moved"
            EndIf
            Standpoint.Log("  s" + i + " she has stood still for " + plainHow + ", " + (d as Int) + " u from the point at " + PosText(w) + " - her walk has ended here (the arrival handling decides what it is)")
        ElseIf (now - slotStallT[i] >= needStill || blockedNow) && d >= 25.0
            ObjectReference tr = slotFaceRef[i]
            If tr
                Float toTarget = tr.GetDistance(w)
                ; v0.9.8: 200 u for a close session with an NPC target (slotRouteOk) - her run-up stop can lie up to
                ; about 165 u from the target, and a walker who does not start her walk in from there must not wait
                ; out the 90 s. (The arrival block then gives her the second walk in, or releases her: see there.)
                Float stallReach = 100.0
                If slotRouteOk
                    If slotRouteOk[i]
                        stallReach = 200.0
                    EndIf
                EndIf
                If toTarget < stallReach
                    stalled = True
                    String stillHow = "8 s and never moved in this session"
                    If slotHasMoved[i]
                        stillHow = "3 s after she had moved"
                    EndIf
                    If blockedNow
                        stillHow = "1 s and the engine reports MOVEMENT BLOCKED"
                    EndIf
                    Standpoint.Log("  s" + i + " CLOSE: she has stood still for " + stillHow + ", " + (d as Int) + " u from the point and " + (toTarget as Int) + " u from her target - her walk has ended here (the arrival handling decides what it is)")
                EndIf
            EndIf
        EndIf
    EndIf
    ; v0.9.4: ONE stack per session in the arrival block. The block waits up to about 11 s inside one
    ; OnUpdate; a session started for ANOTHER walker meanwhile asks for an update of its own (StartSession), and that
    ; second OnUpdate polled this slot too - still "active" - and walked into the same block: two stacks handling one
    ; arrival (an old fault, as old as the four-slot engine; found by reading, 2026-10-09). slotArrivalBusy[i] holds
    ; the stamp (slotStart) of the session whose arrival is being handled: a second stack skips the block and goes on
    ; to the 90 s check below. A new session in the slot has a new stamp, so it is never locked out; the one path that
    ; leaves the block with the same session still walking ("still closing in") clears it.
    If !routePolled && ((idx == 1 && ourPkg && slotSawTravel[i] && d < 80.0) || d < 25.0 || stalled) && slotArrivalBusy[i] != slotStart[i]
        slotArrivalBusy[i] = slotStart[i]
        ; ---- v0.7.4: the session stamp (VRCD h79 §16.2) -----------------------
        ; Measured 2026-10-01 23:49-23:50: this block waits about 11 s in total on ONE OnUpdate stack and never asked whether
        ; the slot still held the session it started with. A consumer released slot 1 and started a NEW session in the same
        ; slot during those waits; the stale poll then measured the NEW session's point, moved the NEW session's marker to
        ; where the walker stood and declared it "ARRIVED (short) ... t=-9S". The new walk never happened.
        ; slotStart is unique per session (StartSession), so it is the stamp: after EVERY wait the block checks SameSession
        ; and, when the session is no longer the one it started with, leaves WITHOUT touching the marker, the walker or the slot.
        Float stamp = slotStart[i]
        ; v1.2.0 (F4, report 11 §7.1): the waits below were flat (3 s, 4 s, 2 s, 2.5 s - about 8.4 s for a walker who
        ; had already stopped). Each is now "UP TO" that long and ends as soon as she has stood still for 1.5 s facing
        ; the right way (SettleWait). blockStill = how long she had stood still when the last of them ended.
        Float blockStill = 0.0
        If !stalled        ; v0.9.0: a stalled walker has already stood still for 3 s
            blockStill = SettleWait(i, w, stamp, 3.0, slotFaceDeg[i])
            If blockStill < 0.0
                Standpoint.Log("  s" + i + " arrival handling abandoned - the session changed while it waited")
                Return
            EndIf
        EndIf
        ddx = w.GetPositionX() - slotX[i]
        ddy = w.GetPositionY() - slotY[i]
        d = Math.Sqrt(ddx * ddx + ddy * ddy)
        Float angErr = NormAng180(w.GetAngleZ() - slotFaceDeg[i])
        ; the d<25 fast path can fire BEFORE the pose phase engages - if the facing
        ; is still way off, give UseIdleMarker one more beat and re-measure
        ; v0.9.1: a CLOSE walker who is short of the point skips this wait - the marker cannot turn her while she is
        ; not on it, and the short-arrival handling below turns her anyway.
        If !stalled && (slotClose[i] == 0 || d <= 8.0) && (angErr > 5.0 || angErr < -5.0)
            blockStill = SettleWait(i, w, stamp, 4.0, slotFaceDeg[i])
            If blockStill < 0.0
                Standpoint.Log("  s" + i + " arrival handling abandoned - the session changed while it waited")
                Return
            EndIf
            ddx = w.GetPositionX() - slotX[i]
            ddy = w.GetPositionY() - slotY[i]
            d = Math.Sqrt(ddx * ddx + ddy * ddy)
            angErr = NormAng180(w.GetAngleZ() - slotFaceDeg[i])
            Standpoint.Log("  s" + i + " re-measured after pose wait")
        EndIf
        ; ---- v0.7.2: she stopped SHORT of the point (VRCD h79 §13) ------------
        ; Measured 2026-10-01: wedged between a table and the NPC she walked to, a walker stood 16 u short with the travel
        ; still unfinished, so UseIdleMarker never ran and she was never turned (93 deg off). The user: "turn her if she come
        ; short, with a recognition and detection if the movement is still possible".
        ;   still closing in (> 4 u in 2 s) -> not arrived: keep walking (the 90 s limit still bounds it)
        ;   stuck -> move the marker to where she stands (same heading): the travel completes and the engine's own
        ;            UseIdleMarker turns and pins her there; a direct turn only if the engine did not do it.
        ; The distance reported in Standpoint_Arrived stays the distance from the point that was asked for.
        Bool wasShort = False
        If d > 8.0
            ; v0.9.0: the stall rule has already established that she is not closing in. v1.2.0 (F4): so has a wait above
            ; that ended with her standing still for 1.5 s - then the 2 s check is skipped too.
            If !stalled && blockStill < 1.5
                Float dA = d
                Utility.Wait(2.0)
                If !SameSession(i, w, stamp)
                    Standpoint.Log("  s" + i + " arrival handling abandoned - the session changed while it waited")
                    Return
                EndIf
                Float dB = DistToSlot(w, i)
                If dB < dA - 4.0
                    Standpoint.Log("  s" + i + " short (" + (dB as Int) + " u) but still closing in - keep walking")
                    If slotArrivalBusy[i] == stamp     ; v0.9.4: the same session walks on - the block is open again for it
                        slotArrivalBusy[i] = 0.0
                    EndIf
                    slotLastPollT[i] = Utility.GetCurrentRealTime()   ; v1.2.0: this wait was not a paused game
                    Return
                EndIf
            EndIf
            ; v0.9.1 / v0.9.2: a walker who is NOT near where she was sent has not arrived, whatever stopped her.
            ; Measured 2026-10-09 19:14: thrown 455 u away and falling 4,700 u below the floor, she was taken as a
            ; "short arrival" - the marker was moved to her in mid-air, pinned her there, and the request answered C.
            ; The rule, for EVERY session since 0.9.2 (the user: "sure"):
            ;   a normal session - 100 u or more from the asked POINT
            ;   a close session  - 100 u or more from her TARGET (its point may lie inside the target's body)
            ;   either           - 100 u above / below the asked point
            ; -> not an arrival: she is released; Standpoint_Failed 6; a move request answers O, detail 15.
            Bool strayed = False
            If slotClose[i] > 0
                ObjectReference tgt = slotFaceRef[i]
                If tgt
                    ; v0.9.8: while her ONE second walk in is still unused, a close walker with an NPC target may
                    ; stand up to 200 u out (a run-up stop) without being "strayed" - the retry below sends her in
                    ; again. NEVER an arrival out there: the short handling is only reached with the retry used, and
                    ; then the limit is 100 u as before.
                    Float strayLimit = 100.0
                    If slotRouteOk[i] && slotRetry[i] == 0
                        strayLimit = 200.0
                    EndIf
                    If tgt.GetDistance(w) >= strayLimit
                        strayed = True
                    EndIf
                EndIf
            ElseIf d >= 100.0
                strayed = True
            EndIf
            Float dz = w.GetPositionZ() - slotZ[i]
            If dz > 100.0 || dz < -100.0
                strayed = True
            EndIf
            If strayed
                Standpoint.Log("  s" + i + " she stands still " + (d as Int) + " u from the point (height off by " + (dz as Int) + " u) - too far to be an arrival: released, Failed 6")
                SendFailed(w, 6.0)
                ApiReport(i, "O", 15)
                FreeSession(i)
                Return
            EndIf
            ; ---- v0.9.8: NO GLIDE - ONE MORE WALK IN (the user's ruling of 2026-10-10; report 05 §16.5, §17) ----
            ; 0.9.3-0.9.7 moved a close walker who had stopped short onto her point with a translate (the glide). The
            ; fourth run showed what that is: all 8 glides slid her across the floor in her wait pose. The user: "moving
            ; a NPC while they are in the wait pose make her slide on the floor. Let's try not to do that again."
            ; The same run showed why she stops short and how to get her there on her feet: within about 50 u of her
            ; marker she does not walk - she poses where she stands; a walk that starts 58 u or more from the point goes
            ; all the way. So, for a close session with an NPC target (slotRouteOk), ONCE per session (slotRetry):
            ;   12 u or less from the point  -> she is taken where she stands (the short handling below: C 9, the real
            ;                                   distance in the result)
            ;   further                      -> she is let go and RouteStage is asked where the walk in should start
            ;                                   from: a stop 60 u or more away (the approach point, or her step back).
            ;                                   She walks there, and from there in again. After that second walk in she
            ;                                   is taken where she stands.
            ; There is no translate anywhere in a session any more (only the water test's wade still uses one).
            Float retryD = DistToSlot(w, i)
            If !SameSession(i, w, stamp)
                Standpoint.Log("  s" + i + " arrival handling abandoned - the session changed while it waited")
                Return
            EndIf
            Bool retryNow = False
            If slotClose[i] > 0 && retryD > 12.0
                If slotRouteOk[i] && slotRetry[i] == 0
                    retryNow = True
                EndIf
            EndIf
            If retryNow
                slotRetry[i] = 1
                ; RouteStage is asked BEFORE she is let go (she stands in her pose: her place does not change). An
                ; empty answer means either "she has the run-up: send her in again" (58 u or more from the point) or
                ; "no stop could be solved" - and then, nearer than 58 u, a new start would not be walked either
                ; (F1): she is taken where she stands without being let go for nothing.
                ObjectReference retryT = slotFaceRef[i]
                Float[] retryStop
                If retryT
                    retryStop = RouteStage(retryT, w, slotFinX[i], slotFinY[i], False)
                    If !SameSession(i, w, stamp)
                        Standpoint.Log("  s" + i + " arrival handling abandoned - the session changed while it waited")
                        Return
                    EndIf
                EndIf
                Bool retryWalk = True
                If retryStop.Length < 5 && retryD < 58.0
                    retryWalk = False
                EndIf
                If retryWalk
                    Standpoint.Log("  s" + i + " she stands " + (retryD as Int) + " u from the point - no slide: she is let go for ONE more walk in")
                    ; 1.0 s: she stands in her wait pose on the asked point's marker (the proven let-go of a held walker)
                    Bool retryLetGo = LetGo(i, w, stamp)
                    If !retryLetGo
                        Standpoint.Log("  s" + i + " arrival handling abandoned - the session changed while she was being let go")
                        Return
                    EndIf
                    Bool retrySent = False
                    If retryStop.Length >= 5
                        Int retryKind = retryStop[4] as Int
                        Standpoint.Log("  s" + i + " ROUTE: -> " + RouteKindName(retryKind) + ", to (" + (retryStop[0] as Int) + "," + (retryStop[1] as Int) + ")")
                        retrySent = RouteSend(i, w, stamp, retryStop[0], retryStop[1], retryStop[2], retryStop[3], retryKind)
                    Else
                        Standpoint.Log("  s" + i + " ROUTE: she has the run-up (" + (retryD as Int) + " u) - sent to the asked point once more")
                        retrySent = RouteSend(i, w, stamp, slotFinX[i], slotFinY[i], slotFinZ[i], slotFinFace[i], 0)
                    EndIf
                    If retrySent
                        If slotArrivalBusy[i] == stamp     ; the same session walks on - the block is open again for it
                            slotArrivalBusy[i] = 0.0
                        EndIf
                        slotLastPollT[i] = Utility.GetCurrentRealTime()   ; v1.2.0: this handling was not a paused game
                    Else
                        Standpoint.Log("  s" + i + " arrival handling abandoned - the session changed while her next leg was being set")
                    EndIf
                    Return
                EndIf
                Standpoint.Log("  s" + i + " she stands " + (retryD as Int) + " u from the point and no stop to walk in from could be solved")
                ; (a walker 100 u or more from her target is not taken: the strayed rule, now that the retry is used)
                ObjectReference retryFar = slotFaceRef[i]
                If retryFar
                    If retryFar.GetDistance(w) >= 100.0
                        Standpoint.Log("  s" + i + " ... but she is 100 u or more from her target - too far to be an arrival: released, Failed 6")
                        SendFailed(w, 6.0)
                        ApiReport(i, "O", 15)
                        FreeSession(i)
                        Return
                    EndIf
                EndIf
            EndIf
            ; ---- v0.9.9: NO WAY (the user, 2026-10-10: "if it cannot find any other path to make it there, then report it
            ; as such") ----
            ; Here every way has been tried: she ended more than 8 u from the point, a second walk in is not possible
            ; any more (used, or she never walked, or no stop could be solved) or the miss is 12 u or less. For a close
            ; request to an NPC (slotRouteOk) "taken where she stands" is only an ARRIVAL when she is NEAR: on the
            ; asked side of her target and within 35 u (50 cm) of the point - that is "stopped short" (C 9, the real
            ; distance in the result). Anything else - the wrong side, a metre away - is not an arrival: the request
            ; answers O, detail 17 (kDetailNoWay), Standpoint_Failed 7, and she is released.
            If slotRouteOk[i]
                Bool endNear = RouteNear(i, w)
                If !SameSession(i, w, stamp)
                    Standpoint.Log("  s" + i + " arrival handling abandoned - the session changed while it waited")
                    Return
                EndIf
                If !endNear
                    Standpoint.Log("  s" + i + " she stands " + (d as Int) + " u from the asked point, not near it, and every way has been tried - NO WAY to it (blocked, or the engine would not walk her there): released, Failed 7")
                    If !SameSession(i, w, stamp)
                        Standpoint.Log("  s" + i + " arrival handling abandoned - the session changed while it waited")
                        Return
                    EndIf
                    ApiReport(i, "O", 17)
                    SendFailed(w, 7.0)
                    If SameSession(i, w, stamp)
                        FreeSession(i)
                    EndIf
                    Return
                EndIf
            EndIf
            wasShort = True
            ; v0.7.3: the heading she is turned to. With a FACE TARGET (GoTo's actor target, or SetFaceTarget): toward it, from
            ; where she actually stands - measured 2026-10-01: stuck 22 u to the side of a point in body contact, she was
            ; turned to the point's heading and missed the NPC she came to by 27 deg. Without one: the asked heading.
            Float wantDeg = slotFaceDeg[i]
            String toWhom = "the asked heading"
            ObjectReference fr = slotFaceRef[i]
            If fr && fr != w
                Float fdx = fr.GetPositionX() - w.GetPositionX()
                Float fdy = fr.GetPositionY() - w.GetPositionY()
                If fdx * fdx + fdy * fdy > 1.0
                    ; v0.8.0: a move request's orientation (her back to the target = 180) rides on top of "toward it"
                    wantDeg = NormAng360(CompassBearing(fdx, fdy) + slotFaceOffset[i])
                    If wantDeg >= 359.95
                        wantDeg = 0.0
                    EndIf
                    toWhom = "her face target " + fr
                EndIf
            EndIf
            ObjectReference mk = CurMarker(i)
            mk.MoveTo(w, 0.0, 0.0, 0.0, False)          ; v1.6.0 BUG 1: the re-anchor is a cross-cell move to her (was SetPosition)
            mk.SetAngle(0.0, 0.0, wantDeg)
            w.EvaluatePackage()
            Bool turnOk = TurnWait(i, w, stamp, 2.5, wantDeg)     ; v1.2.0 (F4): up to 2.5 s, not a flat 2.5 s
            If !turnOk
                Standpoint.Log("  s" + i + " arrival handling abandoned - the session changed while it waited")
                Return
            EndIf
            angErr = NormAng180(w.GetAngleZ() - wantDeg)
            String how = "the engine turned her"
            If angErr > 5.0 || angErr < -5.0
                w.SetAngle(0.0, 0.0, wantDeg)
                Utility.Wait(0.5)
                If !SameSession(i, w, stamp)
                    Standpoint.Log("  s" + i + " arrival handling abandoned - the session changed while it waited")
                    Return
                EndIf
                angErr = NormAng180(w.GetAngleZ() - wantDeg)
                how = "turned directly (the engine did not)"
            EndIf
            d = DistToSlot(w, i)
            Standpoint.Log("  s" + i + " SHORT and stuck " + (d as Int) + " u from the point at " + PosText(w) + " - the marker moved to where she stands; " + how + " toward " + toWhom + ", angErr=" + angErr)
        EndIf
        ; ---- v0.8.1: she is ON the point but faces the wrong way --------------------
        ; Measured 2026-10-08 23:30 (the first move-API run): 3 u from the point, 118 deg off after BOTH waits (7 s) - the
        ; d < 25 fast path had fired with the travel unfinished, so UseIdleMarker never ran. Only a SHORT arrival was ever
        ; corrected; this one was reported as arrived, facing wrong. Same cure as the short arrival: the marker moves to
        ; where she stands (the ASKED heading), the travel completes and the engine turns and pins her; a direct turn
        ; only if the engine did not do it.
        If !wasShort
            If angErr > 5.0 || angErr < -5.0
                Float offBy = angErr
                ObjectReference mkT = CurMarker(i)
                mkT.MoveTo(w, 0.0, 0.0, 0.0, False)     ; v1.6.0 BUG 1: a cross-cell move to her (was SetPosition)
                mkT.SetAngle(0.0, 0.0, slotFaceDeg[i])
                w.EvaluatePackage()
                Bool turnOkT = TurnWait(i, w, stamp, 2.5, slotFaceDeg[i])     ; v1.2.0 (F4): up to 2.5 s
                If !turnOkT
                    Standpoint.Log("  s" + i + " arrival handling abandoned - the session changed while it waited")
                    Return
                EndIf
                angErr = NormAng180(w.GetAngleZ() - slotFaceDeg[i])
                String howT = "the engine turned her"
                If angErr > 5.0 || angErr < -5.0
                    w.SetAngle(0.0, 0.0, slotFaceDeg[i])
                    Utility.Wait(0.5)
                    If !SameSession(i, w, stamp)
                        Standpoint.Log("  s" + i + " arrival handling abandoned - the session changed while it waited")
                        Return
                    EndIf
                    angErr = NormAng180(w.GetAngleZ() - slotFaceDeg[i])
                    howT = "turned directly (the engine did not)"
                EndIf
                d = DistToSlot(w, i)
                Standpoint.Log("  s" + i + " ON the point but " + offBy + " deg off - the marker moved to where she stands; " + howT + ", angErr=" + angErr)
            EndIf
        EndIf
        ; v1.6.0: the rest of the arrival (the ARRIVED line, Standpoint_Arrived, the C answer, the pose, the hold policy) is
        ; ArrivalTail - unchanged, shared with the steered arrival
        ArrivalTail(i, w, d, angErr, wasShort, stamp, now, "")
        Return
    EndIf
    If now - slotStart[i] - slotPausedT[i] > 90.0       ; v1.2.0: a paused game is not counted (report 11 §7.2)
        Standpoint.Log("  s" + i + " TIMEOUT idx=" + idx + " d=" + (d as Int) + " at " + PosText(w))
        SendFailed(w, 2.0)
        ApiReport(i, "O", 2)
        FreeSession(i)
        Return
    EndIf
EndFunction

; v0.7.4: is slot i still the session the arrival block started with? Asked after every wait in that block (VRCD h79 §16.2).
; A release frees the walker and the active flag; a new session (a retask too) writes a new slotStart.
; v0.9.5: a release that is IN FLIGHT (FreeSession(i) running on another stack, slotWalker / slotActive not yet cleared)
; is "not the same session" too - a stale handler gives up at once instead of acting on a slot that is being freed.
Bool Function SameSession(Int i, Actor akW, Float afStamp)
    If slotFreeing             ; (None only on a save older than 0.9.5, before OnPlayerLoadGame's Setup ran)
        If slotFreeing[i]
            Return False
        EndIf
    EndIf
    Return slotWalker[i] == akW && slotActive[i] && slotStart[i] == afStamp
EndFunction

; v1.2.0: take one of the old developer test spells off the player, if she has it
Function RemoveTestSpell(Int aiFormId)
    Spell rts = Game.GetFormFromFile(aiFormId, "Standpoint.esp") as Spell
    Actor rtsPlayer = Game.GetPlayer()
    If rts && rtsPlayer.HasSpell(rts)
        rtsPlayer.RemoveSpell(rts)
        Standpoint.Log("the old test spell " + rts + " was taken off the player")
    EndIf
EndFunction

; v1.2.0: the per-session arrays the BiSR-letter fixes added. Setup makes them new at every load (abNew); on an older
; save a call can come in before OnPlayerLoadGame's Setup ran - then they are made here, once.
Function EnsureV12Arrays(Bool abNew)
    If abNew || !slotPausedT
        slotLastPollT = new Float[4]
        slotPausedT = new Float[4]
        slotLastPX = new Float[4]
        slotLastPY = new Float[4]
    EndIf
    If abNew || !slotLegKicks          ; v1.3.0 (a save made with 1.2.0 has the arrays above but not these)
        slotLegKicks = new Int[4]
        slotPathH = new Int[4]
        slotDetour = new Int[4]
        slotDetourTries = new Int[4]
        slotPlanT = new Float[4]
        slotWpX = new Float[64]
        slotWpY = new Float[64]
        slotWpZ = new Float[64]
        slotWpN = new Int[4]
        slotWpK = new Int[4]
        slotDefState = new Int[4]
        slotDefX = new Float[4]
        slotDefY = new Float[4]
        slotDefZ = new Float[4]
        slotDefFace = new Float[4]
    EndIf
EndFunction

; v1.3.0 - LEG START (the user's BiSR run of 2026-10-30 22:37-23:22, handover 16 §2): a leg that has not STARTED - she
; has not moved 4 u since her package was first seen running on it - is kicked before it can count as failed:
;   2 s -> her package is evaluated again
;   4 s -> she is let go and the SAME leg is sent once more (a fresh start: its own 8 s from there)
; Measured: four legs stood 8 s with 78-96 u to go and counted as FAILED - two of the three NO WAY answers that night;
; the same leg sent again walked at once (23:12:59: reached in 2.5 s).
; Returns False when the session is no longer this one (the caller leaves at once), True otherwise.
Bool Function LegKick(Int i, Actor akW, Float afStamp, Float afNow)
    Float lkFor = afNow - slotStallT[i]
    If slotLegKicks[i] == 0 && lkFor >= 2.0
        slotLegKicks[i] = 1
        Standpoint.Log("  s" + i + " LEG START: she has not moved for 2 s on her " + RouteKindName(slotRoute[i]) + " leg (" + (DistToSlot(akW, i) as Int) + " u to go) - her package is evaluated again")
        akW.EvaluatePackage()
        Return SameSession(i, akW, afStamp)
    EndIf
    If slotLegKicks[i] == 1 && lkFor >= 4.0
        slotLegKicks[i] = 2
        Int lkState = slotRoute[i]
        Float lkX = slotX[i]
        Float lkY = slotY[i]
        Float lkZ = slotZ[i]
        Float lkFace = slotFaceDeg[i]
        Standpoint.Log("  s" + i + " LEG START: still not moving after 4 s - she is let go and her " + RouteKindName(lkState) + " leg is sent once more")
        Bool lkLet = LetGo(i, akW, afStamp, 0.5)
        If !lkLet
            Return False
        EndIf
        Bool lkSent = RouteSend(i, akW, afStamp, lkX, lkY, lkZ, lkFace, lkState)
        If !lkSent
            Return False
        EndIf
        If lkState > 0 && lkState != 4
            slotRouteLegs[i] = slotRouteLegs[i] - 1     ; the same stop again, not a new one (the loop guard counts stops)
        EndIf
        slotLegKicks[i] = 2                            ; (RouteSend reset it for a new leg - this is the same leg)
        slotLastPollT[i] = Utility.GetCurrentRealTime()   ; the let-go wait was not a paused game
    EndIf
    Return True
EndFunction

; v1.3.0 - THE DETOUR (the user, 2026-10-30: "If we have a route and create waypoint on that route, we can travel the
; NPC on that route "manualy" ... right 250u, front 350u, and left 200u ... with check to confirm they arrive").
; Her leg has FAILED with 60 u or more still to go (the engine's MOVEMENT BLOCKED, or she stood still after walking).
; Before it counts, the planner looks for a way round (PathPlanner.cpp: the navmesh for the floor, Havok rays at knee and
; chest height for what stands in the way - furniture, a house mod's walls - a fraction of a second, sliced per frame).
; She stays where she is meanwhile. A way round = she walks its points one by one as DETOUR legs (the same arrival check
; as any stop), then her own leg again from there. No way round = her leg is tried once more as it was; if it fails
; again, the usual rules count it. At most 2 per session. False = the session is no longer this one.
Bool Function DetourStart(Int i, Actor akW, Float afStamp, String asWhy)
    slotDetourTries[i] = slotDetourTries[i] + 1
    slotDefState[i] = slotRoute[i]
    slotDefX[i] = slotX[i]
    slotDefY[i] = slotY[i]
    slotDefZ[i] = slotZ[i]
    slotDefFace[i] = slotFaceDeg[i]
    Standpoint.Log("  s" + i + " DETOUR: " + asWhy + " - a way round to her " + RouteKindName(slotDefState[i]) + " point (" + (slotDefX[i] as Int) + "," + (slotDefY[i] as Int) + ") is planned (try " + slotDetourTries[i] + " of 2)")
    Float dsNow = Utility.GetCurrentRealTime()
    ObjectReference dsAvoid = slotFaceRef[i]       ; the walk's own target (None for a plain walk): the way keeps clear of her
    Int dsH = Standpoint.PathStart(akW, slotDefX[i], slotDefY[i], slotDefZ[i], dsAvoid)
    If !SameSession(i, akW, afStamp)
        If dsH > 0
            Standpoint.PathCancel(dsH)
        EndIf
        Return False
    EndIf
    If dsH <= 0
        Standpoint.Log("  s" + i + " DETOUR: the planner refused - her leg is tried once more as it was")
        slotDetourTries[i] = 2
        Return DetourResume(i, akW, afStamp)
    EndIf
    ; (the review) the four writes in ONE piece - no call out of the script between them, so a retask cannot slip in
    ; between and inherit the planning state
    slotPathH[i] = dsH
    slotDetour[i] = 1
    slotPlanT[i] = dsNow
    slotRoute[i] = 4                 ; RoutePoll owns the session while the way round is planned and walked
    Return True
EndFunction

; v1.3.0: one poll while the way round is being planned. False = the session is no longer this one.
Bool Function DetourPoll(Int i, Actor akW, Float afStamp, Float afNow)
    Int dpH = slotPathH[i]
    Float[] dpR = Standpoint.PathResult(dpH)
    If !SameSession(i, akW, afStamp)
        Return False
    EndIf
    If dpR.Length < 2
        If afNow - slotPlanT[i] < 5.0
            Return True              ; still planning
        EndIf
        Standpoint.Log("  s" + i + " DETOUR: no answer from the planner in 5 s - her leg is tried once more as it was")
        slotPathH[i] = 0
        Standpoint.PathCancel(dpH)
        Return DetourResume(i, akW, afStamp)
    EndIf
    slotPathH[i] = 0
    Standpoint.PathCancel(dpH)
    Int dpSt = dpR[0] as Int
    Int dpN = dpR[1] as Int
    If dpSt == 2 && dpN >= 2 && dpR.Length >= 2 + 3 * dpN
        If dpN > 16
            dpN = 16
        EndIf
        String dpPts = ""
        Int dpK = 0
        While dpK < dpN
            slotWpX[i * 16 + dpK] = dpR[2 + 3 * dpK]
            slotWpY[i * 16 + dpK] = dpR[3 + 3 * dpK]
            slotWpZ[i * 16 + dpK] = dpR[4 + 3 * dpK]
            dpPts = dpPts + " (" + (dpR[2 + 3 * dpK] as Int) + "," + (dpR[3 + 3 * dpK] as Int) + ")"
            dpK += 1
        EndWhile
        slotWpN[i] = dpN
        slotWpK[i] = 0
        slotDetour[i] = 2
        Standpoint.Log("  s" + i + " DETOUR: a way round with " + (dpN - 1) + " point(s) before her own:" + dpPts)
        Return DetourNext(i, akW, afStamp)
    EndIf
    ; (the review) no way round to walk: the planner sees nothing in the way (the block is a body - the target, the
    ; player in a doorway), or there is none. No second detour in this session: her leg is tried once more as it was,
    ; and if it fails again the usual rules count it at once.
    String dpHow = "the planner sees nothing in the way (the block is a body, or something it cannot see)"
    If dpSt == 0
        dpHow = "the planner found no way round"
    ElseIf dpSt == -2
        dpHow = "the plan was lost"
    ElseIf dpSt < 0
        dpHow = "the planner could not run"
    ElseIf dpSt == 2
        dpHow = "the way round is only her own point"
    EndIf
    slotDetourTries[i] = 2
    Standpoint.Log("  s" + i + " DETOUR: " + dpHow + " (the planner's PATH line says why) - her leg is tried once more as it was")
    Return DetourResume(i, akW, afStamp)
EndFunction

; v1.3.0: send her to the next point of the way round (a DETOUR leg), or - after the last one - back onto her own leg.
Bool Function DetourNext(Int i, Actor akW, Float afStamp)
    Int dnK = slotWpK[i]
    If dnK >= slotWpN[i] - 1
        Return DetourResume(i, akW, afStamp)
    EndIf
    Int dnB = i * 16
    Float dnX = slotWpX[dnB + dnK]
    Float dnY = slotWpY[dnB + dnK]
    Float dnZ = slotWpZ[dnB + dnK]
    Float dnFace = CompassBearing(slotWpX[dnB + dnK + 1] - dnX, slotWpY[dnB + dnK + 1] - dnY)   ; on toward the next point
    slotWpK[i] = dnK + 1
    Bool dnLet = LetGo(i, akW, afStamp, 0.5)
    If !dnLet
        Return False
    EndIf
    Standpoint.Log("  s" + i + " DETOUR: -> point " + (dnK + 1) + " of " + (slotWpN[i] - 1) + ", to (" + (dnX as Int) + "," + (dnY as Int) + ")")
    Return RouteSend(i, akW, afStamp, dnX, dnY, dnZ, dnFace, 4)
EndFunction

; v1.3.0: back onto the leg the way round was for - its own route state and point, so its own arrival rules apply.
Bool Function DetourResume(Int i, Actor akW, Float afStamp)
    slotDetour[i] = 0
    Int drState = slotDefState[i]
    Bool drLet = LetGo(i, akW, afStamp, 0.5)
    If !drLet
        Return False
    EndIf
    Standpoint.Log("  s" + i + " DETOUR: -> back onto her " + RouteKindName(drState) + " leg, to (" + (slotDefX[i] as Int) + "," + (slotDefY[i] as Int) + ")")
    Bool drSent = RouteSend(i, akW, afStamp, slotDefX[i], slotDefY[i], slotDefZ[i], slotDefFace[i], drState)
    If drSent && drState > 0 && drState != 4
        slotRouteLegs[i] = slotRouteLegs[i] - 1      ; the same stop as before, not a new one
    EndIf
    Return drSent
EndFunction

; v1.2.0: "(x,y,z)" of a reference, for the log (report 11 §10: put her position on the session and verdict lines)
String Function PosText(ObjectReference akRef)
    If !akRef
        Return "(none)"
    EndIf
    Return "(" + (akRef.GetPositionX() as Int) + "," + (akRef.GetPositionY() as Int) + "," + (akRef.GetPositionZ() as Int) + ")"
EndFunction

; v1.2.0 (F4, report 11 §7.1): wait UP TO afMax seconds, in 0.25 s steps, and end early once she has stood still (under
; 2 u per step) for 1.5 s AND faces afWantDeg within 5 deg. Returns how long she had stood still when it ended (0 = she
; was moving), or -1.0 when the session changed meanwhile (the caller abandons, as after every wait).
Float Function SettleWait(Int i, Actor akW, Float afStamp, Float afMax, Float afWantDeg)
    Float swWaited = 0.0
    Float swStill = 0.0
    Float swX = akW.GetPositionX()
    Float swY = akW.GetPositionY()
    While swWaited < afMax
        Utility.Wait(0.25)
        swWaited += 0.25
        If !SameSession(i, akW, afStamp)
            Return -1.0
        EndIf
        Float swNX = akW.GetPositionX()
        Float swNY = akW.GetPositionY()
        Float swDX = swNX - swX
        Float swDY = swNY - swY
        swX = swNX
        swY = swNY
        If swDX * swDX + swDY * swDY < 4.0
            swStill += 0.25
        Else
            swStill = 0.0
        EndIf
        Float swErr = NormAng180(akW.GetAngleZ() - afWantDeg)
        If swStill >= 1.5 && swErr <= 5.0 && swErr >= -5.0
            Return swStill
        EndIf
    EndWhile
    Return swStill
EndFunction

; v1.2.0 (F4): after the marker has been moved to where she stands, wait UP TO afMax seconds for the engine to turn her to
; afWantDeg (0.25 s steps; done after two readings in a row within 5 deg). False = the session changed meanwhile.
Bool Function TurnWait(Int i, Actor akW, Float afStamp, Float afMax, Float afWantDeg)
    Float twWaited = 0.0
    Int twGood = 0
    While twWaited < afMax
        Utility.Wait(0.25)
        twWaited += 0.25
        If !SameSession(i, akW, afStamp)
            Return False
        EndIf
        Float twErr = NormAng180(akW.GetAngleZ() - afWantDeg)
        If twErr <= 5.0 && twErr >= -5.0
            twGood += 1
            If twGood >= 2
                Return True
            EndIf
        Else
            twGood = 0
        EndIf
    EndWhile
    Return True
EndFunction

; v0.7.2: her flat distance from slot i's point (the point that was asked for).
Float Function DistToSlot(Actor akW, Int i)
    Float dx = akW.GetPositionX() - slotX[i]
    Float dy = akW.GetPositionY() - slotY[i]
    Return Math.Sqrt(dx * dx + dy * dy)
EndFunction

; The cosine of an angle as words for the log ("87 deg"). (v0.9.6; the glide it was written for is gone since v0.9.8.)
String Function GlideAngleText(Float afCos)
    If afCos < -1.5
        Return "at no measurable angle (no target, or she or the point is within 1 u of it)"
    EndIf
    Float cosClamped = afCos
    If cosClamped > 1.0
        cosClamped = 1.0
    ElseIf cosClamped < -1.0
        cosClamped = -1.0
    EndIf
    Return "" + (Math.acos(cosClamped) as Int) + " deg"
EndFunction

; ===================== v0.9.7 / v0.9.8: LET GO, and STEP BACK AND AROUND =====================
; v0.9.8 rewrote the route after the fourth close-contact run (2026-10-10 10:11, report 05 §16). What that run measured:
;   F1  Within about 50 u of her marker she does NOT walk to it: she poses where she stands (8 of 8 stops: 9-45 u).
;   F2  A walk that STARTS 58 u or more from the marker goes all the way (1.2-3.9 u), also onto a point inside the other
;       NPC's space (28 cm and 44 cm centre to centre, by walking).
;   F3  A marker that is moved while she walks is not followed at once: she always walks to where it WAS first. In the
;       final approach (procedure 1) she then poses there and stays.
;   F4  The let-go works (12 of 12), and the engine runs her own package 0.1-0.25 s after ours is taken off.
; And the user's ruling on the glide: "moving a NPC while they are in the wait pose make her slide on the floor. Let's
; try not to do that again." So in 0.9.8:
;   * THERE IS NO GLIDE. The last stretch is WALKED: every walk to the asked point starts 58 u or more from it.
;   * A stop on the way is a real place she walks to and reaches. The leg changes the moment she is there: she is let
;     go, the next leg is chosen from where she really stands, and her package is put back.
;   * On a stop she stands on the PLAIN marker (no pose): the wait pose is for the asked point only.

; Is slot i still in the hands of the start (or the leg change) that carries afStamp? Unlike SameSession this does not
; ask slotActive: StartSession asks it while the slot of a HELD walker is not active yet.
Bool Function StillThisStart(Int i, Actor akW, Float afStamp)
    If slotWalker[i] != akW || slotStart[i] != afStamp
        Return False
    EndIf
    If slotFreeing
        If slotFreeing[i]
            Return False
        EndIf
    EndIf
    Return True
EndFunction

; v0.9.6 / v0.9.7: LET HER GO. What ends a hold: our package override off, IdleForceDefaultState, EvaluatePackage -
; then time for the engine to really drop it. A walker who is pinned on her marker does not walk when her marker is
; moved and the same override is taken off and put back in one go (measured 2026-10-10, report 05 §14.3).
; The wait: afMinWait, then for as long as the engine still reports OUR package as her current one, 0.25 s at a time,
; at most 2 s more (the log says when that was needed - it never was in the fourth run).
;   1.0 s  a walker who was HELD in her pose (StartSession) - proven 12 of 12 in the fourth run
;   0.5 s  a leg change on a stop (v0.9.8): she has only just stopped there, on a marker without a pose. NOT proven;
;          the engine showed her own package after 0.1-0.25 s every time.
; The caller puts the package back (StartSession, RouteSend). Returns False when the slot was released or taken over
; before or while she was being let go: the caller then stops at once and touches nothing. The first check stands
; DIRECTLY before her package is taken off (the caller's own last check may be a native call old).
Bool Function LetGo(Int i, Actor akW, Float afStamp, Float afMinWait = 1.0)
    If !StillThisStart(i, akW, afStamp)
        Return False
    EndIf
    ActorUtil.RemovePackageOverride(akW, slotPkg[i])
    Debug.SendAnimationEvent(akW, "IdleForceDefaultState")
    akW.EvaluatePackage()
    Utility.Wait(afMinWait)
    If !StillThisStart(i, akW, afStamp)
        Return False
    EndIf
    Int lgOurs = slotPkg[i].GetFormID()
    Float lgMore = 0.0
    While lgMore < 2.0 && Standpoint.GetCurrentPackageFormID(akW) == lgOurs
        Utility.Wait(0.25)
        lgMore += 0.25
        If !StillThisStart(i, akW, afStamp)
            Return False
        EndIf
    EndWhile
    If lgMore > 0.0
        Standpoint.Log("session " + i + ": let go - the engine still ran our package " + afMinWait + " s after it was taken off; waited " + lgMore + " s more")
    EndIf
    Return True
EndFunction

; v0.9.8: WHERE DOES SHE GO NEXT? One question, asked when the request comes in, at every stop, and once more when
; she ended short of the asked point: a walker who stands where she stands now, heading for the asked point
; (afFinX, afFinY) at her target akT. Everything is taken from the real positions at this moment.
; Returns an EMPTY array = "walk to the asked point now", or [x, y, z, heading, kind] = a stop to walk to first:
;   kind 3  APPROACH  the stop the walk in starts from: on the asked side, 95 u from the target or further.
;   kind 1  STEP BACK straight away from the target, on the side she stands on, to 95 u or further.
;   kind 2  AROUND    a point 95 u from the target, half of the way round to the asked side.
; The rules (ang = the angle, seen from the target, between her and the asked point; gap = her distance from it):
;   she is on the asked side (ang 40 deg or less)
;       gap 58 u or more -> empty: she has the run-up, the walk in goes all the way (F2)
;       less             -> APPROACH on the asked side, at least 60 u from her AND from the asked point: she steps
;                           back to it and walks in again (a request for a point 10 u away is otherwise not walked: F1)
;   she is on another side (more than 40 deg)
;       nearer than 60 u to the target -> STEP BACK first (not when abNoBack: a step back that was blocked, or done)
;       more than a quarter turn to go -> AROUND, half of the remaining turn (never across the target)
;       a quarter turn or less         -> the asked point itself when it lies 85 u or more out and 58 u or more from
;                                         her (the straight walk stays clear of the target); else APPROACH
; Every stop lies at least 60 u from her by construction, because a shorter leg would not be walked (F1). The solver
; may swap a blocked hour or slide the point (a stove, a wall): a solved point more than 45 degrees from the wanted
; direction, nearer than 60 u to the target, or nearer than 58 u to her is not used -> empty.
Float[] Function RouteStage(ObjectReference akT, Actor akW, Float afFinX, Float afFinY, Bool abNoBack)
    Float[] rgNone
    Float rgTx = akT.GetPositionX()
    Float rgTy = akT.GetPositionY()
    Float rgTFace = akT.GetAngleZ()
    Float rgPosX = akW.GetPositionX()
    Float rgPosY = akW.GetPositionY()
    Float rgWx = rgPosX - rgTx
    Float rgWy = rgPosY - rgTy
    Float rgFx = afFinX - rgTx
    Float rgFy = afFinY - rgTy
    Float rgLenW = Math.Sqrt(rgWx * rgWx + rgWy * rgWy)
    Float rgLenF = Math.Sqrt(rgFx * rgFx + rgFy * rgFy)
    If rgLenW < 1.0 || rgLenF < 1.0
        Return rgNone
    EndIf
    Float rgGapX = rgPosX - afFinX
    Float rgGapY = rgPosY - afFinY
    Float rgGap = Math.Sqrt(rgGapX * rgGapX + rgGapY * rgGapY)
    Float rgCos = (rgWx * rgFx + rgWy * rgFy) / (rgLenW * rgLenF)
    If rgCos > 1.0
        rgCos = 1.0
    ElseIf rgCos < -1.0
        rgCos = -1.0
    EndIf
    Float rgAng = Math.acos(rgCos)
    Float rgBearW = CompassBearing(rgWx, rgWy)
    Float rgBearF = CompassBearing(rgFx, rgFy)
    Int rgKind = 0
    Float rgBear = 0.0                  ; the compass bearing, from the target, of the stop
    Float rgDist = 95.0                 ; its distance from the target, u
    If rgAng <= 40.0
        If rgGap >= 58.0
            Return rgNone
        EndIf
        rgKind = 3
        rgBear = rgBearF
        If rgLenW + 60.0 > rgDist
            rgDist = rgLenW + 60.0
        EndIf
        If rgLenF + 60.0 > rgDist
            rgDist = rgLenF + 60.0
        EndIf
    ElseIf rgLenW < 60.0 && !abNoBack
        rgKind = 1
        rgBear = rgBearW
        If rgLenW + 60.0 > rgDist
            rgDist = rgLenW + 60.0
        EndIf
    ElseIf rgAng > 90.0
        rgKind = 2
        rgBear = rgBearW + NormAng180(rgBearF - rgBearW) / 2.0
    Else
        If rgLenF >= 85.0 && rgGap >= 58.0
            Return rgNone
        EndIf
        rgKind = 3
        rgBear = rgBearF
        If rgLenF + 60.0 > rgDist
            rgDist = rgLenF + 60.0
        EndIf
        ; her distance from that stop: under 60 u she would not walk it (F1) -> the stop moves further out
        Float rgLegSq = rgLenW * rgLenW + rgDist * rgDist - 2.0 * rgLenW * rgDist * rgCos
        If rgLegSq < 3600.0
            If rgLenW + 60.0 > rgDist
                rgDist = rgLenW + 60.0
            EndIf
        EndIf
    EndIf
    Float rgHour = NormAng360(rgBear - rgTFace) / 30.0
    Float[] rgS = Standpoint.SolveClock(akT, akW, rgDist * 1.428, rgHour, 12.0, 1)
    If rgS.Length < 7
        Return rgNone
    EndIf
    Float rgQx = rgS[0] - rgTx
    Float rgQy = rgS[1] - rgTy
    Float rgLenQ = Math.Sqrt(rgQx * rgQx + rgQy * rgQy)
    If rgLenQ < 60.0
        Standpoint.Log("  ROUTE: no stop - the solved point lies only " + (rgLenQ as Int) + " u from her target")
        Return rgNone
    EndIf
    Float rgQCos = (rgQx * Math.Sin(rgBear) + rgQy * Math.Cos(rgBear)) / rgLenQ
    If rgQCos < 0.707
        Standpoint.Log("  ROUTE: no stop - the solved point lies " + GlideAngleText(rgQCos) + " from the wanted direction (blocked there)")
        Return rgNone
    EndIf
    Float rgLegX = rgS[0] - rgPosX
    Float rgLegY = rgS[1] - rgPosY
    Float rgLeg = Math.Sqrt(rgLegX * rgLegX + rgLegY * rgLegY)
    If rgLeg < 58.0
        Standpoint.Log("  ROUTE: no stop - the solved point lies only " + (rgLeg as Int) + " u from her (she would not walk that)")
        Return rgNone
    EndIf
    Float[] rgOut = new Float[5]
    rgOut[0] = rgS[0]
    rgOut[1] = rgS[1]
    rgOut[2] = rgS[2]
    rgOut[3] = rgS[3]
    rgOut[4] = rgKind as Float
    Return rgOut
EndFunction

; The name of a stop kind, for the log.
String Function RouteKindName(Int aiKind)
    If aiKind == 1
        Return "STEP BACK"
    ElseIf aiKind == 2
        Return "AROUND"
    ElseIf aiKind == 3
        Return "APPROACH"
    ElseIf aiKind == 4
        Return "DETOUR"          ; v1.3.0: a point of a planned way round
    EndIf
    Return "IN"
EndFunction

; Point slot i's session at its next leg and put her package back. The CALLER has let her go (LetGo): her package is
; off. (afX, afY, afZ, afFace) becomes the point she is walked to and aiState the session's route state: 1 / 2 / 3 =
; a stop on the way (RouteStage's kinds), 0 = the asked point, under the normal arrival rules.
; The marker: a stop uses the slot's PLAIN marker (base 0x80F, no pose), the asked point the marker the session asked
; for (slotFinPlain). The alias is forced to it while her package is off.
; It is a fresh start, with a fresh start's stall rule (the clock runs from the first poll that sees our package; 8 s
; until she has moved).
; The slot's fields are written in ONE piece right after a SameSession check, with no call out of the script in
; between. Returns False when the session is no longer this one: the caller leaves at once.
Bool Function RouteSend(Int i, Actor akW, Float afStamp, Float afX, Float afY, Float afZ, Float afFace, Int aiState)
    If afFace >= 359.95
        afFace = 0.0
    EndIf
    Bool rsPlain = True
    If aiState == 0
        rsPlain = slotFinPlain[i]
    EndIf
    If rsPlain
        If !slotMarkerPlain[i]
            Form rsBase = Game.GetFormFromFile(0x80F, "Standpoint.esp")
            If rsBase
                ObjectReference rsNew = akW.PlaceAtMe(rsBase, 1, True, False)     ; v1.6.0 BUG 1: made at the walker
                If !slotMarkerPlain[i]
                    slotMarkerPlain[i] = rsNew      ; (the slot's own reusable marker - it belongs to no session)
                EndIf
            EndIf
        EndIf
        If !slotMarkerPlain[i]
            rsPlain = False                         ; an older Standpoint.esp: the standard marker everywhere
        EndIf
    EndIf
    ReferenceAlias rsAlias = GetOwningQuest().GetAlias(i + 1) as ReferenceAlias
    ObjectReference rsAnchor = AnchorFor(i, akW)    ; v1.6.0 BUG 1: the session's target (a relative session), else the walker
    Float rsWx = akW.GetPositionX()
    Float rsWy = akW.GetPositionY()
    Float rsNow = Utility.GetCurrentRealTime()
    If !SameSession(i, akW, afStamp)
        Return False
    EndIf
    slotX[i] = afX
    slotY[i] = afY
    slotZ[i] = afZ
    slotFaceDeg[i] = afFace
    slotStallX[i] = rsWx
    slotStallY[i] = rsWy
    slotStallT[i] = rsNow
    slotSeenOurs[i] = False
    slotHasMoved[i] = False
    slotSawTravel[i] = False
    slotLastIdx[i] = -99
    slotBlockPolls[i] = 0
    slotLegKicks[i] = 0                     ; v1.3.0: a new leg (LegKick sets it back to 2 when this is its own re-send)
    slotPlain[i] = rsPlain
    slotRoute[i] = aiState
    If aiState > 0 && aiState != 4          ; (v1.3.0: the points of a way round are not stops of the route - no count)
        slotRouteLegs[i] = slotRouteLegs[i] + 1
    EndIf
    ObjectReference rsMk = CurMarker(i)
    PlaceMarker(rsMk, rsAnchor, afX, afY, afZ, afFace)    ; v1.6.0 BUG 1: ONE cross-cell move + the heading (was SetPosition)
    If !SameSession(i, akW, afStamp)
        Return False
    EndIf
    If rsAlias
        rsAlias.ForceRefTo(rsMk)
        If !SameSession(i, akW, afStamp)
            Return False
        EndIf
    EndIf
    ActorUtil.AddPackageOverride(akW, slotPkg[i], 95, 1)
    Utility.Wait(0.1)
    If !SameSession(i, akW, afStamp)
        Return False
    EndIf
    akW.EvaluatePackage()
    Return True
EndFunction

; ONE poll of a session that is walking to a stop of its route (slotRoute 1 = her step back, 2 = a point on the way
; around, 3 = the approach point).
;   The leg is OVER when she has REACHED the stop (she is within 8 u of it, or has stopped within 25 u of it), or
;   when she stands still elsewhere (the stall clock of close mode: 3 s once she has moved, 8 s if she has not moved
;   since she was started - something is in her way).
;   Then, at once: she is let go (0.5 s), the next leg is chosen from where she stands, and her package is put back.
;     after ANY stop she reached              -> RouteStage (never a second step back after one). From the approach
;                                                stop it answers "walk in" - unless she is not where she should be
;                                                (she drifted while she was let go): then she gets the stop she needs.
;   v0.9.9 - BLOCKED: the engine's own "movement blocked" (run-once package type 36) with no movement for 1 s ends
;   the leg at once, like a stall. And a leg that FAILED (blocked, or she stood still) has these other ways:
;     a failed STEP BACK                      -> the way around from where she stands (RouteStage without a step back)
;     the FIRST failed leg on the way round   -> one more try from where she stands (RouteStage; "walk in" if she
;       (around / approach)                      has the run-up on the asked side)
;     the SECOND failed leg on the way round  -> NO WAY: the request answers O 17, Standpoint_Failed 7, released
;     she has not moved at all for 8 s        -> as any failed leg (above); and no second walk in later (slotRetry):
;       (and the engine did not say "blocked")   a walker who does not walk gains nothing from it.
;     more than 6 stops (a loop guard)        -> the asked point
; slotArrivalBusy is held while this runs: one stack only (the same lock as the arrival block, which a session on its
; route never enters).
; Returns False when the session is no longer this one (released or replaced meanwhile): PollSlot leaves at once.
Bool Function RoutePoll(Int i, Actor akW, Float afNow)
    Float rpStamp = slotStart[i]
    If slotArrivalBusy[i] == rpStamp
        Return True                     ; another stack is setting her next leg right now
    EndIf
    slotArrivalBusy[i] = rpStamp
    ; v1.3.0: while a way round is being planned, nothing else is looked at (DetourPoll)
    If slotDetour
        If slotDetour[i] == 1
            Bool rpPlanAlive = DetourPoll(i, akW, rpStamp, afNow)
            If !rpPlanAlive
                Return False
            EndIf
            If slotArrivalBusy[i] == rpStamp
                slotArrivalBusy[i] = 0.0
            EndIf
            Return True
        EndIf
    EndIf
    ObjectReference rpT = slotFaceRef[i]
    Float rpPosX = akW.GetPositionX()
    Float rpPosY = akW.GetPositionY()
    Float rpTx = 0.0
    Float rpTy = 0.0
    If rpT
        rpTx = rpT.GetPositionX()
        rpTy = rpT.GetPositionY()
    EndIf
    Int rpRunOnce = Standpoint.GetRunOncePackageTypeId(akW)     ; 36 = the engine's own "movement blocked"
    If !SameSession(i, akW, rpStamp)
        Return False                    ; (the calls above are native calls: another stack may have run)
    EndIf
    Float rpDx = rpPosX - slotX[i]
    Float rpDy = rpPosY - slotY[i]
    Float rpD = Math.Sqrt(rpDx * rpDx + rpDy * rpDy)    ; her flat distance from the stop she is walking to
    Float rpLenW = 0.0                                  ; her flat distance from her target
    If rpT
        Float rpWx = rpPosX - rpTx
        Float rpWy = rpPosY - rpTy
        rpLenW = Math.Sqrt(rpWx * rpWx + rpWy * rpWy)
    EndIf
    ; Math.Sqrt is a call out of this script too, so the session is asked once more. From here to the first Log /
    ; LetGo below there is no such call: the slot is read and written in one piece.
    If !SameSession(i, akW, rpStamp)
        Return False
    EndIf
    Int rpState = slotRoute[i]
    ; the stall clock (the same fields and the same times as close mode's stall rule)
    Bool rpStalled = False
    Bool rpNeverMoved = False
    Bool rpStood = False
    Bool rpBlocked = False
    If slotSeenOurs[i]
        Float rpSx = rpPosX - slotStallX[i]
        Float rpSy = rpPosY - slotStallY[i]
        Float rpNeed = 8.0
        If slotHasMoved[i]
            rpNeed = 3.0
        EndIf
        If rpSx * rpSx + rpSy * rpSy >= 16.0
            slotHasMoved[i] = True
            slotStallX[i] = rpPosX
            slotStallY[i] = rpPosY
            slotStallT[i] = afNow
            slotBlockPolls[i] = 0
        Else
            If rpRunOnce == 36
                slotBlockPolls[i] = slotBlockPolls[i] + 1
            Else
                slotBlockPolls[i] = 0
            EndIf
            If slotHasMoved[i] && afNow - slotStallT[i] >= 0.4
                rpStood = True          ; she has walked on this leg and did not move since the poll before this one
            EndIf
            If afNow - slotStallT[i] >= rpNeed
                rpStalled = True
            ElseIf slotBlockPolls[i] >= 2 && afNow - slotStallT[i] >= 1.0
                ; v0.9.9: the engine itself says MOVEMENT BLOCKED - on two polls in a row, so one passing reading
                ; cannot end a leg - and she has not moved for a second: the leg has failed NOW; no waiting out
                ; the 3 s / 8 s (the user: "if the path is blocked, making the NPCs actually react right away is
                ; probably the best thing to do"). In the three logged runs type 36 showed in three episodes, each a
                ; real block with no movement at all, and it was gone the moment she was let go.
                rpStalled = True
                rpBlocked = True
            EndIf
            If rpStalled
                If !slotHasMoved[i]
                    rpNeverMoved = True
                EndIf
            EndIf
        EndIf
    EndIf
    ; REACHED = she is ON the stop (8 u: the fourth run's walks ended 1.2-3.9 u from the marker), or she has STOPPED
    ; within 25 u of it. Never while she is still walking: let go 25 u short of an approach stop she would stand
    ; nearer than 58 u to the asked point, and the walk in would not be walked (the review of 0.9.8).
    Bool rpWayFailed = False            ; a leg on the way round (around / approach) was not reached
    Bool rpReached = False
    If rpD < 8.0
        rpReached = True
    ElseIf rpD < 25.0 && rpStood
        rpReached = True
    ElseIf rpState == 4 && rpD < 60.0 && slotSeenOurs[i] && !slotHasMoved[i] && afNow - slotStallT[i] >= 1.0
        ; v1.3.0 (the review): a point of a way round less than 60 u away - the engine poses her there instead of walking
        ; (the 0.9.8 measurement), so waiting for a walk that will not come is pointless: the point counts as reached
        rpReached = True
    EndIf
    ; v1.3.0 - LEG START: a leg that has not started is kicked first (LegKick: 2 s evaluate, 4 s the same leg again);
    ; only 8 s after its last kick does it count as failed (rpNeverMoved above).
    If !rpReached && !rpStalled && slotSeenOurs[i] && !slotHasMoved[i] && slotBlockPolls[i] < 2
        Bool rpKick = LegKick(i, akW, rpStamp, afNow)
        If !rpKick
            Return False
        EndIf
    EndIf
    If !rpReached && !rpStalled
        If slotArrivalBusy[i] == rpStamp
            slotArrivalBusy[i] = 0.0
        EndIf
        Return True
    EndIf
    String rpLegName = RouteKindName(rpState)
    String rpWhy = "she has reached her " + rpLegName + " stop, " + (rpLenW as Int) + " u from her target"
    If !rpReached
        If rpBlocked
            rpWhy = "the engine reports MOVEMENT BLOCKED on her " + rpLegName + " leg and she has not moved for 1 s (" + (rpLenW as Int) + " u from her target, " + (rpD as Int) + " u from the stop)"
        ElseIf rpNeverMoved
            rpWhy = "she has not moved for 8 s since she was started on her " + rpLegName + " leg (" + (rpLenW as Int) + " u from her target, " + (rpD as Int) + " u from the stop)"
        Else
            rpWhy = "she has stood still for 3 s on her " + rpLegName + " leg, " + (rpLenW as Int) + " u from her target and " + (rpD as Int) + " u from the stop"
        EndIf
        ; v1.3.0 - a DETOUR leg that failed: the way round is given up; her own leg is tried once more as it was
        If rpState == 4
            Standpoint.Log("  s" + i + " DETOUR: " + rpWhy + " - the way round is given up")
            Bool rpBack = DetourResume(i, akW, rpStamp)
            If !rpBack
                Return False
            EndIf
            If slotArrivalBusy[i] == rpStamp
                slotArrivalBusy[i] = 0.0
            EndIf
            Return True
        EndIf
        ; v1.3.0 - THE DETOUR: blocked, or she stood still after walking, with 60 u or more still to go -> a way round
        ; is planned before the leg counts as failed (a walker who never moved gains nothing from it)
        If !rpNeverMoved && rpD >= 60.0 && slotDetourTries[i] < 2
            Bool rpDet = DetourStart(i, akW, rpStamp, rpWhy)
            If !rpDet
                Return False
            EndIf
            If slotArrivalBusy[i] == rpStamp
                slotArrivalBusy[i] = 0.0
            EndIf
            Return True
        EndIf
        If rpNeverMoved && !rpBlocked
            slotRetry[i] = 1            ; she does not walk at all: no second walk in later
        EndIf
        ; v0.9.9: a leg on the way round (around / approach) that FAILED.
        ;   she stands NEAR the asked point (RouteNear) -> she is sent IN: the arrival block takes her where she
        ;       stands (C 9). (The review: asked for 30 cm from 45 cm with a wall behind her, she cannot step back to
        ;       her approach stop - she is 10 u from the point, that is not "no way".)
        ;   else the FIRST failure -> one more try from where she stands (RouteStage below)
        ;   else the SECOND       -> NO WAY: the request answers O 17, Standpoint_Failed 7, and she is released.
        ; (A failed STEP BACK does not count: its other way is the way around, below.)
        If rpState != 1
            rpWayFailed = True
        EndIf
    EndIf
    ; v1.3.0: a point of the way round reached -> the next one, or back onto her own leg
    If rpReached && rpState == 4
        Standpoint.Log("  s" + i + " DETOUR: she has reached point " + slotWpK[i] + " of " + (slotWpN[i] - 1))
        Bool rpOn = DetourNext(i, akW, rpStamp)
        If !rpOn
            Return False
        EndIf
        If slotArrivalBusy[i] == rpStamp
            slotArrivalBusy[i] = 0.0
        EndIf
        Return True
    EndIf
    Bool rpGoIn = False
    If rpWayFailed
        Bool rpNear = RouteNear(i, akW)
        If !SameSession(i, akW, rpStamp)
            Return False                ; (RouteNear reads positions: another stack may have run)
        EndIf
        If rpNear
            rpGoIn = True
            slotRetry[i] = 1            ; no second walk in: she cannot step back to walk in from further out
        ElseIf slotRouteFails[i] >= 1
            Standpoint.Log("  s" + i + " ROUTE: " + rpWhy + " - a second leg on the way round has failed: NO WAY to the asked point. Released, Failed 7")
            If !SameSession(i, akW, rpStamp)
                Return False
            EndIf
            ApiReport(i, "O", 17)
            SendFailed(akW, 7.0)
            If SameSession(i, akW, rpStamp)
                FreeSession(i)
            EndIf
            Return False
        Else
            slotRouteFails[i] = 1
        EndIf
    EndIf
    If rpGoIn
        rpWhy = rpWhy + "; she stands near the asked point"
    EndIf
    Standpoint.Log("  s" + i + " ROUTE: " + rpWhy + " - she is let go for her next leg")
    Bool rpLetGo = LetGo(i, akW, rpStamp, 0.5)
    If !rpLetGo
        Standpoint.Log("  s" + i + " ROUTE: the session changed while she was being let go - left alone")
        Return False
    EndIf
    ; where next - decided from where she stands NOW (during the let-go she may have moved)
    Bool rpAsk = True
    Bool rpNoBack = False
    If !rpT || rpGoIn
        rpAsk = False
    ElseIf slotRouteLegs[i] >= 6
        rpAsk = False
    ElseIf rpState == 1 || !rpReached
        rpNoBack = True                 ; never a second step back: not after one, not in the middle of a route
    EndIf
    Float[] rpNext
    If rpAsk
        rpNext = RouteStage(rpT, akW, slotFinX[i], slotFinY[i], rpNoBack)
        If !SameSession(i, akW, rpStamp)
            Return False
        EndIf
    EndIf
    Bool rpSent = False
    If rpNext.Length >= 5
        Int rpKind = rpNext[4] as Int
        Standpoint.Log("  s" + i + " ROUTE: -> " + RouteKindName(rpKind) + ", to (" + (rpNext[0] as Int) + "," + (rpNext[1] as Int) + ")")
        rpSent = RouteSend(i, akW, rpStamp, rpNext[0], rpNext[1], rpNext[2], rpNext[3], rpKind)
    Else
        Standpoint.Log("  s" + i + " ROUTE: -> IN, to the asked point (" + (slotFinX[i] as Int) + "," + (slotFinY[i] as Int) + ")")
        rpSent = RouteSend(i, akW, rpStamp, slotFinX[i], slotFinY[i], slotFinZ[i], slotFinFace[i], 0)
    EndIf
    If !rpSent
        Standpoint.Log("  s" + i + " ROUTE: the session changed while her next leg was being set - left alone")
        Return False
    EndIf
    If slotArrivalBusy[i] == rpStamp
        slotArrivalBusy[i] = 0.0
    EndIf
    Return True
EndFunction

; v0.9.9: is she NEAR the asked point - on the asked side of her target (within 40 degrees, seen from it) and within
; 35 u (50 cm) of the point? That is "stopped short" (C 9). Anything else at the end of a close request to an NPC is
; "she could not get there" (O 17, NO WAY). Only read for a session with slotRouteOk.
Bool Function RouteNear(Int i, Actor akW)
    ObjectReference rnT = slotFaceRef[i]
    Float rnPosX = akW.GetPositionX()
    Float rnPosY = akW.GetPositionY()
    Float rnGx = rnPosX - slotFinX[i]
    Float rnGy = rnPosY - slotFinY[i]
    Float rnGapSq = rnGx * rnGx + rnGy * rnGy
    If rnGapSq > 1225.0
        Return False                    ; more than 35 u from the point
    EndIf
    If rnGapSq <= 144.0
        Return True                     ; within 12 u of the point: the accepted miss, whatever the side
    EndIf
    If !rnT
        Return True
    EndIf
    Float rnTx = rnT.GetPositionX()
    Float rnTy = rnT.GetPositionY()
    Float rnWx = rnPosX - rnTx
    Float rnWy = rnPosY - rnTy
    Float rnFx = slotFinX[i] - rnTx
    Float rnFy = slotFinY[i] - rnTy
    Float rnLenW = Math.Sqrt(rnWx * rnWx + rnWy * rnWy)
    Float rnLenF = Math.Sqrt(rnFx * rnFx + rnFy * rnFy)
    If rnLenW < 1.0 || rnLenF < 12.0
        Return True                     ; (no side to compare: she is on the target's centre, or the point lies within 12 u of it)
    EndIf
    If (rnWx * rnFx + rnWy * rnFy) / (rnLenW * rnLenF) < 0.766
        Return False                    ; more than 40 degrees round the target from the asked side
    EndIf
    Return True
EndFunction

Function ExecutePose(Int i)
    Actor w = slotWalker[i]
    If !w
        Return
    EndIf
    Int mode = slotPose[i]
    If mode == 1
        ; auto: target height above the stand-point decides (mirrors the engine's
        ; IdleActivatePickUp >= 41 / Low <= 40 selector)
        Float rel = slotDestZ[i] - w.GetPositionZ()
        If rel < 40.0
            mode = 2
        Else
            mode = 3
        EndIf
    EndIf
    If mode == 2
        ; kneel to floor: idlepickup_ground.hkx - 2.80 s, ZERO root translation
        ; (measured), so the arrival precision survives the clip
        Debug.SendAnimationEvent(w, "IdlePickup_Ground")
        Standpoint.Log("  s" + i + " POSE kneel (ground)")
    ElseIf mode == 3
        ; standing bend to a raised surface - the exact event SeverActions ships
        ; for guards taking items from tables
        Debug.SendAnimationEvent(w, "IdlePickupFromTableStart")
        Standpoint.Log("  s" + i + " POSE bend (table)")
    EndIf
    Utility.Wait(3.2)
    ; one-shot clips return to the marker idle on their own; nothing to reset here
EndFunction

Function SendInterrupted(Actor akWalker, Float afCode)
    ; 1 = combat, 2 = player bump/body-block, 3 = player hand-grab (VR),
    ; 4 = danger on/near the route (session is DROPPED unless aiDanger == 1)
    Int h = ModEvent.Create("Standpoint_Interrupted")
    If h
        ModEvent.PushForm(h, akWalker)
        ModEvent.PushFloat(h, afCode)
        ModEvent.Send(h)
    EndIf
EndFunction

Function SendFailed(Actor akWalker, Float afCode)
    Int h = ModEvent.Create("Standpoint_Failed")
    If h
        ModEvent.PushForm(h, akWalker)
        ModEvent.PushFloat(h, afCode)
        ModEvent.Send(h)
    EndIf
EndFunction

Event OnTravelComplete(String asTagStatus, Float afNumArg, Form akSender)
    If StringUtil.Find(asTagStatus, "standpoint_") == 0
        Standpoint.Log("  SA travel: " + asTagStatus)
    EndIf
EndEvent

Event OnUpdate()
    If running
        S1Poll()
        Return
    EndIf
    Bool anyActive = False
    Bool anyTimedHold = False
    Bool anyLeashedHold = False
    Bool anySteerStand = False      ; v1.6.0: a steered session held by the walk's own stand (its end is polled)
    EnsureSteerArrays()
    Float now = Utility.GetCurrentRealTime()
    Int i = 0
    While i < 4
        If slotActive[i]
            PollSlot(i)
            If slotActive[i]
                anyActive = True
            EndIf
        EndIf
        If slotHolding[i]
            Actor hw = slotWalker[i]
            If hw && slotLeash[i] > 0.0 && !slotDestIsPlayer[i] && hw.GetDistance(Game.GetPlayer()) > slotLeash[i]
                ; arrived-and-holding, but the player has moved on: release her
                ; back to her own AI (mirrors "hold expired" - no Failed event,
                ; the session already Arrived)
                Standpoint.Log("session " + i + " leash exceeded while holding -> released")
                FreeSession(i)
            ElseIf slotHoldUntil[i] > 0.0
                If now >= slotHoldUntil[i]
                    Standpoint.Log("session " + i + " hold expired")
                    FreeSession(i)
                Else
                    anyTimedHold = True
                EndIf
            ElseIf slotLeash[i] > 0.0
                anyLeashedHold = True
            EndIf
        EndIf
        ; v1.6.0 (S1 §2.5): a steered session held by the WALK's stand (off the navmesh, no pose): an end of that stand reported
        ; by the walk (a grab, combat, her animation) frees the session silently, as a hold expiry
        If slotHolding[i] && slotSteerHold[i] == 2
            Int hs = slotSteer[i]
            If hs > 0
                Float[] hst = Standpoint.SteerState(hs)
                If slotHolding[i] && slotSteer[i] == hs
                    Int hPhase = -1
                    If hst.Length >= 19
                        hPhase = hst[0] as Int
                    EndIf
                    If hPhase == 4 || hPhase == 7 || hPhase < 0     ; (FIX ROUND: 7 = her stand ended after the arrival)
                        Standpoint.Log("session " + i + " hold expired (STEER #" + hs + ": her stand ended - the walk's own word is in its WALK line)")
                        FreeSession(i)
                    Else
                        anySteerStand = True
                    EndIf
                EndIf
            EndIf
        EndIf
        i += 1
    EndWhile
    If anyActive
        RegisterForSingleUpdate(0.5)
    ElseIf anyTimedHold || anySteerStand
        RegisterForSingleUpdate(1.0)
    ElseIf anyLeashedHold
        RegisterForSingleUpdate(2.0)
    EndIf
EndEvent

; ============================ S2 SPELL HARNESS ==============================

Function HandleSlotCast(ObjectReference akTarget)
    If running
        Return
    EndIf
    If !akTarget || (akTarget as Actor) == Game.GetPlayer()
        Return
    EndIf
    Actor asActor = akTarget as Actor
    If asActor
        If s2Walker == None
            s2Walker = asActor
            Debug.Notification("Standpoint: walker = " + s2Walker.GetDisplayName())
            Standpoint.Log("S2 walker set: " + s2Walker)
            Return
        ElseIf asActor == s2Walker
            StandpointAPI.Release(s2Walker)
            Debug.Notification("Standpoint: walker released")
            s2Walker = None
            s2Dest = None
            s2Rel = 0
            Return
        EndIf
    EndIf
    If s2Walker == None
        Debug.Notification("Standpoint: cast at an NPC first to pick the walker")
        Return
    EndIf

    If akTarget == s2Dest
        s2Rel += 1
        If s2Rel > 4
            s2Rel = 0
        EndIf
    Else
        s2Rel = 1
        s2Dest = akTarget
        DumpMarkers(akTarget)
    EndIf
    Int pose = 0
    If !(akTarget as Actor) && Standpoint.GetFurnitureMarkerCount(akTarget) <= 0
        pose = 1   ; loose objects: kneel or bend by height, eye-verifiable
    EndIf
    Int h = StandpointAPI.GoTo(s2Walker, akTarget, s2Rel, 0.0, -1.0, pose)
    If h > 0
        Debug.Notification("Standpoint: " + RelName(s2Rel) + " (session " + h + ")")
    Else
        Debug.Notification("Standpoint: no " + RelName(s2Rel) + " slot (see log)")
    EndIf
EndFunction

Function SendWalkerToWater()
    If s2Walker == None
        Debug.Notification("Standpoint: cast at an NPC first to pick the walker")
        Return
    EndIf
    If WaterFinder.GetPluginVersion() == ""
        Debug.Notification("Standpoint: Water Finder not installed")
        Return
    EndIf
    ; detection only - requireNavigable=False deliberately: Water Finder's navigable
    ; check rides the MUTATING CanNavigateToPosition native (KNOWLEDGEBASE 2026-08-15);
    ; reachability is Standpoint+SeverActions' job anyway
    Bool found = WaterFinder.RequestNearestWater(s2Walker, 160.0, False)
    If !found
        Debug.Notification("Standpoint: no water found within range")
        Standpoint.Log("WATER: none found for " + s2Walker)
        Return
    EndIf
    Float[] shore = WaterFinder.GetShorePosition(s2Walker)
    Float[] wet = WaterFinder.GetWaterPosition(s2Walker)
    If shore.Length < 3 || wet.Length < 3
        Debug.Notification("Standpoint: water result incomplete (see log)")
        Standpoint.Log("WATER: shore.len=" + shore.Length + " wet.len=" + wet.Length)
        Return
    EndIf
    ; face from the shore toward the water (compass degrees; Papyrus trig is degrees,
    ; atan2 rebuilt with the transposed convention: bearing = atan2(dx, dy))
    Float dx = wet[0] - shore[0]
    Float dy = wet[1] - shore[1]
    Float facing = CompassBearing(dx, dy)
    String summary = WaterFinder.GetWaterSummary(s2Walker)
    Standpoint.Log("WATER: " + summary + " | shore=(" + (shore[0] as Int) + "," + (shore[1] as Int) + "," + (shore[2] as Int) + ") face=" + (facing as Int))
    Int h = StandpointAPI.GoToPoint(s2Walker, shore[0], shore[1], shore[2], facing)
    If h > 0
        s2WadePending = True
        s2WadeX = wet[0]
        s2WadeY = wet[1]
        s2WadeZ = wet[2]
        Debug.Notification("Standpoint: to water - " + summary)
    Else
        Debug.Notification("Standpoint: could not start (see log)")
    EndIf
EndFunction

Function ExecuteWade(Int i)
    Actor w = slotWalker[i]
    s2WadePending = False
    If !w
        Return
    EndIf
    Standpoint.Log("  s" + i + " WADE: shore reached, gliding last step to (" + (s2WadeX as Int) + "," + (s2WadeY as Int) + "," + (s2WadeZ as Int) + ")")
    ; the package pin would fight the translate - drop it first (she stays put;
    ; the glide starts immediately)
    ActorUtil.RemovePackageOverride(w, slotPkg[i])
    w.EvaluatePackage()
    w.TranslateTo(s2WadeX, s2WadeY, s2WadeZ, 0.0, 0.0, w.GetAngleZ(), 60.0, 90.0)
    Float t = 0.0
    Bool inWater = False
    While t < 15.0 && !inWater
        Utility.Wait(0.5)
        t += 0.5
        inWater = WaterFinder.IsActorInWater(w)
    EndWhile
    w.StopTranslation()
    Float ddx = w.GetPositionX() - s2WadeX
    Float ddy = w.GetPositionY() - s2WadeY
    Float d = Math.Sqrt(ddx * ddx + ddy * ddy)
    Standpoint.Log("  s" + i + " WADE result: inWater=" + inWater + " dist=" + d + " t=" + t + "s")
    If inWater
        Debug.Notification("Standpoint: she is IN the water")
    Else
        Debug.Notification("Standpoint: wade incomplete (d=" + (d as Int) + ", see log)")
    EndIf
    ; session bookkeeping only - no pose reset while she stands in water
    ReferenceAlias a = GetOwningQuest().GetAlias(i + 1) as ReferenceAlias
    If a
        a.Clear()
    EndIf
    slotWalker[i] = None
    slotActive[i] = False
    slotHolding[i] = False
    Standpoint.Log("session " + i + " released (wade)")
EndFunction

Float Function CompassBearing(Float afDx, Float afDy)
    ; Skyrim compass: 0 = north(+Y), 90 = east(+X), clockwise. Papyrus Math has no
    ; Atan2 - rebuild with quadrant fixes on Atan (degrees).
    If afDy == 0.0
        If afDx >= 0.0
            Return 90.0
        EndIf
        Return 270.0
    EndIf
    Float a = Math.Atan(afDx / afDy)
    If afDy < 0.0
        a += 180.0
    ElseIf afDx < 0.0
        a += 360.0
    EndIf
    Return a
EndFunction

Function DumpMarkers(ObjectReference akDest)
    Int n = Standpoint.GetFurnitureMarkerCount(akDest)
    Standpoint.Log("S2 DEST " + akDest + " base=" + akDest.GetBaseObject() + " markers=" + n)
    Int i = 0
    While i < n
        Float[] mi = Standpoint.GetFurnitureMarkerInfo(akDest, i)
        If mi.Length == 7
            Standpoint.Log("  marker " + i + ": pos=(" + (mi[0] as Int) + "," + (mi[1] as Int) + "," + (mi[2] as Int) + ") face=" + (mi[3] as Int) + " points=" + (mi[4] as Int) + " anim=" + (mi[5] as Int) + " enabled=" + (mi[6] as Int))
        EndIf
        i += 1
    EndWhile
EndFunction

String Function RelName(Int aiRel)
    If aiRel == 1
        Return "FRONT"
    ElseIf aiRel == 2
        Return "RIGHT"
    ElseIf aiRel == 3
        Return "BACK"
    ElseIf aiRel == 4
        Return "LEFT"
    EndIf
    Return "NEAREST"
EndFunction

; ======================== S1 LEGACY MEASUREMENT TEST ========================
; (unchanged protocol; uses alias 1 / package 0x803 = slot 0's resources.
;  Do not run while API sessions are active.)

Function StartTest(Actor akNpc)
    If running
        StopTest("aborted by re-cast")
        Return
    EndIf
    If !parkPkg
        Setup()
        If !parkPkg
            Return
        EndIf
    EndIf
    targetNpc = akNpc
    running = True
    phase = 1
    cycle = 0
    timeouts = 0
    savedAvoid = Standpoint.GetEngineSettingFloat("fAvoidPlayerDistance")
    Standpoint.Log("=== S1 START target=" + targetNpc + " fAvoidPlayerDistance=" + savedAvoid)
    NextCycle()
EndFunction

Function NextCycle()
    cycle += 1
    If phase == 1 && cycle > CyclesPhase1
        phase = 2
        cycle = 1
        Standpoint.Log("--- phase B: 55u from player, facing player ---")
    EndIf
    If phase == 2 && cycle > CyclesPhase2
        StopTest("complete")
        Return
    EndIf
    Actor player = Game.GetPlayer()
    If !marker
        Form idlmBase = Game.GetFormFromFile(0x802, "Standpoint.esp")
        marker = player.PlaceAtMe(idlmBase, 1, True, False)
    EndIf
    Float dx
    Float dy
    Float dist
    If phase == 1
        Float bearing = Utility.RandomFloat(0.0, 360.0)
        dist = Utility.RandomFloat(250.0, 450.0)
        faceTarget = Utility.RandomFloat(0.0, 360.0)
        dx = Math.Sin(bearing) * dist
        dy = Math.Cos(bearing) * dist
    Else
        Float pAng = player.GetAngleZ()
        dist = 55.0
        dx = Math.Sin(pAng) * dist
        dy = Math.Cos(pAng) * dist
        faceTarget = NormAng360(pAng + 180.0)
        If cycle == 4
            Standpoint.SetEngineSettingFloat("fAvoidPlayerDistance", 25.0)
        EndIf
    EndIf
    marker.SetPosition(player.GetPositionX() + dx, player.GetPositionY() + dy, player.GetPositionZ())
    marker.SetAngle(0.0, 0.0, faceTarget)
    ReferenceAlias markerAlias = GetOwningQuest().GetAlias(1) as ReferenceAlias
    markerAlias.ForceRefTo(marker)
    ActorUtil.AddPackageOverride(targetNpc, parkPkg, 95, 1)
    Utility.Wait(0.1)
    targetNpc.EvaluatePackage()
    lastIdx = -99
    settleStart = 0.0
    cycleStart = Utility.GetCurrentRealTime()
    Standpoint.Log("cycle " + phase + "." + cycle + ": d=" + (dist as Int) + " face=" + (faceTarget as Int))
    RegisterForSingleUpdate(0.5)
EndFunction

Function S1Poll()
    Float now = Utility.GetCurrentRealTime()
    Int idx = Standpoint.GetCurrentProcedureIndex(targetNpc)
    If idx != lastIdx
        Standpoint.Log("  idx " + lastIdx + " -> " + idx + " t=" + ((now - cycleStart) as Int) + "s")
        lastIdx = idx
    EndIf
    Float ddx = targetNpc.GetPositionX() - marker.GetPositionX()
    Float ddy = targetNpc.GetPositionY() - marker.GetPositionY()
    Float d = Math.Sqrt(ddx * ddx + ddy * ddy)
    If settleStart == 0.0 && d < 40.0
        settleStart = now
    EndIf
    If settleStart > 0.0 && now - settleStart >= 3.0
        Float angErr = NormAng180(targetNpc.GetAngleZ() - faceTarget)
        Standpoint.Log("  RESULT " + phase + "." + cycle + ": dist=" + d + " angErr=" + angErr + " t=" + ((now - cycleStart) as Int) + "s")
        If phase == 2
            Utility.Wait(5.0)
            ddx = targetNpc.GetPositionX() - marker.GetPositionX()
            ddy = targetNpc.GetPositionY() - marker.GetPositionY()
            Standpoint.Log("  DRIFT: dist@5s=" + Math.Sqrt(ddx * ddx + ddy * ddy))
        EndIf
        ActorUtil.RemovePackageOverride(targetNpc, parkPkg)
        Debug.SendAnimationEvent(targetNpc, "IdleForceDefaultState")
        targetNpc.EvaluatePackage()
        Utility.Wait(0.5)
        NextCycle()
        Return
    EndIf
    If now - cycleStart > 60.0
        Standpoint.Log("  TIMEOUT " + phase + "." + cycle)
        timeouts += 1
        ActorUtil.RemovePackageOverride(targetNpc, parkPkg)
        targetNpc.EvaluatePackage()
        NextCycle()
        Return
    EndIf
    RegisterForSingleUpdate(0.5)
EndFunction

Function StopTest(String asReason)
    running = False
    UnregisterForUpdate()
    If targetNpc
        ActorUtil.RemovePackageOverride(targetNpc, parkPkg)
        Debug.SendAnimationEvent(targetNpc, "IdleForceDefaultState")
        targetNpc.EvaluatePackage()
    EndIf
    If savedAvoid > 0.0
        Standpoint.SetEngineSettingFloat("fAvoidPlayerDistance", savedAvoid)
    EndIf
    ReferenceAlias markerAlias = GetOwningQuest().GetAlias(1) as ReferenceAlias
    If markerAlias
        markerAlias.Clear()
    EndIf
    If marker
        marker.Disable()
        marker.Delete()
        marker = None
    EndIf
    Standpoint.Log("=== S1 STOP (" + asReason + ") timeouts=" + timeouts)
    targetNpc = None
EndFunction

; ============================= helpers ======================================

Float Function NormAng180(Float afA)
    While afA > 180.0
        afA -= 360.0
    EndWhile
    While afA < -180.0
        afA += 360.0
    EndWhile
    Return afA
EndFunction

Float Function NormAng360(Float afA)
    While afA >= 360.0
        afA -= 360.0
    EndWhile
    While afA < 0.0
        afA += 360.0
    EndWhile
    Return afA
EndFunction
