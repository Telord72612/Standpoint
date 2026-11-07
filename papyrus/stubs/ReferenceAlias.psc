Scriptname ReferenceAlias extends Alias Native Hidden
{ COMPILE-ONLY corrected stub. The shared compat-headers/ReferenceAlias.psc extends Form and has no
  functions, which breaks both the "as ReferenceAlias" cast and every member call. This one mirrors
  the real class: extends Alias (report: batch2b seam facts). DO NOT SHIP. }

Function ForceRefTo(ObjectReference akNewRef) native
ObjectReference Function GetReference() native
Function Clear() native

; Declared here so a child's handlers are OVERRIDES (Caprica forbids a non-native script
; from defining new events).
Event OnPlayerLoadGame()
EndEvent
Event OnInit()
EndEvent

; SKSE custom ModEvent handler names must be declared in a NATIVE ancestor or Caprica
; refuses them ("Non-native scripts cannot define new events"). Runtime dispatch is by
; name via RegisterForModEvent, so declaring it here is safe.
Event OnTravelComplete(String asTagStatus, Float afNumArg, Form akSender)
EndEvent

; Standpoint 1.6.0: the stpm test's ear (Standpoint_PlayerAlias.ApiTestListen) - the handlers of Standpoint's own six
; Standpoint_* events, with the signatures StandpointAPI.psc documents (same reason as above).
Event OnStpmArrived(Form akWalker, Float afDist, Float afAngErr)
EndEvent
Event OnStpmFailed(Form akWalker, Float afCode)
EndEvent
Event OnStpmInterrupted(Form akWalker, Float afCode)
EndEvent
Event OnStpmMoveResult(Form akWho, Int aiHandle, String asOutcome, Int aiDetail, Float afDistCm, Float afFacingHour, Float afOrientHour, Form akWhere)
EndEvent
Event OnStpmWalkResult(Form akWho, Int aiHandle, String asOutcome, Int aiDetail, Float afDistLeft, Float afSubmerged)
EndEvent
Event OnStpmDirectWalk(String asEventName, String asWhat, Float afWalkHandle, Form akWho)
EndEvent
