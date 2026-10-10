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
