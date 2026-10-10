Scriptname ActiveMagicEffect extends Form Hidden
{ COMPILE-ONLY base-type stub. Extends Form here purely so RegisterForSingleUpdate / OnUpdate
  resolve from the shared Form stub; at runtime the engine supplies the real ActiveMagicEffect.
  DO NOT SHIP. }

Function Dispel() native
Actor Function GetTargetActor() native
Actor Function GetCasterActor() native

; Declared here so a child's handler is an OVERRIDE -- Caprica forbids a non-native script from
; DEFINING a new event, and it doesn't know the ActiveMagicEffect events natively.
Event OnEffectStart(Actor akTarget, Actor akCaster)
EndEvent

Event OnEffectFinish(Actor akTarget, Actor akCaster)
EndEvent
