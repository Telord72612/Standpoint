Scriptname Standpoint_SlotEffect extends ActiveMagicEffect
{ Thin trigger for the S2 slot-test spell. All logic lives on the bound driver. }

Event OnEffectStart(Actor akTarget, Actor akCaster)
    Quest q = Game.GetFormFromFile(0x800, "Standpoint.esp") as Quest
    If !q
        Return
    EndIf
    Standpoint_PlayerAlias driver = q.GetAlias(0) as Standpoint_PlayerAlias
    If driver
        driver.HandleSlotCast(akTarget as ObjectReference)
    EndIf
EndEvent
