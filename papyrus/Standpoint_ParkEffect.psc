Scriptname Standpoint_ParkEffect extends ActiveMagicEffect
{ Thin trigger on Standpoint_TestEffect (aimed). A fire-and-forget AME may already be UNBOUND
  in OnEffectStart (WFTest lesson) — global natives still work, member state does not.
  So this does nothing but hand the target to the permanently-bound driver. }

Event OnEffectStart(Actor akTarget, Actor akCaster)
    If !akTarget || akTarget == Game.GetPlayer()
        Return
    EndIf
    Quest q = Game.GetFormFromFile(0x800, "Standpoint.esp") as Quest
    If !q
        Standpoint.Log("ParkEffect: quest not found")
        Return
    EndIf
    Standpoint_PlayerAlias driver = q.GetAlias(0) as Standpoint_PlayerAlias
    If !driver
        Standpoint.Log("ParkEffect: driver alias cast failed")
        Return
    EndIf
    driver.StartTest(akTarget)
EndEvent
