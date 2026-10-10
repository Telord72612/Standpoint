Scriptname Standpoint_WaterEffect extends ActiveMagicEffect
{ Self-cast: Water Finder locates the nearest water for the WALKER, Standpoint walks
  her to the SHORE point facing the water. The closing integration test - this exact
  "NPC cannot reach water" problem is why both frameworks exist. }

Event OnEffectStart(Actor akTarget, Actor akCaster)
    Quest q = Game.GetFormFromFile(0x800, "Standpoint.esp") as Quest
    If !q
        Return
    EndIf
    Standpoint_PlayerAlias driver = q.GetAlias(0) as Standpoint_PlayerAlias
    If driver
        driver.SendWalkerToWater()
    EndIf
EndEvent
