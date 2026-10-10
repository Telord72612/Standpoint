Scriptname Standpoint_PointedEffect extends ActiveMagicEffect
{ Self-cast selector for NON-ACTOR destinations. An aimed magic effect only ever
  delivers to actors, so furniture cannot be picked by projectile - this spell
  reads the VR hand-activation ray instead (GetCurrentCrosshairRef = the ref
  showing rollover text). Point at the bed, cast, done. }

Event OnEffectStart(Actor akTarget, Actor akCaster)
    ObjectReference dest = Game.GetCurrentCrosshairRef()
    Quest q = Game.GetFormFromFile(0x800, "Standpoint.esp") as Quest
    If !q
        Return
    EndIf
    Standpoint_PlayerAlias driver = q.GetAlias(0) as Standpoint_PlayerAlias
    If !driver
        Return
    EndIf
    If !dest
        Standpoint.Log("PointedEffect: no crosshair ref (point at the object until rollover text shows)")
        Debug.Notification("Standpoint: nothing pointed at")
        Return
    EndIf
    Standpoint.Log("PointedEffect: dest=" + dest)
    driver.HandleSlotCast(dest)
EndEvent
