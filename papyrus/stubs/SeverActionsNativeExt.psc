Scriptname SeverActionsNativeExt Native Hidden
{ COMPILE-ONLY minimal shim (PO3-shim pattern): declares ONLY the natives Standpoint calls.
  Runtime is provided by SeverActionsNative.dll. Full signatures verbatim from
  SeverActions 3.9.2 SeverActionsNativeExt.psc:772-778. DO NOT SHIP. }

Int Function Travel_BeginXY(Actor akActor, Float destX, Float destY, Float destZ, Keyword akKeyword, Float arrivalThreshold, String callbackTag, Int options, Int maxDurationSeconds) Global Native
Int Function Travel_CancelByActor(Actor akActor) Global Native
String Function Travel_GetStateName(Int stateCode) Global Native
