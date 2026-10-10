Scriptname ActorUtil Hidden
{ COMPILE-ONLY soft-dep stub for PapyrusUtil's ActorUtil. Signatures verified against the real
  PapyrusUtil VR source (ActorUtil.psc:13,16). AddPackageOverride is genuinely VOID there.
  DO NOT SHIP. }

Function AddPackageOverride(Actor targetActor, Package targetPackage, int priority = 30, int flags = 0) global native
bool Function RemovePackageOverride(Actor targetActor, Package targetPackage) global native
