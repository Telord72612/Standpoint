Scriptname ModEvent Hidden
{ COMPILE-ONLY stub for SKSE's ModEvent. Return types verified against the real ModEvent.psc
  (Create/Send return values; Push* and Release are genuinely VOID -- declaring a Push as bool
  would route a discarded return into a None temp and spam "::NoneVar"). DO NOT SHIP. }

int Function Create(string eventName) global native
bool Function Send(int handle) global native
Function Release(int handle) global native
Function PushBool(int handle, bool value) global native
Function PushFloat(int handle, float value) global native
Function PushForm(int handle, Form value) global native
