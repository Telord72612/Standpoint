# Compile-only stubs

These files exist only so that Standpoint's scripts compile. **They are never shipped with the mod**, and
they are not part of Standpoint's public-domain dedication: each one declares functions that belong to
somebody else.

| File | Declares functions of |
|---|---|
| `ActorUtil.psc` | PapyrusUtil (package overrides) |
| `SeverActionsNativeExt.psc` | SeverActions — three signatures only; Standpoint calls one of them, and only when SeverActions is installed |
| `ReferenceAlias.psc`, `ActiveMagicEffect.psc`, `ActorBase.psc`, `EncounterZone.psc`, `ModEvent.psc`, `Package.psc` | the base game and SKSE (headers with the members the compiler needs) |

At run time the real scripts and DLLs of those mods provide the functions.
