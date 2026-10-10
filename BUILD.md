# Building Standpoint

Standpoint has three parts: a DLL (C++), seven scripts (Papyrus) and a light plugin (ESP). You only
need to rebuild the part you change. A ready-to-install copy of all three is in `data/`.

## 1. The DLL

### Requirements

| | |
|---|---|
| Visual Studio 2022 or newer | with the C++ desktop workload |
| CMake 3.21+ | the copy bundled with Visual Studio is fine |
| vcpkg | the copy bundled with Visual Studio is fine; set `VCPKG_ROOT` |
| **CommonLibSSE-NG** | a source checkout with VR support (for example CommonLibVR 4.14) — not vendored here |

### Configure and build

```bash
set VCPKG_ROOT=C:\path\to\vcpkg
cmake --preset vr -DCOMMONLIB_PATH="C:/path/to/CommonLibSSE-NG"
cmake --build --preset vr
```

The preset is called `vr` for historical reasons; it builds **one DLL for SE, AE and VR**
(`ENABLE_SKYRIM_SE/AE/VR` are all on). The result is `build/vr/Release/Standpoint.dll`.

To have each build copied straight into your mod folder, add:

```bash
  -DMOD_OUTPUT="C:/path/to/MO2/mods/Standpoint/SKSE/Plugins"
```

Build with the game closed if you do: the running game locks the DLL, and the copy step then fails
at the end of an otherwise green build (`MSB3073`).

### Things that will bite

- `vcpkg.json` lists `directxtk` and `rapidcsv` although Standpoint does not use them: CommonLib's
  own CMake demands them.
- Windows headers define `min` / `max` macros: write `(std::max)(a, b)`.
- Every engine address goes through `RELOCATION_ID(se, ae)`; VR resolves the SE id through the VR
  Address Library. There are no raw offsets in this plugin — keep it that way.
- The version number lives in four places: `project(... VERSION)` in `CMakeLists.txt`,
  `SKSEPluginInfo` and the load line in `src/main.cpp`, and `GetPluginVersion` in
  `src/PapyrusAPI.cpp` (the string other mods compare). The driver script prints its own.
- `src/SlotSolverClock.inc` is included at the end of `SlotSolver.cpp`; it is not a compile unit.

## 2. The scripts

Sources are in `papyrus/`; `papyrus/stubs/` holds compile-only declarations of functions that live
in other mods (PapyrusUtil's `ActorUtil`, a three-function shim for SeverActions, and corrected
headers for a few vanilla classes). **Never ship the stubs.**

With [Caprica](https://github.com/Orvid/Caprica):

```bash
cd papyrus
Caprica.exe --game skyrim ^
  --import "stubs" ^
  --import "C:/path/to/WaterFinder/Scripts/Source" ^
  --import "C:/path/to/Skyrim/Data/Scripts/Source" ^
  --output "../out" --flags "TESV_Papyrus_Flags.flg" ^
  Standpoint.psc StandpointAPI.psc Standpoint_PlayerAlias.psc ^
  Standpoint_ParkEffect.psc Standpoint_SlotEffect.psc ^
  Standpoint_PointedEffect.psc Standpoint_WaterEffect.psc
```

- Run it from inside the source folder with relative file names, and repeat `--import` for each
  folder (a `;`-separated list splits on the drive-letter colon).
- The stubs folder comes first so it shadows the others.
- `Standpoint_WaterEffect.psc` (a test spell) needs Water Finder's sources to compile. If you do not
  have them, leave that one file out.
- **Caprica pitfall:** never pass a *computed* Bool expression as a function argument — put it in a
  named local first. The driver follows this rule everywhere.

A recompiled script only takes effect after a full game restart.

## 3. The plugin

`data/Standpoint.esp` is a light (ESL-flagged) plugin with 15 records: one start-enabled quest whose
player alias carries the driver script, four optional marker aliases, two idle markers (one with the
hands-behind-the-back wait, one with no animation), four travel packages cloned from the vanilla
`TravelAndUseIdle` template, and four test spells.

Edit it with xEdit or the Creation Kit, or round-trip it through
[Spriggit](https://github.com/Mutagen-Modding/Spriggit) if you prefer text. Keep FormIDs between
`0x800` and `0xFFF` (ESL range) and the header version at 1.7.

## 4. Checking a build in game

Start the game and read `SKSE/Standpoint.log`:

```
Standpoint v1.0.0 loaded (...)
driver v1.0.0 ready (engine=TRUE sa=...)
```

`engine=TRUE` proves the four packages resolved. Then run the console test: click an NPC, `stpt`,
click a target, `stpt`, close the console. Every request and result line goes to the log.
