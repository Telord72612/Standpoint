# Standpoint — Integration Guide (V2.0 = 2.0.0)

**"Walk this NPC to a precise stand-slot at a target and have them arrive facing the
right way, then hold there."**

This document is the complete contract for integrating Standpoint — written for both
human modders and AI agents. The numbers in it are field-measured in Skyrim VR on a
~2,000-mod load order unless they are marked *design value* or *not yet run*. If you read
nothing else, read **Standpoint V2.0 in one page**, **The Golden Rules** and the
**Patterns** section.

**V2.0 (2.0.0) is the code of 1.6.1 with a new version number.** Nothing in this document
changed between 1.6.1 and 2.0.0. The parts marked *(historical)* describe older versions and
are kept so that a mod written against them can be understood.

**Writing an SKSE plugin?** Since v0.8.0 you do not need Papyrus at all: hand Standpoint
one 10-slot request line (who, where, how close, which side, facing which way, pose, how
long) and get one measured result line back. Jump to **C++ API → The move request**.

**Need her somewhere the navmesh does not reach** — into a lake up to her knees, onto a bath
platform, over rocks? Since v1.4.0 (Skyrim VR) `WalkTo` plans its own route and walks her
there with her own legs. Jump to **Walking off the navmesh**.

**Want her IN the water — the nearest water she can really reach?** Since v1.5.0 one call does it:
`WalkToWater` asks Water Finder for the water, finds a wading spot she can walk to, walks her in
and turns her to face the water. Jump to **Into the water: WalkToWater**.

**Farther away?** A walk reaches **200 m** over Standpoint's own path (since 1.6.0 — the engine's
far travel is gone), and her pace follows the route that is LEFT: she runs while more than 15 m are
left, jogs down to 3 m and walks the last 3 m. Jump to **Speed bands and the long walk (V2.0)**.
*(historical: 1.5.1 ran her with the engine's travel beyond 30 m — see "Speed by distance and the
far phase (v1.5.1)".)*

---

## Standpoint V2.0 in one page

### Two kinds of destination, two ways of walking

| You call | On Skyrim VR (default) | On SE / AE (default), or VR with `[Session] Steered = 0` |
|---|---|---|
| `GoTo` (a side of a target), `RequestMove` (Papyrus or C++: a clock hour + distance around a target) | **a steered session**: Standpoint solves the spot itself with physics tests, plans ONE route and walks her with its own steering; the driver pins her on arrival | the engine session (travel package + idle marker), exactly as 1.5.1 |
| `GoToPoint` (coordinates + heading) | **a steered session** (same switch) | the engine session |
| `WalkTo`, `WalkToWater`, C++ `RequestWalk` / `RequestWalkToWater` | the steered walk (always) | SE / AE: `B` 19, unless `[DirectWalk] SEAE = 1` and the self-test says READY |
| `RequestMove` with `where = who` (hold your place) | the engine hold (no walk) | the same |

A session stays an ENGINE session on VR too when the target is in another interior / worldspace,
or an actor target is not 3D-loaded (the log says `STEER check: <why> - the ENGINE session walks
this one`).

**"Steered"** = planner direct control: Standpoint steers her own movement controller every frame
(direction, speed, facing); the engine plays her own walk / jog / run. Nothing slides or teleports
her. `IsDirectWalking` (C++) is True and the mod event `Standpoint_DirectWalk` fires during the walk.

### A steered session keeps the whole session contract

Same calls, same handles, `IsBusy` True, `GetArrivalState` 1 → 2, the events `Standpoint_Arrived` /
`_Failed` / `_Interrupted`, `Standpoint_MoveResult` (one per handle), hold `-1` / `0` / `N`, the poses,
`SetFaceTarget`, `Release`, the pair rule for close contact, the leash (`L` 5), the danger rule (`D` 4),
the player bump (`P` 7). What you can see that is new:

- **The spot** is tested by physics: a floor she can stand on, room for her body, not on a stair, not
  inside a standing NPC, the target's own floor level, water no deeper than her thighs (0.45 × her
  height). The asked side / hour first, then the solver's own order. A chair's seat beside its entry no
  longer counts as "a brink" (1.6.1).
- **One route**, then she walks it without stop-and-go: round the target at about her body's width, the
  last stretch straight in on the asked side; from body contact (within ~1.2 m of an NPC target) she
  side-steps round it facing it (the orbit).
- **A moving target is followed:** it moves 32 u, or an NPC target turns 30° for an hour-relative spot →
  a re-solve (her hour first) → she re-aims while walking (no stop) when the spot moved 16 u or more.
  The player as target: his position only.
- **The answer comes sooner** after she stops (measured 1.6.1: a 150 cm RequestMove answered 3.6–6.4 s
  after it was sent, walk and pin included).
- **Waits instead of ends:** with `aiInterrupts 0` (or the player as target) a HIGGS grab makes her WAIT
  (`Interrupted` 3 is sent, report only) and she walks on when the hand lets go; with interrupts 0–1
  combat makes her wait; a ragdoll / knock-down / kill move is waited out. The time limit keeps running.
- **On arrival** she is PINNED where she stopped: the idle marker is moved to her feet with the final
  heading, the slot package goes on, then the pose and the hold run as before. Exceptions: hold `0` +
  pose `0` = she is let go at the point; a `GoToPoint`, or a `RequestMove` with pose 0, ending OFF the
  navmesh (water, a rock) = held by the walk's own stand (driven at speed 0), no marker. (A `GoTo` always
  pins, with the standard-wait marker, as its engine version did.)
- **A sitting walker** (chair, bench, bed, getting up) is given ONE engine leg: the game's walk stands
  her up and walks her (`ENGINE FALLBACK … she sits`).

### The answers of a session (V2.0)

| Result | `Standpoint_MoveResult` / C++ | The walker's event | When |
|---|---|---|---|
| arrived on the spot | `C` 0 | `Arrived` | within **8 u** (11 cm) of the solved spot |
| arrived short | `C` 9 | `Arrived` (afDist = the real distance) | not close: stopped < 100 u flat and ≤ 100 u up / down from the spot · close to an NPC: within 12 u, or within 35 u on the asked side (40°) · an engine leg sent to the nearest navmesh point (below): always `C` 9, distance measured to YOUR point |
| no standable spot | `O` 1 | `Failed` 1 | nothing passed the physics tests (all furniture entries TAKEN = `O` 1 at once, as 1.5.1) · a point off the navmesh with no route and no navmesh within 128 u |
| not there in time | `O` 2 | `Failed` 2 | the walk's limit (below) |
| stopped too far | `O` 15 | `Failed` 6 | not close: 100 u or more from the spot when she stopped (also `O` 21 "would swim" → `Failed` 6) |
| no way | `O` 17 | `Failed` 7 | close contact with no route (at once — never an engine leg) · close to an NPC and not near the spot · a relative spot off the navmesh with no route · **an engine leg that got stuck** (below) |
| the player stopped her | `P` 7 / `P` 8 | `Interrupted` 2 / 3 | bump / body-block (not while she is still planning or waiting) · HIGGS grab with interrupts 1–2 |
| released / replaced / ended | `X` 3 / 6 / 14 | `Failed` 3 while she was walking (no event while she held) | `Release`, a new request, a walk taking her (see the hand-off table), her gate (mounted, bleedout, an animation lock …) |
| refused | `B` 11 / `B` 22 | `Failed` 1 when the DLL refused the steered session at its start (GoTo / GoToPoint already returned a handle) · `Failed` 3 when her walk was refused later (`B` 22: another mod drives her) · none for a RequestMove refused because all 4 slots are busy | |
| danger / leash | `D` 4 / `L` 5 | `Failed` 4 / 5 | as before |

`GoTo` / `GoToPoint` answer only through the walker's events; the letter column is what a `RequestMove`
answers (Papyrus `Standpoint_MoveResult`, C++ callback) and what `Standpoint.log` names for every session.
A RequestMove gets ONE answer per handle — the first; later endings of the same session are not sent.

**The handle comes first now (steered).** An ENGINE `GoTo` that finds no slot returns **0** and sends
`Failed` 1 at once. A STEERED `GoTo` / `GoToPoint` returns a handle **> 0** first (the spot is solved in
the DLL, over a few frames) and answers `Failed` 1 (`O` 1) a moment later. Do not treat a handle > 0 as
"a spot exists" — wait for the event. (`GoTo` still returns 0, with no event, when all 4 slots are busy.)

### No route: the ONE engine leg (the user's rule, 1.6.0 / 1.6.1)

When the steered planner finds no route for a session (`O` 1 / 18 / 26 from its walk), Standpoint first
tries ONE more spot on another side (2 clock hours away or more) for a relative target. Then:

| The session | What happens |
|---|---|
| close contact (`close = 1`) | `O` 17 / `Failed` 7 **at once** — never an engine leg |
| the spot (relative or point) is on the navmesh | **ONE engine leg** to the spot (`ENGINE FALLBACK`), with its own fresh 90 s |
| the point is outside the loaded area (the walk said `O` 26) | ONE engine leg (the game's walk can travel there) |
| a `GoToPoint` point OFF the navmesh (a shore point on a rock / in the water) | ONE engine leg to its **nearest navmesh point within 128 u**; the arrival is `C` 9 / `Arrived` with the distance to YOUR point (she ends at the shore edge — check her distance to your point, not the letter) |
| … and no navmesh within 128 u of that point | `O` 1 / `Failed` 1 at once |
| a relative spot off the navmesh | `O` 17 / `Failed` 7 |

**An engine leg ends when it gets stuck (1.6.1):** 80 u or more from its point, not close, and the
engine reports MOVEMENT BLOCKED on two polls with her 1 s still, or she has not moved 4 u for 6 s (after
she walked, or after both LEG START kicks) → `ENGINE LEG ENDED` → `O` 17 / `Failed` 7 at once (1.6.0
waited out the 90 s). Nearer than 80 u the engine's stall rule makes it an arrival. *(Watch item, not
yet seen in VR: the player standing in her way for ~1 s on a far leg can end it this way.)*

### Who takes her from whom (one owner per NPC)

| She has … | … and this comes | Her old owner hears | Notes |
|---|---|---|---|
| a session (walking) | a new `GoTo` / `GoToPoint` / `RequestMove` | `X` 6 on its MoveResult; **no** `Failed` event | a retask; a steered session of the same kind may re-aim the running walk |
| a session (walking) | `WalkTo` / `WalkToWater` / `RequestWalk*` | **`Failed` 3 + `X` 3** (released — not `X` 6) | the walk releases the session first (VR / where the walk can run) |
| a session (holding, after `C`) | a walk / a new session | nothing (its MoveResult already answered `C`; no event) | the pin comes off, the new owner takes her |
| a walk (walking) | any session start | the walk answers `X` 6 | |
| a walk (standing after `C`) | any session start / a new walk | no second answer (`HOLD END replaced`) | |
| a walk | a new walk | `X` 6 | she stays held until the new one takes her |
| either | `Release` | session: `Failed` 3 (if walking) + `X` 3 · walk: `X` 3 | `Release` ends BOTH kinds — release only what you still own (Pitfalls) |
| a session | a C++ release line (`where = 0`) | `X` 3, and the line itself `X` 3 (`X` 13 if she had nothing) | |

### Time limits (V2.0)

| What | Limit | Answer |
|---|---|---|
| a steered session / a walk | **90 s** (pause excluded); a start more than 2,100 u (30 m) away: **90 s + 1 s per 150 u** of that distance (183 s for 200 m) | `O` 2 / `Failed` 2 |
| the driver's backstop for a steered session | 95 s (+ the same extra) — only if the DLL never answered | `O` 2 / `Failed` 2 |
| an engine leg after no route / a sitting walker | a fresh 90 s; ends early when stuck (above) | `O` 17 / `Failed` 7 |
| an engine session (SE / AE, `Steered = 0`) | 90 s; LEG START (2 s re-evaluate, 4 s re-send), DETOUR (a blocked leg 80–1,500 u out gets a planned way round), the stall rule (within 80 u: 3 s still after moving, 8 s if never moved → the arrival handling) | `O` 2 / `C` 9 / `Failed` 6 |
| a walk that gets stuck | a new route after every stall / door / push in a new place; stuck twice within 16 u of one re-plan start | walk `O` 18 (a session then goes to "no route" above) |

### Settings (`Data/SKSE/Plugins/Standpoint.ini`, read once at start-up)

| Key | Default | What |
|---|---|---|
| `[DirectWalk] SEAE` | 0 | SE / AE only: 1 = install the walk hook when the self-test says READY (VR ignores it) |
| `[Session] Steered` | no line = VR 1, SE / AE 0 | 0 = the engine sessions exactly as 1.5.1 (the VR rollback switch, no rebuild); 1 on SE / AE needs `SEAE = 1` too |
| `[Cleanup] SweepMarkers` | 0 | 1 = at the next load, the orphan idle markers older Standpoints left in the save are disabled + deleted (the ones in use are never touched); 0 = counted only (`MARKERS:` log line) |

### Version checks

- **Papyrus:** `StandpointAPI.GetVersion()` → `""` (absent) or `"2.0.0"`. Gate each call on the version
  that introduced it: `SetFaceTarget` 0.7.3, `GetArrivalState` 0.7.4, `RequestMove` 1.2.0, `WalkTo` 1.4.0,
  `WalkToWater` 1.5.0; the steered behaviour 1.6.0. The comparison is a STRING comparison: it works for
  every version so far (`"2.0.0" >= "1.6.0"`), but would not for a two-digit major.
- **C++:** `GetBuildNumber()` = **4** since 1.5.0 — 1.6.x and 2.0.0 changed no line and no slot, so the
  build number cannot tell 1.5.x from 2.0.0. `GetApiFunction(4) != null` = 1.5.0 or newer.
- **The log:** `Standpoint v2.0.0 loaded`, `driver v2.0.0 ready (engine=TRUE sa=… steered=TRUE|FALSE)`,
  `steered sessions …: ON | OFF`.

---

## What Standpoint guarantees (measured)

| Property | Measured value |
|---|---|
| **V2.0 steered sessions** (1.6.1 VR run, 2026-11-06, 26 arrivals) | position: median **4.5 u** from the solved spot, 22 of 26 within 8 u (max 20.9 u: GoTo BACK of an NPC); facing: median **0.0°**, max 0.8° (pinned) |
| **V2.0 steered walks** (`WalkTo` etc., the same run, 64 × `C` 0) | median **3.5 u**, 62 of 64 within 6 u; facing median 0.0°, max 2.3° |
| **V2.0 RequestMove around an NPC** (the same run) | asked 150 cm at 12 / 3 / 6 / 9 → **145–146 cm**, `C` 0; close 45 cm → **43.9 cm**, `C` 0 |
| Arrival facing error (engine sessions, 0.x) | median **0.12°**, max ~1.5° (engine-driven: `fIdleMarkerAngleTolerance=1`) |
| Arrival position error (engine sessions, 0.x) | typically **centimetres** (0.06–25 u observed) |
| Hold stability | **exactly zero drift** while the session holds (engine pin; the steered session pins the same way) |
| Stuck recovery | **None since v0.9.2.** Nothing moves a walker by force. V2.0: a stuck steered walk gets a new route from where she stands (never twice from one spot); a session with no route gets ONE engine leg, which ends within seconds when it is stuck (`Failed` 7); the time limit ends the rest (`Failed` 2). A walker found 100 u or more from her point is released (`Failed` 6) instead of being reported as arrived. *(historical, engine sessions before 1.6.0: "a walker who gets stuck on the way fails after 90 s (code 2)" — still true of the engine sessions, with LEG START / DETOUR / the 80 u stall rule before it.)* |
| Concurrent walkers | **4** sessions; up to **8** walks open at once (planning, walking or standing — the steered sessions' own walks included; more = `B` 11) |
| Time limit | 90 s (+ 1 s per 150 u for a start beyond 30 m) → `Standpoint_Failed` code 2 — see **Time limits (V2.0)** |
| C++ move request, open floor around an NPC (v0.8.1, engine) | asked 150 cm → **148–153 cm**; asked clock hour → **within 0.1 hour**; asked orientation → **within 0.6°** (5 walks, one VR run) |

Slots come from **real furniture markers** (the NIF's entry points AND the record's
per-marker enable bits MINUS its disabled points — the full legality gate) or, for
anything without markers (tables, chests, loose objects, actors), from the target's
bounds with side selection in the target's own facing frame. Every slot is
**navmesh-snapped** before anyone walks.

## Requirements

- SKSEVR + VR Address Library (SE/AE builds: SKSE64 + Address Library) — the DLL is
  multi-runtime.
- PapyrusUtil (package overrides).
- **Optional:** Water Finder (the water pattern below; `WalkToWater` needs Water Finder 1.6.0 or
  newer, and 1.6.2 or newer to try other water when the nearest has no wading spot she can reach —
  V2.0 is tested with **1.6.2**). HIGGS (VR): the hand-grab answers and the pair rule's collision hook.
  SeverActions is no longer used (v0.9.2): its stuck recovery teleports — see Pitfalls.

Soft-dep check — this is the whole contract:

```papyrus
If StandpointAPI.GetVersion() != ""   ; "" = Standpoint absent
```

---

## Edition support (SE / AE / VR)

**One DLL serves all three editions** (CommonLibSSE-NG runtime multi-targeting; all three
SKSE entry points verified present in the shipped binary — `SKSEPlugin_Version` for SE/AE,
`Query`/`Load` for VR).

- **Addresses:** every engine call is a two-id `RELOCATION_ID(se, ae)` — SE and AE resolve
  DIRECTLY through Address Library for SKSE Plugins; VR translates the SE id through the
  VR Address Library. No hand-RE'd raw offsets are used anywhere in this plugin.
- **AE layout shifts** (post-1.6.629) are handled by CommonLib NG's versioned accessors at
  the one divergent spot this plugin touches (`Actor` runtime data); everything else it
  reads has an unconditional layout.
- **ESL + scripts are edition-agnostic:** header 1.7, Skyrim.esm-only master, and every
  Papyrus native used exists in all editions. (Since 1.2.0 the developer test spells are no
  longer given to the player; the records stay in the ESP.)
- **Close contact without HIGGS (1.2.0):** the pair rule (two NPC bodies ignore each other)
  runs through HIGGS's collision hook in VR, and through Standpoint's own hook on the game's
  collision filter (`bhkCollisionFilter`, ids for SE / AE / VR) when HIGGS is absent. An empty
  file `SKSE/Plugins/Standpoint_OwnPairFilter.test` forces the own hook for a VR test.
- **Per-edition requirements:** SE/AE = SKSE64 + Address Library for SKSE Plugins;
  VR = SKSEVR + VR Address Library. Plus PapyrusUtil for the matching edition.
- **The walk off the navmesh (`WalkTo`, `WalkToWater`) on SE / AE (1.5.0): OFF by default, with a
  switch.** Its driver takes the NPC's own movement controller once per frame at one engine call.
  On VR that call is known and proven. On SE / AE Standpoint CHECKS at start-up, by reading the
  game's code, that the call sits where it is expected (one or two fixed positions per edition,
  just before the engine's "apply movement" call, which many SE / AE mods hook) — a read-only
  **self-test** that changes nothing and writes into `SKSE/Standpoint.log` either
  `SE/AE direct walk: READY` or `SE/AE direct walk: NOT READY because …`, followed by the bytes it
  read. The switch is the file `SKSE/Plugins/Standpoint.ini` (ships with the mod):
  `[DirectWalk] SEAE = 0` (default) = the self-test only — `CanWalkOffNavmesh()` returns False,
  `WalkTo` / `WalkToWater` answer `B` 19 at once, everything else works as in 1.3.0;
  `SEAE = 1` = the walk hook is installed (only when the self-test says READY) and the walks run
  as on VR, with two differences: she stops with her own deceleration (a softer stop — VR's firm
  stop has VR-only addresses), and she turns at the point instead of turning while she walks the
  last steps. Skyrim VR ignores the switch. **Not yet run on SE or AE** — see "For SE / AE
  testers" below.
- **The steered sessions on SE / AE (1.6.0): OFF by default.** `[Session] Steered` defaults to 0 on SE /
  AE: `GoTo` / `GoToPoint` / `RequestMove` are the engine sessions of 1.5.1. With `Steered = 1` AND the
  walk on (`SEAE = 1`, self-test READY) they are steered as on VR. Never run on SE / AE.

**For SE / AE testers (what to do and what to send):**
1. Install, start the game, load a save, wait until you can move, quit. Send
   `Documents/My Games/Skyrim Special Edition/SKSE/Standpoint.log` (GOG: the `Skyrim Special
   Edition GOG` folder) and `skse64.log` from the same folder, and say whether Leash Framework,
   Fluid Motion or True Directional Movement is installed.
2. Only when told "READY, go": set `SEAE = 1` in `SKSE/Plugins/Standpoint.ini`, start the game,
   load a save outdoors or in a town, open the console, click an NPC, type `stpf`, close the
   console. Walk, stop and turn a few times — she should walk to about 3 m in front of you with
   real steps and turn to face you. Type `stpf` again to stop. Quit at once (the log is replaced
   at the next start) and send the log plus one sentence: real steps / slides / walks in place /
   stuck / spins — and any crash log.
3. To go back: `SEAE = 0` (or delete the line). The same DLL.

**The honest caveat: every measurement in this document was taken in Skyrim VR.** SE/AE
behaviour is expected identical — the arrival mechanism is core Bethesda AI, and VR was
the *harder* target (PLANCK/VRIK interactions, 90 Hz) — but "expected" is not "measured."
First SE/AE deployment should sanity-check the launch log (version line + `engine=TRUE`)
and one park-and-face before building features on it.

---

## The Golden Rules

1. **Prefer `GoTo` (a real target ref) over `GoToPoint` (raw coordinates).**
   `GoTo` runs the full solver: marker legality, occupancy, navmesh snap, side
   selection. `GoToPoint` trusts your coordinates RAW — no snap, no validation. If you
   feed it an off-navmesh point, the engine will path the NPC toward the nearest
   reachable thing instead, which can be *through a door and outside*. If you must use
   `GoToPoint`, you own the validity of the point.
   *V2.0 (steered `GoToPoint`, VR):* the planner checks every leg with rays, a point that is no
   place to stand moves toward her (up to ~320 u), a point off the navmesh is walked to directly when
   a route exists, and with no route the ONE engine leg goes to its nearest navmesh point within
   128 u (see **No route**). On the ENGINE path (SE / AE, `Steered = 0`) the point is still raw.

2. **You own the session's end.** A walker HOLDS their slot (engine-pinned, zero
   drift) until you `Release()` them, retask them with a new `GoTo`, or your
   `afHoldSeconds` expires. A held NPC does nothing else — that is the feature. If your
   NPC "stands there frozen," you forgot to release or set a hold policy.

3. **Register events on YOUR OWN form.** SKSE ModEvent registration is per
   (form, event) — registering on a shared form silently steals or loses callbacks.

4. **One walker = one owner.** Re-calling `GoTo` for a busy walker retasks them
   (their slot is reused). 4 concurrent walkers max; the 5th `GoTo` returns 0. A walk and a
   session never own her at the same time — see **Who takes her from whom**.

5. **Compose, don't extend.** Standpoint moves actors to slots. It does not find
   water, choose targets, or decide behaviour — that orchestration is YOUR mod's job
   (see the Water pattern for the canonical composition).

---

## Papyrus API (`StandpointAPI`, all Global — no properties, no ESP dependency)

```papyrus
String  GetVersion()
    ; "" when Standpoint is absent. Call once after load; gate everything on it.

Float[] Solve(ObjectReference akTarget, Int aiRelation = 0, Actor akAgent = None, Float afStandoff = 0.0)
    ; PURE — never moves anyone. [] on failure, else:
    ; [x, y, z, headingDEGREES, markerIndex (-1 = geometric), quality 0..1]
    ; Use when you want the slot but will drive movement yourself.

Int     GoTo(Actor akWalker, ObjectReference akTarget, Int aiRelation = 0,
             Float afStandoff = 0.0, Float afHoldSeconds = -1.0, Int aiPose = 0,
             Int aiInterrupts = 1, Int aiDanger = 2, Float afLeash = 3500.0)
    ; Solve + walk + arrive-facing + hold. Session handle (>0) or 0.

Int     GoToPoint(Actor akWalker, Float afX, Float afY, Float afZ,
                  Float afHeadingDeg, Float afHoldSeconds = -1.0,
                  Int aiInterrupts = 1, Int aiDanger = 2, Float afLeash = 3500.0)
    ; Point + facing. Engine path: NO VALIDATION — see Golden Rule 1. Steered (VR): planned + checked.

Bool    Release(Actor akWalker)   ; end session: package off, pose reset, slot freed
                                  ; (1.4.0: also ends her WalkTo walk / stand - X 3)
Bool    IsBusy(Actor akWalker)    ; a session walking or holding? (False during a WalkTo / WalkToWater)
Int     GetArrivalState(Actor akWalker)              ; 0.7.4: 0 none, 1 walking, 2 arrived + holding
Bool    SetFaceTarget(Actor akWalker, ObjectReference akFace)   ; 0.7.3
Int     CountThreatsOnRoute(Actor akWalker, ObjectReference akTarget, Float afCorridor = 600.0)

; the move request from Papyrus (1.2.0) - see "The move request from Papyrus"
Int     RequestMove(Actor akWho, ObjectReference akWhere, Float afDistCm, Float afFacingHour,
                    Float afOrientHour, Int aiPose, Float afHoldSeconds, Int aiClose)
String  GetMoveOutcome(Int aiHandle)     ; "" unknown, "-" open, else the letter
Float[] GetMoveResult(Int aiHandle)      ; [distanceCm, facingHour, orientationHour, detail]

; the walks (1.4.0 / 1.5.0) - see "Walking off the navmesh" and "Into the water"
Int     WalkTo(Actor akWho, Float afX, Float afY, Float afZ, Float afFacingDeg = -1.0,
               Float afHoldSeconds = -1.0, Float afDepthFrac = 0.45, Float afStopSubmerged = 0.0)
Int     WalkToWater(Actor akWho, Float afMaxMetres = 20.0, Float afDepthMinFrac = 0.15,
                    Float afDepthMaxFrac = 0.45, Float afHoldSeconds = -1.0)
String  GetWalkOutcome(Int aiHandle)
Float[] GetWalkResult(Int aiHandle)      ; [distLeft u, submergedLevel, detail, x, y, z]
Bool    CanWalkOffNavmesh()
Bool    CanWalkToWater()
```

There is no Papyrus call that tells "she is on a walk" (`IsBusy` sees sessions only); use the
`Standpoint_DirectWalk` mod event (`start` / `stop|<why>`) or your own handle's `GetWalkOutcome`
("-" = still open). `ApiWake`, `ApiMoveDone`, `ApiWalkDone`, `ApiWalkFreeSession`, `ApiTestListen`,
`_ReleaseUnlessFar`, `ApiWalkFarStart`, `ApiWalkFarEnd` are Standpoint's own plumbing — never call them
(the two far ones are dormant since 1.6.0).

**`aiRelation`** — in the TARGET's own facing frame (an actor's facing, a chair's
forward, a table's local +Y): `0` nearest · `1` front · `2` right · `3` back · `4` left.

**`afStandoff`** — distance from the anchor, game units (~70 u = 1 m). `<= 0` picks the
per-target default (85 u ring for actors, marker/bounds-derived otherwise).

**`afHoldSeconds`** — post-arrival policy: `-1` hold until Release/retask (default) ·
`0` release immediately after the Arrived event · `N` hold N seconds, then auto-release.

**`aiPose`** — interaction pose on arrival, aimed at the target:
`0` none · `1` auto by target height (< 40 u above her feet → kneel, else bend) ·
`2` kneel to floor (`IdlePickup_Ground`, 2.8 s, zero root translation — arrival
precision survives it) · `3` standing bend (`IdlePickupFromTableStart`).

### Events

```papyrus
RegisterForModEvent("Standpoint_Arrived", "OnStandpointArrived")
RegisterForModEvent("Standpoint_Failed",  "OnStandpointFailed")

Event OnStandpointArrived(Form akWalker, Float afDist, Float afAngErr)
    ; fires ONCE per session, at the slot, with the measured errors
    ; v0.7.2: a walker who stopped SHORT (more than 8 u from the point) and could not get
    ; closer is turned where she stands; afDist is then that distance from the ASKED point
EndEvent

Event OnStandpointFailed(Form akWalker, Float afCode)
    ; 1 = no valid slot (V2.0 steered: may come AFTER GoTo / GoToPoint returned a handle)
    ; 2 = not there in time (90 s; + 1 s per 150 u for a start beyond 30 m)
    ; 3 = ended before arrival: Release, a walk that took her, or (V2.0 steered) any other ending
    ;     without an arrival - X / B on the MoveResult (her gate, another mod driving her, ...)
    ; 4 = danger on the route (dropped)   5 = leash (too far from the player)
    ; 6 = (v0.9.2) she stopped moving 100 u or more from the point: not an arrival, released
    ;     (V2.0 steered: also "would swim", O 21)
    ; 7 = NO WAY: she could not get to the asked point, released. (v0.9.9: a close move request to an
    ;     NPC; V2.0: also a close request with no route, a relative spot off the navmesh with no route,
    ;     and the ONE engine leg that got stuck - "ENGINE LEG ENDED")
EndEvent
```

### Face target (v0.7.3)

```papyrus
Bool Function SetFaceTarget(Actor akWalker, ObjectReference akFace) Global
```
Whom the walker turns to if she stops SHORT of her point and cannot get closer (body
contact with the NPC she walked to, a table in the way). Without one she is turned to the
heading you asked for. `GoTo` sets it by itself for an ACTOR target; after `GoToPoint`,
call it yourself, right after the call - every new session (a retask too) clears it.
`None` clears it; `False` = the walker has no session. Older Standpoint has no such
function: check `GetVersion()` >= "0.7.3" first. Measured reason: stuck 22 u to the side in
body contact, the point's heading missed the target by 27°.

### Arrival state (v0.7.4)

```papyrus
Int Function GetArrivalState(Actor akWalker) Global
```
`0` = no session · `1` = walking, or the arrival handling is still in progress · `2` =
arrived and holding. With hold `0` the session ends at arrival, so the state goes 1 -> 0:
use the `Standpoint_Arrived` event or your own check there. New in 0.7.4 - older Standpoint
has no such function: check `GetVersion()` >= "0.7.4" before calling. Why: a consumer or a
test can wait for Standpoint's own arrival instead of guessing it.

### Danger + leash (v0.7.0 -- the DROP contract)

**Standpoint sells ACCURACY, not guaranteed arrival.** When anything hostile is
upon the walker or standing in the route, the session is DROPPED -- no pausing,
no waiting for a corridor to clear, no auto-resume. Destinations are short-range
by design, so the CONSUMER simply asks again later if it still wants it. That is
a cleaner contract than a walker frozen mid-errand, and it is why there is no
pause state in the driver at all.

**`aiDanger`** (8th GoTo argument). A `Standpoint_Interrupted` code 4 fires
either way:
`0` ignore danger | `1` report only, keep walking into it |
`2` **(default) DROP**: package removed, walker reverts to her own AI,
`Standpoint_Failed` code 4.

A threat qualifies when it is within **~320 u of the walker in ANY direction**
("upon her"), or inside the **600 u-wide corridor** ahead along the route
(4096 u lookahead). Hostiles well off the route never drop a session.
Detection covers LOADED actors only -- the same horizon the engine itself
fights at. `StandpointAPI.CountThreatsOnRoute(walker, target)` gives the same
verdict WITHOUT starting a session.

*Field-measured 2026-08-18:* `IsHostileToActor` fires against both the walker
and the player, and keeps firing when `inCombat` is 0 -- so detection does NOT
require an active fight. Wolves report AIDATA aggression **1 (Aggressive)**, not
2, so the predictive `aggro` signal is thresholded at >= 1.

**`afLeash`** (9th argument, default 3500 u ~= 50 m). If an errand carries the
walker further than this from the PLAYER, the session cancels: `Failed` code 5
while walking, a silent release while holding (she already Arrived). `0.0`
disables; ignored when the player is the destination. Also watched during
infinite holds, so a parked NPC is released when the player moves on.

### Interrupts (v0.5.0)

Travel is no longer fire-and-forget — sessions RECOGNISE interference and report it.
Everything is observation-only: nothing suppresses or alters engine behaviour
(teammates' bump reactions, FollowerBumpGuard's handling, combat AI all run untouched).

**What is recognised:**
- **Combat** — the engine already suspends the package and the walker fights; by
  default travel resumes after. Standpoint reports it (once per session).
- **Player bump-capsule contact** — the engine's bump reaction (run-once package
  type 32; recognition technique from FollowerBumpGuard, observation half only)
  attributed to the player by proximity, plus a sustained body-block fallback
  (2 polls inside contact range).
- **VR hand-grab (HIGGS)** — the player physically grabbing the walker (PLANCK
  actor grab). Immediate. Absent HIGGS (flat editions), this path simply reports
  false and bump/body-block still covers interception.

**The event** (register on your own form):

```papyrus
RegisterForModEvent("Standpoint_Interrupted", "OnStandpointInterrupted")
Event OnStandpointInterrupted(Form akWalker, Float afCode)
    ; 1 = combat   2 = player bump/body-block   3 = player hand-grab (VR)
EndEvent
```

**The policy** (`aiInterrupts`, 7th GoTo argument): `0` report only, never
auto-cancel · `1` (default) cancel the session when the PLAYER physically
intercepts — grab, bump, or block — unless the player is the destination ·
`2` also cancel on combat. On auto-cancel the walker is fully released (package
off, pose reset) and the Interrupted event tells you why.

**V2.0 steered sessions:** the same policy, with one difference in what "not cancelled" means: the
engine's travel package resumed by itself after combat; a steered session WAITS (she stands) and
walks on, on a new route, once the reason has been gone 0.5 s — a HIGGS grab with `aiInterrupts 0`
(or with the player as the target; `Interrupted` 3 is still sent), combat with `aiInterrupts 0`–`1`,
and a ragdoll / knock-down / kill move (no event). The bump / body-block rule (`P` 7) does not fire
while she is still planning or waiting. The time limit keeps running during a wait.

### Low-level natives (class `Standpoint`) — prefer StandpointAPI

`SolveSlot`, `GetFurnitureMarkerCount(ref)`, `GetFurnitureMarkerInfo(ref, i)` →
`[x,y,z,headingDeg,effectiveEntryBits(1F/2B/4R/8L),animType,enabled]` (post-gate truth —
excellent for debugging "why did it pick that side"), `GetCurrentProcedureIndex(actor)`
(0 = travelling, 1 = posing on the marker — ALWAYS pair with
`GetCurrentPackageFormID(actor)`, a sitting NPC's own package also reads 1),
`GetEngineSettingFloat/SetEngineSettingFloat(name[, value])` (GMST + "name:Section" INI
lookups; writes are runtime-only), `Log(msg)` → `SKSE/Standpoint.log`,
`GetPluginVersion()`.

v0.8.0: `SolveClock(target, agent, distCm, facingHour, orientHour)` →
`[x, y, z, headingDeg, orientOffsetDeg, gotHour, quality]` — the move request's clock solve
on its own (pure, never moves anyone; clock hours and centimetres as in the C++ section
below; a marker with no model works). The four `Api…` natives are the driver's mailbox for
C++ move requests: never call them. v1.4.0: `WalkStart`, `WalkStop`, `WalkOutcome`,
`WalkResultValues`, `WalkSupported` are the plumbing under `WalkTo` / `GetWalkOutcome` /
`GetWalkResult` / `CanWalkOffNavmesh` / `Release`: call those instead. v1.5.0: `WalkToWaterStart`
and `WaterWalkSupported` are the plumbing under `WalkToWater` / `CanWalkToWater`. v1.5.1:
`WalkFarPackage`, `WalkFarAlive`, `WalkFarEndMode` and `WalkReleaseMode` (and
`StandpointAPI.ApiWalkFarStart` / `ApiWalkFarEnd`) belong to the far phase's own session: never call them
(1.6.0: neutral stubs — the far phase is gone). 1.6.0: `SteerSupported`, `SteerCheck`, `SteerStart`,
`SteerState`, `SteerHandOff`, `SteerStop` (the steered sessions' glue), `MarkerSweep` (the marker census)
and `TestEvent` (the `stpm` test's ear) — never call them. `PathStart` / `PathResult` / `PathCancel`
(1.3.0) are the engine sessions' DETOUR planner. V2.0 registers 45 natives in all.

---

## C++ API (SKSE plugins)

Copy `API/StandpointAPI.h` into your plugin (self-contained, `<cstdint>` only — your
CommonLib version never has to match). After `kDataLoaded`:

```cpp
StandpointAPI::StandpointMessage msg;
SKSE::GetMessagingInterface()->Dispatch(
    StandpointAPI::StandpointMessage::kGetInterface, &msg, sizeof(msg), "Standpoint");
if (msg.GetApiFunction) {
    g_water = static_cast<StandpointAPI::IStandpoint4*>(msg.GetApiFunction(4));  // 1.5.0+: water walk + walk + move + solve
    g_walk  = static_cast<StandpointAPI::IStandpoint3*>(msg.GetApiFunction(3));  // 1.4.0+: walk + move + solve
    g_move  = static_cast<StandpointAPI::IStandpoint2*>(msg.GetApiFunction(2));  // 0.8.0+: move + solve
    g_solve = static_cast<StandpointAPI::IStandpoint1*>(msg.GetApiFunction(1));  // any version: solve only
}
```

- `msg.GetApiFunction == nullptr` → Standpoint is not installed.
- `GetApiFunction(2) == nullptr` → a Standpoint older than 0.8.0: no move request; the
  solver (revision 1) still answers. `GetApiFunction(3) == nullptr` → older than 1.4.0: no walk
  off the navmesh. `GetApiFunction(4) == nullptr` → older than 1.5.0: no walk into water.
- `IStandpoint2` extends `IStandpoint1`, `IStandpoint3` extends `IStandpoint2`, and `IStandpoint4`
  extends `IStandpoint3`: a newer revision's pointer has everything the older ones have.
- **MAIN THREAD ONLY**, both revisions. From any other thread, go through
  `SKSE::GetTaskInterface()->AddTask`.

### The move request (revision 2, v0.8.0+) — one 10-slot line in, one 10-slot line back

*"Walk this NPC to that target, this close, on this side, looking this way, hold this pose
this long — and tell me how it ended."* You hand in a request line. Standpoint does the
walk the same way as `GoTo` (V2.0 on VR: a steered session — see **Standpoint V2.0 in one page**;
SE / AE: the engine session). One result line comes back through your callback when
she arrives **or** when the move is stopped, whichever comes first.

```cpp
void OnMoveDone(std::uint32_t handle, const StandpointAPI::MoveResult& r, void* user)
{
    if (r.outcome == 'C') { /* she is there: r.distanceCm, r.facing, r.orientation are MEASURED */ }
    else                  { /* r.outcome says why not, r.detail says exactly */ }
}

StandpointAPI::MoveRequest q{};          // zero it first: slots 8-10 must be 0
q.who         = npc->GetFormID();        //  1  the NPC
q.where       = target->GetFormID();     //  2  an NPC, furniture, an object, a marker
q.distanceCm  = 150.f;                   //  3  centre of the target to where she stands
q.facing      = 12.f;                    //  4  in front of the target
q.orientation = 12.f;                    //  5  looking at it
q.pose        = 1.f;                     //  6  standard wait
q.holdSeconds = 10.f;                    //  7
std::uint32_t handle = g_move->RequestMove(q, &OnMoveDone, nullptr);

// let her go early (mid-walk or mid-hold): the same line with where = 0
StandpointAPI::MoveRequest stop{};
stop.who = npc->GetFormID();
g_move->RequestMove(stop, &OnMoveDone, nullptr);
```

**The request line** (`MoveRequest`, 40 bytes)

| # | Field | Meaning |
|---|---|---|
| 1 | `who` | The NPC who moves — her reference FormID. A loaded NPC, never the player. |
| 2 | `where` | The target — any reference FormID: an NPC, furniture, an object, a custom marker. Anything you can click in the console. **`0` = release `who` now.** **`who` herself = "hold your place"** (v0.9.5): she is pinned where she stands, facing the way she faces, for `holdSeconds` (`-1` = until a release line); slots 3–5 and 8 are ignored; it answers `C` about 3 s later. Use it to keep a TARGET still before you send another NPC to her - a request is solved against where the target stands at that moment, and a free NPC walks away. |
| 3 | `distanceCm` | How close, in centimetres, from the centre of the target to where she stands. |
| 4 | `facing` | Where around the target she stands, as a **clock hour in the target's own facing**: `0` or `12` = in front of it, `3` = its right, `6` = behind it, `9` = its left. Decimals are fine (`1.5`). Below 0 (`-1`) = Standpoint picks the side she is already nearest to. |
| 5 | `orientation` | Which way **she** looks once there, as a clock hour: `0` or `12` = at the target, `3` = turned to her right, `6` = her back to the target, `9` = turned to her left. |
| 6 | `pose` | `0` none (a plain stand) · `1` standard wait (hands behind the back) · `2` kneel · `3` standing bend. |
| 7 | `holdSeconds` | How long she holds the spot after arriving. `0` = let go at arrival. `-1` = until a release line. |
| 8 | `close` | `0` normal · `1` **close contact**, for an NPC or the player as target (v0.9.0; an older Standpoint ignores it): no distance floor (the point is exactly as far as asked); a walker who has stood still for 3 s within reach of her target has arrived (`C`, detail 9, the real distance in the result) instead of waiting out the 90 s; the player standing near does not cancel the walk (grabbing her still does). **For an NPC target (not the player), close mode also includes: the pair rule** (v0.9.5; VR with HIGGS) - for as long as the request runs, the walker's and the target's character controllers ignore each other, so her body can overlap the target's instead of meeting it at about 56–65 cm (through HIGGS in VR; since 1.2.0 through Standpoint's own collision-filter hook where HIGGS is absent). Only this one pair is affected; walls, furniture and every other actor collide as always. The rule goes off by itself after the request has ended, once the two are 90 u (about 1.3 m) apart. **She walks every step** (v0.9.8) - 0.9.5–0.9.7 moved her over the last stretch with a translate ("the glide"); that slid her across the floor in her wait pose, and it is gone. Measured instead (2026-10-10): within about 50 u of her marker the engine does not walk an NPC, it poses her where she stands; a walk that starts 58 u (85 cm) or more from the point goes all the way, also into the target's space (28 cm and 44 cm centre to centre, on foot). So a walker who stands nearer than that to the asked point first **steps back** to a stop about 1.35 m from the target on the asked side and walks in from there; and a walker on **another side** of the target steps back from it, goes around it at about 1.35 m and comes in on the asked side (v0.9.7). All of it inside the one request - one answer, at the end; it takes some seconds longer. So ask for the new side or the new distance directly; do not walk her out first. If she still ends more than 12 u (17 cm) from the point she walks in once more from further out; after that - or at once, within 12 u - she is taken where she stands: `C`, detail 9, the measured distance and hour in the result. About **30 cm** is the practical limit on foot. **Blocked** (v0.9.9): when the engine itself reports "movement blocked" and she has not moved for a second, Standpoint reacts at once - a blocked step back goes around instead, a blocked walk in is tried once more from further out; and if she cannot get to the point by any way it tries, the answer is `O`, detail 17, and she is released. For the player as target: no pair rule, no stops, no second walk in. **Hold the target still first** (a `where = who` line for her). |
| 9–10 | `open9`, `open10` | Open for later use. Send `0`. |

**The result line** (`MoveResult`, 40 bytes) — the same shape; slots 3–5 are **measured**
at the moment of the report, so a miss shows in the data.

| # | Field | Meaning |
|---|---|---|
| 1 | `who` | As asked. |
| 2 | `where` | As asked. |
| 3 | `distanceCm` | Measured, centre to centre. |
| 4 | `facing` | Measured clock hour around the target, `[0, 12)`. `0.0` = dead in front, so `11.9` and `0.1` are both "almost in front". |
| 5 | `orientation` | Measured clock hour of her own facing, `[0, 12)`. `0.0` = straight at the target. |
| 6 | `outcome` | One letter — table below. |
| 7 | `detail` | Standpoint's reason code — table below. |
| 8–10 | `open8` … `open10` | Open. `0`. |

**Outcome letters (slot 6) and detail codes (slot 7, `StandpointAPI::MoveDetail`)**

| Letter | Meaning | Detail codes |
|---|---|---|
| `C` | **Completed** — she is at the target. | `0` on the point (within 8 u) · `9` stopped short and could not get closer (bodies touching, something in the way); she was turned where she stood — read `distanceCm`. For a close request to an NPC (v0.9.9) detail 9 always means NEAR: on the asked side, within about 50 cm of the point · `16` (0.9.5–0.9.7 only: on the point, the last stretch by a glide - not produced since 0.9.8, nothing slides her) |
| `P` | The **player** stopped her. | `7` bumped or body-blocked · `8` grabbed (VR) |
| `O` | **Obstacle** — she could not get there. | `1` no standable spot at the target · `2` not reached in time (90 s; + 1 s per 150 u beyond 30 m) · `10` bad request (`who` is not a loaded NPC, or `where` is not a reference) · `15` she stopped moving far from where she was sent (100 u or more): released · `17` (v0.9.9, a close request to an NPC) **no way** to the point: blocked, and no other way worked — released; V2.0 also: a close request with no route (at once), a relative spot off the navmesh with no route, the ONE engine leg stuck (`ENGINE LEG ENDED`). A session never answers `O` 18 / 26 — those are walk answers; a session turns them into an engine leg (see **No route**). |
| `D` | **Danger** — hostiles on or near the route; the walk was dropped. | `4` |
| `L` | **Leash** — she strayed more than 50 m from the player. | `5` |
| `X` | Released or replaced. | `3` released (a `where = 0` line, or Papyrus `Release`) · `6` a newer request or a `GoTo` took this NPC over · `12` a save was loaded · `13` a release line for an NPC who had nothing running · `14` the session ended some other way |
| `B` | Refused (**busy**). | `11` all 4 walkers are busy, or Standpoint's scripts are not running |

**The rules**

1. **One answer per request, always through the callback** — refusals too. It arrives on
   the main thread in a later frame, never from inside `RequestMove` itself. `RequestMove`
   returns a handle (> 0) which the callback hands back; it returns 0 only when the callback
   pointer is null. `user` is handed back untouched — keep whatever it points at alive until
   the callback has run.
2. **The answer comes at arrival, not at the end of the hold.** It lands about 3 s after
   she stops walking (she is left to settle, then measured). The hold then runs out by
   itself, with no second report. Poses 2 and 3 play after the report. *(V2.0 steered: the
   answer comes once she is pinned — sooner; measured 3.6–6.4 s from the request for 150 cm.)*
3. **A blocked side is swapped, not failed.** If the asked hour has no room (the navmesh
   snap has to slide the point more than 16 u — a wall, a table), the next hour is tried,
   alternating both ways (11, 1, 10, 2 …). Slot 4 of the result says where she really went.
   *V2.0 steered:* the same hour order, but each candidate is judged by physics (floor, room for
   her body, the target's floor level, standing NPCs, the reach from the target's edge); a hour
   whose spot has no ROUTE gets ONE more try on another side 2 hours away or more. Measured
   1.6.1: around a chair beside a table, `C` 0 / `C` 9 with side swaps.
4. **Distance floors.** An NPC target: without `close`, the solver will not place her
   closer than about 63 cm. With normal collision two NPC bodies meet at about **56–65 cm**
   centre to centre (measured 2026-10-09; a walker stopped by the other body answers `C`,
   detail 9). With `close = 1` and the pair rule she WALKS to about **30 cm** (measured
   2026-10-10: 28 cm behind the target, 43–44 cm for 45 asked). Nearer than about 30 cm is
   not reachable on foot (a 20 cm request ended at 37 cm), and since v0.9.8 nothing slides
   her. An object: its own size plus her body radius. A
   marker with no model: **no floor** — `distanceCm = 0` means "stand exactly on it, looking
   the way it points" (`orientation` then turns her from the marker's own heading).
5. **Every target is treated the same way.** Furniture is an object here: its sit and
   sleep markers are not consulted (v0.8.1). For those, use Papyrus `GoTo`.
6. **`where = 0` lets her go** — mid-walk or mid-hold. Her running request answers `X`
   (unless it already answered `C`), and the release line itself answers `X` too.
7. **One request per NPC, 4 NPCs at a time** — shared with Papyrus `GoTo` sessions. A new
   request for an NPC who already has one replaces it (the old one answers `X`, detail 6).
8. **What stops a walk is not in the line.** Standpoint's defaults apply: the player
   intercepting her cancels it (`P`), hostiles on the route drop it (`D`), the 50 m leash
   cancels it (`L`), 90 s is the time limit (`O` 2; V2.0: + 1 s per 150 u for a start beyond
   30 m — see **Time limits (V2.0)**). Standpoint sells accuracy, not guaranteed
   arrival — on anything but `C`, ask again when it suits you.
9. **A save being loaded answers every open request with `X`, detail 12**, and slots 3–5
   are 0 — do not touch game objects from that callback.
10. **A close request to an NPC is walked as a route when it needs one (v0.9.7 / v0.9.8).** Another
    side of the target: she steps back, goes around and comes in. The same side, but nearer than
    about 85 cm to the asked point: she steps back and walks in again (a point a hand's width away
    is otherwise never walked). By herself, inside the one request (slot 8 above). Rule 8 applies
    on every part of the route, and the 90 s run from the request. If she ends a little
    short on the asked side, the result is `C` 9 with the measured distance and hour. If she
    cannot get there at all, it is `O` 17 (v0.9.9) and she is released.
    *This rule describes the ENGINE session (SE / AE, `Steered = 0`): its legs are `ROUTE` stops,
    each kicked by LEG START (2 s re-evaluate, 4 s re-send, 1.3.0) and given a DETOUR when blocked
    80–1,500 u out (1.3.0).* **V2.0 steered (VR):** no step-back legs. The pair rule is armed BEFORE
    she walks; she gets ONE route: a plan to an approach point just outside the target's keep-out
    circle + one straight radial leg in — or, when she starts within ~1.2 m of the NPC, the ORBIT (a
    side-step round the target, facing it, in chords of 30° at most, then the radial leg). The last
    point is aimed 5 u past the spot along that leg, so her stop lands ON it (1.6.1). Arrival: within
    8 u = `C` 0; within 12 u, or 35 u on the asked side = `C` 9; else `O` 17. No route (after ONE
    more try on another side) = `O` 17, never an engine leg. Measured 1.6.1: 45 cm → 43.9 cm, `C` 0.

**Measured in VR** (v0.8.1, 2026-10-09 — one NPC walking around another on open floor):
asked 150 cm at hours 12 / 3 / 6 / 9 → 150 / 151 / 153 / 151 cm at hours 12.0 / 3.0 / 6.0 /
9.0; orientation asked 12 / 12 / 6 / 3 → 0.0 / 12.0 / 6.0 / 3.0 (12.0 and 0.0 are the same
direction). Pose 0 holds her on the spot the same way pose 1 does. *(historical — the 0.8.1
"not yet run" list: furniture and objects as targets, the blocked-hour swap, a marker with no
model, the outcomes `P`, `O`, `D`, `L`, `B`.)*

**Measured in VR, V2.0 (1.6.1, 2026-11-06, `stpm`, steered):** around an NPC: 150 cm at hours
12 / 3 / 6 / 9 (orientation 0 / 0 / 6 / 3, poses kneel / bend / 0 / standard wait) → 145–146 cm,
`C` 0; close 45 cm → 43.9 cm, `C` 0. Around a CHAIR beside a table: `C` 0 / `C` 9 with side swaps;
close 45 cm → `C` 9 at 109 cm (the side was blocked). On a heading marker that has 3D, at 0 cm: she
stands ~55 cm from it (a marker with a model is an object — by design). Outcomes seen: `C` 0 / 9,
`O` 1 / 17, `X` 3 / 6. **Not yet run in VR (as of V2.0):** a model-less marker as the target under a
steered session, `P` during a steered session, `D`, `L`, `B`, and anything on SE / AE.

**Known gaps**

- A release line is refused (`O`, detail 10) while the NPC's 3D is not loaded. The leash
  frees her anyway once the player is 50 m away.
- The target must be in the loaded area: the navmesh under it is read. (A steered session needs an
  actor target 3D-loaded and in her own interior / worldspace, else the engine session walks.)
- Poses 2 and 3 are one-shots; she then stands in the standard wait for the rest of the hold.
- *(historical, until 1.2.0: "There is no Papyrus version of the 10-slot line" — since 1.2.0 there
  is: `StandpointAPI.RequestMove`, below.)*
- `where = who` (hold your place) is always the engine hold (marker + package), also on VR; for a
  target standing off the navmesh (in water) that hold is untested (a steered hold there was designed
  but not built in 1.6.0).

**Try it without writing a plugin — the `stpt` console test.** Click an NPC in the console
and type `stpt`: she is the walker. Click any target and type `stpt` again: she is sent
around it in six steps through the same request line a plugin would send (another NPC as
the target runs the close-contact program instead: the target is held in place, then 16
steps, about 4 minutes). Every request
and result line is printed in the console and written to `SKSE/Standpoint.log`
(`MOVE #n REQUEST` / `MOVE #n RESULT`). `stpt` while it runs stops it; clicking the walker
and typing `stpt` (no test running) clears her so you can pick another.
**The walk off the navmesh (1.4.0, VR)** has two programs of its own, reached while the player
is **sneaking** (crouched): with a walker chosen, click HER again and type `stpt` → she walks
into the nearest water within 20 m (needs Water Finder 1.6.0; since 1.5.0 this is a `WalkToWater`:
a wading spot she can reach, 0.15–0.6 of her height deep, facing the water, held 20 s); `prid 14`
(you) and `stpt` → she walks to the point 3 m in front of you, then turns to face you. Lines:
`WALK #n …`.
**`stpf` — she keeps in front of you (1.4.1, VR; no sneaking).** Click an NPC in the console and
type `stpf` (with nothing clicked, the `stpt` walker is used): she walks to the point 3 m in front
of you — off the navmesh where needed — turns to face you and stands there. Whenever that point
has moved 64 u (about 1 m) and both you and the point have then stayed still for half a second,
she walks to the new one. In VR "in front" follows your headset, so looking round alone does not
send her until your gaze settles. `stpt` or `stpf` again stops it and lets her go; so do a game
load, her death or her unloading. It also stops by itself — without taking her back — as soon as
anything else takes her: another mod's request for her, a Release, your hand. Lines:
`[TEST] FOLLOW #k …` and `WALK #n …`. (1.6.0: a walk still on its way is RE-AIMED, no stop; while
she sits / rides / fights nothing is sent — one HUD line — and once she is free she walks.)
**`stpm` — every door side by side (1.6.0; 32 steps since 1.6.1).** Click the walker → `stpm`; click a
target (a chair, an object, an NPC) → `stpm`; stand still. Two open-floor spots are fixed in front of
you (P1 ~3 m ahead, P2 1.5 m to its left; checked by the stand probe — none = `MIX not started`).
Through the PAPYRUS doors a consumer uses: A = the same spot by `GoToPoint`, `WalkTo` and a
`RequestMove` on a heading marker (each walked P2 → P1 from a held start); B = `GoTo` FRONT / RIGHT / BACK /
LEFT / NEAREST + `RequestMove` 150 cm at 12 / 3 / 6 / 9 with the poses + close 45 cm; C = the hand-offs
(`X` 6, `Failed` 3); D = two walkers at once (NPC target only); E = `WalkToWater`. HUD `MIX k/32`, one
`[TEST] MIX …` line per step and per event. `stpm` or `stpt` stops it. A test tool — never build on it.

### The move request from Papyrus (v1.2.0+) — the same line, close mode included

A Papyrus mod sends the very same request line through `StandpointAPI.RequestMove` — the same
mailbox, the same walking, the same measured answer as the C++ line above. This is the only
Papyrus way to **close mode** (slot 8).

```papyrus
; once, e.g. in OnInit / OnPlayerLoadGame:
RegisterForModEvent("Standpoint_MoveResult", "OnMoveResult")

; Lydia walks behind Sofia (hour 6), 48.6 cm away, facing her, close contact, holding until released:
Int h = StandpointAPI.RequestMove(lydia, sofia, 48.6, 6.0, 0.0, 0, -1.0, 1)

Event OnMoveResult(Form akWho, Int aiHandle, String asOutcome, Int aiDetail, Float afDistCm, Float afFacingHour, Float afOrientHour, Form akWhere)
    If aiHandle == h
        ; asOutcome "C": she is there (aiDetail 0 on the point, 9 stopped short but near - see afDistCm)
        ; anything else: the letter and aiDetail say why (see "The result line" above)
    EndIf
EndEvent
```

- **Arguments**, in order: who, where, distance cm, facing hour, orientation hour, pose (Int),
  hold seconds, close (Int 0/1) — slots 1–8 of the request line. `where` None = release `who`.
- **Returns** the handle (≥ 1), or 0 when Standpoint is absent or `who` is None.
- **The answer** comes ONCE per handle as `Standpoint_MoveResult`. Instead of the event you can
  ask `GetMoveOutcome(h)` ("-" = still open, else the letter; "" = unknown) and
  `GetMoveResult(h)` (`[distanceCm, facingHour, orientationHour, detail]`).
- **A game load answers nothing.** Forget your open handles in `OnPlayerLoadGame`;
  `GetMoveOutcome` returns "" for them.
- The walker's own events (`Standpoint_Arrived` / `_Failed` / `_Interrupted`) still fire for the
  same walk — if you handle both, handle the walk once.
- **Gate it:** `RequestMove` is new in 1.2.0. Check `GetVersion()` first; on an older Standpoint
  the call fails with a Papyrus log line and returns 0.
- **Without HIGGS** (SE, AE, or VR without HIGGS) the pair rule still works since 1.2.0:
  Standpoint then hooks the game's collision filter itself (measured only with HIGGS so far).
- **V2.0:** on VR the request is a steered session (one page above); the arguments, the event and the
  handles are unchanged. Measured 1.6.1 through this very door (`stpm`): 150 cm → 145–146 cm `C` 0;
  close 45 cm → 43.9 cm `C` 0.

### The solver (revision 1, any version)

`IStandpoint1` (also on a revision-2 / -3 / -4 pointer): `GetBuildNumber()` (slot 0, forever; `2`
since 0.8.0, `3` since 1.4.0, `4` since 1.5.0 — still `4` in 2.0.0), `SolveSlot(targetFormID, relation, agentFormID, standoff, SlotResult&)`,
`GetFurnitureMarkerCount/Info`. **Pure — it never moves anyone.** Use it when your plugin
drives its own movement and only needs the slot. Fixed 56-byte POD, append-only ABI, MAIN
THREAD ONLY, cheap (call on demand, never per-frame).

---

## Walking off the navmesh (v1.4.0, VR)

*"Walk her to that point — over the rocks, onto the bath platform, into the lake up to her
knees — and tell me how it ended."* `GoTo` and the move request walk on the navmesh only: the
engine's travel package stops where the navmesh stops (at the waterline, at a bath's rim).
`WalkTo` plans its own route — the navmesh, bare ground the navmesh does not cover, and
shallow water — and walks her along it with her **own legs**: Standpoint steers her movement
every frame the way a pull on a chain at her hips would (the same engine mechanism PLANCK's
grab-drag uses), and the engine plays her walk. **Nothing slides or teleports her.**

**Skyrim VR only in 1.4.0.** On SE / AE the call exists and answers `B` 19 at once: check
`CanWalkOffNavmesh()` and keep your old way there (pattern 5B). **1.5.0:** on SE / AE too when the
player has switched it on (`SEAE = 1`, see **Edition support**) — `CanWalkOffNavmesh()` tells you.

### From Papyrus

```papyrus
Int     WalkTo(Actor akWho, Float afX, Float afY, Float afZ, Float afFacingDeg = -1.0,
               Float afHoldSeconds = -1.0, Float afDepthFrac = 0.45, Float afStopSubmerged = 0.0)
String  GetWalkOutcome(Int aiHandle)   ; "-" still open · the answer's letter · "" unknown
Float[] GetWalkResult(Int aiHandle)    ; [distLeft u, submergedLevel, detail, x, y, z]; [] while open
Bool    CanWalkOffNavmesh()            ; True on VR with the walk driver installed
```

- **`afX`, `afY`, `afZ`** — the point. For water: on the bed, the way Water Finder 1.6.0 gives it.
- **`afFacingDeg`** — her facing at the end, compass degrees (0 = north, clockwise). Below 0 =
  she keeps the direction she walked in.
- **`afHoldSeconds`** — `-1` (default) she stands there until `Release`, a new `WalkTo` or a
  `GoTo` (she is HELD standing by the same steering at speed 0 — not posed, no marker) · `0` let
  go at arrival (she returns to her own AI at once — a follower walks back to you) · `N` N seconds.
- **`afDepthFrac`** — the deepest water her route may use, as a fraction of HER height (0.05–0.65;
  0 or less = 0.45). Deeper water is walked around. A point deeper than this — or otherwise no
  place to stand (too steep, no floor) — is moved toward her (up to about 4.5 m) to the first
  place she can stand; she then ends short of your point (`C` 9 when more than 32 u). The engine
  makes an actor swim at a submerged level of 0.70.
  **She may START deeper than this (v1.4.1)** — pulled into a bath, waded in on her own: she is
  already there, so it is no reason to refuse. The deep water she stands in (and every bit of
  water deeper than `afDepthFrac` joined to her spot) may then be walked as deep as she already
  stands — up to her depth + 8 u where that still keeps 6 u under the swim level — for her way
  out, or to your point when it lies in that same water. A point in other deep water is moved
  toward her as usual. Once her route has left that water it never goes back into water deeper
  than `afDepthFrac`. If she already stands at the swim level (submerged 0.70 or more), the walk
  answers `O` 21.
- **`afStopSubmerged`** — above 0: she arrives as soon as her submerged level reaches it (0..1),
  wherever that is on the route (`C` 20). "Walk in until the water is at her waist."
- **Returns** the handle (≥ 1), or 0 when Standpoint is absent or `akWho` is None.
- **She belongs to one owner.** If she has a Standpoint session (`GoTo`, `GoToPoint`,
  `RequestMove`), it is released first — on VR only: on SE / AE nothing of hers is touched and the
  walk answers `B` 19. Her older walk answers `X` 6; if it was holding her (walking or standing),
  she stays held until the new walk takes her. A later `GoTo` / `GoToPoint` / `RequestMove` for her
  ends the walk (`X` 6); `Release(akWho)` ends it (`X` 3) and also ends the stand after it.
- **The answer** comes ONCE as the `Standpoint_WalkResult` event: when she has stopped on the point
  and turned to the asked facing, or when the walk is stopped. The hold then runs by itself, with
  no second answer. A game load answers nothing: forget your open handles in `OnPlayerLoadGame`.

```papyrus
RegisterForModEvent("Standpoint_WalkResult", "OnWalkResult")     ; once, on your own form

Event OnWalkResult(Form akWho, Int aiHandle, String asOutcome, Int aiDetail, Float afDistLeft, Float afSubmerged)
    ; afDistLeft  = flat distance from her to the asked point (game units, 70 u = 1 m)
    ; afSubmerged = her submerged level now (0 dry .. 1 under)
EndEvent
```

- **Gate it:** `WalkTo` is new in 1.4.0 — check `GetVersion()` first (an older Standpoint has no
  `WalkTo`: the call fails with a Papyrus log line). Then `CanWalkOffNavmesh()`.

### The answer

| Letter | Detail | Meaning |
|---|---|---|
| `C` | `0` | Arrived on the point, turned to the asked facing. |
| `C` | `9` | Arrived, but more than 32 u from the asked point: the route was PARTIAL (it ends at the nearest place to stand it could reach), the point itself was no place to stand and was moved toward her, she stopped beside it (something in the way at the end), or (1.6.0) the loaded area ended mid-walk. Read `afDistLeft`. `C` 0 always means within 32 u (a walk) — a SESSION's `C` 0 means within 8 u. |
| `C` | `20` | The asked depth was reached (`afStopSubmerged`): she stopped there. |
| `O` | `1` | No route: too far (1.4–1.5.0: more than about 23 m; **since 1.5.1: more than 200 m**, straight), blocked all round, or no place to stand within about 4.5 m of the point. |
| `O` | `2` | Not there in 90 s (since 1.5.1, a walk that starts more than 30 m away: 90 s + 1 s for every 150 u of that distance — 183 s for 200 m). |
| `O` | `26` | (1.6.0) The point is outside the loaded area — answered at once (no ground is loaded there). The loaded area ending MID-walk is `C` 9 at the last checked point instead. A `GoToPoint` (a session) walks ONE engine leg for such a point. |
| `O` | `10` | `akWho` is not a loaded NPC. |
| `O` | `15` | She was pushed off her route (about 1 m or more to the side) — since 1.5.0: and no new route from where she was pushed to. |
| `O` | `18` | Blocked: she stopped making progress (or a door did not open), and no new route from where she stood got her through — since 1.5.0 a new route is planned after every stall in a new place; `O` 18 also when she stalls again where an earlier new route started. |
| `O` | `21` | She would swim (submerged 0.70 or more, or the engine says she swims): she is let go. Also at the start: she already stands at the swim level. |
| `O` | `23` | (`WalkToWater` only, 1.5.0) Water Finder found no water within `afMaxMetres`. |
| `O` | `24` | (`WalkToWater` only, 1.5.0) Water was found, but no wading spot she can walk to (every water Water Finder offered was tried). |
| `P` | `8` | The player grabbed her (VR, HIGGS). |
| `X` | `3` | Released (`Release`, or a C++ release line). |
| `X` | `6` | Replaced (a new `WalkTo` / `WalkToWater`, a `GoTo` / `GoToPoint`, a move request for her). |
| `X` | `12` | A save was loaded (C++ only — Papyrus gets no answer after a load). |
| `X` | `14` | Another end: combat, a ragdoll or knockdown, a kill move, death, furniture, a mount, bleedout, an animation that kept her feet (since 1.5.1: still 2 s after Standpoint broke her idle), a warp. `Standpoint.log` names it. |
| `B` | `11` | Busy: too many walks at once, or the route planner's table is full. |
| `B` | `19` | Not on this runtime (SE / AE with the switch off, or the self-test NOT READY), or the walk driver is not installed. |
| `B` | `22` | Another mod drives her right now (PLANCK's grab, PPB's push walk), or VR Chain has chained her. Since 1.5.1 the log line names the holder when Standpoint can tell. |
| `B` | `25` | (`WalkToWater` only, 1.5.0) Water Finder is not installed, or older than 1.6.0. |

### How she walks (what to expect)

- **The route** is planned first (a fraction of a second, spread over frames): straight when
  nothing is in the way, otherwise around obstacles and drops — she never steps off a drop
  higher than a step; a ledge is walked around to where the ground comes down gently. 1.4 –
  1.5.0: up to about 23 m (1600 u), further refused (`O` 1). *(historical, 1.5.1: up to 200 m — the
  planned route covered the last ~34 m (2400 u); farther out the engine's far phase brought her.)*
  **V2.0: up to 200 m by Standpoint's own path** — see **Speed bands and the long walk (V2.0)**.
  Planning runs in 1.0 ms slices per frame; she stands (held) while the first piece is planned
  (measured 1.6.1: median 90 ms, max 716 ms; 1.5.1 on hills: 6–10 s).
  **Since 1.5.0 the navmesh and the rays work together:** where she and the point are both on
  the navmesh, the engine's navmesh suggests the way and every leg of it is still checked by
  Standpoint's own rays against her body (step height, width, headroom, slope); a leg that fails
  is replaced by a short way round. Where there is no navmesh, the rays alone find the way, as in
  1.4. Several floors over one spot are told apart (she never "arrives" on the gallery above the
  point). NPCs standing still when the route is planned are walked round.
- **Doors (1.5.0).** An unlocked, ordinary door that is closed may be on her route: she stops in
  front of it — outside the reach of its swinging leaf — opens it herself (the engine's own door
  use: she is the one who opens it), waits for it to open (2 s at most) and walks through. A door
  that is open is walked through, and watched until she is past it: if it shuts before she gets
  there, she stops and opens it. A door that does not open counts as blocked (below). Locked
  doors, load doors (to another cell) and doors wired to something else (a trap, a puzzle, a
  quest script) are walls: she never opens them.
  **1.6.1:** a door that is CLOSING when she gets there is waited for until it is shut, then
  opened; a shut door gets one activate, and one more if it is still shut 2 s later; it counts as
  blocked 2 s after the second activate, or 8 s in all. Its first failure in a walk marks no bad spot
  (the doorway may be the only way); the second does. *(Not yet run in VR: a door turned back while
  closing.)*
- **Stairs (1.6.0).** Fixed floors with navmesh under them pass up to a slope of n.z 0.66 (~49°), both
  ways; she never stops ON a stair (a goal on a stair moves to the nearest standable spot on the same
  floor, 16–160 u). VR-run 1.6.0: up and down at Lakeview. A re-aim on a stair plans from her feet
  (1.6.1, not yet run).
- **Her pace — V2.0:** run / jog / walk by the route LEFT (see **Speed bands and the long walk**).
  *(historical, 1.5.1: she walked when she started within 20 m of the point, otherwise jogged and
  walked the last 10 m; 1.4–1.5.0: always walked.)* She turns where the route turns. A route that starts behind her: she turns on the spot first (if she has not
  turned within 2 s, she turns while walking instead). At a sharp corner she stops and turns.
- **An NPC standing in an idle (since 1.5.1).** A standing NPC plays idle animations now and then,
  and while one plays her feet belong to the animation. While she still has to walk to the point,
  Standpoint ends such an animation once per walk (the same "back to the default stance" event its
  own sessions send when they let an NPC go) and then takes her; only an animation that still holds
  her feet 2 s later ends the walk (`X` 14). Never done to an NPC who sits, sleeps, uses furniture
  or is staggered (the first three are refused anyway), and **never after the answer**: during the
  hold your own animation is yours — when it takes her feet, the hold simply ends after 2 s and
  she is handed back (as in 1.5.0), so "hold `-1`, then your animation, then `Release`" works.
- **Blocked on the way?** She is held where she stopped, the spot ahead of her is marked
  for the planner, and a new route is planned from where she stands. 1.4: ONE new route, blocked
  again = `O` 18. **Since 1.5.0:** a new route after every stall in a new place, after a door that
  did not open, and after a push off her route (1.4: `O` 15 at once) — as often as it happens
  within the walk's 90 s; but never twice from the same spot: stuck again within 16 u of where
  any earlier new route of that walk started = `O` 18.
- **At the point** she stops, then turns to the asked facing (about 1.5 s at most; a turn the
  engine leaves unfinished is completed directly). Since v1.4.1 a walker who can walk sideways
  (most NPCs) already turns toward the asked facing over her last ~0.7 m (48 u) while still walking
  to the point, so she arrives turned: a big turn on the spot moved her about 10 u off the point.
  Still more than 16 u from the point after the facing — and still on her planned way — she takes
  ONE short step back to it, facing kept (a walk that asked no facing keeps the heading it had),
  and faces again. If walking sideways does not work for her (she slows down or drifts), Standpoint
  stops trying for the rest of that walk and turns her at the point instead. (On SE / AE — 1.5.0
  with the switch on — she always turns at the point: walking sideways is read on VR only.)
- **She is let go** (her own AI, her own packages) at the end of the hold, on any stop, and on
  any reaction: combat, a hit that ragdolls her, the player grabbing her. A save while she is
  walked hands her back for the save and takes her again at once.
- **Other mods** see it: the mod event `Standpoint_DirectWalk` (sender = her, `strArg`
  `"start"` when Standpoint takes her movement, `"stop|<why>"` when it gives it back, `numArg`
  = the walk handle), and the C++ `IsDirectWalking(formId)`. V2.0: True all the way, and also during
  a steered SESSION's walk (then `IsBusy` is True as well — the session owns her). *(historical, 1.5.1:
  during the far phase the engine's travel ran her: no `"start"` yet, `IsDirectWalking` False.)*

### Speed bands and the long walk (V2.0; since 1.6.0)

The user's rule (2026-11-06): *"jog until 3m, run until 15m"*; a pace that would last under 1 s is
skipped. It applies to every steered walk — `WalkTo`, `WalkToWater`, `RequestWalk*` and the steered
sessions. Nothing to ask for, no new field.

| Route LEFT (along the route, not straight) | Pace (normalised speed told to the engine) | Measured in VR (1.6.0 run) |
|---|---|---|
| more than 1,050 u (15 m) | **run** (2.0) | 280–293 u/s |
| 1,050 → 210 u (15 m → 3 m) | **jog** (1.5) | 177–191 u/s (1.5.1 run: 212–221 u/s, another walker) |
| the last 210 u (3 m) | **walk** (1.0) | 72–78 u/s (1.6.1 run: median 72 u/s) |

- **Downshifts are anticipated** (ramps 1.5 per second): she is at her jog at 1,050 u and at her walk at
  210 u, so the stop at the point, the turn to the asked facing and the one short step back all happen at
  her walk, as before.
- **Slower where it matters:** a turn over 30° ahead is jogged, over 60° (and a sharp corner, a door)
  walked; she also slows for water edges (1.6.1: only while she really wades — submerged 0.05 or more —
  or from just before the water's edge), legs steeper than 27°, stairs (walked), drop edges, and the end
  of the route checked so far.
- **The long walk — our own path (the engine's far phase is GONE since 1.6.0):**
  1. a **skeleton** over the LOADED navmesh's own triangles (her interior cell, or the attached exterior
     cells) — it never uses a load door; navcut holes are respected; two navmeshes that do not meet are
     bridged by "virtual links" ≤ 256 u, which the rays must then prove;
  2. **verified chunks**: the first ≥ 1,400 u is ray-checked before she starts; the next (≤ 2,400 u) is
     asked while she walks, once less than 1,400 u of checked route is left, and appended without a stop
     (measured 1.6.0: chunks ready in 0.1–0.45 s; no stop between chunks);
  3. a re-plan (a moved goal, a failed chunk, a re-aim) is planned from where she will be and swapped in
     while she moves (1.6.1: never more than 300 u ahead of her).
- **Limits:** a point outside the loaded area answers **`O` 26 at once** (Standpoint walks only where the
  game has the ground loaded); the loaded area ending mid-walk answers **`C` 9 at the last checked point**;
  beyond 200 m (14,000 u) **`O` 1** "too far"; the time limit beyond 2,100 u is 90 s + 1 s per 150 u.
  (A SESSION turns `O` 26 into its ONE engine leg — **No route**.)
- `IsDirectWalking` is True all the way. Log lines: `WALK #n SPEED …`, `CHUNK …`, `EXTEND …`, `SWAP …`,
  `SKELETON …`, `PATH #n …`.

### Speed by distance and the far phase (v1.5.1) — HISTORICAL (replaced in 1.6.0 by the section above)

*"StandPoint is a travel mod for up to 200 m: run if more than 30 m, jog from 30 to 10, walk the
last 10 m — and if she starts within 20 m of her target, just walk."* (the user's rule). It
applies to every `WalkTo`, `WalkToWater` and `RequestWalk` / `RequestWalkToWater`; nothing new
to ask for, no new field.

| She starts (straight distance to the point) | What she does |
|---|---|
| within 20 m (1400 u) | walks the whole way (as 1.4–1.5.0) |
| 20–30 m | jogs while more than 10 m (700 u) of her route are left, walks the last 10 m |
| 30–200 m (VR) | **the far phase**: RUNS with the engine's own travel until she is 30 m from the point; then the walk off the navmesh takes her over **without stopping** — jog, the last 10 m walked |
| beyond 200 m (14,000 u) | `O` 1 "too far (beyond 200 m)" |

- **Every change of pace is gradual** (about a third of a second walk ↔ jog). On a jogged route
  she is back at her walk before a sharp corner, a door on her route and any turn sharper than
  60° — the stop at the point, the turn to the asked facing and the one short step back all
  happen at her walk, exactly as before.
- **The far phase** is Standpoint's own travel session (the one `GoToPoint` uses), aimed at the
  point — or at the nearest navmesh point when the point is off the navmesh (in water, on a
  rock) — with the session's package set to RUN (jog within 30 m, walk within the last 10 m while
  the hand-over is being planned). The engine walks the far part: its own navmesh paths, doors,
  everything it normally does. Within 30 m the route is planned from where she is while she keeps
  going, and the walk takes her over from there, moving. If no route is found, she keeps the
  engine's travel as far as it goes and the walk answers from where it ends (it plans once more
  from there: no way = `O` 1, stuck on the way = `O` 18).
- **A walk that starts more than 30 m away has more time:** 90 s + 1 s for every 150 u of that
  distance (183 s for 200 m).
- **Ending it:** exactly as any walk — a new walk for her (`X` 6), `GoTo` / `GoToPoint` / a move
  request (`X` 6), `Release` (`X` 3), the player's hand (`P` 8), combat, death and the like
  (`X` 14), a game load. The far session always ends with the walk; nothing is left on her.
  **A new walk for her while she runs** (asking again for a moved target) does not stop her. A new
  walk that is far itself turns the running travel toward its own point. A new walk within 30 m
  takes the running travel over: its route is planned from where she is at that moment, it takes
  her on the move at her pace, and only then does the travel end (without letting her go).
  A far session that ends while she is knocked down, ragdolled, held by a hand or fighting —
  whether the walk ends or you call `Release` — is removed without resetting her animation.
- **Skyrim VR only.** On SE / AE (with the switch on) there is no far phase: a walk beyond the
  planned route's reach answers `O` 1 as in 1.5.0; the walk / jog rule applies.
- **Not yet run in VR** at the time of 1.5.1's release. The log lines `WALK #n DISTANCE …`,
  `WALK #n FAR …`, `WALK #n SPEED …` and `WALK #n IDLE BREAK …` say what happened.

### From C++ (revision 3)

```cpp
auto* walk = static_cast<StandpointAPI::IStandpoint3*>(msg.GetApiFunction(3));   // null before 1.4.0

void OnWalkDone(std::uint32_t handle, const StandpointAPI::WalkResult& r, void* user)
{
    // r.outcome + r.detail: the table above; r.distLeft (u, flat), r.submerged, r.x / r.y / r.z: where she is
}

StandpointAPI::WalkRequest q{};          // zero it first: slots 9-10 must be 0
q.who           = npc->GetFormID();      //  1
q.x = wx; q.y = wy; q.z = wz;            //  2-4  the point
q.facingDegrees = bearing;               //  5    < 0 = keep the last travel direction
q.holdSeconds   = -1.f;                  //  6    -1 stand until released, 0 let go, N seconds
q.depthFrac     = 0.f;                   //  7    0 = 0.45 of her height
q.stopSubmerged = 0.f;                   //  8    0 = off
std::uint32_t h = walk->RequestWalk(q, &OnWalkDone, nullptr);
```

- `RequestWalk`: MAIN THREAD. Returns a handle (> 0), or 0 when the callback is null. The answer
  ALWAYS comes through the callback (refusals too), once, on the main thread, in a later frame.
- `IsDirectWalking(formId)`: any thread — True while Standpoint drives her (walking or standing; V2.0:
  also a steered session's walk). *(historical, 1.5.1: False during a walk's far phase.)*
- **To end a walk or a stand:** a move-request release line for her (`where = 0`, answers `X` 3),
  or a new `RequestWalk` / `RequestMove` for her (`X` 6).
- `WalkRequest` and `WalkResult` are fixed 40-byte lines; slots 9–10 are the room for growth.

### Into the water: WalkToWater (v1.5.0)

*"Walk her into the nearest water she can really reach, and let her stand in it."* One call:
Standpoint asks **Water Finder** (1.6.0 or newer) for the nearest water, then plans from HER
toward it and stops the plan at the **first wading spot she can walk to** — water between two
depths you give (fractions of her height), with water round it (at least 5 of the 8 spots
around it), and a place she can stand. Deeper water is never crossed; a dock, a ledge top or
the foot of a wall is never the answer. When that water has no wading spot she can reach,
Standpoint tries the next water Water Finder offers (Water Finder 1.6.2 or newer), all inside
the walk's 90 s. Then she walks there with her own legs, exactly like `WalkTo`, and ends
**facing out over the water** (the direction from where her route entered the water to the
spot).

```papyrus
Int  WalkToWater(Actor akWho, Float afMaxMetres = 20.0, Float afDepthMinFrac = 0.15,
                 Float afDepthMaxFrac = 0.45, Float afHoldSeconds = -1.0)
Bool CanWalkToWater()   ; True when the walk off the navmesh runs here AND Water Finder 1.6.0+ is installed
```

- **`afMaxMetres`** — how far Water Finder looks (0 or less = 20). The walk itself reaches about
  23 m in 1.5.0 (water further away answers `O` 1 / `O` 24) and 200 m since 1.5.1. *(historical,
  1.5.1: the far phase ran her toward the water's nearest navmesh point first.)* V2.0: our own path
  all the way, within the loaded area (`O` 26 beyond it).
- **`afDepthMinFrac` / `afDepthMaxFrac`** — the wading spot's depth, as fractions of HER height H
  (defaults 0.15 and 0.45). The deeper bound is also the deepest water her route may use (0.05–0.65);
  the engine makes an actor swim at 0.70.
- **The wade priority (V2.0, since 1.6.0 — the user's rule: "thigh is the max depth"):**
  1. **entry** = the first reachable **shin-deep** spot (`afDepthMinFrac`, 0.15 H) with water round it
     (5 of its 8 neighbours) that passes the stand-spot test;
  2. **scan 1** within 2 body lengths (2 H) of the entry: a **thigh-deep** spot (0.38 H or more) ends the
     search at once; else the nearest **knee-deep** spot (0.28 H);
  3. **scan 2** within 2 H of that knee spot: the nearest thigh-deep spot;
  4. never deeper than `afDepthMaxFrac` (default **0.45 H** = the thigh max) on any step; start → entry →
     knee → thigh is **ONE route** — she never stops between the steps. A cap below the knee keeps the
     1.5.x behaviour (the entry spot only).
  5. A seed below a ledge: the planner first floods the floor toward the nearest reachable shin-deep water
     (1.6.1: 4,000 squares / 10,000 questions, headed toward the seed).
  Measured 1.6.0 (VR): bathhouse submerged **0.30** (knee; no thigh found), lake pond **0.41** (thigh) — the
  morning's 1.5.1: 0.17 / 0.20. *(Not yet run: the ledge pond on the FIRST try with 1.6.1's flood.)*
- **Plain `WalkTo` / `stpf` walks wade too:** `afDepthFrac` 0.45 is the default, so a dry walk may cross
  water up to her thighs when that is the way (the user, 2026-11-06: *"Yes, up to thigh as now"*). Pass
  a smaller `afDepthFrac` to keep her drier.
- **`afHoldSeconds`** — as `WalkTo`: `-1` she stands there until `Release` / a new walk / a `GoTo`.
- **The answer** is the same `Standpoint_WalkResult` event as `WalkTo` (one handle space;
  `GetWalkOutcome` / `GetWalkResult` work on it). `afDistLeft` is measured to the wading spot it
  chose: `C` 0 = she stands within 32 u of it. New answers: `O` 23 no water within `afMaxMetres`,
  `O` 24 water but no wading spot she can reach, `B` 25 no Water Finder 1.6.0+.
- Her Standpoint session (`GoTo` etc.) is released first only where the water walk can run
  (`CanWalkToWater()`); everything in "She belongs to one owner" above applies.
- **Gate it:** new in 1.5.0 — check `GetVersion()` first, then `CanWalkToWater()`.

```papyrus
If StandpointAPI.GetVersion() >= "1.5.0" && StandpointAPI.CanWalkToWater()
    Int h = StandpointAPI.WalkToWater(npc)          ; 20 m, 0.15-0.45 of her height, stand until Release
    ; OnWalkResult: "C" she stands in the water (afSubmerged = how deep); "O" 23 / 24 no water / no way in
EndIf
```

**From C++ (revision 4):**

```cpp
auto* water = static_cast<StandpointAPI::IStandpoint4*>(msg.GetApiFunction(4));   // null before 1.5.0

StandpointAPI::WaterWalkRequest q{};     // zero it first: slots 7-10 must be 0
q.who           = npc->GetFormID();      //  1
q.maxMetres     = 20.f;                  //  2  <= 0 = 20
q.depthMinFrac  = 0.15f;                 //  3  <= 0 = 0.15 (x her height)
q.depthMaxFrac  = 0.45f;                 //  4  <= 0 = 0.45 (0.05-0.65)
q.holdSeconds   = -1.f;                  //  5  -1 stand until released, 0 let go, N seconds
q.facingDegrees = -1.f;                  //  6  < 0 = she faces the water; else a compass facing
std::uint32_t h = water->RequestWalkToWater(q, &OnWalkDone, nullptr);   // the same WalkCallback / WalkResult
```

- MAIN THREAD. One answer through the callback, as `RequestWalk` (refusals too), the same handle
  numbers. `WaterWalkRequest` is a fixed 40-byte line; slots 7–10 are the room for growth.

*(historical: "Not yet run in VR at the time of 1.5.0's release" — since then the wading-spot search
and the water facing ran in VR: 1.5.0 bathhouse (planned, stopped by an idle lock), 1.5.1 0.17 / 0.20,
1.6.0 0.30 / 0.41. The next-water step (Water Finder 1.6.2 candidates) ran in the 1.6.0 pond walk.)*
The log lines `WALK #n WATER …`, `WALK #n PLAN …| WADE …` and `WALK #n WADING SPOT …` say what happened.

**`WalkTo`, measured once so far.** The first VR run (1.4.0) walked an NPC 236 u over dry ground in
7.6 s at her own walk (35–41 u/s), within 9 u of the line, and stopped about 11 u short of the point;
the steering is the mechanism three VR mods use every day (PLANCK, PPB, VR Chain), and the route
planner is 1.3.0's with new floor and water rules. Water, ledges and the hands are not measured
yet — those times and distances are design values; so are 1.5.0's navmesh-guided routes, doors
and repeated new routes (not yet run in VR at the time of 1.5.0's release). Read the `WALK #n`
lines in `Standpoint.log`; they say what happened at every step (`DOOR …`, `NEW ROUTE #k …`).
**1.5.0's first VR run** (49 walks, one NPC, mostly round a house on a hill): every walk answered
once; 14 arrivals 3–6 u from the point, facing within 1°; a wade into a lake round a 94 u ledge to
30 u of water; her walk ≈ 83–90 u/s. Two walks ended `X` 14 because a standing idle held her feet
(1.5.1 breaks it); long plans on steep hills made walks slow to start (the planner's 1.5.1 work).
*(historical: "1.5.1's speed by distance, the far phase and the idle break are not yet run in VR" — they
ran on 2026-11-06: jog 212–221 u/s, IDLE BREAK worked; the far phase walked into the house because of a
marker bug of ours and was removed in 1.6.0.)*
**V2.0 (1.6.0 + 1.6.1 runs, 2026-11-06):** 1.6.1: 79 walks, 64 × `C` 0 (median 3.5 u, max 10 u; facing
≤ 2.3°), plans median 90 ms / max 716 ms, the longest planning slice 1.08 ms; IDLE BREAK 16 / 16. 1.6.0:
the bands as told (run 280–293, jog 177–191, walk 72–78 u/s), chunks never stopped her, stairs up and
down, wading 0.30 / 0.41, doors opened in 0.1–0.9 s (one CLOSING door failed — fixed in 1.6.1).

---

## Patterns (copy these)

### 1. Send an NPC to stand somewhere, then get back to their life

```papyrus
StandpointAPI.GoTo(npc, campfireRef, 0, 0.0, 20.0)   ; nearest slot, hold 20 s, auto-release
```

### 2. A specific side of furniture, facing it

```papyrus
Int h = StandpointAPI.GoTo(npc, bedRef, 4)   ; LEFT side of the bed, hold until told
; ... your scene plays ...
StandpointAPI.Release(npc)
```

### 3. Walk to an object and interact-pose at it

```papyrus
; coin on the floor -> she walks, faces it, KNEELS; cup on a table -> she BENDS
StandpointAPI.GoTo(npc, itemRef, 0, 0.0, 5.0, 1)     ; pose auto, hold 5 s
; do your actual pickup logic in OnStandpointArrived (add item, play sound, etc.)
```

### 4. Stand relative to an actor (the player included)

```papyrus
StandpointAPI.GoTo(npc, Game.GetPlayer(), 2, 100.0)  ; player's RIGHT, conversation range
; facing is toward the target automatically. Close-range note: the engine biases NPC
; pathing away from the player inside ~100 u (fAvoidPlayerDistance) during TRANSIT;
; a parked NPC is NOT pushed out (measured: zero drift at 49 u).
```

For a continuously MOVING target, re-issue `GoTo` when the target has moved ≥ 100 u —
the engine's own re-path hysteresis (`fAIMoveDistanceToRecalcTravelToActor = 100`);
re-anchoring faster than that fights the engine. *(That advice is for the ENGINE sessions.
V2.0 steered `GoTo` / `RequestMove` follow a target that moves 32 u, or an NPC target that
turns 30°, by themselves while she walks; a re-issue of the same kind while she walks re-aims her
running walk instead of restarting it. Once she has arrived and is pinned, a moving target is no
longer followed — ask again.)*

### 5. THE WATER PATTERN — composing with Water Finder (the canonical composition)

The division of labour, settled the hard way: **Water Finder finds, Standpoint walks,
YOUR MOD orchestrates.** Until 1.4.x neither framework called the other; since 1.5.0 `WalkToWater`
does the finding through Water Finder for you (5-0).

**5-0. Standpoint 1.5.0+, Water Finder 1.6.0+: ONE call.** `WalkToWater` asks Water Finder, picks a
wading spot she can reach, walks her in and turns her to face the water (see **Into the water:
WalkToWater**). Use 5A only when you need your own point.

```papyrus
If StandpointAPI.GetVersion() >= "1.5.0" && StandpointAPI.CanWalkToWater()
    If !WaterFinder.IsActorInWater(npc)                ; already wet: nothing to do
        Int h = StandpointAPI.WalkToWater(npc, 20.0)   ; then OnWalkResult as below ("O" 23 / 24: no water / no way in)
    EndIf
EndIf
```

**5A. Standpoint 1.4.0+ on Skyrim VR, Water Finder 1.6.0+: she WALKS in.** Water Finder 1.6.0
puts its water point IN the water, close to land, on her side (z = the bed). `WalkTo` plans the
way there — round a ledge, down the gentle part of the bank, into the shallows — and walks her
in. No shore stop, no glide.

```papyrus
; 0) already wet? then there is nothing to do
If WaterFinder.GetPluginVersion() == "" || WaterFinder.IsActorInWater(npc)
    Return
EndIf
If !StandpointAPI.CanWalkOffNavmesh()
    ; SE / AE, or Standpoint older than 1.4.0: use 5B
    Return
EndIf

; 1) FIND - near: the walk was local until 1.5.0 (about 23 m at most; 200 m since 1.5.1), 20 m is a good radius
If !WaterFinder.RequestNearestWater(npc, 20.0, False)
    Return   ; no water near her - handle in your mod
EndIf
Float[] wet = WaterFinder.GetWaterPosition(npc)    ; in the water, close to land; [2] = the bed
Float facing = WaterFinder.GetWaterBearing(npc)    ; she ends looking out over the water

; 2) WALK - in, and stand there until you Release her (hold -1)
Int h = StandpointAPI.WalkTo(npc, wet[0], wet[1], wet[2], facing, -1.0)

; 3) in OnWalkResult (register "Standpoint_WalkResult" on your own form):
;    "C" -> she stands in the water: afSubmerged tells how deep (do your bathing, then Release(npc))
;    "C" 9 -> she stands in the water but short of the point (it was deeper than her afDepthFrac): fine for a bath
;    "O" 1 -> no way in (a cliff all round): tell the player, or try 5B's shore
;    "O" 21 -> she would have had to swim: she was let go where she stood
;    anything else: the table in "Walking off the navmesh"
```

- **Depth.** (5A — your own point; `WalkToWater` (5-0) does the shin → knee → thigh search for you.)
  Water Finder aims for water at least 20 u deep (about knee height); where the bed
  drops fast its point can be up to 75 u deep. `WalkTo`'s default limit is 0.45 of her height
  (about 58 u for a 128 u tall NPC): a deeper point is moved toward the shore until it is
  shallow enough (`C` 9 if that is more than 32 u). Pass a larger `afDepthFrac` (up to 0.65) to
  let her go deeper, or `afStopSubmerged` (e.g. 0.35) to stop her as soon as she is that deep on
  the way.

**5B. SE / AE, or Standpoint before 1.4.0: the shore, then the glide.** The travel package
walks on the navmesh only, so she is walked to the shore and the last wet step is a translate
(it slides her — the only way there without 5A).

```papyrus
; 0) already wet? then there is nothing to do
If WaterFinder.GetPluginVersion() == "" || WaterFinder.IsActorInWater(npc)
    Return
EndIf

; 1) FIND — detection only. requireNavigable=False is DELIBERATE: WF's navigable
;    check rides Actor::CanNavigateToPosition, which is unsafe (see Pitfalls), and
;    reachability is the travel layer's job anyway.
If !WaterFinder.RequestNearestWater(npc, 160.0, False)
    Return   ; no water in loaded range - handle in your mod
EndIf

; 2) WALK — to the SHORE (land-side, navmeshed), NEVER the wet point. A travel
;    package aimed at open water stalls forever: navmesh ends at the waterline.
Float[] shore = WaterFinder.GetShorePosition(npc)
Float[] wet   = WaterFinder.GetWaterPosition(npc)
Float facing  = ; compass bearing shore->wet (0=N, 90=E, clockwise; Papyrus has no
                ; Atan2 - rebuild from Math.Atan with quadrant fixes, atan2(dx, dy))
Int h = StandpointAPI.GoToPoint(npc, shore[0], shore[1], shore[2], facing)

; 3) WADE — in OnStandpointArrived: drop the pin, glide the last wet step.
StandpointAPI.Release(npc)                                   ; the pin would fight the glide
npc.TranslateTo(wet[0], wet[1], wet[2], 0.0, 0.0, npc.GetAngleZ(), 60.0, 90.0)
; poll WaterFinder.IsActorInWater(npc) ~0.5 s up to ~15 s, then npc.StopTranslation()
```

**Interior caveat (bath houses):** Water Finder's ring sampler probes through walls;
in complex interiors it can report phantom water (wrong side of a wall, below the
floor) while missing a placed-water tub nearby. Its shore point is then garbage — and
`GoToPoint` will faithfully send your NPC toward garbage (Golden Rule 1). **Sanity-check
interior shore points before sending**: reject a shore whose Z is far below the NPC's
feet, or whose summary distance disagrees wildly with `GetWaterDistance`. Outdoors the
pattern is field-proven end to end (found at 110 m → walked → 0.06 u arrival → waded in).

---

## Pitfalls (each one cost us a debugging session — do not rediscover them)

**V2.0 first:**
- **`Release` after a `Failed` 3 can end a NEWER owner's walk.** `Failed` 3 often means "someone else
  took her" (a `WalkTo` released your session). `Release(npc)` ends WHATEVER Standpoint runs for her,
  the new owner's walk included. Release only what you still own (check your own handle: `GetMoveOutcome`
  / `GetWalkOutcome` = "-", or `IsBusy` for a session you started).
- **A handle > 0 is not a spot** (steered `GoTo` / `GoToPoint`): wait for `Arrived` / `Failed`.
- **"`C` 0" is not one tolerance:** a session's `C` 0 = within 8 u of the spot; a walk's `C` 0 = within
  32 u of the point. Judge by the measured distance when it matters.
- **A walk taking her from your session answers `X` 3 + `Failed` 3**, not `X` 6 (the header's REVISION 2
  text "X 6 = a newer request / GoTo took this NPC over" is about SESSIONS replacing sessions).
- **Furniture under a steered `GoTo`:** its allowed entries are the candidates, and the relation only
  RANKS them — a chair with one allowed entry (Lakeview's NobleChair02) gives the same spot for FRONT /
  RIGHT / BACK / LEFT / NEAREST (measured 1.6.1: 5 × 3.6 u). Only when EVERY entry is refused (no room) is
  she stood at a side of its box (the asked side first, at all three distances, 1.6.1); when an entry is
  TAKEN (someone sits there) she is never stood beside it — every entry taken = `Failed` 1 at once.
- **A marker WITH a model is an object:** `RequestMove` at 0 cm on an `XMarkerHeading` that has 3D stands
  her beside it (~55 cm, measured 1.6.1). Use a model-less marker, or `GoToPoint`, for "exactly here".
- **A plain walk may wade:** `WalkTo`'s default `afDepthFrac` 0.45 lets a dry route cross water up to her
  thighs. Pass less if you want her dry.
- **An engine leg ends fast now:** a far engine leg that the engine reports MOVEMENT BLOCKED for ~1 s
  (the player standing in her way, a passing NPC) ends with `Failed` 7 — ask again (watch item).

- **v0.9.2 - no stuck recovery, and "far away" is never "arrived".** Until 0.9.1 every walk ran
  under SeverActions' stuck monitor. Measured 2026-10-09: a walker who stood still 40.6 u from
  her point for 7.5 s was "leapfrogged" 500 u straight ahead — through her target and a wall,
  into the void under the cell — and Standpoint then reported her as arrived where she fell.
  Now: no Standpoint walk starts that monitor, and a walker who has stopped 100 u or more
  from her point (or 100 u above / below it) is released with `Standpoint_Failed` code 6
  instead of `Standpoint_Arrived`. If your mod treated every `Arrived` as "she is there",
  nothing changes; if it relied on a wedged walker being freed by a teleport, it now gets
  code 2 after 90 s and must decide for itself. *(V2.0: or code 7 within seconds when her
  engine leg is stuck — see **No route**.)*

- **v0.8.1 - a walker ON her point but still facing wrong is turned before she is reported.**
  When she is at the point but more than 5° off after the settle waits, the marker is moved
  to where she stands and, if the engine still does not turn her, she is turned directly
  (the log says `ON the point but N deg off`). `Standpoint_Arrived` then comes up to 3 s
  later than before and carries the corrected `afAngErr`. Applies to every session: `GoTo`,
  `GoToPoint`, the C++ move request. *(V2.0 steered: the pin waits up to 2.5 s for the engine's turn,
  then turns her directly when she is still more than 5° off — `turned directly` in the log.)*

- **v0.7.2 - an NPC target's side can be swapped.** When the requested side's point is not
  standable (a table or a wall right there: the navmesh snap slid it more than 16 u), the
  solver takes the next side instead (the log names it: `Solve: ok … at=N deg`). Read the
  returned point if the side matters to you; the arrival facing is always toward the target.

- **`GoToPoint` = no validation** (the ENGINE path). Off-navmesh input → the NPC walks somewhere you did
  not intend, possibly through a load door. Solve/snap your own raw points. (V2.0 steered: see Golden
  Rule 1.)
- **A held walker looks "frozen."** That is the hold. Set `afHoldSeconds` or `Release()`.
- **`Actor::CanNavigateToPosition` (SKSE/C++) is UNSAFE**: it shares its address with
  `UpdateNavPos`, MUTATES its second argument (overwrites your buffer with a point near
  the agent), and returns false for everything when the agent is sitting. Never use it
  as a reachability oracle; never pass it a buffer you care about.
- **`GetCurrentProcedureIndex` counts procedures at runtime** (0 = Travel,
  1 = UseIdleMarker) — NOT record tree-node indices — and a sitting NPC's own package
  also reads 1. Always pair with `GetCurrentPackageFormID`.
- **Aimed spells only select ACTORS.** A projectile hitting furniture never creates a
  magic effect. To let a player pick furniture/objects in VR, read
  `Game.GetCurrentCrosshairRef()` (the hand-activation-ray target) from a self-cast.
- **`PlayIdle`/anim events on NPCs are fine in VR** (the "PlayIdle fails in VR" lore is
  about the PLAYER under VRIK). But zero-root-translation is a PER-CLIP property —
  verify any new clip before assuming the NPC stays put (exit-class clips often move).
- **Recompiled `.pex` needs a full game restart** — the VM caches script code at launch.
- **Do not carve up the engine's close-range avoidance:** `fAvoidPlayerDistance` (100)
  affects transit only; parked NPCs are stable. Leave it alone unless a specific
  approach visibly fights it.

## Files

```
mods/Standpoint/
  Standpoint.esp            ESL: quest + 4 marker aliases + 4 travel packages + 2 idle markers + test spells
  SKSE/Plugins/Standpoint.dll
  SKSE/Plugins/Standpoint.ini   (1.5.0) [DirectWalk] SEAE - the SE / AE switch of the walk off the navmesh (VR ignores it)
                                (1.6.0) [Session] Steered (commented out = VR 1, SE / AE 0), [Cleanup] SweepMarkers (0)
  Scripts/ + Scripts/Source/  StandpointAPI (public), Standpoint (natives), driver, test effects
  API/StandpointAPI.h       the C++ contract: the water walk + the walk + the move request + the solver (copy into your plugin)
  MODDER_API.md             this document
```

Source: the `src/` (C++) and `papyrus/` folders of the Standpoint repository; how to build
them is in its `BUILD.md`, the version history in `CHANGELOG.md`.

**Versioning:** `GetVersion()`/`GetPluginVersion()` return semver. The Papyrus surface
grows by NEW NAMES only: an existing function never gains a parameter — not even an optional
one, because Papyrus fills defaults in the CALLER's compiled script and the game refuses a call
whose argument count differs ("Incorrect number of arguments") — and an existing event never
gains a value (a handler with the old count is never called again). New behaviour comes as a
new function or a new event; gate it on `GetVersion()`. The C++ vtable is append-only,
slot 0 fixed forever; revision 2 (0.8.0) only added `RequestMove` after revision 1's
functions, revision 3 (1.4.0) only added `RequestWalk` and `IsDirectWalking` after
revision 2's, and revision 4 (1.5.0) only added `RequestWalkToWater` after revision 3's.
`MoveRequest` and `MoveResult` are fixed 40-byte lines — slots 8–10 are the room for growth;
`WalkRequest` and `WalkResult` likewise, slots 9–10; `WaterWalkRequest` (1.5.0), slots 7–10. The
detail codes 18–22 are new in 1.4.0 and come from walks only; 23–25 are new in 1.5.0 and come from
water walks only. 1.5.1 changed no line, no slot and no existing Papyrus signature (its new
functions are for Standpoint's own driver; the walks reach 200 m, their pace follows the distance;
the answers keep their codes). 1.6.0 added the detail code 26 (walks only), the INI keys `[Session]
Steered` / `[Cleanup] SweepMarkers` and the steered sessions — no line, no slot, no existing Papyrus
signature, no event changed. 1.6.1 and 2.0.0 changed none either (`GetBuildNumber` stays 4). Test spells
and the `stpt` / `stpf` / `stpm` console tests may change freely — never build on them.
