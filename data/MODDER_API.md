# Standpoint — Integration Guide (v0.9.9)

**"Walk this NPC to a precise stand-slot at a target and have them arrive facing the
right way, then hold there."**

This document is the complete contract for integrating Standpoint — written for both
human modders and AI agents. Everything in it is field-measured in Skyrim VR on a
~2,000-mod load order, not assumed. If you read nothing else, read **The Golden Rules**
and the **Patterns** section.

**Writing an SKSE plugin?** Since v0.8.0 you do not need Papyrus at all: hand Standpoint
one 10-slot request line (who, where, how close, which side, facing which way, pose, how
long) and get one measured result line back. Jump to **C++ API → The move request**.

---

## What Standpoint guarantees (measured)

| Property | Measured value |
|---|---|
| Arrival facing error | median **0.12°**, max ~1.5° (engine-driven: `fIdleMarkerAngleTolerance=1`) |
| Arrival position error | typically **centimetres** (0.06–25 u observed) |
| Hold stability | **exactly zero drift** while the session holds (engine pin) |
| Stuck recovery | **None since v0.9.2.** Nothing moves a walker by force. A walker who gets stuck on the way fails after 90 s (code 2). A walker who is found 100 u or more from her point when her arrival is handled is released (code 6) instead of being reported as arrived. A close request to an NPC reacts sooner: see slot 8 `close` and detail 17. |
| Concurrent walkers | **4** |
| Travel timeout | 90 s → `Standpoint_Failed` code 2 |
| C++ move request, open floor around an NPC (v0.8.1) | asked 150 cm → **148–153 cm**; asked clock hour → **within 0.1 hour**; asked orientation → **within 0.6°** (5 walks, one VR run) |

Slots come from **real furniture markers** (the NIF's entry points AND the record's
per-marker enable bits MINUS its disabled points — the full legality gate) or, for
anything without markers (tables, chests, loose objects, actors), from the target's
bounds with side selection in the target's own facing frame. Every slot is
**navmesh-snapped** before anyone walks.

## Requirements

- SKSEVR + VR Address Library (SE/AE builds: SKSE64 + Address Library) — the DLL is
  multi-runtime.
- PapyrusUtil (package overrides).
- **Optional:** Water Finder (the water pattern below). SeverActions is no longer used
  (v0.9.2): its stuck recovery teleports — see Pitfalls.

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
  Papyrus native used exists in all editions. On flat SE/AE the pointed test spell is
  actually SIMPLER (`GetCurrentCrosshairRef` is the literal screen crosshair).
- **Per-edition requirements:** SE/AE = SKSE64 + Address Library for SKSE Plugins;
  VR = SKSEVR + VR Address Library. Plus PapyrusUtil for the matching edition.

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

2. **You own the session's end.** A walker HOLDS their slot (engine-pinned, zero
   drift) until you `Release()` them, retask them with a new `GoTo`, or your
   `afHoldSeconds` expires. A held NPC does nothing else — that is the feature. If your
   NPC "stands there frozen," you forgot to release or set a hold policy.

3. **Register events on YOUR OWN form.** SKSE ModEvent registration is per
   (form, event) — registering on a shared form silently steals or loses callbacks.

4. **One walker = one session.** Re-calling `GoTo` for a busy walker retasks them
   (their slot is reused). 4 concurrent walkers max; the 5th `GoTo` returns 0.

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
                  Float afHeadingDeg, Float afHoldSeconds = -1.0)
    ; Raw point + facing. NO VALIDATION — see Golden Rule 1.

Bool    Release(Actor akWalker)   ; end session: package off, pose reset, slot freed
Bool    IsBusy(Actor akWalker)    ; walking or holding?
```

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
    ; 1 = no valid slot   2 = travel timeout (90 s)   3 = released before arrival
    ; 4 = danger on the route (dropped)   5 = leash (too far from the player)
    ; 6 = (v0.9.2) she stopped moving 100 u or more from the point: not an arrival, released
    ; 7 = (v0.9.9, a C++ close move request to an NPC only) NO WAY: she could not get to the asked
    ;     point - blocked, and no other way worked; released
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
C++ move requests: never call them.

---

## C++ API (SKSE plugins)

Copy `API/StandpointAPI.h` into your plugin (self-contained, `<cstdint>` only — your
CommonLib version never has to match). After `kDataLoaded`:

```cpp
StandpointAPI::StandpointMessage msg;
SKSE::GetMessagingInterface()->Dispatch(
    StandpointAPI::StandpointMessage::kGetInterface, &msg, sizeof(msg), "Standpoint");
if (msg.GetApiFunction) {
    g_move  = static_cast<StandpointAPI::IStandpoint2*>(msg.GetApiFunction(2));  // 0.8.0+: move + solve
    g_solve = static_cast<StandpointAPI::IStandpoint1*>(msg.GetApiFunction(1));  // any version: solve only
}
```

- `msg.GetApiFunction == nullptr` → Standpoint is not installed.
- `GetApiFunction(2) == nullptr` → a Standpoint older than 0.8.0: no move request; the
  solver (revision 1) still answers.
- `IStandpoint2` extends `IStandpoint1`, so a revision-2 pointer has the solver too.
- **MAIN THREAD ONLY**, both revisions. From any other thread, go through
  `SKSE::GetTaskInterface()->AddTask`.

### The move request (revision 2, v0.8.0+) — one 10-slot line in, one 10-slot line back

*"Walk this NPC to that target, this close, on this side, looking this way, hold this pose
this long — and tell me how it ended."* You hand in a request line. Standpoint does the
walk with the same engine as `GoTo`. One result line comes back through your callback when
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
| 8 | `close` | `0` normal · `1` **close contact**, for an NPC or the player as target (v0.9.0; an older Standpoint ignores it): no distance floor (the point is exactly as far as asked); a walker who has stood still for 3 s within reach of her target has arrived (`C`, detail 9, the real distance in the result) instead of waiting out the 90 s; the player standing near does not cancel the walk (grabbing her still does). **For an NPC target (not the player), close mode also includes: the pair rule** (v0.9.5; VR with HIGGS) - for as long as the request runs, the walker's and the target's character controllers ignore each other, so her body can overlap the target's instead of meeting it at about 56–65 cm. Only this one pair is affected; walls, furniture and every other actor collide as always. The rule goes off by itself after the request has ended, once the two are 90 u (about 1.3 m) apart. **She walks every step** (v0.9.8) - 0.9.5–0.9.7 moved her over the last stretch with a translate ("the glide"); that slid her across the floor in her wait pose, and it is gone. Measured instead (2026-10-10): within about 50 u of her marker the engine does not walk an NPC, it poses her where she stands; a walk that starts 58 u (85 cm) or more from the point goes all the way, also into the target's space (28 cm and 44 cm centre to centre, on foot). So a walker who stands nearer than that to the asked point first **steps back** to a stop about 1.35 m from the target on the asked side and walks in from there; and a walker on **another side** of the target steps back from it, goes around it at about 1.35 m and comes in on the asked side (v0.9.7). All of it inside the one request - one answer, at the end; it takes some seconds longer. So ask for the new side or the new distance directly; do not walk her out first. If she still ends more than 12 u (17 cm) from the point she walks in once more from further out; after that - or at once, within 12 u - she is taken where she stands: `C`, detail 9, the measured distance and hour in the result. About **30 cm** is the practical limit on foot. **Blocked** (v0.9.9): when the engine itself reports "movement blocked" and she has not moved for a second, Standpoint reacts at once - a blocked step back goes around instead, a blocked walk in is tried once more from further out; and if she cannot get to the point by any way it tries, the answer is `O`, detail 17, and she is released. For the player as target: no pair rule, no stops, no second walk in. **Hold the target still first** (a `where = who` line for her). |
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
| `C` | **Completed** — she is at the target. | `0` on the point · `9` stopped short and could not get closer (bodies touching, something in the way); she was turned where she stood — read `distanceCm`. For a close request to an NPC (v0.9.9) detail 9 always means NEAR: on the asked side, within about 50 cm of the point · `16` (0.9.5–0.9.7 only: on the point, the last stretch by a glide - not produced since 0.9.8, nothing slides her) |
| `P` | The **player** stopped her. | `7` bumped or body-blocked · `8` grabbed (VR) |
| `O` | **Obstacle** — she could not get there. | `1` no standable spot at the target · `2` not reached in 90 s · `10` bad request (`who` is not a loaded NPC, or `where` is not a reference) · `15` she stopped moving far from where she was sent (100 u or more): released · `17` (v0.9.9, a close request to an NPC) **no way** to the point: blocked, and no other way worked — released |
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
   itself, with no second report. Poses 2 and 3 play after the report.
3. **A blocked side is swapped, not failed.** If the asked hour has no room (the navmesh
   snap has to slide the point more than 16 u — a wall, a table), the next hour is tried,
   alternating both ways (11, 1, 10, 2 …). Slot 4 of the result says where she really went.
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
   cancels it (`L`), 90 s is the time limit (`O`). Standpoint sells accuracy, not guaranteed
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

**Measured in VR** (v0.8.1, 2026-10-09 — one NPC walking around another on open floor):
asked 150 cm at hours 12 / 3 / 6 / 9 → 150 / 151 / 153 / 151 cm at hours 12.0 / 3.0 / 6.0 /
9.0; orientation asked 12 / 12 / 6 / 3 → 0.0 / 12.0 / 6.0 / 3.0 (12.0 and 0.0 are the same
direction). Pose 0 holds her on the spot the same way pose 1 does. **Not yet run in VR:**
furniture and objects as targets, the blocked-hour swap, a marker with no model, and the
outcomes `P`, `O`, `D`, `L`, `B`.

**Known gaps**

- A release line is refused (`O`, detail 10) while the NPC's 3D is not loaded. The leash
  frees her anyway once the player is 50 m away.
- The target must be in the loaded area: the navmesh under it is read.
- Poses 2 and 3 are one-shots; she then stands in the standard wait for the rest of the hold.
- There is no Papyrus version of the 10-slot line. Papyrus mods use `GoTo` / `GoToPoint`.

**Try it without writing a plugin — the `stpt` console test.** Click an NPC in the console
and type `stpt`: she is the walker. Click any target and type `stpt` again: she is sent
around it in six steps through the same request line a plugin would send (another NPC as
the target runs the close-contact program instead: the target is held in place, then 16
steps, about 4 minutes). Every request
and result line is printed in the console and written to `SKSE/Standpoint.log`
(`MOVE #n REQUEST` / `MOVE #n RESULT`). `stpt` while it runs stops it; clicking the walker
and typing `stpt` (no test running) clears her so you can pick another.

### The solver (revision 1, any version)

`IStandpoint1` (also on a revision-2 pointer): `GetBuildNumber()` (slot 0, forever; `2`
since 0.8.0), `SolveSlot(targetFormID, relation, agentFormID, standoff, SlotResult&)`,
`GetFurnitureMarkerCount/Info`. **Pure — it never moves anyone.** Use it when your plugin
drives its own movement and only needs the slot. Fixed 56-byte POD, append-only ABI, MAIN
THREAD ONLY, cheap (call on demand, never per-frame).

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
re-anchoring faster than that fights the engine.

### 5. THE WATER PATTERN — composing with Water Finder (the canonical composition)

The division of labour, settled the hard way: **Water Finder finds, Standpoint walks,
YOUR MOD orchestrates.** Neither framework calls the other.

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

- **v0.9.2 - no stuck recovery, and "far away" is never "arrived".** Until 0.9.1 every walk ran
  under SeverActions' stuck monitor. Measured 2026-10-09: a walker who stood still 40.6 u from
  her point for 7.5 s was "leapfrogged" 500 u straight ahead — through her target and a wall,
  into the void under the cell — and Standpoint then reported her as arrived where she fell.
  Now: no Standpoint walk starts that monitor, and a walker who has stopped 100 u or more
  from her point (or 100 u above / below it) is released with `Standpoint_Failed` code 6
  instead of `Standpoint_Arrived`. If your mod treated every `Arrived` as "she is there",
  nothing changes; if it relied on a wedged walker being freed by a teleport, it now gets
  code 2 after 90 s and must decide for itself.

- **v0.8.1 - a walker ON her point but still facing wrong is turned before she is reported.**
  When she is at the point but more than 5° off after the settle waits, the marker is moved
  to where she stands and, if the engine still does not turn her, she is turned directly
  (the log says `ON the point but N deg off`). `Standpoint_Arrived` then comes up to 3 s
  later than before and carries the corrected `afAngErr`. Applies to every session: `GoTo`,
  `GoToPoint`, the C++ move request.

- **v0.7.2 - an NPC target's side can be swapped.** When the requested side's point is not
  standable (a table or a wall right there: the navmesh snap slid it more than 16 u), the
  solver takes the next side instead (the log names it: `Solve: ok … at=N deg`). Read the
  returned point if the side matters to you; the arrival facing is always toward the target.

- **`GoToPoint` = no validation.** Off-navmesh input → the NPC walks somewhere you did
  not intend, possibly through a load door. Solve/snap your own raw points.
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
  Scripts/ + Scripts/Source/  StandpointAPI (public), Standpoint (natives), driver, test effects
  API/StandpointAPI.h       the C++ contract: the move request + the solver (copy into your plugin)
  MODDER_API.md             this document
```

Source: the `src/` (C++) and `papyrus/` folders of the Standpoint repository; how to build
them is in its `BUILD.md`, the version history in `CHANGELOG.md`.

**Versioning:** `GetVersion()`/`GetPluginVersion()` return semver. The Papyrus surface
is append-only (new optional trailing params only). The C++ vtable is append-only,
slot 0 fixed forever; revision 2 (0.8.0) only added `RequestMove` after revision 1's
functions. `MoveRequest` and `MoveResult` are fixed 40-byte lines — slots 8–10 are the room
for growth. Test spells and the `stpt` console test may change freely — never build on them.
