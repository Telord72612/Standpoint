# Standpoint

**Tell an NPC where to stand. She walks there, turns the right way, stays put — and you are told how it went.**

Standpoint is an SKSE framework for Skyrim (VR, SE and AE from one DLL) for mod authors. Your mod says
*"go stand 1 metre to the right of that chair, facing it"* or *"walk into that lake until the water is at her
thighs"*; Standpoint finds the spot, plans the way there, walks her (no teleporting, no sliding) and reports back
what really happened.

**Version 2.0** — Standpoint now plans its own path and steers the NPC itself in Skyrim VR: around tables, through
doors, up and down stairs, off the navmesh into water, and up to the edge of the loaded area (100+ m) with
run, jog and walk. See [What's new in V2.0](#whats-new-in-v20). One DLL for Skyrim VR, SE (1.5.97) and AE (1.6
and 1.7).

> **Free for everyone.** This project is released into the public domain. Use it, copy it, change it, ship it
> inside your own mod, sell it — anything you like. **No permission and no credit needed.** (Details at the
> [bottom](#license).)

---

## What's new in V2.0

Version 1 asked the game's own AI to walk the NPC to the spot. That works on open floor, but the game's AI gets
stuck turning in place next to tables, takes strange detours (even through a house to reach you outside), and
never walks off the navmesh. **In V2.0 (Skyrim VR) Standpoint walks her itself.** Nothing changes in how you
call it: the same requests, the same answers, the same events.

**How a walk works now**

1. **Find the spot.** For "the left side of that chair, 1 m away" the solver works out the exact spot and checks
   it with physics rays: real floor under her feet, room for her body and head, not inside a table, not on a
   stair, not on another level. Bad spots are refused with a reason; the next side is tried.
2. **Plan the way.** The navmesh is used as a guide (a "skeleton" of the route), and every piece of the route is
   tested with rays at her body size before she takes it. Long routes are planned in pieces while she is already
   moving, so she starts within about half a second and never stops between pieces.
3. **Walk her there.** Standpoint steers her the way a hand pulling her would (the same mechanism HIGGS / PLANCK
   use): the game still plays her real walking, jogging and running steps. She opens ordinary doors on the way,
   takes stairs, goes around NPCs standing in the way, and turns to face the asked direction at the end.

**What you get**

| | V2.0 |
|---|---|
| Sides of an NPC or an object | one planned path, no step-back-and-around, no turning in place |
| Close contact (two NPCs) | 45 cm asked → about 44 cm measured |
| Arrivals | typically within 3-5 cm of the spot, facing within 1-2° |
| Long walks | up to the loaded area's edge (about 110 m with default settings); beyond it you get an answer at once (`O` `26`) |
| Speed | runs while more than 15 m from the spot, jogs down to 3 m, walks the last 3 m (a band shorter than 1 s is skipped) |
| Doors | ordinary doors are opened (locked, load and trapped doors are walls); a door swinging shut is waited for |
| Stairs | walked up and down; she never stops on a stair |
| Water | `WalkToWater`: she walks into the nearest water and goes in until shin, then knee, then thigh deep if there is such a spot within two body lengths — all in one smooth walk |
| No way at all | one try with the game's own walk (never for close contact), so a spot behind a load door still works |

**New calls** (Papyrus; the C++ interface has the same):

```papyrus
; walk to an exact point, facing a direction (degrees); -1 = keep the travel direction
Int h = StandpointAPI.WalkTo(npc, x, y, z, 90.0)

; walk into the nearest water within 20 m, shin to thigh deep
Int w = StandpointAPI.WalkToWater(npc, 20.0)

; both answer once, with this event (or poll GetWalkOutcome(handle))
RegisterForModEvent("Standpoint_WalkResult", "OnWalkResult")
Event OnWalkResult(Form akWho, Int aiHandle, String asOutcome, Int aiDetail, Float afDistLeft, Float afSubmerged)
EndEvent
```

**On SE / AE** the new walking is switched off by default (`Standpoint.ini`: `[DirectWalk] SEAE`), and every
request uses the game's own walk exactly as in version 1. **On VR** you can switch the new walking off for
`GoTo` / `GoToPoint` / `RequestMove` with `[Session] Steered = 0`.

---

## The whole API in one picture

You send **one line of 10 values**. You get **one line of 10 values** back.

```
YOU SEND   [ who | where | how close | which side | looking where | pose | how long | close? |  -  |  -  ]

YOU GET    [ who | where | how close | which side | looking where | RESULT | reason |  -  |  -  |  -  ]
                           \_______ measured, not repeated _______/     \__ one letter __/
```

That is all there is to it. The rest of this page explains the ten values and the result letter.

---

## The request: what you send

| # | Name | In plain words | Example |
|---|---|---|---|
| 1 | `who` | The NPC who should move. | Lydia |
| 2 | `where` | What she walks to. | the chair |
| 3 | `distanceCm` | How far from the middle of the target she stands, in centimetres. | `100` |
| 4 | `facing` | **Which side** of the target she stands on, as an hour on a clock. | `3` = its right |
| 5 | `orientation` | **Which way she looks** once there, also as a clock hour. | `12` = at the target |
| 6 | `pose` | What she does with her body once there. | `1` = waits |
| 7 | `holdSeconds` | How long she stays. | `10` |
| 8 | `close` | Normal distance rules (`0`) or close contact (`1`). | `0` |
| 9, 10 | | Not used yet. Send `0`. | |

### One request, said out loud

> *"Lydia, go to the chair. Stand one metre from it, on its right side, looking at it. Hands behind your back.
> Stay ten seconds."*

| who | where | distanceCm | facing | orientation | pose | holdSeconds | close |
|---|---|---|---|---|---|---|---|
| Lydia | the chair | 100 | 3 | 12 | 1 | 10 | 0 |

---

## Every value, and every choice it has

### 1 · `who` — the NPC who moves

You give her **RefID**: the ID of that one person standing in the world. In a plugin it is `actor->GetFormID()`.

| You can send | Example | |
|---|---|---|
| Any NPC who is loaded near the player | Lydia → `000A2C94` | A follower, a townsperson, a guard — anyone. |
| An NPC added by a mod | `DB000243` | The first two digits are the mod's place in the load order, so read the ID while the game runs; never type it into your code. |

| You cannot send | What you get back |
|---|---|
| The player | `O` `10` (bad request) — Standpoint only moves NPCs |
| A **BaseID** (Lydia's is `000A2C8E`) | `O` `10` — a BaseID is the *recipe* for a person, not the person |
| An NPC who is far away and not loaded | `O` `10` |

> **RefID or BaseID?** Click something in the console: the number at the top of the screen is its **RefID** —
> that is the one Standpoint wants. The IDs listed by the console's `help` command are BaseIDs.

### 2 · `where` — what she walks to

Also a **RefID**. Anything you can click in the console works.

| You can send | Example | What "its front" means (for slot 4) |
|---|---|---|
| The player | `00000014` | The way the player's body faces. |
| Another NPC | her RefID | The way she faces. |
| Furniture | that one chair, bed or bench | The way the object itself points in the world. |
| An object | a cup on a table, a chest, a lever, a door | The way the object itself points. |
| A marker | an X-marker or heading marker, even an invisible one you placed yourself | The way the marker points. |
| **`0`** | | *"Let her go."* She stops whatever Standpoint had her doing. |
| **The same ID as `who`** | | *"Stay where you are."* She is pinned on the spot she stands on, facing the way she faces. |

| You cannot send | What you get back |
|---|---|
| A BaseID (*"any chair"*) | `O` `10` — there may be fifty chairs made from one BaseID; Standpoint has to know which one |
| Something that is not in the world (an item inside a container or an inventory) | `O` `10` |

The target has to be in the area that is loaded around the player.

### 3 · `distanceCm` — how far from the target

Measured from the **middle of the target** to where she stands, in centimetres.

| You send | Good for | |
|---|---|---|
| `150` | Talking distance. | |
| `100` | About an arm's length. | |
| `63` or less, target is a person, `close = 0` | | She stops at about 63 cm: people keep a body's width apart. Set `close = 1` to go nearer. |
| `45`, `close = 1` | Touching distance. | |
| `30`, `close = 1` | As near as two people can be walked. | Nearer than about 30 cm they would be inside each other. |
| Any distance, target is furniture or an object | | She never stands nearer than the object's own size plus her own body. |
| `0`, target is a marker | *"Stand exactly on it."* | She then faces the way the marker points (slot 5 turns her from there). |

If she cannot stand as near as you asked, she is placed as near as allowed, and the answer tells you the real
distance.

### 4 · `facing` — which side of the target

A clock hour. Picture a clock lying on the floor with the target in the middle and **12 o'clock pointing where
the target is facing**:

```
                    12   in front of the target
                11      1
             10            2

            9     TARGET     3   on its right
                 (faces 12)
              8            4
                 7      5
                     6   behind it
```

| You send | She stands |
|---|---|
| `12` (or `0`) | in front of the target |
| `3` | on its right |
| `6` | behind it |
| `9` | on its left |
| `1.5`, `4`, `10.5` … | anywhere in between — any number from 0 to 12 |
| `-1` | on whichever side she is already nearest to |

If that side has no room (a wall, a table), she is put on the nearest side that does, and the answer tells you
which hour she really got.

### 5 · `orientation` — which way she looks

Another clock hour, this time about **her**: where is she turned, compared with looking straight at the target?

| You send | She |
|---|---|
| `12` (or `0`) | looks at the target |
| `3` | is turned to her right — the target is at her left shoulder |
| `6` | has her back to the target |
| `9` | is turned to her left — the target is at her right shoulder |
| anything in between | is turned by that much |

### 6 · `pose` — the four poses

| You send | Name | What you see | Use it for |
|---|---|---|---|
| `0` | **Plain stand** | She just stands there, relaxed, like any idle NPC — but she does not wander off. | Scenes where the hands must be free, or where you play your own animation on her. |
| `1` | **Standard wait** | She stands with her hands clasped behind her back. | Waiting politely. The usual choice. |
| `2` | **Kneel** | She arrives, then kneels down to the floor once (about three seconds), as if picking something up. | Something lying on the ground. |
| `3` | **Bend** | She arrives, then bends forward once, as if taking something off a table. | Something on a table, a shelf or a counter. |

Poses `2` and `3` are played **once**, right after your answer arrives — so the moment you get `C` is the moment
to do your own part (give her the item, play a sound). Afterwards she stands in the standard wait for the rest
of her stay. Her feet do not move while she kneels.

### 7 · `holdSeconds` — how long she stays

| You send | What happens |
|---|---|
| `0` | She is let go the moment she has arrived, and goes back to her own life. |
| `10` (any number) | She stays that many seconds, then is let go by herself. |
| `-1` | She stays until you let her go (a request with `where = 0`) or send her somewhere else. |

Your answer always arrives **when she gets there**, not when the stay is over. While she stays she does not
move at all — if an NPC looks frozen, this is why.

### 8 · `close` — `0` or `1`

This switches between two sets of rules for getting near a **person**.

| | `close = 0` — normal | `close = 1` — close contact |
|---|---|---|
| How near a person she can stand | About **63 cm**, however little you ask for. | **Exactly what you ask for**, down to about 30 cm. |
| The two bodies | Bump into each other, as always. | Pass into each other's space for as long as the request runs (VR with HIGGS). Walls and everyone else stay solid. |
| She is asked to another side of the same person | The game's own path-finding is left to it. | She steps back, walks around and comes in on that side. |
| She stops a little short | She is taken where she stands. | She walks in once more from further out before she is taken where she stands. |
| The player walks into her, or stands in her way | The walk is cancelled (`P`). | It carries on. (Grabbing her in VR still cancels it.) |
| The way is blocked | She waits; after 90 seconds you get `O`. | She notices in a second or two and tries another way; if there is none you get `O` `17`. |

Use **`0`** for everything at normal distance: a conversation, standing beside a bed, a guard at a door.
Use **`1`** when two NPCs must be within touching distance: handing something over, a hug, a hand on a shoulder.

For furniture and objects `close` does not bring her nearer: the object's own size is still in the way.
With the **player** as the target, `close = 1` only lifts the distance limit; stepping around and passing into
each other's space are for one NPC approaching another.

### 9 and 10

Not used yet. Send `0`.

### A few complete requests

| What you want | who | where | distanceCm | facing | orientation | pose | holdSeconds | close |
|---|---|---|---|---|---|---|---|---|
| Come and talk to the player | NPC | `00000014` | 150 | 12 | 12 | 1 | 20 | 0 |
| Stand on the left of the bed, facing it, until told | NPC | the bed | 100 | 9 | 12 | 1 | -1 | 0 |
| Stand guard in front of a door, back to it | NPC | the door | 120 | 12 | 6 | 1 | -1 | 0 |
| Pick up a coin from the floor | NPC | the coin | 60 | -1 | 12 | 2 | 5 | 0 |
| Take a cup from a table | NPC | the cup | 80 | -1 | 12 | 3 | 5 | 0 |
| Stand exactly on my marker, looking the way it points | NPC | the marker | 0 | 12 | 12 | 0 | -1 | 0 |
| Step up to another NPC for a hug | NPC | the other NPC | 30 | 12 | 12 | 0 | 15 | 1 |
| Keep that other NPC still while she comes | the other NPC | the same NPC | 0 | 0 | 0 | 0 | -1 | 0 |
| Let her go | NPC | `0` | 0 | 0 | 0 | 0 | 0 | 0 |

---

## The answer: what you get back

The answer arrives **once**, when she has arrived or when the walk was stopped — whichever comes first.

Slots 3, 4 and 5 come back **measured**: where she really stands, not what you asked for. If you asked for
100 cm on the right and she ended up at 96 cm at hour 3.1, that is what you read.

Slot 6 is one letter that tells you what happened:

| Letter | What happened | What you usually do |
|---|---|---|
| **`C`** | **Completed.** She is there. | Carry on with your scene. |
| **`O`** | **Obstacle.** She could not get there: no room to stand, it took too long, or the way was blocked. She has been let go. | Try another side, another place, or try again later. |
| **`P`** | The **player** stopped her (bumped into her, stood in her way, or grabbed her in VR). | Usually: leave her alone. |
| **`D`** | **Danger.** Enemies on the way. The walk was dropped. | Try again when it is safe. |
| **`L`** | **Leash.** She got more than 50 m away from the player. | Try again when she is back. |
| **`X`** | Released, or replaced by a newer request for the same NPC. | Nothing — you (or someone) asked for it. |
| **`B`** | **Busy.** Standpoint moves at most 4 NPCs at a time. | Try again in a moment. |

Slot 7 is a number that gives the exact reason, for when you want to know more. The ones worth knowing:

| Letter + number | Meaning |
|---|---|
| `C` `0` | On the spot. |
| `C` `9` | Close by, but a little short (something was in the way). Read the measured distance. |
| `O` `1` | Nowhere to stand at that target. |
| `O` `2` | Not there after 90 seconds. |
| `O` `17` | No way to get there — blocked, and going another way did not work either. |

The complete list is in the header and in the [in-depth guide](docs/MODDER_API.md).

> **One rule to remember:** Standpoint promises *accuracy*, not *arrival*. When she gets there, she is exactly
> where you asked. When she cannot, you are told why — and your mod decides what happens next.

---

## Use it from a C++ SKSE plugin

Copy one file into your project: [`data/API/StandpointAPI.h`](data/API/StandpointAPI.h). It needs nothing but
`<cstdint>`.

```cpp
#include "StandpointAPI.h"

StandpointAPI::IStandpoint2* g_standpoint = nullptr;

// 1) Say hello once, after the game's data has loaded (SKSE kDataLoaded).
void ConnectToStandpoint()
{
    StandpointAPI::StandpointMessage msg;
    SKSE::GetMessagingInterface()->Dispatch(
        StandpointAPI::StandpointMessage::kGetInterface, &msg, sizeof(msg), "Standpoint");
    if (msg.GetApiFunction)                                   // null = Standpoint is not installed
        g_standpoint = static_cast<StandpointAPI::IStandpoint2*>(msg.GetApiFunction(2));
}

// 3) The answer lands here, once.
void OnMoveDone(std::uint32_t handle, const StandpointAPI::MoveResult& r, void* user)
{
    if (r.outcome == 'C') {
        // she is there: r.distanceCm, r.facing and r.orientation are where she really stands
    } else {
        // r.outcome says why not ('O', 'P', 'D', 'L', 'X', 'B'); r.detail gives the exact reason
    }
}

// 2) Ask.
void SendLydiaToTheChair(RE::Actor* lydia, RE::TESObjectREFR* chair)
{
    if (!g_standpoint) return;

    StandpointAPI::MoveRequest q{};          // start from all zeros
    q.who         = lydia->GetFormID();      //  1
    q.where       = chair->GetFormID();      //  2
    q.distanceCm  = 100.f;                   //  3  one metre
    q.facing      = 3.f;                     //  4  on the chair's right
    q.orientation = 12.f;                    //  5  looking at it
    q.pose        = 1.f;                     //  6  hands behind her back
    q.holdSeconds = 10.f;                    //  7
    g_standpoint->RequestMove(q, &OnMoveDone, nullptr);
}
```

Call it from the game's main thread. That is the whole integration.

## Use it from Papyrus

Papyrus mods use a few simple functions instead of the 10-value line:

```papyrus
If StandpointAPI.GetVersion() != ""                 ; "" = Standpoint is not installed
    StandpointAPI.GoTo(npc, bedRef, 4)              ; the LEFT side of the bed, facing it, stay
EndIf
...
StandpointAPI.Release(npc)                          ; let her go
```

`GoTo` takes a target and a side (`0` nearest, `1` front, `2` right, `3` back, `4` left) plus optional
distance, hold time and pose. Two events tell you how it went:

```papyrus
RegisterForModEvent("Standpoint_Arrived", "OnArrived")   ; (Form walker, Float distance, Float angleError)
RegisterForModEvent("Standpoint_Failed",  "OnFailed")    ; (Form walker, Float reasonCode)
```

To walk to an exact point, or into water, use `WalkTo` / `WalkToWater` and the `Standpoint_WalkResult` event
(see [What's new in V2.0](#whats-new-in-v20)).

Everything Papyrus can do, with copy-paste patterns, is in the [in-depth guide](docs/MODDER_API.md).

---

## Close contact

Set slot 8 (`close`) to `1` when two NPCs have to stand really close — handing something over, a hug, a hand
on a shoulder.

- **She stands exactly as far as you ask.** No personal-space bubble. About **30 cm** is as near as two people
  can be walked; nearer than that they would be inside each other.
- **She may step into the other person's space.** For the length of the request the two do not collide with
  each other (VR with HIGGS). Walls, furniture and everybody else stay solid.
- **She walks every step.** Nothing slides or teleports her.
- **Want her on another side of the same person?** Just ask for it. She walks one planned path around the person
  and comes in on the side you asked — by herself, inside the one request.
- **Blocked?** She notices within a second or two and tries another way. If there is none, you get `O` `17`.

Three habits that make it work:

1. **Keep the target still first** — send the target a "stay where you are" request. A free NPC wanders off,
   and a request is worked out for where the target stands at that moment.
2. **Ask for what you want in one go.** A new side or a new distance is one request. Do not walk her out
   yourself first.
3. **Read the answer.** `C` `0` = on the spot. `C` `9` = close by, a little short. `O` `17` = could not get there.

Rough timings (V2.0, VR): walking in from a metre and a half takes about 2-4 seconds including settling; moving
to another side of the same person about 5-10 seconds.

---

## Try it in the game, without writing anything

1. Open the console, click an NPC, type `stpt`. She is now the walker.
2. Click a target, type `stpt` again, close the console.

She is sent through a short program using the very same request line a plugin would send. Every request and
every answer is shown in the console and written to `SKSE/Standpoint.log`.

- Target is furniture, an object or a marker: six steps around it.
- Target is another NPC: the close-contact program, 16 steps, about four minutes. Stand a few metres back.

Type `stpt` again to stop and release everyone.

Two more test commands (VR):

- **`stpf`** — click an NPC, type `stpf`: she keeps coming to 3 m in front of you whenever you stop and stand
  still. Walk far away and she runs, jogs and walks back to you. `stpf` again stops it.
- **`stpm`** — click an NPC, `stpm`; click a target (a chair or another NPC), `stpm`: 32 steps that send her to
  the same spot with every call (`GoToPoint`, `WalkTo`, `RequestMove`), around the target with `GoTo` and
  `RequestMove`, hand her from one call to another, and into water. Every step and answer goes to the log.
- Crouched `stpt` on the walker: `WalkToWater`.

---

## What you can count on

| | Measured in Skyrim VR |
|---|---|
| Facing on arrival | within about **1°** (typically 0.0-0.4°) |
| Position on arrival | within **3-5 cm** (median, V2.0) |
| Drift while she is held | **none** |
| Two NPCs close together | down to about **30 cm**, on foot (45 cm asked → 44 cm) |
| Planning a route | median **0.1 s**, worst seen 0.7 s; never more than ~1 ms of a frame |
| Speeds (told → measured) | walk ~75, jog ~180, run ~285 game units / s |
| Wading | shin → thigh: 30-41 % of her height under water |
| NPCs moving at the same time | **4** sessions plus the plain walks |

## Install

Install it like any other mod (Mod Organizer 2 or Vortex): the contents of the [`data`](data) folder are the
mod. It adds a light plugin (`Standpoint.esp`, ESL-flagged, 15 records), `SKSE/Plugins/Standpoint.dll`, its
scripts, and the header and guide for modders.

**Needs:** SKSE (SKSEVR for VR, SKSE64 for SE / AE), the matching Address Library, and PapyrusUtil.
**Optional:** HIGGS in VR (lets Standpoint notice the player grabbing an NPC, and lets two NPCs stand inside
each other's space); Water Finder 1.6.2 or newer (for `WalkToWater`).

One DLL serves VR, SE and AE. **Everything here was measured in Skyrim VR.** SE and AE use the same code and the
same core game AI, but have not been measured — check one arrival before you build on it there.

## What it does not do

- **It does not decide anything.** It moves an NPC to a spot. Which bed, which key, and what happens when she
  gets there is your mod's job.
- **It does not make her use furniture.** It parks her *at* it. Give her a sit or sleep package afterwards.
- **It never rescues a stuck NPC by teleporting her.** She fails honestly (`O`) and you decide.
- **It does not cross load doors itself.** Its own path stays in the loaded world around the NPC; for a spot
  in another cell the game's own walk takes one try.
- **Standing within arm's reach of an object** — 5 cm from a table edge to pick something up — is planned, not
  built. Today an object's own size plus her body is the nearest she gets.

## More

| | |
|---|---|
| [`docs/MODDER_API.md`](docs/MODDER_API.md) | **The in-depth guide.** Every function, event and code; the golden rules; copy-paste patterns; every pitfall we hit. |
| [`data/API/StandpointAPI.h`](data/API/StandpointAPI.h) | The C++ contract, with the rules written next to the code. |
| [`BUILD.md`](BUILD.md) | Building the DLL, the scripts and the plugin from source. |
| [`CHANGELOG.md`](CHANGELOG.md) | What changed, version by version. |

## How it works, in five lines

A solver turns "the left side of that bed" or "3 o'clock, 100 cm" into an exact spot and a heading, checked
with physics rays. A planner builds a route from the navmesh's triangles and tests every leg with rays at her
body size; long routes are added piece by piece while she walks. A steering hook (the same "planner direct
control" HIGGS / PLANCK use) moves her along it at walk, jog or run, and the game plays her real steps. A
Papyrus driver keeps the contract — busy state, holds, poses, events, danger and leash rules — and names every
ending. On SE / AE (and with `Steered = 0`) the driver hands the walk to the game's own travel package instead.

## License

**Public domain — [The Unlicense](LICENSE).** Everything in this repository that we wrote is yours to use,
copy, modify, merge into your own mod, publish and sell, for any purpose, with no conditions. **You do not have
to credit anyone or ask anyone.**

Two small exceptions, because they are not ours to give away:

- `src/HiggsInterface.h` re-declares the public plugin interface of
  [HIGGS](https://github.com/adamhynek/higgs) by adamhynek, so that Standpoint can talk to HIGGS at run time.
  HIGGS is licensed under the GNU GPL v3, and that file stays under HIGGS's terms.
- `papyrus/stubs/` holds compile-only declarations of functions that belong to other mods (PapyrusUtil,
  SeverActions) and to the base game. They are needed to compile the scripts and are never shipped.

Built on SKSE, CommonLibSSE-NG, the Address Library and PapyrusUtil.
