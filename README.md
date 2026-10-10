# Standpoint

**Tell an NPC where to stand. She walks there, turns the right way, stays put — and you are told how it went.**

Standpoint is an SKSE framework for Skyrim (VR, SE and AE from one DLL) for mod authors. Your mod says
*"go stand 1 metre to the right of that chair, facing it"*; Standpoint does the walking with the game's own
travel packages (no teleporting, no sliding) and reports back what really happened.

> **Free for everyone.** This project is released into the public domain. Use it, copy it, change it, ship it
> inside your own mod, sell it — anything you like. **No permission and no credit needed.** (Details at the
> [bottom](#license).)

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
| 2 | `where` | What she walks to: a person, a chair, an object, a marker — anything you can click in the console. | the chair |
| 3 | `distanceCm` | How far from the middle of the target she stands, in centimetres. | `100` |
| 4 | `facing` | **Which side** of the target she stands on, as an hour on a clock (see below). | `3` = its right |
| 5 | `orientation` | **Which way she looks** once there, also as a clock hour. `12` = at the target. | `12` |
| 6 | `pose` | `0` just stands · `1` waits, hands behind her back · `2` kneels · `3` bends over | `1` |
| 7 | `holdSeconds` | How long she stays. `0` = leave right away · `-1` = until you release her. | `10` |
| 8 | `close` | `1` = the two may stand really close (see [Close contact](#close-contact)). Otherwise `0`. | `0` |
| 9, 10 | | Not used yet. Send `0`. | |

### The clock

Slot 4 says where around the target she stands. Picture a clock lying on the floor with the target in the
middle and **12 o'clock pointing where the target is facing**:

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

Decimals work (`1.5`), and `-1` means "whichever side she is already nearest to".

Slot 5 uses the same idea for **her own** facing: `12` = she looks at the target, `6` = she has her back to
it, `3` = turned to her right.

### One request, said out loud

> *"Lydia, go to the chair. Stand one metre from it, on its right side, looking at it. Hands behind your back.
> Stay ten seconds."*

| who | where | distanceCm | facing | orientation | pose | holdSeconds | close |
|---|---|---|---|---|---|---|---|
| Lydia | the chair | 100 | 3 | 12 | 1 | 10 | 0 |

Two special requests:

- **Let her go:** send her `who` with `where = 0`. She stops whatever Standpoint had her doing.
- **Stay where you are:** send `where` = the same NPC as `who`. She is pinned on the spot she stands on.

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
- **Want her on another side of the same person?** Just ask for it. She steps back, walks around, and comes in
  on the side you asked — by herself, inside the one request.
- **Blocked?** She notices within a second or two and tries another way. If there is none, you get `O` `17`.

Three habits that make it work:

1. **Keep the target still first** — send the target a "stay where you are" request. A free NPC wanders off,
   and a request is worked out for where the target stands at that moment.
2. **Ask for what you want in one go.** A new side or a new distance is one request. Do not walk her out
   yourself first.
3. **Read the answer.** `C` `0` = on the spot. `C` `9` = close by, a little short. `O` `17` = could not get there.

Rough timings: walking in from a metre and a half takes about 6 seconds including settling; moving to another
side of the same person 13 to 28 seconds.

---

## Try it in the game, without writing anything

1. Open the console, click an NPC, type `stpt`. She is now the walker.
2. Click a target, type `stpt` again, close the console.

She is sent through a short program using the very same request line a plugin would send. Every request and
every answer is shown in the console and written to `SKSE/Standpoint.log`.

- Target is furniture, an object or a marker: six steps around it.
- Target is another NPC: the close-contact program, 16 steps, about four minutes. Stand a few metres back.

Type `stpt` again to stop and release everyone.

---

## What you can count on

| | Measured in Skyrim VR |
|---|---|
| Facing on arrival | within about **1°** (typically 0.1°) |
| Position on arrival | within a few centimetres on open floor |
| Drift while she is held | **none** — the engine pins her |
| Two NPCs close together | down to about **30 cm**, on foot |
| NPCs moving at the same time | **4** |

## Install

Install it like any other mod (Mod Organizer 2 or Vortex): the contents of the [`data`](data) folder are the
mod. It adds a light plugin (`Standpoint.esp`, ESL-flagged, 15 records), `SKSE/Plugins/Standpoint.dll`, its
scripts, and the header and guide for modders.

**Needs:** SKSE (SKSEVR for VR, SKSE64 for SE / AE), the matching Address Library, and PapyrusUtil.
**Optional:** HIGGS in VR (lets Standpoint notice the player grabbing an NPC, and lets two NPCs stand inside
each other's space); Water Finder (only for the "send her to the water" pattern in the guide).

One DLL serves VR, SE and AE. **Everything here was measured in Skyrim VR.** SE and AE use the same code and the
same core game AI, but have not been measured — check one arrival before you build on it there.

## What it does not do

- **It does not decide anything.** It moves an NPC to a spot. Which bed, which key, and what happens when she
  gets there is your mod's job.
- **It does not make her use furniture.** It parks her *at* it. Give her a sit or sleep package afterwards.
- **It never rescues a stuck NPC by teleporting her.** She fails honestly (`O`) and you decide.
- **It is not a path-finder.** The game's navmesh does the walking.
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

A solver turns "the left side of that bed" or "3 o'clock, 100 cm" into an exact point on the navmesh and a
heading. A session engine puts an invisible idle marker there and gives the NPC a travel package that ends
on that marker, so the game's own arrive-and-pose machinery does the precise part. A poll watches for arrival,
interruption, danger and blockage, and names every ending. For C++ callers the DLL is a mailbox: it hands the
request to that engine and measures the result.

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
