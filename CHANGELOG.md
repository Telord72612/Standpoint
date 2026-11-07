# Changelog

Every entry was measured in Skyrim VR unless it says otherwise.

## 2.0.0 — "V2.0": Standpoint walks her itself
The code of 1.6.1 with the version number changed. Summary of 1.4.0 → 1.6.1:
- **Steered walking (VR).** Every request (`GoTo`, `GoToPoint`, `RequestMove`, the new `WalkTo` / `WalkToWater`) is
  walked by Standpoint itself: the spot is checked with physics rays, the route is planned on the navmesh's
  triangles and tested with rays at her body size, and a steering hook (planner direct control, as HIGGS / PLANCK
  use) moves her while the game plays her real steps. The API, the answers and the events are unchanged.
  `[Session] Steered = 0` in `Standpoint.ini` goes back to the game's own walk. SE / AE keep the game's walk by
  default (`[DirectWalk] SEAE = 0`).
- **Long walks** up to the loaded area's edge, planned in pieces while she moves; run beyond 15 m, jog to 3 m,
  walk the last 3 m. A point outside the loaded area answers `O` `26` at once.
- **Doors, stairs, water.** Ordinary doors are opened (a closing door is waited for); stairs both ways; walks off
  the navmesh into water. `WalkToWater` goes shin → knee → thigh deep within two body lengths, in one walk.
- **Fallback.** No route at all → one try with the game's own walk (to the nearest navmesh point for a spot off the
  navmesh), never for close contact. A stuck fallback ends within seconds (`Failed` 7).
- **Fixes.** The walk markers now move between cells (before, a marker made indoors sent NPCs back into the house);
  markers are kept across loads; a sitting NPC is stood up by the game's walk; every ending sends one event.
- **Measured (VR, 2026-11-06):** arrivals median 3.5-4.5 u, facing ≤ 2°; close contact 45 cm → 43.9 cm; planning
  median 0.1 s (was 4-10 s on hills); wading 30-41 % of her height; told walk / jog / run = 75 / 180 / 285 u/s.
- New test commands `stpf` (follow me) and `stpm` (32-step side-by-side test).

## 1.4.0 – 1.6.1
The steps toward 2.0.0 (the walk off the navmesh, `WalkTo`, `WalkToWater`, the mixed planner, doors, the speed
bands, the navmesh skeleton, steered sessions). Not published separately; see 2.0.0.

## 1.3.0 — a way round
- **A planned way round a blocked leg.** When something the navmesh does not know about stands in her way
  (furniture, a house mod's walls) and the engine reports her blocked — or she stops after walking — with 60 u or
  more still to go, Standpoint now plans a way round before giving up: a small grid search on the navmesh, with
  Havok rays at knee and chest height checking that her whole body fits. She then walks the way round point by
  point (each point a normal leg with its arrival check) and carries on with her walk. At most two per walk. The
  log shows each plan as a `PATH #n` line with its cost.
- **Cost:** a plan runs in slices of at most 96 physics questions per frame on the main thread, so it never
  spikes one frame; a typical plan in a room is a few hundred questions spread over a few frames. The straight
  line is checked first (about 5 questions) and nothing more is done when it is clear.
- **A leg that does not start is kicked.** If she has not moved 2 s after a leg was sent her package is evaluated
  again; at 4 s the same leg is sent once more. Before, such a leg counted as failed after 8 s — and two failed
  legs answered "no way".
- New natives `Standpoint.PathStart / PathResult / PathCancel` (the planner; it never moves anyone).
- Not yet run in game.

## 1.2.0 — the BiSR test fixes
Found by the Bathing in Skyrim bridge in a VR test in Breezehome (a raised bath behind a rock wall).
- **Fixed: a point on another floor level.** The clock solve (`SolveClock`, and `GoTo` around an NPC) could
  put the walker's point on the floor *below* her target — a woman standing in a raised bath got a washer sent to
  the foot of the wall, 60 u under her, who then stood there. A point must now stand on the target's level: within
  40 u, or a 30° slope over the distance, whichever is more. Otherwise the next hour is tried, and if none works
  the answer is "no point" (`[]`) instead of a wrong one. A line `clock other level` in the log names each one.
- **Fixed: a re-sent walker who stops short waited 90 s.** Within about 50 u of her new point the engine poses
  her instead of walking, and Standpoint never noticed — she stood still until the 90 s limit. Now that walk ends
  within seconds (arrived short, turned to her face target). Plain `GoTo` / `GoToPoint` walks also get the
  "standing still near the point = the walk has ended" rule that close mode had.
- **New: `StandpointAPI.RequestMove`** — the move request line from Papyrus (the same 8 values as the C++ line),
  so close mode is open to Papyrus mods. Answered once by the new event `Standpoint_MoveResult`, or by asking
  `GetMoveOutcome` / `GetMoveResult`. Nothing existing changed shape.
- **Faster arrivals.** The arrival waits (3 s, 4 s, 2 s, 2.5 s) now end as soon as she stands still facing the
  right way: about 2-3 s instead of about 8 s for a walker who has already stopped.
- A paused game (a menu, the VR dashboard) no longer counts toward the 90 s limit.
- A walker who is walking past the player is no longer taken as "blocked by the player".
- Her position is now in the log on every session and verdict line.
- No longer listens to SeverActions' travel event (it only logged, and the argument count did not match).
- **Close mode without HIGGS (SE, AE).** The pair rule — two NPCs' bodies ignore each other so they can stand
  body to body — used HIGGS's collision hook, so on SE / AE close requests stopped at about 56-65 cm. Standpoint
  now hooks the game's collision filter itself when HIGGS is not there. (Test switch for VR players: an empty file
  `SKSE/Plugins/Standpoint_OwnPairFilter.test` makes it use its own hook even with HIGGS installed.)
- **The four developer test spells are gone** from the player's magic menu (a player who has them loses them at
  the next load). The console test `stpt` stays.
- Not yet run in game.

## 1.1.0 — works on Skyrim AE 1.7 ("V1.1")
- **Fixed:** on Skyrim AE 1.7.x the game stopped at start with an Address Library error
  ("failed to open address library file", `REL/ID.h` line 223). Standpoint was built on an older CommonLib
  that only knew AE as version 1.6, so it took 1.7 for the old Special Edition and looked for the wrong
  Address Library file. It is now built on CommonLibVR 8.3.0, which knows 1.7.
- The DLL now also carries the "Address Library V5" mark that SKSE64 2.3.0 asks for on AE 1.7.99 and later
  (without it SKSE refuses the plugin).
- A leftover diagnostic probe that only makes sense on Skyrim VR no longer runs on SE / AE.
- Nothing else changed in what Standpoint does.
- Runs in VR (2026-10-25); not yet run on AE by us.

## 1.0.0 — the first public release ("V1")
- The code of 0.9.9 with the version number changed. Nothing else.
- Released into the public domain.

## 0.9.9 — blocked means blocked
- When the engine reports "movement blocked" and she has not moved for a second, Standpoint reacts at
  once instead of waiting out its 3 s / 8 s stall time: a blocked step back goes around instead, a
  blocked walk in is tried once more from further out.
- A close request to an NPC that cannot be walked now answers **`O`, detail 17 (no way)** and
  releases her. `C` detail 9 now always means *near*: on the asked side and within about 50 cm.
- New Papyrus event code: `Standpoint_Failed` **7**.
- Not yet run in game.

## 0.9.8 — she walks every step
- The short translate over the last stretch ("the glide") is gone: it slid her across the floor in
  her wait pose. Result detail 16 is no longer produced.
- Every walk to the asked point now starts at least 58 units (85 cm) from it, because nearer than
  about 50 units the engine does not walk an NPC to her marker — it poses her where she stands. A
  walker who stands nearer steps back first.
- Stops on the way are real places, and the leg changes the moment she is there.
- VR run: 13 of 13 requests ended on the point, on foot; 30 cm asked gave 28–31 cm.

## 0.9.7 — step back and around
- A close request for another side of an NPC the walker already stands near is walked as a route
  inside the one request: step back, go around, come in.

## 0.9.3 – 0.9.6 — close contact
- The pair rule: for the length of a close request the walker's and the target's character
  controllers ignore each other (VR, through HIGGS).
- "Hold your place": a request whose target is the NPC herself pins her where she stands.
- A walker who is being held is let go properly before she is sent somewhere else.

## 0.9.0 – 0.9.2 — the close option, and no more rescue by teleport
- Slot 8 of the request line: `close`.
- **No walk is rescued by a teleport any more.** Earlier versions ran SeverActions' stuck recovery
  under every walk; it can throw a walker 500 units through a wall. A stuck walker now fails
  honestly: after 90 s, or at once when she has stopped far from her point (`Standpoint_Failed` 6).

## 0.8.0 – 0.8.2 — the move request for C++ plugins
- `IStandpoint2::RequestMove`: one 10-slot request line in, one measured 10-slot result line back.
- Clock hours and centimetres; any reference as a target, including a marker with no model.
- The `stpt` console test.

## 0.7.x — robustness
- A blocked side of an NPC target is skipped for the next one.
- A walker who stops short is turned where she stands; `SetFaceTarget`; `GetArrivalState`.
- The danger scan only counts loaded actors in the walker's own cell or worldspace.

## 0.5 – 0.7.0 — recognising interference
- Combat, the player bumping or body-blocking a walker, the player grabbing her in VR.
- Danger on the route drops the session; a 50 m leash keeps errands short.

## 0.4 — the slot engine
- Four concurrent walkers; slots from real furniture markers or an object's bounds; navmesh
  snapping; arrival facing within about one degree; interaction poses.
