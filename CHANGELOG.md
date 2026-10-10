# Changelog

Every entry was measured in Skyrim VR unless it says otherwise.

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
