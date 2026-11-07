// ─────────────────────────────────────────────────────────────────────────────
//  StandpointAPI.h — the Standpoint consumer contract for SKSE plugins.
//
//  Four things a plugin can ask Standpoint for:
//
//    REVISION 4 (IStandpoint4, Standpoint 1.5.0+) — THE WALK INTO WATER: "walk
//      this NPC into the nearest water she can really reach and wade in."
//      Standpoint asks Water Finder (1.6.0+) itself. Read the REVISION 4 block.
//
//    REVISION 3 (IStandpoint3, Standpoint 1.4.0+, SKYRIM VR) — THE WALK OFF THE
//      NAVMESH: "walk this NPC to that point - over rocks, onto a platform, into
//      shallow water - with her own legs, and tell me how it ended." One 10-slot
//      WalkRequest in, one 10-slot WalkResult back. Read the REVISION 3 block.
//      (1.5.1, no ABI change: up to 200 m, speed by distance - see that block.)
//      (1.6.0, no ABI change: our own path all the way, run / jog / walk by the
//      route left, O 26 outside the loaded area; and on VR the move request's
//      sessions are walked the same way - see REVISION 2 and 3.)
//
//    REVISION 2 (IStandpoint2, Standpoint 0.8.0+) — THE MOVE REQUEST: "walk this
//      NPC to that target, this close, on this side, looking this way, hold
//      this pose this long — and tell me how it ended." One 10-slot request
//      line in, one 10-slot result line back through your callback.
//      Read the REVISION 2 block further down.
//
//    REVISION 1 (IStandpoint1, any version) — THE SOLVER: "where exactly should
//      this actor stand at that target, and which way should they face?"
//      Pure: it never moves an actor. For plugins that drive their own movement.
//
//  The Papyrus side (StandpointAPI.psc GoTo / Release, the events) and the full
//  guide with every rule and measured number: MODDER_API.md.
//
//  COPY THIS HEADER into your own plugin. Self-contained: only <cstdint>,
//  no CommonLibSSE types — your CommonLib version never has to match ours.
//
//  ── ACQUIRING THE INTERFACE ─────────────────────────────────────────────────
//      StandpointAPI::StandpointMessage msg;
//      SKSE::GetMessagingInterface()->Dispatch(
//          StandpointAPI::StandpointMessage::kGetInterface,
//          &msg, sizeof(msg), "Standpoint");
//      if (msg.GetApiFunction) {
//          g_water = static_cast<StandpointAPI::IStandpoint4*>(msg.GetApiFunction(4));
//          g_walk  = static_cast<StandpointAPI::IStandpoint3*>(msg.GetApiFunction(3));
//          g_move  = static_cast<StandpointAPI::IStandpoint2*>(msg.GetApiFunction(2));
//          g_solve = static_cast<StandpointAPI::IStandpoint1*>(msg.GetApiFunction(1));
//      }
//
//  Dispatch AFTER kDataLoaded. Null GetApiFunction = Standpoint is not installed.
//  Null from GetApiFunction(2) = a Standpoint older than 0.8.0 (revision 1 still
//  answers). IStandpoint2 extends IStandpoint1: a revision-2 pointer has the
//  solver too. Null from GetApiFunction(3) = older than 1.4.0; IStandpoint3
//  extends IStandpoint2. Null from GetApiFunction(4) = older than 1.5.0;
//  IStandpoint4 extends IStandpoint3.
//
//  ── VERSIONING CONTRACT ─────────────────────────────────────────────────────
//  * Vtable is APPEND-ONLY; no virtual destructor; slot 0 = GetBuildNumber forever.
//  * SlotResult is fixed-size POD, append-only via its reserved tail.
//  * MoveRequest / MoveResult are fixed 40-byte lines; slots 8-10 are the room
//    for growth (send 0, expect 0).
//
//  ── THREADING ───────────────────────────────────────────────────────────────
//  * MAIN THREAD ONLY (reads live 3D, navmesh, extra-data). Synchronous, cheap
//    (a solve is marker reads + a handful of navmesh queries). Call on demand,
//    never every frame.
//
//  ── SEMANTICS ───────────────────────────────────────────────────────────────
//  * relation: 0=nearest 1=front 2=right 3=back 4=left — in the TARGET'S OWN
//    facing frame (an actor's facing, a chair's forward, a table's local +Y).
//  * The slot is navmesh-snapped and, for furniture, respects the full legality
//    gate (NIF entry points AND record enable bits MINUS record-disabled points)
//    plus live occupancy (used AND reserved markers).
//  * headingDegrees is the ARRIVAL FACING (compass, 0=N, clockwise) — for the
//    park-and-face pattern, set your IdleMarker's Z angle to exactly this.
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include <cstdint>

namespace StandpointAPI {

    // Fixed-size POD result. Append new fields into `reserved` only.
    struct SlotResult {
        bool          found;             // false = no valid slot (see failCode)
        std::int32_t  failCode;          // 0 ok; 1 noTarget 2 no3D 3 noCandidates 4 snapFailed 6 occupied
        float         x, y, z;           // world stand-point, navmesh-snapped
        float         headingDegrees;    // arrival facing: compass, 0 = north, clockwise
        std::int32_t  markerIndex;       // furniture FRN marker serving it; -1 = geometric slot
        float         quality;           // 1.0 clean; penalties for side mismatch / snap distance
        std::uint8_t  reserved[24];      // future, zeroed
    };
    static_assert(sizeof(SlotResult) == 56, "SlotResult layout is part of the ABI");

    struct IStandpoint1 {
        // Slot 0, forever.
        virtual unsigned int GetBuildNumber() = 0;

        // The solver. targetFormID = any reference (furniture / object / actor).
        // agentFormID may be 0 (no agent-radius refinement). standoffUnits <= 0
        // picks the per-target default. Returns out.found.
        virtual bool SolveSlot(std::uint32_t targetFormID, std::int32_t relation,
                               std::uint32_t agentFormID, float standoffUnits,
                               SlotResult& out) = 0;

        // Furniture introspection (post-gate truth, world space).
        // Returns marker count, -1 if the ref has no loaded 3D.
        virtual std::int32_t GetFurnitureMarkerCount(std::uint32_t targetFormID) = 0;
        // info[7] = {x, y, z, headingDeg, effectiveEntryBits, animTypeBits, enabled}
        virtual bool GetFurnitureMarkerInfo(std::uint32_t targetFormID,
                                            std::int32_t markerIndex, float (&info)[7]) = 0;
    };

    // ═════════════════════════════════════════════════════════════════════════
    //  REVISION 2 (Standpoint 0.8.0) — THE MOVE REQUEST: one 10-slot line in,
    //  one 10-slot line back. Ask with GetApiFunction(2).
    //
    //      StandpointAPI::MoveRequest q{};
    //      q.who = npc->GetFormID();  q.where = target->GetFormID();
    //      q.distanceCm = 150;  q.facing = 12;  q.orientation = 12;
    //      q.pose = 1;  q.holdSeconds = 10;
    //      g_stand2->RequestMove(q, &OnMoveDone, nullptr);
    //
    //      void OnMoveDone(std::uint32_t handle, const StandpointAPI::MoveResult& r, void*)
    //      { if (r.outcome == 'C') { /* she is there */ } }
    //
    //  * MAIN THREAD ONLY for RequestMove. The callback also arrives on the main
    //    thread, in a later frame (never from inside RequestMove), exactly ONCE
    //    per request: when she arrives, or when the move is stopped, whichever
    //    comes first. It lands ~3 s after she stops walking (she is left to
    //    settle, then measured). The hold then runs out by itself.
    //  * CLOCK HOURS, seen from above, clockwise; decimals allowed.
    //      facing      = where around the target she stands, in the TARGET's own
    //                    facing: 0 or 12 = in front of it, 3 = its right,
    //                    6 = behind it, 9 = its left. < 0 = Standpoint picks the
    //                    side she is already nearest to.
    //      orientation = which way SHE looks once there: 0 or 12 = at the target,
    //                    3 = turned to her right, 6 = her back to the target.
    //  * distanceCm is centre of the target to where she stands. Floors: an NPC
    //    target - the solver places her no nearer than about 63 cm (unless
    //    close = 1), and two NPC bodies meet at about 56-65 cm centre to centre
    //    (measured; a walker stopped by the body is 'C' with detail 9); an
    //    object - its own size + her radius; a model-less marker - NONE (0 =
    //    stand exactly ON the marker, looking the way it points). The result
    //    reports what she really got.
    //  * Furniture is an object like any other: she stands at the asked hour and
    //    distance around it. (To use its sit / sleep entries, see Papyrus GoTo.)
    //  * A blocked hour is swapped for the nearest free one. Read the result.
    //  * where = 0 RELEASES the NPC now (mid-walk or mid-hold): her running
    //    request answers 'X' (unless it already answered 'C'), and so does the
    //    release line itself.
    //  * where = who (Standpoint 0.9.5+) means HOLD YOUR PLACE: she is pinned
    //    where she stands, facing the way she faces, for holdSeconds (-1 = until
    //    released). Slots 3-5 and 8 are ignored; pose 0 = a plain stand. It
    //    answers 'C' about 3 s later. Use it to keep a TARGET still while
    //    another NPC is sent to her: a request is solved against where the
    //    target stands at that moment, and a free NPC walks away.
    //  * What stops a walk is not in the line - Standpoint's defaults apply: the
    //    player intercepting her ('P'), hostiles on the route ('D'), the 50 m
    //    leash ('L'), 90 s without arriving ('O'). On anything but 'C', ask
    //    again when it suits you.
    //  * A save being loaded answers every open request 'X' (detail 12) with
    //    slots 3-5 zero: do not touch game objects from that callback.
    //  * close = 1 (slot 8, Standpoint 0.9.0+; an older Standpoint ignores it) is CLOSE CONTACT, for an NPC or
    //    the player as target: no distance floor (the point is exactly as far as asked); a walker who has
    //    stood still for 3 s within reach of her target has arrived ('C', detail 9, the real distance in the
    //    result) instead of timing out; the player standing near does not cancel the walk (grabbing her still
    //    does). (Since 0.9.2 no Standpoint walk of any kind starts SeverActions' stuck recovery.) For
    //    furniture and objects the distance floor stays (their own size + her radius); the other two rules apply
    //    to any target.
    //    For an NPC target (not the player), close mode also includes:
    //      - THE PAIR RULE (0.9.5; VR with HIGGS): for as long as the request runs, the walker's and the target's
    //        character controllers ignore each other, so her body can overlap the target's instead of meeting it
    //        at about 56-65 cm (measured: 41 cm in front, 46 cm at the side, the target not moved). Only this one
    //        pair is affected; walls, furniture and every other actor collide as always. The rule is switched off
    //        by itself after the request has ended, once the two are 90 u (about 1.3 m) apart.
    //      - SHE WALKS EVERY STEP (0.9.8). 0.9.5-0.9.7 moved her over the last stretch with a translate ("the
    //        glide"); that slid her across the floor in her wait pose and is gone. Measured instead: within about
    //        50 u of her marker the engine does not walk an NPC - it poses her where she stands; a walk that
    //        starts 58 u (85 cm) or more from the point goes all the way, also into the target's space. So:
    //          * a walker who stands nearer than that to the asked point first steps back to a stop about 1.35 m
    //            from the target on the asked side, and walks in from there;
    //          * a walker who stands on ANOTHER side of the target steps back from it, goes around it at about
    //            1.35 m and comes in on the asked side (0.9.7);
    //          * all of it inside the one request - one answer, at the end. It takes some seconds longer.
    //        If she still ends more than 12 u (17 cm) from the point she walks in once more from further out.
    //        After that - or at once, when she is within 12 u - she is taken where she stands: 'C', detail 9, the
    //        measured distance and hour in the result. About 30 cm is the practical limit on foot (measured:
    //        28 cm). Detail 16 is no longer produced.
    //      - BLOCKED (0.9.9): when the engine itself reports "movement blocked" and she has not moved for a
    //        second, Standpoint reacts at once - a blocked step back goes around instead, a blocked walk in is
    //        tried once more from further out. 'C' detail 9 now always means NEAR: on the asked side and within
    //        about 50 cm of the point. If she cannot get there by any way it tries, the answer is 'O', detail 17
    //        (kDetailNoWay), and she is released: ask again later, or for another place.
    //    For the player as target: no pair rule, no stops, no second walk in (as in 0.9.2). Without HIGGS: no
    //    pair rule - what physically stops two bodies is then unchanged, they meet at about 56-65 cm.
    //    Hold the target still yourself (a `where = who` line for her, see above) before you send someone to her.
    //  * Up to 4 NPCs at a time. A new request for an NPC who has one replaces it
    //    (the old one answers 'X').
    //  * 1.6.0 (no ABI change) - THE STEERED SESSION, Skyrim VR with Data/SKSE/Plugins/Standpoint.ini [Session] Steered = 1
    //    (the default on VR; 0 = the engine's walk as before - the default on SE / AE): the same line, the same answers, but
    //    Standpoint finds the spot itself with physics tests (her floor, the room for her body, furniture and walls, standing
    //    NPCs, water up to her thighs - the asked hour first, then the solver's own order), plans ONE route and walks her there
    //    with its own steering (as RequestWalk does): she comes round the target at about her body's width and the last
    //    stretch straight in on the asked side; from body contact she side-steps round the target facing it. At the arrival
    //    she is held where she stopped; the pose and the hold follow as before. 'C' 0 = within 8 u of the spot (as before).
    //    What you can see: the answer comes sooner after she stops; a target that moves 32 u (an NPC that turns 30 deg, for
    //    an hour-relative spot) is followed - she re-aims while she walks; O 17 also means "no route" for a request that is
    //    not close (a close request with no route answers O 17 at once; any other request then walks ONE engine leg to the
    //    spot, as 1.5.1 did); IsDirectWalking is true during the walk. A target in another interior / worldspace keeps the
    //    engine's walk (load doors).
    //  * 2.0.0 (2026-11-06): 1.6.1 released as Standpoint 2.0 - a version number only, no ABI or behaviour change.
    //  * 1.6.1 (no ABI change): that ONE engine leg ends when it gets stuck - the engine reports her movement blocked, or she
    //    has not moved for 6 s, 80 u or more from the spot - with 'O' 17 at once (1.6.0 waited out the 90 s); a CLOSE request
    //    ends on the asked distance (1.6.0 stopped about 5 u short of it on the way in).
    // ═════════════════════════════════════════════════════════════════════════

    // The request line. Slots 8-10 are open: send 0.
    struct MoveRequest {
        std::uint32_t who;           //  1  the NPC who moves (reference FormID)
        std::uint32_t where;         //  2  the target reference (NPC / furniture / object / marker); 0 = release `who`; `who` herself = hold your place (0.9.5)
        float         distanceCm;    //  3  how close, centimetres, centre to centre
        float         facing;        //  4  clock hour around the target (see above); < 0 = nearest side
        float         orientation;   //  5  clock hour of her own facing once there
        float         pose;          //  6  0 none (plain stand) | 1 standard wait (hands behind the back) | 2 kneel | 3 bend
        float         holdSeconds;   //  7  seconds she holds the spot after arrival; 0 = let go at arrival; -1 = until released
        float         close;         //  8  0 normal | 1 CLOSE CONTACT (0.9.0; the pair rule since 0.9.5, every step walked since 0.9.8 - see the rules above); was open8
        float         open9;         //  9  open - send 0
        float         open10;        // 10  open - send 0
    };
    static_assert(sizeof(MoveRequest) == 40, "MoveRequest layout is part of the ABI");

    // The result line. Slots 1-5 are MEASURED at the moment of the report.
    struct MoveResult {
        std::uint32_t who;           //  1  as asked
        std::uint32_t where;         //  2  as asked
        float         distanceCm;    //  3  measured, centre to centre
        float         facing;        //  4  measured clock hour around the target, [0, 12)  (0 = dead in front)
        float         orientation;   //  5  measured clock hour of her facing, [0, 12)      (0 = straight at the target)
        char          outcome;       //  6  'C' completed   'P' the player stopped her (grab / bump / block)
                                     //     'O' obstacle: no standable spot, or not reached in 90 s
                                     //     'D' danger on the route   'L' too far from the player (50 m leash)
                                     //     'X' released / replaced   'B' refused (all 4 walkers busy, or not ready)
        char          pad[3];        //     zero
        float         detail;        //  7  Standpoint's reason code, see MoveDetail
        float         open8;         //  8  open - 0
        float         open9;         //  9  open - 0
        float         open10;        // 10  open - 0
    };
    static_assert(sizeof(MoveResult) == 40, "MoveResult layout is part of the ABI");

    // Slot 7 of the result.
    enum MoveDetail : std::int32_t {
        kDetailClean        = 0,   // C  arrived on the point
        kDetailNoSlot       = 1,   // O  no standable spot at the target
        kDetailTimeout      = 2,   // O  not reached in 90 s
        kDetailReleased     = 3,   // X  released (a where = 0 line, or Papyrus Release)
        kDetailDanger       = 4,   // D  hostiles on or near the route
        kDetailLeash        = 5,   // L  she strayed too far from the player
        kDetailReplaced     = 6,   // X  a newer request / GoTo took this NPC over
        kDetailPlayerBump   = 7,   // P  the player bumped or body-blocked her
        kDetailPlayerGrab   = 8,   // P  the player grabbed her (VR)
        kDetailArrivedShort = 9,   // C  stopped short and could not get closer; turned where she stood (see distanceCm)
        kDetailBadRequest   = 10,  // O  `who` is not a loaded NPC, or `where` does not resolve
        kDetailNotReady     = 11,  // B  all 4 walkers busy, or Standpoint's scripts are not running
        kDetailGameLoaded   = 12,  // X  a save was loaded while the request was open
        kDetailNoSession    = 13,  // X  a release line for an NPC who had nothing running
        kDetailEnded        = 14,  // X  the session ended some other way (e.g. combat policy)
        kDetailStrayed      = 15,  // O  she stopped moving far from where she was sent (100 u or more) - not an arrival; released
        kDetailGlided       = 16,  // C  0.9.5-0.9.7 ONLY: on the point, the last stretch by a glide. Not produced since 0.9.8 (nothing slides her)
        kDetailNoWay        = 17,  // O  (0.9.9, close request to an NPC) she could not get to the point: the way is blocked and no other way worked; released
        // 1.4.0 - produced by walks (IStandpoint3::RequestWalk / Papyrus WalkTo) only:
        kDetailBlocked      = 18,  // O  blocked: no progress, and one new route from where she stood did not get her through
                                   //    (1.5.0: a new route after every stall in a new place; O 18 = none got her through,
                                   //    or a second stall at a spot an earlier new route started from)
        kDetailNotSupported = 19,  // B  not on this runtime (SE / AE), or the walk driver is not installed
        kDetailDepthReached = 20,  // C  the asked submerged level (stopSubmerged) was reached: she stopped there
        kDetailWouldSwim    = 21,  // O  she would swim (submerged 0.70 or more, or the engine says she swims): let go
        kDetailOtherDriver  = 22,  // B  another plugin drives her right now (PLANCK's grab, PPB's push walk), or VR Chain chained her
        // 1.5.0 - produced by water walks (IStandpoint4::RequestWalkToWater / Papyrus WalkToWater) only:
        kDetailNoWater       = 23, // O  Water Finder found no water within maxMetres of her
        kDetailNoWadeSpot    = 24, // O  water was found, but no wading spot she can walk to (every seed Water Finder gave was tried)
        kDetailNoWaterFinder = 25, // B  Water Finder is absent, or older than 1.6.0 (no C++ revision 2)
        // 1.6.0 - produced by walks (RequestWalk / RequestWalkToWater / Papyrus WalkTo, WalkToWater) only:
        kDetailOutsideLoaded = 26, // O  the point is outside the loaded area (the game has no ground loaded there): answered at
                                   //    once - Standpoint walks only where the ground is loaded; GoToPoint can travel there
    };

    using MoveCallback = void (*)(std::uint32_t handle, const MoveResult& result, void* user);

    struct IStandpoint2 : IStandpoint1 {
        // Hand in a request line. Returns a handle (> 0), or 0 when a_callback is null. The answer ALWAYS comes
        // through the callback (refusals too), on the main thread, once. `user` is handed back untouched.
        virtual std::uint32_t RequestMove(const MoveRequest& request, MoveCallback callback, void* user) = 0;
    };

    // ═════════════════════════════════════════════════════════════════════════
    //  REVISION 3 (Standpoint 1.4.0, SKYRIM VR) — THE WALK OFF THE NAVMESH.
    //  Ask with GetApiFunction(3).
    //
    //      StandpointAPI::WalkRequest q{};
    //      q.who = npc->GetFormID();  q.x = wx;  q.y = wy;  q.z = wz;
    //      q.facingDegrees = bearing;  q.holdSeconds = -1;
    //      g_walk->RequestWalk(q, &OnWalkDone, nullptr);
    //
    //      void OnWalkDone(std::uint32_t handle, const StandpointAPI::WalkResult& r, void*)
    //      { if (r.outcome == 'C') { /* she is there: r.distLeft, r.submerged */ } }
    //
    //  * Standpoint plans a route (the navmesh, bare ground, shallow water up to
    //    depthFrac x her height; a point that is not a place to stand moves toward
    //    her) and walks her along it with her OWN legs: it steers her movement
    //    controller every frame (planner direct control - the mechanism PLANCK's
    //    grab-drag uses); the engine plays her walk. Nothing slides or teleports her.
    //  * SKYRIM VR ONLY in 1.4.0: on SE / AE every request answers 'B', detail 19.
    //    1.5.0: on SE / AE too once Standpoint's start-up self-test says READY and
    //    Data/SKSE/Plugins/Standpoint.ini has [DirectWalk] SEAE = 1 (default 0 =
    //    B 19 as before). Since 1.5.0 the route also follows the navmesh where there
    //    is one (every leg still checked by rays), she opens unlocked ordinary doors
    //    on the way, keeps clear of NPCs standing still, and gets a new route after
    //    every stall or push in a new place.
    //  * MAIN THREAD ONLY for RequestWalk. The callback arrives on the main thread,
    //    in a later frame, exactly ONCE per request: when she has stopped on the
    //    point and turned to the asked facing, or when the walk is stopped.
    //    Refusals come through the callback too. The hold then runs by itself.
    //  * If she has a Standpoint move session (GoTo / RequestMove), it is released
    //    first (VR only - on SE / AE nothing is touched: B 19). A new RequestWalk for
    //    her replaces her older walk (X 6) and keeps her held until it takes her.
    //    To end a walk or the stand after it: a RequestMove release line
    //    (where = 0, 'X' 3), or a new RequestWalk / RequestMove for her ('X' 6).
    //  * Answers (outcome, detail): C 0 arrived (within 32 u) · C 9 she ended more than
    //    32 u away (a partial route, a point moved toward her, or something in the way
    //    at the end - read distLeft) · C 20 the stop depth was
    //    reached · O 1 no route · O 2 not there in 90 s · O 10 not a loaded NPC ·
    //    O 15 pushed off the route (1.5.0: and no new route from there) · O 18
    //    blocked (1.4.x: after one new route; 1.5.0: a new route after every stall in
    //    a new place - O 18 when none helps, or twice from one spot) · O 21 she
    //    would swim · P 8 grabbed (HIGGS) · X 3 released · X 6 replaced · X 12 a
    //    save was loaded · X 14 another end (combat, ragdoll, death, ...: the log
    //    names it) · B 11 busy · B 19 not on this runtime · B 22 another plugin
    //    drives her.
    //  * Up to about 23 m (1600 u). Not yet run in VR at the time of 1.4.0's release.
    //  * 1.5.1 (no ABI change) - UP TO 200 m (14,000 u; further = O 1 "too far
    //    (beyond 200 m)"), SPEED BY DISTANCE: she starts within 20 m of the point ->
    //    she walks all the way; 20-30 m -> she jogs, the last 10 m walked; beyond
    //    30 m (VR) -> Standpoint's own travel session RUNS her first (the engine's
    //    travel), and at 30 m the walk above takes her over without stopping (jog,
    //    the last 10 m walked). A walk beyond 30 m has 90 s + 1 s per 150 u of her
    //    distance (O 2 after it). A standing NPC's idle is broken before she is
    //    driven. B 22's log line names what holds her (PLANCK's grab, VR Chain, PPB's
    //    push walk). IsDirectWalking is false during the far phase (the engine's
    //    travel runs her then, not planner direct control). Same for RequestWalkToWater.
    //  * 1.6.0 (no ABI change) - OUR OWN PATH ALL THE WAY: the engine's far phase is GONE. Beyond the planner's
    //    reach the route follows the loaded navmesh's own triangles (a skeleton) and is checked ahead of her in
    //    pieces while she walks - she never stops for it. SPEED by the route LEFT: she runs while more than 15 m
    //    (1,050 u) are left, jogs down to 3 m (210 u) and walks the last 3 m; a pace that would last under a
    //    second is skipped; she slows for sharp turns, doors, stairs and water edges. Stairs both ways where the
    //    navmesh confirms them (she never stops ON a stair). A point outside the loaded area answers O 26
    //    (kDetailOutsideLoaded) at once; the loaded area ending mid-walk answers C 9 at the last checked point.
    //    The limit beyond 2,100 u: 90 s + 1 s per 150 u of her distance (O 2). IsDirectWalking is true all the way.
    //    The steered sessions (REVISION 2 above) are walked by this same walk.
    // ═════════════════════════════════════════════════════════════════════════

    // The walk request line. Slots 9-10 are open: send 0.
    struct WalkRequest {
        std::uint32_t who;           //  1  the NPC who walks (reference FormID; never the player)
        float         x;             //  2  the point, game units (in water: on the bed)
        float         y;             //  3
        float         z;             //  4
        float         facingDegrees; //  5  her facing at the end: compass, 0 = north, clockwise; < 0 = keep her last travel direction
        float         holdSeconds;   //  6  -1 = she stands there (held, speed 0) until released / replaced; 0 = let go at arrival; N seconds
        float         depthFrac;     //  7  the deepest water her route may use, x her height (0.05-0.65); 0 = 0.45
        float         stopSubmerged; //  8  > 0: she arrives as soon as her submerged level (0..1) reaches it; 0 = off
        float         open9;         //  9  open - send 0
        float         open10;        // 10  open - send 0
    };
    static_assert(sizeof(WalkRequest) == 40, "WalkRequest layout is part of the ABI");

    // The walk result line. Slots 4-8 are MEASURED at the moment of the report.
    struct WalkResult {
        std::uint32_t who;           //  1  as asked
        char          outcome;       //  2  'C' 'O' 'P' 'X' 'B' (see above)
        char          pad[3];        //     zero
        float         detail;        //  3  the reason code (MoveDetail)
        float         distLeft;      //  4  flat distance from her to the asked point, game units
        float         submerged;     //  5  her submerged level, 0 (dry) .. 1
        float         x;             //  6  where she is
        float         y;             //  7
        float         z;             //  8
        float         open9;         //  9  open - 0
        float         open10;        // 10  open - 0
    };
    static_assert(sizeof(WalkResult) == 40, "WalkResult layout is part of the ABI");

    using WalkCallback = void (*)(std::uint32_t handle, const WalkResult& result, void* user);

    struct IStandpoint3 : IStandpoint2 {
        // Hand in a walk line. Returns a handle (> 0), or 0 when a_callback is null. The answer ALWAYS comes through the
        // callback (refusals too), on the main thread, once. `user` is handed back untouched.
        virtual std::uint32_t RequestWalk(const WalkRequest& request, WalkCallback callback, void* user) = 0;
        // True while Standpoint drives this actor's movement (walking or standing). Any thread.
        virtual bool IsDirectWalking(std::uint32_t formId) = 0;
    };

    // ═════════════════════════════════════════════════════════════════════════
    //  REVISION 4 (Standpoint 1.5.0) — THE WALK INTO WATER. Ask with GetApiFunction(4).
    //
    //      StandpointAPI::WaterWalkRequest q{};
    //      q.who = npc->GetFormID();  q.maxMetres = 20;  q.holdSeconds = -1;
    //      q.facingDegrees = -1;                       // she ends facing the water
    //      g_water->RequestWalkToWater(q, &OnWalkDone, nullptr);
    //
    //  * Standpoint asks Water Finder (1.6.0+, a soft link) for the nearest water
    //    within maxMetres, then plans from HER toward it and stops the plan at the
    //    first wading spot she can really reach: water between depthMinFrac and
    //    depthMaxFrac x her height, with water round it (5 of its 8 neighbours), a
    //    place she can stand. Deeper water is never crossed. None reachable near
    //    that water: the next water Water Finder 1.6.2 offers, inside the same 90 s.
    //    1.6.0 - DEEPER, BY PRIORITY, IN ONE ROUTE: that first spot is the ENTRY
    //    (shin-deep: depthMinFrac, 0.15); within two body lengths of it the nearest
    //    THIGH-deep spot (0.38 x her height or more) ends the search, else the
    //    nearest KNEE-deep one (0.28), and within two body lengths of that the
    //    nearest thigh-deep one; depthMaxFrac (default 0.45 = the thigh) is the
    //    deepest on every step. Start -> entry -> knee -> thigh is ONE route - she
    //    never stops between them. A cap below the knee keeps the 1.5.x behaviour.
    //    The search first floods the floor toward the nearest reachable shin-deep
    //    water (a seed below a ledge no longer answers O 24).
    //    Then she walks there with her own legs (as RequestWalk) and turns to face
    //    out over the water (facingDegrees < 0) or to the asked facing.
    //  * The answer is a WalkResult through the same WalkCallback, once, main
    //    thread, a later frame; the handles share RequestWalk's numbers. distLeft is
    //    measured to the wading spot it chose (C 0 = within 32 u of it).
    //  * Answers: as RequestWalk (C 0 / C 9, O 1 / 2 / 10 / 15 / 18 / 21, P 8, X 3 /
    //    6 / 12 / 14, B 11 / 19 / 22), plus O 23 no water within maxMetres, O 24 water
    //    but no wading spot she can reach, B 25 Water Finder absent or older than
    //    1.6.0. Ending a walk / a stand: as RequestWalk.
    // ═════════════════════════════════════════════════════════════════════════

    // The water walk request line. Slots 7-10 are open: send 0.
    struct WaterWalkRequest {
        std::uint32_t who;           //  1  the NPC who walks (reference FormID; never the player)
        float         maxMetres;     //  2  how far Water Finder looks for water, metres; <= 0 = 20 (the walk itself reaches ~23 m; 1.5.1: 200 m)
        float         depthMinFrac;  //  3  the shallowest a wading spot may be, x her height; <= 0 = 0.15
        float         depthMaxFrac;  //  4  the deepest water her route and spot may use, x her height (0.05-0.65); <= 0 = 0.45
        float         holdSeconds;   //  5  -1 = she stands there until released / replaced; 0 = let go at arrival; N seconds
        float         facingDegrees; //  6  < 0 = she faces the water; else a compass facing (0 = north, clockwise)
        float         open7;         //  7  open - send 0
        float         open8;         //  8  open - send 0
        float         open9;         //  9  open - send 0
        float         open10;        // 10  open - send 0
    };
    static_assert(sizeof(WaterWalkRequest) == 40, "WaterWalkRequest layout is part of the ABI");

    struct IStandpoint4 : IStandpoint3 {
        // Hand in a water walk line. Returns a handle (> 0), or 0 when a_callback is null. The answer ALWAYS comes through
        // the callback (refusals too), on the main thread, once. `user` is handed back untouched. MAIN THREAD.
        virtual std::uint32_t RequestWalkToWater(const WaterWalkRequest& request, WalkCallback callback, void* user) = 0;
    };

    struct StandpointMessage {
        enum : std::uint32_t { kGetInterface = 0x53545054 };  // 'STPT'
        void* (*GetApiFunction)(unsigned int revision) = nullptr;
    };

}  // namespace StandpointAPI
