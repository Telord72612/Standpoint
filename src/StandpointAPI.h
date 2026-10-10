// ─────────────────────────────────────────────────────────────────────────────
//  StandpointAPI.h — the Standpoint consumer contract for SKSE plugins.
//
//  Two things a plugin can ask Standpoint for:
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
//          g_move  = static_cast<StandpointAPI::IStandpoint2*>(msg.GetApiFunction(2));
//          g_solve = static_cast<StandpointAPI::IStandpoint1*>(msg.GetApiFunction(1));
//      }
//
//  Dispatch AFTER kDataLoaded. Null GetApiFunction = Standpoint is not installed.
//  Null from GetApiFunction(2) = a Standpoint older than 0.8.0 (revision 1 still
//  answers). IStandpoint2 extends IStandpoint1: a revision-2 pointer has the
//  solver too.
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
    };

    using MoveCallback = void (*)(std::uint32_t handle, const MoveResult& result, void* user);

    struct IStandpoint2 : IStandpoint1 {
        // Hand in a request line. Returns a handle (> 0), or 0 when a_callback is null. The answer ALWAYS comes
        // through the callback (refusals too), on the main thread, once. `user` is handed back untouched.
        virtual std::uint32_t RequestMove(const MoveRequest& request, MoveCallback callback, void* user) = 0;
    };

    struct StandpointMessage {
        enum : std::uint32_t { kGetInterface = 0x53545054 };  // 'STPT'
        void* (*GetApiFunction)(unsigned int revision) = nullptr;
    };

}  // namespace StandpointAPI
