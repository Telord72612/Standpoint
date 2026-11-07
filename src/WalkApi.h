#pragma once
// ─────────────────────────────────────────────────────────────────────────────
//  WalkApi.h — v1.4.0: THE WALK OFF THE NAVMESH, end to end ("Walking off the navmesh" in the integration guide).
//
//  One request in, one answer out:
//    request (Papyrus StandpointAPI.WalkTo / C++ IStandpoint3::RequestWalk / the stpt test)
//      -> the hybrid planner, all-direct (PathPlanner.cpp, standpoint_path::StartEx: navmesh + bare floor + shallow water)
//      -> the chain-pull driver walks her along the route (DirectWalk.cpp: planner direct control, real steps)
//      -> ONE answer: the C++ callback (main thread, a later frame), or the Papyrus event "Standpoint_WalkResult" + the
//         poll natives (WalkOutcome / WalkResultValues).
//  A progress stall gets a re-plan from where she stands (the stall spot is marked bad for the planner first; 1.4.x: ONE, then
//  O 18 - 1.5.0: see below).
//  The overall limit is Standpoint's 90 s (pause excluded; the hold after the answer is not counted).
//  VR only in 1.4.0: on SE / AE every request answers B 19.
//  1.5.0: the MIX (navmesh corridor + rays, doors opened, standing actors kept clear of); a new route after EVERY stall, door
//  that did not open or push off the line in a new place (never twice from one spot: O 18); THE WADE-SPOT WALK (RequestWater:
//  Water Finder's seed -> the planner's wade-spot search -> the walk, she faces the water); SE / AE walk when the driver's
//  self-test is READY and Standpoint.ini says SEAE = 1 (else B 19).
//  1.5.1: UP TO 200 m with SPEED BY DISTANCE (the user's rule); a walk beyond 30 m has 90 s + d0 / 150 u/s. Beyond 200 m: O 1.
//  1.6.0 (report "1.6.0 Plan" D = the spec, S2 §2-§3, S1 §6-§7):
//    * THE ENGINE FAR PHASE IS GONE (no slot package at RUN, no far marker, no adoption, no leak guard). Its four Papyrus natives
//      stay registered with neutral answers so the driver's dormant far code still links.
//    * OUR OWN LONG PATH, ROLLING: the planner's navmesh SKELETON (NavGraph: the loaded navmesh triangles), then verified CHUNKS
//      (chunk 1 >= 1,400 u; the next <= 2,400 u, asked while she walks once < 1,400 u of checked route are left), each appended
//      to the route she walks (DirectWalk::Extend) - she never stops for it. A point outside the loaded cells = O 26 at once;
//      the loaded edge reached mid-walk = C 9 at the last checked point. Re-plans (a moved goal, a failed chunk) are planned
//      from where she will be and SWAPPED in while she moves (DirectWalk::Swap) - one mechanism for Retarget, stpf and chunks.
//    * THE SPEED BANDS by route LEFT (DirectWalk Params::bands): run beyond 1,050 u, jog to 210 u, walk the last 210 u; a band
//      that would last under 1 s is skipped. The limit of a walk beyond 2,100 u: 90 s + d0 / 150 u/s.
//    * THE STEERED SESSIONS' WALK (S1 W1-W7): RequestSteer / Retarget / HandOff / HoldHere below (SteerSession.cpp uses them).
//
//  Answers (letter, detail - StandpointAPI.h MoveDetail):
//    C 0 arrived (within 32 u) · C 9 more than 32 u away (PARTIAL route, moved point, blocked at the end; 1.6.0: the loaded
//    edge reached mid-walk) · C 20 depth reached
//    O 1 no route / too far (beyond 200 m) · O 2 the time limit (90 s; beyond 2,100 u: 90 s + d0 / 150 u/s) · O 10 not a
//    loaded NPC · O 15 strayed (pushed off the line, no new route)
//    O 18 blocked (no new route, or none twice from one spot) · O 21 she would swim · P 8 grabbed (HIGGS) · X 3 released
//    X 6 replaced · X 12 a game load · X 14 another end (the log names it) · B 11 busy · B 19 not on this runtime / hook not live
//    B 22 another plugin drives her (or VR Chain chained her) · 1.5.0 water walks: O 23 no water · O 24 no reachable wading
//    spot · B 25 no Water Finder 1.6.0+ · 1.6.0: O 26 the point is outside the loaded area
// ─────────────────────────────────────────────────────────────────────────────
#include "StandpointAPI.h"

namespace standpoint_walk {

    // C++ door (IStandpoint3::RequestWalk). MAIN THREAD. A handle >= 1, or 0 when a_cb is null. The answer ALWAYS comes
    // through the callback (refusals too), once, on the main thread, in a later frame.
    std::uint32_t Request(const StandpointAPI::WalkRequest& a_req, StandpointAPI::WalkCallback a_cb, void* a_user);
    // 1.5.0 C++ door (IStandpoint4::RequestWalkToWater) - the wade-spot walk; the same one-answer contract as Request.
    std::uint32_t RequestWater(const StandpointAPI::WaterWalkRequest& a_req, StandpointAPI::WalkCallback a_cb, void* a_user);
    // 1.5.0 kDataLoaded: Water Finder's C++ interface is asked for once (revision 2 = the seed; revision 3 = other candidates)
    void OnDataLoaded();
    // v1.4.1, INTERNAL (the stpt follow program - not in the API). EVERY end of such a walk: its ONE answer (a_holdEnd false)
    // and, after a C, the end of the stand that follows (a_holdEnd true), with the driver's word for it ("replaced",
    // "released", "grabbed", "taken", "other-driver", "combat", "arrived", ...: static strings). Main thread, a later frame.
    using EndNoticeFn = void (*)(std::uint32_t a_handle, const StandpointAPI::WalkResult& a_r, const char* a_word, bool a_holdEnd,
                                 void* a_user);
    // Request with: her facing at the arrival and while she stands = toward a_faceWho, read live (a_req.facingDegrees is the
    // fallback); the answer AND the stand's end through a_notice (no public callback); a_freeSession false = her Standpoint
    // session is NOT released first (review R #3: only the user's own command may take her from another owner). MAIN THREAD.
    std::uint32_t RequestFacing(const StandpointAPI::WalkRequest& a_req, RE::FormID a_faceWho, EndNoticeFn a_notice, void* a_user,
                                bool a_freeSession);
    // v1.4.1 INTERNAL: requests / stops for ONE watched NPC that did not come from RequestFacing are counted - a new walk
    // (WalkTo / RequestWalk), WalkStop (Release, every Standpoint session start: GoTo / GoToPoint / RequestMove), a C++ release
    // line. a_who 0 = watch nobody. Any thread.
    void          WatchForeign(RE::FormID a_who);
    std::uint32_t ForeignCount();
    bool          HandleOwned(std::int32_t a_handle);   // its walk / stand still runs and NO stop is marked on it (by anyone)
    // 1.6.0 (S1 W1): a request / stop for a_who that is not the follow program's own - counted (it ends stpf for her). RequestSteer
    // and HoldHere call it themselves; a steered session's start may call it before its first walk (the solve). Any thread.
    void          NoteForeign(RE::FormID a_who);

    // ─────────────────────────────────────────────────────────────────────────────────────────────────────────────────
    //  1.6.0 THE STEERED SESSIONS' WALK (report "1.6.0 Plan" S1 §7 W1-W7, spec D §1 items 4 + 5) - INTERNAL (SteerSession.cpp).
    //  Every one of these walks is an ordinary walk off the navmesh (the rolling plan, the speed bands, doors, re-plans after a
    //  stall, the 90 s / long-walk limit, one answer, then the stand) with these differences: her Standpoint session is NEVER
    //  released (the Papyrus driver owns it - the walk is the session's legs); the planner gets `avoid` (the target's keep-out
    //  circle, never an actor disc) and the caller's flags; fixed points can follow the planned route (the radial last leg) or
    //  replace the plan (the orbit); she faces faceWho read live + an offset; the NEAR RULE; every line says STEER #steerId and
    //  every notice carries it. MAIN THREAD for all four functions (the frame step: SteerSession's Frame runs inside it).
    // ─────────────────────────────────────────────────────────────────────────────────────────────────────────────────

    // One end of a steered walk: its ONE answer (holdEnd false), then - after a C with a hold - the end of her stand (holdEnd
    // true). Delivered on the main thread in a later frame (an SKSE task), never from inside the call that caused it.
    struct SteerNotice {
        std::int32_t              steerId{ 0 };        // as asked (SteerAsk::steerId / HoldHere's a_steerId)
        std::uint32_t             walkId{ 0 };         // the walk's handle (RequestSteer / HoldHere; Retarget keeps it)
        StandpointAPI::WalkResult r{};                 // letter, detail, distLeft (flat, to the spot S), submerged, x / y / z
        const char*               word{ "" };          // the driver's word (a static string): "arrived", "near", "replaced",
                                                       // "session" (HandOff), "grabbed", "combat", "furniture", "no-route", ...
        bool                      holdEnd{ false };    // false = the answer; true = the end of the stand after a C answer
        bool                      holding{ false };    // (answer) a C, and she now stands, driven at speed 0, until HandOff /
                                                       // StopHandle / a newer walk of hers / the hold time / its own end
        bool                      nearRule{ false };   // (answer) the near rule made it: a BLOCKED / pushed-off report within
                                                       // nearRadius of S - she stands held where she stopped (C 9, word "near")
        float                     facingErrDeg{ 0.f }; // (a C answer) her heading vs the final heading at the arrival
    };
    using SteerNoticeFn = void (*)(const SteerNotice& a_notice, void* a_user);

    // W1: what a steered session asks of the walk (S1 §2.2: P = her position, T = the target, S = the spot, E = the approach point)
    struct SteerAsk {
        RE::FormID                who{ 0 };             // the walker (a loaded NPC, never the player)
        std::int32_t              steerId{ 0 };         // SteerSession's id: every WALK line of this walk names STEER #steerId
        RE::NiPoint3              spot{};               // S: where her feet end - distLeft, C 0 (32 u) and the near rule measure to it
        RE::NiPoint3              planTo{};             // the planner's goal: S itself, or the approach point E (tail = [S]).
                                                        // Ignored with a fixedRoute
        std::vector<RE::NiPoint3> tail;                 // fixed points walked AFTER the planned route, in order (E -> S: the radial
                                                        // last leg - probed by the caller; never planned or tested here)
        std::vector<RE::NiPoint3> fixedRoute;           // non-empty = NO plan: she walks exactly these points from where she stands
                                                        // (the orbit: step-out, arc chords, the radial leg - probed by the caller);
                                                        // its last point should be S
        RE::FormID                avoid{ 0 };           // the target: the planner's keep-out circle, never an actor disc; 0 = none
        std::uint32_t             planFlags{ 0 };       // 0 = the walk's MIX; else exactly these standpoint_path flags (kAllDirect is
                                                        // always added: every leg is walked directly) - e.g. the MIX without
                                                        // kPartial / kGoalMayMove (RELATIVE), the MIX as it is (POINT legs)
        float                     depthFrac{ 0.45f };   // the deepest water a route may use, x her height (0.05-0.65)
        float                     facingDeg{ -1.f };    // her final heading (compass) when faceWho is 0 / not loaded; < 0 = her
                                                        // last travel direction
        RE::FormID                faceWho{ 0 };         // her facing at the arrival and while she stands = toward this reference,
                                                        // read LIVE, + faceOffsetDeg (D1); 0 = facingDeg
        float                     faceOffsetDeg{ 0.f }; // D1: added to the live bearing toward faceWho (6h = 180: her back to it)
        bool                      faceAll{ false };     // D2: hold that facing over the WHOLE route when she can strafe (the orbit's
                                                        // side-step); the blend watch turns it off if she does not strafe
        float                     hold{ -1.f };         // -1: she stands, driven, until HandOff / StopHandle; 0: let go at the
                                                        // point; N: seconds
        float                     nearRadius{ 0.f };    // > 0: THE NEAR RULE - a BLOCKED / pushed-off report within this of S (2D)
                                                        // is an ARRIVAL (C 9, word "near", she stands held), never a re-plan
        SteerNoticeFn             notice{ nullptr };    // REQUIRED: every end (the answer, then the stand's end)
        void*                     user{ nullptr };      // handed back untouched
    };

    // W1: the steered walk. A handle >= 1, or 0 when a_ask.notice is null or who is 0 (nothing is opened then). Her older walk
    // / stand is REPLACED (X 6 to its owner) and HOLDS her until this one's drive takes her (no let-go between: the planning
    // hold, HoldHere). Counts as a foreign request for the follow program (stpf ends). Her session is NOT released. MAIN THREAD.
    std::uint32_t RequestSteer(const SteerAsk& a_ask);
    // W2 (S1 §2.6, S2 §2.4): her steered walk aims somewhere new WITHOUT a stop - the new route is planned from where she will
    // be in 0.5 s along her current one while she keeps walking it (a new spot more than 90 deg off her travel direction: she
    // is held at speed 0 meanwhile), then swapped in where she is (DirectWalk::Swap). Takes a_ask's spot / planTo / tail /
    // fixedRoute / facingDeg / faceWho / faceOffsetDeg / faceAll / nearRadius / avoid / planFlags / depthFrac; who / steerId /
    // hold / notice / user are kept from the request. Still planning: planned again at once. false = no such walk, it has
    // answered (she stands: ask a new RequestSteer), or a stop is marked on it. MAIN THREAD.
    bool Retarget(std::uint32_t a_handle, const SteerAsk& a_ask);
    // W3 (S1 §2.5): the session takes her over (the pin): her stand ends QUIETLY - no new answer (a walk that has not answered
    // yet answers X 3 with the word "session"), a "HOLD END: session" line, the hold-end notice (word "session"); the hook hands
    // her back at its next visit and her package is re-evaluated then (the session's override is on by then). false = no such
    // walk. MAIN THREAD.
    bool HandOff(std::uint32_t a_handle);
    // W7 (S1 §2.2 planning hold, §2.5 `where = who` off the navmesh): a stand-only drive at her feet - no plan. She is taken,
    // faces toward a_faceWho (+ a_faceOffsetDeg) when given, else keeps her facing, and stands driven at speed 0 for a_hold (-1
    // until HandOff / StopHandle / a newer walk of hers - which INHERITS the hold: no let-go between). Answers C 0 at once (the
    // notice, holding true), then the stand's end. A handle >= 1 (0 when who is 0). Foreign for stpf. MAIN THREAD.
    std::uint32_t HoldHere(RE::FormID a_who, float a_hold, std::int32_t a_steerId = 0, SteerNoticeFn a_notice = nullptr,
                           void* a_user = nullptr, RE::FormID a_faceWho = 0, float a_faceOffsetDeg = 0.f);
    // W4 helper: the walk pump (the frame step that calls standpoint_steer::Frame) runs - started again when it had stopped.
    // SteerSession's natives call it after recording work for Frame. Any thread.
    void Wake();

    // any thread: Standpoint drives her (walking or standing) - planner direct control (1.6.0: a steered session's legs too)
    bool IsDirectWalking(std::uint32_t a_formId);
    bool Supported();                               // any thread: VR and the driver's hook installed

    // Any thread. Her walk / stand ends now: an open walk answers X a_detail (3 released, 6 replaced); a stand after the
    // answer simply ends. true = she had one.
    bool StopFor(RE::FormID a_who, std::int32_t a_detail);
    // The same, by handle (the stpt tests only). v1.4.1: true only when THIS call marked the stop - false when the walk is
    // gone or another stop / a replacement was already marked on it (she is someone else's then: not touched).
    bool StopHandle(std::int32_t a_handle, std::int32_t a_detail);
    bool HandleLive(std::int32_t a_handle);                          // a walk / stand of this handle still runs

    // kSaveGame: the driver hands every driven walker back before the save; she is re-taken next frame
    void OnSaveGame();
    // kPreLoadGame / kNewGame: every open walk answers X 12 (C++ callbacks only; 1.6.0: no notice for internal walks); nothing
    // is touched
    void OnGameLoad();

    // class Standpoint: WalkStart, WalkStop, WalkOutcome, WalkResultValues, WalkSupported (1.4.0); WalkToWaterStart,
    // WaterWalkSupported (1.5.0); WalkFarPackage, WalkFarAlive, WalkFarEndMode, WalkReleaseMode (1.5.1 - 1.6.0: neutral stubs,
    // the far phase is gone: no-op / False / 1 / 0)
    bool RegisterNatives(RE::BSScript::IVirtualMachine* a_vm);

}  // namespace standpoint_walk

// The two walk programs of the `stpt` console test (TestCmd.cpp triggers them). Main thread (console).
namespace standpoint_walk_test {
    bool Running();                          // a test walk (or its hold) still runs
    void StopRunning();                      // ... ends it: X 3
    void StartWaterWalk(RE::Actor* a_walker);   // 1.5.0: a WalkToWater (20 m, depth 0.15-0.45 (1.6.0: thigh), hold 20 s, facing the water)
    void StartFrontWalk(RE::Actor* a_walker);   // she walks to the point 3 m in front of the player
    void OnGameLoad();                       // (also ends the follow program, quietly: the walks answer X 12 themselves)

    // v1.4.1 THE FOLLOW-FRONT PROGRAM (its own console command, TestCmd.cpp): she keeps walking to the point 3 m in front of
    // the player's body facing, facing him, until it is stopped. A new walk when that point has moved >= 64 u from her
    // current / last walk's point AND he has stood still (< 8 u) for 0.5 s, or - after a walk that did not end C - as soon as
    // the point has moved >= 64 u. Main thread, except FollowOn (any thread: the walk pump keeps running while it is on).
    // 1.6.0: a walk still on its way is RE-AIMED (no stop: the Retarget mechanism); a new walk only once she stands. While the
    // driver's gate refuses her (sitting, mounted, combat ...) nothing is sent (ONE HUD line); once she is free the trigger is "the
    // point is >= 64 u from HER" (S1 §6) - 1.6.1 (B1): only after such a wait; after any other failed walk the point must move
    // 64 u from that walk's point (1.4.1), and a new walk measures from its own point again.
    bool FollowOn();
    void StartFollow(RE::Actor* a_walker);   // a single walk test of hers (StartFrontWalk / StartWaterWalk) is replaced (X 6)
    void StopFollow(const char* a_why);      // she is let go (her walk / stand: X 3); logged with a_why
    void FollowFrame(float a_dt);            // WalkApi's frame step, every frame while it is on (pause excluded)
}
