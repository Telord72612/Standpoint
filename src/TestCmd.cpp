// ─────────────────────────────────────────────────────────────────────────────
//  TestCmd.cpp — the console test of the move API (v0.8.0; ONE command since v0.8.2; two programs since v0.9.0).
//
//      stpt   click an NPC in the console, type stpt        -> she is the walker
//             click the target, type stpt again             -> the test runs
//             type stpt while it runs                       -> it stops, she is released
//             click the WALKER and type stpt (no test)      -> the walker is cleared, pick another
//
//  The walker is sent around the target through the SAME request line another plugin would send
//  (standpoint_move::Request), one step after the other; every request and result line is printed
//  (console + corner + Standpoint.log).
//
//  WHICH PROGRAM RUNS depends on what the target is:
//    * anything that is NOT an actor (furniture, an object, a marker) -> the STANDARD program (6 steps, 150 cm):
//      front facing it | its right, plain stand | behind it, her back to it | its left, turned to her right |
//      as close as allowed on the nearest side | front, a 60 s hold released after 5 s.
//    * an ACTOR -> the CLOSE-CONTACT program (v0.9.0; this one is v0.9.8 - report 05 §12-§17): the measuring run.
//      The TARGET is first told to hold her place (a `where = who` line). Every walker step is a CLOSE request; the
//      pair "ignore each other" rule, the stops of a route and the second walk in all come from the driver, so the
//      test arms nothing. The walker is HELD between the steps (every step asks hold -1; the next request retasks
//      her). From 30 cm on one side the test asks straight for the next side (v0.9.7); since v0.9.8 nothing slides
//      her, so "30 cm" right after "45 cm" on the same side is a step back and a new walk in. 16 steps:
//        front                150 cm -> 45 cm -> 30 cm
//        right, then behind   45 cm (a quarter turn round: back and around) -> 30 cm
//        front                30 cm, from behind her (the opposite side)
//        left                 45 cm (a quarter turn the other way) -> 30 cm
//        front                150 cm from her left (back, then across) -> 30 cm straight from there -> back out to 150 cm
//        then the walker is released, then the target.
//      The log shows both actors' controller shapes, the gap between them, what both AIs did on the way in, and
//      where the PLAYER stood.
//  Nothing here is part of the API: other plugins never see it.
//
//  v1.4.0 - THE WALK OFF THE NAVMESH (VR): two more programs, reached ONLY while the player is SNEAKING (crouched), so that
//  nothing that works today changes (a walker + `prid 14` + stpt, standing, is still the close-contact program toward the
//  player):
//    * sneaking, a walker chosen, click HER again (prid her) + stpt  -> the WATER WALK: Water Finder (soft dependency,
//      its C++ API) finds water within 20 m of her; she walks in to its point, facing the water, holds 20 s
//      (standpoint_walk_test::StartWaterWalk in WalkApi.cpp - depth up to 0.6 x her height, the test only; 1.6.0: 0.45).
//      v1.5.0: it is a WalkToWater (IStandpoint4's path: Standpoint asks Water Finder, the planner's wade-spot search picks
//      the wading spot she can reach - depth 0.15-0.6 x her height - she faces the water, holds 20 s). 1.6.0: the cap is
//      0.45 x her height (the thigh, the API's own default) and she wades by priority - shin, then knee, then thigh, in one route.
//    * sneaking, a walker chosen, prid 14 (the player) + stpt        -> she walks to the point 3 m in front of the player,
//      then turns to face him, holds 10 s (StartFrontWalk)
//    * stpt while a walk test runs                                    -> it stops (X 3), she is let go
//  Both go through the same path as IStandpoint3::RequestWalk; the log lines are WALK #n ... and [TEST] ...
//
//  v1.4.1 - THE FOLLOW-FRONT PROGRAM, on a SECOND console command (the user, 2026-11-03, after the sneak trigger went
//  wrong three times in VR: "DO a different console command for having a NPC arrive 3m in front of me, and not doing only
//  one target, like it stay on until i type STPT again"). No sneaking:
//    * an NPC clicked + the command        -> she becomes the walker; the program starts
//    * nothing clicked (or the player / an object) + the command -> the current stpt walker; none -> a one-line hint
//    * the command again, or stpt          -> it stops, she is let go (X 3); a game load / her death / unload stop it too
//  She walks to the point 3 m in front of the player's body facing (walks off the navmesh, hold -1, facing him), and again
//  whenever that point has moved 64 u from her walk's point and he has stood still 0.5 s (after a walk that did not end C:
//  as soon as it has moved 64 u). Lines: [TEST] FOLLOW #k ... and WALK #n ... (standpoint_walk_test::StartFollow, WalkApi.cpp).
//  Its donor list is its OWN (kFollowDonors), separate from stpt's, so stpt's donor never changes.
//
//  v1.5.1 - every walk of both programs follows the 1.5.1 rules (WalkApi.cpp / DirectWalk.cpp; nothing changed here): within
//  20 m she walks; 20-30 m she jogs and walks the last 10 m; beyond 30 m (the player walked off before he stood still) the far
//  phase - Standpoint's own travel session runs her, then the direct walk takes her over at 30 m; up to 200 m. A standing walker's
//  idle is broken before she is driven (the 1.5.0 run's two animation-lock ends were stpf / stpt walks).
//
//  WHY ONE COMMAND (v0.8.2, 2026-10-09): v0.8.0 / v0.8.1 registered two (stpw + stpt). In this load order the free
//  vanilla console slots are almost gone; Standpoint registers before the VRCD AddOn and its second command took
//  'ToggleDecompressToCopy' - the slot the AddOn's `vrcw` walk test lands on. VRTEDD_AddOn.log 2026-10-09 08:18:
//  "no free console command left for 'vrcw'". One command on a slot the AddOn does not need gives `vrcw` back.
//
//  v1.6.0 - THE SIDE-BY-SIDE TEST `stpm` (report "1.5.1 Test Plan" 00 §6; "1.6.0 Plan" S1 §11, S2 §11; spec D item 11), a THIRD
//  command on its OWN donor list (the census's spares - stpf registers first and keeps its own donor):
//    * click an NPC + stpm      -> she is the mix walker (A)
//    * click a target + stpm    -> the run: a chair / an object / an NPC (an NPC target also becomes walker B in part D)
//    * stpm while it runs       -> it stops (both walkers released); a game load stops it too
//  ~32 steps (1.6.1; 29 in 1.6.0) on a timer, through the PAPYRUS doors the consumers use (StandpointAPI.GoToPoint / GoTo / WalkTo / WalkToWater /
//  RequestMove / Release - VM calls, never a latent walk call): A the same spot three ways (point / walk / a marker at 0 cm),
//  B the same target by GoTo (4 sides, nearest + auto pose) vs RequestMove (12 / 3 / 6 / 9 with kneel / bend, close 45 cm),
//  C the hand-offs on one NPC (walk -> session X 6, session -> walk Failed 3, Release after a hold and mid-walk), D two NPCs at
//  once, E the walk into water. Every step: ONE "[TEST] MIX k/N ..." log marker + ONE HUD line (<= 14 words, <= 75 characters -
//  checked at compile time); every Standpoint_* event the driver's ear hears is logged ("[TEST] MIX event ..."); every wait has a
//  time limit. Spots: on the ground in front of the player, fixed at the start (never mid-air).
//
//  v1.6.1 (the first 1.6.0 VR run - report "1.6.0 Plan" R3 §1, §5, §9): (C1) the event names are compared case-insensitively (the
//  VM handed "failed" over for the driver's "Failed": B1's answer was never seen and the run hung); (C2) P1 and P2 are STANDABLE
//  OPEN FLOOR - the planner's stand probe + the navmesh, else the nearest spot that is (logged; none within 128 u = the run does
//  not start) - runs 1 and 3 put P1 under the dining table; and every A door walks a real distance: she is walked to P2 first
//  (A1s / A3s / A5s), so A1, A3 and A5 each walk P2 -> P1 (A3 and A5 started 0-5 u from P1).
// ─────────────────────────────────────────────────────────────────────────────
#include "PCH.h"
#include "MoveApi.h"
#include "WalkApi.h"
#include "SteerSession.h"
#include "PathPlanner.h"
#include "SlotSolver.h"

#include <array>
#include <atomic>
#include <cctype>
#include <chrono>
#include <string_view>
#include <thread>

namespace logger = SKSE::log;

namespace standpoint_interrupts {
    std::int32_t GetRunOncePackageTypeId(RE::Actor* a_actor);
}

namespace standpoint_test {
namespace {

    constexpr float kHavokPerUnit = 0.0142875f;   // Havok units per game unit (FollowerBumpGuard's constant)
    constexpr float kCmPerUnit    = 1.428f;

    // v0.9.3: a step says WHO moves and WHERE she is sent.
    enum Mover : std::uint8_t { kWalkerMoves = 0, kTargetMoves = 1 };
    enum Aim   : std::uint8_t {
        kAtTarget = 0,   // where = the target (a walker step)
        kHoldPlace = 1,  // where = who: "hold your place"
        kReleaseLine = 2 // where = 0: the release line for `who`
    };

    struct Step {
        float       distCm, facing, orient, pose, hold, close;
        float       pause;       // v0.9.4: seconds between this step's result (when it was C) and the next step
        Mover       who;
        Aim         aim;
        const char* what;
    };

    // PROGRAM 1 - a target that is not an actor (unchanged since v0.8.2; every step is the walker going to the target;
    // pause 8 s = the old "hold + 2 s")
    constexpr Step kStandard[] = {
        { 150.f, 12.f, 12.f, 1.f, 6.f,  0.f, 8.f, kWalkerMoves, kAtTarget, "in front of it, facing it, standard wait" },
        { 150.f, 3.f,  12.f, 0.f, 6.f,  0.f, 8.f, kWalkerMoves, kAtTarget, "at its right, facing it, plain stand (pose 0)" },
        { 150.f, 6.f,  6.f,  1.f, 6.f,  0.f, 8.f, kWalkerMoves, kAtTarget, "behind it, her back to it" },
        { 150.f, 9.f,  3.f,  1.f, 6.f,  0.f, 8.f, kWalkerMoves, kAtTarget, "at its left, turned to her right" },
        { 10.f,  -1.f, 12.f, 1.f, 6.f,  0.f, 8.f, kWalkerMoves, kAtTarget, "as close as allowed, on the nearest side" },
        { 150.f, 12.f, 12.f, 1.f, 60.f, 0.f, 8.f, kWalkerMoves, kAtTarget, "in front, a 60 s hold - released after 5 s" },
    };

    // PROGRAM 2 - an actor target: the close-contact measuring run (report 05).
    // v0.9.1 (after the first run - report 05 §10): EVERY walker step is a close request, and every close approach
    //   starts from 150 cm on the SAME side ("go" steps) - asked to go from the target's front to her right while in
    //   body contact, the engine did not move her at all.
    // v0.9.3 (after the second run - report 05 §12):
    //  * The TARGET is held first (a `where = who` line, pose 0, until released): Carmella walked 214 u away during
    //    the last run, and a request is solved against where the target stands at that moment.
    //  * No step arms anything: every close request toward an NPC gets the pair rule and the glide from the driver.
    //  * 45 cm was reached in the last run on two sides (41 / 46 cm) and missed on two (the engine's walk stopped
    //    15-40 u short): with the glide, is 45 cm reached on ALL four sides? Then 30 cm straight in from there, and
    //    20 cm at her front: how close do the pair rule and the glide really bring her, and what do the two physical
    //    bodies do when they overlap that far?
    // v0.9.4 (the review of 0.9.3): the WALKER is never free between two steps. Every walker step asks hold -1 - she
    //   is held on her spot until the next request retasks her (0.9.3 let each hold run out 2 s before the next step:
    //   a follower then started back to the player, and "30 cm straight in" could begin from anywhere). The pause
    //   before the next step is the step's own field: 1 s after a "go" step, 3 s after a close step, 4 s after the
    //   20 cm step. A retask of the same two keeps the pair rule armed without a gap (the DLL's pool reuses the
    //   entry). At the end the walker gets a release line, then the target.
    // v0.9.7 (the user's ruling after the third run - report 05 §14.4, §15): STEP BACK AND AROUND is the driver's own
    //   business now, so the test no longer walks her out by hand before each side. From 30 cm on one side it asks
    //   straight for the NEXT side: a quarter turn to the right (R45), another one (B45), the OPPOSITE side (F30 from
    //   behind her), a quarter turn the other way (L45), and a far point on another side (150 cm in front, from her
    //   left). The driver must turn each of these into a route (its ROUTE lines in the log).
    // v0.9.8 (after the fourth run - report 05 §16, §17): NOTHING SLIDES HER. "30 cm" asked right after "45 cm" on the
    //   same side (10 u apart) is no longer a glide: she steps back to her approach stop and walks in again. The
    //   20 cm step is gone (not reachable on foot: the fourth run ended 37 cm on the far side of the point); in its
    //   place 30 cm is asked straight from 150 cm. Every text is short: the corner of the headset shows under 15 words.
    constexpr Step kClose[] = {
        { 0.f,   0.f,  0.f,  0.f, -1.f, 0.f, 2.f, kTargetMoves, kHoldPlace,   "the TARGET holds her place (plain stand)" },
        { 150.f, 12.f, 12.f, 1.f, -1.f, 1.f, 1.f, kWalkerMoves, kAtTarget,    "go: 150 cm in FRONT of her" },
        { 45.f,  12.f, 12.f, 1.f, -1.f, 1.f, 3.f, kWalkerMoves, kAtTarget,    "F45  45 cm at her FRONT" },
        { 30.f,  12.f, 12.f, 1.f, -1.f, 1.f, 3.f, kWalkerMoves, kAtTarget,    "F30  30 cm at her FRONT - back, and in again" },
        { 45.f,  3.f,  12.f, 1.f, -1.f, 1.f, 3.f, kWalkerMoves, kAtTarget,    "R45  45 cm at her RIGHT - back and around" },
        { 30.f,  3.f,  12.f, 1.f, -1.f, 1.f, 3.f, kWalkerMoves, kAtTarget,    "R30  30 cm at her RIGHT - back, and in again" },
        { 45.f,  6.f,  12.f, 1.f, -1.f, 1.f, 3.f, kWalkerMoves, kAtTarget,    "B45  45 cm BEHIND her - back and around" },
        { 30.f,  6.f,  12.f, 1.f, -1.f, 1.f, 3.f, kWalkerMoves, kAtTarget,    "B30  30 cm BEHIND her - back, and in again" },
        { 30.f,  12.f, 12.f, 1.f, -1.f, 1.f, 3.f, kWalkerMoves, kAtTarget,    "F30  30 cm at her FRONT - around from behind her" },
        { 45.f,  9.f,  12.f, 1.f, -1.f, 1.f, 3.f, kWalkerMoves, kAtTarget,    "L45  45 cm at her LEFT - back and around" },
        { 30.f,  9.f,  12.f, 1.f, -1.f, 1.f, 3.f, kWalkerMoves, kAtTarget,    "L30  30 cm at her LEFT - back, and in again" },
        { 150.f, 12.f, 12.f, 1.f, -1.f, 1.f, 1.f, kWalkerMoves, kAtTarget,    "go: 150 cm FRONT from her left - back, across" },
        { 30.f,  12.f, 12.f, 1.f, -1.f, 1.f, 3.f, kWalkerMoves, kAtTarget,    "F30  30 cm at her FRONT, straight from 150 cm" },
        { 150.f, 12.f, 12.f, 1.f, -1.f, 1.f, 1.f, kWalkerMoves, kAtTarget,    "go: straight back out to 150 cm in FRONT" },
        { 0.f,   0.f,  0.f,  0.f, 0.f,  0.f, 1.f, kWalkerMoves, kReleaseLine, "the WALKER is released" },
        { 0.f,   0.f,  0.f,  0.f, 0.f,  0.f, 0.f, kTargetMoves, kReleaseLine, "the TARGET is released" },
    };

    // main thread only (console exec, task-interface tasks, the API callback)
    const Step*      g_prog = kStandard;
    int              g_progN = static_cast<int>(std::size(kStandard));
    bool             g_closeProg = false;
    RE::FormID       g_walker = 0;
    RE::FormID       g_target = 0;
    int              g_step = -1;            // -1 = no test running
    bool             g_releasing = false;    // the standard program's release line is out
    std::atomic<int> g_gen{ 0 };             // a stop / a new test / a load makes older timers void

    // the approach trace (close-contact program): one sample every 250 ms between a request and its result
    bool                                  g_traceOn = false;
    std::chrono::steady_clock::time_point g_traceT0;
    float                                 g_traceLastCc = -1000.f;
    std::int32_t                          g_traceLast[4] = { -99, -99, -99, -99 };
    std::uint32_t                         g_traceLastPkg = 0;
    RE::NiPoint3                          g_traceLastW, g_traceLastT, g_traceLastP;
    std::uint32_t                         g_pairHitsAtStep = 0;
    bool                                  g_firstWalkDumped = false;   // the CONTACT REPORT before the first walker step is out

    void Say(const std::string& a_msg, bool a_corner = false)
    {
        logger::info("[TEST] {}", a_msg);
        if (auto* c = RE::ConsoleLog::GetSingleton()) c->Print("%s", a_msg.c_str());
        if (a_corner) RE::SendHUDMessage::ShowHUDMessage(a_msg.c_str());
    }

    // log only (the capsule tables and the trace would flood the console)
    void Note(const std::string& a_msg)
    {
        logger::info("[TEST] {}", a_msg);
    }

    std::string NameOf(RE::TESObjectREFR* a_ref)
    {
        if (!a_ref) return "?";
        const char* n = a_ref->GetDisplayFullName();
        return std::format("{} [{:08X}]", n && n[0] ? n : "(no name)", a_ref->GetFormID());
    }

    RE::Actor* WalkerActor() { return g_walker ? RE::TESForm::LookupByID<RE::Actor>(g_walker) : nullptr; }
    RE::Actor* TargetActor() { return g_target ? RE::TESForm::LookupByID<RE::Actor>(g_target) : nullptr; }

    // The pair rule as it stands for the walker and the target right now (v0.9.3: the DRIVER arms and releases it).
    std::string PairRuleText()
    {
        int entry = -1, uses = 0;
        bool pending = false;
        const std::uint32_t groups = standpoint_pair::ArmedFor(g_walker, g_target, entry, pending, uses);
        if (!groups) return "NOT armed for these two";
        return std::format("ARMED for these two (pool entry #{}, groups {} / {}, {} use(s)){}", entry, groups >> 16,
                           groups & 0xFFFF, uses, pending ? ", release ordered - off once they are 90 u apart" : "");
    }

    void SendStep();
    void OnResult(std::uint32_t a_handle, const StandpointAPI::MoveResult& r, void* a_user);
    bool MixRunning();                       // 1.6.0: the stpm run (below) is on
    void MixStopBy(const char* a_what);      // ... stop it (both walkers released)

    // Run a_fn on the main thread after a_seconds, unless the test changed meanwhile.
    void After(float a_seconds, void (*a_fn)())
    {
        const int gen = g_gen.load();
        std::thread([a_seconds, a_fn, gen]() {
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(a_seconds * 1000.f)));
            if (auto* task = SKSE::GetTaskInterface()) {
                task->AddTask([a_fn, gen]() {
                    if (gen == g_gen.load()) a_fn();
                });
            }
        }).detach();
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  THE MEASURING INSTRUMENT (close-contact program). READ-ONLY: nothing below writes to an actor, a controller
    //  or a shape. The walk through the controller's shapes is FollowerBumpGuard's, VR-proven there:
    //  GetCharController() -> GetRigidBody() -> collidable.shape (an hkpListShape) -> its hkpCapsuleShape children;
    //  the capsules live in the controller's own frame, which carries the actor's yaw (FBG report 01 §A1).
    // ═════════════════════════════════════════════════════════════════════════
    struct Cap {
        RE::NiPoint3 a, b;        // the capsule's two end points, world, game units
        RE::NiPoint3 local;       // its middle in the controller's frame (x = right, y = FORWARD, z = up), game units
        float        r{ 0.f };
    };
    struct Other {                // a child of the controller's shape that is not a capsule
        int          type{ -1 };            // hkpShapeType
        bool         hasBox{ false };       // a convex-vertices shape: its cached box, in HER frame, game units
        RE::NiPoint3 half, centre;
    };
    struct BodyDump {
        bool          ok{ false };
        const char*   why{ "" };
        int           n{ 0 };
        int           others{ 0 };          // children that are not capsules (first run: ONE per NPC - her BODY)
        Other         other[4];
        RE::NiPoint3  boundCentre, boundHalf;   // the controller's own bound (what Standpoint's solver reads as her radius)
        Cap           caps[8];
        RE::NiPoint3  pos, ctrl;            // the actor's position / the controller transform's translation (world, units)
        float         yawDeg{ 0.f };
        std::uint32_t filterBody{ 0 };      // the controller body's collision filter word
        int           shapeIs{ -1 };        // which of con->shapes[0..1] the body's live shape is (-1 = neither)
    };

    RE::NiPoint3 ToWorld(const RE::hkTransform& t, const float* v)
    {
        const float* c0 = reinterpret_cast<const float*>(&t.rotation.col0);
        const float* c1 = reinterpret_cast<const float*>(&t.rotation.col1);
        const float* c2 = reinterpret_cast<const float*>(&t.rotation.col2);
        const float* tr = reinterpret_cast<const float*>(&t.translation);
        return { (c0[0] * v[0] + c1[0] * v[1] + c2[0] * v[2] + tr[0]) / kHavokPerUnit,
                 (c0[1] * v[0] + c1[1] * v[1] + c2[1] * v[2] + tr[1]) / kHavokPerUnit,
                 (c0[2] * v[0] + c1[2] * v[1] + c2[2] * v[2] + tr[2]) / kHavokPerUnit };
    }

    void AddCaps(BodyDump& d, const RE::hkpShape* a_shape, const RE::hkTransform& t, int a_depth)
    {
        if (!a_shape || a_depth > 2) return;
        if (a_shape->type == RE::hkpShapeType::kCapsule) {
            if (d.n >= 8) return;
            const auto* c = static_cast<const RE::hkpCapsuleShape*>(a_shape);
            const float* A = reinterpret_cast<const float*>(&c->vertexA);
            const float* B = reinterpret_cast<const float*>(&c->vertexB);
            Cap& e = d.caps[d.n++];
            e.r = c->radius / kHavokPerUnit;
            e.a = ToWorld(t, A);
            e.b = ToWorld(t, B);
            e.local = { (A[0] + B[0]) * 0.5f / kHavokPerUnit, (A[1] + B[1]) * 0.5f / kHavokPerUnit,
                        (A[2] + B[2]) * 0.5f / kHavokPerUnit };
            return;
        }
        if (a_shape->type == RE::hkpShapeType::kList) {
            const auto* list = static_cast<const RE::hkpListShape*>(a_shape);
            const std::int32_t cn = list->childInfo.size();
            for (std::int32_t i = 0; i < cn; ++i) AddCaps(d, list->childInfo[i].shape, t, a_depth + 1);
            return;
        }
        // v0.9.1: say WHAT the other child is. The first run showed one capsule (the forward bumper) plus one child of
        // another type per NPC - her body, the thing the two of them actually met on.
        if (d.others < 4) {
            Other& o = d.other[d.others];
            o.type = static_cast<int>(a_shape->type);
            if (a_shape->type == RE::hkpShapeType::kConvexVertices) {
                const auto* cv = static_cast<const RE::hkpConvexVerticesShape*>(a_shape);
                const float* h = reinterpret_cast<const float*>(&cv->aabbHalfExtents);
                const float* c = reinterpret_cast<const float*>(&cv->aabbCenter);
                o.hasBox = true;
                o.half   = { h[0] / kHavokPerUnit, h[1] / kHavokPerUnit, h[2] / kHavokPerUnit };
                o.centre = { c[0] / kHavokPerUnit, c[1] / kHavokPerUnit, c[2] / kHavokPerUnit };
            }
        }
        ++d.others;
    }

    BodyDump ReadBody(RE::Actor* a)
    {
        BodyDump d;
        if (!a) { d.why = "no actor"; return d; }
        d.pos = a->GetPosition();
        d.yawDeg = a->GetAngleZ() * 57.2957795f;
        // (The engine's Actor::GetCollisionFilterInfo is deliberately NOT called: it is an address-library function this
        // plugin has never used in VR, and the controller BODY's filter word below is the one the collision filter sees.)
        auto* con = a->GetCharController();
        if (!con) { d.why = "no character controller"; return d; }
        d.boundCentre = con->collisionBound.center;
        d.boundHalf   = con->collisionBound.extents;
        RE::hkTransform t{};
        con->GetTransformImpl(t);
        const float* tr = reinterpret_cast<const float*>(&t.translation);
        d.ctrl = { tr[0] / kHavokPerUnit, tr[1] / kHavokPerUnit, tr[2] / kHavokPerUnit };

        const RE::hkpShape* live = nullptr;
        if (auto* rb = con->GetRigidBody()) {
            d.filterBody = rb->collidable.broadPhaseHandle.collisionFilterInfo.filter;
            live = rb->collidable.shape;
        }
        for (int i = 0; i < 2; ++i) {
            auto* bs = con->shapes[i].get();
            const auto* hs = bs ? static_cast<const RE::hkpShape*>(bs->referencedObject.get()) : nullptr;
            if (hs && hs == live) d.shapeIs = i;
            if (!live && hs && i == 0) live = hs;      // no rigid body (a proxy controller): fall back to shapes[0]
        }
        if (!live) { d.why = "the controller has no shape"; return d; }
        AddCaps(d, live, t, 0);
        d.ok = d.n > 0;
        if (!d.ok) d.why = "no capsule in the controller's shape";
        return d;
    }

    float Dot(const RE::NiPoint3& a, const RE::NiPoint3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

    // The shortest distance between two segments (the capsules' axes).
    float SegSegDist(const RE::NiPoint3& p1, const RE::NiPoint3& q1, const RE::NiPoint3& p2, const RE::NiPoint3& q2)
    {
        const RE::NiPoint3 d1 = q1 - p1, d2 = q2 - p2, r = p1 - p2;
        const float a = Dot(d1, d1), e = Dot(d2, d2), f = Dot(d2, r);
        float s = 0.f, t = 0.f;
        constexpr float eps = 1e-6f;
        if (a <= eps && e <= eps) {
            s = t = 0.f;
        } else if (a <= eps) {
            t = std::clamp(f / e, 0.f, 1.f);
        } else {
            const float c = Dot(d1, r);
            if (e <= eps) {
                s = std::clamp(-c / a, 0.f, 1.f);
            } else {
                const float b = Dot(d1, d2);
                const float denom = a * e - b * b;
                s = denom > eps ? std::clamp((b * f - c * e) / denom, 0.f, 1.f) : 0.f;
                t = (b * s + f) / e;
                if (t < 0.f) { t = 0.f; s = std::clamp(-c / a, 0.f, 1.f); }
                else if (t > 1.f) { t = 1.f; s = std::clamp((b - c) / a, 0.f, 1.f); }
            }
        }
        const RE::NiPoint3 c1 = p1 + d1 * s, c2 = p2 + d2 * t, dd = c1 - c2;
        return std::sqrt(Dot(dd, dd));
    }

    void NoteBody(const char* a_who, RE::Actor* a, const BodyDump& d)
    {
        Note(std::format("  CAPS {} {}: pos=({:.1f},{:.1f},{:.1f}) yaw={:.1f} deg | controller at ({:.1f},{:.1f},{:.1f}) | "
                         "controller filter={:08X} layer {} group {} | live shape = shapes[{}]{}",
                         a_who, NameOf(a), d.pos.x, d.pos.y, d.pos.z, d.yawDeg, d.ctrl.x, d.ctrl.y, d.ctrl.z,
                         d.filterBody, d.filterBody & 0x7F, d.filterBody >> 16, d.shapeIs,
                         d.ok ? std::string() : std::format(" | NOT READ: {}", d.why)));
        for (int i = 0; i < d.n; ++i) {
            const Cap& c = d.caps[i];
            const RE::NiPoint3 ax = c.b - c.a;
            Note(std::format("    {} capsule #{}: r={:.1f} u, axis length {:.1f} u, middle in HER frame (right {:+.1f}, FORWARD {:+.1f}, "
                             "up {:+.1f}) | world A=({:.1f},{:.1f},{:.1f}) B=({:.1f},{:.1f},{:.1f})",
                             a_who, i, c.r, std::sqrt(Dot(ax, ax)), c.local.x, c.local.y, c.local.z,
                             c.a.x, c.a.y, c.a.z, c.b.x, c.b.y, c.b.z));
        }
        Note(std::format("    {} controller bound: half-extents ({:.1f}, {:.1f}, {:.1f}) u, centre ({:.1f}, {:.1f}, {:.1f})",
                         a_who, d.boundHalf.x, d.boundHalf.y, d.boundHalf.z, d.boundCentre.x, d.boundCentre.y, d.boundCentre.z));
        for (int i = 0; i < d.others && i < 4; ++i) {
            const Other& o = d.other[i];
            if (o.hasBox) {
                Note(std::format("    {} other child #{}: a convex shape (type {}) - box half-extents in HER frame (right {:.1f}, forward {:.1f}, "
                                 "up {:.1f}) u, centre (right {:+.1f}, FORWARD {:+.1f}, up {:+.1f})",
                                 a_who, i, o.type, o.half.x, o.half.y, o.half.z, o.centre.x, o.centre.y, o.centre.z));
            } else {
                Note(std::format("    {} other child #{}: shape type {} (not measured)", a_who, i, o.type));
            }
        }
    }

    // What the two controllers look like at this moment, and how far apart every pair of their capsules is.
    void DumpContact(const char* a_when)
    {
        auto* w = WalkerActor();
        auto* t = TargetActor();
        if (!w || !t) return;
        const BodyDump bw = ReadBody(w), bt = ReadBody(t);
        const float dx = bw.pos.x - bt.pos.x, dy = bw.pos.y - bt.pos.y;
        const float cc = std::sqrt(dx * dx + dy * dy);
        Note(std::format("  CONTACT REPORT ({}): centre to centre {:.1f} u = {:.0f} cm", a_when, cc, cc * kCmPerUnit));
        if (auto* pc = RE::PlayerCharacter::GetSingleton()) {
            // v0.9.3: where the player stands (the open question of the second run: does his position play a part?)
            const RE::NiPoint3 pp = pc->GetPosition();
            Note(std::format("    player at ({:.0f},{:.0f},{:.0f}): {:.0f} u from the walker, {:.0f} u from the target",
                             pp.x, pp.y, pp.z,
                             std::sqrt((pp.x - bw.pos.x) * (pp.x - bw.pos.x) + (pp.y - bw.pos.y) * (pp.y - bw.pos.y)),
                             std::sqrt((pp.x - bt.pos.x) * (pp.x - bt.pos.x) + (pp.y - bt.pos.y) * (pp.y - bt.pos.y))));
        }
        NoteBody("walker", w, bw);
        NoteBody("target", t, bt);
        if (bw.ok && bt.ok) {
            std::string all;
            float best = 1e9f;
            int bi = -1, bj = -1;
            for (int i = 0; i < bw.n; ++i) {
                for (int j = 0; j < bt.n; ++j) {
                    const float gap = SegSegDist(bw.caps[i].a, bw.caps[i].b, bt.caps[j].a, bt.caps[j].b) - bw.caps[i].r - bt.caps[j].r;
                    all += std::format("  walker#{} x target#{}: {:+.1f}", i, j, gap);
                    if (gap < best) { best = gap; bi = i; bj = j; }
                }
            }
            Note(std::format("    surface gaps in u (+ apart, - overlapping):{}", all));
            Note(std::format("    NEAREST PAIR: walker capsule #{} (r {:.1f}) x target capsule #{} (r {:.1f}) - their surfaces are {:.1f} u {}",
                             bi, bw.caps[bi].r, bj, bt.caps[bj].r, std::fabs(best), best >= 0.f ? "APART" : "OVERLAPPING"));
        }
        std::uint32_t hits = 0, seen = 0;
        standpoint_pair::Stats(hits, seen);
        Note(std::format("    pair rule: {} | since this step began the filter answered IGNORE {} time(s) "
                         "(any controller pair while a rule was armed: {} in total)",
                         PairRuleText(), hits - g_pairHitsAtStep, seen));
    }

    void AiState(RE::Actor* a, std::int32_t& a_proc, std::uint32_t& a_pkg)
    {
        a_proc = -2;
        a_pkg = 0;
        auto* p = a ? a->GetActorRuntimeData().currentProcess : nullptr;
        if (!p) return;
        RE::BSSpinLockGuard lock(p->currentPackage.packageLock);
        a_proc = p->currentPackage.currentProcedureIndex;
        if (auto* k = p->currentPackage.package) a_pkg = k->GetFormID();
    }

    // One line whenever something changed: how far apart, what each AI is running (run-once 32 = the bump reaction,
    // 36 = "movement blocked"), and whether the pair rule was consulted.
    void TraceTick()
    {
        if (!g_traceOn || g_step < 0) return;
        auto* w = WalkerActor();
        auto* t = TargetActor();
        if (!w || !t) return;
        const RE::NiPoint3 pw = w->GetPosition(), pt = t->GetPosition();
        const float cc = std::sqrt((pw.x - pt.x) * (pw.x - pt.x) + (pw.y - pt.y) * (pw.y - pt.y));
        std::int32_t  procW = -2, procT = -2;
        std::uint32_t pkgW = 0, pkgT = 0;
        AiState(w, procW, pkgW);
        AiState(t, procT, pkgT);
        const std::int32_t onceW = standpoint_interrupts::GetRunOncePackageTypeId(w);
        const std::int32_t onceT = standpoint_interrupts::GetRunOncePackageTypeId(t);
        const std::int32_t now[4] = { procW, onceW, procT, onceT };
        // v0.9.1: either of them moving 3 u is a change too (the first run's push of the target showed no line, because
        // the walker advanced as fast as the target gave way and the distance between them stayed the same)
        const float movedW = std::sqrt((pw.x - g_traceLastW.x) * (pw.x - g_traceLastW.x) + (pw.y - g_traceLastW.y) * (pw.y - g_traceLastW.y));
        const float movedT = std::sqrt((pt.x - g_traceLastT.x) * (pt.x - g_traceLastT.x) + (pt.y - g_traceLastT.y) * (pt.y - g_traceLastT.y));
        // v0.9.3: the PLAYER's position is on every line (the open question of the second run: does it play a part in
        // where the engine's walk stops?). A step of half a metre by the player is a change too, so a line also
        // appears when he moves while the walker stands still.
        RE::NiPoint3 pp{};
        if (auto* pc = RE::PlayerCharacter::GetSingleton()) pp = pc->GetPosition();
        const float movedP = std::sqrt((pp.x - g_traceLastP.x) * (pp.x - g_traceLastP.x) + (pp.y - g_traceLastP.y) * (pp.y - g_traceLastP.y));
        const bool changed = now[0] != g_traceLast[0] || now[1] != g_traceLast[1] || now[2] != g_traceLast[2] ||
                             now[3] != g_traceLast[3] || pkgW != g_traceLastPkg || std::fabs(cc - g_traceLastCc) >= 3.f ||
                             movedW >= 3.f || movedT >= 3.f || std::fabs(pw.z - g_traceLastW.z) >= 20.f || movedP >= 35.f;
        if (changed) {
            std::uint32_t hits = 0, seen = 0;
            standpoint_pair::Stats(hits, seen);
            const float secs = std::chrono::duration<float>(std::chrono::steady_clock::now() - g_traceT0).count();
            const float pToW = std::sqrt((pp.x - pw.x) * (pp.x - pw.x) + (pp.y - pw.y) * (pp.y - pw.y));
            Note(std::format("    trace +{:5.2f}s  apart {:6.1f} u | walker at ({:.0f},{:.0f},{:.0f}): procedure {} package {:08X} run-once {} | "
                             "target at ({:.0f},{:.0f}): procedure {} run-once {} | player at ({:.0f},{:.0f}), {:.0f} u from the walker | "
                             "pair-rule answers {}",
                             secs, cc, pw.x, pw.y, pw.z, procW, pkgW, onceW, pt.x, pt.y, procT, onceT, pp.x, pp.y, pToW,
                             hits - g_pairHitsAtStep));
            for (int i = 0; i < 4; ++i) g_traceLast[i] = now[i];
            g_traceLastPkg = pkgW;
            g_traceLastCc = cc;
            g_traceLastW = pw;
            g_traceLastT = pt;
            g_traceLastP = pp;
        }
        After(0.25f, &TraceTick);
    }

    void StartTrace()
    {
        g_traceOn = true;
        g_traceT0 = std::chrono::steady_clock::now();
        g_traceLastCc = -1000.f;
        for (auto& v : g_traceLast) v = -99;
        g_traceLastPkg = 0;
        g_traceLastW = {};
        g_traceLastT = {};
        g_traceLastP = {};
        std::uint32_t hits = 0, seen = 0;
        standpoint_pair::Stats(hits, seen);
        g_pairHitsAtStep = hits;
        TraceTick();
    }

    // v0.9.3: the test no longer arms the pair rule. Every close request toward an NPC gets it from the driver
    // (Standpoint.PairArm at the start of the session, PairRelease at its end); the "off only once the two are 90 u
    // apart" logic that lived here in v0.9.2 is the pair module's own now (Interrupts.cpp, ReleaseWhenApart).

    // ═════════════════════════════════════════════════════════════════════════

    void Finish(const std::string& a_why)
    {
        Say(std::format("Standpoint test: {}.", a_why), true);
        g_traceOn = false;
        g_step = -1;
        g_releasing = false;
        ++g_gen;
    }

    // A release line (where = 0) for a_who. Its answer comes back through OnResult like any other.
    void SendRelease(RE::FormID a_who)
    {
        StandpointAPI::MoveRequest q{};
        q.who = a_who;                                  // where = 0: release
        standpoint_move::Request(q, &OnResult, reinterpret_cast<void*>(static_cast<std::intptr_t>(g_gen.load())));
    }

    void OnResult(std::uint32_t a_handle, const StandpointAPI::MoveResult& r, void* a_user)
    {
        const int gen = static_cast<int>(reinterpret_cast<std::intptr_t>(a_user));
        Say(std::format("  RESULT #{}: 1 who={:08X}  2 where={:08X}  3 dist={:.0f} cm  4 facing={:.1f}  5 orient={:.1f}  6 outcome={}  7 detail={:.0f}",
                        a_handle, r.who, r.where, r.distanceCm, r.facing, r.orientation, r.outcome, r.detail));
        if (gen != g_gen.load() || g_step < 0) return;   // a result of a stopped test

        if (g_releasing) {
            Finish(r.outcome == 'X' ? "finished - the release line answered X as it should"
                                    : "finished - but the release line did not answer X (see the log)");
            return;
        }
        const Step& s = g_prog[g_step];
        const bool walkerToTarget = s.who == kWalkerMoves && s.aim == kAtTarget;
        if (walkerToTarget) {
            RE::SendHUDMessage::ShowHUDMessage(std::format("Standpoint test {}/{}: {}  {:.0f} cm  at {:.1f} o'clock, looking {:.1f}",
                                                           g_step + 1, g_progN, r.outcome, r.distanceCm, r.facing, r.orientation).c_str());
        } else {
            RE::SendHUDMessage::ShowHUDMessage(std::format("Standpoint test {}/{}: {}", g_step + 1, g_progN, r.outcome).c_str());
        }
        if (g_closeProg && walkerToTarget) {
            g_traceOn = false;
            DumpContact("at the result");
        }
        if (s.who == kTargetMoves && s.aim == kHoldPlace && r.outcome != 'C') {
            Say(std::format("  WARNING: the TARGET is NOT held (outcome {}, detail {:.0f}) - she may walk off during the run; it goes on anyway",
                            r.outcome, r.detail));
        }
        if (g_step == g_progN - 1) {
            if (g_closeProg) {
                // the close-contact program ends with the target's release line: this is its answer
                Finish("the close-contact run is finished - the numbers are in Standpoint.log (CONTACT REPORT, trace)");
                return;
            }
            if (r.outcome != 'C') { Finish("finished - the last walk did not complete, so its release was not tried"); return; }
            After(5.f, []() {
                g_releasing = true;
                Say("  RELEASE line: 1 who + 2 where=0 (she should leave the spot now, 55 s before her hold ends)");
                SendRelease(g_walker);
            });
            return;
        }
        // v0.9.4: the pause before the next step is the step's own field (she is HELD meanwhile - hold -1 - so the pause
        // only lets her be seen). A step that did not complete is followed after 2 s, as before; a release line's
        // answer (X) is its normal one.
        const float wait = (r.outcome == 'C' || s.aim == kReleaseLine) ? s.pause : 2.f;
        ++g_step;
        After(wait, &SendStep);
    }

    void SendStep()
    {
        if (g_step < 0 || g_step >= g_progN) return;
        const Step& s = g_prog[g_step];

        // v0.9.3: who moves, and where she is sent
        const RE::FormID mover = s.who == kTargetMoves ? g_target : g_walker;
        if (s.who == kTargetMoves) {
            auto* t = TargetActor();
            if (!t || t->IsPlayerRef()) {
                // the player (or a target that is no actor) cannot be held or released: the step is skipped
                Say(std::format("Standpoint test {}/{}: skipped (the target is the player - nothing to hold or release)", g_step + 1, g_progN));
                if (g_step == g_progN - 1) {
                    Finish("the close-contact run is finished - the numbers are in Standpoint.log (CONTACT REPORT, trace)");
                    return;
                }
                ++g_step;
                SendStep();
                return;
            }
        }

        StandpointAPI::MoveRequest q{};
        q.who = mover;
        if (s.aim == kReleaseLine) {
            Say(std::format("Standpoint test {}/{}: {}", g_step + 1, g_progN, s.what), true);
            Say(std::format("  RELEASE line: 1 who={:08X}  2 where=0", q.who));
            SendRelease(mover);
            return;
        }
        q.where       = s.aim == kHoldPlace ? mover : g_target;
        q.distanceCm  = s.distCm;
        q.facing      = s.facing;
        q.orientation = s.orient;
        q.pose        = s.pose;
        q.holdSeconds = s.hold;
        q.close       = s.close;
        Say(std::format("Standpoint test {}/{}: {}", g_step + 1, g_progN, s.what), true);
        Say(std::format("  REQUEST: 1 who={:08X}  2 where={:08X}  3 dist={:.0f} cm  4 facing={:.0f}  5 orient={:.0f}  6 pose={:.0f}  7 hold={:.0f} s  8 close={:.0f}",
                        q.who, q.where, q.distanceCm, q.facing, q.orientation, q.pose, q.holdSeconds, q.close));
        if (g_closeProg && s.who == kWalkerMoves && s.aim == kAtTarget) {
            if (!g_firstWalkDumped) {
                g_firstWalkDumped = true;
                DumpContact("before the first walk");
            }
            StartTrace();
        }
        standpoint_move::Request(q, &OnResult, reinterpret_cast<void*>(static_cast<std::intptr_t>(g_gen.load())));
    }

    bool ExecTest(const RE::SCRIPT_PARAMETER*, RE::SCRIPT_FUNCTION::ScriptData*, RE::TESObjectREFR*, RE::TESObjectREFR*,
                  RE::Script*, RE::ScriptLocals*, double&, std::uint32_t&)
    {
        // 1.6.0: the stpm run is on: stpt stops it too (stpt stays the one stop key)
        if (MixRunning()) {
            MixStopBy("stpt");
            return true;
        }
        // v1.4.1: the follow program is on: stpt stops it (and does nothing else this time)
        if (standpoint_walk_test::FollowOn()) {
            standpoint_walk_test::StopFollow("stpt");
            return true;
        }
        // v1.4.0: a walk test is running (or holding): stpt stops it
        if (standpoint_walk_test::Running()) {
            standpoint_walk_test::StopRunning();
            Say("Standpoint test: the walk test is stopped - she is let go (X 3).", true);
            return true;
        }
        // a test is running: stpt stops it
        if (g_step >= 0) {
            ++g_gen;
            g_step = -1;
            g_releasing = false;
            g_traceOn = false;
            SendRelease(g_walker);
            // v0.9.3: the close-contact program holds the TARGET too - she gets her own release line.
            // (The pair rule needs nothing here: the walker's release orders it off, once the two are apart.)
            auto* held = g_closeProg ? TargetActor() : nullptr;
            if (held && !held->IsPlayerRef()) {
                SendRelease(g_target);
                Say("Standpoint test: stopped - the walker and the target are released.");
            } else {
                Say("Standpoint test: stopped - the walker is released.");
            }
            return true;
        }
        auto ref = RE::Console::GetSelectedRef();
        if (!ref) {
            Say("Standpoint test: click an NPC in the console first (her FormID shows at the top), then type stpt - she becomes "
                "the walker. Then click the target and type stpt again.");
            return true;
        }
        auto* walker = WalkerActor();
        if (walker && !walker->Is3DLoaded()) walker = nullptr;      // she is gone: pick again
        // no walker yet: the clicked NPC becomes the walker
        if (!walker) {
            auto* a = ref->As<RE::Actor>();
            if (!a || a->IsPlayerRef() || !a->Is3DLoaded()) {
                Say("Standpoint test: no walker yet - click an NPC standing near you and type stpt: she becomes the walker. "
                    "Then click the target and type stpt again.");
                return true;
            }
            g_walker = a->GetFormID();
            Say(std::format("Standpoint test: the walker is {}. Now click the TARGET and type stpt again - another NPC runs the "
                            "CLOSE-CONTACT measuring program, furniture / an object / a marker runs the standard one. "
                            "To pick another walker, click her and type stpt.", NameOf(a)));
            return true;
        }
        // v1.4.0: the two walk programs - only while the player is SNEAKING (nothing below changes when he stands)
        if (auto* pc = RE::PlayerCharacter::GetSingleton(); pc && pc->IsSneaking()) {
            if (ref->GetFormID() == g_walker) {
                standpoint_walk_test::StartWaterWalk(walker);
                return true;
            }
            if (ref->IsPlayerRef()) {
                standpoint_walk_test::StartFrontWalk(walker);
                return true;
            }
        }
        // the walker herself is clicked: clear her
        if (ref->GetFormID() == g_walker) {
            g_walker = 0;
            Say(std::format("Standpoint test: {} is no longer the walker - click an NPC and type stpt to pick one.", NameOf(walker)));
            return true;
        }
        // anything else is the target: run. An actor target = the close-contact program.
        g_target = ref->GetFormID();
        g_closeProg = ref->As<RE::Actor>() != nullptr;
        g_prog = g_closeProg ? kClose : kStandard;
        g_progN = static_cast<int>(g_closeProg ? std::size(kClose) : std::size(kStandard));
        g_step = 0;
        g_releasing = false;
        g_firstWalkDumped = false;
        ++g_gen;
        Say(std::format("Standpoint test ({} program): {} walks around {} in {} steps - close the console. Type stpt again to stop. "
                        "Every line also goes to Standpoint.log in your game's My Games\\...\\SKSE folder",
                        g_closeProg ? "CLOSE-CONTACT" : "standard", NameOf(walker), NameOf(ref.get()), g_progN));
        if (g_closeProg) {
            Say("  The target is told to hold her place first; every walker step is a close request (the pair rule and "
                "the step back and around come with it; nothing slides her any more) and the walker is held between "
                "the steps. Stay at least 4 metres from both of them: the walker is meant to stand inside the target's "
                "space, she walks out to about 1.5 metres from the target on her way round, and FollowerBumpGuard "
                "would shrink whichever of them is nearest to you.");
        }
        SendStep();
        return true;
    }

    // v1.4.1 THE FOLLOW-FRONT COMMAND (its own console command - no sneaking; the program is standpoint_walk_test::StartFollow
    // in WalkApi.cpp). Typed with an NPC clicked: she becomes the walker and the program starts; with nothing clicked (or
    // the player / an object): the current stpt walker; no walker: a one-line hint. Typed while the program runs: it stops.
    const char* g_followName = "the follow command";     // its short name once registered (for the hint lines)

    bool ExecFollow(const RE::SCRIPT_PARAMETER*, RE::SCRIPT_FUNCTION::ScriptData*, RE::TESObjectREFR*, RE::TESObjectREFR*,
                    RE::Script*, RE::ScriptLocals*, double&, std::uint32_t&)
    {
        if (standpoint_walk_test::FollowOn()) {
            standpoint_walk_test::StopFollow(std::format("{} again", g_followName).c_str());
            return true;
        }
        if (g_step >= 0) {
            // a stpt move test drives her through sessions; its release lines would end her follow walks - one test at a time
            Say(std::format("Standpoint follow: a stpt test is running - type stpt to stop it first, then {}.", g_followName));
            return true;
        }
        if (MixRunning()) {
            Say(std::format("Standpoint follow: the stpm run is on - type stpt (or stpm) to stop it first, then {}.", g_followName));
            return true;
        }
        RE::Actor* walker = nullptr;
        if (auto ref = RE::Console::GetSelectedRef()) {
            auto* a = ref->As<RE::Actor>();
            if (a && !a->IsPlayerRef()) {
                if (!a->Is3DLoaded() || a->IsDead()) {
                    Say(std::format("Standpoint follow: {} is not a living NPC near you - click another and type {}.", NameOf(a), g_followName));
                    return true;
                }
                walker = a;
            }
        }
        if (!walker) {
            walker = WalkerActor();
            if (walker && (!walker->Is3DLoaded() || walker->IsDead())) walker = nullptr;
        }
        if (!walker) {
            Say(std::format("Standpoint follow: click an NPC in the console and type {} - she stays 3 m in front of you until stpt.",
                            g_followName));
            return true;
        }
        g_walker = walker->GetFormID();                  // she is the stpt walker from now on too
        standpoint_walk_test::StartFollow(walker);
        return true;
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  1.6.0 `stpm` - THE SIDE-BY-SIDE TEST (see the file header). Test-only: it changes nothing in the walking code; it calls
    //  the PUBLIC Papyrus doors (StandpointAPI.*) through the VM, as a consumer's script would, and listens to the events through
    //  the driver's test ear (StandpointAPI.ApiTestListen -> Standpoint.TestEvent). Main thread (console, timers, VM returns
    //  and events arrive as SKSE tasks).
    // ═════════════════════════════════════════════════════════════════════════
    enum class MixDoor : std::uint8_t { kGoToPoint, kGoTo, kWalkTo, kWalkToWater, kRequestMove, kRelease, kReleaseBoth };
    enum class MixWho : std::uint8_t { kA, kB };
    enum class MixAt : std::uint8_t { kNone, kP1, kP2, kMarker, kTarget };
    enum class MixWait : std::uint8_t { kNone, kArrival, kMoveResult, kWalkResult, kFailed, kWalkBAndArrivalA };

    struct MixStep {
        const char* part;
        MixDoor     door;
        MixWho      who;
        MixAt       at;
        int         relation;   // GoTo: 0 nearest 1 front 2 right 3 back 4 left
        float       distCm;     // RequestMove
        float       hour;       // RequestMove: where around the target
        float       orient;     // RequestMove: her own facing (0 = at the target)
        int         pose;
        float       hold;
        int         close;
        MixWait     wait;
        float       limitS;     // the wait's time limit
        float       nextAfterS; // >= 0: the next step starts this long after this one's call, without waiting (a hand-off)
        float       pauseS;     // after the answer (or the time limit): she is seen standing meanwhile
        bool        needNpc;    // part D: the target must be an NPC (she becomes walker B) - skipped otherwise
        const char* hud;        // the HUD line after "MIX k/N " (checked below: <= 14 words / 75 characters in all)
        const char* what;       // the log marker's detail
    };

    // clang-format off
    constexpr MixStep kMix[] = {
        // A - THE SAME SPOT THREE WAYS (a point 3 m in front of the player, facing him). 1.6.1 (C2): before each door she is walked
        // to P2 and held there (A1s / A3s / A5s), so every door walks her P2 -> P1 - the same start, a real distance (R3 §5)
        { "A1s", MixDoor::kWalkTo,     MixWho::kA, MixAt::kP2,     0, 0.f,   0.f,  0.f, 0, -1.f, 0, MixWait::kWalkResult, 60.f, -1.f, 1.f, false,
          "To the start: she walks to the second spot", "WalkTo P2 (the start spot), hold -1 - held there: GoToPoint walks her P2 -> P1" },
        { "A1", MixDoor::kGoToPoint,   MixWho::kA, MixAt::kP1,     0, 0.f,   0.f,  0.f, 0, -1.f, 0, MixWait::kArrival,    60.f, -1.f, 2.f, false,
          "GoToPoint: the spot 3 m in front of you", "GoToPoint to P1 (3 m in front of the player, on the ground), facing him, hold -1" },
        { "A2", MixDoor::kRelease,     MixWho::kA, MixAt::kNone,   0, 0.f,   0.f,  0.f, 0,  0.f, 0, MixWait::kNone,        0.f, -1.f, 2.f, false,
          "Release: she lets go of the spot", "Release the walker (her session's hold ends)" },
        { "A3s", MixDoor::kWalkTo,     MixWho::kA, MixAt::kP2,     0, 0.f,   0.f,  0.f, 0, -1.f, 0, MixWait::kWalkResult, 60.f, -1.f, 1.f, false,
          "Back to the start: the second spot again", "WalkTo P2 (the start spot), hold -1 - held there: WalkTo walks her P2 -> P1" },
        { "A3", MixDoor::kWalkTo,      MixWho::kA, MixAt::kP1,     0, 0.f,   0.f,  0.f, 0, -1.f, 0, MixWait::kWalkResult, 60.f, -1.f, 2.f, false,
          "WalkTo: the same spot, the walk door", "WalkTo the same P1 (Papyrus WalkTo), facing the player, hold -1" },
        { "A4", MixDoor::kRelease,     MixWho::kA, MixAt::kNone,   0, 0.f,   0.f,  0.f, 0,  0.f, 0, MixWait::kNone,        0.f, -1.f, 2.f, false,
          "Release: she lets go again", "Release the walker (her walk's stand ends)" },
        { "A5s", MixDoor::kWalkTo,     MixWho::kA, MixAt::kP2,     0, 0.f,   0.f,  0.f, 0, -1.f, 0, MixWait::kWalkResult, 60.f, -1.f, 1.f, false,
          "Back to the start once more", "WalkTo P2 (the start spot), hold -1 - held there: RequestMove walks her P2 -> P1" },
        { "A5", MixDoor::kRequestMove, MixWho::kA, MixAt::kMarker, 0, 0.f,   0.f,  0.f, 0, -1.f, 0, MixWait::kMoveResult, 60.f, -1.f, 2.f, false,
          "RequestMove: on a marker at the same spot", "RequestMove ON a heading marker placed at P1 (0 cm, the way it points), hold -1" },
        { "A6", MixDoor::kRelease,     MixWho::kA, MixAt::kNone,   0, 0.f,   0.f,  0.f, 0,  0.f, 0, MixWait::kNone,        0.f, -1.f, 3.f, false,
          "Release: the walker is free", "Release the walker" },
        // B - THE SAME TARGET: GoTo (the solver's sides) vs RequestMove (the clock), the poses
        { "B1", MixDoor::kGoTo,        MixWho::kA, MixAt::kTarget, 1, 0.f,   0.f,  0.f, 0, -1.f, 0, MixWait::kArrival,    60.f, -1.f, 2.f, false,
          "GoTo FRONT of the target", "GoTo the target, FRONT, pose 0, hold -1" },
        { "B2", MixDoor::kGoTo,        MixWho::kA, MixAt::kTarget, 2, 0.f,   0.f,  0.f, 0, -1.f, 0, MixWait::kArrival,    60.f, -1.f, 2.f, false,
          "GoTo RIGHT of the target", "GoTo the target, RIGHT, pose 0, hold -1" },
        { "B3", MixDoor::kGoTo,        MixWho::kA, MixAt::kTarget, 3, 0.f,   0.f,  0.f, 0, -1.f, 0, MixWait::kArrival,    60.f, -1.f, 2.f, false,
          "GoTo BACK of the target", "GoTo the target, BACK, pose 0, hold -1" },
        { "B4", MixDoor::kGoTo,        MixWho::kA, MixAt::kTarget, 4, 0.f,   0.f,  0.f, 0, -1.f, 0, MixWait::kArrival,    60.f, -1.f, 2.f, false,
          "GoTo LEFT of the target", "GoTo the target, LEFT, pose 0, hold -1" },
        { "B5", MixDoor::kGoTo,        MixWho::kA, MixAt::kTarget, 0, 0.f,   0.f,  0.f, 1,  6.f, 0, MixWait::kArrival,    60.f, -1.f, 9.f, false,
          "GoTo NEAREST side, auto pose, a 6 s hold", "GoTo the target, NEAREST, pose 1 (auto: kneel / bend by height), hold 6 s (ends by itself)" },
        { "B6", MixDoor::kRequestMove, MixWho::kA, MixAt::kTarget, 0, 150.f, 12.f, 0.f, 2, -1.f, 0, MixWait::kMoveResult, 60.f, -1.f, 4.f, false,
          "RequestMove: 150 cm at 12 o'clock, kneel", "RequestMove 150 cm, hour 12, orientation 0, pose 2 (kneel), hold -1" },
        { "B7", MixDoor::kRequestMove, MixWho::kA, MixAt::kTarget, 0, 150.f, 3.f,  0.f, 3, -1.f, 0, MixWait::kMoveResult, 60.f, -1.f, 4.f, false,
          "RequestMove: 150 cm at 3 o'clock, bend", "RequestMove 150 cm, hour 3, orientation 0, pose 3 (bend), hold -1" },
        { "B8", MixDoor::kRequestMove, MixWho::kA, MixAt::kTarget, 0, 150.f, 6.f,  6.f, 0, -1.f, 0, MixWait::kMoveResult, 60.f, -1.f, 2.f, false,
          "RequestMove: 150 cm at 6, her back to it", "RequestMove 150 cm, hour 6, orientation 6 (her back to it), pose 0, hold -1" },
        { "B9", MixDoor::kRequestMove, MixWho::kA, MixAt::kTarget, 0, 150.f, 9.f,  3.f, 1, -1.f, 0, MixWait::kMoveResult, 60.f, -1.f, 2.f, false,
          "RequestMove: 150 cm at 9, turned to her right", "RequestMove 150 cm, hour 9, orientation 3, pose 1 (standard wait), hold -1" },
        { "B10", MixDoor::kRequestMove, MixWho::kA, MixAt::kTarget, 0, 45.f, 12.f, 0.f, 0, -1.f, 1, MixWait::kMoveResult, 60.f, -1.f, 2.f, false,
          "RequestMove close: 45 cm in front of the target", "RequestMove CLOSE 45 cm, hour 12, orientation 0, pose 0, hold -1" },
        { "B11", MixDoor::kRelease,    MixWho::kA, MixAt::kNone,   0, 0.f,   0.f,  0.f, 0,  0.f, 0, MixWait::kNone,        0.f, -1.f, 3.f, false,
          "Release: she steps away from the target", "Release the walker" },
        // C - THE HAND-OFFS ON ONE NPC (exact timing: the next call 1-1.5 s into the walk)
        { "C1", MixDoor::kWalkTo,      MixWho::kA, MixAt::kP2,     0, 0.f,   0.f,  0.f, 0, -1.f, 0, MixWait::kNone,        0.f, 1.5f, 0.f, false,
          "WalkTo the second spot; a GoTo takes her soon", "WalkTo P2 (3 m ahead, 1.5 m to the left), hold -1 - GoTo follows after 1.5 s" },
        { "C2", MixDoor::kGoTo,        MixWho::kA, MixAt::kTarget, 1, 0.f,   0.f,  0.f, 0, -1.f, 0, MixWait::kArrival,    60.f, -1.f, 2.f, false,
          "GoTo takes her mid-walk: the walk ends X 6", "GoTo the target FRONT while she walks - expected: the walk's WalkResult X 6, then Arrived" },
        { "C3", MixDoor::kGoToPoint,   MixWho::kA, MixAt::kP2,     0, 0.f,   0.f,  0.f, 0, -1.f, 0, MixWait::kNone,        0.f, 1.5f, 0.f, false,
          "GoToPoint the second spot; a WalkTo takes her soon", "GoToPoint P2, hold -1 - WalkTo follows after 1.5 s" },
        { "C4", MixDoor::kWalkTo,      MixWho::kA, MixAt::kP1,     0, 0.f,   0.f,  0.f, 0, -1.f, 0, MixWait::kWalkResult, 60.f, -1.f, 2.f, false,
          "WalkTo takes her mid-walk: the session fails 3", "WalkTo P1 while her session walks - expected: Failed 3 (the session), then the walk's C" },
        { "C5", MixDoor::kRelease,     MixWho::kA, MixAt::kNone,   0, 0.f,   0.f,  0.f, 0,  0.f, 0, MixWait::kNone,        0.f, -1.f, 2.f, false,
          "Release: her walk's hold ends", "Release after the walk's hold -1 (letter 29's pattern: hold, your animation, Release)" },
        { "C6", MixDoor::kGoToPoint,   MixWho::kA, MixAt::kP2,     0, 0.f,   0.f,  0.f, 0, -1.f, 0, MixWait::kNone,        0.f, 1.0f, 0.f, false,
          "GoToPoint again, released after one second", "GoToPoint P2, hold -1 - Release follows after 1 s" },
        { "C7", MixDoor::kRelease,     MixWho::kA, MixAt::kNone,   0, 0.f,   0.f,  0.f, 0,  0.f, 0, MixWait::kFailed,      8.f, -1.f, 2.f, false,
          "Release mid-walk: Failed 3 expected", "Release while her session walks - expected: Failed 3" },
        // D - TWO NPCs AT ONCE (an NPC target becomes walker B)
        { "D1", MixDoor::kGoToPoint,   MixWho::kA, MixAt::kP1,     0, 0.f,   0.f,  0.f, 0, -1.f, 0, MixWait::kNone,        0.f, 0.3f, 0.f, true,
          "Two at once: the walker to the first spot", "GoToPoint P1 for walker A - walker B starts 0.3 s later" },
        { "D2", MixDoor::kWalkTo,      MixWho::kB, MixAt::kP2,     0, 0.f,   0.f,  0.f, 0, -1.f, 0, MixWait::kWalkBAndArrivalA, 75.f, -1.f, 3.f, true,
          "Two at once: the target walks to the second spot", "WalkTo P2 for walker B (the target) - waits for B's WalkResult AND A's arrival" },
        { "D3", MixDoor::kReleaseBoth, MixWho::kA, MixAt::kNone,   0, 0.f,   0.f,  0.f, 0,  0.f, 0, MixWait::kNone,        0.f, -1.f, 3.f, true,
          "Release both walkers", "Release walker A and walker B" },
        // E - THE WALK INTO WATER (1.6.0: shin -> knee -> thigh in one route; O 23 when there is no water within 20 m)
        { "E1", MixDoor::kWalkToWater, MixWho::kA, MixAt::kNone,   0, 0.f,   0.f,  0.f, 0, 10.f, 0, MixWait::kWalkResult, 75.f, -1.f, 12.f, false,
          "WalkToWater: the nearest wading spot, 10 s hold", "WalkToWater within 20 m, depth 0.15-0.45 x her height, hold 10 s" },
        { "E2", MixDoor::kRelease,     MixWho::kA, MixAt::kNone,   0, 0.f,   0.f,  0.f, 0,  0.f, 0, MixWait::kNone,        0.f, -1.f, 1.f, false,
          "Release: the run is over", "Release the walker - the end" },
    };
    // clang-format on
    constexpr int kMixN = static_cast<int>(std::size(kMix));

    constexpr int Words(std::string_view a_s)
    {
        int  n = 0;
        bool in = false;
        for (const char c : a_s) {
            if (c == ' ') {
                in = false;
            } else if (!in) {
                in = true;
                ++n;
            }
        }
        return n;
    }
    // the user's rule: ONE HUD line under 15 words - "MIX 32/32 " is 2 words and 10 characters of it
    constexpr bool MixHudOk()
    {
        for (const auto& s : kMix) {
            const std::string_view h(s.hud);
            if (h.size() + 10 > 75 || Words(h) + 2 > 14) return false;
        }
        return true;
    }
    static_assert(MixHudOk(), "an stpm HUD line is longer than 14 words or 75 characters");
    static_assert(kMixN <= 99, "the HUD prefix assumes at most 99 steps");

    struct MixEvent {
        std::string  name;
        RE::FormID   who{ 0 };
        std::int32_t handle{ 0 };
        std::string  outcome;
        std::int32_t detail{ 0 };
        float        a{ 0.f }, b{ 0.f }, c{ 0.f };
    };

    struct MixRun {
        bool         on{ false };
        int          step{ -1 };
        RE::FormID   a{ 0 };          // walker A
        RE::FormID   b{ 0 };          // walker B = an NPC target (0 = none: part D is skipped)
        RE::FormID   target{ 0 };
        RE::FormID   marker{ 0 };     // the heading marker placed at P1 for A5 (deleted at the end)
        RE::NiPoint3 p1{}, p2{};
        float        face1{ 0.f }, face2{ 0.f };    // compass degrees: toward the player
        MixWait      wait{ MixWait::kNone };
        bool         waiting{ false };
        int          token{ 0 };      // new for every step: an older step's timer or VM return is void
        std::int32_t handle{ 0 };     // the step's handle (RequestMove / WalkTo / WalkToWater), 0 = not known yet
        RE::FormID   waitWho{ 0 };    // whose WalkResult the step waits for
        bool         gotA{ false }, gotB{ false };
        std::chrono::steady_clock::time_point t0{};
        int          answers{ 0 }, limits{ 0 }, skipped{ 0 }, events{ 0 };
    };
    MixRun           g_mix;
    RE::FormID       g_mixWalker = 0;
    std::atomic<int> g_mixGen{ 0 };       // a stop / a load makes every older timer, VM return and event of the run void
    const char*      g_mixName = "stpm";
    RE::FormID       g_mixMarkerAfterLoad = 0;   // FIX ROUND (R1 NIT 17): a run's heading marker to look for after a load ...
    RE::NiPoint3     g_mixMarkerP1{};             // ... where it stood

    RE::Actor*         MixActor(RE::FormID a_id) { return a_id ? RE::TESForm::LookupByID<RE::Actor>(a_id) : nullptr; }
    RE::TESObjectREFR* MixRef(RE::FormID a_id) { return a_id ? RE::TESForm::LookupByID<RE::TESObjectREFR>(a_id) : nullptr; }
    std::string        MixName(RE::FormID a_id) { return a_id ? NameOf(MixRef(a_id)) : std::string("none"); }

    void MixSendFn(int a_token);

    // Run a_fn(a_token) on the main thread after a_seconds, unless the run was stopped / reloaded meanwhile.
    void MixAfter(float a_seconds, int a_token, void (*a_fn)(int))
    {
        const int gen = g_mixGen.load();
        std::thread([a_seconds, a_token, a_fn, gen]() {
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(a_seconds * 1000.f)));
            if (auto* task = SKSE::GetTaskInterface()) {
                task->AddTask([a_token, a_fn, gen]() {
                    if (gen == g_mixGen.load()) a_fn(a_token);
                });
            }
        }).detach();
    }

    void MixReturned(int a_gen, int a_token, const char* a_door, std::int32_t a_value);

    // the value a Papyrus door returned (a handle, a Bool) - comes back on a VM thread, handed to the main thread
    class MixReturn final : public RE::BSScript::IStackCallbackFunctor {
    public:
        MixReturn(int a_gen, int a_token, const char* a_door) : m_gen(a_gen), m_token(a_token), m_door(a_door) {}
        void operator()(RE::BSScript::Variable a_result) override
        {
            std::int32_t value = 0;
            if (a_result.IsInt()) {
                value = a_result.GetSInt();
            } else if (a_result.IsBool()) {
                value = a_result.GetBool() ? 1 : 0;
            }
            const int   gen = m_gen, token = m_token;
            const char* door = m_door;
            if (auto* task = SKSE::GetTaskInterface()) task->AddTask([gen, token, door, value]() { MixReturned(gen, token, door, value); });
        }
        void SetObject(const RE::BSTSmartPointer<RE::BSScript::Object>&) override {}

    private:
        int         m_gen;
        int         m_token;
        const char* m_door;
    };

    // StandpointAPI.<a_fn>(args) through the VM - exactly what a consumer's script calls
    bool MixCall(const char* a_fn, RE::BSScript::IFunctionArguments* a_args)
    {
        auto* vm = RE::BSScript::Internal::VirtualMachine::GetSingleton();
        bool  sent = false;
        if (vm) {
            RE::BSTSmartPointer<RE::BSScript::IStackCallbackFunctor> cb(new MixReturn(g_mixGen.load(), g_mix.token, a_fn));
            sent = vm->DispatchStaticCall("StandpointAPI", a_fn, a_args, cb);
        }
        delete a_args;
        if (!sent) Note(std::format("  MIX: StandpointAPI.{} could not be dispatched (Standpoint's scripts are not running?)", a_fn));
        return sent;
    }

    void MixListen(bool a_on)
    {
        auto* vm = RE::BSScript::Internal::VirtualMachine::GetSingleton();
        if (!vm) return;
        auto*                                                    args = RE::MakeFunctionArguments(static_cast<bool>(a_on));
        RE::BSTSmartPointer<RE::BSScript::IStackCallbackFunctor> none;
        if (!vm->DispatchStaticCall("StandpointAPI", "ApiTestListen", args, none)) {
            Note("  MIX: StandpointAPI.ApiTestListen could not be dispatched - the run hears no events (only the time limits end its waits)");
        }
        delete args;
    }

    std::string MixStepText(int a_k) { return std::format("MIX {}/{} {}", a_k + 1, kMixN, kMix[a_k].part); }

    void MixFinish(const std::string& a_why, bool a_releaseWalkers)
    {
        if (!g_mix.on) return;
        if (a_releaseWalkers) {
            if (auto* a = MixActor(g_mix.a)) MixCall("Release", RE::MakeFunctionArguments(static_cast<RE::Actor*>(a)));
            if (auto* b = MixActor(g_mix.b)) MixCall("Release", RE::MakeFunctionArguments(static_cast<RE::Actor*>(b)));
        }
        g_mix.on = false;
        g_mix.waiting = false;
        ++g_mixGen;
        MixListen(false);
        if (auto* m = MixRef(g_mix.marker)) {
            m->Disable();
            m->SetDelete(true);
        }
        Say(std::format("Standpoint mix: {} - {} answer(s), {} time limit(s), {} step(s) skipped, {} event(s) heard. The lines are "
                        "[TEST] MIX ... in Standpoint.log.",
                        a_why, g_mix.answers, g_mix.limits, g_mix.skipped, g_mix.events));
        RE::SendHUDMessage::ShowHUDMessage(std::format("MIX over: {} answers, {} time limits", g_mix.answers, g_mix.limits).c_str());
    }

    void MixNextFn(int a_token)
    {
        if (!g_mix.on || a_token != g_mix.token) return;
        ++g_mix.step;
        MixSendFn(g_mix.token);
    }

    void MixLimitFn(int a_token)
    {
        if (!g_mix.on || a_token != g_mix.token || !g_mix.waiting) return;
        g_mix.waiting = false;
        ++g_mix.limits;
        const MixStep& s = kMix[g_mix.step];
        Say(std::format("{} TIME LIMIT ({:.0f} s): no answer - on to the next step", MixStepText(g_mix.step), s.limitS));
        MixAfter(s.pauseS, a_token, &MixNextFn);
    }

    void MixReturned(int a_gen, int a_token, const char* a_door, std::int32_t a_value)
    {
        if (a_gen != g_mixGen.load()) return;
        const bool cur = g_mix.on && a_token == g_mix.token && g_mix.step >= 0 && g_mix.step < kMixN;
        Note(std::format("  MIX: StandpointAPI.{} returned {}{}", a_door, a_value, cur ? "" : " (an earlier step's call)"));
        if (!cur) return;
        const MixDoor d = kMix[g_mix.step].door;
        if (d == MixDoor::kRequestMove || d == MixDoor::kWalkTo || d == MixDoor::kWalkToWater) {
            if (a_value > 0) {
                g_mix.handle = a_value;
            } else if (g_mix.waiting) {
                // the door refused (0): nothing will answer - the step ends now
                g_mix.waiting = false;
                Say(std::format("{} answer: StandpointAPI.{} returned 0 (refused) - on to the next step", MixStepText(g_mix.step), a_door));
                MixAfter(kMix[g_mix.step].pauseS, a_token, &MixNextFn);
            }
        }
    }

    // 1.6.1 (C1; run R3 §1): the event NAME is compared case-insensitively - the driver sends "Failed", but the VM handed
    // "failed" over (a BSFixedString keeps the casing it was first interned with): B1's answer was never seen, the run hung
    bool NameIs(const MixEvent& a_e, std::string_view a_name)
    {
        if (a_e.name.size() != a_name.size()) return false;
        for (std::size_t k = 0; k < a_name.size(); ++k) {
            if (std::tolower(static_cast<unsigned char>(a_e.name[k])) != std::tolower(static_cast<unsigned char>(a_name[k]))) return false;
        }
        return true;
    }

    bool IsArrivalEnd(const MixEvent& a_e)
    {
        if (NameIs(a_e, "Arrived") || NameIs(a_e, "Failed")) return true;
        return NameIs(a_e, "Interrupted") && (a_e.detail == 2 || a_e.detail == 3);   // P 7 / P 8 end the session (interrupts 1)
    }

    void OnMixEvent(const MixEvent& a_e)
    {
        ++g_mix.events;
        Note(std::format("MIX event Standpoint_{}: who={} handle={} outcome={} detail={} values=({:.1f}, {:.1f}, {:.1f}){}", a_e.name,
                         MixName(a_e.who), a_e.handle, a_e.outcome.empty() ? "-" : a_e.outcome, a_e.detail, a_e.a, a_e.b, a_e.c,
                         g_mix.on ? std::format(" [during {}]", MixStepText((std::clamp)(g_mix.step, 0, kMixN - 1))) : std::string(" [no stpm run]")));
        if (!g_mix.on || !g_mix.waiting || g_mix.step < 0 || g_mix.step >= kMixN) return;
        const MixStep& s = kMix[g_mix.step];
        bool           done = false;
        switch (g_mix.wait) {
        case MixWait::kArrival:
            done = a_e.who == g_mix.a && IsArrivalEnd(a_e);
            break;
        case MixWait::kMoveResult:
            done = NameIs(a_e, "MoveResult") && (g_mix.handle > 0 ? a_e.handle == g_mix.handle : a_e.who == g_mix.a);
            break;
        case MixWait::kWalkResult:
            done = NameIs(a_e, "WalkResult") && (g_mix.handle > 0 ? a_e.handle == g_mix.handle : a_e.who == g_mix.waitWho);
            break;
        case MixWait::kFailed:
            done = NameIs(a_e, "Failed") && a_e.who == g_mix.a;
            break;
        case MixWait::kWalkBAndArrivalA:
            if (NameIs(a_e, "WalkResult") && a_e.who == g_mix.b) g_mix.gotB = true;
            if (a_e.who == g_mix.a && IsArrivalEnd(a_e)) g_mix.gotA = true;
            done = g_mix.gotA && g_mix.gotB;
            break;
        default:
            break;
        }
        if (!done) return;
        g_mix.waiting = false;
        ++g_mix.answers;
        const float secs = std::chrono::duration<float>(std::chrono::steady_clock::now() - g_mix.t0).count();
        Say(std::format("{} answer after {:.1f} s: Standpoint_{} {} {} ({:.1f}, {:.1f}, {:.1f})", MixStepText(g_mix.step), secs, a_e.name,
                        a_e.outcome.empty() ? "-" : a_e.outcome, a_e.detail, a_e.a, a_e.b, a_e.c));
        MixAfter(s.pauseS, g_mix.token, &MixNextFn);
    }

    void MixDoorCall(const MixStep& s, RE::Actor* a_who)
    {
        // (MakeFunctionArguments wants VALUES: every argument below is a prvalue of the Papyrus parameter's own type)
        const RE::NiPoint3 at = s.at == MixAt::kP2 ? g_mix.p2 : g_mix.p1;
        const float        face = s.at == MixAt::kP2 ? g_mix.face2 : g_mix.face1;
        RE::TESObjectREFR* target = MixRef(g_mix.target);
        RE::Actor*         who = a_who;
        const auto         f = [](float a_v) { return a_v; };
        const auto         n = [](int a_v) { return static_cast<std::int32_t>(a_v); };
        switch (s.door) {
        case MixDoor::kGoToPoint:   // GoToPoint(Actor, Float x, y, z, Float heading, Float hold, Int interrupts, Int danger, Float leash)
            MixCall("GoToPoint", RE::MakeFunctionArguments(static_cast<RE::Actor*>(who), f(at.x), f(at.y), f(at.z), f(face), f(s.hold), n(1), n(2),
                                                           f(3500.f)));
            break;
        case MixDoor::kGoTo:        // GoTo(Actor, ObjectReference, Int relation, Float standoff, Float hold, Int pose, Int interrupts, Int danger, Float leash)
            MixCall("GoTo", RE::MakeFunctionArguments(static_cast<RE::Actor*>(who), static_cast<RE::TESObjectREFR*>(target), n(s.relation), f(0.f),
                                                      f(s.hold), n(s.pose), n(1), n(2), f(3500.f)));
            break;
        case MixDoor::kWalkTo:      // WalkTo(Actor, Float x, y, z, Float facing, Float hold, Float depthFrac, Float stopSubmerged)
            MixCall("WalkTo", RE::MakeFunctionArguments(static_cast<RE::Actor*>(who), f(at.x), f(at.y), f(at.z), f(face), f(s.hold), f(0.45f), f(0.f)));
            break;
        case MixDoor::kWalkToWater: // WalkToWater(Actor, Float maxMetres, Float depthMin, Float depthMax, Float hold)
            MixCall("WalkToWater", RE::MakeFunctionArguments(static_cast<RE::Actor*>(who), f(20.f), f(0.15f), f(0.45f), f(s.hold)));
            break;
        case MixDoor::kRequestMove: {
            // RequestMove(Actor, ObjectReference, Float dist, Float facing, Float orient, Int pose, Float hold, Int close)
            RE::TESObjectREFR* where = s.at == MixAt::kMarker ? MixRef(g_mix.marker) : target;
            MixCall("RequestMove", RE::MakeFunctionArguments(static_cast<RE::Actor*>(who), static_cast<RE::TESObjectREFR*>(where), f(s.distCm),
                                                             f(s.hour), f(s.orient), n(s.pose), f(s.hold), n(s.close)));
            break;
        }
        case MixDoor::kRelease:
            MixCall("Release", RE::MakeFunctionArguments(static_cast<RE::Actor*>(who)));
            break;
        case MixDoor::kReleaseBoth:
            MixCall("Release", RE::MakeFunctionArguments(static_cast<RE::Actor*>(who)));
            if (auto* b = MixActor(g_mix.b)) MixCall("Release", RE::MakeFunctionArguments(static_cast<RE::Actor*>(b)));
            break;
        }
    }

    void MixSendFn(int a_token)
    {
        if (!g_mix.on || a_token != g_mix.token) return;
        if (g_mix.step >= kMixN) {
            MixFinish("all steps done", false);
            return;
        }
        const int      k = g_mix.step;
        const MixStep& s = kMix[k];
        ++g_mix.token;
        auto* a = MixActor(g_mix.a);
        if (!a || !a->Is3DLoaded() || a->IsDead()) {
            MixFinish(std::format("stopped at {} - the walker {} is not loaded (or dead)", MixStepText(k), MixName(g_mix.a)), false);
            return;
        }
        RE::Actor* who = a;
        if (s.needNpc || s.who == MixWho::kB) {
            auto* b = MixActor(g_mix.b);
            if (!b || !b->Is3DLoaded() || b->IsDead()) {
                ++g_mix.skipped;
                Say(std::format("{} skipped - {}", MixStepText(k),
                                g_mix.b ? "walker B (the target) is not loaded" : "the target is not an NPC (part D needs one)"));
                g_mix.step = k + 1;
                MixAfter(0.2f, g_mix.token, &MixSendFn);
                return;
            }
            if (s.who == MixWho::kB) who = b;
        }
        Say(std::format("{}: {} - {} for {}", MixStepText(k), s.hud, s.what, MixName(who->GetFormID())));
        RE::SendHUDMessage::ShowHUDMessage(std::format("MIX {}/{} {}", k + 1, kMixN, s.hud).c_str());
        g_mix.wait = s.wait;
        g_mix.waiting = s.wait != MixWait::kNone;
        g_mix.handle = 0;
        g_mix.waitWho = who->GetFormID();
        g_mix.gotA = false;
        g_mix.gotB = false;
        g_mix.t0 = std::chrono::steady_clock::now();
        MixDoorCall(s, who);
        if (s.nextAfterS >= 0.f) {
            MixAfter(s.nextAfterS, g_mix.token, &MixNextFn);
        } else if (s.wait == MixWait::kNone) {
            MixAfter(s.pauseS, g_mix.token, &MixNextFn);
        } else {
            MixAfter(s.limitS, g_mix.token, &MixLimitFn);
        }
    }

    // 1.6.1 (C2; run R3 §1 / §9: runs 1 and 3 put P1 UNDER the dining table - no headroom, no navmesh - and A1, A3 and A5 failed on
    // the spot, not on the walk): a spot of the run must be STANDABLE OPEN FLOOR - the planner's own stand probe (a floor and its
    // slope, no brink beside it, headroom, dry) with the navmesh under it (16 u flat / 24 u high, as a steered POINT), on the
    // player's level (40 u), 96 u or more from him (no bump) and 80 u or more from a_other (P2 from P1: a real walk between them).
    // The spot asked first, then the nearest one that passes on rings of 16 u out to 128 u. a_p gets the spot (z = its floor);
    // a_text says what happened (the MIX START line). MAIN THREAD (the probe casts rays; the console command runs there).
    bool MixFindSpot(RE::Actor* a_walker, const RE::NiPoint3& a_player, const RE::NiPoint3* a_other, RE::NiPoint3& a_p, std::string& a_text)
    {
        const RE::NiPoint3 asked = a_p;
        int                tested = 0;
        const auto         test = [&](const RE::NiPoint3& a_c, std::string& a_why) -> bool {
            ++tested;
            if (std::hypot(a_c.x - a_player.x, a_c.y - a_player.y) < 96.f) {
                a_why = "within 96 u of the player";
                return false;
            }
            if (a_other && std::hypot(a_c.x - a_other->x, a_c.y - a_other->y) < 80.f) {
                a_why = "within 80 u of P1";
                return false;
            }
            float                      floorZ = a_c.z, depth = 0.f;
            standpoint_path::ProbeNote note;
            if (!standpoint_path::ProbeStandSpot(a_walker, a_c, 0.05f, floorZ, depth, true, &note)) {
                a_why = note.why.empty() ? std::string("not a place to stand") : note.why;
                return false;
            }
            if (std::fabs(floorZ - a_player.z) > 40.f) {
                a_why = std::format("another level ({:.0f} u from the player's floor)", floorZ - a_player.z);
                return false;
            }
            RE::NiPoint3 sn{ a_c.x, a_c.y, floorZ };
            if (!standpoint::SnapToNavmeshNear(a_walker, sn) || std::hypot(sn.x - a_c.x, sn.y - a_c.y) > 16.f || std::fabs(sn.z - floorZ) > 24.f) {
                a_why = "no navmesh under it";
                return false;
            }
            a_p = RE::NiPoint3{ a_c.x, a_c.y, floorZ };
            return true;
        };
        std::string firstWhy;
        if (test(asked, firstWhy)) {
            a_text = "open floor as placed";
            return true;
        }
        const float toPlayer = std::atan2(a_player.x - asked.x, a_player.y - asked.y);
        for (int ring = 1; ring <= 8; ++ring) {
            const float r = 16.f * static_cast<float>(ring);
            const int   n = ring == 1 ? 8 : ring == 2 ? 12 : 16;
            const float step = 6.2831853f / static_cast<float>(n);
            for (int k = 0; k < n; ++k) {
                // (toward the player first, then alternating to both sides)
                const float        ang = toPlayer + ((k % 2) ? -1.f : 1.f) * static_cast<float>((k + 1) / 2) * step;
                const RE::NiPoint3 c{ asked.x + std::sin(ang) * r, asked.y + std::cos(ang) * r, asked.z };
                std::string        why;
                if (test(c, why)) {
                    a_text = std::format("MOVED {:.0f} u from ({:.0f},{:.0f}) to the nearest open floor - there: {}", r, asked.x, asked.y, firstWhy);
                    return true;
                }
            }
        }
        a_text = std::format("NO open floor within 128 u of ({:.0f},{:.0f}) ({} spot(s) tested) - there: {}", asked.x, asked.y, tested, firstWhy);
        return false;
    }

    void MixStart(RE::Actor* a_walker, RE::TESObjectREFR* a_target)
    {
        auto* pc = RE::PlayerCharacter::GetSingleton();
        if (!pc) return;
        // the spots: on the ground in front of the player (his feet's height), fixed for the whole run
        const float        yaw = pc->GetAngleZ();
        const RE::NiPoint3 pp = pc->GetPosition();
        const RE::NiPoint3 fwd{ std::sin(yaw), std::cos(yaw), 0.f };
        const RE::NiPoint3 left{ -std::cos(yaw), std::sin(yaw), 0.f };
        // 1.6.1 (C2): both spots are STANDABLE open floor (MixFindSpot) - P2 is laid out from where P1 ended up; none = no run
        RE::NiPoint3 p1{ pp.x + fwd.x * 210.f, pp.y + fwd.y * 210.f, pp.z };
        std::string  p1Text, p2Text;
        const bool   p1Ok = MixFindSpot(a_walker, pp, nullptr, p1, p1Text);
        RE::NiPoint3 p2{ p1.x + left.x * 105.f, p1.y + left.y * 105.f, pp.z };
        const bool   p2Ok = p1Ok && MixFindSpot(a_walker, pp, &p1, p2, p2Text);
        if (!p1Ok || !p2Ok) {
            Say(std::format("Standpoint mix NOT started: {} - {}. Face open floor (3 m ahead, 1.5 m to its left) and type {} again.",
                            p1Ok ? "P2" : "P1", p1Ok ? p2Text : p1Text, g_mixName));
            Note(std::format("MIX NOT STARTED: the player at ({:.0f},{:.0f},{:.0f}) yaw {:.0f} deg; P1 {}{}", pp.x, pp.y, pp.z, yaw * 57.2957795f,
                             p1Text, p1Ok ? std::format("; P2 {}", p2Text) : std::string()));
            RE::SendHUDMessage::ShowHUDMessage("MIX not started: no open floor in front of you");
            return;
        }
        g_mix = MixRun{};
        g_mix.p1 = p1;
        g_mix.p2 = p2;
        const auto bearingTo = [&](const RE::NiPoint3& a_from) {
            float deg = std::atan2(pp.x - a_from.x, pp.y - a_from.y) * 57.2957795f;
            if (deg < 0.f) deg += 360.f;
            return deg >= 359.95f ? 0.f : deg;
        };
        g_mix.face1 = bearingTo(g_mix.p1);
        g_mix.face2 = bearingTo(g_mix.p2);
        g_mix.a = a_walker->GetFormID();
        g_mix.target = a_target->GetFormID();
        if (auto* ta = a_target->As<RE::Actor>(); ta && !ta->IsPlayerRef() && ta != a_walker) g_mix.b = ta->GetFormID();
        // A5's marker: a heading marker (Skyrim.esm XMarkerHeading 0x34) at P1, pointing at the player; deleted at the end
        std::string markerText = "none (XMarkerHeading not found)";
        if (auto* mb = RE::TESForm::LookupByID<RE::TESBoundObject>(0x34)) {
            if (auto mref = pc->PlaceObjectAtMe(mb, false)) {
                mref->SetPosition(g_mix.p1);
                mref->SetAngle(RE::NiPoint3{ 0.f, 0.f, g_mix.face1 * 0.0174532925f });
                g_mix.marker = mref->GetFormID();
                markerText = std::format("{:08X} ({})", g_mix.marker, mref->Get3D() ? "it has 3D: solved as an object" : "no 3D: a bare marker");
            }
        }
        g_mix.on = true;
        g_mix.step = 0;
        ++g_mixGen;
        MixListen(true);
        Say(std::format("Standpoint mix ({} steps, ~12 min): walker A = {}, target = {}{} - close the console, stand still, touch nothing. "
                        "Type {} again (or stpt) to stop.",
                        kMixN, NameOf(a_walker), NameOf(a_target), g_mix.b ? " (an NPC: she is walker B in part D)" : " (not an NPC: part D is skipped)",
                        g_mixName));
        Note(std::format("MIX START: P1 ({:.0f},{:.0f},{:.0f}) facing {:.0f} deg, P2 ({:.0f},{:.0f},{:.0f}) facing {:.0f} deg ({:.0f} u apart), the "
                         "player at ({:.0f},{:.0f},{:.0f}) yaw {:.0f} deg; marker {}; steered sessions {}",
                         g_mix.p1.x, g_mix.p1.y, g_mix.p1.z, g_mix.face1, g_mix.p2.x, g_mix.p2.y, g_mix.p2.z, g_mix.face2,
                         std::hypot(g_mix.p1.x - g_mix.p2.x, g_mix.p1.y - g_mix.p2.y), pp.x, pp.y, pp.z, yaw * 57.2957795f, markerText,
                         standpoint_steer::Enabled() ? "ON (Standpoint.ini [Session] Steered = 1)" : "OFF (the engine's walk)"));
        Note(std::format("MIX START: P1 {}; P2 {} (the stand probe + the navmesh, 1.6.1 C2)", p1Text, p2Text));
        for (int k = 0; k < kMixN; ++k) Note(std::format("MIX plan {}: {}", MixStepText(k), kMix[k].what));
        RE::SendHUDMessage::ShowHUDMessage(std::format("MIX: {} steps - stand still, close the console", kMixN).c_str());
        MixAfter(2.0f, g_mix.token, &MixSendFn);
    }

    // the stpm console command (a third command, its own donor list)
    bool ExecMix(const RE::SCRIPT_PARAMETER*, RE::SCRIPT_FUNCTION::ScriptData*, RE::TESObjectREFR*, RE::TESObjectREFR*, RE::Script*,
                 RE::ScriptLocals*, double&, std::uint32_t&)
    {
        if (g_mix.on) {
            MixFinish(std::format("stopped ({} again)", g_mixName), true);
            return true;
        }
        if (g_step >= 0 || standpoint_walk_test::FollowOn() || standpoint_walk_test::Running()) {
            Say(std::format("Standpoint mix: a stpt / stpf test is running - type stpt to stop it first, then {}.", g_mixName));
            return true;
        }
        auto  ref = RE::Console::GetSelectedRef();
        auto* walker = MixActor(g_mixWalker);
        if (walker && (!walker->Is3DLoaded() || walker->IsDead())) walker = nullptr;
        if (!ref) {
            Say(std::format("Standpoint mix: click an NPC in the console and type {} - she is the walker; then click the target (a chair, an "
                            "object or an NPC) and type {} again.", g_mixName, g_mixName));
            return true;
        }
        if (!walker) {
            auto* a = ref->As<RE::Actor>();
            if (!a || a->IsPlayerRef() || !a->Is3DLoaded() || a->IsDead()) {
                Say(std::format("Standpoint mix: no walker yet - click a living NPC near you and type {}.", g_mixName));
                return true;
            }
            g_mixWalker = a->GetFormID();
            Say(std::format("Standpoint mix: the walker is {}. Now click the TARGET (a chair, an object or an NPC) and type {} again. To pick "
                            "another walker, click her and type {}.", NameOf(a), g_mixName, g_mixName));
            return true;
        }
        if (ref->GetFormID() == g_mixWalker) {
            g_mixWalker = 0;
            Say(std::format("Standpoint mix: {} is no longer the walker - click an NPC and type {} to pick one.", NameOf(walker), g_mixName));
            return true;
        }
        if (ref->IsPlayerRef()) {
            Say("Standpoint mix: the player cannot be the target - click a chair, an object or an NPC.");
            return true;
        }
        MixStart(walker, ref.get());
        return true;
    }

    // class Standpoint: the driver's test ear (Standpoint_PlayerAlias.OnStpm*) hands every Standpoint_* event here (a VM thread)
    void TestEventNative(RE::StaticFunctionTag*, std::string a_event, RE::TESForm* a_who, std::int32_t a_handle, std::string a_outcome,
                         std::int32_t a_detail, float a_a, float a_b, float a_c)
    {
        MixEvent e;
        e.name = std::move(a_event);
        e.who = a_who ? a_who->GetFormID() : 0;
        e.handle = a_handle;
        e.outcome = std::move(a_outcome);
        e.detail = a_detail;
        e.a = a_a;
        e.b = a_b;
        e.c = a_c;
        if (auto* task = SKSE::GetTaskInterface()) task->AddTask([e]() { OnMixEvent(e); });
    }

    bool MixRunning() { return g_mix.on; }

    void MixStopBy(const char* a_what) { MixFinish(std::format("stopped ({})", a_what), true); }

}  // namespace

void OnGameLoad()
{
    ++g_gen;
    g_step = -1;
    g_releasing = false;
    g_traceOn = false;
    g_walker = 0;
    g_target = 0;
    g_firstWalkDumped = false;
    // 1.6.0: the stpm run ends quietly (no Papyrus call while the world is torn down: the driver's Setup switches its ear off,
    // the walkers' sessions are gone with the load); the walker is forgotten (a fresh check after a load)
    if (g_mix.on) {
        logger::info("[TEST] MIX stopped: a game is being loaded");
        // FIX ROUND (review R1 NIT 17): its heading marker goes too - after the load, if a save made during the run brings it back
        // (OnPostLoad: the same FormID, an XMarkerHeading standing on P1). 1.6.1 (review R1 § VERIFY N3): nothing is done to it
        // HERE - the load discards the world being torn down, and no world change belongs in that moment
        g_mixMarkerAfterLoad = g_mix.marker;
        g_mixMarkerP1 = g_mix.p1;
    }
    g_mix.on = false;
    g_mix.waiting = false;
    ++g_mixGen;
    g_mixWalker = 0;
    standpoint_pair::ClearAll();    // the world is being rebuilt: all four pair-rule slots off at once
    standpoint_walk_test::OnGameLoad();
}

bool RegisterNatives(RE::BSScript::IVirtualMachine* a_vm)
{
    a_vm->RegisterFunction("TestEvent", "Standpoint", TestEventNative);    // 1.6.0: the stpm test's ear
    return true;
}

// FIX ROUND (review R1 NIT 17): kPostLoadGame - a save made during an stpm run brings its heading marker back: it is deleted (only
// when the FormID names an XMarkerHeading still standing on the run's P1 - FF FormIDs are recycled, nothing else is touched)
void OnPostLoad()
{
    const RE::FormID id = g_mixMarkerAfterLoad;
    g_mixMarkerAfterLoad = 0;
    if (!id) return;
    auto* m = RE::TESForm::LookupByID<RE::TESObjectREFR>(id);
    if (!m || m->IsDeleted()) return;
    const auto* base = m->GetBaseObject();
    if (!base || base->GetFormID() != 0x34) return;
    const RE::NiPoint3 p = m->GetPosition();
    if (std::hypot(p.x - g_mixMarkerP1.x, p.y - g_mixMarkerP1.y) > 2.f) return;
    m->Disable();
    m->SetDelete(true);
    logger::info("[TEST] MIX: the heading marker {:08X} of the run before the load was in the loaded save - deleted", id);
}

namespace {
    struct Cmd { const char* longName; const char* shortName; const char* help; RE::SCRIPT_FUNCTION::Execute_t* fn; };

    // The first donor of a_donors that still carries its vanilla name becomes a_cmd. None left = logged, skipped (a donor
    // already renamed - by Standpoint or by any other plugin - is never found by its vanilla name, so never taken twice).
    bool TakeDonor(const Cmd& a_cmd, const char* const* a_donors, std::size_t a_count)
    {
        RE::SCRIPT_FUNCTION* slot = nullptr;
        const char*          donor = nullptr;
        for (std::size_t d = 0; d < a_count && !slot; ++d) {
            slot = RE::SCRIPT_FUNCTION::LocateConsoleCommand(a_donors[d]);
            donor = a_donors[d];
            if (!slot) {
                logger::info("[TEST] console donor '{}' for '{}' not found (taken by another plugin, or not in this game's table)",
                             a_donors[d], a_cmd.shortName);
            }
        }
        if (!slot) {
            logger::warn("[TEST] no free console command left for '{}' - the command is unavailable", a_cmd.shortName);
            return false;
        }
        slot->functionName        = a_cmd.longName;
        slot->shortName           = a_cmd.shortName;
        slot->helpString          = a_cmd.help;
        slot->referenceFunction   = false;          // the ref + zero-param combo dies in the console compiler (CollViz v2)
        slot->SetParameters();
        slot->executeFunction     = a_cmd.fn;
        slot->conditionFunction   = nullptr;
        slot->editorFilter        = false;
        slot->invalidatesCellList = false;
        logger::info("[TEST] console command '{}' ({}) registered on the unused vanilla '{}'", a_cmd.shortName, a_cmd.longName, donor);
        return true;
    }
}  // namespace

void RegisterConsole()
{
    // An unused vanilla console command, re-purposed (the po3 / CollViz pattern; KNOWLEDGEBASE "Console-command DONOR SLOT
    // REGISTRY"). A donor is taken only while it still has its vanilla name.
    // ⚠ v0.8.2: ONE command, and NEVER one of 'TestDegrade', 'TestHandleManagerWarnAndKillSDM', 'ToggleDecompressToCopy' -
    // the three slots the VRCD AddOn's ddab / vrcd / vrcw land on (it registers AFTER Standpoint, so whatever Standpoint
    // takes from its list is gone for it). Measured 2026-10-09: of the whole shared candidate list only those three and
    // 'DumpConditionFunctions' exist un-renamed in this load order (PPB holds 'ToggleTrijuice' + 'StopTrackPlayerDoors').
    // (v1.4.1: the same list, registered first and alone as before - stpt's donor never changes.)
    static constexpr const char* kDonors[] = { "DumpConditionFunctions", "TestCameraPath" };
    const Cmd stpt{ "StandpointTestMove", "stpt",
                    "Select an NPC, stpt = she is the walker; select a target, stpt = she is sent around it through Standpoint's move API (again = stop)",
                    &ExecTest };
    TakeDonor(stpt, kDonors, std::size(kDonors));

    // v1.4.1 THE FOLLOW-FRONT COMMAND - its OWN donor list, never one of stpt's above, never one on another plugin's list.
    // From the 2026-11-03 census of this load order (report "1.4.1 Test Round" S1): all eight are 0-parameter, non-reference
    // entries CommonLibVR can see (index < 436), a release-build stub in SkyrimVR 1.4.15 (`mov al,1 ; ret`: nobody loses a
    // working command), present in the SE 1.5.97 table, and named by no plugin, script or config found - the first five in
    // order, then the three spares the census cleared the same way.
    static constexpr const char* kFollowDonors[] = { "ExportPerfTrackingData", "ToggleGUIOverlay", "EvalActorTextures",
                                                     "ToggleMultiboundCheck", "PrintAiList",
                                                     "ToggleShadowVolumes", "ToggleGrassUpdate", "ToggleHDRDebug" };
    const Cmd follow{ "StandpointFollowMe", "stpf",
                      "Select an NPC (or keep the stpt walker), stpf = she keeps walking to 3 m in front of you, facing you, until stpt or stpf again",
                      &ExecFollow };
    if (RE::SCRIPT_FUNCTION::LocateConsoleCommand(follow.longName)) {
        logger::warn("[TEST] console command '{}' ({}) already exists - not registered twice", follow.shortName, follow.longName);
    } else if (TakeDonor(follow, kFollowDonors, std::size(kFollowDonors))) {
        g_followName = follow.shortName;
    }

    // 1.6.0 THE SIDE-BY-SIDE TEST - its OWN donor list: the census's three spares (report "1.4.1 Test Round" S1 §1: 0-parameter,
    // non-reference, a release-build stub in SkyrimVR 1.4.15, in the SE 1.5.97 table, named by no plugin / script / config found;
    // ToggleHDRDebug last - its short name THD was the one not scanned). They are also the LAST three of stpf's list above, which
    // registers first and takes the first free one of ITS list (ExportPerfTrackingData today): stpf's donor never changes.
    static constexpr const char* kMixDonors[] = { "ToggleGrassUpdate", "ToggleShadowVolumes", "ToggleHDRDebug" };
    const Cmd mix{ "StandpointMixTest", "stpm",
                   "Select an NPC, stpm = she is the walker; select a target (chair / object / NPC), stpm = the side-by-side run (again = stop)",
                   &ExecMix };
    if (RE::SCRIPT_FUNCTION::LocateConsoleCommand(mix.longName)) {
        logger::warn("[TEST] console command '{}' ({}) already exists - not registered twice", mix.shortName, mix.longName);
        return;
    }
    if (TakeDonor(mix, kMixDonors, std::size(kMixDonors))) g_mixName = mix.shortName;
}

}  // namespace standpoint_test
