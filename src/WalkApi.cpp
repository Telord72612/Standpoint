// ─────────────────────────────────────────────────────────────────────────────
//  WalkApi.cpp — v1.4.0: the walk off the navmesh, end to end (see WalkApi.h).
//
//  THE FLOW, per walk (main thread, one step a frame from the pump below):
//    launch    the start checks (DirectWalk::CanTake) -> the planner, all-direct (standpoint_path::StartEx, B §8.3 default
//              flags + kAllDirect)
//    planning  poll GetRoute; a finished plan is copied and freed (PathCancel); "none" answers O 1
//    ready     her Standpoint session (C++ requests only) must be released first (Papyrus ApiWalkFreeSession, confirmed
//              by a stack callback); she moved far since the plan started -> the joining leg is probed (ProbeSegment),
//              one re-plan if it fails; then DirectWalk::Begin
//    driving   the driver reports: arrival (C 0 / C 9 / C 20, then a stand for the hold), a stall (a re-plan from where
//              she stands, the stall spot marked bad first; 1.5.0: after every stall / door / push in a new place - from the
//              same spot twice = O 18), or an end (the letter it maps to)
//    holding   she stands, driven at speed 0, until the hold ends / Release / a new walk / GoTo / a load
//  Every request is answered ONCE. The 90 s limit runs from the request to the answer (a paused game is not counted).
//
//  THREADS: the natives run on Papyrus VM threads: they only add a record, mark a stop or read an answer, under g_mtx.
//  Everything that touches the game runs in the frame step (main thread). The frame step holds no lock while it calls
//  the planner, the driver or the game.
//
//  1.5.0 (report "1.5.0 Plan" D §3):
//    * the MIX: every walk plans with kNavGuide | kDoors | kActorsKeepOut on top of 1.4.x's flags (the navmesh suggests the
//      way, rays check every leg; unlocked ordinary doors are opened by her; standing actors are kept clear of);
//    * a stall in a NEW place, a door that did not open, a push off the line -> a new route from where she stands, again and
//      again (the 90 s bound it) - but never twice from one spot (within 16 u of any earlier re-plan's start): then O 18;
//    * THE WADE-SPOT WALK (WalkToWater / IStandpoint4::RequestWalkToWater): Standpoint asks Water Finder (soft link, its C++
//      revision 2; revision 3's candidates when present) for the seed, the planner's WADE-SPOT search ends at the first
//      reachable wading patch, the driver walks her there and she faces out over the water. "None reachable" -> the next
//      candidate, inside the same 90 s. O 23 no water, O 24 no reachable wading spot, B 25 no Water Finder (1.6.0+).
//
//  1.5.1 (report "1.5.1 Plan" D §2 / §3): up to 200 m (beyond: O 1 "too far"), speed by distance, the engine's far phase.
//
//  1.6.0 (report "1.6.0 Plan" D = the spec §1 items 3, 6, 7, 10; S2 §2-§3; S1 §6-§7):
//    * THE ENGINE FAR PHASE IS REMOVED (the slot package at RUN, the far marker, the adoption, the leak guard - the user: "our own
//      path ... Skyrim one is too unpredictable"). Its four Papyrus natives stay registered with neutral answers.
//    * OUR OWN LONG PATH, ROLLING (S2 §2.1-§2.4): a walk whose point lies beyond the planner's reach (2400 u straight) gets the
//      navmesh SKELETON first (NavGraph: A* over the loaded navmesh triangles + a funnel, no physics question), then CHUNK 1
//      (the MIX leg tests on the skeleton's first >= 1,400 u - she starts as soon as it is in), then, while she walks and less
//      than 1,400 u of checked route are left ahead of her, the NEXT CHUNK (<= 2,400 u, from the last checked point) appended to
//      the route she walks (DirectWalk::Extend) - she never stops for it. A chunk that fails: the skeleton again from the last
//      checked point (<= 3 times; the planner remembers the bad leg), else she ends at the last checked point (C 9 / O 1). A
//      point outside the loaded cells = O 26 at once; the loaded edge reached mid-walk = C 9 at the last checked point. A walk
//      within reach is one plan (the planner builds its own skeleton when the straight line fails). A water walk's last piece
//      (within reach of the seed) is the wade-spot search from the last checked point.
//    * RE-PLANS ON THE MOVE (S1 W2 + D3, S2 §2.4 - one mechanism): a moved goal (a steered session's Retarget, an stpf re-aim) is
//      planned from where she will be in 0.5 s along her route while she keeps walking it (or, the new point more than 90 deg
//      off her way, held at speed 0), then swapped in where she is (DirectWalk::Swap). The time limit of a walk beyond 2,100 u:
//      90 s + d0 / 150 u/s.
//    * THE SPEED BANDS (DirectWalk Params::bands, S2 §3) on every walk: the driver gets the unverified rest (remainAfterU).
//    * THE STEERED SESSIONS' WALK (S1 W1-W7): RequestSteer / Retarget / HandOff / HoldHere (WalkApi.h); the frame step calls the
//      steered sessions' own frame (SteerSession.h).
// ─────────────────────────────────────────────────────────────────────────────
#include "PCH.h"
#include "WalkApi.h"
#include "DirectWalk.h"
#include "NavGraph.h"
#include "PathPlanner.h"
#include "SteerSession.h"

#include <array>
#include <atomic>
#include <chrono>
#include <cstring>
#include <deque>
#include <format>
#include <memory>
#include <mutex>
#include <optional>
#include <string_view>
#include <thread>
#include <unordered_map>

namespace logger = SKSE::log;

namespace standpoint_interrupts {
    bool IsPlayerGrabbing(RE::Actor* a_actor);   // Interrupts.cpp: two atomics, safe from any thread (the follow program)
}

namespace standpoint_walk {
namespace {

    using Clock = std::chrono::steady_clock;

    constexpr float       kLimitS       = 90.f;    // Standpoint's existing walk limit (D §1.5: no new timer)
    constexpr float       kDefaultDepth = 0.45f;   // x her height: the deepest water a route may use (B §2)
    constexpr float       kMinDepth     = 0.05f;
    constexpr float       kMaxDepth     = 0.65f;
    constexpr float       kMaxStopSub   = 0.69f;   // a stop level at 0.70 or more could never come before the swim stop
    constexpr float       kDriftProbe   = 48.f;    // she moved this far from the plan's start before the drive: probe the joining leg
    constexpr float       kSkipLead     = 8.f;     // a route point this near where she stands is not walked to
    constexpr float       kArriveRadius = 32.f;    // C 0 only this near the asked point; farther = C 9 with the distance (review H #7)
    constexpr float       kBadSpotS     = 60.f;    // a stall's bad spot lives this long in the planner (B §7.7 default)
    constexpr int         kMaxOpen      = 8;       // walks at once (planning, walking or standing)
    constexpr std::size_t kPapKeep      = 128;     // Papyrus answers kept for the poll natives
    constexpr float       kStepMaxDt    = 0.1f;    // a long frame counts at most this much
    constexpr float       kDegToRad     = 0.0174532925f;
    constexpr float       kRadToDeg     = 57.2957795f;
    // 1.5.0 the re-plan rule (D §3 item 1): no new route from within this of where an earlier one started (no progress;
    // review R2 S7: ANY earlier one of this walk - a stall ping-pong between two spots ends too)
    constexpr float       kReplanSameSpot = 16.f;
    constexpr std::size_t kReplanSpots    = 16;    // the newest re-plan starts kept per walk (a list, not a count limit)
    // 1.5.0 the wade-spot walk
    constexpr float       kWaterRangeM  = 20.f;    // Water Finder looks this far (metres) when the caller says <= 0
    constexpr float       kWadeMinFrac  = 0.15f;   // the shallowest wading spot, x her height, when the caller says <= 0
    constexpr int         kMaxSeeds     = 4;       // Water Finder 1.6.2's GetWaterCandidates hands out at most 4
    // a candidate this near a seed already planned is that seed (review R2 S9: Water Finder's own sameness distance - its
    // candidates of one water lie >= 128 u apart; she may have moved between the first seed and the candidate list)
    constexpr float       kSeedSame     = 128.f;
    constexpr float       kFaceWaterMin = 16.f;    // "face the water" needs a direction at least this long
    // 1.5.1 / 1.6.0 THE RANGE AND THE LIMIT - 1 m = 70 u
    constexpr float       kMaxRangeU     = 14000.f;  // 200 m: further = O 1 "too far (beyond 200 m)"
    constexpr float       kLongLimitFromU = 2100.f;  // a walk beyond 30 m (d0) gets the longer limit ...
    constexpr float       kLongLimitUps  = 150.f;    // ... 90 s + d0 / this (the user's 200 m)
    // 1.6.0 THE ROLLING PLAN (S2 §2.3) and RE-PLANS ON THE MOVE (S2 §2.4)
    constexpr float       kChunkAskBelowU = standpoint_path::kChunkFirstU;   // the next chunk is asked once less checked route than
                                                     // this is left ahead of her (~3 s at a run; a chunk takes 0.05-0.6 s)
    constexpr int         kMaxReskeletons = 3;       // a chunk that fails: the skeleton again from the last checked point, this often
    constexpr float       kRetargetLeadS = 0.5f;     // a re-target is planned from where she will be this much later along her route
    constexpr float       kRetargetLeadMaxS = 2.0f;  // (review R2 #2) ... or the last plan's wall time, up to this
    constexpr float       kRetargetLeadMaxU = 300.f; // (1.6.1, S2's verify NIT b) ... and never farther than this (a late turn otherwise)
    // (1.6.1 B3) the point ahead is "off her floor" when it lies more than this + kAheadDzSlope x the lead above / below her feet (a
    // smooth slope up to 27 deg never counts; a stair's route line can run 65 u under its surface) - the plan starts at her feet then
    constexpr float       kAheadDzPad    = 48.f;
    constexpr float       kAheadDzSlope  = 0.5f;
    constexpr float       kRetargetCoastDeg = 90.f;  // ... while the new point lies within this of her travel direction (else held)
    constexpr float       kCoastMinUps   = 30.f;     // ... and she moves at least this fast (else she is held where she is)
    constexpr float       kWalkUps       = 98.f;     // her walk (Sofia, told 1.0) - the lead when she barely moves
    // (review R2 #12) a re-aim while a re-aim plan runs: the running plan is KEPT when the new point lies within this of the one it
    // plans for (stpf's own trigger is >= 64 u, a head turn moves its point ~150-300 u) - the latest point is planned when it lands;
    // farther = a different place: planned again at once
    constexpr float       kReaimKeepU    = 256.f;
    constexpr float       kReaimSameU    = 16.f;     // ... the landed plan's point this near the latest one = nothing more to plan
    // (review R2 #11) a far water walk's skeleton: the seed may lie this far off the shore's navmesh (else 256, NavGraph's default)
    constexpr float       kWaterJoinMaxU = 512.f;
    // (the four 1.5.1 far natives' neutral answers - the far phase is gone: no far session can exist any more)
    constexpr std::int32_t kEndEvaluate = 2;         // WalkFarEndMode: the override off + EvaluatePackage (never the let-go)
    constexpr std::int32_t kEndLetGo = 0;            // WalkReleaseMode: the session's ordinary release

    // ── 1.5.0 Water Finder's C++ interface (its WaterFinderAPI.h - the user's own public-domain header, made to be copied),
    // re-declared slot by slot (moved here from the stpt test). Soft dependency: SKSE messaging only, asked once at
    // kDataLoaded. Revision 2 (1.6.0) = the seed; revision 3 (1.6.2) = other candidates. Its calls cast rays on the
    // calling thread under the physics world's read lock: made from the frame step (an SKSE task), never from a VM thread.
    struct WfResult {
        bool          found;
        bool          navigable;
        float         x, y, z;            // 1.6.0: the water target - in the water, close to land; z = the bed
        float         distanceMetres;
        float         bearingDegrees;     // 0 = north, clockwise
        char          direction[16];
        char          summary[64];
        std::uint32_t markerFormID;
        std::uint32_t reserved[8];
    };
    class WfApi1 {
    public:
        virtual unsigned int  GetBuildNumber() = 0;                                                      // 0
        virtual bool          FindNearestWater(std::uint32_t a_actor, float a_maxM, bool a_nav, WfResult& a_out) = 0;  // 1
        virtual bool          GetLastResult(std::uint32_t a_actor, WfResult& a_out) = 0;                // 2
        virtual bool          IsWaterAtPosition(float a_x, float a_y, float a_z) = 0;                   // 3
        virtual bool          IsActorInWater(std::uint32_t a_actor) = 0;                                // 4
        virtual std::uint32_t AcquireWaterMarker(std::uint32_t a_actor) = 0;                            // 5
        virtual bool          ReleaseWaterMarker(std::uint32_t a_actor) = 0;                            // 6
        virtual bool          MoveMarkerToWater(std::uint32_t a_actor, std::uint32_t a_marker) = 0;     // 7
    };
    class WfApi2 : public WfApi1 {
    public:
        virtual bool GetWaterSurfacePoint(std::uint32_t a_actor, float& a_x, float& a_y, float& a_surfaceZ) = 0;   // 8 (1.6.0)
    };
    class WfApi3 : public WfApi2 {
    public:
        // 9 (1.6.2): up to a_maxCount (<= 4) seeds from DIFFERENT water bodies / directions, nearest first, each a water target
        // as FindNearestWater gives it (x, y, the bed z) into a_xyz[3 * i ..]; returns the count
        virtual int GetWaterCandidates(std::uint32_t a_actor, float a_maxMetres, float* a_xyz, int a_maxCount) = 0;
    };
    struct WfHandshake {
        enum : unsigned int { kGetInterface = 0x57544652 };   // 'WTFR'
        void* (*GetApiFunction)(unsigned int) = nullptr;
    };
    std::atomic<WfApi2*> g_wf2{ nullptr };     // set once at kDataLoaded (null: absent, or older than 1.6.0)
    std::atomic<WfApi3*> g_wf3{ nullptr };     // ... (null: older than 1.6.2 - one seed only)

    // (1.6.0: the 1.5.1 stage kFar is gone with the far phase)
    enum Stage : int { kLaunch = 0, kPlanning, kReady, kDriving, kReplanning, kHolding, kDone };

    const char* StageName(int a_s)
    {
        switch (a_s) {
        case kLaunch:     return "launch";
        case kPlanning:   return "planning";
        case kReady:      return "ready";
        case kDriving:    return "driving";
        case kReplanning: return "re-planning";
        case kHolding:    return "holding";
        case kDone:       return "done";
        }
        return "?";
    }

    // 1.6.0 what the running skeleton / plan of a walk is for (one at a time)
    enum Job : int {
        kJobNone = 0,
        kJobFirst,      // her first route (stage kPlanning): from where she stands - she stands meanwhile
        kJobReplan,     // a new route after a stall / a push / a shut door (stage kReplanning): she is HELD (paused) meanwhile
        kJobChunk,      // the next verified chunk while she walks (stage kDriving) -> DirectWalk::Extend
        kJobSwap,       // a new route to a moved point while she walks or is held (stage kDriving) -> DirectWalk::Swap
    };

    const char* JobName(int a_j)
    {
        switch (a_j) {
        case kJobNone:   return "none";
        case kJobFirst:  return "the first route";
        case kJobReplan: return "a new route from where she stands";
        case kJobChunk:  return "the next chunk";
        case kJobSwap:   return "a new route on the move";
        }
        return "?";
    }

    struct Walk {
        std::int32_t                id{ 0 };
        RE::FormID                  who{ 0 };
        RE::NiPoint3                goal{};
        float                       facingDeg{ -1.f };       // < 0 = keep the last travel direction
        float                       hold{ -1.f };
        float                       depthFrac{ kDefaultDepth };
        float                       stopSub{ 0.f };
        bool                        papyrus{ false };
        StandpointAPI::WalkCallback cb{ nullptr };
        void*                       user{ nullptr };
        std::uint32_t               gen{ 0 };
        std::atomic<int>            stage{ kLaunch };
        std::atomic<int>            stopDetail{ 0 };          // > 0: a stop was asked (any thread) - the X detail
        std::atomic<bool>           sessionFree{ true };      // her Standpoint session (if any) is released
        // main thread only
        std::int32_t                plan{ 0 };
        Clock::time_point           planT0{};
        RE::NiPoint3                planFrom{};
        std::vector<standpoint_path::RoutePoint> route;
        bool                        partial{ false };         // the plan was PARTIAL, or it moved the goal toward her
        int                         replans{ 0 };
        bool                        driftReplanned{ false };
        bool                        driverHas{ false };       // DirectWalk has her (Begin succeeded, no final report yet)
        bool                        answered{ false };
        float                       active{ 0.f };            // seconds toward the 90 s limit
        bool                        waitLogged{ false };
        std::int32_t                inheritId{ 0 };           // an older walk's driver walker that HOLDS her until this one's
                                                              // Begin takes her over (review H #8); stopped if this walk ends first
        RE::FormID                  faceWho{ 0 };             // v1.4.1 (the stpt follow program only): face this reference at the
                                                              // arrival and while she stands (DirectWalk Params::faceWho)
        EndNoticeFn                 notice{ nullptr };        // v1.4.1 internal walks: every end (the answer + the stand's end)
        const char*                 endWord{ "" };            // ... with the driver's word for it (a static string)
        // 1.5.0 the re-plan rule (audit C fix 2): a new route after every stall / door / push in a NEW place
        std::vector<RE::NiPoint3>   replanFroms;              // (review R2 S7) where EVERY re-plan of this walk started (the
                                                              // newest kReplanSpots): none again within kReplanSameSpot of any
        bool                        replanStray{ false };     // the running re-plan follows a push (its "no way" answers O 15)
        bool                        replanDrifted{ false };   // (review R2 S6) the running re-plan's one drift re-plan is used
        // 1.5.0 THE WADE-SPOT WALK (WalkToWater / IStandpoint4::RequestWalkToWater)
        bool                        water{ false };
        float                       maxMetres{ kWaterRangeM };// how far Water Finder looks
        float                       wadeMin{ kWadeMinFrac };  // the shallowest wading spot, x her height (depthFrac = the deepest)
        bool                        faceWater{ false };       // she ends facing out over the water (the caller's facing < 0)
        RE::NiPoint3                seed{};                   // the seed being planned (a Water Finder water target: the bed)
        std::vector<RE::NiPoint3>   tried;                    // the seeds planned so far
        std::vector<RE::NiPoint3>   cands;                    // Water Finder 1.6.2's other candidates (asked once, at the first "none")
        bool                        candsAsked{ false };
        bool                        wadeNoneSeen{ false };    // a plan answered "no reachable wading spot"
        // 1.5.1 SPEED BY DISTANCE (D §2) - main thread
        RE::NiPoint3                startPos{};               // where she stood at the launch
        float                       d0{ 0.f };                // her straight (3D) distance to the point there (a water walk: to the seed)
        float                       limitS{ kLimitS };        // 90 s; a walk beyond 30 m: 90 s + d0 / 150 u/s
        // (verify V NEW-2) the driver had her in this walk at least once (every report's everDrove; asked before a Stop)
        bool                        drove{ false };
        // ── 1.6.0 THE ROLLING PLAN / RE-PLANS ON THE MOVE (main thread) ──
        int                         job{ kJobNone };          // what the running skeleton / plan (w.skel / w.plan) is for
        std::int32_t                skel{ 0 };                // a NavGraph skeleton job running (0 = none)
        Clock::time_point           skelT0{};
        bool                        jobUseFrom{ false };      // the job plans from jobFrom (a point ahead of her / the last checked
        RE::NiPoint3                jobFrom{};                // point) - else from where she stands
        RE::NiPoint3                jobTo{};                  // ... to here
        std::vector<standpoint_nav::SkelPoint> jobSkel;       // the skeleton its plan was given (chunk mode)
        bool                        jobWade{ false };         // its plan is the wade-spot search (a water walk's last piece)
        bool                        chunked{ false };         // the route she has ends at a checked point with more route after it
        std::vector<standpoint_nav::SkelPoint> skelRest;      // ... the skeleton after that point (the next chunk's corridor)
        float                       remainAfter{ 0.f };       // ... its length (the driver's Params::remainAfterU)
        RE::NiPoint3                verifiedEnd{};            // the last point of the route the driver has
        int                         chunks{ 0 };              // chunks handed to the driver (the first included)
        int                         reskels{ 0 };             // skeletons run again after a failed chunk (<= kMaxReskeletons)
        int                         retargets{ 0 };           // re-aims on the move (Retarget, an stpf re-aim, the next water seed)
        bool                        swapHeld{ false };        // the running swap job holds her at speed 0 (the new point was off her way)
        Clock::time_point           jobT0{};                  // (review R2 #2) when the running job started ...
        float                       lastJobS{ 0.f };          // ... and how long the last one took, s (the coast's lead)
        bool                        reaimPending{ false };    // (review R2 #12) the point moved again while a re-aim plan ran (kept):
                                                              // planned to the LATEST point when that plan lands
        // an end decided before she gets there (the loaded edge C 9, no way on O 1, no wading spot O 24): she walks the checked
        // route to its end and her arrival answers with it
        char                        endLetter{ 0 };
        std::int32_t                endDetail{ 0 };
        std::string                 endWhy;
        // ── 1.6.0 A STEERED SESSION'S WALK (S1 W1 / W2 / W7) ──
        bool                        steer{ false };
        std::int32_t                steerId{ 0 };
        SteerNoticeFn               steerNotice{ nullptr };
        RE::NiPoint3                planTo{};                 // the planner's goal (S, or the approach point E); w.goal = the spot S
        std::vector<RE::NiPoint3>   tail;                     // fixed points after the planned route (E -> S)
        std::vector<RE::NiPoint3>   fixedRoute;               // non-empty: no plan - she walks these (the orbit)
        RE::FormID                  avoid{ 0 };               // the planner's keep-out circle
        std::uint32_t               planFlags{ 0 };           // 0 = the MIX; else these (+ kAllDirect)
        float                       faceOffsetDeg{ 0.f };     // D1
        bool                        faceAll{ false };         // D2
        float                       nearRadius{ 0.f };        // > 0: the near rule
        bool                        nearHit{ false };         // the near rule made her arrival
        bool                        holdHere{ false };        // W7: a stand-only drive at her feet (no plan)
        bool                        handedOff{ false };       // W3: the session took her over (word "session")
        bool                        lastHolding{ false };     // (the notice) the last answer leaves her standing, driven
        float                       lastFaceErr{ 0.f };       // (the notice) her facing error at the arrival
    };
    using WalkPtr = std::shared_ptr<Walk>;

    // " (STEER #n)" for a steered session's walk - every new line of such a walk names it (spec D §4)
    std::string SteerTag(const Walk& w)
    {
        return w.steer ? std::format(" (STEER #{})", w.steerId) : std::string();
    }

    struct PapLine {
        char         letter{ '-' };
        std::int32_t detail{ 0 };
        float        dist{ 0.f }, sub{ 0.f }, x{ 0.f }, y{ 0.f }, z{ 0.f };
    };

    std::int32_t SeedId()
    {
        const auto t = static_cast<std::uint64_t>(std::chrono::system_clock::now().time_since_epoch().count());
        return 1 + static_cast<std::int32_t>((t / 7u) % 0x00FFF000u);
    }

    std::mutex                                 g_mtx;        // records, ids, Papyrus answers
    std::unordered_map<std::int32_t, WalkPtr>  g_walks;
    std::int32_t                               g_nextId = SeedId();
    std::unordered_map<std::int32_t, PapLine>  g_pap;
    std::deque<std::int32_t>                   g_papOrder;
    std::atomic<std::uint32_t>                 g_gen{ 1 };   // +1 at every load: an older Papyrus walk is never answered

    std::atomic<RE::FormID>                    g_watchWho{ 0 };       // v1.4.1: the follow program's walker (0 = none) ...
    std::atomic<std::uint32_t>                 g_watchForeign{ 0 };   // ... requests / stops for her that were not its own

    void CountForeign(RE::FormID a_who)
    {
        if (a_who && a_who == g_watchWho.load()) g_watchForeign.fetch_add(1);
    }

    std::atomic<bool>                          g_pumpOn{ false };
    std::atomic<bool>                          g_stepQueued{ false };
    Clock::time_point                          g_lastStep{};
    bool                                       g_haveLastStep = false;

    float Dist2D(const RE::NiPoint3& a_a, const RE::NiPoint3& a_b) { return std::hypot(a_a.x - a_b.x, a_a.y - a_b.y); }

    RE::Actor* LookupActor(RE::FormID a_id) { return a_id ? RE::TESForm::LookupByID<RE::Actor>(a_id) : nullptr; }

    std::string ActorText(RE::Actor* a_actor, RE::FormID a_id)
    {
        if (!a_actor) return std::format("[{:08X}]", a_id);
        const char* n = a_actor->GetDisplayFullName();
        return std::format("{} [{:08X}]", n && n[0] ? n : "(no name)", a_id);
    }

    WalkPtr Find(std::int32_t a_id)
    {
        std::scoped_lock lk(g_mtx);
        const auto it = g_walks.find(a_id);
        return it == g_walks.end() ? nullptr : it->second;
    }

    // ── the answer ───────────────────────────────────────────────────────────────────────────────────────────────
    void DeliverPapyrus(const Walk& w, const StandpointAPI::WalkResult& r)
    {
        if (w.gen != g_gen.load()) return;
        if (r.outcome == 'X' && static_cast<std::int32_t>(r.detail) == StandpointAPI::kDetailGameLoaded) return;
        {
            std::scoped_lock lk(g_mtx);
            const auto it = g_pap.find(w.id);
            if (it == g_pap.end()) return;
            it->second = PapLine{ r.outcome, static_cast<std::int32_t>(r.detail), r.distLeft, r.submerged, r.x, r.y, r.z };
        }
        auto* vm = RE::BSScript::Internal::VirtualMachine::GetSingleton();
        if (!vm) return;
        auto* args = RE::MakeFunctionArguments(static_cast<std::int32_t>(w.id), static_cast<std::int32_t>(w.who),
                                               RE::BSFixedString(std::string(1, r.outcome)), static_cast<std::int32_t>(r.detail),
                                               static_cast<float>(r.distLeft), static_cast<float>(r.submerged));
        RE::BSTSmartPointer<RE::BSScript::IStackCallbackFunctor> none;
        if (!vm->DispatchStaticCall("StandpointAPI", "ApiWalkDone", args, none)) {
            logger::warn("WALK #{}: StandpointAPI.ApiWalkDone could not be dispatched - the answer is only kept for GetWalkOutcome", w.id);
        }
        delete args;
    }

    // where she is now, into a result line (distLeft to the asked point, submerged, x / y / z)
    void MeasureInto(const Walk& w, StandpointAPI::WalkResult& r)
    {
        if (auto* a = LookupActor(w.who); a && a->Is3DLoaded()) {
            const RE::NiPoint3 p = a->GetPosition();
            r.distLeft = Dist2D(p, w.goal);
            if (auto* cell = a->GetParentCell()) r.submerged = a->GetSubmergedLevel(p.z, cell);
            r.x = p.x;
            r.y = p.y;
            r.z = p.z;
        }
    }

    // v1.4.1: an internal walk's notice, queued (main thread, a later frame); a_word is a static string
    // (1.6.0: + a steered session's walk - its SteerNotice carries the steer id, holding, the near rule, the facing error)
    void QueueNotice(const Walk& w, const StandpointAPI::WalkResult& r, bool a_holdEnd)
    {
        auto* task = SKSE::GetTaskInterface();
        if (!task) return;
        if (w.steerNotice) {
            SteerNotice n{};
            n.steerId = w.steerId;
            n.walkId = static_cast<std::uint32_t>(w.id);
            n.r = r;
            n.word = w.endWord;
            n.holdEnd = a_holdEnd;
            n.holding = !a_holdEnd && w.lastHolding;
            n.nearRule = !a_holdEnd && w.nearHit;
            n.facingErrDeg = w.lastFaceErr;
            const auto  fn = w.steerNotice;
            void* const user = w.user;
            task->AddTask([fn, n, user]() { fn(n, user); });
            return;
        }
        if (!w.notice) return;
        const auto  fn = w.notice;
        void* const user = w.user;
        const auto  h = static_cast<std::uint32_t>(w.id);
        const char* word = w.endWord;
        task->AddTask([fn, h, r, word, a_holdEnd, user]() { fn(h, r, word, a_holdEnd, user); });
    }

    // v1.4.1: the end of the STAND after an internal walk's answer (the API's walks get no second answer: only the log line)
    void NoticeHoldEnd(const Walk& w, char a_letter, std::int32_t a_detail)
    {
        if (!w.notice && !w.steerNotice) return;
        StandpointAPI::WalkResult r{};
        r.who = w.who;
        r.outcome = a_letter;
        r.detail = static_cast<float>(a_detail);
        MeasureInto(w, r);
        QueueNotice(w, r, true);
    }

    // ONE answer per walk. Main thread. a_measure = false while a load tears the world down.
    void Answer(Walk& w, char a_letter, std::int32_t a_detail, const std::string& a_why, bool a_measure = true)
    {
        if (w.answered) return;
        w.answered = true;
        StandpointAPI::WalkResult r{};
        r.who = w.who;
        r.outcome = a_letter;
        r.detail = static_cast<float>(a_detail);
        if (a_measure) MeasureInto(w, r);
        logger::info("WALK #{}{} RESULT {} {} - {}: {:.0f} u from the asked point (flat), submerged {:.2f}, at ({:.0f},{:.0f},{:.0f}) "
                     "after {:.1f} s{}",
                     w.id, SteerTag(w), a_letter, a_detail, a_why, r.distLeft, r.submerged, r.x, r.y, r.z, w.active,
                     w.papyrus     ? " (Papyrus: Standpoint_WalkResult)" :
                     w.cb          ? " (C++ callback)" :
                     w.steerNotice ? " (the steered session's notice)" : " (the internal notice)");
        if (w.papyrus) {
            DeliverPapyrus(w, r);
        } else if (w.cb) {
            const auto cb = w.cb;
            void* const user = w.user;
            const auto h = static_cast<std::uint32_t>(w.id);
            if (auto* task = SKSE::GetTaskInterface()) task->AddTask([cb, h, r, user]() { cb(h, r, user); });
        }
        // v1.4.1: an internal walk's answer (nothing for the API's walks); 1.6.0: none at a load (a_measure false) - the steered
        // sessions and the follow program forget their walks there themselves
        if (a_measure) QueueNotice(w, r, false);
    }

    // the driver's End -> (letter, detail). a_atStart = a refusal before she was ever driven.
    void MapEnd(standpoint_direct::End a_end, bool a_atStart, char& a_letter, std::int32_t& a_detail)
    {
        using E = standpoint_direct::End;
        switch (a_end) {
        case E::kArrived:      a_letter = 'C'; a_detail = StandpointAPI::kDetailClean; return;
        case E::kDepthReached: a_letter = 'C'; a_detail = StandpointAPI::kDetailDepthReached; return;
        case E::kBlocked:      a_letter = 'O'; a_detail = StandpointAPI::kDetailBlocked; return;
        case E::kStrayed:      a_letter = 'O'; a_detail = StandpointAPI::kDetailStrayed; return;
        case E::kWouldSwim:    a_letter = 'O'; a_detail = StandpointAPI::kDetailWouldSwim; return;
        case E::kBadPath:      a_letter = 'O'; a_detail = StandpointAPI::kDetailNoSlot; return;
        case E::kGrabbed:      a_letter = 'P'; a_detail = StandpointAPI::kDetailPlayerGrab; return;
        case E::kOtherDriver:  a_letter = 'B'; a_detail = StandpointAPI::kDetailOtherDriver; return;
        case E::kNotSupported: a_letter = 'B'; a_detail = StandpointAPI::kDetailNotSupported; return;
        case E::kTableFull:    a_letter = 'B'; a_detail = StandpointAPI::kDetailNotReady; return;
        case E::kGone:
        case E::kDead:
            if (a_atStart) { a_letter = 'O'; a_detail = StandpointAPI::kDetailBadRequest; return; }
            a_letter = 'X'; a_detail = StandpointAPI::kDetailEnded; return;
        default:               a_letter = 'X'; a_detail = StandpointAPI::kDetailEnded; return;
        }
    }

    // ── the planner ──────────────────────────────────────────────────────────────────────────────────────────────
    // (1.6.0: a_wade false for a water walk = the MIX - the chunks of a long walk toward the water; a steered session's walk takes
    //  its own flags, kAllDirect always added)
    standpoint_path::PlanOptions OptionsFor(const Walk& w, bool a_wade = true)
    {
        standpoint_path::PlanOptions o;
        o.tag = static_cast<std::uint32_t>(w.id);
        if (w.steer && w.planFlags != 0u) {
            o.flags = w.planFlags | standpoint_path::kAllDirect;
        } else if (w.water && a_wade) {
            // 1.5.0 the WADE-SPOT search (D §2 item 8, Y §6 step 3): from her toward the seed, ending at the first reachable
            // water cell that is a wading patch. No kPartial (a dry ledge top is never the answer: none = status 0), no
            // kGoalMayMove (no straight march toward her), no kNavGuide (in this search the navmesh stays a label and a cost -
            // Y §6 item 7)
            o.flags = standpoint_path::kFloor | standpoint_path::kWater | standpoint_path::kAllDirect | standpoint_path::kWadeSpot |
                      standpoint_path::kDoors | standpoint_path::kActorsKeepOut;
            o.wadeMinFrac = w.wadeMin;
        } else {
            // 1.5.0 THE MIX (D §3 item 2): 1.4.x's kFloor | kWater | kPartial | kGoalMayMove | kAllDirect (kNavFirst was ignored
            // with kAllDirect) + the navmesh corridor as the guide, doors opened, standing actors kept clear of
            o.flags = standpoint_path::kFloor | standpoint_path::kWater | standpoint_path::kPartial | standpoint_path::kGoalMayMove |
                      standpoint_path::kAllDirect | standpoint_path::kNavGuide | standpoint_path::kDoors |
                      standpoint_path::kActorsKeepOut;
        }
        o.depthFrac = w.depthFrac;
        return o;
    }

    const char* PlanStatusName(int a_s)
    {
        switch (a_s) {
        case 2:  return "ROUTE";
        case 1:  return "STRAIGHT";
        case 3:  return "PARTIAL";
        case 0:  return "NONE";
        case -1: return "FAILED";
        case -2: return "UNKNOWN";
        }
        return "?";
    }

    // the point a walk's plans aim at: a water walk's seed, a steered walk's planTo (S or the approach point E), else the point
    RE::NiPoint3 TargetOf(const Walk& w)
    {
        return w.water ? w.seed : w.steer ? w.planTo : w.goal;
    }

    // 1.6.0: the running skeleton / plan is dropped (a newer job replaces it, or the walk ends)
    void CancelJob(Walk& w)
    {
        if (w.plan > 0) standpoint_path::Cancel(w.plan);
        if (w.skel > 0) standpoint_nav::Cancel(w.skel);
        w.plan = 0;
        w.skel = 0;
        w.job = kJobNone;
        w.jobSkel.clear();
        w.jobWade = false;
        w.reaimPending = false;      // (a newer job plans to the latest point itself)
    }

    // one planner job of the running walk job: a_skeleton non-null = CHUNK MODE along it (a_chunkMin / a_chunkMax); a_wade = a water
    // walk's wade-spot search (else the MIX); the plan starts at w.jobFrom when w.jobUseFrom. false = the planner refused it.
    bool StartPlanStep(Walk& w, RE::Actor* a_actor, const std::vector<standpoint_nav::SkelPoint>* a_skeleton, float a_chunkMin,
                       float a_chunkMax, bool a_wade, const char* a_why)
    {
        standpoint_path::PlanOptions o = OptionsFor(w, a_wade);
        if (a_skeleton) {
            o.skeleton = *a_skeleton;
            o.chunkMinU = a_chunkMin;
            o.chunkMaxU = a_chunkMax;
        }
        if (w.jobUseFrom) {
            o.useFrom = true;
            o.from = w.jobFrom;
        }
        auto* const        avoid = w.avoid ? RE::TESForm::LookupByID<RE::TESObjectREFR>(w.avoid) : nullptr;
        const RE::NiPoint3 to = w.jobTo;
        const std::int32_t h = standpoint_path::StartEx(a_actor, to, avoid, o);
        if (h <= 0) return false;
        w.plan = h;
        w.planT0 = Clock::now();
        w.planFrom = w.jobUseFrom ? w.jobFrom : a_actor->GetPosition();
        w.jobSkel = a_skeleton ? *a_skeleton : std::vector<standpoint_nav::SkelPoint>{};
        w.jobWade = a_wade && w.water;
        logger::info("WALK #{}{} PLAN asked ({}; {}): plan #{} from ({:.0f},{:.0f},{:.0f}){} to ({:.0f},{:.0f},{:.0f}){}, flags 0x{:X}, depth {:.2f} x "
                     "her height{}{}{}",
                     w.id, SteerTag(w), a_why, JobName(w.job), h, w.planFrom.x, w.planFrom.y, w.planFrom.z,
                     w.jobUseFrom ? " [a point ahead of her / the last checked point]" : "", to.x, to.y, to.z,
                     w.jobWade ? " [Water Finder's seed: the WADE-SPOT search]" : w.water ? " [toward Water Finder's seed: the MIX]" : "",
                     o.flags, o.depthFrac,
                     w.jobWade ? std::format(", a wading spot {:.2f}-{:.2f} x her height with {} of its 8 neighbours wet", o.wadeMinFrac,
                                             o.depthFrac, o.wadePatchMin)
                               : std::string(),
                     a_skeleton ? std::format(" | CHUNK MODE: {} skeleton point(s), {:.0f}-{:.0f} u of it", a_skeleton->size(), a_chunkMin,
                                              a_chunkMax)
                                : std::string(),
                     avoid ? std::format(" | avoid [{:08X}] (its keep-out circle)", avoid->GetFormID()) : std::string());
        return true;
    }

    // 1.6.0 A WALK JOB (S2 §2.1): from where she stands (or w.jobFrom) to a_to - beyond the planner's reach (2400 u straight) the
    // navmesh SKELETON first (NavGraph, its own pump), then chunk mode along it; within reach ONE plan (a water walk: the wade-spot
    // search). false = refused (the planner's / NavGraph's table is full, she has no 3D): the caller answers.
    bool StartJob(Walk& w, RE::Actor* a_actor, int a_kind, bool a_useFrom, const RE::NiPoint3& a_from, const RE::NiPoint3& a_to,
                  const char* a_why)
    {
        CancelJob(w);
        if (!a_actor) return false;
        w.job = a_kind;
        w.jobUseFrom = a_useFrom;
        w.jobFrom = a_from;
        w.jobTo = a_to;
        w.jobT0 = Clock::now();
        const RE::NiPoint3 start = a_useFrom ? a_from : a_actor->GetPosition();
        const float        flat = Dist2D(start, a_to);
        if (flat > standpoint_path::kRangeV15) {
            standpoint_nav::SkelOptions so;
            so.tag = static_cast<std::uint32_t>(w.id);
            // (review R2 #11) water triangles keep their x4 cost on a far water walk's skeleton - the chunks follow the land; its last
            // piece is the wade-spot search from the last checked point. The seed may lie up to 512 u off the shore's navmesh
            so.water = false;
            if (w.water) so.joinMax = kWaterJoinMaxU;
            const std::int32_t h = standpoint_nav::BuildSkeleton(a_actor, start, a_to, so);
            if (h <= 0) {
                w.job = kJobNone;
                return false;
            }
            w.skel = h;
            w.skelT0 = Clock::now();
            w.planFrom = start;
            logger::info("WALK #{}{} SKELETON asked ({}; {}): job #{} from ({:.0f},{:.0f},{:.0f}) to ({:.0f},{:.0f},{:.0f}) - {:.0f} u straight, "
                         "beyond the planner's {:.0f} u reach: the navmesh skeleton first, then verified chunks", w.id, SteerTag(w), a_why,
                         JobName(a_kind), h, start.x, start.y, start.z, a_to.x, a_to.y, a_to.z, flat, standpoint_path::kRangeV15);
            return true;
        }
        if (!StartPlanStep(w, a_actor, nullptr, 0.f, 0.f, true, a_why)) {
            w.job = kJobNone;
            return false;
        }
        return true;
    }

    // the walk's first route / a new route from where she stands (stage kPlanning / kReplanning)
    bool StartPlan(Walk& w, RE::Actor* a_actor, int a_kind, const char* a_why)
    {
        return StartJob(w, a_actor, a_kind, false, {}, TargetOf(w), a_why);
    }

    const char* SkelSourceName(int a_s)
    {
        switch (a_s) {
        case 1: return "the walk's skeleton";
        case 2: return "its own skeleton (NavGraph)";
        case 3: return "the 1.5.x grid corridor";
        }
        return "none";
    }

    void LogPlan(const Walk& w, const standpoint_path::PlanInfo& a_info, const std::vector<standpoint_path::RoutePoint>& a_pts, double a_ms)
    {
        logger::info("WALK #{}{} PLAN status {} ({}) for {}, {} point(s), switch {}, {} questions, {:.0f} ms wall, H {:.0f} u, depth cap {:.0f} u, "
                     "length {:.0f} u, goal moved {:.0f} u - {}{}{}{}",
                     w.id, SteerTag(w), a_info.status, PlanStatusName(a_info.status), JobName(w.job), a_pts.size(), a_info.switchIndex,
                     a_info.questions, a_ms, a_info.height, a_info.depthCap, a_info.length, a_info.goalMovedBy, a_info.why,
                     !w.jobWade        ? std::string() :
                     a_info.wadeNone   ? std::string(" | WADE: no reachable wading spot") :
                                         std::format(" | WADE: the spot is {:.0f} u deep, {} of its 8 neighbours wet (stage {}: {}; the entry "
                                                     "{:.0f} u deep)", a_info.wadeDepth, a_info.wadePatch, a_info.wadeStage,
                                                     a_info.wadeStage == 2 ? "scan 2 found thigh" : a_info.wadeStage == 1 ? "scan 1 went deeper"
                                                                                                                         : "the entry",
                                                     a_info.wadeEntryDepth),
                     // 1.5.1 (the planner's appended PlanInfo fields): the riser on every floor, the cone riser (land on the navmesh
                     // only), the terrain allowance, the cell cache
                     a_info.riser > 0.f ? std::format(" | 1.5.1: riser {:.0f} u (land on the navmesh: {:.0f} u), terrain allowance {}, "
                                                      "cell cache answers {}", a_info.riser, a_info.riserTerrain, a_info.terrainSteps,
                                                      a_info.cacheHits)
                                        : std::string(),
                     // 1.6.0: the corridor, the chunk, the stairs, the corners
                     std::format(" | 1.6.0: corridor {} ({:.0f} u){}, stair allowance {}, kPtNoCut corners {}{}", SkelSourceName(a_info.skelSource),
                                 a_info.skelLength,
                                 a_info.chunked ? std::format(", CHUNK{} - {} skeleton point(s) taken, {:.0f} u of skeleton after it",
                                                              a_info.chunkFinal ? " (the last: it reaches the point)" : "", a_info.skelTaken,
                                                              a_info.remainAfterU)
                                                : std::string(),
                                 a_info.stairSteps, a_info.noCutPoints, a_info.outsideLoaded ? ", the point is OUTSIDE the loaded area" : ""));
        for (std::size_t k = 0; k < a_pts.size(); ++k) {
            const auto& p = a_pts[k];
            logger::info("WALK #{}   point {}: ({:.0f},{:.0f},{:.0f}) kind {} depth {:.0f} flags 0x{:X}{}{}{}{}{}", w.id, k, p.pos.x, p.pos.y,
                         p.pos.z, static_cast<int>(p.kind), p.depth, p.flags,
                         (p.flags & standpoint_path::kPtDoor) ? std::format(" DOOR [{:08X}] on the leg into it", p.door) : std::string(),
                         (p.flags & standpoint_path::kPtWade) ? " WADING SPOT" : "",
                         (p.flags & standpoint_path::kPtStair) ? " STAIR leg" : "",
                         (p.flags & standpoint_path::kPtNoCut) ? " NO-CUT corner" : "",
                         (p.flags & standpoint_path::kPtChunkEnd) ? " CHUNK END" : "");
        }
    }

    // 1.5.0: a water walk's "face the water" - from where her route enters the water to the wading spot (else from where the
    // plan started, else from the spot toward the seed). false = no direction that long: she keeps her last travel direction.
    bool WaterFacing(const Walk& w, float& a_deg)
    {
        if (w.route.empty()) return false;
        const RE::NiPoint3 end = w.route.back().pos;
        RE::NiPoint3       from = w.planFrom;
        for (std::size_t i = w.route.size() - 1; i-- > 0;) {
            if (w.route[i].kind != standpoint_path::Kind::kWater) {
                from = w.route[i].pos;
                break;
            }
        }
        RE::NiPoint3 a = from;
        RE::NiPoint3 b = end;
        if (Dist2D(a, b) < kFaceWaterMin) a = w.planFrom;
        if (Dist2D(a, b) < kFaceWaterMin) {
            a = end;
            b = w.seed;
        }
        if (Dist2D(a, b) < kFaceWaterMin) return false;
        float deg = std::atan2(b.x - a.x, b.y - a.y) * kRadToDeg;
        while (deg < 0.f) deg += 360.f;
        while (deg >= 360.f) deg -= 360.f;
        a_deg = deg;
        return true;
    }

    // one planned point for the driver
    standpoint_direct::PathPoint DrivePoint(const standpoint_path::RoutePoint& a_r)
    {
        standpoint_direct::PathPoint p{};
        p.pos = a_r.pos;
        p.flags = a_r.flags;
        p.kind = static_cast<std::uint8_t>(a_r.kind);
        p.depth = a_r.depth;
        p.door = (a_r.flags & standpoint_path::kPtDoor) ? a_r.door : 0;   // 1.5.0: the driver opens it before the leg into p
        return p;
    }

    // the driver's route: where she stands now, then the planned points (a first point right at her feet is skipped)
    std::vector<standpoint_direct::PathPoint> BuildPath(RE::Actor* a_actor, const std::vector<standpoint_path::RoutePoint>& a_route)
    {
        std::vector<standpoint_direct::PathPoint> out;
        out.reserve(a_route.size() + 1);
        standpoint_direct::PathPoint first{};
        first.pos = a_actor->GetPosition();
        out.push_back(first);
        for (std::size_t k = 0; k < a_route.size(); ++k) {
            const auto& r = a_route[k];
            if (out.size() == 1 && k + 1 < a_route.size() && Dist2D(r.pos, first.pos) < kSkipLead) continue;
            out.push_back(DrivePoint(r));
        }
        return out;
    }

    // 1.6.0: a route that starts at a_from (a chunk from the last checked point, a swap from a point ahead of her): [a_from, the
    // planned points ...] - Extend drops a first point at the route's end, Swap joins it where she is
    std::vector<standpoint_direct::PathPoint> PathFrom(const RE::NiPoint3& a_from, const std::vector<standpoint_path::RoutePoint>& a_route)
    {
        std::vector<standpoint_direct::PathPoint> out;
        out.reserve(a_route.size() + 1);
        standpoint_direct::PathPoint first{};
        first.pos = a_from;
        out.push_back(first);
        for (const auto& r : a_route) out.push_back(DrivePoint(r));
        return out;
    }

    // 1.6.0 (S1 W1) fixed points as route points (the radial last leg, the orbit): floor points with no flags - the caller probed them
    void AppendFixed(std::vector<standpoint_path::RoutePoint>& a_route, const std::vector<RE::NiPoint3>& a_pts)
    {
        for (const auto& p : a_pts) {
            if (!a_route.empty() && Dist2D(a_route.back().pos, p) < 1.f) continue;
            a_route.push_back(standpoint_path::RoutePoint{ p, standpoint_path::Kind::kFloor, 0.f, 0u, 0u });
        }
    }

    // ── the driver's reports (main thread, from inside DirectWalk::Tick) ─────────────────────────────────────────
    void OnDriverReport(std::int32_t a_walkId, const standpoint_direct::Report& a_rep);

    void Done(Walk& w)
    {
        w.driverHas = false;
        CancelJob(w);                    // (1.6.0: a running skeleton too)
        if (w.inheritId) {
            // an older walk was still holding her for this one, which ends before it took her: she is let go now
            standpoint_direct::Stop(w.inheritId, "replaced");
            w.inheritId = 0;
        }
        w.stage.store(kDone);
    }

    // a refusal / end before the driver has her (or after it let her go): answer, done
    void Refuse(Walk& w, standpoint_direct::End a_end, const std::string& a_text, bool a_atStart)
    {
        char         letter = 'X';
        std::int32_t detail = StandpointAPI::kDetailEnded;
        MapEnd(a_end, a_atStart, letter, detail);
        w.endWord = standpoint_direct::EndName(a_end);
        Answer(w, letter, detail, std::format("{}: {}", standpoint_direct::EndName(a_end), a_text));
        Done(w);
    }

    // the driver's Params of this walk (1.6.0: the bands on every walk, the unverified rest, D1 / D2, the steer id)
    standpoint_direct::Params ParamsOf(const Walk& w)
    {
        standpoint_direct::Params prm{};
        prm.walkId = w.id;
        prm.hasFacing = w.facingDeg >= 0.f;
        prm.facingRad = w.facingDeg * kDegToRad;
        prm.holdSeconds = w.hold;
        prm.stopSubmerged = w.stopSub;
        prm.faceWho = w.faceWho;         // v1.4.1 the follow program's "face the player"; 1.6.0 a steered session's target
        prm.faceOffsetRad = w.faceOffsetDeg * kDegToRad;
        prm.faceAll = w.faceAll;
        prm.steerId = w.steerId;
        prm.d0 = w.d0;
        prm.bands = true;                // 1.6.0 (spec D §1 item 7): every steered walk, sessions included
        prm.remainAfterU = w.chunked ? w.remainAfter : 0.f;
        return prm;
    }

    void BeginDrive(Walk& w)
    {
        auto* a = LookupActor(w.who);
        if (!a || !a->Is3DLoaded()) {
            Refuse(w, standpoint_direct::End::kGone, "she is not loaded any more", true);
            return;
        }
        auto        path = BuildPath(a, w.route);
        const float drift = Dist2D(a->GetPosition(), w.planFrom);
        if (drift > kDriftProbe && path.size() >= 2 && !w.holdHere && w.fixedRoute.empty()) {
            const bool ok = standpoint_path::ProbeSegment(a, path[0].pos, path[1].pos, w.depthFrac);
            logger::info("WALK #{}{} she moved since the plan started: {:.0f} u - the joining leg to point 1 {}", w.id, SteerTag(w), drift,
                         ok ? "is clear" : "is NOT clear");
            if (!ok) {
                if (!w.driftReplanned && StartPlan(w, a, kJobFirst, "she moved away from the plan's start")) {
                    w.driftReplanned = true;
                    w.stage.store(kPlanning);
                    return;
                }
                w.endWord = "no-route";
                Answer(w, 'O', StandpointAPI::kDetailNoSlot, "no route: she keeps moving away from where the route starts");
                Done(w);
                return;
            }
        }
        const standpoint_direct::Params prm = ParamsOf(w);
        const RE::NiPoint3              last = path.back().pos;
        standpoint_direct::End          why = standpoint_direct::End::kNone;
        std::string                     text;
        if (!standpoint_direct::Begin(a, std::move(path), prm, &OnDriverReport, why, text)) {
            Refuse(w, why, text, true);
            return;
        }
        w.verifiedEnd = last;
        w.inheritId = 0;            // Begin has taken her over from the older walk that held her (it dropped that walker)
        w.driverHas = true;
        w.stage.store(kDriving);
    }

    void OnDriverReport(std::int32_t a_walkId, const standpoint_direct::Report& a_rep)
    {
        const WalkPtr wp = Find(a_walkId);
        if (!wp || wp->stage.load() == kDone) return;     // a walk already answered and closed (e.g. handed over): ignored
        Walk& w = *wp;
        using E = standpoint_direct::End;
        w.drove = w.drove || a_rep.everDrove;              // (verify V NEW-2)
        if (a_rep.holdEnded) {
            logger::info("WALK #{}{} HOLD END: {} (the answer was given at the arrival)", w.id, SteerTag(w), standpoint_direct::EndName(a_rep.why));
            // v1.4.1: an internal walk is told why her stand ended (the follow program stops when another owner took her)
            char         letter = 'X';
            std::int32_t detail = StandpointAPI::kDetailEnded;
            MapEnd(a_rep.why, false, letter, detail);
            w.endWord = standpoint_direct::EndName(a_rep.why);
            NoticeHoldEnd(w, letter, detail);
            Done(w);
            return;
        }
        if ((a_rep.why == E::kBlocked || a_rep.why == E::kStrayed) && a_rep.paused && w.steer && w.nearRadius > 0.f &&
            Dist2D(a_rep.pos, w.goal) <= w.nearRadius && !w.nearHit) {
            // 1.6.0 THE NEAR RULE (S1 §2.4, W1): a stall / a push within the near zone of the spot is an ARRIVAL - never a re-plan
            // from the target's own body. She settles, faces (faceWho + the offset) and stands held where she stopped: C 9, word
            // "near" (the session decides C 9 / O 17 by its own near rule)
            w.nearHit = true;
            CancelJob(w);
            std::vector<standpoint_direct::PathPoint> here(1);
            here[0].pos = a_rep.pos;
            const bool ok = standpoint_direct::Restart(w.id, std::move(here), std::nullopt, 0.f);
            logger::info("WALK #{}{} NEAR RULE: {} {:.0f} u from the spot (the near zone is {:.0f} u) - an ARRIVAL where she stopped{}", w.id,
                         SteerTag(w), a_rep.why == E::kStrayed ? "pushed off the route" : "blocked", Dist2D(a_rep.pos, w.goal), w.nearRadius,
                         ok ? ": she settles and faces, then stands held" : " - the driver had no paused walker: answered now");
            if (ok) {
                w.stage.store(kDriving);
                return;
            }
            w.endWord = "near";
            standpoint_direct::Stop(w.id, "near");
            Answer(w, 'C', StandpointAPI::kDetailArrivedShort, "the near rule: she stopped within the near zone of the spot");
            Done(w);
            return;
        }
        if ((a_rep.why == E::kBlocked || a_rep.why == E::kStrayed) && a_rep.paused) {
            // 1.5.0 (D §3 item 1, audit C fix 2): a stall in a NEW place, a door that did not open, a push off the line -> a new
            // route from where she stands (she is held meanwhile), as often as it happens inside the walk's 90 s; never twice
            // from one spot (within kReplanSameSpot of where any earlier re-plan of this walk started: no progress) -> O 18
            // (1.6.0: a chunk / a re-aim planned meanwhile is dropped - the new route from here goes to the walk's point, through the
            //  skeleton and chunks again when it lies beyond the planner's reach)
            const bool        stray = a_rep.why == E::kStrayed;
            auto*             a = LookupActor(w.who);
            float             nearest = -1.f;         // (review R2 S7) to the nearest earlier re-plan start of this walk
            for (const auto& p : w.replanFroms) {
                const float d = (a_rep.pos - p).Length();
                if (nearest < 0.f || d < nearest) nearest = d;
            }
            const std::string what = stray        ? std::string("pushed off the route") :
                                     a_rep.door   ? std::format("the door [{:08X}] did not open", a_rep.door) :
                                                    std::string("blocked: no progress");
            std::string       why;
            std::int32_t      detail = stray ? StandpointAPI::kDetailStrayed : StandpointAPI::kDetailBlocked;
            if (!a) {
                why = what + ", and she is gone";
            } else if (nearest >= 0.f && nearest <= kReplanSameSpot) {
                detail = StandpointAPI::kDetailBlocked;
                why = std::format("{} - {:.0f} u from where an earlier new route started: never planned twice from one spot without "
                                  "progress", what, nearest);
            } else {
                ++w.replans;
                // (1.6.1 B2) an openable door's first failure in the walk reports markBad false: no bad spot - the doorway may be the
                // only way, and the new route may try the door again
                const bool markBad = !stray && a_rep.markBad;
                if (markBad) standpoint_path::MarkBad(w.who, a_rep.stallSpot, kBadSpotS);
                logger::info("WALK #{} {} at ({:.0f},{:.0f},{:.0f}){} - NEW ROUTE #{} from where she stands (she is held meanwhile){}",
                             w.id, stray ? "PUSHED off the route" : a_rep.door ? "BLOCKED by a door" : "BLOCKED", a_rep.pos.x, a_rep.pos.y,
                             a_rep.pos.z,
                             stray ? std::string() :
                             !markBad ? std::string(" - NO bad spot (the door's first failure in this walk: the new route may try it again)") :
                                     std::format(" - bad spot marked at ({:.0f},{:.0f},{:.0f}) for {:.0f} s (refused within about {:.0f} u "
                                                 "of it, {:.0f} u from her)", a_rep.stallSpot.x, a_rep.stallSpot.y, a_rep.stallSpot.z,
                                                 kBadSpotS, a_rep.stallRadius, Dist2D(a_rep.pos, a_rep.stallSpot)),
                             w.replans,
                             nearest >= 0.f ? std::format("; {:.0f} u from the nearest earlier new route's start", nearest) : std::string());
                if (w.replanFroms.size() >= kReplanSpots) w.replanFroms.erase(w.replanFroms.begin());
                w.replanFroms.push_back(a_rep.pos);
                w.replanStray = stray;
                w.replanDrifted = false;
                w.swapHeld = false;
                if (StartPlan(w, a, kJobReplan, stray ? "a new route after a push off the line" :
                                                a_rep.door ? "a new route: the door did not open" : "a new route after a stall")) {
                    w.stage.store(kReplanning);
                    return;
                }
                why = what + ", and the planner refused the new route (its table is full, or she has no 3D)";
            }
            const char* word = stray ? "strayed" : "blocked";
            standpoint_direct::Stop(w.id, word);
            w.endWord = word;
            Answer(w, 'O', detail, why);
            Done(w);
            return;
        }
        if (a_rep.why == E::kArrived || a_rep.why == E::kDepthReached) {
            w.lastHolding = a_rep.holding;
            w.lastFaceErr = a_rep.facingErrDeg;
            if (w.endLetter != 0 && w.endLetter != 'C') {
                // 1.6.0: no way on from the last checked point (O 1 / O 24): she has walked the checked route to its end - answered
                // and let go
                w.endWord = w.endDetail == StandpointAPI::kDetailNoWadeSpot ? "no-wading-spot" : "no-route";
                w.lastHolding = false;
                standpoint_direct::Stop(w.id, w.endWord);
                Answer(w, w.endLetter, w.endDetail, std::format("{} - she stopped at the last checked point", w.endWhy));
                Done(w);
                return;
            }
            w.endWord = w.nearHit ? "near" : standpoint_direct::EndName(a_rep.why);
            std::int32_t detail = a_rep.why == E::kDepthReached ? StandpointAPI::kDetailDepthReached : StandpointAPI::kDetailClean;
            std::string  why = a_rep.why == E::kDepthReached ? "arrived: the asked depth is reached" : "arrived on the point";
            const float  left = Dist2D(a_rep.pos, w.goal);
            if (w.nearHit) {
                detail = StandpointAPI::kDetailArrivedShort;     // (S1 W1: the near rule - the session judges near / not near)
                why = std::format("the near rule: stopped {:.0f} u from the spot, within its near zone ({:.0f} u)", left, w.nearRadius);
            } else if (w.endLetter == 'C') {
                detail = w.endDetail;                           // 1.6.0: the loaded edge / no way on (a partial walk) - C 9
                why = std::format("{} - {:.0f} u short of the asked point", w.endWhy, left);
            } else if (a_rep.why == E::kArrived && left > kArriveRadius) {
                // (review H #7) C 0 only on the point: anything farther is C 9 with the measured distance, partial or not
                detail = StandpointAPI::kDetailArrivedShort;
                why = std::format("arrived {:.0f} u short of the asked point ({})", left,
                                  w.partial ? "a PARTIAL route, or a point the planner moved toward her"
                                            : "she stopped beside it - something in the way at the end");
            }
            CancelJob(w);
            Answer(w, 'C', detail, std::format("{} (facing off by {:.1f} deg)", why, a_rep.facingErrDeg));
            if (a_rep.holding) {
                w.stage.store(kHolding);
            } else {
                Done(w);
            }
            return;
        }
        w.driverHas = false;                       // every other report: the driver has already let her go
        char         letter = 'X';
        std::int32_t detail = StandpointAPI::kDetailEnded;
        // (review H #12) an end reported by the driver is never "not a loaded NPC": she was taken (or about to be) - X 14;
        // kOtherDriver still maps to B 22
        MapEnd(a_rep.why, false, letter, detail);
        w.endWord = standpoint_direct::EndName(a_rep.why);
        Answer(w, letter, detail, standpoint_direct::EndName(a_rep.why));
        Done(w);
    }

    // a planned route is in: the drive starts once her Standpoint session (C++ requests) is released
    void ReadyToDrive(Walk& w)
    {
        if (!w.sessionFree.load()) {
            if (!w.waitLogged) {
                w.waitLogged = true;
                logger::info("WALK #{} waits for her Standpoint session to be released (StandpointAPI.ApiWalkFreeSession)", w.id);
            }
            return;
        }
        BeginDrive(w);
    }

    // ── 1.5.0 the wade-spot walk's seeds (main thread: Water Finder's rays run here) ─────────────────────────────────
    std::string SummaryText(const WfResult& a_r)
    {
        return std::string(a_r.summary, ::strnlen(a_r.summary, sizeof(a_r.summary)));
    }

    // the first seed: Water Finder's nearest water target within maxMetres. false = answered O 23 (no water), the walk is done
    bool FirstSeed(Walk& w)
    {
        auto*    wf = g_wf2.load();
        WfResult r{};
        if (wf) wf->FindNearestWater(w.who, w.maxMetres, false, r);
        if (!wf || !r.found) {
            w.endWord = "no-water";
            Answer(w, 'O', StandpointAPI::kDetailNoWater, std::format("no water within {:.0f} m of her (Water Finder)", w.maxMetres));
            Done(w);
            return false;
        }
        w.seed = RE::NiPoint3(r.x, r.y, r.z);
        w.tried.push_back(w.seed);
        w.goal = w.seed;
        float      sx = 0.f, sy = 0.f, sz = 0.f;
        const bool surface = wf->GetWaterSurfacePoint(w.who, sx, sy, sz);
        logger::info("WALK #{} WATER: Water Finder's seed ({:.0f},{:.0f},{:.0f}) [the bed]{} - {} ({:.1f} m, bearing {:.0f} deg)", w.id, r.x,
                     r.y, r.z, surface ? std::format(", surface {:.0f} -> {:.0f} u deep there", sz, sz - r.z) : std::string(),
                     SummaryText(r), r.distanceMetres, r.bearingDegrees);
        return true;
    }

    // "no reachable wading spot" near the seed: the NEXT candidate (Water Finder 1.6.2's GetWaterCandidates, asked once) that
    // is no seed tried yet, planned at once: 1 = planned, 0 = none left, -1 = the planner refused the plan (review R2 S8: the
    // seed / point are rolled back; the caller answers B 11). (1.6.0: as job a_kind - from a_from when a_useFrom: a swap on the move)
    int NextSeed(Walk& w, RE::Actor* a_actor, int a_kind, bool a_useFrom = false, const RE::NiPoint3& a_from = {})
    {
        if (!w.candsAsked) {
            w.candsAsked = true;
            if (auto* wf3 = g_wf3.load()) {
                float     xyz[3 * kMaxSeeds]{};
                const int got = (std::clamp)(wf3->GetWaterCandidates(w.who, w.maxMetres, xyz, kMaxSeeds), 0, kMaxSeeds);
                std::string list;
                for (int i = 0; i < got; ++i) {
                    w.cands.emplace_back(xyz[3 * i], xyz[3 * i + 1], xyz[3 * i + 2]);
                    list += std::format(" ({:.0f},{:.0f},{:.0f})", xyz[3 * i], xyz[3 * i + 1], xyz[3 * i + 2]);
                }
                logger::info("WALK #{} WATER: Water Finder's candidates within {:.0f} m (revision 3): {}{}", w.id, w.maxMetres, got,
                             list);
            } else {
                logger::info("WALK #{} WATER: no other candidate - this Water Finder hands out one seed (its C++ revision 3, Water "
                             "Finder 1.6.2, is absent)", w.id);
            }
        }
        for (const auto& c : w.cands) {
            bool seen = false;
            for (const auto& t : w.tried) {
                if ((c - t).Length() <= kSeedSame) seen = true;
            }
            if (seen) continue;
            const RE::NiPoint3 oldSeed = w.seed;
            const RE::NiPoint3 oldGoal = w.goal;
            w.seed = c;
            w.goal = c;
            if (!StartJob(w, a_actor, a_kind, a_useFrom, a_from, c, "the next water candidate")) {
                w.seed = oldSeed;
                w.goal = oldGoal;
                return -1;
            }
            w.tried.push_back(c);
            return 1;
        }
        return 0;
    }

    // ── 1.6.0 THE ROLLING PLAN / RE-PLANS ON THE MOVE (main thread) ─────────────────────────────────────────────────
    // the end of the walk is decided before she gets there (the loaded edge, no way on): she walks the checked route to its end
    // (DirectWalk::Extend with nothing more) and her arrival answers a_letter a_detail
    void EndAtVerified(Walk& w, char a_letter, std::int32_t a_detail, const std::string& a_why)
    {
        CancelJob(w);
        w.chunked = false;
        w.remainAfter = 0.f;
        w.skelRest.clear();
        w.endLetter = a_letter;
        w.endDetail = a_detail;
        w.endWhy = a_why;
        const bool ok = standpoint_direct::Extend(w.id, {}, 0.f);
        logger::info("WALK #{}{} ENDS AT THE LAST CHECKED POINT ({:.0f},{:.0f},{:.0f}): {} - she walks there and her arrival answers {} {}{}",
                     w.id, SteerTag(w), w.verifiedEnd.x, w.verifiedEnd.y, w.verifiedEnd.z, a_why, a_letter, a_detail,
                     ok ? "" : " (the driver has no walker of hers: answered now)");
        if (!ok) {
            w.endWord = a_letter == 'C' ? "arrived" : "no-route";
            if (w.driverHas) standpoint_direct::Stop(w.id, w.endWord);
            Answer(w, a_letter, a_detail, a_why);
            Done(w);
        }
    }

    // the route of a plan that is the walk's FINAL route (it reaches the point): a steered walk's fixed tail (E -> S) is appended
    // when the plan reached its goal (never after a partial / moved end: that leg would be untested)
    void FinishRoute(Walk& w, std::vector<standpoint_path::RoutePoint>& a_route, const standpoint_path::PlanInfo& a_info)
    {
        if (!w.steer || w.tail.empty() || a_route.empty()) return;
        if (a_info.status == 3 || a_info.goalMovedBy > 0.f || Dist2D(a_route.back().pos, w.planTo) > kSkipLead) {
            logger::info("WALK #{}{} the fixed last leg is NOT appended: the route ends {:.0f} u from the approach point (partial / moved)",
                         w.id, SteerTag(w), Dist2D(a_route.back().pos, w.planTo));
            return;
        }
        AppendFixed(a_route, w.tail);
        logger::info("WALK #{}{} the fixed last leg appended: {} point(s) to the spot ({:.0f},{:.0f},{:.0f})", w.id, SteerTag(w), w.tail.size(),
                     w.goal.x, w.goal.y, w.goal.z);
    }

    // the chunk state after a plan the driver gets (first route, a new route, a swap, an extension)
    void TakeChunkState(Walk& w, const standpoint_path::PlanInfo& a_info)
    {
        if (a_info.chunked && !a_info.chunkFinal) {
            w.chunked = true;
            w.remainAfter = (std::max)(1.f, a_info.remainAfterU);
            const auto taken = static_cast<std::size_t>((std::clamp)(a_info.skelTaken, 0, static_cast<int>(w.jobSkel.size())));
            w.skelRest.assign(w.jobSkel.begin() + static_cast<std::ptrdiff_t>(taken), w.jobSkel.end());
            if (w.skelRest.empty()) w.remainAfter = 0.f;    // (nothing left to chunk: the route as it is ends the walk)
            w.chunked = !w.skelRest.empty();
        } else {
            w.chunked = false;
            w.remainAfter = 0.f;
            w.skelRest.clear();
        }
        ++w.chunks;
    }

    // a water walk's final face-the-water heading from its final route (nullopt = none)
    std::optional<float> FaceRadOf(Walk& w)
    {
        if (!w.water) return std::nullopt;
        if (float deg = 0.f; w.faceWater && WaterFacing(w, deg)) {
            w.facingDeg = deg;
            logger::info("WALK #{} she will face the water: {:.0f} deg (from where her route enters the water to the spot)", w.id, deg);
            return deg * kDegToRad;
        }
        return std::nullopt;
    }

    // 1.6.0: the next verified chunk while she walks (S2 §2.3): asked once less than kChunkAskBelowU of checked route is left
    // ahead of her. The point outside the loaded area by then (the player walked off) = she ends at the last checked point, C 9.
    // A water walk's last piece (the seed within the planner's reach of the last checked point) = the wade-spot search from it.
    void MaybeAskChunk(Walk& w, RE::Actor* a_actor)
    {
        if (!w.chunked || w.job != kJobNone || w.endLetter != 0 || !w.driverHas) return;
        standpoint_direct::Progress p{};
        if (!standpoint_direct::GetProgress(w.id, p) || p.answered || p.paused) return;
        const float ahead = (std::max)(0.f, p.total - p.s);
        if (ahead >= kChunkAskBelowU && !p.waitsForRoute) return;
        const RE::NiPoint3 target = TargetOf(w);
        if (std::string why; !standpoint_nav::GoalLoaded(a_actor, target, &why)) {
            EndAtVerified(w, 'C', StandpointAPI::kDetailArrivedShort,
                          std::format("the loaded area ended {:.0f} u before the point ({})", (target - w.verifiedEnd).Length(), why));
            return;
        }
        bool started = false;
        CancelJob(w);
        w.job = kJobChunk;
        w.jobUseFrom = true;
        w.jobFrom = w.verifiedEnd;
        w.jobTo = target;
        w.jobT0 = Clock::now();
        if (w.water && Dist2D(w.verifiedEnd, target) <= standpoint_path::kRangeV15 && w.remainAfter <= standpoint_path::kChunkNextU) {
            started = StartPlanStep(w, a_actor, nullptr, 0.f, 0.f, true, "the last piece: the wade-spot search from the last checked point");
        } else {
            const auto skel = w.skelRest;
            started = StartPlanStep(w, a_actor, &skel, 0.f, standpoint_path::kChunkNextU, false,
                                    std::format("CHUNK {}: {:.0f} u of checked route left ahead of her", w.chunks + 1, ahead).c_str());
        }
        if (!started) {
            w.job = kJobNone;     // the planner's table is full: asked again next frame (she slows toward the checked end meanwhile)
        }
    }

    // 1.6.0 the answer of a skeleton job (any job): 0 = not in yet; 1 = in, and its first chunk's plan is started (or, with no
    // skeleton, one plan within the planner's reach); -1 = no way on (a_info says why: outsideLoaded = O 26 / the loaded edge) -
    // the caller answers. (a_actor is loaded: the callers checked)
    int PollSkeleton(Walk& w, RE::Actor* a_actor, standpoint_nav::SkelInfo& info)
    {
        std::vector<standpoint_nav::SkelPoint> pts;
        if (!standpoint_nav::GetSkeleton(w.skel, info, pts)) return 0;
        const double ms = std::chrono::duration<double, std::milli>(Clock::now() - w.skelT0).count();
        standpoint_nav::Cancel(w.skel);
        w.skel = 0;
        logger::info("WALK #{}{} SKELETON status {} for {}: {} point(s), {:.0f} u, {} triangle(s) expanded over {} navmesh(es), {} virtual "
                     "link(s), {} restart(s), {} slice(s) ({:.2f} ms CPU, longest {:.2f} ms), {:.0f} ms wall{}{} - {}", w.id, SteerTag(w),
                     info.status, JobName(w.job), pts.size(), info.length, info.expanded, info.meshes, info.links, info.restarts, info.slices,
                     info.ms, info.msMax, ms, info.selfCheckFailed ? " | the SELF-CHECK FAILED: the 1.5.x grid corridor" : "",
                     info.outsideLoaded ? " | the point is OUTSIDE the loaded area" : "", info.why);
        if (info.status == 1 && !pts.empty()) {
            // chunk mode along it: the first chunk short (she starts / goes on soon), the next ones by the planner's rule
            const bool  first = w.job == kJobFirst || w.job == kJobReplan || w.job == kJobSwap;
            const float cmin = first ? standpoint_path::kChunkFirstU : 0.f;
            const float cmax = first ? standpoint_path::kChunkFirstU : standpoint_path::kChunkNextU;
            if (StartPlanStep(w, a_actor, &pts, cmin, cmax, false,
                              std::format("chunk {} of a {:.0f} u skeleton", w.job == kJobChunk ? w.chunks + 1 : 1, info.length).c_str())) {
                return 1;
            }
            info.why = "the planner refused the chunk's plan (its table of 16 plans is full, or she has no 3D)";
            return -1;
        }
        if (info.outsideLoaded) return -1;    // O 26 / the loaded edge (the caller)
        // no skeleton (status 0: the navmesh has no way; -1: it could not be read; -2: dropped) - within the planner's reach, ONE
        // plan as 1.5.x (its own corridor / the full ray search); beyond it there is no way to plan
        const RE::NiPoint3 start = w.jobUseFrom ? w.jobFrom : a_actor->GetPosition();
        if (Dist2D(start, w.jobTo) <= standpoint_path::kRangeV15 &&
            StartPlanStep(w, a_actor, nullptr, 0.f, 0.f, true, "no skeleton: one plan within the planner's reach")) {
            return 1;
        }
        return -1;
    }

    float WrapRad(float a_rad)
    {
        constexpr float kPi = 3.14159265f;
        while (a_rad > kPi) a_rad -= 2.f * kPi;
        while (a_rad < -kPi) a_rad += 2.f * kPi;
        return a_rad;
    }

    // 1.6.0 (S2 §2.4, S1 W2): before a new route is planned while she walks - she COASTS on her route (never arriving at its end:
    // more route comes) when the new point lies within 90 deg of her travel direction and she moves; else she is HELD at speed 0
    // ("else today's hold"). a_from = where the new route's plan starts: her point 0.5 s ahead along her route, or where she
    // stands. true = coast (the plan starts at a_from).
    bool PrepareSwap(Walk& w, RE::Actor* a_actor, const RE::NiPoint3& a_to, RE::NiPoint3& a_from, std::string& a_how)
    {
        standpoint_direct::Progress p{};
        const bool                  have = standpoint_direct::GetProgress(w.id, p);
        a_from = a_actor->GetPosition();
        float off = 180.f;
        if (have && p.moving && p.speedUps >= kCoastMinUps) {
            off = std::fabs(WrapRad(std::atan2(a_to.x - p.pos.x, a_to.y - p.pos.y) - p.travelRad)) * kRadToDeg;
            if (off <= kRetargetCoastDeg) {
                // (review R2 #2) the lead = 0.5 s, or the last plan's wall time when that was longer (<= 2 s): the plan should land
                // before she passes its start, so the new route joins on her own (tested) route. (1.6.1, S2's verify NIT b) its
                // distance at most kRetargetLeadMaxU: Swap keeps her OLD route up to the plan's start - 2 s at a run (~720 u) was a
                // late turn after a head turn
                const float  leadS = (std::clamp)(w.lastJobS, kRetargetLeadS, kRetargetLeadMaxS);
                const float  lead = (std::min)(leadS * (std::max)(p.speedUps, kWalkUps), kRetargetLeadMaxU);
                RE::NiPoint3 ahead{};
                if (standpoint_direct::PointAhead(w.id, lead, ahead)) {
                    standpoint_direct::Extend(w.id, {}, 1.f);      // she never arrives at the old end now: more route comes
                    w.swapHeld = false;
                    // (1.6.1 B3, run 2026-11-06 #7595821) on a STAIR leg - or when the point ahead lies off her floor (the route line
                    // runs under / over a stair's surface: its z is no floor) - the plan starts at HER FEET: a plan from a point 65 u
                    // under the stair found no way and she stopped on the stair 0.4 s. She keeps walking; the plan's start is then
                    // behind her when it lands, and the join (SwapJoin: probed when more than 48 u off) decides
                    const float dz = ahead.z - p.pos.z;
                    if (p.onStairLeg || std::fabs(dz) > kAheadDzPad + kAheadDzSlope * lead) {
                        a_from = p.pos;
                        a_how = std::format("she keeps walking her route (the new point is {:.0f} deg off her way, she moves {:.0f} u/s): planned "
                                            "from HER FEET ({:.0f},{:.0f},{:.0f}) - {}", off, p.speedUps, p.pos.x, p.pos.y, p.pos.z,
                                            p.onStairLeg ? std::string("she is on a stair leg (the route line's height there is no floor)")
                                                         : std::format("her point {:.0f} u ahead lies {:.0f} u {} her (no floor of hers)", lead,
                                                                       std::fabs(dz), dz < 0.f ? "below" : "above"));
                        return true;
                    }
                    a_from = ahead;
                    a_how = std::format("she keeps walking her route (the new point is {:.0f} deg off her way, she moves {:.0f} u/s): planned "
                                        "from her point {:.0f} u ({:.1f} s) ahead ({:.0f},{:.0f},{:.0f})", off, p.speedUps, lead, leadS,
                                        ahead.x, ahead.y, ahead.z);
                    return true;
                }
            }
        }
        standpoint_direct::Pause(w.id, "a new route is planned");
        w.swapHeld = true;
        a_how = have && p.moving && p.speedUps >= kCoastMinUps
                    ? std::format("held at speed 0 - the new point is {:.0f} deg off her way (more than {:.0f})", off, kRetargetCoastDeg)
                    : std::string("held at speed 0 - she was not on her way (standing / turning / waiting)");
        return false;
    }

    // 1.6.0: a new route on the move toward TargetOf(w) (a re-aim, the next water seed): planned by the walk-job rule (the skeleton
    // beyond the planner's reach) and swapped in when it comes (StepDriving). false = refused (she is held or coasts meanwhile:
    // the caller answers).
    bool StartSwap(Walk& w, RE::Actor* a_actor, const char* a_why)
    {
        CancelJob(w);
        const RE::NiPoint3 to = TargetOf(w);
        RE::NiPoint3       from{};
        std::string        how;
        const bool         coast = PrepareSwap(w, a_actor, to, from, how);
        ++w.retargets;
        logger::info("WALK #{}{} RE-AIM #{} ({}): toward ({:.0f},{:.0f},{:.0f}) - {}", w.id, SteerTag(w), w.retargets, a_why, to.x, to.y, to.z,
                     how);
        if (StartJob(w, a_actor, kJobSwap, coast, from, to, a_why)) return true;
        // refused (the planner's / NavGraph's table is full): she is not left held or coasting toward an old point - answered
        standpoint_direct::Stop(w.id, "busy");
        w.endWord = "busy";
        Answer(w, 'B', StandpointAPI::kDetailNotReady, "the planner refused the new route on the move (its table of 16 plans is full)");
        Done(w);
        return false;
    }

    // 1.6.0: the route of a chunk / a swap job is in - appended (Extend) or joined where she is (Swap), she never stops for it
    void TakeRoute(Walk& w, RE::Actor* a_actor, const standpoint_path::PlanInfo& a_info, std::vector<standpoint_path::RoutePoint>& a_pts)
    {
        const bool         swap = w.job == kJobSwap;
        const bool         finalRoute = !(a_info.chunked && !a_info.chunkFinal);
        const RE::NiPoint3 start = w.jobUseFrom ? w.jobFrom : w.planFrom;
        const RE::NiPoint3 plannedTo = w.jobTo;
        w.lastJobS = std::chrono::duration<float>(Clock::now() - w.jobT0).count();
        if (finalRoute) FinishRoute(w, a_pts, a_info);
        if (swap) {
            // (review R2 #2) she passed the new route's start (or it is not on her route): Swap joins the new route's leg nearest to
            // her along a line NOBODY TESTED. More than 48 u off it (BeginDrive's rule) -> that line is probed first; not clear ->
            // she is held where she is and the new route is planned again from there (its join is then at her feet)
            RE::NiPoint3 joinFrom{}, joinTo{};
            float        off = 0.f;
            if (standpoint_direct::SwapJoin(w.id, PathFrom(start, a_pts), joinFrom, joinTo, off) && off > kDriftProbe) {
                standpoint_path::ProbeNote note;
                const bool clear = standpoint_path::ProbeSegment(a_actor, joinFrom, joinTo, w.depthFrac, true, &note);
                logger::info("WALK #{}{} SWAP JOIN: she is {:.0f} u off the new route's nearest leg - the joining line ({:.0f},{:.0f},{:.0f}) -> "
                             "({:.0f},{:.0f},{:.0f}) {}", w.id, SteerTag(w), off, joinFrom.x, joinFrom.y, joinFrom.z, joinTo.x, joinTo.y,
                             joinTo.z,
                             clear ? std::string("is clear") :
                                     std::format("is NOT clear ({} at {:.0f} u, {} questions): held, a new route from where she stands",
                                                 note.why, note.failAt, note.questions));
                if (!clear) {
                    standpoint_direct::Pause(w.id, "the joining line to the new route is not clear");
                    w.swapHeld = true;
                    if (!StartJob(w, a_actor, kJobSwap, false, {}, TargetOf(w), "the joining line was not clear: from where she stands")) {
                        standpoint_direct::Stop(w.id, "busy");
                        w.endWord = "busy";
                        Answer(w, 'B', StandpointAPI::kDetailNotReady,
                               "the planner refused the new route on the move (its table of 16 plans is full)");
                        Done(w);
                    }
                    return;
                }
            }
        }
        // (review R2 #12) the point moved again while this re-aim plan ran (it was kept): it is swapped in, but never as the end
        // (no arrival at a point that is not the latest) - the latest point is planned at once, from where she walks
        const bool chainNext = swap && w.reaimPending && (TargetOf(w) - plannedTo).Length() >= kReaimSameU;
        w.reaimPending = false;
        TakeChunkState(w, a_info);
        w.route = a_pts;
        w.planFrom = start;
        if (finalRoute) w.partial = !w.water && (a_info.status == 3 || a_info.goalMovedBy > 0.f);
        std::optional<float> face;
        if (w.water && w.jobWade) {
            w.goal = w.route.back().pos;
            logger::info("WALK #{} WADING SPOT ({:.0f},{:.0f},{:.0f}): {:.0f} u deep, {} of its 8 neighbours wet - {:.0f} u from the seed ({} "
                         "seed(s) planned so far)", w.id, w.goal.x, w.goal.y, w.goal.z, a_info.wadeDepth, a_info.wadePatch,
                         (w.goal - w.seed).Length(), w.tried.size());
            face = FaceRadOf(w);
        }
        const float remain = chainNext ? (std::max)(1.f, w.chunked ? w.remainAfter : 0.f) : (w.chunked ? w.remainAfter : 0.f);
        auto        path = PathFrom(start, w.route);
        const RE::NiPoint3 last = path.back().pos;
        const bool  held = w.swapHeld;
        w.job = kJobNone;
        w.jobSkel.clear();
        w.jobWade = false;
        w.swapHeld = false;
        const bool ok = swap ? standpoint_direct::Swap(w.id, std::move(path), remain, face)
                             : standpoint_direct::Extend(w.id, path, remain, face);
        if (ok) {
            w.verifiedEnd = last;
            w.reskels = 0;
        }
        logger::info("WALK #{}{} {} {}: {} point(s), {:.0f} u checked, {:.2f} s{}{}{}", w.id, SteerTag(w), swap ? "NEW ROUTE ON THE MOVE" : "CHUNK",
                     swap ? std::format("(re-aim #{}, {})", w.retargets, held ? "she was held" : "she kept walking") :
                            std::format("{} in", w.chunks),
                     w.route.size(), a_info.length, w.lastJobS,
                     w.chunked ? std::format("; {:.0f} u of skeleton after it - more chunks follow", w.remainAfter)
                               : chainNext ? std::string("; the point has moved again meanwhile - its end is no arrival")
                                           : std::string("; it reaches the point (the last piece)"),
                     ok ? (swap ? " - joined where she is (no stop)" : " - appended to the route she walks (no stop)")
                        : " - the driver could not take it (she has arrived / ended meanwhile: its report answers)",
                     w.chunked ? std::format(" | ends at ({:.0f},{:.0f},{:.0f})", last.x, last.y, last.z) : std::string());
        if (ok && chainNext) StartSwap(w, a_actor, "the point moved again while the last plan ran: the latest point");
    }

    // 1.6.0: a chunk / a swap job found no usable route (a_wadeNone: a water walk's last piece found no wading spot)
    void JobFailed(Walk& w, RE::Actor* a_actor, bool a_outside, bool a_wadeNone, const std::string& a_why)
    {
        const bool swap = w.job == kJobSwap;
        const bool pending = w.reaimPending;      // (review R2 #12) the point moved again while this kept plan ran
        CancelJob(w);
        if (a_wadeNone) {
            // the next water candidate, planned from where she stands (she is held meanwhile) - else no wading spot
            w.wadeNoneSeen = true;
            standpoint_direct::Pause(w.id, "the next water candidate is planned");
            w.swapHeld = true;
            const int next = NextSeed(w, a_actor, kJobSwap);
            if (next > 0) {
                ++w.retargets;
                logger::info("WALK #{} WATER: no wading spot near the seed ({}) - the next candidate ({:.0f},{:.0f},{:.0f}) is planned from where "
                             "she stands (held meanwhile)", w.id, a_why, w.seed.x, w.seed.y, w.seed.z);
                return;
            }
            w.endWord = next < 0 ? "busy" : "no-wading-spot";
            standpoint_direct::Stop(w.id, w.endWord);
            Answer(w, next < 0 ? 'B' : 'O', next < 0 ? StandpointAPI::kDetailNotReady : StandpointAPI::kDetailNoWadeSpot,
                   next < 0 ? std::string("the planner refused the plan for the next water candidate (its table of 16 plans is full)")
                            : std::format("water found, no reachable wading spot ({} seed(s) tried): {}", w.tried.size(), a_why));
            Done(w);
            return;
        }
        if (swap && pending) {
            // (review R2 #12) the kept plan (to an older point) found no way - the LATEST point is planned once before any answer
            logger::info("WALK #{}{} the kept re-aim plan found no way ({}) - the latest point is planned instead", w.id, SteerTag(w), a_why);
            StartSwap(w, a_actor, "the kept plan found no way: the latest point");    // (refused = answered B 11 there)
            return;
        }
        if (swap) {
            // a re-aim found no way to the new point: answered (the session / the follow program decides what comes next)
            const char* word = a_outside ? "outside-loaded" : "no-route";
            standpoint_direct::Stop(w.id, word);
            w.endWord = word;
            Answer(w, 'O', a_outside ? StandpointAPI::kDetailOutsideLoaded : StandpointAPI::kDetailNoSlot,
                   std::format("{}: {}", a_outside ? "the new point is outside the loaded area" : "no route to the new point", a_why));
            Done(w);
            return;
        }
        // a chunk: the loaded edge -> C 9 at the last checked point; else the skeleton again from there (<= 3), else she ends there
        if (a_outside) {
            EndAtVerified(w, 'C', StandpointAPI::kDetailArrivedShort, std::format("the loaded area ended before the point: {}", a_why));
            return;
        }
        if (w.reskels < kMaxReskeletons) {
            ++w.reskels;
            const std::string why = std::format("a chunk failed ({}): the way again from the last checked point ({} of {})", a_why,
                                                w.reskels, kMaxReskeletons);
            if (StartJob(w, a_actor, kJobChunk, true, w.verifiedEnd, TargetOf(w), why.c_str())) return;
        }
        const bool partialOk = !w.water && (OptionsFor(w).flags & standpoint_path::kPartial) != 0u;
        if (partialOk) {
            EndAtVerified(w, 'C', StandpointAPI::kDetailArrivedShort, std::format("no way on from the last checked point: {}", a_why));
        } else {
            EndAtVerified(w, 'O', w.water && w.wadeNoneSeen ? StandpointAPI::kDetailNoWadeSpot : StandpointAPI::kDetailNoSlot,
                          std::format("no route on from the last checked point: {}", a_why));
        }
    }

    // 1.6.0 stage kDriving: the job that runs while she walks (the next chunk / a new route on the move), else the chunk trigger
    void StepDriving(Walk& w)
    {
        auto* a = LookupActor(w.who);
        if (!a || !a->Is3DLoaded() || !w.driverHas) return;      // (the driver reports her end)
        if (w.job != kJobChunk && w.job != kJobSwap) {
            MaybeAskChunk(w, a);
            return;
        }
        if (w.skel) {
            standpoint_nav::SkelInfo si{};
            const int                r = PollSkeleton(w, a, si);
            if (r >= 0) return;
            JobFailed(w, a, si.outsideLoaded, false, std::format("the navmesh skeleton (status {}): {}", si.status, si.why));
            return;
        }
        if (w.plan <= 0) {
            CancelJob(w);           // (nothing runs: the chunk trigger asks again)
            return;
        }
        standpoint_path::PlanInfo                info{};
        std::vector<standpoint_path::RoutePoint> pts;
        if (!standpoint_path::GetRoute(w.plan, info, pts)) return;
        const double ms = std::chrono::duration<double, std::milli>(Clock::now() - w.planT0).count();
        standpoint_path::Cancel(w.plan);
        w.plan = 0;
        LogPlan(w, info, pts, ms);
        const bool partialOk = !w.water && (OptionsFor(w).flags & standpoint_path::kPartial) != 0u;
        const bool usable = (info.status == 1 || info.status == 2 || (info.status == 3 && partialOk)) && !pts.empty();
        if (!usable) {
            JobFailed(w, a, info.outsideLoaded, w.jobWade && (info.status == 0 || info.status == 3) && !info.atSwimLevel,
                      std::format("{}: {}", PlanStatusName(info.status), info.why));
            return;
        }
        TakeRoute(w, a, info, pts);
    }

    // 1.6.0 RE-AIM (S1 W2, S2 §2.4): the walk's point (a steered walk: its whole ask) has changed - planned again by its stage, she
    // never stops for it while she walks. false = she has answered (stands), a stop is marked, or she is gone.
    bool Reaim(Walk& w, const char* a_why)
    {
        const int stage = w.stage.load();
        if (stage == kDone || stage == kHolding || w.answered || w.stopDetail.load() != 0) return false;
        auto* a = LookupActor(w.who);
        w.endLetter = 0;
        w.endDetail = 0;
        w.endWhy.clear();
        w.nearHit = false;
        w.partial = false;
        w.replanFroms.clear();      // (a new point: a new route from a spot an old one started from is no ping-pong)
        if (stage == kLaunch) return true;    // (the launch reads the new ask)
        if (!a || !a->Is3DLoaded()) return false;
        if (stage == kPlanning || stage == kReady) {
            w.driftReplanned = false;
            if (!w.fixedRoute.empty()) {
                CancelJob(w);
                w.route.clear();
                AppendFixed(w.route, w.fixedRoute);
                w.planFrom = a->GetPosition();
                w.stage.store(kReady);
                return true;
            }
            if (!StartPlan(w, a, kJobFirst, a_why)) return false;
            w.stage.store(kPlanning);
            return true;
        }
        if (stage == kReplanning) return StartPlan(w, a, kJobReplan, a_why);
        if (stage != kDriving) return false;
        if (!w.fixedRoute.empty()) {
            // a fixed route (the orbit) needs no plan: swapped in at once
            CancelJob(w);
            std::vector<standpoint_path::RoutePoint> fixed;
            AppendFixed(fixed, w.fixedRoute);
            auto path = PathFrom(a->GetPosition(), fixed);
            const RE::NiPoint3 last = path.back().pos;
            w.chunked = false;
            w.remainAfter = 0.f;
            w.skelRest.clear();
            w.route = fixed;
            ++w.retargets;
            const bool ok = standpoint_direct::Swap(w.id, std::move(path), 0.f,
                                                    w.facingDeg >= 0.f ? std::optional<float>(w.facingDeg * kDegToRad) : std::nullopt);
            if (ok) w.verifiedEnd = last;
            logger::info("WALK #{}{} RE-AIM #{} ({}): a FIXED route of {} point(s) - {}", w.id, SteerTag(w), w.retargets, a_why, fixed.size(),
                         ok ? "joined where she is (no stop)" : "the driver could not take it");
            return ok;
        }
        // (review R2 #12) a re-aim plan is still running: KEPT when the new point lies within kReaimKeepU of the one it plans for -
        // re-aims coming faster than plans finish never cancel each other (she would coast to the old end and wait there); the
        // latest point is planned as soon as the running plan lands (TakeRoute) or fails (JobFailed)
        if (w.job == kJobSwap && (w.plan > 0 || w.skel > 0)) {
            const float apart = (TargetOf(w) - w.jobTo).Length();
            if (apart <= kReaimKeepU) {
                w.reaimPending = true;
                logger::info("WALK #{}{} RE-AIM ({}): the running re-aim plan (to ({:.0f},{:.0f},{:.0f})) is KEPT - the new point is {:.0f} u from "
                             "it; the latest point is planned when it lands", w.id, SteerTag(w), a_why, w.jobTo.x, w.jobTo.y, w.jobTo.z, apart);
                return true;
            }
        }
        return StartSwap(w, a, a_why);
    }

    // ── one walk, one frame (main thread) ────────────────────────────────────────────────────────────────────────
    void StepWalk(Walk& w, float a_dt)
    {
        const int stage = w.stage.load();
        if (stage == kDone) return;

        if (const int d = w.stopDetail.exchange(0); d > 0) {
            const char* word = d == StandpointAPI::kDetailReplaced ? "replaced" : "released";
            if (w.driverHas) w.drove = w.drove || standpoint_direct::EverDrove(w.id);    // (verify V NEW-2) before the walker goes
            // (review H #8) replaced by a newer WALK of hers: she is not let go while the newer one plans - this walk's driver
            // walker keeps holding her (or the holder it inherited is passed on) until the newer walk's Begin takes her
            bool handedOver = false;
            if (d == StandpointAPI::kDetailReplaced && (w.driverHas || w.inheritId)) {
                WalkPtr succ;
                {
                    std::scoped_lock lk(g_mtx);
                    for (const auto& [id, o] : g_walks) {
                        if (id != w.id && o->who == w.who && o->stage.load() != kDone && (!succ || id > succ->id)) succ = o;
                    }
                }
                if (succ && !succ->answered) {
                    const std::int32_t holder = w.driverHas ? w.id : w.inheritId;
                    if (w.driverHas ? standpoint_direct::Hold(w.id) : true) {
                        if (succ->inheritId && succ->inheritId != holder) standpoint_direct::Stop(succ->inheritId, "replaced");
                        succ->inheritId = holder;
                        if (holder == w.inheritId) w.inheritId = 0;     // passed on: this walk's Done must not stop it
                        w.driverHas = false;
                        handedOver = true;
                        logger::info("WALK #{} replaced by walk #{}: she stays held (driver walker #{}) until it takes her", w.id,
                                     succ->id, holder);
                    }
                }
            }
            if (w.driverHas && !handedOver) standpoint_direct::Stop(w.id, word);
            w.endWord = word;
            if (w.answered) {
                logger::info("WALK #{}{} HOLD END: {} (X {}; the answer was given at the arrival)", w.id, SteerTag(w), word, d);
                NoticeHoldEnd(w, 'X', d);     // v1.4.1: an internal walk is told (the follow program: another owner took her)
            } else {
                Answer(w, 'X', d, std::format("{} while {}{}", word, StageName(stage),
                                              w.job != kJobNone ? std::format(" ({})", JobName(w.job)) : std::string()));
            }
            Done(w);
            return;
        }
        if (!w.answered) {
            w.active += a_dt;
            if (w.active > w.limitS) {      // (1.6.0: 90 s; a walk beyond 30 m: 90 s + d0 / 150 u/s)
                if (w.driverHas) {
                    w.drove = w.drove || standpoint_direct::EverDrove(w.id);   // (verify V NEW-2)
                    standpoint_direct::Stop(w.id, "limit");
                }
                w.endWord = "limit";
                Answer(w, 'O', StandpointAPI::kDetailTimeout,
                       std::format("not there in {:.0f} s (stage {}{})", w.limitS, StageName(stage),
                                   w.job != kJobNone ? std::format(", {}", JobName(w.job)) : std::string()));
                Done(w);
                return;
            }
        }

        switch (stage) {
        case kLaunch: {
            // (review H #6 / #12) not on this runtime: B 19 before anything else is looked at (1.5.0: the driver's own reason -
            // "SE/AE direct walk: OFF (...)" / "... NOT READY because ...")
            if (!standpoint_direct::HookLive()) {
                Refuse(w, standpoint_direct::End::kNotSupported, standpoint_direct::NotLiveText(), true);
                return;
            }
            // 1.5.0: a water walk needs Water Finder 1.6.0+ (its C++ revision 2) - B 25, before anything else is looked at
            if (w.water && !g_wf2.load()) {
                w.endWord = "no-water-finder";
                Answer(w, 'B', StandpointAPI::kDetailNoWaterFinder,
                       "Water Finder is absent or older than 1.6.0 (it answered no C++ revision 2)");
                Done(w);
                return;
            }
            // (review H #12) the OLDER live walks count, whatever their stage (two launches in one frame see each other)
            int open = 0;
            {
                std::scoped_lock lk(g_mtx);
                for (const auto& [id, o] : g_walks) {
                    if (id != w.id && id < w.id && o->stage.load() != kDone) ++open;
                }
            }
            if (open >= kMaxOpen) {
                Refuse(w, standpoint_direct::End::kTableFull, std::format("{} walks are already running", open), true);
                return;
            }
            auto*                  a = LookupActor(w.who);
            standpoint_direct::End why = standpoint_direct::End::kNone;
            std::string            text;
            if (!a) {
                Refuse(w, standpoint_direct::End::kGone, "'who' is not an actor", true);
                return;
            }
            if (!standpoint_direct::CanTake(a, why, text)) {
                Refuse(w, why, text, true);
                return;
            }
            if (w.water && !FirstSeed(w)) return;     // 1.5.0: no water within maxMetres - answered O 23
            // 1.6.0 (S1 W7) HoldHere: no plan - a one-point route at her feet: she is taken, faces, answers C 0 and stands
            if (w.holdHere) {
                w.startPos = a->GetPosition();
                w.goal = w.startPos;
                w.planFrom = w.startPos;
                if (w.faceWho == 0u && w.facingDeg < 0.f) {
                    float deg = a->GetAngleZ() * kRadToDeg;          // she keeps her facing
                    while (deg < 0.f) deg += 360.f;
                    while (deg >= 360.f) deg -= 360.f;
                    w.facingDeg = deg;
                }
                w.route.clear();
                logger::info("WALK #{}{} HOLD HERE: a stand-only drive at her feet ({:.0f},{:.0f},{:.0f}), facing {}, hold {}", w.id, SteerTag(w),
                             w.startPos.x, w.startPos.y, w.startPos.z,
                             w.faceWho ? std::format("toward [{:08X}] {:+.0f} deg", w.faceWho, w.faceOffsetDeg) : std::format("{:.0f} deg", w.facingDeg),
                             w.hold < 0.f ? std::string("-1") : std::format("{:.1f} s", w.hold));
                w.stage.store(kReady);
                ReadyToDrive(w);
                break;
            }
            // 1.5.1 / 1.6.0 THE DISTANCE: d0 = her straight distance to the point (a water walk: to the seed); beyond 200 m = O 1; a
            // point outside the loaded cells = O 26 at once (spec D §1 item 6: Standpoint walks only where the game has the ground
            // loaded); the limit of a walk beyond 30 m = 90 s + d0 / 150
            {
                w.startPos = a->GetPosition();
                const RE::NiPoint3 target = w.fixedRoute.empty() ? TargetOf(w) : w.fixedRoute.back();
                w.d0 = (w.startPos - target).Length();
                if (w.d0 > kMaxRangeU) {
                    w.endWord = "too-far";
                    Answer(w, 'O', StandpointAPI::kDetailNoSlot,
                           std::format("too far (beyond 200 m): {:.0f} u ({:.0f} m) to the point, straight - Standpoint walks up to {:.0f} u",
                                       w.d0, w.d0 / 70.f, kMaxRangeU));
                    Done(w);
                    return;
                }
                if (std::string outside; !standpoint_nav::GoalLoaded(a, target, &outside)) {
                    w.endWord = "outside-loaded";
                    Answer(w, 'O', StandpointAPI::kDetailOutsideLoaded,
                           outside.empty() ? std::string("the point is outside the loaded area (the game has no ground loaded there)") : outside);
                    Done(w);
                    return;
                }
                w.limitS = w.d0 > kLongLimitFromU ? kLimitS + w.d0 / kLongLimitUps : kLimitS;
                const float flat = Dist2D(w.startPos, target);
                logger::info("WALK #{}{} DISTANCE {:.0f} u ({:.1f} m) to the {}, straight - {}; the speed bands by the route left (run beyond "
                             "15 m, jog to 3 m, walk the last 3 m); time limit {:.0f} s{}", w.id, SteerTag(w), w.d0, w.d0 / 70.f,
                             w.water ? "seed" : "point",
                             !w.fixedRoute.empty()                 ? std::format("a FIXED route of {} point(s) (no plan)", w.fixedRoute.size()) :
                             flat > standpoint_path::kRangeV15     ? std::format("beyond the planner's reach ({:.0f} u): the navmesh skeleton, "
                                                                                 "then verified chunks while she walks", standpoint_path::kRangeV15) :
                                                                     std::string("within the planner's reach: one plan"),
                             w.limitS, w.d0 > kLongLimitFromU ? std::format(" (90 s + {:.0f} u / {:.0f} u/s)", w.d0, kLongLimitUps) : std::string());
            }
            // 1.6.0 (S1 W1) a steered session's FIXED route (the orbit): no plan - she walks the caller's points
            if (!w.fixedRoute.empty()) {
                w.route.clear();
                AppendFixed(w.route, w.fixedRoute);
                w.planFrom = w.startPos;
                w.stage.store(kReady);
                ReadyToDrive(w);
                break;
            }
            if (!StartPlan(w, a, kJobFirst, w.water ? "the request: the wade-spot search toward Water Finder's seed" : "the request")) {
                w.endWord = "busy";
                Answer(w, 'B', StandpointAPI::kDetailNotReady,
                       "the planner refused the plan (she has no 3D / parent cell, or its table of 16 plans / skeletons is full)");
                Done(w);
                return;
            }
            w.stage.store(kPlanning);
            break;
        }
        case kPlanning:
        case kReplanning: {
            const int jobKind = stage == kReplanning ? kJobReplan : kJobFirst;
            if (w.skel) {
                // 1.6.0: the navmesh skeleton of a walk beyond the planner's reach - then chunk 1 along it
                auto* a = LookupActor(w.who);
                if (!a || !a->Is3DLoaded()) {
                    if (w.driverHas) standpoint_direct::Stop(w.id, "gone");
                    Refuse(w, standpoint_direct::End::kGone, "she is not loaded any more", !w.driverHas);
                    return;
                }
                standpoint_nav::SkelInfo si{};
                const int                r = PollSkeleton(w, a, si);
                if (r >= 0) break;          // still searching (0), or chunk 1 is being planned (1): polled from the next frame
                if (w.driverHas) standpoint_direct::Stop(w.id, w.replanStray ? "strayed" : "blocked");
                if (si.outsideLoaded) {
                    w.endWord = "outside-loaded";
                    Answer(w, 'O', StandpointAPI::kDetailOutsideLoaded,
                           si.why.empty() ? std::string("the point is outside the loaded area") : si.why);
                } else if (stage == kReplanning) {
                    w.endWord = w.replanStray ? "strayed" : "blocked";
                    Answer(w, 'O', w.replanStray ? StandpointAPI::kDetailStrayed : StandpointAPI::kDetailBlocked,
                           std::format("{}; no way on from where she stands - the navmesh skeleton: {}",
                                       w.replanStray ? "pushed off the route" : "blocked", si.why));
                } else {
                    w.endWord = "no-route";
                    Answer(w, 'O', StandpointAPI::kDetailNoSlot,
                           std::format("no route: the point is beyond the planner's reach ({:.0f} u) and the navmesh skeleton found no way "
                                       "there (status {}): {}", standpoint_path::kRangeV15, si.status, si.why));
                }
                Done(w);
                return;
            }
            standpoint_path::PlanInfo                info{};
            std::vector<standpoint_path::RoutePoint> pts;
            if (w.plan <= 0) {
                // (no job runs: a refused start was answered where it was asked - this cannot wait)
                w.endWord = "busy";
                if (w.driverHas) standpoint_direct::Stop(w.id, "busy");
                Answer(w, 'B', StandpointAPI::kDetailNotReady, "no plan is running for this walk (the planner refused it)");
                Done(w);
                return;
            }
            if (!standpoint_path::GetRoute(w.plan, info, pts)) break;     // still planning (the 90 s limit bounds it)
            const double ms = std::chrono::duration<double, std::milli>(Clock::now() - w.planT0).count();
            standpoint_path::Cancel(w.plan);
            w.plan = 0;
            w.lastJobS = std::chrono::duration<float>(Clock::now() - w.jobT0).count();   // (review R2 #2: the coast's lead)
            LogPlan(w, info, pts, ms);
            // (1.5.0: a water walk takes a route that ends on a wading spot only - status 1 / 2; never a partial one)
            // (1.6.0: a water walk's CHUNK toward the water is a MIX plan - its partial answer is no wading spot either)
            const bool partialOk = !w.water && (OptionsFor(w).flags & standpoint_path::kPartial) != 0u;
            const bool usable = (info.status == 1 || info.status == 2 || (info.status == 3 && partialOk)) && !pts.empty();
            if (!usable && info.outsideLoaded) {
                // 1.6.0: the point lies outside the loaded cells (the planner's own check) - O 26
                if (w.driverHas) standpoint_direct::Stop(w.id, w.replanStray ? "strayed" : "blocked");
                w.endWord = "outside-loaded";
                Answer(w, 'O', StandpointAPI::kDetailOutsideLoaded, std::format("the point is outside the loaded area: {}", info.why));
                Done(w);
                return;
            }
            if (!usable && w.water && w.jobWade && (info.status == 0 || info.status == 3) && !info.atSwimLevel) {
                // 1.5.0: no reachable wading spot near this seed - the next water candidate, planned at once (same stage; she
                // is held meanwhile on a re-plan), inside the walk's 90 s
                if (info.wadeNone) w.wadeNoneSeen = true;
                const int next = [&]() { auto* a = LookupActor(w.who); return a ? NextSeed(w, a, jobKind) : 0; }();
                if (next > 0) break;
                if (next < 0) {
                    // (review R2 S8) a candidate is left but the planner refused its plan (its table of 16 is full): busy
                    if (w.driverHas) standpoint_direct::Stop(w.id, "busy");
                    w.endWord = "busy";
                    Answer(w, 'B', StandpointAPI::kDetailNotReady,
                           "the planner refused the plan for the next water candidate (its table of 16 plans is full)");
                    Done(w);
                    return;
                }
            }
            if (!usable) {
                // status 0 = no way; -1 = the plan failed (walker / physics world gone); -2 = the planner no longer knows the
                // handle (dropped: a load, or 16 newer plans) - I1's GetRoute answers it at once now, no 90 s wait
                if (w.driverHas) standpoint_direct::Stop(w.id, w.replanStray ? "strayed" : "blocked");
                if (info.status == 0 && info.atSwimLevel) {
                    // v1.4.1: the planner refused because she stands at the swim level (or swims): the walk's "she would swim"
                    w.endWord = "would-swim";
                    Answer(w, 'O', StandpointAPI::kDetailWouldSwim, std::format("would swim - no route: {}", info.why));
                } else if (stage == kReplanning && w.replanStray && (info.status == 0 || info.status == 3)) {
                    // 1.5.0: pushed off the route, and no way from where she was pushed to
                    w.endWord = "strayed";
                    Answer(w, 'O', StandpointAPI::kDetailStrayed, std::format("pushed off the route; no new route from there: {}", info.why));
                } else if (stage == kReplanning && (info.status == 0 || info.status == 3)) {
                    w.endWord = "blocked";
                    Answer(w, 'O', StandpointAPI::kDetailBlocked, std::format("blocked; the re-plan found no way: {}", info.why));
                } else if (w.water && (info.status == 0 || info.status == 3) && w.wadeNoneSeen) {
                    // 1.5.0: water was found, but no wading spot she can reach (every seed tried)
                    w.endWord = "no-wading-spot";
                    Answer(w, 'O', StandpointAPI::kDetailNoWadeSpot,
                           std::format("water found, no reachable wading spot ({} seed(s) tried): {}", w.tried.size(), info.why));
                } else {
                    w.endWord = "no-route";
                    Answer(w, 'O', StandpointAPI::kDetailNoSlot,
                           std::format("no route ({}){}: {}", PlanStatusName(info.status),
                                       info.status == -2 ? " - the planner dropped the plan" : "", info.why));
                }
                Done(w);
                return;
            }
            // 1.6.0: a CHUNK of a long walk (more route after it), or the final route (the steered walk's fixed last leg appended)
            const bool finalRoute = !(info.chunked && !info.chunkFinal);
            if (finalRoute) FinishRoute(w, pts, info);
            TakeChunkState(w, info);
            w.job = kJobNone;
            w.jobSkel.clear();
            w.route = std::move(pts);
            if (finalRoute) w.partial = !w.water && (info.status == 3 || info.goalMovedBy > 0.f);
            std::optional<float> faceRad;          // 1.5.0: a water walk's facing, from this route
            if (w.water && w.jobWade) {
                // the wading spot is the walk's point from here on: C 0 within 32 u of it, distLeft measured to it
                w.goal = w.route.back().pos;
                logger::info("WALK #{} WADING SPOT ({:.0f},{:.0f},{:.0f}): {:.0f} u deep, {} of its 8 neighbours wet - {:.0f} u from the "
                             "seed ({} seed(s) planned so far)", w.id, w.goal.x, w.goal.y, w.goal.z, info.wadeDepth, info.wadePatch,
                             (w.goal - w.seed).Length(), w.tried.size());
                faceRad = FaceRadOf(w);
            }
            w.jobWade = false;
            if (w.chunked) {
                logger::info("WALK #{}{} CHUNK 1 in: {} point(s), {:.0f} u checked; {:.0f} u of skeleton after it ({} point(s)) - the next "
                             "chunk is asked once less than {:.0f} u of checked route is left ahead of her", w.id, SteerTag(w),
                             w.route.size(), info.length, w.remainAfter, w.skelRest.size(), kChunkAskBelowU);
            }
            if (stage == kReplanning) {
                auto* a = LookupActor(w.who);
                auto  path = a ? BuildPath(a, w.route) : std::vector<standpoint_direct::PathPoint>{};
                // (review R2 S6) she is held while the new route is planned, but she may be shoved meanwhile: the joining leg
                // (where she stands now -> the route's first point) is probed before she walks it - after a push always, after
                // a stall when she drifted more than kDriftProbe. Not clear -> ONE more re-plan from here, then the end.
                if (a && path.size() >= 2) {
                    const float drift = Dist2D(a->GetPosition(), w.planFrom);
                    if (w.replanStray || drift > kDriftProbe) {
                        const bool ok = standpoint_path::ProbeSegment(a, path[0].pos, path[1].pos, w.depthFrac);
                        logger::info("WALK #{} the new route's joining leg (she is {:.0f} u from where it was planned) {}", w.id, drift,
                                     ok ? "is clear" : "is NOT clear");
                        if (!ok) {
                            if (!w.replanDrifted && StartPlan(w, a, kJobReplan, "she moved away from the new route's start")) {
                                w.replanDrifted = true;
                                break;      // stage stays kReplanning: the next plan is polled from the next frame
                            }
                            standpoint_direct::Stop(w.id, w.replanStray ? "strayed" : "blocked");
                            w.endWord = w.replanStray ? "strayed" : "blocked";
                            Answer(w, 'O', w.replanStray ? StandpointAPI::kDetailStrayed : StandpointAPI::kDetailBlocked,
                                   std::format("{}; the way on from where she stands is not clear (she keeps moving away from the new "
                                               "route's start)", w.replanStray ? "pushed off the route" : "blocked"));
                            Done(w);
                            return;
                        }
                    }
                }
                const RE::NiPoint3 last = path.empty() ? w.verifiedEnd : path.back().pos;
                if (!a || !standpoint_direct::Restart(w.id, std::move(path), faceRad, w.chunked ? w.remainAfter : 0.f)) {
                    if (w.driverHas) standpoint_direct::Stop(w.id, "blocked");
                    w.endWord = "blocked";
                    Answer(w, 'O', StandpointAPI::kDetailBlocked, "blocked; the re-planned route could not be handed to the driver");
                    Done(w);
                    return;
                }
                w.verifiedEnd = last;
                w.stage.store(kDriving);
                break;
            }
            w.stage.store(kReady);
            ReadyToDrive(w);
            break;
        }
        case kReady:
            ReadyToDrive(w);
            break;
        case kDriving:
            StepDriving(w);  // 1.6.0: the next chunk / a re-aim while she walks
            break;
        default:
            break;     // holding: the driver reports
        }
    }

    // ── the pump: one frame step on the main thread, at most one queued at a time ────────────────────────────────
    bool AnythingLive()
    {
        if (standpoint_direct::LiveCount() > 0) return true;
        if (standpoint_walk_test::FollowOn()) return true;     // v1.4.1: the follow program watches the player every frame
        if (standpoint_steer::Live()) return true;             // 1.6.0 (S1 W4): a steered session needs its frame (solve, watch)
        std::scoped_lock lk(g_mtx);
        return !g_walks.empty();
    }

    void FrameStep()
    {
        g_stepQueued.store(false);
        const auto now = Clock::now();
        float dt = g_haveLastStep ? std::chrono::duration<float>(now - g_lastStep).count() : 0.f;
        g_lastStep = now;
        g_haveLastStep = true;
        if (auto* ui = RE::UI::GetSingleton(); ui && ui->GameIsPaused()) return;     // a paused game is not counted
        dt = (std::clamp)(dt, 0.f, kStepMaxDt);

        std::vector<WalkPtr> walks;
        {
            std::scoped_lock lk(g_mtx);
            walks.reserve(g_walks.size());
            for (const auto& [id, w] : g_walks) walks.push_back(w);
        }
        for (const auto& w : walks) StepWalk(*w, dt);
        standpoint_steer::Frame(dt);                 // 1.6.0 (S1 W4): the steered sessions (they may open / re-aim / hand off walks)
        standpoint_direct::Tick(dt);
        standpoint_walk_test::FollowFrame(dt);       // v1.4.1: the stpt follow program (it may open a walk: stepped next frame)
        {
            std::scoped_lock lk(g_mtx);
            for (auto it = g_walks.begin(); it != g_walks.end();) {
                if (it->second->stage.load() == kDone) {
                    it = g_walks.erase(it);
                } else {
                    ++it;
                }
            }
        }
    }

    // A small thread that hands the main thread ONE frame step about once a frame - never a second before the first ran
    // (a task that queues itself again would make SKSE drain its queue forever: a freeze). It ends when nothing is left.
    void EnsurePump()
    {
        bool expected = false;
        if (!g_pumpOn.compare_exchange_strong(expected, true)) return;
        try {
            std::thread([]() {
                for (;;) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(8));
                    if (!AnythingLive()) {
                        g_pumpOn.store(false);
                        if (!AnythingLive()) return;               // nothing came in meanwhile: done
                        bool again = false;
                        if (!g_pumpOn.compare_exchange_strong(again, true)) return;   // another pump took over
                        continue;
                    }
                    bool idle = false;
                    if (g_stepQueued.compare_exchange_strong(idle, true)) {
                        if (auto* task = SKSE::GetTaskInterface()) {
                            task->AddTask([]() { FrameStep(); });
                        } else {
                            g_stepQueued.store(false);
                        }
                    }
                }
            }).detach();
        } catch (...) {
            g_pumpOn.store(false);      // no thread could be made: the next request tries again
        }
    }

    // ── a C++ request's walker may hold a Standpoint session: released first (Papyrus), confirmed by a callback ──
    class SessionReleased final : public RE::BSScript::IStackCallbackFunctor {
    public:
        explicit SessionReleased(WalkPtr a_walk) : walk(std::move(a_walk)) {}
        void operator()(RE::BSScript::Variable) override
        {
            if (walk) walk->sessionFree.store(true);
        }
        void SetObject(const RE::BSTSmartPointer<RE::BSScript::Object>&) override {}
    private:
        WalkPtr walk;
    };

    void ReleaseHerSession(const WalkPtr& a_w, RE::Actor* a_actor)
    {
        bool sent = false;
        auto* vm = RE::BSScript::Internal::VirtualMachine::GetSingleton();
        if (vm && a_actor) {
            a_w->sessionFree.store(false);
            auto* args = RE::MakeFunctionArguments(static_cast<RE::Actor*>(a_actor));
            RE::BSTSmartPointer<RE::BSScript::IStackCallbackFunctor> done(new SessionReleased(a_w));
            sent = vm->DispatchStaticCall("StandpointAPI", "ApiWalkFreeSession", args, done);
            delete args;
        }
        if (!sent) {
            a_w->sessionFree.store(true);       // the scripts are not running: no session can exist
            logger::info("WALK #{}: StandpointAPI.ApiWalkFreeSession could not be dispatched - no session check", a_w->id);
        }
    }

    // ── a new record ─────────────────────────────────────────────────────────────────────────────────────────────
    // 1.5.0: what a WalkToWater / RequestWalkToWater asks (a_depthFrac of Open = the deepest water, depthMax)
    struct WaterAsk {
        float maxMetres{ 0.f };     // <= 0 = kWaterRangeM
        float depthMin{ 0.f };      // <= 0 = kWadeMinFrac; never above the deepest
        bool  faceWater{ true };    // she ends facing out over the water (the caller's facing < 0)
    };

    WalkPtr Open(RE::FormID a_who, const RE::NiPoint3& a_goal, float a_facingDeg, float a_hold, float a_depthFrac, float a_stopSub,
                 bool a_papyrus, StandpointAPI::WalkCallback a_cb, void* a_user, RE::FormID a_faceWho = 0, EndNoticeFn a_notice = nullptr,
                 const WaterAsk* a_water = nullptr)
    {
        if (!a_notice) CountForeign(a_who);     // v1.4.1: a walk for the follow program's walker that is not its own
        auto w = std::make_shared<Walk>();
        w->who = a_who;
        w->goal = a_goal;
        w->faceWho = a_faceWho;
        w->notice = a_notice;
        w->facingDeg = a_facingDeg < 0.f ? -1.f : std::fmod(a_facingDeg, 360.f);
        w->hold = a_hold < 0.f ? -1.f : a_hold;
        w->depthFrac = a_depthFrac <= 0.f ? kDefaultDepth : (std::clamp)(a_depthFrac, kMinDepth, kMaxDepth);
        w->stopSub = (std::clamp)(a_stopSub, 0.f, kMaxStopSub);
        w->papyrus = a_papyrus;
        w->cb = a_cb;
        w->user = a_user;
        if (a_water) {
            w->water = true;
            w->maxMetres = a_water->maxMetres > 0.f ? a_water->maxMetres : kWaterRangeM;
            w->wadeMin = (std::min)(a_water->depthMin > 0.f ? a_water->depthMin : kWadeMinFrac, w->depthFrac);
            w->faceWater = a_water->faceWater;
            w->stopSub = 0.f;
        }
        int replaced = 0;
        {
            std::scoped_lock lk(g_mtx);
            w->id = g_nextId++;
            if (g_nextId > 0x00FFFFFF) g_nextId = 1;     // stays exact as a Papyrus Int / Float
            w->gen = g_gen.load();
            for (const auto& [id, o] : g_walks) {        // her older walk / stand is REPLACED (X 6)
                int none = 0;
                if (o->who == a_who && o->stage.load() != kDone && o->stopDetail.compare_exchange_strong(none, StandpointAPI::kDetailReplaced)) ++replaced;
            }
            g_walks[w->id] = w;
            if (a_papyrus) {
                g_pap[w->id] = PapLine{};
                g_papOrder.push_back(w->id);
                while (g_papOrder.size() > kPapKeep) {
                    g_pap.erase(g_papOrder.front());
                    g_papOrder.pop_front();
                }
            }
        }
        if (w->water) {
            logger::info("WALK #{} REQUEST WATER who={:08X}: the nearest water within {:.0f} m (Water Finder), a wading spot {:.2f}-{:.2f} x "
                         "her height, facing {} hold {}{} ({})",
                         w->id, a_who, w->maxMetres, w->wadeMin, w->depthFrac,
                         w->faceWater ? std::string("the water") : std::format("{:.0f} deg", w->facingDeg),
                         w->hold < 0.f ? std::string("-1") : std::format("{:.1f} s", w->hold),
                         replaced ? std::format(" - replaces {} older walk(s) of hers (X 6)", replaced) : std::string(),
                         a_papyrus ? "Papyrus WalkToWater" : "C++ RequestWalkToWater");
            return w;
        }
        logger::info("WALK #{} REQUEST who={:08X} to ({:.0f},{:.0f},{:.0f}) facing {} hold {} depth {:.2f} x her height stopSubmerged {}{} ({})",
                     w->id, a_who, a_goal.x, a_goal.y, a_goal.z,
                     a_faceWho != 0u ? std::format("toward [{:08X}] at the arrival (now {:.0f} deg)", a_faceWho, w->facingDeg) :
                     w->facingDeg < 0.f ? std::string("keep-travel") : std::format("{:.0f} deg", w->facingDeg),
                     w->hold < 0.f ? std::string("-1") : std::format("{:.1f} s", w->hold), w->depthFrac,
                     w->stopSub > 0.f ? std::format("{:.2f}", w->stopSub) : std::string("off"),
                     replaced ? std::format(" - replaces {} older walk(s) of hers (X 6)", replaced) : std::string(),
                     a_papyrus ? "Papyrus" : "C++");
        return w;
    }

    // 1.5.0: the water walk can run here - the walk driver is live AND Water Finder 1.6.0+ answered (its C++ revision 2)
    bool WaterSupported()
    {
        return Supported() && g_wf2.load() != nullptr;
    }

    // ── natives (class Standpoint; Papyrus VM threads) ───────────────────────────────────────────────────────────
    std::int32_t WalkStartNative(RE::StaticFunctionTag*, RE::Actor* a_who, float a_x, float a_y, float a_z, float a_facingDeg,
                                 float a_holdSeconds, float a_depthFrac, float a_stopSubmerged)
    {
        if (!a_who) return 0;
        // the Papyrus door (StandpointAPI.WalkTo) has released her Standpoint session itself, before this call
        const WalkPtr w = Open(a_who->GetFormID(), RE::NiPoint3{ a_x, a_y, a_z }, a_facingDeg, a_holdSeconds, a_depthFrac,
                               a_stopSubmerged, true, nullptr, nullptr);
        EnsurePump();
        return w->id;
    }

    bool WalkStopNative(RE::StaticFunctionTag*, RE::Actor* a_who, std::int32_t a_detail)
    {
        if (!a_who) return false;
        return StopFor(a_who->GetFormID(), a_detail > 0 ? a_detail : StandpointAPI::kDetailReleased);
    }

    RE::BSFixedString WalkOutcomeNative(RE::StaticFunctionTag*, std::int32_t a_handle)
    {
        std::scoped_lock lk(g_mtx);
        const auto it = g_pap.find(a_handle);
        if (it == g_pap.end()) return RE::BSFixedString("");
        return RE::BSFixedString(std::string(1, it->second.letter));
    }

    // [distLeft u, submergedLevel, detail, x, y, z] once answered; [] while open or unknown
    std::vector<float> WalkResultValuesNative(RE::StaticFunctionTag*, std::int32_t a_handle)
    {
        std::vector<float> out;
        std::scoped_lock   lk(g_mtx);
        const auto         it = g_pap.find(a_handle);
        if (it == g_pap.end() || it->second.letter == '-') return out;
        const PapLine& p = it->second;
        out = { p.dist, p.sub, static_cast<float>(p.detail), p.x, p.y, p.z };
        return out;
    }

    bool WalkSupportedNative(RE::StaticFunctionTag*)
    {
        return Supported();
    }

    // 1.5.0 THE WADE-SPOT WALK from Papyrus (StandpointAPI.WalkToWater): she faces the water at the end. The same id space,
    // event and poll natives as WalkStart. The Papyrus door has released her Standpoint session itself (only where the water
    // walk can run: WaterWalkSupported).
    std::int32_t WalkToWaterStartNative(RE::StaticFunctionTag*, RE::Actor* a_who, float a_maxMetres, float a_depthMinFrac,
                                        float a_depthMaxFrac, float a_holdSeconds)
    {
        if (!a_who) return 0;
        const WaterAsk ask{ a_maxMetres, a_depthMinFrac, true };
        const WalkPtr  w = Open(a_who->GetFormID(), a_who->GetPosition(), -1.f, a_holdSeconds, a_depthMaxFrac, 0.f, true, nullptr,
                                nullptr, 0, nullptr, &ask);
        EnsurePump();
        return w->id;
    }

    bool WaterWalkSupportedNative(RE::StaticFunctionTag*)
    {
        return WaterSupported();
    }

    // ── the four 1.5.1 far natives (1.6.0: NEUTRAL STUBS - the far phase is gone, the DLL never starts a far session, so the
    // driver's dormant far code can only meet them with a far session that cannot exist; they stay registered so its native
    // declarations still bind) ──
    void WalkFarPackageNative(RE::StaticFunctionTag*, std::int32_t, std::int32_t, RE::TESPackage*)
    {
        // no-op: no far phase sets a package speed any more
    }

    bool WalkFarAliveNative(RE::StaticFunctionTag*, std::int32_t)
    {
        return false;      // no walk wants a far session (a stray one is freed by the driver's poll)
    }

    std::int32_t WalkReleaseModeNative(RE::StaticFunctionTag*, RE::Actor*)
    {
        return kEndLetGo;  // a session's ordinary release
    }

    std::int32_t WalkFarEndModeNative(RE::StaticFunctionTag*, std::int32_t, RE::Actor*, std::int32_t)
    {
        return kEndEvaluate;   // the override off + EvaluatePackage: never the let-go, never a stale travel package
    }

}  // namespace

std::uint32_t Request(const StandpointAPI::WalkRequest& a_req, StandpointAPI::WalkCallback a_cb, void* a_user)
{
    if (!a_cb) return 0;
    const WalkPtr w = Open(a_req.who, RE::NiPoint3{ a_req.x, a_req.y, a_req.z }, a_req.facingDegrees, a_req.holdSeconds,
                           a_req.depthFrac, a_req.stopSubmerged, false, a_cb, a_user);
    // (review H #6) only where the walk can run: on SE / AE (or with the hook not live) her session is NOT touched - the
    // walk answers B 19 at its launch, once
    if (Supported()) ReleaseHerSession(w, LookupActor(a_req.who));
    EnsurePump();
    return static_cast<std::uint32_t>(w->id);
}

std::uint32_t RequestWater(const StandpointAPI::WaterWalkRequest& a_req, StandpointAPI::WalkCallback a_cb, void* a_user)
{
    if (!a_cb) return 0;
    auto*          a = LookupActor(a_req.who);
    const WaterAsk ask{ a_req.maxMetres, a_req.depthMinFrac, a_req.facingDegrees < 0.f };
    const WalkPtr  w = Open(a_req.who, a ? a->GetPosition() : RE::NiPoint3{}, a_req.facingDegrees, a_req.holdSeconds, a_req.depthMaxFrac,
                            0.f, false, a_cb, a_user, 0, nullptr, &ask);
    // (as RequestWalk - review H #6) her session is released only where the water walk can run: driver live + Water Finder
    if (WaterSupported()) ReleaseHerSession(w, a);
    EnsurePump();
    return static_cast<std::uint32_t>(w->id);
}

void OnDataLoaded()
{
    WfHandshake hs;
    if (auto* m = SKSE::GetMessagingInterface()) m->Dispatch(WfHandshake::kGetInterface, &hs, sizeof(hs), "WaterFinder");
    auto* wf2 = hs.GetApiFunction ? static_cast<WfApi2*>(hs.GetApiFunction(2)) : nullptr;   // null before Water Finder 1.6.0
    auto* wf3 = wf2 ? static_cast<WfApi3*>(hs.GetApiFunction(3)) : nullptr;                  // null before Water Finder 1.6.2
    g_wf2.store(wf2);
    g_wf3.store(wf3);
    logger::info("water walk (WalkToWater / RequestWalkToWater): Water Finder {}",
                 !hs.GetApiFunction ? "is not installed - a water walk answers B 25" :
                 !wf2               ? "is older than 1.6.0 (no C++ revision 2) - a water walk answers B 25" :
                 wf3                ? "1.6.2+ answered (revisions 2 and 3: the seed, then other water if none is reachable)" :
                                      "1.6.0+ answered (revision 2: one seed; other water candidates need Water Finder 1.6.2)");
}

std::uint32_t RequestFacing(const StandpointAPI::WalkRequest& a_req, RE::FormID a_faceWho, EndNoticeFn a_notice, void* a_user,
                            bool a_freeSession)
{
    if (!a_notice) return 0;
    const WalkPtr w = Open(a_req.who, RE::NiPoint3{ a_req.x, a_req.y, a_req.z }, a_req.facingDegrees, a_req.holdSeconds,
                           a_req.depthFrac, a_req.stopSubmerged, false, nullptr, a_user, a_faceWho, a_notice);
    // (review R #3) only the user's own start of the program may release a session of hers; a later follow walk never
    // ends another owner's session (sessionFree stays true: nothing is dispatched)
    if (a_freeSession && Supported()) ReleaseHerSession(w, LookupActor(a_req.who));
    EnsurePump();
    return static_cast<std::uint32_t>(w->id);
}

void WatchForeign(RE::FormID a_who)
{
    g_watchWho.store(a_who);
}

void NoteForeign(RE::FormID a_who)
{
    CountForeign(a_who);
}

namespace {
    // 1.6.0 a steered ask into its walk record (RequestSteer, Retarget) - main thread
    void ApplyAsk(Walk& w, const SteerAsk& a_ask)
    {
        w.goal = a_ask.spot;
        // (planTo left at (0,0,0) = not given: the plan goes to the spot itself)
        const bool planToGiven = a_ask.planTo.x != 0.f || a_ask.planTo.y != 0.f || a_ask.planTo.z != 0.f;
        w.planTo = !a_ask.fixedRoute.empty() ? a_ask.fixedRoute.back() : planToGiven ? a_ask.planTo : a_ask.spot;
        w.tail = a_ask.tail;
        w.fixedRoute = a_ask.fixedRoute;
        w.avoid = a_ask.avoid;
        w.planFlags = a_ask.planFlags;
        w.depthFrac = a_ask.depthFrac <= 0.f ? kDefaultDepth : (std::clamp)(a_ask.depthFrac, kMinDepth, kMaxDepth);
        w.facingDeg = a_ask.facingDeg < 0.f ? -1.f : std::fmod(a_ask.facingDeg, 360.f);
        w.faceWho = a_ask.faceWho;
        w.faceOffsetDeg = a_ask.faceOffsetDeg;
        w.faceAll = a_ask.faceAll;
        w.nearRadius = (std::max)(0.f, a_ask.nearRadius);
    }

    // 1.6.0 (the follow program): her walk of this handle is still on its way - no answer yet, no stop marked
    bool OnItsWay(std::int32_t a_handle)
    {
        const WalkPtr w = Find(a_handle);
        if (!w) return false;
        const int stage = w->stage.load();
        return stage != kDone && stage != kHolding && !w->answered && w->stopDetail.load() == 0;
    }

    // 1.6.0 (the follow program, S2 §2.4 "stpf re-targets"): her plain walk aims at a new point WITHOUT a stop (the Retarget
    // mechanism). false = it is not on its way any more (a new walk then)
    bool RetargetGoal(std::int32_t a_handle, const RE::NiPoint3& a_goal, float a_facingDeg)
    {
        const WalkPtr w = Find(a_handle);
        if (!w || w->steer || w->water) return false;
        const int stage = w->stage.load();
        if (stage == kDone || stage == kHolding || w->answered || w->stopDetail.load() != 0) return false;
        w->goal = a_goal;
        w->facingDeg = a_facingDeg < 0.f ? -1.f : std::fmod(a_facingDeg, 360.f);
        if (w->driverHas) {
            standpoint_direct::SetFacing(w->id, w->facingDeg >= 0.f, w->facingDeg * kDegToRad, w->faceWho, w->faceOffsetDeg * kDegToRad,
                                         w->faceAll);
        }
        logger::info("WALK #{} RETARGET (the follow program, stage {}): the new point ({:.0f},{:.0f},{:.0f})", w->id, StageName(stage), a_goal.x,
                     a_goal.y, a_goal.z);
        return Reaim(*w, "the follow program's point moved");
    }

    std::string AskText(const Walk& w)
    {
        return std::format("spot ({:.0f},{:.0f},{:.0f}), {}{}{}, avoid {}, flags {}, facing {}{}, near zone {}, hold {}", w.goal.x, w.goal.y,
                           w.goal.z,
                           !w.fixedRoute.empty() ? std::format("a FIXED route of {} point(s) (no plan)", w.fixedRoute.size())
                                                 : std::format("planned to ({:.0f},{:.0f},{:.0f})", w.planTo.x, w.planTo.y, w.planTo.z),
                           w.tail.empty() ? std::string() : std::format(" + a fixed last leg of {} point(s)", w.tail.size()),
                           w.depthFrac != kDefaultDepth ? std::format(", depth {:.2f} x her height", w.depthFrac) : std::string(),
                           w.avoid ? std::format("[{:08X}] (its keep-out circle)", w.avoid) : std::string("none"),
                           w.planFlags ? std::format("0x{:X} (+ kAllDirect)", w.planFlags) : std::string("the MIX"),
                           w.faceWho ? std::format("toward [{:08X}] live {:+.0f} deg", w.faceWho, w.faceOffsetDeg)
                                     : w.facingDeg < 0.f ? std::string("keep-travel") : std::format("{:.0f} deg", w.facingDeg),
                           w.faceAll ? " held over the whole route (faceAll)" : "",
                           w.nearRadius > 0.f ? std::format("{:.0f} u", w.nearRadius) : std::string("off"),
                           w.hold < 0.f ? std::string("-1") : std::format("{:.1f} s", w.hold));
    }
}  // namespace

std::uint32_t RequestSteer(const SteerAsk& a_ask)
{
    if (!a_ask.notice || !a_ask.who) return 0;
    // Open counts it as a foreign request for the follow program (no EndNoticeFn): a steered session's walk ends stpf for her (S1
    // W1 NoteForeign); her older walk / stand is replaced (X 6) and holds her until this walk's drive takes her
    const WalkPtr w = Open(a_ask.who, a_ask.spot, a_ask.facingDeg, a_ask.hold, a_ask.depthFrac, 0.f, false, nullptr, a_ask.user,
                           a_ask.faceWho);
    w->steer = true;
    w->steerId = a_ask.steerId;
    w->steerNotice = a_ask.notice;
    ApplyAsk(*w, a_ask);
    logger::info("WALK #{}{} STEER ASK (a steered session's walk - her session is not released): {}", w->id, SteerTag(*w), AskText(*w));
    EnsurePump();
    return static_cast<std::uint32_t>(w->id);
}

bool Retarget(std::uint32_t a_handle, const SteerAsk& a_ask)
{
    const WalkPtr w = Find(static_cast<std::int32_t>(a_handle));
    if (!w || !w->steer || w->holdHere) return false;
    const int stage = w->stage.load();
    if (stage == kDone || stage == kHolding || w->answered || w->stopDetail.load() != 0) return false;
    ApplyAsk(*w, a_ask);
    logger::info("WALK #{}{} RETARGET (stage {}): {}", w->id, SteerTag(*w), StageName(stage), AskText(*w));
    if (w->driverHas) {
        standpoint_direct::SetFacing(w->id, w->facingDeg >= 0.f, w->facingDeg * kDegToRad, w->faceWho, w->faceOffsetDeg * kDegToRad,
                                     w->faceAll);
    }
    const bool ok = Reaim(*w, "the steered session re-aims (its target moved)");
    EnsurePump();
    return ok;
}

bool HandOff(std::uint32_t a_handle)
{
    const WalkPtr w = Find(static_cast<std::int32_t>(a_handle));
    if (!w || w->stage.load() == kDone) return false;
    w->handedOff = true;
    w->endWord = "session";
    if (w->driverHas) standpoint_direct::Stop(w->id, "session");     // the hook hands her back at its next visit + EvaluatePackage
    if (w->inheritId) {
        standpoint_direct::Stop(w->inheritId, "session");             // an older walk's walker that still held her for this one
        w->inheritId = 0;
    }
    w->driverHas = false;
    if (!w->answered) {
        Answer(*w, 'X', StandpointAPI::kDetailReleased, "the steered session took her over (HandOff) before the walk answered");
    } else {
        logger::info("WALK #{}{} HOLD END: session (HandOff: the steered session takes her over - the pin; the answer was given at the "
                     "arrival)", w->id, SteerTag(*w));
        NoticeHoldEnd(*w, 'X', StandpointAPI::kDetailReleased);
    }
    Done(*w);
    return true;
}

std::uint32_t HoldHere(RE::FormID a_who, float a_hold, std::int32_t a_steerId, SteerNoticeFn a_notice, void* a_user,
                       RE::FormID a_faceWho, float a_faceOffsetDeg)
{
    if (!a_who) return 0;
    auto* const   a = LookupActor(a_who);
    const WalkPtr w = Open(a_who, a ? a->GetPosition() : RE::NiPoint3{}, -1.f, a_hold, 0.f, 0.f, false, nullptr, a_user, a_faceWho);
    w->steer = true;
    w->steerId = a_steerId;
    w->steerNotice = a_notice;
    w->holdHere = true;
    w->faceOffsetDeg = a_faceOffsetDeg;
    EnsurePump();
    return static_cast<std::uint32_t>(w->id);
}

void Wake()
{
    EnsurePump();
}

std::uint32_t ForeignCount()
{
    return g_watchForeign.load();
}

bool HandleOwned(std::int32_t a_handle)
{
    const WalkPtr w = Find(a_handle);
    return w && w->stage.load() != kDone && w->stopDetail.load() == 0;
}

bool IsDirectWalking(std::uint32_t a_formId)
{
    return standpoint_direct::IsDirectWalking(a_formId);
}

bool Supported()
{
    return standpoint_direct::HookLive();
}

bool StopFor(RE::FormID a_who, std::int32_t a_detail)
{
    if (!a_who) return false;
    CountForeign(a_who);    // v1.4.1: WalkStop (Release, a session start) or a C++ release line - never the follow program's own
    bool found = false;
    {
        std::scoped_lock lk(g_mtx);
        for (const auto& [id, w] : g_walks) {
            if (w->who != a_who || w->stage.load() == kDone) continue;
            int none = 0;
            w->stopDetail.compare_exchange_strong(none, a_detail);
            found = true;
        }
    }
    if (found) EnsurePump();
    return found;
}

bool StopHandle(std::int32_t a_handle, std::int32_t a_detail)
{
    const WalkPtr w = Find(a_handle);
    if (!w || w->stage.load() == kDone) return false;
    int        none = 0;
    const bool mine = w->stopDetail.compare_exchange_strong(none, a_detail);   // (review R nit 13) false: a stop / replacement
    EnsurePump();                                                               // was already marked - she is not ours to let go
    return mine;
}

bool HandleLive(std::int32_t a_handle)
{
    const WalkPtr w = Find(a_handle);
    return w && w->stage.load() != kDone;
}

void OnSaveGame()
{
    standpoint_direct::OnSaveGame();     // (1.6.0: no far phase - no package speed to put back)
}

void OnGameLoad()
{
    std::vector<WalkPtr> open;
    {
        std::scoped_lock lk(g_mtx);
        ++g_gen;                                  // every Papyrus walk of the old game is forgotten - never answered
        g_pap.clear();
        g_papOrder.clear();
        for (const auto& [id, w] : g_walks) open.push_back(w);
        g_walks.clear();
    }
    standpoint_direct::OnPreLoad();               // forget, touch nothing: her 3D is going away
    for (const auto& w : open) {
        w->driverHas = false;
        CancelJob(*w);                            // (1.6.0: a running skeleton too)
        Answer(*w, 'X', StandpointAPI::kDetailGameLoaded, "a game is being loaded", false);   // C++ callbacks only
        w->stage.store(kDone);
    }
    g_haveLastStep = false;
}

bool RegisterNatives(RE::BSScript::IVirtualMachine* a_vm)
{
    a_vm->RegisterFunction("WalkStart", "Standpoint", WalkStartNative);
    a_vm->RegisterFunction("WalkStop", "Standpoint", WalkStopNative);
    a_vm->RegisterFunction("WalkOutcome", "Standpoint", WalkOutcomeNative);
    a_vm->RegisterFunction("WalkResultValues", "Standpoint", WalkResultValuesNative);
    a_vm->RegisterFunction("WalkSupported", "Standpoint", WalkSupportedNative);
    a_vm->RegisterFunction("WalkToWaterStart", "Standpoint", WalkToWaterStartNative);     // 1.5.0
    a_vm->RegisterFunction("WaterWalkSupported", "Standpoint", WaterWalkSupportedNative); // 1.5.0
    a_vm->RegisterFunction("WalkFarPackage", "Standpoint", WalkFarPackageNative);         // 1.5.1 - 1.6.0: a neutral stub (no-op)
    a_vm->RegisterFunction("WalkFarAlive", "Standpoint", WalkFarAliveNative);             // 1.5.1 - 1.6.0: a neutral stub (False)
    a_vm->RegisterFunction("WalkFarEndMode", "Standpoint", WalkFarEndModeNative);         // 1.5.1 - 1.6.0: a neutral stub (2)
    a_vm->RegisterFunction("WalkReleaseMode", "Standpoint", WalkReleaseModeNative);       // 1.5.1 - 1.6.0: a neutral stub (0)
    logger::info("walk natives registered (class Standpoint, 11 functions; the 4 far ones are neutral stubs); the walk off the navmesh is "
                 "{}; 1.6.0: our own path up to 200 m (the navmesh skeleton + verified chunks, no engine travel), the speed bands (run "
                 "beyond 15 m, jog to 3 m, walk the last 3 m), O 26 beyond the loaded area",
                 Supported() ? std::string("AVAILABLE (the motion-check hook is live)")
                             : std::format("NOT available (B 19): {}", standpoint_direct::NotLiveText()));
    return true;
}

}  // namespace standpoint_walk

// ═════════════════════════════════════════════════════════════════════════════
//  THE stpt WALK PROGRAMS (test only - never part of the API)
// ═════════════════════════════════════════════════════════════════════════════
namespace standpoint_walk_test {
namespace {

    // the WATER program (1.5.0: it is a WalkToWater - Standpoint asks Water Finder itself, the planner's wade-spot search
    // picks the wading spot she can reach, she ends facing the water; D §3 item 4: 20 m, depth 0.15-0.6, hold 20 s)
    // (1.6.0, spec D §1 item 10 / S2 §6.5: the deepest = the THIGH MAX 0.45 - "thigh is the max depth"; the wade search's knee /
    //  thigh scans run inside it)
    constexpr float kWaterRangeM    = 20.f;   // water within 20 m of her
    constexpr float kWaterHoldS     = 20.f;
    constexpr float kWaterDepthMin  = 0.15f;  // the shallowest wading spot (the shin entry), x her height
    constexpr float kWaterDepthFrac = 0.45f;  // the deepest: the thigh MAX (1.5.x: 0.6 = waist - never used now)
    constexpr float kFrontDistU     = 210.f;  // 3 m
    constexpr float kFrontHoldS     = 10.f;
    constexpr float kPi             = 3.14159265f;

    std::uint32_t g_handle = 0;
    RE::FormID    g_handleWho = 0;    // v1.4.1: whose single walk test g_handle is (the follow program replaces or stops it)
    std::uint32_t g_quietHandle = 0;  // v1.4.1 (review R nit 13): a single test the follow program replaced - its X 6, log only

    void Say(const std::string& a_msg, bool a_corner = false)
    {
        logger::info("[TEST] {}", a_msg);
        if (auto* c = RE::ConsoleLog::GetSingleton()) c->Print("%s", a_msg.c_str());
        if (a_corner) RE::SendHUDMessage::ShowHUDMessage(a_msg.c_str());
    }

    std::string NameOf(RE::Actor* a_actor)
    {
        if (!a_actor) return "?";
        const char* n = a_actor->GetDisplayFullName();
        return std::format("{} [{:08X}]", n && n[0] ? n : "(no name)", a_actor->GetFormID());
    }

    void OnAnswer(std::uint32_t a_handle, const StandpointAPI::WalkResult& a_r, void*)
    {
        if (a_handle != 0u && a_handle == g_quietHandle) {
            logger::info("[TEST] Standpoint walk test #{}: {} {:.0f} - replaced by the follow program (no headset line)", a_handle,
                         a_r.outcome, a_r.detail);
            g_quietHandle = 0;
            return;
        }
        Say(std::format("Standpoint walk test #{}: {} {:.0f} - {:.0f} u from the point, submerged {:.2f}, at ({:.0f},{:.0f},{:.0f})",
                        a_handle, a_r.outcome, a_r.detail, a_r.distLeft, a_r.submerged, a_r.x, a_r.y, a_r.z), false);
        RE::SendHUDMessage::ShowHUDMessage(std::format("Walk test: {} {:.0f}, {:.0f} u off", a_r.outcome, a_r.detail, a_r.distLeft).c_str());
        if (a_handle == g_handle && a_r.outcome == 'C') {
            Say("  she holds the spot now (driven at speed 0); the test ends with the hold - stpt stops it earlier");
        }
    }

    float Compass(float a_rad)
    {
        float d = a_rad * 57.2957795f;
        while (d < 0.f) d += 360.f;
        while (d >= 360.f) d -= 360.f;
        return d;
    }

}  // namespace

bool Running()
{
    return g_handle != 0 && standpoint_walk::HandleLive(static_cast<std::int32_t>(g_handle));
}

void StopRunning()
{
    if (g_handle) standpoint_walk::StopHandle(static_cast<std::int32_t>(g_handle), StandpointAPI::kDetailReleased);
    g_handle = 0;
}

void StartWaterWalk(RE::Actor* a_walker)
{
    if (!a_walker) return;
    // 1.5.0: the same path as IStandpoint4::RequestWalkToWater (no Water Finder = its answer B 25, no water = O 23, no wading
    // spot she can reach = O 24 - each printed by OnAnswer)
    StandpointAPI::WaterWalkRequest q{};
    q.who = a_walker->GetFormID();
    q.maxMetres = kWaterRangeM;
    q.depthMinFrac = kWaterDepthMin;
    q.depthMaxFrac = kWaterDepthFrac;
    q.holdSeconds = kWaterHoldS;
    q.facingDegrees = -1.f;                        // she faces the water
    g_handle = standpoint_walk::RequestWater(q, &OnAnswer, nullptr);
    g_handleWho = q.who;
    Say(std::format("Standpoint walk test (WATER, WalkToWater): WALK #{}: {} walks to the nearest wading spot she can reach within "
                    "{:.0f} m (depth {:.2f}-{:.2f} x her height), faces the water, holds {:.0f} s - close the console. stpt stops it. "
                    "Every line is in Standpoint.log (WALK #{}).",
                    g_handle, NameOf(a_walker), kWaterRangeM, kWaterDepthMin, kWaterDepthFrac, kWaterHoldS, g_handle), true);
}

void StartFrontWalk(RE::Actor* a_walker)
{
    auto* pc = RE::PlayerCharacter::GetSingleton();
    if (!a_walker || !pc) return;
    const float        yaw = pc->GetAngleZ();
    const RE::NiPoint3 p = pc->GetPosition();
    StandpointAPI::WalkRequest q{};
    q.who = a_walker->GetFormID();
    q.x = p.x + kFrontDistU * std::sin(yaw);
    q.y = p.y + kFrontDistU * std::cos(yaw);
    q.z = p.z;
    q.facingDegrees = Compass(yaw + kPi);          // she ends facing the player
    q.holdSeconds = kFrontHoldS;
    q.depthFrac = 0.f;                             // the default (0.45)
    q.stopSubmerged = 0.f;
    g_handle = standpoint_walk::Request(q, &OnAnswer, nullptr);
    g_handleWho = q.who;
    Say(std::format("Standpoint walk test (3 m IN FRONT OF YOU): WALK #{}: {} walks to ({:.0f},{:.0f},{:.0f}), facing you, hold {:.0f} s "
                    "- close the console. stpt stops it.", g_handle, NameOf(a_walker), q.x, q.y, q.z, kFrontHoldS), true);
}

// ═════════════════════════════════════════════════════════════════════════════
//  v1.4.1 THE FOLLOW-FRONT PROGRAM - `stpf` (TestCmd.cpp), no sneaking. The user, 2026-11-03: "DO a different console command
//  for having a NPC arrive 3m in front of me, and not doing only one target, like it stay on until i type STPT again". Every
//  walk is an ordinary walk off the navmesh (RequestFacing: the API's path + "face the player" read live + every end
//  reported); between walks she stands, driven (hold -1). No timer of its own: the one window is the stillness below.
//  Review R (2026-11-03): (#1) in VR the player's GetAngleZ follows the HEADSET (KNOWLEDGEBASE: an 87 deg head-only turn moved
//  it 87.3 deg), so the point moves with every glance - a new walk only once the POINT itself and he have both been still
//  for 0.5 s, for every rule; (#3) ONE owner at a time - any other request, release, grab or driver for her ends the
//  program, which never takes her back and never ends another owner's session.
// ═════════════════════════════════════════════════════════════════════════════
namespace {

    constexpr float kFollowMoveU = 64.f;   // the point has moved this far from her walk's point: a new walk ...
    constexpr float kStillMoveU  = 8.f;    // ... once the point AND he stay within this of where they stopped (8 u of the
    constexpr float kStillS      = 0.5f;   // ... point = 2 deg of his yaw) for this long: a glance or a step never starts one

    struct FollowState {
        RE::FormID    walker{ 0 };
        std::uint32_t handle{ 0 };       // her current / last walk
        int           k{ 0 };            // walks sent (FOLLOW #k)
        RE::NiPoint3  goal{};            // the point asked of her current / last walk (his z: the planner finds the floor)
        bool          haveGoal{ false };
        bool          failed{ false };   // her current walk / stand ended for a reason that is no other owner (combat, no route)
        char          last{ '-' };       // that end's letter ('-' while the walk is open; 'C' after its answer C)
        std::uint32_t foreign{ 0 };      // standpoint_walk::ForeignCount() when the program started (any change = another owner)
        RE::NiPoint3  anchorP{};         // where he and ...
        RE::NiPoint3  anchorT{};         // ... the point stopped (the stillness watch) ...
        float         stillT{ 0.f };     // ... and how long both have stayed within kStillMoveU of it (pause excluded)
        // 1.6.0 W5 (S1 §6): the driver's gate refuses her (sitting, mounted, combat ...): nothing is sent, ONE line per episode
        bool          waiting{ false };
        bool          fromHer{ false };  // the trigger is "the point is >= 64 u from HER" - only after a sitting wait (1.6.1 B1:
                                         // not after other failures, and never during a walk: FollowWalk clears it)
        int           reaims{ 0 };       // re-aims of her walk on its way (the Retarget mechanism)
    };

    std::atomic<bool> g_followOn{ false };    // also read by the walk pump's thread
    std::atomic<int>  g_followGen{ 0 };       // +1 at every start / stop / load: an older program's notices are only logged
    FollowState       g_follow;               // main thread

    // the point 3 m (210 u) in front of the player's facing (GetAngleZ - in VR the HEADSET's yaw), at his z
    RE::NiPoint3 FrontOf(RE::PlayerCharacter* a_pc, float& a_yaw)
    {
        a_yaw = a_pc->GetAngleZ();
        const RE::NiPoint3 p = a_pc->GetPosition();
        return { p.x + kFrontDistU * std::sin(a_yaw), p.y + kFrontDistU * std::cos(a_yaw), p.z };
    }

    // (review R #3) the end words that say another owner has her now
    bool OtherOwnerWord(const char* a_word)
    {
        const std::string_view w(a_word ? a_word : "");
        return w == "replaced" || w == "released" || w == "grabbed" || w == "taken" || w == "other-driver";
    }

    // every end of a follow walk: its answer, and after a C the end of her stand (WalkApi's internal notice, main thread)
    void OnFollowNotice(std::uint32_t a_handle, const StandpointAPI::WalkResult& a_r, const char* a_word, bool a_holdEnd, void* a_user)
    {
        const int   gen = static_cast<int>(reinterpret_cast<std::intptr_t>(a_user));
        const bool  mine = g_followOn.load() && gen == g_followGen.load();
        const char* word = a_word && a_word[0] ? a_word : "-";
        if (!mine || a_handle != g_follow.handle) {
            // a walk the program has replaced since (X 6), or one of an ended program (X 3 / X 12): the log only
            logger::info("[TEST] FOLLOW: walk #{} {} {} {:.0f} ({}), {:.0f} u off - {}", a_handle, a_holdEnd ? "stand ended" : "answered",
                         a_r.outcome, a_r.detail, word, a_r.distLeft,
                         mine ? "an earlier walk of the program, replaced by the next one" : "the program has ended");
            return;
        }
        const auto detail = static_cast<std::int32_t>(a_r.detail);
        const bool other = a_r.outcome == 'P' || (a_r.outcome == 'B' && detail == StandpointAPI::kDetailOtherDriver) ||
                           (a_r.outcome == 'X' && (detail == StandpointAPI::kDetailReplaced || detail == StandpointAPI::kDetailReleased)) ||
                           OtherOwnerWord(word);
        if (other) {
            // (review R #3) another owner has her (a request, a release, the player's hand, another driver): the program ends,
            // she is not taken back
            StopFollow(std::format("{} {} {} - another owner has her", a_r.outcome, detail, word).c_str());
            return;
        }
        if (!a_holdEnd && a_r.outcome == 'C') {
            g_follow.last = 'C';
            g_follow.failed = false;
            g_follow.fromHer = false;
            Say(std::format("FOLLOW #{} C {}: {:.0f} u off - she stands, facing you", g_follow.k, detail, a_r.distLeft));
            return;
        }
        g_follow.last = a_r.outcome;
        g_follow.failed = true;
        if (std::string_view(word) == "furniture") {
            // 1.6.0 W5: she sat down (her own AI) - a wait, not a failure: the program waits until the gate lets her go; then the
            // point is measured from HER (her own AI may have walked her far from the old point)
            g_follow.fromHer = true;
            Say(std::format("FOLLOW #{} {} {} (furniture): she sits - the next walk once she is free and the point is 64 u from her",
                            g_follow.k, a_r.outcome, detail));
            return;
        }
        // (1.6.1 B1, run 2026-11-06: 33 x O 1 in 21 s) any other failure keeps the 1.4.1 rule: the next walk only once the POINT has
        // moved 64 u from this walk's point (and stayed) - never "64 u from HER", which re-sent a walk every 0.5 s while she could
        // not get there with the player standing still
        Say(std::format("FOLLOW #{} {} {} ({}){}: {:.0f} u off - next walk once the point moves 64 u and stays", g_follow.k, a_r.outcome,
                        detail, word, a_holdEnd ? ", her stand ended" : "", a_r.distLeft));
    }

    // 1.6.0 W5: the gate's word for the one HUD line of a waiting episode (under 15 words, under 75 characters)
    const char* WaitWord(standpoint_direct::End a_end)
    {
        using E = standpoint_direct::End;
        switch (a_end) {
        case E::kFurniture:  return "Follow: she sits - waiting till she stands";
        case E::kMounted:    return "Follow: she rides - waiting till she dismounts";
        case E::kCombat:     return "Follow: she fights - waiting till combat ends";
        case E::kBleedout:   return "Follow: she is down - waiting till she recovers";
        case E::kRagdoll:
        case E::kKnocked:    return "Follow: she was knocked down - waiting till she stands";
        case E::kRestrained: return "Follow: she is restrained - waiting till she is free";
        case E::kKillMove:   return "Follow: a kill move - waiting till it ends";
        default:             return "Follow: she cannot walk now - waiting";
        }
    }

    // one walk to the point a_t (3 m in front of him), facing him at the arrival (hold -1: she then stands, driven).
    // a_freeSession: only the user's own start of the program may release a Standpoint session of hers (review R #3)
    void FollowWalk(RE::PlayerCharacter* a_pc, const RE::NiPoint3& a_t, float a_yaw, const std::string& a_why, bool a_freeSession)
    {
        StandpointAPI::WalkRequest q{};
        q.who = g_follow.walker;
        q.x = a_t.x;
        q.y = a_t.y;
        q.z = a_t.z;
        q.facingDegrees = Compass(a_yaw + kPi);     // toward him from the point (the fallback: the driver reads him live)
        q.holdSeconds = -1.f;
        q.depthFrac = 0.f;                          // the default (0.45)
        q.stopSubmerged = 0.f;
        ++g_follow.k;
        g_follow.handle = standpoint_walk::RequestFacing(q, a_pc->GetFormID(), &OnFollowNotice,
                                                        reinterpret_cast<void*>(static_cast<std::intptr_t>(g_followGen.load())),
                                                        a_freeSession);
        g_follow.goal = a_t;
        g_follow.haveGoal = true;
        g_follow.failed = false;
        g_follow.fromHer = false;                   // (1.6.1 B1) this walk's point is the reference from now on - measured from HER
                                                    // during a walk, every 0.5 s re-aimed her with the player standing still
        g_follow.last = '-';
        g_follow.anchorP = a_pc->GetPosition();     // (review R #1) the stillness clock starts again with every walk
        g_follow.anchorT = a_t;
        g_follow.stillT = 0.f;
        Say(std::format("FOLLOW #{} walk #{}: 3 m in front of you - {}", g_follow.k, g_follow.handle, a_why));
    }

}  // namespace

bool FollowOn()
{
    return g_followOn.load();
}

void StartFollow(RE::Actor* a_walker)
{
    auto* pc = RE::PlayerCharacter::GetSingleton();
    if (!a_walker || !pc) return;
    if (!standpoint_walk::Supported()) {
        Say("Standpoint FOLLOW: Skyrim VR only - the walk driver is not installed here (B 19).", true);
        return;
    }
    if (g_handle) {
        // a single walk test (front / water) still runs: hers is replaced by the first follow walk (X 6 - she stays held; its
        // answer is logged only - review R nit 13); another NPC's is stopped (X 3 - she is let go)
        if (g_handleWho != a_walker->GetFormID()) {
            StopRunning();
        } else {
            g_quietHandle = g_handle;
        }
        g_handle = 0;
    }
    ++g_followGen;
    g_follow = FollowState{};
    g_follow.walker = a_walker->GetFormID();
    standpoint_walk::WatchForeign(g_follow.walker);         // (review R #3) any other request / stop for her from now on
    g_follow.foreign = standpoint_walk::ForeignCount();
    g_followOn.store(true);
    logger::info("[TEST] FOLLOW START: {} keeps to the point {:.0f} u in front of the player's facing (GetAngleZ: in VR the "
                 "headset's yaw; z = his), facing him, hold -1 between walks. A new walk when that point has moved {:.0f} u or more "
                 "from her walk's point AND the point and he have both stayed within {:.0f} u for {:.1f} s. Ends: stpt, stpf "
                 "again, a game load, her death or unload, and any other owner (a request, a release, a grab, another driver).",
                 NameOf(a_walker), kFrontDistU, kFollowMoveU, kStillMoveU, kStillS);
    Say(std::format("Standpoint FOLLOW on: {} stays 3 m in front of you - stpt stops it.", NameOf(a_walker)));
    const char* shown = a_walker->GetDisplayFullName();
    RE::SendHUDMessage::ShowHUDMessage(std::format("Follow on: {} - stpt stops", shown && shown[0] ? shown : "she").c_str());
    float              yaw = 0.f;
    const RE::NiPoint3 t = FrontOf(pc, yaw);
    FollowWalk(pc, t, yaw, "the start", true);    // the user's own command: a session of hers is released, as by any WalkTo
}

void StopFollow(const char* a_why)
{
    if (!g_followOn.exchange(false)) return;
    ++g_followGen;                    // her walk's notices are only logged from now on
    standpoint_walk::WatchForeign(0);
    // (review R #3 / nit 13) she is let go only when the program still owns her: StopHandle is true only when OUR stop won
    const bool ours = g_follow.handle != 0u &&
                      standpoint_walk::StopHandle(static_cast<std::int32_t>(g_follow.handle), StandpointAPI::kDetailReleased);
    Say(std::format("Standpoint FOLLOW off ({}): {} walk(s) - {}.", a_why ? a_why : "stopped", g_follow.k,
                    ours ? "she is let go (X 3)" : "she was not the program's any more: not touched"));
    RE::SendHUDMessage::ShowHUDMessage(std::format("Follow off: {}", a_why ? a_why : "stopped").c_str());
    g_follow = FollowState{};
}

void FollowFrame(float a_dt)
{
    if (!g_followOn.load()) return;
    auto* a = g_follow.walker ? RE::TESForm::LookupByID<RE::Actor>(g_follow.walker) : nullptr;
    if (!a) {
        StopFollow("she is gone");
        return;
    }
    if (a->IsDead()) {
        StopFollow("she died");
        return;
    }
    if (!a->Is3DLoaded()) {
        StopFollow("she is not loaded any more");
        return;
    }
    // (review R #3) ONE owner: any request / stop for her that was not the program's (WalkTo, GoTo / GoToPoint / RequestMove -
    // their session start - a Release, a release line) ends it; she is not taken back and no other session is touched
    if (standpoint_walk::ForeignCount() != g_follow.foreign) {
        StopFollow("another request or a release came for her - not taken back");
        return;
    }
    if (standpoint_interrupts::IsPlayerGrabbing(a)) {        // (the walk's own P 8 notice says it a frame later: send nothing)
        StopFollow("the player's hand holds her");
        return;
    }
    auto* pc = RE::PlayerCharacter::GetSingleton();
    if (!pc) return;
    // (review R #1) the stillness watch: the POINT (his gaze in VR) and he both stay within kStillMoveU of where they stopped
    const RE::NiPoint3 p = pc->GetPosition();
    float              yaw = 0.f;
    const RE::NiPoint3 t = FrontOf(pc, yaw);
    // 1.6.0 W5 (S1 §6, spec D §1 item 3): while the driver's gate refuses her (she sits / sleeps / uses furniture, rides, fights,
    // bleeds out, is restrained, knocked down ...) NOTHING is sent - one log + HUD line per episode, no timer, nothing makes her
    // stand. Once she is free: the stillness clock starts again and the trigger is "the point is >= 64 u from HER" (her own AI may
    // have walked her far from the old point while the point stayed)
    {
        std::string gateText;
        const auto  g = standpoint_direct::Gate(a, gateText);
        if (g != standpoint_direct::End::kNone) {
            if (!g_follow.waiting) {
                g_follow.waiting = true;
                logger::info("[TEST] FOLLOW: {} - {}: nothing is sent while the gate refuses her ({})", NameOf(a), WaitWord(g), gateText);
                RE::SendHUDMessage::ShowHUDMessage(WaitWord(g));
            }
            return;
        }
        if (g_follow.waiting) {
            g_follow.waiting = false;
            g_follow.fromHer = true;
            g_follow.anchorP = p;
            g_follow.anchorT = t;
            g_follow.stillT = 0.f;
            logger::info("[TEST] FOLLOW: {} is free again - the next walk once the point is {:.0f} u from her and he stays still {:.1f} s",
                         NameOf(a), kFollowMoveU, kStillS);
            return;
        }
    }
    if ((p - g_follow.anchorP).Length() >= kStillMoveU || (t - g_follow.anchorT).Length() >= kStillMoveU) {
        g_follow.anchorP = p;
        g_follow.anchorT = t;
        g_follow.stillT = 0.f;
        return;
    }
    g_follow.stillT += a_dt;
    if (g_follow.stillT < kStillS || !g_follow.haveGoal) return;
    // (1.6.0 W5 / 1.6.1 B1: after a SITTING wait the point is measured from HER, flat - else, failed walks too, from her walk's point)
    const float moved = g_follow.fromHer ? std::hypot(t.x - a->GetPosition().x, t.y - a->GetPosition().y) : (t - g_follow.goal).Length();
    if (moved < kFollowMoveU) return;
    // 1.6.0 (S2 §2.4 "stpf re-targets"): her walk is still on its way - it is RE-AIMED (no stop: planned from where she will be,
    // swapped in while she walks); she only gets a new walk once she stands
    const auto h = static_cast<std::int32_t>(g_follow.handle);
    if (!g_follow.failed && standpoint_walk::OnItsWay(h) && standpoint_walk::RetargetGoal(h, t, Compass(yaw + kPi))) {
        ++g_follow.reaims;
        g_follow.goal = t;
        g_follow.anchorP = p;
        g_follow.anchorT = t;
        g_follow.stillT = 0.f;
        Say(std::format("FOLLOW #{} walk #{}: RE-AIMED on the move (#{}): point moved {:.0f} u", g_follow.k, g_follow.handle, g_follow.reaims,
                        moved));
        return;
    }
    // her walk / stand must still be the program's, with nothing marked on it - or have ended for a reason that is no other
    // owner (failed: combat, no route ...). Anything else: its end notice arrives in a frame - send nothing meanwhile.
    if (!g_follow.failed && !standpoint_walk::HandleOwned(h)) return;
    FollowWalk(pc, t, yaw,
               g_follow.failed ? std::format("last walk {}, point {:.0f} u {} and stayed {:.1f} s", g_follow.last, moved,
                                             g_follow.fromHer ? "from her" : "from its point", g_follow.stillT)
                               : std::format("point moved {:.0f} u{} and stayed {:.1f} s", moved, g_follow.fromHer ? " from her" : "",
                                             g_follow.stillT),
               false);
}

void OnGameLoad()
{
    g_handle = 0;
    g_handleWho = 0;
    g_quietHandle = 0;
    if (g_followOn.exchange(false)) {
        logger::info("[TEST] FOLLOW off: a game is being loaded ({} walk(s); her walk answers X 12 by itself)", g_follow.k);
    }
    standpoint_walk::WatchForeign(0);
    ++g_followGen;
    g_follow = FollowState{};
}

}  // namespace standpoint_walk_test
