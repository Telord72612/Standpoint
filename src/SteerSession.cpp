// ─────────────────────────────────────────────────────────────────────────────
//  SteerSession.cpp — 1.6.0 THE STEERED SESSION, the DLL half (see SteerSession.h; report "1.6.0 Plan" S1 §2.0-§2.9, §3;
//  spec D §1 items 4 + 5).
//
//  ONE steered session, per driver slot, per frame (main thread, inside WalkApi's frame step):
//    solving   the planning hold (WalkApi HoldHere: she stands held at her feet) + the physics solve (RelativeSolve, a time
//              budget per frame). A POINT session has its spot at once. No spot = O 1.
//    planning  the route's shape from where she stands (S1 §2.2): (1) the spot outside the target's keep-out circle - one plan
//              P -> S with the target as `avoid`; (2) a CLOSE spot inside it - the plan to the approach point E just outside the
//              circle + the radial last leg E -> S (probed by the solve); (3) she starts inside the near zone of an NPC target -
//              NO plan: the ORBIT (a step out on the shorter side, arc chords of 30 deg, the radial leg; every chord probed and
//              kept clear of standing actors; the other side, else a plan from where she stands). Then ONE steered walk
//              (WalkApi RequestSteer) that inherits the hold.
//    walking   the walk's notices: the arrival -> the verdict (S1 §2.3); no route -> ONE more spot on another side (2 hours
//              away or more), then close -> O 17, else the ENGINE FALLBACK (the driver walks ONE engine leg: F); a grab with
//              interrupts 0 (or toward the player) / combat with interrupts 0-1 -> WAITING, then a new route. The target watch:
//              moved 32 u or (an NPC, an hour-relative spot) turned 30 deg -> a re-solve (her hour first) -> a new spot 16 u
//              away or more -> RE-AIM (WalkApi Retarget, else a new walk). The player as target: his position only.
//    arrived   she stands held by the walk's stand; the driver pins her (SteerHandOff) or keeps the stand.
//  The natives only record / read under g_mtx (VM threads). Every log line names STEER #id; the walk's own lines name it too.
//
//  1.6.1 (the first 1.6.0 VR run, report "1.6.0 Plan" R3; review R1 § VERIFY): a POINT off the navmesh with no route gets its ONE
//  engine leg to its nearest navmesh point, O 1 only with no navmesh within 128 u; O 26 (outside the loaded area) keeps the engine
//  leg (C4 + the FIX-UP, FinalNoRoute); a close request's last point lies 5 u past the spot along her last leg, so the walk's 6 u stop
//  lands on it (C5, AimPast); a retask adopts her walk only within the same hold class (N2).
// ─────────────────────────────────────────────────────────────────────────────
#include "PCH.h"
#include "SteerSession.h"
#include "RelativeSolve.h"
#include "MarkerSweep.h"
#include "WalkApi.h"
#include "DirectWalk.h"
#include "PathPlanner.h"
#include "SlotSolver.h"

#include <array>
#include <atomic>
#include <chrono>
#include <format>
#include <memory>
#include <mutex>
#include <string_view>
#include <unordered_map>

namespace logger = SKSE::log;

namespace standpoint_interrupts {
    bool IsPlayerGrabbing(RE::Actor* a_actor);   // Interrupts.cpp: two atomics, safe from any thread
}

namespace standpoint_steer {
namespace {

    constexpr float  kPi             = 3.14159265358979f;
    constexpr float  k2Pi            = 6.28318530717959f;
    constexpr float  kRadToDeg       = 57.2957795f;
    constexpr float  kDegToRad       = 0.0174532925f;
    constexpr float  kCmPerUnit      = 1.428f;   // 70 u = 1 m
    constexpr float  kOnPointU       = 8.f;      // C 0: within 8 u of the spot - the session's own tolerance (S1 §2.3)
    constexpr float  kNearAnyU       = 12.f;     // close + NPC: near on any side within 12 u ...
    constexpr float  kNearSideU      = 35.f;     // ... or within 35 u on the asked side (40 deg, seen from the target): C 9
    constexpr float  kNearSideCos    = 0.766f;
    constexpr float  kStrayU         = 100.f;    // not close: 100 u or more flat (or 100 u up / down) = O 15 (PA's strayed rule)
    constexpr float  kOrbitOut       = 12.f;     // the orbit's radius = the keep-out circle + 12 (S1 §2.2)
    constexpr float  kNearZonePad    = 24.f;     // she starts within the orbit radius + 24 of an NPC target: the orbit
    constexpr float  kOrbitFirst     = 15.f * 0.0174532925f;   // the step out: 15 deg along the shorter side
    constexpr float  kOrbitChord     = 30.f * 0.0174532925f;   // arc chords of at most 30 deg
    constexpr float  kOrbitStraight  = 20.f * 0.0174532925f;   // the spot within 20 deg of her side: straight in / out
    constexpr float  kMovedU         = 32.f;     // the target watch (S1 §2.6): moved this far ...
    constexpr float  kTurnedRad      = 30.f * 0.0174532925f;   // ... or turned this much (an NPC, an hour-relative spot)
    constexpr float  kNewSpotU       = 16.f;     // a re-solved spot this far from the old one: she re-aims
    constexpr float  kNearRuleClose  = 48.f;     // the walk's near rule (W1) for a close request to an NPC ...
    constexpr float  kNearRule       = 32.f;     // ... and for every other session
    constexpr float  kHoldReadyS     = 0.25f;    // the planning hold: the route is asked once the hold took her, or after this
    constexpr float  kClearS         = 0.5f;     // a wait is over once its reason has been gone this long
    constexpr double kBudgetMs       = 0.75;     // the solve / probe budget per frame, all steered sessions together ...
    constexpr double kBudgetMinMs    = 0.2;      // ... at least this each (and always at least one probe)
    constexpr float  kKeepEndedS     = 120.f;    // an ended / handed-off record is forgotten after this (the driver has read it)
    constexpr float  kDepthCap       = standpoint_path::kWadeThighMaxFrac;   // S2-3: the thigh max = the relative spot's water cap
    constexpr float  kNavFlat        = 16.f;     // "on the navmesh" (a point): the solver's snap moved it at most this far ...
    constexpr float  kNavHigh        = 24.f;     // ... and this far up / down
    constexpr float  kAimPastU       = 5.f;      // 1.6.1 (C5): a close request's last point lies this far beyond the spot along her
                                                 // last leg - the walk is told 0 at 6 u from its last point (DirectWalk kStopRadius) - 1
    constexpr float  kAimPastMinLeg  = 8.f;      // ... when that leg is at least this long
    constexpr float  kNavNearU       = 128.f;    // 1.6.1 FIX-UP (C4a-2): a POINT off the navmesh - its nearest navmesh point (the
                                                 // solver's snap) is the engine leg's goal when it lies within this, flat

    // (FIX ROUND, review R1 NIT 14: 7 = she ARRIVED, but her stand ended before the driver's pin - still an arrival)
    enum Phase : int { kSolving = 0, kPlanning = 1, kWalking = 2, kArrived = 3, kEnded = 4, kHandedOff = 5, kWaiting = 6, kStandEnded = 7 };
    enum Code : int { kNone = 0, kC = 1, kP = 2, kO = 3, kD = 4, kL = 5, kX = 6, kB = 7, kF = 8, kH = 9 };
    enum Kind : int { kKindClock = 1, kKindGoTo = 2, kKindPoint = 3 };
    enum RouteKind : int { kRoutePlan = 0, kRouteApproach = 1, kRouteOrbit = 2, kRouteStraight = 3 };
    constexpr std::size_t kSnap = 23;     // 1.6.1 FIX-UP: + [19] the engine leg goes to the nearest navmesh point, [20..22] that point

    const char* PhaseName(int a_p)
    {
        switch (a_p) {
        case kSolving:   return "solving";
        case kPlanning:  return "planning";
        case kWalking:   return "walking";
        case kArrived:   return "arrived";
        case kEnded:     return "ended";
        case kHandedOff: return "handed off";
        case kWaiting:   return "waiting";
        case kStandEnded: return "arrived (her stand ended)";
        }
        return "?";
    }

    char LetterOf(int a_code)
    {
        constexpr char k[] = { '-', 'C', 'P', 'O', 'D', 'L', 'X', 'B', 'F', 'H' };
        return a_code >= 0 && a_code <= 9 ? k[a_code] : '?';
    }

    const char* RouteName(int a_r)
    {
        switch (a_r) {
        case kRoutePlan:     return "a plan to the spot";
        case kRouteApproach: return "a plan to the approach point E + the radial leg in";
        case kRouteOrbit:    return "the ORBIT (no plan)";
        case kRouteStraight: return "straight to the spot (no plan)";
        }
        return "?";
    }

    float Norm2Pi(float a_a)
    {
        while (a_a >= k2Pi) a_a -= k2Pi;
        while (a_a < 0.f) a_a += k2Pi;
        return a_a;
    }

    float NormPi(float a_a)
    {
        while (a_a > kPi) a_a -= k2Pi;
        while (a_a < -kPi) a_a += k2Pi;
        return a_a;
    }

    RE::NiPoint3 Dir(float a_h) { return { std::sin(a_h), std::cos(a_h), 0.f }; }
    float        HeadingOf(float a_dx, float a_dy) { return Norm2Pi(std::atan2(a_dx, a_dy)); }
    float        Dist2D(const RE::NiPoint3& a_a, const RE::NiPoint3& a_b) { return std::hypot(a_a.x - a_b.x, a_a.y - a_b.y); }

    std::int32_t SeedId()
    {
        const auto t = static_cast<std::uint64_t>(std::chrono::system_clock::now().time_since_epoch().count());
        return 1 + static_cast<std::int32_t>((t / 13u) % 0x00FFF000u);
    }

    struct Steer {
        // set at SteerStart (a VM thread), read-only afterwards
        std::int32_t id{ 0 };
        RE::FormID   who{ 0 };
        RE::FormID   target{ 0 };
        int          slot{ -1 };
        int          kind{ kKindPoint };
        float        hold{ -1.f };
        int          pose{ 0 };
        int          interrupts{ 1 };
        bool         close{ false };
        float        distCm{ 0.f }, facingH{ 12.f }, orientH{ 0.f };
        int          relation{ 0 };
        float        standoffU{ 0.f };
        RE::NiPoint3 point{};
        float        pointHeadingDeg{ 0.f };
        bool         targetIsActor{ false };
        bool         targetIsPlayer{ false };
        bool         targetBare{ false };      // a model-less marker: no body, no keep-out circle
        // the natives' side (under g_mtx)
        bool                         stopWanted{ false };
        bool                         handOffWanted{ false };
        bool                         superseded{ false };
        std::int32_t                 supersededBy{ 0 };   // FIX ROUND (R1 #4): the newer session of hers that replaced this one
        std::uint32_t                adoptWalk{ 0 };      // FIX ROUND (R1 #11): a same-kind retask of her WALKING session: re-aim
                                                          // that walk (Retarget) instead of a new one with a planning hold
        std::array<float, kSnap>     snap{};
        int                          phaseShared{ kSolving };
        // the main thread's side
        int                          phase{ kSolving };
        int                          code{ kNone };
        int                          detail{ 0 };
        bool                         started{ false };   // the planning hold has been asked for this attempt
        bool                         solveBegun{ false };
        standpoint_relsolve::Job     solve;
        bool                         spotKnown{ false };
        standpoint_relsolve::Spot    spot;
        std::uint32_t                holdId{ 0 };
        bool                         holdReady{ false };
        float                        holdT{ 0.f };
        std::uint32_t                walkId{ 0 };
        int                          sides{ 0 };          // spots routed to (the best + ONE more on another side)
        std::vector<float>           failedRels;
        bool                         released{ false };   // hold 0 + pose 0: the walk let her go at the point
        int                          waitReason{ 0 };     // 1 a grab, 2 combat
        float                        clearT{ 0.f };
        RE::NiPoint3                 targetAtSolve{};
        float                        targetYawAtSolve{ 0.f };
        bool                         resolving{ false };
        standpoint_relsolve::Job     resolve;
        bool                         reaimPending{ false };   // a re-aim the walk refused: a new walk is asked next frame
        // the route being prepared (planning)
        bool                         geomDecided{ false };
        bool                         routeDecided{ false };
        int                          routeKind{ kRoutePlan };
        RE::NiPoint3                 planTo{};
        std::vector<RE::NiPoint3>    tail;
        std::vector<RE::NiPoint3>    fixedRoute;
        int                          orbitSide{ 0 };       // 0 the shorter side, 1 the other
        std::vector<RE::NiPoint3>    orbitPts;
        std::size_t                  orbitSeg{ 0 };
        RE::NiPoint3                 orbitFrom{};
        std::string                  orbitWhy;
        float                        endedT{ 0.f };
        // FIX ROUND (review R1)
        bool                         sits{ false };       // #1: she sat (chair / bench / bed / getting up) at the start
        std::vector<std::uint32_t>   inherited;           // #4: the walks / planning holds of the sessions this one replaced -
                                                          //     stopped if this one ends before it opened its own walk
        bool                         adopting{ false };   // #11: re-aiming the adopted walk (adoptMain) - no planning hold
        std::uint32_t                adoptMain{ 0 };      // #11: adoptWalk, taken over by the main thread (Frame)
        bool                         frameYawSet{ false };// #7: the player as target - the first solve's frame yaw, kept
        float                        frameYaw{ 0.f };
        bool                         firstSpotKnown{ false };   // #5: the spot of the first side (no route there) ...
        standpoint_relsolve::Spot    firstSpot;
        int                          firstDet{ 0 };       //     ... and the walk's no-route detail there (1 / 18)
        // 1.6.1 FIX-UP (C4a-2, review R1 § VERIFY 1.6.1): a POINT off the navmesh - its nearest navmesh point (the solver's snap,
        // within kNavNearU flat); with no route the ONE engine leg goes there (legToNav), the driver measures the arrival to the point
        bool                         navNearKnown{ false };
        RE::NiPoint3                 navNear{};
        bool                         legToNav{ false };
    };
    using SteerPtr = std::shared_ptr<Steer>;

    std::mutex                                g_mtx;
    std::unordered_map<std::int32_t, SteerPtr> g_steers;
    std::int32_t                              g_nextId = SeedId();
    std::atomic<int>                          g_steered{ -1 };    // [Session] Steered (Init)
    std::string                               g_steerNote;

    SteerPtr Find(std::int32_t a_id)
    {
        std::scoped_lock lk(g_mtx);
        const auto it = g_steers.find(a_id);
        return it == g_steers.end() ? nullptr : it->second;
    }

    // FIX ROUND (R1 #11): an ADOPTED walk keeps the steer id it was opened with (WalkApi keeps who / steerId / notice / user on a
    // Retarget) - its notices reach the session that adopted it through its walk id. Main thread (walkId is written there).
    SteerPtr FindByWalk(std::uint32_t a_walkId, RE::FormID a_who)
    {
        if (!a_walkId) return nullptr;
        std::scoped_lock lk(g_mtx);
        for (const auto& [id, s] : g_steers) {
            if (s->walkId == a_walkId && s->who == a_who && !s->superseded) return s;
        }
        return nullptr;
    }

    RE::Actor*         ActorOf(RE::FormID a_id) { return a_id ? RE::TESForm::LookupByID<RE::Actor>(a_id) : nullptr; }
    RE::TESObjectREFR* RefOf(RE::FormID a_id) { return a_id ? RE::TESForm::LookupByID<RE::TESObjectREFR>(a_id) : nullptr; }

    std::string NameOf(RE::TESObjectREFR* a_ref, RE::FormID a_id)
    {
        if (!a_ref) return std::format("[{:08X}]", a_id);
        const char* n = a_ref->GetDisplayFullName();
        return std::format("{} [{:08X}]", n && n[0] ? n : "(no name)", a_id);
    }

    bool CloseNpc(const Steer& s) { return s.close && s.targetIsActor && !s.targetIsPlayer; }

    // her final heading now (rad): an actor target is faced LIVE (+ the offset); else the solved / asked heading
    float FinalHeading(const Steer& s, const RE::NiPoint3& a_her, float a_yaw)
    {
        if (s.kind == kKindPoint) return Norm2Pi(s.pointHeadingDeg * kDegToRad);
        if (s.spotKnown && s.spot.faceLive) {
            if (auto* t = RefOf(s.target)) {
                const RE::NiPoint3 tp = t->GetPosition();
                if (Dist2D(tp, a_her) > 1.f) return Norm2Pi(HeadingOf(tp.x - a_her.x, tp.y - a_her.y) + s.spot.faceOffsetRad);
            }
        }
        if (s.spotKnown) return s.spot.headingRad;
        return a_yaw;
    }

    // the state line the driver polls (SteerState): written on the main thread, read under g_mtx
    void Snapshot(Steer& s)
    {
        std::array<float, kSnap> v{};
        auto*                    a = ActorOf(s.who);
        const RE::NiPoint3       her = a ? a->GetPosition() : RE::NiPoint3{};
        const float              yaw = a ? a->GetAngleZ() : 0.f;
        const RE::NiPoint3       sp = s.spotKnown ? s.spot.pos : (s.kind == kKindPoint ? s.point : RE::NiPoint3{});
        const float              fh = FinalHeading(s, her, yaw);
        v[0] = static_cast<float>(s.phase);
        v[1] = static_cast<float>(s.code);
        v[2] = static_cast<float>(s.detail);
        v[3] = s.spotKnown ? Dist2D(her, sp) : 0.f;
        v[4] = NormPi(yaw - fh) * kRadToDeg;
        v[5] = her.x;
        v[6] = her.y;
        v[7] = her.z;
        v[8] = sp.x;
        v[9] = sp.y;
        v[10] = sp.z;
        float deg = Norm2Pi(fh) * kRadToDeg;
        if (deg >= 359.95f) deg = 0.f;
        v[11] = deg;
        v[12] = s.spotKnown ? s.spot.hours : 0.f;
        v[13] = s.spotKnown && s.spot.onNavmesh ? 1.f : 0.f;
        v[14] = static_cast<float>(s.walkId);
        v[15] = static_cast<float>(s.waitReason);
        v[16] = s.released ? 1.f : 0.f;
        v[17] = s.spotKnown ? s.spot.faceOffsetRad * kRadToDeg : 0.f;
        v[18] = s.spotKnown ? 1.f : 0.f;
        v[19] = s.legToNav ? 1.f : 0.f;      // 1.6.1 FIX-UP (C4a-2): the engine leg goes to the nearest navmesh point ...
        v[20] = s.legToNav ? s.navNear.x : 0.f;
        v[21] = s.legToNav ? s.navNear.y : 0.f;
        v[22] = s.legToNav ? s.navNear.z : 0.f;
        std::scoped_lock lk(g_mtx);
        s.snap = v;
        s.phaseShared = s.phase;
    }

    void StopWalks(Steer& s, const char* a_why)
    {
        bool w = false, h = false, inh = false;
        if (s.walkId) w = standpoint_walk::StopHandle(static_cast<std::int32_t>(s.walkId), StandpointAPI::kDetailReleased);
        if (s.holdId && s.holdId != s.walkId) h = standpoint_walk::StopHandle(static_cast<std::int32_t>(s.holdId), StandpointAPI::kDetailReleased);
        // FIX ROUND (R1 #4): the walks / holds of the sessions this one replaced, if this one never opened its own (that would have
        // replaced them): stopped too - an owner-less walk must never stand her at the old spot
        for (const auto id : s.inherited) {
            if (id && id != s.walkId && id != s.holdId) inh = standpoint_walk::StopHandle(static_cast<std::int32_t>(id), StandpointAPI::kDetailReleased) || inh;
        }
        s.inherited.clear();
        if (w || h || inh) {
            logger::info("STEER #{} her {}{} ended ({}): she is handed back", s.id,
                         w && h ? "walk and planning hold" : w ? "walk / stand" : h ? "planning hold" : "replaced session's walk",
                         inh && (w || h) ? " (and the replaced session's walk)" : "", a_why);
        }
    }

    // FIX ROUND (R1 #4): this session opened a walk of its own - it replaced the inherited ones (WalkApi's hand-over keeps her held)
    void OwnWalkOpened(Steer& s)
    {
        s.inherited.clear();
    }

    void End(Steer& s, int a_code, int a_detail, const std::string& a_why)
    {
        s.phase = kEnded;
        s.code = a_code;
        s.detail = a_detail;
        s.resolving = false;
        logger::info("STEER #{} END {} {} - {}", s.id, LetterOf(a_code), a_detail, a_why);
        StopWalks(s, "the session's answer is given");
    }

    // ── the solve ────────────────────────────────────────────────────────────────────────────────────────────────
    standpoint_relsolve::Ask MakeAsk(const Steer& s, bool a_resolve)
    {
        standpoint_relsolve::Ask a;
        a.kind = s.kind == kKindClock ? standpoint_relsolve::Kind::kClock : standpoint_relsolve::Kind::kGoTo;
        a.who = s.who;
        a.target = s.target;
        a.steerId = s.id;
        a.distU = s.distCm / kCmPerUnit;
        a.facingH = s.facingH;
        a.orientH = s.orientH;
        a.close = s.close;
        a.relation = s.relation;
        a.standoffU = s.standoffU;
        a.depthFrac = kDepthCap;
        a.avoidRels = s.failedRels;
        if (a_resolve && s.spotKnown) {
            a.preferSet = true;
            a.preferRel = s.spot.rel;
        }
        a.frameYawSet = s.frameYawSet;      // FIX ROUND (R1 #7): the player - the first solve's frame, his position only
        a.frameYaw = s.frameYaw;
        return a;
    }

    void                      AskHold(Steer& s);
    standpoint_walk::SteerAsk MakeSteerAsk(const Steer& s);

    // The no-route answer once every try is used (S1 §2.8; the user's rule, spec D §0: no route -> the game's walk for that one trip):
    // close contact -> O 17 at once; a spot on the navmesh (a POINT or a relative spot), or OUTSIDE THE LOADED AREA (O 26 - the
    // contract: "GoToPoint can travel there") -> F = the driver walks ONE engine leg there; a relative spot off the navmesh -> O 17.
    // 1.6.1 (C4; run R3 §3 / §7) + FIX-UP (C4a-2, review R1 § VERIFY 1.6.1): a POINT off the navmesh -> the ONE engine leg goes to its
    // NEAREST NAVMESH POINT (the snap, within 128 u flat - BiSR's shore point on a rock / in the water: she ends at the shore edge,
    // as 1.6.0's leg usually took her; the driver answers a SHORT arrival measured to the point); no navmesh point -> O 1 at once.
    // (The driver ends an engine leg that is blocked or makes no progress - R3's point under the table answers in seconds.)
    void FinalNoRoute(Steer& s, int a_det, const std::string& a_why)
    {
        if (s.close) {
            End(s, kO, StandpointAPI::kDetailNoWay, a_why + " - close contact: no engine leg, O 17 at once");
            return;
        }
        if (a_det == StandpointAPI::kDetailOutsideLoaded) {
            End(s, kF, a_det, a_why + " - outside the loaded area: the ENGINE FALLBACK - the game's walk for that trip (ONE engine leg)");
            return;
        }
        if (s.spot.onNavmesh) {
            End(s, kF, a_det, a_why + " - the ENGINE FALLBACK: the driver walks ONE engine leg to the spot");
            return;
        }
        if (s.kind == kKindPoint) {
            if (s.navNearKnown) {
                s.legToNav = true;
                End(s, kF, a_det,
                    std::format("{} - the point is off the navmesh: the ENGINE FALLBACK walks ONE engine leg to its nearest navmesh point "
                                "({:.0f},{:.0f},{:.0f}), {:.0f} u from it (a SHORT arrival, measured to the point)",
                                a_why, s.navNear.x, s.navNear.y, s.navNear.z, Dist2D(s.navNear, s.point)));
                return;
            }
            End(s, kO, StandpointAPI::kDetailNoSlot,
                std::format("{} - and the point is off the navmesh with no navmesh point within {:.0f} u of it: no engine leg, O 1 at once", a_why,
                            kNavNearU));
            return;
        }
        End(s, kO, StandpointAPI::kDetailNoWay, a_why + " - and the spot is off the navmesh (the engine cannot walk there either)");
    }

    // FIX ROUND (R1 #5): a solve that finds nothing AFTER the first side had a spot (but no route) answers as no route - never O 1
    void SolveFailed(Steer& s, const std::string& a_why)
    {
        if (!s.failedRels.empty() && s.firstSpotKnown) {
            s.spot = s.firstSpot;
            s.spotKnown = true;
            FinalNoRoute(s, s.firstDet ? s.firstDet : StandpointAPI::kDetailNoSlot,
                         std::format("no route to the first side, and no other side has a spot ({})", a_why));
            return;
        }
        End(s, kO, StandpointAPI::kDetailNoSlot, "no standable spot: " + a_why);
    }

    // FIX ROUND (R1 #1): she SITS (a chair, a bench, a bed, getting up): the steered walk cannot take her (the driver's gate refuses
    // furniture) - the game's own walk stands her up and walks her: ONE engine leg to the spot, for every kind (not "no route":
    // the close-contact O 17 rule does not apply). The driver's ENGINE FALLBACK (F, detail 14).
    void SitsEngineLeg(Steer& s, const char* a_when)
    {
        logger::info("STEER #{} she sits ({}) - the game's walk stands her up (ONE engine leg)", s.id, a_when);
        End(s, kF, StandpointAPI::kDetailEnded, "she sits - the game's walk stands her up (ONE engine leg)");
    }

    // FIX ROUND (R1 #11): a same-kind retask of her WALKING session - its walk is re-aimed at the new spot (WalkApi Retarget: no
    // planning hold, no stop). false = it could not be (it answered, a stop is marked on it, a refused plan): the ordinary way then.
    bool TryAdopt(Steer& s)
    {
        const std::uint32_t w = s.adoptMain;
        s.adoptMain = 0;
        s.adopting = false;
        if (!w) return false;
        s.tail.clear();
        s.fixedRoute.clear();
        s.routeKind = (s.kind != kKindPoint && s.spot.inCircle) ? kRouteApproach : kRoutePlan;
        s.planTo = s.routeKind == kRouteApproach ? s.spot.approach : s.spot.pos;
        if (s.routeKind == kRouteApproach) s.tail = { s.spot.pos };
        if (!standpoint_walk::Retarget(w, MakeSteerAsk(s))) {
            logger::info("STEER #{} RETASK: her walk #{} could not be re-aimed (it answered, or a stop is marked on it) - a new walk", s.id, w);
            return false;
        }
        s.walkId = w;
        ++s.sides;
        OwnWalkOpened(s);
        s.phase = kWalking;
        logger::info("STEER #{} RETASK: her walk #{} re-aimed at the new spot ({:.0f},{:.0f},{:.0f}) - no stop, no planning hold", s.id, w,
                     s.spot.pos.x, s.spot.pos.y, s.spot.pos.z);
        return true;
    }

    // the spot is known: the engine leg for a sitting walker, the adopted walk re-aimed, or the route planned (after the hold)
    void SpotReady(Steer& s)
    {
        if (s.sits) {
            SitsEngineLeg(s, "at the start");
            return;
        }
        if (s.adopting) {
            if (TryAdopt(s)) return;
            AskHold(s);                      // (the ordinary way: she stands held while the new route is planned)
        }
        s.phase = kPlanning;
    }

    void StepSolve(Steer& s, RE::Actor* a_her, double a_budget)
    {
        if (!s.started) {
            s.started = true;
            std::string gateText;
            if (standpoint_direct::Gate(a_her, gateText) == standpoint_direct::End::kFurniture) {
                s.sits = true;               // (no planning hold: the gate would refuse it too)
                logger::info("STEER #{} she sits at the start ({}) - her spot is worked out, then the game's walk stands her up", s.id, gateText);
            } else if (s.adoptMain) {
                s.adopting = true;           // (no planning hold: her walk keeps her walking while the new spot is worked out)
            } else {
                AskHold(s);
            }
        }
        if (s.kind == kKindPoint) {
            s.spot = standpoint_relsolve::Spot{};
            s.spot.found = true;
            s.spot.pos = s.point;
            s.spot.headingRad = Norm2Pi(s.pointHeadingDeg * kDegToRad);
            RE::NiPoint3 sn = s.point;
            const bool   snapped = standpoint::SnapToNavmeshNear(a_her, sn);
            s.spot.onNavmesh = snapped && Dist2D(sn, s.point) <= kNavFlat && std::fabs(sn.z - s.point.z) <= kNavHigh;
            // 1.6.1 FIX-UP (C4a-2): off the navmesh - its nearest navmesh point (the snap's window: 96 u above, 160 u below), kept as
            // the engine leg's goal should the point have no route (FinalNoRoute)
            s.navNearKnown = snapped && !s.spot.onNavmesh && Dist2D(sn, s.point) <= kNavNearU;
            if (s.navNearKnown) s.navNear = sn;
            s.spotKnown = true;
            SpotReady(s);
            return;
        }
        if (!s.solveBegun) {
            s.solveBegun = true;
            if (s.targetIsPlayer && !s.frameYawSet) {
                // FIX ROUND (R1 #7): the player's frame is his yaw NOW, kept for every re-solve (his headset turns with every glance)
                if (auto* t = RefOf(s.target)) {
                    s.frameYawSet = true;
                    s.frameYaw = t->GetAngle().z;
                }
            }
            std::string why;
            if (!s.solve.Begin(MakeAsk(s, false), why)) {
                SolveFailed(s, why);
                return;
            }
        }
        if (!s.solve.Step(a_budget)) return;
        const auto& r = s.solve.Result();
        if (!r.found) {
            SolveFailed(s, r.why);
            return;
        }
        s.spot = r;
        s.spotKnown = true;
        if (auto* t = RefOf(s.target)) {
            s.targetAtSolve = t->GetPosition();
            s.targetYawAtSolve = t->GetAngle().z;
        }
        SpotReady(s);
    }

    // ── the planning hold (S1 §2.2: she stands still while her spot / route is worked out) ──────────────────────
    void OnNotice(const standpoint_walk::SteerNotice& a_n, void* a_user);

    void AskHold(Steer& s)
    {
        s.started = true;
        s.holdReady = false;
        s.holdT = 0.f;
        s.holdId = standpoint_walk::HoldHere(s.who, -1.f, s.id, &OnNotice, nullptr);
        if (s.holdId) OwnWalkOpened(s);    // (FIX ROUND R1 #4: the hold replaces the replaced session's walk - WalkApi's hand-over)
        logger::info("STEER #{} HOLD: she stands held while her spot and route are worked out (walk #{})", s.id, s.holdId);
    }

    // ── the route's shape (S1 §2.2) ─────────────────────────────────────────────────────────────────────────────
    std::vector<RE::NiPoint3> OrbitPoints(const RE::NiPoint3& a_p, const RE::NiPoint3& a_t, const RE::NiPoint3& a_s, float a_ro, int a_dir)
    {
        const float thP = HeadingOf(a_p.x - a_t.x, a_p.y - a_t.y);
        const float thS = HeadingOf(a_s.x - a_t.x, a_s.y - a_t.y);
        const float dl = NormPi(thS - thP);
        const float span = (a_dir > 0) == (dl > 0.f) ? std::fabs(dl) : k2Pi - std::fabs(dl);
        const float sg = static_cast<float>(a_dir);
        std::vector<RE::NiPoint3> pts;
        const auto at = [&](float a_ang, float a_z) {
            const RE::NiPoint3 d = Dir(a_ang);
            return RE::NiPoint3{ a_t.x + d.x * a_ro, a_t.y + d.y * a_ro, a_z };
        };
        float done = kOrbitFirst;
        pts.push_back(at(thP + sg * done, a_p.z));                 // the step out (and round at once)
        while (span - done > kOrbitChord) {
            done += kOrbitChord;
            pts.push_back(at(thP + sg * done, a_p.z));
        }
        if (span - done < 0.05f && pts.size() > 1) pts.pop_back(); // (the last chord would be a sliver)
        pts.push_back(at(thS, a_s.z));                             // the arc ends on the spot's bearing ...
        pts.push_back(a_s);                                        // ... and the radial leg in (or out)
        return pts;
    }

    // Decide the route's shape; the orbit's probes are sliced by the budget. true = decided.
    bool BuildRoute(Steer& s, RE::Actor* a_her, double a_budget)
    {
        if (s.routeDecided) return true;
        const RE::NiPoint3 p = a_her->GetPosition();
        const RE::NiPoint3 sp = s.spot.pos;
        if (!s.geomDecided) {
            s.geomDecided = true;
            s.tail.clear();
            s.fixedRoute.clear();
            s.planTo = sp;
            s.routeKind = kRoutePlan;
            if (s.kind == kKindPoint) {
                s.routeDecided = true;
                return true;
            }
            auto*       t = RefOf(s.target);
            const float keepR = s.targetBare ? 0.f : standpoint_relsolve::KeepOutRadius(t, a_her);
            const float ro = keepR + kOrbitOut;
            const RE::NiPoint3 tp = t ? t->GetPosition() : sp;
            if (t && s.targetIsActor && !s.targetIsPlayer && Dist2D(p, tp) < ro + kNearZonePad) {
                // (3) she starts inside the near zone of an NPC target: no planner
                const float thP = HeadingOf(p.x - tp.x, p.y - tp.y);
                const float thS = HeadingOf(sp.x - tp.x, sp.y - tp.y);
                if (std::fabs(NormPi(thS - thP)) <= kOrbitStraight) {
                    s.routeKind = kRouteStraight;
                    s.orbitPts = { sp };
                } else {
                    s.routeKind = kRouteOrbit;
                    const int shortSide = NormPi(thS - thP) > 0.f ? 1 : -1;
                    s.orbitSide = 0;
                    s.orbitPts = OrbitPoints(p, tp, sp, ro, shortSide);
                }
                s.orbitSeg = 0;
                s.orbitFrom = p;
                s.orbitWhy.clear();
                logger::info("STEER #{} NEAR ROUTE: she starts {:.0f} u from the target (the near zone is {:.0f} u) - {}, {} point(s)", s.id,
                             Dist2D(p, tp), ro + kNearZonePad,
                             s.routeKind == kRouteStraight ? "straight in / out on her side" : "round the target on the shorter side",
                             s.orbitPts.size());
            } else if (s.spot.inCircle) {
                // (2) a close spot inside the keep-out circle: the plan to E (the circle keeps its full radius), then E -> S
                s.routeKind = kRouteApproach;
                s.planTo = s.spot.approach;
                s.tail = { sp };
                s.routeDecided = true;
                return true;
            } else {
                // (1) the spot outside the circle: one plan P -> S, the target as `avoid`
                s.routeDecided = true;
                return true;
            }
        }
        // the orbit / the straight leg: every chord probed, every point kept clear of standing actors
        auto*      walker = a_her;
        const auto t0 = std::chrono::steady_clock::now();
        do {
            if (s.orbitSeg >= s.orbitPts.size()) {
                s.fixedRoute = s.orbitPts;
                s.routeDecided = true;
                logger::info("STEER #{} {}: every leg clear ({} point(s){})", s.id, s.routeKind == kRouteStraight ? "STRAIGHT" : "ORBIT",
                             s.fixedRoute.size(), s.routeKind == kRouteOrbit ? (s.orbitSide == 0 ? ", the shorter side" : ", the other side") : "");
                return true;
            }
            const RE::NiPoint3 a = s.orbitSeg == 0 ? s.orbitFrom : s.orbitPts[s.orbitSeg - 1];
            const RE::NiPoint3 b = s.orbitPts[s.orbitSeg];
            RE::FormID         blocker = 0;
            float              br = 0.f;
            std::string        why;
            if (s.orbitSeg + 1 < s.orbitPts.size() && standpoint_relsolve::StandingActorAt(walker, s.target, b, blocker, br)) {
                why = std::format("point {} lies in a standing actor's place ([{:08X}], {:.0f} u)", s.orbitSeg + 1, blocker, br);
            } else {
                standpoint_path::ProbeNote note;
                if (!standpoint_path::ProbeSegment(walker, a, b, kDepthCap, true, &note)) {
                    why = std::format("leg {} blocked at {:.0f} u - {}", s.orbitSeg + 1, note.failAt, note.why);
                }
            }
            if (why.empty()) {
                ++s.orbitSeg;
                continue;
            }
            logger::info("STEER #{} {} ({}): {}", s.id, s.routeKind == kRouteStraight ? "STRAIGHT" : "ORBIT",
                         s.routeKind == kRouteStraight ? "her side" : (s.orbitSide == 0 ? "the shorter side" : "the other side"), why);
            if (s.routeKind == kRouteOrbit && s.orbitSide == 0) {
                // the other way round
                auto*              t = RefOf(s.target);
                const RE::NiPoint3 tp = t ? t->GetPosition() : sp;
                const float        keepR = standpoint_relsolve::KeepOutRadius(t, walker);
                const float        thP = HeadingOf(s.orbitFrom.x - tp.x, s.orbitFrom.y - tp.y);
                const float        thS = HeadingOf(sp.x - tp.x, sp.y - tp.y);
                const int          otherSide = NormPi(thS - thP) > 0.f ? -1 : 1;
                s.orbitSide = 1;
                s.orbitPts = OrbitPoints(s.orbitFrom, tp, sp, keepR + kOrbitOut, otherSide);
                s.orbitSeg = 0;
                continue;
            }
            // both ways blocked (or the straight leg): a plan from where she stands - the circle shrinks to her distance
            s.routeKind = s.spot.inCircle ? kRouteApproach : kRoutePlan;
            s.planTo = s.spot.inCircle ? s.spot.approach : sp;
            s.tail.clear();
            if (s.spot.inCircle) s.tail = { sp };
            s.fixedRoute.clear();
            s.routeDecided = true;
            logger::info("STEER #{} NEAR ROUTE: no clear way round - {} from where she stands (the keep-out circle shrinks to her distance)", s.id,
                         RouteName(s.routeKind));
            return true;
        } while (std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() < a_budget);
        return false;
    }

    // 1.6.1 (C5; run R3 §6: the close arrivals of the stpt program stopped 4-5 u SHORT on their way in - 45 cm -> 49-52 cm, 30 cm ->
    // 36-37 cm): the walk is told 0 at 6 u from its LAST point, so a CLOSE request's last point is moved kAimPastU beyond the spot
    // along her last leg (E -> S; the orbit's radial leg; the straight step in OR out from body contact) - she stops ON the spot. The
    // spot itself (the walk's distance, C 0, the near rule - and the session's verdict) stays S. A plan straight to the spot (its last
    // leg is the planner's, unknown here) keeps its last point.
    void AimPast(const Steer& s, standpoint_walk::SteerAsk& a_ask)
    {
        const RE::NiPoint3& sp = s.spot.pos;
        RE::NiPoint3*       last = nullptr;
        RE::NiPoint3        from{};
        if (!a_ask.fixedRoute.empty()) {
            last = &a_ask.fixedRoute.back();
            from = a_ask.fixedRoute.size() >= 2 ? a_ask.fixedRoute[a_ask.fixedRoute.size() - 2] : s.orbitFrom;
        } else if (!a_ask.tail.empty()) {
            last = &a_ask.tail.back();
            from = a_ask.tail.size() >= 2 ? a_ask.tail[a_ask.tail.size() - 2] : a_ask.planTo;
        }
        if (!last || Dist2D(*last, sp) > 1.f) return;
        const float dx = sp.x - from.x, dy = sp.y - from.y;
        const float len = std::hypot(dx, dy);
        if (len < kAimPastMinLeg) return;
        last->x = sp.x + dx / len * kAimPastU;
        last->y = sp.y + dy / len * kAimPastU;
    }

    standpoint_walk::SteerAsk MakeSteerAsk(const Steer& s)
    {
        using namespace standpoint_path;
        standpoint_walk::SteerAsk ask;
        ask.who = s.who;
        ask.steerId = s.id;
        ask.spot = s.spot.pos;
        ask.planTo = s.planTo;
        ask.tail = s.tail;
        ask.fixedRoute = s.fixedRoute;
        ask.avoid = (s.kind != kKindPoint && !s.targetBare) ? s.target : 0;
        // RELATIVE: the MIX without kPartial / kGoalMayMove (the solve proved the spot: a partial or marched goal is not the
        // asked place); POINT legs keep kGoalMayMove (a point that is no place to stand moves toward her; the strayed rule
        // decides). Neither has kPartial: no way = the user's rule (one engine leg), never a half route.
        ask.planFlags = kFloor | kWater | kAllDirect | kNavGuide | kDoors | kActorsKeepOut;
        if (s.kind == kKindPoint) ask.planFlags |= kGoalMayMove;
        ask.depthFrac = kDepthCap;
        if (s.kind != kKindPoint && s.spot.faceLive && s.target) {
            ask.faceWho = s.target;
            ask.faceOffsetDeg = s.spot.faceOffsetRad * kRadToDeg;
            ask.facingDeg = s.spot.headingRad * kRadToDeg;     // (the fallback when the target is gone)
        } else if (s.kind == kKindPoint) {
            ask.facingDeg = Norm2Pi(s.pointHeadingDeg * kDegToRad) * kRadToDeg;
        } else {
            ask.facingDeg = s.spot.headingRad * kRadToDeg;
        }
        // the orbit side-steps round the target facing it; FIX ROUND (R1 #10): so does the straight step out / in from body contact
        // (50-84 u - longer than the end's 48 u blend, it started with a 180-degree turn)
        ask.faceAll = s.routeKind == kRouteOrbit || s.routeKind == kRouteStraight;
        ask.hold = (s.hold == 0.f && s.pose == 0) ? 0.f : -1.f;
        ask.nearRadius = CloseNpc(s) ? kNearRuleClose : kNearRule;
        ask.notice = &OnNotice;
        ask.user = nullptr;
        if (s.close) AimPast(s, ask);     // 1.6.1 (C5)
        return ask;
    }

    void StepPlan(Steer& s, RE::Actor* a_her, float a_dt, double a_budget)
    {
        s.holdT += a_dt;
        if (!s.holdReady && s.holdT < kHoldReadyS) return;     // the hold takes her first
        if (!BuildRoute(s, a_her, a_budget)) return;
        const auto ask = MakeSteerAsk(s);
        const std::uint32_t h = standpoint_walk::RequestSteer(ask);
        if (!h) {
            End(s, kX, StandpointAPI::kDetailEnded, "the walk refused the steered route (no walker)");
            return;
        }
        s.walkId = h;
        ++s.sides;
        OwnWalkOpened(s);
        s.phase = kWalking;
        s.geomDecided = false;
        s.routeDecided = false;
        const RE::NiPoint3 p = a_her->GetPosition();
        const bool         aimed = (!ask.fixedRoute.empty() && Dist2D(ask.fixedRoute.back(), s.spot.pos) > 1.f) ||
                                   (!ask.tail.empty() && Dist2D(ask.tail.back(), s.spot.pos) > 1.f);
        logger::info("STEER #{} PLAN -> walk #{}: {} from ({:.0f},{:.0f},{:.0f}) to the spot ({:.0f},{:.0f},{:.0f}){}{}{}, facing {}, hold {}, near rule "
                     "{:.0f} u{}",
                     s.id, h, RouteName(s.routeKind), p.x, p.y, p.z, s.spot.pos.x, s.spot.pos.y, s.spot.pos.z,
                     aimed ? std::format(" (close: her last point {:.0f} u past it on her last leg, so the stop lands on it)", kAimPastU) : std::string(),
                     s.routeKind == kRouteApproach ? std::format(" via E ({:.0f},{:.0f},{:.0f})", s.planTo.x, s.planTo.y, s.planTo.z) : std::string(),
                     ask.avoid ? std::format(", keeping clear of [{:08X}]", ask.avoid) : std::string(),
                     ask.faceWho ? std::format("[{:08X}] live + {:.0f} deg{}", ask.faceWho, ask.faceOffsetDeg, ask.faceAll ? " (the whole way)" : "")
                                 : std::format("{:.0f} deg", ask.facingDeg),
                     ask.hold < 0.f ? std::string("-1 (the driver pins her)") : std::string("0 (let go at the point)"), ask.nearRadius,
                     s.failedRels.empty() ? std::string() : std::format(" - another side (the first one had no route; route #{})", s.sides));
    }

    // ── a moving target (S1 §2.6) ───────────────────────────────────────────────────────────────────────────────
    bool HourRelative(const Steer& s)
    {
        if (!s.targetIsActor || s.targetIsPlayer) return false;      // the player: his position only (a glance would re-solve)
        if (s.kind == kKindClock) return s.facingH >= 0.f;
        if (s.kind == kKindGoTo) return s.relation >= 1 && s.relation <= 4;
        return false;
    }

    void StepWatch(Steer& s, double a_budget)
    {
        if (s.reaimPending) {
            // the re-aim the walk refused last frame: the new walk now
            s.reaimPending = false;
            const std::uint32_t h = standpoint_walk::RequestSteer(MakeSteerAsk(s));
            if (!h) {
                End(s, kX, StandpointAPI::kDetailEnded, "the walk refused the re-aimed route");
                return;
            }
            s.walkId = h;
            logger::info("STEER #{} RE-AIM: walk #{} to the new spot (hour {:.1f})", s.id, h, s.spot.hours);
            return;
        }
        if (s.kind == kKindPoint) return;
        auto* t = RefOf(s.target);
        if (!t) return;
        if (s.resolving) {
            if (!s.resolve.Step(a_budget)) return;
            s.resolving = false;
            const auto& r = s.resolve.Result();
            if (!r.found) {
                logger::info("STEER #{} RE-SOLVE: no standable spot now ({}) - she keeps the old one", s.id, r.why);
                return;
            }
            const float moved = Dist2D(r.pos, s.spot.pos);
            if (moved < kNewSpotU) {
                logger::info("STEER #{} RE-SOLVE: the spot moved only {:.0f} u - kept", s.id, moved);
                return;
            }
            s.spot = r;
            // the route to the new spot: a plan (the orbit is for a start next to the target, not for a walker on her way)
            s.routeKind = s.spot.inCircle ? kRouteApproach : kRoutePlan;
            s.planTo = s.spot.inCircle ? s.spot.approach : s.spot.pos;
            s.tail.clear();
            if (s.spot.inCircle) s.tail = { s.spot.pos };
            s.fixedRoute.clear();
            const auto ask = MakeSteerAsk(s);
            if (standpoint_walk::Retarget(s.walkId, ask)) {
                logger::info("STEER #{} RE-AIM: the new spot (hour {:.1f}, {:.0f} u from the old one) - walk #{} re-planned on the move", s.id,
                             s.spot.hours, moved, s.walkId);
            } else {
                // (B's note: a refused re-aim answers B 11 "busy" through the notice AND Retarget returns false; or the walk has
                // just answered / a stop is marked on it.) The old walk is let go of here - its answer is not this session's
                // (walkId 0 ignores it) - and a NEW walk is asked on the next frame (never two walks of hers in one frame).
                logger::info("STEER #{} RE-AIM: the new spot (hour {:.1f}, {:.0f} u from the old one) - walk #{} could not be re-aimed: a new walk "
                             "next frame", s.id, s.spot.hours, moved, s.walkId);
                s.walkId = 0;
                s.reaimPending = true;
            }
            return;
        }
        const RE::NiPoint3 tp = t->GetPosition();
        const float        moved = (tp - s.targetAtSolve).Length();
        const float        turned = HourRelative(s) ? std::fabs(NormPi(t->GetAngle().z - s.targetYawAtSolve)) : 0.f;
        if (moved < kMovedU && turned < kTurnedRad) return;
        logger::info("STEER #{} TARGET MOVED {:.0f} u{} - a re-solve, her hour {:.1f} first", s.id, moved,
                     turned >= kTurnedRad ? std::format(", turned {:.0f} deg", turned * kRadToDeg) : std::string(), s.spot.hours);
        s.targetAtSolve = tp;
        s.targetYawAtSolve = t->GetAngle().z;
        std::string why;
        if (!s.resolve.Begin(MakeAsk(s, true), why)) {
            logger::info("STEER #{} RE-SOLVE refused: {} - she keeps the old spot", s.id, why);
            return;
        }
        s.resolving = true;
    }

    // ── waiting for a grab / combat / a reaction (FIX ROUND R1 #8: ragdoll, knocked, kill move) to end (S1 §2.7, D5) ────
    const char* WaitName(int a_reason)
    {
        return a_reason == 1 ? "hand-grab" : a_reason == 2 ? "combat" : "reaction (ragdoll / knocked / kill move)";
    }

    void StepWait(Steer& s, RE::Actor* a_her, float a_dt)
    {
        bool clear = true;
        if (s.waitReason == 1 && standpoint_interrupts::IsPlayerGrabbing(a_her)) clear = false;
        if (s.waitReason == 2 && a_her->IsInCombat()) clear = false;
        std::string gateText;
        const auto  gate = standpoint_direct::Gate(a_her, gateText);
        if (gate == standpoint_direct::End::kFurniture && s.spotKnown) {
            SitsEngineLeg(s, "she sat down while she waited");    // (FIX ROUND R1 #1: the steered walk cannot take her now)
            return;
        }
        if (clear && gate != standpoint_direct::End::kNone) clear = false;   // (the gate covers ragdoll / knocked / kill move)
        if (!clear) {
            s.clearT = 0.f;
            return;
        }
        s.clearT += a_dt;
        if (s.clearT < kClearS) return;
        logger::info("STEER #{} RESUME: the {} is over - a new route from where she is", s.id, WaitName(s.waitReason));
        s.waitReason = 0;
        s.walkId = 0;
        s.started = false;
        s.geomDecided = false;
        s.routeDecided = false;
        bool moved = false;
        if (s.kind != kKindPoint) {
            if (auto* t = RefOf(s.target)) moved = (t->GetPosition() - s.targetAtSolve).Length() >= kMovedU;
        }
        if (moved) {
            s.solveBegun = false;
            s.phase = kSolving;           // the target moved meanwhile: solved again (the hold is asked there)
        } else {
            AskHold(s);
            s.phase = kPlanning;
        }
    }

    // ── the walk's notices (main thread) ────────────────────────────────────────────────────────────────────────
    bool NearRule(const Steer& s, const RE::NiPoint3& a_her)
    {
        const RE::NiPoint3& sp = s.spot.pos;
        const float         gap = Dist2D(a_her, sp);
        if (gap > kNearSideU) return false;
        if (gap <= kNearAnyU) return true;
        auto* t = RefOf(s.target);
        if (!t) return true;
        const RE::NiPoint3 tp = t->GetPosition();
        const float wx = a_her.x - tp.x, wy = a_her.y - tp.y, fx = sp.x - tp.x, fy = sp.y - tp.y;
        const float lw = std::hypot(wx, wy), lf = std::hypot(fx, fy);
        if (lw < 1.f || lf < 12.f) return true;
        return (wx * fx + wy * fy) / (lw * lf) >= kNearSideCos;
    }

    void Arrive(Steer& s, const standpoint_walk::SteerNotice& a_n)
    {
        auto*              a = ActorOf(s.who);
        const RE::NiPoint3 her = a ? a->GetPosition() : RE::NiPoint3{ a_n.r.x, a_n.r.y, a_n.r.z };
        const RE::NiPoint3 sp = s.spot.pos;
        const float        d = Dist2D(her, sp);
        const float        dz = std::fabs(her.z - sp.z);
        int                code = kC;
        int                det = StandpointAPI::kDetailClean;
        std::string        rule;
        if (d <= kOnPointU) {
            rule = "on the spot";
        } else if (CloseNpc(s)) {
            if (NearRule(s, her)) {
                det = StandpointAPI::kDetailArrivedShort;
                rule = "close: NEAR the spot (within 12 u, or 35 u on the asked side)";
            } else {
                code = kO;
                det = StandpointAPI::kDetailNoWay;
                rule = "close: NOT near the spot";
            }
        } else if (d < kStrayU && dz <= kStrayU) {
            det = StandpointAPI::kDetailArrivedShort;
            rule = "short of the spot";
        } else {
            code = kO;
            det = StandpointAPI::kDetailStrayed;
            rule = "too far from the spot to be an arrival";
        }
        logger::info("STEER #{} ARRIVED {:.1f} u from the spot (height {:.0f} u; walk #{} '{}'{}), facing off {:.1f} deg - {} -> {} {}", s.id, d, dz,
                     a_n.walkId, a_n.word, a_n.nearRule ? ", the near rule" : "", a_n.facingErrDeg, rule, LetterOf(code), det);
        if (code == kC) {
            s.phase = kArrived;
            s.code = kC;
            s.detail = det;
            s.released = !a_n.holding;
            s.resolving = false;
            return;
        }
        End(s, code, det, rule);
    }

    void NoRoute(Steer& s, int a_det, std::string_view a_word)
    {
        const bool relative = s.kind != kKindPoint;
        if (relative && a_det != StandpointAPI::kDetailOutsideLoaded && s.failedRels.empty() && s.spotKnown) {
            s.failedRels.push_back(s.spot.rel);
            s.firstSpot = s.spot;            // (FIX ROUND R1 #5: if no other side has a spot, this one's no-route answer stands)
            s.firstSpotKnown = true;
            s.firstDet = a_det;
            logger::info("STEER #{} NO ROUTE to hour {:.1f} (walk #{}: {} {}) - ONE more spot, on another side (2 hours away or more)", s.id,
                         s.spot.hours, s.walkId, a_det, a_word);
            s.walkId = 0;
            s.started = false;
            s.solveBegun = false;
            s.spotKnown = false;
            s.phase = kSolving;
            return;
        }
        FinalNoRoute(s, a_det, std::format("no route ({} {})", a_det, a_word));
    }

    void Wait(Steer& s, int a_reason)
    {
        s.phase = kWaiting;
        s.waitReason = a_reason;
        s.clearT = 0.f;
        s.resolving = false;
        logger::info("STEER #{} WAITS: {} (interrupts {}{}) - she walks on when it is over", s.id,
                     a_reason == 1 ? "the player's hand holds her" : a_reason == 2 ? "combat" : "a reaction (ragdoll / knocked / kill move)",
                     s.interrupts, s.targetIsPlayer ? ", toward the player" : "");
    }

    void OnAnswer(Steer& s, const standpoint_walk::SteerNotice& a_n)
    {
        const char             letter = a_n.r.outcome;
        const int              det = static_cast<int>(a_n.r.detail);
        const std::string_view word = a_n.word ? a_n.word : "";
        if (s.phase != kWalking) {
            logger::info("STEER #{} (walk #{} answered {} {} in phase {} - not used)", s.id, a_n.walkId, letter, det, PhaseName(s.phase));
            return;
        }
        switch (letter) {
        case 'C':
            Arrive(s, a_n);
            return;
        case 'O':
            if (det == StandpointAPI::kDetailNoSlot || det == StandpointAPI::kDetailBlocked || det == StandpointAPI::kDetailOutsideLoaded) {
                NoRoute(s, det, word);
            } else if (det == StandpointAPI::kDetailBadRequest) {
                End(s, kX, StandpointAPI::kDetailEnded, "she is not a loaded NPC any more");
            } else {
                End(s, kO, det, std::format("the walk answered O {} ({})", det, word));
            }
            return;
        case 'P':
            if (s.interrupts >= 1 && !s.targetIsPlayer) {
                End(s, kP, det, "a hand-grab (interrupts 1-2)");
            } else {
                Wait(s, 1);
            }
            return;
        case 'X':
            if (det == StandpointAPI::kDetailGameLoaded) return;
            if (det == StandpointAPI::kDetailEnded && word == "combat") {
                if (s.interrupts >= 2) {
                    End(s, kX, StandpointAPI::kDetailEnded, "combat (interrupts 2)");
                } else {
                    Wait(s, 2);
                }
                return;
            }
            if (det == StandpointAPI::kDetailEnded && word == "furniture") {
                // FIX ROUND (R1 #1): she sits (sat down before the walk could take her): the game's walk stands her up
                SitsEngineLeg(s, std::format("walk #{} refused: furniture", a_n.walkId).c_str());
                return;
            }
            if (det == StandpointAPI::kDetailEnded && (word == "ragdoll" || word == "knocked" || word == "kill-move")) {
                // FIX ROUND (R1 #8): a reaction is waited out like combat (1.5.1's travel package resumed after it); the backstop bounds it
                Wait(s, 3);
                return;
            }
            if (det == StandpointAPI::kDetailReleased && word == "session") return;    // (our own hand-off / stop)
            End(s, kX, det, std::format("the walk answered X {} ({})", det, word));
            return;
        case 'B':
            End(s, kB, det, std::format("the walk answered B {} ({})", det, word));
            return;
        default:
            End(s, kX, StandpointAPI::kDetailEnded, std::format("the walk answered '{}' {} ({})", letter, det, word));
            return;
        }
    }

    void OnNotice(const standpoint_walk::SteerNotice& a_n, void*)
    {
        SteerPtr sp = Find(a_n.steerId);
        if (!sp || sp->walkId != a_n.walkId) {
            // FIX ROUND (R1 #11): an ADOPTED walk still carries the id of the session it was opened by - found by its walk id
            if (SteerPtr adopter = FindByWalk(a_n.walkId, a_n.r.who)) sp = adopter;
        }
        if (!sp) return;                              // a session that is over (or replaced): its walks' ends are not ours
        Steer& s = *sp;
        {
            std::scoped_lock lk(g_mtx);
            if (s.superseded || s.stopWanted) return;
        }
        if (a_n.walkId == s.holdId && a_n.walkId != s.walkId) {
            if (!a_n.holdEnd) s.holdReady = true;     // the planning hold has taken her (or answered at once)
            return;
        }
        if (!s.walkId || a_n.walkId != s.walkId) return;   // an older walk of this session (a re-route's predecessor)
        if (a_n.holdEnd) {
            // the end of her stand after the C answer: the driver's own hand-off ("session"), or something else ended it. FIX ROUND
            // (R1 NIT 14): before the driver's pin it is still an ARRIVAL - phase 7: the driver pins her (or lets her go) and answers C
            if (s.phase == kArrived && std::string_view(a_n.word ? a_n.word : "") != "session") {
                s.phase = kStandEnded;
                logger::info("STEER #{} her stand ended ({}) before the driver's pin - still an arrival (the pin holds her, or she is let go)",
                             s.id, a_n.word);
            }
            Snapshot(s);
            return;
        }
        OnAnswer(s, a_n);
        Snapshot(s);
    }

    void DoHandOff(Steer& s)
    {
        if (s.phase == kStandEnded) {
            s.phase = kHandedOff;           // (her stand had ended already: the pin has her - nothing to hand off)
            logger::info("STEER #{} HANDOFF: the driver's pin has her (her stand had ended already)", s.id);
            return;
        }
        if (s.phase != kArrived || !s.walkId) {
            logger::info("STEER #{} HANDOFF asked in phase {} - nothing to hand off", s.id, PhaseName(s.phase));
            return;
        }
        const bool ok = standpoint_walk::HandOff(s.walkId);
        s.phase = kHandedOff;
        logger::info("STEER #{} HANDOFF: the driver's pin has her - her stand ends quietly (walk #{}{})", s.id, s.walkId, ok ? "" : ": already gone");
    }

    void Step(Steer& s, float a_dt, double a_budget)
    {
        if (s.phase == kSolving || s.phase == kPlanning || s.phase == kWalking || s.phase == kWaiting) {
            auto* a = ActorOf(s.who);
            if (!a || !a->Is3DLoaded() || a->IsDead()) {
                End(s, kX, StandpointAPI::kDetailEnded, "she is gone (not loaded, or dead)");
                return;
            }
            switch (s.phase) {
            case kSolving:  StepSolve(s, a, a_budget); break;
            case kPlanning: StepPlan(s, a, a_dt, a_budget); break;
            case kWalking:  StepWatch(s, a_budget); break;
            case kWaiting:  StepWait(s, a, a_dt); break;
            default: break;
            }
            return;
        }
        if (s.phase == kEnded || s.phase == kHandedOff) s.endedT += a_dt;
    }

    // ── the natives (class Standpoint; Papyrus VM threads: record / read only) ───────────────────────────────────
    bool SteerSupportedNative(RE::StaticFunctionTag*) { return Enabled(); }

    bool SteerCheckNative(RE::StaticFunctionTag*, RE::Actor* a_who, RE::TESObjectREFR* a_target, float a_x, float a_y, float a_z)
    {
        if (!Enabled()) return false;
        std::string why;
        if (!a_who || a_who->IsPlayerRef()) {
            why = "no walker (or the player)";
        } else if (!a_who->Is3DLoaded() || a_who->IsDead()) {
            why = std::format("{} is not a loaded, living NPC", NameOf(a_who, a_who->GetFormID()));
        } else if (a_target) {
            auto* cw = a_who->GetParentCell();
            auto* ct = a_target->GetParentCell();
            if (a_target == a_who) {
                why = "she is her own target (hold your place)";
            } else if (!cw || !ct) {
                why = "a cell could not be read";
            } else if (cw->IsInteriorCell() || ct->IsInteriorCell()) {
                if (cw != ct) why = std::format("the target {} is in another interior (a load door between them)", NameOf(a_target, a_target->GetFormID()));
            } else if (a_who->GetWorldspace() != a_target->GetWorldspace()) {
                why = std::format("the target {} is in another worldspace", NameOf(a_target, a_target->GetFormID()));
            } else if (!ct->IsAttached()) {
                why = std::format("the target {} stands in a cell that is not loaded", NameOf(a_target, a_target->GetFormID()));
            }
            if (why.empty() && a_target->As<RE::Actor>() && !a_target->Is3DLoaded()) {
                why = std::format("the target {} is not loaded", NameOf(a_target, a_target->GetFormID()));
            }
        } else if (!std::isfinite(a_x) || !std::isfinite(a_y) || !std::isfinite(a_z)) {
            why = "the point is not a number";
        }
        if (!why.empty()) {
            logger::info("STEER check: {} - the ENGINE session walks this one", why);
            return false;
        }
        return true;
    }

    std::int32_t SteerStartNative(RE::StaticFunctionTag*, RE::Actor* a_who, std::int32_t a_slot, RE::TESObjectREFR* a_target,
                                  std::vector<float> a_ask)
    {
        if (!a_who || !Enabled() || a_ask.size() < 5) return 0;
        const int kind = static_cast<int>(a_ask[0]);
        if (kind < kKindClock || kind > kKindPoint) return 0;
        if (kind != kKindPoint && !a_target) return 0;
        if ((kind == kKindClock && a_ask.size() < 8) || (kind == kKindGoTo && a_ask.size() < 7) || (kind == kKindPoint && a_ask.size() < 9)) return 0;
        auto s = std::make_shared<Steer>();
        s->who = a_who->GetFormID();
        s->target = a_target ? a_target->GetFormID() : 0;
        s->slot = a_slot;
        s->kind = kind;
        s->hold = a_ask[1] < 0.f ? -1.f : a_ask[1];
        s->pose = static_cast<int>(a_ask[2]);
        s->interrupts = static_cast<int>(a_ask[3]);
        s->close = a_ask[4] > 0.5f;
        if (kind == kKindClock) {
            s->distCm = a_ask[5];
            s->facingH = a_ask[6];
            s->orientH = a_ask[7];
        } else if (kind == kKindGoTo) {
            s->relation = static_cast<int>(a_ask[5]);
            s->standoffU = a_ask[6];
        } else {
            s->point = RE::NiPoint3{ a_ask[5], a_ask[6], a_ask[7] };
            s->pointHeadingDeg = a_ask[8];
        }
        s->targetIsActor = a_target && a_target->As<RE::Actor>() != nullptr;
        s->targetIsPlayer = a_target && a_target->IsPlayerRef();
        s->targetBare = a_target && !s->targetIsActor && !a_target->Get3D();
        std::string replaced;
        {
            std::scoped_lock lk(g_mtx);
            s->id = g_nextId++;
            if (g_nextId > 0x00FFFFFF) g_nextId = 1;     // stays exact as a Papyrus Int / Float
            for (const auto& [id, o] : g_steers) {
                if (o->who == s->who && !o->superseded) {
                    o->superseded = true;                 // her older steered session: replaced (its walk keeps her held
                    o->supersededBy = s->id;              // until this one's own walk takes her - FIX ROUND R1 #4)
                    replaced += std::format(" #{}", id);
                    // FIX ROUND (R1 #11): a SAME-KIND retask of a session that is WALKING (VRCD re-solves and calls GoToPoint again
                    // on a 100 u target move) re-aims that walk on the move (Retarget) instead of a new one with a planning hold.
                    // 1.6.1 (review R1 § VERIFY N2): only within the same HOLD CLASS - a re-aimed walk keeps the hold it was opened
                    // with (Retarget does not change it): "let go at the point" (hold 0 + pose 0) and "the driver pins her" never mix
                    const bool sameKind = o->kind == s->kind && (s->kind == kKindPoint || o->target == s->target);
                    const bool sameHold = (o->hold == 0.f && o->pose == 0) == (s->hold == 0.f && s->pose == 0);
                    const auto oWalk = static_cast<std::uint32_t>(o->snap[14]);
                    if (sameKind && sameHold && o->phaseShared == kWalking && oWalk) s->adoptWalk = oWalk;
                }
            }
            if (s->adoptWalk) replaced += std::format(" (a same-kind RETASK: her walk #{} is re-aimed, no stop)", s->adoptWalk);
            s->snap[0] = static_cast<float>(kSolving);
            g_steers[s->id] = s;
        }
        std::string what;
        if (kind == kKindClock) {
            what = std::format("the clock: {:.0f} cm at hour {:.1f}, orientation {:.1f}h{}", s->distCm, s->facingH, s->orientH, s->close ? ", CLOSE" : "");
        } else if (kind == kKindGoTo) {
            what = std::format("GoTo: relation {}, standoff {:.0f} u", s->relation, s->standoffU);
        } else {
            what = std::format("GoToPoint: ({:.0f},{:.0f},{:.0f}) heading {:.0f} deg", s->point.x, s->point.y, s->point.z, s->pointHeadingDeg);
        }
        logger::info("STEER #{} REQUEST session {}: {} -> {} ({}; pose {}, hold {}, interrupts {}){}", s->id, a_slot, NameOf(a_who, s->who),
                     a_target ? NameOf(a_target, s->target) : std::string("a point"), what, s->pose,
                     s->hold < 0.f ? std::string("-1") : std::format("{:.1f} s", s->hold), s->interrupts,
                     replaced.empty() ? std::string() : std::format(" - replaces STEER{}", replaced));
        standpoint_walk::NoteForeign(s->who);     // a session start ends the stpf follow program for her (as WalkStop did)
        standpoint_walk::Wake();
        return s->id;
    }

    std::vector<float> SteerStateNative(RE::StaticFunctionTag*, std::int32_t a_id)
    {
        std::vector<float> out;
        std::scoped_lock   lk(g_mtx);
        const auto         it = g_steers.find(a_id);
        if (it == g_steers.end()) return out;
        out.assign(it->second->snap.begin(), it->second->snap.end());
        return out;
    }

    void SteerHandOffNative(RE::StaticFunctionTag*, std::int32_t a_id)
    {
        {
            std::scoped_lock lk(g_mtx);
            const auto       it = g_steers.find(a_id);
            if (it == g_steers.end()) return;
            it->second->handOffWanted = true;
        }
        standpoint_walk::Wake();
    }

    void SteerStopNative(RE::StaticFunctionTag*, std::int32_t a_id)
    {
        {
            std::scoped_lock lk(g_mtx);
            const auto       it = g_steers.find(a_id);
            if (it == g_steers.end()) return;
            it->second->stopWanted = true;
        }
        standpoint_walk::Wake();
    }

}  // namespace

void Frame(float a_dt)
{
    std::vector<SteerPtr> list;
    {
        std::scoped_lock lk(g_mtx);
        if (g_steers.empty()) return;
        list.reserve(g_steers.size());
        for (const auto& [id, s] : g_steers) list.push_back(s);
    }
    int working = 0;
    for (const auto& s : list) {
        if (s->phase == kSolving || s->phase == kPlanning || s->resolving) ++working;
    }
    const double budget = (std::max)(kBudgetMinMs, kBudgetMs / static_cast<double>((std::max)(1, working)));
    std::vector<std::int32_t> drop;
    // FIX ROUND (R1 #4): a REPLACED session first - its walk and planning hold go to the session that replaced it (which stops them
    // if it ends before opening a walk of its own; its own first walk replaces them with WalkApi's hand-over). Never owner-less.
    for (const auto& sp : list) {
        Steer&       s = *sp;
        bool         superseded = false;
        std::int32_t by = 0;
        {
            std::scoped_lock lk(g_mtx);
            superseded = s.superseded;
            by = s.supersededBy;
        }
        if (!superseded) continue;
        // the FINAL successor (two retasks between two frames: A -> B -> C - the walks go to C)
        SteerPtr heir = by ? Find(by) : nullptr;
        for (int hop = 0; heir && hop < 8; ++hop) {
            bool         heirGone = false;
            std::int32_t next = 0;
            {
                std::scoped_lock lk(g_mtx);
                heirGone = heir->superseded;
                next = heir->supersededBy;
            }
            if (!heirGone) break;
            heir = next ? Find(next) : nullptr;
        }
        if (heir && heir->phase == kSolving && !heir->started) {
            if (s.walkId) heir->inherited.push_back(s.walkId);
            if (s.holdId && s.holdId != s.walkId) heir->inherited.push_back(s.holdId);
            for (const auto id : s.inherited) heir->inherited.push_back(id);
            s.inherited.clear();
            logger::info("STEER #{} replaced by STEER #{} (phase {}) - its walk #{} / hold #{} go to the new session until it opens its own",
                         s.id, heir->id, PhaseName(s.phase), s.walkId, s.holdId);
        } else if (s.phase != kHandedOff && s.phase != kEnded) {
            // (the new session is gone or already walks: nobody would ever end this one's walk - it is stopped now)
            logger::info("STEER #{} replaced by STEER #{} (phase {}) - no session takes its walk over: it is stopped", s.id, by, PhaseName(s.phase));
            if (!heir || heir->walkId != s.walkId) StopWalks(s, "replaced, and no newer session took the walk over");
        }
        drop.push_back(s.id);
    }
    for (const auto& sp : list) {
        Steer& s = *sp;
        bool   superseded = false, stop = false, handOff = false;
        {
            std::scoped_lock lk(g_mtx);
            superseded = s.superseded;
            stop = s.stopWanted;
            handOff = s.handOffWanted;
            s.handOffWanted = false;
            if (s.adoptWalk) {
                s.adoptMain = s.adoptWalk;      // (FIX ROUND R1 #11: the walk to re-aim - main-thread copy)
                s.adoptWalk = 0;
            }
        }
        if (superseded) continue;               // (handled above)
        if (stop) {
            logger::info("STEER #{} STOP (the driver freed the session in phase {})", s.id, PhaseName(s.phase));
            if (s.phase != kHandedOff) StopWalks(s, "the session was freed");
            drop.push_back(s.id);
            continue;
        }
        if (handOff) DoHandOff(s);
        Step(s, a_dt, budget);
        if ((s.phase == kEnded || s.phase == kHandedOff) && s.endedT > kKeepEndedS) {
            drop.push_back(s.id);
            continue;
        }
        Snapshot(s);
    }
    if (!drop.empty()) {
        std::scoped_lock lk(g_mtx);
        for (const auto id : drop) g_steers.erase(id);
    }
}

bool Live()
{
    std::scoped_lock lk(g_mtx);
    for (const auto& [id, s] : g_steers) {
        if (s->superseded || s->stopWanted || s->handOffWanted) return true;
        const int p = s->phaseShared;
        if (p == kSolving || p == kPlanning || p == kWalking || p == kWaiting) return true;
    }
    return false;
}

void Init()
{
    const int def = REL::Module::IsVR() ? 1 : 0;
    std::string note;
    const int   v = standpoint_ini::ReadInt("Session", "Steered", def, note);
    g_steered.store(v == 1 ? 1 : 0);
    g_steerNote = note;
    logger::info("steered sessions (GoTo / GoToPoint / RequestMove walked by Standpoint's own steering): {} ({}; the default on {} is {}) - "
                 "they also need the walk off the navmesh (VR, or SE / AE with [DirectWalk] SEAE = 1)",
                 v == 1 ? "ON" : "OFF - the engine's walk as in 1.5.1", note, REL::Module::IsVR() ? "VR" : "SE / AE", def);
}

bool Enabled()
{
    return g_steered.load() == 1 && standpoint_walk::Supported();
}

void OnGameLoad()
{
    std::scoped_lock lk(g_mtx);
    g_steers.clear();
}

bool RegisterNatives(RE::BSScript::IVirtualMachine* a_vm)
{
    a_vm->RegisterFunction("SteerSupported", "Standpoint", SteerSupportedNative);
    a_vm->RegisterFunction("SteerCheck", "Standpoint", SteerCheckNative);
    a_vm->RegisterFunction("SteerStart", "Standpoint", SteerStartNative);
    a_vm->RegisterFunction("SteerState", "Standpoint", SteerStateNative);
    a_vm->RegisterFunction("SteerHandOff", "Standpoint", SteerHandOffNative);
    a_vm->RegisterFunction("SteerStop", "Standpoint", SteerStopNative);
    logger::info("steered-session natives registered (class Standpoint, 6 functions)");
    return true;
}

}  // namespace standpoint_steer
