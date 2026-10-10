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
//  WHY ONE COMMAND (v0.8.2, 2026-10-09): v0.8.0 / v0.8.1 registered two (stpw + stpt). In this load order the free
//  vanilla console slots are almost gone; Standpoint registers before the VRCD AddOn and its second command took
//  'ToggleDecompressToCopy' - the slot the AddOn's `vrcw` walk test lands on. VRTEDD_AddOn.log 2026-10-09 08:18:
//  "no free console command left for 'vrcw'". One command on a slot the AddOn does not need gives `vrcw` back.
// ─────────────────────────────────────────────────────────────────────────────
#include "PCH.h"
#include "MoveApi.h"

#include <atomic>
#include <chrono>
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
                        "Every line also goes to Documents\\My Games\\Skyrim VR\\SKSE\\Standpoint.log",
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
    standpoint_pair::ClearAll();    // the world is being rebuilt: all four pair-rule slots off at once
}

void RegisterConsole()
{
    // An unused vanilla console command, re-purposed (the po3 / CollViz pattern; KNOWLEDGEBASE "Console-command DONOR SLOT
    // REGISTRY"). A donor is taken only while it still has its vanilla name.
    // ⚠ v0.8.2: ONE command, and NEVER one of 'TestDegrade', 'TestHandleManagerWarnAndKillSDM', 'ToggleDecompressToCopy' -
    // the three slots the VRCD AddOn's ddab / vrcd / vrcw land on (it registers AFTER Standpoint, so whatever Standpoint
    // takes from its list is gone for it). Measured 2026-10-09: of the whole shared candidate list only those three and
    // 'DumpConditionFunctions' exist un-renamed in this load order (PPB holds 'ToggleTrijuice' + 'StopTrackPlayerDoors').
    static constexpr const char* kDonors[] = { "DumpConditionFunctions", "TestCameraPath" };
    struct Cmd { const char* longName; const char* shortName; const char* help; RE::SCRIPT_FUNCTION::Execute_t* fn; };
    const Cmd cmds[] = {
        { "StandpointTestMove", "stpt", "Select an NPC, stpt = she is the walker; select a target, stpt = she is sent around it through Standpoint's move API (again = stop)", &ExecTest },
    };
    std::size_t d = 0;
    for (const auto& c : cmds) {
        RE::SCRIPT_FUNCTION* slot = nullptr;
        const char* donor = nullptr;
        for (; d < std::size(kDonors) && !slot; ++d) {
            slot = RE::SCRIPT_FUNCTION::LocateConsoleCommand(kDonors[d]);
            donor = kDonors[d];
        }
        if (!slot) {
            logger::warn("[TEST] no free console command left for '{}' - the command is unavailable", c.shortName);
            continue;
        }
        slot->functionName        = c.longName;
        slot->shortName           = c.shortName;
        slot->helpString          = c.help;
        slot->referenceFunction   = false;          // the ref + zero-param combo dies in the console compiler (CollViz v2)
        slot->SetParameters();
        slot->executeFunction     = c.fn;
        slot->conditionFunction   = nullptr;
        slot->editorFilter        = false;
        slot->invalidatesCellList = false;
        logger::info("[TEST] console command '{}' ({}) registered on the unused vanilla '{}'", c.shortName, c.longName, donor);
    }
}

}  // namespace standpoint_test
