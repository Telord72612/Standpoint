// ─────────────────────────────────────────────────────────────────────────────
//  Interrupts.cpp — v0.5.0: player-intercept RECOGNITION for traveling walkers.
//
//  OBSERVATION ONLY. Nothing here suppresses, blocks, or alters any engine
//  behaviour — teammates keep their vanilla/FollowerBumpGuard bump reactions
//  untouched. Standpoint merely RECOGNISES two things and reports them to the
//  Papyrus driver, which decides what a travel session does about them:
//
//  1. BUMP RECOGNITION (technique ported from FollowerBumpGuard, observation
//     half only): the engine's bump reaction manifests as a run-once package of
//     packType 32 in MiddleHighProcessData::runOncePackage (FBG measured 32 in
//     VR — CommonLib's enum mislabels it). Reading that slot is a pure lookup.
//     WHO bumped is attributed in the driver (player-proximity gate).
//
//  2. VR HAND-GRAB (HIGGS): the grabbed/dropped callbacks track what the
//     player's hands hold. Grabbing an actor's body (PLANCK makes actor bodies
//     grabbable) reports the ACTOR ref — if that actor is a walker, the driver
//     stops the travel. Degrades to absent on flat SE/AE (no HIGGS): natives
//     return false and the proximity/bump paths still work.
// ─────────────────────────────────────────────────────────────────────────────
#include "PCH.h"
#include "HiggsInterface.h"
#include "MoveApi.h"

#include <atomic>
#include <chrono>
#include <format>
#include <thread>

namespace logger = SKSE::log;

namespace {

    Higgs::IHiggsInterface001* g_higgs = nullptr;

    // Per-hand grabbed reference FormIDs, written from HIGGS's callbacks.
    std::atomic<std::uint32_t> g_grabbed[2] = { 0, 0 };

    struct HiggsMessage { void* (*GetApiFunction)(unsigned int revisionNumber) = nullptr; };
    constexpr std::uint32_t kHiggsGetInterface = 0xF9279A57u;

    void OnGrabbed(bool a_isLeft, RE::TESObjectREFR* a_refr)
    {
        g_grabbed[a_isLeft ? 0 : 1].store(a_refr ? a_refr->GetFormID() : 0);
    }

    void OnDropped(bool a_isLeft, RE::TESObjectREFR* a_refr)
    {
        (void)a_refr;
        g_grabbed[a_isLeft ? 0 : 1].store(0);
    }

    // ── v0.9.0: THE PAIR RULE - "these two character controllers ignore each other" (report 05, lever B) ──
    // What keeps two NPCs apart is their character controllers: the two bodies meet at about 56-65 cm centre to
    // centre (measured, report 05 §10.2 / §12.2 - the 0.9.0 text here blamed the forward bumper capsule; wrong).
    // PLANCK already makes the PLAYER's controller ignore every NPC controller through this same HIGGS hook; this
    // does it for a pair of actors. Nothing is resized, so walls, steps and doors are untouched and there is
    // nothing to restore.
    //
    // v0.9.3 made the rule part of CLOSE mode: the driver arms it when a close session with an NPC target starts
    // (native Standpoint.PairArm) and releases it when that session ends (native Standpoint.PairRelease). VR-proven on
    // ONE pair in the second close-contact run (report 05 §12): 41 cm front and 46 cm right, the target not moved.
    //
    // v0.9.4: A POOL OF 8 PAIRS, not tied to the driver's session slots. (0.9.3 had one pair per session slot: a new
    // session in a slot switched that slot's old pair off at once - possibly while those two still overlapped. It was
    // replaced before any VR run.)
    //   * Arming a pair that is already in the pool (armed, or its release pending) reuses that entry: the pending
    //     release is cancelled and the use is counted. A release takes one use away; only at count 0 does the
    //     "release when apart" start.
    //   * A new pair takes a free entry. None free: the OLDEST release-pending entry is switched off at once and
    //     reused. None pending either: the arm fails (handle 0).
    //
    // Rules of the hook (it runs on Havok's collision threads, for every pair the world compares): integers only,
    // no logging, no locks. The first registered callback that does not answer Continue wins - PLANCK answers
    // Continue for two NPC controllers, the others are unchecked, hence the counters: if g_pairHits stays 0 while a
    // walker presses on her target, somebody answered before us.
    // KNOWN LIMIT: an Ignore only affects a pair whose boxes START to overlap after it is armed (an existing contact
    // stays until the two separate).
    //
    // TWO HALVES. What the filter reads is ONE atomic word per entry - the packed pair, (lower group << 16) | higher
    // group, 0 = off - and nothing else: no lock. The bookkeeping (who, use count, pending, serial) sits behind a
    // mutex that only the natives (Papyrus VM threads) and main-thread tasks take. The filter never takes it.
    constexpr std::uint32_t        kLayerCharController = 30;
    constexpr int                  kPairEntries = 8;
    constexpr float                kPairApartUnits = 90.f;     // a released pair is switched off at this flat distance
    std::atomic<std::uint32_t>     g_pairGroups[kPairEntries] = { 0, 0, 0, 0, 0, 0, 0, 0 };   // ALL the filter reads
    std::atomic<std::uint32_t>     g_pairHits{ 0 };     // comparisons of an armed pair that reached us (we answered Ignore)
    std::atomic<std::uint32_t>     g_ccSeen{ 0 };       // controller x controller comparisons that reached us while armed
    bool                           g_pairHooked = false;

    struct PairEntry {
        RE::FormID    a{ 0 }, b{ 0 };        // the two actors
        std::uint32_t groups{ 0 };           // what g_pairGroups[i] holds; 0 = the entry is free
        std::int32_t  uses{ 0 };             // how many sessions hold it
        bool          pending{ false };      // its release is ordered: off once the two are apart
        std::uint32_t serial{ 0 };           // changes with every arm, release order and switch-off: voids a sleeping check
        std::uint32_t gen{ 0 };              // changes when the entry is given to a pair: voids that entry's old handles
        std::uint64_t pendingSince{ 0 };     // the order of the release orders (the oldest pending entry is the one given up)
    };
    std::mutex    g_pairMtx;                 // the bookkeeping only - NEVER taken by the filter
    PairEntry     g_pair[kPairEntries];
    std::uint64_t g_pairOrder = 0;

    Higgs::IHiggsInterface001::CollisionFilterComparisonResult PairFilter(void*, std::uint32_t a_infoA, std::uint32_t a_infoB)
    {
        using R = Higgs::IHiggsInterface001::CollisionFilterComparisonResult;
        // the fast path: anything that is not two character controllers leaves on its two arguments alone
        if ((a_infoA & 0x7F) != kLayerCharController || (a_infoB & 0x7F) != kLayerCharController) return R::Continue;
        std::uint32_t g[kPairEntries];
        std::uint32_t any = 0;
        for (int i = 0; i < kPairEntries; ++i) {
            g[i] = g_pairGroups[i].load(std::memory_order_relaxed);
            any |= g[i];
        }
        if (!any) return R::Continue;                     // nothing armed
        g_ccSeen.fetch_add(1, std::memory_order_relaxed);
        std::uint32_t ga = a_infoA >> 16, gb = a_infoB >> 16;
        if (ga > gb) { const std::uint32_t t = ga; ga = gb; gb = t; }
        // An armed entry never holds a group 0 (Arm refuses it), so a pair with a group 0 is never ours. The test is
        // needed: two controllers that both sit in group 0 pack to 0, which is also what a FREE entry holds.
        if (!ga || !gb) return R::Continue;
        const std::uint32_t pair = (ga << 16) | gb;
        for (int i = 0; i < kPairEntries; ++i) {
            if (g[i] == pair) {
                g_pairHits.fetch_add(1, std::memory_order_relaxed);
                return R::Ignore;
            }
        }
        return R::Continue;
    }

    // The collision filter word of the actor's controller body (0 = none). The route is FollowerBumpGuard's.
    std::uint32_t ControllerFilter(RE::Actor* a_actor)
    {
        if (!a_actor) return 0;
        auto* con = a_actor->GetCharController();
        if (!con) return 0;
        auto* rb = con->GetRigidBody();
        return rb ? rb->collidable.broadPhaseHandle.collisionFilterInfo.filter : 0;
    }

    // A handle names an entry AND the pair it was given to: (gen << 4) | (index + 1). Always >= 1; 0 = "not armed".
    constexpr std::uint32_t kPairGenMask = 0x07FFFFFFu;
    std::int32_t HandleOf(int a_index, std::uint32_t a_gen)
    {
        return static_cast<std::int32_t>(((a_gen & kPairGenMask) << 4) | static_cast<std::uint32_t>(a_index + 1));
    }

    // g_pairMtx held. The entry a handle names, -1 = no live pair (a free entry, another pair's entry, rubbish).
    int EntryOfHandle(std::int32_t a_handle)
    {
        if (a_handle <= 0) return -1;
        const int idx = (a_handle & 0xF) - 1;
        if (idx < 0 || idx >= kPairEntries) return -1;
        const PairEntry& e = g_pair[idx];
        if (!e.groups || (e.gen & kPairGenMask) != (static_cast<std::uint32_t>(a_handle) >> 4)) return -1;
        return idx;
    }

    // g_pairMtx held. The entry is free from here on; the filter sees it at once.
    void SwitchOffLocked(int a_index)
    {
        PairEntry& e = g_pair[a_index];
        g_pairGroups[a_index].store(0);
        e.groups = 0;
        e.uses = 0;
        e.pending = false;
        e.pendingSince = 0;
        e.a = e.b = 0;
        ++e.serial;
    }

    // "Release when apart". A released pair is never switched off while the two are still close: that would hand the
    // physics two interpenetrating controllers at its next re-check. Off = once they are 90 u apart (flat), or one of
    // them is gone / has no loaded 3D; looked at once a second until then (the first look one second after the order,
    // so a retask - release, then the same pair armed again a moment later - never sees a gap).
    // This DLL has no tick, so: a detached thread sleeps, then adds ONE task for the main thread. NEVER a task that
    // adds itself again directly - SKSE drains its task queue until it is empty, the game would freeze.
    void CheckPairRelease(int a_index, std::uint32_t a_serial);

    void CheckPairReleaseLater(int a_index, std::uint32_t a_serial, float a_seconds)
    {
        std::thread([a_index, a_serial, a_seconds]() {
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(a_seconds * 1000.f)));
            if (auto* task = SKSE::GetTaskInterface()) {
                task->AddTask([a_index, a_serial]() { CheckPairRelease(a_index, a_serial); });
            }
        }).detach();
    }

    // Main thread (a task). a_serial = the entry's serial at the moment its release was ordered.
    void CheckPairRelease(int a_index, std::uint32_t a_serial)
    {
        RE::FormID ia = 0, ib = 0;
        {
            std::scoped_lock lk(g_pairMtx);
            const PairEntry& e = g_pair[a_index];
            if (e.serial != a_serial || !e.pending || !e.groups) return;    // armed again, cleared, or a newer order
            ia = e.a;
            ib = e.b;
        }
        // game objects are read without the lock
        auto* a = ia ? RE::TESForm::LookupByID<RE::Actor>(ia) : nullptr;
        auto* b = ib ? RE::TESForm::LookupByID<RE::Actor>(ib) : nullptr;
        float cc = 1e9f;
        if (a && b && a->Is3DLoaded() && b->Is3DLoaded()) {
            const RE::NiPoint3 pa = a->GetPosition(), pb = b->GetPosition();
            cc = std::sqrt((pa.x - pb.x) * (pa.x - pb.x) + (pa.y - pb.y) * (pa.y - pb.y));
        }
        if (cc < kPairApartUnits) {
            CheckPairReleaseLater(a_index, a_serial, 1.f);
            return;
        }
        {
            std::scoped_lock lk(g_pairMtx);
            const PairEntry& e = g_pair[a_index];
            if (e.serial != a_serial || !e.pending) return;                 // armed again while we measured
            SwitchOffLocked(a_index);
        }
        if (cc > 1e8f) {
            logger::info("pair rule #{}: off ({:08X} / {:08X} - one of the two is gone)", a_index, ia, ib);
        } else {
            logger::info("pair rule #{}: off ({:08X} / {:08X} are {:.0f} u apart)", a_index, ia, ib, cc);
        }
    }

}  // namespace

namespace standpoint_interrupts {

void AcquireHiggs()  // call at kDataLoaded; idempotent; VR-only by construction
{
    if (g_higgs) return;
    if (!::GetModuleHandleA("higgs_vr.dll")) {
        logger::info("interrupts: HIGGS absent — grab recognition disabled (flat edition or no HIGGS)");
        return;
    }
    auto* msging = SKSE::GetMessagingInterface();
    if (!msging) return;
    HiggsMessage hm;
    msging->Dispatch(kHiggsGetInterface, static_cast<void*>(&hm), sizeof(HiggsMessage*), "HIGGS");
    if (!hm.GetApiFunction) {
        logger::warn("interrupts: HIGGS did not answer the interface handshake");
        return;
    }
    g_higgs = static_cast<Higgs::IHiggsInterface001*>(hm.GetApiFunction(1));
    if (g_higgs) {
        g_higgs->AddGrabbedCallback(OnGrabbed);
        g_higgs->AddDroppedCallback(OnDropped);
        g_higgs->AddCollisionFilterComparisonCallback(PairFilter);   // v0.9.0: answers Continue unless a pair is armed
        g_pairHooked = true;
        logger::info("interrupts: the pair rule's collision-filter callback is registered (idle until a pair is armed)");
        logger::info("interrupts: HIGGS grab recognition ON (build {})", g_higgs->GetBuildNumber());
    }
}

// True while either of the player's HIGGS hands holds this actor.
bool IsPlayerGrabbing(RE::Actor* a_actor)
{
    if (!a_actor || !g_higgs) return false;
    const auto id = a_actor->GetFormID();
    return id != 0 && (g_grabbed[0].load() == id || g_grabbed[1].load() == id);
}

// The packType sitting in the actor's run-once package slot, -1 if none.
// Bump reaction = 32 (FollowerBumpGuard's in-VR measurement; the technique is
// FBG's RunOnceType, carried here observation-only with its source citations:
// Actor.h:705 currentProcess, AIProcess.h:201 middleHigh,
// MiddleHighProcessData.h:136 runOncePackage, TESPackage.h:204 packType).
std::int32_t GetRunOncePackageTypeId(RE::Actor* a_actor)
{
    if (!a_actor) return -1;
    auto* proc = a_actor->GetActorRuntimeData().currentProcess;
    if (!proc || !proc->middleHigh) return -1;
    auto* p = proc->middleHigh->runOncePackage.package;
    return p ? static_cast<std::int32_t>(p->packData.packType.underlying()) : -1;
}

}  // namespace standpoint_interrupts

// ── the pair rule (see PairFilter above). v0.9.4: a pool of 8 pairs with handles and use counts. Arm and Release
//    are called by the driver's natives on Papyrus VM threads; the release checks and ClearAll run on the main
//    thread. The bookkeeping is behind g_pairMtx; no game object is touched while it is held, and (v0.9.5)
//    nothing is logged while it is held: the text is built under the lock and written after it. ────────────────
namespace standpoint_pair {

std::int32_t Arm(RE::Actor* a_a, RE::Actor* a_b)
{
    if (!g_pairHooked || !a_a || !a_b || a_a == a_b) return 0;
    // The one read of game objects: the two controller bodies' filter words (three pointers each, nothing is
    // written) and the two positions for the log. It runs on a Papyrus VM thread, like the SolveClock native's
    // reads of the same actors. Done BEFORE the lock is taken.
    const std::uint32_t fa = ControllerFilter(a_a), fb = ControllerFilter(a_b);
    std::uint32_t ga = fa >> 16, gb = fb >> 16;
    const RE::FormID ia = a_a->GetFormID(), ib = a_b->GetFormID();
    if (!ga || !gb || ga == gb) {
        logger::warn("pair rule: NOT armed for {:08X} (filter {:08X}) / {:08X} (filter {:08X}) - a controller has "
                     "no body, no group, or both share one group", ia, fa, ib, fb);
        return 0;
    }
    const RE::NiPoint3 pa = a_a->GetPosition(), pb = a_b->GetPosition();
    const float cc = std::sqrt((pa.x - pb.x) * (pa.x - pb.x) + (pa.y - pb.y) * (pa.y - pb.y));
    if (ga > gb) { const std::uint32_t t = ga; ga = gb; gb = t; }
    const std::uint32_t pair = (ga << 16) | gb;

    // v0.9.5: NOTHING IS LOGGED WHILE THE MUTEX IS HELD. The text is built under the lock and written after it.
    std::string  noteOff;           // "off at once" - the pool was full and a waiting entry was given up
    std::string  note;              // KEPT / ARMED / NOT armed
    bool         failed = false;
    std::int32_t handle = 0;
    {
        std::scoped_lock lk(g_pairMtx);

        // 1. the pair is in the pool already (armed, or its release pending): the entry is reused, the rule never drops
        int idx = -1;
        for (int i = 0; i < kPairEntries && idx < 0; ++i) {
            const PairEntry& p = g_pair[i];
            if (p.groups && ((p.a == ia && p.b == ib) || (p.a == ib && p.b == ia))) idx = i;
        }
        if (idx >= 0) {
            PairEntry& e = g_pair[idx];
            const bool wasPending = e.pending;
            e.pending = false;
            e.pendingSince = 0;
            ++e.serial;                              // a release check still sleeping is void from here on
            ++e.uses;
            if (e.groups != pair) {                  // a controller came back with another group (her 3D was reloaded)
                e.groups = pair;
                g_pairGroups[idx].store(pair);
            }
            note = std::format("pair rule #{}: KEPT for {:08X} / {:08X} (groups {} / {}, {} use(s)){}; they are {:.0f} u apart",
                               idx, ia, ib, ga, gb, e.uses, wasPending ? " - its pending release is cancelled" : "", cc);
            handle = HandleOf(idx, e.gen);
        } else {
            // 2. a free entry
            for (int i = 0; i < kPairEntries && idx < 0; ++i) {
                if (!g_pair[i].groups) idx = i;
            }
            // 3. none free: the OLDEST release-pending entry is switched off at once and reused
            if (idx < 0) {
                std::uint64_t oldest = (std::numeric_limits<std::uint64_t>::max)();
                for (int i = 0; i < kPairEntries; ++i) {
                    if (g_pair[i].pending && g_pair[i].pendingSince < oldest) {
                        oldest = g_pair[i].pendingSince;
                        idx = i;
                    }
                }
                if (idx < 0) {
                    failed = true;
                    note = std::format("pair rule: NOT armed for {:08X} / {:08X} - all {} entries are in use and none is "
                                       "waiting for its release", ia, ib, kPairEntries);
                } else {
                    noteOff = std::format("pair rule #{}: off at once ({:08X} / {:08X}, its release was pending) - the pool is "
                                          "full and a new pair needs the entry", idx, g_pair[idx].a, g_pair[idx].b);
                    SwitchOffLocked(idx);
                }
            }
            if (idx >= 0) {
                PairEntry& e = g_pair[idx];
                e.a = ia;
                e.b = ib;
                e.groups = pair;
                e.uses = 1;
                e.pending = false;
                e.pendingSince = 0;
                ++e.serial;
                ++e.gen;
                g_pairGroups[idx].store(pair);
                note = std::format("pair rule #{}: ARMED - the controllers of {:08X} (filter {:08X}) and {:08X} (filter {:08X}) "
                                   "ignore each other (groups {} / {}); they are {:.0f} u apart{}",
                                   idx, ia, fa, ib, fb, ga, gb, cc,
                                   cc < kPairApartUnits ? " | UNDER 90 u: their boxes may already overlap - the rule then bites only after they have separated once" : "");
                handle = HandleOf(idx, e.gen);
            }
        }
    }
    if (!noteOff.empty()) logger::info("{}", noteOff);
    if (failed) {
        logger::warn("{}", note);
    } else {
        logger::info("{}", note);
    }
    return handle;
}

void Release(std::int32_t a_handle)
{
    int           idx = -1;
    std::uint32_t serial = 0;
    bool          ordered = false;      // the release-when-apart starts now
    std::string   note;                 // v0.9.5: built under the lock, logged after it
    {
        std::scoped_lock lk(g_pairMtx);
        idx = EntryOfHandle(a_handle);
        if (idx < 0) {
            note = std::format("pair rule: release of handle {} ignored - it names no live pair (already off, or cleared)", a_handle);
        } else {
            PairEntry& e = g_pair[idx];
            if (e.uses > 0) --e.uses;
            if (e.uses > 0) {
                note = std::format("pair rule #{}: one use less for {:08X} / {:08X} - {} left, the rule stays", idx, e.a, e.b, e.uses);
            } else if (!e.pending) {
                e.pending = true;
                e.pendingSince = ++g_pairOrder;
                ++e.serial;
                serial = e.serial;
                ordered = true;
                note = std::format("pair rule #{}: release ordered ({:08X} / {:08X}) - off once the two are {:.0f} u apart",
                                   idx, e.a, e.b, kPairApartUnits);
            }
            // (count 0 and pending already: its release is ordered, that check is running - nothing to do or to say)
        }
    }
    if (!note.empty()) logger::info("{}", note);
    if (ordered) CheckPairReleaseLater(idx, serial, 1.f);
}

void ClearAll()
{
    std::vector<std::string> notes;     // v0.9.5: built under the lock, logged after it
    {
        std::scoped_lock lk(g_pairMtx);
        for (int i = 0; i < kPairEntries; ++i) {
            if (g_pair[i].groups) {
                notes.push_back(std::format("pair rule #{}: off ({:08X} / {:08X} - a save is being loaded, the whole pool is "
                                            "cleared at once)", i, g_pair[i].a, g_pair[i].b));
            }
            SwitchOffLocked(i);
        }
    }
    for (const auto& n : notes) logger::info("{}", n);
}

void Stats(std::uint32_t& a_hits, std::uint32_t& a_ccSeen)
{
    a_hits   = g_pairHits.load();
    a_ccSeen = g_ccSeen.load();
}

std::uint32_t ArmedFor(RE::FormID a_a, RE::FormID a_b, int& a_entry, bool& a_releasePending, int& a_uses)
{
    a_entry = -1;
    a_releasePending = false;
    a_uses = 0;
    if (!a_a || !a_b) return 0;
    std::scoped_lock lk(g_pairMtx);
    for (int i = 0; i < kPairEntries; ++i) {
        const PairEntry& e = g_pair[i];
        if (!e.groups) continue;
        if ((e.a == a_a && e.b == a_b) || (e.a == a_b && e.b == a_a)) {
            a_entry = i;
            a_releasePending = e.pending;
            a_uses = e.uses;
            return e.groups;
        }
    }
    return 0;
}

}  // namespace standpoint_pair
