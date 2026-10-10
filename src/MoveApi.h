#pragma once

#include "StandpointAPI.h"

// ---------------------------------------------------------------------------
// v0.8.0 - the move API's mailbox (contract: StandpointAPI.h, IStandpoint2).
//
// The DLL cannot walk anyone by itself: the travel package and the marker aliases belong to the quest, so the walking
// lives in the Papyrus driver (Standpoint_PlayerAlias.psc). This file is the mailbox between a C++ plugin and that
// driver:
//   Request()      main thread - stores the line, wakes the driver (VM DispatchStaticCall StandpointAPI.ApiWake(id))
//   the natives    the driver reads the line (ApiReqWho / ApiReqWhere / ApiReqValues), solves it (SolveClock) and
//                  walks her with its own session engine
//   ApiPostResult  the driver names the ending (letter + detail); the DLL measures the result line on the main thread
//                  and calls the asking plugin back, once
// ---------------------------------------------------------------------------
namespace standpoint_move {

    std::uint32_t Request(const StandpointAPI::MoveRequest& a_req, StandpointAPI::MoveCallback a_cb, void* a_user);

    // Every open request answers with this ending (a save is being loaded: the driver's sessions are gone).
    void FailAll(char a_letter, std::int32_t a_detail);

    bool RegisterNatives(RE::BSScript::IVirtualMachine* a_vm);

}  // namespace standpoint_move

namespace standpoint_pair {
    // "These two character controllers ignore each other" (Interrupts.cpp, on HIGGS's collision-filter hook).
    // Part of CLOSE mode since v0.9.3: the driver arms it when a close session with an NPC target starts and releases
    // it when that session ends (the natives Standpoint.PairArm / PairRelease, MoveApi.cpp).
    // v0.9.4: a POOL of 8 pairs with handles and use counts, not tied to the driver's session slots. The filter reads
    // one atomic word per entry and takes no lock; the bookkeeping is behind a mutex. Every function here may be
    // called from any thread.

    // A handle >= 1, or 0 = not armed (HIGGS absent, a controller without a body / group, both in one group, or the
    // pool is full with nothing waiting for its release).
    // A pair that is in the pool already (armed, or its release pending) keeps its entry: the pending release is
    // cancelled and the use is counted - the rule never drops. A new pair takes a free entry; if none is free the
    // oldest release-pending entry is switched off at once and reused.
    std::int32_t Arm(RE::Actor* a_a, RE::Actor* a_b);

    // One use less. At 0 the pair is released "when apart": switched off once the two are 90 u or more apart (flat),
    // or one of them is gone / has no loaded 3D; looked at once a second (main thread), the first time after 1 s.
    // A handle that names no live pair is ignored.
    void Release(std::int32_t a_handle);

    void ClearAll();    // a save is being loaded: the whole pool off at once, no waiting

    void Stats(std::uint32_t& a_hits, std::uint32_t& a_ccSeen);     // the two counters (all pairs together)

    // The packed groups the rule is armed with for these two actors (0 = not armed for them); which entry, whether
    // its release is already ordered, and its use count. For the test's reports.
    std::uint32_t ArmedFor(RE::FormID a_a, RE::FormID a_b, int& a_entry, bool& a_releasePending, int& a_uses);
}

namespace standpoint_test {
    void RegisterConsole();   // kDataLoaded: the `stpt` console test
    void OnGameLoad();        // a save is being loaded: the running test is dropped
}
