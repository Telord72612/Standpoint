#pragma once
// ─────────────────────────────────────────────────────────────────────────────
//  SteerSession.h — 1.6.0: THE STEERED SESSION (report "1.6.0 Plan" S1 §2, §3; spec D §1 items 4 + 5).
//
//  The Papyrus driver (Standpoint_PlayerAlias.psc) keeps the SLOT and the whole session contract (IsBusy, GetArrivalState,
//  the events, ApiReport, Release, hold, poses, the pair rule, leash / interrupts / danger); this module is the DLL half:
//  it solves the spot by physics (RelativeSolve.cpp), plans ONE route and walks her there with the steered walk (WalkApi's
//  RequestSteer), watches a moving target, and keeps a small state line the driver polls (SteerState).
//
//  Threads: the natives only record / read under a mutex (VM threads); everything that touches the game runs in Frame
//  (the main thread, WalkApi's frame step) and in the walk's notices (main thread tasks).
// ─────────────────────────────────────────────────────────────────────────────

namespace standpoint_steer {

    // WalkApi's frame step (main thread, pause excluded): the steered sessions' work for this frame (solve slices, plans,
    // the target watch, the verdicts). Cheap when nothing runs.
    void Frame(float dt);
    // any thread: a steered session still needs Frame (WalkApi keeps its pump running while this is true)
    bool Live();

    // SKSEPlugin_Load: Standpoint.ini [Session] Steered is read once (VR default 1, SE / AE default 0) and logged.
    void Init();
    // any thread: the driver's sessions are walked steered (the walk is available here AND [Session] Steered = 1)
    bool Enabled();
    // kPreLoadGame / kNewGame: every steered session is forgotten (the driver's Setup makes its slots new; its walks answer X 12
    // to nobody). Touches nothing.
    void OnGameLoad();
    // class Standpoint: SteerSupported, SteerCheck, SteerStart, SteerState, SteerHandOff, SteerStop
    bool RegisterNatives(RE::BSScript::IVirtualMachine* a_vm);

}  // namespace standpoint_steer
