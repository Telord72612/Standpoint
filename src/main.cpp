#include "PCH.h"
#include "MoveApi.h"
#include "PathPlanner.h"
#include "DirectWalk.h"
#include "WalkApi.h"
#include "SteerSession.h"

// ---------------------------------------------------------------------------
// Standpoint v0.1.0 — milestone S0: the engine-settings probe.
//
// Report 04 (Design Verification) found a class of engine settings that are
// NEITHER ini keys NOR GMST records — invisible to xEdit and to ini mining —
// reachable only through GameSettingCollection. fAvoidPlayerDistance lives
// there and may fight every close-range placement (the module's #1 use case).
// This milestone reads every constant the arrival design depends on and logs
// where each one actually lives (GMST / INI / INIPref / nowhere) and its live
// value in THIS install. Nothing else; nothing is written or hooked.
//
// Anchors with known values validate the probe itself:
//   fAIMarkerDestinationRadius    = 50   (GMST 0964D9)
//   fAIFurnitureDestinationRadius = 150  (GMST 050739)
//   fFurnitureMarkerAngleTolerance= 10   (GMST 035331)
// If those three read back wrong, trust nothing else in the log.
// ---------------------------------------------------------------------------

namespace logger = SKSE::log;

namespace StandpointPapyrus {
    bool Register(RE::BSScript::IVirtualMachine* vm);
}
namespace standpoint_api {
    void OnSKSEMessage(SKSE::MessagingInterface::Message* msg);
}
namespace standpoint_interrupts {
    void AcquireHiggs();
}
namespace standpoint_avoid {
    void ProbeSingletons();
}
namespace standpoint_test {
    void OnPostLoad();   // 1.6.0 FIX ROUND (review R1 NIT 17): an stpm heading marker that came back with a save is deleted
}

SKSEPluginInfo(
    .Version              = { 2, 0, 0 },
    .Name                 = "Standpoint",
    .Author               = "mad72",
    .StructCompatibility  = SKSE::StructCompatibility::Independent,
    .RuntimeCompatibility = SKSE::VersionIndependence::AddressLibrary
)

namespace {

struct ProbeEntry {
    const char* name;     // bare setting name
    const char* section;  // ini section if this is (also) an ini-class key, else nullptr
    const char* note;     // why we care / expected value if known
};

// clang-format off
constexpr ProbeEntry kProbes[] = {
    // --- anchors (known GMST values; validate the probe) ------------------
    { "fAIMarkerDestinationRadius",             nullptr,       "anchor: expect 50" },
    { "fAIFurnitureDestinationRadius",          nullptr,       "anchor: expect 150" },
    { "fFurnitureMarkerAngleTolerance",         nullptr,       "anchor: expect 10" },

    // --- E1: the player/target avoidance family (the #1 threat) ----------
    { "fAvoidPlayerDistance",                   nullptr,       "engine biases NPC paths AWAY from the player by this radius" },
    { "fMovementTargetAvoidRadius",             nullptr,       "avoid radius around a movement target" },
    { "fMovementTargetAvoidRadiusMult",         nullptr,       "" },
    { "fMovementNearTargetAvoidRadius",         nullptr,       "" },
    { "fMovementNearTargetAvoidCost",           nullptr,       "" },
    { "fAvoidanceDistanceRadiusMult",           "Pathfinding", "" },

    // --- E3: the engine's own moving-target re-path hysteresis ------------
    { "fAIMoveDistanceToRecalcTravelToActor",   nullptr,       "how far a target actor moves before re-path — replaces invented H_pos" },
    { "fAIMoveDistanceToRecalcTravelPath",      nullptr,       "" },
    { "fAIMoveDistanceToRecalcFollowPath",      nullptr,       "" },

    // --- E2: turn rate -----------------------------------------------------
    { "fAITurnSpeedScale",                      "Pathfinding", "how fast an NPC turns" },
    { "fAICombatTurnSpeedScale",                "Pathfinding", "" },
    { "fRotateTowardsPathThreshold",            "Pathfinding", "" },

    // --- the angle-tolerance family (only the furniture one has a GMST) ---
    { "fIdleMarkerAngleTolerance",              nullptr,       "arrival facing tolerance for IdleMarkers — our PRIMARY facing mechanism" },
    { "fHeadingMarkerAngleTolerance",           nullptr,       "" },
    { "fDefaultAngleTolerance",                 nullptr,       "" },
    { "fCombatAngleTolerance",                  nullptr,       "" },
    { "fTeleportNodeAngleTolerance",            "Pathfinding", "" },

    // --- activation reach (no GMST record anywhere in the load order) -----
    { "fAIActivateReach",                       nullptr,       "how close an NPC must be to activate" },
    { "fAIActivateHeight",                      nullptr,       "" },

    // --- report 01 [Pathfinding] values, read LIVE (were ~85% confidence) --
    { "bUseActorAvoidance",                     "Pathfinding", "expect 1" },
    { "bUseVelocityObstacles",                  "Pathfinding", "expect 1" },
    { "bSlowDownForActorAvoidance",             "Pathfinding", "expect 0 — the deadlock switch" },
    { "bStaticAvoidanceTriggerMovementBlocked", "Pathfinding", "expect 0 — the chair-wedge switch" },
    { "bUseMovementBlockedPackage",             "General",     "expect 0 — run-a-package-when-blocked" },
    { "fSlowDownMultiplier",                    "Pathfinding", "expect 0.5" },
    { "fMinNormalizedSpeedForSlowdown",         "Pathfinding", "expect 0.75" },
    { "fMaxTimeBlockedByActors",                "Pathfinding", "expect 1.0" },
    { "fMinAvoidanceRadius",                    "Pathfinding", "expect 256" },
    { "fMaxAvoidanceRadius",                    "Pathfinding", "expect 512" },
    { "fAvoidNodeCost",                         "Pathfinding", "expect 24" },
    { "fObstacleManagerMinWidth",               "Pathfinding", "expect 16" },
    { "fObstacleManagerMinHeight",              "Pathfinding", "expect 48 — why chairs wedge NPCs" },
    { "iPathRequestsAllowedPerFrame",           "Pathfinding", "expect 2" },

    // --- bump reaction (vanilla furniture-eviction family) -----------------
    { "fBumpReactionSmallDelayTime",            nullptr,       "" },
    { "fBumpReactionIdealMoveDist",             nullptr,       "" },
    { "fBumpReactionMinMoveDist",               nullptr,       "" },

    // --- misc distances the solver will care about -------------------------
    { "fAIFindBedChairsDistance",               nullptr,       "" },
    { "fAIAcquireObjectDistance",               nullptr,       "" },
};
// clang-format on

std::string FormatValue(const RE::Setting* s, const char* bareName)
{
    switch (bareName[0]) {
    case 'b': return s->data.b ? "1 (true)" : "0 (false)";
    case 'f': return fmt::format("{:g}", s->data.f);
    case 'i': return fmt::format("{}", s->data.i);
    case 'u': return fmt::format("{}", s->data.u);
    case 's': return s->data.s ? fmt::format("\"{}\"", s->data.s) : "(null)";
    default:  return fmt::format("raw=0x{:016X}", reinterpret_cast<std::uintptr_t>(s->data.s));
    }
}

void RunSettingsProbe()
{
    auto* gmst = RE::GameSettingCollection::GetSingleton();
    auto* ini  = RE::INISettingCollection::GetSingleton();
    auto* pref = RE::INIPrefSettingCollection::GetSingleton();

    logger::info("=== S0 SETTINGS PROBE ===================================================");
    logger::info("collections: GameSetting={} INI={} INIPref={}",
                 static_cast<void*>(gmst), static_cast<void*>(ini), static_cast<void*>(pref));

    int found = 0, missing = 0;
    for (const auto& p : kProbes) {
        const RE::Setting* s   = nullptr;
        const char*        src = nullptr;

        // 1) game-setting collection, bare name (the "third class" lives here)
        if (gmst) {
            if ((s = gmst->GetSetting(p.name))) src = "GMST";
        }
        // 2) ini collections, "name:Section" (how the engine registers ini keys)
        if (!s && p.section) {
            const std::string full = fmt::format("{}:{}", p.name, p.section);
            if (ini  && (s = ini->GetSetting(full)))  src = "INI";
            if (!s && pref && (s = pref->GetSetting(full))) src = "INIPref";
        }
        // 3) ini collections, bare name (fallback; some registrations differ)
        if (!s) {
            if (ini  && (s = ini->GetSetting(p.name)))  src = "INI(bare)";
            if (!s && pref && (s = pref->GetSetting(p.name))) src = "INIPref(bare)";
        }

        if (s) {
            ++found;
            logger::info("[{:>13}] {:<42} = {:<12} {}", src, p.name, FormatValue(s, p.name),
                         p.note[0] ? fmt::format("// {}", p.note) : "");
        } else {
            ++missing;
            logger::info("[    NOT FOUND] {:<42}   {}", p.name,
                         p.note[0] ? fmt::format("// {}", p.note) : "");
        }
    }
    logger::info("=== probe done: {} found, {} not found ==================================", found, missing);
}

void OnSKSEMessage(SKSE::MessagingInterface::Message* msg)
{
    if (msg->type == SKSE::MessagingInterface::kDataLoaded) {
        RunSettingsProbe();
        standpoint_interrupts::AcquireHiggs();
        if (REL::Module::IsVR()) {
            standpoint_avoid::ProbeSingletons();  // Tier 3 Phase A1: module-only, no game objects (v1.1.0: VR only)
        }
        standpoint_test::RegisterConsole();   // v0.8.0: the `stpt` console test of the move API (one command since v0.8.2)
        standpoint_walk::OnDataLoaded();      // v1.5.0: Water Finder's C++ interface, asked once (the wade-spot walk)
    } else if (msg->type == SKSE::MessagingInterface::kSaveGame) {
        // v1.4.0: every walker the walk driver holds is handed back before the save is written (re-taken next frame)
        standpoint_walk::OnSaveGame();
    } else if (msg->type == SKSE::MessagingInterface::kPreLoadGame || msg->type == SKSE::MessagingInterface::kNewGame) {
        // v0.8.0: the driver rebuilds its sessions on every load, so a request still open can never be answered by it
        standpoint_test::OnGameLoad();        // (v1.4.0: also forgets the stpt walk program's handle; 1.6.0: and the stpm run)
        standpoint_pair::ClearAll();         // v0.9.3: the world is being rebuilt - the whole pair-rule pool off at once
        standpoint_move::FailAll('X', StandpointAPI::kDetailGameLoaded);
        standpoint_steer::OnGameLoad();       // 1.6.0: every steered session is forgotten (the driver's Setup makes its slots new)
        standpoint_walk::OnGameLoad();        // v1.4.0: every walk answers X 12 (C++), the driver forgets all, touches nothing
        standpoint_path::ClearAll();          // v1.3.0: every route plan of the old game is dropped
    } else if (msg->type == SKSE::MessagingInterface::kPostLoadGame) {
        standpoint_test::OnPostLoad();        // 1.6.0 FIX ROUND: a test marker a save brought back is deleted (stpm only)
    }
}

void InitLogging()
{
    auto path = SKSE::log::log_directory();
    if (!path) return;
    *path /= "Standpoint.log";

    auto sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(path->string(), true);
    auto log  = std::make_shared<spdlog::logger>("Standpoint", sink);
    log->set_level(spdlog::level::info);
    log->flush_on(spdlog::level::info);
    spdlog::set_default_logger(std::move(log));
    spdlog::set_pattern("[%H:%M:%S.%e] [%l] %v");
}

}  // namespace

SKSEPluginLoad(const SKSE::LoadInterface* skse)
{
    InitLogging();
    SKSE::Init(skse);
    // v0.7.1 (2026-09-30): the danger scan counts only actors in the walker's own loaded cell / worldspace (Threats.cpp).
    // v0.7.2 (2026-10-01, VRCD h79 §13): an actor's side whose point had to slide on the navmesh is BLOCKED - the next side is
    // used (SlotSolver.cpp); the driver (Standpoint_PlayerAlias.psc 0.7.2) turns a walker who stops short and cannot get closer.
    // v0.7.3 (2026-10-01, VRCD h79 §15): no DLL code change - the version tells consumers that StandpointAPI.SetFaceTarget exists
    // (the driver turns a walker who stops short toward her face target).
    // v0.7.4 (2026-10-03, VRCD h79 §17): driver - the arrival block re-checks its session after every wait; new StandpointAPI.GetArrivalState.
    // v0.8.0 (2026-10-08): THE MOVE API - IStandpoint2::RequestMove (StandpointAPI.h): a 10-slot request line in, a 10-slot result
    // line back. MoveApi.cpp = the mailbox to the Papyrus driver; SlotSolverClock.inc = the clock solve (any hour around the
    // target, centre-to-centre distance, model-less markers); TestCmd.cpp = the `stpw` / `stpt` console test.
    // v0.8.1 (2026-10-08, after the first VR run): the clock solve treats furniture as an object (no sit / sleep markers);
    // driver - a walker ON her point but facing wrong after the waits is turned (it was done only for a SHORT arrival).
    // v0.8.2 (2026-10-09): DLL only - the console test is ONE command (`stpt`; `stpw` is gone). The second command had taken
    // the console slot the VRCD AddOn's `vrcw` needs (TestCmd.cpp). Scripts, ESP and the API are unchanged from 0.8.1.
    // v0.9.0 (2026-10-09, report 05 stages 0-1 + the lever-B experiment): slot 8 of the request line = CLOSE CONTACT (no floor
    // for an actor target, a stall near her = arrival, the player near does not cancel); `stpt` on an ACTOR runs the
    // close-contact measuring program (controller capsules + approach trace in the log); the pair "ignore each other" rule on
    // HIGGS's collision-filter hook exists but only that test arms it.
    // v0.9.1 (2026-10-09, after the first close-contact run - report 05 §10): a close session no longer starts SeverActions'
    // stuck recovery (its 500 u "leapfrog" threw the walker through the wall); a close walker who is not near her target has
    // not arrived (O 15); the close-contact test approaches every side from 150 cm and measures the body shape too.
    // v0.9.2 (2026-10-09, the user's rulings on report 05 §10.5): NO walk starts SeverActions' stuck recovery ("teleportation
    // is never good"); "stopped far from the point" is not an arrival for ANY session (Failed 6 / O 15); the close-contact
    // test gets pass C (35 and 25 cm with the pair rule) and switches the rule off only once the two are apart.
    // v0.9.3 (2026-10-09, after the second close-contact run - report 05 §12, the user: "Sure, let's try it"): the pair rule
    // is part of CLOSE mode (four slots, armed / released by the driver per session: natives PairArm / PairRelease); the
    // driver glides a close walker over the last stretch the engine's walk leaves open (result C, detail 16); a request
    // whose `where` is `who` herself = hold your place; the stall clock starts with the first poll that sees OUR package.
    // v0.9.4 (2026-10-09, an independent review of 0.9.3 before any VR run): the pair rule is a POOL of 8 pairs with handles
    // and use counts (0.9.3's per-session-slot pairs switched an overlapping old pair off); the glide keeps the package ON
    // first and only then tries without it; the driver's arrival handling can no longer run twice for one session; the
    // test holds the walker between its steps.
    // v0.9.5 (2026-10-09, the second review, still before any VR run): driver - the close-mode stall rule waits 8 s for a
    // walker who has not moved yet in her session (3 s once she has: a retask from the held pose is not a stall); the glide
    // never crosses the target; a release in flight is "not the same session". DLL - the pair pool logs outside its mutex.
    // v0.9.6 (2026-10-10, after the third close-contact run - report 05 §14: the glide with the package on worked 6 of 6,
    // 30 cm on all four sides): DRIVER ONLY - a walker who is HELD is let go (pose ended, 1 s) before she is retasked (held,
    // she did not walk at all); the glide is only for the last straight stretch (within 30 degrees, no swing around the
    // target) and runs at 35 u/s instead of 60. No change in the DLL's logic.
    // v0.9.7 (2026-10-10, the user's ruling after the third run: "yes, have standpoint do the step back and around itself"
    // - report 05 §15): DRIVER - a close request for ANOTHER side of an NPC the walker already stands near becomes a route
    // inside the one session: she steps back, goes around at a distance, then walks in; the let-go of a held walker also
    // waits for the engine to drop our package. TEST - the close-contact program asks for the next side directly.
    // No change in the DLL's logic.
    // v0.9.8 (2026-10-10, after the fourth close-contact run - report 05 §16; the user: "moving a NPC while they are in the
    // wait pose make her slide on the floor. Let's try not to do that again" and "yes build both"): DRIVER - there is NO
    // GLIDE any more: a close walker always walks to her point, from a stop 58 u or more away (nearer, the engine poses
    // her where she stands instead of walking); a stop on the way is a real place and the leg changes the moment she
    // is there. TEST - the 20 cm step is gone (not reachable on foot). No change in the DLL's logic.
    // v0.9.9 (2026-10-10, after the fifth close-contact run - report 05 §18: 13 of 13 on the point, on foot; the user:
    // "if the path is blocked, making the NPCs actually react right away is probably the best thing to do. And if it
    // cannot find any other path to make it there, then report it as such"): DRIVER - the engine's own "movement blocked"
    // (run-once package type 36) with no movement for 1 s ends a leg at once; a close request to an NPC that cannot be
    // walked answers O, detail 17 (kDetailNoWay), Standpoint_Failed 7, and releases her - no longer "completed, stopped
    // short" far from the point. No change in the DLL's logic.
    // v1.0.0 (2026-10-10): THE FIRST PUBLIC RELEASE ("V1"). The code of 0.9.9 with the version number changed -
    // nothing else. Published at https://github.com/Telord72612/Standpoint, public domain.
    // v1.1.0 (2026-10-24; built as 1.0.1): built on CommonLibVR 8.3.0 - Skyrim AE 1.7.x is recognised as AE (it stopped with "failed to
    // open address library file" before: 4.14.0 took any minor version but 6 for SE); the old avoid-node probe runs on VR
    // only. No change in what Standpoint does.
    // v1.2.0 (2026-10-30, the BiSR bridge's letter - report 11): a ring point is never on another floor level (F1); a
    // re-sent walker who stops short is answered in seconds, not after 90 s (F2); RequestMove = the move request line
    // from Papyrus, close mode included (F3); the arrival waits end as soon as she stands still (F4); a paused game is
    // not counted; a walker moving past the player is not "blocked" by him.
    // v1.3.0 (2026-10-31, the user's waypoint plan): a leg that does not start is kicked (2 s evaluate, 4 s sent again)
    // before it can count as failed; a leg blocked with 60 u or more to go gets a PLANNED WAY ROUND (PathPlanner.cpp:
    // navmesh floor + Havok rays, sliced per frame), walked point by point as DETOUR legs.
    // v1.4.0 (2026-11-02, the user: "we can use HIGGS/PLANCK way to travel, exactly like if someone was pulling a chain on
    // that NPC's hips"): THE WALK OFF THE NAVMESH - StandpointAPI.WalkTo / IStandpoint3::RequestWalk. The route planner's
    // Ex mode (PathPlanner.cpp: navmesh + bare floor + shallow water) plans the route; the chain-pull driver
    // (DirectWalk.cpp: planner direct control from the VR motion-check hook) walks her along all of it with her own legs;
    // WalkApi.cpp gives ONE answer. VR only (SE / AE answer B 19). A save hands driven walkers back for the save.
    // v1.4.1 (2026-11-03, after the user's first VR run of 1.4.0): DLL only. A walk that STARTS in water deeper than the cap
    // (pulled into a bath) is planned out of it instead of refused - her start region up to her own depth + 8 u, never at
    // the swim level (PathPlanner.cpp); a walker who can strafe turns to her final facing over the last 48 u and gets ONE
    // short re-approach when still more than 16 u off (DirectWalk.cpp: the turn in place at the point had drifted her
    // 10 u); the console test gets a second command, the FOLLOW-FRONT program (she keeps 3 m in front of the player until
    // stpt - no sneaking; TestCmd.cpp + WalkApi.cpp). The API and the scripts are unchanged.
    // v1.5.0 (2026-11-05, the user: "All of 1.5.0 now"; report "1.5.0 Plan" D): THE MIX - the planner follows the navmesh
    // corridor where there is one and checks every leg with rays, several floors per map square, the body rules, standing
    // actors kept clear of, unlocked ordinary doors opened by the walker (PathPlanner.cpp); a new route after every stall or
    // push in a new place (WalkApi.cpp); THE WALK INTO WATER - StandpointAPI.WalkToWater / IStandpoint4::RequestWalkToWater:
    // Water Finder's seed, the planner's wade-spot search, she wades in and faces the water; SE / AE: the walk driver's site
    // is found at run time by a read-only self-test, switched on by Standpoint.ini [DirectWalk] SEAE = 1 (DirectWalk.cpp).
    // v1.5.1 (2026-11-05, after the user's first VR run of 1.5.0: "A bit slow, which we can fix by adding some speed according to
    // distance" / "StandPoint is a travel mod for up to 200m"; report "1.5.1 Plan" D): a standing NPC's idle is broken
    // (IdleForceDefaultState) before she is driven; SPEED BY DISTANCE - within 20 m she walks, beyond it the direct walk jogs and
    // walks the last 10 m; THE FAR PHASE beyond 30 m (VR): Standpoint's own travel session runs her (the slot package at RUN)
    // until 30 m, then the direct walk takes her over moving; up to 200 m (O 1 beyond); B 22 names the holder (DirectWalk.cpp,
    // WalkApi.cpp, Standpoint_PlayerAlias.psc); the planner climbs her real step, trusts the navmesh's slope on terrain and keeps
    // classified cells for 60 s (PathPlanner.cpp).
    // v1.6.0 (2026-11-06, after the user's first VR run of 1.5.1 - report "1.6.0 Plan" D, the spec): RELATIVE destinations "like the
    // water walk" - THE STEERED SESSION (SteerSession.cpp + RelativeSolve.cpp + the driver): GoTo / RequestMove (and GoToPoint, the
    // same switch) keep their whole Papyrus contract, but the DLL solves the spot by physics, plans ONE route and walks her with
    // Standpoint's own steering; the driver pins her after the arrival (Standpoint.ini [Session] Steered: VR 1, SE / AE 0 - the
    // rollback switch). BUG 1: the session markers are made at the walker and placed by ONE cross-cell MoveTo (they stayed in the
    // cell they were made in); BUG 2: the markers are kept across loads, the slot packages' orphan overrides removed at every load,
    // the old litter COUNTED (MarkerSweep.cpp; deleted only with [Cleanup] SweepMarkers = 1). OUR OWN LONG PATH (NavGraph.cpp: the
    // loaded navmesh's triangles, verified chunks while she walks - the engine far phase is gone; O 26 outside the loaded area),
    // the speed bands (run beyond 15 m, jog to 3 m, walk the last 3 m), faster planning, stairs both ways, deeper wading (shin -> knee
    // -> thigh in one route) - PathPlanner.cpp, WalkApi.cpp, DirectWalk.cpp. The `stpm` test command (TestCmd.cpp).
    // v1.6.1 (2026-11-06, after the first 1.6.0 VR run - report "1.6.0 Plan" R3 / R4): the fix round - a GoTo on furniture falls back
    // to its box sides, a POINT off the navmesh with no route answers O 1 (no engine leg), an engine leg ends when it is blocked, close
    // arrivals land on the spot, stpm's event names and open-floor spots (SteerSession.cpp, RelativeSolve.cpp, TestCmd.cpp, the
    // driver); the planner's and the walk's own fixes (PathPlanner.cpp, NavGraph.cpp, WalkApi.cpp, DirectWalk.cpp).
    // v2.0.0 (2026-11-06, the user: Standpoint "V2.0" is complete): a version-only release of 1.6.1 - no behaviour change.
    logger::info("Standpoint v2.0.0 loaded (C++ interface revision 4: the walk into water - shin, knee, thigh in one route; revision 3: "
                 "the walk off the navmesh - our own path over the loaded navmesh, run / jog / walk by the route left, O 26 outside the "
                 "loaded area; the move API: revision 2 - VR: the STEERED SESSIONS (the spot solved by physics, one route, our own "
                 "steering, pinned at the arrival); danger=DROP - loaded actors of the walker's own cell only)");
    standpoint_direct::Install();             // v1.4.0: the walk driver's hooks (v1.5.0: the self-test on every runtime; SE / AE
                                              // install only with Standpoint.ini [DirectWalk] SEAE = 1 and a READY self-test)
    standpoint_steer::Init();                 // 1.6.0: Standpoint.ini [Session] Steered (VR 1, SE / AE 0), read once, logged

    if (auto* msg = SKSE::GetMessagingInterface()) {
        msg->RegisterListener(OnSKSEMessage);
        msg->RegisterListener(nullptr, standpoint_api::OnSKSEMessage);  // all senders: the 'STPT' handshake
    } else {
        logger::error("messaging interface unavailable — probe will not run");
    }
    if (auto* papyrus = SKSE::GetPapyrusInterface()) {
        papyrus->Register(StandpointPapyrus::Register);
    } else {
        logger::error("papyrus interface unavailable — natives will not exist");
    }
    return true;
}
