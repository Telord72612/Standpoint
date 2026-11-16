#pragma once
// ─────────────────────────────────────────────────────────────────────────────
//  AddrLib.h — 2.0.2: which Address Library ids Standpoint depends on exist on the RUNNING executable (SE / AE).
//
//  A user on Skyrim AE 1.7.104 stopped at start-up (Water Finder) with "REL/IDDB.cpp(31): Failed to find the id within the
//  address library: 13358" - the AE id of TES::GetWaterHeight is not in 1.7.104's Address Library, and CommonLib's id lookup
//  has no non-failing form (it reports and quits). Standpoint's planner calls the same function. So ONCE, at SKSEPluginLoad,
//  on SE / AE only, the running table is read through REL::Offset2ID (public, never fails; its v5 reader skips the zero
//  entries) and each id below is marked present / missing (the SE or the AE one of the pair, for the running runtime); then
//  the table is freed. ONE log line names what is missing.
//  VR: never checked - VR's own table is proven, every id counts as present (nothing changes there).
//  A call whose id may be missing asks Has() first and takes the fallback its site already had; an id with no fallback is
//  only named in the log line, so a reporter's log says which function it is. (DirectWalk's own ids - the motion check and
//  the speed normaliser - keep its SE / AE self-test.)
// ─────────────────────────────────────────────────────────────────────────────
#include <algorithm>
#include <atomic>
#include <cstdint>
#include <format>
#include <iterator>
#include <string>
#include <vector>

namespace standpoint_addrlib {

    enum Fn : int {
        kTESGetWaterHeight = 0,     // guarded (PathPlanner DepthAt): -> the cell's water height -> no water there
        kCellGetWaterHeight,        // guarded (PathPlanner DepthAt): -> no water there
        kTESGetSingleton,
        kTESGetCell,
        kCellGetbhkWorld,
        kPathingGetSingleton,
        kPathingGetPathingCell,
        kPathingFindClosestPointOnNavmesh,
        kPlayerGetSingleton,
        kProcessListsGetSingleton,
        kDataHandlerGetSingleton,
        kFormGetAllForms,
        kLockForRead,
        kUnlockForRead,
        kFindCollidableRef,
        kFindFurnitureMarkerNode,
        kCollisionObjectGetRigidBody,
        kRefrGetScale,
        kRefrGetLock,
        kOpenCloseGetOpenState,
        kRefrActivateRef,
        kRefrGetDisplayFullName,
        kRefrSetAngle,
        kActorIsMoving,
        kActorIsGhost,
        kActorIsHostileToActor,
        kActorGetSubmergedLevel,
        kActorIsInRagdollState,
        kActorIsMovementAnimationDriven,
        kActorEvaluatePackage,
        kActorSetHeading,
        kConsoleLogGetSingleton,
        kConsoleLogVPrint,
        kShowHUDMessage,
        kUIGetSingleton,
        kGameSettingsGetSingleton,
        kINISettingsGetSingleton,
        kINIPrefSettingsGetSingleton,
        kFirstConsoleCommand,
        kCount
    };

    struct Entry {
        const char*   name;
        std::uint64_t se;
        std::uint64_t ae;
    };

    // (the ids are CommonLibVR 8.3.0's RELOCATION_ID pairs of these functions)
    inline constexpr Entry kEntries[] = {
        { "TES::GetWaterHeight", 13212, 13358 },
        { "TESObjectCELL::GetWaterHeight", 18543, 19002 },
        { "TES::GetSingleton", 516923, 403450 },
        { "TES::GetCell", 13177, 13322 },
        { "TESObjectCELL::GetbhkWorld", 18536, 18995 },
        { "Pathing::GetSingleton", 514893, 401037 },
        { "Pathing::GetPathingCell", 29866, 30682 },
        { "Pathing::FindClosestPointOnNavmesh", 29832, 30648 },
        { "PlayerCharacter::GetSingleton", 517014, 403521 },
        { "ProcessLists::GetSingleton", 514167, 400315 },
        { "TESDataHandler::GetSingleton", 514141, 400269 },
        { "TESForm::GetAllForms", 514351, 400507 },
        { "BSReadWriteLock::LockForRead", 66976, 68233 },
        { "BSReadWriteLock::UnlockForRead", 66982, 68239 },
        { "TESHavokUtilities::FindCollidableRef", 25466, 26003 },
        { "BSFurnitureMarkerNode::FindBSFurnitureMarkerNode", 74889, 76647 },
        { "bhkCollisionObject::GetRigidBody", 12784, 20014 },
        { "TESObjectREFR::GetScale", 19238, 19664 },
        { "TESObjectREFR::GetLock", 19818, 20223 },
        { "BGSOpenCloseForm::GetOpenState", 14180, 14288 },
        { "TESObjectREFR::ActivateRef", 19369, 19796 },
        { "TESObjectREFR::GetDisplayFullName", 19354, 19781 },
        { "TESObjectREFR::SetAngle", 19359, 19786 },
        { "Actor::IsMoving", 36928, 37953 },
        { "Actor::IsGhost", 36286, 37275 },
        { "Actor::IsHostileToActor", 36537, 37537 },
        { "Actor::GetSubmergedLevel", 36452, 37448 },
        { "Actor::IsInRagdollState", 36492, 37491 },
        { "Actor::IsMovementAnimationDriven", 36487, 37486 },
        { "Actor::EvaluatePackage", 36407, 37401 },
        { "Actor::SetHeading", 36248, 37230 },
        { "ConsoleLog::GetSingleton", 515064, 401203 },
        { "ConsoleLog::VPrint", 50180, 51110 },
        { "SendHUDMessage::ShowHUDMessage", 52050, 52933 },
        { "UI::GetSingleton", 514178, 400327 },
        { "GameSettingCollection::GetSingleton", 514622, 400782 },
        { "INISettingCollection::GetSingleton", 524557, 411155 },
        { "INIPrefSettingCollection::GetSingleton", 523673, 410219 },
        { "SCRIPT_FUNCTION::GetFirstConsoleCommand", 501797, 365650 },
    };
    static_assert(std::size(kEntries) == kCount, "one entry per Fn");
    static_assert(kCount <= 64, "the missing mask is 64 bits");

    inline std::atomic<std::uint64_t> g_missing{ 0 };   // bit i = kEntries[i] is NOT in the running table (0 = all present)

    // true = the function may be called (VR, not checked yet, or its id is in the running table)
    inline bool Has(Fn a_fn) { return (g_missing.load(std::memory_order_relaxed) & (1ull << a_fn)) == 0; }

    // ONCE, at SKSEPluginLoad (before anything of ours can call the game). SE / AE only.
    inline void Check()
    {
        if (REL::Module::IsVR()) {
            SKSE::log::info("address library: VR - not checked ({} ids; VR's own table, every one counts as present)", static_cast<int>(kCount));
            return;
        }
        const bool                 ae = REL::Module::IsAE();
        std::vector<std::uint64_t> want;
        want.reserve(kCount);
        for (const auto& e : kEntries) want.push_back(ae ? e.ae : e.se);
        std::vector<std::uint64_t> sorted = want;
        std::sort(sorted.begin(), sorted.end());
        std::vector<std::uint64_t> found;
        {
            const REL::Offset2ID table;   // every (id, offset) of the running table - non-failing; freed at the end of this block
            for (const auto& m : table) {
                if (std::binary_search(sorted.begin(), sorted.end(), m.id)) found.push_back(m.id);
            }
        }
        std::sort(found.begin(), found.end());
        std::uint64_t missing = 0;
        std::string   names;
        for (int i = 0; i < kCount; ++i) {
            const std::uint64_t id = want[static_cast<std::size_t>(i)];
            if (id != 0 && std::binary_search(found.begin(), found.end(), id)) continue;
            missing |= 1ull << i;
            names += std::format("{}{} (id {})", names.empty() ? "" : ", ", kEntries[i].name, id);
        }
        g_missing.store(missing, std::memory_order_relaxed);
        SKSE::log::info("address library: {} ids checked ({}), missing: {}", static_cast<int>(kCount), ae ? "AE" : "SE",
                        names.empty() ? std::string("none - all present") : names);
    }

}  // namespace standpoint_addrlib
