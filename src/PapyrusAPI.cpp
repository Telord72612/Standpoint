#include "PCH.h"
#include "SlotSolver.h"
#include "Threats.h"
#include "MoveApi.h"

namespace standpoint_interrupts {
    void AcquireHiggs();
    bool IsPlayerGrabbing(RE::Actor* a_actor);
    std::int32_t GetRunOncePackageTypeId(RE::Actor* a_actor);
}
namespace standpoint_avoid {
    void ProbeActorAgents(RE::Actor* a_actor);
}

// ---------------------------------------------------------------------------
// Standpoint Papyrus natives — v0.2.0 (S1 "park and face" support).
//
//   Standpoint.Log(msg)                      -> SKSE/Standpoint.log
//   Standpoint.GetEngineSettingFloat(name)   -> GMST / INI / INIPref lookup
//   Standpoint.SetEngineSettingFloat(name,v) -> runtime-only write (mitigation test)
//   Standpoint.GetCurrentProcedureIndex(npc) -> engine's live cursor into the
//        package procedure tree. For our TravelAndUseIdle clone: 1 = Travel
//        running, 2 = UseIdleMarker running — the engine's own "arrived"
//        signal (report 04 §E4), replacing any guessed stall detector.
//   Standpoint.GetCurrentPackageFormID(npc)  -> verify our override is live.
//
// INI-class names must be passed in the engine's registered form
// ("fAITurnSpeedScale:Pathfinding"); GMST-class names are bare.
// ---------------------------------------------------------------------------

namespace logger = SKSE::log;

namespace {

constexpr float kMissing = -999999.0f;

RE::Setting* FindSetting(const std::string& a_name)
{
    if (auto* gmst = RE::GameSettingCollection::GetSingleton()) {
        if (auto* s = gmst->GetSetting(a_name.c_str())) return s;
    }
    if (auto* ini = RE::INISettingCollection::GetSingleton()) {
        if (auto* s = ini->GetSetting(a_name)) return s;
    }
    if (auto* pref = RE::INIPrefSettingCollection::GetSingleton()) {
        if (auto* s = pref->GetSetting(a_name)) return s;
    }
    return nullptr;
}

void Log(RE::StaticFunctionTag*, std::string a_msg)
{
    logger::info("[psc] {}", a_msg);
}

float GetEngineSettingFloat(RE::StaticFunctionTag*, std::string a_name)
{
    if (auto* s = FindSetting(a_name)) return s->data.f;
    logger::warn("GetEngineSettingFloat: '{}' not found in any collection", a_name);
    return kMissing;
}

bool SetEngineSettingFloat(RE::StaticFunctionTag*, std::string a_name, float a_value)
{
    if (auto* s = FindSetting(a_name)) {
        const float old = s->data.f;
        s->data.f = a_value;
        logger::info("SetEngineSettingFloat: {} {} -> {} (runtime-only)", a_name, old, a_value);
        return true;
    }
    logger::warn("SetEngineSettingFloat: '{}' not found — nothing written", a_name);
    return false;
}

std::int32_t GetCurrentProcedureIndex(RE::StaticFunctionTag*, RE::Actor* a_actor)
{
    if (!a_actor) return -2;
    auto* proc = a_actor->GetActorRuntimeData().currentProcess;
    if (!proc) return -2;
    RE::BSSpinLockGuard lock(proc->currentPackage.packageLock);
    return proc->currentPackage.currentProcedureIndex;
}

std::int32_t GetCurrentPackageFormID(RE::StaticFunctionTag*, RE::Actor* a_actor)
{
    if (!a_actor) return 0;
    auto* proc = a_actor->GetActorRuntimeData().currentProcess;
    if (!proc) return 0;
    RE::BSSpinLockGuard lock(proc->currentPackage.packageLock);
    auto* pkg = proc->currentPackage.package;
    return pkg ? static_cast<std::int32_t>(pkg->GetFormID()) : 0;
}

std::vector<float> SolveSlot(RE::StaticFunctionTag*, RE::TESObjectREFR* a_target,
                             std::int32_t a_relation, RE::Actor* a_agent, float a_standoff)
{
    std::vector<float> out;
    const auto r = standpoint::Solve(a_target, static_cast<standpoint::Relation>(a_relation),
                                     a_agent, a_standoff);
    if (r.fail != standpoint::kOK) {
        logger::info("SolveSlot: fail code {}", r.fail);
        return out;  // empty = failure; code in the log
    }
    out.reserve(6);
    out.push_back(r.pos.x);
    out.push_back(r.pos.y);
    out.push_back(r.pos.z);
    out.push_back(r.headingRad * 57.2957795f);  // DEGREES at the Papyrus seam (report 04 C3)
    out.push_back(static_cast<float>(r.markerIndex));
    out.push_back(r.quality);
    return out;
}

std::string GetPluginVersion(RE::StaticFunctionTag*)
{
    return "1.0.0";  // "" when the DLL is absent = the WaterFinder soft-dep contract
}

bool IsPlayerGrabbing(RE::StaticFunctionTag*, RE::Actor* a_actor)
{
    return standpoint_interrupts::IsPlayerGrabbing(a_actor);
}

std::int32_t GetRunOncePackageTypeId(RE::StaticFunctionTag*, RE::Actor* a_actor)
{
    return standpoint_interrupts::GetRunOncePackageTypeId(a_actor);
}

// Tier 1: corridor threat scan. Returns 10 floats, or [] if the walker is gone:
//   [0] count (gating union)  [1] nearest dist (-1 none)  [2..4] danger centroid
//   [5] blob spread  [6] declared  [7] viaPlayer  [8] inCombat  [9] veryAggressive
// Indices 6..9 are the four raw signals: the driver logs them all so one field
// session tells us which predicate actually catches a camp before it aggros.
std::vector<float> ScanThreats(RE::StaticFunctionTag*, RE::Actor* a_walker,
                               float a_destX, float a_destY, float a_destZ,
                               float a_corridor, float a_lookAhead)
{
    std::vector<float> out;
    if (!a_walker) return out;
    const auto r = standpoint_threats::Scan(a_walker, RE::NiPoint3{ a_destX, a_destY, a_destZ },
                                            a_corridor, a_lookAhead);
    out.reserve(10);
    out.push_back(static_cast<float>(r.count));
    out.push_back(r.nearest);
    out.push_back(r.centroid.x);
    out.push_back(r.centroid.y);
    out.push_back(r.centroid.z);
    out.push_back(r.spread);
    out.push_back(static_cast<float>(r.declared));
    out.push_back(static_cast<float>(r.viaPlayer));
    out.push_back(static_cast<float>(r.inCombat));
    out.push_back(static_cast<float>(r.veryAggro));
    return out;
}

// Tier 3 Phase A part 2 — read-only reconnaissance on a live travelling actor.
void ProbeAvoidance(RE::StaticFunctionTag*, RE::Actor* a_actor)
{
    standpoint_avoid::ProbeActorAgents(a_actor);
}

std::int32_t GetFurnitureMarkerCount(RE::StaticFunctionTag*, RE::TESObjectREFR* a_target)
{
    return standpoint::GetMarkerCount(a_target);
}

std::vector<float> GetFurnitureMarkerInfo(RE::StaticFunctionTag*, RE::TESObjectREFR* a_target, std::int32_t a_index)
{
    std::vector<float> out;
    float v[7];
    if (!standpoint::GetMarkerInfo(a_target, a_index, v)) return out;
    out.assign(v, v + 7);
    out[3] *= 57.2957795f;  // heading -> degrees at the seam
    return out;
}

}  // namespace

namespace StandpointPapyrus {

bool Register(RE::BSScript::IVirtualMachine* a_vm)
{
    a_vm->RegisterFunction("Log",                      "Standpoint", Log);
    a_vm->RegisterFunction("GetEngineSettingFloat",    "Standpoint", GetEngineSettingFloat);
    a_vm->RegisterFunction("SetEngineSettingFloat",    "Standpoint", SetEngineSettingFloat);
    a_vm->RegisterFunction("GetCurrentProcedureIndex", "Standpoint", GetCurrentProcedureIndex);
    a_vm->RegisterFunction("GetCurrentPackageFormID",  "Standpoint", GetCurrentPackageFormID);
    a_vm->RegisterFunction("SolveSlot",                "Standpoint", SolveSlot);
    a_vm->RegisterFunction("GetPluginVersion",         "Standpoint", GetPluginVersion);
    a_vm->RegisterFunction("IsPlayerGrabbing",         "Standpoint", IsPlayerGrabbing);
    a_vm->RegisterFunction("GetRunOncePackageTypeId",  "Standpoint", GetRunOncePackageTypeId);
    a_vm->RegisterFunction("GetFurnitureMarkerCount",  "Standpoint", GetFurnitureMarkerCount);
    a_vm->RegisterFunction("GetFurnitureMarkerInfo",   "Standpoint", GetFurnitureMarkerInfo);
    a_vm->RegisterFunction("ScanThreats",              "Standpoint", ScanThreats);
    a_vm->RegisterFunction("ProbeAvoidance",           "Standpoint", ProbeAvoidance);
    logger::info("Papyrus natives registered (class Standpoint, 13 functions)");
    standpoint_move::RegisterNatives(a_vm);   // v0.8.0: the move API's mailbox natives
    return true;
}

}  // namespace StandpointPapyrus
