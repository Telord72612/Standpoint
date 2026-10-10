// ─────────────────────────────────────────────────────────────────────────────
//  MoveApi.cpp — v0.8.0: the mailbox behind IStandpoint2::RequestMove (see MoveApi.h for the flow).
//  v0.9.3: + the two pair-rule natives (PairArm / PairRelease); a request whose `where` is `who` herself passes
//  Request() like any other reference - the driver reads it as "hold your place".
//  v0.9.4: the pair natives work with a handle (Int PairArm(a, b) / PairRelease(handle)), not a session slot.
// ─────────────────────────────────────────────────────────────────────────────
#include "PCH.h"
#include "MoveApi.h"
#include "SlotSolver.h"

namespace logger = SKSE::log;

namespace standpoint_move {
namespace {

    constexpr float kCmPerUnit = 1.428f;   // 1 game unit = 1.428 cm (70 u = 1 m)

    struct Pending {
        StandpointAPI::MoveRequest  req{};
        StandpointAPI::MoveCallback cb{ nullptr };
        void*                       user{ nullptr };
    };

    std::mutex                                g_mtx;      // the natives run on Papyrus VM threads
    std::unordered_map<std::int32_t, Pending> g_open;
    std::int32_t                              g_nextId = 1;

    bool Peek(std::int32_t a_id, Pending& a_out)
    {
        std::scoped_lock lk(g_mtx);
        const auto it = g_open.find(a_id);
        if (it == g_open.end()) return false;
        a_out = it->second;
        return true;
    }

    // Close the request: measure the result line and call the asking plugin back, on the main thread, once.
    void Deliver(std::int32_t a_id, char a_letter, std::int32_t a_detail, bool a_measure = true)
    {
        Pending p;
        {
            std::scoped_lock lk(g_mtx);
            const auto it = g_open.find(a_id);
            if (it == g_open.end()) return;      // answered already (the driver's catch-all after a named ending)
            p = it->second;
            g_open.erase(it);
        }
        auto* task = SKSE::GetTaskInterface();
        if (!task) return;
        task->AddTask([a_id, a_letter, a_detail, a_measure, p]() {
            StandpointAPI::MoveResult res{};
            res.who     = p.req.who;
            res.where   = p.req.where;
            res.outcome = a_letter;
            res.detail  = static_cast<float>(a_detail);
            auto* who   = a_measure && p.req.who ? RE::TESForm::LookupByID<RE::Actor>(p.req.who) : nullptr;
            auto* where = a_measure && p.req.where ? RE::TESForm::LookupByID<RE::TESObjectREFR>(p.req.where) : nullptr;
            if (a_measure && who && where) {
                float d = 0.f, fh = 0.f, oh = 0.f;
                standpoint::MeasureClock(where, who, d, fh, oh);
                res.distanceCm  = d * kCmPerUnit;
                res.facing      = fh;
                res.orientation = oh;
            }
            logger::info("MOVE #{} RESULT who={:08X} where={:08X} dist={:.0f}cm facing={:.1f}h orient={:.1f}h outcome={} detail={}",
                         a_id, res.who, res.where, res.distanceCm, res.facing, res.orientation, a_letter, a_detail);
            if (p.cb) p.cb(static_cast<std::uint32_t>(a_id), res, p.user);
        });
    }

    // ---- natives (class Standpoint) - called by the driver only -------------------------------------------

    RE::Actor* ApiReqWho(RE::StaticFunctionTag*, std::int32_t a_id)
    {
        Pending p;
        if (!Peek(a_id, p) || !p.req.who) return nullptr;
        return RE::TESForm::LookupByID<RE::Actor>(p.req.who);
    }

    RE::TESObjectREFR* ApiReqWhere(RE::StaticFunctionTag*, std::int32_t a_id)
    {
        Pending p;
        if (!Peek(a_id, p) || !p.req.where) return nullptr;
        return RE::TESForm::LookupByID<RE::TESObjectREFR>(p.req.where);
    }

    // [distanceCm, facing, orientation, pose, holdSeconds, isRelease(0/1), close(0/1)]; [] = no such request
    // (v0.9.0: the 7th value; an older driver reads only the first six)
    std::vector<float> ApiReqValues(RE::StaticFunctionTag*, std::int32_t a_id)
    {
        std::vector<float> out;
        Pending p;
        if (!Peek(a_id, p)) return out;
        out = { p.req.distanceCm, p.req.facing, p.req.orientation, p.req.pose, p.req.holdSeconds,
                p.req.where == 0 ? 1.f : 0.f, p.req.close };
        return out;
    }

    void ApiPostResult(RE::StaticFunctionTag*, std::int32_t a_id, std::string a_letter, std::int32_t a_detail)
    {
        Deliver(a_id, a_letter.empty() ? 'X' : a_letter[0], a_detail);
    }

    // [x, y, z, headingDEGREES, orientOffsetDEGREES, gotHour, quality]; [] = no standable point (reason in the log)
    // a_close (v0.9.0): 1 = close contact - an ACTOR target has no distance floor
    std::vector<float> SolveClock(RE::StaticFunctionTag*, RE::TESObjectREFR* a_target, RE::Actor* a_agent,
                                  float a_distCm, float a_facingHours, float a_orientHours, std::int32_t a_close)
    {
        std::vector<float> out;
        const auto c = standpoint::SolveClock(a_target, a_agent, a_distCm / kCmPerUnit, a_facingHours, a_orientHours,
                                              a_close != 0);
        if (c.slot.fail != standpoint::kOK) return out;
        out = { c.slot.pos.x, c.slot.pos.y, c.slot.pos.z, c.slot.headingRad * 57.2957795f,
                c.orientOffsetRad * 57.2957795f, c.gotHours, c.slot.quality };
        return out;
    }

    // The pair "ignore each other" rule is part of close mode (Interrupts.cpp, standpoint_pair). The driver arms it
    // when a close session with an NPC target starts and releases it when that session ends.
    // v0.9.4: a HANDLE instead of the driver's session slot (the pool of 8 pairs counts its users): PairArm returns a
    // handle >= 1, or 0 = not armed; PairRelease takes that handle.
    // These run on Papyrus VM threads. The pair module's one read of game objects (the two controllers' filter words
    // and positions) follows SolveClock above, which reads the same actors here; it happens outside the module's lock.
    std::int32_t PairArm(RE::StaticFunctionTag*, RE::Actor* a_a, RE::Actor* a_b)
    {
        if (!a_a || !a_b || !a_a->Is3DLoaded() || !a_b->Is3DLoaded()) return 0;
        return standpoint_pair::Arm(a_a, a_b);
    }

    void PairRelease(RE::StaticFunctionTag*, std::int32_t a_handle)
    {
        standpoint_pair::Release(a_handle);
    }

}  // namespace

std::uint32_t Request(const StandpointAPI::MoveRequest& a_req, StandpointAPI::MoveCallback a_cb, void* a_user)
{
    if (!a_cb) return 0;
    std::int32_t id;
    {
        std::scoped_lock lk(g_mtx);
        id = g_nextId++;
        if (g_nextId > 0x00FFFFFF) g_nextId = 1;     // stays exact as a Papyrus Int / Float
        g_open[id] = Pending{ a_req, a_cb, a_user };
    }
    logger::info("MOVE #{} REQUEST who={:08X} where={:08X} dist={:.0f}cm facing={:.1f}h orient={:.1f}h pose={:.0f} hold={:.1f}s close={:.0f}{}",
                 id, a_req.who, a_req.where, a_req.distanceCm, a_req.facing, a_req.orientation, a_req.pose,
                 a_req.holdSeconds, a_req.close, a_req.where == 0 ? " (RELEASE)" : "");

    auto* who = a_req.who ? RE::TESForm::LookupByID<RE::Actor>(a_req.who) : nullptr;
    if (!who || who->IsPlayerRef() || !who->Is3DLoaded()) {
        logger::warn("MOVE #{}: 'who' is not a loaded NPC", id);
        Deliver(id, 'O', StandpointAPI::kDetailBadRequest);
        return static_cast<std::uint32_t>(id);
    }
    if (a_req.where && !RE::TESForm::LookupByID<RE::TESObjectREFR>(a_req.where)) {
        logger::warn("MOVE #{}: 'where' is not a reference", id);
        Deliver(id, 'O', StandpointAPI::kDetailBadRequest);
        return static_cast<std::uint32_t>(id);
    }

    // wake the driver: StandpointAPI.ApiWake(id) -> Standpoint_PlayerAlias.ApiRunRequest(id)
    auto* vm = RE::BSScript::Internal::VirtualMachine::GetSingleton();
    bool sent = false;
    if (vm) {
        auto* args = RE::MakeFunctionArguments(static_cast<std::int32_t>(id));
        RE::BSTSmartPointer<RE::BSScript::IStackCallbackFunctor> none;
        sent = vm->DispatchStaticCall("StandpointAPI", "ApiWake", args, none);
        delete args;
    }
    if (!sent) {
        logger::warn("MOVE #{}: StandpointAPI.ApiWake could not be dispatched - Standpoint's scripts are not running", id);
        Deliver(id, 'B', StandpointAPI::kDetailNotReady);
    }
    return static_cast<std::uint32_t>(id);
}

void FailAll(char a_letter, std::int32_t a_detail)
{
    std::vector<std::int32_t> ids;
    {
        std::scoped_lock lk(g_mtx);
        for (const auto& [id, p] : g_open) ids.push_back(id);
    }
    // no measuring: the world is being torn down for the load
    for (const auto id : ids) Deliver(id, a_letter, a_detail, false);
}

bool RegisterNatives(RE::BSScript::IVirtualMachine* a_vm)
{
    a_vm->RegisterFunction("ApiReqWho",     "Standpoint", ApiReqWho);
    a_vm->RegisterFunction("ApiReqWhere",   "Standpoint", ApiReqWhere);
    a_vm->RegisterFunction("ApiReqValues",  "Standpoint", ApiReqValues);
    a_vm->RegisterFunction("ApiPostResult", "Standpoint", ApiPostResult);
    a_vm->RegisterFunction("SolveClock",    "Standpoint", SolveClock);
    a_vm->RegisterFunction("PairArm",       "Standpoint", PairArm);        // v0.9.3; a handle since v0.9.4
    a_vm->RegisterFunction("PairRelease",   "Standpoint", PairRelease);    // v0.9.3; a handle since v0.9.4
    logger::info("move API natives registered (class Standpoint, 7 functions)");
    return true;
}

}  // namespace standpoint_move
