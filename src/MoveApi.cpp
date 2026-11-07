// ─────────────────────────────────────────────────────────────────────────────
//  MoveApi.cpp — v0.8.0: the mailbox behind IStandpoint2::RequestMove (see MoveApi.h for the flow).
//  v0.9.3: + the two pair-rule natives (PairArm / PairRelease); a request whose `where` is `who` herself passes
//  Request() like any other reference - the driver reads it as "hold your place".
//  v0.9.4: the pair natives work with a handle (Int PairArm(a, b) / PairRelease(handle)), not a session slot.
// ─────────────────────────────────────────────────────────────────────────────
#include "PCH.h"
#include "MoveApi.h"
#include "SlotSolver.h"
#include "WalkApi.h"

namespace logger = SKSE::log;

namespace standpoint_move {
namespace {

    constexpr float kCmPerUnit = 1.428f;   // 1 game unit = 1.428 cm (70 u = 1 m)

    struct Pending {
        StandpointAPI::MoveRequest  req{};
        StandpointAPI::MoveCallback cb{ nullptr };
        void*                       user{ nullptr };
    };

    // v1.2.0: the ids start at a different number in every game run (they started at 1 every run), so a handle kept in a
    // save - or by a script stack that a load resumes - almost never names a live request after a restart.
    std::int32_t SeedId()
    {
        const auto t = static_cast<std::uint64_t>(std::chrono::system_clock::now().time_since_epoch().count());
        return 1 + static_cast<std::int32_t>(t % 0x00FFF000u);
    }

    std::mutex                                g_mtx;      // the natives run on Papyrus VM threads
    std::unordered_map<std::int32_t, Pending> g_open;
    std::int32_t                              g_nextId = SeedId();

    // v1.2.0: the Papyrus door (StandpointAPI.RequestMove). One line per handle, '-' while the request is open, then the
    // answer - read back by StandpointAPI.GetMoveOutcome / GetMoveResult. The newest 128 are kept (under g_mtx).
    struct PapLine {
        char         letter{ '-' };
        std::int32_t detail{ 0 };
        float        dist{ 0.f };
        float        facing{ 0.f };
        float        orient{ 0.f };
    };
    std::unordered_map<std::int32_t, PapLine> g_pap;
    std::deque<std::int32_t>                  g_papOrder;
    // the load generation: +1 at every load (FailAll). A Papyrus request carries the generation it was made in; an answer
    // from an older one is dropped, so nothing asked before a load is ever answered into the loaded game.
    std::atomic<std::uint32_t>                g_gen{ 1 };

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

    // v1.2.0: Request() in two parts, so the Papyrus door can hand its caller the handle at once and wake the driver one
    // frame later (from an SKSE task - not from inside a native). Open = the line gets its id; Launch = the checks and
    // the wake. The C++ RequestMove still does both at once, exactly as before.
    std::int32_t Open(const StandpointAPI::MoveRequest& a_req, StandpointAPI::MoveCallback a_cb, void* a_user, bool a_papyrus)
    {
        std::int32_t id;
        {
            std::scoped_lock lk(g_mtx);
            id = g_nextId++;
            if (g_nextId > 0x00FFFFFF) g_nextId = 1;     // stays exact as a Papyrus Int / Float
            g_open[id] = Pending{ a_req, a_cb, a_user };
            if (a_papyrus) {
                g_pap[id] = PapLine{};
                g_papOrder.push_back(id);
                while (g_papOrder.size() > 128) {
                    g_pap.erase(g_papOrder.front());
                    g_papOrder.pop_front();
                }
            }
        }
        logger::info("MOVE #{} REQUEST who={:08X} where={:08X} dist={:.0f}cm facing={:.1f}h orient={:.1f}h pose={:.0f} hold={:.1f}s close={:.0f}{}{}",
                     id, a_req.who, a_req.where, a_req.distanceCm, a_req.facing, a_req.orientation, a_req.pose,
                     a_req.holdSeconds, a_req.close, a_req.where == 0 ? " (RELEASE)" : "", a_papyrus ? " (Papyrus)" : "");
        return id;
    }

    void Launch(std::int32_t a_id)
    {
        Pending p;
        if (!Peek(a_id, p)) return;                      // answered meanwhile (a load came in between)
        const auto& req = p.req;
        // v1.4.0: a release line also ends her walk off the navmesh / the stand after it (that walk answers X 3)
        if (!req.where && req.who) standpoint_walk::StopFor(req.who, StandpointAPI::kDetailReleased);
        auto* who = req.who ? RE::TESForm::LookupByID<RE::Actor>(req.who) : nullptr;
        if (!who || who->IsPlayerRef() || !who->Is3DLoaded()) {
            logger::warn("MOVE #{}: 'who' is not a loaded NPC", a_id);
            Deliver(a_id, 'O', StandpointAPI::kDetailBadRequest);
            return;
        }
        if (req.where && !RE::TESForm::LookupByID<RE::TESObjectREFR>(req.where)) {
            logger::warn("MOVE #{}: 'where' is not a reference", a_id);
            Deliver(a_id, 'O', StandpointAPI::kDetailBadRequest);
            return;
        }
        // wake the driver: StandpointAPI.ApiWake(id) -> Standpoint_PlayerAlias.ApiRunRequest(id)
        auto* vm = RE::BSScript::Internal::VirtualMachine::GetSingleton();
        bool sent = false;
        if (vm) {
            auto* args = RE::MakeFunctionArguments(static_cast<std::int32_t>(a_id));
            RE::BSTSmartPointer<RE::BSScript::IStackCallbackFunctor> none;
            sent = vm->DispatchStaticCall("StandpointAPI", "ApiWake", args, none);
            delete args;
        }
        if (!sent) {
            logger::warn("MOVE #{}: StandpointAPI.ApiWake could not be dispatched - Standpoint's scripts are not running", a_id);
            Deliver(a_id, 'B', StandpointAPI::kDetailNotReady);
        }
    }

    // v1.2.0: the answer to a Papyrus request (main thread, inside Deliver's task): kept for GetMoveOutcome / GetMoveResult
    // and sent as the "Standpoint_MoveResult" event by StandpointAPI.ApiMoveDone. Never for a request of an older load.
    void PapyrusDone(std::uint32_t a_handle, const StandpointAPI::MoveResult& a_res, void* a_user)
    {
        const auto gen = static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(a_user));
        if (gen != g_gen.load()) return;
        if (a_res.outcome == 'X' && static_cast<std::int32_t>(a_res.detail) == StandpointAPI::kDetailGameLoaded) return;
        const auto h = static_cast<std::int32_t>(a_handle);
        {
            std::scoped_lock lk(g_mtx);
            const auto it = g_pap.find(h);
            if (it == g_pap.end()) return;
            it->second = PapLine{ a_res.outcome, static_cast<std::int32_t>(a_res.detail), a_res.distanceCm, a_res.facing,
                                  a_res.orientation };
        }
        auto* vm = RE::BSScript::Internal::VirtualMachine::GetSingleton();
        if (!vm) return;
        auto* args = RE::MakeFunctionArguments(static_cast<std::int32_t>(h), static_cast<std::int32_t>(a_res.who),
                                               static_cast<std::int32_t>(a_res.where),
                                               RE::BSFixedString(std::string(1, a_res.outcome)),
                                               static_cast<std::int32_t>(a_res.detail), static_cast<float>(a_res.distanceCm),
                                               static_cast<float>(a_res.facing), static_cast<float>(a_res.orientation));
        RE::BSTSmartPointer<RE::BSScript::IStackCallbackFunctor> none;
        if (!vm->DispatchStaticCall("StandpointAPI", "ApiMoveDone", args, none)) {
            logger::warn("MOVE #{}: StandpointAPI.ApiMoveDone could not be dispatched - the answer is only kept for GetMoveOutcome", h);
        }
        delete args;
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

    // ---- v1.2.0: the Papyrus door to the request line (StandpointAPI.RequestMove / GetMoveOutcome / GetMoveResult) ----
    // The same line as the C++ RequestMove - the same mailbox, the same driver path (ApiWake -> ApiRunRequest), the same
    // measured answer - so CLOSE mode (slot 8) is open to Papyrus mods. Returns the handle (>= 1), or 0 for no walker.
    std::int32_t ApiMoveSubmit(RE::StaticFunctionTag*, RE::Actor* a_who, RE::TESObjectREFR* a_where, float a_distCm,
                               float a_facingHours, float a_orientHours, std::int32_t a_pose, float a_holdSeconds,
                               std::int32_t a_close)
    {
        if (!a_who) return 0;
        StandpointAPI::MoveRequest q{};
        q.who         = a_who->GetFormID();
        q.where       = a_where ? a_where->GetFormID() : 0;
        q.distanceCm  = a_distCm;
        q.facing      = a_facingHours;
        q.orientation = a_orientHours;
        q.pose        = static_cast<float>(a_pose);
        q.holdSeconds = a_holdSeconds;
        q.close       = a_close != 0 ? 1.f : 0.f;
        void* user = reinterpret_cast<void*>(static_cast<std::uintptr_t>(g_gen.load()));
        const std::int32_t id = Open(q, PapyrusDone, user, true);
        if (auto* task = SKSE::GetTaskInterface()) {
            task->AddTask([id]() { Launch(id); });
        } else {
            Deliver(id, 'B', StandpointAPI::kDetailNotReady);
        }
        return id;
    }

    // "" = no such handle (never made, older than the newest 128, or made before the last load); "-" = still open;
    // else the answer's letter (C, P, O, D, L, X, B).
    RE::BSFixedString ApiMoveOutcome(RE::StaticFunctionTag*, std::int32_t a_handle)
    {
        std::scoped_lock lk(g_mtx);
        const auto it = g_pap.find(a_handle);
        if (it == g_pap.end()) return RE::BSFixedString("");
        return RE::BSFixedString(std::string(1, it->second.letter));
    }

    // [distanceCm, facingHour, orientationHour, detail] once answered; [] while open or unknown.
    std::vector<float> ApiMoveResult(RE::StaticFunctionTag*, std::int32_t a_handle)
    {
        std::vector<float> out;
        std::scoped_lock lk(g_mtx);
        const auto it = g_pap.find(a_handle);
        if (it == g_pap.end() || it->second.letter == '-') return out;
        out = { it->second.dist, it->second.facing, it->second.orient, static_cast<float>(it->second.detail) };
        return out;
    }

}  // namespace

std::uint32_t Request(const StandpointAPI::MoveRequest& a_req, StandpointAPI::MoveCallback a_cb, void* a_user)
{
    if (!a_cb) return 0;
    const std::int32_t id = Open(a_req, a_cb, a_user, false);
    Launch(id);
    return static_cast<std::uint32_t>(id);
}

void FailAll(char a_letter, std::int32_t a_detail)
{
    std::vector<std::int32_t> ids;
    {
        std::scoped_lock lk(g_mtx);
        ++g_gen;                     // v1.2.0: every Papyrus request of the old game is forgotten - it is never answered
        g_pap.clear();
        g_papOrder.clear();
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
    a_vm->RegisterFunction("ApiMoveSubmit", "Standpoint", ApiMoveSubmit);  // v1.2.0: the Papyrus door
    a_vm->RegisterFunction("ApiMoveOutcome", "Standpoint", ApiMoveOutcome);
    a_vm->RegisterFunction("ApiMoveResult", "Standpoint", ApiMoveResult);
    logger::info("move API natives registered (class Standpoint, 10 functions)");
    return true;
}

}  // namespace standpoint_move
