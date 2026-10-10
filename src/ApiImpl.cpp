// ─────────────────────────────────────────────────────────────────────────────
//  ApiImpl.cpp — the SKSE-plugin-facing Standpoint API (contract: StandpointAPI.h).
//  Mirrors the shipped WaterFinder / PpbTouchAPI pattern: 'STPT' messaging
//  handshake → GetApiFunction(revision) → stable interface pointer.
//  MAIN THREAD ONLY.
// ─────────────────────────────────────────────────────────────────────────────
#include "PCH.h"
#include "SlotSolver.h"
#include "StandpointAPI.h"
#include "MoveApi.h"

namespace logger = SKSE::log;

namespace {

    constexpr unsigned int kBuildNumber = 2;   // 2 = Standpoint 0.8.0: IStandpoint2 (RequestMove)

    RE::TESObjectREFR* RefFrom(std::uint32_t formID)
    {
        return formID ? RE::TESForm::LookupByID<RE::TESObjectREFR>(formID) : nullptr;
    }

    class StandpointInterface : public StandpointAPI::IStandpoint2 {
    public:
        unsigned int GetBuildNumber() override { return kBuildNumber; }

        bool SolveSlot(std::uint32_t targetFormID, std::int32_t relation,
                       std::uint32_t agentFormID, float standoffUnits,
                       StandpointAPI::SlotResult& out) override
        {
            std::memset(&out, 0, sizeof(out));
            auto* target = RefFrom(targetFormID);
            if (!target) {
                out.failCode = standpoint::kNoTarget;
                logger::warn("API SolveSlot: no target {:08X}", targetFormID);
                return false;
            }
            RE::Actor* agent = agentFormID ? RE::TESForm::LookupByID<RE::Actor>(agentFormID) : nullptr;
            const auto r = standpoint::Solve(target, static_cast<standpoint::Relation>(relation),
                                             agent, standoffUnits);
            out.found          = r.fail == standpoint::kOK;
            out.failCode       = r.fail;
            out.x              = r.pos.x;
            out.y              = r.pos.y;
            out.z              = r.pos.z;
            out.headingDegrees = r.headingRad * 57.2957795f;
            out.markerIndex    = r.markerIndex;
            out.quality        = r.quality;
            return out.found;
        }

        std::int32_t GetFurnitureMarkerCount(std::uint32_t targetFormID) override
        {
            return standpoint::GetMarkerCount(RefFrom(targetFormID));
        }

        bool GetFurnitureMarkerInfo(std::uint32_t targetFormID, std::int32_t markerIndex,
                                    float (&info)[7]) override
        {
            float v[7];
            if (!standpoint::GetMarkerInfo(RefFrom(targetFormID), markerIndex, v)) return false;
            v[3] *= 57.2957795f;  // degrees at the boundary, matching the header contract
            std::memcpy(info, v, sizeof(v));
            return true;
        }

        // revision 2 (v0.8.0): the 10-slot move request - the mailbox to the Papyrus driver (MoveApi.cpp)
        std::uint32_t RequestMove(const StandpointAPI::MoveRequest& request, StandpointAPI::MoveCallback callback,
                                  void* user) override
        {
            return standpoint_move::Request(request, callback, user);
        }
    };

    StandpointInterface g_interface;

    void* GetApiFunction(unsigned int revision)
    {
        // IStandpoint2 only APPENDS to IStandpoint1's vtable, so a revision-1 caller reads the same object
        if (revision == 1) return static_cast<StandpointAPI::IStandpoint1*>(&g_interface);
        if (revision == 2) return static_cast<StandpointAPI::IStandpoint2*>(&g_interface);
        logger::warn("API: revision {} not supported (max 2)", revision);
        return nullptr;
    }

}  // namespace

namespace standpoint_api {

void OnSKSEMessage(SKSE::MessagingInterface::Message* msg)
{
    if (msg && msg->type == StandpointAPI::StandpointMessage::kGetInterface && msg->data &&
        msg->dataLen >= sizeof(StandpointAPI::StandpointMessage)) {
        auto* m = static_cast<StandpointAPI::StandpointMessage*>(msg->data);
        m->GetApiFunction = GetApiFunction;
        logger::info("C++ API interface handed to '{}'", msg->sender ? msg->sender : "?");
    }
}

}  // namespace standpoint_api
