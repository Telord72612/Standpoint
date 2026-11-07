// ─────────────────────────────────────────────────────────────────────────────
//  NavGraph.cpp — v1.6.0 THE SKELETON (see NavGraph.h; Report/StandPoint Module/1.6.0 Plan/D_Standpoint_160_Build_Spec.md §1 item 6,
//  S2_Long_Path_Speed_Stairs_Wading.md §2.2 / §2.5 / §2.6). Builder A, 2026-11-06.
//
//  WHAT IT READS (CommonLibVR 8.3.0, all read only, MAIN THREAD only):
//    * the loaded cells: her interior cell, or the attached exterior cells of her worldspace (TES::GetCell at every cell of the loaded
//      square round the player - uGridsToLoad - and round her; TES's own grid array is not read); each cell's NavMeshArray
//      (TESObjectCELL::GetRuntimeData().navMeshes - VR = the SE layout, +0x78; every cell and navmesh vtable-checked) -> NavMesh
//      (TESForm + TESChildCell + BSNavmesh): vertices, triangles (3 vertex indices, 3 neighbour indices: 0xFFFF none; an edge whose
//      link bit (0-2) is set indexes extraEdgeInfo instead: a portal to ANOTHER navmesh by FormID + its triangle), closedDoors
//    * the engine's runtime navcuts: the cutter marks the triangles it replaces with bit 5 (CommonLib's kOverlapping) and sets
//      their neighbours' links to 0xFFFF (Report/RESEARCH/Runtime Navmesh/A_Engine_Feasibility.md §2.1, read in the VR exe), so a
//      cut hole is already closed by its links; bit 3 (kDeleted) is never entered. Bit 5: (1.6.1 A1, the 17:12 run) in a navmesh
//      that carries RUNTIME CUTS (its cut records, a one-way link into a bit-5 triangle, or cut-made triangles - Slot::cutWalls)
//      it is the retire mark = a WALL, never located on; elsewhere (the CK's "Overlapping" flag) it costs x6 (fix round, check 6)
//      and loses every tie in Locate (kTieDz). One line per navmesh says which rule it got (SKELETON navmesh …)
//    * the engine swaps a cut navmesh's COPY in (a new NavMesh object under the same FormID) on the main thread while pathfinding is
//      paused (ibid. §2.3 / §2.4): every step re-resolves every navmesh it uses by FormID from the loaded cells and holds their smart
//      pointers while it reads; a changed object or count restarts the search (<= 3); nothing engine-side is kept across steps
//  THE SEARCH: A* over triangles - a node's place = the middle of the edge it was entered by; g = 3D length x the cost of the
//  triangle entered (water x4 unless asked, preferred x0.9, a disc / a remembered bad leg / bit 5 x6, a closed door 1.25 or refused);
//  h = straight 3D distance x 1.2. THE FUNNEL: the crossed edges narrowed by her clearance at both ends, string-pulled; straight
//  stretches cut every <= 256 u (z from the triangles crossed). VIRTUAL LINKS when the triangles end: border edges of the reached
//  island and of other islands <= 256 u apart (cost x2) - the planner's leg test proves or refuses each.
// ─────────────────────────────────────────────────────────────────────────────
#include "PCH.h"
#include "NavGraph.h"
#include "PathPlanner.h"     // standpoint_path::IsOpenableDoor - the ONE door rule of the planner, the driver and the skeleton

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <format>
#include <mutex>
#include <queue>
#include <thread>
#include <unordered_map>
#include <unordered_set>

namespace logger = SKSE::log;

namespace standpoint_nav {
namespace {
    using Clock = std::chrono::steady_clock;

    // ---- the navmesh's own data ------------------------------------------------------------------------------------------
    constexpr std::uint16_t kNo16          = 0xFFFF;   // no neighbour across this edge
    constexpr std::uint16_t kTriDeleted    = 0x0008;   // CommonLib kDeleted: never entered
    // (fix round, check 6 - the session's decision) bit 5 is CommonLib's kOverlapping: the engine's cutter sets it on the triangles a
    // navcut replaces (research A §2.1), but in plugin data it is the CK's "Overlapping" flag that stairs and bridges carry - so it is
    // NOT a wall: such a triangle costs x6 (like a disc); the self-check line counts bit 3 and bit 5 apart.
    // (1.6.1 A1 - the 17:12 run, R4 §2) at Lakeview bit 5 WAS the runtime retire mark (memory 259 triangles over the record, no
    // Overlapping on disk, 780 bit-5 = holes cut by HearthFires' furniture / walls): in a navmesh that carries RUNTIME CUTS (Slot::
    // cutWalls - its cut records, a one-way link into a bit-5 triangle, or cut-made triangles) bit 5 is a WALL; elsewhere x6 as before
    constexpr std::uint16_t kTriOverlap    = 0x0020;
    constexpr std::uint16_t kTriCutDepth   = 0xE000;   // (1.6.1 A1) bits 13-15: the "cut depth" the cutter writes on triangles it made
                                                       // (research A §2.1 step 5: 80 %; its meaning 60 %) - evidence of a runtime cut
    constexpr float         kTieDz         = 4.f;      // (1.6.1 A1) Locate: a bit-5 triangle wins only by more than this (the tie case)
    constexpr std::uint16_t kTriPreferred  = 0x0040;   // "preferred" (roads)
    constexpr std::uint16_t kTriWater      = 0x0200;   // xEdit / UESP call it "water" (an inference, 70 %): a cost only
    constexpr std::uint32_t kPortalType    = 0;        // EDGE_EXTRA_INFO_TYPE::kPortal (ledges / enable portals are not followed)
    constexpr std::uint32_t kMaxIndex      = 0xFFFF;   // 16-bit indices: at most 65,535 vertices and triangles per navmesh
    constexpr std::uint32_t kMaxMeshesCell = 64;       // a cell with more navmeshes than this is not believed (skipped)
    constexpr std::uint32_t kMaxDoors      = 256;      // closed doors read per navmesh at most

    // ---- the search -------------------------------------------------------------------------------------------------------
    constexpr float       kHeur          = 1.2f;
    constexpr float       kCostWater     = 4.f;
    constexpr float       kCostPreferred = 0.9f;
    constexpr float       kCostDisc      = 6.f;
    constexpr float       kCostBadLeg    = 6.f;
    constexpr float       kCostOverlap   = 6.f;        // (fix round, check 6) a bit-5 triangle
    constexpr std::uint32_t kChunk       = 4096;       // (fix round, nit 7) vertices / triangles one unit of work reads at most
    constexpr float       kCostDoor      = 1.25f;
    constexpr float       kCostLink      = 2.f;
    constexpr int         kMaxExpand     = 40000;      // triangles expanded per search at most
    constexpr int         kMaxRestarts   = 3;
    constexpr std::size_t kMaxSlots      = 512;        // navmeshes one search may read (reserved: no slot ever moves)
    constexpr float       kOnTriDz       = 48.f;       // a point is ON a triangle: inside it (2D) and within this of its plane
    constexpr float       kJoinDz        = 96.f;       // a join: the nearest triangle within joinMax (2D) and this height
    constexpr float       kMaxStraight   = 256.f;      // straight stretches are cut at least this often
    constexpr float       kLinkMax       = 256.f;      // virtual links: border edges at most this far apart (2D) ...
    constexpr float       kLinkDz        = 96.f;       // ... and this in height
    constexpr int         kLinkRounds    = 2;
    constexpr std::size_t kLinksMax      = 256;        // links offered per round at most (the shortest)
    constexpr float       kBadLegNear    = 48.f;       // a remembered bad leg: an edge crossing this close to it costs x6
    constexpr std::size_t kBadLegsMax    = 64;
    constexpr float       kDiscDz        = 128.f;
    constexpr float       kCellU         = 4096.f;     // an exterior cell's side
    constexpr float       kDupe          = 4.f;        // two skeleton points this close are one
    constexpr int         kCheckEvery    = 16;         // the clock is read every 16 expansions / edges
    constexpr float       kUnitsPerM     = 70.f;

    // ---- its own jobs ---------------------------------------------------------------------------------------------------
    constexpr double kJobFrameUs = 250.0;              // (fix round, nit 14) ONE frame budget shared by all running jobs ...
    constexpr double kJobMinUs   = 50.0;               // ... a job's step never less than this
    constexpr int    kMaxJobs    = 16;
    constexpr int    kPumpTicks  = 7500;               // ~60 s at 8 ms

    struct Node {                                      // one triangle's search state
        float         g{ 3.0e38f };
        float         px{ 0.f }, py{ 0.f }, pz{ 0.f };  // where the way enters it (the middle of the edge crossed; the start: her point)
        std::int32_t  parentSlot{ -1 };
        std::int32_t  link{ -1 };                      // the virtual link it came by (via 3)
        std::uint16_t parentTri{ kNo16 };
        std::uint8_t  via{ 0xFF };                     // the parent's edge it came through (0-2); 3 a virtual link; 0xFF the start
        bool          closed{ false };
    };

    struct Slot {                                      // one navmesh a search reads
        RE::FormID                 id{ 0 };
        const void*                ptr{ nullptr };     // its object when first read (compared at every step, never followed)
        std::uint32_t              nTri{ 0 }, nVert{ 0 }, nExtra{ 0 };
        bool                       bad{ false };       // its self-check failed: never entered
        float                      x0{ 0.f }, y0{ 0.f }, x1{ 0.f }, y1{ 0.f }, z0{ 0.f }, z1{ 0.f };
        std::vector<Node>          nodes;              // per triangle, made at the first touch
        std::vector<std::uint16_t> doorTri;            // its closed doors' triangles (sorted) ...
        std::vector<std::uint8_t>  doorPass;           // ... 1 = she may pass (openable, open, or not readable), 0 = a wall
        int                        deleted{ 0 }, overlap{ 0 }, links{ 0 }, doorsWall{ 0 };   // bit-3 / bit-5 triangles, edge links
        bool                       doorsUnread{ false };   // (nit 8) its closed-door list did not look like one: not read
        // (1.6.1 A1) runtime-cut evidence, read by the self-check: the navmesh's cut records (BSNavmesh::obstacles - the engine's
        // undo stack of navcuts; obstaclesOdd = a count that is not believable: not used), bit-5 triangles a live neighbour of
        // theirs no longer links back to (the cutter's step 4), triangles with a cut depth (bits 13-15: made by a cut)
        std::uint32_t              obstacles{ 0 };
        bool                       obstaclesOdd{ false };
        int                        oneWay{ 0 }, cutMade{ 0 };
        bool                       cutWalls{ false };  // runtime cuts here: bit 5 = a WALL (navcut-retired); else bit 5 costs x6
        int                        checkStage{ 0 };    // (nit 7) the self-check, resumable: 0 vertices, 1 triangles, 2 doors, 3 done
        std::uint32_t              checkK{ 0 };
        RE::NavMesh*               mesh{ nullptr };    // THIS step only
        bool Checked() const { return checkStage >= 3; }
    };

    struct LiveMesh {
        RE::NavMesh* mesh{ nullptr };
        std::int32_t cx{ 0 }, cy{ 0 };
    };

    struct Live {                                      // the loaded area's navmeshes, resolved for ONE step
        // their references, held while the step reads - through the BSNavmesh base (the same refcount: a BSTSmartPointer<NavMesh>
        // cannot be made here, NavMesh has two bases with an operator delete)
        std::vector<RE::BSTSmartPointer<RE::BSNavmesh>> hold;
        std::unordered_map<RE::FormID, LiveMesh>      byId;
        bool                                          interior{ false };
        int                                           rejected{ 0 };      // navmesh objects that did not carry the NavMesh vtable
    };

    struct Open {
        float         f{ 0.f }, g{ 0.f };
        std::int32_t  slot{ -1 };
        std::uint16_t tri{ kNo16 };
        bool operator>(const Open& a_o) const { return f > a_o.f; }
    };

    struct VLink {
        std::int32_t  fs{ -1 }, ts{ -1 };
        std::uint16_t ft{ kNo16 }, tt{ kNo16 };
        RE::NiPoint3  a{}, b{};
        float         len{ 0.f };
    };

    struct BorderEdge {                                // an edge of a reached triangle with no neighbour
        std::int32_t  slot{ -1 };
        std::uint16_t tri{ kNo16 };
        RE::NiPoint3  mid{};
    };

    struct BadLeg {
        RE::NiPoint3      a{}, b{};
        Clock::time_point until{};
    };

    struct ChainTri {                                  // the funnel's copy of one triangle of the way (no engine data after it)
        std::array<RE::NiPoint3, 3> v{};
        std::uint16_t               flags{ 0 };
        bool                        door{ false };
    };

    struct Portal {                                    // left / right as seen walking it; a break = a virtual link's two ends
        RE::NiPoint3 l{}, r{};
        std::size_t  tri{ 0 };                         // the chain triangle it leads into
        bool         brk{ false };
    };

    struct LocState {                                  // (fix round, nit 7) a Locate in progress - resumable, one chunk a unit
        bool                    begun{ false };
        std::vector<RE::FormID> ids;                   // the candidate navmeshes (sorted)
        std::size_t             k{ 0 };                // the one being scanned ...
        std::uint32_t           tri{ 0 };              // ... from this triangle
        float                   bestOn{ 3.0e38f }, bestJoin{ 3.0e38f };
        RE::NiPoint3            onPt{}, joinPt{};
        std::int32_t            onS{ -1 }, joinS{ -1 };
        std::uint16_t           onT{ 0xFFFF }, joinT{ 0xFFFF };
    };

    // ---- shared memories (g_memMtx; taken last, never while holding it is anything else taken) ---------------------------
    std::mutex                       g_memMtx;
    std::vector<BadLeg>              g_badLegs;
    std::unordered_set<RE::FormID>   g_selfLogged;     // walkers whose self-check line was written this game run
    std::unordered_map<RE::FormID, std::uint64_t> g_ruleLogged;   // (1.6.1 A1) navmeshes whose bit-5 rule line was written (its signature)

    float Dist3(const RE::NiPoint3& a_a, const RE::NiPoint3& a_b) { return (a_a - a_b).Length(); }
    float Dist2(const RE::NiPoint3& a_a, const RE::NiPoint3& a_b) { return std::hypot(a_a.x - a_b.x, a_a.y - a_b.y); }

    // (a - o) x (b - o) in 2D: > 0 = b lies LEFT of the ray o -> a (x east, y north)
    float Cross2(const RE::NiPoint3& a_o, const RE::NiPoint3& a_a, const RE::NiPoint3& a_b)
    {
        return (a_a.x - a_o.x) * (a_b.y - a_o.y) - (a_a.y - a_o.y) * (a_b.x - a_o.x);
    }

    bool Same2(const RE::NiPoint3& a_a, const RE::NiPoint3& a_b) { return std::fabs(a_a.x - a_b.x) < 0.01f && std::fabs(a_a.y - a_b.y) < 0.01f; }

    bool Finite(const RE::NiPoint3& a_p)
    {
        return std::isfinite(a_p.x) && std::isfinite(a_p.y) && std::isfinite(a_p.z) && std::fabs(a_p.x) < 4.0e6f && std::fabs(a_p.y) < 4.0e6f &&
               std::fabs(a_p.z) < 1.0e5f;
    }

    // the plane's height at (x, y) over triangle (a, b, c); false = degenerate
    bool PlaneZ(const RE::NiPoint3& a_a, const RE::NiPoint3& a_b, const RE::NiPoint3& a_c, float a_x, float a_y, float& a_z)
    {
        const float d = (a_b.y - a_c.y) * (a_a.x - a_c.x) + (a_c.x - a_b.x) * (a_a.y - a_c.y);
        if (std::fabs(d) < 1e-4f) return false;
        const float l1 = ((a_b.y - a_c.y) * (a_x - a_c.x) + (a_c.x - a_b.x) * (a_y - a_c.y)) / d;
        const float l2 = ((a_c.y - a_a.y) * (a_x - a_c.x) + (a_a.x - a_c.x) * (a_y - a_c.y)) / d;
        a_z = l1 * a_a.z + l2 * a_b.z + (1.f - l1 - l2) * a_c.z;
        return true;
    }

    // (x, y) inside triangle (a, b, c) in 2D (either winding; on an edge counts)
    bool Inside2(const RE::NiPoint3& a_a, const RE::NiPoint3& a_b, const RE::NiPoint3& a_c, float a_x, float a_y)
    {
        const RE::NiPoint3 p{ a_x, a_y, 0.f };
        const float        d1 = Cross2(a_a, a_b, p), d2 = Cross2(a_b, a_c, p), d3 = Cross2(a_c, a_a, p);
        const bool         neg = d1 < -0.01f || d2 < -0.01f || d3 < -0.01f, pos = d1 > 0.01f || d2 > 0.01f || d3 > 0.01f;
        return !(neg && pos);
    }

    // the point of triangle (a, b, c) nearest (x, y) in 2D, with the plane's height there
    RE::NiPoint3 Nearest2(const RE::NiPoint3& a_a, const RE::NiPoint3& a_b, const RE::NiPoint3& a_c, float a_x, float a_y)
    {
        if (Inside2(a_a, a_b, a_c, a_x, a_y)) {
            float z = (a_a.z + a_b.z + a_c.z) / 3.f;
            PlaneZ(a_a, a_b, a_c, a_x, a_y, z);
            return { a_x, a_y, z };
        }
        RE::NiPoint3 best{};
        float        bestD = 3.0e38f;
        const std::array<std::pair<const RE::NiPoint3*, const RE::NiPoint3*>, 3> edges{ { { &a_a, &a_b }, { &a_b, &a_c }, { &a_c, &a_a } } };
        for (const auto& [p, q] : edges) {
            const float ex = q->x - p->x, ey = q->y - p->y, l2 = ex * ex + ey * ey;
            float       t = l2 > 1e-4f ? ((a_x - p->x) * ex + (a_y - p->y) * ey) / l2 : 0.f;
            t = std::clamp(t, 0.f, 1.f);
            const RE::NiPoint3 c{ p->x + ex * t, p->y + ey * t, p->z + (q->z - p->z) * t };
            const float        d = std::hypot(c.x - a_x, c.y - a_y);
            if (d < bestD) {
                bestD = d;
                best = c;
            }
        }
        return best;
    }

    // the 2D distance from p to the segment a-b
    float SegDist2(const RE::NiPoint3& a_p, const RE::NiPoint3& a_a, const RE::NiPoint3& a_b)
    {
        const float ex = a_b.x - a_a.x, ey = a_b.y - a_a.y, l2 = ex * ex + ey * ey;
        float       t = l2 > 1e-4f ? ((a_p.x - a_a.x) * ex + (a_p.y - a_a.y) * ey) / l2 : 0.f;
        t = std::clamp(t, 0.f, 1.f);
        return std::hypot(a_a.x + ex * t - a_p.x, a_a.y + ey * t - a_p.y);
    }

    // the object really is what we read it as: its first word is the game's own vtable for that class (the way Calibrate checks the
    // controller - a wrong offset can never be followed further)
    bool IsCellObj(const RE::TESObjectCELL* a_c) { return a_c && *reinterpret_cast<const std::uintptr_t*>(a_c) == RE::VTABLE_TESObjectCELL[0].address(); }
    bool IsNavMeshObj(const RE::NavMesh* a_m) { return a_m && *reinterpret_cast<const std::uintptr_t*>(a_m) == RE::VTABLE_NavMesh[0].address(); }

    // the loaded square's half width in cells: uGridsToLoad / 2 (Skyrim.ini [General]; read once - main thread only; default 5 -> 2)
    std::int32_t GridRadius()
    {
        static std::int32_t r = -1;
        if (r < 0) {
            std::uint32_t g = 5;
            if (auto* ini = RE::INISettingCollection::GetSingleton()) {
                if (auto* st = ini->GetSetting("uGridsToLoad:General"); st && st->GetType() == RE::Setting::Type::kUnsignedInteger) {
                    g = st->GetUnsignedInteger();
                }
            }
            if (g < 3 || g > 15 || (g % 2u) == 0u) g = 5;
            r = static_cast<std::int32_t>(g / 2u);
        }
        return r;
    }

    // the loaded area's navmeshes NOW: her interior cell, else the attached exterior cells of her worldspace. Their references are
    // held in a_live until the step ends. The exterior cells are found with TES::GetCell (the engine's own look-up the planner has used
    // in VR since 1.4.0) at the middle of every cell of the loaded square round the player (uGridsToLoad / 2) and within 1 of hers -
    // TES's own grid array is NOT read (its VR layout was never measured). Every cell and every navmesh must carry the game's vtable.
    bool CollectLive(RE::Actor* a_w, Live& a_live)
    {
        a_live.hold.clear();
        a_live.byId.clear();
        a_live.rejected = 0;
        auto* cell = a_w ? a_w->GetParentCell() : nullptr;
        if (!cell || !IsCellObj(cell)) return false;
        const auto add = [&a_live](RE::TESObjectCELL* a_c, bool a_int) {
            RE::NavMeshArray* arr = a_c->GetRuntimeData().navMeshes;
            if (!arr) return;
            std::int32_t cx = 0, cy = 0;
            if (!a_int) {
                if (const auto* ext = a_c->GetCoordinates()) {
                    cx = ext->cellX;
                    cy = ext->cellY;
                }
            }
            const std::uint32_t n = arr->navMeshes.size();
            if (n > kMaxMeshesCell || (n && !arr->navMeshes.data())) return;
            for (std::uint32_t i = 0; i < n; ++i) {
                const RE::BSTSmartPointer<RE::NavMesh>& sp = arr->navMeshes[i];
                RE::NavMesh*                            m = sp.get();
                if (!m) continue;
                if (!IsNavMeshObj(m)) {
                    ++a_live.rejected;
                    continue;
                }
                const RE::FormID fid = m->GetFormID();
                if (!fid || a_live.byId.contains(fid)) continue;
                a_live.hold.emplace_back(static_cast<RE::BSNavmesh*>(m));
                a_live.byId.emplace(fid, LiveMesh{ m, cx, cy });
            }
        };
        a_live.interior = cell->IsInteriorCell();
        if (a_live.interior) {
            add(cell, true);
            return true;
        }
        auto* tes = RE::TES::GetSingleton();
        if (!tes) return false;
        auto*                           ws = a_w->GetWorldspace();
        std::vector<RE::TESObjectCELL*> seen;
        const auto visit = [&](std::int32_t a_cx, std::int32_t a_cy) {
            RE::TESObjectCELL* c = tes->GetCell(RE::NiPoint3{ (static_cast<float>(a_cx) + 0.5f) * kCellU, (static_cast<float>(a_cy) + 0.5f) * kCellU, 0.f });
            if (!c || std::find(seen.begin(), seen.end(), c) != seen.end()) return;
            seen.push_back(c);
            if (!IsCellObj(c) || c->IsInteriorCell() || !c->IsAttached() || c->GetRuntimeData().worldSpace != ws) return;
            add(c, false);
        };
        const auto around = [&](const RE::NiPoint3& a_p, std::int32_t a_r) {
            const auto cx = static_cast<std::int32_t>(std::floor(a_p.x / kCellU)), cy = static_cast<std::int32_t>(std::floor(a_p.y / kCellU));
            for (std::int32_t dx = -a_r; dx <= a_r; ++dx) {
                for (std::int32_t dy = -a_r; dy <= a_r; ++dy) visit(cx + dx, cy + dy);
            }
        };
        if (auto* pl = RE::PlayerCharacter::GetSingleton(); pl && pl->GetWorldspace() == ws) around(pl->GetPosition(), GridRadius());
        around(a_w->GetPosition(), 1);
        return true;
    }

    // THE SELF-CHECK of one navmesh, part 1 (at its first read, cheap): counts sane, data pointers present. false = a_why.
    bool CheckCounts(Slot& a_sl, std::string& a_why)
    {
        const RE::NavMesh* m = a_sl.mesh;
        const auto&        V = m->vertices;
        const auto&        T = m->triangles;
        const auto&        E = m->extraEdgeInfo;
        a_sl.nTri = T.size();
        a_sl.nVert = V.size();
        a_sl.nExtra = E.size();
        if (a_sl.nTri > kMaxIndex || a_sl.nVert > kMaxIndex || a_sl.nExtra > 0x100000u) {
            a_why = std::format("navmesh {:08X}: {} triangles / {} vertices / {} links - not believable counts", a_sl.id, a_sl.nTri, a_sl.nVert, a_sl.nExtra);
            return false;
        }
        if ((a_sl.nTri && !T.data()) || (a_sl.nVert && !V.data()) || (a_sl.nExtra && !E.data())) {
            a_why = std::format("navmesh {:08X}: an array with a count but no data", a_sl.id);
            return false;
        }
        // (1.6.1 A1) its runtime cut records: only the count is read (BSNavmesh +0xD0, research A §1.1: 90 %); a count that does not
        // look like one is no evidence either way
        {
            const auto&         O = m->obstacles;
            const std::uint32_t nO = O.size();
            if (nO <= 0x1000u && (nO == 0 || O.data())) {
                a_sl.obstacles = nO;
            } else {
                a_sl.obstaclesOdd = true;
            }
        }
        a_sl.x0 = a_sl.y0 = a_sl.z0 = 3.0e38f;
        a_sl.x1 = a_sl.y1 = a_sl.z1 = -3.0e38f;
        a_sl.checkStage = 0;
        a_sl.checkK = 0;
        return true;
    }

    // THE SELF-CHECK, part 2 - ONE unit of it (fix round, nit 7: resumable, never a 0.5-1.5 ms hitch in a big navmesh): <= 4,096
    // vertices (finite; the bounds), or <= 4,096 triangles (every vertex index and every internal neighbour index in range - the engine
    // trusts them unchecked: out of range = we read the layout wrong; a link edge whose extraEdgeInfo index is out of range is only a
    // border; bit 3 / bit 5 counted apart), or its closed doors (a closed locked / load / wired door's triangle is a wall; an openable or
    // open one, or one that cannot be read, is passed - the planner's rays meet the door and open it). 1 = done, 0 = more units,
    // -1 = failed (a_why).
    int CheckMeshStep(Slot& a_sl, std::string& a_why)
    {
        if (a_sl.checkStage >= 3) return 1;
        const RE::NavMesh* m = a_sl.mesh;
        if (!m) return 0;                              // (not resolved this step: never - the callers hold a resolved slot)
        if (a_sl.checkStage == 0) {
            const RE::BSNavmeshVertex* V = m->vertices.data();
            const std::uint32_t        end = (std::min)(a_sl.nVert, a_sl.checkK + kChunk);
            for (std::uint32_t i = a_sl.checkK; i < end; ++i) {
                const RE::NiPoint3& p = V[i].location;
                if (!Finite(p)) {
                    a_why = std::format("navmesh {:08X}: vertex {} is not a finite point", a_sl.id, i);
                    return -1;
                }
                a_sl.x0 = (std::min)(a_sl.x0, p.x);
                a_sl.y0 = (std::min)(a_sl.y0, p.y);
                a_sl.z0 = (std::min)(a_sl.z0, p.z);
                a_sl.x1 = (std::max)(a_sl.x1, p.x);
                a_sl.y1 = (std::max)(a_sl.y1, p.y);
                a_sl.z1 = (std::max)(a_sl.z1, p.z);
            }
            a_sl.checkK = end;
            if (end >= a_sl.nVert) {
                a_sl.checkStage = 1;
                a_sl.checkK = 0;
            }
            return 0;
        }
        if (a_sl.checkStage == 1) {
            const RE::BSNavmeshTriangle* T = m->triangles.data();
            const std::uint32_t          end = (std::min)(a_sl.nTri, a_sl.checkK + kChunk);
            for (std::uint32_t i = a_sl.checkK; i < end; ++i) {
                const RE::BSNavmeshTriangle& t = T[i];
                const std::uint16_t          fl = t.triangleFlags.underlying();
                for (int e = 0; e < 3; ++e) {
                    if (t.vertices[e] >= a_sl.nVert) {
                        a_why = std::format("navmesh {:08X}: triangle {} names vertex {} of {}", a_sl.id, i, t.vertices[e], a_sl.nVert);
                        return -1;
                    }
                    if (fl & (1u << e)) {
                        ++a_sl.links;
                    } else if (t.triangles[e] != kNo16 && t.triangles[e] >= a_sl.nTri) {
                        a_why = std::format("navmesh {:08X}: triangle {} names neighbour {} of {}", a_sl.id, i, t.triangles[e], a_sl.nTri);
                        return -1;
                    }
                }
                if (fl & kTriDeleted) ++a_sl.deleted;
                if (fl & kTriCutDepth) ++a_sl.cutMade;     // (1.6.1 A1) made by a runtime cut
                if (fl & kTriOverlap) {
                    ++a_sl.overlap;
                    // (1.6.1 A1) the cutter's step 4: a live neighbour (this navmesh, no bit 5) that no longer links back to it. Only the
                    // two triangles' own index values are compared (both arrays were range-checked above / are checked in this pass).
                    for (int e = 0; e < 3; ++e) {
                        const std::uint16_t nb = t.triangles[e];
                        if ((fl & (1u << e)) || nb == kNo16 || nb >= a_sl.nTri) continue;
                        const RE::BSNavmeshTriangle& n = T[nb];
                        const std::uint16_t          nfl = n.triangleFlags.underlying();
                        if (nfl & kTriOverlap) continue;  // retired beside retired: no evidence
                        bool back = false;
                        for (int f = 0; f < 3; ++f) {
                            if (!(nfl & (1u << f)) && n.triangles[f] == static_cast<std::uint16_t>(i)) back = true;
                        }
                        if (!back) {
                            ++a_sl.oneWay;
                            break;
                        }
                    }
                }
            }
            a_sl.checkK = end;
            if (end >= a_sl.nTri) {
                // (1.6.1 A1) THE RULE for this navmesh's bit-5 triangles: any runtime-cut evidence -> walls (navcut-retired)
                a_sl.cutWalls = a_sl.overlap > 0 && (a_sl.obstacles > 0 || a_sl.oneWay > 0 || a_sl.cutMade > 0);
                a_sl.checkStage = 2;
                a_sl.checkK = 0;
            }
            return 0;
        }
        // its closed doors (fix round, nit 8: a list that does not look like one - more entries than triangles, or no data - is not
        // read at all, so the vtable compare below never touches a wild pointer)
        const auto&         D = m->closedDoors;
        const std::uint32_t nD = D.size();
        if (nD > a_sl.nTri || nD > kMaxDoors || (nD && !D.data())) {
            a_sl.doorsUnread = nD != 0;
        } else {
            std::vector<std::pair<std::uint16_t, std::uint8_t>> doors;
            for (std::uint32_t i = 0; i < nD; ++i) {
                const RE::BSNavmeshClosedDoorInfo& cd = D.data()[i];
                if (cd.triangleIndex >= a_sl.nTri) continue;
                std::uint8_t       pass = 1;
                RE::BSPathingDoor* bd = cd.door.get();
                if (bd && *reinterpret_cast<const std::uintptr_t*>(bd) == RE::VTABLE_PathingDoor[0].address()) {
                    const auto*                      pd = static_cast<const RE::PathingDoor*>(bd);
                    RE::NiPointer<RE::TESObjectREFR> ref;
                    if (pd->refHandle && RE::TESObjectREFR::LookupByHandle(pd->refHandle, ref) && ref) {
                        using OS = RE::BGSOpenCloseForm::OPEN_STATE;
                        const OS   st = RE::BGSOpenCloseForm::GetOpenState(ref.get());
                        const bool closed = st == OS::kClosed || st == OS::kClosing;
                        pass = (!closed || standpoint_path::IsOpenableDoor(ref.get())) ? 1 : 0;
                    }
                }
                if (!pass) ++a_sl.doorsWall;
                doors.emplace_back(cd.triangleIndex, pass);
            }
            std::sort(doors.begin(), doors.end());
            for (const auto& [t, p] : doors) {
                a_sl.doorTri.push_back(t);
                a_sl.doorPass.push_back(p);
            }
        }
        a_sl.checkStage = 3;
        return 1;
    }

    // -1 no closed door on this triangle, 0 a wall, 1 she may pass
    int DoorAt(const Slot& a_sl, std::uint16_t a_tri)
    {
        if (a_sl.doorTri.empty()) return -1;
        const auto it = std::lower_bound(a_sl.doorTri.begin(), a_sl.doorTri.end(), a_tri);
        if (it == a_sl.doorTri.end() || *it != a_tri) return -1;
        return a_sl.doorPass[static_cast<std::size_t>(it - a_sl.doorTri.begin())];
    }
}  // namespace

// ─────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────
struct Search {
    // the request
    RE::FormID   walker{ 0 };
    RE::NiPoint3 from{}, to{};
    SkelOptions  o{};
    std::int32_t id{ 0 };              // its job's handle (0 = stepped inside a plan: logged by the plan)
    Clock::time_point t0{};
    // state
    int                                          stage{ 0 };   // 0 her start, 1 the goal, 2 the A*, 3 links, 4 the chain, 5 the funnel, 6 done
    std::vector<Slot>                            slots;
    std::unordered_map<RE::FormID, std::int32_t> slotOf;
    Live                                         live;
    std::priority_queue<Open, std::vector<Open>, std::greater<Open>> open;
    std::int32_t                                 startSlot{ -1 }, goalSlot{ -1 };
    std::uint16_t                                startTri{ kNo16 }, goalTri{ kNo16 };
    RE::NiPoint3                                 startPt{}, goalPt{};
    std::vector<VLink>                           links;
    int                                          linkRound{ 0 };
    int                                          linkStage{ 0 };      // 0 the reached border, 1 the other islands' edges (one navmesh a unit)
    std::vector<BorderEdge>                      border;
    std::unordered_map<std::uint64_t, std::vector<std::uint32_t>> borderGrid;   // 256 u buckets -> border edges
    std::vector<RE::FormID>                      linkMeshes;          // the navmeshes the link search still looks at
    std::size_t                                  linkK{ 0 };
    std::unordered_map<std::uint64_t, VLink>     linkBest;            // the shortest link into each other-island triangle
    float                                        bx0{ 0.f }, by0{ 0.f }, bx1{ 0.f }, by1{ 0.f };   // the reached border's box
    std::vector<BadLeg>                          bad;                 // the remembered bad legs near this search
    int                                          badMeshes{ 0 };
    std::string                                  badWhy;
    // (fix round, nit 7) the resumable pieces: a Locate in progress; a navmesh the A* reached whose self-check must finish before the
    // expansion that met it is done again; the link search's place inside the current navmesh
    LocState                                     loc;
    std::int32_t                                 pendSlot{ -1 };
    std::int32_t                                 pendNodeSlot{ -1 };
    std::uint16_t                                pendNodeTri{ kNo16 };
    std::uint32_t                                linkTri{ 0 };
    // the way, copied out of the engine's arrays (stage 4) for the funnel (stage 5)
    std::vector<ChainTri>                        chain;
    std::vector<Portal>                          portals;
    std::vector<std::pair<RE::NiPoint3, RE::NiPoint3>> chainLinks;   // each virtual link's two ends, in order
    // the answer
    SkelInfo                                     info{};
    std::vector<SkelPoint>                       pts;
    bool                                         done{ false };
};

void SearchDeleter::operator()(Search* a_s) const noexcept { delete a_s; }

namespace {
    void Finish(Search& s, int a_status, std::string a_why)
    {
        s.info.status = a_status;
        s.info.why = std::move(a_why);
        s.stage = 6;
        s.done = true;
        if (a_status != 1) s.pts.clear();
        // the search's memory goes now (its answer stays)
        std::vector<Slot>().swap(s.slots);
        s.slotOf.clear();
        decltype(s.open)().swap(s.open);
        std::vector<BorderEdge>().swap(s.border);
        s.borderGrid.clear();
        s.linkBest.clear();
        std::vector<ChainTri>().swap(s.chain);
        std::vector<Portal>().swap(s.portals);
    }

    void MarkBad(Search& s, Slot& a_sl, std::string a_why)
    {
        a_sl.bad = true;
        a_sl.checkStage = 3;
        ++s.badMeshes;
        if (s.badWhy.empty()) s.badWhy = std::move(a_why);
    }

    // the slot of a loaded navmesh, made at its first use (its counts checked at once; the rest of its self-check over the next units:
    // CheckUnit - a slot is used only once Checked()); -1 = not loaded, or no room
    std::int32_t SlotFor(Search& s, RE::FormID a_id)
    {
        if (const auto it = s.slotOf.find(a_id); it != s.slotOf.end()) return it->second;
        const auto lt = s.live.byId.find(a_id);
        if (lt == s.live.byId.end() || s.slots.size() >= kMaxSlots) return -1;
        Slot sl;
        sl.id = a_id;
        sl.mesh = lt->second.mesh;
        sl.ptr = sl.mesh;
        const auto idx = static_cast<std::int32_t>(s.slots.size());
        s.slots.push_back(std::move(sl));              // (reserved: no slot moves)
        s.slotOf.emplace(a_id, idx);
        s.info.meshes = static_cast<int>(s.slots.size());
        Slot&       made = s.slots.back();
        std::string why;
        if (!CheckCounts(made, why)) MarkBad(s, made, std::move(why));
        return idx;
    }

    void RuleLine(const Search& s, const Slot& a_sl);     // (1.6.1 A1, defined below)

    // ONE unit of a slot's self-check; true = it is done (checked, or found bad)
    bool CheckUnit(Search& s, Slot& a_sl)
    {
        if (a_sl.Checked()) return true;
        std::string why;
        const int   r = CheckMeshStep(a_sl, why);
        if (r < 0) {
            MarkBad(s, a_sl, std::move(why));
            return true;
        }
        if (r > 0) RuleLine(s, a_sl);             // (1.6.1 A1) the rule its bit-5 triangles got
        return r > 0;
    }

    // every slot's navmesh for this step; false = one is gone or was swapped / changed (the engine cut or restored a hole) - restart
    bool ResolveSlots(Search& s)
    {
        for (auto& sl : s.slots) {
            sl.mesh = nullptr;
            const auto it = s.live.byId.find(sl.id);
            if (it == s.live.byId.end()) return false;
            RE::NavMesh* m = it->second.mesh;
            if (static_cast<const void*>(m) != sl.ptr || m->triangles.size() != sl.nTri || m->vertices.size() != sl.nVert ||
                m->extraEdgeInfo.size() != sl.nExtra) {
                return false;
            }
            sl.mesh = m;
        }
        return true;
    }

    void Restart(Search& s)
    {
        std::vector<Slot>().swap(s.slots);
        s.slots.reserve(kMaxSlots);
        s.slotOf.clear();
        decltype(s.open)().swap(s.open);
        s.links.clear();
        s.linkRound = 0;
        s.linkStage = 0;
        s.border.clear();
        s.borderGrid.clear();
        s.linkMeshes.clear();
        s.linkBest.clear();
        s.startSlot = s.goalSlot = -1;
        s.startTri = s.goalTri = kNo16;
        s.loc = LocState{};
        s.pendSlot = -1;
        s.pendNodeSlot = -1;
        s.pendNodeTri = kNo16;
        s.linkTri = 0;
        s.stage = 0;
    }

    const RE::NiPoint3& Vert(const Slot& a_sl, std::uint16_t a_i) { return a_sl.mesh->vertices.data()[a_i].location; }
    const RE::BSNavmeshTriangle& Tri(const Slot& a_sl, std::uint16_t a_i) { return a_sl.mesh->triangles.data()[a_i]; }

    // a triangle the search may enter: its navmesh fully self-checked and good, the index in range, not deleted (bit 3). (Fix round,
    // check 6: bit 5 is no wall - it costs x6 in Expand.) (1.6.1 A1) except in a navmesh with runtime cuts: there bit 5 = retired by
    // a navcut = a WALL (never entered, never located on, never a virtual link's end)
    bool Usable(const Slot& a_sl, std::uint16_t a_t)
    {
        if (a_sl.bad || !a_sl.Checked() || a_t >= a_sl.nTri) return false;
        const std::uint16_t fl = Tri(a_sl, a_t).triangleFlags.underlying();
        return (fl & kTriDeleted) == 0u && !(a_sl.cutWalls && (fl & kTriOverlap));
    }

    // ONE unit of locating a point's triangle (fix round, nit 7: resumable - a navmesh's self-check chunk, or 4,096 of its triangles,
    // a unit): the one it lies ON (inside in 2D, its plane within 48 u - the nearest such plane; 1.6.1: a bit-5 triangle only when
    // more than 4 u nearer than a live one, and never in a navmesh with runtime cuts), else (a join) the nearest triangle
    // within joinMax (2D) and 96 u of height. Exterior: the navmeshes of the cells within joinMax (+ 64 u) of it. 0 = more units,
    // 1 = found (the outputs set), -1 = none.
    int LocateStep(Search& s, const RE::NiPoint3& a_p, std::int32_t& a_slot, std::uint16_t& a_tri, RE::NiPoint3& a_on, bool& a_joined)
    {
        LocState&   L = s.loc;
        const float join = std::clamp(s.o.joinMax, 0.f, 1024.f);
        if (!L.begun) {                                // the candidates (its own unit)
            L = LocState{};
            L.begun = true;
            for (const auto& [fid, lm] : s.live.byId) {
                if (!s.live.interior) {
                    const float x0 = static_cast<float>(lm.cx) * kCellU, y0 = static_cast<float>(lm.cy) * kCellU;
                    const float dx = (std::max)({ 0.f, x0 - a_p.x, a_p.x - (x0 + kCellU) });
                    const float dy = (std::max)({ 0.f, y0 - a_p.y, a_p.y - (y0 + kCellU) });
                    if (std::hypot(dx, dy) > join + 64.f) continue;
                }
                L.ids.push_back(fid);
            }
            std::sort(L.ids.begin(), L.ids.end());     // the same order every time
            return 0;
        }
        if (L.k < L.ids.size()) {
            const std::int32_t si = SlotFor(s, L.ids[L.k]);
            if (si < 0) {
                ++L.k;
                L.tri = 0;
                return 0;
            }
            Slot& sl = s.slots[static_cast<std::size_t>(si)];
            if (!sl.Checked()) {                       // its self-check first, one chunk a unit
                CheckUnit(s, sl);
                return 0;
            }
            if (sl.bad || sl.nTri == 0 || a_p.x < sl.x0 - join || a_p.x > sl.x1 + join || a_p.y < sl.y0 - join || a_p.y > sl.y1 + join) {
                ++L.k;
                L.tri = 0;
                return 0;
            }
            const std::uint32_t end = (std::min)(sl.nTri, L.tri + kChunk);
            for (std::uint32_t t = L.tri; t < end; ++t) {
                const auto tt = static_cast<std::uint16_t>(t);
                if (!Usable(sl, tt)) continue;
                const RE::BSNavmeshTriangle& tri = Tri(sl, tt);
                const RE::NiPoint3&          a = Vert(sl, tri.vertices[0]);
                const RE::NiPoint3&          b = Vert(sl, tri.vertices[1]);
                const RE::NiPoint3&          c = Vert(sl, tri.vertices[2]);
                const float minx = (std::min)({ a.x, b.x, c.x }), maxx = (std::max)({ a.x, b.x, c.x });
                const float miny = (std::min)({ a.y, b.y, c.y }), maxy = (std::max)({ a.y, b.y, c.y });
                if (a_p.x < minx - join || a_p.x > maxx + join || a_p.y < miny - join || a_p.y > maxy + join) continue;
                // (1.6.1 A1) the TIE rule: a bit-5 triangle (where it is no wall) wins only when it lies more than kTieDz nearer - a retired
                // original and its live replacement share one plane, and the first in index order was the retired one ("none of them
                // the goal's" x2 at Lakeview); an overlapping stair well above a floor still wins on its height
                const float tie = (tri.triangleFlags.underlying() & kTriOverlap) ? kTieDz : 0.f;
                if (a_p.x >= minx - 0.01f && a_p.x <= maxx + 0.01f && a_p.y >= miny - 0.01f && a_p.y <= maxy + 0.01f && Inside2(a, b, c, a_p.x, a_p.y)) {
                    float z = (a.z + b.z + c.z) / 3.f;
                    PlaneZ(a, b, c, a_p.x, a_p.y, z);
                    const float dz = std::fabs(z - a_p.z);
                    if (dz <= kOnTriDz && dz + tie < L.bestOn) {
                        L.bestOn = dz + tie;
                        L.onPt = RE::NiPoint3{ a_p.x, a_p.y, z };
                        L.onS = si;
                        L.onT = tt;
                    }
                }
                if (join > 0.f) {
                    const RE::NiPoint3 q = Nearest2(a, b, c, a_p.x, a_p.y);
                    const float        d2 = std::hypot(q.x - a_p.x, q.y - a_p.y), dz = std::fabs(q.z - a_p.z);
                    if (d2 <= join && dz <= kJoinDz) {
                        const float score = d2 + dz * 0.5f + tie;
                        if (score < L.bestJoin) {
                            L.bestJoin = score;
                            L.joinPt = q;
                            L.joinS = si;
                            L.joinT = tt;
                        }
                    }
                }
            }
            L.tri = end;
            if (end >= sl.nTri) {
                ++L.k;
                L.tri = 0;
            }
            return 0;
        }
        L.begun = false;                               // every candidate scanned: the answer
        a_joined = false;
        if (L.onS >= 0) {
            a_slot = L.onS;
            a_tri = L.onT;
            a_on = L.onPt;
            return 1;
        }
        if (L.joinS >= 0) {
            a_slot = L.joinS;
            a_tri = L.joinT;
            a_on = L.joinPt;
            a_joined = true;
            return 1;
        }
        return -1;
    }

    bool InDisc(const Search& s, const RE::NiPoint3& a_p)
    {
        for (const auto& d : s.o.discs) {
            const float dx = a_p.x - d.x, dy = a_p.y - d.y;
            if (dx * dx + dy * dy < d.r * d.r && std::fabs(a_p.z - d.z) <= kDiscDz) return true;
        }
        return false;
    }

    bool NearBadLeg(const Search& s, const RE::NiPoint3& a_p)
    {
        for (const auto& b : s.bad) {
            if (SegDist2(a_p, b.a, b.b) <= kBadLegNear && std::fabs(a_p.z - (b.a.z + b.b.z) * 0.5f) <= kDiscDz) return true;
        }
        return false;
    }

    // a virtual link whose two ends match a remembered bad leg (each within 48 u) is never offered again
    bool LinkIsBad(const Search& s, const RE::NiPoint3& a_a, const RE::NiPoint3& a_b)
    {
        for (const auto& b : s.bad) {
            if ((Dist2(a_a, b.a) <= kBadLegNear && Dist2(a_b, b.b) <= kBadLegNear) || (Dist2(a_a, b.b) <= kBadLegNear && Dist2(a_b, b.a) <= kBadLegNear)) {
                return true;
            }
        }
        return false;
    }

    Node& NodeOf(Search& s, std::int32_t a_slot, std::uint16_t a_tri)
    {
        Slot& sl = s.slots[static_cast<std::size_t>(a_slot)];
        if (sl.nodes.size() != sl.nTri) sl.nodes.assign(sl.nTri, Node{});
        return sl.nodes[a_tri];
    }

    RE::NiPoint3 Centroid(const Slot& a_sl, std::uint16_t a_t)
    {
        const RE::BSNavmeshTriangle& t = Tri(a_sl, a_t);
        const RE::NiPoint3&          a = Vert(a_sl, t.vertices[0]);
        const RE::NiPoint3&          b = Vert(a_sl, t.vertices[1]);
        const RE::NiPoint3&          c = Vert(a_sl, t.vertices[2]);
        return { (a.x + b.x + c.x) / 3.f, (a.y + b.y + c.y) / 3.f, (a.z + b.z + c.z) / 3.f };
    }

    // ONE expansion of the A*: the triangle's 3 edges -> its neighbours in this navmesh, or across a portal into another LOADED one.
    // (Fix round, nit 7) a portal into a navmesh whose self-check has not finished stops it: false, s.pendSlot set - the check runs
    // its units first, then this expansion is done again (pushes it already made are not made twice: the same g is no improvement).
    bool Expand(Search& s, std::int32_t a_slot, std::uint16_t a_tri)
    {
        const Slot&                  sl = s.slots[static_cast<std::size_t>(a_slot)];
        const RE::BSNavmeshTriangle& t = Tri(sl, a_tri);
        const std::uint16_t          fl = t.triangleFlags.underlying();
        const Node                   cur = s.slots[static_cast<std::size_t>(a_slot)].nodes[a_tri];
        const RE::NiPoint3           curP{ cur.px, cur.py, cur.pz };
        for (int e = 0; e < 3; ++e) {
            const std::uint16_t va = t.vertices[e], vb = t.vertices[(e + 1) % 3];
            if (va >= sl.nVert || vb >= sl.nVert) continue;
            std::int32_t        ns = -1;
            std::uint16_t       nt = kNo16;
            const std::uint16_t raw = t.triangles[e];
            if (fl & (1u << e)) {                       // a portal to another navmesh - followed only into a LOADED one
                if (raw >= sl.nExtra) continue;
                const RE::BSNavmeshEdgeExtraInfo& ex = sl.mesh->extraEdgeInfo.data()[raw];
                if (ex.type.underlying() != kPortalType) continue;
                ns = SlotFor(s, ex.portal.otherMeshID);
                if (ns < 0) continue;
                if (!s.slots[static_cast<std::size_t>(ns)].Checked()) {     // its self-check first (one chunk a unit)
                    s.pendSlot = ns;
                    s.pendNodeSlot = a_slot;
                    s.pendNodeTri = a_tri;
                    return false;
                }
                nt = ex.portal.triangle;
            } else {
                if (raw == kNo16 || raw >= sl.nTri) continue;
                ns = a_slot;
                nt = raw;
            }
            const Slot& nsl = s.slots[static_cast<std::size_t>(ns)];
            if (!nsl.mesh || !Usable(nsl, nt)) continue;
            Node& nn = NodeOf(s, ns, nt);
            if (nn.closed) continue;
            float     mult = 1.f;
            const int door = DoorAt(nsl, nt);
            if (door == 0) continue;                    // a closed door she may not open: a wall
            if (door == 1) mult *= kCostDoor;
            const std::uint16_t nfl = Tri(nsl, nt).triangleFlags.underlying();
            if ((nfl & kTriWater) && !s.o.water) mult *= kCostWater;
            if (nfl & kTriPreferred) mult *= kCostPreferred;
            if (nfl & kTriOverlap) mult *= kCostOverlap;     // (check 6) bit 5 where it is no wall (1.6.1: Usable walls it in a cut navmesh)
            const RE::NiPoint3& A = Vert(sl, va);
            const RE::NiPoint3& B = Vert(sl, vb);
            const RE::NiPoint3  mid{ (A.x + B.x) * 0.5f, (A.y + B.y) * 0.5f, (A.z + B.z) * 0.5f };
            if (!s.o.discs.empty() && InDisc(s, Centroid(nsl, nt))) mult *= kCostDisc;
            if (!s.bad.empty() && NearBadLeg(s, mid)) mult *= kCostBadLeg;
            const float ng = cur.g + Dist3(mid, curP) * mult;
            if (ng < nn.g) {
                nn.g = ng;
                nn.px = mid.x;
                nn.py = mid.y;
                nn.pz = mid.z;
                nn.parentSlot = a_slot;
                nn.parentTri = a_tri;
                nn.via = static_cast<std::uint8_t>(e);
                nn.link = -1;
                s.open.push(Open{ ng + kHeur * Dist3(mid, s.goalPt), ng, ns, nt });
            }
        }
        return true;
    }

    std::uint64_t BucketKey(float a_x, float a_y)
    {
        const auto bx = static_cast<std::int32_t>(std::floor(a_x / kLinkMax)), by = static_cast<std::int32_t>(std::floor(a_y / kLinkMax));
        return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(bx)) << 32) | static_cast<std::uint32_t>(by);
    }

    // LINKS unit 1: the border edges of the triangles the search reached, bucketed by 256 u (fix round, nit 7: 4,096 triangles a unit;
    // true = this navmesh is done)
    bool LinkBorder(Search& s, std::size_t a_slot)
    {
        const Slot& sl = s.slots[a_slot];
        if (sl.bad || !sl.Checked() || !sl.mesh || sl.nodes.size() != sl.nTri) return true;
        const std::uint32_t end = (std::min)(sl.nTri, s.linkTri + kChunk);
        for (std::uint32_t i = s.linkTri; i < end; ++i) {
            if (!sl.nodes[i].closed) continue;
            const RE::BSNavmeshTriangle& t = Tri(sl, static_cast<std::uint16_t>(i));
            const std::uint16_t          fl = t.triangleFlags.underlying();
            for (int e = 0; e < 3; ++e) {
                if ((fl & (1u << e)) || t.triangles[e] != kNo16) continue;
                const RE::NiPoint3& A = Vert(sl, t.vertices[e]);
                const RE::NiPoint3& B = Vert(sl, t.vertices[(e + 1) % 3]);
                BorderEdge          be;
                be.slot = static_cast<std::int32_t>(a_slot);
                be.tri = static_cast<std::uint16_t>(i);
                be.mid = RE::NiPoint3{ (A.x + B.x) * 0.5f, (A.y + B.y) * 0.5f, (A.z + B.z) * 0.5f };
                s.borderGrid[BucketKey(be.mid.x, be.mid.y)].push_back(static_cast<std::uint32_t>(s.border.size()));
                s.border.push_back(be);
                s.bx0 = (std::min)(s.bx0, be.mid.x);
                s.by0 = (std::min)(s.by0, be.mid.y);
                s.bx1 = (std::max)(s.bx1, be.mid.x);
                s.by1 = (std::max)(s.by1, be.mid.y);
            }
        }
        s.linkTri = end;
        if (end < sl.nTri) return false;
        s.linkTri = 0;
        return true;
    }

    // LINKS unit 2: one navmesh's border edges of triangles NOT reached, within 256 u (2D) and 96 u (height) of a reached border
    // edge -> the shortest link into each such triangle (fix round, nit 7: its self-check chunk or 4,096 triangles a unit; true = this
    // navmesh is done)
    bool LinkPartners(Search& s, RE::FormID a_id)
    {
        if (!s.live.interior) {                        // its cell's square must come within 256 u (+ 64 u) of the reached border's box
            const auto lt = s.live.byId.find(a_id);
            if (lt == s.live.byId.end()) return true;
            const float x0 = static_cast<float>(lt->second.cx) * kCellU, y0 = static_cast<float>(lt->second.cy) * kCellU;
            const float pad = kLinkMax + 64.f;
            if (x0 + kCellU < s.bx0 - pad || x0 > s.bx1 + pad || y0 + kCellU < s.by0 - pad || y0 > s.by1 + pad) return true;
        }
        const std::int32_t si = SlotFor(s, a_id);
        if (si < 0) return true;
        Slot& slw = s.slots[static_cast<std::size_t>(si)];
        if (!slw.Checked()) {
            CheckUnit(s, slw);
            return false;
        }
        const Slot& sl = slw;
        if (sl.bad || !sl.mesh || sl.nTri == 0) return true;
        if (sl.x1 < s.bx0 - kLinkMax || sl.x0 > s.bx1 + kLinkMax || sl.y1 < s.by0 - kLinkMax || sl.y0 > s.by1 + kLinkMax) return true;
        const bool          hasNodes = sl.nodes.size() == sl.nTri;
        const std::uint32_t end = (std::min)(sl.nTri, s.linkTri + kChunk);
        for (std::uint32_t i = s.linkTri; i < end; ++i) {
            const auto tt = static_cast<std::uint16_t>(i);
            if (hasNodes && sl.nodes[i].closed) continue;
            if (!Usable(sl, tt) || DoorAt(sl, tt) == 0) continue;
            const RE::BSNavmeshTriangle& t = Tri(sl, tt);
            const std::uint16_t          fl = t.triangleFlags.underlying();
            for (int e = 0; e < 3; ++e) {
                if ((fl & (1u << e)) || t.triangles[e] != kNo16) continue;
                const RE::NiPoint3& A = Vert(sl, t.vertices[e]);
                const RE::NiPoint3& B = Vert(sl, t.vertices[(e + 1) % 3]);
                const RE::NiPoint3  mid{ (A.x + B.x) * 0.5f, (A.y + B.y) * 0.5f, (A.z + B.z) * 0.5f };
                if (mid.x < s.bx0 - kLinkMax || mid.x > s.bx1 + kLinkMax || mid.y < s.by0 - kLinkMax || mid.y > s.by1 + kLinkMax) continue;
                const auto bx = static_cast<std::int32_t>(std::floor(mid.x / kLinkMax)), by = static_cast<std::int32_t>(std::floor(mid.y / kLinkMax));
                for (int dx = -1; dx <= 1; ++dx) {
                    for (int dy = -1; dy <= 1; ++dy) {
                        const std::uint64_t key = (static_cast<std::uint64_t>(static_cast<std::uint32_t>(bx + dx)) << 32) |
                                                  static_cast<std::uint32_t>(by + dy);
                        const auto it = s.borderGrid.find(key);
                        if (it == s.borderGrid.end()) continue;
                        for (const std::uint32_t bi : it->second) {
                            const BorderEdge& be = s.border[bi];
                            if (be.slot == si && be.tri == tt) continue;
                            const float d2 = Dist2(be.mid, mid), dz = std::fabs(be.mid.z - mid.z);
                            if (d2 > kLinkMax || dz > kLinkDz || d2 < 1.f) continue;
                            if (LinkIsBad(s, be.mid, mid)) continue;
                            const std::uint64_t tk = (static_cast<std::uint64_t>(static_cast<std::uint32_t>(si)) << 16) | tt;
                            const float         len = std::hypot(d2, dz);
                            auto                lt = s.linkBest.find(tk);
                            if (lt == s.linkBest.end() || len < lt->second.len) {
                                VLink l;
                                l.fs = be.slot;
                                l.ft = be.tri;
                                l.ts = si;
                                l.tt = tt;
                                l.a = be.mid;
                                l.b = mid;
                                l.len = len;
                                s.linkBest[tk] = l;
                            }
                        }
                    }
                }
            }
        }
        s.linkTri = end;
        if (end < sl.nTri) return false;
        s.linkTri = 0;
        return true;
    }

    // LINKS unit 3: the shortest links (at most 256) are offered to the A* - each target triangle entered from its reached border
    // triangle at cost x2; the A* goes on
    void LinkOffer(Search& s)
    {
        std::vector<VLink> cand;
        cand.reserve(s.linkBest.size());
        for (const auto& [k, l] : s.linkBest) cand.push_back(l);
        std::sort(cand.begin(), cand.end(), [](const VLink& l, const VLink& r) { return l.len < r.len; });
        if (cand.size() > kLinksMax) cand.resize(kLinksMax);
        int offered = 0;
        for (const auto& l : cand) {
            const Node from = NodeOf(s, l.fs, l.ft);
            if (!from.closed) continue;
            Node&       to = NodeOf(s, l.ts, l.tt);
            const float ng = from.g + Dist3(RE::NiPoint3{ from.px, from.py, from.pz }, l.a) + l.len * kCostLink;
            if (to.closed || ng >= to.g) continue;
            to.g = ng;
            to.px = l.b.x;
            to.py = l.b.y;
            to.pz = l.b.z;
            to.parentSlot = l.fs;
            to.parentTri = l.ft;
            to.via = 3;
            to.link = static_cast<std::int32_t>(s.links.size());
            s.links.push_back(l);
            s.open.push(Open{ ng + kHeur * Dist3(l.b, s.goalPt), ng, l.ts, l.tt });
            ++offered;
        }
        s.linkBest.clear();
        s.border.clear();
        s.borderGrid.clear();
        s.linkMeshes.clear();
        if (offered == 0) {
            Finish(s, 0, std::format("no way on the navmesh: the loaded triangles round her ({} reached) do not reach the goal's, and no "
                                     "island edge lies within {:.0f} u of hers{}",
                                     s.info.expanded, kLinkMax, s.linkRound > 1 ? " (after virtual links)" : ""));
        }
    }

    // THE CHAIN (one unit, the last engine reads): the triangles of the way from her start to the goal, their corners, and the
    // edges crossed (left / right as she walks them, narrowed by her clearance); a virtual link is a break between two runs
    bool BuildChain(Search& s)
    {
        std::vector<std::pair<std::int32_t, std::uint16_t>> rev;
        std::int32_t                                        cs = s.goalSlot;
        std::uint16_t                                       ct = s.goalTri;
        while (cs >= 0 && rev.size() <= static_cast<std::size_t>(kMaxExpand)) {
            rev.emplace_back(cs, ct);
            const Node& n = s.slots[static_cast<std::size_t>(cs)].nodes[ct];
            if (n.via == 0xFF) break;
            const std::int32_t ps = n.parentSlot;
            const std::uint16_t pt = n.parentTri;
            cs = ps;
            ct = pt;
        }
        if (rev.empty() || rev.back().first != s.startSlot || rev.back().second != s.startTri) return false;
        std::reverse(rev.begin(), rev.end());
        s.chain.clear();
        s.portals.clear();
        s.chainLinks.clear();
        const float rc = std::clamp(s.o.clearance, 0.f, 64.f);
        Portal      first;
        first.l = first.r = s.startPt;
        first.tri = 0;
        s.portals.push_back(first);
        for (std::size_t i = 0; i < rev.size(); ++i) {
            const Slot&                  sl = s.slots[static_cast<std::size_t>(rev[i].first)];
            const RE::BSNavmeshTriangle& t = Tri(sl, rev[i].second);
            ChainTri                     c;
            for (int k = 0; k < 3; ++k) c.v[static_cast<std::size_t>(k)] = Vert(sl, t.vertices[k]);
            c.flags = t.triangleFlags.underlying();
            c.door = DoorAt(sl, rev[i].second) == 1;
            s.chain.push_back(c);
            if (i == 0) continue;
            const Node& n = sl.nodes[rev[i].second];
            if (n.via == 3) {                           // a virtual link: the run ends at its near end, the next starts at its far end
                if (n.link < 0 || static_cast<std::size_t>(n.link) >= s.links.size()) return false;
                const VLink& l = s.links[static_cast<std::size_t>(n.link)];
                Portal       a;
                a.l = a.r = l.a;
                a.tri = i - 1;
                a.brk = true;
                s.portals.push_back(a);
                Portal b;
                b.l = b.r = l.b;
                b.tri = i;
                s.portals.push_back(b);
                s.chainLinks.emplace_back(l.a, l.b);
                continue;
            }
            const Slot&                  psl = s.slots[static_cast<std::size_t>(rev[i - 1].first)];
            const RE::BSNavmeshTriangle& pt = Tri(psl, rev[i - 1].second);
            const std::uint8_t           e = n.via;
            if (e > 2) return false;
            RE::NiPoint3 A = Vert(psl, pt.vertices[e]), B = Vert(psl, pt.vertices[(e + 1) % 3]);
            // left / right as she walks from the parent into this triangle: cross(B - A, travel) > 0 -> A lies on her left
            const RE::NiPoint3 cp = Centroid(psl, rev[i - 1].second), cc = Centroid(sl, rev[i].second);
            const RE::NiPoint3 d = cc - cp;
            const float        side = (B.x - A.x) * d.y - (B.y - A.y) * d.x;
            Portal             p;
            p.tri = i;
            if (side >= 0.f) {
                p.l = A;
                p.r = B;
            } else {
                p.l = B;
                p.r = A;
            }
            const float len = Dist3(p.l, p.r);
            if (len > 2.f * rc + 0.01f && len > 1e-3f) {      // narrowed by her clearance at both ends ...
                const RE::NiPoint3 u = (p.r - p.l) * (1.f / len);
                p.l = p.l + u * rc;
                p.r = p.r - u * rc;
            } else {                                          // ... a narrower edge keeps its middle (the rays decide)
                const RE::NiPoint3 m = (p.l + p.r) * 0.5f;
                p.l = p.r = m;
            }
            s.portals.push_back(p);
        }
        Portal last;
        last.l = last.r = s.goalPt;
        last.tri = s.chain.size() - 1;
        s.portals.push_back(last);
        return true;
    }

    // the funnel (string pull) over one run of portals [a_first .. a_last] (the first = the run's start, the last = its end, both
    // points): the corners after the start, the end last; a_idx = each one's portal
    void Funnel(const std::vector<Portal>& P, std::size_t a_first, std::size_t a_last, std::vector<RE::NiPoint3>& a_out,
                std::vector<std::size_t>& a_idx)
    {
        RE::NiPoint3 apex = P[a_first].l, left = P[a_first].l, right = P[a_first].r;
        std::size_t  apexI = a_first, leftI = a_first, rightI = a_first;
        std::size_t  guard = 0;
        const std::size_t guardMax = 64 * (a_last - a_first + 1) + 256;   // (each corner re-scans from itself: bounded, never a hang)
        std::size_t  i = a_first + 1;
        for (; i <= a_last; ++i) {
            if (++guard > guardMax) break;
            const RE::NiPoint3& l = P[i].l;
            const RE::NiPoint3& r = P[i].r;
            // the right side: r tightens the funnel when it lies left of (or on) apex -> right
            if (Cross2(apex, right, r) >= 0.f) {
                if (Same2(apex, right) || Cross2(apex, left, r) < 0.f) {
                    right = r;
                    rightI = i;
                } else {                                // r crosses over the left side: the left point is a corner
                    a_out.push_back(left);
                    a_idx.push_back(leftI);
                    apex = left;
                    apexI = leftI;
                    left = right = apex;
                    leftI = rightI = apexI;
                    i = apexI;
                    continue;
                }
            }
            // the left side: l tightens the funnel when it lies right of (or on) apex -> left
            if (Cross2(apex, left, l) <= 0.f) {
                if (Same2(apex, left) || Cross2(apex, right, l) > 0.f) {
                    left = l;
                    leftI = i;
                } else {                                // l crosses over the right side: the right point is a corner
                    a_out.push_back(right);
                    a_idx.push_back(rightI);
                    apex = right;
                    apexI = rightI;
                    left = right = apex;
                    leftI = rightI = apexI;
                    i = apexI;
                    continue;
                }
            }
        }
        if (i <= a_last) {                              // (the guard) the rest as the crossed edges' middles - the planner's pull straightens them
            for (std::size_t k = apexI + 1; k < a_last; ++k) {
                a_out.push_back((P[k].l + P[k].r) * 0.5f);
                a_idx.push_back(k);
            }
        }
        a_out.push_back(P[a_last].l);
        a_idx.push_back(a_last);
    }

    // the floor's height (the navmesh's plane) at (x, y) among the chain's triangles [a_lo .. a_hi]; false = none holds it
    bool ChainZ(const Search& s, std::size_t a_lo, std::size_t a_hi, float a_x, float a_y, float a_zHint, float& a_z)
    {
        bool  found = false;
        float best = 3.0e38f;
        for (std::size_t i = a_lo; i <= a_hi && i < s.chain.size(); ++i) {
            const ChainTri& c = s.chain[i];
            if (!Inside2(c.v[0], c.v[1], c.v[2], a_x, a_y)) continue;
            float z = 0.f;
            if (!PlaneZ(c.v[0], c.v[1], c.v[2], a_x, a_y, z)) continue;
            if (std::fabs(z - a_zHint) < best) {
                best = std::fabs(z - a_zHint);
                a_z = z;
                found = true;
            }
        }
        return found;
    }

    // THE FUNNEL (no engine data): each run between virtual links string-pulled, straight stretches cut every <= 256 u (their z from
    // the triangles crossed), the flags; then the joins and the goal
    void BuildPoints(Search& s)
    {
        std::vector<RE::NiPoint3> corners;
        std::vector<std::size_t>  idx;
        std::vector<std::uint32_t> kinds;
        std::size_t runStart = 0;
        for (std::size_t i = 1; i < s.portals.size(); ++i) {
            if (!s.portals[i].brk && i + 1 != s.portals.size()) continue;
            std::vector<RE::NiPoint3> c;
            std::vector<std::size_t>  ci;
            Funnel(s.portals, runStart, i, c, ci);
            for (std::size_t k = 0; k < c.size(); ++k) {
                corners.push_back(c[k]);
                idx.push_back(ci[k]);
                kinds.push_back(kSkCorner);
            }
            if (s.portals[i].brk) {                    // the link's far end starts the next run
                corners.push_back(s.portals[i + 1].l);
                idx.push_back(i + 1);
                kinds.push_back(kSkVirtual);
                runStart = i + 1;
                ++i;
            }
        }
        // cut the long stretches; the points' flags (water: the triangle it lies on; door: a closed-door triangle crossed since the
        // point before)
        RE::NiPoint3 prev = s.startPt;
        std::size_t  prevI = 0;
        for (std::size_t k = 0; k < corners.size(); ++k) {
            const RE::NiPoint3 q = corners[k];
            const std::size_t  qi = idx[k];
            const std::size_t  loTri = s.portals[prevI].tri, hiTri = s.portals[qi].tri;
            const float        L = Dist2(prev, q);
            if (!(kinds[k] & kSkVirtual) && L > kMaxStraight) {
                const int n = static_cast<int>(std::ceil(L / kMaxStraight));
                for (int m = 1; m < n; ++m) {
                    const float  t = static_cast<float>(m) / static_cast<float>(n);
                    RE::NiPoint3 p{ prev.x + (q.x - prev.x) * t, prev.y + (q.y - prev.y) * t, prev.z + (q.z - prev.z) * t };
                    float        z = p.z;
                    if (ChainZ(s, loTri > 0 ? loTri - 1 : 0, hiTri + 1, p.x, p.y, p.z, z)) p.z = z;
                    SkelPoint sp;
                    sp.pos = p;
                    sp.flags = kSkCut;
                    s.pts.push_back(sp);
                }
            }
            SkelPoint sp;
            sp.pos = q;
            sp.flags = kinds[k];
            for (std::size_t t = loTri; t <= hiTri && t < s.chain.size(); ++t) {
                if (s.chain[t].door) sp.flags |= kSkDoor;
            }
            if (hiTri < s.chain.size() && (s.chain[hiTri].flags & kTriWater)) sp.flags |= kSkWater;
            if (Dist3(q, prev) >= kDupe || (sp.flags & kSkVirtual)) s.pts.push_back(sp);
            prev = q;
            prevI = qi;
        }
        // the joins: her start off the navmesh -> the skeleton starts at its join point; the goal off it -> its join point, then the
        // goal itself. The goal point is always the asked one (kSkGoal).
        if (s.info.startJoined && Dist3(s.startPt, s.from) >= kDupe) {
            SkelPoint j0;
            j0.pos = s.startPt;
            j0.flags = kSkJoin;
            s.pts.insert(s.pts.begin(), j0);
        }
        if (!s.pts.empty() && Dist3(s.pts.back().pos, s.goalPt) < kDupe) {
            if (s.info.goalJoined) {
                s.pts.back().flags |= kSkJoin;
            } else {
                s.pts.pop_back();                      // the goal itself follows
            }
        }
        SkelPoint g;
        g.pos = s.to;
        g.flags = kSkGoal;
        if (!s.pts.empty() && Dist3(s.pts.back().pos, s.to) < kDupe) s.pts.pop_back();
        s.pts.push_back(g);
        float        len = 0.f;
        RE::NiPoint3 lp = s.from;
        for (const auto& p : s.pts) {
            len += Dist3(lp, p.pos);
            lp = p.pos;
        }
        s.info.length = len;
        s.info.links = static_cast<int>(s.chainLinks.size());
    }

    void SelfCheckLine(const Search& s, bool a_failed)
    {
        bool first = false;
        {
            std::scoped_lock lk(g_memMtx);
            first = g_selfLogged.insert(s.walker).second;
        }
        if (!first && !a_failed) return;
        int tris = 0, verts = 0, links = 0, deleted = 0, overlap = 0, doors = 0, walls = 0, unread = 0, checked = 0;
        int cutMeshes = 0, cutWalls = 0;
        for (const auto& sl : s.slots) {
            tris += static_cast<int>(sl.nTri);
            verts += static_cast<int>(sl.nVert);
            links += sl.links;
            deleted += sl.deleted;
            overlap += sl.overlap;
            doors += static_cast<int>(sl.doorTri.size());
            walls += sl.doorsWall;
            unread += sl.doorsUnread ? 1 : 0;
            checked += sl.Checked() ? 1 : 0;
            if (sl.cutWalls) {                    // (1.6.1 A1)
                ++cutMeshes;
                cutWalls += sl.overlap;
            }
        }
        const std::string tag = s.id ? std::format("SKELETON #{}{}", s.id, s.o.tag ? std::format(" for #{}", s.o.tag) : std::string()) :
                                       std::format("PATH #{}", s.o.tag);
        // (fix round, check 6) bit 3 (deleted: walls) and bit 5 apart; (1.6.1 A1) bit 5 = walls in the navmeshes with runtime cuts, x6 elsewhere
        logger::info("SKELETON self-check {:08X} ({}){}: {} navmesh(es) read ({} fully checked) - {} triangle(s), {} vertices, {} edge link(s), {} "
                     "bit-3 'deleted' triangle(s) (walls), {} bit-5 'overlapping' triangle(s) ({} walls: navcut-retired in {} navmesh(es) with "
                     "runtime cuts; the rest cost x6), {} closed door triangle(s) ({} a wall){} - {}",
                     s.walker, tag, first ? " (once per walker per game run)" : "", s.slots.size(), checked, tris, verts, links, deleted, overlap,
                     cutWalls, cutMeshes, doors, walls, unread ? std::format(", {} door list(s) not read (not believable)", unread) : std::string(),
                     s.badMeshes ? std::format("{} navmesh(es) FAILED (first: {}){}", s.badMeshes, s.badWhy,
                                               a_failed ? " - the plan uses the 1.5.x grid corridor" : " - left out (borders)") :
                                   std::string("OK"));
    }

    const char* Tag(const Search& s, std::string& a_buf)
    {
        a_buf = s.id ? std::format("SKELETON #{}{}", s.id, s.o.tag ? std::format(" (for #{})", s.o.tag) : std::string()) :
                       std::format("PATH #{} SKELETON", s.o.tag);
        return a_buf.c_str();
    }

    // (1.6.1 A1) which rule a navmesh's bit-5 triangles got, with the evidence - once per navmesh per game run (again when its
    // triangle count, its bit-5 count or its rule changes: a new cut swapped in)
    void RuleLine(const Search& s, const Slot& a_sl)
    {
        if (a_sl.bad || a_sl.overlap == 0) return;
        const std::uint64_t sig = (static_cast<std::uint64_t>(a_sl.nTri) << 33) | (static_cast<std::uint64_t>(a_sl.overlap) << 1) |
                                  (a_sl.cutWalls ? 1u : 0u);
        {
            std::scoped_lock lk(g_memMtx);
            auto [it, fresh] = g_ruleLogged.try_emplace(a_sl.id, sig);
            if (!fresh) {
                if (it->second == sig) return;
                it->second = sig;
            }
        }
        std::string buf;
        logger::info("SKELETON navmesh {:08X} ({}): {} bit-5 triangle(s) of {} - {}; evidence: {} cut record(s){}, {} one-way link(s) into "
                     "bit-5 triangles, {} cut-made triangle(s)",
                     a_sl.id, Tag(s, buf), a_sl.overlap, a_sl.nTri,
                     a_sl.cutWalls ? "RUNTIME CUTS here: bit 5 = a WALL (navcut-retired - never entered, never located on)" :
                                     "no runtime cut seen: bit 5 = cost x6 (the CK's Overlapping flag)",
                     a_sl.obstacles, a_sl.obstaclesOdd ? " (its count not believable: not used)" : "", a_sl.oneWay, a_sl.cutMade);
    }

    // the work of one step until a_deadline (the slots are resolved; the walker is loaded)
    void Work(Search& s, Clock::time_point a_deadline)
    {
        while (!s.done) {
            if (Clock::now() >= a_deadline) return;
            switch (s.stage) {
            case 0: {                                  // her start (resumable: one chunk a unit)
                bool      joined = false;
                const int r = LocateStep(s, s.from, s.startSlot, s.startTri, s.startPt, joined);
                if (r == 0) break;
                if (r < 0) {
                    if (s.badMeshes && s.slots.size() == static_cast<std::size_t>(s.badMeshes)) {
                        s.info.selfCheckFailed = true;
                        SelfCheckLine(s, true);
                        Finish(s, -1, "the navmesh data could not be read as expected (self-check): " + s.badWhy);
                        return;
                    }
                    Finish(s, 0, std::format("her start ({:.0f},{:.0f},{:.0f}) is not within {:.0f} u of the loaded navmesh", s.from.x, s.from.y,
                                             s.from.z, s.o.joinMax));
                    return;
                }
                s.info.startJoined = joined;
                s.stage = 1;
                break;
            }
            case 1: {                                  // the goal (resumable)
                bool      joined = false;
                const int r = LocateStep(s, s.to, s.goalSlot, s.goalTri, s.goalPt, joined);
                if (r == 0) break;
                if (r < 0) {
                    Finish(s, 0, std::format("the goal ({:.0f},{:.0f},{:.0f}) is not within {:.0f} u of the loaded navmesh", s.to.x, s.to.y, s.to.z,
                                             s.o.joinMax));
                    return;
                }
                s.info.goalJoined = joined;
                Node& sn = NodeOf(s, s.startSlot, s.startTri);
                sn.g = 0.f;
                sn.px = s.startPt.x;
                sn.py = s.startPt.y;
                sn.pz = s.startPt.z;
                sn.via = 0xFF;
                s.open.push(Open{ kHeur * Dist3(s.startPt, s.goalPt), 0.f, s.startSlot, s.startTri });
                s.stage = 2;
                break;
            }
            case 2: {                                  // the A*: up to 16 pops, then the clock
                if (s.pendSlot >= 0) {                 // (nit 7) a navmesh it reached finishes its self-check first, one chunk a unit
                    if (!CheckUnit(s, s.slots[static_cast<std::size_t>(s.pendSlot)])) break;
                    s.pendSlot = -1;
                    Expand(s, s.pendNodeSlot, s.pendNodeTri);     // the expansion that met it, again (it may stop at another one)
                    break;
                }
                for (int k = 0; k < kCheckEvery && s.stage == 2 && s.pendSlot < 0; ++k) {
                    if (s.open.empty()) {
                        if (!s.o.links || s.linkRound >= kLinkRounds) {
                            Finish(s, 0, std::format("no way on the navmesh: {} triangle(s) reached, none of them the goal's{}", s.info.expanded,
                                                     s.linkRound ? std::format(" (after {} round(s) of virtual links)", s.linkRound) : std::string()));
                            return;
                        }
                        ++s.linkRound;
                        s.linkStage = 0;
                        s.linkK = 0;
                        s.linkTri = 0;
                        s.border.clear();
                        s.borderGrid.clear();
                        s.linkBest.clear();
                        s.bx0 = s.by0 = 3.0e38f;
                        s.bx1 = s.by1 = -3.0e38f;
                        s.stage = 3;
                        break;
                    }
                    const Open top = s.open.top();
                    s.open.pop();
                    Node& nd = NodeOf(s, top.slot, top.tri);
                    if (nd.closed || top.g > nd.g + 0.01f) continue;
                    nd.closed = true;
                    ++s.info.expanded;
                    if (top.slot == s.goalSlot && top.tri == s.goalTri) {
                        s.stage = 4;
                        break;
                    }
                    if (s.info.expanded >= kMaxExpand) {
                        Finish(s, 0, std::format("the skeleton search ran out ({} triangles expanded)", s.info.expanded));
                        return;
                    }
                    Expand(s, top.slot, top.tri);
                }
                break;
            }
            case 3: {                                  // virtual links: the reached border, the other islands, the offer (4,096 a unit)
                if (s.linkStage == 0) {
                    if (s.linkK < s.slots.size()) {
                        if (LinkBorder(s, s.linkK)) ++s.linkK;
                        break;
                    }
                    s.linkStage = 1;
                    s.linkK = 0;
                    s.linkTri = 0;
                    s.linkMeshes.clear();
                    for (const auto& [fid, lm] : s.live.byId) s.linkMeshes.push_back(fid);
                    std::sort(s.linkMeshes.begin(), s.linkMeshes.end());
                    if (s.border.empty()) {
                        Finish(s, 0, std::format("no way on the navmesh: {} triangle(s) reached, an island with no open edge", s.info.expanded));
                        return;
                    }
                    break;
                }
                if (s.linkK < s.linkMeshes.size()) {
                    if (LinkPartners(s, s.linkMeshes[s.linkK])) ++s.linkK;
                    break;
                }
                LinkOffer(s);
                if (!s.done) s.stage = 2;
                break;
            }
            case 4: {                                  // the chain: the last engine reads
                if (!BuildChain(s)) {
                    Finish(s, -1, "the way's chain of triangles could not be followed back (internal)");
                    return;
                }
                s.stage = 5;
                break;
            }
            case 5: {                                  // the funnel
                BuildPoints(s);
                if (!s.slots.empty()) SelfCheckLine(s, false);
                Finish(s, 1, std::format("a way over {} triangle(s)", s.chain.size()));
                return;
            }
            default:
                Finish(s, -1, "internal: an unknown stage");
                return;
            }
        }
    }

    // ---- its own jobs (BuildSkeleton) -------------------------------------------------------------------------------------
    struct Job {
        std::int32_t  handle{ 0 };
        SearchPtr     s;
        bool          taskQueued{ false };
        bool          finished{ false };
        std::uint64_t order{ 0 };
    };

    std::mutex                                             g_navMtx;     // the jobs (natives / WalkApi / the pump threads / the steps)
    std::unordered_map<std::int32_t, std::unique_ptr<Job>> g_navJobs;
    std::int32_t                                           g_navNext = 1;
    std::uint64_t                                          g_navOrder = 0;
    std::atomic<int>                                       g_navRunning{ 0 };

    void LogDone(const Search& s)
    {
        std::string tag;
        std::string pts;
        for (const auto& p : s.pts) {
            pts += std::format(" ({:.0f},{:.0f},{:.0f}{}{}{}{}{}{})", p.pos.x, p.pos.y, p.pos.z, (p.flags & kSkVirtual) ? " v" : "",
                               (p.flags & kSkCut) ? " c" : "", (p.flags & kSkDoor) ? " d" : "", (p.flags & kSkWater) ? " w" : "",
                               (p.flags & kSkJoin) ? " j" : "", (p.flags & kSkGoal) ? " g" : "");
        }
        logger::info("{}: {} [{}] - {} point(s){} | {:.0f} u, {} triangle(s) expanded in {} navmesh(es), {} virtual link(s), {} restart(s), {} step(s), "
                     "{:.2f} ms in all, longest step {:.2f} ms{}{}",
                     Tag(s, tag), s.info.status == 1 ? "BUILT" : s.info.status == 0 ? "NONE" : "FAILED", s.info.why, s.pts.size(), pts,
                     s.info.length, s.info.expanded, s.info.meshes, s.info.links, s.info.restarts, s.info.slices, s.info.ms, s.info.msMax,
                     s.info.startJoined ? " | her start joined the navmesh" : "", s.info.goalJoined ? " | the goal joined the navmesh" : "");
    }

    void Step(std::int32_t a_handle)
    {
        std::scoped_lock lk(g_navMtx);
        const auto       it = g_navJobs.find(a_handle);
        if (it == g_navJobs.end()) return;
        Job& j = *it->second;
        j.taskQueued = false;
        if (j.finished || !j.s) return;
        // (fix round, nit 14) ONE frame budget for all running skeleton jobs together (0.25 ms), each its share (>= 0.05 ms)
        const int    running = (std::max)(1, g_navRunning.load());
        const double share = (std::max)(kJobMinUs, kJobFrameUs / static_cast<double>(running));
        const bool   done = SearchStep(*j.s, Clock::now() + std::chrono::microseconds(static_cast<long long>(share)));
        if (done) {
            j.finished = true;
            --g_navRunning;
            LogDone(*j.s);
        }
    }

    // a small thread per job: about once a frame (~8 ms) it hands the main thread ONE step task, never a second before the first ran
    bool Pump(std::int32_t a_handle)
    {
        try {
            std::thread([a_handle]() {
                for (int n = 0; n < kPumpTicks; ++n) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(8));
                    bool queue = false;
                    {
                        std::scoped_lock lk(g_navMtx);
                        const auto       it = g_navJobs.find(a_handle);
                        if (it == g_navJobs.end() || it->second->finished) return;
                        if (!it->second->taskQueued) {
                            it->second->taskQueued = true;
                            queue = true;
                        }
                    }
                    if (queue) {
                        auto* task = SKSE::GetTaskInterface();
                        if (!task) return;
                        task->AddTask([a_handle]() { Step(a_handle); });
                    }
                }
                std::scoped_lock lk(g_navMtx);
                const auto       it = g_navJobs.find(a_handle);
                if (it != g_navJobs.end() && !it->second->finished && it->second->s) {
                    Finish(*it->second->s, -1, "the skeleton job outlived its pump (~60 s)");
                    it->second->finished = true;
                    --g_navRunning;
                    LogDone(*it->second->s);
                }
            }).detach();
            return true;
        } catch (...) {
            return false;
        }
    }
}  // namespace

SearchPtr SearchBegin(RE::Actor* a_walker, const RE::NiPoint3& a_from, const RE::NiPoint3& a_to, const SkelOptions& a_o)
{
    SearchPtr s(new Search);
    s->o = a_o;
    s->from = a_from;
    s->to = a_to;
    s->t0 = Clock::now();
    s->slots.reserve(kMaxSlots);
    s->bx0 = s->by0 = 3.0e38f;
    s->bx1 = s->by1 = -3.0e38f;
    if (!a_walker || !a_walker->Is3DLoaded() || !a_walker->GetParentCell()) {
        Finish(*s, -1, "the walker is not loaded");
        return s;
    }
    if (!std::isfinite(a_from.x) || !std::isfinite(a_from.y) || !std::isfinite(a_from.z) || !std::isfinite(a_to.x) || !std::isfinite(a_to.y) ||
        !std::isfinite(a_to.z)) {
        Finish(*s, -1, "not a finite point");
        return s;
    }
    s->walker = a_walker->GetFormID();
    // the remembered bad legs near this search (its start / goal box + 1,024 u)
    const float x0 = (std::min)(a_from.x, a_to.x) - 1024.f, x1 = (std::max)(a_from.x, a_to.x) + 1024.f;
    const float y0 = (std::min)(a_from.y, a_to.y) - 1024.f, y1 = (std::max)(a_from.y, a_to.y) + 1024.f;
    {
        std::scoped_lock lk(g_memMtx);
        const auto       now = Clock::now();
        g_badLegs.erase(std::remove_if(g_badLegs.begin(), g_badLegs.end(), [&now](const BadLeg& b) { return b.until <= now; }), g_badLegs.end());
        for (const auto& b : g_badLegs) {
            const float mx = (b.a.x + b.b.x) * 0.5f, my = (b.a.y + b.b.y) * 0.5f;
            if (mx >= x0 && mx <= x1 && my >= y0 && my <= y1) s->bad.push_back(b);
        }
    }
    return s;
}

bool SearchStep(Search& a_s, std::chrono::steady_clock::time_point a_deadline)
{
    if (a_s.done) return true;
    const auto t0 = Clock::now();
    ++a_s.info.slices;
    auto* w = RE::TESForm::LookupByID<RE::Actor>(a_s.walker);
    if (!w || !w->Is3DLoaded() || !w->GetParentCell()) {
        Finish(a_s, -1, "the walker is gone or no longer loaded");
    } else if (!CollectLive(w, a_s.live) || (a_s.live.byId.empty() && a_s.live.rejected > 0)) {
        // her cell / its navmeshes are not what we read them as (the vtable check): the self-check fails - the 1.5.x corridor
        a_s.info.selfCheckFailed = true;
        Finish(a_s, -1, a_s.live.rejected ? std::format("{} navmesh object(s) did not carry the game's NavMesh vtable (self-check)", a_s.live.rejected)
                                          : std::string("her cell could not be read as a cell (self-check)"));
    } else {
        if (!ResolveSlots(a_s)) {                      // the loaded area changed under the search (a cell came / went, a navcut)
            if (++a_s.info.restarts > kMaxRestarts) {
                Finish(a_s, -1, "the loaded navmeshes kept changing under the search");
            } else {
                Restart(a_s);
            }
        }
        if (!a_s.done) Work(a_s, a_deadline);
    }
    // nothing engine-side outlives the step: the smart pointers go, every slot forgets its object
    a_s.live.hold.clear();
    a_s.live.byId.clear();
    for (auto& sl : a_s.slots) sl.mesh = nullptr;
    const double ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
    a_s.info.ms += ms;
    a_s.info.msMax = (std::max)(a_s.info.msMax, ms);
    return a_s.done;
}

void SearchResult(const Search& a_s, SkelInfo& a_info, std::vector<SkelPoint>& a_pts)
{
    a_info = a_s.info;
    a_pts = a_s.pts;
}

std::int32_t BuildSkeleton(RE::Actor* a_walker, const RE::NiPoint3& a_from, const RE::NiPoint3& a_to, const SkelOptions& a_o)
{
    if (!a_walker || !a_walker->Is3DLoaded() || !a_walker->GetParentCell()) return 0;
    SearchPtr s = SearchBegin(a_walker, a_from, a_to, a_o);
    if (!s->done) {
        std::string why;
        if (!GoalLoaded(a_walker, a_to, &why)) {
            s->info.outsideLoaded = true;
            Finish(*s, 0, why);
        }
    }
    std::int32_t h = 0;
    {
        std::scoped_lock lk(g_navMtx);
        if (g_navJobs.size() >= static_cast<std::size_t>(kMaxJobs)) {
            std::int32_t  oldest = 0;
            std::uint64_t oldestOrder = UINT64_MAX;
            for (const auto& [k, v] : g_navJobs) {
                if (v->finished && v->order < oldestOrder) {
                    oldest = k;
                    oldestOrder = v->order;
                }
            }
            if (!oldest) return 0;
            g_navJobs.erase(oldest);
        }
        h = g_navNext++;
        if (g_navNext > 0x00FFFFFF) g_navNext = 1;
        auto job = std::make_unique<Job>();
        job->handle = h;
        job->order = ++g_navOrder;
        s->id = h;
        job->finished = s->done;
        job->s = std::move(s);
        if (!job->finished) ++g_navRunning;
        g_navJobs[h] = std::move(job);
    }
    logger::info("SKELETON #{} asked{}: {:08X} from ({:.0f},{:.0f},{:.0f}) to ({:.0f},{:.0f},{:.0f}), clearance {:.0f} u, joins <= {:.0f} u{}{}", h,
                 a_o.tag ? std::format(" (for #{})", a_o.tag) : std::string(), a_walker->GetFormID(), a_from.x, a_from.y, a_from.z, a_to.x, a_to.y,
                 a_to.z, a_o.clearance, a_o.joinMax, a_o.water ? ", water triangles allowed" : "", a_o.links ? ", virtual links on" : "");
    bool finishedNow = false;
    {
        std::scoped_lock lk(g_navMtx);
        const auto       it = g_navJobs.find(h);
        finishedNow = it != g_navJobs.end() && it->second->finished;
        if (finishedNow && it->second->s) LogDone(*it->second->s);
    }
    if (!finishedNow && !Pump(h)) {
        std::scoped_lock lk(g_navMtx);
        const auto       it = g_navJobs.find(h);
        if (it != g_navJobs.end() && !it->second->finished && it->second->s) {
            Finish(*it->second->s, -1, "no thread could be started for the skeleton job");
            it->second->finished = true;
            --g_navRunning;
            LogDone(*it->second->s);
        }
    }
    return h;
}

bool GetSkeleton(std::int32_t a_handle, SkelInfo& a_info, std::vector<SkelPoint>& a_pts)
{
    std::scoped_lock lk(g_navMtx);
    const auto       it = g_navJobs.find(a_handle);
    if (it == g_navJobs.end() || !it->second->s) {
        a_info = SkelInfo{};
        a_info.status = -2;
        a_info.why = "no such skeleton job (unknown handle, dropped by a load, Cancel or the 16-job limit)";
        a_pts.clear();
        return true;
    }
    if (!it->second->finished) return false;
    SearchResult(*it->second->s, a_info, a_pts);
    return true;
}

void Cancel(std::int32_t a_handle)
{
    std::scoped_lock lk(g_navMtx);
    const auto       it = g_navJobs.find(a_handle);
    if (it == g_navJobs.end()) return;
    if (!it->second->finished) --g_navRunning;
    g_navJobs.erase(it);
}

void ClearAll()
{
    {
        std::scoped_lock lk(g_navMtx);
        g_navJobs.clear();
        g_navRunning = 0;
    }
    std::scoped_lock lk(g_memMtx);
    g_badLegs.clear();
    g_selfLogged.clear();
    g_ruleLogged.clear();
}

int RunningJobs() { return (std::max)(0, g_navRunning.load()); }

bool GoalLoaded(RE::Actor* a_walker, const RE::NiPoint3& a_p, std::string* a_why)
{
    // (B uses a_why word for word as the O 26 answer: ONE short plain sentence)
    const auto say = [&]() {
        if (!a_why) return;
        float d = 0.f;
        if (auto* pl = RE::PlayerCharacter::GetSingleton()) d = std::hypot(pl->GetPositionX() - a_p.x, pl->GetPositionY() - a_p.y);
        *a_why = std::format("the point is outside the loaded area, {:.0f} m from the player - GoToPoint can travel there", d / kUnitsPerM);
    };
    if (!a_walker || !std::isfinite(a_p.x) || !std::isfinite(a_p.y) || !std::isfinite(a_p.z)) {
        if (a_why) *a_why = "no walker or no valid point";
        return false;
    }
    auto* cell = a_walker->GetParentCell();
    if (!cell) {
        if (a_why) *a_why = "she is in no loaded cell";
        return false;
    }
    if (cell->IsInteriorCell()) return true;           // an interior is loaded whole: every point of it is her cell's
    auto*              tes = RE::TES::GetSingleton();
    RE::TESObjectCELL* c = tes ? tes->GetCell(a_p) : nullptr;
    if (!c || c->IsInteriorCell() || !c->IsAttached() || c->GetRuntimeData().worldSpace != a_walker->GetWorldspace()) {
        say();
        return false;
    }
    // the cell found must be the one under the point (its grid coordinates) - else the look-up answered something else
    if (const auto* ext = c->GetCoordinates()) {
        const auto cx = static_cast<std::int32_t>(std::floor(a_p.x / kCellU)), cy = static_cast<std::int32_t>(std::floor(a_p.y / kCellU));
        if (ext->cellX != cx || ext->cellY != cy) {
            say();
            return false;
        }
    }
    return true;
}

void NoteBadLeg(const RE::NiPoint3& a_a, const RE::NiPoint3& a_b, float a_seconds)
{
    const float life = std::clamp(a_seconds > 0.f ? a_seconds : 60.f, 5.f, 300.f);
    std::scoped_lock lk(g_memMtx);
    const auto       now = Clock::now();
    g_badLegs.erase(std::remove_if(g_badLegs.begin(), g_badLegs.end(), [&now](const BadLeg& b) { return b.until <= now; }), g_badLegs.end());
    if (g_badLegs.size() >= kBadLegsMax) g_badLegs.erase(g_badLegs.begin());
    BadLeg b;
    b.a = a_a;
    b.b = a_b;
    b.until = now + std::chrono::milliseconds(static_cast<long long>(life * 1000.f));
    g_badLegs.push_back(b);
}

}  // namespace standpoint_nav
