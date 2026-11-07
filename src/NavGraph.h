#pragma once
// ─────────────────────────────────────────────────────────────────────────────
//  NavGraph.h — v1.6.0: Standpoint's own long path, THE SKELETON (Report/StandPoint Module/1.6.0 Plan/D_Standpoint_160_Build_Spec.md
//  §1 item 6; the design S2_Long_Path_Speed_Stairs_Wading.md §2.2 / §2.5 / §2.6).
//
//  An A* over the navmesh TRIANGLES of the LOADED cells (her interior cell, or the attached exterior cells of her worldspace),
//  read straight from the engine's own arrays (CommonLibVR 8.3.0 BSNavmesh: vertices, triangles, extraEdgeInfo, closedDoors), then
//  a funnel (string pull) over the crossed edges -> a few corner points. It asks NO physics question: it only tells the WAY (which
//  side of the lake, which stair, which gap). Every leg she walks is still tested by the planner's rays (the MIX rule: the navmesh
//  suggests, the rays decide).
//    * never a load door (a teleport is no triangle link), never a cell that is not loaded (a portal into an unloaded navmesh is a
//      border); the engine's navcut holes are respected (a triangle flagged deleted (bit 3) is never entered; the cutter's cut
//      edges are already "none"); bit 5 (1.6.1): in a navmesh with RUNTIME CUTS it is the cutter's retire mark = a wall, never a
//      start / goal triangle; elsewhere (the CK's "Overlapping" flag - stairs, bridges) it costs x6 and loses every Locate tie
//    * a closed ordinary door's triangle is passed only when standpoint_path::IsOpenableDoor says yes (a locked / wired / load door
//      is a wall); a door that cannot be read is passed (the leg test decides)
//    * MAIN THREAD only (SKSE tasks / the planner's slices); every slice re-resolves each navmesh by its FormID from the loaded cells
//      and holds their smart pointers while it reads; no raw pointer is kept across slices; every index is checked against its
//      array's size; a navmesh whose pointer or counts changed between slices (the engine's copy -> cut -> swap) restarts the
//      search (at most 3 times)
//    * a SELF-CHECK of every navmesh it reads (counts sane, every index in range, vertices finite) - a failure answers status -1
//      with selfCheckFailed: the caller falls back to the 1.5.x grid corridor (logged once per walker per game run, failures always)
//    * VIRTUAL LINKS: no triangle path inside the loaded cells (two mods' navmeshes that do not meet) -> border edges of the reached
//      island and of other islands within 256 u of each other are linked (cost x2, flagged kSkVirtual); the planner's leg test
//      proves or refuses them (a refused leg: NoteBadLeg, remembered 60 s)
//    * her start / the goal off the navmesh (water, a rock, a balcony) join it at the nearest triangle within 256 u (kSkJoin)
//    * straight stretches are cut at least every 256 u (kSkCut) so a failed leg's repair stays local
//  Its own jobs (BuildSkeleton) run on their own pump: one main-thread slice of <= 0.25 ms per job per frame.
// ─────────────────────────────────────────────────────────────────────────────

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace standpoint_nav {

    // a skeleton point's flags
    enum SkelFlag : std::uint32_t {
        kSkCorner  = 1,     // a funnel corner: the way turns here
        kSkCut     = 2,     // added so no straight stretch is longer than 256 u
        kSkVirtual = 4,     // the leg INTO this point is a VIRTUAL LINK between two navmesh islands (<= 256 u): not proven walkable
        kSkWater   = 8,     // on a water triangle (navmesh flag 0x200 - an inference, 70 %)
        kSkDoor    = 16,    // the leg INTO this point crosses a closed ordinary door's triangle (the planner's rays pass the door)
        kSkJoin    = 32,    // her start / the goal lies off the navmesh: the skeleton joins it here (<= joinMax)
        kSkGoal    = 64     // the goal itself (the last point)
    };

    struct SkelPoint {
        RE::NiPoint3  pos{};           // on the navmesh (z = the triangle's plane there); the goal: the asked point
        std::uint32_t flags{ 0 };
    };

    struct Disc {                      // a place the skeleton keeps away from (a standing actor, a spot she got stuck at)
        float x{ 0.f }, y{ 0.f }, z{ 0.f }, r{ 0.f };
    };

    struct SkelOptions {
        std::uint32_t     tag{ 0 };            // the caller's id (WALK / STEER / PATH #n) - printed on every SKELETON line
        bool              water{ false };      // water triangles cost 1 (a water walk's last stretch); else x4
        float             clearance{ 24.f };   // her clearance rc: each crossed edge is narrowed by it at both ends (the funnel)
        float             joinMax{ 256.f };    // her start / the goal off the navmesh: joined at the nearest triangle within this
        bool              links{ true };       // no triangle path: virtual links between islands <= 256 u apart
        std::vector<Disc> discs;               // a triangle whose middle lies inside one costs x6 (never refused: the rays decide)
    };

    struct SkelInfo {
        int         status{ -2 };      // 1 a skeleton · 0 none (why; outsideLoaded) · -1 failed (walker gone, no navmesh read, the
                                       // self-check failed: selfCheckFailed) · -2 no such job (unknown handle, dropped, a load)
        std::string why;
        float       length{ 0.f };     // u along the skeleton, her start -> the goal
        int         expanded{ 0 };     // triangles expanded
        int         meshes{ 0 };       // navmeshes read
        int         links{ 0 };        // virtual links on the skeleton
        int         restarts{ 0 };     // restarts after a navmesh changed under the search
        int         slices{ 0 };
        double      ms{ 0.0 }, msMax{ 0.0 };
        bool        selfCheckFailed{ false };   // the navmesh data looked wrong: use the 1.5.x grid corridor instead
        bool        outsideLoaded{ false };     // the goal lies outside the loaded cells (the walk answers O 26)
        bool        startJoined{ false }, goalJoined{ false };
    };

    // A skeleton job on its own pump (main-thread slices <= 0.25 ms). MAIN THREAD (reads her cell). A handle >= 1, or 0 = refused
    // (no walker / not loaded / 16 jobs running). Every job ends within ~60 s.
    std::int32_t BuildSkeleton(RE::Actor* a_walker, const RE::NiPoint3& a_from, const RE::NiPoint3& a_to, const SkelOptions& a_o);
    // false = still running. An unknown handle answers true with status -2. Any thread (copies under its lock).
    bool GetSkeleton(std::int32_t a_handle, SkelInfo& a_info, std::vector<SkelPoint>& a_pts);
    void Cancel(std::int32_t a_handle);
    void ClearAll();       // a game load: every job and the 60 s memories go (standpoint_path::ClearAll calls it)
    int  RunningJobs();    // jobs still searching (the planner leaves them their share of the frame)

    // MAIN THREAD. Is a_p inside the area the game has loaded for her: her interior cell (an interior is loaded whole), or an
    // ATTACHED exterior cell of her worldspace (TES::GetCell)? false -> a_why (when given): "the point is outside the loaded area
    // (N m from the player): ..." - the walk answers O 26.
    bool GoalLoaded(RE::Actor* a_walker, const RE::NiPoint3& a_p, std::string* a_why = nullptr);

    // Any thread. A leg a -> b that her body cannot walk (its line test AND its local repair failed): for 60 s (5-300) a skeleton
    // keeps away from it (the triangles it crosses cost x6; a virtual link with these ends is never offered again).
    void NoteBadLeg(const RE::NiPoint3& a_a, const RE::NiPoint3& a_b, float a_seconds = 60.f);

    // ---- a search stepped INSIDE another main-thread job (the planner's MIX: the skeleton instead of the 32 u grid corridor) ----
    struct Search;
    struct SearchDeleter { void operator()(Search* a_s) const noexcept; };
    using SearchPtr = std::unique_ptr<Search, SearchDeleter>;
    // MAIN THREAD. Never null (a refused begin is a finished search with its status).
    SearchPtr SearchBegin(RE::Actor* a_walker, const RE::NiPoint3& a_from, const RE::NiPoint3& a_to, const SkelOptions& a_o);
    // MAIN THREAD. Works until a_deadline (one unit past it at most). true = finished (read SearchResult).
    bool SearchStep(Search& a_s, std::chrono::steady_clock::time_point a_deadline);
    void SearchResult(const Search& a_s, SkelInfo& a_info, std::vector<SkelPoint>& a_pts);

}  // namespace standpoint_nav
