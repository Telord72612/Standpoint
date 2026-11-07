#pragma once
// ─────────────────────────────────────────────────────────────────────────────
//  RelativeSolve.h — 1.6.0: THE PHYSICS SOLVE of a relative spot (report "1.6.0 Plan" S1 §2.1; spec D §1 item 4).
//
//  "Find the proper spot around that object / NPC according to the request" (the user, 2026-11-06). The candidates are the
//  solver's OWN (SlotSolver.h's exports - the clock ring in its hour order with its distance floors, or GoTo's furniture
//  entries / box sides / actor sides in their rank), but each one is judged by PHYSICS instead of the navmesh snap:
//    1. the planner's stand-spot probe (a floor she can stand on - n.z >= 0.85, 0.75 on navmesh-backed terrain - headroom,
//       a riser on each side, water within the cap): its floor z becomes the spot's z;
//    2. the F1 level rule (|floor - the target's floor| <= max(40, 0.58 x flat distance));
//    3. not inside another standing actor's disc (their radius + her clearance + 8 - the walker and the target left out,
//       moving actors ignored: the planner's own eligibility; the probes scan no actors);
//    4. her body can get there: a CLOSE spot (inside the target's keep-out circle) needs the approach point E just outside
//       it to be a stand spot and the radial leg E -> spot to be clear; a FAR spot needs the "reach" leg from the target's
//       edge to the spot to be clear (a spot behind a wall from the target fails here).
//  The asked side is tried first, then the solver's own order; pass 1 (an object's wider ring, an actor + 16 u) only when
//  pass 0 has nothing. MAIN THREAD (the probes cast rays). Sliced: Step() spends a time budget per frame, at least one probe.
//  1.6.1 (the first 1.6.0 VR run, report "1.6.0 Plan" R3 §2): the stand probes' side rays step through the TARGET's own body
//  (PathPlanner's a_ignoreRef - a chair's seat beside its entry spot is no brink), a refusal line names the side ray and its
//  heights, and a GoTo on furniture tries its BOX SIDES after its entries (when every entry is refused - no more O 1 then; the
//  FIX-UP: never when an entry is TAKEN - an occupied chair answers O 1 as 1.5.1 did; the sides ranked by side across the tiers).
// ─────────────────────────────────────────────────────────────────────────────
#include "SlotSolver.h"

namespace standpoint_relsolve {

    enum class Kind : std::uint8_t { kClock = 1, kGoTo = 2 };

    struct Ask {
        Kind               kind{ Kind::kClock };
        RE::FormID         who{ 0 };           // the walker
        RE::FormID         target{ 0 };        // the relative target
        std::int32_t       steerId{ 0 };       // every log line names STEER #steerId
        // the clock (RequestMove)
        float              distU{ 0.f };       // centre of the target to her, game units
        float              facingH{ 12.f };    // the clock hour around the target (target's facing); < 0 = the side she is nearest
        float              orientH{ 0.f };     // her own facing once there (0 = toward the target)
        bool               close{ false };     // close contact: an actor target gets no distance floor
        // GoTo
        int                relation{ 0 };      // 0 nearest, 1 front, 2 right, 3 back, 4 left
        float              standoffU{ 0.f };   // <= 0 = the solver's default
        // both
        float              depthFrac{ 0.45f }; // the deepest water a spot may have, x her height (S2: the thigh max)
        bool               preferSet{ false }; // a RE-SOLVE: this side first (she does not flip sides on a moving target)
        float              preferRel{ 0.f };   //   (rad, the target's frame)
        std::vector<float> avoidRels;          // sides never taken (rad, target frame): within 2 hours of a side whose plan failed
        // FIX ROUND (review R1 #7): the target's facing frame is THIS yaw (compass rad), not its live one - the PLAYER as target:
        // his headset turns with every glance, so a re-solve keeps the first solve's frame and only follows his position
        bool               frameYawSet{ false };
        float              frameYaw{ 0.f };
    };

    struct Spot {
        bool         found{ false };
        RE::NiPoint3 pos{};                // her feet: x / y as solved, z = the floor the stand-spot probe found
        float        headingRad{ 0.f };    // her final heading (compass) as solved
        bool         faceLive{ false };    // an actor target: she faces it LIVE (+ faceOffsetRad) - the walk's faceWho
        float        faceOffsetRad{ 0.f }; // clockwise from "toward the target" (the clock's orientation)
        float        rel{ 0.f };           // the side, rad in the target's frame
        float        hours{ 0.f };         // the same as a clock hour [0, 12)
        float        distU{ 0.f };         // centre of the target to the spot (u)
        bool         inCircle{ false };    // inside the target's keep-out circle: the walk comes in from `approach` (E)
        RE::NiPoint3 approach{};           // E (z = its floor), when inCircle
        bool         onNavmesh{ false };   // the navmesh lies under the spot (within 16 u flat / 24 u high)
        std::int32_t markerIndex{ -1 };    // GoTo: the furniture marker it serves
        int          tested{ 0 };          // candidates looked at
        int          probes{ 0 };          // probes asked
        std::string  why;                  // nothing found: the reasons, in short
    };

    // The keep-out circle the planner draws round a_target for a_walker (PathPlanner's StartEx rule before its shrink): the
    // target's controller radius (an actor; else 24) + her radius (else 20) + 8.
    float KeepOutRadius(RE::TESObjectREFR* a_target, RE::Actor* a_walker);
    // Her radius as the planner reads it (collisionBound.extents.x, 4-96 u; else 20).
    float WalkerRadius(RE::Actor* a_walker);
    // Is a_p inside a STANDING actor's disc (their radius + her clearance + 8; the walker and a_target left out, moving actors
    // ignored - the planner's eligibility)? a_who / a_r name it. MAIN THREAD (reads the process lists).
    bool StandingActorAt(RE::Actor* a_walker, RE::FormID a_target, const RE::NiPoint3& a_p, RE::FormID& a_who, float& a_r);

    // One solve. Begin builds the candidate list (no rays). Step tests candidates until a_budgetMs is spent (at least one probe)
    // and returns true once finished; Result() then holds the spot (found) or the reasons (not found). MAIN THREAD.
    class Job {
    public:
        bool        Begin(const Ask& a_ask, std::string& a_why);
        bool        Step(double a_budgetMs);
        bool        Finished() const { return m_done; }
        const Spot& Result() const { return m_spot; }

    private:
        struct Cand {
            RE::NiPoint3 pos{};
            float        rel{ 0.f };
            float        distU{ 0.f };
            float        faceRad{ 0.f };
            float        reachU{ 0.f };       // the target's reach toward this side (the "reach" leg starts beyond it)
            std::int32_t marker{ -1 };
            int          pass{ 0 };
        };
        struct Disc {
            RE::FormID id{ 0 };
            float      x{ 0.f }, y{ 0.f }, r{ 0.f };
        };
        void Finish(bool a_found);
        void LogCand(const Cand& a_c, const std::string& a_verdict) const;

        Ask                       m_ask{};
        std::vector<Cand>         m_cands;
        std::vector<Disc>         m_discs;
        std::size_t               m_next{ 0 };      // the candidate being tested
        int                       m_sub{ 0 };       // its sub-step: 0 the stand spot, 1 E's stand spot, 2 the leg
        RE::NiPoint3              m_cur{};          // the candidate with its floor z (after sub-step 0)
        RE::NiPoint3              m_e{};            // E (close spots)
        RE::NiPoint3              m_tpos{};         // the target's position (the keep-out circle's centre)
        RE::NiPoint3              m_centre{};       // TargetCentre (hours and distances)
        float                     m_targetFacing{ 0.f };
        float                     m_floorZ{ 0.f };  // the level every spot must stand on
        float                     m_keepR{ 0.f };   // the keep-out circle (KeepOutRadius)
        float                     m_rc{ 0.f };      // her clearance (radius + 4)
        float                     m_orientOff{ 0.f };
        int                       m_ignoredHits{ 0 }; // 1.6.1: the candidate's stand probe stepped through the target's own body this often
        bool                      m_targetActor{ false };
        bool                      m_done{ false };
        Spot                      m_spot{};
        std::string               m_lastWhy;
        std::chrono::steady_clock::time_point m_t0{};
        double                    m_ms{ 0.0 };
        int                       m_steps{ 0 };
    };

}  // namespace standpoint_relsolve
