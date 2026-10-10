#pragma once

// ---------------------------------------------------------------------------
// Standpoint threat scan (v0.6.0 Tier 1) — pure, read-only, synchronous.
// "Between this walker and where I'm sending them, is there anything hostile?"
// Corridor-shaped, not radius-shaped. See Threats.cpp for the reasoning behind
// the four separately-reported hostility signals.
// ---------------------------------------------------------------------------

namespace standpoint_threats {

struct ScanOut {
    std::int32_t count      = 0;     // the gating union: declared | viaPlayer
    std::int32_t declared   = 0;     // engine: hostile to the walker
    std::int32_t viaPlayer  = 0;     // engine: hostile to the player
    std::int32_t inCombat   = 0;     // actively fighting (unambiguous but late)
    std::int32_t veryAggro  = 0;     // AIDATA aggression >= VeryAggressive (predictive)
    float        nearest    = -1.0f; // walker -> nearest unioned threat, -1 if none
    RE::NiPoint3 centroid{};         // of the unioned threats: the danger blob
    float        spread     = 0.0f;  // blob radius (feeds Tier 3's avoid-node radius)
};

ScanOut Scan(RE::Actor* a_walker, const RE::NiPoint3& a_dest, float a_corridor, float a_lookAhead);

}  // namespace standpoint_threats
