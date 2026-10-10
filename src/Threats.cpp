// ─────────────────────────────────────────────────────────────────────────────
//  Threats.cpp — v0.6.0 Tier 1: "don't walk into danger" RECOGNITION.
//
//  OBSERVATION ONLY, same contract as Interrupts.cpp. Nothing here steers,
//  suppresses, or alters engine behaviour. It answers one question for the
//  Papyrus driver, cheaply, on demand:
//
//      "Between this walker and where I'm sending them, is there anything
//       that wants them dead?"
//
//  ── THE CORRIDOR, not a radius ──────────────────────────────────────────────
//  A plain proximity scan is the wrong test: a bandit 900 units to the LEFT is
//  irrelevant if the route runs right, and a bandit 900 units straight AHEAD is
//  exactly the case we care about. So candidates are tested against the
//  walker→destination segment:
//
//      along = how far down the route the threat sits (clamped to lookAhead)
//      perp  = how far off the route it sits           (must be <= corridor)
//
//  Anything inside `kAlwaysRadius` of the walker qualifies regardless of
//  direction — danger standing next to her is danger whatever the heading.
//  A generous Z gate keeps enemies two floors below from firing.
//
//  ── THE HOSTILITY PREDICATE IS THE OPEN QUESTION ────────────────────────────
//  `IsHostileToActor` reliably reports DECLARED hostility, but a bandit camp
//  that has not noticed anyone yet may read as not-hostile — which is precisely
//  the case this feature exists for. Rather than guess, this scan computes FOUR
//  independently-rooted signals and reports all of them:
//
//      declared   – engine says hostile to the walker (either direction)
//      viaPlayer  – engine says hostile to the PLAYER (fires before the walker
//                   is noticed; a follower's danger is the player's danger)
//      inCombat   – actively fighting right now (unambiguous, but late)
//      veryAggro  – AIDATA aggression >= 2 (VeryAggressive/Frenzied = "attacks
//                   anyone not in my own faction"): bandits, wildlife, draugr.
//                   PREDICTIVE — fires on a camp that is still asleep.
//
//  The union that actually gates travel is (declared | viaPlayer); veryAggro is
//  measured but NOT unioned by default, because it can flag a friendly
//  aggressive NPC. The driver logs every count whenever any is non-zero, so one
//  field session tells us which predicate really catches a bandit camp before
//  it aggros — then we promote it on evidence instead of on theory.
//  (Method: PPB's "log the unknown, then build against it".)
// ─────────────────────────────────────────────────────────────────────────────
#include "PCH.h"
#include "Threats.h"

namespace logger = SKSE::log;

namespace {

constexpr float kAlwaysRadius = 320.0f;   // danger this close counts in any direction
constexpr float kZGate        = 400.0f;   // ignore threats this far above/below
constexpr float kBehind       = 96.0f;    // tolerance for "just behind the walker"

struct Candidate {
    RE::NiPoint3 pos;
    bool         declared;
    bool         viaPlayer;
    bool         inCombat;
    bool         veryAggro;
};

// AIDATA aggression is a 2-bit field split across two bools (TESAIForm.h:71-72).
// 0 Unaggressive, 1 Aggressive, 2 VeryAggressive, 3 Frenzied.
std::int32_t AggressionOf(RE::Actor* a_actor)
{
    auto* base = a_actor->GetActorBase();
    if (!base) return -1;
    const auto& ai = base->aiData;
    return (ai.aggression1 ? 1 : 0) | (ai.aggression2 ? 2 : 0);
}

bool Eligible(RE::Actor* a_actor, RE::Actor* a_walker, RE::Actor* a_player)
{
    if (!a_actor || a_actor == a_walker || a_actor == a_player) return false;
    // ★ v0.7.1 (2026-09-30) - ONLY ACTORS IN THE WALKER'S OWN LOADED SPACE.
    // ProcessLists' middle-low and low arrays hold actors in UNLOADED cells all over the game, and GetPosition() answers in
    // the coordinates of whatever cell an actor is in. Exterior coordinates are unique, so outdoors nothing far away could
    // pass the distance test (the 2026-08-18 wolf field test). INTERIORS ARE ALL BUILT AROUND THE ORIGIN: inside any
    // house, every bandit of every dungeon in the game sits "a few units away" on paper.
    // MEASURED 2026-09-30 23:13 (VRCD's `vrcw` console test, a player home, nobody fighting): 20-30 "hostiles", the
    // nearest "7 u" from the walker, 84-105 aggressive actors in the corridor - and every session DROPPED on its first
    // poll (danger = 2). The same false alarm would drop any interior errand (bath houses included).
    // The scan's own contract is "LOADED actors only - the same horizon the engine fights at": an actor without 3D, in
    // another interior cell, or in another worldspace cannot be on the route.
    if (!a_actor->Is3DLoaded()) return false;
    {
        auto* cellW = a_walker ? a_walker->GetParentCell() : nullptr;
        auto* cellA = a_actor->GetParentCell();
        if (!cellW || !cellA) return false;
        if (cellW->IsInteriorCell() || cellA->IsInteriorCell()) {
            if (cellW != cellA) return false;
        } else if (a_actor->GetWorldspace() != a_walker->GetWorldspace()) {
            return false;
        }
    }
    if (a_actor->IsDead() || a_actor->IsDisabled() || a_actor->IsGhost()) return false;
    if (a_actor->IsPlayerTeammate()) return false;
    return true;
}

}  // namespace

namespace standpoint_threats {

// Scan the corridor from the walker to `a_dest`. Pure read; no allocation on
// the game's behalf; safe to call at the driver's 0.5 s poll rate.
ScanOut Scan(RE::Actor* a_walker, const RE::NiPoint3& a_dest, float a_corridor, float a_lookAhead)
{
    ScanOut out;
    if (!a_walker) return out;

    auto* lists = RE::ProcessLists::GetSingleton();
    if (!lists) return out;
    auto* player = RE::PlayerCharacter::GetSingleton();

    const RE::NiPoint3 origin = a_walker->GetPosition();
    RE::NiPoint3       route  = a_dest - origin;
    route.z = 0.0f;
    const float routeLen = std::sqrt(route.x * route.x + route.y * route.y);
    const bool  haveRoute = routeLen > 1.0f;
    const RE::NiPoint3 dir = haveRoute ? RE::NiPoint3{ route.x / routeLen, route.y / routeLen, 0.0f }
                                       : RE::NiPoint3{ 0.0f, 0.0f, 0.0f };
    const float reach = haveRoute ? (std::min)(routeLen, a_lookAhead) : a_lookAhead;

    std::vector<Candidate> hits;
    hits.reserve(16);

    const RE::BSTArray<RE::ActorHandle>* arrays[4] = {
        &lists->highActorHandles, &lists->middleHighActorHandles,
        &lists->middleLowActorHandles, &lists->lowActorHandles
    };

    for (const auto* arr : arrays) {
        for (const auto& handle : *arr) {
            auto ptr = handle.get();
            RE::Actor* a = ptr.get();
            if (!Eligible(a, a_walker, player)) continue;

            const RE::NiPoint3 p = a->GetPosition();
            RE::NiPoint3       w = p - origin;
            if (std::fabs(w.z) > kZGate) continue;
            w.z = 0.0f;
            const float flat = std::sqrt(w.x * w.x + w.y * w.y);

            bool inCorridor = flat <= kAlwaysRadius;
            if (!inCorridor && haveRoute) {
                const float along = w.x * dir.x + w.y * dir.y;
                if (along >= -kBehind && along <= reach) {
                    const float px = w.x - dir.x * along;
                    const float py = w.y - dir.y * along;
                    inCorridor = std::sqrt(px * px + py * py) <= a_corridor;
                }
            }
            if (!inCorridor) continue;

            Candidate c{};
            c.pos       = p;
            c.declared  = a->IsHostileToActor(a_walker) || a_walker->IsHostileToActor(a);
            c.viaPlayer = player && (a->IsHostileToActor(player) || player->IsHostileToActor(a));
            c.inCombat  = a->IsInCombat();
            c.veryAggro = AggressionOf(a) >= 1;  // MEASURED 2026-08-18: EncWolf is
                                         // Aggressive(1); the original >= 2
                                         // (VeryAggressive) reported ZERO on
                                         // live hostile wolves - too strict to
                                         // be the safety net it was meant as.

            if (c.declared)  ++out.declared;
            if (c.viaPlayer) ++out.viaPlayer;
            if (c.inCombat)  ++out.inCombat;
            if (c.veryAggro) ++out.veryAggro;

            if (c.declared || c.viaPlayer) {
                hits.push_back(c);
                if (out.nearest < 0.0f || flat < out.nearest) out.nearest = flat;
            }
        }
    }

    out.count = static_cast<std::int32_t>(hits.size());
    if (!hits.empty()) {
        RE::NiPoint3 sum{ 0.0f, 0.0f, 0.0f };
        for (const auto& h : hits) sum += h.pos;
        const float inv = 1.0f / static_cast<float>(hits.size());
        out.centroid = RE::NiPoint3{ sum.x * inv, sum.y * inv, sum.z * inv };
        for (const auto& h : hits) {
            const float dx = h.pos.x - out.centroid.x;
            const float dy = h.pos.y - out.centroid.y;
            const float d  = std::sqrt(dx * dx + dy * dy);
            if (d > out.spread) out.spread = d;
        }
    }
    return out;
}

}  // namespace standpoint_threats
