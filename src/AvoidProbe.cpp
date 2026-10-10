// ─────────────────────────────────────────────────────────────────────────────
//  AvoidProbe.cpp — v0.6.0 Tier 3, PHASE A: the avoid-node reconnaissance spike.
//
//  READ-ONLY. This file writes nothing to the game and calls no game function
//  it did not first prove is safe to reach. Its entire job is to answer, from
//  the live VR runtime, the questions static analysis cannot: SkyrimVR.exe is
//  SteamStub-encrypted, so the code cannot be disassembled offline — but the
//  RTTI, the vtables and the live objects are all readable at runtime.
//
//  ── WHAT WE ARE AFTER ───────────────────────────────────────────────────────
//  Every path the engine solves carries BSPathingRestrictions::avoidNodeArray —
//  a BSTArray<BSPathingAvoidNode> where each node is {point1, point2, radius,
//  COST, type} and one of the types is literally AVOID_NODE_SPHERE_THREAT
//  (BSPathingAvoidNode.h, sizeof 0x24, static_asserted). Feed a threat node into
//  that array and the engine's OWN A* routes around it — with its own doors,
//  ledges, portals and funnel smoothing, none of which we would have to rebuild.
//  Report 01 §8 already recommended exactly this: "inject BSPathingAvoidNodes
//  rather than planning a path."
//
//  The chain, from the RTTI mined out of the VR binary:
//      StaticAvoidNodeManager                (global registry, a singleton)
//        -> MovementPathManagerAgentStaticAvoider   (per-actor movement agent)
//           -> BSPathingRequest::restrictions.avoidNodeArray  (per path solve)
//
//  CommonLibVR 4.14.0 binds the STRUCTS but not the MANAGER — RTTI and vtable
//  addresses only, no header, no functions. Hence a spike rather than a build.
//
//  ── WHY IT IS SHAPED LIKE THIS ──────────────────────────────────────────────
//  Two deliberate choices keep this from being able to crash the user's game:
//
//  1. NOTHING is dereferenced without VirtualQuery proving the pages are
//     committed and readable first (`Readable`). Every walk is bounded.
//  2. Objects are found by VTABLE IDENTITY, never by struct offset. CommonLib
//     only static_asserts MovementControllerAI's layout under
//     EXCLUSIVE_SKYRIM_FLAT — in VR those offsets are unverified, so reading
//     `agents` at a hardcoded 0x030 would be a guess. Instead we sweep the
//     controller's memory for any pointer whose first qword matches a known
//     agent vtable. That is layout-independent by construction: it finds the
//     agents wherever VR actually puts them, and finds nothing if they are not
//     there, rather than reading garbage and reporting it as fact.
//
//  Output is a log block. Phase B (actually registering a threat node) is
//  written against what this prints — not before.
// ─────────────────────────────────────────────────────────────────────────────
#include "PCH.h"

namespace logger = SKSE::log;

namespace {

// ── memory safety ───────────────────────────────────────────────────────────
bool Readable(const void* a_ptr, std::size_t a_bytes)
{
    if (!a_ptr || a_bytes == 0) return false;
    auto addr = reinterpret_cast<std::uintptr_t>(a_ptr);
    const auto end = addr + a_bytes;
    // CTD lesson (crash-2026-08-18-19-44-10): 0xFFFFFFFFFFFFFFFF sentinels are
    // COMMON in swept memory; masked to ...FFFE, addr+8 wraps past zero, the
    // while loop below never runs, and this function silently said "readable".
    if (end < addr) return false;                          // arithmetic wrap
    if (end > 0x00007FFFFFFFFFFFull) return false;         // non-canonical / kernel half
    MEMORY_BASIC_INFORMATION mbi{};
    while (addr < end) {
        if (::VirtualQuery(reinterpret_cast<LPCVOID>(addr), &mbi, sizeof(mbi)) == 0) return false;
        if (mbi.State != MEM_COMMIT) return false;
        if (mbi.Protect & PAGE_GUARD) return false;
        if ((mbi.Protect & 0xFF) == PAGE_NOACCESS) return false;
        addr = reinterpret_cast<std::uintptr_t>(mbi.BaseAddress) + mbi.RegionSize;
    }
    return true;
}

struct ModuleRange {
    std::uintptr_t base = 0;
    std::uintptr_t end  = 0;
    bool Contains(std::uintptr_t a) const { return a >= base && a < end; }
};

ModuleRange g_module{};

// EXECUTABLE sections, kept as a LIST. An earlier version stored one min..max
// span, which swallowed the .rdata sitting between two executable sections —
// so RTTI locators still passed IsCode() and the vtable walk still over-ran
// (measured 2026-08-18: it ran to slot 0x0A and stopped on a string constant).
constexpr std::size_t kMaxCodeSections = 8;
ModuleRange           g_code[kMaxCodeSections]{};
std::size_t           g_codeCount = 0;

// Is this address inside an EXECUTABLE section? Vtables are terminated in
// practice by an RTTI complete-object-locator pointer, which lives in .rdata
// and passes a naive "is it in the module" test — that is exactly how the
// 2026-08-18 A3 dump walked past the end of two vtables and tried to
// disassemble RTTI structures as code (0x1974870 / 0x1974B78 were data).
bool IsCode(std::uintptr_t a_addr)
{
    for (std::size_t i = 0; i < g_codeCount; ++i) {
        if (g_code[i].Contains(a_addr)) return true;
    }
    return false;
}

void InitModuleRange()
{
    if (g_module.base) return;
    const auto base = REL::Module::get().base();
    if (!Readable(reinterpret_cast<void*>(base), sizeof(IMAGE_DOS_HEADER))) return;
    auto* dos = reinterpret_cast<const IMAGE_DOS_HEADER*>(base);
    auto* nt  = reinterpret_cast<const IMAGE_NT_HEADERS64*>(base + dos->e_lfanew);
    if (!Readable(nt, sizeof(IMAGE_NT_HEADERS64))) return;
    g_module.base = base;
    g_module.end  = base + nt->OptionalHeader.SizeOfImage;

    auto* sec = IMAGE_FIRST_SECTION(nt);
    for (int i = 0; i < nt->FileHeader.NumberOfSections; ++i, ++sec) {
        if (!(sec->Characteristics & IMAGE_SCN_MEM_EXECUTE)) continue;
        if (g_codeCount >= kMaxCodeSections) break;
        g_code[g_codeCount].base = base + sec->VirtualAddress;
        g_code[g_codeCount].end  = g_code[g_codeCount].base + sec->Misc.VirtualSize;
        ++g_codeCount;
    }
}

// Format an address as "base+0xNNNNNN" when it lives in SkyrimVR.exe — that is
// the form that cross-references against Address Library and the RTTI dumps.
std::string Rel(std::uintptr_t a_addr)
{
    if (g_module.Contains(a_addr)) return fmt::format("SkyrimVR+0x{:X}", a_addr - g_module.base);
    if (a_addr == 0) return "null";
    return fmt::format("0x{:016X}", a_addr);
}

void HexDump(const char* a_label, const void* a_obj, std::size_t a_bytes)
{
    if (!Readable(a_obj, a_bytes)) {
        logger::info("    {}: <unreadable>", a_label);
        return;
    }
    const auto* q = reinterpret_cast<const std::uintptr_t*>(a_obj);
    const std::size_t words = a_bytes / 8;
    for (std::size_t i = 0; i < words; i += 4) {
        std::string line = fmt::format("    {} +{:03X}:", a_label, i * 8);
        for (std::size_t j = i; j < i + 4 && j < words; ++j) {
            line += fmt::format(" {:016X}", q[j]);
        }
        logger::info("{}", line);
    }
}

// Dump raw CODE bytes of a function as continuous hex. SkyrimVR.exe is
// SteamStub-encrypted ON DISK but decrypted in memory — we are inside the
// process, so this is the one place its code can be read for offline
// disassembly. READ ONLY; nothing is ever written or executed from here.
void DumpCode(std::uintptr_t a_fn, std::size_t a_bytes)
{
    if (!g_module.Contains(a_fn) || !Readable(reinterpret_cast<void*>(a_fn), a_bytes)) {
        logger::info("  code {}: <unreadable>", Rel(a_fn));
        return;
    }
    const auto* b = reinterpret_cast<const std::uint8_t*>(a_fn);
    std::string hex;
    hex.reserve(a_bytes * 2);
    for (std::size_t i = 0; i < a_bytes; ++i) hex += fmt::format("{:02X}", b[i]);
    // 96 hex chars (48 bytes) per line keeps the log grep-able
    for (std::size_t i = 0; i < hex.size(); i += 96) {
        logger::info("  code {} +{:03X}: {}", Rel(a_fn), i / 2, hex.substr(i, 96));
    }
}

// Belt-and-braces for every raw dereference in the sweeps: page-check first
// (Readable), then read under SEH so a check-to-use race can never crash the
// game. No C++ objects in here — SEH and unwinding don't mix (C2712).
bool SafeReadQword(std::uintptr_t a_addr, std::uintptr_t& a_out)
{
    __try {
        a_out = *reinterpret_cast<const volatile std::uintptr_t*>(a_addr);
        return true;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

// ── the vtables we are hunting for ──────────────────────────────────────────
struct KnownVTable {
    const char*    name;
    std::uintptr_t addr;
};

std::vector<KnownVTable> BuildKnownVTables()
{
    std::vector<KnownVTable> v;
    auto add = [&v](const char* n, std::uintptr_t a) { if (a) v.push_back({ n, a }); };

    add("IStaticAvoidNodeManager",                RE::VTABLE_IStaticAvoidNodeManager[0].address());
    add("StaticAvoidNodeManager[0]",              RE::VTABLE_StaticAvoidNodeManager[0].address());
    add("StaticAvoidNodeManager[1]",              RE::VTABLE_StaticAvoidNodeManager[1].address());
    add("MovementPathManagerAgentStaticAvoider[0]", RE::VTABLE_MovementPathManagerAgentStaticAvoider[0].address());
    add("MovementPathManagerAgentStaticAvoider[1]", RE::VTABLE_MovementPathManagerAgentStaticAvoider[1].address());
    add("MovementPathManagerAgentStaticAvoider[2]", RE::VTABLE_MovementPathManagerAgentStaticAvoider[2].address());
    add("MovementPathManagerAgentStaticAvoider[3]", RE::VTABLE_MovementPathManagerAgentStaticAvoider[3].address());
    add("MovementPathManagerAgentAvoidBox[0]",    RE::VTABLE_MovementPathManagerAgentAvoidBox[0].address());
    add("MovementAgentActorAvoider[0]",           RE::VTABLE_MovementAgentActorAvoider[0].address());
    add("MovementAvoidBoxEventAdapter[0]",        RE::VTABLE_MovementAvoidBoxEventAdapter[0].address());
    return v;
}

// Walk a vtable and log its slots as module-relative code offsets. Those
// offsets are what Phase B binds against (and what an Address Library lookup
// turns back into a stable RELOCATION_ID).
void DumpVTable(const char* a_label, std::uintptr_t a_vtable, int a_maxSlots)
{
    if (!g_module.Contains(a_vtable) || !Readable(reinterpret_cast<void*>(a_vtable), 8)) {
        logger::info("  vtable {} @ {} <unreadable>", a_label, Rel(a_vtable));
        return;
    }
    logger::info("  vtable {} @ {}", a_label, Rel(a_vtable));
    const auto* slots = reinterpret_cast<const std::uintptr_t*>(a_vtable);
    for (int i = 0; i < a_maxSlots; ++i) {
        if (!Readable(&slots[i], 8)) break;
        const auto fn = slots[i];
        if (!IsCode(fn)) {   // RTTI locator => real end of this vtable
            logger::info("    (end of table at slot {:02X}; next qword {} is data)", i, Rel(fn));
            break;
        }
        logger::info("    slot {:02X} -> {}", i, Rel(fn));
    }
}

// Sweep a memory window for pointers to objects whose first qword is a known
// vtable. Layout-independent: this is how we find the per-actor agents without
// trusting an unverified VR struct offset.
int SweepForKnownObjects(const char* a_what, const void* a_start, std::size_t a_bytes,
                         const std::vector<KnownVTable>& a_known)
{
    if (!Readable(a_start, a_bytes)) {
        logger::info("  {}: window unreadable", a_what);
        return 0;
    }
    const auto* q = reinterpret_cast<const std::uintptr_t*>(a_start);
    const std::size_t words = a_bytes / 8;
    int found = 0;
    for (std::size_t i = 0; i < words; ++i) {
        // BSTSmallArray<ActiveSmartPtr> steals bit 0 as an "active" flag
        // (MovementControllerAI.h) — mask it before treating it as a pointer.
        const auto cand = q[i] & ~static_cast<std::uintptr_t>(1);
        if (cand < 0x10000) continue;
        if (g_module.Contains(cand)) continue;   // a module pointer is not a heap object
        if (!Readable(reinterpret_cast<void*>(cand), 8)) continue;
        std::uintptr_t vt = 0;
        if (!SafeReadQword(cand, vt)) continue;
        for (const auto& k : a_known) {
            if (vt == k.addr) {
                ++found;
                logger::info("  ★ {} +{:03X} -> {} @ 0x{:016X}", a_what, i * 8, k.name, cand);
                HexDump(k.name, reinterpret_cast<void*>(cand), 0x60);
                break;
            }
        }
    }
    if (!found && !std::strstr(a_what, "->")) {
        logger::info("  {}: no known avoidance objects in window", a_what);
    }
    return found;
}

bool g_singletonProbeDone = false;

}  // namespace

namespace standpoint_avoid {

// Phase A part 1 — module-only. Runs at kDataLoaded. Touches no game object at
// all: it reads this process's own PE sections looking for the singleton, so it
// cannot interact with anything the engine is doing.
void ProbeSingletons()
{
    if (g_singletonProbeDone) return;
    g_singletonProbeDone = true;
    InitModuleRange();
    if (!g_module.base) {
        logger::warn("avoid-probe: could not resolve module range — skipped");
        return;
    }

    logger::info("=== TIER-3 AVOID PROBE (A1: singleton hunt) =============================");
    logger::info("module SkyrimVR base=0x{:016X} size=0x{:X}", g_module.base, g_module.end - g_module.base);

    const auto known = BuildKnownVTables();
    for (const auto& k : known) logger::info("  known vtable {:<42} {}", k.name, Rel(k.addr));

    // The manager is a BSTSingletonSDM — its instance lives in a static buffer
    // in a writable data section, so its vtable pointer is findable by scanning.
    auto* dos = reinterpret_cast<const IMAGE_DOS_HEADER*>(g_module.base);
    auto* nt  = reinterpret_cast<const IMAGE_NT_HEADERS64*>(g_module.base + dos->e_lfanew);
    auto* sec = IMAGE_FIRST_SECTION(nt);

    const std::uintptr_t hunt[3] = {
        RE::VTABLE_StaticAvoidNodeManager[0].address(),
        RE::VTABLE_StaticAvoidNodeManager[1].address(),
        RE::VTABLE_IStaticAvoidNodeManager[0].address(),
    };

    int hits = 0;
    for (int s = 0; s < nt->FileHeader.NumberOfSections; ++s, ++sec) {
        if (!(sec->Characteristics & IMAGE_SCN_MEM_WRITE)) continue;
        const auto start = g_module.base + sec->VirtualAddress;
        const auto len   = sec->Misc.VirtualSize;
        char name[9]{};
        std::memcpy(name, sec->Name, 8);
        logger::info("  scanning writable section {:<8} {} len=0x{:X}", name, Rel(start), len);
        if (!Readable(reinterpret_cast<void*>(start), 8)) continue;

        const auto* q = reinterpret_cast<const std::uintptr_t*>(start);
        const std::size_t words = len / 8;
        for (std::size_t i = 0; i < words; ++i) {
            if ((i & 0xFFF) == 0 && !Readable(&q[i], 0x8000 < (words - i) * 8 ? 0x8000 : (words - i) * 8)) break;
            for (int h = 0; h < 3; ++h) {
                if (q[i] == hunt[h]) {
                    ++hits;
                    const auto inst = reinterpret_cast<std::uintptr_t>(&q[i]);
                    if (h != 0) continue;   // h1/h2 are the SAME object seen via its
                                            // second vtable pointer at +0x08
                    logger::info("  ★ StaticAvoidNodeManager instance at {} (sizeof 0x50, "
                                 "measured from its deleting dtor)", Rel(inst));
                    HexDump("mgr", reinterpret_cast<void*>(inst), 0x50);
                    // hash-map shape read out of the lookup at SkyrimVR+0x735BF0:
                    //   [+0x24] capacity-1 mask   [+0x40] entry table   24-byte entries
                    const auto* w = reinterpret_cast<const std::uint8_t*>(inst);
                    if (Readable(w, 0x50)) {
                        const auto cap   = *reinterpret_cast<const std::uint32_t*>(w + 0x24);
                        const auto table = *reinterpret_cast<const std::uintptr_t*>(w + 0x40);
                        logger::info("    map: capacity={} table={} -> {} static avoid nodes",
                                     cap, Rel(table), table ? "SOME" : "ZERO");
                    }
                }
            }
        }
    }
    if (!hits) {
        logger::info("  no in-module instance found — manager may be heap-allocated "
                     "(Phase B would then reach it via the per-actor agent instead)");
    }

    DumpVTable("IStaticAvoidNodeManager", RE::VTABLE_IStaticAvoidNodeManager[0].address(), 16);
    DumpVTable("StaticAvoidNodeManager[0]", RE::VTABLE_StaticAvoidNodeManager[0].address(), 16);

    // A3: dump every unique function reachable from both tables (0x180 bytes
    // each) so the add/remove/clear entry points can be identified by offline
    // disassembly instead of by guessing.
    {
        std::vector<std::uintptr_t> fns;
        for (auto vt : { RE::VTABLE_IStaticAvoidNodeManager[0].address(),
                         RE::VTABLE_StaticAvoidNodeManager[0].address() }) {
            if (!Readable(reinterpret_cast<void*>(vt), 8)) continue;
            const auto* slots = reinterpret_cast<const std::uintptr_t*>(vt);
            for (int i = 0; i < 12; ++i) {
                if (!Readable(&slots[i], 8)) break;
                const auto fn = slots[i];
                if (!g_module.Contains(fn)) break;
                if (std::find(fns.begin(), fns.end(), fn) == fns.end()) fns.push_back(fn);
            }
        }
        logger::info("  --- A3 code dump: {} unique manager functions ---", fns.size());
        for (auto fn : fns) DumpCode(fn, 0x180);

        // A4: the manager exposes NO public add (measured 2026-08-18 — its two
        // interface methods are hash-map lookups). Population happens from the
        // avoid-box adapter, so dump ITS virtuals: the insert entry point is a
        // non-virtual call from inside one of them, and non-virtual functions
        // never appear in any vtable.
        std::vector<std::uintptr_t> adapter;
        const auto avt = RE::VTABLE_MovementAvoidBoxEventAdapter[0].address();
        if (Readable(reinterpret_cast<void*>(avt), 8)) {
            const auto* slots2 = reinterpret_cast<const std::uintptr_t*>(avt);
            for (int i = 0; i < 12; ++i) {
                if (!Readable(&slots2[i], 8)) break;
                if (!IsCode(slots2[i])) break;
                if (std::find(adapter.begin(), adapter.end(), slots2[i]) == adapter.end()) {
                    adapter.push_back(slots2[i]);
                }
            }
        }
        logger::info("  --- A4 code dump: MovementAvoidBoxEventAdapter, {} functions ---", adapter.size());
        for (auto fn : adapter) DumpCode(fn, 0x200);

        // A5: the manager's INTERNAL helpers, from the call graph reconstructed
        // offline over the A3 dump. These are non-virtual, so no vtable walk can
        // reach them; they are the only remaining place an insert can live.
        //   +0x735FF0 <- called by the cell-attach event sink (the node BUILDER)
        //   +0x7363D0 / +0x736250 / +0x736800 <- called by the radius query
        //   +0x7360F0 <- called by the hash lookup
        // VR 1.4.15 offsets: diagnostic only, and every one is IsCode()-gated
        // and page-checked, so a wrong offset logs nothing rather than crashing.
        constexpr std::uintptr_t kHelpers[] = {
            0x735FF0, 0x7360F0, 0x736250, 0x7363D0, 0x736800, 0x736430
        };
        logger::info("  --- A5 code dump: manager internals (VR 1.4.15 offsets) ---");
        for (auto off : kHelpers) {
            const auto fn = g_module.base + off;
            if (!IsCode(fn)) {
                logger::info("  helper +0x{:X}: not in an executable section - "
                             "wrong runtime? skipped", off);
                continue;
            }
            DumpCode(fn, 0x200);
        }
    }
    logger::info("=== avoid probe A1 done ({} candidates) ================================", hits);
}

// Phase A part 2 — needs a live, loaded actor that is currently travelling, so
// the driver calls it once per game session on the first GoTo.
void ProbeActorAgents(RE::Actor* a_actor)
{
    InitModuleRange();
    if (!g_module.base || !a_actor) return;

    logger::info("=== TIER-3 AVOID PROBE (A2: per-actor agents) ==========================");
    logger::info("actor {:08X} '{}'", a_actor->GetFormID(), a_actor->GetName());

    auto& rt = a_actor->GetActorRuntimeData();
    auto* ctrl = rt.movementController.get();
    if (!ctrl) {
        logger::info("  no movementController — actor not under AI movement right now");
        logger::info("=== avoid probe A2 done (no controller) ===============================");
        return;
    }
    logger::info("  movementController @ 0x{:016X}", reinterpret_cast<std::uintptr_t>(ctrl));
    if (Readable(ctrl, 8)) {
        logger::info("  controller vtable {}", Rel(*reinterpret_cast<std::uintptr_t*>(ctrl)));
    }
    HexDump("ctrl", ctrl, 0x80);

    const auto known = BuildKnownVTables();
    // 0x400 covers MovementControllerNPC (0x1D0 flat) with generous headroom for
    // whatever VR adds; every read inside is page-checked.
    SweepForKnownObjects("ctrl", ctrl, 0x400, known);

    // A3 (measured 2026-08-18): the direct sweep finds nothing because the
    // controller's agent/arbiter BSTSmallArray buffers are HEAP-allocated -
    // the ActiveSmartPtrs live one hop away. Follow every heap-pointer-looking
    // qword in the controller window and sweep the pointed block as well.
    {
        const auto* q = reinterpret_cast<const std::uintptr_t*>(ctrl);
        int hops = 0;
        for (std::size_t i = 0; i < 0x400 / 8 && hops < 48; ++i) {
            if (!Readable(&q[i], 8)) break;
            const auto cand = q[i] & ~static_cast<std::uintptr_t>(1);
            if (cand < 0x10000 || g_module.Contains(cand)) continue;
            if (!Readable(reinterpret_cast<void*>(cand), 0x200)) continue;
            ++hops;
            const int f = SweepForKnownObjects(
                fmt::format("ctrl->[+{:03X}]", i * 8).c_str(),
                reinterpret_cast<void*>(cand), 0x200, known);
            (void)f;
        }
        logger::info("  (depth-2: followed {} heap blocks)", hops);
    }

    logger::info("=== avoid probe A2 done ================================================");
}

}  // namespace standpoint_avoid
