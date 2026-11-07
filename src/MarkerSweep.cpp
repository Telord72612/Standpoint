// ─────────────────────────────────────────────────────────────────────────────
//  MarkerSweep.cpp — 1.6.0 BUG 2: count (and, on the user's word, sweep) the driver's orphaned IdleMarker references.
//  See MarkerSweep.h. Report "1.6.0 Plan" S1 §5; spec D §1 item 2.
//
//  The native runs on a Papyrus VM thread: it only copies the kept FormIDs and queues ONE SKSE task. The census runs in that
//  task (main thread): the game's form map is read under its own read lock and only FormIDs are collected; the optional
//  sweep looks each one up again afterwards (no form pointer is kept past the lock). Pattern for the delete: Disable() +
//  SetDelete(true) - a PlaceAtMe'd FF reference is otherwise saved forever (KNOWLEDGEBASE, "MoveableStatic ... PlaceAtMe").
// ─────────────────────────────────────────────────────────────────────────────
#include "PCH.h"
#include "MarkerSweep.h"

#include <atomic>
#include <format>
#include <fstream>
#include <string_view>

namespace logger = SKSE::log;

namespace standpoint_ini {

int ReadInt(const char* a_section, const char* a_key, int a_default, std::string& a_note)
{
    constexpr const char* kIniPath = "Data/SKSE/Plugins/Standpoint.ini";
    std::ifstream in(kIniPath);
    if (!in) {
        a_note = std::format("{} not found - [{}] {} = {}", kIniPath, a_section, a_key, a_default);
        return a_default;
    }
    const auto trim = [](std::string_view a_s) {
        while (!a_s.empty() && std::isspace(static_cast<unsigned char>(a_s.front()))) a_s.remove_prefix(1);
        while (!a_s.empty() && std::isspace(static_cast<unsigned char>(a_s.back()))) a_s.remove_suffix(1);
        return a_s;
    };
    const auto same = [](std::string_view a_a, std::string_view a_b) {
        if (a_a.size() != a_b.size()) return false;
        for (std::size_t i = 0; i < a_a.size(); ++i) {
            if (std::tolower(static_cast<unsigned char>(a_a[i])) != std::tolower(static_cast<unsigned char>(a_b[i]))) return false;
        }
        return true;
    };
    std::string line;
    bool        inSection = false;
    while (std::getline(in, line)) {
        if (line.starts_with("\xEF\xBB\xBF")) line.erase(0, 3);    // a UTF-8 mark (Notepad) before the first line
        std::string_view s = trim(line);
        if (s.empty() || s.front() == ';' || s.front() == '#') continue;
        if (s.front() == '[') {
            const auto close = s.find(']');
            inSection = close != std::string_view::npos && same(trim(s.substr(1, close - 1)), a_section);
            continue;
        }
        if (!inSection) continue;
        const auto eq = s.find('=');
        if (eq == std::string_view::npos || !same(trim(s.substr(0, eq)), a_key)) continue;
        std::string_view v = trim(s.substr(eq + 1));
        if (const auto c = v.find_first_of(";#"); c != std::string_view::npos) v = trim(v.substr(0, c));
        int        value = 0;
        bool       ok = !v.empty();
        bool       neg = false;
        std::size_t i = 0;
        if (ok && (v[0] == '-' || v[0] == '+')) {
            neg = v[0] == '-';
            i = 1;
            ok = v.size() > 1;
        }
        for (; ok && i < v.size(); ++i) {
            if (v[i] < '0' || v[i] > '9' || value > 100000) {
                ok = false;
                break;
            }
            value = value * 10 + (v[i] - '0');
        }
        if (!ok) {
            a_note = std::format("{} [{}] {} = {} (not a whole number: {} is used)", kIniPath, a_section, a_key, std::string(v), a_default);
            return a_default;
        }
        value = neg ? -value : value;
        a_note = std::format("{} [{}] {} = {}", kIniPath, a_section, a_key, value);
        return value;
    }
    a_note = std::format("{} has no [{}] {} line - {} is used", kIniPath, a_section, a_key, a_default);
    return a_default;
}

}  // namespace standpoint_ini

namespace standpoint_markers {
namespace {

    constexpr RE::FormID kBaseStandard = 0x802;   // Standpoint.esp: the slot's IdleMarker (the standard wait)
    constexpr RE::FormID kBasePlain    = 0x80F;   // Standpoint.esp: the slot's plain IdleMarker (no idle animation)
    constexpr std::size_t kListMax     = 12;      // FormIDs named in the log line (the rest are counted)

    std::atomic<int> g_sweepSwitch{ -1 };         // [Cleanup] SweepMarkers, read once (-1 = not read yet)
    std::string      g_sweepNote;

    int SweepSwitch()
    {
        int v = g_sweepSwitch.load();
        if (v < 0) {
            std::string note;
            v = standpoint_ini::ReadInt("Cleanup", "SweepMarkers", 0, note) == 1 ? 1 : 0;
            g_sweepNote = note;
            g_sweepSwitch.store(v);
        }
        return v;
    }

    // Main thread (an SKSE task). a_keep = the driver's own markers (its saved arrays) - never counted as litter.
    void Census(std::vector<RE::FormID> a_keep)
    {
        auto* dh = RE::TESDataHandler::GetSingleton();
        auto* bStd = dh ? dh->LookupForm(kBaseStandard, "Standpoint.esp") : nullptr;
        auto* bPlain = dh ? dh->LookupForm(kBasePlain, "Standpoint.esp") : nullptr;
        if (!bStd && !bPlain) {
            logger::info("MARKERS: Standpoint.esp's marker bases 0x802 / 0x80F were not found - no census");
            return;
        }
        const auto t0 = std::chrono::steady_clock::now();
        int        nStd = 0, nPlain = 0, kept = 0, alreadyDeleted = 0, disabled = 0;
        std::vector<RE::FormID> litter;
        {
            const auto& [map, lock] = RE::TESForm::GetAllForms();
            if (!map) {
                logger::info("MARKERS: the game's form map is not available - no census");
                return;
            }
            const RE::BSReadLockGuard guard{ lock };
            for (const auto& [id, form] : *map) {
                if (!form || (id >> 24) != 0xFFu) continue;                       // created at run time (PlaceAtMe): FF......
                if (form->GetFormType() != RE::FormType::Reference) continue;
                auto* ref = static_cast<RE::TESObjectREFR*>(form);
                const auto* base = ref->GetBaseObject();
                if (!base || (base != bStd && base != bPlain)) continue;
                if (base == bStd) {
                    ++nStd;
                } else {
                    ++nPlain;
                }
                if (std::find(a_keep.begin(), a_keep.end(), id) != a_keep.end()) {
                    ++kept;
                    continue;
                }
                if (ref->IsDeleted()) {
                    ++alreadyDeleted;           // marked deleted already (a sweep of an earlier load): the save drops it
                    continue;
                }
                if (ref->IsDisabled()) ++disabled;
                litter.push_back(id);
            }
        }
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        std::string  ids;
        for (std::size_t i = 0; i < litter.size() && i < kListMax; ++i) ids += std::format(" {:08X}", litter[i]);
        if (litter.size() > kListMax) ids += std::format(" (+{} more)", litter.size() - kListMax);
        const int sweep = SweepSwitch();
        logger::info("MARKERS: {} Standpoint idle markers ({} x 0x802, {} x 0x80F), {} kept, {} litter{}{}{} | census {:.1f} ms | {}",
                     nStd + nPlain, nStd, nPlain, kept, litter.size(), litter.empty() ? "" : ":", ids,
                     alreadyDeleted ? std::format(", {} already marked deleted", alreadyDeleted) : std::string(), ms, g_sweepNote);
        if (litter.empty()) return;
        if (sweep != 1) {
            logger::info("MARKERS: [Cleanup] SweepMarkers is not 1 - counted only, nothing changed ({} of them disabled already)", disabled);
            return;
        }
        // the user's go: each one is disabled and marked deleted (looked up again: no pointer is kept past the lock)
        int done = 0, gone = 0;
        for (const auto id : litter) {
            auto* ref = RE::TESForm::LookupByID<RE::TESObjectREFR>(id);
            if (!ref || ref->IsDeleted()) {
                ++gone;
                continue;
            }
            if (!ref->IsDisabled()) ref->Disable();
            ref->SetDelete(true);
            ++done;
        }
        logger::info("MARKERS: SWEEP ([Cleanup] SweepMarkers = 1): {} litter marker(s) disabled + marked deleted{} - the next load should "
                     "count 0 litter", done, gone ? std::format(", {} gone meanwhile", gone) : std::string());
    }

    // class Standpoint: the driver's Setup (once per load): its own markers are a_keep (None entries allowed)
    void MarkerSweepNative(RE::StaticFunctionTag*, std::vector<RE::TESObjectREFR*> a_keep)
    {
        std::vector<RE::FormID> keep;
        keep.reserve(a_keep.size());
        for (auto* r : a_keep) {
            if (r) keep.push_back(r->GetFormID());
        }
        if (auto* task = SKSE::GetTaskInterface()) {
            task->AddTask([keep = std::move(keep)]() mutable { Census(std::move(keep)); });
        } else {
            logger::info("MARKERS: no task interface - no census");
        }
    }

}  // namespace

bool RegisterNatives(RE::BSScript::IVirtualMachine* a_vm)
{
    a_vm->RegisterFunction("MarkerSweep", "Standpoint", MarkerSweepNative);   // 1.6.0 (the driver's Setup)
    return true;
}

}  // namespace standpoint_markers
