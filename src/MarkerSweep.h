#pragma once
// ─────────────────────────────────────────────────────────────────────────────
//  MarkerSweep.h — 1.6.0 BUG 2, the marker litter (report "1.6.0 Plan" S1 §5; spec D §1 item 2).
//
//  Up to 1.5.1 the driver's Setup() made its marker arrays new at EVERY game load, so each load could leave up to 8 more
//  persistent IdleMarker references (Standpoint.esp 0x802 / 0x80F, PlaceAtMe'd: FF FormIDs) in the save, never deleted.
//  1.6.0's Setup keeps its markers and ends with Standpoint.MarkerSweep(<the kept ones>): on the main thread the DLL walks
//  the game's form map and COUNTS the other references of those two bases (the litter) - one log line. Only with
//  Data/SKSE/Plugins/Standpoint.ini [Cleanup] SweepMarkers = 1 (default 0: count only - the user decides after he has seen
//  the count) it disables them and marks them deleted (Disable + SetDelete(true): the save drops them).
//
//  Also the small Standpoint.ini reader the 1.6.0 modules share ([Session] Steered, [Cleanup] SweepMarkers).
// ─────────────────────────────────────────────────────────────────────────────

namespace standpoint_ini {
    // Data/SKSE/Plugins/Standpoint.ini [a_section] a_key as an integer; a_default when the file, the section or the key is
    // missing or the value is not a whole number. a_note says what was found (for the log). Any thread; reads the file.
    int ReadInt(const char* a_section, const char* a_key, int a_default, std::string& a_note);
}

namespace standpoint_markers {
    // class Standpoint: MarkerSweep(ObjectReference[] akKeep) - the driver's Setup, once per load
    bool RegisterNatives(RE::BSScript::IVirtualMachine* a_vm);
}
