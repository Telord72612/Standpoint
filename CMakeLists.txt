cmake_minimum_required(VERSION 3.21)

project(Standpoint LANGUAGES CXX VERSION 1.1.0)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---------------------------------------------------------------------------
# Standpoint - "stand exactly here, facing exactly this way" for NPCs.
#
# An arrival framework: given a destination (an NPC, furniture, an object, a
# marker) and a desired relation (side + final facing), it produces a validated
# stand-slot and takes the actor there with the game's own travel packages.
# One DLL for Skyrim SE, AE and VR (CommonLibSSE-NG multi-runtime).
# See README.md and BUILD.md.
# ---------------------------------------------------------------------------

set(COMMONLIB_PATH "" CACHE PATH "Path to a CommonLibSSE-NG source checkout with VR support (required)")
if(NOT COMMONLIB_PATH)
    message(FATAL_ERROR "Set COMMONLIB_PATH: cmake --preset vr -DCOMMONLIB_PATH=\"C:/path/to/CommonLibSSE-NG\"")
endif()

set(ENABLE_SKYRIM_SE ON  CACHE BOOL "" FORCE)
set(ENABLE_SKYRIM_AE ON  CACHE BOOL "" FORCE)
set(ENABLE_SKYRIM_VR ON  CACHE BOOL "" FORCE)
set(BUILD_TESTS      OFF CACHE BOOL "" FORCE)

add_subdirectory("${COMMONLIB_PATH}" CommonLibSSE EXCLUDE_FROM_ALL)

add_library(Standpoint SHARED
    src/main.cpp          # SKSE entry, logging, S0 settings probe
    src/PapyrusAPI.cpp    # S1+S2 natives
    src/SlotSolver.cpp    # S2: the read-only slot + heading solver
    src/ApiImpl.cpp       # S3: the IStandpoint1 inter-plugin API (STPT handshake)
    src/Interrupts.cpp    # v0.5: bump + HIGGS-grab recognition (observation only)
    src/Threats.cpp       # v0.6 Tier 1: corridor threat scan (observation only)
    src/AvoidProbe.cpp    # v0.6 Tier 3 Phase A: avoid-node reconnaissance (read-only)
    src/MoveApi.cpp       # v0.8 the move API: IStandpoint2::RequestMove mailbox to the Papyrus driver
    src/TestCmd.cpp       # v0.8 the stpt console test of the move API
)

target_precompile_headers(Standpoint PRIVATE src/PCH.h)
target_link_libraries(Standpoint PRIVATE CommonLibSSE::CommonLibSSE)

set_target_properties(Standpoint PROPERTIES
    OUTPUT_NAME "Standpoint"
    DEBUG_POSTFIX ""
)

# Optional: copy the built DLL straight into your mod folder (build with the game closed).
set(MOD_OUTPUT "" CACHE PATH "Optional: copy the built DLL into this folder (your mod's SKSE/Plugins)")
if(MOD_OUTPUT)
    add_custom_command(TARGET Standpoint POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory "${MOD_OUTPUT}"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different "$<TARGET_FILE:Standpoint>" "${MOD_OUTPUT}/"
        COMMENT "Copying Standpoint.dll to the mod folder"
    )
endif()
