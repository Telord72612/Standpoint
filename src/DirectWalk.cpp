// ─────────────────────────────────────────────────────────────────────────────
//  DirectWalk.cpp — v1.4.0: the "chain-pull" walk driver (see DirectWalk.h for the shape).
//
//  ENGINE FACTS USED HERE (facts only - every one is listed in report "1.4.0 Water Walk" A §2 with its source; the
//  code below is written for Standpoint, nothing is copied from PLANCK / VR Chain / PPB):
//    * the hook site: the CALL at SE id 36359 + 0xE5 (VR 0x5E0885), a 5-byte E8 call to `void (Actor*)` - the per-actor
//      motion / animation-driven check. Three shipped VR mods chain it; each skips the rest of the chain only for the
//      actors it drives (the engine's own check would end planner direct control every frame).
//    * MovementControllerNPC (Actor runtime data `movementController`): +0x128 the motion-driven interface (slots 1 to
//      high process, 2 from high, 3 animation-driven, 4 motion-driven, 5 allow rotation, 6/7/8 the three "is" tests),
//      +0x140 the planner-direct interface (slots 1 take, 2 travel direction [euler radians], 3 speed [NORMALISED:
//      1.0 = her walk], 4 facing [absolute euler radians], 5 release); +0x1C0 movement type (8 = planner direct),
//      +0x1C7 the planner flag, +0x1C9 high process. The same layout on SE / AE / VR (only the Actor offsets move:
//      CommonLib's accessors are used for those).
//    * the enter order: to high process -> take the planner -> motion-driven; then check that it took.
//      The exit: clear OUR flag whenever it is still set, forget unconditionally, re-evaluate her package LATER (an
//      inline evaluate from the hook froze the game once - 2026-08-29), never "only while she still reads as planner
//      direct" (a stagger changes her movement type first: our flag stayed behind for 16 s - KNOWLEDGEBASE).
//    * "her animation owns her feet" = the graph variables bAnimationDriven / bAllowRotation (not the controller's own
//      IsAnimationDriven, which is true for any NPC walking a package).
//    * the arbiter's two parameter CALLs (VR 0x116D362 speeds, 0x116D39D turns; no address-library id): a parameter
//      object with 9 virtual slots (0 dtor, 1 walk/run share, 2 accel, 3 decel, 4 turn accel, 5 rotation share, 6 type,
//      7/8 write/read); the shipped values for a driven actor are accel 10, decel 10, turn accel 10, rotation 2.5.
//    * ActorState::NormalizeSpeed = SE id 88499 (VR 0x1120AA0): u/s -> normalised (used for the LOG only here).
//    * measured: a direct-controlled NPC told less than her walk speed does not move (24 u/s -> 0; 88 -> 89): she is
//      told 1.0 (her walk) or 0, never in between.
//  1.5.0 (report "1.5.0 Plan" X §1-§3, D §3):
//    * the hook site's container is Actor::UpdateMovement = SE id 36359 = AE id 37350 (VR 0x5E07A0, the SE id through the
//      VR CSV) - REL::RelocationID(36359, 37350): REL::ID(36359) does not exist in the AE tables (a fatal lookup there).
//      Its "apply movement" CALL is public (9+ SE/AE mods hook it): SE +0xF0, AE +0xFB, VR +0xF0. The motion-check CALL is
//      the last CALL before it (VR +0xE5, proven) - found at run time by reading the bytes; nothing else is assumed.
//    * CommonLib's IMovementState::CanStrafe has AE id 0 (a fatal lookup on AE): called on VR only.
//    * a door: base form Door; GetOpenState (SE 14180 / AE 14288) 1 open, 2 opening, 3 closed, 4 closing; an NPC opens one
//      with the engine's activate (ActivateRef, SE 19369 / AE 19796) with her as the activator - only a CLOSED one (the
//      activate toggles: an open door would close). Never a load door (ExtraTeleport: the activate would take her through).
//  1.5.1 (report "1.5.1 Plan" D §1, §2, §5):
//    * a STANDING NPC plays random idle clips that set the graph's animation-driven flags (the 1.5.0 VR run: two walks ended
//      animation-lock, one a correctly planned bath wade); "IdleForceDefaultState" sent to her graph ends the clip - the event
//      Standpoint's own session let-go sends (Debug.SendAnimationEvent = IAnimationGraphManagerHolder::NotifyAnimationGraph).
//    * the planner-direct speed is NORMALISED: 1.0 = her walk (~83-90 u/s measured for Sofia), ~2.0 = her full run (PPB's
//      measure); never a value between 0 and her walk (she does not move).
//    * PLANCK = activeragdoll.dll (it leads an NPC the player's HIGGS hand grabs); PPB = PPB.dll (its push walk takes planner
//      direct control while a hand presses her).
//  1.6.0 (report "1.6.0 Plan" D §1 item 7, S2 §3, S1 §7 D1-D3): the speed bands by route left (run told 2.0 - its speed under
//  direct control is unmeasured: the trace's yaw rate and speed tune it), the turn caps and slow zones, the look-ahead by speed,
//  the chunked route (Extend / the wait at the verified end), Swap (a new route on the move), faceOffsetRad / faceAll. Nothing
//  here sets a position: Swap and Extend only change the route the pure pursuit follows.
// ─────────────────────────────────────────────────────────────────────────────
#include "PCH.h"
#include "DirectWalk.h"
#include "PathPlanner.h"     // 1.5.0 (review R2 S1 / S3): standpoint_path::IsOpenableDoor - ONE door rule for planner and driver

#include <array>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <format>
#include <fstream>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace logger = SKSE::log;

namespace standpoint_interrupts {
    bool IsPlayerGrabbing(RE::Actor* a_actor);   // Interrupts.cpp: two atomics, safe from any thread
}

namespace standpoint_direct {
namespace {

    // ── addresses ─────────────────────────────────────────────────────────────────────────────────────────────────
    constexpr std::uint64_t  kMotionCheckFnId    = 36359;      // Actor::UpdateMovement, SE id (VR: through the VR CSV)
    constexpr std::uint64_t  kMotionCheckFnIdAE  = 37350;      // ... its AE id (1.5.0: never REL::ID(36359) on AE - fatal)
    constexpr std::uintptr_t kMotionCallOffset   = 0xE5;       // the motion-check CALL inside it - VR 0x5E0885 (proven)
    constexpr std::uintptr_t kApplyOffsetSE      = 0xF0;       // the "apply movement" CALL (public, 9+ SE/AE mods hook it)
    constexpr std::uintptr_t kApplyOffsetAE      = 0xFB;
    constexpr std::uintptr_t kApplyOffsetVR      = 0xF0;
    // (review R2 S5) the motion-check CALL's PINNED candidate offsets per runtime - no scan over arbitrary bytes. VR +0xE5 is
    // the proven site; SE = the VR body (X §3a.2, 90 %); AE's anchor moved 11 bytes (+0xFB): +0xF0 first, then +0xE5 (75 %).
    // Each is verified (E8, a real target, not the anchor's own target); the first that passes is the site.
    constexpr std::uintptr_t kSiteVR[]           = { 0xE5 };
    constexpr std::uintptr_t kSiteSE[]           = { 0xE5 };
    constexpr std::uintptr_t kSiteAE[]           = { 0xF0, 0xE5 };
    constexpr std::size_t    kSelfTestDump       = 0x120;      // bytes of Actor::UpdateMovement logged by the self-test
    constexpr std::size_t    kCalleeDump         = 0x40;       // ... and of the callee, when it is the engine's own
    constexpr std::uintptr_t kArbiterSpeedsVR    = 0x116D362;  // CALL CalculateSpeedsWithAcceleration (VR only, no id)
    constexpr std::uintptr_t kArbiterTurnsVR     = 0x116D39D;  // CALL CalculateRotSpeeds (VR only, no id)
    constexpr std::uint64_t  kNormalizeSpeedId   = 88499;      // ActorState::NormalizeSpeed (log only)
    // ⚠ CommonLibVR 8.3.0: SKSE::AllocTrampoline works ONCE (a call_once inside; a later call silently does nothing).
    // Standpoint allocates in exactly one place (Install, below) and this size must cover EVERY write_call<5> it makes:
    // three today (the motion check + the two arbiter CALLs) x 14 bytes = 42. (Interrupts.cpp's own pair-filter hook is a
    // vtable write - no trampoline.) A new trampoline hook anywhere in Standpoint = raise this constant, never a 2nd alloc.
    constexpr std::size_t    kTrampolineBytes    = 64;
    // 1.5.0 the SE / AE switch (read once at SKSEPlugin_Load; Skyrim VR ignores it): 0 = the self-test only, 1 = install + walk
    constexpr const char*    kIniPath            = "Data/SKSE/Plugins/Standpoint.ini";

    // ── the movement controller (same on SE / AE / VR) ────────────────────────────────────────────────────────────
    constexpr std::ptrdiff_t kMcMotionIface      = 0x128;
    constexpr std::ptrdiff_t kMcPlannerIface     = 0x140;
    constexpr std::ptrdiff_t kMcMoveType         = 0x1C0;
    constexpr std::ptrdiff_t kMcPlannerFlag      = 0x1C7;
    constexpr std::ptrdiff_t kMcHighProcess      = 0x1C9;
    constexpr std::uint32_t  kTypePlannerDirect  = 8;

    // ── the driven actor's movement parameters (the shipped VR values) ────────────────────────────────────────────
    constexpr float kParamAccel     = 10.f;
    constexpr float kParamDecel     = 10.f;
    constexpr float kParamTurnAccel = 10.f;
    constexpr float kParamTurnShare = 2.5f;

    // ── the driver's own numbers (proposals of report A §6, dialled in VR from the trace) ─────────────────────────
    constexpr int   kSlots           = 16;      // walkers the hook can know at once (a walk's slot lingers until handed back)
    constexpr int   kStaleVisits     = 6;       // hook visits without a fresh order -> the order is dead: hand her back
    constexpr float kWalkNorm        = 1.0f;    // her own walk (normalised); never a value between 0 and this
    constexpr float kSteerMinSpeed   = 0.01f;   // the travel direction is written only above this speed
    constexpr float kLookAhead       = 48.f;    // pure pursuit: the aim point this far ahead along the route (u) - 1.6.0: the
                                                // look-ahead at her walk (and near a kPtNoCut corner); it grows with her speed
    // told 0 this near the last point (`atEnd` -> settle; the walk's and the re-approach's own stop). v1.4.1 (the session's
    // decision, 2026-11-03): 12 -> 6 u - the 1.4.0 run slid ~1 u after "told 0" at 39 u/s and stopped 11 u short at 12 u;
    // 6 u keeps margin and should land her ~5 u from the point. (C 0 = within 32 u, WalkApi kArriveRadius: not derived
    // from this; kCornerReach below is the SHARP-corner rule, not this.)
    constexpr float kStopRadius      = 6.f;
    constexpr float kCornerReach     = 12.f;    // a SHARP corner counts as reached this near
    constexpr float kTurnFirstDeg    = 90.f;    // heading error above this: stand and turn first
    constexpr float kTurnDoneDeg     = 30.f;    // ... until below this
    constexpr float kCornerTurnDeg   = 45.f;    // after a SHARP corner: a turn larger than this is done standing
    constexpr float kTurnMaxS        = 2.0f;    // a stand-and-turn that has not finished by then walks on (she turns walking)
    constexpr float kSettleSpeed     = 5.f;     // u/s: she has stopped
    constexpr float kSettleMaxS      = 1.0f;
    constexpr float kFaceOkDeg       = 3.f;     // final facing reached
    constexpr float kFaceSetDeg      = 5.f;     // still off by more after kFaceMaxS: her heading is set directly (0.8.1 rule)
    constexpr float kFaceMaxS        = 1.5f;
    constexpr float kStrayDist       = 64.f;    // cross-track: pushed this far off the line = strayed
    constexpr float kProgressMin     = 10.f;    // the progress watchdog: at least this much along the route ...
    constexpr float kProgressWindowS = 1.5f;    // ... in this long while told to walk and driven
    constexpr int   kStarveTicks     = 30;      // frames without a hook visit while we want her = someone upstream has her
    constexpr float kOtherDriverS    = 1.0f;    // "another system drives her" seen this long = give her up
    constexpr float kAnimLockS       = 2.0f;    // "her animation owns her feet" seen this long = animation lock
    constexpr int   kMaxRetakes      = 12;      // lost control this often IN A ROW (no steady driven visit between) = give up
    constexpr float kWarpDist        = 200.f;   // her root jumped this far in one frame = a warp: never chased
    constexpr float kSwimLevel       = 0.70f;   // the engine swims an actor at this submerged level (report 14 §4)
    // A stall's bad spot (review H #1 + G BUG 3): on the route ahead of her, at her radius + the planner's bad-spot pad
    // (PathPlanner kXBadPad 24 - keep the two equal) + 8 u. The planner refuses cells within r + 24 u of the spot, so the
    // refused disc starts 8 u in front of her centre - it covers what blocked her body - and her own spot stays legal (the
    // re-plan starts there). The planner exempts only a spot within 16 u of a plan's start (kXBadUnderHer): never this one.
    constexpr float kPlannerBadPad   = 24.f;
    constexpr float kStallClear      = 8.f;
    constexpr float kTraceS          = 0.5f;    // the trace line, while she moves
    constexpr float kStandTraceS     = 2.0f;    // ... while she stands, for the first kStandTraceLongAfterS
    constexpr float kStandTraceLongS = 30.f;    // ... then this seldom (a hold -1 can last for hours)
    constexpr float kStandTraceLongAfterS = 30.f;
    constexpr float kStopTraceDist   = 64.f;    // every frame is logged from this near the end (the overshoot question)
    constexpr int   kStopTraceMax    = 60;
    // v1.4.1 ARRIVAL PRECISION. The first VR run (2026-11-03, WALK #7424706): told 0 at 12 u she stopped 11 u short (about
    // 1 u of overshoot at 39 u/s), then the ~145 deg turn in place to the final facing moved her ~10 u - ARRIVED 21.6 u. A
    // walker who CAN STRAFE (planner direct control steers her travel direction and her facing apart) turns while she still
    // walks: over the last kFaceBlendDist of the route her facing target blends from the travel direction to the final
    // heading (the direction keeps pointing at the point), and the target never moves faster than kFaceBlendRate - she
    // arrives already turned, and the face phase has (almost) nothing left to turn. Still more than kReapproachMin from the
    // point after the facing: ONE short re-approach (walked at her walk, facing held), then the facing again - never twice.
    constexpr float kFaceBlendDist   = 48.f;    // u of route before the point (= kLookAhead: the aim is the point itself)
    constexpr float kFaceBlendRate   = 3.1415927f;   // rad/s: the facing target turns at most 180 deg a second (never a snap)
    constexpr float kReapproachMin   = 16.f;    // after the facing still farther than this from the point: one re-approach ...
    constexpr float kReapproachMax   = 64.f;    // ... unless she is this far off (= kStrayDist: pushed away - she arrives as is)
    constexpr float kStandReFaceDeg  = 15.f;    // (Params::faceWho) standing, she re-faces the reference once it is this far off
    // review R (1.4.1): the blend is LATCHED - on once started (for the rest of the track: no snap back, no stand-and-turn
    // in the middle of it), and OFF for the rest of the walk once it makes her stop moving or leave her steered direction
    // (the engine would then not strafe her: 1.4.0's walk + turn at the point instead, never a walk <-> turn cycle)
    constexpr float kBlendCheckS     = 0.5f;    // trouble seen this long while blending = blend OFF
    constexpr float kBlendSlowNorm   = 0.5f;    // ... she moves under half her walk (normalised) while told 1.0 ...
    constexpr float kBlendOffDirDeg  = 45.f;    // ... or she moves more than this off the steered direction ...
    constexpr float kBlendMoveMin    = 10.f;    // ... (the direction test only while she moves at least this, u/s)
    constexpr float kBlendFlipRad    = 2.6179939f;   // 150 deg: past this the blend keeps the turn side it started with
    constexpr float kHoldFinalRoute  = 96.f;    // a strafer whose facing is nearer the final heading than the route keeps it
                                                // all the way on a route this short (2 x the blend): no turn there and back
    constexpr float kReapproachOffWay = 24.f;   // the re-approach only from within this of her planned last leg (a slide
                                                // along the way the planner tested - never an untested line after a push)
    // 1.5.0 DOORS. A swinging leaf turns about its origin (the hinge - KNOWLEDGEBASE: "a door leaf ... origin on the hinge"),
    // so it sweeps the disc round the ref's position out to its farthest bound corner (x scale). She stops OUTSIDE that sweep
    // (+ her radius + this pad - review R2 S4), opens it, waits at most kDoorWaitS (a detection window, not a timer).
    constexpr float kDoorPad         = 16.f;
    constexpr float kDoorSwingMin    = 32.f;    // the leaf's reach from its hinge is taken within these limits
    constexpr float kDoorSwingMax    = 192.f;
    constexpr float kDoorBehind      = 16.f;    // a doorway more than this behind her along the route is passed: forgotten
    constexpr float kDoorInside      = 8.f;     // (log) she is this far or more inside the sweep when she meets the door
    constexpr float kDoorWaitS       = 2.0f;
    constexpr float kDoorMaxS        = 8.0f;    // 1.6.1 (B2): the whole door phase at most (a closing leaf, two activates, their waits)
    // (verify V: S2 partly + N3) THE DOORS OF A ROUTE - every door whose closed-leaf line (its doorway) a leg of her route
    // crosses: the planner's door points (closed at plan time) AND doors that were OPEN at plan time (no door point - they may
    // have shut since). Found once per route in her cell(s) - the planner's own placed-water scan pattern.
    constexpr float kDoorScanPad     = 128.f;   // the reference search box = the route's box + this
    constexpr int   kDoorScanMaxRefs = 4096;    // exterior cells only: references looked at (an interior is looked at whole)
    constexpr float kDoorCrossPad    = 8.f;     // the doorway line is lengthened this much at each end for the crossing test
    constexpr float kDoorFloorDz     = 96.f;    // a door counts only on her floor: its base this near the leg's height there
    constexpr float kDoorLeafMinH    = 48.f;    // a standing leaf only (never a floor hatch): at least this tall ...
    constexpr float kDoorLeafTall    = 0.75f;   // ... and at least this x its width
    // 1.6.0 THE SPEED BANDS (spec D §1 item 7, S2 §3; the user, 2026-11-06: "jog until 3m, run until15m"; a band that would last
    // under 1 s is skipped - the question tool). By route LEFT (toGo = the route left + Params::remainAfterU). PPB's measure:
    // normalised 1.0 = her walk, ~2.0 her full run (1.5.1 run: told 1.5 = 212-221 u/s; the far phase's engine run 290-349 u/s).
    constexpr float kJogNorm         = 1.5f;    // her jog (normalised) - "it looked great" (1.5.1)
    constexpr float kRunNorm         = 2.0f;    // her run (normalised) - untested under direct control before 1.6.0 (S2 F16)
    constexpr float kRunAboveU       = 1050.f;  // 15 m: she runs while more route than this is left ...
    constexpr float kJogAboveU       = 210.f;   // ... jogs down to 3 m, walks the last 3 m (the 6 u stop, the facing, the re-approach)
    constexpr float kMinBandS        = 1.0f;    // a band is entered only if it would last this long at its own speed
    constexpr float kWalkUps         = 98.f;    // her speeds per band for the 1 s rule and the ramp leads when not measured (Sofia:
    constexpr float kJogUps          = 220.f;   // told 1.0 ~98 u/s, told 1.5 212-229 u/s, told 2.0 ~360 u/s - R2's fit, S2 §3.1)
    constexpr float kRunUps          = 360.f;
    constexpr float kSpeedRampPerS   = 1.5f;    // normalised per second, both ways: walk <-> jog in a third of a second, run -> walk
                                                // in 0.7 s; the downshifts are ANTICIPATED by the distance the ramp takes
    constexpr float kSlowLeadU       = 128.f;   // a slow zone / turn cap applies from this far ahead (route length) + the ramp lead
                                                // from her band to the capped one (~280 u at a run before a walked turn)
    constexpr float kRunTurnDeg      = 30.f;    // the turn caps at a route point: up to this may be run ...
    constexpr float kSlowTurnDeg     = 60.f;    // ... up to this jogged; sharper is walked (the 64 u push rule must never mistake a
                                                // wide fast turn for a push)
    constexpr float kNoCutTurnDeg    = 15.f;    // a kPtNoCut corner (its 128 u chord failed) turning more than this: jog at most,
                                                // look-ahead 48 u
    constexpr float kSteepRatio      = 0.5f;    // a leg steeper than this rise / run (27 deg), up or down: jog at most
    constexpr float kSteepMinLegU    = 8.f;     // (legs shorter than this are not judged for steepness)
    constexpr float kInWaterLvl      = 0.05f;   // 1.6.1 (B4): "she is in the water" from this submerged level (~7 u for Sofia)
    constexpr float kBedSlope        = 0.25f;   // ... a water point's edge lies its depth / this before it (a gentle 14 deg bed) ...
    constexpr float kWaterEdgeMinU   = 64.f;    // ... at least this far before it
    constexpr float kLookAheadS      = 0.35f;   // 1.6.0 the pure-pursuit look-ahead = this x her measured speed ...
    constexpr float kLookAheadMax    = 128.f;   // ... between kLookAhead (48 u, her walk) and this (her run: the planner chord-tests
                                                // corners over +-128 u too - kPtNoCut when that fails)
    constexpr float kYawRateK        = 0.25f;   // the yaw-rate trace's smoothing window, s

    constexpr std::uint32_t kFlagSharp = standpoint_path::kPtSharp;   // the planner's point flags (PathPlanner.h PointFlag) ...
    constexpr std::uint32_t kFlagEdge  = standpoint_path::kPtEdge;    // ... a drop / a hole within one cell
    constexpr std::uint32_t kFlagNoCut = standpoint_path::kPtNoCut;   // ... 1.6.0: this corner's +-128 u chord failed (S2 §3.3)
    constexpr std::uint32_t kFlagStair = standpoint_path::kPtStair;   // ... 1.6.0: the leg INTO it uses the stair allowance (S2 §5.3)
    constexpr std::uint8_t  kKindWater = static_cast<std::uint8_t>(standpoint_path::Kind::kWater);
    constexpr std::uint32_t kVrChainMsg = 0x56434842u;   // 'VCHB' - VR Chain's bind-interface handshake

    constexpr float kRadToDeg = 57.2957795f;
    constexpr float kPi = 3.14159265f;

    // ═══ the engine interfaces, re-declared (the slot ORDER and argument types are the engine's; names are ours) ═══
    struct EngineIface {
        virtual ~EngineIface() = default;                                  // 0 (never called)
    };
    struct MotionModes : EngineIface {
        virtual void ToHighProcess() = 0;                                  // 1
        virtual void FromHighProcess() = 0;                                // 2
        virtual void AnimationDrives() = 0;                                // 3
        virtual void MotionDrives() = 0;                                   // 4
        virtual void RotationPasses() = 0;                                 // 5
        virtual bool IsMotionDriven() = 0;                                 // 6
        virtual bool IsAnimationDriven() = 0;                              // 7
        virtual bool IsRotationAllowed() = 0;                              // 8
    };
    struct PlannerSteer : EngineIface {
        virtual void TakePlanner() = 0;                                    // 1
        virtual void SteerDirection(const RE::NiPoint3& a_euler) = 0;      // 2
        virtual void SteerSpeed(float a_normalised) = 0;                   // 3
        virtual void SteerFacing(const RE::NiPoint3& a_euler) = 0;         // 4
        virtual void ReleasePlanner() = 0;                                 // 5
    };
    struct MoveParamSlots {
        virtual ~MoveParamSlots() = default;                               // 0
        virtual float         WalkRunShare() = 0;                          // 1
        virtual float         Acceleration() = 0;                          // 2
        virtual float         Deceleration() = 0;                          // 3
        virtual float         TurnAcceleration() = 0;                      // 4
        virtual float         RotationShare() = 0;                         // 5
        virtual std::uint32_t ParamType() = 0;                             // 6
        virtual void          WriteOut(void*) = 0;                         // 7
        virtual void          ReadIn(void*) = 0;                           // 8
    };
    // What the arbiter is handed for a walker we drive: her own walk/run share and type, Standpoint's response values.
    class DrivenParams final : public MoveParamSlots {
    public:
        explicit DrivenParams(MoveParamSlots* a_hers) : walkRun(a_hers->WalkRunShare()), type(a_hers->ParamType()) {}
        float         WalkRunShare() override { return walkRun; }
        float         Acceleration() override { return kParamAccel; }
        float         Deceleration() override { return kParamDecel; }
        float         TurnAcceleration() override { return kParamTurnAccel; }
        float         RotationShare() override { return kParamTurnShare; }
        std::uint32_t ParamType() override { return type; }
        void          WriteOut(void*) override {}
        void          ReadIn(void*) override {}
    private:
        float         walkRun;
        std::uint32_t type;
    };

    // VR Chain's public bind interface (its HavokBind.h, the user's own code): only the first three slots are used.
    struct VrChainQueries {
        virtual std::uint32_t BuildNumber() = 0;                           // 0
        virtual bool          IsBoundToWorld(std::uint32_t a_ref) = 0;     // 1
        virtual std::int32_t  ChainCount(std::uint32_t a_actor) = 0;       // 2
    };

    // ═══ the controller, read ═══
    std::uintptr_t ControllerOf(RE::Actor* a_actor)
    {
        return reinterpret_cast<std::uintptr_t>(a_actor->GetActorRuntimeData().movementController.get());
    }
    MotionModes*  Modes(std::uintptr_t a_mc) { return reinterpret_cast<MotionModes*>(a_mc + kMcMotionIface); }
    PlannerSteer* Steer(std::uintptr_t a_mc) { return reinterpret_cast<PlannerSteer*>(a_mc + kMcPlannerIface); }
    bool PlannerFlag(std::uintptr_t a_mc) { return *reinterpret_cast<const std::uint8_t*>(a_mc + kMcPlannerFlag) != 0; }
    std::uint32_t MoveType(std::uintptr_t a_mc) { return *reinterpret_cast<const std::uint32_t*>(a_mc + kMcMoveType); }
    bool HighProcess(std::uintptr_t a_mc) { return *reinterpret_cast<const std::uint8_t*>(a_mc + kMcHighProcess) != 0; }
    // planner direct RIGHT NOW = high process AND movement type 8 AND the flag (a bare flag drives nobody)
    bool PlannerDirectNow(std::uintptr_t a_mc) { return HighProcess(a_mc) && MoveType(a_mc) == kTypePlannerDirect && PlannerFlag(a_mc); }

    const RE::BSFixedString& AnimDrivenVar()
    {
        static const RE::BSFixedString s("bAnimationDriven");
        return s;
    }
    const RE::BSFixedString& AllowRotationVar()
    {
        static const RE::BSFixedString s("bAllowRotation");
        return s;
    }
    // Her feet are free for a motion driver when NEITHER graph variable is set (an idle, an attack, a scene sets them).
    bool FeetAreFree(RE::Actor* a_actor)
    {
        bool anim = false, rot = false;
        a_actor->GetGraphVariableBool(AnimDrivenVar(), anim);
        a_actor->GetGraphVariableBool(AllowRotationVar(), rot);
        return !anim && !rot;
    }
    // 1.5.1 the idle break's graph event (the same one Standpoint's session let-go sends with Debug.SendAnimationEvent)
    const RE::BSFixedString& IdleBreakEvent()
    {
        static const RE::BSFixedString s("IdleForceDefaultState");
        return s;
    }

    // ═══ THE SHARED TABLE (main thread <-> the hook) ═══
    enum class Seen : std::uint8_t {
        kNone = 0,
        kEngaged,       // the enter ritual took: she is ours
        kDriving,       // driven this visit
        kAnimOwns,      // her animation owns her feet: not driven, the chain was called
        kOtherDriver,   // planner direct right now, and not ours: not touched
        kDidNotTake,    // the ritual ran but she did not become planner direct: our flag cleared again
        kLost,          // we drove her and she is no longer planner direct (a stagger, an idle): handed back, re-taken later
        kHandedBack,    // the walk withdrew her: flag cleared, package re-evaluated
        kYieldGrab,     // HIGGS grabbed her: handed back in the same visit, the chain called (PLANCK leads)
        kStale,         // the order went stale (Tick stopped writing): handed back
        kNoController,  // she has no movement controller
    };
    const char* SeenText(Seen a_s)
    {
        switch (a_s) {
        case Seen::kEngaged:      return "ENGAGED - she walks under planner direct control";
        case Seen::kDriving:      return "driving";
        case Seen::kAnimOwns:     return "her animation owns her feet (graph variables) - not driven, the engine's check ran";
        case Seen::kOtherDriver:  return "ANOTHER system drives her (planner direct, not ours) - not touched";
        case Seen::kDidNotTake:   return "the take did not hold (she did not become planner direct) - our flag cleared again";
        case Seen::kLost:         return "LOST her (her movement type changed) - flag cleared, package re-evaluated, re-taken next frame";
        case Seen::kHandedBack:   return "handed back (flag cleared, package re-evaluated)";
        case Seen::kYieldGrab:    return "yielded to the HIGGS grab in the same visit (PLANCK leads her)";
        case Seen::kStale:        return "the order went stale - handed back";
        case Seen::kNoController: return "she has no movement controller";
        default:                  return "nothing yet";
        }
    }

    struct Slot {
        RE::FormID     who{ 0 };            // 0 = free
        std::uintptr_t stateAddr{ 0 };      // her ActorState (the arbiter hooks' test)
        // the order (main thread -> hook)
        bool           wantDrive{ false };
        float          speed{ 0.f };        // normalised
        float          dirZ{ 0.f };         // travel heading, radians
        float          faceZ{ 0.f };        // facing, radians, absolute
        std::uint32_t  orderSeq{ 0 };
        // the hook's side
        std::uint32_t  seenSeq{ 0 };
        std::uint32_t  staleVisits{ 0 };
        bool           owned{ false };      // OUR planner flag is (maybe) set on her controller
        bool           driving{ false };    // the hook drove her at its last visit
        Seen           seen{ Seen::kNone };
        std::uint32_t  visits{ 0 };
        std::uint32_t  engages{ 0 };
        std::uint32_t  losts{ 0 };
        std::uint32_t  handBacks{ 0 };
        std::uint32_t  hookThread{ 0 };
    };

    std::mutex                                g_slotMtx;
    std::array<Slot, kSlots>                  g_slot{};
    std::atomic<int>                          g_inUse{ 0 };               // slots with who != 0: the hook's fast path
    std::array<std::atomic<std::uintptr_t>, kSlots> g_drivenState{};      // her ActorState while the hook drives her
    std::array<std::atomic<RE::FormID>, kSlots>     g_slotWho{};          // a lock-free copy of each slot's `who` (review H
                                                                          // #10): every other actor leaves the hook unlocked
    std::atomic<bool>                         g_hookLive{ false };
    std::atomic<bool>                         g_paramsLive{ false };
    std::atomic<std::uint32_t>                g_hookThread{ 0 };

    // (1.5.0) the motion check is `void (Actor*)` on VR (PLANCK's facts). The hook takes - and hands down the chain - all
    // four argument registers untouched, so a callee found at run time on SE / AE that reads a second argument still gets
    // what the engine passed (the extra registers cost nothing for a one-argument callee).
    using MotionCheckFn = void (*)(RE::Actor*, void*, void*, void*);
    using SpeedsFn      = void (*)(void*, void*, void*, void*, float*, float*, float*);
    using TurnsFn       = void (*)(void*, void*, void*, void*, float*, float*);
    MotionCheckFn g_nextMotion = nullptr;
    SpeedsFn      g_nextSpeeds = nullptr;
    TurnsFn       g_nextTurns  = nullptr;

    // g_slotMtx held
    int SlotOf(RE::FormID a_who)
    {
        if (!a_who) return -1;
        for (int i = 0; i < kSlots; ++i) {
            if (g_slot[i].who == a_who) return i;
        }
        return -1;
    }

    // g_slotMtx held
    void FreeSlotLocked(int a_i)
    {
        if (g_slot[a_i].who) g_inUse.fetch_sub(1, std::memory_order_acq_rel);
        g_slot[a_i] = Slot{};
        g_drivenState[a_i].store(0, std::memory_order_release);
        g_slotWho[a_i].store(0, std::memory_order_release);
    }

    // the hook's lock-free test: is this actor in the table at all?
    bool MaybeOurs(RE::FormID a_id)
    {
        for (const auto& w : g_slotWho) {
            if (w.load(std::memory_order_acquire) == a_id) return true;
        }
        return false;
    }

    // Her package, re-evaluated LATER (never inline from the hook). AddTask from this hook is how the VR-proven drivers do it.
    void QueueEvaluate(RE::FormID a_id)
    {
        if (auto* task = SKSE::GetTaskInterface()) {
            task->AddTask([a_id]() {
                if (auto* a = RE::TESForm::LookupByID<RE::Actor>(a_id)) a->EvaluatePackage(false, false);
            });
        }
    }

    // What the hook learnt at one visit; written back under the lock if the slot still names her.
    struct VisitResult {
        bool  owned{ false };
        bool  driving{ false };
        Seen  seen{ Seen::kNone };
        bool  engaged{ false };
        bool  lost{ false };
        bool  handedBack{ false };
        bool  freeIfDone{ false };
    };

    void WriteBack(int a_i, RE::FormID a_who, const VisitResult& a_r)
    {
        std::scoped_lock lk(g_slotMtx);
        Slot& s = g_slot[a_i];
        if (s.who != a_who) return;                 // cleared (a load) meanwhile
        s.owned = a_r.owned;
        s.driving = a_r.driving;
        s.seen = a_r.seen;
        if (a_r.engaged) ++s.engages;
        if (a_r.lost) ++s.losts;
        if (a_r.handedBack) ++s.handBacks;
        if (a_r.freeIfDone && !s.wantDrive && !s.owned) {
            FreeSlotLocked(a_i);                    // the walk is over and she is handed back: the slot is free
            return;
        }
        g_drivenState[a_i].store(s.driving ? s.stateAddr : 0, std::memory_order_release);
    }

    // ═══ THE HOOK: the per-actor motion check (VR 0x5E0885; 1.5.0 SE / AE: the site the self-test found). Integers and the
    // controller only. a_2..a_4: the engine's other argument registers, handed down the chain as they came. ═══
    void OnMotionCheck(RE::Actor* a_actor, void* a_2, void* a_3, void* a_4)
    {
        if (!a_actor || g_inUse.load(std::memory_order_acquire) == 0) {
            g_nextMotion(a_actor, a_2, a_3, a_4);
            return;
        }
        const RE::FormID id = a_actor->GetFormID();
        if (!MaybeOurs(id)) {                       // not one of ours: no lock taken (review H #10)
            g_nextMotion(a_actor, a_2, a_3, a_4);
            return;
        }
        int  i = -1;
        Slot s{};
        {
            std::scoped_lock lk(g_slotMtx);
            i = SlotOf(id);
            if (i >= 0) {
                Slot& live = g_slot[i];
                ++live.visits;
                if (live.orderSeq != live.seenSeq) {
                    live.seenSeq = live.orderSeq;
                    live.staleVisits = 0;
                } else if (live.staleVisits < 100000) {
                    ++live.staleVisits;
                }
                if (!live.hookThread) live.hookThread = ::GetCurrentThreadId();
                s = live;
            }
        }
        if (i < 0) {
            g_nextMotion(a_actor, a_2, a_3, a_4);
            return;
        }
        if (!g_hookThread.load(std::memory_order_relaxed)) g_hookThread.store(::GetCurrentThreadId(), std::memory_order_relaxed);

        const std::uintptr_t mc = ControllerOf(a_actor);
        const bool fresh = s.staleVisits <= static_cast<std::uint32_t>(kStaleVisits);
        const bool grabbed = s.wantDrive && standpoint_interrupts::IsPlayerGrabbing(a_actor);
        VisitResult r{};

        if (!s.wantDrive || !fresh || !mc || grabbed) {
            // THE HAND-BACK. Our flag is cleared whenever it is still set; the belief that we drive her is dropped
            // unconditionally - a driver that keeps believing would skip the engine's check forever (she would play her
            // walk without moving).
            const bool ours = s.owned || s.driving;
            if (ours && mc && PlannerFlag(mc)) Steer(mc)->ReleasePlanner();
            r.owned = false;
            r.driving = false;
            r.handedBack = ours;
            r.seen = !mc ? Seen::kNoController : grabbed ? Seen::kYieldGrab : (s.wantDrive && !fresh) ? Seen::kStale
                                                                            : ours ? Seen::kHandedBack : s.seen;
            r.freeIfDone = true;
            WriteBack(i, id, r);
            if (ours) QueueEvaluate(id);
            g_nextMotion(a_actor, a_2, a_3, a_4);
            return;
        }

        if (!FeetAreFree(a_actor)) {
            // her animation owns her feet: not driven this visit; the engine's own check runs (and ends planner control);
            // `owned` stays as it was - our flag, if still set, is still ours
            r.owned = s.owned;
            r.driving = false;
            r.seen = Seen::kAnimOwns;
            WriteBack(i, id, r);
            g_nextMotion(a_actor, a_2, a_3, a_4);
            return;
        }

        if (!s.driving) {
            if (PlannerDirectNow(mc) && !s.owned) {
                r.owned = false;
                r.driving = false;
                r.seen = Seen::kOtherDriver;
                WriteBack(i, id, r);
                g_nextMotion(a_actor, a_2, a_3, a_4);
                return;
            }
            Modes(mc)->ToHighProcess();
            Steer(mc)->TakePlanner();
            Modes(mc)->MotionDrives();
            if (!PlannerDirectNow(mc)) {
                if (PlannerFlag(mc)) Steer(mc)->ReleasePlanner();   // the flag we just set is ours: never leave it behind
                r.owned = false;
                r.driving = false;
                r.seen = Seen::kDidNotTake;
                WriteBack(i, id, r);
                g_nextMotion(a_actor, a_2, a_3, a_4);
                return;
            }
            r.engaged = true;
            r.seen = Seen::kEngaged;
        } else if (!PlannerDirectNow(mc)) {
            // lost: something changed her movement type while we drove her. Nobody upstream drives her in this visit (an
            // upstream driver never passes its actor down), so a flag still set is ours.
            if (PlannerFlag(mc)) Steer(mc)->ReleasePlanner();
            r.owned = false;
            r.driving = false;
            r.lost = true;
            r.seen = Seen::kLost;
            WriteBack(i, id, r);
            QueueEvaluate(id);
            g_nextMotion(a_actor, a_2, a_3, a_4);
            return;
        } else {
            r.seen = Seen::kDriving;
        }

        // DRIVE: the three targets. The chain is NOT called for her (the engine's original would end planner control).
        Steer(mc)->SteerSpeed(s.speed);
        if (s.speed > kSteerMinSpeed) Steer(mc)->SteerDirection(RE::NiPoint3(0.f, 0.f, s.dirZ));
        Steer(mc)->SteerFacing(RE::NiPoint3(0.f, 0.f, s.faceZ));
        r.owned = true;
        r.driving = true;
        WriteBack(i, id, r);
    }

    // ═══ the arbiter hooks (VR): Standpoint's response values for the walkers the hook is driving right now ═══
    bool IsDrivenState(void* a_state)
    {
        if (!a_state || g_inUse.load(std::memory_order_acquire) == 0) return false;
        const auto v = reinterpret_cast<std::uintptr_t>(a_state);
        for (const auto& d : g_drivenState) {
            if (d.load(std::memory_order_acquire) == v) return true;
        }
        return false;
    }

    void OnArbiterSpeeds(void* a_state, void* a_params, void* a_3, void* a_4, float* a_5, float* a_6, float* a_7)
    {
        if (a_params && IsDrivenState(a_state)) {
            DrivenParams p(static_cast<MoveParamSlots*>(a_params));
            g_nextSpeeds(a_state, &p, a_3, a_4, a_5, a_6, a_7);
            return;
        }
        g_nextSpeeds(a_state, a_params, a_3, a_4, a_5, a_6, a_7);
    }

    void OnArbiterTurns(void* a_state, void* a_params, void* a_3, void* a_4, float* a_5, float* a_6)
    {
        if (a_params && IsDrivenState(a_state)) {
            DrivenParams p(static_cast<MoveParamSlots*>(a_params));
            g_nextTurns(a_state, &p, a_3, a_4, a_5, a_6);
            return;
        }
        g_nextTurns(a_state, a_params, a_3, a_4, a_5, a_6);
    }

    // 1.5.0: a_len bytes at a_addr are committed, readable memory (an address decoded from bytes is never read blindly)
    bool Readable(std::uintptr_t a_addr, std::size_t a_len)
    {
        if (!a_addr) return false;
        MEMORY_BASIC_INFORMATION mbi{};
        if (::VirtualQuery(reinterpret_cast<LPCVOID>(a_addr), &mbi, sizeof(mbi)) != sizeof(mbi)) return false;
        if (mbi.State != MEM_COMMIT || (mbi.Protect & (PAGE_NOACCESS | PAGE_GUARD)) || mbi.Protect == 0) return false;
        const auto end = reinterpret_cast<std::uintptr_t>(mbi.BaseAddress) + mbi.RegionSize;
        return a_addr + a_len <= end;
    }

    // the loaded module holding a_addr (null = none)
    HMODULE ModuleAt(std::uintptr_t a_addr)
    {
        HMODULE mod = nullptr;
        if (!::GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                                  reinterpret_cast<LPCSTR>(a_addr), &mod)) {
            return nullptr;
        }
        return mod;
    }

    // a branch-pool stub (jmp [rip+0] + an absolute address): its absolute target; 0 = not a stub
    std::uintptr_t StubTarget(std::uintptr_t a_addr)
    {
        if (!Readable(a_addr, 14)) return 0;
        const auto* b = reinterpret_cast<const std::uint8_t*>(a_addr);
        if (b[0] == 0xFF && b[1] == 0x25 && b[2] == 0 && b[3] == 0 && b[4] == 0 && b[5] == 0) {
            return *reinterpret_cast<const std::uintptr_t*>(a_addr + 6);
        }
        return 0;
    }

    // The module a call target belongs to (the hook chain order shows in the log). A branch-pool stub
    // (jmp [rip+0] + an absolute address) is followed.
    std::string ModuleOfAddress(std::uintptr_t a_addr)
    {
        for (int hop = 0; hop < 3 && a_addr; ++hop) {
            if (HMODULE mod = ModuleAt(a_addr)) {
                char path[MAX_PATH]{};
                ::GetModuleFileNameA(mod, path, MAX_PATH);
                std::string name(path);
                const auto cut = name.find_last_of("\\/");
                if (cut != std::string::npos) name = name.substr(cut + 1);
                return std::format("{}+0x{:X}", name, a_addr - reinterpret_cast<std::uintptr_t>(mod));
            }
            if (const auto next = StubTarget(a_addr)) {
                a_addr = next;
                continue;
            }
            break;
        }
        return std::format("0x{:X} (in no module)", a_addr);
    }

    // ═══ 1.5.0 THE RUN-TIME SITE CHECK (report "1.5.0 Plan" X §3a; review R2 S5) - read-only: it reads bytes, patches nothing ═══
    // The motion-check CALL is one of this runtime's PINNED offsets (kSiteVR / kSiteSE / kSiteAE, in order) - never a byte
    // found by scanning. A candidate passes when it holds E8, its target is a real function - in the game's own code (.text)
    // and 16-byte aligned (engine functions start so), or in another module / behind a branch-pool stub (another mod hooked it
    // first, as PLANCK does on VR) - and that target is not the anchor's own. The anchor ("apply movement", E8) sits at its
    // pinned offset after it. The 0x120 bytes are logged either way: a tester's log confirms (or refutes) the offset offline.
    struct SiteScan {
        bool           ready{ false };
        std::string    why;                   // NOT READY because ...
        std::uintptr_t fn{ 0 };               // Actor::UpdateMovement
        std::uint64_t  fnId{ 0 };
        std::uintptr_t apply{ 0 };            // the anchor CALL (SE +0xF0 / AE +0xFB / VR +0xF0)
        std::uintptr_t site{ 0 };             // the motion-check CALL that passed (0 = none)
        std::uintptr_t target{ 0 };           // ... its target
        bool           engineOwn{ false };    // ... is the engine's own function (not hooked by another mod)
        std::string    tried;                 // every pinned candidate and its verdict - for the log
    };

    bool InGameText(std::uintptr_t a_addr)
    {
        const auto seg = REL::Module::get().segment(REL::Segment::textx);
        return seg.size() > 0 && a_addr >= seg.address() && a_addr < seg.address() + seg.size();
    }

    std::uintptr_t CallTargetAt(std::uintptr_t a_call)
    {
        const auto rel = *reinterpret_cast<const std::int32_t*>(a_call + 1);
        return a_call + 5 + static_cast<std::intptr_t>(rel);
    }

    // a CALL target that is a real function (see above); a_own = the engine's own
    bool RealFunction(std::uintptr_t a_target, bool& a_own)
    {
        a_own = false;
        if (InGameText(a_target)) {
            a_own = (a_target & 0xF) == 0;
            return a_own;
        }
        const std::uintptr_t exe = REL::Module::get().base();
        if (const HMODULE mod = ModuleAt(a_target)) {                   // another module (in the exe but not .text: no)
            return reinterpret_cast<std::uintptr_t>(mod) != exe;
        }
        const std::uintptr_t next = StubTarget(a_target);               // a branch-pool stub of another mod's hook
        return next && ModuleAt(next) != nullptr;
    }

    SiteScan FindMotionSite()
    {
        SiteScan r{};
        r.fnId = REL::Module::IsAE() ? kMotionCheckFnIdAE : kMotionCheckFnId;
        r.fn = REL::RelocationID(kMotionCheckFnId, kMotionCheckFnIdAE).address();
        if (!r.fn || !InGameText(r.fn) || !Readable(r.fn, kSelfTestDump)) {
            r.why = std::format("Actor::UpdateMovement (id {}) is not in the game's code (0x{:X})", r.fnId, r.fn);
            return r;
        }
        r.apply = r.fn + REL::VariantOffset(kApplyOffsetSE, kApplyOffsetAE, kApplyOffsetVR).offset();
        if (*reinterpret_cast<const std::uint8_t*>(r.apply) != 0xE8) {
            r.why = std::format("the anchor (the apply-movement CALL at +0x{:X}) holds 0x{:02X}, not E8", r.apply - r.fn,
                                *reinterpret_cast<const std::uint8_t*>(r.apply));
            return r;
        }
        const std::uintptr_t applyTarget = CallTargetAt(r.apply);
        const std::uintptr_t* cand = REL::Module::IsAE() ? kSiteAE : REL::Module::IsVR() ? kSiteVR : kSiteSE;
        const std::size_t     count = REL::Module::IsAE() ? std::size(kSiteAE) : REL::Module::IsVR() ? std::size(kSiteVR) : std::size(kSiteSE);
        for (std::size_t c = 0; c < count; ++c) {
            const std::uintptr_t p = r.fn + cand[c];
            const char*          verdict = nullptr;      // null = it passes
            std::uintptr_t       t = 0;
            bool                 own = false;
            if (p + 5 > r.apply) {
                verdict = "not before the anchor";
            } else if (*reinterpret_cast<const std::uint8_t*>(p) != 0xE8) {
                verdict = "not a CALL (E8)";
            } else if (t = CallTargetAt(p); t == applyTarget) {
                verdict = "the anchor's own target";
            } else if (!RealFunction(t, own)) {
                verdict = "its target is no function start (not 16-aligned in .text, no module, no hook stub)";
            }
            r.tried += std::format("{}+0x{:X} {}", r.tried.empty() ? "" : "; ", cand[c], verdict ? verdict : "PASSES");
            if (!verdict && !r.site) {
                r.site = p;
                r.target = t;
                r.engineOwn = own;
            }
        }
        if (!r.site) {
            r.why = std::format("no pinned candidate passes ({})", r.tried);
            return r;
        }
        r.ready = true;
        return r;
    }

    // a_len bytes from a_addr as hex lines of 32 (the self-test's dump - for the offline read of SE / AE bytes)
    void LogBytes(const char* a_what, std::uintptr_t a_addr, std::size_t a_len)
    {
        if (!Readable(a_addr, a_len)) {
            logger::info("direct walk self-test: {} - not readable", a_what);
            return;
        }
        const auto* b = reinterpret_cast<const std::uint8_t*>(a_addr);
        for (std::size_t off = 0; off < a_len; off += 32) {
            std::string line;
            for (std::size_t k = off; k < a_len && k < off + 32; ++k) line += std::format("{:02X} ", b[k]);
            logger::info("direct walk self-test: {} +0x{:03X}: {}", a_what, off, line);
        }
    }

    // the address-library id at exactly a_offset (0 = none) - a NON-fatal reverse lookup (REL::Offset2ID's own operator()
    // calls report_and_fail on a miss)
    std::uint64_t IdAtOffset(std::uintptr_t a_offset)
    {
        const REL::Offset2ID table;
        const REL::Offset2ID::value_type key{ 0, a_offset };
        const auto it = std::lower_bound(table.begin(), table.end(), key,
                                         [](const auto& a_l, const auto& a_r) { return a_l.offset < a_r.offset; });
        return (it != table.end() && it->offset == a_offset) ? it->id : 0;
    }

    // 1.5.0 the SE / AE switch: Standpoint.ini [DirectWalk] SEAE (0 = the self-test only, 1 = install + walk). Read once.
    int ReadSeaeSwitch(std::string& a_note)
    {
        std::ifstream in(kIniPath);
        if (!in) {
            a_note = std::format("{} not found - SEAE = 0", kIniPath);
            return 0;
        }
        auto trim = [](std::string_view a_s) {
            while (!a_s.empty() && std::isspace(static_cast<unsigned char>(a_s.front()))) a_s.remove_prefix(1);
            while (!a_s.empty() && std::isspace(static_cast<unsigned char>(a_s.back()))) a_s.remove_suffix(1);
            return a_s;
        };
        auto same = [](std::string_view a_a, std::string_view a_b) {
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
                inSection = close != std::string_view::npos && same(trim(s.substr(1, close - 1)), "DirectWalk");
                continue;
            }
            if (!inSection) continue;
            const auto eq = s.find('=');
            if (eq == std::string_view::npos || !same(trim(s.substr(0, eq)), "SEAE")) continue;
            std::string_view v = trim(s.substr(eq + 1));
            if (const auto c = v.find_first_of(";#"); c != std::string_view::npos) v = trim(v.substr(0, c));
            if (v == "1") {
                a_note = std::format("{} [DirectWalk] SEAE = 1", kIniPath);
                return 1;
            }
            a_note = std::format("{} [DirectWalk] SEAE = {}{}", kIniPath, std::string(v), v == "0" ? "" : " (not 0 or 1: 0 is used)");
            return 0;
        }
        a_note = std::format("{} has no [DirectWalk] SEAE line - SEAE = 0", kIniPath);
        return 0;
    }

    std::string g_notLiveText = "the walk driver is not installed yet";   // set once in Install (the B 19 text)

    // ═══ VR Chain (soft: SKSE messaging only; absent = no chain check) ═══
    VrChainQueries* g_vrChain = nullptr;
    bool            g_vrChainAsked = false;

    void AskVrChainOnce()
    {
        if (g_vrChainAsked) return;
        g_vrChainAsked = true;
        if (!::GetModuleHandleA("VRChain.dll")) {
            logger::info("direct walk: VR Chain is not loaded - no chain check");
            return;
        }
        struct BindHandshake { void* (*GetApiFunction)(unsigned int) = nullptr; } msg;
        if (auto* m = SKSE::GetMessagingInterface()) m->Dispatch(kVrChainMsg, &msg, sizeof(msg), "VRChain");
        if (msg.GetApiFunction) g_vrChain = static_cast<VrChainQueries*>(msg.GetApiFunction(1));
        if (g_vrChain && g_vrChain->BuildNumber() < 10000) g_vrChain = nullptr;
        logger::info("direct walk: VR Chain's bind interface {} - a chained NPC is {}",
                     g_vrChain ? std::format("answered (build {})", g_vrChain->BuildNumber()) : std::string("did NOT answer"),
                     g_vrChain ? "never taken" : "NOT recognised (no chain check)");
    }

    bool ChainedByVrChain(RE::Actor* a_actor)
    {
        return g_vrChain && g_vrChain->ChainCount(a_actor->GetFormID()) > 0;
    }

    // 1.5.1 (D §5): who holds her when another driver has her (B 22 / a take-over) - named when Standpoint can tell: the
    // player's HIGGS grab = PLANCK (activeragdoll.dll leads a grabbed NPC), VR Chain's chain, else PPB's push walk (a hand on her)
    // when PPB is loaded. The 1.5.0 run's B 22 (20:54:47) was most likely PPB's.
    std::string HolderText(RE::Actor* a_actor)
    {
        if (standpoint_interrupts::IsPlayerGrabbing(a_actor)) return "PLANCK has her (the player's HIGGS hand grabs her)";
        if (ChainedByVrChain(a_actor)) return "VR Chain has her (she is chained)";
        if (::GetModuleHandleA("PPB.dll")) return "most likely PPB's push walk (a hand on her - PPB is loaded), or another plugin";
        return "another plugin (PPB is not loaded; no HIGGS grab, no VR Chain chain)";
    }

    // ═══ geometry (Skyrim heading: 0 = north = +Y, clockwise) ═══
    float Dist2D(const RE::NiPoint3& a_a, const RE::NiPoint3& a_b) { return std::hypot(a_a.x - a_b.x, a_a.y - a_b.y); }
    float HeadingTo(const RE::NiPoint3& a_from, const RE::NiPoint3& a_to) { return std::atan2(a_to.x - a_from.x, a_to.y - a_from.y); }
    float Wrap(float a_rad)
    {
        while (a_rad > kPi) a_rad -= 2.f * kPi;
        while (a_rad < -kPi) a_rad += 2.f * kPi;
        return a_rad;
    }
    float Deg(float a_rad) { return a_rad * kRadToDeg; }
    float Compass(float a_rad)
    {
        float d = Deg(a_rad);
        while (d < 0.f) d += 360.f;
        while (d >= 360.f) d -= 360.f;
        return d;
    }

    // ═══ the walkers (main thread only) ═══
    // (1.5.0: + kDoor - she stands at speed 0 before a closed door she has opened, until it is open or kDoorWaitS)
    // (1.6.0: + kWait - she stands at speed 0 at the end of the VERIFIED route (Params::remainAfterU > 0) until Extend brings the
    //  next chunk, or an empty Extend makes that end the goal)
    enum class Phase : std::uint8_t { kTurn, kWalk, kSettle, kFace, kStand, kPaused, kDoor, kWait };
    const char* PhaseName(Phase a_p)
    {
        switch (a_p) {
        case Phase::kTurn:   return "turn";
        case Phase::kWalk:   return "walk";
        case Phase::kSettle: return "settle";
        case Phase::kFace:   return "face";
        case Phase::kStand:  return "stand";
        case Phase::kPaused: return "paused";
        case Phase::kDoor:   return "door";
        case Phase::kWait:   return "wait-route";
        }
        return "?";
    }

    const char* OpenStateName(RE::BGSOpenCloseForm::OPEN_STATE a_s)
    {
        switch (a_s) {
        case RE::BGSOpenCloseForm::OPEN_STATE::kOpen:    return "open";
        case RE::BGSOpenCloseForm::OPEN_STATE::kOpening: return "opening";
        case RE::BGSOpenCloseForm::OPEN_STATE::kClosed:  return "closed";
        case RE::BGSOpenCloseForm::OPEN_STATE::kClosing: return "closing";
        default:                                         return "no open state";
        }
    }

    // 1.5.0 one door of her route (CollectDoors), in route order
    struct RouteDoor {
        RE::FormID   id{ 0 };
        float        arc{ 0.f };             // where her route crosses its doorway: the arc position along the track
        RE::NiPoint3 hinge{};                // the ref's position: a swinging leaf's origin (KNOWLEDGEBASE: "origin on the hinge")
        RE::NiPoint3 mid{};                  // the middle of its doorway (the closed leaf's line) - the bad spot if it stays shut
        float        swing{ 0.f };           // how far the leaf reaches from the hinge (its farthest bound corner, x scale)
        float        zone{ 0.f };            // swing + her radius + kDoorPad: she stops outside this
        bool         planned{ false };       // a planner door point (closed at plan time); false = found crossing (open then)
        bool         done{ false };          // passed (or gone): forgotten
        bool         openLogged{ false };    // "is open - she walks through" logged
    };

    struct Walker {
        Params                 prm{};
        ReportFn               report{ nullptr };
        RE::FormID             who{ 0 };
        int                    slot{ -1 };
        std::vector<PathPoint> pts;
        std::vector<float>     cum;            // 2D arc length at each point
        float                  total{ 0.f };
        Phase                  phase{ Phase::kWalk };
        float                  phaseT{ 0.f };
        int                    seg{ 0 };
        int                    segMax{ 0 };
        float                  tSeg{ 0.f };
        float                  s{ 0.f };       // arc position
        float                  xtrack{ 0.f };
        float                  sharpPassedS{ -1.f };
        float                  sBest{ 0.f };
        float                  noProgressT{ 0.f };
        float                  travelDir{ 0.f };
        bool                   haveTravel{ false };
        float                  finalFace{ 0.f };
        End                    arriveWhy{ End::kArrived };
        bool                   answered{ false };   // the arrival report is out (she stands for the hold)
        float                  holdLeft{ -1.f };
        float                  pauseYaw{ 0.f };
        float                  cmdSpeed{ 0.f }, cmdDir{ 0.f }, cmdFace{ 0.f };
        RE::NiPoint3           lastPos{};
        float                  speedMeas{ 0.f };
        float                  t{ 0.f };
        float                  traceT{ 0.f };
        int                    stopLines{ 0 };
        // the hook's receipts as this walk last saw them
        std::uint32_t          visitsSeen{ 0 };
        int                    ticksNoVisit{ 0 };
        Seen                   lastSeen{ Seen::kNone };
        std::uint32_t          lostsSeen{ 0 };
        std::uint32_t          engagesSeen{ 0 };
        float                  animT{ 0.f };
        float                  otherT{ 0.f };
        float                  noTakeT{ 0.f };
        float                  noCtlT{ 0.f };
        bool                   everDrove{ false };
        bool                   startSent{ false };
        int                    retakes{ 0 };          // losts IN A ROW (reset by a steady driven visit)
        bool                   turnLatched{ false };  // a stand-and-turn timed out: she turns while walking (review H #3)
        float                  standT{ 0.f };         // seconds in the stand (the trace slows down)
        // v1.4.1 arrival precision
        bool                   canStrafe{ false };    // IMovementState::CanStrafe at Begin: her facing can differ from her travel
        bool                   blendOn{ false };      // the facing blend of the last kFaceBlendDist has started (logged once)
        bool                   reapproached{ false }; // the ONE re-approach is used
        bool                   reapproach{ false };   // walking it now (a stall there = she stays: no BLOCKED re-plan)
        bool                   holdFinal{ false };    // the facing target is the final heading all along (the re-approach, or a
                                                      // route that lies wholly inside the blend from its start)
        bool                   blendOff{ false };     // review R #4: the blend failed this walk - OFF for the rest of it
        float                  blendBadT{ 0.f };      // seconds of blend trouble in a row
        int                    blendDir{ 0 };         // review R nit 9: the turn side the blend started with (+1 / -1)
        bool                   keepFace{ false };     // review R #2: a keep-travel walk's re-approach keeps the heading ...
        float                  keptFace{ 0.f };       // ... she had before it (never "the way she came")
        float                  velX{ 0.f }, velY{ 0.f };   // her smoothed velocity, u/s (the blend's direction test)
        // 1.5.0 doors: the doors of this route in route order (CollectDoors), and the door phase
        std::vector<RouteDoor> doors;
        int                    doorIdx{ -1 };         // the door she is opening (an index into doors)
        RE::FormID             doorRef{ 0 };          // ... the door
        int                    doorActs{ 0 };         // ... how often she has activated it in this door phase (1.6.1 B2: at most 2)
        float                  doorActT{ 0.f };       // ... the phase time of her last activation
        bool                   doorCloseLogged{ false }; // ... "it is closing: she waits until it is shut" logged
        std::vector<RE::FormID> doorFailed;           // 1.6.1 B2: openable doors that already stayed shut once in this walk (the
                                                      // first such failure marks no bad spot - the doorway may be the only way)
        // 1.5.1 the idle break (D §1): IdleForceDefaultState sent ONCE per walk (review R2 N1), only while she still has to walk
        // to the point (review R2 B1: never during the hold after the answer - her animation is the consumer's then)
        bool                   idleBroken{ false };   // sent; the episode ends when the hook drives her again
        float                  idleBreakT{ 0.f };     // seconds since the break (the walk ends only when the animation still owns
                                                      // her feet kAnimLockS after it)
        int                    idleBreaks{ 0 };       // breaks sent in this walk (one at most)
        std::uint32_t          breakVisits{ 0 };      // the hook's visit count at the break: "worked" needs a newer visit (R2 N7)
        float                  breakAfter{ 0.f };     // a moving take-over (the far hand-over): her animation may own her feet
                                                      // this long before a break is sent (a locomotion transition - review R2 R1)
        // 1.6.0 the speed bands (S2 §3): the band she is told now - 0 her walk, 1 her jog, 2 her run (1.5.1's "her pace when
        // taken over moving" maps to the nearest band)
        int                    band{ 0 };
        float                  lookAhead{ 48.f };     // 1.6.0: the pure-pursuit look-ahead this frame (0.35 s x her speed, 48-128 u)
        float                  lastYaw{ 0.f };        // 1.6.0 the yaw-rate trace: her yaw at the last frame ...
        float                  yawRate{ 0.f };        // ... and her smoothed yaw rate, deg/s
        bool                   haveYaw{ false };
        int                    extends{ 0 };          // 1.6.0: Extend calls with points (chunks appended) - log
        int                    swaps{ 0 };            // 1.6.0: Swap calls (new routes joined on the move) - log
        float                  submerged{ 0.f };      // 1.6.1 (B4): her submerged level this frame (the swim watch's) - the water cap
                                                      // holds while she is IN the water, not over a whole leg out of it
    };

    std::unordered_map<std::int32_t, Walker> g_walkers;
    std::atomic<int>                         g_liveWalkers{ 0 };
    bool                                     g_threadsLogged = false;

    void SetLive() { g_liveWalkers.store(static_cast<int>(g_walkers.size()), std::memory_order_release); }

    std::string ActorText(RE::Actor* a_actor)
    {
        if (!a_actor) return "?";
        const char* n = a_actor->GetDisplayFullName();
        return std::format("{} [{:08X}]", n && n[0] ? n : "(no name)", a_actor->GetFormID());
    }

    // the u/s -> normalised conversion of HER gait, for the log (VR only; -1 elsewhere)
    float NormalisedForLog(RE::Actor* a_actor, float a_uPerS)
    {
        if (!REL::Module::IsVR()) return -1.f;
        using Fn = float (*)(void*, float);
        static REL::Relocation<Fn> normalise{ REL::ID(kNormalizeSpeedId) };
        return normalise(a_actor->AsActorState(), a_uPerS);
    }

    void SendDirectWalkEvent(RE::Actor* a_actor, std::int32_t a_walkId, const std::string& a_what)
    {
        auto* src = SKSE::GetModCallbackEventSource();
        if (!src || !a_actor) return;
        SKSE::ModCallbackEvent ev{ RE::BSFixedString("Standpoint_DirectWalk"), RE::BSFixedString(a_what),
                                   static_cast<float>(a_walkId), a_actor };
        src->SendEvent(&ev);
    }

    // the gate: what makes driving her wrong right now (the same list at the start and every frame)
    End GateOf(RE::Actor* a_actor, std::string& a_text)
    {
        if (!a_actor->Is3DLoaded()) { a_text = "her 3D is not loaded"; return End::kGone; }
        auto* st = a_actor->AsActorState();
        const auto life = st->GetLifeState();
        if (life == RE::ACTOR_LIFE_STATE::kDead || life == RE::ACTOR_LIFE_STATE::kDying) { a_text = "she is dead / dying"; return End::kDead; }
        if (st->IsBleedingOut()) { a_text = "she is bleeding out"; return End::kBleedout; }
        if (life == RE::ACTOR_LIFE_STATE::kUnconcious) { a_text = "she is unconscious"; return End::kKnocked; }
        if (life == RE::ACTOR_LIFE_STATE::kRestrained) { a_text = "she is restrained (life state)"; return End::kRestrained; }
        if (a_actor->IsInKillMove()) { a_text = "she is in a kill move"; return End::kKillMove; }
        if (a_actor->IsInRagdollState()) { a_text = "she is ragdolled"; return End::kRagdoll; }
        if (st->GetKnockState() != RE::KNOCK_STATE_ENUM::kNormal) {
            a_text = std::format("her knock state is {}", static_cast<int>(st->GetKnockState()));
            return End::kKnocked;
        }
        if (st->GetSitSleepState() != RE::SIT_SLEEP_STATE::kNormal) {
            a_text = std::format("she sits / sleeps / uses furniture (state {})", static_cast<int>(st->GetSitSleepState()));
            return End::kFurniture;
        }
        if (a_actor->IsOnMount()) { a_text = "she is mounted"; return End::kMounted; }
        if (a_actor->IsInCombat()) { a_text = "she is in combat"; return End::kCombat; }
        return End::kNone;
    }

    void BuildTrack(Walker& w)
    {
        w.cum.assign(w.pts.size(), 0.f);
        for (std::size_t k = 1; k < w.pts.size(); ++k) w.cum[k] = w.cum[k - 1] + Dist2D(w.pts[k - 1].pos, w.pts[k].pos);
        w.total = w.cum.empty() ? 0.f : w.cum.back();
        w.seg = 0;
        w.segMax = 0;
        w.tSeg = 0.f;
        w.s = 0.f;
        w.sBest = 0.f;
        w.noProgressT = 0.f;
        w.sharpPassedS = -1.f;
        w.stopLines = 0;
    }

    // 1.6.0 (Extend): the arc lengths of points appended from index a_from on - the track she walks is kept (s, seg, sBest ...)
    void ExtendTrack(Walker& w, std::size_t a_from)
    {
        w.cum.resize(w.pts.size(), 0.f);
        for (std::size_t k = (std::max)(a_from, std::size_t{ 1 }); k < w.pts.size(); ++k) {
            w.cum[k] = w.cum[k - 1] + Dist2D(w.pts[k - 1].pos, w.pts[k].pos);
        }
        w.total = w.cum.empty() ? 0.f : w.cum.back();
    }

    void Project(const Walker& w, int a_j, const RE::NiPoint3& a_p, float& a_t, float& a_d)
    {
        const RE::NiPoint3& a = w.pts[a_j].pos;
        const RE::NiPoint3& b = w.pts[a_j + 1].pos;
        const float vx = b.x - a.x, vy = b.y - a.y;
        const float len2 = vx * vx + vy * vy;
        a_t = len2 > 0.0001f ? ((a_p.x - a.x) * vx + (a_p.y - a.y) * vy) / len2 : 1.f;
        a_t = (std::clamp)(a_t, 0.f, 1.f);
        a_d = std::hypot(a.x + vx * a_t - a_p.x, a.y + vy * a_t - a_p.y);
    }

    // where she is along the route: the nearest segment among the current one (one back at most) and the next three
    void Follow(Walker& w, const RE::NiPoint3& a_p)
    {
        const int n = static_cast<int>(w.pts.size());
        if (n < 2) {
            w.s = 0.f;
            w.tSeg = 1.f;
            w.xtrack = 0.f;
            return;
        }
        const int lo = (std::max)(0, w.segMax - 1);
        const int hi = (std::min)(n - 2, w.seg + 3);
        int   best = (std::clamp)(w.seg, lo, hi);
        float bestT = 0.f, bestD = 3.0e38f;
        for (int j = lo; j <= hi; ++j) {
            float t = 0.f, d = 0.f;
            Project(w, j, a_p, t, d);
            if (d <= bestD + 0.5f) {          // ties go to the later segment (she stands on a corner point)
                if (d < bestD || j > best) {
                    bestD = (std::min)(bestD, d);
                    best = j;
                    bestT = t;
                }
            }
        }
        w.seg = best;
        w.segMax = (std::max)(w.segMax, best);
        w.tSeg = bestT;
        w.xtrack = bestD;
        w.s = w.cum[best] + bestT * (w.cum[best + 1] - w.cum[best]);
    }

    RE::NiPoint3 PointAt(const Walker& w, float a_s)
    {
        const int n = static_cast<int>(w.pts.size());
        if (n == 1 || a_s >= w.total) return w.pts.back().pos;
        if (a_s <= 0.f) return w.pts.front().pos;
        int j = 0;
        while (j < n - 2 && w.cum[j + 1] < a_s) ++j;
        const float len = w.cum[j + 1] - w.cum[j];
        const float t = len > 0.0001f ? (a_s - w.cum[j]) / len : 1.f;
        const RE::NiPoint3& a = w.pts[j].pos;
        const RE::NiPoint3& b = w.pts[j + 1].pos;
        return RE::NiPoint3(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t);
    }

    // the next SHARP corner she has not passed (-1 = none before the goal)
    int NextSharp(const Walker& w)
    {
        const int n = static_cast<int>(w.pts.size());
        for (int c = 1; c < n - 1; ++c) {
            if ((w.pts[c].flags & kFlagSharp) && w.cum[c] > w.sharpPassedS) return c;
        }
        return -1;
    }

    // the aim point's arc position: the look-ahead ahead (1.6.0: w.lookAhead - 48 u at her walk, up to 128 u at her run), never
    // past the end of the route, never past a SHARP corner not yet reached
    float AimS(const Walker& w)
    {
        float l = (std::min)(w.total, w.s + w.lookAhead);
        const int c = NextSharp(w);
        if (c >= 0) l = (std::min)(l, w.cum[c]);
        return l;
    }

    // v1.4.1: the heading she must end with, seen from a_pos - toward Params::faceWho (read live; 1.6.0 D1: + faceOffsetRad),
    // else the asked facing. false = none asked (she keeps her last travel direction: nothing to blend toward).
    bool FinalHeading(const Walker& w, const RE::NiPoint3& a_pos, float& a_out)
    {
        if (w.prm.faceWho) {
            if (auto* t = RE::TESForm::LookupByID<RE::TESObjectREFR>(w.prm.faceWho); t && t->Is3DLoaded()) {
                const RE::NiPoint3 tp = t->GetPosition();
                if (Dist2D(a_pos, tp) > 1.f) {
                    a_out = Wrap(HeadingTo(a_pos, tp) + w.prm.faceOffsetRad);
                    return true;
                }
            }
        }
        if (w.prm.hasFacing) {
            a_out = w.prm.facingRad;
            return true;
        }
        return false;
    }

    // a heading moved from a_from toward a_to by at most a_maxStep (radians, the shorter way round)
    float TurnToward(float a_from, float a_to, float a_maxStep)
    {
        const float d = Wrap(a_to - a_from);
        if (std::fabs(d) <= a_maxStep) return Wrap(a_to);
        return Wrap(a_from + (d > 0.f ? a_maxStep : -a_maxStep));
    }

    // v1.4.1: the facing blend runs now - she can strafe, a final heading is asked, and she walks the re-approach (or a route
    // wholly inside the blend) or the last kFaceBlendDist of a route with no SHARP corner left before its end
    bool BlendNow(const Walker& w, bool a_haveFinal)
    {
        if (!w.canStrafe || !a_haveFinal || w.blendOff) return false;
        if (w.holdFinal || w.blendOn) return true;     // (review R #4) latched on for the rest of the track
        // (1.6.0: never at the end of a VERIFIED chunk - more route follows it)
        return w.prm.remainAfterU <= 0.f && NextSharp(w) < 0 && (w.total - w.s) <= kFaceBlendDist;
    }

    // 1.6.0 D2 (Params::faceAll): she holds the final facing over the WHOLE route when she can strafe and a final heading is asked
    // (the orbit's side-step round the target); a blend that failed this walk stays off
    bool FaceAllNow(const Walker& w, bool a_haveFinal)
    {
        return w.prm.faceAll && w.canStrafe && a_haveFinal && !w.blendOff;
    }

    // distance from a_p to the route's LAST leg (review R nit 8: the re-approach stays on the way the planner tested)
    float OffLastLeg(const Walker& w, const RE::NiPoint3& a_p)
    {
        const int n = static_cast<int>(w.pts.size());
        if (n < 2) return Dist2D(a_p, w.pts.back().pos);
        float t = 0.f, d = 0.f;
        Project(w, n - 2, a_p, t, d);
        return d;
    }

    // The order for the hook. g_slotMtx is taken here; nothing else is done under it.
    void WriteOrder(const Walker& w, bool a_want)
    {
        std::scoped_lock lk(g_slotMtx);
        if (w.slot < 0) return;
        Slot& s = g_slot[w.slot];
        if (s.who != w.who) return;
        s.wantDrive = a_want;
        s.speed = w.cmdSpeed;
        s.dirZ = w.cmdDir;
        s.faceZ = w.cmdFace;
        ++s.orderSeq;
        // A slot the hook has never visited is freed now. Any other one is freed by the hook itself at its next visit (it
        // may be in the middle of a visit right now, about to write back that OUR flag is set - that fact must not be lost).
        if (!a_want && s.visits == 0) FreeSlotLocked(w.slot);
    }

    Slot ReadSlot(const Walker& w)
    {
        std::scoped_lock lk(g_slotMtx);
        if (w.slot < 0 || g_slot[w.slot].who != w.who) return Slot{};
        return g_slot[w.slot];
    }

    // The drive is over: withdraw the order (the hook hands her back at its next visit), forget the walker, report.
    // a_report = false for Stop() (the caller asked; it needs no report).
    void Finish(std::int32_t a_id, End a_why, const std::string& a_text, bool a_report)
    {
        auto it = g_walkers.find(a_id);
        if (it == g_walkers.end()) return;
        Walker& w = it->second;
        const Slot sl = ReadSlot(w);          // the receipts BEFORE the order is withdrawn (a never-taken slot is freed then)
        w.cmdSpeed = 0.f;
        WriteOrder(w, false);
        if (a_why == End::kTaken || a_why == End::kOtherDriver) {
            // (review H #9) another system has her: the planner flag on her controller is ITS flag now. Our beliefs go, so
            // neither the hand-back nor a later walk of hers treats that flag as ours.
            std::scoped_lock lk(g_slotMtx);
            if (w.slot >= 0 && g_slot[w.slot].who == w.who) {
                g_slot[w.slot].owned = false;
                g_slot[w.slot].driving = false;
                g_drivenState[w.slot].store(0, std::memory_order_release);
            }
        }
        auto* a = RE::TESForm::LookupByID<RE::Actor>(w.who);
        // a caller's Stop names its own reason word ("released", "replaced", ...): it goes into the mod event as it is
        const std::string word = a_why == End::kNone ? a_text : std::string(EndName(a_why));
        logger::info("WALK #{} END {}{}: {} - after {:.1f} s, phase {}, at s {:.0f} of {:.0f} u (engaged {} time(s), lost {}, "
                     "hand-backs {}, hook visits {})",
                     a_id, a_why == End::kNone ? "stopped" : EndName(a_why), w.answered ? " (the hold)" : "", a_text, w.t,
                     PhaseName(w.phase), w.s, w.total, sl.engages, sl.losts, sl.handBacks, sl.visits);
        if (w.startSent && a) SendDirectWalkEvent(a, a_id, "stop|" + word);
        Report rep{};
        rep.why = a_why;
        rep.holdEnded = w.answered;
        rep.everDrove = w.everDrove;
        if (a) {
            rep.pos = a->GetPosition();
            if (auto* cell = a->GetParentCell(); cell && a->Is3DLoaded()) rep.submerged = a->GetSubmergedLevel(rep.pos.z, cell);
        }
        const ReportFn fn = w.report;
        g_walkers.erase(it);
        SetLive();
        if (a_report && fn) fn(a_id, rep);    // LAST: the callback may start or stop walks
    }

    void EnterPhase(Walker& w, Phase a_p, const char* a_why)
    {
        if (w.phase == a_p) return;
        logger::info("WALK #{} phase {} -> {} ({}) at s {:.0f} of {:.0f} u, t {:.1f} s", w.prm.walkId, PhaseName(w.phase),
                     PhaseName(a_p), a_why, w.s, w.total, w.t);
        w.phase = a_p;
        w.phaseT = 0.f;
        if (a_p == Phase::kWalk || a_p == Phase::kTurn) w.noProgressT = 0.f;
    }

    // the arrival: final facing is done. Report it; then let go (hold 0) or stand (hold -1 / N).
    void Arrive(std::int32_t a_id, RE::Actor* a_actor, float a_faceErrDeg)
    {
        auto it = g_walkers.find(a_id);
        if (it == g_walkers.end()) return;
        Walker& w = it->second;
        Report rep{};
        rep.why = w.arriveWhy;
        rep.everDrove = w.everDrove;
        rep.pos = a_actor->GetPosition();
        if (auto* cell = a_actor->GetParentCell()) rep.submerged = a_actor->GetSubmergedLevel(rep.pos.z, cell);
        rep.facingErrDeg = a_faceErrDeg;
        const ReportFn fn = w.report;
        if (w.prm.holdSeconds == 0.f) {
            w.cmdSpeed = 0.f;
            WriteOrder(w, false);
            logger::info("WALK #{} END {}: hold 0 - let go at the point (facing off by {:.1f} deg)", a_id, EndName(w.arriveWhy), a_faceErrDeg);
            if (w.startSent) SendDirectWalkEvent(a_actor, a_id, std::string("stop|") + EndName(w.arriveWhy));
            g_walkers.erase(it);
            SetLive();
        } else {
            w.answered = true;
            w.holdLeft = w.prm.holdSeconds;
            EnterPhase(w, Phase::kStand, w.prm.holdSeconds < 0.f ? "hold: she stands, driven at speed 0, until released"
                                                                : "hold: she stands, driven at speed 0, for the hold time");
            rep.holding = true;
        }
        if (fn) fn(a_id, rep);
    }

    // her body's radius (the character controller's capsule; 20 u when it cannot be read)
    float BodyRadius(RE::Actor* a_actor)
    {
        float radius = 20.f;
        if (auto* cc = a_actor->GetCharController()) {
            const float r = cc->collisionBound.extents.x;
            if (r > 4.f && r < 96.f) radius = r;
        }
        return radius;
    }

    // She is HELD where she stands (speed 0, her facing kept) and the walk is reported for a re-plan: a stall, a door that did
    // not open (kBlocked), a push off the line (kStrayed, 1.5.0). The report is the LAST thing done: the caller returns at once
    // (WalkApi may Restart her, or Stop her - the walker may be gone after it).
    // (1.6.1 B2: a_markBad false = the stall spot is reported but WalkApi must NOT make it a bad spot - an openable door's first failure)
    void PauseAndReport(std::int32_t a_id, Walker& w, End a_why, const RE::NiPoint3& a_pos, float a_yaw, float a_lvl,
                        const RE::NiPoint3& a_spot, float a_spotRadius, RE::FormID a_door, const std::string& a_text, bool a_markBad = true)
    {
        w.pauseYaw = a_yaw;
        EnterPhase(w, Phase::kPaused, a_text.c_str());
        w.cmdSpeed = 0.f;
        w.cmdFace = a_yaw;
        WriteOrder(w, true);
        Report rep{};
        rep.why = a_why;
        rep.paused = true;
        rep.markBad = a_markBad;
        rep.everDrove = w.everDrove;
        rep.pos = a_pos;
        rep.submerged = a_lvl;
        rep.stallSpot = a_spot;
        rep.stallRadius = a_spotRadius;
        rep.door = a_door;
        const ReportFn fn = w.report;
        if (fn) fn(a_id, rep);       // LAST (WalkApi re-plans, or stops her)
    }

    // 1.5.0 a door's doorway = its CLOSED leaf's line in 2D: the base bounds' longer horizontal axis through the box's middle,
    // turned by the ref's yaw (Skyrim's frame: local +X -> (cos z, -sin z), local +Y -> (sin z, cos z); heading 0 = north,
    // clockwise) and scaled - the ref's own rotation, never the animated leaf's. Also the leaf's reach from the hinge (the ref's
    // position: its farthest bound corner) and whether it is a STANDING leaf (never a floor hatch). false = no usable bounds.
    bool DoorwayOf(RE::TESObjectREFR* a_door, RE::NiPoint3& a_a, RE::NiPoint3& a_b, float& a_reach, bool& a_standing)
    {
        const RE::NiPoint3 mn = a_door->GetBoundMin();
        const RE::NiPoint3 mx = a_door->GetBoundMax();
        const float        s = a_door->GetScale() > 0.f ? a_door->GetScale() : 1.f;
        const float        wx = mx.x - mn.x, wy = mx.y - mn.y, hz = mx.z - mn.z;
        if (wx <= 1.f && wy <= 1.f) return false;
        float ax = 0.f, ay = 0.f, bx = 0.f, by = 0.f;     // the doorway's two ends, local
        if (wx >= wy) {
            ax = mn.x;
            bx = mx.x;
            ay = by = 0.5f * (mn.y + mx.y);
        } else {
            ay = mn.y;
            by = mx.y;
            ax = bx = 0.5f * (mn.x + mx.x);
        }
        const float        z = a_door->GetAngleZ();
        const float        c = std::cos(z), sn = std::sin(z);
        const RE::NiPoint3 p = a_door->GetPosition();
        a_a = RE::NiPoint3(p.x + s * (ax * c + ay * sn), p.y + s * (-ax * sn + ay * c), p.z);
        a_b = RE::NiPoint3(p.x + s * (bx * c + by * sn), p.y + s * (-bx * sn + by * c), p.z);
        a_reach = std::hypot((std::max)(std::fabs(mn.x), std::fabs(mx.x)), (std::max)(std::fabs(mn.y), std::fabs(mx.y))) * s;
        a_standing = hz * s >= kDoorLeafMinH && hz >= kDoorLeafTall * (std::max)(wx, wy);
        return true;
    }

    // where the leg p0 -> p1 crosses the doorway a -> b (2D; the doorway lengthened by kDoorCrossPad at each end): true + a_t
    // (0..1 along the leg)
    bool LegCrosses(const RE::NiPoint3& a_p0, const RE::NiPoint3& a_p1, RE::NiPoint3 a_a, RE::NiPoint3 a_b, float& a_t)
    {
        const float ex = a_b.x - a_a.x, ey = a_b.y - a_a.y;
        const float el = std::hypot(ex, ey);
        if (el > 0.001f) {
            a_a.x -= ex / el * kDoorCrossPad;
            a_a.y -= ey / el * kDoorCrossPad;
            a_b.x += ex / el * kDoorCrossPad;
            a_b.y += ey / el * kDoorCrossPad;
        }
        const float rx = a_p1.x - a_p0.x, ry = a_p1.y - a_p0.y;
        const float sx = a_b.x - a_a.x, sy = a_b.y - a_a.y;
        const float den = rx * sy - ry * sx;
        if (std::fabs(den) < 0.0001f) return false;                 // parallel: it never crosses the doorway
        const float qx = a_a.x - a_p0.x, qy = a_a.y - a_p0.y;
        const float t = (qx * sy - qy * sx) / den;                  // along the leg
        const float u = (qx * ry - qy * rx) / den;                  // along the doorway
        if (t < 0.f || t > 1.f || u < 0.f || u > 1.f) return false;
        a_t = t;
        return true;
    }

    // 1.5.0 (verify V: S2 partly + N3) THE DOORS OF HER ROUTE, once per route (Begin, Restart): every Door reference of her
    // cell(s) whose doorway one of the route's legs crosses - the planner's door points (closed at plan time) and doors that were
    // OPEN when the route was planned (no door point: they may shut before she gets there) - in route order. A door found only
    // by the crossing must be a standing leaf on her floor. The cells are locked during the look: pointers are collected there,
    // judged after (the planner's placed-water scan pattern, VR-proven since 1.4.x).
    // (1.6.0: a_firstLeg > 0 = Extend - only the legs from point a_firstLeg on are looked at and their doors APPENDED; the doors
    //  found before keep their state - passed / opened)
    void CollectDoors(std::int32_t a_id, Walker& w, RE::Actor* a_actor, int a_firstLeg = 0)
    {
        if (a_firstLeg <= 0) w.doors.clear();
        const int n = static_cast<int>(w.pts.size());
        auto*     pcell = a_actor ? a_actor->GetParentCell() : nullptr;
        const int first = (std::max)(0, a_firstLeg);
        if (n < 2 || first >= n - 1 || !pcell) return;
        float bx0 = w.pts[first].pos.x, bx1 = bx0, by0 = w.pts[first].pos.y, by1 = by0;
        for (int k = first; k < n; ++k) {
            const auto& p = w.pts[k];
            bx0 = (std::min)(bx0, p.pos.x);
            bx1 = (std::max)(bx1, p.pos.x);
            by0 = (std::min)(by0, p.pos.y);
            by1 = (std::max)(by1, p.pos.y);
        }
        bx0 -= kDoorScanPad;
        bx1 += kDoorScanPad;
        by0 -= kDoorScanPad;
        by1 += kDoorScanPad;
        std::vector<RE::TESObjectREFR*> found;
        int                             looked = 0;
        bool                            capped = false;
        const bool                      interior = pcell->IsInteriorCell();
        const auto visit = [&](RE::TESObjectREFR* a_ref) {
            if (!interior && ++looked > kDoorScanMaxRefs) {
                capped = true;
                return RE::BSContainer::ForEachResult::kStop;
            }
            auto* base = a_ref ? a_ref->GetBaseObject() : nullptr;
            if (base && base->Is(RE::FormType::Door)) {
                const RE::NiPoint3 p = a_ref->GetPosition();
                if (p.x >= bx0 && p.x <= bx1 && p.y >= by0 && p.y <= by1) found.push_back(a_ref);
            }
            return RE::BSContainer::ForEachResult::kContinue;
        };
        if (interior) {
            pcell->ForEachReference(visit);
        } else if (auto* tes = RE::TES::GetSingleton()) {
            std::vector<RE::TESObjectCELL*> seen;
            const float xs[3] = { bx0, (bx0 + bx1) * 0.5f, bx1 }, ys[3] = { by0, (by0 + by1) * 0.5f, by1 };
            for (const float px : xs) {
                for (const float py : ys) {
                    if (capped) break;
                    auto* c = tes->GetCell(RE::NiPoint3{ px, py, w.pts[first].pos.z });
                    if (!c || c->IsInteriorCell() || !c->IsAttached() || std::find(seen.begin(), seen.end(), c) != seen.end()) continue;
                    seen.push_back(c);
                    c->ForEachReference(visit);
                }
            }
        }
        // the planner's door points are doors of this route whatever the look found (a ref outside the box, a capped look)
        for (int k = first + 1; k < n; ++k) {
            if (!w.pts[k].door) continue;
            auto* d = RE::TESForm::LookupByID<RE::TESObjectREFR>(w.pts[k].door);
            if (d && std::find(found.begin(), found.end(), d) == found.end()) found.push_back(d);
        }
        const float       radius = BodyRadius(a_actor);
        std::string       list;
        const std::size_t had = w.doors.size();
        for (auto* d : found) {
            const RE::FormID id = d->GetFormID();
            if (std::any_of(w.doors.begin(), w.doors.begin() + static_cast<std::ptrdiff_t>(had),
                            [id](const RouteDoor& a_d) { return a_d.id == id && !a_d.done; })) {
                continue;                                       // (Extend) already on her route and not passed yet
            }
            RE::NiPoint3     da{}, db{};
            float            reach = 0.f;
            bool             standing = false;
            if (!DoorwayOf(d, da, db, reach, standing)) continue;
            int planned = -1;                                   // the planner's door point naming it
            for (int k = first + 1; k < n && planned < 0; ++k) {
                if (w.pts[k].door == id) planned = k;
            }
            float arc = -1.f;
            for (int k = first + 1; k < n && arc < 0.f; ++k) {
                float t = 0.f;
                if (!LegCrosses(w.pts[k - 1].pos, w.pts[k].pos, da, db, t)) continue;
                const float legZ = w.pts[k - 1].pos.z + t * (w.pts[k].pos.z - w.pts[k - 1].pos.z);
                if (planned < 0 && (!standing || std::fabs(d->GetPosition().z - legZ) > kDoorFloorDz)) continue;
                arc = w.cum[k - 1] + t * (w.cum[k] - w.cum[k - 1]);
            }
            if (arc < 0.f && planned > 0) {
                // the planner's door, its doorway not crossed by the line (odd bounds): where its ref lies along that leg
                float t = 0.f, off = 0.f;
                Project(w, planned - 1, d->GetPosition(), t, off);
                arc = w.cum[planned - 1] + t * (w.cum[planned] - w.cum[planned - 1]);
            }
            if (arc < 0.f) continue;                            // not on her way
            RouteDoor rd{};
            rd.id = id;
            rd.arc = arc;
            rd.hinge = d->GetPosition();
            rd.mid = RE::NiPoint3(0.5f * (da.x + db.x), 0.5f * (da.y + db.y), d->GetPosition().z);
            rd.swing = (std::clamp)(reach, kDoorSwingMin, kDoorSwingMax);
            rd.zone = rd.swing + radius + kDoorPad;
            rd.planned = planned > 0;
            w.doors.push_back(rd);
        }
        for (std::size_t i = had; i < w.doors.size(); ++i) {
            const auto& rd = w.doors[i];
            list += std::format(" [{:08X}] at {:.0f} u ({}, its leaf reaches {:.0f} u)", rd.id, rd.arc,
                                rd.planned ? "closed when planned" : "open when planned", rd.swing);
        }
        const bool added = w.doors.size() > had;
        std::sort(w.doors.begin(), w.doors.end(), [](const RouteDoor& a_l, const RouteDoor& a_r) { return a_l.arc < a_r.arc; });
        if (added || capped) {
            logger::info("WALK #{} doors on {} ({} door reference(s) looked at near it{}):{}", a_id,
                         first > 0 ? std::format("the new legs from point {}", first) : std::string("her route"), found.size(),
                         capped ? std::format(", the look stopped at {} references", kDoorScanMaxRefs) : std::string(),
                         list.empty() ? std::string(" none") : list);
        }
    }

    // 1.5.0 DOORS, each walking frame: EVERY door of her route she has not passed (verify V N3: two doors close together are
    // each handled, in route order). At the edge of a door's sweep: OPEN -> she walks on (it stays watched until she is past
    // it - review R2 S2); not open -> she stops HERE, outside the sweep (the door phase, true): a door that was open when the
    // route was planned and has shut is opened again by the same rule, never made a bad spot unless it fails that rule or stays
    // shut (verify V: S2 partly). A doorway passed by more than kDoorBehind along the route is forgotten.
    bool DoorCheck(std::int32_t a_id, Walker& w, const RE::NiPoint3& a_pos)
    {
        for (int i = 0; i < static_cast<int>(w.doors.size()); ++i) {
            RouteDoor& rd = w.doors[i];
            if (rd.done) continue;
            if (w.s > rd.arc + kDoorBehind) {
                rd.done = true;
                logger::info("WALK #{} DOOR [{:08X}]: she is past its doorway - forgotten", a_id, rd.id);
                continue;
            }
            auto* d = RE::TESForm::LookupByID<RE::TESObjectREFR>(rd.id);
            if (!d) {
                rd.done = true;
                logger::info("WALK #{} DOOR [{:08X}]: gone - forgotten", a_id, rd.id);
                continue;
            }
            if (!d->Is3DLoaded()) continue;                     // not loaded yet: it stays on her route
            const float dist = Dist2D(a_pos, rd.hinge);
            if (dist > rd.zone) continue;                       // not at its sweep yet (the next one may be: N3)
            const auto state = RE::BGSOpenCloseForm::GetOpenState(d);
            if (state == RE::BGSOpenCloseForm::OPEN_STATE::kOpen) {
                if (!rd.openLogged) {
                    rd.openLogged = true;
                    logger::info("WALK #{} DOOR [{:08X}] is open - she walks through (watched until she is past it)", a_id, rd.id);
                }
                continue;
            }
            rd.openLogged = false;
            w.doorIdx = i;
            w.doorRef = rd.id;
            w.doorActs = 0;
            w.doorActT = 0.f;
            w.doorCloseLogged = false;
            EnterPhase(w, Phase::kDoor, std::format("DOOR [{:08X}] ahead is {} ({}; {:.0f} u from its hinge, its leaf reaches {:.0f} u){}",
                                                    rd.id, OpenStateName(state), rd.planned ? "closed when planned" : "open when planned",
                                                    dist, rd.swing,
                                                    dist < rd.zone - kDoorInside ?
                                                        " - she is INSIDE its sweep (she started there, or it closed as she came): opened from here" :
                                                        " - she stops outside its sweep and opens it").c_str());
            return true;
        }
        return false;
    }

    // 1.5.1 THE IDLE BREAK (D §1): IdleForceDefaultState to her graph - ONCE per walk, while she still has to walk to the point
    // (a random idle while she stood: the 1.5.0 run's two animation-lock ends). The callers have passed the gate (furniture /
    // sitting / sleeping, ragdoll, knock, kill move, bleedout are refused before) and never send it during a stagger or after
    // the answer. Then the hook keeps trying to take her; the walk ends only if the animation still owns her feet kAnimLockS
    // after the break.
    void SendIdleBreak(std::int32_t a_id, Walker& w, RE::Actor* a_actor, bool a_atStart)
    {
        bool anim = false, rot = false;
        a_actor->GetGraphVariableBool(AnimDrivenVar(), anim);
        a_actor->GetGraphVariableBool(AllowRotationVar(), rot);
        const bool took = a_actor->NotifyAnimationGraph(IdleBreakEvent());
        w.idleBroken = true;
        w.idleBreakT = 0.f;
        ++w.idleBreaks;
        w.breakVisits = ReadSlot(w).visits;
        logger::info("WALK #{} IDLE BREAK: her animation owned her feet ({}graph variables bAnimationDriven {}, bAllowRotation {}) - "
                     "IdleForceDefaultState sent ({}); the walk ends only if it still owns them {:.1f} s from now", a_id,
                     a_atStart ? "at the walk's start; " : "", anim ? 1 : 0, rot ? 1 : 0,
                     took ? "her graph took the event" : "her graph did not take the event", kAnimLockS);
    }

    // ═══ 1.6.0 THE SPEED BANDS (spec D §1 item 7, S2 §3): 0 walk (told 1.0), 1 jog (1.5), 2 run (2.0) ═══
    float BandNorm(int a_band) { return a_band >= 2 ? kRunNorm : a_band == 1 ? kJogNorm : kWalkNorm; }
    float BandUps(int a_band) { return a_band >= 2 ? kRunUps : a_band == 1 ? kJogUps : kWalkUps; }
    const char* BandName(int a_band) { return a_band >= 2 ? "run" : a_band == 1 ? "jog" : "walk"; }

    // the route length a downshift from band a_from to band a_to takes at the ramp (S2 §3.2: (v_from + v_to) / 2 x dnorm / 1.5) -
    // her measured speed stands for the band she is in when it is the faster (a slower gait: the estimate anticipates more)
    float RampLead(const Walker& w, int a_from, int a_to)
    {
        if (a_to >= a_from) return 0.f;
        const float vFrom = a_from == w.band ? (std::max)(w.speedMeas, BandUps(a_from)) : BandUps(a_from);
        return 0.5f * (vFrom + BandUps(a_to)) * (BandNorm(a_from) - BandNorm(a_to)) / kSpeedRampPerS;
    }

    // the band a route LEFT of a_left allows (the goal's rule, the user's: run beyond 15 m, jog to 3 m, walk the last 3 m) with the
    // downshifts ANTICIPATED - she IS at her jog at 1,050 u and at her walk at 210 u
    int BandByLeft(const Walker& w, float a_left)
    {
        if (a_left > kRunAboveU + RampLead(w, 2, 1)) return 2;
        if (a_left > kJogAboveU + RampLead(w, 1, 0)) return 1;
        return 0;
    }

    // (the user's rule, the question tool 2026-11-06) band a_band would last at least kMinBandS at its own speed before its own
    // downshift point of the goal's rule (a_left = the route left to the goal)
    bool LastsASecond(const Walker& w, int a_band, float a_left)
    {
        const float exitAt = a_band >= 2 ? kRunAboveU + RampLead(w, 2, 1) : a_band == 1 ? kJogAboveU + RampLead(w, 1, 0) : 0.f;
        return a_left - exitAt >= BandUps(a_band) * kMinBandS;
    }

    // THE TURN CAPS AND SLOW ZONES (S2 §3.3 / §3.4): the fastest band the route AHEAD allows. Each feature caps the band from
    // kSlowLeadU + the ramp lead from band a_from to its cap (+ a_extra: an upshift looks one second further) ahead of it:
    //   a turn at a route point: <= 30 deg run, <= 60 deg jog, sharper walk · a SHARP corner walk · a kPtNoCut corner turning more
    //   than 15 deg jog (and the look-ahead held at 48 u: a_noCut) · a door of her route walk · the water's edge ahead jog, IN the
    //   water walk (1.6.1 B4: by her submerged level - a leg out of the water is free once she is out) · a depth-stop walk near
    //   water walk · a leg steeper than 27 deg (up or down) jog · a stair leg
    //   walk · a drop edge (kPtEdge) jog · the end of the VERIFIED route (more route after it): walk within her stopping distance
    //   only (review R2 #1 - not the goal's band rule).
    // Returns the cap (2 = none); a_why / a_at = the feature that sets it (route length ahead of her).
    int BandCap(const Walker& w, int a_from, float a_extra, const char*& a_why, float& a_at, bool& a_noCut)
    {
        int cap = 2;
        a_why = nullptr;
        a_at = 0.f;
        a_noCut = false;
        const auto within = [&](int a_cap, float a_ahead) { return a_ahead <= kSlowLeadU + RampLead(w, a_from, a_cap) + a_extra; };
        const auto take = [&](int a_cap, float a_ahead, const char* a_what) {
            if (a_cap >= cap || !within(a_cap, a_ahead)) return;
            cap = a_cap;
            a_why = a_what;
            a_at = (std::max)(0.f, a_ahead);
        };
        const int   n = static_cast<int>(w.pts.size());
        const float horizon = kSlowLeadU + RampLead(w, a_from, 0) + a_extra;    // nothing farther can cap anything
        for (int k = w.seg + 1; k < n; ++k) {
            const float legStart = w.cum[k - 1] - w.s;     // the leg into point k starts this far ahead (<= 0: the leg she is on)
            if (legStart > horizon) break;
            const float      ahead = w.cum[k] - w.s;       // point k is this far ahead
            const PathPoint& p0 = w.pts[k - 1];
            const PathPoint& p1 = w.pts[k];
            const float      dxy = Dist2D(p0.pos, p1.pos);
            if (dxy >= kSteepMinLegU && std::fabs(p1.pos.z - p0.pos.z) > kSteepRatio * dxy) take(1, legStart, "a leg steeper than 27 deg");
            if (p1.flags & kFlagStair) take(0, legStart, "a stair");
            // the water (1.6.1 B4, run 2026-11-06 #7595835: the cap held over ~840 u of a DRY leg out of the pond): WALK while she IS
            // in the water (her submerged level, on a leg that touches water); JOG from the lead before the water's EDGE on a leg into
            // water (the edge taken depth / kBedSlope before the water point - a gentle bed - at least kWaterEdgeMinU, at most the
            // leg); a leg OUT of the water caps nothing once she is out of it (plain walks still wade to their cap, 0.45 H)
            const bool inWater = w.submerged >= kInWaterLvl;
            const bool wetEnd = p1.kind == kKindWater;
            if (legStart <= 0.f && inWater && (wetEnd || p0.kind == kKindWater)) take(0, 0.f, "she wades");
            float edgeAhead = 0.f;
            if (wetEnd) {
                const float edgeBefore = (std::clamp)(p1.depth / kBedSlope, kWaterEdgeMinU, (std::max)(kWaterEdgeMinU, dxy));
                edgeAhead = ahead - edgeBefore;
                if (!inWater) take(1, edgeAhead, "the water's edge ahead");
            }
            if (w.prm.stopSubmerged > 0.f) {
                if (wetEnd) take(0, edgeAhead, "water (a depth stop is asked)");
                if (legStart <= 0.f && inWater) take(0, 0.f, "water (a depth stop is asked)");
            }
            if (p1.flags & kFlagEdge) take(1, ahead, "a drop edge");
            if (k >= n - 1) continue;                       // the route's last point: no turn there
            if (p1.flags & kFlagSharp) take(0, ahead, "a SHARP corner");
            const PathPoint& p2 = w.pts[k + 1];
            if (dxy < 1.f || Dist2D(p1.pos, p2.pos) < 1.f) continue;
            const float turn = std::fabs(Deg(Wrap(HeadingTo(p1.pos, p2.pos) - HeadingTo(p0.pos, p1.pos))));
            if (turn > kSlowTurnDeg) {
                take(0, ahead, "a turn over 60 deg");
            } else if (turn > kRunTurnDeg) {
                take(1, ahead, "a turn over 30 deg");
            }
            if ((p1.flags & kFlagNoCut) && turn > kNoCutTurnDeg) {
                take(1, ahead, "a corner not to be cut at speed (kPtNoCut)");
                if (within(1, ahead)) a_noCut = true;
            }
        }
        for (const auto& rd : w.doors) {
            if (!rd.done) take(0, rd.arc - w.s - rd.zone, "a door");   // (a doorway just passed stays "ahead" until it is forgotten)
        }
        if (w.prm.remainAfterU > 0.f) {
            // (review R2 #1) the end of a CHECKED chunk is no goal: it caps her only by her STOPPING distance - walk within kSlowLeadU
            // + the ramp lead from her band to her walk (~280 u from a run), no cap before that - so she can stop there if the next
            // chunk is late, and never jogs ~1,150 u early before every chunk end (run / jog / run surges). The goal's band rule
            // (BandByLeft) is for the real goal only (toGo includes the unverified rest)
            take(0, w.total - w.s, "the end of the checked route (the next chunk is not in yet) - her stopping distance");
        }
        return cap;
    }

    // a normalised speed moved from a_from toward a_to by at most a_step (the ramp: never a jump)
    float RampToward(float a_from, float a_to, float a_step)
    {
        if (std::fabs(a_to - a_from) <= a_step) return a_to;
        return a_from + (a_to > a_from ? a_step : -a_step);
    }

    // One walker, one frame. Every path that ends the walk RETURNS at once (the walker is gone after Finish / Arrive).
    void StepWalker(std::int32_t a_id, float a_dt)
    {
        auto it = g_walkers.find(a_id);
        if (it == g_walkers.end()) return;
        Walker& w = it->second;
        w.t += a_dt;
        w.phaseT += a_dt;

        auto* a = RE::TESForm::LookupByID<RE::Actor>(w.who);
        if (!a) { Finish(a_id, End::kGone, "she is gone (no form)", true); return; }
        std::string text;
        if (const End g = GateOf(a, text); g != End::kNone) { Finish(a_id, g, text, true); return; }
        if (standpoint_interrupts::IsPlayerGrabbing(a)) { Finish(a_id, End::kGrabbed, "the player's HIGGS hand holds her", true); return; }
        if (ChainedByVrChain(a)) {
            Finish(a_id, w.everDrove ? End::kTaken : End::kOtherDriver, "VR Chain has chained her (its chain brakes / pulls her)", true);
            return;
        }

        // ── the hook's receipts: one log line per change, outcome not intention ──
        const Slot sl = ReadSlot(w);
        if (sl.visits != w.visitsSeen) {
            w.visitsSeen = sl.visits;
            w.ticksNoVisit = 0;
        } else {
            ++w.ticksNoVisit;
        }
        if (sl.seen != w.lastSeen && sl.seen != Seen::kDriving) {
            logger::info("WALK #{} hook: {} (visit {}, t {:.2f} s)", a_id, SeenText(sl.seen), sl.visits, w.t);
        }
        w.lastSeen = sl.seen;
        if (sl.engages != w.engagesSeen) {
            w.engagesSeen = sl.engages;
            if (!w.everDrove) {
                w.everDrove = true;
                if (!w.startSent) {
                    w.startSent = true;
                    SendDirectWalkEvent(a, a_id, "start");
                }
            }
        }
        if (sl.losts != w.lostsSeen) {
            const int n = static_cast<int>(sl.losts - w.lostsSeen);
            w.lostsSeen = sl.losts;
            w.retakes += n;
            auto* st = a->AsActorState();
            logger::info("WALK #{} LOST control {} time(s) ({} in a row): staggered {}, knock {}, movement-animation-driven {} - "
                         "re-taken next frame", a_id, n, w.retakes, st->IsStaggered() ? 1 : 0, static_cast<int>(st->GetKnockState()),
                         a->IsMovementAnimationDriven() ? 1 : 0);
            if (w.retakes > kMaxRetakes) {
                Finish(a_id, End::kLostTooOften, std::format("lost control {} times in a row, never driven steadily in between",
                                                             w.retakes), true);
                return;
            }
        } else if (sl.seen == Seen::kDriving && w.retakes > 0) {
            // (review H #4) a steady driven visit: the losts so far were episodes she came back from (a stagger, a save's
            // hand-back). Only losts IN A ROW count - a long stand is never ended by the occasional re-take.
            w.retakes = 0;
        }
        // 1.5.1 THE IDLE BREAK (D §1): her animation owns her feet while she still has to walk to the point (the gate above has
        // passed: no furniture, ragdoll, knock, kill move, bleedout) -> IdleForceDefaultState ONCE in this walk (review R2 N1), not
        // during a stagger (it ends by itself), NEVER after the answer (review R2 B1: the hold's animation is the consumer's - she
        // is then handed back after kAnimLockS as in 1.5.0), and on a moving take-over only once the animation has held her feet
        // for breakAfter (review R2 R1: a locomotion transition passes by itself). The episode ends at a fresh driven visit.
        const bool breakAllowed = !w.answered && w.idleBreaks == 0 && !a->AsActorState()->IsStaggered();
        if (sl.seen == Seen::kAnimOwns) {
            w.animT += a_dt;
            if (w.idleBroken) {
                w.idleBreakT += a_dt;
            } else if (breakAllowed && w.animT >= w.breakAfter) {
                SendIdleBreak(a_id, w, a, false);
            }
        } else {
            const bool drivenNow = sl.seen == Seen::kEngaged || sl.seen == Seen::kDriving;
            if (w.idleBroken && drivenNow && sl.visits > w.breakVisits) {
                logger::info("WALK #{} IDLE BREAK worked: the hook drives her {:.2f} s after it", a_id, w.idleBreakT);
                w.idleBroken = false;
            } else if (w.idleBroken) {
                w.idleBreakT += a_dt;
            }
            w.animT = 0.f;
        }
        // (R1; verify V NEW-3) a moving take-over keeps its window for the walk's first breakAfter seconds, whatever the first
        // engage (a locomotion transition one visit later passes by itself too); then, once no episode runs, the window is over
        if (w.breakAfter > 0.f && w.t >= w.breakAfter && w.animT <= 0.f) w.breakAfter = 0.f;
        if (sl.seen == Seen::kOtherDriver) w.otherT += a_dt; else w.otherT = 0.f;
        if (sl.seen == Seen::kDidNotTake) w.noTakeT += a_dt; else w.noTakeT = 0.f;
        if (sl.seen == Seen::kNoController) w.noCtlT += a_dt; else w.noCtlT = 0.f;
        if (w.noCtlT > kAnimLockS) {
            Finish(a_id, End::kGone, std::format("she has had no movement controller for {:.1f} s", w.noCtlT), true);
            return;
        }
        // (a break still due - a moving take-over's window - comes first: the end waits for it and its own 2 s)
        if (w.idleBroken ? (sl.seen == Seen::kAnimOwns && w.idleBreakT > kAnimLockS)
                         : (w.animT > kAnimLockS + (breakAllowed ? w.breakAfter : 0.f))) {
            Finish(a_id, End::kAnimLock,
                   w.idleBroken ? std::format("her animation still owned her feet {:.1f} s after the idle break (IdleForceDefaultState)",
                                              w.idleBreakT)
                                : std::format("her animation owned her feet for {:.1f} s ({})", w.animT,
                                              w.answered          ? "the hold after the answer: her animation is never broken there - "
                                                                    "the hold ends, as in 1.5.0" :
                                              w.idleBreaks > 0    ? "this walk's one idle break was used" :
                                                                    "a stagger: no idle break sent"),
                   true);
            return;
        }
        if (w.noTakeT > kAnimLockS) {
            Finish(a_id, End::kNoTake, std::format("the take never held for {:.1f} s (her movement type never became planner "
                                                   "direct; type {})", w.noTakeT, ControllerOf(a) ? MoveType(ControllerOf(a)) : 0u), true);
            return;
        }
        if (w.otherT > kOtherDriverS) {
            Finish(a_id, w.everDrove ? End::kTaken : End::kOtherDriver,
                   std::format("another system held her under planner direct control for {:.1f} s - {}", w.otherT, HolderText(a)), true);
            return;
        }
        if (w.ticksNoVisit > kStarveTicks) {
            Finish(a_id, w.everDrove ? End::kTaken : End::kOtherDriver,
                   std::format("the motion check has not reached Standpoint for her in {} frames (a driver upstream has her) - {}",
                               w.ticksNoVisit, HolderText(a)), true);
            return;
        }

        // ── where she is ──
        const RE::NiPoint3 pos = a->GetPosition();
        const float        yaw = a->GetAngleZ();
        const float        moved = Dist2D(pos, w.lastPos);
        if (moved > kWarpDist) {
            Finish(a_id, End::kWarp, std::format("her root jumped {:.0f} u in one frame - never chased", moved), true);
            return;
        }
        if (a_dt > 0.0001f) {
            const float k = (std::clamp)(a_dt / 0.25f, 0.f, 1.f);
            w.speedMeas += (moved / a_dt - w.speedMeas) * k;
            w.velX += ((pos.x - w.lastPos.x) / a_dt - w.velX) * k;      // (review R #4: the direction she really moves in)
            w.velY += ((pos.y - w.lastPos.y) / a_dt - w.velY) * k;
            // 1.6.0 (S2 §3.5) her yaw rate, deg/s (smoothed) - the trace's, to tune the 30 / 60 deg turn caps from one run
            if (w.haveYaw) {
                const float ky = (std::clamp)(a_dt / kYawRateK, 0.f, 1.f);
                w.yawRate += (std::fabs(Deg(Wrap(yaw - w.lastYaw))) / a_dt - w.yawRate) * ky;
            }
            w.lastYaw = yaw;
            w.haveYaw = true;
        }
        w.lastPos = pos;

        // ── the swim watch ──
        auto*       cell = a->GetParentCell();
        const float lvl = cell ? a->GetSubmergedLevel(pos.z, cell) : 0.f;
        w.submerged = lvl;                                         // (1.6.1 B4: BandCap's "in the water")
        const bool  swimming = a->AsActorState()->IsSwimming();
        if (swimming || lvl >= kSwimLevel) {
            Finish(a_id, End::kWouldSwim, std::format("submerged {:.2f}, swimming {} - she would swim", lvl, swimming ? 1 : 0), true);
            return;
        }

        // ── the route ──
        Follow(w, pos);
        bool cornerPassed = false;
        if (const int c = NextSharp(w); c >= 0 && w.s >= w.cum[c] - kCornerReach) {
            w.sharpPassedS = w.cum[c];
            cornerPassed = true;
            logger::info("WALK #{} SHARP corner {} reached ({:.0f} u from it)", a_id, c, Dist2D(pos, w.pts[c].pos));
        }
        const RE::NiPoint3 aim = PointAt(w, AimS(w));
        const float        pursuit = Dist2D(pos, aim) > 1.f ? HeadingTo(pos, aim) : (w.haveTravel ? w.travelDir : yaw);
        const float        err = Wrap(pursuit - yaw);
        const RE::NiPoint3 goal = w.pts.back().pos;      // (v1.4.1: a copy - the re-approach below replaces w.pts)
        const float        dGoal = Dist2D(pos, goal);
        const int          n = static_cast<int>(w.pts.size());
        const bool         atEnd = dGoal <= kStopRadius || n < 2 || (w.seg >= n - 2 && w.tSeg >= 1.f);
        const bool         driven = sl.seen == Seen::kEngaged || sl.seen == Seen::kDriving;
        float              fin = 0.f;                              // v1.4.1: the heading she must end with (from here)
        bool               haveFin = FinalHeading(w, pos, fin);
        if (!haveFin && w.keepFace) {                              // (review R #2) a keep-travel walk's re-approach
            fin = w.keptFace;
            haveFin = true;
        }

        switch (w.phase) {
        case Phase::kTurn:
        case Phase::kWalk:
            if (w.prm.stopSubmerged > 0.f && lvl >= w.prm.stopSubmerged) {
                w.arriveWhy = End::kDepthReached;
                EnterPhase(w, Phase::kSettle, std::format("submerged {:.2f} >= the asked {:.2f}", lvl, w.prm.stopSubmerged).c_str());
                break;
            }
            if (atEnd && w.prm.remainAfterU > 0.f) {
                // 1.6.0: the end of the VERIFIED route, more route after it (the next chunk is late): she stands and waits for
                // Extend - never an arrival here
                EnterPhase(w, Phase::kWait, std::format("the end of the verified route ({:.0f} u from it), {:.0f} u of unverified route "
                                                        "after it - she waits for the next chunk", dGoal, w.prm.remainAfterU).c_str());
                break;
            }
            if (atEnd) {
                w.arriveWhy = End::kArrived;
                EnterPhase(w, Phase::kSettle, std::format("{:.0f} u from the last point", dGoal).c_str());
                break;
            }
            if (w.xtrack > kStrayDist) {
                // 1.5.0 (audit C fix 2): a push off the line is no end - she is held where she was pushed to and WalkApi plans
                // again from there (never twice from one spot without progress; the 90 s bound it)
                PauseAndReport(a_id, w, End::kStrayed, pos, yaw, lvl, pos, 0.f, 0,
                               std::format("PUSHED off the route: {:.0f} u off it - held for a new route from here", w.xtrack));
                return;
            }
            // 1.5.0 DOORS: near a closed door on the leg into a door point, she stops and opens it (the door phase below)
            if (w.phase == Phase::kWalk && !w.reapproach && DoorCheck(a_id, w, pos)) break;
            // (review H #3) a stand-and-turn that timed out LATCHES "turn while walking": no new stand-and-turn until she has
            // once lined up (under kTurnDoneDeg) or reaches the next SHARP corner. Without the latch a turn the engine will
            // not do at speed 0 flipped turn <-> walk every 2 s until the 90 s limit. Walking, the turn is in the steering
            // (direction = the route, facing = the route) and the progress watchdog still applies.
            if (w.turnLatched && (std::fabs(Deg(err)) < kTurnDoneDeg || cornerPassed)) {
                w.turnLatched = false;
                logger::info("WALK #{} turn latch cleared ({})", a_id, cornerPassed ? "a SHARP corner" : "she is lined up");
            }
            if (w.phase == Phase::kTurn) {
                if (std::fabs(Deg(err)) < kTurnDoneDeg) {
                    EnterPhase(w, Phase::kWalk, std::format("turned, {:.0f} deg off", Deg(err)).c_str());
                } else if (w.phaseT > kTurnMaxS) {
                    w.turnLatched = true;
                    EnterPhase(w, Phase::kWalk, std::format("still {:.0f} deg off after {:.1f} s at speed 0 - TURN LATCHED: she turns "
                                                            "while walking", Deg(err), w.phaseT).c_str());
                }
            } else if (!w.turnLatched) {
                // (v1.4.1: the "stand and turn first" is not entered while the facing blend runs - she faces away from her
                // travel direction on purpose then; review R nit 7: a SHARP corner's own turn still is, unless the blend
                // had already started before the corner)
                if (!BlendNow(w, haveFin) && std::fabs(Deg(err)) > kTurnFirstDeg) {
                    EnterPhase(w, Phase::kTurn, std::format("{:.0f} deg off - stand and turn first", Deg(err)).c_str());
                } else if (cornerPassed && !w.blendOn && std::fabs(Deg(err)) > kCornerTurnDeg) {
                    EnterPhase(w, Phase::kTurn, std::format("SHARP corner, {:.0f} deg to turn", Deg(err)).c_str());
                }
            }
            // (review R #4) the blend's own watch: told her walk while blending, she moves under half of it, or off the
            // steered direction (the engine does not strafe her) for kBlendCheckS -> the blend is OFF for the rest of this
            // walk, logged once: 1.4.0's walk (turn-first allowed again) and the turn at the point
            if (w.phase == Phase::kWalk && driven && w.blendOn && !w.blendOff && w.cmdSpeed >= kWalkNorm) {
                const float norm = NormalisedForLog(a, w.speedMeas);
                const bool  slow = norm >= 0.f && norm < kBlendSlowNorm;
                bool        offDir = false;
                float       dirErr = 0.f;
                if (w.speedMeas >= kBlendMoveMin) {
                    dirErr = std::fabs(Deg(Wrap(std::atan2(w.velX, w.velY) - w.cmdDir)));
                    offDir = dirErr > kBlendOffDirDeg;
                }
                w.blendBadT = (slow || offDir) ? w.blendBadT + a_dt : 0.f;
                if (w.blendBadT >= kBlendCheckS) {
                    w.blendOff = true;
                    w.holdFinal = false;
                    logger::info("WALK #{} facing blend OFF for the rest of this walk: {} for {:.1f} s while told {:.2f} (moved {:.0f} u/s, "
                                 "her normalised {:.2f}, {:.0f} deg off the steered direction) - 1.4.0's walk and the turn at the point",
                                 a_id, slow ? "under half her walk" : "off the steered direction", w.blendBadT, w.cmdSpeed, w.speedMeas,
                                 norm, dirErr);
                }
            }
            // the progress watchdog: only while she is told to walk AND the hook is driving her
            if (w.phase == Phase::kWalk && driven) {
                if (w.s > w.sBest + kProgressMin) {
                    w.sBest = w.s;
                    w.noProgressT = 0.f;
                } else {
                    w.noProgressT += a_dt;
                }
                if (w.noProgressT > kProgressWindowS && w.reapproach) {
                    // v1.4.1: the short re-approach is a correction, not a walk - no progress = she stays where she is: the
                    // facing again, then the arrival (C 0 / C 9 by the distance) - never a BLOCKED re-plan for it
                    w.arriveWhy = End::kArrived;
                    EnterPhase(w, Phase::kSettle, std::format("the re-approach made less than {:.0f} u in {:.1f} s - she stays, {:.0f} u "
                                                              "from the point", kProgressMin, w.noProgressT, dGoal).c_str());
                } else if (w.noProgressT > kProgressWindowS) {
                    // the bad spot: on the ROUTE ahead of her (the direction she was steered, not her yaw), at her radius +
                    // the planner's pad + 8 u - see kPlannerBadPad
                    const float radius = BodyRadius(a);
                    const float ahead = radius + kPlannerBadPad + kStallClear;
                    const float dir = w.haveTravel ? w.travelDir : pursuit;
                    const RE::NiPoint3 spot(pos.x + ahead * std::sin(dir), pos.y + ahead * std::cos(dir), pos.z);
                    logger::info("WALK #{} stall spot {:.0f} u ahead along the route ({:.0f} deg): the planner will refuse cells within "
                                 "{:.0f} u of it - from {:.0f} u in front of her centre on; her own spot stays legal", a_id, ahead,
                                 Compass(dir), radius + kPlannerBadPad, ahead - (radius + kPlannerBadPad));
                    PauseAndReport(a_id, w, End::kBlocked, pos, yaw, lvl, spot, radius + kPlannerBadPad, 0,
                                   std::format("BLOCKED: less than {:.0f} u along the route in {:.1f} s", kProgressMin, w.noProgressT));
                    return;
                }
            } else if (w.phase != Phase::kWalk) {
                w.noProgressT = 0.f;
            }
            break;
        case Phase::kSettle:
            if ((w.speedMeas < kSettleSpeed && w.phaseT > 0.2f) || w.phaseT > kSettleMaxS) {
                // (v1.4.1: haveFin = toward Params::faceWho read NOW, else the asked facing)
                w.finalFace = haveFin ? fin : (w.haveTravel ? w.travelDir : yaw);
                w.reapproach = false;       // (the re-approach, if it ran, is walked; `reapproached` keeps it to one)
                w.holdFinal = false;
                EnterPhase(w, Phase::kFace, std::format("stopped ({:.0f} u/s), {:.0f} u from the point; final facing {:.0f} deg{}, {:.0f} deg "
                                                        "to turn",
                                                        w.speedMeas, dGoal, Compass(w.finalFace),
                                                        !haveFin            ? " (her last travel direction)" :
                                                        w.prm.faceWho != 0u ? std::format(" (toward [{:08X}])", w.prm.faceWho) : std::string(),
                                                        std::fabs(Deg(Wrap(w.finalFace - yaw)))).c_str());
            }
            break;
        case Phase::kFace: {
            const float fe = std::fabs(Deg(Wrap(w.finalFace - yaw)));
            if (fe <= kFaceOkDeg || w.phaseT >= kFaceMaxS) {
                float left = fe;
                if (fe > kFaceSetDeg) {
                    a->SetHeading(w.finalFace);
                    left = std::fabs(Deg(Wrap(w.finalFace - a->GetAngleZ())));
                    logger::info("WALK #{} final facing: still {:.1f} deg off after {:.1f} s at speed 0 - her heading set directly "
                                 "(the 0.8.1 rule); now {:.1f} deg off", a_id, fe, w.phaseT, left);
                } else {
                    logger::info("WALK #{} final facing: {:.1f} deg off after {:.1f} s at speed 0", a_id, fe, w.phaseT);
                }
                // v1.4.1: ONE short re-approach - a walker who can strafe and still stands more than kReapproachMin from the
                // point (the stop and the turn left her there) walks back to it at her walk, her facing held; then the facing
                // again, and the arrival whatever the distance. Not after a depth stop (C 20 is where she stopped on purpose),
                // not when she was pushed far off (> kReapproachMax: she arrives as is, C 9), (review R) not when the blend
                // failed this walk (she does not strafe), not from off her planned last leg (an untested line). A keep-travel
                // walk keeps the heading she has now through it and after it (review R #2: never "the way she came").
                const float offWay = OffLastLeg(w, pos);
                if (w.canStrafe && !w.blendOff && !w.reapproached && w.arriveWhy == End::kArrived && dGoal > kReapproachMin &&
                    dGoal <= kReapproachMax && offWay <= kReapproachOffWay) {
                    if (!haveFin) {
                        w.keepFace = true;
                        w.keptFace = w.finalFace;
                    }
                    w.reapproached = true;
                    w.reapproach = true;
                    w.holdFinal = true;
                    logger::info("WALK #{} RE-APPROACH: {:.1f} u from the point after the facing (more than {:.0f} u), {:.0f} u off her last "
                                 "leg - ONE short walk back to it at her walk, facing held at {:.0f} deg{}; then the facing again", a_id, dGoal,
                                 kReapproachMin, offWay, Compass(w.finalFace), w.keepFace ? " (keep-travel: her heading now)" : "");
                } else if (w.canStrafe && !w.reapproached && w.arriveWhy == End::kArrived && dGoal > kReapproachMin) {
                    logger::info("WALK #{} no re-approach: {:.1f} u from the point, {:.0f} u off her last leg{}", a_id, dGoal, offWay,
                                 w.blendOff ? " - the blend failed this walk (no strafing)" :
                                 dGoal > kReapproachMax ? " - pushed too far off" : " - not on the way the planner tested");
                }
                if (w.reapproach && w.phase == Phase::kFace) {
                    // (the re-approach was just armed above: its route and phase)
                    PathPoint here{};
                    here.pos = pos;
                    const PathPoint end = w.pts.back();
                    w.pts.assign({ here, end });
                    BuildTrack(w);
                    w.doors.clear();                // (its arcs belonged to the old track; the re-approach checks no doors)
                    w.doorIdx = -1;
                    w.cmdFace = w.finalFace;        // the blend's rate limit starts from the facing she has now
                    EnterPhase(w, Phase::kWalk, "the re-approach");
                    break;                          // -> the order below: walk, direction at the point, facing held
                }
                logger::info("WALK #{} ARRIVED ({}): {:.1f} u from the point (flat), z {:+.0f} u, submerged {:.2f}, t {:.1f} s{}",
                             a_id, EndName(w.arriveWhy), dGoal, pos.z - goal.z, lvl, w.t,
                             w.reapproached ? " (after the re-approach)" : "");
                w.cmdSpeed = 0.f;
                w.cmdFace = w.finalFace;
                WriteOrder(w, true);
                Arrive(a_id, a, left);
                return;
            }
            break;
        }
        case Phase::kStand:
            if (w.holdLeft > 0.f) {
                w.holdLeft -= a_dt;
                if (w.holdLeft <= 0.f) {
                    Finish(a_id, End::kHoldOver, "the hold time is over - she is let go", true);
                    return;
                }
            }
            // v1.4.1 (Params::faceWho - the stpt follow program): she keeps facing him while she stands; a new facing only
            // once he is more than kStandReFaceDeg off it (no turning on every small step of his)
            if (w.prm.faceWho != 0u && haveFin && std::fabs(Deg(Wrap(fin - w.finalFace))) > kStandReFaceDeg) {
                logger::info("WALK #{} stand: re-faced toward [{:08X}] - he was {:.0f} deg off her facing; now {:.0f} deg", a_id,
                             w.prm.faceWho, std::fabs(Deg(Wrap(fin - w.finalFace))), Compass(fin));
                w.finalFace = fin;
            }
            break;
        case Phase::kPaused:
            break;
        case Phase::kWait:
            // 1.6.0: she waits at the end of the verified route. Extend brings the next chunk (it puts her back in the walk); an
            // empty Extend ("no more chunks" - the loaded edge, a chunk that failed) makes this end the goal: she arrives here
            if (w.prm.remainAfterU <= 0.f) {
                w.arriveWhy = End::kArrived;
                EnterPhase(w, Phase::kSettle, std::format("no more route after this end - she arrives here, {:.0f} u from it", dGoal).c_str());
            } else if (!atEnd) {
                EnterPhase(w, Phase::kWalk, "the route goes on");
            }
            break;
        case Phase::kDoor: {
            // 1.5.0: she stands (speed 0, facing her way on) before the door. ONE door rule for the planner and the driver
            // (review R2 S1 / S3): standpoint_path::IsOpenableDoor (base Door, an open / close state, not locked, not a load
            // door, no activate links - a trapped door would fire its trap), checked again AT the moment of opening (the route
            // may be old). It fails -> a wall: BLOCKED at once. Open -> she walks on (the door stays on her route, watched, until
            // she is past it). Opening -> she waits.
            // 1.6.1 (B2, run 2026-11-06 17:49: a door activated while CLOSING never opened - the activate turned it round, the
            // doorway became a 60 s bad spot and 33 walks failed): CLOSING -> she WAITS until it is shut (no activate meanwhile);
            // SHUT -> activated, by her; still shut kDoorWaitS later -> activated ONCE MORE; still shut kDoorWaitS after that (or
            // kDoorMaxS in all) -> BLOCKED, a new route from where she stands - the doorway is NOT a bad spot on this door's
            // first such failure in the walk (it may be the only way), only on a second one
            auto*      d = RE::TESForm::LookupByID<RE::TESObjectREFR>(w.doorRef);
            const bool loaded = d && d->Is3DLoaded();
            const auto state = loaded ? RE::BGSOpenCloseForm::GetOpenState(d) : RE::BGSOpenCloseForm::OPEN_STATE::kNone;
            if (state == RE::BGSOpenCloseForm::OPEN_STATE::kOpen) {
                logger::info("WALK #{} DOOR [{:08X}] opened after {:.1f} s{} - she walks through", a_id, w.doorRef, w.phaseT,
                             w.doorActs > 1 ? " (her second activate opened it)" : w.doorActs == 1 ? " (she opened it)" : " (it was opening)");
                if (w.doorIdx >= 0 && w.doorIdx < static_cast<int>(w.doors.size())) {
                    w.doors[w.doorIdx].openLogged = true;   // still on her route, watched as an open door until she is past it
                }
                EnterPhase(w, Phase::kWalk, "the door is open");
                break;
            }
            const bool wall = !loaded || !standpoint_path::IsOpenableDoor(d);
            if (!wall) {
                if (state == RE::BGSOpenCloseForm::OPEN_STATE::kClosing) {
                    if (!w.doorCloseLogged) {
                        w.doorCloseLogged = true;
                        logger::info("WALK #{} DOOR [{:08X}] is CLOSING - she waits until it is shut, then opens it (an activate now would "
                                     "turn it round)", a_id, w.doorRef);
                    }
                } else if (state == RE::BGSOpenCloseForm::OPEN_STATE::kClosed &&
                           (w.doorActs == 0 || (w.doorActs == 1 && w.phaseT - w.doorActT >= kDoorWaitS))) {
                    ++w.doorActs;
                    w.doorActT = w.phaseT;
                    const bool took = d->ActivateRef(a, 0, nullptr, 1, false);
                    logger::info("WALK #{} DOOR [{:08X}] closed: she activates it{} (the engine's own door activate, she is the activator) - {}",
                                 a_id, w.doorRef,
                                 w.doorActs > 1 ? std::format(" ONCE MORE (still shut {:.1f} s after her first)", kDoorWaitS) : std::string(),
                                 took ? "accepted" : "the activate returned false");
                }
            }
            const bool tried = w.doorActs >= 2 && w.phaseT - w.doorActT >= kDoorWaitS;
            if (wall || tried || w.phaseT >= kDoorMaxS) {
                // (the log's reason only - the decision above is IsOpenableDoor's)
                const auto* lock = loaded ? d->GetLock() : nullptr;
                const char* why = !loaded                                                  ? "it is not loaded any more" :
                                  !wall && w.doorActs >= 2                                 ? "still shut after two activates" :
                                  !wall && w.doorActs == 0                                 ? "it never came to rest shut (closing / opening too long)" :
                                  !wall                                                    ? "still not open" :
                                  d->extraList.HasType(RE::ExtraDataType::kTeleport)       ? "a LOAD door (a wall: never activated)" :
                                  (lock && lock->IsLocked())                               ? "LOCKED (a wall: never activated)" :
                                  (d->extraList.HasType(RE::ExtraDataType::kActivateRef) ||
                                   d->extraList.HasType(RE::ExtraDataType::kActivateRefChildren)) ?
                                                                                             "wired to other references - a trap or a script (a wall: never activated)" :
                                  state == RE::BGSOpenCloseForm::OPEN_STATE::kNone         ? "it has no open state (a wall: never activated)" :
                                                                                             "not openable by the planner's door rule (a wall: never activated)";
                const bool   known = w.doorIdx >= 0 && w.doorIdx < static_cast<int>(w.doors.size());
                RE::NiPoint3 spot = known ? w.doors[w.doorIdx].mid : pos;    // the middle of its doorway (the closed leaf's line)
                spot.z = pos.z;
                if (known) w.doors[w.doorIdx].done = true;
                const float radius = BodyRadius(a);
                const RE::FormID doorId = w.doorRef;
                // (1.6.1 B2) an OPENABLE door that stayed shut: its first failure in this walk marks NO bad spot - the doorway may be
                // the only way between two rooms (a 60 s bad spot there failed every plan); a second one does. A wall always does.
                const bool firstFail = !wall && std::find(w.doorFailed.begin(), w.doorFailed.end(), doorId) == w.doorFailed.end();
                if (firstFail) w.doorFailed.push_back(doorId);
                logger::info("WALK #{} DOOR [{:08X}] did not open ({}, after {:.1f} s, state {}, {} activate(s)) - BLOCKED: {}, a new route from "
                             "here", a_id, doorId, why, w.phaseT, OpenStateName(state), w.doorActs,
                             firstFail ? std::string("its first failure in this walk - the doorway is NOT made a bad spot (the new route may "
                                                     "try it again)")
                                       : std::format("the doorway is the bad spot (the planner refuses cells within {:.0f} u of it)",
                                                     radius + kPlannerBadPad));
                w.doorIdx = -1;
                w.doorRef = 0;
                PauseAndReport(a_id, w, End::kBlocked, pos, yaw, lvl, spot, radius + kPlannerBadPad, doorId,
                               std::format("BLOCKED: the door [{:08X}] did not open", doorId), !firstFail);
                return;
            }
            break;
        }
        }

        // ── the order ──
        switch (w.phase) {
        case Phase::kWalk: {
            // 1.6.0 THE SPEED BANDS (spec D §1 item 7, S2 §3): by route LEFT (the verified route left + the unverified rest): run
            // beyond 1,050 u, jog down to 210 u, walk the last 210 u, the downshifts anticipated by the ramp; capped by the turn
            // caps and slow zones ahead (BandCap); an UPSHIFT only into a band that would last >= 1 s (the goal's rule, and no cap
            // below it within a second more at its speed) - else the fastest band below that would, else she keeps hers. The
            // re-approach is walked. RAMPED (never a jump), and never between 0 and her walk (measured 1.4.0: she does not move
            // there) - a walk from a stand starts at her walk.
            const float vLeft = (std::max)(0.f, w.total - w.s);
            const float toGo = vLeft + w.prm.remainAfterU;
            int         wantBand = 0;
            const char* why = nullptr;
            float       whyAt = 0.f;
            bool        noCut = false;
            if (w.prm.bands && !w.reapproach) {
                const int   byGoal = BandByLeft(w, toGo);
                const char* capWhy = nullptr;
                float       capAt = 0.f;
                const int   cap = BandCap(w, w.band, 0.f, capWhy, capAt, noCut);
                wantBand = (std::min)(byGoal, cap);
                if (cap < byGoal) {
                    why = capWhy;
                    whyAt = capAt;
                }
                if (wantBand > w.band) {
                    int up = w.band;
                    for (int b = wantBand; b > w.band; --b) {
                        const char* w2 = nullptr;
                        float       a2 = 0.f;
                        bool        nc2 = false;
                        if (LastsASecond(w, b, toGo) && BandCap(w, b, BandUps(b) * kMinBandS, w2, a2, nc2) >= b) {
                            up = b;
                            break;
                        }
                    }
                    if (up < wantBand) why = "a faster band would last under 1 s - skipped";
                    wantBand = up;
                }
            }
            // the look-ahead for the next frames: 0.35 s at her speed (48 u at her walk, ~126 u at her run), 48 u near a kPtNoCut
            // corner (its 128 u chord failed: only the 48 u one was tested)
            w.lookAhead = noCut ? kLookAhead : (std::clamp)(kLookAheadS * w.speedMeas, kLookAhead, kLookAheadMax);
            const float wantNorm = BandNorm(wantBand);
            const float fromNorm = w.cmdSpeed < kWalkNorm ? kWalkNorm : w.cmdSpeed;
            w.cmdSpeed = RampToward(fromNorm, wantNorm, kSpeedRampPerS * a_dt);
            if (wantBand != w.band) {
                logger::info("WALK #{} SPEED {} -> {} at {:.0f} u to go ({:.0f} verified + {:.0f} after it){} (told {:.2f}, ramped toward "
                             "{:.2f}; moved {:.0f} u/s, look-ahead {:.0f} u, yaw rate {:.0f} deg/s)", a_id,
                             BandName(w.band), BandName(wantBand), toGo, vLeft, w.prm.remainAfterU,
                             !w.prm.bands     ? std::string(" (the bands are off: walk all the way)") :
                             w.reapproach     ? std::string(" (the re-approach)") :
                             why              ? std::format(" ({}{})", why, whyAt > 0.f ? std::format(", {:.0f} u ahead", whyAt) : std::string()) :
                             wantBand == 0    ? std::format(" (the last {:.0f} u)", kJogAboveU) :
                             wantBand == 1    ? std::format(" (within {:.0f} u)", kRunAboveU) : std::string(),
                             w.cmdSpeed, wantNorm, w.speedMeas, w.lookAhead, w.yawRate);
                w.band = wantBand;
            }
            w.cmdDir = pursuit;
            w.travelDir = pursuit;
            w.haveTravel = true;
            if (BlendNow(w, haveFin)) {
                // v1.4.1: the direction keeps pointing at the point; the facing target goes from the travel direction (48 u
                // before the end) to the final heading (at the end) - the re-approach holds the final heading all along -
                // and never moves faster than kFaceBlendRate
                const float rest = (std::max)(0.f, w.total - w.s);
                const float f = w.holdFinal ? 1.f : std::clamp(1.f - rest / kFaceBlendDist, 0.f, 1.f);
                // (review R nit 9) the side she turns toward is chosen once, at the blend's start: near 180 deg the shorter way
                // would flip with every step she makes across his line
                float span = Wrap(fin - pursuit);
                if (!w.blendOn) {
                    w.blendDir = span >= 0.f ? 1 : -1;
                } else if (std::fabs(span) > kBlendFlipRad && (span >= 0.f) != (w.blendDir > 0)) {
                    span += w.blendDir > 0 ? 2.f * kPi : -2.f * kPi;
                }
                const float want = Wrap(pursuit + f * span);
                if (!w.blendOn) {
                    w.blendOn = true;
                    logger::info("WALK #{} facing blend: from {:.0f} deg toward the final {:.0f} deg over the last {:.0f} u, at most "
                                 "{:.0f} deg/s (she can strafe; the direction stays at the point){}",
                                 a_id, Compass(w.cmdFace), Compass(fin), (std::min)(rest, kFaceBlendDist), Deg(kFaceBlendRate),
                                 w.reapproach ? " - the re-approach, facing held" : w.holdFinal ? " - the whole route, facing held" : "");
                }
                w.cmdFace = TurnToward(w.cmdFace, want, kFaceBlendRate * a_dt);
            } else {
                w.cmdFace = pursuit;
            }
            break;
        }
        case Phase::kTurn:
            w.cmdSpeed = 0.f;
            w.cmdDir = pursuit;
            w.cmdFace = pursuit;
            break;
        case Phase::kSettle:
            w.cmdSpeed = 0.f;
            if (w.blendOn && !w.blendOff && haveFin && w.canStrafe) {
                w.cmdFace = TurnToward(w.cmdFace, fin, kFaceBlendRate * a_dt);    // v1.4.1: the blend goes on while she stops
            } else {
                w.cmdFace = w.haveTravel ? w.travelDir : yaw;
            }
            break;
        case Phase::kFace:
        case Phase::kStand:
            w.cmdSpeed = 0.f;
            w.cmdFace = w.finalFace;
            break;
        case Phase::kPaused:
            w.cmdSpeed = 0.f;
            w.cmdFace = w.pauseYaw;
            break;
        case Phase::kDoor:            // 1.5.0: she waits before the door, facing her way on
            w.cmdSpeed = 0.f;
            w.cmdDir = pursuit;
            w.cmdFace = pursuit;
            break;
        case Phase::kWait:            // 1.6.0: she waits at the end of the verified route, facing the way she came along it
            w.cmdSpeed = 0.f;
            w.cmdFace = w.haveTravel ? w.travelDir : yaw;
            break;
        }
        // 1.6.0: stopped (a turn, a door, the wait for a chunk, the settle ...) = she starts again from her walk: the next band she
        // is told is chosen again from there (its 1 s rule included), never resumed as if she still ran
        if (w.phase != Phase::kWalk) w.band = 0;
        WriteOrder(w, true);

        // ── the trace ──
        const bool nearEnd = (w.phase == Phase::kWalk || w.phase == Phase::kSettle) && (w.total - w.s) <= kStopTraceDist &&
                             w.prm.remainAfterU <= 0.f;     // (1.6.0: the goal's end only, never a chunk's)
        if (nearEnd && w.stopLines < kStopTraceMax) {
            ++w.stopLines;
            logger::info("WALK #{} stop: {:.1f} u to the point (arc {:.1f}), told {:.2f}, moved {:.0f} u/s, phase {}", a_id, dGoal,
                         w.total - w.s, w.cmdSpeed, w.speedMeas, PhaseName(w.phase));
        }
        w.traceT += a_dt;
        if (w.phase == Phase::kStand) w.standT += a_dt;
        const float every = w.phase != Phase::kStand ? kTraceS : (w.standT < kStandTraceLongAfterS ? kStandTraceS : kStandTraceLongS);
        if (w.traceT >= every) {
            w.traceT = 0.f;
            const std::uintptr_t mc = ControllerOf(a);
            const int segShown = (std::min)(w.seg + 1, n - 1);
            logger::info("WALK #{} trace t {:.1f} s [{}] pos ({:.0f},{:.0f},{:.0f}) yaw {:.0f} | told {:.2f} ({}) dir {:.0f} face {:.0f} | "
                         "moved {:.0f} u/s (her normalised {:.2f}) | heading err {:.0f} deg, yaw rate {:.0f} deg/s, look-ahead {:.0f} u | "
                         "seg {}/{} s {:.0f}/{:.0f} u (+{:.0f} u after it) xtrack {:.0f} u, "
                         "next point kind {} depth {:.0f} | submerged {:.2f} swim {} | planner-direct {} ours {} (flag {} type {} high {}) | "
                         "hook visits {} {}",
                         a_id, w.t, PhaseName(w.phase), pos.x, pos.y, pos.z, Compass(yaw), w.cmdSpeed,
                         w.phase == Phase::kWalk ? BandName(w.band) : "stopped", Compass(w.cmdDir),
                         Compass(w.cmdFace), w.speedMeas, NormalisedForLog(a, w.speedMeas), Deg(err), w.yawRate, w.lookAhead, segShown,
                         n - 1, w.s, w.total, w.prm.remainAfterU,
                         w.xtrack, static_cast<int>(w.pts[segShown].kind), w.pts[segShown].depth, lvl, swimming ? 1 : 0,
                         mc && PlannerDirectNow(mc) ? 1 : 0, sl.owned ? 1 : 0, mc && PlannerFlag(mc) ? 1 : 0, mc ? MoveType(mc) : 0u,
                         mc && HighProcess(mc) ? 1 : 0, sl.visits, SeenText(sl.seen));
        }
    }

    // the start checks (shared by CanTake and Begin)
    bool CheckTake(RE::Actor* a_actor, End& a_why, std::string& a_text)
    {
        if (!g_hookLive.load()) {
            a_why = End::kNotSupported;
            a_text = g_notLiveText;    // 1.5.0: "SE/AE direct walk: OFF (...)" / "... NOT READY because ..." / VR: not installed
            return false;
        }
        if (!a_actor || a_actor->IsPlayerRef() || !a_actor->Is3DLoaded()) {
            a_why = End::kGone;
            a_text = "not a loaded NPC";
            return false;
        }
        AskVrChainOnce();
        if (const End g = GateOf(a_actor, a_text); g != End::kNone) {
            a_why = g;
            return false;
        }
        if (standpoint_interrupts::IsPlayerGrabbing(a_actor)) {
            a_why = End::kGrabbed;
            a_text = "the player's HIGGS hand holds her";
            return false;
        }
        const std::uintptr_t mc = ControllerOf(a_actor);
        if (!mc) {
            a_why = End::kGone;
            a_text = "she has no movement controller";
            return false;
        }
        bool ours = false;
        {
            std::scoped_lock lk(g_slotMtx);
            const int i = SlotOf(a_actor->GetFormID());
            ours = i >= 0 && (g_slot[i].owned || g_slot[i].driving);
        }
        if (PlannerDirectNow(mc) && !ours) {
            a_why = End::kOtherDriver;
            // 1.5.1 (D §5): the holder named when Standpoint can tell
            a_text = std::format("another plugin drives her right now (planner direct control that is not Standpoint's) - {}",
                                 HolderText(a_actor));
            return false;
        }
        if (ChainedByVrChain(a_actor)) {
            a_why = End::kOtherDriver;
            a_text = "VR Chain has chained her";
            return false;
        }
        const RE::NiPoint3 p = a_actor->GetPosition();
        auto*       cell = a_actor->GetParentCell();
        const float lvl = cell ? a_actor->GetSubmergedLevel(p.z, cell) : 0.f;
        if (a_actor->AsActorState()->IsSwimming() || lvl >= kSwimLevel) {
            a_why = End::kWouldSwim;
            a_text = std::format("she already swims (submerged {:.2f})", lvl);
            return false;
        }
        return true;
    }

}  // namespace

const char* EndName(End a_end)
{
    switch (a_end) {
    case End::kNone:          return "stopped";
    case End::kArrived:       return "arrived";
    case End::kDepthReached:  return "depth-reached";
    case End::kBlocked:       return "blocked";
    case End::kStrayed:       return "strayed";
    case End::kWouldSwim:     return "would-swim";
    case End::kGrabbed:       return "grabbed";
    case End::kOtherDriver:   return "other-driver";
    case End::kTaken:         return "taken";
    case End::kAnimLock:      return "animation-lock";
    case End::kLostTooOften:  return "lost-too-often";
    case End::kNoTake:        return "take-never-held";
    case End::kCombat:        return "combat";
    case End::kRagdoll:       return "ragdoll";
    case End::kKnocked:       return "knocked";
    case End::kKillMove:      return "kill-move";
    case End::kFurniture:     return "furniture";
    case End::kBleedout:      return "bleedout";
    case End::kMounted:       return "mounted";
    case End::kRestrained:    return "restrained";
    case End::kDead:          return "dead";
    case End::kGone:          return "gone";
    case End::kWarp:          return "warp";
    case End::kHoldOver:      return "hold-over";
    case End::kNotSupported:  return "not-supported";
    case End::kTableFull:     return "table-full";
    case End::kBadPath:       return "bad-path";
    }
    return "?";
}

bool CanTake(RE::Actor* a_actor, End& a_why, std::string& a_text)
{
    return CheckTake(a_actor, a_why, a_text);
}

End Gate(RE::Actor* a_actor, std::string& a_text)
{
    if (!a_actor) {
        a_text = "she is gone (no form)";
        return End::kGone;
    }
    return GateOf(a_actor, a_text);
}

bool Begin(RE::Actor* a_actor, std::vector<PathPoint> a_path, const Params& a_params, ReportFn a_report, End& a_why,
           std::string& a_text)
{
    if (!CheckTake(a_actor, a_why, a_text)) return false;
    if (a_path.empty()) {
        a_why = End::kBadPath;
        a_text = "the route is empty";
        return false;
    }
    if (g_walkers.count(a_params.walkId)) {
        a_why = End::kTableFull;
        a_text = "this walk is already driven";
        return false;
    }
    (void)AnimDrivenVar();       // the two graph-variable names are made here, on the main thread, before the hook reads them
    (void)AllowRotationVar();
    const RE::FormID who = a_actor->GetFormID();
    // her older walk (if any) is not this one. WalkApi hands a STANDING / walking older walk over to this one (it holds her
    // until now - review H #8): it is dropped here, its "start" closed with "stop|replaced" (review H #5)
    for (auto it = g_walkers.begin(); it != g_walkers.end(); ++it) {
        if (it->second.who == who) {
            logger::info("WALK #{} DRIVE: takes {} over from walk #{} (that walk is dropped here, she stays held)", a_params.walkId,
                         ActorText(a_actor), it->first);
            if (it->second.startSent) SendDirectWalkEvent(a_actor, it->first, "stop|replaced");
            g_walkers.erase(it);
            SetLive();
            break;
        }
    }
    int  slot = -1;
    bool heldNow = false;                         // the hook holds her right now (an older walk's slot, reused)
    {
        std::scoped_lock lk(g_slotMtx);
        slot = SlotOf(who);                       // a slot of hers that is still handing back: reused (our flag stays ours)
        if (slot < 0) {
            for (int i = 0; i < kSlots && slot < 0; ++i) {
                if (!g_slot[i].who) slot = i;
            }
            if (slot >= 0) {
                g_slot[slot] = Slot{};
                g_slot[slot].who = who;
                g_slotWho[slot].store(who, std::memory_order_release);
                g_inUse.fetch_add(1, std::memory_order_acq_rel);
            }
        } else {
            Slot& s = g_slot[slot];
            heldNow = s.owned || s.driving;
            // (review H #9) a slot left from a walk that ended (no order): its "driving" belief may be stale - the next
            // visit goes through the enter ritual again. `owned` stays: a flag we set is still ours.
            if (!s.wantDrive) {
                s.driving = false;
                g_drivenState[slot].store(0, std::memory_order_release);
            }
        }
        if (slot >= 0) {
            Slot& s = g_slot[slot];
            s.stateAddr = reinterpret_cast<std::uintptr_t>(a_actor->AsActorState());
            s.wantDrive = true;
            s.speed = 0.f;
            s.dirZ = a_actor->GetAngleZ();
            s.faceZ = a_actor->GetAngleZ();
            ++s.orderSeq;
        }
    }
    if (slot < 0) {
        a_why = End::kTableFull;
        a_text = std::format("all {} driver slots are in use", kSlots);
        return false;
    }
    Walker w{};
    w.prm = a_params;
    w.report = a_report;
    w.who = who;
    w.slot = slot;
    w.pts = std::move(a_path);
    BuildTrack(w);
    CollectDoors(a_params.walkId, w, a_actor);     // 1.5.0: every door whose doorway her route crosses, in route order
    w.lastPos = a_actor->GetPosition();
    w.cmdDir = a_actor->GetAngleZ();
    w.cmdFace = w.cmdDir;
    {
        const Slot sl = ReadSlot(w);
        w.visitsSeen = sl.visits;
        w.engagesSeen = sl.engages;
        w.lostsSeen = sl.losts;
        w.lastSeen = sl.seen;
    }
    if (heldNow) {
        // (review H #5) she is under our control already (an older walk's): this walk drives her from its first frame - it
        // opens its own "start" (the older one was closed above / by its Stop) and its ends map as "after driving"
        w.everDrove = true;
        w.startSent = true;
        SendDirectWalkEvent(a_actor, a_params.walkId, "start");
    }
    // the first phase: stand and turn first if the route starts behind her - (v1.4.1) unless she can strafe and the whole
    // route lies inside the facing blend: then she walks it facing toward her final heading at once (no turn there and back)
    const RE::NiPoint3 aim = PointAt(w, AimS(w));
    // (1.6.0: a one-point route at her feet - HoldHere, the near rule - has no direction: no turn first)
    const float        err = Dist2D(w.lastPos, aim) > 1.f ? Wrap(HeadingTo(w.lastPos, aim) - a_actor->GetAngleZ()) : 0.f;
    // v1.4.1: the blend uses it. 1.5.0: read on VR ONLY - CommonLib's CanStrafe is RELOCATION_ID(88498, 0): AE id 0 = a fatal
    // lookup; off VR she turns at the point (1.4.0's way), the blend and the re-approach stay off
    const bool canStrafe = REL::Module::IsVR() && a_actor->AsActorState()->CanStrafe();
    w.canStrafe = canStrafe;
    float      fin0 = 0.f;
    const bool haveFin0 = FinalHeading(w, w.lastPos, fin0);
    // (review R nit 10) also a short route (<= 2 x the blend) when her facing now is nearer the final heading than the route's
    // direction is - the follow program's commonest step (he steps forward: she backs away along his line, facing him)
    const bool shortHold = canStrafe && haveFin0 && NextSharp(w) < 0 && w.total <= kHoldFinalRoute && w.prm.remainAfterU <= 0.f &&
                           std::fabs(Wrap(fin0 - a_actor->GetAngleZ())) < std::fabs(err);
    const bool faceAll = FaceAllNow(w, haveFin0);     // 1.6.0 D2: the orbit's side-step - facing held over the whole route
    const bool allBlend = BlendNow(w, haveFin0) || shortHold || faceAll;
    w.holdFinal = allBlend;       // (a turn toward her travel direction and back would be the very drift the blend removes)
    w.phase = (!allBlend && std::fabs(Deg(err)) > kTurnFirstDeg) ? Phase::kTurn : Phase::kWalk;
    w.lookAhead = kLookAhead;
    logger::info("WALK #{} DRIVE start{}: {} from ({:.0f},{:.0f},{:.0f}) yaw {:.0f} deg, can strafe {}, {} points, {:.0f} u{}, first phase {} "
                 "({:.0f} deg off{}); told 0 at {:.0f} u from the point (stop radius); final facing {}; facing blend over the last "
                 "{:.0f} u {}; hold {}; stop at submerged {}; arbiter parameters {}; driver slot {}",
                 a_params.walkId, a_params.steerId ? std::format(" (STEER #{})", a_params.steerId) : std::string(), ActorText(a_actor),
                 w.lastPos.x, w.lastPos.y, w.lastPos.z, Compass(a_actor->GetAngleZ()),
                 canStrafe ? 1 : 0, w.pts.size(), w.total,
                 a_params.remainAfterU > 0.f ? std::format(" verified (+{:.0f} u of unverified route after it: more chunks come)",
                                                           a_params.remainAfterU) : std::string(),
                 PhaseName(w.phase), Deg(err),
                 !allBlend ? "" : faceAll ? " - D2 faceAll: her final facing held over the whole route (she side-steps)" :
                 shortHold && w.total > kFaceBlendDist ? " - no turn: a short route, her facing is nearer the final heading - held all the way"
                                                       : " - the whole route is inside the facing blend: facing held",
                 kStopRadius,
                 a_params.faceWho != 0u ? std::format("toward [{:08X}]{} (read live{})", a_params.faceWho,
                                                 std::fabs(a_params.faceOffsetRad) > 0.0001f ?
                                                     std::format(" {:+.0f} deg", Deg(a_params.faceOffsetRad)) : std::string(),
                                                 haveFin0 ? std::format(", now {:.0f} deg", Compass(fin0)) : std::string()) :
                 a_params.hasFacing ? std::format("{:.0f} deg", Compass(a_params.facingRad)) : std::string("her last travel direction"),
                 kFaceBlendDist, !REL::Module::IsVR() ? "OFF (strafing is read on VR only: 1.4.0's turn at the point)" :
                                 !canStrafe ? "OFF (she cannot strafe: 1.4.0's turn at the point)" :
                                 !haveFin0  ? "OFF (no final facing asked)" : "ON (then one re-approach if more than 16 u off)",
                 a_params.holdSeconds < 0.f ? std::string("-1 (stand until released)") : std::format("{:.1f} s", a_params.holdSeconds),
                 a_params.stopSubmerged > 0.f ? std::format("{:.2f}", a_params.stopSubmerged) : std::string("off"),
                 g_paramsLive.load() ? "Standpoint's (accel 10, decel 10, turn 10, rotation 2.5)" : "HER OWN (the arbiter hooks are not installed)",
                 slot);
    // 1.5.1 (D §2): the walk's speed rule; a take-over while she MOVES (the far phase's hand-over) starts the order at her pace -
    // normalised, read on VR (the far phase is VR only), between her walk and her full run - and ramps from there: no stop
    float startNorm = 0.f, startRead = -1.f;
    if (a_params.startSpeedUps > 1.f) {
        startRead = NormalisedForLog(a_actor, a_params.startSpeedUps);
        startNorm = startRead > kWalkNorm ? (std::min)(startRead, kRunNorm) : kWalkNorm;
    }
    if (startNorm > 0.f && w.phase == Phase::kWalk) {
        w.cmdSpeed = startNorm;
        w.cmdDir = HeadingTo(w.lastPos, aim);
        w.cmdFace = w.cmdDir;
        w.band = startNorm >= 0.5f * (kJogNorm + kRunNorm) ? 2 : startNorm >= 0.5f * (kWalkNorm + kJogNorm) ? 1 : 0;   // (her pace)
        WriteOrder(w, true);      // the hook's first visit drives her on at her pace (the slot's first order above said 0)
    }
    logger::info("WALK #{} SPEED RULE: {}{}", a_params.walkId,
                 a_params.bands ? std::format("{:.0f} u of route{} ({:.0f} u straight at the request) - by the route LEFT: RUN (told {:.1f}) "
                                              "beyond {:.0f} u, JOG ({:.1f}) down to {:.0f} u, WALK ({:.1f}) the last {:.0f} u; a band that "
                                              "would last under {:.0f} s is skipped; ramped {:.1f} a second, the downshifts anticipated; "
                                              "capped ahead of turns (over {:.0f} deg jog, over {:.0f} deg / SHARP / doors walk), water, "
                                              "steep legs, stairs, drop edges and the end of the verified route; look-ahead {:.2f} s x her "
                                              "speed ({:.0f}-{:.0f} u)",
                                              w.total + a_params.remainAfterU,
                                              a_params.remainAfterU > 0.f ? std::format(" ({:.0f} verified so far)", w.total) : std::string(),
                                              a_params.d0, kRunNorm, kRunAboveU, kJogNorm, kJogAboveU, kWalkNorm, kJogAboveU, kMinBandS,
                                              kSpeedRampPerS, kRunTurnDeg, kSlowTurnDeg, kLookAheadS, kLookAhead, kLookAheadMax)
                                : std::format("the bands are off - she walks all the way (told {:.1f})", kWalkNorm),
                 startNorm <= 0.f                ? std::string() :
                 w.phase == Phase::kWalk         ? std::format("; taken over MOVING at {:.0f} u/s (her normalised {:.2f}): the order starts "
                                                               "at {:.2f} and is ramped - no stop", a_params.startSpeedUps, startRead, startNorm) :
                                                   std::format("; taken over moving at {:.0f} u/s, but the route starts behind her: she "
                                                               "stops and turns first", a_params.startSpeedUps));
    // 1.5.1 THE IDLE BREAK at the walk's start (D §1): her animation owns her feet now (a random idle while she stood) - broken
    // at once. CheckTake's gate has refused furniture / sitting / sleeping, ragdoll, knock, kill move, bleedout; never during a
    // stagger (it ends by itself). (review R2 R1) NOT at a moving take-over (the far phase's hand-over): a running NPC's
    // locomotion transition can set the same graph variables - the break only comes if her animation still owns her feet after
    // kAnimLockS of the hook's attempts (StepWalker; breakAfter)
    const bool movingTake = a_params.startSpeedUps > 1.f;
    w.breakAfter = movingTake ? kAnimLockS : 0.f;
    if (!FeetAreFree(a_actor)) {
        if (movingTake) {
            logger::info("WALK #{} her animation owns her feet at the MOVING take-over (a locomotion transition?) - no idle break unless "
                         "it still owns them after {:.1f} s", a_params.walkId, kAnimLockS);
        } else if (!a_actor->AsActorState()->IsStaggered()) {
            SendIdleBreak(a_params.walkId, w, a_actor, true);
        }
    }
    g_walkers.emplace(a_params.walkId, std::move(w));
    SetLive();
    return true;
}

namespace {
    // (1.6.0 Swap) the far hand-over's join rule (1.5.1 WalkApi JoinIndex): the leg of a_path nearest to a_pos on her floor (2D; the
    // leg's height there within kSwapJoinZ of her feet; a later leg wins a tie) - the index of its END point (the first point
    // kept). 1 when no leg is on her floor (the route as planned: its first leg).
    constexpr float kSwapJoinZ   = 128.f;
    constexpr float kSwapOnTrack = 24.f;     // the plan's start lies on her current route when within this of it (2D)

    std::size_t NearestLegEnd(const std::vector<PathPoint>& a_path, const RE::NiPoint3& a_pos, float& a_off)
    {
        std::size_t best = 1;
        float       bestD = 3.0e38f;
        for (std::size_t k = 1; k < a_path.size(); ++k) {
            const RE::NiPoint3& a = a_path[k - 1].pos;
            const RE::NiPoint3& b = a_path[k].pos;
            const float vx = b.x - a.x, vy = b.y - a.y;
            const float len2 = vx * vx + vy * vy;
            float       t = len2 > 0.0001f ? ((a_pos.x - a.x) * vx + (a_pos.y - a.y) * vy) / len2 : 1.f;
            t = (std::clamp)(t, 0.f, 1.f);
            const float d = std::hypot(a.x + vx * t - a_pos.x, a.y + vy * t - a_pos.y);
            const float z = a.z + (b.z - a.z) * t;
            if (std::fabs(z - a_pos.z) <= kSwapJoinZ && d <= bestD + 0.5f) {
                best = k;
                bestD = (std::min)(bestD, d);
            }
        }
        a_off = bestD > 1.0e37f ? (a_path.size() > 1 ? Dist2D(a_pos, a_path[1].pos) : 0.f) : bestD;
        return best;
    }

    // a final heading is asked of her right now (FinalHeading from where she stands)
    bool HasFinalNow(const Walker& w)
    {
        float       f = 0.f;
        auto* const a = RE::TESForm::LookupByID<RE::Actor>(w.who);
        return FinalHeading(w, a ? a->GetPosition() : w.lastPos, f);
    }

    // (1.6.0 Swap / SwapJoin - review R2 #2) WHERE a new route joins her: its plan's start a_path.front() still AHEAD of her on the
    // route she walks (sJ > s: her route up to it is kept, then the new one - no new line), else the new route's leg nearest to her
    // on her floor (first = its end point; off = how far she is from it: the line her position -> a_path[first] is UNTESTED)
    struct JoinInfo {
        bool        ahead{ false };
        float       sJ{ -1.f };       // the start's arc on her current route (-1 = not on it)
        std::size_t first{ 1 };       // the first point of a_path walked after the join
        float       off{ 0.f };       // her distance to the joined leg (0 when ahead)
    };

    JoinInfo JoinOf(Walker& w, const RE::NiPoint3& a_pos, const std::vector<PathPoint>& a_path)
    {
        JoinInfo j{};
        Follow(w, a_pos);                                  // her arc position on the route she walks, NOW
        const RE::NiPoint3 start = a_path.front().pos;
        float              onBest = kSwapOnTrack;
        const int          n = static_cast<int>(w.pts.size());
        for (int k = (std::max)(0, w.seg); k < n - 1; ++k) {
            float t = 0.f, d = 0.f;
            Project(w, k, start, t, d);
            const float z = w.pts[k].pos.z + t * (w.pts[k + 1].pos.z - w.pts[k].pos.z);
            if (d < onBest && std::fabs(z - start.z) <= kSwapJoinZ) {
                onBest = d;
                j.sJ = w.cum[k] + t * (w.cum[k + 1] - w.cum[k]);
            }
        }
        if (j.sJ > w.s + 1.f) {
            j.ahead = true;
            j.first = 1;
            j.off = 0.f;
            return j;
        }
        j.first = NearestLegEnd(a_path, a_pos, j.off);
        return j;
    }
}  // namespace

bool Extend(std::int32_t a_walkId, const std::vector<PathPoint>& a_pts, float a_remainAfterU, std::optional<float> a_facingRad)
{
    auto it = g_walkers.find(a_walkId);
    if (it == g_walkers.end()) return false;
    Walker& w = it->second;
    if (w.answered || w.pts.empty()) return false;
    const float       before = w.total;
    const float       afterWas = w.prm.remainAfterU;
    const std::size_t from = w.pts.size() - 1;     // the old last point: the new legs start there
    w.prm.remainAfterU = (std::max)(0.f, a_remainAfterU);
    if (a_facingRad) {
        w.prm.hasFacing = true;
        w.prm.facingRad = *a_facingRad;
    }
    std::size_t added = 0;
    for (std::size_t k = 0; k < a_pts.size(); ++k) {
        if (k == 0 && Dist2D(a_pts[k].pos, w.pts.back().pos) < 1.f) continue;   // the chunk's start = the old last point
        w.pts.push_back(a_pts[k]);
        ++added;
    }
    if (added > 0) {
        ExtendTrack(w, from + 1);
        CollectDoors(a_walkId, w, RE::TESForm::LookupByID<RE::Actor>(w.who), static_cast<int>(from));
        ++w.extends;
    }
    logger::info("WALK #{} EXTEND #{}: {} - the verified route {:.0f} -> {:.0f} u ({} point(s) added), she is at s {:.0f} ({:.0f} u of it "
                 "ahead of her), the unverified rest {:.0f} -> {:.0f} u{}", a_walkId, w.extends,
                 added > 0 ? std::string("the next verified chunk appended") :
                 w.prm.remainAfterU <= 0.f ? std::string("NO MORE CHUNKS - the end of the route as it is becomes the goal") :
                                             std::string("MORE ROUTE COMES (a new one is planned) - she never arrives at this end"),
                 before, w.total, added, w.s, (std::max)(0.f, w.total - w.s), afterWas, w.prm.remainAfterU,
                 w.phase == Phase::kWait && added > 0 ? " - she waited at the old end: she walks on" : "");
    if (added > 0 && (w.phase == Phase::kWait || w.phase == Phase::kSettle || w.phase == Phase::kFace)) {
        w.arriveWhy = End::kArrived;
        EnterPhase(w, Phase::kWalk, "the route goes on (the next verified chunk)");
    } else if (added == 0 && w.prm.remainAfterU > 0.f && (w.phase == Phase::kSettle || w.phase == Phase::kFace)) {
        EnterPhase(w, Phase::kWait, "more route comes (a new one is planned) - she waits here, no arrival");
    }
    return true;
}

bool SetFacing(std::int32_t a_walkId, bool a_hasFacing, float a_facingRad, RE::FormID a_faceWho, float a_faceOffsetRad, bool a_faceAll)
{
    auto it = g_walkers.find(a_walkId);
    if (it == g_walkers.end()) return false;
    Walker& w = it->second;
    w.prm.hasFacing = a_hasFacing;
    w.prm.facingRad = a_facingRad;
    w.prm.faceWho = a_faceWho;
    w.prm.faceOffsetRad = a_faceOffsetRad;
    const bool wasAll = w.prm.faceAll;
    w.prm.faceAll = a_faceAll;
    if (wasAll && !a_faceAll && !w.reapproach) w.holdFinal = false;    // (the blend's own rule takes over again)
    if (a_faceAll && !w.answered) w.holdFinal = FaceAllNow(w, HasFinalNow(w));
    return true;
}

bool Pause(std::int32_t a_walkId, const char* a_why)
{
    auto it = g_walkers.find(a_walkId);
    if (it == g_walkers.end()) return false;
    Walker& w = it->second;
    if (w.answered) return false;
    if (w.phase == Phase::kPaused) return true;
    auto* a = RE::TESForm::LookupByID<RE::Actor>(w.who);
    w.pauseYaw = a ? a->GetAngleZ() : w.cmdFace;
    EnterPhase(w, Phase::kPaused, std::format("held at speed 0 - {} (no report: a new route follows)", a_why ? a_why : "paused").c_str());
    w.cmdSpeed = 0.f;
    w.cmdFace = w.pauseYaw;
    WriteOrder(w, true);
    return true;
}

bool SwapJoin(std::int32_t a_walkId, const std::vector<PathPoint>& a_path, RE::NiPoint3& a_from, RE::NiPoint3& a_to, float& a_off)
{
    auto it = g_walkers.find(a_walkId);
    if (it == g_walkers.end() || a_path.empty()) return false;
    Walker& w = it->second;
    auto* a = RE::TESForm::LookupByID<RE::Actor>(w.who);
    if (w.answered || !a || w.pts.empty()) return false;
    const RE::NiPoint3 pos = a->GetPosition();
    const JoinInfo     j = JoinOf(w, pos, a_path);
    a_from = pos;
    a_to = j.ahead ? a_path.front().pos : a_path[(std::min)(j.first, a_path.size() - 1)].pos;
    a_off = j.off;
    return true;
}

bool Swap(std::int32_t a_walkId, std::vector<PathPoint> a_path, float a_remainAfterU, std::optional<float> a_facingRad)
{
    auto it = g_walkers.find(a_walkId);
    if (it == g_walkers.end() || a_path.empty()) return false;
    Walker& w = it->second;
    if (w.answered) return false;
    auto* a = RE::TESForm::LookupByID<RE::Actor>(w.who);
    if (!a || w.pts.empty()) return false;
    const RE::NiPoint3 pos = a->GetPosition();
    const JoinInfo     j = JoinOf(w, pos, a_path);     // (where the new route's plan started vs the route she walks NOW)
    const int          n = static_cast<int>(w.pts.size());
    std::vector<PathPoint> pts;
    pts.reserve(a_path.size() + 8);
    PathPoint here{};
    here.pos = pos;
    pts.push_back(here);
    std::size_t first = j.first;                       // the first point of a_path walked after what comes before it
    std::string how;
    if (j.ahead) {
        // the plan's start is still ahead of her on the route she walks: that route up to it, then the new one
        int kept = 0;
        for (int k = w.seg + 1; k < n && w.cum[k] < j.sJ - 1.f; ++k) {
            pts.push_back(w.pts[k]);
            ++kept;
        }
        pts.push_back(a_path.front());
        first = 1;
        how = std::format("the new route starts {:.0f} u ahead of her on her current one ({} point(s) of it kept up to there)", j.sJ - w.s,
                          kept);
    } else {
        how = j.sJ >= 0.f ? std::format("she has passed the new route's start - joined at its leg {} ({:.0f} u off it)", first, j.off)
                          : std::format("the new route's start is not on her route - joined at its leg {} ({:.0f} u off it)", first, j.off);
    }
    for (std::size_t k = first; k < a_path.size(); ++k) {
        if (pts.size() == 1 && k + 1 < a_path.size() && Dist2D(a_path[k].pos, pos) < 1.f) continue;   // a point at her feet
        pts.push_back(a_path[k]);
    }
    const Phase was = w.phase;
    w.pts = std::move(pts);
    BuildTrack(w);
    w.prm.remainAfterU = (std::max)(0.f, a_remainAfterU);
    if (a_facingRad) {
        w.prm.hasFacing = true;
        w.prm.facingRad = *a_facingRad;
    }
    w.blendOn = false;            // the new route's own facing blend; a blend that failed on this walk stays OFF (blendOff)
    w.blendBadT = 0.f;
    w.holdFinal = FaceAllNow(w, HasFinalNow(w));
    w.reapproach = false;         // a new point: one re-approach is allowed again
    w.reapproached = false;
    w.keepFace = false;
    w.turnLatched = false;
    w.arriveWhy = End::kArrived;
    w.doorIdx = -1;
    w.doorRef = 0;
    w.doorActs = 0;
    w.doorCloseLogged = false;
    CollectDoors(a_walkId, w, a);
    ++w.swaps;
    logger::info("WALK #{} SWAP #{}: a new route joined where she is - {}; {} point(s), {:.0f} u{}{}; she goes on ({}, told {:.2f}, moving "
                 "{:.0f} u/s){}", a_walkId, w.swaps, how, w.pts.size(), w.total,
                 w.prm.remainAfterU > 0.f ? std::format(" verified + {:.0f} u after it", w.prm.remainAfterU) : std::string(),
                 a_facingRad ? std::format(", final facing {:.0f} deg", Compass(*a_facingRad)) : std::string(), PhaseName(was), w.cmdSpeed,
                 w.speedMeas, w.holdFinal ? " - D2 faceAll: facing held over the whole route" : "");
    if (w.phase != Phase::kWalk) EnterPhase(w, Phase::kWalk, "a new route on the move (Swap)");
    return true;
}

bool GetProgress(std::int32_t a_walkId, Progress& a_out)
{
    a_out = Progress{};
    const auto it = g_walkers.find(a_walkId);
    if (it == g_walkers.end()) return false;
    const Walker& w = it->second;
    auto* const   a = RE::TESForm::LookupByID<RE::Actor>(w.who);
    a_out.pos = a ? a->GetPosition() : w.lastPos;
    a_out.s = w.s;
    a_out.total = w.total;
    a_out.remainAfterU = w.prm.remainAfterU;
    a_out.speedUps = w.speedMeas;
    a_out.travelRad = w.haveTravel ? w.travelDir : (a ? a->GetAngleZ() : w.cmdFace);
    a_out.moving = w.phase == Phase::kWalk || w.phase == Phase::kTurn;
    a_out.waitsForRoute = w.phase == Phase::kWait;
    a_out.paused = w.phase == Phase::kPaused;
    a_out.answered = w.answered;
    const int n = static_cast<int>(w.pts.size());
    for (int k = w.seg + 1; k < n && k <= w.seg + 2; ++k) {     // (1.6.1 B3) the leg she walks, and the next one
        if (w.pts[k].flags & kFlagStair) a_out.onStairLeg = true;
    }
    return true;
}

bool PointAhead(std::int32_t a_walkId, float a_aheadU, RE::NiPoint3& a_out)
{
    const auto it = g_walkers.find(a_walkId);
    if (it == g_walkers.end() || it->second.pts.empty()) return false;
    const Walker& w = it->second;
    a_out = PointAt(w, (std::min)(w.total, w.s + (std::max)(0.f, a_aheadU)));
    return true;
}

bool Restart(std::int32_t a_walkId, std::vector<PathPoint> a_path, std::optional<float> a_facingRad, float a_remainAfterU)
{
    auto it = g_walkers.find(a_walkId);
    if (it == g_walkers.end() || it->second.phase != Phase::kPaused || a_path.empty()) return false;
    Walker& w = it->second;
    w.prm.remainAfterU = (std::max)(0.f, a_remainAfterU);
    w.pts = std::move(a_path);
    BuildTrack(w);
    w.blendOn = false;            // v1.4.1: the new route's own facing blend (and its log line) ...
    w.holdFinal = FaceAllNow(w, HasFinalNow(w));     // (1.6.0 D2: faceAll holds it over the whole new route)
    w.blendBadT = 0.f;            // ... (a blend that failed on this walk stays OFF: blendOff is kept)
    w.doorIdx = -1;               // 1.5.0: the new route's doors are its own (found again for it, below)
    w.doorRef = 0;
    w.doorActs = 0;
    w.doorCloseLogged = false;
    CollectDoors(a_walkId, w, RE::TESForm::LookupByID<RE::Actor>(w.who));
    w.reapproach = false;         // 1.5.0: a push during the re-approach is re-planned - the new route is a walk, not that
                                  // correction (its stalls re-plan, its doors are opened; `reapproached` keeps it to one)
    if (a_facingRad) {            // 1.5.0: a water walk faces the water as seen from the new route
        w.prm.hasFacing = true;
        w.prm.facingRad = *a_facingRad;
    }
    EnterPhase(w, Phase::kWalk, std::format("a new route: {} points, {:.0f} u{}", w.pts.size(), w.total,
                                            a_facingRad ? std::format(", final facing {:.0f} deg", Compass(*a_facingRad)) : std::string()).c_str());
    return true;
}

void Stop(std::int32_t a_walkId, const char* a_why)
{
    Finish(a_walkId, End::kNone, a_why ? a_why : "stopped", false);
}

bool Hold(std::int32_t a_walkId)
{
    auto it = g_walkers.find(a_walkId);
    if (it == g_walkers.end()) return false;
    Walker& w = it->second;
    w.prm.holdSeconds = -1.f;     // an arrival on the way stands (never lets go: the newer walk takes her)
    w.holdLeft = -1.f;            // a stand's own hold time no longer ends her
    w.answered = true;            // its answer is out (X 6): any end from here on is a hold end
    w.reapproached = true;        // v1.4.1: no re-approach toward the old point - she only waits for the newer walk
    w.prm.remainAfterU = 0.f;     // 1.6.0: no chunk comes for it any more
    if (w.phase == Phase::kTurn || w.phase == Phase::kWalk || w.phase == Phase::kDoor || w.phase == Phase::kWait) {
        auto* a = RE::TESForm::LookupByID<RE::Actor>(w.who);
        w.pauseYaw = a ? a->GetAngleZ() : w.cmdFace;
        EnterPhase(w, Phase::kPaused, "handed over to a newer walk: held at speed 0 until it takes her");
        w.cmdSpeed = 0.f;
        w.cmdFace = w.pauseYaw;
        WriteOrder(w, true);
    } else {
        logger::info("WALK #{} handed over to a newer walk: held ({}) until it takes her", a_walkId, PhaseName(w.phase));
    }
    return true;
}

bool EverDrove(std::int32_t a_walkId)
{
    const auto it = g_walkers.find(a_walkId);
    return it != g_walkers.end() && it->second.everDrove;
}

bool IsDirectWalking(RE::FormID a_who)
{
    if (!a_who || g_inUse.load(std::memory_order_acquire) == 0) return false;
    std::scoped_lock lk(g_slotMtx);
    const int i = SlotOf(a_who);
    return i >= 0 && (g_slot[i].wantDrive || g_slot[i].driving);
}

bool HookLive()
{
    return g_hookLive.load();
}

const std::string& NotLiveText()
{
    return g_notLiveText;
}

int LiveCount()
{
    return g_liveWalkers.load(std::memory_order_acquire);
}

void Install()
{
    const bool           vr = REL::Module::IsVR();
    const char*          runtime = vr ? "VR" : REL::Module::IsAE() ? "AE" : "SE";
    const std::string    version = REL::Module::get().version().string(".");
    const std::uintptr_t base = REL::Module::get().base();
    std::string          iniNote;
    const int            seae = ReadSeaeSwitch(iniNote);     // read ONCE, here

    // ── 1.5.0 THE READ-ONLY SELF-TEST (every runtime; it patches nothing) ──
    const SiteScan scan = FindMotionSite();
    std::string    callee;
    if (scan.site) {
        callee = ModuleOfAddress(scan.target);
        if (scan.engineOwn) {
            const std::uint64_t id = IdAtOffset(scan.target - base);
            callee += id ? std::format(" = id {}", id) : std::string(" (no address-library id at that address)");
        } else {
            callee += " (another mod hooked it first: a hook here chains to it)";
        }
    }
    logger::info("direct walk self-test ({} {}; Actor::UpdateMovement = id {} at +0x{:X}): {}", runtime, version, scan.fnId,
                 scan.fn ? scan.fn - base : 0,
                 scan.ready ? std::format("READY - the motion-check CALL at the pinned +0x{:X} (+0x{:X} in the exe) -> {}; the "
                                          "apply-movement CALL (the anchor) at +0x{:X} -> {}; pinned candidates: {}",
                                          scan.site - scan.fn, scan.site - base, callee, scan.apply - scan.fn,
                                          ModuleOfAddress(CallTargetAt(scan.apply)), scan.tried) :
                              std::format("NOT READY because {}", scan.why));
    if (scan.fn) LogBytes("Actor::UpdateMovement", scan.fn, kSelfTestDump);
    if (scan.site && scan.engineOwn) LogBytes("the callee", scan.target, kCalleeDump);

    if (!vr) {
        // ── SE / AE: the switch decides; the default patches nothing ──
        if (seae != 1) {
            g_notLiveText = std::format("SE/AE direct walk: OFF ({} - the self-test only{})", iniNote,
                                        scan.ready ? "" : std::format("; NOT READY because {}", scan.why));
            logger::info("SE/AE direct walk: {} - {}: NOTHING patched; a walk off the navmesh answers B 19. A tester sends this "
                         "Standpoint.log; SEAE = 1 in Data/SKSE/Plugins/Standpoint.ini installs the walk hook (only when READY).",
                         scan.ready ? std::string("READY") : std::format("NOT READY because {}", scan.why), iniNote);
            return;
        }
        if (!scan.ready) {
            g_notLiveText = std::format("SE/AE direct walk: NOT READY because {}", scan.why);
            logger::warn("SE/AE direct walk: NOT READY because {} - {} asks for the walk hook, but NOTHING is patched; a walk off "
                         "the navmesh answers B 19", scan.why, iniNote);
            return;
        }
#pragma warning(suppress: 4996)
        SKSE::AllocTrampoline(kTrampolineBytes);
        auto& tr = SKSE::GetTrampoline();
        g_nextMotion = reinterpret_cast<MotionCheckFn>(tr.write_call<5>(scan.site, reinterpret_cast<std::uintptr_t>(&OnMotionCheck)));
        if (!g_nextMotion) {
            g_notLiveText = "SE/AE direct walk: the found CALL had no target - not installed";
            logger::error("SE/AE direct walk: the found motion-check CALL had no target - NOT installed; walks answer B 19");
            return;
        }
        g_hookLive.store(true);
        logger::info("SE/AE direct walk: READY + {}: the motion-check hook INSTALLED at +0x{:X} in the exe (Actor::UpdateMovement id "
                     "{} + 0x{:X}); the previous target is {}. The firm-stop arbiter hooks are VR only: she stops with her own "
                     "deceleration; no strafing facing blend (read on VR only) - she turns at the point.",
                     iniNote, scan.site - base, scan.fnId, scan.site - scan.fn,
                     ModuleOfAddress(reinterpret_cast<std::uintptr_t>(g_nextMotion)));
        return;
    }

    // ── VR: the proven site (+0xE5) - the self-test must agree with it by itself ──
    logger::info("direct walk: {} - the switch is for SE / AE only: ignored on Skyrim VR (the walk hook is always installed here)",
                 iniNote);
    const std::uintptr_t motionSite = REL::RelocationID(kMotionCheckFnId, kMotionCheckFnIdAE).address() + kMotionCallOffset;
    if (scan.ready && scan.site == motionSite) {
        logger::info("direct walk self-test on VR: the pinned +0x{:X} passes the same check SE / AE get - the proven site",
                     kMotionCallOffset);
    } else {
        logger::error("direct walk self-test MISMATCH on VR: the check {} - the proven +0x{:X} is kept. The check is wrong here: "
                      "please send this Standpoint.log",
                      scan.ready ? std::format("passed +0x{:X}", scan.site - scan.fn) : std::format("is NOT READY because {}", scan.why),
                      kMotionCallOffset);
    }
    g_notLiveText = "the motion-check hook is not installed (see the load lines)";
    const std::uintptr_t speedsSite = REL::Offset(kArbiterSpeedsVR).address();
    const std::uintptr_t turnsSite  = REL::Offset(kArbiterTurnsVR).address();
    const std::uint8_t   bm = *reinterpret_cast<const std::uint8_t*>(motionSite);
    const std::uint8_t   bs = *reinterpret_cast<const std::uint8_t*>(speedsSite);
    const std::uint8_t   bt = *reinterpret_cast<const std::uint8_t*>(turnsSite);
    if (bm != 0xE8) {
        logger::error("direct walk: the motion-check site {:X} holds 0x{:02X}, not a CALL (E8) - NOT installed; walks answer B 19",
                      motionSite - REL::Module::get().base(), bm);
        return;
    }
#pragma warning(suppress: 4996)
    SKSE::AllocTrampoline(kTrampolineBytes);
    auto& tr = SKSE::GetTrampoline();
    g_nextMotion = reinterpret_cast<MotionCheckFn>(tr.write_call<5>(motionSite, reinterpret_cast<std::uintptr_t>(&OnMotionCheck)));
    if (!g_nextMotion) {
        logger::error("direct walk: the motion-check CALL had no target - NOT installed");
        return;
    }
    g_hookLive.store(true);
    logger::info("direct walk: motion-check hook INSTALLED at SkyrimVR+0x{:X} (id {} + 0x{:X}); the previous target is {} "
                 "(installed at SKSEPlugin_Load: VR Chain / PPB chain after us and stay upstream of Standpoint)",
                 motionSite - REL::Module::get().base(), kMotionCheckFnId, kMotionCallOffset,
                 ModuleOfAddress(reinterpret_cast<std::uintptr_t>(g_nextMotion)));
    if (bs == 0xE8 && bt == 0xE8) {
        g_nextSpeeds = reinterpret_cast<SpeedsFn>(tr.write_call<5>(speedsSite, reinterpret_cast<std::uintptr_t>(&OnArbiterSpeeds)));
        g_nextTurns  = reinterpret_cast<TurnsFn>(tr.write_call<5>(turnsSite, reinterpret_cast<std::uintptr_t>(&OnArbiterTurns)));
        g_paramsLive.store(g_nextSpeeds && g_nextTurns);
        logger::info("direct walk: arbiter parameter hooks INSTALLED (speeds +0x{:X} -> previous {}; turns +0x{:X} -> previous {}) - "
                     "only actors Standpoint drives right now get accel 10 / decel 10 / turn 10 / rotation 2.5",
                     kArbiterSpeedsVR, ModuleOfAddress(reinterpret_cast<std::uintptr_t>(g_nextSpeeds)), kArbiterTurnsVR,
                     ModuleOfAddress(reinterpret_cast<std::uintptr_t>(g_nextTurns)));
    } else {
        logger::warn("direct walk: arbiter sites hold 0x{:02X} / 0x{:02X}, not two CALLs - not hooked; a driven walker keeps her "
                     "own movement parameters", bs, bt);
    }
}

void Tick(float a_dt)
{
    if (g_walkers.empty()) return;
    if (!g_threadsLogged) {
        if (const auto ht = g_hookThread.load(std::memory_order_relaxed)) {
            g_threadsLogged = true;
            logger::info("direct walk: the motion-check hook runs on thread {} - the main thread (Tick) is {}{}", ht,
                         ::GetCurrentThreadId(), ht == ::GetCurrentThreadId() ? " (the SAME thread)" : " (another thread)");
        }
    }
    std::vector<std::int32_t> ids;
    ids.reserve(g_walkers.size());
    for (const auto& [id, w] : g_walkers) ids.push_back(id);
    for (const auto id : ids) StepWalker(id, a_dt);
}

void OnSaveGame()
{
    struct Held { RE::FormID who; int slot; bool continues; };
    std::vector<Held> held;
    {
        std::scoped_lock lk(g_slotMtx);
        for (int i = 0; i < kSlots; ++i) {
            if (g_slot[i].who && (g_slot[i].owned || g_slot[i].driving)) held.push_back({ g_slot[i].who, i, g_slot[i].wantDrive });
        }
    }
    for (const auto& h : held) {
        auto*                a = RE::TESForm::LookupByID<RE::Actor>(h.who);
        const std::uintptr_t mc = a ? ControllerOf(a) : 0;
        const bool           flag = mc && PlannerFlag(mc);
        if (flag) Steer(mc)->ReleasePlanner();
        {
            std::scoped_lock lk(g_slotMtx);
            Slot& s = g_slot[h.slot];
            if (s.who == h.who) {
                s.owned = false;
                s.driving = false;
                ++s.handBacks;
                g_drivenState[h.slot].store(0, std::memory_order_release);
                if (!s.wantDrive) FreeSlotLocked(h.slot);
            }
        }
        // (review H #4) her package is re-evaluated only when no walk continues: for a walk / stand that goes on, a queued
        // evaluate running after the re-take could only cost a "lost"
        if (!h.continues) QueueEvaluate(h.who);
        logger::info("direct walk: SAVE - {} handed back before the save is written (flag {}); {}", ActorText(a),
                     flag ? "cleared" : "was not set",
                     h.continues ? "her walk / stand goes on: re-taken next frame" : "her package is re-evaluated");
    }
}

void OnPreLoad()
{
    {
        std::scoped_lock lk(g_slotMtx);
        for (int i = 0; i < kSlots; ++i) {
            g_slot[i] = Slot{};
            g_drivenState[i].store(0, std::memory_order_release);
            g_slotWho[i].store(0, std::memory_order_release);
        }
        g_inUse.store(0, std::memory_order_release);
    }
    if (!g_walkers.empty()) logger::info("direct walk: a game is being loaded - {} walker(s) forgotten, nothing touched", g_walkers.size());
    g_walkers.clear();
    SetLive();
}

}  // namespace standpoint_direct
