#pragma once
// ─────────────────────────────────────────────────────────────────────────────
//  DirectWalk.h — v1.4.0: the "chain-pull" walk driver (the user, 2026-11-02: "we can use HIGGS/PLANCK way to travel,
//  exactly like if someone was pulling a chain on that NPC's hips").
//
//  The NPC's own movement controller is switched to PLANNER DIRECT CONTROL and steered every frame with three targets
//  (a travel direction, a NORMALISED speed, a facing angle). The engine then plays her own walk animation and moves her
//  with her own character controller: real steps, wherever her body can go (bare ground, shallow water). No slide, no
//  teleport: nothing here sets a position.
//
//  Two halves (report "1.4.0 Water Walk" A §6.1):
//    * Tick (main thread, once a frame, from WalkApi's pump): every world read and every decision - the route, the gate,
//      the watchdogs, the swim watch. It writes ONE small order per walker into a locked table.
//    * the hook (VR: the CALL inside the per-actor motion check, SE id 36359 + 0xE5): executes the order for our walkers
//      only - the enter ritual, the three targets, the hand-back - and records what it found. No 3D, no logging, no
//      Papyrus, no allocation in there. Every other actor goes straight on down the chain.
//  VR ONLY in 1.4.0 (the SE / AE call sites are not known on this PC): on SE / AE nothing is installed and Begin refuses.
//  v1.4.1 ARRIVAL PRECISION (the user's first VR run, 2026-11-03: the turn in place at the point drifted her ~10 u): a walker
//  who CAN STRAFE turns to her final facing while still walking the last 48 u (rate-limited), so she arrives already turned;
//  still more than 16 u from the point after the facing = ONE short re-approach (facing kept), then the facing again.
//  v1.5.0 (report "1.5.0 Plan" D §3):
//    * DOORS: every door whose doorway her route crosses (the planner's kPtDoor points AND doors that were open when the route
//      was planned) is watched in route order; at the edge of its leaf's sweep a door that is not open stops her, she opens it
//      herself (only when standpoint_path::IsOpenableDoor says so: ActivateRef, she is the activator - the engine's own way an
//      NPC opens a door), waits up to 2 s for it to open (a detection window), then walks on; a door that fails the rule or
//      stays shut = BLOCKED (paused, the doorway is the bad spot: re-plan).
//    * PUSHED off the line (strayed) = she is PAUSED and reported like a stall (WalkApi re-plans from where she stands).
//    * SE / AE: the motion-check CALL is FOUND AT RUN TIME (the last call before Actor::UpdateMovement's "apply movement"
//      call, SE 36359 + 0xF0 / AE 37350 + 0xFB) by a read-only self-test that logs READY / NOT READY + the bytes it read.
//      Data/SKSE/Plugins/Standpoint.ini [DirectWalk] SEAE = 1 installs it (only when READY); 0 (the default) patches nothing.
//      VR keeps the proven site (+0xE5) and only checks that the finder picks it by itself (a mismatch is logged loudly).
//  v1.5.1 (report "1.5.1 Plan" D §1, §2, §5):
//    * THE IDLE BREAK: "her animation owns her feet" (a random idle while she stood) -> IdleForceDefaultState to her graph ONCE
//      per walk (the event Standpoint's own let-go sends), then she is taken; the walk ends (animation-lock) only when the
//      animation still owns her feet 2 s AFTER the break. Only while she still has to walk to the point: never in the hold after
//      the answer (her animation is the consumer's: the hold ends after 2 s as in 1.5.0), never during a stagger, at a moving
//      take-over only after 2 s; the gate refuses furniture first.
//    * SPEED BY DISTANCE (the user's rule): Params::jog = she JOGS (normalised 1.5) while more than 700 u of route are left and
//      walks the last 700 u; walk-only walks (she started within 20 m) as 1.5.0. Ramped (never a jump); she is at her walk
//      before a SHARP corner, a door of her route and a sharp turn. A take-over from the far phase starts at her pace.
//    * B 22 names what holds her when it can tell (PLANCK's grab, VR Chain, PPB's push walk).
//  v1.6.0 (report "1.6.0 Plan" D = the spec §1 item 7, S2 §3, S1 §7 D1-D3):
//    * THE SPEED BANDS by route LEFT (toGo = the route left + Params::remainAfterU, the unverified rest): run (told 2.0) beyond
//      1,050 u, jog (1.5) down to 210 u, walk (1.0) the last 210 u; a band that would last under 1 s is skipped; ramps 1.5 / s,
//      the downshifts ANTICIPATED (she IS at jog at 1,050 u and at walk at 210 u). The look-ahead grows with her speed (0.35 s,
//      48-128 u). Turn caps: a turn over 30 deg ahead is jogged, over 60 deg (and a SHARP corner, a door) walked; slower too
//      for water, legs steeper than 27 deg, stairs (walked), drop edges, kPtNoCut corners and the end of the verified route.
//      The trace gets her yaw rate.
//    * Extend (the next verified chunk appended while she walks), Swap (a new route for a walker on her way, no pause),
//      GetProgress / PointAhead (where she is on her route), Params::faceOffsetRad (D1) and Params::faceAll (D2).
//
//  The engine facts used (addresses, slot order, call order, units, measured numbers) are listed with their sources in
//  DirectWalk.cpp. The code, names and structure are Standpoint's own.
// ─────────────────────────────────────────────────────────────────────────────

namespace standpoint_direct {

    // Why a drive ended (the driver's words; WalkApi turns them into letters and detail codes).
    enum class End : std::uint8_t {
        kNone = 0,
        kArrived,        // on the point, final facing done
        kDepthReached,   // her submerged level reached the asked stop level
        kBlocked,        // no progress (the progress watchdog), or (1.5.0) a door that did not open - she is HELD (paused)
                         // for a re-plan; see Report::paused
        kStrayed,        // pushed off the line (cross-track too far) - 1.5.0: she is HELD (paused) for a re-plan, as kBlocked
        kWouldSwim,      // submerged >= 0.70, or the engine says she swims
        kGrabbed,        // HIGGS holds her (PLANCK leads her then)
        kOtherDriver,    // another plugin drives her (or VR Chain chained her) - before we ever drove her
        kTaken,          // another plugin took her while we drove her (starved / busy / chained)
        kAnimLock,       // her animation owned her feet for too long (1.5.1: still 2 s after the idle break)
        kLostTooOften,   // lost control again and again with no reason found
        kNoTake,         // the enter ritual never held (she never became planner direct)
        kCombat, kRagdoll, kKnocked, kKillMove, kFurniture, kBleedout, kMounted, kRestrained, kDead, kGone, kWarp,
        kHoldOver,       // (hold end only) the hold time ran out
        kNotSupported,   // not VR, or the hook is not installed
        kTableFull,      // every driver slot is in use
        kBadPath,        // an empty route
    };
    const char* EndName(End a_end);

    // One point of the route she walks (WalkApi converts the planner's points; point 0 = where she stood at Begin).
    struct PathPoint {
        RE::NiPoint3  pos;
        std::uint32_t flags{ 0 };   // the planner's flags: bit1 2 = SHARP corner (reach it, stop, turn), bit2 4 = edge cell
        std::uint8_t  kind{ 0 };    // the planner's kind: 1 nav, 2 floor, 3 water (log only)
        float         depth{ 0.f }; // water depth at the point, game units (log only)
        RE::FormID    door{ 0 };    // 1.5.0: the leg INTO this point goes through this door (the planner's kPtDoor); 0 = none
    };

    struct Params {
        std::int32_t walkId{ 0 };          // WalkApi's handle: every log line says WALK #walkId
        bool         hasFacing{ false };   // false = keep the last travel direction
        float        facingRad{ 0.f };     // final heading (Skyrim: 0 = north, clockwise)
        float        holdSeconds{ -1.f };  // -1 stand (driven, speed 0) until Stop; 0 let go at arrival; N seconds
        float        stopSubmerged{ 0.f }; // > 0: arrive as soon as her submerged level reaches it
        RE::FormID   faceWho{ 0 };         // v1.4.1: her final facing = toward this reference, read live at the arrival; while
                                           // she stands she keeps facing it. 0 = facingRad. (1.6.0: + faceOffsetRad)
        float        d0{ 0.f };            // 1.5.1 (log only): her straight distance to the point when the walk was asked, u
        float        startSpeedUps{ 0.f }; // 1.5.1: she is taken over MOVING at this speed (u/s); the order starts at her pace
                                           // (normalised, VR) and is ramped from there. 0 = from a stand (1.6.0: unused by WalkApi)
        // 1.6.0 THE SPEED BANDS (spec D §1 item 7, S2 §3) by route LEFT = the route left + remainAfterU: run (told 2.0) beyond
        // 1,050 u, jog (1.5) down to 210 u, walk (1.0) the last 210 u; a band that would last under 1 s is skipped; the slow
        // zones and turn caps cap the band ahead of a turn / door / water / steep leg / stair / drop edge / kPtNoCut corner / the
        // end of the verified route. false = she walks the whole way (1.0, as 1.5.0's walk-only walks).
        bool         bands{ true };
        float        remainAfterU{ 0.f };  // the UNVERIFIED route still after this route's last point (the skeleton's rest), u.
                                           // > 0: the last point is NOT the goal - it caps her only by her stopping distance (she
                                           // walks its last ~280 u if the next chunk is late) and she waits there (speed 0) for
                                           // Extend; 0 = the last point is the goal (an arrival)
        float        faceOffsetRad{ 0.f }; // 1.6.0 D1: added to the live bearing toward faceWho (the arrival, the stand re-face)
        bool         faceAll{ false };     // 1.6.0 D2: hold the final facing (faceWho + offset, else facingRad) over the WHOLE
                                           // route when she can strafe; the blend watch turns it off if she does not strafe
        std::int32_t steerId{ 0 };         // 1.6.0 (log only): the steered session's id (0 = none) - the DRIVE start line names it
    };

    struct Report {
        End          why{ End::kNone };
        bool         paused{ false };     // kBlocked: she stands HELD where she stopped; WalkApi calls Restart or Stop
        bool         holding{ false };    // an arrival with a hold: she now stands, driven, until the hold ends (a 2nd report)
        bool         holdEnded{ false };  // the 2nd report: the hold ended (why = kHoldOver, or what ended it)
        bool         everDrove{ false };  // the hook took her at least once during this walk
        RE::NiPoint3 pos{};               // where she is
        float        submerged{ 0.f };    // her submerged level now (0..1)
        float        facingErrDeg{ 0.f }; // arrival: her heading vs the final heading
        RE::NiPoint3 stallSpot{};         // kBlocked: the planner's bad spot - on the route ahead, her radius + 24 + 8 u away
                                          // (1.5.0: a door that did not open = the doorway)
        float        stallRadius{ 0.f };  // kBlocked: the disc the planner will refuse round it (her radius + 24 u), for the log
        RE::FormID   door{ 0 };           // 1.5.0 kBlocked: the door that did not open (0 = a stall)
        bool         markBad{ true };     // 1.6.1 (B2) kBlocked: false = do NOT make stallSpot a bad spot (an openable door's first
                                          // failure in this walk - the doorway may be the only way); the re-plan may try it again
    };
    // Called on the main thread from inside Tick / Begin (never from the hook).
    using ReportFn = void (*)(std::int32_t a_walkId, const Report& a_report);

    // Main thread. Take her and walk her along a_path. false = refused: a_why names the End and a_text the reason
    // (no report is sent for a refusal).
    bool Begin(RE::Actor* a_actor, std::vector<PathPoint> a_path, const Params& a_params, ReportFn a_report, End& a_why,
               std::string& a_text);

    // Main thread. A paused walker (kBlocked / kStrayed) gets a new route from where she stands. false = no such paused
    // walker. a_facingRad (1.5.0, a water walk's "face the water" from the new route): her final heading from now on.
    // (1.6.0: + a_remainAfterU, as Extend)
    bool Restart(std::int32_t a_walkId, std::vector<PathPoint> a_path, std::optional<float> a_facingRad = std::nullopt,
                 float a_remainAfterU = 0.f);

    // 1.6.0 (S2 §2.3, §7): the next VERIFIED chunk appended to the route she walks - she never pauses for it. a_pts continue
    // from the route's last point (a first point within 1 u of it is dropped); the new legs' doors are collected (as Restart).
    // a_remainAfterU = the unverified route still after the new last point (0 = the new last point is the goal). An EMPTY a_pts
    // with a_remainAfterU 0 = "no more chunks": the route as it is ends the walk (she arrives at its last point - the loaded
    // edge, a chunk that failed: WalkApi answers C 9). An EMPTY a_pts with a_remainAfterU > 0 = "more route comes" (a re-target
    // is planned): she never arrives at this end - she waits there if it comes late. a_facingRad: her final heading from now on
    // (a water walk's last piece: "face the water"). false = no such walker, or it has answered (she stands). Main thread.
    bool Extend(std::int32_t a_walkId, const std::vector<PathPoint>& a_pts, float a_remainAfterU,
                std::optional<float> a_facingRad = std::nullopt);

    // 1.6.0 (WalkApi's Retarget, S1 W2): her final facing from now on - a_hasFacing / a_facingRad, toward a_faceWho read live
    // + a_faceOffsetRad (D1), held over the whole route when a_faceAll and she can strafe (D2). false = no such walker. Main thread.
    bool SetFacing(std::int32_t a_walkId, bool a_hasFacing, float a_facingRad, RE::FormID a_faceWho, float a_faceOffsetRad, bool a_faceAll);

    // 1.6.0 (WalkApi's re-target, S2 §2.4 "else today's hold"): she stops and is HELD where she is (speed 0, her facing kept),
    // the walk goes on with no report - Swap / Restart give her a route again (a_why: one word for the log). false = no such
    // walker, or it has answered. Main thread.
    bool Pause(std::int32_t a_walkId, const char* a_why);

    // 1.6.0 D3 (S1 §7, S2 §2.4): a NEW route for a walker who is on her way - NO pause (Restart is for a paused walker).
    // a_path.front() = where the new route's plan started: a point on her current route AHEAD of her (PointAhead), or where
    // she stood. The driver joins it where she is: [her position, her current route's points up to a_path.front() while it is
    // still ahead of her, a_path...]; a start she has already passed = joined at the new route's leg nearest to her (on her
    // floor, the far hand-over's rule). Her speed, band and travel go on; any phase before the arrival report (a turn, a door
    // wait, a settle, the facing, a pause) becomes the walk; the new route's doors are collected; one re-approach is allowed
    // again. a_facingRad: her final heading from now on (nullopt = unchanged); a_remainAfterU as Extend. false = no such
    // walker, or it has answered (she stands: WalkApi starts a new walk then). Main thread.
    bool Swap(std::int32_t a_walkId, std::vector<PathPoint> a_path, float a_remainAfterU,
              std::optional<float> a_facingRad = std::nullopt);
    // 1.6.0 (review R2 #2): where Swap would join a_path to her route RIGHT NOW (call it in the same frame, before Swap): a_from =
    // her position, a_to = the first point she would walk to, a_off = how far she is from the new route's leg she joins (0 = the
    // plan's start is still ahead of her on her current route: no new line). a_off > 48 u = the line a_from -> a_to is UNTESTED -
    // WalkApi probes it before Swap. false = no such walker, or it has answered. Main thread.
    bool SwapJoin(std::int32_t a_walkId, const std::vector<PathPoint>& a_path, RE::NiPoint3& a_from, RE::NiPoint3& a_to, float& a_off);

    // 1.6.0 (WalkApi's rolling plan and re-target): where she is on her route right now. Main thread.
    struct Progress {
        RE::NiPoint3 pos{};                 // where she is
        float        s{ 0.f };              // her arc position along the route (2D u)
        float        total{ 0.f };          // the route's length (2D u) - the verified route she has
        float        remainAfterU{ 0.f };   // the unverified route after it (Params / the last Extend / Swap)
        float        speedUps{ 0.f };       // her measured speed, u/s
        float        travelRad{ 0.f };      // the direction she is steered (her yaw before her first step)
        bool         moving{ false };       // the walk phase (told to move, or about to)
        bool         waitsForRoute{ false };// she stands at the end of the verified route: the next chunk is late
        bool         paused{ false };       // held for a re-plan (kBlocked / kStrayed reported)
        bool         answered{ false };     // the arrival report is out: she stands (the hold)
        bool         onStairLeg{ false };   // 1.6.1 (B3): the leg she walks (or the next one) uses the stair allowance (kPtStair) -
                                            // the route line's height there is no floor: a re-aim plans from her feet
    };
    bool GetProgress(std::int32_t a_walkId, Progress& a_out);   // false = no such walker
    // the point a_aheadU (2D u) further along her route than where she is now - its last point at most. false = no such walker.
    bool PointAhead(std::int32_t a_walkId, float a_aheadU, RE::NiPoint3& a_out);

    // Main thread. Hand her back now and forget the walk (no report). a_why is ONE word ("released", "replaced",
    // "limit", ...): the log and the mod event "stop|<a_why>" carry it.
    void Stop(std::int32_t a_walkId, const char* a_why);

    // Main thread. A newer walk of the same NPC will take her (review H #8): this walker keeps her HELD at speed 0 (a walk in
    // progress is paused; an arrival or a stand never lets her go by itself) until the newer walk's Begin drops it, or Stop.
    // Its reports from here on are hold ends. false = no such walker.
    bool Hold(std::int32_t a_walkId);

    // Main thread. Would Begin take her right now? (the start checks only; a refusal fills a_why / a_text)
    bool CanTake(RE::Actor* a_actor, End& a_why, std::string& a_text);

    // 1.5.1 (the far phase, WalkApi): the driver's gate on its own - what makes walking her wrong right now (no 3D, dead,
    // bleeding out, unconscious, restrained, kill move, ragdoll, knocked, sitting / sleeping, mounted, combat). kNone = none.
    // Main thread.
    End Gate(RE::Actor* a_actor, std::string& a_text);

    // Main thread. 1.5.1 (verify V NEW-2): has the hook taken her in this walk at least once? false = no such walker / never.
    bool EverDrove(std::int32_t a_walkId);

    bool IsDirectWalking(RE::FormID a_who);   // any thread: we drive her, or want to (also during a stand)
    bool HookLive();                          // any thread: the motion-check hook is installed (VR; 1.5.0: SE / AE with SEAE = 1)
    int  LiveCount();                         // any thread: walkers Tick still has to look after
    // any thread (set once at load): why HookLive() is false - "SE/AE direct walk: OFF ..." / "... NOT READY because ...";
    // the walks' B 19 text
    const std::string& NotLiveText();

    // SKSEPlugin_Load: the read-only self-test of the motion-check site on every runtime (logged), then VR: the proven hooks;
    // SE / AE: Standpoint.ini [DirectWalk] SEAE (read once here) = 1 installs the found site when the self-test is READY
    void Install();
    void Tick(float a_dt);   // main thread, once a frame while LiveCount() > 0 (WalkApi's pump); a_dt seconds, pause excluded
    void OnSaveGame();       // kSaveGame: hand every driven walker back before the save is written; re-taken next frame
    void OnPreLoad();        // kPreLoadGame / kNewGame: forget everything, touch nothing (her 3D is going away)

}  // namespace standpoint_direct
