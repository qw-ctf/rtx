// SPDX-License-Identifier: AGPL-3.0-or-later

//! The rtx tunable registry: every `rtx_*` cvar and its first-run default, declared as one table so
//! the whole tunable set reads (and registers) as data. Seeded in [`GameState::init`](crate::game)
//! via `cvar_default`, which only writes a cvar that's unset — so a value from `server.cfg` (or a
//! `set` before `map`) survives each `GAME_INIT`.

use crate::host::CvarValue;

/// A default value for one rtx tunable — bundles the three [`CvarValue`] kinds so the whole tunable
/// set can register from one table ([`RTX_CVAR_DEFAULTS`]). Its token is identical to calling
/// `cvar_default` with the underlying `bool`/`f32`/`&str` directly.
#[derive(Clone, Copy)]
pub(crate) enum CvarSeed {
    Bool(bool),
    Float(f32),
    Str(&'static str),
}

impl CvarValue for CvarSeed {
    fn cvar_token(&self) -> String {
        match self {
            CvarSeed::Bool(b) => b.cvar_token(),
            CvarSeed::Float(f) => f.cvar_token(),
            CvarSeed::Str(s) => s.cvar_token(),
        }
    }
}

/// The rtx-specific movement and combat features — the ones the *server* applies (in
/// `PlayerPreThink`, or in the shootable-grenade combat path), so they exist only on an rtx server.
///
/// This is the set a network client must not assume. A double jump is the sharp one: the navmesh
/// plans routes across gaps that only cross with the mid-air second jump (`nav_build.rs`, the `djump`
/// links), and on a KTX or vanilla server that jump is never granted — the bot would commit to the
/// leap and fall in. So the server **advertises** these in serverinfo (like KTX's `pm_*` keys) and a
/// client mirrors them; a client on any other server forces them off. Grapple isn't here — the client
/// forces it off unconditionally, because the hook's *state* isn't on the wire to mirror at all.
pub(crate) const RTX_MOVE_CVARS: &[&str] = &[
    "rtx_doublejump",
    "rtx_walljump",
    "rtx_elevator_jump",
    "rtx_shootable_grenades",
];

/// The rtx tunables and their first-run defaults, registered in [`GameState::init`](crate::game).
/// `cvar_default` only seeds a cvar that's unset, so a value from `server.cfg` (or a `set` before
/// `map`) survives each `GAME_INIT`. Declared as data so the tunables read as one registry.
pub(crate) const RTX_CVAR_DEFAULTS: &[(&str, CvarSeed)] = {
    use CvarSeed::{Bool, Float, Str};
    &[
        // Mid-air double jump, on by default (set `rtx_doublejump 0` to disable).
        ("rtx_doublejump", Bool(false)),
        // Bots bunnyhop (air-strafe to build speed) on open stretches; on by default.
        ("rtx_bot_bhop", Bool(true)),
        // Generate curl jumps (run-up down a corridor, air-turn onto an offset platform), certified by
        // a pmove rollout in the navmesh build. A sub-toggle of bhop (`rtx_bot_bhop 0` disables it too).
        ("rtx_bot_curljump", Bool(true)),
        // Bots ground-zigzag (circle-strafe) on straight corridors too short to hop; on by default.
        // A sub-toggle of the bhop controller — `rtx_bot_bhop 0` disables it regardless.
        ("rtx_bot_zigzag", Bool(true)),
        // Bots plan over speed bands (kinodynamic A*), crediting speed carried between legs so
        // chained speed jumps and hot corridors route; on by default. Escape hatch: 0 → plain A*.
        ("rtx_bot_bandplan", Bool(true)),
        // Chain-entry gate: when a fresh repath's first leg from the bot's current cell is a
        // *chained* speed jump (no runway of its own — see `navmesh::SpeedJumpTraversal::chained`)
        // and the bot's actual speed is under half the link's `v_req`, exclude that link from just
        // this one search and replan. Measured on dm3 (two sub-`MAX_SPEED` chained links, v_req
        // 296/304): the banded planner's start band floors at `MAX_SPEED` regardless of a bot's real
        // speed, so it read those links as already flyable from a dead stop — 0% success stationary,
        // 80-100% with any carried speed, and every failed attempt fired the stall watchdog into a
        // repath that chose the identical link again. On by default; `0` restores that behavior.
        ("rtx_bot_chain_entry_gate", Bool(true)),
        // Require the curl too-slow abort to run on grounded frames only. On by default; `0` restores
        // the legacy speed-only abort on airborne frames too.
        ("rtx_bot_sj_abort_grounded", Bool(true)),
        // Apply the map-pinned navmesh patches (`nav_patch::PATCHES`) when a build finishes: hand-
        // verified cells + drops for standable surfaces the column carve cannot see (dm3's west
        // shelf being the worked example — bots climb onto it in pairs and wedge for minutes).
        // Fail-closed per patch with an `applied`/`skipped`/`failed` console line. 0 → build only.
        ("rtx_nav_patch", Bool(true)),
        // Fan a goal pick's independent navmesh floods out across a persistent worker pool (see
        // `bot::par`) — on by default; the result is bit-identical to serial. 0 → run them inline on
        // the main thread (the live A/B switch, and the fallback if the pool can't be built).
        ("rtx_bot_par", Bool(true)),
        // Navigate over the coarse LOD hierarchy (clusters + portals — see `navmesh::lod`) instead of
        // whole-graph floods: goal scoring and long-range steering read a near-exact/bounded-overestimate
        // coarse cost, and the steer search is bounded to the corridor window. On by default (validated:
        // stronghold/rocka/ultrav 6 bots stay under the frame budget, behaviour parity incl. the ultrav
        // gated quad). Escape hatch: 0 → the exact whole-graph floods + unwindowed steer.
        ("rtx_bot_lod", Bool(true)),
        // Steer the last metre off a fine (8u) near-field clearance grid built around each grounded bot
        // (see `nearfield`): the wish is nudged off nearby walls and drop-edges and centres through
        // doorways, instead of homing on the raw 32u cell centre. Off the routing path — only the
        // immediate wish changes. On by default; 0 → today's `hazard::edge_bias` drop-only probe.
        ("rtx_bot_nearfield", Bool(true)),
        // Sub-toggle of `rtx_bot_nearfield`: when the near-field certifies a straight look-ahead chord
        // is clear, glide toward it instead of the next 32u cell centre (smooths the residual grid
        // zigzag on plain walk legs). On by default; inert when `rtx_bot_nearfield` is 0.
        ("rtx_bot_glide", Bool(true)),
        // Predictive hop planning on ledge corridors (a spiral staircase's inner edge): roll the pmove
        // a hop ahead and take only hops whose predicted landing stays on the route, so the bot bhops a
        // curved walkway at speed instead of edge-braking or carrying off it. Gated to ledge-flagged
        // cells, so open play is untouched. See `bot::hopsim`.
        ("rtx_bot_hopplan", Bool(true)),
        // Certified walk tracking on grounded walk/step legs: roll the pmove forward under the
        // waypoint-pursuit policy the steerer runs and let only a line it *proves* stays on the floor
        // own the wish. While one is certified the reactive edge margin and both ledge brakes stand
        // down — they become the fallback for "no trackable line exists" rather than the first
        // responder, which is what made a bot brake mid-stride on a stair diagonal. See `bot::walksim`.
        ("rtx_bot_walkplan", Bool(true)),
        // A bot's health weights how willing it is to shortcut through lava/slime: hurt bots detour,
        // healthy (or armored, or biosuited) ones clip the corner. `0` prices every bot as a bare
        // spawn — hazards still cost, but the same to everyone. See `bot::bot_hazard_strength`.
        ("rtx_bot_hazard_health", Bool(true)),
        // Seconds of detour a bot accepts per unit of "fraction of surviving strength" a hazard eats
        // (rtx-nav's `HAZARD_TIME_K`). Higher = more timid. The default prices a bare spawn's
        // waterlevel-1 lava cell at ~1.7s.
        ("rtx_bot_hazard_k", Float(15.0)),
        // Wall jump (kick off a wall you jump into), on by default (`rtx_walljump 0` to disable).
        ("rtx_walljump", Bool(false)),
        // Elevator jump: a rising lift boosts your jump by `lift_speed * rtx_elevator_jump`. A
        // multiplier (0 disables, 1 = add the lift's true speed, 2 = double it, …).
        ("rtx_elevator_jump", Float(0.0)),
        // Shoot live grenades to detonate them early, on by default (`rtx_shootable_grenades 0`
        // to restore classic non-shootable grenades).
        ("rtx_shootable_grenades", Bool(false)),
        // Grappling hook (purectf port), on by default — every player spawns with it (impulse 22
        // to select). `rtx_grapple 0` to disable.
        ("rtx_grapple", Bool(false)),
        // Hook throw / reel-in speed multipliers (purectf's `localinfo hookspeed`/`hookpull`), each
        // scaling its base `× sv_maxspeed`. Defaults match purectf's shipped server.cfg.
        ("rtx_hook_speed", Float(1.25)),
        ("rtx_hook_pull", Float(1.0)),
        // Enabled weapons: a space-separated list of weapon tokens the server runs with —
        // `axe hook sg ssg ng sng gl rl lg` (the full roster, the default = no change). A weapon
        // whose token is absent is removed everywhere: its map pickup (`weapon_*`) is dropped at map
        // load and it's stripped from every spawn kit (so it can never be picked up or fired).
        // Unknown tokens are ignored. `hook` composes with `rtx_grapple` (both must allow it).
        ("rtx_weapons", Str("axe sg ssg ng sng gl rl lg")),
        // Game mode (ruleset): `dm` (deathmatch, the default), `ra` (Rocket Arena), `midair`, or
        // `ctf`. Read live each frame. A string cvar. See `crate::mode`.
        ("rtx_mode", Str("dm")),
        // Race (`rtx_mode race`): which of the map's routes is being run (0-based index into the
        // loaded routes, clamped; see `crate::race`). Read live — changing it mid-map moves
        // everyone to the new route's start.
        ("rtx_race_route", Float(0.0)),
        // Offline racing-line optimizer (race mode): iterations *in thousands* to spend TAS'ing each
        // route's line on a worker thread at map load. `0` (default) = off — bots bhop the plain
        // navmesh route. See `crate::raceline`.
        ("rtx_race_optimize", Float(0.0)),
        // Race bots track the offline-optimized line (when one exists); on by default, but inert
        // unless `rtx_race_optimize` produced a line. `0` = always follow the plain navmesh route.
        ("rtx_race_line", Bool(true)),
        // Match composition (organization), orthogonal to the mode: `""` (auto — the mode's natural
        // default), `ffa` (open free-for-all), or a team format `1on1`/`duel`/`2on2`/`2on2on2`/…
        // (a locked N×M match). `ra` ignores this (its 1v1 round queue is fixed). See `crate::mode`.
        ("rtx_match", Str("")),
        // Rocket Arena: seconds of spawn-protected countdown before "FIGHT". (Always a 1v1 duel.)
        ("rtx_ra_countdown", Float(3.0)),
        // Team match (`rtx_match 1on1`/`2on2`/…): seconds of spawn-protected countdown after the
        // match-start map reload before "FIGHT".
        ("rtx_match_countdown", Float(3.0)),
        // The locked roster, carried across the match-start map reload. Written by `start_match`
        // immediately before `changelevel` and consumed (and cleared) by `on_worldspawn`.
        //
        // Not a user knob — a persistence channel. The engine unloads the game module on a map
        // change and builds a fresh `GameState`, so `resuming`, `roster` and `bot_roster` are all
        // destroyed by the very reload that is supposed to arm the countdown: the match came back in
        // warmup with an empty roster and simply never started. Cvars live in the engine and survive,
        // which makes this the shortest honest way to carry the intent across. Registered here
        // because `cvar_set` is a no-op on a cvar that does not exist yet.
        ("rtx_match_resume", Str("")),
        // CTF: captures a team needs to win the match (`0` = no limit, ends on timelimit only).
        ("rtx_capturelimit", Float(8.0)),
        // CTF runes: `0` = on (with the Haste speed boost), `1` = off, `2` = on without the speed
        // boost (Haste is attack-rate only). Runes spawn only in CTF.
        ("rtx_runes", Float(0.0)),
        // CTF: allow voluntarily tossing your carried flag (impulse 26) / held rune (impulse 24).
        ("rtx_ctf_tossflag", Bool(true)),
        ("rtx_ctf_tossrune", Bool(true)),
        // Any mode: let players drop items for teammates — a capped ammo backpack (impulse 20) and
        // the current weapon (impulse 21), as in purectf. `0` disables both.
        ("rtx_dropitems", Bool(false)),
        // Midair: minimum height above the floor (units) for a victim to count as airborne, and the
        // knockback multipliers for airborne (`kb_air`) vs grounded (`kb_ground`) rocket hits — the
        // ground value is bigger to pop grounded players up into the air.
        ("rtx_midair_minheight", Float(40.0)),
        ("rtx_midair_kb_ground", Float(6.0)),
        ("rtx_midair_kb_air", Float(3.0)),
        // Navmesh bots: how many to keep on the server (0 = none), and their skill. Bots only spawn
        // once a map's navmesh is built.
        ("rtx_bot_count", Float(0.0)),
        ("rtx_bot_skill", Float(3.0)),
        // Keep bots on the server even with no humans connected (default off).
        ("rtx_bot_alone", Bool(true)),
        // Pacifist bots: in FFA, don't fight — just trail the nearest human (for experimenting).
        ("rtx_bot_pacifist", Bool(false)),
        // Greedy bots: let a fighting bot break off to grab a compelling nearby pickup (powerup, a
        // weapon it lacks, big health/armor) instead of only chasing the enemy — ktx-style item play.
        ("rtx_bot_greed", Bool(true)),
        // Waypoint magnetism: bend the immediate steering waypoint through a desirable, up item lying
        // just off the route corridor so the bot actually steps onto it. The fake-client pickup net
        // grabs incidental items via a generous touch box; a real network client only gets the tight
        // server-side trigger overlap, so steering has to put the hull on the trigger. On by default.
        ("rtx_bot_magnet", Bool(true)),
        // Resource discipline: value health/armor more steeply below the bare-spawn stack, enter the
        // Recover posture on a thin stack (not just low health), treat red armor and megahealth as
        // major "must-cycle" pickups, and panic for ammo when a bot's firepower is about to collapse.
        // Off = the leaner ktx-parity valuation (a topped-up bot ignores items until a true need). On.
        ("rtx_bot_stack", Bool(true)),
        // Measured power scoring: rate a fighter by the frozen power score — expected kills over the
        // next minute as a multiple of a fresh spawn, fitted on 550 dm3 4on4 games — instead of the
        // legacy effective-HP × firepower product. Changes what "am I winning this?" means (equipment
        // dominates, and enemy quad/pentagram finally count), and reprices target selection by what a
        // kill is *worth*: a naked spawn stops being a magnet, a quad carrier becomes one. On.
        // Off restores the legacy currency for A/B measurement. See [`crate::bot::power`].
        ("rtx_bot_power", Bool(true)),
        // Pack discipline: you drop the pack of the weapon *in your hands* when you die, so a bot
        // with no fight on holsters the rocket launcher back to the shotgun, and a bot that just
        // fragged an armed enemy (or whose teammate just died) goes and takes the pack before the
        // other side does. Measured, an armed kill is worth +0.48 forward frags as denial and +0.99
        // when the weapon changes sides — half its value is decided in the seconds after the frag.
        // On. Off restores the old behaviour (carry the big gun always, treat packs as routine).
        ("rtx_bot_pack", Bool(true)),
        // Measured item pricing: value a pickup by how much power it would add to *this* bot, plus
        // the measured forward frag price of taking it (which is mostly the denial). Replaces the
        // hand-written per-category desire rules, and reorders the shopping list where they were
        // wrong: an unarmed bot now wants the rocket launcher more than red armor, cells are priced
        // by the step they move the lightning gun's ammo curve over, and the ring of shadows stops
        // being chased like a quad. On. Off restores the ktx-style desires for A/B measurement.
        ("rtx_bot_power_goals", Bool(true)),
        // Restrict every measured-power behaviour (`rtx_bot_power`, `rtx_bot_power_goals`,
        // `rtx_bot_pack`, and the oracle's team planning) to a single team number, so one match is
        // the whole experiment: same map, same clock, same machine, the two sides differing only in
        // how they value what they are looking at, and the frag differential is the answer.
        // Sequential arms cannot control for any of that. `0` (default) = every team.
        ("rtx_bot_power_team", Float(0.0)),
        // Per-bot goal/pickup diagnostics (off by default). When on, high-rate trace lines go to the
        // per-bot audit ring buffer (dumped via the control channel's `audit` verb), not the console —
        // a per-frame `conprint` floods the console and drops packets. See [`crate::bot::state::Audit`].
        ("rtx_bot_debug", Bool(false)),
        // Per-bot audit ring-buffer budget, in MB (the `rtx_bot_debug` trace store). Oldest lines are
        // evicted once a bot's buffer exceeds this; only allocated while `rtx_bot_debug` is on.
        ("rtx_bot_auditlog", Float(10.0)),
        // Bot evaluation profiler: seconds between server-console reports of what the bot brain costs
        // per bot frame (p95, worst, and the head-room left against the `maxfps` slice the engine
        // allots it). On by default — it's console-only, three lines per report, and the timing is a
        // couple of clock reads per bot; a server that gets slow should say so unprompted rather than
        // wait to be asked. `0` = off, and nothing is timed at all. See [`crate::bot::prof`].
        ("rtx_bot_prof", Float(10.0)),
        // Bots rocket-jump to ledges that would otherwise need a long detour (or are unreachable).
        // Costs health, so a bot only plans one when it clearly beats the walk and it's fit to fly it
        // (has the RL, a rocket, and the health). On by default.
        ("rtx_bot_rocketjump", Bool(true)),
        // Rocket-jump test harness: a TCP control channel (localhost) an external driver connects to
        // for scripted bot puppetry (go to a spot, fly a specific RJ link, read back telemetry). `0`
        // (default) = disabled — no socket is bound. See [`crate::control`].
        ("rtx_control_port", Float(0.0)),
        // Per-frame player movement events on the control channel (movement lab).
        // Off by default: it is pure observability, and a server nobody is measuring
        // should not pay for a per-frame event.
        ("rtx_telemetry", Float(0.0)),
        // Rocket-jump driver knobs, read live each frame and threaded into the driver so the harness
        // can tune them without a rebuild. Each default mirrors the constant it replaces, so live
        // behaviour is unchanged until a knob is set. See [`crate::bot::rj`] / [`crate::bot`].
        ("rtx_rj_stance", Float(crate::bot::RJ_STANCE)),
        ("rtx_rj_aim_tol", Float(crate::bot::RJ_AIM_TOL)),
        ("rtx_rj_stance_timeout", Float(crate::bot::RJ_STANCE_TIMEOUT)),
        ("rtx_rj_liftoff_timeout", Float(crate::bot::RJ_LIFTOFF_TIMEOUT)),
        ("rtx_rj_ballistic_slack", Float(crate::bot::RJ_BALLISTIC_SLACK)),
        // Biases *added* to every solved rocket jump: `delay_bias` to the fire delay (seconds after
        // the jump press), `pitch_bias` to the fire pitch (degrees, QW positive-down). `0` = fly the
        // solved value; both may be negative. A blunt global tuning dial over the offline solve.
        ("rtx_rj_delay_bias", Float(0.0)),
        ("rtx_rj_pitch_bias", Float(0.0)),
        // Curl-jump tuning for plain jump legs (JumpGap/DoubleJump). A navmesh jump link certifies only
        // the straight source→target center line; the bot takes off offset and homes back onto the
        // center, which can sweep its arc into an edge wall beside the certified line. These shape that:
        //  - `rtx_jump_curl_hold`: fraction of the gap to fly holding the *takeoff* heading before the
        //    curl engages — "curl later", so the bot clears the near wall before turning onto the target.
        //  - `rtx_jump_curl_gain`: the air-curl proportional gain (°/s per ° of heading error). `0`
        //    (the default) = use each link's own baked gain (a curl speed jump carries one; the JumpGap
        //    band-aid falls back to `bhop::AIR_CORRECT_GAIN`). Set `> 0` to override every curl for
        //    tuning; lower = a gentler, wider, later-converging curl.
        //  - `rtx_jump_runup`: minimum ground speed (fraction of `sv_maxspeed`) before the takeoff jump
        //    fires, so the bot runs to the lip at speed instead of hopping slow the instant the leg
        //    turns current — "more speed". Held at most ~1s so a cornered bot never deadlocks.
        // All default to today's behavior.
        ("rtx_jump_curl_hold", Float(0.0)),
        ("rtx_jump_curl_gain", Float(0.0)),
        // Minimum run-up speed toward the waypoint (fraction of sv_maxspeed) before a plain jump leg
        // fires — 0.5 (160 ups, ~4 ticks of ground accel) kills the useless standstill pogo without
        // stalling short approaches (a jump within JUMP_NOW_DIST of the lip fires regardless). 0 = off.
        ("rtx_jump_runup", Float(0.5)),
        // Perception (human-like targeting). `rtx_bot_fov` is the view cone (full angle, degrees)
        // within which a bot can *see* a target, widened with skill; 0 = 360° (see everywhere, the
        // old behavior). `rtx_bot_reaction` is the base delay (seconds) a target must stay seen
        // before the bot reacts, shortened with skill; 0 = instant. Both 0 ≈ pre-perception bots.
        ("rtx_bot_fov", Float(120.0)),
        ("rtx_bot_reaction", Float(0.01)),
        // Ceiling on how fast a bot's view turns (deg/s) — the aim spring's angular-speed clamp, so a
        // large look-target flip (a spawn-wait scan, a goal re-pick, a flickering enemy) is a fast human
        // pan, not an instant snap or a spin. 0 = the skill-scaled default (`combat::aim_rate_cap`);
        // > 0 overrides it for tuning. Small combat corrections sit well under it, so aim is unaffected.
        ("rtx_bot_turnrate", Float(0.0)),
        // Opponent modeling: bots keep a shared, observation-gated hypothesis of each opponent's
        // health/armor stack and arsenal (per-team blackboards; the FFA bots share one), reset on
        // death, updated only from events a bot could witness (pickups/gunfire in earshot, damage it
        // dealt). Feeds item denial, target selection, and combat risk. Team coordination itself is
        // unconditional in a team composition; this switch controls only the inferred enemy model.
        // 0 = the old estimate-free behavior.
        ("rtx_bot_model", Bool(true)),
        // Slow team-level reasoning. The worker observes only owned, observation-gated snapshots and
        // publishes expiring advice: rebuild when the measured equipment gap says we are losing on
        // material, take the room around an armor or quad that is about to return (control is future
        // stack — holding the red-armor area swings the next minute by ±1.75 team frags beyond
        // whatever is standing in it), and converge on a believed quad carrier while the kill is
        // still worth its +0.92.
        //
        // **On since it was measured.** A five-arm leave-one-out on dm3 4on4 — each arm a ~24 minute
        // split-team match with `rtx_bot_power_team 1`, so the treated and control sides shared a map,
        // an opponent and a clock — put this at the largest single contribution of the whole bundle:
        // the treated team took 56.5% of frags with everything on and 42.3% with only this switched
        // off, a 14-point swing and the biggest of the four. That is a direct outcome measurement,
        // which is strictly better evidence than the nugget-level holdout originally planned for the
        // promotion, so the holdout (`rtx_bot_oracle_eval` + `rtx_bot_oracle_holdout`) is now a tool
        // for attributing *which* advice pays rather than a gate on shipping any of it.
        //
        // Caveat kept honest: one match per arm, and slice-to-slice variance within a match is large
        // enough that a single 20-minute window has been seen to reverse. The ordering held across
        // every arm, which is what this rests on.
        ("rtx_bot_oracle", Bool(true)),
        ("rtx_bot_oracle_debug", Bool(false)),
        ("rtx_bot_oracle_eval", Bool(false)),
        // Fraction of complete team-plan episodes retained as shadow controls during evaluation.
        ("rtx_bot_oracle_holdout", Float(0.0)),
    ]
};

/// The registered default for an rtx cvar, if it's in the table. Used to read a build-gating cvar
/// with its intended value even before `GAME_INIT`'s `cvar_default` `set` has flushed from the engine
/// command buffer (see [`GameState::rtx_cvar_bool`](crate::game::GameState::rtx_cvar_bool)). A small
/// linear scan over the table — only called during a navmesh build, not per frame.
pub(crate) fn default_of(name: &str) -> Option<CvarSeed> {
    RTX_CVAR_DEFAULTS
        .iter()
        .find(|(n, _)| *n == name)
        .map(|&(_, seed)| seed)
}
