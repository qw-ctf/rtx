# Cvar Reference

Every rtx tunable on one page. Defaults are **seeded only when a cvar is unset**, so a value
from `server.cfg` (or a `set` before `map`) survives each map load. The authoritative registry
is `RTX_CVAR_DEFAULTS` in `crates/rtx-game/src/cvars.rs`.

Part of the [rtx manual](../README.md)

## Movement & combat

Covered in depth in [movement & combat](movement.md).

| cvar | default | effect |
|------|---------|--------|
| `rtx_doublejump` | `1` | Mid-air double jump, once per air travel. |
| `rtx_walljump` | `1` | Kick off a wall you jump into (UT's wall dodge). |
| `rtx_elevator_jump` | `2` | Rising-lift jump boost: `lift_speed × cvar + base`. A multiplier (`0` off, `1` = the lift's true speed, …). |
| `rtx_shootable_grenades` | `1` | Shoot live grenades to detonate them early (the shock combo). |
| `rtx_grapple` | `1` | Grappling hook (purectf port) — every player spawns with it (impulse 22 to select). |
| `rtx_hook_speed` | `1.25` | Hook throw speed multiplier (purectf's `hookspeed`). |
| `rtx_hook_pull` | `1.0` | Hook reel-in speed multiplier (purectf's `hookpull`). |
| `rtx_weapons` | `axe hook sg ssg ng sng gl rl lg` | The weapons the server runs with; an absent token is removed everywhere. |

## Modes & match

Covered in depth in [game modes & match composition](modes.md).

| cvar | default | effect |
|------|---------|--------|
| `rtx_mode` | `dm` | Ruleset: `dm`, `ra`, `midair`, `ctf`, or `race`. Read live each frame. |
| `rtx_match` | *(empty)* | Composition: empty = the mode's natural default, `ffa`, or a team format (`1on1`/`duel`, `2on2`, `2on2on2`, any `NonM…`). Ignored by `ra`. |
| `rtx_ra_countdown` | `3` | Rocket Arena: seconds of spawn-protected countdown before "FIGHT". |
| `rtx_match_countdown` | `3` | Team match / CTF: countdown seconds after the match-start map reload. |
| `rtx_capturelimit` | `8` | CTF: captures to win (`0` = no limit, ends on `timelimit`). |
| `rtx_runes` | `0` | CTF runes: `0` = on, `1` = off, `2` = on without the Haste speed boost. |
| `rtx_ctf_tossflag` | `0` | CTF: allow tossing the carried flag (impulse 26). |
| `rtx_ctf_tossrune` | `0` | CTF: allow tossing the held rune (impulse 24). |
| `rtx_dropitems` | `0` | Let players drop a capped ammo backpack (impulse 20) and their current weapon (impulse 21). |
| `rtx_midair_minheight` | `40` | Midair: minimum height (units) above the floor to count as airborne. |
| `rtx_midair_kb_ground` | `6` | Midair: rocket knockback multiplier for grounded victims. |
| `rtx_midair_kb_air` | `3` | Midair: rocket knockback multiplier for airborne victims. |
| `rtx_maplist` | *(empty)* | Whitespace-separated map rotation, cycled in order at each level end. Not seeded — read live. |

## Race

Covered in depth in [the race mode](modes.md#race--timed-ktx-race-routes).

| cvar | default | effect |
|------|---------|--------|
| `rtx_race_route` | `0` | Which of the map's routes is being run (0-based, clamped). Read live — changing it moves everyone to the new route's start. |
| `rtx_race_optimize` | `0` | Offline racing-line optimizer: iterations *in thousands* to spend TAS'ing each route's line on a worker thread at map load. `0` = off. |
| `rtx_race_line` | `1` | Race bots track the offline-optimized line when one exists; inert unless `rtx_race_optimize` produced one. |

## Bots — population & behaviour

Covered in depth in [the bots](bots.md).

| cvar | default | effect |
|------|---------|--------|
| `rtx_bot_count` | `0` | How many bots to keep on the server (population reconciled, humans get room). |
| `rtx_bot_skill` | `3` | Skill 0–7: aim, turn/track speed, view cone, reaction time. |
| `rtx_bot_alone` | `0` | Keep bots on an empty server (`1`) or have them leave (`0`). |
| `rtx_bot_pacifist` | `0` | Bots don't fight — they trail the nearest human (outside Race). |
| `rtx_bot_greed` | `1` | Allow optional ordinary item detours during a fight. |
| `rtx_bot_fov` | `120` | View cone (full angle, degrees) a bot can see targets in; widened with skill. `0` = 360°. |
| `rtx_bot_reaction` | `0.4` | Base reaction delay (seconds) before acting on a newly seen target; shortened with skill. `0` = instant. |
| `rtx_bot_model` | `1` | Opponent modeling: shared observation-gated hypotheses of enemy stack/arsenal. `0` = estimate-free. |
| `rtx_bot_stack` | `1` | Resource discipline: steeper valuation below the bare-spawn stack, RA/mega cycling, ammo panic. |
| `rtx_bot_magnet` | `1` | Waypoint magnetism: bend the steering waypoint through a desirable item just off the route. |
| `rtx_bot_turnrate` | `0` | Ceiling on how fast a bot's view turns (deg/s) — the aim spring's angular-speed clamp, so a big look-flip pans like a human. `0` = skill-scaled default; `>0` overrides for tuning. |

## Bots — movement

Covered in depth in [bot navigation](bots.md#navigation).

| cvar | default | effect |
|------|---------|--------|
| `rtx_bot_bhop` | `1` | Bunnyhop (air-strafe to build speed) on open stretches. |
| `rtx_bot_zigzag` | `1` | Ground-zigzag (circle-strafe) on straight corridors too short to hop. Sub-toggle of bhop. |
| `rtx_bot_curljump` | `0` | Generate curl jumps (run-up + air-turn onto an offset platform), certified by a pmove rollout. Sub-toggle of bhop. |
| `rtx_bot_bandplan` | `1` | Plan over speed bands (kinodynamic A*), crediting speed carried between legs. `0` = plain A*. |
| `rtx_bot_rocketjump` | `1` | Rocket-jump to ledges when it clearly beats the walk and the bot is fit to fly it. |
| `rtx_bot_hazard_health` | `1` | A bot's health weights its willingness to shortcut through lava/slime. `0` = price every bot as a bare spawn. |
| `rtx_bot_hazard_k` | `15` | Seconds of detour accepted per unit of survival strength a hazard eats (higher = more timid). |
| `rtx_bot_lod` | `1` | Navigate over the coarse LOD cluster/portal hierarchy — goal scoring and long steering read a bounded-overestimate coarse cost. `0` = exact whole-graph floods. |
| `rtx_bot_nearfield` | `1` | Steer the last metre off a fine 8u clearance grid: nudge off walls and drop-edges, centre through doorways. `0` = the drop-only edge probe. |
| `rtx_bot_glide` | `1` | When the near-field certifies a straight look-ahead chord is clear, glide toward it instead of the next cell centre (smooths the grid zigzag). Sub-toggle of nearfield. |
| `rtx_bot_ledgecap` | `210` | Careful-ledge walk-speed cap (u/s) on cells flagged beside a fatal drop (an open-cored spiral's inner edge). `0` = full maxspeed. |

## Development & tuning

Debug and harness knobs — safe to ignore on a play server. See
[development & tooling](development.md).

| cvar | default | effect |
|------|---------|--------|
| `rtx_bot_debug` | `0` | Per-bot goal/pickup diagnostics to the server console. |
| `rtx_bot_par` | `1` | Fan a goal pick's independent navmesh floods across a persistent worker pool (bit-identical to serial). `0` = run them inline on the main thread. |
| `rtx_bot_prof` | `10` | Seconds between bot-evaluation profile reports on the server console (p95, worst frame, head-room against the engine's `maxfps` slice). `0` = off, and nothing is timed. |
| `rtx_control_port` | `0` | TCP control channel (localhost) for scripted bot puppetry — teleport, goto, fly a link, read telemetry. `0` = no socket bound. |
| `rtx_rj_stance` | `16` | Rocket-jump driver: stance offset. |
| `rtx_rj_aim_tol` | `0.5` | Rocket-jump driver: aim tolerance (degrees). |
| `rtx_rj_stance_timeout` | `2.5` | Rocket-jump driver: stance timeout (seconds). |
| `rtx_rj_liftoff_timeout` | `0.3` | Rocket-jump driver: liftoff timeout (seconds). |
| `rtx_rj_ballistic_slack` | `1.0` | Rocket-jump driver: ballistic slack. |
| `rtx_rj_delay_bias` | `0` | Seconds *added* to every solved rocket-jump fire delay (may be negative). |
| `rtx_rj_pitch_bias` | `0` | Degrees *added* to every solved rocket-jump fire pitch (QW positive-down; may be negative). |
| `rtx_jump_curl_hold` | `0` | Fraction of a gap flown on the takeoff heading before the air-curl engages ("curl later"). |
| `rtx_jump_curl_gain` | `0` | Air-curl proportional gain override (°/s per °). `0` = each link's own baked gain. |
| `rtx_jump_runup` | `0.5` | Minimum run-up speed toward the waypoint (fraction of `sv_maxspeed`) before a plain jump leg fires — `0.5` (~160 ups) kills the standstill pogo; a jump close to the lip fires regardless. `0` = off. |
| `rtx_wedge_debug` | *(unset)* | Dev-only: log the animation-wedge catch. Read directly, never seeded. |

The `rtx_rj_*` and `rtx_jump_*` knobs are read live each frame and default to the constants they
replace, so behaviour is unchanged until one is set — they exist for the
[rocket-jump tuning harness](../crates/rjmcp/README.md).

---

*See also: [movement & combat](movement.md) · [game modes](modes.md) · [the bots](bots.md) ·
[network client](netclient.md) · [development & tooling](development.md)*
