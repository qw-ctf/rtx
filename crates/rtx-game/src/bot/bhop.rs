// SPDX-License-Identifier: AGPL-3.0-or-later

//! Bunnyhop — the QuakeWorld air-strafe controller for bots. Chaining jumps (so ground friction
//! never bites) while **air-strafing** lets a player accelerate far past `sv_maxspeed`: QW's air
//! acceleration clamps the *projected* wish speed to a small cap (~30 ups), so when the wish
//! direction is held roughly perpendicular to the velocity — one strafe key, view swept to keep the
//! angle — the speed grows every frame without bound. This module is the whole hop cycle as pure
//! math: a [`Bhop`] state machine (prestrafe → hop → landing → re-takeoff) that turns one frame of
//! inputs into one usercmd, plus the per-frame strafe/prestrafe angle solvers. The engine runs the
//! actual `PM_PlayerMove`; the bot only emits the usercmd (`crate::bot`).
//!
//! Two engine facts the whole design leans on (verified against FTEQW `common/pmove.c`):
//! - `PM_CheckJump` runs **before** `PM_Friction`, so a landing frame with jump pressed clears
//!   `onground` first — it skips ground friction entirely and takes a full frame of *air* accel.
//!   The landing frame therefore strafes exactly like an air frame, with jump held.
//! - Ground acceleration has no 30-ups cap (only `sv_maxspeed`), so angling the wish direction off
//!   the velocity on the ground — the speedrunner's circle-jump prestrafe — pushes well past
//!   `sv_maxspeed` before the first hop (equilibrium against friction ≈ 490 ups at stock cvars).

use glam::Vec2;

use super::human_profile::HumanMovementProfile;
use crate::math::{wrap180, yaw_of};
// The band the takeoff regime holds a curl's solved speed within — single-sourced from the certifier,
// which proves the landing across exactly this band.
use crate::navmesh::CURL_V_HOLD_TOL;
use rtx_nav::qphys::AIR_CAP;
// The pure movement oracles + `Cmd`/`Strafe`/`MOVE_SPEED` now live in `rtx_nav::strafe`, so the
// navmesh build can run the exact physics the controller flies (curl-jump certification). Glob
// re-export them here, so both this module's state machine and the rest of the game crate keep
// using `bhop::air_correct`, `bhop::Cmd`, `bhop::apply_airaccel`, `bhop::MOVE_SPEED`, … unchanged.
pub use rtx_nav::strafe::*;

/// The engine's fixed bot tick (bots run `SV_RunCmd` at ~77 Hz regardless of what we pass as
/// `msec`), used only to size the weave band. The live accel math uses the real per-frame `dt`.
const DT_NOMINAL: f32 = 1.0 / 77.0;
/// Flat-ground hop airtime: `2 · JUMP_VZ / gravity` = 2·270/800 (see `navmesh`). Public so the
/// caller can size its forward wall probe to one hop's flight (`speed · T_HOP`).
pub const T_HOP: f32 = 0.675;
/// How many times per hop the ground serpentine ([`prestrafe`]/zigzag) switches sides. ~3 matches
/// how human runners weave; the *air* hop path uses the lobe scheduler below instead.
const FLIPS_PER_HOP: f32 = 3.0;
/// Three-reversal hop phases for a symmetric `+ - + -` max-gain slalom. With normalized outer
/// lobe duration `a` and inner duration `b`, final heading requires `2a + 2b = 1`; zero integrated
/// heading (and therefore zero first-order lateral displacement) requires `a² + 2ab − b² = 0`.
/// Solving gives `a = (1 − 1/√2)/2` (14.64%); the stable 77 Hz boundary is 8/52 = 15.38%.
/// A discrete rollout gives the same tick at 600, 800, and 1200 ups: perpendicular max-gain accel
/// adds the same lateral velocity per tick, so reduced angular change cancels greater forward speed.
const AIR_FLIP_PHASES: [f32; 3] = [8.0 / 52.0, 0.5, 44.0 / 52.0];
/// The QWD control aggregate that motivated phase locking covers dedicated 600+ ups bunny runs.
/// Below this, retain heading-error reversals: their shorter transient is safer during launch.
const PHASE_LOCK_MIN_SPEED: f32 = 600.0;

/// The profile's air-lobe rate is a request, capped by [`omega_gain_max`]. Dedicated high-speed QWD
/// bunny runs put almost every moving command near perpendicular to velocity, so the calibrated
/// profile deliberately requests that physical maximum instead of the old gentle-rate compromise.
/// View yaw still rides the velocity in [`strafe_rate`]; changing strafe side bends the path without
/// snapping the bot's eyes.
///
/// The profile's symmetric lobe deadband keeps that max-gain weave centered on the route. A tighter
/// band changes side about twice per hop and bounds lateral travel; bearing-error correction adds no
/// extra rate once the physical maximum has already been reached.

/// How long the entry conditions must hold before engaging, so a momentary straightaway doesn't
/// stutter the gait. Applies only to the initial engage; disengage decisions happen on landings.
/// Kept short — the runway is *consumed* while waiting (~32u per 0.1s at walk speed), so a long
/// delay quietly raises the effective entry bar well past [`RUNWAY_ENGAGE`].
/// Prestrafe launches into the first hop at this speed — just under the ground-friction
/// equilibrium (~490), so it's reachable quickly and the jump leaves before gains flatten.
/// Give up circling and just jump if the target speed hasn't arrived by then (shoved, uphill…).
/// Only bother prestrafing with this much corridor ahead; shorter → hop immediately.
/// Takeoff regime: how close (units, run-up remaining) the takeoff edge must be before a committed
/// high-speed jump leaps. The bot keeps the ground circle-strafe until it's within this of the lip,
/// then jumps *once* — so the takeoff point stays where the link was certified. Kept small (a couple
/// of ticks of travel) so the leap point tracks the certified edge. For a curl the caller passes a
/// *signed along-corridor* runway, so crossing the takeoff line (progress `< LIP_REACH`, incl. negative
/// past it) triggers the leap — a radial ball the weave can skirt would U-turn. Distinct from the eager
/// [`LAUNCH_MIN_FRAC`] leap a plain hop takes. `pub` so steer gates the run-up aim on the same threshold.
pub const LIP_REACH: f32 = 28.0;
/// Fixed runway required to engage — enough for one worthwhile hop (a flat hop at walk speed
/// covers ~216u). Deliberately *not* speed-scaled: the old `speed·0.9` bar rose as the bot gained
/// speed and disengaged it mid-run — the policy capped the very thing it built.
pub const RUNWAY_ENGAGE: f32 = 256.0;
/// Minimum ground speed before the hop cycle engages: a human *runs up* to near `sv_maxspeed` and
/// then leaps into the circle-jump, never bunnyhops from a standstill. Below this the bot just runs
/// (normal ground acceleration) until it's moving, so the first hop leaves at a real running speed.
pub const RUN_UP_SPEED: f32 = 280.0;
/// Fraction of `sv_maxspeed` a grounded bot must reach before it will *leap*. This is the backstop
/// inside the controller (the [`RUN_UP_SPEED`] gate only decides *engagement*): every takeoff path —
/// a short-runway direct-to-Hop entry, a committed speed jump, or a mid-chain landing after a bump —
/// keeps circle-strafing on the ground until it's at full run speed rather than leaping slow. Below
/// ~maxspeed, ground accel (~40 ups/tick toward the wish) far outgains the 30-ups air cap, so a slow
/// leap is strictly worse than one more ground stride.
/// Don't leap unless a hop's flight (`speed·T_HOP`) fits this fraction of the forward clear distance
/// (`Input.clear`): a bot flying at a wall is better off carving on the ground (which turns far
/// faster than air) and re-launching once it's aimed down open corridor — the land-carve-rejump a
/// human does at a bend, instead of face-planting the wall mid-arc.
/// Airborne, if the wall is nearer than this many seconds of flight, abandon the slalom lobe and
/// carve toward the bearing (the open corridor) at the full physical turn rate.
const WALL_PANIC_T: f32 = 0.3;
/// Slack beyond the current hop's flight distance when deciding whether another hop fits.

/// Minimum corridor (≈3 grid cells) to bother with a ground zigzag: too short for a hop
/// ([`RUNWAY_ENGAGE`]) but long and straight enough to profit from the circle-strafe. The caller
/// gates on this; the controller just runs the strafe until the corridor bends or a hop fits.
pub const ZIGZAG_ENGAGE: f32 = 96.0;
/// Cap the weave band on a ground zigzag. The ground-optimal angle `θg = acos(u*/v)` grows toward
/// ~55° near the friction equilibrium, and an uncapped serpentine sweeps too wide for a 3-cell
/// corridor — clamp the deadband so the S-curve stays inside the walls. Launch prestrafe (which
/// has a long runway by construction) is left uncapped.

// `air_accel_max`, `theta_star`, `omega_gain_max`, `strafe_rate`, `air_correct`, and `AIR_CORRECT_GAIN_DEFAULT`
// now live in `rtx_nav::strafe` (glob-re-exported above), shared with the navmesh build's curl certifier.

/// Heading deadband (degrees) for the strafe-sign weave, sized from the physics so the sign flips
/// ~[`FLIPS_PER_HOP`] times per hop: a perpendicular strafe rotates the velocity by
/// `ψ = atan(cap/v)` per tick (≈5.4° at 320 ups, shrinking with speed), a hop is `T_HOP/dt` ticks,
/// and one weave period spans two bands. The old fixed ±3° band flipped every tick or two at low
/// speed — the "view shake"; this yields a smooth ±25–45° serpentine that tightens as speed grows.
fn weave_band(speed: f32) -> f32 {
    let psi = (AIR_CAP / speed.max(1.0)).atan().to_degrees();
    (psi * (T_HOP / DT_NOMINAL) / (2.0 * FLIPS_PER_HOP)).clamp(8.0, 45.0)
}

/// The sticky strafe sign: keep curving `prev_sigma`'s way until the bearing error overshoots the
/// weave band on the other side, then flip — an S-curve whose average heading is the bearing.
fn weave_sigma(err: f32, prev_sigma: f32, band: f32) -> f32 {
    if prev_sigma == 0.0 {
        if err >= 0.0 {
            1.0
        } else {
            -1.0
        }
    } else if err * prev_sigma < -band {
        -prev_sigma // overshot the other way — flip and weave back
    } else {
        prev_sigma // keep curving the same way
    }
}

/// The **max-gain** air-strafe: aim the view so a single held strafe key puts the wish direction at
/// the speed-*optimal* ([`theta_star`], ~perpendicular) angle off the velocity, and weave the strafe
/// side toward `wp_bearing`. This is [`strafe_rate`] at [`omega_gain_max`] — maximum speed gain and
/// its corresponding turn rate. The calibrated live hop path now selects the same regime;
/// this remains the reference primitive and speed-gain oracle used by the unit tests and navmesh
/// speed-jump model.
#[allow(dead_code)]
pub fn strafe(v_xy: Vec2, wp_bearing: f32, prev_sigma: f32, a_max: f32) -> Strafe {
    let speed = v_xy.length().max(1.0);
    let vel_yaw = yaw_of(v_xy);
    let err = wrap180(wp_bearing - vel_yaw);
    let sigma = weave_sigma(err, prev_sigma, weave_band(speed));
    let theta = theta_star(speed, a_max);
    Strafe {
        view_yaw: wrap180(vel_yaw + sigma * (theta - 90.0)),
        forward: 0.0,
        side: -sigma * MOVE_SPEED,
        sigma,
    }
}

/// The ground circle-strafe (the speedrunner's prestrafe / circle jump): hold the wish direction
/// at the ground-optimal angle off the velocity to accelerate past `sv_maxspeed` before takeoff.
/// FTE applies ground friction before acceleration, so the solver first predicts that friction step
/// and chooses the command from the velocity the accelerator will actually see. From the ground-
/// accel geometry (`addspeed = maxspeed − u`, cap `a_g = accel·maxspeed·dt`), the
/// gain² `2·u·a + a²` under `a = min(a_g, maxspeed − u)` peaks at `u* = maxspeed − a_g`, i.e.
/// `θg = acos(u*/speed)` — 0° until `speed > u*` (≈278), then bending outward as speed grows.
///
/// The wish angle is expressed through the `forward`/`side` split (`wishvel = forward·fwd +
/// side·right`), decoupled from the view: below `u*` the eyes look straight down the bearing (a
/// plain run-up); once the circle-strafe arc begins the view **rides the velocity heading**, so the
/// eyes tilt into the arc and sweep up through the takeoff direction — the human circle-jump wind-up
/// — then hand off continuously to the air lobe (which also looks along the velocity). The world
/// wishdir is identical either way; `emit` reprojects it onto the spring-smoothed view, so moving
/// the look target never disturbs the movement or snaps the eyes.
#[allow(clippy::too_many_arguments)]
pub fn prestrafe(
    v_xy: Vec2,
    bearing: f32,
    prev_sigma: f32,
    a_g: f32,
    maxspeed: f32,
    friction: f32,
    stopspeed: f32,
    dt: f32,
    band_cap: f32,
) -> Strafe {
    let v_accel = apply_friction(v_xy, friction, stopspeed, dt);
    let speed = v_accel.length();
    // Below the angling threshold (or barely moving) there's nothing to exploit: run at the
    // bearing to build base speed. Also avoids steering off a garbage yaw from a ~zero velocity.
    let u_star = (maxspeed - a_g).max(0.0);
    if speed <= u_star.max(60.0) {
        return Strafe {
            view_yaw: bearing,
            forward: MOVE_SPEED,
            side: 0.0,
            sigma: prev_sigma,
        };
    }
    let vel_yaw = yaw_of(v_accel);
    let err = wrap180(bearing - vel_yaw);
    let sigma = weave_sigma(err, prev_sigma, weave_band(speed).min(band_cap));
    let theta_g = (u_star / speed).clamp(0.0, 1.0).acos().to_degrees();
    // Wish at `vel_yaw + sigma·θg` in world; with the view riding the velocity that is `sigma·θg`
    // relative to the view, carried in forward/side (same wishdir as a bearing-locked view).
    let (ts, tc) = theta_g.to_radians().sin_cos();
    Strafe {
        view_yaw: vel_yaw,
        forward: MOVE_SPEED * tc,
        side: -sigma * MOVE_SPEED * ts,
        sigma,
    }
}

// --- the hop-cycle state machine ----------------------------------------------------------------

/// Where the controller is in the hop cycle.
#[derive(Default, Clone, Copy, PartialEq, Eq, Debug)]
pub enum Phase {
    /// Not engaged; the caller steers normally through the aim spring.
    #[default]
    Off,
    /// On the ground building launch speed with the circle-strafe.
    Prestrafe,
    /// The hop loop: air-strafing, and strafe+jump on each landing frame.
    Hop,
    /// A standalone ground circle-strafe on a corridor too short to hop: gain toward the friction
    /// equilibrium without leaving the ground, and hand off to [`Phase::Hop`] the moment a runway
    /// opens. Same math as [`Phase::Prestrafe`] but with a capped weave band and no launch.
    Zigzag,
}

/// Physics inputs the controller needs each frame, read from cvars by the caller.
pub struct Env {
    /// Frame time (s).
    pub dt: f32,
    /// `sv_accelerate`.
    pub accel: f32,
    /// `sv_maxspeed`.
    pub maxspeed: f32,
    /// `sv_friction` and `sv_stopspeed`, used to predict the velocity that ground acceleration sees.
    pub friction: f32,
    pub stopspeed: f32,
    pub profile: HumanMovementProfile,
}

/// One frame of world state + policy verdicts from the caller (`bot.rs` owns everything that needs
/// game state — combat, legs, cvars; the controller owns *when* in the hop cycle they may apply).
pub struct Input {
    /// Horizontal velocity.
    pub v_xy: Vec2,
    /// `FL_ONGROUND` this frame.
    pub on_ground: bool,
    /// Corridor bearing to weave around (degrees).
    pub bearing: f32,
    /// Remaining straight-ish corridor (units); on a speed jump, the run-up to the takeoff.
    pub runway: f32,
    /// Entry conditions hold (leg kind, goal distance, runway ≥ [`RUNWAY_ENGAGE`]).
    pub eligible: bool,
    /// A ground zigzag is worth running: a straight Walk/Step corridor ≥ [`ZIGZAG_ENGAGE`] but too
    /// short to satisfy `eligible`. Ignored once a hop engages; superseded by `eligible`/`committed`.
    pub zigzag: bool,
    /// Lenient conditions to take *another* hop from a landing (leg kind still ok, goal ahead).
    pub sustain: bool,
    /// Hard off — combat/hook/gate/grenade wants the view, or the cvar is off. The only thing
    /// that may cut a hop mid-air.
    pub veto: bool,
    /// A `SpeedJump` leg: the runway is pre-verified, so bypass entry/continuation checks.
    pub committed: bool,
    /// The banded planner routed this run to carry speed here (a leg's planned band ≥ 1). Licenses
    /// engaging straight into the hop cycle when already fast (don't ground into friction to
    /// prestrafe) and keeping the chain alive across leg-kind churn without a runway re-check.
    pub carry: bool,
    /// At the takeoff edge too slow to clear the gap (`sj_hold`): keep building, don't leap.
    pub hold_jump: bool,
    /// Required takeoff speed for a committed speed jump (0 = none / a plain leg). A high-`v_req` jump
    /// (a curl/speed jump the bot can't reach at ordinary run speed) drives the *takeoff regime*: keep
    /// **ground prestrafe** (circle-strafe, which outgains the air cap far below maxspeed) all the way
    /// to the lip, then leap once at `~0.9·v_req` — not the eager `LAUNCH_MIN_FRAC` hop that leaves slow.
    pub takeoff_speed: f32,
    /// Air-curl gain for a committed speed jump: once airborne, home the velocity onto the bearing with
    /// [`air_correct`] at this rate (a single smooth pursuit curl onto the landing) rather than the hop
    /// slalom. `> 0` selects the curl (a speed jump is one leap, not a chain); `≤ 0` keeps the slalom.
    pub curl_gain: f32,
    /// Free flight distance straight ahead along the velocity (units), from a forward hull trace —
    /// how far the bot could fly before hitting a wall. `f32::INFINITY` = unknown/open (the default
    /// off the live path). Gates leaping at a wall and drives the mid-air carve; see [`Bhop::step`].
    pub clear: f32,
    /// Game time (s).
    pub now: f32,
}

// `Cmd` (the usercmd the controller emits) now lives in `rtx_nav::strafe`, glob-re-exported above.

/// The bunnyhop controller's per-bot state. Lives on `BotState`; drive with [`Bhop::step`].
#[derive(Default, Clone, Debug)]
pub struct Bhop {
    /// Where in the hop cycle we are.
    pub phase: Phase,
    /// Sticky strafe sign (±1; 0 = unseeded). On the air lobe this is the current slalom side,
    /// flipped when the heading curves a deadband past the bearing; on the ground
    /// ([`prestrafe`]/zigzag) it is the weave sign.
    sigma: f32,
    /// When the entry conditions started holding (0 = not holding) — the engage hysteresis clock.
    eligible_since: f32,
    /// When the current phase began.
    phase_start: f32,
    /// Last frame's cmd pressed jump — the pulse guard: if a press didn't take (still grounded),
    /// release for one frame so `PM_CheckJump` sees a fresh edge, then press again.
    jump_prev: bool,
    /// Game time of the current takeoff and the next phase-locked reversal (0..3).
    air_start: f32,
    air_flip_stage: u8,
    /// Frozen at takeoff so one hop never changes reversal policy in mid-air.
    air_phase_locked: bool,
    /// Per-hop middle reversal phase, biased from 0.5 to correct the takeoff heading error.
    air_mid_phase: f32,
    /// Telemetry: hops taken, weave sign flips, and peak speed this engagement.
    pub hops: u32,
    pub flips: u32,
    pub peak: f32,
    /// Telemetry: why the last engagement ended ("veto" / "runway" / "leg").
    pub off_reason: &'static str,
}

impl Bhop {
    /// Drive one frame. `Some(cmd)` = the controller owns the view and move this frame;
    /// `None` = not engaged — the caller steers through the normal aim-spring path.
    pub fn step(&mut self, i: &Input, env: &Env) -> Option<Cmd> {
        if i.veto {
            if self.phase != Phase::Off {
                self.disengage("veto");
            }
            self.eligible_since = 0.0;
            return None;
        }
        if self.phase == Phase::Off {
            let engage = if i.committed {
                true // a SpeedJump leg is a pre-verified runway — no hysteresis
            } else if i.eligible {
                if self.eligible_since == 0.0 {
                    self.eligible_since = i.now;
                }
                i.now - self.eligible_since >= env.profile.engage_delay
            } else {
                self.eligible_since = 0.0;
                false
            };
            if engage {
                self.engage(i, env.maxspeed, env.profile);
            } else if i.zigzag && i.on_ground {
                // No hop yet, but a short straight corridor is worth a ground circle-strafe.
                self.enter_zigzag(i);
            } else {
                return None;
            }
        }
        let speed = i.v_xy.length();
        self.peak = self.peak.max(speed);
        let a_max = air_accel_max(env.accel, env.maxspeed, env.dt);
        let a_g = a_max; // same cap formula on the ground; only the addspeed limit differs

        if self.phase == Phase::Zigzag {
            // A real runway opened (or a SpeedJump leg committed): promote to the hop cycle,
            // carrying the speed we built — `engage` picks Prestrafe vs Hop by speed/runway.
            if i.eligible || i.committed {
                self.engage(i, env.maxspeed, env.profile);
            } else if !i.zigzag {
                // Corridor bent or ran out (`runway()` stops at bends), so corners exit cleanly.
                self.disengage("zigzag");
                return None;
            } else if !i.on_ground {
                // Tolerate the 1–2 airborne frames pmove yields stepping down a Step leg: hold the
                // bearing rather than applying ground math mid-air or disengaging.
                return Some(Cmd {
                    view_yaw: i.bearing,
                    forward: MOVE_SPEED,
                    side: 0.0,
                    jump: false,
                });
            } else {
                return Some(self.ground_cmd(i, a_g, env, env.profile.zigzag_band_cap));
            }
        }

        if self.phase == Phase::Prestrafe {
            // Takeoff regime: a committed high-speed jump holds the circle-strafe all the way to the
            // lip (the early-launch clause below would leap the moment prestrafe is entered on a short
            // runway, leaving slow). Everything else launches at target speed / time / runway-out.
            let sj_build = i.committed && i.takeoff_speed > 0.0;
            let launch = if sj_build {
                !i.on_ground || i.runway < LIP_REACH
            } else {
                !i.on_ground
                    || speed >= env.profile.prestrafe_target
                    || i.now - self.phase_start > env.profile.prestrafe_max_t
                    || i.runway < speed * T_HOP * 2.0 + env.profile.hop_margin // keep room to actually hop
            };
            if !launch {
                // `takeoff_cmd` holds a curl's solved speed; a plain prestrafe (takeoff_speed 0) builds.
                return Some(self.takeoff_cmd(i, a_g, env));
            }
            self.phase = Phase::Hop;
            self.phase_start = i.now;
            // The ground circle and air slalom have different reversal geometry. Seed the first air
            // lobe from its own bearing error instead of inheriting whichever side the run-up ended on.
            self.sigma = 0.0;
        }
        self.hop_cmd(i, speed, a_max, a_g, env)
    }

    /// The hop loop: air-strafe while airborne; on a landing frame decide whether another hop
    /// fits, and if so take off again with a strafe+jump cmd (full air-accel gain — see module
    /// docs on `PM_CheckJump` running before `PM_Friction`).
    fn hop_cmd(&mut self, i: &Input, speed: f32, a_max: f32, a_g: f32, env: &Env) -> Option<Cmd> {
        let maxspeed = env.maxspeed;
        let dt = env.dt;
        let profile = env.profile;
        if !i.on_ground {
            self.jump_prev = false; // airborne releases the button, re-arming PM_CheckJump
                                    // A committed speed jump is a single leap onto a fixed landing: curl the velocity smoothly
                                    // onto the bearing with `air_correct` (pursuit guidance), not the hop slalom (whose lobe
                                    // flips scatter the landing point). The bearing already tracks the target once airborne.
            let s = if i.committed && i.takeoff_speed > 0.0 && i.curl_gain > 0.0 {
                air_correct(i.v_xy, i.bearing, a_max, dt, i.curl_gain)
            } else {
                self.air_strafe(i, speed, a_max, dt, profile)
            };
            return Some(Cmd {
                view_yaw: s.view_yaw,
                forward: s.forward,
                side: s.side,
                jump: false,
            });
        }
        // Landing (or first) ground frame — the only place a run ends by policy. A planned carry
        // keeps the chain alive across leg-kind churn even where the per-landing runway arithmetic
        // would give up (the planner already proved speed belongs here).
        let keep_hopping = i.committed || i.carry || (i.sustain && i.runway >= speed * T_HOP + profile.hop_margin);
        if !keep_hopping {
            self.disengage(if i.sustain { "runway" } else { "leg" });
            return None;
        }
        // Run up before the leap: keep circle-strafing on the ground rather than take off slow.
        // A committed high-speed jump follows the *takeoff regime* — build all the way to the lip and
        // leap there once, so the takeoff point stays where the link was certified (leaping early even
        // when already fast would undershoot). Everything else uses the eager run-up gate: hold until
        // full run speed ([`LAUNCH_MIN_FRAC`], the human "run first, then jump"), a speed jump's edge
        // is fast enough (`hold_jump`), and we're not about to fly into a wall. Ground accel outgains
        // the air cap below maxspeed, so holding on the ground only ever helps the speed.
        let sj_takeoff = i.committed && i.takeoff_speed > 0.0;
        if sj_takeoff {
            if i.runway >= LIP_REACH {
                // Hold the solved takeoff speed to the lip (coast above the band, build below).
                return Some(self.takeoff_cmd(i, a_g, env));
            }
        } else if i.hold_jump
            || speed < profile.launch_min_frac * maxspeed
            || i.clear < speed * T_HOP * profile.wall_hold_frac
        {
            return Some(self.ground_cmd(i, a_g, env, f32::INFINITY));
        }
        let jump = !self.jump_prev;
        self.jump_prev = jump;
        if jump {
            self.hops += 1;
            self.air_start = i.now;
            self.air_flip_stage = 0;
            self.air_phase_locked = profile.phase_locked_flips && speed >= PHASE_LOCK_MIN_SPEED;
            self.air_mid_phase = 0.5;
            if self.air_phase_locked {
                let err = wrap180(i.bearing - yaw_of(i.v_xy));
                if err.abs() > 1.0 {
                    self.sigma = err.signum();
                }
                // Moving the middle reversal earlier by `b` changes the signed turn time by
                // `2b·T_HOP`. Apply a finite-pmove correction to that first-order solution, capped
                // so all four lobes remain substantial and the three-reversal gait cannot collapse.
                let omega = omega_gain_max(speed, a_max, dt).max(1.0);
                let bias = (err.abs() / (1.5 * omega * T_HOP)).clamp(0.0, 0.18);
                self.air_mid_phase -= bias;
            }
            // A curl leaps straight down the corridor — the certifier's tick-0 is a bearing-forward wish,
            // so don't spend the launch frame on a slalom lobe that skews the takeoff heading.
            if sj_takeoff {
                return Some(Cmd {
                    view_yaw: i.bearing,
                    forward: MOVE_SPEED,
                    side: 0.0,
                    jump: true,
                });
            }
            let s = self.air_strafe(i, speed, a_max, dt, profile);
            Some(Cmd {
                view_yaw: s.view_yaw,
                forward: s.forward,
                side: s.side,
                jump: true,
            })
        } else {
            // The pulse-release frame after a press that didn't take: still gain on the ground.
            Some(self.ground_cmd(i, a_g, env, f32::INFINITY))
        }
    }

    /// One air-strafe frame at the profile's requested lobe rate, capped at the physical
    /// [`omega_gain_max`]. Flip the strafe side once the heading has curved past the profile deadband;
    /// the symmetric threshold makes the max-gain S-curve self-center on the route.
    fn air_strafe(&mut self, i: &Input, speed: f32, a_max: f32, dt: f32, profile: HumanMovementProfile) -> Strafe {
        let vel_yaw = yaw_of(i.v_xy);
        let err = wrap180(i.bearing - vel_yaw);
        if self.sigma == 0.0 {
            self.sigma = if err >= 0.0 { 1.0 } else { -1.0 };
        }
        // Wall panic: flying at a wall within `WALL_PANIC_T` of flight — drop the slalom and carve
        // toward the bearing (the open corridor the look-ahead points down) at the full physical
        // turn rate. Holds the current side if we're already dead on the bearing.
        if i.clear < speed * WALL_PANIC_T {
            if err.abs() > 1.0 {
                self.sigma = err.signum();
            }
            return max_turn_strafe(i.v_xy, self.sigma, a_max);
        }
        if self.air_phase_locked {
            // Four symmetric, unequal lobes put three human/optimizer-style direction changes at
            // phases that return both heading and first-order lateral displacement to zero. Carrying
            // the final side into the next takeoff also alternates the mirror-image hop shapes,
            // cancelling the small remainder from discrete frame quantization over a pair of hops.
            let phase = ((i.now - self.air_start) / T_HOP).clamp(0.0, 1.0);
            let phases = [AIR_FLIP_PHASES[0], self.air_mid_phase, AIR_FLIP_PHASES[2]];
            let due = phases.iter().filter(|&&at| phase >= at).count() as u8;
            while self.air_flip_stage < due {
                self.sigma = -self.sigma;
                self.air_flip_stage += 1;
                self.flips += 1;
            }
        } else if err * self.sigma < -profile.lobe_deadband {
            self.sigma = -self.sigma;
            self.flips += 1;
        }
        // Base rate + within-hop correction, plus a steep ramp once the error runs past the lobe
        // band (a bend or wall to carve) — always capped by what this tick can physically deliver.
        let omega = (profile.omega_base
            + err.abs() / T_HOP
            + (err.abs() - profile.lobe_deadband).max(0.0) * profile.error_gain)
            .min(omega_gain_max(speed, a_max, dt));
        strafe_rate(i.v_xy, self.sigma, omega, a_max, dt)
    }

    /// The takeoff regime's ground frame: *hold* the link's solved takeoff speed rather than max out.
    /// Above the hold band, coast on the bearing — ground accel adds nothing past `sv_maxspeed`, so
    /// friction bleeds toward the target while the heading re-centres on the corridor (which also keeps
    /// the launch heading inside the certified guard); below it, circle-strafe to build. A human holds a
    /// controlled speed exactly this way (396-416 across the recorded demos) instead of leaping at the
    /// ~484 prestrafe equilibrium, whose reach overshoots any moderate gap. The certifier proves the
    /// landing across this same [`CURL_V_HOLD_TOL`] band, so the two must stay in step.
    fn takeoff_cmd(&mut self, i: &Input, a_g: f32, env: &Env) -> Cmd {
        if i.takeoff_speed > 0.0 && i.v_xy.length() > i.takeoff_speed * (1.0 + CURL_V_HOLD_TOL) {
            self.jump_prev = false;
            return Cmd {
                view_yaw: i.bearing,
                forward: MOVE_SPEED,
                side: 0.0,
                jump: false,
            };
        }
        self.ground_cmd(i, a_g, env, f32::INFINITY)
    }

    /// A prestrafe cmd, with sigma/flip bookkeeping. `band_cap` clamps the weave deadband — `∞` for
    /// the launch prestrafe (long runway), [`ZIGZAG_BAND_CAP`] for a tight zigzag corridor.
    fn ground_cmd(&mut self, i: &Input, a_g: f32, env: &Env, band_cap: f32) -> Cmd {
        self.jump_prev = false;
        let s = self.weave(prestrafe(
            i.v_xy,
            i.bearing,
            self.sigma,
            a_g,
            env.maxspeed,
            env.friction,
            env.stopspeed,
            env.dt,
            band_cap,
        ));
        Cmd {
            view_yaw: s.view_yaw,
            forward: s.forward,
            side: s.side,
            jump: false,
        }
    }

    /// Record a strafe's sign into the sticky state, counting flips for telemetry.
    fn weave(&mut self, s: Strafe) -> Strafe {
        if self.sigma != 0.0 && s.sigma != self.sigma {
            self.flips += 1;
        }
        self.sigma = s.sigma;
        s
    }

    fn engage(&mut self, i: &Input, maxspeed: f32, profile: HumanMovementProfile) {
        let speed = i.v_xy.length();
        // Prestrafe only from a genuine standing-ish start with runway to spare. If the planner
        // routed a carry here and we're already at speed, skip straight to the hop cycle — grounding
        // to prestrafe would bleed the carried speed to friction, the opposite of the intent.
        let hot_carry = i.carry && speed >= maxspeed;
        // A committed high-speed jump below its required takeoff speed prestrafes *regardless of the
        // 512 runway gate*: the ground circle-strafe outgains the air cap far below maxspeed, so it's
        // the only way to make `v_req` on a short (~450u) run-up. The `runway()` for such a leg is the
        // run-up to the takeoff edge, and the Prestrafe phase holds it to the lip (see [`Self::step`]).
        let sj_build = i.committed && i.takeoff_speed > 0.0 && speed < i.takeoff_speed;
        self.phase = if i.on_ground
            && !hot_carry
            && speed < profile.prestrafe_target
            && (i.runway > profile.prestrafe_min_runway || sj_build)
        {
            Phase::Prestrafe
        } else {
            Phase::Hop
        };
        self.phase_start = i.now;
        self.sigma = 0.0;
        self.jump_prev = false;
        self.air_start = i.now;
        self.air_flip_stage = 0;
        self.air_phase_locked = false;
        self.air_mid_phase = 0.5;
        self.hops = 0;
        self.flips = 0;
        self.peak = 0.0;
    }

    /// Enter a standalone ground zigzag. Same bookkeeping reset as [`Self::engage`]; the phase is
    /// held until a hop engages (`eligible`/`committed`) or the corridor bends (`!zigzag`).
    fn enter_zigzag(&mut self, i: &Input) {
        self.phase = Phase::Zigzag;
        self.phase_start = i.now;
        self.sigma = 0.0;
        self.jump_prev = false;
        self.air_start = i.now;
        self.air_flip_stage = 0;
        self.air_phase_locked = false;
        self.air_mid_phase = 0.5;
        self.hops = 0;
        self.flips = 0;
        self.peak = 0.0;
    }

    fn disengage(&mut self, reason: &'static str) {
        self.phase = Phase::Off;
        self.sigma = 0.0;
        self.jump_prev = false;
        self.air_flip_stage = 0;
        self.air_phase_locked = false;
        self.air_mid_phase = 0.5;
        self.eligible_since = 0.0;
        self.off_reason = reason;
    }
}

// The one-tick engine oracles (`apply_airaccel`/`apply_groundaccel`/`apply_friction`, `wishdir_fs`,
// `wishdir_of`) now live in `rtx_nav::strafe` (glob-re-exported above), shared with the offline pmove
// sim and the navmesh build's curl certifier so all three drive identical physics.

#[cfg(test)]
mod tests {
    use super::*;

    const ACCEL: f32 = 10.0;
    const MAXSPEED: f32 = 320.0;

    #[test]
    fn theta_star_regimes() {
        // Coarse-enough tick → a_max ≥ cap → perpendicular optimum.
        let a = air_accel_max(ACCEL, MAXSPEED, 1.0 / 77.0);
        assert!(a >= AIR_CAP);
        assert!((theta_star(400.0, a) - 90.0).abs() < 0.01);
        // Tiny a_max → optimum wish angle bends forward (< 90°), and shrinks as a_max grows.
        let t_small = theta_star(400.0, 5.0);
        let t_big = theta_star(400.0, 20.0);
        assert!(t_small < 90.0 && t_big < 90.0);
        assert!(t_big > t_small, "θ* increases toward 90° as a_max grows");
    }

    #[test]
    fn strafe_output_strictly_gains_speed() {
        for &s in &[100.0f32, 320.0, 500.0, 800.0, 1500.0] {
            for &dt in &[1.0 / 77.0, 1.0 / 30.0, 1.0 / 13.0] {
                let a = air_accel_max(ACCEL, MAXSPEED, dt);
                let v = Vec2::new(s, 0.0);
                let cmd = strafe(v, 0.0, 1.0, a);
                let wd = wishdir_of(cmd.view_yaw, cmd.side);
                let v2 = apply_airaccel(v, wd, MAXSPEED, ACCEL, dt);
                assert!(
                    v2.length() > v.length(),
                    "no gain at s={s} dt={dt}: {} -> {}",
                    v.length(),
                    v2.length()
                );
            }
        }
    }

    #[test]
    fn strafe_rate_turns_at_requested_rate_and_gains() {
        // The turn-rate dial: driven through the real air-accel oracle, `strafe_rate` should rotate
        // the velocity at ~`omega` (within 10%) while still growing the speed — the property the
        // smooth lobe leans on (a gentle turn that nonetheless accelerates).
        let dt = 1.0 / 77.0;
        let a = air_accel_max(ACCEL, MAXSPEED, dt);
        for &s in &[350.0f32, 500.0, 700.0] {
            let wmax = omega_gain_max(s, a, dt);
            for &omega in &[60.0f32, 100.0, 150.0] {
                if omega >= wmax {
                    continue; // beyond the gain-preserving ceiling — degenerates to max-gain
                }
                let v = Vec2::new(s, 0.0);
                let cmd = strafe_rate(v, 1.0, omega, a, dt);
                let v2 = apply_airaccel(v, wishdir_fs(cmd.view_yaw, cmd.forward, cmd.side), MAXSPEED, ACCEL, dt);
                let turned = yaw_of(v2);
                let want = omega * dt;
                assert!(
                    (turned - want).abs() <= 0.1 * want + 0.02,
                    "s={s} ω={omega}: turned {turned}° want {want}°"
                );
                assert!(v2.length() > s, "s={s} ω={omega}: no speed gain {s} -> {}", v2.length());
            }
        }
    }

    #[test]
    fn prestrafe_view_rides_arc() {
        // During the circle-jump arc (speed > u*) the eyes ride the velocity heading (the wind-up
        // into the takeoff), and the world wishdir is unchanged — still at `vel_yaw + sigma·θg`.
        let dt = 1.0 / 77.0;
        let a_g = air_accel_max(ACCEL, MAXSPEED, dt);
        let v = Vec2::new(400.0, 0.0); // vel_yaw = 0, above u* ≈ 278
        let s = prestrafe(v, 20.0, 1.0, a_g, MAXSPEED, 4.0, 100.0, dt, f32::INFINITY);
        assert!(
            s.view_yaw.abs() < 0.01,
            "view should ride the velocity (0°), got {}",
            s.view_yaw
        );
        let post_friction = apply_friction(v, 4.0, 100.0, dt).length();
        let theta_g = ((MAXSPEED - a_g).max(0.0) / post_friction)
            .clamp(0.0, 1.0)
            .acos()
            .to_degrees();
        let wd = wishdir_fs(s.view_yaw, s.forward, s.side);
        let wish_yaw = yaw_of(wd);
        assert!(
            (wish_yaw - s.sigma * theta_g).abs() < 0.5,
            "wishdir {wish_yaw}° should be sigma·θg {}°",
            s.sigma * theta_g
        );
        // Below u* it's a plain run-up: eyes on the bearing, no strafe.
        let s2 = prestrafe(
            Vec2::new(150.0, 0.0),
            20.0,
            1.0,
            a_g,
            MAXSPEED,
            4.0,
            100.0,
            dt,
            f32::INFINITY,
        );
        assert!(
            (s2.view_yaw - 20.0).abs() < 0.01 && s2.side == 0.0,
            "below u* should run straight at the bearing"
        );
    }

    #[test]
    fn prestrafe_is_optimal_after_ground_friction() {
        let dt = 1.0 / 77.0;
        let friction = 4.0;
        let stopspeed = 100.0;
        let a_g = air_accel_max(ACCEL, MAXSPEED, dt);
        let v = Vec2::new(475.0, 0.0);
        let v_accel = apply_friction(v, friction, stopspeed, dt);
        let s = prestrafe(v, 0.0, 1.0, a_g, MAXSPEED, friction, stopspeed, dt, f32::INFINITY);
        let wish = wishdir_fs(s.view_yaw, s.forward, s.side);
        let theta = v_accel.normalize().dot(wish).clamp(-1.0, 1.0).acos().to_degrees();
        let expected = ((MAXSPEED - a_g) / v_accel.length()).acos().to_degrees();
        assert!(
            (theta - expected).abs() < 0.01,
            "chosen {theta}°, post-friction optimum {expected}°"
        );

        let best = apply_groundaccel(v_accel, wish, MAXSPEED, ACCEL, dt).length();
        let wish_yaw = yaw_of(wish);
        for offset in [-5.0f32, -2.0, -1.0, 1.0, 2.0, 5.0] {
            let radians = (wish_yaw + offset).to_radians();
            let candidate = Vec2::new(radians.cos(), radians.sin());
            let gained = apply_groundaccel(v_accel, candidate, MAXSPEED, ACCEL, dt).length();
            assert!(
                best + 1e-4 >= gained,
                "offset {offset}° beat post-friction optimum: {gained} > {best}"
            );
        }
    }

    #[test]
    fn chosen_angle_beats_offsets() {
        // The controller's yaw should give at least as much gain as small offsets from it.
        let dt = 1.0 / 77.0;
        let a = air_accel_max(ACCEL, MAXSPEED, dt);
        let v = Vec2::new(500.0, 0.0);
        let cmd = strafe(v, 0.0, 1.0, a);
        let best = apply_airaccel(v, wishdir_of(cmd.view_yaw, cmd.side), MAXSPEED, ACCEL, dt).length();
        for off in [-10.0f32, -5.0, -2.0, 2.0, 5.0, 10.0] {
            let g = apply_airaccel(v, wishdir_of(cmd.view_yaw + off, cmd.side), MAXSPEED, ACCEL, dt).length();
            assert!(best + 1e-3 >= g, "offset {off} beat the chosen angle ({g} > {best})");
        }
    }

    /// Circular mean (degrees) of the headings of a velocity trace's tail.
    fn mean_heading(tail: &[Vec2]) -> f32 {
        let sum: Vec2 = tail.iter().map(|v| v.normalize_or_zero()).sum();
        yaw_of(sum)
    }

    #[test]
    fn ramps_far_past_maxspeed_and_tracks_bearing() {
        let dt = 1.0 / 77.0;
        let a = air_accel_max(ACCEL, MAXSPEED, dt);
        let mut v = Vec2::new(MAXSPEED, 0.0); // first hop, along +x; bearing also +x (0°)
        let mut sigma = 0.0;
        let mut flips = 0;
        let mut trace = Vec::new();
        for _ in 0..385 {
            let cmd = strafe(v, 0.0, sigma, a);
            if sigma != 0.0 && cmd.sigma != sigma {
                flips += 1;
            }
            sigma = cmd.sigma;
            v = apply_airaccel(v, wishdir_of(cmd.view_yaw, cmd.side), MAXSPEED, ACCEL, dt);
            trace.push(v);
        }
        assert!(v.length() > 600.0, "only reached {} ups over 5s", v.length());
        // The weave swings the instantaneous heading through ±weave_band; judge the *mean*.
        let heading = mean_heading(&trace[trace.len() - 100..]);
        assert!(heading.abs() < 12.0, "mean heading drifted to {heading}");
        assert!(flips > 5, "the weave should flip the strafe sign repeatedly ({flips})");
    }

    #[test]
    fn tracks_an_offset_bearing() {
        // Velocity along +x, but the waypoint is 30° to the left → heading should converge there.
        let dt = 1.0 / 77.0;
        let a = air_accel_max(ACCEL, MAXSPEED, dt);
        let mut v = Vec2::new(MAXSPEED, 0.0);
        let mut sigma = 0.0;
        let mut trace = Vec::new();
        for _ in 0..300 {
            let cmd = strafe(v, 30.0, sigma, a);
            sigma = cmd.sigma;
            v = apply_airaccel(v, wishdir_of(cmd.view_yaw, cmd.side), MAXSPEED, ACCEL, dt);
            trace.push(v);
        }
        let heading = mean_heading(&trace[trace.len() - 100..]);
        assert!((heading - 30.0).abs() < 12.0, "did not converge to 30°: {heading}");
    }

    /// The navmesh's derated speed-jump model (`rtx_nav::navmesh`, planned offline) must be
    /// *conservative* against this actual controller: simulate a real air-strafe over an 800u runway
    /// and confirm the controller reaches at least the speed the planner credited. Lives here rather
    /// than in `rtx-nav` because it exercises the controller sim, which is defined in this crate.
    #[test]
    fn navmesh_speed_jump_model_is_conservative() {
        use rtx_nav::navmesh::{attainable_speed, bhop_k, BHOP_EFF, MAX_SPEED};
        let k = bhop_k(ACCEL, MAX_SPEED);
        let dt = 1.0 / 72.0; // the planner's conservative model tickrate
        let a_max = air_accel_max(ACCEL, MAX_SPEED, dt);
        let steps = (800.0 / (MAX_SPEED * dt)) as i32; // ~time to cover the runway, air frames only
        let mut vel = Vec2::new(MAX_SPEED, 0.0);
        let mut sigma = 0.0;
        for _ in 0..steps {
            let s = strafe(vel, 0.0, sigma, a_max);
            sigma = s.sigma;
            vel = apply_airaccel(vel, wishdir_of(s.view_yaw, s.side), MAX_SPEED, ACCEL, dt);
        }
        let planned = BHOP_EFF * attainable_speed(MAX_SPEED, 800.0, k);
        assert!(
            vel.length() >= planned,
            "controller {} slower than planned {planned}",
            vel.length()
        );
    }
}

/// Full hop-cycle simulation: [`Bhop::step`] driven against a pmove oracle that mirrors FTEQW's
/// frame order — `PM_CheckJump` **before** `PM_Friction`, then ground/air accel, then gravity.
/// This is the harness that would have caught the original integration bugs: it exercises
/// landings, takeoffs, prestrafe, and the engage/disengage policy, not just the pure air math.
#[cfg(test)]
mod sim {
    use super::*;

    const DT: f32 = 1.0 / 77.0;
    const ENV: Env = Env {
        dt: DT,
        accel: 10.0,
        maxspeed: 320.0,
        friction: FRICTION,
        stopspeed: STOPSPEED,
        profile: HumanMovementProfile::calibrated(),
    };
    const JUMP_VZ: f32 = 270.0;
    const GRAVITY: f32 = 800.0;
    const FRICTION: f32 = 4.0;
    const STOPSPEED: f32 = 100.0;

    struct World {
        pos: Vec2,
        z: f32,
        vz: f32,
        v: Vec2,
        on_ground: bool,
        /// QW `pmove.jump_held`: set by an actual jump, cleared only by a released button.
        jump_held: bool,
    }

    impl World {
        fn grounded(speed: f32) -> Self {
            World {
                pos: Vec2::ZERO,
                z: 0.0,
                vz: 0.0,
                v: Vec2::new(speed, 0.0),
                on_ground: true,
                jump_held: false,
            }
        }
    }

    /// One engine frame in FTEQW order. `deny_jump` swallows the jump (models a press the engine
    /// refuses) to exercise the controller's pulse guard.
    fn pm_frame(w: &mut World, cmd: &Cmd, deny_jump: bool) {
        // PM_CheckJump — before friction: a landing-frame jump skips ground friction entirely.
        if !cmd.jump {
            w.jump_held = false;
        } else if w.on_ground && !w.jump_held && !deny_jump {
            w.on_ground = false;
            w.vz = JUMP_VZ;
            w.jump_held = true;
        }
        if w.on_ground {
            w.v = apply_friction(w.v, FRICTION, STOPSPEED, DT);
        }
        let wishdir = wishdir_fs(cmd.view_yaw, cmd.forward, cmd.side);
        let wishspeed = Vec2::new(cmd.forward, cmd.side).length().min(ENV.maxspeed);
        if w.on_ground {
            w.v = apply_groundaccel(w.v, wishdir, wishspeed, ENV.accel, DT);
        } else {
            w.v = apply_airaccel(w.v, wishdir, wishspeed, ENV.accel, DT);
            w.vz -= GRAVITY * DT;
            w.z += w.vz * DT;
            if w.z <= 0.0 && w.vz <= 0.0 {
                w.z = 0.0;
                w.vz = 0.0;
                w.on_ground = true;
            }
        }
        w.pos += w.v * DT;
    }

    fn input(w: &World, bearing: f32, runway: f32, now: f32) -> Input {
        Input {
            v_xy: w.v,
            on_ground: w.on_ground,
            bearing,
            runway,
            eligible: true,
            zigzag: false,
            sustain: true,
            veto: false,
            committed: false,
            carry: false,
            hold_jump: false,
            takeoff_speed: 0.0,
            curl_gain: 0.0,
            clear: f32::INFINITY,
            now,
        }
    }

    /// While the controller is Off, the bot runs at the bearing through the normal steering path.
    fn run_cmd(bearing: f32) -> Cmd {
        Cmd {
            view_yaw: bearing,
            forward: MOVE_SPEED,
            side: 0.0,
            jump: false,
        }
    }

    /// An input with `eligible`/`zigzag`/`on_ground` under test control (the default `input` fixes
    /// `eligible = true`, which would hop immediately and never exercise the zigzag phase).
    #[allow(clippy::too_many_arguments)]
    fn zz_input(v: Vec2, on_ground: bool, bearing: f32, runway: f32, eligible: bool, zigzag: bool, now: f32) -> Input {
        Input {
            v_xy: v,
            on_ground,
            bearing,
            runway,
            eligible,
            zigzag,
            sustain: true,
            veto: false,
            committed: false,
            carry: false,
            hold_jump: false,
            takeoff_speed: 0.0,
            curl_gain: 0.0,
            clear: f32::INFINITY,
            now,
        }
    }

    fn mean_heading(tail: &[Vec2]) -> f32 {
        let sum: Vec2 = tail.iter().map(|v| v.normalize_or_zero()).sum();
        yaw_of(sum)
    }

    /// Run the normal engagement policy from rest down a straight, finite corridor. This keeps the
    /// regression model tied to the same runway consumption and launch gates as the live 100m test.
    fn directed_flat_run(distance: f32) -> (f32, f32, f32) {
        let mut w = World::grounded(0.0);
        let mut b = Bhop::default();
        let mut now = 0.0;
        let mut peak = 0.0f32;
        let mut max_side = 0.0f32;
        while w.pos.x < distance && now < 15.0 {
            let runway = (distance - w.pos.x).max(0.0);
            let eligible = w.v.length() >= RUN_UP_SPEED && runway >= RUNWAY_ENGAGE;
            let i = Input {
                eligible,
                sustain: runway > 24.0,
                runway,
                ..input(&w, 0.0, runway, now)
            };
            let cmd = b.step(&i, &ENV).unwrap_or(run_cmd(0.0));
            pm_frame(&mut w, &cmd, false);
            peak = peak.max(w.v.length());
            max_side = max_side.max(w.pos.y.abs());
            now += DT;
        }
        (now, peak, max_side)
    }

    #[test]
    fn corpus_max_gain_profile_is_fast_and_corridor_bounded() {
        // The exact user-directed leg must stay in its narrow route even though it is too short for
        // 800 ups. The full map runway should approach that speed in this conservative oracle; the
        // release build is accepted separately against live PM_PlayerMove at an 800-ups floor.
        let (short_time, short_peak, short_side) = directed_flat_run(1552.0);
        assert!(short_peak >= 600.0, "short route peaked at only {short_peak} ups");
        assert!(short_side <= 64.0, "short route drifted {short_side} units");
        assert!(short_time <= 3.6, "short route took {short_time}s");

        let (long_time, long_peak, long_side) = directed_flat_run(4392.0);
        assert!(long_peak >= 795.0, "long runway peaked at only {long_peak} ups");
        assert!(long_side <= 96.0, "long runway drifted {long_side} units");
        assert!(long_time <= 7.5, "long runway took {long_time}s");
    }

    #[test]
    fn full_run_ramps_320_to_550() {
        let mut w = World::grounded(320.0);
        let mut b = Bhop::default();
        let mut launch_speed = None;
        let mut trace = Vec::new();
        for f in 0..770 {
            let now = f as f32 * DT;
            let phase_was = b.phase;
            let cmd = b.step(&input(&w, 0.0, 4096.0, now), &ENV).unwrap_or(run_cmd(0.0));
            if phase_was == Phase::Prestrafe && b.phase == Phase::Hop {
                launch_speed = Some(w.v.length());
            }
            pm_frame(&mut w, &cmd, false);
            trace.push(w.v);
        }
        let launch = launch_speed.expect("never launched from prestrafe");
        assert!(launch >= 420.0, "launched at only {launch} ups");
        assert!(
            w.v.length() >= 550.0,
            "only {} ups after 10s (peak {})",
            w.v.length(),
            b.peak
        );
        // Average over a couple of weave periods: the slalom lobe is wide (±LOBE_DEADBAND), so a
        // short window catches a lobe peak, not the centered mean.
        let heading = mean_heading(&trace[trace.len() - 154..]);
        assert!(heading.abs() < 8.0, "mean heading drifted to {heading}");
        assert!(b.hops >= 3, "only {} hops in 10s", b.hops);
        // Launch hops retain their compact heading-error weave; once the corpus-backed 600-ups
        // regime begins, each complete hop contributes exactly three direction changes.
        let flips_per_hop = b.flips as f32 / b.hops as f32;
        assert!(
            (1.5..=3.2).contains(&flips_per_hop),
            "{} flips over {} hops (want bounded launch + three-reversal cruise)",
            b.flips,
            b.hops
        );
    }

    #[test]
    fn high_speed_hop_has_exactly_three_phase_locked_reversals() {
        for speed in [600.0, 800.0] {
            let mut w = World::grounded(speed);
            let mut b = Bhop::default();
            let mut was_airborne = false;
            for frame in 0..100 {
                let now = frame as f32 * DT;
                let cmd = b
                    .step(&input(&w, 0.0, 4096.0, now), &ENV)
                    .expect("high-speed hop owns frame");
                pm_frame(&mut w, &cmd, false);
                was_airborne |= !w.on_ground;
                if was_airborne && w.on_ground {
                    break;
                }
            }
            assert!(was_airborne && w.on_ground, "{speed} ups hop never completed");
            assert_eq!(b.hops, 1, "{speed} ups test should stop on its first landing");
            assert_eq!(b.flips, 3, "{speed} ups hop did not use the three-reversal gait");
        }
    }

    #[test]
    fn never_leaps_below_full_run() {
        // A human runs up before leaping. From a near-standstill, no takeoff may happen below the
        // launch floor (full maxspeed) on a long runway (via Prestrafe), a medium one, or a short one
        // (the 256–512u direct-to-Hop band Daniel saw hopping slow) — the bot builds speed on the
        // ground to ≥ maxspeed first, then leaps.
        let floor = ENV.profile.launch_min_frac * ENV.maxspeed;
        for &(runway, min_takeoff) in &[(4096.0f32, 420.0f32), (400.0, 319.0), (300.0, 319.0)] {
            let mut w = World::grounded(100.0);
            let mut b = Bhop::default();
            let mut first_jump = None;
            for f in 0..770 {
                let now = f as f32 * DT;
                let cmd = b.step(&input(&w, 0.0, runway, now), &ENV).unwrap_or(run_cmd(0.0));
                if cmd.jump {
                    assert!(
                        w.v.length() >= floor - 1.0,
                        "leaped at {} ups (runway {runway})",
                        w.v.length()
                    );
                    first_jump.get_or_insert(w.v.length());
                }
                pm_frame(&mut w, &cmd, false);
            }
            let fj = first_jump.expect("never took off");
            assert!(
                fj >= min_takeoff,
                "first takeoff at {fj} ups (runway {runway}), want ≥ {min_takeoff}"
            );
        }
    }

    #[test]
    fn committed_high_speed_jump_builds_to_the_lip() {
        // The takeoff regime: a committed speed jump needing v_req = 415 on a SHORT (~470u) runway
        // holds the ground circle-strafe all the way to the lip and leaps near v_req — not the eager
        // 304 the plain short-runway direct-to-Hop would take (that's what left the bravado LG jump
        // short). The runway shrinks as the bot advances; it must leap within a lip-reach of the edge.
        const V_REQ: f32 = 415.0;
        let start_runway = 470.0f32;
        let mut w = World::grounded(320.0); // arrives at run speed off the corridor
        let mut b = Bhop::default();
        let mut takeoff = None;
        for f in 0..300 {
            let now = f as f32 * DT;
            let runway = (start_runway - w.pos.x).max(0.0);
            let input = Input {
                v_xy: w.v,
                on_ground: w.on_ground,
                bearing: 0.0,
                runway,
                eligible: false,
                zigzag: false,
                sustain: false,
                veto: false,
                committed: true,
                carry: false,
                hold_jump: false,
                takeoff_speed: V_REQ,
                curl_gain: 0.0,
                clear: f32::INFINITY,
                now,
            };
            let cmd = b.step(&input, &ENV).unwrap_or(run_cmd(0.0));
            if cmd.jump && takeoff.is_none() {
                takeoff = Some((w.v.length(), runway));
            }
            pm_frame(&mut w, &cmd, false);
        }
        let (spd, rw) = takeoff.expect("never took off");
        assert!(
            spd >= 400.0,
            "took off at {spd} ups on the short runway, want ≥ 400 (build to ~v_req)"
        );
        assert!(rw < LIP_REACH + 8.0, "leaped {rw}u short of the lip, want at the edge");
    }

    #[test]
    fn curl_takeoff_holds_the_solved_speed() {
        // The takeoff regime holds the link's *solved* takeoff speed rather than maxing to the ~484
        // prestrafe equilibrium (whose reach overshoots any moderate gap — a human holds ~400 too, as
        // every recorded demo does). From a slow entry it must build up into the band; from a hot
        // bhop-carry entry it must coast down into it. Either way it leaps at the lip, in-band — which
        // is exactly the speed envelope the certifier proves the landing across.
        const V_STAR: f32 = 400.0;
        for entry in [320.0f32, 480.0] {
            let start_runway = 480.0f32;
            let mut w = World::grounded(entry);
            let mut b = Bhop::default();
            let mut takeoff = None;
            for f in 0..400 {
                let now = f as f32 * DT;
                let runway = start_runway - w.pos.x; // signed: negative past the lip
                let input = Input {
                    v_xy: w.v,
                    on_ground: w.on_ground,
                    bearing: 0.0,
                    runway,
                    eligible: false,
                    zigzag: false,
                    sustain: false,
                    veto: false,
                    committed: true,
                    carry: false,
                    hold_jump: false,
                    takeoff_speed: V_STAR,
                    curl_gain: 12.0,
                    clear: f32::INFINITY,
                    now,
                };
                let cmd = b.step(&input, &ENV).unwrap_or(run_cmd(0.0));
                if cmd.jump && takeoff.is_none() {
                    takeoff = Some((w.v.length(), runway));
                }
                pm_frame(&mut w, &cmd, false);
            }
            let (spd, rw) = takeoff.expect("never took off");
            let band = V_STAR * CURL_V_HOLD_TOL * 2.0; // the hold ripple may span the band either side
            assert!(
                (spd - V_STAR).abs() <= band,
                "entry {entry}: took off at {spd} ups, want {V_STAR} ±{band}"
            );
            assert!(rw < LIP_REACH + 8.0, "entry {entry}: leaped {rw}u short of the lip");
        }
    }

    #[test]
    fn curl_past_lip_leaps_immediately() {
        // Signed runway: a committed curl already past the takeoff line (negative runway — the weave
        // skirted the radial ball, or a repath dropped it past the lip) must leap at once, not hold
        // ground and U-turn back toward the takeoff. (Whether it *should* leap that slow is steer's
        // abort call; the controller's job is: past the lip → commit, never reverse.)
        let mut w = World::grounded(450.0);
        let mut b = Bhop::default();
        let mut leaped = false;
        for f in 0..20 {
            let now = f as f32 * DT;
            let input = Input {
                v_xy: w.v,
                on_ground: w.on_ground,
                bearing: 0.0,
                runway: -20.0, // past the lip
                eligible: false,
                zigzag: false,
                sustain: false,
                veto: false,
                committed: true,
                carry: false,
                hold_jump: false,
                takeoff_speed: 415.0,
                curl_gain: 12.0,
                clear: f32::INFINITY,
                now,
            };
            let cmd = b.step(&input, &ENV).unwrap_or(run_cmd(0.0));
            if cmd.jump {
                leaped = true;
                break;
            }
            pm_frame(&mut w, &cmd, false);
        }
        assert!(
            leaped,
            "a committed curl past the lip (negative runway) must leap, not hold"
        );
    }

    #[test]
    fn slow_landing_regrounds_and_rebuilds() {
        // Hopping fast, then a bump drops us to a slow grounded state mid-chain: the controller must
        // not leap slow — it circle-strafes on the ground until it's rebuilt past the launch floor.
        let mut w = World::grounded(460.0);
        let mut b = Bhop::default();
        for f in 0..40 {
            let now = f as f32 * DT;
            let cmd = b.step(&input(&w, 0.0, 4096.0, now), &ENV).unwrap_or(run_cmd(0.0));
            pm_frame(&mut w, &cmd, false);
        }
        assert!(b.hops >= 1, "never got hopping");
        // Bump: force a slow grounded state.
        w.v = Vec2::new(250.0, 0.0);
        w.z = 0.0;
        w.vz = 0.0;
        w.on_ground = true;
        w.jump_held = false;
        let cmd = b
            .step(&input(&w, 0.0, 4096.0, 40.0 * DT), &ENV)
            .expect("stays engaged after the bump");
        assert!(!cmd.jump, "leaped at 250 ups after a bump");
        // Drive forward: it rebuilds on the ground and only re-jumps past the floor.
        let mut rejump = None;
        for f in 41..160 {
            let now = f as f32 * DT;
            let cmd = b.step(&input(&w, 0.0, 4096.0, now), &ENV).unwrap_or(run_cmd(0.0));
            if cmd.jump {
                rejump.get_or_insert(w.v.length());
            }
            pm_frame(&mut w, &cmd, false);
        }
        let rj = rejump.expect("never re-jumped");
        assert!(
            rj >= 0.95 * ENV.maxspeed - 1.0,
            "re-jumped at {rj} ups (below the floor)"
        );
    }

    #[test]
    fn wall_ahead_holds_the_leap() {
        // Grounded, fast, but a wall is close ahead (`clear` short): don't leap into it — keep
        // circle-strafing on the ground (which turns far faster than air) instead of face-planting.
        let mut w = World::grounded(460.0);
        let mut b = Bhop::default();
        for f in 0..30 {
            let now = f as f32 * DT;
            let cmd = b.step(&input(&w, 0.0, 4096.0, now), &ENV).unwrap_or(run_cmd(0.0));
            pm_frame(&mut w, &cmd, false);
        }
        w.v = Vec2::new(460.0, 0.0);
        w.z = 0.0;
        w.vz = 0.0;
        w.on_ground = true;
        w.jump_held = false;
        let mut inp = input(&w, 0.0, 4096.0, 30.0 * DT);
        inp.clear = 50.0; // wall 50u ahead; a hop's flight would be ~310u
        assert!(
            !b.step(&inp, &ENV).expect("engaged").jump,
            "leaped toward a wall 50u away"
        );
    }

    #[test]
    fn wall_panic_carves_at_max_rate() {
        // Airborne, flying at a near wall, heading 60° off the bearing: drop the slalom and carve
        // toward the bearing at the full physical turn rate (not the gentle lobe rate).
        let a = air_accel_max(ENV.accel, ENV.maxspeed, DT);
        let w = World {
            pos: Vec2::ZERO,
            z: 60.0,
            vz: 0.0,
            v: Vec2::new(450.0, 0.0),
            on_ground: false,
            jump_held: true,
        };
        let mut inp = input(&w, 60.0, 4096.0, 0.0); // bearing +60° from the +x velocity
        inp.committed = true; // engage immediately, no hysteresis
        inp.clear = 30.0; // wall within WALL_PANIC_T (450·0.3 = 135u)
        let mut b = Bhop::default();
        let cmd = b.step(&inp, &ENV).expect("airborne engaged");
        let wd = wishdir_fs(cmd.view_yaw, cmd.forward, cmd.side);
        let v2 = apply_airaccel(w.v, wd, ENV.maxspeed, ENV.accel, DT);
        let turned = yaw_of(v2);
        let wmax = (a / 450.0).asin().to_degrees();
        let gain_turn = omega_gain_max(450.0, a, DT) * DT;
        assert!(turned > 0.0, "should carve toward the +60° bearing, turned {turned}°");
        assert!((turned - wmax).abs() < 0.02, "carve {turned}° should be ~max {wmax}°");
        assert!(
            turned > gain_turn * 1.3,
            "emergency {turned}° barely beat max-gain {gain_turn}°"
        );
        assert!(
            v2.length() > w.v.length() * 0.99,
            "emergency carve discarded too much speed"
        );
    }

    #[test]
    fn curved_chain_gains_like_the_demo() {
        // The demo bar (bridge_rl): carry ~446 ups into a chain of hops while the corridor bends
        // ~80°, and come out faster — the human reached 468 over four such hops. The bot must gain
        // at least that while retaining a bounded max-gain weave and a human-range velocity sweep.
        let mut w = World::grounded(446.0);
        let mut b = Bhop::default();
        let mut air_rates = Vec::new();
        let mut prev_hd = yaw_of(w.v);
        for f in 0..270 {
            let now = f as f32 * DT;
            let bearing = (now / 2.7 * 80.0).min(80.0); // 80° over ~4 hops
            let was_air = !w.on_ground;
            let cmd = b
                .step(&input(&w, bearing, 4096.0, now), &ENV)
                .unwrap_or(run_cmd(bearing));
            pm_frame(&mut w, &cmd, false);
            let hd = yaw_of(w.v);
            if was_air && !w.on_ground {
                air_rates.push(wrap180(hd - prev_hd).abs() / DT);
            }
            prev_hd = hd;
        }
        assert!(
            w.v.length() >= 468.0,
            "curved chain gained too little: {} ups",
            w.v.length()
        );
        assert!(b.hops >= 4, "only {} hops in the chain", b.hops);
        let flips_per_hop = b.flips as f32 / b.hops as f32;
        assert!(flips_per_hop <= 3.2, "{flips_per_hop} flips/hop — unbounded weave");
        air_rates.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let median = air_rates[air_rates.len() / 2];
        assert!(
            (80.0..=280.0).contains(&median),
            "median air-strafe yaw rate {median} °/s off human band"
        );
    }

    #[test]
    fn tracks_a_30_degree_bend() {
        let mut w = World::grounded(450.0); // past the prestrafe target → engages straight to Hop
        let mut b = Bhop::default();
        let mut trace = Vec::new();
        for f in 0..539 {
            let now = f as f32 * DT;
            let bearing = if now < 3.0 { 0.0 } else { 30.0 };
            let cmd = b
                .step(&input(&w, bearing, 4096.0, now), &ENV)
                .unwrap_or(run_cmd(bearing));
            pm_frame(&mut w, &cmd, false);
            trace.push(w.v);
        }
        // 3s at bearing 0, then 2s at 30°: the last ~1.7s (a couple of the wide slalom lobes)
        // should average onto the new bearing. A shorter window catches a single lobe's peak.
        let heading = mean_heading(&trace[trace.len() - 130..]);
        assert!((heading - 30.0).abs() < 10.0, "did not converge to 30°: {heading}");
    }

    #[test]
    fn landing_frames_never_lose_speed() {
        let mut w = World::grounded(450.0);
        let mut b = Bhop::default();
        for f in 0..770 {
            let now = f as f32 * DT;
            let grounded_in_hop = w.on_ground && b.phase == Phase::Hop;
            let before = w.v.length();
            let cmd = b.step(&input(&w, 0.0, 4096.0, now), &ENV).unwrap_or(run_cmd(0.0));
            pm_frame(&mut w, &cmd, false);
            if grounded_in_hop {
                // CheckJump-before-Friction: the landing frame jumps and takes air accel — never
                // a friction frame. (The old controller ran plain-forward here: zero gain.)
                assert!(
                    w.v.length() >= before - 0.01,
                    "landing frame lost speed at t={now}: {before} -> {}",
                    w.v.length()
                );
            }
        }
        assert!(b.hops >= 5, "hop cycle never got going ({} hops)", b.hops);
    }

    #[test]
    fn denied_jump_pulses_and_recovers() {
        let mut w = World::grounded(450.0);
        let mut b = Bhop::default();
        let mut denied_at = None;
        for f in 0..770 {
            let now = f as f32 * DT;
            let cmd = b.step(&input(&w, 0.0, 4096.0, now), &ENV).unwrap_or(run_cmd(0.0));
            // Swallow the very first landing-frame press; the pulse guard must release and retry.
            let deny = cmd.jump && denied_at.is_none() && {
                denied_at = Some(f);
                true
            };
            pm_frame(&mut w, &cmd, deny);
            if let Some(d) = denied_at {
                if !w.on_ground {
                    assert!(f - d <= 3, "took {} frames to recover from a denied jump", f - d);
                    return;
                }
            }
        }
        panic!("never airborne after the denied jump");
    }

    #[test]
    fn disengages_only_on_ground() {
        let mut w = World::grounded(450.0);
        let mut b = Bhop::default();
        for f in 0..1540 {
            let now = f as f32 * DT;
            let runway = 2048.0 - w.pos.length(); // the straightaway runs out as we cover it
            let was_on = b.phase != Phase::Off;
            match b.step(&input(&w, 0.0, runway.max(0.0), now), &ENV) {
                Some(cmd) => pm_frame(&mut w, &cmd, false),
                None => {
                    if was_on {
                        assert!(w.on_ground, "disengaged mid-air at t={now}");
                        assert_eq!(b.off_reason, "runway");
                        assert!(b.hops >= 1, "never hopped before the runway ran out");
                        return;
                    }
                    pm_frame(&mut w, &run_cmd(0.0), false);
                }
            }
        }
        panic!("never disengaged on the shrinking runway");
    }

    #[test]
    fn prestrafe_beats_maxspeed() {
        // Pure ground circle-strafe from maxspeed: past 440 within a second, equilibrium < 520.
        let mut v = Vec2::new(320.0, 0.0);
        let mut sigma = 0.0;
        let a_g = air_accel_max(ENV.accel, ENV.maxspeed, DT);
        let mut at_1s = 0.0;
        for f in 0..231 {
            let s = prestrafe(
                v,
                0.0,
                sigma,
                a_g,
                ENV.maxspeed,
                ENV.friction,
                ENV.stopspeed,
                ENV.dt,
                f32::INFINITY,
            );
            sigma = s.sigma;
            v = apply_friction(v, FRICTION, STOPSPEED, DT);
            let wishdir = wishdir_fs(s.view_yaw, s.forward, s.side);
            let wishspeed = Vec2::new(s.forward, s.side).length().min(ENV.maxspeed);
            v = apply_groundaccel(v, wishdir, wishspeed, ENV.accel, DT);
            if f == 76 {
                at_1s = v.length();
            }
        }
        assert!(at_1s >= 440.0, "only {at_1s} ups after 1s of prestrafe");
        assert!(
            v.length() < 520.0,
            "prestrafe equilibrium implausibly high: {}",
            v.length()
        );
    }

    #[test]
    fn zigzag_gains_past_maxspeed_in_short_corridor() {
        // A corridor too short to hop (runway < RUNWAY_ENGAGE) but zigzag-eligible: the ground
        // circle-strafe alone should push well past maxspeed while staying inside the corridor.
        let mut w = World::grounded(320.0);
        let mut b = Bhop::default();
        let mut max_lat = 0.0f32;
        for f in 0..116 {
            // ~1.5s
            let now = f as f32 * DT;
            let cmd = b
                .step(&zz_input(w.v, w.on_ground, 0.0, 180.0, false, true, now), &ENV)
                .expect("zigzag should own the frame");
            assert_eq!(b.phase, Phase::Zigzag, "left the zigzag phase at t={now}");
            pm_frame(&mut w, &cmd, false);
            max_lat = max_lat.max(w.pos.y.abs());
        }
        assert!(
            w.v.length() >= 430.0,
            "zigzag only reached {} ups in 1.5s",
            w.v.length()
        );
        assert!(
            max_lat <= 96.0,
            "zigzag swept {max_lat}u off the corridor centerline (band cap failed)"
        );
    }

    #[test]
    fn zigzag_hands_off_to_hop() {
        // Start on a short corridor (zigzag only); after 1s the runway opens and the hop cycle
        // takes over. The transition must go Zigzag -> Prestrafe/Hop directly — never through Off —
        // and must not shed the speed the zigzag built.
        let mut w = World::grounded(320.0);
        let mut b = Bhop::default();
        let mut saw_zigzag = false;
        let mut engaged_yet = false;
        for f in 0..770 {
            let now = f as f32 * DT;
            let eligible = now >= 1.0;
            let runway = if eligible { 4096.0 } else { 180.0 };
            let cmd = b
                .step(&zz_input(w.v, w.on_ground, 0.0, runway, eligible, true, now), &ENV)
                .expect("controller engaged the whole run");
            if b.phase == Phase::Zigzag {
                saw_zigzag = true;
            }
            if b.phase != Phase::Off {
                engaged_yet = true;
            }
            assert!(!(engaged_yet && b.phase == Phase::Off), "fell back to Off at t={now}");
            pm_frame(&mut w, &cmd, false);
        }
        assert!(saw_zigzag, "never entered the zigzag phase");
        assert_eq!(b.phase, Phase::Hop, "never handed off to the hop cycle: {:?}", b.phase);
        assert!(
            w.v.length() >= 430.0,
            "lost speed across the handoff: {} ups",
            w.v.length()
        );
    }

    #[test]
    fn zigzag_exits_on_bend() {
        // Enter the zigzag, then the corridor bends (`runway()` returns `zigzag = false`): the next
        // ground frame disengages cleanly rather than fighting the corner.
        let mut b = Bhop::default();
        let v = Vec2::new(400.0, 0.0);
        let engaged = b.step(&zz_input(v, true, 0.0, 180.0, false, true, 0.0), &ENV);
        assert!(engaged.is_some() && b.phase == Phase::Zigzag);
        let bent = b.step(&zz_input(v, true, 0.0, 180.0, false, false, DT), &ENV);
        assert!(bent.is_none(), "kept strafing past the bend");
        assert_eq!(b.phase, Phase::Off);
        assert_eq!(b.off_reason, "zigzag");
    }

    #[test]
    fn zigzag_tolerates_air_frames() {
        // A Step leg can yield a frame or two airborne; the zigzag must stay engaged (holding the
        // bearing) rather than disengaging or applying ground math mid-air.
        let mut b = Bhop::default();
        let v = Vec2::new(400.0, 0.0);
        assert!(b.step(&zz_input(v, true, 0.0, 180.0, false, true, 0.0), &ENV).is_some());
        assert_eq!(b.phase, Phase::Zigzag);
        for f in 1..=2 {
            let now = f as f32 * DT;
            let cmd = b
                .step(&zz_input(v, false, 0.0, 180.0, false, true, now), &ENV)
                .expect("stayed engaged through the air frame");
            assert_eq!(b.phase, Phase::Zigzag, "disengaged on an air frame");
            assert!(!cmd.jump && cmd.side == 0.0, "air frame should be a plain bearing-run");
        }
    }
}
