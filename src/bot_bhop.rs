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

use crate::bot::wrap180;

/// The projected-wishspeed cap in QW air acceleration (`PM_AirAccelerate`) — an engine literal
/// (`movevars_maxairspeed`), not a cvar. Only this much of the wish speed counts against the
/// current velocity each tick, which is exactly what lets a perpendicular strafe keep gaining.
const AIR_CAP: f32 = 30.0;
/// Usercmd move scale (as in `bot.rs`; pmove clamps wish speed to `sv_maxspeed`).
const MOVE_SPEED: f32 = 800.0;

/// The engine's fixed bot tick (bots run `SV_RunCmd` at ~77 Hz regardless of what we pass as
/// `msec`), used only to size the weave band. The live accel math uses the real per-frame `dt`.
const DT_NOMINAL: f32 = 1.0 / 77.0;
/// Flat-ground hop airtime: `2 · JUMP_VZ / gravity` = 2·270/800 (see `navmesh`).
const T_HOP: f32 = 0.675;
/// How many times per hop the serpentine should switch sides. ~3 matches how human runners weave.
const FLIPS_PER_HOP: f32 = 3.0;

/// How long the entry conditions must hold before engaging, so a momentary straightaway doesn't
/// stutter the gait. Applies only to the initial engage; disengage decisions happen on landings.
/// Kept short — the runway is *consumed* while waiting (~32u per 0.1s at walk speed), so a long
/// delay quietly raises the effective entry bar well past [`RUNWAY_ENGAGE`].
const ENGAGE_DELAY: f32 = 0.15;
/// Prestrafe launches into the first hop at this speed — just under the ground-friction
/// equilibrium (~490), so it's reachable quickly and the jump leaves before gains flatten.
const PRESTRAFE_TARGET: f32 = 450.0;
/// Give up circling and just jump if the target speed hasn't arrived by then (shoved, uphill…).
const PRESTRAFE_MAX_T: f32 = 1.2;
/// Only bother prestrafing with this much corridor ahead; shorter → hop immediately.
const PRESTRAFE_MIN_RUNWAY: f32 = 512.0;
/// Fixed runway required to engage — enough for one worthwhile hop (a flat hop at walk speed
/// covers ~216u). Deliberately *not* speed-scaled: the old `speed·0.9` bar rose as the bot gained
/// speed and disengaged it mid-run — the policy capped the very thing it built.
pub const RUNWAY_ENGAGE: f32 = 256.0;
/// Slack beyond the current hop's flight distance when deciding whether another hop fits.
const HOP_MARGIN: f32 = 64.0;

/// The most air speed a single tick can add along the wish direction: `accel · maxspeed · dt`. At any
/// sane tickrate this exceeds [`AIR_CAP`], putting the optimum at a perpendicular strafe.
pub fn air_accel_max(accel: f32, maxspeed: f32, dt: f32) -> f32 {
    accel * maxspeed * dt
}

/// The wish angle off the velocity (degrees) that maximizes the per-tick speed gain, from the
/// air-accel geometry: the gain² is `2·u·a + a²` with `u = s·cosθ` and `a = min(a_max, cap − u)`.
/// When `a_max ≥ cap` (the usual case) the optimum is `u = 0` → **90°, perpendicular**; otherwise
/// it's `u = cap − a_max`. One formula covers both and degrades gracefully at coarse tickrates.
pub fn theta_star(speed: f32, a_max: f32) -> f32 {
    let u_star = (AIR_CAP - a_max.min(AIR_CAP)).max(0.0);
    (u_star / speed.max(1.0)).clamp(0.0, 1.0).acos().to_degrees()
}

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

/// One frame's strafe usercmd, from [`strafe`] (air) or [`prestrafe`] (ground).
#[derive(Clone, Copy, Debug)]
pub struct Strafe {
    /// View yaw to send (degrees).
    pub view_yaw: f32,
    /// `forward` move component: 0 in the air (single-key strafe); the bearing-aligned share of
    /// the wish during a ground prestrafe.
    pub forward: f32,
    /// `side` move component (± [`MOVE_SPEED`]).
    pub side: f32,
    /// The strafe sign chosen this frame (±1), to carry into the next as sticky state.
    pub sigma: f32,
}

/// Compute the air-strafe: aim the view so a single held strafe key puts the wish direction at the
/// speed-optimal angle off the current velocity, and choose the strafe side to weave the heading
/// toward `wp_bearing`. `prev_sigma` is last frame's side (`0` on the first frame).
///
/// With `forward = 0` and `side = −sigma·MOVE`, the engine's `right` vector places the wish
/// direction at `view_yaw ± 90°`, so `view_yaw = vel_yaw + sigma·(θ*−90)` — in practice the view
/// rides exactly on the velocity and sweeps smoothly with it; sign flips move only the strafe key.
pub fn strafe(v_xy: Vec2, wp_bearing: f32, prev_sigma: f32, a_max: f32) -> Strafe {
    let speed = v_xy.length().max(1.0);
    let vel_yaw = v_xy.y.atan2(v_xy.x).to_degrees();
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
/// From the ground-accel geometry (`addspeed = maxspeed − u`, cap `a_g = accel·maxspeed·dt`), the
/// gain² `2·u·a + a²` under `a = min(a_g, maxspeed − u)` peaks at `u* = maxspeed − a_g`, i.e.
/// `θg = acos(u*/speed)` — 0° until `speed > u*` (≈278), then bending outward as speed grows.
///
/// Unlike the air strafe the **view stays on the bearing** and the angle is expressed through the
/// forward/side split instead (`wishvel = forward·fwd + side·right` rotates by `δ` exactly), so
/// engaging/disengaging and sign flips never move the view — no snap, just a curving run.
pub fn prestrafe(v_xy: Vec2, bearing: f32, prev_sigma: f32, a_g: f32, maxspeed: f32) -> Strafe {
    let speed = v_xy.length();
    // Below the angling threshold (or barely moving) there's nothing to exploit: run at the
    // bearing to build base speed. Also avoids steering off a garbage yaw from a ~zero velocity.
    let u_star = (maxspeed - a_g).max(0.0);
    if speed <= u_star.max(60.0) {
        return Strafe { view_yaw: bearing, forward: MOVE_SPEED, side: 0.0, sigma: prev_sigma };
    }
    let vel_yaw = v_xy.y.atan2(v_xy.x).to_degrees();
    let err = wrap180(bearing - vel_yaw);
    let sigma = weave_sigma(err, prev_sigma, weave_band(speed));
    let theta_g = (u_star / speed).clamp(0.0, 1.0).acos().to_degrees();
    let delta = wrap180(vel_yaw + sigma * theta_g - bearing); // wish yaw relative to the view
    let (ds, dc) = delta.to_radians().sin_cos();
    Strafe {
        view_yaw: bearing,
        forward: MOVE_SPEED * dc,
        side: -MOVE_SPEED * ds,
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
}

/// Physics inputs the controller needs each frame, read from cvars by the caller.
pub struct Env {
    /// Frame time (s).
    pub dt: f32,
    /// `sv_accelerate`.
    pub accel: f32,
    /// `sv_maxspeed`.
    pub maxspeed: f32,
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
    /// Lenient conditions to take *another* hop from a landing (leg kind still ok, goal ahead).
    pub sustain: bool,
    /// Hard off — combat/hook/gate/grenade wants the view, or the cvar is off. The only thing
    /// that may cut a hop mid-air.
    pub veto: bool,
    /// A `SpeedJump` leg: the runway is pre-verified, so bypass entry/continuation checks.
    pub committed: bool,
    /// At the takeoff edge too slow to clear the gap (`sj_hold`): keep building, don't leap.
    pub hold_jump: bool,
    /// Game time (s).
    pub now: f32,
}

/// The usercmd the controller wants this frame.
#[derive(Clone, Copy, Debug)]
pub struct Cmd {
    /// View yaw to send (degrees); the caller supplies pitch.
    pub view_yaw: f32,
    /// `forward` move component.
    pub forward: f32,
    /// `side` move component.
    pub side: f32,
    /// Press `BUTTON_JUMP` this frame.
    pub jump: bool,
}

/// The bunnyhop controller's per-bot state. Lives on `BotState`; drive with [`Bhop::step`].
#[derive(Default, Clone, Debug)]
pub struct Bhop {
    /// Where in the hop cycle we are.
    pub phase: Phase,
    /// Sticky strafe sign (±1; 0 = unseeded).
    sigma: f32,
    /// When the entry conditions started holding (0 = not holding) — the engage hysteresis clock.
    eligible_since: f32,
    /// When the current phase began.
    phase_start: f32,
    /// Last frame's cmd pressed jump — the pulse guard: if a press didn't take (still grounded),
    /// release for one frame so `PM_CheckJump` sees a fresh edge, then press again.
    jump_prev: bool,
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
                i.now - self.eligible_since >= ENGAGE_DELAY
            } else {
                self.eligible_since = 0.0;
                false
            };
            if !engage {
                return None;
            }
            self.engage(i);
        }
        let speed = i.v_xy.length();
        self.peak = self.peak.max(speed);
        let a_max = air_accel_max(env.accel, env.maxspeed, env.dt);
        let a_g = a_max; // same cap formula on the ground; only the addspeed limit differs

        if self.phase == Phase::Prestrafe {
            let launch = !i.on_ground
                || speed >= PRESTRAFE_TARGET
                || i.now - self.phase_start > PRESTRAFE_MAX_T
                || i.runway < speed * T_HOP * 2.0 + HOP_MARGIN; // keep room to actually hop
            if !launch {
                return Some(self.ground_cmd(i, a_g, env.maxspeed));
            }
            self.phase = Phase::Hop;
            self.phase_start = i.now;
        }
        self.hop_cmd(i, speed, a_max, a_g, env.maxspeed)
    }

    /// The hop loop: air-strafe while airborne; on a landing frame decide whether another hop
    /// fits, and if so take off again with a strafe+jump cmd (full air-accel gain — see module
    /// docs on `PM_CheckJump` running before `PM_Friction`).
    fn hop_cmd(&mut self, i: &Input, speed: f32, a_max: f32, a_g: f32, maxspeed: f32) -> Option<Cmd> {
        if !i.on_ground {
            self.jump_prev = false; // airborne releases the button, re-arming PM_CheckJump
            let s = self.weave(strafe(i.v_xy, i.bearing, self.sigma, a_max));
            return Some(Cmd { view_yaw: s.view_yaw, forward: s.forward, side: s.side, jump: false });
        }
        // Landing (or first) ground frame — the only place a run ends by policy.
        if !i.committed && !(i.sustain && i.runway >= speed * T_HOP + HOP_MARGIN) {
            self.disengage(if i.sustain { "runway" } else { "leg" });
            return None;
        }
        if i.hold_jump {
            // Too slow at the takeoff edge: keep gaining on the ground instead of leaping short.
            return Some(self.ground_cmd(i, a_g, maxspeed));
        }
        let jump = !self.jump_prev;
        self.jump_prev = jump;
        if jump {
            self.hops += 1;
            let s = self.weave(strafe(i.v_xy, i.bearing, self.sigma, a_max));
            Some(Cmd { view_yaw: s.view_yaw, forward: s.forward, side: s.side, jump: true })
        } else {
            // The pulse-release frame after a press that didn't take: still gain on the ground.
            Some(self.ground_cmd(i, a_g, maxspeed))
        }
    }

    /// A prestrafe cmd, with sigma/flip bookkeeping.
    fn ground_cmd(&mut self, i: &Input, a_g: f32, maxspeed: f32) -> Cmd {
        self.jump_prev = false;
        let s = self.weave(prestrafe(i.v_xy, i.bearing, self.sigma, a_g, maxspeed));
        Cmd { view_yaw: s.view_yaw, forward: s.forward, side: s.side, jump: false }
    }

    /// Record a strafe's sign into the sticky state, counting flips for telemetry.
    fn weave(&mut self, s: Strafe) -> Strafe {
        if self.sigma != 0.0 && s.sigma != self.sigma {
            self.flips += 1;
        }
        self.sigma = s.sigma;
        s
    }

    fn engage(&mut self, i: &Input) {
        let speed = i.v_xy.length();
        self.phase = if i.on_ground && speed < PRESTRAFE_TARGET && i.runway > PRESTRAFE_MIN_RUNWAY {
            Phase::Prestrafe
        } else {
            Phase::Hop
        };
        self.phase_start = i.now;
        self.sigma = 0.0;
        self.jump_prev = false;
        self.hops = 0;
        self.flips = 0;
        self.peak = 0.0;
    }

    fn disengage(&mut self, reason: &'static str) {
        self.phase = Phase::Off;
        self.sigma = 0.0;
        self.jump_prev = false;
        self.eligible_since = 0.0;
        self.off_reason = reason;
    }
}

// --- engine oracles (tests + documentation of the model the live engine implements) --------------

/// A faithful one-tick QuakeWorld `PM_AirAccelerate`: the wish speed's projection onto the velocity
/// is capped at [`AIR_CAP`], and `accel·wishspeed·dt` (uncapped `wishspeed`) is added along
/// `wishdir`. Used as the unit-test oracle for the controller (and to document the model the live
/// engine implements — the engine, not this module, applies it at runtime).
#[allow(dead_code)]
pub fn apply_airaccel(v: Vec2, wishdir: Vec2, wishspeed: f32, accel: f32, dt: f32) -> Vec2 {
    let addspeed = wishspeed.min(AIR_CAP) - v.dot(wishdir);
    if addspeed <= 0.0 {
        return v;
    }
    let accelspeed = (accel * wishspeed * dt).min(addspeed);
    v + wishdir * accelspeed
}

/// A faithful one-tick QuakeWorld `PM_Accelerate` (ground): as [`apply_airaccel`] but the
/// projection limit is the full `wishspeed` (≤ `sv_maxspeed`), not [`AIR_CAP`].
#[allow(dead_code)]
pub fn apply_groundaccel(v: Vec2, wishdir: Vec2, wishspeed: f32, accel: f32, dt: f32) -> Vec2 {
    let addspeed = wishspeed - v.dot(wishdir);
    if addspeed <= 0.0 {
        return v;
    }
    let accelspeed = (accel * wishspeed * dt).min(addspeed);
    v + wishdir * accelspeed
}

/// A faithful one-tick QuakeWorld `PM_Friction` on flat ground: drop `max(speed, stopspeed) ·
/// friction · dt`, floored at zero.
#[allow(dead_code)]
pub fn apply_friction(v: Vec2, friction: f32, stopspeed: f32, dt: f32) -> Vec2 {
    let speed = v.length();
    if speed < 1.0 {
        return Vec2::ZERO;
    }
    let drop = speed.max(stopspeed) * friction * dt;
    v * ((speed - drop).max(0.0) / speed)
}

/// The wish direction the engine derives from a view yaw and forward/side move components:
/// `wishvel = forward·(cos, sin) + side·(sin, −cos)`, normalized. Exposed for tests and to make
/// the view↔wishdir geometry explicit.
#[allow(dead_code)]
pub fn wishdir_fs(view_yaw: f32, forward: f32, side: f32) -> Vec2 {
    let (sy, cy) = view_yaw.to_radians().sin_cos();
    (Vec2::new(cy, sy) * forward + Vec2::new(sy, -cy) * side).normalize_or_zero()
}

/// [`wishdir_fs`] for the single-key air strafe (`forward = 0`).
#[allow(dead_code)]
pub fn wishdir_of(view_yaw: f32, side: f32) -> Vec2 {
    wishdir_fs(view_yaw, 0.0, side)
}

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
        sum.y.atan2(sum.x).to_degrees()
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
}

/// Full hop-cycle simulation: [`Bhop::step`] driven against a pmove oracle that mirrors FTEQW's
/// frame order — `PM_CheckJump` **before** `PM_Friction`, then ground/air accel, then gravity.
/// This is the harness that would have caught the original integration bugs: it exercises
/// landings, takeoffs, prestrafe, and the engage/disengage policy, not just the pure air math.
#[cfg(test)]
mod sim {
    use super::*;

    const DT: f32 = 1.0 / 77.0;
    const ENV: Env = Env { dt: DT, accel: 10.0, maxspeed: 320.0 };
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
            World { pos: Vec2::ZERO, z: 0.0, vz: 0.0, v: Vec2::new(speed, 0.0), on_ground: true, jump_held: false }
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
            sustain: true,
            veto: false,
            committed: false,
            hold_jump: false,
            now,
        }
    }

    /// While the controller is Off, the bot runs at the bearing through the normal steering path.
    fn run_cmd(bearing: f32) -> Cmd {
        Cmd { view_yaw: bearing, forward: MOVE_SPEED, side: 0.0, jump: false }
    }

    fn mean_heading(tail: &[Vec2]) -> f32 {
        let sum: Vec2 = tail.iter().map(|v| v.normalize_or_zero()).sum();
        sum.y.atan2(sum.x).to_degrees()
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
        assert!(w.v.length() >= 550.0, "only {} ups after 10s (peak {})", w.v.length(), b.peak);
        let heading = mean_heading(&trace[trace.len() - 77..]);
        assert!(heading.abs() < 8.0, "mean heading drifted to {heading}");
        assert!(b.hops >= 3, "only {} hops in 10s", b.hops);
        let flips_per_hop = b.flips as f32 / b.hops as f32;
        assert!((1.0..=6.0).contains(&flips_per_hop), "{} flips over {} hops", b.flips, b.hops);
    }

    #[test]
    fn tracks_a_30_degree_bend() {
        let mut w = World::grounded(450.0); // past the prestrafe target → engages straight to Hop
        let mut b = Bhop::default();
        let mut trace = Vec::new();
        for f in 0..539 {
            let now = f as f32 * DT;
            let bearing = if now < 3.0 { 0.0 } else { 30.0 };
            let cmd = b.step(&input(&w, bearing, 4096.0, now), &ENV).unwrap_or(run_cmd(bearing));
            pm_frame(&mut w, &cmd, false);
            trace.push(w.v);
        }
        // 3s at bearing 0, then 2s at 30°: the last half-second should sit on the new bearing.
        let heading = mean_heading(&trace[trace.len() - 38..]);
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
            let s = prestrafe(v, 0.0, sigma, a_g, ENV.maxspeed);
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
        assert!(v.length() < 520.0, "prestrafe equilibrium implausibly high: {}", v.length());
    }
}
