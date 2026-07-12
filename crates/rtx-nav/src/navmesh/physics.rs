// SPDX-License-Identifier: AGPL-3.0-or-later

//! QuakeWorld movement kinematics: the pmove-derived constants and the closed-form jump/bhop math
//! (airtime, required entry speed, air-strafe speed gain) the graph build and the banded planner
//! share. Pure — no graph or BSP state. `STEP_HEIGHT` (pmove's free step-up) is shared from
//! [`crate::qphys`].

use crate::qphys::{AIR_CAP, JUMP_VZ};

// --- player + movement constants (QuakeWorld pmove) ---

/// Height delta treated as effectively flat ground (a `Walk`).
pub(super) const WALK_DZ: f32 = 8.0;
/// Largest one-way fall we'll encode as a landing (`Drop` links, and `JumpGap` links that leap
/// out and plunge). Deliberately huge: QW fall damage is a flat 5 HP past the 650 u/s landing
/// threshold, and deliberate multi-thousand-unit plunges are core movement — race maps are
/// *built* around them. The cost carries the real fall time (see `link_cost`), so a bot never
/// prefers a pit over a lift without reason.
pub(super) const MAX_DROP: f32 = 4096.0;
/// Double jumps keep the old, shallow landing floor: descent is already covered by the cheaper
/// Drop/JumpGap kinds, and the double-jump dedup is per-octant *without* elevation bands — a
/// deep pit target would shadow the level crossing the link kind exists for.
pub(super) const DJ_MAX_DROP: f32 = 240.0;
/// Fall height beyond which QW fall damage applies (`MAX_SAFE_FALL` ≈ when speed > 580).
pub(super) const SAFE_FALL: f32 = 88.0;
/// Apex a standing jump adds: `jump_vel² / (2·gravity)` = `270² / 1600`. Public so a viewer can
/// re-fly a [`LinkKind::JumpGap`](super::LinkKind) arc with [`arc_point`](super::arc_point) using
/// the same apex the build cleared with.
pub const JUMP_APEX: f32 = 45.0;
/// Horizontal reach of a running jump (`maxspeed · air-time`), conservatively floored.
pub(super) const JUMP_REACH: f32 = 200.0;
/// Extra reach/rise unlocked by rtx's mid-air **double jump** (`rtx_doublejump`): a second jump near
/// the apex restacks a ~45u arc, roughly doubling both. Conservatively floored so a bot with slightly
/// off air-jump timing still clears the linked gap.
pub(super) const DOUBLE_JUMP_REACH: f32 = 300.0;
pub(super) const DOUBLE_JUMP_APEX: f32 = 80.0;
/// Clearance envelope for a double jump — the real two-arc path peaks ~91u above the launch, so
/// sample the arc a touch higher to be safe. Public for the same reason as [`JUMP_APEX`]: a viewer
/// re-flies a [`LinkKind::DoubleJump`](super::LinkKind) arc with [`arc_point`](super::arc_point) at
/// this apex.
pub const DOUBLE_ARC_PEAK: f32 = 100.0;
/// `sv_maxspeed` default — the cost denominator (travel time = distance / speed).
pub const MAX_SPEED: f32 = 320.0;

// --- speed jumps (bunnyhop-carried leaps across wide gaps) ---

/// Conservative server tickrate assumed for the bhop acceleration model (see [`crate::qphys`] on why
/// this deliberately differs from the live controller's ~77 Hz).
const SJ_TICKRATE: f32 = 72.0;
/// Speed we'll plan bhop runways up to (reach ≈ `V·0.675` ≈ 600u); real runways bound it further.
pub(super) const SPEED_JUMP_V_CAP: f32 = 900.0;
/// Derate the ideal bhop model to attainable speed (the S-weave + a friction frame per landing).
/// Calibrated against the controller's own pmove-oracle sim (`bhop::sim`): a 10s run covers
/// ~4500u and lands at ~0.75 of the ideal `(v0³+3k·len)^⅓` — 0.8 rides just above it, with
/// [`SJ_MARGIN`] absorbing the difference.
pub const BHOP_EFF: f32 = 0.8;
/// Longest runway we bother measuring. Sized so the model can credit the speeds the controller
/// demonstrably reaches (its sim sustains gains past 550 u/s over ~4500u): at 4096u the
/// effective takeoff is ~605 u/s — flat gaps to ~350u, dropping gaps to ~620u — where the old
/// 2048 cap forfeited everything past ~490 u/s. Race maps are what need the far end.
pub(super) const RUNWAY_MAX: f32 = 4096.0;
/// The measured runway must reach this multiple of the jump's required entry speed.
pub(super) const SJ_MARGIN: f32 = 1.15;
/// Walkable floor must continue this far past the landing (the takeoff-phase window).
pub(super) const SJ_LANDING_DEPTH: f32 = 96.0;
/// Speed-jump landing floor — separate from (and smaller than) [`MAX_DROP`] because the
/// target-scan radius grows with fall airtime (`reach = v · t`): 1024 quadruples the old 240
/// envelope while keeping the per-ledge scan bounded.
pub(super) const SJ_MAX_DROP: f32 = 1024.0;
/// At most this many stand-start speed-jump links per source cell.
pub(super) const SPEED_JUMP_MAX_PER_CELL: usize = 3;
/// At most this many *chained* speed-jump links per source cell — kept separate (and small) so a
/// chained candidate never evicts a self-contained stand-start jump from the per-cell budget.
pub(super) const SPEED_JUMP_CHAINED_MAX_PER_CELL: usize = 2;

// --- speed bands (kinodynamic planning over (cell, band); see `find_path_banded`) ---

/// Coarse entry-speed classes for the banded planner. A route's carried speed changes both the
/// feasibility of a leg (a chained speed jump needs a minimum band) and its cost (a fast band
/// covers a Walk leg quicker). Four bands keep the search state at `cells · 4`.
/// `BAND_EDGES[i]` is the upper edge of band `i`: `<340 → 0`, `<430 → 1`, `<540 → 2`, else `3`.
pub const BAND_EDGES: [f32; 3] = [340.0, 430.0, 540.0];
/// Number of speed bands.
pub const NBANDS: usize = 4;
/// The speed *credited* to a band — its lower edge. Feasibility and cost always use this floor,
/// never a midpoint, so the planner never assumes more speed than a band guarantees.
pub const BAND_FLOOR: [f32; NBANDS] = [MAX_SPEED, 340.0, 430.0, 540.0];
/// Planning speed ceiling — the banded heuristic's denominator (matches [`SPEED_JUMP_V_CAP`]).
/// Larger than [`MAX_SPEED`], so the banded heuristic is *smaller* (more conservative) than the
/// cell-only one — at worst it expands more nodes, never less optimal than the existing search.
pub const BAND_V_MAX: f32 = SPEED_JUMP_V_CAP;
/// Runway a standing start (band 0) must spend spinning up before air-strafe gains begin, charged
/// against a Walk/Step leg's length. Mirrors the game's bhop controller engage runway.
pub(super) const BAND_SPINUP: f32 = 256.0;
/// A carried-speed leg only counts if the corridor continues roughly straight: if the turn from the
/// link that *reached* a cell to the candidate link exceeds this, the planner treats the entry as
/// band 0 (speed is not carried around a sharp corner).
pub(super) const SPEED_CONE_DEG: f32 = 45.0;

/// The speed band a given horizontal speed falls into (`0..NBANDS`).
pub fn band_of(speed: f32) -> u8 {
    BAND_EDGES.iter().position(|&e| speed < e).unwrap_or(NBANDS - 1) as u8
}

/// Airtime of a jump reaching a target `dz` above (or below) the takeoff, at gravity `g`: the
/// descending root of `JUMP_VZ·t − ½g·t² = dz`. `0` if `dz` is unreachable (above the apex).
pub(super) fn jump_airtime(dz: f32, gravity: f32) -> f32 {
    let disc = JUMP_VZ * JUMP_VZ - 2.0 * gravity * dz;
    if disc < 0.0 {
        return 0.0;
    }
    (JUMP_VZ + disc.sqrt()) / gravity
}

/// The horizontal entry speed needed to clear `horiz` while rising/falling `dz`, at gravity `g`.
pub(super) fn v_required(horiz: f32, dz: f32, gravity: f32) -> f32 {
    let t = jump_airtime(dz, gravity);
    if t <= 0.0 {
        f32::INFINITY
    } else {
        horiz / t
    }
}

/// Bhop speed-gain constant `k`: velocity² grows at `2k` per second while air-strafing. Derived from
/// the perpendicular air-accel cap and the tickrate (`k = tick · a² / 2`, `a = min(accel·maxspeed/tick, cap)`).
pub fn bhop_k(accel: f32, maxspeed: f32) -> f32 {
    let a = (accel * maxspeed / SJ_TICKRATE).min(AIR_CAP);
    SJ_TICKRATE * a * a / 2.0
}

/// Speed reached after air-strafing `len` units from `v0`: `(v0³ + 3k·len)^⅓`.
pub fn attainable_speed(v0: f32, len: f32, k: f32) -> f32 {
    (v0.powi(3) + 3.0 * k * len.max(0.0)).cbrt()
}

/// Runway length needed to air-strafe from `v0` up to `v`: `(v³ − v0³) / 3k`.
pub(super) fn runway_len_for(v: f32, v0: f32, k: f32) -> f32 {
    ((v.powi(3) - v0.powi(3)) / (3.0 * k)).max(0.0)
}

/// Time to air-strafe from `v0` up to `v`: `(v² − v0²) / 2k`.
pub(super) fn runway_time(v: f32, v0: f32, k: f32) -> f32 {
    ((v * v - v0 * v0) / (2.0 * k)).max(0.0)
}
