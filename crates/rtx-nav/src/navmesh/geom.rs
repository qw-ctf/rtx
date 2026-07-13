// SPDX-License-Identifier: AGPL-3.0-or-later

//! Pure BSP geometry queries and grid math the build leans on: floor bisection, segment/arc
//! clearance traces, the jump parabola sampler ([`arc_point`], shared with the viewer so both
//! trace the identical curve), and the grid-column / compass / elevation-band helpers used to
//! dedup jump links. Stateless — every function is math over the parsed [`Bsp`] or plain ints.

use glam::{Vec3, Vec3Swizzles};

use super::physics::{jump_airtime, DOUBLE_JUMP_REACH, JUMP_APEX, JUMP_REACH, MAX_DROP};
use super::{GRID, GROUND_SAMPLE, GROUND_SLACK};
use crate::bsp::Bsp;
use crate::qphys::{JUMP_VZ, STEP_HEIGHT};

/// Bisect the floor origin height between a solid sample below and an empty one above.
pub(super) fn bisect_floor(bsp: &Bsp, x: f32, y: f32, z_solid: f32, z_empty: f32) -> f32 {
    let (mut lo, mut hi) = (z_solid, z_empty);
    for _ in 0..8 {
        let mid = (lo + hi) * 0.5;
        if bsp.is_solid(Vec3::new(x, y, mid)) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    hi
}

/// Whether the standing player hull can actually **descend** into `to` from height `from_z`: trace
/// the hull straight down the column above `to`. A floor gap too small for the ±16 hull — a grate or
/// slot you can see the water through but can't fit through — blocks the trace, so no drop / down-jump
/// link is generated into it. Point-sampled floor finding falls through such slots; this doesn't.
pub(super) fn descent_clear(bsp: &Bsp, from_z: f32, to: Vec3) -> bool {
    if from_z <= to.z {
        return true; // not a descent
    }
    let tr = bsp.hull1_trace(Vec3::new(to.x, to.y, from_z), to);
    !tr.start_solid && tr.fraction > 0.99
}

/// Whether the straight segment between two standing origins is free of solid (sampled at the
/// higher origin so a wall or low ceiling between the cells blocks the move).
pub(super) fn path_clear(bsp: &Bsp, a: Vec3, b: Vec3) -> bool {
    let z = a.z.max(b.z);
    let steps = ((b.xy() - a.xy()).length() / 16.0).ceil().max(1.0) as i32;
    (0..=steps).all(|i| {
        let t = i as f32 / steps as f32;
        let p = a.lerp(b, t);
        !bsp.is_solid(Vec3::new(p.x, p.y, z))
    })
}

/// Whether solid floor continues *under* the straight segment between two standing origins — the
/// floor-continuity test [`path_clear`] deliberately doesn't do (it samples the head-height corridor
/// for walls/ceilings, so an air gap beneath the segment reads "clear"). Interior points every
/// `GROUND_SAMPLE` must have hull-1 solid within a step below the interpolated origin height; the
/// endpoints are carved cells (already supported), so only the span between them is checked. Because
/// it queries the same ±16 box-expanded hull the carve uses, a floor narrower than the player box
/// still reads supported (you can't fall through it) — so balancing along a thin wall-top survives,
/// while a diagonal Walk/Step link whose centre line crosses an L-shaped ledge corner's air fails.
pub(super) fn ground_along(is_solid: &impl Fn(Vec3) -> bool, a: Vec3, b: Vec3) -> bool {
    let steps = ((b.xy() - a.xy()).length() / GROUND_SAMPLE).ceil().max(1.0) as i32;
    (1..steps).all(|i| {
        let p = a.lerp(b, i as f32 / steps as f32);
        is_solid(Vec3::new(p.x, p.y, p.z - (STEP_HEIGHT + GROUND_SLACK)))
    })
}

/// Whether a jump arc from `a` to `b` clears geometry: sample a parabola peaking `JUMP_APEX`
/// above the higher endpoint and require every point to be open.
pub(super) fn arc_clear(bsp: &Bsp, a: Vec3, b: Vec3) -> bool {
    arc_clear_peak(bsp, a, b, JUMP_APEX, 8)
}

/// Clearance along the **true ballistic path** of a run-jump onto a target far below. The
/// symmetric parabola of [`arc_clear_peak`] interpolates z against *horizontal* progress, which
/// on a deep plunge dives toward the floor midway — the real jump keeps most of its height
/// early (constant horizontal speed, quadratic fall), so sample z(t) with nominal gravity and
/// xy linear in t.
pub(super) fn ballistic_clear(bsp: &Bsp, a: Vec3, b: Vec3) -> bool {
    let t_land = jump_airtime(b.z - a.z, 800.0);
    if t_land <= 0.0 {
        return false;
    }
    let steps = ((a.distance(b) / 64.0).ceil() as i32).clamp(8, 48);
    (0..=steps).all(|i| {
        let f = i as f32 / steps as f32;
        let t = t_land * f;
        let xy = a.xy().lerp(b.xy(), f);
        let z = a.z + JUMP_VZ * t - 400.0 * t * t; // ½·800·t²
        !bsp.is_solid(Vec3::new(xy.x, xy.y, z))
    })
}

/// A point at parameter `t ∈ [0, 1]` along a jump arc from `a` to `b` with apex `apex` above the
/// higher endpoint: xy is linear in `t`, z is the parabola through `a.z` (t=0) and `b.z` (t=1)
/// peaking at `max(a.z, b.z) + apex`. Shared by the build's clearance check (`arc_clear_peak`) and
/// any consumer that re-flies the same arc for display, so both trace the identical curve.
pub fn arc_point(a: Vec3, b: Vec3, apex: f32, t: f32) -> Vec3 {
    let peak = a.z.max(b.z) + apex;
    let xy = a.xy().lerp(b.xy(), t);
    let z = a.z + (b.z - a.z) * t + 4.0 * (peak - a.z.max(b.z)) * t * (1.0 - t);
    Vec3::new(xy.x, xy.y, z)
}

/// [`arc_clear`] with a caller-chosen apex height (for the taller double-jump arc) and step count.
pub(super) fn arc_clear_peak(bsp: &Bsp, a: Vec3, b: Vec3, apex: f32, steps: i32) -> bool {
    (0..=steps).all(|i| !bsp.is_solid(arc_point(a, b, apex, i as f32 / steps as f32)))
}

/// Grid column index for a world coordinate.
pub(super) fn floor_grid(v: f32) -> i32 {
    (v / GRID).floor() as i32
}

/// The eight compass grid directions (used to find hook launch edges).
pub(super) const COMPASS: [(i32, i32); 8] = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)];

/// Whether the segment `p0`→`p1` intersects the axis-aligned box `[min, max]` (slab method).
/// Used to decide which navmesh links a closed door's volume blocks.
pub(super) fn segment_aabb_intersect(p0: Vec3, p1: Vec3, min: Vec3, max: Vec3) -> bool {
    let (o, d) = (p0.to_array(), (p1 - p0).to_array());
    let (lo, hi) = (min.to_array(), max.to_array());
    let (mut tmin, mut tmax) = (0.0f32, 1.0f32);
    for i in 0..3 {
        if d[i].abs() < 1e-6 {
            if o[i] < lo[i] || o[i] > hi[i] {
                return false; // parallel to this slab and outside it
            }
        } else {
            let inv = 1.0 / d[i];
            let mut t0 = (lo[i] - o[i]) * inv;
            let mut t1 = (hi[i] - o[i]) * inv;
            if t0 > t1 {
                std::mem::swap(&mut t0, &mut t1);
            }
            tmin = tmin.max(t0);
            tmax = tmax.min(t1);
            if tmin > tmax {
                return false;
            }
        }
    }
    true
}

/// How many grid columns a jump can span.
pub(super) fn jump_grid_radius() -> i32 {
    (JUMP_REACH / GRID).ceil() as i32
}

/// How many grid columns a double jump can span.
pub(super) fn double_jump_grid_radius() -> i32 {
    (DOUBLE_JUMP_REACH / GRID).ceil() as i32
}

/// Bucket a grid direction into a 3×3 compass cell (0..9, center index 4 unused), for jump
/// dedup. Distinct for all 8 surrounding directions — opposite directions never collide.
pub(super) fn dir_bucket(dgx: i32, dgy: i32) -> usize {
    ((dgx.signum() + 1) + (dgy.signum() + 1) * 3) as usize
}

/// Height span of one jump-dedup elevation band — one "storey", matching the hook pass's 128u
/// elevation banding. Same-octant targets within a band are true duplicates (land on the nearer,
/// walk on); a band apart they are distinct destinations that must not shadow each other.
pub(super) const JUMP_ELEV_SPAN: f32 = 128.0;
/// Band indices a jump target can occupy: `round(dz / JUMP_ELEV_SPAN)` over the jump's dz gate
/// `[-MAX_DROP, JUMP_APEX]` — bands `{-(MAX_DROP/SPAN) .. 0}`, sized from the constants.
pub(super) const JUMP_ELEV_BANDS: usize = (MAX_DROP / JUMP_ELEV_SPAN) as usize + 1;

/// Elevation band of a jump target's height delta, as an index into `0..JUMP_ELEV_BANDS`.
/// `round`, not `floor`, so the top band is centred on "level with the takeoff": a −16u
/// ledge-to-ledge crossing and a −128u drop to the pit floor under it must land in different
/// bands (with `floor` both would hit the same band and the nearer pit drop would win the dedup).
pub(super) fn jump_elev_band(dz: f32) -> usize {
    (((dz / JUMP_ELEV_SPAN).round() as i32) + JUMP_ELEV_BANDS as i32 - 1).clamp(0, JUMP_ELEV_BANDS as i32 - 1) as usize
}
