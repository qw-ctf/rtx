// SPDX-License-Identifier: AGPL-3.0-or-later

//! Pure Quake angle/vector math — the direction↔angle conversions and angle wrapping that glam
//! doesn't provide. Every function here is a free function of its inputs (no `self`, no
//! `GameState`), so they unit-test in isolation and belong to no single game system.

use glam::{Vec2, Vec3, Vec3Swizzles};

/// Planar heading of `dir` in degrees (Quake yaw): `atan2(y, x)`. Range `(-180, 180]`.
pub(crate) fn yaw_of(dir: Vec2) -> f32 {
    dir.y.atan2(dir.x).to_degrees()
}

/// Elevation of `dir` above horizontal in degrees (positive = up). The horizontal length is
/// floored at 1.0 to keep the near-vertical case finite. A view *pitch* is the negation of this.
pub(crate) fn elevation_of(dir: Vec3) -> f32 {
    dir.z.atan2(dir.xy().length().max(1.0)).to_degrees()
}

/// View angles (pitch, yaw, 0) from `eye` toward `point`.
pub(crate) fn angles_to(eye: Vec3, point: Vec3) -> Vec3 {
    let d = point - eye;
    Vec3::new(-elevation_of(d), yaw_of(d.xy()), 0.0)
}

/// `SUB_NormalizeAngles` — wrap each angle into `(-360, 360)` (QuakeC's `fmod(a, 360)`).
pub(crate) fn normalize_angles(a: Vec3) -> Vec3 {
    Vec3::new(a.x % 360.0, a.y % 360.0, a.z % 360.0)
}

/// Wrap an angle into (-180, 180].
pub(crate) fn wrap180(a: f32) -> f32 {
    let mut a = a % 360.0;
    if a > 180.0 {
        a -= 360.0;
    } else if a < -180.0 {
        a += 360.0;
    }
    a
}

/// `vectoangles` — convert a direction to `(pitch, yaw, 0)` Euler angles (degrees). Keeps QuakeC's
/// own `0..360` normalization and straight-up/down special-case, so it is *not* `elevation_of`.
pub(crate) fn vectoangles(v: Vec3) -> Vec3 {
    if v.x == 0.0 && v.y == 0.0 {
        let pitch = if v.z > 0.0 { 90.0 } else { 270.0 };
        return Vec3::new(pitch, 0.0, 0.0);
    }
    let mut yaw = v.y.atan2(v.x).to_degrees();
    if yaw < 0.0 {
        yaw += 360.0;
    }
    let forward = (v.x * v.x + v.y * v.y).sqrt();
    let mut pitch = v.z.atan2(forward).to_degrees();
    if pitch < 0.0 {
        pitch += 360.0;
    }
    Vec3::new(pitch, yaw, 0.0)
}

/// QuakeWorld `AngleVectors` (roll assumed 0): the view's forward, right, and up unit vectors —
/// exactly what the engine's `makevectors` produces and `w_fire_grenade` orients the launch by.
pub(crate) fn angle_vectors(angles: Vec3) -> (Vec3, Vec3, Vec3) {
    let (sy, cy) = angles.y.to_radians().sin_cos();
    let (sp, cp) = angles.x.to_radians().sin_cos();
    let forward = Vec3::new(cp * cy, cp * sy, -sp);
    let right = Vec3::new(sy, -cy, 0.0);
    let up = Vec3::new(sp * cy, sp * sy, cp);
    (forward, right, up)
}
