// SPDX-License-Identifier: AGPL-3.0-or-later

//! The two pure angle helpers the movement oracles ([`crate::strafe`]) and the offline pmove sim
//! ([`crate::pmove`]) need. Kept in rtx-nav so the navmesh build can run those oracles (curl-jump
//! certification) without depending on the game crate. The game's richer angle math re-exports these.

use glam::Vec2;

/// Planar heading of `dir` in degrees (Quake yaw): `atan2(y, x)`. Range `(-180, 180]`.
pub fn yaw_of(dir: Vec2) -> f32 {
    dir.y.atan2(dir.x).to_degrees()
}

/// Wrap an angle into (-180, 180].
pub fn wrap180(a: f32) -> f32 {
    let mut a = a % 360.0;
    if a > 180.0 {
        a -= 360.0;
    } else if a < -180.0 {
        a += 360.0;
    }
    a
}
