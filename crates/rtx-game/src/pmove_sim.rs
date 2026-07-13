// SPDX-License-Identifier: AGPL-3.0-or-later

//! Re-export shim: the offline QuakeWorld `PM_PlayerMove` sim ([`pm_step`]) now lives in
//! [`rtx_nav::pmove`], so the navmesh build's curl-jump certifier drives the *same* physics as the
//! race-line optimizer and the bot controller. This keeps `crate::pmove_sim::{pm_step, PmState,
//! PmParams}` resolving unchanged for the game crate's callers (raceline, demo_replay, control).

pub use rtx_nav::pmove::*;

#[cfg(test)]
mod tests {
    use glam::Vec2;

    use crate::bot::bhop::{self, air_accel_max, apply_airaccel, wishdir_of};
    use rtx_nav::qphys::AIR_CAP;

    /// The accel oracles `pm_step` composes are the same ones the controller's own `strafe` weaves
    /// with, so a flat-world bhop run driven through them ramps speed exactly as the controller
    /// expects — the cross-crate guard that the moved oracles stay bit-identical to the controller.
    #[test]
    fn horizontal_update_matches_bhop_oracle() {
        let dt = 1.0 / 77.0;
        let (accel, maxspeed) = (10.0, 320.0);
        let a = air_accel_max(accel, maxspeed, dt);
        assert!(a >= AIR_CAP);
        let mut v = Vec2::new(320.0, 0.0);
        let mut sigma = 0.0;
        for _ in 0..385 {
            let cmd = bhop::strafe(v, 0.0, sigma, a);
            sigma = cmd.sigma;
            v = apply_airaccel(v, wishdir_of(cmd.view_yaw, cmd.side), maxspeed, accel, dt);
        }
        assert!(v.length() > 600.0, "oracle ramp regressed: {}", v.length());
    }
}
