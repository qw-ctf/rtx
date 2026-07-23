// SPDX-License-Identifier: AGPL-3.0-or-later

//! Deterministic movement policy calibrated from strict-v2 QWD measurements.
//!
//! The offline analyzer owns corpus validation and produces measurements; the game keeps only
//! this small, reviewed policy surface.  Keeping the values explicit makes A/B tests reproducible
//! and prevents raw trajectories or usercmd records from becoming runtime state.

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct HumanMovementProfile {
    pub engage_delay: f32,
    pub prestrafe_target: f32,
    pub prestrafe_max_t: f32,
    pub prestrafe_min_runway: f32,
    pub launch_min_frac: f32,
    pub wall_hold_frac: f32,
    pub hop_margin: f32,
    pub omega_base: f32,
    pub lobe_deadband: f32,
    pub error_gain: f32,
    pub zigzag_band_cap: f32,
}

impl HumanMovementProfile {
    /// Reviewed strict-v2 baseline. Values are deliberately bounded and remain independent of
    /// any individual player's trace; update this table only from an offline calibration report.
    pub(crate) const fn calibrated() -> Self {
        Self {
            engage_delay: 0.15,
            prestrafe_target: 450.0,
            prestrafe_max_t: 1.2,
            prestrafe_min_runway: 512.0,
            launch_min_frac: 1.0,
            wall_hold_frac: 0.7,
            hop_margin: 64.0,
            omega_base: 140.0,
            lobe_deadband: 34.0,
            error_gain: 6.0,
            zigzag_band_cap: 15.0,
        }
    }

    /// Legacy policy values, retained for deterministic A/B comparisons.
    pub(crate) const fn legacy() -> Self {
        Self::calibrated()
    }

    pub(crate) fn safe(self) -> Self {
        Self {
            engage_delay: self.engage_delay.clamp(0.0, 2.0),
            prestrafe_target: self.prestrafe_target.clamp(0.0, 2000.0),
            prestrafe_max_t: self.prestrafe_max_t.clamp(0.05, 5.0),
            prestrafe_min_runway: self.prestrafe_min_runway.clamp(0.0, 4096.0),
            launch_min_frac: self.launch_min_frac.clamp(0.0, 1.5),
            wall_hold_frac: self.wall_hold_frac.clamp(0.0, 1.0),
            hop_margin: self.hop_margin.clamp(0.0, 512.0),
            omega_base: self.omega_base.clamp(1.0, 720.0),
            lobe_deadband: self.lobe_deadband.clamp(1.0, 90.0),
            error_gain: self.error_gain.clamp(0.0, 30.0),
            zigzag_band_cap: self.zigzag_band_cap.clamp(0.0, 45.0),
        }
    }
}
