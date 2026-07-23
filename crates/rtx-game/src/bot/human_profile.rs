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
    /// Reverse at three fixed phases of each hop instead of heading-error hysteresis.
    pub phase_locked_flips: bool,
}

impl HumanMovementProfile {
    /// Reviewed strict-v2-compatible baseline. Values deliberately preserve the established
    /// movement behavior while remaining bounded and independent of any individual player's trace;
    /// update this table only from an aggregate offline calibration report.
    pub(crate) const fn calibrated() -> Self {
        Self {
            // A sustained straightaway has already passed the route/open-space gates. Enter on the
            // first eligible frame so the finite runway is spent accelerating, not waiting.
            engage_delay: 0.0,
            // Accepted QWD movement samples put ordinary sustained travel around 420–625 ups;
            // launch at the lower-middle of that band instead of the old 450 ups baseline.
            prestrafe_target: 475.0,
            prestrafe_max_t: 1.2,
            prestrafe_min_runway: 512.0,
            launch_min_frac: 1.0,
            wall_hold_frac: 0.7,
            hop_margin: 64.0,
            // Dedicated 600+ ups QWD bunny runs use full-magnitude, near-perpendicular pure-side
            // commands on 98–100% of moving samples. Request the physical maximum air-turn rate;
            // `strafe_rate` still clamps this to what the current speed/tick can deliver.
            omega_base: 720.0,
            // Max-gain strafes curve harder than the prior gentle lobes. The high-speed phase
            // scheduler makes three reversals per hop; this remains the fallback/error scale.
            lobe_deadband: 29.0,
            error_gain: 6.0,
            zigzag_band_cap: 15.0,
            phase_locked_flips: true,
        }
    }

    /// Legacy policy values, retained for deterministic A/B comparisons.
    pub(crate) const fn legacy() -> Self {
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
            phase_locked_flips: false,
        }
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
            phase_locked_flips: self.phase_locked_flips,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::HumanMovementProfile;

    #[test]
    fn legacy_profile_is_a_real_ab_baseline() {
        let calibrated = HumanMovementProfile::calibrated();
        let legacy = HumanMovementProfile::legacy();
        assert_ne!(legacy, calibrated);
        assert!(!legacy.phase_locked_flips);
        assert!(calibrated.phase_locked_flips);
        assert_eq!(legacy.prestrafe_target, 450.0);
        assert_eq!(legacy.omega_base, 140.0);
        assert_eq!(legacy.lobe_deadband, 34.0);
    }
}
