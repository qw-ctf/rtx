// SPDX-License-Identifier: AGPL-3.0-or-later

//! Shared QuakeWorld movement-physics constants. These live in `rtx_nav::qphys` (single-sourced so
//! the navmesh planner and the game's movement code can't drift apart); this module re-exports them
//! so game code keeps referring to `crate::qphys::{AIR_CAP, JUMP_VZ, STEP_HEIGHT}` unchanged.

pub use rtx_nav::qphys::{AIR_CAP, JUMP_VZ, STEP_HEIGHT};
