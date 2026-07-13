// SPDX-License-Identifier: AGPL-3.0-or-later

//! `rtx-nav` — the pure navigation core shared by the `rtx` game module and the `navview` viewer.
//!
//! Contains the minimal collision-hull BSP reader ([`bsp`]), the bot navmesh builder and query
//! layer ([`navmesh`]), and the handful of engine-fixed movement constants both depend on
//! ([`qphys`]). No engine or game-state access — everything here is deterministic math over the
//! parsed BSP, so it builds and unit-tests without the host, and a standalone tool can reuse it.

pub mod bsp;
pub mod hazard;
pub mod math;
pub mod navmesh;
pub mod pmove;
pub mod qphys;
pub mod strafe;
