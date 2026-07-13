// SPDX-License-Identifier: AGPL-3.0-or-later

//! Shared QuakeWorld movement-physics constants — single-sourced so the game's bhop controller
//! and pmove sim and the navmesh speed-jump planner ([`crate::navmesh`]) model the same engine and
//! can't drift apart. Only genuinely engine-fixed values live here; each consumer keeps its own
//! *model* tickrate (the planner assumes a conservative ~72 Hz, the live controller sizes its weave
//! to the ~77 Hz the engine actually steps bots at), because those are modelling choices, not
//! engine constants, and reconciling them would change which links the navmesh builds.
//!
//! The game crate re-exports these from its own `qphys` module, so game code keeps referring to
//! `crate::qphys::{AIR_CAP, JUMP_VZ, STEP_HEIGHT}` unchanged.

/// The QW `PM_AirAccelerate` projected-wishspeed cap (an engine literal, `movevars_maxairspeed`;
/// not a cvar). Only this much of the wish speed counts against the current velocity each air tick,
/// which is exactly what lets a perpendicular air-strafe keep gaining speed past `sv_maxspeed`.
pub const AIR_CAP: f32 = 30.0;

/// Jump impulse (`velocity.z`) — fixed, so a jump's airtime/apex don't change with horizontal
/// speed; only the reach does (`speed · airtime`), which is what lets a fast bhopper clear a wide
/// gap.
pub const JUMP_VZ: f32 = 270.0;

/// The tallest lip pmove steps over without a jump (`MAX_STEP_HEIGHT`). Shared by the navmesh
/// (what counts as a walkable step vs a ledge) and the game-side offline pmove sim.
pub const STEP_HEIGHT: f32 = 18.0;

/// A standing player's origin sits this far above its feet (the render/visual floor): the QW player
/// box is `mins.z = -24`. A carved cell stores the *origin* height, so drop by this to reach the
/// floor a foot-level probe or a viewer's floor tile wants.
pub const ORIGIN_TO_FEET: f32 = 24.0;
