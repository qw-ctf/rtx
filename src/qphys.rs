// SPDX-License-Identifier: AGPL-3.0-or-later

//! Shared QuakeWorld movement-physics constants — single-sourced so the bhop controller
//! ([`crate::bot::bhop`]) and the navmesh speed-jump planner ([`crate::navmesh`]) model the same
//! engine and can't drift apart. Only genuinely engine-fixed values live here; each module keeps
//! its own *model* tickrate (the planner assumes a conservative ~72 Hz, the live controller sizes
//! its weave to the ~77 Hz the engine actually steps bots at), because those are modelling choices,
//! not engine constants, and reconciling them would change which links the navmesh builds.

/// The QW `PM_AirAccelerate` projected-wishspeed cap (an engine literal, `movevars_maxairspeed`;
/// not a cvar). Only this much of the wish speed counts against the current velocity each air tick,
/// which is exactly what lets a perpendicular air-strafe keep gaining speed past `sv_maxspeed`.
pub const AIR_CAP: f32 = 30.0;

/// Jump impulse (`velocity.z`) — fixed, so a jump's airtime/apex don't change with horizontal
/// speed; only the reach does (`speed · airtime`), which is what lets a fast bhopper clear a wide
/// gap.
pub const JUMP_VZ: f32 = 270.0;

/// The tallest lip pmove steps over without a jump (`MAX_STEP_HEIGHT`). Shared by the navmesh
/// (what counts as a walkable step vs a ledge) and the offline pmove sim ([`crate::pmove_sim`]).
pub const STEP_HEIGHT: f32 = 18.0;
