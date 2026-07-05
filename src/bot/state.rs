// SPDX-License-Identifier: AGPL-3.0-or-later

//! Per-bot AI state carried on the bot's client edict, plus the two small phase-machine enums the
//! hook and grenade drivers run on. Split out of `entity.rs` so the ~60-field blackboard lives with
//! the code that reads it (`crate::bot`), not with the engine-shared entity layout.

use glam::Vec3;

/// Per-bot navigation/AI state, on the bot's client edict (`1..=maxclients`). Non-bot edicts
/// keep this at its `Default` (`is_bot == false`). See [`crate::bot`].
#[derive(Default)]
pub struct BotState {
    /// Whether this client edict is an rtx-driven bot (fake client).
    pub is_bot: bool,
    /// 1-based engine client number, for `set_bot_cmd`/`remove_bot`.
    pub client: i32,
    /// Current A* route as link indices into the navmesh, and our leg within it.
    pub route: Vec<u32>,
    pub route_pos: usize,
    /// The cell we last routed toward (`u32::MAX` = none), to detect when to re-path.
    pub goal_cell: u32,
    /// The gate currently being opened as an errand (`None` = following the human normally).
    pub gate: Option<usize>,
    /// The item entity this bot is fetching (`0` = none → follow a human), and the navmesh cell
    /// it sits in. See [`crate::bot::goals`].
    pub goal_item: u32,
    pub goal_item_cell: u32,
    /// Earliest time the bot may re-pick its item goal (throttles the catalog scan).
    pub goal_select_time: f32,
    /// When the bot began chasing its current item goal. If it's *still* chasing the same item
    /// long after (one it can't actually reach — e.g. behind an elevator/button/movewall/teleporter
    /// chain the router can't thread), it abandons that goal rather than circling forever. Uses
    /// time-on-goal, not distance, so a legitimate route that walks *away* toward a teleporter
    /// isn't mistaken for being stuck.
    pub goal_started: f32,
    /// An item to skip while picking goals, until `avoid_until` — set when we gave up reaching it,
    /// so we don't immediately re-fixate on the same unreachable pickup.
    pub avoid_item: u32,
    pub avoid_until: f32,
    /// Earliest time we may recompute the route (throttles A*).
    pub repath_time: f32,
    /// Stuck detector: where we were when last checked, and since when we've been there.
    pub stuck_origin: Vec3,
    pub stuck_since: f32,
    /// Origin on the previous bot frame, to detect a teleport (a large instant jump) and
    /// re-path from the landing spot.
    pub last_origin: Vec3,
    /// Per-frame toggle, flipped each tick, used to *pulse* buttons that QW only acts on at a
    /// press edge (the respawn key, which needs a release between presses).
    pub pulse: bool,
    /// Smoothed view state: a critically damped spring drives `aim` (current view angles, seeded
    /// from `v_angle`) toward the frame's look target with angular velocity `aim_vel` (deg/s), so
    /// a spectated bot turns like a mouse-controlled human — fast proportional flicks, smooth
    /// settle, no per-frame snapping. Spring stiffness scales with skill, so low-skill bots also
    /// track moving targets more slowly.
    pub aim: Vec3,
    pub aim_vel: Vec3,
    /// Drifting aim error (degrees, x=pitch y=yaw): wanders smoothly toward `aim_err_target`,
    /// which is resampled at `aim_err_until` — misses sweep past the target and drift back rather
    /// than buzz (per-frame white noise reads as jitter). Magnitude scales inversely with skill.
    pub aim_err: Vec3,
    pub aim_err_target: Vec3,
    pub aim_err_until: f32,
    /// Where the combat enemy was last actually visible, and when. While line of sight is briefly
    /// lost the bot *holds this angle* (like a player holding a corner) instead of snapping back
    /// to its navigation view.
    pub enemy_seen_at: Vec3,
    pub enemy_seen_time: f32,
    /// Last frame's clean firing-solution angles and their timestamp, to estimate how fast the
    /// solution is moving (deg/s). Feeds the aim feed-forward that cancels the spring's tracking
    /// lag against a strafing target.
    pub look_prev: Vec3,
    pub look_prev_time: f32,
    /// Audience-wander destination (a round mode's stands) and the next time to pick a new one.
    /// Only used while the mode marks this bot as an audience/spectator; zero otherwise.
    pub wander_target: Vec3,
    pub wander_time: f32,
    /// Gate-errand progress watchdog: the closest we've gotten to the target button and the time
    /// we last got closer. If we stop making progress (stuck at a door we can't reach the button
    /// of) we give up — a flat timeout would wrongly abandon a button that's simply far away. Plus
    /// the gate index + expiry to avoid re-taking that errand for a while after giving up.
    pub gate_best_dist: f32,
    pub gate_since: f32,
    pub avoid_gate: i32,
    pub avoid_gate_until: f32,
    /// Grappling-hook traversal state machine (see [`crate::bot`]), driven when the current route leg
    /// is a [`LinkKind::Hook`](crate::navmesh::LinkKind::Hook): aim at the anchor, throw, reel to
    /// build speed, release into a parabola, ride it to the target.
    pub hook_phase: HookPhase,
    /// The hook leg (link index) currently being flown, and when the active phase began (per-phase
    /// timeout base).
    pub hook_link: u32,
    pub hook_started: f32,
    /// Distance-from-anchor at which to release, re-solved against the *live* anchor once the hook
    /// bites (so the parabola lands on the target despite aim/stance error), plus last frame's
    /// distance-to-anchor to detect the release crossing and a stalled reel.
    pub hook_release_dist: f32,
    pub hook_prev_dist: f32,
    /// Consecutive failed hook attempts toward the current goal — two in a row abandons the goal.
    pub hook_fails: u8,
    /// Grenade lob→shoot combo state machine (see [`crate::bot::grenade`]): aim a lobbed grenade,
    /// then detonate it to airburst an enemy or shove them into a hazard.
    pub grenade_phase: GrenadePhase,
    /// When the current combo phase began (per-phase timeout base; becomes the fuse clock once the
    /// grenade is fired).
    pub grenade_started: f32,
    /// The blast point the lob targets, and the solved view angles to lob it there.
    pub grenade_target: Vec3,
    pub grenade_look: Vec3,
    /// The lobbed grenade entity once captured (`0` = not yet in flight / none).
    pub grenade_ent: u32,
    /// This combo is a no-line-of-sight **bank shot** — detonate on the fuse, not by shooting the
    /// grenade (which the bot can't see). See `crate::bot::grenade`'s bank-shot start.
    pub grenade_bank: bool,
    /// Desired shove direction (unit, horizontal) when the combo is a hazard shove; `ZERO` for a
    /// plain airburst.
    pub grenade_shove_dir: Vec3,
    /// Distance from the enemy to the hazard edge — the shove must carry them at least this far.
    pub grenade_shove_edge: f32,
    /// Earliest time the bot may start another combo (anti-spam).
    pub grenade_next_try: f32,
    /// The bunnyhop controller (see [`crate::bot::bhop`]): the hop-cycle phase machine, sticky
    /// strafe sign, engage hysteresis, and telemetry.
    pub bhop: crate::bot::bhop::Bhop,
    /// Last frame's sent bhop view yaw (to seed the aim spring's angular velocity when combat
    /// resumes).
    pub bhop_prev_yaw: f32,
    /// The [`LinkKind::SpeedJump`](crate::navmesh::LinkKind::SpeedJump) leg currently being flown (a
    /// committed bhop run-up + leap), and when it began. `None` = not on a speed jump.
    pub sj_leg: Option<u32>,
    pub sj_started: f32,
}

/// Phase of a bot's grenade lob→shoot combo. `Idle` unless a combo is in progress.
#[derive(Default, Clone, Copy, PartialEq, Eq, Debug)]
pub enum GrenadePhase {
    /// Not running a combo.
    #[default]
    Idle,
    /// Grenade launcher selected, settling the view onto the lob angles before firing.
    Windup,
    /// Grenade in the air; switching to a detonator and tracking it.
    Lobbed,
    /// Detonator in hand; shoot the grenade the instant its blast lands the enemy where we want.
    Detonate,
}

/// Phase of a bot's grappling-hook traversal. `Idle` unless the current route leg is a hook link.
#[derive(Default, Clone, Copy, PartialEq, Eq, Debug)]
pub enum HookPhase {
    /// Not flying a hook.
    #[default]
    Idle,
    /// Selecting the grapple and settling the view onto the anchor before the throw.
    Aim,
    /// Hook thrown, waiting for it to bite.
    Flight,
    /// Anchored: reeling in, holding fire, until the release point.
    Reel,
    /// Released: riding the parabola with no input until it lands.
    Ballistic,
}
