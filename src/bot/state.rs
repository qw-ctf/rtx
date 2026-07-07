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
    /// Handoff hold (team opponent modeling): a spawned RL/LG this bot stands on but deliberately does
    /// **not** pick up (`bot_pickup_items` skips it), reserving it for a powerup-carrying teammate that
    /// lacks it. `0` = not holding. `hold_for` is that teammate; `hold_until` the hard deadline after
    /// which the bot takes the weapon itself (denial beats a handoff that never arrives).
    pub hold_item: u32,
    pub hold_for: u32,
    pub hold_until: f32,
    /// Earliest time the bot may re-pick its item goal (throttles the catalog scan).
    pub goal_select_time: f32,
    /// When the bot began chasing its current item goal. If it's *still* chasing the same item
    /// long after (one it can't actually reach — e.g. behind an elevator/button/movewall/teleporter
    /// chain the router can't thread), it abandons that goal rather than circling forever. Uses
    /// time-on-goal, not distance, so a legitimate route that walks *away* toward a teleporter
    /// isn't mistaken for being stuck.
    pub goal_started: f32,
    /// Items to skip while picking goals, each until its paired expiry — set when we gave up reaching
    /// one (unreachable pickup) or just collected one (so an instant-respawn item or lingering
    /// weapons-stay trigger can't re-capture the goal slot the same second). A small ring: a fresh
    /// entry evicts the soonest-to-expire slot. `(item entid, until)`; a `0` item marks an empty slot.
    pub avoid_items: [(u32, f32); 4],
    /// Links this bot recently failed to traverse (stuck, stalled speed-jump, given-up hook), each a
    /// `(link idx, until, strikes)`: a per-bot A* surcharge (growing with strikes) that makes the
    /// planner *divert* around a dead leg instead of re-issuing the identical route to retry forever.
    /// A fixed ring; a fresh failure bumps a matching entry or evicts the soonest-to-expire slot.
    pub failed_links: [(u32, f32, u8); 8],
    /// Earliest time we may recompute the route (throttles A*).
    pub repath_time: f32,
    /// Stuck detector: where we were when last checked, and since when we've been there.
    pub stuck_origin: Vec3,
    pub stuck_since: f32,
    /// Path-progress watchdog: the closest straight-line distance to the goal we've reached on the
    /// current route, and when it last improved. No improvement for a while ⇒ the leg is failing in a
    /// way the displacement stuck-detector can't see (orbiting, wall-sliding) — penalize and re-path.
    pub progress_best: f32,
    pub progress_since: f32,
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
    /// to its navigation view. Written by combat under *true* line of sight only (the bhop veto and
    /// corner-hold key off it) — perception memory below is separate.
    pub enemy_seen_at: Vec3,
    pub enemy_seen_time: f32,
    /// Perception ([`crate::bot::perception`]): the target we're currently accruing sight-reaction
    /// time on and since when (a change of target or a break in sight restarts it); the target we've
    /// been promoted to *aware of* and the expiry of that awareness memory; where it was last
    /// perceived (hunted while aware but out of sight); and when continuous line of sight to the
    /// current target began (drives combat's aim-error convergence). Distinct from `enemy_seen_*`:
    /// these advance on hear/feel too, so they must not be read where true line of sight is meant.
    pub percept_ent: u32,
    pub percept_since: f32,
    pub known_enemy: u32,
    pub known_until: f32,
    pub percept_last_seen: Vec3,
    pub vis_since: f32,
    /// Last frame's clean firing-solution angles and their timestamp, to estimate how fast the
    /// solution is moving (deg/s). Feeds the aim feed-forward that cancels the spring's tracking
    /// lag against a strafing target.
    pub look_prev: Vec3,
    pub look_prev_time: f32,
    /// Audience-wander destination (a round mode's stands) and the next time to pick a new one.
    /// Only used while the mode marks this bot as an audience/spectator; zero otherwise.
    pub wander_target: Vec3,
    pub wander_time: f32,
    /// Item vigil ([`crate::bot::vigil`]): while waiting on an uncollectable goal item (mid-respawn,
    /// or a handoff-held weapon) the bot cruises a short walk away and scans the room. `vigil_post` is
    /// the current cruise spot (`ZERO` = none / heading back to the item) with its re-pick deadline;
    /// `scan_point` is the world point the eyes sweep to, held until `scan_until`. Disjoint from
    /// `wander_*` (roam needs `goal_item == 0`; audience wander needs a Move intent — a vigil bot is
    /// by definition chasing an item), so the two never overlap despite the similar shape.
    pub vigil_post: Vec3,
    pub vigil_post_until: f32,
    pub scan_point: Vec3,
    pub scan_until: f32,
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
    /// The plain jump leg (JumpGap/DoubleJump) currently being flown, and when the commitment began.
    /// Latched at takeoff, it freezes the route and locks out combat until landing — so an enemy
    /// appearing mid-arc can't flip the goal, replace the route, and yank the bot off the jump (the
    /// `sj_leg`/rocket-jump commitment, which plain jumps previously lacked). `None` = not committed.
    pub air_leg: Option<u32>,
    pub air_started: f32,
    /// Rocket-jump traversal machine (see [`crate::bot::rj`]): stance → jump → fire → ride the blast
    /// arc onto a high ledge. `rj_link` is the leg being flown; `rj_started` the per-phase timeout
    /// base; `rj_jump_time` the moment the jump was pressed (the fire-delay clock); `rj_fails` the
    /// consecutive-failure count (two aborts avoid the goal, like the hook).
    pub rj_phase: RjPhase,
    pub rj_link: u32,
    pub rj_started: f32,
    pub rj_jump_time: f32,
    pub rj_fails: u8,
}

impl BotState {
    /// Add `item` to the avoid ring until `until` (bumping a matching entry's expiry, else evicting
    /// the soonest-to-expire slot). Ignores `0` (the "no item" sentinel).
    pub fn mark_avoid(&mut self, item: u32, until: f32) {
        if item == 0 {
            return;
        }
        if let Some(slot) = self.avoid_items.iter_mut().find(|(it, _)| *it == item) {
            slot.1 = slot.1.max(until);
        } else if let Some(slot) = self.avoid_items.iter_mut().min_by(|a, b| a.1.total_cmp(&b.1)) {
            *slot = (item, until);
        }
    }

    /// Whether `item` is currently on the avoid ring (an unexpired entry).
    pub fn is_avoided(&self, item: u32, now: f32) -> bool {
        self.avoid_items.iter().any(|&(it, until)| it == item && now < until)
    }
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

/// Phase of a bot's rocket-jump traversal. `Idle` unless the current route leg is a rocket jump.
#[derive(Default, Clone, Copy, PartialEq, Eq, Debug)]
pub enum RjPhase {
    /// Not flying a rocket jump.
    #[default]
    Idle,
    /// Walking to the launch cell, RL selected, view settling on the solved fire angles.
    Stance,
    /// Jump pressed; holding the aim and counting down `fire_delay` to the shot.
    Rise,
    /// Blast taken: riding the arc with gentle air-correction toward the landing.
    Ballistic,
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
