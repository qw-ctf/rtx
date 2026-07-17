// SPDX-License-Identifier: AGPL-3.0-or-later

//! Per-bot AI state carried on the bot's client edict, plus the two small phase-machine enums the
//! hook and grenade drivers run on. Split out of `entity.rs` so the ~60-field blackboard lives with
//! the code that reads it (`crate::bot`), not with the engine-shared entity layout.

use glam::Vec3;

use crate::navmesh::CellId;

/// Per-bot navigation/AI state, on the bot's client edict (`1..=maxclients`). Non-bot edicts
/// keep this at its `Default` (`is_bot == false`). See [`crate::bot`].
#[derive(Default)]
pub struct BotState {
    /// Whether this client edict is an rtx-driven bot (fake client).
    pub is_bot: bool,
    /// 1-based engine client number, for `set_bot_cmd`/`remove_bot`.
    pub client: i32,
    /// Whether this body was alive on the previous bot frame. The false→true edge is the one spawn
    /// signal shared by server bots and network-client mirrors.
    pub was_alive: bool,
    /// Fresh-DM-spawn stack run: suppress initiating a fight and keep one reachable armor/weapon
    /// pickup completion-critical. Ends as soon as that pickup changes armor or weapon inventory.
    pub spawn_exit: bool,
    /// Current A* route as link indices into the navmesh, and our leg within it.
    pub route: Vec<u32>,
    pub route_pos: usize,
    /// Planned entry speed band per leg (parallel to `route`), from the banded planner
    /// ([`crate::navmesh::NavGraph::find_path_banded`]); all-zero when speed-band planning is off.
    /// A band ≥ 1 on the current or next leg tells the bhop controller to carry speed through the
    /// waypoint instead of disengaging (see `carry` in [`crate::bot::bhop::Input`]).
    pub route_bands: Vec<u8>,
    /// The cell we last routed toward (`None` = nothing yet), to detect when to re-path.
    pub goal_cell: Option<CellId>,
    /// The gate the bot is opening as an errand, plus the gate it's avoiding. See [`GateState`].
    pub gate: GateState,
    /// The bot's item-fetch goal: which item, where, the re-pick throttle, and the handoff/avoid
    /// bookkeeping. See [`GoalState`].
    pub goal: GoalState,
    /// Links this bot recently failed to traverse (stuck, stalled speed-jump, given-up hook), each a
    /// `(link idx, until, strikes)`: a per-bot A* surcharge (growing with strikes) that makes the
    /// planner *divert* around a dead leg instead of re-issuing the identical route to retry forever.
    /// A fixed ring; a fresh failure bumps a matching entry or evicts the soonest-to-expire slot.
    pub failed_links: [(u32, f32, u8); 8],
    /// Teleport links this bot recently rode, each `(link idx, until)` — a per-bot decaying surcharge
    /// ring (mirrors [`Self::failed_links`], evict-soonest) priced in `bot_link_pricing`. Damps the
    /// re-entry shuttle: after teleporting, the just-used pad *and* the reverse pad by the exit cost
    /// extra for a few seconds, so a stable far-side goal re-routes by foot (or gives up) instead of
    /// bouncing back through the free 0.2 link. Cost-shaping, not a ban — a sole-route teleport is still
    /// taken.
    pub recent_tele: [(u32, f32); 8],
    /// Earliest time we may recompute the route (throttles A*).
    pub repath_time: f32,
    /// The route-progress watchdogs — three ways a bot notices it isn't getting anywhere.
    pub watchdog: Watchdog,
    /// Per-frame toggle, flipped each tick, used to *pulse* buttons that QW only acts on at a
    /// press edge (the respawn key, which needs a release between presses).
    pub pulse: bool,
    /// Smoothed view state: the aim spring plus its error and feed-forward memory. See [`Aim`].
    pub aim: Aim,
    /// Where the combat enemy was last actually visible, under true line of sight only. See [`SeenEnemy`].
    pub seen: SeenEnemy,
    /// Perception memory (hear/feel as well as sight). See [`Perception`].
    pub percept: Perception,
    /// Strategic fight posture derived from relative effective strength/firepower. Recovery owns
    /// movement toward a useful item; Hold/Press leave full movement to combat.
    pub posture: CombatPosture,
    /// Audience-wander state (a round mode's stands). See [`Wander`].
    pub wander: Wander,
    /// Anti-drown surface target: the air spot the nearest-breathable flood picked, with a short TTL
    /// (`time`) so a drowning bot doesn't re-run the graph Dijkstra every frame. `target == ZERO`
    /// (or an expired `time`) means "recompute". Reuses the [`Wander`] `{target, time}` shape.
    pub surface: Wander,
    /// Burn-escape target: the nearest non-hazard cell the escape flood picked when the bot is stuck
    /// standing in lava/slime, cached with the same short TTL as [`surface`](Self::surface). A separate
    /// cache because deep lava can trip both reflexes at once and the targets differ — breathable air
    /// for drowning vs. any safe footing for burning.
    pub burn: Wander,
    /// When the bot first started burning (standing in lava/slime), or `0.0` when not. The escape
    /// reflex fires only once this has persisted past [`BURN_PANIC_SECS`](crate::bot::BURN_PANIC_SECS),
    /// so a deliberate moat/bridge crossing isn't hijacked.
    pub burn_since: f32,
    /// Audience watch (Rocket Arena): the fighter this bot's eyes are held on. See [`Watch`].
    pub watch: Watch,
    /// Item vigil ([`crate::bot::vigil`]): cruise-and-scan while waiting on an uncollectable goal
    /// item. See [`Vigil`].
    pub vigil: Vigil,
    /// Plat standoff — the lift we're holding off from while it's raised. See [`PlatWait`].
    pub plat_wait: Option<PlatWait>,
    /// Near-field steering grid (see [`crate::nearfield`]): an 8u clearance field around the bot for
    /// last-metre wall/ledge repulsion, rebuilt lazily when the bot strays off it or a nearby door
    /// changes state (`None` until first built, or when `rtx_bot_nearfield` is off). Off the routing
    /// path entirely — it only reshapes the immediate wish on grounded walk/step/approach legs.
    pub near: Option<crate::nearfield::NearField>,
    /// Grappling-hook traversal state machine (see [`HookState`]), driven when the current route leg
    /// is a [`LinkKind::Hook`](crate::navmesh::LinkKind::Hook).
    pub hook: HookState,
    /// Grenade lob→shoot combo state machine (see [`GrenadeState`]).
    pub grenade: GrenadeState,
    /// The bunnyhop controller (see [`crate::bot::bhop`]): the hop-cycle phase machine, sticky
    /// strafe sign, engage hysteresis, and telemetry.
    pub bhop: crate::bot::bhop::Bhop,
    /// A committed [`LinkKind::SpeedJump`](crate::navmesh::LinkKind::SpeedJump) leg (a bhop run-up +
    /// leap) being flown. `None` = not on a speed jump. See [`Commit`].
    pub sj: Option<Commit>,
    /// A committed plain jump leg (JumpGap/DoubleJump) being flown. Pre-armed before objective
    /// selection, it freezes the route and locks out combat until a physical landing — so an enemy
    /// appearing at the lip or mid-arc cannot flip the goal and yank the bot off the jump.
    pub air: Option<AirCommit>,
    /// Rocket-jump traversal machine (see [`RjState`]): stance → jump → fire → ride the blast arc
    /// onto a high ledge.
    pub rj: RjState,
    /// External puppet control (rocket-jump test harness, see [`crate::control`]): a scripted order
    /// overriding the bot's own objective, plus the goto stall tracker. `order == None` for a normal,
    /// autonomous bot — the whole harness path is inert unless the control channel issues an order.
    pub puppet: Puppet,
}

impl BotState {
    /// Add `item` to the avoid ring until `until` (bumping a matching entry's expiry, else evicting
    /// the soonest-to-expire slot). Ignores `0` (the "no item" sentinel).
    pub fn mark_avoid(&mut self, item: u32, until: f32) {
        if item == 0 {
            return;
        }
        if let Some(slot) = self.goal.avoid_items.iter_mut().find(|(it, _)| *it == item) {
            slot.1 = slot.1.max(until);
        } else if let Some(slot) = self.goal.avoid_items.iter_mut().min_by(|a, b| a.1.total_cmp(&b.1)) {
            *slot = (item, until);
        }
    }

    /// Whether `item` is currently on the avoid ring (an unexpired entry).
    pub fn is_avoided(&self, item: u32, now: f32) -> bool {
        self.goal.avoid_items.iter().any(|&(it, until)| it == item && now < until)
    }

    /// Shared failure tail for the hook / rocket-jump leg drivers (see [`Driver`]): bump the driver's
    /// consecutive-fail count and force a repath; after two failures in a row, abandon a chased goal
    /// item — drop the route, briefly avoid-list the item, and re-select next frame. The
    /// driver-specific phase reset (and the hook's deferred grapple reset) stay at the call site.
    pub(crate) fn traversal_failed(&mut self, driver: Driver, chasing: bool, now: f32) {
        let n = match driver {
            Driver::Hook => {
                self.hook.fails = self.hook.fails.saturating_add(1);
                self.hook.fails
            }
            Driver::RocketJump => {
                self.rj.fails = self.rj.fails.saturating_add(1);
                self.rj.fails
            }
        };
        self.repath_time = now;
        if n >= 2 {
            match driver {
                Driver::Hook => self.hook.fails = 0,
                Driver::RocketJump => self.rj.fails = 0,
            }
            self.route.clear();
            if chasing {
                self.mark_avoid(self.goal.item, now + super::GOAL_AVOID_TIME);
                self.goal.item = 0;
                self.goal.next_item = 0;
                self.goal.commit = GoalCommit::None;
                self.goal.next_commit = GoalCommit::None;
                self.goal.next_pick = now;
            }
        }
    }
}

/// Where the combat enemy was last *actually visible*, and when — written by combat under true line
/// of sight only (the bhop veto and corner-hold key off it). While sight is briefly lost the bot
/// holds this angle like a player holding a corner. Distinct from [`Perception`], which advances on
/// hear/feel too, so these must not be read where true line of sight is meant.
#[derive(Default)]
pub struct SeenEnemy {
    pub at: Vec3,
    pub time: f32,
}

/// Perception memory (see [`crate::bot::perception`]): the target currently accruing sight-reaction
/// time and since when (a change of target or a break in sight restarts it); the target promoted to
/// *aware of* and the expiry of that awareness; where it was last perceived (hunted while aware but
/// out of sight); and when continuous line of sight to the current target began (drives combat's
/// aim-error convergence). These advance on hear/feel too — see [`SeenEnemy`] for the sight-only spot.
#[derive(Default)]
pub struct Perception {
    pub ent: u32,
    pub since: f32,
    pub known_enemy: u32,
    pub known_until: f32,
    pub last_seen: Vec3,
    pub vis_since: f32,
}

/// Smoothed view state carried on a [`BotState`]. A critically damped spring drives `angles` (the
/// current view, seeded from `v_angle`) toward the frame's look target at `vel` (deg/s), so a
/// spectated bot turns like a mouse-controlled human — fast proportional flicks, smooth settle, no
/// per-frame snapping (stiffness scales with skill). `err*` is the drifting aim error; `look_prev*`
/// seeds the feed-forward. The bhop air-strafe runs through this same spring (its wish is carried in
/// forward/side, decoupled from the view), so it needs no separate bypass state.
#[derive(Default)]
pub struct Aim {
    /// Current smoothed view angles (deg), seeded from `v_angle` on first use.
    pub angles: Vec3,
    /// Angular velocity of the aim spring (deg/s).
    pub vel: Vec3,
    /// Drifting aim error (x=pitch, y=yaw, deg): wanders smoothly toward `err_target`, resampled at
    /// `err_until` — misses sweep past the target and drift back rather than buzz (per-frame white
    /// noise reads as jitter). Magnitude scales inversely with skill.
    pub err: Vec3,
    pub err_target: Vec3,
    pub err_until: f32,
    /// Last frame's clean firing-solution angles and their timestamp, to estimate how fast the
    /// solution is moving (deg/s). Feeds the aim feed-forward that cancels the spring's tracking lag
    /// against a strafing target.
    pub look_prev: Vec3,
    pub look_prev_time: f32,
    /// That rate, smoothed (deg/s). The raw one-frame difference carries the real tracking motion but
    /// also per-frame excursions of hundreds of deg/s, and drops to zero whenever a sample is rejected
    /// as a discontinuity. The lead multiplies all of it, so the view chases the smoothed estimate —
    /// per-frame white noise reads as jitter here for the same reason it does in `err`.
    pub rate: Vec3,
}

/// Audience-wander state: where a spectating bot is strolling in a round mode's stands, and the next
/// time to pick a new destination. Only used while the mode marks this bot as audience; zero otherwise.
#[derive(Default)]
pub struct Wander {
    pub target: Vec3,
    pub time: f32,
}

/// Audience watch (Rocket Arena): the live fighter this bot's eyes are held on (`0` = nobody), and
/// when to re-pick (or retry after losing sight). Chosen and LOS-validated by the mode; held ~1-2s
/// so the gaze doesn't ping-pong between duelists or flicker when sight blinks.
#[derive(Default)]
pub struct Watch {
    pub ent: u32,
    pub time: f32,
}

/// Item vigil (see [`crate::bot::vigil`]): while waiting on an uncollectable goal item (mid-respawn,
/// or a handoff-held weapon) the bot cruises a short walk away and scans the room. `post` is the
/// current cruise spot (`ZERO` = none / heading back to the item) with its re-pick deadline
/// `post_until`; `scan_point` is the world point the eyes sweep to, held until `scan_until`. Disjoint
/// from [`Wander`] (roam needs no item goal; audience wander needs a Move intent — a vigil bot is by
/// definition chasing an item), so the two never overlap despite the similar shape.
#[derive(Default)]
pub struct Vigil {
    pub post: Vec3,
    pub post_until: f32,
    pub scan_point: Vec3,
    pub scan_until: f32,
}

/// A latched route-leg commitment — a speed jump or a plain-jump arc — that freezes the route while
/// in flight: the leg being flown and when the commitment began (the watchdog's timeout base). While
/// Some, a goal flip mid-air can't replace the route and yank the bot off the jump.
#[derive(Clone, Copy)]
pub struct Commit {
    pub leg: u32,
    pub since: f32,
}

/// A plain gap/double-jump commitment. Unlike [`Commit`] (the speed-jump run-up latch), this records
/// whether the bot has actually left the ground and the expected landing cell: route advancement is
/// not evidence of landing, and a grounded takeoff frame must not release the lock.
#[derive(Clone, Copy)]
pub struct AirCommit {
    pub leg: u32,
    pub target: CellId,
    pub since: f32,
    pub airborne: bool,
}

/// Plat standoff (see [`crate::bot::steer`]): the navmesh plat index the bot is holding off from
/// while it's raised, and when the hold began (the give-up timeout base). Keyed on the plat index,
/// not the leg, so the 0.4s repath churn doesn't reset the timer.
#[derive(Clone, Copy)]
pub struct PlatWait {
    pub plat: usize,
    pub since: f32,
}

/// A gate the bot is diverting to open (its button unreachable by the normal route), plus its
/// progress watchdog: the closest we've gotten to the button and when we last got closer. If we stop
/// making progress (stuck at a door we can't reach the button of) we give up — a flat timeout would
/// wrongly abandon a button that's simply far away.
#[derive(Clone, Copy)]
pub struct GateErrand {
    pub index: usize,
    pub best_dist: f32,
    pub since: f32,
}

/// A bot's gate-opening state (see [`crate::bot::steer`]): the errand it's currently on, if any, and
/// a gate it recently gave up on and is avoiding for a while (so `route_blocking_gate` doesn't
/// immediately re-camp on a button that's walled off behind its own gate).
#[derive(Default)]
pub struct GateState {
    /// The gate errand in progress (`None` = following the human normally).
    pub errand: Option<GateErrand>,
    /// A gate to skip when picking the next errand, until the paired expiry (`None` = none avoided).
    pub avoid: Option<(usize, f32)>,
    /// Gate ids the last repath's LOD corridor crosses beyond the interim window, **nearest first**.
    /// The route stops at the interim, so `route_blocking_gate` can't see a far shut door; this stands
    /// in, restoring exact mode's far button-errand pre-arm — the far block works the first *shut* one
    /// in route order (not the lowest id). Empty when lod is off or the corridor crosses no far gate.
    pub corridor_gates: Vec<u32>,
}

/// A bot's item-fetch goal (see [`crate::bot::goals`]) and the bookkeeping around it.
#[derive(Default)]
pub struct GoalState {
    /// The item entity this bot is fetching (`0` = none → follow a human), and the navmesh cell it
    /// sits in.
    pub item: u32,
    pub item_cell: u32,
    /// Revalidated continuation from the bounded two-leg planner. Touching `item` promotes this
    /// candidate, but the normal validity pass may discard it immediately if world state changed.
    pub next_item: u32,
    pub next_cell: u32,
    pub next_commit: GoalCommit,
    /// Earliest time the bot may re-pick its item goal (throttles the catalog scan).
    pub next_pick: f32,
    /// Earliest time to run the cheap nearby lifesaving-pickup pre-pass. This is much faster than
    /// normal goal selection but need not flood the navmesh every server frame.
    pub next_urgent: f32,
    /// Waypoint magnetism (see [`crate::bot::goals::GameState::select_route_magnet`]): a desirable
    /// item lying just off the current route the bot bends its immediate waypoint through so it steps
    /// onto the trigger. `0` = none. Distinct from `item` — this is grabbed *in passing*, never
    /// chased — and re-picked on `magnet_pick`'s throttle (the classname scan isn't per-frame cheap).
    pub magnet_item: u32,
    pub magnet_pick: f32,
    /// When the bot began chasing its current item goal. If it's *still* chasing the same item long
    /// after (one it can't actually reach — e.g. behind an elevator/button/movewall/teleporter chain
    /// the router can't thread), it abandons that goal rather than circling forever. Uses time-on-
    /// goal, not distance, so a legitimate route that walks *away* toward a teleporter isn't mistaken
    /// for being stuck.
    pub since: f32,
    /// Completion lock for a pickup that must not lose movement ownership to combat or a periodic
    /// goal re-pick. `Pickup` covers local recovery, a selected major, or a fresh-spawn stack item;
    /// `Powerup` is a selected timed powerup. The lock ends only on touch, invalidation, route
    /// failure, or its watchdog.
    pub commit: GoalCommit,
    /// Handoff hold (team opponent modeling): a spawned RL/LG this bot stands on but deliberately
    /// does **not** pick up (`bot_pickup_items` skips it), reserving it for a powerup-carrying
    /// teammate that lacks it. `0` = not holding. `hold_for` is that teammate; `hold_until` the hard
    /// deadline after which the bot takes the weapon itself (denial beats a handoff that never arrives).
    pub hold_item: u32,
    pub hold_for: u32,
    pub hold_until: f32,
    /// Items to skip while picking goals, each until its paired expiry — set when we gave up reaching
    /// one (unreachable pickup) or just collected one (so an instant-respawn item or lingering
    /// weapons-stay trigger can't re-capture the goal slot the same second). A small ring: a fresh
    /// entry evicts the soonest-to-expire slot. `(item entid, until)`; a `0` item marks an empty slot.
    pub avoid_items: [(u32, f32); 4],
}

/// How strongly the current item goal is committed. Ordinary goals remain freely re-scored;
/// completion-critical stack pickups and timed powerups own movement until completion.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum GoalCommit {
    #[default]
    None,
    Pickup,
    Powerup,
}

/// Strategic combat posture with hysteresis. This is intentionally independent of aim skill: low
/// skill reacts and shoots less precisely, but does not become strategically suicidal.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum CombatPosture {
    Recover,
    #[default]
    Hold,
    Press,
}

/// Grappling-hook traversal state (see [`crate::bot::hook`]): aim at the anchor, throw, reel to
/// build speed, release into a parabola, ride it to the target.
#[derive(Default)]
pub struct HookState {
    pub phase: HookPhase,
    /// The hook leg (link index) currently being flown, and when the active phase began (per-phase
    /// timeout base).
    pub link: u32,
    pub started: f32,
    /// Distance-from-anchor at which to release, re-solved against the *live* anchor once the hook
    /// bites (so the parabola lands on the target despite aim/stance error), plus last frame's
    /// distance-to-anchor to detect the release crossing and a stalled reel.
    pub release_dist: f32,
    pub prev_dist: f32,
    /// Consecutive failed hook attempts toward the current goal — two in a row abandons the goal.
    pub fails: u8,
}

/// Grenade lob→shoot combo state (see [`crate::bot::grenade`]): aim a lobbed grenade, then detonate
/// it to airburst an enemy or shove them into a hazard.
#[derive(Default)]
pub struct GrenadeState {
    pub phase: GrenadePhase,
    /// When the current combo phase began (per-phase timeout base; becomes the fuse clock once the
    /// grenade is fired).
    pub started: f32,
    /// The blast point the lob targets, and the solved view angles to lob it there.
    pub target: Vec3,
    pub look: Vec3,
    /// The lobbed grenade entity once captured (`0` = not yet in flight / none).
    pub ent: u32,
    /// This combo is a no-line-of-sight **bank shot** — detonate on the fuse, not by shooting the
    /// grenade (which the bot can't see). See `crate::bot::grenade`'s bank-shot start.
    pub bank: bool,
    /// Desired shove direction (unit, horizontal) when the combo is a hazard shove; `ZERO` for a
    /// plain airburst.
    pub shove_dir: Vec3,
    /// Distance from the enemy to the hazard edge — the shove must carry them at least this far.
    pub shove_edge: f32,
    /// Earliest time the bot may start another combo (anti-spam).
    pub next_try: f32,
}

/// Rocket-jump traversal state (see [`crate::bot::rj`]): stance → jump → fire → ride the blast arc
/// onto a high ledge.
#[derive(Default)]
pub struct RjState {
    pub phase: RjPhase,
    /// The leg (link index) being flown.
    pub link: u32,
    /// The per-phase timeout base.
    pub started: f32,
    /// The moment the jump was pressed (the fire-delay clock).
    pub jump_time: f32,
    /// Consecutive-failure count (two aborts avoid the goal, like the hook).
    pub fails: u8,
    /// Per-attempt telemetry for the test harness — written by the driver / `emit` as the attempt
    /// unfolds and drained by [`crate::control`]. Inert for autonomous play (nothing consumes it; the
    /// next Stance entry overwrites it), so it only matters under a puppet order. See [`RjTelemetry`].
    pub telem: RjTelemetry,
}

/// One rocket-jump attempt's measurements, for the tuning harness (see [`crate::control`]). Populated
/// as the attempt runs — the solved plan and knob biases at Stance entry, the actual jump press and
/// rocket fire, and a terminal [`RjOutcome`] — then read once by the control poller to emit an
/// `rj_result` event. Everything the offline solve *predicted* sits beside what the bot *did*, so the
/// per-attempt error (stance offset, aim error, fire-timing error, landing miss) is directly derivable.
#[derive(Default, Clone)]
pub struct RjTelemetry {
    /// The rocket-jump link this attempt is flying.
    pub link: u32,
    /// The launch cell origin (source) and the target ledge cell origin, from the graph.
    pub src: Vec3,
    pub tgt: Vec3,
    /// The offline solve for this link: fire angles (pitch,yaw), fire delay after the jump, post-blast
    /// airtime, and pre-armor self-damage.
    pub solved_angles: Vec3,
    pub solved_delay: f32,
    pub airtime: f32,
    pub self_damage: f32,
    /// The knob biases in force at Stance entry (added to the solved delay/pitch), snapshotted so the
    /// result reports what was actually flown even if a knob changes mid-attempt.
    pub delay_bias: f32,
    pub pitch_bias: f32,
    /// The jump press (`None` until it happens), the rocket fire, and the terminal outcome.
    pub press: Option<RjPress>,
    pub fire: Option<RjFire>,
    pub outcome: Option<RjOutcome>,
}

/// The jump-press moment of a rocket-jump attempt (see [`RjTelemetry`]): when it fired, the origin and
/// post-spring view then, and the residual aim error (degrees) against the biased fire angles.
#[derive(Clone, Copy)]
pub struct RjPress {
    pub t: f32,
    pub origin: Vec3,
    pub view: Vec3,
    pub aim_err: f32,
}

/// The rocket-fire moment of a rocket-jump attempt (see [`RjTelemetry`]): when it fired, the actual
/// delay since the jump press (vs the solved delay), the origin, and the post-spring view sent with
/// `+attack` (filled in `emit`, where the settled view is known).
#[derive(Clone, Copy)]
pub struct RjFire {
    pub t: f32,
    pub actual_delay: f32,
    pub origin: Vec3,
    pub view: Vec3,
}

/// How a rocket-jump attempt ended (see [`RjTelemetry`]). `Landed`/`Overran` carry the touchdown
/// origin and time so the harness can measure the landing miss; the failure variants mark *where* the
/// attempt broke down (stance never aligned, jump swallowed, unfit, etc.).
#[derive(Clone, Copy)]
pub enum RjOutcome {
    /// Touched down; `on_target` is whether it was within the on-target window of the goal.
    Landed { on_target: bool, origin: Vec3, t: f32 },
    /// Never landed cleanly within the airtime budget.
    Overran { origin: Vec3, t: f32 },
    /// The stance never aligned enough to release the jump within the timeout.
    StanceTimeout,
    /// The jump was pressed but the bot never left the ground (swallowed).
    LiftoffTimeout,
    /// The fitness pre-check failed on arrival (no RL/rocket/health, or quad running).
    Unfit,
    /// An enemy appeared before commitment, so combat pre-empted (never fires under a puppet order).
    EnemyAbort,
    /// The pinned leg is no longer a solvable rocket jump (graph rebuilt out from under the attempt).
    LegVanished,
}

/// External puppet control state (rocket-jump test harness, see [`crate::control`]): the active order
/// (if any) plus the goto stall tracker. Default (`order == None`) is a normal autonomous bot.
#[derive(Default)]
pub struct Puppet {
    /// The scripted order overriding this bot's objective, or `None` for autonomous play.
    pub order: Option<ControlOrder>,
    /// Goto stall detection: the closest straight-line distance to the goto target reached so far, and
    /// when it last improved. No improvement for a while ⇒ the target is (currently) inaccessible.
    pub best_dist: f32,
    pub best_since: f32,
    /// Per-frame flight trace of a rocket-jump attempt: `(time, origin, velocity)` sampled each frame
    /// (post-move) while a RocketJump order is active, so the harness can compare the *actual* arc to
    /// the offline solve's prediction. Capped and cleared when the attempt's result is emitted.
    pub traj: Vec<(f32, Vec3, Vec3)>,
    /// FlyLink bring-up: whether the bot has left the ground since the order began (so a landing frame
    /// after takeoff — not the initial ground frames — terminates the attempt). Reset with the order.
    pub fly_airborne: bool,
    /// FlyLink bring-up: the horizontal speed captured at the takeoff frame (ground → airborne). 0 until
    /// takeoff. Reported in `fly_result` so the harness can read what the takeoff regime delivered.
    pub fly_takeoff_speed: f32,
}

/// A scripted control order for a puppeted bot (see [`crate::control`]). Lives here (not in
/// `control.rs`) so [`BotState`] carries it without a module cycle. All-`Copy`.
#[derive(Clone, Copy, PartialEq)]
pub enum ControlOrder {
    /// Stand still (between tests, or after an order completes).
    Hold,
    /// Navigate to a world position (normal pathfinding, no fighting). Arrival/stall reported.
    Goto { target: Vec3 },
    /// Fly a specific rocket-jump link (route pinned to it, repath suppressed). Result reported.
    RocketJump { link: u32 },
    /// Fly a specific non-RJ link (route pinned to it, repath suppressed) via the normal steer/bhop/
    /// speed-jump path — no rocket-jump driver. For harness bring-up of a hand-planted speed/curl jump.
    FlyLink { link: u32 },
}

/// The route-progress watchdogs carried on a [`BotState`]: three independent ways a bot detects it
/// has stopped making headway on its current route, each triggering a penalize-and-repath.
#[derive(Default)]
pub struct Watchdog {
    /// Displacement stuck-detector: where we were when last checked, and since when we've been there.
    pub stuck_origin: Vec3,
    pub stuck_since: f32,
    /// Path-progress watchdog: the closest straight-line distance to the goal we've reached on the
    /// current route, and when it last improved. No improvement for a while ⇒ the leg is failing in a
    /// way the displacement stuck-detector can't see (orbiting, wall-sliding) — penalize and re-path.
    pub progress_best: f32,
    pub progress_since: f32,
    /// Origin on the previous bot frame, to detect a teleport (a large instant jump) and re-path
    /// from the landing spot.
    pub last_origin: Vec3,
}

/// Which ballistic leg driver a failure belongs to — selects the per-driver consecutive-failure
/// counter in [`BotState::traversal_failed`].
#[derive(Clone, Copy)]
pub(crate) enum Driver {
    Hook,
    RocketJump,
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
