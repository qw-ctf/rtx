// SPDX-License-Identifier: AGPL-3.0-or-later

//! Per-bot AI state carried on the bot's client edict, plus the two small phase-machine enums the
//! hook and grenade drivers run on. Split out of `entity.rs` so the ~60-field blackboard lives with
//! the code that reads it (`crate::bot`), not with the engine-shared entity layout.

use std::collections::VecDeque;

use glam::Vec3;

use crate::navmesh::{CellId, LinkKind};

/// One fired steering watchdog, parked on the bot until [`crate::control`] drains it into a
/// `BotStall` event.
///
/// Deferred for the same reason [`RjTelemetry`] is: `crate::bot::steer` runs on `&mut BotState`
/// with no `&GameState`, so it cannot reach the control channel itself. Every field is a copy of
/// something the watchdog already had in hand — logging a stall costs no new work.
#[derive(Clone, Debug)]
pub struct StallRecord {
    pub t: f32,
    /// Which watchdog fired, and what it did about it. `&'static str` because both are drawn from
    /// a closed set the watchdogs spell out literally.
    pub reason: &'static str,
    pub action: &'static str,
    pub origin: Vec3,
    pub cell: CellId,
    pub goal_cell: CellId,
    pub goal_dist: f32,
    /// The route leg in force, and its kind — `None` off-route.
    pub link: Option<u32>,
    pub kind: Option<LinkKind>,
    pub speed: f32,
    /// The route as it stood *before* the watchdog cleared it, and the leg within it.
    pub route_len: u32,
    pub route_pos: u32,
}

/// How many [`StallRecord`]s a bot may hold. Deliberately tiny: with no control channel attached
/// nothing ever drains the buffer, so [`BotState::push_stall`] drops the oldest record instead of
/// letting an unobserved server grow a log it will never ship.
pub const STALL_BUF_CAP: usize = 4;

/// Client entities allocate private AI state on first mutation. Read-only access to unused
/// slots shares an immutable default; clearing an entity releases its private allocation.
#[cfg(feature = "netclient")]
#[derive(Default)]
pub struct EntityBotState(Option<Box<BotState>>);

/// Server-module entities retain their existing inline AI state and layout.
#[cfg(not(feature = "netclient"))]
pub type EntityBotState = BotState;

#[cfg(feature = "netclient")]
impl From<BotState> for EntityBotState {
    fn from(state: BotState) -> Self {
        Self(Some(Box::new(state)))
    }
}

#[cfg(feature = "netclient")]
impl core::ops::Deref for EntityBotState {
    type Target = BotState;

    fn deref(&self) -> &BotState {
        static EMPTY: std::sync::OnceLock<BotState> = std::sync::OnceLock::new();
        self.0
            .as_deref()
            .unwrap_or_else(|| EMPTY.get_or_init(BotState::default))
    }
}

#[cfg(feature = "netclient")]
impl core::ops::DerefMut for EntityBotState {
    fn deref_mut(&mut self) -> &mut BotState {
        self.0.get_or_insert_with(|| Box::new(BotState::default()))
    }
}

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
    /// Stall records the watchdogs parked this frame, awaiting the control channel. Bounded ring —
    /// see [`StallRecord`] / [`BotState::push_stall`].
    pub stall_events: VecDeque<StallRecord>,
    /// Per-bot flight recorder (`rtx_bot_debug`): a fixed, once-allocated ring of compact per-frame
    /// [`rtx_auditlog::AuditFrame`] sensor snapshots, dumped on demand via the control channel's
    /// `audit` verb instead of flooding the server console. Budget is `rtx_bot_auditlog` MB.
    pub audit: rtx_auditlog::Audit,
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
    /// World time this bot was last in a live fight — set every frame [`crate::bot::combat::engage`]
    /// runs. Drives the pack-protection weapon swap: you drop the pack of the weapon *in your hands*
    /// when you die, so a bot with no fight on should not be holding the rocket launcher.
    pub last_engaged: f32,
    /// Running tallies for the pack economy. See [`PackStats`].
    pub packs: PackStats,
    /// Expiring, timestamped team-strategy advice. The oracle worker never touches this directly;
    /// the main bot frame validates and delivers addressed nuggets.
    pub oracle: crate::bot::oracle::OracleInbox,
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
    /// The predictive hop plan (see [`crate::bot::hopsim`]) being flown on a ledge corridor: the aim
    /// point and gain whose rolled-out landing stays on the route. Planned on a grounded frame, held
    /// across the hop's flight, and cleared on landing. `None` off ledge corridors or when boxed.
    pub hop: Option<crate::bot::hopsim::HopPlan>,
    /// The certified walk-tracking latch (see [`crate::bot::walksim`]): the pursuit policy a pmove
    /// rollout proved keeps the bot on its route's floor. While it is live the tracker owns the
    /// grounded wish and the reactive edge margin/ledge brakes stand down; `None` means unproven
    /// ground, and the brakes are armed.
    pub walk: Option<WalkGuide>,
    /// Earliest time to re-attempt a walk certification after one found nothing — the boxed back-off,
    /// so a bot the rollout can't certify doesn't re-roll the whole failing fan every frame.
    pub walk_retry: f32,
    /// A committed [`LinkKind::SpeedJump`](crate::navmesh::LinkKind::SpeedJump) leg (a bhop run-up +
    /// leap) being flown. `None` = not on a speed jump. See [`Commit`].
    pub sj: Option<Commit>,
    /// A committed plain jump leg (JumpGap/DoubleJump) being flown. Pre-armed before objective
    /// selection, it freezes the route and locks out combat until a physical landing — so an enemy
    /// appearing at the lip or mid-arc cannot flip the goal and yank the bot off the jump.
    pub air: Option<AirCommit>,
    /// A committed haul-out: the `Swim` leg being climbed, and when it began.
    ///
    /// While set, the exit stance owns the view (see [`crate::bot::steer`]). It is a latch and not a
    /// per-frame test because the condition that starts it — the exit being above the bot — is one
    /// the bot is actively *rising through*, so re-deciding each frame snaps the view between the
    /// exit stance and the path look a few units apart, several times a second.
    pub water_exit: Option<Commit>,
    /// The direction the committed haul-out is facing, fixed when it armed.
    ///
    /// Recomputed per frame it is not stable: which directions `PM_CheckWaterJump` grants depends on
    /// the bot's own height, so a rising bot sees the set change under it and the stance swings between
    /// members of it — the climb oscillating in heading for no reason the bot could act on. The grant
    /// only has to hold at the instant of the jump, so choosing once and holding is both steadier and
    /// no less correct.
    pub water_exit_face: Option<glam::Vec2>,
    /// Rocket-jump traversal machine (see [`RjState`]): stance → jump → fire → ride the blast arc
    /// onto a high ledge.
    pub rj: RjState,
    /// External puppet control (rocket-jump test harness, see [`crate::control`]): a scripted order
    /// overriding the bot's own objective, plus the goto stall tracker. `order == None` for a normal,
    /// autonomous bot — the whole harness path is inert unless the control channel issues an order.
    pub puppet: Puppet,
    /// The navmesh cell the bot resolved itself to on the last planning frame. Diagnostic only —
    /// every route is searched from it, so the flight recorder needs it to tell "no path exists" from
    /// "planned from the wrong place". `None` before the first resolve, or when none was found.
    pub cell: Option<u32>,
    /// The LOD corridor interim the bot is committed to, and the goal it was chosen for.
    ///
    /// The corridor re-derives its interim from the bot's *current* cell on every repath, and two
    /// adjacent cells either side of a cluster boundary can yield interims in opposite directions —
    /// so a bot sitting on that boundary is handed first one and then the other and swims between
    /// them indefinitely. Holding the choice until it is reached turns that into a waypoint.
    pub interim: Option<(u32, u32)>,
    /// The cell the last A\* actually searched *to*, after the goal was redirected for reachability
    /// and (with LOD) truncated to a corridor interim. Diagnostic only.
    ///
    /// An empty route has three quite different causes — the repath never ran, it ran and the search
    /// came back with nothing, or it ran and something cleared the result afterwards — and the log
    /// cannot tell them apart from `route_len` alone. Recording the target the search was given, and
    /// stamping [`Self::replanned`] when one ran, separates all three.
    pub route_target: Option<u32>,
    /// The goal cell the last repath was handed, before any redirection. Diagnostic only.
    pub route_goal: Option<u32>,
    /// Whether a repath ran on this frame (cleared at the top of every steer pass).
    pub replanned: bool,
    /// What this frame's steering decided about the leg it is flying. Diagnostic only.
    ///
    /// A fumbled jump and a jump the bot never attempted look identical in the rest of the record —
    /// both are "grounded, then falling". Which one it was depends on the *kind* of the leg at the
    /// route cursor and on whether the takeoff gate was open, and neither is derivable from the
    /// route cursor alone (the leg's kind lives in the graph, and the gate is a velocity test against
    /// a waypoint that moves). See [`crate::bot::steer`]'s run-up gate.
    pub takeoff: TakeoffDiag,
}

/// One frame's view of the leg being flown and its takeoff gate. Diagnostic only.
#[derive(Clone, Copy, Default)]
pub struct TakeoffDiag {
    /// The kind of the link at the route cursor, if the route has one.
    pub leg: Option<rtx_nav::navmesh::LinkKind>,
    /// Speed carried *toward the steering waypoint* — what the run-up gate measures.
    pub runup: f32,
    /// Distance to that waypoint (the gate's own escape hatch reads this).
    pub wp: f32,
    /// Floor left ahead along travel before it drops away, or `None` when not probed (not grounded, or
    /// not on a jump leg). This is the quantity the takeoff is really racing.
    pub lip: Option<f32>,
    /// The run-up gate's verdict this frame.
    pub ok: bool,
    /// A speed jump's leap is held for its certified envelope this frame.
    pub sj_held: bool,
}

impl BotState {
    /// Park a watchdog firing for [`crate::control`] to drain, dropping the oldest record once the
    /// buffer is full. Never blocks and never grows: on a server with nothing listening this
    /// degrades to "the last few stalls", which is all the buffer is for.
    pub fn push_stall(&mut self, rec: StallRecord) {
        if self.stall_events.len() >= STALL_BUF_CAP {
            self.stall_events.pop_front();
        }
        self.stall_events.push_back(rec);
    }

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
        self.goal
            .avoid_items
            .iter()
            .any(|&(it, until)| it == item && now < until)
    }

    /// An item this bot was planning around has just left the floor. Drop a stale current/next
    /// reference immediately; if it was the first leg, preserve the already-planned continuation.
    /// The picker also gets a short avoid entry so an incidental touch cannot be selected straight
    /// back while the authoritative respawn state is settling.
    pub(crate) fn item_taken(&mut self, item: u32, now: f32, picked_by_me: bool, avoid_for: f32) {
        if picked_by_me {
            self.mark_avoid(item, now + avoid_for);
        }
        if self.goal.item == item {
            if self.goal.next_item != 0 {
                (self.goal.item, self.goal.item_cell, self.goal.commit) =
                    (self.goal.next_item, self.goal.next_cell, self.goal.next_commit);
                self.goal.since = now;
            } else {
                self.goal.item = 0;
                self.goal.item_cell = 0;
                self.goal.commit = GoalCommit::None;
            }
            (self.goal.next_item, self.goal.next_cell, self.goal.next_commit) = (0, 0, GoalCommit::None);
            self.goal.next_pick = now;
        } else if self.goal.next_item == item {
            (self.goal.next_item, self.goal.next_cell, self.goal.next_commit) = (0, 0, GoalCommit::None);
        }
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
    /// Which item this watch is over, and when it started — the bound on how long the watch may keep
    /// the give-up watchdog at bay. `0` when no watch is running.
    pub over: u32,
    pub began: f32,
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

/// A live certified walk plan (see [`crate::bot::walksim`]) plus what it was certified against, so
/// the steerer can tell each frame whether the proof still covers the ground under the bot: the
/// policy itself, when it was proven (the re-certification clock), and the leading route legs the
/// rollout saw. A repath swaps the route out from under the plan, and the current leg falling outside
/// that window is how it shows up.
#[derive(Clone, Copy)]
pub struct WalkGuide {
    pub plan: crate::bot::walksim::WalkPlan,
    pub since: f32,
    pub legs: [u32; WALK_LEGS],
    /// How many of `legs` are valid — link id 0 is a real id, so a sentinel won't do.
    pub n: u8,
}

/// How many leading route legs a [`WalkGuide`] remembers. The rollout looks 12 legs down the
/// corridor; the window only has to be long enough that ordinary leg advancement stays inside it.
pub const WALK_LEGS: usize = 8;

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
    /// goal re-pick. `Pickup` is a nearby lifesaving health/armor item; `Powerup` is a selected timed
    /// powerup. The lock ends only on touch, invalidation, route failure, or its watchdog.
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
    /// A backpack this bot has a *claim* on, and when the claim was stamped. Set when the bot fragged
    /// an armed enemy (secure the transfer) or when a teammate died nearby (reclaim it before the
    /// enemy does). `0` = none. See [`crate::bot::goals::PACK_HINT_SECS`].
    ///
    /// Why this exists: a dropped pack is otherwise just another ordinary-horizon item, and it loses
    /// to the hysteresis bonus on whatever the bot was already fetching. But the measurement is
    /// blunt — killing an armed enemy is worth +0.48 on its own and +0.99 when your side takes the
    /// pack, and it collapses to a spawn frag when *their* side reclaims it. Half the value of the
    /// best kill in the game is in the five seconds after it.
    pub pack_hint: u32,
    pub pack_hint_at: f32,
    /// How many times this bot has abandoned one live item goal for another. Not a bug on its own —
    /// re-picking is the whole design — but the *rate* is the tripwire for a valuation change that
    /// has made two pickups trade places every re-pick. An oscillating bot never trips a navigation
    /// watchdog, because it never stops moving.
    pub switches: u32,
}

/// How strongly the current item goal is committed. Ordinary goals remain freely re-scored;
/// lifesaving local pickups and timed powerups own movement until completion.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum GoalCommit {
    #[default]
    None,
    Pickup,
    Powerup,
}

/// Per-bot tallies for the pack economy — the scoreboard for the weapon-transfer market that the
/// frag count doesn't show. Read back over the control channel so a bot-vs-bot A/B can be judged on
/// the thing that was actually changed, rather than only on frags (which move slowly and noisily).
///
/// The measured stakes, per armed kill: +0.48 forward team frags for the denial alone, +0.99 when
/// the killing side also takes the pack, and about zero when the victim's side reclaims it. From the
/// dying side, holding the shotgun instead of the launcher halves what the death costs.
#[derive(Default)]
pub struct PackStats {
    /// Times this bot holstered a big gun because no fight was on. Counts *decisions* — the rising
    /// edge of the discipline's verdict — not frames it held true, which is what it used to count
    /// and which inflated the figure by roughly the length of every holster.
    pub sg_swaps: u32,
    /// Claims handed to this bot: a kill it made, or a teammate's death it was nearest to. The
    /// denominator `secured` needs — without it, a low secured count cannot distinguish "rarely
    /// claimed" from "claimed and never converted", which is exactly the ambiguity that hid a broken
    /// conversion path behind a plausible-looking number.
    pub hinted: u32,
    /// Claimed packs this bot went and collected.
    pub secured: u32,
    /// Times this bot died holding a big gun, dropping a pack an enemy could take. The number the
    /// swap discipline exists to drive down.
    pub fed: u32,
    /// Whether the holster discipline wanted the shotgun out last frame — the edge detector behind
    /// `sg_swaps`. Not reported; it is bookkeeping, not a statistic.
    pub swap_active: bool,
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
    /// Where the bot was when the stall clock last reset, so "is it making progress" can be answered
    /// by *movement* rather than by beating a record. A detour that temporarily increases the distance
    /// to the target is still progress; only a bot that has stopped moving is stuck.
    pub anchor: Vec3,
    /// Highest altitude the goto seeker has reached — a vertical climb (a spiral to a target above)
    /// holds XY distance near-constant while ascending correctly, so a rise counts as progress and
    /// resets the stall clock. Rebased to `-inf` when the goto order is (re)issued.
    pub best_z: f32,
    /// Per-frame flight trace of a rocket-jump attempt: `(time, origin, velocity)` sampled each frame
    /// (post-move) while a RocketJump order is active, so the harness can compare the *actual* arc to
    /// the offline solve's prediction. Capped and cleared when the attempt's result is emitted.
    pub traj: Vec<(f32, Vec3, Vec3, u8)>,
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
    /// Highest floor altitude reached on the current route (landings only). Gaining altitude counts as
    /// progress even when `progress_best` plateaus — so a spiral-staircase climb, whose horizontal
    /// orbit holds route-remaining near-constant, doesn't false-trip the progress watchdog.
    pub climb_best: f32,
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

#[cfg(test)]
mod tests {
    use super::*;

    #[cfg(feature = "netclient")]
    #[test]
    fn lazy_state_shares_only_immutable_defaults() {
        let mut a = EntityBotState::default();
        let b = EntityBotState::default();
        assert!(!a.is_bot);
        assert!(a.route.is_empty());
        assert!(a.0.is_none() && b.0.is_none(), "reads must not allocate per entity");
        assert!(core::ptr::eq(&*a, &*b));

        a.is_bot = true;
        a.route.push(42);
        assert!(a.0.is_some());
        assert!(!b.is_bot && b.route.is_empty());
        assert!(b.0.is_none());
        let address = &*a as *const BotState;
        a.client = 7;
        assert_eq!(&*a as *const BotState, address, "reuse the allocation on later writes");

        a = EntityBotState::default();
        assert!(a.0.is_none());
        assert!(!a.is_bot && a.route.is_empty());
        assert!(core::ptr::eq(&*a, &*b));
    }

    #[cfg(feature = "netclient")]
    #[test]
    fn explicit_state_survives_lazy_conversion_and_entity_clear() {
        let mut entity = crate::entity::Entity::default();
        entity.bot = BotState {
            is_bot: true,
            client: 3,
            route: vec![1, 2],
            ..Default::default()
        }
        .into();
        assert_eq!(entity.bot.client, 3);
        assert_eq!(entity.bot.route, [1, 2]);
        crate::bot::on_disconnect(&mut entity);
        assert!(entity.bot.0.is_none(), "disconnect releases the private state");
        entity.bot.route.push(5);
        entity.reset();
        assert!(entity.bot.0.is_none(), "entity reset releases state too");
        assert!(entity.bot.route.is_empty());
    }

    #[test]
    fn taken_goal_promotes_its_continuation_and_avoids_for_the_picker() {
        let mut bot = BotState::default();
        bot.goal.item = 10;
        bot.goal.item_cell = 100;
        bot.goal.commit = GoalCommit::Pickup;
        bot.goal.next_item = 20;
        bot.goal.next_cell = 200;
        bot.goal.next_commit = GoalCommit::Powerup;

        bot.item_taken(10, 5.0, true, 3.0);

        assert_eq!((bot.goal.item, bot.goal.item_cell), (20, 200));
        assert_eq!(bot.goal.commit, GoalCommit::Powerup);
        assert_eq!(bot.goal.next_item, 0);
        assert!(bot.is_avoided(10, 7.9));
        assert!(!bot.is_avoided(10, 8.0));
    }

    #[test]
    fn teammate_taking_a_watched_item_releases_the_stale_goal() {
        let mut bot = BotState::default();
        bot.goal.item = 10;
        bot.goal.item_cell = 100;
        bot.goal.commit = GoalCommit::Pickup;

        bot.item_taken(10, 5.0, false, 3.0);

        assert_eq!(bot.goal.item, 0);
        assert_eq!(bot.goal.item_cell, 0);
        assert_eq!(bot.goal.commit, GoalCommit::None);
        assert_eq!(bot.goal.next_pick, 5.0);
        assert!(!bot.is_avoided(10, 5.0));
    }
}
