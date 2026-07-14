// SPDX-License-Identifier: AGPL-3.0-or-later

//! Navmesh-driven bots. First deliverable (P4): multiple fake clients that **follow the
//! nearest human around the map** — full navigation over the auto-generated [navmesh], no
//! shooting. Goals and combat layer on later.
//!
//! A bot is a real client edict (`1..=maxclients`) spawned via `add_bot`; the engine runs its
//! [`BotState`]-driven usercmd through the same `SV_RunCmd`/`PM_PlayerMove` as a human, so
//! gravity, friction, stepping, and jumps come for free. Each server bot-frame
//! ([`GameState::start_frame`] with `is_bot_frame`) we recompute intent and emit one
//! `set_bot_cmd` per bot — the engine zeroes the cmd after running it, so it must be re-sent
//! every frame.
//!
//! [navmesh]: crate::navmesh


use glam::{Vec2, Vec3, Vec3Swizzles};

pub(crate) mod bhop;
mod combat;
pub(crate) mod goals;
mod grenade;
mod hook;
pub(crate) mod model;
pub(crate) mod perception;
mod population;
mod rj;
pub(crate) mod state;
mod steer;
mod vigil;

pub use population::manage_population;
pub(crate) use population::{drain_roster, RosterOp};
#[cfg(test)]
use population::bot_target;

use crate::bot::state::{
    AirCommit, BotState, CombatPosture, Commit, GoalCommit, GrenadePhase, HookPhase, RjPhase, Wander,
};
use crate::defs::{
    Bits, DeadFlag, Flags, Items, Solid, TakeDamage, Weapon, BUTTON_ATTACK, BUTTON_JUMP, VEC_VIEW_OFS,
};
use crate::entity::{EntId, Entity, Touch};
use crate::game::{cstring, GameState};
use crate::math::{angle_vectors, wrap180, yaw_of};
use crate::mode::BotIntent;
use crate::navmesh::{CellId, LinkCosts, LinkKind, NavGraph};

/// Impulse to select the shotgun (for shooting a health-gated button).
const IMPULSE_SHOTGUN: i32 = 2;
/// Impulse to select the grappling hook (for flying a hook leg).
const IMPULSE_GRAPPLE: i32 = 22;
/// Impulse to select the rocket launcher (for firing a rocket jump).
const IMPULSE_ROCKET: i32 = 7;

// --- grappling-hook leg execution ---

/// Within this of the hook leg's source cell counts as "in stance" to throw from.
const HOOK_STANCE: f32 = 24.0;
/// Throw once the smoothed view is within this many degrees of the anchor.
const HOOK_AIM_TOL: f32 = 2.0;
/// Give up aiming/throwing if the hook hasn't bitten within these windows.
const HOOK_AIM_TIMEOUT: f32 = 1.5;
const HOOK_FLIGHT_TIMEOUT: f32 = 1.0;
/// Abort a reel that runs this long (snagged, or a moving/none anchor) without releasing.
const HOOK_REEL_TIMEOUT: f32 = 3.0;
/// Abort if the live anchor sits this far from where the build expected it (hooked a player, a
/// moving door, or a sky brush the runtime rejected) — the solved arc no longer applies.
const HOOK_ANCHOR_DRIFT: f32 = 48.0;
/// Ballistic watchdog slack added to the solved airtime before we give up waiting to land.
const HOOK_BALLISTIC_SLACK: f32 = 1.0;

// --- rocket-jump leg execution ---

// Defaults for the `rtx_rj_*` knobs (see [`crate::cvars`]); the driver reads the live cvar each
// frame via [`RjKnobs`] and only falls back to these as the registered defaults. `pub(crate)` so the
// cvar table can seed from them and they can't drift from the documented default.
/// Within this of the launch cell counts as "in stance" to jump from — tighter than the hook's
/// (the solve's ±16u launch perturb bounds how far off the spot the arc still lands).
pub(crate) const RJ_STANCE: f32 = 16.0;
/// Fire the jump once the smoothed view is within this many degrees of the solved fire angles.
pub(crate) const RJ_AIM_TOL: f32 = 2.0;
/// Give up the stance if the RL/aim/ground alignment hasn't let the jump go within this.
pub(crate) const RJ_STANCE_TIMEOUT: f32 = 2.5;
/// If the bot is still on the ground this long after pressing jump, the jump was swallowed — abort.
pub(crate) const RJ_LIFTOFF_TIMEOUT: f32 = 0.3;
/// Ballistic watchdog slack added to the solved airtime before we give up waiting to land.
pub(crate) const RJ_BALLISTIC_SLACK: f32 = 1.0;

/// Advance to the next route leg once within this of the current waypoint (≈ ¾ of a grid).
const ARRIVE_RADIUS: f32 = 24.0;

/// Edge-safety (see the ledge brake / turn slowdown in [`steer`]). Below this speed a single frame
/// of wish can't carry a grounded bot over an edge, so the brake stays off (avoids a stand-still lock).
const LEDGE_MIN_SPEED: f32 = 60.0;
/// The ledge brake fires only when the bot's velocity has drifted this far off the corridor to its
/// waypoint (cosine — 0.5 ≈ 60°). An aligned Walk/Step leg has floor under it (the build's
/// `ground_along`), and a thin balance-beam path keeps velocity aligned, so both stay unbraked; only a
/// genuinely overshot corner heading off-route trips it. Must sit above the grid's 45° zigzag (cos ≈
/// 0.707), which reads as aligned.
const LEDGE_ALIGN_COS: f32 = 0.5;
/// Approach distance under which the arrival slowdown considers easing off for a sharp turn at a ledge.
const TURN_SLOW_RADIUS: f32 = 96.0;
/// Floor the arrival-slowdown wish scale never drops below (≈200 u/s of wish at `ARRIVE_RADIUS`).
const TURN_SLOW_MIN: f32 = 0.25;
/// The route must turn past this (cosine — 0.5 ≈ 60°, above the grid's constant 45° zigzag) at the next
/// leg for the arrival slowdown to engage, so flat straightaways and gentle zigzags keep full speed.
const TURN_SLOW_COS: f32 = 0.5;
/// How hard the lateral edge-margin nudge ([`hazard::edge_bias`]) bends a grounded bot's wish away from
/// a one-sided drop (blended with the waypoint direction; 0.6 ≈ a 31° lean). Enough to hold a bot off
/// the inner edge of an open-cored spiral without pinning it to the outer wall or stalling progress.
const EDGE_BIAS_WEIGHT: f32 = 0.6;

/// Outcome of a ballistic-phase landing check, shared by the hook and rocket-jump drivers: both fly
/// a frictionless arc that matches their solve, so the only questions are whether we've touched down
/// and whether we overran the predicted airtime without a clean landing.
pub(super) enum Landing {
    /// Still airborne within the airtime budget — keep riding the arc.
    Riding,
    /// Touched down; `on_target` is whether it was within 2·[`ARRIVE_RADIUS`] of the goal (so the
    /// driver can clear its consecutive-fails counter).
    Down { on_target: bool },
    /// Never landed cleanly within `airtime + slack` — give up and repath.
    Overran,
}

/// Classify a ballistic landing (see [`Landing`]). `elapsed` is time since the ballistic phase
/// began; a touchdown only counts after a 0.1 s settle so the takeoff frame isn't read as an instant
/// landing. `airtime_budget` is the solved airtime plus the driver's watchdog slack.
pub(super) fn ballistic_landing(origin: Vec3, target: Vec3, on_ground: bool, elapsed: f32, airtime_budget: f32) -> Landing {
    if on_ground && elapsed > 0.1 {
        Landing::Down {
            on_target: (origin.xy() - target.xy()).length() <= ARRIVE_RADIUS * 2.0,
        }
    } else if elapsed > airtime_budget {
        Landing::Overran
    } else {
        Landing::Riding
    }
}
/// Stop closing once this near the followed human, so bots tail rather than shove.
const POLITE_DIST: f32 = 64.0;
/// Seconds of air left at which a submerged bot drops everything and makes for the surface. The tank
/// is 12s (see [`crate::client::movement`]); triggering with 5 left leaves ~1100u of swimming
/// (0.7·MAX_SPEED for the panic window) before drown damage — ample slack for any id1 water pocket.
const DROWN_PANIC_SECS: f32 = 5.0;
/// How long a bot must stand *in* lava/slime before the escape reflex hijacks its goal toward the
/// nearest safe cell. Short — the game burns from the first frame (`apply_liquid_damage`) — but not
/// zero: a bot deliberately crossing a slime moat or lava bridge is off it within a damage tick or
/// two, so gating on dwell lets those intentional crossings through while still rescuing a bot parked
/// on liquid by a plat standoff, a polite yield, or a knockback.
const BURN_PANIC_SECS: f32 = 0.75;
/// How long the anti-drown surface target is reused before re-flooding the graph — a short cache so
/// a drowning bot doesn't run a full Dijkstra every frame while it swims out. Shared by the burn
/// escape reflex, which floods the same way.
const SURFACE_CACHE_TTL: f32 = 0.5;
/// Minimum seconds between A* re-paths (the human keeps moving).
const REPATH_INTERVAL: f32 = 0.4;
/// Stuck detector: if we move less than this over `STUCK_TIME`, jump and re-path.
const STUCK_MOVE: f32 = 16.0;
const STUCK_TIME: f32 = 0.7;
/// A plat ride is "done" once we've risen to within this of the exit-floor height.
const PLAT_RISE_TOL: f32 = 18.0;
/// While an upcoming leg boards a raised plat, hold this far outside the plat's XY footprint.
/// The inner trigger is the footprint shrunk 25u in XY, so a 40u standoff keeps the bot's
/// 16u-half-width body well clear of holding the lift up.
const PLAT_STANDOFF: f32 = 40.0;
/// Engage the standoff hold only once this near the footprint — further out, just keep walking in.
const PLAT_ENGAGE: f32 = 96.0;
/// Give up on a raised plat that never descends (a camped lift, or a targeted plat that only its
/// own trigger lowers): strike the ride link and re-path.
const PLAT_WAIT_TIMEOUT: f32 = 8.0;
/// How many upcoming route legs to scan for a plat tag — the walk-in legs before the tagged
/// boarding leg sit within a cell or two of it, so a small window covers the approach.
const PLAT_LOOKAHEAD: usize = 4;
/// Minimum seconds between item-goal re-selections (so a bot commits to a pickup rather than
/// flip-flopping between two of similar worth each frame).
const GOAL_SELECT_INTERVAL: f32 = 1.5;
/// How long after the last line-of-sight frame the nav view may still point at a Fight enemy's
/// live origin. Set equal to combat's `HOLD_ANGLE_TIME` (the 2s corner-hold in `combat::engage`):
/// while that hold owns the view it overrides `look` anyway, so the handoff is seamless — hold the
/// corner for 2s, then look where we're travelling instead of tracking the enemy through geometry.
const LOOK_LOS_GRACE: f32 = 2.0;
/// Goal watchdog: if a bot has been chasing the *same* item this long (and isn't detouring to open
/// a gate) without ever collecting it, that item is effectively unreachable for it — abandon it and
/// avoid it for `GOAL_AVOID_TIME`, then retry. Time-based (not distance) so a legitimate route that
/// detours away — riding an elevator, walking to a teleporter — isn't mistaken for being stuck.
const GOAL_GIVEUP_TIME: f32 = 10.0;
const GOAL_AVOID_TIME: f32 = 12.0;
/// Airborne jump commitment settle grace: after the commitment has observed flight, require this
/// much elapsed time before a grounded frame counts as touchdown. A bot that never got airborne is
/// run-up/stalled, not landed, and remains committed until the watchdog.
const AIR_COMMIT_GRACE: f32 = 0.2;
/// And if we're *still airborne* after this long, no real jump arc lasts that long — we fell into the
/// void or wedged. Abandon the leg (penalize + re-path), the speed-jump watchdog's shape.
const AIR_COMMIT_MAX: f32 = 2.5;
/// Gate errand give-up: if a bot goes this long without getting any closer to a gate's button, the
/// button is out of reach (or unusable) — abandon and avoid the gate for `GATE_AVOID_TIME`. Keyed
/// on lack of progress, not elapsed time, so a button that's just far away is still pursued.
const GATE_GIVEUP_TIME: f32 = 4.0;
const GATE_AVOID_TIME: f32 = 6.0;
/// Failed-link penalty: when a leg fails (stuck, speed-jump stall, hook give-up) it gets a per-bot
/// travel-time surcharge in this bot's next A* so the planner *diverts* around it instead of handing
/// back the identical route to retry forever. Surcharge grows with repeat strikes (`strikes²·STEP`,
/// capped) and expires after `PENALTY_TTL` with no fresh strike. The cap sits far below the navmesh's
/// closed-gate penalty, so it reshapes a route without ever forcing one through a shut door — and,
/// being finite, never makes a cell unreachable (a lone corridor is still taken, just last).
const PENALTY_STEP: f32 = 4.0;
const PENALTY_CAP: f32 = 30.0;
const PENALTY_TTL: f32 = 8.0;
/// Path-progress watchdog: if the straight-line distance to the goal hasn't improved by at least
/// `PROGRESS_EPS` for this long, the current leg is failing in a way the displacement stuck-detector
/// can't see (e.g. orbiting a pillar at full speed) — penalize the leg and re-path.
const PROGRESS_STALL_TIME: f32 = 2.5;
const PROGRESS_EPS: f32 = 32.0;

/// One bot's accumulated frame command: what navigation proposes and the combat/grenade overlays
/// mutate in turn, before the aim spring and view projection in `run_bot` turn it into the final
/// `set_bot_cmd`. `look` (desired view angles, pre-spring) and `move_world` (desired world-space
/// velocity) are deliberately decoupled — the bot can run one way while looking another.
pub(crate) struct BotCmd {
    pub look: Vec3,
    pub move_world: Vec3,
    pub buttons: i32,
    pub impulse: i32,
}

// --- per-frame driving (P2 follow + P4 behavior) ---

/// Drive every bot for this server bot-frame: pick the nearest human, path to them, and emit
/// each bot's usercmd. Skipped entirely without a navmesh.
pub fn run_bots(game: &mut GameState) {
    if !game.nav.is_loaded() {
        return;
    }
    let maxclients = game.host().cvar(c"maxclients") as i32;
    for i in 1..=maxclients as u32 {
        let e = EntId(i);
        if game.entities[e].bot.is_bot && game.entities[e].in_use {
            bot_pickup_items(game, e);
            run_bot(game, e);
        }
    }
}

/// Generous fixed pickup half-extents (the player hull ±16 plus the item trigger ±16, with a
/// little slack). We use a fixed box rather than the item's engine-side `mins`/`maxs` because
/// `set_size` may not sync those to our shadow `EntVars`, which would make the test a degenerate
/// point and almost never fire.
const PICKUP_XY: f32 = 40.0;
const PICKUP_Z: f32 = 48.0;
/// How long a just-collected goal item stays on the avoid ring, so the bot re-picks a fresh goal
/// instead of re-fixating on a pickup that respawns (or lingers solid) the same second.
const PICKUP_AVOID_TIME: f32 = 3.0;

/// Whether a trigger touch is a player pickup fake clients must receive manually. Keeping this an
/// explicit allow-list prevents the bot pass from firing doors, teleports, damage, or map scripts.
fn bot_pickup_touch(touch: Touch) -> bool {
    matches!(
        touch,
        Touch::ItemHealth
            | Touch::ItemArmor
            | Touch::ItemWeapon
            | Touch::ItemAmmo
            | Touch::ItemPowerup
            | Touch::Backpack
            | Touch::Flag
            | Touch::Rune
    )
}

/// Manually collect any item the bot is standing on. The engine doesn't run the trigger-touch
/// phase for `SetBotCMD` fake clients the way it does for human `SV_RunCmd`, so a bot would walk
/// onto a pickup and never actually take it — it'd just keep wanting it and circle. We replicate
/// the touch here, guarded by `solid == Trigger` (a respawning item that's already been taken is
/// non-solid → skipped) so this can't double-grant even if an engine *does* fire the touch. The
/// scan intentionally includes dynamic CTF flags/runes and backpacks, none of which can live in the
/// static nav goal catalog.
fn bot_pickup_items(game: &mut GameState, e: EntId) {
    if !game.entities[e].is_alive() {
        return;
    }
    let origin = game.entities[e].v.origin;
    // Gather first (immutable borrow of entities), then fire touches (needs `&mut game`). A Quake
    // server has a small fixed edict table; scanning it once per bot frame is both bounded and the
    // only correct way to catch moving/dynamically-created pickup entities.
    let hits: Vec<EntId> = game
        .entities
        .iter()
        .enumerate()
        .filter_map(|(i, it)| {
            (it.v.solid == Solid::Trigger && bot_pickup_touch(it.touch) && on_item(origin, it.v.origin))
                .then_some(EntId(i as u32))
        })
        .collect();
    let now = game.time();
    let goal_item = game.entities[e].bot.goal.item;
    let next_item = game.entities[e].bot.goal.next_item;
    let next_cell = game.entities[e].bot.goal.next_cell;
    let next_commit = game.entities[e].bot.goal.next_commit;
    let hold_item = game.entities[e].bot.goal.hold_item;
    let holding = hold_item != 0 && now < game.entities[e].bot.goal.hold_until;
    for item in hits {
        // Handoff hold: don't pick up the weapon we're reserving for a powerup-carrying teammate
        // (the engine never fires trigger touches for fake clients, so skipping here is sufficient).
        if holding && item.0 == hold_item {
            continue;
        }
        if game.entities[item].v.solid == Solid::Trigger {
            game.run_touch(item, e);
            // Just collected our goal item — briefly avoid it so an instant-respawn pickup (or a
            // weapons-stay trigger that lingers solid) can't recapture the goal slot the same second;
            // the slot frees up for the next-best pickup instead of re-fixating in place.
            if item.0 == goal_item {
                let b = &mut game.entities[e].bot;
                b.mark_avoid(item.0, now + PICKUP_AVOID_TIME);
                if next_item != 0 {
                    (b.goal.item, b.goal.item_cell, b.goal.commit) = (next_item, next_cell, next_commit);
                    b.goal.since = now;
                    b.goal.next_pick = now + GOAL_SELECT_INTERVAL;
                } else {
                    b.goal.item = 0;
                    b.goal.commit = GoalCommit::None;
                    b.goal.next_pick = now;
                }
                (b.goal.next_item, b.goal.next_cell, b.goal.next_commit) = (0, 0, GoalCommit::None);
            }
        }
    }
}

/// Whether a bot at `bot_origin` is close enough to an item at `item_origin` to collect it.
fn on_item(bot_origin: Vec3, item_origin: Vec3) -> bool {
    let d = item_origin - bot_origin;
    d.x.abs() <= PICKUP_XY && d.y.abs() <= PICKUP_XY && d.z.abs() <= PICKUP_Z
}

/// What the bot is trying to do this frame — the output of [`resolve_objective`]. All-Copy so the
/// spine can read a few fields and still hand the whole thing to [`steer::steer`] via `SteerCtx`.
#[derive(Clone, Copy)]
struct Objective {
    /// Currently flying (or committed to) a grappling-hook leg.
    hooking: bool,
    /// Currently committed to a speed-jump leg (route frozen, like `hooking`).
    on_sj: bool,
    /// Currently flying (or committed to) a rocket-jump leg (route frozen, like `hooking`).
    on_rj: bool,
    /// A mode Fight intent's enemy to engage, if any.
    enemy: Option<EntId>,
    /// Chasing a committed item goal (idle pickup or greedy combat detour).
    chasing: bool,
    /// The item chase owns movement until touch/invalidation. Combat may aim and fire, but must not
    /// strafe or clear navigation's jump button while this is true.
    item_committed: bool,
    /// Stop [`POLITE_DIST`] short of the destination — set only when tailing a human (the
    /// pacifist override / no-goal follow fallback) or idle-roaming. A mode-issued Move must be
    /// walked all the way onto: a race checkpoint is a hull-sized touch box, and stopping 64u
    /// out would park the runner just outside it forever.
    polite: bool,
    /// Standing vigil over an uncollectable goal item (cruising/scanning near it) — exempts the
    /// stuck/progress watchdogs and drives the eyes with the scan sweep.
    vigil: bool,
    /// Where to navigate toward this frame.
    target_origin: Vec3,
    /// The navmesh cell of the item goal, when chasing one (skips a `nearest` lookup).
    item_cell: Option<CellId>,
    /// Fighter eye point an arena audience bot holds its eyes on (a `Spectate` intent); `None`
    /// otherwise, leaving the eyes on the walk corridor.
    watch_point: Option<Vec3>,
    /// Anti-drown override active: fully submerged and low on air, so the normal goal is replaced
    /// with a dash to the nearest breathable cell (and combat yields movement to navigation). Set in
    /// `run_bot` once the graph is borrowed, not by [`resolve_objective`].
    surfacing: bool,
    /// While `surfacing`, whether open water sits overhead so holding jump swims the bot up (an open
    /// pool). False in a roofed tunnel, where pressing up only pins it to the ceiling — there it
    /// swims *out* to the breathing spot on the navigation move alone.
    swim_up: bool,
    /// Puppet rocket-jump order (test harness): the link to pin the route to and fly. `Some` only
    /// while a [`ControlOrder::RocketJump`](crate::bot::state::ControlOrder) is active — `steer` then
    /// pins `route = [link]` and suppresses repath so the driver can take the jump. `None` otherwise.
    order_link: Option<u32>,
}

/// Resolve what this bot pursues this frame: reconcile a stale hook, ask the mode for an intent
/// (or the pacifist override), (re)pick an item goal on a slow cadence, and settle on the world
/// target to steer toward. Runs while `&mut game` is free — before the navmesh borrow.
/// The immutable per-frame snapshot of a bot's edict — read once so the later `&mut bot` /
/// `&mut nav` borrows in `run_bot` don't have to re-borrow the entity to read it (the grapple
/// fields are set in the previous frame's PlayerPreThink, so they're stable across this frame).
#[derive(Clone, Copy)]
struct Sense {
    host: crate::host::HostApi,
    now: f32,
    frametime: f32,
    msec: i32,
    origin: Vec3,
    v_angle: Vec3,
    client: i32,
    weapon: Weapon,
    on_ground: bool,
    /// Waist-deep or deeper (`waterlevel >= 2`): the engine's pmove swims here — jumps become
    /// swim-up strokes — so bunnyhopping is impossible. Vetoes the bhop controller.
    in_water: bool,
    /// Fully submerged (`waterlevel == 3`, eyes underwater): the only state that *drowns* — the air
    /// tank counts down here (see [`crate::client::movement`]). Drives the anti-drown override.
    submerged: bool,
    /// Standing in lava or slime deep enough to burn (`waterlevel >= 1` on a lava/slime watertype —
    /// the exact gate `apply_liquid_damage` uses). Drives the escape reflex that hijacks the goal
    /// toward safe ground; see [`combat::is_burning`].
    burning: bool,
    /// Seconds of air left before drowning (`combat.air_finished - now`): the 12s tank, ticking down
    /// only while `submerged`, refreshed on every surfacing breath. Read only when `submerged`.
    air_left: f32,
    alive: bool,
    vz: f32,
    air_jumped: bool,
    enemy_seen_time: f32,
    v_xy: glam::Vec2,
    speed: f32,
    grapple_hook: EntId,
    has_grapple: bool,
    hook_out: bool,
    on_hook: bool,
    anchor: Vec3,
    reel_half_step: f32,
    // Rocket-jump fitness + fire gating.
    attack_finished: f32,
    has_rl: bool,
    ammo_rockets: f32,
    health: f32,
    armortype: f32,
    armorvalue: f32,
    quad: bool,
    /// Live rocket-jump tuning knobs (`rtx_rj_*`), read once here so the driver and `emit` stay pure
    /// snapshot consumers. See [`rj::RjKnobs`].
    rj_knobs: rj::RjKnobs,
}

fn sense(game: &GameState, e: EntId) -> Sense {
    let host = *game.host();
    let now = game.time();
    let frametime = game.globals.frametime;
    let msec = ((frametime * 1000.0) as i32).clamp(1, 100);

    let origin = game.entities[e].v.origin;
    let v_angle = game.entities[e].v.v_angle;
    let client = game.entities[e].bot.client;
    let weapon = game.entities[e].v.weapon;
    let on_ground = game.entities[e].v.flags.has(Flags::ONGROUND);
    // Swimming (waterlevel >= 2): matches the swim gate in `player_jump` and combat's `swimming`.
    let in_water = game.entities[e].v.waterlevel >= 2.0;
    // Fully under (waterlevel == 3) is the drowning state; snapshot the air tank for the override.
    let submerged = game.entities[e].v.waterlevel >= 3.0;
    // Feet in lava/slime (waterlevel >= 1 on that watertype) — burning now, the escape-reflex trigger.
    let burning = combat::is_burning(&game.entities[e].v);
    let air_left = game.entities[e].combat.air_finished - now;
    let alive = game.entities[e].is_alive();
    // Vertical speed and whether the once-per-air-travel double jump is still available — snapshot
    // now, since the `&mut bot` binding below blocks reading the edict during the move logic.
    let vz = game.entities[e].v.velocity.z;
    let air_jumped = game.entities[e].combat.air_jumped;
    // When combat last had line of sight (see `combat::engage`) — snapshot for the bhop veto.
    let enemy_seen_time = game.entities[e].bot.seen.time;
    let v_xy = game.entities[e].v.velocity.xy();
    let speed = v_xy.length();
    // Snapshot grapple state up front (it's set in the previous frame's PlayerPreThink, so it's
    // stable across this bot frame) — lets the hook driver read it without re-borrowing the edict
    // while the `&mut bot` binding is live. `anchor` is meaningful only once `on_hook`.
    let grapple_hook = EntId(game.entities[e].grapple.hook);
    let has_grapple = game.entities[e].v.items.has(Items::GRAPPLE);
    let hook_out = game.entities[e].grapple.hook_out;
    let on_hook = game.entities[e].grapple.on_hook;
    let anchor = if hook_out {
        game.entities[grapple_hook].v.origin
    } else {
        Vec3::ZERO
    };
    // Live reel speed, for the release-crossing prediction (half a frame of lookahead).
    let reel_half_step = crate::navmesh::HOOK_PULL_BASE * host.cvar(c"rtx_hook_pull") * game.globals.frametime * 0.5;
    // Rocket-jump fitness + fire gating (read here so the driver stays a pure snapshot consumer).
    let attack_finished = game.entities[e].combat.attack_finished;
    let has_rl = game.entities[e].v.items.has(Items::ROCKET_LAUNCHER);
    let ammo_rockets = game.entities[e].v.ammo_rockets;
    let health = game.entities[e].v.health;
    let armortype = game.entities[e].v.armortype;
    let armorvalue = game.entities[e].v.armorvalue;
    let quad = game.entities[e].combat.super_damage_finished > now;
    // Live rocket-jump knobs — the physical windows clamp to ≥ 0 (a negative stance/timeout is
    // nonsense); the two biases are deliberately left signed so they can pull the solved value either
    // way. Read every frame so the harness can retune between attempts with no rebuild.
    let rj_knobs = rj::RjKnobs {
        stance: host.cvar(c"rtx_rj_stance").max(0.0),
        aim_tol: host.cvar(c"rtx_rj_aim_tol").max(0.0),
        stance_timeout: host.cvar(c"rtx_rj_stance_timeout").max(0.0),
        liftoff_timeout: host.cvar(c"rtx_rj_liftoff_timeout").max(0.0),
        ballistic_slack: host.cvar(c"rtx_rj_ballistic_slack").max(0.0),
        delay_bias: host.cvar(c"rtx_rj_delay_bias"),
        pitch_bias: host.cvar(c"rtx_rj_pitch_bias"),
    };
    Sense {
        host, now, frametime, msec, origin, v_angle, client, weapon, on_ground, in_water, submerged, burning, air_left, alive, vz, air_jumped, enemy_seen_time, v_xy, speed, grapple_hook, has_grapple, hook_out, on_hook, anchor, reel_half_step,
        attack_finished, has_rl, ammo_rockets, health, armortype, armorvalue, quad, rj_knobs,
    }
}

/// Whether a mode intent is indivisible and therefore forbids every item-plan detour. Race uses
/// `Move(next checkpoint)`, so this is the small policy seam that keeps a stale backpack or queued
/// pickup from surviving a restart and replacing the course objective.
fn hard_mode_objective(intent: Option<BotIntent>) -> bool {
    matches!(intent, Some(BotIntent::Move(_) | BotIntent::Spectate { .. }))
}

fn resolve_objective(game: &mut GameState, e: EntId, now: f32, origin: Vec3, client: i32) -> Objective {
    let host = *game.host();
    // Hook invariant net: if we're mid-hook but no longer hold the grapple (a mode loadout stripped
    // it, e.g. Rocket Arena), abandon the traversal cleanly — release any live hook and reset the
    // phase. Runs before the nav borrow, where `&mut game` is free. Other aborts (leg changed, hook
    // vanished, timeouts) are handled inside the hook driver below.
    if game.entities[e].bot.hook.phase != HookPhase::Idle && !game.entities[e].v.items.has(Items::GRAPPLE) {
        if game.entities[e].grapple.hook_out {
            let hook = EntId(game.entities[e].grapple.hook);
            game.reset_grapple(hook);
        }
        game.entities[e].bot.hook.phase = HookPhase::Idle;
        game.entities[e].bot.hook.fails = 0;
    }
    let hooking = game.entities[e].bot.hook.phase != HookPhase::Idle;
    // Rocket-jump invariant net: mid-RJ but the RL or its ammo is gone (dropped, spent, stripped) —
    // abort so the bot doesn't jump into a shot it can't fire. Timeouts/misfires are the driver's.
    if game.entities[e].bot.rj.phase != RjPhase::Idle
        && (!game.entities[e].v.items.has(Items::ROCKET_LAUNCHER) || game.entities[e].v.ammo_rockets < 1.0)
    {
        game.entities[e].bot.rj.phase = RjPhase::Idle;
        game.entities[e].bot.rj.fails = 0;
    }
    // On a speed-jump leg the route must be frozen: the link's `from` is the runway start, now behind
    // the bot, so a repath would drop the link and turn the bot around at speed. Treated like `hooking`.
    let on_sj = game.entities[e].bot.sj.is_some();
    // A rocket-jump leg freezes the route the same way (stance stands still, the arc flies fast).
    let on_rj = game.entities[e].bot.rj.phase != RjPhase::Idle;

    // Puppet override (rocket-jump test harness, see [`crate::control`]): a scripted order supersedes
    // the whole intent/item/perception pipeline. Runs *after* the invariant nets above (a stripped RL
    // must still abort a rocket jump) but before the mode is consulted — combat is off (`enemy: None`)
    // and item chasing is suppressed. Inert for a normal bot (`order == None`). The `RocketJump` order
    // additionally sets `order_link`, which `steer` uses to pin the route to that one link.
    if let Some(order) = game.entities[e].bot.puppet.order {
        use crate::bot::state::ControlOrder;
        game.entities[e].bot.goal.item = 0; // no item chase under a puppet order
        game.entities[e].bot.goal.next_item = 0;
        game.entities[e].bot.goal.commit = GoalCommit::None;
        game.entities[e].bot.goal.next_commit = GoalCommit::None;
        let (target_origin, item_cell, order_link) = match order {
            ControlOrder::Hold => (origin, None, None),
            ControlOrder::Goto { target } => (target, None, None),
            ControlOrder::RocketJump { link } | ControlOrder::FlyLink { link } => {
                // The graph is guaranteed present (the control command validated the link before
                // issuing the order); target the link's destination ledge. FlyLink differs only in
                // that no RJ driver runs — the pinned leg is flown by the normal steer/bhop path.
                let g = game.nav.graph.as_ref().unwrap();
                let cell = g.link_target(link);
                (g.cell_origin(cell), Some(cell), Some(link))
            }
        };
        return Objective {
            hooking, on_sj, on_rj,
            enemy: None,
            chasing: false,
            item_committed: false,
            polite: false,
            vigil: false,
            target_origin,
            item_cell,
            watch_point: None,
            surfacing: false,
            swim_up: false,
            order_link,
        };
    }

    // Ask the active mode for this bot's intent. A round mode (Rocket Arena) returns Fight/Move to
    // drive combat or audience-roaming; FFA hunts the nearest player. Every mode-specific bot
    // adaptation lives behind this one hook — the rest of run_bot stays mode-agnostic and reusable.
    let mode = game.mode;
    // Whether to stop politely short of the destination (see `Objective::polite`) — flagged at
    // the arms that tail a human or roam, never for a mode-issued intent.
    let mut polite = false;
    let intent = if crate::mode::team::benched(game, e) {
        // Benched spectator (structured match, off the locked roster): stroll the stands, no fighting.
        Some(BotIntent::Move(crate::mode::wander_point(game, e, "info_player_deathmatch", |_| None)))
    } else if host.cvar_bool(c"rtx_bot_pacifist") && mode.allows_bot_pacifist_override() {
        // Global override where the mode permits it: don't fight — just tail the nearest human.
        // Race refuses this because its hard Move intent is the ordered checkpoint/finish route.
        polite = true;
        nearest_human(game, e).map(|h| BotIntent::Move(game.entities[h].v.origin))
    } else {
        mode.bot_intent(game, e)
    };
    // Item goal (P5): re-pick the best reachable pickup on a slow cadence, and drop a chosen item
    // once it's been grabbed (no longer available/respawning soon) so the bot moves on.
    //
    // Three cases feed the same `goal_item` slot:
    //  - No mode intent → the full item brain (idle pickup, or follow-a-human fallback below).
    //  - A **Fight** intent always considers major powerups/runes; `rtx_bot_greed` additionally lets
    //    a compelling ordinary weapon/health/armor plan compete. `enemy` stays set, so combat can
    //    keep aiming/firing while a completion-critical pickup owns movement.
    //  - A soft **Advance** intent (CTF lane/escort) runs the full item brain, then resumes its lane.
    //  - A hard **Move/Spectate** intent (flag run/return, race, arena audience) forbids item detours.
    // Perception gate: a mode nominates a target, but a human-like bot only acts on one it has
    // actually perceived. Downgrade a Fight the bot is *unaware* of to no intent (so it patrols and
    // collects until real contact — the biggest believability change); keep an *aware but unseen*
    // target as Fight but hunt where it was last seen (`combat_last_seen`) rather than its live
    // origin; leave a *visible* target as-is. Non-Fight intents (Move/None) aren't perceived.
    let (intent, combat_last_seen) = match intent {
        Some(BotIntent::Fight(en)) => match perception::perceive(game, e, en, now) {
            perception::Awareness::Unaware => (None, None),
            perception::Awareness::Known { last_seen } => (Some(BotIntent::Fight(en)), Some(last_seen)),
            perception::Awareness::Visible => (Some(BotIntent::Fight(en)), None),
        },
        other => {
            // No combat target this frame: clear the visibility clock so the next engagement starts
            // with loose first-glimpse aim rather than reading a stale, long-settled duration.
            game.entities[e].bot.percept.vis_since = 0.0;
            (other, None)
        }
    };

    let greedy = matches!(intent, Some(BotIntent::Fight(_))) && host.cvar_bool(c"rtx_bot_greed");

    // A mode-issued Move/Spectate is a hard objective (flag running, race checkpoint, arena
    // audience). It supersedes an old item completion inherited from a previous Fight/idle frame.
    if hard_mode_objective(intent) {
        let b = &mut game.entities[e].bot;
        b.goal.item = 0;
        b.goal.next_item = 0;
        b.goal.commit = GoalCommit::None;
        b.goal.next_commit = GoalCommit::None;
        b.goal.next_pick = now;
    }

    // Completion locks end only when the item is no longer a valid pickup (touch handling clears
    // the common success path immediately). Validate before any periodic selection can replace it.
    let committed_item = game.entities[e].bot.goal.item;
    let committed = game.entities[e].bot.goal.commit != GoalCommit::None;
    if committed && (committed_item == 0 || !game.item_goal_valid(e, EntId(committed_item), now)) {
        let b = &mut game.entities[e].bot;
        b.goal.item = 0;
        b.goal.next_item = 0;
        b.goal.commit = GoalCommit::None;
        b.goal.next_commit = GoalCommit::None;
        b.goal.next_pick = now;
    }

    let traversal_committed = hooking
        || on_sj
        || on_rj
        || game.entities[e].bot.air.is_some();

    // Strategic recovery: relative strength/firepower and critical health can make a reachable
    // health/armor/weapon pickup own movement for several seconds. A powerup plan normally wins;
    // only a genuinely critical (≤20 hp) bot inserts recovery ahead of it and preserves the old
    // target as the continuation.
    if let Some(BotIntent::Fight(en)) = intent {
        if !traversal_committed {
            let previous = game.entities[e].bot.posture;
            let (posture, recovery) = game.recovery_decision(e, en, now, previous);
            game.entities[e].bot.posture = posture;
            let may_preempt = game.entities[e].bot.goal.commit != GoalCommit::Powerup
                || game.entities[e].v.health <= 20.0;
            if may_preempt {
                if let Some((item, cell)) = recovery {
                    let b = &mut game.entities[e].bot;
                    let preserve = (b.goal.commit == GoalCommit::Powerup && b.goal.item != 0)
                        .then_some((b.goal.item, b.goal.item_cell, GoalCommit::Powerup));
                    if item.0 != b.goal.item {
                        b.goal.since = now;
                    }
                    (b.goal.item, b.goal.item_cell, b.goal.commit) = (item.0, cell, GoalCommit::Pickup);
                    (b.goal.next_item, b.goal.next_cell, b.goal.next_commit) =
                        preserve.unwrap_or((0, 0, GoalCommit::None));
                    b.goal.next_pick = now + GOAL_SELECT_INTERVAL;
                }
            }
        }
    } else {
        game.entities[e].bot.posture = CombatPosture::Hold;
    }

    // Fast local survival pass. It is independent of greed and runs while idle or fighting, but not
    // during a ballistic traversal whose route/view already have an indivisible owner.
    let urgent_allowed = matches!(intent, None | Some(BotIntent::Fight(_) | BotIntent::Advance(_)))
        && !traversal_committed;
    // A timed powerup commitment normally freezes item selection, but a known respawn wait is spare
    // route time: use it to collect a nearby health/armor/weapon only when the complete two-leg path
    // still preserves the powerup arrival. Keep the powerup as a completion-critical continuation,
    // so touching the bridge item immediately resumes the quad/pent run.
    if urgent_allowed
        && game.entities[e].bot.goal.commit == GoalCommit::Powerup
        && now >= game.entities[e].bot.goal.next_urgent
    {
        let powerup = EntId(game.entities[e].bot.goal.item);
        let powerup_cell = game.entities[e].bot.goal.item_cell;
        let pick = game.select_powerup_bridge_item(e, powerup, powerup_cell, now);
        let b = &mut game.entities[e].bot;
        b.goal.next_urgent = now + 0.2;
        if let Some((item, cell)) = pick {
            b.goal.since = now;
            (b.goal.item, b.goal.item_cell, b.goal.commit) =
                (item.0, cell, GoalCommit::Pickup);
            (b.goal.next_item, b.goal.next_cell, b.goal.next_commit) =
                (powerup.0, powerup_cell, GoalCommit::Powerup);
            b.goal.next_pick = now + GOAL_SELECT_INTERVAL;
        }
    }
    if urgent_allowed
        && game.entities[e].bot.goal.commit == GoalCommit::None
        && now >= game.entities[e].bot.goal.next_urgent
    {
        let pick = game.select_urgent_local_item(e);
        let b = &mut game.entities[e].bot;
        b.goal.next_urgent = now + 0.2;
        if let Some((item, cell, commit)) = pick {
            if item.0 != b.goal.item {
                b.goal.since = now;
            }
            (b.goal.item, b.goal.item_cell, b.goal.commit) = (item.0, cell, commit);
            (b.goal.next_item, b.goal.next_cell, b.goal.next_commit) = (0, 0, GoalCommit::None);
            b.goal.next_pick = now + GOAL_SELECT_INTERVAL;
        }
    }

    // Handoff hold: an idle bot may reserve a spawned RL/LG for a powerup-carrying teammate (standing
    // on it without taking it). While holding, the reservation owns `goal_item`, pre-empting normal
    // item selection; a fight or move objective (idle == false) drops the hold inside this call.
    let holding = game.update_handoff_hold(
        e,
        now,
        intent.is_none() && game.entities[e].bot.goal.commit == GoalCommit::None,
    );
    if holding {
        // `update_handoff_hold` set `goal_item` to the held weapon — nothing else to pick this frame.
    } else if game.entities[e].bot.goal.commit != GoalCommit::None {
        // Nearby recovery / timed powerup completion owns the slot until its terminal condition.
    } else if intent.is_none() || matches!(intent, Some(BotIntent::Fight(_) | BotIntent::Advance(_))) {
        if now >= game.entities[e].bot.goal.next_pick {
            let pick = match intent {
                Some(BotIntent::Fight(_)) if greedy => game.select_combat_item(e),
                Some(BotIntent::Fight(_)) => game.select_major_item(e),
                _ => game.select_item_goal(e),
            };
            let (new_item, new_cell, next_item, next_cell, commit, next_commit) = match pick {
                Some(plan) => {
                    let (first, first_cell) = plan.first;
                    let (next, next_cell) = plan.second.map_or((0, 0), |(it, cell)| (it.0, cell));
                    let first_powerup = game.is_powerup_item(first);
                    let next_powerup = next != 0 && game.is_powerup_item(EntId(next));
                    let commit = if first_powerup || plan.contains_powerup {
                        GoalCommit::Powerup
                    } else {
                        GoalCommit::None
                    };
                    let next_commit = if next_powerup {
                        GoalCommit::Powerup
                    } else {
                        GoalCommit::None
                    };
                    (first.0, first_cell, next, next_cell, commit, next_commit)
                }
                None => (0, 0, 0, 0, GoalCommit::None, GoalCommit::None),
            };
            let b = &mut game.entities[e].bot;
            if new_item != b.goal.item {
                b.goal.since = now; // restart the watchdog for a new goal
            }
            (b.goal.item, b.goal.item_cell) = (new_item, new_cell);
            (b.goal.next_item, b.goal.next_cell, b.goal.next_commit) = (next_item, next_cell, next_commit);
            b.goal.next_pick = now + GOAL_SELECT_INTERVAL;
            b.goal.commit = commit;
        }
        if game.entities[e].bot.goal.item != 0 && !game.item_goal_valid(e, EntId(game.entities[e].bot.goal.item), now) {
            let b = &mut game.entities[e].bot;
            b.goal.item = 0;
            b.goal.next_item = 0;
            b.goal.commit = GoalCommit::None;
            b.goal.next_commit = GoalCommit::None;
            b.goal.next_pick = now; // re-pick next frame
        }
    } else {
        let b = &mut game.entities[e].bot;
        b.goal.item = 0; // a Move objective supersedes any item chase
        b.goal.next_item = 0;
        b.goal.commit = GoalCommit::None;
        b.goal.next_commit = GoalCommit::None;
    }

    // Item vigil: if the goal item isn't collectable yet (mid-respawn, or a weapon held for a
    // teammate) and we're already standing near it, cruise a short walk off and scan the room instead
    // of twitching on the spot. Returns the overridden navigation target; `None` = carry on normally.
    let vigil = if game.entities[e].bot.goal.item != 0 {
        vigil::maybe(game, e, origin, holding, now)
    } else {
        None
    };

    // Opt-in diagnostics (`rtx_bot_debug 1`): one throttled line per bot — what it wants, how far,
    // whether it's standing on that item, and whether it owns the LG. Pinpoints pickup-vs-desire.
    if host.cvar_bool(c"rtx_bot_debug") && now >= game.entities[e].bot.repath_time {
        let gi = game.entities[e].bot.goal.item;
        let (goal, dist, overlap) = if gi != 0 {
            let it = &game.entities[EntId(gi)];
            let on = it.v.solid == Solid::Trigger && on_item(origin, it.v.origin);
            let name = it
                .classname()
                .unwrap_or(if it.touch == Touch::Backpack { "backpack" } else { "?" });
            (name, (it.v.origin - origin).length(), on)
        } else {
            ("human", 0.0, false)
        };
        let own_lg = game.entities[e].v.items.has(Items::LIGHTNING) as i32;
        let b = &game.entities[e].bot;
        // Loop-free-nav telemetry: live failed-link penalties and whether the target is currently
        // perceived (aware of / in memory), so a stuck/looping bot's divert can be watched live.
        let pen = b.failed_links.iter().filter(|&&(_, until, _)| until > now).count();
        let aware = (b.percept.known_enemy != 0 && now < b.percept.known_until) as i32;
        let hold = b.goal.hold_item;
        // Opponent-model telemetry: this bot's current hypothesis of the enemy it's aware of — the
        // estimated health/armor stack and believed arsenal bits — so the shared read can be watched
        // converge and reset live. Blank (`est=-`) when there's no belief / modeling is off.
        let est = if b.percept.known_enemy != 0 && now < b.percept.known_until {
            game.opponent_est(e, EntId(b.percept.known_enemy), now)
        } else {
            None
        };
        let est = est.map_or_else(
            || "-".to_string(),
            |o| format!("H{:.0}/A{:.0} ars={:03x}", o.health, o.armor_value, o.items as u32),
        );
        let vig = vigil.is_some() as i32;
        // Arena spectating: which fighter (edict) this bot is watching, `0` = none.
        let wat = if let Some(BotIntent::Spectate { watch, .. }) = intent { watch.0 } else { 0 };
        let commit = b.goal.commit;
        let posture = b.posture;
        let msg = cstring(&format!(
            "rtx bot{client}: want={goal} dist={dist:.0} on_item={overlap} ownLG={own_lg} cells={:.0} pen={pen} aware={aware} est={est} hold={hold} commit={commit:?} posture={posture:?} vig={vig} watch={wat}\n",
            game.entities[e].v.ammo_cells,
        ));
        host.conprint(&msg); // conprint always shows; dprint needs `developer 1`
    }

    // The mode's intent (fight an enemy / roam to a spot), if any, and whether we're on an item.
    let enemy = if let Some(BotIntent::Fight(en)) = intent {
        Some(en)
    } else {
        None
    };
    // A committed item goal drives *movement* — set by the idle brain, major-objective planning,
    // strategic recovery, or a greedy ordinary combat detour. Under Fight the enemy stays tracked, so
    // the combat overlay keeps aiming/firing on line of sight (and its range-keeping owns movement
    // then); the detour only steers navigation while the enemy is *out* of sight, when navigation
    // would otherwise just beeline the enemy. This is ktx's "the enemy is one more goal" in effect.
    let chasing = game.entities[e].bot.goal.item != 0;
    let item_committed = game.entities[e].bot.goal.commit != GoalCommit::None;
    let goal_item_org = {
        let it = EntId(game.entities[e].bot.goal.item);
        (game.entities[it].v.origin, Some(game.entities[e].bot.goal.item_cell))
    };
    // Where we're headed: the vigil post (waiting on an uncollectable item), the detour item, the
    // mode's target, the chosen item, or the nearest human.
    let (target_origin, item_cell) = match intent {
        Some(BotIntent::Fight(_)) if chasing => vigil.unwrap_or(goal_item_org),
        // Visible → the enemy's live origin (combat owns aim on sight); aware-but-unseen → the
        // last-seen spot, so the bot searches where they went instead of tracking through walls.
        Some(BotIntent::Fight(en)) => (combat_last_seen.unwrap_or(game.entities[en].v.origin), None),
        Some(BotIntent::Advance(_)) if chasing => vigil.unwrap_or(goal_item_org),
        Some(BotIntent::Advance(pos)) => (pos, None),
        Some(BotIntent::Move(pos)) => (pos, None),
        // Spectate navigates exactly like Move; the watched fighter only redirects the eyes (below).
        Some(BotIntent::Spectate { goal, .. }) => (goal, None),
        None if chasing => vigil.unwrap_or(goal_item_org),
        None => {
            polite = true; // following / roaming: no need to stand on the exact spot
            if let Some(h) = nearest_human(game, e) {
                (game.entities[h].v.origin, None)
            } else {
                // Nothing to chase and no human to follow: **roam** to a random reachable spot
                // instead of freezing, so a human-less server (e.g. bots-only FFA) keeps its bots
                // moving — and finding items and each other — rather than standing on spawn.
                (roam_target(game, e, origin, now), None)
            }
        }
    };

    // Goal watchdog: while chasing an item and *not* already detouring to open a gate, give up on
    // one we've chased too long without collecting (behind an elevator/button/movewall/teleporter
    // chain the router can't thread) so we stop circling and go fetch something reachable instead.
    // A powerup goal gets a longer leash — a cross-map quad/pent run legitimately outlasts the
    // ordinary give-up time (the progress watchdog still catches a genuinely stuck bot sooner).
    let giveup = if game.is_powerup_item(EntId(game.entities[e].bot.goal.item)) {
        goals::POWERUP_GIVEUP
    } else {
        GOAL_GIVEUP_TIME
    };
    if chasing && game.entities[e].bot.gate.errand.is_none() && now - game.entities[e].bot.goal.since > giveup {
        let b = &mut game.entities[e].bot;
        b.mark_avoid(b.goal.item, now + GOAL_AVOID_TIME);
        b.goal.item = 0;
        b.goal.next_item = 0;
        b.goal.commit = GoalCommit::None;
        b.goal.next_commit = GoalCommit::None;
        b.goal.next_pick = now; // re-pick (skipping the abandoned item) next frame
    }

    // Audience watch (arena Spectate): snapshot the chosen fighter's eye point so the look override
    // in `run_bot` — deep inside the `&mut bot` / `&nav` borrow — can point the eyes there without
    // re-reading another entity. `None` for every other intent leaves the eyes on the corridor.
    let watch_point = match intent {
        Some(BotIntent::Spectate { watch, .. }) => Some(game.entities[watch].v.origin + VEC_VIEW_OFS),
        _ => None,
    };

    // `surfacing`/`swim_up` are decided in `run_bot` (they need the borrowed graph + bot cell), so
    // the normal objective leaves them off — the anti-drown override flips them on when it fires.
    Objective {
        hooking, on_sj, on_rj, enemy, chasing, item_committed, polite, target_origin, item_cell, watch_point,
        vigil: vigil.is_some(), surfacing: false, swim_up: false, order_link: None,
    }
}

/// Turn the frame's accumulated decisions into the engine usercmd. Bunnyhopping bypasses the aim
/// spring (an air-strafe sweeps the view yaw independently of travel, which the world-move
/// reprojection can't express); otherwise the critically-damped spring smooths the view and the
/// world move is projected onto it. The hook fire is decided here, *after* the spring, so the
/// throw waits for the smoothed view to settle on the anchor.
#[allow(clippy::too_many_arguments)] // the frame's decision bundle; grouping it would just relocate
fn emit(
    game: &mut GameState,
    e: EntId,
    s: Sense,
    cmd: BotCmd,
    bhop_cmd: Option<bhop::Cmd>,
    hook: &hook::HookDrive,
    rj: &rj::RjDrive,
    enemy: Option<EntId>,
) {
    let Sense { host, now, frametime, v_angle, client, msec, speed, .. } = s;
    let BotCmd { look, move_world, mut buttons, impulse } = cmd;

    // View + move for the frame. When the bhop controller is driving, translate its usercmd into a
    // world-space wish and a swept view target (yaw = the velocity heading it chose) so it can go
    // through the *same* aim spring as everything else. The air-strafe wish now lives in
    // `forward`/`side` decoupled from the view (see `bhop::strafe_rate`), so smoothing the eyes no
    // longer corrupts the movement — the reprojection below reproduces the world wish onto whatever
    // view the spring settles on. This keeps the view a smooth spring-lagged sweep instead of a raw
    // per-frame yaw that kinks at each strafe flip and bobs in pitch every hop.
    let (look, move_world) = match bhop_cmd {
        Some(c) => {
            let w = bhop::wishdir_fs(c.view_yaw, c.forward, c.side);
            (Vec3::new(look.x, c.view_yaw, 0.0), Vec3::new(w.x, w.y, 0.0) * crate::defs::BOT_MOVE_SPEED)
        }
        None => (look, move_world),
    };
    let (view, forward, side) = {
        let dt = frametime.clamp(0.001, 0.05);
        let skill = host.cvar(c"rtx_bot_skill").clamp(0.0, 7.0);
        // Spring stiffness (1/s): sluggish → pro-snappy. Shared with the combat feed-forward,
        // whose lag compensation assumes exactly this spring.
        let omega = combat::aim_omega(skill);
        let b = &mut game.entities[e].bot;
        if b.aim.angles == Vec3::ZERO {
            b.aim.angles = v_angle; // seed from the real view so the first frame doesn't snap from zero
        }
        let spring = |a: f32, v: f32, target: f32| {
            let d = wrap180(target - a);
            let v = v + (omega * omega * d - 2.0 * omega * v) * dt;
            (wrap180(a + v * dt), v)
        };
        let (pitch, pv) = spring(b.aim.angles.x, b.aim.vel.x, look.x);
        let (yaw, yv) = spring(b.aim.angles.y, b.aim.vel.y, look.y);
        b.aim.angles = Vec3::new(pitch, yaw, 0.0);
        b.aim.vel = Vec3::new(pv, yv, 0.0);
        let view = b.aim.angles;
        let (vf, vr, _) = angle_vectors(view);
        (
            view,
            vf.dot(move_world).round() as i32,
            vr.dot(move_world).round() as i32,
        )
    };

    // Hook fire, decided *after* the spring: the hook flies along `v_angle` (the smoothed view we're
    // about to send), so the throw must wait until that view has actually settled onto the anchor —
    // gating on the raw target would launch the hook while the aim is still swinging. Once thrown we
    // hold +attack every frame through the reel (the engine zeroes the cmd each tick, and a single
    // released frame would drop the hook at impact or unhook mid-reel).
    if hook.fire_ready {
        let err = wrap180(view.x - look.x).abs().max(wrap180(view.y - look.y).abs());
        if err < HOOK_AIM_TOL {
            buttons |= BUTTON_ATTACK;
            let b = &mut game.entities[e].bot;
            b.hook.phase = HookPhase::Flight;
            b.hook.started = now;
        }
    } else if hook.hold_fire {
        buttons |= BUTTON_ATTACK;
    }
    // Rocket-jump launch, decided *after* the spring like the hook throw: the shot leaves along the
    // smoothed view, so wait until that view has settled onto the solved fire angles before pressing
    // jump — pressing early would jump with the aim still swinging and fire the rocket off-angle.
    if rj.jump_ready {
        let err = wrap180(view.x - look.x).abs().max(wrap180(view.y - look.y).abs());
        if err < s.rj_knobs.aim_tol {
            buttons |= BUTTON_JUMP;
            let b = &mut game.entities[e].bot;
            b.rj.phase = RjPhase::Rise;
            b.rj.jump_time = now;
            // Harness telemetry: the actual press moment, the settled view, and the residual aim
            // error against the (biased) fire angles. Inert without a puppet order consuming it.
            b.rj.telem.press = Some(state::RjPress { t: now, origin: s.origin, view, aim_err: err });
        }
    }
    // The rocket fires this frame (the driver set `rj.fire` in Stance-timed Rise): stamp the settled
    // view actually sent with +attack into the fire telemetry the driver pre-filled.
    if rj.fire {
        if let Some(f) = game.entities[e].bot.rj.telem.fire.as_mut() {
            f.view = view;
        }
    }
    // Flush any deferred hook release now the graph/bot borrows are done.
    if let Some(h) = hook.reset {
        game.reset_grapple(h);
    }

    // Combat/gate diagnostics: what the bot is chasing and whether it's stuck at a gate. Enable
    // with `rtx_bot_debug 1` (conprint shows without `developer`).
    if host.cvar_bool(c"rtx_bot_debug") {
        let gate = game.entities[e].bot.gate.errand.map(|er| er.index);
        let route = game.entities[e].bot.route.len();
        let hph = game.entities[e].bot.hook.phase;
        let rjph = game.entities[e].bot.rj.phase;
        let bpos = game.entities[e].bot.route_pos;
        let band = game.entities[e].bot.route_bands.get(bpos).copied().unwrap_or(0);
        let bh = &game.entities[e].bot.bhop;
        host.conprint(&cstring(&format!(
            "rtx bot{client}: enemy={} gate={gate:?} hook={hph:?} rj={rjph:?} bhop={:?} hops={} flips={} peak={:.0} \
             spd={speed:.0} route={route} band={band} fwd={forward} side={side} atk={}\n",
            enemy.is_some(),
            bh.phase,
            bh.hops,
            bh.flips,
            bh.peak,
            (buttons & BUTTON_ATTACK) != 0,
        )));
    }

    host.set_bot_cmd(client, msec, view, forward, side, 0, buttons, impulse);
}

/// Pre-arm an indivisible jump traversal from the route that existed at frame start. This runs
/// before objective resolution; the steering core repeats the latch defensively after any fresh
/// repath. Existing commitments are never cleared here — only their physical lifecycle may do so.
fn prearm_traversal(game: &mut GameState, e: EntId, now: f32, on_ground: bool) {
    let current = {
        let Some(graph) = game.nav.graph.as_ref() else {
            return;
        };
        let b = &game.entities[e].bot;
        b.route.get(b.route_pos).copied().map(|leg| {
            (
                leg,
                graph.link_kind(leg),
                graph.link_target(leg),
            )
        })
    };
    let Some((leg, kind, target)) = current else {
        return;
    };
    let bhop = game.host().cvar_bool(c"rtx_bot_bhop");
    let b = &mut game.entities[e].bot;
    match kind {
        LinkKind::JumpGap | LinkKind::DoubleJump => {
            if b.air.map(|c| c.leg) != Some(leg) {
                b.air = Some(AirCommit {
                    leg,
                    target,
                    since: now,
                    airborne: !on_ground,
                });
            } else if !on_ground {
                b.air.as_mut().unwrap().airborne = true;
            }
        }
        LinkKind::SpeedJump if bhop && b.sj.map(|c| c.leg) != Some(leg) => {
            b.sj = Some(Commit { leg, since: now });
        }
        _ => {}
    }
}

fn run_bot(game: &mut GameState, e: EntId) {
    let s = sense(game, e);
    // The spine reads only these; `steer` re-destructures the full snapshot from `s` via `SteerCtx`.
    let Sense { host, now, msec, origin, v_angle, client, alive, .. } = s;
    // Flip the per-frame pulse used for press/release-edge buttons.
    let pulse = {
        let b = &mut game.entities[e].bot;
        b.pulse = !b.pulse;
        b.pulse
    };

    // A dead bot holds nothing and isn't mid-jump — drop the handoff reservation and any airborne
    // commitment so neither resumes after respawn.
    if !alive {
        let b = &mut game.entities[e].bot;
        if b.goal.hold_item != 0 {
            (b.goal.hold_item, b.goal.hold_for, b.goal.hold_until) = (0, 0, 0.0);
        }
        b.air = None;
        b.sj = None;
        b.goal.commit = GoalCommit::None;
        b.posture = CombatPosture::Hold;
    }

    // Connected but never spawned (health 0, not dead): the engine defers `PutClientInServer` — the
    // full spawn that sets health/loadout — to the bot's spawn on a *bot frame*, which an empty
    // (bots-only) server never runs. So the bot sits at 0 health forever, and the respawn pulse below
    // can't help it (`death_think` only runs for `deadflag >= Dead`). Seed fresh spawn parms before
    // spawning; FFA/team keep those decoded parms, while fixed-kit modes overwrite them.
    if !alive && game.entities[e].v.deadflag == DeadFlag::No {
        // Don't place onto an occupied spot the mode can't telefrag clear (Rocket Arena): postpone
        // and retry next bot frame — an early return without a command is this branch's own shape.
        let mode = game.mode;
        if !mode.spawn_area_clear(game, e) {
            return;
        }
        game.set_new_parms();
        game.put_client_in_server(e);
        return;
    }
    // Genuinely dead (fragged): pulse +attack to respawn. rtx's death-think needs all buttons
    // *released* (Dead → Respawnable) and then *pressed* again — so the button must be pulsed.
    if !alive {
        let buttons = if pulse { BUTTON_ATTACK } else { 0 };
        host.set_bot_cmd(client, msec, v_angle, 0, 0, 0, buttons, 0);
        return;
    }

    let idle = |angles: Vec3| host.set_bot_cmd(client, msec, angles, 0, 0, 0, 0, 0);

    // Arm traversal ownership *before* resolving this frame's mode/enemy/item objective. The route
    // was selected on an earlier frame; once its current leg is a gap/double/speed jump, a newly
    // perceived enemy must not get one frame in which to replace that route or turn the view at the
    // lip. `steer` owns the physical lifecycle and fallback latch.
    prearm_traversal(game, e, now, s.on_ground);

    let mut o = resolve_objective(game, e, now, origin, client);
    // The spine and prologue read a few fields; `steer` re-destructures the rest from `o` via `SteerCtx`.
    let Objective { enemy, item_cell, target_origin, .. } = o;

    // Whether weapons may fire right now (a match-mode countdown locks them out). Read before the nav
    // borrow: a rocket jump must not jump when the engine would swallow its rocket (jump, no blast).
    let weapons_hot = {
        let mode = game.mode;
        mode.weapons_hot(game)
    };

    // Current door states, for gate-aware pathfinding. A shut gate makes its links expensive, so
    // `find_path` bends the route around a closed door when any open way exists and only crosses
    // one (leaving the bot to detour to the button) when there's no alternative. Computed before
    // the nav borrow (it reads the obstruction edicts).
    // This bot's per-frame A* pricing: closed gates (so `find_path` bends the route around a shut
    // door when it can), this bot's failed-link surcharges (so the planner diverts off legs it keeps
    // failing rather than handing back the same dead route), and its rocket-jump fitness gate. Built
    // from an immutable read so the owned Vecs outlive the disjoint `&mut bot` borrow below.
    let pricing = game.bot_link_pricing(e, now);

    // Live lift states, for the plat standoff. A bot approaching a raised `func_plat` must hold
    // outside its inner trigger — standing under it resets the lift's lower-timer and it never comes
    // down (see `plat_statuses`). Read before the nav borrow (it reads the plat edicts).
    let plat_status = game.plat_statuses();

    // A per-bot jitter seed varies otherwise-equal routes so two bots don't tread an identical line
    // (cheap route variety that also reads as more human).
    let costs = pricing.costs(e.0);

    // Graph queries (borrows game.nav) and bot-state updates (borrows game.entities) are on
    // disjoint fields, so they coexist; `host` is a Copy, no game borrow held across the send.
    let graph = game.nav.graph.as_ref().unwrap();
    let Some(bot_cell) = graph.nearest(origin) else {
        idle(v_angle);
        return;
    };
    let Some(mut goal_cell) = item_cell.or_else(|| graph.nearest(target_origin)) else {
        idle(v_angle);
        return;
    };

    // Anti-drown override: fully submerged and low on air, so drop the current goal and make for
    // air. `surfacing`/`swim_up` tell the combat spine to hand movement back to navigation and hold
    // jump (open water above) — see the `engage` overlay below. When a breathing cell is reachable
    // the goal is redirected onto it; otherwise the bot at least swims straight up. Fires only in the
    // rare panic window and self-limits (one breath refills the tank), so it wins outright.
    if s.submerged && s.air_left < DROWN_PANIC_SECS {
        o.surfacing = true;
        o.swim_up = crate::hazard::surface_above(&|p| host.pointcontents(p), origin);
        let air = surface_target(&mut game.entities[e].bot.surface, graph, bot_cell, &costs, now);
        if let Some(cell) = air.and_then(|a| graph.nearest(a)) {
            goal_cell = cell;
            o.target_origin = graph.cell_origin(cell);
            o.item_cell = Some(cell);
            o.polite = false;
        }
    }

    // Burn-escape reflex: standing in lava/slime burns from the first frame, but a bot deliberately
    // crossing a moat/bridge is off it within a tick — so only once it's been stuck burning past
    // `BURN_PANIC_SECS` do we hijack the goal onto the nearest safe cell (the same flood as anti-drown).
    // `burn_since` stamps when the burn began and clears when it ends. Unlike drowning this sets no
    // `surfacing`/`swim_up`: while a fight owns movement the combat guard keeps the bot off the coals,
    // and the redirected goal takes over the instant combat releases it.
    if !s.burning {
        game.entities[e].bot.burn_since = 0.0;
    } else if game.entities[e].bot.burn_since == 0.0 {
        game.entities[e].bot.burn_since = now;
    }
    if s.burning && now - game.entities[e].bot.burn_since >= BURN_PANIC_SECS {
        let safe = escape_target(&mut game.entities[e].bot.burn, graph, bot_cell, &costs, now);
        if let Some(cell) = safe.and_then(|a| graph.nearest(a)) {
            goal_cell = cell;
            o.target_origin = graph.cell_origin(cell);
            o.item_cell = Some(cell);
            o.polite = false;
        }
    }

    // Race mode: the offline-optimized racing line's look-ahead point to bias the bhop bearing
    // toward (`None` outside race, with the feature off, or when the bot has strayed off the line —
    // then it recovers on the plain navmesh route). Computed here while only `graph` is borrowed.
    let race_line_ahead = game.race_line_lookahead(origin);

    // Whether each gate's activator can be triggered right now: a shoot activator is "ready" only
    // while it takes damage — re-triggerable triggers go dead during their cooldown.
    let gate_ready: Vec<bool> = (0..graph.gate_count())
        .map(|gi| {
            let g = graph.gate(gi);
            !g.shoot || game.entities[EntId(g.activator)].v.takedamage != TakeDamage::No
        })
        .collect();

    let bot = &mut game.entities[e].bot;
    let steer::SteerOut { mut cmd, bhop_cmd, hook, rj, traversal_lock, overlays_ok } = steer::steer(
        graph,
        bot,
        steer::SteerCtx {
            s,
            o,
            costs,
            plat_status: &plat_status,
            gate_ready: &gate_ready,
            bot_cell,
            goal_cell,
            race_line_ahead,
            weapons_hot,
            bsp: game.nav.bsp.as_ref(),
        },
    );

    // The `&nav` / `&mut bot` steering borrows have ended; the spine resumes with `&mut game`.
    // Navigation's world-move, kept so combat can't strand the bot in water: a swimmer isn't
    // ONGROUND, so `combat_move`'s hazard filter never runs and it would strafe in place until it
    // drowns. In water we restore this route move (which the water surcharge aims at shore).
    let nav_move = cmd.move_world;
    let nav_jump = cmd.buttons & BUTTON_JUMP;

    // Combat overlay: with an enemy in sight, `engage` picks the look (live aim with drifting error)
    // and its own movement; traversal-critical legs are locked out (see `SteerOut::traversal_lock`).
    if let Some(en) = enemy.filter(|_| !traversal_lock) {
        combat::engage(game, e, en, origin, now, &mut cmd);
    }

    // Completion-critical pickups own travel even with line of sight. Combat still supplies aim,
    // weapon choice, and +attack, but it may not strafe away from armor/health/powerup or clear the
    // route driver's jump input on the final approach.
    if o.item_committed {
        cmd.move_world = nav_move;
        cmd.buttons = (cmd.buttons & !BUTTON_JUMP) | nav_jump;
    }

    // In water, navigation owns *travel* — combat keeps aiming and firing, but the bot heads for
    // shore (or dives on to a genuinely wet goal) instead of treading water in a fight. When actively
    // surfacing with open water overhead, also hold jump to swim up. `engage` clears BUTTON_JUMP, so
    // this runs after it — and before the grenade overlays, so a grenade flee can still take over.
    if s.in_water {
        cmd.move_world = nav_move;
        if o.surfacing && o.swim_up {
            cmd.buttons |= BUTTON_JUMP;
        }
    }
    // Splash/projectile overlays, after `engage`: shoot/flee a live grenade when possible, dodge any
    // incoming rocket/grenade/nail, finish a lob->shoot combo, take a one-shot rocket hazard shove,
    // else start a grenade combo. Skipped while hooking/rj/bhop-ing or locked into a jump traversal
    // (movement/buttons already spoken for) — that is `overlays_ok`.
    if overlays_ok && !o.item_committed {
        let handled = combat::projectile_dodge(game, e, origin, now, &mut cmd)
            || combat::grenade_tactics(game, e, enemy, origin, &mut cmd);
        if handled {
            game.entities[e].bot.grenade.phase = GrenadePhase::Idle; // defence drops a stale combo
        } else {
            // A one-shot rocket shove takes priority over *starting* a grenade combo, but never
            // interrupts one already in progress (the short-circuit keeps a running combo going).
            let running = game.entities[e].bot.grenade.phase != GrenadePhase::Idle;
            if running || !grenade::rocket_shove(game, e, enemy, origin, &mut cmd) {
                grenade::grenade_combo(game, e, enemy, origin, now, &mut cmd);
            }
        }
    }
    emit(game, e, s, cmd, bhop_cmd, &hook, &rj, enemy);
}

/// The first shut gate whose blocked cell lies on the remaining route, if any.
fn route_blocking_gate(graph: &NavGraph, route: &[u32], from: usize, closed: &[bool]) -> Option<usize> {
    route.get(from..)?.iter().find_map(|&leg| {
        let gi = graph.gate_of_link(leg)?;
        (*closed.get(gi)?).then_some(gi)
    })
}

/// Travel-time surcharge for a link with `strikes` recorded failures (`strikes²·PENALTY_STEP`,
/// capped). Grows super-linearly so a link that keeps failing is diverted harder each time.
fn link_penalty_secs(strikes: u8) -> f32 {
    ((strikes as f32).powi(2) * PENALTY_STEP).min(PENALTY_CAP)
}

/// Radius around a teleport destination that counts as occupied by a teammate, and the modest
/// route surcharge used to stagger arrivals. It diverts only when an alternative is competitive;
/// the teleport remains usable when it is the sole route.
const TELEPORT_EXIT_CLEAR: f32 = 96.0;
const TELEPORT_TEAM_SURCHARGE: f32 = 2.0;

/// Add or merge a transient per-link surcharge. Failed-link and team-occupancy penalties can target
/// the same link; merging matters because the nav query intentionally stores one extra per link.
fn merge_link_penalty(penalties: &mut Vec<(u32, f32)>, link: u32, extra: f32) {
    if let Some((_, old)) = penalties.iter_mut().find(|(li, _)| *li == link) {
        *old += extra;
    } else {
        penalties.push((link, extra));
    }
}

/// A bot's per-frame A* pricing inputs, gathered from an immutable `&GameState` so the owned Vecs
/// outlive the disjoint `&mut bot` borrow: the live closed-gate flags, this bot's unexpired
/// failed-link surcharges, and its rocket-jump fitness gate. Build once with
/// [`GameState::bot_link_pricing`], then take a [`LinkCosts`] view per query with [`costs`](Self::costs)
/// — the only difference between callers is the jitter seed (per-bot for routing variety, `0` for
/// stable item scoring). Replaces the identical assembly `run_bot` and `best_item_goal` each spelled out.
pub(crate) struct LinkPricing {
    gate_closed: Vec<bool>,
    penalties: Vec<(u32, f32)>,
    rj_extra: f32,
}

impl LinkPricing {
    /// A `LinkCosts` view over this pricing, with the caller's `jitter_seed`.
    pub(crate) fn costs(&self, jitter_seed: u32) -> LinkCosts<'_> {
        LinkCosts {
            gate_closed: &self.gate_closed,
            penalties: &self.penalties,
            jitter_seed,
            rocket_jump_extra: self.rj_extra,
        }
    }
}

impl GameState {
    /// Gather bot `e`'s live A* pricing (see [`LinkPricing`]) — closed gates, its failed-link
    /// surcharges (expired entries dropped), and its rocket-jump fitness gate.
    pub(crate) fn bot_link_pricing(&self, e: EntId, now: f32) -> LinkPricing {
        let mut penalties: Vec<(u32, f32)> = self.entities[e]
            .bot
            .failed_links
            .iter()
            .filter(|&&(_, until, _)| until > now)
            .map(|&(li, _, strikes)| (li, link_penalty_secs(strikes)))
            .collect();
        let my_team = self.entities[e].mode_p.team;
        if my_team != 0 {
            if let Some(graph) = &self.nav.graph {
                let maxclients = self.host().cvar(c"maxclients") as u32;
                for li in 0..graph.links.len() as u32 {
                    if graph.link_kind(li) != LinkKind::Teleport {
                        continue;
                    }
                    let exit = graph.cell_origin(graph.link_target(li));
                    let occupied = (1..=maxclients).map(EntId).any(|mate| {
                        let m = &self.entities[mate];
                        mate != e
                            && m.is_player()
                            && m.is_alive()
                            && m.mode_p.team == my_team
                            && (m.v.origin - exit).length_squared() < TELEPORT_EXIT_CLEAR * TELEPORT_EXIT_CLEAR
                    });
                    if occupied {
                        merge_link_penalty(&mut penalties, li, TELEPORT_TEAM_SURCHARGE);
                    }
                }
            }
        }
        let rj_extra = rj::rocket_jump_extra(&self.entities[e].v, self.entities[e].combat.super_damage_finished, now);
        LinkPricing {
            gate_closed: self.gate_closed_flags(),
            penalties,
            rj_extra,
        }
    }
}

/// Whether the goal-ward distance has stalled: no improvement of at least `PROGRESS_EPS` below the
/// best-seen (`best`) for at least `PROGRESS_STALL_TIME` since it last improved (`since`). Pure, so
/// the threshold logic is unit-testable apart from the frame plumbing.
fn progress_stalled(best: f32, since: f32, remaining: f32, now: f32) -> bool {
    now - since > PROGRESS_STALL_TIME && remaining > best - PROGRESS_EPS
}

/// Whether `p` lies within the box `[fp_min, fp_max]` grown by `margin` on every side.
pub(crate) fn in_footprint(p: Vec2, fp_min: Vec2, fp_max: Vec2, margin: f32) -> bool {
    p.x >= fp_min.x - margin
        && p.x <= fp_max.x + margin
        && p.y >= fp_min.y - margin
        && p.y <= fp_max.y + margin
}

/// Where to stand while a raised plat comes down: the bot's current spot if it's already at least
/// [`PLAT_STANDOFF`] clear of the footprint (so it holds still — no jitter), else the nearest point
/// pushed just past that boundary along whichever face it's closest to escaping, so it steps
/// straight back out of the trigger the way it came rather than crossing under the lift. Z is kept
/// at the bot's own height.
fn plat_standoff(origin: Vec3, fp_min: Vec2, fp_max: Vec2) -> Vec3 {
    if !in_footprint(origin.xy(), fp_min, fp_max, PLAT_STANDOFF) {
        return origin;
    }
    // How far to move outward to clear the standoff boundary across each of the four faces.
    let out_min_x = origin.x - (fp_min.x - PLAT_STANDOFF);
    let out_max_x = (fp_max.x + PLAT_STANDOFF) - origin.x;
    let out_min_y = origin.y - (fp_min.y - PLAT_STANDOFF);
    let out_max_y = (fp_max.y + PLAT_STANDOFF) - origin.y;
    let m = out_min_x.min(out_max_x).min(out_min_y).min(out_max_y);
    let (mut x, mut y) = (origin.x, origin.y);
    if m == out_min_x {
        x = fp_min.x - PLAT_STANDOFF;
    } else if m == out_max_x {
        x = fp_max.x + PLAT_STANDOFF;
    } else if m == out_min_y {
        y = fp_min.y - PLAT_STANDOFF;
    } else {
        y = fp_max.y + PLAT_STANDOFF;
    }
    Vec3::new(x, y, origin.z)
}

/// Record that this bot just failed to traverse `link`, so its next A* diverts around it. `Plat`
/// and `Teleport` legs are exempt: waiting on a lift or standing in a teleport trigger reads as
/// "stuck" to the watchdogs but is not a routing mistake. Bumps an existing live entry's strike
/// count (harder divert on repeat) or claims the expired/oldest slot of the fixed ring.
/// The fate of an airborne jump commitment this frame (see [`AIR_COMMIT_GRACE`]/[`AIR_COMMIT_MAX`]).
#[derive(Debug, PartialEq)]
enum AirRelease {
    /// Stay committed — freeze the route and lock out combat.
    Keep,
    /// Physically landed after an observed airborne phase; normal navigation resumes.
    Land,
    /// Still airborne well past any real arc: abandon the leg (penalize + re-path).
    Timeout,
}

/// Pure core of the airborne-commitment lifecycle. Route advancement is deliberately absent: only
/// an observed airborne phase followed by settled ground contact proves a landing.
fn air_commit_decision(on_ground: bool, was_airborne: bool, elapsed: f32) -> AirRelease {
    if was_airborne && on_ground && elapsed > AIR_COMMIT_GRACE {
        AirRelease::Land
    } else if elapsed > AIR_COMMIT_MAX {
        AirRelease::Timeout
    } else {
        AirRelease::Keep
    }
}

fn penalize_leg(bot: &mut BotState, link: Option<u32>, kind: Option<LinkKind>, now: f32) {
    let Some(link) = link else { return };
    if matches!(kind, Some(LinkKind::Plat | LinkKind::Teleport)) {
        return;
    }
    penalize_link(bot, link, now);
}

/// Record a failed `link` unconditionally — the ring-insert core of [`penalize_leg`], without its
/// `None`/`Plat`/`Teleport` guards. The plat-wait timeout calls this directly: it *must* strike a
/// `Plat` link, the very kind `penalize_leg` exempts (the ordinary watchdogs misread waiting on a
/// lift as failure, but an 8s hold with no descent is a genuine one worth diverting from).
fn penalize_link(bot: &mut BotState, link: u32, now: f32) {
    if let Some(slot) = bot.failed_links.iter_mut().find(|(l, until, _)| *l == link && *until > now) {
        slot.2 = slot.2.saturating_add(1);
        slot.1 = now + PENALTY_TTL;
        return;
    }
    // No live entry: reuse the slot expiring soonest (an expired/unused one has the smallest `until`).
    if let Some(slot) = bot.failed_links.iter_mut().min_by(|a, b| a.1.total_cmp(&b.1)) {
        *slot = (link, now + PENALTY_TTL, 1);
    }
}

/// Whether gate `gi`'s button can be reached from `from` *without* crossing gate `gi`'s own shut
/// door. False for the chicken-and-egg case (e.g. arenazap's central plate, which opens all four
/// pillars but sits behind them): a bot outside can't reach it, so committing to that gate is
/// futile — it should route around the pillar instead. A `None` path counts as unreachable.
fn button_reachable(graph: &NavGraph, from: CellId, gi: usize, costs: &LinkCosts) -> bool {
    match graph.find_path(from, graph.gate(gi).button_cell, costs) {
        Some(route) => !route.iter().any(|&leg| graph.gate_of_link(leg) == Some(gi)),
        None => false,
    }
}

/// The target points of the leading `Walk`/`Step` legs from `route_pos` (the ground corridor a
/// bunnyhop can run), stopping at the first non-ground leg. Shared by [`runway`] and
/// [`corridor_point`] so both trace the exact same corridor.
fn ground_leg_targets<'a>(
    graph: &'a NavGraph,
    route: &'a [u32],
    route_pos: usize,
) -> impl Iterator<Item = Vec3> + 'a {
    route
        .get(route_pos..)
        .unwrap_or_default()
        .iter()
        .take_while(move |&&leg| matches!(graph.link_kind(leg), LinkKind::Walk | LinkKind::Step))
        .map(move |&leg| graph.cell_origin(graph.link_target(leg)))
}

/// Straight-and-level runway from `origin` along a corridor of successive leg-target points: sum XY
/// leg lengths while the corridor keeps roughly its heading *and* stays roughly level, stopping
/// before the first ~96u chord that either turns more than `MAX_BEND` or **climbs** more than
/// `MAX_CLIMB`. The climb stop is why bots run (not hop) up stairs: an ascending staircase — a chain
/// of positive-dz Step legs, riser 8–18u per 32u cell — rises ~24u per chord and reads as "not a
/// bhop runway". Descents never stop it (hopping *down* stairs is fine), and a lone step inside an
/// otherwise level chord stays under the threshold (single steps are hoppable; pmove steps up on
/// landing anyway). Judging on chords rather than per 32u leg avoids misreading grid-quantized cell
/// centres (which zigzag between grid axes) as constant turning.
fn runway_over(origin: Vec3, targets: impl Iterator<Item = Vec3>) -> f32 {
    const CHORD: f32 = 96.0;
    const MAX_BEND: f32 = 35.0;
    const MAX_CLIMB: f32 = 20.0;
    let (mut dist, mut prev) = (0.0, origin.xy());
    let (mut anchor, mut anchor_dist, mut anchor_z) = (origin.xy(), 0.0, origin.z);
    let mut chord_yaw = None::<f32>;
    for tgt in targets {
        let t = tgt.xy();
        dist += (t - prev).length();
        prev = t;
        if dist - anchor_dist >= CHORD {
            let c = t - anchor;
            let yaw = yaw_of(c);
            if chord_yaw.is_some_and(|p| wrap180(yaw - p).abs() > MAX_BEND) || tgt.z - anchor_z > MAX_CLIMB {
                return anchor_dist; // the corridor turned or climbed in this chord — stop before it
            }
            chord_yaw = Some(yaw);
            (anchor, anchor_dist, anchor_z) = (t, dist, tgt.z);
        }
    }
    dist
}

/// See [`runway_over`]; this is the graph-backed wrapper over the leading ground legs.
fn runway(graph: &NavGraph, route: &[u32], route_pos: usize, origin: Vec3) -> f32 {
    runway_over(origin, ground_leg_targets(graph, route, route_pos))
}

/// The point at arc-distance `d` along a corridor of successive leg-target points from `origin`
/// (clamped to the last point if the corridor is shorter). Placing the bhop look-ahead here at a
/// **speed-scaled** distance gives a fast bot enough anticipation to start curving into the corridor,
/// instead of chasing a fixed ~2-legs-ahead point it has already overrun.
fn point_along(origin: Vec3, targets: impl Iterator<Item = Vec3>, d: f32) -> Vec3 {
    let mut prev = origin;
    let mut acc = 0.0;
    for tgt in targets {
        let seg = (tgt.xy() - prev.xy()).length();
        if acc + seg >= d {
            let t = if seg > 1e-3 { (d - acc) / seg } else { 0.0 };
            return prev.lerp(tgt, t);
        }
        acc += seg;
        prev = tgt;
    }
    prev
}

/// See [`point_along`]; graph-backed wrapper over the leading ground legs.
fn corridor_point(graph: &NavGraph, route: &[u32], route_pos: usize, origin: Vec3, d: f32) -> Vec3 {
    point_along(origin, ground_leg_targets(graph, route, route_pos), d)
}

/// A wander destination for an idle bot with nothing to chase: a random reachable navmesh cell,
/// refreshed on arrival or every few seconds. Keeps bots moving on a human-less server instead of
/// freezing on the spawn (the "bots stand still with no human" case).
fn roam_target(game: &mut GameState, e: EntId, origin: Vec3, now: f32) -> Vec3 {
    let (wt, wtime) = {
        let b = &game.entities[e].bot;
        (b.wander.target, b.wander.time)
    };
    let reached = wt != Vec3::ZERO && (wt.xy() - origin.xy()).length() < 64.0;
    if wt != Vec3::ZERO && !reached && now < wtime {
        return wt;
    }
    // Draw a handful of candidates before the graph borrow (each `random` needs `&mut game`).
    let draws: [f32; 8] = std::array::from_fn(|_| game.random());
    let Some(g) = game.nav.graph.as_ref() else {
        return origin;
    };
    if g.cells.is_empty() {
        return origin;
    }
    let pick = |r: f32| ((r * g.cells.len() as f32) as usize).min(g.cells.len() - 1);
    // Prefer a safe, dry destination clear of the lifts — roaming into water to turn around is loiter,
    // roaming onto lava/slime is suicide, and roaming under a raised plat parks a body where it blocks
    // the lift's descent. Take the first draw that is none of those; if every draw lands wet, burning or
    // under a lift (a mostly-flooded/flooded-with-liquid map) keep the last.
    let idx = draws
        .iter()
        .map(|&r| pick(r))
        .find(|&i| {
            !g.cell_in_water(i as u32) && g.cell_hazard(i as u32).is_none() && g.cell_under_plat(i as u32).is_none()
        })
        .unwrap_or_else(|| pick(draws[7]));
    let cell = g.cell_origin(idx as u32);
    game.entities[e].bot.wander.target = cell; // disjoint field from game.nav — coexists with `g`
    game.entities[e].bot.wander.time = now + 5.0;
    cell
}

/// The nearest cell to `bot_cell` (by travel cost) where a bot can breathe — the anti-drown target.
/// Floods link costs once and takes the cheapest breathable cell (its own cell, at cost 0, when that
/// already breaks the surface). `None` if every reachable cell is underwater (a sealed flooded
/// pocket), leaving the caller to just swim up toward any open surface.
fn nearest_air(graph: &NavGraph, bot_cell: CellId, costs: &LinkCosts) -> Option<CellId> {
    let flood = graph.costs_from(bot_cell, costs);
    flood
        .iter()
        .enumerate()
        .filter(|&(c, &d)| d.is_finite() && graph.cell_breathable(c as CellId))
        .min_by(|&(_, &a), &(_, &b)| a.total_cmp(&b))
        .map(|(c, _)| c as CellId)
}

/// The origin of the nearest breathing spot for a drowning bot, cached in `cache` for
/// [`SURFACE_CACHE_TTL`] so the graph flood ([`nearest_air`]) doesn't run every frame while it swims
/// out. `None` if no breathable cell is reachable.
fn surface_target(cache: &mut Wander, graph: &NavGraph, bot_cell: CellId, costs: &LinkCosts, now: f32) -> Option<Vec3> {
    if cache.target != Vec3::ZERO && now < cache.time {
        return Some(cache.target);
    }
    let picked = nearest_air(graph, bot_cell, costs).map(|c| graph.cell_origin(c));
    cache.target = picked.unwrap_or(Vec3::ZERO);
    cache.time = now + SURFACE_CACHE_TTL;
    picked
}

/// The nearest cell to `bot_cell` (by travel cost) with safe footing — the burn-escape target. Floods
/// link costs once and takes the cheapest cell that isn't lava/slime (water counts as safe here:
/// diving into a pool to escape lava is right). `None` only if every reachable cell burns.
fn nearest_safe_ground(graph: &NavGraph, bot_cell: CellId, costs: &LinkCosts) -> Option<CellId> {
    let flood = graph.costs_from(bot_cell, costs);
    flood
        .iter()
        .enumerate()
        .filter(|&(c, &d)| d.is_finite() && graph.cell_hazard(c as CellId).is_none())
        .min_by(|&(_, &a), &(_, &b)| a.total_cmp(&b))
        .map(|(c, _)| c as CellId)
}

/// The origin of the nearest safe-footing spot for a burning bot, cached in `cache` for
/// [`SURFACE_CACHE_TTL`] like [`surface_target`]. `None` if no safe cell is reachable.
fn escape_target(cache: &mut Wander, graph: &NavGraph, bot_cell: CellId, costs: &LinkCosts, now: f32) -> Option<Vec3> {
    if cache.target != Vec3::ZERO && now < cache.time {
        return Some(cache.target);
    }
    let picked = nearest_safe_ground(graph, bot_cell, costs).map(|c| graph.cell_origin(c));
    cache.target = picked.unwrap_or(Vec3::ZERO);
    cache.time = now + SURFACE_CACHE_TTL;
    picked
}

/// The nearest living human player to bot `bot_e` (skips bots, spectators, and the dead).
fn nearest_human(game: &GameState, bot_e: EntId) -> Option<EntId> {
    let maxclients = game.host().cvar(c"maxclients") as i32;
    let origin = game.entities[bot_e].v.origin;
    let mut best: Option<(EntId, f32)> = None;
    for i in 1..=maxclients as u32 {
        let e = EntId(i);
        if e == bot_e {
            continue;
        }
        let ent = &game.entities[e];
        if !ent.in_use || ent.bot.is_bot || !ent.is_player() {
            continue;
        }
        if !ent.is_alive() {
            continue;
        }
        let d = (ent.v.origin - origin).length_squared();
        if best.is_none_or(|(_, bd)| d < bd) {
            best = Some((e, d));
        }
    }
    best.map(|(e, _)| e)
}


/// Drop bot bookkeeping when a bot client disconnects (kicked, or removed by the manager), so
/// a slot reused by a future human isn't mistaken for a bot.
pub fn on_disconnect(ent: &mut Entity) {
    if ent.bot.is_bot {
        ent.bot = BotState::default();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bsp::Bsp;
    use crate::mode::team::MatchConfig;
    use crate::navmesh::NavGraph;

    #[test]
    fn hard_mode_objectives_forbid_item_detours() {
        assert!(hard_mode_objective(Some(BotIntent::Move(Vec3::ZERO))));
        assert!(hard_mode_objective(Some(BotIntent::Spectate {
            goal: Vec3::ZERO,
            watch: EntId(1),
        })));
        assert!(!hard_mode_objective(Some(BotIntent::Advance(Vec3::ZERO))));
        assert!(!hard_mode_objective(Some(BotIntent::Fight(EntId(1)))));
        assert!(!hard_mode_objective(None));
    }

    /// The corridor scan (used pure, no NavGraph): a straight level corridor is all runway; a sharp
    /// bend or an ascending staircase truncates it; a lone step or a descent does not.
    #[test]
    fn runway_over_stops_at_bends_and_ascents() {
        let origin = Vec3::ZERO;
        let flat: Vec<Vec3> = (1..=12).map(|i| Vec3::new(i as f32 * 32.0, 0.0, 0.0)).collect();
        assert!((runway_over(origin, flat.iter().copied()) - 384.0).abs() < 1.0, "flat corridor should be full length");

        // A 90° bend after ~128u: runway stops around the last straight chord (before the turn).
        let mut bend = flat[..4].to_vec();
        bend.extend((1..=8).map(|i| Vec3::new(128.0, i as f32 * 32.0, 0.0)));
        let r = runway_over(origin, bend.iter().copied());
        assert!((96.0..=200.0).contains(&r), "bend should stop the runway near it, got {r}");

        // Ascending staircase: 8u riser per 32u run ⇒ ~24u climb per 96u chord ⇒ stops early.
        let stairs: Vec<Vec3> = (1..=12).map(|i| Vec3::new(i as f32 * 32.0, 0.0, i as f32 * 8.0)).collect();
        assert!(runway_over(origin, stairs.iter().copied()) < 160.0, "should not treat stairs as runway");

        // Descending staircase (hopping down is fine) and a lone 16u step both stay full length.
        let down: Vec<Vec3> = (1..=12).map(|i| Vec3::new(i as f32 * 32.0, 0.0, -(i as f32) * 8.0)).collect();
        assert!(runway_over(origin, down.iter().copied()) > 350.0, "descending stairs should stay runway");
        let mut step = flat.clone();
        for p in step.iter_mut().skip(6) {
            p.z = 16.0; // a single 16u lip partway along an otherwise level corridor
        }
        assert!(runway_over(origin, step.iter().copied()) > 350.0, "a lone step should not truncate the runway");
    }

    #[test]
    fn point_along_walks_and_clamps() {
        let origin = Vec3::ZERO;
        let pts: Vec<Vec3> = (1..=4).map(|i| Vec3::new(i as f32 * 100.0, 0.0, 0.0)).collect();
        assert!((point_along(origin, pts.iter().copied(), 250.0).x - 250.0).abs() < 0.5);
        assert!((point_along(origin, pts.iter().copied(), 9999.0).x - 400.0).abs() < 0.5, "clamps to the last point");
        assert!((point_along(origin, pts.iter().copied(), 0.0).x - 0.0).abs() < 0.5);
    }

    #[test]
    fn bot_target_caps_in_warmup_and_freezes_live() {
        let open = MatchConfig { teams: 0, size: 0 };
        let pickup = MatchConfig { teams: 2, size: 0 }; // CTF: not structured
        let two_by_two = MatchConfig { teams: 2, size: 2 };
        // Open play and open team pickup pass the cvar through, warmup or not.
        assert_eq!(bot_target(5, 1, open, true), Some(5));
        assert_eq!(bot_target(5, 1, pickup, false), Some(5));
        // Structured warmup caps the fill to the empty seats (4 seats − 1 human = 3).
        assert_eq!(bot_target(5, 1, two_by_two, true), Some(3));
        assert_eq!(bot_target(2, 1, two_by_two, true), Some(2), "cvar below empty seats wins");
        assert_eq!(bot_target(5, 4, two_by_two, true), Some(0), "humans fill every seat");
        // Structured live → freeze (no add, no trim).
        assert_eq!(bot_target(5, 1, two_by_two, false), None);
    }

    /// On a real map, traversal routes must expose usable bhop runways. Regression for the grid
    /// staircase: cell centres sit on a 32u grid, so any route heading between grid axes zigzags
    /// (segments alternating 0°/45°), and a naive per-segment bend test reads that as a constant
    /// sharp turn — truncating every runway to nothing and keeping bhop permanently off in play.
    /// Run with `RTX_TEST_BSP=…/dm6.bsp`; skipped (vacuously green) when unset.
    #[test]
    fn real_map_routes_have_runways() {
        let Ok(path) = std::env::var("RTX_TEST_BSP") else {
            return;
        };
        let bytes = std::fs::read(&path).expect("read bsp");
        let bsp = Bsp::parse(&bytes).expect("parse bsp");
        let graph = NavGraph::build(&bsp);
        // Sample routes between far-apart cell pairs and record the best runway a bot would see
        // anywhere along each route.
        let mut best = 0.0f32;
        let (mut positions, mut kind_ok, mut run_ok, mut entry_ok) = (0u32, 0u32, 0u32, 0u32);
        let stride = (graph.cells.len() / 64).max(1);
        let sample: Vec<u32> = (0..graph.cells.len() as u32).step_by(stride).collect();
        let mut routes = 0;
        for (i, &a) in sample.iter().enumerate() {
            for &b in &sample[i + 1..] {
                let d = (graph.cell_origin(a).xy() - graph.cell_origin(b).xy()).length();
                if d < 600.0 {
                    continue;
                }
                let Some(route) = graph.find_path(a, b, &LinkCosts::default()) else {
                    continue;
                };
                routes += 1;
                for pos in 0..route.len() {
                    let origin = graph.cell_origin(graph.link_source(route[pos]));
                    let r = runway(&graph, &route, pos, origin);
                    best = best.max(r);
                    let walkish = matches!(graph.link_kind(route[pos]), LinkKind::Walk | LinkKind::Step);
                    let goal_far = (graph.cell_origin(b).xy() - origin.xy()).length() > 300.0;
                    positions += 1;
                    kind_ok += walkish as u32;
                    run_ok += (r >= bhop::RUNWAY_ENGAGE) as u32;
                    entry_ok += (walkish && goal_far && r >= bhop::RUNWAY_ENGAGE) as u32;
                }
                if routes >= 40 {
                    break;
                }
            }
            if routes >= 40 {
                break;
            }
        }
        assert!(routes > 0, "no long routes found to sample");
        eprintln!(
            "runway stats over {routes} routes, {positions} positions: best={best:.0}u \
             kind_ok={:.0}% runway_ok={:.0}% entry_ok={:.0}%",
            100.0 * kind_ok as f32 / positions as f32,
            100.0 * run_ok as f32 / positions as f32,
            100.0 * entry_ok as f32 / positions as f32,
        );
        assert!(
            best >= 400.0,
            "best runway across {routes} long routes is only {best:.0}u — real-map corridors \
             never clear the {:.0}u engage bar",
            bhop::RUNWAY_ENGAGE
        );
    }

    /// On bravado, the quad platform must be broadly reachable. Regression for jump-link octant
    /// dedup: the platform is an island whose only inbound route is a level ~192u jump from a
    /// nearby ledge, and the nearest-per-octant dedup let a short descending jump into the pit
    /// below shadow that crossing — leaving the island with zero inbound climb links, so bots
    /// never went for the quad. Run with `RTX_TEST_BSP=…/bravado.bsp`; skipped (vacuously green)
    /// when unset or when the map isn't bravado.
    #[test]
    fn bravado_quad_reachability() {
        let Ok(path) = std::env::var("RTX_TEST_BSP") else {
            return;
        };
        if !path.to_lowercase().contains("bravado") {
            return; // map-specific geometry — other RTX_TEST_BSP maps can't run it
        }
        let bytes = std::fs::read(&path).expect("read bsp");
        let bsp = Bsp::parse(&bytes).expect("parse bsp");
        let graph = NavGraph::build(&bsp);
        let quad = graph.nearest(Vec3::new(752.0, 24.0, 288.0)).expect("no cell near the quad");

        // The platform proper: the walk/step-connected plateau containing the quad cell (jump
        // links excluded, so the launch ledge across the void doesn't count as "on it").
        let mut on_platform = vec![false; graph.cells.len()];
        let mut stack = vec![quad];
        on_platform[quad as usize] = true;
        while let Some(c) = stack.pop() {
            for l in &graph.links {
                let walkish = matches!(l.kind, LinkKind::Walk | LinkKind::Step);
                for (a, b) in [(l.from, l.to), (l.to, l.from)] {
                    if walkish && a == c && !on_platform[b as usize] {
                        on_platform[b as usize] = true;
                        stack.push(b);
                    }
                }
            }
        }

        // Directed reachability *to* the quad: reverse flood over links.
        let mut reaches = vec![false; graph.cells.len()];
        let mut stack = vec![quad];
        reaches[quad as usize] = true;
        while let Some(c) = stack.pop() {
            for l in &graph.links {
                if l.to == c && !reaches[l.from as usize] {
                    reaches[l.from as usize] = true;
                    stack.push(l.from);
                }
            }
        }
        let reachable = reaches.iter().filter(|&&r| r).count();

        // Inbound climb links: non-drop links landing on the platform from off it.
        let climbs = graph
            .links
            .iter()
            .filter(|l| {
                on_platform[l.to as usize]
                    && !on_platform[l.from as usize]
                    && !matches!(l.kind, LinkKind::Drop)
            })
            .count();
        eprintln!(
            "quad platform: {} cells; reachable IN from {reachable}/{} cells; \
             inbound climb links onto platform = {climbs}",
            on_platform.iter().filter(|&&p| p).count(),
            graph.cells.len(),
        );
        assert!(climbs >= 1, "no inbound climb link onto the quad platform");
        assert!(
            reachable * 2 > graph.cells.len(),
            "quad reachable from only {reachable}/{} cells",
            graph.cells.len()
        );
    }

    /// On a real map, traversal routes must expose usable bhop runways. Regression for the grid
    /// staircase: cell centres sit on a 32u grid, so any route heading between grid axes zigzags
    /// (segments alternating 0°/45°), and a naive per-segment bend test reads that as a constant
    /// sharp turn — truncating every runway to nothing and keeping bhop permanently off in play.
    /// Run with `RTX_TEST_BSP=…/dm6.bsp`; skipped (vacuously green) when unset.
    #[test]
    fn air_commit_lifecycle() {
        // Airborne within the arc window → stay committed.
        assert_eq!(air_commit_decision(false, true, 0.5), AirRelease::Keep);
        // Grounded before ever leaving the floor is still run-up, even beyond the grace.
        assert_eq!(air_commit_decision(true, false, 0.5), AirRelease::Keep);
        // A route-index change while airborne cannot release the physical commitment; route state
        // is not an input to this function.
        assert_eq!(air_commit_decision(false, true, 1.0), AirRelease::Keep);
        // Landed after having been airborne (grounded past the grace) → release.
        assert_eq!(air_commit_decision(true, true, 0.3), AirRelease::Land);
        // Never took off, or still airborne, long past the budget → watchdog timeout.
        assert_eq!(air_commit_decision(true, false, 3.0), AirRelease::Timeout);
        assert_eq!(air_commit_decision(false, true, 3.0), AirRelease::Timeout);
    }

    #[test]
    fn fake_client_pickup_allowlist_includes_dynamic_ctf_objects_only() {
        for touch in [
            Touch::ItemHealth,
            Touch::ItemArmor,
            Touch::ItemWeapon,
            Touch::ItemAmmo,
            Touch::ItemPowerup,
            Touch::Backpack,
            Touch::Flag,
            Touch::Rune,
        ] {
            assert!(bot_pickup_touch(touch), "{touch:?} should be collected manually");
        }
        for touch in [Touch::Teleport, Touch::Hurt, Touch::ButtonTouch, Touch::Multi, Touch::PlatCenter] {
            assert!(!bot_pickup_touch(touch), "{touch:?} must remain engine/map-owned");
        }
    }

    #[test]
    fn penalty_scales_with_strikes_and_caps() {
        assert_eq!(link_penalty_secs(0), 0.0);
        assert_eq!(link_penalty_secs(1), PENALTY_STEP);
        assert_eq!(link_penalty_secs(2), 4.0 * PENALTY_STEP);
        assert_eq!(link_penalty_secs(10), PENALTY_CAP, "surcharge is capped");
        // The cap must stay far below the navmesh's closed-gate penalty (100_000s) so a failed-link
        // surcharge only reshapes a route, never forces one through a shut door.
        const { assert!(PENALTY_CAP < 1_000.0) };
    }

    #[test]
    fn transient_link_penalties_merge_instead_of_masking_each_other() {
        let mut penalties = vec![(7, 3.0)];
        merge_link_penalty(&mut penalties, 7, 2.0);
        merge_link_penalty(&mut penalties, 9, 1.5);
        assert_eq!(penalties, vec![(7, 5.0), (9, 1.5)]);
    }

    #[test]
    fn progress_stall_thresholds() {
        // Within the stall window: never flagged, however little progress.
        assert!(!progress_stalled(100.0, 0.0, 100.0, PROGRESS_STALL_TIME - 0.1));
        // Past the window with no meaningful improvement: stalled.
        assert!(progress_stalled(100.0, 0.0, 100.0, PROGRESS_STALL_TIME + 0.1));
        // Past the window but the remaining distance dropped well below best: not stalled.
        assert!(!progress_stalled(100.0, 0.0, 100.0 - PROGRESS_EPS - 1.0, PROGRESS_STALL_TIME + 0.1));
    }

    #[test]
    fn penalize_leg_bumps_and_exempts() {
        let mut b = BotState::default();
        // None link, and Plat/Teleport kinds, are no-ops.
        penalize_leg(&mut b, None, Some(LinkKind::Walk), 1.0);
        penalize_leg(&mut b, Some(3), Some(LinkKind::Plat), 1.0);
        penalize_leg(&mut b, Some(3), Some(LinkKind::Teleport), 1.0);
        assert!(b.failed_links.iter().all(|&(_, until, _)| until == 0.0), "exempt legs recorded nothing");
        // A walk leg records one strike; failing it again bumps strikes and refreshes expiry.
        penalize_leg(&mut b, Some(3), Some(LinkKind::Walk), 1.0);
        penalize_leg(&mut b, Some(3), Some(LinkKind::Walk), 2.0);
        let e = b.failed_links.iter().find(|&&(l, _, _)| l == 3).unwrap();
        assert_eq!((e.1, e.2), (2.0 + PENALTY_TTL, 2), "same leg bumped to 2 strikes, expiry refreshed");
    }

    #[test]
    fn penalize_link_records_plat_leg() {
        // The plat-wait timeout must strike a Plat link directly — the kind `penalize_leg` exempts.
        let mut b = BotState::default();
        penalize_leg(&mut b, Some(9), Some(LinkKind::Plat), 1.0);
        assert!(b.failed_links.iter().all(|&(_, until, _)| until == 0.0), "penalize_leg still exempts Plat");
        penalize_link(&mut b, 9, 1.0);
        let e = b.failed_links.iter().find(|&&(l, _, _)| l == 9).expect("plat link recorded");
        assert_eq!((e.1, e.2), (1.0 + PENALTY_TTL, 1), "plat link struck once");
    }

    #[test]
    fn in_footprint_margins() {
        let (lo, hi) = (Vec2::new(-16.0, -16.0), Vec2::new(16.0, 16.0));
        assert!(in_footprint(Vec2::ZERO, lo, hi, 0.0), "centre is inside");
        assert!(!in_footprint(Vec2::new(20.0, 0.0), lo, hi, 0.0), "20u past +X face is outside");
        assert!(in_footprint(Vec2::new(20.0, 0.0), lo, hi, 8.0), "within an 8u margin it's inside");
        assert!(!in_footprint(Vec2::new(25.0, 0.0), lo, hi, 8.0), "past the margin it's outside again");
    }

    #[test]
    fn plat_standoff_pushes_out_and_holds() {
        let (lo, hi) = (Vec2::new(-16.0, -16.0), Vec2::new(16.0, 16.0));
        // Already clear of the footprint + standoff: hold still (return the origin unchanged).
        let clear = Vec3::new(80.0, 0.0, 24.0);
        assert_eq!(plat_standoff(clear, lo, hi), clear, "outside the standoff, stand still");
        // Just inside the +X side: pushed straight out past fp_max.x + PLAT_STANDOFF, Z preserved.
        let near = Vec3::new(10.0, 2.0, 24.0);
        let out = plat_standoff(near, lo, hi);
        assert_eq!(out.x, 16.0 + PLAT_STANDOFF, "pushed past the +X face");
        assert_eq!(out.z, 24.0, "keeps the bot's own height");
        assert!(!in_footprint(out.xy(), lo, hi, PLAT_STANDOFF - 0.01), "result clears the standoff box");
        // Dead centre is degenerate (all faces equal) but must still resolve to a point outside.
        let centre = plat_standoff(Vec3::new(0.0, 0.0, 24.0), lo, hi);
        assert!(!in_footprint(centre.xy(), lo, hi, PLAT_STANDOFF - 0.01), "centre still escapes");
    }

    #[test]
    fn avoid_ring_marks_expires_and_evicts() {
        let mut b = BotState::default();
        b.mark_avoid(0, 100.0); // the 0 sentinel is ignored
        assert!(!b.is_avoided(0, 1.0));
        b.mark_avoid(5, 10.0);
        assert!(b.is_avoided(5, 9.0));
        assert!(!b.is_avoided(5, 10.0), "expiry is exclusive");
        assert!(!b.is_avoided(6, 9.0));
        // Overfill the 4-slot ring; the soonest-to-expire entry is evicted, later ones survive.
        for (item, until) in [(11u32, 20.0f32), (12, 30.0), (13, 40.0), (14, 50.0)] {
            b.mark_avoid(item, until);
        }
        assert!(!b.is_avoided(5, 9.0), "the earliest-expiring entry was evicted");
        assert!(b.is_avoided(14, 49.0) && b.is_avoided(11, 19.0));
    }
}
