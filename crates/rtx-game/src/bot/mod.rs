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

use crate::bot::state::{BotState, GrenadePhase, HookPhase, RjPhase};
use crate::defs::{
    Bits, DeadFlag, Flags, Items, Solid, TakeDamage, Weapon, BUTTON_ATTACK, BUTTON_JUMP, VEC_VIEW_OFS,
};
use crate::entity::{EntId, Entity, Touch};
use crate::game::{cstring, GameState};
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

/// Within this of the launch cell counts as "in stance" to jump from — tighter than the hook's
/// (the solve's ±16u launch perturb bounds how far off the spot the arc still lands).
const RJ_STANCE: f32 = 16.0;
/// Fire the jump once the smoothed view is within this many degrees of the solved fire angles.
const RJ_AIM_TOL: f32 = 2.0;
/// Give up the stance if the RL/aim/ground alignment hasn't let the jump go within this.
const RJ_STANCE_TIMEOUT: f32 = 2.5;
/// If the bot is still on the ground this long after pressing jump, the jump was swallowed — abort.
const RJ_LIFTOFF_TIMEOUT: f32 = 0.3;
/// Ballistic watchdog slack added to the solved airtime before we give up waiting to land.
const RJ_BALLISTIC_SLACK: f32 = 1.0;

/// Advance to the next route leg once within this of the current waypoint (≈ ¾ of a grid).
const ARRIVE_RADIUS: f32 = 24.0;

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
/// Airborne jump commitment (`air_leg`): once committed, still on the ground after this long means we
/// either landed or never got airborne — release and let normal navigation resume. Covers the one or
/// two ground frames at takeoff without oscillating.
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

/// Manually collect any item the bot is standing on. The engine doesn't run the trigger-touch
/// phase for `SetBotCMD` fake clients the way it does for human `SV_RunCmd`, so a bot would walk
/// onto a pickup and never actually take it — it'd just keep wanting it and circle. We replicate
/// the touch here, guarded by `solid == Trigger` (a respawning item that's already been taken is
/// non-solid → skipped) so this can't double-grant even if an engine *does* fire the touch.
fn bot_pickup_items(game: &mut GameState, e: EntId) {
    if !game.entities[e].is_alive() {
        return;
    }
    let origin = game.entities[e].v.origin;
    // Gather first (immutable borrow of nav/entities), then fire touches (needs `&mut game`).
    // Catalog items (weapons/ammo/health/armor/powerups)...
    let mut hits: Vec<EntId> = game
        .nav
        .goals
        .iter()
        .filter_map(|&(idx, _)| {
            let item = EntId(idx);
            let it = &game.entities[item];
            (it.v.solid == Solid::Trigger && on_item(origin, it.v.origin)).then_some(item)
        })
        .collect();
    // ...plus any dropped backpack we're standing on. Backpacks aren't in the static catalog (they
    // spawn on death / a teammate's toss), so without this a bot would walk over one and never take
    // it — the engine skips the trigger-touch phase for `SetBotCMD` fake clients.
    hits.extend(game.entities.iter().enumerate().filter_map(|(i, it)| {
        (it.touch == Touch::Backpack && it.v.solid == Solid::Trigger && on_item(origin, it.v.origin))
            .then_some(EntId(i as u32))
    }));
    let now = game.time();
    let goal_item = game.entities[e].bot.goal.item;
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
                game.entities[e].bot.mark_avoid(item.0, now + PICKUP_AVOID_TIME);
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
    Sense {
        host, now, frametime, msec, origin, v_angle, client, weapon, on_ground, in_water, alive, vz, air_jumped, enemy_seen_time, v_xy, speed, grapple_hook, has_grapple, hook_out, on_hook, anchor, reel_half_step,
        attack_finished, has_rl, ammo_rockets, health, armortype, armorvalue, quad,
    }
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
    let on_sj = game.entities[e].bot.sj_leg.is_some();
    // A rocket-jump leg freezes the route the same way (stance stands still, the arc flies fast).
    let on_rj = game.entities[e].bot.rj.phase != RjPhase::Idle;

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
    } else if host.cvar_bool(c"rtx_bot_pacifist") {
        // Global override, any mode: don't fight — just tail the nearest human around the map.
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
    //  - A **Fight** intent with `rtx_bot_greed` → a *combat detour*: still pick a compelling item
    //    (`select_combat_item` bars trivial ones) so the bot can break off to grab the quad / a
    //    needed weapon / big health, ktx-style, without abandoning combat — `enemy` stays set, so
    //    the combat overlay keeps aiming and firing whenever it has line of sight (see below).
    //  - Any other mode intent (a **Move** objective, or Fight with greed off) → no item chase.
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
    // Handoff hold: an idle bot may reserve a spawned RL/LG for a powerup-carrying teammate (standing
    // on it without taking it). While holding, the reservation owns `goal_item`, pre-empting normal
    // item selection; a fight or move objective (idle == false) drops the hold inside this call.
    let holding = game.update_handoff_hold(e, now, intent.is_none());
    if holding {
        // `update_handoff_hold` set `goal_item` to the held weapon — nothing else to pick this frame.
    } else if intent.is_none() || greedy {
        if now >= game.entities[e].bot.goal.next_pick {
            let pick = if greedy {
                game.select_combat_item(e)
            } else {
                game.select_item_goal(e)
            };
            let (new_item, new_cell) = pick.map_or((0, 0), |(it, c)| (it.0, c));
            let b = &mut game.entities[e].bot;
            if new_item != b.goal.item {
                b.goal.since = now; // restart the watchdog for a new goal
            }
            (b.goal.item, b.goal.item_cell) = (new_item, new_cell);
            b.goal.next_pick = now + GOAL_SELECT_INTERVAL;
        }
        if game.entities[e].bot.goal.item != 0 && !game.item_goal_valid(e, EntId(game.entities[e].bot.goal.item), now) {
            let b = &mut game.entities[e].bot;
            b.goal.item = 0;
            b.goal.next_pick = now; // re-pick next frame
        }
    } else {
        game.entities[e].bot.goal.item = 0; // a Move objective supersedes any item chase
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
        let msg = cstring(&format!(
            "rtx bot{client}: want={goal} dist={dist:.0} on_item={overlap} ownLG={own_lg} cells={:.0} pen={pen} aware={aware} est={est} hold={hold} vig={vig} watch={wat}\n",
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
    // A committed item goal drives *movement* — set by the idle brain, or by the greedy combat
    // detour (Fight + `rtx_bot_greed`). Under a Fight intent the enemy is still tracked above, so
    // the combat overlay keeps aiming/firing on line of sight (and its range-keeping owns movement
    // then); the detour only steers navigation while the enemy is *out* of sight, when navigation
    // would otherwise just beeline the enemy. This is ktx's "the enemy is one more goal" in effect.
    let chasing = game.entities[e].bot.goal.item != 0;
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
        b.goal.next_pick = now; // re-pick (skipping the abandoned item) next frame
    }

    // Audience watch (arena Spectate): snapshot the chosen fighter's eye point so the look override
    // in `run_bot` — deep inside the `&mut bot` / `&nav` borrow — can point the eyes there without
    // re-reading another entity. `None` for every other intent leaves the eyes on the corridor.
    let watch_point = match intent {
        Some(BotIntent::Spectate { watch, .. }) => Some(game.entities[watch].v.origin + VEC_VIEW_OFS),
        _ => None,
    };

    Objective { hooking, on_sj, on_rj, enemy, chasing, polite, target_origin, item_cell, watch_point, vigil: vigil.is_some() }
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

    // View + move for the frame. Bunnyhopping bypasses the aim spring and the world-move reprojection
    // entirely — an air-strafe needs the view yaw swept *independently* of the travel direction, with
    // `forward = 0` and one strafe key held, which the reprojection can't express. The controller
    // already decided the whole cmd (air strafe, landing strafe+jump, or ground prestrafe); consume
    // it here. Otherwise the normal aim spring smooths the view and the world move is projected onto it.
    let (view, forward, side) = if let Some(c) = bhop_cmd {
        let dt = frametime.clamp(0.001, 0.05);
        let view = Vec3::new(look.x, c.view_yaw, 0.0);
        // Seed the aim spring so combat re-acquisition continues from the real view with a plausible
        // turn rate (a human-like flick) instead of snapping.
        let b = &mut game.entities[e].bot;
        let yv = if b.aim.bhop_prev_yaw == 0.0 {
            0.0
        } else {
            (wrap180(view.y - b.aim.bhop_prev_yaw) / dt).clamp(-720.0, 720.0)
        };
        b.aim.bhop_prev_yaw = view.y;
        b.aim.angles = view;
        b.aim.vel = Vec3::new(0.0, yv, 0.0);
        (view, c.forward.round() as i32, c.side.round() as i32)
    } else {
        game.entities[e].bot.aim.bhop_prev_yaw = 0.0; // forget the bhop yaw so the next engage seeds clean
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
        if err < RJ_AIM_TOL {
            buttons |= BUTTON_JUMP;
            let b = &mut game.entities[e].bot;
            b.rj.phase = RjPhase::Rise;
            b.rj.jump_time = now;
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
        b.air_leg = None;
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

    let o = resolve_objective(game, e, now, origin, client);
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
    let Some(goal_cell) = item_cell.or_else(|| graph.nearest(target_origin)) else {
        idle(v_angle);
        return;
    };

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
        },
    );

    // The `&nav` / `&mut bot` steering borrows have ended; the spine resumes with `&mut game`.
    // Combat overlay: with an enemy in sight, `engage` picks the look (live aim with drifting error)
    // and its own movement; traversal-critical legs are locked out (see `SteerOut::traversal_lock`).
    if let Some(en) = enemy.filter(|_| !traversal_lock) {
        combat::engage(game, e, en, origin, now, &mut cmd);
    }
    // Splash-weapon overlays, after `engage`: react to live grenades, finish a lob->shoot combo, take
    // a one-shot rocket hazard shove, else start a grenade combo. Skipped while hooking/rj/bhop-ing or
    // locked into a jump traversal (movement/buttons already spoken for) — that is `overlays_ok`.
    if overlays_ok {
        let handled = combat::grenade_tactics(game, e, enemy, origin, &mut cmd);
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

/// Wrap an angle into (-180, 180].
pub(crate) fn wrap180(a: f32) -> f32 {
    let mut a = a % 360.0;
    if a > 180.0 {
        a -= 360.0;
    } else if a < -180.0 {
        a += 360.0;
    }
    a
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
        let penalties = self.entities[e]
            .bot
            .failed_links
            .iter()
            .filter(|&&(_, until, _)| until > now)
            .map(|&(li, _, strikes)| (li, link_penalty_secs(strikes)))
            .collect();
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
fn in_footprint(p: Vec2, fp_min: Vec2, fp_max: Vec2, margin: f32) -> bool {
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
    /// Landed (or advanced off the jump leg): release; normal navigation/stuck handling resumes.
    Land,
    /// Still airborne well past any real arc: abandon the leg (penalize + re-path).
    Timeout,
}

/// Pure core of the airborne-commitment lifecycle. `on_jump_leg` is whether the current leg is still a
/// JumpGap/DoubleJump; `elapsed` is time since the commitment latched.
fn air_commit_decision(on_ground: bool, on_jump_leg: bool, elapsed: f32) -> AirRelease {
    if !on_jump_leg || (on_ground && elapsed > AIR_COMMIT_GRACE) {
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

/// How far a bunnyhop runway extends from `route_pos`: sum the lengths of the leading `Walk`/`Step`
/// legs while the corridor keeps roughly its heading, stopping at the first sharp turn, non-ground
/// leg, or route end. Each segment may bend up to ~30° from the *previous* segment — measuring
/// against the neighbour rather than the initial heading lets a gently curving corridor accumulate
/// (the weave tracks such bends fine); the old initial-heading test cut real-map runways so short
/// that bhop barely ever engaged.
fn runway(graph: &NavGraph, route: &[u32], route_pos: usize, origin: Vec3) -> f32 {
    // Judge bends on ~chord-length spans, not per 32u leg: grid-quantized cell centres zigzag on
    // any heading between grid axes, which a per-segment angle test misreads as constant turning.
    const CHORD: f32 = 96.0;
    const MAX_BEND: f32 = 35.0;
    let (mut dist, mut prev) = (0.0, origin.xy());
    let (mut anchor, mut anchor_dist) = (origin.xy(), 0.0);
    let mut chord_yaw = None::<f32>;
    for &leg in route.get(route_pos..).unwrap_or_default() {
        if !matches!(graph.link_kind(leg), LinkKind::Walk | LinkKind::Step) {
            break;
        }
        let tgt = graph.cell_origin(graph.link_target(leg)).xy();
        dist += (tgt - prev).length();
        prev = tgt;
        if dist - anchor_dist >= CHORD {
            let c = tgt - anchor;
            let yaw = c.y.atan2(c.x).to_degrees();
            if chord_yaw.is_some_and(|p| wrap180(yaw - p).abs() > MAX_BEND) {
                return anchor_dist; // the corridor turned somewhere in this chord — stop before it
            }
            chord_yaw = Some(yaw);
            anchor = tgt;
            anchor_dist = dist;
        }
    }
    dist
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
    let r = game.random(); // before borrowing the graph (needs &mut game)
    let Some(g) = game.nav.graph.as_ref() else {
        return origin;
    };
    if g.cells.is_empty() {
        return origin;
    }
    let idx = ((r * g.cells.len() as f32) as usize).min(g.cells.len() - 1);
    let cell = g.cell_origin(idx as u32);
    game.entities[e].bot.wander.target = cell; // disjoint field from game.nav — coexists with `g`
    game.entities[e].bot.wander.time = now + 5.0;
    cell
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

/// QuakeWorld `AngleVectors` (roll assumed 0): the view's forward, right, and up unit vectors —
/// exactly what the engine's `makevectors` produces and `w_fire_grenade` orients the launch by.
pub(crate) fn angle_vectors(angles: Vec3) -> (Vec3, Vec3, Vec3) {
    let (sy, cy) = angles.y.to_radians().sin_cos();
    let (sp, cp) = angles.x.to_radians().sin_cos();
    let forward = Vec3::new(cp * cy, cp * sy, -sp);
    let right = Vec3::new(sy, -cy, 0.0);
    let up = Vec3::new(sp * cy, sp * sy, cp);
    (forward, right, up)
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
        // Airborne on the jump leg, within the arc window → stay committed.
        assert_eq!(air_commit_decision(false, true, 0.5), AirRelease::Keep);
        // On the takeoff frame (grounded, tiny elapsed) → still committed (grace absorbs it).
        assert_eq!(air_commit_decision(true, true, 0.1), AirRelease::Keep);
        // Landed (grounded past the grace) → release.
        assert_eq!(air_commit_decision(true, true, 0.3), AirRelease::Land);
        // Advanced off the jump leg (kind no longer a jump) → release regardless of ground state.
        assert_eq!(air_commit_decision(false, false, 0.5), AirRelease::Land);
        // Still airborne long past any real arc → watchdog timeout.
        assert_eq!(air_commit_decision(false, true, 3.0), AirRelease::Timeout);
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
