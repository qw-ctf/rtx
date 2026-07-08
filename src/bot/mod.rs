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
mod rj;
pub(crate) mod state;
mod vigil;

use crate::bot::state::{BotState, GrenadePhase, HookPhase, RjPhase};
use crate::defs::{
    Bits, Flags, Items, Solid, Weapon, BOT_MOVE_SPEED as MOVE_SPEED, BUTTON_ATTACK, BUTTON_JUMP, VEC_VIEW_OFS,
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

// --- population management (P3) ---

/// Reconcile the live bot count to `rtx_bot_count`, one add/remove per call (called each normal
/// server frame). No-ops until a navmesh exists for the map, so bots never spawn blind.
pub fn manage_population(game: &mut GameState) {
    let host = *game.host();
    let maxclients = host.cvar(c"maxclients") as i32;

    // Tally humans and bots in one pass.
    let (mut humans, mut count, mut last_bot) = (0, 0, None);
    for i in 1..=maxclients as u32 {
        let ent = &game.entities[EntId(i)];
        if ent.bot.is_bot {
            count += 1;
            last_bot = Some(EntId(i));
        } else if ent.in_use && ent.classname() == Some("player") {
            humans += 1;
        }
    }

    // Field bots while at least one human is in the game — an empty server (or one whose last human
    // just left) wants none, so the trim path below removes them. `rtx_bot_alone` overrides that,
    // keeping bots on even with no humans (a demo/idle server that plays itself).
    let want = if humans >= 1 || host.cvar_bool(c"rtx_bot_alone") {
        host.cvar(c"rtx_bot_count").max(0.0) as i32
    } else {
        0
    };

    // In a structured match, cap the fill to the empty seats during warmup and freeze the roster once
    // the match is under way — see `bot_target`.
    let in_warmup = matches!(game.team_match.phase, crate::mode::MatchPhase::Warmup);
    let want = match bot_target(want, humans, game.team_match.config, in_warmup) {
        Some(w) => w,
        None => return, // structured match live — don't add or trim (would bench noise / drop a rostered bot)
    };

    // Build the navmesh on demand the first time bots are actually wanted.
    if want > 0 {
        game.ensure_navmesh();
        if !game.nav.is_loaded() {
            return;
        }
    }

    if count < want {
        add_one_bot(game, count);
    } else if count > want {
        if let Some(e) = last_bot {
            host.remove_bot(game.entities[e].bot.client);
            game.retire_slot(e); // fully retire the slot (bot state + in_use/classname/arena)
        }
    }
}

/// How many bots to field this frame, given the raw `cvar_want`, the human count, the resolved
/// composition, and whether a team match is in warmup. Open play (and CTF pickup) passes `cvar_want`
/// through. A **structured** match caps the fill to the empty seats during warmup — so bots exactly
/// top up teams×size around the humans — and returns `None` (freeze: don't add or trim) once the
/// match is live, since a fresh bot would only be benched and a trim could drop a rostered one. Pure.
fn bot_target(cvar_want: i32, humans: i32, cfg: crate::mode::team::MatchConfig, in_warmup: bool) -> Option<i32> {
    let structured = cfg.teams >= 2 && cfg.size >= 1;
    if !structured {
        return Some(cvar_want);
    }
    if !in_warmup {
        return None;
    }
    let seats = (cfg.teams * cfg.size) as i32;
    Some(cvar_want.min((seats - humans).max(0)))
}

/// Spawn one bot. `add_bot` runs the module's ClientConnect + PutClientInServer for the new
/// edict synchronously, then we tag that edict as bot-driven. No-op if the server is full.
fn add_one_bot(game: &mut GameState, index: i32) {
    let host = *game.host();
    let name = cstring(&format!("[rtx]{}", bot_name(index)));
    let (bottom, top) = bot_colors(index);
    // `add_bot` already sets the bot's name in its userinfo and broadcasts it (the
    // "[rtx]Grunt entered the game" line) — don't re-set "name" afterwards: doing so renamed the
    // bot to an empty string and is what kept bots off the scoreboard.
    let client = host.add_bot(&name, bottom, top, c"base");
    if client > 0 {
        game.entities[EntId(client as u32)].bot = BotState {
            is_bot: true,
            client,
            goal_cell: u32::MAX,
            ..Default::default()
        };
    }
}

/// A rotating set of bot names.
fn bot_name(index: i32) -> &'static str {
    const NAMES: [&str; 8] = [
        "Grunt",
        "Ranger",
        "Visor",
        "Sarge",
        "Bitterman",
        "Hossman",
        "Daemia",
        "Klesk",
    ];
    NAMES[(index as usize) % NAMES.len()]
}

/// Distinct shirt/pants colors per bot (QW palette 0–13).
fn bot_colors(index: i32) -> (i32, i32) {
    let c = [4, 11, 12, 13, 2, 6, 3, 10];
    (c[(index as usize) % c.len()], c[(index as usize * 3 + 1) % c.len()])
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
    if game.entities[e].v.health <= 0.0 || game.entities[e].v.deadflag != 0.0 {
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
    let goal_item = game.entities[e].bot.goal_item;
    let hold_item = game.entities[e].bot.hold_item;
    let holding = hold_item != 0 && now < game.entities[e].bot.hold_until;
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

/// What the bot is trying to do this frame — the output of [`resolve_objective`].
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
    let alive = game.entities[e].v.health > 0.0 && game.entities[e].v.deadflag == 0.0;
    // Vertical speed and whether the once-per-air-travel double jump is still available — snapshot
    // now, since the `&mut bot` binding below blocks reading the edict during the move logic.
    let vz = game.entities[e].v.velocity.z;
    let air_jumped = game.entities[e].combat.air_jumped;
    // When combat last had line of sight (see `combat::engage`) — snapshot for the bhop veto.
    let enemy_seen_time = game.entities[e].bot.enemy_seen_time;
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
        host, now, frametime, msec, origin, v_angle, client, weapon, on_ground, alive, vz, air_jumped, enemy_seen_time, v_xy, speed, grapple_hook, has_grapple, hook_out, on_hook, anchor, reel_half_step,
        attack_finished, has_rl, ammo_rockets, health, armortype, armorvalue, quad,
    }
}

fn resolve_objective(game: &mut GameState, e: EntId, now: f32, origin: Vec3, client: i32) -> Objective {
    let host = *game.host();
    // Hook invariant net: if we're mid-hook but no longer hold the grapple (a mode loadout stripped
    // it, e.g. Rocket Arena), abandon the traversal cleanly — release any live hook and reset the
    // phase. Runs before the nav borrow, where `&mut game` is free. Other aborts (leg changed, hook
    // vanished, timeouts) are handled inside the hook driver below.
    if game.entities[e].bot.hook_phase != HookPhase::Idle && !game.entities[e].v.items.has(Items::GRAPPLE) {
        if game.entities[e].grapple.hook_out {
            let hook = EntId(game.entities[e].grapple.hook);
            game.reset_grapple(hook);
        }
        game.entities[e].bot.hook_phase = HookPhase::Idle;
        game.entities[e].bot.hook_fails = 0;
    }
    let hooking = game.entities[e].bot.hook_phase != HookPhase::Idle;
    // Rocket-jump invariant net: mid-RJ but the RL or its ammo is gone (dropped, spent, stripped) —
    // abort so the bot doesn't jump into a shot it can't fire. Timeouts/misfires are the driver's.
    if game.entities[e].bot.rj_phase != RjPhase::Idle
        && (!game.entities[e].v.items.has(Items::ROCKET_LAUNCHER) || game.entities[e].v.ammo_rockets < 1.0)
    {
        game.entities[e].bot.rj_phase = RjPhase::Idle;
        game.entities[e].bot.rj_fails = 0;
    }
    // On a speed-jump leg the route must be frozen: the link's `from` is the runway start, now behind
    // the bot, so a repath would drop the link and turn the bot around at speed. Treated like `hooking`.
    let on_sj = game.entities[e].bot.sj_leg.is_some();
    // A rocket-jump leg freezes the route the same way (stance stands still, the arc flies fast).
    let on_rj = game.entities[e].bot.rj_phase != RjPhase::Idle;

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
            game.entities[e].bot.vis_since = 0.0;
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
        if now >= game.entities[e].bot.goal_select_time {
            let pick = if greedy {
                game.select_combat_item(e)
            } else {
                game.select_item_goal(e)
            };
            let (new_item, new_cell) = pick.map_or((0, 0), |(it, c)| (it.0, c));
            let b = &mut game.entities[e].bot;
            if new_item != b.goal_item {
                b.goal_started = now; // restart the watchdog for a new goal
            }
            (b.goal_item, b.goal_item_cell) = (new_item, new_cell);
            b.goal_select_time = now + GOAL_SELECT_INTERVAL;
        }
        if game.entities[e].bot.goal_item != 0 && !game.item_goal_valid(e, EntId(game.entities[e].bot.goal_item), now) {
            let b = &mut game.entities[e].bot;
            b.goal_item = 0;
            b.goal_select_time = now; // re-pick next frame
        }
    } else {
        game.entities[e].bot.goal_item = 0; // a Move objective supersedes any item chase
    }

    // Item vigil: if the goal item isn't collectable yet (mid-respawn, or a weapon held for a
    // teammate) and we're already standing near it, cruise a short walk off and scan the room instead
    // of twitching on the spot. Returns the overridden navigation target; `None` = carry on normally.
    let vigil = if game.entities[e].bot.goal_item != 0 {
        vigil::maybe(game, e, origin, holding, now)
    } else {
        None
    };

    // Opt-in diagnostics (`rtx_bot_debug 1`): one throttled line per bot — what it wants, how far,
    // whether it's standing on that item, and whether it owns the LG. Pinpoints pickup-vs-desire.
    if host.cvar_bool(c"rtx_bot_debug") && now >= game.entities[e].bot.repath_time {
        let gi = game.entities[e].bot.goal_item;
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
        let aware = (b.known_enemy != 0 && now < b.known_until) as i32;
        let hold = b.hold_item;
        // Opponent-model telemetry: this bot's current hypothesis of the enemy it's aware of — the
        // estimated health/armor stack and believed arsenal bits — so the shared read can be watched
        // converge and reset live. Blank (`est=-`) when there's no belief / modeling is off.
        let est = if b.known_enemy != 0 && now < b.known_until {
            game.opponent_est(e, EntId(b.known_enemy), now)
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
    let chasing = game.entities[e].bot.goal_item != 0;
    let goal_item_org = {
        let it = EntId(game.entities[e].bot.goal_item);
        (game.entities[it].v.origin, Some(game.entities[e].bot.goal_item_cell))
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
    let giveup = if game.is_powerup_item(EntId(game.entities[e].bot.goal_item)) {
        goals::POWERUP_GIVEUP
    } else {
        GOAL_GIVEUP_TIME
    };
    if chasing && game.entities[e].bot.gate.is_none() && now - game.entities[e].bot.goal_started > giveup {
        let b = &mut game.entities[e].bot;
        b.mark_avoid(b.goal_item, now + GOAL_AVOID_TIME);
        b.goal_item = 0;
        b.goal_select_time = now; // re-pick (skipping the abandoned item) next frame
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
        let yv = if b.bhop_prev_yaw == 0.0 {
            0.0
        } else {
            (wrap180(view.y - b.bhop_prev_yaw) / dt).clamp(-720.0, 720.0)
        };
        b.bhop_prev_yaw = view.y;
        b.aim = view;
        b.aim_vel = Vec3::new(0.0, yv, 0.0);
        (view, c.forward.round() as i32, c.side.round() as i32)
    } else {
        game.entities[e].bot.bhop_prev_yaw = 0.0; // forget the bhop yaw so the next engage seeds clean
        let dt = frametime.clamp(0.001, 0.05);
        let skill = host.cvar(c"rtx_bot_skill").clamp(0.0, 7.0);
        // Spring stiffness (1/s): sluggish → pro-snappy. Shared with the combat feed-forward,
        // whose lag compensation assumes exactly this spring.
        let omega = combat::aim_omega(skill);
        let b = &mut game.entities[e].bot;
        if b.aim == Vec3::ZERO {
            b.aim = v_angle; // seed from the real view so the first frame doesn't snap from zero
        }
        let spring = |a: f32, v: f32, target: f32| {
            let d = wrap180(target - a);
            let v = v + (omega * omega * d - 2.0 * omega * v) * dt;
            (wrap180(a + v * dt), v)
        };
        let (pitch, pv) = spring(b.aim.x, b.aim_vel.x, look.x);
        let (yaw, yv) = spring(b.aim.y, b.aim_vel.y, look.y);
        b.aim = Vec3::new(pitch, yaw, 0.0);
        b.aim_vel = Vec3::new(pv, yv, 0.0);
        let view = b.aim;
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
            b.hook_phase = HookPhase::Flight;
            b.hook_started = now;
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
            b.rj_phase = RjPhase::Rise;
            b.rj_jump_time = now;
        }
    }
    // Flush any deferred hook release now the graph/bot borrows are done.
    if let Some(h) = hook.reset {
        game.reset_grapple(h);
    }

    // Combat/gate diagnostics: what the bot is chasing and whether it's stuck at a gate. Enable
    // with `rtx_bot_debug 1` (conprint shows without `developer`).
    if host.cvar_bool(c"rtx_bot_debug") {
        let gate = game.entities[e].bot.gate;
        let route = game.entities[e].bot.route.len();
        let hph = game.entities[e].bot.hook_phase;
        let rjph = game.entities[e].bot.rj_phase;
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
    let Sense {
        host, now, frametime, msec, origin, v_angle, client, weapon, on_ground, alive, vz,
        air_jumped, enemy_seen_time, v_xy, speed, grapple_hook, has_grapple, hook_out, on_hook,
        anchor, reel_half_step, attack_finished, has_rl, ammo_rockets, health, armortype,
        armorvalue, quad,
    } = s;
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
        if b.hold_item != 0 {
            (b.hold_item, b.hold_for, b.hold_until) = (0, 0, 0.0);
        }
        b.air_leg = None;
    }

    // Connected but never spawned (health 0, not dead): the engine defers `PutClientInServer` — the
    // full spawn that sets health/loadout — to the bot's spawn on a *bot frame*, which an empty
    // (bots-only) server never runs. So the bot sits at 0 health forever, and the respawn pulse below
    // can't help it (`death_think` only runs for `deadflag >= Dead`). Seed fresh spawn parms before
    // spawning; FFA/team keep those decoded parms, while fixed-kit modes overwrite them.
    if !alive && game.entities[e].v.deadflag == 0.0 {
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

    let Objective { hooking, on_sj, on_rj, enemy, chasing, polite, vigil, target_origin, item_cell, watch_point } =
        resolve_objective(game, e, now, origin, client);

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
    let gate_closed = game.gate_closed_flags();

    // Live lift states, for the plat standoff. A bot approaching a raised `func_plat` must hold
    // outside its inner trigger — standing under it resets the lift's lower-timer and it never comes
    // down (see `plat_statuses`). Read before the nav borrow (it reads the plat edicts).
    let plat_status = game.plat_statuses();

    // This bot's live failed-link surcharges: legs it recently failed to traverse (stuck, stalled
    // speed-jump, given-up hook) cost extra in *its* A* so the planner diverts instead of handing
    // back the identical dead route to retry until a coarse goal-timeout fires. Built here from an
    // immutable read so the owned Vecs outlive the disjoint `&mut bot` borrow below; new failures
    // recorded later this frame apply next frame. Expired entries are dropped as they're gathered.
    let penalties: Vec<(u32, f32)> = game.entities[e]
        .bot
        .failed_links
        .iter()
        .filter(|&&(_, until, _)| until > now)
        .map(|&(li, _, strikes)| (li, link_penalty_secs(strikes)))
        .collect();
    // Gates + this bot's penalties + a per-bot jitter seed (so two bots vary otherwise-equal routes
    // rather than treading an identical line — cheap route variety that also reads as more human) +
    // the rocket-jump fitness gate (so a bot with no RL / rocket / health plans around RJ links).
    let rj_extra = rj::rocket_jump_extra(
        &game.entities[e].v,
        game.entities[e].combat.super_damage_finished,
        now,
    );
    let costs = LinkCosts {
        gate_closed: &gate_closed,
        penalties: &penalties,
        jitter_seed: e.0,
        rocket_jump_extra: rj_extra,
    };

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

    // Whether each gate's activator can be triggered right now: a shoot activator is "ready" only
    // while it takes damage — re-triggerable triggers go dead during their cooldown.
    let gate_ready: Vec<bool> = (0..graph.gate_count())
        .map(|gi| {
            let g = graph.gate(gi);
            !g.shoot || game.entities[EntId(g.activator)].v.takedamage != 0.0
        })
        .collect();

    let bot = &mut game.entities[e].bot;

    // Airborne jump commitment (latched last frame at takeoff): while flying a plain jump leg, freeze
    // the route and lock out combat so an enemy appearing mid-arc can't flip the goal and yank us off
    // the jump. Read here (before the repath gate and leg-advance) like `on_sj`/`on_rj`.
    let on_air = bot.air_leg.is_some();

    // A teleport (or any large instant displacement) invalidates the planned route — drop it
    // and re-path from where we landed. ~200u in one frame is far beyond running/falling. Skipped
    // mid-hook: the reel and the parabola move fast on purpose and must not clear the hook route.
    if !hooking && !on_sj && !on_rj && bot.last_origin != Vec3::ZERO && (origin - bot.last_origin).length() > 200.0 {
        bot.route.clear();
        bot.repath_time = now;
    }
    bot.last_origin = origin;

    // Gate errand: drop it once the gate's door has opened — or give up if we stop making progress
    // toward its button (stuck at a door whose button we can't actually reach), so we don't camp
    // there. Progress-based, not a flat timeout: a button that's simply far across the map (e.g.
    // when we spawned right next to the door) still gets reached. Suspended mid-hook.
    if !hooking && !on_sj && !on_rj && bot.gate.is_some() {
        let gi = bot.gate.unwrap();
        let give_up = |bot: &mut BotState| {
            bot.avoid_gate = gi as i32;
            bot.avoid_gate_until = now + GATE_AVOID_TIME;
            bot.gate = None;
            bot.route.clear();
            bot.repath_time = now;
        };
        if gate_closed.get(gi).copied() != Some(true) {
            bot.gate = None; // door opened — done
            bot.route.clear();
            bot.repath_time = now;
        } else if !button_reachable(graph, bot_cell, gi, &costs) {
            give_up(bot); // button is walled off behind this very gate — route around instead
        } else {
            let d = (graph.cell_origin(graph.gate(gi).button_cell).xy() - origin.xy()).length();
            if d < bot.gate_best_dist - 4.0 {
                bot.gate_best_dist = d; // got closer — reset the give-up clock
                bot.gate_since = now;
            } else if now - bot.gate_since > GATE_GIVEUP_TIME {
                give_up(bot); // no progress toward a reachable button — stuck; try elsewhere
            }
        }
    }

    // Effective goal: the human, or — while opening a gate — that gate's button.
    let goal = match bot.gate {
        Some(gi) => graph.gate(gi).button_cell,
        None => goal_cell,
    };

    // Re-path when the route is empty, the goal changed, or the timer elapsed. Frozen mid-hook, on a
    // speed/rocket jump, or committed to a plain jump arc, so the traversal keeps the route that put
    // it on that leg (a goal flip mid-air must not replace the route and turn the bot around).
    if !hooking && !on_sj && !on_rj && !on_air && (bot.route.is_empty() || bot.goal_cell != goal || now >= bot.repath_time) {
        // Speed-band planning credits the speed a bot carries between legs (chained speed jumps,
        // cheaper hot Walk legs) — gated on bhop being on (no speed-jump links otherwise) plus its
        // own escape-hatch cvar. `speed` seeds the start band, so a mid-run re-path keeps a hop
        // chain alive. Falls back to the plain cell A* (bands all-zero) when off.
        let use_bands = host.cvar_bool(c"rtx_bot_bhop") && host.cvar_bool(c"rtx_bot_bandplan");
        let banded = |from, to| use_bands.then(|| graph.find_path_banded(from, to, speed, &costs)).flatten();
        let (mut route, mut bands) = match banded(bot_cell, goal) {
            Some(r) => (r.links, r.bands),
            None if use_bands => (Vec::new(), Vec::new()),
            None => (graph.find_path(bot_cell, goal, &costs).unwrap_or_default(), Vec::new()),
        };
        // Goal unreachable from here (behind a shut door with no way around from this spot, or a
        // disconnected pocket)? Don't home straight into a wall — head to the reachable cell
        // nearest the goal, approaching as far as the graph allows (often enough for line of sight
        // or to find a connection). Better than freezing until the target wanders into view.
        if route.is_empty() && bot_cell != goal {
            if let Some(near) = graph.nearest_reachable_to(bot_cell, goal, &costs) {
                match banded(bot_cell, near) {
                    Some(r) => (route, bands) = (r.links, r.bands),
                    None => route = graph.find_path(bot_cell, near, &costs).unwrap_or_default(),
                }
            }
        }
        // Keep `route_bands` parallel to `route`: zero-fill when unbanded (or on any length mismatch).
        if bands.len() != route.len() {
            bands = vec![0u8; route.len()];
        }
        bot.route = route;
        bot.route_bands = bands;
        bot.route_pos = 0;
        bot.goal_cell = goal;
        bot.repath_time = now + REPATH_INTERVAL;
        // Restart the progress watchdog against the new route (INFINITY ⇒ the first frame records the
        // real starting distance rather than reading as an instant stall on an old baseline).
        bot.progress_best = f32::INFINITY;
        bot.progress_since = now;
    }
    // If we've fallen off the planned route (missed a jump, got shoved), re-localize next.
    if !hooking && !on_sj && !on_rj && !on_air && bot.route_pos >= bot.route.len() && bot_cell != goal && now >= bot.repath_time {
        bot.repath_time = now; // force a fresh path next frame
    }

    // Not on an errand yet? `find_path` already routes *around* a shut gate when it can (its links
    // are priced high), so if the chosen route still crosses one, there's no other way in — divert
    // to that gate's button. Skip a gate we recently gave up on (its button was unreachable) so we
    // don't immediately re-camp on it.
    if !hooking && !on_sj && !on_rj && !on_air && bot.gate.is_none() {
        let avoid = if now < bot.avoid_gate_until { bot.avoid_gate } else { -1 };
        let block =
            route_blocking_gate(graph, &bot.route, bot.route_pos, &gate_closed).filter(|&gi| gi as i32 != avoid);
        if let Some(gi) = block {
            if button_reachable(graph, bot_cell, gi, &costs) {
                let button_cell = graph.gate(gi).button_cell;
                bot.gate = Some(gi);
                bot.gate_since = now;
                bot.gate_best_dist = f32::INFINITY; // first frame records the starting distance
                bot.route = graph.find_path(bot_cell, button_cell, &costs).unwrap_or_default();
                bot.route_bands = vec![0u8; bot.route.len()]; // a walking errand, no carried speed
                bot.route_pos = 0;
                bot.goal_cell = button_cell;
                bot.repath_time = now + REPATH_INTERVAL;
            } else {
                // Button is walled off behind this gate — don't chase it; avoid the gate so
                // route_blocking_gate stops re-selecting it and find_path routes around the pillar.
                bot.avoid_gate = gi as i32;
                bot.avoid_gate_until = now + GATE_AVOID_TIME;
            }
        }
    }

    // Advance past route legs we've already reached. A plat leg completes when we've *risen*
    // to the exit height (Z), not on XY arrival — we're standing still on the lift while it
    // carries us up, so XY barely changes.
    // A bunnyhopping bot covers ground fast enough to orbit a 24u waypoint, so widen the arrival gate
    // with speed and also advance once a waypoint slips *behind* the velocity.
    let arrive_r = if bot.bhop.phase != bhop::Phase::Off || bot.sj_leg.is_some() {
        ARRIVE_RADIUS.max(2.0 * speed * frametime)
    } else {
        ARRIVE_RADIUS
    };
    // While committed to a plain jump arc and still airborne, don't advance the leg: keep `kind` and
    // the waypoint pinned to the jump so steering stays on the landing point and the air-jump
    // undershoot recovery keeps firing (the leg advances naturally once we land). Like Hook/RocketJump,
    // whose drivers advance on landing, not on passing the target XY.
    while (on_ground || !on_air) && bot.route_pos < bot.route.len() {
        let leg = bot.route[bot.route_pos];
        let target = graph.cell_origin(graph.link_target(leg));
        let arrived = match graph.link_kind(leg) {
            LinkKind::Plat => origin.z >= target.z - PLAT_RISE_TOL,
            // A hook leg never auto-advances on XY: a near-vertical pull-up passes the XY test while
            // still at the *bottom* of the swing. The hook driver advances it only once the parabola
            // has landed (see below).
            LinkKind::Hook => false,
            // Same for a rocket jump — its driver advances on landing, not on passing the target XY.
            LinkKind::RocketJump => false,
            _ => {
                let to = target.xy() - origin.xy();
                let fast = bot.bhop.phase != bhop::Phase::Off || bot.sj_leg.is_some();
                to.length() <= arrive_r || (fast && to.dot(v_xy) < 0.0 && to.length() <= 64.0)
            }
        };
        if arrived {
            bot.route_pos += 1;
        } else {
            break;
        }
    }

    // Current waypoint + how to traverse to it. Past the route's end, home straight in on the
    // human (final approach). While riding a plat, steer toward the plat *centre* (the leg's
    // source cell) to stay aboard as it rises, instead of toward the far exit ledge.
    let (waypoint, kind, final_leg, cur_leg) = if bot.route_pos < bot.route.len() {
        let leg = bot.route[bot.route_pos];
        let k = graph.link_kind(leg);
        let aim = if k == LinkKind::Plat {
            graph.cell_origin(graph.link_source(leg))
        } else {
            graph.cell_origin(graph.link_target(leg))
        };
        (aim, Some(k), false, Some(leg))
    } else {
        (target_origin, None, true, None)
    };

    // Plat standoff. If an upcoming leg boards/rides a func_plat that isn't at its bottom, and we're
    // not already aboard it, walking to the board point would put us inside the lift's inner trigger
    // (the footprint shrunk 25u, spanning the full travel height) — and `plat_center_touch` resets
    // the lower-timer for any live player inside, so a bot waiting there would hold the lift raised
    // forever (and can wedge under a non-solid one). Instead hold a standoff outside the footprint
    // until it descends. The board leg itself may be a couple of Walk legs ahead, so scan a small
    // window and gate on proximity — the walk-in cells sit inside the full-height trigger too.
    let plat_hold: Option<usize> = bot
        .route
        .get(bot.route_pos..)
        .into_iter()
        .flatten()
        .take(PLAT_LOOKAHEAD)
        .find_map(|&l| graph.plat_of_link(l))
        .filter(|&pi| {
            let st = &plat_status[pi];
            let p = graph.plat(pi);
            let riding =
                origin.z > st.surface_z + 8.0 && in_footprint(origin.xy(), p.fp_min, p.fp_max, 0.0);
            !st.down && !riding && in_footprint(origin.xy(), p.fp_min, p.fp_max, PLAT_ENGAGE)
        });
    // While holding, steer to the standoff point and borrow the Plat leg's driver treatment (no
    // jump-press, no bhop entry, no air-latch, progress-watchdog exempt) by presenting `kind` as Plat.
    let (waypoint, kind) = match plat_hold {
        Some(pi) => {
            let p = graph.plat(pi);
            (plat_standoff(origin, p.fp_min, p.fp_max), Some(LinkKind::Plat))
        }
        None => (waypoint, kind),
    };
    // Plat-wait timeout: keyed on the plat index (not the leg, which the 0.4s repath churn rebuilds),
    // give up on a lift that never descends — a camped one, or a targeted plat only its own trigger
    // lowers — by striking its ride link so this bot's A* diverts, then re-path.
    match plat_hold {
        Some(pi) => {
            if bot.plat_wait != Some(pi) {
                bot.plat_wait = Some(pi);
                bot.plat_wait_since = now;
            } else if now - bot.plat_wait_since > PLAT_WAIT_TIMEOUT {
                let ride = bot.route[bot.route_pos..]
                    .iter()
                    .copied()
                    .find(|&l| graph.link_kind(l) == LinkKind::Plat && graph.plat_of_link(l) == Some(pi));
                if let Some(ride) = ride {
                    penalize_link(bot, ride, now);
                }
                bot.plat_wait = None;
                bot.route.clear();
                bot.repath_time = now;
            }
        }
        None => bot.plat_wait = None,
    }

    let hook_active = matches!(kind, Some(LinkKind::Hook)) || hooking;
    // Same for a rocket-jump leg: standing in stance and riding the blast arc must be exempt from the
    // stuck/progress watchdogs and the bhop veto, exactly like a hook leg.
    let rj_active = matches!(kind, Some(LinkKind::RocketJump)) || on_rj;
    // Where the *eyes* go while navigating: a couple of legs ahead of the feet (or the final
    // target when the route is short), so the view sweeps down the corridor instead of snapping
    // to every 32u grid cell the bot steps through. Steering still uses `waypoint`.
    //
    // But a Fight target we're *not* detouring on sets `target_origin` to the enemy's LIVE origin,
    // so aiming the eyes there while we can't see the enemy tracks it through walls — an aimbot
    // look. Once combat's 2s corner-hold lapses (or if we never saw them), look where we're
    // *travelling* instead. Non-combat targets — a human we follow, a committed item goal, or a
    // greedy detour (`chasing`) — are exactly where we want to look, so they keep `target_origin`.
    let combat_blind =
        enemy.is_some() && !chasing && (enemy_seen_time <= 0.0 || now - enemy_seen_time > LOOK_LOS_GRACE);
    let look_point = if vigil && bot.scan_point != Vec3::ZERO {
        // Standing vigil: sweep the eyes across the room (the scan point the aim spring pans to).
        // This drives the perception cone too (perception reads `bot.aim`), so it's real scouting;
        // combat's `engage` still overrides the moment a target comes into sight.
        bot.scan_point
    } else if let Some(pi) = plat_hold {
        // Holding off a raised lift: watch it, so we notice it descend (and combat's `engage` still
        // overrides the instant a target comes into sight).
        let p = graph.plat(pi);
        Vec3::new(
            (p.fp_min.x + p.fp_max.x) * 0.5,
            (p.fp_min.y + p.fp_max.y) * 0.5,
            plat_status[pi].surface_z + 24.0,
        )
    } else if bot.route_pos + 2 < bot.route.len() {
        graph.cell_origin(graph.link_target(bot.route[bot.route_pos + 2]))
    } else if combat_blind {
        // Past the route's end `waypoint` *is* `target_origin` (the enemy), so there fall through
        // to our actual travel heading rather than re-pointing the eyes at the hidden enemy.
        if final_leg && speed > 20.0 {
            origin + Vec3::new(v_xy.x, v_xy.y, 0.0)
        } else {
            waypoint
        }
    } else {
        target_origin
    };

    let goal_dist = (target_origin.xy() - origin.xy()).length();

    // Stuck detection. Suppressed mid-hook: standing in the throw stance, reeling, and riding the
    // parabola all look "stuck" to it, and a force-jump/repath there would wreck the traversal — the
    // hook driver's own per-phase timeouts are its stuck detection.
    let mut force_jump = false;
    if hook_active
        || rj_active
        || on_air
        || vigil
        || plat_hold.is_some()
        || (origin - bot.stuck_origin).length() > STUCK_MOVE
    {
        bot.stuck_origin = origin;
        bot.stuck_since = now;
    } else if now - bot.stuck_since > STUCK_TIME {
        force_jump = true;
        // Penalize the leg we're wedged on so the forced re-path actually *diverts* — without this
        // the deterministic A* hands back the identical route and the bot re-wedges every 0.7s.
        penalize_leg(bot, cur_leg, kind, now);
        bot.repath_time = now; // re-path next frame
        bot.stuck_since = now;
    }

    // Path-progress watchdog: catches a bot that *is* moving (so the displacement detector above
    // stays satisfied) yet makes no headway toward the goal — orbiting a pillar, sliding along a
    // wall, riding a mis-linked jump back and forth. If the straight-line distance to the goal hasn't
    // improved by `PROGRESS_EPS` for `PROGRESS_STALL_TIME`, treat the current leg as failing: penalize
    // it and re-path. Suspended while hooking / on a committed speed-jump / riding a plat (all of which
    // legitimately hold or reverse XY progress for a while).
    let plat_leg = matches!(kind, Some(LinkKind::Plat));
    if !hook_active && !rj_active && !on_sj && !on_air && !plat_leg && !vigil {
        if progress_stalled(bot.progress_best, bot.progress_since, goal_dist, now) {
            penalize_leg(bot, cur_leg, kind, now);
            bot.route.clear();
            bot.repath_time = now;
            bot.progress_best = goal_dist;
            bot.progress_since = now;
        } else if goal_dist < bot.progress_best - PROGRESS_EPS {
            bot.progress_best = goal_dist;
            bot.progress_since = now;
        }
    } else {
        // Keep the baseline current so a stall isn't falsely flagged the instant we resume.
        bot.progress_best = goal_dist;
        bot.progress_since = now;
    }

    // Bunnyhop policy verdicts — everything that needs game state is judged here; *when* each
    // verdict may apply in the hop cycle (engage hysteresis, mid-hop commitment, landing-only
    // disengage) is `bhop::Bhop::step`'s job. The entry runway bar is deliberately fixed:
    // the old `speed·0.9` bar rose as the bot gained speed and cut runs short mid-air.
    let runway_dist = runway(graph, &bot.route, bot.route_pos, origin);
    // Combat only vetoes bhop while it *owns the view* — the enemy is in sight (or lost a moment
    // ago), when the eyes must aim, not sweep a strafe. A mere Fight target being chased across
    // the map is navigation, and navigation bunnyhops; in FFA every bot always has a target, so
    // gating on target existence kept bhop permanently off. The grace here is deliberately much
    // shorter than combat's 2s corner-hold: on a small open FFA map sight contact is frequent,
    // and a long window suppresses hopping almost everywhere.
    const BHOP_COMBAT_GRACE: f32 = 0.5;
    let combat_view = enemy.is_some() && enemy_seen_time > 0.0 && now - enemy_seen_time < BHOP_COMBAT_GRACE;
    let bhop_veto = !host.cvar_bool(c"rtx_bot_bhop")
        || combat_view
        || hook_active
        || rj_active
        // Spectating: a bhop cmd would overwrite the view yaw in `emit` and clobber the watch —
        // and a spectator strolling the stands shouldn't be bunnyhopping anyway.
        || watch_point.is_some()
        || bot.gate.is_some()
        || bot.grenade_phase != GrenadePhase::Idle;
    // The banded planner's intent for this run: a band ≥ 1 on the current or next leg means the
    // route was planned to carry speed here, so admit bhop even on a short leg (the goal-distance
    // gates below exist to avoid hopping on trivial approaches — the plan overrides that judgment)
    // and tell the controller to hold the chain across the waypoint rather than disengage per leg.
    let planned_band = bot.route_bands.get(bot.route_pos).copied().unwrap_or(0);
    let carry = planned_band >= 1 || bot.route_bands.get(bot.route_pos + 1).copied().unwrap_or(0) >= 1;
    let bhop_entry = !final_leg
        && matches!(kind, Some(LinkKind::Walk | LinkKind::Step))
        && (goal_dist > 300.0 || planned_band >= 1)
        && runway_dist >= bhop::RUNWAY_ENGAGE;
    // Lenient continuation gate for taking *another* hop from a landing: leg kinds churn as the
    // route advances, and a run in progress shouldn't be dumped by the stricter entry conditions.
    let bhop_sustain =
        matches!(kind, Some(LinkKind::Walk | LinkKind::Step)) && (goal_dist > 150.0 || planned_band >= 1);
    // Ground zigzag: a corridor too short for a hop ([`bhop::RUNWAY_ENGAGE`]) but straight and long
    // enough ([`bhop::ZIGZAG_ENGAGE`]) to gain speed from the circle-strafe alone. The controller
    // hands off to the hop cycle if `bhop_entry` opens up mid-run, and `bhop_veto` (which includes
    // `!rtx_bot_bhop`) still gates it, so this is purely a sub-toggle on the same controller.
    let zigzag_ok = host.cvar_bool(c"rtx_bot_zigzag")
        && matches!(kind, Some(LinkKind::Walk | LinkKind::Step))
        && !final_leg
        && goal_dist > 150.0
        && runway_dist >= bhop::ZIGZAG_ENGAGE;
    // A speed-jump leg is a *committed* bhop run-up + leap: engage bhop unconditionally (the link is a
    // pre-verified runway) and track it so the route stays frozen. Latch/clear `sj_leg` on the leg.
    let mut sj_active =
        matches!(kind, Some(LinkKind::SpeedJump)) && host.cvar_bool(c"rtx_bot_bhop") && !hook_active && !rj_active;
    if sj_active {
        if bot.sj_leg != cur_leg {
            bot.sj_leg = cur_leg;
            bot.sj_started = now;
        }
        // Watchdog: the route is frozen mid-leg, so if the run-up stalls (blocked, shoved, never
        // built speed) abandon it and re-path rather than wedging on the runway forever.
        if now - bot.sj_started > 4.0 {
            bot.sj_leg = None;
            bot.route.clear();
            bot.repath_time = now;
            sj_active = false;
        }
    } else if bot.sj_leg.is_some() {
        bot.sj_leg = None;
    }
    // Airborne commitment for a plain jump leg (JumpGap/DoubleJump): latch it (like `sj_leg`) so the
    // route stays frozen and combat locked until we land. Latched whenever we're on such a leg (the
    // grace in the release decision absorbs the one or two ground frames at takeoff); released on
    // landing, on advancing off the leg, or by the watchdog if we never come down.
    let on_jump_leg = matches!(kind, Some(LinkKind::JumpGap | LinkKind::DoubleJump));
    if on_jump_leg && bot.air_leg != cur_leg {
        bot.air_leg = cur_leg;
        bot.air_started = now;
    }
    if let Some(committed) = bot.air_leg {
        match air_commit_decision(on_ground, on_jump_leg, now - bot.air_started) {
            AirRelease::Keep => {}
            AirRelease::Land => bot.air_leg = None,
            AirRelease::Timeout => {
                penalize_leg(bot, Some(committed), kind, now);
                bot.air_leg = None;
                bot.route.clear();
                bot.repath_time = now;
            }
        }
    }
    // "Don't leap to your death": if we somehow reach the takeoff edge too slow to clear the gap,
    // hold the jump (keep accelerating) rather than launching short into it.
    let sj_takeoff = cur_leg
        .and_then(|l| graph.speed_jump_of_link(l))
        .map(|tr| (tr.takeoff, tr.v_req));
    let sj_hold = sj_active && {
        match sj_takeoff {
            Some((takeoff, v_req)) => {
                let to_edge = takeoff.xy() - origin.xy();
                (to_edge.length() < 48.0 || to_edge.dot(v_xy) < 0.0) && speed < v_req * 0.9
            }
            None => false,
        }
    };

    // Drive the hop-cycle controller (see `bhop::Bhop`). On a speed jump the runway is the
    // run-up to the takeoff edge and the bearing aims straight at the landing so the leap goes
    // across the gap; otherwise steer toward the look-ahead corridor point (smoother than the 32u
    // next cell) with as much straight-ish corridor as the route offers.
    let bhop_cmd = {
        let dt = frametime.clamp(0.001, 0.05);
        let accel = host.cvar(c"sv_accelerate");
        let maxspeed = host.cvar(c"sv_maxspeed");
        let env = bhop::Env {
            dt,
            accel: if accel > 0.0 { accel } else { 10.0 },
            maxspeed: if maxspeed > 0.0 { maxspeed } else { 320.0 },
        };
        let ahead = if sj_active { waypoint.xy() } else { look_point.xy() } - origin.xy();
        let to_wp = waypoint.xy() - origin.xy();
        let dir = if ahead.length() > 8.0 { ahead } else { to_wp };
        let bearing = dir.y.atan2(dir.x).to_degrees();
        let bhop_runway = match sj_takeoff {
            Some((takeoff, _)) if sj_active => (takeoff.xy() - origin.xy()).length(),
            _ => runway_dist,
        };
        let phase_was = bot.bhop.phase;
        let cmd = bot.bhop.step(
            &bhop::Input {
                v_xy,
                on_ground,
                bearing,
                runway: bhop_runway,
                eligible: bhop_entry,
                zigzag: zigzag_ok,
                sustain: bhop_sustain,
                veto: bhop_veto,
                committed: sj_active,
                carry,
                hold_jump: sj_hold,
                now,
            },
            &env,
        );
        // A phase transition is the interesting diagnostic moment — why a run started or ended.
        if bot.bhop.phase != phase_was && host.cvar_bool(c"rtx_bot_debug") {
            let why = if bot.bhop.phase == bhop::Phase::Off {
                format!(" ({})", bot.bhop.off_reason)
            } else {
                String::new()
            };
            host.conprint(&cstring(&format!(
                "rtx bot{client}: bhop {phase_was:?}->{:?}{why} spd={speed:.0} runway={bhop_runway:.0}\n",
                bot.bhop.phase,
            )));
        }
        cmd
    };
    let bhop_active = bhop_cmd.is_some();

    // Steering: face the waypoint and run toward it.
    let to_wp = waypoint.xy() - origin.xy();
    let dist = to_wp.length();
    let yaw = to_wp.y.atan2(to_wp.x).to_degrees();
    let mut angles = Vec3::new(0.0, yaw, 0.0);

    // Nav look target: eyes on the look-ahead point down the corridor (combat/gate may override
    // below). When the look point is basically on top of us (standing on the goal/waypoint), both it
    // and the steering yaw degenerate — `atan2` on a near-zero vector jitters frame to frame, which is
    // the source of the on-the-spot twitch — so hold the current smoothed view instead of chasing
    // noise. 48u guard (not 8) so a bot idling at a pickup doesn't re-solve a garbage angle.
    let eye = origin + Vec3::new(0.0, 0.0, 22.0);
    let to_look = look_point - eye;
    let mut look = if to_look.xy().length() > 48.0 {
        Vec3::new(
            -to_look.z.atan2(to_look.xy().length()).to_degrees(),
            to_look.y.atan2(to_look.x).to_degrees(),
            0.0,
        )
    } else if dist > 8.0 {
        angles // steering yaw is still meaningful — look where we're walking
    } else if bot.aim != Vec3::ZERO {
        bot.aim // standing still on the point — hold the current view, don't snap to yaw 0
    } else {
        v_angle
    };

    // Grappling-hook leg driver: fly a LinkKind::Hook leg (select the grapple, settle the view on
    // the anchor, throw, reel to build speed, release into a parabola onto the target ledge). Its
    // whole state machine lives in `hook::drive_hook`; here we just feed it the frame snapshot and
    // apply the HookDrive it returns. The deferred `reset` (needs `&mut game`) is flushed later.
    let hook = hook::drive_hook(
        graph,
        bot,
        hook::HookCtx {
            hook_active,
            cur_leg,
            enemy,
            hook_out,
            on_hook,
            grapple_hook,
            has_grapple,
            now,
            weapon,
            origin,
            on_ground,
            anchor,
            reel_half_step,
            chasing,
        },
    );
    // Whether the hook is actively steering this frame (survives the abort branches above).
    let hook_engaged = bot.hook_phase != HookPhase::Idle;
    let hook_lock = matches!(
        bot.hook_phase,
        HookPhase::Flight | HookPhase::Reel | HookPhase::Ballistic
    );

    // Rocket-jump leg driver: walk to the launch cell with the RL out, settle the aim on the solved
    // fire angles, jump, fire after the solved delay, ride the blast arc onto the ledge. Same shape as
    // the hook driver — a snapshot in, an `RjDrive` out that the code below applies.
    let rj = rj::drive_rj(
        graph,
        bot,
        rj::RjCtx {
            rj_active,
            cur_leg,
            enemy,
            chasing,
            now,
            weapon,
            origin,
            on_ground,
            attack_finished,
            weapons_hot,
            has_rl,
            ammo_rockets,
            health,
            armortype,
            armorvalue,
            quad,
        },
    );
    let rj_engaged = bot.rj_phase != RjPhase::Idle;
    let rj_lock = matches!(bot.rj_phase, RjPhase::Rise | RjPhase::Ballistic);

    if let Some(t) = hook.look_target {
        let d = t - eye;
        if d.xy().length() > 1.0 {
            look = Vec3::new(
                -d.z.atan2(d.xy().length()).to_degrees(),
                d.y.atan2(d.x).to_degrees(),
                0.0,
            );
        }
    }
    // Rocket-jump look: Stance/Rise hold the solved fire *angles* directly (the shot flies along the
    // view); Ballistic looks at the landing *point* (reprojected like the hook's).
    if let Some(a) = rj.look_target_angles {
        look = a;
    } else if let Some(t) = rj.look_target {
        let d = t - eye;
        if d.xy().length() > 1.0 {
            look = Vec3::new(
                -d.z.atan2(d.xy().length()).to_degrees(),
                d.y.atan2(d.x).to_degrees(),
                0.0,
            );
        }
    }
    // Audience watch (arena Spectate): eyes on the fighter the mode chose — already LOS-validated
    // there and held ~1-2s. Post-hoc like the hook/rj overrides, so bhop steering and the route
    // look-ahead stay untouched; the aim spring in `emit` turns it into a human pan and perception
    // follows through `bot.aim`. Same 48u degenerate-angle guard as the nav look. Audience bots
    // have no grapple/RL, so the hook/rj guard is belt-and-braces.
    if !hook_engaged && !rj_engaged {
        if let Some(t) = watch_point {
            if (t - eye).xy().length() > 48.0 {
                look = combat::angles_to(eye, t);
            }
        }
    }

    let (mut forward, mut side, mut buttons, mut impulse) = (0, 0, 0, 0);
    // Politely stop short only when tailing a human or roaming (`Objective::polite`). Everything
    // else walks all the way in: an item pickup needs its touch to fire, a race checkpoint is a
    // hull-sized touch box, and when hunting an enemy stopping short would halt the bot 64u out
    // — e.g. right at a door between it and its target (the combat layer manages the actual
    // fighting distance once it has line of sight). `polite` is never set alongside a chase or
    // a Fight intent, so it alone decides.
    let close_enough = final_leg && polite && dist <= POLITE_DIST;
    if !close_enough {
        let (fwd, right, _) = angle_vectors(angles);
        let dir = Vec3::new(to_wp.x, to_wp.y, 0.0).normalize_or_zero();
        forward = (fwd.dot(dir) * MOVE_SPEED) as i32;
        side = (right.dot(dir) * MOVE_SPEED) as i32;
    }
    // Jump only while on the ground: QW pmove jumps once per press and needs the button
    // released (airborne) before it'll fire again. Gating on ground state pulses it correctly,
    // so a jump that falls short is retried on the next landing instead of the bot getting
    // stuck holding +jump against a ledge.
    if on_ground
        && (force_jump
            || bhop_cmd.is_some_and(|c| c.jump)
            || matches!(kind, Some(LinkKind::JumpGap | LinkKind::DoubleJump)))
    {
        buttons |= BUTTON_JUMP;
    }
    // Mid-air (double) jump: rtx grants one air jump per air travel. On a double-jump leg, spend it
    // near the apex (`vz` small) to restack the arc and clear the wider gap; on a plain jump leg,
    // spend it as a *recovery* only when we're descending short of a higher target (an undershoot).
    // `air_jumped` gates re-pressing it, and the engine ignores it when the floor's close (landing).
    if !on_ground && !air_jumped && vz <= 30.0 {
        let air_jump = match kind {
            Some(LinkKind::DoubleJump) => true,
            Some(LinkKind::JumpGap) => vz < 0.0 && waypoint.z > origin.z + 20.0,
            _ => false,
        };
        if air_jump {
            buttons |= BUTTON_JUMP;
        }
    }

    // Opening a gate's button: once at it, face it and push (walk in) or shoot it.
    if let Some(gi) = bot.gate {
        let g = graph.gate(gi);
        let at_button =
            bot.route_pos >= bot.route.len() || (origin.xy() - graph.cell_origin(g.button_cell).xy()).length() < 40.0;
        if at_button {
            let d = g.aim - eye;
            let yaw = d.y.atan2(d.x).to_degrees();
            let pitch = -d.z.atan2(d.xy().length()).to_degrees();
            angles = Vec3::new(pitch, yaw, 0.0);
            look = angles; // the button needs a precise aim; the spring settles on it while parked
            buttons &= !BUTTON_JUMP;
            if g.shoot {
                // Switch to the shotgun and fire at the activator. If it's so high above us that
                // aiming would exceed the view-pitch limit (the shot lands under it), back
                // straight away first for a shallower angle — ground movement stays horizontal
                // regardless of look pitch, so we can keep aiming up while backpedalling. Only
                // fire while the activator is ready (not in its post-trigger cooldown).
                impulse = IMPULSE_SHOTGUN;
                if pitch < -68.0 {
                    forward = (-MOVE_SPEED) as i32;
                    side = 0;
                } else {
                    (forward, side) = (0, 0);
                    if weapon == Weapon::Shotgun && gate_ready[gi] {
                        buttons |= BUTTON_ATTACK;
                    }
                }
            } else {
                // Walk into the button to push it.
                let (fwd, right, _) = angle_vectors(Vec3::new(0.0, yaw, 0.0));
                let dir = (g.aim - origin).normalize_or_zero();
                forward = (fwd.dot(dir) * MOVE_SPEED) as i32;
                side = (right.dot(dir) * MOVE_SPEED) as i32;
            }
        }
    }

    // The frame's movement as a world-space velocity, decoupled from the view: smoothing the eyes
    // below can't change where the bot goes, and combat can steer independently of its aim.
    let (nf, nr, _) = angle_vectors(angles);
    let mut move_world = nf * forward as f32 + nr * side as f32;

    // Hook override: stand still while reeling/flying (the pull owns velocity; ground input would
    // fight it or, airborne, break the frictionless arc), or walk toward the throw stance in Aim.
    if hook_engaged {
        move_world = match hook.approach {
            _ if hook.stand => Vec3::ZERO,
            Some(src) => Vec3::new(src.x - origin.x, src.y - origin.y, 0.0).normalize_or_zero() * MOVE_SPEED,
            None => Vec3::ZERO,
        };
        buttons &= !BUTTON_JUMP;
        if hook.select {
            impulse = IMPULSE_GRAPPLE;
        }
    }

    // Rocket-jump override: walk to the launch cell (Stance), stand and hold the aim (Rise), or ride
    // the arc with a gentle wish toward the landing (Ballistic — the in-flight air-strafe correction).
    // The jump itself is pressed post-spring in `emit` (via `rj.jump_ready`); the rocket fires on the
    // driver's pure-timing `rj.fire`.
    if rj_engaged {
        move_world = match rj.approach {
            _ if rj.stand => Vec3::ZERO,
            Some(src) => Vec3::new(src.x - origin.x, src.y - origin.y, 0.0).normalize_or_zero() * MOVE_SPEED,
            None => rj
                .air_correct
                .map_or(Vec3::ZERO, |t| Vec3::new(t.x - origin.x, t.y - origin.y, 0.0).normalize_or_zero() * MOVE_SPEED),
        };
        buttons &= !BUTTON_JUMP; // the launch jump is pressed only via `emit`'s post-spring gate
        if rj.select {
            impulse = IMPULSE_ROCKET;
        }
        if rj.fire {
            buttons |= BUTTON_ATTACK;
        }
    }

    // Bundle the frame's decisions into one command for the combat/grenade overlays to mutate.
    let mut cmd = BotCmd { look, move_world, buttons, impulse };

    // Combat overlay: with an enemy in sight, the combat layer picks the look (live aim with a
    // drifting error) and its own movement; having *just lost* sight it holds the angle where the
    // enemy vanished while navigation keeps driving; otherwise navigation's look/move stand.
    // Traversal-critical legs lock out combat because `engage` owns movement and clears +jump; doing
    // that during a gap/double/speed jump cancels the route even though the planner chose it.
    let traversal_lock = hook_lock
        || rj_lock
        || on_air
        || matches!(
            kind,
            Some(LinkKind::JumpGap | LinkKind::DoubleJump | LinkKind::SpeedJump)
        );
    if let Some(en) = enemy.filter(|_| !traversal_lock) {
        combat::engage(game, e, en, origin, now, &mut cmd);
    }

    // Splash-weapon overlays, run after `engage` (they override its aim/movement) and only when not
    // flying a hook leg. Priority: (1) defensive/opportunistic reaction to live grenades — if it
    // handled the frame it wins and any stale combo is dropped; (2) finish an in-progress grenade
    // lob→shoot combo; (3) a one-shot rocket **hazard shove** when the bot is already positioned for
    // it (cheaper than a lob); (4) otherwise start a grenade combo (hazard shove via a lobbed arc, or
    // a plain airburst). The hazard shove is the generic strategy — the knockback shoves regardless
    // of which splash weapon delivers the blast. Skipped while bunnyhopping or locked into a jump
    // traversal, where movement/buttons are already spoken for.
    if !hook_engaged && !rj_engaged && !bhop_active && !traversal_lock {
        let handled = combat::grenade_tactics(game, e, enemy, origin, &mut cmd);
        if handled {
            game.entities[e].bot.grenade_phase = GrenadePhase::Idle;
        // defence drops a stale combo
        } else {
            // A one-shot rocket shove takes priority over *starting* a grenade combo, but never
            // interrupts one already in progress (the short-circuit keeps a running combo going).
            let running = game.entities[e].bot.grenade_phase != GrenadePhase::Idle;
            if running || !grenade::rocket_shove(game, e, enemy, origin, &mut cmd) {
                grenade::grenade_combo(game, e, enemy, origin, now, &mut cmd);
            }
        }
    }

    // Overlays done — unpack the command back into the frame's locals for the aim spring / emit.
    // `buttons` is still touched by the post-spring hook fire below; the rest are read-only now.
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
        (b.wander_target, b.wander_time)
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
    game.entities[e].bot.wander_target = cell; // disjoint field from game.nav — coexists with `g`
    game.entities[e].bot.wander_time = now + 5.0;
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
        if !ent.in_use || ent.bot.is_bot || ent.classname() != Some("player") {
            continue;
        }
        if ent.v.health <= 0.0 || ent.v.deadflag != 0.0 {
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
