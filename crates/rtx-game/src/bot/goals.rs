// SPDX-License-Identifier: AGPL-3.0-or-later

//! Bot item goals (plan P5) — deciding *which* pickup a bot should fetch, ktx-inspired.
//!
//! ktx scores each item by a **desirability** that's really the marginal *effective-HP* (for
//! health/armor) or firepower gain (for weapons/ammo) it would give *this* bot right now, then
//! weights that by how soon the bot could reach and collect it. We port that shape: a per-bot
//! [`Stats`] snapshot drives [`item_desire`](GameState::item_desire); a Dijkstra cost-flood
//! ([`NavGraph::costs_from`](crate::navmesh::NavGraph::costs_from)) gives the travel time to every
//! item at once; and the final score is `desire × (LOOKAHEAD − t) / (t + 5)` — closer and
//! sooner-available wins. Powerups (quad/pent/ring) get a flat dominating desire; the enviro suit
//! is ignored, as in ktx.
//!
//! Availability rides in through `t`: an item waiting to respawn costs the time until it's back
//! (so a bot will head for a quad that's about to return), and anything that won't be collectable
//! within `LOOKAHEAD` is dropped. The catalog of (item, cell) pairs is static per map and built
//! once with the navmesh (see `GameState::collect_goals`); the live availability and the bot's
//! own stats are read fresh each time a goal is chosen.

use glam::{Vec3, Vec3Swizzles};

use crate::arsenal::AmmoKind;
use crate::defs::{Bits, Items, Solid, RUNE_HASTE, RUNE_MASK, RUNE_REGEN, RUNE_RESISTANCE, RUNE_STRENGTH};
use crate::entity::{EntId, Think, Touch};
use crate::game::GameState;
use crate::navmesh::{CellId, LinkCosts, NavGraph, CLOSED_GATE_PENALTY};

/// Beyond this travel-or-respawn time (seconds) an *ordinary* item isn't worth pursuing.
pub(crate) const LOOKAHEAD: f32 = 10.0;

/// Powerups (quad/pentagram) are understood much farther out than ordinary pickups. This bounds the
/// score and any cross-map route; [`POWERUP_SETUP_LEAD`] separately prevents a nearby bot from leaving
/// before it is time to establish control. Ordinary items keep the tight [`LOOKAHEAD`].
const POWERUP_LOOKAHEAD: f32 = 30.0;
/// How early a bot may arrive to establish powerup control after accounting for its actual route.
/// This keeps cross-map foresight (`travel + lead`) without turning the last half of every Quad
/// cycle into a vigil directly on the spawn.
const POWERUP_SETUP_LEAD: f32 = 4.0;

/// Give-up leash for a *powerup* goal (bot/mod.rs's `GOAL_GIVEUP_TIME` for ordinary items is 10 s).
/// A cross-map quad run legitimately takes longer than that; the progress watchdog still catches a
/// genuinely stuck bot far sooner. Sized to [`POWERUP_LOOKAHEAD`] plus a margin.
pub(crate) const POWERUP_GIVEUP: f32 = 35.0;

/// Red armour and megahealth — the "holy grail" pickups — are understood farther out than a shard but
/// not as far as a powerup. The full 20-second cycle remains visible to scoring, while
/// [`MAJOR_SETUP_LEAD`] decides when a route should actually begin. Only when [`Stats::stack`]
/// discipline is on.
const MAJOR_LOOKAHEAD: f32 = 20.0;
/// RA/mega need a little timing margin, not their entire respawn spent camping the pickup. Two
/// seconds is enough to take a nearby position and contest while leaving most of the 20-second RA
/// cycle for weapons, health, and territory control.
const MAJOR_SETUP_LEAD: f32 = 2.0;
/// Give-up leash for a *major* (RA/mega) goal: longer than an ordinary item's 10 s (a cross-room run
/// is legitimate), shorter than a powerup's, sized to [`MAJOR_LOOKAHEAD`] plus a margin.
pub(crate) const MAJOR_GIVEUP: f32 = 25.0;

/// How far out a goal is understood, by tier. A quad is worth crossing the map for; red armour/mega
/// are worth a trek and a cycle; an ordinary shard only a short detour. The score curve
/// ([`item_score`]) decays each to zero at its own horizon, while timed departure is gated separately.
#[derive(Clone, Copy, PartialEq)]
enum Horizon {
    Ordinary,
    Major,
    Powerup,
}

/// Goal-valuation price (seconds) for crossing a closed gate whose button the bot can reach — the
/// button-detour errand the bot actually runs, standing in for the full [`CLOSED_GATE_PENALTY`] the
/// planner keeps. Small enough that a prize behind an openable door still fits inside the item and
/// powerup horizons, so the bot *chooses* it and heads over to work the button; large enough that an
/// item on the near side of an open corridor is still preferred to one that needs a door opened.
const GATE_OPEN_COST: f32 = 8.0;

/// Powerup team-split threshold: defer to any strictly nearer teammate. Exact distance ties settle
/// through the stable entity-id reservation, so only one teammate owns the item while the other is
/// free to cover armor, weapons, and approach routes.
const POWERUP_DEFER_RATIO: f32 = 1.0;

/// Minimum *desire* an item must have for a bot to break off combat and detour for it (the
/// optional `rtx_bot_greed` behavior). Set so a genuinely wanted weapon/health/armor swing clears
/// it while a minor ammo top-off (≈2.5) doesn't. Major powerups bypass this optional gate entirely.
pub(crate) const COMBAT_GREED_MIN_DESIRE: f32 = 40.0;

/// A health/armor pickup is a completion-critical local recovery when it adds at least this much
/// effective strength. At critical health any positive recovery qualifies instead.
const LOCAL_RECOVERY_GAIN: f32 = 25.0;
/// Only pickups reachable inside this travel-time budget can pre-empt combat as a local completion.
const LOCAL_PICKUP_TRAVEL: f32 = 1.0;
/// Euclidean pre-filter before the local Dijkstra. It is deliberately looser than one second of
/// stock running to admit a short stair/turn while avoiding a flood when no relevant item is nearby.
const LOCAL_PICKUP_RADIUS: f32 = 384.0;
/// While committed to a respawning powerup, only consider ordinary pickups this close and this
/// quick to reach. The second leg is checked against the powerup timer below, so this is a cost
/// bound as well as a guard against scanning/flooding unrelated items across the room.
const POWERUP_BRIDGE_TRAVEL: f32 = 1.5;
/// Navigation costs are quantized at cell/link granularity and the item touch happens before the
/// next route is built. Allow half a second over the direct arrival so a pickup lying on the route
/// is not rejected by those small bookkeeping differences; a real detour still has to fit inside
/// the respawn wait.
const POWERUP_BRIDGE_SLACK: f32 = 0.5;
/// Strategic recovery may break contact for an item reachable inside this many seconds.
const RECOVERY_TRAVEL: f32 = 4.0;

/// Waypoint magnetism (Phase 1). A desirable, up item within this Euclidean XY radius of the bot is a
/// candidate to bend the walk through — a side-step, not a detour, so it's kept short (the corridor
/// test in `steer` bounds the actual bend far tighter, to [`super::MAGNET_LATERAL`]).
const MAGNET_RADIUS: f32 = 160.0;
/// Only bend toward a magnet on roughly the bot's own floor: a pickup this many units above or below
/// is across a ledge/stair the walk doesn't cross, and steering at it would drag the bot off its path.
const MAGNET_DZ: f32 = 32.0;

/// Resource discipline (`rtx_bot_stack`). The bare-spawn effective-HP line: 50 health + 50 armour
/// absorb is exactly this, the >50/>50 stack the bot strives to keep. Below it, health/armour desire
/// scales up so the bot values topping its stack the way a human does — and reliably clears the
/// combat-detour bar mid-fight instead of walking thin-stacked into the next duel.
const STACK_NEUTRAL: f32 = 100.0;
/// Ceiling on the stack-pressure multiplier, reached at ≈ a third of the neutral stack. Capped so a
/// hurt bot raises health/armour up its priority list without becoming an item-obsessed coward.
const STACK_PRESSURE_MAX: f32 = 2.5;

/// Combat-posture stack thresholds (stack-aware, `rtx_bot_stack`). Enter Recover when health is one
/// rocket from death (30) *or* the whole stack is thin (60 EHP) — so 40hp+yellow (100 EHP) fights on
/// while 40hp naked (40 EHP) pulls back. Exit needs both a real health cushion and a rebuilt stack,
/// each with margin over its entry point so posture doesn't oscillate on a single pickup.
const RECOVER_ENTER_HEALTH: f32 = 30.0;
const RECOVER_ENTER_STRENGTH: f32 = 60.0;
const RECOVER_EXIT_HEALTH: f32 = 50.0;
const RECOVER_EXIT_STRENGTH: f32 = 90.0;

/// Escape-gated disengage (stack discipline). Breaking off to heal hands the feet to navigation, which
/// can turn the bot's back on the enemy. In mutual line of sight it only does so past this separation
/// — a lead long enough to reach cover/the pickup before a rocket lands; closer, the bot stays and
/// fights with combat's facing backpedal instead of getting shot in the back. A touch beyond the 400 u
/// preferred fighting range so a bot already backing off clears the bar as the gap opens. Live-tunable.
const DISENGAGE_SAFE_RANGE: f32 = 550.0;

/// Dry-primary ammo urgency (stack discipline). Below this firepower a bot armed with a primary is
/// about to be reduced to its shotgun/axe — a stack loss as real as low armour, so ammo for that
/// primary spikes in desire. 50 ≈ a fed super-nailgun, the best fallback worth staying above.
const AMMO_DRY_FIREPOWER: f32 = 50.0;
/// Base desire for a dry primary's ammo, deliberately equal to [`COMBAT_GREED_MIN_DESIRE`]: running a
/// primary dry is worth breaking off a fight to refill, so it clears the combat-detour bar exactly.
const AMMO_DRY_BASE: f32 = COMBAT_GREED_MIN_DESIRE;
/// How much the dry-ammo desire rises per point of firepower below the threshold — the emptier the
/// primary, the more urgent the refill (fp 0 → 75, fp 49 → ~41).
const AMMO_DRY_SLOPE: f32 = 0.7;

/// Bounded two-leg planning: only the strongest one-step candidates receive a second nav flood.
/// This makes the worst-case query count explicit (one origin flood + this many continuation floods).
const PLAN_PRIMARY_LIMIT: usize = 6;
/// Total time window for a second ordinary pickup. The first leg keeps the tighter [`LOOKAHEAD`]
/// so a bot does not cross the map for a shard; a useful nearby pickup may, however, bridge toward
/// the next strategic item in the KTX two-goal style.
const PLAN_LOOKAHEAD: f32 = 30.0;
/// A future pickup is valuable but less certain than the one we can collect first: availability,
/// combat, and inventory may change before arrival, so discount its score.
const SECONDARY_WEIGHT: f32 = 0.5;
/// How much of an observed opponent's item need becomes denial value for this bot.
const ENEMY_DENIAL_WEIGHT: f32 = 0.5;
/// A weaker bot yields an ordinary pickup when the known enemy reaches it first by this multiplier.
const LOST_CONTEST_MULT: f32 = 0.35;

/// Relative bonus the item a bot is *already* chasing gets in goal scoring, so a marginally-better
/// alternative doesn't make it flip-flop between two near-equal pickups each re-selection (a loop
/// the navigation watchdogs can't see, since the bot keeps moving). Q3's goal-selection dampening.
const GOAL_HYSTERESIS: f32 = 1.3;

/// Score multiplier for an ordinary item a teammate bot has already claimed (is fetching), so the
/// second bot strongly prefers another weapon/armor route. Timed powerups use deterministic ownership
/// below rather than this soft multiplier. Human Bravado winners shared an exact strategic goal only
/// 0.7% of sampled time; `0.1` still permits convergence when there truly is no competitive alternative.
const CLAIM_DISCOUNT: f32 = 0.1;

/// Desire *floor* for a big weapon (RL/LG) the enemy side is believed to lack, in a game where
/// weapons don't stay — item denial, the deathmatch-1 team play (weapons hide and respawn in 30 s,
/// so taking one keeps it out of the enemy's hands). 30 is chosen to sit:
///  - above every ammo/minor top-off desire (≈2.5–20), so an otherwise-idle bot spends the time
///    denying a weapon spawn instead of grabbing shells;
///  - below [`COMBAT_GREED_MIN_DESIRE`] (40), so a bot never breaks off a firefight to go camp one;
///  - far below a genuinely needed weapon (a missing RL scores ≈100−firepower) or a powerup (200+),
///    so real needs always outrank denial.
const DENIAL_DESIRE: f32 = 30.0;

/// The weapons a bot can want, and the ammo kinds. Real enums (not raw [`Items`] bits) so the
/// desire match stays exhaustive.
#[derive(Clone, Copy)]
enum WeaponKind {
    Ssg,
    Ng,
    Sng,
    Gl,
    Rl,
    Lg,
}

/// What kind of goal an item classname denotes. Armor carries its own absorb parameters so the
/// effective-HP math is a single branch.
#[derive(Clone, Copy)]
enum Category {
    /// Quad / pent / ring — always near-top priority.
    Powerup,
    Health,
    /// `value`/`at` feed `TotalStrength`; `gate` is the current-absorb threshold above which it's
    /// not worth taking; `double` weights the cheap green armor up (matching ktx).
    Armor {
        value: f32,
        at: f32,
        gate: f32,
        double: bool,
    },
    Weapon(WeaponKind),
    Ammo(AmmoKind),
}

/// Strategic value of a CTF rune. Resistance/Strength decide direct fights most strongly, while
/// Haste and Regeneration still outrank ordinary inventory. A bot already holding any rune scores
/// all rune pickups at zero (the game allows one).
fn rune_desire(held: u8, bit: u8) -> f32 {
    if held & RUNE_MASK != 0 {
        return 0.0;
    }
    match bit {
        RUNE_RESISTANCE => 240.0,
        RUNE_STRENGTH => 220.0,
        RUNE_HASTE => 200.0,
        RUNE_REGEN => 180.0,
        _ => 0.0,
    }
}

/// A bounded item plan returned to the objective arbiter. `second` is promoted and revalidated after
/// touching `first`; it is never followed blindly. `contains_powerup` makes a bridge-to-powerup plan
/// completion-critical even when its first pickup is ordinary.
#[derive(Clone, Copy)]
pub(crate) struct ItemPlan {
    pub first: (EntId, CellId),
    pub second: Option<(EntId, CellId)>,
    pub first_desire: f32,
    pub contains_powerup: bool,
    /// The plan reaches a genuinely-wanted major pickup (RA/mega whose post-contest desire clears the
    /// combat-detour bar), so a greed-off fighting bot may break off for it — but a bare denial floor
    /// (below the bar) can't, keeping [`select_major_item`] from yanking a bot out of every fight.
    pub contains_major: bool,
}

#[derive(Clone, Copy)]
struct ItemCandidate {
    item: EntId,
    cell: CellId,
    desire: f32,
    time: f32,
    mult: f32,
    horizon: Horizon,
    one_score: f32,
}

/// Travel costs for goal scoring: either the exact whole-graph flood or the LOD coarse estimate. Both
/// answer `cost_to(cell)`, so `best_item_plan`'s scoring is written once; `rtx_bot_lod` picks which
/// `best_item_plan` builds. An unreachable cell reads `INFINITY` from either.
enum PlanCosts<'a> {
    Exact(Vec<f32>),
    Coarse(crate::navmesh::CoarseCosts<'a>),
}
impl<'a> PlanCosts<'a> {
    /// The goal-scoring costs from `from` under `costs`: the coarse LOD estimate when `lod`, else an
    /// exact whole-graph flood. The single place the exact/coarse choice is written for a lone source —
    /// the batched exact floods (`bot_pool.join`/`flood_batch`) stay explicit at their call sites, since
    /// they fan out and have no coarse equivalent.
    fn single(graph: &'a NavGraph, from: CellId, costs: &'a LinkCosts<'a>, lod: bool) -> PlanCosts<'a> {
        if lod {
            PlanCosts::Coarse(graph.coarse_costs(from, costs, true))
        } else {
            PlanCosts::Exact(graph.costs_from(from, costs))
        }
    }

    #[inline]
    fn cost_to(&self, cell: CellId) -> f32 {
        match self {
            // An out-of-range cell (a goal cell held stale across a navmesh rebuild) reads as
            // unreachable rather than panicking the game module — the graceful skip the pre-refactor
            // `costs.get(cell)?` gave, now at the single chokepoint every reader passes through.
            PlanCosts::Exact(v) => v.get(cell as usize).copied().unwrap_or(f32::INFINITY),
            PlanCosts::Coarse(c) => c.cost_to(cell),
        }
    }
}

/// Classify a pickup by its classname (the enviro suit and anything unlisted return `None`, so
/// bots ignore them).
fn category(classname: &str) -> Option<Category> {
    use AmmoKind::*;
    use WeaponKind::*;
    Some(match classname {
        "item_artifact_super_damage" | "item_artifact_invulnerability" | "item_artifact_invisibility" => {
            Category::Powerup
        }
        "item_health" => Category::Health,
        "item_armor1" => Category::Armor {
            value: 100.0,
            at: 0.3,
            gate: 30.0,
            double: true,
        },
        "item_armor2" => Category::Armor {
            value: 150.0,
            at: 0.6,
            gate: 90.0,
            double: false,
        },
        "item_armorInv" => Category::Armor {
            value: 200.0,
            at: 0.8,
            gate: 160.0,
            double: false,
        },
        "weapon_supershotgun" => Category::Weapon(Ssg),
        "weapon_nailgun" => Category::Weapon(Ng),
        "weapon_supernailgun" => Category::Weapon(Sng),
        "weapon_grenadelauncher" => Category::Weapon(Gl),
        "weapon_rocketlauncher" => Category::Weapon(Rl),
        "weapon_lightning" => Category::Weapon(Lg),
        "item_shells" => Category::Ammo(Shells),
        "item_spikes" => Category::Ammo(Nails),
        "item_rockets" => Category::Ammo(Rockets),
        "item_cells" => Category::Ammo(Cells),
        _ => return None,
    })
}

/// Whether a classname is any kind of bot goal — used to build the static per-map catalog.
pub(crate) fn is_goal_classname(classname: &str) -> bool {
    category(classname).is_some()
}

/// `TotalStrength` — a fighter's *effective* hit points: health scaled by armor absorption,
/// capped by health-plus-armor. The currency ktx weights health/armor pickups in.
pub(crate) fn total_strength(health: f32, armor_value: f32, armor_type: f32) -> f32 {
    (health / (1.0 - armor_type)).min(health + armor_value).max(0.0)
}

/// How much more a bot values a health/armour pickup given its current effective HP: `1.0` at or
/// above the bare-spawn stack, rising to [`STACK_PRESSURE_MAX`] as the stack thins toward death — the
/// >50/>50 discipline as a smooth curve rather than a cliff. `1.0` when stack discipline is off.
fn stack_pressure(strength: f32) -> f32 {
    (STACK_NEUTRAL / strength.max(1.0)).clamp(1.0, STACK_PRESSURE_MAX)
}

/// The health/armour desire multiplier for a bot: [`stack_pressure`] of its current stack when
/// resource discipline is on, else `1.0`. Applied to the marginal-EHP desire so a thin stack raises
/// health/armour up the priority list; a full one leaves it at ktx-parity value.
fn stack_mult(s: &Stats) -> f32 {
    if s.stack {
        stack_pressure(s.strength)
    } else {
        1.0
    }
}

/// The denial desire floor for a weapon pickup: [`DENIAL_DESIRE`] for a rocket launcher or lightning
/// gun the enemy side provably lacks in a no-weapons-stay game, else `0` (nothing to deny). Only the
/// two big weapons are worth denying — the lesser guns don't swing a fight enough to camp a spawn.
fn denial_floor(kind: WeaponKind, weapons_stay: bool, enemy_side_has_bit: bool) -> f32 {
    if weapons_stay || enemy_side_has_bit {
        return 0.0;
    }
    match kind {
        WeaponKind::Rl | WeaponKind::Lg => DENIAL_DESIRE,
        _ => 0.0,
    }
}

/// Whether `other_team` is an enemy of `my_team`. In FFA (`my_team == 0`) every other player is their
/// own one-person team, so *everyone else* is an enemy to deny; in a team composition it's the players
/// on a different team. Denies to a shared function so weapon/item denial reads the same in both.
fn enemy_of(my_team: u8, other_team: u8) -> bool {
    my_team == 0 || other_team != my_team
}

/// How long a bot will hold a weapon for a powerup carrier before taking it itself. Under a third of
/// the 30 s powerup window, so the carrier still profits from what's left, yet long enough to cross a
/// map's mid.
const HOLD_MAX: f32 = 9.0;
/// A carrier is only worth holding for if its powerup has at least this long left — no point reserving
/// a weapon for a quad that's about to expire before the carrier can arrive and use it.
const HOLD_MATE_MIN_POWER: f32 = 3.0;
/// How near a spawned RL/LG a bot must be to take up the hold — a positioning nudge, not a cross-map
/// march (the carrier, not the holder, is the one that should travel).
const HOLD_REACH: f32 = 700.0;
/// A perceived enemy within this range of the held weapon contests it: the bot takes it (denial)
/// rather than leave it on the floor for the enemy.
const HOLD_CONTEST_RANGE: f32 = 400.0;

/// Pure core of the handoff-hold "keep going?" decision: a hold runs only while the deadline is in the
/// future, the weapon is still on the floor, and the carrier is still alive, still powered, and still
/// lacking it — with no enemy contesting the spot. Any failed condition ends the hold, at which point
/// a bot standing on the (still-solid) weapon grabs it (denial) and otherwise re-picks a goal.
fn hold_continues(
    now: f32,
    hold_until: f32,
    item_on_floor: bool,
    mate_alive: bool,
    mate_powered: bool,
    mate_has_weapon: bool,
    enemy_contesting: bool,
) -> bool {
    now < hold_until && item_on_floor && mate_alive && mate_powered && !mate_has_weapon && !enemy_contesting
}

/// The `Items` bit a weapon pickup grants (to check whether the bot already owns it).
fn weapon_bit(w: WeaponKind) -> Items {
    match w {
        WeaponKind::Ssg => Items::SUPER_SHOTGUN,
        WeaponKind::Ng => Items::NAILGUN,
        WeaponKind::Sng => Items::SUPER_NAILGUN,
        WeaponKind::Gl => Items::GRENADE_LAUNCHER,
        WeaponKind::Rl => Items::ROCKET_LAUNCHER,
        WeaponKind::Lg => Items::LIGHTNING,
    }
}

/// The best weapon a backpack's `items` bitfield carries, if any (a death drop holds a single
/// weapon, but be defensive and take the strongest). Axe/shotgun carry no bit we care about — the
/// bot always owns those — so only the six pickup weapons map.
fn weapon_kind_of(items: f32) -> Option<WeaponKind> {
    use WeaponKind::*;
    [
        (Items::LIGHTNING, Lg),
        (Items::ROCKET_LAUNCHER, Rl),
        (Items::GRENADE_LAUNCHER, Gl),
        (Items::SUPER_NAILGUN, Sng),
        (Items::NAILGUN, Ng),
        (Items::SUPER_SHOTGUN, Ssg),
    ]
    .into_iter()
    .find_map(|(bit, w)| items.has(bit).then_some(w))
}

/// The ammo a weapon pickup carries — what you still gain by re-grabbing an owned weapon when it
/// respawns (i.e. not under weapons-stay).
fn weapon_ammo(w: WeaponKind) -> AmmoKind {
    match w {
        WeaponKind::Ssg => AmmoKind::Shells,
        WeaponKind::Ng | WeaponKind::Sng => AmmoKind::Nails,
        WeaponKind::Gl | WeaponKind::Rl => AmmoKind::Rockets,
        WeaponKind::Lg => AmmoKind::Cells,
    }
}

/// A bot's combat-relevant state, snapshotted once per goal evaluation.
struct Stats {
    health: f32,
    armor_value: f32,
    armor_type: f32,
    /// `v.items` bitfield (carried as `f32`, the engine type; tested via the [`Bits`] trait).
    items: f32,
    shells: f32,
    nails: f32,
    rockets: f32,
    cells: f32,
    /// Current effective HP (`TotalStrength`).
    strength: f32,
    /// Current armor absorb (`armortype × armorvalue`), gating armor desire.
    armor: f32,
    /// 0–100 firepower proxy: how well the bot can already deal damage (gates weapon/ammo desire).
    firepower: f32,
    /// "Weapons stay" mode (deathmatch 2/3/5): a picked-up weapon's entity lingers and re-touching
    /// it does nothing, so an owned weapon is worthless. Otherwise re-grabbing one refills its ammo.
    weapons_stay: bool,
    /// Resource discipline on (`rtx_bot_stack`): scale health/armour desire up below the bare-spawn
    /// stack and panic for a dry primary. An enemy's estimated `Stats` carries the *bot's own* flag so
    /// its denial desire scales the same way ("he's weak and needs that armour" is worth more).
    stack: bool,
}

/// A 0–100 estimate of the bot's offensive capability — its best weapon fed by its ammo. Higher
/// firepower means lower desire for more weapons/ammo (ktx's `firepower_`).
fn firepower(items: f32, shells: f32, nails: f32, rockets: f32, cells: f32) -> f32 {
    let has = |b: Items| items.has(b);
    let mut fp = 0.0_f32;
    if has(Items::ROCKET_LAUNCHER) {
        fp = fp.max((rockets * 8.0).min(100.0));
    }
    if has(Items::LIGHTNING) {
        fp = fp.max((cells * 5.0).min(100.0));
    }
    if has(Items::GRENADE_LAUNCHER) {
        fp = fp.max((rockets * 6.0).min(50.0));
    }
    if has(Items::SUPER_NAILGUN) {
        fp = fp.max((nails * 0.5).min(50.0));
    }
    if has(Items::NAILGUN) {
        fp = fp.max((nails * 0.25).min(35.0));
    }
    if has(Items::SUPER_SHOTGUN) {
        fp = fp.max(shells.min(20.0));
    }
    fp.max(shells.min(10.0)).min(100.0) // shotgun/axe baseline
}

/// Convert an observation-gated opponent estimate into the same scoring currency as the bot. Ammo
/// is not observable after a pickup, so assume a modest fed loadout for weapons known to be owned;
/// this is used only for relative risk and denial, never aiming or exact damage prediction.
fn estimated_stats(est: crate::bot::model::OpponentEstimate, weapons_stay: bool, stack: bool) -> Stats {
    let has = |bit: Items| est.items.has(bit);
    let shells = if has(Items::SUPER_SHOTGUN) { 20.0 } else { 10.0 };
    let nails = if has(Items::NAILGUN) || has(Items::SUPER_NAILGUN) {
        50.0
    } else {
        0.0
    };
    let rockets = if has(Items::GRENADE_LAUNCHER) || has(Items::ROCKET_LAUNCHER) {
        10.0
    } else {
        0.0
    };
    let cells = if has(Items::LIGHTNING) { 30.0 } else { 0.0 };
    let strength = total_strength(est.health, est.armor_value, est.armor_type);
    Stats {
        health: est.health,
        armor_value: est.armor_value,
        armor_type: est.armor_type,
        items: est.items,
        shells,
        nails,
        rockets,
        cells,
        strength,
        armor: est.armor_type * est.armor_value,
        firepower: firepower(est.items, shells, nails, rockets, cells),
        weapons_stay,
        stack,
    }
}

/// Denial/contest adjustment in pure form for tests. A known enemy's need makes the item more useful
/// to take away; if that enemy reaches an ordinary item first and the bot is weaker, the route is
/// discounted rather than feeding a losing fight. Powerups remain contest-worthy.
fn contest_adjust(
    own_desire: f32,
    enemy_desire: f32,
    my_eta: f32,
    enemy_eta: Option<f32>,
    weaker: bool,
    powerup: bool,
) -> (f32, f32) {
    let desire = own_desire.max(0.0) + ENEMY_DENIAL_WEIGHT * enemy_desire.max(0.0);
    let lost = enemy_eta.is_some_and(|eta| eta + 0.25 < my_eta);
    let mult = if lost && weaker && !powerup {
        LOST_CONTEST_MULT
    } else {
        1.0
    };
    (desire, mult)
}

/// Whether disengaging to heal is survivable: safe when the enemy can't currently see the bot (no
/// mutual line of sight → a free break for cover), or when far enough to reach the pickup/cover before
/// being punished. In the enemy's face it returns false — running only turns the back into the shot,
/// so the bot is better off fighting on with combat's facing backpedal.
fn disengage_safe(has_los: bool, enemy_dist: f32) -> bool {
    !has_los || enemy_dist >= DISENGAGE_SAFE_RANGE
}

fn posture_step(
    previous: super::state::CombatPosture,
    health: f32,
    strength: f32,
    own_power: f32,
    enemy_power: Option<f32>,
    stack: bool,
) -> super::state::CombatPosture {
    use super::state::CombatPosture::*;
    let ratio = enemy_power.filter(|&p| p > 0.0).map_or(1.0, |p| own_power / p);
    if previous == Recover {
        // Exit only when back on our feet: a health cushion *and* (stack-aware) a rebuilt stack, plus
        // a fair power ratio. Each threshold clears its entry point so one pickup doesn't oscillate us.
        let recovered = if stack {
            health >= RECOVER_EXIT_HEALTH && strength >= RECOVER_EXIT_STRENGTH && ratio >= 0.85
        } else {
            health >= 60.0 && ratio >= 0.85
        };
        if !recovered {
            return Recover;
        }
    } else {
        // Enter Recover on a health that's one rocket from death, a thin whole stack (stack-aware), or
        // a clearly losing power ratio. Without discipline it's the leaner health-only entry.
        let critical = if stack {
            health <= RECOVER_ENTER_HEALTH || strength <= RECOVER_ENTER_STRENGTH || ratio < 0.6
        } else {
            health <= 40.0 || ratio < 0.6
        };
        if critical {
            return Recover;
        }
    }
    if ratio > 1.35 {
        Press
    } else {
        Hold
    }
}

/// Desire for a weapon the bot doesn't yet own (ktx's firepower-gap weighting).
fn weapon_desire(s: &Stats, w: WeaponKind) -> f32 {
    let fp = s.firepower;
    let desire_rockets = (20.0 - s.rockets).max(5.0);
    let desire_cells = ((50.0 - s.cells) * 0.2).max(2.5);
    match w {
        WeaponKind::Rl => (100.0 - fp).max(desire_rockets),
        WeaponKind::Lg => (100.0 - fp).max(desire_rockets).max(desire_cells),
        WeaponKind::Gl => (if fp < 50.0 { 50.0 - fp } else { 0.0 }).max(desire_rockets),
        WeaponKind::Ng | WeaponKind::Sng => {
            let nails = if fp < 20.0 { 2.5 - s.nails * 0.0125 } else { 0.0 };
            (20.0 - fp).max(0.0).max(nails)
        }
        WeaponKind::Ssg => {
            let shells = if fp < 20.0 && s.shells < 50.0 {
                2.5 - s.shells * 0.05
            } else {
                0.0
            };
            (20.0 - fp).max(0.0).max(shells)
        }
    }
}

/// The dry-primary ammo urgency at a given firepower: `0` once the bot is adequately armed
/// ([`AMMO_DRY_FIREPOWER`] or above), else a desire that rises as the primary empties out — high
/// enough to clear the combat-detour bar, because going dry mid-fight is a stack loss.
fn dry_urgency(firepower: f32) -> f32 {
    if firepower >= AMMO_DRY_FIREPOWER {
        0.0
    } else {
        AMMO_DRY_BASE + (AMMO_DRY_FIREPOWER - firepower) * AMMO_DRY_SLOPE
    }
}

/// Desire for an ammo pickup — scales with how empty the bot is, zero once it's near the cap or
/// already well-armed (for the secondary ammo types). Under stack discipline the ammo of a *dry
/// primary* the bot actually owns spikes via [`dry_urgency`]: the firepower gate self-solves the
/// multi-gun case (a fed LG makes rockets non-urgent), so it keys on overall firepower, not per-gun.
fn ammo_desire(s: &Stats, a: AmmoKind) -> f32 {
    let fp = s.firepower;
    let dry = |owns: bool| if s.stack && owns { dry_urgency(fp) } else { 0.0 };
    match a {
        AmmoKind::Rockets => {
            if s.rockets < 100.0 {
                let owns = s.items.has(Items::ROCKET_LAUNCHER) || s.items.has(Items::GRENADE_LAUNCHER);
                (20.0 - s.rockets).max(5.0).max(dry(owns))
            } else {
                0.0
            }
        }
        AmmoKind::Cells => {
            if s.cells < 100.0 {
                ((50.0 - s.cells) * 0.2)
                    .max(2.5)
                    .max(dry(s.items.has(Items::LIGHTNING)))
            } else {
                0.0
            }
        }
        AmmoKind::Nails => {
            if fp < 20.0 && s.nails < 200.0 {
                (2.5 - s.nails * 0.0125).max(0.0)
            } else {
                0.0
            }
        }
        AmmoKind::Shells => {
            if fp < 20.0 && s.shells < 100.0 {
                (2.5 - s.shells * 0.05).max(0.0)
            } else {
                0.0
            }
        }
    }
}

impl GameState {
    /// Snapshot a bot's combat state for desire scoring.
    fn bot_stats(&self, e: EntId) -> Stats {
        let v = &self.entities[e].v;
        Stats {
            health: v.health,
            armor_value: v.armorvalue,
            armor_type: v.armortype,
            items: v.items,
            shells: v.ammo_shells,
            nails: v.ammo_nails,
            rockets: v.ammo_rockets,
            cells: v.ammo_cells,
            strength: total_strength(v.health, v.armorvalue, v.armortype),
            armor: v.armortype * v.armorvalue,
            firepower: firepower(v.items, v.ammo_shells, v.ammo_nails, v.ammo_rockets, v.ammo_cells),
            weapons_stay: matches!(self.level.deathmatch, 2 | 3 | 5),
            stack: self.host().cvar_bool(c"rtx_bot_stack"),
        }
    }

    /// Which planning horizon `item` gets. Quad/pent/ring are always powerups; red armour and mega are
    /// "major" (worth a trek and a cycle) but only once resource discipline is on — off, they score as
    /// the ordinary armour/health they were, preserving ktx-parity valuation. Everything else ordinary.
    fn horizon_of(&self, item: EntId, cat: Category, stack: bool) -> Horizon {
        match cat {
            Category::Powerup => Horizon::Powerup,
            Category::Health if stack && self.entities[item].item.healtype == 2.0 => Horizon::Major,
            Category::Armor { gate, .. } if stack && gate >= 160.0 => Horizon::Major,
            _ => Horizon::Ordinary,
        }
    }

    /// How much this bot wants `item` right now (0 = don't bother).
    fn item_desire(&self, s: &Stats, item: EntId, cat: Category) -> f32 {
        match cat {
            // Quad/pent/ring dominate, scaled by the bot's current worth (ktx: `200 + strength`).
            Category::Powerup => 200.0 + s.strength,
            Category::Health => {
                let (amount, mega) = {
                    let it = &self.entities[item].item;
                    (it.healamount, it.healtype == 2.0)
                };
                let raw = if mega {
                    if s.health < 250.0 {
                        let new = (s.health + amount).min(250.0);
                        (total_strength(new, s.armor_value, s.armor_type) - s.strength).max(0.0)
                    } else {
                        0.0
                    }
                } else if s.health < 100.0 {
                    let new = (s.health + amount).min(100.0);
                    (2.0 * (total_strength(new, s.armor_value, s.armor_type) - s.strength)).max(0.0)
                } else {
                    0.0
                };
                // Below the bare-spawn stack, value the top-up more (>50/>50 discipline). Zero stays
                // zero and the caps still zero out — only a genuine gain is scaled.
                raw * stack_mult(s)
            }
            Category::Armor {
                value,
                at,
                gate,
                double,
            } => {
                let raw = if s.armor < gate {
                    let gain = (total_strength(s.health, value, at) - s.strength).max(0.0);
                    if double {
                        gain * 2.0
                    } else {
                        gain
                    }
                } else {
                    0.0
                };
                raw * stack_mult(s)
            }
            Category::Weapon(w) => {
                if !s.items.has(weapon_bit(w)) {
                    weapon_desire(s, w) // don't own it — want the weapon itself
                } else if s.weapons_stay {
                    0.0 // own it and it stays put — re-touching does nothing
                } else {
                    ammo_desire(s, weapon_ammo(w)) // own it but it respawns — re-grab for ammo
                }
            }
            Category::Ammo(a) => ammo_desire(s, a),
        }
    }

    /// How much this bot wants a dropped **backpack** (from a death drop or a teammate's toss).
    /// A backpack carries a single weapon bit plus a slice of ammo, so its worth is the best of the
    /// weapon it grants (if the bot lacks it) and the ammo it refills — the same currency the
    /// catalog items use, so a backpack competes on equal footing. Unlike map items a backpack is
    /// dynamic (spawned on death, auto-removed after 120 s), so it never lives in the static catalog.
    fn backpack_desire(&self, s: &Stats, item: EntId) -> f32 {
        let v = &self.entities[item].v;
        let mut desire = 0.0_f32;
        if let Some(w) = weapon_kind_of(v.items) {
            if !s.items.has(weapon_bit(w)) {
                desire = desire.max(weapon_desire(s, w));
            }
        }
        for (amount, kind) in [
            (v.ammo_shells, AmmoKind::Shells),
            (v.ammo_nails, AmmoKind::Nails),
            (v.ammo_rockets, AmmoKind::Rockets),
            (v.ammo_cells, AmmoKind::Cells),
        ] {
            if amount > 0.0 {
                desire = desire.max(ammo_desire(s, kind));
            }
        }
        desire
    }

    /// The effective time-to-collect for `item` given the raw travel cost: `None` if it's hidden
    /// with no scheduled respawn (uncollectable), else `max(travel, respawn-wait)`.
    fn item_collect_time(&self, item: EntId, travel: f32, now: f32) -> Option<f32> {
        let ent = &self.entities[item];
        if ent.v.solid == Solid::Trigger {
            Some(travel) // on the floor now
        } else if matches!(ent.think, Think::SubRegen) && ent.v.nextthink > now {
            Some(travel.max(ent.v.nextthink - now)) // respawning — wait for it
        } else {
            None
        }
    }

    /// Whether a chosen item is still worth heading to: still reachable-soon (available, or
    /// respawning within the window) *and* still desirable to this bot. The desire test matters in
    /// "weapons stay" modes (deathmatch 2/3/5), where a picked-up weapon's entity lingers as a live
    /// trigger forever — without it a bot that just grabbed the weapon would keep homing onto the
    /// now-worthless pickup and circle it until the throttled re-select kicked in.
    pub(crate) fn item_goal_valid(&self, bot_e: EntId, item: EntId, now: f32) -> bool {
        // A weapon held for a powerup carrier (handoff) is a valid goal while it's still on the floor,
        // even though the holder may already own it (so its ordinary item_desire is 0).
        if item.0 != 0 && item.0 == self.entities[bot_e].bot.goal.hold_item {
            return self.entities[item].v.solid == Solid::Trigger;
        }
        let ent = &self.entities[item];
        // A backpack has no goal classname and never respawns: it's valid while it's still on the
        // floor (solid) and still carries something this bot wants.
        if ent.touch == Touch::Backpack {
            return ent.v.solid == Solid::Trigger && self.backpack_desire(&self.bot_stats(bot_e), item) > 0.0;
        }
        if ent.touch == Touch::Rune {
            return ent.v.solid == Solid::Trigger
                && rune_desire(self.entities[bot_e].mode_p.ctf.runes, ent.item.rune_bit) > 0.0;
        }
        let Some(cat) = ent.classname().and_then(category) else {
            return false;
        };
        let s = self.bot_stats(bot_e);
        let tier = self.horizon_of(item, cat, s.stack);
        // Revalidate deterministic powerup ownership as teammates move. Selection alone is not
        // enough: a farther bot can publish the goal first, then a nearer teammate legitimately
        // takes ownership on its later frame. Without this check the first bot's Powerup commit
        // prevents re-selection and both remain locked onto Quad until somebody touches it.
        let my_team = self.entities[bot_e].mode_p.team;
        if matches!(cat, Category::Powerup) && my_team != 0 {
            let point = ent.v.origin;
            let my_dist = (point - self.entities[bot_e].v.origin).length();
            if defer_powerup_to_teammate(
                self.item_claimed_by_teammate(bot_e, my_team, item.0),
                my_dist,
                self.nearest_teammate_dist(bot_e, my_team, point),
            ) {
                return false;
            }
        }
        // RA/mega use a need-and-distance owner rather than a soft claim discount. A stacked bot
        // that selected RA first must release it when a newly hurt teammate becomes the better use
        // of the pickup; otherwise both keep the same live goal until one touches it.
        if tier == Horizon::Major && my_team != 0 && self.major_item_owned_by_teammate(bot_e, my_team, item, cat, now) {
            return false;
        }
        // A goal respawning within its own tier's horizon is still a valid standing goal (arrive early
        // and wait): a powerup out to 30 s, a major (RA/mega) to 20, an ordinary item the tight 10.
        let horizon = match tier {
            Horizon::Powerup => POWERUP_LOOKAHEAD,
            Horizon::Major => MAJOR_LOOKAHEAD,
            Horizon::Ordinary => LOOKAHEAD,
        };
        let reachable_soon =
            ent.v.solid == Solid::Trigger || (matches!(ent.think, Think::SubRegen) && ent.v.nextthink - now < horizon);
        if !reachable_soon {
            return false;
        }
        // Include denial floors — a bot standing on a denial-worthy RL/mega has a *valid* goal even at
        // zero ordinary desire, or the next-frame re-pick (mod.rs) would drop it the instant it's chosen.
        self.desire_with_floors(bot_e, &s, item, cat, self.deny_active(bot_e), now) > 0.0
    }

    /// Pick the highest-scoring reachable item goal for a bot, or `None` to fall back to following
    /// a human. Scans the static per-map catalog *and* any live dropped backpacks against a single
    /// Dijkstra flood from the bot.
    pub(crate) fn select_item_goal(&self, bot_e: EntId) -> Option<ItemPlan> {
        self.best_item_plan(bot_e)
    }

    /// Pick an item worth *breaking off combat* for — the same best goal, but only returned when
    /// its desire clears [`COMBAT_GREED_MIN_DESIRE`]. Lets a fighting bot detour for the quad, a
    /// weapon it lacks, or a big health/armor pickup without chasing every trivial ammo box.
    pub(crate) fn select_combat_item(&self, bot_e: EntId) -> Option<ItemPlan> {
        self.best_item_plan(bot_e)
            .filter(|p| p.first_desire >= COMBAT_GREED_MIN_DESIRE || p.contains_powerup)
    }

    /// Major pickup planning while optional combat greed is disabled. A timed powerup or CTF rune is a
    /// match objective, not a personality detour; red armour/mega count too once they're genuinely
    /// wanted (`contains_major`) — holding them is map control, worth breaking a fight for. An ordinary
    /// first leg is allowed only when the bounded plan proves it bridges to one of those.
    pub(crate) fn select_major_item(&self, bot_e: EntId) -> Option<ItemPlan> {
        self.best_item_plan(bot_e)
            .filter(|p| p.contains_powerup || p.contains_major)
    }

    /// Pick a spawned, nearby health/armor recovery or timed powerup that must be completed before
    /// combat movement resumes. This pass is independent of `rtx_bot_greed`: greed governs optional
    /// detours, not stepping the final metre onto armor that prevents an immediate death.
    pub(crate) fn select_urgent_local_item(&self, bot_e: EntId) -> Option<(EntId, CellId, super::state::GoalCommit)> {
        let graph = self.nav.graph.as_ref()?;
        let origin = self.entities[bot_e].v.origin;
        let stats = self.bot_stats(bot_e);
        let now = self.time();

        // First establish that a relevant pickup is geometrically local. Most frames stop here and
        // avoid the graph flood; the exact travel-time/reachability test follows only when needed.
        let mut nearby = Vec::new();
        for &(idx, cell) in &self.nav.goals {
            let item = EntId(idx);
            let ent = &self.entities[item];
            if ent.v.solid != Solid::Trigger
                || self.entities[bot_e].bot.is_avoided(idx, now)
                || (ent.v.origin - origin).length() > LOCAL_PICKUP_RADIUS
            {
                continue;
            }
            let Some(cat) = ent.classname().and_then(category) else {
                continue;
            };
            let desire = self.item_desire(&stats, item, cat);
            let commit = match cat {
                Category::Powerup => super::state::GoalCommit::Powerup,
                Category::Health | Category::Armor { .. }
                    if desire >= LOCAL_RECOVERY_GAIN || (stats.health <= 40.0 && desire > 0.0) =>
                {
                    super::state::GoalCommit::Pickup
                }
                _ => continue,
            };
            nearby.push((item, cell, desire, commit));
        }
        if nearby.is_empty() {
            return None;
        }

        let from = graph.nearest(origin)?;
        let pricing = self.bot_link_pricing(bot_e, now);
        // Every candidate is rejected past `LOCAL_PICKUP_TRAVEL`, so a bounded flood to exactly that
        // horizon is identical to the whole-graph one here — and stops at the local neighbourhood.
        let (travel, _) = graph.costs_from_within(from, &pricing.costs(0), LOCAL_PICKUP_TRAVEL);
        nearby
            .into_iter()
            .filter_map(|(item, cell, desire, commit)| {
                let t = travel[cell as usize];
                (t.is_finite() && t <= LOCAL_PICKUP_TRAVEL).then_some((item, cell, commit, desire / (t + 0.25)))
            })
            .max_by(|a, b| a.3.total_cmp(&b.3).then_with(|| b.0 .0.cmp(&a.0 .0)))
            .map(|(item, cell, commit, _)| (item, cell, commit))
    }

    /// Pick a desirable, up item near the bot that's worth a small side-step onto while it travels —
    /// waypoint magnetism (`rtx_bot_magnet`). This is *not* a goal: the bot never changes course to
    /// fetch it; `steer` only bends the immediate waypoint through it when it actually lies on the
    /// route corridor (see [`super::steer::magnet_on_corridor`]). So the bar is only "would touching
    /// it help, is it near, on my floor" — the corridor test in `steer` does the rest.
    ///
    /// Excludes the item the bot is already chasing (steering leads there anyway) and any weapon held
    /// for a teammate (a handoff must not be stepped on — the server would grant the touch).
    pub(crate) fn select_route_magnet(&self, bot_e: EntId) -> Option<EntId> {
        let origin = self.entities[bot_e].v.origin;
        let now = self.time();
        let s = self.bot_stats(bot_e);
        let goal = self.entities[bot_e].bot.goal.item;
        let hold = self.entities[bot_e].bot.goal.hold_item;
        let deny = self.deny_active(bot_e);
        let mut candidates = Vec::new();
        for &(idx, _cell) in &self.nav.goals {
            if idx == goal || idx == hold || self.entities[bot_e].bot.is_avoided(idx, now) {
                continue;
            }
            let ent = &self.entities[EntId(idx)];
            if ent.v.solid != Solid::Trigger {
                continue;
            }
            let d = ent.v.origin - origin;
            if d.z.abs() > MAGNET_DZ || d.xy().length() > MAGNET_RADIUS {
                continue;
            }
            let Some(cat) = ent.classname().and_then(category) else {
                continue;
            };
            // Include denial floors, so a stacked bot still bends onto an enemy's RL/mega in passing.
            let desire = self.desire_with_floors(bot_e, &s, EntId(idx), cat, deny, now);
            if desire > 0.0 {
                candidates.push((idx, desire, d.xy().length()));
            }
        }
        best_magnet(&candidates).map(EntId)
    }

    /// Find a useful ordinary pickup that can be collected without arriving late for a powerup the
    /// bot is already timing. A powerup commitment deliberately blocks normal re-scoring, but that
    /// must not make a bot walk past a nearby RL/armor either on the opening run to a live Quad or
    /// while heading toward a timed respawn. Only a spawned nearby health/armor/weapon qualifies, and
    /// only when the complete `bot -> pickup -> powerup` route reaches the powerup no later than the
    /// direct route (plus [`POWERUP_BRIDGE_SLACK`]). Respawn slack therefore becomes useful preparation
    /// time, while a live powerup permits only an effectively on-route pickup.
    pub(crate) fn select_powerup_bridge_item(
        &self,
        bot_e: EntId,
        powerup: EntId,
        powerup_cell: CellId,
        now: f32,
    ) -> Option<(EntId, CellId)> {
        let graph = self.nav.graph.as_ref()?;
        let power = &self.entities[powerup];
        let respawn_wait = if power.v.solid == Solid::Trigger {
            0.0
        } else if matches!(power.think, Think::SubRegen) && power.v.nextthink > now {
            power.v.nextthink - now
        } else {
            return None;
        };

        let origin = self.entities[bot_e].v.origin;
        let from = graph.nearest(origin)?;
        let stats = self.bot_stats(bot_e);
        let pricing = self.bot_link_pricing(bot_e, now);
        let link_costs = pricing.costs(0);
        // Coarse when `rtx_bot_lod` is on (the bridge otherwise runs an exact whole-graph flood — plus
        // one per candidate below — the last such path in goal selection). `direct` reaches the
        // powerup, which can be far, so this genuinely needs the far field, not just a bounded flood.
        let lod = self.host().cvar_bool(c"rtx_bot_lod");
        let from_costs = PlanCosts::single(graph, from, &link_costs, lod);
        let direct = from_costs.cost_to(powerup_cell);
        if !direct.is_finite() {
            return None;
        }
        let mut candidates = Vec::new();
        for &(idx, cell) in &self.nav.goals {
            if idx == powerup.0 || self.entities[bot_e].bot.is_avoided(idx, now) {
                continue;
            }
            let item = EntId(idx);
            let ent = &self.entities[item];
            if ent.v.solid != Solid::Trigger || (ent.v.origin - origin).length() > LOCAL_PICKUP_RADIUS {
                continue;
            }
            let Some(cat) = ent.classname().and_then(category) else {
                continue;
            };
            if !matches!(cat, Category::Health | Category::Armor { .. } | Category::Weapon(_)) {
                continue;
            }
            let desire = self.item_desire(&stats, item, cat);
            let first_leg = from_costs.cost_to(cell);
            if desire <= 0.0 || !first_leg.is_finite() || first_leg > POWERUP_BRIDGE_TRAVEL {
                continue;
            }
            candidates.push((item, cell, desire, first_leg));
        }

        // The winner is the highest `desire / (first_leg + 0.25)` that also arrives in time. That score
        // is independent of the second leg (which only *gates*, via `arrives_in_time`), so rank by it
        // up front and pay the expensive per-candidate second-leg flood — a whole abstract-graph
        // Dijkstra from each candidate — only until the first survivor: that highest-ranked one is the
        // answer. Cap the floods at `PLAN_PRIMARY_LIMIT` so a dense pickup cluster (a pile of items
        // round the quad) can't turn one goal pick into a dozen floods (a squad-sync objective spike).
        candidates.sort_by(|a, b| {
            let (sa, sb) = (a.2 / (a.3 + 0.25), b.2 / (b.3 + 0.25));
            sb.total_cmp(&sa).then_with(|| a.0 .0.cmp(&b.0 .0))
        });
        candidates
            .into_iter()
            .take(PLAN_PRIMARY_LIMIT)
            .find_map(|(item, cell, _desire, first_leg)| {
                let second_leg = PlanCosts::single(graph, cell, &link_costs, lod).cost_to(powerup_cell);
                (second_leg.is_finite() && powerup_bridge_arrives_in_time(direct, first_leg + second_leg, respawn_wait))
                    .then_some((item, cell))
            })
    }

    /// Strategic fight posture plus its best recovery target. Relative power uses only the shared
    /// opponent estimate; with modeling disabled/unavailable, critical health still triggers but no
    /// hidden enemy stack is read. Recovery is cancelled when no useful reachable pickup exists.
    pub(crate) fn recovery_decision(
        &self,
        bot_e: EntId,
        enemy: EntId,
        now: f32,
        previous: super::state::CombatPosture,
    ) -> (super::state::CombatPosture, Option<(EntId, CellId)>) {
        use super::state::CombatPosture::*;
        let s = self.bot_stats(bot_e);
        let own_power = s.strength * s.firepower.max(10.0);
        let enemy_power = self.opponent_est(bot_e, enemy, now).map(|est| {
            let es = estimated_stats(est, s.weapons_stay, s.stack);
            es.strength * es.firepower.max(10.0)
        });
        let posture = posture_step(previous, s.health, s.strength, own_power, enemy_power, s.stack);
        if posture != Recover {
            return (posture, None);
        }
        let Some(item) = self.select_recovery_item(bot_e, &s, now) else {
            return (Hold, None);
        };
        // Escape gate (stack discipline): committing to this pickup hands the feet to navigation, which
        // may turn the bot's back on the enemy. Only do so when a disengage is survivable — otherwise
        // stay in Recover with no goal so combat_move fights on with its facing backpedal, and break the
        // instant a gap opens. Without discipline, keep the old unconditional break-off.
        if s.stack {
            let has_los = self.entities[bot_e].bot.percept.vis_since != 0.0;
            let enemy_dist = (self.entities[enemy].v.origin - self.entities[bot_e].v.origin).length();
            if !disengage_safe(has_los, enemy_dist) {
                return (Recover, None);
            }
        }
        (Recover, Some(item))
    }

    fn select_recovery_item(&self, bot_e: EntId, s: &Stats, now: f32) -> Option<(EntId, CellId)> {
        let graph = self.nav.graph.as_ref()?;
        let from = graph.nearest(self.entities[bot_e].v.origin)?;
        let pricing = self.bot_link_pricing(bot_e, now);
        // Recovery items past `RECOVERY_TRAVEL` are dropped below, so bound the flood there — exact for
        // every cell it keeps, and it never floods the far half of a big map for a hurt bot.
        let (costs, _) = graph.costs_from_within(from, &pricing.costs(0), RECOVERY_TRAVEL);
        let my_team = self.entities[bot_e].mode_p.team;
        let teamwork = my_team != 0;
        self.nav
            .goals
            .iter()
            .filter_map(|&(idx, cell)| {
                let item = EntId(idx);
                let ent = &self.entities[item];
                if ent.v.solid != Solid::Trigger || self.entities[bot_e].bot.is_avoided(idx, now) {
                    return None;
                }
                let cat = ent.classname().and_then(category)?;
                let desire = self.item_desire(s, item, cat);
                let useful = match cat {
                    Category::Health | Category::Armor { .. } => desire > 0.0,
                    Category::Weapon(_) => desire >= COMBAT_GREED_MIN_DESIRE,
                    _ => false,
                };
                if !useful {
                    return None;
                }
                if teamwork
                    && self.horizon_of(item, cat, s.stack) == Horizon::Major
                    && self.major_item_owned_by_teammate(bot_e, my_team, item, cat, now)
                {
                    return None;
                }
                let t = costs[cell as usize];
                if !t.is_finite() || t > RECOVERY_TRAVEL {
                    return None;
                }
                let claim = if teamwork && self.item_claimed_by_teammate(bot_e, my_team, idx) {
                    CLAIM_DISCOUNT
                } else {
                    1.0
                };
                Some((item, cell, desire * claim / (t + 0.5)))
            })
            .max_by(|a, b| a.2.total_cmp(&b.2).then_with(|| b.0 .0.cmp(&a.0 .0)))
            .map(|(item, cell, _)| (item, cell))
    }

    /// The best `(item, cell, desire)` for a bot by `desire × (LOOKAHEAD − t) / (t + 5)`, over both
    /// the static catalog and live backpacks. `desire` is returned so callers can apply their own
    /// bar (combat-detour vs. idle pickup). Backpacks are on the floor now, so their `t` is pure
    /// travel; the catalog folds in respawn-wait via [`item_collect_time`].
    /// Whether a living teammate bot has the stable reservation for item `idx`. Every current
    /// claimant plus this bot is ordered by straight-line distance and then edict id, so update order
    /// cannot make two bots repeatedly yield the pickup back and forth.
    fn item_claimed_by_teammate(&self, bot_e: EntId, my_team: u8, idx: u32) -> bool {
        if idx == 0 {
            return false;
        }
        let point = self.entities[EntId(idx)].v.origin;
        let maxclients = self.host().cvar(c"maxclients") as u32;
        let mut candidates = vec![(bot_e, (self.entities[bot_e].v.origin - point).length_squared())];
        candidates.extend((1..=maxclients).map(EntId).filter_map(|t| {
            (t != bot_e && {
                let e = &self.entities[t];
                e.bot.is_bot && e.v.health > 0.0 && e.mode_p.team == my_team && e.bot.goal.item == idx
            })
            .then_some((t, (self.entities[t].v.origin - point).length_squared()))
        }));
        reservation_owner(&candidates).is_some_and(|owner| owner != bot_e)
    }

    /// Whether another available team bot is the better RA/mega owner. Ownership balances actual
    /// marginal value against straight-line travel: a critically weak bot can take over from a
    /// nearby stacked denial bot, while a tiny need advantage does not pull somebody across the map.
    /// A teammate committed to a different powerup is excluded so reserving Quad cannot leave RA
    /// ownerless. The edict-id tie break makes every sequential bot frame reach the same answer.
    fn major_item_owned_by_teammate(&self, bot_e: EntId, my_team: u8, item: EntId, cat: Category, now: f32) -> bool {
        let point = self.entities[item].v.origin;
        let maxclients = self.host().cvar(c"maxclients") as u32;
        let candidates: Vec<(EntId, f32)> = (1..=maxclients)
            .map(EntId)
            .filter_map(|t| {
                let e = &self.entities[t];
                let available =
                    t == bot_e || e.bot.goal.commit != super::state::GoalCommit::Powerup || e.bot.goal.item == item.0;
                (e.bot.is_bot && e.v.health > 0.0 && e.mode_p.team == my_team && available).then(|| {
                    let stats = self.bot_stats(t);
                    let desire = self.desire_with_floors(t, &stats, item, cat, self.deny_active(t), now);
                    let distance = (e.v.origin - point).length();
                    (t, strategic_owner_score(desire, distance))
                })
            })
            .collect();
        strategic_reservation_owner(&candidates).is_some_and(|owner| owner != bot_e)
    }

    /// Whether any living enemy player is believed — in this bot's shared opponent-model pool — to
    /// hold weapon `bit`. The signal [`denial_floor`] reads: an unheld big weapon on the enemy side
    /// is worth denying. `false` when opponent modeling is off (`opponent_est` yields nothing), so the
    /// caller must gate denial on the cvar itself rather than treat "no belief" as "enemy lacks it".
    fn enemy_side_has_weapon(&self, bot_e: EntId, my_team: u8, bit: Items, now: f32) -> bool {
        let maxclients = self.host().cvar(c"maxclients") as u32;
        (1..=maxclients).map(EntId).any(|t| {
            let e = &self.entities[t];
            e.is_player()
                && e.v.health > 0.0
                && enemy_of(my_team, e.mode_p.team)
                && self.opponent_est(bot_e, t, now).is_some_and(|est| est.items.has(bit))
        })
    }

    /// Whether any living enemy player exists (FFA: anyone else; team: an opposing player). Denial is
    /// pointless with the field cleared, so the deny gate requires it — and it's what lets FFA denial
    /// mean "keep it from everyone" rather than the vacuous "keep it from my (empty) team".
    fn any_living_enemy(&self, bot_e: EntId) -> bool {
        let my_team = self.entities[bot_e].mode_p.team;
        let maxclients = self.host().cvar(c"maxclients") as u32;
        (1..=maxclients).map(EntId).any(|t| {
            t != bot_e && {
                let e = &self.entities[t];
                e.is_player() && e.v.health > 0.0 && enemy_of(my_team, e.mode_p.team)
            }
        })
    }

    /// Whether item denial is active for this bot: opponent modeling on and a living enemy to deny.
    /// Both `best_item_plan` (once per frame) and the one-shot callers price denial through this so
    /// selection and validation can't disagree on whether a floor applies.
    fn deny_active(&self, bot_e: EntId) -> bool {
        self.host().cvar_bool(c"rtx_bot_model") && self.any_living_enemy(bot_e)
    }

    /// This bot's desire for `item`, raised to a denial floor where one applies: a big weapon (RL/LG)
    /// the enemy side lacks, or — because taking mega at 200 hp is correct QW play — a megahealth or red
    /// armour, regardless of the bot's own stack. `deny` is the [`deny_active`] gate, passed in so the
    /// per-frame catalog scan computes it once. Selection *and* [`item_goal_valid`] must both price
    /// through here, or a denial goal would be invalidated the frame after it's chosen (mod.rs re-pick).
    fn desire_with_floors(&self, bot_e: EntId, s: &Stats, item: EntId, cat: Category, deny: bool, now: f32) -> f32 {
        let mut desire = self.item_desire(s, item, cat);
        if !deny {
            return desire;
        }
        let my_team = self.entities[bot_e].mode_p.team;
        match cat {
            Category::Weapon(w) => {
                let enemy_has = self.enemy_side_has_weapon(bot_e, my_team, weapon_bit(w), now);
                desire = desire.max(denial_floor(w, s.weapons_stay, enemy_has));
            }
            // Mega/RA detected independent of the stack cvar (`true`): denial is a modeling feature, not
            // resource discipline, so it holds even when the leaner valuation is selected.
            Category::Health | Category::Armor { .. } if self.horizon_of(item, cat, true) == Horizon::Major => {
                desire = desire.max(DENIAL_DESIRE);
            }
            _ => {}
        }
        desire
    }

    /// Maintain or begin a **handoff hold**: an idle bot may reserve a spawned RL/LG for a
    /// powerup-carrying teammate that lacks it — standing on the weapon without taking it until the
    /// carrier arrives, or taking it itself when the reservation lapses (denial beats a no-show).
    /// Returns whether the bot is holding this frame (its `goal_item` then points at the weapon and
    /// `bot_pickup_items` suppresses the grab). Team modes only, gated by `rtx_bot_model` +
    /// `rtx_bot_model`; a non-idle bot (one with a fight or a move objective) never holds.
    pub(crate) fn update_handoff_hold(&mut self, e: EntId, now: f32, idle: bool) -> bool {
        let enabled = idle && self.entities[e].mode_p.team != 0 && self.host().cvar_bool(c"rtx_bot_model");
        if !enabled {
            self.clear_hold(e);
            return false;
        }
        let held = self.entities[e].bot.goal.hold_item;
        if held != 0 {
            if self.hold_should_continue(e, EntId(held), now) {
                self.pin_hold_goal(e, EntId(held));
                return true;
            }
            // Abort (deadline, carrier gone/expired/armed, item taken, or an enemy contest): drop the
            // reservation. If the item's still on the floor and the bot is on it, the next
            // `bot_pickup_items` grabs it (denial); if it's gone, normal selection re-picks.
            self.clear_hold(e);
            return false;
        }
        if let Some((item, mate)) = self.handoff_hold_target(e, now) {
            {
                let b = &mut self.entities[e].bot;
                b.goal.hold_item = item.0;
                b.goal.hold_for = mate.0;
                b.goal.hold_until = now + HOLD_MAX;
                b.goal.since = now;
            }
            self.pin_hold_goal(e, item);
            return true;
        }
        false
    }

    /// Point the bot's movement goal at the held weapon (navigation carries it there and keeps it
    /// standing on the spot).
    fn pin_hold_goal(&mut self, e: EntId, item: EntId) {
        let cell = self
            .nav
            .graph
            .as_ref()
            .and_then(|g| g.nearest(self.entities[item].v.origin));
        let b = &mut self.entities[e].bot;
        b.goal.item = item.0;
        if let Some(c) = cell {
            b.goal.item_cell = c;
        }
    }

    fn clear_hold(&mut self, e: EntId) {
        let b = &mut self.entities[e].bot;
        if b.goal.hold_item != 0 {
            (b.goal.hold_item, b.goal.hold_for, b.goal.hold_until) = (0, 0, 0.0);
        }
    }

    /// Whether an active hold should keep running: gathers the live facts and defers to the pure
    /// [`hold_continues`]. Any failed condition ends the hold (see [`update_handoff_hold`]).
    fn hold_should_continue(&self, e: EntId, item: EntId, now: f32) -> bool {
        let it = &self.entities[item];
        let item_on_floor = it.v.solid == Solid::Trigger;
        let Some(Category::Weapon(w)) = it.classname().and_then(category) else {
            return false;
        };
        let mate = EntId(self.entities[e].bot.goal.hold_for);
        let m = &self.entities[mate];
        let mate_alive = mate.0 != 0 && m.is_player() && m.v.health > 0.0;
        let mate_powered = m.combat.super_damage_finished > now || m.combat.invincible_finished > now;
        let mate_has_weapon = m.v.items.has(weapon_bit(w));
        // Contest: a perceived, living enemy near the weapon means "take it rather than leave it".
        let known = self.entities[e].bot.percept.known_enemy;
        let enemy_contesting = known != 0 && now < self.entities[e].bot.percept.known_until && {
            let ent = &self.entities[EntId(known)];
            ent.v.health > 0.0
                && (ent.v.origin - it.v.origin).length_squared() < HOLD_CONTEST_RANGE * HOLD_CONTEST_RANGE
        };
        hold_continues(
            now,
            self.entities[e].bot.goal.hold_until,
            item_on_floor,
            mate_alive,
            mate_powered,
            mate_has_weapon,
            enemy_contesting,
        )
    }

    /// A handoff opportunity: a living powerup-carrying teammate that lacks RL and/or LG, plus a
    /// spawned copy of a weapon they lack within reach. Own-team arsenals are read directly — that
    /// truthful sharing between teammates *is* the coordination the feature is about.
    fn handoff_hold_target(&self, bot: EntId, now: f32) -> Option<(EntId, EntId)> {
        let my_team = self.entities[bot].mode_p.team;
        let bot_org = self.entities[bot].v.origin;
        let maxclients = self.host().cvar(c"maxclients") as u32;
        let mate = (1..=maxclients).map(EntId).find(|&t| {
            t != bot && {
                let m = &self.entities[t];
                m.is_player()
                    && m.v.health > 0.0
                    && m.mode_p.team == my_team
                    && (m.combat.super_damage_finished > now + HOLD_MATE_MIN_POWER
                        || m.combat.invincible_finished > now + HOLD_MATE_MIN_POWER)
                    && (!m.v.items.has(Items::ROCKET_LAUNCHER) || !m.v.items.has(Items::LIGHTNING))
            }
        })?;
        let mate_items = self.entities[mate].v.items;
        let graph = self.nav.graph.as_ref()?;
        let mut best: Option<(EntId, f32)> = None;
        for &(idx, _) in &self.nav.goals {
            let item = EntId(idx);
            let it = &self.entities[item];
            if it.v.solid != Solid::Trigger {
                continue;
            }
            let Some(Category::Weapon(w)) = it.classname().and_then(category) else {
                continue;
            };
            if !matches!(w, WeaponKind::Rl | WeaponKind::Lg) || mate_items.has(weapon_bit(w)) {
                continue;
            }
            if self.item_claimed_by_teammate(bot, my_team, idx) || graph.nearest(it.v.origin).is_none() {
                continue;
            }
            let d = (it.v.origin - bot_org).length_squared();
            if d <= HOLD_REACH * HOLD_REACH && best.is_none_or(|(_, bd)| d < bd) {
                best = Some((item, d));
            }
        }
        best.map(|(item, _)| (item, mate))
    }

    fn best_item_plan(&self, bot_e: EntId) -> Option<ItemPlan> {
        let graph = self.nav.graph.as_ref()?;
        let bot_cell = graph.nearest(self.entities[bot_e].v.origin)?;
        let now = self.time();
        // Gate-aware, and charged with this bot's failed-link surcharges and rocket-jump fitness gate
        // (jitter off — `0` — so item scoring stays stable): an item behind a shut door, only
        // reachable via a leg it keeps failing, or needing a rocket jump it can't make, floods to a
        // higher `t` and scores lower, so the bot stops re-choosing an item it can never actually
        // reach. Same pricing `run_bot` routes with (see [`LinkPricing`](super::LinkPricing)).
        let pricing = self.bot_link_pricing(bot_e, now);
        let base = pricing.costs(0);
        let s = self.bot_stats(bot_e);
        let own_power = s.strength * s.firepower.max(10.0);
        // Only a currently remembered opponent contributes denial/contest information. Position is
        // the perception hypothesis (exact only when seen), and inventory/stack comes exclusively
        // from the observation-gated model — never from the live enemy edict. The *flood* from their
        // position is deferred into the fan-out below (it's independent of our own base flood); only
        // the serial prep — their cell, their estimated stats, their pricing — happens here.
        let (enemy_prep, enemy_theirs): (Option<(Stats, CellId, bool)>, Option<super::LinkPricing>) = {
            let p = &self.entities[bot_e].bot.percept;
            let enemy = EntId(p.known_enemy);
            if enemy.0 != 0 && now < p.known_until && self.entities[enemy].v.health > 0.0 {
                match self.opponent_est(bot_e, enemy, now).and_then(|est| {
                    let cell = graph.nearest(p.last_seen)?;
                    let stats = estimated_stats(est, s.weapons_stay, s.stack);
                    let power = stats.strength * stats.firepower.max(10.0);
                    // Price the enemy's flood by *their* strength: how far they can get is no business
                    // of our health (hazards are valued by whoever is wading).
                    let theirs = pricing.for_strength(stats.strength);
                    Some(((stats, cell, own_power < 0.6 * power), theirs))
                }) {
                    Some((prep, theirs)) => (Some(prep), Some(theirs)),
                    None => (None, None),
                }
            } else {
                (None, None)
            }
        };
        // Base costs (travel from us) and enemy-contest costs (travel from them). With `rtx_bot_lod`
        // these are coarse LOD estimates — near-exact and cheap; otherwise the exact whole-graph floods,
        // still fanned out across the worker pool (bit-identical to serial). Scoring below reads either
        // through `PlanCosts::cost_to`. Both floods use jitter 0, so item scoring stays stable.
        let lod = self.host().cvar_bool(c"rtx_bot_lod");
        let their_costs = enemy_theirs.as_ref().map(|t| t.costs(0));
        let (base_costs, enemy_context) = if lod {
            let enemy_c = match (enemy_prep, &their_costs) {
                (Some((stats, cell, weaker)), Some(tc)) => {
                    Some((stats, PlanCosts::Coarse(graph.coarse_costs(cell, tc, true)), weaker))
                }
                _ => None,
            };
            (PlanCosts::Coarse(graph.coarse_costs(bot_cell, &base, true)), enemy_c)
        } else {
            let (normal, enemy_flood) = self.bot_pool.join(
                || graph.costs_from(bot_cell, &base),
                || match (&enemy_prep, &their_costs) {
                    (Some((_, cell, _)), Some(tc)) => Some(graph.costs_from(*cell, tc)),
                    _ => None,
                },
            );
            let enemy_c = match (enemy_prep, enemy_flood) {
                (Some((stats, _, weaker)), Some(f)) => Some((stats, PlanCosts::Exact(f), weaker)),
                _ => None,
            };
            (PlanCosts::Exact(normal), enemy_c)
        };
        // Valuation prices a shut door the bot can *open* as the button-detour errand it is, not the
        // full route-around wall — otherwise a prize reachable only through a gate (the ultrav quad
        // behind its teleporter door) costs ~100k and is never a *choosable* goal, so no bot ever
        // heads there to work the button, and it sits untaken until the door happens to open for some
        // other reason. A gate is openable-from-here when its button costs below the route-around
        // penalty in the plain costs (i.e. its button is reachable without crossing a shut gate); a
        // sealed one (button on the far side) keeps the full price. Only re-cost when a gate is shut —
        // most frames there is none, and *path* planning (`run_bot`) still pays the full penalty.
        let openable: Vec<bool> = base
            .gate_closed
            .iter()
            .enumerate()
            .map(|(gi, &shut)| shut && base_costs.cost_to(graph.gate(gi).button_cell) < CLOSED_GATE_PENALTY)
            .collect();
        let mut vcosts = base;
        vcosts.openable_gates = &openable;
        vcosts.open_gate_cost = GATE_OPEN_COST;
        let costs = if openable.iter().any(|&o| o) {
            PlanCosts::single(graph, bot_cell, &vcosts, lod)
        } else {
            base_costs
        };
        // The item we're already chasing, for the hysteresis bonus below.
        let current_goal = self.entities[bot_e].bot.goal.item;
        // Item claims (teamwork): an ordinary item a living teammate bot is already fetching is
        // strongly discounted, so teammates spread across weapons/armor instead of racing the same
        // one. Powerups use the deterministic ownership gate below. Off in FFA (no team).
        let my_team = self.entities[bot_e].mode_p.team;
        let teamwork = my_team != 0;
        // Item denial (opponent modeling): raise the desire to hold a big weapon the enemy side is
        // believed to lack (and mega/RA outright), so bots secure/guard those spawns. On whenever
        // modeling is on and a living enemy exists — FFA included, where everyone else is the enemy
        // side ([`enemy_of`]) — not just team play. Modeling gates it so "no belief" can't masquerade
        // as "enemy lacks it".
        let deny = self.deny_active(bot_e);
        let claim_mult = |idx: u32| {
            if teamwork && self.item_claimed_by_teammate(bot_e, my_team, idx) {
                CLAIM_DISCOUNT
            } else {
                1.0
            }
        };
        let skip = |idx: u32| self.entities[bot_e].bot.is_avoided(idx, now);

        let mut candidates = Vec::new();
        let mut consider = |item: EntId, cell: CellId, desire: f32, t: f32, mult: f32, horizon: Horizon| {
            if t >= PLAN_LOOKAHEAD {
                return;
            }
            // Beyond the tight first-goal horizon an ordinary item has no one-step score, but it
            // remains eligible as a second leg from one of the six useful nearby primaries.
            let mut score = item_score(desire, t, horizon).unwrap_or(0.0) * mult;
            if score > 0.0 && item.0 == current_goal {
                score *= GOAL_HYSTERESIS; // stick with the current goal against a near-tie
            }
            candidates.push(ItemCandidate {
                item,
                cell,
                desire,
                time: t,
                mult,
                horizon,
                one_score: score,
            });
        };

        for &(idx, cell) in &self.nav.goals {
            if skip(idx) {
                continue;
            }
            let item = EntId(idx);
            let Some(cat) = self.entities[item].classname().and_then(category) else {
                continue;
            };
            let powerup = matches!(cat, Category::Powerup);
            // The scoring tier (RA/mega reach farther than a shard). Only a true powerup keeps contest
            // immunity and the team split below — a major is contested and yielded like an ordinary
            // item (a weaker bot losing a mega race shouldn't feed a fight over it).
            let horizon = self.horizon_of(item, cat, s.stack);
            if horizon == Horizon::Major
                && teamwork
                && self.major_item_owned_by_teammate(bot_e, my_team, item, cat, now)
            {
                continue;
            }
            // Desire including any denial floor (weapon the enemy lacks, or mega/RA). Priced through the
            // shared helper so `item_goal_valid` agrees the goal is still worth holding.
            let desire = self.desire_with_floors(bot_e, &s, item, cat, deny, now);
            let travel = costs.cost_to(cell);
            if !travel.is_finite() {
                continue; // unreachable from here
            }
            let ent = &self.entities[item];
            if ent.v.solid != Solid::Trigger
                && matches!(ent.think, Think::SubRegen)
                && ent.v.nextthink > now
                && !respawn_departure_ready(horizon, ent.v.nextthink - now, travel)
            {
                continue;
            }
            let Some(t) = self.item_collect_time(item, travel, now) else {
                continue;
            };
            if self.team_match.live_until > now && now + t >= self.team_match.live_until {
                continue; // it cannot be collected before the structured match ends
            }
            let enemy_eta = enemy_context.as_ref().and_then(|(_, enemy_costs, _)| {
                let eta = enemy_costs.cost_to(cell);
                eta.is_finite()
                    .then(|| self.item_collect_time(item, eta, now))
                    .flatten()
            });
            // Give a powerup one deterministic owner while teammates take armor/weapons or cover its
            // approaches. Human Bravado 2on2 decisions keep these strategic goals split even under
            // contest pressure; the combat overlay can still pull the second bot into the fight.
            if powerup && teamwork {
                let item_org = self.entities[item].v.origin;
                let my_dist = (item_org - self.entities[bot_e].v.origin).length();
                let mate_dist = self.nearest_teammate_dist(bot_e, my_team, item_org);
                let claimed = self.item_claimed_by_teammate(bot_e, my_team, idx);
                if defer_powerup_to_teammate(claimed, my_dist, mate_dist) {
                    continue;
                }
            }
            let (desire, contest_mult) = if let Some((enemy_stats, _, weaker)) = &enemy_context {
                let enemy_desire = self.item_desire(enemy_stats, item, cat);
                contest_adjust(desire, enemy_desire, t, enemy_eta, *weaker, powerup)
            } else {
                (desire, 1.0)
            };
            if desire <= 0.0 {
                continue;
            }
            consider(item, cell, desire, t, claim_mult(idx) * contest_mult, horizon);
        }

        // CTF runes spawn and relocate dynamically, so they cannot live in the map-start static
        // catalog. Treat a live rune as a major powerup goal and feed it through the same bounded
        // planner/reservation rules. A holder cannot take another rune and therefore never scores it.
        for (i, ent) in self.entities.iter().enumerate() {
            if ent.touch != Touch::Rune || ent.v.solid != Solid::Trigger || skip(i as u32) {
                continue;
            }
            let desire = rune_desire(self.entities[bot_e].mode_p.ctf.runes, ent.item.rune_bit);
            if desire <= 0.0 {
                continue;
            }
            let Some(cell) = graph.nearest(ent.v.origin) else {
                continue;
            };
            let travel = costs.cost_to(cell);
            if !travel.is_finite() || (self.team_match.live_until > now && now + travel >= self.team_match.live_until) {
                continue;
            }
            let enemy_eta = enemy_context.as_ref().and_then(|(_, enemy_costs, _)| {
                let eta = enemy_costs.cost_to(cell);
                eta.is_finite().then_some(eta)
            });
            if teamwork && !enemy_eta.is_some_and(|eta| eta <= travel + 1.5) {
                let my_dist = (ent.v.origin - self.entities[bot_e].v.origin).length();
                let mate_dist = self.nearest_teammate_dist(bot_e, my_team, ent.v.origin);
                let claimed = self.item_claimed_by_teammate(bot_e, my_team, i as u32);
                if defer_powerup_to_teammate(claimed, my_dist, mate_dist) {
                    continue;
                }
            }
            consider(
                EntId(i as u32),
                cell,
                desire,
                travel,
                claim_mult(i as u32),
                Horizon::Powerup,
            );
        }

        // Live backpacks aren't in the static catalog (they spawn on death / a teammate's toss and
        // auto-remove), so scan the edicts for them each time.
        for (i, ent) in self.entities.iter().enumerate() {
            if ent.touch != Touch::Backpack || ent.v.solid != Solid::Trigger || skip(i as u32) {
                continue;
            }
            let item = EntId(i as u32);
            let desire = self.backpack_desire(&s, item);
            if desire <= 0.0 {
                continue;
            }
            let Some(cell) = graph.nearest(ent.v.origin) else {
                continue;
            };
            let travel = costs.cost_to(cell);
            if !travel.is_finite() {
                continue;
            }
            consider(item, cell, desire, travel, claim_mult(i as u32), Horizon::Ordinary);
        }

        // KTX-style two-goal evaluation, bounded to six continuation floods. A second goal changes
        // which first pickup is best (e.g. grab nearby rockets on the way to quad) without making
        // the bot blindly retain stale future state: the caller stores it only as a revalidated
        // continuation. Stable entity-id tie breaks keep identical matches deterministic.
        candidates.sort_by(|a, b| {
            b.one_score
                .total_cmp(&a.one_score)
                .then_with(|| a.item.0.cmp(&b.item.0))
        });
        // The up-to-six continuation costs (one per primary), all under the same pricing the base costs
        // used (`base` == `pricing.costs(0)`, still valid — `LinkCosts` is `Copy`). Coarse: a cheap
        // abstract Dijkstra each. Exact: the mutually-independent floods, fanned out in one ordered
        // batch across the worker pool. `leg_costs[pi]` is primary `pi`'s costs.
        let primaries: Vec<ItemCandidate> = candidates
            .iter()
            .filter(|c| c.one_score > 0.0)
            .take(PLAN_PRIMARY_LIMIT)
            .copied()
            .collect();
        let leg_costs: Vec<PlanCosts> = if lod {
            primaries
                .iter()
                .map(|c| PlanCosts::Coarse(graph.coarse_costs(c.cell, &base, true)))
                .collect()
        } else {
            let source_cells: Vec<CellId> = primaries.iter().map(|c| c.cell).collect();
            self.bot_pool
                .flood_batch(graph, &source_cells, &base)
                .into_iter()
                .map(PlanCosts::Exact)
                .collect()
        };
        let mut best: Option<(ItemCandidate, Option<ItemCandidate>, f32)> = None;
        for (pi, &first) in primaries.iter().enumerate() {
            let from_first = &leg_costs[pi];
            let mut best_second: Option<(ItemCandidate, f32)> = None;
            for &second in &candidates {
                if second.item == first.item {
                    continue;
                }
                let leg = from_first.cost_to(second.cell);
                if !leg.is_finite() {
                    continue;
                }
                let raw_total = first.time + leg;
                let Some(total) = self.item_collect_time(second.item, raw_total, now) else {
                    continue;
                };
                if self.team_match.live_until > now && now + total >= self.team_match.live_until {
                    continue;
                }
                let Some(score) = secondary_item_score(second.desire, total, second.horizon) else {
                    continue;
                };
                let score = score * second.mult * SECONDARY_WEIGHT;
                if best_second.is_none_or(|(_, old)| score > old) {
                    best_second = Some((second, score));
                }
            }
            let total = first.one_score + best_second.map_or(0.0, |(_, score)| score);
            if best.is_none_or(|(old, _, old_score)| {
                total > old_score || (total == old_score && first.item.0 < old.item.0)
            }) {
                best = Some((first, best_second.map(|(second, _)| second), total));
            }
        }
        best.map(|(first, second, _)| ItemPlan {
            first: (first.item, first.cell),
            second: second.map(|s| (s.item, s.cell)),
            first_desire: first.desire,
            contains_powerup: first.horizon == Horizon::Powerup
                || second.is_some_and(|s| s.horizon == Horizon::Powerup),
            contains_major: major_wanted(first.horizon, first.desire)
                || second.is_some_and(|s| major_wanted(s.horizon, s.desire)),
        })
    }

    /// The distance from the nearest living teammate (excluding `bot_e`, humans included) to `point`,
    /// or `None` if the bot has no living teammate. The powerup team-split reads this against the
    /// bot's own distance so the team spreads instead of dogpiling the quad.
    fn nearest_teammate_dist(&self, bot_e: EntId, my_team: u8, point: Vec3) -> Option<f32> {
        let maxclients = self.host().cvar(c"maxclients") as u32;
        (1..=maxclients)
            .map(EntId)
            .filter(|&t| {
                t != bot_e && {
                    let e = &self.entities[t];
                    e.is_player() && e.v.health > 0.0 && e.mode_p.team == my_team
                }
            })
            .map(|t| (self.entities[t].v.origin - point).length())
            .min_by(f32::total_cmp)
    }

    /// Whether an item entity is a major timed pickup (quad/pentagram/ring or a CTF rune) — the goal
    /// watchdog gives these a longer leash ([`POWERUP_GIVEUP`]) since a cross-map run is legitimate.
    pub(crate) fn is_powerup_item(&self, item: EntId) -> bool {
        self.entities[item].touch == Touch::Rune
            || self.entities[item]
                .classname()
                .and_then(category)
                .is_some_and(|c| matches!(c, Category::Powerup))
    }

    /// Whether an item is a *major* pickup (red armour or megahealth) under resource discipline — the
    /// goal watchdog gives these a middle leash ([`MAJOR_GIVEUP`]), between a shard and a powerup.
    pub(crate) fn is_major_item(&self, item: EntId) -> bool {
        let stack = self.host().cvar_bool(c"rtx_bot_stack");
        self.entities[item]
            .classname()
            .and_then(category)
            .is_some_and(|c| self.horizon_of(item, c, stack) == Horizon::Major)
    }
}

/// Stable owner of an item reservation: shortest distance, then lowest edict id. Callers include
/// the evaluating bot even before it has published a goal, making sequential bot frames converge
/// on the same owner regardless of who selected first.
fn reservation_owner(candidates: &[(EntId, f32)]) -> Option<EntId> {
    candidates
        .iter()
        .min_by(|a, b| a.1.total_cmp(&b.1).then_with(|| a.0 .0.cmp(&b.0 .0)))
        .map(|&(e, _)| e)
}

/// Need-versus-distance score used to reserve RA/mega within one team. One second of straight-line
/// running roughly doubles the denominator; actual route cost still decides whether the owner's
/// ordinary goal selection considers the item reachable at all.
fn strategic_owner_score(desire: f32, distance: f32) -> f32 {
    desire.max(DENIAL_DESIRE) / (1.0 + distance.max(0.0) / 320.0)
}

fn strategic_reservation_owner(candidates: &[(EntId, f32)]) -> Option<EntId> {
    candidates
        .iter()
        .max_by(|a, b| a.1.total_cmp(&b.1).then_with(|| b.0 .0.cmp(&a.0 .0)))
        .map(|&(e, _)| e)
}

/// Whether a plan leg is a *genuinely wanted* major pickup — a major-tier item (RA/mega) whose
/// post-contest desire clears the combat-detour bar. [`GameState::select_major_item`] breaks a
/// greed-off fight for one of these, but never for a bare denial floor (which sits below the bar), so
/// denial can't yank a bot out of every fight.
fn major_wanted(horizon: Horizon, desire: f32) -> bool {
    horizon == Horizon::Major && desire >= COMBAT_GREED_MIN_DESIRE
}

/// Pure core of magnet selection over `(entid, desire, dist)`: the item maximizing desire per unit
/// distance, so a near wanted item beats a far slightly-more-wanted one (a side-step, not a detour).
/// The `+ 32` smooths the point-blank divide and stops a touching item dominating everything. Lower
/// entity id breaks ties so sequential bot frames agree, like [`reservation_owner`].
fn best_magnet(candidates: &[(u32, f32, f32)]) -> Option<u32> {
    candidates
        .iter()
        .max_by(|a, b| {
            (a.1 / (a.2 + 32.0))
                .total_cmp(&(b.1 / (b.2 + 32.0)))
                .then_with(|| b.0.cmp(&a.0))
        })
        .map(|&(idx, _, _)| idx)
}

/// Pure core of the powerup team-split: defer (skip the powerup as a goal) when a teammate has
/// already claimed it, or the nearest teammate is within [`POWERUP_DEFER_RATIO`] of the bot's own
/// distance to it. With the ratio at `1.0`, any strictly nearer teammate takes precedence; exact
/// ties are resolved by the stable claim owner on the following selection.
fn defer_powerup_to_teammate(claimed: bool, my_dist: f32, best_mate_dist: Option<f32>) -> bool {
    claimed || best_mate_dist.is_some_and(|d| d < POWERUP_DEFER_RATIO * my_dist)
}

/// Whether it is time to leave for a hidden timed pickup. The broad scoring horizons still bound
/// how far ahead the bot understands a cycle, but departure is `route travel + setup lead`: a bot
/// across the map starts earlier than one already standing beside the spawn. Ordinary items retain
/// their historical horizon behavior; the anti-camp policy applies only to strategic timed items.
fn respawn_departure_ready(horizon: Horizon, respawn_wait: f32, travel: f32) -> bool {
    let lead = match horizon {
        Horizon::Powerup => POWERUP_SETUP_LEAD,
        Horizon::Major => MAJOR_SETUP_LEAD,
        Horizon::Ordinary => return true,
    };
    respawn_wait <= travel + lead
}

/// The goal score for an item, or `None` beyond its horizon. Ordinary items decay to zero at
/// [`LOOKAHEAD`]; a powerup is scored out to the wider [`POWERUP_LOOKAHEAD`] with a dominant,
/// gently-decaying numerator so it stays positive and outranks every ordinary item at any reachable
/// collect time `t` (with desire ≈ 200+). Higher is better; callers multiply claim/hysteresis factors.
fn item_score(desire: f32, t: f32, horizon: Horizon) -> Option<f32> {
    match horizon {
        Horizon::Powerup => (t < POWERUP_LOOKAHEAD).then(|| desire * POWERUP_LOOKAHEAD / (t + 5.0)),
        // Same decaying shape as an ordinary item but reaching twice as far, so an equal-desire major
        // at the same distance scores well above a shard — the "holy grail" bias as arithmetic.
        Horizon::Major => (t < MAJOR_LOOKAHEAD).then(|| desire * (MAJOR_LOOKAHEAD - t) / (t + 5.0)),
        Horizon::Ordinary => (t < LOOKAHEAD).then(|| desire * (LOOKAHEAD - t) / (t + 5.0)),
    }
}

/// Continuation score: timed powerups keep their dominant powerup curve; major and ordinary second
/// legs both use the wider total-plan horizon because the first pickup is already useful and makes the
/// route a deliberate sequence rather than a long single-item detour.
fn secondary_item_score(desire: f32, total_t: f32, horizon: Horizon) -> Option<f32> {
    match horizon {
        Horizon::Powerup => item_score(desire, total_t, Horizon::Powerup),
        _ => (total_t < PLAN_LOOKAHEAD).then(|| desire * (PLAN_LOOKAHEAD - total_t) / (total_t + 5.0)),
    }
}

/// Whether an ordinary first stop preserves the timing of a committed respawning powerup.
fn powerup_bridge_arrives_in_time(direct: f32, via: f32, respawn_wait: f32) -> bool {
    via.max(respawn_wait) <= direct.max(respawn_wait) + POWERUP_BRIDGE_SLACK
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn powerup_bridge_uses_respawn_slack_without_missing_spawn() {
        // A four-second side trip is free when both routes still arrive before a ten-second spawn.
        assert!(powerup_bridge_arrives_in_time(2.0, 6.0, 10.0));
        // The same trip is a real delay once the powerup is already due.
        assert!(!powerup_bridge_arrives_in_time(2.0, 6.0, 0.0));
        // Cell-level path noise on a pickup lying along the route gets the documented tolerance.
        assert!(powerup_bridge_arrives_in_time(2.0, 2.5, 0.0));
        assert!(!powerup_bridge_arrives_in_time(2.0, 2.51, 0.0));
    }

    #[test]
    fn timed_items_depart_by_travel_plus_setup_not_full_respawn() {
        // A bot beside a just-taken RA or halfway-due Quad keeps cycling the rest of the map.
        assert!(!respawn_departure_ready(Horizon::Major, 20.0, 0.25));
        assert!(!respawn_departure_ready(Horizon::Powerup, 30.0, 2.0));
        // A remote bot leaves sooner, but still arrives only by the intended setup margin.
        assert!(respawn_departure_ready(Horizon::Major, 5.0, 3.0));
        assert!(respawn_departure_ready(Horizon::Powerup, 7.0, 3.0));
        // Ordinary item behavior is deliberately unchanged.
        assert!(respawn_departure_ready(Horizon::Ordinary, 9.0, 0.0));
    }

    #[test]
    fn major_owner_balances_need_distance_and_stable_ties() {
        let weak_far = strategic_owner_score(120.0, 320.0);
        let stacked_near = strategic_owner_score(0.0, 64.0);
        assert!(weak_far > stacked_near, "real recovery need should beat nearby denial");

        let tiny_need_far = strategic_owner_score(31.0, 640.0);
        assert!(
            tiny_need_far < stacked_near,
            "a marginal need must not pull a bot across the map"
        );

        let tied = [(EntId(7), 10.0), (EntId(3), 10.0)];
        assert_eq!(strategic_reservation_owner(&tied), Some(EntId(3)));
    }

    #[test]
    fn denial_floor_gates() {
        // A big weapon the enemy lacks, no weapons-stay → the denial floor.
        assert_eq!(denial_floor(WeaponKind::Rl, false, false), DENIAL_DESIRE);
        assert_eq!(denial_floor(WeaponKind::Lg, false, false), DENIAL_DESIRE);
        // Lesser weapons aren't worth denying.
        assert_eq!(denial_floor(WeaponKind::Ssg, false, false), 0.0);
        assert_eq!(denial_floor(WeaponKind::Gl, false, false), 0.0);
        // Enemy already has it → nothing to deny.
        assert_eq!(denial_floor(WeaponKind::Rl, false, true), 0.0);
        // Weapons-stay (dm 2/3/5): the item lingers, so denial is meaningless.
        assert_eq!(denial_floor(WeaponKind::Rl, true, false), 0.0);
        // The floor sits below the combat-detour bar (so a bot never breaks off a fight to camp) and
        // above minor top-offs.
        assert!(denial_floor(WeaponKind::Rl, false, false) < COMBAT_GREED_MIN_DESIRE);
    }

    #[test]
    fn ffa_treats_everyone_as_the_enemy_side() {
        // FFA (my team 0): every other player is their own team, so all are the enemy side to deny.
        assert!(enemy_of(0, 0));
        assert!(enemy_of(0, 3));
        // A team composition: only a different team is an enemy; a teammate is not.
        assert!(enemy_of(1, 2));
        assert!(!enemy_of(1, 1));
    }

    #[test]
    fn hold_continues_until_an_abort() {
        // The happy path: before the deadline, weapon down, carrier alive/powered/lacking it, no
        // contest → keep holding.
        assert!(hold_continues(5.0, 9.0, true, true, true, false, false));
        // Each abort condition ends the hold.
        assert!(!hold_continues(9.0, 9.0, true, true, true, false, false)); // deadline reached
        assert!(!hold_continues(5.0, 9.0, false, true, true, false, false)); // weapon taken
        assert!(!hold_continues(5.0, 9.0, true, false, true, false, false)); // carrier died
        assert!(!hold_continues(5.0, 9.0, true, true, false, false, false)); // powerup expired
        assert!(!hold_continues(5.0, 9.0, true, true, true, true, false)); // carrier armed elsewhere
        assert!(!hold_continues(5.0, 9.0, true, true, true, false, true)); // enemy contesting
    }

    #[test]
    fn powerup_score_dominates_and_reaches_far() {
        // A quad (desire ~207) at t=20 must outrank a nearby health pack (desire ~50 at t=1) and a
        // nearby armor (desire ~80 at t=3) — the "MUST get" directive as arithmetic.
        let quad = item_score(207.0, 20.0, Horizon::Powerup).unwrap();
        let health = item_score(50.0, 1.0, Horizon::Ordinary).unwrap();
        let armor = item_score(80.0, 3.0, Horizon::Ordinary).unwrap();
        assert!(
            quad > health && quad > armor,
            "quad {quad} vs health {health}, armor {armor}"
        );
        // An ordinary item past LOOKAHEAD is dropped; a powerup at the same t is still a candidate.
        assert!(item_score(200.0, 12.0, Horizon::Ordinary).is_none());
        assert!(item_score(200.0, 12.0, Horizon::Powerup).is_some());
        // A powerup past its own (wider) horizon is dropped.
        assert!(item_score(200.0, POWERUP_LOOKAHEAD, Horizon::Powerup).is_none());
        // A quad far out (t=20) still beats a near red armour (t=5) — the powerup tier stays on top.
        let far_quad = item_score(207.0, 20.0, Horizon::Powerup).unwrap();
        let near_ra = item_score(100.0, 5.0, Horizon::Major).unwrap();
        assert!(far_quad > near_ra, "far quad {far_quad} vs near RA {near_ra}");
    }

    #[test]
    fn major_items_reach_past_the_ordinary_horizon() {
        // At t=15 a major (RA/mega) is still worth heading for; an equal-desire shard is long gone.
        assert!(item_score(100.0, 15.0, Horizon::Major).is_some());
        assert!(item_score(100.0, 15.0, Horizon::Ordinary).is_none());
        // A major decays to nothing at its own 20 s horizon.
        assert!(item_score(100.0, MAJOR_LOOKAHEAD, Horizon::Major).is_none());
        // Same distance, same desire: a major outweighs a shard (twice the reach → higher numerator).
        let major = item_score(100.0, 5.0, Horizon::Major).unwrap();
        let shard = item_score(100.0, 5.0, Horizon::Ordinary).unwrap();
        assert!(major > shard, "major {major} vs shard {shard}");
    }

    #[test]
    fn major_plans_break_greedless_fights_only_when_genuinely_wanted() {
        // A greed-off bot breaks off for a major only when its desire clears the combat-detour bar.
        assert!(major_wanted(Horizon::Major, COMBAT_GREED_MIN_DESIRE));
        assert!(major_wanted(Horizon::Major, 80.0));
        // A bare denial floor (30, below the bar) on a major must not yank the bot out of a fight.
        assert!(!major_wanted(Horizon::Major, DENIAL_DESIRE));
        // Ordinary and powerup legs are never "major-wanted" (powerups gate on contains_powerup).
        assert!(!major_wanted(Horizon::Ordinary, 100.0));
        assert!(!major_wanted(Horizon::Powerup, 100.0));
    }

    #[test]
    fn dry_primary_ammo_clears_the_combat_bar() {
        // Zero once adequately armed; rising as firepower collapses, past the combat-detour bar.
        assert_eq!(dry_urgency(AMMO_DRY_FIREPOWER), 0.0);
        assert_eq!(dry_urgency(60.0), 0.0);
        assert!(dry_urgency(24.0) >= COMBAT_GREED_MIN_DESIRE); // RL + 3 rockets (fp 24) ≈ 58
        assert!(dry_urgency(0.0) > dry_urgency(30.0)); // emptier = more urgent

        let stats = |items: Items, rockets: f32, cells: f32, fp: f32, stack: bool| Stats {
            health: 100.0,
            armor_value: 0.0,
            armor_type: 0.0,
            items: items.as_f32(),
            shells: 0.0,
            nails: 0.0,
            rockets,
            cells,
            strength: 100.0,
            armor: 0.0,
            firepower: fp,
            weapons_stay: false,
            stack,
        };
        let bar = COMBAT_GREED_MIN_DESIRE;
        // RL nearly dry (fp 16): rocket boxes clear the combat bar.
        assert!(ammo_desire(&stats(Items::ROCKET_LAUNCHER, 2.0, 0.0, 16.0, true), AmmoKind::Rockets) >= bar);
        // RL well-fed (fp 100): only the ordinary top-off, no panic.
        assert!(
            ammo_desire(
                &stats(Items::ROCKET_LAUNCHER, 15.0, 0.0, 100.0, true),
                AmmoKind::Rockets
            ) < bar
        );
        // Owns RL but LG-fed (fp 100 from cells): low rockets don't panic — the firepower gate self-solves.
        let lg_fed = stats(Items::LIGHTNING | Items::ROCKET_LAUNCHER, 2.0, 100.0, 100.0, true);
        assert!(ammo_desire(&lg_fed, AmmoKind::Rockets) < bar);
        // No launcher at all → no dry panic, just the base top-off.
        assert!(ammo_desire(&stats(Items::SUPER_SHOTGUN, 2.0, 0.0, 10.0, true), AmmoKind::Rockets) < bar);
        // Stack discipline off → base only, even bone-dry.
        assert!(ammo_desire(&stats(Items::ROCKET_LAUNCHER, 2.0, 0.0, 16.0, false), AmmoKind::Rockets) < bar);
    }

    #[test]
    fn secondary_item_extends_only_the_continuation_horizon() {
        // An ordinary item is too far away to begin a plan, but remains useful as the second stop
        // after a worthwhile nearby pickup has already put the bot on that route.
        assert!(item_score(60.0, LOOKAHEAD + 1.0, Horizon::Ordinary).is_none());
        assert!(secondary_item_score(60.0, LOOKAHEAD + 1.0, Horizon::Ordinary).is_some());
        assert!(secondary_item_score(60.0, PLAN_LOOKAHEAD - 0.1, Horizon::Ordinary).is_some());
        assert!(secondary_item_score(60.0, PLAN_LOOKAHEAD, Horizon::Ordinary).is_none());
        // A major second leg uses the continuation horizon too (not its own tighter 20 s bound).
        assert!(secondary_item_score(60.0, MAJOR_LOOKAHEAD + 1.0, Horizon::Major).is_some());
    }

    #[test]
    fn observed_enemy_need_adds_denial_without_forcing_a_lost_fight() {
        let (desire, mult) = contest_adjust(40.0, 80.0, 2.0, Some(3.0), false, false);
        assert_eq!(desire, 80.0);
        assert_eq!(mult, 1.0);

        // A substantially earlier, stronger enemy makes an ordinary pickup a poor feed. Timed
        // powerups are still worth contesting because yielding those can decide the whole fight.
        let (_, lost_mult) = contest_adjust(40.0, 80.0, 3.0, Some(2.0), true, false);
        assert_eq!(lost_mult, LOST_CONTEST_MULT);
        let (_, powerup_mult) = contest_adjust(40.0, 80.0, 3.0, Some(2.0), true, true);
        assert_eq!(powerup_mult, 1.0);
    }

    #[test]
    fn recovery_posture_has_stable_entry_and_exit_thresholds() {
        use super::super::state::CombatPosture::{Hold, Press, Recover};

        // Legacy health-only path (stack discipline off); strength is ignored, passed as 100.
        assert_eq!(posture_step(Hold, 40.0, 100.0, 100.0, Some(100.0), false), Recover);
        assert_eq!(posture_step(Hold, 100.0, 100.0, 50.0, Some(100.0), false), Recover);
        // Once recovering, crossing only one exit threshold is not enough to oscillate back.
        assert_eq!(posture_step(Recover, 59.0, 100.0, 100.0, Some(100.0), false), Recover);
        assert_eq!(posture_step(Recover, 100.0, 100.0, 84.0, Some(100.0), false), Recover);
        assert_eq!(posture_step(Recover, 60.0, 100.0, 85.0, Some(100.0), false), Hold);
        assert_eq!(posture_step(Hold, 100.0, 100.0, 136.0, Some(100.0), false), Press);
    }

    #[test]
    fn recovery_posture_is_stack_aware() {
        use super::super::state::CombatPosture::{Hold, Recover};
        // 40 hp behind yellow armour is 100 EHP — a real stack, so fight on rather than flee.
        let yellow = total_strength(40.0, 150.0, 0.6);
        assert!((yellow - 100.0).abs() < 0.01, "40hp+yellow is ~100 EHP, got {yellow}");
        assert_eq!(posture_step(Hold, 40.0, yellow, 100.0, None, true), Hold);
        // 40 hp naked is 40 EHP — a thin stack trips the strength floor even though health > 30.
        assert_eq!(posture_step(Hold, 40.0, 40.0, 100.0, None, true), Recover);
        // 30 hp behind red armour is a big stack, but one rocket from death — the health floor recovers.
        let red = total_strength(30.0, 200.0, 0.8);
        assert!(red > RECOVER_ENTER_STRENGTH);
        assert_eq!(posture_step(Hold, 30.0, red, 100.0, None, true), Recover);
    }

    #[test]
    fn disengage_only_when_the_break_is_survivable() {
        // Out of the enemy's sight → always safe to turn and go for cover/health.
        assert!(disengage_safe(false, 0.0));
        assert!(disengage_safe(false, 10_000.0));
        // In mutual sight → only with enough separation to reach cover before being shot in the back.
        assert!(!disengage_safe(true, DISENGAGE_SAFE_RANGE - 1.0));
        assert!(disengage_safe(true, DISENGAGE_SAFE_RANGE));
    }

    #[test]
    fn stack_pressure_ramps_below_the_5050_line() {
        // At or above the bare-spawn stack, no urgency; below it, up to the cap at ≈ a third.
        assert_eq!(stack_pressure(100.0), 1.0);
        assert_eq!(stack_pressure(200.0), 1.0);
        assert_eq!(stack_pressure(80.0), 1.25);
        assert_eq!(stack_pressure(40.0), STACK_PRESSURE_MAX); // 100/40 = 2.5, at the cap
        assert_eq!(stack_pressure(10.0), STACK_PRESSURE_MAX); // clamped, not runaway
                                                              // Monotonic: a thinner stack is never valued less.
        assert!(stack_pressure(70.0) > stack_pressure(90.0));
    }

    #[test]
    fn rune_goals_are_major_and_one_per_holder() {
        assert!(rune_desire(0, RUNE_RESISTANCE) > rune_desire(0, RUNE_STRENGTH));
        assert!(rune_desire(0, RUNE_STRENGTH) > rune_desire(0, RUNE_HASTE));
        assert!(rune_desire(0, RUNE_HASTE) > rune_desire(0, RUNE_REGEN));
        assert_eq!(rune_desire(RUNE_REGEN, RUNE_RESISTANCE), 0.0);
        assert_eq!(rune_desire(0, 0xff), 0.0);
    }

    #[test]
    fn reservation_owner_is_distance_then_entity_id() {
        let tied = [(EntId(7), 100.0), (EntId(3), 100.0), (EntId(5), 25.0)];
        assert_eq!(reservation_owner(&tied), Some(EntId(5)));
        let id_tie = [(EntId(7), 25.0), (EntId(3), 25.0)];
        assert_eq!(reservation_owner(&id_tie), Some(EntId(3)));
        assert_eq!(reservation_owner(&[]), None);
    }

    #[test]
    fn best_magnet_prefers_desire_per_distance_and_tie_breaks_on_id() {
        // A near, wanted item beats a far, slightly-more-wanted one (a side-step, not a detour).
        let near = (5u32, 60.0, 20.0); // 60/52 ≈ 1.15
        let far = (9u32, 100.0, 140.0); // 100/172 ≈ 0.58
        assert_eq!(best_magnet(&[near, far]), Some(5));
        // Exact tie on desire and distance → the lower entity id, so sequential frames agree.
        assert_eq!(best_magnet(&[(7, 40.0, 30.0), (3, 40.0, 30.0)]), Some(3));
        assert_eq!(best_magnet(&[]), None);
    }

    #[test]
    fn powerup_defer_splits_the_team() {
        // A teammate claim, or any strictly nearer teammate, defers this bot to something else.
        assert!(defer_powerup_to_teammate(true, 100.0, None)); // claimed
        assert!(defer_powerup_to_teammate(false, 1000.0, Some(500.0))); // mate at 0.5× my dist
        assert!(defer_powerup_to_teammate(false, 1000.0, Some(900.0))); // mate at 0.9× my dist
                                                                        // A farther teammate or no teammate leaves this bot as the powerup owner.
        assert!(!defer_powerup_to_teammate(false, 1000.0, Some(1100.0)));
        assert!(!defer_powerup_to_teammate(false, 1000.0, None));
    }
}
