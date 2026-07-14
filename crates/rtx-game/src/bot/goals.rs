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

use glam::Vec3;

use crate::arsenal::AmmoKind;
use crate::defs::{Bits, Items, Solid};
use crate::entity::{EntId, Think, Touch};
use crate::game::GameState;
use crate::navmesh::CellId;

/// Beyond this travel-or-respawn time (seconds) an *ordinary* item isn't worth pursuing.
pub(crate) const LOOKAHEAD: f32 = 10.0;

/// Powerups (quad/pentagram) are planned much farther out than ordinary pickups: a powerup is worth
/// crossing the map for and worth *waiting at* (arrive early, deny the enemy its timing). 30 s covers
/// any trek plus the tail of the 60 s quad respawn, while ordinary items keep the tight [`LOOKAHEAD`]
/// so a bot doesn't detour half a minute for a shard.
const POWERUP_LOOKAHEAD: f32 = 30.0;

/// Give-up leash for a *powerup* goal (bot/mod.rs's `GOAL_GIVEUP_TIME` for ordinary items is 10 s).
/// A cross-map quad run legitimately takes longer than that; the progress watchdog still catches a
/// genuinely stuck bot far sooner. Sized to [`POWERUP_LOOKAHEAD`] plus a margin.
pub(crate) const POWERUP_GIVEUP: f32 = 35.0;

/// Powerup team-split threshold: a teammate counts as "so much nearer that I should take something
/// else" once they're within this fraction of the bot's own distance to the powerup. Below 1.0 so a
/// near-tie still lets both press it (redundancy against a contest) but a clear lead defers cleanly.
const POWERUP_DEFER_RATIO: f32 = 0.7;

/// Minimum *desire* an item must have for a bot to break off combat and detour for it (the
/// `rtx_bot_greed` behavior). Set so powerups (200+) and a genuinely wanted weapon/health/armor
/// swing clear it, while a minor ammo top-off (≈2.5) doesn't — a bot won't abandon a firefight for
/// a handful of shells, but it will for the quad, an RL it lacks, or a big health/armor pickup.
pub(crate) const COMBAT_GREED_MIN_DESIRE: f32 = 40.0;

/// A health/armor pickup is a completion-critical local recovery when it adds at least this much
/// effective strength. At critical health any positive recovery qualifies instead.
const LOCAL_RECOVERY_GAIN: f32 = 25.0;
/// Only pickups reachable inside this travel-time budget can pre-empt combat as a local completion.
const LOCAL_PICKUP_TRAVEL: f32 = 1.0;
/// Euclidean pre-filter before the local Dijkstra. It is deliberately looser than one second of
/// stock running to admit a short stair/turn while avoiding a flood when no relevant item is nearby.
const LOCAL_PICKUP_RADIUS: f32 = 384.0;
/// Strategic recovery may break contact for an item reachable inside this many seconds.
const RECOVERY_TRAVEL: f32 = 4.0;

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

/// Score multiplier for an item a teammate bot has already claimed (is fetching), so teammates don't
/// race the same pickup. Small enough that a powerup's dominating desire still wins it (the quad
/// stays contested), large enough to discourage two bots converging on one health/armor.
const CLAIM_DISCOUNT: f32 = 0.3;

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

/// A bounded item plan returned to the objective arbiter. `second` is promoted and revalidated after
/// touching `first`; it is never followed blindly. `contains_powerup` makes a bridge-to-powerup plan
/// completion-critical even when its first pickup is ordinary.
#[derive(Clone, Copy)]
pub(crate) struct ItemPlan {
    pub first: (EntId, CellId),
    pub second: Option<(EntId, CellId)>,
    pub first_desire: f32,
    pub contains_powerup: bool,
}

#[derive(Clone, Copy)]
struct ItemCandidate {
    item: EntId,
    cell: CellId,
    desire: f32,
    time: f32,
    mult: f32,
    powerup: bool,
    one_score: f32,
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
    now < hold_until
        && item_on_floor
        && mate_alive
        && mate_powered
        && !mate_has_weapon
        && !enemy_contesting
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
fn estimated_stats(est: crate::bot::model::OpponentEstimate, weapons_stay: bool) -> Stats {
    let has = |bit: Items| est.items.has(bit);
    let shells = if has(Items::SUPER_SHOTGUN) { 20.0 } else { 10.0 };
    let nails = if has(Items::NAILGUN) || has(Items::SUPER_NAILGUN) { 50.0 } else { 0.0 };
    let rockets = if has(Items::GRENADE_LAUNCHER) || has(Items::ROCKET_LAUNCHER) { 10.0 } else { 0.0 };
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

fn posture_step(
    previous: super::state::CombatPosture,
    health: f32,
    own_power: f32,
    enemy_power: Option<f32>,
) -> super::state::CombatPosture {
    use super::state::CombatPosture::*;
    let ratio = enemy_power.filter(|&p| p > 0.0).map_or(1.0, |p| own_power / p);
    if previous == Recover {
        if health < 60.0 || ratio < 0.85 {
            return Recover;
        }
    } else if health <= 40.0 || ratio < 0.6 {
        return Recover;
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

/// Desire for an ammo pickup — scales with how empty the bot is, zero once it's near the cap or
/// already well-armed (for the secondary ammo types).
fn ammo_desire(s: &Stats, a: AmmoKind) -> f32 {
    let fp = s.firepower;
    match a {
        AmmoKind::Rockets => {
            if s.rockets < 100.0 {
                (20.0 - s.rockets).max(5.0)
            } else {
                0.0
            }
        }
        AmmoKind::Cells => {
            if s.cells < 100.0 {
                ((50.0 - s.cells) * 0.2).max(2.5)
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
                if mega {
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
                }
            }
            Category::Armor {
                value,
                at,
                gate,
                double,
            } => {
                if s.armor < gate {
                    let gain = (total_strength(s.health, value, at) - s.strength).max(0.0);
                    if double {
                        gain * 2.0
                    } else {
                        gain
                    }
                } else {
                    0.0
                }
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
        let Some(cat) = ent.classname().and_then(category) else {
            return false;
        };
        // A powerup respawning within the wider powerup horizon is a valid standing goal (arrive
        // early and wait); ordinary items use the tight LOOKAHEAD.
        let horizon = if matches!(cat, Category::Powerup) {
            POWERUP_LOOKAHEAD
        } else {
            LOOKAHEAD
        };
        let reachable_soon = ent.v.solid == Solid::Trigger
            || (matches!(ent.think, Think::SubRegen) && ent.v.nextthink - now < horizon);
        if !reachable_soon {
            return false;
        }
        self.item_desire(&self.bot_stats(bot_e), item, cat) > 0.0
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
        let travel = graph.costs_from(from, &pricing.costs(0));
        nearby
            .into_iter()
            .filter_map(|(item, cell, desire, commit)| {
                let t = travel[cell as usize];
                (t.is_finite() && t <= LOCAL_PICKUP_TRAVEL)
                    .then_some((item, cell, commit, desire / (t + 0.25)))
            })
            .max_by(|a, b| a.3.total_cmp(&b.3).then_with(|| b.0.0.cmp(&a.0.0)))
            .map(|(item, cell, commit, _)| (item, cell, commit))
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
            let es = estimated_stats(est, s.weapons_stay);
            es.strength * es.firepower.max(10.0)
        });
        let posture = posture_step(previous, s.health, own_power, enemy_power);
        if posture != Recover {
            return (posture, None);
        }
        let Some(item) = self.select_recovery_item(bot_e, &s, now) else {
            return (Hold, None);
        };
        (Recover, Some(item))
    }

    fn select_recovery_item(&self, bot_e: EntId, s: &Stats, now: f32) -> Option<(EntId, CellId)> {
        let graph = self.nav.graph.as_ref()?;
        let from = graph.nearest(self.entities[bot_e].v.origin)?;
        let pricing = self.bot_link_pricing(bot_e, now);
        let costs = graph.costs_from(from, &pricing.costs(0));
        let my_team = self.entities[bot_e].mode_p.team;
        let teamwork = my_team != 0 && self.host().cvar_bool(c"rtx_bot_teamwork");
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
            .max_by(|a, b| a.2.total_cmp(&b.2).then_with(|| b.0.0.cmp(&a.0.0)))
            .map(|(item, cell, _)| (item, cell))
    }

    /// The best `(item, cell, desire)` for a bot by `desire × (LOOKAHEAD − t) / (t + 5)`, over both
    /// the static catalog and live backpacks. `desire` is returned so callers can apply their own
    /// bar (combat-detour vs. idle pickup). Backpacks are on the floor now, so their `t` is pure
    /// travel; the catalog folds in respawn-wait via [`item_collect_time`].
    /// Whether a living teammate bot is already fetching item `idx` (its `goal_item`) — the claim
    /// signal the item-scoring discount reads, so bots on a team don't converge on the same pickup.
    fn item_claimed_by_teammate(&self, bot_e: EntId, my_team: u8, idx: u32) -> bool {
        if idx == 0 {
            return false;
        }
        let maxclients = self.host().cvar(c"maxclients") as u32;
        (1..=maxclients).map(EntId).any(|t| {
            t != bot_e && {
                let e = &self.entities[t];
                e.bot.is_bot && e.v.health > 0.0 && e.mode_p.team == my_team && e.bot.goal.item == idx
            }
        })
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
                && e.mode_p.team != my_team
                && self
                    .opponent_est(bot_e, t, now)
                    .is_some_and(|est| est.items.has(bit))
        })
    }

    /// Maintain or begin a **handoff hold**: an idle bot may reserve a spawned RL/LG for a
    /// powerup-carrying teammate that lacks it — standing on the weapon without taking it until the
    /// carrier arrives, or taking it itself when the reservation lapses (denial beats a no-show).
    /// Returns whether the bot is holding this frame (its `goal_item` then points at the weapon and
    /// `bot_pickup_items` suppresses the grab). Team modes only, gated by `rtx_bot_model` +
    /// `rtx_bot_teamwork`; a non-idle bot (one with a fight or a move objective) never holds.
    pub(crate) fn update_handoff_hold(&mut self, e: EntId, now: f32, idle: bool) -> bool {
        let enabled = idle
            && self.entities[e].mode_p.team != 0
            && self.host().cvar_bool(c"rtx_bot_model")
            && self.host().cvar_bool(c"rtx_bot_teamwork");
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
        let mate_powered =
            m.combat.super_damage_finished > now || m.combat.invincible_finished > now;
        let mate_has_weapon = m.v.items.has(weapon_bit(w));
        // Contest: a perceived, living enemy near the weapon means "take it rather than leave it".
        let known = self.entities[e].bot.percept.known_enemy;
        let enemy_contesting = known != 0 && now < self.entities[e].bot.percept.known_until && {
            let ent = &self.entities[EntId(known)];
            ent.v.health > 0.0
                && (ent.v.origin - it.v.origin).length_squared()
                    < HOLD_CONTEST_RANGE * HOLD_CONTEST_RANGE
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
        let costs = graph.costs_from(bot_cell, &pricing.costs(0));
        let s = self.bot_stats(bot_e);
        let own_power = s.strength * s.firepower.max(10.0);
        // Only a currently remembered opponent contributes denial/contest information. Position is
        // the perception hypothesis (exact only when seen), and inventory/stack comes exclusively
        // from the observation-gated model — never from the live enemy edict.
        let enemy_context = {
            let p = &self.entities[bot_e].bot.percept;
            let enemy = EntId(p.known_enemy);
            if enemy.0 != 0 && now < p.known_until && self.entities[enemy].v.health > 0.0 {
                self.opponent_est(bot_e, enemy, now).and_then(|est| {
                    let cell = graph.nearest(p.last_seen)?;
                    let stats = estimated_stats(est, s.weapons_stay);
                    let power = stats.strength * stats.firepower.max(10.0);
                    Some((stats, graph.costs_from(cell, &pricing.costs(0)), own_power < 0.6 * power))
                })
            } else {
                None
            }
        };
        // The item we're already chasing, for the hysteresis bonus below.
        let current_goal = self.entities[bot_e].bot.goal.item;
        // Item claims (teamwork): an item a living teammate bot is already fetching is discounted, so
        // teammates spread across pickups instead of racing the same one. A powerup's dominating
        // desire still beats the discount, so the quad stays contested. Off in FFA (no team).
        let my_team = self.entities[bot_e].mode_p.team;
        let teamwork = my_team != 0 && self.host().cvar_bool(c"rtx_bot_teamwork");
        // Item denial (opponent modeling): raise the desire to hold a big weapon the enemy side is
        // believed to lack, so a team secures/guards the RL and LG spawns. Team play only, and only
        // when modeling is on (else "no belief" would masquerade as "enemy lacks it").
        let deny = teamwork && self.host().cvar_bool(c"rtx_bot_model");
        let claim_mult = |idx: u32| {
            if teamwork && self.item_claimed_by_teammate(bot_e, my_team, idx) {
                CLAIM_DISCOUNT
            } else {
                1.0
            }
        };
        let skip = |idx: u32| self.entities[bot_e].bot.is_avoided(idx, now);

        let mut candidates = Vec::new();
        let mut consider = |item: EntId, cell: CellId, desire: f32, t: f32, mult: f32, powerup: bool| {
            if t >= PLAN_LOOKAHEAD {
                return;
            }
            // Beyond the tight first-goal horizon an ordinary item has no one-step score, but it
            // remains eligible as a second leg from one of the six useful nearby primaries.
            let mut score = item_score(desire, t, powerup).unwrap_or(0.0) * mult;
            if score > 0.0 && item.0 == current_goal {
                score *= GOAL_HYSTERESIS; // stick with the current goal against a near-tie
            }
            candidates.push(ItemCandidate {
                item,
                cell,
                desire,
                time: t,
                mult,
                powerup,
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
            // Powerup team-split: the quad/pent is dominant, so left alone every team bot would
            // dogpile it. Instead a bot *defers* — skips it entirely — when a teammate has already
            // claimed it or is substantially nearer, and then picks the next-best item (armor/weapon
            // control). "You take quad, I take RA." Team modes only; FFA keeps everyone contesting.
            if powerup && teamwork {
                let item_org = self.entities[item].v.origin;
                let my_dist = (item_org - self.entities[bot_e].v.origin).length();
                let mate_dist = self.nearest_teammate_dist(bot_e, my_team, item_org);
                let claimed = self.item_claimed_by_teammate(bot_e, my_team, idx);
                if defer_powerup_to_teammate(claimed, my_dist, mate_dist) {
                    continue;
                }
            }
            let mut desire = self.item_desire(&s, item, cat);
            // Denial floor: an owned RL/LG normally scores ~0 desire, but if the enemy side lacks it
            // and it won't stay on the map, holding it still has value. Applied before the >0 gate so
            // an already-owned weapon isn't skipped.
            if deny {
                if let Category::Weapon(w) = cat {
                    let enemy_has = self.enemy_side_has_weapon(bot_e, my_team, weapon_bit(w), now);
                    desire = desire.max(denial_floor(w, s.weapons_stay, enemy_has));
                }
            }
            let travel = costs[cell as usize];
            if !travel.is_finite() {
                continue; // unreachable from here
            }
            let Some(t) = self.item_collect_time(item, travel, now) else {
                continue;
            };
            if self.team_match.live_until > now && now + t >= self.team_match.live_until {
                continue; // it cannot be collected before the structured match ends
            }
            let (desire, contest_mult) = if let Some((enemy_stats, enemy_costs, weaker)) = &enemy_context {
                let enemy_desire = self.item_desire(enemy_stats, item, cat);
                let eta = enemy_costs[cell as usize];
                contest_adjust(desire, enemy_desire, t, eta.is_finite().then_some(eta), *weaker, powerup)
            } else {
                (desire, 1.0)
            };
            if desire <= 0.0 {
                continue;
            }
            consider(item, cell, desire, t, claim_mult(idx) * contest_mult, powerup);
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
            let travel = costs[cell as usize];
            if !travel.is_finite() {
                continue;
            }
            consider(item, cell, desire, travel, claim_mult(i as u32), false);
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
        let pricing = self.bot_link_pricing(bot_e, now);
        let link_costs = pricing.costs(0);
        let mut best: Option<(ItemCandidate, Option<ItemCandidate>, f32)> = None;
        for &first in candidates.iter().filter(|c| c.one_score > 0.0).take(PLAN_PRIMARY_LIMIT) {
            let from_first = graph.costs_from(first.cell, &link_costs);
            let mut best_second: Option<(ItemCandidate, f32)> = None;
            for &second in &candidates {
                if second.item == first.item {
                    continue;
                }
                let leg = from_first[second.cell as usize];
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
                let Some(score) = secondary_item_score(second.desire, total, second.powerup) else {
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
            contains_powerup: first.powerup || second.is_some_and(|s| s.powerup),
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

    /// Whether an item entity is a powerup pickup (quad/pentagram/ring) — the goal watchdog gives
    /// these a longer leash ([`POWERUP_GIVEUP`]) since a cross-map powerup run is legitimately slow.
    pub(crate) fn is_powerup_item(&self, item: EntId) -> bool {
        self.entities[item]
            .classname()
            .and_then(category)
            .is_some_and(|c| matches!(c, Category::Powerup))
    }
}

/// Pure core of the powerup team-split: defer (skip the powerup as a goal) when a teammate has
/// already claimed it, or the nearest teammate is within [`POWERUP_DEFER_RATIO`] of the bot's own
/// distance to it. A clear teammate lead defers; a near-tie lets both press it.
fn defer_powerup_to_teammate(claimed: bool, my_dist: f32, best_mate_dist: Option<f32>) -> bool {
    claimed || best_mate_dist.is_some_and(|d| d < POWERUP_DEFER_RATIO * my_dist)
}

/// The goal score for an item, or `None` beyond its horizon. Ordinary items decay to zero at
/// [`LOOKAHEAD`]; a powerup is scored out to the wider [`POWERUP_LOOKAHEAD`] with a dominant,
/// gently-decaying numerator so it stays positive and outranks every ordinary item at any reachable
/// collect time `t` (with desire ≈ 200+). Higher is better; callers multiply claim/hysteresis factors.
fn item_score(desire: f32, t: f32, powerup: bool) -> Option<f32> {
    if powerup {
        (t < POWERUP_LOOKAHEAD).then(|| desire * POWERUP_LOOKAHEAD / (t + 5.0))
    } else {
        (t < LOOKAHEAD).then(|| desire * (LOOKAHEAD - t) / (t + 5.0))
    }
}

/// Continuation score: timed powerups keep their dominant powerup curve; ordinary second legs use
/// the wider total-plan horizon because the first pickup is already useful and makes the route a
/// deliberate sequence rather than a long single-item detour.
fn secondary_item_score(desire: f32, total_t: f32, powerup: bool) -> Option<f32> {
    if powerup {
        item_score(desire, total_t, true)
    } else {
        (total_t < PLAN_LOOKAHEAD).then(|| desire * (PLAN_LOOKAHEAD - total_t) / (total_t + 5.0))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
        let quad = item_score(207.0, 20.0, true).unwrap();
        let health = item_score(50.0, 1.0, false).unwrap();
        let armor = item_score(80.0, 3.0, false).unwrap();
        assert!(quad > health && quad > armor, "quad {quad} vs health {health}, armor {armor}");
        // An ordinary item past LOOKAHEAD is dropped; a powerup at the same t is still a candidate.
        assert!(item_score(200.0, 12.0, false).is_none());
        assert!(item_score(200.0, 12.0, true).is_some());
        // A powerup past its own (wider) horizon is dropped.
        assert!(item_score(200.0, POWERUP_LOOKAHEAD, true).is_none());
    }

    #[test]
    fn secondary_item_extends_only_the_continuation_horizon() {
        // An ordinary item is too far away to begin a plan, but remains useful as the second stop
        // after a worthwhile nearby pickup has already put the bot on that route.
        assert!(item_score(60.0, LOOKAHEAD + 1.0, false).is_none());
        assert!(secondary_item_score(60.0, LOOKAHEAD + 1.0, false).is_some());
        assert!(secondary_item_score(60.0, PLAN_LOOKAHEAD - 0.1, false).is_some());
        assert!(secondary_item_score(60.0, PLAN_LOOKAHEAD, false).is_none());
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

        assert_eq!(posture_step(Hold, 40.0, 100.0, Some(100.0)), Recover);
        assert_eq!(posture_step(Hold, 100.0, 50.0, Some(100.0)), Recover);
        // Once recovering, crossing only one exit threshold is not enough to oscillate back.
        assert_eq!(posture_step(Recover, 59.0, 100.0, Some(100.0)), Recover);
        assert_eq!(posture_step(Recover, 100.0, 84.0, Some(100.0)), Recover);
        assert_eq!(posture_step(Recover, 60.0, 85.0, Some(100.0)), Hold);
        assert_eq!(posture_step(Hold, 100.0, 136.0, Some(100.0)), Press);
    }

    #[test]
    fn powerup_defer_splits_the_team() {
        // A teammate claim, or one substantially nearer, defers this bot to something else.
        assert!(defer_powerup_to_teammate(true, 100.0, None)); // claimed
        assert!(defer_powerup_to_teammate(false, 1000.0, Some(500.0))); // mate at 0.5× my dist
        // A near-tie (mate at 0.9×) or no teammate → both press it / this bot pursues.
        assert!(!defer_powerup_to_teammate(false, 1000.0, Some(900.0)));
        assert!(!defer_powerup_to_teammate(false, 1000.0, None));
    }
}
