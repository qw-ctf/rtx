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

use crate::arsenal::AmmoKind;
use crate::defs::{Bits, Items, Solid};
use crate::entity::{EntId, Think, Touch};
use crate::game::GameState;
use crate::navmesh::{CellId, LinkCosts};

/// Beyond this travel-or-respawn time (seconds) an item isn't worth pursuing.
pub(crate) const LOOKAHEAD: f32 = 10.0;

/// Minimum *desire* an item must have for a bot to break off combat and detour for it (the
/// `rtx_bot_greed` behavior). Set so powerups (200+) and a genuinely wanted weapon/health/armor
/// swing clear it, while a minor ammo top-off (≈2.5) doesn't — a bot won't abandon a firefight for
/// a handful of shells, but it will for the quad, an RL it lacks, or a big health/armor pickup.
pub(crate) const COMBAT_GREED_MIN_DESIRE: f32 = 40.0;

/// Relative bonus the item a bot is *already* chasing gets in goal scoring, so a marginally-better
/// alternative doesn't make it flip-flop between two near-equal pickups each re-selection (a loop
/// the navigation watchdogs can't see, since the bot keeps moving). Q3's goal-selection dampening.
const GOAL_HYSTERESIS: f32 = 1.3;

/// Score multiplier for an item a teammate bot has already claimed (is fetching), so teammates don't
/// race the same pickup. Small enough that a powerup's dominating desire still wins it (the quad
/// stays contested), large enough to discourage two bots converging on one health/armor.
const CLAIM_DISCOUNT: f32 = 0.3;

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
fn total_strength(health: f32, armor_value: f32, armor_type: f32) -> f32 {
    (health / (1.0 - armor_type)).min(health + armor_value).max(0.0)
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
        let ent = &self.entities[item];
        // A backpack has no goal classname and never respawns: it's valid while it's still on the
        // floor (solid) and still carries something this bot wants.
        if ent.touch == Touch::Backpack {
            return ent.v.solid == Solid::Trigger && self.backpack_desire(&self.bot_stats(bot_e), item) > 0.0;
        }
        let reachable_soon = ent.v.solid == Solid::Trigger
            || (matches!(ent.think, Think::SubRegen) && ent.v.nextthink - now < LOOKAHEAD);
        if !reachable_soon {
            return false;
        }
        let Some(cat) = ent.classname().and_then(category) else {
            return false;
        };
        self.item_desire(&self.bot_stats(bot_e), item, cat) > 0.0
    }

    /// Pick the highest-scoring reachable item goal for a bot, or `None` to fall back to following
    /// a human. Scans the static per-map catalog *and* any live dropped backpacks against a single
    /// Dijkstra flood from the bot.
    pub(crate) fn select_item_goal(&self, bot_e: EntId) -> Option<(EntId, CellId)> {
        self.best_item_goal(bot_e).map(|(item, cell, _)| (item, cell))
    }

    /// Pick an item worth *breaking off combat* for — the same best goal, but only returned when
    /// its desire clears [`COMBAT_GREED_MIN_DESIRE`]. Lets a fighting bot detour for the quad, a
    /// weapon it lacks, or a big health/armor pickup without chasing every trivial ammo box.
    pub(crate) fn select_combat_item(&self, bot_e: EntId) -> Option<(EntId, CellId)> {
        self.best_item_goal(bot_e)
            .filter(|&(_, _, desire)| desire >= COMBAT_GREED_MIN_DESIRE)
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
                e.bot.is_bot && e.v.health > 0.0 && e.mode_p.team == my_team && e.bot.goal_item == idx
            }
        })
    }

    fn best_item_goal(&self, bot_e: EntId) -> Option<(EntId, CellId, f32)> {
        let graph = self.nav.graph.as_ref()?;
        let bot_cell = graph.nearest(self.entities[bot_e].v.origin)?;
        let now = self.time();
        // Gate-aware: items behind a shut door cost more, so a bot prefers ones it can reach now.
        // Also charge this bot's failed-link penalties (jitter off — item scoring stays stable): an
        // item only reachable via a leg it keeps failing floods to a higher `t` and scores lower, so
        // it stops re-choosing an item it can path a route to but never actually traverse.
        let gate_closed = self.gate_closed_flags();
        let penalties: Vec<(u32, f32)> = self.entities[bot_e]
            .bot
            .failed_links
            .iter()
            .filter(|&&(_, until, _)| until > now)
            .map(|&(li, _, strikes)| (li, super::link_penalty_secs(strikes)))
            .collect();
        // The rocket-jump fitness gate too, so an item only reachable via a rocket jump scores as
        // (near-)unreachable for a bot that can't currently make one — it stops choosing it.
        let rj_extra = super::rj::rocket_jump_extra(
            &self.entities[bot_e].v,
            self.entities[bot_e].combat.super_damage_finished,
            now,
        );
        let costs = graph.costs_from(
            bot_cell,
            &LinkCosts {
                gate_closed: &gate_closed,
                penalties: &penalties,
                jitter_seed: 0,
                rocket_jump_extra: rj_extra,
            },
        );
        let s = self.bot_stats(bot_e);
        // The item we're already chasing, for the hysteresis bonus below.
        let current_goal = self.entities[bot_e].bot.goal_item;
        // Item claims (teamwork): an item a living teammate bot is already fetching is discounted, so
        // teammates spread across pickups instead of racing the same one. A powerup's dominating
        // desire still beats the discount, so the quad stays contested. Off in FFA (no team).
        let my_team = self.entities[bot_e].mode_p.team;
        let teamwork = my_team != 0 && self.host().cvar_bool(c"rtx_bot_teamwork");
        let claim_mult = |idx: u32| {
            if teamwork && self.item_claimed_by_teammate(bot_e, my_team, idx) {
                CLAIM_DISCOUNT
            } else {
                1.0
            }
        };
        let skip = |idx: u32| self.entities[bot_e].bot.is_avoided(idx, now);
        let score_of = |desire: f32, t: f32| desire * (LOOKAHEAD - t) / (t + 5.0);

        // best = (item, cell, desire, score) — score orders the search, desire is handed back.
        let mut best: Option<(EntId, CellId, f32, f32)> = None;
        let mut consider = |item: EntId, cell: CellId, desire: f32, t: f32, mult: f32| {
            if t < LOOKAHEAD {
                let mut score = score_of(desire, t) * mult;
                if item.0 == current_goal {
                    score *= GOAL_HYSTERESIS; // stick with the current goal against a near-tie
                }
                if best.is_none_or(|(_, _, _, b)| score > b) {
                    best = Some((item, cell, desire, score));
                }
            }
        };

        for &(idx, cell) in &self.nav.goals {
            if skip(idx) {
                continue;
            }
            let item = EntId(idx);
            let Some(cat) = self.entities[item].classname().and_then(category) else {
                continue;
            };
            let desire = self.item_desire(&s, item, cat);
            if desire <= 0.0 {
                continue;
            }
            let travel = costs[cell as usize];
            if !travel.is_finite() {
                continue; // unreachable from here
            }
            let Some(t) = self.item_collect_time(item, travel, now) else {
                continue;
            };
            consider(item, cell, desire, t, claim_mult(idx));
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
            consider(item, cell, desire, travel, claim_mult(i as u32));
        }

        best.map(|(item, cell, desire, _)| (item, cell, desire))
    }
}
