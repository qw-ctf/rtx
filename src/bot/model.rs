// SPDX-License-Identifier: AGPL-3.0-or-later

//! Opponent modeling — a shared, observation-gated hypothesis of each player's strength and arsenal.
//!
//! Bots today fight every opponent as an unknown: they read an enemy's *position* to aim, but never
//! its health, armor, or weapons (that would be a wallhack). A human, though, keeps a running read on
//! each opponent — "he's on low health", "they never got the RL", "that one has quad" — built from
//! what they've *seen and heard*, and in a team calls it out so teammates share the picture. This
//! module is the machine version of that read: a small point-estimate per opponent, updated only from
//! events a bot could legally witness, and pooled per side so a team's bots share one blackboard.
//!
//! **Design lineage.** The store is a shared blackboard / centralized working memory in the sense of
//! F.E.A.R.'s squad AI and Killzone 2/3's commander→squad→member hierarchy (members report what they
//! observe; the team acts on the merged picture). Items are treated as strategic currency the way
//! van Waveren's Quake III Arena team AI does — knowing the enemy lacks a key weapon turns its spawn
//! into ground worth denying. And every update is observation-gated, the honesty discipline of the
//! opponent-modeling literature: a belief only moves on evidence a bot could actually perceive.
//!
//! **Deliberately game-y.** Each [`OpponentEstimate`] is a `Copy` bag of point estimates plus a
//! last-observed clock, not a Bayes filter — a fighting game wants a cheap, legible "how weak / what
//! do they have", not a probability distribution. The estimates feed item denial, target selection,
//! combat risk, and the powerup handoff (see the consumers in [`goals`](crate::bot::goals),
//! [`team`](crate::mode::team), and [`combat`](crate::bot::combat)).
//!
//! **Pools.** Pool `0` is the FFA bot collective — in a free-for-all every bot shares one picture, so
//! "bots share this between each other" holds even though FFA has no teams. Pools `1..=8` are the
//! per-team blackboards; team A's sightings never leak into team B's pool. A player's entry is reset
//! to the mode's spawn kit when they die (death is public — frags are broadcast) or respawn.
//!
//! **Known limitations** (all documented so a future sound-event bus / occlusion pass can lift them):
//! - The witness test is earshot-by-radius with no occlusion — symmetric with perception's own
//!   [`HEAR_RADIUS`](crate::bot::perception) stand-in; both improve together when a real sound bus lands.
//! - Splash damage is credited to the attacker even through a wall (a human hears the pain — acceptable).
//! - Armor is never drifted upward, so an unseen armor pickup leaves a bot overestimating weakness;
//!   mitigated by the freshness gate on the one consumer that spends risk on the estimate.
//! - A hitscan hit from beyond earshot teaches nothing (no projectile to identify, fire out of range).
//! - Pool 0 is bots-only; a human's private knowledge in FFA isn't modeled.

use glam::Vec3;

use crate::defs::{Bits, Items, Weapon};
use crate::entity::EntId;
use crate::game::GameState;

/// Pools: index `0` is the FFA bot collective, `1..=8` the per-team blackboards (QW caps teams at 8).
const MAX_POOLS: usize = 9;
/// Entries indexed by client slot `EntId.0`; slot `0` (the world) is unused. QW's `maxclients` ceiling.
const MAX_SLOTS: usize = 33;

/// A witness must be within this range of an event (gunfire, a pickup) to update its pool — mirrors
/// [`perception::HEAR_RADIUS`](crate::bot::perception); every sound involved is `Attenuation::Norm`,
/// so this is the same earshot perception already treats as audible.
const WITNESS_RADIUS: f32 = 1000.0;

/// A below-prior health estimate only starts drifting back up after this many unobserved seconds —
/// the same object-permanence window as [`perception::MEMORY`](crate::bot::perception). Inside it the
/// last read still stands.
const DRIFT_GRACE: f32 = 5.0;
/// Then the estimate rises toward the 100 prior at this rate: an unseen player is presumed to be
/// picking health back up. 2 hp/s means a 30 hp survivor reads as recovered ~40 s later — about two
/// health-respawn cycles plus travel, the human "he's probably healed by now".
const DRIFT_RATE: f32 = 2.0;

/// How long a witnessed quad/pentagram is believed to last (matches the powerup grant in `items.rs`).
const POWERUP_SECS: f32 = 30.0;

/// One observer-pool's belief about one player: the "enemy low, no RL" callout as data. `Copy`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct OpponentEstimate {
    /// Estimated health. Drifts back toward the 100 prior when unobserved ([`drifted_health`]).
    pub health: f32,
    /// Estimated armor points and absorb type (0.0 / 0.3 / 0.6 / 0.8). Modeled *jointly* with health
    /// because an attacker can't observe the take-vs-save split of a hit (see [`apply_damage`]).
    pub armor_value: f32,
    pub armor_type: f32,
    /// Believed arsenal (`Items` weapon bits, carried as `f32` like the engine). Sticky until death —
    /// in QuakeWorld a weapon is only lost by dying.
    pub items: f32,
    /// Witnessed powerup expiries (`0` = none believed). Quad glows blue and hums, pentagram flashes —
    /// both externally observable, and the pickup sound is audible.
    pub quad_until: f32,
    pub pent_until: f32,
    /// World time of the last witnessed observation — drives the staleness drift and the freshness
    /// gate on the risk consumer.
    pub last_update: f32,
}

impl Default for OpponentEstimate {
    /// The stock respawn kit (`PlayerParms::fresh`): shotgun + axe, 100 health, no armor.
    fn default() -> Self {
        Self {
            health: 100.0,
            armor_value: 0.0,
            armor_type: 0.0,
            items: (Items::SHOTGUN | Items::AXE).as_f32(),
            quad_until: 0.0,
            pent_until: 0.0,
            last_update: 0.0,
        }
    }
}

/// A witnessed pickup, mapped to the estimate change it implies. Mirrors the touch handlers in
/// `items.rs` (armor *replaces* value/type; health caps at 100, megahealth at 250).
#[derive(Clone, Copy, Debug)]
pub(crate) enum PickupKind {
    Health(f32),
    Mega,
    Armor { value: f32, atype: f32 },
    Weapon(Items),
    Quad,
    Pent,
    Backpack(f32),
}

/// The shared store: one estimate per player per pool, plus the mode's spawn-kit baseline that a
/// death/respawn resets to. Fixed arrays (~9 KB) — no per-frame cost, no allocation.
pub(crate) struct OpponentModel {
    pools: [[OpponentEstimate; MAX_SLOTS]; MAX_POOLS],
    baseline: OpponentEstimate,
}

impl Default for OpponentModel {
    fn default() -> Self {
        Self::new(OpponentEstimate::default())
    }
}

impl OpponentModel {
    /// A fresh store where every entry starts at `baseline` (the mode's spawn kit).
    pub(crate) fn new(baseline: OpponentEstimate) -> Self {
        Self {
            pools: [[baseline; MAX_SLOTS]; MAX_POOLS],
            baseline,
        }
    }

    /// Read a drifted-nothing snapshot of what `pool` believes about `target` (defaults if OOB).
    pub(crate) fn entry(&self, pool: usize, target: EntId) -> OpponentEstimate {
        let slot = target.0 as usize;
        if pool >= MAX_POOLS || slot == 0 || slot >= MAX_SLOTS {
            return OpponentEstimate::default();
        }
        self.pools[pool][slot]
    }

    /// Mutable access, bounds-guarded — a hook can never panic on an exotic `maxclients`.
    fn entry_mut(&mut self, pool: usize, target: EntId) -> Option<&mut OpponentEstimate> {
        let slot = target.0 as usize;
        if pool >= MAX_POOLS || slot == 0 || slot >= MAX_SLOTS {
            return None;
        }
        Some(&mut self.pools[pool][slot])
    }

    /// Reset `target`'s entry in **every** pool to the spawn-kit baseline — called on death (public,
    /// broadcast) and respawn, so a fresh loadout is never fought as if it were the old one.
    pub(crate) fn reset_target(&mut self, target: EntId, now: f32) {
        let slot = target.0 as usize;
        if slot == 0 || slot >= MAX_SLOTS {
            return;
        }
        let mut fresh = self.baseline;
        fresh.last_update = now;
        for pool in &mut self.pools {
            pool[slot] = fresh;
        }
    }

    /// A witnessed weapon fire / hit proves `target` owns that weapon.
    pub(crate) fn note_weapon(&mut self, pool: usize, target: EntId, bit: Items, now: f32) {
        if let Some(e) = self.entry_mut(pool, target) {
            e.items = e.items.with(bit);
            e.last_update = now;
        }
    }

    /// Damage the attacker delivered (pre-armor) run through the estimate's own armor model.
    pub(crate) fn note_damage(&mut self, pool: usize, target: EntId, damage: f32, now: f32) {
        if let Some(e) = self.entry_mut(pool, target) {
            *e = apply_damage(*e, damage);
            e.last_update = now;
        }
    }

    /// A witnessed pickup raises the corresponding estimate.
    pub(crate) fn note_pickup(&mut self, pool: usize, target: EntId, kind: PickupKind, now: f32) {
        let Some(e) = self.entry_mut(pool, target) else {
            return;
        };
        match kind {
            PickupKind::Health(amount) => e.health = (e.health + amount).min(100.0),
            PickupKind::Mega => e.health = (e.health + 100.0).min(250.0),
            PickupKind::Armor { value, atype } => {
                e.armor_value = value;
                e.armor_type = atype;
            }
            PickupKind::Weapon(bit) => e.items = e.items.with(bit),
            PickupKind::Quad => e.quad_until = now + POWERUP_SECS,
            PickupKind::Pent => e.pent_until = now + POWERUP_SECS,
            PickupKind::Backpack(bits) => e.items = e.items.with(bits),
        }
        e.last_update = now;
    }
}

/// The estimate every entry resets to for the active mode. FFA / team / CTF spawn the stock fresh kit
/// (their `apply_loadout` only assigns teams); Midair is the fixed RL kit (RL+axe, 250 hp, red armor);
/// Arena fighters get the full arsenal + red armor. In those fixed-kit modes the arsenal hypothesis is
/// trivially correct and the strength consumers still work from the right starting stack.
pub(crate) fn baseline_for_mode(name: &str) -> OpponentEstimate {
    let mut est = OpponentEstimate::default();
    match name {
        "midair" => {
            est.items = (Items::AXE | Items::ROCKET_LAUNCHER).as_f32();
            est.health = 250.0;
            est.armor_value = 200.0;
            est.armor_type = 0.8;
        }
        "arena" => {
            est.items = (Items::AXE
                | Items::SHOTGUN
                | Items::SUPER_SHOTGUN
                | Items::NAILGUN
                | Items::SUPER_NAILGUN
                | Items::GRENADE_LAUNCHER
                | Items::ROCKET_LAUNCHER)
                .as_f32();
            est.armor_value = 200.0;
            est.armor_type = 0.8;
        }
        _ => {}
    }
    est
}

/// A below-prior health estimate rises back toward 100 at [`DRIFT_RATE`] once it's gone
/// [`DRIFT_GRACE`] seconds without a fresh observation — an unseen player is presumed to be topping
/// up. Estimates at or above the prior (including witnessed megahealth overheal) are left alone;
/// megahealth rots on its own and modeling that would be false precision.
pub(crate) fn drifted_health(health: f32, last_update: f32, now: f32) -> f32 {
    if health >= 100.0 {
        return health;
    }
    let idle = now - last_update - DRIFT_GRACE;
    if idle <= 0.0 {
        return health;
    }
    (health + DRIFT_RATE * idle).min(100.0)
}

/// Apply `damage` (the pre-armor amount the attacker knows they delivered) to an estimate, mirroring
/// `t_damage`'s absorb exactly: `save = min(ceil(type · damage), value)`, a broken plate zeroes the
/// type, and `health` drops by `ceil(damage − save)`. Because the attacker knows their own quad and
/// the mode ruleset is public, the caller passes the already-scaled delivered damage — no divide-out.
pub(crate) fn apply_damage(mut est: OpponentEstimate, damage: f32) -> OpponentEstimate {
    let mut save = (est.armor_type * damage).ceil();
    if save >= est.armor_value {
        save = est.armor_value;
        est.armor_type = 0.0;
    }
    est.armor_value -= save;
    est.health -= (damage - save).ceil();
    est
}

/// The estimated effective hit points (`TotalStrength`): drifted health run through the estimated
/// armor. The single number the strength consumers rank opponents by.
pub(crate) fn est_strength(est: &OpponentEstimate, now: f32) -> f32 {
    let health = drifted_health(est.health, est.last_update, now);
    crate::bot::goals::total_strength(health, est.armor_value, est.armor_type)
}

/// Distance² multiplier for weighing a candidate target: a weak stack scores as if nearer, a strong
/// one as if farther, and a target believed to hold a big weapon in a no-weapons-stay game is nudged
/// preferred (killing them resets their kit — a real swing in deathmatch 1). Clamped so the bias
/// only reorders near-ties; it never lets a distant target leapfrog a much closer one.
///   strength: 30 hp naked → 0.4×; a full 100/200-red stack → 2.5×.
pub(crate) fn target_bias(est_strength: f32, armed_big: bool, weapons_stay: bool) -> f32 {
    let strength_mult = (est_strength / 100.0).clamp(0.4, 2.5);
    let armed_mult = if armed_big && !weapons_stay { 0.8 } else { 1.0 };
    strength_mult * armed_mult
}

/// The `Items` bit a fired active weapon proves ownership of, or `None` for Axe / Grapple / no weapon
/// (which carry no arsenal information worth recording).
pub(crate) fn weapon_fire_bit(w: Weapon) -> Option<Items> {
    if w == Weapon::None || w == Weapon::Axe || w == Weapon::Grapple {
        return None;
    }
    Some(w.item())
}

/// Walk the pools named by a witness bitmask (from [`GameState::witness_pools`]).
pub(crate) fn iter_pools(mask: u16) -> impl Iterator<Item = usize> {
    (0..MAX_POOLS).filter(move |&p| (mask >> p) & 1 == 1)
}

impl GameState {
    /// The observer's pool: their team in team modes, the FFA bot collective (`0`) for a team-0 *bot*,
    /// or `None` for a team-0 human (whose private knowledge isn't modeled). Pure classification — the
    /// cvar gate lives on the hook wrappers below, so this stays usable from tests and read paths.
    pub(crate) fn observer_pool(&self, observer: EntId) -> Option<usize> {
        let ent = &self.entities[observer];
        let team = ent.mode_p.team as usize;
        if team >= 1 {
            Some(team.min(MAX_POOLS - 1))
        } else if ent.bot.is_bot {
            Some(0)
        } else {
            None
        }
    }

    /// The set of pools with a plausible witness to an event at `pos`: any live bot on that pool's
    /// side within [`WITNESS_RADIUS`]. One pass over the client slots — events (shots, pickups) are
    /// rare, not per-frame. Empty when opponent modeling is off.
    pub(crate) fn witness_pools(&self, pos: Vec3) -> u16 {
        if !self.host.cvar_bool(c"rtx_bot_model") {
            return 0;
        }
        let maxclients = self.host().cvar(c"maxclients") as u32;
        let mut mask = 0u16;
        for e in (1..=maxclients).map(EntId) {
            let ent = &self.entities[e];
            if !ent.in_use || !ent.bot.is_bot || ent.v.health <= 0.0 {
                continue;
            }
            if ent.classname() != Some("player") {
                continue;
            }
            let Some(pool) = self.observer_pool(e) else {
                continue;
            };
            if (mask >> pool) & 1 == 1 {
                continue; // pool already has a witness
            }
            if (ent.v.origin - pos).length_squared() <= WITNESS_RADIUS * WITNESS_RADIUS {
                mask |= 1 << pool;
            }
        }
        mask
    }

    /// What `observer`'s pool believes about `target` right now (drift applied to health). `None` when
    /// modeling is off or the observer has no pool (a team-0 human). The returned copy keeps
    /// `last_update` intact so a consumer can age the belief; only `health` is drifted.
    pub(crate) fn opponent_est(
        &self,
        observer: EntId,
        target: EntId,
        now: f32,
    ) -> Option<OpponentEstimate> {
        if !self.host.cvar_bool(c"rtx_bot_model") {
            return None;
        }
        let pool = self.observer_pool(observer)?;
        let mut est = self.opponents.entry(pool, target);
        est.health = drifted_health(est.health, est.last_update, now);
        Some(est)
    }

    /// Hook: `attacker` dealt `damage` (pre-armor) to `targ`. The attacker's whole side learns it —
    /// they saw the hit land — so this updates the attacker's pool directly, not by earshot.
    pub(crate) fn model_note_damage(&mut self, attacker: EntId, targ: EntId, damage: f32) {
        if !self.host.cvar_bool(c"rtx_bot_model") || attacker == targ {
            return;
        }
        if self.entities[attacker].classname() != Some("player")
            || self.entities[targ].classname() != Some("player")
        {
            return;
        }
        if let Some(pool) = self.observer_pool(attacker) {
            let now = self.time();
            self.opponents.note_damage(pool, targ, damage, now);
        }
    }

    /// Hook: `attacker` proved they own `bit` (e.g. by hitting `targ` with an identifiable projectile).
    /// Learned by whoever `targ`'s hit implicates — the *victim's* side (they felt what hit them).
    pub(crate) fn model_note_weapon_of_attacker(&mut self, victim: EntId, attacker: EntId, bit: Items) {
        if !self.host.cvar_bool(c"rtx_bot_model") || attacker == victim {
            return;
        }
        if self.entities[attacker].classname() != Some("player") {
            return;
        }
        if let Some(pool) = self.observer_pool(victim) {
            let now = self.time();
            self.opponents.note_weapon(pool, attacker, bit, now);
        }
    }

    /// Hook: `firer` started a weapon fire. Every pool with a bot in earshot learns which weapon.
    pub(crate) fn model_note_weapon_fire(&mut self, firer: EntId) {
        if !self.host.cvar_bool(c"rtx_bot_model") {
            return;
        }
        let Some(bit) = weapon_fire_bit(self.entities[firer].v.weapon) else {
            return;
        };
        let org = self.entities[firer].v.origin;
        let now = self.time();
        let mask = self.witness_pools(org);
        for pool in iter_pools(mask) {
            self.opponents.note_weapon(pool, firer, bit, now);
        }
    }

    /// Hook: `picker` collected an item. Every pool with a bot in earshot of the pickup learns it.
    pub(crate) fn model_note_pickup(&mut self, picker: EntId, kind: PickupKind) {
        if !self.host.cvar_bool(c"rtx_bot_model") {
            return;
        }
        let org = self.entities[picker].v.origin;
        let now = self.time();
        let mask = self.witness_pools(org);
        for pool in iter_pools(mask) {
            self.opponents.note_pickup(pool, picker, kind, now);
        }
    }

    /// The distance² weighting for choosing `target` from `observer`'s view under opponent modeling:
    /// [`target_bias`] fed by the shared estimate, or `1.0` when there's no belief (or modeling is
    /// off), so a caller can multiply its raw dist² unconditionally and get plain nearest when off.
    pub(crate) fn target_dist_bias(
        &self,
        observer: EntId,
        target: EntId,
        now: f32,
        weapons_stay: bool,
    ) -> f32 {
        match self.opponent_est(observer, target, now) {
            Some(est) => {
                let armed_big =
                    est.items.has(Items::ROCKET_LAUNCHER) || est.items.has(Items::LIGHTNING);
                target_bias(est_strength(&est, now), armed_big, weapons_stay)
            }
            None => 1.0,
        }
    }

    /// Hook: `target` died or respawned — wipe its entry back to the spawn kit in every pool.
    pub(crate) fn model_reset_target(&mut self, target: EntId) {
        if !self.host.cvar_bool(c"rtx_bot_model") {
            return;
        }
        if self.entities[target].classname() != Some("player") {
            return;
        }
        let now = self.time();
        self.opponents.reset_target(target, now);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn est(health: f32, armor_value: f32, armor_type: f32) -> OpponentEstimate {
        OpponentEstimate {
            health,
            armor_value,
            armor_type,
            ..Default::default()
        }
    }

    #[test]
    fn baseline_is_fresh_kit() {
        let e = OpponentEstimate::default();
        assert_eq!(e.health, 100.0);
        assert_eq!(e.armor_value, 0.0);
        assert!(e.items.has(Items::SHOTGUN) && e.items.has(Items::AXE));
        assert!(!e.items.has(Items::ROCKET_LAUNCHER));
    }

    #[test]
    fn baseline_for_mode_arms_fixed_kits() {
        let midair = baseline_for_mode("midair");
        assert_eq!(midair.health, 250.0);
        assert!(midair.items.has(Items::ROCKET_LAUNCHER));
        assert_eq!(midair.armor_type, 0.8);

        let arena = baseline_for_mode("arena");
        assert!(arena.items.has(Items::ROCKET_LAUNCHER) && arena.items.has(Items::SUPER_NAILGUN));
        assert_eq!(arena.armor_value, 200.0);

        // Everything else is the stock kit.
        assert_eq!(baseline_for_mode("ffa"), OpponentEstimate::default());
        assert_eq!(baseline_for_mode("teamplay"), OpponentEstimate::default());
    }

    #[test]
    fn damage_absorb_matches_t_damage() {
        // 60 damage into 200-point red armor (0.8): save = ceil(0.8·60)=48, health −ceil(60−48)=−12.
        let after = apply_damage(est(100.0, 200.0, 0.8), 60.0);
        assert_eq!(after.armor_value, 152.0);
        assert_eq!(after.armor_type, 0.8);
        assert_eq!(after.health, 88.0);
    }

    #[test]
    fn damage_breaks_thin_armor_and_zeroes_type() {
        // 100 damage into 10-point green armor (0.3): save clamps to 10, plate breaks (type→0),
        // health −ceil(100−10)=−90.
        let after = apply_damage(est(100.0, 10.0, 0.3), 100.0);
        assert_eq!(after.armor_value, 0.0);
        assert_eq!(after.armor_type, 0.0);
        assert_eq!(after.health, 10.0);
    }

    #[test]
    fn quadded_damage_is_the_delivered_amount() {
        // The caller passes post-quad delivered damage; a quad rocket to the chest reads as 120 into
        // a naked stack → 120 health gone. (Contract: no divide-out inside the model.)
        let after = apply_damage(est(100.0, 0.0, 0.0), 120.0);
        assert_eq!(after.health, -20.0);
    }

    #[test]
    fn pickup_deltas_mirror_handlers() {
        let mut m = OpponentModel::default();
        let t = EntId(3);
        // Health caps at 100; mega lifts overheal to 250.
        m.note_pickup(0, t, PickupKind::Health(25.0), 1.0);
        assert_eq!(m.entry(0, t).health, 100.0);
        m.note_pickup(0, t, PickupKind::Mega, 1.0);
        assert_eq!(m.entry(0, t).health, 200.0);
        // Armor replaces value/type.
        m.note_pickup(0, t, PickupKind::Armor { value: 150.0, atype: 0.6 }, 1.0);
        assert_eq!(m.entry(0, t).armor_value, 150.0);
        assert_eq!(m.entry(0, t).armor_type, 0.6);
        // Weapon ORs the bit; quad sets a 30 s expiry.
        m.note_pickup(0, t, PickupKind::Weapon(Items::ROCKET_LAUNCHER), 1.0);
        assert!(m.entry(0, t).items.has(Items::ROCKET_LAUNCHER));
        m.note_pickup(0, t, PickupKind::Quad, 5.0);
        assert_eq!(m.entry(0, t).quad_until, 35.0);
    }

    #[test]
    fn death_resets_all_pools() {
        let mut m = OpponentModel::default();
        let t = EntId(4);
        m.note_pickup(0, t, PickupKind::Weapon(Items::ROCKET_LAUNCHER), 1.0);
        m.note_pickup(3, t, PickupKind::Weapon(Items::LIGHTNING), 1.0);
        m.note_damage(0, t, 50.0, 1.0);
        m.reset_target(t, 9.0);
        for pool in [0, 3] {
            let e = m.entry(pool, t);
            assert_eq!(e.health, 100.0);
            assert!(!e.items.has(Items::ROCKET_LAUNCHER) && !e.items.has(Items::LIGHTNING));
            assert_eq!(e.last_update, 9.0);
        }
    }

    #[test]
    fn staleness_drifts_up_after_grace_and_caps() {
        // Below prior: no drift inside the grace window, then +2/s, capped at 100.
        assert_eq!(drifted_health(30.0, 0.0, 4.0), 30.0); // within DRIFT_GRACE
        assert_eq!(drifted_health(30.0, 0.0, 10.0), 40.0); // 5 s past grace · 2/s
        assert_eq!(drifted_health(30.0, 0.0, 100.0), 100.0); // capped
        // At/above prior (incl. witnessed mega overheal) never drifts.
        assert_eq!(drifted_health(100.0, 0.0, 100.0), 100.0);
        assert_eq!(drifted_health(250.0, 0.0, 100.0), 250.0);
    }

    #[test]
    fn est_strength_uses_drift_and_armor() {
        // Fresh naked stack.
        assert_eq!(est_strength(&est(100.0, 0.0, 0.0), 0.0), 100.0);
        // 50 hp under yellow armor (0.6, 50 pts): effective = min(50/0.4, 50+50) = min(125,100) = 100.
        assert_eq!(est_strength(&est(50.0, 50.0, 0.6), 0.0), 100.0);
    }

    #[test]
    fn weapon_fire_bit_skips_melee_and_grapple() {
        assert_eq!(weapon_fire_bit(Weapon::Axe), None);
        assert_eq!(weapon_fire_bit(Weapon::Grapple), None);
        assert_eq!(weapon_fire_bit(Weapon::None), None);
        assert_eq!(weapon_fire_bit(Weapon::RocketLauncher), Some(Items::ROCKET_LAUNCHER));
    }

    #[test]
    fn iter_pools_walks_set_bits() {
        let mask = (1 << 0) | (1 << 3) | (1 << 8);
        assert_eq!(iter_pools(mask).collect::<Vec<_>>(), vec![0, 3, 8]);
    }

    #[test]
    fn target_bias_orders_weak_first() {
        // A weak stack reads as nearer, a strong one as farther: a weak enemy at distance 200 should
        // outrank (lower biased dist²) a full-stack enemy at 150.
        let weak = 200.0_f32.powi(2) * target_bias(30.0, false, false);
        let strong = 150.0_f32.powi(2) * target_bias(200.0, false, false);
        assert!(weak < strong);
        // Clamps: 30 hp → 0.4×, a 250 overheal stack → 2.5× (no further).
        assert_eq!(target_bias(30.0, false, false), 0.4);
        assert_eq!(target_bias(250.0, false, false), 2.5);
        // A big-armed target is nudged preferred (×0.8) only when weapons don't stay (killing them
        // resets the kit); under weapons-stay there's nothing to deny, so no nudge.
        assert_eq!(target_bias(100.0, true, false), 0.8);
        assert_eq!(target_bias(100.0, true, true), 1.0);
    }
}
