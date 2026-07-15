// SPDX-License-Identifier: AGPL-3.0-or-later

//! The senses a client has instead of a server's certainty.
//!
//! A bot inside the server reads an opponent's health out of their entity. A client can't: no stat
//! carries it, and none ever will. What a client gets instead is what a player gets — it sees them,
//! it hears them fire, it feels itself get hit, it reads the obituary. This module turns each of
//! those into the fields the brain already reads, so the same brain reaches the same kind of
//! conclusion from the same kind of evidence.
//!
//! That isn't a workaround for a missing feature. It's the reason these bots can play against people
//! at all: a bot that knew your health through a wall would be a cheat with good manners, and one
//! that has to work it out from a sound and a scoreboard is just a player.
//!
//! Each channel below writes somewhere the brain is already looking:
//!
//! - **See** — nothing to do. The server only sends entities we could see, so the mirror's world is
//!   PVS-culled before we touch it; it's stricter than the server-side bot's, not looser.
//! - **Hear** — `perceive` treats an enemy whose `combat.attack_finished` is still running and who
//!   is within earshot as *heard*. So a fire sound on the wire sets exactly that field, and the hear
//!   channel lights up with no change to perception at all.
//! - **Feel** — `svc_damage` says how hard and roughly from where. That's the same stamp
//!   `T_Damage` leaves on a server-side bot: a bearing, not a position.
//! - **Read** — the obituary tells us who died, which is the one moment an estimate is *known*
//!   rather than guessed: a dead player is a fresh spawn.

use glam::Vec3;

use crate::arsenal;
use crate::bot::model::{weapon_fire_bit, PickupKind};
use crate::bot::perception::{heard_hypothesis, MEMORY};
use crate::defs::Items;
use crate::entity::EntId;
use crate::game::GameState;

/// A weapon-fire sound, and which weapon made it.
///
/// This is the bot's ears. QuakeWorld sends sounds by PHS rather than PVS — you hear things through
/// walls, which is the whole point of listening — so this reaches further than sight and is exactly
/// the information a player is working from when they say "he's got the rocket launcher".
const FIRE_SOUNDS: &[(&str, Items)] = &[
    ("weapons/guncock.wav", Items::SHOTGUN),
    ("weapons/shotgn2.wav", Items::SUPER_SHOTGUN),
    ("weapons/rocket1i.wav", Items::NAILGUN),
    ("weapons/spike2.wav", Items::SUPER_NAILGUN),
    ("weapons/grenade.wav", Items::GRENADE_LAUNCHER),
    ("weapons/sgun1.wav", Items::ROCKET_LAUNCHER),
    ("weapons/lstart.wav", Items::LIGHTNING),
    ("weapons/lhit.wav", Items::LIGHTNING),
    ("weapons/ax1.wav", Items::AXE),
];

/// The weapon whose fire sound this is, if any.
pub(crate) fn fire_sound(name: &str) -> Option<Items> {
    FIRE_SOUNDS.iter().find(|(n, _)| *n == name).map(|(_, w)| *w)
}

/// How long a weapon takes to re-arm, from the table the server fires by.
fn cooldown_of(item: Items) -> f32 {
    arsenal::weapon_spec(item).map(|s| s.cooldown).unwrap_or(0.5)
}

impl GameState {
    /// Someone fired, and we heard it.
    ///
    /// Two things follow, and they're the two a player draws from the same sound. The shooter is
    /// **audible** — `perceive` reads a running `attack_finished` within earshot as heard, so
    /// stamping it is all the hear channel needs. And they are **armed with that weapon**, which is
    /// worth more than it sounds: it's how a bot learns the enemy has the rocket launcher without
    /// ever seeing them hold it.
    pub(crate) fn client_heard_fire(&mut self, shooter: EntId, weapon: Items, _where: Vec3) {
        if !self.entities[shooter].is_player() {
            return;
        }
        let now = self.time();
        self.entities[shooter].combat.attack_finished = now + cooldown_of(weapon);
        // Whoever could have heard it now knows they're carrying it. `model_note_weapon_fire` takes
        // the weapon from the firer's `.weapon` field, which a client doesn't have for other
        // players — so the sound *is* the weapon, and it's noted directly.
        let pos = self.entities[shooter].v.origin;
        for pool in crate::bot::model::iter_pools(self.witness_pools(pos)) {
            self.opponents.note_weapon(pool, shooter, weapon, now);
        }
    }

    /// We got hit, and roughly from over there.
    ///
    /// The same stamp `T_Damage` leaves on a server-side bot, from the same kind of information: a
    /// bearing, not a position. `svc_damage` carries where the hit came from, so the bot turns
    /// toward a *hypothesis* along that line — it knows it's being shot at and roughly from where,
    /// which is what being shot tells you.
    pub(crate) fn client_felt_damage(&mut self, victim: EntId, from: Vec3, armor: f32, blood: f32) {
        let total = armor + blood;
        if total <= 0.0 {
            return;
        }
        let now = self.time();
        let victim_org = self.entities[victim].v.origin;

        // Who shot us isn't on the wire. The nearest live player along the bearing is the only
        // candidate a client has, and it's the one a player would assume too.
        let Some(attacker) = self.nearest_player_toward(victim, from) else {
            return;
        };

        let (r_lat, r_dist) = (self.random(), self.random());
        let pt = heard_hypothesis(victim_org, from, r_lat, r_dist);
        let b = &mut self.entities[victim].bot;
        b.percept.known_enemy = attacker.0;
        b.percept.known_until = now + MEMORY;
        b.percept.last_seen = pt;

        // And nothing about *their* condition. Being shot tells you someone is there and roughly
        // where; it says nothing about how much fight they have left. (`model_note_damage` models
        // the attacker's side learning the *victim* is hurt — the enemy's beliefs, which our bots
        // have no business acting on.) The one thing it does reveal is that they're armed, and the
        // hear channel already has that from the shot itself.
        let _ = total;
    }

    /// Someone died, which is the one moment an estimate stops being a guess.
    ///
    /// A dead player respawns with exactly a spawn's worth of health and a shotgun. Every hypothesis
    /// about them — how hurt they were, what they were carrying — is void, and the truth is known.
    pub(crate) fn client_saw_death(&mut self, victim: EntId) {
        let now = self.time();
        self.opponents.reset_target(victim, now);
    }

    /// Someone picked up something worth noting — a quad's hum, a pentagram's glow.
    pub(crate) fn client_saw_pickup(&mut self, picker: EntId, kind: PickupKind) {
        self.model_note_pickup(picker, kind);
    }

    /// The live player most nearly in the direction a hit came from.
    ///
    /// `svc_damage`'s `from` is where the *damage* came from — the rocket's blast centre, the
    /// shooter's gun — not a player id. Picking the best candidate along that bearing is the same
    /// inference a player makes, and being wrong costs a bot a glance in the wrong direction.
    fn nearest_player_toward(&self, victim: EntId, from: Vec3) -> Option<EntId> {
        let maxclients = self.host.cvar(c"maxclients") as u32;
        let mut best: Option<(f32, EntId)> = None;
        for i in 1..=maxclients {
            let p = EntId(i);
            if p == victim || !self.entities[p].in_use || !self.entities[p].is_alive() {
                continue;
            }
            let d = self.entities[p].v.origin.distance(from);
            if best.is_none_or(|(bd, _)| d < bd) {
                best = Some((d, p));
            }
        }
        best.map(|(_, p)| p)
    }

    /// Write what we believe about every opponent into the fields the brain reads them from.
    ///
    /// The brain's own strength/posture logic already goes through `opponent_est`, so this is mostly
    /// belt and braces for the paths that read an entity directly. What matters is what it does
    /// *not* do: `health` here is an estimate wearing health's clothes, and nothing about it is
    /// authoritative. Death isn't sourced from here — that's `PF_DEAD`, on the wire, and true.
    pub(crate) fn client_write_enemy_estimates(&mut self, observer: EntId) {
        let now = self.time();
        let maxclients = self.host.cvar(c"maxclients") as u32;
        for i in 1..=maxclients {
            let p = EntId(i);
            if p == observer || !self.entities[p].in_use || !self.entities[p].is_player() {
                continue;
            }
            // A dead body's condition isn't in question.
            if !self.entities[p].is_alive() {
                continue;
            }
            let Some(est) = self.opponent_est(observer, p, now) else {
                continue;
            };
            let v = &mut self.entities[p].v;
            v.health = est.health.max(1.0);
            v.armorvalue = est.armor_value;
            v.armortype = est.armor_type;
            v.items = est.items;
        }
    }

    /// Our own weapon re-arm, tracked because there's nothing else to track it with.
    ///
    /// The server owns `attack_finished` and never sends it. But we know what we fired and when we
    /// pressed the button, and the delay is the same table the server fires by — so the bot's own
    /// idea of "am I ready" stays in step with the server's without being told. Reading it wrong
    /// means a bot that either waits too long or squeezes at nothing.
    pub(crate) fn client_note_own_fire(&mut self, e: EntId) {
        let now = self.time();
        let weapon = self.entities[e].v.weapon;
        let Some(bit) = weapon_fire_bit(weapon) else {
            return;
        };
        let ent = &mut self.entities[e];
        if ent.combat.attack_finished > now {
            return; // already re-arming; the button being held isn't a new shot
        }
        ent.combat.attack_finished = now + cooldown_of(bit);
    }

    /// The powerup windows, from the item bits going up.
    ///
    /// A powerup's *countdown* isn't on the wire — only the fact that we have one, in `STAT_ITEMS`.
    /// The moment the bit appears is the moment it started, and 30 seconds is what the server gives.
    /// The brain reads these to decide whether to press a fight, so being roughly right matters and
    /// being precise doesn't.
    pub(crate) fn client_note_own_powerups(&mut self, e: EntId, before: Items, after: Items) {
        let now = self.time();
        let gained = after & !before;
        let c = &mut self.entities[e].combat;
        if gained.contains(Items::QUAD) {
            c.super_damage_finished = now + POWERUP_TIME;
        }
        if gained.contains(Items::INVULNERABILITY) {
            c.invincible_finished = now + POWERUP_TIME;
        }
        if gained.contains(Items::SUIT) {
            c.radsuit_finished = now + POWERUP_TIME;
        }
        if gained.contains(Items::INVISIBILITY) {
            c.invisible_finished = now + POWERUP_TIME;
        }
        // And when a bit goes away, it's over — whatever we'd predicted.
        let lost = before & !after;
        if lost.contains(Items::QUAD) {
            c.super_damage_finished = 0.0;
        }
        if lost.contains(Items::INVULNERABILITY) {
            c.invincible_finished = 0.0;
        }
    }
}

/// How long a powerup lasts. The server's number; a client only sees the bit appear.
const POWERUP_TIME: f32 = 30.0;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::defs::{Bits, Weapon};
    use crate::netclient::host::NetHost;
    use std::path::PathBuf;

    fn game() -> GameState {
        let host: &'static NetHost = Box::leak(Box::new(NetHost::new(PathBuf::from("/nonexistent"))));
        let mut g = GameState::new_client(host);
        host.set("maxclients", "8");
        host.set("rtx_bot_model", "1");
        g.globals.time = 100.0;
        g
    }

    fn player(g: &mut GameState, slot: u32, at: Vec3) -> EntId {
        let e = EntId(slot);
        g.entities[e].in_use = true;
        g.entities[e].classname = Some("player".into());
        g.entities[e].v.health = 100.0;
        g.entities[e].v.origin = at;
        e
    }

    /// The sound table is the bot's ears: each of these is a weapon a player identifies by ear, and
    /// getting one wrong means a bot that thinks a shotgun is a rocket launcher.
    #[test]
    fn recognises_weapon_fire_by_sound() {
        assert_eq!(fire_sound("weapons/sgun1.wav"), Some(Items::ROCKET_LAUNCHER));
        assert_eq!(fire_sound("weapons/shotgn2.wav"), Some(Items::SUPER_SHOTGUN));
        assert_eq!(fire_sound("weapons/lstart.wav"), Some(Items::LIGHTNING));
        assert_eq!(fire_sound("weapons/grenade.wav"), Some(Items::GRENADE_LAUNCHER));

        // Not every sound is a shot — an item respawning isn't someone shooting at you.
        assert_eq!(fire_sound("items/damage.wav"), None);
        assert_eq!(fire_sound("player/land.wav"), None);
        assert_eq!(fire_sound(""), None);
    }

    /// Hearing a shot does exactly what `perceive` is already looking for: a running
    /// `attack_finished` on someone within earshot *is* the hear channel. No change to perception,
    /// which is the point.
    #[test]
    fn hearing_a_shot_makes_the_shooter_audible() {
        let mut g = game();
        let shooter = player(&mut g, 2, Vec3::new(500.0, 0.0, 0.0));

        g.client_heard_fire(shooter, Items::ROCKET_LAUNCHER, Vec3::new(500.0, 0.0, 0.0));

        // `perceive` reads exactly this to decide it heard something.
        assert!(g.entities[shooter].combat.attack_finished > g.time());
        assert_eq!(g.entities[shooter].combat.attack_finished, 100.0 + 0.8, "the RL's own cooldown");
    }

    /// And it teaches the estimate what they're carrying — which is how a bot learns the enemy has
    /// the rocket launcher without ever seeing them hold it.
    #[test]
    fn hearing_a_shot_reveals_the_weapon() {
        let mut g = game();
        let me = player(&mut g, 1, Vec3::ZERO);
        let shooter = player(&mut g, 2, Vec3::new(300.0, 0.0, 0.0));
        g.entities[me].bot.is_bot = true;

        let est = g.opponent_est(me, shooter, g.time()).expect("model on");
        assert!(!est.items.has(Items::ROCKET_LAUNCHER), "not yet known");

        g.client_heard_fire(shooter, Items::ROCKET_LAUNCHER, Vec3::new(300.0, 0.0, 0.0));
        let est = g.opponent_est(me, shooter, g.time()).expect("model on");
        assert!(est.items.has(Items::ROCKET_LAUNCHER), "heard it, so they have it");
    }

    /// Being shot tells you a direction, not a position — so the bot turns toward a hypothesis along
    /// the bearing, exactly as a server-side bot does when `T_Damage` stamps it.
    #[test]
    fn being_hit_reveals_a_bearing_not_a_position() {
        let mut g = game();
        let me = player(&mut g, 1, Vec3::ZERO);
        let them = player(&mut g, 2, Vec3::new(1000.0, 0.0, 0.0));
        g.entities[me].bot.is_bot = true;

        let from = Vec3::new(1000.0, 0.0, 0.0);
        g.client_felt_damage(me, from, 10.0, 25.0);

        let p = &g.entities[me].bot.percept;
        assert_eq!(p.known_enemy, them.0, "the only candidate along that bearing");
        assert!(p.known_until > g.time());
        assert_ne!(p.last_seen, from, "a hypothesis, not the true origin");
        assert!(p.last_seen.length() > 0.0);

        // And it tells us nothing about *their* condition — being shot says someone is there, not
        // how much fight they have left.
        let est = g.opponent_est(me, them, g.time()).expect("model on");
        assert_eq!(est.health, 100.0, "no claim about the shooter from having been shot");
    }

    /// Damage that did nothing says nothing.
    #[test]
    fn a_harmless_hit_stamps_nothing() {
        let mut g = game();
        let me = player(&mut g, 1, Vec3::ZERO);
        player(&mut g, 2, Vec3::new(100.0, 0.0, 0.0));

        g.client_felt_damage(me, Vec3::new(100.0, 0.0, 0.0), 0.0, 0.0);
        assert_eq!(g.entities[me].bot.percept.known_enemy, 0);
    }

    /// A death is the one moment an estimate is *known*: they're about to be a fresh spawn, so every
    /// hypothesis about how hurt they were is void.
    #[test]
    fn a_death_resets_what_we_believed() {
        let mut g = game();
        let me = player(&mut g, 1, Vec3::ZERO);
        let them = player(&mut g, 2, Vec3::new(300.0, 0.0, 0.0));
        g.entities[me].bot.is_bot = true;

        g.client_heard_fire(them, Items::ROCKET_LAUNCHER, Vec3::new(300.0, 0.0, 0.0));
        g.model_note_damage(me, them, 80.0);
        let est = g.opponent_est(me, them, g.time()).expect("model on");
        assert!(est.health < 100.0);

        g.client_saw_death(them);
        let est = g.opponent_est(me, them, g.time()).expect("model on");
        assert!(est.health >= 100.0, "a fresh spawn, and we know it");
    }

    /// Our own refire is tracked because nothing tells us: the server owns `attack_finished` and
    /// never sends it. Reading it wrong is a bot that waits too long, or squeezes at nothing.
    #[test]
    fn our_own_refire_follows_the_weapon_we_fired() {
        let mut g = game();
        let me = player(&mut g, 1, Vec3::ZERO);
        g.entities[me].v.weapon = Weapon::RocketLauncher;

        g.client_note_own_fire(me);
        assert_eq!(g.entities[me].combat.attack_finished, 100.0 + 0.8);

        // Holding the button down isn't a second shot — the cooldown mustn't keep resetting.
        g.globals.time = 100.4;
        g.client_note_own_fire(me);
        assert_eq!(g.entities[me].combat.attack_finished, 100.8, "still the first shot's window");

        // Once it's elapsed, the next shot starts a new one.
        g.globals.time = 101.0;
        g.client_note_own_fire(me);
        assert_eq!(g.entities[me].combat.attack_finished, 101.8);
    }

    /// A powerup's countdown isn't on the wire — only the bit. The moment it appears is the moment
    /// it started; the moment it goes, it's over.
    #[test]
    fn powerup_windows_start_when_the_bit_appears() {
        let mut g = game();
        let me = player(&mut g, 1, Vec3::ZERO);

        g.client_note_own_powerups(me, Items::empty(), Items::QUAD);
        assert_eq!(g.entities[me].combat.super_damage_finished, 130.0);

        // Holding it doesn't restart it.
        g.globals.time = 110.0;
        g.client_note_own_powerups(me, Items::QUAD, Items::QUAD);
        assert_eq!(g.entities[me].combat.super_damage_finished, 130.0);

        // Losing the bit ends it, whatever we'd predicted — the server is the one that decides.
        g.client_note_own_powerups(me, Items::QUAD, Items::empty());
        assert_eq!(g.entities[me].combat.super_damage_finished, 0.0);
    }

    /// The estimates are written where a stray direct read would find them, but they are estimates:
    /// death comes from the wire, never from here.
    #[test]
    fn enemy_estimates_are_written_but_never_claim_death() {
        let mut g = game();
        let me = player(&mut g, 1, Vec3::ZERO);
        let them = player(&mut g, 2, Vec3::new(300.0, 0.0, 0.0));
        g.entities[me].bot.is_bot = true;

        g.model_note_damage(me, them, 60.0);
        g.client_write_enemy_estimates(me);
        assert!(g.entities[them].v.health < 100.0, "what we believe, in health's clothes");
        assert!(g.entities[them].is_alive(), "an estimate must never kill anyone");

        // A body the wire says is dead is left alone — that fact isn't ours to revise.
        g.entities[them].v.health = 0.0;
        g.entities[them].v.deadflag = crate::defs::DeadFlag::Dead;
        g.client_write_enemy_estimates(me);
        assert_eq!(g.entities[them].v.health, 0.0, "the wire said dead; we don't argue");
    }
}
