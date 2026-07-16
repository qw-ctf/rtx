// SPDX-License-Identifier: AGPL-3.0-or-later

//! Weapon firing and projectiles, ported from `qw-qc/weapons.qc`.
//!
//! Single-shot weapons fire directly from [`GameState::w_attack`]; the nailgun and lightning
//! gun fire from their looping animation think-chains (see `player.rs`). Projectiles carry a
//! [`Touch`] behaviour and (for grenades/rockets) a timed [`Think`].

use core::ffi::CStr;

use glam::Vec3;

use crate::arsenal::AmmoKind;
use crate::assets::{Model, Sound};
use crate::defs::*;
use crate::entity::{Die, EntId, Think, Touch};
use crate::game::GameState;
use crate::obituary::DeathType;

mod select;
mod shootable_grenade;
mod hitscan;
pub(crate) mod projectiles;

/// QuakeC `crandom` — a float in `[-1, 1)`.
fn crandom(game: &mut GameState) -> f32 {
    2.0 * (game.random() - 0.5)
}

const SHOOTABLE_GRENADE_HIT_RADIUS: f32 = 8.0;
const SHOOTABLE_GRENADE_MINS: Vec3 = Vec3::splat(-4.0);
const SHOOTABLE_GRENADE_MAXS: Vec3 = Vec3::splat(4.0);

/// Which nail a [`GameState::spike_touch`] is servicing — the nailgun's single spike or the
/// super-nailgun's heavier one. Replaces a positional `bool` at the touch dispatcher.
#[derive(Clone, Copy)]
pub(crate) enum SpikeKind {
    Nail,
    Super,
}

impl SpikeKind {
    /// Impact damage, obituary stamp, and wall temp-entity for this nail.
    fn effect(self) -> (f32, DeathType, Te) {
        match self {
            SpikeKind::Nail => (9.0, DeathType::Nailgun, Te::Spike),
            SpikeKind::Super => (18.0, DeathType::SuperNailgun, Te::SuperSpike),
        }
    }
}

/// `w_fire_rocket`'s muzzle point: the projectile spawn height (`origin + 16` up) plus the forward
/// nudge (`+ fwd*8`). The single source for both the real rocket spawn and the bot's line-of-fire
/// *prediction* of it (`bot::combat::fire_gate`) — computing them the same way is what keeps the
/// corner-self-splash gate honest. rtx-nav's rocket-jump pricing mirrors the same offset across the
/// crate boundary (`navmesh::rocketjump` `MUZZLE_FWD`/`MUZZLE_Z`); keep all three in step.
pub(crate) fn rocket_muzzle(origin: Vec3, fwd: Vec3) -> Vec3 {
    origin + fwd * 8.0 + Vec3::new(0.0, 0.0, 16.0)
}

impl GameState {
    /// `aim` — autoaim direction. QW deathmatch effectively disables vertical autoaim, so we
    /// return straight-ahead `v_forward` (after refreshing the angle vectors).
    pub(crate) fn aim_dir(&mut self, e: EntId) -> Vec3 {
        let v_angle = self.entities[e].v.v_angle;
        self.make_vectors(v_angle);
        self.globals.v_forward
    }

    /// `muzzleflash` — networked muzzle flash for the firing player.
    pub(crate) fn muzzleflash(&self, e: EntId) {
        let origin = self.entities[e].v.origin;
        self.host.write_svc(MsgDest::Multicast, Svc::MuzzleFlash);
        self.host.write_entity(MsgDest::Multicast, e);
        self.host.multicast(origin, Multicast::Pvs);
    }

    /// `SuperDamageSound` — periodic quad-damage hum.
    pub(crate) fn super_damage_sound(&mut self, e: EntId) {
        let time = self.time();
        let ent = &self.entities[e];
        if ent.combat.super_damage_finished > time && ent.combat.super_sound < time {
            self.entities[e].combat.super_sound = time + 1.0;
            self.host
                .sound(e, Channel::Body, Sound::ITEMS_DAMAGE3, 1.0, Attenuation::Norm);
        }
    }

    /// `SpawnBlood` — networked blood puff.
    pub(crate) fn spawn_blood(&self, org: Vec3, count: i32) {
        self.host.write_te(MsgDest::Multicast, Te::Blood);
        self.host.write_byte(MsgDest::Multicast, count);
        self.write_coords(MsgDest::Multicast, org);
        self.host.multicast(org, Multicast::Pvs);
    }


    // --- weapon selection & frame loop ---

    /// `W_WeaponFrame` — once per `PlayerPostThink`: handle impulses and trigger attacks.
    pub(crate) fn w_weapon_frame(&mut self, e: EntId) {
        if self.time() < self.entities[e].combat.attack_finished {
            return;
        }
        self.impulse_commands(e);
        if self.entities[e].v.button0 != 0.0 {
            // The active mode may lock out firing (e.g. Rocket Arena's pre-"FIGHT" countdown).
            // Weapon *switching* above still works; only the shot is withheld.
            let mode = self.mode;
            if mode.weapons_hot(self) {
                self.super_damage_sound(e);
                self.w_attack(e);
            } else {
                self.deny_fire(e);
            }
        }
    }

    /// Firing is disabled right now: blink a human's screen (throttled) so a held fire button gives
    /// feedback instead of silence. Bots are skipped (they don't hold fire pre-round anyway).
    fn deny_fire(&mut self, e: EntId) {
        if self.entities[e].bot.is_bot {
            return;
        }
        let now = self.time();
        if now < self.entities[e].mode_p.arena.flash_time {
            return;
        }
        self.entities[e].mode_p.arena.flash_time = now + 0.5;
        self.screen_flash(e);
    }

    // --- small helpers ---
    //
    // These all unicast to one client (view punch / console print). A bot is a fake client with no
    // connection, so the engine rejects a unicast aimed at one and logs "msg_entity: not a client"
    // / "Not a client" every time — skip bots up front (the effect is a client-side no-op anyway).
    // Same guard as `deny_fire` / `centerprint_all`.

    /// `Svc::SmallKick` view punch to a single client (`msg_entity = e; WriteByte MsgDest::One`).
    pub(crate) fn small_kick(&mut self, e: EntId) {
        if self.entities[e].bot.is_bot {
            return;
        }
        self.globals.msg_entity = e.to_prog();
        self.host.write_svc(MsgDest::One, Svc::SmallKick);
    }

    /// `Svc::BigKick` view punch (super shotgun).
    fn big_kick(&mut self, e: EntId) {
        if self.entities[e].bot.is_bot {
            return;
        }
        self.globals.msg_entity = e.to_prog();
        self.host.write_svc(MsgDest::One, Svc::BigKick);
    }

    /// `sprint(self, PrintLevel::High, ...)` to a player.
    pub(crate) fn sprint_to(&self, e: EntId, msg: &CStr) {
        if self.entities[e].bot.is_bot {
            return;
        }
        self.host.sprint(e, PrintLevel::High, msg);
    }

    /// `centerprint(e, ...)` to a single player (single-recipient sibling of `centerprint_all`).
    pub(crate) fn centerprint_to(&self, e: EntId, msg: &CStr) {
        if self.entities[e].bot.is_bot {
            return;
        }
        self.host.centerprint(e, msg);
    }
}
