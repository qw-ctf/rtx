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

/// `vectoangles` — convert a direction to `(pitch, yaw, 0)` Euler angles (degrees).
pub(crate) fn vectoangles(v: Vec3) -> Vec3 {
    if v.x == 0.0 && v.y == 0.0 {
        let pitch = if v.z > 0.0 { 90.0 } else { 270.0 };
        return Vec3::new(pitch, 0.0, 0.0);
    }
    let mut yaw = v.y.atan2(v.x).to_degrees();
    if yaw < 0.0 {
        yaw += 360.0;
    }
    let forward = (v.x * v.x + v.y * v.y).sqrt();
    let mut pitch = v.z.atan2(forward).to_degrees();
    if pitch < 0.0 {
        pitch += 360.0;
    }
    Vec3::new(pitch, yaw, 0.0)
}

impl GameState {
    fn shootable_grenades_enabled(&self) -> bool {
        self.host.cvar_bool(c"rtx_shootable_grenades")
    }

    pub(crate) fn is_grenade(&self, e: EntId) -> bool {
        self.entities[e].classname() == Some("grenade")
    }

    pub(crate) fn is_live_shootable_grenade(&self, e: EntId) -> bool {
        self.shootable_grenades_enabled()
            && self.is_grenade(e)
            && self.entities[e].in_use
            && self.entities[e].combat.voided == 0.0
    }

    pub(crate) fn try_detonate_shootable_grenade(&mut self, e: EntId) -> bool {
        if !self.is_live_shootable_grenade(e) {
            return false;
        }
        self.grenade_explode(e);
        true
    }

    fn shootable_grenade_on_line(&self, start: Vec3, end: Vec3, max_fraction: f32) -> Option<EntId> {
        // Engine traces can skip entities owned by the ignored player. Grenades keep their
        // owner for collision filtering and damage credit, so hitscan weapons need this
        // explicit line check to support shooting your own grenade.
        let dir = end - start;
        let len2 = dir.length_squared();
        if len2 == 0.0 {
            return None;
        }

        let mut best = max_fraction;
        let mut hit = None;
        for (i, ent) in self.entities.iter().enumerate() {
            if ent.classname() != Some("grenade") || !ent.in_use || ent.combat.voided != 0.0 {
                continue;
            }
            let t = (ent.v.origin - start).dot(dir) / len2;
            if !(0.0..best).contains(&t) {
                continue;
            }
            let closest = start + dir * t;
            if (ent.v.origin - closest).length_squared() <= SHOOTABLE_GRENADE_HIT_RADIUS * SHOOTABLE_GRENADE_HIT_RADIUS
            {
                best = t;
                hit = Some(EntId(i as u32));
            }
        }
        hit
    }

    fn try_detonate_shootable_grenade_on_line(&mut self, start: Vec3, end: Vec3, max_fraction: f32) -> bool {
        if !self.shootable_grenades_enabled() {
            return false;
        }
        let Some(grenade) = self.shootable_grenade_on_line(start, end, max_fraction) else {
            return false;
        };
        self.grenade_explode(grenade);
        true
    }

    /// `aim` — autoaim direction. QW deathmatch effectively disables vertical autoaim, so we
    /// return straight-ahead `v_forward` (after refreshing the angle vectors).
    pub(crate) fn aim_dir(&mut self, e: EntId) -> Vec3 {
        let v_angle = self.entities[e].v.v_angle;
        self.host.make_vectors(v_angle);
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

    /// Write a vector as three `WriteCoord`s.
    fn write_coords(&self, to: MsgDest, v: Vec3) {
        self.host.write_coord(to, v.x);
        self.host.write_coord(to, v.y);
        self.host.write_coord(to, v.z);
    }

    /// Broadcast a payload-less point temp-entity (an explosion, teleport fog, …) to the PHS: the
    /// `write_te` → coords → `multicast` ritual every such effect hand-rolls.
    pub(crate) fn temp_entity_point(&self, te: Te, org: Vec3) {
        self.host.write_te(MsgDest::Multicast, te);
        self.write_coords(MsgDest::Multicast, org);
        self.host.multicast(org, Multicast::Phs);
    }

    // --- axe ---

    /// `W_FireAxe` — short melee trace.
    pub(crate) fn w_fire_axe(&mut self, e: EntId) {
        let v_angle = self.entities[e].v.v_angle;
        self.host.make_vectors(v_angle);
        let v_forward = self.globals.v_forward;
        let source = self.entities[e].v.origin + Vec3::new(0.0, 0.0, 16.0);
        let end = source + v_forward * 64.0;
        let tr = self.traceline(source, end, false, e);
        if self.try_detonate_shootable_grenade_on_line(source, end, tr.fraction) {
            return;
        }
        if tr.fraction == 1.0 {
            return;
        }
        let org = tr.endpos - v_forward * 4.0;
        if self.try_detonate_shootable_grenade(tr.ent) {
            return;
        }
        if self.entities[tr.ent].v.takedamage != TakeDamage::No {
            self.entities[tr.ent].combat.axhitme = 1.0;
            self.spawn_blood(org, 20);
            let dmg = if self.level.deathmatch > 3 { 75.0 } else { 20.0 };
            self.entities[tr.ent].deathtype = DeathType::Axe;
            self.t_damage(tr.ent, e, e, dmg);
        } else {
            self.host
                .sound(e, Channel::Weapon, Sound::PLAYER_AXHIT2, 1.0, Attenuation::Norm);
            self.host.write_te(MsgDest::Multicast, Te::Gunshot);
            self.host.write_byte(MsgDest::Multicast, 3);
            self.write_coords(MsgDest::Multicast, org);
            self.host.multicast(org, Multicast::Pvs);
        }
    }

    // --- bullets (shotgun family) ---

    /// `FireBullets` (+ `TraceAttack`/multi-damage) — fire `shotcount` pellets in a cone,
    /// combining hits on the same target into one `T_Damage` call.
    fn fire_bullets(&mut self, e: EntId, shotcount: i32, dir: Vec3, spread: Vec3, dtype: DeathType) {
        let v_angle = self.entities[e].v.v_angle;
        self.host.make_vectors(v_angle);
        let v_right = self.globals.v_right;
        let v_up = self.globals.v_up;
        let v_forward = self.globals.v_forward;

        let (origin, absmin_z, size_z) = {
            let v = &self.entities[e].v;
            (v.origin, v.absmin.z, v.size.z)
        };
        let mut src = origin + v_forward * 10.0;
        src.z = absmin_z + size_z * 0.7;

        let center_end = src + dir * 2048.0;
        let tr0 = self.traceline(src, center_end, false, e);
        if self.try_detonate_shootable_grenade_on_line(src, center_end, tr0.fraction) {
            return;
        }
        if self.try_detonate_shootable_grenade(tr0.ent) {
            return;
        }
        let puff_org = tr0.endpos - dir * 4.0;
        let mut puff_count = 0;
        let mut blood_count = 0;
        let mut blood_org = Vec3::ZERO;

        // Multi-damage accumulation (single combined T_Damage per struck entity).
        let mut multi_ent = EntId::WORLD;
        let mut multi_damage = 0.0f32;

        for _ in 0..shotcount {
            let direction = dir + crandom(self) * spread.x * v_right + crandom(self) * spread.y * v_up;
            let end = src + direction * 2048.0;
            let tr = self.traceline(src, end, false, e);
            if self.try_detonate_shootable_grenade_on_line(src, end, tr.fraction) {
                continue;
            }
            if tr.fraction == 1.0 {
                continue;
            }
            let org = tr.endpos - direction * 4.0;
            if self.try_detonate_shootable_grenade(tr.ent) {
                continue;
            }
            if self.entities[tr.ent].v.takedamage != TakeDamage::No {
                blood_count += 1;
                blood_org = org;
                if tr.ent != multi_ent {
                    if multi_ent != EntId::WORLD {
                        self.entities[multi_ent].deathtype = dtype;
                        self.t_damage(multi_ent, e, e, multi_damage);
                    }
                    multi_damage = 4.0;
                    multi_ent = tr.ent;
                } else {
                    multi_damage += 4.0;
                }
            } else {
                puff_count += 1;
            }
        }
        if multi_ent != EntId::WORLD {
            self.entities[multi_ent].deathtype = dtype;
            self.t_damage(multi_ent, e, e, multi_damage);
        }

        // Multi_Finish: networked gunshot puffs and blood.
        if puff_count != 0 {
            self.host.write_te(MsgDest::Multicast, Te::Gunshot);
            self.host.write_byte(MsgDest::Multicast, puff_count);
            self.write_coords(MsgDest::Multicast, puff_org);
            self.host.multicast(puff_org, Multicast::Pvs);
        }
        if blood_count != 0 {
            self.host.write_te(MsgDest::Multicast, Te::Blood);
            self.host.write_byte(MsgDest::Multicast, blood_count);
            self.write_coords(MsgDest::Multicast, blood_org);
            self.host.multicast(puff_org, Multicast::Pvs);
        }
    }

    /// `W_FireShotgun`.
    fn w_fire_shotgun(&mut self, e: EntId) {
        self.host
            .sound(e, Channel::Weapon, Sound::WEAPONS_GUNCOCK, 1.0, Attenuation::Norm);
        self.small_kick(e);
        self.consume_ammo(e, AmmoKind::Shells, 1.0);
        let dir = self.aim_dir(e);
        self.fire_bullets(e, 6, dir, Vec3::new(0.04, 0.04, 0.0), DeathType::Shotgun);
    }

    /// `W_FireSuperShotgun`.
    fn w_fire_super_shotgun(&mut self, e: EntId) {
        if self.entities[e].v.currentammo == 1.0 {
            self.w_fire_shotgun(e);
            return;
        }
        self.host
            .sound(e, Channel::Weapon, Sound::WEAPONS_SHOTGN2, 1.0, Attenuation::Norm);
        self.big_kick(e);
        self.consume_ammo(e, AmmoKind::Shells, 2.0);
        let dir = self.aim_dir(e);
        self.fire_bullets(e, 14, dir, Vec3::new(0.14, 0.08, 0.0), DeathType::SuperShotgun);
    }

    // --- rockets ---

    /// `T_MissileTouch` — rocket impact.
    pub(crate) fn t_missile_touch(&mut self, e: EntId, other: EntId) {
        if other == self.entities[e].owner() {
            return;
        }
        if self.entities[e].combat.voided != 0.0 {
            return;
        }
        self.entities[e].combat.voided = 1.0;

        let origin = self.entities[e].v.origin;
        if self.host.pointcontents(origin).is(Content::Sky) {
            self.free(e);
            return;
        }

        let owner = self.entities[e].owner();
        let damg = 100.0 + self.random() * 20.0;
        if self.try_detonate_shootable_grenade(other) {
            // The rocket still explodes below; the grenade detonation keeps its own owner credit.
        } else if self.entities[other].v.health != 0.0 {
            self.entities[other].deathtype = DeathType::Rocket;
            self.t_damage(other, e, owner, damg);
        }
        self.t_radius_damage(e, owner, 120.0, other, DeathType::Rocket);

        let velocity = self.entities[e].v.velocity;
        let org = self.entities[e].v.origin - velocity.normalize_or_zero() * 8.0;
        self.entities[e].v.origin = org;
        self.temp_entity_point(Te::Explosion, org);
        self.free(e);
    }

    /// `W_FireRocket`.
    fn w_fire_rocket(&mut self, e: EntId) {
        self.consume_ammo(e, AmmoKind::Rockets, 1.0);
        self.host
            .sound(e, Channel::Weapon, Sound::WEAPONS_SGUN1, 1.0, Attenuation::Norm);
        self.small_kick(e);

        let origin = self.entities[e].v.origin;
        let dir = self.aim_dir(e); // also refreshes v_forward
        let v_forward = self.globals.v_forward;

        let m = self.spawn();
        {
            let mis = &mut self.entities[m];
            mis.set_owner(e);
            mis.v.movetype = MoveType::FlyMissile;
            mis.v.solid = Solid::BBox;
            mis.v.velocity = dir * 1000.0;
            mis.v.angles = vectoangles(mis.v.velocity);
            mis.set_touch(Touch::Missile);
            mis.combat.voided = 0.0;
            mis.v.nextthink = self.globals.time + 5.0;
            mis.think = Think::SubRemove;
            mis.classname = Some("rocket".into());
        }
        self.host.set_model(m, Model::PROGS_MISSILE);
        self.host.set_size(m, Vec3::ZERO, Vec3::ZERO);
        self.host
            .set_origin(m, origin + v_forward * 8.0 + Vec3::new(0.0, 0.0, 16.0));
        // Stamp the shooter's firing origin (set after `set_origin` so it isn't clobbered): the
        // midair mode scores airshots by the vertical shooter→victim distance. Unused otherwise —
        // `FlyMissile` physics never touch `oldorigin`.
        self.entities[m].v.oldorigin = origin;
    }

    // --- lightning ---

    fn lightning_hit(&mut self, from: EntId, damage: f32) {
        let endpos = self.globals.trace_endpos;
        let trace_ent = EntId::from_prog(self.globals.trace_ent);
        self.host.write_te(MsgDest::Multicast, Te::LightningBlood);
        self.write_coords(MsgDest::Multicast, endpos);
        self.host.multicast(endpos, Multicast::Pvs);
        self.entities[trace_ent].deathtype = DeathType::LightningBeam;
        self.t_damage(trace_ent, from, from, damage);
    }

    /// `LightningDamage` — three parallel traces along the bolt.
    fn lightning_damage(&mut self, p1: Vec3, p2: Vec3, from: EntId, damage: f32) {
        let mut f = (p2 - p1).normalize_or_zero();
        f = Vec3::new(-f.y, f.x, 0.0) * 16.0;

        let tr = self.traceline(p1, p2, false, from);
        let e1 = tr.ent;
        if self.try_detonate_shootable_grenade_on_line(p1, p2, tr.fraction) {
            return;
        }
        if self.try_detonate_shootable_grenade(e1) {
            return;
        }
        if self.entities[e1].v.takedamage != TakeDamage::No {
            self.lightning_hit(from, damage);
        }

        let start = p1 + f;
        let end = p2 + f;
        let tr = self.traceline(start, end, false, from);
        let e2 = tr.ent;
        if self.try_detonate_shootable_grenade_on_line(start, end, tr.fraction) {
            return;
        }
        if self.try_detonate_shootable_grenade(e2) {
            return;
        }
        if e2 != e1 && self.entities[e2].v.takedamage != TakeDamage::No {
            self.lightning_hit(from, damage);
        }

        let start = p1 - f;
        let end = p2 - f;
        let tr = self.traceline(start, end, false, from);
        let e3 = tr.ent;
        if self.try_detonate_shootable_grenade_on_line(start, end, tr.fraction) {
            return;
        }
        if self.try_detonate_shootable_grenade(e3) {
            return;
        }
        if e3 != e1 && e3 != e2 && self.entities[e3].v.takedamage != TakeDamage::No {
            self.lightning_hit(from, damage);
        }
    }

    /// `W_FireLightning` — one bolt; underwater discharge dumps all cells as radius damage.
    pub(crate) fn w_fire_lightning(&mut self, e: EntId) {
        let time = self.time();
        if self.entities[e].v.ammo_cells < 1.0 {
            let best = self.w_best_weapon(e);
            self.entities[e].v.weapon = best;
            self.w_set_current_ammo(e);
            return;
        }

        // Underwater discharge.
        if self.entities[e].v.waterlevel > 1.0 {
            let cells = self.entities[e].v.ammo_cells;
            self.entities[e].v.ammo_cells = 0.0;
            self.w_set_current_ammo(e);
            self.t_radius_damage(e, e, 35.0 * cells, EntId::WORLD, DeathType::Discharge);
            return;
        }

        if self.entities[e].mover.t_width < time {
            self.host
                .sound(e, Channel::Weapon, Sound::WEAPONS_LHIT, 1.0, Attenuation::Norm);
            self.entities[e].mover.t_width = time + 0.6;
        }
        self.small_kick(e);
        self.consume_ammo(e, AmmoKind::Cells, 1.0);

        let v_angle = self.entities[e].v.v_angle;
        self.host.make_vectors(v_angle);
        let v_forward = self.globals.v_forward;
        let org = self.entities[e].v.origin + Vec3::new(0.0, 0.0, 16.0);
        let tr = self.traceline(org, org + v_forward * 600.0, true, e);
        let endpos = tr.endpos;

        self.host.write_te(MsgDest::Multicast, Te::Lightning2);
        self.host.write_entity(MsgDest::Multicast, e);
        self.write_coords(MsgDest::Multicast, org);
        self.write_coords(MsgDest::Multicast, endpos);
        self.host.multicast(org, Multicast::Phs);

        let origin = self.entities[e].v.origin;
        self.lightning_damage(origin, endpos + v_forward * 4.0, e, 30.0);
    }

    // --- grenades ---

    /// `GrenadeExplode` — timed or impact detonation.
    pub(crate) fn grenade_explode(&mut self, e: EntId) {
        if self.entities[e].combat.voided != 0.0 {
            return;
        }
        self.entities[e].combat.voided = 1.0;
        let owner = self.entities[e].owner();
        self.t_radius_damage(e, owner, 120.0, EntId::WORLD, DeathType::Grenade);

        let origin = self.entities[e].v.origin;
        self.temp_entity_point(Te::Explosion, origin);
        self.free(e);
    }

    /// `GrenadeTouch` — explode on players, else bounce.
    pub(crate) fn grenade_touch(&mut self, e: EntId, other: EntId) {
        if other == self.entities[e].owner() {
            return;
        }
        if self.entities[other].v.takedamage == TakeDamage::Aim {
            self.grenade_explode(e);
            return;
        }
        self.host
            .sound(e, Channel::Weapon, Sound::WEAPONS_BOUNCE, 1.0, Attenuation::Norm);
        if self.entities[e].v.velocity == Vec3::ZERO {
            self.entities[e].v.avelocity = Vec3::ZERO;
        }
    }

    /// `W_FireGrenade`.
    fn w_fire_grenade(&mut self, e: EntId) {
        let time = self.time();
        self.consume_ammo(e, AmmoKind::Rockets, 1.0);
        self.host
            .sound(e, Channel::Weapon, Sound::WEAPONS_GRENADE, 1.0, Attenuation::Norm);
        self.small_kick(e);

        let (origin, v_angle) = {
            let v = &self.entities[e].v;
            (v.origin, v.v_angle)
        };
        self.host.make_vectors(v_angle);
        let v_forward = self.globals.v_forward;
        let v_right = self.globals.v_right;
        let v_up = self.globals.v_up;

        let shootable = self.shootable_grenades_enabled();
        let m = self.spawn();
        let velocity = if v_angle.x != 0.0 {
            v_forward * 600.0 + v_up * 200.0 + crandom(self) * v_right * 10.0 + crandom(self) * v_up * 10.0
        } else {
            let mut vel = self.aim_dir(e) * 600.0;
            vel.z = 200.0;
            vel
        };
        {
            let mis = &mut self.entities[m];
            mis.combat.voided = 0.0;
            mis.set_owner(e);
            mis.v.movetype = MoveType::Bounce;
            mis.v.solid = Solid::BBox;
            mis.classname = Some("grenade".into());
            mis.v.velocity = velocity;
            mis.v.avelocity = Vec3::new(300.0, 300.0, 300.0);
            mis.v.angles = vectoangles(velocity);
            mis.set_touch(Touch::Grenade);
            mis.v.nextthink = time + 2.5;
            mis.think = Think::GrenadeExplode;
            if shootable {
                mis.v.takedamage = TakeDamage::Aim;
                mis.v.health = 1.0;
                mis.th_die = Die::GrenadeExplode;
            }
        }
        self.host.set_model(m, Model::PROGS_GRENADE);
        if shootable {
            self.host.set_size(m, SHOOTABLE_GRENADE_MINS, SHOOTABLE_GRENADE_MAXS);
        } else {
            self.host.set_size(m, Vec3::ZERO, Vec3::ZERO);
        }
        self.host.set_origin(m, origin);
    }

    // --- nails (spikes) ---

    /// `launch_spike` — spawn a spike travelling `dir`; returns the missile entity.
    fn launch_spike(&mut self, e: EntId, org: Vec3, dir: Vec3) -> EntId {
        let time = self.time();
        let m = self.spawn();
        {
            let mis = &mut self.entities[m];
            mis.combat.voided = 0.0;
            mis.set_owner(e);
            mis.v.movetype = MoveType::FlyMissile;
            mis.v.solid = Solid::BBox;
            mis.v.angles = vectoangles(dir);
            mis.set_touch(Touch::Spike);
            mis.classname = Some("spike".into());
            mis.think = Think::SubRemove;
            mis.v.nextthink = time + 6.0;
            mis.v.velocity = dir * 1000.0;
        }
        self.host.set_model(m, Model::PROGS_SPIKE);
        self.host.set_size(m, Vec3::ZERO, Vec3::ZERO);
        self.host.set_origin(m, org);
        m
    }

    /// `W_FireSuperSpikes`.
    fn w_fire_super_spikes(&mut self, e: EntId) {
        let time = self.time();
        self.host
            .sound(e, Channel::Weapon, Sound::WEAPONS_SPIKE2, 1.0, Attenuation::Norm);
        self.entities[e].combat.attack_finished = time + 0.2;
        self.consume_ammo(e, AmmoKind::Nails, 2.0);
        let dir = self.aim_dir(e);
        let org = self.entities[e].v.origin + Vec3::new(0.0, 0.0, 16.0);
        let m = self.launch_spike(e, org, dir);
        self.entities[m].set_touch(Touch::SuperSpike);
        self.host.set_model(m, Model::PROGS_S_SPIKE);
        self.host.set_size(m, Vec3::ZERO, Vec3::ZERO);
        self.small_kick(e);
    }

    /// `W_FireSpikes` — nailgun fire (delegates to super spikes for the SNG).
    pub(crate) fn w_fire_spikes(&mut self, e: EntId, ox: f32) {
        let time = self.time();
        let v_angle = self.entities[e].v.v_angle;
        self.host.make_vectors(v_angle);
        let v_right = self.globals.v_right;

        let (ammo_nails, weapon) = {
            let v = &self.entities[e].v;
            (v.ammo_nails, v.weapon)
        };
        if ammo_nails >= 2.0 && weapon == Weapon::SuperNailgun {
            self.w_fire_super_spikes(e);
            return;
        }
        if ammo_nails < 1.0 {
            let best = self.w_best_weapon(e);
            self.entities[e].v.weapon = best;
            self.w_set_current_ammo(e);
            return;
        }
        self.host
            .sound(e, Channel::Weapon, Sound::WEAPONS_ROCKET1I, 1.0, Attenuation::Norm);
        self.entities[e].combat.attack_finished = time + 0.2;
        self.consume_ammo(e, AmmoKind::Nails, 1.0);
        let dir = self.aim_dir(e);
        let org = self.entities[e].v.origin + Vec3::new(0.0, 0.0, 16.0) + v_right * ox;
        self.launch_spike(e, org, dir);
        self.small_kick(e);
    }

    /// `spike_touch` / `superspike_touch` — spike impact.
    pub(crate) fn spike_touch(&mut self, e: EntId, other: EntId, kind: SpikeKind) {
        if other == self.entities[e].owner() {
            return;
        }
        if self.entities[e].combat.voided != 0.0 {
            return;
        }
        self.entities[e].combat.voided = 1.0;
        if self.entities[other].v.solid == Solid::Trigger {
            return;
        }
        let origin = self.entities[e].v.origin;
        if self.host.pointcontents(origin).is(Content::Sky) {
            self.free(e);
            return;
        }

        let owner = self.entities[e].owner();
        let (damage, dtype, te) = kind.effect();

        if self.entities[other].v.takedamage != TakeDamage::No {
            if self.try_detonate_shootable_grenade(other) {
                self.free(e);
                return;
            }
            self.spawn_touchblood(e, damage);
            self.entities[other].deathtype = dtype;
            self.t_damage(other, e, owner, damage);
        } else {
            self.temp_entity_point(te, origin);
        }
        self.free(e);
    }

    /// `spawn_touchblood` — blood spray along the projectile's deflection.
    fn spawn_touchblood(&mut self, e: EntId, damage: f32) {
        let vel = self.wall_velocity(e) * 0.2;
        let origin = self.entities[e].v.origin;
        self.spawn_blood(origin + vel * 0.01, damage as i32);
    }

    /// `wall_velocity` — a deflected spray velocity off the last trace plane.
    fn wall_velocity(&mut self, e: EntId) -> Vec3 {
        let velocity = self.entities[e].v.velocity;
        let plane_normal = self.globals.trace_plane_normal;
        let r1 = self.random();
        let r2 = self.random();
        let v_up = self.globals.v_up;
        let v_right = self.globals.v_right;
        let mut vel = velocity.normalize_or_zero();
        vel = (vel + v_up * (r1 - 0.5) + v_right * (r2 - 0.5)).normalize_or_zero();
        vel += 2.0 * plane_normal;
        vel * 200.0
    }

    // --- weapon selection & frame loop ---

    /// `W_BestWeapon` — best weapon the player can currently fire. Walks the arsenal's auto-pick
    /// chain (LG → SNG → SSG → NG → SG), skipping any the player doesn't own or can't feed; the
    /// Lightning Gun is additionally gated out underwater (it discharges). Falls back to the axe.
    pub(crate) fn w_best_weapon(&self, e: EntId) -> Weapon {
        let v = &self.entities[e].v;
        for spec in crate::arsenal::auto_pick_chain() {
            if !v.items.has(spec.item) {
                continue;
            }
            if spec.item == Items::LIGHTNING && v.waterlevel > 1.0 {
                continue; // never auto-pick the LG in water — it would discharge
            }
            if let Some(kind) = spec.ammo_kind {
                if crate::arsenal::ammo_count(v, kind) < spec.min_ammo {
                    continue;
                }
            }
            return Weapon::from(spec.item);
        }
        Weapon::Axe
    }

    /// Spend `n` rounds from the `kind` pool and re-sync `currentammo` — the stock `player.qc`
    /// decrement each `w_fire_*` does, skipped in deathmatch 4 (infinite ammo).
    fn consume_ammo(&mut self, e: EntId, kind: AmmoKind, n: f32) {
        if self.level.deathmatch == 4 {
            return;
        }
        let v = &mut self.entities[e].v;
        let remaining = match kind {
            AmmoKind::Shells => {
                v.ammo_shells -= n;
                v.ammo_shells
            }
            AmmoKind::Nails => {
                v.ammo_nails -= n;
                v.ammo_nails
            }
            AmmoKind::Rockets => {
                v.ammo_rockets -= n;
                v.ammo_rockets
            }
            AmmoKind::Cells => {
                v.ammo_cells -= n;
                v.ammo_cells
            }
        };
        v.currentammo = remaining;
    }

    /// `W_CheckNoAmmo` — switch off an empty weapon; returns whether we can fire.
    fn w_check_no_ammo(&mut self, e: EntId) -> bool {
        let v = &self.entities[e].v;
        if v.currentammo > 0.0 || v.weapon == Weapon::Axe || v.weapon == Weapon::Grapple {
            return true; // axe and grapple use no ammo
        }
        let best = self.w_best_weapon(e);
        self.entities[e].v.weapon = best;
        self.w_set_current_ammo(e);
        false
    }

    /// `W_Attack` — start the active weapon's fire animation and/or fire it.
    pub(crate) fn w_attack(&mut self, e: EntId) {
        if !self.w_check_no_ammo(e) {
            return;
        }
        let time = self.time();
        let v_angle = self.entities[e].v.v_angle;
        self.host.make_vectors(v_angle);
        self.entities[e].combat.show_hostile = time + 1.0;

        // Opponent modeling: a genuine shot (ammo checked) is world-audible, so any side with a bot in
        // earshot learns which weapon the firer holds. No-op when modeling is off.
        self.model_note_weapon_fire(e);

        match self.entities[e].v.weapon {
            w if w == Weapon::Axe => {
                self.entities[e].combat.attack_finished = time + 0.5;
                self.host
                    .sound(e, Channel::Weapon, Sound::WEAPONS_AX1, 1.0, Attenuation::Norm);
                self.start_axe_anim(e);
            }
            w if w == Weapon::Shotgun => {
                self.start_shot_anim(e);
                self.entities[e].combat.attack_finished = time + 0.5;
                self.w_fire_shotgun(e);
            }
            w if w == Weapon::SuperShotgun => {
                self.start_shot_anim(e);
                self.entities[e].combat.attack_finished = time + 0.7;
                self.w_fire_super_shotgun(e);
            }
            w if w == Weapon::Nailgun || w == Weapon::SuperNailgun => {
                self.start_nail(e);
            }
            w if w == Weapon::GrenadeLauncher => {
                self.start_rocket_anim(e);
                self.entities[e].combat.attack_finished = time + 0.6;
                self.w_fire_grenade(e);
            }
            w if w == Weapon::RocketLauncher => {
                self.start_rocket_anim(e);
                self.entities[e].combat.attack_finished = time + 0.8;
                self.w_fire_rocket(e);
            }
            w if w == Weapon::Lightning => {
                self.entities[e].combat.attack_finished = time + 0.1;
                self.host
                    .sound(e, Channel::Auto, Sound::WEAPONS_LSTART, 1.0, Attenuation::Norm);
                self.start_light(e);
            }
            w if w == Weapon::Grapple => {
                self.entities[e].combat.attack_finished = time + 0.1;
                // Throws on the first press and animates the viewmodel; a no-op while out.
                self.start_grapple_throw(e);
            }
            _ => {}
        }
        // Mode cooldown scaling applied to the cooldown the weapon just set (CTF's Haste rune fires
        // ~2× as fast).
        let mode = self.mode;
        let scale = mode.attack_cooldown_scale(self, e);
        if scale != 1.0 {
            let af = self.entities[e].combat.attack_finished;
            if af > time {
                self.entities[e].combat.attack_finished = time + (af - time) * scale;
            }
        }
    }

    /// `W_ChangeWeapon` — switch to the impulse-selected weapon if owned and fed.
    fn w_change_weapon(&mut self, e: EntId) {
        // Impulse 1 toggles axe <-> grapple: from a gun it selects the axe, and a second tap (now
        // on the axe) reaches the grapple — so double-tapping "1" throws you onto the hook.
        if self.entities[e].v.impulse as i32 == 1
            && self.entities[e].v.weapon == Weapon::Axe
            && self.entities[e].v.items.has(Items::GRAPPLE)
        {
            self.select_grapple(e);
            return;
        }
        let (weapon, needs_ammo): (Items, bool) = {
            let v = &self.entities[e].v;
            match v.impulse as i32 {
                1 => (Items::AXE, false),
                2 => (Items::SHOTGUN, v.ammo_shells < 1.0),
                3 => (Items::SUPER_SHOTGUN, v.ammo_shells < 2.0),
                4 => (Items::NAILGUN, v.ammo_nails < 1.0),
                5 => (Items::SUPER_NAILGUN, v.ammo_nails < 2.0),
                6 => (Items::GRENADE_LAUNCHER, v.ammo_rockets < 1.0),
                7 => (Items::ROCKET_LAUNCHER, v.ammo_rockets < 1.0),
                8 => (Items::LIGHTNING, v.ammo_cells < 1.0),
                _ => (Items::empty(), false),
            }
        };
        self.entities[e].v.impulse = 0.0;

        if !self.entities[e].v.items.has(weapon) {
            self.sprint_to(e, c"no weapon.\n");
            return;
        }
        if needs_ammo {
            self.sprint_to(e, c"not enough ammo.\n");
            return;
        }
        self.entities[e].v.weapon = weapon.into();
        self.w_set_current_ammo(e);
    }

    /// Select the grappling hook (impulse 22), if the player has it.
    fn select_grapple(&mut self, e: EntId) {
        self.entities[e].v.impulse = 0.0;
        if !self.entities[e].v.items.has(Items::GRAPPLE) {
            self.sprint_to(e, c"no grapple.\n");
            return;
        }
        if self.entities[e].v.weapon == Weapon::Grapple {
            return; // already selected — don't disturb an active hook
        }
        // Switching onto the grapple drops any hook already out (a fresh start).
        if self.entities[e].grapple.hook_out {
            let hook = EntId(self.entities[e].grapple.hook);
            self.reset_grapple(hook);
        }
        self.entities[e].v.weapon = Weapon::Grapple;
        self.w_set_current_ammo(e);
    }

    /// `CheatCommand` — give all weapons/ammo (impulse 9; harmless in deathmatch where it is
    /// gated off by the QuakeC original, but we keep it for listen/dev servers).
    fn cheat_command(&mut self, e: EntId) {
        let ent = &mut self.entities[e];
        ent.v.ammo_rockets = 100.0;
        ent.v.ammo_nails = 200.0;
        ent.v.ammo_shells = 100.0;
        ent.v.ammo_cells = 200.0;
        ent.v.items = ent.v.items.with(
            Items::AXE
                | Items::SHOTGUN
                | Items::SUPER_SHOTGUN
                | Items::NAILGUN
                | Items::SUPER_NAILGUN
                | Items::GRENADE_LAUNCHER
                | Items::ROCKET_LAUNCHER
                | Items::LIGHTNING
                | Items::KEY1
                | Items::KEY2,
        );
        ent.v.weapon = Weapon::RocketLauncher;
        ent.v.impulse = 0.0;
        self.w_set_current_ammo(e);
    }

    /// `CycleWeaponCommand` — advance to the next owned, fed weapon.
    fn cycle_weapon(&mut self, e: EntId, reverse: bool) {
        self.entities[e].v.impulse = 0.0;
        const ORDER: [Items; 8] = [
            Items::AXE,
            Items::SHOTGUN,
            Items::SUPER_SHOTGUN,
            Items::NAILGUN,
            Items::SUPER_NAILGUN,
            Items::GRENADE_LAUNCHER,
            Items::ROCKET_LAUNCHER,
            Items::LIGHTNING,
        ];
        for step in 1..=ORDER.len() {
            let weapon = self.entities[e].v.weapon.item();
            let cur = ORDER.iter().position(|&w| w == weapon).unwrap_or(0);
            let next = if reverse {
                (cur + ORDER.len() - (step % ORDER.len())) % ORDER.len()
            } else {
                (cur + step) % ORDER.len()
            };
            let weapon = ORDER[next];
            if self.weapon_fed(e, weapon) {
                self.entities[e].v.weapon = weapon.into();
                self.w_set_current_ammo(e);
                return;
            }
        }
    }

    /// Whether the player owns `weapon` and has ammo for it.
    fn weapon_fed(&self, e: EntId, weapon: Items) -> bool {
        let v = &self.entities[e].v;
        if !v.items.has(weapon) {
            return false;
        }
        match crate::arsenal::weapon_spec(weapon).and_then(|s| s.ammo_kind.map(|k| (k, s.min_ammo))) {
            Some((kind, min)) => crate::arsenal::ammo_count(v, kind) >= min,
            None => true, // axe / grapple / unknown: no ammo needed
        }
    }

    /// `ImpulseCommands` — dispatch the pending impulse, then clear it. The active mode gets first
    /// refusal (CTF claims the flag/rune toss impulses); the stock table handles the rest.
    fn impulse_commands(&mut self, e: EntId) {
        let impulse = self.entities[e].v.impulse as i32;
        let mode = self.mode;
        if !mode.handle_impulse(self, e, impulse) {
            match impulse {
                1..=8 => self.w_change_weapon(e),
                9 => self.cheat_command(e),
                10 => self.cycle_weapon(e, false),
                11 => self.entities[e].v.team += 1.0, // ServerflagsCommand stand-in
                12 => self.cycle_weapon(e, true),
                20 => self.toss_ammo(e),      // drop a capped ammo backpack (rtx_dropitems)
                21 => self.toss_weapon(e),    // drop your current weapon (rtx_dropitems)
                22 => self.select_grapple(e), // grappling hook
                _ => {}
            }
        }
        self.entities[e].v.impulse = 0.0;
    }

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
