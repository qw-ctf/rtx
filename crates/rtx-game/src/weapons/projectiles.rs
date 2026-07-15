// SPDX-License-Identifier: AGPL-3.0-or-later

//! Projectile weapons, split out of `weapons/mod.rs`: rockets, grenades, and nails (spikes) —
//! each launches an entity that carries a `Touch`/`Think` and does its damage on impact or fuse.
//! `t_missile_touch`/`grenade_touch`/`spike_touch` are the engine touch callbacks.

use glam::Vec3;

use super::*;
use crate::math::vectoangles;

impl GameState {
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
    pub(super) fn w_fire_rocket(&mut self, e: EntId) {
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
        self.set_model(m, Model::PROGS_MISSILE);
        self.set_size(m, Vec3::ZERO, Vec3::ZERO);
        self.set_origin(m, rocket_muzzle(origin, v_forward));
        // Stamp the shooter's firing origin (set after `set_origin` so it isn't clobbered): the
        // midair mode scores airshots by the vertical shooter→victim distance. Unused otherwise —
        // `FlyMissile` physics never touch `oldorigin`.
        self.entities[m].v.oldorigin = origin;
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
    pub(super) fn w_fire_grenade(&mut self, e: EntId) {
        let time = self.time();
        self.consume_ammo(e, AmmoKind::Rockets, 1.0);
        self.host
            .sound(e, Channel::Weapon, Sound::WEAPONS_GRENADE, 1.0, Attenuation::Norm);
        self.small_kick(e);

        let (origin, v_angle) = {
            let v = &self.entities[e].v;
            (v.origin, v.v_angle)
        };
        self.make_vectors(v_angle);
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
        self.set_model(m, Model::PROGS_GRENADE);
        if shootable {
            self.set_size(m, SHOOTABLE_GRENADE_MINS, SHOOTABLE_GRENADE_MAXS);
        } else {
            self.set_size(m, Vec3::ZERO, Vec3::ZERO);
        }
        self.set_origin(m, origin);
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
        self.set_model(m, Model::PROGS_SPIKE);
        self.set_size(m, Vec3::ZERO, Vec3::ZERO);
        self.set_origin(m, org);
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
        self.set_model(m, Model::PROGS_S_SPIKE);
        self.set_size(m, Vec3::ZERO, Vec3::ZERO);
        self.small_kick(e);
    }

    /// `W_FireSpikes` — nailgun fire (delegates to super spikes for the SNG).
    pub(crate) fn w_fire_spikes(&mut self, e: EntId, ox: f32) {
        let time = self.time();
        let v_angle = self.entities[e].v.v_angle;
        self.make_vectors(v_angle);
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
}
