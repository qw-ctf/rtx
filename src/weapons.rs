//! Weapon firing and projectiles, ported from `qw-qc/weapons.qc`.
//!
//! Single-shot weapons fire directly from [`GameState::w_attack`]; the nailgun and lightning
//! gun fire from their looping animation think-chains (see `player.rs`). Projectiles carry a
//! [`Touch`] behaviour and (for grenades/rockets) a timed [`Think`].

use core::ffi::CStr;

use glam::Vec3;

use crate::assets::Sound;
use crate::defs::*;
use crate::entity::{Die, EntId, Think, Touch};
use crate::game::GameState;

/// QuakeC `crandom` — a float in `[-1, 1)`.
fn crandom(game: &mut GameState) -> f32 {
    2.0 * (game.random() - 0.5)
}

const SHOOTABLE_GRENADE_HIT_RADIUS: f32 = 8.0;
const SHOOTABLE_GRENADE_MINS: Vec3 = Vec3::splat(-4.0);
const SHOOTABLE_GRENADE_MAXS: Vec3 = Vec3::splat(4.0);

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
        self.host.cvar(c"rtx_shootable_grenades") != 0.0
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

    fn shootable_grenade_on_line(
        &self,
        start: Vec3,
        end: Vec3,
        max_fraction: f32,
    ) -> Option<EntId> {
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
            if ent.classname() != Some("grenade")
                || !ent.in_use
                || ent.combat.voided != 0.0
            {
                continue;
            }
            let t = (ent.v.origin - start).dot(dir) / len2;
            if !(0.0..best).contains(&t) {
                continue;
            }
            let closest = start + dir * t;
            if (ent.v.origin - closest).length_squared()
                <= SHOOTABLE_GRENADE_HIT_RADIUS * SHOOTABLE_GRENADE_HIT_RADIUS
            {
                best = t;
                hit = Some(EntId(i as u32));
            }
        }
        hit
    }

    fn try_detonate_shootable_grenade_on_line(
        &mut self,
        start: Vec3,
        end: Vec3,
        max_fraction: f32,
    ) -> bool {
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
    fn aim_dir(&mut self, e: EntId) -> Vec3 {
        let v_angle = self.entities[e].v.v_angle;
        self.host.make_vectors(v_angle);
        self.globals.v_forward
    }

    /// `muzzleflash` — networked muzzle flash for the firing player.
    pub(crate) fn muzzleflash(&self, e: EntId) {
        let origin = self.entities[e].v.origin;
        self.host.write_svc(MsgDest::Multicast, Svc::MuzzleFlash);
        self.host.write_entity(MsgDest::Multicast, e.0 as i32);
        self.host.multicast(origin, Multicast::Pvs);
    }

    /// `SuperDamageSound` — periodic quad-damage hum.
    pub(crate) fn super_damage_sound(&mut self, e: EntId) {
        let time = self.time();
        let ent = &self.entities[e];
        if ent.combat.super_damage_finished > time && ent.combat.super_sound < time {
            self.entities[e].combat.super_sound = time + 1.0;
            self.host
                .sound(e.0 as i32, Channel::Body, Sound::ITEMS_DAMAGE3, 1.0, Attenuation::Norm);
        }
    }

    /// `SpawnBlood` — networked blood puff.
    fn spawn_blood(&self, org: Vec3, count: i32) {
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
        if self.entities[tr.ent].v.takedamage != 0.0 {
            self.entities[tr.ent].combat.axhitme = 1.0;
            self.spawn_blood(org, 20);
            let dmg = if self.level.deathmatch > 3 { 75.0 } else { 20.0 };
            self.t_damage(tr.ent, e, e, dmg);
        } else {
            self.host
                .sound(e.0 as i32, Channel::Weapon, Sound::PLAYER_AXHIT2, 1.0, Attenuation::Norm);
            self.host.write_te(MsgDest::Multicast, Te::Gunshot);
            self.host.write_byte(MsgDest::Multicast, 3);
            self.write_coords(MsgDest::Multicast, org);
            self.host.multicast(org, Multicast::Pvs);
        }
    }

    // --- bullets (shotgun family) ---

    /// `FireBullets` (+ `TraceAttack`/multi-damage) — fire `shotcount` pellets in a cone,
    /// combining hits on the same target into one `T_Damage` call.
    fn fire_bullets(&mut self, e: EntId, shotcount: i32, dir: Vec3, spread: Vec3) {
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
            let direction =
                dir + crandom(self) * spread.x * v_right + crandom(self) * spread.y * v_up;
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
            if self.entities[tr.ent].v.takedamage != 0.0 {
                blood_count += 1;
                blood_org = org;
                if tr.ent != multi_ent {
                    if multi_ent != EntId::WORLD {
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
            .sound(e.0 as i32, Channel::Weapon, Sound::WEAPONS_GUNCOCK, 1.0, Attenuation::Norm);
        self.small_kick(e);
        if self.level.deathmatch != 4 {
            let ent = &mut self.entities[e];
            ent.v.ammo_shells -= 1.0;
            ent.v.currentammo = ent.v.ammo_shells;
        }
        let dir = self.aim_dir(e);
        self.fire_bullets(e, 6, dir, Vec3::new(0.04, 0.04, 0.0));
    }

    /// `W_FireSuperShotgun`.
    fn w_fire_super_shotgun(&mut self, e: EntId) {
        if self.entities[e].v.currentammo == 1.0 {
            self.w_fire_shotgun(e);
            return;
        }
        self.host
            .sound(e.0 as i32, Channel::Weapon, Sound::WEAPONS_SHOTGN2, 1.0, Attenuation::Norm);
        self.big_kick(e);
        if self.level.deathmatch != 4 {
            let ent = &mut self.entities[e];
            ent.v.ammo_shells -= 2.0;
            ent.v.currentammo = ent.v.ammo_shells;
        }
        let dir = self.aim_dir(e);
        self.fire_bullets(e, 14, dir, Vec3::new(0.14, 0.08, 0.0));
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
            self.entities[other].deathtype = Some("rocket".into());
            self.t_damage(other, e, owner, damg);
        }
        self.t_radius_damage(e, owner, 120.0, other, "rocket");

        let velocity = self.entities[e].v.velocity;
        let org = self.entities[e].v.origin - velocity.normalize_or_zero() * 8.0;
        self.entities[e].v.origin = org;
        self.host.write_te(MsgDest::Multicast, Te::Explosion);
        self.write_coords(MsgDest::Multicast, org);
        self.host.multicast(org, Multicast::Phs);
        self.free(e);
    }

    /// `W_FireRocket`.
    fn w_fire_rocket(&mut self, e: EntId) {
        if self.level.deathmatch != 4 {
            let ent = &mut self.entities[e];
            ent.v.ammo_rockets -= 1.0;
            ent.v.currentammo = ent.v.ammo_rockets;
        }
        self.host
            .sound(e.0 as i32, Channel::Weapon, Sound::WEAPONS_SGUN1, 1.0, Attenuation::Norm);
        self.small_kick(e);

        let origin = self.entities[e].v.origin;
        let dir = self.aim_dir(e); // also refreshes v_forward
        let v_forward = self.globals.v_forward;

        let m = self.spawn();
        {
            let mis = &mut self.entities[m];
            mis.set_owner(e);
            mis.v.movetype = MoveType::FlyMissile.as_f32();
            mis.v.solid = Solid::BBox.as_f32();
            mis.v.velocity = dir * 1000.0;
            mis.v.angles = vectoangles(mis.v.velocity);
            mis.set_touch(Touch::Missile);
            mis.combat.voided = 0.0;
            mis.v.nextthink = self.globals.time + 5.0;
            mis.think = Think::SubRemove;
            mis.classname = Some("rocket".into());
        }
        self.host.set_model(m.0 as i32, c"progs/missile.mdl");
        self.host.set_size(m.0 as i32, Vec3::ZERO, Vec3::ZERO);
        self.host
            .set_origin(m.0 as i32, origin + v_forward * 8.0 + Vec3::new(0.0, 0.0, 16.0));
    }

    // --- lightning ---

    fn lightning_hit(&mut self, from: EntId, damage: f32) {
        let endpos = self.globals.trace_endpos;
        let trace_ent = EntId::from_prog(self.globals.trace_ent);
        self.host.write_te(MsgDest::Multicast, Te::LightningBlood);
        self.write_coords(MsgDest::Multicast, endpos);
        self.host.multicast(endpos, Multicast::Pvs);
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
        if self.entities[e1].v.takedamage != 0.0 {
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
        if e2 != e1 && self.entities[e2].v.takedamage != 0.0 {
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
        if e3 != e1 && e3 != e2 && self.entities[e3].v.takedamage != 0.0 {
            self.lightning_hit(from, damage);
        }
    }

    /// `W_FireLightning` — one bolt; underwater discharge dumps all cells as radius damage.
    pub(crate) fn w_fire_lightning(&mut self, e: EntId) {
        let time = self.time();
        if self.entities[e].v.ammo_cells < 1.0 {
            let best = self.w_best_weapon(e);
            self.entities[e].v.weapon = best.as_f32();
            self.w_set_current_ammo(e);
            return;
        }

        // Underwater discharge.
        if self.entities[e].v.waterlevel > 1.0 {
            let cells = self.entities[e].v.ammo_cells;
            self.entities[e].v.ammo_cells = 0.0;
            self.w_set_current_ammo(e);
            self.t_radius_damage(e, e, 35.0 * cells, EntId::WORLD, "");
            return;
        }

        if self.entities[e].mover.t_width < time {
            self.host
                .sound(e.0 as i32, Channel::Weapon, Sound::WEAPONS_LHIT, 1.0, Attenuation::Norm);
            self.entities[e].mover.t_width = time + 0.6;
        }
        self.small_kick(e);
        if self.level.deathmatch != 4 {
            let ent = &mut self.entities[e];
            ent.v.ammo_cells -= 1.0;
            ent.v.currentammo = ent.v.ammo_cells;
        }

        let v_angle = self.entities[e].v.v_angle;
        self.host.make_vectors(v_angle);
        let v_forward = self.globals.v_forward;
        let org = self.entities[e].v.origin + Vec3::new(0.0, 0.0, 16.0);
        let tr = self.traceline(org, org + v_forward * 600.0, true, e);
        let endpos = tr.endpos;

        self.host.write_te(MsgDest::Multicast, Te::Lightning2);
        self.host.write_entity(MsgDest::Multicast, e.0 as i32);
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
        self.t_radius_damage(e, owner, 120.0, EntId::WORLD, "grenade");

        let origin = self.entities[e].v.origin;
        self.host.write_te(MsgDest::Multicast, Te::Explosion);
        self.write_coords(MsgDest::Multicast, origin);
        self.host.multicast(origin, Multicast::Phs);
        self.free(e);
    }

    /// `GrenadeTouch` — explode on players, else bounce.
    pub(crate) fn grenade_touch(&mut self, e: EntId, other: EntId) {
        if other == self.entities[e].owner() {
            return;
        }
        if self.entities[other].v.takedamage.is(TakeDamage::Aim) {
            self.grenade_explode(e);
            return;
        }
        self.host
            .sound(e.0 as i32, Channel::Weapon, Sound::WEAPONS_BOUNCE, 1.0, Attenuation::Norm);
        if self.entities[e].v.velocity == Vec3::ZERO {
            self.entities[e].v.avelocity = Vec3::ZERO;
        }
    }

    /// `W_FireGrenade`.
    fn w_fire_grenade(&mut self, e: EntId) {
        let time = self.time();
        if self.level.deathmatch != 4 {
            let ent = &mut self.entities[e];
            ent.v.ammo_rockets -= 1.0;
            ent.v.currentammo = ent.v.ammo_rockets;
        }
        self.host
            .sound(e.0 as i32, Channel::Weapon, Sound::WEAPONS_GRENADE, 1.0, Attenuation::Norm);
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
            v_forward * 600.0
                + v_up * 200.0
                + crandom(self) * v_right * 10.0
                + crandom(self) * v_up * 10.0
        } else {
            let mut vel = self.aim_dir(e) * 600.0;
            vel.z = 200.0;
            vel
        };
        {
            let mis = &mut self.entities[m];
            mis.combat.voided = 0.0;
            mis.set_owner(e);
            mis.v.movetype = MoveType::Bounce.as_f32();
            mis.v.solid = Solid::BBox.as_f32();
            mis.classname = Some("grenade".into());
            mis.v.velocity = velocity;
            mis.v.avelocity = Vec3::new(300.0, 300.0, 300.0);
            mis.v.angles = vectoangles(velocity);
            mis.set_touch(Touch::Grenade);
            mis.v.nextthink = time + 2.5;
            mis.think = Think::GrenadeExplode;
            if shootable {
                mis.v.takedamage = TakeDamage::Aim.as_f32();
                mis.v.health = 1.0;
                mis.th_die = Die::GrenadeExplode;
            }
        }
        self.host.set_model(m.0 as i32, c"progs/grenade.mdl");
        if shootable {
            self.host
                .set_size(m.0 as i32, SHOOTABLE_GRENADE_MINS, SHOOTABLE_GRENADE_MAXS);
        } else {
            self.host.set_size(m.0 as i32, Vec3::ZERO, Vec3::ZERO);
        }
        self.host.set_origin(m.0 as i32, origin);
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
            mis.v.movetype = MoveType::FlyMissile.as_f32();
            mis.v.solid = Solid::BBox.as_f32();
            mis.v.angles = vectoangles(dir);
            mis.set_touch(Touch::Spike);
            mis.classname = Some("spike".into());
            mis.think = Think::SubRemove;
            mis.v.nextthink = time + 6.0;
            mis.v.velocity = dir * 1000.0;
        }
        self.host.set_model(m.0 as i32, c"progs/spike.mdl");
        self.host.set_size(m.0 as i32, Vec3::ZERO, Vec3::ZERO);
        self.host.set_origin(m.0 as i32, org);
        m
    }

    /// `W_FireSuperSpikes`.
    fn w_fire_super_spikes(&mut self, e: EntId) {
        let time = self.time();
        self.host
            .sound(e.0 as i32, Channel::Weapon, Sound::WEAPONS_SPIKE2, 1.0, Attenuation::Norm);
        self.entities[e].combat.attack_finished = time + 0.2;
        if self.level.deathmatch != 4 {
            let ent = &mut self.entities[e];
            ent.v.ammo_nails -= 2.0;
            ent.v.currentammo = ent.v.ammo_nails;
        }
        let dir = self.aim_dir(e);
        let org = self.entities[e].v.origin + Vec3::new(0.0, 0.0, 16.0);
        let m = self.launch_spike(e, org, dir);
        self.entities[m].set_touch(Touch::SuperSpike);
        self.host.set_model(m.0 as i32, c"progs/s_spike.mdl");
        self.host.set_size(m.0 as i32, Vec3::ZERO, Vec3::ZERO);
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
        if ammo_nails >= 2.0 && weapon.is(Items::SUPER_NAILGUN) {
            self.w_fire_super_spikes(e);
            return;
        }
        if ammo_nails < 1.0 {
            let best = self.w_best_weapon(e);
            self.entities[e].v.weapon = best.as_f32();
            self.w_set_current_ammo(e);
            return;
        }
        self.host
            .sound(e.0 as i32, Channel::Weapon, Sound::WEAPONS_ROCKET1I, 1.0, Attenuation::Norm);
        self.entities[e].combat.attack_finished = time + 0.2;
        if self.level.deathmatch != 4 {
            let ent = &mut self.entities[e];
            ent.v.ammo_nails -= 1.0;
            ent.v.currentammo = ent.v.ammo_nails;
        }
        let dir = self.aim_dir(e);
        let org = self.entities[e].v.origin + Vec3::new(0.0, 0.0, 16.0) + v_right * ox;
        self.launch_spike(e, org, dir);
        self.small_kick(e);
    }

    /// `spike_touch` / `superspike_touch` — spike impact.
    pub(crate) fn spike_touch(&mut self, e: EntId, other: EntId, super_spike: bool) {
        if other == self.entities[e].owner() {
            return;
        }
        if self.entities[e].combat.voided != 0.0 {
            return;
        }
        self.entities[e].combat.voided = 1.0;
        if self.entities[other].v.solid.is(Solid::Trigger) {
            return;
        }
        let origin = self.entities[e].v.origin;
        if self.host.pointcontents(origin).is(Content::Sky) {
            self.free(e);
            return;
        }

        let owner = self.entities[e].owner();
        let (damage, dtype, te) = if super_spike {
            (18.0, "supernail", Te::SuperSpike)
        } else {
            (9.0, "nail", Te::Spike)
        };

        if self.entities[other].v.takedamage != 0.0 {
            if self.try_detonate_shootable_grenade(other) {
                self.free(e);
                return;
            }
            self.spawn_touchblood(e, damage);
            self.entities[other].deathtype = Some(dtype.into());
            self.t_damage(other, e, owner, damage);
        } else {
            self.host.write_te(MsgDest::Multicast, te);
            self.write_coords(MsgDest::Multicast, origin);
            self.host.multicast(origin, Multicast::Phs);
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

    /// `W_BestWeapon` — best weapon the player can currently fire.
    pub(crate) fn w_best_weapon(&self, e: EntId) -> Items {
        let v = &self.entities[e].v;
        let has = |w: Items| v.items.has(w);
        if v.waterlevel <= 1.0 && v.ammo_cells >= 1.0 && has(Items::LIGHTNING) {
            Items::LIGHTNING
        } else if v.ammo_nails >= 2.0 && has(Items::SUPER_NAILGUN) {
            Items::SUPER_NAILGUN
        } else if v.ammo_shells >= 2.0 && has(Items::SUPER_SHOTGUN) {
            Items::SUPER_SHOTGUN
        } else if v.ammo_nails >= 1.0 && has(Items::NAILGUN) {
            Items::NAILGUN
        } else if v.ammo_shells >= 1.0 && has(Items::SHOTGUN) {
            Items::SHOTGUN
        } else {
            Items::AXE
        }
    }

    /// `W_CheckNoAmmo` — switch off an empty weapon; returns whether we can fire.
    fn w_check_no_ammo(&mut self, e: EntId) -> bool {
        let v = &self.entities[e].v;
        if v.currentammo > 0.0 || Items::from_f32(v.weapon) == Items::AXE {
            return true;
        }
        let best = self.w_best_weapon(e);
        self.entities[e].v.weapon = best.as_f32();
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

        match Items::from_f32(self.entities[e].v.weapon) {
            w if w == Items::AXE => {
                self.entities[e].combat.attack_finished = time + 0.5;
                self.host
                    .sound(e.0 as i32, Channel::Weapon, Sound::WEAPONS_AX1, 1.0, Attenuation::Norm);
                self.start_axe_anim(e);
            }
            w if w == Items::SHOTGUN => {
                self.start_shot_anim(e);
                self.entities[e].combat.attack_finished = time + 0.5;
                self.w_fire_shotgun(e);
            }
            w if w == Items::SUPER_SHOTGUN => {
                self.start_shot_anim(e);
                self.entities[e].combat.attack_finished = time + 0.7;
                self.w_fire_super_shotgun(e);
            }
            w if w == Items::NAILGUN || w == Items::SUPER_NAILGUN => {
                self.start_nail(e);
            }
            w if w == Items::GRENADE_LAUNCHER => {
                self.start_rocket_anim(e);
                self.entities[e].combat.attack_finished = time + 0.6;
                self.w_fire_grenade(e);
            }
            w if w == Items::ROCKET_LAUNCHER => {
                self.start_rocket_anim(e);
                self.entities[e].combat.attack_finished = time + 0.8;
                self.w_fire_rocket(e);
            }
            w if w == Items::LIGHTNING => {
                self.entities[e].combat.attack_finished = time + 0.1;
                self.host
                    .sound(e.0 as i32, Channel::Auto, Sound::WEAPONS_LSTART, 1.0, Attenuation::Norm);
                self.start_light(e);
            }
            _ => {}
        }
    }

    /// `W_ChangeWeapon` — switch to the impulse-selected weapon if owned and fed.
    fn w_change_weapon(&mut self, e: EntId) {
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
        self.entities[e].v.weapon = weapon.as_f32();
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
            Items::AXE | Items::SHOTGUN | Items::SUPER_SHOTGUN | Items::NAILGUN | Items::SUPER_NAILGUN | Items::GRENADE_LAUNCHER | Items::ROCKET_LAUNCHER | Items::LIGHTNING | Items::KEY1 | Items::KEY2,
        );
        ent.v.weapon = Items::ROCKET_LAUNCHER.as_f32();
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
            let weapon = Items::from_f32(self.entities[e].v.weapon);
            let cur = ORDER.iter().position(|&w| w == weapon).unwrap_or(0);
            let next = if reverse {
                (cur + ORDER.len() - (step % ORDER.len())) % ORDER.len()
            } else {
                (cur + step) % ORDER.len()
            };
            let weapon = ORDER[next];
            if self.weapon_fed(e, weapon) {
                self.entities[e].v.weapon = weapon.as_f32();
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
        match weapon {
            w if w == Items::SHOTGUN => v.ammo_shells >= 1.0,
            w if w == Items::SUPER_SHOTGUN => v.ammo_shells >= 2.0,
            w if w == Items::NAILGUN => v.ammo_nails >= 1.0,
            w if w == Items::SUPER_NAILGUN => v.ammo_nails >= 2.0,
            w if w == Items::GRENADE_LAUNCHER || w == Items::ROCKET_LAUNCHER => {
                v.ammo_rockets >= 1.0
            }
            w if w == Items::LIGHTNING => v.ammo_cells >= 1.0,
            _ => true, // axe
        }
    }

    /// `ImpulseCommands` — dispatch the pending impulse, then clear it.
    fn impulse_commands(&mut self, e: EntId) {
        let impulse = self.entities[e].v.impulse as i32;
        match impulse {
            1..=8 => self.w_change_weapon(e),
            9 => self.cheat_command(e),
            10 => self.cycle_weapon(e, false),
            11 => self.entities[e].v.team += 1.0, // ServerflagsCommand stand-in
            12 => self.cycle_weapon(e, true),
            _ => {}
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
            self.super_damage_sound(e);
            self.w_attack(e);
        }
    }

    // --- small helpers ---

    /// `Svc::SmallKick` view punch to a single client (`msg_entity = e; WriteByte MsgDest::One`).
    fn small_kick(&mut self, e: EntId) {
        self.globals.msg_entity = e.to_prog();
        self.host.write_svc(MsgDest::One, Svc::SmallKick);
    }

    /// `Svc::BigKick` view punch (super shotgun).
    fn big_kick(&mut self, e: EntId) {
        self.globals.msg_entity = e.to_prog();
        self.host.write_svc(MsgDest::One, Svc::BigKick);
    }

    /// `sprint(self, PrintLevel::High, ...)` to a player.
    fn sprint_to(&self, e: EntId, msg: &CStr) {
        self.host.sprint(e.0 as i32, PrintLevel::High, msg);
    }
}
