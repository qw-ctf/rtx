// SPDX-License-Identifier: AGPL-3.0-or-later

//! Hitscan weapons, split out of `weapons/mod.rs`: the axe, the shotgun family
//! (`fire_bullets` + the two shotguns), and the lightning gun. Instant-hit, no projectile entity —
//! each traces to its target(s) this frame. The bullet and beam paths detonate any live shootable
//! grenade their line crosses (`try_detonate_shootable_grenade_on_line`).

use glam::Vec3;

use super::*;

impl GameState {
    // --- axe ---

    /// `W_FireAxe` — short melee trace.
    pub(crate) fn w_fire_axe(&mut self, e: EntId) {
        let v_angle = self.entities[e].v.v_angle;
        self.make_vectors(v_angle);
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
        self.make_vectors(v_angle);
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
    pub(super) fn w_fire_shotgun(&mut self, e: EntId) {
        self.host
            .sound(e, Channel::Weapon, Sound::WEAPONS_GUNCOCK, 1.0, Attenuation::Norm);
        self.small_kick(e);
        self.consume_ammo(e, AmmoKind::Shells, 1.0);
        let dir = self.aim_dir(e);
        self.fire_bullets(e, 6, dir, Vec3::new(0.04, 0.04, 0.0), DeathType::Shotgun);
    }

    /// `W_FireSuperShotgun`.
    pub(super) fn w_fire_super_shotgun(&mut self, e: EntId) {
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

    // --- lightning ---

    /// The lightning blood + damage for one bolt trace that hit `hit` at `endpos`. Takes the impact
    /// from the caller's [`TraceResult`] rather than the shared `trace_*` globals — the same values
    /// the LG traceline produced, but robust to any intervening trace.
    fn lightning_hit(&mut self, endpos: Vec3, hit: EntId, from: EntId, damage: f32) {
        self.host.write_te(MsgDest::Multicast, Te::LightningBlood);
        self.write_coords(MsgDest::Multicast, endpos);
        self.host.multicast(endpos, Multicast::Pvs);
        self.entities[hit].deathtype = DeathType::LightningBeam;
        self.t_damage(hit, from, from, damage);
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
            self.lightning_hit(tr.endpos, e1, from, damage);
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
            self.lightning_hit(tr.endpos, e2, from, damage);
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
            self.lightning_hit(tr.endpos, e3, from, damage);
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
        self.make_vectors(v_angle);
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
}
