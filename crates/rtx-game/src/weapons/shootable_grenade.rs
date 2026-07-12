// SPDX-License-Identifier: AGPL-3.0-or-later

//! The rtx *shootable grenades* feature (`rtx_shootable_grenades`), split out of `weapons/mod.rs`:
//! detecting a live grenade (`is_grenade`/`is_live_shootable_grenade`), detonating one on contact
//! (`try_detonate_shootable_grenade`), and detonating any grenade a hitscan shot's line passes
//! through (`shootable_grenade_on_line`/`try_detonate_shootable_grenade_on_line`). The bullet and
//! lightning fire paths call the on-line variants; combat and the bot combo call the rest.

use glam::Vec3;

use super::*;

impl GameState {
    pub(super) fn shootable_grenades_enabled(&self) -> bool {
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

    pub(super) fn try_detonate_shootable_grenade_on_line(&mut self, start: Vec3, end: Vec3, max_fraction: f32) -> bool {
        if !self.shootable_grenades_enabled() {
            return false;
        }
        let Some(grenade) = self.shootable_grenade_on_line(start, end, max_fraction) else {
            return false;
        };
        self.grenade_explode(grenade);
        true
    }
}
