// SPDX-License-Identifier: AGPL-3.0-or-later

//! Weapon selection and the per-frame fire dispatch: `W_BestWeapon` / `weapon_fed` (the arsenal-
//! driven auto-pick), `w_attack` (route a press to the held weapon's fire), the impulse handlers
//! (`w_change_weapon` / `cycle_weapon` / `select_grapple` / cheats), and `w_check_no_ammo`. The
//! actual projectile/hitscan fire lives in the sibling weapon modules.

use crate::arsenal::{self, AmmoKind};
use crate::assets::Sound;
use crate::defs::{Attenuation, Bits, Channel, Items, Weapon};
use crate::entity::EntId;
use crate::game::GameState;

impl GameState {
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
    pub(super) fn consume_ammo(&mut self, e: EntId, kind: AmmoKind, n: f32) {
        if self.level.deathmatch == 4 {
            return;
        }
        let v = &mut self.entities[e].v;
        let field = arsenal::ammo_field_mut(v, kind);
        *field -= n;
        let remaining = *field;
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

        // Post-shot cooldown from the arsenal for the single-press weapons (the nailguns arm inside
        // `start_nail`; the lightning gun's continuous beam re-arms faster on this opening press).
        let cd = arsenal::cooldown_of(self.entities[e].v.weapon.item());
        match self.entities[e].v.weapon {
            w if w == Weapon::Axe => {
                self.entities[e].combat.attack_finished = time + cd;
                self.host
                    .sound(e, Channel::Weapon, Sound::WEAPONS_AX1, 1.0, Attenuation::Norm);
                self.start_axe_anim(e);
            }
            w if w == Weapon::Shotgun => {
                self.start_shot_anim(e);
                self.entities[e].combat.attack_finished = time + cd;
                self.w_fire_shotgun(e);
            }
            w if w == Weapon::SuperShotgun => {
                self.start_shot_anim(e);
                self.entities[e].combat.attack_finished = time + cd;
                self.w_fire_super_shotgun(e);
            }
            w if w == Weapon::Nailgun || w == Weapon::SuperNailgun => {
                self.start_nail(e);
            }
            w if w == Weapon::GrenadeLauncher => {
                self.start_rocket_anim(e);
                self.entities[e].combat.attack_finished = time + cd;
                self.w_fire_grenade(e);
            }
            w if w == Weapon::RocketLauncher => {
                self.start_rocket_anim(e);
                self.entities[e].combat.attack_finished = time + cd;
                self.w_fire_rocket(e);
            }
            w if w == Weapon::Lightning => {
                self.entities[e].combat.attack_finished = time + 0.1; // beam opens faster than its per-tick cd
                self.host
                    .sound(e, Channel::Auto, Sound::WEAPONS_LSTART, 1.0, Attenuation::Norm);
                self.start_light(e);
            }
            w if w == Weapon::Grapple => {
                self.entities[e].combat.attack_finished = time + cd;
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
        // impulse 1..=8 selects a weapon; its ammo gate comes from the arsenal (`min_ammo`), not a
        // second copy of the thresholds here. (`switch_code` is the `.weapon` value, not the impulse,
        // so the impulse map stays explicit.)
        let weapon = match self.entities[e].v.impulse as i32 {
            1 => Items::AXE,
            2 => Items::SHOTGUN,
            3 => Items::SUPER_SHOTGUN,
            4 => Items::NAILGUN,
            5 => Items::SUPER_NAILGUN,
            6 => Items::GRENADE_LAUNCHER,
            7 => Items::ROCKET_LAUNCHER,
            8 => Items::LIGHTNING,
            _ => Items::empty(),
        };
        let needs_ammo = arsenal::out_of_ammo(&self.entities[e].v, weapon);
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
        let order: Vec<Items> = arsenal::cycle_order().collect();
        for step in 1..=order.len() {
            let weapon = self.entities[e].v.weapon.item();
            let cur = order.iter().position(|&w| w == weapon).unwrap_or(0);
            let next = if reverse {
                (cur + order.len() - (step % order.len())) % order.len()
            } else {
                (cur + step) % order.len()
            };
            let weapon = order[next];
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
    pub(super) fn impulse_commands(&mut self, e: EntId) {
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
}
