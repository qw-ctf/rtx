// SPDX-License-Identifier: AGPL-3.0-or-later

//! Items and pickups, ported from `qw-qc/items.qc`: health, armor, ammo, weapons,
//! powerups, and the death backpack. Map item entities are placed on the floor after spawn,
//! hide themselves when taken, and (in deathmatch) re-appear via `SUB_regen`.

use core::ffi::CStr;

use glam::Vec3;

use crate::assets::{Model, Sound};
use crate::defs::*;
use crate::entity::{EntId, Think, Touch};
use crate::game::GameState;

/// The four ammo pools.
#[derive(Clone, Copy)]
enum AmmoKind {
    Shells,
    Nails,
    Rockets,
    Cells,
}

#[derive(Clone, Copy)]
struct WeaponSpec {
    item: Items,
    classname: Option<&'static str>,
    pickup_model: Option<Model>,
    pickup_name: &'static str,
    backpack_name: &'static str,
    ammo_kind: Option<AmmoKind>,
    pickup_ammo: f32,
    view_model: Option<Model>,
    ammo_bit: Items,
    rank: i32,
    switch_code: f32,
}

const WEAPON_SPECS: &[WeaponSpec] = &[
    WeaponSpec {
        item: Items::AXE,
        classname: None,
        pickup_model: None,
        pickup_name: "Axe",
        backpack_name: "Axe",
        ammo_kind: None,
        pickup_ammo: 0.0,
        view_model: Some(Model::PROGS_V_AXE),
        ammo_bit: Items::empty(),
        rank: 7,
        switch_code: 1.0,
    },
    WeaponSpec {
        item: Items::SHOTGUN,
        classname: None,
        pickup_model: None,
        pickup_name: "Shotgun",
        backpack_name: "Shotgun",
        ammo_kind: Some(AmmoKind::Shells),
        pickup_ammo: 0.0,
        view_model: Some(Model::PROGS_V_SHOT),
        ammo_bit: Items::SHELLS,
        rank: 7,
        switch_code: 1.0,
    },
    WeaponSpec {
        item: Items::SUPER_SHOTGUN,
        classname: Some("weapon_supershotgun"),
        pickup_model: Some(Model::PROGS_G_SHOT),
        pickup_name: "Double-barrelled Shotgun",
        backpack_name: "Double-barrelled Shotgun",
        ammo_kind: Some(AmmoKind::Shells),
        pickup_ammo: 5.0,
        view_model: Some(Model::PROGS_V_SHOT2),
        ammo_bit: Items::SHELLS,
        rank: 5,
        switch_code: 3.0,
    },
    WeaponSpec {
        item: Items::NAILGUN,
        classname: Some("weapon_nailgun"),
        pickup_model: Some(Model::PROGS_G_NAIL),
        pickup_name: "nailgun",
        backpack_name: "Nailgun",
        ammo_kind: Some(AmmoKind::Nails),
        pickup_ammo: 30.0,
        view_model: Some(Model::PROGS_V_NAIL),
        ammo_bit: Items::NAILS,
        rank: 6,
        switch_code: 4.0,
    },
    WeaponSpec {
        item: Items::SUPER_NAILGUN,
        classname: Some("weapon_supernailgun"),
        pickup_model: Some(Model::PROGS_G_NAIL2),
        pickup_name: "Super Nailgun",
        backpack_name: "Super Nailgun",
        ammo_kind: Some(AmmoKind::Nails),
        pickup_ammo: 30.0,
        view_model: Some(Model::PROGS_V_NAIL2),
        ammo_bit: Items::NAILS,
        rank: 3,
        switch_code: 5.0,
    },
    WeaponSpec {
        item: Items::GRENADE_LAUNCHER,
        classname: Some("weapon_grenadelauncher"),
        pickup_model: Some(Model::PROGS_G_ROCK),
        pickup_name: "Grenade Launcher",
        backpack_name: "Grenade Launcher",
        ammo_kind: Some(AmmoKind::Rockets),
        pickup_ammo: 5.0,
        view_model: Some(Model::PROGS_V_ROCK),
        ammo_bit: Items::ROCKETS,
        rank: 4,
        switch_code: 6.0,
    },
    WeaponSpec {
        item: Items::ROCKET_LAUNCHER,
        classname: Some("weapon_rocketlauncher"),
        pickup_model: Some(Model::PROGS_G_ROCK2),
        pickup_name: "Rocket Launcher",
        backpack_name: "Rocket Launcher",
        ammo_kind: Some(AmmoKind::Rockets),
        pickup_ammo: 5.0,
        view_model: Some(Model::PROGS_V_ROCK2),
        ammo_bit: Items::ROCKETS,
        rank: 2,
        switch_code: 7.0,
    },
    WeaponSpec {
        item: Items::LIGHTNING,
        classname: Some("weapon_lightning"),
        pickup_model: Some(Model::PROGS_G_LIGHT),
        pickup_name: "Thunderbolt",
        backpack_name: "Thunderbolt",
        ammo_kind: Some(AmmoKind::Cells),
        pickup_ammo: 15.0,
        view_model: Some(Model::PROGS_V_LIGHT),
        ammo_bit: Items::CELLS,
        rank: 1,
        switch_code: 8.0,
    },
    // Grappling hook (rtx). Not a map pickup — handed out at spawn behind `rtx_grapple` and
    // selected by impulse, so it has no `classname`/pickup. Uses the hook viewmodel and no ammo.
    WeaponSpec {
        item: Items::GRAPPLE,
        classname: None,
        pickup_model: None,
        pickup_name: "Grappling Hook",
        backpack_name: "Grappling Hook",
        ammo_kind: None,
        pickup_ammo: 0.0,
        view_model: Some(Model::PROGS_V_STAR),
        ammo_bit: Items::empty(),
        rank: 8,
        switch_code: 0.0,
    },
];

fn weapon_spec(item: Items) -> Option<&'static WeaponSpec> {
    WEAPON_SPECS.iter().find(|spec| spec.item == item)
}

fn weapon_spec_for_classname(classname: &str) -> Option<&'static WeaponSpec> {
    WEAPON_SPECS.iter().find(|spec| spec.classname == Some(classname))
}

impl GameState {
    // --- placement & respawn ---

    /// Set an item's model handle (kept for respawn — see `entity.rs`).
    fn set_item_model(&mut self, e: EntId, model: Model) {
        self.entities[e].model_cstr = Some(model);
        self.host.set_model(e, model);
    }

    /// `StartItem` — schedule the item to drop to the floor after other solids settle.
    fn start_item(&mut self, e: EntId) {
        let time = self.time();
        let ent = &mut self.entities[e];
        ent.v.nextthink = time + 0.2;
        ent.think = Think::PlaceItem;
    }

    /// `PlaceItem` — make the item a wide touch trigger and drop it to the floor.
    pub(crate) fn place_item(&mut self, e: EntId) {
        {
            let ent = &mut self.entities[e];
            ent.v.flags = Flags::ITEM.as_f32();
            ent.v.solid = Solid::Trigger;
            ent.v.movetype = MoveType::Toss;
            ent.v.velocity = Vec3::ZERO;
            ent.v.origin.z += 6.0;
        }
        if !self.host.droptofloor(e) {
            self.free(e);
        }
    }

    /// `SUB_regen` — re-show a picked-up item after its respawn delay.
    pub(crate) fn sub_regen(&mut self, e: EntId) {
        if let Some(model) = self.entities[e].model_cstr {
            self.host.set_model(e, model);
        }
        self.entities[e].v.solid = Solid::Trigger;
        self.host
            .sound(e, Channel::Voice, Sound::ITEMS_ITEMBK2, 1.0, Attenuation::Norm);
        let origin = self.entities[e].v.origin;
        self.host.set_origin(e, origin);
    }

    /// Hide a just-taken item (the native ABI hides via `modelindex`, not the model string).
    fn pickup_hide(&mut self, e: EntId) {
        let ent = &mut self.entities[e];
        ent.v.modelindex = 0.0;
        ent.v.solid = Solid::Not;
    }

    /// Schedule an item respawn (`SUB_regen`) after `delay`, then fire targets.
    fn pickup_finish(&mut self, e: EntId, other: EntId, delay: Option<f32>) {
        self.pickup_hide(e);
        if let Some(delay) = delay {
            let time = self.time();
            let ent = &mut self.entities[e];
            ent.v.nextthink = time + delay;
            ent.think = Think::SubRegen;
        }
        self.activator = other;
        self.sub_use_targets(e);
    }

    // --- health ---

    /// `T_Heal` — add health up to (optionally past) the recipient's max.
    fn t_heal(&mut self, e: EntId, heal: f32, ignore: bool) -> bool {
        let max_health = self.entities[e].v.max_health;
        let health = self.entities[e].v.health;
        if health <= 0.0 || (!ignore && health >= max_health) {
            return false;
        }
        let mut new = health + heal.ceil();
        if !ignore && new >= max_health {
            new = max_health;
        }
        if new > 250.0 {
            new = 250.0;
        }
        self.entities[e].v.health = new;
        true
    }

    /// `health_touch`.
    pub(crate) fn health_touch(&mut self, e: EntId, other: EntId) {
        if !self.is_live_player(other) {
            return;
        }
        if self.level.deathmatch == 4 && self.entities[other].combat.invincible_time > 0.0 {
            return;
        }
        let (healamount, healtype) = {
            let s = &self.entities[e];
            (s.item.healamount, s.item.healtype)
        };
        let healed = if healtype == 2.0 {
            self.entities[other].v.health < 250.0 && self.t_heal(other, healamount, true)
        } else {
            self.t_heal(other, healamount, false)
        };
        if !healed {
            return;
        }

        self.sprint_low(other, &format!("You receive {} health\n", healamount as i32));
        self.item_pickup_sound(e, other, Channel::Item);

        let time = self.time();
        if healtype == 2.0 {
            // Megahealth: rot back down later, no normal respawn.
            self.entities[other].v.items = self.entities[other].v.items.with(Items::SUPERHEALTH);
            self.pickup_hide(e);
            if self.level.deathmatch != 4 {
                let ent = &mut self.entities[e];
                ent.v.nextthink = time + 5.0;
                ent.think = Think::MegaHealthRot;
            }
            self.entities[e].set_owner(other);
            self.activator = other;
            self.sub_use_targets(e);
        } else {
            let delay = if self.level.deathmatch != 2 { Some(20.0) } else { None };
            self.pickup_finish(e, other, delay);
        }
    }

    /// `item_megahealth_rot`.
    pub(crate) fn mega_health_rot(&mut self, e: EntId) {
        let owner = self.entities[e].owner();
        let time = self.time();
        let (health, max_health) = {
            let v = &self.entities[owner].v;
            (v.health, v.max_health)
        };
        if health > max_health {
            self.entities[owner].v.health -= 1.0;
            self.entities[e].v.nextthink = time + 1.0;
            return;
        }
        self.entities[owner].v.items = self.entities[owner].v.items.without(Items::SUPERHEALTH);
        if self.level.deathmatch != 2 {
            let ent = &mut self.entities[e];
            ent.v.nextthink = time + 20.0;
            ent.think = Think::SubRegen;
        }
    }

    // --- armor ---

    /// `armor_touch`.
    pub(crate) fn armor_touch(&mut self, e: EntId, other: EntId) {
        if !self.is_live_player(other) {
            return;
        }
        if self.level.deathmatch == 4 && self.entities[other].combat.invincible_time > 0.0 {
            return;
        }
        let (type_, value, bit) = match self.entities[e].classname() {
            Some("item_armor1") => (0.3, 100.0, Items::ARMOR1.as_f32()),
            Some("item_armor2") => (0.6, 150.0, Items::ARMOR2.as_f32()),
            _ => (0.8, 200.0, Items::ARMOR3.as_f32()), // item_armorInv
        };
        {
            let v = &self.entities[other].v;
            if v.armortype * v.armorvalue >= type_ * value {
                return;
            }
        }
        {
            let v = &mut self.entities[other].v;
            v.armortype = type_;
            v.armorvalue = value;
            v.items = v.items.without(Items::ARMOR1 | Items::ARMOR2 | Items::ARMOR3).with(bit);
        }
        self.sprint_low(other, "You got armor\n");
        self.host
            .sound(other, Channel::Item, Sound::ITEMS_ARMOR1, 1.0, Attenuation::Norm);
        self.host.stuffcmd(other, c"bf\n");
        let delay = if self.level.deathmatch != 2 { Some(20.0) } else { None };
        self.pickup_finish(e, other, delay);
    }

    // --- weapons ---

    /// `weapon_touch`.
    pub(crate) fn weapon_touch(&mut self, e: EntId, other: EntId) {
        if !self.entities[other].v.flags.has(Flags::CLIENT) {
            return;
        }
        let w_switch = self.infokey_float(other, c"w_switch", 8.0);
        let best = self.w_best_weapon(other);
        let dm = self.level.deathmatch;
        let leave = dm == 2 || dm == 3 || dm == 5;

        let Some(spec) = self.entities[e].classname().and_then(weapon_spec_for_classname) else {
            return;
        };
        let Some(ammo_field) = spec.ammo_kind else {
            return;
        };
        let new = Weapon::from(spec.item);

        if leave && self.entities[other].v.items.has(new) {
            return;
        }
        self.add_ammo(other, ammo_field, spec.pickup_ammo);

        let netname = self.netname_of(e);
        self.sprint_low(other, &format!("You got the {netname}\n"));
        self.host
            .sound(other, Channel::Item, Sound::WEAPONS_PKUP, 1.0, Attenuation::Norm);
        self.host.stuffcmd(other, c"bf\n");

        self.bound_other_ammo(other);
        let old = self.entities[other].v.items;
        self.entities[other].v.items = old.with(new);

        if self.weapon_code(new) <= w_switch {
            let in_water = self.entities[other].v.flags.has(Flags::INWATER);
            if !in_water || new != Weapon::Lightning {
                self.deathmatch_weapon(other, new);
            }
        }
        self.w_set_current_ammo(other);

        if leave {
            return;
        }
        let _ = best;
        let delay = if self.level.deathmatch != 2 { Some(30.0) } else { None };
        self.pickup_finish(e, other, delay);
    }

    // --- ammo ---

    /// `ammo_touch`.
    pub(crate) fn ammo_touch(&mut self, e: EntId, other: EntId) {
        if !self.is_live_player(other) {
            return;
        }
        let best = self.w_best_weapon(other);
        let (kind, aflag) = {
            let s = &self.entities[e];
            // For an ammo item, `.weapon` is overloaded to hold the ammo-kind code (1–4).
            (s.v.weapon.as_f32(), s.item.aflag)
        };
        let (field, cap) = match kind as i32 {
            1 => (AmmoKind::Shells, 100.0),
            2 => (AmmoKind::Nails, 200.0),
            3 => (AmmoKind::Rockets, 100.0),
            _ => (AmmoKind::Cells, 100.0),
        };
        if self.ammo_of(other, field) >= cap {
            return;
        }
        self.add_ammo(other, field, aflag);
        self.bound_other_ammo(other);

        let netname = self.netname_of(e);
        self.sprint_low(other, &format!("You got the {netname}\n"));
        self.host
            .sound(other, Channel::Item, Sound::WEAPONS_LOCK4, 1.0, Attenuation::Norm);
        self.host.stuffcmd(other, c"bf\n");

        // Switch up to a better weapon if we were already on our best.
        if self.entities[other].v.weapon == best {
            let nb = self.w_best_weapon(other);
            self.entities[other].v.weapon = nb;
        }
        self.w_set_current_ammo(other);

        let dm = self.level.deathmatch;
        let delay = if dm == 3 || dm == 5 {
            Some(15.0)
        } else if dm != 2 {
            Some(30.0)
        } else {
            None
        };
        self.pickup_finish(e, other, delay);
    }

    // --- powerups ---

    /// `powerup_touch`.
    pub(crate) fn powerup_touch(&mut self, e: EntId, other: EntId) {
        if !self.is_live_player(other) {
            return;
        }
        let netname = self.netname_of(e);
        self.sprint_low(other, &format!("You got the {netname}\n"));

        let time = self.time();
        let class = self.entities[e].classname().map(str::to_owned);
        let long = matches!(
            class.as_deref(),
            Some("item_artifact_invulnerability") | Some("item_artifact_invisibility")
        );
        let item_bits = self.entities[e].v.items;

        self.item_pickup_sound(e, other, Channel::Voice);
        self.entities[other].v.items = self.entities[other].v.items.with(item_bits);

        match class.as_deref() {
            Some("item_artifact_envirosuit") => {
                let o = &mut self.entities[other];
                o.combat.rad_time = 1.0;
                o.combat.radsuit_finished = time + 30.0;
            }
            Some("item_artifact_invulnerability") => {
                let o = &mut self.entities[other];
                o.combat.invincible_time = 1.0;
                o.combat.invincible_finished = time + 30.0;
            }
            Some("item_artifact_invisibility") => {
                let o = &mut self.entities[other];
                o.combat.invisible_time = 1.0;
                o.combat.invisible_finished = time + 30.0;
            }
            Some("item_artifact_super_damage") => {
                if self.level.deathmatch == 4 {
                    let o = &mut self.entities[other];
                    o.v.armortype = 0.0;
                    o.v.armorvalue = 0.0;
                    o.v.ammo_cells = 0.0;
                }
                let o = &mut self.entities[other];
                o.combat.super_time = 1.0;
                o.combat.super_damage_finished = time + 30.0;
            }
            _ => {}
        }
        let delay = if long { Some(60.0 * 5.0) } else { Some(60.0) };
        self.pickup_finish(e, other, delay);
    }

    // --- backpacks ---

    /// `BackpackTouch` — collect a dropped backpack's ammo/weapon.
    pub(crate) fn backpack_touch(&mut self, e: EntId, other: EntId) {
        if !self.is_live_player(other) {
            return;
        }
        let b_switch = self.infokey_float(other, c"b_switch", 8.0);
        let best = self.w_best_weapon(other);

        let (s_shells, s_nails, s_rockets, s_cells, new_bits) = {
            let s = &self.entities[e].v;
            (s.ammo_shells, s.ammo_nails, s.ammo_rockets, s.ammo_cells, s.items)
        };

        {
            let o = &mut self.entities[other].v;
            o.ammo_shells += s_shells;
            o.ammo_nails += s_nails;
            o.ammo_rockets += s_rockets;
            o.ammo_cells += s_cells;
        }
        let new = if new_bits != 0.0 {
            Weapon::from_f32(new_bits)
        } else {
            self.entities[other].v.weapon
        };
        let old = self.entities[other].v.items;
        self.entities[other].v.items = old.with(new_bits);
        self.bound_other_ammo(other);

        let netname = self.netname_of(e);
        self.sprint_low(other, &format!("You get {netname}\n"));
        self.host
            .sound(other, Channel::Item, Sound::WEAPONS_LOCK4, 1.0, Attenuation::Norm);
        self.host.stuffcmd(other, c"bf\n");

        self.free(e);

        let _ = best;
        if self.weapon_code(new) <= b_switch {
            let in_water = self.entities[other].v.flags.has(Flags::INWATER);
            if !in_water || new != Weapon::Lightning {
                self.deathmatch_weapon(other, new);
            }
        }
        self.w_set_current_ammo(other);
    }

    /// `DropBackpack` — drop the player's current weapon + ammo on death.
    pub(crate) fn drop_backpack(&mut self, e: EntId) {
        let (shells, nails, rockets, cells, weapon, origin) = {
            let v = &self.entities[e].v;
            (
                v.ammo_shells,
                v.ammo_nails,
                v.ammo_rockets,
                v.ammo_cells,
                v.weapon,
                v.origin,
            )
        };
        if shells + nails + rockets + cells == 0.0 {
            return;
        }
        let netname = weapon_spec(weapon.item()).map_or("", |spec| spec.backpack_name);
        let vx = -100.0 + self.random() * 200.0;
        let vy = -100.0 + self.random() * 200.0;
        let time = self.time();
        let item = self.spawn();
        {
            let it = &mut self.entities[item];
            it.v.origin = origin - Vec3::new(0.0, 0.0, 24.0);
            it.v.items = weapon.as_f32();
            it.netname = Some(netname.into());
            it.v.ammo_shells = shells;
            it.v.ammo_nails = nails;
            it.v.ammo_rockets = rockets;
            it.v.ammo_cells = cells;
            it.v.velocity = Vec3::new(vx, vy, 300.0);
            it.v.flags = Flags::ITEM.as_f32();
            it.v.solid = Solid::Trigger;
            it.v.movetype = MoveType::Toss;
            it.set_touch(Touch::Backpack);
            it.v.nextthink = time + 120.0;
            it.think = Think::SubRemove;
        }
        self.host.set_model(item, Model::PROGS_BACKPACK);
        self.host
            .set_size(item, Vec3::new(-16.0, -16.0, 0.0), Vec3::new(16.0, 16.0, 56.0));
    }

    // --- ammo/weapon helpers ---

    fn bound_other_ammo(&mut self, e: EntId) {
        let v = &mut self.entities[e].v;
        v.ammo_shells = v.ammo_shells.min(100.0);
        v.ammo_nails = v.ammo_nails.min(200.0);
        v.ammo_rockets = v.ammo_rockets.min(100.0);
        v.ammo_cells = v.ammo_cells.min(100.0);
    }

    fn ammo_of(&self, e: EntId, kind: AmmoKind) -> f32 {
        let v = &self.entities[e].v;
        match kind {
            AmmoKind::Shells => v.ammo_shells,
            AmmoKind::Nails => v.ammo_nails,
            AmmoKind::Rockets => v.ammo_rockets,
            AmmoKind::Cells => v.ammo_cells,
        }
    }

    fn add_ammo(&mut self, e: EntId, kind: AmmoKind, amount: f32) {
        let v = &mut self.entities[e].v;
        match kind {
            AmmoKind::Shells => v.ammo_shells += amount,
            AmmoKind::Nails => v.ammo_nails += amount,
            AmmoKind::Rockets => v.ammo_rockets += amount,
            AmmoKind::Cells => v.ammo_cells += amount,
        }
    }

    /// `RankForWeapon` (lower is better).
    fn rank_for_weapon(&self, w: Weapon) -> i32 {
        weapon_spec(w.item()).map_or(7, |spec| spec.rank)
    }

    /// `WeaponCode` — the `w_switch`/`b_switch` index of a weapon.
    fn weapon_code(&self, w: Weapon) -> f32 {
        weapon_spec(w.item()).map_or(1.0, |spec| spec.switch_code)
    }

    /// `Deathmatch_Weapon` — switch up to `new` if it outranks the current weapon.
    fn deathmatch_weapon(&mut self, e: EntId, new: Weapon) {
        let cur = self.entities[e].v.weapon;
        if self.rank_for_weapon(new) < self.rank_for_weapon(cur) {
            self.entities[e].v.weapon = new;
        }
    }

    // --- small helpers ---

    fn is_live_player(&self, e: EntId) -> bool {
        self.entities[e].classname() == Some("player") && self.entities[e].v.health > 0.0
    }

    fn sprint_low(&self, e: EntId, msg: &str) {
        let c = crate::game::cstring(msg);
        self.host.sprint(e, PrintLevel::Low, &c);
    }

    /// Pickup sound on `chan` using the item's `noise`, then a screen flash.
    fn item_pickup_sound(&mut self, e: EntId, other: EntId, chan: Channel) {
        if let Some(noise) = self.entities[e].noise {
            self.host.sound(other, chan, noise, 1.0, Attenuation::Norm);
        }
        self.host.stuffcmd(other, c"bf\n");
    }

    fn infokey_float(&self, e: EntId, key: &CStr, default: f32) -> f32 {
        let mut buf = [0u8; 32];
        let s = self.host.infokey(e, key, &mut buf);
        let v: f32 = s.trim().parse().unwrap_or(0.0);
        if v == 0.0 {
            default
        } else {
            v
        }
    }
}

// Spawnflags for items.

/// Item spawn functions (dispatched from `call_spawn` by classname). Each returns whether
/// the item should remain (deathmatch modes suppress some items).
impl GameState {
    fn spawnflags(&self, e: EntId) -> f32 {
        self.entities[e].v.spawnflags
    }

    fn set_noise(&mut self, e: EntId, noise: Sound) {
        self.entities[e].noise = Some(noise);
    }

    pub(crate) fn spawn_item_health(&mut self, e: EntId) -> bool {
        self.entities[e].set_touch(Touch::ItemHealth);
        let flags = self.spawnflags(e);
        if flags.has(HealthFlags::ROTTEN) {
            self.set_item_model(e, Model::MAPS_B_BH10);
            self.set_noise(e, Sound::ITEMS_R_ITEM1);
            let ent = &mut self.entities[e];
            ent.item.healamount = 15.0;
            ent.item.healtype = 0.0;
        } else if flags.has(HealthFlags::MEGA) {
            self.set_item_model(e, Model::MAPS_B_BH100);
            self.set_noise(e, Sound::ITEMS_R_ITEM2);
            let ent = &mut self.entities[e];
            ent.item.healamount = 100.0;
            ent.item.healtype = 2.0;
        } else {
            self.set_item_model(e, Model::MAPS_B_BH25);
            self.set_noise(e, Sound::ITEMS_HEALTH1);
            let ent = &mut self.entities[e];
            ent.item.healamount = 25.0;
            ent.item.healtype = 1.0;
        }
        self.host.set_size(e, Vec3::ZERO, Vec3::new(32.0, 32.0, 56.0));
        self.start_item(e);
        true
    }

    pub(crate) fn spawn_item_armor(&mut self, e: EntId, skin: f32) -> bool {
        self.entities[e].set_touch(Touch::ItemArmor);
        self.set_item_model(e, Model::PROGS_ARMOR);
        self.entities[e].v.skin = skin;
        self.host
            .set_size(e, Vec3::new(-16.0, -16.0, 0.0), Vec3::new(16.0, 16.0, 56.0));
        self.start_item(e);
        true
    }

    pub(crate) fn spawn_weapon_by_classname(&mut self, e: EntId, classname: &str) -> bool {
        let Some(spec) = weapon_spec_for_classname(classname) else {
            return false;
        };
        self.spawn_weapon(e, spec)
    }

    fn spawn_weapon(&mut self, e: EntId, spec: &WeaponSpec) -> bool {
        if self.level.deathmatch > 3 {
            return false;
        }
        let Some(model) = spec.pickup_model else {
            return false;
        };
        self.set_item_model(e, model);
        self.entities[e].set_touch(Touch::ItemWeapon);
        self.entities[e].netname = Some(spec.pickup_name.into());
        self.host
            .set_size(e, Vec3::new(-16.0, -16.0, 0.0), Vec3::new(16.0, 16.0, 56.0));
        self.start_item(e);
        true
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn spawn_ammo(
        &mut self,
        e: EntId,
        weapon_code: f32,
        netname: &'static str,
        small: Model,
        small_amt: f32,
        big: Model,
        big_amt: f32,
    ) -> bool {
        if self.level.deathmatch == 4 {
            return false;
        }
        self.entities[e].set_touch(Touch::ItemAmmo);
        let (model, amt) = if self.spawnflags(e).has(AmmoFlags::BIG) {
            (big, big_amt)
        } else {
            (small, small_amt)
        };
        self.set_item_model(e, model);
        {
            let ent = &mut self.entities[e];
            ent.item.aflag = amt;
            // Ammo items overload `.weapon` to carry their ammo-kind code (a raw f32).
            ent.v.weapon = Weapon::from_f32(weapon_code);
            ent.netname = Some(netname.into());
        }
        self.host.set_size(e, Vec3::ZERO, Vec3::new(32.0, 32.0, 56.0));
        self.start_item(e);
        true
    }

    pub(crate) fn spawn_powerup(
        &mut self,
        e: EntId,
        model: Model,
        noise: Sound,
        netname: &'static str,
        item_bit: Items,
        effect: Effects,
    ) -> bool {
        self.entities[e].set_touch(Touch::ItemPowerup);
        self.set_item_model(e, model);
        self.set_noise(e, noise);
        {
            let ent = &mut self.entities[e];
            ent.netname = Some(netname.into());
            ent.v.items = item_bit.as_f32();
            ent.v.effects = ent.v.effects.with(effect);
        }
        self.host
            .set_size(e, Vec3::new(-16.0, -16.0, -24.0), Vec3::new(16.0, 16.0, 32.0));
        self.start_item(e);
        true
    }

    pub(crate) fn current_weapon_ammo_state(&self, player: EntId) -> (f32, Option<Model>, Items) {
        let v = &self.entities[player].v;
        let Some(spec) = weapon_spec(v.weapon.item()) else {
            return (0.0, None, Items::empty());
        };
        let ammo = spec.ammo_kind.map_or(0.0, |kind| self.ammo_of(player, kind));
        (ammo, spec.view_model, spec.ammo_bit)
    }
}
