// SPDX-License-Identifier: AGPL-3.0-or-later

//! The single source of truth for the weapon/ammo taxonomy: one [`WeaponSpec`] per weapon, keyed by
//! its [`Items`] bit, plus the four ammo pools. Pickup/backpack data (models, names, pickup ammo)
//! feeds `items.rs`; the fire thresholds and auto-pick order (`min_ammo`, `auto_pick`) feed
//! `weapons.rs`'s `W_BestWeapon`/`weapon_fed`. Before this table these facts were spread across
//! `items.rs`, `weapons.rs`, and the bot's own parallel weapon enum.

use crate::abi::EntVars;
use crate::assets::Model;
use crate::defs::Items;

/// The four ammo pools a weapon can draw from.
#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum AmmoKind {
    Shells,
    Nails,
    Rockets,
    Cells,
}

/// Everything the game needs to know about one weapon, keyed by its [`Items`] bit.
#[derive(Clone, Copy)]
pub(crate) struct WeaponSpec {
    /// The `Items` ownership bit (and the `.weapon` selection value).
    pub item: Items,
    /// Short console token for the `rtx_weapons` enable-list (e.g. `"rl"`, `"lg"`, `"hook"`).
    pub token: &'static str,
    /// Map pickup classname (`None` for the axe/shotgun starting kit and the rtx grapple).
    pub classname: Option<&'static str>,
    /// World pickup model (`None` when not a map pickup).
    pub pickup_model: Option<Model>,
    /// Sprint/console name on pickup.
    pub pickup_name: &'static str,
    /// Name shown when the weapon is carried in a death backpack.
    pub backpack_name: &'static str,
    /// The ammo pool this weapon draws from (`None` = the axe/grapple, which use no ammo).
    pub ammo_kind: Option<AmmoKind>,
    /// Ammo granted by picking the weapon up.
    pub pickup_ammo: f32,
    /// First-person view model.
    pub view_model: Option<Model>,
    /// The ammo `Items` bit this weapon flags as the current-ammo icon.
    pub ammo_bit: Items,
    /// Pickup desirability rank (lower = better; the death-drop picker keeps the strongest).
    pub rank: i32,
    /// `.weapon` switch code / selection impulse (`1..=8`; `0` = not impulse-selected here).
    pub switch_code: f32,
    /// Minimum ammo in [`ammo_kind`] needed to fire one shot (`0` for the axe/grapple).
    pub min_ammo: f32,
    /// Post-shot re-arm delay (seconds) added to `attack_finished` — the cooldown literal that used
    /// to be scattered across `w_attack`'s arms and the nail/light loop thinks. The lightning gun's
    /// *continuous* beam re-arms faster on the opening press (`w_attack` keeps its explicit 0.1);
    /// this is its steady per-tick rate.
    pub cooldown: f32,
    /// Position in `W_BestWeapon`'s auto-pick chain, best first (`Some(0)` = tried first). `None`
    /// means never auto-selected: the explosives (stock QW never auto-switches to GL/RL) and the
    /// axe (the hard fallback) / grapple.
    pub auto_pick: Option<u8>,
}

/// The weapon table. Order here is only the pickup/backpack iteration order; the auto-pick chain is
/// driven by [`WeaponSpec::auto_pick`], not by position.
pub(crate) const WEAPON_SPECS: &[WeaponSpec] = &[
    WeaponSpec {
        item: Items::AXE,
        token: "axe",
        cooldown: 0.5,
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
        min_ammo: 0.0,
        auto_pick: None,
    },
    WeaponSpec {
        item: Items::SHOTGUN,
        token: "sg",
        cooldown: 0.5,
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
        min_ammo: 1.0,
        auto_pick: Some(4),
    },
    WeaponSpec {
        item: Items::SUPER_SHOTGUN,
        token: "ssg",
        cooldown: 0.7,
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
        min_ammo: 2.0,
        auto_pick: Some(2),
    },
    WeaponSpec {
        item: Items::NAILGUN,
        token: "ng",
        cooldown: 0.2,
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
        min_ammo: 1.0,
        auto_pick: Some(3),
    },
    WeaponSpec {
        item: Items::SUPER_NAILGUN,
        token: "sng",
        cooldown: 0.2,
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
        min_ammo: 2.0,
        auto_pick: Some(1),
    },
    WeaponSpec {
        item: Items::GRENADE_LAUNCHER,
        token: "gl",
        cooldown: 0.6,
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
        min_ammo: 1.0,
        auto_pick: None,
    },
    WeaponSpec {
        item: Items::ROCKET_LAUNCHER,
        token: "rl",
        cooldown: 0.8,
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
        min_ammo: 1.0,
        auto_pick: None,
    },
    WeaponSpec {
        item: Items::LIGHTNING,
        token: "lg",
        cooldown: 0.2,
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
        min_ammo: 1.0,
        auto_pick: Some(0),
    },
    // Grappling hook (rtx). Not a map pickup — handed out at spawn behind `rtx_grapple` and
    // selected by impulse, so it has no `classname`/pickup. Uses the hook viewmodel and no ammo.
    WeaponSpec {
        item: Items::GRAPPLE,
        token: "hook",
        cooldown: 0.1,
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
        min_ammo: 0.0,
        auto_pick: None,
    },
];

/// The spec for a weapon's [`Items`] bit, if it's a known weapon.
pub(crate) fn weapon_spec(item: Items) -> Option<&'static WeaponSpec> {
    WEAPON_SPECS.iter().find(|spec| spec.item == item)
}

/// The spec for a map pickup classname (`weapon_*`), if any.
pub(crate) fn weapon_spec_for_classname(classname: &str) -> Option<&'static WeaponSpec> {
    WEAPON_SPECS.iter().find(|spec| spec.classname == Some(classname))
}

/// Every weapon ownership bit (axe, both shotguns, both nailguns, GL, RL, LG, grapple) as one mask
/// — the bits `rtx_weapons` filters. Non-weapon `Items` (ammo, armor, powerups, keys) are excluded.
pub(crate) fn all_weapon_bits() -> Items {
    WEAPON_SPECS.iter().fold(Items::empty(), |mask, spec| mask | spec.item)
}

/// Parse an `rtx_weapons` token list (e.g. `"axe hook sg rl lg"`) into the mask of enabled weapon
/// [`Items`] bits. Unknown tokens (a not-yet-existing `"coil"`, typos) are silently ignored, so the
/// list stays forward-compatible with weapons added later.
pub(crate) fn enabled_weapons(list: &str) -> Items {
    list.split_whitespace()
        .filter_map(|tok| WEAPON_SPECS.iter().find(|spec| spec.token == tok))
        .fold(Items::empty(), |mask, spec| mask | spec.item)
}

/// Current amount held in an ammo pool.
pub(crate) fn ammo_count(v: &EntVars, kind: AmmoKind) -> f32 {
    match kind {
        AmmoKind::Shells => v.ammo_shells,
        AmmoKind::Nails => v.ammo_nails,
        AmmoKind::Rockets => v.ammo_rockets,
        AmmoKind::Cells => v.ammo_cells,
    }
}

/// Mutable handle to an ammo pool — for spending (`consume_ammo`) and granting (`add_ammo`) without
/// re-matching the four `ammo_*` fields by hand at each site.
pub(crate) fn ammo_field_mut(v: &mut EntVars, kind: AmmoKind) -> &mut f32 {
    match kind {
        AmmoKind::Shells => &mut v.ammo_shells,
        AmmoKind::Nails => &mut v.ammo_nails,
        AmmoKind::Rockets => &mut v.ammo_rockets,
        AmmoKind::Cells => &mut v.ammo_cells,
    }
}

/// `W_BestWeapon`'s auto-pick chain, best first (LG, SNG, SSG, NG, SG). Explosives and the
/// axe/grapple are absent (`auto_pick == None`): stock QW never auto-switches to GL/RL, and the axe
/// is the hard fallback.
pub(crate) fn auto_pick_chain() -> impl Iterator<Item = &'static WeaponSpec> {
    (0u8..).map_while(|rank| WEAPON_SPECS.iter().find(|s| s.auto_pick == Some(rank)))
}

/// The post-shot re-arm delay for a weapon's [`Items`] bit (`0` if it's not a known weapon). The
/// value `w_attack` and the nail/light loop thinks add to `attack_finished`.
pub(crate) fn cooldown_of(item: Items) -> f32 {
    weapon_spec(item).map_or(0.0, |s| s.cooldown)
}

/// The `CycleWeaponCommand` order: every impulse-selectable weapon (the eight guns; the grapple is
/// impulse-22 only, `switch_code == 0`) in table order.
pub(crate) fn cycle_order() -> impl Iterator<Item = Items> {
    WEAPON_SPECS.iter().filter(|s| s.switch_code > 0.0).map(|s| s.item)
}

/// Whether `v` lacks the ammo to fire the weapon `item` — the per-weapon fire gate, from the table's
/// `ammo_kind`/`min_ammo` (always feedable for the axe/grapple, which draw no ammo).
pub(crate) fn out_of_ammo(v: &EntVars, item: Items) -> bool {
    weapon_spec(item).is_some_and(|s| s.ammo_kind.is_some_and(|k| ammo_count(v, k) < s.min_ammo))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Pins the per-weapon fire threshold (`weapon_fed` / `w_best_weapon`'s ammo gate) against the
    /// stock QuakeC values, so the table-driven rewrites can't drift them.
    #[test]
    fn min_ammo_matches_stock() {
        let expect = [
            (Items::AXE, 0.0),
            (Items::SHOTGUN, 1.0),
            (Items::SUPER_SHOTGUN, 2.0),
            (Items::NAILGUN, 1.0),
            (Items::SUPER_NAILGUN, 2.0),
            (Items::GRENADE_LAUNCHER, 1.0),
            (Items::ROCKET_LAUNCHER, 1.0),
            (Items::LIGHTNING, 1.0),
            (Items::GRAPPLE, 0.0),
        ];
        for (item, min) in expect {
            assert_eq!(weapon_spec(item).unwrap().min_ammo, min, "min_ammo for {item:?}");
        }
    }

    /// `rtx_weapons` token parsing: known tokens fold into their `Items` bits, unknown tokens
    /// (e.g. a not-yet-existing "coil") are ignored, and an empty list enables nothing.
    #[test]
    fn enabled_weapons_parses_tokens() {
        assert_eq!(enabled_weapons("rl lg"), Items::ROCKET_LAUNCHER | Items::LIGHTNING);
        assert_eq!(enabled_weapons("hook"), Items::GRAPPLE);
        // Unknown tokens are dropped; known ones still register.
        assert_eq!(enabled_weapons("coil rl plasma"), Items::ROCKET_LAUNCHER);
        assert_eq!(enabled_weapons(""), Items::empty());
        // The default roster enables exactly the full weapon set.
        assert_eq!(enabled_weapons("axe hook sg ssg ng sng gl rl lg"), all_weapon_bits());
    }

    /// Pins `W_BestWeapon`'s auto-pick order: Lightning, Super Nailgun, Super Shotgun, Nailgun,
    /// Shotgun — explosives and the axe are never in the chain.
    #[test]
    fn auto_pick_chain_is_stock_order() {
        let chain: Vec<Items> = auto_pick_chain().map(|s| s.item).collect();
        assert_eq!(
            chain,
            vec![
                Items::LIGHTNING,
                Items::SUPER_NAILGUN,
                Items::SUPER_SHOTGUN,
                Items::NAILGUN,
                Items::SHOTGUN,
            ]
        );
    }

    /// Pins the post-shot cooldowns against the stock QuakeC `attack_finished` deltas the table now
    /// owns, so `w_attack` and the nail/light loop thinks can't drift them. (The lightning gun's
    /// opening-press 0.1 stays an explicit literal in `w_attack`; this is its per-tick rate.)
    #[test]
    fn cooldowns_match_stock() {
        let expect = [
            (Items::AXE, 0.5),
            (Items::SHOTGUN, 0.5),
            (Items::SUPER_SHOTGUN, 0.7),
            (Items::NAILGUN, 0.2),
            (Items::SUPER_NAILGUN, 0.2),
            (Items::GRENADE_LAUNCHER, 0.6),
            (Items::ROCKET_LAUNCHER, 0.8),
            (Items::LIGHTNING, 0.2),
            (Items::GRAPPLE, 0.1),
        ];
        for (item, cd) in expect {
            assert_eq!(cooldown_of(item), cd, "cooldown for {item:?}");
        }
    }

    /// Pins `CycleWeaponCommand`'s order: the eight guns in impulse order, grapple excluded.
    #[test]
    fn cycle_order_is_stock() {
        let order: Vec<Items> = cycle_order().collect();
        assert_eq!(
            order,
            vec![
                Items::AXE,
                Items::SHOTGUN,
                Items::SUPER_SHOTGUN,
                Items::NAILGUN,
                Items::SUPER_NAILGUN,
                Items::GRENADE_LAUNCHER,
                Items::ROCKET_LAUNCHER,
                Items::LIGHTNING,
            ]
        );
    }
}
