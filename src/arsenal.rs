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

/// Current amount held in an ammo pool.
pub(crate) fn ammo_count(v: &EntVars, kind: AmmoKind) -> f32 {
    match kind {
        AmmoKind::Shells => v.ammo_shells,
        AmmoKind::Nails => v.ammo_nails,
        AmmoKind::Rockets => v.ammo_rockets,
        AmmoKind::Cells => v.ammo_cells,
    }
}

/// `W_BestWeapon`'s auto-pick chain, best first (LG, SNG, SSG, NG, SG). Explosives and the
/// axe/grapple are absent (`auto_pick == None`): stock QW never auto-switches to GL/RL, and the axe
/// is the hard fallback.
pub(crate) fn auto_pick_chain() -> impl Iterator<Item = &'static WeaponSpec> {
    (0u8..).map_while(|rank| WEAPON_SPECS.iter().find(|s| s.auto_pick == Some(rank)))
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
}
