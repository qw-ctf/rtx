// SPDX-License-Identifier: AGPL-3.0-or-later

//! Bot rocket jumps — the runtime side of the `navmesh::LinkKind::RocketJump` links.
//!
//! This phase supplies the **planning gate**: [`rocket_jump_extra`] tells the pathfinder how much to
//! surcharge rocket-jump links for a given bot, so one that can't currently fly a rocket jump (no
//! launcher, no rocket, too little health, or quad running) never plans a route through one. The
//! execution driver (a `HookPhase`-style machine) lands with the next phase.

use crate::abi::EntVars;
use crate::defs::{Bits, Items};
use crate::navmesh::RJ_UNFIT_PENALTY;

/// Worst-case self-damage the planner budgets for: a point-blank floor rocket, unarmored (`120`
/// radius damage, `×0.5` self). Real solved links cost a touch less (~47–50, blast a few units out),
/// so gating on the worst case keeps a bot from planning a jump it lands too hurt to survive.
const RJ_WORST_SELF_DAMAGE: f32 = 60.0;
/// Health kept in reserve above the blast — a bot won't rocket-jump itself down to the wire, since it
/// often arrives into a fight (the conservative policy).
const RJ_HEALTH_MARGIN: f32 = 25.0;

/// `0.0` when this bot can fly a rocket-jump leg right now, else [`RJ_UNFIT_PENALTY`]. Unfit when it
/// lacks the rocket launcher or a rocket, has too little health for the worst-case self-blast (after
/// armor, mirroring `t_damage`'s `save = ceil(armortype·dmg)` clamped to `armorvalue`), or is running
/// **quad** — `t_damage` applies quad *before* the mode split, so a self-rocket under quad deals
/// (and knocks back) 4×, which is both lethal and off-model for the solved arc.
pub(crate) fn rocket_jump_extra(v: &EntVars, quad_until: f32, now: f32) -> f32 {
    let has_rl = v.items.has(Items::ROCKET_LAUNCHER);
    let quad = quad_until > now;
    // Armor absorbs the health share of the blast (not the knockback), exactly as `t_damage` does.
    let armor_save = (v.armortype * RJ_WORST_SELF_DAMAGE).ceil().min(v.armorvalue);
    let effective = RJ_WORST_SELF_DAMAGE - armor_save;
    let fit = has_rl && v.ammo_rockets >= 1.0 && !quad && v.health > effective + RJ_HEALTH_MARGIN;
    if fit {
        0.0
    } else {
        RJ_UNFIT_PENALTY
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn vars(items: Items, rockets: f32, health: f32, armortype: f32, armorvalue: f32) -> EntVars {
        EntVars {
            items: items.as_f32(),
            ammo_rockets: rockets,
            health,
            armortype,
            armorvalue,
            ..Default::default()
        }
    }

    #[test]
    fn fit_and_unfit_cases() {
        let rl = Items::ROCKET_LAUNCHER;
        // Healthy, armed, no quad → fit.
        assert_eq!(rocket_jump_extra(&vars(rl, 5.0, 100.0, 0.0, 0.0), 0.0, 1.0), 0.0);
        // No launcher → unfit.
        assert_eq!(rocket_jump_extra(&vars(Items::empty(), 5.0, 100.0, 0.0, 0.0), 0.0, 1.0), RJ_UNFIT_PENALTY);
        // No rocket → unfit.
        assert_eq!(rocket_jump_extra(&vars(rl, 0.0, 100.0, 0.0, 0.0), 0.0, 1.0), RJ_UNFIT_PENALTY);
        // Too little health unarmored (needs > 60 + 25 = 85) → 80 unfit, 90 fit.
        assert_eq!(rocket_jump_extra(&vars(rl, 5.0, 80.0, 0.0, 0.0), 0.0, 1.0), RJ_UNFIT_PENALTY);
        assert_eq!(rocket_jump_extra(&vars(rl, 5.0, 90.0, 0.0, 0.0), 0.0, 1.0), 0.0);
        // Quad running → unfit even when otherwise healthy.
        assert_eq!(rocket_jump_extra(&vars(rl, 5.0, 100.0, 0.0, 0.0), 5.0, 1.0), RJ_UNFIT_PENALTY);
    }

    #[test]
    fn armor_lowers_the_health_bar() {
        let rl = Items::ROCKET_LAUNCHER;
        // Yellow armor (0.6, plenty of value): save = ceil(0.6·60) = 36 → effective 24, bar 24+25=49.
        // So 50 health is fit, 45 is not — armor makes rocket jumps viable at lower health.
        assert_eq!(rocket_jump_extra(&vars(rl, 5.0, 50.0, 0.6, 100.0), 0.0, 1.0), 0.0);
        assert_eq!(rocket_jump_extra(&vars(rl, 5.0, 45.0, 0.6, 100.0), 0.0, 1.0), RJ_UNFIT_PENALTY);
    }
}
