//! Weapon handling, ported from `qw-qc/weapons.qc`. This slice covers the per-frame
//! weapon loop and impulse weapon switching; firing (`W_Attack` and the individual fire
//! functions) lands in the next slice.

use core::ffi::CStr;

use crate::defs::*;
use crate::entity::EntId;
use crate::game::GameState;

impl GameState {
    /// `W_WeaponFrame` — run once per `PlayerPostThink`: process impulses and, when the
    /// attack button is held and the weapon is ready, fire.
    pub(crate) fn w_weapon_frame(&mut self, e: EntId) {
        if self.globals.time < self.entities[e.index()].attack_finished {
            return;
        }

        self.impulse_commands(e);

        if self.entities[e.index()].v.button0 != 0.0 {
            // W_Attack (firing) is implemented in the next slice.
        }
    }

    /// `ImpulseCommands` — dispatch the pending impulse, then clear it.
    fn impulse_commands(&mut self, e: EntId) {
        let impulse = self.entities[e.index()].v.impulse as i32;
        if (1..=8).contains(&impulse) {
            self.w_change_weapon(e);
        }
        // Impulses 9-12 (cheat / cycle / serverflags) arrive in a later slice.
        self.entities[e.index()].v.impulse = 0.0;
    }

    /// `W_ChangeWeapon` — select the weapon for the current impulse if owned and fed.
    fn w_change_weapon(&mut self, e: EntId) {
        let (weapon, lacks_ammo) = {
            let v = &self.entities[e.index()].v;
            match v.impulse as i32 {
                1 => (IT_AXE, false),
                2 => (IT_SHOTGUN, v.ammo_shells < 1.0),
                3 => (IT_SUPER_SHOTGUN, v.ammo_shells < 2.0),
                4 => (IT_NAILGUN, v.ammo_nails < 1.0),
                5 => (IT_SUPER_NAILGUN, v.ammo_nails < 2.0),
                6 => (IT_GRENADE_LAUNCHER, v.ammo_rockets < 1.0),
                7 => (IT_ROCKET_LAUNCHER, v.ammo_rockets < 1.0),
                8 => (IT_LIGHTNING, v.ammo_cells < 1.0),
                _ => (0.0, false),
            }
        };

        let owns = self.entities[e.index()].v.items as i32 & weapon as i32 != 0;
        if !owns {
            self.sprint_to(e, c"no weapon.\n");
            return;
        }
        if lacks_ammo {
            self.sprint_to(e, c"not enough ammo.\n");
            return;
        }

        self.entities[e.index()].v.weapon = weapon;
        self.w_set_current_ammo(e);
    }

    /// `sprint(self, PRINT_HIGH, ...)` to a player.
    fn sprint_to(&self, e: EntId, msg: &CStr) {
        self.host.sprint(e.0 as i32, PRINT_HIGH, msg);
    }
}
