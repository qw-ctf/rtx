//! Central callback dispatch.
//!
//! QuakeC stores behaviour in function-pointer fields (`.think`, `.touch`, `.use`,
//! `.blocked`, `.th_pain`, `.th_die`). We model each as a `Copy` enum (see `entity.rs`) and
//! resolve them here with one exhaustive `match` apiece — so every behaviour is accounted
//! for at compile time. Engine entry points (`GAME_EDICT_THINK`/`TOUCH`/`BLOCKED`) and the
//! game logic both route through these.

use crate::entity::{Blocked, Die, EntId, Pain, Think, Touch, Use};
use crate::game::GameState;

impl GameState {
    /// Run an entity's scheduled `think` (`GAME_EDICT_THINK`).
    pub(crate) fn run_think(&mut self, e: EntId) {
        let think = self.entities[e].think;
        self.run_think_now(e, think);
    }

    /// Run a specific think on `e` immediately (used for `think1` chaining in subs).
    pub(crate) fn run_think_now(&mut self, e: EntId, think: Think) {
        use Think::*;
        match think {
            None => {}
            // subs.qc
            SubCalcMoveDone => self.sub_calc_move_done(e),
            SubRemove => self.sub_remove(e),
            DelayedUse => self.delayed_use(e),
            // player.qc
            PlayerStand => self.player_stand1(e),
            PlayerRun => self.player_run(e),
            PlayerAnim => self.player_anim_tick(e),
            PlayerDead => self.player_dead(e),
            PlayerWeaponAnim => self.player_weapon_anim(e),
            PlayerNail => self.player_nail(e),
            PlayerLight => self.player_light(e),
            // weapons.qc
            GrenadeExplode => self.grenade_explode(e),
            // items.qc
            SubRegen => self.sub_regen(e),
            PlaceItem => self.place_item(e),
            MegaHealthRot => self.mega_health_rot(e),
            // triggers.qc
            MultiWait => self.multi_wait(e),
            HurtOn => self.hurt_on(e),
            PlayTeleport => self.play_teleport(e),
            ExecuteChangelevel => self.execute_changelevel(),
            // buttons.qc
            ButtonWait => self.button_wait(e),
            ButtonReturn => self.button_return(e),
            ButtonDone => self.button_done(e),
            // doors.qc
            DoorLink => self.door_link(e),
            DoorGoDown => self.door_go_down(e),
            DoorHitTop => self.door_hit_top(e),
            DoorHitBottom => self.door_hit_bottom(e),
            // plats.qc
            PlatGoDown => self.plat_go_down(e),
            PlatHitTop => self.plat_hit_top(e),
            PlatHitBottom => self.plat_hit_bottom(e),
            TrainNext => self.train_next(e),
            TrainWait => self.train_wait(e),
            FuncTrainFind => self.func_train_find(e),
        }
    }

    /// Run an entity's `touch` (`GAME_EDICT_TOUCH`); `other` is the toucher.
    pub(crate) fn run_touch(&mut self, e: EntId, other: EntId) {
        use Touch::*;
        match self.entities[e].touch {
            None => {}
            // weapons.qc projectiles
            Missile => self.t_missile_touch(e, other),
            Grenade => self.grenade_touch(e, other),
            Spike => self.spike_touch(e, other, false),
            SuperSpike => self.spike_touch(e, other, true),
            // items.qc
            ItemHealth => self.health_touch(e, other),
            ItemArmor => self.armor_touch(e, other),
            ItemWeapon => self.weapon_touch(e, other),
            ItemAmmo => self.ammo_touch(e, other),
            ItemPowerup => self.powerup_touch(e, other),
            Backpack => self.backpack_touch(e, other),
            // triggers.qc
            Multi => self.multi_touch(e, other),
            Teleport => self.teleport_touch(e, other),
            Hurt => self.hurt_touch(e, other),
            Push => self.trigger_push_touch(e, other),
            Tdeath => self.tdeath_touch(e, other),
            // buttons.qc
            ButtonTouch => self.button_touch(e, other),
            // doors.qc
            DoorTouch => self.door_touch(e, other),
            DoorTriggerField => self.door_trigger_touch(e, other),
            // plats.qc
            PlatCenter => self.plat_center_touch(e, other),
            // server.qc
            Changelevel => self.changelevel_touch(e, other),
            // trigger_monsterjump only affects monsters (absent in this subset).
            TriggerMonsterjump => {
                let _ = other;
            }
        }
    }

    /// Run an entity's `use`. `self.activator` should already be set.
    pub(crate) fn run_use(&mut self, e: EntId) {
        use Use::*;
        match self.entities[e].use_ {
            None => {}
            // triggers.qc
            MultiUse => self.multi_use(e),
            CounterUse => self.counter_use(e),
            TeleportUse => self.teleport_use(e),
            TriggerRelay => self.sub_use_targets(e),
            // buttons.qc
            ButtonUse => self.button_use(e),
            // doors.qc
            DoorUse => self.door_use(e),
            // plats.qc
            PlatTrigger => self.plat_trigger_use(e),
            PlatUse => self.plat_use(e),
            TrainUse => self.train_use(e),
            // misc.qc
            LightUse => self.light_use(e),
            FuncWallUse => self.func_wall_use(e),
        }
    }

    /// Run an entity's `blocked` (`GAME_EDICT_BLOCKED`); `other` is the obstruction.
    pub(crate) fn run_blocked(&mut self, e: EntId, other: EntId) {
        use Blocked::*;
        match self.entities[e].blocked {
            None => {}
            DoorBlocked => self.door_blocked(e, other),
            PlatBlocked => self.plat_crush(e, other),
            TrainBlocked => self.train_blocked(e, other),
        }
    }

    /// Run an entity's `th_pain` (from `T_Damage`).
    pub(crate) fn run_pain(&mut self, e: EntId, attacker: EntId, damage: f32) {
        use Pain::*;
        match self.entities[e].th_pain {
            None => {}
            Player => self.player_pain(e, attacker, damage),
        }
    }

    /// Run an entity's `th_die` (from `Killed`).
    pub(crate) fn run_die(&mut self, e: EntId) {
        use Die::*;
        match self.entities[e].th_die {
            None => {}
            Player => self.player_die(e),
            GrenadeExplode => self.grenade_explode(e),
            TriggerKilled => self.multi_killed(e),
            ButtonKilled => self.button_killed(e),
            DoorKilled => self.door_killed(e),
            // misc.qc
            ExploBoxDie => self.barrel_explode(e),
        }
    }
}
