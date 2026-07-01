// SPDX-License-Identifier: AGPL-3.0-or-later

//! Bot combat — the shooting/dodging layer the navmesh bots gained for Rocket Arena (the
//! README's deferred "combat is planned next"). It is mode-agnostic: [`crate::bot::run_bot`]
//! calls [`engage`] whenever the active mode hands it an enemy via
//! [`GameMode::bot_enemy`](crate::mode::GameMode::bot_enemy), so any future mode (instagib, CTF,
//! …) reuses it for free.
//!
//! Bots run their usercmd through the engine's player-move + weapon code just like humans, so
//! "combat" here is purely a matter of choosing the view angles, weapon (an `impulse`), the
//! attack button, and evasive movement — never a direct weapon-fire call. [`engage`] overlays
//! those onto the movement the navmesh already produced: while the bot has no line of sight it
//! keeps navigating toward the enemy untouched; once it can see them it aims (leading the target
//! for projectiles), picks a weapon by range, strafes/retreats, and fires.

use glam::{Vec3, Vec3Swizzles};

use crate::defs::{Bits, Items, Weapon, VEC_VIEW_OFS};
use crate::entity::EntId;
use crate::game::GameState;

const BUTTON_ATTACK: i32 = 1;
const BUTTON_JUMP: i32 = 2;
/// Move-component scale (as in `bot.rs`: pmove clamps to `sv_maxspeed`).
const MOVE_SPEED: f32 = 800.0;
/// Rocket/grenade projectile speed (QuakeWorld `SV_FireRocket`), for target leading.
const ROCKET_SPEED: f32 = 1000.0;
/// Preferred fighting distance for the rocket launcher — close enough to hit, far enough to dodge
/// the reply and not splash ourselves.
const PREFERRED_RANGE: f32 = 400.0;
/// Below this we're in self-splash territory for the RL — switch to the super shotgun.
const SPLASH_RANGE: f32 = 140.0;
/// Retreat when hurt below this.
const LOW_HEALTH: f32 = 40.0;

/// A weapon choice for the current range: the impulse that selects it, the [`Weapon`] it yields
/// (to avoid re-selecting what we already hold), and its projectile speed (`0` = hitscan, so no
/// target leading).
struct WeaponChoice {
    impulse: i32,
    weapon: Weapon,
    projectile_speed: f32,
}

/// Pick a weapon for `dist`, given what the bot owns and has ammo for.
fn choose_weapon(g: &GameState, e: EntId, dist: f32) -> WeaponChoice {
    let v = &g.entities[e].v;
    let items = v.items;
    let have = |bit: Items| items.has(bit);

    // Point blank: the super shotgun (hitscan, no self-splash). Fall back to the axe if somehow
    // unarmed (audience never gets here).
    if dist < SPLASH_RANGE {
        if have(Items::SUPER_SHOTGUN) && v.ammo_shells >= 2.0 {
            return WeaponChoice { impulse: 3, weapon: Weapon::SuperShotgun, projectile_speed: 0.0 };
        }
        if have(Items::SHOTGUN) && v.ammo_shells >= 1.0 {
            return WeaponChoice { impulse: 2, weapon: Weapon::Shotgun, projectile_speed: 0.0 };
        }
    }

    // Mid range: the lightning gun (fast, high DPS) when fed.
    if dist < PREFERRED_RANGE + 150.0 && have(Items::LIGHTNING) && v.ammo_cells >= 1.0 {
        return WeaponChoice { impulse: 8, weapon: Weapon::Lightning, projectile_speed: 0.0 };
    }

    // Default: the rocket launcher (projectile, lead the target).
    if have(Items::ROCKET_LAUNCHER) && v.ammo_rockets >= 1.0 {
        return WeaponChoice { impulse: 7, weapon: Weapon::RocketLauncher, projectile_speed: ROCKET_SPEED };
    }
    // Ammo-starved fallbacks.
    if have(Items::SUPER_SHOTGUN) && v.ammo_shells >= 2.0 {
        return WeaponChoice { impulse: 3, weapon: Weapon::SuperShotgun, projectile_speed: 0.0 };
    }
    WeaponChoice { impulse: 1, weapon: Weapon::Axe, projectile_speed: 0.0 }
}

/// Overlay combat onto the already-computed movement command. Only takes over once the bot has a
/// clear line of sight to `enemy`; before that it leaves the navmesh command alone so the bot
/// keeps pathing toward its target.
#[allow(clippy::too_many_arguments)]
pub(crate) fn engage(
    game: &mut GameState,
    e: EntId,
    enemy: EntId,
    origin: Vec3,
    now: f32,
    angles: &mut Vec3,
    forward: &mut i32,
    side: &mut i32,
    buttons: &mut i32,
    impulse: &mut i32,
) {
    let my_eye = origin + VEC_VIEW_OFS;
    let enemy_org = game.entities[enemy].v.origin;
    let enemy_eye = enemy_org + VEC_VIEW_OFS;
    let enemy_vel = game.entities[enemy].v.velocity;

    // Line of sight? Trace to the enemy's eyes, ignoring ourselves. Clear if we hit the enemy or
    // nothing at all. Without LOS, leave navigation in charge (no shooting through walls).
    let tr = game.traceline(my_eye, enemy_eye, false, e);
    let los = tr.ent == enemy || tr.fraction > 0.95;
    if !los {
        return;
    }

    let to_enemy = enemy_eye - my_eye;
    let dist = to_enemy.length().max(1.0);
    let choice = choose_weapon(game, e, dist);

    // Switch weapon only when we don't already hold the desired one (setting `impulse` re-runs
    // W_ChangeWeapon each frame otherwise).
    if game.entities[e].v.weapon != choice.weapon {
        *impulse = choice.impulse;
    }

    // Aim point: lead the target for projectiles (constant-velocity prediction), direct for
    // hitscan.
    let aim = if choice.projectile_speed > 0.0 {
        let lead = dist / choice.projectile_speed;
        enemy_eye + enemy_vel * lead
    } else {
        enemy_eye
    };

    // Skill-scaled aim error: higher `rtx_bot_skill` → tighter aim.
    let skill = game.host().cvar(c"rtx_bot_skill").clamp(0.0, 7.0);
    let spread = (7.0 - skill).max(0.0); // half-range, degrees
    let err_yaw = (game.random() - 0.5) * 2.0 * spread;
    let err_pitch = (game.random() - 0.5) * 2.0 * spread;

    let d = aim - my_eye;
    let yaw = d.y.atan2(d.x).to_degrees() + err_yaw;
    let pitch = -d.z.atan2(d.xy().length()).to_degrees() + err_pitch;
    *angles = Vec3::new(pitch, yaw, 0.0);

    // Movement in the enemy-facing frame: hold a preferred range and strafe to dodge. Retreat
    // when hurt.
    let health = game.entities[e].v.health;
    let strafe_sign = if ((now * 0.9) + e.0 as f32).sin() >= 0.0 { 1.0 } else { -1.0 };
    let want_forward = if health < LOW_HEALTH || dist < PREFERRED_RANGE - 100.0 {
        -MOVE_SPEED // back off
    } else if dist > PREFERRED_RANGE + 100.0 {
        MOVE_SPEED // close in
    } else {
        0.0 // hold and strafe
    };
    *forward = want_forward as i32;
    *side = (strafe_sign * MOVE_SPEED) as i32;
    *buttons &= !BUTTON_JUMP; // don't bunny-hop while dueling

    // Fire: LOS is clear and we're aimed at the enemy. The engine paces shots via
    // `attack_finished`, so holding the button each frame fires at the weapon's rate.
    *buttons |= BUTTON_ATTACK;
}
