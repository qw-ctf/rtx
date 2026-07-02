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

use crate::bot;
use crate::defs::{Bits, Flags, Items, Weapon, VEC_VIEW_OFS};
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
            return WeaponChoice {
                impulse: 3,
                weapon: Weapon::SuperShotgun,
                projectile_speed: 0.0,
            };
        }
        if have(Items::SHOTGUN) && v.ammo_shells >= 1.0 {
            return WeaponChoice {
                impulse: 2,
                weapon: Weapon::Shotgun,
                projectile_speed: 0.0,
            };
        }
    }

    // Mid range: the lightning gun (fast, high DPS) when fed.
    if dist < PREFERRED_RANGE + 150.0 && have(Items::LIGHTNING) && v.ammo_cells >= 1.0 {
        return WeaponChoice {
            impulse: 8,
            weapon: Weapon::Lightning,
            projectile_speed: 0.0,
        };
    }

    // Default: the rocket launcher (projectile, lead the target).
    if have(Items::ROCKET_LAUNCHER) && v.ammo_rockets >= 1.0 {
        return WeaponChoice {
            impulse: 7,
            weapon: Weapon::RocketLauncher,
            projectile_speed: ROCKET_SPEED,
        };
    }
    // Ammo-starved fallbacks.
    if have(Items::SUPER_SHOTGUN) && v.ammo_shells >= 2.0 {
        return WeaponChoice {
            impulse: 3,
            weapon: Weapon::SuperShotgun,
            projectile_speed: 0.0,
        };
    }
    WeaponChoice {
        impulse: 1,
        weapon: Weapon::Axe,
        projectile_speed: 0.0,
    }
}

/// How long after losing sight of the enemy the bot keeps *holding the angle* where they vanished
/// (like a player holding a corner) before its eyes fall back to the navigation view.
const HOLD_ANGLE_TIME: f32 = 2.0;

/// Aim-spring stiffness (1/s) for a given skill — the single source shared with the spring
/// integrator in `bot.rs`, so the feed-forward lag estimate here matches the actual spring.
pub(crate) fn aim_omega(skill: f32) -> f32 {
    6.0 + skill * 2.0
}

/// Time for a projectile of speed `s` fired from the origin to meet a target at relative position
/// `r` moving at constant velocity `v`: the smallest positive root of `|r + v·t| = s·t`
/// (quadratic `(v·v − s²)t² + 2(r·v)t + r·r = 0`). This is what makes lead *geometry-aware*:
/// motion perpendicular to the line of fire lengthens the flight and shifts the aim a lot, motion
/// straight toward/away barely does. `None` when no positive intercept exists (target outrunning
/// the projectile).
fn intercept_time(r: Vec3, v: Vec3, s: f32) -> Option<f32> {
    let a = v.dot(v) - s * s;
    let b = 2.0 * r.dot(v);
    let c = r.dot(r);
    if a.abs() < 1e-3 {
        // Degenerate: target speed equals projectile speed — the quadratic collapses to linear.
        let t = -c / b;
        return (b.abs() > 1e-6 && t > 0.0).then_some(t);
    }
    let disc = b * b - 4.0 * a * c;
    if disc < 0.0 {
        return None;
    }
    let sq = disc.sqrt();
    let mut best = f32::INFINITY;
    for t in [(-b - sq) / (2.0 * a), (-b + sq) / (2.0 * a)] {
        if t > 0.0 && t < best {
            best = t;
        }
    }
    best.is_finite().then_some(best)
}

/// View angles (pitch, yaw, 0) from `eye` toward `point`.
fn angles_to(eye: Vec3, point: Vec3) -> Vec3 {
    let d = point - eye;
    let yaw = d.y.atan2(d.x).to_degrees();
    let pitch = -d.z.atan2(d.xy().length().max(1.0)).to_degrees();
    Vec3::new(pitch, yaw, 0.0)
}

/// Overlay combat onto the frame's decisions. `look` is the desired view (smoothed downstream by
/// the aim spring in `bot.rs`); `move_world` is the desired world-space velocity — the two are
/// independent, so the bot can run one way while looking another. With line of sight it aims
/// (leading the target, plus a smoothly drifting skill-scaled error), fights for range, and fires;
/// having *recently* lost sight it holds the angle where the enemy vanished while navigation keeps
/// it moving; otherwise it leaves the navigation view/movement untouched.
#[allow(clippy::too_many_arguments)]
pub(crate) fn engage(
    game: &mut GameState,
    e: EntId,
    enemy: EntId,
    origin: Vec3,
    now: f32,
    look: &mut Vec3,
    move_world: &mut Vec3,
    buttons: &mut i32,
    impulse: &mut i32,
) {
    let my_eye = origin + VEC_VIEW_OFS;
    let enemy_org = game.entities[enemy].v.origin;
    let enemy_eye = enemy_org + VEC_VIEW_OFS;
    let enemy_vel = game.entities[enemy].v.velocity;

    // Line of sight? Trace to the enemy's eyes, ignoring ourselves. Clear if we hit the enemy or
    // nothing at all.
    let tr = game.traceline(my_eye, enemy_eye, false, e);
    let los = tr.ent == enemy || tr.fraction > 0.95;
    if !los {
        // No shooting through walls; navigation keeps driving the movement. But if we saw the
        // enemy moments ago, hold the angle where they disappeared instead of snapping the view
        // back to the route — the human "holding the corner" look, and it kills the nav↔enemy
        // view flip-flop while line of sight flickers at an edge.
        let b = &game.entities[e].bot;
        if b.enemy_seen_time > 0.0 && now - b.enemy_seen_time < HOLD_ANGLE_TIME {
            *look = angles_to(my_eye, b.enemy_seen_at);
        }
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

    // Predicted velocity for the intercept: a grounded target is led horizontally; an airborne
    // one with its full velocity (plus the ballistic correction below — gravity isn't in the
    // linear solve).
    let grounded = game.entities[enemy].v.flags.has(Flags::ONGROUND);
    let pred_vel = if grounded {
        Vec3::new(enemy_vel.x, enemy_vel.y, 0.0)
    } else {
        enemy_vel
    };

    // Aim point. For projectiles, solve the true intercept — where the enemy *will be* when the
    // rocket arrives — instead of offsetting by the flight time to where they are *now* (which
    // under-leads exactly when it matters most: motion perpendicular to the line of fire).
    let aim = if choice.projectile_speed > 0.0 {
        let s = choice.projectile_speed;
        let t = intercept_time(enemy_eye - my_eye, pred_vel, s).unwrap_or(dist / s);
        let mut aim = enemy_eye + pred_vel * t;
        if !grounded {
            // Ballistic correction: an airborne enemy falls under gravity during the flight.
            aim.z -= 0.5 * game.host().cvar(c"sv_gravity") * t * t;
        } else if choice.weapon == Weapon::RocketLauncher && pred_vel.xy().length() > 150.0 {
            // A grounded strafer is hard to hit directly — aim at the shins so a near miss
            // becomes floor splash (the human counter to a strafing target).
            aim.z -= 38.0; // eye (+22 over origin) → shin (−16)
        }
        aim
    } else {
        enemy_eye
    };

    // Skill-scaled *drifting* aim error: the error wanders smoothly toward a periodically
    // resampled offset (never a fresh random per frame — white noise reads as jitter on the view).
    // Misses sweep past the target and drift back, like human tracking error. Pitch error is kept
    // smaller than yaw (vertical mouse control is steadier). Skill 7 ⇒ error ≈ 0.
    let skill = game.host().cvar(c"rtx_bot_skill").clamp(0.0, 7.0);
    let spread = (7.0 - skill).max(0.0); // half-range, degrees
    let frametime = game.globals.frametime;
    if now >= game.entities[e].bot.aim_err_until {
        let (r1, r2, r3) = (game.random(), game.random(), game.random());
        let b = &mut game.entities[e].bot;
        b.aim_err_target = Vec3::new((r1 - 0.5) * spread, (r2 - 0.5) * 2.0 * spread, 0.0);
        b.aim_err_until = now + 0.3 + r3 * 0.3;
    }
    let err = {
        let b = &mut game.entities[e].bot;
        let t = (4.0 * frametime).min(1.0);
        b.aim_err = b.aim_err + (b.aim_err_target - b.aim_err) * t;
        // Remember where the enemy is while we can see them, for the hold-the-angle behavior.
        b.enemy_seen_at = aim;
        b.enemy_seen_time = now;
        b.aim_err
    };

    let clean = angles_to(my_eye, aim);

    // Feed-forward: the aim spring tracks a moving solution with a steady-state lag of
    // 2·rate/ω, so on a constant strafer the crosshair would trail forever. Estimate how fast the
    // solution is moving (from last frame's clean angles) and aim ahead by the expected lag —
    // skill-scaled, so skill 7 locks onto strafers while low skill keeps trailing them.
    let ff = {
        let b = &mut game.entities[e].bot;
        let dt = now - b.look_prev_time;
        let rate = if b.look_prev_time > 0.0 && dt > 1e-3 && dt < 0.25 {
            Vec3::new(
                (bot::wrap180(clean.x - b.look_prev.x) / dt).clamp(-180.0, 180.0),
                (bot::wrap180(clean.y - b.look_prev.y) / dt).clamp(-180.0, 180.0),
                0.0,
            )
        } else {
            Vec3::ZERO // stale/first sample (just acquired the target) — no estimate yet
        };
        b.look_prev = clean;
        b.look_prev_time = now;
        rate * (2.0 / aim_omega(skill)) * (skill / 7.0)
    };

    *look = Vec3::new(clean.x + ff.x + err.x, clean.y + ff.y + err.y, 0.0);

    // Movement (world-space): hold a preferred range and strafe to dodge; retreat when hurt.
    let health = game.entities[e].v.health;
    let strafe_sign = if ((now * 0.9) + e.0 as f32).sin() >= 0.0 {
        1.0
    } else {
        -1.0
    };
    let want_forward = if health < LOW_HEALTH || dist < PREFERRED_RANGE - 100.0 {
        -MOVE_SPEED // back off
    } else if dist > PREFERRED_RANGE + 100.0 {
        MOVE_SPEED // close in
    } else {
        0.0 // hold and strafe
    };
    let dir = Vec3::new(to_enemy.x, to_enemy.y, 0.0).normalize_or_zero();
    let perp = Vec3::new(-dir.y, dir.x, 0.0);
    *move_world = dir * want_forward + perp * (strafe_sign * MOVE_SPEED);
    *buttons &= !BUTTON_JUMP; // don't bunny-hop while dueling

    // Fire only when the crosshair is on the spot. The shot leaves along the *smoothed* view
    // (`bot.aim`, last frame's spring output) — firing every frame would put rockets wherever the
    // lagging view happens to point, i.e. behind a strafer no matter how good the intercept is.
    // Humans fire when the crosshair reaches the target; the cone is weapon-based plus low-skill
    // leniency (a low-skill bot fires looser and misses, rather than never firing).
    let view = game.entities[e].bot.aim;
    let base_cone = match choice.weapon {
        Weapon::Lightning => 2.5,
        Weapon::RocketLauncher => 4.0,
        _ => 5.0,
    };
    let cone = base_cone + (7.0 - skill);
    let dp = bot::wrap180(view.x - clean.x);
    let dy = bot::wrap180(view.y - clean.y);
    let on_target = view == Vec3::ZERO || (dp * dp + dy * dy).sqrt() <= cone;
    if on_target {
        // The engine paces shots via `attack_finished`; holding fire shoots at the weapon's rate.
        *buttons |= BUTTON_ATTACK;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn intercept_leads_perpendicular_motion() {
        // Strafer at 400u moving 320 ups perpendicular to the line of fire, rocket at 1000 ups:
        // the true intercept takes longer than the naive dist/speed = 0.4s, and the solution must
        // sit exactly on the projectile sphere |r + v·t| = s·t.
        let r = Vec3::new(400.0, 0.0, 0.0);
        let v = Vec3::new(0.0, 320.0, 0.0);
        let s = 1000.0;
        let t = intercept_time(r, v, s).expect("intercept exists");
        assert!(
            t > 0.4,
            "perpendicular motion must lengthen the flight (naive 0.4), got {t}"
        );
        let miss = ((r + v * t).length() - s * t).abs();
        assert!(miss < 0.1, "intercept not on the projectile sphere: off by {miss}");

        // Radial motion is the near-no-op case: running straight away at 320 ups gives the exact
        // closing-speed time 400/(1000-320).
        let t2 = intercept_time(r, Vec3::new(320.0, 0.0, 0.0), s).unwrap();
        assert!((t2 - 400.0 / 680.0).abs() < 1e-3, "radial case wrong: {t2}");

        // Outrunnable target: no positive intercept.
        assert!(intercept_time(r, Vec3::new(1100.0, 0.0, 0.0), 1000.0).is_none());
    }
}
