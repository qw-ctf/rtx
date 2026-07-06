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

use crate::bot::{self, BotCmd};
use crate::defs::{
    Bits, Flags, Items, Weapon, BOT_MOVE_SPEED as MOVE_SPEED, BUTTON_ATTACK, BUTTON_JUMP,
    VEC_VIEW_OFS,
};
use crate::entity::EntId;
use crate::game::GameState;

/// Rocket/grenade projectile speed (QuakeWorld `SV_FireRocket`), for target leading.
const ROCKET_SPEED: f32 = 1000.0;
/// Preferred fighting distance for the rocket launcher — close enough to hit, far enough to dodge
/// the reply and not splash ourselves.
const PREFERRED_RANGE: f32 = 400.0;
/// Below this we're in self-splash territory for the RL — switch to the super shotgun.
const SPLASH_RANGE: f32 = 140.0;
/// How far short of the aim point a projectile may land and still count as a clear shot. A wall
/// that stops the rocket more than this before `aim` means the muzzle→aim path is blocked (corner
/// self-splash; blast radius is 160 and attacker self-damage is only halved). Matches the slack in
/// [`crate::bot::grenade::rocket_shove`].
const LINE_OF_FIRE_SLACK: f32 = 48.0;
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
    // Ammo-starved fallbacks: pick the best owned gun with ammo before resorting to the axe. This
    // is also the *only* branch a bot with just the stock loadout (shotgun + axe) reaches at range,
    // so without the shotgun/lightning arms here it would roam throwing the axe at distant enemies.
    if have(Items::SUPER_SHOTGUN) && v.ammo_shells >= 2.0 {
        return WeaponChoice {
            impulse: 3,
            weapon: Weapon::SuperShotgun,
            projectile_speed: 0.0,
        };
    }
    if have(Items::LIGHTNING) && v.ammo_cells >= 1.0 {
        return WeaponChoice {
            impulse: 8,
            weapon: Weapon::Lightning,
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

/// Multiplier on the base aim spread from three human tracking factors, all ≥ 1 so they only ever
/// *widen* the error: **convergence** (loose on first sight at `visible_for = 0`, tightening below 1
/// over ~1.5s of continuous line of sight), **own motion** (worse while running, up to +40% at
/// 320ups), and **target crossing** (worse the faster the target moves across the line of fire,
/// `perp_speed/dist` ≈ angular rate). Pure, so the clamps/monotonicity are unit-testable; skill's
/// contribution stays in the base spread (so skill 7 ⇒ base 0 ⇒ spread 0 regardless of this).
fn spread_scale(visible_for: f32, own_speed: f32, perp_speed: f32, dist: f32) -> f32 {
    let converge = 1.6 + (0.7 - 1.6) * (visible_for / 1.5).clamp(0.0, 1.0);
    let move_factor = 1.0 + 0.4 * (own_speed / 320.0).min(1.0);
    let track_factor = 1.0 + 0.5 * (perp_speed / dist.max(1.0)).min(1.0);
    converge * move_factor * track_factor
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
pub(crate) fn angles_to(eye: Vec3, point: Vec3) -> Vec3 {
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
pub(crate) fn engage(
    game: &mut GameState,
    e: EntId,
    enemy: EntId,
    origin: Vec3,
    now: f32,
    cmd: &mut BotCmd,
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
            cmd.look = angles_to(my_eye, b.enemy_seen_at);
        }
        return;
    }

    let to_enemy = enemy_eye - my_eye;
    let dist = to_enemy.length().max(1.0);
    let choice = choose_weapon(game, e, dist);

    // Switch weapon only when we don't already hold the desired one (setting `impulse` re-runs
    // W_ChangeWeapon each frame otherwise).
    if game.entities[e].v.weapon != choice.weapon {
        cmd.impulse = choice.impulse;
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
    // Base half-range shrinks with skill (skill 7 ⇒ 0 ⇒ perfect), then widens with three human
    // tracking factors, so first-glimpse and running snap-shots are looser than a settled duel:
    //  • convergence — loose on first sight, tightening over ~1.5s of continuous line of sight
    //    (`vis_since`, set by perception); the reaction delay already removed the insta-lock tell.
    //  • own motion — harder to aim while running/bhopping.
    //  • target crossing — a fast perpendicular mover is harder to track than a stationary one.
    let base_spread = (7.0 - skill).max(0.0);
    let visible_for = {
        let vs = game.entities[e].bot.vis_since;
        if vs > 0.0 {
            now - vs
        } else {
            0.0
        }
    };
    let own_speed = game.entities[e].v.velocity.xy().length();
    let perp_speed = {
        let los_dir = to_enemy / dist;
        (enemy_vel - los_dir * enemy_vel.dot(los_dir)).length() // target motion across the line of fire
    };
    let spread = base_spread * spread_scale(visible_for, own_speed, perp_speed, dist);
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

    cmd.look = Vec3::new(clean.x + ff.x + err.x, clean.y + ff.y + err.y, 0.0);

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
    cmd.move_world = dir * want_forward + perp * (strafe_sign * MOVE_SPEED);
    cmd.buttons &= !BUTTON_JUMP; // don't bunny-hop while dueling

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

    // Line-of-fire clearance for projectiles (in practice only the RL — LG/SG are hitscan, speed 0).
    // The eye→enemy_eye gate at the top says the *enemy* is visible, but the rocket doesn't leave
    // the eye: it spawns at the muzzle (`origin + fwd*8 + (0,0,16)`; see weapons.rs `w_fire_rocket`)
    // and flies at the led/shin-dropped `aim` point, not the enemy's eyes. A peeker around a corner
    // can pass the eye ray while the muzzle→aim path clips the corner and self-splashes. So trace
    // the real line of fire and hold fire when a wall stops the rocket short of `aim`.
    //
    // Trace along `clean` (the geometric muzzle→aim direction), not the smoothed `bot.aim`: this
    // judges the *intended* shot and stays steady frame-to-frame rather than chattering with the aim
    // spring's lag and drifting error. `on_target` already keeps the emitted view within `cone` of
    // `clean`, so over the 8u muzzle offset the two directions differ by well under a unit.
    let lof_clear = if choice.projectile_speed > 0.0 {
        let fwd = bot::angle_vectors(clean).0;
        let muzzle = origin + fwd * 8.0 + Vec3::new(0.0, 0.0, 16.0);
        let tr = game.traceline(muzzle, aim, false, e);
        // Enemy body in the way ⇒ a direct hit, clear. Otherwise clear only if nothing stopped the
        // rocket appreciably short of `aim`.
        tr.ent == enemy || (muzzle + (aim - muzzle) * tr.fraction - aim).length() <= LINE_OF_FIRE_SLACK
    } else {
        true // hitscan: the eye-ray LoS above already governs the shot
    };
    if on_target && lof_clear {
        // The engine paces shots via `attack_finished`; holding fire shoots at the weapon's rate.
        cmd.buttons |= BUTTON_ATTACK;
    }
}

// --- shootable-grenade tactics -----------------------------------------------------------------

/// Grenade blast: damage at the centre and the radius over which it falls off (`grenade_explode`
/// deals 120 over `damage + 40` units; see `combat.rs::t_radius_damage`).
const GRENADE_BLAST_DAMAGE: f32 = 120.0;
pub(crate) const GRENADE_BLAST_RADIUS: f32 = 160.0;
/// How near a grenade must be for a bot to notice it at all.
const GRENADE_AWARE: f32 = 320.0;
/// Never shoot a grenade closer than this — detonating it point-blank is worse than the threat.
pub(crate) const GRENADE_MIN_SHOOT: f32 = 100.0;
/// Only shoot a grenade to disarm/airburst it if the splash we'd eat is at most this share of our
/// health — a low-health bot only detonates ones already outside its own blast, a healthy one will
/// trade a little splash for the disarm (the "far enough vs health" call).
pub(crate) const GRENADE_SHOOT_HEALTH_FRAC: f32 = 0.5;

/// Splash a blast at `dist` from its centre would deal to a player (linear falloff to the radius).
pub(crate) fn blast_self_damage(dist: f32) -> f32 {
    if dist < GRENADE_BLAST_RADIUS {
        (GRENADE_BLAST_DAMAGE - 0.5 * dist).max(0.0)
    } else {
        0.0
    }
}

/// The best hitscan weapon the bot owns and can feed, for detonating a grenade precisely: the
/// lightning beam first (a continuous line, most reliable on the 8u hit radius), then the shotguns.
pub(crate) fn hitscan_choice(g: &GameState, e: EntId) -> Option<(i32, Weapon)> {
    let v = &g.entities[e].v;
    if v.items.has(Items::LIGHTNING) && v.ammo_cells >= 1.0 {
        Some((8, Weapon::Lightning))
    } else if v.items.has(Items::SUPER_SHOTGUN) && v.ammo_shells >= 2.0 {
        Some((3, Weapon::SuperShotgun))
    } else if v.items.has(Items::SHOTGUN) && v.ammo_shells >= 1.0 {
        Some((2, Weapon::Shotgun))
    } else {
        None
    }
}

/// Whether a living teammate (other than `e`) stands within a blast at `pos` — so we don't detonate
/// a grenade on our own side. Always false in non-team play (`my_team == 0`).
pub(crate) fn teammate_in_blast(g: &GameState, e: EntId, my_team: u8, pos: Vec3) -> bool {
    if my_team == 0 {
        return false;
    }
    let maxclients = g.host().cvar(c"maxclients") as i32;
    (1..=maxclients as u32).map(EntId).any(|p| {
        let ent = &g.entities[p];
        p != e
            && ent.in_use
            && ent.classname() == Some("player")
            && ent.v.health > 0.0
            && ent.mode_p.team == my_team
            && (ent.v.origin - pos).length() < GRENADE_BLAST_RADIUS
    })
}

/// Whether the bot has a clear line to the grenade's centre (so a shot could actually reach it).
pub(crate) fn can_hit_grenade(game: &mut GameState, e: EntId, grenade: EntId) -> bool {
    let from = game.entities[e].v.origin + VEC_VIEW_OFS;
    let to = game.entities[grenade].v.origin;
    let tr = game.traceline(from, to, false, e);
    tr.fraction > 0.9 || tr.ent == grenade
}

/// Aim at a grenade and fire a hitscan shot to detonate it; select the gun first if needed. Returns
/// `false` (didn't commit) if the bot has no usable hitscan weapon. The shot leaves along the
/// *smoothed* view, so it fires only once that view has swung onto the grenade (and the gun is in
/// hand), matching how `engage` gates its shots.
pub(crate) fn shoot_grenade(game: &mut GameState, e: EntId, grenade: EntId, cmd: &mut BotCmd) -> bool {
    let Some((imp, weapon)) = hitscan_choice(game, e) else {
        return false;
    };
    let eye = game.entities[e].v.origin + VEC_VIEW_OFS;
    let gpos = game.entities[grenade].v.origin;
    cmd.look = angles_to(eye, gpos);
    if game.entities[e].v.weapon != weapon {
        cmd.impulse = imp; // switching takes a frame; fire once we hold it
        return true;
    }
    // The detonation line check accepts a shot passing within 8u of the grenade — convert that to an
    // angular tolerance at this range so the bot fires as soon as its aim is close enough to connect.
    let dist = (gpos - eye).length().max(1.0);
    let cone = (8.0 / dist).atan().to_degrees().clamp(1.5, 5.0);
    let view = game.entities[e].bot.aim;
    let dp = bot::wrap180(view.x - cmd.look.x);
    let dy = bot::wrap180(view.y - cmd.look.y);
    if view == Vec3::ZERO || (dp * dp + dy * dy).sqrt() <= cone {
        cmd.buttons |= BUTTON_ATTACK;
    }
    true
}

/// React to live shootable grenades, overlaid *after* [`engage`]. Two uses of the blast's area of
/// effect, both weighed against our own splash so we never blow ourselves up:
///
/// - **Defensive** — the nearest enemy grenade that's within (or heading into) our blast. If we can
///   detonate it at a safe distance (the splash we'd take is a small share of our health) we shoot
///   it down; if it's too close for that, we **run and hop away** instead.
/// - **Offensive** — a grenade sitting on the current enemy (and clear of our teammates): a free
///   airburst, shot from outside its blast.
///
/// Everything routes through the frame's [`BotCmd`]; the shot detonates the grenade through the
/// engine's `shootable_grenade_on_line`/`t_damage` path.
pub(crate) fn grenade_tactics(
    game: &mut GameState,
    e: EntId,
    enemy: Option<EntId>,
    origin: Vec3,
    cmd: &mut BotCmd,
) -> bool {
    if !game.host().cvar_bool(c"rtx_shootable_grenades") {
        return false; // grenades aren't hittable (and are point-size), so there's nothing to exploit
    }
    let live: Vec<EntId> = game
        .entities
        .iter()
        .enumerate()
        .filter(|(_, ent)| ent.classname() == Some("grenade") && ent.in_use && ent.combat.voided == 0.0)
        .map(|(i, _)| EntId(i as u32))
        .collect();
    if live.is_empty() {
        return false;
    }
    let my_team = game.entities[e].mode_p.team;
    let health = game.entities[e].v.health.max(1.0);
    // A grenade this bot is running as a lob→shoot combo (see `super::grenade`): don't let the
    // opportunistic offence below detonate it early — that would blow it *short* of the enemy and
    // shove them the wrong way. The combo driver detonates it at the right moment itself.
    let combo_grenade = game.entities[e].bot.grenade_ent;

    // Nearest threatening enemy grenade (defence), and the nearest grenade sitting on our enemy that
    // we can safely detonate (offence).
    let mut threat: Option<(EntId, f32)> = None; // (grenade, dist to us)
    let mut offense: Option<(EntId, f32)> = None;
    for grenade in live {
        let gpos = game.entities[grenade].v.origin;
        let gvel = game.entities[grenade].v.velocity;
        let d = (gpos - origin).length();
        if d > GRENADE_AWARE {
            continue;
        }
        let owner = game.entities[grenade].owner();
        let ally = owner == e || (my_team != 0 && owner.is_some() && game.entities[owner].mode_p.team == my_team);
        if !ally {
            // A threat if it's already within our splash, or approaching us from range.
            let approaching = (origin - gpos).dot(gvel) > 0.0;
            if (d < GRENADE_BLAST_RADIUS + 40.0 || approaching) && threat.is_none_or(|(_, bd)| d < bd) {
                threat = Some((grenade, d));
            }
        }
        if let Some(en) = enemy {
            let on_enemy = (game.entities[en].v.origin - gpos).length() < GRENADE_BLAST_RADIUS;
            if grenade.0 != combo_grenade
                && on_enemy
                && blast_self_damage(d) <= health * GRENADE_SHOOT_HEALTH_FRAC
                && !teammate_in_blast(game, e, my_team, gpos)
                && offense.is_none_or(|(_, bd)| d < bd)
            {
                offense = Some((grenade, d));
            }
        }
    }

    // Defence takes priority — survival first.
    if let Some((grenade, d)) = threat {
        let safe_to_shoot = d >= GRENADE_MIN_SHOOT && blast_self_damage(d) <= health * GRENADE_SHOOT_HEALTH_FRAC;
        if safe_to_shoot && can_hit_grenade(game, e, grenade) && shoot_grenade(game, e, grenade, cmd) {
            return true;
        }
        // Too close (or no clear shot / no hitscan gun): run directly away and hop off the ground —
        // put distance between us and the blast rather than setting it off in our face.
        let gpos = game.entities[grenade].v.origin;
        let away = Vec3::new(origin.x - gpos.x, origin.y - gpos.y, 0.0).normalize_or_zero();
        cmd.move_world = away * MOVE_SPEED;
        if game.entities[e].v.flags.has(Flags::ONGROUND) {
            cmd.buttons |= BUTTON_JUMP;
        }
        return true;
    }

    // Offence: airburst a grenade sitting on the enemy.
    if let Some((grenade, _)) = offense {
        if can_hit_grenade(game, e, grenade) {
            shoot_grenade(game, e, grenade, cmd);
        }
    }
    false
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Blast falloff: full damage at the centre, tapering to zero at the radius, matching
    /// `t_radius_damage`'s `120 - 0.5·dist`.
    #[test]
    fn grenade_blast_falloff() {
        assert_eq!(blast_self_damage(0.0), 120.0);
        assert_eq!(blast_self_damage(160.0), 0.0); // past the radius
        assert!((blast_self_damage(140.0) - 50.0).abs() < 0.01);
        assert_eq!(blast_self_damage(400.0), 0.0);
    }

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

    #[test]
    fn spread_scale_converges_and_widens() {
        let dist = 500.0;
        // First glimpse (visible_for 0), still, target not crossing → the loosest convergence, 1.6×.
        let first = spread_scale(0.0, 0.0, 0.0, dist);
        assert!((first - 1.6).abs() < 1e-6);
        // After sustained sight the convergence bottoms out at 0.7× (tighter than a fresh glimpse).
        let settled = spread_scale(2.0, 0.0, 0.0, dist);
        assert!((settled - 0.7).abs() < 1e-6);
        assert!(settled < first, "sustained sight must tighten aim");
        // Own motion and target crossing only ever widen the spread, each within its cap.
        assert!(spread_scale(2.0, 320.0, 0.0, dist) > settled);
        assert!(spread_scale(2.0, 0.0, 800.0, dist) > settled);
        let capped = spread_scale(0.0, 9999.0, 9_999_999.0, dist);
        assert!(capped <= 1.6 * 1.4 * 1.5 + 1e-4, "factors must stay within their caps");
    }

    #[test]
    fn spread_scale_never_below_settled_floor() {
        // The multiplier is bounded below by the fully-converged, still, non-crossing case (0.7),
        // so a high-skill bot's zero base spread stays zero and a low-skill bot never over-tightens.
        for &(v, own, perp) in &[(0.0, 0.0, 0.0), (1.5, 100.0, 200.0), (5.0, 320.0, 1000.0)] {
            assert!(spread_scale(v, own, perp, 400.0) >= 0.7 - 1e-6);
        }
    }
}
