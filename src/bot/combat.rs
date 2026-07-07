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

use crate::bot::state::GrenadePhase;
use crate::bot::{self, grenade, BotCmd};
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

/// Opponent-model "press the advantage" thresholds. When the current enemy is believed to be on a
/// finishable stack (below [`FINISH_STACK`] — under one rocket even through green armor), the belief
/// is fresh (younger than [`FINISH_FRESH`]; a stale estimate has drifted and shouldn't buy risk), and
/// the bot itself isn't critical (health ≥ [`PRESS_FLOOR`]), the bot lowers its own retreat threshold
/// and closes in to finish the kill instead of holding range — the deathmatch-1 "he's low, go get
/// him" read the user asked for.
const FINISH_STACK: f32 = 35.0;
const FINISH_FRESH: f32 = 4.0;
const PRESS_FLOOR: f32 = 20.0;

/// Whether to press a finishable kill rather than retreat — see the threshold constants above. Pure.
fn press_advantage(own_health: f32, enemy_stack: f32, est_age: f32) -> bool {
    enemy_stack < FINISH_STACK && est_age < FINISH_FRESH && own_health >= PRESS_FLOOR
}

/// A weapon choice for the current range: the impulse that selects it, the [`Weapon`] it yields
/// (to avoid re-selecting what we already hold), and its projectile speed (`0` = hitscan, so no
/// target leading). `grenade_arc` marks a *lobbed* grenade solution (airborne intercept or grounded
/// lead-lob) whose view angles come pre-solved from the ballistic solver — the aim code fires along
/// them directly instead of pointing straight at the target.
struct WeaponChoice {
    impulse: i32,
    weapon: Weapon,
    projectile_speed: f32,
    grenade_arc: bool,
}

/// Pick a weapon for `dist`, given what the bot owns and has ammo for. `gl_air`/`gl_ground` are set
/// by [`engage`] once it has a *validated* grenade-arc solution (an airborne intercept, or a
/// grounded lead-lob used when the RL is absent) — when either holds, the grenade launcher wins.
fn choose_weapon(g: &GameState, e: EntId, dist: f32, gl_air: bool, gl_ground: bool) -> WeaponChoice {
    let v = &g.entities[e].v;
    let items = v.items;
    let have = |bit: Items| items.has(bit);

    // A solved airborne grenade intercept takes precedence: it's the shot we came here to take.
    if gl_air {
        return WeaponChoice {
            impulse: grenade::GL_IMPULSE,
            weapon: Weapon::GrenadeLauncher,
            projectile_speed: grenade::GL_SPEED,
            grenade_arc: true,
        };
    }

    // Point blank: the super shotgun (hitscan, no self-splash). Fall back to the axe if somehow
    // unarmed (audience never gets here).
    if dist < SPLASH_RANGE {
        if have(Items::SUPER_SHOTGUN) && v.ammo_shells >= 2.0 {
            return WeaponChoice {
                impulse: 3,
                weapon: Weapon::SuperShotgun,
                projectile_speed: 0.0,
                grenade_arc: false,
            };
        }
        if have(Items::SHOTGUN) && v.ammo_shells >= 1.0 {
            return WeaponChoice {
                impulse: 2,
                weapon: Weapon::Shotgun,
                projectile_speed: 0.0,
                grenade_arc: false,
            };
        }
    }

    // Mid range: the lightning gun (fast, high DPS) when fed.
    if dist < PREFERRED_RANGE + 150.0 && have(Items::LIGHTNING) && v.ammo_cells >= 1.0 {
        return WeaponChoice {
            impulse: 8,
            weapon: Weapon::Lightning,
            projectile_speed: 0.0,
            grenade_arc: false,
        };
    }

    // Default: the rocket launcher (projectile, lead the target).
    if have(Items::ROCKET_LAUNCHER) && v.ammo_rockets >= 1.0 {
        return WeaponChoice {
            impulse: 7,
            weapon: Weapon::RocketLauncher,
            projectile_speed: ROCKET_SPEED,
            grenade_arc: false,
        };
    }
    // No rocket launcher but a solved grenade lob at a grounded target: prefer the arc over the
    // shotgun at range (the GL reaches where the SSG can't). `engage` only sets this when a lob
    // actually solves, so we never pick it hopelessly.
    if gl_ground {
        return WeaponChoice {
            impulse: grenade::GL_IMPULSE,
            weapon: Weapon::GrenadeLauncher,
            projectile_speed: grenade::GL_SPEED,
            grenade_arc: true,
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
            grenade_arc: false,
        };
    }
    if have(Items::LIGHTNING) && v.ammo_cells >= 1.0 {
        return WeaponChoice {
            impulse: 8,
            weapon: Weapon::Lightning,
            projectile_speed: 0.0,
            grenade_arc: false,
        };
    }
    if have(Items::SHOTGUN) && v.ammo_shells >= 1.0 {
        return WeaponChoice {
            impulse: 2,
            weapon: Weapon::Shotgun,
            projectile_speed: 0.0,
            grenade_arc: false,
        };
    }
    WeaponChoice {
        impulse: 1,
        weapon: Weapon::Axe,
        projectile_speed: 0.0,
        grenade_arc: false,
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
/// the projectile). Shared with the grenade airborne solver (`grenade::solve_air_intercept`), where
/// gravity cancels between projectile and free-falling target so the meet reduces to this linear one.
pub(crate) fn intercept_time(r: Vec3, v: Vec3, s: f32) -> Option<f32> {
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

/// Where and when a free-falling target's parabola first meets the world, within `horizon`
/// seconds — the landing spot for the aim clamp below. Integrated through the `trace` oracle
/// (hull1, the player hull) exactly like [`crate::bot::grenade::simulate_bounce`] so the discrete
/// step matches the engine's `SV_Physics_Toss`; a non-floor hit (wall/ceiling) just clips the
/// velocity and the fall continues. `None` when it's still airborne at the horizon. Pure over the
/// oracle, so it's unit-testable against a synthetic floor.
pub(crate) fn fall_land(
    trace: &impl Fn(Vec3, Vec3) -> crate::bsp::HullTrace,
    p0: Vec3,
    v0: Vec3,
    gravity: f32,
    horizon: f32,
) -> Option<(f32, Vec3)> {
    let (mut p, mut v, mut t) = (p0, v0, 0.0f32);
    // Step cap: a fall wedged into a corner can slide in tiny sub-steps that barely advance `t`;
    // bound the work and treat an unresolved fall as "still airborne" (no clamp), like the horizon.
    for _ in 0..512 {
        if t >= horizon {
            break;
        }
        let dt = (16.0 / v.length().max(1.0)).min(0.02);
        v.z -= gravity * dt; // SV_AddGravity before the move
        let target = p + v * dt;
        let tr = trace(p, target);
        if tr.fraction < 1.0 {
            if tr.plane_normal.z > 0.7 {
                return Some((t + tr.fraction * dt, tr.endpos)); // rests on a floor
            }
            // Wall/ceiling: slide along it (kill the into-surface component) and keep falling.
            p = tr.endpos + tr.plane_normal * 0.25;
            v -= v.dot(tr.plane_normal) * tr.plane_normal;
            t += tr.fraction * dt;
        } else {
            p = target;
            t += dt;
        }
    }
    None
}

/// A free-falling target's origin at time `t`: the parabola `p0 + v0·t − ½g·t²·ẑ` until it lands,
/// then held at the landing point drifting at the ground horizontal speed. The clamp is the whole
/// point — without it a rocket aimed at a jumping enemy near the floor is led *through* the floor,
/// to a point the enemy will never occupy because they'll be standing on the ground when it arrives.
pub(crate) fn ballistic_pos(p0: Vec3, v0: Vec3, gravity: f32, land: Option<(f32, Vec3)>, t: f32) -> Vec3 {
    if let Some((t_land, land_pos)) = land {
        if t >= t_land {
            return land_pos + Vec3::new(v0.x, v0.y, 0.0) * (t - t_land);
        }
    }
    p0 + v0 * t - Vec3::new(0.0, 0.0, 0.5 * gravity * t * t)
}

/// Flight time (and meet point) for a straight projectile of speed `s` from `from` to intercept a
/// target whose position at time `t` is `pos_at(t)` — used for rockets, whose flat flight can't be
/// folded into gravity the way a grenade's can. Fixed-point `t ← |pos_at(t) − from| / s` seeded by
/// the gravity-free linear intercept; five rounds converge to well under a unit at rocket speeds.
/// `None` if it doesn't settle onto the projectile sphere (a target falling away faster than the
/// rocket closes), so the caller can fall back to a plain linear lead.
fn ballistic_intercept(from: Vec3, pos_at: &impl Fn(f32) -> Vec3, s: f32, seed: f32) -> Option<(f32, Vec3)> {
    let mut t = seed.max(0.0);
    for _ in 0..5 {
        t = (pos_at(t) - from).length() / s;
    }
    let meet = pos_at(t);
    let residual = (meet - from).length() - s * t;
    (t > 0.0 && residual.abs() < 1.0).then_some((t, meet))
}

/// Lateral world-space miss at `range` for the angular gap between the smoothed view and the clean
/// firing solution (`miss ≈ sin(Δ)·range`). Gating fire on *distance off the target* rather than a
/// fixed angular cone keeps a skill-7 solve honest: 4° of slack is ~28u at 400u — a whole player
/// width — while 16u is a guaranteed hull hit regardless of range. The angle is clamped to 90°
/// before the `sin`: past a quarter turn the crosshair is nowhere near the target and any real
/// tolerance is tens of units, so saturating at the full `range` keeps the gate monotone (an
/// unclamped `sin` dips back toward zero — even negative past 180° — and would wrongly pass a shot
/// aimed the opposite way, e.g. mid-flick onto an enemy that just appeared behind the bot).
fn miss_distance(view: Vec3, clean: Vec3, range: f32) -> f32 {
    let dp = bot::wrap180(view.x - clean.x).to_radians();
    let dy = bot::wrap180(view.y - clean.y).to_radians();
    (dp * dp + dy * dy).sqrt().min(std::f32::consts::FRAC_PI_2).sin() * range
}

/// Fire-gate tolerance (world units) at the intercept range. A **direct** hit must land inside the
/// ±16u hull (16u at skill 7); a **splash** shot rides the 160u blast, so it fires far looser (40u
/// at skill 7). Both widen with `(7 − skill)` so a low-skill bot still fires — loose, and misses —
/// rather than freezing when its lagging aim never quite reaches the tight cone.
fn fire_tolerance(skill: f32, direct: bool) -> f32 {
    let s = skill.clamp(0.0, 7.0);
    if direct {
        16.0 + (7.0 - s) * 18.0
    } else {
        40.0 + (7.0 - s) * 25.0
    }
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
    let skill = game.host().cvar(c"rtx_bot_skill").clamp(0.0, 7.0);
    let gravity = game.host().cvar(c"sv_gravity");

    // Target motion state: a grounded target is led horizontally; a swimmer moves freely but does
    // *not* free-fall (no gravity term); an airborne one is on a gravity parabola we solve exactly.
    let grounded = game.entities[enemy].v.flags.has(Flags::ONGROUND);
    let swimming = game.entities[enemy].v.waterlevel >= 2.0;
    let airborne = !grounded && !swimming;

    // Weapon inventory relevant to the projectile choice (RL and GL share the rocket ammo pool).
    let (has_rl, has_gl) = {
        let v = &game.entities[e].v;
        (
            v.items.has(Items::ROCKET_LAUNCHER) && v.ammo_rockets >= 1.0,
            v.items.has(Items::GRENADE_LAUNCHER) && v.ammo_rockets >= 1.0,
        )
    };
    let idle = game.entities[e].bot.grenade_phase == GrenadePhase::Idle;
    // The lob→shoot combo (`grenade::grenade_combo`, run after us) owns grounded grenade offence when
    // shootable grenades are enabled. So engage only ground-lobs as the fallback when the combo is
    // off — otherwise both would throw at the same grounded enemy and the combo could adopt our
    // grenade as its own. The airborne intercept below is engage-exclusive (the combo never does it).
    let combos_on = game.host().cvar_bool(c"rtx_shootable_grenades");

    // Ballistic planning against real geometry (needs the BSP hull), all inside one immutable borrow
    // of the navmesh BSP, then handed back as plain data:
    //  • `land` — where an airborne enemy would touch down, so the rocket lead clamps at the floor
    //    instead of aiming through it;
    //  • `air_gl` — a *validated* airborne grenade intercept (still airborne at the meet, far enough
    //    that its blast doesn't catch us, and a real bounce sim confirms the arc reaches them);
    //  • `gl_ground_sol` — the RL-less grounded lead-lob, arc-cleared like the combo's `try_start`.
    let mut land: Option<(f32, Vec3)> = None;
    let mut air_gl: Option<(Vec3, f32, Vec3)> = None;
    let mut gl_ground_sol: Option<(Vec3, f32, Vec3)> = None;
    if let Some(bsp) = game.nav.bsp.as_ref() {
        let trace = |a: Vec3, b: Vec3| bsp.hull1_trace(a, b);
        if airborne {
            land = fall_land(&trace, enemy_org, enemy_vel, gravity, grenade::GL_FUSE);
            // Only bother solving the grenade arc when it could actually be chosen — the RL is the
            // better airborne weapon whenever it's in hand, so a grenade intercept is a *fallback*.
            if has_gl && !has_rl && idle {
                if let Some((look, t, meet)) = grenade::solve_air_intercept(origin, enemy_org, enemy_vel, gravity) {
                    let airborne_at_meet = land.is_none_or(|(t_land, _)| t < t_land);
                    // Keep the blast off ourselves: the meet must sit a full blast radius away.
                    let safe_range = (meet - origin).length() >= GRENADE_BLAST_RADIUS;
                    let enemy_at =
                        |tt: f32| ballistic_pos(enemy_org, enemy_vel, gravity, land, tt) + Vec3::new(0.0, 0.0, 4.0);
                    let sim = grenade::simulate_bounce(&trace, origin, grenade::launch_velocity(look), gravity, &enemy_at);
                    if airborne_at_meet && safe_range && sim.hit_enemy {
                        air_gl = Some((look, t, meet));
                    }
                }
            }
        } else if !swimming && has_gl && !has_rl && idle && !combos_on {
            // Grounded lead-lob (RL gone, GL stocked, combo off): two-round lead so a strafer stays in the blast,
            // then verify the arc actually clears geometry onto the led point — a purely ballistic
            // solve would happily hurl the grenade into a low ceiling and bounce it back onto us.
            let feet = enemy_org - Vec3::new(0.0, 0.0, 24.0);
            let vel_xy = Vec3::new(enemy_vel.x, enemy_vel.y, 0.0);
            if let Some((look, flight, led)) = grenade::solve_ground_lead(origin, feet, vel_xy, gravity) {
                let clear = crate::navmesh::arc_land(bsp, origin, grenade::launch_velocity(look), gravity)
                    .is_some_and(|(land_pt, _, _)| (land_pt.xy() - led.xy()).length() < grenade::LOB_LAND_TOL);
                if clear {
                    gl_ground_sol = Some((look, flight, led));
                }
            }
        }
    }

    // Take the airborne grenade intercept only when the RL is unavailable. Keeping the choice keyed
    // solely on inventory (not a clock or geometry threshold) means it can't flip mid-jump and re-slew
    // the aim off the shot — and since RL/GL share the ammo pool, the only transition is running that
    // pool dry, which grounds both at once. Midair's RL-only loadout never reaches the grenade path.
    let gl_air = air_gl.is_some(); // `air_gl` is already gated on `!has_rl` above
    let gl_ground = gl_ground_sol.is_some();

    let choice = choose_weapon(game, e, dist, gl_air, gl_ground);

    // Switch weapon only when we don't already hold the desired one (setting `impulse` re-runs
    // W_ChangeWeapon each frame otherwise).
    if game.entities[e].v.weapon != choice.weapon {
        cmd.impulse = choice.impulse;
    }

    // Aim point and clean firing angles. Projectiles solve the true intercept — where the enemy
    // *will be* when the shot arrives — not where they are now. `gate_direct` marks a shot that
    // needs a direct hull hit (airborne) vs. one that can lean on splash (grounded/hitscan).
    let muzzle_base = origin + Vec3::new(0.0, 0.0, 16.0); // rocket/grenade spawn height (w_fire_rocket)
    let s = choice.projectile_speed;
    let (aim, clean, gate_direct) = if choice.grenade_arc {
        // Lobbed grenade: the solver already produced the launch view; fire straight along it. The
        // meet point is only for aim memory and the fire gate.
        let (look, meet) = if airborne {
            let (l, _t, m) = air_gl.expect("gl_air ⇒ air_gl set");
            (l, m)
        } else {
            let (l, _t, m) = gl_ground_sol.expect("gl_ground ⇒ solved");
            (l, m)
        };
        (meet, look, airborne)
    } else if s > 0.0 {
        if airborne {
            // Consistent ballistic intercept: iterate the flight time against the gravity-displaced,
            // floor-clamped target, solved from the muzzle. Falls back to a linear lead if it can't
            // settle. Aim at the hull centre (origin +4) — the most direct-hit margin for an airshot.
            let seed =
                intercept_time(enemy_org - muzzle_base, enemy_vel, s).unwrap_or((enemy_org - muzzle_base).length() / s);
            let pos_at = |t: f32| ballistic_pos(enemy_org, enemy_vel, gravity, land, t);
            // Fallback (fixed point didn't settle — a target falling away near projectile speed):
            // the linear-seed flight time evaluated on the *clamped* `pos_at`, so a target that lands
            // mid-flight still resolves to the landing spot rather than a point below the floor.
            let (_t, meet) =
                ballistic_intercept(muzzle_base, &pos_at, s, seed).unwrap_or((seed, pos_at(seed)));
            let aim = meet + Vec3::new(0.0, 0.0, 4.0);
            (aim, angles_to(muzzle_base, aim), true)
        } else {
            // Grounded or swimming: linear lead from the eye (unchanged behaviour). A grounded
            // strafer gets the shin-drop so a near miss becomes floor splash; a swimmer is led in
            // full 3D with no gravity term (water isn't free-fall).
            let pred_vel = if swimming {
                enemy_vel
            } else {
                Vec3::new(enemy_vel.x, enemy_vel.y, 0.0)
            };
            let t = intercept_time(enemy_eye - my_eye, pred_vel, s).unwrap_or(dist / s);
            let mut aim = enemy_eye + pred_vel * t;
            if !swimming && choice.weapon == Weapon::RocketLauncher && pred_vel.xy().length() > 150.0 {
                aim.z -= 38.0; // eye (+22 over origin) → shin (−16)
            }
            (aim, angles_to(my_eye, aim), false)
        }
    } else {
        // Hitscan: no lead.
        (enemy_eye, angles_to(my_eye, enemy_eye), false)
    };

    // Skill-scaled *drifting* aim error: the error wanders smoothly toward a periodically
    // resampled offset (never a fresh random per frame — white noise reads as jitter on the view).
    // Misses sweep past the target and drift back, like human tracking error. Pitch error is kept
    // smaller than yaw (vertical mouse control is steadier). Skill 7 ⇒ error ≈ 0.
    //
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

    // Feed-forward: the aim spring tracks a moving solution with a steady-state lag of
    // 2·rate/ω, so on a constant strafer the crosshair would trail forever. Estimate how fast the
    // solution is moving (from last frame's clean angles) and aim ahead by the expected lag —
    // skill-scaled, so skill 7 locks onto strafers while low skill keeps trailing them.
    let ff = {
        let b = &mut game.entities[e].bot;
        let dt = now - b.look_prev_time;
        let raw = if b.look_prev_time > 0.0 && dt > 1e-3 && dt < 0.25 {
            Vec3::new(bot::wrap180(clean.x - b.look_prev.x) / dt, bot::wrap180(clean.y - b.look_prev.y) / dt, 0.0)
        } else {
            Vec3::ZERO // stale/first sample (just acquired the target) — no estimate yet
        };
        // A jump too fast for human tracking is a discontinuity, not motion (a target/weapon-kind
        // switch — e.g. the ~18° step from rocket angles to the grenade loft — or a teleport). Don't
        // feed-forward a phantom slew; treat it as a fresh sample. Genuine crossing tops out near
        // 230°/s even up close, well under this.
        let rate = if raw.x.abs() > 360.0 || raw.y.abs() > 360.0 {
            Vec3::ZERO
        } else {
            Vec3::new(raw.x.clamp(-180.0, 180.0), raw.y.clamp(-180.0, 180.0), 0.0)
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
    // Opponent modeling: if the enemy is believed finishable (and the belief is fresh, and we're not
    // ourselves critical), press — retreat only when badly hurt and close in to finish rather than
    // hold range. `press` is false when modeling is off, so the range logic below is unchanged then.
    let press = game
        .opponent_est(e, enemy, now)
        .is_some_and(|est| {
            press_advantage(health, crate::bot::model::est_strength(&est, now), now - est.last_update)
        });
    let retreat_health = if press { LOW_HEALTH / 2.0 } else { LOW_HEALTH };
    let want_forward = if health < retreat_health || dist < PREFERRED_RANGE - 100.0 {
        -MOVE_SPEED // back off (too hurt, or inside self-splash range)
    } else if dist > PREFERRED_RANGE + 100.0 || press {
        MOVE_SPEED // close in — normally only when far, but also to finish a pressed kill
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
    // For projectiles we gate on the predicted *miss distance* at the intercept range rather than a
    // fixed angular cone, so a good solve isn't wasted by ~28u of angular slack at long range; a
    // direct-hit shot (airborne) needs the hull, a splash shot leans on the blast. Hitscan keeps the
    // simple weapon cone plus low-skill leniency (a low-skill bot fires looser and misses).
    let view = game.entities[e].bot.aim;
    let on_target = if s > 0.0 {
        let launch = if choice.grenade_arc { origin } else { muzzle_base };
        let range = (aim - launch).length().max(1.0);
        view == Vec3::ZERO || miss_distance(view, clean, range) <= fire_tolerance(skill, gate_direct)
    } else {
        // Original per-weapon base cone, minus the RL (now a projectile, gated above): the lightning
        // beam is tight, the shotguns/axe looser — plus low-skill leniency.
        let base_cone = if choice.weapon == Weapon::Lightning { 2.5 } else { 5.0 };
        let cone = base_cone + (7.0 - skill);
        let dp = bot::wrap180(view.x - clean.x);
        let dy = bot::wrap180(view.y - clean.y);
        view == Vec3::ZERO || (dp * dp + dy * dy).sqrt() <= cone
    };

    // Don't fire the *currently held* gun along a grenade-loft view while the weapon switch to the GL
    // is still pending — the rocket would leave ~18° high. `impulse` above requests the switch; hold
    // fire until we actually hold the GL (mirrors the grenade combo's `windup`).
    let switching_to_gl = choice.grenade_arc && game.entities[e].v.weapon != Weapon::GrenadeLauncher;

    // Line-of-fire clearance. For a rocket, trace the real muzzle→aim line (the eye→enemy LoS at the
    // top doesn't cover the muzzle, so a peeker around a corner could self-splash): trace along
    // `clean` (the geometric direction), not the smoothed view, so it stays steady frame-to-frame. A
    // grenade arc already carries its own geometry check — the airborne intercept via its bounce sim,
    // the grounded lob via `arc_land` — so it skips the straight-line trace (which a lofted arc,
    // rising well above the muzzle→target chord, would spuriously fail).
    let lof_clear = if choice.grenade_arc {
        true
    } else if s > 0.0 {
        let fwd = bot::angle_vectors(clean).0;
        let muzzle = origin + fwd * 8.0 + Vec3::new(0.0, 0.0, 16.0);
        let tr = game.traceline(muzzle, aim, false, e);
        tr.ent == enemy || (muzzle + (aim - muzzle) * tr.fraction - aim).length() <= LINE_OF_FIRE_SLACK
    } else {
        true // hitscan: the eye-ray LoS above already governs the shot
    };
    if on_target && lof_clear && !switching_to_gl {
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
    fn press_advantage_bounds() {
        // Fresh belief of a finishable stack, healthy bot → press.
        assert!(press_advantage(100.0, 20.0, 1.0));
        // Stale belief → don't buy risk on a drifted estimate.
        assert!(!press_advantage(100.0, 20.0, 5.0));
        // Enemy not actually low → no press.
        assert!(!press_advantage(100.0, 60.0, 1.0));
        // Bot itself critical → don't press even a finishable kill.
        assert!(!press_advantage(15.0, 20.0, 1.0));
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

    #[test]
    fn ballistic_intercept_hits_falling_target() {
        // A crossing, rising target that then falls under gravity; rocket at 1000 ups from below.
        let from = Vec3::new(0.0, 0.0, 40.0);
        let g = 800.0;
        let p0 = Vec3::new(500.0, 0.0, 300.0);
        let v0 = Vec3::new(0.0, 300.0, 100.0);
        let s = 1000.0;
        let pos_at = |t: f32| ballistic_pos(p0, v0, g, None, t);
        let seed = intercept_time(p0 - from, v0, s).unwrap();
        let (t, meet) = ballistic_intercept(from, &pos_at, s, seed).expect("intercept");
        // The meet sits exactly on the projectile sphere |meet − from| = s·t.
        let residual = ((meet - from).length() - s * t).abs();
        assert!(residual < 0.1, "off the projectile sphere by {residual}");
        assert!(t > 0.0);
    }

    #[test]
    fn ballistic_pos_lands_and_runs() {
        let g = 800.0;
        let p0 = Vec3::new(0.0, 0.0, 100.0);
        let v0 = Vec3::new(200.0, 0.0, 0.0); // horizontal, falls to the floor at t = 0.5 (x = 100)
        let land = Some((0.5, Vec3::new(100.0, 0.0, 0.0)));
        // Before landing: on the parabola, between launch height and the floor.
        let mid = ballistic_pos(p0, v0, g, land, 0.25);
        assert!(mid.z > 0.0 && mid.z < 100.0, "mid z {}", mid.z);
        // After landing: clamped to the floor height, drifting on at the ground speed.
        let after = ballistic_pos(p0, v0, g, land, 1.0);
        assert_eq!(after.z, 0.0);
        assert!((after.x - (100.0 + 200.0 * 0.5)).abs() < 1e-3, "x {}", after.x);
        // Never predicted below the floor — the whole point of the clamp.
        assert!(ballistic_pos(p0, v0, g, land, 5.0).z >= 0.0);
    }

    #[test]
    fn fire_tolerance_monotonic_and_tight_at_seven() {
        assert!((fire_tolerance(7.0, true) - 16.0).abs() < 1e-6); // ±hull at skill 7
        assert!((fire_tolerance(7.0, false) - 40.0).abs() < 1e-6);
        assert!(fire_tolerance(0.0, true) > fire_tolerance(7.0, true)); // widens as skill drops
        assert!(fire_tolerance(3.0, true) < fire_tolerance(3.0, false)); // direct tighter than splash
    }

    #[test]
    fn miss_distance_small_angle() {
        // 1° of yaw at 400u ≈ sin(1°)·400 ≈ 6.98u.
        let m = miss_distance(Vec3::new(0.0, 1.0, 0.0), Vec3::ZERO, 400.0);
        assert!((m - 6.98).abs() < 0.05, "miss {m}");
        assert_eq!(miss_distance(Vec3::ZERO, Vec3::ZERO, 400.0), 0.0); // on target
    }

    #[test]
    fn miss_distance_saturates_past_quarter_turn() {
        // The bug this guards: an unclamped sin dips back toward zero (and goes negative past 180°),
        // so a view aimed away from the solution would read as a *hit*. Any gap ≥90° must saturate to
        // the full range so the fire gate can never pass a back-to-front shot.
        for &(dp, dy) in &[(0.0, 175.0), (0.0, 180.0), (60.0, 170.0), (90.0, 90.0)] {
            let m = miss_distance(Vec3::new(dp, dy, 0.0), Vec3::ZERO, 400.0);
            assert!((m - 400.0).abs() < 1e-3, "off by {dp},{dy} should saturate to range, got {m}");
        }
    }
}
