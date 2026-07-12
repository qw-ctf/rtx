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

mod aim;
pub(crate) use aim::*;

use crate::abi::EntVars;
use crate::arsenal::{self, AmmoKind};
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
/// Nail (spike) projectile speed (`launch_spike`, weapons.rs) — nailguns fire straight, no gravity.
const NAIL_SPEED: f32 = 1000.0;
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
#[derive(Clone, Copy)]
struct WeaponChoice {
    impulse: i32,
    weapon: Weapon,
    projectile_speed: f32,
    grenade_arc: bool,
}

impl WeaponChoice {
    /// The direct-fire choice for a weapon: its select impulse and (for projectile guns) the shot
    /// speed the aim lead uses; hitscan is `0.0`. One table in place of the per-arm struct literals.
    fn of(weapon: Weapon) -> WeaponChoice {
        let (impulse, projectile_speed) = match weapon {
            Weapon::SuperShotgun => (3, 0.0),
            Weapon::Shotgun => (2, 0.0),
            Weapon::Lightning => (8, 0.0),
            Weapon::RocketLauncher => (7, ROCKET_SPEED),
            Weapon::SuperNailgun => (5, NAIL_SPEED),
            Weapon::Nailgun => (4, NAIL_SPEED),
            _ => (1, 0.0), // axe — the hard fallback
        };
        WeaponChoice { impulse, weapon, projectile_speed, grenade_arc: false }
    }

    /// The grenade-launcher arc shot (a validated lob/intercept solution — `engage` owns the aim).
    fn grenade() -> WeaponChoice {
        WeaponChoice {
            impulse: grenade::GL_IMPULSE,
            weapon: Weapon::GrenadeLauncher,
            projectile_speed: grenade::GL_SPEED,
            grenade_arc: true,
        }
    }
}

/// A bot's weapon inventory and ammo pools — the pure inputs to weapon selection, so [`choose_weapon`]
/// and [`Loadout::gl_primary`] can be unit-tested without a live [`GameState`].
#[derive(Clone, Copy)]
struct Loadout {
    items: Items,
    shells: f32,
    nails: f32,
    rockets: f32,
    cells: f32,
}

impl Loadout {
    fn of(v: &EntVars) -> Loadout {
        Loadout {
            items: Items::from_f32(v.items),
            shells: v.ammo_shells,
            nails: v.ammo_nails,
            rockets: v.ammo_rockets,
            cells: v.ammo_cells,
        }
    }

    fn has(&self, bit: Items) -> bool {
        self.items.contains(bit)
    }

    /// Amount held in an ammo pool.
    fn ammo(&self, kind: AmmoKind) -> f32 {
        match kind {
            AmmoKind::Shells => self.shells,
            AmmoKind::Nails => self.nails,
            AmmoKind::Rockets => self.rockets,
            AmmoKind::Cells => self.cells,
        }
    }

    /// Owns `w` and has the ammo to fire it — the arsenal's `min_ammo` gate (the axe/grapple, which
    /// draw no ammo, are fed whenever owned).
    fn fed(&self, w: Weapon) -> bool {
        let item = w.item();
        self.has(item)
            && arsenal::weapon_spec(item).is_some_and(|s| s.ammo_kind.is_none_or(|k| self.ammo(k) >= s.min_ammo))
    }

    /// The direct (non-explosive, before-axe) guns [`choose_weapon`] can pick, best first.
    const DIRECT_GUNS: [Weapon; 5] = [
        Weapon::SuperShotgun,
        Weapon::Lightning,
        Weapon::SuperNailgun,
        Weapon::Nailgun,
        Weapon::Shotgun,
    ];

    /// A fireable super-shotgun / lightning / super-nailgun / nailgun / shotgun — any hitscan or nail
    /// gun [`choose_weapon`] can pick before the axe. Used to decide whether the GL is the bot's
    /// *only* real weapon.
    fn has_direct_gun(&self) -> bool {
        Self::DIRECT_GUNS.iter().any(|&w| self.fed(w))
    }

    /// The grenade launcher is the bot's only offensive gun: a fireable GL, no RL, and no direct gun.
    /// (RL and GL share the rocket pool, so a fireable GL + no RL owned ⇒ no fireable RL.) When true,
    /// [`engage`] solves the grenade arc itself even with shootable grenades on — the lob→shoot combo
    /// can't help (it has no hitscan detonator), so the bot would otherwise be stuck on the axe.
    fn gl_primary(&self) -> bool {
        self.has(Items::GRENADE_LAUNCHER)
            && self.rockets >= 1.0
            && !self.has(Items::ROCKET_LAUNCHER)
            && !self.has_direct_gun()
    }
}

/// Pick a weapon for `dist`, given what the bot owns and has ammo for. `gl_air`/`gl_ground` are set
/// by [`engage`] once it has a *validated* grenade-arc solution (an airborne intercept, or a
/// grounded lead-lob) — when either holds, the grenade launcher wins.
fn choose_weapon(inv: Loadout, dist: f32, gl_air: bool, gl_ground: bool) -> WeaponChoice {
    // A solved airborne grenade intercept takes precedence: it's the shot we came here to take.
    if gl_air {
        return WeaponChoice::grenade();
    }
    // Point blank: the super shotgun then shotgun (hitscan, no self-splash).
    if dist < SPLASH_RANGE {
        if inv.fed(Weapon::SuperShotgun) {
            return WeaponChoice::of(Weapon::SuperShotgun);
        }
        if inv.fed(Weapon::Shotgun) {
            return WeaponChoice::of(Weapon::Shotgun);
        }
    }
    // Mid range: the lightning gun (fast, high DPS) when fed.
    if dist < PREFERRED_RANGE + 150.0 && inv.fed(Weapon::Lightning) {
        return WeaponChoice::of(Weapon::Lightning);
    }
    // Default: the rocket launcher (projectile, lead the target).
    if inv.fed(Weapon::RocketLauncher) {
        return WeaponChoice::of(Weapon::RocketLauncher);
    }
    // No rocket launcher but a solved grenade lob: prefer the arc over the shotgun at range (the GL
    // reaches where the SSG can't). `engage` only sets this when a lob actually solves, so we never
    // pick it hopelessly.
    if gl_ground {
        return WeaponChoice::grenade();
    }
    // Ammo-starved fallbacks: the best owned+fed gun before the axe. This is also the *only* branch a
    // stock-loadout (shotgun + axe) bot reaches at range, so without these it would roam throwing the
    // axe at distant enemies; the nailguns sit here too (never preferred over RL/SSG/LG), so a bot
    // restricted to a nailgun via `rtx_weapons` still fights with it. The axe is the hard fallback.
    Loadout::DIRECT_GUNS
        .into_iter()
        .find(|&w| inv.fed(w))
        .map_or_else(|| WeaponChoice::of(Weapon::Axe), WeaponChoice::of)
}

/// How long after losing sight of the enemy the bot keeps *holding the angle* where they vanished
/// (like a player holding a corner) before its eyes fall back to the navigation view.
const HOLD_ANGLE_TIME: f32 = 2.0;

/// Choose the safest combat move near a hazard. Candidates are tried in priority order — the wanted
/// move, the wanted move with the strafe flipped, each strafe alone, then the forward/backpedal
/// component alone — and the first that doesn't step toward lava/slime/a pit wins; if every option is
/// unsafe the bot holds ground rather than walking off. `unsafe_dir` reports whether a horizontal
/// move heads into a hazard. Backpedal is dropped before the strafe, so a bot pinned at a lava edge
/// sidesteps rather than backing in. Pure over the oracle, so the priority order is unit-testable.
fn safe_combat_move(
    unsafe_dir: &impl Fn(Vec3) -> bool,
    dir: Vec3,
    perp: Vec3,
    want_forward: f32,
    strafe_sign: f32,
) -> Vec3 {
    let fwd = dir * want_forward;
    let strafe = perp * (strafe_sign * MOVE_SPEED);
    for mv in [fwd + strafe, fwd - strafe, strafe, -strafe, fwd] {
        if !unsafe_dir(mv) {
            return mv;
        }
    }
    Vec3::ZERO
}

/// A fleeing bot's move away from a threat at heading `away` (a horizontal unit vector), kept off
/// hazards: straight away when that's clear, else the safer perpendicular, else hold ground. The
/// returned bool is whether the bot may still hop to clear the blast — suppressed when every escape
/// is unsafe, since a jump surrenders the ground control needed to stop at an edge. Used by the
/// grenade-flee in [`grenade::grenade_tactics`].
fn safe_flee_move(game: &GameState, e: EntId, origin: Vec3, away: Vec3) -> (Vec3, bool) {
    let grounded = game.entities[e].v.flags.has(Flags::ONGROUND);
    let Some(bsp) = game.nav.bsp.as_ref() else {
        return (away * MOVE_SPEED, grounded);
    };
    let host = game.host();
    let is_solid = |p: Vec3| bsp.is_solid(p);
    let contents = |p: Vec3| host.pointcontents(p);
    let feet = origin - Vec3::new(0.0, 0.0, 24.0);
    let hazardous = |d: Vec3| {
        let n = Vec3::new(d.x, d.y, 0.0).normalize_or_zero();
        n != Vec3::ZERO && crate::hazard::hazard_ahead(&is_solid, &contents, feet, n).is_some()
    };
    if !hazardous(away) {
        return (away * MOVE_SPEED, grounded);
    }
    let perp = Vec3::new(-away.y, away.x, 0.0);
    for cand in [perp, -perp] {
        if !hazardous(cand) {
            return (cand * MOVE_SPEED, grounded);
        }
    }
    (Vec3::ZERO, false) // hazards on every side — hold, don't hop off the edge
}

/// Verdict on one line-of-fire trace for a splash projectile. Clear when the shot reached the enemy,
/// or flew *unobstructed* (an open shot — an intended miss beside a strafer is fine), or stopped
/// essentially at the aim point (near-target splash still lands). Blocked only when it strikes a wall
/// short of the target *and inside our own blast radius* — the corner self-splash this whole gate
/// exists to stop (the rocket blast is [`GRENADE_BLAST_RADIUS`]; attacker self-damage is merely
/// halved). `impact` is where the trace stopped; `obstructed` is whether it hit anything at all.
fn lof_verdict(origin: Vec3, aim: Vec3, impact: Vec3, hit_enemy: bool, obstructed: bool) -> bool {
    if hit_enemy || !obstructed {
        return true;
    }
    (impact - aim).length() <= LINE_OF_FIRE_SLACK && (impact - origin).length() >= GRENADE_BLAST_RADIUS
}

/// The enemy's kinematics for one engagement frame — snapshotted once so the ballistics and aim
/// helpers share a single read of the target's motion state. A grounded target is led horizontally;
/// a swimmer moves freely but does *not* free-fall (no gravity term); an airborne one is on a
/// gravity parabola solved exactly.
struct Target {
    org: Vec3, // entity origin
    eye: Vec3, // origin + VEC_VIEW_OFS
    vel: Vec3,
    dist: f32,      // eye-to-eye separation, clamped ≥ 1
    swimming: bool, // waterlevel ≥ 2 — led in full 3D, no gravity term
    airborne: bool, // neither grounded nor swimming — on a gravity parabola
}

/// A validated grenade-arc solution: the pre-solved launch view angles and the meet point. Produced
/// by [`plan_ballistics`] (airborne intercept or grounded lead-lob), consumed by the aim path.
#[derive(Clone, Copy)]
struct GrenadeSol {
    look: Vec3,
    meet: Vec3,
}

/// Projectile planning computed inside one immutable BSP borrow, handed back as plain data.
struct BallisticPlan {
    /// Where an airborne enemy would touch down (time, point), so the rocket lead clamps at the
    /// floor instead of aiming through it.
    land: Option<(f32, Vec3)>,
    /// A *validated* airborne grenade intercept: still airborne at the meet, far enough that the
    /// blast doesn't catch us, and a real bounce sim confirms the arc reaches them.
    air_gl: Option<GrenadeSol>,
    /// The RL-less grounded lead-lob, arc-cleared like the combo's `try_start`.
    gl_ground: Option<GrenadeSol>,
}

/// The resolved shot for the fire gate: which weapon, where it aims, the clean angles to there, the
/// projectile spawn height, and whether it must land a direct hull hit (vs. leaning on splash).
struct Shot {
    choice: WeaponChoice,
    aim: Vec3,
    clean: Vec3,
    muzzle_base: Vec3,
    gate_direct: bool,
}

/// Solve the geometry-aware projectile options against the real BSP hull, all inside one immutable
/// navmesh borrow (see the field docs on [`BallisticPlan`]). The grenade arcs are only bothered with
/// when they could actually be chosen — the RL is the better weapon whenever it's in hand, so a
/// grenade intercept is a *fallback* (no RL, GL stocked). `idle`/`combos_on` gate the grounded lob
/// the way [`engage`] did: the lob→shoot combo owns grounded grenade offence when shootable grenades
/// are on, so engage only ground-lobs as the fallback when the combo is off — unless the GL is our
/// only weapon (`gl_primary`), where the combo bails and engage is the sole driver.
fn plan_ballistics(
    game: &GameState,
    origin: Vec3,
    tgt: &Target,
    gravity: f32,
    inv: Loadout,
    idle: bool,
    combos_on: bool,
) -> BallisticPlan {
    // RL and GL share the rocket ammo pool.
    let has_rl = inv.has(Items::ROCKET_LAUNCHER) && inv.rockets >= 1.0;
    let has_gl = inv.has(Items::GRENADE_LAUNCHER) && inv.rockets >= 1.0;
    let gl_primary = inv.gl_primary();
    let mut plan = BallisticPlan { land: None, air_gl: None, gl_ground: None };
    let Some(bsp) = game.nav.bsp.as_ref() else {
        return plan;
    };
    let trace = |a: Vec3, b: Vec3| bsp.hull1_trace(a, b);
    if tgt.airborne {
        let land = fall_land(&trace, tgt.org, tgt.vel, gravity, grenade::GL_FUSE);
        plan.land = land;
        // Only solve the grenade arc when it could actually be chosen — a fallback for when the RL
        // is gone (`has_gl && !has_rl`).
        if has_gl && !has_rl && idle {
            if let Some((look, t, meet)) = grenade::solve_air_intercept(origin, tgt.org, tgt.vel, gravity) {
                let airborne_at_meet = land.is_none_or(|(t_land, _)| t < t_land);
                // Keep the blast off ourselves: the meet must sit a full blast radius away.
                let safe_range = (meet - origin).length() >= GRENADE_BLAST_RADIUS;
                let enemy_at =
                    |tt: f32| ballistic_pos(tgt.org, tgt.vel, gravity, land, tt) + Vec3::new(0.0, 0.0, 4.0);
                let sim = grenade::simulate_bounce(&trace, origin, grenade::launch_velocity(look), gravity, &enemy_at);
                if airborne_at_meet && safe_range && sim.hit_enemy {
                    plan.air_gl = Some(GrenadeSol { look, meet });
                }
            }
        }
    } else if !tgt.swimming && has_gl && !has_rl && idle && (!combos_on || gl_primary) {
        // Grounded lead-lob (RL gone, GL stocked, combo off *or* GL is our only weapon): two-round
        // lead so a strafer stays in the blast, then verify the arc actually clears geometry onto the
        // led point — a purely ballistic solve would happily hurl the grenade into a low ceiling and
        // bounce it back onto us.
        let feet = tgt.org - Vec3::new(0.0, 0.0, 24.0);
        let vel_xy = Vec3::new(tgt.vel.x, tgt.vel.y, 0.0);
        if let Some((look, _flight, led)) = grenade::solve_ground_lead(origin, feet, vel_xy, gravity) {
            let clear = crate::navmesh::arc_land(bsp, origin, grenade::launch_velocity(look), gravity)
                .is_some_and(|(land_pt, _, _)| (land_pt.xy() - led.xy()).length() < grenade::LOB_LAND_TOL);
            if clear {
                plan.gl_ground = Some(GrenadeSol { look, meet: led });
            }
        }
    }
    plan
}

/// The aim point, the clean firing angles to it, and whether the shot needs a direct hull hit
/// (`gate_direct`, vs. leaning on splash). Pure ballistics. A lobbed grenade fires along its
/// pre-solved launch view (the meet point is only for aim memory and the fire gate). Other
/// projectiles solve the true intercept — where the enemy *will be* when the shot arrives; airborne
/// targets ride the gravity-displaced, floor-clamped parabola from the muzzle (falling back to a
/// linear lead if the fixed point can't settle), aimed at the hull centre (+4) for the most
/// direct-hit margin; grounded/swimming targets get a linear lead from the eye (a grounded RL
/// strafer gets the shin-drop so a near miss becomes floor splash, but nailguns need a direct hit).
/// Hitscan aims straight at the eye with no lead.
fn aim_solution(
    choice: WeaponChoice,
    tgt: &Target,
    my_eye: Vec3,
    muzzle_base: Vec3,
    gravity: f32,
    land: Option<(f32, Vec3)>,
    grenade_sol: Option<GrenadeSol>,
) -> (Vec3, Vec3, bool) {
    let s = choice.projectile_speed;
    if choice.grenade_arc {
        // Exactly one of air_gl/gl_ground is set when `grenade_arc` holds, and it aligns with
        // `airborne` (air intercept ⇒ airborne). Fire straight along the solved view.
        let sol = grenade_sol.expect("grenade_arc ⇒ a grenade solution was validated");
        (sol.meet, sol.look, tgt.airborne)
    } else if s > 0.0 {
        if tgt.airborne {
            let seed =
                intercept_time(tgt.org - muzzle_base, tgt.vel, s).unwrap_or((tgt.org - muzzle_base).length() / s);
            let pos_at = |t: f32| ballistic_pos(tgt.org, tgt.vel, gravity, land, t);
            // Fallback (fixed point didn't settle — a target falling away near projectile speed):
            // the linear-seed flight time evaluated on the *clamped* `pos_at`, so a target that lands
            // mid-flight still resolves to the landing spot rather than a point below the floor.
            let (_t, meet) = ballistic_intercept(muzzle_base, &pos_at, s, seed).unwrap_or((seed, pos_at(seed)));
            let aim = meet + Vec3::new(0.0, 0.0, 4.0);
            (aim, angles_to(muzzle_base, aim), true)
        } else {
            // A swimmer is led in full 3D with no gravity term (water isn't free-fall).
            let pred_vel = if tgt.swimming {
                tgt.vel
            } else {
                Vec3::new(tgt.vel.x, tgt.vel.y, 0.0)
            };
            let t = intercept_time(tgt.eye - my_eye, pred_vel, s).unwrap_or(tgt.dist / s);
            let mut aim = tgt.eye + pred_vel * t;
            if !tgt.swimming && choice.weapon == Weapon::RocketLauncher && pred_vel.xy().length() > 150.0 {
                aim.z -= 38.0; // eye (+22 over origin) → shin (−16)
            }
            (aim, angles_to(my_eye, aim), choice.weapon != Weapon::RocketLauncher)
        }
    } else {
        (tgt.eye, angles_to(my_eye, tgt.eye), false)
    }
}

/// The skill-scaled *drifting* aim error added to the view this frame, and the "last seen" memory
/// update. The error wanders smoothly toward a periodically resampled offset (never fresh white
/// noise per frame — that reads as view jitter), so misses sweep past the target and drift back like
/// human tracking error; pitch error is kept smaller than yaw (vertical mouse control is steadier);
/// skill 7 ⇒ error ≈ 0. The base half-range shrinks with skill, then widens with three human
/// tracking factors so snap-shots are looser than a settled duel: convergence (loose on first sight,
/// tightening over ~1.5s of continuous LoS), own motion, and target crossing speed.
fn aim_error(game: &mut GameState, e: EntId, now: f32, skill: f32, aim: Vec3, my_eye: Vec3, tgt: &Target) -> Vec3 {
    let base_spread = (7.0 - skill).max(0.0);
    let visible_for = {
        let vs = game.entities[e].bot.percept.vis_since;
        if vs > 0.0 {
            now - vs
        } else {
            0.0
        }
    };
    let own_speed = game.entities[e].v.velocity.xy().length();
    let to_enemy = tgt.eye - my_eye;
    let perp_speed = {
        let los_dir = to_enemy / tgt.dist;
        (tgt.vel - los_dir * tgt.vel.dot(los_dir)).length() // target motion across the line of fire
    };
    let spread = base_spread * spread_scale(visible_for, own_speed, perp_speed, tgt.dist);
    let frametime = game.globals.frametime;
    if now >= game.entities[e].bot.aim.err_until {
        let (r1, r2, r3) = (game.random(), game.random(), game.random());
        let b = &mut game.entities[e].bot;
        b.aim.err_target = Vec3::new((r1 - 0.5) * spread, (r2 - 0.5) * 2.0 * spread, 0.0);
        b.aim.err_until = now + 0.3 + r3 * 0.3;
    }
    let b = &mut game.entities[e].bot;
    let t = (4.0 * frametime).min(1.0);
    b.aim.err = b.aim.err + (b.aim.err_target - b.aim.err) * t;
    // Remember where the enemy is while we can see them, for the hold-the-angle behavior.
    b.seen.at = aim;
    b.seen.time = now;
    b.aim.err
}

/// Feed-forward lead for the aim spring. The spring tracks a moving solution with a steady-state lag
/// of 2·rate/ω, so on a constant strafer the crosshair would trail forever. Estimate how fast the
/// solution is moving (from last frame's clean angles) and aim ahead by the expected lag —
/// skill-scaled, so skill 7 locks onto strafers while low skill keeps trailing them. A jump too fast
/// for human tracking is treated as a discontinuity (target/weapon switch or teleport), not motion,
/// so no phantom slew is fed forward.
fn feed_forward(game: &mut GameState, e: EntId, now: f32, skill: f32, clean: Vec3) -> Vec3 {
    let b = &mut game.entities[e].bot;
    let dt = now - b.aim.look_prev_time;
    let raw = if b.aim.look_prev_time > 0.0 && dt > 1e-3 && dt < 0.25 {
        Vec3::new(bot::wrap180(clean.x - b.aim.look_prev.x) / dt, bot::wrap180(clean.y - b.aim.look_prev.y) / dt, 0.0)
    } else {
        Vec3::ZERO // stale/first sample (just acquired the target) — no estimate yet
    };
    // Genuine crossing tops out near 230°/s even up close, well under this 360°/s discontinuity cut.
    let rate = if raw.x.abs() > 360.0 || raw.y.abs() > 360.0 {
        Vec3::ZERO
    } else {
        Vec3::new(raw.x.clamp(-180.0, 180.0), raw.y.clamp(-180.0, 180.0), 0.0)
    };
    b.aim.look_prev = clean;
    b.aim.look_prev_time = now;
    rate * (2.0 / aim_omega(skill)) * (skill / 7.0)
}

/// The world-space combat movement: hold a preferred range and strafe to dodge, retreating when
/// hurt. Opponent modeling can flip this to *press* — if the enemy is believed finishable (belief
/// fresh, and we're not ourselves critical) the bot closes to finish rather than holding range;
/// `press` is false when modeling is off, leaving the range logic unchanged. Grounded near a hazard
/// the move is filtered so the bot won't strafe or backpedal into lava/slime or off a ledge (the
/// probes reuse the offensive-shove oracles — clip-hull solidity plus the engine's `pointcontents`,
/// the only hull that reports liquids); airborne or map-less it's the original blind composition.
fn combat_move(game: &mut GameState, e: EntId, enemy: EntId, now: f32, origin: Vec3, to_enemy: Vec3) -> Vec3 {
    let health = game.entities[e].v.health;
    let strafe_sign = if ((now * 0.9) + e.0 as f32).sin() >= 0.0 {
        1.0
    } else {
        -1.0
    };
    let press = game.opponent_est(e, enemy, now).is_some_and(|est| {
        press_advantage(health, crate::bot::model::est_strength(&est, now), now - est.last_update)
    });
    let retreat_health = if press { LOW_HEALTH / 2.0 } else { LOW_HEALTH };
    let dist = to_enemy.length().max(1.0);
    let want_forward = if health < retreat_health || dist < PREFERRED_RANGE - 100.0 {
        -MOVE_SPEED // back off (too hurt, or inside self-splash range)
    } else if dist > PREFERRED_RANGE + 100.0 || press {
        MOVE_SPEED // close in — normally only when far, but also to finish a pressed kill
    } else {
        0.0 // hold and strafe
    };
    let dir = Vec3::new(to_enemy.x, to_enemy.y, 0.0).normalize_or_zero();
    let perp = Vec3::new(-dir.y, dir.x, 0.0);
    let grounded_self = game.entities[e].v.flags.has(Flags::ONGROUND);
    match (grounded_self, game.nav.bsp.as_ref()) {
        (true, Some(bsp)) => {
            let host = game.host();
            let is_solid = |p: Vec3| bsp.is_solid(p);
            let contents = |p: Vec3| host.pointcontents(p);
            let feet = origin - Vec3::new(0.0, 0.0, 24.0);
            let unsafe_dir = |mv: Vec3| {
                let d = Vec3::new(mv.x, mv.y, 0.0).normalize_or_zero();
                d != Vec3::ZERO && crate::hazard::hazard_ahead(&is_solid, &contents, feet, d).is_some()
            };
            safe_combat_move(&unsafe_dir, dir, perp, want_forward, strafe_sign)
        }
        _ => dir * want_forward + perp * (strafe_sign * MOVE_SPEED),
    }
}

/// Whether to pull the trigger this frame. Fire only when the crosshair is on the spot *and* the
/// line of fire is clear. The shot leaves along the *smoothed* view (`bot.aim.angles`, last frame's spring
/// output) — firing every frame would put rockets wherever the lagging view points, behind a strafer
/// no matter how good the intercept. Projectiles gate on the predicted *miss distance* at intercept
/// range (a direct-hit shot needs the hull, a splash shot leans on the blast); hitscan keeps the
/// per-weapon cone plus low-skill leniency. A rocket also traces its real muzzle→aim line *twice*
/// (the steady `clean` ray and the ray it will actually fly along the smoothed view) and needs both
/// clear — the corner self-splash fix. Fire is held while a switch to the GL is still pending, so the
/// held gun doesn't loose along the ~18°-high grenade-loft view.
fn fire_gate(game: &mut GameState, e: EntId, enemy: EntId, origin: Vec3, skill: f32, shot: &Shot) -> bool {
    let Shot { choice, aim, clean, muzzle_base, gate_direct } = *shot;
    let s = choice.projectile_speed;
    let view = game.entities[e].bot.aim.angles;
    let on_target = if s > 0.0 {
        let launch = if choice.grenade_arc { origin } else { muzzle_base };
        let range = (aim - launch).length().max(1.0);
        view == Vec3::ZERO || miss_distance(view, clean, range) <= fire_tolerance(skill, gate_direct)
    } else {
        // Per-weapon base cone (RL is a projectile, gated above): the lightning beam is tight, the
        // shotguns/axe looser — plus low-skill leniency.
        let base_cone = if choice.weapon == Weapon::Lightning { 2.5 } else { 5.0 };
        let cone = base_cone + (7.0 - skill);
        let dp = bot::wrap180(view.x - clean.x);
        let dy = bot::wrap180(view.y - clean.y);
        view == Vec3::ZERO || (dp * dp + dy * dy).sqrt() <= cone
    };
    let switching_to_gl = choice.grenade_arc && game.entities[e].v.weapon != Weapon::GrenadeLauncher;
    // Muzzle matches `w_fire_rocket` (origin + forward·8 + 16 up), taken from each ray's own forward.
    // A grenade arc keeps its own geometry check (bounce sim / arc_land) and skips this straight-line
    // trace, which its lofted path would spuriously fail.
    let mut ray_clear = |ang: Vec3| {
        let fwd = bot::angle_vectors(ang).0;
        let muzzle = crate::weapons::rocket_muzzle(origin, fwd);
        let end = muzzle + fwd * (aim - muzzle).length();
        let tr = game.traceline(muzzle, end, false, e);
        let impact = muzzle + (end - muzzle) * tr.fraction;
        lof_verdict(origin, aim, impact, tr.ent == enemy, tr.fraction < 1.0)
    };
    let lof_clear = if choice.grenade_arc {
        true
    } else if s > 0.0 {
        // The view ray needs a spring sample; on the first frame (view == ZERO) lean on `clean` alone.
        ray_clear(clean) && (view == Vec3::ZERO || ray_clear(view))
    } else {
        true // hitscan: the eye-ray LoS above already governs the shot
    };
    on_target && lof_clear && !switching_to_gl
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
        if b.seen.time > 0.0 && now - b.seen.time < HOLD_ANGLE_TIME {
            cmd.look = angles_to(my_eye, b.seen.at);
        }
        return;
    }

    let to_enemy = enemy_eye - my_eye;
    let dist = to_enemy.length().max(1.0);
    let skill = game.host().cvar(c"rtx_bot_skill").clamp(0.0, 7.0);
    let gravity = game.host().cvar(c"sv_gravity");

    // Target motion state, snapshotted once for the ballistics/aim helpers. A grounded target is led
    // horizontally; a swimmer moves freely but does *not* free-fall; an airborne one rides a parabola.
    let grounded = game.entities[enemy].v.flags.has(Flags::ONGROUND);
    let swimming = game.entities[enemy].v.waterlevel >= 2.0;
    let tgt = Target {
        org: enemy_org,
        eye: enemy_eye,
        vel: enemy_vel,
        dist,
        swimming,
        airborne: !grounded && !swimming,
    };

    // Weapon inventory relevant to the projectile choice (RL and GL share the rocket ammo pool).
    let inv = Loadout::of(&game.entities[e].v);
    // The lob→shoot combo (`grenade::grenade_combo`, run after us) owns grounded grenade offence when
    // shootable grenades are enabled; `idle`/`combos_on` tell `plan_ballistics` whether engage is the
    // driver (see its doc). The airborne intercept is engage-exclusive.
    let idle = game.entities[e].bot.grenade_phase == GrenadePhase::Idle;
    let combos_on = game.host().cvar_bool(c"rtx_shootable_grenades");

    // Geometry-aware projectile planning, all inside one immutable BSP borrow, handed back as data.
    let plan = plan_ballistics(game, origin, &tgt, gravity, inv, idle, combos_on);

    // Weapon for the range. The grenade choice is keyed solely on inventory (a validated arc, RL
    // unavailable) — not a clock or geometry threshold — so it can't flip mid-jump and re-slew the
    // aim off the shot; RL/GL share ammo, so the only transition is the pool running dry, grounding
    // both at once. Midair's RL-only loadout never reaches the grenade path.
    let choice = choose_weapon(inv, dist, plan.air_gl.is_some(), plan.gl_ground.is_some());
    // Switch weapon only when we don't already hold the desired one (setting `impulse` re-runs
    // W_ChangeWeapon each frame otherwise).
    if game.entities[e].v.weapon != choice.weapon {
        cmd.impulse = choice.impulse;
    }

    // Aim point and clean firing angles (pure ballistics). `gate_direct` marks a shot that needs a
    // direct hull hit vs. one that can lean on splash.
    let muzzle_base = origin + Vec3::new(0.0, 0.0, 16.0); // rocket/grenade spawn height (w_fire_rocket)
    let grenade_sol = plan.air_gl.or(plan.gl_ground);
    let (aim, clean, gate_direct) = aim_solution(choice, &tgt, my_eye, muzzle_base, gravity, plan.land, grenade_sol);

    // Compose the view: clean angles + feed-forward lead + drifting skill error. `aim_error` also
    // records the "last seen" spot/time for the hold-the-angle behavior.
    let err = aim_error(game, e, now, skill, aim, my_eye, &tgt);
    let ff = feed_forward(game, e, now, skill, clean);
    cmd.look = Vec3::new(clean.x + ff.x + err.x, clean.y + ff.y + err.y, 0.0);

    // Movement (world-space) then the fire decision.
    cmd.move_world = combat_move(game, e, enemy, now, origin, to_enemy);
    cmd.buttons &= !BUTTON_JUMP; // don't bunny-hop while dueling

    let shot = Shot { choice, aim, clean, muzzle_base, gate_direct };
    if fire_gate(game, e, enemy, origin, skill, &shot) {
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
            && ent.is_player()
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
    let view = game.entities[e].bot.aim.angles;
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
        // Too close (or no clear shot / no hitscan gun): run away and hop off the ground to put
        // distance between us and the blast — but not into lava or off a ledge. Sidestep along a
        // hazard, and hold (no hop) when every escape is unsafe rather than leaping into the pit.
        let gpos = game.entities[grenade].v.origin;
        let away = Vec3::new(origin.x - gpos.x, origin.y - gpos.y, 0.0).normalize_or_zero();
        let (mv, may_hop) = safe_flee_move(game, e, origin, away);
        cmd.move_world = mv;
        if may_hop {
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

    // Combat-move geometry for the hazard-guard tests: enemy along +x, so `dir = +x`, `perp = +y`.
    const DIR: Vec3 = Vec3::new(1.0, 0.0, 0.0);
    const PERP: Vec3 = Vec3::new(0.0, 1.0, 0.0);

    #[test]
    fn safe_combat_move_passes_through_when_clear() {
        // Nothing unsafe → the exact original composition (dir·forward + perp·strafe·speed).
        let never = |_: Vec3| false;
        let mv = safe_combat_move(&never, DIR, PERP, MOVE_SPEED, 1.0);
        assert_eq!(mv, DIR * MOVE_SPEED + PERP * MOVE_SPEED);
    }

    #[test]
    fn safe_combat_move_prefers_flip() {
        // Holding range (no forward), strafing +y is unsafe → flip to −y, same speed.
        let unsafe_plus_y = |mv: Vec3| mv.y > 0.0;
        let mv = safe_combat_move(&unsafe_plus_y, DIR, PERP, 0.0, 1.0);
        assert_eq!(mv, PERP * -MOVE_SPEED);
    }

    #[test]
    fn safe_combat_move_drops_backpedal_before_strafe() {
        // Retreating (−x) is unsafe but the strafe is fine → take the strafe alone, no backpedal.
        let unsafe_back = |mv: Vec3| mv.x < 0.0;
        let mv = safe_combat_move(&unsafe_back, DIR, PERP, -MOVE_SPEED, 1.0);
        assert_eq!(mv, PERP * MOVE_SPEED); // strafe only, no −x component
        assert_eq!(mv.x, 0.0);
    }

    #[test]
    fn safe_combat_move_holds_ground_when_surrounded() {
        // Every candidate steps toward a hazard → hold ground rather than walk into it.
        let all_unsafe = |mv: Vec3| mv != Vec3::ZERO;
        let mv = safe_combat_move(&all_unsafe, DIR, PERP, -MOVE_SPEED, 1.0);
        assert_eq!(mv, Vec3::ZERO);
    }

    // Line-of-fire verdict: bot at the origin, aim 400u downrange along +x unless noted.
    const AIM: Vec3 = Vec3::new(400.0, 0.0, 0.0);

    #[test]
    fn lof_verdict_hits_enemy_always_clear() {
        // A hull hit is clear even point-blank (the super-shotgun-fallback close shot).
        assert!(lof_verdict(Vec3::ZERO, AIM, Vec3::new(60.0, 0.0, 0.0), true, true));
    }

    #[test]
    fn lof_verdict_open_shot_is_clear() {
        // Flew unobstructed but 80u wide of the target — an intended low-skill miss, not a corner.
        assert!(lof_verdict(Vec3::ZERO, AIM, Vec3::new(400.0, 80.0, 0.0), false, false));
    }

    #[test]
    fn lof_verdict_wall_near_aim_is_clear() {
        // Wall 40u short of a distant aim point: the blast still lands on the target, safe for us.
        assert!(lof_verdict(Vec3::ZERO, AIM, Vec3::new(360.0, 0.0, 0.0), false, true));
    }

    #[test]
    fn lof_verdict_corner_blocks() {
        // Wall 300u short of the target — a corner the rocket would detonate on, missing the enemy.
        assert!(!lof_verdict(Vec3::ZERO, AIM, Vec3::new(100.0, 0.0, 0.0), false, true));
    }

    #[test]
    fn lof_verdict_self_splash_blocks() {
        // Impact within 48u of a *close* aim (150u) but only 120u from us — inside our own blast
        // radius. Cleared under the old aim-point-only slack; the self-splash guard now blocks it.
        let aim = Vec3::new(150.0, 0.0, 0.0);
        assert!(!lof_verdict(Vec3::ZERO, aim, Vec3::new(120.0, 0.0, 0.0), false, true));
    }

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

    /// A loadout with the given weapon bits and every ammo pool full — the ammo count is what most
    /// `choose_weapon` branches gate on, so tests that want a weapon *fireable* start here and drain
    /// what they mean to.
    fn armed(items: Items) -> Loadout {
        Loadout { items, shells: 100.0, nails: 100.0, rockets: 100.0, cells: 100.0 }
    }

    #[test]
    fn choose_weapon_falls_back_to_nailguns() {
        // A bot restricted to a nailgun (via rtx_weapons) must fire it, not the axe. Super first.
        let sng = choose_weapon(armed(Items::SUPER_NAILGUN), 400.0, false, false);
        assert_eq!(sng.weapon, Weapon::SuperNailgun);
        assert_eq!(sng.projectile_speed, NAIL_SPEED);
        let ng = choose_weapon(armed(Items::NAILGUN), 400.0, false, false);
        assert_eq!(ng.weapon, Weapon::Nailgun);
        // Out of nails → nothing else to fire → the axe.
        let dry = Loadout { nails: 0.0, ..armed(Items::SUPER_NAILGUN | Items::NAILGUN) };
        assert_eq!(choose_weapon(dry, 400.0, false, false).weapon, Weapon::Axe);
    }

    #[test]
    fn choose_weapon_gl_needs_a_solution() {
        // A GL-only bot only fires the GL when engage supplies an arc (gl_ground/gl_air); with no
        // solution there's nothing else to fire, so it holds the axe (never lobs blindly).
        let gl = armed(Items::GRENADE_LAUNCHER);
        assert_eq!(choose_weapon(gl, 400.0, false, false).weapon, Weapon::Axe);
        assert_eq!(choose_weapon(gl, 400.0, false, true).weapon, Weapon::GrenadeLauncher);
        assert_eq!(choose_weapon(gl, 400.0, true, false).weapon, Weapon::GrenadeLauncher);
    }

    #[test]
    fn choose_weapon_full_arsenal_unchanged() {
        // Regression guard: with everything, the range order is still SSG (point-blank <140) / LG
        // (mid <550) / RL (beyond) — the nailgun fallbacks never pre-empt it.
        let all = armed(Items::all());
        assert_eq!(choose_weapon(all, 100.0, false, false).weapon, Weapon::SuperShotgun);
        assert_eq!(choose_weapon(all, 400.0, false, false).weapon, Weapon::Lightning);
        assert_eq!(choose_weapon(all, 600.0, false, false).weapon, Weapon::RocketLauncher);
    }

    #[test]
    fn gl_primary_only_when_gl_is_the_only_gun() {
        // GL alone (with rockets) → primary; out of rockets → not (can't fire it).
        assert!(armed(Items::GRENADE_LAUNCHER).gl_primary());
        assert!(!Loadout { rockets: 0.0, ..armed(Items::GRENADE_LAUNCHER) }.gl_primary());
        // Any other fireable gun, or the RL, disqualifies it (the combo / the RL take over).
        assert!(!armed(Items::GRENADE_LAUNCHER | Items::SHOTGUN).gl_primary());
        assert!(!armed(Items::GRENADE_LAUNCHER | Items::ROCKET_LAUNCHER).gl_primary());
        // A GL + an *empty* shotgun is still GL-primary (the shotgun can't fire).
        assert!(Loadout { shells: 0.0, ..armed(Items::GRENADE_LAUNCHER | Items::SHOTGUN) }.gl_primary());
    }
}
