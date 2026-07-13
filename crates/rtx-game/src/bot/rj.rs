// SPDX-License-Identifier: AGPL-3.0-or-later

//! Bot rocket jumps — the runtime side of the `navmesh::LinkKind::RocketJump` links.
//!
//! This phase supplies the **planning gate**: [`rocket_jump_extra`] tells the pathfinder how much to
//! surcharge rocket-jump links for a given bot, so one that can't currently fly a rocket jump (no
//! launcher, no rocket, too little health, or quad running) never plans a route through one. The
//! execution driver (a `HookPhase`-style machine) lands with the next phase.

use glam::{Vec3, Vec3Swizzles};

use super::state::{BotState, Driver, RjFire, RjOutcome, RjPhase, RjTelemetry};
use super::{ballistic_landing, Landing};
use crate::abi::EntVars;
use crate::defs::{Bits, Items, Weapon};
use crate::entity::EntId;
use crate::navmesh::{NavGraph, RJ_UNFIT_PENALTY};

/// Worst-case self-damage the planner budgets for: a point-blank floor rocket, unarmored (`120`
/// radius damage, `×0.5` self). Real solved links cost a touch less (~47–50, blast a few units out),
/// so gating on the worst case keeps a bot from planning a jump it lands too hurt to survive.
const RJ_WORST_SELF_DAMAGE: f32 = 60.0;
/// Health kept in reserve above the blast — a bot won't rocket-jump itself down to the wire, since it
/// often arrives into a fight (the conservative policy).
const RJ_HEALTH_MARGIN: f32 = 25.0;

/// Health actually lost to a `dmg`-point blast after armor absorbs its share, mirroring `t_damage`:
/// `save = ceil(armortype·dmg)` clamped to `armorvalue`, and the knockback is *not* reduced.
fn effective_self_damage(dmg: f32, armortype: f32, armorvalue: f32) -> f32 {
    let save = (armortype * dmg).ceil().min(armorvalue);
    dmg - save
}

/// `0.0` when this bot can fly a rocket-jump leg right now, else [`RJ_UNFIT_PENALTY`]. Unfit when it
/// lacks the rocket launcher or a rocket, has too little health for the worst-case self-blast (after
/// armor), or is running **quad** — `t_damage` applies quad *before* the mode split, so a self-rocket
/// under quad deals (and knocks back) 4×, which is both lethal and off-model for the solved arc.
pub(crate) fn rocket_jump_extra(v: &EntVars, quad_until: f32, now: f32) -> f32 {
    let effective = effective_self_damage(RJ_WORST_SELF_DAMAGE, v.armortype, v.armorvalue);
    let fit = v.items.has(Items::ROCKET_LAUNCHER)
        && v.ammo_rockets >= 1.0
        && quad_until <= now
        && v.health > effective + RJ_HEALTH_MARGIN;
    if fit {
        0.0
    } else {
        RJ_UNFIT_PENALTY
    }
}

/// The live `rtx_rj_*` tuning knobs (see [`crate::cvars`]), read once per frame in `sense` and
/// threaded through the driver (and `emit`'s aim-settle gate). All-Copy. Each default mirrors the
/// `RJ_*` constant it replaces, so an untouched server flies exactly as before — the knobs exist for
/// the tuning harness ([`crate::control`]) to sweep without a rebuild.
#[derive(Clone, Copy)]
pub(crate) struct RjKnobs {
    /// XY radius from the launch cell counted as "in stance" (`rtx_rj_stance`).
    pub stance: f32,
    /// Aim-settle tolerance before the jump presses, degrees (`rtx_rj_aim_tol`, used in `emit`).
    pub aim_tol: f32,
    /// Stance give-up timeout, seconds (`rtx_rj_stance_timeout`).
    pub stance_timeout: f32,
    /// Post-jump "still grounded ⇒ swallowed" timeout, seconds (`rtx_rj_liftoff_timeout`).
    pub liftoff_timeout: f32,
    /// Slack added to the solved airtime before the ballistic watchdog gives up (`rtx_rj_ballistic_slack`).
    pub ballistic_slack: f32,
    /// Added to the solved fire delay, seconds (`rtx_rj_delay_bias`; may be negative).
    pub delay_bias: f32,
    /// Added to the solved fire pitch, degrees (`rtx_rj_pitch_bias`; may be negative).
    pub pitch_bias: f32,
}

/// The rocket-jump driver's frame decisions, applied by `run_bot` after the graph/bot borrows end.
pub(crate) struct RjDrive {
    /// Stance/Rise: hold the view directly on these fire angles (not a look *point* — the shot flies
    /// straight along the view, and the timing matters more than a spring-settled point).
    pub look_target_angles: Option<Vec3>,
    /// Ballistic: look at the landing point (a natural travel look; the arc is already committed).
    pub look_target: Option<Vec3>,
    /// Hold ground still (Stance in-position / Rise).
    pub stand: bool,
    /// Stance: walk toward the launch cell.
    pub approach: Option<Vec3>,
    /// Need to switch to the rocket launcher (impulse 7, re-sent every frame).
    pub select: bool,
    /// Stance→Rise trigger: press jump once the smoothed view has settled (resolved in `emit`).
    pub jump_ready: bool,
    /// Rise: fire the rocket this frame (pure timing — the aim was pre-settled in Stance).
    pub fire: bool,
    /// Ballistic: world-space wish toward the landing, for gentle in-flight air-strafe correction.
    pub air_correct: Option<Vec3>,
}

/// The per-frame snapshot the rocket-jump driver reads (all Copy). The fitness fields (`has_rl` …
/// `quad`) let it re-check at leg start that the bot can still fly the jump the planner chose.
pub(crate) struct RjCtx {
    pub rj_active: bool,
    pub cur_leg: Option<u32>,
    pub enemy: Option<EntId>,
    pub chasing: bool,
    pub now: f32,
    pub weapon: Weapon,
    pub origin: Vec3,
    pub on_ground: bool,
    pub attack_finished: f32,
    pub weapons_hot: bool,
    pub has_rl: bool,
    pub ammo_rockets: f32,
    pub health: f32,
    pub armortype: f32,
    pub armorvalue: f32,
    pub quad: bool,
    pub knobs: RjKnobs,
}

/// Fly a `LinkKind::RocketJump` leg: walk to the launch cell with the RL out and the view settled on
/// the solved fire angles, jump, fire the rocket after the solved delay, then ride the blast arc onto
/// the target ledge. The Stance→Rise jump and the aim settle are resolved post-spring in `emit`; the
/// fire is pure timing (the aim was prepaid in Stance). Per-phase timeouts are the stuck detection.
pub(crate) fn drive_rj(graph: &NavGraph, bot: &mut BotState, c: RjCtx) -> RjDrive {
    let RjCtx {
        rj_active,
        cur_leg,
        enemy,
        chasing,
        now,
        weapon,
        origin,
        on_ground,
        attack_finished,
        weapons_hot,
        has_rl,
        ammo_rockets,
        health,
        armortype,
        armorvalue,
        quad,
        knobs: k,
    } = c;
    // The solved fire pitch, biased by the knob — the settle gate in `emit` measures the smoothed view
    // against exactly this, so bias and gate stay consistent.
    let pitch_bias = Vec3::new(k.pitch_bias, 0.0, 0.0);

    let mut look_target_angles = None;
    let mut look_target = None;
    let mut stand = false;
    let mut approach = None;
    let mut select = false;
    let mut jump_ready = false;
    let mut fire = false;
    let mut air_correct = None;
    let mut failed = false;

    if rj_active {
        if let Some((leg, tr)) = cur_leg.and_then(|l| graph.rocket_jump_of_link(l).copied().map(|t| (l, t))) {
            let src = graph.cell_origin(graph.link_source(leg));
            let tgt = graph.cell_origin(graph.link_target(leg));
            // An enemy while not yet committed (Idle/Stance) → let combat win; abort cleanly. (Never
            // fires under a puppet order, which forces `enemy = None`; the telemetry is defensive.)
            if enemy.is_some() && matches!(bot.rj.phase, RjPhase::Idle | RjPhase::Stance) {
                bot.rj.phase = RjPhase::Idle;
                bot.rj.telem.outcome = Some(RjOutcome::EnemyAbort);
            } else {
                if bot.rj.phase == RjPhase::Idle {
                    // Fitness pre-check on arrival: the bot's state can change between plan and here,
                    // so verify it can still afford the specific leg's blast before committing.
                    let effective = effective_self_damage(tr.self_damage, armortype, armorvalue);
                    let fit = has_rl
                        && ammo_rockets >= 1.0
                        && weapons_hot
                        && !quad
                        && health > effective + RJ_HEALTH_MARGIN;
                    if !fit {
                        // Record enough for the harness to see *which* leg was unfit before we bail.
                        bot.rj.telem.link = leg;
                        bot.rj.telem.src = src;
                        bot.rj.telem.tgt = tgt;
                        bot.rj.telem.outcome = Some(RjOutcome::Unfit);
                        failed = true;
                    } else {
                        bot.rj.phase = RjPhase::Stance;
                        bot.rj.link = leg;
                        bot.rj.started = now;
                        // Snapshot the plan + knob biases for this attempt (clears any prior outcome).
                        bot.rj.telem = RjTelemetry {
                            link: leg,
                            src,
                            tgt,
                            solved_angles: tr.fire_angles,
                            solved_delay: tr.fire_delay,
                            airtime: tr.airtime,
                            self_damage: tr.self_damage,
                            delay_bias: k.delay_bias,
                            pitch_bias: k.pitch_bias,
                            press: None,
                            fire: None,
                            outcome: None,
                        };
                    }
                }
                match bot.rj.phase {
                    RjPhase::Stance => {
                        look_target_angles = Some(tr.fire_angles + pitch_bias);
                        if weapon != Weapon::RocketLauncher {
                            select = true; // impulse 7, re-sent (swallowed until the current cooldown ends)
                        }
                        if (origin.xy() - src.xy()).length() <= k.stance {
                            stand = true;
                            // Ready to jump once the RL is in hand, on the ground, off cooldown (else the
                            // mid-air +attack is swallowed), and no enemy has appeared.
                            if weapon == Weapon::RocketLauncher && on_ground && now >= attack_finished && enemy.is_none() {
                                jump_ready = true; // the jump presses post-spring, once the aim settles
                            }
                        } else {
                            approach = Some(src);
                        }
                        if now - bot.rj.started > k.stance_timeout {
                            bot.rj.telem.outcome = Some(RjOutcome::StanceTimeout);
                            failed = true;
                        }
                    }
                    RjPhase::Rise => {
                        look_target_angles = Some(tr.fire_angles + pitch_bias); // keep holding the settled aim
                        stand = true;
                        if on_ground && now - bot.rj.jump_time > k.liftoff_timeout {
                            bot.rj.telem.outcome = Some(RjOutcome::LiftoffTimeout);
                            failed = true; // the jump was swallowed — never left the ground
                        } else if now - bot.rj.jump_time >= tr.fire_delay + k.delay_bias {
                            fire = true; // fire this frame (aim already held since Stance)
                            // The post-spring view sent with +attack is filled in by `emit`.
                            bot.rj.telem.fire = Some(RjFire {
                                t: now,
                                actual_delay: now - bot.rj.jump_time,
                                origin,
                                view: Vec3::ZERO,
                            });
                            bot.rj.phase = RjPhase::Ballistic;
                            bot.rj.started = now;
                        }
                    }
                    RjPhase::Ballistic => {
                        look_target = Some(tgt);
                        // Gentle correction toward the landing — QW air accel caps this to a nudge
                        // within the perturb-guaranteed neighborhood (the user's error-correction).
                        air_correct = Some(tgt);
                        let elapsed = now - bot.rj.started;
                        match ballistic_landing(origin, tgt, on_ground, elapsed, tr.airtime + k.ballistic_slack) {
                            Landing::Down { on_target } => {
                                if on_target {
                                    bot.rj.fails = 0;
                                }
                                bot.rj.telem.outcome = Some(RjOutcome::Landed { on_target, origin, t: now });
                                bot.route_pos += 1; // clear the leg; repath from the landing
                                bot.rj.phase = RjPhase::Idle;
                                bot.repath_time = now;
                            }
                            Landing::Overran => {
                                bot.rj.telem.outcome = Some(RjOutcome::Overran { origin, t: now });
                                bot.rj.phase = RjPhase::Idle; // never landed cleanly — repath
                                bot.repath_time = now;
                            }
                            Landing::Riding => {}
                        }
                    }
                    RjPhase::Idle => {}
                }
            }
        } else {
            // The pinned/current leg isn't a solvable rocket jump — abort. Under a puppet order this
            // means the graph was rebuilt out from under the attempt (link ids aren't stable).
            if bot.rj.phase != RjPhase::Idle {
                bot.rj.telem.outcome = Some(RjOutcome::LegVanished);
            }
            bot.rj.phase = RjPhase::Idle;
        }
    }
    if failed {
        bot.rj.phase = RjPhase::Idle;
        bot.traversal_failed(Driver::RocketJump, chasing, now);
    }

    RjDrive {
        look_target_angles,
        look_target,
        stand,
        approach,
        select,
        jump_ready,
        fire,
        air_correct,
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
