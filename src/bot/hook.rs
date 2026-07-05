// SPDX-License-Identifier: AGPL-3.0-or-later

//! The grappling-hook leg driver — the `HookPhase` state machine that flies a `LinkKind::Hook`
//! route leg. Split out of `run_bot` because it's a self-contained sub-machine: it reads a
//! per-frame snapshot (all Copy) plus the shared navmesh graph, mutates only the bot's own state,
//! and emits its frame decisions as a [`HookDrive`] the caller applies once the graph/bot borrows
//! end. `reset_grapple` (which needs `&mut GameState`) is deferred via [`HookDrive::reset`].

use glam::{Vec3, Vec3Swizzles};

use super::{
    ARRIVE_RADIUS, GOAL_AVOID_TIME, HOOK_AIM_TIMEOUT, HOOK_ANCHOR_DRIFT, HOOK_BALLISTIC_SLACK,
    HOOK_FLIGHT_TIMEOUT, HOOK_REEL_TIMEOUT, HOOK_STANCE,
};
use crate::bot::state::{BotState, HookPhase};
use crate::defs::Weapon;
use crate::entity::EntId;
use crate::navmesh::NavGraph;

/// The hook driver's frame decisions, applied by `run_bot` after the graph/bot borrows release.
pub(crate) struct HookDrive {
    /// View override → the anchor / landing point (`None` = leave the nav look alone).
    pub look_target: Option<Vec3>,
    /// Hold ground still (reel/parabola own the velocity).
    pub stand: bool,
    /// Aim phase: walk toward the throw stance (source cell).
    pub approach: Option<Vec3>,
    /// Need to switch to the grapple.
    pub select: bool,
    /// Aim phase: eligible to throw once the smoothed view settles.
    pub fire_ready: bool,
    /// Flight/Reel: keep +attack held.
    pub hold_fire: bool,
    /// Deferred `reset_grapple` target (flushed once `&mut game` is free again).
    pub reset: Option<EntId>,
}

/// The per-frame snapshot the hook driver reads (all Copy). The physics reads (`hook_out`/`on_hook`/
/// `anchor`) come from the grapple snapshot `run_bot` takes at the top of the frame.
pub(crate) struct HookCtx {
    pub hook_active: bool,
    pub cur_leg: Option<u32>,
    pub enemy: Option<EntId>,
    pub hook_out: bool,
    pub on_hook: bool,
    pub grapple_hook: EntId,
    pub has_grapple: bool,
    pub now: f32,
    pub weapon: Weapon,
    pub origin: Vec3,
    pub on_ground: bool,
    pub anchor: Vec3,
    pub reel_half_step: f32,
    pub chasing: bool,
}

/// Fly a `LinkKind::Hook` leg: select the grapple, settle the view on the anchor, throw, reel to
/// build speed, then release into a gravity parabola that lands on the target ledge. All physics
/// reads come from the `anchor`/`hook_out`/`on_hook` snapshot; graph reads use `cur_leg`'s stored
/// HookTraversal. Per-phase timeouts are the traversal's own stuck detection.
pub(crate) fn drive_hook(graph: &NavGraph, bot: &mut BotState, c: HookCtx) -> HookDrive {
    let HookCtx {
        hook_active,
        cur_leg,
        enemy,
        hook_out,
        on_hook,
        grapple_hook,
        has_grapple,
        now,
        weapon,
        origin,
        on_ground,
        anchor,
        reel_half_step,
        chasing,
    } = c;

    let mut hook_look_target: Option<Vec3> = None; // view override → the anchor / landing
    let mut hook_stand = false; // hold ground still (reel/parabola own the velocity)
    let mut hook_approach: Option<Vec3> = None; // Aim: walk toward the throw stance (source cell)
    let mut hook_select = false; // need to switch to the grapple
    let mut hook_fire_ready = false; // Aim: eligible to throw once the smoothed view settles
    let mut hook_hold_fire = false; // Flight/Reel: keep +attack held
    let mut hook_reset: Option<EntId> = None; // deferred reset_grapple target
    let mut hook_failed = false;
    if hook_active {
        if let Some((leg, tr)) = cur_leg.and_then(|l| graph.hook_of_link(l).copied().map(|t| (l, t))) {
            let src = graph.cell_origin(graph.link_source(leg));
            let tgt = graph.cell_origin(graph.link_target(leg));
            // An enemy in an early phase → let combat win; abort a hook we haven't committed to.
            if enemy.is_some() && matches!(bot.hook_phase, HookPhase::Idle | HookPhase::Aim) {
                if hook_out {
                    hook_reset = Some(grapple_hook);
                }
                bot.hook_phase = HookPhase::Idle;
            } else {
                if bot.hook_phase == HookPhase::Idle {
                    if !has_grapple {
                        hook_failed = true; // no hook to fly this leg (a mode stripped it)
                    } else {
                        bot.hook_phase = HookPhase::Aim;
                        bot.hook_link = leg;
                        bot.hook_started = now;
                        bot.hook_release_dist = tr.release_dist;
                    }
                }
                match bot.hook_phase {
                    HookPhase::Aim => {
                        hook_look_target = Some(tr.stick);
                        if weapon != Weapon::Grapple {
                            hook_select = true;
                        }
                        if (origin.xy() - src.xy()).length() <= HOOK_STANCE {
                            hook_stand = true;
                            if weapon == Weapon::Grapple && on_ground && !hook_out {
                                hook_fire_ready = true; // the throw fires post-spring, once aimed
                            }
                        } else {
                            hook_approach = Some(src);
                        }
                        if now - bot.hook_started > HOOK_AIM_TIMEOUT {
                            hook_failed = true;
                        }
                    }
                    HookPhase::Flight => {
                        hook_look_target = Some(tr.stick);
                        hook_stand = true;
                        hook_hold_fire = true;
                        if on_hook {
                            if (anchor - tr.stick).length() > HOOK_ANCHOR_DRIFT {
                                hook_failed = true; // stuck somewhere the solve didn't predict
                            } else {
                                bot.hook_phase = HookPhase::Reel;
                                bot.hook_started = now;
                                bot.hook_prev_dist = (origin - anchor).length();
                            }
                        } else if !hook_out || now - bot.hook_started > HOOK_FLIGHT_TIMEOUT {
                            hook_failed = true; // throw missed / hit sky (server reset it)
                        }
                    }
                    HookPhase::Reel => {
                        hook_look_target = Some(tgt); // pre-aim the landing
                        hook_stand = true;
                        hook_hold_fire = true;
                        let d = (origin - anchor).length();
                        if !on_hook || !hook_out || (anchor - tr.stick).length() > HOOK_ANCHOR_DRIFT {
                            hook_failed = true; // hook lost or the anchor moved (door/plat/player)
                        } else if d - reel_half_step <= bot.hook_release_dist {
                            // Release: drop +attack so `service_grapple` lets go next PreThink.
                            hook_hold_fire = false;
                            bot.hook_phase = HookPhase::Ballistic;
                            bot.hook_started = now;
                        } else if d > bot.hook_prev_dist - 1.0 && now - bot.hook_started > HOOK_REEL_TIMEOUT {
                            hook_failed = true; // reel stalled against a lip
                        } else {
                            bot.hook_prev_dist = d.min(bot.hook_prev_dist);
                        }
                    }
                    HookPhase::Ballistic => {
                        hook_look_target = Some(tgt);
                        hook_stand = true; // zero input: the frictionless arc must match the solve
                        if on_ground && now - bot.hook_started > 0.1 {
                            if (origin.xy() - tgt.xy()).length() <= ARRIVE_RADIUS * 2.0 {
                                bot.hook_fails = 0;
                            }
                            bot.route_pos += 1; // clear the hook leg; repath from the landing
                            bot.hook_phase = HookPhase::Idle;
                            bot.repath_time = now;
                        } else if now - bot.hook_started > tr.airtime + HOOK_BALLISTIC_SLACK {
                            bot.hook_phase = HookPhase::Idle; // never landed cleanly — just repath
                            bot.repath_time = now;
                        }
                    }
                    HookPhase::Idle => {}
                }
            }
        } else {
            // hooking but the current leg isn't a solvable hook (route changed under us) — abort.
            if hook_out {
                hook_reset = Some(grapple_hook);
            }
            bot.hook_phase = HookPhase::Idle;
        }
    }
    if hook_failed {
        if hook_out {
            hook_reset = Some(grapple_hook);
        }
        bot.hook_phase = HookPhase::Idle;
        bot.hook_fails = bot.hook_fails.saturating_add(1);
        bot.repath_time = now;
        if bot.hook_fails >= 2 {
            bot.hook_fails = 0;
            bot.route.clear();
            if chasing {
                bot.avoid_item = bot.goal_item;
                bot.avoid_until = now + GOAL_AVOID_TIME;
                bot.goal_item = 0;
                bot.goal_select_time = now;
            }
        }
    }

    HookDrive {
        look_target: hook_look_target,
        stand: hook_stand,
        approach: hook_approach,
        select: hook_select,
        fire_ready: hook_fire_ready,
        hold_fire: hook_hold_fire,
        reset: hook_reset,
    }
}
