// SPDX-License-Identifier: AGPL-3.0-or-later

//! Bot perception — the gate between what the mode *nominates* as a target and what a human-like bot
//! actually *knows* about. Without it the bot fights a target the instant the mode picks one: 360°,
//! any distance, straight through the reaction time a person needs. With it a nominated enemy only
//! becomes actionable once the bot has **seen** it (inside a view cone, with line of sight, held for
//! a reaction beat), **heard** its gunfire nearby, or **felt** its damage. Awareness then persists a
//! few seconds ([`MEMORY`]) so a bot that loses sight hunts the last-seen spot instead of tracking a
//! target through walls, then gives up — the believability win the plan calls out.
//!
//! One [`perceive`] call per bot per frame, from [`resolve_objective`](super::resolve_objective),
//! advances the reaction/memory clocks and returns this frame's [`Awareness`]. The "feel" channel is
//! pushed the other way: [`GameState::t_damage`](crate::game::GameState) stamps `known_enemy` on a
//! hurt bot directly, so getting shot in the back turns it around without waiting to see the shooter.

use glam::Vec3;

use crate::defs::VEC_VIEW_OFS;
use crate::entity::EntId;
use crate::game::GameState;

/// How long a bot stays aware of a target after last perceiving it, with no fresh contact — the
/// object-permanence window during which it hunts the last-seen position before giving up.
pub(crate) const MEMORY: f32 = 5.0;
/// A bot hears a target's gunfire within this range (no line of sight required).
const HEAR_RADIUS: f32 = 1000.0;

/// What a bot knows about a nominated target this frame.
pub(crate) enum Awareness {
    /// Not perceived — the bot has no business acting on it (keeps patrolling / collecting items).
    Unaware,
    /// Aware but without current line of sight: hunt `last_seen`, where it was last perceived.
    Known { last_seen: Vec3 },
    /// In sight right now, past the reaction beat — engage.
    Visible,
}

/// Effective view cone (full angle, degrees) for *seeing* a target: the `rtx_bot_fov` base widened
/// with skill. `0` (or less) means the sight cone is disabled — 360° awareness, the old behavior.
fn effective_fov(fov: f32, skill: f32) -> f32 {
    if fov <= 0.0 {
        0.0
    } else {
        (fov + 4.0 * skill).min(360.0)
    }
}

/// Seconds a target must stay seen before the bot reacts: the `rtx_bot_reaction` base shortened with
/// skill (floored so even skill 7 isn't quite instant). `0` base ⇒ instant, the old behavior.
fn reaction_time(base: f32, skill: f32) -> f32 {
    if base <= 0.0 {
        0.0
    } else {
        (base * (1.0 - skill / 8.0)).max(0.05)
    }
}

/// Decide this bot's awareness of `enemy`, advancing its reaction/memory clocks. Call once per frame
/// per bot. Mutates the bot's perception fields; reads the live world for sight/sound.
pub(crate) fn perceive(game: &mut GameState, e: EntId, enemy: EntId, now: f32) -> Awareness {
    let host = *game.host();
    let skill = host.cvar(c"rtx_bot_skill").clamp(0.0, 7.0);
    let fov = effective_fov(host.cvar(c"rtx_bot_fov"), skill);
    let reaction = reaction_time(host.cvar(c"rtx_bot_reaction"), skill);

    let my_origin = game.entities[e].v.origin;
    let my_eye = my_origin + VEC_VIEW_OFS;
    let enemy_org = game.entities[enemy].v.origin;
    let enemy_eye = enemy_org + VEC_VIEW_OFS;

    // See: clear line of sight (same test as combat's) AND within the view cone around our view.
    let tr = game.traceline(my_eye, enemy_eye, false, e);
    let los = tr.ent == enemy || tr.fraction > 0.95;
    let in_fov = fov <= 0.0 || {
        let fwd = crate::bot::angle_vectors(game.entities[e].bot.aim).0;
        let to = (enemy_eye - my_eye).normalize_or_zero();
        fwd.dot(to) >= (0.5 * fov).to_radians().cos()
    };
    let can_see = los && in_fov;

    // Hear: the target fired recently (its attack cooldown is still running) within earshot. A cheap
    // stand-in for a full sound-event bus — no line of sight, so it turns a bot toward nearby fire.
    let heard = (enemy_org - my_origin).length() < HEAR_RADIUS && game.entities[enemy].combat.attack_finished > now;

    let b = &mut game.entities[e].bot;
    let same_target = b.percept_ent == enemy.0;

    // Continuous-visibility clock (read by combat for aim convergence): starts on the first
    // line-of-sight frame of *this* target and clears when sight breaks, so switching targets resets
    // it (a fresh face gets the loose first-glimpse aim, not the previous target's settled tracking).
    if los {
        if b.vis_since == 0.0 || !same_target {
            b.vis_since = now;
        }
    } else {
        b.vis_since = 0.0;
    }

    // Reaction clock for *sight*: accumulate continuous sight of this enemy — a changed target or a
    // break in sight restarts it — and promote to "known" once it clears the reaction time.
    if can_see {
        if b.percept_ent != enemy.0 {
            b.percept_ent = enemy.0;
            b.percept_since = now;
        }
        b.percept_last_seen = enemy_org;
        if now - b.percept_since >= reaction {
            b.known_enemy = enemy.0;
            b.known_until = now + MEMORY;
        }
    } else {
        if b.percept_ent == enemy.0 {
            b.percept_ent = 0; // lost sight — the next sighting starts a fresh reaction beat
        }
        if heard {
            b.known_enemy = enemy.0;
            b.known_until = now + MEMORY;
            b.percept_last_seen = enemy_org;
        }
    }

    // `known` also covers the "feel" channel: `t_damage` stamps `known_enemy`/`known_until` when this
    // bot is hurt, so a hit registers as awareness here without any sight/sound this frame.
    let known = b.known_enemy == enemy.0 && now < b.known_until;
    match (los, known) {
        (true, true) => Awareness::Visible,
        (false, true) => Awareness::Known {
            last_seen: b.percept_last_seen,
        },
        _ => Awareness::Unaware,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fov_widens_with_skill_and_disables_at_zero() {
        assert_eq!(effective_fov(0.0, 5.0), 0.0, "0 base disables the sight cone (360°)");
        assert_eq!(effective_fov(120.0, 0.0), 120.0);
        assert_eq!(effective_fov(120.0, 7.0), 148.0, "each skill point adds 4°");
        assert_eq!(effective_fov(400.0, 7.0), 360.0, "clamped to a full sphere");
    }

    #[test]
    fn reaction_shortens_with_skill_and_floors() {
        assert_eq!(reaction_time(0.0, 3.0), 0.0, "0 base is instant");
        assert_eq!(reaction_time(0.4, 0.0), 0.4, "unskilled bot pays the full delay");
        assert!(reaction_time(0.4, 7.0) < reaction_time(0.4, 0.0), "skill shortens reaction");
        assert!(reaction_time(0.4, 7.0) >= 0.05, "but never below the floor");
        assert_eq!(reaction_time(0.01, 7.0), 0.05, "the floor also catches a tiny base");
    }
}
