// SPDX-License-Identifier: AGPL-3.0-or-later

//! Bot perception — the gate between what the mode *nominates* as a target and what a human-like bot
//! actually *knows* about. Without it the bot fights a target the instant the mode picks one: 360°,
//! any distance, straight through the reaction time a person needs. With it a nominated enemy only
//! becomes actionable once the bot has **seen** it (inside a view cone, with line of sight, held for
//! a reaction beat), **heard** its gunfire nearby, or **felt** its damage. Awareness then persists a
//! few seconds ([`MEMORY`]) so a bot that loses sight hunts the last-seen spot instead of tracking a
//! target through walls, then gives up — the believability win the plan calls out.
//!
//! Only **sight** yields an exact position: line of sight through the view frustum pins the enemy's
//! true origin. The non-visual channels reveal a **direction**, not a place — hearing gunfire or
//! feeling a hit tells the bot roughly *which way* an opponent is, and it investigates a hypothesised
//! point along that bearing (see [`heard_hypothesis`]) rather than beelining the true coordinate.
//!
//! One [`perceive`] call per bot per frame, from [`resolve_objective`](super::resolve_objective),
//! advances the reaction/memory clocks and returns this frame's [`Awareness`]. The "feel" channel is
//! pushed the other way: [`GameState::t_damage`](crate::game::GameState) stamps `known_enemy` on a
//! hurt bot directly, so getting shot in the back turns it toward the hit without seeing the shooter.

use glam::Vec3;

use crate::defs::VEC_VIEW_OFS;
use crate::entity::EntId;
use crate::game::GameState;
use crate::math::angle_vectors;

/// How long a bot stays aware of a target after last perceiving it, with no fresh contact — the
/// object-permanence window during which it hunts the last-seen position before giving up.
pub(crate) const MEMORY: f32 = 5.0;
/// A bot hears a target's gunfire within this range (no line of sight required).
const HEAR_RADIUS: f32 = 1000.0;
/// Coarse distance grain for a non-visual cue: the guessed range is snapped to this bucket, so a bot
/// can tell roughly how far a sound is by ear, never exactly.
const CUE_DIST_BUCKET: f32 = 256.0;
/// Lateral spread of a heard/felt guess, as a fraction of true distance — the guess lands on a short
/// arc across the bearing, not a pinpoint on the line to the target.
const CUE_LATERAL: f32 = 0.15;

/// A hypothesis of where an unseen opponent is, from a non-visual cue (sound / damage). The
/// **direction** from `listener` to `source` is exact; the **distance** is quantized to
/// [`CUE_DIST_BUCKET`] and jittered, and a lateral offset scatters the point off the exact ray — so
/// the bot investigates the right general direction without wall-hacking the true position. `r_lat`,
/// `r_dist` are two `game.random()` draws in `0.0..1.0` (drawn by the caller before it borrows
/// `&mut bot`). Degenerate (coincident) input returns `source`.
pub(crate) fn heard_hypothesis(listener: Vec3, source: Vec3, r_lat: f32, r_dist: f32) -> Vec3 {
    let to = source - listener;
    let bearing = to.normalize_or_zero();
    if bearing == Vec3::ZERO {
        return source;
    }
    let true_dist = to.length();
    // Rough range: snap to the bucket (floored at one bucket so a close cue isn't guessed at 0), plus
    // up to ±half a bucket of jitter.
    let rough = ((true_dist / CUE_DIST_BUCKET).round().max(1.0)) * CUE_DIST_BUCKET + (r_dist - 0.5) * CUE_DIST_BUCKET;
    // Horizontal perpendicular to the bearing (fall back to X if the bearing is near-vertical).
    let right = {
        let r = bearing.cross(Vec3::Z);
        if r.length_squared() < 1e-4 {
            Vec3::X
        } else {
            r.normalize()
        }
    };
    let lateral = right * (r_lat - 0.5) * 2.0 * CUE_LATERAL * true_dist;
    listener + bearing * rough + lateral
}

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

    // See: clear line of sight (same test as combat's) AND within the view cone around our view — and
    // the shadow we'd be sighting isn't a stale ghost the target left behind on the network client (a
    // player that teleported or ducked out of PVS, frozen where we last saw it; see `net_shadow_stale`).
    let tr = game.traceline(my_eye, enemy_eye, false, e);
    let los = (tr.ent == enemy || tr.fraction > 0.95) && !game.net_shadow_stale(enemy, now);
    let in_fov = fov <= 0.0 || {
        let fwd = angle_vectors(game.entities[e].bot.aim.angles).0;
        let to = (enemy_eye - my_eye).normalize_or_zero();
        fwd.dot(to) >= (0.5 * fov).to_radians().cos()
    };
    let can_see = los && in_fov;

    // Hear: the target fired recently (its attack cooldown is still running) within earshot. A cheap
    // stand-in for a full sound-event bus — no line of sight, so it reveals only a *direction*: the
    // bot investigates a hypothesised point along the bearing, never the true origin. Draw the guess
    // now, before the `&mut bot` borrow (`game.random()` needs `&mut game`).
    let heard = (enemy_org - my_origin).length() < HEAR_RADIUS && game.entities[enemy].combat.attack_finished > now;
    let heard_pt = heard.then(|| heard_hypothesis(my_origin, enemy_org, game.random(), game.random()));

    let b = &mut game.entities[e].bot;
    let same_target = b.percept.ent == enemy.0;

    // Continuous-visibility clock (read by combat for aim convergence): starts on the first
    // line-of-sight frame of *this* target and clears when sight breaks, so switching targets resets
    // it (a fresh face gets the loose first-glimpse aim, not the previous target's settled tracking).
    if los {
        if b.percept.vis_since == 0.0 || !same_target {
            b.percept.vis_since = now;
        }
    } else {
        b.percept.vis_since = 0.0;
    }

    // Reaction clock for *sight*: accumulate continuous sight of this enemy — a changed target or a
    // break in sight restarts it — and promote to "known" once it clears the reaction time.
    if can_see {
        if b.percept.ent != enemy.0 {
            b.percept.ent = enemy.0;
            b.percept.since = now;
        }
        b.percept.last_seen = enemy_org;
        if now - b.percept.since >= reaction {
            b.percept.known_enemy = enemy.0;
            b.percept.known_until = now + MEMORY;
        }
    } else {
        if b.percept.ent == enemy.0 {
            b.percept.ent = 0; // lost sight — the next sighting starts a fresh reaction beat
        }
        if let Some(pt) = heard_pt {
            b.percept.known_enemy = enemy.0;
            b.percept.known_until = now + MEMORY;
            b.percept.last_seen = pt; // direction-only hypothesis, not the true origin
        }
    }

    // `known` also covers the "feel" channel: `t_damage` stamps `known_enemy`/`known_until` when this
    // bot is hurt, so a hit registers as awareness here without any sight/sound this frame.
    let known = b.percept.known_enemy == enemy.0 && now < b.percept.known_until;
    match (los, known) {
        (true, true) => Awareness::Visible,
        (false, true) => Awareness::Known {
            last_seen: b.percept.last_seen,
        },
        _ => Awareness::Unaware,
    }
}

/// How long a networked player's shadow may go un-updated before we treat it as a frozen ghost rather
/// than a live target. A player in our PVS is sent every frame, so a few frames' silence (a dropped
/// packet) is tolerated, but a fifth of a second means it genuinely left our view.
const NET_STALE_GRACE: f32 = 0.2;

impl GameState {
    /// See [`Entity::net_seen`]: on the network client, a player whose shadow the server has stopped
    /// updating has left our PVS — behind a wall, or through a teleporter — so its shadow is frozen at
    /// the last spot we saw it, and a live line of sight to *that spot* is a ghost, not a target.
    /// Perception and combat both AND `!net_shadow_stale` into their line-of-sight test, so the bot
    /// stops firing the instant the real player is gone instead of emptying its magazine into the
    /// empty teleporter. Never true server-side: `is_client()` is false and the live edict keeps moving.
    pub(crate) fn net_shadow_stale(&self, e: EntId, now: f32) -> bool {
        self.host().is_client() && now - self.entities[e].net_seen > NET_STALE_GRACE
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

    #[test]
    fn hypothesis_keeps_the_bearing() {
        let listener = Vec3::new(0.0, 0.0, 0.0);
        let source = Vec3::new(600.0, 0.0, 0.0); // due +X, not on a bucket edge
        // Sweep the random draws: the guess must always point the same general way as the source.
        for &r_lat in &[0.0, 0.5, 1.0] {
            for &r_dist in &[0.0, 0.5, 1.0] {
                let g = heard_hypothesis(listener, source, r_lat, r_dist) - listener;
                let dir = (source - listener).normalize();
                assert!(g.normalize().dot(dir) > 0.9, "guess {g:?} must hug the source bearing");
            }
        }
    }

    #[test]
    fn hypothesis_is_never_the_exact_spot() {
        let listener = Vec3::new(10.0, 20.0, 30.0);
        let source = Vec3::new(610.0, 20.0, 30.0); // 600u away, off a bucket multiple
        let g = heard_hypothesis(listener, source, 0.5, 0.5);
        assert!((g - source).length() > 1.0, "a heard guess must not land on the true origin");
    }

    #[test]
    fn hypothesis_snaps_range_to_the_bucket() {
        let listener = Vec3::ZERO;
        // 800u away: nearest bucket multiple is 768 (3×256); jitter is ±128, so range ∈ [640, 896].
        let source = Vec3::new(800.0, 0.0, 0.0);
        for &r_dist in &[0.0, 0.5, 1.0] {
            let range = (heard_hypothesis(listener, source, 0.5, r_dist) - listener).length();
            assert!((640.0..=896.0).contains(&range), "range {range} outside the bucket ± jitter band");
        }
    }

    #[test]
    fn hypothesis_handles_coincident_input() {
        let p = Vec3::new(5.0, 5.0, 5.0);
        let g = heard_hypothesis(p, p, 0.5, 0.5);
        assert_eq!(g, p, "coincident listener/source returns the source without NaN");
        assert!(g.is_finite(), "no NaN from a zero bearing");
    }
}
