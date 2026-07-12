// SPDX-License-Identifier: AGPL-3.0-or-later

//! Deathmatch — the baseline mode. Every gameplay hook is the trait default (selecting `dm` changes
//! nothing about how the game plays), so it exists to make deathmatch *a* mode rather than *the*
//! mode. The one behavior it supplies is the **bot brain**: bots hunt and frag the nearest enemy
//! (breaking off to grab health when hurt), which is what turns them from item-collecting wanderers
//! into deathmatch opponents. Under an open composition everyone is an enemy; under a team
//! composition (`rtx_match 2on2`/…) it defers to the team-aware picker so bots don't shoot allies.
//! (The global `rtx_bot_pacifist` override, which tails the nearest human in *any* mode, lives in
//! `bot::run_bot`.)

use super::{nearest_player_where, team, BotIntent, GameMode};
use crate::defs::DeadFlag;
use crate::entity::EntId;
use crate::game::GameState;

/// The deathmatch mode (`rtx_mode dm`).
pub(crate) struct Dm;

impl GameMode for Dm {
    fn name(&self) -> &'static str {
        "dm"
    }

    fn bot_intent(&self, g: &mut GameState, bot: EntId) -> Option<BotIntent> {
        // In a team composition, hunt the nearest *enemy* (the team-aware picker skips allies and
        // deconflicts); otherwise everyone is an enemy — hunt the nearest living player (human or
        // bot). The mode-agnostic combat layer (`bot_combat`) paths to them, aims, shoots, and
        // retreats when hurt; items along the way are picked up automatically. Deliberately *not*
        // gated on health/ammo — falling back to the item/follow brain would idle a bot on a
        // human-less server (the "bots stand still with no human" bug). `None` only when this bot is
        // the last one alive, and the roam fallback in `run_bot` keeps even that moving.
        if team::lifecycle_active(g) {
            return team::nearest_enemy(g, bot).map(BotIntent::Fight);
        }
        nearest_player(g, bot).map(BotIntent::Fight)
    }
}

/// The nearest living *other* player to `bot` — everyone is an enemy in open deathmatch (humans and
/// bots alike). With opponent modeling on, the pick is weighted toward weaker (and, in a
/// no-weapons-stay game, better-armed) targets via the shared collective pool: a finishable frag
/// beats a marginally closer fresh one. Falls back to plain nearest when modeling is off.
fn nearest_player(g: &GameState, bot: EntId) -> Option<EntId> {
    let origin = g.entities[bot].v.origin;
    if !g.host().cvar_bool(c"rtx_bot_model") {
        return nearest_player_where(g, origin, bot, |_, _| true);
    }
    let now = g.time();
    let weapons_stay = matches!(g.level.deathmatch, 2 | 3 | 5);
    crate::mode::players(g)
        .into_iter()
        .filter(|&en| {
            en != bot && {
                let e = &g.entities[en];
                e.v.health > 0.0 && e.v.deadflag == DeadFlag::No
            }
        })
        .map(|en| {
            let d = (g.entities[en].v.origin - origin).length_squared()
                * g.target_dist_bias(bot, en, now, weapons_stay);
            (en, d)
        })
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map(|(en, _)| en)
}
