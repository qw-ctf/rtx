// SPDX-License-Identifier: AGPL-3.0-or-later

//! Free-for-all deathmatch — the baseline mode. Every gameplay hook is the trait default (selecting
//! `ffa` changes nothing about how the game plays), so it exists to make FFA *a* mode rather than
//! *the* mode. The one behavior it does supply is the **bot brain**: in FFA everyone is an enemy, so
//! bots hunt and frag the nearest player (breaking off to grab health when hurt), which is what turns
//! them from item-collecting wanderers into deathmatch opponents. (The global `rtx_bot_pacifist`
//! override, which makes bots tail the nearest human in *any* mode, lives in `bot::run_bot`.)

use super::{nearest_player_where, BotIntent, GameMode};
use crate::entity::EntId;
use crate::game::GameState;

/// The free-for-all deathmatch mode (`rtx_mode ffa`).
pub(crate) struct Ffa;

impl GameMode for Ffa {
    fn name(&self) -> &'static str {
        "ffa"
    }

    fn bot_intent(&self, g: &mut GameState, bot: EntId) -> Option<BotIntent> {
        // Everyone is an enemy: always hunt the nearest living player (human or bot). The
        // mode-agnostic combat layer (`bot_combat`) paths to them, aims, shoots, and retreats when
        // hurt; items along the way are picked up automatically. Deliberately *not* gated on
        // health/ammo — falling back to the item/follow brain would idle a bot on a human-less
        // server (which is exactly the "bots stand still with no human" bug). `None` only when this
        // bot is the last one alive, and the roam fallback in `run_bot` keeps even that moving.
        nearest_player(g, bot).map(BotIntent::Fight)
    }
}

/// The nearest living *other* player to `bot` — everyone is an enemy in FFA (humans and bots alike).
/// With opponent modeling on, the pick is weighted toward weaker (and, in a no-weapons-stay game,
/// better-armed) targets via the shared FFA-collective pool: a finishable frag beats a marginally
/// closer fresh one. Falls back to plain nearest when modeling is off.
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
                e.v.health > 0.0 && e.v.deadflag == 0.0
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
