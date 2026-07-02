// SPDX-License-Identifier: AGPL-3.0-or-later

//! Free-for-all deathmatch — the baseline mode. Every gameplay hook is the trait default (selecting
//! `ffa` changes nothing about how the game plays), so it exists to make FFA *a* mode rather than
//! *the* mode. The one behavior it does supply is the **bot brain**: in FFA everyone is an enemy, so
//! bots hunt and frag the nearest player (breaking off to grab health when hurt), which is what turns
//! them from item-collecting wanderers into deathmatch opponents. (The global `rtx_bot_pacifist`
//! override, which makes bots tail the nearest human in *any* mode, lives in `bot::run_bot`.)

use super::{team, BotIntent, GameMode};
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
fn nearest_player(g: &GameState, bot: EntId) -> Option<EntId> {
    let origin = g.entities[bot].v.origin;
    team::players(g)
        .into_iter()
        .filter(|&e| {
            let ent = &g.entities[e];
            e != bot && ent.v.health > 0.0 && ent.v.deadflag == 0.0
        })
        .min_by(|&a, &b| {
            let d = |e: EntId| (g.entities[e].v.origin - origin).length_squared();
            d(a).total_cmp(&d(b))
        })
}
