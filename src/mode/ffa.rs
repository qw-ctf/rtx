// SPDX-License-Identifier: AGPL-3.0-or-later

//! Free-for-all deathmatch — the baseline mode. Every gameplay hook is the trait default (selecting
//! `ffa` changes nothing about how the game plays), so it exists to make FFA *a* mode rather than
//! *the* mode. The one behavior it does supply is the **bot brain**: in FFA everyone is an enemy, so
//! bots hunt and frag the nearest player (breaking off to grab health when hurt), which is what turns
//! them from item-collecting wanderers into deathmatch opponents. (The global `rtx_bot_pacifist`
//! override, which makes bots tail the nearest human in *any* mode, lives in `bot::run_bot`.)

use super::{team, BotIntent, GameMode};
use crate::defs::Bits;
use crate::entity::EntId;
use crate::game::GameState;

/// The free-for-all deathmatch mode (`rtx_mode ffa`).
pub(crate) struct Ffa;

/// Health below which a bot breaks off the fight to go find a health pickup.
const LOW_HEALTH: f32 = 50.0;

impl GameMode for Ffa {
    fn name(&self) -> &'static str {
        "ffa"
    }

    fn bot_intent(&self, g: &mut GameState, bot: EntId) -> Option<BotIntent> {
        // Survive: when hurt or completely out of ammo, fall through to the generic item brain so
        // it fetches health / a weapon instead of charging in weak. (`None` = default behavior.)
        let (health, unarmed) = {
            let v = &g.entities[bot].v;
            let ammo = v.ammo_shells + v.ammo_nails + v.ammo_rockets + v.ammo_cells;
            (v.health, ammo <= 0.0 && !v.items.has(crate::defs::Items::LIGHTNING))
        };
        if health < LOW_HEALTH || unarmed {
            return None;
        }
        // Frag: hunt the nearest living opponent. The mode-agnostic combat layer (`bot_combat`) then
        // paths to them, aims, and shoots; items along the way are still picked up automatically.
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
