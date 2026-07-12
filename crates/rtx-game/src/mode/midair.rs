// SPDX-License-Identifier: AGPL-3.0-or-later

//! Midair (`rtx_mode midair`) — airborne-only rocket deathmatch, modeled on KTX's midair
//! (`ktx/src/combat.c`, `client.c`, `weapons.c`).
//!
//! Everyone spawns with a rocket launcher (+ axe), a big rocket count, red armor and overheal.
//! A direct rocket on an **airborne** victim is an instant kill; on a **grounded** victim it does
//! no health damage but delivers a hard knockback that launches them skyward — so you rocket
//! someone up, then airshot them out of the air. Non-rocket damage is harmless, and self-rockets
//! do no damage but still fling you (free rocket-jumps). Kills score by how high the victim was —
//! the vertical distance from where the shooter fired — awarding bronze/silver/gold/platinum
//! (+1/+2/+4/+8), with a ranked airshot line replacing the normal obituary.
//!
//! Gravity stays normal (800): the "float" comes from the launch knockback, and keeping gravity
//! standard leaves the per-map navmesh (and bot navigation) valid. Bots play the mode through the
//! shared combat overlay, which already leads airborne targets.

use glam::Vec3;

use super::{nearest_player_where, BotIntent, DamageOutcome, GameMode};
use crate::defs::{Bits, Flags, Items, PrintLevel, Weapon};
use crate::entity::EntId;
use crate::game::GameState;

/// The Midair mode descriptor (stateless — everything is computed live at the hit).
pub(crate) struct Midair;

impl GameMode for Midair {
    fn name(&self) -> &'static str {
        "midair"
    }

    fn apply_loadout(&self, g: &mut GameState, e: EntId) {
        // Rocket launcher + axe only, a big rocket stock, red armor and 250 overheal — KTX's
        // midair kit (client.c:2190). Assigning `items` (not `.with`) drops the grapple bit that
        // `put_client_in_server` hands out first, so there's no hook in the arena.
        let v = &mut g.entities[e].v;
        v.items = (Items::AXE | Items::ROCKET_LAUNCHER).as_f32();
        v.health = 250.0;
        v.max_health = 250.0;
        v.armorvalue = 200.0;
        v.armortype = 0.8; // red armor
        v.ammo_shells = 0.0;
        v.ammo_nails = 0.0;
        v.ammo_rockets = 255.0;
        v.ammo_cells = 0.0;
        v.weapon = Weapon::RocketLauncher;
    }

    fn player_damage(
        &self,
        g: &mut GameState,
        targ: EntId,
        attacker: EntId,
        inflictor: EntId,
        incoming: f32,
    ) -> DamageOutcome {
        // Only players obey midair rules — doors/buttons/grenades take normal damage (bots shoot
        // gate buttons to open them).
        if !g.entities[targ].is_player() {
            return DamageOutcome::pass(incoming);
        }
        // Only rockets do anything. The axe, fall damage (inflictor = world), drowning, etc. are
        // all harmless — which is what makes every player kill a rocket airshot (see announce_death).
        if g.entities[inflictor].classname() != Some("rocket") {
            return DamageOutcome::none();
        }

        let kb_air = g.host().cvar(c"rtx_midair_kb_air").max(0.0);
        let kb_ground = g.host().cvar(c"rtx_midair_kb_ground").max(0.0);

        // Self-rocket: no self-damage, but keep the knockback — free rocket-jumps. (Self hits only
        // ever arrive via splash, so `attacker == targ` catches them.)
        if attacker == targ {
            return DamageOutcome {
                health: 0.0,
                knockback: incoming * kb_air,
            };
        }

        let minheight = g.host().cvar(c"rtx_midair_minheight").max(0.0);
        if airborne(g, targ, minheight) {
            // Airborne victim: instant kill. 10000 blows straight through any armor the same frame.
            DamageOutcome {
                health: 10_000.0,
                knockback: incoming * kb_air,
            }
        } else {
            // Grounded victim: no health damage, but a hard launch to pop them into the air.
            DamageOutcome {
                health: 0.0,
                knockback: incoming * kb_ground,
            }
        }
    }

    fn announce_death(&self, g: &mut GameState, victim: EntId, attacker: EntId) -> bool {
        // The rocket that scored the kill: `damage_inflictor` is set in the same `t_damage` call
        // that reaches `killed`, and the rocket isn't freed until after `killed` returns.
        let inflictor = g.damage_inflictor;
        if !g.entities[attacker].is_player()
            || attacker == victim
            || !g.entities[inflictor].in_use
            || g.entities[inflictor].classname() != Some("rocket")
        {
            // Not a rocket airshot (world death, telefrag, suicide) — let the default obituary run.
            return false;
        }

        // Height = vertical distance from where the shooter fired (stamped on the rocket's
        // `oldorigin` in `w_fire_rocket`) to the victim.
        let delta = (g.entities[victim].v.origin.z - g.entities[inflictor].v.oldorigin.z).abs();
        let (frags, rank) = if delta > 1024.0 {
            (8.0, "platinum")
        } else if delta > 512.0 {
            (4.0, "gold")
        } else if delta > 256.0 {
            (2.0, "silver")
        } else {
            (1.0, "bronze")
        };

        g.award_frag(attacker, frags, victim);
        let att = g.netname_of(attacker);
        let vic = g.netname_of(victim);
        g.broadcast(
            PrintLevel::Medium,
            &format!("{att} airshot {vic} — {rank} ({delta:.0}u, +{frags:.0})\n"),
        );
        true
    }

    fn bot_intent(&self, g: &mut GameState, bot: EntId) -> Option<BotIntent> {
        // Hunt the nearest enemy — team-aware under a team composition (a 2on2 midair), else the
        // nearest living player in a free-for-all. The shared combat overlay leads airborne targets;
        // a bot rocketing a grounded enemy launches them, then airshots — emergent.
        if crate::mode::team::lifecycle_active(g) {
            return crate::mode::team::nearest_enemy(g, bot).map(BotIntent::Fight);
        }
        nearest_player(g, bot).map(BotIntent::Fight)
    }
}

/// Whether `e` is airborne above `minheight` — off the ground and more than `minheight` units above
/// the floor (a straight-down trace). Skips the trace entirely when the engine already flags the
/// entity on the ground.
fn airborne(g: &mut GameState, e: EntId, minheight: f32) -> bool {
    if g.entities[e].v.flags.has(Flags::ONGROUND) {
        return false;
    }
    let origin = g.entities[e].v.origin;
    let tr = g.traceline(origin, origin - Vec3::new(0.0, 0.0, 4096.0), true, e);
    let height = if tr.fraction < 1.0 {
        origin.z - tr.endpos.z
    } else {
        4096.0
    };
    height > minheight
}

/// The nearest living player (human or bot) to `bot`, excluding itself — everyone is an enemy in
/// this free-for-all. Unlike `bot.rs`'s `nearest_human`, this includes other bots.
fn nearest_player(g: &GameState, bot: EntId) -> Option<EntId> {
    let origin = g.entities[bot].v.origin;
    nearest_player_where(g, origin, bot, |_, _| true)
}
