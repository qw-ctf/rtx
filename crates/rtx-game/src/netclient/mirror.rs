// SPDX-License-Identifier: AGPL-3.0-or-later

//! The world mirror: what the server said, written into the fields the brain already reads.
//!
//! This is the hinge of the whole design. Inside a server, the engine fills each entity's `EntVars`
//! and the bot reads them. Here nothing fills them — so this does, from `svc_playerinfo`,
//! `svc_packetentities` and the stats. The discipline is one sentence long and worth keeping:
//! **write network truth into exactly the fields the brain already reads**, never teach the brain a
//! second way to ask. Everything the bots know how to do survives that; nothing survives a fork.
//!
//! # What a client can and cannot know
//!
//! Our own player is known exactly — origin, velocity, health, ammo, every item — because the
//! server tells us, every frame, in stats meant for us.
//!
//! Other players are known only as much as anyone watching them would know: where they are, which
//! way they're facing, whether they're on the ground, what they appear to be holding. **Not their
//! health, not their armour, not their ammo.** No stat carries that, and none ever will — it's the
//! protocol working as designed. A bot playing as a client therefore has to estimate what a bot
//! inside the server could simply read, which is exactly the honest position a human is in, and the
//! reason these bots can play against people without being something other than a player.
//!
//! # Coordinates
//!
//! Player slots are 0-based on the wire and 1-based as entities, forever and confusingly. The
//! conversion happens once, here, in [`slot_to_ent`].

use glam::Vec3;
use rtx_proto::protocol::stat;
use rtx_proto::svc::{PlayerInfo, SvcEvent};

use crate::defs::{Bits, DeadFlag, Flags, Items, MoveType, Solid, Weapon};
use crate::entity::{EntId, Entity};
use crate::game::GameState;

/// The wire's player slot as an entity id. Slots are 0-based; entity 0 is the world, so players
/// start at 1.
fn slot_to_ent(slot: u8) -> EntId {
    EntId(slot as u32 + 1)
}

/// Everything the mirror remembers between frames, per connection.
#[derive(Default)]
pub(crate) struct Mirror {
    /// Our own player slot, once the server has told us.
    playernum: u8,
    /// Our stats, as last sent. Kept whole because the server sends only what changed.
    stats: [i32; stat::COUNT],
    /// Whether our body has been set up as a bot the brain will drive.
    embodied: bool,
}

impl Mirror {
    /// Point the mirror at a player slot. Called when `svc_serverdata` says which one we are.
    pub(crate) fn set_playernum(&mut self, playernum: u8) {
        self.playernum = playernum;
        self.stats = [0; stat::COUNT];
        self.embodied = false;
    }

    /// Our own entity.
    pub(crate) fn own(&self) -> EntId {
        slot_to_ent(self.playernum)
    }

    /// A stat, as last sent.
    pub(crate) fn stat(&self, which: u8) -> i32 {
        self.stats.get(which as usize).copied().unwrap_or(0)
    }

    /// Fold one server message into the world.
    pub(crate) fn apply(&mut self, game: &mut GameState, ev: &SvcEvent) {
        match ev {
            SvcEvent::ServerData(sd) => self.set_playernum(sd.playernum),
            SvcEvent::UpdateStat { stat, value } => {
                if let Some(slot) = self.stats.get_mut(*stat as usize) {
                    *slot = *value;
                }
                self.write_own_stats(game);
            }
            SvcEvent::PlayerInfo(pi) => self.write_player(game, pi),
            SvcEvent::UpdateUserinfo { player, userinfo, .. } => {
                self.write_userinfo(game, *player, userinfo)
            }
            SvcEvent::UpdateFrags { player, frags } => {
                let e = slot_to_ent(*player);
                if is_player_slot(game, e) {
                    game.entities[e].v.frags = *frags as f32;
                }
            }
            // The server is placing our view — a teleport, or a respawn. Adopt it as *our own* aim,
            // or the bot's view spring would spend the next few frames hauling back to where it was
            // looking before it got moved.
            SvcEvent::SetAngle { angles, .. } => {
                let e = self.own();
                game.entities[e].v.v_angle = *angles;
                game.entities[e].v.angles = *angles;
                game.entities[e].bot.aim.angles = *angles;
            }
            _ => {}
        }
    }

    /// Make our body something the brain will drive.
    ///
    /// A bot inside a server is created by the server; here we *are* the client, so the body already
    /// exists on the wire and this only has to describe it the way the brain expects: a player, in
    /// use, flagged as bot-driven, with the client number its usercmds will be tagged with.
    fn embody(&mut self, game: &mut GameState) {
        let e = self.own();
        let client = e.0 as i32;
        // The engine seeds a client's move cap from this extended field; without it a bot can jump
        // but not walk (`entity.rs`). Read before the entity borrow.
        let maxspeed = game.host().cvar(c"sv_maxspeed");
        let ent = &mut game.entities[e];
        ent.in_use = true;
        ent.classname = Some("player".into());
        ent.bot = crate::bot::state::BotState {
            is_bot: true,
            client,
            ..Default::default()
        };
        ent.v.flags = ent.v.flags.with(Flags::CLIENT);
        ent.v.movetype = MoveType::Walk;
        ent.v.solid = Solid::SlideBox;
        ent.v.mins = crate::defs::VEC_HULL_MIN;
        ent.v.maxs = crate::defs::VEC_HULL_MAX;
        ent.maxspeed = maxspeed;
        self.embodied = true;
    }

    /// Our stats into our own entity.
    ///
    /// Health is the one to be careful with: `deadflag` is what the brain's `is_alive` reads, and
    /// what decides whether `run_bot` tries to *respawn* us (pulsing +attack, which is what a real
    /// client does) or to play. Deriving it from health rather than guessing keeps the server-only
    /// spawn path — which a client must never take — permanently out of reach.
    fn write_own_stats(&mut self, game: &mut GameState) {
        if !self.embodied {
            self.embody(game);
        }
        let e = self.own();
        let health = self.stat(stat::HEALTH);
        let items = Items::from_bits_truncate(self.stat(stat::ITEMS) as u32);

        let v = &mut game.entities[e].v;
        v.health = health as f32;
        v.deadflag = if health <= 0 { DeadFlag::Dead } else { DeadFlag::No };
        v.armorvalue = self.stat(stat::ARMOR) as f32;
        v.armortype = armor_type(items);
        v.items = items.as_f32();
        v.ammo_shells = self.stat(stat::SHELLS) as f32;
        v.ammo_nails = self.stat(stat::NAILS) as f32;
        v.ammo_rockets = self.stat(stat::ROCKETS) as f32;
        v.ammo_cells = self.stat(stat::CELLS) as f32;

        // `STAT_WEAPON` is the *viewmodel* index; the weapon itself is the IT_ bit in
        // ACTIVEWEAPON. Reading the wrong one gives a bot a weapon it doesn't have.
        let active = Items::from_bits_truncate(self.stat(stat::ACTIVEWEAPON) as u32);
        v.weapon = Weapon::from_f32(active.as_f32());
        v.currentammo = current_ammo(v.weapon, v);
    }

    /// One player's per-frame state.
    fn write_player(&mut self, game: &mut GameState, pi: &PlayerInfo) {
        let e = slot_to_ent(pi.player);
        let own = pi.player == self.playernum;
        if own && !self.embodied {
            self.embody(game);
        }
        // A slot we've heard nothing about — no userinfo yet — isn't a player we should be
        // reasoning about. It'll arrive.
        if !own && !is_player_slot(game, e) {
            return;
        }

        {
            let v = &mut game.entities[e].v;
            v.origin = pi.origin;
            v.velocity = pi.velocity;
            v.frame = pi.frame as f32;
            v.modelindex = pi.modelindex.unwrap_or(1) as f32;
            v.effects = pi.effects.unwrap_or(0) as f32;
            v.movetype = MoveType::Walk;
            v.solid = Solid::SlideBox;
            v.mins = crate::defs::VEC_HULL_MIN;
            v.maxs = crate::defs::VEC_HULL_MAX;

            // A player's own client sent these angles; the server passes them on. It's how a bot
            // knows where an opponent is *looking* — the same thing a human reads off a model.
            if let Some(cmd) = pi.command {
                v.v_angle = cmd.angles;
                // The body's yaw only; a model doesn't pitch.
                v.angles = Vec3::new(0.0, cmd.angles.y, 0.0);
            }

            // On the ground is knowable for *everyone* only because we asked for `Z_EXT_PF_ONGROUND`
            // at connect. It's what tells a bot an enemy is airborne, and airborne enemies are the
            // ones worth a rocket.
            v.flags = if pi.on_ground() {
                v.flags.with(Flags::ONGROUND)
            } else {
                v.flags.without(Flags::ONGROUND)
            };
            v.flags = v.flags.with(Flags::CLIENT);

            // Death is on the wire. Health is not — for anyone but us — so a dead body is known
            // exactly while a live one's condition is a guess. Our own health arrives via stats and
            // must not be clobbered here.
            if !own {
                v.deadflag = if pi.dead() { DeadFlag::Dead } else { DeadFlag::No };
                // Enough to be "alive" for the brain's gates. What it's actually *worth* is the
                // opponent model's business, and writing a real number here would be a lie with a
                // decimal point on it.
                v.health = if pi.dead() { 0.0 } else { 100.0 };
            }
        }

        // Water is not on the wire either, but it's not a secret: anyone can see where the water is.
        // The map says, and we have the map.
        self.write_waterlevel(game, e);
        game.link_edict(e);
    }

    /// Where a player is relative to the water, from the map rather than from the server.
    ///
    /// Quake's `waterlevel` is a count of how many of feet/waist/eyes are submerged, and the brain
    /// leans on it hard — swimming, drowning, whether the lightning gun is about to be a very bad
    /// idea. The three probe heights are `SV_CheckWater`'s.
    fn write_waterlevel(&mut self, game: &mut GameState, e: EntId) {
        let (origin, mins, maxs) = {
            let v = &game.entities[e].v;
            (v.origin, v.mins, v.maxs)
        };
        let host = *game.host();
        let probe = |z: f32| host.pointcontents(Vec3::new(origin.x, origin.y, z));

        let feet = probe(origin.z + mins.z + 1.0);
        let v = &mut game.entities[e].v;
        if !is_liquid(feet) {
            v.waterlevel = 0.0;
            v.watertype = rtx_nav::bsp::CONTENTS_EMPTY as f32;
            v.flags = v.flags.without(Flags::INWATER);
            return;
        }
        v.watertype = feet;
        v.waterlevel = 1.0;
        v.flags = v.flags.with(Flags::INWATER);

        // Waist, then eyes. `SV_CheckWater` samples the middle of the box and the view height.
        let waist = probe(origin.z + (mins.z + maxs.z) * 0.5);
        if is_liquid(waist) {
            game.entities[e].v.waterlevel = 2.0;
            let eyes = probe(origin.z + 22.0);
            if is_liquid(eyes) {
                game.entities[e].v.waterlevel = 3.0;
            }
        }
    }

    /// A player's name and team, from their userinfo.
    ///
    /// This is also what makes a slot count as a player at all: until we know who's in it, there's
    /// nobody there to fight.
    fn write_userinfo(&mut self, game: &mut GameState, slot: u8, userinfo: &str) {
        let e = slot_to_ent(slot);
        let info = rtx_proto::info::Info::parse(userinfo);

        // An empty userinfo means the slot emptied — the player left.
        if info.get("name").is_none_or(str::is_empty) {
            if slot != self.playernum {
                game.entities[e] = Entity::default();
            }
            return;
        }
        // A spectator occupies a slot but isn't in the game. Leaving them without a classname is
        // what keeps every scan over players from finding them.
        if info.get("*spectator").is_some_and(|s| !s.is_empty()) {
            game.entities[e] = Entity::default();
            return;
        }

        let ent = &mut game.entities[e];
        ent.in_use = true;
        ent.classname = Some("player".into());
        ent.netname = info.get("name").map(Box::from);
        ent.mode_p.team = team_id(info.get("team").unwrap_or(""));
    }
}

/// Whether a slot holds someone worth reasoning about.
fn is_player_slot(game: &GameState, e: EntId) -> bool {
    game.entities.get(e.0 as usize).is_some_and(|x| x.in_use && x.is_player())
}

/// Whether a `pointcontents` value is one of the liquids.
fn is_liquid(contents: f32) -> bool {
    use rtx_nav::bsp::{CONTENTS_LAVA, CONTENTS_SLIME, CONTENTS_WATER};
    let c = contents as i32;
    c == CONTENTS_WATER || c == CONTENTS_SLIME || c == CONTENTS_LAVA
}

/// The armour's damage-absorption fraction, from which armour we're wearing. There's no stat for
/// this — only `STAT_ARMOR` (how much is left) and the `IT_ARMOR*` bit (which kind).
fn armor_type(items: Items) -> f32 {
    if items.contains(Items::ARMOR3) {
        0.8
    } else if items.contains(Items::ARMOR2) {
        0.6
    } else if items.contains(Items::ARMOR1) {
        0.3
    } else {
        0.0
    }
}

/// The ammo count for the weapon in hand — QuakeC keeps it duplicated in `currentammo`, and weapon
/// logic reads it there.
fn current_ammo(weapon: Weapon, v: &crate::abi::EntVars) -> f32 {
    match weapon {
        Weapon::Shotgun | Weapon::SuperShotgun => v.ammo_shells,
        Weapon::Nailgun | Weapon::SuperNailgun => v.ammo_nails,
        Weapon::GrenadeLauncher | Weapon::RocketLauncher => v.ammo_rockets,
        Weapon::Lightning => v.ammo_cells,
        _ => 0.0,
    }
}

/// A team string as the small integer the mode layer uses.
///
/// The wire has team *names*, free-form; the game has team *numbers*. Any stable mapping will do so
/// long as it agrees across a squad, and the conventional colours are what KTX servers actually use.
fn team_id(team: &str) -> u8 {
    match team.trim().to_ascii_lowercase().as_str() {
        "" => 0,
        "red" => 1,
        "blue" => 2,
        "green" => 3,
        "yellow" => 4,
        // An unconventional name still has to be *a* team, and has to be the same one for every bot
        // that sees it. A hash of the name is stable across the squad without needing agreement.
        other => 5 + (other.bytes().fold(0u32, |a, b| a.wrapping_mul(31).wrapping_add(b as u32)) % 8) as u8,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::netclient::host::NetHost;
    use rtx_proto::svc::Usercmd;
    use std::path::PathBuf;

    fn game() -> GameState {
        let host: &'static NetHost = Box::leak(Box::new(NetHost::new(PathBuf::from("/nonexistent"))));
        GameState::new_client(host)
    }

    fn playerinfo(player: u8, flags: u32) -> Box<PlayerInfo> {
        Box::new(PlayerInfo {
            player,
            flags,
            origin: Vec3::new(10.0, 20.0, 30.0),
            frame: 0,
            msec: None,
            command: None,
            velocity: Vec3::ZERO,
            modelindex: None,
            skinnum: None,
            effects: None,
            weaponframe: None,
            alpha: None,
            pm_type: None,
            jump_held: false,
        })
    }

    /// Slots are 0-based on the wire and 1-based as entities. Off by one here and a bot drives
    /// somebody else's body.
    #[test]
    fn player_slots_are_offset_by_the_world() {
        assert_eq!(slot_to_ent(0), EntId(1));
        assert_eq!(slot_to_ent(7), EntId(8));
    }

    /// Our own body has to look like something the brain will pick up and drive — that's what
    /// `run_bots` scans for.
    #[test]
    fn embodies_our_own_player() {
        let mut g = game();
        let mut m = Mirror::default();
        m.set_playernum(2);
        m.apply(&mut g, &SvcEvent::UpdateStat { stat: stat::HEALTH, value: 100 });

        let e = m.own();
        assert_eq!(e, EntId(3));
        assert!(g.entities[e].in_use);
        assert!(g.entities[e].is_player());
        assert!(g.entities[e].bot.is_bot, "or run_bots would never look at it");
        assert_eq!(g.entities[e].bot.client, 3, "the client number its usercmds carry");
        assert!(g.entities[e].is_alive());
    }

    /// Stats are our own condition, and the fields they land in are the ones the brain reads.
    /// `STAT_WEAPON` is the viewmodel index and `STAT_ACTIVEWEAPON` the weapon — taking the wrong
    /// one hands the bot a gun it doesn't have.
    #[test]
    fn writes_our_stats_where_the_brain_reads_them() {
        let mut g = game();
        let mut m = Mirror::default();
        m.set_playernum(0);

        for (s, v) in [
            (stat::HEALTH, 87),
            (stat::ARMOR, 150),
            (stat::SHELLS, 20),
            (stat::NAILS, 100),
            (stat::ROCKETS, 15),
            (stat::CELLS, 60),
            (stat::WEAPON, 9999), // the viewmodel index — must not be read as the weapon
            (stat::ACTIVEWEAPON, Items::ROCKET_LAUNCHER.bits() as i32),
            (
                stat::ITEMS,
                (Items::ROCKET_LAUNCHER | Items::LIGHTNING | Items::ARMOR2 | Items::QUAD).bits() as i32,
            ),
        ] {
            m.apply(&mut g, &SvcEvent::UpdateStat { stat: s, value: v });
        }

        let v = &g.entities[m.own()].v;
        assert_eq!(v.health, 87.0);
        assert_eq!(v.armorvalue, 150.0);
        assert_eq!(v.armortype, 0.6, "yellow armour absorbs 60%");
        assert_eq!(v.ammo_rockets, 15.0);
        assert_eq!(v.weapon, Weapon::RocketLauncher);
        assert_eq!(v.currentammo, 15.0, "the ammo for the gun in hand");
        assert!(v.items.has(Items::QUAD));
        assert!(v.items.has(Items::LIGHTNING));
    }

    /// Death drives `is_alive`, which decides whether `run_bot` plays or pulses +attack to respawn
    /// — and, crucially, keeps it away from the server-only spawn path a client must never take.
    #[test]
    fn health_drives_deadflag_both_ways() {
        let mut g = game();
        let mut m = Mirror::default();
        m.set_playernum(0);

        m.apply(&mut g, &SvcEvent::UpdateStat { stat: stat::HEALTH, value: 100 });
        assert!(g.entities[m.own()].is_alive());
        assert_eq!(g.entities[m.own()].v.deadflag, DeadFlag::No);

        m.apply(&mut g, &SvcEvent::UpdateStat { stat: stat::HEALTH, value: 0 });
        assert!(!g.entities[m.own()].is_alive());
        assert_eq!(g.entities[m.own()].v.deadflag, DeadFlag::Dead);

        // And back, on respawn.
        m.apply(&mut g, &SvcEvent::UpdateStat { stat: stat::HEALTH, value: 100 });
        assert!(g.entities[m.own()].is_alive());
    }

    /// A player's own client sent their view angles and the server passed them on — so a bot can
    /// know where an opponent is looking, exactly as a human reads it off the model.
    #[test]
    fn writes_another_players_position_and_aim() {
        let mut g = game();
        let mut m = Mirror::default();
        m.set_playernum(0);
        m.apply(&mut g, &SvcEvent::UpdateUserinfo {
            player: 3,
            userid: 7,
            userinfo: "\\name\\victim".to_string(),
        });

        let mut pi = playerinfo(3, rtx_proto::svc::pf::ONGROUND);
        pi.velocity = Vec3::new(320.0, 0.0, 0.0);
        pi.command = Some(Usercmd { angles: Vec3::new(-10.0, 90.0, 0.0), ..Default::default() });
        m.apply(&mut g, &SvcEvent::PlayerInfo(pi));

        let e = slot_to_ent(3);
        assert_eq!(g.entities[e].netname.as_deref(), Some("victim"));
        assert_eq!(g.entities[e].v.origin, Vec3::new(10.0, 20.0, 30.0));
        assert_eq!(g.entities[e].v.velocity, Vec3::new(320.0, 0.0, 0.0));
        assert_eq!(g.entities[e].v.v_angle, Vec3::new(-10.0, 90.0, 0.0));
        assert_eq!(g.entities[e].v.angles.x, 0.0, "a body doesn't pitch");
        assert!(g.entities[e].v.flags.has(Flags::ONGROUND));
        assert!(g.entities[e].is_alive());

        // Airborne — the thing a rocket is for, and knowable only because we asked for
        // Z_EXT_PF_ONGROUND at connect.
        m.apply(&mut g, &SvcEvent::PlayerInfo(playerinfo(3, 0)));
        assert!(!g.entities[slot_to_ent(3)].v.flags.has(Flags::ONGROUND));
    }

    /// An opponent's death is on the wire; their health never is. The mirror must not invent one.
    #[test]
    fn another_players_death_is_known_but_their_health_is_not() {
        let mut g = game();
        let mut m = Mirror::default();
        m.set_playernum(0);
        m.apply(&mut g, &SvcEvent::UpdateUserinfo {
            player: 1,
            userid: 1,
            userinfo: "\\name\\enemy".to_string(),
        });

        m.apply(&mut g, &SvcEvent::PlayerInfo(playerinfo(1, rtx_proto::svc::pf::DEAD)));
        assert!(!g.entities[slot_to_ent(1)].is_alive(), "death is authoritative");

        m.apply(&mut g, &SvcEvent::PlayerInfo(playerinfo(1, 0)));
        let v = &g.entities[slot_to_ent(1)].v;
        assert!(v.health > 0.0, "alive enough for the brain's gates");
        assert_eq!(v.armorvalue, 0.0, "and no invented armour — that's the opponent model's job");
    }

    /// A spectator holds a slot but isn't in the game, and an emptied slot is gone. Either one left
    /// looking like a player is an enemy the bot would hunt and never find.
    #[test]
    fn spectators_and_leavers_are_not_players() {
        let mut g = game();
        let mut m = Mirror::default();
        m.set_playernum(0);

        m.apply(&mut g, &SvcEvent::UpdateUserinfo {
            player: 4,
            userid: 4,
            userinfo: "\\name\\watcher\\*spectator\\1".to_string(),
        });
        assert!(!g.entities[slot_to_ent(4)].is_player());

        // Joins for real…
        m.apply(&mut g, &SvcEvent::UpdateUserinfo {
            player: 4,
            userid: 4,
            userinfo: "\\name\\watcher".to_string(),
        });
        assert!(g.entities[slot_to_ent(4)].is_player());

        // …then leaves.
        m.apply(&mut g, &SvcEvent::UpdateUserinfo {
            player: 4,
            userid: 4,
            userinfo: String::new(),
        });
        assert!(!g.entities[slot_to_ent(4)].is_player());
    }

    /// Teams are names on the wire and numbers in the game. Any stable mapping does, provided every
    /// bot in a squad agrees — otherwise two of them disagree about who's on their side.
    #[test]
    fn team_names_map_stably_to_numbers() {
        assert_eq!(team_id(""), 0);
        assert_ne!(team_id("red"), team_id("blue"));
        assert_eq!(team_id("red"), team_id("RED"), "and case doesn't make a new team");
        assert_eq!(team_id(" red "), team_id("red"));

        // An unconventional name is still a team, and the same one every time.
        assert_eq!(team_id("clan∆"), team_id("clan∆"));
        assert_ne!(team_id("clan∆"), 0, "and not mistaken for no team at all");
    }

    /// `svc_setangle` moves our view — the server teleported or respawned us. The aim spring has to
    /// be moved with it, or the bot spends the next few frames hauling back to where it was looking.
    #[test]
    fn setangle_moves_the_aim_with_the_view() {
        let mut g = game();
        let mut m = Mirror::default();
        m.set_playernum(0);
        m.apply(&mut g, &SvcEvent::UpdateStat { stat: stat::HEALTH, value: 100 });

        let angles = Vec3::new(0.0, 135.0, 0.0);
        m.apply(&mut g, &SvcEvent::SetAngle { kind: Some(1), angles });
        assert_eq!(g.entities[m.own()].v.v_angle, angles);
        assert_eq!(g.entities[m.own()].bot.aim.angles, angles, "the spring goes too");
    }

    /// Armour type comes from which `IT_ARMOR*` bit is held, not from a stat — and the best one
    /// wins, since that's what the server would be applying.
    #[test]
    fn armor_type_follows_the_item_bit() {
        assert_eq!(armor_type(Items::empty()), 0.0);
        assert_eq!(armor_type(Items::ARMOR1), 0.3);
        assert_eq!(armor_type(Items::ARMOR2), 0.6);
        assert_eq!(armor_type(Items::ARMOR3), 0.8);
        assert_eq!(armor_type(Items::ARMOR1 | Items::ARMOR3), 0.8, "the best one wins");
    }
}
