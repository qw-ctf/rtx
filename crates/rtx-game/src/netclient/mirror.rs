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

use crate::bot::model::PickupKind;
use crate::defs::{Bits, DeadFlag, Effects, Flags, Items, MoveType, Solid, Weapon};
use crate::entity::MoverPhase;
use crate::entity::{EntId, Entity, Touch};
use crate::game::GameState;
use crate::netclient::frames::EntityState;

/// The wire's player slot as an entity id. Slots are 0-based; entity 0 is the world, so players
/// start at 1.
fn slot_to_ent(slot: u8) -> EntId {
    EntId(slot as u32 + 1)
}

/// QuakeWorld's hard limit on player slots, and the width of everything a slot number indexes.
///
/// The wire's slot is a byte, so a server could name slot 200. Everything here is 32 wide, and the
/// difference between checking that and folding it around is a player written into a stranger's body.
pub(crate) const MAX_CLIENTS: usize = 32;

/// What a mirror needs to know that no single connection can answer.
///
/// A squad is several clients watching one game and writing into one world. Two of the rules for
/// writing it can only be stated across the whole squad, and a mirror on its own gets both wrong:
///
/// - **Whose body is that?** Every bot receives everyone's `svc_playerinfo`, our own squadmates'
///   included. A mirror can tell *its* body from the rest, but not a squadmate's from a stranger's
///   — so it writes a stranger's placeholder health over a body whose exact health another mirror
///   already knows from its own stats.
/// - **Could we have seen it?** An item's absence is only evidence if somebody had a clear look at
///   the spot. The world is the union of what the squad can see, so the looking has to be too:
///   asking one bot's eyes about a spot another bot is standing next to declares a plainly-present
///   item taken.
#[derive(Default)]
pub(crate) struct Squad {
    /// The slots our own bots occupy. Not "is it mine" — "is it ours".
    slots: [bool; MAX_CLIENTS],
    /// Each of our live bots: its body, and where that body looks from. Paired, because a trace has
    /// to ignore the body it starts inside — an eye sits within its own player box, so a trace that
    /// doesn't skip it hits it immediately and reports the whole map blocked.
    eyes: Vec<(EntId, Vec3)>,
}

impl Squad {
    /// Build from what each connection knows about itself.
    pub(crate) fn new(slots: [bool; MAX_CLIENTS], eyes: Vec<(EntId, Vec3)>) -> Self {
        Squad { slots, eyes }
    }

    /// Whether a slot holds one of our own bots — whose truth its own mirror owns, and no other
    /// mirror may touch.
    fn ours(&self, slot: u8) -> bool {
        self.slots.get(slot as usize).copied().unwrap_or(false)
    }
}

/// Everything the mirror remembers between frames, per connection.
pub(crate) struct Mirror {
    /// Our own player slot, once the server has told us.
    playernum: u8,
    /// Whether we joined to watch rather than to play.
    ///
    /// A spectator holds a slot and a scoreboard entry but has no body — so there is nothing here to
    /// drive, and nothing that should try. It matters more than it sounds: mvdsv sends a spectator
    /// the *tracked player's* stats, so a mirror that embodied anyway would take a stranger's health
    /// and ammo for its own and play off them, and the respawn pulse it sent for the 0-health slot it
    /// started on would cycle the camera through the server's players instead.
    spectator: bool,
    /// Our stats, as last sent. Kept whole because the server sends only what changed.
    stats: [i32; stat::COUNT],
    /// Whether our body has been set up as a bot the brain will drive.
    embodied: bool,
    /// The map's items and where they live, so their *absence* can be reasoned about.
    items: Vec<(EntId, Vec3)>,
    /// Submodel index → the shadow entity we spawned from it, so a moving door finds its twin.
    brushes: std::collections::HashMap<usize, EntId>,
    /// Networked entities we are mirroring into their own slots: where each was last frame, by
    /// server entity number. The previous position is the whole point — nothing sends a velocity,
    /// so a rocket's heading is the difference between two sightings of it.
    tracked: std::collections::HashMap<u16, Vec3>,
    /// The slots the nails live in.
    ///
    /// Nails come in a message of their own with no entity numbers attached, so unlike everything
    /// else on the wire they have nowhere to be written until we give them somewhere. Allocated once
    /// per map and reused every frame, because `svc_nails` is not a delta — it carries the whole set
    /// in flight, every time.
    nails: Vec<EntId>,
    /// The sound list. `svc_sound` carries an index into it, and the *name* is what says whether we
    /// just heard a rocket launcher or a footstep — so the bot's ears are downstream of this.
    sounds: Vec<String>,
    /// Who we last believed was alive, so a death is noticed as it happens. `PF_DEAD` is on the
    /// wire and authoritative — far better evidence than reading the obituary, and it works on any
    /// mod, whatever it calls the message.
    alive: [bool; MAX_CLIENTS],
    /// Who we last saw glowing, and in which colour, so a powerup is noticed the moment it lights up
    /// rather than every frame it stays lit.
    glowing: [Effects; MAX_CLIENTS],
    /// How many projectiles we've ever seen fly, and the most at once. A path that never runs looks
    /// exactly like one with nothing to do — this tells them apart.
    pub(crate) projectiles_seen: u32,
    pub(crate) projectiles_peak: usize,
}

impl Default for Mirror {
    fn default() -> Self {
        Mirror {
            playernum: 0,
            spectator: false,
            stats: [0; stat::COUNT],
            embodied: false,
            items: Vec::new(),
            brushes: Default::default(),
            tracked: Default::default(),
            nails: Vec::new(),
            sounds: Vec::new(),
            alive: [false; MAX_CLIENTS],
            glowing: [Effects::empty(); MAX_CLIENTS],
            projectiles_seen: 0,
            projectiles_peak: 0,
        }
    }
}

impl Mirror {
    /// Point the mirror at a player slot. Called when `svc_serverdata` says which one we are — and
    /// whether we're here to play or only to watch.
    pub(crate) fn set_slot(&mut self, playernum: u8, spectator: bool) {
        self.playernum = playernum;
        self.spectator = spectator;
        self.stats = [0; stat::COUNT];
        self.embodied = false;
    }

    /// Our own entity.
    pub(crate) fn own(&self) -> EntId {
        slot_to_ent(self.playernum)
    }

    /// Whether this connection is watching rather than playing.
    pub(crate) fn spectating(&self) -> bool {
        self.spectator
    }

    /// A stat, as last sent.
    pub(crate) fn stat(&self, which: u8) -> i32 {
        self.stats.get(which as usize).copied().unwrap_or(0)
    }

    /// Fold one server message into the world.
    pub(crate) fn apply(&mut self, game: &mut GameState, squad: &Squad, ev: &SvcEvent) {
        match ev {
            SvcEvent::ServerData(sd) => self.set_slot(sd.playernum, sd.spectator),
            SvcEvent::UpdateStat { stat, value } => {
                if let Some(slot) = self.stats.get_mut(*stat as usize) {
                    *slot = *value;
                }
                self.write_own_stats(game);
            }
            SvcEvent::PlayerInfo(pi) => self.write_player(game, squad, pi),
            // The bot's ears. Sounds carry by PHS rather than PVS — you hear things through walls,
            // which is the whole point of listening — so this reaches further than sight, and is
            // exactly what a player works from when they say "he's got the rocket launcher".
            SvcEvent::Sound { entity, sound, origin, .. } => {
                let name = self.sounds.get(*sound as usize).map(String::as_str).unwrap_or("");
                if let Some(weapon) = super::adapters::fire_sound(name) {
                    let e = EntId(*entity as u32);
                    if is_player_slot(game, e) {
                        game.client_heard_fire(e, weapon, *origin);
                    }
                }
            }
            // Being shot. Tells us someone's there and roughly where — a bearing, not a position.
            SvcEvent::Damage { armor, blood, from } => {
                let e = self.own();
                if game.entities[e].in_use {
                    game.client_felt_damage(e, *from, *armor as f32, *blood as f32);
                }
            }
            SvcEvent::SoundList(list) => {
                // Index 0 is a placeholder the server never sends; pad so the indices line up, or
                // every sound is named as the one before it.
                if self.sounds.is_empty() {
                    self.sounds.push(String::new());
                }
                self.sounds.truncate((list.start as usize).max(1));
                self.sounds.extend_from_slice(&list.names);
            }
            SvcEvent::Nails(nails) => self.write_nails(game, nails),
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
    ///
    /// A spectator has none of that and gets none of this.
    fn embody(&mut self, game: &mut GameState) {
        if self.spectator {
            return;
        }
        let e = self.own();
        let client = e.0 as i32;
        // The engine seeds a client's move cap from this extended field; without it a bot can jump
        // but not walk (`entity.rs`). Read before the entity borrow.
        let maxspeed = game.host().cvar(c"sv_maxspeed");
        let now = game.time();
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

        // Dead until the server says otherwise, which it is about to: our first `STAT_HEALTH` is
        // moments away and overwrites this.
        //
        // The default would be `No`, and a body with no health that isn't dead is a body the *server*
        // never spawned — so `run_bot` reaches for `put_client_in_server` to finish the job, which
        // teleports our shadow body to a spawn point and invents it a loadout. That's the engine's
        // work, and a client that does it is inventing a game the server isn't playing. Starting from
        // `Dead` puts the only honest reading of "no health yet" in the field: the respawn pulse,
        // which is exactly what a real client sends at a respawn prompt.
        ent.v.deadflag = DeadFlag::Dead;
        // A full tank, as `PutClientInServer` gives a fresh body. `write_air` keeps it topped up
        // from here; this is only for a body that starts its life underwater.
        ent.combat.air_finished = now + AIR_TIME;
        self.embodied = true;
    }

    /// Our stats into our own entity.
    ///
    /// Health is the one to be careful with: `deadflag` is what the brain's `is_alive` reads, and
    /// what decides whether `run_bot` tries to *respawn* us (pulsing +attack, which is what a real
    /// client does) or to play. Deriving it from health rather than guessing keeps the server-only
    /// spawn path — which a client must never take — permanently out of reach.
    fn write_own_stats(&mut self, game: &mut GameState) {
        // A spectator's "own" stats are whoever they're tracking — mvdsv sends the tracked player's
        // health and ammo, or zeros when tracking nobody. Neither is ours; adopting either would be
        // playing a stranger's body from the sidelines.
        if self.spectator {
            return;
        }
        if !self.embodied {
            self.embody(game);
        }
        let e = self.own();
        let health = self.stat(stat::HEALTH);
        let items = Items::from_bits_truncate(self.stat(stat::ITEMS) as u32);

        // A powerup's countdown isn't on the wire — only the bit. The moment it appears is the
        // moment it started, so the *rise* is the event and the previous value is needed to see it.
        let before = Items::from_bits_truncate(game.entities[e].v.items as u32);
        if before != items {
            game.client_note_own_powerups(e, before, items);
        }

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
    fn write_player(&mut self, game: &mut GameState, squad: &Squad, pi: &PlayerInfo) {
        if pi.player as usize >= MAX_CLIENTS {
            return; // not a slot this server has; everything below indexes by it
        }
        let e = slot_to_ent(pi.player);
        let own = !self.spectator && pi.player == self.playernum;
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
            // exactly while a live one's condition is a guess.
            //
            // Our own health arrives via stats and must not be clobbered here. Neither must a
            // squadmate's: they are another client of ours, with their own stats and their own mirror
            // that knows their health exactly, and this mirror has no more business overwriting it
            // than a stranger's. Every bot receives *everyone's* playerinfo, so without the squad
            // test each bot's health would be whatever the last mirror to run decided — which is the
            // placeholder below, forever, for all of them.
            if !own && !squad.ours(pi.player) {
                v.deadflag = if pi.dead() { DeadFlag::Dead } else { DeadFlag::No };
                // Enough to be "alive" for the brain's gates. What it's actually *worth* is the
                // opponent model's business, and writing a real number here would be a lie with a
                // decimal point on it.
                v.health = if pi.dead() { 0.0 } else { 100.0 };
            }
        }

        // A powerup is not a secret: the quad glows blue and the pentagram red, and `svc_playerinfo`
        // carries the effect bits, so a bot learns who has one the same way a player does — by
        // looking at them. Noting the moment it appears is what dates the ~30s window.
        if !own {
            let glow = Effects::from_bits_truncate(pi.effects.unwrap_or(0) as u32);
            let known = &mut self.glowing[pi.player as usize];
            for (bit, kind) in [
                (Effects::BLUE, PickupKind::Quad),
                (Effects::RED, PickupKind::Pent),
            ] {
                let lit = glow.contains(bit);
                if lit && !known.contains(bit) {
                    game.client_saw_pickup(e, kind);
                }
                known.set(bit, lit);
            }
        }

        // A death is the one moment an estimate stops being a guess: whoever that was is about to be
        // a fresh spawn, so everything we believed about how hurt they were is void. Noticing the
        // *transition* is what makes it an event rather than a state.
        let now_alive = !pi.dead();
        if self.alive[pi.player as usize] && !now_alive {
            game.client_saw_death(e);
        }
        self.alive[pi.player as usize] = now_alive;

        // Water is not on the wire either, but it's not a secret: anyone can see where the water is.
        // The map says, and we have the map.
        self.write_waterlevel(game, e);
        if own {
            self.write_air(game, e);
        }
        game.link_edict(e);
    }

    /// How long we can hold our breath.
    ///
    /// `WaterMove`'s rule, exactly: anything short of fully under refills the tank, because your head
    /// is out and you're breathing. Nothing sends it — the server owns `air_finished` and never
    /// mentions it — but the rule is public and the input (are we underwater) we work out from the map.
    ///
    /// Left unwritten it sits at zero, and `air_left` becomes `-now`: a number that starts negative
    /// and gets worse for as long as the process runs. The bot then meets the drowning panic the
    /// instant its eyes go under — drop everything, make for the surface — and can never leave it,
    /// because the panic's own way out is a breath, and a breath refills a tank nobody was filling.
    /// Only ours: nothing reads a stranger's air, and we'd be inventing it.
    fn write_air(&mut self, game: &mut GameState, e: EntId) {
        if game.entities[e].v.waterlevel < 3.0 {
            game.entities[e].combat.air_finished = game.time() + AIR_TIME;
        }
    }

    /// The nails in the air.
    ///
    /// Nails travel in a message of their own, six bit-packed bytes each, because a nailgun puts
    /// enough of them in the air to matter to the bandwidth — which is the same reason they matter
    /// to a bot. The dodge already knows what to do with a `Touch::Spike`; it had simply never been
    /// shown one, so a bot would stroll through a stream of nails it could see coming.
    ///
    /// Unlike an entity delta this is the **whole** set every frame, so the pool is rewritten from
    /// scratch and the tail cleared: a nail that hit something is a nail the next message just
    /// doesn't mention.
    ///
    /// No velocity is sent — but none is needed. A nail flies where it points at a speed the game
    /// fixes, so its heading *is* its velocity, which is more than can be said for a rocket's first
    /// frame.
    fn write_nails(&mut self, game: &mut GameState, nails: &[rtx_proto::svc::Nail]) {
        while self.nails.len() < nails.len() {
            let slot = game.host().spawn();
            if slot <= 0 {
                break; // out of slots; mirror what fits rather than nothing
            }
            self.nails.push(EntId(slot as u32));
        }

        for (i, &slot) in self.nails.iter().enumerate() {
            let Some(nail) = nails.get(i) else {
                // Past the end of this frame's set: whatever was here has landed or expired.
                game.entities[slot] = Entity::default();
                continue;
            };
            let ent = &mut game.entities[slot];
            *ent = Entity::default();
            ent.in_use = true;
            ent.v.movetype = MoveType::FlyMissile;
            ent.v.solid = Solid::BBox;
            ent.set_touch(Touch::Spike);
            ent.v.origin = nail.origin;
            ent.v.angles = Vec3::new(nail.pitch, nail.yaw, 0.0);
            let (fwd, _, _) = crate::math::angle_vectors(ent.v.angles);
            ent.v.velocity = fwd * NAIL_SPEED;
            game.link_edict(slot);
        }
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

/// What a networked entity turned out to be.
///
/// The wire says only "entity 43, model 17, here". Everything else — is that a rocket, is that the
/// red armour, is that the door — is inference from the model's *name*, which is why the model list
/// matters as much as the entity list.
enum Kind {
    /// Part of the level: a door, a plat, a trigger. `"*3"` — we have a shadow twin of it already,
    /// and the wire is telling us where it has moved to.
    Brush(usize),
    /// Something that flies and hurts.
    Projectile(Touch),
    /// A dead player's dropped weapon and ammo.
    Backpack,
    /// Decoration: gibs, blood, the player models we already track through `svc_playerinfo`.
    Ignore,
}

/// Read an entity's model name as what the entity *is*.
fn classify(model: &str) -> Kind {
    if let Some(n) = model.strip_prefix('*').and_then(|n| n.parse().ok()) {
        return Kind::Brush(n);
    }
    match model {
        "progs/missile.mdl" => Kind::Projectile(Touch::Missile),
        "progs/grenade.mdl" => Kind::Projectile(Touch::Grenade),
        "progs/spike.mdl" => Kind::Projectile(Touch::Spike),
        "progs/s_spike.mdl" => Kind::Projectile(Touch::SuperSpike),
        "progs/backpack.mdl" => Kind::Backpack,
        _ => Kind::Ignore,
    }
}

/// How long a grenade burns before it goes off. The wire never says — a grenade looks like any
/// other model — so the fuse is counted from when we first saw it, which is what a player does.
const GRENADE_FUSE: f32 = 2.5;

/// How close a networked entity must be to a shadow item to *be* that item. Items don't move, so
/// this only has to absorb the difference between where the mapper put it and where it settled.
const ITEM_MATCH_DIST: f32 = 48.0;

/// How near its end a mover has to be to count as resting there. The same eight units
/// `gate_closed_flags` calls a closed door, and for the same reason: the wire quantizes coordinates,
/// so "arrived" is a neighbourhood rather than a point.
const MOVER_REST: f32 = 8.0;

/// Beyond this, an item's absence says nothing: it's out of the server's PVS and simply isn't being
/// sent, whether it's there or not.
const ITEM_SIGHT_RANGE: f32 = 2000.0;

/// What this frame said about one item.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Evidence {
    /// We can see it. It's there.
    Present,
    /// We can see where it lives, and it isn't there. Someone took it.
    Taken,
    /// We couldn't have seen it either way, so this frame says nothing about it.
    Unknown,
}

/// What to conclude about an item, given what we saw and whether anybody had a look at the spot.
///
/// Split out from the looking so the rule can be read on its own, because the rule is the whole
/// point: presence is proof, and absence is proof only when we had a clear look at the spot. Anything
/// else is a guess, and a bot that guesses about items is a bot that walks across the map for
/// nothing — or, worse, one that knows where the quad is without anybody having gone to check.
fn item_evidence(visible_now: bool, looked: bool) -> Evidence {
    if visible_now {
        Evidence::Present
    } else if looked {
        Evidence::Taken
    } else {
        Evidence::Unknown
    }
}

/// Whether anyone in the squad had a clear look at where an item lives.
///
/// The conditions are joined per *eye*, and that's the whole subtlety: near enough **and** with a
/// clear line, both from the same body. Testing them separately across a squad — one bot close, a
/// different bot with the angle — concludes a look that nobody actually took.
fn anyone_looked(game: &mut GameState, squad: &Squad, home: Vec3) -> bool {
    let to = home + Vec3::new(0.0, 0.0, 24.0);
    squad
        .eyes
        .iter()
        .filter(|(_, eye)| eye.distance(home) <= ITEM_SIGHT_RANGE)
        .any(|&(body, eye)| game.client_traceline(eye, to, body).fraction >= 1.0)
}

impl Mirror {
    /// Fold this frame's entities into the world.
    ///
    /// Called once per frame with the union of what every bot can see — a squad shares one world, so
    /// an item one bot can see is an item they all know about, exactly as it is inside qwprogs.
    pub(crate) fn apply_frame(&mut self, game: &mut GameState, squad: &Squad, seen: &[EntityState], models: &[String]) {
        let now = game.time();
        let name = |m: u16| models.get(m as usize).map(String::as_str).unwrap_or("");

        // Anything we tracked last frame and don't see now is gone — a rocket that hit something, a
        // grenade that went off. Retire them before the new set lands.
        self.retire_unseen(game, seen);

        for e in seen {
            match classify(name(e.model)) {
                Kind::Brush(n) => self.write_brush(game, n, e),
                Kind::Projectile(touch) => self.write_projectile(game, e, touch, now),
                Kind::Backpack => self.write_backpack(game, e),
                Kind::Ignore => {}
            }
        }
        self.write_item_presence(game, squad, seen, models, now);
        self.projectiles_peak = self.projectiles_peak.max(self.tracked.len());
    }

    /// A piece of the level that has moved: a door opening, a plat rising.
    ///
    /// We spawned a twin of it from the map, so it already knows what it is and where it belongs;
    /// all the wire adds is where it is *now*.
    fn write_brush(&mut self, game: &mut GameState, submodel: usize, e: &EntityState) {
        let Some(twin) = self.brush_twin(game, submodel) else { return };
        let was = game.entities[twin].v.origin;
        game.set_origin(twin, e.origin);
        game.entities[twin].v.angles = e.angles;
        self.write_mover_phase(game, twin, was);
    }

    /// The shadow entity built from submodel `n`, if the map had one.
    ///
    /// Matched by the model's **name**, which is the only thing that identifies it. The submodel
    /// index looks like a tempting shortcut — `client_set_model` parks it in `modelindex` — but that
    /// field carries a flat `1.0` for every *external* model (a rocket, a backpack: their real index
    /// would come from the model list, and nothing client-side needs it). So `modelindex == 1` says
    /// "submodel `*1`" and "some rocket" in the same breath, and matching on it hands `*1`'s door
    /// updates to whichever unrelated entity happens to sit at a lower slot — an explosive box, say,
    /// which the door then drags around the map for the rest of the run, since the match is cached.
    ///
    /// A trigger can't be matched by mistake, incidentally: `InitTrigger` clears the name it was
    /// spawned with, because a trigger is invisible and the server never sends it either.
    fn brush_twin(&mut self, game: &GameState, submodel: usize) -> Option<EntId> {
        if let Some(&e) = self.brushes.get(&submodel) {
            return Some(e);
        }
        let name = format!("*{submodel}");
        let found = game
            .entities
            .live()
            .find(|(_, x)| x.model.as_deref() == Some(name.as_str()))
            .map(|(i, _)| i)?;
        self.brushes.insert(submodel, found);
        Some(found)
    }

    /// Where a lift has got to in its travel.
    ///
    /// The server never sends a mover's *phase* — only its position, like everything else — but the
    /// phase is what the bots read. `plat_statuses` uses it to hold a bot **outside** a raised lift's
    /// trigger until it comes down, and that standoff exists for a reason: a body standing in the
    /// shaft resets the lift's lower-timer, so a bot that walks in to wait is a bot waiting for a
    /// lift it is personally holding up. Left unwritten, the phase keeps its spawn value forever —
    /// `Bottom` for the common untargeted lift — and every raised lift reads as down.
    ///
    /// Only plats need this, which is a mercy. `pos1` and `pos2` mean *opposite* things for the two
    /// kinds of mover, faithfully reproduced from the original QuakeC: a door goes down to `pos1` and
    /// up to `pos2`; a plat goes up to `pos1` and down to `pos2`. Reading one the other's way would
    /// have a bot board a lift at the top of its shaft believing it was at the bottom. Doors are
    /// spared the question — nothing reads a door's phase, because the gate logic asks where the door
    /// *is* (`gate_closed_flags` compares its origin against the closed one), which the wire says
    /// outright.
    fn write_mover_phase(&mut self, game: &mut GameState, e: EntId, was: Vec3) {
        if game.entities[e].classname() != Some("func_plat") {
            return;
        }
        let (now, top, bottom) = {
            let ent = &game.entities[e];
            (ent.v.origin, ent.mover.pos1, ent.mover.pos2)
        };
        game.entities[e].mover.state = if now.distance(bottom) < MOVER_REST {
            MoverPhase::Bottom
        } else if now.distance(top) < MOVER_REST {
            MoverPhase::Top
        } else if now.z >= was.z {
            MoverPhase::Up
        } else {
            MoverPhase::Down
        };
    }

    /// Something in flight.
    ///
    /// Velocity isn't sent — a client sees a rocket's *position*, once per frame, like everyone
    /// else — so it's differenced from where the thing was last frame. That's a frame behind, which
    /// is exactly how far behind a player's read of it is too.
    fn write_projectile(&mut self, game: &mut GameState, e: &EntityState, touch: Touch, now: f32) {
        let slot = EntId(e.number as u32);
        let first = !self.tracked.contains_key(&e.number);
        let previous = self.tracked.get(&e.number).copied();

        // Who fired it, decided once, at the only moment the question is answerable.
        let owner = first.then(|| shooter_of(game, e.origin)).flatten();

        let ent = &mut game.entities[slot];
        if first {
            *ent = Entity::default();
            ent.in_use = true;
            ent.v.movetype = MoveType::FlyMissile;
            ent.v.solid = Solid::BBox;
            ent.set_touch(touch);
            if let Some(owner) = owner {
                ent.set_owner(owner);
            }
            // A grenade's fuse starts when we first see it. A rocket has no fuse; a nail has none
            // that matters.
            if touch == Touch::Grenade {
                ent.classname = Some("grenade".into());
                ent.think = crate::entity::Think::GrenadeExplode;
                ent.v.nextthink = now + GRENADE_FUSE;
            }
            self.projectiles_seen += 1;
        }
        ent.v.origin = e.origin;
        ent.v.angles = e.angles;
        ent.v.modelindex = e.model as f32;
        ent.combat.voided = 0.0;

        // Differenced velocity. A first sighting has nothing to difference against, so a rocket's
        // heading is taken from its angles — it flies where it points — which beats claiming it's
        // stationary for the one frame that matters most.
        let dt = game.globals.frametime.max(1e-3);
        ent.v.velocity = match previous {
            Some(p) if p != e.origin => (e.origin - p) / dt,
            _ if touch == Touch::Missile => {
                let (fwd, _, _) = crate::math::angle_vectors(e.angles);
                fwd * ROCKET_SPEED
            }
            _ => ent.v.velocity,
        };
        game.link_edict(slot);
        self.tracked.insert(e.number, e.origin);
    }

    /// A dead player's dropped kit.
    ///
    /// It's mirrored as a real entity so it exists to be seen and walked over, but deliberately not
    /// made a *goal*: what's inside is not on the wire, and a bot that pathed to a pack knowing what
    /// it held would know something it has no way of knowing. Reasoning about that from evidence —
    /// who died there, and what we last saw them holding — is the opponent model's business.
    fn write_backpack(&mut self, game: &mut GameState, e: &EntityState) {
        let slot = EntId(e.number as u32);
        if !self.tracked.contains_key(&e.number) {
            let ent = &mut game.entities[slot];
            *ent = Entity::default();
            ent.in_use = true;
            ent.set_touch(Touch::Backpack);
            ent.v.solid = Solid::Not; // present, but not a goal — see above
            ent.v.mins = Vec3::new(-16.0, -16.0, 0.0);
            ent.v.maxs = Vec3::new(16.0, 16.0, 56.0);
        }
        game.entities[slot].v.origin = e.origin;
        game.link_edict(slot);
        self.tracked.insert(e.number, e.origin);
    }

    /// Which of the map's items are actually there.
    ///
    /// The shadow world spawns every item the map has, all of them available, because that's the
    /// state a map file describes. The server then never mentions an item again until someone can
    /// see it — so *presence* is easy and *absence* is the interesting half.
    ///
    /// An absent item is only known taken if one of us would have seen it: within range, and with a
    /// clear line to where it should be. That's the same thing a player knows — you looked at the
    /// spot and the armour wasn't there — and it's why a bot can't tell whether the quad across the
    /// map is up until someone goes and looks.
    fn write_item_presence(
        &mut self,
        game: &mut GameState,
        squad: &Squad,
        seen: &[EntityState],
        models: &[String],
        now: f32,
    ) {
        // What's visibly present this frame: anything that isn't level brushwork, a projectile or a
        // pack, standing where an item lives.
        //
        // Deliberately *not* filtered by model name. An item is not necessarily a `progs/*.mdl` —
        // Quake ships health and ammo boxes as brush models in their own `.bsp` files
        // (`maps/b_bh25.bsp`, `maps/b_rock0.bsp`), which look nothing like the armour beside them.
        // Requiring a name pattern silently declared every health box on the map taken. Position is
        // the reliable signal: items don't move, so anything standing exactly where one lives almost
        // certainly is one. A stray gib landing on a spawn can say "up" when it isn't, which costs a
        // bot a walk to go and look — the opposite mistake sends it away from an item that's there.
        let mut present: Vec<Vec3> = Vec::new();
        for e in seen {
            let name = models.get(e.model as usize).map(String::as_str).unwrap_or("");
            if matches!(classify(name), Kind::Ignore) {
                present.push(e.origin);
            }
        }

        for idx in 0..self.items.len() {
            let (item, home) = self.items[idx];
            let visible_now = present.iter().any(|p| p.distance(home) < ITEM_MATCH_DIST);

            // Only pay for the traces when the answer could change anything.
            let looked = !visible_now && anyone_looked(game, squad, home);

            match item_evidence(visible_now, looked) {
                Evidence::Present => self.restore_item(game, item),
                Evidence::Taken => self.take_item(game, item, now),
                // Nothing seen either way — but a timer we started earlier can still come due.
                // That's *item timing*: we watched it go, we know the rule the server brings it back
                // by, so we know when to be there. An expectation, not a fact: the moment we can see
                // the spot again, what's actually there wins.
                Evidence::Unknown => self.expect_respawn(game, item, now),
            }
        }
    }

    /// Bring an item back on schedule, if its timer has come due.
    fn expect_respawn(&mut self, game: &mut GameState, item: EntId, now: f32) {
        let ent = &mut game.entities[item];
        if ent.think == crate::entity::Think::SubRegen && ent.v.nextthink <= now {
            ent.v.solid = Solid::Trigger;
            ent.think = crate::entity::Think::None;
            ent.v.nextthink = 0.0;
        }
    }

    /// This bot's body and where it looks from, for "would anyone have seen it".
    ///
    /// A spectator contributes nothing: we don't know where its camera is, and it has no body to
    /// trace from or to ignore.
    pub(crate) fn eyes(&self, game: &GameState) -> Option<(EntId, Vec3)> {
        if self.spectator {
            return None;
        }
        let e = self.own();
        let ent = game.entities.get(e.0 as usize)?;
        (ent.in_use && ent.is_alive()).then(|| (e, ent.v.origin + Vec3::new(0.0, 0.0, 22.0)))
    }

    /// Mark an item as taken, and expect it back on the server's schedule.
    ///
    /// Writes exactly what the server's own pickup writes — non-solid, with a `SubRegen` think
    /// scheduled — so `item_goal_valid` and `item_collect_time` read it without knowing the
    /// difference. A bot will still route to it and wait, which is what it should do.
    fn take_item(&mut self, game: &mut GameState, item: EntId, now: f32) {
        if game.entities[item].v.solid != Solid::Trigger {
            return; // already known gone
        }
        // The same rule the server's own pickup uses (`items.rs`), asked rather than copied.
        let delay = game.entities[item]
            .classname()
            .map(str::to_owned)
            .and_then(|cn| game.respawn_delay_of(&cn));
        let ent = &mut game.entities[item];
        ent.v.solid = Solid::Not;
        ent.v.modelindex = 0.0;
        match delay {
            Some(d) => {
                ent.think = crate::entity::Think::SubRegen;
                ent.v.nextthink = now + d;
            }
            // Deathmatch 2 doesn't respawn items at all, and neither does a dropped one.
            None => ent.think = crate::entity::Think::None,
        }
    }

    /// Mark an item as present.
    fn restore_item(&mut self, game: &mut GameState, item: EntId) {
        let ent = &mut game.entities[item];
        ent.v.solid = Solid::Trigger;
        ent.think = crate::entity::Think::None;
        ent.v.nextthink = 0.0;
    }

    /// Drop anything we were tracking that the server has stopped sending.
    fn retire_unseen(&mut self, game: &mut GameState, seen: &[EntityState]) {
        let live: std::collections::HashSet<u16> = seen.iter().map(|e| e.number).collect();
        self.tracked.retain(|&num, _| {
            if live.contains(&num) {
                return true;
            }
            let slot = EntId(num as u32);
            if let Some(ent) = game.entities.get_mut(slot.0 as usize) {
                *ent = Entity::default();
            }
            false
        });
    }

    /// What the mirror currently believes, for the report: items up, items known taken, and how
    /// many things are in the air.
    pub(crate) fn census(&self, game: &GameState) -> (usize, usize, usize) {
        let up = self
            .items
            .iter()
            .filter(|(e, _)| game.entities[*e].v.solid == Solid::Trigger)
            .count();
        // "Waiting" rather than "gone": we saw it taken and we know when it's due back.
        let waiting = self
            .items
            .iter()
            .filter(|(e, _)| game.entities[*e].think == crate::entity::Think::SubRegen)
            .count();
        (up, waiting, self.tracked.len())
    }

    /// Note the map's items, so their absence can be reasoned about. Called once per map, after the
    /// shadow world is spawned.
    pub(crate) fn index_items(&mut self, game: &GameState) {
        self.items = game
            .entities
            .live()
            .filter(|(_, e)| e.classname().is_some_and(crate::bot::goals::is_goal_classname))
            .map(|(i, e)| (i, e.v.origin))
            .collect();
        self.brushes.clear();
        self.tracked.clear();
        // The slots these named belong to the old map — `spawn_shadow_world` has reset the
        // allocator, and holding them would hand the new map's furniture to the nails.
        self.nails.clear();
    }
}

/// A rocket's speed. Fixed in QuakeWorld, and the reason a rocket's heading can be read off its
/// angles the moment it appears, before there are two frames to difference.
const ROCKET_SPEED: f32 = 1000.0;

/// A nail's speed (`W_FireSpikes`). Fixed too, and the whole of what makes a nail's packed pitch and
/// yaw enough to know where it's going.
const NAIL_SPEED: f32 = 1000.0;

/// How long a lungful lasts (`WaterMove`).
const AIR_TIME: f32 = 12.0;

/// How near a projectile's first sighting a player has to be to have fired it.
///
/// A rocket is spawned at the muzzle, a stride in front of the shooter's eyes, and the first frame
/// we see it has already flown for one server frame — about 14 units at rocket speed. Generous
/// enough to survive both, tight enough that a bystander doesn't get the credit.
const MUZZLE_DIST: f32 = 160.0;

/// Who fired the thing that just appeared here.
///
/// The wire doesn't say — an entity update is a position and a model, and `owner` is a server-side
/// field no client ever receives. But it isn't a mystery either: a projectile is born at its
/// shooter's muzzle, so the player standing where it appeared is the one who fired it, which is
/// exactly the inference a human makes from the same sight.
///
/// It matters more than attribution usually does. `owner` is what the dodge reads to skip **our own**
/// rockets, so a projectile with no owner is one every bot flinches from — including the bot that
/// just fired it, at the exact moment it wants to hold its aim. It's also what marks a grenade as
/// *enemy* owned, without which the shoot-it-down tactic never triggers at all.
fn shooter_of(game: &GameState, at: Vec3) -> Option<EntId> {
    let maxclients = game.host().cvar(c"maxclients") as u32;
    (1..=maxclients)
        .map(EntId)
        .filter(|&p| game.entities[p].in_use && game.entities[p].is_alive())
        .map(|p| (p, game.entities[p].v.origin.distance(at)))
        .filter(|&(_, d)| d <= MUZZLE_DIST)
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map(|(p, _)| p)
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
        host.set("maxclients", "8");
        GameState::new_client(host)
    }

    /// A live body standing at `at`, the way a mirrored player is.
    fn player(g: &mut GameState, slot: u32, at: Vec3) -> EntId {
        let e = EntId(slot);
        let ent = &mut g.entities[e];
        ent.in_use = true;
        ent.classname = Some("player".into());
        ent.v.health = 100.0;
        ent.v.origin = at;
        ent.v.mins = crate::defs::VEC_HULL_MIN;
        ent.v.maxs = crate::defs::VEC_HULL_MAX;
        e
    }

    /// The squad as the tick loop builds it, for a client of one: this mirror's slot, and its eyes.
    fn squad_of(m: &Mirror, g: &GameState) -> Squad {
        let mut slots = [false; MAX_CLIENTS];
        slots[m.playernum as usize] = !m.spectating();
        Squad::new(slots, m.eyes(g).into_iter().collect())
    }

    /// Fold in one message the way the tick loop does: work out the squad from the world as it
    /// currently stands, then write.
    fn apply(m: &mut Mirror, g: &mut GameState, ev: &SvcEvent) {
        let squad = squad_of(m, g);
        m.apply(g, &squad, ev);
    }

    /// Likewise for a frame's worth of entities.
    fn apply_frame(m: &mut Mirror, g: &mut GameState, seen: &[EntityState], models: &[String]) {
        let squad = squad_of(m, g);
        m.apply_frame(g, &squad, seen, models);
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
        m.set_slot(2, false);
        apply(&mut m, &mut g, &SvcEvent::UpdateStat { stat: stat::HEALTH, value: 100 });

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
        m.set_slot(0, false);

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
            apply(&mut m, &mut g, &SvcEvent::UpdateStat { stat: s, value: v });
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
        m.set_slot(0, false);

        apply(&mut m, &mut g, &SvcEvent::UpdateStat { stat: stat::HEALTH, value: 100 });
        assert!(g.entities[m.own()].is_alive());
        assert_eq!(g.entities[m.own()].v.deadflag, DeadFlag::No);

        apply(&mut m, &mut g, &SvcEvent::UpdateStat { stat: stat::HEALTH, value: 0 });
        assert!(!g.entities[m.own()].is_alive());
        assert_eq!(g.entities[m.own()].v.deadflag, DeadFlag::Dead);

        // And back, on respawn.
        apply(&mut m, &mut g, &SvcEvent::UpdateStat { stat: stat::HEALTH, value: 100 });
        assert!(g.entities[m.own()].is_alive());
    }

    /// A player's own client sent their view angles and the server passed them on — so a bot can
    /// know where an opponent is looking, exactly as a human reads it off the model.
    #[test]
    fn writes_another_players_position_and_aim() {
        let mut g = game();
        let mut m = Mirror::default();
        m.set_slot(0, false);
        apply(&mut m, &mut g, &SvcEvent::UpdateUserinfo {
            player: 3,
            userid: 7,
            userinfo: "\\name\\victim".to_string(),
        });

        let mut pi = playerinfo(3, rtx_proto::svc::pf::ONGROUND);
        pi.velocity = Vec3::new(320.0, 0.0, 0.0);
        pi.command = Some(Usercmd { angles: Vec3::new(-10.0, 90.0, 0.0), ..Default::default() });
        apply(&mut m, &mut g, &SvcEvent::PlayerInfo(pi));

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
        apply(&mut m, &mut g, &SvcEvent::PlayerInfo(playerinfo(3, 0)));
        assert!(!g.entities[slot_to_ent(3)].v.flags.has(Flags::ONGROUND));
    }

    /// An opponent's death is on the wire; their health never is. The mirror must not invent one.
    #[test]
    fn another_players_death_is_known_but_their_health_is_not() {
        let mut g = game();
        let mut m = Mirror::default();
        m.set_slot(0, false);
        apply(&mut m, &mut g, &SvcEvent::UpdateUserinfo {
            player: 1,
            userid: 1,
            userinfo: "\\name\\enemy".to_string(),
        });

        apply(&mut m, &mut g, &SvcEvent::PlayerInfo(playerinfo(1, rtx_proto::svc::pf::DEAD)));
        assert!(!g.entities[slot_to_ent(1)].is_alive(), "death is authoritative");

        apply(&mut m, &mut g, &SvcEvent::PlayerInfo(playerinfo(1, 0)));
        let v = &g.entities[slot_to_ent(1)].v;
        assert!(v.health > 0.0, "alive enough for the brain's gates");
        assert_eq!(v.armorvalue, 0.0, "and no invented armour — that's the opponent model's job");
    }

    /// A squadmate's body belongs to the squadmate's own mirror.
    ///
    /// Every bot receives *everyone's* playerinfo, our own bots' included, and a mirror can tell its
    /// own body from the rest but not a friend's from a stranger's. Without the squad to ask, each
    /// mirror writes the "alive enough" placeholder over the others — so every bot in the squad reads
    /// its own health as 100 no matter what the server just did to it, and none of them ever retreats.
    #[test]
    fn a_squadmate_keeps_the_health_its_own_mirror_knows() {
        let mut g = game();
        // Two of ours: slot 0 (this mirror) and slot 1. Slot 2 is a stranger.
        let mut ours = [false; MAX_CLIENTS];
        (ours[0], ours[1]) = (true, true);
        let mate = player(&mut g, slot_to_ent(1).0, Vec3::ZERO);
        player(&mut g, slot_to_ent(2).0, Vec3::ZERO);

        // Our squadmate is badly hurt, as their own mirror's stats established.
        g.entities[mate].v.health = 12.0;

        let mut m = Mirror::default();
        m.set_slot(0, false);
        let squad = Squad::new(ours, Vec::new());
        m.apply(&mut g, &squad, &SvcEvent::PlayerInfo(playerinfo(1, 0)));
        assert_eq!(g.entities[mate].v.health, 12.0, "not this mirror's to overwrite");

        // A stranger is a different matter: we've no stats for them and never will, so the
        // placeholder is the honest thing to write.
        m.apply(&mut g, &squad, &SvcEvent::PlayerInfo(playerinfo(2, 0)));
        assert_eq!(g.entities[slot_to_ent(2)].v.health, 100.0, "alive, and that's all we can say");
    }

    /// A body with no health that isn't dead is a body the server never spawned — and `run_bot` will
    /// try to finish the job itself, which is the engine's work and a client's business never. The
    /// window is small (our first stat is moments away) and it reopens on every map change, so the
    /// mirror closes it at the only moment it can: the instant the body exists.
    #[test]
    fn a_body_starts_dead_rather_than_unspawned() {
        let mut g = game();
        let mut m = Mirror::default();
        m.set_slot(0, false);

        // Playerinfo arrives before any stat — the body exists, its health doesn't yet.
        apply(&mut m, &mut g, &SvcEvent::PlayerInfo(playerinfo(0, 0)));
        let ent = &g.entities[m.own()];
        assert!(ent.in_use && ent.bot.is_bot, "embodied");
        assert!(!ent.is_alive());
        assert_eq!(ent.v.deadflag, DeadFlag::Dead, "so run_bot pulses +attack, as a real client does");

        // And the moment the server says otherwise, it's alive.
        apply(&mut m, &mut g, &SvcEvent::UpdateStat { stat: stat::HEALTH, value: 100 });
        assert!(g.entities[m.own()].is_alive());
    }

    /// A spectator has a slot and a scoreboard entry but no body. Nothing here may drive one.
    ///
    /// It matters because mvdsv sends a spectator the *tracked player's* stats: a mirror that
    /// embodied anyway would take a stranger's health and ammo for its own and play off them, and the
    /// respawn pulse for the 0-health slot it started on would cycle the camera through the server's
    /// players. `--spectate` is how this thing is soaked, so it has to be genuinely quiet.
    #[test]
    fn a_spectator_watches_and_nothing_more() {
        let mut g = game();
        let mut m = Mirror::default();
        m.set_slot(3, true);
        assert!(m.spectating());

        // The stats of whoever the server is showing us. Not ours.
        for (s, v) in [(stat::HEALTH, 100), (stat::ARMOR, 200), (stat::ROCKETS, 30)] {
            apply(&mut m, &mut g, &SvcEvent::UpdateStat { stat: s, value: v });
        }
        let ent = &g.entities[m.own()];
        assert!(!ent.in_use, "no body");
        assert!(!ent.bot.is_bot, "and so nothing for run_bots to pick up and drive");
        assert_eq!(ent.v.health, 0.0, "and no stranger's health worn as our own");

        // It contributes no eyes either — we don't know where a spectator's camera is.
        assert_eq!(m.eyes(&g), None);
    }

    /// A spectator holds a slot but isn't in the game, and an emptied slot is gone. Either one left
    /// looking like a player is an enemy the bot would hunt and never find.
    #[test]
    fn spectators_and_leavers_are_not_players() {
        let mut g = game();
        let mut m = Mirror::default();
        m.set_slot(0, false);

        apply(&mut m, &mut g, &SvcEvent::UpdateUserinfo {
            player: 4,
            userid: 4,
            userinfo: "\\name\\watcher\\*spectator\\1".to_string(),
        });
        assert!(!g.entities[slot_to_ent(4)].is_player());

        // Joins for real…
        apply(&mut m, &mut g, &SvcEvent::UpdateUserinfo {
            player: 4,
            userid: 4,
            userinfo: "\\name\\watcher".to_string(),
        });
        assert!(g.entities[slot_to_ent(4)].is_player());

        // …then leaves.
        apply(&mut m, &mut g, &SvcEvent::UpdateUserinfo {
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
        m.set_slot(0, false);
        apply(&mut m, &mut g, &SvcEvent::UpdateStat { stat: stat::HEALTH, value: 100 });

        let angles = Vec3::new(0.0, 135.0, 0.0);
        apply(&mut m, &mut g, &SvcEvent::SetAngle { kind: Some(1), angles });
        assert_eq!(g.entities[m.own()].v.v_angle, angles);
        assert_eq!(g.entities[m.own()].bot.aim.angles, angles, "the spring goes too");
    }

    fn state(number: u16, model: u16, origin: Vec3) -> EntityState {
        EntityState { number, model, origin, ..Default::default() }
    }

    /// A model list shaped like a real one: index 0 is the placeholder, 1 the map.
    fn models() -> Vec<String> {
        ["", "maps/dm4.bsp", "progs/missile.mdl", "progs/grenade.mdl", "progs/backpack.mdl",
         "progs/armor.mdl", "maps/b_bh25.bsp", "*3", "progs/gib1.mdl"]
            .iter().map(|s| s.to_string()).collect()
    }

    /// An entity is only what its *model name* says. The trap worth pinning: Quake ships health and
    /// ammo boxes as brush models in their own `.bsp` files, so "is it an item" is emphatically not
    /// "does it start with progs/".
    #[test]
    fn classifies_entities_by_model_name() {
        assert!(matches!(classify("*3"), Kind::Brush(3)));
        assert!(matches!(classify("*17"), Kind::Brush(17)));
        assert!(matches!(classify("progs/missile.mdl"), Kind::Projectile(Touch::Missile)));
        assert!(matches!(classify("progs/grenade.mdl"), Kind::Projectile(Touch::Grenade)));
        assert!(matches!(classify("progs/spike.mdl"), Kind::Projectile(Touch::Spike)));
        assert!(matches!(classify("progs/backpack.mdl"), Kind::Backpack));

        // Items — including the ones that are `.bsp` files and look nothing like the rest.
        assert!(matches!(classify("progs/armor.mdl"), Kind::Ignore));
        assert!(matches!(classify("maps/b_bh25.bsp"), Kind::Ignore));
        assert!(matches!(classify("maps/b_rock0.bsp"), Kind::Ignore));
        assert!(matches!(classify(""), Kind::Ignore));
    }

    /// A rocket in flight becomes something the dodge logic can reason about — and its velocity is
    /// differenced from where it was, because the wire never says how fast anything is going.
    #[test]
    fn tracks_a_rocket_and_differences_its_velocity() {
        let mut g = game();
        g.globals.frametime = 0.1;
        let mut m = Mirror::default();
        m.set_slot(0, false);
        let models = models();

        // First sighting: nothing to difference against, so the heading comes off its angles — a
        // rocket flies where it points, and that one frame is the one that matters most.
        let mut e = state(50, 2, Vec3::new(0.0, 0.0, 0.0));
        e.angles = Vec3::ZERO; // facing +x
        apply_frame(&mut m, &mut g, &[e], &models);

        let slot = EntId(50);
        assert!(g.entities[slot].in_use);
        assert_eq!(g.entities[slot].touch, Touch::Missile);
        assert!(g.entities[slot].v.velocity.x > 900.0, "{:?}", g.entities[slot].v.velocity);
        assert_eq!(m.projectiles_seen, 1);

        // Second frame: a real difference.
        apply_frame(&mut m, &mut g, &[state(50, 2, Vec3::new(100.0, 0.0, 0.0))], &models);
        assert_eq!(g.entities[slot].v.velocity, Vec3::new(1000.0, 0.0, 0.0));
        assert_eq!(g.entities[slot].v.origin.x, 100.0);

        // It hits something and the server stops sending it — the slot must be released, or the bot
        // dodges a rocket that no longer exists.
        apply_frame(&mut m, &mut g, &[], &models);
        assert!(!g.entities[slot].in_use);
    }

    /// Put a lift into the world the way `spawn_func_plat` does: `pos1` at the top of its travel,
    /// `pos2` at the bottom, resting at the bottom.
    fn place_plat(g: &mut GameState, model: &'static str, top: f32) -> EntId {
        let e = EntId(1200);
        let ent = &mut g.entities[e];
        *ent = Entity::default();
        ent.in_use = true;
        ent.classname = Some("func_plat".into());
        ent.model = Some(model.into());
        ent.mover.pos1 = Vec3::new(0.0, 0.0, top);
        ent.mover.pos2 = Vec3::ZERO;
        ent.v.origin = Vec3::ZERO;
        ent.mover.state = MoverPhase::Bottom;
        e
    }

    /// A lift's phase, from the only thing the wire says about it: where it is.
    ///
    /// Nothing sends a mover's phase, and it's the phase the bots read — `plat_statuses` holds a bot
    /// outside a raised lift's trigger until it descends, because a body in the shaft resets the
    /// lower-timer and the lift then never comes down for it. Unwritten, the phase keeps its spawn
    /// value (`Bottom`) forever, and the bot walks into the shaft of a lift at the top of its travel
    /// and waits there for a lift it is itself holding up.
    #[test]
    fn a_lift_gets_its_phase_from_where_it_has_got_to() {
        let mut g = game();
        let mut m = Mirror::default();
        m.set_slot(0, false);
        let plat = place_plat(&mut g, "*3", 128.0);
        let models = models(); // index 7 is "*3"

        // At the top of its shaft. It is emphatically not down, whatever it spawned believing.
        apply_frame(&mut m, &mut g, &[state(80, 7, Vec3::new(0.0, 0.0, 128.0))], &models);
        assert_eq!(g.entities[plat].mover.state, MoverPhase::Top);

        // On its way down, and still not down.
        apply_frame(&mut m, &mut g, &[state(80, 7, Vec3::new(0.0, 0.0, 64.0))], &models);
        assert_eq!(g.entities[plat].mover.state, MoverPhase::Down);

        // Arrived — now a bot may board it.
        apply_frame(&mut m, &mut g, &[state(80, 7, Vec3::new(0.0, 0.0, 2.0))], &models);
        assert_eq!(g.entities[plat].mover.state, MoverPhase::Bottom, "within the resting tolerance");

        // And back up, called by someone else.
        apply_frame(&mut m, &mut g, &[state(80, 7, Vec3::new(0.0, 0.0, 40.0))], &models);
        assert_eq!(g.entities[plat].mover.state, MoverPhase::Up);
    }

    /// A brush is matched by its model's **name**, not by the index parked in `modelindex`.
    ///
    /// Every external model carries a flat `modelindex` of 1, so matching on the number makes `*1`
    /// ambiguous with every rocket and crate on the map — and the match is cached, so a door that
    /// grabs an explosive box drags it around for the rest of the run.
    #[test]
    fn a_door_finds_its_own_twin_and_not_a_lookalike() {
        let mut g = game();
        let mut m = Mirror::default();
        m.set_slot(0, false);

        // An external-model entity at a lower slot, wearing the `modelindex` that used to collide.
        let decoy = EntId(20);
        g.entities[decoy].in_use = true;
        g.entities[decoy].classname = Some("misc_explobox".into());
        g.entities[decoy].model = Some("progs/b_explob.bsp".into());
        g.entities[decoy].v.modelindex = 1.0;
        g.entities[decoy].v.solid = Solid::BBox;

        let plat = place_plat(&mut g, "*1", 128.0);
        g.entities[plat].v.modelindex = 1.0; // as `client_set_model` leaves a submodel `*1`

        let mut models = models();
        models[7] = "*1".to_string();
        apply_frame(&mut m, &mut g, &[state(80, 7, Vec3::new(0.0, 0.0, 128.0))], &models);

        assert_eq!(g.entities[plat].v.origin.z, 128.0, "the door moved");
        assert_eq!(g.entities[decoy].v.origin, Vec3::ZERO, "and the box stayed where it was");
    }

    /// Who fired it is not on the wire — `owner` is a server-side field no client receives — but it
    /// isn't a mystery: a rocket is born at its shooter's muzzle, so whoever is standing where it
    /// appeared fired it. That's the same inference a human makes from the same sight.
    ///
    /// Without it every projectile is owned by the world, and the dodge's "not my own rocket" test
    /// passes for *everything* — so a bot flinches from the rocket it just fired, at precisely the
    /// moment it wants to hold its aim on what it fired at.
    #[test]
    fn a_rocket_belongs_to_whoever_it_appeared_in_front_of() {
        let mut g = game();
        let mut m = Mirror::default();
        m.set_slot(0, false);

        let shooter = player(&mut g, 2, Vec3::new(100.0, 0.0, 0.0));
        let bystander = player(&mut g, 3, Vec3::new(600.0, 0.0, 0.0));

        // A rocket appears at the shooter's muzzle, a stride in front of them.
        apply_frame(&mut m, &mut g, &[state(50, 2, Vec3::new(130.0, 0.0, 16.0))], &models());
        assert_eq!(g.entities[EntId(50)].owner(), shooter);
        assert!(g.entities[EntId(50)].owner().is_some(), "or the shootable-grenade tactic never fires");

        // One that appears in the open belongs to nobody, and a bystander doesn't get the credit.
        apply_frame(&mut m, &mut g, &[state(51, 2, Vec3::new(2000.0, 0.0, 16.0))], &models());
        assert_ne!(g.entities[EntId(51)].owner(), bystander);
        assert!(!g.entities[EntId(51)].owner().is_some(), "nobody was near enough to have fired it");

        // And a dead body fires nothing.
        g.entities[shooter].v.health = 0.0;
        g.entities[shooter].v.deadflag = DeadFlag::Dead;
        apply_frame(&mut m, &mut g, &[state(52, 2, Vec3::new(130.0, 0.0, 16.0))], &models());
        assert_ne!(g.entities[EntId(52)].owner(), shooter);
    }

    /// Nails travel in a message of their own — six packed bytes each, because a nailgun puts enough
    /// of them in the air to matter — and they carry no entity number, so unlike everything else on
    /// the wire they need somewhere to be put. Unconsumed, a stream of nails is invisible: the dodge
    /// knows what a `Touch::Spike` is and had simply never been shown one.
    #[test]
    fn nails_get_slots_and_a_heading() {
        use rtx_proto::svc::Nail;
        let mut g = game();
        let mut m = Mirror::default();
        m.set_slot(0, false);

        let nail = |x: f32, yaw: f32| Nail { number: None, origin: Vec3::new(x, 0.0, 0.0), pitch: 0.0, yaw };
        apply(&mut m, &mut g, &SvcEvent::Nails(vec![nail(10.0, 0.0), nail(20.0, 90.0)]));

        assert_eq!(m.nails.len(), 2);
        let first = &g.entities[m.nails[0]];
        assert!(first.in_use);
        assert_eq!(first.touch, Touch::Spike, "which is exactly what the dodge is looking for");
        assert_eq!(first.v.origin.x, 10.0);
        // No velocity is sent, and none is needed: a nail flies where it points, at a fixed speed.
        assert!(first.v.velocity.x > 900.0, "{:?}", first.v.velocity);
        assert!(g.entities[m.nails[1]].v.velocity.y > 900.0, "yawed 90° — flying +y");

        // The message is the *whole* set every frame, not a delta: one that hit something is simply
        // not mentioned again, and must not hang in the air for a bot to keep dodging.
        apply(&mut m, &mut g, &SvcEvent::Nails(vec![nail(30.0, 0.0)]));
        assert!(g.entities[m.nails[0]].in_use);
        assert!(!g.entities[m.nails[1]].in_use, "gone, not still flying");

        // And the pool is reused rather than regrown — the slots are the same ones.
        let pool = m.nails.clone();
        apply(&mut m, &mut g, &SvcEvent::Nails(vec![nail(40.0, 0.0), nail(50.0, 0.0)]));
        assert_eq!(m.nails, pool);
    }

    /// The air tank. Nothing sends it — the server owns `air_finished` and never mentions it — but
    /// `WaterMove`'s rule is public: anything short of fully under is a breath.
    ///
    /// Unwritten it sits at zero, and the bot's `air_left` becomes `-now`: a number that starts
    /// negative and grows worse for as long as the process runs. The bot then hits the drowning panic
    /// the instant its eyes go under and can never leave it, because the panic's own way out is a
    /// breath — and a breath refills a tank nobody was filling.
    #[test]
    fn a_breath_refills_the_tank_and_being_under_does_not() {
        let mut g = game();
        g.globals.time = 500.0;
        let mut m = Mirror::default();
        m.set_slot(0, false);
        apply(&mut m, &mut g, &SvcEvent::UpdateStat { stat: stat::HEALTH, value: 100 });

        // A body starts with a full one, as `PutClientInServer` gives it.
        assert_eq!(g.entities[m.own()].combat.air_finished, 500.0 + AIR_TIME);

        // Out of the water: topped up, every frame, from now.
        g.globals.time = 520.0;
        apply(&mut m, &mut g, &SvcEvent::PlayerInfo(playerinfo(0, 0)));
        assert_eq!(g.entities[m.own()].v.waterlevel, 0.0, "the map has no water in it — nor a map");
        assert_eq!(g.entities[m.own()].combat.air_finished, 520.0 + AIR_TIME);

        // Fully under: the tank runs down, because that's the whole idea of a tank.
        g.entities[m.own()].v.waterlevel = 3.0;
        g.globals.time = 525.0;
        m.write_air(&mut g, m.own());
        assert_eq!(g.entities[m.own()].combat.air_finished, 532.0, "still counting down from the last breath");

        // Head back out and it's a fresh lungful.
        g.entities[m.own()].v.waterlevel = 2.0;
        m.write_air(&mut g, m.own());
        assert_eq!(g.entities[m.own()].combat.air_finished, 525.0 + AIR_TIME);
    }

    /// A grenade has a fuse, and the wire never mentions it. Counting from first sighting is what a
    /// player does, and what makes "is it about to go off" answerable at all.
    #[test]
    fn a_grenade_gets_a_fuse_from_when_we_first_saw_it() {
        let mut g = game();
        g.globals.time = 100.0;
        let mut m = Mirror::default();
        m.set_slot(0, false);

        apply_frame(&mut m, &mut g, &[state(60, 3, Vec3::ZERO)], &models());
        let ent = &g.entities[EntId(60)];
        assert_eq!(ent.touch, Touch::Grenade);
        assert_eq!(ent.classname(), Some("grenade"));
        assert_eq!(ent.v.nextthink, 100.0 + GRENADE_FUSE);
    }

    /// A pack is mirrored so it exists, but is deliberately not a goal: what's in it isn't on the
    /// wire, and a bot that pathed to one knowing its contents would know something it can't.
    #[test]
    fn a_backpack_exists_but_is_not_a_goal() {
        let mut g = game();
        let mut m = Mirror::default();
        m.set_slot(0, false);
        apply_frame(&mut m, &mut g, &[state(70, 4, Vec3::new(5.0, 5.0, 5.0))], &models());

        let ent = &g.entities[EntId(70)];
        assert!(ent.in_use);
        assert_eq!(ent.touch, Touch::Backpack);
        assert_ne!(ent.v.solid, Solid::Trigger, "the goal scan requires Trigger — it must not qualify");
    }

    /// Put a real item into the world at `home`, the way the shadow world would.
    fn place_item(g: &mut GameState, at: Vec3, classname: &'static str) -> EntId {
        let e = EntId(1500);
        let ent = &mut g.entities[e];
        *ent = Entity::default();
        ent.in_use = true;
        ent.classname = Some(classname.into());
        ent.v.origin = at;
        ent.v.solid = Solid::Trigger;
        e
    }

    /// The heart of item knowledge, and the reason it's honest: an item's *absence* only means
    /// something if we'd have seen it. Out of range, it says nothing at all — which is why a bot
    /// can't know whether the quad across the map is up until someone goes and looks.
    #[test]
    fn an_items_absence_is_only_evidence_when_we_could_see_it() {
        let mut g = game();
        g.globals.time = 50.0;
        let mut m = Mirror::default();
        m.set_slot(0, false);
        apply(&mut m, &mut g, &SvcEvent::UpdateStat { stat: stat::HEALTH, value: 100 });

        // An item far across the map, and us standing at the origin.
        let far = Vec3::new(9999.0, 0.0, 0.0);
        let item = place_item(&mut g, far, "item_armor2");
        m.items = vec![(item, far)];
        g.entities[m.own()].v.origin = Vec3::ZERO;

        apply_frame(&mut m, &mut g, &[], &models());
        assert_eq!(g.entities[item].v.solid, Solid::Trigger, "too far away to conclude anything");
    }

    /// Seeing it is the simple half.
    #[test]
    fn seeing_an_item_says_it_is_there() {
        let mut g = game();
        let mut m = Mirror::default();
        m.set_slot(0, false);
        apply(&mut m, &mut g, &SvcEvent::UpdateStat { stat: stat::HEALTH, value: 100 });

        let at = Vec3::new(64.0, 0.0, 0.0);
        let item = place_item(&mut g, at, "item_armor2");
        m.items = vec![(item, at)];
        // Believed taken…
        g.entities[item].v.solid = Solid::Not;
        g.entities[item].think = crate::entity::Think::SubRegen;

        // …then we see it. What's actually there always wins over what we expected.
        apply_frame(&mut m, &mut g, &[state(90, 5, at)], &models());
        assert_eq!(g.entities[item].v.solid, Solid::Trigger);
        assert_eq!(g.entities[item].think, crate::entity::Think::None);
    }

    /// The rule, on its own: presence is proof; absence is proof only when somebody had a clear look.
    /// Anything else is a guess — and a bot that guesses about items either walks across the map for
    /// nothing, or knows where the quad is without anybody having gone to check.
    #[test]
    fn absence_is_only_evidence_with_a_clear_look() {
        // Seen: there, whatever else is true.
        assert_eq!(item_evidence(true, false), Evidence::Present);
        assert_eq!(item_evidence(true, true), Evidence::Present);

        // Not seen, and somebody had a clear look at the spot: someone took it.
        assert_eq!(item_evidence(false, true), Evidence::Taken);

        // Not seen, and nobody looked — that's not evidence of anything.
        assert_eq!(item_evidence(false, false), Evidence::Unknown);
    }

    /// Nobody looks through a map we haven't got.
    ///
    /// A trace with no BSP behind it reports solid ([`super::host::no_map`]), so `anyone_looked` says
    /// no and every item keeps whatever we last knew about it. That's fail-closed, and the direction
    /// worth failing in: a client that concluded "taken" from a trace it couldn't answer would empty
    /// the map of items on the strength of not having read it, and send its bots to stand hopefully
    /// over bare floor. It's also why the tests around this one are honest about asserting so little
    /// — without a map on disk there's no geometry here to see anything through.
    #[test]
    fn nobody_looks_through_a_map_we_do_not_have() {
        let mut g = game();
        let home = Vec3::new(500.0, 0.0, 0.0);

        assert!(!anyone_looked(&mut g, &Squad::default(), home), "nobody to do the looking");

        let near = Squad::new([false; MAX_CLIENTS], vec![(EntId(1), Vec3::new(600.0, 0.0, 0.0))]);
        assert!(!anyone_looked(&mut g, &near, home), "right there, and still no map to look through");
    }


    /// Item timing: we watched it go, we know the rule it comes back by, so we know when to be
    /// there.    /// Item timing: we watched it go, we know the rule it comes back by, so we know when to be
    /// there. The brain reads this as `SubRegen` + `nextthink` — the same fields the server's own
    /// pickup writes — so `item_goal_valid` still routes a bot there to wait.
    #[test]
    fn a_taken_item_is_timed_and_comes_back_on_schedule() {
        let mut g = game();
        g.globals.time = 100.0;
        g.level.deathmatch = 1;
        let mut m = Mirror::default();
        m.set_slot(0, false);

        let at = Vec3::new(64.0, 0.0, 0.0);
        let item = place_item(&mut g, at, "item_armor2");

        // We looked, and it was gone.
        m.take_item(&mut g, item, 100.0);
        assert_eq!(g.entities[item].v.solid, Solid::Not);
        assert_eq!(g.entities[item].think, crate::entity::Think::SubRegen);
        assert_eq!(g.entities[item].v.nextthink, 120.0, "armour is a 20-second item");

        // Not due yet, and we still can't see it.
        m.expect_respawn(&mut g, item, 115.0);
        assert_eq!(g.entities[item].v.solid, Solid::Not);

        // Due — expect it back, without having watched it return.
        m.expect_respawn(&mut g, item, 121.0);
        assert_eq!(g.entities[item].v.solid, Solid::Trigger, "timed back in");
        assert_eq!(g.entities[item].think, crate::entity::Think::None);
    }

    /// An item that never respawns must not be timed back in, or a bot queues forever for something
    /// that isn't coming.
    #[test]
    fn an_item_that_never_respawns_is_not_timed() {
        let mut g = game();
        g.level.deathmatch = 2; // nothing respawns here
        let mut m = Mirror::default();
        m.set_slot(0, false);

        let item = place_item(&mut g, Vec3::ZERO, "item_armor2");
        m.take_item(&mut g, item, 100.0);
        assert_eq!(g.entities[item].v.solid, Solid::Not);
        assert_eq!(g.entities[item].think, crate::entity::Think::None, "no schedule to wait on");

        m.expect_respawn(&mut g, item, 100_000.0);
        assert_eq!(g.entities[item].v.solid, Solid::Not, "and it never comes back");
    }

    /// The respawn rule is the server's, and it's asked rather than copied — including the modes
    /// where the answer changes.
    #[test]
    fn respawn_timing_follows_the_servers_rules() {
        let mut g = game();

        g.level.deathmatch = 1;
        assert_eq!(g.respawn_delay_of("item_armor2"), Some(20.0));
        assert_eq!(g.respawn_delay_of("weapon_rocketlauncher"), Some(30.0));
        assert_eq!(g.respawn_delay_of("item_artifact_super_damage"), Some(60.0));
        assert_eq!(g.respawn_delay_of("item_artifact_invulnerability"), Some(300.0));

        // Weapons stay put in dm 3/5, so a weapon is quick; dm 2 respawns nothing at all.
        g.level.deathmatch = 3;
        assert_eq!(g.respawn_delay_of("weapon_rocketlauncher"), Some(15.0));
        g.level.deathmatch = 2;
        assert_eq!(g.respawn_delay_of("item_armor2"), None);
        assert_eq!(g.respawn_delay_of("weapon_rocketlauncher"), None);
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
