// SPDX-License-Identifier: AGPL-3.0-or-later

//! Spectator hooks, ported from `qw-qc/spectate.qc`. mvdsv routes the spectator variants of
//! connect/disconnect/postthink here (selected by the `isSpectator` argument in `game.rs`).

use crate::defs;
use crate::entity::EntId;
use crate::game::GameState;

impl GameState {
    /// `SpectatorConnect`.
    pub(crate) fn spectator_connect(&mut self, e: EntId) {
        let name = self.read_netname(e);
        let ent = &mut self.entities[e];
        ent.in_use = true;
        ent.netname = Some(name.as_str().into());
        ent.set_goalentity(EntId::WORLD);
        self.broadcast(
            defs::PrintLevel::Medium,
            &format!("Spectator {name} entered the game\n"),
        );
    }

    /// `PutSpectatorInServer` — reset the impulse cycle target.
    pub(crate) fn put_spectator_in_server(&mut self, e: EntId) {
        self.entities[e].set_goalentity(EntId::WORLD);
    }

    /// `SpectatorDisconnect`.
    pub(crate) fn spectator_disconnect(&mut self, e: EntId) {
        let name = self.entities[e].netname.as_deref().unwrap_or("").to_owned();
        self.broadcast(defs::PrintLevel::Medium, &format!("Spectator {name} left the game\n"));
    }

    /// `SpectatorThink` — handle the free-fly cycle impulse.
    pub(crate) fn spectator_think(&mut self, e: EntId) {
        if self.entities[e].v.impulse != 0.0 {
            self.spectator_impulse_command(e);
        }
    }

    /// `SpectatorImpulseCommand` — impulse 1 jumps to the next deathmatch spawn.
    fn spectator_impulse_command(&mut self, e: EntId) {
        if self.entities[e].v.impulse as i32 == 1 {
            let after = self.entities[e].goalentity();
            let spot = self
                .find_by_classname_after("info_player_deathmatch", after)
                .or_else(|| self.find_by_classname("info_player_deathmatch").next());
            if let Some(spot) = spot {
                self.entities[e].set_goalentity(spot);
                let (origin, angles) = {
                    let v = &self.entities[spot].v;
                    (v.origin, v.angles)
                };
                self.set_origin(e, origin);
                let ent = &mut self.entities[e];
                ent.v.origin = origin;
                ent.v.angles = angles;
                ent.v.fixangle = 1.0;
            }
        }
        self.entities[e].v.impulse = 0.0;
    }

    /// First entity with `classname` at an index greater than `after`.
    fn find_by_classname_after(&self, name: &str, after: EntId) -> Option<EntId> {
        self.find_where(move |e| e.classname() == Some(name))
            .find(|e| e.index() > after.index())
    }
}
