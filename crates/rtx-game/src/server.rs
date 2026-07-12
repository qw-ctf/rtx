// SPDX-License-Identifier: AGPL-3.0-or-later

//! Level flow, ported from `qw-qc/server.qc` + the changelevel/intermission code in
//! `client.qc`: rule-limit level changes, the intermission camera, and `trigger_changelevel`.

use crate::defs::*;
use crate::entity::{EntId, Think, Touch};
use crate::game::{cstring, GameState};
use crate::obituary::DeathType;

impl GameState {
    /// `NextLevel` — queue the next map and start intermission once a rule limit is hit.
    pub(crate) fn next_level(&mut self) {
        if self.intermission_running {
            return;
        }
        // A configured rotation (`rtx_maplist`) wins; else a serverinfo `nextmap`; else replay.
        self.level.nextmap = if let Some(m) = self.queued_next_map() {
            m
        } else {
            let mut buf = [0u8; 64];
            let nextmap = self.host.infokey(EntId::WORLD, c"nextmap", &mut buf).to_owned();
            if nextmap.is_empty() {
                self.level.mapname.clone()
            } else {
                nextmap
            }
        };
        self.execute_changelevel();
    }

    /// The next map in the configured rotation — `rtx_maplist` is a whitespace-separated list of map
    /// names, cycled after the current map. `None` when no list is set (the stock `nextmap`/replay
    /// behaviour then applies). If the current map isn't in the list, the rotation starts at its
    /// first entry.
    pub(crate) fn queued_next_map(&self) -> Option<String> {
        // Match the engine's own `set` ceiling (`MAX_COM_TOKEN`/config-line = 1024) so we never read
        // less of the list than a config could set — ~80-100 map names, far more than any rotation.
        let mut buf = [0u8; 1024];
        let list = self.host.cvar_string(c"rtx_maplist", &mut buf);
        let maps: Vec<&str> = list.split_whitespace().collect();
        if maps.is_empty() {
            return None;
        }
        let cur = self.level.mapname.as_str();
        let next = match maps.iter().position(|&m| m == cur) {
            Some(i) => maps[(i + 1) % maps.len()],
            None => maps[0],
        };
        Some(next.to_owned())
    }

    /// Per-frame rotation driver: once the intermission scoreboard has been shown for its pause,
    /// advance to the next map **without** waiting for a player button press — but only when a
    /// rotation (`rtx_maplist`) is configured, so the stock button-to-advance intermission is
    /// unchanged otherwise. Called each server frame from `start_frame`.
    pub(crate) fn map_queue_frame(&mut self) {
        if !self.intermission_running || self.time() < self.intermission_exit_time {
            return;
        }
        if self.queued_next_map().is_some() {
            self.goto_next_map();
        }
    }

    /// `FindIntermission` — the camera entity for the scoreboard view.
    fn find_intermission(&self) -> EntId {
        self.find_by_classname("info_intermission")
            .next()
            .or_else(|| self.find_by_classname("info_player_start").next())
            .unwrap_or(EntId::WORLD)
    }

    /// `execute_changelevel` — freeze players and show the intermission view.
    pub(crate) fn execute_changelevel(&mut self) {
        self.intermission_running = true;
        self.intermission_exit_time = self.time() + 5.0;

        let pos = self.find_intermission();
        let (origin, mangle) = {
            let p = &self.entities[pos];
            (p.v.origin, p.mover.mangle)
        };

        self.host.write_svc(MsgDest::All, Svc::CdTrack);
        self.host.write_byte(MsgDest::All, 3);
        self.host.write_svc(MsgDest::All, Svc::Intermission);
        self.write_coords(MsgDest::All, origin);
        self.write_angles(MsgDest::All, mangle);

        let players: Vec<EntId> = self.find_by_classname("player").collect();
        for p in players {
            let v = &mut self.entities[p].v;
            v.takedamage = TakeDamage::No;
            v.solid = Solid::Not;
            v.movetype = MoveType::None;
            v.modelindex = 0.0;
        }
    }

    /// `IntermissionThink` — a button press during intermission advances the map.
    pub(crate) fn intermission_think(&mut self) {
        if self.time() < self.intermission_exit_time {
            return;
        }
        let player = EntId::from_prog(self.globals.self_);
        let v = &self.entities[player].v;
        if v.button0 == 0.0 && v.button1 == 0.0 && v.button2 == 0.0 {
            return;
        }
        self.goto_next_map();
    }

    /// `GotoNextMap`.
    fn goto_next_map(&mut self) {
        let samelevel = self.host.cvar(c"samelevel") as i32;
        // `nextmap` wins only when we aren't forcing the same level and one was actually set;
        // otherwise (samelevel, or no nextmap) we replay the current map.
        let map = if samelevel != 1 && !self.level.nextmap.is_empty() {
            &self.level.nextmap
        } else {
            &self.level.mapname
        };
        let c = cstring(map);
        self.host.changelevel(&c);
    }

    /// `changelevel_touch`.
    pub(crate) fn changelevel_touch(&mut self, e: EntId, other: EntId) {
        if !self.entities[other].is_player() {
            return;
        }
        let samelevel = self.host.cvar(c"samelevel") as i32;
        if samelevel == 2 || (samelevel == 3 && self.level.mapname != "start") {
            self.entities[other].deathtype = DeathType::Changelevel;
            self.t_damage(other, e, e, 50000.0);
            return;
        }
        let name = self.netname_of(other);
        self.broadcast(PrintLevel::High, &format!("{name} exited the level\n"));
        self.level.nextmap = self.entities[e].map.as_deref().unwrap_or("").to_owned();
        self.activator = other;
        self.sub_use_targets(e);
        let time = self.time();
        let ent = &mut self.entities[e];
        ent.set_touch(Touch::None);
        ent.think = Think::ExecuteChangelevel;
        ent.v.nextthink = time + 0.1;
    }

    /// `trigger_changelevel` spawn.
    pub(crate) fn spawn_trigger_changelevel(&mut self, e: EntId) -> bool {
        if self.entities[e].map.is_none() {
            return false;
        }
        self.init_trigger(e);
        self.entities[e].set_touch(Touch::Changelevel);
        true
    }
}
