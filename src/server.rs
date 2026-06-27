//! Level flow, ported from `qw-qc/server.qc` + the changelevel/intermission code in
//! `client.qc`: rule-limit level changes, the intermission camera, and `trigger_changelevel`.

use crate::defs::*;
use crate::entity::{EntId, Think, Touch};
use crate::game::{cstring, GameState};

impl GameState {
    /// `NextLevel` — queue the next map and start intermission once a rule limit is hit.
    pub(crate) fn next_level(&mut self) {
        if self.intermission_running {
            return;
        }
        // Prefer a serverinfo `nextmap`, else replay the current level.
        let mut buf = [0u8; 64];
        let nextmap = self.host.infokey(0, c"nextmap", &mut buf).to_owned();
        self.level.nextmap = if nextmap.is_empty() {
            self.level.mapname.clone()
        } else {
            nextmap
        };
        self.execute_changelevel();
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
            (p.v.origin, p.mangle)
        };

        self.host.write_svc(MsgDest::All, Svc::CdTrack);
        self.host.write_byte(MsgDest::All, 3);
        self.host.write_svc(MsgDest::All, Svc::Intermission);
        self.host.write_coord(MsgDest::All, origin.x);
        self.host.write_coord(MsgDest::All, origin.y);
        self.host.write_coord(MsgDest::All, origin.z);
        self.host.write_angle(MsgDest::All, mangle.x);
        self.host.write_angle(MsgDest::All, mangle.y);
        self.host.write_angle(MsgDest::All, mangle.z);

        let players: Vec<EntId> = self.find_by_classname("player").collect();
        for p in players {
            let v = &mut self.entities[p].v;
            v.takedamage = TakeDamage::No.as_f32();
            v.solid = Solid::Not.as_f32();
            v.movetype = MoveType::None.as_f32();
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
        let map = if samelevel == 1 {
            self.level.mapname.clone()
        } else if !self.level.nextmap.is_empty() {
            self.level.nextmap.clone()
        } else {
            self.level.mapname.clone()
        };
        let c = cstring(&map);
        self.host.changelevel(&c);
    }

    /// `changelevel_touch`.
    pub(crate) fn changelevel_touch(&mut self, e: EntId, other: EntId) {
        if self.entities[other].classname() != Some("player") {
            return;
        }
        let samelevel = self.host.cvar(c"samelevel") as i32;
        if samelevel == 2 || (samelevel == 3 && self.level.mapname != "start") {
            self.t_damage(other, e, e, 50000.0);
            return;
        }
        let name = self.netname_of(other);
        self.broadcast(PrintLevel::High, &format!("{name} exited the level\n"));
        self.level.nextmap = self
            .entities[e]
            .map
            .as_deref()
            .unwrap_or("")
            .to_owned();
        self.activator = other;
        self.sub_use_targets(e);
        let time = self.time();
        let ent = &mut self.entities[e];
        ent.touch = Touch::None;
        ent.think = Think::ExecuteChangelevel;
        ent.v.nextthink = time + 0.1;
    }

    /// `trigger_changelevel` spawn.
    pub(crate) fn spawn_trigger_changelevel(&mut self, e: EntId) -> bool {
        if self.entities[e].map.is_none() {
            return false;
        }
        self.init_trigger(e);
        self.entities[e].touch = Touch::Changelevel;
        true
    }
}
