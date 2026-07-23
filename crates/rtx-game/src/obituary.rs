// SPDX-License-Identifier: AGPL-3.0-or-later

//! Death cause + obituary flavour text, ported from KTX `ClientObituary` (`ktx/src/client.c`).
//!
//! QuakeWorld clients (ezQuake, …) scrape the console for death messages with fragfile-style
//! regexes to attribute frags and weapons, so these strings must match KTX **byte-for-byte**.
//! [`DeathType`] is stamped on the victim at the damage site (the KTX `deathType_t` role) and read
//! exactly once, here, to pick the message. The string tables live in pure functions so they can be
//! unit-tested against the reference without a live [`GameState`].

use crate::defs::{Content, FieldEq, PrintLevel};
use crate::entity::EntId;
use crate::game::GameState;

/// How a player died. Stamped on the victim just before the lethal `t_damage`, cleared each
/// `player_pre_think`. Only the causes rtx can actually produce are modelled; KTX variants with no
/// rtx source (lasers, fireballs, stomp landings, dmm4 self-discharge, spawnicide, monsters) are
/// omitted.
#[derive(Default, Clone, Copy, PartialEq, Eq, Debug)]
pub(crate) enum DeathType {
    #[default]
    None,
    Axe,
    Shotgun,
    SuperShotgun,
    Nailgun,
    SuperNailgun,
    Grenade,
    Rocket,
    LightningBeam,
    /// Underwater lightning discharge (KTX `dtLG_DIS`).
    Discharge,
    /// Grappling hook (KTX `dtHOOK`).
    Hook,
    /// Crushed by a door/plat/train (KTX `dtSQUISH`).
    Squish,
    Fall,
    /// Ordinary telefrag (KTX `dtTELE1`).
    Telefrag,
    /// A mortal telefrags a pentagram carrier and dies instead (KTX `dtTELE2`).
    TelefragDeflected,
    /// Two pentagram carriers telefrag each other (KTX `dtTELE3`).
    TelefragMutual,
    /// Exploding barrel (KTX `dtEXPLO_BOX`).
    ExploBox,
    /// Drowning (KTX `dtWATER_DMG`).
    Water,
    Slime,
    Lava,
    TriggerHurt,
    /// Killed by a `samelevel`-blocked exit (KTX `dtCHANGELEVEL`).
    Changelevel,
    /// The console `kill` command (KTX `dtSUICIDE`).
    Suicide,
}

impl DeathType {
    /// Any telefrag flavour (KTX `TELEDEATH`).
    pub(crate) fn is_telefrag(self) -> bool {
        matches!(self, Self::Telefrag | Self::TelefragDeflected | Self::TelefragMutual)
    }
}

impl GameState {
    /// `ClientObituary` — pick the death message, apply frag scoring + `logfrag`, and broadcast it
    /// at [`PrintLevel::Medium`]. Message selection is delegated to the pure functions below.
    pub(crate) fn client_obituary(&mut self, targ: EntId, attacker: EntId) {
        if !self.entities[targ].is_player() {
            return;
        }
        let death = self.entities[targ].deathtype;
        let victim = self.netname_of(targ);

        // Pentagram telefrag specials resolve before the player/world split (KTX client.c:5218).
        match death {
            DeathType::TelefragDeflected => {
                self.dock_frag(targ, 1.0);
                self.broadcast(
                    PrintLevel::Medium,
                    &format!("Satan's power deflects {victim}'s telefrag\n"),
                );
                return;
            }
            DeathType::TelefragMutual => {
                let att = self.netname_of(attacker);
                self.dock_frag(targ, 1.0);
                self.broadcast(
                    PrintLevel::Medium,
                    &format!("{victim} was telefragged by {att}'s Satan's power\n"),
                );
                return;
            }
            _ => {}
        }

        let r = self.random();
        let attacker_is_player = self.entities[attacker].is_player();

        if !attacker_is_player {
            // World / environment death.
            let health = self.entities[targ].v.health;
            self.dock_frag(targ, 1.0);
            let s = world_death_string(death, health, r);
            self.broadcast(PrintLevel::Medium, &format!("{victim}{s}"));
            return;
        }

        if targ == attacker {
            // Killed self with a weapon / discharge / console `kill`.
            let watertype = self.entities[targ].v.watertype;
            let dock = if death == DeathType::Suicide { 2.0 } else { 1.0 };
            self.dock_frag(targ, dock);
            let s = self_kill_string(death, watertype, r);
            self.broadcast(PrintLevel::Medium, &format!("{victim}{s}"));
            return;
        }

        let att = self.netname_of(attacker);
        if self.obituary_is_teamkill(targ, attacker) {
            let (msg, docks) = teamkill_message(death, &victim, &att, r);
            if docks {
                self.dock_frag(attacker, 1.0); // ZOID: killing a teammate logs as a suicide
            }
            self.broadcast(PrintLevel::Medium, &msg);
            return;
        }

        // Normal kill.
        let gibbed = self.entities[targ].v.health < -40.0;
        let quad = self.entities[attacker].combat.super_damage_finished > 0.0;
        self.award_frag(attacker, 1.0, targ);
        let msg = frag_message(death, &victim, &att, gibbed, quad, r);
        self.broadcast(PrintLevel::Medium, &msg);
    }

    /// Whether `attacker`'s kill of `targ` counts as a teamkill (same non-empty team, teamplay on).
    fn obituary_is_teamkill(&self, targ: EntId, attacker: EntId) -> bool {
        if self.level.teamplay == 0 {
            return false;
        }
        let at = self.team_of(attacker);
        !at.is_empty() && at == self.team_of(targ)
    }
}

/// Variant index for a KTX `(int)(g_random() * n)` pick; `r` is a single draw in `[0, 1)`.
fn pick(r: f32, n: u32) -> u32 {
    (r * n as f32) as u32
}

/// Message appended after the victim's name for a self-inflicted death (`{victim}{ret}`).
fn self_kill_string(death: DeathType, watertype: f32, r: f32) -> &'static str {
    match death {
        DeathType::Grenade => " tries to put the pin back in\n",
        DeathType::Rocket => match pick(r, 2) {
            0 => " discovers blast radius\n",
            _ => " becomes bored with life\n",
        },
        DeathType::Squish => " was squished\n",
        DeathType::Discharge => {
            if watertype.is(Content::Slime) {
                " discharges into the slime\n"
            } else if watertype.is(Content::Lava) {
                " discharges into the lava\n"
            } else {
                match pick(r, 2) {
                    0 => " heats up the water\n",
                    _ => " discharges into the water\n",
                }
            }
        }
        DeathType::Suicide => " suicides\n",
        _ => " somehow becomes bored with life\n",
    }
}

/// Full teamkill message, plus whether the attacker loses a frag (team telefrags don't, by default).
fn teamkill_message(death: DeathType, victim: &str, attacker: &str, r: f32) -> (String, bool) {
    match death {
        DeathType::Telefrag => (format!("{victim} was telefragged by his teammate\n"), false),
        DeathType::Squish => (format!("{attacker} squished a teammate\n"), true),
        _ => {
            let ds = match pick(r, 4) {
                0 => " checks his glasses\n",
                1 => " loses another friend\n",
                2 => " gets a frag for the other team\n",
                _ => " mows down a teammate\n",
            };
            (format!("{attacker}{ds}"), true)
        }
    }
}

/// Full message for a normal player-vs-player kill.
fn frag_message(death: DeathType, victim: &str, attacker: &str, gibbed: bool, quad: bool, r: f32) -> String {
    match death {
        DeathType::Telefrag => format!("{victim} was telefragged by {attacker}\n"),
        DeathType::Squish => format!("{attacker} squishes {victim}\n"),
        DeathType::Nailgun => {
            let ds = match pick(r, 2) {
                0 => " was body pierced by ",
                _ => " was nailed by ",
            };
            format!("{victim}{ds}{attacker}\n")
        }
        DeathType::SuperNailgun => {
            let ds = if gibbed {
                " was straw-cuttered by "
            } else {
                match pick(r, 3) {
                    0 => " was punctured by ",
                    1 => " was perforated by ",
                    _ => " was ventilated by ",
                }
            };
            format!("{victim}{ds}{attacker}\n")
        }
        DeathType::Grenade => {
            if gibbed {
                format!("{victim} was gibbed by {attacker}'s grenade\n")
            } else {
                format!("{victim} eats {attacker}'s pineapple\n")
            }
        }
        DeathType::Rocket => {
            if quad && gibbed {
                match pick(r, 3) {
                    0 => format!("{victim} was brutalized by {attacker}'s quad rocket\n"),
                    1 => format!("{victim} was smeared by {attacker}'s quad rocket\n"),
                    _ => format!("{attacker} rips {victim} a new one\n"),
                }
            } else {
                let ds = if gibbed { " was gibbed by " } else { " rides " };
                format!("{victim}{ds}{attacker}'s rocket\n")
            }
        }
        DeathType::Axe => format!("{victim} was ax-murdered by {attacker}\n"),
        DeathType::Hook => format!("{victim} was hooked by {attacker}\n"),
        DeathType::Shotgun => {
            if gibbed {
                format!("{victim} was lead poisoned by {attacker}\n")
            } else {
                format!("{victim} chewed on {attacker}'s boomstick\n")
            }
        }
        DeathType::SuperShotgun => {
            let ds = if quad { " ate 8 loads of " } else { " ate 2 loads of " };
            format!("{victim}{ds}{attacker}'s buckshot\n")
        }
        DeathType::LightningBeam => {
            if gibbed {
                format!("{victim} gets a natural disaster from {attacker}\n")
            } else {
                format!("{victim} accepts {attacker}'s shaft\n")
            }
        }
        DeathType::Discharge => match pick(r, 2) {
            0 => format!("{victim} drains {attacker}'s batteries\n"),
            _ => format!("{victim} accepts {attacker}'s discharge\n"),
        },
        _ => format!("{victim} killed by {attacker} ?\n"),
    }
}

/// Message appended after the victim's name for a world/environment death (`{victim}{ret}`).
fn world_death_string(death: DeathType, health: f32, r: f32) -> &'static str {
    match death {
        DeathType::ExploBox => " blew up\n",
        DeathType::Fall => match pick(r, 2) {
            0 => " cratered\n",
            _ => " fell to his death\n",
        },
        DeathType::Nailgun | DeathType::SuperNailgun => " was spiked\n",
        DeathType::Changelevel => " tried to leave\n",
        DeathType::Squish => " was squished\n",
        DeathType::Water => match pick(r, 2) {
            0 => " sleeps with the fishes\n",
            _ => " sucks it down\n",
        },
        DeathType::Slime => match pick(r, 2) {
            0 => " gulped a load of slime\n",
            _ => " can't exist on slime alone\n",
        },
        DeathType::Lava => {
            if health < -15.0 {
                " burst into flames\n"
            } else {
                match pick(r, 2) {
                    0 => " turned into hot slag\n",
                    _ => " visits the Volcano God\n",
                }
            }
        }
        // TriggerHurt and any unmodelled world cause.
        _ => " died\n",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Random buckets: 0.1→idx0, 0.4→idx1, 0.6→idx2, 0.9→idx3 for the largest (4-way) pick.
    const SLIME: f32 = -4.0; // Content::Slime
    const LAVA: f32 = -5.0; // Content::Lava
    const WATER: f32 = -3.0; // Content::Water

    #[test]
    fn self_kill_strings() {
        assert_eq!(
            self_kill_string(DeathType::Grenade, WATER, 0.0),
            " tries to put the pin back in\n"
        );
        assert_eq!(
            self_kill_string(DeathType::Rocket, WATER, 0.1),
            " discovers blast radius\n"
        );
        assert_eq!(
            self_kill_string(DeathType::Rocket, WATER, 0.9),
            " becomes bored with life\n"
        );
        assert_eq!(self_kill_string(DeathType::Squish, WATER, 0.0), " was squished\n");
        assert_eq!(
            self_kill_string(DeathType::Discharge, SLIME, 0.0),
            " discharges into the slime\n"
        );
        assert_eq!(
            self_kill_string(DeathType::Discharge, LAVA, 0.0),
            " discharges into the lava\n"
        );
        assert_eq!(
            self_kill_string(DeathType::Discharge, WATER, 0.1),
            " heats up the water\n"
        );
        assert_eq!(
            self_kill_string(DeathType::Discharge, WATER, 0.9),
            " discharges into the water\n"
        );
        assert_eq!(self_kill_string(DeathType::Suicide, WATER, 0.0), " suicides\n");
        assert_eq!(
            self_kill_string(DeathType::Fall, WATER, 0.0),
            " somehow becomes bored with life\n"
        );
    }

    #[test]
    fn teamkill_strings() {
        assert_eq!(
            teamkill_message(DeathType::Telefrag, "V", "A", 0.0),
            ("V was telefragged by his teammate\n".to_string(), false)
        );
        assert_eq!(
            teamkill_message(DeathType::Squish, "V", "A", 0.0),
            ("A squished a teammate\n".to_string(), true)
        );
        assert_eq!(
            teamkill_message(DeathType::Rocket, "V", "A", 0.1).0,
            "A checks his glasses\n"
        );
        assert_eq!(
            teamkill_message(DeathType::Rocket, "V", "A", 0.3).0,
            "A loses another friend\n"
        );
        assert_eq!(
            teamkill_message(DeathType::Rocket, "V", "A", 0.6).0,
            "A gets a frag for the other team\n"
        );
        assert_eq!(
            teamkill_message(DeathType::Rocket, "V", "A", 0.9).0,
            "A mows down a teammate\n"
        );
    }

    #[test]
    fn frag_strings() {
        assert_eq!(
            frag_message(DeathType::Telefrag, "V", "A", false, false, 0.0),
            "V was telefragged by A\n"
        );
        assert_eq!(
            frag_message(DeathType::Squish, "V", "A", false, false, 0.0),
            "A squishes V\n"
        );
        assert_eq!(
            frag_message(DeathType::Nailgun, "V", "A", false, false, 0.1),
            "V was body pierced by A\n"
        );
        assert_eq!(
            frag_message(DeathType::Nailgun, "V", "A", false, false, 0.9),
            "V was nailed by A\n"
        );
        assert_eq!(
            frag_message(DeathType::SuperNailgun, "V", "A", false, false, 0.1),
            "V was punctured by A\n"
        );
        assert_eq!(
            frag_message(DeathType::SuperNailgun, "V", "A", false, false, 0.4),
            "V was perforated by A\n"
        );
        assert_eq!(
            frag_message(DeathType::SuperNailgun, "V", "A", false, false, 0.9),
            "V was ventilated by A\n"
        );
        assert_eq!(
            frag_message(DeathType::SuperNailgun, "V", "A", true, false, 0.0),
            "V was straw-cuttered by A\n"
        );
        assert_eq!(
            frag_message(DeathType::Grenade, "V", "A", false, false, 0.0),
            "V eats A's pineapple\n"
        );
        assert_eq!(
            frag_message(DeathType::Grenade, "V", "A", true, false, 0.0),
            "V was gibbed by A's grenade\n"
        );
        assert_eq!(
            frag_message(DeathType::Rocket, "V", "A", false, false, 0.0),
            "V rides A's rocket\n"
        );
        assert_eq!(
            frag_message(DeathType::Rocket, "V", "A", true, false, 0.0),
            "V was gibbed by A's rocket\n"
        );
        assert_eq!(
            frag_message(DeathType::Rocket, "V", "A", true, true, 0.1),
            "V was brutalized by A's quad rocket\n"
        );
        assert_eq!(
            frag_message(DeathType::Rocket, "V", "A", true, true, 0.4),
            "V was smeared by A's quad rocket\n"
        );
        assert_eq!(
            frag_message(DeathType::Rocket, "V", "A", true, true, 0.9),
            "A rips V a new one\n"
        );
        // Quad but not gibbed still rides the plain rocket line.
        assert_eq!(
            frag_message(DeathType::Rocket, "V", "A", false, true, 0.0),
            "V rides A's rocket\n"
        );
        assert_eq!(
            frag_message(DeathType::Axe, "V", "A", false, false, 0.0),
            "V was ax-murdered by A\n"
        );
        assert_eq!(
            frag_message(DeathType::Hook, "V", "A", false, false, 0.0),
            "V was hooked by A\n"
        );
        assert_eq!(
            frag_message(DeathType::Shotgun, "V", "A", false, false, 0.0),
            "V chewed on A's boomstick\n"
        );
        assert_eq!(
            frag_message(DeathType::Shotgun, "V", "A", true, false, 0.0),
            "V was lead poisoned by A\n"
        );
        assert_eq!(
            frag_message(DeathType::SuperShotgun, "V", "A", false, false, 0.0),
            "V ate 2 loads of A's buckshot\n"
        );
        assert_eq!(
            frag_message(DeathType::SuperShotgun, "V", "A", false, true, 0.0),
            "V ate 8 loads of A's buckshot\n"
        );
        assert_eq!(
            frag_message(DeathType::LightningBeam, "V", "A", false, false, 0.0),
            "V accepts A's shaft\n"
        );
        assert_eq!(
            frag_message(DeathType::LightningBeam, "V", "A", true, false, 0.0),
            "V gets a natural disaster from A\n"
        );
        assert_eq!(
            frag_message(DeathType::Discharge, "V", "A", false, false, 0.1),
            "V drains A's batteries\n"
        );
        assert_eq!(
            frag_message(DeathType::Discharge, "V", "A", false, false, 0.9),
            "V accepts A's discharge\n"
        );
        assert_eq!(
            frag_message(DeathType::None, "V", "A", false, false, 0.0),
            "V killed by A ?\n"
        );
    }

    #[test]
    fn world_death_strings() {
        assert_eq!(world_death_string(DeathType::ExploBox, 0.0, 0.0), " blew up\n");
        assert_eq!(world_death_string(DeathType::Fall, 0.0, 0.1), " cratered\n");
        assert_eq!(world_death_string(DeathType::Fall, 0.0, 0.9), " fell to his death\n");
        assert_eq!(world_death_string(DeathType::Nailgun, 0.0, 0.0), " was spiked\n");
        assert_eq!(world_death_string(DeathType::SuperNailgun, 0.0, 0.0), " was spiked\n");
        assert_eq!(
            world_death_string(DeathType::Changelevel, 0.0, 0.0),
            " tried to leave\n"
        );
        assert_eq!(world_death_string(DeathType::Squish, 0.0, 0.0), " was squished\n");
        assert_eq!(
            world_death_string(DeathType::Water, 0.0, 0.1),
            " sleeps with the fishes\n"
        );
        assert_eq!(world_death_string(DeathType::Water, 0.0, 0.9), " sucks it down\n");
        assert_eq!(
            world_death_string(DeathType::Slime, 0.0, 0.1),
            " gulped a load of slime\n"
        );
        assert_eq!(
            world_death_string(DeathType::Slime, 0.0, 0.9),
            " can't exist on slime alone\n"
        );
        assert_eq!(world_death_string(DeathType::Lava, 0.0, 0.1), " turned into hot slag\n");
        assert_eq!(
            world_death_string(DeathType::Lava, 0.0, 0.9),
            " visits the Volcano God\n"
        );
        assert_eq!(world_death_string(DeathType::Lava, -20.0, 0.0), " burst into flames\n");
        assert_eq!(world_death_string(DeathType::TriggerHurt, 0.0, 0.0), " died\n");
    }

    #[test]
    fn telefrag_classification() {
        assert!(DeathType::Telefrag.is_telefrag());
        assert!(DeathType::TelefragDeflected.is_telefrag());
        assert!(DeathType::TelefragMutual.is_telefrag());
        assert!(!DeathType::Rocket.is_telefrag());
        assert!(!DeathType::None.is_telefrag());
    }
}
