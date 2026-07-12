// SPDX-License-Identifier: AGPL-3.0-or-later

//! Deathmatch spawn-point selection: pick a spot (preferring unoccupied ones, with the KTX `k_spw 4`
//! fairness spread), and the pure occupancy/choice rules it rests on — extracted so they unit-test
//! without a live server. Dispatched from `put_client_in_server` (see the sibling `client` modules).

use crate::entity::EntId;
use crate::game::GameState;

impl GameState {
    /// `SelectSpawnPoint` — pick a deathmatch spawn (preferring unoccupied ones), falling
    /// back to the single-player start. `who` is the spawning player (`None` for non-player
    /// placement like CTF runes — no spawn memory, no self-exclusion).
    pub(crate) fn select_spawn_point(&mut self, who: Option<EntId>) -> EntId {
        let spot = self.pick_spawn_of("info_player_deathmatch", who);
        if spot != EntId::WORLD {
            return spot;
        }
        self.find_by_classname("info_player_start")
            .next()
            .unwrap_or(EntId::WORLD)
    }

    /// Pick a spawn point of a specific classname (e.g. `info_teleport_destination` for arena
    /// fighters), preferring an unoccupied one. `EntId::WORLD` if the map has none — the caller
    /// decides the fallback.
    pub(crate) fn select_spawn_point_of(&mut self, classname: &str, who: Option<EntId>) -> EntId {
        self.pick_spawn_of(classname, who)
    }

    /// Shared spawn-point picker — KTX's default spawn model (`k_spw 4` "KTX2"): a random
    /// unoccupied entity of `classname` (any of them if all are occupied — the following
    /// telefrag clears the collision), or `EntId::WORLD` if none exist. Under live rules
    /// ([`GameMode::spawn_rules_live`](crate::mode::GameMode::spawn_rules_live)) a pick that
    /// repeats `who`'s previous spawn is re-rolled once, and the spot is remembered — but only
    /// on maps with more than two spots, where avoidance means something.
    fn pick_spawn_of(&mut self, classname: &str, who: Option<EntId>) -> EntId {
        let spots: Vec<EntId> = self.find_by_classname(classname).collect();
        if spots.is_empty() {
            return EntId::WORLD;
        }
        let mode = self.mode;
        let rules = if mode.spawn_rules_live(self) { SpawnRules::Live } else { SpawnRules::Warmup };
        let blocked: Vec<bool> = spots.iter().map(|&s| self.spot_occupied(s, who, rules)).collect();
        // The previous spawn as an index into this classname's spots; a stale or foreign
        // `last_spot` simply doesn't match and costs nothing.
        let prev = who.and_then(|w| spots.iter().position(|&s| s == self.entities[w].spawn.last_spot));
        let (r1, r2) = (self.random(), self.random());
        let pick = spots[choose_spot(&blocked, prev, rules, r1, r2)];
        if let Some(w) = who {
            if records_last_spot(spots.len(), rules) {
                self.entities[w].spawn.last_spot = pick;
            }
        }
        pick
    }

    /// Whether a live player (other than `who`) fences this spawn spot: within 84 units and
    /// blocking per [`blocks_spot`]. Dead players never block — their corpse is about to
    /// respawn elsewhere, and KTX skips them too.
    fn spot_occupied(&self, spot: EntId, who: Option<EntId>, rules: SpawnRules) -> bool {
        let time = self.time();
        let origin = self.entities[spot].v.origin;
        self.find_by_classname("player").any(|p| {
            if Some(p) == who
                || self.entities[p].v.health <= 0.0
                || (self.entities[p].v.origin - origin).length() >= 84.0
            {
                return false;
            }
            // A bystander a telefrag can't clear must always fence: benched spectators (composition
            // layer) and any mode's untouchable bystanders (the Rocket Arena audience) are solid but
            // damage-refused, so spawning into them would wedge both players.
            let untouchable = crate::mode::team::benched(self, p) || self.mode.untouchable_bystander(self, p);
            blocks_spot(rules, untouchable, self.entities[p].spawn.grace_until, time)
        })
    }

    /// Is any spot of `classname` free of other players for `spawning` to take? Uses the strict
    /// (non-live) occupancy rule — any *other, living* player within 84 units fences a spot —
    /// because this answers "would placing here wedge them", not spawn fairness. The wedge-avoidance
    /// gate behind [`GameMode::spawn_area_clear`] (Rocket Arena, whose pre-round damage gate eats the
    /// spawn telefrag).
    pub(crate) fn has_free_spawn_of(&self, classname: &str, spawning: EntId) -> bool {
        self.find_by_classname(classname)
            .any(|s| !self.spot_occupied(s, Some(spawning), SpawnRules::Warmup))
    }

    // --- small helpers ---

    /// Read the player's `name` userinfo key.
    pub(crate) fn read_netname(&self, player: EntId) -> String {
        let mut buf = [0u8; 64];
        self.host.infokey(player, c"name", &mut buf).to_owned()
    }
}

// --- spawn-selection rules (KTX k_spw 4), extracted pure for unit tests ---

/// Which occupancy regime the spawn picker runs under: live play (an established player only fences
/// a spot while inside its post-spawn grace, so camping doesn't remove it forever) or warmup/strict
/// (any nearby living player fences — used in warmup, and when merely avoiding a wedge).
#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum SpawnRules {
    Live,
    Warmup,
}


/// Does one nearby live non-self player fence a spawn spot? Outside live play always (the stock
/// rule — dead/self/radius are filtered by the caller); during live play only while inside their
/// own post-spawn/teleport grace window, so an established player camping a spot doesn't remove
/// it from the pool forever. Untouchable bystanders (benched spectators, arena audience) always
/// fence — a telefrag can't clear them.
fn blocks_spot(rules: SpawnRules, untouchable: bool, grace_until: f32, time: f32) -> bool {
    rules == SpawnRules::Warmup || untouchable || grace_until >= time
}

/// KTX's `SelectSpawnPoint` over `Sub_SelectSpawnPoint`: a random free spot (any spot when all
/// are blocked), re-rolled once under live rules when the pick repeats `prev` — the previous
/// spawn. The re-roll may land on `prev` again; that's accepted, exactly like KTX. `r1`/`r2`
/// are the two pre-drawn uniform `[0,1)` rolls.
fn choose_spot(blocked: &[bool], prev: Option<usize>, rules: SpawnRules, r1: f32, r2: f32) -> usize {
    let free: Vec<usize> = (0..blocked.len()).filter(|&i| !blocked[i]).collect();
    let roll = |r: f32| -> usize {
        if free.is_empty() {
            let i = (r * blocked.len() as f32) as usize;
            i.min(blocked.len() - 1)
        } else {
            let i = (r * free.len() as f32) as usize;
            free[i.min(free.len() - 1)]
        }
    };
    let first = roll(r1);
    if rules == SpawnRules::Live && Some(first) == prev {
        return roll(r2);
    }
    first
}

/// KTX records the previous spawn only on maps with more than two spots (with two, avoidance
/// would just ping-pong deterministically), and only under live rules.
fn records_last_spot(total: usize, rules: SpawnRules) -> bool {
    rules == SpawnRules::Live && total > 2
}

#[cfg(test)]
mod tests {
    use super::{blocks_spot, choose_spot, records_last_spot, SpawnRules};

    #[test]
    fn warmup_blocks_regardless_of_grace() {
        // Outside live rules any nearby live player fences, grace long expired or not.
        assert!(blocks_spot(SpawnRules::Warmup, false, 0.0, 100.0));
        assert!(blocks_spot(SpawnRules::Warmup, false, 200.0, 100.0));
    }

    #[test]
    fn live_grace_gates_blocking() {
        assert!(blocks_spot(SpawnRules::Live, false, 100.0, 100.0)); // inside grace (inclusive)
        assert!(!blocks_spot(SpawnRules::Live, false, 99.9, 100.0)); // grace lapsed — no longer fences
    }

    #[test]
    fn untouchable_always_fences() {
        assert!(blocks_spot(SpawnRules::Live, true, 0.0, 100.0));
    }

    #[test]
    fn prefers_free_spots() {
        let blocked = [true, false, true, false];
        for r in [0.0, 0.3, 0.6, 0.99] {
            let pick = choose_spot(&blocked, None, SpawnRules::Live, r, 0.0);
            assert!(!blocked[pick]);
        }
    }

    #[test]
    fn all_blocked_falls_back_to_any() {
        let blocked = [true, true, true];
        assert_eq!(choose_spot(&blocked, None, SpawnRules::Live, 0.0, 0.0), 0);
        assert_eq!(choose_spot(&blocked, None, SpawnRules::Live, 0.5, 0.0), 1);
        assert_eq!(choose_spot(&blocked, None, SpawnRules::Live, 0.99, 0.0), 2);
    }

    #[test]
    fn reroll_once_on_repeat() {
        let blocked = [false, false, false, false];
        // r1 lands on the previous spawn (index 1) → the second roll decides (index 3).
        assert_eq!(choose_spot(&blocked, Some(1), SpawnRules::Live, 0.3, 0.9), 3);
        // The re-roll landing on prev again is accepted — no third roll.
        assert_eq!(choose_spot(&blocked, Some(1), SpawnRules::Live, 0.3, 0.3), 1);
    }

    #[test]
    fn no_reroll_when_not_live() {
        let blocked = [false, false, false, false];
        assert_eq!(choose_spot(&blocked, Some(1), SpawnRules::Warmup, 0.3, 0.9), 1);
    }

    #[test]
    fn no_reroll_without_memory() {
        // The `who: None` paths (CTF runes, bot roam points) carry no previous spawn.
        let blocked = [false, false, false, false];
        assert_eq!(choose_spot(&blocked, None, SpawnRules::Live, 0.3, 0.9), 1);
    }

    #[test]
    fn last_spot_recorded_only_when_meaningful() {
        assert!(!records_last_spot(2, SpawnRules::Live)); // two spots would ping-pong
        assert!(records_last_spot(3, SpawnRules::Live));
        assert!(!records_last_spot(5, SpawnRules::Warmup)); // warmup never records
    }
}
