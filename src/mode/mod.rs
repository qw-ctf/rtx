// SPDX-License-Identifier: AGPL-3.0-or-later

//! Pluggable game modes.
//!
//! rtx started life as a single implicit mode: free-for-all deathmatch. To make room for the
//! modes coming next (1on1/2on2/4on4, timed matches, CTF, instagib) without scattering
//! `if arena { … }` branches across the codebase — the `#ifdef ARENA` sprawl the reference
//! Frogbot-Rocket-Arena QuakeC suffers from — the mode is factored behind one small trait,
//! [`GameMode`], whose hooks the generic lifecycle code (spawn, death, respawn, damage,
//! per-frame tick, bot think) calls at a handful of well-defined seams.
//!
//! A mode is a **stateless behavior descriptor** (a zero-sized struct) exposed as
//! `&'static dyn GameMode`; all mutable per-match state lives in [`GameState`]. Because a
//! `&'static dyn` is `Copy`, a hook site copies the descriptor out of `self` first and is then
//! free to take `&mut GameState`:
//!
//! ```ignore
//! let mode = self.mode;   // copies the fat pointer — no borrow of self
//! mode.tick(self);        // now free to take &mut self
//! ```
//!
//! [`Ffa`] is the baseline (every hook is the current default), so today's gameplay is
//! unchanged; [`Arena`] (`rtx_mode ra`) is the first mode layered on top and overrides only the
//! hooks it needs. Adding a mode = one struct + `impl GameMode` + a line in [`select_mode`].

use glam::Vec3;

use crate::entity::EntId;
use crate::game::GameState;

mod arena;
mod ffa;

pub(crate) use arena::{Arena, ArenaState};
pub(crate) use ffa::Ffa;

/// A player's standing in a round-based mode (Rocket Arena): fighting in the arena, or waiting
/// in the audience (fresh joiners, and players eliminated until the next round). Stored per
/// player on the entity; irrelevant (always `Audience`) in modes that don't use it.
#[derive(Default, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ArenaRole {
    #[default]
    Audience,
    Fighter,
}

/// Per-player mode state carried on every [`crate::entity::Entity`]. Small and mode-agnostic;
/// only the round-based modes read it.
#[derive(Default)]
pub(crate) struct ArenaPlayer {
    pub role: ArenaRole,
    pub round_wins: i32,
    /// Audience queue position (a monotonic stamp; lower = waited longer). `0` means unqueued —
    /// either a current fighter, or an audience member not yet stamped. See the arena's round
    /// former, which pulls the lowest-stamped audience member in as the next challenger.
    pub queue: u32,
}

/// A mode's per-frame directive for one bot — the *only* channel through which a mode influences
/// bot behavior. The generic bot brain ([`crate::bot::run_bot`]) navigates and, for [`Fight`], runs
/// the shared combat layer ([`crate::bot_combat`]); the mode just says *what* to pursue. This keeps
/// every mode-specific bot adaptation behind [`GameMode::bot_intent`] rather than in `bot.rs`.
///
/// [`Fight`]: BotIntent::Fight
#[derive(Clone, Copy)]
pub(crate) enum BotIntent {
    /// Navigate to and fight this enemy (the combat overlay engages once there's line of sight).
    Fight(EntId),
    /// Roam toward this world position without fighting (e.g. an arena audience member).
    Move(Vec3),
}

/// The behavior of one game mode. Every hook has a default that reproduces stock FFA deathmatch,
/// so a mode only overrides the policy pieces it changes:
///
/// - **Ruleset** — [`tick`](GameMode::tick), [`damage_allowed`](GameMode::damage_allowed),
///   [`on_death`](GameMode::on_death), [`allow_respawn`](GameMode::allow_respawn).
/// - **Spawns** — [`select_spawn`](GameMode::select_spawn).
/// - **Loadout** — [`apply_loadout`](GameMode::apply_loadout).
/// - **Bots** — [`bot_enemy`](GameMode::bot_enemy), [`bot_audience`](GameMode::bot_audience).
pub(crate) trait GameMode: Sync {
    /// The `rtx_mode` value that selects this mode.
    fn name(&self) -> &'static str;

    /// Per-frame state machine (round countdown / fight / reset). Runs once per normal server
    /// frame. Default: nothing.
    fn tick(&self, _g: &mut GameState) {}

    /// Choose the spawn point entity for (re)spawning player `e`. Default: a standard free
    /// `info_player_deathmatch` (the stock rule).
    fn select_spawn(&self, g: &mut GameState, _e: EntId) -> EntId {
        g.select_spawn_point()
    }

    /// Set weapons / ammo / armor / health after `DecodeLevelParms`. Default: keep the decoded
    /// spawn parms (stock FFA loadout).
    fn apply_loadout(&self, _g: &mut GameState, _e: EntId) {}

    /// May `targ` take damage right now? Used for round countdown spawn-protection and to keep
    /// audience players harmless. Default: yes.
    fn damage_allowed(&self, _g: &GameState, _targ: EntId) -> bool {
        true
    }

    /// Called after a player dies (obituary already printed): record eliminations, check for a
    /// round end. Default: nothing.
    fn on_death(&self, _g: &mut GameState, _victim: EntId, _attacker: EntId) {}

    /// May a dead player respawn now via the death-think button press? A mode that drives
    /// respawns itself can return false. Default: yes (input-driven, as stock).
    fn allow_respawn(&self, _g: &GameState, _e: EntId) -> bool {
        true
    }

    /// The mode-specific directive for one bot this frame — the sole seam through which a mode
    /// adapts bot behavior. `Some` overrides the generic item/human brain (fight an enemy, or roam
    /// to a spot); `None` leaves the default behavior in charge. Default: `None`.
    fn bot_intent(&self, _g: &mut GameState, _bot: EntId) -> Option<BotIntent> {
        None
    }
}

/// The FFA singleton — the baseline mode and the default when `rtx_mode` is unset/unknown.
static FFA: Ffa = Ffa;
/// The Rocket Arena singleton (`rtx_mode ra`).
static ARENA: Arena = Arena;

/// The default mode used before a map selects one (matches `rtx_mode ffa`).
pub(crate) fn default_mode() -> &'static dyn GameMode {
    &FFA
}

/// Resolve an `rtx_mode` string to its mode descriptor. Unknown values fall back to FFA.
pub(crate) fn select_mode(name: &str) -> &'static dyn GameMode {
    match name {
        "ra" => &ARENA,
        _ => &FFA,
    }
}

impl GameState {
    /// Sync the active mode to the `rtx_mode` cvar. Read live every frame (like every other rtx
    /// cvar) so `rtx_mode ra` takes effect without a map reload — and because `init` re-defaults
    /// the cvar to `ffa` on each map load right before the first `worldspawn`, a one-shot read
    /// there would always see `ffa`. Re-selecting only on an actual change keeps round state from
    /// resetting every frame; switching modes deliberately does reset it (a fresh match).
    pub(crate) fn refresh_mode(&mut self) {
        let host = self.host;
        let mut buf = [0u8; 16];
        let name = host.cvar_string(c"rtx_mode", &mut buf);
        let next = select_mode(name);
        if next.name() != self.mode.name() {
            self.mode = next;
            self.arena = ArenaState::default();
            host.conprint(&crate::game::cstring(&format!(
                "rtx: game mode = {}\n",
                next.name()
            )));
        }
    }
}
