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
use crate::game::{cstring, GameState};

mod arena;
mod ctf;
mod ffa;
mod midair;
mod team;

pub(crate) use arena::{Arena, ArenaState};
pub(crate) use ctf::Ctf;
pub(crate) use ffa::Ffa;
pub(crate) use midair::Midair;
pub(crate) use team::{is_match_mode, MatchState, TeamMatch};

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
    /// Throttle for the "can't fire yet" screen blink during the countdown (world time of the last
    /// blink), so a held fire button flashes once in a while rather than every frame.
    pub flash_time: f32,
    /// Team id in a team match (`1..=N`; `0` = unassigned/none). Only the team modes read it. See
    /// [`crate::mode::team`].
    pub team: u8,
    /// In CTF, the team id of the enemy flag this player is carrying (`0` = not carrying). See
    /// [`crate::mode::ctf`].
    pub carrying: u8,
    /// CTF held-rune bitfield (`crate::defs::RUNE_*`; `0` = none). One rune per player.
    pub runes: u8,
    // --- CTF assist/defense bookkeeping (world times; `0` = never) ---
    /// When this player grabbed the enemy flag (a short grace before a carrier-frag scores).
    pub flag_since: f32,
    /// When this player last returned their own flag (a return→capture assist window).
    pub last_returned_flag: f32,
    /// When this player last fragged an enemy flag carrier (a frag→capture assist window).
    pub last_fragged_carrier: f32,
    /// When this player last damaged an enemy flag carrier (a carrier-defense window).
    pub last_hurt_carrier: f32,
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

/// How a mode shapes one hit's effect on a player (see [`GameMode::player_damage`]). `health` is
/// the damage sent into the normal armor/health path (a large value one-shots through armor);
/// `knockback` is the impulse basis the engine turns into velocity. They're separate so a mode can
/// deal zero health damage yet still fling the target — Midair launches grounded players this way.
pub(crate) struct DamageOutcome {
    pub health: f32,
    pub knockback: f32,
}

impl DamageOutcome {
    /// Take the hit unchanged (health and knockback both the incoming damage) — the stock rule.
    pub fn pass(incoming: f32) -> Self {
        Self {
            health: incoming,
            knockback: incoming,
        }
    }

    /// No effect at all — no health loss and no knockback (spawn protection, untouchable audience).
    pub fn none() -> Self {
        Self {
            health: 0.0,
            knockback: 0.0,
        }
    }
}

/// The behavior of one game mode. Every hook has a default that reproduces stock FFA deathmatch,
/// so a mode only overrides the policy pieces it changes:
///
/// - **Ruleset** — [`tick`](GameMode::tick), [`player_damage`](GameMode::player_damage),
///   [`weapons_hot`](GameMode::weapons_hot), [`on_death`](GameMode::on_death),
///   [`announce_death`](GameMode::announce_death), [`allow_respawn`](GameMode::allow_respawn).
/// - **Spawns** — [`select_spawn`](GameMode::select_spawn).
/// - **Loadout** — [`apply_loadout`](GameMode::apply_loadout).
/// - **Bots** — [`bot_intent`](GameMode::bot_intent).
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

    /// Shape one hit landing on `targ` — the mode's damage ruleset, consulted in `t_damage` after
    /// the quad multiplier and before armor. Returns the health damage to apply and the knockback
    /// impulse basis (see [`DamageOutcome`]); [`DamageOutcome::none`] blocks the hit entirely (no
    /// pain, no knockback), reproducing a hard damage gate. `incoming` is the post-quad damage.
    /// Default: pass through unchanged. Used for Rocket Arena countdown protection / untouchable
    /// audience, and for Midair's airborne-only kills + launch knockback.
    fn player_damage(
        &self,
        _g: &mut GameState,
        _targ: EntId,
        _attacker: EntId,
        _inflictor: EntId,
        incoming: f32,
    ) -> DamageOutcome {
        DamageOutcome::pass(incoming)
    }

    /// May weapons fire right now? Gates the actual shot (muzzle/projectile), not just damage —
    /// used to lock out firing before "FIGHT" in a round mode. Default: yes.
    fn weapons_hot(&self, _g: &GameState) -> bool {
        true
    }

    /// Print the mode's own kill announcement / scoring instead of the default obituary. Called in
    /// `killed` just before `client_obituary`; return `true` to suppress the default (the mode has
    /// handled the frag + broadcast itself). Default: `false` (use the stock obituary). Midair uses
    /// this for its height-tiered airshot scoring.
    fn announce_death(&self, _g: &mut GameState, _victim: EntId, _attacker: EntId) -> bool {
        false
    }

    /// Called after a player dies (obituary already printed / suppressed): record eliminations,
    /// check for a round end. Default: nothing.
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

    /// The active mode's own map-(re)load hook, run by [`on_worldspawn`] after the shared Arena /
    /// team-match reset. Default: nothing.
    fn on_worldspawn(&self, _g: &mut GameState) {}

    /// A client is leaving (a human disconnecting, or a bot trimmed by the population manager),
    /// called before the slot is retired. Used by CTF to drop a carried flag. Default: nothing.
    fn on_client_disconnect(&self, _g: &mut GameState, _e: EntId) {}

    /// Early in each live player's `PlayerPreThink`, before the death/dying checks. Used by CTF for
    /// the Regeneration rune's periodic heal. Default: nothing.
    fn player_prethink(&self, _g: &mut GameState, _e: EntId) {}

    /// The player's body is dying (`player_die`, after the backpack drop and before the corpse is
    /// tossed). Used by CTF to drop the carried flag and any held runes. Default: nothing.
    fn player_died(&self, _g: &mut GameState, _e: EntId) {}

    /// Offer a pending player impulse to the mode; return `true` if the mode consumed it (skipping
    /// the stock impulse table). Used by CTF for the flag/rune toss (impulses 24 / 26). Default:
    /// `false`.
    fn handle_impulse(&self, _g: &mut GameState, _e: EntId, _impulse: i32) -> bool {
        false
    }

    /// Offer a client console command to the mode; return `true` if the mode consumed it. Used by
    /// the match modes for `start`. Default: `false`.
    fn handle_command(&self, _g: &mut GameState, _e: EntId, _cmd: &str) -> bool {
        false
    }

    /// Multiplier applied to a freshly-set attack cooldown (`attack_finished`), letting a mode speed
    /// up or slow down firing. CTF's Haste rune returns `0.5` (fire ~2× as fast). Default: `1.0`.
    fn attack_cooldown_scale(&self, _g: &GameState, _e: EntId) -> f32 {
        1.0
    }
}

/// The FFA singleton — the baseline mode and the default when `rtx_mode` is unset/unknown.
static FFA: Ffa = Ffa;
/// The Rocket Arena singleton (`rtx_mode ra`).
static ARENA: Arena = Arena;
/// The Midair singleton (`rtx_mode midair`).
static MIDAIR: Midair = Midair;
/// The team-match singleton — every team alias (`1on1`/`duel`/`2on2`/`2on2on2`/`NonM…`) resolves to
/// it; the parsed format lives in [`GameState::team_match`], not the descriptor.
static TEAM: TeamMatch = TeamMatch;
/// The Capture-the-Flag singleton (`rtx_mode ctf`) — reuses the match lifecycle + team layer.
static CTF: Ctf = Ctf;

/// The default mode used before a map selects one (matches `rtx_mode ffa`).
pub(crate) fn default_mode() -> &'static dyn GameMode {
    &FFA
}

/// Resolve an `rtx_mode` string to its mode descriptor. Any team format alias resolves to the
/// shared team-match descriptor; unknown values fall back to FFA.
pub(crate) fn select_mode(name: &str) -> &'static dyn GameMode {
    match name {
        "ra" => &ARENA,
        "midair" => &MIDAIR,
        "ctf" => &CTF,
        _ if team::parse_match_alias(name).is_some() => &TEAM,
        _ => &FFA,
    }
}

impl GameState {
    /// Sync the active mode to the `rtx_mode` cvar. Read live every frame (like every other rtx
    /// cvar) so a mode change takes effect without a map reload. Re-selecting only on an actual
    /// change keeps round/match state from resetting every frame; switching modes deliberately does
    /// reset it (a fresh match). `rtx_mode` is preserved across a map reload (`cvar_default` only
    /// seeds an unset cvar), so the team-match start-reload keeps its alias.
    pub(crate) fn refresh_mode(&mut self) {
        let host = self.host;
        let mut buf = [0u8; 16];
        let name = host.cvar_string(c"rtx_mode", &mut buf);
        let next = select_mode(name);
        // The team descriptor's `name()` is a constant, so switching format (`2on2`→`3on3`) isn't a
        // descriptor change — track the parsed format separately (below).
        let team_config = team::parse_match_alias(name);
        if next.name() != self.mode.name() {
            self.mode = next;
            self.arena = ArenaState::default();
            host.conprint(&cstring(&format!("rtx: game mode = {}\n", next.name())));
        }
        // `name`'s borrow of `buf` is done (config is Copy); free to take `&mut self`. Both match
        // modes carry their format in `MatchState` (the descriptor `name()` is constant): team DM
        // parses the alias; CTF is a fixed 2-team format. A changed format is a fresh match.
        if is_match_mode(next.name()) {
            let cfg = if next.name() == "ctf" {
                team::MatchConfig { teams: 2, size: 0 }
            } else {
                team_config.unwrap_or_default()
            };
            if cfg != self.team_match.config {
                self.team_match = MatchState {
                    config: cfg,
                    ..Default::default()
                };
            }
            // Team play needs friendly-fire protection on (the `"team"` userinfo does the rest).
            if host.cvar(c"teamplay") == 0.0 {
                host.cvar_set(c"teamplay", c"1");
            }
        }
    }
}

/// Map (re)load housekeeping for the mode layer, called from `worldspawn` after `refresh_mode`.
/// Resets the per-map round state (Arena) and reconciles the team-match state — which must run for
/// *every* mode, since switching away from a match mode has to clear it (so it can't hang off the
/// active mode's own hook). Then the active mode's own [`GameMode::on_worldspawn`] runs.
pub(crate) fn on_worldspawn(g: &mut GameState) {
    g.arena = ArenaState::default();
    team::on_worldspawn(g);
    let mode = g.mode;
    mode.on_worldspawn(g);
}

// --- shared player-roster helpers (used by every mode) -----------------------------------------

/// Every connected player edict (humans and bots occupy client slots `1..=maxclients`).
pub(crate) fn players(g: &GameState) -> Vec<EntId> {
    let maxclients = g.host().cvar(c"maxclients") as i32;
    (1..=maxclients as u32)
        .map(EntId)
        .filter(|&e| g.entities[e].in_use && g.entities[e].classname() == Some("player"))
        .collect()
}

/// The nearest *living* player to `point` that satisfies `pred`, skipping `skip` (pass
/// [`EntId::WORLD`] to skip nobody). The one min-by-distance loop behind every mode's enemy picker:
/// FFA/Midair pass `|_, _| true` (everyone's an enemy), Arena filters to live fighters, team modes
/// filter by opposing team.
pub(crate) fn nearest_player_where(
    g: &GameState,
    point: Vec3,
    skip: EntId,
    pred: impl Fn(&GameState, EntId) -> bool,
) -> Option<EntId> {
    let mut best: Option<(EntId, f32)> = None;
    for e in players(g) {
        if e == skip {
            continue;
        }
        let ent = &g.entities[e];
        if ent.v.health <= 0.0 || ent.v.deadflag != 0.0 || !pred(g, e) {
            continue;
        }
        let d = (ent.v.origin - point).length_squared();
        if best.is_none_or(|(_, bd)| d < bd) {
            best = Some((e, d));
        }
    }
    best.map(|(e, _)| e)
}

/// Center-print to every connected human (bots are fake clients with no connection — a unicast to
/// one makes the engine warn "msg_entity: not a client").
pub(crate) fn centerprint_all(g: &GameState, msg: &str) {
    let host = *g.host();
    let cmsg = cstring(msg);
    for e in players(g) {
        if g.entities[e].bot.is_bot {
            continue;
        }
        host.centerprint(e, &cmsg);
    }
}
