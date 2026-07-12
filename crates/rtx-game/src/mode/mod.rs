// SPDX-License-Identifier: AGPL-3.0-or-later

//! Pluggable game modes, on two orthogonal axes.
//!
//! Gameplay splits into a **game mode** (the ruleset — `rtx_mode`) and a **match composition** (how
//! the match is organized — `rtx_match`). They're independent: deathmatch and CTF each run under any
//! composition (open free-for-all, or a locked `2on2`/`4on4`/…), while Rocket Arena and midair are
//! inherently duel rulesets. Keeping them separate avoids the `#ifdef ARENA` sprawl the reference
//! Frogbot-Rocket-Arena QuakeC suffers from.
//!
//! - **Game mode** is factored behind one small trait, [`GameMode`], whose hooks the generic
//!   lifecycle code (spawn, death, respawn, damage, per-frame tick, bot think) calls at a handful of
//!   well-defined seams. There are four: [`Dm`] (the baseline — every hook is the default, so plain
//!   deathmatch plays as before), [`Arena`] (`ra`), [`Midair`], and [`Ctf`], each overriding only
//!   the hooks it changes. Adding one = a struct + `impl GameMode` + a line in [`select_mode`].
//! - **Match composition** lives in [`team`]: `rtx_match` resolves to a `MatchConfig` of N teams of
//!   size M, which drives (for `teams ≥ 2`) the shared warmup→start→countdown→live match lifecycle
//!   and the team layer (assignment, colours, team-aware bot targeting). A structured match's three
//!   variation points — go-live slate, win condition, result line — are the `on_match_*` hooks,
//!   whose defaults are team deathmatch, so [`Dm`]/[`Midair`] get structured play for free and
//!   [`Ctf`] overrides them.
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

use glam::Vec3;

use crate::defs::{Items, Weapon};
use crate::entity::EntId;
use crate::game::{cstring, GameState};

mod arena;
mod ctf;
mod dm;
mod midair;
mod race;
pub(crate) mod team;

pub(crate) use arena::{Arena, ArenaState};
pub(crate) use ctf::Ctf;
pub(crate) use dm::Dm;
pub(crate) use midair::Midair;
pub(crate) use race::{Race, RaceSlot};
pub(crate) use team::{MatchPhase, MatchState};

/// A player's standing in a round-based mode (Rocket Arena): fighting in the arena, or waiting
/// in the audience (fresh joiners, and players eliminated until the next round). Stored per
/// player on the entity; irrelevant (always `Audience`) in modes that don't use it.
#[derive(Default, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ArenaRole {
    #[default]
    Audience,
    Fighter,
}

/// Per-player mode state carried on every [`crate::entity::Entity`], grouped by the mode family
/// that reads it so the arena / team / CTF concerns don't pile into one flat struct. Small and
/// mode-agnostic overall; each nested group is left at its default in modes that don't use it.
#[derive(Default)]
pub(crate) struct ModePlayer {
    /// Rocket-Arena round standing.
    pub arena: ArenaSlot,
    /// Team id in a team match (`1..=N`; `0` = unassigned/none). Only the team modes read it. See
    /// [`crate::mode::team`].
    pub team: u8,
    /// CTF carry / rune / assist bookkeeping.
    pub ctf: CtfPlayer,
    /// Race run progress (next node / clock / best time). See [`crate::mode::race`].
    pub race: RaceSlot,
}

/// A player's Rocket-Arena round standing: fighting in the arena, or waiting in the audience.
#[derive(Default)]
pub(crate) struct ArenaSlot {
    pub role: ArenaRole,
    pub round_wins: i32,
    /// Audience queue position (a monotonic stamp; lower = waited longer). `0` means unqueued —
    /// either a current fighter, or an audience member not yet stamped. See the arena's round
    /// former, which pulls the lowest-stamped audience member in as the next challenger.
    pub queue: u32,
    /// Throttle for the "can't fire yet" screen blink during the countdown (world time of the last
    /// blink), so a held fire button flashes once in a while rather than every frame.
    pub flash_time: f32,
    /// Set when this player has been promoted to fighter for the forming round but not yet placed
    /// in the arena, because the spawn area wasn't clear (see [`GameMode::spawn_area_clear`]). The
    /// round former / tick loop keeps retrying the placement; any spawn clears it (in
    /// `select_spawn`). Per-player rather than in `ArenaState` so `retire_slot` drops it with the
    /// slot — it can never dangle across slot reuse.
    pub pending_spawn: bool,
}

/// A player's CTF state: the flag they carry, their held rune, and the assist/defense windows.
/// (A rune *pickup* entity keeps its own bit in [`crate::entity::ItemState::rune_bit`], not here.)
#[derive(Default)]
pub(crate) struct CtfPlayer {
    /// The team id of the enemy flag this player is carrying (`0` = not carrying). See
    /// [`crate::mode::ctf`].
    pub carrying: u8,
    /// CTF held-rune bitfield (`crate::defs::RUNE_*`; `0` = none). One rune per player.
    pub runes: u8,
    // --- assist/defense bookkeeping (world times; `0` = never) ---
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
/// the shared combat layer ([`crate::bot::combat`]); the mode just says *what* to pursue. This keeps
/// every mode-specific bot adaptation behind [`GameMode::bot_intent`] rather than in `bot.rs`.
///
/// [`Fight`]: BotIntent::Fight
#[derive(Clone, Copy)]
pub(crate) enum BotIntent {
    /// Navigate to and fight this enemy (the combat overlay engages once there's line of sight).
    Fight(EntId),
    /// Roam toward this world position without fighting (e.g. an arena audience member).
    Move(Vec3),
    /// Roam toward `goal` while keeping the eyes on `watch` — an arena audience member spectating
    /// the live fighters. Navigation is exactly [`Move`](BotIntent::Move)`(goal)`; only the look is
    /// redirected (a post-hoc override in `run_bot`), so movement and bunnyhop steering are untouched.
    Spectate { goal: Vec3, watch: EntId },
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

/// The behavior of one game mode. Every hook has a default that reproduces stock deathmatch, so a
/// mode only overrides the policy pieces it changes:
///
/// - **Ruleset** — [`tick`](GameMode::tick), [`player_damage`](GameMode::player_damage),
///   [`weapons_hot`](GameMode::weapons_hot), [`on_death`](GameMode::on_death),
///   [`announce_death`](GameMode::announce_death), [`allow_respawn`](GameMode::allow_respawn).
/// - **Spawns** — [`select_spawn`](GameMode::select_spawn).
/// - **Loadout** — [`apply_loadout`](GameMode::apply_loadout).
/// - **Bots** — [`bot_intent`](GameMode::bot_intent).
/// - **Structured match** — [`on_match_go_live`](GameMode::on_match_go_live),
///   [`match_limit_reached`](GameMode::match_limit_reached),
///   [`announce_match_result`](GameMode::announce_match_result). Defaults are team deathmatch.
pub(crate) trait GameMode: Sync {
    /// The `rtx_mode` value that selects this mode.
    fn name(&self) -> &'static str;

    /// Per-frame state machine (round countdown / fight / reset). Runs once per normal server
    /// frame. Default: nothing. (The team-match lifecycle is driven separately, off `rtx_match`.)
    fn tick(&self, _g: &mut GameState) {}

    /// Choose the spawn point entity for (re)spawning player `e`. Default: this player's team spawn
    /// (`info_player_teamN`) in a team composition, falling back to a standard free
    /// `info_player_deathmatch` — which is exactly the stock rule when the player has no team.
    fn select_spawn(&self, g: &mut GameState, e: EntId) -> EntId {
        team::team_spawn(g, e)
    }

    /// Do the live spawn-fairness rules apply right now — KTX's `match_in_progress == 2`, i.e.
    /// "actually playing"? Under live rules a bystander fences nearby spawn spots only during
    /// their own post-spawn grace window, and a re-roll avoids back-to-back same-spot respawns;
    /// outside them any nearby live player blocks (the stock rule). Default: a Live team match,
    /// or any composition with no lifecycle at all (plain FFA ≈ a KTX matchless server). Arena
    /// overrides this with its round state.
    fn spawn_rules_live(&self, g: &GameState) -> bool {
        !team::lifecycle_active(g) || matches!(g.team_match.phase, MatchPhase::Live)
    }

    /// Is `e`'s spawn area clear enough to place them without wedging into another player? A game-
    /// driven spawn (round formation, a bot's first spawn, a death-think respawn) consults this and
    /// postpones the placement while it's false, retrying rather than stacking two players on one
    /// spot. Default: always clear (stock behavior — the spawn telefrag resolves any overlap).
    /// Rocket Arena overrides it, because its pre-round damage gate swallows that telefrag and the
    /// two players would interpenetrate permanently. Not consulted on the engine-driven human
    /// spawn path (connect / map change), which can't be deferred.
    fn spawn_area_clear(&self, _g: &GameState, _e: EntId) -> bool {
        true
    }

    /// Final say on a spawn origin, once the spot is chosen and offset — the last funnel before the
    /// player is relinked, so it covers every spawn path (engine-driven, round former, respawn).
    /// Default: unchanged. Rocket Arena nudges a player off any other live player here, so two
    /// fighters can never share a position (their pre-round protection nullifies the telefrag that
    /// would otherwise unstack them) — this is what lets even a single-spawn arena run.
    fn adjust_spawn_origin(&self, _g: &mut GameState, _e: EntId, origin: Vec3) -> Vec3 {
        origin
    }

    /// Set weapons / ammo / armor / health after `DecodeLevelParms`. Default: keep the decoded
    /// spawn parms (stock deathmatch loadout). Team assignment is applied generically before this,
    /// so a mode's loadout composes with any team composition.
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
    /// used to lock out firing before "FIGHT". Default: hot except during a team-match countdown
    /// (in open play the phase never leaves warmup, so this is always hot). Arena overrides it.
    fn weapons_hot(&self, g: &GameState) -> bool {
        team::match_weapons_hot(g)
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

    /// Multiplier applied to a freshly-set attack cooldown (`attack_finished`), letting a mode speed
    /// up or slow down firing. CTF's Haste rune returns `0.5` (fire ~2× as fast). Default: `1.0`.
    fn attack_cooldown_scale(&self, _g: &GameState, _e: EntId) -> f32 {
        1.0
    }

    /// Whether this mode restricts movement to stock QW plus bunnyhop: no double jump, no wall
    /// jump, no elevator jump, no grapple, no rocket jumps — neither as live mechanics
    /// (`check_jump` and friends consult this) nor as navmesh links (`ensure_navmesh` skips
    /// generating them, so bots can't plan them and a failed pathfind is honest). Race uses it:
    /// its maps are authored for exactly that movement set. Default: everything stays on.
    fn stock_movement_only(&self) -> bool {
        false
    }

    /// Whether `e` is a bystander a spawn telefrag cannot clear (solid but damage-refused), so it must
    /// always fence spawn spots regardless of the live/warmup rule. Default: no one. Arena returns its
    /// audience members. (The team-bench half of the spawn-fence check stays at the call site —
    /// benching is the composition layer, not the mode.)
    fn untouchable_bystander(&self, _g: &GameState, _e: EntId) -> bool {
        false
    }

    /// Whether this mode fields CTF furniture (flags and runes), gating the map-entity spawns that
    /// other modes drop. Default: no; CTF overrides.
    fn uses_ctf_objects(&self) -> bool {
        false
    }

    // --- structured-match variation points (only reached when a team match is active) ---

    /// Reset the mode's slate when a match countdown expires: frags/scores for team DM, plus flags
    /// and runes for CTF. The shared machine then arms the timelimit and flips to Live, so this must
    /// *not* touch either. Default: team deathmatch (see [`team::default_go_live`]).
    fn on_match_go_live(&self, g: &mut GameState) {
        team::default_go_live(g);
    }

    /// One Live frame: refresh scores if the mode tallies them here, and report whether the win
    /// limit (frags / captures) is reached — the shared machine ends the match on the timelimit
    /// itself. Default: team deathmatch fraglimit (see [`team::frag_limit_reached`]).
    fn match_limit_reached(&self, g: &mut GameState) -> bool {
        team::frag_limit_reached(g)
    }

    /// Broadcast the result line as a match ends (the phase transition is the machine's job).
    /// Default: the team deathmatch scoreline (see [`team::announce_team_result`]).
    fn announce_match_result(&self, g: &mut GameState) {
        team::announce_team_result(g);
    }
}

/// The deathmatch singleton — the baseline mode and the default when `rtx_mode` is unset/unknown.
static DM: Dm = Dm;
/// The Rocket Arena singleton (`rtx_mode ra`).
static ARENA: Arena = Arena;
/// The Midair singleton (`rtx_mode midair`).
static MIDAIR: Midair = Midair;
/// The Capture-the-Flag singleton (`rtx_mode ctf`) — reuses the match lifecycle + team layer.
static CTF: Ctf = Ctf;
/// The Race singleton (`rtx_mode race`) — timed KTX race routes; also the bot bhop harness.
static RACE: Race = Race;

/// The default mode used before a map selects one (matches `rtx_mode dm`).
pub(crate) fn default_mode() -> &'static dyn GameMode {
    &DM
}

/// Resolve an `rtx_mode` string to its ruleset descriptor. Unknown values (including the old team
/// aliases, now moved to `rtx_match`) fall back to deathmatch; `refresh_mode` hints about it once.
pub(crate) fn select_mode(name: &str) -> &'static dyn GameMode {
    match name {
        "ra" => &ARENA,
        "midair" => &MIDAIR,
        "ctf" => &CTF,
        "race" => &RACE,
        _ => &DM,
    }
}

impl GameState {
    /// Sync the active mode + composition to the `rtx_mode` / `rtx_match` cvars. Read live every
    /// frame (like every other rtx cvar) so a change takes effect without a map reload. State is
    /// reset only on an actual change — switching the *ruleset* abandons a running match; changing
    /// the *format* starts a fresh warmup — so an in-progress match isn't wiped every frame. Both
    /// cvars survive a map reload (`cvar_default` only seeds an unset cvar), so the match-start
    /// reload (where neither changes) preserves the locked roster and `resuming` flag.
    pub(crate) fn refresh_mode(&mut self) {
        let host = self.host;
        let mut mbuf = [0u8; 16];
        let mut fbuf = [0u8; 32]; // room for a long `NonMon…` chain
        let mode_name = host.cvar_string(c"rtx_mode", &mut mbuf);
        let match_alias = host.cvar_string(c"rtx_match", &mut fbuf);
        let next = select_mode(mode_name);
        let cfg = team::resolve_composition(next.name(), match_alias);

        // One-shot hints, fired when either *raw* string changes — an unknown/renamed value may
        // resolve to the same descriptor/config, so detect the raw change, not just the resolved one.
        // (These read/write self fields but not the cvar buffers, so no borrow conflict.)
        if self.mode_cvar != mode_name {
            self.mode_cvar = mode_name.to_string();
            if !matches!(mode_name, "dm" | "ra" | "midair" | "ctf" | "race") {
                host.conprint(&cstring(&format!(
                    "rtx: unknown rtx_mode \"{mode_name}\" — modes are dm|ra|midair|ctf|race; team formats like 2on2 are now rtx_match. Using dm.\n"
                )));
            }
        }
        if self.match_cvar != match_alias {
            self.match_cvar = match_alias.to_string();
            if next.name() == "ra" && !match_alias.is_empty() {
                host.conprint(&cstring(
                    "rtx: rtx_match is ignored in Rocket Arena — its 1v1 round queue is its composition.\n",
                ));
            } else if next.name() == "race" && !match_alias.is_empty() {
                host.conprint(&cstring(
                    "rtx: rtx_match is ignored in race — runs are timed per player, no teams.\n",
                ));
            } else if !matches!(match_alias, "" | "ffa") && team::parse_match_alias(match_alias).is_none() {
                host.conprint(&cstring(&format!(
                    "rtx: unknown rtx_match \"{match_alias}\" — try ffa or a format like 2on2. Using the mode default.\n"
                )));
            } else if next.name() == "ctf" {
                if let Some(p) = team::parse_match_alias(match_alias) {
                    if p.teams != 2 {
                        host.conprint(&cstring("rtx: CTF is always 2 teams — clamping the format to 2 sides.\n"));
                    }
                }
            }
        }

        // The cvar-buffer borrows (`mode_name`/`match_alias`) are read-only above and `next`/`cfg`
        // are `Copy`, so we're free to take `&mut self` now.
        if next.name() != self.mode.name() {
            self.mode = next;
            self.arena = ArenaState::default();
            self.team_match = MatchState {
                config: cfg,
                ..Default::default()
            };
            host.conprint(&cstring(&format!("rtx: game mode = {}\n", next.name())));
        } else if cfg != self.team_match.config {
            self.team_match = MatchState {
                config: cfg,
                ..Default::default()
            };
            if cfg.teams >= 2 {
                host.conprint(&cstring(&format!("rtx: match format = {}\n", team::format_label(cfg))));
            }
        }
        // Team play needs friendly-fire protection on (the `"team"` userinfo does the rest).
        if cfg.teams >= 2 && host.cvar(c"teamplay") == 0.0 {
            host.cvar_set(c"teamplay", c"1");
        }
    }
}

/// Map (re)load housekeeping for the mode layer, called from `worldspawn` after `refresh_mode`.
/// Resets the per-map round state (Arena) and reconciles the team-match state — which must run
/// regardless of mode, since switching away from a team composition has to clear it (so it can't
/// hang off the active mode's own hook). Then the active mode's own [`GameMode::on_worldspawn`] runs.
pub(crate) fn on_worldspawn(g: &mut GameState) {
    g.arena = ArenaState::default();
    // Fresh opponent hypotheses, seeded with this mode's spawn kit (Arena/Midair hand out fixed
    // arsenals; everything else starts at the stock respawn kit).
    g.opponents = crate::bot::model::OpponentModel::new(crate::bot::model::baseline_for_mode(g.mode.name()));
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
        .filter(|&e| g.entities[e].in_use && g.entities[e].is_player())
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
        if !ent.is_alive() || !pred(g, e) {
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

/// One announce step of a 3…2…1 countdown ending at world time `until`. Given the last whole second
/// already shown (`last`), returns the updated marker and the second to centerprint now — each shown
/// exactly once, and only while positive (the "FIGHT!" at zero is the caller's own go-live step,
/// which differs between arena and team). Pure, so the tick logic is unit-testable without a match.
pub(crate) fn countdown_announce(until: f32, now: f32, last: i32) -> (i32, Option<i32>) {
    let remaining = (until - now).ceil() as i32;
    if remaining != last {
        (remaining, (remaining > 0).then_some(remaining))
    } else {
        (last, None)
    }
}

// --- shared spectator/audience helpers -----------------------------------------------------------

/// The harmless-spectator loadout: axe only, no ammo, positive health/armor. Shared by Rocket
/// Arena's audience and a structured match's benched late-joiners; damage to (and from) these
/// players is refused by the bench/audience damage gates. Health/armor must stay positive — a
/// client (and the bot AI) treats 0 health as dead and locks movement, freezing the spectator.
/// A fixed spawn kit — the arena fighter, midair, race and audience kits each hand-wrote these
/// fields. `apply` *assigns* `items` (not `.with`), which drops the grapple bit
/// `put_client_in_server` hands out first (the intended no-hook-in-the-arena behavior). `max_health:
/// None` leaves the current max untouched (the audience kit never set it).
pub(crate) struct Loadout {
    pub items: Items,
    pub health: f32,
    pub max_health: Option<f32>,
    pub armorvalue: f32,
    pub armortype: f32,
    pub shells: f32,
    pub nails: f32,
    pub rockets: f32,
    pub cells: f32,
    pub weapon: Weapon,
}

impl Loadout {
    pub(crate) fn apply(&self, g: &mut GameState, e: EntId) {
        let v = &mut g.entities[e].v;
        v.items = self.items.as_f32();
        v.health = self.health;
        if let Some(mh) = self.max_health {
            v.max_health = mh;
        }
        v.armorvalue = self.armorvalue;
        v.armortype = self.armortype;
        v.ammo_shells = self.shells;
        v.ammo_nails = self.nails;
        v.ammo_rockets = self.rockets;
        v.ammo_cells = self.cells;
        v.weapon = self.weapon;
    }
}

pub(crate) fn audience_loadout(g: &mut GameState, e: EntId) {
    Loadout {
        items: Items::AXE,
        health: 100.0,
        max_health: None,
        armorvalue: 100.0,
        armortype: 0.8,
        shells: 0.0,
        nails: 0.0,
        rockets: 0.0,
        cells: 0.0,
        weapon: Weapon::Axe,
    }
    .apply(g, e);
}

/// A roaming destination among `classname` spawns for a bot with nothing to fight — re-picked on a
/// staggered timer or once it has nearly arrived, so the bot keeps strolling between points instead
/// of freezing on the spot (a frozen bot also trips the stuck-jumper). `prefer` is consulted at the
/// moment of re-pick for a smarter destination (Rocket Arena's audience picks a spot overlooking the
/// duel); it's evaluated lazily so any line-of-sight traces it runs happen only when actually
/// re-picking, and `None` falls back to a random `classname` spawn. Shared by Arena's fighter /
/// audience roam and a benched player's idle wander.
pub(crate) fn wander_point(
    g: &mut GameState,
    bot: EntId,
    classname: &str,
    prefer: impl FnOnce(&mut GameState) -> Option<Vec3>,
) -> Vec3 {
    let now = g.time();
    let origin = g.entities[bot].v.origin;
    let target = g.entities[bot].bot.wander_target;
    let (dx, dy) = (target.x - origin.x, target.y - origin.y);
    let arrived = target != Vec3::ZERO && (dx * dx + dy * dy).sqrt() < 48.0;
    if now >= g.entities[bot].bot.wander_time || target == Vec3::ZERO || arrived {
        let next = prefer(g).unwrap_or_else(|| {
            // A roam destination, not a spawn — no spawn memory, no self-exclusion.
            let spot = g.select_spawn_point_of(classname, None);
            if spot != EntId::WORLD {
                g.entities[spot].v.origin
            } else {
                origin
            }
        });
        let jitter = g.random();
        let b = &mut g.entities[bot].bot;
        b.wander_target = next;
        b.wander_time = now + 3.0 + jitter * 3.0;
    }
    g.entities[bot].bot.wander_target
}

#[cfg(test)]
mod tests {
    use super::countdown_announce;

    /// Each whole second of a countdown is announced exactly once (only while positive); the same
    /// second polled again yields nothing, and zero/negative print nothing (the caller owns "FIGHT!").
    #[test]
    fn countdown_announces_each_second_once() {
        // 3s out, nothing shown yet (last = -1): print 3.
        assert_eq!(countdown_announce(3.0, 0.0, -1), (3, Some(3)));
        // Still within the 3-second, already shown: nothing.
        assert_eq!(countdown_announce(3.0, 0.4, 3), (3, None));
        // Crossed into the 2-second: print 2.
        assert_eq!(countdown_announce(3.0, 1.5, 3), (2, Some(2)));
        // At/after zero: update the marker but print nothing.
        assert_eq!(countdown_announce(3.0, 3.0, 1), (0, None));
        assert_eq!(countdown_announce(3.0, 3.5, 0), (0, None));
    }
}
