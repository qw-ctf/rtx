// SPDX-License-Identifier: AGPL-3.0-or-later

//! Reading the game from the server's own description of it.
//!
//! A network client can't be told to play ctf by a command line it never sees — it has to *work out*
//! what game it has joined, before it spawns the world, so `refresh_mode` selects the right brain. The
//! server describes itself in serverinfo, and rtx and KTX describe themselves in the **same
//! vocabulary** (rtx publishes KTX's `mode`/`status` keys deliberately, `game.rs::publish_serverinfo`)
//! — so one parser reads both, and everything downstream is the mode machinery the bots already have.
//!
//! Two things come out of the serverinfo:
//!
//! - **which mode** — the [`ModeChoice`] the oracle resolves into the `rtx_mode`/`rtx_match` cvars
//!   `refresh_mode` reads. See [`select_mode`].
//! - **which phase** — Standby / Countdown / live, from `status`, so the bot's rocket-jump gate knows
//!   not to fire into a countdown the server would swallow. See [`match_phase`] and [`Phase`].

use rtx_proto::info::Info;

/// What the oracle decided the client should play — the two cvars `refresh_mode` resolves from.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct ModeChoice {
    /// `rtx_mode`: the ruleset (`dm`/`ctf`/`midair`/`ra`/`race`).
    pub mode: &'static str,
    /// `rtx_match`: the composition alias (`ffa`, `2on2`, …). Empty means "the mode's default".
    pub composition: String,
}

impl ModeChoice {
    fn new(mode: &'static str, composition: &str) -> ModeChoice {
        ModeChoice { mode, composition: composition.to_string() }
    }
}

/// Work out what game the server is running, from what it says about itself.
///
/// rtx (`rtxver`) and KTX (`ktxver`) both publish `mode` in the same grammar, so one parse serves
/// both — with the single ambiguity (`…-ra`) resolved by which mod it is. A server that says nothing
/// we recognise is deathmatch, open play: teams, if any, still arrive per-player through userinfo and
/// drive the "have I got a side" seam, so we don't need a vanilla server's composition to fight the
/// right people.
///
/// The operator's `+set rtx_mode`/`rtx_match` overrides this — but that's the *caller's* business (it
/// holds the command line and simply doesn't overwrite a key the operator pinned); the oracle only
/// reads the wire.
pub(crate) fn select_mode(info: &Info) -> ModeChoice {
    let rtx = info.get("rtxver").is_some_and(|v| !v.is_empty());
    let ktx = info.get("ktxver").is_some_and(|v| !v.is_empty());
    match info.get("mode") {
        Some(mode) if (rtx || ktx) && !mode.is_empty() => parse_mode(mode, rtx),
        _ => ModeChoice::new("dm", "ffa"),
    }
}

/// Parse a `mode` string in KTX's `<base>[-suffix…]` grammar into a [`ModeChoice`].
///
/// The head token is the composition (`ffa`, `1on1`, `2on2`, `ctf`, …); the suffixes are modifiers
/// (`-midair`, `-race`, `-ra`, and KTX's `-ca`/`-wo`/`-instagib`/… which we don't play and ignore).
/// `rtx` is whether this is an rtx server — the one place it matters is `-ra`: rtx's Rocket Arena is a
/// role/round machine we drive from published state, while KTX's "RA" is a winner-stays duel that the
/// plain deathmatch brain plays correctly, so the same suffix means different brains.
fn parse_mode(mode: &str, rtx: bool) -> ModeChoice {
    let mut parts = mode.split('-');
    let head = parts.next().unwrap_or("");
    let suffixes: Vec<&str> = parts.collect();

    // The ruleset: a recognised suffix wins, else CTF by its head, else deathmatch.
    let ruleset = if suffixes.contains(&"midair") {
        "midair"
    } else if suffixes.contains(&"race") {
        "race"
    } else if suffixes.contains(&"ra") {
        if rtx {
            "ra"
        } else {
            "dm"
        }
    } else if head == "ctf" {
        "ctf"
    } else {
        "dm"
    };

    // The composition: a duel or ffa is *open play* — a KTX duel has no teams, and a team filter with
    // everyone at team 0 would find no enemies; a rtx duel's teams arrive through userinfo and drive
    // the seam anyway. A real team format (`2on2`, `4on4`, …) is kept verbatim. ctf/ra/race resolve
    // their own composition, so `ffa` is a harmless placeholder for them.
    let composition = if head == "1on1" || head == "ffa" || head == "ctf" {
        "ffa"
    } else if head.contains("on") {
        head // NonMon: kept as-is for `refresh_mode`'s `resolve_composition` to parse
    } else {
        "ffa"
    };

    ModeChoice::new(ruleset, composition)
}

/// The match phase, in the shape the brain reads it — see [`match_phase`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Phase {
    /// Prewar / warmup — playable, but no match is running.
    Warmup,
    /// A ready countdown is ticking; the server will swallow a shot fired now.
    Countdown,
    /// The game is on.
    Live,
}

/// The match phase from the server's `status`, parsed exactly as ezquake parses it
/// (`cl_parse.c:2402`): case-insensitive `Standby`/`Countdown`, and **anything else is live** — a
/// live clock (`"7 min left"`), `Forcestart`, or an absent key (open play, which is always live).
///
/// The one client-side consumer is the rocket-jump gate (via `team_match.phase`): a bot mustn't jump
/// into a countdown whose rocket the server won't fire. Everything else about a countdown — the
/// freeze, the respawn — is the server's to enforce on our body.
pub(crate) fn match_phase(info: &Info) -> Phase {
    match info.get("status") {
        Some(s) if s.eq_ignore_ascii_case("standby") => Phase::Warmup,
        Some(s) if s.eq_ignore_ascii_case("countdown") => Phase::Countdown,
        Some(s) if s.eq_ignore_ascii_case("forcestart") => Phase::Countdown,
        _ => Phase::Live,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn info(pairs: &[(&str, &str)]) -> Info {
        let mut i = Info::new();
        for (k, v) in pairs {
            i.set(k, v);
        }
        i
    }

    /// The core of the oracle: the KTX/rtx `mode` grammar → the two cvars the brain resolves from.
    /// Every row here is a real server we could join.
    #[test]
    fn reads_the_mode_a_server_describes() {
        let rtx = |m: &str| parse_mode(m, true);
        let ktx = |m: &str| parse_mode(m, false);

        // Open deathmatch, both flavours of the head.
        assert_eq!(rtx("ffa"), ModeChoice::new("dm", "ffa"));
        assert_eq!(rtx("1on1"), ModeChoice::new("dm", "ffa"), "a duel is open play — teams come via userinfo");

        // Team deathmatch keeps its composition for `resolve_composition`.
        assert_eq!(rtx("2on2"), ModeChoice::new("dm", "2on2"));
        assert_eq!(rtx("4on4"), ModeChoice::new("dm", "4on4"));

        // CTF is its own head.
        assert_eq!(rtx("ctf"), ModeChoice::new("ctf", "ffa"));

        // Modifiers ride as suffixes.
        assert_eq!(rtx("ffa-midair"), ModeChoice::new("midair", "ffa"));
        assert_eq!(rtx("1on1-midair"), ModeChoice::new("midair", "ffa"));
        assert_eq!(rtx("ffa-race"), ModeChoice::new("race", "ffa"));

        // The one ambiguity: `-ra` is rtx's arena only on an rtx server. KTX's "RA" is a winner-stays
        // duel, which the deathmatch brain plays right.
        assert_eq!(rtx("1on1-ra"), ModeChoice::new("ra", "ffa"));
        assert_eq!(ktx("1on1-ra"), ModeChoice::new("dm", "ffa"), "KTX RA is a duel, not rtx's arena");

        // KTX modes we don't model get the deathmatch brain and let the server enforce the rules.
        assert_eq!(ktx("4on4-ca"), ModeChoice::new("dm", "4on4"));
        assert_eq!(ktx("wipeout"), ModeChoice::new("dm", "ffa"));
        assert_eq!(ktx("ffa-instagib"), ModeChoice::new("dm", "ffa"));
    }

    /// A server that says nothing we recognise is deathmatch, open play — the honest default. The
    /// public server the probe hit (mode `ffa`, `ktxver` present) lands here as ffa either way.
    #[test]
    fn an_unknown_or_silent_server_is_open_deathmatch() {
        assert_eq!(select_mode(&info(&[])), ModeChoice::new("dm", "ffa"));
        // A `mode` with no `rtxver`/`ktxver` marker isn't trusted — a vanilla server's `mode` key, if
        // any, isn't this grammar.
        assert_eq!(
            select_mode(&info(&[("mode", "ctf")])),
            ModeChoice::new("dm", "ffa"),
            "no rtxver/ktxver — don't trust the string",
        );
        // With the marker, it's read.
        assert_eq!(
            select_mode(&info(&[("mode", "ctf"), ("ktxver", "1.48")])),
            ModeChoice::new("ctf", "ffa"),
        );
    }

    /// The phase feed, ezquake's rules: Standby/Countdown by name, everything else live.
    #[test]
    fn reads_the_match_phase_like_ezquake() {
        assert_eq!(match_phase(&info(&[("status", "Standby")])), Phase::Warmup);
        assert_eq!(match_phase(&info(&[("status", "standby")])), Phase::Warmup, "case-insensitive");
        assert_eq!(match_phase(&info(&[("status", "Countdown")])), Phase::Countdown);
        assert_eq!(match_phase(&info(&[("status", "Forcestart")])), Phase::Countdown);
        assert_eq!(match_phase(&info(&[("status", "7 min left")])), Phase::Live);
        assert_eq!(match_phase(&info(&[("status", "in progress")])), Phase::Live);
        assert_eq!(match_phase(&info(&[])), Phase::Live, "no status = open play = always live");
    }
}
