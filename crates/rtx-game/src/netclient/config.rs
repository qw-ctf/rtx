// SPDX-License-Identifier: AGPL-3.0-or-later

//! What to tell a bot client before it connects.
//!
//! Hand-rolled argument parsing, matching the rest of the workspace's tools — the whole surface is
//! a couple of dozen flags, and a dependency to read them would cost more than it saves.

use std::net::{SocketAddr, ToSocketAddrs};
use std::path::PathBuf;

/// Which network protocol family to speak. A connection picks one; they share no wire bytes.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Protocol {
    /// QuakeWorld (the default): `getchallenge`/`connect`, port 27500.
    #[default]
    Qw,
    /// NetQuake — the original Quake protocol: `CCREQ_CONNECT`, port 26000.
    Nq,
}

/// A bot client's configuration.
#[derive(Clone, Debug)]
pub struct Config {
    /// Which protocol family to speak.
    pub proto: Protocol,
    /// The gamedir to look for maps under. NetQuake never announces one, so it defaults to `id1`;
    /// QuakeWorld learns it from `svc_serverdata`, so this stays empty there unless overridden.
    pub game: String,
    /// The server to connect to.
    pub server: SocketAddr,
    /// The directory holding `qw/`, `id1/`, … — where the maps are.
    pub basedir: PathBuf,
    /// How many bots to bring.
    pub bots: usize,
    /// The label after the `bot•` tag. `None` draws a name per bot from the built-in list
    /// (`bot•Grunt`, …); `Some(base)` uses it instead (`bot•base`, a squad appending a number).
    pub name: Option<String>,
    /// Team string, for teamplay servers.
    pub team: String,
    /// Skin name; empty for the default.
    pub skin: String,
    /// Shirt/trouser colours, 0–13.
    pub colors: (u8, u8),
    /// Bot skill, 0–7.
    pub skill: f32,
    /// Connect as a spectator and just watch — the parser soak, and a way to look before playing.
    pub spectate: bool,
    /// Say `ready` on a server that waits to be told, so a match can actually start.
    pub auto_ready: bool,
    /// Fetch a map we don't have rather than fail the connection: HTTP first, then the connected
    /// QuakeWorld server when available. See [`crate::netclient::download`].
    pub download: bool,
    /// A cfg file of cvar settings to apply on startup — the client's `server.cfg`. Defaults to
    /// `<basedir>/rtx.cfg` if that exists; `--config` names another. Empty = none.
    pub config_file: Option<PathBuf>,
    /// Bind the control channel here, so a harness can drive and inspect the bots. See
    /// [`crate::control`].
    pub control_port: Option<u16>,
    /// Exit after this many seconds; `None` runs until stopped.
    pub soak: Option<u64>,
    /// Write every datagram here, as a parser fixture.
    pub wiretap: Option<PathBuf>,
    /// `rtx_*` overrides, applied after the defaults.
    pub cvars: Vec<(String, String)>,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            proto: Protocol::Qw,
            game: String::new(),
            server: ([127, 0, 0, 1], rtx_proto::protocol::PORT).into(),
            basedir: PathBuf::from("."),
            bots: 1,
            name: None,
            team: String::new(),
            skin: String::new(),
            colors: (0, 0),
            skill: 3.0,
            spectate: false,
            auto_ready: true,
            download: true,
            config_file: None,
            control_port: None,
            soak: None,
            wiretap: None,
            cvars: Vec::new(),
        }
    }
}

/// The `--help` text.
pub const USAGE: &str = "\
rtx-client — the rtx bots, as real QuakeWorld or NetQuake clients

usage: rtx-client --server <host[:port]> --basedir <dir> [options]

  --server <host[:port]>  server to join (default port 27500, or 26000 with --proto nq)
  --proto <qw|nq>         wire protocol: QuakeWorld (default) or NetQuake
  --game <dir>            gamedir the maps live under (default id1 for --proto nq)
  --basedir <dir>         Quake directory holding qw/ and id1/ — the maps must be here
  --bots <n>              how many bots to bring (default 1)
  --name <s>              label after the `bot•` tag (default: a random name per bot)
  --team <s>              team, on a teamplay server
  --skin <s>              skin name
  --colors <top> <bottom> shirt and trouser colours, 0-13
  --skill <0..7>          bot skill (default 3)
  --spectate              watch instead of playing
  --no-auto-ready         don't answer KTX ready/join prompts
  --no-download           don't fetch a missing map — fail the connection instead
  --config <file>         cvar cfg to apply on startup (default <basedir>/rtx.cfg if present)
  --soak <secs>           exit after this long
  --wiretap <dir>         write every datagram there, as a parser fixture
  +set <cvar> <value>     override an rtx tunable (repeatable)
  -h, --help              this
";

/// Parse a command line. `Err` carries a message worth printing.
pub fn parse(argv: &[String]) -> Result<Config, String> {
    let mut c = Config::default();
    // The server's default port depends on `--proto`, which may be given after `--server`, so hold
    // the raw string and resolve it once the whole line is parsed.
    let mut server_raw: Option<String> = None;
    let mut i = 0;

    // Positional-ish flags need their value; report the missing one rather than panicking.
    let need = |i: usize, what: &str| -> Result<String, String> {
        argv.get(i + 1).cloned().ok_or_else(|| format!("{what} needs a value"))
    };

    while i < argv.len() {
        match argv[i].as_str() {
            "-h" | "--help" => return Err(USAGE.to_string()),
            "--spectate" => {
                c.spectate = true;
                i += 1;
            }
            "--no-auto-ready" => {
                c.auto_ready = false;
                i += 1;
            }
            "--no-download" => {
                c.download = false;
                i += 1;
            }
            "--config" => {
                c.config_file = Some(PathBuf::from(need(i, "--config")?));
                i += 2;
            }
            "--control-port" => {
                c.control_port = Some(
                    need(i, "--control-port")?
                        .parse()
                        .map_err(|_| "--control-port wants a port number".to_string())?,
                );
                i += 2;
            }
            "--server" => {
                server_raw = Some(need(i, "--server")?);
                i += 2;
            }
            "--proto" => {
                c.proto = match need(i, "--proto")?.as_str() {
                    "qw" => Protocol::Qw,
                    "nq" => Protocol::Nq,
                    other => return Err(format!("--proto wants `qw` or `nq`, not `{other}`")),
                };
                i += 2;
            }
            "--game" => {
                c.game = need(i, "--game")?;
                i += 2;
            }
            "--basedir" => {
                c.basedir = PathBuf::from(need(i, "--basedir")?);
                i += 2;
            }
            "--bots" => {
                c.bots = need(i, "--bots")?.parse().map_err(|_| "--bots wants a number".to_string())?;
                i += 2;
            }
            "--name" => {
                c.name = Some(need(i, "--name")?);
                i += 2;
            }
            "--team" => {
                c.team = need(i, "--team")?;
                i += 2;
            }
            "--skin" => {
                c.skin = need(i, "--skin")?;
                i += 2;
            }
            "--skill" => {
                c.skill = need(i, "--skill")?.parse().map_err(|_| "--skill wants a number".to_string())?;
                i += 2;
            }
            "--colors" => {
                let top = need(i, "--colors")?.parse().map_err(|_| "--colors wants two numbers".to_string())?;
                let bottom = argv
                    .get(i + 2)
                    .ok_or("--colors wants two numbers")?
                    .parse()
                    .map_err(|_| "--colors wants two numbers".to_string())?;
                c.colors = (top, bottom);
                i += 3;
            }
            "--soak" => {
                c.soak = Some(need(i, "--soak")?.parse().map_err(|_| "--soak wants seconds".to_string())?);
                i += 2;
            }
            "--wiretap" => {
                c.wiretap = Some(PathBuf::from(need(i, "--wiretap")?));
                i += 2;
            }
            "+set" => {
                let name = need(i, "+set")?;
                let value = argv.get(i + 2).cloned().ok_or("+set wants a name and a value")?;
                c.cvars.push((name, value));
                i += 3;
            }
            other => return Err(format!("unknown option `{other}`\n\n{USAGE}")),
        }
    }

    let Some(server_raw) = server_raw else {
        return Err(format!("--server is required\n\n{USAGE}"));
    };
    // Resolve now that the protocol (hence the default port) is settled.
    let default_port = match c.proto {
        Protocol::Qw => rtx_proto::protocol::PORT,
        Protocol::Nq => rtx_proto::nq::protocol::PORT,
    };
    c.server = resolve(&server_raw, default_port)?;
    // NetQuake never announces a gamedir, so it needs a default; QuakeWorld learns it on connect.
    if c.proto == Protocol::Nq && c.game.is_empty() {
        c.game = "id1".to_string();
    }
    if c.bots == 0 {
        return Err("--bots must be at least 1".to_string());
    }
    Ok(c)
}

/// One meaningful line of a Quake console cfg, as far as a headless client is concerned.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) enum CfgLine {
    /// `set NAME VALUE` (or the bare `NAME VALUE`) — a cvar to apply.
    Set(String, String),
    /// `exec FILE` — another cfg to read, relative to this one.
    Exec(String),
}

/// Read the cvar sets (and `exec`s) out of a Quake console cfg.
///
/// A cfg is console commands, one per line. A headless bot has no console to run most of them — a
/// `bind`, an `alias`, a `map` — so it takes only what it can act on: the cvar sets (`set NAME
/// VALUE`, `seta NAME VALUE`, and the bare `NAME VALUE` form QuakeWorld also accepts) and `exec`,
/// which chains to another cfg. Everything else is ignored rather than errored — a real cfg is full
/// of lines a bot has no use for, and refusing them would make the whole file unusable. `//` starts a
/// comment; a `"quoted"` value keeps its spaces.
pub(crate) fn parse_cfg(text: &str) -> Vec<CfgLine> {
    text.lines().filter_map(parse_cfg_line).collect()
}

fn parse_cfg_line(line: &str) -> Option<CfgLine> {
    let line = line.split("//").next().unwrap_or("").trim();
    if line.is_empty() {
        return None;
    }
    let toks = tokenize(line);
    match toks.as_slice() {
        [kw, name, value] if kw == "set" || kw == "seta" => Some(CfgLine::Set(name.clone(), value.clone())),
        [kw, file] if kw == "exec" => Some(CfgLine::Exec(file.clone())),
        // The bare `cvar value` form — but not a two-word *command* (a lone `exec x` was caught
        // above; anything else with two tokens we treat as a cvar, since a spurious one nobody reads
        // is harmless and the real ones are what the operator meant).
        [name, value] => Some(CfgLine::Set(name.clone(), value.clone())),
        _ => None,
    }
}

/// Split a cfg line into tokens, treating a `"quoted string"` as one — the only grouping a Quake cfg
/// has.
fn tokenize(line: &str) -> Vec<String> {
    let mut out = Vec::new();
    let mut chars = line.chars().peekable();
    while let Some(&c) = chars.peek() {
        if c.is_whitespace() {
            chars.next();
        } else if c == '"' {
            chars.next(); // opening quote
            let mut tok = String::new();
            for c in chars.by_ref() {
                if c == '"' {
                    break;
                }
                tok.push(c);
            }
            out.push(tok);
        } else {
            let mut tok = String::new();
            while let Some(&c) = chars.peek() {
                if c.is_whitespace() {
                    break;
                }
                tok.push(c);
                chars.next();
            }
            out.push(tok);
        }
    }
    out
}

/// Resolve `host` or `host:port`, defaulting to `default_port` when none is given.
fn resolve(s: &str, default_port: u16) -> Result<SocketAddr, String> {
    // An IPv6 literal is full of colons, so "does it have a colon" isn't the question — "does it
    // end in one that isn't inside brackets" is.
    let with_port = if s.rsplit(':').next().is_some_and(|p| p.parse::<u16>().is_ok()) {
        s.to_string()
    } else {
        format!("{s}:{default_port}")
    };
    with_port
        .to_socket_addrs()
        .map_err(|e| format!("can't resolve `{s}`: {e}"))?
        .next()
        .ok_or_else(|| format!("`{s}` resolved to nothing"))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn args(s: &[&str]) -> Vec<String> {
        s.iter().map(|x| x.to_string()).collect()
    }

    /// A cfg is server-style console lines; the client takes the cvar sets and the `exec` chains and
    /// ignores the commands it has no console for — so a real, cluttered cfg is usable as-is rather
    /// than all-or-nothing.
    #[test]
    fn reads_the_cvars_out_of_a_cfg() {
        let cfg = r#"
            // the bot's tuning
            set rtx_bot_curljump 1
            seta rtx_bot_skill 7
            rtx_bot_count 4            // the bare form works too
            hostname "My Bots"        // a quoted value keeps its spaces
            bind x "+forward"         // a command we can't run — ignored, not an error
            exec more.cfg
        "#;
        assert_eq!(parse_cfg(cfg), vec![
            CfgLine::Set("rtx_bot_curljump".into(), "1".into()),
            CfgLine::Set("rtx_bot_skill".into(), "7".into()),
            CfgLine::Set("rtx_bot_count".into(), "4".into()),
            CfgLine::Set("hostname".into(), "My Bots".into()),
            CfgLine::Exec("more.cfg".into()),
        ]);
        // `bind x "+forward"` is three tokens and not a `set`, so it's dropped — a spurious cvar
        // named `bind` would be harmless, but three-token commands don't even become one.
        assert!(!parse_cfg("bind x \"+forward\"").iter().any(|l| matches!(l, CfgLine::Set(n, _) if n == "bind")));
        // A comment-only or blank cfg yields nothing.
        assert!(parse_cfg("// just a note\n\n   \n").is_empty());
    }

    /// The common case, and the defaults that ride along with it.
    #[test]
    fn parses_a_typical_command_line() {
        let c = parse(&args(&["--server", "127.0.0.1:27500", "--basedir", "/q", "--bots", "2"])).unwrap();
        assert_eq!(c.server, "127.0.0.1:27500".parse::<SocketAddr>().unwrap());
        assert_eq!(c.basedir, PathBuf::from("/q"));
        assert_eq!(c.bots, 2);
        assert_eq!(c.skill, 3.0);
        assert!(c.auto_ready, "bots join matches by themselves unless told not to");
        assert!(!c.spectate);
    }

    /// A bare host means the QuakeWorld port — nobody types 27500.
    #[test]
    fn server_port_defaults() {
        let c = parse(&args(&["--server", "127.0.0.1"])).unwrap();
        assert_eq!(c.server.port(), 27500);

        let c = parse(&args(&["--server", "127.0.0.1:1234"])).unwrap();
        assert_eq!(c.server.port(), 1234);
    }

    /// `--proto nq` changes the default port to 26000 and gamedir to id1 — even when `--server`
    /// comes first, since the port is resolved after the whole line is parsed. An explicit port and
    /// gamedir still win.
    #[test]
    fn nq_protocol_sets_port_and_gamedir_defaults() {
        let c = parse(&args(&["--server", "127.0.0.1", "--proto", "nq"])).unwrap();
        assert_eq!(c.proto, Protocol::Nq);
        assert_eq!(c.server.port(), 26000);
        assert_eq!(c.game, "id1");

        // Defaults yield to explicit values.
        let c = parse(&args(&["--proto", "nq", "--server", "127.0.0.1:12345", "--game", "rogue"])).unwrap();
        assert_eq!(c.server.port(), 12345);
        assert_eq!(c.game, "rogue");

        // Without --proto, QuakeWorld defaults are unchanged.
        let c = parse(&args(&["--server", "127.0.0.1"])).unwrap();
        assert_eq!(c.proto, Protocol::Qw);
        assert_eq!(c.server.port(), 27500);
    }

    /// A bad protocol name is a sentence, not a panic.
    #[test]
    fn rejects_unknown_protocol() {
        assert!(parse(&args(&["--server", "x", "--proto", "dp"])).unwrap_err().contains("qw` or `nq"));
    }

    /// Every flag, so a rename or a mis-stepped index shows up here rather than at a LAN party.
    #[test]
    fn parses_every_option() {
        let c = parse(&args(&[
            "--server", "127.0.0.1", "--basedir", "/q", "--bots", "4", "--name", "botto", "--team",
            "red", "--skin", "base", "--colors", "4", "11", "--skill", "7", "--spectate",
            "--no-auto-ready", "--soak", "600", "--wiretap", "/tmp/fix", "+set", "rtx_bot_bhop", "0",
            "+set", "rtx_mode", "ctf",
        ]))
        .unwrap();
        assert_eq!(c.bots, 4);
        assert_eq!(c.name, Some("botto".to_string()));
        assert_eq!(c.team, "red");
        assert_eq!(c.skin, "base");
        assert_eq!(c.colors, (4, 11));
        assert_eq!(c.skill, 7.0);
        assert!(c.spectate);
        assert!(!c.auto_ready);
        assert_eq!(c.soak, Some(600));
        assert_eq!(c.wiretap, Some(PathBuf::from("/tmp/fix")));
        assert_eq!(
            c.cvars,
            vec![
                ("rtx_bot_bhop".to_string(), "0".to_string()),
                ("rtx_mode".to_string(), "ctf".to_string())
            ]
        );
    }

    /// Bad input gets a sentence, not a panic and not a stack trace.
    #[test]
    fn rejects_bad_input_with_a_message() {
        assert!(parse(&args(&["--basedir", "/q"])).unwrap_err().contains("--server is required"));
        assert!(parse(&args(&["--server"])).unwrap_err().contains("needs a value"));
        assert!(parse(&args(&["--server", "127.0.0.1", "--bots", "x"])).unwrap_err().contains("number"));
        assert!(parse(&args(&["--server", "127.0.0.1", "--bots", "0"])).unwrap_err().contains("at least 1"));
        assert!(parse(&args(&["--nonsense"])).unwrap_err().contains("unknown option"));
        assert!(parse(&args(&["--help"])).unwrap_err().contains("usage:"));
        assert!(parse(&args(&["--server", "no such host.invalid"])).is_err());
        // `--colors` takes two, and saying so beats silently reading the next flag as a number.
        assert!(parse(&args(&["--server", "127.0.0.1", "--colors", "4"])).unwrap_err().contains("two numbers"));
    }
}
