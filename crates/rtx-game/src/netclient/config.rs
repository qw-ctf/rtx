// SPDX-License-Identifier: AGPL-3.0-or-later

//! What to tell a bot client before it connects.
//!
//! Hand-rolled argument parsing, matching the rest of the workspace's tools — the whole surface is
//! a couple of dozen flags, and a dependency to read them would cost more than it saves.

use std::net::{SocketAddr, ToSocketAddrs};
use std::path::PathBuf;

/// A bot client's configuration.
#[derive(Clone, Debug)]
pub struct Config {
    /// The server to connect to.
    pub server: SocketAddr,
    /// The directory holding `qw/`, `id1/`, … — where the maps are.
    pub basedir: PathBuf,
    /// How many bots to bring.
    pub bots: usize,
    /// Base name; a squad appends a number.
    pub name: String,
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
    /// Answer KTX's ready/join prompts by ourselves.
    pub auto_ready: bool,
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
            server: ([127, 0, 0, 1], rtx_proto::protocol::PORT).into(),
            basedir: PathBuf::from("."),
            bots: 1,
            name: "rtx".to_string(),
            team: String::new(),
            skin: String::new(),
            colors: (0, 0),
            skill: 3.0,
            spectate: false,
            auto_ready: true,
            soak: None,
            wiretap: None,
            cvars: Vec::new(),
        }
    }
}

/// The `--help` text.
pub const USAGE: &str = "\
rtx-client — the rtx bots, as real QuakeWorld clients

usage: rtx-client --server <host[:port]> --basedir <dir> [options]

  --server <host[:port]>  server to join (default port 27500)
  --basedir <dir>         Quake directory holding qw/ and id1/ — the maps must be here
  --bots <n>              how many bots to bring (default 1)
  --name <s>              base name; a squad appends a number (default \"rtx\")
  --team <s>              team, on a teamplay server
  --skin <s>              skin name
  --colors <top> <bottom> shirt and trouser colours, 0-13
  --skill <0..7>          bot skill (default 3)
  --spectate              watch instead of playing
  --no-auto-ready         don't answer KTX ready/join prompts
  --soak <secs>           exit after this long
  --wiretap <dir>         write every datagram there, as a parser fixture
  +set <cvar> <value>     override an rtx tunable (repeatable)
  -h, --help              this
";

/// Parse a command line. `Err` carries a message worth printing.
pub fn parse(argv: &[String]) -> Result<Config, String> {
    let mut c = Config::default();
    let mut have_server = false;
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
            "--server" => {
                c.server = resolve(&need(i, "--server")?)?;
                have_server = true;
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
                c.name = need(i, "--name")?;
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

    if !have_server {
        return Err(format!("--server is required\n\n{USAGE}"));
    }
    if c.bots == 0 {
        return Err("--bots must be at least 1".to_string());
    }
    Ok(c)
}

/// Resolve `host` or `host:port`, defaulting to the QuakeWorld port.
fn resolve(s: &str) -> Result<SocketAddr, String> {
    // An IPv6 literal is full of colons, so "does it have a colon" isn't the question — "does it
    // end in one that isn't inside brackets" is.
    let with_port = if s.rsplit(':').next().is_some_and(|p| p.parse::<u16>().is_ok()) {
        s.to_string()
    } else {
        format!("{s}:{}", rtx_proto::protocol::PORT)
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
        assert_eq!(c.name, "botto");
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
