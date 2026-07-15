// SPDX-License-Identifier: AGPL-3.0-or-later

//! `rtx-client` — the rtx bots as real QuakeWorld clients.
//!
//! Connects to any QuakeWorld server over UDP and plays, running the same brain the server-side
//! bots use. Point it at a server humans are on and they'll have company.
//!
//! This binary is only the front door: it parses a command line and hands over to
//! [`rtx::netclient`], which is where the client actually lives (it needs the game's internals, and
//! reaching them from out here would mean making them public — see that module's docs).
//!
//! ```sh
//! cargo run -p rtx-client -- --server 127.0.0.1:27500 --basedir ~/quake --bots 2
//! ```
//!
//! It sits outside the workspace's default members, so neither `cargo build` nor the `qwprogs`
//! release ever compiles it.

use std::process::ExitCode;

fn main() -> ExitCode {
    let argv: Vec<String> = std::env::args().skip(1).collect();
    if argv.is_empty() {
        eprint!("{}", rtx::netclient::USAGE);
        return ExitCode::from(2);
    }

    let config = match rtx::netclient::parse_args(&argv) {
        Ok(c) => c,
        Err(msg) => {
            // `--help` comes back this way too; both want the text and a non-zero exit, and
            // neither wants a backtrace.
            eprint!("{msg}");
            return ExitCode::from(2);
        }
    };

    match rtx::netclient::run(config) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("rtx-client: {e}");
            ExitCode::FAILURE
        }
    }
}
