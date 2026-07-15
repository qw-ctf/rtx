// SPDX-License-Identifier: AGPL-3.0-or-later

//! The bot brain as a real QuakeWorld **network client**.
//!
//! The same bots that run inside the server's game module, embodied instead as clients that connect
//! over UDP — so they can play against humans on any server, or against qwprogs-hosted bots. The
//! brain is not reimplemented and not forked: it is the *same code*, reading the same
//! [`GameState`](crate::game::GameState), and it does not know which of the two hosts it's under.
//!
//! # How the same brain runs in two places
//!
//! Inside the server, the engine fills each entity's `EntVars` and runs the bot's usercmd through
//! `SV_RunCmd`. Here, neither happens — so this module supplies both ends:
//!
//! ```text
//!   the server module                     the network client
//!   ─────────────────                     ──────────────────
//!   engine fills EntVars       ──▶        mirror writes EntVars from svc_playerinfo /
//!                                         svc_packetentities / stats
//!   engine answers traceline,  ──▶        NetHost answers from the map's own BSP
//!     pointcontents, cvars                (rtx-nav) and its own cvar store
//!   set_bot_cmd → SV_RunCmd    ──▶        cmd sink → clc_move on the wire
//!   server runs trigger touches──▶        the server does it for us (we're a real client)
//! ```
//!
//! Everything else — perception, goals, combat, steering, bhop, the navmesh — is untouched. The
//! trick that makes that possible is the [`ClientHost`](crate::host::ClientHost) seam plus a
//! discipline: **write network truth into exactly the fields the brain already reads**, rather than
//! teaching the brain a second way to ask.
//!
//! # What it will not know
//!
//! A server-side bot can read an enemy's health straight out of their entity. A client cannot —
//! that isn't on the wire, and no amount of parsing will put it there. The gap is filled by the
//! opponent model the bots already use for observation-gated estimates, which is the honest answer
//! and, not by coincidence, the one that makes them play like a player rather than a cheat.
//!
//! Status: the seam is in place and [`NetHost`] answers the game's questions locally. The session
//! (handshake, signon), the world mirror and the tick loop land next; see the plan's milestones.

pub(crate) mod host;

use std::path::PathBuf;

use crate::game::GameState;
use host::NetHost;

/// A bot client: the brain, hosted by [`NetHost`] instead of a server.
///
/// The session that drives it (handshake, signon, mirror, tick) is the next milestone; for now the
/// accessors exist to prove the seam holds together, and the crate's own tests use them.
#[allow(dead_code)]
pub struct Client {
    game: GameState,
    host: &'static NetHost,
}

#[allow(dead_code)]
impl Client {
    /// Build a client rooted at `basedir` (the directory holding `qw/`, `id1/`, …).
    ///
    /// The host is leaked deliberately: [`HostApi`](crate::host::HostApi) is `Copy` and is
    /// snapshotted throughout the bot code, so the reference it carries has to be `'static`. There
    /// is one host per process and it lives as long as the process, so `'static` is the truth
    /// rather than a workaround.
    pub fn new(basedir: PathBuf) -> Self {
        let host: &'static NetHost = Box::leak(Box::new(NetHost::new(basedir)));
        Client {
            game: GameState::new_client(host),
            host,
        }
    }

    /// The host, for the session to feed with what the server tells us.
    pub(crate) fn host(&self) -> &'static NetHost {
        self.host
    }

    /// The world the brain reads.
    pub(crate) fn game(&mut self) -> &mut GameState {
        &mut self.game
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The whole point of the seam: a `GameState` can exist with no engine behind it, and the game
    /// reads its tunables from the client's own store without noticing the difference.
    ///
    /// This is a small test for a large claim. Everything downstream — the mirror, the brain, the
    /// tick loop — assumes the game runs unmodified against a non-server host; if that were false,
    /// it would be false here first.
    #[test]
    fn builds_a_game_with_no_engine_behind_it() {
        let mut client = Client::new(PathBuf::from("/nonexistent"));

        assert!(client.game().host().is_client());
        // Tunables resolve through NetHost, seeded with the same defaults the server registers.
        assert_eq!(client.game().host().cvar(c"rtx_bot_skill"), 3.0);
        assert_eq!(client.game().host().cvar(c"rtx_bot_bhop"), 1.0);

        // And the server's physics come from the server, not from us.
        client.host().set_movevars(rtx_proto::svc::MoveVars {
            gravity: 640.0,
            ..Default::default()
        });
        assert_eq!(client.game().host().cvar(c"sv_gravity"), 640.0);
    }
}
