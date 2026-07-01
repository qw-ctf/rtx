// SPDX-License-Identifier: AGPL-3.0-or-later

//! Free-for-all deathmatch — the baseline mode. It is rtx's original, implicit behavior lifted
//! verbatim into the [`GameMode`](super::GameMode) abstraction: every hook is the trait default,
//! so selecting `ffa` changes nothing about how the game plays. It exists so that FFA is *a*
//! mode rather than *the* mode, and so `ra` (and everything after it) is a peer layered on top.

use super::GameMode;

/// The free-for-all deathmatch mode (`rtx_mode ffa`).
pub(crate) struct Ffa;

impl GameMode for Ffa {
    fn name(&self) -> &'static str {
        "ffa"
    }
}
