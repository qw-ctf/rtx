// SPDX-License-Identifier: AGPL-3.0-or-later

//! Fetching a map the machine doesn't have.
//!
//! Most public servers run maps no local install carries — the one the probe hit runs `pkeg1` — and
//! a client without the map is dropped at `prespawn` (the checksum it would send is a checksum of
//! nothing). A real client downloads it; so does this one, over HTTP from the community's map
//! repository, before it finishes signing on.
//!
//! # Why a thread, not a blocking call
//!
//! The signon is client-driven: the server has said `cmd prespawn` and is *waiting for us*, while the
//! tick loop's per-frame nops keep the netchan alive and stop the connection timing out. A blocking
//! fetch inside the poll would freeze those nops (and, in a squad, every other bot's) for the length
//! of the download. So the fetch runs on its own thread and the session polls a channel — staying
//! responsive, and letting a squad keep playing while one member waits for a map.
//!
//! # Scope
//!
//! HTTP only. The in-protocol download — the stock `svc_download` chunks, and the FTE
//! `CHUNKEDDOWNLOADS` the probe shows servers offering us — is deliberately not here yet; this covers
//! the common case a headless bot needs to reach a public server.

use std::path::PathBuf;
use std::sync::mpsc::{self, Receiver};

/// The community's primary map source — direct `.bsp` files, the URL shape most servers point real
/// clients at. See <https://maps.quakeworld.nu/>.
const MAP_REPO: &str = "https://maps.quakeworld.nu/all";

/// How long to give one download before giving up. Generous — a big map on a slow link — but bounded,
/// so a dead mirror doesn't hang the bot at the scoreboard forever.
const TIMEOUT: std::time::Duration = std::time::Duration::from_secs(60);

/// A download in flight: the channel its result arrives on.
pub(crate) struct Download {
    rx: Receiver<Result<PathBuf, String>>,
}

impl Download {
    /// Start fetching `<map>.bsp` into `<basedir>/<gamedir>/maps/`, in the background.
    ///
    /// Returns immediately; the result — the written path, or why it failed — arrives on the channel
    /// [`poll`](Self::poll) reads. The destination is where a real client puts a download, so the next
    /// `find` picks it up with no special-casing.
    pub(crate) fn start(basedir: PathBuf, gamedir: String, map: String) -> Download {
        let (tx, rx) = mpsc::channel();
        std::thread::spawn(move || {
            let _ = tx.send(fetch(&basedir, &gamedir, &map));
        });
        Download { rx }
    }

    /// Has the download finished? `None` while it's still running, then the written path or the
    /// failure once — the channel yields a single result.
    pub(crate) fn poll(&self) -> Option<Result<PathBuf, String>> {
        match self.rx.try_recv() {
            Ok(result) => Some(result),
            Err(mpsc::TryRecvError::Empty) => None,
            // The worker died without sending — treat as a failure rather than hang.
            Err(mpsc::TryRecvError::Disconnected) => Some(Err("download worker vanished".to_string())),
        }
    }
}

/// Fetch and install one map, synchronously (this is the thread body).
fn fetch(basedir: &std::path::Path, gamedir: &str, map: &str) -> Result<PathBuf, String> {
    let url = format!("{MAP_REPO}/{map}.bsp");
    let dir = basedir.join(if gamedir.is_empty() { "qw" } else { gamedir }).join("maps");
    std::fs::create_dir_all(&dir).map_err(|e| format!("can't make {}: {e}", dir.display()))?;
    let dest = dir.join(format!("{map}.bsp"));

    let bytes = http_get(&url)?;
    if !is_bsp(&bytes) {
        // The repository answers a missing map with an HTML 404 page, not an error status — so the
        // header check is what actually rejects it, before it can be parsed or checksummed.
        return Err(format!("{url} did not return a Quake BSP (got {} bytes)", bytes.len()));
    }
    // Write beside the destination and rename into place, so a partial or failed write never leaves a
    // half-written file that a later run would trust as the map.
    let tmp = dir.join(format!(".{map}.bsp.part"));
    std::fs::write(&tmp, &bytes).map_err(|e| format!("can't write {}: {e}", tmp.display()))?;
    std::fs::rename(&tmp, &dest).map_err(|e| format!("can't install {}: {e}", dest.display()))?;
    Ok(dest)
}

/// GET `url` and return the body. A non-success status (a 5xx, a refused connection) is an error
/// here; a 404 that returns a body still succeeds and is rejected by the header check on the bytes.
fn http_get(url: &str) -> Result<Vec<u8>, String> {
    let client = reqwest::blocking::Client::builder()
        .timeout(TIMEOUT)
        .build()
        .map_err(|e| format!("couldn't build the http client: {e}"))?;
    let resp = client.get(url).send().map_err(|e| format!("fetching {url}: {e}"))?;
    let resp = resp.error_for_status().map_err(|e| format!("fetching {url}: {e}"))?;
    resp.bytes()
        .map(|b| b.to_vec())
        .map_err(|e| format!("reading {url}: {e}"))
}

/// Whether these bytes begin with a Quake BSP header (version 29). The one check that rejects a 404
/// HTML page saved under a `.bsp` name — which would otherwise be handed to the parser and, worse,
/// checksummed and sent to the server.
fn is_bsp(bytes: &[u8]) -> bool {
    bytes.len() >= 4 && i32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]) == 29
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The validation that stands between a 404 page and the map parser: a real BSP starts with
    /// version 29, and anything else — an HTML error body, an empty file — is not a map.
    #[test]
    fn only_a_real_bsp_header_is_accepted() {
        assert!(is_bsp(&29i32.to_le_bytes()), "version 29 is the Quake BSP");
        assert!(!is_bsp(b"<!DOCTYPE html><title>404"), "a 404 page is not a map");
        assert!(!is_bsp(b""), "nor an empty file");
        assert!(!is_bsp(&30i32.to_le_bytes()), "nor a version we don't read");
    }
}
