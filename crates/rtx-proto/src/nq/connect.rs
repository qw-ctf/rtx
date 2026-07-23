// SPDX-License-Identifier: AGPL-3.0-or-later

//! The NetQuake control handshake — `CCREQ_CONNECT` out, `CCREP_ACCEPT`/`CCREP_REJECT` back.
//!
//! Unlike QuakeWorld's text handshake, this is binary and framed like every other NetQuake packet:
//! a big-endian `[NETFLAG_CTL | length]` long (length counts the header), then the payload. A
//! control packet has **no sequence long** — that second header word only exists on in-band
//! ([`chan`](super::chan)) traffic.
//!
//! ```text
//! C→S  [CTL|len] CCREQ_CONNECT "QUAKE\0" 3  [1 35 0 <i32 password>]   ← ProQuake block
//! S→C  [CTL|len] CCREP_ACCEPT  <i32 port>   [<mod> <ver> <flags>]     ← optional ProQuake trailer
//!  or  [CTL|len] CCREP_REJECT  "<reason>\0"
//! ```
//!
//! We always send the ProQuake block (`mod == MOD_PROQUAKE`). It costs nothing on a server that
//! ignores it and, on a protocol-15 server that echoes it back, upgrades our client→server move
//! angles from 8-bit to 16-bit — the difference between 1.4°-quantized aim and precise aim. The
//! `port` in the accept is the data port to switch to, *unless* the server sets `PQF_IGNOREPORT`
//! (QuakeSpasm-Spiked and other single-socket servers do), in which case we keep talking to the
//! control port we already used.
//!
//! Ported from QuakeSpasm-Spiked `Quake/net_dgrm.c` (`_Datagram_Connect`) and `Quake/net_defs.h`.

use super::{NETFLAG_CTL, NETFLAG_LENGTH_MASK};
use crate::sizebuf::{Reader, Writer};

/// Control command bytes (`net_defs.h`).
const CCREQ_CONNECT: u8 = 0x01;
const CCREP_ACCEPT: u8 = 0x81;
const CCREP_REJECT: u8 = 0x82;

/// The control-connection protocol version — not the game protocol (`NET_PROTOCOL_VERSION`).
const NET_PROTOCOL_VERSION: u8 = 3;

/// ProQuake `mod` identifier: an engine that wants 16-bit client→server angles.
const MOD_PROQUAKE: u8 = 1;
/// Our advertised ProQuake mod version, matched to QuakeSpasm-Spiked's own.
const MOD_VERSION: u8 = 35;

/// The server demands a cheat-protected protocol we don't speak (`PQF_CHEATFREE`) — a hard reject.
const PQF_CHEATFREE: u8 = 0x01;
/// The server uses one socket for every client and wants us to keep our current port (`PQF_IGNOREPORT`).
const PQF_IGNOREPORT: u8 = 0x80;

/// The `CCREQ_CONNECT` packet that opens a NetQuake connection.
///
/// Sent to the server's control port; the reply is a [`Ccrep`]. Includes the ProQuake block with a
/// zero password (we never connect to password servers).
pub fn connect_request() -> Vec<u8> {
    let mut payload = Writer::new();
    payload.u8(CCREQ_CONNECT);
    payload.string("QUAKE"); // MSG_WriteString → "QUAKE\0"
    payload.u8(NET_PROTOCOL_VERSION);
    // ProQuake block: mod, mod version, flags, then a little-endian password long (0 = none).
    payload.u8(MOD_PROQUAKE);
    payload.u8(MOD_VERSION);
    payload.u8(0);
    payload.i32(0);
    frame_control(&payload.into_vec())
}

/// Wrap a control payload in its big-endian `[NETFLAG_CTL | length]` header. Length counts the
/// 4-byte header itself.
fn frame_control(payload: &[u8]) -> Vec<u8> {
    let len = (4 + payload.len()) as u32 & NETFLAG_LENGTH_MASK;
    let mut out = Writer::new();
    out.u32_be(NETFLAG_CTL | len);
    out.bytes(payload);
    out.into_vec()
}

/// A parsed control reply from the server.
#[derive(Clone, Debug, PartialEq)]
pub enum Ccrep {
    /// Connection accepted.
    Accept {
        /// The data port the server assigned. Combine with `ignore_port` via [`Accept::switch_to`].
        port: i32,
        /// The server echoed `mod == MOD_PROQUAKE`, so our move angles may be 16-bit.
        proquake: bool,
        /// The server wants us to keep our current port rather than switch to `port`.
        ignore_port: bool,
    },
    /// Connection refused, with the server's reason (a full server, a ban, a bad protocol).
    Reject(String),
    /// A control packet we recognise the framing of but don't act on.
    Other,
}

impl Ccrep {
    /// The port to send subsequent game traffic to, given the address we connected on used
    /// `current` as its port. `None` means "keep the current port" — both the `PQF_IGNOREPORT`
    /// single-socket case and a server that omitted the port (Quake Enhanced does).
    ///
    /// Only meaningful on [`Ccrep::Accept`]; other variants yield `None`.
    pub fn switch_to(&self) -> Option<u16> {
        match self {
            Ccrep::Accept { port, ignore_port, .. } if !ignore_port && *port != 0 => Some(*port as u16),
            _ => None,
        }
    }
}

/// Whether a datagram is a NetQuake **control** packet (`NETFLAG_CTL` exactly set), as opposed to
/// in-band traffic or a QuakeWorld-style `0xffffffff` out-of-band packet (whose high bits differ).
pub fn is_control(data: &[u8]) -> bool {
    data.len() >= 4 && {
        let ctl = u32::from_be_bytes([data[0], data[1], data[2], data[3]]);
        ctl & !NETFLAG_LENGTH_MASK == NETFLAG_CTL
    }
}

/// Parse a control reply. Returns `None` if the datagram isn't a well-formed control packet (wrong
/// flags, or a length that disagrees with the datagram) — the caller ignores it and waits for a
/// resend, exactly as the reference client does.
pub fn parse_control(data: &[u8]) -> Option<Ccrep> {
    if !is_control(data) {
        return None;
    }
    let mut r = Reader::new(data);
    let control = r.u32_be().ok()?;
    // The length field must match the datagram, or it's corrupt / a fragment we misframed.
    if (control & NETFLAG_LENGTH_MASK) as usize != data.len() {
        return None;
    }
    match r.u8().ok()? {
        CCREP_ACCEPT => {
            // Quake Enhanced omits the port; treat a missing one as 0 ("don't switch").
            let port = r.i32().unwrap_or(0);
            // Optional ProQuake trailer; each byte is present only if the server sent it.
            let mod_id = r.u8().unwrap_or(0);
            let _ver = r.u8().unwrap_or(0);
            let flags = r.u8().unwrap_or(0);
            let proquake = mod_id == MOD_PROQUAKE;
            if proquake && flags & PQF_CHEATFREE != 0 {
                return Some(Ccrep::Reject("server requires a cheat-protected protocol".into()));
            }
            let ignore_port = proquake && flags & PQF_IGNOREPORT != 0;
            Some(Ccrep::Accept {
                port,
                proquake,
                ignore_port,
            })
        }
        CCREP_REJECT => Some(Ccrep::Reject(r.string().ok()?)),
        _ => Some(Ccrep::Other),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The exact bytes of the connect request: a big-endian control header whose length counts
    /// itself, then `CCREQ_CONNECT`, `"QUAKE\0"`, the protocol version, and the ProQuake block.
    #[test]
    fn connect_request_bytes() {
        let pkt = connect_request();
        // Payload: 0x01, "QUAKE\0" (6), 3, 1, 35, 0, i32 0 (4) = 1+6+1+1+1+1+4 = 15; total = 19.
        assert_eq!(pkt.len(), 19);
        let header = u32::from_be_bytes([pkt[0], pkt[1], pkt[2], pkt[3]]);
        assert_eq!(header & !NETFLAG_LENGTH_MASK, NETFLAG_CTL);
        assert_eq!((header & NETFLAG_LENGTH_MASK) as usize, pkt.len());
        assert_eq!(
            &pkt[4..12],
            &[CCREQ_CONNECT, b'Q', b'U', b'A', b'K', b'E', 0, NET_PROTOCOL_VERSION]
        );
        assert_eq!(&pkt[12..15], &[MOD_PROQUAKE, MOD_VERSION, 0]);
        assert_eq!(&pkt[15..19], &0i32.to_le_bytes()); // password
    }

    /// Build a control reply the way a server does, so the parser is tested against the real framing.
    fn control(payload: &[u8]) -> Vec<u8> {
        super::frame_control(payload)
    }

    /// A single-socket server (QuakeSpasm-Spiked) accepts with its own port but `PQF_IGNOREPORT`, so
    /// we must *not* switch — the port stays whatever we already used.
    #[test]
    fn accept_with_ignore_port_keeps_current() {
        let mut p = vec![CCREP_ACCEPT];
        p.extend_from_slice(&26000i32.to_le_bytes());
        p.extend_from_slice(&[MOD_PROQUAKE, 30, PQF_IGNOREPORT]);
        let rep = parse_control(&control(&p)).unwrap();
        assert_eq!(
            rep,
            Ccrep::Accept {
                port: 26000,
                proquake: true,
                ignore_port: true
            }
        );
        assert_eq!(rep.switch_to(), None);
    }

    /// A vanilla NetQuake server assigns a fresh data port and sends no ProQuake trailer, so we
    /// switch to it and fall back to 8-bit angles.
    #[test]
    fn accept_vanilla_switches_port() {
        let mut p = vec![CCREP_ACCEPT];
        p.extend_from_slice(&26001i32.to_le_bytes());
        let rep = parse_control(&control(&p)).unwrap();
        assert_eq!(
            rep,
            Ccrep::Accept {
                port: 26001,
                proquake: false,
                ignore_port: false
            }
        );
        assert_eq!(rep.switch_to(), Some(26001));
    }

    /// A cheat-free server is unplayable for us; the parser turns it into a reject rather than a
    /// misleading accept.
    #[test]
    fn accept_cheatfree_is_a_reject() {
        let mut p = vec![CCREP_ACCEPT];
        p.extend_from_slice(&26000i32.to_le_bytes());
        p.extend_from_slice(&[MOD_PROQUAKE, 30, PQF_CHEATFREE]);
        assert!(matches!(parse_control(&control(&p)), Some(Ccrep::Reject(_))));
    }

    /// A rejection carries the reason the server wouldn't let us in.
    #[test]
    fn reject_carries_reason() {
        let mut p = vec![CCREP_REJECT];
        p.extend_from_slice(b"Server is full.\0");
        assert_eq!(
            parse_control(&control(&p)),
            Some(Ccrep::Reject("Server is full.".into()))
        );
    }

    /// Framing guards: a length that disagrees with the datagram, non-control flags, and a truncated
    /// header are all "not a control packet", not a panic.
    #[test]
    fn rejects_malformed_framing() {
        // Right flags, wrong length.
        let mut bad = control(&[CCREP_ACCEPT, 0, 0, 0, 0]);
        bad.push(0xff);
        assert_eq!(parse_control(&bad), None);
        // In-band flags, not control.
        let inband = (NETFLAG_DATA_STUB | 8).to_be_bytes();
        assert!(!is_control(&inband));
        assert_eq!(parse_control(&inband), None);
        // Too short to hold a header.
        assert_eq!(parse_control(&[0xff, 0xff]), None);
    }

    // A stand-in for NETFLAG_DATA so the test doesn't reach across modules for one constant.
    const NETFLAG_DATA_STUB: u32 = 0x0001_0000;
}
