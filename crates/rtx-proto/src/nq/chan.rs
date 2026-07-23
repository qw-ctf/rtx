// SPDX-License-Identifier: AGPL-3.0-or-later

//! The NetQuake "Datagram" transport — two streams over one UDP socket, framed with a big-endian
//! `[flags|len][sequence]` header.
//!
//! This is nothing like QuakeWorld's [`Netchan`](crate::netchan). There is no `qport` and no
//! reliable bit folded into a shared sequence. Instead:
//!
//! - **Unreliable** ([`NETFLAG_UNRELIABLE`]) packets carry the per-frame game snapshot. They have
//!   their own monotonically increasing sequence; the receiver drops any that arrive out of order
//!   and counts the gap as loss. Nothing is acked or resent.
//! - **Reliable** ([`NETFLAG_DATA`]) packets carry signon and stringcmd traffic. They are a
//!   *separate* stop-and-wait stream: a message is split into fragments of at most
//!   [`RELIABLE_FRAGMENT`] bytes, only one fragment is ever in flight, the receiver **acks every
//!   fragment it sees** (even a duplicate), and the last fragment is flagged [`NETFLAG_EOM`]. The
//!   sender retransmits the in-flight fragment on a timer the caller owns.
//!
//! Like [`Netchan`](crate::netchan) this is IO-free and clock-free: [`process`](NqChan::process)
//! decodes one datagram and hands back what to parse plus any ack to put on the wire, and the caller
//! does the reading, writing and retransmit timing. Ported from QuakeSpasm-Spiked
//! `Quake/net_dgrm.c` (`Datagram_SendMessage`/`SendUnreliableMessage`/`ProcessPacket`).

use super::{HEADER_BYTES, NETFLAG_ACK, NETFLAG_CTL, NETFLAG_DATA, NETFLAG_EOM};
use super::{NETFLAG_LENGTH_MASK, NETFLAG_UNRELIABLE};
use crate::sizebuf::Writer;
use std::collections::VecDeque;

/// The largest reliable fragment we send. NetQuake's default `max_datagram`; our reliables (signon
/// stringcmds) are all far smaller, so in practice we never fragment — but the machinery is here for
/// a server that sends us a large reliable to reassemble.
pub const RELIABLE_FRAGMENT: usize = 1024;

/// The largest unreliable payload for a non-local client (`DATAGRAM_MTU`). We never approach it —
/// our moves are a few dozen bytes — but oversize inbound unreliables are dropped whole.
pub const MTU: usize = 1400;

/// What a decoded datagram delivered to the caller.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Incoming {
    /// Nothing to parse (an ack, a stale/duplicate packet, or a non-final reliable fragment).
    None,
    /// A complete unreliable message — the per-frame game data.
    Unreliable(Vec<u8>),
    /// A fully reassembled reliable message.
    Reliable(Vec<u8>),
}

/// The reliable fragment currently in flight (or ready to send).
#[derive(Clone, Copy, Debug)]
struct Active {
    seq: u32,
    off: usize,
    len: usize,
    eom: bool,
}

/// One end of a NetQuake Datagram connection: the client's, but symmetric enough to drive a test
/// peer.
#[derive(Debug, Default)]
pub struct NqChan {
    // Unreliable stream.
    unreliable_send_seq: u32,
    unreliable_recv_seq: u32,
    // Reliable send: whole messages queued, one fragmented message current, one fragment active.
    send_seq: u32,
    pending: VecDeque<Vec<u8>>,
    cur: Vec<u8>,
    cur_off: usize,
    active: Option<Active>,
    active_sent: bool,
    // Reliable receive: sequence expected next, and the reassembly buffer.
    recv_seq: u32,
    recv_msg: Vec<u8>,
}

impl NqChan {
    /// A fresh channel, all sequences at zero — the state right after `CCREP_ACCEPT`.
    pub fn new() -> Self {
        NqChan::default()
    }

    /// Build an in-band datagram header: the big-endian `[flags | length]` long (length counts the
    /// header) followed by the big-endian sequence.
    fn header(w: &mut Writer, flags: u32, payload_len: usize, seq: u32) {
        let total = (HEADER_BYTES + payload_len) as u32 & NETFLAG_LENGTH_MASK;
        w.u32_be(flags | total);
        w.u32_be(seq);
    }

    /// Frame an unreliable payload (a `clc_move` or `clc_nop`) for transmission. Consumes one
    /// unreliable sequence number.
    pub fn transmit_unreliable(&mut self, payload: &[u8]) -> Vec<u8> {
        let mut w = Writer::new();
        Self::header(&mut w, NETFLAG_UNRELIABLE, payload.len(), self.unreliable_send_seq);
        self.unreliable_send_seq += 1;
        w.bytes(payload);
        w.into_vec()
    }

    /// Queue a whole reliable message (a signon stringcmd). It is sent fragment by fragment as the
    /// stream frees up; call [`reliable_to_send`](Self::reliable_to_send) to pull the bytes.
    pub fn queue_reliable(&mut self, data: &[u8]) {
        self.pending.push_back(data.to_vec());
        self.promote();
    }

    /// Whether any reliable message is queued, in flight, or mid-fragment — i.e. whether we are
    /// still waiting to hand a reliable off completely.
    pub fn reliable_pending(&self) -> bool {
        self.active.is_some() || !self.cur.is_empty() || !self.pending.is_empty()
    }

    /// The next reliable fragment that has **not yet been transmitted**, if one is ready. Returns
    /// `None` while a fragment is in flight awaiting its ack (stop-and-wait) or when nothing is
    /// queued. Marks the fragment sent, so a second call returns `None` until an ack advances us.
    pub fn reliable_to_send(&mut self) -> Option<Vec<u8>> {
        if self.active.is_some() && !self.active_sent {
            self.active_sent = true;
            return Some(self.frame_active());
        }
        None
    }

    /// The in-flight reliable fragment again, for the caller's retransmit timer. `None` if nothing is
    /// awaiting an ack. Reuses the same sequence, exactly as `ReSendMessage` does.
    pub fn reliable_resend(&mut self) -> Option<Vec<u8>> {
        if self.active.is_some() && self.active_sent {
            Some(self.frame_active())
        } else {
            None
        }
    }

    /// Decode one received datagram. Returns what to parse (if anything) and an ack to transmit (if
    /// the datagram was a reliable fragment — one is produced for *every* fragment, including
    /// duplicates, which is what keeps the sender unstuck under loss).
    pub fn process(&mut self, data: &[u8]) -> (Incoming, Option<Vec<u8>>) {
        if data.len() < HEADER_BYTES {
            return (Incoming::None, None);
        }
        let raw_len = u32::from_be_bytes([data[0], data[1], data[2], data[3]]);
        let seq = u32::from_be_bytes([data[4], data[5], data[6], data[7]]);
        let flags = raw_len & !NETFLAG_LENGTH_MASK;
        let len = (raw_len & NETFLAG_LENGTH_MASK) as usize;

        if flags & NETFLAG_CTL != 0 {
            // Control packets are the handshake's business, not the in-band stream's.
            return (Incoming::None, None);
        }
        // Trust the header's length, but never read past the datagram we actually got.
        let payload_len = len.saturating_sub(HEADER_BYTES).min(data.len() - HEADER_BYTES);
        let payload = &data[HEADER_BYTES..HEADER_BYTES + payload_len];

        if flags & NETFLAG_UNRELIABLE != 0 {
            if seq < self.unreliable_recv_seq {
                return (Incoming::None, None); // stale — a newer frame already superseded it
            }
            self.unreliable_recv_seq = seq + 1;
            (Incoming::Unreliable(payload.to_vec()), None)
        } else if flags & NETFLAG_ACK != 0 {
            self.handle_ack(seq);
            (Incoming::None, None)
        } else if flags & NETFLAG_DATA != 0 {
            let ack = self.build_ack(seq);
            if seq != self.recv_seq {
                return (Incoming::None, Some(ack)); // duplicate — re-ack but don't re-deliver
            }
            self.recv_seq += 1;
            if flags & NETFLAG_EOM != 0 {
                let mut msg = std::mem::take(&mut self.recv_msg);
                msg.extend_from_slice(payload);
                (Incoming::Reliable(msg), Some(ack))
            } else {
                self.recv_msg.extend_from_slice(payload);
                (Incoming::None, Some(ack))
            }
        } else {
            (Incoming::None, None) // unknown flags
        }
    }

    /// Apply an ack for the in-flight fragment, advancing to the next fragment or completing the
    /// message. A stale or duplicate ack (one not for the fragment we're waiting on) is ignored.
    fn handle_ack(&mut self, seq: u32) {
        let Some(active) = self.active else { return };
        if !self.active_sent || seq != active.seq {
            return;
        }
        self.cur_off += active.len;
        self.active = None;
        if self.cur_off < self.cur.len() {
            self.build_active();
        } else {
            self.cur.clear();
            self.cur_off = 0;
            self.promote();
        }
    }

    /// Promote the next queued message into the current slot if nothing is in flight.
    fn promote(&mut self) {
        if self.active.is_none() && self.cur.is_empty() {
            if let Some(next) = self.pending.pop_front() {
                self.cur = next;
                self.cur_off = 0;
                self.build_active();
            }
        }
    }

    /// Compute the fragment starting at `cur_off`, assigning it the next sequence.
    fn build_active(&mut self) {
        let remaining = self.cur.len() - self.cur_off;
        let len = remaining.min(RELIABLE_FRAGMENT);
        let seq = self.send_seq;
        self.send_seq += 1;
        self.active = Some(Active {
            seq,
            off: self.cur_off,
            len,
            eom: len == remaining,
        });
        self.active_sent = false;
    }

    /// Frame the active fragment for the wire.
    fn frame_active(&self) -> Vec<u8> {
        let f = self.active.expect("frame_active with no active fragment");
        let eom = if f.eom { NETFLAG_EOM } else { 0 };
        let mut w = Writer::new();
        Self::header(&mut w, NETFLAG_DATA | eom, f.len, f.seq);
        w.bytes(&self.cur[f.off..f.off + f.len]);
        w.into_vec()
    }

    /// An 8-byte ack packet echoing a received reliable fragment's sequence.
    fn build_ack(&self, seq: u32) -> Vec<u8> {
        let mut w = Writer::new();
        Self::header(&mut w, NETFLAG_ACK, 0, seq);
        w.into_vec()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A single small reliable: one fragment out, one ack back, delivered whole. This is the shape
    /// of every signon stringcmd we send.
    #[test]
    fn single_fragment_reliable_round_trips() {
        let mut client = NqChan::new();
        let mut server = NqChan::new();

        client.queue_reliable(b"prespawn");
        let frag = client.reliable_to_send().expect("a fragment to send");
        assert!(client.reliable_to_send().is_none(), "stop-and-wait: only one in flight");

        let (incoming, ack) = server.process(&frag);
        assert_eq!(incoming, Incoming::Reliable(b"prespawn".to_vec()));
        let ack = ack.expect("server acks the fragment");

        let (_, no_ack) = client.process(&ack);
        assert!(no_ack.is_none());
        assert!(!client.reliable_pending(), "acked message is complete");
    }

    /// A message larger than a fragment is split, and each fragment only goes out once the previous
    /// is acked — never two in flight at once. The receiver reassembles on the EOM fragment.
    #[test]
    fn multi_fragment_reassembles_in_order() {
        let mut client = NqChan::new();
        let mut server = NqChan::new();

        let big: Vec<u8> = (0..(RELIABLE_FRAGMENT * 2 + 300)).map(|i| i as u8).collect();
        client.queue_reliable(&big);

        let mut delivered = None;
        for _ in 0..5 {
            let Some(frag) = client.reliable_to_send() else { break };
            let (incoming, ack) = server.process(&frag);
            if let Incoming::Reliable(msg) = incoming {
                delivered = Some(msg);
            }
            // Deliver the ack so the next fragment can go.
            client.process(&ack.expect("each fragment is acked"));
        }
        assert_eq!(delivered, Some(big));
        assert!(!client.reliable_pending());
    }

    /// A lost fragment: the client resends the *same* fragment (same sequence) on its timer, and the
    /// server accepts the retransmit. Nothing advances until the ack arrives.
    #[test]
    fn lost_fragment_is_resent_with_same_sequence() {
        let mut client = NqChan::new();
        let mut server = NqChan::new();

        client.queue_reliable(b"begin");
        let first = client.reliable_to_send().unwrap();
        // Pretend `first` was lost: the server never sees it. The client's timer fires → resend.
        let resend = client.reliable_resend().unwrap();
        assert_eq!(first, resend, "resend reuses the fragment and its sequence");

        let (incoming, ack) = server.process(&resend);
        assert_eq!(incoming, Incoming::Reliable(b"begin".to_vec()));
        client.process(&ack.unwrap());
        assert!(!client.reliable_pending());
    }

    /// A duplicated reliable fragment (the ack was lost, so the sender resent) must be re-acked but
    /// not re-delivered — otherwise the signon stream would process "begin" twice.
    #[test]
    fn duplicate_data_is_reacked_not_redelivered() {
        let mut client = NqChan::new();
        let mut server = NqChan::new();
        client.queue_reliable(b"spawn");
        let frag = client.reliable_to_send().unwrap();

        let (first, ack1) = server.process(&frag);
        assert_eq!(first, Incoming::Reliable(b"spawn".to_vec()));
        assert!(ack1.is_some());

        // Same fragment again: the server acks it (so the sender unsticks) but delivers nothing.
        let (second, ack2) = server.process(&frag);
        assert_eq!(second, Incoming::None);
        assert!(ack2.is_some(), "every DATA fragment is acked, even duplicates");
    }

    /// A stale ack (for a fragment already acked, or one we never sent) leaves the in-flight state
    /// untouched rather than falsely completing the message.
    #[test]
    fn stale_ack_is_ignored() {
        let mut client = NqChan::new();
        client.queue_reliable(b"name \"bot\"");
        let _frag = client.reliable_to_send().unwrap();

        // An ack for sequence 7 — we're waiting on sequence 0.
        let mut w = Writer::new();
        NqChan::header(&mut w, NETFLAG_ACK, 0, 7);
        client.process(&w.into_vec());
        assert!(client.reliable_pending(), "the real fragment is still outstanding");
    }

    /// Unreliable packets are sequenced-drop: an older one arriving after a newer is discarded, and
    /// a gap is tolerated (the frame just moves on). This is what stops a late move-frame from
    /// rewinding the world.
    #[test]
    fn unreliable_drops_stale_and_tolerates_gaps() {
        let mut a = NqChan::new();
        let mut b = NqChan::new();

        let p0 = a.transmit_unreliable(b"frame0");
        let p1 = a.transmit_unreliable(b"frame1");
        let p2 = a.transmit_unreliable(b"frame2");

        // In order: 0 then 2 (1 was lost) — both accepted, gap tolerated.
        assert_eq!(b.process(&p0).0, Incoming::Unreliable(b"frame0".to_vec()));
        assert_eq!(b.process(&p2).0, Incoming::Unreliable(b"frame2".to_vec()));
        // The late frame1 is now stale and dropped.
        assert_eq!(b.process(&p1).0, Incoming::None);
    }

    /// Interleaving: an unreliable frame arriving between a reliable fragment and its ack must not
    /// disturb the reliable stream.
    #[test]
    fn unreliable_between_reliable_fragments() {
        let mut client = NqChan::new();
        let mut server = NqChan::new();

        let big: Vec<u8> = (0..(RELIABLE_FRAGMENT + 10)).map(|i| i as u8).collect();
        client.queue_reliable(&big);
        let frag0 = client.reliable_to_send().unwrap();
        let (_, ack0) = server.process(&frag0);

        // A game frame arrives before the client processes the ack.
        let move_frame = client.transmit_unreliable(b"move");
        assert_eq!(server.process(&move_frame).0, Incoming::Unreliable(b"move".to_vec()));

        // Ack advances the reliable to its final fragment.
        client.process(&ack0.unwrap());
        let frag1 = client.reliable_to_send().expect("second fragment after ack");
        let (incoming, _) = server.process(&frag1);
        assert_eq!(incoming, Incoming::Reliable(big));
    }

    /// A control packet reaching the in-band decoder is ignored, not misparsed as game data.
    #[test]
    fn control_flags_are_not_in_band() {
        let mut chan = NqChan::new();
        let mut w = Writer::new();
        w.u32_be(NETFLAG_CTL | 8);
        w.u32_be(0);
        assert_eq!(chan.process(&w.into_vec()), (Incoming::None, None));
    }
}
