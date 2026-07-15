// SPDX-License-Identifier: AGPL-3.0-or-later

//! The netchan — QuakeWorld's transport over UDP: sequencing, duplicate rejection, and a
//! one-message-deep reliable channel.
//!
//! It is much smaller than a real reliable transport, and the shape is worth understanding before
//! touching it. There is **one** reliable message in flight at a time, tracked by a **single bit**
//! that toggles per message. The remote echoes that bit back; if the echo doesn't match what we
//! sent, the message was lost and we resend the same bytes. That's the entire retransmit
//! mechanism — no windows, no selective ack, no fragmentation. It works because QuakeWorld's
//! reliable traffic is small and rare (console commands, signon), while the interesting data
//! (moves, entity snapshots) is unreliable by design: a dropped snapshot is stale by the time you
//! could resend it, so the next one supersedes it.
//!
//! Ported from ezQuake's `src/net_chan.c`, faithfully — including the off-by-one in
//! [`last_reliable_sequence`](Netchan::last_reliable_sequence), which id assigns *after* bumping
//! the outgoing sequence, so a resend waits one extra packet. That's the behaviour real servers
//! have interoperated with for decades; "fixing" it would only make us the odd one out.

use crate::protocol::MAX_MSGLEN;
use crate::sizebuf::Reader;

/// Header size: two sequence longs, plus the qport short on client→server packets.
pub const HEADER_BYTES: usize = 8 + 2;

/// One end of a QuakeWorld connection.
///
/// Client-side only: we always write the qport and never read one.
#[derive(Clone, Debug)]
pub struct Netchan {
    /// Our port identifier, echoed in every packet so the server can follow us across a NAT
    /// rebinding — the address may change mid-game, the qport doesn't.
    pub qport: u16,
    /// Sequence of the next packet we send.
    pub outgoing_sequence: u32,
    /// Highest sequence we've received.
    pub incoming_sequence: u32,
    /// The last outgoing sequence the remote confirmed seeing — the basis of round-trip timing.
    pub incoming_acknowledged: u32,
    /// How many packets went missing before the one we just processed.
    pub dropped: u32,
    /// Total packets lost over the connection's life.
    pub drop_count: u32,

    /// The remote's echo of our reliable bit.
    incoming_reliable_acknowledged: u32,
    /// Our echo of the remote's reliable bit.
    incoming_reliable_sequence: u32,
    /// The bit identifying our in-flight reliable message; toggles each new one.
    reliable_sequence: u32,
    /// The outgoing sequence at which the in-flight reliable message was last sent.
    last_reliable_sequence: u32,

    /// Reliable bytes queued but not yet sent.
    message: Vec<u8>,
    /// The in-flight reliable message, held until acked so it can be resent verbatim.
    reliable_buf: Vec<u8>,
}

impl Netchan {
    /// Open a channel with the given qport.
    pub fn new(qport: u16) -> Self {
        Netchan {
            qport,
            outgoing_sequence: 1,
            incoming_sequence: 0,
            incoming_acknowledged: 0,
            dropped: 0,
            drop_count: 0,
            incoming_reliable_acknowledged: 0,
            incoming_reliable_sequence: 0,
            reliable_sequence: 0,
            last_reliable_sequence: 0,
            message: Vec::new(),
            reliable_buf: Vec::new(),
        }
    }

    /// Whether new reliable data can be queued. False while a message is in flight — with only one
    /// slot, the caller must hold its next command until this one is acked.
    pub fn can_reliable(&self) -> bool {
        self.reliable_buf.is_empty()
    }

    /// Queue reliable bytes (a `clc_stringcmd`, typically) for the next [`transmit`](Self::transmit).
    ///
    /// Appends to the pending buffer, which is only promoted to "in flight" once the previous
    /// reliable message is acked — so it's safe to call while [`can_reliable`](Self::can_reliable)
    /// is false; the bytes just wait their turn.
    pub fn queue_reliable(&mut self, data: &[u8]) {
        self.message.extend_from_slice(data);
    }

    /// Whether any reliable data is pending or in flight.
    pub fn reliable_pending(&self) -> bool {
        !self.message.is_empty() || !self.reliable_buf.is_empty()
    }

    /// Build the next datagram: header, the reliable message if one needs (re)sending, then as much
    /// of `unreliable` as fits.
    ///
    /// The unreliable part is **dropped silently if it doesn't fit** — id's behaviour, and
    /// harmless: the next move packet supersedes it a frame later.
    pub fn transmit(&mut self, unreliable: &[u8]) -> Vec<u8> {
        // Resend if the remote's echo shows our in-flight reliable never landed.
        let mut send_reliable = self.incoming_acknowledged > self.last_reliable_sequence
            && self.incoming_reliable_acknowledged != self.reliable_sequence;

        // Promote queued bytes to in-flight, if the slot is free.
        if self.reliable_buf.is_empty() && !self.message.is_empty() {
            self.reliable_buf = std::mem::take(&mut self.message);
            self.reliable_sequence ^= 1;
            send_reliable = true;
        }

        let mut out = Vec::with_capacity(HEADER_BYTES + unreliable.len());
        let w1 = self.outgoing_sequence | (send_reliable as u32) << 31;
        let w2 = self.incoming_sequence | self.incoming_reliable_sequence << 31;
        self.outgoing_sequence += 1;

        out.extend_from_slice(&w1.to_le_bytes());
        out.extend_from_slice(&w2.to_le_bytes());
        out.extend_from_slice(&self.qport.to_le_bytes());

        if send_reliable {
            out.extend_from_slice(&self.reliable_buf);
            // Assigned after the bump above — see the module note.
            self.last_reliable_sequence = self.outgoing_sequence;
        }
        if out.len() + unreliable.len() <= MAX_MSGLEN {
            out.extend_from_slice(unreliable);
        }
        out
    }

    /// Consume a datagram's header and hand back the payload, or `None` if the packet is stale, a
    /// duplicate, or too short to hold a header.
    ///
    /// Also updates the ack state, which is what lets [`transmit`](Self::transmit) know whether a
    /// reliable message needs resending.
    pub fn process<'a>(&mut self, data: &'a [u8]) -> Option<&'a [u8]> {
        let mut r = Reader::new(data);
        let sequence = r.u32().ok()?;
        let sequence_ack = r.u32().ok()?;

        let reliable_message = sequence >> 31;
        let reliable_ack = sequence_ack >> 31;
        let sequence = sequence & !(1 << 31);
        let sequence_ack = sequence_ack & !(1 << 31);

        // Stale or duplicated: the sequence must strictly advance. UDP reorders freely, and an old
        // snapshot is worse than none.
        if sequence <= self.incoming_sequence {
            return None;
        }

        self.dropped = sequence - (self.incoming_sequence + 1);
        if self.dropped > 0 {
            self.drop_count += 1;
        }

        // Our reliable message landed — free the slot for the next one.
        if reliable_ack == self.reliable_sequence {
            self.reliable_buf.clear();
        }

        self.incoming_sequence = sequence;
        self.incoming_acknowledged = sequence_ack;
        self.incoming_reliable_acknowledged = reliable_ack;
        if reliable_message != 0 {
            self.incoming_reliable_sequence ^= 1;
        }

        Some(&data[r.pos()..])
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A minimal stand-in for the other end: acks whatever it last saw, and echoes the reliable
    /// bit the way a real server does.
    #[derive(Default)]
    struct Peer {
        seq: u32,
        seen_reliable_bit: u32,
        acked: u32,
        /// Reliable payloads delivered, in order, with duplicates included.
        received: Vec<Vec<u8>>,
    }

    impl Peer {
        /// Take a client packet: record the reliable payload (if the header says there is one) and
        /// remember what to ack.
        fn recv(&mut self, pkt: &[u8], reliable_len: usize) {
            let w1 = u32::from_le_bytes(pkt[0..4].try_into().unwrap());
            let has_reliable = w1 >> 31;
            self.acked = w1 & !(1 << 31);
            if has_reliable != 0 {
                self.seen_reliable_bit ^= 1;
                self.received.push(pkt[HEADER_BYTES..HEADER_BYTES + reliable_len].to_vec());
            }
        }

        /// Build a server→client packet acking what we've seen.
        fn send(&mut self) -> Vec<u8> {
            self.seq += 1;
            let mut out = Vec::new();
            out.extend_from_slice(&self.seq.to_le_bytes());
            out.extend_from_slice(&(self.acked | self.seen_reliable_bit << 31).to_le_bytes());
            out.extend_from_slice(b"payload");
            out
        }
    }

    /// The header layout, byte for byte: two little-endian sequence longs then the qport. A server
    /// reads these at fixed offsets, so the shape is not negotiable.
    #[test]
    fn header_layout_is_two_longs_and_a_qport() {
        let mut c = Netchan::new(0x1234);
        let pkt = c.transmit(b"move");

        assert_eq!(u32::from_le_bytes(pkt[0..4].try_into().unwrap()), 1); // outgoing sequence
        assert_eq!(u32::from_le_bytes(pkt[4..8].try_into().unwrap()), 0); // nothing seen yet
        assert_eq!(u16::from_le_bytes(pkt[8..10].try_into().unwrap()), 0x1234);
        assert_eq!(&pkt[HEADER_BYTES..], b"move");
        assert_eq!(c.outgoing_sequence, 2);
    }

    /// Sequences must strictly advance: a replayed or reordered packet is dropped rather than
    /// applied on top of newer state.
    #[test]
    fn rejects_stale_and_duplicate_packets() {
        let mut c = Netchan::new(1);
        let mut s = Peer::default();

        let p1 = s.send();
        assert_eq!(c.process(&p1), Some(&b"payload"[..]));
        assert_eq!(c.incoming_sequence, 1);

        // The same packet again, and an older one: both refused, state untouched.
        assert_eq!(c.process(&p1), None);
        assert_eq!(c.incoming_sequence, 1);

        let p2 = s.send();
        let p3 = s.send();
        assert!(c.process(&p3).is_some());
        assert_eq!(c.process(&p2), None, "packet 2 arriving after 3 is stale");
        assert_eq!(c.incoming_sequence, 3);
    }

    /// A gap in the sequence is counted but doesn't stop the packet being used — the newest
    /// snapshot is always worth having.
    #[test]
    fn counts_dropped_packets_without_discarding_the_new_one() {
        let mut c = Netchan::new(1);
        let mut s = Peer::default();

        assert!(c.process(&s.send()).is_some());
        s.seq += 3; // three packets vanish
        assert!(c.process(&s.send()).is_some());
        assert_eq!(c.dropped, 3);
        assert_eq!(c.drop_count, 1);
    }

    /// A truncated datagram is a network event, not a panic.
    #[test]
    fn ignores_runt_packets() {
        let mut c = Netchan::new(1);
        assert_eq!(c.process(&[]), None);
        assert_eq!(c.process(&[0u8; 7]), None);
    }

    /// The happy path for the reliable channel: one message goes out once, gets acked, and the
    /// slot frees for the next.
    ///
    /// Note when the slot is actually occupied: queueing doesn't take it — `transmit` does, when it
    /// promotes the bytes to in-flight. That's id's `Netchan_CanReliable`, which tests the
    /// in-flight buffer and not the pending one.
    #[test]
    fn reliable_message_is_sent_once_and_acked() {
        let mut c = Netchan::new(1);
        let mut s = Peer::default();

        c.queue_reliable(b"new");
        assert!(c.can_reliable(), "queueing alone doesn't occupy the in-flight slot");
        assert!(c.reliable_pending());

        let pkt = c.transmit(b"");
        assert!(!c.can_reliable(), "now it's in flight, and stays so until acked");
        s.recv(&pkt, 3);
        assert_eq!(s.received, vec![b"new".to_vec()]);

        // Server acks; the slot frees and the message is not resent.
        c.process(&s.send());
        assert!(c.can_reliable());
        assert!(!c.reliable_pending());
        let pkt = c.transmit(b"");
        s.recv(&pkt, 3);
        assert_eq!(s.received.len(), 1, "acked message must not be resent");
    }

    /// The core of the mechanism: if the server never sees the reliable message, the *identical*
    /// bytes go again under a fresh sequence. Without this, a lost `new` would hang signon forever.
    ///
    /// The resend is not immediate. id compares the remote's ack against `last_reliable_sequence`,
    /// which it assigns *after* bumping the outgoing sequence — so the ack has to reach one past
    /// the packet that followed the reliable one. In practice that costs a single extra packet
    /// (~14 ms at 72 Hz), and matching it keeps our retransmit timing identical to every other
    /// client the server has ever seen.
    #[test]
    fn resends_reliable_message_when_the_ack_shows_it_was_lost() {
        let mut c = Netchan::new(1);
        let mut s = Peer::default();

        c.queue_reliable(b"prespawn");
        let lost = c.transmit(b"");
        assert_eq!(lost[0..4], (1u32 | 1 << 31).to_le_bytes(), "sent as packet 1, reliable bit set");

        // The packet never arrives. The server acks later packets of ours, still echoing the
        // *stale* reliable bit (0) — that mismatch is what tells us it was lost.
        s.acked = 2;
        c.process(&s.send());
        let held = c.transmit(b"");
        assert_eq!(held[0..4], 2u32.to_le_bytes(), "no resend yet — ack hasn't passed the mark");

        s.acked = 4;
        c.process(&s.send());
        let resent = c.transmit(b"");
        assert_eq!(resent[0..4], (3u32 | 1 << 31).to_le_bytes(), "resent as packet 3");

        s.recv(&resent, 8);
        assert_eq!(s.received, vec![b"prespawn".to_vec()], "same bytes, new sequence");
    }

    /// Reliable data queued while a message is in flight waits its turn, then goes as one — it is
    /// never interleaved into the in-flight message or dropped.
    #[test]
    fn queues_behind_an_in_flight_message() {
        let mut c = Netchan::new(1);
        let mut s = Peer::default();

        c.queue_reliable(b"aaa");
        let pkt = c.transmit(b"");
        s.recv(&pkt, 3);

        // Queued while the first is still in flight.
        c.queue_reliable(b"bbb");
        let pkt = c.transmit(b"");
        assert_eq!(pkt.len(), HEADER_BYTES, "nothing new goes out until the first is acked");
        let _ = pkt;

        c.process(&s.send()); // ack of "aaa"
        let pkt = c.transmit(b"");
        s.recv(&pkt, 3);
        assert_eq!(s.received, vec![b"aaa".to_vec(), b"bbb".to_vec()]);
    }

    /// The reliable bit alternates per message; that single bit is the whole retransmit protocol,
    /// so it must actually flip.
    #[test]
    fn reliable_bit_toggles_between_messages() {
        let mut c = Netchan::new(1);
        let mut s = Peer::default();
        let mut bits = Vec::new();

        for msg in [&b"one"[..], b"two", b"six"] {
            c.queue_reliable(msg);
            let pkt = c.transmit(b"");
            bits.push(c.reliable_sequence);
            s.recv(&pkt, 3);
            c.process(&s.send());
        }
        assert_eq!(bits, vec![1, 0, 1]);
        assert_eq!(s.received, vec![b"one".to_vec(), b"two".to_vec(), b"six".to_vec()]);
    }

    /// The reliable part is written before the unreliable part, and an unreliable payload that
    /// won't fit is dropped rather than truncated — a half-written usercmd would fail its checksum
    /// and desync the move stream.
    #[test]
    fn oversized_unreliable_payload_is_dropped_whole() {
        let mut c = Netchan::new(1);
        c.queue_reliable(b"reliable");
        let huge = vec![0xaa; MAX_MSGLEN];
        let pkt = c.transmit(&huge);
        assert_eq!(pkt.len(), HEADER_BYTES + 8, "reliable kept, oversized unreliable dropped");
        assert_eq!(&pkt[HEADER_BYTES..], b"reliable");

        let pkt = c.transmit(b"small");
        assert!(pkt.ends_with(b"small"));
    }
}
