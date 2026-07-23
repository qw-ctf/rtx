// SPDX-License-Identifier: AGPL-3.0-or-later

//! The client→server message stream: everything a bot says to the world.
//!
//! It's a small vocabulary — a move, a console command, a delta acknowledgement — but [`Move`] is
//! where a network client is most easily caught out, so three properties are worth stating up
//! front:
//!
//! **Every packet carries the last three moves, not one.** Moves are unreliable, and a dropped one
//! is a frame the server has to guess at. Re-sending the previous two costs a few bytes and makes
//! single- and double-drops invisible. The server ignores the ones it already ran.
//!
//! **`msec` is not a free parameter.** It's how long the move covers, and the server integrates it:
//! a client whose `msec` sum runs ahead of the wall clock is moving faster than real time, which is
//! precisely what a speed cheat looks like. mvdsv checks, and kicks. So `msec` must come from a
//! real clock — never from the bot's idea of a frame.
//!
//! **The checksum proves we're a client.** [`crc::block_sequence_crc_byte`](crate::crc) over the
//! move payload, keyed by the packet's sequence. Wrong, and the move is silently discarded.
//!
//! Ported from ezQuake's `src/cl_input.c` (`CL_SendCmd`) and `src/com_msg.c`
//! (`MSG_WriteDeltaUsercmd`).

use crate::crc::block_sequence_crc_byte;
use crate::sizebuf::{quantize_angle16, to_angle16, Writer};
use crate::svc::{cm, Usercmd};

/// `clc_*` opcodes.
pub mod op {
    pub const BAD: u8 = 0;
    pub const NOP: u8 = 1;
    pub const MOVE: u8 = 3;
    pub const STRINGCMD: u8 = 4;
    pub const DELTA: u8 = 5;
    pub const TMOVE: u8 = 6;
    pub const UPLOAD: u8 = 7;
}

/// Button bits in a usercmd.
pub mod button {
    /// Fire.
    pub const ATTACK: u8 = 1;
    /// Jump.
    pub const JUMP: u8 = 2;
    /// Use — vestigial in QuakeWorld, but the bit exists.
    pub const USE: u8 = 4;
}

/// The largest `msec` a client may claim for one move; beyond this the server clamps.
pub const MAX_MSEC: u8 = 250;

/// Move components are clamped to ±`127 * 4`. The odd-looking bound is id's: protocol 26 packed a
/// move into a `char` scaled by 4, and although 28 sends a full short, `MakeChar` still enforces
/// the old range so that what a modern client sends stays expressible in the old encoding.
const MAX_MOVE: i32 = 127 * 4;

/// Build a usercmd the way a client must send it.
///
/// Angles are [`quantize_angle16`]d, which is not cosmetic: the server sees the quantized value, so
/// a bot that aims with full precision and sends 16-bit is aiming somewhere slightly different from
/// where it thinks. Moves go through [`quantize_move`], so what we send is what a real client could
/// have sent.
pub fn make_usercmd(
    msec: u8,
    angles: glam::Vec3,
    forward: i16,
    side: i16,
    up: i16,
    buttons: u8,
    impulse: u8,
) -> Usercmd {
    Usercmd {
        msec: msec.min(MAX_MSEC),
        angles: glam::Vec3::new(
            quantize_angle16(angles.x),
            quantize_angle16(angles.y),
            quantize_angle16(angles.z),
        ),
        forward: quantize_move(forward),
        side: quantize_move(side),
        up: quantize_move(up),
        buttons,
        impulse,
    }
}

/// Quantize a move component the way id's `MakeChar` does: mask to a multiple of 4, *then* clamp.
///
/// The mask is bitwise, so it floors toward negative infinity rather than truncating toward zero —
/// `-401` becomes `-404`, not `-400`. Order matters too: masking after clamping would let ±508
/// through unrounded. Neither detail is arbitrary; both are what a real client's bytes look like.
fn quantize_move(v: i16) -> i16 {
    ((v as i32 & !3).clamp(-MAX_MOVE, MAX_MOVE)) as i16
}

/// Write one delta-encoded usercmd against `from`.
///
/// Only changed fields are sent, flagged by the `CM_*` bits — except `msec`, which always trails.
/// Returns the bits written, which is useful for tests and debugging.
pub fn write_delta_usercmd(w: &mut Writer, from: &Usercmd, cmd: &Usercmd) -> u8 {
    let mut bits = 0u8;
    if cmd.angles.x != from.angles.x {
        bits |= cm::ANGLE1;
    }
    if cmd.angles.y != from.angles.y {
        bits |= cm::ANGLE2;
    }
    if cmd.angles.z != from.angles.z {
        bits |= cm::ANGLE3;
    }
    if cmd.forward != from.forward {
        bits |= cm::FORWARD;
    }
    if cmd.side != from.side {
        bits |= cm::SIDE;
    }
    if cmd.up != from.up {
        bits |= cm::UP;
    }
    if cmd.buttons != from.buttons {
        bits |= cm::BUTTONS;
    }
    if cmd.impulse != from.impulse {
        bits |= cm::IMPULSE;
    }

    w.u8(bits);
    if bits & cm::ANGLE1 != 0 {
        w.u16(to_angle16(cmd.angles.x));
    }
    if bits & cm::ANGLE2 != 0 {
        w.u16(to_angle16(cmd.angles.y));
    }
    if bits & cm::ANGLE3 != 0 {
        w.u16(to_angle16(cmd.angles.z));
    }
    if bits & cm::FORWARD != 0 {
        w.i16(cmd.forward);
    }
    if bits & cm::SIDE != 0 {
        w.i16(cmd.side);
    }
    if bits & cm::UP != 0 {
        w.i16(cmd.up);
    }
    if bits & cm::BUTTONS != 0 {
        w.u8(cmd.buttons);
    }
    if bits & cm::IMPULSE != 0 {
        w.u8(cmd.impulse);
    }
    w.u8(cmd.msec); // always sent
    bits
}

/// A `clc_move` message: three moves, a loss report, and the checksum that authenticates them.
#[derive(Clone, Copy, Debug)]
pub struct Move {
    /// The move two frames ago.
    pub oldest: Usercmd,
    /// The move one frame ago.
    pub previous: Usercmd,
    /// This frame's move.
    pub current: Usercmd,
    /// Packet loss we're seeing, 0–100. The server uses it for its own diagnostics.
    pub loss: u8,
}

/// Build a `clc_move`, and optionally the `clc_delta` that follows it in the same packet.
///
/// `sequence` must be the netchan's **outgoing sequence for this packet** — the same number the
/// header will carry — because it keys the checksum. Off by one and every move is discarded.
///
/// `delta_sequence` is the last entity frame we successfully parsed; sending it asks the server to
/// compress the next update against that frame. `None` asks for a full update, which is what to do
/// after any parse failure — see [`PacketEntities`](crate::svc::PacketEntities).
pub fn write_move(m: &Move, sequence: u32, delta_sequence: Option<u8>) -> Vec<u8> {
    let mut w = Writer::new();
    w.u8(op::MOVE);

    // Reserve the checksum: it covers the bytes after it, which don't exist yet.
    let checksum_at = w.len();
    w.u8(0);
    w.u8(m.loss);

    // Each move deltas against the one before it; the oldest deltas against nothing.
    let null = Usercmd::default();
    write_delta_usercmd(&mut w, &null, &m.oldest);
    write_delta_usercmd(&mut w, &m.oldest, &m.previous);
    write_delta_usercmd(&mut w, &m.previous, &m.current);

    let checksum = block_sequence_crc_byte(&w.as_slice()[checksum_at + 1..], sequence);
    w.patch_u8(checksum_at, checksum);

    // The delta request rides in the same packet, after the move but outside its checksum.
    if let Some(seq) = delta_sequence {
        w.u8(op::DELTA);
        w.u8(seq);
    }
    w.into_vec()
}

/// Build a `clc_stringcmd` — a console command for the server: `new`, `prespawn`, `spawn`,
/// `begin`, `setinfo`, `say`, `kill`, and the rest.
pub fn write_stringcmd(cmd: &str) -> Vec<u8> {
    let mut w = Writer::new();
    w.u8(op::STRINGCMD);
    w.string(cmd);
    w.into_vec()
}

/// Build a bare `clc_delta`.
pub fn write_delta(sequence: u8) -> Vec<u8> {
    vec![op::DELTA, sequence]
}

/// Build a `clc_nop` — a packet with nothing to say, which still keeps the sequence advancing and
/// the connection alive.
pub fn write_nop() -> Vec<u8> {
    vec![op::NOP]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::ProtoState;
    use crate::sizebuf::Reader;
    use crate::svc::read_delta_usercmd;
    use glam::Vec3;

    fn cmd(msec: u8, yaw: f32, forward: i16, buttons: u8) -> Usercmd {
        make_usercmd(msec, Vec3::new(0.0, yaw, 0.0), forward, 0, 0, buttons, 0)
    }

    /// A bot aims with the angle the *server* will see. If `make_usercmd` didn't quantize, the bot
    /// would believe it was aiming at a value it never sent.
    #[test]
    fn usercmd_angles_are_quantized_to_what_the_wire_carries() {
        let c = make_usercmd(13, Vec3::new(11.111, 22.222, 33.333), 0, 0, 0, 0, 0);
        for (got, want) in [(c.angles.x, 11.111f32), (c.angles.y, 22.222), (c.angles.z, 33.333)] {
            assert!((got - want).abs() < 360.0 / 65536.0);
            assert_eq!(got, quantize_angle16(want), "must be exactly the wire value");
        }
    }

    /// Moves are multiples of 4 within ±508 — a real client can't express anything else, and a bot
    /// that sends 507 forward is identifiable as not-a-client from one packet.
    ///
    /// The rounding is a bitwise mask, so it floors: negative moves round *away* from zero, not
    /// toward it. Copying id here isn't pedantry — `-404` vs `-400` is the difference between our
    /// bytes and a real client's.
    #[test]
    fn move_components_are_quantized_and_clamped() {
        assert_eq!(quantize_move(0), 0);
        assert_eq!(quantize_move(400), 400);
        assert_eq!(quantize_move(401), 400);
        assert_eq!(quantize_move(403), 400);
        assert_eq!(quantize_move(-401), -404, "the mask floors toward negative infinity");
        assert_eq!(quantize_move(-400), -400);
        assert_eq!(quantize_move(508), 508);
        assert_eq!(quantize_move(32000), 508);
        assert_eq!(quantize_move(-32000), -508);

        // Every output is a multiple of 4 inside the range, for every input.
        for v in (i16::MIN..=i16::MAX).step_by(7) {
            let q = quantize_move(v);
            assert_eq!(q % 4, 0, "{v} -> {q}");
            assert!((-508..=508).contains(&q), "{v} -> {q}");
        }

        let c = make_usercmd(13, Vec3::ZERO, 9999, -9999, 3, 0, 0);
        assert_eq!((c.forward, c.side, c.up), (508, -508, 0));
    }

    /// `msec` is clamped, because the server clamps it too and a client that claims more time than
    /// it had is a client that moved faster than real time.
    #[test]
    fn msec_is_clamped() {
        assert_eq!(make_usercmd(255, Vec3::ZERO, 0, 0, 0, 0, 0).msec, MAX_MSEC);
        assert_eq!(make_usercmd(13, Vec3::ZERO, 0, 0, 0, 0, 0).msec, 13);
    }

    /// The delta encoding must round-trip through our own reader — the same reader that parses the
    /// server's copy of a player's usercmd out of `svc_playerinfo`.
    #[test]
    fn delta_usercmd_round_trips() {
        let from = cmd(13, 90.0, 0, 0);
        let to = make_usercmd(
            14,
            Vec3::new(-10.0, 95.0, 0.0),
            400,
            -200,
            100,
            button::ATTACK | button::JUMP,
            7,
        );

        let mut w = Writer::new();
        let bits = write_delta_usercmd(&mut w, &from, &to);
        assert_eq!(
            bits,
            cm::ANGLE1 | cm::ANGLE2 | cm::FORWARD | cm::SIDE | cm::UP | cm::BUTTONS | cm::IMPULSE
        );
        assert_eq!(bits & cm::ANGLE3, 0, "roll didn't change");

        let buf = w.into_vec();
        let mut r = Reader::new(&buf);
        let back = read_delta_usercmd(&mut r, &from).unwrap();
        assert_eq!(back, to);
        assert!(r.at_end(), "no bytes left over");
    }

    /// An unchanged field costs one bit and no bytes — that's the point of the encoding.
    #[test]
    fn delta_usercmd_omits_unchanged_fields() {
        let same = cmd(13, 90.0, 400, button::ATTACK);
        let mut w = Writer::new();
        let bits = write_delta_usercmd(&mut w, &same, &same);
        assert_eq!(bits, 0);
        assert_eq!(w.as_slice(), &[0, 13], "just the bits byte and msec");

        // msec is always sent even when identical — a move with no duration is meaningless.
        let mut w = Writer::new();
        write_delta_usercmd(&mut w, &same, &cmd(20, 90.0, 400, button::ATTACK));
        assert_eq!(w.as_slice(), &[0, 20]);
    }

    /// The whole `clc_move`: opcode, checksum, loss, three deltas — and every one of those moves
    /// must be recoverable, since that's exactly what the server does with them.
    #[test]
    fn move_packet_layout_and_round_trip() {
        let m = Move {
            oldest: cmd(13, 10.0, 0, 0),
            previous: cmd(14, 20.0, 400, 0),
            current: cmd(13, 30.0, 400, button::ATTACK),
            loss: 5,
        };
        let pkt = write_move(&m, 42, None);

        assert_eq!(pkt[0], op::MOVE);
        assert_eq!(pkt[2], 5, "loss byte");

        // The checksum covers everything after itself, keyed by the sequence.
        assert_eq!(pkt[1], block_sequence_crc_byte(&pkt[2..], 42));

        let mut r = Reader::new(&pkt[3..]);
        let null = Usercmd::default();
        let oldest = read_delta_usercmd(&mut r, &null).unwrap();
        let previous = read_delta_usercmd(&mut r, &oldest).unwrap();
        let current = read_delta_usercmd(&mut r, &previous).unwrap();
        assert_eq!(oldest, m.oldest);
        assert_eq!(previous, m.previous);
        assert_eq!(current, m.current);
        assert!(r.at_end());
    }

    /// The checksum is keyed by the packet's sequence, so the same move in a different packet is a
    /// different byte. Sending the netchan's *next* sequence instead of this packet's would fail
    /// every move silently — hence pinning it.
    #[test]
    fn move_checksum_is_keyed_by_sequence() {
        let m = Move {
            oldest: cmd(13, 0.0, 0, 0),
            previous: cmd(13, 0.0, 0, 0),
            current: cmd(13, 0.0, 0, 0),
            loss: 0,
        };
        let a = write_move(&m, 100, None);
        let b = write_move(&m, 101, None);
        assert_ne!(a[1], b[1], "checksum must depend on the sequence");
        assert_eq!(&a[2..], &b[2..], "…and nothing else changed");
    }

    /// `clc_delta` rides in the same packet, after the move, and outside its checksum.
    #[test]
    fn move_packet_appends_delta_request() {
        let m = Move {
            oldest: cmd(13, 0.0, 0, 0),
            previous: cmd(13, 0.0, 0, 0),
            current: cmd(13, 0.0, 0, 0),
            loss: 0,
        };
        let without = write_move(&m, 7, None);
        let with = write_move(&m, 7, Some(200));

        assert_eq!(&with[..without.len()], &without[..], "the move is byte-identical");
        assert_eq!(&with[without.len()..], &[op::DELTA, 200]);
        assert_eq!(with[1], without[1], "the delta request is outside the checksum");
    }

    /// The stringcmds that drive signon: opcode 4, then a NUL-terminated command.
    #[test]
    fn stringcmd_layout() {
        assert_eq!(write_stringcmd("new"), vec![op::STRINGCMD, b'n', b'e', b'w', 0]);
        let pkt = write_stringcmd("prespawn 5 0 12345");
        assert_eq!(pkt[0], op::STRINGCMD);
        assert_eq!(Reader::new(&pkt[1..]).string().unwrap(), "prespawn 5 0 12345");
    }

    /// A move packet must fit comfortably inside a datagram alongside the netchan header — this is
    /// the thing sent 72 times a second, so its size is the connection's baseline cost.
    #[test]
    fn move_packet_is_small() {
        let big = make_usercmd(13, Vec3::new(1.0, 2.0, 3.0), 400, 400, 400, 7, 9);
        let m = Move {
            oldest: big,
            previous: big,
            current: big,
            loss: 100,
        };
        let pkt = write_move(&m, 1, Some(0));
        assert!(pkt.len() < 64, "{} bytes", pkt.len());

        // And it survives the netchan, which is what actually goes on the wire.
        let mut chan = crate::netchan::Netchan::new(1);
        let datagram = chan.transmit(&pkt);
        assert!(datagram.len() < crate::protocol::MAX_MSGLEN);
    }

    /// End to end: a move built here, parsed back by the svc reader through a netchan, is what the
    /// server does. If these two halves ever disagree, this fails.
    #[test]
    fn move_survives_the_netchan_round_trip() {
        let m = Move {
            oldest: cmd(13, 10.0, 0, 0),
            previous: cmd(14, 20.0, 400, 0),
            current: cmd(13, 30.0, 400, button::JUMP),
            loss: 0,
        };

        let mut client = crate::netchan::Netchan::new(0xbeef);
        let seq = client.outgoing_sequence;
        let datagram = client.transmit(&write_move(&m, seq, None));

        // Server side: strip the header (two longs + qport), then read the move.
        let payload = &datagram[crate::netchan::HEADER_BYTES..];
        assert_eq!(payload[0], op::MOVE);
        assert_eq!(payload[1], block_sequence_crc_byte(&payload[2..], seq));

        let mut r = Reader::new(&payload[3..]);
        let null = Usercmd::default();
        let a = read_delta_usercmd(&mut r, &null).unwrap();
        let b = read_delta_usercmd(&mut r, &a).unwrap();
        let c = read_delta_usercmd(&mut r, &b).unwrap();
        assert_eq!(c.buttons, button::JUMP);
        assert!((c.angles.y - 30.0).abs() < 0.01);

        // The parser and the builder agree on the whole packet, with nothing left over.
        assert!(r.at_end());
        let _ = ProtoState::new();
    }
}
