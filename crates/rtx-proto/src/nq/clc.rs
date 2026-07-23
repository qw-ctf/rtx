// SPDX-License-Identifier: AGPL-3.0-or-later

//! The NetQuake client→server builder — much smaller than QuakeWorld's.
//!
//! A NetQuake `clc_move` has none of QuakeWorld's machinery: no `msec`, no block-sequence checksum,
//! no delta ack, no three-command backlog. It's a single command — the echoed server time (for the
//! server's ping calc), three view angles at the negotiated width, three move axes, a buttons byte
//! and an impulse byte — sent unreliably once per network frame. The server applies it to our edict
//! and runs the physics itself, so there is nothing to predict or checksum.
//!
//! Ported from QuakeSpasm-Spiked `Quake/cl_input.c` (`CL_SendMove`).

use super::protocol::NqProtoState;
use crate::sizebuf::Writer;
use glam::Vec3;

/// Client→server opcodes (`protocol.h`).
mod op {
    pub const DISCONNECT: u8 = 2;
    pub const MOVE: u8 = 3;
    pub const STRINGCMD: u8 = 4;
    pub const NOP: u8 = 1;
}

/// Button bits carried in `clc_move`.
pub mod button {
    /// `+attack`.
    pub const ATTACK: u8 = 1 << 0;
    /// `+jump`.
    pub const JUMP: u8 = 1 << 1;
}

/// Build a `clc_move`. `last_svc_time` is the most recent `svc_time` we heard, echoed so the server
/// can compute our ping; the angles go out at the width [`NqProtoState::write_move_angle`] chooses
/// (8-bit on a plain protocol-15 server, 16-bit otherwise). Sent unreliably.
#[allow(clippy::too_many_arguments)]
pub fn write_move(
    p: &NqProtoState,
    last_svc_time: f32,
    angles: Vec3,
    forward: i16,
    side: i16,
    up: i16,
    buttons: u8,
    impulse: u8,
) -> Vec<u8> {
    let mut w = Writer::new();
    w.u8(op::MOVE);
    w.u32(f32::to_bits(last_svc_time)); // MSG_WriteFloat, little-endian
    p.write_move_angle(&mut w, angles.x); // pitch
    p.write_move_angle(&mut w, angles.y); // yaw
    p.write_move_angle(&mut w, angles.z); // roll
    w.i16(forward);
    w.i16(side);
    w.i16(up);
    w.u8(buttons);
    w.u8(impulse);
    w.into_vec()
}

/// A `clc_stringcmd` — a console command for the server (`name`, `prespawn`, `spawn`, `begin`, …).
/// Sent reliably.
pub fn write_stringcmd(cmd: &str) -> Vec<u8> {
    let mut w = Writer::new();
    w.u8(op::STRINGCMD);
    w.string(cmd);
    w.into_vec()
}

/// A `clc_nop` — a keepalive / NAT-punch filler with no payload.
pub fn write_nop() -> Vec<u8> {
    vec![op::NOP]
}

/// A `clc_disconnect` — tell the server we're leaving.
pub fn write_disconnect() -> Vec<u8> {
    vec![op::DISCONNECT]
}

#[cfg(test)]
mod tests {
    use super::super::protocol::{NqProtoState, NETQUAKE};
    use super::*;
    use crate::sizebuf::Reader;

    /// A protocol-15 move without ProQuake: 8-bit angles, so the whole message is
    /// 1 + 4 + 3 + 6 + 1 + 1 = 16 bytes, and the echoed time round-trips.
    #[test]
    fn move_uses_8bit_angles_on_plain_proto15() {
        let p = NqProtoState {
            version: NETQUAKE,
            flags: 0,
            proquake_angles: false,
        };
        let pkt = write_move(&p, 1.5, Vec3::new(0.0, 90.0, 0.0), 400, 0, 0, button::ATTACK, 7);
        assert_eq!(pkt.len(), 16);
        let mut r = Reader::new(&pkt);
        assert_eq!(r.u8().unwrap(), op::MOVE);
        assert_eq!(f32::from_bits(r.u32().unwrap()), 1.5);
        // Three 8-bit angles: pitch 0, yaw 90 (=64), roll 0.
        assert_eq!(r.u8().unwrap(), 0);
        assert_eq!(r.u8().unwrap(), 64);
        assert_eq!(r.u8().unwrap(), 0);
        assert_eq!(r.i16().unwrap(), 400); // forward
        assert_eq!(r.i16().unwrap(), 0);
        assert_eq!(r.i16().unwrap(), 0);
        assert_eq!(r.u8().unwrap(), button::ATTACK);
        assert_eq!(r.u8().unwrap(), 7);
        assert!(r.at_end());
    }

    /// With ProQuake agreed, the same move uses 16-bit angles, growing the packet by three bytes.
    #[test]
    fn move_uses_16bit_angles_with_proquake() {
        let p = NqProtoState {
            version: NETQUAKE,
            flags: 0,
            proquake_angles: true,
        };
        let pkt = write_move(&p, 0.0, Vec3::new(0.0, 90.0, 0.0), 0, 0, 0, 0, 0);
        assert_eq!(pkt.len(), 19); // 16 + 3 for the wider angles
    }

    /// The reliable helpers are single-purpose and tiny, but the opcode bytes must be exact.
    #[test]
    fn stringcmd_and_control_bytes() {
        assert_eq!(
            write_stringcmd("begin"),
            [op::STRINGCMD, b'b', b'e', b'g', b'i', b'n', 0]
        );
        assert_eq!(write_nop(), [op::NOP]);
        assert_eq!(write_disconnect(), [op::DISCONNECT]);
    }
}
