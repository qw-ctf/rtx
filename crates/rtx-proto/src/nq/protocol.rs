// SPDX-License-Identifier: AGPL-3.0-or-later

//! NetQuake protocol versions and the coord/angle widths each one implies.
//!
//! Where QuakeWorld negotiates a single FLOATCOORDS bit, NetQuake keys the wire widths off a
//! protocol *version* the server announces in `svc_serverinfo`, and — for `PROTOCOL_RMQ` (999) — a
//! `protocolflags` word of independent [`prfl`] bits. [`NqProtoState`] carries the version and flags
//! and owns the [`coord`](NqProtoState::coord)/[`angle`](NqProtoState::angle) reads, so the parser
//! never branches on width itself.
//!
//! This is the NetQuake analogue of [`ProtoState`](crate::protocol::ProtoState): same job (source of
//! truth for the negotiated widths), different source (a version int, not FTE masks). It stays a
//! separate type because the width *sources* don't overlap — 24-bit coords are short+byte and
//! `INT32COORD` is a long, neither of which fits the QW reader's `coord_bytes: 2|4` field.
//!
//! Formulas are ported verbatim from QuakeSpasm-Spiked `Quake/common.c` (`MSG_ReadCoord`,
//! `MSG_ReadAngle`, and the `MSG_Write*` twins).

use crate::sizebuf::{Reader, Result, Writer};
use glam::Vec3;

/// Default NetQuake server port (`net.h` `DEFAULTnet_hostport`).
pub const PORT: u16 = 26000;

/// Vanilla NetQuake (`PROTOCOL_NETQUAKE`): 16-bit coords, 8-bit entity angles, byte model/frame.
pub const NETQUAKE: u32 = 15;
/// FitzQuake (`PROTOCOL_FITZQUAKE`): adds `U_EXTEND` bits — alpha, large model/frame — but the same
/// coord/angle widths as vanilla (`protocolflags` is 0).
pub const FITZQUAKE: u32 = 666;
/// RMQ (`PROTOCOL_RMQ`): FitzQuake plus a `protocolflags` word selecting coord/angle precision.
pub const RMQ: u32 = 999;

/// `protocolflags` bits, meaningful only under [`RMQ`] (`protocol.h` `PRFL_*`). Absent (flags 0),
/// coords are 16-bit ⅛-unit and angles are 8-bit — the vanilla widths.
pub mod prfl {
    /// Entity angles are 16-bit (`360/65536` turn) rather than 8-bit.
    pub const SHORTANGLE: u32 = 1 << 1;
    /// Angles are raw `f32`.
    pub const FLOATANGLE: u32 = 1 << 2;
    /// Coords are 16.8 fixed point: a short plus a `/255` byte fraction.
    pub const BIT24COORD: u32 = 1 << 3;
    /// Coords are raw `f32`.
    pub const FLOATCOORD: u32 = 1 << 4;
    /// Edicts carry a scale byte. Parsed and dropped.
    pub const EDICTSCALE: u32 = 1 << 5;
    /// Coords are a 32-bit `/16` fixed point.
    pub const INT32COORD: u32 = 1 << 7;
}

/// The negotiated NetQuake wire state — everything a coord/angle read depends on.
///
/// `proquake_angles` is not a width in `protocolflags`: it's the ProQuake handshake result that
/// bumps *client→server* move angles to 16 bits on a protocol-15 server (server→client widths are
/// unaffected). It lives here because [`write_move_angle`](Self::write_move_angle) needs it.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct NqProtoState {
    /// 15, 666 or 999, from `svc_serverinfo`.
    pub version: u32,
    /// `protocolflags`, read only when `version == 999`; 0 otherwise.
    pub flags: u32,
    /// The server echoed `mod == MOD_PROQUAKE` in `CCREP_ACCEPT`, so our move angles are 16-bit.
    pub proquake_angles: bool,
}

impl NqProtoState {
    /// A fresh state at vanilla widths, before any `svc_serverinfo`.
    pub fn new() -> Self {
        NqProtoState {
            version: NETQUAKE,
            flags: 0,
            proquake_angles: false,
        }
    }

    /// Adopt the version and flags from a parsed `svc_serverinfo`. Flags are only meaningful for
    /// [`RMQ`]; callers pass 0 for 15/666.
    pub fn set(&mut self, version: u32, flags: u32) {
        self.version = version;
        self.flags = if version == RMQ { flags } else { 0 };
    }

    /// One coordinate, at the negotiated width. Priority mirrors `MSG_ReadCoord`: float, then
    /// 32-bit, then 24-bit, else the vanilla 16-bit ⅛ unit.
    pub fn coord(&self, r: &mut Reader) -> Result<f32> {
        if self.flags & prfl::FLOATCOORD != 0 {
            r.f32()
        } else if self.flags & prfl::INT32COORD != 0 {
            Ok(r.i32()? as f32 * (1.0 / 16.0))
        } else if self.flags & prfl::BIT24COORD != 0 {
            // 16.8 fixed point: a signed short whole part and an unsigned /255 fraction.
            let whole = r.i16()? as f32;
            let frac = r.u8()? as f32 * (1.0 / 255.0);
            Ok(whole + frac)
        } else {
            Ok(r.i16()? as f32 * (1.0 / 8.0))
        }
    }

    /// Three coords, x/y/z.
    pub fn coord3(&self, r: &mut Reader) -> Result<Vec3> {
        Ok(Vec3::new(self.coord(r)?, self.coord(r)?, self.coord(r)?))
    }

    /// One angle in degrees, at the negotiated width: float, 16-bit, else vanilla 8-bit.
    pub fn angle(&self, r: &mut Reader) -> Result<f32> {
        if self.flags & prfl::FLOATANGLE != 0 {
            r.f32()
        } else if self.flags & prfl::SHORTANGLE != 0 {
            Ok(r.i16()? as f32 * (360.0 / 65536.0))
        } else {
            Ok(r.i8()? as f32 * (360.0 / 256.0))
        }
    }

    /// Three angles, pitch/yaw/roll.
    pub fn angle3(&self, r: &mut Reader) -> Result<Vec3> {
        Ok(Vec3::new(self.angle(r)?, self.angle(r)?, self.angle(r)?))
    }

    /// Write one `clc_move` view angle at the width the server expects for *our* moves.
    ///
    /// This is not the same rule as [`angle`](Self::angle): server→client angles follow
    /// `protocolflags`, but client→server move angles are 8-bit only on a plain protocol-15 server
    /// that didn't agree the ProQuake hack. Everything else — 666, 999, or 15-with-ProQuake — sends
    /// 16-bit (or float, under `FLOATANGLE`). Mirrors `CL_SendMove`'s angle branch.
    pub fn write_move_angle(&self, w: &mut Writer, degrees: f32) {
        let eight_bit = self.version == NETQUAKE
            && !self.proquake_angles
            && self.flags & (prfl::SHORTANGLE | prfl::FLOATANGLE) == 0;
        if self.flags & prfl::FLOATANGLE != 0 && !eight_bit {
            w.u32(f32::to_bits(degrees));
        } else if eight_bit {
            w.u8(to_angle8(degrees));
        } else {
            w.angle16(degrees);
        }
    }
}

/// Encode an angle in degrees as NetQuake's 8-bit fixed-point turn (`MSG_WriteAngle`, vanilla path).
pub fn to_angle8(degrees: f32) -> u8 {
    let scaled = degrees * (256.0 / 360.0);
    let rounded = if scaled >= 0.0 { scaled + 0.5 } else { scaled - 0.5 };
    (rounded as i32 & 0xff) as u8
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sizebuf::Writer;

    fn read_coord(state: &NqProtoState, bytes: &[u8]) -> f32 {
        state.coord(&mut Reader::new(bytes)).unwrap()
    }

    /// Vanilla 15/666 (flags 0): coord is a signed short over 8, angle a signed byte turn — the
    /// widths a stock server serves. Getting these wrong desyncs from the first entity.
    #[test]
    fn vanilla_widths() {
        let p = NqProtoState {
            version: NETQUAKE,
            flags: 0,
            proquake_angles: false,
        };
        assert_eq!(read_coord(&p, &64i16.to_le_bytes()), 8.0);
        assert_eq!(read_coord(&p, &(-64i16).to_le_bytes()), -8.0);
        assert_eq!(p.angle(&mut Reader::new(&[64])).unwrap(), 90.0);
        assert_eq!(p.angle(&mut Reader::new(&[128])).unwrap(), -180.0);
    }

    /// RMQ's `protocolflags` select each width independently. QuakeSpasm's own default for a
    /// pext-capable server is `FLOATCOORD | SHORTANGLE`, so that pairing must be exact.
    #[test]
    fn rmq_flag_widths() {
        let float_short = NqProtoState {
            version: RMQ,
            flags: prfl::FLOATCOORD | prfl::SHORTANGLE,
            proquake_angles: false,
        };
        assert_eq!(read_coord(&float_short, &123.5f32.to_le_bytes()), 123.5);
        assert_eq!(
            float_short.angle(&mut Reader::new(&16384i16.to_le_bytes())).unwrap(),
            90.0
        );

        // 24-bit is short-plus-byte-fraction; 32-bit is a /16 long.
        let c24 = NqProtoState {
            version: RMQ,
            flags: prfl::BIT24COORD,
            proquake_angles: false,
        };
        let mut b = Vec::new();
        b.extend_from_slice(&100i16.to_le_bytes());
        b.push(128); // 128/255 ≈ 0.502
        assert!((read_coord(&c24, &b) - 100.502).abs() < 0.01);

        let c32 = NqProtoState {
            version: RMQ,
            flags: prfl::INT32COORD,
            proquake_angles: false,
        };
        assert_eq!(read_coord(&c32, &(160i32).to_le_bytes()), 10.0);

        let fa = NqProtoState {
            version: RMQ,
            flags: prfl::FLOATANGLE,
            proquake_angles: false,
        };
        assert_eq!(fa.angle(&mut Reader::new(&270.0f32.to_le_bytes())).unwrap(), 270.0);
    }

    /// `set` only keeps `protocolflags` for RMQ; a 666 server that somehow left flag bits lying
    /// around must still read at vanilla widths.
    #[test]
    fn set_ignores_flags_below_rmq() {
        let mut p = NqProtoState::new();
        p.set(FITZQUAKE, prfl::FLOATCOORD);
        assert_eq!(p.flags, 0);
        p.set(RMQ, prfl::FLOATCOORD);
        assert_eq!(p.flags, prfl::FLOATCOORD);
    }

    /// The move-angle width is a different rule from the read width: 8-bit only on plain proto-15,
    /// 16-bit once ProQuake is agreed or the protocol is 666/999.
    #[test]
    fn move_angle_width_follows_proquake_and_version() {
        // Plain 15: 8-bit, one byte on the wire.
        let plain15 = NqProtoState {
            version: NETQUAKE,
            flags: 0,
            proquake_angles: false,
        };
        let mut w = Writer::new();
        plain15.write_move_angle(&mut w, 90.0);
        assert_eq!(w.len(), 1);
        assert_eq!(w.as_slice()[0], 64);

        // 15 with ProQuake: 16-bit.
        let pq15 = NqProtoState {
            version: NETQUAKE,
            flags: 0,
            proquake_angles: true,
        };
        let mut w = Writer::new();
        pq15.write_move_angle(&mut w, 90.0);
        assert_eq!(w.len(), 2);

        // 666: 16-bit even without ProQuake.
        let fitz = NqProtoState {
            version: FITZQUAKE,
            flags: 0,
            proquake_angles: false,
        };
        let mut w = Writer::new();
        fitz.write_move_angle(&mut w, 90.0);
        assert_eq!(w.len(), 2);

        // 999 + FLOATANGLE: a raw float.
        let rmqf = NqProtoState {
            version: RMQ,
            flags: prfl::FLOATANGLE,
            proquake_angles: false,
        };
        let mut w = Writer::new();
        rmqf.write_move_angle(&mut w, 90.0);
        assert_eq!(w.len(), 4);
    }
}
