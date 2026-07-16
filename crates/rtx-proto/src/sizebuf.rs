// SPDX-License-Identifier: AGPL-3.0-or-later

//! Message buffers — id's `sizebuf_t` / `MSG_Read*` / `MSG_Write*`, as a [`Reader`] and a
//! [`Writer`].
//!
//! Two things here are load-bearing and easy to get wrong:
//!
//! **Coord and angle widths are negotiated, not fixed.** With `FTE_PEXT_FLOATCOORDS` a coord is a
//! 4-byte `f32` and an angle is a 2-byte fixed-point turn; without it they're 2 and 1. Nothing in
//! the byte stream announces which — the reader has to already know, from `svc_serverdata`. Get it
//! wrong and every byte after the first coord is garbage, usually surfacing as a bogus svc opcode
//! several messages later. So the width lives on the [`Reader`] itself ([`Reader::with_widths`]),
//! sourced from [`ProtoState`](crate::protocol::ProtoState), and there is no global to forget.
//!
//! **Reads fail, they don't panic.** A truncated or corrupt datagram is a network event, not a bug:
//! every read returns [`Result`] and a short buffer yields [`Underflow`]. The client drops the
//! packet and moves on.

use crate::protocol::ProtoState;

/// A read ran off the end of the buffer. The offset is where the read was attempted, which is the
/// useful thing to print next to a hexdump when a parse goes wrong.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Underflow {
    /// Read position at the time of the failure.
    pub at: usize,
    /// How many bytes the read wanted.
    pub want: usize,
    /// How many bytes were left.
    pub have: usize,
}

impl std::fmt::Display for Underflow {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "message underflow at {}: wanted {} byte(s), {} left", self.at, self.want, self.have)
    }
}

impl std::error::Error for Underflow {}

/// Shorthand for a read result.
pub type Result<T> = core::result::Result<T, Underflow>;

/// A little-endian message reader over a borrowed datagram.
///
/// Every QuakeWorld scalar is little-endian; the only interesting types are the two whose width the
/// protocol negotiates ([`coord`](Self::coord), [`angle`](Self::angle)) and the string, which has a
/// quirk worth knowing about (see [`string`](Self::string)).
pub struct Reader<'a> {
    buf: &'a [u8],
    pos: usize,
    /// Negotiated coord width in bytes: 2 (fixed-point ⅛ unit) or 4 (`f32`).
    pub coord_bytes: u8,
    /// Negotiated angle width in bytes: 1 (1/256 turn) or 2 (1/65536 turn).
    pub angle_bytes: u8,
}

impl<'a> Reader<'a> {
    /// A reader at vanilla widths (coord 2, angle 1) — correct before `svc_serverdata` has been
    /// parsed, and for any server that doesn't negotiate `FTE_PEXT_FLOATCOORDS`.
    pub fn new(buf: &'a [u8]) -> Self {
        Reader { buf, pos: 0, coord_bytes: 2, angle_bytes: 1 }
    }

    /// A reader at the negotiated widths. This is the constructor the client uses for everything
    /// after signon; [`new`](Self::new) is for the handshake, before there's a negotiation.
    pub fn with_widths(buf: &'a [u8], p: &ProtoState) -> Self {
        Reader {
            buf,
            pos: 0,
            coord_bytes: p.coord_bytes,
            angle_bytes: p.angle_bytes,
        }
    }

    /// Current read offset.
    pub fn pos(&self) -> usize {
        self.pos
    }

    /// Bytes not yet consumed.
    pub fn remaining(&self) -> usize {
        self.buf.len() - self.pos
    }

    /// Whether the whole buffer has been consumed.
    pub fn at_end(&self) -> bool {
        self.pos >= self.buf.len()
    }

    /// The full underlying buffer, for hexdumping a message that failed to parse.
    pub fn buf(&self) -> &'a [u8] {
        self.buf
    }

    /// Rewind (or jump) to an absolute offset. The entity-delta reader needs this: under
    /// `FTE_PEXT_ENTITYDBL` the entity *number* depends on bits stored after it, so the loop peeks
    /// ahead and then rewinds to parse the delta properly.
    pub fn seek(&mut self, pos: usize) {
        self.pos = pos.min(self.buf.len());
    }

    /// Take `n` raw bytes.
    pub fn bytes(&mut self, n: usize) -> Result<&'a [u8]> {
        if self.remaining() < n {
            return Err(Underflow { at: self.pos, want: n, have: self.remaining() });
        }
        let out = &self.buf[self.pos..self.pos + n];
        self.pos += n;
        Ok(out)
    }

    /// Skip `n` bytes — for a message we recognise but don't care about (`svc_download` payloads,
    /// voice chat).
    pub fn skip(&mut self, n: usize) -> Result<()> {
        self.bytes(n).map(|_| ())
    }

    /// `MSG_ReadByte`.
    pub fn u8(&mut self) -> Result<u8> {
        Ok(self.bytes(1)?[0])
    }

    /// `MSG_ReadChar` — a *signed* byte. The distinction matters: `svc_temp_entity` counts are
    /// unsigned, vanilla angles are signed.
    pub fn i8(&mut self) -> Result<i8> {
        Ok(self.u8()? as i8)
    }

    /// An unsigned 16-bit word.
    pub fn u16(&mut self) -> Result<u16> {
        let b = self.bytes(2)?;
        Ok(u16::from_le_bytes([b[0], b[1]]))
    }

    /// `MSG_ReadShort` — a signed 16-bit word.
    pub fn i16(&mut self) -> Result<i16> {
        Ok(self.u16()? as i16)
    }

    /// `MSG_ReadLong` — a signed 32-bit word.
    pub fn i32(&mut self) -> Result<i32> {
        let b = self.bytes(4)?;
        Ok(i32::from_le_bytes([b[0], b[1], b[2], b[3]]))
    }

    /// An unsigned 32-bit word — the extension masks in the handshake.
    pub fn u32(&mut self) -> Result<u32> {
        Ok(self.i32()? as u32)
    }

    /// A **big-endian** unsigned 32-bit word. QuakeWorld is little-endian everywhere, but NetQuake's
    /// Datagram transport frames every packet with a big-endian `[flags|len][sequence]` header (id's
    /// `BigLong`). Only the 8-byte header is big-endian; the payload after it is ordinary LE.
    pub fn u32_be(&mut self) -> Result<u32> {
        let b = self.bytes(4)?;
        Ok(u32::from_be_bytes([b[0], b[1], b[2], b[3]]))
    }

    /// `MSG_ReadFloat` — a 32-bit IEEE float.
    pub fn f32(&mut self) -> Result<f32> {
        Ok(f32::from_bits(self.i32()? as u32))
    }

    /// `MSG_ReadString` — a NUL-terminated string.
    ///
    /// Two id quirks, both reproduced because the byte stream depends on them: bytes equal to 255
    /// are **skipped** rather than stored (an old anti-exploit against clients that treated them as
    /// terminators), and the string ends at a NUL *or* at the end of the buffer.
    ///
    /// Decoded as latin-1, so each byte maps to one `char` and nothing is lost. QuakeWorld's
    /// character set puts "coloured" text in the high half, and callers that care (obituary
    /// matching) strip the high bit themselves.
    pub fn string(&mut self) -> Result<String> {
        let mut out = String::new();
        loop {
            if self.at_end() {
                break; // unterminated at end of message — id stops here too
            }
            let c = self.u8()?;
            if c == 255 {
                continue;
            }
            if c == 0 {
                break;
            }
            out.push(c as char);
        }
        Ok(out)
    }

    /// A coordinate, at the negotiated width: 2 bytes of ⅛-unit fixed point (the vanilla ±4096
    /// map limit) or a raw 4-byte float.
    pub fn coord(&mut self) -> Result<f32> {
        match self.coord_bytes {
            4 => self.f32(),
            _ => Ok(self.i16()? as f32 * (1.0 / 8.0)),
        }
    }

    /// A coordinate that is always a raw float, regardless of the negotiated width — MVDSV's
    /// `MVD_PEXT1_FLOATCOORDS` floats entity and player origins without widening anything else.
    pub fn float_coord(&mut self) -> Result<f32> {
        self.f32()
    }

    /// Three coords in x, y, z order.
    pub fn coord3(&mut self) -> Result<glam::Vec3> {
        let x = self.coord()?;
        let y = self.coord()?;
        let z = self.coord()?;
        Ok(glam::Vec3::new(x, y, z))
    }

    /// An angle in degrees, at the negotiated width: a signed byte of 1/256 turn, or a signed
    /// short of 1/65536 turn.
    pub fn angle(&mut self) -> Result<f32> {
        match self.angle_bytes {
            2 => self.angle16(),
            _ => Ok(self.i8()? as f32 * (360.0 / 256.0)),
        }
    }

    /// A 16-bit angle in degrees, regardless of the negotiated width — usercmds always use this
    /// precision, negotiation or not.
    pub fn angle16(&mut self) -> Result<f32> {
        Ok(self.i16()? as f32 * (360.0 / 65536.0))
    }

    /// Three angles in pitch, yaw, roll order.
    pub fn angle3(&mut self) -> Result<glam::Vec3> {
        let x = self.angle()?;
        let y = self.angle()?;
        let z = self.angle()?;
        Ok(glam::Vec3::new(x, y, z))
    }
}

/// A little-endian message writer.
///
/// Only the client→server direction needs one, which is a much smaller surface than reading: a
/// handful of scalars, `clc_move`'s delta-encoded usercmds, and strings.
#[derive(Clone, Debug, Default)]
pub struct Writer {
    buf: Vec<u8>,
}

impl Writer {
    /// An empty writer.
    pub fn new() -> Self {
        Writer { buf: Vec::new() }
    }

    /// The bytes written so far.
    pub fn as_slice(&self) -> &[u8] {
        &self.buf
    }

    /// Consume the writer for its bytes.
    pub fn into_vec(self) -> Vec<u8> {
        self.buf
    }

    /// Bytes written so far.
    pub fn len(&self) -> usize {
        self.buf.len()
    }

    /// Whether nothing has been written.
    pub fn is_empty(&self) -> bool {
        self.buf.is_empty()
    }

    /// Drop everything written.
    pub fn clear(&mut self) {
        self.buf.clear();
    }

    /// Overwrite one already-written byte. `clc_move` needs it: the checksum covers the bytes that
    /// follow it, so the byte is reserved first and filled in once the payload exists.
    pub fn patch_u8(&mut self, at: usize, v: u8) {
        self.buf[at] = v;
    }

    /// Append a byte.
    pub fn u8(&mut self, v: u8) {
        self.buf.push(v);
    }

    /// Append raw bytes.
    pub fn bytes(&mut self, v: &[u8]) {
        self.buf.extend_from_slice(v);
    }

    /// Append a signed 16-bit word.
    pub fn i16(&mut self, v: i16) {
        self.buf.extend_from_slice(&v.to_le_bytes());
    }

    /// Append an unsigned 16-bit word.
    pub fn u16(&mut self, v: u16) {
        self.buf.extend_from_slice(&v.to_le_bytes());
    }

    /// Append a signed 32-bit word.
    pub fn i32(&mut self, v: i32) {
        self.buf.extend_from_slice(&v.to_le_bytes());
    }

    /// Append an unsigned 32-bit word.
    pub fn u32(&mut self, v: u32) {
        self.buf.extend_from_slice(&v.to_le_bytes());
    }

    /// Append a **big-endian** unsigned 32-bit word — NetQuake's Datagram header longs (see
    /// [`Reader::u32_be`]).
    pub fn u32_be(&mut self, v: u32) {
        self.buf.extend_from_slice(&v.to_be_bytes());
    }

    /// Append a NUL-terminated string, encoded latin-1 (chars above U+00FF are dropped rather than
    /// mangled into multi-byte UTF-8, which a QuakeWorld server would misread). See [`latin1_bytes`].
    pub fn string(&mut self, s: &str) {
        self.buf.extend(latin1(s));
        self.buf.push(0);
    }

    /// Quantize an angle in degrees to 16-bit precision and append it. Usercmd angles are always
    /// 16-bit; the same rounding the client applies to its own stored angles lives in
    /// [`quantize_angle16`].
    pub fn angle16(&mut self, degrees: f32) {
        self.u16(to_angle16(degrees));
    }
}

/// The latin-1 bytes of a string: each `char` below U+0100 as a single byte, dropping the rest
/// (and interior NULs).
///
/// A QuakeWorld string on the wire is raw bytes, not UTF-8 — a "coloured" conchar is one byte in
/// the high half, and UTF-8 would split it into two the server misreads. [`Writer::string`] appends
/// these plus a terminating NUL; a raw context that has no terminator (the `connect` packet) takes
/// the bytes alone.
pub fn latin1_bytes(s: &str) -> Vec<u8> {
    latin1(s).collect()
}

/// The latin-1 bytes as an iterator, so [`Writer::string`] can `extend` without an intermediate
/// allocation.
fn latin1(s: &str) -> impl Iterator<Item = u8> + '_ {
    s.chars().filter(|&c| (c as u32) < 256 && c != '\0').map(|c| c as u8)
}

/// Encode an angle in degrees as QuakeWorld's 16-bit fixed-point turn.
pub fn to_angle16(degrees: f32) -> u16 {
    let scaled = degrees * (65536.0 / 360.0);
    let rounded = if scaled >= 0.0 { scaled + 0.5 } else { scaled - 0.5 };
    (rounded as i32 & 0xffff) as u16
}

/// Round-trip an angle through the 16-bit wire encoding.
///
/// The client must aim with the angle the *server* will see, not the one it computed: a bot that
/// keeps its own full-precision angle and sends a quantized one is aiming at a slightly different
/// place than it thinks. ezQuake applies exactly this to its stored view angles before sending.
pub fn quantize_angle16(degrees: f32) -> f32 {
    to_angle16(degrees) as i16 as f32 * (360.0 / 65536.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Scalars are little-endian and signedness is not decorative — `i16` vs `u16` is the
    /// difference between an entity at -1 and an entity at 65535.
    #[test]
    fn reads_scalars_little_endian() {
        let buf = [0x01, 0xff, 0xff, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12];
        let mut r = Reader::new(&buf);
        assert_eq!(r.u8().unwrap(), 1);
        assert_eq!(r.i16().unwrap(), -1);
        assert_eq!(r.u16().unwrap(), 0x1234);
        assert_eq!(r.i32().unwrap(), 0x1234_5678);
        assert!(r.at_end());
    }

    /// A short buffer is a network event: it reports where and how far it fell short, and leaves
    /// the reader usable rather than unwinding.
    #[test]
    fn underflow_reports_position() {
        let buf = [0x01, 0x02];
        let mut r = Reader::new(&buf);
        assert_eq!(r.u8().unwrap(), 1);
        let e = r.i32().unwrap_err();
        assert_eq!(e, Underflow { at: 1, want: 4, have: 1 });
    }

    /// The two string quirks that keep the read position honest: 255 bytes are skipped (but still
    /// consumed), and the string ends at the NUL with the reader positioned after it.
    #[test]
    fn string_skips_255_and_stops_at_nul() {
        let buf = *b"he\xffllo\0next";
        let mut r = Reader::new(&buf);
        assert_eq!(r.string().unwrap(), "hello");
        assert_eq!(r.pos(), 7); // consumed the 255 and the NUL
        assert_eq!(r.string().unwrap(), "next");
    }

    /// An unterminated string at the end of a message stops there rather than underflowing — id
    /// does the same, and a truncated `svc_print` shouldn't drop the whole packet.
    #[test]
    fn string_stops_at_end_of_buffer() {
        let buf = *b"tail";
        let mut r = Reader::new(&buf);
        assert_eq!(r.string().unwrap(), "tail");
        assert!(r.at_end());
    }

    /// High-bit bytes (QuakeWorld's coloured character set) survive as themselves, one byte to one
    /// char — obituary matching strips the high bit downstream and would break on lossy decoding.
    #[test]
    fn string_decodes_latin1_losslessly() {
        let buf = [b'a' | 0x80, b'b', 0];
        let mut r = Reader::new(&buf);
        let s = r.string().unwrap();
        assert_eq!(s.chars().map(|c| c as u32).collect::<Vec<_>>(), vec![0xe1, 0x62]);
    }

    /// The negotiated-width switch, which is the whole reason widths live on the reader: the same
    /// bytes mean different things, and a coord read at the wrong width desyncs everything after.
    #[test]
    fn coord_and_angle_follow_negotiated_width() {
        // Vanilla: coord = short/8, angle = char * 360/256.
        let buf = [0x40, 0x00, 0x40];
        let mut r = Reader::new(&buf);
        assert_eq!(r.coord().unwrap(), 8.0);
        assert_eq!(r.angle().unwrap(), 90.0);

        // FLOATCOORDS: coord = f32, angle = short * 360/65536.
        let mut buf = Vec::new();
        buf.extend_from_slice(&123.5f32.to_le_bytes());
        buf.extend_from_slice(&16384i16.to_le_bytes());
        let mut p = ProtoState::new();
        p.apply(crate::protocol::fte::FLOATCOORDS, 0, 0);
        let mut r = Reader::with_widths(&buf, &p);
        assert_eq!(r.coord().unwrap(), 123.5);
        assert_eq!(r.angle().unwrap(), 90.0);
    }

    /// Vanilla coords are *signed* ⅛-unit fixed point: the negative half of the map is the half a
    /// sign slip loses.
    #[test]
    fn vanilla_coord_is_signed() {
        let buf = (-4096i16 * 8 / 8).to_le_bytes();
        let mut r = Reader::new(&buf);
        assert_eq!(r.coord().unwrap(), -512.0);
    }

    /// Vanilla angles are signed bytes, so the wire has no +180: it encodes to the same byte as
    /// -180 and reads back negative. That's the same *direction*, and callers comparing angles must
    /// already handle the wrap — but a test asserting `180.0` round-trips would fail, and the
    /// reason is here rather than in whichever parser test trips over it.
    #[test]
    fn vanilla_angle_wraps_at_180() {
        let mut r = Reader::new(&[128]);
        assert_eq!(r.angle().unwrap(), -180.0);

        // Everything strictly inside the range round-trips to within a quantum (1.4°).
        for deg in [0.0f32, 45.0, 90.0, -90.0, 178.0] {
            let byte = ((deg * (256.0 / 360.0)).round() as i32 & 255) as u8;
            let back = Reader::new(&[byte]).angle().unwrap();
            assert!((back - deg).abs() < 360.0 / 256.0, "{deg} -> {back}");
        }
    }

    /// `seek` exists for the entity-delta peek-and-rewind; it must be exact, not approximate.
    #[test]
    fn seek_rewinds() {
        let buf = [1, 2, 3, 4];
        let mut r = Reader::new(&buf);
        assert_eq!(r.u16().unwrap(), 0x0201);
        r.seek(0);
        assert_eq!(r.u8().unwrap(), 1);
    }

    /// The writer's scalars must be byte-identical to what the reader expects, since the server
    /// runs the same code in reverse.
    #[test]
    fn writer_round_trips_through_reader() {
        let mut w = Writer::new();
        w.u8(9);
        w.i16(-300);
        w.i32(70000);
        w.string("hi");
        let buf = w.into_vec();

        let mut r = Reader::new(&buf);
        assert_eq!(r.u8().unwrap(), 9);
        assert_eq!(r.i16().unwrap(), -300);
        assert_eq!(r.i32().unwrap(), 70000);
        assert_eq!(r.string().unwrap(), "hi");
        assert!(r.at_end());
    }

    /// NetQuake's Datagram header is big-endian while the payload stays little-endian, so the two
    /// widths must not be confused: `0x8000_0400` is a control packet of length 0x400, and reading
    /// it little-endian would call it length 0x8000 with the wrong flag bits.
    #[test]
    fn u32_be_round_trips_and_differs_from_le() {
        let mut w = Writer::new();
        w.u32_be(0x8000_0400);
        let buf = w.into_vec();
        assert_eq!(buf, [0x80, 0x00, 0x04, 0x00]);
        let mut r = Reader::new(&buf);
        assert_eq!(r.u32_be().unwrap(), 0x8000_0400);
        // The same bytes read little-endian are a different number entirely.
        assert_eq!(Reader::new(&buf).u32().unwrap(), 0x0004_0080);
    }

    /// Angle quantization must round-trip through the wire encoding and wrap the way the server's
    /// does — a bot aims with the quantized value, so an off-by-one here is an aim error.
    #[test]
    fn angle16_round_trips_and_wraps() {
        for deg in [0.0f32, 90.0, -90.0, 179.9, -179.9] {
            let back = quantize_angle16(deg);
            assert!((back - deg).abs() < 360.0 / 65536.0, "{deg} -> {back}");
        }
        // 360 wraps to 0, and the encoding is modular rather than saturating.
        assert_eq!(to_angle16(360.0), 0);
        assert_eq!(quantize_angle16(360.0), 0.0);

        let mut w = Writer::new();
        w.angle16(90.0);
        let buf = w.into_vec();
        let mut r = Reader::new(&buf);
        assert!((r.angle16().unwrap() - 90.0).abs() < 0.01);
    }
}
