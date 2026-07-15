// SPDX-License-Identifier: AGPL-3.0-or-later

//! Protocol constants: version magics, the extension masks this client advertises, and the
//! negotiated state every coord/angle read depends on.
//!
//! Values mirror qualia's `src/protocol.cppm` and ezQuake's `qwprot/src/protocol.h` — both descend
//! from id's original `protocol.h`, so the numbers are fixed by the wire, not by us.
//!
//! **Negotiation is an intersection.** The client advertises what it can parse, the server
//! advertises what it can speak, and both sides use `client & server`. So advertising a bit we
//! handle costs nothing against a server that lacks it; advertising one we *don't* handle is a
//! desync waiting for the first packet that uses it. Everything in [`FTE`], [`FTE2`], [`MVD1`] and
//! [`Z_EXT`] below is therefore something this crate actually parses.

/// The base protocol version every QuakeWorld client speaks (`PROTOCOL_VERSION`).
pub const VERSION: u32 = 28;

/// Default QuakeWorld server port.
pub const PORT: u16 = 27500;

/// Largest datagram we'll build or accept (`MAX_MSGLEN`).
pub const MAX_MSGLEN: usize = 1450;

/// The four-byte prefix marking a connectionless (out-of-band) packet — an `i32` of `-1`.
pub const CONNECTIONLESS: [u8; 4] = [0xff, 0xff, 0xff, 0xff];

/// Extension-family magics, sent as an `i32` tag ahead of each mask. ASCII, little-endian:
/// `"FTEX"`, `"FTE2"`, `"MVD1"`. A server may also advertise `"FRAG"`/`"DTLS"`, which we read
/// and ignore.
pub mod magic {
    /// `PROTOCOL_VERSION_FTE` — FTE extensions, first mask.
    pub const FTE: u32 = u32::from_le_bytes(*b"FTEX");
    /// `PROTOCOL_VERSION_FTE2` — FTE extensions, second mask.
    pub const FTE2: u32 = u32::from_le_bytes(*b"FTE2");
    /// `PROTOCOL_VERSION_MVD1` — MVDSV extensions.
    pub const MVD1: u32 = u32::from_le_bytes(*b"MVD1");
    /// `PROTOCOL_VERSION_FRAG` — server supports packets larger than `MAX_MSGLEN`. Read and
    /// ignored: we never ask for one.
    pub const FRAG: u32 = u32::from_le_bytes(*b"FRAG");
    /// `PROTOCOL_VERSION_DTLS` — server offers DTLS. Read and ignored.
    pub const DTLS: u32 = u32::from_le_bytes(*b"DTLS");
}

/// FTE protocol extensions (`FTE_PEXT_*`) — the first mask.
pub mod fte {
    /// `.alpha` support. Adds a byte per entity delta, and relocates `PF_ONGROUND`/`PF_SOLID` in
    /// `svc_playerinfo` to bits 22/23 (with `PF_EXTRA_PFS` extending the flags to 24 bits).
    pub const TRANS: u32 = 0x0000_0008;
    /// Server sends `STAT_TIME`, so game time is exact rather than interpolated.
    pub const ACCURATE_TIMINGS: u32 = 0x0000_0040;
    /// No wire change — it stops FTE servers complaining that we might not grok a Half-Life BSP.
    pub const HLBSP: u32 = 0x0000_0200;
    /// Model indices above 255: enables `svc_fte_modellistshort`, `U_FTE_MODELDBL`, and the
    /// skin-bit-7 model+256 encoding in `svc_playerinfo`.
    pub const MODELDBL: u32 = 0x0000_1000;
    /// Entity numbers to 1024 (`U_FTE_ENTITYDBL` adds 512).
    pub const ENTITYDBL: u32 = 0x0000_2000;
    /// Entity numbers to 2048 (`U_FTE_ENTITYDBL2` adds 1024).
    pub const ENTITYDBL2: u32 = 0x0000_4000;
    /// Floating-point origins. **Reconfigures every subsequent coord/angle read** — see
    /// [`ProtoState::apply`](super::ProtoState::apply).
    pub const FLOATCOORDS: u32 = 0x0000_8000;
    /// Three bytes of colour modulation per entity delta (`U_FTE_COLOURMOD`).
    pub const COLOURMOD: u32 = 0x0008_0000;
    /// Baselines arrive as entity deltas (`svc_fte_spawnstatic2`/`svc_fte_spawnbaseline2`).
    pub const SPAWNSTATIC2: u32 = 0x0040_0000;
    /// Up to 256 entities per `svc_packetentities`.
    pub const PACKETENTITIES_256: u32 = 0x0100_0000;
    /// Alternate download method. **Not advertised** — we never download.
    pub const CHUNKEDDOWNLOADS: u32 = 0x2000_0000;
    /// Client-side QuakeC. **Not advertised** — we have no CSQC VM.
    pub const CSQC: u32 = 0x4000_0000;
}

/// FTE protocol extensions, second mask (`FTE_PEXT2_*`).
pub mod fte2 {
    /// Speex voice chat. **Not advertised** — no audio.
    pub const VOICECHAT: u32 = 0x0000_0002;
    /// The `svc_fte_updateentities` delta stream. **Not advertised** — qualia doesn't take it
    /// either, and it replaces the entity encoding wholesale.
    pub const REPLACEMENTDELTAS: u32 = 0x0000_0008;
}

/// MVDSV protocol extensions (`MVD_PEXT1_*`).
pub mod mvd1 {
    /// Like [`fte::FLOATCOORDS`](super::fte::FLOATCOORDS) but for entity/player origins only.
    pub const FLOATCOORDS: u32 = 1 << 0;
    /// `svc_setangle` gains a leading type byte (1 = teleport, 2 = respawn) so a client can fix up
    /// queued moves instead of walking the wrong way for a round-trip.
    pub const HIGHLAGTELEPORT: u32 = 1 << 1;
    /// Deprecated server-side weapon selection.
    pub const SERVERSIDEWEAPON: u32 = 1 << 2;
    /// Send weapon-choice explanations to the server for logging.
    pub const DEBUG_WEAPON: u32 = 1 << 3;
    /// Send predicted positions to the server to compare against antilag.
    pub const DEBUG_ANTILAG: u32 = 1 << 4;
    /// `dem_multiple(0)` packets carry length-prefixed hidden messages. MVD-only; not advertised.
    pub const HIDDEN_MESSAGES: u32 = 1 << 5;
    /// Deprecated server-side weapon selection, second revision.
    pub const SERVERSIDEWEAPON2: u32 = 1 << 6;
    /// Extends `svc_playerinfo`'s weaponframe block with predicted weapon state. Not advertised
    /// yet — it changes playerinfo parsing and we don't predict weapons.
    pub const WEAPONPREDICTION: u32 = 1 << 7;
    /// Projectiles as numbered semi-stateless entities (`svc_packetprojectiles`). Not advertised
    /// yet; it's the clean fix for re-associating anonymous nails frame to frame.
    pub const SIMPLEPROJECTILE: u32 = 1 << 8;
}

/// ZQuake extensions (`Z_EXT_*`), advertised through the `*z_ext` userinfo key rather than the
/// challenge handshake, and echoed back by the server in serverinfo.
pub mod z_ext {
    /// Basic `PM_TYPE` support (reliable `jump_held`).
    pub const PM_TYPE: u32 = 1 << 0;
    /// Adds `PM_FLY` / `PM_SPECTATOR` move types.
    pub const PM_TYPE_NEW: u32 = 1 << 1;
    /// `STAT_VIEWHEIGHT`.
    pub const VIEWHEIGHT: u32 = 1 << 2;
    /// `STAT_TIME` — authoritative server time.
    pub const SERVERTIME: u32 = 1 << 3;
    /// `maxpitch` / `minpitch` serverinfo keys.
    pub const PITCHLIMITS: u32 = 1 << 4;
    /// The server understands the `join` and `observe` commands.
    pub const JOIN_OBSERVE: u32 = 1 << 5;
    /// `PF_ONGROUND` is valid for *every* `svc_playerinfo`, not just our own — without this a bot
    /// cannot tell whether an enemy is airborne.
    pub const PF_ONGROUND: u32 = 1 << 6;
    /// ZQ_VWEP (visible weapons).
    pub const VWEP: u32 = 1 << 7;
    /// `PF_SOLID` is valid for every `svc_playerinfo`.
    pub const PF_SOLID: u32 = 1 << 8;
}

/// The FTE extensions this client advertises: everything it parses, minus downloads (we never ask
/// for a file) and CSQC (no VM). Matches qualia's set apart from those two.
pub const FTE: u32 = fte::TRANS
    | fte::ACCURATE_TIMINGS
    | fte::HLBSP
    | fte::MODELDBL
    | fte::ENTITYDBL
    | fte::ENTITYDBL2
    | fte::FLOATCOORDS
    | fte::COLOURMOD
    | fte::SPAWNSTATIC2
    | fte::PACKETENTITIES_256;

/// The FTE2 extensions this client advertises: none. Both bits in that mask (voice, replacement
/// deltas) would change what we must parse without giving a bot anything.
pub const FTE2: u32 = 0;

/// The MVDSV extensions this client advertises. `FLOATCOORDS` for origin precision and
/// `HIGHLAGTELEPORT` so a teleport doesn't send us walking the wrong way for a round-trip; the
/// weapon-prediction and simple-projectile bits are deliberately deferred until we parse them.
pub const MVD1: u32 = mvd1::FLOATCOORDS | mvd1::HIGHLAGTELEPORT;

/// The ZQuake extensions this client advertises — all of them. `PF_ONGROUND` and `SERVERTIME` are
/// the load-bearing ones for a bot: whether an enemy is airborne, and what time it is.
pub const Z_EXT: u32 = z_ext::PM_TYPE
    | z_ext::PM_TYPE_NEW
    | z_ext::VIEWHEIGHT
    | z_ext::SERVERTIME
    | z_ext::PITCHLIMITS
    | z_ext::JOIN_OBSERVE
    | z_ext::PF_ONGROUND
    | z_ext::VWEP
    | z_ext::PF_SOLID;

/// The negotiated protocol: which extension bits both ends agreed on, and the coord/angle widths
/// that follow from them.
///
/// The widths are the reason this is a struct rather than four loose masks. `FTE_PEXT_FLOATCOORDS`
/// silently re-sizes *every* coordinate and angle on the wire — in baselines, sounds, temp
/// entities, player and entity deltas alike — so a reader that doesn't carry the negotiated width
/// misparses everything after the first coord. [`Reader`](super::sizebuf::Reader) takes its widths
/// from here, and [`apply`](Self::apply) is the one place they're set.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ProtoState {
    /// Negotiated `FTE_PEXT_*` bits (`client & server`).
    pub fte: u32,
    /// Negotiated `FTE_PEXT2_*` bits.
    pub fte2: u32,
    /// Negotiated `MVD_PEXT1_*` bits.
    pub mvd1: u32,
    /// `Z_EXT_*` bits the server confirmed via the `*z_ext` serverinfo key.
    pub z_ext: u32,
    /// Width of a coord on the wire: 2 (fixed-point 1/8 unit) or 4 (`f32`).
    pub coord_bytes: u8,
    /// Width of an angle on the wire: 1 (1/256 turn) or 2 (1/65536 turn).
    pub angle_bytes: u8,
}

impl ProtoState {
    /// A fresh, un-negotiated protocol: vanilla widths, no extensions.
    pub fn new() -> Self {
        ProtoState {
            fte: 0,
            fte2: 0,
            mvd1: 0,
            z_ext: 0,
            coord_bytes: 2,
            angle_bytes: 1,
        }
    }

    /// Adopt the masks the server echoed in `svc_serverdata` and derive the coord/angle widths
    /// from them. Called once per `svc_serverdata` — i.e. once per map — before any coord is read.
    pub fn apply(&mut self, fte: u32, fte2: u32, mvd1: u32) {
        self.fte = fte;
        self.fte2 = fte2;
        self.mvd1 = mvd1;
        (self.coord_bytes, self.angle_bytes) = if fte & fte::FLOATCOORDS != 0 { (4, 2) } else { (2, 1) };
    }

    /// Whether a negotiated FTE bit is live.
    pub fn has_fte(&self, bit: u32) -> bool {
        self.fte & bit != 0
    }

    /// Whether a negotiated MVDSV bit is live.
    pub fn has_mvd1(&self, bit: u32) -> bool {
        self.mvd1 & bit != 0
    }

    /// Whether the server confirmed a ZQuake extension bit.
    pub fn has_z_ext(&self, bit: u32) -> bool {
        self.z_ext & bit != 0
    }

    /// Whether entity/player origins use raw `f32`s. True under either the FTE extension (which
    /// widens *everything*) or MVDSV's narrower version (origins only).
    pub fn float_origins(&self) -> bool {
        self.has_fte(fte::FLOATCOORDS) || self.has_mvd1(mvd1::FLOATCOORDS)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The magics are ASCII tags read as little-endian `i32`s. Pin the numbers a server actually
    /// sends, so a byte-order slip can't quietly turn the handshake into "unknown extension".
    #[test]
    fn extension_magics_are_ascii_le() {
        assert_eq!(magic::FTE, 0x5845_5446);
        assert_eq!(magic::FTE2, 0x3245_5446);
        assert_eq!(magic::MVD1, 0x3144_564d);
    }

    /// The advertised masks are a promise: every bit here is one the parser honours. Pinning the
    /// totals makes adding a bit a deliberate, reviewable act rather than a merge artifact.
    #[test]
    fn advertised_masks_are_what_we_parse() {
        assert_eq!(FTE, 0x0148_f248);
        assert_eq!(FTE2, 0);
        assert_eq!(MVD1, 0b11);
        assert_eq!(Z_EXT, 0x1ff);

        // The three we deliberately don't ask for, because each would change parsing.
        assert_eq!(FTE & fte::CHUNKEDDOWNLOADS, 0);
        assert_eq!(FTE & fte::CSQC, 0);
        assert_eq!(FTE2 & fte2::REPLACEMENTDELTAS, 0);
    }

    /// `FLOATCOORDS` is the one negotiated bit that re-sizes the reader. Both directions matter:
    /// a stale width from a previous map would misparse the new one.
    #[test]
    fn floatcoords_sets_widths() {
        let mut p = ProtoState::new();
        assert_eq!((p.coord_bytes, p.angle_bytes), (2, 1));

        p.apply(fte::FLOATCOORDS, 0, 0);
        assert_eq!((p.coord_bytes, p.angle_bytes), (4, 2));
        assert!(p.float_origins());

        p.apply(0, 0, 0);
        assert_eq!((p.coord_bytes, p.angle_bytes), (2, 1));
        assert!(!p.float_origins());

        // MVDSV's variant floats the origins without widening the reader.
        p.apply(0, 0, mvd1::FLOATCOORDS);
        assert_eq!((p.coord_bytes, p.angle_bytes), (2, 1));
        assert!(p.float_origins());
    }
}
