// SPDX-License-Identifier: AGPL-3.0-or-later

//! NetQuake — the *original* Quake network protocol, as a pure codec alongside the QuakeWorld one.
//!
//! QuakeWorld and NetQuake share a lineage and a lot of vocabulary (the same `svc_*` idea, the same
//! coord/angle quantization, mostly the same entity fields) but agree on almost nothing at the byte
//! level. The differences that matter here:
//!
//! - **Transport.** QW rides one netchan with the reliable bit folded into the sequence word and a
//!   client `qport`; NetQuake's "Datagram" protocol ([`chan`]) frames every packet with a
//!   **big-endian** `[flags|len][sequence]` header and runs two independent streams — sequenced
//!   unreliables and a *separate* stop-and-wait reliable channel that fragments and ACKs.
//! - **Handshake.** QW is a text `getchallenge`/`connect` exchange; NetQuake is the binary
//!   `CCREQ_CONNECT`/`CCREP_ACCEPT` control protocol ([`connect`]).
//! - **Widths.** QW negotiates one FLOATCOORDS bit; NetQuake keys coord/angle widths off a protocol
//!   *version* (15/666/999) and, for 999, a `protocolflags` word ([`protocol::NqProtoState`]).
//!
//! What is shared is deliberately reused rather than reimplemented: the low-level [`Reader`] and
//! [`Writer`](crate::sizebuf::Writer) scalars (NetQuake payloads are little-endian; only the 8-byte
//! Datagram header is big-endian), the [`SvcEvent`](crate::svc::SvcEvent) vocabulary the parser
//! emits, and the `EntityDelta`/`Baseline` field carriers. So a caller that already consumes QW
//! `SvcEvent`s consumes NetQuake ones through the same match.
//!
//! Scope: vanilla protocols 15, 666 and 999, declining every FTE `PEXT` extension (the client
//! answers the server's `cmd pext` with a bare `pext`, staying on the base wire format). The
//! reference is QuakeSpasm-Spiked (`Quake/net_dgrm.c`, `Quake/cl_parse.c`, `Quake/common.c`,
//! `Quake/protocol.h`), whose numbers descend from id's original `net.h`/`protocol.h`.

pub mod chan;
pub mod clc;
pub mod connect;
pub mod protocol;
pub mod svc;

/// Datagram header flags (`net_defs.h`). The header is one big-endian long whose high 16 bits are
/// these flags and whose low 16 bits ([`NETFLAG_LENGTH_MASK`]) are the packet length *including* the
/// header. A control packet carries only [`NETFLAG_CTL`]; an in-band packet carries exactly one of
/// [`NETFLAG_DATA`]/[`NETFLAG_ACK`]/[`NETFLAG_UNRELIABLE`] (plus [`NETFLAG_EOM`] on the last reliable
/// fragment) and is followed by a second big-endian long, the sequence.
pub const NETFLAG_LENGTH_MASK: u32 = 0x0000_ffff;
/// A reliable-stream fragment.
pub const NETFLAG_DATA: u32 = 0x0001_0000;
/// Acknowledges one reliable fragment by sequence.
pub const NETFLAG_ACK: u32 = 0x0002_0000;
/// A negative ack — unused by us, but named so a stray one is recognisable.
pub const NETFLAG_NAK: u32 = 0x0004_0000;
/// End of a reliable message: the fragment carrying it is the last.
pub const NETFLAG_EOM: u32 = 0x0008_0000;
/// A sequenced unreliable packet (the per-frame game datagram).
pub const NETFLAG_UNRELIABLE: u32 = 0x0010_0000;
/// A connectionless control packet (`CCREQ_*`/`CCREP_*`).
pub const NETFLAG_CTL: u32 = 0x8000_0000;

/// Bytes in an in-band Datagram header: the flags|len long and the sequence long.
pub const HEADER_BYTES: usize = 8;
