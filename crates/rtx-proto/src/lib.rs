// SPDX-License-Identifier: AGPL-3.0-or-later

//! `rtx-proto` — the QuakeWorld wire protocol, as a pure codec.
//!
//! Everything a network client needs to *speak* QuakeWorld and nothing about *playing* it: message
//! buffers ([`sizebuf`]), info strings ([`info`]), the two checksums a real client must compute
//! ([`crc`], [`checksum`]), the protocol's constants and negotiated extension masks ([`protocol`]),
//! the reliable transport ([`netchan`]), the connectionless handshake ([`oob`]), the server→client
//! parser ([`svc`]) and the client→server builder ([`clc`]).
//!
//! No IO, no threads, no world state — the parser hands back typed events and the caller decides
//! what they mean. That keeps the wire format testable against recorded datagram fixtures (see
//! `tests/`) independently of the bot that consumes them, and it's why this crate builds and tests
//! in CI while the client that uses it does not.
//!
//! The reference implementations are [qualia](https://github.com/dsvensson/qualia) (`src/protocol.cppm`,
//! `src/cl_parse.cppm`) and [ezQuake](https://github.com/QW-Group/ezquake-source) (`qwprot/src/protocol.h`,
//! `src/cl_parse.c`); where this crate ports id-derived tables verbatim, the module says so.

pub mod checksum;
pub mod clc;
pub mod crc;
pub mod info;
pub mod mdfour;
pub mod netchan;
pub mod oob;
pub mod protocol;
pub mod sizebuf;
pub mod svc;
