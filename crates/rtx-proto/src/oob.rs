// SPDX-License-Identifier: AGPL-3.0-or-later

//! The connectionless handshake — the packets sent before there is a [`Netchan`](crate::netchan)
//! to send them on.
//!
//! Out-of-band packets are prefixed with four `0xff` bytes (an `i32` of `-1`, which no real
//! sequence number can be) and are otherwise mostly ASCII. The exchange is:
//!
//! ```text
//! C→S  getchallenge
//! S→C  c<challenge>\0 [i32 magic][i32 mask]…     ← 'c', then the server's extension offers
//! C→S  connect 28 <qport> <challenge> "<userinfo>"
//!      0x<magic> 0x<mask>                        ← one line per family we're accepting
//! S→C  j                                         ← accepted; open the netchan and say "new"
//! ```
//!
//! The challenge exists to prove we can receive at the address we claim, which stops the server
//! being used to reflect traffic at a spoofed victim.
//!
//! Note the shape of the challenge reply: the challenge itself is **ASCII decimal terminated by a
//! NUL**, and the extension masks are **binary longs appended after that NUL**. So it is neither a
//! text protocol nor a binary one, and a parser that stops at the NUL (as a string reader would)
//! silently negotiates zero extensions rather than failing.

use crate::protocol::{self, magic};
use crate::sizebuf::Reader;
use crate::svc::{op, DOWNLOAD_CHUNK_SIZE};

/// Out-of-band packet type bytes.
mod s2c {
    /// `S2C_CHALLENGE` — the challenge, with the server's extension offers.
    pub const CHALLENGE: u8 = b'c';
    /// `S2C_CONNECTION` — connection accepted.
    pub const CONNECTION: u8 = b'j';
    /// `A2C_PRINT` — a message to show the user, typically a rejection reason.
    pub const PRINT: u8 = b'n';
    /// `A2A_PING`.
    pub const PING: u8 = b'k';
    /// `A2A_ACK`.
    pub const ACK: u8 = b'l';
    /// `A2C_CLIENT_COMMAND` — a command for the client to run.
    pub const CLIENT_COMMAND: u8 = b'B';
}

/// What the server said out-of-band.
#[derive(Clone, Debug, PartialEq)]
pub enum Oob {
    /// The challenge and the extension masks the server offers. Masks are what the *server*
    /// supports; the client intersects them with its own before replying.
    Challenge {
        /// The challenge number to echo back in `connect`.
        challenge: i32,
        /// The server's `FTE_PEXT_*` offer.
        fte: u32,
        /// The server's `FTE_PEXT2_*` offer.
        fte2: u32,
        /// The server's `MVD_PEXT1_*` offer.
        mvd1: u32,
    },
    /// Connection accepted — open the netchan and start signon.
    Accepted,
    /// A message for the user. When a connection is refused (a password, a ban, a full server),
    /// this carries the reason.
    Print(String),
    /// A ping to answer with an ack.
    Ping,
    /// An ack of our ping.
    Ack,
    /// A console command from the server.
    ClientCommand(String),
    /// An FTE chunked-download reply sent outside the sequenced netchan to keep the pipe full.
    DownloadChunk {
        cookie: u32,
        chunk: u32,
        data: Box<[u8; DOWNLOAD_CHUNK_SIZE]>,
    },
    /// Recognisably out-of-band, but not something we act on.
    Unknown(u8),
}

/// Whether a datagram is connectionless (`0xffffffff`-prefixed) rather than netchan traffic.
pub fn is_oob(data: &[u8]) -> bool {
    data.len() >= 4 && data[..4] == protocol::CONNECTIONLESS
}

/// Wrap a payload as a connectionless packet.
pub fn wrap(payload: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(4 + payload.len());
    out.extend_from_slice(&protocol::CONNECTIONLESS);
    out.extend_from_slice(payload);
    out
}

/// The `getchallenge` request that opens a connection.
pub fn getchallenge() -> Vec<u8> {
    wrap(b"getchallenge\n")
}

/// Answer a `A2A_PING` with an `A2A_ACK`.
pub fn ack() -> Vec<u8> {
    wrap(&[s2c::ACK])
}

/// Parse a connectionless packet from the server.
///
/// Returns `None` if it isn't out-of-band or is empty. An unrecognised type byte is
/// [`Oob::Unknown`] rather than an error: unlike the in-band stream, where an unknown opcode means
/// we've lost our place in the bytes, each of these is a self-contained datagram and ignoring one
/// costs nothing.
pub fn parse(data: &[u8]) -> Option<Oob> {
    if !is_oob(data) {
        return None;
    }
    let body = &data[4..];
    let (&kind, rest) = body.split_first()?;
    match kind {
        s2c::CHALLENGE => {
            let mut r = Reader::new(rest);
            // ASCII decimal up to the NUL. `parse` ignores trailing junk the way `atoi` does; a
            // challenge is always plain digits, possibly with a leading '-'.
            let text = r.string().ok()?;
            let challenge = parse_leading_i32(&text)?;

            // Binary longs after the NUL: (magic, mask) pairs, terminated by running out of bytes.
            // An unknown magic still consumes its mask — that's how "FRAG"/"DTLS" pass by without
            // derailing the ones we want.
            let (mut fte, mut fte2, mut mvd1) = (0, 0, 0);
            while let Ok(tag) = r.u32() {
                let Ok(mask) = r.u32() else { break };
                match tag {
                    magic::FTE => fte = mask,
                    magic::FTE2 => fte2 = mask,
                    magic::MVD1 => mvd1 = mask,
                    _ => {}
                }
            }
            Some(Oob::Challenge {
                challenge,
                fte,
                fte2,
                mvd1,
            })
        }
        s2c::CONNECTION => Some(Oob::Accepted),
        s2c::PRINT if rest.starts_with(b"\\chunk") => {
            let mut r = Reader::new(&rest[6..]);
            let cookie = r.u32().ok()?;
            if r.u8().ok()? != op::DOWNLOAD {
                return Some(Oob::Unknown(kind));
            }
            let chunk = r.u32().ok()?;
            let data: [u8; DOWNLOAD_CHUNK_SIZE] = r.bytes(DOWNLOAD_CHUNK_SIZE).ok()?.try_into().ok()?;
            Some(Oob::DownloadChunk {
                cookie,
                chunk,
                data: Box::new(data),
            })
        }
        s2c::PRINT => Some(Oob::Print(Reader::new(rest).string().ok()?)),
        s2c::PING => Some(Oob::Ping),
        s2c::ACK => Some(Oob::Ack),
        s2c::CLIENT_COMMAND => Some(Oob::ClientCommand(Reader::new(rest).string().ok()?)),
        other => Some(Oob::Unknown(other)),
    }
}

/// Read a decimal integer from the front of a string, ignoring whatever follows — C's `atoi`,
/// which is what the reference client parses the challenge with.
fn parse_leading_i32(s: &str) -> Option<i32> {
    let s = s.trim_start();
    let digits = s.trim_start_matches('-');
    let sign_len = s.len() - digits.len();
    let end = digits.find(|c: char| !c.is_ascii_digit()).unwrap_or(digits.len());
    s[..sign_len + end].parse().ok()
}

/// The masks we and the server agreed on: `client & server`, per family.
///
/// This is where the extension negotiation actually happens, and the direction matters. We can only
/// *narrow* the server's offer, never widen it — so a bit set here is one both ends promised to
/// speak. It's also computed twice in a session: once here, and again when `svc_serverdata` echoes
/// the masks back. They must agree, and the server's echo wins.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Negotiated {
    /// Agreed `FTE_PEXT_*` bits.
    pub fte: u32,
    /// Agreed `FTE_PEXT2_*` bits.
    pub fte2: u32,
    /// Agreed `MVD_PEXT1_*` bits.
    pub mvd1: u32,
}

impl Negotiated {
    /// Intersect the server's offer with what this client can parse.
    pub fn intersect(offered_fte: u32, offered_fte2: u32, offered_mvd1: u32) -> Self {
        Negotiated {
            fte: offered_fte & protocol::FTE,
            fte2: offered_fte2 & protocol::FTE2,
            mvd1: offered_mvd1 & protocol::MVD1,
        }
    }
}

/// Build the `connect` packet.
///
/// `userinfo` must already carry `*z_ext` (see
/// [`UserinfoBuilder`](crate::info::UserinfoBuilder)) — it's a star key, and connect is the one
/// moment a client may set one.
///
/// The extension lines are appended to the same datagram as plain text, one per family, and only
/// for families we actually agreed on. A zero mask means "not this family", and sending
/// `0x… 0x0` instead of omitting the line confuses some servers.
pub fn connect(qport: u16, challenge: i32, userinfo: &str, n: &Negotiated) -> Vec<u8> {
    let mut s = format!(
        "connect {} {} {} \"{}\"\n",
        protocol::VERSION,
        qport,
        challenge,
        userinfo
    );
    for (tag, mask) in [(magic::FTE, n.fte), (magic::FTE2, n.fte2), (magic::MVD1, n.mvd1)] {
        if mask != 0 {
            s.push_str(&format!("0x{tag:x} 0x{mask:x}\n"));
        }
    }
    // Latin-1, not UTF-8: a coloured player name carries high-half conchars (0x80+), and each must
    // go out as a single byte. The ASCII of the command itself is unchanged by this.
    wrap(&crate::sizebuf::latin1_bytes(&s))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a challenge reply the way a server does: `'c'`, ASCII decimal, a NUL, then binary
    /// (magic, mask) pairs.
    fn challenge_reply(challenge: i32, pairs: &[(u32, u32)]) -> Vec<u8> {
        let mut body = vec![s2c::CHALLENGE];
        body.extend_from_slice(challenge.to_string().as_bytes());
        body.push(0);
        for &(tag, mask) in pairs {
            body.extend_from_slice(&tag.to_le_bytes());
            body.extend_from_slice(&mask.to_le_bytes());
        }
        wrap(&body)
    }

    /// Only `0xffffffff`-prefixed datagrams are out-of-band; everything else is netchan traffic and
    /// must not be misread as a handshake.
    #[test]
    fn recognises_out_of_band_framing() {
        assert!(is_oob(&[0xff, 0xff, 0xff, 0xff, b'j']));
        assert!(!is_oob(&[0x01, 0x00, 0x00, 0x00, b'j']));
        assert!(!is_oob(&[0xff, 0xff]));
        assert_eq!(parse(&[0x01, 0x00, 0x00, 0x00]), None);
        assert_eq!(parse(&[]), None);
    }

    /// The reply's mixed text/binary shape: the masks live *after* the NUL that ends the challenge
    /// text. A reader that stops at the NUL would silently negotiate nothing.
    #[test]
    fn parses_challenge_with_extension_masks() {
        let pkt = challenge_reply(1234567, &[(magic::FTE, 0xdead), (magic::FTE2, 0x2), (magic::MVD1, 0x3)]);
        assert_eq!(
            parse(&pkt),
            Some(Oob::Challenge {
                challenge: 1234567,
                fte: 0xdead,
                fte2: 0x2,
                mvd1: 0x3
            })
        );
    }

    /// A server that offers no extensions (or families we don't know) still yields a usable
    /// challenge — an unknown magic consumes its mask and the loop carries on to the ones we want.
    #[test]
    fn parses_challenge_without_or_with_unknown_extensions() {
        assert_eq!(
            parse(&challenge_reply(42, &[])),
            Some(Oob::Challenge {
                challenge: 42,
                fte: 0,
                fte2: 0,
                mvd1: 0
            })
        );

        // "FRAG" and "DTLS" sit between the families we care about; both must be skipped cleanly.
        let pkt = challenge_reply(
            7,
            &[
                (magic::FRAG, 1400),
                (magic::FTE, 0x8),
                (magic::DTLS, 1),
                (magic::MVD1, 0x3),
            ],
        );
        assert_eq!(
            parse(&pkt),
            Some(Oob::Challenge {
                challenge: 7,
                fte: 0x8,
                fte2: 0,
                mvd1: 0x3
            })
        );
    }

    /// Servers hand out negative challenges too (it's a signed long); and a truncated trailing pair
    /// must not lose the pairs already read.
    #[test]
    fn challenge_tolerates_negative_and_truncated_input() {
        assert!(matches!(
            parse(&challenge_reply(-99, &[])),
            Some(Oob::Challenge { challenge: -99, .. })
        ));

        let mut pkt = challenge_reply(5, &[(magic::FTE, 0x8)]);
        pkt.extend_from_slice(&magic::MVD1.to_le_bytes()); // magic with no mask behind it
        assert_eq!(
            parse(&pkt),
            Some(Oob::Challenge {
                challenge: 5,
                fte: 0x8,
                fte2: 0,
                mvd1: 0
            })
        );
    }

    /// The other out-of-band types, including the rejection message — which is the only place a
    /// server explains *why* it wouldn't let us in.
    #[test]
    fn parses_other_oob_packets() {
        assert_eq!(parse(&wrap(b"j")), Some(Oob::Accepted));
        assert_eq!(
            parse(&wrap(b"nserver is full\0")),
            Some(Oob::Print("server is full".into()))
        );
        assert_eq!(parse(&wrap(b"k")), Some(Oob::Ping));
        assert_eq!(parse(&wrap(b"l")), Some(Oob::Ack));
        assert_eq!(
            parse(&wrap(b"Bcmd pext\0")),
            Some(Oob::ClientCommand("cmd pext".into()))
        );
        assert_eq!(parse(&wrap(b"?")), Some(Oob::Unknown(b'?')));
    }

    /// Extra chunk replies masquerade as `A2C_PRINT` packets, but are binary after the six-byte
    /// marker and must not be fed through the console-string parser.
    #[test]
    fn parses_oob_download_chunks() {
        let mut body = Vec::new();
        body.push(s2c::PRINT);
        body.extend_from_slice(b"\\chunk");
        body.extend_from_slice(&17u32.to_le_bytes());
        body.push(op::DOWNLOAD);
        body.extend_from_slice(&23u32.to_le_bytes());
        body.extend_from_slice(&[0xa5; DOWNLOAD_CHUNK_SIZE]);
        assert_eq!(
            parse(&wrap(&body)),
            Some(Oob::DownloadChunk {
                cookie: 17,
                chunk: 23,
                data: Box::new([0xa5; DOWNLOAD_CHUNK_SIZE]),
            })
        );

        body.truncate(body.len() - 1);
        assert_eq!(parse(&wrap(&body)), None, "a truncated fixed-size chunk is not usable");
    }

    /// Negotiation can only narrow: a server offering a bit we can't parse must not end up in the
    /// agreed set, because the agreed set is a promise that we'll understand what arrives.
    #[test]
    fn negotiation_intersects_and_never_widens() {
        // A server offering everything gets exactly our advertised set back.
        let n = Negotiated::intersect(u32::MAX, u32::MAX, u32::MAX);
        assert_eq!(
            n,
            Negotiated {
                fte: protocol::FTE,
                fte2: protocol::FTE2,
                mvd1: protocol::MVD1
            }
        );

        // Chunked downloads are now part of the promise; unrelated unimplemented bits stay out.
        assert_ne!(n.fte & protocol::fte::CHUNKEDDOWNLOADS, 0);
        assert_eq!(n.fte & protocol::fte::CSQC, 0);
        assert_eq!(n.fte2, 0);
        assert_eq!(n.mvd1 & protocol::mvd1::SIMPLEPROJECTILE, 0);

        // A server offering nothing yields nothing.
        assert_eq!(
            Negotiated::intersect(0, 0, 0),
            Negotiated {
                fte: 0,
                fte2: 0,
                mvd1: 0
            }
        );

        // And a plain vanilla server offering one bit we know yields just that bit.
        let n = Negotiated::intersect(protocol::fte::FLOATCOORDS, 0, 0);
        assert_eq!(n.fte, protocol::fte::FLOATCOORDS);
    }

    /// The `connect` packet's exact text. A server parses this with `sscanf`-grade tolerance, so
    /// the field order, the quotes around userinfo and the trailing newline all matter.
    #[test]
    fn builds_connect_packet() {
        let n = Negotiated {
            fte: 0x8,
            fte2: 0,
            mvd1: 0x3,
        };
        let pkt = connect(0x1234, 999, "\\name\\bot", &n);

        assert_eq!(&pkt[..4], &protocol::CONNECTIONLESS);
        let text = std::str::from_utf8(&pkt[4..]).unwrap();
        assert_eq!(
            text,
            "connect 28 4660 999 \"\\name\\bot\"\n0x58455446 0x8\n0x3144564d 0x3\n"
        );
    }

    /// A family we agreed nothing on gets **no line at all** — not a line with a zero mask, which
    /// some servers treat as a malformed offer.
    #[test]
    fn connect_omits_empty_extension_families() {
        let pkt = connect(
            1,
            2,
            "\\name\\bot",
            &Negotiated {
                fte: 0,
                fte2: 0,
                mvd1: 0,
            },
        );
        let text = std::str::from_utf8(&pkt[4..]).unwrap();
        assert_eq!(text, "connect 28 1 2 \"\\name\\bot\"\n");
        assert!(!text.contains("0x0"));
    }

    /// The whole handshake, end to end, as a client actually walks it.
    #[test]
    fn handshake_round_trip() {
        // Server offers a realistic mvdsv set, including bits we don't take.
        let offer = protocol::FTE | protocol::fte::CHUNKEDDOWNLOADS | protocol::fte::CSQC;
        let reply = challenge_reply(31337, &[(magic::FTE, offer), (magic::MVD1, 0x1ff)]);

        let Some(Oob::Challenge {
            challenge,
            fte,
            fte2,
            mvd1,
        }) = parse(&reply)
        else {
            panic!("expected a challenge");
        };
        let n = Negotiated::intersect(fte, fte2, mvd1);
        assert_eq!(n.fte, protocol::FTE, "narrowed to what we parse");
        assert_eq!(n.mvd1, protocol::MVD1);

        let pkt = connect(4242, challenge, "\\name\\bot\\*z_ext\\511", &n);
        let text = std::str::from_utf8(&pkt[4..]).unwrap();
        assert!(text.starts_with("connect 28 4242 31337 \"\\name\\bot\\*z_ext\\511\"\n"));

        assert_eq!(parse(&wrap(b"j")), Some(Oob::Accepted));
    }

    /// A coloured name reaches the wire as single high-half bytes, not the two-byte UTF-8 a Rust
    /// `String` would otherwise encode them as — the server reads conchars, not Unicode. The name
    /// here is a coloured `bot`, a `0x85` dot, then plain `Grunt` (`\u{e2}\u{ef}\u{f4}\u{85}Grunt`).
    #[test]
    fn connect_encodes_coloured_name_as_latin1() {
        let name = "\u{e2}\u{ef}\u{f4}\u{85}Grunt";
        let pkt = connect(
            1,
            2,
            &format!("\\name\\{name}"),
            &Negotiated {
                fte: 0,
                fte2: 0,
                mvd1: 0,
            },
        );

        // The coloured `bot`, the dot, and `Grunt` are each one byte on the wire.
        let needle = [0xe2, 0xef, 0xf4, 0x85, b'G', b'r', b'u', b'n', b't'];
        assert!(
            pkt.windows(needle.len()).any(|w| w == needle),
            "coloured name not found as latin-1 bytes in {pkt:?}"
        );
        // And no UTF-8 lead byte (0xc3) smuggled a two-byte sequence in.
        assert!(!pkt.contains(&0xc3), "name was UTF-8 encoded, not latin-1");
    }
}
