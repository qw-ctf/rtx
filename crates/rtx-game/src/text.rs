// SPDX-License-Identifier: AGPL-3.0-or-later

//! Building QuakeWorld "conchar" strings — the game's character set, not ASCII.
//!
//! QuakeWorld draws text through a 256-glyph console font. The low half is roughly ASCII; the
//! **high half** (byte `| 0x80`) is a *coloured* (brown/red) copy of the same glyphs, and a few
//! special glyphs live down in the control range — among them a filled dot at `0x05`, whose
//! coloured copy `0x85` is the separator you see in names like `bot•Grunt`.
//!
//! We build these as ordinary Rust `char`s kept in the Latin-1 range (U+0000..=U+00FF): a coloured
//! `b` is `'\u{e2}'`, not a multi-byte UTF-8 sequence. On the way out they become one byte each —
//! the netclient's userinfo goes through the wire's latin-1 encoder (`rtx_proto::sizebuf`), and the
//! qwprogs roster uses [`latin1_bytes`] here to build the `CString` the engine stores verbatim.
//! Both paths need this because [`rtx_proto`](rtx_proto) is an optional dependency, absent from the
//! default (qwprogs) build.

/// The high-half offset: OR an ASCII byte with this to get its coloured conchar.
const COLOURED: u8 = 0x80;

/// The coloured bullet separator — conchar `0x85`, the high-half copy of the white dot `0x05`.
pub(crate) const DOT: char = '\u{85}';

/// The coloured (brown/red) copy of an ASCII string: each byte moved into the high half.
///
/// Intended for ASCII input (tags and names are). A non-ASCII byte — the continuation byte of a
/// UTF-8 sequence — would be `| 0x80`'d too, but callers don't feed one.
pub(crate) fn coloured(s: &str) -> String {
    s.bytes().map(|b| (b | COLOURED) as char).collect()
}

/// The latin-1 bytes of a string: each `char` below U+0100 as one byte, dropping the rest (and
/// interior NULs). The same rule the wire uses (`rtx_proto::sizebuf::latin1_bytes`), duplicated
/// here because that crate isn't on the default build — used to turn a built name into the bytes a
/// `CString` carries to the engine.
pub(crate) fn latin1_bytes(s: &str) -> Vec<u8> {
    s.chars().filter(|&c| (c as u32) < 256 && c != '\0').map(|c| c as u8).collect()
}

/// The readable-ASCII copy of a conchar string, for logging.
///
/// Colour (the high bit) is stripped, and the special glyphs QuakeWorld hides in the control range
/// are mapped to the character they depict — so a coloured `bot•Grunt` logs as `bot·Grunt` (a real
/// middle dot), and the gold HUD digits (`0x12`–`0x1b`) read as digits. Output is UTF-8, so a glyph
/// is drawn with the best Unicode match rather than a lossy ASCII one. A `char` outside the 256-glyph
/// conchar set (there shouldn't be one in wire text) passes through untouched.
///
/// Only the netclient logs server-originated conchar text; the qwprogs build never calls this.
#[cfg(feature = "netclient")]
pub(crate) fn readable(s: &str) -> String {
    s.chars()
        .map(|c| if (c as u32) < 256 { READABLE[(c as u8 & 0x7f) as usize] } else { c })
        .collect()
}

/// id's console-glyph → character table for the low half; the coloured (high) half maps the same
/// once the colour bit is stripped. Follows ezQuake's `readable[]` but reaches for Unicode where it
/// reads better: `0x12`–`0x1b` are the gold HUD digits, `0x10`/`0x11` the gold brackets, the dot
/// glyphs (`0x05`/`0x0e`/`0x0f`/`0x1c`) a middle dot `·`, and `0x7f` the box glyph.
#[cfg(feature = "netclient")]
const READABLE: [char; 128] = [
    ' ', '#', '#', '#', '#', '·', '#', '#', '#', '\t', '\n', '#', ' ', '\r', '·', '·', //
    '[', ']', '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '·', '<', '=', '>', //
    ' ', '!', '"', '#', '$', '%', '&', '\'', '(', ')', '*', '+', ',', '-', '.', '/', //
    '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', ':', ';', '<', '=', '>', '?', //
    '@', 'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', //
    'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z', '[', '\\', ']', '^', '_', //
    '`', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', //
    'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z', '{', '|', '}', '~', '#', //
];

/// A small builder for conchar strings: chain coloured and plain segments and separators, then
/// [`build`](Conchars::build) the finished `String`.
///
/// ```ignore
/// let name = Conchars::default().coloured("bot").ch(DOT).plain("Grunt").build();
/// assert_eq!(name, "\u{e2}\u{ef}\u{f4}\u{85}Grunt");
/// ```
#[derive(Clone, Debug, Default)]
pub(crate) struct Conchars(String);

impl Conchars {
    /// Append `s` in the coloured (high-half) conchars.
    pub(crate) fn coloured(mut self, s: &str) -> Self {
        self.0.push_str(&coloured(s));
        self
    }

    /// Append `s` verbatim (plain white text).
    pub(crate) fn plain(mut self, s: &str) -> Self {
        self.0.push_str(s);
        self
    }

    /// Append a single character — a [`DOT`] separator, say.
    pub(crate) fn ch(mut self, c: char) -> Self {
        self.0.push(c);
        self
    }

    /// The finished string.
    pub(crate) fn build(self) -> String {
        self.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The high half is a straight `| 0x80` on each byte: lowercase `bot` becomes `e2 ef f4`.
    #[test]
    fn colours_by_setting_the_high_bit() {
        assert_eq!(coloured("bot"), "\u{e2}\u{ef}\u{f4}");
        assert_eq!(coloured("").len(), 0);
    }

    /// The dot separator is conchar `0x85`.
    #[test]
    fn dot_is_conchar_133() {
        assert_eq!(DOT as u32, 0x85);
    }

    /// The builder composes coloured tag + dot + plain name into the bytes a scoreboard draws.
    #[test]
    fn builder_assembles_a_coloured_name() {
        let name = Conchars::default().coloured("bot").ch(DOT).plain("Grunt").build();
        assert_eq!(name, "\u{e2}\u{ef}\u{f4}\u{85}Grunt");
    }

    /// Latin-1 encoding keeps a high-half char one byte and drops anything above the code page (and
    /// interior NULs), so a coloured name survives into a `CString` intact.
    #[test]
    fn latin1_single_bytes_the_high_half() {
        let name = Conchars::default().coloured("bot").ch(DOT).plain("Grunt").build();
        assert_eq!(latin1_bytes(&name), vec![0xe2, 0xef, 0xf4, 0x85, b'G', b'r', b'u', b'n', b't']);
        // A char past the Latin-1 range is dropped, not mangled into UTF-8 bytes.
        assert_eq!(latin1_bytes("a\u{2022}b\0c"), vec![b'a', b'b', b'c']);
    }

    /// Logging normalizes conchars for readability: colour is stripped, the separator dot reads as
    /// a real middle dot `·`, and the gold HUD glyphs read as what they depict.
    #[cfg(feature = "netclient")]
    #[test]
    fn readable_strips_colour_and_maps_glyphs() {
        let name = Conchars::default().coloured("bot").ch(DOT).plain("Grunt").build();
        assert_eq!(readable(&name), "bot·Grunt");
        // Coloured text is just its uncoloured self.
        assert_eq!(readable(&coloured("hello")), "hello");
        // The gold HUD digits 0x12..0x1b read as 0..9; the gold brackets as [ ].
        assert_eq!(readable("\u{12}\u{1b}\u{10}\u{11}"), "09[]");
        // A real ASCII period stays a period; plain text is untouched.
        assert_eq!(readable("Grunt was gibbed."), "Grunt was gibbed.");
    }
}
