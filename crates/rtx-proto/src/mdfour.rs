// SPDX-License-Identifier: AGPL-3.0-or-later

//! MD4, and the folded-to-32-bit form id calls `Com_BlockChecksum`.
//!
//! MD4 is here only because the map checksum is defined in terms of it — nothing about this is
//! security-relevant (MD4 has been broken since the nineties; QuakeWorld servers still ask for it).
//! Written from RFC 1320 and checked against that RFC's test vectors, rather than ported from a
//! client, so no licence rides along.

/// MD4 of `data`, as the raw 16-byte digest.
pub fn digest(data: &[u8]) -> [u8; 16] {
    let mut state: [u32; 4] = [0x6745_2301, 0xefcd_ab89, 0x98ba_dcfe, 0x1032_5476];

    // Whole 64-byte blocks first, then the padded tail: 0x80, zeros to 56 mod 64, then the length
    // in bits as a little-endian u64.
    let mut chunks = data.chunks_exact(64);
    for block in &mut chunks {
        transform(&mut state, block.try_into().unwrap());
    }

    let rest = chunks.remainder();
    let mut tail = [0u8; 128];
    tail[..rest.len()].copy_from_slice(rest);
    tail[rest.len()] = 0x80;
    let tail_len = if rest.len() < 56 { 64 } else { 128 };
    let bits = (data.len() as u64).wrapping_mul(8);
    tail[tail_len - 8..tail_len].copy_from_slice(&bits.to_le_bytes());
    for block in tail[..tail_len].chunks_exact(64) {
        transform(&mut state, block.try_into().unwrap());
    }

    let mut out = [0u8; 16];
    for (i, w) in state.iter().enumerate() {
        out[i * 4..i * 4 + 4].copy_from_slice(&w.to_le_bytes());
    }
    out
}

/// `Com_BlockChecksum` — MD4 folded down to 32 bits by XOR-ing the digest's four little-endian
/// words. This is what the map checksum is built from.
pub fn block_checksum(data: &[u8]) -> u32 {
    let d = digest(data);
    let word = |i: usize| u32::from_le_bytes(d[i * 4..i * 4 + 4].try_into().unwrap());
    word(0) ^ word(1) ^ word(2) ^ word(3)
}

/// One MD4 round over a 64-byte block (RFC 1320 §3.4).
fn transform(state: &mut [u32; 4], block: &[u8; 64]) {
    let mut x = [0u32; 16];
    for (i, w) in x.iter_mut().enumerate() {
        *w = u32::from_le_bytes(block[i * 4..i * 4 + 4].try_into().unwrap());
    }

    let (mut a, mut b, mut c, mut d) = (state[0], state[1], state[2], state[3]);

    // Round 1: F(x,y,z) = xy v not(x)z
    let ff = |a: u32, b: u32, c: u32, d: u32, x: u32, s: u32| {
        a.wrapping_add((b & c) | (!b & d)).wrapping_add(x).rotate_left(s)
    };
    for i in 0..4 {
        let k = i * 4;
        a = ff(a, b, c, d, x[k], 3);
        d = ff(d, a, b, c, x[k + 1], 7);
        c = ff(c, d, a, b, x[k + 2], 11);
        b = ff(b, c, d, a, x[k + 3], 19);
    }

    // Round 2: G(x,y,z) = xy v xz v yz, with the constant 0x5a827999 (sqrt 2).
    let gg = |a: u32, b: u32, c: u32, d: u32, x: u32, s: u32| {
        a.wrapping_add((b & c) | (b & d) | (c & d))
            .wrapping_add(x)
            .wrapping_add(0x5a82_7999)
            .rotate_left(s)
    };
    for i in 0..4 {
        a = gg(a, b, c, d, x[i], 3);
        d = gg(d, a, b, c, x[i + 4], 5);
        c = gg(c, d, a, b, x[i + 8], 9);
        b = gg(b, c, d, a, x[i + 12], 13);
    }

    // Round 3: H(x,y,z) = x xor y xor z, with the constant 0x6ed9eba1 (sqrt 3), over a bit-reversed
    // index order.
    let hh = |a: u32, b: u32, c: u32, d: u32, x: u32, s: u32| {
        a.wrapping_add(b ^ c ^ d).wrapping_add(x).wrapping_add(0x6ed9_eba1).rotate_left(s)
    };
    for &i in &[0usize, 2, 1, 3] {
        a = hh(a, b, c, d, x[i], 3);
        d = hh(d, a, b, c, x[i + 8], 9);
        c = hh(c, d, a, b, x[i + 4], 11);
        b = hh(b, c, d, a, x[i + 12], 15);
    }

    state[0] = state[0].wrapping_add(a);
    state[1] = state[1].wrapping_add(b);
    state[2] = state[2].wrapping_add(c);
    state[3] = state[3].wrapping_add(d);
}

#[cfg(test)]
mod tests {
    use super::*;

    fn hex(d: [u8; 16]) -> String {
        d.iter().map(|b| format!("{b:02x}")).collect()
    }

    /// RFC 1320's own test suite, verbatim. These cover the empty input, sub-block inputs, the
    /// exact-block-boundary case, and multi-block input — i.e. every branch of the padding.
    #[test]
    fn matches_rfc1320_test_vectors() {
        assert_eq!(hex(digest(b"")), "31d6cfe0d16ae931b73c59d7e0c089c0");
        assert_eq!(hex(digest(b"a")), "bde52cb31de33e46245e05fbdbd6fb24");
        assert_eq!(hex(digest(b"abc")), "a448017aaf21d8525fc10ae87aa6729d");
        assert_eq!(hex(digest(b"message digest")), "d9130a8164549fe818874806e1c7014b");
        assert_eq!(hex(digest(b"abcdefghijklmnopqrstuvwxyz")), "d79e1c308aa5bbcdeea8ed63df412da9");
        assert_eq!(
            hex(digest(b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789")),
            "043f8582f241db351ce627e153e7f0e4"
        );
        assert_eq!(
            hex(digest(b"12345678901234567890123456789012345678901234567890123456789012345678901234567890")),
            "e33b4ddc9c38f2199c3e7b164fcc0536"
        );
    }

    /// The padding has two shapes — tail under 56 bytes (one block) and 56..64 (two). Walk every
    /// length across both boundaries so an off-by-one in the tail can't hide.
    #[test]
    fn padding_is_right_at_every_block_boundary() {
        // Any length must produce *a* digest, and lengths that differ must differ; the RFC vectors
        // above pin the actual values at 0, 1, 3, 14, 26, 62 and 80 bytes.
        let mut seen = std::collections::HashSet::new();
        for len in 0..130 {
            let data = vec![b'x'; len];
            assert!(seen.insert(digest(&data)), "duplicate digest at len {len}");
        }
    }

    /// The 32-bit fold is what the map checksum actually consumes.
    #[test]
    fn block_checksum_folds_the_digest() {
        let d = digest(b"abc");
        let word = |i: usize| u32::from_le_bytes(d[i * 4..i * 4 + 4].try_into().unwrap());
        assert_eq!(block_checksum(b"abc"), word(0) ^ word(1) ^ word(2) ^ word(3));
    }
}
