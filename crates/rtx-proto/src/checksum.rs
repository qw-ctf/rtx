// SPDX-License-Identifier: AGPL-3.0-or-later

//! The map checksum a client must present at `prespawn`.
//!
//! The server asks "which map do you have?" and refuses the connection unless the answer matches
//! its own. The answer is [`map_checksum2`]: XOR the [`block_checksum`](crate::mdfour::block_checksum)
//! of each BSP lump, skipping four of them, then run the result through [`translate`].
//!
//! Which lumps are skipped is the whole subtlety. `checksum2` omits **entities** (the server
//! rewrites them), and **visibility, leafs and nodes** (the render tree, which mappers re-light
//! and re-vis without changing gameplay). What's left — planes, clipnodes, models, textures, the
//! geometry a player can actually collide with — is the same on both ends or the map is genuinely
//! different. There's a sibling `checksum1` that omits only entities; the server keeps that one for
//! its own use and never asks us for it.
//!
//! Ported from ezQuake's `CM_CalcChecksum` (`src/cmodel.c`) and qualia's `Bsp::checksums`
//! (`src/assets/bsp.cppm`), which agree.

use crate::mdfour::block_checksum;

/// Number of lumps in the BSP directory (`HEADER_LUMPS`).
const HEADER_LUMPS: usize = 15;

const LUMP_ENTITIES: usize = 0;
const LUMP_VISIBILITY: usize = 4;
const LUMP_NODES: usize = 5;
const LUMP_LEAFS: usize = 10;

/// The BSP is too short, or a lump points outside it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MalformedBsp;

impl std::fmt::Display for MalformedBsp {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("malformed BSP: truncated header or out-of-range lump")
    }
}

impl std::error::Error for MalformedBsp {}

/// Compute the `checksum2` a client sends with `prespawn`, already [`translate`]d for the map name.
///
/// `mapname` is the bare name as the server knows it (`"dm4"`, not `"maps/dm4.bsp"`).
pub fn map_checksum2(bsp: &[u8], mapname: &str) -> Result<i32, MalformedBsp> {
    Ok(translate(mapname, raw_checksum2(bsp)?))
}

/// The untranslated `checksum2`: XOR of the per-lump checksums, skipping entities, visibility,
/// nodes and leafs.
pub fn raw_checksum2(bsp: &[u8]) -> Result<i32, MalformedBsp> {
    // Header: a version long, then 15 (offset, length) pairs.
    if bsp.len() < 4 + HEADER_LUMPS * 8 {
        return Err(MalformedBsp);
    }
    let mut sum: u32 = 0;
    for i in 0..HEADER_LUMPS {
        if matches!(i, LUMP_ENTITIES | LUMP_VISIBILITY | LUMP_NODES | LUMP_LEAFS) {
            continue;
        }
        let base = 4 + i * 8;
        let off = u32::from_le_bytes(bsp[base..base + 4].try_into().unwrap()) as usize;
        let len = u32::from_le_bytes(bsp[base + 4..base + 8].try_into().unwrap()) as usize;
        let end = off.checked_add(len).ok_or(MalformedBsp)?;
        if end > bsp.len() {
            return Err(MalformedBsp);
        }
        sum ^= block_checksum(&bsp[off..end]);
    }
    Ok(sum as i32)
}

/// The maps with a freely-licensed rebuild in circulation: `(name, id original, GPL rebuild)`.
/// From ezQuake's `src/common.c`. See [`translate`].
const TABLE: &[(&str, u32, u32)] = &[
    // AquaShark's "simpletextures" maps.
    ("dm1", 0xc5c7_dab3, 0x7d37_618e),
    ("dm2", 0x65f6_3634, 0x7b33_7440),
    ("dm3", 0x15e2_0df8, 0x9127_81ae),
    ("dm4", 0x9c6f_e4bf, 0xc374_df89),
    ("dm5", 0xb02d_48fd, 0x77ca_7ce5),
    ("dm6", 0x5208_da2b, 0x200c_8b5d),
    ("end", 0xbbd4_b4a5, 0xf89b_12ae),  // the version with the extra room
    ("end", 0xbbd4_b4a5, 0x924f_4d33),  // GPL end
    ("e2m2", 0xaf96_1d4d, 0xa231_26c5), // GPL e2m2
];

/// id's "GPL map" fixup (`Com_TranslateMapChecksum`).
///
/// Some id maps were redistributed as freely-licensed rebuilds with identical layout but different
/// bytes. A client holding one of those reports the *original* map's checksum, so it can still play
/// alongside clients holding the id original. Without this table a player on a GPL `dm4` is
/// refused by a server on the id `dm4` and vice versa — so it's not optional, even for a bot.
pub fn translate(mapname: &str, checksum: i32) -> i32 {
    for &(name, original, gpl) in TABLE {
        if name == mapname && checksum as u32 == gpl {
            return original as i32;
        }
    }
    checksum
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a BSP header with the given (offset, len) lumps over a body of `body_len` bytes.
    fn synth_bsp(lumps: [(u32, u32); HEADER_LUMPS], body_len: usize) -> Vec<u8> {
        let mut b = Vec::new();
        b.extend_from_slice(&29u32.to_le_bytes());
        for (off, len) in lumps {
            b.extend_from_slice(&off.to_le_bytes());
            b.extend_from_slice(&len.to_le_bytes());
        }
        let header = b.len();
        b.resize(header + body_len, 0);
        for (i, byte) in b[header..].iter_mut().enumerate() {
            *byte = (i % 251) as u8; // deterministic, non-uniform filler
        }
        b
    }

    /// The four skipped lumps are the point of `checksum2`: a server that re-vis'd or re-lit a map,
    /// or that rewrote its entities, must still accept us. Changing bytes in any of them must not
    /// move the checksum; changing a geometry lump must.
    #[test]
    fn skips_entities_visibility_nodes_and_leafs() {
        let body = 64usize;
        let base = 4 + HEADER_LUMPS as u32 * 8;
        let lumps = std::array::from_fn(|i| (base + (i as u32 * 4), 4));
        let bsp = synth_bsp(lumps, body);
        let baseline = raw_checksum2(&bsp).unwrap();

        for skipped in [LUMP_ENTITIES, LUMP_VISIBILITY, LUMP_NODES, LUMP_LEAFS] {
            let mut m = bsp.clone();
            let off = (base + skipped as u32 * 4) as usize;
            m[off] ^= 0xff;
            assert_eq!(raw_checksum2(&m).unwrap(), baseline, "lump {skipped} should be excluded");
        }

        // Planes (1) and clipnodes (9) are geometry — they must count.
        for counted in [1usize, 9] {
            let mut m = bsp.clone();
            let off = (base + counted as u32 * 4) as usize;
            m[off] ^= 0xff;
            assert_ne!(raw_checksum2(&m).unwrap(), baseline, "lump {counted} should be included");
        }
    }

    /// A lump pointing past the end of the file is a corrupt map, not a panic.
    #[test]
    fn rejects_out_of_range_lumps() {
        let mut lumps = [(0u32, 0u32); HEADER_LUMPS];
        lumps[1] = (4 + HEADER_LUMPS as u32 * 8, 9999);
        let bsp = synth_bsp(lumps, 16);
        assert_eq!(raw_checksum2(&bsp), Err(MalformedBsp));

        assert_eq!(raw_checksum2(&[0u8; 8]), Err(MalformedBsp));

        // An offset+length that overflows a usize must not wrap into a valid-looking range.
        let mut lumps = [(0u32, 0u32); HEADER_LUMPS];
        lumps[1] = (u32::MAX, u32::MAX);
        let bsp = synth_bsp(lumps, 16);
        assert_eq!(raw_checksum2(&bsp), Err(MalformedBsp));
    }

    /// The GPL fixup only fires for the exact (map, checksum) pair — anything else passes through
    /// untouched, including the same checksum under a different map name.
    #[test]
    fn translates_only_known_gpl_maps() {
        assert_eq!(translate("dm4", 0xc374_df89u32 as i32), 0x9c6f_e4bfu32 as i32);
        assert_eq!(translate("dm4", 0x1234_5678), 0x1234_5678);
        assert_eq!(translate("dm3", 0xc374_df89u32 as i32), 0xc374_df89u32 as i32);
        assert_eq!(translate("aerowalk", 0xdead_beefu32 as i32), 0xdead_beefu32 as i32);

        // `end` appears twice with two different GPL rebuilds; both map to the same original.
        assert_eq!(translate("end", 0xf89b_12aeu32 as i32), 0xbbd4_b4a5u32 as i32);
        assert_eq!(translate("end", 0x924f_4d33u32 as i32), 0xbbd4_b4a5u32 as i32);
    }

    /// Checksum a real map (path from `RTX_TEST_BSP`, e.g. a Quake `dm4.bsp`) — the same opt-in
    /// idiom as the rtx-nav BSP tests, so CI is green without a map checked in.
    ///
    /// For the six id deathmatch maps this is a **golden test with a free oracle**: the `original`
    /// column of [`translate`]'s table *is* the checksum2 of the id original, computed by ezQuake's
    /// authors with an independent implementation. Matching all six pins the MD4, the 32-bit fold,
    /// the excluded-lump set and the XOR at once — a bug in any of them moves the number.
    #[test]
    fn checksums_a_real_bsp() {
        let Ok(path) = std::env::var("RTX_TEST_BSP") else {
            eprintln!("RTX_TEST_BSP not set; skipping");
            return;
        };
        let bytes = std::fs::read(&path).expect("read bsp");
        let name = std::path::Path::new(&path)
            .file_stem()
            .and_then(|s| s.to_str())
            .unwrap_or("");
        let raw = raw_checksum2(&bytes).expect("checksum");
        let sum = map_checksum2(&bytes, name).expect("checksum");
        eprintln!("{name}: checksum2 = 0x{:08x}", sum as u32);

        // Deterministic, with the GPL translation layered on top of the raw value.
        assert_eq!(sum, map_checksum2(&bytes, name).unwrap());
        assert_eq!(sum, translate(name, raw));

        // The oracle: an id original must hash to the value id's own table calls original. (A GPL
        // rebuild of the same map hashes to `gpl` and translates *to* `original` — either way the
        // translated answer is the same, which is the point of the table.)
        if let Some(&(_, original, gpl)) = TABLE.iter().find(|&&(n, ..)| n == name) {
            assert!(
                raw as u32 == original || raw as u32 == gpl,
                "{name}: 0x{:08x} is neither the id original (0x{original:08x}) nor the GPL rebuild \
                 (0x{gpl:08x}) — checksum2 is wrong, or this is a third variant of the map",
                raw as u32
            );
            assert_eq!(sum as u32, original, "{name}: translation should land on the id original");
        }
    }
}
