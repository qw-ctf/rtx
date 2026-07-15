// SPDX-License-Identifier: AGPL-3.0-or-later

//! Info strings — QuakeWorld's `\key\value\key\value` key-value format, used for both the client's
//! userinfo and the server's serverinfo.
//!
//! The format is as fragile as it looks: keys and values are delimited by the same character that
//! is illegal inside them, so anything containing a backslash has to be rejected rather than
//! escaped (there is no escape). Quotes and semicolons are dropped too, matching id's
//! `Info_SetValueForStarKey` — a name with a quote in it would break the `connect` packet, which
//! wraps the whole userinfo in quotes.
//!
//! Keys beginning with `*` are "star keys": the server owns them and a client can't change them
//! after connect (`*spectator`, `*z_ext`, `*version`). We write `*z_ext` exactly once, into the
//! userinfo we hand to the `connect` packet, which is the one moment it's allowed.

use std::collections::BTreeMap;

/// Longest info string a server will accept (`MAX_INFO_STRING`).
pub const MAX_INFO_STRING: usize = 512;

/// A parsed info string. Ordering is stable (sorted by key) so a rebuilt string is deterministic —
/// which makes fixtures and tests possible; the wire doesn't care about order.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Info {
    keys: BTreeMap<String, String>,
}

impl Info {
    /// An empty info string.
    pub fn new() -> Self {
        Info::default()
    }

    /// Parse a `\key\value…` string. A trailing key with no value is dropped, as is any key or
    /// value that would be illegal to write back ([`valid_token`]) — a malformed serverinfo from a
    /// hostile or buggy server shouldn't be able to smuggle a delimiter into our state.
    pub fn parse(s: &str) -> Self {
        let mut keys = BTreeMap::new();
        let mut it = s.split('\\');
        // A well-formed info string starts with the delimiter, so the first split is empty.
        if s.starts_with('\\') {
            it.next();
        }
        loop {
            let (Some(k), Some(v)) = (it.next(), it.next()) else {
                break;
            };
            if !k.is_empty() && valid_token(k) && valid_token(v) {
                keys.insert(k.to_string(), v.to_string());
            }
        }
        Info { keys }
    }

    /// Look up a key. Case-sensitive, as on the wire.
    pub fn get(&self, key: &str) -> Option<&str> {
        self.keys.get(key).map(|s| s.as_str())
    }

    /// A key parsed as `f32`, or `None` if absent or unparseable. Most numeric serverinfo keys
    /// (`maxfps`, `teamplay`, `deathmatch`) arrive as decimal text.
    pub fn get_f32(&self, key: &str) -> Option<f32> {
        self.get(key)?.trim().parse().ok()
    }

    /// A key parsed as `i32`.
    pub fn get_i32(&self, key: &str) -> Option<i32> {
        self.get_f32(key).map(|v| v as i32)
    }

    /// A key parsed as `u32`, accepting the `0x…` hex form the `*z_ext` key uses on some servers.
    pub fn get_u32(&self, key: &str) -> Option<u32> {
        let v = self.get(key)?.trim();
        match v.strip_prefix("0x").or_else(|| v.strip_prefix("0X")) {
            Some(hex) => u32::from_str_radix(hex, 16).ok(),
            None => v.parse().ok(),
        }
    }

    /// Set a key. An empty value removes it, as id's `Info_SetValueForKey` does. Returns `false`
    /// (leaving the info unchanged) if the key or value contains a character that can't survive
    /// the format.
    pub fn set(&mut self, key: &str, value: &str) -> bool {
        if key.is_empty() || !valid_token(key) || !valid_token(value) {
            return false;
        }
        if value.is_empty() {
            self.keys.remove(key);
        } else {
            self.keys.insert(key.to_string(), value.to_string());
        }
        true
    }

    /// Remove a key.
    pub fn remove(&mut self, key: &str) {
        self.keys.remove(key);
    }

    /// Every key/value pair, sorted by key.
    pub fn iter(&self) -> impl Iterator<Item = (&str, &str)> {
        self.keys.iter().map(|(k, v)| (k.as_str(), v.as_str()))
    }
}

/// Serialize back to `\key\value…` — the form that goes into a `connect` packet or a `setinfo`.
impl std::fmt::Display for Info {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        for (k, v) in &self.keys {
            write!(f, "\\{k}\\{v}")?;
        }
        Ok(())
    }
}

/// Whether a key or value can be written into an info string.
///
/// A backslash would be read back as a delimiter; a quote would terminate the userinfo early in
/// the `connect` packet; a semicolon would end the command in any console that re-parses it. The
/// format has no escapes, so the only safe answer is refusal.
pub fn valid_token(s: &str) -> bool {
    !s.contains(['\\', '"', ';']) && s.chars().all(|c| c != '\0' && (c as u32) < 256)
}

/// Build the userinfo a bot connects with.
///
/// The keys a QuakeWorld server actually reads about a player: who they are, what colours to draw
/// them, how fast to send to them, and whether they're playing. `*z_ext` announces which ZQuake
/// extensions we understand — it's a star key, settable only here, at connect.
#[derive(Clone, Debug)]
pub struct UserinfoBuilder {
    /// Player name.
    pub name: String,
    /// Team string (KTX reads this for teamplay; empty in FFA).
    pub team: String,
    /// Skin name; empty means the default `base`.
    pub skin: String,
    /// Shirt colour, 0–13.
    pub topcolor: u8,
    /// Trouser colour, 0–13.
    pub bottomcolor: u8,
    /// Bytes per second the server may send us. The stock client default.
    pub rate: u32,
    /// Message level: which console spam we're willing to receive.
    pub msg: u8,
    /// Connect as a spectator rather than a player.
    pub spectator: bool,
    /// The ZQuake extension mask to advertise.
    pub z_ext: u32,
}

impl Default for UserinfoBuilder {
    fn default() -> Self {
        UserinfoBuilder {
            name: "rtx".to_string(),
            team: String::new(),
            skin: String::new(),
            topcolor: 0,
            bottomcolor: 0,
            rate: 25000,
            msg: 1,
            spectator: false,
            z_ext: crate::protocol::Z_EXT,
        }
    }
}

impl UserinfoBuilder {
    /// Render to an [`Info`].
    ///
    /// Note what's *absent*: `pmodel` and `emodel`, the checksums of `progs/player.mdl` and
    /// `progs/eyes.mdl`. A real client can't compute those until it has loaded the model list, so
    /// they're sent later as `setinfo` during signon rather than at connect — same as ezQuake.
    pub fn build(&self) -> Info {
        let mut info = Info::new();
        info.set("name", &self.name);
        if !self.team.is_empty() {
            info.set("team", &self.team);
        }
        if !self.skin.is_empty() {
            info.set("skin", &self.skin);
        }
        info.set("topcolor", &self.topcolor.min(13).to_string());
        info.set("bottomcolor", &self.bottomcolor.min(13).to_string());
        info.set("rate", &self.rate.to_string());
        info.set("msg", &self.msg.to_string());
        if self.spectator {
            info.set("spectator", "1");
        }
        info.set("*z_ext", &self.z_ext.to_string());
        info
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The basic shape, including the leading delimiter, and a round trip through the format.
    #[test]
    fn parses_and_rebuilds() {
        let info = Info::parse("\\name\\bot\\rate\\25000");
        assert_eq!(info.get("name"), Some("bot"));
        assert_eq!(info.get("rate"), Some("25000"));
        assert_eq!(info.get("missing"), None);
        assert_eq!(info.to_string(), "\\name\\bot\\rate\\25000");
    }

    /// Real servers send keys we must tolerate: a leading delimiter or not, empty values, a
    /// trailing key with no value, and stray empty keys.
    #[test]
    fn tolerates_malformed_input() {
        assert_eq!(Info::parse("name\\bot").get("name"), Some("bot"));
        assert_eq!(Info::parse("\\name\\bot\\dangling").get("name"), Some("bot"));
        assert_eq!(Info::parse("\\name\\bot\\dangling").get("dangling"), None);
        assert_eq!(Info::parse("").iter().count(), 0);
        assert_eq!(Info::parse("\\\\value").iter().count(), 0);
    }

    /// The format has no escapes, so a token carrying a delimiter is refused rather than written —
    /// otherwise a crafted player name would let a server (or us) inject arbitrary keys.
    #[test]
    fn refuses_tokens_that_would_break_the_format() {
        let mut info = Info::new();
        assert!(!info.set("name", "evil\\team\\red"));
        assert!(!info.set("name", "quote\"name"));
        assert!(!info.set("name", "semi;colon"));
        assert!(!info.set("", "value"));
        assert_eq!(info.iter().count(), 0);

        // And the same on the way in: a hostile serverinfo can't smuggle one through parse.
        let info = Info::parse("\\name\\ok\\bad\\has\"quote");
        assert_eq!(info.get("name"), Some("ok"));
        assert_eq!(info.get("bad"), None);

        assert!(info.to_string().chars().filter(|&c| c == '\\').count() % 2 == 0);
    }

    /// Setting a key to empty removes it — id's behaviour, and how a client clears `team`.
    #[test]
    fn empty_value_removes_key() {
        let mut info = Info::parse("\\team\\red");
        assert!(info.set("team", ""));
        assert_eq!(info.get("team"), None);
    }

    /// Numeric accessors: decimal for ordinary keys, and hex for `*z_ext`, which servers send
    /// either way.
    #[test]
    fn parses_numeric_values() {
        let info = Info::parse("\\maxfps\\77\\teamplay\\2\\*z_ext\\0x1ff\\junk\\abc");
        assert_eq!(info.get_f32("maxfps"), Some(77.0));
        assert_eq!(info.get_i32("teamplay"), Some(2));
        assert_eq!(info.get_u32("*z_ext"), Some(0x1ff));
        assert_eq!(info.get_f32("junk"), None);
        assert_eq!(info.get_f32("absent"), None);

        assert_eq!(Info::parse("\\*z_ext\\511").get_u32("*z_ext"), Some(511));
    }

    /// The userinfo a bot connects with: the keys a server reads, and `*z_ext` announcing what we
    /// can parse. `pmodel`/`emodel` are deliberately not here — they're `setinfo`'d during signon.
    #[test]
    fn userinfo_builder_has_the_keys_a_server_reads() {
        let ui = UserinfoBuilder {
            name: "rtxbot".to_string(),
            team: "red".to_string(),
            ..Default::default()
        };
        let info = ui.build();
        assert_eq!(info.get("name"), Some("rtxbot"));
        assert_eq!(info.get("team"), Some("red"));
        assert_eq!(info.get("rate"), Some("25000"));
        assert_eq!(info.get("msg"), Some("1"));
        assert_eq!(info.get("*z_ext"), Some(crate::protocol::Z_EXT.to_string().as_str()));
        assert_eq!(info.get("spectator"), None);
        assert_eq!(info.get("pmodel"), None);
        assert_eq!(info.get("emodel"), None);

        // A spectator says so; colours are clamped to the palette rather than sent raw.
        let spec = UserinfoBuilder {
            spectator: true,
            topcolor: 200,
            ..Default::default()
        };
        assert_eq!(spec.build().get("spectator"), Some("1"));
        assert_eq!(spec.build().get("topcolor"), Some("13"));
    }

    /// Userinfo goes inside quotes in the `connect` packet and must fit `MAX_INFO_STRING`.
    #[test]
    fn userinfo_fits_a_connect_packet() {
        let s = UserinfoBuilder::default().build().to_string();
        assert!(s.len() < MAX_INFO_STRING, "{} bytes", s.len());
        assert!(!s.contains('"'));
    }
}
