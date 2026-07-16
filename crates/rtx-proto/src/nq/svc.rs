// SPDX-License-Identifier: AGPL-3.0-or-later

//! The NetQuake server→client parser: bytes in, [`SvcEvent`]s out.
//!
//! It emits the *same* [`SvcEvent`] vocabulary as the QuakeWorld parser ([`crate::svc`]) so a
//! consumer matches one enum for both protocols. The divergences are all here: NetQuake's own opcode
//! table ([`op`]), its fast-entity-update bit layout ([`u`]), its monolithic `svc_clientdata`
//! ([`su`]), an inline-precache `svc_serverinfo`, and a temp-entity table that differs from
//! QuakeWorld's at bytes 2/12/13. Widths come from [`NqProtoState`], never from the [`Reader`]'s own
//! coord/angle fields.
//!
//! **Delta base.** A NetQuake fast update is delta'd from the entity's *baseline*, not the previous
//! frame — the opposite of QuakeWorld. The parser is stateless, so it just reports which fields the
//! update carried ([`SvcEvent::EntityUpdate`]); the frame store resolves the absent ones.
//!
//! **Unknown opcode is fatal**, as in the QuakeWorld parser: the stream is only self-delimiting if
//! every message is understood. Extension opcodes (`svcdp_*`, `svcfte_*`) fall here — we declined
//! every FTE `PEXT`, so a well-behaved server never sends them.
//!
//! Ported from QuakeSpasm-Spiked `Quake/cl_parse.c`, `Quake/cl_tent.c` and `Quake/protocol.h`.

use super::protocol::NqProtoState;
use crate::sizebuf::Reader;
use crate::svc::{Baseline, ClientData, EntityDelta, NqServerData, ParseError, SvcEvent};
use crate::svc::{TempEntity, TempEntityKind};
use glam::Vec3;

/// NetQuake server→client opcodes (`protocol.h`). Only the ones a vanilla 15/666/999 stream can send
/// are named; extension opcodes are deliberately absent and hit [`ParseError::UnknownSvc`].
mod op {
    pub const BAD: u8 = 0;
    pub const NOP: u8 = 1;
    pub const DISCONNECT: u8 = 2;
    pub const UPDATESTAT: u8 = 3;
    pub const VERSION: u8 = 4;
    pub const SETVIEW: u8 = 5;
    pub const SOUND: u8 = 6;
    pub const TIME: u8 = 7;
    pub const PRINT: u8 = 8;
    pub const STUFFTEXT: u8 = 9;
    pub const SETANGLE: u8 = 10;
    pub const SERVERINFO: u8 = 11;
    pub const LIGHTSTYLE: u8 = 12;
    pub const UPDATENAME: u8 = 13;
    pub const UPDATEFRAGS: u8 = 14;
    pub const CLIENTDATA: u8 = 15;
    pub const STOPSOUND: u8 = 16;
    pub const UPDATECOLORS: u8 = 17;
    pub const PARTICLE: u8 = 18;
    pub const DAMAGE: u8 = 19;
    pub const SPAWNSTATIC: u8 = 20;
    pub const SPAWNBASELINE: u8 = 22;
    pub const TEMP_ENTITY: u8 = 23;
    pub const SETPAUSE: u8 = 24;
    pub const SIGNONNUM: u8 = 25;
    pub const CENTERPRINT: u8 = 26;
    pub const KILLEDMONSTER: u8 = 27;
    pub const FOUNDSECRET: u8 = 28;
    pub const SPAWNSTATICSOUND: u8 = 29;
    pub const INTERMISSION: u8 = 30;
    pub const FINALE: u8 = 31;
    pub const CDTRACK: u8 = 32;
    pub const SELLSCREEN: u8 = 33;
    pub const CUTSCENE: u8 = 34;
    // FitzQuake (666/999) additions the client must tolerate even if it ignores them.
    pub const SKYBOX: u8 = 37;
    pub const BF: u8 = 40;
    pub const FOG: u8 = 41;
    pub const SPAWNBASELINE2: u8 = 42;
    pub const SPAWNSTATIC2: u8 = 43;
    pub const SPAWNSTATICSOUND2: u8 = 44;
}

/// Fast-update bits (`U_*`). The first byte's high bit ([`SIGNAL`]) flags a fast update and its low
/// seven bits are the first bit-byte; [`MOREBITS`]/[`EXTEND1`]/[`EXTEND2`] chain in further bytes.
mod u {
    pub const MOREBITS: u32 = 1 << 0;
    pub const ORIGIN1: u32 = 1 << 1;
    pub const ORIGIN2: u32 = 1 << 2;
    pub const ORIGIN3: u32 = 1 << 3;
    pub const ANGLE2: u32 = 1 << 4;
    // Bit 5 is U_STEP, a lerp hint with no payload — the parser ignores it.
    pub const FRAME: u32 = 1 << 6;
    pub const SIGNAL: u8 = 1 << 7;
    pub const ANGLE1: u32 = 1 << 8;
    pub const ANGLE3: u32 = 1 << 9;
    pub const MODEL: u32 = 1 << 10;
    pub const COLORMAP: u32 = 1 << 11;
    pub const SKIN: u32 = 1 << 12;
    pub const EFFECTS: u32 = 1 << 13;
    pub const LONGENTITY: u32 = 1 << 14;
    pub const EXTEND1: u32 = 1 << 15;
    /// On protocol 15 this bit is the Nehahra transparency hack instead of an extend byte.
    pub const TRANS: u32 = 1 << 15;
    pub const ALPHA: u32 = 1 << 16;
    pub const FRAME2: u32 = 1 << 17;
    pub const MODEL2: u32 = 1 << 18;
    pub const LERPFINISH: u32 = 1 << 19;
    pub const SCALE: u32 = 1 << 20;
    pub const EXTEND2: u32 = 1 << 23;
}

/// `svc_clientdata` bits (`SU_*`).
mod su {
    pub const VIEWHEIGHT: u32 = 1 << 0;
    pub const IDEALPITCH: u32 = 1 << 1;
    pub const PUNCH1: u32 = 1 << 2;
    pub const VELOCITY1: u32 = 1 << 5;
    // Bit 9 is SU_ITEMS; a non-DP7 server always sets it, so the parser reads items unconditionally.
    pub const ONGROUND: u32 = 1 << 10;
    pub const INWATER: u32 = 1 << 11;
    pub const WEAPONFRAME: u32 = 1 << 12;
    pub const ARMOR: u32 = 1 << 13;
    pub const WEAPON: u32 = 1 << 14;
    pub const EXTEND1: u32 = 1 << 15;
    pub const WEAPON2: u32 = 1 << 16;
    pub const ARMOR2: u32 = 1 << 17;
    pub const AMMO2: u32 = 1 << 18;
    pub const SHELLS2: u32 = 1 << 19;
    pub const NAILS2: u32 = 1 << 20;
    pub const ROCKETS2: u32 = 1 << 21;
    pub const CELLS2: u32 = 1 << 22;
    pub const EXTEND2: u32 = 1 << 23;
    pub const WEAPONFRAME2: u32 = 1 << 24;
    pub const WEAPONALPHA: u32 = 1 << 25;
}

/// `svc_sound` field-mask bits (`SND_*`). Only the vanilla/FitzQuake ones; the FTE flags never
/// appear because we decline `PEXT`.
mod snd {
    pub const VOLUME: u8 = 1 << 0;
    pub const ATTENUATION: u8 = 1 << 1;
    pub const LARGEENTITY: u8 = 1 << 3;
    pub const LARGESOUND: u8 = 1 << 4;
}

/// Baseline flags for `svc_spawnbaseline2`/`svc_spawnstatic2` (`B_*`).
mod b {
    pub const LARGEMODEL: u8 = 1 << 0;
    pub const LARGEFRAME: u8 = 1 << 1;
    pub const ALPHA: u8 = 1 << 2;
    pub const SCALE: u8 = 1 << 3;
}

const DEFAULT_VIEWHEIGHT: i16 = 22;
const DEFAULT_SOUND_VOLUME: u8 = 255;
const DEFAULT_SOUND_ATTENUATION: f32 = 1.0;
const PRINT_HIGH: u8 = 2;

/// Parse every message in one NetQuake datagram (a reliable message or an unreliable frame).
///
/// `proto` is updated in place when `svc_serverinfo` sets the protocol version and flags, and must
/// be, because those widths apply to the rest of *this* datagram. On any error the events parsed so
/// far are discarded — a desynced stream is untrustworthy from the first misread byte.
pub fn parse(proto: &mut NqProtoState, data: &[u8]) -> Result<Vec<SvcEvent>, ParseError> {
    let mut r = Reader::new(data);
    let mut out = Vec::new();
    while !r.at_end() {
        let offset = r.pos();
        let cmd = r.u8()?;
        if cmd & u::SIGNAL != 0 {
            // The low seven bits are the first fast-update bit-byte.
            let delta = read_delta_entity(proto, &mut r, (cmd & 0x7f) as u32)?;
            out.push(SvcEvent::EntityUpdate(delta));
            continue;
        }
        if let Some(ev) = parse_one(proto, &mut r, cmd, offset)? {
            out.push(ev);
        }
    }
    Ok(out)
}

/// Parse one non-fast-update message. `None` means "recognised but carries no event" (a skybox
/// name, a `bf` flash — parsed for its bytes, then dropped).
fn parse_one(
    proto: &mut NqProtoState,
    r: &mut Reader,
    cmd: u8,
    offset: usize,
) -> Result<Option<SvcEvent>, ParseError> {
    let ev = match cmd {
        op::BAD => return Err(ParseError::Bad { offset }),
        op::NOP => SvcEvent::Nop,
        op::DISCONNECT => SvcEvent::Disconnect,
        op::UPDATESTAT => SvcEvent::UpdateStat { stat: r.u8()?, value: r.i32()? },
        op::VERSION => {
            // The version is authoritative from serverinfo; a later svc_version just re-states it.
            r.i32()?;
            return Ok(None);
        }
        op::SETVIEW => SvcEvent::SetView(r.u16()?),
        op::SOUND => read_sound(proto, r)?,
        op::TIME => SvcEvent::Time(r.f32()?),
        // NetQuake's svc_print carries no level byte; treat server prints as high priority.
        op::PRINT => SvcEvent::Print { level: PRINT_HIGH, text: r.string()? },
        // A 0x01-prefixed stufftext is a binary ProQuake message whose args are NUL-free by design,
        // so reading it as a string consumes exactly its bytes; the session ignores the \x01 text.
        op::STUFFTEXT => SvcEvent::StuffText(r.string()?),
        op::SETANGLE => SvcEvent::SetAngle { kind: None, angles: proto.angle3(r)? },
        op::SERVERINFO => read_serverinfo(proto, r)?,
        op::LIGHTSTYLE => SvcEvent::LightStyle { index: r.u8()?, pattern: r.string()? },
        op::UPDATENAME => SvcEvent::UpdateName { player: r.u8()?, name: r.string()? },
        op::UPDATEFRAGS => SvcEvent::UpdateFrags { player: r.u8()?, frags: r.i16()? },
        op::CLIENTDATA => SvcEvent::ClientData(Box::new(read_clientdata(proto, r)?)),
        op::STOPSOUND => {
            let w = r.u16()?;
            SvcEvent::StopSound { entity: w >> 3, channel: (w & 7) as u8 }
        }
        op::UPDATECOLORS => SvcEvent::UpdateColors { player: r.u8()?, colors: r.u8()? },
        op::PARTICLE => {
            let origin = proto.coord3(r)?;
            // Direction is three signed bytes in 1/16-unit steps.
            let dir = Vec3::new(r.i8()? as f32 / 16.0, r.i8()? as f32 / 16.0, r.i8()? as f32 / 16.0);
            SvcEvent::Particle { origin, dir, count: r.u8()?, color: r.u8()? }
        }
        op::DAMAGE => SvcEvent::Damage { armor: r.u8()?, blood: r.u8()?, from: proto.coord3(r)? },
        op::SPAWNSTATIC => SvcEvent::SpawnStatic(read_baseline(proto, r, 1)?),
        op::SPAWNBASELINE => {
            let entity = r.u16()?;
            SvcEvent::SpawnBaseline { entity, baseline: read_baseline(proto, r, 1)? }
        }
        op::SPAWNBASELINE2 => {
            let entity = r.u16()?;
            SvcEvent::SpawnBaseline { entity, baseline: read_baseline(proto, r, 2)? }
        }
        op::SPAWNSTATIC2 => SvcEvent::SpawnStatic(read_baseline(proto, r, 2)?),
        op::TEMP_ENTITY => read_temp_entity(proto, r)?,
        op::SETPAUSE => SvcEvent::SetPause(r.u8()? != 0),
        op::SIGNONNUM => SvcEvent::SignonNum(r.u8()?),
        op::CENTERPRINT => SvcEvent::CenterPrint(r.string()?),
        op::KILLEDMONSTER => SvcEvent::KilledMonster,
        op::FOUNDSECRET => SvcEvent::FoundSecret,
        op::SPAWNSTATICSOUND => SvcEvent::SpawnStaticSound {
            origin: proto.coord3(r)?,
            sound: r.u8()? as u16,
            volume: r.u8()?,
            attenuation: r.u8()?,
        },
        op::SPAWNSTATICSOUND2 => SvcEvent::SpawnStaticSound {
            origin: proto.coord3(r)?,
            sound: r.u16()?,
            volume: r.u8()?,
            attenuation: r.u8()?,
        },
        // NetQuake's svc_intermission carries no camera position (the client freezes the view).
        op::INTERMISSION => SvcEvent::Intermission { origin: Vec3::ZERO, angles: Vec3::ZERO },
        op::FINALE => SvcEvent::Finale(r.string()?),
        // A cutscene is an intermission with text; the session treats it the same.
        op::CUTSCENE => SvcEvent::Finale(r.string()?),
        op::CDTRACK => {
            let track = r.u8()?;
            r.u8()?; // looptrack, unused
            SvcEvent::CdTrack(track)
        }
        op::SELLSCREEN => SvcEvent::SellScreen,
        // Parsed for their bytes, then dropped — a 666/999 server may send them on any map.
        op::SKYBOX => {
            r.string()?;
            return Ok(None);
        }
        op::BF => return Ok(None),
        op::FOG => {
            r.skip(4)?; // density, r, g, b
            r.i16()?; // time
            return Ok(None);
        }
        other => return Err(ParseError::UnknownSvc { svc: other, offset }),
    };
    Ok(Some(ev))
}

/// A NetQuake fast entity update. `first_bits` is the low seven bits from the opcode byte.
fn read_delta_entity(
    proto: &NqProtoState,
    r: &mut Reader,
    first_bits: u32,
) -> Result<EntityDelta, ParseError> {
    let mut bits = first_bits;
    if bits & u::MOREBITS != 0 {
        bits |= (r.u8()? as u32) << 8;
    }
    // The extend bytes exist only on FitzQuake/RMQ; on protocol 15, bit 15 is U_TRANS instead.
    let fitz = proto.version == super::protocol::FITZQUAKE || proto.version == super::protocol::RMQ;
    if fitz {
        if bits & u::EXTEND1 != 0 {
            bits |= (r.u8()? as u32) << 16;
        }
        if bits & u::EXTEND2 != 0 {
            bits |= (r.u8()? as u32) << 24;
        }
    }

    let mut d = EntityDelta {
        number: if bits & u::LONGENTITY != 0 { r.u16()? } else { r.u8()? as u16 },
        ..Default::default()
    };

    if bits & u::MODEL != 0 {
        d.model = Some(r.u8()? as u16);
    }
    if bits & u::FRAME != 0 {
        d.frame = Some(r.u8()? as u16);
    }
    if bits & u::COLORMAP != 0 {
        d.colormap = Some(r.u8()?);
    }
    if bits & u::SKIN != 0 {
        d.skin = Some(r.u8()?);
    }
    if bits & u::EFFECTS != 0 {
        d.effects = Some(r.u8()?);
    }
    // Origin and angle interleave, one axis at a time.
    if bits & u::ORIGIN1 != 0 {
        d.origin[0] = Some(proto.coord(r)?);
    }
    if bits & u::ANGLE1 != 0 {
        d.angles[0] = Some(proto.angle(r)?);
    }
    if bits & u::ORIGIN2 != 0 {
        d.origin[1] = Some(proto.coord(r)?);
    }
    if bits & u::ANGLE2 != 0 {
        d.angles[1] = Some(proto.angle(r)?);
    }
    if bits & u::ORIGIN3 != 0 {
        d.origin[2] = Some(proto.coord(r)?);
    }
    if bits & u::ANGLE3 != 0 {
        d.angles[2] = Some(proto.angle(r)?);
    }
    // U_STEP is a lerp hint with no payload.

    if fitz {
        if bits & u::ALPHA != 0 {
            d.trans = Some(r.u8()?);
        }
        if bits & u::SCALE != 0 {
            r.u8()?; // RMQ edict scale, unused
        }
        if bits & u::FRAME2 != 0 {
            // High byte of a large frame; combine with the low byte if that was sent too.
            let hi = (r.u8()? as u16) << 8;
            d.frame = Some(d.frame.unwrap_or(0) | hi);
        }
        if bits & u::MODEL2 != 0 {
            let hi = (r.u8()? as u16) << 8;
            d.model = Some(d.model.unwrap_or(0) | hi);
        }
        if bits & u::LERPFINISH != 0 {
            r.u8()?; // lerp finish time, unused
        }
    } else if bits & u::TRANS != 0 {
        // Nehahra transparency hack on protocol 15: read its floats so the stream stays aligned.
        let a = r.f32()?;
        r.f32()?; // alpha
        if a == 2.0 {
            r.f32()?; // fullbright flag
        }
    }

    Ok(d)
}

/// `svc_serverinfo` — protocol, flags, maxclients, gametype, level name, then the two inline
/// precache lists. Mutates `proto`, whose new widths apply to the rest of the datagram.
fn read_serverinfo(proto: &mut NqProtoState, r: &mut Reader) -> Result<SvcEvent, ParseError> {
    // Tolerate (but never expect) leading FTE pext magic longs — we declined them, so a compliant
    // server omits them, but a stray one must be skipped rather than read as the protocol version.
    let mut protocol = r.i32()? as u32;
    while protocol == crate::protocol::magic::FTE || protocol == crate::protocol::magic::FTE2 {
        r.i32()?; // the extension mask we didn't ask for
        protocol = r.i32()? as u32;
    }
    let flags = if protocol == super::protocol::RMQ { r.u32()? } else { 0 };
    proto.set(protocol, flags);

    let maxclients = r.u8()?;
    let gametype = r.u8()?;
    let levelname = r.string()?;
    let models = read_precache_list(r)?;
    let sounds = read_precache_list(r)?;

    Ok(SvcEvent::NqServerData(Box::new(NqServerData {
        protocol,
        flags,
        maxclients,
        gametype,
        levelname,
        models,
        sounds,
    })))
}

/// A NUL-string-terminated precache list. The wire is 1-indexed (index 0 is a reserved empty slot),
/// so the returned Vec keeps a leading empty string and a wire index dereferences it directly.
fn read_precache_list(r: &mut Reader) -> Result<Vec<String>, ParseError> {
    let mut names = vec![String::new()];
    loop {
        let name = r.string()?;
        if name.is_empty() {
            break;
        }
        names.push(name);
    }
    Ok(names)
}

/// `svc_clientdata` — our whole own-player state in one bitfield.
fn read_clientdata(proto: &NqProtoState, r: &mut Reader) -> Result<ClientData, ParseError> {
    let mut bits = r.u16()? as u32;
    if bits & su::EXTEND1 != 0 {
        bits |= (r.u8()? as u32) << 16;
    }
    if bits & su::EXTEND2 != 0 {
        bits |= (r.u8()? as u32) << 24;
    }
    let _ = proto; // widths don't reach clientdata (velocity is char*16, punch is char)

    let mut cd = ClientData {
        viewheight: if bits & su::VIEWHEIGHT != 0 { r.i8()? as i16 } else { DEFAULT_VIEWHEIGHT },
        ..Default::default()
    };
    if bits & su::IDEALPITCH != 0 {
        cd.ideal_pitch = r.i8()?;
    }
    // Punch and velocity interleave per axis.
    for i in 0..3 {
        if bits & (su::PUNCH1 << i) != 0 {
            cd.punch[i] = r.i8()? as f32;
        }
        if bits & (su::VELOCITY1 << i) != 0 {
            cd.velocity[i] = r.i8()? as f32 * 16.0;
        }
    }
    // A non-DP7 server always sets SU_ITEMS.
    cd.items = r.i32()? as u32;
    cd.on_ground = bits & su::ONGROUND != 0;
    cd.in_water = bits & su::INWATER != 0;

    if bits & su::WEAPONFRAME != 0 {
        cd.weaponframe = r.u8()? as u16;
    }
    if bits & su::ARMOR != 0 {
        cd.armor = r.u8()? as u16;
    }
    if bits & su::WEAPON != 0 {
        cd.weapon_model = r.u8()? as u16;
    }
    // These five are always present.
    cd.health = r.i16()?;
    cd.ammo = r.u8()? as u16;
    cd.shells = r.u8()? as u16;
    cd.nails = r.u8()? as u16;
    cd.rockets = r.u8()? as u16;
    cd.cells = r.u8()? as u16;
    cd.active_weapon = r.u8()?;

    // FitzQuake high bytes.
    if bits & su::WEAPON2 != 0 {
        cd.weapon_model |= (r.u8()? as u16) << 8;
    }
    if bits & su::ARMOR2 != 0 {
        cd.armor |= (r.u8()? as u16) << 8;
    }
    if bits & su::AMMO2 != 0 {
        cd.ammo |= (r.u8()? as u16) << 8;
    }
    if bits & su::SHELLS2 != 0 {
        cd.shells |= (r.u8()? as u16) << 8;
    }
    if bits & su::NAILS2 != 0 {
        cd.nails |= (r.u8()? as u16) << 8;
    }
    if bits & su::ROCKETS2 != 0 {
        cd.rockets |= (r.u8()? as u16) << 8;
    }
    if bits & su::CELLS2 != 0 {
        cd.cells |= (r.u8()? as u16) << 8;
    }
    if bits & su::WEAPONFRAME2 != 0 {
        cd.weaponframe |= (r.u8()? as u16) << 8;
    }
    if bits & su::WEAPONALPHA != 0 {
        r.u8()?; // viewmodel alpha, unused
    }
    Ok(cd)
}

/// A baseline body. `version` 1 is the classic byte-model/byte-frame form; version 2 is FitzQuake's
/// flagged form with optional large model/frame/alpha/scale.
fn read_baseline(proto: &NqProtoState, r: &mut Reader, version: u8) -> Result<Baseline, ParseError> {
    let bits = if version == 2 { r.u8()? } else { 0 };
    let modelindex = if bits & b::LARGEMODEL != 0 { r.u16()? } else { r.u8()? as u16 };
    let frame = if bits & b::LARGEFRAME != 0 { r.u16()? } else { r.u8()? as u16 };
    let colormap = r.u8()?;
    let skinnum = r.u8()?;
    let mut origin = Vec3::ZERO;
    let mut angles = Vec3::ZERO;
    for i in 0..3 {
        origin[i] = proto.coord(r)?;
        angles[i] = proto.angle(r)?;
    }
    if bits & b::ALPHA != 0 {
        r.u8()?; // alpha, unused
    }
    if bits & b::SCALE != 0 {
        r.u8()?; // scale, unused
    }
    Ok(Baseline { modelindex, frame, colormap, skinnum, origin, angles })
}

/// `svc_sound` — a sound started somewhere. Emits the shared [`SvcEvent::Sound`].
fn read_sound(proto: &NqProtoState, r: &mut Reader) -> Result<SvcEvent, ParseError> {
    let mask = r.u8()?;
    let volume = if mask & snd::VOLUME != 0 { r.u8()? } else { DEFAULT_SOUND_VOLUME };
    let attenuation =
        if mask & snd::ATTENUATION != 0 { r.u8()? as f32 / 64.0 } else { DEFAULT_SOUND_ATTENUATION };
    let (entity, channel) = if mask & snd::LARGEENTITY != 0 {
        (r.u16()?, r.u8()?)
    } else {
        // The entity is the top 13 bits — no mask. (QuakeWorld packs extra flag bits here and masks
        // to 10, but NetQuake does not; masking would alias an entity ≥1024 onto a low player slot.)
        let w = r.u16()?;
        (w >> 3, (w & 7) as u8)
    };
    let sound = if mask & snd::LARGESOUND != 0 { r.u16()? } else { r.u8()? as u16 };
    Ok(SvcEvent::Sound {
        entity,
        channel,
        sound,
        volume,
        attenuation,
        origin: proto.coord3(r)?,
    })
}

/// `svc_temp_entity`. The payload shape depends on the type, and NetQuake's table differs from
/// QuakeWorld's — notably `TE_GUNSHOT` has no count byte, and bytes 12/13 are `EXPLOSION2`/`BEAM`.
fn read_temp_entity(proto: &NqProtoState, r: &mut Reader) -> Result<SvcEvent, ParseError> {
    use TempEntityKind::*;
    let raw = r.u8()?;
    let point = |kind, r: &mut Reader| -> Result<SvcEvent, ParseError> {
        Ok(SvcEvent::TempEntity(TempEntity::Point { kind, origin: proto.coord3(r)? }))
    };
    let te = match raw {
        0 => return point(Spike, r),
        1 => return point(SuperSpike, r),
        2 => return point(Gunshot, r), // no count byte, unlike QuakeWorld
        3 => return point(Explosion, r),
        4 => return point(TarExplosion, r),
        5 => read_beam(proto, r, Lightning1)?,
        6 => read_beam(proto, r, Lightning2)?,
        7 => return point(WizSpike, r),
        8 => return point(KnightSpike, r),
        9 => read_beam(proto, r, Lightning3)?,
        10 => return point(LavaSplash, r),
        11 => return point(Teleport, r),
        12 => {
            // Colour-mapped explosion: origin then two palette bytes.
            let origin = proto.coord3(r)?;
            r.u8()?; // colour start
            r.u8()?; // colour length
            TempEntity::Point { kind: Explosion2, origin }
        }
        13 => read_beam(proto, r, GrappleBeam)?,
        other => {
            return Err(ParseError::UnknownSvc { svc: other, offset: r.pos() - 1 });
        }
    };
    Ok(SvcEvent::TempEntity(te))
}

/// A beam temp entity: a short entity number, then start and end coords.
fn read_beam(proto: &NqProtoState, r: &mut Reader, kind: TempEntityKind) -> Result<TempEntity, ParseError> {
    let entity = r.u16()?;
    let start = proto.coord3(r)?;
    let end = proto.coord3(r)?;
    Ok(TempEntity::Beam { kind, entity, start, end })
}

#[cfg(test)]
mod tests {
    use super::super::protocol::{prfl, NqProtoState, FITZQUAKE, NETQUAKE, RMQ};
    use super::*;
    use crate::sizebuf::Writer;

    /// A hand-built protocol-15 `svc_serverinfo`: the parser must pick up the protocol, maxclients,
    /// gametype and both 1-indexed precache lists (with `models[1]` = the map).
    #[test]
    fn parses_serverinfo_with_inline_precaches() {
        let mut w = Writer::new();
        w.u8(op::SERVERINFO);
        w.i32(NETQUAKE as i32);
        w.u8(8); // maxclients
        w.u8(1); // deathmatch
        w.string("The Bad Place");
        w.string("maps/dm4.bsp");
        w.string("progs/player.mdl");
        w.string(""); // end of models
        w.string("weapons/rocket.wav");
        w.string(""); // end of sounds

        let mut proto = NqProtoState::new();
        let evs = parse(&mut proto, &w.into_vec()).unwrap();
        let SvcEvent::NqServerData(sd) = &evs[0] else { panic!("{evs:?}") };
        assert_eq!(sd.protocol, NETQUAKE);
        assert_eq!(sd.maxclients, 8);
        assert_eq!(sd.gametype, 1);
        assert_eq!(sd.models[1], "maps/dm4.bsp"); // 1-indexed
        assert_eq!(sd.models.len(), 3); // empty slot + 2 models
        assert_eq!(sd.sounds[1], "weapons/rocket.wav");
        assert_eq!(proto.version, NETQUAKE);
    }

    /// An RMQ serverinfo carries a `protocolflags` long that sets the coord/angle widths for the
    /// rest of the stream — QuakeSpasm's default is `FLOATCOORD | SHORTANGLE`.
    #[test]
    fn serverinfo_adopts_rmq_protocolflags() {
        let mut w = Writer::new();
        w.u8(op::SERVERINFO);
        w.i32(RMQ as i32);
        w.u32(prfl::FLOATCOORD | prfl::SHORTANGLE);
        w.u8(4);
        w.u8(1);
        w.string("");
        w.string(""); // no models
        w.string(""); // no sounds

        let mut proto = NqProtoState::new();
        parse(&mut proto, &w.into_vec()).unwrap();
        assert_eq!(proto.version, RMQ);
        assert_eq!(proto.flags, prfl::FLOATCOORD | prfl::SHORTANGLE);
    }

    /// A fast update carrying only origin.x: the delta reports that one axis and leaves the rest
    /// `None`, so the store falls back to the baseline for them (the inverse of QuakeWorld).
    #[test]
    fn fast_update_reports_only_present_fields() {
        let mut w = Writer::new();
        // Opcode: U_SIGNAL | U_ORIGIN1, entity 42, origin.x = 64 (=8.0 at vanilla width).
        w.u8(u::SIGNAL | u::ORIGIN1 as u8);
        w.u8(42);
        w.i16(64);

        let mut proto = NqProtoState::new();
        let evs = parse(&mut proto, &w.into_vec()).unwrap();
        let SvcEvent::EntityUpdate(d) = &evs[0] else { panic!("{evs:?}") };
        assert_eq!(d.number, 42);
        assert_eq!(d.origin[0], Some(8.0));
        assert_eq!(d.origin[1], None); // store resolves from baseline
        assert_eq!(d.model, None);
        assert!(!d.remove);
    }

    /// A fast update with MOREBITS reads a second bit-byte, and model/frame/effects land in the
    /// delta. This is the common "a rocket moved and animated" update.
    #[test]
    fn fast_update_with_morebits_and_model() {
        let mut w = Writer::new();
        w.u8(u::SIGNAL | u::MOREBITS as u8 | u::FRAME as u8);
        w.u8((u::MODEL >> 8) as u8); // second bit-byte: U_MODEL
        w.u8(7); // entity
        // Wire order follows CL_ParseUpdate: model precedes frame.
        w.u8(9); // model
        w.u8(5); // frame

        let mut proto = NqProtoState::new();
        let evs = parse(&mut proto, &w.into_vec()).unwrap();
        let SvcEvent::EntityUpdate(d) = &evs[0] else { panic!("{evs:?}") };
        assert_eq!(d.number, 7);
        assert_eq!(d.frame, Some(5));
        assert_eq!(d.model, Some(9));
    }

    /// `svc_clientdata`: the always-present tail (health/ammo/ammo counts/weapon) plus a couple of
    /// optional fields. This is where the bot reads its own health and items every frame.
    #[test]
    fn parses_clientdata_tail_and_flags() {
        let mut w = Writer::new();
        w.u8(op::CLIENTDATA);
        let bits = su::ONGROUND | su::ARMOR | su::WEAPON;
        w.u16(bits as u16);
        // Non-DP7 always adds SU_ITEMS, but SU_ITEMS wasn't set in `bits`, so no items long is on
        // the wire — the parser reads items unconditionally, so include it here.
        w.i32(0); // items
        w.u8(50); // armor (SU_ARMOR)
        w.u8(8); // weapon model (SU_WEAPON)
        w.i16(87); // health
        w.u8(25); // ammo
        w.u8(1); // shells
        w.u8(2); // nails
        w.u8(3); // rockets
        w.u8(4); // cells
        w.u8(32); // active weapon (IT_ROCKET_LAUNCHER bit)

        let mut proto = NqProtoState::new();
        let evs = parse(&mut proto, &w.into_vec()).unwrap();
        let SvcEvent::ClientData(cd) = &evs[0] else { panic!("{evs:?}") };
        assert_eq!(cd.health, 87);
        assert_eq!(cd.armor, 50);
        assert_eq!(cd.weapon_model, 8);
        assert!(cd.on_ground);
        assert!(!cd.in_water);
        assert_eq!(cd.rockets, 3);
        assert_eq!(cd.active_weapon, 32);
        assert_eq!(cd.viewheight, DEFAULT_VIEWHEIGHT);
    }

    /// The temp-entity table diverges from QuakeWorld at three points: `TE_GUNSHOT` has no count
    /// byte, byte 12 is a colour explosion (two extra bytes), byte 13 is a beam. Copying QuakeWorld's
    /// reader would desync mid-packet, so this pins the NetQuake shapes.
    #[test]
    fn temp_entity_table_matches_netquake() {
        // Gunshot: type + coord3, no count. Followed by a nop to prove we stopped in the right place.
        let mut w = Writer::new();
        w.u8(op::TEMP_ENTITY);
        w.u8(2); // TE_GUNSHOT
        w.i16(8);
        w.i16(16);
        w.i16(24);
        w.u8(op::NOP);
        let mut proto = NqProtoState::new();
        let evs = parse(&mut proto, &w.into_vec()).unwrap();
        assert!(matches!(
            evs[0],
            SvcEvent::TempEntity(TempEntity::Point { kind: TempEntityKind::Gunshot, .. })
        ));
        assert_eq!(evs[1], SvcEvent::Nop, "gunshot consumed exactly 6 bytes");

        // Explosion2 (12): coord3 + two palette bytes.
        let mut w = Writer::new();
        w.u8(op::TEMP_ENTITY);
        w.u8(12);
        w.i16(0);
        w.i16(0);
        w.i16(0);
        w.u8(107); // colour start
        w.u8(8); // colour length
        w.u8(op::NOP);
        let evs = parse(&mut NqProtoState::new(), &w.into_vec()).unwrap();
        assert!(matches!(
            evs[0],
            SvcEvent::TempEntity(TempEntity::Point { kind: TempEntityKind::Explosion2, .. })
        ));
        assert_eq!(evs[1], SvcEvent::Nop);

        // Beam (13): short entity + two coord3.
        let mut w = Writer::new();
        w.u8(op::TEMP_ENTITY);
        w.u8(13);
        w.u16(3); // entity
        w.i16(0);
        w.i16(0);
        w.i16(0);
        w.i16(80);
        w.i16(0);
        w.i16(0);
        w.u8(op::NOP);
        let evs = parse(&mut NqProtoState::new(), &w.into_vec()).unwrap();
        assert!(matches!(
            evs[0],
            SvcEvent::TempEntity(TempEntity::Beam { kind: TempEntityKind::GrappleBeam, entity: 3, .. })
        ));
        assert_eq!(evs[1], SvcEvent::Nop);
    }

    /// `svc_sound` unpacks the packed entity/channel word and honours the volume/attenuation flags,
    /// emitting the shared Sound event the mirror already consumes.
    #[test]
    fn parses_sound_packet() {
        let mut w = Writer::new();
        w.u8(op::SOUND);
        w.u8(snd::VOLUME); // mask
        w.u8(200); // volume
        w.u16((12 << 3) | 1); // entity 12, channel 1
        w.u8(40); // sound index
        w.i16(8);
        w.i16(16);
        w.i16(24);
        let evs = parse(&mut NqProtoState::new(), &w.into_vec()).unwrap();
        let SvcEvent::Sound { entity, channel, sound, volume, .. } = evs[0] else { panic!("{evs:?}") };
        assert_eq!((entity, channel, sound, volume), (12, 1, 40, 200));

        // The entity is 13 bits, not 10: a high-numbered entity must not alias onto a low player
        // slot (QuakeWorld masks to 10 here; NetQuake doesn't).
        let mut w = Writer::new();
        w.u8(op::SOUND);
        w.u8(0); // no flags
        w.u16(2000 << 3); // entity 2000, channel 0
        w.u8(5);
        w.i16(0);
        w.i16(0);
        w.i16(0);
        let evs = parse(&mut NqProtoState::new(), &w.into_vec()).unwrap();
        let SvcEvent::Sound { entity, .. } = evs[0] else { panic!("{evs:?}") };
        assert_eq!(entity, 2000, "entity must not be masked to 10 bits");
    }

    /// A whole signon-shaped datagram parses end to end: serverinfo (which sets widths), a baseline,
    /// clientdata, and signonnum, all in one packet.
    #[test]
    fn parses_a_signon_datagram() {
        let mut w = Writer::new();
        w.u8(op::SERVERINFO);
        w.i32(FITZQUAKE as i32);
        w.u8(8);
        w.u8(1);
        w.string("dm4");
        w.string("maps/dm4.bsp");
        w.string("");
        w.string("");
        // A baseline for entity 1 (a player).
        w.u8(op::SPAWNBASELINE);
        w.u16(1);
        w.u8(1); // model
        w.u8(0); // frame
        w.u8(0); // colormap
        w.u8(0); // skin
        for _ in 0..3 {
            w.i16(0); // origin coord (vanilla width for 666)
            w.u8(0); // angle (8-bit for 666)
        }
        w.u8(op::SIGNONNUM);
        w.u8(2);

        let mut proto = NqProtoState::new();
        let evs = parse(&mut proto, &w.into_vec()).unwrap();
        assert!(matches!(evs[0], SvcEvent::NqServerData(_)));
        assert!(matches!(evs[1], SvcEvent::SpawnBaseline { entity: 1, .. }));
        assert_eq!(evs[2], SvcEvent::SignonNum(2));
    }

    /// An unknown opcode is fatal, and it names the byte and offset for the desync log.
    #[test]
    fn unknown_opcode_is_fatal() {
        // 50 is svcdp_downloaddata — an extension we declined, so it must not be silently skipped.
        let err = parse(&mut NqProtoState::new(), &[op::NOP, 50]).unwrap_err();
        assert_eq!(err, ParseError::UnknownSvc { svc: 50, offset: 1 });
    }
}
