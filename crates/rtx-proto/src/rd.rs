// SPDX-License-Identifier: AGPL-3.0-or-later

//! Base FTE QuakeWorld replacement deltas, following FTE f0395c07's cl_ents.c.
//!
//! This module decodes changes only. The caller retains entities and baselines, validates
//! frame availability, and applies a complete message before acknowledging a valid world.

use crate::protocol::{fte2, ProtoState};
use crate::sizebuf::{Reader, Underflow};
use crate::svc::ParseError;
use glam::Vec3;

// UF_16BIT_LERPTIME is the WIDE marker in this base profile; with PEXT2_LERPTIME
// it is redefined and frame/model fields use ULEB128 instead.
mod uf {
    pub const FRAME: u32 = 1 << 0;
    pub const ORIGINXY: u32 = 1 << 1;
    pub const ORIGINZ: u32 = 1 << 2;
    pub const ANGLESXZ: u32 = 1 << 3;
    pub const ANGLESY: u32 = 1 << 4;
    pub const EFFECTS: u32 = 1 << 5;
    pub const PREDINFO: u32 = 1 << 6;
    pub const EXTEND1: u32 = 1 << 7;
    pub const RESET: u32 = 1 << 8;
    pub const WIDE: u32 = 1 << 9;
    pub const MODEL: u32 = 1 << 10;
    pub const SKIN: u32 = 1 << 11;
    pub const COLORMAP: u32 = 1 << 12;
    pub const SOLID: u32 = 1 << 13;
    pub const FLAGS: u32 = 1 << 14;
    pub const EXTEND2: u32 = 1 << 15;
    pub const ALPHA: u32 = 1 << 16;
    pub const SCALE: u32 = 1 << 17;
    pub const BONEDATA: u32 = 1 << 18;
    pub const DRAWFLAGS: u32 = 1 << 19;
    pub const TAGINFO: u32 = 1 << 20;
    pub const LIGHT: u32 = 1 << 21;
    pub const TRAILEFFECT: u32 = 1 << 22;
    pub const EXTEND3: u32 = 1 << 23;
    pub const COLORMOD: u32 = 1 << 24;
    pub const GLOW: u32 = 1 << 25;
    pub const FATNESS: u32 = 1 << 26;
    pub const MODELINDEX2: u32 = 1 << 27;
    pub const GRAVITYDIR: u32 = 1 << 28;
    pub const EFFECTS2: u32 = 1 << 29;
    pub const UNUSED1: u32 = 1 << 30;
    pub const EXTEND4: u32 = 1 << 31;
}

/// Supported network entity slots; slot zero belongs to the world/reset marker.
pub const MAX_ENTITIES: u32 = 16384;

/// The complete base-RD state. Prediction velocity is retained in wire units (1/8 unit/s).
#[derive(Clone, Debug, PartialEq)]
pub struct State {
    pub origin: Vec3,
    pub angles: Vec3,
    pub viewangles: Vec3,
    pub modelindex: u16,
    pub modelindex2: u16,
    pub frame: u16,
    pub skinnum: i16,
    pub colormap: u8,
    pub effects: u32,
    pub solidsize: u16,
    pub movement: [i16; 3],
    pub velocity: [i16; 3],
    pub pmovetype: u8,
    pub msec: u8,
    pub weaponframe: u16,
    pub dpflags: u8,
    pub trans: u8,
    pub scale: u8,
    pub drawflags: u8,
    pub abslight: u8,
    pub tagentity: u32,
    /// Zero by default; the wire byte 255 denotes -1.
    pub tagindex: i16,
    pub light: [u16; 4],
    pub lightstyle: u8,
    pub lightpflags: u8,
    pub traileffect: u16,
    pub emiteffect: u16,
    pub colourmod: [u8; 3],
    pub glowsize: u8,
    pub glowcolour: u8,
    pub glowmod: [u8; 3],
    pub fatness: i8,
    pub gravitydir: [u8; 2],
}

impl Default for State {
    fn default() -> Self {
        Self {
            origin: Vec3::ZERO,
            angles: Vec3::ZERO,
            viewangles: Vec3::ZERO,
            modelindex: 0,
            modelindex2: 0,
            frame: 0,
            skinnum: 0,
            colormap: 0,
            effects: 0,
            solidsize: 0,
            movement: [0; 3],
            velocity: [0; 3],
            pmovetype: 0,
            msec: 0,
            weaponframe: 0,
            dpflags: 0,
            trans: 255,
            scale: 16,
            drawflags: 96,
            abslight: 0,
            tagentity: 0,
            tagindex: 0,
            light: [0; 4],
            lightstyle: 0,
            lightpflags: 0,
            traileffect: 0,
            emiteffect: 0,
            colourmod: [32; 3],
            glowsize: 0,
            glowcolour: 0,
            glowmod: [32; 3],
            fatness: 0,
            gravitydir: [0; 2],
        }
    }
}

/// Changed persistent fields plus prediction fields that reset on every entity update.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Delta {
    pub reset: bool,
    /// Base UF_PREDINFO, independent of the separate PEXT2_PREDINFO capability.
    pub predinfo: bool,
    pub origin: [Option<f32>; 3],
    pub angles: [Option<f32>; 3],
    pub modelindex: Option<u16>,
    pub modelindex2: Option<u16>,
    pub frame: Option<u16>,
    pub skinnum: Option<i16>,
    pub colormap: Option<u8>,
    pub effects: Option<u32>,
    pub solidsize: Option<u16>,
    pub movement: [i16; 3],
    pub velocity: [i16; 3],
    pub pmovetype: Option<u8>,
    pub msec: u8,
    pub weaponframe: Option<u16>,
    pub dpflags: Option<u8>,
    pub trans: Option<u8>,
    pub scale: Option<u8>,
    pub drawflags: Option<u8>,
    pub abslight: Option<u8>,
    pub tagentity: Option<u32>,
    pub tagindex: Option<i16>,
    pub light: Option<[u16; 4]>,
    pub lightstyle: Option<u8>,
    pub lightpflags: Option<u8>,
    pub traileffect: Option<u16>,
    pub emiteffect: Option<u16>,
    pub colourmod: Option<[u8; 3]>,
    pub glowsize: Option<u8>,
    pub glowcolour: Option<u8>,
    pub glowmod: Option<[u8; 3]>,
    pub fatness: Option<i8>,
    pub gravitydir: Option<[u8; 2]>,
}

impl Delta {
    /// Reset from the baseline, or patch the previous state. Missing bases use FTE defaults.
    /// Movement, velocity and msec are transient even when their UF_PREDINFO block is absent.
    pub fn apply(&self, previous: Option<&State>, baseline: Option<&State>) -> State {
        let mut next = if self.reset { baseline } else { previous }
            .cloned()
            .unwrap_or_default();
        macro_rules! patch { ($($field:ident),* $(,)?) => { $(if let Some(value) = self.$field { next.$field = value; })* }; }
        patch!(
            modelindex,
            modelindex2,
            frame,
            skinnum,
            colormap,
            effects,
            solidsize,
            pmovetype,
            weaponframe,
            dpflags,
            trans,
            scale,
            drawflags,
            abslight,
            tagentity,
            tagindex,
            light,
            lightstyle,
            lightpflags,
            traileffect,
            emiteffect,
            colourmod,
            glowsize,
            glowcolour,
            glowmod,
            fatness,
            gravitydir
        );
        next.movement = self.movement;
        next.velocity = self.velocity;
        next.msec = self.msec;
        for axis in 0..3 {
            if let Some(value) = self.origin[axis] {
                next.origin[axis] = value;
            }
            if let Some(value) = self.angles[axis] {
                next.angles[axis] = value;
                let value = value
                    * if axis == 0 {
                        if self.predinfo {
                            -3.0
                        } else {
                            -1.0
                        }
                    } else {
                        1.0
                    };
                // FTE ANGLE2SHORT truncates, then wraps to a 16-bit turn.
                next.viewangles[axis] = ((value * (65536.0 / 360.0)) as i32 & 65535) as f32 * (360.0 / 65536.0);
            }
        }
        next
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum Update {
    ResetWorld,
    Remove(u32),
    Change { entity: u32, delta: Delta },
}

#[derive(Clone, Debug, PartialEq)]
pub struct Frame {
    pub time: f32,
    pub updates: Vec<Update>,
}

fn invalid(offset: usize, reason: &'static str) -> ParseError {
    ParseError::InvalidPayload { offset, reason }
}

/// Ordinary FTE entity reference: a 15-bit short or 15 high bits followed by eight low bits.
/// Deliberately independent of the update/removal header and of the application's slot limit.
pub fn read_entity_ref(r: &mut Reader<'_>) -> Result<u32, Underflow> {
    let word = r.u16()? as u32;
    Ok(if word & 0x8000 != 0 {
        ((word & 0x7fff) << 8) | r.u8()? as u32
    } else {
        word
    })
}

/// Update/removal header: 14 low bits, optional eight high bits, and a separate removal flag.
/// `(0, false)` terminates a stream; `(0, true)` resets the world.
pub fn read_update_index(r: &mut Reader<'_>) -> Result<(u32, bool), Underflow> {
    let word = r.u16()? as u32;
    let entity = (word & 0x3fff) | if word & 0x4000 != 0 { (r.u8()? as u32) << 14 } else { 0 };
    Ok((entity, word & 0x8000 != 0))
}

fn base_profile(proto: &ProtoState, offset: usize) -> Result<(), ParseError> {
    if proto.fte2 & fte2::REPLACEMENTDELTAS == 0 {
        return Err(invalid(offset, "replacement deltas were not negotiated"));
    }
    // These capabilities redefine UF fields, not merely the availability of an opcode.
    if proto.fte2 & (0x20 | 0x40 | 0x400) != 0 {
        return Err(invalid(offset, "unsupported replacement-delta capability"));
    }
    Ok(())
}

/// Decode a base UF payload, with no entity reference or update header.
pub fn read_delta(proto: &ProtoState, r: &mut Reader<'_>) -> Result<Delta, ParseError> {
    let offset = r.pos();
    base_profile(proto, offset)?;
    use uf::*;
    let mut bits = r.u8()? as u32;
    if bits & EXTEND1 != 0 {
        bits |= (r.u8()? as u32) << 8;
    }
    if bits & EXTEND2 != 0 {
        bits |= (r.u8()? as u32) << 16;
    }
    if bits & EXTEND3 != 0 {
        bits |= (r.u8()? as u32) << 24;
    }
    if bits & (BONEDATA | UNUSED1 | EXTEND4) != 0 {
        return Err(invalid(offset, "unsupported replacement-delta field"));
    }
    let has = |mask: u32| bits & mask != 0;
    let wide = has(WIDE);
    let mut delta = Delta {
        reset: has(RESET),
        predinfo: has(PREDINFO),
        ..Delta::default()
    };
    if has(FRAME) {
        delta.frame = Some(if wide { r.u16()? } else { r.u8()? as u16 });
    }
    for axis in 0..3 {
        if has(if axis == 2 { ORIGINZ } else { ORIGINXY }) {
            let value = if proto.float_origins() { r.f32()? } else { r.coord()? };
            if !value.is_finite() {
                return Err(invalid(offset, "nonfinite replacement origin"));
            }
            delta.origin[axis] = Some(value);
        }
    }
    // FTE serializes X,Z together before Y, rather than in axis order.
    for axis in [0, 2, 1] {
        if has(if axis == 1 { ANGLESY } else { ANGLESXZ }) {
            let value = if delta.predinfo { r.angle16()? } else { r.angle()? };
            delta.angles[axis] = Some(value);
        }
    }
    delta.effects = match (has(EFFECTS), has(EFFECTS2)) {
        (true, true) => Some(r.u32()?),
        (true, false) => Some(r.u8()? as u32),
        (false, true) => Some(r.u16()? as u32),
        (false, false) => None,
    };
    if delta.predinfo {
        let pred = r.u8()?;
        for axis in 0..3 {
            if pred & (1 << axis) != 0 {
                delta.movement[axis] = r.i16()?;
            }
        }
        if pred & 8 != 0 {
            delta.pmovetype = Some(r.u8()?);
        }
        if pred & 16 != 0 {
            delta.velocity[0] = r.i16()?;
            delta.velocity[1] = r.i16()?;
        }
        if pred & 32 != 0 {
            delta.velocity[2] = r.i16()?;
        }
        if pred & 64 != 0 {
            delta.msec = r.u8()?;
        }
        if pred & 128 != 0 {
            let first = r.u8()? as u16;
            delta.weaponframe = Some(if first & 128 != 0 {
                (first & 127) | ((r.u8()? as u16) << 7)
            } else {
                first
            });
        }
    }
    if has(MODEL) {
        delta.modelindex = Some(if wide { r.u16()? } else { r.u8()? as u16 });
    }
    if has(SKIN) {
        delta.skinnum = Some(if wide { r.i16()? } else { r.u8()? as i16 });
    }
    if has(COLORMAP) {
        delta.colormap = Some(r.u8()?);
    }
    if has(SOLID) {
        delta.solidsize = Some(r.u16()?);
    }
    if has(FLAGS) {
        delta.dpflags = Some(r.u8()?);
    }
    if has(ALPHA) {
        delta.trans = Some(r.u8()?);
    }
    if has(SCALE) {
        delta.scale = Some(r.u8()?);
    }
    if has(DRAWFLAGS) {
        let flags = r.u8()?;
        delta.drawflags = Some(flags);
        delta.abslight = Some(if flags & 7 >= 6 { r.u8()? } else { 0 });
    }
    if has(TAGINFO) {
        let entity = read_entity_ref(r)?;
        if entity >= MAX_ENTITIES {
            return Err(invalid(offset, "replacement tag entity exceeds capacity"));
        }
        delta.tagentity = Some(entity);
        let index = r.u8()?;
        delta.tagindex = Some(if index == 255 { -1 } else { index as i16 });
    }
    if has(LIGHT) {
        delta.light = Some([r.u16()?, r.u16()?, r.u16()?, r.u16()?]);
        delta.lightstyle = Some(r.u8()?);
        delta.lightpflags = Some(r.u8()?);
    }
    if has(TRAILEFFECT) {
        let value = r.u16()?;
        delta.traileffect = Some(value & 0x3fff);
        delta.emiteffect = Some(if value & 0x8000 != 0 { r.u16()? & 0x3fff } else { 0 });
    }
    if has(COLORMOD) {
        delta.colourmod = Some([r.u8()?, r.u8()?, r.u8()?]);
    }
    if has(GLOW) {
        delta.glowsize = Some(r.u8()?);
        delta.glowcolour = Some(r.u8()?);
        delta.glowmod = Some([r.u8()?, r.u8()?, r.u8()?]);
    }
    if has(FATNESS) {
        delta.fatness = Some(r.i8()?);
    }
    if has(MODELINDEX2) {
        delta.modelindex2 = Some(if wide { r.u16()? } else { r.u8()? as u16 });
    }
    if has(GRAVITYDIR) {
        delta.gravitydir = Some([r.u8()?, r.u8()?]);
    }
    Ok(delta)
}

/// Decode one persistent entity update stream. IDs must be increasing after an optional reset.
pub fn read_frame(proto: &ProtoState, r: &mut Reader<'_>) -> Result<Frame, ParseError> {
    let offset = r.pos();
    base_profile(proto, offset)?;
    let time = r.f32()?;
    if !time.is_finite() {
        return Err(invalid(offset, "nonfinite replacement time"));
    }
    let mut updates = Vec::new();
    let mut last = 0;
    loop {
        let offset = r.pos();
        let (entity, remove) = read_update_index(r)?;
        if entity == 0 && !remove {
            break;
        }
        if entity == 0 && remove && updates.is_empty() {
            updates.push(Update::ResetWorld);
            continue;
        }
        if entity == 0 || entity <= last || entity >= MAX_ENTITIES {
            return Err(invalid(offset, "invalid replacement entity order or capacity"));
        }
        last = entity;
        updates.push(if remove {
            Update::Remove(entity)
        } else {
            Update::Change {
                entity,
                delta: read_delta(proto, r)?,
            }
        });
    }
    Ok(Frame { time, updates })
}

/// Project a retained RD player entity into the same prediction/evidence input as playerinfo.
/// The returned player slot is zero-based. Entity IDs outside the 32 QW players have no projection.
pub fn player_info(entity: u32, state: &State) -> Option<crate::svc::PlayerInfo> {
    use crate::svc::{pf, PlayerInfo, PmType, Usercmd};
    if !(1..=32).contains(&entity) {
        return None;
    }
    let pm_type = match state.pmovetype & 63 {
        0 => PmType::None,
        5 => PmType::Fly,
        6 | 10 => PmType::Dead,
        8 => PmType::Spectator,
        _ => PmType::Normal,
    };
    let mut flags = pf::MSEC
        | pf::COMMAND
        | pf::VELOCITY1
        | (pf::VELOCITY1 << 1)
        | (pf::VELOCITY1 << 2)
        | pf::MODEL
        | pf::SKINNUM
        | pf::EFFECTS
        | pf::WEAPONFRAME
        | pf::TRANS_Z;
    if state.pmovetype & 128 != 0 {
        flags |= pf::ONGROUND;
    }
    if state.solidsize > 0 {
        flags |= pf::SOLID;
    }
    if pm_type == PmType::Dead {
        flags |= pf::DEAD;
    }
    // FTE retains view angles as unsigned shorts. Playerinfo consumers expect the signed
    // representation used by MSG_ReadAngle16, particularly when clamping a player's pitch.
    let angles = state.viewangles.map(|angle| (angle + 180.0).rem_euclid(360.0) - 180.0);
    Some(PlayerInfo {
        player: (entity - 1) as u8,
        flags,
        origin: state.origin,
        frame: state.frame,
        msec: Some(state.msec),
        command: Some(Usercmd {
            msec: state.msec,
            angles,
            forward: state.movement[0],
            side: state.movement[1],
            up: state.movement[2],
            ..Usercmd::default()
        }),
        velocity: Vec3::new(
            state.velocity[0] as f32,
            state.velocity[1] as f32,
            state.velocity[2] as f32,
        ) / 8.0,
        modelindex: Some(state.modelindex),
        skinnum: Some(state.skinnum),
        effects: Some(state.effects),
        weaponframe: Some(state.weaponframe),
        alpha: Some(state.trans),
        pm_type: Some(pm_type),
        jump_held: state.pmovetype & 64 != 0,
    })
}
