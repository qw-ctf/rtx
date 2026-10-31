//! Base FTE replacement-delta wire regressions; no world state belongs in the parser.
use rtx_proto::protocol::{fte2, ProtoState};
use rtx_proto::svc;

fn rd() -> ProtoState {
    let mut proto = ProtoState::new();
    proto.apply(0, fte2::REPLACEMENTDELTAS, 0);
    proto
}

#[test]
fn accepts_reset_world_and_empty_retained_frame() {
    let mut packet = vec![86];
    packet.extend(1.0f32.to_le_bytes());
    packet.extend([0, 128, 0, 0]);
    packet.push(86);
    packet.extend(2.0f32.to_le_bytes());
    packet.extend([0, 0]);
    assert!(svc::parse(&mut rd(), &packet).is_ok(), "base RD frames must decode");
}

#[test]
fn replacement_signon_uses_reference_then_payload_and_static_payload_only() {
    // Opcode 66: entity 16383, UF_EXTEND1|UF_RESET|UF_MODEL, model 7.
    // Opcode 21: UF_MODEL (extended flags), model 8. Nop proves exact consumption.
    let events = svc::parse(&mut rd(), &[66, 255, 63, 128, 5, 7, 21, 128, 4, 8, 1]).unwrap();
    assert!(
        matches!(&events[0], svc::SvcEvent::SpawnReplacementBaseline { entity: 16383, delta }
        if delta.reset && delta.modelindex == Some(7))
    );
    assert!(matches!(&events[1], svc::SvcEvent::SpawnReplacementStatic(delta) if delta.modelindex == Some(8)));
    assert_eq!(events[2], svc::SvcEvent::Nop);
}

#[test]
fn consumes_every_base_field_from_retained_fte_writer_fixtures() {
    for (width, fixture) in [
        (
            2,
            include_bytes!("fixtures/replacement_deltas/fte-base-2.bin").as_slice(),
        ),
        (
            4,
            include_bytes!("fixtures/replacement_deltas/fte-base-4.bin").as_slice(),
        ),
    ] {
        let mut proto = rd();
        if width == 4 {
            proto.apply(rtx_proto::protocol::fte::FLOATCOORDS, fte2::REPLACEMENTDELTAS, 0);
        }
        let mut packet = vec![86];
        packet.extend(3.0f32.to_le_bytes());
        packet.extend([42, 0]);
        packet.extend(fixture);
        packet.extend([0, 0, 1]);
        let result = svc::parse(&mut proto, &packet);
        assert!(result.is_ok(), "width {width}: {result:?}");
    }
}

fn bits(value: u32) -> Vec<u8> {
    let mut value = value;
    if value & 0xff00_0000 != 0 {
        value |= 1 << 23;
    }
    if value & 0xffff_0000 != 0 {
        value |= 1 << 15;
    }
    if value & 0xffff_ff00 != 0 {
        value |= 1 << 7;
    }
    let bytes = value.to_le_bytes();
    bytes[..if value & (1 << 23) != 0 {
        4
    } else if value & (1 << 15) != 0 {
        3
    } else if value & (1 << 7) != 0 {
        2
    } else {
        1
    }]
        .to_vec()
}

fn frame_packet(records: &[u8]) -> Vec<u8> {
    let mut packet = vec![86];
    packet.extend(1.0f32.to_le_bytes());
    packet.extend(records);
    packet.extend([0, 0]);
    packet
}

fn first_delta(proto: &mut ProtoState, payload: &[u8]) -> rtx_proto::rd::Delta {
    let mut records = vec![1, 0];
    records.extend(payload);
    let events = svc::parse(proto, &frame_packet(&records)).unwrap();
    match events.into_iter().next().unwrap() {
        svc::SvcEvent::ReplacementEntities(frame) => match frame.updates.into_iter().next().unwrap() {
            rtx_proto::rd::Update::Change { entity: 1, delta } => delta,
            other => panic!("unexpected {other:?}"),
        },
        other => panic!("unexpected {other:?}"),
    }
}

#[test]
fn fte_fixture_retains_full_precision_and_all_ancillary_state() {
    use glam::Vec3;
    let d = first_delta(&mut rd(), include_bytes!("fixtures/replacement_deltas/fte-base-2.bin"));
    assert!(d.reset && d.predinfo);
    let s = d.apply(None, None);
    assert_eq!((s.frame, s.modelindex, s.modelindex2, s.skinnum), (300, 511, 400, -1));
    assert_eq!((s.colormap, s.effects, s.solidsize), (31, 0x12345678, 31));
    assert_eq!(s.origin, Vec3::new(1.25, -2.5, 3.75));
    assert!((s.angles - Vec3::new(30., 45., 60.)).abs().max_element() < 0.006);
    assert!((s.viewangles.x - 270.).abs() < 0.02);
    assert_eq!(s.movement, [100, -200, 300]);
    assert_eq!(s.velocity, [800, -1600, 2400]);
    assert_eq!((s.pmovetype, s.msec, s.weaponframe), (3, 14, 200));
    assert_eq!((s.dpflags, s.trans, s.scale), (9, 127, 24));
    assert_eq!((s.drawflags, s.abslight), (6, 177));
    assert_eq!((s.tagentity, s.tagindex), (2047, -1));
    assert_eq!(s.light, [65535, 32768, 1024, 256]);
    assert_eq!((s.lightstyle, s.lightpflags), (7, 13));
    assert_eq!((s.traileffect, s.emiteffect), (16383, 15));
    assert_eq!(s.colourmod, [1, 2, 3]);
    assert_eq!((s.glowsize, s.glowcolour, s.glowmod), (12, 8, [10, 20, 30]));
    assert_eq!(s.fatness, -5);
    assert_eq!(s.gravitydir, [12, 34]);
}

#[test]
fn applies_reset_baseline_and_transient_prediction_rules() {
    use rtx_proto::rd::{Delta, State};
    let previous = State {
        modelindex: 9,
        frame: 12,
        weaponframe: 300,
        pmovetype: 3,
        velocity: [8, 16, 24],
        movement: [100, 200, 300],
        msec: 10,
        ..State::default()
    };
    let baseline = State {
        modelindex: 5,
        weaponframe: 200,
        pmovetype: 2,
        ..State::default()
    };
    let changed = Delta {
        frame: Some(400),
        ..Delta::default()
    }
    .apply(Some(&previous), Some(&baseline));
    assert_eq!(
        (
            changed.modelindex,
            changed.frame,
            changed.weaponframe,
            changed.pmovetype
        ),
        (9, 400, 300, 3)
    );
    assert_eq!((changed.movement, changed.velocity, changed.msec), ([0; 3], [0; 3], 0));
    let reset = Delta {
        reset: true,
        frame: Some(400),
        ..Delta::default()
    }
    .apply(Some(&previous), Some(&baseline));
    assert_eq!(
        (reset.modelindex, reset.frame, reset.weaponframe, reset.pmovetype),
        (5, 400, 200, 2)
    );
    assert_eq!(Delta::default().apply(None, Some(&baseline)), State::default());
    assert_eq!(
        Delta {
            reset: true,
            ..Delta::default()
        }
        .apply(Some(&previous), None),
        State::default()
    );
    assert_eq!(
        (
            State::default().trans,
            State::default().scale,
            State::default().drawflags,
            State::default().tagindex
        ),
        (255, 16, 96, 0)
    );
    assert_eq!(
        (State::default().colourmod, State::default().glowmod),
        ([32; 3], [32; 3])
    );
}

#[test]
fn clears_absent_drawlight_and_emitter_but_retains_other_persistent_fields() {
    use rtx_proto::rd::State;
    let mut payload = bits((1 << 19) | (1 << 22));
    payload.extend([5, 7, 0]); // drawflags 5 has no abslight; trail 7 has no emitter.
    let old = State {
        abslight: 100,
        emiteffect: 90,
        trans: 70,
        ..State::default()
    };
    let next = first_delta(&mut rd(), &payload).apply(Some(&old), None);
    assert_eq!(
        (next.abslight, next.traileffect, next.emiteffect, next.trans),
        (0, 7, 0, 70)
    );
}

#[test]
fn reference_and_update_index_encodings_have_independent_boundaries() {
    use rtx_proto::rd::{read_entity_ref, read_update_index};
    use rtx_proto::sizebuf::Reader;
    for (bytes, expected) in [
        (vec![255, 63], 16383),
        (vec![0, 64], 16384),
        (vec![255, 127], 32767),
        (vec![128, 128, 0], 32768),
        (vec![255, 255, 255], 0x7fffff),
    ] {
        assert_eq!(read_entity_ref(&mut Reader::new(&bytes)).unwrap(), expected);
        for end in 0..bytes.len() {
            assert!(read_entity_ref(&mut Reader::new(&bytes[..end])).is_err());
        }
    }
    for (bytes, expected, remove) in [
        (vec![255, 63], 16383, false),
        (vec![0, 64, 1], 16384, false),
        (vec![255, 127, 255], 0x3fffff, false),
        (vec![0, 128], 0, true),
        (vec![255, 255, 255], 0x3fffff, true),
    ] {
        assert_eq!(read_update_index(&mut Reader::new(&bytes)).unwrap(), (expected, remove));
        for end in 0..bytes.len() {
            assert!(read_update_index(&mut Reader::new(&bytes[..end])).is_err());
        }
    }
}

#[test]
fn world_frame_rejects_bad_order_repeated_reset_and_out_of_capacity_ids() {
    // Each change uses a legal empty UF payload. Removal carries no UF byte.
    for records in [
        vec![1, 0, 0, 1, 0, 0],
        vec![2, 0, 0, 1, 0, 0],
        vec![0, 128, 0, 128],
        vec![1, 0, 0, 0, 128],
        vec![0, 64, 1, 0],
        vec![255, 255, 255],
    ] {
        assert!(svc::parse(&mut rd(), &frame_packet(&records)).is_err(), "{records:?}");
    }
    let events = svc::parse(&mut rd(), &frame_packet(&[0, 128, 255, 63, 0, 255, 191])).unwrap_err();
    assert!(matches!(events, svc::ParseError::InvalidPayload { .. })); // duplicate high entity
    let events = svc::parse(&mut rd(), &frame_packet(&[0, 128, 1, 128, 255, 63, 0])).unwrap();
    let svc::SvcEvent::ReplacementEntities(frame) = &events[0] else {
        panic!()
    };
    assert_eq!(frame.updates[0], rtx_proto::rd::Update::ResetWorld);
    assert_eq!(frame.updates[1], rtx_proto::rd::Update::Remove(1));
    assert!(matches!(
        frame.updates[2],
        rtx_proto::rd::Update::Change { entity: 16383, .. }
    ));
}

#[test]
fn all_truncated_fte_frames_fail_without_publishing_partial_events() {
    for fixture in [
        include_bytes!("fixtures/replacement_deltas/fte-base-2.bin").as_slice(),
        include_bytes!("fixtures/replacement_deltas/fte-base-4.bin").as_slice(),
    ] {
        let mut proto = rd();
        if fixture.len() == 87 {
            proto.apply(rtx_proto::protocol::fte::FLOATCOORDS, fte2::REPLACEMENTDELTAS, 0);
        }
        let mut records = vec![0, 128, 42, 0];
        records.extend(fixture);
        let packet = frame_packet(&records);
        for end in 1..packet.len() {
            assert!(
                svc::parse(&mut proto, &packet[..end]).is_err(),
                "accepted truncation {end}/{}",
                packet.len()
            );
        }
        assert!(svc::parse(&mut proto, &packet).is_ok());
        let mut malformed_tail = packet.clone();
        malformed_tail.push(255);
        assert!(svc::parse(&mut proto, &malformed_tail).is_err());
    }
}

#[test]
fn rejects_reserved_fields_bones_and_unimplemented_layout_capabilities() {
    for bit in [18, 30, 31] {
        let mut records = vec![1, 0];
        records.extend(bits(1 << bit));
        assert!(matches!(
            svc::parse(&mut rd(), &frame_packet(&records)),
            Err(svc::ParseError::InvalidPayload { .. })
        ));
    }
    for capability in [0x20, 0x40, 0x400] {
        let mut proto = rd();
        proto.fte2 |= capability;
        assert!(matches!(
            svc::parse(&mut proto, &frame_packet(&[])),
            Err(svc::ParseError::InvalidPayload { .. })
        ));
    }
    assert!(svc::parse(&mut ProtoState::new(), &frame_packet(&[])).is_err());
    // 67 is customtempent, not a replacement baseline/static variant.
    assert!(matches!(
        svc::parse(&mut rd(), &[67, 0]),
        Err(svc::ParseError::UnknownSvc { svc: 67, .. })
    ));
}

#[test]
fn nonfinite_time_and_float_origins_are_rejected() {
    let mut proto = rd();
    proto.apply(0, fte2::REPLACEMENTDELTAS, rtx_proto::protocol::mvd1::FLOATCOORDS);
    for value in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
        let mut packet = frame_packet(&[]);
        packet[1..5].copy_from_slice(&value.to_le_bytes());
        assert!(svc::parse(&mut proto, &packet).is_err());
        let mut records = vec![1, 0, 4];
        records.extend(value.to_le_bytes());
        assert!(svc::parse(&mut proto, &frame_packet(&records)).is_err());
    }
}

#[test]
fn entity_only_floatcoords_does_not_widen_nonpredicted_angles() {
    use rtx_proto::protocol::mvd1;
    let mut proto = rd();
    proto.apply(0, fte2::REPLACEMENTDELTAS, mvd1::FLOATCOORDS);
    let mut payload = bits((1 << 1) | (1 << 2) | (1 << 3) | (1 << 4));
    for value in [1.25f32, -2.5, 3.75] {
        payload.extend(value.to_le_bytes());
    }
    payload.extend([64, 192, 128]); // X, Z, Y as byte angles.
    let state = first_delta(&mut proto, &payload).apply(None, None);
    assert_eq!(state.origin, glam::Vec3::new(1.25, -2.5, 3.75));
    assert_eq!(state.angles, glam::Vec3::new(90., -180., -90.));
    assert_eq!(state.viewangles, glam::Vec3::new(270., 180., 270.));
}

#[test]
fn prediction_angle_precision_and_variable_weaponframe_are_base_rd() {
    for weaponframe in [0u16, 127, 128, 32767] {
        let mut payload = bits((1 << 3) | (1 << 4) | (1 << 6));
        payload.extend(5461u16.to_le_bytes());
        payload.extend(10922u16.to_le_bytes());
        payload.extend(16384u16.to_le_bytes());
        payload.push(128); // no movement/velocity/msec; old weaponframe present.
        if weaponframe > 127 {
            payload.extend([(weaponframe as u8 & 127) | 128, (weaponframe >> 7) as u8]);
        } else {
            payload.push(weaponframe as u8);
        }
        let state = first_delta(&mut rd(), &payload).apply(None, None);
        assert_eq!(state.weaponframe, weaponframe);
        assert!((state.viewangles.x - 270.).abs() < 0.02);
        assert_eq!(state.angles.y, 90.);
    }
}

#[test]
fn width_marker_and_effect_width_combinations_are_independent() {
    for (mask, value, bytes) in [
        (1 << 5, 0xfeu32, vec![254]),
        (1 << 29, 0xfedc, vec![220, 254]),
        ((1 << 5) | (1 << 29), 0xfedcba98, vec![152, 186, 220, 254]),
    ] {
        let mut payload = bits(mask);
        payload.extend(bytes);
        assert_eq!(first_delta(&mut rd(), &payload).effects, Some(value));
    }
    let mut payload = bits((1 << 0) | (1 << 9) | (1 << 10) | (1 << 11) | (1 << 27));
    for value in [65535u16, 511, (-200i16) as u16, 400] {
        payload.extend(value.to_le_bytes());
    }
    let state = first_delta(&mut rd(), &payload).apply(None, None);
    assert_eq!(
        (state.frame, state.modelindex, state.skinnum, state.modelindex2),
        (65535, 511, -200, 400)
    );
}

#[test]
fn ordinary_references_widen_sound_and_muzzle_but_preserve_packed_stopsound() {
    use svc::{op, SvcEvent};
    // LARGEENTITY with three-byte ordinary reference 32768, channel 7, sound 2.
    let mut packet = vec![op::FTE_SOUNDEXTENDED, 8, 128, 128, 0, 7, 2];
    packet.extend([0; 6]);
    packet.extend([op::MUZZLEFLASH, 128, 128, 0]);
    packet.extend([op::STOPSOUND, 255, 255]);
    packet.extend([op::SPAWNBASELINE, 255, 63]); // highest supported live RD slot
    packet.extend([0; 13]); // classic model/frame/colormap/skin, three coord+angle pairs
    packet.push(op::NOP);
    let events = svc::parse(&mut rd(), &packet).unwrap();
    assert!(matches!(
        events[0],
        SvcEvent::Sound {
            entity: 32768,
            channel: 7,
            sound: 2,
            ..
        }
    ));
    assert_eq!(events[1], SvcEvent::MuzzleFlash { entity: 32768 });
    assert_eq!(
        events[2],
        SvcEvent::StopSound {
            entity: 8191,
            channel: 7
        }
    );
    assert!(matches!(events[3], SvcEvent::SpawnBaseline { entity: 16383, .. }));
    assert_eq!(events[4], SvcEvent::Nop);
}

#[test]
fn classic_rd_baseline_validation_reports_its_real_opcode_offset() {
    use svc::{op, ParseError};
    for reference in [vec![0, 64], vec![128, 128, 0]] {
        // 16384 and 32768
        let mut packet = vec![op::NOP, op::SPAWNBASELINE];
        packet.extend(reference);
        packet.extend([0; 13]);
        assert!(matches!(
            svc::parse(&mut rd(), &packet),
            Err(ParseError::InvalidPayload { offset: 1, .. })
        ));
    }
    for value in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
        let mut proto = rd();
        proto.apply(rtx_proto::protocol::fte::FLOATCOORDS, fte2::REPLACEMENTDELTAS, 0);
        let mut packet = vec![op::NOP, op::SPAWNBASELINE, 1, 0];
        packet.extend([0; 4]); // model, frame, colormap, skin
        for coordinate in [value, 0.0, 0.0] {
            packet.extend(coordinate.to_le_bytes());
            packet.extend([0; 2]); // angle16
        }
        assert!(matches!(
            svc::parse(&mut proto, &packet),
            Err(ParseError::InvalidPayload { offset: 1, .. })
        ));
        // This migration only tightens the RD profile, not legacy classic baselines.
        proto.apply(rtx_proto::protocol::fte::FLOATCOORDS, 0, 0);
        assert!(svc::parse(&mut proto, &packet).is_ok());
    }
}

fn serverdata_for_classic_baseline_test(replacement: bool) -> Vec<u8> {
    let mut packet = vec![svc::op::SERVERDATA];
    if replacement {
        packet.extend(rtx_proto::protocol::magic::FTE2.to_le_bytes());
        packet.extend(fte2::REPLACEMENTDELTAS.to_le_bytes());
    }
    packet.extend(rtx_proto::protocol::VERSION.to_le_bytes());
    packet.extend(1i32.to_le_bytes());
    packet.extend(b"qw\0");
    packet.push(0); // playernum
    packet.extend(b"map\0");
    packet.extend([0; 40]); // movevars
    packet
}

#[test]
fn classic_baseline_validation_follows_serverdata_protocol_changes_in_the_same_packet() {
    for replacement in [false, true] {
        let mut packet = serverdata_for_classic_baseline_test(replacement);
        let offset = packet.len();
        packet.extend([svc::op::SPAWNBASELINE, 0, 64]);
        packet.extend([0; 13]);
        packet.push(svc::op::NOP);
        let mut proto = if replacement { ProtoState::new() } else { rd() };
        let result = svc::parse(&mut proto, &packet);
        if replacement {
            assert!(matches!(result, Err(svc::ParseError::InvalidPayload { offset: found, .. }) if found == offset));
        } else {
            assert!(matches!(result.unwrap().last(), Some(svc::SvcEvent::Nop)));
        }
    }
}

#[test]
fn replacement_signon_accepts_all_coordinate_profiles_and_rejects_truncation() {
    use rtx_proto::protocol::{fte, mvd1};
    use svc::{op, SvcEvent};
    for (fte, mvd, fixture) in [
        (
            0,
            0,
            include_bytes!("fixtures/replacement_deltas/fte-base-2.bin").as_slice(),
        ),
        (
            fte::FLOATCOORDS,
            0,
            include_bytes!("fixtures/replacement_deltas/fte-base-4.bin").as_slice(),
        ),
        (
            0,
            mvd1::FLOATCOORDS,
            include_bytes!("fixtures/replacement_deltas/fte-base-4.bin").as_slice(),
        ),
    ] {
        let mut proto = rd();
        proto.apply(fte, fte2::REPLACEMENTDELTAS, mvd);
        for opcode in [op::FTE_SPAWNBASELINE2, op::FTE_SPAWNSTATIC2] {
            let mut packet = vec![opcode];
            if opcode == op::FTE_SPAWNBASELINE2 {
                packet.extend([255, 63]);
            }
            packet.extend(fixture);
            let events = svc::parse(&mut proto, &packet).unwrap();
            let state = match &events[0] {
                SvcEvent::SpawnReplacementBaseline { entity: 16383, delta } => delta.apply(None, None),
                SvcEvent::SpawnReplacementStatic(delta) => delta.apply(None, None),
                other => panic!("{other:?}"),
            };
            assert_eq!(state.modelindex, 511);
            assert_eq!(state.origin, glam::Vec3::new(1.25, -2.5, 3.75));
            for end in 1..packet.len() {
                assert!(
                    svc::parse(&mut proto, &packet[..end]).is_err(),
                    "opcode {opcode} truncation {end}"
                );
            }
        }
        let mut oversized = vec![op::FTE_SPAWNBASELINE2, 0, 64];
        oversized.extend(fixture);
        assert!(matches!(
            svc::parse(&mut proto, &oversized),
            Err(svc::ParseError::InvalidPayload { .. })
        ));
    }
}

#[test]
fn beam_references_follow_rd_while_netquake_largeentity_stays_unsigned_short() {
    use svc::{op, SvcEvent, TempEntity};
    let mut beam = vec![op::TEMP_ENTITY, 5, 128, 128, 0];
    beam.extend([0; 12]);
    beam.push(op::NOP);
    let events = svc::parse(&mut rd(), &beam).unwrap();
    assert!(matches!(
        events[0],
        SvcEvent::TempEntity(TempEntity::Beam { entity: 32768, .. })
    ));
    assert_eq!(events[1], SvcEvent::Nop);
    let mut nq = rtx_proto::nq::protocol::NqProtoState::new();
    let mut sound = vec![6, 8, 0, 128, 7, 2]; // entity 32768 encoded as fixed unsigned short.
    sound.extend([0; 6]);
    sound.push(1);
    let events = rtx_proto::nq::svc::parse(&mut nq, &sound).unwrap();
    assert!(matches!(
        events[0],
        SvcEvent::Sound {
            entity: 32768,
            channel: 7,
            sound: 2,
            ..
        }
    ));
    assert_eq!(events[1], SvcEvent::Nop);
}

#[test]
fn player_projection_carries_prediction_signed_viewangles_and_full_width_state() {
    use rtx_proto::rd::{player_info, State};
    use svc::{pf, PmType};
    let state = State {
        origin: glam::Vec3::new(1., 2., 3.),
        viewangles: glam::Vec3::new(330., 180., 270.),
        modelindex: 511,
        frame: 65535,
        skinnum: -200,
        effects: 0xfedcba98,
        weaponframe: 32767,
        pmovetype: 3 | 128 | 64,
        solidsize: 31,
        movement: [100, -200, 300],
        velocity: [800, -1600, 2400],
        msec: 14,
        trans: 127,
        ..State::default()
    };
    let player = player_info(32, &state).unwrap();
    assert_eq!(player.player, 31);
    assert_eq!(player.origin, state.origin);
    assert_eq!(player.velocity, glam::Vec3::new(100., -200., 300.));
    assert_eq!(player.msec, Some(14));
    assert!(player.on_ground() && player.solid() && player.jump_held);
    assert!(!player.dead() && !player.gib());
    assert_eq!(player.pm_type, Some(PmType::Normal));
    let command = player.command.unwrap();
    assert_eq!(command.angles, glam::Vec3::new(-30., -180., -90.));
    assert_eq!(
        (command.forward, command.side, command.up, command.msec),
        (100, -200, 300, 14)
    );
    assert_eq!(
        (
            player.modelindex,
            player.frame,
            player.skinnum,
            player.effects,
            player.weaponframe
        ),
        (Some(511), 65535, Some(-200), Some(0xfedcba98), Some(32767))
    );
    assert_eq!(player.alpha, Some(127));
    assert_eq!(
        player.flags & (pf::VELOCITY1 | pf::VELOCITY1 << 1 | pf::VELOCITY1 << 2),
        pf::VELOCITY1 | pf::VELOCITY1 << 1 | pf::VELOCITY1 << 2
    );
    assert_ne!(player.flags & pf::COMMAND, 0);
    assert!(player_info(0, &state).is_none());
    assert!(player_info(33, &state).is_none());
    assert!(player_info(u32::MAX, &state).is_none());
}

#[test]
fn player_projection_decodes_death_and_move_flags_without_guessing_gibs() {
    use rtx_proto::rd::{player_info, State};
    use svc::PmType;
    for (code, expected) in [
        (0, PmType::None),
        (3, PmType::Normal),
        (5, PmType::Fly),
        (6, PmType::Dead),
        (8, PmType::Spectator),
        (10, PmType::Dead),
        (63, PmType::Normal),
    ] {
        let state = State {
            pmovetype: code,
            ..State::default()
        };
        let player = player_info(1, &state).unwrap();
        assert_eq!(player.pm_type, Some(expected));
        assert_eq!(player.dead(), expected == PmType::Dead);
        assert!(!player.gib() && !player.solid() && !player.on_ground() && !player.jump_held);
        assert_eq!(player.velocity, glam::Vec3::ZERO);
        assert_eq!(player.alpha, Some(255));
    }
    // Model pitch -1/3 of view pitch must project back to -30, rather than clamp 330 to +80.
    let delta = rtx_proto::rd::Delta {
        predinfo: true,
        angles: [Some(10.), Some(270.), None],
        ..rtx_proto::rd::Delta::default()
    };
    let player = player_info(1, &delta.apply(None, None)).unwrap();
    assert!(
        (player.command.unwrap().angles - glam::Vec3::new(-30., -90., 0.))
            .abs()
            .max_element()
            < 0.006
    );
}
