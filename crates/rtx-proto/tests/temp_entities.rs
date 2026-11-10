//! QuakeWorld FTE temp entities have wire layouts independent of NetQuake's type table.
use rtx_proto::protocol::{fte, fte2, ProtoState};
use rtx_proto::sizebuf::Writer;
use rtx_proto::svc::{self, op, SvcEvent, TempEntity, TempEntityKind};

#[test]
fn qw_railtrail_consumes_only_six_coords() {
    let packet = [
        op::TEMP_ENTITY,
        17,
        8,
        0,
        240,
        255,
        24,
        0,
        224,
        255,
        40,
        0,
        208,
        255,
        op::NOP,
    ];
    let events = svc::parse(&mut ProtoState::new(), &packet).unwrap();
    assert_eq!(
        events,
        vec![
            SvcEvent::TempEntity(TempEntity::Trail {
                kind: TempEntityKind::RailTrail,
                start: glam::Vec3::new(1.0, -2.0, 3.0),
                end: glam::Vec3::new(-4.0, 5.0, -6.0),
            }),
            SvcEvent::Nop,
        ]
    );
}

fn proto(float_coords: bool, rd: bool) -> ProtoState {
    let mut proto = ProtoState::new();
    proto.apply(
        if float_coords { fte::FLOATCOORDS } else { 0 },
        if rd { fte2::REPLACEMENTDELTAS } else { 0 },
        0,
    );
    proto
}

fn packet(kind: u8, reference: &[u8], float_coords: bool) -> Vec<u8> {
    let mut w = Writer::new();
    w.u8(op::TEMP_ENTITY);
    w.u8(kind);
    w.bytes(reference);
    for coord in [1.25f32, -2.5, 3.75, -4.0, 5.125, -6.25] {
        if float_coords {
            w.bytes(&coord.to_le_bytes());
        } else {
            w.i16((coord * 8.0) as i16);
        }
    }
    w.u8(op::NOP);
    w.into_vec()
}

fn assert_complete_and_truncated(proto: &ProtoState, packet: &[u8], expected: TempEntity) {
    assert_eq!(
        svc::parse(&mut proto.clone(), packet).unwrap(),
        vec![SvcEvent::TempEntity(expected), SvcEvent::Nop],
    );
    // The NOP is outside the TE payload. Every shorter prefix of that payload must underflow.
    for end in 1..packet.len() - 1 {
        assert!(
            matches!(
                svc::parse(&mut proto.clone(), &packet[..end]),
                Err(svc::ParseError::Underflow(_))
            ),
            "truncation {end} of {packet:?}",
        );
    }
}

#[test]
fn railtrail_honors_coord_width_without_entity_or_color_even_with_rd() {
    for float_coords in [false, true] {
        for rd in [false, true] {
            assert_complete_and_truncated(
                &proto(float_coords, rd),
                &packet(17, &[], float_coords),
                TempEntity::Trail {
                    kind: TempEntityKind::RailTrail,
                    start: glam::Vec3::new(1.25, -2.5, 3.75),
                    end: glam::Vec3::new(-4.0, 5.125, -6.25),
                },
            );
        }
    }
}

#[test]
fn qw_beams_use_ordinary_rd_references_not_update_headers() {
    // Ordinary references use 15 bits inline (bit 14 is not an extension flag), or 23 bits
    // with eight low bits in the next byte. Update/removal headers use a different layout.
    let references: &[(u32, &[u8])] = &[
        (0, &[0, 0]),
        (16383, &[255, 63]),
        (16384, &[0, 64]),
        (32767, &[255, 127]),
        (32768, &[128, 128, 0]),
        (65535, &[255, 128, 255]),
        (8388607, &[255, 255, 255]),
    ];
    for float_coords in [false, true] {
        for (raw, kind) in [
            (5, TempEntityKind::Lightning1),
            (6, TempEntityKind::Lightning2),
            (9, TempEntityKind::Lightning3),
            (18, TempEntityKind::GrappleBeam),
        ] {
            for &(entity, reference) in references {
                assert_complete_and_truncated(
                    &proto(float_coords, true),
                    &packet(raw, reference, float_coords),
                    TempEntity::Beam {
                        kind,
                        entity,
                        start: glam::Vec3::new(1.25, -2.5, 3.75),
                        end: glam::Vec3::new(-4.0, 5.125, -6.25),
                    },
                );
            }
        }
    }
}

#[test]
fn legacy_beam_entity_bitpatterns_remain_unsigned_shorts() {
    for float_coords in [false, true] {
        for (raw, kind) in [
            (5, TempEntityKind::Lightning1),
            (6, TempEntityKind::Lightning2),
            (9, TempEntityKind::Lightning3),
            (18, TempEntityKind::GrappleBeam),
        ] {
            // Negative lightning-entity rail aliases (-1, -7) remain the prior raw Beam event;
            // they must not consume an RD extension byte or change the payload interpretation.
            for (reference, entity) in [([0, 128], 32768), ([255, 255], 65535), ([249, 255], 65529)] {
                assert_complete_and_truncated(
                    &proto(float_coords, false),
                    &packet(raw, &reference, float_coords),
                    TempEntity::Beam {
                        kind,
                        entity,
                        start: glam::Vec3::new(1.25, -2.5, 3.75),
                        end: glam::Vec3::new(-4.0, 5.125, -6.25),
                    },
                );
            }
        }
    }
}

#[test]
fn qw_17_and_18_do_not_enable_conflicting_netquake_extensions() {
    for raw in [17, 18] {
        let mut nq = rtx_proto::nq::protocol::NqProtoState::new();
        assert!(matches!(
            rtx_proto::nq::svc::parse(&mut nq, &packet(raw, &[], false)),
            Err(svc::ParseError::UnknownSvc { svc, offset: 1 }) if svc == raw,
        ));
    }
}

#[test]
fn qw_beam_uses_the_existing_grapple_beam_kind() {
    let packet = [
        op::TEMP_ENTITY,
        18,
        7,
        0,
        8,
        0,
        240,
        255,
        24,
        0,
        224,
        255,
        40,
        0,
        208,
        255,
        op::NOP,
    ];
    let events = svc::parse(&mut ProtoState::new(), &packet).unwrap();
    assert_eq!(
        events,
        vec![
            SvcEvent::TempEntity(TempEntity::Beam {
                kind: TempEntityKind::GrappleBeam,
                entity: 7,
                start: glam::Vec3::new(1.0, -2.0, 3.0),
                end: glam::Vec3::new(-4.0, 5.0, -6.0),
            }),
            SvcEvent::Nop,
        ]
    );
}

// Hand-encoded position (1.25, -2.5, 3.75), followed by svc_nop (1).
// These intentionally do not use Writer or the semantic enum's discriminants.
fn qw_point_fixtures(float_coords: bool) -> [&'static [u8]; 5] {
    if float_coords {
        [
            &[23, 14, 0, 0, 160, 63, 0, 0, 32, 192, 0, 0, 112, 64, 1],
            &[23, 15, 0, 0, 160, 63, 0, 0, 32, 192, 0, 0, 112, 64, 1],
            &[23, 19, 0, 0, 160, 63, 0, 0, 32, 192, 0, 0, 112, 64, 0, 255, 1],
            &[23, 20, 0, 0, 160, 63, 0, 0, 32, 192, 0, 0, 112, 64, 1],
            &[23, 21, 0, 0, 160, 63, 0, 0, 32, 192, 0, 0, 112, 64, 1],
        ]
    } else {
        [
            &[23, 14, 10, 0, 236, 255, 30, 0, 1],
            &[23, 15, 10, 0, 236, 255, 30, 0, 1],
            &[23, 19, 10, 0, 236, 255, 30, 0, 0, 255, 1],
            &[23, 20, 10, 0, 236, 255, 30, 0, 1],
            &[23, 21, 10, 0, 236, 255, 30, 0, 1],
        ]
    }
}

#[test]
fn qw_remaining_fte_points_do_not_infer_capabilities() {
    for float_coords in [false, true] {
        for rd in [false, true] {
            for packet in qw_point_fixtures(float_coords) {
                let mut state = proto(float_coords, rd);
                let before = state;
                let events = svc::parse(&mut state, packet).unwrap();
                assert!(matches!(events.as_slice(), [SvcEvent::TempEntity(_), SvcEvent::Nop]));
                assert_eq!(state, before, "receiving a TE must not infer capabilities");
            }
        }
    }
}

#[test]
fn qw_remaining_fte_points_preserve_identity_alignment_and_truncation_errors() {
    let origin = glam::Vec3::new(1.25, -2.5, 3.75);
    let expected = [
        TempEntity::Point {
            kind: TempEntityKind::Bullet,
            origin,
        },
        TempEntity::Point {
            kind: TempEntityKind::SuperBullet,
            origin,
        },
        TempEntity::PaletteExplosion {
            origin,
            palette_start: 0,
            palette_length: 255,
        },
        TempEntity::Point {
            kind: TempEntityKind::NqExplosion,
            origin,
        },
        TempEntity::Point {
            kind: TempEntityKind::Gunshot,
            origin,
        },
    ];
    for float_coords in [false, true] {
        for rd in [false, true] {
            for (packet, expected) in qw_point_fixtures(float_coords).into_iter().zip(expected) {
                assert_complete_and_truncated(&proto(float_coords, rd), packet, expected);
            }
        }
    }
}

#[test]
fn qw_palette_explosion_keeps_zero_and_maximum_palette_bytes() {
    for float_coords in [false, true] {
        for (palette_start, palette_length) in [(0, 0), (0, 255), (255, 0), (255, 255)] {
            let mut packet = qw_point_fixtures(float_coords)[2].to_vec();
            let len = packet.len();
            packet[len - 3] = palette_start;
            packet[len - 2] = palette_length;
            assert_complete_and_truncated(
                &proto(float_coords, false),
                &packet,
                TempEntity::PaletteExplosion {
                    origin: glam::Vec3::new(1.25, -2.5, 3.75),
                    palette_start,
                    palette_length,
                },
            );
        }
    }
}

#[test]
fn default_qw_and_nq_gunshots_and_explosions_keep_their_existing_variants() {
    let origin = glam::Vec3::new(1.25, -2.5, 3.75);
    // QW type 2 still has a count; type 3 remains the regular QW explosion.
    let qw = [23, 2, 7, 10, 0, 236, 255, 30, 0, 23, 3, 10, 0, 236, 255, 30, 0, 1];
    assert_eq!(
        svc::parse(&mut ProtoState::new(), &qw).unwrap(),
        vec![
            SvcEvent::TempEntity(TempEntity::Puff {
                kind: TempEntityKind::Gunshot,
                count: 7,
                origin
            }),
            SvcEvent::TempEntity(TempEntity::Point {
                kind: TempEntityKind::Explosion,
                origin
            }),
            SvcEvent::Nop,
        ],
    );
    // NQ type 2 remains count-free, and its native type 3 retains the existing event kind.
    let nq = [23, 2, 10, 0, 236, 255, 30, 0, 23, 3, 10, 0, 236, 255, 30, 0, 1];
    assert_eq!(
        rtx_proto::nq::svc::parse(&mut rtx_proto::nq::protocol::NqProtoState::new(), &nq).unwrap(),
        vec![
            SvcEvent::TempEntity(TempEntity::Point {
                kind: TempEntityKind::Gunshot,
                origin
            }),
            SvcEvent::TempEntity(TempEntity::Point {
                kind: TempEntityKind::Explosion,
                origin
            }),
            SvcEvent::Nop,
        ],
    );
}

#[test]
fn nq_overlapping_ids_and_palette_output_remain_unchanged_after_width_reset() {
    use rtx_proto::nq::{
        protocol::{prfl, NqProtoState, RMQ},
        svc as nq_svc,
    };
    let mut state = NqProtoState::new();
    state.set(RMQ, prfl::FLOATCOORD);
    // svc_serverinfo: protocol 15, maxclients 1, gametype 0, empty level/model/sound lists.
    let reset = [11, 15, 0, 0, 0, 1, 0, 0, 0, 0];
    let palette = [23, 12, 10, 0, 236, 255, 30, 0, 255, 0, 1];
    let mut packet = reset.to_vec();
    packet.extend(palette);
    let events = nq_svc::parse(&mut state, &packet).unwrap();
    assert_eq!(state, NqProtoState::new());
    assert_eq!(
        &events[1..],
        &[
            SvcEvent::TempEntity(TempEntity::Point {
                kind: TempEntityKind::Explosion2,
                origin: glam::Vec3::new(1.25, -2.5, 3.75),
            }),
            SvcEvent::Nop,
        ],
    );
    for raw in [14, 15, 17, 18, 19, 20, 21] {
        // These QW IDs must not become NQ extensions through a shared dispatch table.
        assert!(matches!(
            nq_svc::parse(&mut state, &[23, raw, 10, 0, 236, 255, 30, 0, 1]),
            Err(svc::ParseError::UnknownSvc { svc, offset: 1 }) if svc == raw,
        ));
    }
    for end in 1..palette.len() - 1 {
        assert!(matches!(
            nq_svc::parse(&mut state, &palette[..end]),
            Err(svc::ParseError::Underflow(_)),
        ));
    }
    // NQ's type 13 still owns its entity short, unlike QW's point-only lightning blood.
    assert_eq!(
        nq_svc::parse(&mut state, &[23, 13, 7, 0, 8, 0, 16, 0, 24, 0, 32, 0, 40, 0, 48, 0, 1]).unwrap(),
        vec![
            SvcEvent::TempEntity(TempEntity::Beam {
                kind: TempEntityKind::GrappleBeam,
                entity: 7,
                start: glam::Vec3::new(1.0, 2.0, 3.0),
                end: glam::Vec3::new(4.0, 5.0, 6.0),
            }),
            SvcEvent::Nop,
        ],
    );
}

#[test]
fn qw_remaining_fte_points_follow_serverdata_width_and_capability_resets() {
    // Vanilla QW serverdata with empty game/map names and 40 bytes of movevars.
    let mut reset = vec![11, 28, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0];
    reset.extend([0; 40]);
    let mut state = proto(true, true);
    for packet in qw_point_fixtures(false) {
        reset.extend(packet);
    }
    let events = svc::parse(&mut state, &reset).unwrap();
    assert!(matches!(events[0], SvcEvent::ServerData(_)));
    assert_eq!(events.len(), 11);
    assert_eq!(state, ProtoState::new());
}
