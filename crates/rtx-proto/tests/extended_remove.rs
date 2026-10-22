// SPDX-License-Identifier: AGPL-3.0-or-later

//! Always-on regressions for MVDSV's extended U_REMOVE encoding. The .hex files
//! are complete svc payloads, extracted unchanged from RTX's failed-packet
//! dumps on AD's start map (MVDSV 2264ea8, QCX Native/Wasm, 2026-10-21).
//! They exclude netchan headers; the captured negotiation is fixed below.

use rtx_proto::protocol::ProtoState;
use rtx_proto::svc::{self, EntityDelta, SvcEvent};

fn captured_protocol() -> ProtoState {
    let mut proto = ProtoState::new();
    proto.apply(558_395_400, 0, 0);
    proto.z_ext = 511;
    proto
}

fn check_capture(hex: &str, length: usize, delta_from: u8, removed: &[u16], first_z: f32) {
    let bytes: Vec<_> = hex
        .split_whitespace()
        .map(|byte| u8::from_str_radix(byte, 16).expect("captured hex byte"))
        .collect();
    assert_eq!(bytes.len(), length);
    let events = svc::parse(&mut captured_protocol(), &bytes).expect("complete captured packet must parse");
    let [SvcEvent::PlayerInfo(player), SvcEvent::PacketEntities(packet)] = events.as_slice() else {
        panic!("unexpected captured messages: {events:?}");
    };
    assert_eq!(player.player, 0);
    assert_eq!(packet.delta_from, Some(delta_from));
    assert_eq!(packet.updates[0].number, 92);
    assert_eq!(packet.updates[0].origin, [None, None, Some(first_z)]);
    assert!(packet.updates.windows(2).all(|pair| pair[0].number < pair[1].number));
    assert!(packet.updates.iter().all(|update| update.number < 2048));
    let removals: Vec<_> = packet.updates.iter().filter(|update| update.remove).collect();
    assert_eq!(removals.iter().map(|update| update.number).collect::<Vec<_>>(), removed);
    for update in removals {
        assert_eq!(
            *update,
            EntityDelta {
                number: update.number,
                remove: true,
                ..Default::default()
            },
            "a removal must not invent any update fields"
        );
    }
}

#[test]
fn replays_ad_start_wasm_sequence_53() {
    check_capture(include_str!("data/ad-start-wasm-seq53.hex"), 321, 52, &[658], 37.625);
}

#[test]
fn replays_ad_start_native_sequence_50() {
    check_capture(
        include_str!("data/ad-start-native-seq50.hex"),
        248,
        49,
        &[666, 667, 669, 674, 677, 678],
        27.125,
    );
}

#[test]
fn replays_ad_start_native_sequence_61() {
    check_capture(
        include_str!("data/ad-start-native-seq61.hex"),
        186,
        60,
        &[626, 630, 684],
        50.25,
    );
}

#[test]
fn removes_ids_across_extension_boundaries_without_consuming_the_terminator() {
    // Hand-encoded headers from MVDSV's writer, deliberately retaining the
    // high ID bits in the leading word as the real writer does. They overlap
    // ORIGIN1/ORIGIN2 but do not announce coordinate payloads for U_REMOVE.
    let cases: &[(u16, &[u8])] = &[
        (114, &[0x72, 0x40]),
        (511, &[0xff, 0x41]),
        (512, &[0x00, 0xc2, 0x80, 0x20]),
        (626, &[0x72, 0xc2, 0x80, 0x20]),
        (1023, &[0xff, 0xc3, 0x80, 0x20]),
        (1024, &[0x00, 0xc4, 0x80, 0x40]),
        (1042, &[0x12, 0xc4, 0x80, 0x40]),
        (1535, &[0xff, 0xc5, 0x80, 0x40]),
        (1536, &[0x00, 0xc6, 0x80, 0x60]),
        (1554, &[0x12, 0xc6, 0x80, 0x60]),
        (2047, &[0xff, 0xc7, 0x80, 0x60]),
    ];
    for &(number, header) in cases {
        let mut bytes = vec![0x30, 42]; // svc_deltapacketentities, previous frame
        bytes.extend_from_slice(header);
        bytes.extend_from_slice(&[0, 0, 1]); // list terminator, svc_nop
        let events =
            svc::parse(&mut captured_protocol(), &bytes).unwrap_or_else(|error| panic!("ID {number}: {error}"));
        let [SvcEvent::PacketEntities(packet), SvcEvent::Nop] = events.as_slice() else {
            panic!("ID {number}: lost message alignment: {events:?}");
        };
        assert_eq!(packet.delta_from, Some(42));
        assert_eq!(
            packet.updates,
            [EntityDelta {
                number,
                remove: true,
                ..Default::default()
            }]
        );
    }
}

#[test]
fn removal_does_not_consume_the_next_entity_update() {
    let bytes = [
        0x30, 42, // svc_deltapacketentities
        0x72, 0xc2, 0x80, 0x20, // remove entity 626
        0x73, 0xa2, 0x84, 0x20, 7, 9, 0x00, 0x03, // entity 627: model 7, frame 9, x=96
        0, 0, 1, // terminator, svc_nop
    ];
    let events = svc::parse(&mut captured_protocol(), &bytes).expect("remove followed by ordinary delta");
    let [SvcEvent::PacketEntities(packet), SvcEvent::Nop] = events.as_slice() else {
        panic!("lost message alignment: {events:?}");
    };
    assert_eq!(
        packet.updates,
        [
            EntityDelta {
                number: 626,
                remove: true,
                ..Default::default()
            },
            EntityDelta {
                number: 627,
                model: Some(7),
                frame: Some(9),
                origin: [Some(96.0), None, None],
                ..Default::default()
            }
        ]
    );
}

#[test]
fn truncated_remove_extension_headers_are_still_rejected() {
    for bytes in [
        &[0x30, 42, 0x72, 0xc2][..],         // missing MOREBITS byte
        &[0x30, 42, 0x72, 0xc2, 0x80],       // missing FTE extension
        &[0x30, 42, 0x72, 0xc2, 0x80, 0xa0], // missing YETMORE byte
    ] {
        assert!(svc::parse(&mut captured_protocol(), bytes).is_err());
    }
}
