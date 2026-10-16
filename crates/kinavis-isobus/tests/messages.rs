//! ISOBUS messages: bytes of known values, the ends of the ranges, all ones,
//! and every message there and back.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::indexing_slicing
)]

use kinavis_isobus::{
    Direction, GroundBasedSpeed, GuidanceMachineInfo, GuidanceSystemCommand, Indicator,
    LimitStatus, MachineSelectedSpeed, MachineSelectedSpeedCommand, Message, SpeedSource,
    WheelBasedSpeed, MAX_CURVATURE,
};
use kinavis_j1939::{CanId, Frame, Pgn};
use kinavis_kernel::{Distance, Speed};
use proptest::prelude::*;

fn frame(pgn: Pgn, bytes: [u8; 8]) -> Frame {
    Frame::new(CanId::from_parts(3, pgn, 0xFF, 0x47), &bytes).unwrap()
}

#[test]
fn the_curvature_offset_and_resolution_are_those_of_the_bus() {
    // Zero curvature is raw 32128 = 0x7D80; one step is 0.25 km⁻¹.
    let zero = GuidanceSystemCommand::from_bytes([0x80, 0x7D, 0xFD, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]);
    assert_eq!(zero.curvature, Some(0.0));
    assert_eq!(zero.intends_to_steer, Indicator::On);
    let step = GuidanceSystemCommand::from_bytes([0x81, 0x7D, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]);
    assert_eq!(step.curvature, Some(0.000_25));
    let lowest = GuidanceSystemCommand::from_bytes([0, 0, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]);
    assert_eq!(lowest.curvature, Some(-8.032));
    let highest =
        GuidanceSystemCommand::from_bytes([0xFF, 0xFA, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]);
    assert_eq!(highest.curvature, Some(MAX_CURVATURE));
    // Beyond the range: clamped on the way out.
    let tight = GuidanceSystemCommand {
        curvature: Some(20.0),
        intends_to_steer: Indicator::On,
    };
    assert_eq!(&tight.to_bytes()[..2], &[0xFF, 0xFA]);
}

#[test]
fn all_ones_is_not_available_everywhere() {
    let ones = [0xFF; 8];
    let wheel = WheelBasedSpeed::from_bytes(ones);
    assert_eq!(
        (wheel.speed, wheel.distance, wheel.power_time_minutes),
        (None, None, None)
    );
    assert_eq!(wheel.direction, Direction::NotAvailable);
    assert_eq!(wheel.key_switch, Indicator::NotAvailable);
    let ground = GroundBasedSpeed::from_bytes(ones);
    assert_eq!(
        (ground.speed, ground.distance, ground.direction),
        (None, None, Direction::NotAvailable)
    );
    let selected = MachineSelectedSpeed::from_bytes(ones);
    assert_eq!(selected.exit_reason, None);
    assert_eq!(selected.source, SpeedSource::NotAvailable);
    assert_eq!(selected.limit, LimitStatus::NotAvailable);
    let info = GuidanceMachineInfo::from_bytes(ones);
    assert_eq!(info.curvature, None);
    assert_eq!(info.steering_ready, Indicator::NotAvailable);
    // Error codes above the valid range are not values either.
    let error = GroundBasedSpeed::from_bytes([0x00, 0xFE, 0x00, 0x00, 0x00, 0xFE, 0xFF, 0xFE]);
    assert_eq!(
        (error.speed, error.distance, error.direction),
        (None, None, Direction::Error)
    );
}

#[test]
fn a_wheel_based_speed_reads_field_by_field() {
    // 4.567 m/s, 123 456.789 m, 45 min, forward, key not off, implement
    // operations started, operator not reversed: 0b00_01_01_01.
    let bytes = [0xD7, 0x11, 0x15, 0xCD, 0x5B, 0x07, 45, 0b0001_0101];
    let wheel = WheelBasedSpeed::from_bytes(bytes);
    assert_eq!(wheel.speed.unwrap().metres_per_second(), 4.567);
    assert!((wheel.distance.unwrap().metres() - 123_456.789).abs() < 1e-9);
    assert_eq!(wheel.power_time_minutes, Some(45));
    assert_eq!(wheel.direction, Direction::Forward);
    assert_eq!(wheel.key_switch, Indicator::On);
    assert_eq!(wheel.implement_operations, Indicator::On);
    assert_eq!(wheel.operator_reversed, Indicator::Off);
    assert_eq!(wheel.to_bytes(), bytes);
}

#[test]
fn every_group_is_decoded_from_its_frame_and_others_are_passed_over() {
    let ones = [0xFF; 8];
    let cases = [
        (WheelBasedSpeed::PGN, 65_096),
        (GroundBasedSpeed::PGN, 65_097),
        (MachineSelectedSpeed::PGN, 61_474),
        (MachineSelectedSpeedCommand::PGN, 64_835),
        (GuidanceMachineInfo::PGN, 44_032),
        (GuidanceSystemCommand::PGN, 44_288),
    ];
    for (pgn, number) in cases {
        assert_eq!(pgn.number(), number);
        assert!(
            Message::decode(&frame(pgn, ones)).unwrap().is_some(),
            "{pgn}"
        );
        let short = Frame::new(CanId::from_parts(3, pgn, 0xFF, 0x47), &[0; 7]).unwrap();
        assert!(Message::decode(&short).is_err());
    }
    assert_eq!(
        Message::decode(&frame(Pgn::new(65_265), ones)).unwrap(),
        None
    );
}

fn speed() -> impl Strategy<Value = Option<Speed>> {
    prop_oneof![
        Just(None),
        (0u16..=0xFAFF)
            .prop_map(|mm| Some(Speed::from_metres_per_second(f64::from(mm) / 1000.0).unwrap())),
    ]
}

fn distance() -> impl Strategy<Value = Option<Distance>> {
    prop_oneof![
        Just(None),
        (0u32..=0xFAFF_FFFF)
            .prop_map(|mm| Some(Distance::from_metres(f64::from(mm) / 1000.0).unwrap())),
    ]
}

fn direction() -> impl Strategy<Value = Direction> {
    prop_oneof![
        Just(Direction::Reverse),
        Just(Direction::Forward),
        Just(Direction::Error),
        Just(Direction::NotAvailable)
    ]
}

fn indicator() -> impl Strategy<Value = Indicator> {
    prop_oneof![
        Just(Indicator::Off),
        Just(Indicator::On),
        Just(Indicator::Error),
        Just(Indicator::NotAvailable)
    ]
}

fn limit() -> impl Strategy<Value = LimitStatus> {
    (0u8..8)
        .prop_map(|bits| GuidanceMachineInfo::from_bytes([0, 0, 0, bits << 5, 0, 0, 0, 0]).limit)
}

fn curvature() -> impl Strategy<Value = Option<f64>> {
    prop_oneof![
        Just(None),
        (0u16..=0xFAFF).prop_map(|raw| Some((f64::from(raw) * 0.25 - 8_032.0) / 1000.0))
    ]
}

/// Speeds and distances come back to the millimetre, not the bit.
fn same_speed(a: Option<Speed>, b: Option<Speed>) -> bool {
    match (a, b) {
        (Some(a), Some(b)) => (a.metres_per_second() - b.metres_per_second()).abs() < 5e-7,
        (a, b) => a.is_none() && b.is_none(),
    }
}

fn same_distance(a: Option<Distance>, b: Option<Distance>) -> bool {
    match (a, b) {
        (Some(a), Some(b)) => (a.metres() - b.metres()).abs() < 5e-7,
        (a, b) => a.is_none() && b.is_none(),
    }
}

proptest! {
    #[test]
    fn speed_messages_come_back_from_their_bytes(
        v in speed(), d in distance(), dir in direction(),
        key in indicator(), operations in indicator(), reversed in indicator(),
        power in prop::option::of(0u8..=250), exit in prop::option::of(0u8..63),
        source in 0u8..8, lim in limit(), set_point in speed(), cap in speed(),
    ) {
        let wheel = WheelBasedSpeed { speed: v, distance: d, power_time_minutes: power, direction: dir, key_switch: key, implement_operations: operations, operator_reversed: reversed };
        let back = WheelBasedSpeed::from_bytes(wheel.to_bytes());
        prop_assert!(same_speed(back.speed, v) && same_distance(back.distance, d));
        prop_assert_eq!((back.power_time_minutes, back.direction, back.key_switch, back.implement_operations, back.operator_reversed), (power, dir, key, operations, reversed));

        let ground = GroundBasedSpeed { speed: v, distance: d, direction: dir };
        let back = GroundBasedSpeed::from_bytes(ground.to_bytes());
        prop_assert!(same_speed(back.speed, v) && same_distance(back.distance, d));
        prop_assert_eq!(back.direction, dir);

        let source = MachineSelectedSpeed::from_bytes([0, 0, 0, 0, 0, 0, 0, source << 2]).source;
        let selected = MachineSelectedSpeed { speed: v, distance: d, exit_reason: exit, direction: dir, source, limit: lim };
        let back = MachineSelectedSpeed::from_bytes(selected.to_bytes());
        prop_assert!(same_speed(back.speed, v) && same_distance(back.distance, d));
        prop_assert_eq!((back.exit_reason, back.direction, back.source, back.limit), (exit, dir, source, lim));

        let command = MachineSelectedSpeedCommand { set_point, limit: cap, direction: dir };
        let back = MachineSelectedSpeedCommand::from_bytes(command.to_bytes());
        prop_assert!(same_speed(back.set_point, set_point) && same_speed(back.limit, cap));
        prop_assert_eq!(back.direction, dir);
    }

    #[test]
    fn guidance_messages_come_back_from_their_bytes(
        k in curvature(), a in indicator(), b in indicator(), c in indicator(), d in indicator(),
        e in indicator(), lim in limit(), exit in prop::option::of(0u8..63),
    ) {
        let info = GuidanceMachineInfo { curvature: k, mechanical_lockout: a, steering_ready: b, steering_input_position: c, reset_required: d, limit: lim, exit_reason: exit, remote_engage: e };
        prop_assert_eq!(GuidanceMachineInfo::from_bytes(info.to_bytes()), info);
        let command = GuidanceSystemCommand { curvature: k, intends_to_steer: a };
        prop_assert_eq!(GuidanceSystemCommand::from_bytes(command.to_bytes()), command);
    }

    #[test]
    fn any_bytes_decode(bytes in any::<[u8; 8]>(), pick in 0usize..6) {
        let pgns = [WheelBasedSpeed::PGN, GroundBasedSpeed::PGN, MachineSelectedSpeed::PGN, MachineSelectedSpeedCommand::PGN, GuidanceMachineInfo::PGN, GuidanceSystemCommand::PGN];
        let message = Message::decode(&frame(pgns[pick], bytes)).unwrap();
        prop_assert!(message.is_some());
    }
}
