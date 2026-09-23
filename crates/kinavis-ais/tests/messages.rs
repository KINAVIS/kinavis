//! Position report decoding against values derived by hand from the ITU-R
//! M.1371 bit tables.

#![allow(
    clippy::unwrap_used,
    clippy::panic,
    clippy::float_cmp,
    clippy::redundant_closure_for_method_calls
)]

use kinavis_ais::{
    AisError, Bits, Message, NavigationStatus, PositionFixingDevice, PositionReport, ShipCategory,
    StationClass, Turn,
};
use kinavis_kernel::KernelError;
use kinavis_nmea0183::{parse, Sentence};

/// Single-sentence report.
fn report(line: &str) -> Result<PositionReport, AisError> {
    let Sentence::Vdm(vdm) = parse(line.as_bytes()).unwrap() else {
        panic!("not an AIS sentence");
    };
    let bits = Bits::unarmour(vdm.payload.as_bytes(), vdm.fill_bits)?;
    match Message::decode(&bits)? {
        Message::PositionReport(report) => Ok(report),
        other => panic!("not a position report: {other:?}"),
    }
}

#[test]
fn a_class_a_report_reads_every_field() {
    // Type 3, MMSI 366123456, under way using engine, ROT field 47, SOG
    // 12.3 kn, accurate, 37.8°N 122.4°W, COG 245.0°, HDG 247°, second 33.
    let report = report("!AIVDM,1,1,,A,35M:Ih0;iso?d`0E`Ah9TWg20000,0*60").unwrap();
    assert_eq!(report.kind, 3);
    assert_eq!(report.station_class(), StationClass::A);
    assert_eq!(report.mmsi.number(), 366_123_456);
    assert_eq!(report.status, Some(NavigationStatus::UnderWayUsingEngine));
    let Some(Turn::Rate(rate)) = report.turn else {
        panic!("a measured turn: {:?}", report.turn);
    };
    // (47 / 4.733)² = 98.6°/min, to starboard.
    assert!((rate.degrees_per_minute() - 98.61).abs() < 0.01, "{rate:?}");
    assert!(!rate.is_to_port());
    assert_eq!(report.speed.map(|s| s.knots()), Some(12.3));
    assert!(report.accurate);
    let position = report.position.unwrap();
    assert!((position.latitude().degrees() - 37.8).abs() < 1e-9);
    assert!((position.longitude().degrees() + 122.4).abs() < 1e-9);
    assert_eq!(report.course.map(|c| c.degrees()), Some(245.0));
    assert_eq!(report.heading.map(|h| h.degrees()), Some(247.0));
    assert_eq!(report.second, Some(33));
    assert!(!report.raim);

    let track = report.ground_track().unwrap();
    assert_eq!(track.course_over_ground.degrees(), 245.0);
    assert_eq!(track.speed_over_ground.knots(), 12.3);
    assert_eq!(Message::PositionReport(report).mmsi(), Some(report.mmsi));
}

#[test]
fn a_turn_to_port_is_signed_to_port() {
    // Type 2, fishing, ROT field −47, SOG 0.5 kn, 52.5°N 4.5°E, HDG 359°,
    // second 59.
    let report = report("!AIVDM,1,1,,A,23aEOK7l@5PDVG0N2Vp00;?n0000,0*2C").unwrap();
    assert_eq!(report.kind, 2);
    assert_eq!(report.status, Some(NavigationStatus::Fishing));
    let Some(Turn::Rate(rate)) = report.turn else {
        panic!("a measured turn: {:?}", report.turn);
    };
    assert!((rate.degrees_per_minute() + 98.61).abs() < 0.01, "{rate:?}");
    assert!(rate.is_to_port());
    assert_eq!(report.speed.map(|s| s.knots()), Some(0.5));
    assert_eq!(report.course.map(|c| c.degrees()), Some(0.0));
    assert_eq!(report.heading.map(|h| h.degrees()), Some(359.0));
    assert_eq!(report.second, Some(59));
}

#[test]
fn not_available_is_none_and_never_a_zero() {
    // Type 1, at anchor, ROT −127, SOG 1022, 181°/91°, COG 3600, HDG 511,
    // second 60, RAIM.
    let synthetic = report("!AIVDM,1,1,,A,13aEOK1POv<tSF0l4Q@>4?wp2000,0*48").unwrap();
    assert_eq!(synthetic.status, Some(NavigationStatus::AtAnchor));
    assert_eq!(synthetic.turn, Some(Turn::OffScale { to_port: true }));
    assert_eq!(
        synthetic.speed.map(|s| s.knots()),
        Some(102.2),
        "the limit, or more"
    );
    assert_eq!(synthetic.position, None);
    assert_eq!(synthetic.course, None);
    assert_eq!(synthetic.heading, None);
    assert_eq!(synthetic.second, None);
    assert!(synthetic.raim);
    assert_eq!(synthetic.ground_track(), None);

    // The recorded sentence: status 15 and ROT −128, both "not defined".
    let recorded = report("!AIVDM,1,1,,A,13aEOK?P00PD2wVMdLDRhgvL289?,0*26").unwrap();
    assert_eq!(recorded.status, None);
    assert_eq!(recorded.turn, None);
    assert_eq!(
        recorded.speed.map(|s| s.knots()),
        Some(0.0),
        "zero is a speed"
    );
    assert_eq!(recorded.second, Some(14));
}

#[test]
fn a_class_b_report_has_no_status_and_no_turn() {
    // A recorded type 18: MMSI 211570180, SOG 0.3 kn, 8.9366°N 79.5555°W,
    // COG 339.9°, HDG not available, second 51, RAIM.
    let report = report("!AIVDO,1,1,,,B39i>1000nTu;gQAlAgDOwqUoP06,0*53").unwrap();
    assert_eq!(report.kind, 18);
    assert_eq!(report.station_class(), StationClass::B);
    assert_eq!(report.mmsi.number(), 211_570_180);
    assert_eq!(report.status, None);
    assert_eq!(report.turn, None);
    assert_eq!(report.speed.map(|s| s.knots()), Some(0.3));
    assert!(!report.accurate);
    assert_eq!(report.name, None, "a standard report has no name");
    assert_eq!(report.ship_type, None);
    assert_eq!(report.dimensions, None);
    assert_eq!(report.fixing_device, None);
    let position = report.position.unwrap();
    assert!((position.latitude().degrees() - 8.936_578_3).abs() < 1e-6);
    assert!((position.longitude().degrees() + 79.555_468_3).abs() < 1e-6);
    assert_eq!(report.course.map(|c| c.degrees()), Some(339.9));
    assert_eq!(report.heading, None);
    assert_eq!(report.second, Some(51));
    assert!(report.raim);
}

#[test]
fn an_extended_class_b_report_reads_the_same_fields_from_its_own_places() {
    // Type 19, 312 bits: MMSI 367059850, SOG 8.7 kn, 29.5437°N 88.8104°W,
    // COG 148.3°, HDG 185°, second 40, RAIM at bit 305.
    let report =
        report("!AIVDM,1,1,,A,C5N3SRP0EnJGEBT>NhMLeLl0`:Va04N2`00000000000S0`:1R30,0*49").unwrap();
    assert_eq!(report.kind, 19);
    assert_eq!(report.station_class(), StationClass::B);
    assert_eq!(report.mmsi.number(), 367_059_850);
    assert_eq!(report.speed.map(|s| s.knots()), Some(8.7));
    let position = report.position.unwrap();
    assert!((position.latitude().degrees() - 29.543_691_7).abs() < 1e-6);
    assert!((position.longitude().degrees() + 88.810_391_7).abs() < 1e-6);
    assert_eq!(report.course.map(|c| c.degrees()), Some(148.3));
    assert_eq!(report.heading.map(|h| h.degrees()), Some(185.0));
    assert_eq!(report.second, Some(40));
    assert!(report.raim);

    // Static fields absent from message 18: TEST BOAT, cargo, 30 × 7 m, GPS.
    assert_eq!(report.name.unwrap(), "TEST BOAT");
    assert_eq!(report.ship_type.unwrap().category(), ShipCategory::Cargo);
    let dimensions = report.dimensions.unwrap();
    assert_eq!(dimensions.to_bow.metres().round(), 10.0);
    assert_eq!(dimensions.to_stern.metres().round(), 20.0);
    assert_eq!(dimensions.to_port.metres().round(), 3.0);
    assert_eq!(dimensions.to_starboard.metres().round(), 4.0);
    assert_eq!(report.fixing_device, Some(PositionFixingDevice::Gps));
}

#[test]
fn a_value_the_domain_rejects_is_an_error_not_a_position() {
    // Latitude 95°.
    assert!(matches!(
        report("!AIVDM,1,1,,A,13aEOK000000000nG0@000000000,0*5D"),
        Err(AisError::Value {
            field: "position",
            error: KernelError::OutOfRange { .. }
        })
    ));
    // Heading 400°.
    assert!(matches!(
        report("!AIVDM,1,1,,A,13aEOK000000000000000<P00000,0*68"),
        Err(AisError::Value {
            field: "heading",
            error: KernelError::OutOfRange { .. }
        })
    ));
}

#[test]
fn a_report_cut_short_says_how_much_was_needed() {
    let bits = Bits::unarmour(b"13aEOK?P00PD2wVMdLDRhgvL289", 0).unwrap();
    assert_eq!(
        Message::decode(&bits),
        Err(AisError::TooShort {
            bits: 162,
            needed: 168
        })
    );
    assert_eq!(
        Message::decode(&Bits::new()),
        Err(AisError::TooShort { bits: 0, needed: 6 })
    );
    // Message 19 requires all 312 bits, though only 306 are read.
    let bits = Bits::unarmour(b"C5N3SRP0EnJGEBT>NhMLeLl0`:Va04N2`00000000000S0`:1R3", 0).unwrap();
    assert_eq!(
        Message::decode(&bits),
        Err(AisError::TooShort {
            bits: 306,
            needed: 312
        })
    );
}

#[test]
fn a_message_of_another_type_is_unsupported_and_named() {
    // Type 8, binary broadcast.
    let bits = Bits::unarmour(b"800000P000000000000000000000", 0).unwrap();
    let message = Message::decode(&bits).unwrap();
    assert_eq!(message, Message::Unsupported { kind: 8 });
    assert_eq!(message.mmsi(), None);
}

#[test]
fn the_navigation_statuses_are_the_standards_codes() {
    for code in 0..16 {
        match NavigationStatus::from_code(code) {
            Some(status) => assert_eq!(status.code(), code),
            None => assert_eq!(code, 15),
        }
    }
    assert_eq!(
        NavigationStatus::from_code(9),
        Some(NavigationStatus::Reserved(9))
    );
    assert_eq!(
        NavigationStatus::from_code(14),
        Some(NavigationStatus::SearchAndRescueTransmitter)
    );
    assert_eq!(format!("{}", NavigationStatus::Aground), "aground");
    assert_eq!(format!("{}", StationClass::B), "class B");
}

#[cfg(feature = "serde")]
#[test]
fn a_message_round_trips_through_serde() {
    let message =
        Message::PositionReport(report("!AIVDM,1,1,,A,35M:Ih0;iso?d`0E`Ah9TWg20000,0*60").unwrap());
    let json = serde_json::to_string(&message).unwrap();
    assert_eq!(serde_json::from_str::<Message>(&json).unwrap(), message);
}
