//! PGN decoding against byte layouts built by hand from the standard's field
//! tables.

#![allow(
    clippy::unwrap_used,
    clippy::panic,
    clippy::float_cmp,
    clippy::redundant_closure_for_method_calls
)]

use kinavis_kernel::gnss::FixType;
use kinavis_kernel::time::{Civil, Instant, Utc};
use kinavis_kernel::KernelError;
use kinavis_nmea2000::{
    Assembler, CanId, Frame, GnssSystem, Integrity, Message, Nmea2000Error, Payload, Pgn,
    Referenced, StationClass,
};

fn now() -> Instant<Utc> {
    Instant::from_unix_seconds(1_789_000_000)
}

/// Decodes a group from these frames, source address 35.
fn message(pgn: Pgn, frames: &[&[u8]]) -> Result<Message, Nmea2000Error> {
    let mut assembler = Assembler::new();
    let id = CanId::from_parts(2, pgn, 255, 35);
    let mut whole = None;
    for data in frames {
        whole = assembler.push(&Frame::new(id, data)?, now())?;
    }
    let payload = whole.unwrap();
    assert_eq!(payload.source(), 35);
    Message::decode(&payload)
}

#[test]
fn a_position_rapid_update_is_two_signed_words() {
    // 50.755°N 1.333°W in 1e-7°.
    let Message::PositionRapidUpdate(update) = message(
        Pgn::POSITION_RAPID_UPDATE,
        &[&[0x30, 0x99, 0x40, 0x1E, 0xB0, 0x99, 0x34, 0xFF]],
    )
    .unwrap() else {
        panic!("not a position");
    };
    let position = update.position.unwrap();
    assert!((position.latitude().degrees() - 50.755).abs() <= 1e-7);
    assert!((position.longitude().degrees() + 1.333).abs() <= 1e-7);

    // Not available: 0x7FFFFFFF in a signed field.
    let Message::PositionRapidUpdate(update) = message(
        Pgn::POSITION_RAPID_UPDATE,
        &[&[0xFF, 0xFF, 0xFF, 0x7F, 0xFF, 0xFF, 0xFF, 0x7F]],
    )
    .unwrap() else {
        panic!("not a position");
    };
    assert_eq!(update.position, None);
}

#[test]
fn course_and_speed_carry_their_reference() {
    // SID 5, true, COG 245.0° (42761 in 1e-4 rad), SOG 6.33 m/s.
    let Message::CourseAndSpeed(cog) = message(
        Pgn::COG_SOG_RAPID_UPDATE,
        &[&[0x05, 0xFC, 0x09, 0xA7, 0x79, 0x02, 0xFF, 0xFF]],
    )
    .unwrap() else {
        panic!("not a course");
    };
    assert_eq!(cog.sid, Some(5));
    let course = cog.course.unwrap();
    assert!((course.degrees() - 245.0).abs() < 0.01);
    assert!(course.as_true().is_some());
    assert_eq!(course.as_magnetic(), None);
    assert_eq!(format!("{course}"), "245.0°T");
    assert!((cog.speed.unwrap().metres_per_second() - 6.33).abs() < 1e-9);

    // Reference "magnetic", then "error", then "not available".
    let Message::CourseAndSpeed(cog) = message(
        Pgn::COG_SOG_RAPID_UPDATE,
        &[&[0xFF, 0xFD, 0x09, 0xA7, 0xFF, 0xFF, 0xFF, 0xFF]],
    )
    .unwrap() else {
        panic!("not a course");
    };
    assert_eq!(cog.sid, None);
    assert!(matches!(cog.course, Some(Referenced::Magnetic(_))));
    assert_eq!(cog.speed, None);
    for reference in [0xFE, 0xFF] {
        let Message::CourseAndSpeed(cog) = message(
            Pgn::COG_SOG_RAPID_UPDATE,
            &[&[0x05, reference, 0x09, 0xA7, 0x79, 0x02, 0xFF, 0xFF]],
        )
        .unwrap() else {
            panic!("not a course");
        };
        assert_eq!(cog.course, None, "reference {reference:#x}");
    }
}

#[test]
fn a_heading_brings_deviation_and_variation_with_it() {
    // SID 7, heading 247.0° magnetic, deviation −2.5°, variation +3.0°.
    let Message::VesselHeading(heading) = message(
        Pgn::VESSEL_HEADING,
        &[&[0x07, 0x66, 0xA8, 0x4C, 0xFE, 0x0C, 0x02, 0xFD]],
    )
    .unwrap() else {
        panic!("not a heading");
    };
    let referenced = heading.heading.unwrap();
    assert!((referenced.degrees() - 247.0).abs() < 0.01);
    assert_eq!(format!("{referenced:.2}"), "247.00°M");
    assert!((heading.deviation.unwrap().degrees() + 2.5).abs() < 0.01);
    assert!((heading.variation.unwrap().degrees() - 3.0).abs() < 0.01);

    // All fields not available.
    let Message::VesselHeading(heading) = message(
        Pgn::VESSEL_HEADING,
        &[&[0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0xFF, 0x7F, 0xFF]],
    )
    .unwrap() else {
        panic!("not a heading");
    };
    assert_eq!(heading.heading, None);
    assert_eq!(heading.deviation, None);
    assert_eq!(heading.variation, None);
}

#[test]
fn a_depth_and_its_offset_add_up() {
    // SID 1, 12.34 m below the transducer, offset −1.5 m to the keel, range 100
    // m.
    let Message::WaterDepth(depth) = message(
        Pgn::WATER_DEPTH,
        &[&[0x01, 0xD2, 0x04, 0x00, 0x00, 0x24, 0xFA, 0x0A]],
    )
    .unwrap() else {
        panic!("not a depth");
    };
    assert!((depth.depth.unwrap().metres() - 12.34).abs() < 1e-9);
    assert!((depth.offset.unwrap().metres() + 1.5).abs() < 1e-9);
    assert!((depth.range.unwrap().metres() - 100.0).abs() < 1e-9);
    assert!((depth.depth_from_reference().unwrap().metres() - 10.84).abs() < 1e-9);

    let Message::WaterDepth(depth) = message(
        Pgn::WATER_DEPTH,
        &[&[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0xFF]],
    )
    .unwrap() else {
        panic!("not a depth");
    };
    assert_eq!(depth.depth, None);
    assert_eq!(depth.offset, None);
    assert_eq!(depth.range, None);
    assert_eq!(depth.depth_from_reference(), None);
}

/// GNSS position data with all fields not available: all ones in unsigned
/// fields, `0x7F..` in signed ones.
const NOT_AVAILABLE: [u8; 43] = [
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0xFF,
    0xFF, 0xFF, 0xFF, 0x7F, 0xFF, 0x7F, 0xFF, 0xFF, 0xFF, 0x7F, 0xFF,
];

#[test]
fn gnss_position_data_is_a_whole_fix() {
    // SID 3, 2026-09-18 12:34:56.789, 50.755°N 1.333°W, 48.5 m, GPS+SBAS,
    // differential, integrity safe, 9 satellites, HDOP 0.9, PDOP 1.5, geoidal
    // separation 47.12 m, no reference stations.
    let Message::GnssPosition(gnss) = message(
        Pgn::GNSS_POSITION_DATA,
        &[
            &[0x40, 0x2B, 0x03, 0xEA, 0x50, 0xD2, 0xBD, 0xFF],
            &[0x41, 0x1A, 0x00, 0xE0, 0xBF, 0x2F, 0x09, 0x2E],
            &[0x42, 0x0B, 0x07, 0x00, 0xE0, 0x24, 0x85, 0x6F],
            &[0x43, 0xA4, 0xD0, 0xFF, 0x20, 0x0D, 0xE4, 0x02],
            &[0x44, 0x00, 0x00, 0x00, 0x00, 0x23, 0xFD, 0x09],
            &[0x45, 0x5A, 0x00, 0x96, 0x00, 0x68, 0x12, 0x00],
            &[0x46, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF],
        ],
    )
    .unwrap() else {
        panic!("not a fix");
    };
    assert_eq!(gnss.sid, Some(3));
    let taken_at = gnss.taken_at.unwrap();
    let midnight = Instant::<Utc>::from_civil(Civil::date(2026, 9, 18)).unwrap();
    assert_eq!(
        taken_at.seconds() - midnight.seconds(),
        12 * 3600 + 34 * 60 + 56
    );
    assert_eq!(taken_at.subsec_nanos(), 789_000_000);
    let position = gnss.position.unwrap();
    assert!((position.latitude().degrees() - 50.755).abs() < 1e-12);
    assert!((position.longitude().degrees() + 1.333).abs() < 1e-12);
    assert!((gnss.altitude.unwrap().value().metres() - 48.5).abs() < 1e-6);
    assert_eq!(gnss.system, GnssSystem::GpsWithSbas);
    assert_eq!(gnss.method, Some(FixType::Differential));
    assert_eq!(gnss.integrity, Some(Integrity::Safe));
    assert_eq!(gnss.satellites, Some(9));
    assert_eq!(gnss.hdop.unwrap().value(), 0.9);
    assert_eq!(gnss.pdop.unwrap().value(), 1.5);
    assert!((gnss.geoidal_separation.unwrap().metres() - 47.12).abs() < 1e-9);
    assert_eq!(gnss.reference_stations, Some(0));

    let fix = gnss.fix().unwrap();
    assert_eq!(fix.taken_at(), taken_at);
    assert_eq!(fix.quality().fix_type(), FixType::Differential);
    assert_eq!(fix.quality().satellites(), Some(9));
    assert_eq!(fix.quality().hdop().unwrap().value(), 0.9);
    assert_eq!(fix.quality().pdop().unwrap().value(), 1.5);
    assert_eq!(fix.quality().vdop(), None);

    // All fields not available; no time, so no fix.
    let payload = Payload::from_bytes(Pgn::GNSS_POSITION_DATA, 1, &NOT_AVAILABLE).unwrap();
    let Message::GnssPosition(empty) = Message::decode(&payload).unwrap() else {
        panic!("not a fix");
    };
    assert_eq!(empty.taken_at, None);
    assert_eq!(empty.position, None);
    assert_eq!(empty.method, None);
    assert_eq!(empty.integrity, None);
    assert_eq!(empty.satellites, None);
    assert_eq!(empty.hdop, None);
    assert_eq!(empty.geoidal_separation, None);
    assert_eq!(empty.fix(), None);
    assert_eq!(format!("{}", empty.system), "reserved system 15");

    // Negative DOP is not "not available": it is an out-of-domain value.
    let mut garbage = NOT_AVAILABLE;
    garbage[35] = 0xFF;
    let payload = Payload::from_bytes(Pgn::GNSS_POSITION_DATA, 1, &garbage).unwrap();
    assert!(matches!(
        Message::decode(&payload),
        Err(Nmea2000Error::Value {
            field: "dilution of precision",
            ..
        })
    ));
}

#[test]
fn an_ais_class_a_report_reads_every_field() {
    // Message 1, MMSI 366123456, 37.8°N 122.4°W, accurate, second 33,
    // COG 245.0°, SOG 6.33 m/s, HDG 247°, ROT 918 (98.6°/min), under
    // way using engine.
    let Message::AisPositionReport(report) = message(
        Pgn::AIS_CLASS_A_POSITION_REPORT,
        &[
            &[0xA0, 0x1C, 0x01, 0xC0, 0x99, 0xD2, 0x15, 0x00],
            &[0xA1, 0x3E, 0x0B, 0xB7, 0x80, 0xD2, 0x87, 0x16],
            &[0xA2, 0x85, 0x09, 0xA7, 0x79, 0x02, 0x00, 0x00],
            &[0xA3, 0x00, 0x66, 0xA8, 0x96, 0x03, 0xF0, 0xFF],
            &[0xA4, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF],
        ],
    )
    .unwrap() else {
        panic!("not a report");
    };
    assert_eq!(report.class, StationClass::A);
    assert_eq!(report.message_id, 1);
    assert_eq!(report.mmsi.number(), 366_123_456);
    let position = report.position.unwrap();
    assert!((position.latitude().degrees() - 37.8).abs() < 1e-7);
    assert!((position.longitude().degrees() + 122.4).abs() < 1e-7);
    assert!(report.accurate);
    assert!(!report.raim);
    assert_eq!(report.second, Some(33));
    assert!((report.course.unwrap().degrees() - 245.0).abs() < 0.01);
    assert!((report.speed.unwrap().metres_per_second() - 6.33).abs() < 1e-9);
    assert!((report.heading.unwrap().degrees() - 247.0).abs() < 0.01);
    let turn = report.turn.unwrap();
    assert!((turn.degrees_per_minute() - 98.62).abs() < 0.01, "{turn:?}");
    assert!(!turn.is_to_port());
    assert_eq!(report.status, Some(0));
    let track = report.ground_track().unwrap();
    assert!((track.course_over_ground.degrees() - 245.0).abs() < 0.01);
}

#[test]
fn an_ais_class_b_report_has_no_status_and_no_turn() {
    // Message 18, MMSI 211570180, 8.9365783°N 79.5554683°W, RAIM, second
    // 51, COG 339.9°, SOG 0.15 m/s, heading not available.
    let Message::AisPositionReport(report) = message(
        Pgn::AIS_CLASS_B_POSITION_REPORT,
        &[
            &[0x20, 0x1A, 0x12, 0x04, 0x4E, 0x9C, 0x0C, 0x85],
            &[0x21, 0xCC, 0x94, 0xD0, 0x17, 0x9D, 0x53, 0x05],
            &[0x22, 0xCE, 0xBC, 0xE7, 0x0F, 0x00, 0x00, 0x00],
            &[0x23, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF],
        ],
    )
    .unwrap() else {
        panic!("not a report");
    };
    assert_eq!(report.class, StationClass::B);
    assert_eq!(report.message_id, 18);
    assert_eq!(report.mmsi.number(), 211_570_180);
    let position = report.position.unwrap();
    assert!((position.latitude().degrees() - 8.936_578_3).abs() < 1e-7);
    assert!((position.longitude().degrees() + 79.555_468_3).abs() < 1e-7);
    assert!(!report.accurate);
    assert!(report.raim);
    assert_eq!(report.second, Some(51));
    assert!((report.course.unwrap().degrees() - 339.9).abs() < 0.01);
    assert!((report.speed.unwrap().metres_per_second() - 0.15).abs() < 1e-9);
    assert_eq!(report.heading, None);
    assert_eq!(report.turn, None);
    assert_eq!(report.status, None);
}

#[test]
fn a_group_cut_short_says_how_much_was_needed() {
    let payload = Payload::from_bytes(Pgn::WATER_DEPTH, 1, &[0; 7]).unwrap();
    assert_eq!(
        Message::decode(&payload),
        Err(Nmea2000Error::TooShort {
            pgn: Pgn::WATER_DEPTH,
            bytes: 7,
            needed: 8
        })
    );
    let payload = Payload::from_bytes(Pgn::AIS_CLASS_A_POSITION_REPORT, 1, &[0; 27]).unwrap();
    assert!(matches!(
        Message::decode(&payload),
        Err(Nmea2000Error::TooShort { needed: 28, .. })
    ));
}

#[test]
fn a_value_the_domain_rejects_is_an_error_not_a_position() {
    // Latitude 95° in 1e-7°: 950 000 000 = 0x389FD980.
    let payload = Payload::from_bytes(
        Pgn::POSITION_RAPID_UPDATE,
        1,
        &[0x80, 0xD9, 0x9F, 0x38, 0, 0, 0, 0],
    )
    .unwrap();
    assert!(matches!(
        Message::decode(&payload),
        Err(Nmea2000Error::Value {
            field: "position",
            error: KernelError::OutOfRange { .. }
        })
    ));
}

#[test]
fn a_group_of_another_kind_is_unsupported_and_named() {
    let payload = Payload::from_bytes(Pgn::new(130_306), 1, &[0; 8]).unwrap();
    assert_eq!(
        Message::decode(&payload),
        Ok(Message::Unsupported {
            pgn: Pgn::new(130_306)
        })
    );
}

#[cfg(feature = "serde")]
#[test]
fn a_message_round_trips_through_serde() {
    let message = message(
        Pgn::VESSEL_HEADING,
        &[&[0x07, 0x66, 0xA8, 0x4C, 0xFE, 0x0C, 0x02, 0xFD]],
    )
    .unwrap();
    let json = serde_json::to_string(&message).unwrap();
    assert_eq!(serde_json::from_str::<Message>(&json).unwrap(), message);
}

#[cfg(feature = "serde")]
#[test]
fn a_stored_frame_cannot_claim_more_bytes_than_a_frame_holds() {
    use kinavis_nmea2000::{CanId, Frame};

    let id = CanId::from_parts(2, Pgn::VESSEL_HEADING, 255, 35);
    let frame = Frame::new(id, &[0x07, 0x66, 0xA8]).unwrap();
    let json = serde_json::to_string(&frame).unwrap();
    assert_eq!(serde_json::from_str::<Frame>(&json).unwrap(), frame);
    let too_long = json.replace("\"len\":3", "\"len\":9");
    assert_ne!(too_long, json);
    assert!(serde_json::from_str::<Frame>(&too_long).is_err());
}
