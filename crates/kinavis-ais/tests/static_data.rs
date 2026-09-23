//! Messages 5 and 24 decoding against values derived by hand from the ITU-R
//! M.1371 bit tables.

#![allow(
    clippy::unwrap_used,
    clippy::panic,
    clippy::float_cmp,
    clippy::redundant_closure_for_method_calls
)]

use kinavis_ais::{
    AisError, Assembler, Bits, ClassBStaticData, Eta, HazardCategory, Message,
    PositionFixingDevice, ShipCategory, ShipType, StaticDataPart,
};
use kinavis_kernel::{Instant, Utc};
use kinavis_nmea0183::{parse, Sentence};

/// Reassembled message from the sentences.
fn message(lines: &[&str]) -> Result<Message, AisError> {
    let mut assembler = Assembler::new();
    let now = Instant::<Utc>::from_unix_seconds(1_789_000_000);
    let mut whole = None;
    for line in lines {
        let Sentence::Vdm(vdm) = parse(line.as_bytes()).unwrap() else {
            panic!("not an AIS sentence");
        };
        whole = assembler.push(&vdm, now)?;
    }
    Message::decode(&whole.unwrap())
}

#[test]
fn a_recorded_type_5_reads_every_field() {
    // MT. MITCHELL, US survey ship: MMSI 369190000, IMO 6710932, call sign
    // WDA9674, type 99, 180 × 20 m, GPS, ETA 2 January 08:00, draught 6.0 m,
    // destination Seattle.
    let Message::StaticAndVoyageData(data) = message(&[
        "!AIVDM,2,1,3,B,55P5TL01VIaAL@7WKO@mBplU@<PDhh000000001S;AJ::4A80?4i@E53,0*3E",
        "!AIVDM,2,2,3,B,1@0000000000000,2*55",
    ])
    .unwrap() else {
        panic!("not static and voyage data");
    };
    assert_eq!(data.mmsi.number(), 369_190_000);
    assert_eq!(data.ais_version, 0);
    assert_eq!(data.imo, Some(6_710_932));
    assert_eq!(data.callsign.unwrap(), "WDA9674");
    assert_eq!(data.name.unwrap(), "MT.MITCHELL");
    let ship_type = data.ship_type.unwrap();
    assert_eq!(ship_type.code(), 99);
    assert_eq!(ship_type.category(), ShipCategory::Other);
    assert_eq!(ship_type.hazard(), None);
    let dimensions = data.dimensions.unwrap();
    assert_eq!(dimensions.to_bow.metres().round(), 90.0);
    assert_eq!(dimensions.to_stern.metres().round(), 90.0);
    assert_eq!(dimensions.to_port.metres().round(), 10.0);
    assert_eq!(dimensions.to_starboard.metres().round(), 10.0);
    assert_eq!(dimensions.length().metres().round(), 180.0);
    assert_eq!(dimensions.beam().metres().round(), 20.0);
    assert_eq!(data.fixing_device, Some(PositionFixingDevice::Gps));
    assert_eq!(
        data.eta,
        Eta {
            month: Some(1),
            day: Some(2),
            hour: Some(8),
            minute: Some(0),
        }
    );
    assert_eq!(format!("{}", data.eta), "01-02 08:00");
    assert!((data.draught.unwrap().metres() - 6.0).abs() < 1e-9);
    assert_eq!(data.destination.unwrap(), "SEATTLE");
    assert!(data.data_terminal_ready);
}

#[test]
fn not_available_is_none_in_the_static_data_too() {
    // IMO 0, all-`@` call sign, name and destination, dimensions 0,
    // EPFD 0, ETA month 0, day 0, hour 17, minute 60, draught 255,
    // type 81, DTE 1.
    let Message::StaticAndVoyageData(data) = message(&[
        "!AIVDM,2,1,1,A,53aEOK800000000000000000000000000000001A0000000Atwh00000,0*60",
        "!AIVDM,2,2,1,A,000000000000008,2*2D",
    ])
    .unwrap() else {
        panic!("not static and voyage data");
    };
    assert_eq!(data.mmsi.number(), 244_670_316);
    assert_eq!(data.ais_version, 2);
    assert_eq!(data.imo, None);
    assert_eq!(data.callsign, None);
    assert_eq!(data.name, None);
    assert_eq!(data.dimensions, None);
    assert_eq!(data.fixing_device, None);
    assert_eq!(data.destination, None);
    assert_eq!(
        data.eta,
        Eta {
            month: None,
            day: None,
            hour: Some(17),
            minute: None,
        }
    );
    assert!(data.eta.is_given());
    assert_eq!(format!("{}", data.eta), "??-?? 17:??");
    assert!(!Eta::NONE.is_given());
    assert!(
        (data.draught.unwrap().metres() - 25.5).abs() < 1e-9,
        "the limit, or more"
    );
    let ship_type = data.ship_type.unwrap();
    assert_eq!(ship_type.category(), ShipCategory::Tanker);
    assert_eq!(ship_type.hazard(), Some(HazardCategory::A));
    assert_eq!(format!("{ship_type}"), "tanker, hazard category A");
    assert!(!data.data_terminal_ready);
}

#[test]
fn text_keeps_inner_spaces_and_loses_the_padding() {
    // Call sign `DA BC`, 20-character name, destination `ROTTERDAM  ` padded
    // with `@`; dimensions at their limits; ETA month 13 (undefined).
    let Message::StaticAndVoyageData(data) = message(&[
        "!AIVDM,2,1,2,B,539>JhD2:N2T@608<00EHE:0LUHDr0tJ104p4l5:wwwww?Oos`4Sm51D,0*19",
        "!AIVDM,2,2,2,B,Q0CH80000000000,2*47",
    ])
    .unwrap() else {
        panic!("not static and voyage data");
    };
    assert_eq!(data.imo, Some(9_074_729));
    assert_eq!(data.callsign.unwrap(), "DA BC");
    assert_eq!(data.name.unwrap(), "EVER GIVEN OF PANAMA");
    assert_eq!(data.destination.unwrap(), "ROTTERDAM");
    let dimensions = data.dimensions.unwrap();
    assert_eq!(dimensions.to_bow.metres().round(), 511.0);
    assert_eq!(dimensions.to_starboard.metres().round(), 63.0);
    assert_eq!(
        data.fixing_device,
        Some(PositionFixingDevice::GpsAndGlonass)
    );
    assert_eq!(
        data.eta,
        Eta {
            month: None,
            day: Some(31),
            hour: Some(23),
            minute: Some(59),
        }
    );
    assert_eq!(data.ship_type.unwrap().category(), ShipCategory::Cargo);
    assert_eq!(data.ship_type.unwrap().hazard(), Some(HazardCategory::D));
    assert!((data.draught.unwrap().metres() - 16.0).abs() < 1e-9);
}

#[test]
fn a_type_5_cut_short_says_how_much_was_needed() {
    let bits = Bits::unarmour(
        b"55P5TL01VIaAL@7WKO@mBplU@<PDhh000000001S;AJ::4A80?4i@E53",
        0,
    )
    .unwrap();
    assert_eq!(
        Message::decode(&bits),
        Err(AisError::TooShort {
            bits: 336,
            needed: 424
        })
    );
}

#[test]
fn a_class_b_static_data_report_comes_in_two_parts() {
    // Part A, 160 bits: PELICAN.
    let Message::StaticDataReport(part_a) =
        message(&["!AIVDM,1,1,,B,H52MJh10DhT<4p0000000000000,2*36"]).unwrap()
    else {
        panic!("not a static data report");
    };
    assert_eq!(part_a.mmsi.number(), 338_123_456);
    let StaticDataPart::A { name } = part_a.part else {
        panic!("not part A: {:?}", part_a.part);
    };
    assert_eq!(name.unwrap(), "PELICAN");

    // Part B: pleasure craft, vendor SRT model 3 serial 123456, call sign
    // WDF1234, 16 × 5 m, internal GNSS.
    let Message::StaticDataReport(part_b) =
        message(&["!AIVDM,1,1,,B,H52MJh4UCBD<N90G46ijkl1P423t,0*3B"]).unwrap()
    else {
        panic!("not a static data report");
    };
    assert_eq!(part_b.mmsi, part_a.mmsi);
    let StaticDataPart::B(ClassBStaticData {
        ship_type,
        vendor_id,
        unit_model,
        serial_number,
        callsign,
        dimensions,
        mother_ship,
        fixing_device,
    }) = part_b.part
    else {
        panic!("not part B: {:?}", part_b.part);
    };
    assert_eq!(ship_type, ShipType::from_code(37));
    assert_eq!(ship_type.unwrap().category(), ShipCategory::PleasureCraft);
    assert_eq!(vendor_id.unwrap(), "SRT");
    assert_eq!(unit_model, 3);
    assert_eq!(serial_number, 123_456);
    assert_eq!(callsign.unwrap(), "WDF1234");
    let dimensions = dimensions.unwrap();
    assert_eq!(dimensions.length().metres().round(), 16.0);
    assert_eq!(dimensions.beam().metres().round(), 5.0);
    assert_eq!(mother_ship, None);
    assert_eq!(fixing_device, Some(PositionFixingDevice::InternalGnss));
}

#[test]
fn a_craft_carried_by_a_mother_ship_names_her_instead_of_dimensions() {
    // MMSI 982111234 (98, MID, four digits), mothership MMSI 211123456 in place
    // of the dimensions.
    let Message::StaticDataReport(report) =
        message(&["!AIVDM,1,1,,A,H>`WD0T000000000000000<UGl00,0*35"]).unwrap()
    else {
        panic!("not a static data report");
    };
    assert_eq!(report.mmsi.number(), 982_111_234);
    let StaticDataPart::B(data) = report.part else {
        panic!("not part B: {:?}", report.part);
    };
    assert_eq!(data.dimensions, None);
    assert_eq!(data.mother_ship.unwrap().number(), 211_123_456);
    assert_eq!(data.ship_type, None);
    assert_eq!(data.vendor_id, None);
    assert_eq!(data.callsign, None);
    assert_eq!(data.fixing_device, None);
}

#[test]
fn a_reserved_part_number_is_unsupported() {
    assert_eq!(
        message(&["!AIVDM,1,1,,A,H52MJh800000000000000000000,2*3C"]),
        Ok(Message::Unsupported { kind: 24 })
    );
    // Part B requires 168 bits; part A 160.
    let bits = Bits::unarmour(b"H52MJh4UCBD<N90G46ijkl1P423", 0).unwrap();
    assert_eq!(
        Message::decode(&bits),
        Err(AisError::TooShort {
            bits: 162,
            needed: 168
        })
    );
    let bits = Bits::unarmour(b"H52MJh10DhT<4p000000000000", 0).unwrap();
    assert_eq!(
        Message::decode(&bits),
        Err(AisError::TooShort {
            bits: 156,
            needed: 160
        })
    );
}

#[test]
fn the_ship_type_codes_group_by_their_first_digit() {
    assert_eq!(ShipType::from_code(0), None);
    let cases = [
        (20, ShipCategory::WingInGround, None),
        (24, ShipCategory::WingInGround, Some(HazardCategory::D)),
        (30, ShipCategory::Fishing, None),
        (32, ShipCategory::TowingLarge, None),
        (36, ShipCategory::Sailing, None),
        (42, ShipCategory::HighSpeedCraft, Some(HazardCategory::B)),
        (49, ShipCategory::HighSpeedCraft, None),
        (51, ShipCategory::SearchAndRescue, None),
        (52, ShipCategory::Tug, None),
        (59, ShipCategory::Noncombatant, None),
        (63, ShipCategory::Passenger, Some(HazardCategory::C)),
        (70, ShipCategory::Cargo, None),
        (85, ShipCategory::Tanker, None),
        (91, ShipCategory::Other, Some(HazardCategory::A)),
        (19, ShipCategory::Reserved, None),
        (56, ShipCategory::Reserved, None),
        (100, ShipCategory::Reserved, None),
        (255, ShipCategory::Reserved, None),
    ];
    for (code, category, hazard) in cases {
        let ship_type = ShipType::from_code(code).unwrap();
        assert_eq!(ship_type.code(), code);
        assert_eq!(ship_type.category(), category, "{code}");
        assert_eq!(ship_type.hazard(), hazard, "{code}");
    }
    assert_eq!(format!("{}", ShipType::from_code(52).unwrap()), "tug");
    assert_eq!(
        format!("{}", ShipType::from_code(100).unwrap()),
        "ship type 100"
    );
    assert_eq!(
        format!("{}", ShipType::from_code(63).unwrap()),
        "passenger, hazard category C"
    );
}

#[test]
fn the_fixing_devices_are_the_standards_codes() {
    assert_eq!(PositionFixingDevice::from_code(0), None);
    for code in 1..16 {
        let device = PositionFixingDevice::from_code(code).unwrap();
        assert_eq!(device.code(), code);
        assert_eq!(
            matches!(device, PositionFixingDevice::Reserved(_)),
            (9..=14).contains(&code),
            "{code}"
        );
    }
    assert_eq!(format!("{}", PositionFixingDevice::Surveyed), "surveyed");
    assert_eq!(
        format!("{}", PositionFixingDevice::Reserved(9)),
        "reserved device 9"
    );
}

#[cfg(feature = "serde")]
#[test]
fn the_static_data_round_trips_through_serde() {
    let voyage = message(&[
        "!AIVDM,2,1,3,B,55P5TL01VIaAL@7WKO@mBplU@<PDhh000000001S;AJ::4A80?4i@E53,0*3E",
        "!AIVDM,2,2,3,B,1@0000000000000,2*55",
    ])
    .unwrap();
    let json = serde_json::to_string(&voyage).unwrap();
    assert!(json.contains("\"MT.MITCHELL\""), "{json}");
    assert_eq!(serde_json::from_str::<Message>(&json).unwrap(), voyage);

    let part_b = message(&["!AIVDM,1,1,,B,H52MJh4UCBD<N90G46ijkl1P423t,0*3B"]).unwrap();
    let json = serde_json::to_string(&part_b).unwrap();
    assert_eq!(serde_json::from_str::<Message>(&json).unwrap(), part_b);
}
