//! Message 21 decoding against values derived by hand from the ITU-R M.1371 bit
//! table.

#![allow(clippy::unwrap_used, clippy::panic, clippy::float_cmp)]

use kinavis_ais::{
    AidToNavigation, AidType, AisError, Bits, Mark, Message, PositionFixingDevice, Quadrant,
};
use kinavis_nmea0183::{parse, Sentence};

fn aid(line: &str) -> Result<AidToNavigation, AisError> {
    let Sentence::Vdm(vdm) = parse(line.as_bytes()).unwrap() else {
        panic!("not an AIS sentence");
    };
    let bits = Bits::unarmour(vdm.payload.as_bytes(), vdm.fill_bits)?;
    match Message::decode(&bits)? {
        Message::AidToNavigation(aid) => Ok(aid),
        other => panic!("not an aid to navigation: {other:?}"),
    }
}

#[test]
fn an_aid_reads_every_field_and_its_name_extension() {
    // South cardinal buoy ROYAL SOVEREIGN SHOALS BUOY (20 characters in the
    // name field, 7 in the extension) at 50°43.0'N 000°26.0'E, surveyed, 4 × 2
    // m, second 45, off position, regional 0xA5, RAIM.
    let aid =
        aid("!AIVDM,1,1,,A,E>jHC6;97dPV@9Wc2a2TSW@9T7Ph0wN<>Pa`P20@8;nrF34p0UCn@,4*69").unwrap();
    assert_eq!(aid.mmsi.number(), 992_351_000);
    assert_eq!(
        aid.aid_type,
        Some(AidType::Floating(Mark::Cardinal(Quadrant::South)))
    );
    assert_eq!(aid.aid_type.unwrap().code(), 22);
    assert_eq!(aid.name.unwrap(), "ROYAL SOVEREIGN SHOALS BUOY");
    assert!(aid.accurate);
    let position = aid.position.unwrap();
    assert!((position.latitude().degrees() - 50.7167).abs() < 1e-6);
    assert!((position.longitude().degrees() - 0.4333).abs() < 1e-6);
    let dimensions = aid.dimensions.unwrap();
    assert_eq!(dimensions.length().metres().round(), 4.0);
    assert_eq!(dimensions.beam().metres().round(), 2.0);
    assert_eq!(aid.fixing_device, Some(PositionFixingDevice::Surveyed));
    assert_eq!(aid.second, Some(45));
    assert_eq!(aid.off_position, Some(true));
    assert_eq!(aid.regional, 0xA5);
    assert!(aid.raim);
    assert!(!aid.is_virtual);
    assert!(!aid.assigned);
}

#[test]
fn a_virtual_aid_has_no_position_of_its_own_to_be_off() {
    // Aid type 0, name `VIRTUAL  `, 181°/91°, no dimensions, EPFD 0, second 60,
    // off-position bit set but undefined, virtual, assigned.
    let aid = aid("!AIVDM,1,1,,B,E>jR060;4a::PV@@000000000006NAc0J2@`000000N@1P,4*51").unwrap();
    assert_eq!(aid.mmsi.number(), 992_509_976);
    assert_eq!(aid.aid_type, None);
    assert_eq!(aid.name.unwrap(), "VIRTUAL");
    assert!(!aid.accurate);
    assert_eq!(aid.position, None);
    assert_eq!(aid.dimensions, None);
    assert_eq!(aid.fixing_device, None);
    assert_eq!(aid.second, None);
    assert_eq!(aid.off_position, None, "only valid with a second");
    assert_eq!(aid.regional, 0);
    assert!(!aid.raim);
    assert!(aid.is_virtual);
    assert!(aid.assigned);
}

#[test]
fn an_extension_of_padding_adds_nothing_and_a_short_message_is_refused() {
    // Name of exactly 20 characters, then three `@`.
    let aid = aid("!AIVDM,1,1,,B,E00000TPQ1R2S3T4U5V6W7`8a9b0000000000000000000000,4*52").unwrap();
    assert_eq!(aid.name.unwrap(), "ABCDEFGHIJKLMNOPQRST");
    assert_eq!(
        aid.aid_type,
        Some(AidType::Beacon(Mark::Cardinal(Quadrant::North)))
    );
    assert_eq!(aid.position.unwrap().latitude().degrees(), 0.0);

    // 6 bits short of the fixed part.
    let bits = Bits::unarmour(b"E>jR060;4a::PV@@000000000006NAc0J2@`000000N@1", 0).unwrap();
    assert_eq!(
        Message::decode(&bits),
        Err(AisError::TooShort {
            bits: 270,
            needed: 272
        })
    );
}

#[test]
fn the_aid_types_are_the_standards_codes() {
    assert_eq!(AidType::from_code(0), None);
    for code in 1..32 {
        let aid_type = AidType::from_code(code).unwrap();
        assert_eq!(aid_type.code(), code, "{aid_type:?}");
    }
    assert_eq!(AidType::from_code(4), Some(AidType::Reserved(4)));
    assert_eq!(
        AidType::from_code(9),
        Some(AidType::Beacon(Mark::Cardinal(Quadrant::North)))
    );
    assert_eq!(AidType::from_code(19), Some(AidType::Beacon(Mark::Special)));
    assert_eq!(
        AidType::from_code(24),
        Some(AidType::Floating(Mark::PortHand))
    );
    assert_eq!(
        AidType::from_code(29),
        Some(AidType::Floating(Mark::SafeWater))
    );
    assert_eq!(AidType::from_code(31), Some(AidType::LightVessel));
    assert_eq!(
        format!("{}", AidType::Floating(Mark::PreferredChannelStarboardHand)),
        "floating mark, preferred channel, starboard hand"
    );
    assert_eq!(
        format!("{}", AidType::Beacon(Mark::Cardinal(Quadrant::West))),
        "beacon, cardinal west"
    );
    assert_eq!(
        format!("{}", AidType::Light { sectored: true }),
        "light with sectors"
    );
}

#[cfg(feature = "serde")]
#[test]
fn an_aid_round_trips_through_serde() {
    let message = Message::AidToNavigation(
        aid("!AIVDM,1,1,,A,E>jHC6;97dPV@9Wc2a2TSW@9T7Ph0wN<>Pa`P20@8;nrF34p0UCn@,4*69").unwrap(),
    );
    let json = serde_json::to_string(&message).unwrap();
    assert!(json.contains("\"ROYAL SOVEREIGN SHOALS BUOY\""), "{json}");
    assert_eq!(serde_json::from_str::<Message>(&json).unwrap(), message);
}
