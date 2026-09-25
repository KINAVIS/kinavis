//! Real-world receiver sentences and their expected decoding.

#![allow(
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::panic,
    clippy::indexing_slicing
)]

use core::time::Duration;

use kinavis_kernel::gnss::FixType;
use kinavis_kernel::GnssFix;
use kinavis_kernel::KernelError;
use kinavis_nmea0183::{
    encode, parse, Channel, Date, Mode, NmeaError, Sentence, Status, Talker, TimeOfDay,
    TranslationError, MAX_ACCEPTED_BYTES, MAX_PAYLOAD_CHARS, MAX_SENTENCE_BYTES,
};

#[test]
fn rmc_from_a_receiver() {
    let line = b"$GPRMC,081836,A,3751.65,S,14507.36,E,000.0,360.0,130998,011.3,E*62\r\n";
    let Sentence::Rmc(rmc) = parse(line).unwrap() else {
        panic!("not RMC")
    };
    assert_eq!(rmc.talker, Talker::GP);
    assert_eq!(
        rmc.time,
        Some(TimeOfDay {
            hour: 8,
            minute: 18,
            second: 36,
            nanos: 0
        })
    );
    assert_eq!(rmc.status, Status::Valid);
    let position = rmc.position.unwrap();
    assert!((position.latitude().degrees() + (37.0 + 51.65 / 60.0)).abs() < 1e-12);
    assert!((position.longitude().degrees() - (145.0 + 7.36 / 60.0)).abs() < 1e-12);
    assert_eq!(rmc.speed_over_ground.unwrap().knots(), 0.0);
    // Course 360.0 is north, formatted as 0.
    assert_eq!(rmc.course_over_ground.unwrap().degrees(), 0.0);
    assert_eq!(
        rmc.date,
        Some(Date {
            year: 2098,
            month: 9,
            day: 13
        })
    );
    assert_eq!(rmc.variation.unwrap().degrees(), 11.3);
    assert_eq!(rmc.mode, None);
    assert_eq!(rmc.fix_type(), FixType::Autonomous);
}

#[test]
fn rmc_with_a_warning_and_nothing_else() {
    let Sentence::Rmc(rmc) = parse(b"$GPRMC,,V,,,,,,,,,,N*53").unwrap_or_else(|e| panic!("{e}"))
    else {
        panic!("not RMC")
    };
    assert_eq!(rmc.status, Status::Warning);
    assert_eq!(rmc.position, None);
    assert_eq!(rmc.mode, Some(Mode::NoFix));
    assert_eq!(rmc.fix_type(), FixType::None);
    assert_eq!(GnssFix::try_from(rmc), Err(TranslationError::NoPosition));
}

#[test]
fn gga_from_a_receiver_becomes_a_fix_given_a_date() {
    let line = b"$GPGGA,092750.000,5321.6802,N,00630.3372,W,1,8,1.03,61.7,M,55.2,M,,*76";
    let Sentence::Gga(gga) = parse(line).unwrap() else {
        panic!("not GGA")
    };
    assert_eq!(gga.fix_type, FixType::Autonomous);
    assert_eq!(gga.satellites, Some(8));
    assert_eq!(gga.hdop.unwrap().value(), 1.03);
    assert!((gga.altitude.unwrap().metres() - 61.7).abs() < 1e-9);
    assert!((gga.geoid_separation.unwrap().metres() - 55.2).abs() < 1e-9);
    assert_eq!(gga.differential_age, None);
    assert_eq!(gga.station, None);

    let fix = gga
        .fix_on(Date {
            year: 2011,
            month: 3,
            day: 20,
        })
        .unwrap();
    assert_eq!(format!("{}", fix.taken_at()), "2011-03-20T09:27:50.000 UTC");
    assert_eq!(fix.quality().satellites(), Some(8));
    // HDOP 1.03 × UERE 4 m for an autonomous fix.
    assert!((fix.horizontal_accuracy().unwrap().metres() - 4.12).abs() < 1e-9);
}

#[test]
fn gll_and_vtg_read_their_fields() {
    let Sentence::Gll(gll) = parse(b"$GPGLL,4916.45,N,12311.12,W,225444,A,A*5C").unwrap() else {
        panic!("not GLL")
    };
    assert_eq!(gll.status, Status::Valid);
    assert_eq!(gll.mode, Some(Mode::Autonomous));
    assert_eq!(gll.time.unwrap().hour, 22);

    let Sentence::Vtg(vtg) = parse(b"$GPVTG,054.7,T,034.4,M,005.5,N,010.2,K,A*25").unwrap() else {
        panic!("not VTG")
    };
    assert_eq!(vtg.course_true.unwrap().degrees(), 54.7);
    assert_eq!(vtg.course_magnetic.unwrap().degrees(), 34.4);
    assert_eq!(vtg.speed_knots.unwrap().knots(), 5.5);
    assert!((vtg.speed_kmh.unwrap().kilometres_per_hour() - 10.2).abs() < 1e-9);
    assert_eq!(vtg.speed_over_ground().unwrap().knots(), 5.5);
}

#[test]
fn unsupported_sentences_are_not_errors() {
    let sentence = parse(b"$GPGSA,A,3,04,05,,09,12,,,24,,,,,2.5,1.3,2.1*39").unwrap();
    let Sentence::Unsupported { address } = sentence else {
        panic!("should be unsupported")
    };
    assert_eq!(address.as_str(), "GPGSA");
    // Proprietary and encapsulation sentences too, checksum verified.
    assert!(matches!(
        parse(b"$PGRMM,WGS 84*06").unwrap(),
        Sentence::Unsupported { .. }
    ));
    assert!(matches!(
        parse(b"!AIXYZ,1,1,,A,13aEOK?P00PD2wVMdLDRhgvL289?,0*22").unwrap(),
        Sentence::Unsupported { .. }
    ));
    assert!(matches!(
        parse(b"$PGRMM,WGS 84*00").unwrap_err(),
        NmeaError::BadChecksum { .. }
    ));
}

#[test]
fn values_the_domain_rejects_are_named_by_field() {
    // Latitude 95°: parses, rejected by the type.
    let err = parse(b"$GPGLL,9516.45,N,12311.12,W,225444,A*30").unwrap_err();
    assert!(matches!(err, NmeaError::Value { index: 0, .. }), "{err:?}");
    // A course of 400°.
    let err = parse(b"$GPVTG,400.0,T,,M,5.0,N,9.3,K*6B").unwrap_err();
    assert!(matches!(err, NmeaError::Value { index: 0, .. }), "{err:?}");
    // Too few fields.
    let err = parse(b"$GPRMC,081836,A*0E").unwrap_err();
    assert!(
        matches!(
            err,
            NmeaError::TooFewFields {
                found: 2,
                required: 11
            }
        ),
        "{err:?}"
    );
}

#[test]
fn a_leap_second_reading_names_no_moment() {
    let Sentence::Rmc(rmc) =
        parse(b"$GPRMC,235960,A,4916.45,N,12311.12,W,0.0,0.0,311216,,*0A").unwrap()
    else {
        panic!("not RMC")
    };
    assert!(matches!(
        GnssFix::try_from(rmc),
        Err(TranslationError::BadMoment(_))
    ));
}

#[test]
fn every_supported_sentence_survives_a_round_trip() {
    let lines: [&[u8]; 6] = [
        b"$GPRMC,081836.00,A,3751.6500,S,14507.3600,E,0.0,0.0,130998,11.3,E*79",
        b"$GNRMC,225444.25,A,4916.4500,N,12311.1200,W,3.0,272.5,110926,5.0,W,D*29",
        b"$GPGGA,092750.00,5321.6802,N,00630.3372,W,1,08,1.0,61.7,M,55.2,M,,*45",
        b"$GPGGA,,,,,,0,,,,M,,M,,*66",
        b"$GPGLL,4916.4500,N,12311.1200,W,225444.00,A,A*72",
        b"$GPVTG,54.7,T,34.4,M,5.5,N,10.2,K,A*15",
    ];
    for line in lines {
        let sentence =
            parse(line).unwrap_or_else(|e| panic!("{e}: {}", String::from_utf8_lossy(line)));
        let written = format!("{sentence}");
        assert_eq!(
            written.as_bytes(),
            line,
            "{}",
            String::from_utf8_lossy(line)
        );
        let again = parse(written.as_bytes()).unwrap();
        assert_eq!(again, sentence);

        let mut buffer = [0_u8; kinavis_nmea0183::MAX_SENTENCE_BYTES];
        let length = encode(&sentence, &mut buffer).unwrap();
        assert_eq!(&buffer[..length - 2], line);
        assert_eq!(&buffer[length - 2..length], b"\r\n");
        assert_eq!(
            encode(&sentence, &mut [0_u8; 10]),
            Err(NmeaError::BufferTooSmall)
        );
    }
}

#[test]
fn differential_age_is_a_duration() {
    let Sentence::Gga(gga) =
        parse(b"$GPGGA,092750.00,5321.6802,N,00630.3372,W,2,08,1.0,61.7,M,55.2,M,3.5,0120*6D")
            .unwrap()
    else {
        panic!("not GGA")
    };
    assert_eq!(gga.differential_age, Some(Duration::from_millis(3500)));
    assert_eq!(gga.station, Some(120));
    assert_eq!(gga.fix_type, FixType::Differential);
}

#[test]
fn vdm_carries_an_ais_message_still_armoured() {
    let line = b"!AIVDM,1,1,,A,13aEOK?P00PD2wVMdLDRhgvL289?,0*26\r\n";
    let Sentence::Vdm(vdm) = parse(line).unwrap() else {
        panic!("not VDM")
    };
    assert_eq!(vdm.talker.as_str(), "AI");
    assert!(!vdm.own);
    assert!(vdm.is_whole());
    assert_eq!((vdm.fragments, vdm.fragment), (1, 1));
    assert_eq!(vdm.sequence, None);
    assert_eq!(vdm.channel, Some(Channel::A));
    assert_eq!(vdm.payload.as_str(), "13aEOK?P00PD2wVMdLDRhgvL289?");
    assert_eq!(vdm.payload.as_bytes().len(), 28);
    assert_eq!(vdm.fill_bits, 0);
    // Round trip.
    assert_eq!(
        format!("{vdm}"),
        "!AIVDM,1,1,,A,13aEOK?P00PD2wVMdLDRhgvL289?,0*26"
    );
    let mut out = [0_u8; 82];
    let written = encode(&vdm, &mut out).unwrap();
    assert_eq!(
        &out[..written],
        b"!AIVDM,1,1,,A,13aEOK?P00PD2wVMdLDRhgvL289?,0*26\r\n"
    );

    // Numeric channel reads the same.
    let Sentence::Vdm(digit) = parse(b"!AIVDM,1,1,,1,13aEOK?P00PD2wVMdLDRhgvL289?,0*56").unwrap()
    else {
        panic!("not VDM")
    };
    assert_eq!(digit.channel, Some(Channel::A));
}

#[test]
fn vdm_fragments_carry_their_sequence_and_fill_bits() {
    let first = b"!AIVDM,2,1,3,B,55P5TL01VIaAL@7WKO@mBplU@<PDhh000000001S;AJ::4A80?4i@E53,0*3E";
    let second = b"!AIVDM,2,2,3,B,1@0000000000000,2*55";
    let Sentence::Vdm(one) = parse(first).unwrap() else {
        panic!("not VDM")
    };
    let Sentence::Vdm(two) = parse(second).unwrap() else {
        panic!("not VDM")
    };
    assert!(!one.is_whole());
    assert_eq!((one.fragments, one.fragment), (2, 1));
    assert_eq!((two.fragments, two.fragment), (2, 2));
    assert_eq!(one.sequence, Some(3));
    assert_eq!(two.sequence, Some(3));
    assert_eq!(one.channel, Some(Channel::B));
    assert_eq!(one.fill_bits, 0);
    assert_eq!(two.fill_bits, 2);
    assert_eq!(two.payload.as_str(), "1@0000000000000");
    assert_eq!(format!("{two}"), core::str::from_utf8(second).unwrap());
}

#[test]
fn vdo_is_own_ship_and_may_leave_the_channel_empty() {
    let Sentence::Vdm(own) = parse(b"!AIVDO,1,1,,,B39i>1000nTu;gQAlAgDOwqUoP06,0*53").unwrap()
    else {
        panic!("not VDO")
    };
    assert!(own.own);
    assert_eq!(own.channel, None);
    assert_eq!(
        format!("{own}"),
        "!AIVDO,1,1,,,B39i>1000nTu;gQAlAgDOwqUoP06,0*53"
    );
}

#[test]
fn vdm_fields_that_cannot_be_are_refused_by_position() {
    // Checksums computed per body.
    let bad = |body: &str| {
        let sum = body.bytes().fold(0_u8, |sum, byte| sum ^ byte);
        let line = format!("!{body}*{sum:02X}");
        parse(line.as_bytes()).unwrap_err()
    };
    // Too many fragments; fragment number beyond count; multi-fragment without
    // sequence; sequence out of range; unknown channel; character outside the
    // armouring; too many fill bits; too few fields.
    assert!(matches!(
        bad("AIVDM,10,1,,A,13aEOK,0"),
        NmeaError::BadField { index: 0, .. }
    ));
    assert!(matches!(
        bad("AIVDM,1,2,,A,13aEOK,0"),
        NmeaError::BadField { index: 1, .. }
    ));
    assert!(matches!(
        bad("AIVDM,2,1,,A,13aEOK,0"),
        NmeaError::BadField { index: 2, .. }
    ));
    assert!(matches!(
        bad("AIVDM,2,1,12,A,13aEOK,0"),
        NmeaError::BadField { index: 2, .. }
    ));
    assert!(matches!(
        bad("AIVDM,1,1,,C,13aEOK,0"),
        NmeaError::BadField { index: 3, .. }
    ));
    assert!(matches!(
        bad("AIVDM,1,1,,A,13aE{K,0"),
        NmeaError::BadField { index: 4, .. }
    ));
    assert!(matches!(
        bad("AIVDM,1,1,,A,13aEOK,6"),
        NmeaError::BadField { index: 5, .. }
    ));
    assert!(matches!(
        bad("AIVDM,1,1,,A,13aEOK"),
        NmeaError::TooFewFields { .. }
    ));

    // Longest payload accepted; a sentence over the maximum length is rejected
    // before field parsing.
    let longest = "0".repeat(MAX_PAYLOAD_CHARS);
    let body = format!("AIVDM,1,1,,A,{longest},0");
    let sum = body.bytes().fold(0_u8, |sum, byte| sum ^ byte);
    assert!(parse(format!("!{body}*{sum:02X}").as_bytes()).is_ok());
}

#[test]
fn a_dop_below_the_written_precision_still_parses_after_writing() {
    // Fuzz-found: HDOP `.03` was written as `0.0`, which is not a valid DOP.
    let line = b"$GPGGA,092750.000,5321.6802,N,00630.3372,W,1,81,.03,61.7,M,5.52,M*76\r\n";
    let first = parse(line).unwrap();
    let written = first.to_string();
    let again = parse(written.as_bytes()).unwrap();
    assert_eq!(again.to_string(), written);
    let Sentence::Gga(gga) = again else {
        panic!("not GGA")
    };
    assert_eq!(gga.hdop.unwrap().value(), 0.1);
}

/// `$body*hh`.
fn with_checksum(body: &str) -> String {
    let sum = body.bytes().fold(0_u8, |sum, byte| sum ^ byte);
    format!("${body}*{sum:02X}")
}

#[test]
fn values_beyond_the_plausibility_bounds_are_refused_by_field() {
    let cases = [
        // Fuzz-found: a 55-digit speed that re-encoded past 82 bytes.
        (
            "GPRMC,,V,,,,,1111111111111111111111111111111111111111111111111111113,333,,,,N",
            6,
        ),
        ("GPRMC,,V,,,,,-0.1,,,,,N", 6),
        ("GPVTG,,T,,M,,N,1852.1,K,A", 6),
        ("GPGGA,,,,,,1,,100.1,,M,,M,,", 7),
        ("GPGGA,,,,,,1,,,100000.1,M,,M,,", 8),
        ("GPGGA,,,,,,1,,,,M,-1000.1,M,,", 10),
        ("GPGGA,,,,,,1,,,,M,,M,9999.1,", 12),
    ];
    for (body, field) in cases {
        match parse(with_checksum(body).as_bytes()) {
            Err(NmeaError::Value {
                index,
                error: KernelError::OutOfRange { .. },
            }) => assert_eq!(index, field, "{body}"),
            other => panic!("{body}: {other:?}"),
        }
    }
}

#[test]
fn sentences_at_their_bounds_fit_the_standard_length() {
    // Shortest input forms of the widest values: every field grows on writing.
    let bodies = [
        "GPRMC,000000,A,8959.99999,S,17959.99999,W,1000,359.99,311299,180,W,A",
        "GPGLL,8959.99999,S,17959.99999,W,000000,A,A",
        "GPVTG,359.99,T,359.99,M,1000,N,1852,K,A",
        "GPGGA,000000,8959.99999,S,17959.99999,W,8,99,99.99,-9999.9,M,-999.9,M,,1023",
    ];
    for body in bodies {
        let sentence = parse(with_checksum(body).as_bytes()).unwrap();
        let mut out = [0_u8; MAX_SENTENCE_BYTES];
        let length = encode(&sentence, &mut out).unwrap_or_else(|e| panic!("{body}: {e}"));
        let again = parse(&out[..length]).unwrap();
        assert_eq!(again.to_string(), sentence.to_string(), "{body}");
    }
}

#[test]
fn a_sentence_that_would_exceed_the_standard_length_is_refused_by_encode() {
    let body = "GPGGA,000000,0000,N,00000,E,1,99,100,-10000,M,-1000,M,9999,9999";
    let sentence = parse(with_checksum(body).as_bytes()).unwrap();
    assert!(sentence.to_string().len() + 2 > MAX_SENTENCE_BYTES);
    assert!(matches!(
        encode(&sentence, &mut [0_u8; 256]),
        Err(NmeaError::TooLong {
            limit: MAX_SENTENCE_BYTES,
            ..
        })
    ));
}

// Sentences below are from the gpsd regression logs (`test/daemon`, copyright
// the GPSD project, BSD-2-Clause), recorded from the receivers named.

#[test]
fn a_high_precision_receiver_is_read_past_the_standard_length() {
    // u-blox ZED-F9P with high-precision NMEA: 89 bytes.
    let line = "$GNGGA,014500.00,4404.1306024,N,12118.8446777,W,2,12,0.49,1129.913,M,-21.350,M,,0278*4C\r\n";
    assert!(line.len() > MAX_SENTENCE_BYTES);
    let Sentence::Gga(gga) = parse(line.as_bytes()).unwrap() else {
        panic!("not GGA")
    };
    let position = gga.position.unwrap();
    assert_eq!(
        format!("{:.7}", position.latitude().degrees()),
        "44.0688434"
    );
    assert_eq!(
        format!("{:.7}", position.longitude().degrees()),
        "-121.3140780"
    );
    assert_eq!(gga.hdop.unwrap().value(), 0.49);
}

#[test]
fn a_dop_of_zero_before_the_first_fix_reads_as_not_available() {
    // SkyTraq S2525F8-BD-RTK after a cold start: 84 bytes, HDOP `0.0`.
    let line = "$GPGGA,212406.000,0000.0000000,N,00000.0000000,E,0,00,0.0,0.000,M,0.000,M,,0000*6E";
    let Sentence::Gga(gga) = parse(line.as_bytes()).unwrap() else {
        panic!("not GGA")
    };
    assert_eq!(gga.hdop, None);
    assert_eq!(gga.fix_type, FixType::None);
}

#[test]
fn an_ais_fragment_with_a_64_character_payload_is_read() {
    // AIS receiver: the first of two fragments of a type 5 message, 86 bytes.
    let line =
        "!AIVDM,2,1,0,A,56K2=I02ADS``C;W3J0PuE8Tr0pvs>222222221@6`8376630:2T81SP`3iQ`888,0*51";
    let Sentence::Vdm(vdm) = parse(line.as_bytes()).unwrap() else {
        panic!("not VDM")
    };
    assert_eq!(vdm.payload.as_bytes().len(), 64);
}

#[test]
fn a_sentence_past_the_accepted_length_is_refused() {
    let body = format!("GPGGA,{}0", "0,".repeat(46));
    let line = with_checksum(&body);
    assert_eq!(line.len(), MAX_ACCEPTED_BYTES + 1);
    assert_eq!(
        parse(line.as_bytes()),
        Err(NmeaError::TooLong {
            length: MAX_ACCEPTED_BYTES + 1,
            limit: MAX_ACCEPTED_BYTES,
        })
    );
}

#[test]
fn a_variation_of_999_9_reads_as_not_available() {
    // Caterpillar MS352 without a magnetic model: 93 bytes, variation `999.9`.
    let line =
        "$GPRMC,113938.50,A,3842.86006889,N,11705.43645510,W,0.011,46.405,231122,999.9000,E,D*2E";
    let Sentence::Rmc(rmc) = parse(line.as_bytes()).unwrap() else {
        panic!("not RMC")
    };
    assert_eq!(rmc.variation, None);
    assert!(GnssFix::try_from(rmc).is_ok());
    // Between the physical limit and the marker: a corrupt field.
    assert!(matches!(
        parse(
            with_checksum("GPRMC,113938.50,A,3842.86,N,11705.43,W,0.0,46.4,231122,500.0,E,D")
                .as_bytes()
        ),
        Err(NmeaError::Value { .. })
    ));
}

#[test]
fn a_variation_without_a_magnitude_reads_as_not_available() {
    // Saab R4 AIS transponder: no variation direction field, so the mode
    // indicator sits where the direction belongs.
    let line = "$GPRMC,130711.00,A,5012.790800,N,00806.879600,W,0.5,3.0,010611,,A*6A";
    let Sentence::Rmc(rmc) = parse(line.as_bytes()).unwrap() else {
        panic!("not RMC")
    };
    assert_eq!(rmc.variation, None);
    assert!(GnssFix::try_from(rmc).is_ok());
}
