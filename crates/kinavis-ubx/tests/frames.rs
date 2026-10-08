//! Framing on a noisy, mixed port; translation of a fix into the kernel.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::float_cmp,
    clippy::indexing_slicing,
    clippy::type_complexity,
    clippy::semicolon_if_nothing_returned
)]

use kinavis_kernel::gnss::FixType;
use kinavis_kernel::time::{Civil, Instant, LeapSeconds, Utc};
use kinavis_ubx::{checksum, DeframeError, Deframer, Event, Message, SYNC_1, SYNC_2};
use proptest::prelude::*;

/// A complete frame with its checksum.
fn frame(class: u8, id: u8, payload: &[u8]) -> Vec<u8> {
    let mut out = vec![SYNC_1, SYNC_2, class, id];
    out.extend_from_slice(&u16::try_from(payload.len()).unwrap().to_le_bytes());
    out.extend_from_slice(payload);
    let (a, b) = checksum(class, id, payload);
    out.extend_from_slice(&[a, b]);
    out
}

/// Everything the deframer reports for a byte stream: frames as (class, id,
/// payload), passthrough bytes, errors.
fn feed(bytes: &[u8]) -> (Vec<(u8, u8, Vec<u8>)>, Vec<u8>, Vec<DeframeError>) {
    let mut deframer: Deframer<128> = Deframer::new();
    let (mut frames, mut text, mut errors) = (vec![], vec![], vec![]);
    for &byte in bytes {
        match deframer.push(byte) {
            Some(Event::Frame(frame)) => {
                frames.push((frame.class, frame.id, frame.payload.to_vec()))
            }
            Some(Event::Passthrough(byte)) => text.push(byte),
            Some(Event::Error(error)) => errors.push(error),
            None => {}
        }
    }
    (frames, text, errors)
}

/// A NAV-PVT of an RTK-fixed receiver on 2026-10-08 12:34:56.25 UTC.
fn pvt_payload() -> Vec<u8> {
    let mut p = vec![0_u8; 92];
    let mut put =
        |offset: usize, bytes: &[u8]| p[offset..offset + bytes.len()].copy_from_slice(bytes);
    put(0, &123_456_000_u32.to_le_bytes());
    put(4, &2026_u16.to_le_bytes());
    put(6, &[10, 8, 12, 34, 56, 0x07]);
    put(12, &25_u32.to_le_bytes());
    put(16, &250_000_000_i32.to_le_bytes());
    put(20, &[3, 0x01 | 0x02 | 0x80, 0, 21]);
    put(24, &(-12_312_345_i32).to_le_bytes()); // lon −1.2312345°
    put(28, &507_612_345_i32.to_le_bytes()); // lat 50.7612345°
    put(32, &98_765_i32.to_le_bytes()); // 98.765 m
    put(36, &51_234_i32.to_le_bytes());
    put(40, &14_u32.to_le_bytes()); // hAcc 14 mm
    put(44, &21_u32.to_le_bytes());
    put(48, &1500_i32.to_le_bytes());
    put(52, &(-500_i32).to_le_bytes());
    put(56, &20_i32.to_le_bytes());
    put(60, &1581_i32.to_le_bytes());
    put(64, &34_000_000_i32.to_le_bytes()); // 340°
    put(76, &123_u16.to_le_bytes());
    p
}

#[test]
fn frames_are_found_among_nmea_and_noise() {
    let mut stream = b"$GPGGA,1*00\r\n".to_vec();
    stream.extend(frame(0x05, 0x01, &[0x06, 0x8A]));
    stream.extend([0xB5, 0x00, 0x13]); // a sync byte followed by nothing UBX
    stream.extend(frame(0x01, 0x04, &[0; 18]));
    let (frames, text, errors) = feed(&stream);
    assert_eq!(frames.len(), 2);
    assert_eq!(frames[0].0, 0x05);
    assert!(text.starts_with(b"$GPGGA,1*00\r\n"));
    assert_eq!(errors.len(), 0);
}

#[test]
fn a_corrupt_or_oversized_frame_is_reported_and_the_next_one_read() {
    let mut corrupt = frame(0x01, 0x04, &[1; 18]);
    let last = corrupt.len() - 1;
    corrupt[last] ^= 0xFF;
    let mut stream = corrupt;
    stream.extend([SYNC_1, SYNC_2, 0x02, 0x15, 0x00, 0x10]); // 4096 bytes announced
    stream.extend(frame(0x05, 0x00, &[0x06, 0x8A]));
    let (frames, _, errors) = feed(&stream);
    assert_eq!(
        errors,
        vec![
            DeframeError::BadChecksum {
                class: 0x01,
                id: 0x04
            },
            DeframeError::TooLong {
                length: 4096,
                capacity: 128
            },
        ]
    );
    assert_eq!(frames.len(), 1);
    let frame = kinavis_ubx::Frame {
        class: frames[0].0,
        id: frames[0].1,
        payload: &frames[0].2,
    };
    assert_eq!(
        Message::decode(&frame).unwrap(),
        Message::Nak {
            class: 0x06,
            id: 0x8A
        }
    );
}

#[test]
fn a_pvt_becomes_a_kernel_fix() {
    let payload = pvt_payload();
    let frame = kinavis_ubx::Frame {
        class: 0x01,
        id: 0x07,
        payload: &payload,
    };
    let Message::NavPvt(pvt) = Message::decode(&frame).unwrap() else {
        panic!("NAV-PVT")
    };
    let fix = pvt.fix().unwrap();
    assert_eq!(fix.quality().fix_type(), FixType::RtkFixed);
    assert_eq!(format!("{}", fix.taken_at()), "2026-10-08T12:34:56.250 UTC");
    assert!((fix.position().latitude().degrees() - 50.761_234_5).abs() < 1e-12);
    // hAcc as the root-sum-square error: 14 mm.
    assert!((fix.horizontal_accuracy().unwrap().metres() - 0.014).abs() < 1e-12);
    assert!((pvt.velocity.climb_rate().metres_per_second() + 0.02).abs() < 1e-12);
    assert!((pvt.geodetic_point().height().value().metres() - 98.765).abs() < 1e-9);
    assert_eq!(pvt.pdop.unwrap().value(), 1.23);

    // Without a valid date the fix has no instant.
    let mut undated = payload.clone();
    undated[11] = 0x02;
    let frame = kinavis_ubx::Frame {
        class: 0x01,
        id: 0x07,
        payload: &undated,
    };
    let Message::NavPvt(pvt) = Message::decode(&frame).unwrap() else {
        panic!("NAV-PVT")
    };
    assert!(pvt.fix().is_err());
    // A short payload is a length error, not a misread.
    let frame = kinavis_ubx::Frame {
        class: 0x01,
        id: 0x07,
        payload: &payload[..80],
    };
    assert!(Message::decode(&frame).is_err());
}

#[test]
fn the_receivers_leap_second_count_serves_as_a_table() {
    let mut payload = vec![0_u8; 24];
    payload[9] = 18; // GPS − UTC
    payload[11] = 1; // next change +1
    payload[12..16].copy_from_slice(&86_400_i32.to_le_bytes());
    payload[23] = 0x03;
    let frame = kinavis_ubx::Frame {
        class: 0x01,
        id: 0x26,
        payload: &payload,
    };
    let Message::NavTimels(timels) = Message::decode(&frame).unwrap() else {
        panic!("NAV-TIMELS")
    };
    let received = Instant::<Utc>::from_civil(Civil::date(2026, 12, 31)).unwrap();
    let table = timels.leap_seconds(received).unwrap();
    assert_eq!(table.tai_minus_utc(received).unwrap().as_secs(), 37);
    let after = Instant::<Utc>::from_civil(Civil::date(2027, 1, 2)).unwrap();
    assert_eq!(table.tai_minus_utc(after).unwrap().as_secs(), 38);
    let before = Instant::<Utc>::from_civil(Civil::date(2026, 1, 1)).unwrap();
    assert!(table.tai_minus_utc(before).is_err());
}

proptest! {
    #[test]
    fn any_byte_stream_is_read_without_panic_and_frames_survive_noise(
        noise in proptest::collection::vec(any::<u8>(), 0..300),
        payload in proptest::collection::vec(any::<u8>(), 0..64),
    ) {
        let mut stream = noise.clone();
        stream.extend(frame(0x0A, 0x04, &payload));
        let (frames, _, _) = feed(&stream);
        // The frame after the noise is read, unless the noise left the
        // deframer inside a frame that swallows it.
        let last = frames.last();
        let swallowed = noise.windows(2).any(|w| w == [SYNC_1, SYNC_2]);
        prop_assert!(swallowed || last.map(|f| &f.2) == Some(&payload));
        for (class, id, payload) in &frames {
            let frame = kinavis_ubx::Frame { class: *class, id: *id, payload };
            let _ = Message::decode(&frame);
        }
    }
}
