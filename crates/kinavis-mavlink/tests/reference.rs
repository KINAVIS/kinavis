//! Frames encoded by pymavlink (`tools/mavlink_reference.py`, values only):
//! each decodes here to the same fields and encodes back to the same bytes;
//! signed frames verify with the key and are signed again alike.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::indexing_slicing
)]

use kinavis_mavlink::{Deframer, Frame, Message, Version, MAX_FRAME};

const REFERENCE: &str = include_str!("data/mavlink_reference.txt");

fn bytes(hex: &str) -> Vec<u8> {
    (0..hex.len() / 2)
        .map(|i| u8::from_str_radix(&hex[2 * i..2 * i + 2], 16).unwrap())
        .collect()
}

fn deframe(bytes: &[u8]) -> Frame {
    let mut deframer = Deframer::new();
    let mut frames = bytes.iter().filter_map(|&byte| deframer.push(byte));
    let frame = frames.next().expect("a frame").expect("a valid frame");
    assert!(frames.next().is_none());
    frame
}

/// The struct of a message, as Debug prints it.
fn debug(message: &Message) -> String {
    let text = format!("{message:?}");
    // `GpsRawInt(GpsRawInt { … })` → `GpsRawInt { … }`
    let open = text.find('(').unwrap();
    text[open + 1..text.len() - 1].to_owned()
}

#[test]
fn every_frame_of_pymavlink_decodes_to_its_fields_and_back_to_its_bytes() {
    let mut checked = 0;
    for line in REFERENCE
        .lines()
        .filter(|line| line.starts_with("unsigned "))
    {
        let (_, rest) = line.split_once(' ').unwrap();
        let (hex, expected) = rest.split_once(' ').unwrap();
        let sent = bytes(hex);
        let frame = deframe(&sent);
        assert_eq!(frame.version, Version::V2);
        let message = frame.message().expect("a known message");
        assert_eq!(debug(&message), expected, "{hex}");
        let again = Frame::new(frame.system, frame.component, frame.sequence, &message);
        let mut out = [0u8; MAX_FRAME];
        let len = again.encode(&mut out);
        assert_eq!(&out[..len], sent.as_slice(), "{expected}");
        checked += 1;
    }
    assert_eq!(checked, 320);
}

#[test]
fn mavlink_one_frames_decode_and_encode_back() {
    let mut checked = 0;
    for line in REFERENCE.lines().filter(|line| line.starts_with("v1 ")) {
        let (_, rest) = line.split_once(' ').unwrap();
        let (hex, expected) = rest.split_once(' ').unwrap();
        let sent = bytes(hex);
        let frame = deframe(&sent);
        assert_eq!(frame.version, Version::V1);
        assert_eq!(debug(&frame.message().unwrap()), expected);
        let mut out = [0u8; MAX_FRAME];
        let len = frame.encode(&mut out);
        assert_eq!(&out[..len], sent.as_slice());
        checked += 1;
    }
    assert_eq!(checked, 10);
}

#[test]
fn a_corrupt_byte_is_a_bad_checksum_and_the_next_frame_still_comes() {
    let line = REFERENCE
        .lines()
        .find(|line| line.starts_with("unsigned fd"))
        .unwrap();
    let good = bytes(line.split(' ').nth(1).unwrap());
    let mut bad = good.clone();
    bad[12] ^= 0x01;
    let mut deframer = Deframer::new();
    let outcomes: Vec<_> = b"\x00junk"
        .iter()
        .chain(&bad)
        .chain(&good)
        .filter_map(|&byte| deframer.push(byte))
        .collect();
    assert!(matches!(
        outcomes.first(),
        Some(Err(kinavis_mavlink::MavlinkError::BadChecksum { .. }))
    ));
    assert!(outcomes.last().is_some_and(Result::is_ok));
}

#[cfg(feature = "signing")]
#[test]
fn signed_frames_verify_and_are_signed_again_alike() {
    use kinavis_mavlink::signing::{sign, verify};
    let key: [u8; 32] = core::array::from_fn(|i| u8::try_from(i).unwrap());
    let mut checked = 0;
    for line in REFERENCE.lines().filter(|line| line.starts_with("signed ")) {
        let fields: Vec<&str> = line.split(' ').collect();
        let sent = bytes(fields[1]);
        let frame = deframe(&sent);
        let signature = frame.signature.expect("signed");
        assert_eq!(signature.link, fields[2].parse::<u8>().unwrap());
        assert_eq!(signature.timestamp, fields[3].parse::<u64>().unwrap());
        assert!(verify(&frame, &key));
        let mut wrong = key;
        wrong[0] ^= 1;
        assert!(!verify(&frame, &wrong));
        let mut again = Frame::new(
            frame.system,
            frame.component,
            frame.sequence,
            &frame.message().unwrap(),
        );
        sign(&mut again, &key, signature.link, signature.timestamp);
        let mut out = [0u8; MAX_FRAME];
        let len = again.encode(&mut out);
        assert_eq!(&out[..len], sent.as_slice());
        checked += 1;
    }
    assert_eq!(checked, 20);
}
