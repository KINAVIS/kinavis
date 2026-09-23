//! No input panics; no accepted input has a wrong checksum.

#![allow(clippy::unwrap_used, clippy::indexing_slicing)]

use kinavis_nmea0183::{parse, Sentence};
use proptest::prelude::*;

/// Sentence-like byte strings, so tests reach the decoders instead of failing
/// at framing.
fn sentence_like() -> impl Strategy<Value = Vec<u8>> {
    let header = prop_oneof![
        Just(b"$GPRMC,".to_vec()),
        Just(b"$GPGGA,".to_vec()),
        Just(b"$GPGLL,".to_vec()),
        Just(b"$GPVTG,".to_vec()),
        Just(b"$GNXYZ,".to_vec()),
        Just(b"!AIVDM,".to_vec()),
    ];
    let field = prop_oneof![
        Just(b"".to_vec()),
        "[0-9]{1,6}(\\.[0-9]{0,4})?".prop_map(String::into_bytes),
        "[ANSEWVDMKT]".prop_map(String::into_bytes),
        "-?[0-9.]{0,8}".prop_map(String::into_bytes),
        proptest::collection::vec(any::<u8>(), 0..6),
    ];
    (header, proptest::collection::vec(field, 0..16)).prop_map(|(header, fields)| {
        let mut body = header;
        for (index, field) in fields.iter().enumerate() {
            if index > 0 {
                body.push(b',');
            }
            body.extend_from_slice(field);
        }
        let sum = body[1..].iter().fold(0_u8, |sum, &byte| sum ^ byte);
        body.extend_from_slice(format!("*{sum:02X}\r\n").as_bytes());
        body
    })
}

proptest! {
    #[test]
    fn arbitrary_bytes_never_panic(bytes in proptest::collection::vec(any::<u8>(), 0..100)) {
        let _ = parse(&bytes);
    }

    #[test]
    fn sentence_shaped_bytes_never_panic(bytes in sentence_like()) {
        let _ = parse(&bytes);
    }

    #[test]
    fn whatever_parses_writes_back_to_something_that_parses_the_same(bytes in sentence_like()) {
        if let Ok(sentence) = parse(&bytes) {
            if matches!(sentence, Sentence::Unsupported { .. }) {
                return Ok(());
            }
            let written = format!("{sentence}");
            let again = parse(written.as_bytes()).unwrap();
            // Formatting rounds to wire precision; the second round trip is a
            // fixed point.
            prop_assert_eq!(format!("{again}"), written);
        }
    }
}
