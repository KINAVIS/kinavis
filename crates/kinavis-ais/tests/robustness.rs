//! No payload, whole or fragmented, in any order, panics.

#![allow(clippy::unwrap_used, clippy::indexing_slicing)]

use kinavis_ais::{Assembler, Bits, Message, MAX_MESSAGE_BITS};
use kinavis_kernel::{Instant, Utc};
use kinavis_nmea0183::{parse, Sentence};
use proptest::prelude::*;

fn payload() -> impl Strategy<Value = Vec<u8>> {
    prop::collection::vec(any::<u8>(), 0..=70)
}

fn armoured() -> impl Strategy<Value = Vec<u8>> {
    prop::collection::vec(
        prop_oneof![(b'0'..=b'W').prop_map(|b| b), (b'`'..=b'w').prop_map(|b| b)],
        // Long enough for a type 5 (424 bits).
        0..=72,
    )
}

proptest! {
    #[test]
    fn any_bytes_unarmour_or_are_refused(payload in payload(), fill in 0u8..=7) {
        if let Ok(bits) = Bits::unarmour(&payload, fill) {
            prop_assert!(bits.len() <= MAX_MESSAGE_BITS);
            let _ = Message::decode(&bits);
            for offset in 0..bits.len() + 2 {
                let _ = bits.unsigned(offset, 30);
                let _ = bits.signed(offset, 28);
                let _ = bits.bit(offset);
            }
        }
    }

    #[test]
    fn any_armoured_message_decodes_or_is_refused(payload in armoured(), fill in 0u8..=5) {
        let bits = Bits::unarmour(&payload, fill);
        if payload.is_empty() && fill > 0 {
            prop_assert!(bits.is_err());
        } else {
            let _ = Message::decode(&bits.unwrap());
        }
    }

    #[test]
    fn any_stream_of_fragments_is_taken_without_panic(
        fragments in prop::collection::vec(
            (1u8..=9, 1u8..=9, 0u8..=9, 0u8..=1, armoured(), 0u8..=5, 0i64..=30),
            0..=40,
        )
    ) {
        let mut assembler = Assembler::new();
        for (count, number, sequence, channel, payload, fill, second) in fragments {
            let (count, number) = (count.max(number), number);
            let channel = if channel == 0 { "A" } else { "B" };
            let payload = core::str::from_utf8(&payload).unwrap();
            let body = format!("AIVDM,{count},{number},{sequence},{channel},{payload},{fill}");
            let checksum = body.bytes().fold(0, |sum, byte| sum ^ byte);
            let line = format!("!{body}*{checksum:02X}");
            let Ok(Sentence::Vdm(vdm)) = parse(line.as_bytes()) else { continue };
            let now = Instant::<Utc>::from_unix_seconds(1_789_000_000 + second);
            if let Ok(Some(bits)) = assembler.push(&vdm, now) {
                let _ = Message::decode(&bits);
            }
        }
    }
}
