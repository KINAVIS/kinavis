//! No frame sequence, single or fast packet, in any order, panics.

#![allow(clippy::unwrap_used, clippy::indexing_slicing)]

use kinavis_kernel::{Instant, Utc};
use kinavis_nmea2000::{Assembler, CanId, Frame, Message, Payload, Pgn, Transport};
use proptest::prelude::*;

fn known_pgn() -> impl Strategy<Value = Pgn> {
    prop_oneof![
        Just(Pgn::POSITION_RAPID_UPDATE),
        Just(Pgn::COG_SOG_RAPID_UPDATE),
        Just(Pgn::GNSS_POSITION_DATA),
        Just(Pgn::VESSEL_HEADING),
        Just(Pgn::WATER_DEPTH),
        Just(Pgn::AIS_CLASS_A_POSITION_REPORT),
        Just(Pgn::AIS_CLASS_B_POSITION_REPORT),
        (0_u32..0x3_FFFF).prop_map(Pgn::new),
    ]
}

proptest! {
    #[test]
    fn any_bytes_decode_or_are_refused(
        pgn in known_pgn(),
        bytes in prop::collection::vec(any::<u8>(), 0..=223),
    ) {
        let payload = Payload::from_bytes(pgn, 1, &bytes).unwrap();
        let _ = Message::decode(&payload);
        for offset in 0..bytes.len() + 2 {
            let _ = payload.unsigned(offset, 8);
            let _ = payload.signed(offset, 8);
            let _ = payload.bits(offset * 8 + 3, 19);
        }
    }

    #[test]
    fn any_identifier_reads(raw in 0_u32..(1 << 29)) {
        let id = CanId::new(raw).unwrap();
        let _ = (id.pgn(), id.source(), id.destination(), id.priority());
        prop_assert_eq!(CanId::from_parts(id.priority(), id.pgn(), id.destination().unwrap_or(0), id.source()), id);
    }

    #[test]
    fn any_stream_of_frames_is_taken_without_panic(
        frames in prop::collection::vec(
            (known_pgn(), 0_u8..=3, prop::collection::vec(any::<u8>(), 0..=8), 0_i64..=3, any::<bool>()),
            0..=60,
        )
    ) {
        let mut assembler = Assembler::new();
        for (pgn, source, data, second, fast) in frames {
            let frame = Frame::new(CanId::from_parts(2, pgn, 255, source), &data).unwrap();
            let now = Instant::<Utc>::from_unix_seconds(1_789_000_000 + second);
            let transport = if fast { Transport::FastPacket } else { Transport::SingleFrame };
            if let Ok(Some(payload)) = assembler.push_as(&frame, transport, now) {
                let _ = Message::decode(&payload);
            }
            let _ = assembler.push(&frame, now);
        }
    }
}
