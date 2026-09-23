//! Fast-packet reassembly: ordering, interleaving, timeouts, slots.

#![allow(clippy::unwrap_used, clippy::panic, clippy::indexing_slicing)]

use core::time::Duration;

use kinavis_kernel::time::{Instant, Utc};
use kinavis_nmea2000::{
    Assembler, CanId, Frame, Nmea2000Error, Pgn, Transport, DEFAULT_TIMEOUT, MAX_ASSEMBLIES,
    MAX_PAYLOAD_BYTES,
};

fn at(seconds: i64) -> Instant<Utc> {
    Instant::from_unix_seconds(1_789_000_000 + seconds)
}

fn frame(pgn: Pgn, source: u8, data: &[u8]) -> Frame {
    Frame::new(CanId::from_parts(2, pgn, 255, source), data).unwrap()
}

/// Frames of a fast packet of `len` bytes with the given sequence counter,
/// payload counting up from 1.
fn packet(sequence: u8, len: u8) -> Vec<Vec<u8>> {
    let bytes: Vec<u8> = (1..=len).collect();
    let mut frames = vec![];
    let mut first = vec![sequence << 5, len];
    first.extend(bytes.iter().take(6));
    frames.push(first);
    for (number, chunk) in bytes
        .iter()
        .skip(6)
        .collect::<Vec<_>>()
        .chunks(7)
        .enumerate()
    {
        // At most 32 frames.
        #[allow(clippy::cast_possible_truncation)]
        let mut frame = vec![sequence << 5 | (number as u8 + 1)];
        frame.extend(chunk.iter().copied());
        frames.push(frame);
    }
    frames
}

#[test]
fn a_fast_packet_is_the_declared_bytes_and_no_more() {
    let mut assembler = Assembler::new();
    let pgn = Pgn::GNSS_POSITION_DATA;
    let frames = packet(3, 43);
    assert_eq!(frames.len(), 7);
    for data in frames.iter().take(6) {
        assert_eq!(assembler.push(&frame(pgn, 1, data), at(0)), Ok(None));
    }
    assert_eq!(assembler.pending(), 1);
    let payload = assembler
        .push(&frame(pgn, 1, &frames[6]), at(0))
        .unwrap()
        .unwrap();
    assert_eq!(assembler.pending(), 0);
    assert_eq!(payload.len(), 43);
    assert_eq!(payload.pgn(), pgn);
    assert_eq!(payload.source(), 1);
    let expected: Vec<u8> = (1..=43).collect();
    assert_eq!(payload.bytes(), expected.as_slice());
}

#[test]
fn a_single_frame_group_comes_straight_back() {
    let mut assembler = Assembler::new();
    let payload = assembler
        .push(
            &frame(Pgn::WATER_DEPTH, 9, &[1, 2, 3, 4, 5, 6, 7, 8]),
            at(0),
        )
        .unwrap()
        .unwrap();
    assert_eq!(payload.bytes(), &[1, 2, 3, 4, 5, 6, 7, 8]);
    assert_eq!(assembler.pending(), 0);
}

#[test]
fn an_unknown_group_needs_to_be_told_its_transport() {
    let mut assembler = Assembler::new();
    let wind = Pgn::new(130_306);
    assert_eq!(
        assembler.push(&frame(wind, 9, &[0; 8]), at(0)),
        Err(Nmea2000Error::UnknownPgn { pgn: wind })
    );
    let payload = assembler
        .push_as(&frame(wind, 9, &[0; 8]), Transport::SingleFrame, at(0))
        .unwrap()
        .unwrap();
    assert_eq!(payload.len(), 8);
    // Fast packet with a caller-supplied transport.
    let frames = packet(0, 9);
    assert_eq!(
        assembler.push_as(&frame(wind, 9, &frames[0]), Transport::FastPacket, at(0)),
        Ok(None)
    );
    let payload = assembler
        .push_as(&frame(wind, 9, &frames[1]), Transport::FastPacket, at(0))
        .unwrap()
        .unwrap();
    assert_eq!(payload.bytes(), &[1, 2, 3, 4, 5, 6, 7, 8, 9]);
}

#[test]
fn packets_from_different_sources_interleave() {
    let mut assembler = Assembler::new();
    let pgn = Pgn::AIS_CLASS_B_POSITION_REPORT;
    let this = packet(1, 26);
    let other = packet(6, 20);
    assert_eq!(assembler.push(&frame(pgn, 1, &this[0]), at(0)), Ok(None));
    assert_eq!(assembler.push(&frame(pgn, 2, &other[0]), at(0)), Ok(None));
    assert_eq!(assembler.pending(), 2);
    assert_eq!(assembler.push(&frame(pgn, 1, &this[1]), at(0)), Ok(None));
    assert_eq!(assembler.push(&frame(pgn, 2, &other[1]), at(0)), Ok(None));
    let done = assembler
        .push(&frame(pgn, 2, &other[2]), at(0))
        .unwrap()
        .unwrap();
    assert_eq!(done.len(), 20);
    assert_eq!(done.source(), 2);
    assert_eq!(assembler.push(&frame(pgn, 1, &this[2]), at(0)), Ok(None));
    let done = assembler
        .push(&frame(pgn, 1, &this[3]), at(0))
        .unwrap()
        .unwrap();
    assert_eq!(done.len(), 26);
    assert_eq!(assembler.pending(), 0);
}

#[test]
fn a_frame_out_of_order_or_of_another_sequence_is_refused() {
    let mut assembler = Assembler::new();
    let pgn = Pgn::GNSS_POSITION_DATA;
    let frames = packet(3, 43);
    assert_eq!(assembler.push(&frame(pgn, 1, &frames[0]), at(0)), Ok(None));
    assert_eq!(
        assembler.push(&frame(pgn, 1, &frames[2]), at(0)),
        Err(Nmea2000Error::UnexpectedFrame {
            expected: 1,
            found: 2
        })
    );
    assert_eq!(assembler.pending(), 0, "the assembly is given up");

    // Wrong sequence counter on the next frame.
    assert_eq!(assembler.push(&frame(pgn, 1, &frames[0]), at(0)), Ok(None));
    let mut other = frames[1].clone();
    other[0] = 4 << 5 | 1;
    assert_eq!(
        assembler.push(&frame(pgn, 1, &other), at(0)),
        Err(Nmea2000Error::UnexpectedFrame {
            expected: 1,
            found: 1
        })
    );

    // Continuation without a first frame.
    assert_eq!(
        assembler.push(&frame(pgn, 1, &frames[1]), at(0)),
        Err(Nmea2000Error::UnexpectedFrame {
            expected: 0,
            found: 1
        })
    );

    // A first frame replaces an incomplete packet from the same source.
    assert_eq!(assembler.push(&frame(pgn, 1, &frames[0]), at(0)), Ok(None));
    assert_eq!(assembler.push(&frame(pgn, 1, &frames[0]), at(0)), Ok(None));
    assert_eq!(assembler.pending(), 1);
}

#[test]
fn an_unfinished_packet_is_given_up_after_the_timeout() {
    let mut assembler = Assembler::with_timeout(Duration::from_millis(500));
    assert_eq!(assembler.timeout(), Duration::from_millis(500));
    let pgn = Pgn::GNSS_POSITION_DATA;
    let frames = packet(3, 43);
    assert_eq!(assembler.push(&frame(pgn, 1, &frames[0]), at(0)), Ok(None));
    assert_eq!(assembler.pending(), 1);
    assembler.expire(at(0));
    assert_eq!(assembler.pending(), 1, "not yet");
    assembler.expire(at(1));
    assert_eq!(assembler.pending(), 0);

    // A clock step backwards also drops the packet.
    assert_eq!(assembler.push(&frame(pgn, 1, &frames[0]), at(10)), Ok(None));
    assert_eq!(
        assembler.push(&frame(pgn, 1, &frames[1]), at(9)),
        Err(Nmea2000Error::UnexpectedFrame {
            expected: 0,
            found: 1
        })
    );
    assert_eq!(
        Assembler::default().timeout(),
        kinavis_nmea2000::DEFAULT_TIMEOUT
    );
}

#[test]
fn every_slot_busy_gives_the_oldest_packet_up_for_a_new_one() {
    // Long timeout, so only slots are exhausted.
    let mut assembler = Assembler::with_timeout(Duration::from_secs(60));
    let pgn = Pgn::GNSS_POSITION_DATA;
    let frames = packet(0, 43);
    // Eight sources each start a packet, 1 s apart; source 0 is the oldest.
    #[allow(clippy::cast_possible_truncation)]
    for source in 0..MAX_ASSEMBLIES as u8 {
        assert_eq!(
            assembler.push(&frame(pgn, source, &frames[0]), at(i64::from(source))),
            Ok(None)
        );
    }
    assert_eq!(assembler.pending(), MAX_ASSEMBLIES);
    // A ninth evicts source 0; its next frame has no assembly.
    assert_eq!(
        assembler.push(&frame(pgn, 200, &frames[0]), at(8)),
        Ok(None)
    );
    assert_eq!(assembler.pending(), MAX_ASSEMBLIES);
    assert!(matches!(
        assembler.push(&frame(pgn, 0, &frames[1]), at(8)),
        Err(Nmea2000Error::UnexpectedFrame { .. })
    ));
    // A packet completed by its first frame needs no slot.
    let short = packet(0, 6);
    assert!(assembler
        .push(&frame(pgn, 201, &short[0]), at(8))
        .unwrap()
        .is_some());
    // The newcomer completes and frees its slot.
    for data in frames.iter().skip(1) {
        let _ = assembler.push(&frame(pgn, 200, data), at(8)).unwrap();
    }
    assert_eq!(assembler.pending(), MAX_ASSEMBLIES - 1);
    // More slots hold more assemblies.
    let mut wide = Assembler::<16>::with_slots(DEFAULT_TIMEOUT);
    for source in 0..16 {
        assert_eq!(wide.push(&frame(pgn, source, &frames[0]), at(0)), Ok(None));
    }
    assert_eq!(wide.pending(), 16);
}

#[test]
fn the_longest_packet_fits_and_a_longer_one_is_refused() {
    let mut assembler = Assembler::new();
    let pgn = Pgn::GNSS_POSITION_DATA;
    // 223 bytes: 32 frames.
    #[allow(clippy::cast_possible_truncation)]
    let frames = packet(7, MAX_PAYLOAD_BYTES as u8);
    assert_eq!(frames.len(), 32);
    let mut whole = None;
    for data in &frames {
        whole = assembler.push(&frame(pgn, 1, data), at(0)).unwrap();
    }
    assert_eq!(whole.unwrap().len(), MAX_PAYLOAD_BYTES);

    assert_eq!(
        assembler.push(&frame(pgn, 1, &[7 << 5, 224, 1, 2, 3, 4, 5, 6]), at(0)),
        Err(Nmea2000Error::BadLength {
            declared: 224,
            limit: MAX_PAYLOAD_BYTES
        })
    );
    // Empty frame is rejected.
    assert!(assembler.push(&frame(pgn, 1, &[]), at(0)).is_err());
}
