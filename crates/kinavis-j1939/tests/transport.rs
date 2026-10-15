//! Transport protocols: broadcasts of every length, connections with flow
//! control between a sender and a receiver with a monitor listening, lost
//! packets, timeouts, refusals, and the extended protocol.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::indexing_slicing
)]

use core::time::Duration;

use kinavis_j1939::{
    broadcast, AbortReason, CanId, ExtendedReceiver, Frame, Pgn, SenderState, TransportReceiver,
    TransportSender, TP_MAX_BYTES,
};
use kinavis_kernel::{Instant, Utc};
use proptest::prelude::*;

fn at(millis: u64) -> Instant<Utc> {
    Instant::<Utc>::from_unix_seconds(1_760_000_000).saturating_add(Duration::from_millis(millis))
}

fn bytes(len: usize, seed: u8) -> Vec<u8> {
    (0..len)
        .map(|i| (i as u8).wrapping_mul(31).wrapping_add(seed))
        .collect()
}

const TASK_DATA: Pgn = Pgn::new(52_224);

#[test]
fn a_broadcast_of_every_length_arrives_whole() {
    for len in 9..=TP_MAX_BYTES {
        let data = bytes(len, 7);
        let mut monitor = TransportReceiver::<2, TP_MAX_BYTES>::new(None);
        let mut node = TransportReceiver::<2, TP_MAX_BYTES>::new(Some(0x26));
        let mut received = Vec::new();
        broadcast(Pgn::new(65_242), 0x1C, &data, |frame| {
            for receiver in [&mut monitor, &mut node] {
                let outcome = receiver.push(&frame, at(0));
                assert_eq!(outcome.reply, None, "a broadcast is never answered");
                if let Some(transfer) = outcome.transfer {
                    received.push(transfer);
                }
            }
        })
        .unwrap();
        assert_eq!(received.len(), 2, "{len} bytes");
        for transfer in &received {
            assert_eq!(transfer.data(), data.as_slice());
            assert_eq!(transfer.source, 0x1C);
            assert_eq!(transfer.destination, None);
        }
    }
    assert!(broadcast(Pgn::new(65_242), 0x1C, &[0; 8], |_| {}).is_err());
    assert!(broadcast(Pgn::new(65_242), 0x1C, &[0; 1786], |_| {}).is_err());
}

/// One connection from 0x1C to 0x26 on a bus with a monitor: frames are
/// delivered in order, the receiver's answers back to the sender. `lose`
/// drops the data packet of that sequence number once.
fn exchange(
    len: usize,
    window: u8,
    lose: Option<u8>,
) -> (SenderState, Vec<Vec<u8>>, Vec<AbortReason>) {
    let data = bytes(len, 3);
    let mut sender = TransportSender::<TP_MAX_BYTES>::new(TASK_DATA, 0x1C, 0x26, &data).unwrap();
    let mut receiver = TransportReceiver::<2, TP_MAX_BYTES>::new(Some(0x26)).with_window(window);
    let mut monitor = TransportReceiver::<2, TP_MAX_BYTES>::new(None);
    let mut lost = lose;
    let mut bus = vec![sender.start(at(0))];
    let (mut transfers, mut dropped) = (Vec::new(), Vec::new());
    let mut steps = 0;
    while let Some(frame) = bus.pop() {
        steps += 1;
        assert!(steps < 10_000, "the exchange does not end");
        let is_lost = frame.id.pgn() == Pgn::TP_DATA && frame.data().first() == lost.as_ref();
        if is_lost {
            lost = None;
            continue;
        }
        let mut next = Vec::new();
        if frame.id.source() == 0x1C {
            let outcome = receiver.push(&frame, at(0));
            transfers.extend(outcome.transfer.map(|t| t.data().to_vec()));
            dropped.extend(outcome.dropped);
            next.extend(outcome.reply);
        } else {
            sender.handle(&frame, at(0), |packet| next.push(packet));
        }
        let heard = monitor.push(&frame, at(0));
        assert_eq!(heard.reply, None, "a monitor never answers");
        transfers.extend(heard.transfer.map(|t| t.data().to_vec()));
        // Deliver in order: the stack is reversed.
        next.reverse();
        bus.extend(next);
    }
    (sender.state(), transfers, dropped)
}

proptest! {
    #[test]
    fn a_connection_arrives_whole_at_the_receiver_and_the_monitor(
        len in 9usize..=TP_MAX_BYTES, window in 1u8..=20,
    ) {
        let (state, transfers, dropped) = exchange(len, window, None);
        prop_assert_eq!(state, SenderState::Done);
        prop_assert!(dropped.is_empty());
        prop_assert_eq!(transfers.len(), 2);
        for transfer in &transfers {
            prop_assert_eq!(transfer, &bytes(len, 3));
        }
    }
}

#[test]
fn a_lost_packet_aborts_the_connection_on_both_sides() {
    let (state, transfers, dropped) = exchange(100, 4, Some(3));
    assert_eq!(state, SenderState::Aborted(AbortReason::BadSequence));
    assert_eq!(transfers, Vec::<Vec<u8>>::new());
    assert_eq!(dropped, vec![AbortReason::BadSequence]);
}

#[test]
fn silence_is_a_timeout_on_both_sides() {
    let mut sender = TransportSender::<64>::new(TASK_DATA, 0x1C, 0x26, &bytes(40, 1)).unwrap();
    let mut receiver = TransportReceiver::<1, 64>::new(Some(0x26));
    let rts = sender.start(at(0));
    let outcome = receiver.push(&rts, at(0));
    assert!(outcome
        .reply
        .is_some_and(|cts| cts.data().first() == Some(&17)));
    // The clear-to-send is lost; the sender gives up after T3, the receiver
    // after T2.
    let mut sent = Vec::new();
    sender.poll(at(1_200), |frame| sent.push(frame));
    assert_eq!(sent, Vec::new());
    sender.poll(at(1_300), |frame| sent.push(frame));
    assert_eq!(sender.state(), SenderState::Aborted(AbortReason::Timeout));
    assert_eq!(sent.first().and_then(|f| f.data().get(1).copied()), Some(3));
    let mut owed = Vec::new();
    assert_eq!(receiver.expire(at(1_200), |frame| owed.push(frame)), 0);
    assert_eq!(receiver.expire(at(1_300), |frame| owed.push(frame)), 1);
    assert_eq!(
        owed.first().map(|f| (f.id.source(), f.id.destination())),
        Some((0x26, Some(0x1C)))
    );
    assert_eq!(receiver.pending(), 0);
}

#[test]
fn a_transfer_too_large_for_the_receiver_is_refused() {
    let mut sender = TransportSender::<512>::new(TASK_DATA, 0x1C, 0x26, &bytes(300, 1)).unwrap();
    let mut receiver = TransportReceiver::<1, 256>::new(Some(0x26));
    let outcome = receiver.push(&sender.start(at(0)), at(0));
    let reply = outcome.reply.expect("an abort");
    assert_eq!(reply.data().first(), Some(&255));
    assert_eq!(reply.data().get(1), Some(&2));
    sender.handle(&reply, at(0), |_| {});
    assert_eq!(
        sender.state(),
        SenderState::Aborted(AbortReason::ResourcesNeeded)
    );
}

/// ETP sender played by the test: answers each clear-to-send with an offset
/// message and the packets asked for.
fn etp_frames(data: &[u8], cts: &Frame) -> Vec<Frame> {
    let bytes = cts.data();
    let count = bytes[1];
    let next = u32::from_le_bytes([bytes[2], bytes[3], bytes[4], 0]);
    let pgn = TASK_DATA.to_bytes();
    let offset = (next - 1).to_le_bytes();
    let mut frames = vec![Frame::of(
        7,
        Pgn::ETP_CONNECTION,
        0x26,
        0x1C,
        &[
            22, count, offset[0], offset[1], offset[2], pgn[0], pgn[1], pgn[2],
        ],
    )
    .unwrap()];
    for sequence in 1..=count {
        let packet = (next - 1 + u32::from(sequence)) as usize;
        let mut chunk = [0xFF; 8];
        chunk[0] = sequence;
        for (slot, byte) in chunk[1..]
            .iter_mut()
            .zip(data.iter().skip((packet - 1) * 7))
        {
            *slot = *byte;
        }
        frames.push(Frame::of(7, Pgn::ETP_DATA, 0x26, 0x1C, &chunk).unwrap());
    }
    frames
}

#[test]
fn an_extended_transfer_arrives_whole_window_after_window() {
    for (len, window) in [(1_786, 255), (5_000, 255), (20_000, 100)] {
        let data = bytes(len, 9);
        let mut receiver = ExtendedReceiver::<1, 20_000>::new(Some(0x26)).with_window(window);
        let mut monitor = ExtendedReceiver::<1, 20_000>::new(None);
        let size = (len as u32).to_le_bytes();
        let pgn = TASK_DATA.to_bytes();
        let rts = Frame::of(
            7,
            Pgn::ETP_CONNECTION,
            0x26,
            0x1C,
            &[
                20, size[0], size[1], size[2], size[3], pgn[0], pgn[1], pgn[2],
            ],
        )
        .unwrap();
        let _ = monitor.push(&rts, at(0));
        let mut reply = receiver.push(&rts, at(0)).reply.expect("a clear-to-send");
        let mut transfers = Vec::new();
        let mut acknowledged = false;
        while !acknowledged {
            let mut next = None;
            for frame in etp_frames(&data, &reply) {
                let outcome = receiver.push(&frame, at(0));
                transfers.extend(outcome.transfer.map(|t| t.data().to_vec()));
                let heard = monitor.push(&frame, at(0));
                transfers.extend(heard.transfer.map(|t| t.data().to_vec()));
                next = next.or(outcome.reply);
            }
            let answer = next.expect("an answer to every window");
            acknowledged = answer.data().first() == Some(&23);
            reply = answer;
        }
        assert_eq!(transfers.len(), 2, "{len} bytes");
        for transfer in &transfers {
            assert_eq!(transfer, &data);
        }
        assert_eq!(&reply.data()[1..5], &size);
    }
}

#[test]
fn an_extended_transfer_too_small_or_too_large_is_refused() {
    let mut receiver = ExtendedReceiver::<1, 4_096>::new(Some(0x26));
    let pgn = TASK_DATA.to_bytes();
    for size in [1_785u32, 4_097] {
        let s = size.to_le_bytes();
        let rts = Frame::of(
            7,
            Pgn::ETP_CONNECTION,
            0x26,
            0x1C,
            &[20, s[0], s[1], s[2], s[3], pgn[0], pgn[1], pgn[2]],
        )
        .unwrap();
        let outcome = receiver.push(&rts, at(0));
        assert!(outcome
            .reply
            .is_some_and(|f| f.data().first() == Some(&255)));
        assert_eq!(receiver.pending(), 0);
    }
    assert_eq!(CanId::new(0x1CC8_261C).unwrap().pgn(), Pgn::ETP_CONNECTION);
}
