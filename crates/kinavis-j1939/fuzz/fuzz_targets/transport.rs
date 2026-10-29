//! Fuzz target: CAN frames to the transport receivers (TP and ETP, as a
//! participant and as a monitor), the ETP sender and the address claimer.
//!
//! Input: 13-byte records (raw identifier, length, 8 data bytes) with a
//! clock moving per frame. Invariants: no panic; sessions never exceed the
//! slots; a monitor never answers; a completed transfer is no longer than
//! its receiver holds; the sender answers a frame with at most an offset
//! message and 255 packets, all its own, a data packet carrying its bytes.
#![no_main]

use core::time::Duration;

use libfuzzer_sys::fuzz_target;

use kinavis_j1939::{
    AddressClaimer, CanId, ExtendedReceiver, ExtendedSender, Frame, Name, Pgn,
    TransportReceiver,
};
use kinavis_kernel::time::{Instant, Utc};

fuzz_target!(|data: &[u8]| {
    let mut participant = TransportReceiver::<3, 256>::new(Some(0x26)).with_window(3);
    let mut monitor = TransportReceiver::<3, 1785>::new(None);
    let mut extended = ExtendedReceiver::<2, 4096>::new(Some(0x26)).with_window(4);
    let mut extended_monitor = ExtendedReceiver::<2, 4096>::new(None);
    let mut claimer = AddressClaimer::new(Name::from_raw(0xA002_3400_0430_744D), 0x80).unwrap_or_else(|_| unreachable!());
    let mut now = Instant::<Utc>::from_unix_seconds(1_760_000_000);
    let _ = claimer.start(now);
    let payload: [u8; 2_000] = core::array::from_fn(|i| (i % 251) as u8);
    let mut sender = ExtendedSender::new(Pgn::new(52_224), 0x26, 0x1C, &payload).unwrap_or_else(|_| unreachable!());
    let _ = sender.start(now);
    for record in data.chunks(13) {
        let [i0, i1, i2, i3, len, bytes @ ..] = record else {
            return;
        };
        let raw = u32::from_le_bytes([*i0, *i1, *i2, *i3]);
        let Ok(id) = CanId::new(raw & 0x1FFF_FFFF) else {
            continue;
        };
        let Ok(frame) = Frame::new(id, bytes.get(..usize::from(*len % 9).min(bytes.len())).unwrap_or(&[])) else {
            continue;
        };
        now = now.saturating_add(Duration::from_millis(u64::from(*len) * 41));
        let heard = participant.push(&frame, now);
        if let Some(transfer) = heard.transfer {
            assert!(transfer.data().len() <= 256);
        }
        assert!(monitor.push(&frame, now).reply.is_none());
        let _ = extended.push(&frame, now);
        assert!(extended_monitor.push(&frame, now).reply.is_none());
        let mut sent = 0usize;
        sender.handle(&frame, now, |out| {
            sent += 1;
            assert_eq!((out.id.source(), out.id.destination()), (0x26, Some(0x1C)));
            if out.id.pgn() == Pgn::ETP_DATA {
                assert!(out.data().first().is_some_and(|&s| s >= 1));
                assert!(out.data().iter().skip(1).all(|&b| b < 251 || b == 0xFF));
            }
        });
        assert!(sent <= 256);
        sender.poll(now, |_| {});
        let _ = claimer.handle(&frame, now);
        let _ = claimer.poll(now);
        let _ = participant.expire(now, |_| {});
        let _ = extended.expire(now, |_| {});
        assert!(participant.pending() <= 3 && monitor.pending() <= 3);
    }
});
