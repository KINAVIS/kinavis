//! Fuzz target: CAN frames to messages — identifier, single-frame and
//! fast-packet reassembly (slots, timeouts), decoder.
//!
//! Input: 13-byte records (raw identifier, length, 8 data bytes), fed to one
//! assembler with a clock advancing per frame. The identifier's top bits (not
//! on the bus) select the transport for unknown PGNs. Invariants: no panic;
//! pending groups never exceed the slot count.
#![no_main]

use core::time::Duration;

use libfuzzer_sys::fuzz_target;

use kinavis_kernel::time::{Instant, Utc};
use kinavis_nmea2000::{Assembler, CanId, Frame, Message, Transport, MAX_ASSEMBLIES};

fuzz_target!(|data: &[u8]| {
    let mut assembler = Assembler::new();
    let mut now = Instant::<Utc>::from_unix_seconds(1_789_000_000);
    for record in data.chunks(13) {
        let [i0, i1, i2, i3, len, bytes @ ..] = record else {
            return;
        };
        let raw = u32::from_le_bytes([*i0, *i1, *i2, *i3]);
        let Ok(id) = CanId::new(raw & 0x1FFF_FFFF) else {
            continue;
        };
        let Ok(frame) = Frame::new(
            id,
            bytes
                .get(..usize::from(*len % 9).min(bytes.len()))
                .unwrap_or(&[]),
        ) else {
            continue;
        };
        // Mostly forward, occasionally backward: the assembler must tolerate
        // clock steps.
        now = if raw & 0x8000_0000 == 0 {
            now.saturating_add(Duration::from_millis(u64::from(*len) * 7))
        } else {
            now.saturating_sub(Duration::from_millis(u64::from(*len) * 7))
        };
        let pushed = match id.pgn().transport() {
            Some(_) => assembler.push(&frame, now),
            None if raw & 0x4000_0000 == 0 => {
                assembler.push_as(&frame, Transport::SingleFrame, now)
            }
            None => assembler.push_as(&frame, Transport::FastPacket, now),
        };
        if let Ok(Some(payload)) = pushed {
            let _ = Message::decode(&payload);
        }
        assert!(assembler.pending() <= MAX_ASSEMBLIES);
    }
});
