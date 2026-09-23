//! Fuzz target: raw lines to messages — sentence framing, fragment reassembly
//! (slots, timeouts), decoder.
//!
//! Each line is parsed as a sentence; `!AIVDM`/`!AIVDO` go through one
//! assembler with a clock advancing per line, exercising sequences, channels,
//! out-of-order fragments and expiry. Invariants: no panic; pending messages
//! never exceed the slot count.
#![no_main]

use core::time::Duration;

use libfuzzer_sys::fuzz_target;

use kinavis_ais::{Assembler, Message, MAX_ASSEMBLIES};
use kinavis_kernel::time::{Instant, Utc};
use kinavis_nmea0183::{parse, Sentence};

fuzz_target!(|data: &[u8]| {
    let mut assembler = Assembler::with_timeout(Duration::from_millis(500));
    let mut now = Instant::<Utc>::from_unix_seconds(1_789_000_000);
    for line in data.split(|&byte| byte == b'\n') {
        // The step is derived from the line, so the clock occasionally goes
        // backwards.
        let step = i64::from(line.first().copied().unwrap_or(0)) - 64;
        now = if step >= 0 {
            now.saturating_add(Duration::from_millis(step as u64))
        } else {
            now.saturating_sub(Duration::from_millis(step.unsigned_abs()))
        };
        let Ok(Sentence::Vdm(vdm)) = parse(line) else {
            continue;
        };
        if let Ok(Some(bits)) = assembler.push(&vdm, now) {
            let _ = Message::decode(&bits);
        }
        assert!(assembler.pending() <= MAX_ASSEMBLIES);
    }
});
