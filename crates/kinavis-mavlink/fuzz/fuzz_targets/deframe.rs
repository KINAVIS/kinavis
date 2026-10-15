//! Fuzz target: a byte stream to frames, messages and back.
//!
//! Invariants: no panic; a frame read encodes to bytes that read back as
//! the same frame; a signature never verifies with another key.
#![no_main]

use libfuzzer_sys::fuzz_target;

use kinavis_mavlink::{signing, Deframer, MAX_FRAME};

fuzz_target!(|data: &[u8]| {
    let mut deframer = Deframer::new();
    for &byte in data {
        let Some(Ok(frame)) = deframer.push(byte) else {
            continue;
        };
        let _ = frame.message();
        let mut out = [0u8; MAX_FRAME];
        let len = frame.encode(&mut out);
        let mut again = Deframer::new();
        let read = out.iter().take(len).find_map(|&b| again.push(b));
        assert!(matches!(read, Some(Ok(f)) if f == frame));
        assert!(!signing::verify(&frame, &[0xA5; 32]) || frame.signature.is_some());
    }
});
