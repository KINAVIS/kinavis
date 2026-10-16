//! Fuzz target: the PGN decoder on arbitrary bytes.
//!
//! Bytes 0–2: PGN; byte 3: source address; the rest: reassembled payload.
//! Invariants: no panic; decoding is deterministic.
#![no_main]

use libfuzzer_sys::fuzz_target;

use kinavis_nmea2000::{Message, Payload, Pgn};

fuzz_target!(|data: &[u8]| {
    let [a, b, c, source, bytes @ ..] = data else {
        return;
    };
    let pgn = Pgn::new(u32::from_le_bytes([*a, *b, *c, 0]) & 0x3_FFFF);
    let Ok(payload) = Payload::from_bytes(pgn, *source, bytes) else {
        return;
    };
    let first = Message::decode(&payload);
    let again = Message::decode(&payload);
    assert_eq!(first, again, "decoding is not a function of the bytes");
});
