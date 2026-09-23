//! Fuzz target: the message decoder on arbitrary bits.
//!
//! Byte 0: fill bits in the last character; the rest: armoured payload.
//! Invariants: no panic; decoding is deterministic.
#![no_main]

use libfuzzer_sys::fuzz_target;

use kinavis_ais::{Bits, Message};

fuzz_target!(|data: &[u8]| {
    let Some((&fill, payload)) = data.split_first() else {
        return;
    };
    let Ok(bits) = Bits::unarmour(payload, fill % 8) else {
        return;
    };
    let first = Message::decode(&bits);
    let again = Message::decode(&bits);
    assert_eq!(first, again, "decoding is not a function of the bits");
});
