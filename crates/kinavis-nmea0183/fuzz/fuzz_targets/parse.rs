//! Fuzz target: the parser on arbitrary bytes.
//!
//! The parser is the entry point for untrusted data. Invariants: parsing never
//! panics; anything that parses either re-encodes to a sentence that parses to
//! the same value, or is refused by `encode` as longer than the standard allows.
#![no_main]

use libfuzzer_sys::fuzz_target;

use kinavis_nmea0183::{encode, parse, NmeaError, Sentence, MAX_SENTENCE_BYTES};

fuzz_target!(|data: &[u8]| {
    let Ok(sentence) = parse(data) else {
        return;
    };
    if matches!(sentence, Sentence::Unsupported { .. }) {
        return;
    }
    let mut out = [0_u8; MAX_SENTENCE_BYTES];
    let length = match encode(&sentence, &mut out) {
        Ok(length) => length,
        Err(NmeaError::TooLong { .. }) => return,
        Err(error) => panic!("encoding failed: {error:?}"),
    };
    let written = out.get(..length).expect("encode returns a length within the buffer");
    let again = parse(written).expect("what was written must parse");
    let mut rewritten = [0_u8; MAX_SENTENCE_BYTES];
    let relength = encode(&again, &mut rewritten).expect("what was written must write again");
    assert_eq!(rewritten.get(..relength), Some(written), "writing is not a fixed point");
});
