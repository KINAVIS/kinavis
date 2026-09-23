//! Fuzz target: the parser on arbitrary bytes.
//!
//! The parser is the entry point for untrusted data. Invariants: parsing never
//! panics, and anything that parses re-encodes to a sentence that parses to the
//! same value.
#![no_main]

use libfuzzer_sys::fuzz_target;

use kinavis_nmea0183::{parse, Sentence};

fuzz_target!(|data: &[u8]| {
    let Ok(sentence) = parse(data) else {
        return;
    };
    if matches!(sentence, Sentence::Unsupported { .. }) {
        return;
    }
    let written = sentence.to_string();
    let again = parse(written.as_bytes()).expect("what was written must parse");
    assert_eq!(again.to_string(), written, "writing is not a fixed point");
});
