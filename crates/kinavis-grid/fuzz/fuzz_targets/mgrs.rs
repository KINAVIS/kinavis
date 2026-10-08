//! Fuzz target: an MGRS reference read from arbitrary text.
//!
//! Invariants: no panic; reading is a function of the text; a square that
//! reads either converts to a position or is refused with an error.
#![no_main]

use libfuzzer_sys::fuzz_target;

use kinavis_grid::{Mgrs, MgrsSquare};

fuzz_target!(|data: &[u8]| {
    let Ok(text) = core::str::from_utf8(data) else {
        return;
    };
    let first = text.parse::<MgrsSquare>();
    let again = text.parse::<MgrsSquare>();
    assert_eq!(first.is_ok(), again.is_ok(), "reading is not a function of the text");
    if let Ok(square) = first {
        if let Ok((position, _, _)) = square.centre.to_position() {
            let _ = Mgrs::from_position(position, square.precision.unwrap_or(0));
        }
    }
});
