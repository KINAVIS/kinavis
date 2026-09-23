//! Fuzz target: route deserialisation on arbitrary bytes.
//!
//! Stored passage plans are untrusted input. Invariants: no panic; a
//! deserialised route re-serialises to one that deserialises to the same route
//! (deserialisation applies construction-time validation and loses nothing).
#![no_main]

use libfuzzer_sys::fuzz_target;

use kinavis::route::Route;

fuzz_target!(|data: &[u8]| {
    let Ok(route) = serde_json::from_slice::<Route>(data) else {
        return;
    };
    let written = serde_json::to_string(&route).expect("a route serialises");
    let again: Route = serde_json::from_str(&written).expect("what was written must read");
    assert_eq!(again, route, "writing is not a fixed point");
});
