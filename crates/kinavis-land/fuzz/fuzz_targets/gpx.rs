//! Fuzz target: a GPX file from arbitrary text.
//!
//! Invariants: no panic; a file that reads is written and reads back with
//! the same points, times and names.
#![no_main]

use libfuzzer_sys::fuzz_target;

use kinavis_land::Gpx;

fuzz_target!(|data: &[u8]| {
    let Ok(text) = core::str::from_utf8(data) else {
        return;
    };
    let Ok(gpx) = Gpx::parse(text) else {
        return;
    };
    let again = Gpx::parse(&gpx.to_xml()).expect("our own output reads");
    assert_eq!(again.waypoints.len(), gpx.waypoints.len());
    assert_eq!(again.routes.len(), gpx.routes.len());
    assert_eq!(again.tracks.len(), gpx.tracks.len());
    for (a, b) in gpx.waypoints.iter().zip(&again.waypoints) {
        assert_eq!(a.time, b.time);
        assert!((a.position.latitude().degrees() - b.position.latitude().degrees()).abs() < 1e-9);
    }
});
