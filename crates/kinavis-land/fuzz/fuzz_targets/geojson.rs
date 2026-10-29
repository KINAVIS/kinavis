//! Fuzz target: GeoJSON from arbitrary text.
//!
//! Invariants: no panic; features that read are written and read back as
//! many.
#![no_main]

use libfuzzer_sys::fuzz_target;

use kinavis_land::{parse_geojson, to_geojson};

fuzz_target!(|data: &[u8]| {
    let Ok(text) = core::str::from_utf8(data) else {
        return;
    };
    if let Ok(features) = parse_geojson(text) {
        let again = parse_geojson(&to_geojson(&features)).expect("our own output reads");
        assert_eq!(again.len(), features.len());
    }
});
