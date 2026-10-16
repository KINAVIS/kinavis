//! Fuzz target: region deserialisation on arbitrary bytes, then its geometry.
//!
//! Stored field boundaries are untrusted input. Invariants: no panic; a
//! deserialised region re-serialises to one that deserialises to the same
//! region; an inset, when there is one, lies inside the region and holds no
//! more area.
#![no_main]

use libfuzzer_sys::fuzz_target;

use kinavis::region::{Containment, Region};
use kinavis::Distance;

type Field = Region<64, 32, 4>;

fuzz_target!(|data: &[u8]| {
    let Ok(region) = serde_json::from_slice::<Field>(data) else {
        return;
    };
    let written = serde_json::to_string(&region).expect("a region serialises");
    let again: Field = serde_json::from_str(&written).expect("what was written must read");
    assert_eq!(again, region, "writing is not a fixed point");

    let probe = region.outer().vertices()[0];
    let _ = region.contains(probe);
    let _ = region.distance_to_boundary(probe);
    // 0.125 to 25 m, never zero: a zero inset is the region itself, its
    // vertices on its own boundary.
    let eighths = u32::try_from(data.len() % 200).unwrap_or(0) + 1;
    let distance = Distance::from_metres(f64::from(eighths) / 8.0).expect("finite");
    if let Ok(inset) = region.inset(distance) {
        assert!(inset.area() <= region.area(), "an inset grew");
        for vertex in inset.outer().vertices() {
            assert_eq!(region.contains(*vertex), Containment::Inside, "inset vertex outside");
        }
    }
});
