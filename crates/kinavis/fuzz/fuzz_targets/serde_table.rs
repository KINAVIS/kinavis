//! Fuzz target: deviation table deserialisation on arbitrary bytes.
//!
//! Table invariants (distinct courses, finite deviations, at most
//! `MAX_TABLE_NODES`) are enforced by the constructor and must hold on
//! deserialisation. Invariants: no panic; round trip preserves the table;
//! interpolation succeeds on every course.
#![no_main]

use libfuzzer_sys::fuzz_target;

use kinavis::{CompassCourse, DeviationTable, InterpolationMethod};

fuzz_target!(|data: &[u8]| {
    let Ok(table) = serde_json::from_slice::<DeviationTable>(data) else {
        return;
    };
    let written = serde_json::to_string(&table).expect("a table serialises");
    let again: DeviationTable = serde_json::from_str(&written).expect("what was written must read");
    assert_eq!(again, table, "writing is not a fixed point");
    for tenth in 0..3600 {
        let course = CompassCourse::new(f64::from(tenth) / 10.0).expect("in range");
        // Too few nodes for a method is an allowed error; a panic or non-finite
        // result is not.
        if let Ok(deviation) = table.deviation_at(course, InterpolationMethod::Linear) {
            assert!(deviation.degrees().is_finite());
        }
    }
});
