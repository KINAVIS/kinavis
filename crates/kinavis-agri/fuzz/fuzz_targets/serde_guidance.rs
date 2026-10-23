//! Fuzz target: a stored job — field, guidance lines, pass pattern, boom,
//! machine — read from arbitrary bytes, then the geometry on it.
//!
//! Stored jobs are untrusted input. Invariants: no panic; a job read back
//! writes to one that reads back the same; a curve passes through its own
//! vertices; every vertex of a parallel curve, when there is one, lies on the
//! exact offset within the bound `CurveLine::offset` states.
#![no_main]

use libfuzzer_sys::fuzz_target;

use kinavis::region::Containment;
use kinavis_agri::{
    AbLine, Boom, CurveLine, Field, PassIndex, PassPattern, PivotLine, ReferenceLine,
    TrackerConfig, VehicleGeometry,
};
use kinavis_kernel::Distance;

#[derive(Debug, PartialEq, serde::Serialize, serde::Deserialize)]
struct Job {
    #[serde(default)]
    field: Option<Field<64, 32, 4>>,
    #[serde(default)]
    curve: Option<CurveLine<64>>,
    #[serde(default)]
    ab: Option<AbLine>,
    #[serde(default)]
    pivot: Option<PivotLine>,
    #[serde(default)]
    pattern: Option<PassPattern>,
    #[serde(default)]
    boom: Option<Boom<8>>,
    #[serde(default)]
    vehicle: Option<VehicleGeometry>,
    #[serde(default)]
    tracker: Option<TrackerConfig>,
}

const TOLERANCE_METRES: f64 = 0.05;

fuzz_target!(|data: &[u8]| {
    let Ok(job) = serde_json::from_slice::<Job>(data) else {
        return;
    };
    // A grid plane on a datum other than the presets never reads, so every
    // job read writes.
    let written = serde_json::to_string(&job).expect("a job read writes");
    let again: Job = serde_json::from_str(&written).expect("what was written must read");
    assert_eq!(again, job, "writing is not a fixed point");

    if let Some(ab) = job.ab {
        assert!(
            ab.station(ab.origin()).offset.metres().abs() < 1e-6,
            "A off its line"
        );
    }
    if let Some(field) = job.field {
        let probe = field.region().outer().vertices()[0];
        assert_ne!(
            field.region().contains(probe),
            Containment::Inside,
            "vertex inside"
        );
    }
    if let Some(boom) = job.boom {
        assert!(boom.width().metres() > 0.0, "a boom without width");
    }
    let Some(curve) = job.curve else {
        return;
    };
    for vertex in curve.vertices() {
        let offset = curve.station(*vertex).offset.metres();
        assert!(
            offset.abs() < 1e-6,
            "vertex {vertex:?} is {offset} m off its curve"
        );
    }
    let pattern = job.pattern.unwrap_or(
        PassPattern::new(Distance::from_metres(6.0).expect("finite"), Distance::ZERO)
            .expect("a pattern"),
    );
    let tolerance = Distance::from_metres(TOLERANCE_METRES).expect("finite");
    for number in -3..=3 {
        let d = pattern.offset_of(PassIndex::new(number)).metres();
        let Ok(pass) = curve.offset::<256>(Distance::from_metres(d).expect("finite"), tolerance)
        else {
            continue;
        };
        let bound = 2.0 * TOLERANCE_METRES + 1e-6 * (1.0 + d.abs());
        for vertex in pass.vertices() {
            let error = (curve.station(*vertex).offset.metres() - d).abs();
            assert!(
                error <= bound,
                "pass {number} at {d} m: vertex {vertex:?} off by {error} m"
            );
        }
    }
});
