//! Identical results with `std` and `libm` maths.
//!
//! `std` uses the platform libm; the `libm` feature uses the pure-Rust crate.
//! The snapshot below comes from the `std` build; CI runs this test in both
//! configurations.
//!
//! To regenerate after an intentional change to the maths:
//!
//! ```text
//! cargo test --test determinism -- --ignored --nocapture
//! ```
//!
//! and state it in the commit message.

// Tests may panic on a failed expectation; the library may not. The snapshot is
// machine-written at round-trip precision.
#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::panic,
    clippy::unreadable_literal
)]

use core::time::Duration;

use kinavis::relative_motion::{Contact, Vessel};
use kinavis_kernel::{
    Angle, Distance, Instant, Position, Speed, TargetId, TrueBearing, TrueCourse, Utc,
};
use kinavis_traffic::{
    assess, avoid, CpaPolicy, ManoeuvreConstraints, PermittedSides, TargetObservation,
    TrackingPolicy, Traffic,
};

/// Allowed relative difference between the two implementations.
///
/// Track smoothing, assessment and manoeuvre search each end in a few
/// transcendental calls; 1000 ε covers them, far below bearing resolution.
const RELATIVE_TOLERANCE: f64 = 1e-13;

/// Output of the `std` build; see the module docs.
const EXPECTED: &[(&str, f64)] = &[
    ("smoothed course", 2.2249713593737386e2),
    ("smoothed speed", 1.3573646059616996e1),
    ("smoothed latitude", 5.008749093854444e1),
    ("smoothed longitude", -9.179236301305309e-1),
    ("relative course", 2.238110475065684e2),
    ("relative speed", 2.856684770227076e1),
    ("bearing drift", -1.3766572157915797e0),
    ("cpa distance", 3.229773657048144e0),
    ("cpa minutes", 1.537247960675e1),
    ("cpa bearing", 3.138110475065684e2),
    ("alteration", 1.1785880657272692e1),
    ("course to steer", 5.678588065727269e1),
    ("least passing", 3.999999999999999e0),
];

#[test]
fn the_two_maths_libraries_agree() {
    let measured = measurements();
    assert_eq!(
        measured.len(),
        EXPECTED.len(),
        "the snapshot is stale: regenerate it"
    );

    let mut worst = (0.0_f64, "");
    for ((name, found), (stored, expected)) in measured.iter().zip(EXPECTED) {
        assert_eq!(name, stored, "the snapshot is out of order: regenerate it");
        let error = (found - expected).abs() / expected.abs().max(1.0);
        if error > worst.0 {
            worst = (error, name);
        }
        assert!(
            error <= RELATIVE_TOLERANCE,
            "{name}: {found:e} vs {expected:e}, relative error {error:e}"
        );
    }
    println!("worst relative difference {:e}, at {}", worst.0, worst.1);
}

/// Prints the snapshot in the form of the constant above.
#[test]
#[ignore = "regenerates the snapshot rather than checking it"]
fn print_the_snapshot() {
    for (name, value) in measurements() {
        println!("    (\"{name}\", {value:e}),");
    }
}

/// Five plots smoothed into a track, its assessment against own ship, and the
/// clearing manoeuvre.
fn measurements() -> Vec<(String, f64)> {
    let mut out = Vec::new();
    let mut record = |name: &str, value: f64| out.push((name.to_owned(), value));

    let policy = TrackingPolicy::new(
        3,
        Duration::from_secs(30),
        Duration::from_secs(180),
        Speed::from_knots(60.0).unwrap(),
    )
    .unwrap();
    let mut traffic = Traffic::new(policy);
    let start = Instant::<Utc>::from_unix_seconds(1_789_000_000);
    let target = TargetId::new(7);
    // Plots 1 min apart with small scatter, heading roughly south-west.
    let plots = [
        (50.1000, -0.9000),
        (50.0972, -0.9041),
        (50.0945, -0.9079),
        (50.0916, -0.9122),
        (50.0889, -0.9158),
    ];
    for (minute, (latitude, longitude)) in (0_u64..).zip(plots) {
        let at = start.checked_add(Duration::from_secs(60 * minute)).unwrap();
        let position = Position::from_degrees(latitude, longitude).unwrap();
        let _ = traffic
            .ingest(TargetObservation::new(target, position, at))
            .unwrap();
    }
    let now = start.checked_add(Duration::from_secs(270)).unwrap();
    let view = traffic.view(now);
    let seen = *view.targets().first().expect("the target was acquired");
    let motion = seen.motion.unwrap();
    record("smoothed course", motion.course_over_ground.degrees());
    record("smoothed speed", motion.speed_over_ground.knots());
    record("smoothed latitude", seen.position.latitude().degrees());
    record("smoothed longitude", seen.position.longitude().degrees());

    let own = Vessel {
        course: TrueCourse::new(45.0).unwrap(),
        speed: Speed::from_knots(15.0).unwrap(),
    };
    let contact = Contact {
        bearing: TrueBearing::new(20.0).unwrap(),
        range: Distance::from_nautical_miles(8.0).unwrap(),
    };
    let other = Vessel {
        course: motion.course_over_ground,
        speed: motion.speed_over_ground,
    };
    let cpa = CpaPolicy::new(
        Distance::from_nautical_miles(2.0).unwrap(),
        Duration::from_secs(1800),
    )
    .unwrap();
    let assessment = assess(own, contact, other, &cpa).unwrap();
    record("relative course", assessment.relative_course().degrees());
    record("relative speed", assessment.relative_speed().knots());
    record(
        "bearing drift",
        assessment.bearing_drift().degrees_per_minute(),
    );
    let closest = assessment.cpa().unwrap();
    record("cpa distance", closest.distance.nautical_miles());
    record("cpa minutes", closest.time_to_go.as_secs_f64() / 60.0);
    record("cpa bearing", closest.bearing.degrees());

    let constraints = ManoeuvreConstraints::new(
        PermittedSides::Either,
        Angle::from_degrees(10.0).unwrap(),
        Angle::from_degrees(120.0).unwrap(),
    )
    .unwrap();
    let manoeuvre = avoid(
        own,
        contact,
        other,
        Distance::from_nautical_miles(4.0).unwrap(),
        &constraints,
    )
    .unwrap();
    record("alteration", manoeuvre.alteration().degrees());
    record("course to steer", manoeuvre.course().degrees());
    record("least passing", manoeuvre.least_passing().nautical_miles());

    out
}
