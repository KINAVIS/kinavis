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

use kinavis_ins::{
    gravity_down, GatingPolicy, ImuNoise, ImuSample, InsFilter, InsPriors, Quaternion, Strapdown,
    EARTH_RATE,
};
use kinavis_kernel::{
    Angle, Distance, GeodeticPoint, Height, Instant, Ned, Position, Speed, TrueCourse, Utc, Vector3,
};

/// Allowed relative difference between the two implementations.
///
/// A minute of strapdown integration is thousands of chained quaternion
/// products and trigonometric calls; 1000 ε covers the rounding and is far
/// below any reported sigma.
const RELATIVE_TOLERANCE: f64 = 1e-13;

/// Output of the `std` build; see the module docs.
const EXPECTED: &[(&str, f64)] = &[
    ("drifted roll", 1.8434292678053597e0),
    ("drifted pitch", -7.49414610131153e-1),
    ("drifted yaw", 3.7000058943942314e1),
    ("drifted north velocity", -1.1395520067787872e0),
    ("drifted east velocity", 1.5014683805871942e0),
    ("drifted latitude", 5.075478880999492e1),
    ("drifted longitude", -1.332860544057894e0),
    ("drifted position sigma", 4.370685914843071e2),
    ("drifted heading sigma", 4.242822208962965e0),
    ("corrected yaw", 3.7201304604532034e1),
    ("corrected velocity", 2.324467089994937e-6),
    ("corrected heading sigma", 4.9656362661726305e-1),
    ("corrected velocity sigma", 1.9999988358020047e-2),
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

/// One minute stationary, then heading and zero-velocity updates:
/// mechanisation, covariance propagation and update in one run.
fn measurements() -> Vec<(String, f64)> {
    let mut out = Vec::new();
    let mut record = |name: &str, value: f64| out.push((name.to_owned(), value));

    let start = Instant::<Utc>::from_unix_seconds(1_789_000_000);
    let berth = GeodeticPoint::new(
        Position::from_degrees(50.755, -1.3333).unwrap(),
        Height::above_ellipsoid(Distance::ZERO),
    );
    let attitude = Quaternion::from_euler(
        Angle::from_degrees(1.5).unwrap(),
        Angle::from_degrees(-0.75).unwrap(),
        TrueCourse::new(37.0).unwrap(),
    );
    let still = Vector3::<Ned, Speed>::new(Speed::ZERO, Speed::ZERO, Speed::ZERO);
    let mut ins = InsFilter::new(
        Strapdown::new(start, berth, still, attitude).unwrap(),
        ImuNoise::mems(),
        &InsPriors::standard(),
    );

    // IMU reading at rest plus a small constant error per axis, so the
    // integration has something to carry.
    let latitude = 50.755_f64.to_radians();
    let earth = [
        EARTH_RATE * latitude.cos(),
        0.0,
        -EARTH_RATE * latitude.sin(),
    ];
    let gravity = [0.0, 0.0, -gravity_down(latitude.sin(), 0.0)];
    let mut rates = attitude.rotate_back(earth);
    let mut forces = attitude.rotate_back(gravity);
    rates[0] += 1e-4;
    forces[1] += 2e-3;
    let sample = ImuSample::new(rates, forces, Duration::from_millis(100)).unwrap();
    for _ in 0..600 {
        ins.predict(&sample).unwrap();
    }
    let drifted = ins.attitude();
    record("drifted roll", drifted.roll.degrees());
    record("drifted pitch", drifted.pitch.degrees());
    record("drifted yaw", drifted.yaw.degrees());
    record(
        "drifted north velocity",
        ins.velocity().north().metres_per_second(),
    );
    record(
        "drifted east velocity",
        ins.velocity().east().metres_per_second(),
    );
    let position = ins.position().unwrap();
    record("drifted latitude", position.latitude().degrees());
    record("drifted longitude", position.longitude().degrees());
    record(
        "drifted position sigma",
        ins.horizontal_error().semi_major().metres(),
    );
    record("drifted heading sigma", ins.heading_sigma().degrees());

    ins.update_heading(
        TrueCourse::new(37.2).unwrap(),
        Angle::from_degrees(0.5).unwrap(),
        GatingPolicy::none(),
    )
    .unwrap();
    ins.update_zero_velocity(
        Speed::from_metres_per_second(0.02).unwrap(),
        GatingPolicy::none(),
    )
    .unwrap();
    let corrected = ins.attitude();
    record("corrected yaw", corrected.yaw.degrees());
    record(
        "corrected velocity",
        ins.velocity().magnitude().metres_per_second(),
    );
    record("corrected heading sigma", ins.heading_sigma().degrees());
    record(
        "corrected velocity sigma",
        ins.velocity_sigma().metres_per_second(),
    );

    out
}
