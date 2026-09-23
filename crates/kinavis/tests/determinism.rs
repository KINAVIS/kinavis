//! Identical results with `std` and `libm` maths.
//!
//! `std` uses the platform libm; the `libm` feature uses the pure-Rust crate.
//! They need not agree to the last bit, so the allowed difference is stated and
//! tested: a plan computed ashore and on the bridge must agree.
//!
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
    clippy::cast_precision_loss,
    clippy::unreadable_literal
)]

use kinavis::navigation_solutions::{convert_compass_course_to_true_course, course_to_steer};
use kinavis::relative_motion::{closest_point_of_approach, Approach, Contact, Vessel};
use kinavis::sailings::{cross_track, geodesic, great_circle, rhumb_line};
use kinavis::{
    CompassCourse, DeviationTable, Distance, InterpolationMethod, Latitude, Longitude, Position,
    Speed, TrueCourse, Variation,
};

/// Allowed relative difference between the two implementations.
///
/// 1000 ε: covers the rounding of chained transcendental calls, far below any
/// steering or fixing resolution.
const RELATIVE_TOLERANCE: f64 = 1e-13;

/// Output of the `std` build; see the module docs.
const EXPECTED: &[(&str, f64)] = &[
    ("geodesic distance", 4.400569641545919e6),
    ("geodesic initial course", 2.855186098598503e2),
    ("geodesic final course", 2.398290723772108e2),
    ("great circle distance", 4.387957396422941e6),
    ("great circle initial course", 2.8549288870176326e2),
    ("rhumb distance", 4.502369467231812e6),
    ("rhumb course", 2.6190742239797544e2),
    ("cross track", 5.428876433675403e5),
    ("along track", 1.8480527232796166e6),
    ("linear true course", 1.1864609563147455e2),
    ("linear error", 9.867653873996574e-3),
    ("cubic true course", 1.1865453885696725e2),
    ("cubic error", 9.867653873996574e-3),
    ("parametric true course", 1.1865454358979022e2),
    ("parametric error", 5.576907401953478e-16),
    ("shape preserving true course", 1.1865521326777915e2),
    ("shape preserving error", 9.867653873996574e-3),
    ("heading to steer", 2.640165414566188e2),
    ("speed made good", 1.0896027870557544e1),
    ("drift angle", -1.0983458543381188e1),
    ("cpa distance", 3.6462417517518966e0),
    ("cpa minutes", 1.6447758083983334e1),
    ("cpa bearing", 3.171151461328941e2),
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

fn at(latitude: f64, longitude: f64) -> Position {
    Position::new(
        Latitude::from_degrees(latitude).expect("in range"),
        Longitude::from_degrees(longitude).expect("in range"),
    )
}

/// Gentle swing, exercising every interpolation method.
fn table() -> DeviationTable {
    let values: [f64; 36] =
        core::array::from_fn(|index| 3.0 * ((index as f64 * 10.0).to_radians()).sin());
    DeviationTable::from_deviations(&values).expect("36 values is a valid swing")
}

/// One pass through the kernels that depend on the floating-point library.
fn measurements() -> Vec<(String, f64)> {
    let mut out = Vec::new();
    let mut record = |name: &str, value: f64| out.push((name.to_owned(), value));

    let plymouth = at(50.35, -4.15);
    let halifax = at(44.65, -63.57);

    let g = geodesic(plymouth, halifax).unwrap();
    record("geodesic distance", g.distance.metres());
    record("geodesic initial course", g.initial_course.degrees());
    record("geodesic final course", g.final_course.degrees());

    let c = great_circle(plymouth, halifax).unwrap();
    record("great circle distance", c.distance.metres());
    record("great circle initial course", c.initial_course.degrees());

    let r = rhumb_line(plymouth, halifax).unwrap();
    record("rhumb distance", r.distance.metres());
    record("rhumb course", r.initial_course.degrees());

    let off = cross_track(at(47.0, -30.0), plymouth, halifax).unwrap();
    record("cross track", off.distance.metres());
    record("along track", off.along_track.metres());

    let table = table();
    for (name, method) in [
        ("linear", InterpolationMethod::Linear),
        ("cubic", InterpolationMethod::Cubic),
        ("parametric", InterpolationMethod::Parametric),
        ("shape preserving", InterpolationMethod::ShapePreserving),
    ] {
        let solution = convert_compass_course_to_true_course(
            CompassCourse::new(123.4).unwrap(),
            Variation::new(-7.25).unwrap(),
            &table,
            method,
        )
        .unwrap();
        record(&format!("{name} true course"), solution.course.degrees());
        record(&format!("{name} error"), solution.estimated_error);
    }

    let steer = course_to_steer(
        TrueCourse::new(275.0).unwrap(),
        Speed::from_knots(12.5).unwrap(),
        TrueCourse::new(35.0).unwrap(),
        Speed::from_knots(2.75).unwrap(),
    )
    .unwrap();
    record("heading to steer", steer.heading.degrees());
    record("speed made good", steer.speed_over_ground.knots());
    record("drift angle", steer.drift_angle.degrees());

    let own = Vessel {
        course: TrueCourse::new(45.0).unwrap(),
        speed: Speed::from_knots(15.0).unwrap(),
    };
    let target = Vessel {
        course: TrueCourse::new(230.0).unwrap(),
        speed: Speed::from_knots(11.0).unwrap(),
    };
    let contact = Contact {
        bearing: kinavis::TrueBearing::new(20.0).expect("in range"),
        range: Distance::from_nautical_miles(8.0).unwrap(),
    };
    match closest_point_of_approach(own, contact, target).unwrap() {
        Approach::Closing(cpa) => {
            record("cpa distance", cpa.distance.nautical_miles());
            record("cpa minutes", cpa.time_to_go.as_secs_f64() / 60.0);
            record("cpa bearing", cpa.bearing.degrees());
        }
        other => panic!("the fixture must be a closing contact, not {other:?}"),
    }

    out
}
