//! Properties over the whole domain.
//!
//! A worked example checks a formula at one point; these check the defining
//! relations — inverse conversions, symmetric distances — everywhere the types
//! allow, including poles, the antimeridian and a table's wrap-around segment.

// Tests may panic on a failed expectation; the library may not. Idempotence is
// exact: wrapping a wrapped value returns identical bits.
#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::cast_precision_loss,
    clippy::float_cmp
)]

use kinavis::navigation_solutions::{
    convert_compass_course_to_true_course, convert_true_course_to_compass_course,
};
use kinavis::sailings::{geodesic, geodesic_destination, great_circle, rhumb_line};
use kinavis::{
    wrap180, wrap360, CompassCourse, DeviationTable, Distance, InterpolationMethod, Latitude,
    Longitude, Position, TrueCourse, Variation,
};
use proptest::prelude::*;

/// Any compass direction in degrees.
fn course() -> impl Strategy<Value = f64> {
    0.0_f64..360.0
}

/// Latitude away from the poles, where rhumb lines are defined.
fn latitude() -> impl Strategy<Value = f64> {
    -89.0_f64..89.0
}

fn longitude() -> impl Strategy<Value = f64> {
    -180.0_f64..180.0
}

fn at(latitude: f64, longitude: f64) -> Position {
    Position::new(
        Latitude::from_degrees(latitude).expect("in range"),
        Longitude::from_degrees(longitude).expect("in range"),
    )
}

/// Gentle invertible swing: ±3° sinusoidal deviation.
fn gentle_table() -> DeviationTable {
    let values: [f64; 36] =
        core::array::from_fn(|index| 3.0 * ((index as f64 * 10.0).to_radians()).sin());
    DeviationTable::from_deviations(&values).expect("36 values is a valid swing")
}

proptest! {
    /// Wrapping is idempotent and lands in the promised interval.
    #[test]
    fn wrapping_is_idempotent(degrees in -10_000.0_f64..10_000.0) {
        let once = wrap360(degrees);
        prop_assert!((0.0..360.0).contains(&once), "{once}");
        prop_assert_eq!(once, wrap360(once));

        let signed = wrap180(degrees);
        prop_assert!((-180.0..180.0).contains(&signed), "{signed}");
        prop_assert_eq!(signed, wrap180(signed));
    }

    /// Compass → true → compass is the identity for an invertible table.
    #[test]
    fn the_compass_conversion_round_trips(
        degrees in course(),
        variation_degrees in -30.0_f64..30.0,
    ) {
        let table = gentle_table();
        prop_assume!(table.is_invertible());

        let compass = CompassCourse::new(degrees).unwrap();
        let variation = Variation::new(variation_degrees).unwrap();

        for method in [
            InterpolationMethod::Linear,
            InterpolationMethod::Cubic,
            InterpolationMethod::ShapePreserving,
        ] {
            let out = convert_compass_course_to_true_course(compass, variation, &table, method)
                .unwrap();
            let back = convert_true_course_to_compass_course(
                out.course,
                variation,
                &table,
                method,
            )
            .unwrap();
            let error = compass.angular_distance(back.course);
            prop_assert!(error < 1e-6, "{method:?} at {degrees}: off by {error}");
        }
    }

    /// Direct geodesic inverts the inverse geodesic.
    #[test]
    fn the_geodesic_round_trips(
        from_latitude in latitude(),
        from_longitude in longitude(),
        to_latitude in latitude(),
        to_longitude in longitude(),
    ) {
        let from = at(from_latitude, from_longitude);
        let to = at(to_latitude, to_longitude);

        let Ok(sailing) = geodesic(from, to) else {
            // Nearly antipodal: documented non-convergence, not a failure.
            return Ok(());
        };
        prop_assume!(sailing.distance.nautical_miles() > 1e-6);

        let arrived = geodesic_destination(from, sailing.initial_course, sailing.distance)
            .unwrap();
        let error = geodesic(arrived.position, to)
            .map_or(f64::MAX, |check| check.distance.metres());
        prop_assert!(error < 1e-3, "{error} m from {from} to {to}");
    }

    /// Distance is symmetric.
    #[test]
    fn distances_are_symmetric(
        from_latitude in latitude(),
        from_longitude in longitude(),
        to_latitude in latitude(),
        to_longitude in longitude(),
    ) {
        let from = at(from_latitude, from_longitude);
        let to = at(to_latitude, to_longitude);

        let there = great_circle(from, to).unwrap().distance.metres();
        let back = great_circle(to, from).unwrap().distance.metres();
        prop_assert!((there - back).abs() < 1e-6, "{there} vs {back}");

        let rhumb_there = rhumb_line(from, to).unwrap().distance.metres();
        let rhumb_back = rhumb_line(to, from).unwrap().distance.metres();
        prop_assert!(
            (rhumb_there - rhumb_back).abs() < 1e-6,
            "{rhumb_there} vs {rhumb_back}"
        );
    }

    /// A rhumb line is never shorter than the great circle.
    #[test]
    fn the_great_circle_is_the_shorter_track(
        from_latitude in latitude(),
        from_longitude in longitude(),
        to_latitude in latitude(),
        to_longitude in longitude(),
    ) {
        let from = at(from_latitude, from_longitude);
        let to = at(to_latitude, to_longitude);

        let shortest = great_circle(from, to).unwrap().distance.metres();
        let steered = rhumb_line(from, to).unwrap().distance.metres();
        // Same sphere, so the comparison is exact in principle; the tolerance
        // covers arithmetic only.
        prop_assert!(steered >= shortest - 1e-6, "{steered} < {shortest}");
    }

    /// Steering the reported course for the reported distance reaches the
    /// destination.
    ///
    /// Restricted to mid latitudes and one day's run: a high-latitude rhumb
    /// line winds round the pole and the arrival position does not record the
    /// windings, so measuring back gives a shorter distance. Found by proptest;
    /// documented at [`kinavis::sailings::rhumb_destination`].
    #[test]
    fn a_rhumb_line_leads_where_it_says(
        from_latitude in -60.0_f64..60.0,
        from_longitude in longitude(),
        course_degrees in course(),
        miles in 1.0_f64..600.0,
    ) {
        let from = at(from_latitude, from_longitude);
        let steered = TrueCourse::new(course_degrees).unwrap();
        let run = Distance::from_nautical_miles(miles).unwrap();

        let Ok(arrived) = kinavis::sailings::rhumb_destination(from, steered, run) else {
            return Ok(());
        };
        // Tracks crossing a pole have no rhumb line back; skipped.
        prop_assume!(arrived.latitude().degrees().abs() < 80.0);

        let measured = rhumb_line(from, arrived).unwrap();
        prop_assert!(
            (measured.distance.nautical_miles() - miles).abs() < 1e-6,
            "{} vs {miles}",
            measured.distance.nautical_miles()
        );
        prop_assert!(
            steered.angular_distance(measured.initial_course) < 1e-6,
            "{} vs {course_degrees}",
            measured.initial_course.degrees()
        );
    }

    /// Interpolation stays within the observed range of the swing.
    #[test]
    fn interpolated_deviation_stays_within_the_table(degrees in course()) {
        let table = gentle_table();
        let compass = CompassCourse::new(degrees).unwrap();
        let largest = table.max_abs_deviation();

        for method in [
            InterpolationMethod::Linear,
            InterpolationMethod::ShapePreserving,
        ] {
            let value = table.deviation_at(compass, method).unwrap().degrees();
            prop_assert!(
                value.abs() <= largest + 1e-9,
                "{method:?} gave {value}, table peaks at {largest}"
            );
        }
    }
}
