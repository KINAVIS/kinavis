//! Conversions and current triangle against hand calculations and each other.

#![allow(clippy::unwrap_used, clippy::float_cmp, clippy::indexing_slicing)]

use super::*;
use crate::angle::MagneticCourse;
use crate::deviation::{DeviationCoefficients, DeviationTable, Interpolation, InterpolationMethod};
use crate::environment::CompassModel;
use crate::error::{KernelError, NavigationError};
use crate::math;
use crate::position::Latitude;
use crate::CompassCourse;
use alloc::format;
use alloc::vec::Vec;

fn readme_table() -> DeviationTable {
    DeviationTable::from_deviations(&[
        -2.5, -0.5, 1.6, 4.4, -1.7, 0.0, 1.0, 0.3, -0.9, 0.5, -1.2, 0.8, -0.3, 1.7, -2.1, 0.4,
        -0.6, 1.2, -1.3, 0.0, 0.9, -1.1, 1.5, -0.7, -13.2, -15.7, -17.9, -19.2, -18.1, 1.8, -0.4,
        0.7, -0.2, 1.4, -4.4, -2.9,
    ])
    .unwrap()
}

/// Smooth swing with slope well under 1° per degree: uniquely invertible.
fn realistic_table() -> DeviationTable {
    let truth = crate::SmithCoefficients {
        a: 2.0,
        b: 3.0,
        c: -4.0,
        d: 1.5,
        e: -0.5,
    };
    let values: Vec<f64> = (0..36)
        .map(|index| truth.deviation_at(CompassCourse::wrap(f64::from(index) * 10.0).unwrap()))
        .collect();
    DeviationTable::from_deviations(&values).unwrap()
}

#[test]
fn corrections_are_inverses_of_each_other() {
    let variation = Variation::new(-7.5).unwrap();
    let deviation = Deviation::new(3.25).unwrap();

    for degrees in [0.0, 1.0, 90.0, 183.7, 359.5] {
        let magnetic = MagneticCourse::new(degrees).unwrap();
        let back = true_to_magnetic(magnetic_to_true(magnetic, variation), variation);
        assert!(back.angular_distance(magnetic) < 1e-12);

        let compass = CompassCourse::new(degrees).unwrap();
        let round = magnetic_to_compass(compass_to_magnetic(compass, deviation), deviation);
        assert!(round.angular_distance(compass) < 1e-12);
    }
}

#[test]
fn corrections_wrap_instead_of_going_negative() {
    // Plain subtraction would give 10 - 170 = -160 here.
    let deviation = Deviation::new(170.0).unwrap();
    let compass = magnetic_to_compass(MagneticCourse::new(10.0).unwrap(), deviation);
    assert!((compass.degrees() - 200.0).abs() < 1e-12);
    assert!((0.0..360.0).contains(&compass.degrees()));
}

#[test]
fn course_angle_is_measured_from_the_head() {
    let course = TrueCourse::new(90.0).unwrap();
    assert_eq!(
        calculate_course_angle(course, TrueCourse::new(180.0).unwrap()).degrees(),
        90.0
    );
    assert_eq!(
        calculate_course_angle(course, TrueCourse::new(45.0).unwrap()).degrees(),
        315.0
    );
    let relative = calculate_course_angle(course, TrueCourse::new(45.0).unwrap());
    assert!(
        bearing_from_relative(course, relative).angular_distance(TrueCourse::new(45.0).unwrap())
            < 1e-12
    );
}

#[test]
fn compass_to_true_matches_the_hand_calculation() {
    let mut table = DeviationTable::default();
    table
        .set_deviation(0, Deviation::new(-2.5).unwrap())
        .unwrap();
    table
        .set_deviation(10, Deviation::new(-1.5).unwrap())
        .unwrap();

    let solution = convert_compass_course_to_true_course(
        CompassCourse::new(5.0).unwrap(),
        Variation::new(-10.0).unwrap(),
        &table,
        InterpolationMethod::Linear,
    )
    .unwrap();

    assert_eq!(format!("{:.2}", solution.course.degrees()), "353.00");
    assert_eq!(format!("{:.2}", solution.deviation.degrees()), "-2.00");
    assert_eq!(format!("{:.2}", solution.total_correction), "-12.00");
    assert!(!solution.check_data_required());
}

#[test]
fn round_trip_is_exact_for_every_method() {
    let table = realistic_table();
    assert!(table.is_invertible());
    let variation = Variation::new(-2.7).unwrap();

    for method in [
        InterpolationMethod::Linear,
        InterpolationMethod::Cubic,
        InterpolationMethod::Parametric,
    ] {
        let mut course = 0.0;
        while course < 360.0 {
            let compass = CompassCourse::new(course).unwrap();
            let out =
                convert_compass_course_to_true_course(compass, variation, &table, method).unwrap();
            let back = convert_true_course_to_compass_course(out.course, variation, &table, method)
                .unwrap();

            assert!(
                back.course.angular_distance(compass) < 1e-6,
                "{method:?} at {course}: came back {}",
                back.course.degrees()
            );
            course += 0.5;
        }
    }
}

#[test]
fn a_non_invertible_table_still_yields_a_course_that_checks_out() {
    // 12.5° of deviation over one 10° step: two compass courses share a
    // magnetic course, so the round trip cannot be the identity. The returned
    // compass course must still produce the requested true course.
    let table = readme_table();
    assert!(!table.is_invertible());
    let variation = Variation::new(-2.7).unwrap();

    for method in [
        InterpolationMethod::Linear,
        InterpolationMethod::Cubic,
        InterpolationMethod::Parametric,
    ] {
        let mut course = 0.0;
        while course < 360.0 {
            let requested = TrueCourse::new(course).unwrap();
            let compass =
                convert_true_course_to_compass_course(requested, variation, &table, method)
                    .unwrap();
            let achieved =
                convert_compass_course_to_true_course(compass.course, variation, &table, method)
                    .unwrap();

            assert!(
                achieved.course.angular_distance(requested) < 1e-6,
                "{method:?} at {course}: steering {} makes good {}",
                compass.course.degrees(),
                achieved.course.degrees()
            );
            assert!(compass.advisories.non_invertible_table);
            course += 0.5;
        }
    }
}

#[test]
fn round_trip_survives_the_steep_part_of_the_curve() {
    // 250° compass is where the curve is steepest.
    let table = readme_table();
    let variation = Variation::new(-2.7).unwrap();
    let compass = CompassCourse::new(250.0).unwrap();

    let out = convert_compass_course_to_true_course(
        compass,
        variation,
        &table,
        InterpolationMethod::Linear,
    )
    .unwrap();
    let back = convert_true_course_to_compass_course(
        out.course,
        variation,
        &table,
        InterpolationMethod::Linear,
    )
    .unwrap();

    assert!(back.course.angular_distance(compass) < 1e-9);
    assert!(out.advisories.large_deviation);
}

#[test]
fn magnetic_round_trip_uses_the_same_solver() {
    let table = readme_table();
    for degrees in [0.0, 33.0, 180.0, 254.0, 359.0] {
        let compass = CompassCourse::new(degrees).unwrap();
        let magnetic =
            convert_compass_course_to_magnetic_course(compass, &table, InterpolationMethod::Cubic)
                .unwrap();
        let back = convert_magnetic_course_to_compass_course(
            magnetic.course,
            &table,
            InterpolationMethod::Cubic,
        )
        .unwrap();
        assert!(back.course.angular_distance(compass) < 1e-6);
    }
}

#[test]
fn solver_handles_a_steep_but_invertible_curve() {
    // 0.9° of deviation per degree of heading: extreme but invertible.
    let values: Vec<f64> = (0..36)
        .map(|index| {
            let course = f64::from(index) * 10.0;
            9.0 * math::sin(math::to_radians(course))
        })
        .collect();
    let table = DeviationTable::from_deviations(&values).unwrap();

    let mut course = 0.0;
    while course < 360.0 {
        let compass = CompassCourse::new(course).unwrap();
        let magnetic =
            convert_compass_course_to_magnetic_course(compass, &table, InterpolationMethod::Linear)
                .unwrap();
        let back = convert_magnetic_course_to_compass_course(
            magnetic.course,
            &table,
            InterpolationMethod::Linear,
        )
        .unwrap();
        assert!(
            back.course.angular_distance(compass) < 1e-4,
            "at {course}: {} came back as {}",
            magnetic.course.degrees(),
            back.course.degrees()
        );
        course += 1.0;
    }
}

#[test]
fn advisories_flag_unusual_data() {
    let quiet = convert_compass_course_to_true_course(
        CompassCourse::new(100.0).unwrap(),
        Variation::new(-2.7).unwrap(),
        &realistic_table(),
        InterpolationMethod::Linear,
    )
    .unwrap();
    assert!(!quiet.advisories.any());

    let loud = convert_compass_course_to_true_course(
        CompassCourse::new(270.0).unwrap(),
        Variation::new(-20.0).unwrap(),
        &readme_table(),
        InterpolationMethod::Linear,
    )
    .unwrap();
    assert!(loud.advisories.large_variation);
    assert!(loud.advisories.large_deviation);
    assert!(loud.advisories.non_invertible_table);
    assert!(!loud.advisories.coarse_table);

    // An eight-point swing is normal practice, not coarse...
    let cardinal = convert_compass_course_to_true_course(
        CompassCourse::new(10.0).unwrap(),
        Variation::ZERO,
        &DeviationTable::from_cardinal_directions(),
        InterpolationMethod::Linear,
    )
    .unwrap();
    assert!(!cardinal.advisories.coarse_table);

    // ...four points is.
    let coarse = convert_compass_course_to_true_course(
        CompassCourse::new(10.0).unwrap(),
        Variation::ZERO,
        &DeviationTable::from_step(90).unwrap(),
        InterpolationMethod::Linear,
    )
    .unwrap();
    assert!(coarse.advisories.coarse_table);
}

#[test]
fn estimated_error_is_zero_on_a_straight_curve() {
    let values: Vec<f64> = (0..36).map(|_| 1.0).collect();
    let table = DeviationTable::from_deviations(&values).unwrap();
    let solution = convert_compass_course_to_true_course(
        CompassCourse::new(37.0).unwrap(),
        Variation::ZERO,
        &table,
        InterpolationMethod::Linear,
    )
    .unwrap();
    assert!(solution.estimated_error < 1e-12);

    let bumpy = readme_table();
    let bumpy_solution = convert_compass_course_to_true_course(
        CompassCourse::new(255.0).unwrap(),
        Variation::ZERO,
        &bumpy,
        InterpolationMethod::Linear,
    )
    .unwrap();
    assert!(bumpy_solution.estimated_error > 0.0);
}

#[test]
fn custom_coefficients_reach_the_conversion() {
    let table = readme_table();
    let coefficients = DeviationCoefficients {
        a: Some(0.0),
        b: Some(0.0),
        c: Some(0.0),
        d: Some(0.0),
        e: Some(0.0),
    };
    let solution = convert_compass_course_to_true_course(
        CompassCourse::new(123.0).unwrap(),
        Variation::ZERO,
        &table,
        Interpolation {
            method: InterpolationMethod::Parametric,
            coefficients: Some(&coefficients),
        },
    )
    .unwrap();
    assert_eq!(solution.deviation.degrees(), 0.0);
    assert_eq!(solution.course.degrees(), 123.0);
}

#[test]
fn current_triangle_is_self_consistent() {
    let heading = TrueCourse::new(35.0).unwrap();
    let set = TrueCourse::new(150.0).unwrap();
    let through_water = Speed::from_knots(12.0).unwrap();
    let drift = Speed::from_knots(3.0).unwrap();
    let track = course_over_ground(heading, through_water, set, drift).unwrap();

    let current = estimate_current(
        heading,
        through_water,
        track.course_over_ground,
        track.speed_over_ground,
    )
    .unwrap();
    assert!(current.set.angular_distance(set) < 1e-9);
    assert!((current.drift.knots() - 3.0).abs() < 1e-9);

    let steering = course_to_steer(track.course_over_ground, through_water, set, drift).unwrap();
    assert!(steering.heading.angular_distance(heading) < 1e-9);
    assert!((steering.speed_over_ground.knots() - track.speed_over_ground.knots()).abs() < 1e-9);
}

#[test]
fn course_over_ground_matches_a_hand_calculation() {
    let track = course_over_ground(
        TrueCourse::new(0.0).unwrap(),
        Speed::from_knots(10.0).unwrap(),
        TrueCourse::new(90.0).unwrap(),
        Speed::from_knots(2.0).unwrap(),
    )
    .unwrap();
    assert_eq!(
        format!("{:.4}", track.course_over_ground.degrees()),
        "11.3099"
    );
    assert_eq!(format!("{:.4}", track.speed_over_ground.knots()), "10.1980");
}

#[test]
fn current_triangle_validates_speeds() {
    let north = TrueCourse::NORTH;
    let one = Speed::from_knots(1.0).unwrap();
    let ten = Speed::from_knots(10.0).unwrap();
    let sternway = Speed::from_knots(-1.0).unwrap();

    assert!(course_over_ground(north, sternway, north, one).is_err());
    assert!(course_to_steer(north, ten, north, sternway).is_err());
    assert!(estimate_current(north, ten, north, sternway).is_err());
    // A non-finite speed cannot be constructed.
    assert!(Speed::from_knots(f64::NAN).is_err());
    assert!(Speed::from_knots(f64::INFINITY).is_err());
}

#[test]
fn dead_water_has_no_course_over_ground() {
    let track = course_over_ground(
        TrueCourse::new(0.0).unwrap(),
        Speed::from_knots(5.0).unwrap(),
        TrueCourse::new(180.0).unwrap(),
        Speed::from_knots(5.0).unwrap(),
    );
    assert_eq!(
        track.unwrap_err(),
        NavigationError::Kernel(KernelError::Missing {
            what: "course over ground"
        })
    );
}

#[test]
fn a_current_stronger_than_the_ship_is_reported() {
    let result = course_to_steer(
        TrueCourse::new(0.0).unwrap(),
        Speed::from_knots(2.0).unwrap(),
        TrueCourse::new(90.0).unwrap(),
        Speed::from_knots(10.0).unwrap(),
    );
    assert!(matches!(
        result.unwrap_err(),
        NavigationError::CurrentTooStrong { .. }
    ));

    // A following current stronger than the ship is valid.
    let following = course_to_steer(
        TrueCourse::new(0.0).unwrap(),
        Speed::from_knots(2.0).unwrap(),
        TrueCourse::new(0.0).unwrap(),
        Speed::from_knots(10.0).unwrap(),
    )
    .unwrap();
    assert!((following.speed_over_ground.knots() - 12.0).abs() < 1e-9);
}

#[test]
fn gyro_corrections_are_inverses() {
    use crate::GyroCourse;
    let error = Angle::from_degrees(-1.5).unwrap();
    for degrees in [0.0, 1.0, 90.0, 359.5] {
        let gyro = GyroCourse::new(degrees).unwrap();
        let back = true_to_gyro(gyro_to_true(gyro, error), error);
        assert!(back.angular_distance(gyro) < 1e-12);
    }
}

#[test]
fn gyro_error_comes_out_of_a_transit() {
    use crate::{GyroBearing, TrueBearing};
    let observed = GyroBearing::new(46.5).unwrap();
    let reference = TrueBearing::new(45.0).unwrap();
    let error = gyro_error_from_transit(observed, reference);
    assert!((error.degrees() + 1.5).abs() < 1e-12);
    // Applying it to the observation recovers the reference.
    assert!(gyro_to_true(observed, error).angular_distance(reference) < 1e-12);
}

#[test]
fn gyro_speed_error_is_westerly_going_north_and_easterly_going_south() {
    let latitude = Latitude::from_degrees(60.0).unwrap();
    let speed = Speed::from_knots(20.0).unwrap();

    let north = gyro_speed_error(latitude, TrueCourse::NORTH, speed).unwrap();
    let south = gyro_speed_error(latitude, TrueCourse::SOUTH, speed).unwrap();
    assert!(north.degrees() < 0.0);
    assert!(south.degrees() > 0.0);
    assert!((north.degrees() + south.degrees()).abs() < 1e-9);

    // Same rule in the southern hemisphere.
    let southern = Latitude::from_degrees(-40.0).unwrap();
    assert!(
        gyro_speed_error(southern, TrueCourse::NORTH, speed)
            .unwrap()
            .degrees()
            < 0.0
    );

    // No speed error heading due east or west, or when stopped.
    assert!(
        gyro_speed_error(latitude, TrueCourse::EAST, speed)
            .unwrap()
            .degrees()
            .abs()
            < 1e-12
    );
    assert!(
        gyro_speed_error(latitude, TrueCourse::NORTH, Speed::ZERO)
            .unwrap()
            .degrees()
            .abs()
            < 1e-12
    );
}

#[test]
fn gyro_speed_error_grows_with_latitude_and_speed() {
    let slow = Speed::from_knots(10.0).unwrap();
    let fast = Speed::from_knots(25.0).unwrap();
    let low = Latitude::from_degrees(10.0).unwrap();
    let high = Latitude::from_degrees(70.0).unwrap();

    let a = gyro_speed_error(low, TrueCourse::NORTH, slow).unwrap();
    let b = gyro_speed_error(high, TrueCourse::NORTH, slow).unwrap();
    let c = gyro_speed_error(low, TrueCourse::NORTH, fast).unwrap();
    assert!(b.degrees() < a.degrees());
    assert!(c.degrees() < a.degrees());

    // The gyro does not settle at the pole.
    assert!(gyro_speed_error(Latitude::NORTH_POLE, TrueCourse::NORTH, slow).is_err());
    assert!(gyro_speed_error(low, TrueCourse::NORTH, Speed::from_knots(-1.0).unwrap()).is_err());
}

#[test]
fn slack_water_leaves_the_current_direction_meaningless_but_defined() {
    let heading = TrueCourse::new(42.0).unwrap();
    let eight = Speed::from_knots(8.0).unwrap();
    let current = estimate_current(heading, eight, heading, eight).unwrap();
    assert_eq!(current.drift.knots(), 0.0);
    assert_eq!(current.set.degrees(), 0.0);
}

// -----------------------------------------------------------------------
// Through a compass model
// -----------------------------------------------------------------------

#[test]
fn a_model_conversion_agrees_with_the_table_conversion() {
    let table = readme_table();
    let variation = Variation::new(-2.7).unwrap();
    for method in [
        InterpolationMethod::Linear,
        InterpolationMethod::Cubic,
        InterpolationMethod::ShapePreserving,
        InterpolationMethod::Parametric,
    ] {
        let model = table.interpolated(method).unwrap();
        for degrees in [0.0, 47.3, 250.0, 359.9] {
            let compass = CompassCourse::new(degrees).unwrap();
            let by_table =
                convert_compass_course_to_true_course(compass, variation, &table, method).unwrap();
            let by_model = compass_to_true_by(compass, &model, variation).unwrap();
            assert!(by_model.angular_distance(by_table.course) < 1e-12);

            let back_by_table =
                convert_true_course_to_compass_course(by_model, variation, &table, method).unwrap();
            let back_by_model = true_to_compass_by(by_model, &model, variation).unwrap();
            assert!(back_by_model.angular_distance(back_by_table.course) < 1e-12);
            // The steep sector is not invertible under every method; the
            // inverse may find another root, which must still check out.
            let checked = compass_to_true_by(back_by_model, &model, variation).unwrap();
            assert!(checked.angular_distance(by_model) < 1e-9, "{method:?}");
        }
    }
}

#[test]
fn the_bare_table_and_the_coefficients_are_models_too() {
    let table = realistic_table();
    let coefficients = crate::deviation::smith_coefficients(&table).unwrap();
    let magnetic = MagneticCourse::new(123.4).unwrap();

    let via_table = magnetic_to_compass_by(magnetic, &table).unwrap();
    let via_fit = magnetic_to_compass_by(magnetic, &coefficients).unwrap();
    // The table was generated from a five-coefficient model, so the fit
    // reproduces it and both readings agree to interpolation error.
    assert!(via_table.angular_distance(via_fit) < 0.05);
    assert!(
        compass_to_magnetic_by(via_fit, &coefficients)
            .unwrap()
            .angular_distance(magnetic)
            < 1e-9
    );
}

/// Model failing on one sector, like a partial calibration.
struct Refusing;

impl CompassModel for Refusing {
    fn deviation(&self, course: CompassCourse) -> kinavis_kernel::Result<Deviation> {
        if (100.0..110.0).contains(&course.degrees()) {
            return Err(KernelError::OutsideValidity {
                data: "the calibration",
            });
        }
        Deviation::new(2.0)
    }
}

#[test]
fn a_refusal_from_the_model_is_passed_on_not_papered_over() {
    let fine = compass_to_true_by(
        CompassCourse::new(50.0).unwrap(),
        &Refusing,
        Variation::ZERO,
    )
    .unwrap();
    assert!((fine.degrees() - 52.0).abs() < 1e-12);

    assert!(matches!(
        compass_to_true_by(
            CompassCourse::new(105.0).unwrap(),
            &Refusing,
            Variation::ZERO
        ),
        Err(NavigationError::Kernel(KernelError::OutsideValidity { .. }))
    ));
    // The solver starts from the magnetic course, which falls in the gap.
    assert!(matches!(
        true_to_compass_by(TrueCourse::new(105.0).unwrap(), &Refusing, Variation::ZERO),
        Err(NavigationError::Kernel(KernelError::OutsideValidity { .. }))
    ));
}
