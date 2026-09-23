//! The public API never panics, whatever the input.
//!
//! Each case is an input that could abort or produce a nonsensical value. Kept
//! as an integration test so it exercises exactly the downstream-visible API.

// Tests may panic on a failed expectation; the library may not.
#![allow(clippy::expect_used, clippy::unwrap_used)]

use core::time::Duration;
use kinavis::anchor::{anchor_position, swinging_radius, AnchorWatch};
use kinavis::clearance::{
    height_of_tide_needed, squat, under_keel_clearance, ClearancePolicy, Hull, Waterway,
};
use kinavis::composite::composite_sailing;
use kinavis::dead_reckoning::{dead_reckoning, traverse, water_track, Leg};
use kinavis::fix::{
    bearing_fix, dipping_distance, distance_by_two_bearings, distance_by_vertical_angle,
    horizon_distance, two_range_fix, PositionLine,
};
use kinavis::guidance::{guide, GuidanceConfig};
use kinavis::mob::ManOverboard;
use kinavis::navigation_solutions::{
    calculate_course_angle, compass_to_magnetic, convert_compass_course_to_true_course,
    convert_magnetic_course_to_compass_course, convert_true_course_to_compass_course,
    course_over_ground, course_to_steer, estimate_current, magnetic_to_compass, magnetic_to_true,
    true_to_magnetic,
};
use kinavis::relative_motion::{
    bow_crossing_range, closest_point_of_approach, course_for_cpa, target_from_plot, Approach,
    Contact, Vessel,
};
use kinavis::route::{LegCursor, LegKind, Route};
use kinavis::sailings::{
    cross_track, geodesic, geodesic_destination, great_circle, great_circle_destination,
    great_circle_vertex, great_circle_waypoints_into, rhumb_destination, rhumb_intersection,
    rhumb_line,
};
use kinavis::schedule::RouteSchedule;
use kinavis::snapshot::GroundTrack;
use kinavis::turning::{TurnMode, TurnParameters};
use kinavis::{
    Angle, CompassCourse, Current, Deviation, DeviationCoefficients, DeviationTable, Distance,
    EnvironmentSample, GeodeticPoint, Height, Instant, Interpolation, InterpolationMethod,
    KernelError, Latitude, Longitude, MagneticCourse, NavigationError, NavigationSnapshot,
    ObservationStatus, Observed, Position, PositionSource, Quality, RelativeBearing, Speed,
    TrueBearing, TrueCourse, Variation, Wind, MAX_TABLE_NODES,
};

/// Values known to break floating-point code.
const HOSTILE: [f64; 12] = [
    f64::NAN,
    f64::INFINITY,
    f64::NEG_INFINITY,
    f64::MAX,
    f64::MIN,
    f64::MIN_POSITIVE,
    -0.0,
    0.0,
    -1e300,
    1e300,
    -360.000_000_1,
    360.000_000_1,
];

const METHODS: [InterpolationMethod; 3] = [
    InterpolationMethod::Linear,
    InterpolationMethod::Cubic,
    InterpolationMethod::Parametric,
];

fn sample_table() -> DeviationTable {
    DeviationTable::from_deviations(&[
        -2.5, -0.5, 1.6, 4.4, -1.7, 0.0, 1.0, 0.3, -0.9, 0.5, -1.2, 0.8, -0.3, 1.7, -2.1, 0.4,
        -0.6, 1.2, -1.3, 0.0, 0.9, -1.1, 1.5, -0.7, -13.2, -15.7, -17.9, -19.2, -18.1, 1.8, -0.4,
        0.7, -0.2, 1.4, -4.4, -2.9,
    ])
    .expect("36 values is a valid swing")
}

#[test]
fn table_constructors_reject_bad_input_without_panicking() {
    for step in [i32::MIN, -360, -1, 0, 181, 360, i32::MAX] {
        assert!(DeviationTable::from_step(step).is_err(), "step {step}");
    }
    for step in 1..=180 {
        // A step is rejected only for capacity, never by panicking; any
        // returned table is usable.
        match DeviationTable::from_step(step) {
            Ok(table) => assert!(table.len() >= 2 && table.len() <= MAX_TABLE_NODES),
            Err(error) => assert!(
                matches!(
                    error,
                    NavigationError::Kernel(KernelError::CapacityExceeded { .. })
                ) && 360 / step > 72,
                "step {step}: {error}"
            ),
        }
    }

    // An empty or single-node table has nothing to interpolate between.
    assert!(DeviationTable::from_pairs(&[]).is_err());
    assert!(DeviationTable::from_pairs(&[(0, 0.0)]).is_err());

    for value in HOSTILE {
        assert!(
            DeviationTable::from_pairs(&[(0, value), (180, 0.0)]).is_ok()
                || !value.is_finite()
                || value.abs() > 180.0
        );
    }
    let full = [0.0_f64; 100];
    for length in [0, 1, 35, 37, 100] {
        let offered = full.get(..length).expect("length is within the array");
        assert!(DeviationTable::from_deviations(offered).is_err());
    }
}

#[test]
fn extreme_course_keys_normalise_instead_of_wrapping_around() {
    for course in [i32::MIN, -100_000, -360, -1, 0, 359, 360, 100_000, i32::MAX] {
        let table = DeviationTable::from_pairs(&[(course, 1.0), (course.wrapping_add(180), 2.0)]);
        if let Ok(table) = table {
            for node in table.nodes() {
                assert!(
                    (0..360).contains(&node.course()),
                    "course {} escaped 0..360",
                    node.course()
                );
            }
        }
    }
}

#[test]
fn interpolation_survives_hostile_angles() {
    let table = sample_table();
    for method in METHODS {
        for angle in HOSTILE {
            // Hostile numbers are rejected when constructing the course;
            // anything that passes must be answerable.
            let Ok(course) = CompassCourse::new(angle) else {
                continue;
            };
            let single = table
                .deviation_at(course, method)
                .expect("a valid course is always answerable");
            assert!(single.degrees().is_finite());

            let mut batch = [Deviation::ZERO; 1];
            table
                .interpolate_deviation(&[course], method, &mut batch)
                .expect("a valid course is always answerable");
            // Bit-exact: the batch path must equal the single path.
            assert!((batch[0].degrees() - single.degrees()).abs() < f64::EPSILON);
        }

        // Dense sweep of the circle stays finite and in range.
        let mut angle = 0.0;
        while angle <= 360.0 {
            let value = table
                .deviation_at(CompassCourse::new(angle).expect("in-range angle"), method)
                .expect("in-range angle");
            assert!(value.degrees().is_finite(), "{method:?} at {angle}");
            angle += 0.125;
        }
    }
}

#[test]
fn interpolation_survives_hostile_coefficients() {
    let table = sample_table();
    for value in HOSTILE {
        let coefficients = DeviationCoefficients {
            a: Some(value),
            b: Some(value),
            c: Some(value),
            d: Some(value),
            e: Some(value),
        };
        let result = table.deviation_at(
            CompassCourse::new(123.0).expect("in-range course"),
            Interpolation {
                method: InterpolationMethod::Parametric,
                coefficients: Some(&coefficients),
            },
        );
        if let Ok(deviation) = result {
            assert!(deviation.degrees().is_finite());
        }
    }
}

#[test]
fn angle_constructors_never_produce_an_out_of_range_value() {
    for value in HOSTILE {
        // Each frame is a distinct type, so they cannot be collected into one
        // array — the purpose of the newtypes.
        let candidates = [
            TrueCourse::new(value).ok().map(TrueCourse::degrees),
            TrueCourse::wrap(value).ok().map(TrueCourse::degrees),
            MagneticCourse::new(value).ok().map(MagneticCourse::degrees),
            CompassCourse::wrap(value).ok().map(CompassCourse::degrees),
        ];
        for degrees in candidates.into_iter().flatten() {
            assert!(
                (0.0..360.0).contains(&degrees),
                "{value} produced {degrees}"
            );
        }

        if let Ok(variation) = Variation::new(value) {
            assert!(variation.degrees().is_finite());
            assert!(variation.degrees().abs() <= 180.0);
        }
        if let Ok(deviation) = Deviation::new(value) {
            assert!(deviation.degrees().abs() <= 180.0);
        }
    }
}

#[test]
fn corrections_always_land_inside_the_circle() {
    let variations: Vec<Variation> = [-180.0, -90.0, -0.0, 0.5, 90.0, 180.0]
        .into_iter()
        .filter_map(|value| Variation::new(value).ok())
        .collect();
    let deviations: Vec<Deviation> = [-180.0, -37.5, 0.0, 37.5, 180.0]
        .into_iter()
        .filter_map(|value| Deviation::new(value).ok())
        .collect();

    let mut degrees = 0.0;
    while degrees < 360.0 {
        let magnetic = MagneticCourse::wrap(degrees).expect("finite");
        let compass = CompassCourse::wrap(degrees).expect("finite");

        for &variation in &variations {
            for direction in [
                magnetic_to_true(magnetic, variation).degrees(),
                true_to_magnetic(TrueCourse::wrap(degrees).expect("finite"), variation).degrees(),
            ] {
                assert!((0.0..360.0).contains(&direction), "{direction}");
            }
        }
        for &deviation in &deviations {
            for direction in [
                compass_to_magnetic(compass, deviation).degrees(),
                magnetic_to_compass(magnetic, deviation).degrees(),
            ] {
                assert!((0.0..360.0).contains(&direction), "{direction}");
            }
        }

        let relative = calculate_course_angle(magnetic, MagneticCourse::NORTH);
        assert!((0.0..360.0).contains(&relative.degrees()));

        degrees += 0.5;
    }
}

#[test]
fn conversions_sweep_the_whole_circle_without_panicking() {
    let table = sample_table();
    let coefficients = DeviationCoefficients {
        a: Some(1.0),
        ..DeviationCoefficients::default()
    };

    for method in METHODS {
        for variation_degrees in [-180.0, -20.0, 0.0, 2.7, 180.0] {
            let variation = Variation::new(variation_degrees).expect("valid variation");
            let mut degrees = 0.0;
            while degrees < 360.0 {
                let compass = CompassCourse::wrap(degrees).expect("finite");
                let forward =
                    convert_compass_course_to_true_course(compass, variation, &table, method)
                        .expect("in-range conversion");
                assert!((0.0..360.0).contains(&forward.course.degrees()));
                assert!(forward.estimated_error.is_finite());

                let inverse = convert_true_course_to_compass_course(
                    forward.course,
                    variation,
                    &table,
                    method,
                )
                .expect("solvable inverse");
                assert!((0.0..360.0).contains(&inverse.course.degrees()));

                // The returned compass course must actually make good the
                // requested true course.
                let achieved = convert_compass_course_to_true_course(
                    inverse.course,
                    variation,
                    &table,
                    method,
                )
                .expect("in-range conversion");
                assert!(
                    achieved.course.angular_distance(forward.course) < 1e-6,
                    "{method:?} at {degrees}: asked {}, steering {} makes {}",
                    forward.course.degrees(),
                    inverse.course.degrees(),
                    achieved.course.degrees()
                );

                let pinned = convert_magnetic_course_to_compass_course(
                    MagneticCourse::wrap(degrees).expect("finite"),
                    &table,
                    Interpolation {
                        method,
                        coefficients: Some(&coefficients),
                    },
                )
                .expect("solvable inverse");
                assert!((0.0..360.0).contains(&pinned.course.degrees()));

                degrees += 1.0;
            }
        }
    }
}

#[test]
fn the_current_triangle_survives_hostile_speeds() {
    let north = TrueCourse::NORTH;
    let east = TrueCourse::EAST;

    for value in HOSTILE {
        // Non-finite speeds are rejected at construction.
        let Ok(speed) = Speed::from_knots(value) else {
            continue;
        };
        for other in [0.0, 1.0, 1e300] {
            let other = Speed::from_knots(other).expect("finite");
            let _ = course_over_ground(north, speed, east, other);
            let _ = course_over_ground(north, other, east, speed);
            let _ = course_to_steer(north, speed, east, other);
            let _ = course_to_steer(north, other, east, speed);
            let _ = estimate_current(north, speed, east, other);
            let _ = estimate_current(north, other, east, speed);
        }
    }
}

#[test]
fn a_solvable_current_triangle_is_always_consistent() {
    let mut heading_degrees = 0.0;
    while heading_degrees < 360.0 {
        let heading = TrueCourse::wrap(heading_degrees).expect("finite");
        let mut set_degrees = 0.0;
        while set_degrees < 360.0 {
            let set = TrueCourse::wrap(set_degrees).expect("finite");
            let through_water = Speed::from_knots(10.0).expect("finite");
            let drift = Speed::from_knots(3.0).expect("finite");
            if let Ok(track) = course_over_ground(heading, through_water, set, drift) {
                assert!(track.speed_over_ground.knots() > 0.0);
                assert!((0.0..360.0).contains(&track.course_over_ground.degrees()));

                let current = estimate_current(
                    heading,
                    through_water,
                    track.course_over_ground,
                    track.speed_over_ground,
                )
                .expect("valid speeds");
                assert!((current.drift.knots() - 3.0).abs() < 1e-9);
                assert!(current.set.angular_distance(set) < 1e-9);

                let steering = course_to_steer(track.course_over_ground, through_water, set, drift)
                    .expect("the ship outruns this current");
                assert!(steering.heading.angular_distance(heading) < 1e-9);
                assert!(steering.drift_angle.abs().degrees() <= 90.0);
            }
            set_degrees += 7.0;
        }
        heading_degrees += 7.0;
    }
}

// ---------------------------------------------------------------------------
// Positions and sailings
// ---------------------------------------------------------------------------

/// Latitudes and longitudes covering every edge case.
const AWKWARD: [(f64, f64); 12] = [
    (0.0, 0.0),
    (0.0, 180.0),
    (0.0, -180.0),
    (89.9999, 0.0),
    (-89.9999, 179.9999),
    (90.0, 0.0),
    (-90.0, 0.0),
    (45.0, -179.9999),
    (-45.0, 179.9999),
    (0.000_001, 0.000_001),
    (60.0, 90.0),
    (-60.0, -90.0),
];

#[test]
fn coordinates_reject_or_wrap_hostile_input() {
    for value in HOSTILE {
        if let Ok(latitude) = Latitude::from_degrees(value) {
            assert!((-90.0..=90.0).contains(&latitude.degrees()));
        }
        if let Ok(longitude) = Longitude::from_degrees(value) {
            assert!((-180.0..180.0).contains(&longitude.degrees()));
        }
        assert!(Distance::from_nautical_miles(value).is_ok() || !value.is_finite());
        assert!(Angle::from_degrees(value).is_ok() || !value.is_finite());
    }
    // Out-of-range latitudes are rejected, not clamped.
    assert!(Latitude::from_degrees(90.000_001).is_err());
    assert!(Latitude::from_degrees(-90.000_001).is_err());
}

#[test]
fn sailings_never_panic_between_awkward_positions() {
    for (from_latitude, from_longitude) in AWKWARD {
        let Ok(from) = Position::from_degrees(from_latitude, from_longitude) else {
            continue;
        };
        for (to_latitude, to_longitude) in AWKWARD {
            let Ok(to) = Position::from_degrees(to_latitude, to_longitude) else {
                continue;
            };

            if let Ok(sailing) = great_circle(from, to) {
                assert!(sailing.distance.nautical_miles().is_finite());
                assert!(sailing.distance.nautical_miles() >= 0.0);
                assert!((0.0..360.0).contains(&sailing.initial_course.degrees()));
                assert!((0.0..360.0).contains(&sailing.final_course.degrees()));
            }
            if let Ok(sailing) = rhumb_line(from, to) {
                assert!(sailing.distance.nautical_miles().is_finite());
                assert!(sailing.distance.nautical_miles() >= 0.0);
            }
            if let Ok(sailing) = geodesic(from, to) {
                assert!(sailing.distance.metres().is_finite());
                assert!(sailing.distance.metres() >= 0.0);
            }
            let _ = cross_track(from, to, from);
            let _ = rhumb_intersection(from, TrueCourse::NORTH, to, TrueCourse::EAST);
        }
    }
}

#[test]
fn destinations_stay_on_the_globe_however_far_they_run() {
    let distances = [0.0, 1e-9, 1.0, 5_400.0, 10_800.0, 1e6, 1e12];
    for (latitude, longitude) in AWKWARD {
        let Ok(from) = Position::from_degrees(latitude, longitude) else {
            continue;
        };
        let mut course = 0.0;
        while course < 360.0 {
            let bearing = TrueCourse::new(course).expect("in range");
            for miles in distances {
                let distance = Distance::from_nautical_miles(miles).expect("finite");
                if let Ok(arrival) = great_circle_destination(from, bearing, distance) {
                    assert_on_the_globe(arrival.position);
                }
                if let Ok(position) = rhumb_destination(from, bearing, distance) {
                    assert_on_the_globe(position);
                }
                if let Ok(arrival) = geodesic_destination(from, bearing, distance) {
                    assert_on_the_globe(arrival.position);
                }
            }
            let _ = great_circle_vertex(from, bearing);
            course += 37.0;
        }
    }
}

fn assert_on_the_globe(position: Position) {
    assert!(
        (-90.0..=90.0).contains(&position.latitude().degrees()),
        "latitude escaped: {}",
        position.latitude().degrees()
    );
    assert!(
        (-180.0..180.0).contains(&position.longitude().degrees()),
        "longitude escaped: {}",
        position.longitude().degrees()
    );
}

#[test]
fn waypoints_refuse_a_request_that_would_exhaust_memory() {
    let from = Position::from_degrees(0.0, 0.0).expect("valid");
    let to = Position::from_degrees(0.0, 90.0).expect("valid");
    let mut track = [from; 16];
    for interval in [0.0, -1.0, 1e-12, f64::MIN_POSITIVE] {
        let distance = Distance::from_nautical_miles(interval).expect("finite");
        assert!(
            great_circle_waypoints_into(from, to, distance, &mut track).is_err(),
            "{interval}"
        );
    }
    let sensible = Distance::from_nautical_miles(600.0).expect("finite");
    assert!(great_circle_waypoints_into(from, to, sensible, &mut track).is_ok());
}

// ---------------------------------------------------------------------------
// Dead reckoning
// ---------------------------------------------------------------------------

#[test]
fn dead_reckoning_survives_hostile_speeds_and_times() {
    let from = Position::from_degrees(45.0, 10.0).expect("valid");
    let course = TrueCourse::new(45.0).expect("valid");
    for value in HOSTILE {
        let Ok(speed) = Speed::from_knots(value) else {
            continue;
        };
        for seconds in [0, 1, 3600, 86_400, 86_400 * 365] {
            let _ = dead_reckoning(from, course, speed, Duration::from_secs(seconds));
        }
    }
    // Empty and very long traverses both behave.
    assert_eq!(traverse(from, &[]).expect("no legs"), from);
    let legs: Vec<Leg> = (0..360)
        .map(|degrees| Leg {
            course: TrueCourse::new(f64::from(degrees)).expect("in range"),
            distance: Distance::from_nautical_miles(1.0).expect("finite"),
        })
        .collect();
    assert!(traverse(from, &legs).is_ok());
}

#[test]
fn leeway_always_lands_on_the_compass() {
    let mut heading_degrees = 0.0;
    while heading_degrees < 360.0 {
        let heading = TrueCourse::new(heading_degrees).expect("in range");
        for leeway_degrees in [-720.0, -10.0, 0.0, 10.0, 720.0] {
            let leeway = Angle::from_degrees(leeway_degrees).expect("finite");
            let mut wind = 0.0;
            while wind < 360.0 {
                let track = water_track(heading, leeway, TrueCourse::new(wind).expect("in range"));
                assert!((0.0..360.0).contains(&track.degrees()));
                wind += 45.0;
            }
        }
        heading_degrees += 45.0;
    }
}

// ---------------------------------------------------------------------------
// Fixes
// ---------------------------------------------------------------------------

#[test]
fn fixes_survive_degenerate_geometry() {
    let object = Position::from_degrees(50.0, -4.0).expect("valid");
    let same = PositionLine::new(object, TrueBearing::new(90.0).expect("valid"));
    let reciprocal = PositionLine::new(object, TrueBearing::new(270.0).expect("valid"));

    // Coincident, parallel and reciprocal lines are reported, not solved.
    assert!(bearing_fix(&[same, same]).is_err());
    assert!(bearing_fix(&[same, reciprocal]).is_err());

    // Many lines through one point are fine.
    let lines: Vec<PositionLine> = (0..36)
        .map(|step| {
            PositionLine::new(
                object,
                TrueBearing::new(f64::from(step) * 10.0).expect("in range"),
            )
        })
        .collect();
    let fix = bearing_fix(&lines).expect("a pencil of lines through one point");
    assert!(fix.rms_residual.nautical_miles().is_finite());
}

#[test]
fn distance_off_methods_refuse_nonsense_without_panicking() {
    for value in HOSTILE {
        let Ok(height) = Distance::from_nautical_miles(value) else {
            continue;
        };
        let Ok(angle) = Angle::from_degrees(value) else {
            continue;
        };
        let _ = distance_by_vertical_angle(height, angle);
        let _ = horizon_distance(height);
        let _ = dipping_distance(height, height);
    }

    let run = Distance::from_nautical_miles(4.0).expect("finite");
    let mut first = 0.0;
    while first < 360.0 {
        let mut second = 0.0;
        while second < 360.0 {
            let result = distance_by_two_bearings(
                RelativeBearing::new(first).expect("in range"),
                RelativeBearing::new(second).expect("in range"),
                run,
            );
            if let Ok(distance) = result {
                assert!(distance.abeam.nautical_miles().is_finite());
                assert!(distance.at_second_bearing.nautical_miles() >= 0.0);
            }
            second += 15.0;
        }
        first += 15.0;
    }
}

#[test]
fn two_range_fixes_never_invent_a_position() {
    let first = Position::from_degrees(50.0, -4.0).expect("valid");
    let second = Position::from_degrees(50.0, -4.5).expect("valid");
    for first_miles in [0.0, 1.0, 10.0, 100.0, 1e6] {
        for second_miles in [0.0, 1.0, 10.0, 100.0, 1e6] {
            let result = two_range_fix(
                first,
                Distance::from_nautical_miles(first_miles).expect("finite"),
                second,
                Distance::from_nautical_miles(second_miles).expect("finite"),
                first,
            );
            if let Ok(position) = result {
                assert_on_the_globe(position);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Relative motion
// ---------------------------------------------------------------------------

#[test]
fn relative_motion_sweeps_every_geometry_without_panicking() {
    let own = Vessel {
        course: TrueCourse::NORTH,
        speed: Speed::from_knots(15.0).expect("finite"),
    };

    let mut bearing = 0.0;
    while bearing < 360.0 {
        let contact = Contact {
            bearing: TrueBearing::new(bearing).expect("in range"),
            range: Distance::from_nautical_miles(8.0).expect("finite"),
        };
        let mut course = 0.0;
        while course < 360.0 {
            for knots in [0.0, 3.0, 15.0, 40.0] {
                let target = Vessel {
                    course: TrueCourse::new(course).expect("in range"),
                    speed: Speed::from_knots(knots).expect("finite"),
                };

                if let Ok(Approach::Closing(cpa)) = closest_point_of_approach(own, contact, target)
                {
                    assert!(cpa.distance.nautical_miles() >= 0.0);
                    assert!(cpa.distance.nautical_miles() <= 8.0 + 1e-9);
                }
                let _ = bow_crossing_range(own, contact, target);

                // Every offered avoidance course must achieve the distance.
                let wanted = Distance::from_nautical_miles(2.0).expect("finite");
                if let Ok(avoidance) = course_for_cpa(own, contact, target, wanted) {
                    for altered in [avoidance.starboard, avoidance.port].into_iter().flatten() {
                        let steered = Vessel {
                            course: altered,
                            speed: own.speed,
                        };
                        if let Ok(Approach::Closing(cpa)) =
                            closest_point_of_approach(steered, contact, target)
                        {
                            assert!(
                                cpa.distance.nautical_miles() >= 2.0 - 1e-6,
                                "bearing {bearing}, target {course} at {knots}: CPA {}",
                                cpa.distance.nautical_miles()
                            );
                        }
                    }
                }
            }
            course += 30.0;
        }
        bearing += 30.0;
    }
}

#[test]
fn a_plot_over_any_interval_stays_finite() {
    let own = Vessel {
        course: TrueCourse::new(35.0).expect("valid"),
        speed: Speed::from_knots(12.0).expect("finite"),
    };
    let first = Contact {
        bearing: TrueBearing::new(80.0).expect("valid"),
        range: Distance::from_nautical_miles(9.0).expect("finite"),
    };
    for seconds in [0, 1, 60, 3600, 86_400] {
        for range in [0.0, 0.001, 9.0, 1e6] {
            let second = Contact {
                bearing: TrueBearing::new(81.0).expect("valid"),
                range: Distance::from_nautical_miles(range).expect("finite"),
            };
            if let Ok(solution) = target_from_plot(own, first, second, Duration::from_secs(seconds))
            {
                assert!(solution.vessel.speed.knots().is_finite());
                assert!((0.0..360.0).contains(&solution.vessel.course.degrees()));
                assert!(solution.aspect.degrees().abs() <= 180.0);
            }
        }
    }
}

#[test]
fn guidance_never_panics_anywhere_on_the_globe() {
    let waypoints = AWKWARD
        .iter()
        .filter_map(|&(latitude, longitude)| Position::from_degrees(latitude, longitude).ok())
        .collect::<Vec<_>>();
    let config = GuidanceConfig::new(
        Distance::from_nautical_miles(1.0).unwrap(),
        Distance::from_cables(5.0).unwrap(),
    )
    .unwrap()
    .with_leeway(Angle::from_degrees(3.0).unwrap())
    .anticipating_turns(
        TurnParameters::new(
            TurnMode::Radius(Distance::from_cables(5.0).unwrap()),
            Distance::from_cables(6.0).unwrap(),
            Distance::from_cables(4.0).unwrap(),
        )
        .unwrap(),
    );
    let when = Instant::from_unix_seconds(1_789_000_000);

    for kind in [LegKind::RhumbLine, LegKind::GreatCircle] {
        let route = Route::new(&waypoints, kind).unwrap();
        for &here in &waypoints {
            let environment = EnvironmentSample::at(
                GeodeticPoint::new(here, Height::above_ellipsoid(Distance::ZERO)),
                when,
            )
            .with_current(Current {
                set: TrueCourse::EAST,
                drift: Speed::from_knots(3.0).unwrap(),
            });
            for knots in [0.0, 1.0, 15.0, 1e6] {
                let state = NavigationSnapshot::EMPTY
                    .with_position(
                        Observed::new(
                            here,
                            when,
                            Quality::<Distance>::new(ObservationStatus::Valid),
                        ),
                        PositionSource::Gnss,
                    )
                    .with_ground_track(GroundTrack {
                        course_over_ground: TrueCourse::NORTH,
                        speed_over_ground: Speed::from_knots(knots).unwrap(),
                    });
                // Every leg, and the one past the end, which the route rejects.
                for active in (0..=route.leg_count())
                    .filter_map(|index| LegCursor::try_from(u16::try_from(index).ok()?).ok())
                {
                    if let Ok((view, events)) = guide(&state, &route, active, &environment, &config)
                    {
                        assert!((0.0..360.0).contains(&view.desired_track().degrees()));
                        assert!((0.0..360.0).contains(&view.course_to_steer().degrees()));
                        assert!(view
                            .cross_track_error()
                            .distance
                            .nautical_miles()
                            .is_finite());
                        assert!(view.distance_to_waypoint().nautical_miles() >= 0.0);
                        assert!(view.distance_to_end().nautical_miles() >= 0.0);
                        assert!(!events.overflowed());
                    }
                }
            }
        }
    }
}

#[test]
fn schedules_never_panic_anywhere_on_the_globe() {
    let waypoints = AWKWARD
        .iter()
        .filter_map(|&(latitude, longitude)| Position::from_degrees(latitude, longitude).ok())
        .collect::<Vec<_>>();
    let config = GuidanceConfig::new(
        Distance::from_nautical_miles(1.0).unwrap(),
        Distance::from_cables(5.0).unwrap(),
    )
    .unwrap();
    let departure = Instant::from_unix_seconds(1_789_000_000);

    for kind in [LegKind::RhumbLine, LegKind::GreatCircle] {
        let route = Route::new(&waypoints, kind).unwrap();
        for planned in [1e-300, 1e-6, 12.0, 1e6, f64::MAX] {
            // Leg speeds are validated on input; an unrepresentable schedule is
            // rejected.
            let Ok(schedule) =
                RouteSchedule::new(&route, departure, Speed::from_knots(planned).unwrap())
            else {
                continue;
            };
            assert!(schedule.arrival() >= departure);
            assert_eq!(schedule.leg_count(), route.leg_count());
            for waypoint in 0..=u16::MAX {
                if schedule.eta(waypoint).is_none() {
                    break;
                }
            }

            for &here in &waypoints {
                let environment = EnvironmentSample::at(
                    GeodeticPoint::new(here, Height::above_ellipsoid(Distance::ZERO)),
                    departure,
                );
                for when in [
                    Instant::from_unix_seconds(i64::MIN),
                    departure,
                    schedule.arrival(),
                    Instant::from_unix_seconds(i64::MAX),
                ] {
                    for knots in [0.0, 1e-9, 15.0, 1e6, f64::MAX] {
                        let state = NavigationSnapshot::EMPTY
                            .with_position(
                                Observed::new(
                                    here,
                                    when,
                                    Quality::<Distance>::new(ObservationStatus::Valid),
                                ),
                                PositionSource::Gnss,
                            )
                            .with_ground_track(GroundTrack {
                                course_over_ground: TrueCourse::EAST,
                                speed_over_ground: Speed::from_knots(knots).unwrap(),
                            });
                        for active in (0..=route.leg_count()).filter_map(|index| {
                            LegCursor::try_from(u16::try_from(index).ok()?).ok()
                        }) {
                            let Ok((view, _)) =
                                guide(&state, &route, active, &environment, &config)
                            else {
                                continue;
                            };
                            if let Ok(estimate) = schedule.estimate(&state, &view) {
                                assert!(estimate.distance_to_go().nautical_miles() >= 0.0);
                                if let Some(speed) = estimate.speed_made_good() {
                                    assert!(speed.knots().is_finite());
                                }
                                assert!(estimate.eta_at_planned_speed() >= estimate.at());
                                // Cannot be both ahead and behind.
                                assert!(
                                    estimate.ahead_of_schedule().is_none()
                                        || estimate.behind_schedule().is_none()
                                );
                            }
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn under_keel_clearance_never_panics_on_hostile_numbers() {
    let when = Instant::from_unix_seconds(1_789_000_000);
    let here = Position::from_degrees(51.0, 3.0).unwrap();
    let point = GeodeticPoint::new(here, Height::above_ellipsoid(Distance::ZERO));
    let hulls = HOSTILE
        .iter()
        .flat_map(|&draught| HOSTILE.iter().map(move |&cb| (draught, cb)))
        .filter_map(|(draught, cb)| Hull::new(Distance::from_metres(draught).ok()?, cb).ok())
        .chain([Hull::new(Distance::from_metres(12.0).unwrap(), 0.85).unwrap()])
        .collect::<Vec<_>>();
    assert!(!hulls.is_empty());

    for hull in &hulls {
        for &knots in &HOSTILE {
            let Ok(speed) = Speed::from_knots(knots) else {
                continue;
            };
            for waterway in [
                Waterway::OpenWater,
                Waterway::ConfinedChannel,
                Waterway::Blockage(0.0),
                Waterway::Blockage(0.15),
                Waterway::Blockage(f64::NAN),
                Waterway::Blockage(f64::MAX),
            ] {
                if let Ok(sinkage) = squat(hull, speed, waterway) {
                    assert!(sinkage.metres() >= 0.0);
                }
                for &depth in &HOSTILE {
                    let Ok(charted) = Distance::from_metres(depth) else {
                        continue;
                    };
                    for &tide in &HOSTILE {
                        let environment = match Distance::from_metres(tide) {
                            Ok(height) => EnvironmentSample::at(point, when).with_tide(height),
                            Err(_) => EnvironmentSample::at(point, when),
                        };
                        for &fraction in &HOSTILE {
                            let policy = ClearancePolicy {
                                minimum: Distance::from_metres(fraction).unwrap_or(Distance::ZERO),
                                fraction_of_draught: fraction,
                            };
                            let _ = height_of_tide_needed(hull, speed, waterway, charted, &policy);
                            if let Ok((clearance, events)) = under_keel_clearance(
                                hull,
                                speed,
                                waterway,
                                charted,
                                &environment,
                                &policy,
                            ) {
                                assert_eq!(!clearance.is_sufficient(), !events.is_empty());
                                assert!(!events.overflowed());
                            }
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn composite_sailing_never_panics_anywhere_on_the_globe() {
    let positions = AWKWARD
        .iter()
        .filter_map(|&(latitude, longitude)| Position::from_degrees(latitude, longitude).ok())
        .collect::<Vec<_>>();
    let limits = HOSTILE
        .iter()
        .chain(&[45.0, -45.0, 89.0, 0.5])
        .filter_map(|&degrees| Latitude::from_degrees(degrees).ok());
    for limit in limits {
        for &from in &positions {
            for &to in &positions {
                if let Ok(track) = composite_sailing(from, to, limit) {
                    let total = track.total_distance().nautical_miles();
                    assert!(total.is_finite() && total >= 0.0);
                    assert!(track.extra_distance().nautical_miles() >= -1e-6);
                    if let Some(run) = track.parallel_run() {
                        assert!(
                            (run.first_vertex.latitude().degrees() - limit.degrees()).abs() < 1e-9
                        );
                    }
                    if let Ok(route) = track.route(Distance::from_nautical_miles(500.0).unwrap()) {
                        for waypoint in route.waypoints() {
                            let beyond = if limit.degrees() < 0.0 {
                                waypoint.latitude().degrees() < limit.degrees() - 1e-6
                            } else {
                                waypoint.latitude().degrees() > limit.degrees() + 1e-6
                            };
                            assert!(!beyond, "{waypoint} beyond {limit}");
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn the_anchor_watch_and_the_mob_datum_never_panic_on_hostile_numbers() {
    let when = Instant::from_unix_seconds(1_789_000_000);
    let positions = AWKWARD
        .iter()
        .filter_map(|&(latitude, longitude)| Position::from_degrees(latitude, longitude).ok())
        .collect::<Vec<_>>();
    let lengths = HOSTILE
        .iter()
        .filter_map(|&metres| Distance::from_metres(metres).ok())
        .collect::<Vec<_>>();

    for &here in &positions {
        for &cable in &lengths {
            for &depth in &lengths {
                let _ = swinging_radius(cable, depth, Distance::ZERO);
            }
            let _ = anchor_position(here, TrueCourse::NORTH, cable);
            let Ok(watch) = AnchorWatch::new(here, cable) else {
                continue;
            };
            for &there in &positions {
                let state = NavigationSnapshot::EMPTY.with_position(
                    Observed::new(
                        there,
                        when,
                        Quality::<Distance>::new(ObservationStatus::Valid),
                    ),
                    PositionSource::Gnss,
                );
                if let Ok((view, events)) = watch.check(&state) {
                    assert!(view.distance_from_anchor().nautical_miles() >= 0.0);
                    assert_eq!(view.is_dragging(), !events.is_empty());
                }
            }
        }

        let mob = ManOverboard::new(here, when);
        for &knots in &HOSTILE {
            let Ok(speed) = Speed::from_knots(knots) else {
                continue;
            };
            let point = GeodeticPoint::new(here, Height::above_ellipsoid(Distance::ZERO));
            let mut environment = EnvironmentSample::at(point, when).with_current(Current {
                set: TrueCourse::EAST,
                drift: speed,
            });
            if let Ok(wind) = Wind::new(TrueCourse::NORTH, speed) {
                environment = environment.with_wind(wind);
            }
            for seconds in [0, 1, 3600, 86_400 * 365, i64::MAX / 4] {
                let now = Instant::from_unix_seconds(1_789_000_000_i64.saturating_add(seconds));
                for factor in [0.0, 0.02, 1.0, f64::NAN] {
                    if let Ok(datum) = mob.datum(&environment, now, factor) {
                        assert!(datum.drift().nautical_miles() >= 0.0);
                        assert_eq!(datum.set().is_none(), datum.drift().nautical_miles() == 0.0);
                    }
                }
            }
        }
    }
}
