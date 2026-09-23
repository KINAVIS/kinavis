//! Error filter: correction sign, bias estimation, gating, covariance validity.

#![allow(clippy::unwrap_used, clippy::float_cmp, clippy::indexing_slicing)]

use core::time::Duration;

use kinavis_ins::{
    gravity_down, GatingPolicy, ImuNoise, ImuSample, InsFilter, InsPriors, Quaternion, Strapdown,
    EARTH_RATE,
};
use kinavis_kernel::{
    Angle, Distance, GeodeticPoint, Height, Instant, Ned, Position, Speed, TrueCourse, Utc, Vector3,
};

const LATITUDE: f64 = 50.0;
const DT: Duration = Duration::from_millis(100);

fn origin() -> GeodeticPoint {
    GeodeticPoint::new(
        Position::from_degrees(LATITUDE, -1.0).unwrap(),
        Height::above_ellipsoid(Distance::ZERO),
    )
}

fn level(yaw_degrees: f64) -> Quaternion {
    Quaternion::from_euler(
        Angle::ZERO,
        Angle::ZERO,
        TrueCourse::new(yaw_degrees).unwrap(),
    )
}

fn still() -> Vector3<Ned, Speed> {
    Vector3::new(Speed::ZERO, Speed::ZERO, Speed::ZERO)
}

fn filter(yaw_degrees: f64) -> InsFilter {
    let nominal = Strapdown::new(
        Instant::<Utc>::from_unix_seconds(1_789_000_000),
        origin(),
        still(),
        level(yaw_degrees),
    )
    .unwrap();
    InsFilter::new(nominal, ImuNoise::mems(), &InsPriors::standard())
}

/// IMU reading at rest at this attitude, with the given biases.
fn at_rest(attitude: &Quaternion, gyro_bias: [f64; 3], accel_bias: [f64; 3]) -> ImuSample {
    let latitude = LATITUDE.to_radians();
    let earth = [
        EARTH_RATE * latitude.cos(),
        0.0,
        -EARTH_RATE * latitude.sin(),
    ];
    let rate = attitude.rotate_back(earth);
    let force = attitude.rotate_back([0.0, 0.0, -gravity_down(latitude.sin(), 0.0)]);
    ImuSample::new(
        [
            rate[0] + gyro_bias[0],
            rate[1] + gyro_bias[1],
            rate[2] + gyro_bias[2],
        ],
        [
            force[0] + accel_bias[0],
            force[1] + accel_bias[1],
            force[2] + accel_bias[2],
        ],
        DT,
    )
    .unwrap()
}

#[test]
fn a_heading_observation_moves_the_heading_towards_it() {
    let mut filter = filter(45.0);
    let update = filter
        .update_heading(
            TrueCourse::new(44.0).unwrap(),
            Angle::from_degrees(1.0).unwrap(),
            GatingPolicy::none(),
        )
        .unwrap();
    assert!(update.accepted);
    assert_eq!(update.dof, 1);
    let yaw = filter.attitude().yaw.degrees();
    assert!(yaw < 45.0 && yaw > 44.0, "{yaw}");
    // Prior 3°, observation 1°: posterior 9/10 of the way to the observation.
    assert!((yaw - 44.1).abs() < 0.01, "{yaw}");
    assert!(filter.heading_sigma().degrees() < 1.0);

    // Across north: 359° observed vs 1° estimated is −2°.
    let mut filter = filter_at(1.0);
    filter
        .update_heading(
            TrueCourse::new(359.0).unwrap(),
            Angle::from_degrees(1.0).unwrap(),
            GatingPolicy::none(),
        )
        .unwrap();
    let yaw = filter.attitude().yaw.degrees();
    assert!((yaw - 359.2).abs() < 0.01, "{yaw}");
}

fn filter_at(yaw_degrees: f64) -> InsFilter {
    filter(yaw_degrees)
}

#[test]
fn a_position_fix_pulls_the_estimate_and_shrinks_the_ellipse() {
    let mut filter = filter(0.0);
    let before = filter.horizontal_error().semi_major().metres();
    assert!((before - 10.0).abs() < 1e-9);
    // Fix 8 m north of the origin, σ 5 m: posterior 8 × 100/125.
    let north = GeodeticPoint::new(
        Position::from_degrees(LATITUDE + 8.0 / 111_320.0 * (1.0 - 0.0033), -1.0).unwrap(),
        Height::above_ellipsoid(Distance::ZERO),
    );
    let measured = filter
        .nominal()
        .frame()
        .ned_of(north)
        .unwrap()
        .north()
        .metres();
    let update = filter
        .update_position(
            north.position(),
            Distance::from_metres(5.0).unwrap(),
            GatingPolicy::none(),
        )
        .unwrap();
    assert!(update.accepted);
    assert_eq!(update.dof, 2);
    let moved = filter.nominal().displacement().north().metres();
    assert!(
        (moved - measured * 0.8).abs() < 0.01,
        "{moved} vs {measured}"
    );
    let after = filter.horizontal_error().semi_major().metres();
    assert!(
        (after - (1.0_f64 / (1.0 / 100.0 + 1.0 / 25.0)).sqrt()).abs() < 1e-6,
        "{after}"
    );
    assert!(
        filter.height_sigma().metres() > 19.0,
        "the height was not observed"
    );
}

#[test]
fn the_gate_refuses_an_improbable_fix_and_says_so() {
    let mut filter = filter(0.0);
    // 100 m off, prior 10 m, σ 5 m: NIS 80 on 2 dof.
    let far = Position::from_degrees(LATITUDE + 100.0 / 111_000.0, -1.0).unwrap();
    let update = filter
        .update_position(
            far,
            Distance::from_metres(5.0).unwrap(),
            GatingPolicy::reject_above(9.21),
        )
        .unwrap();
    assert!(!update.accepted);
    assert!(update.nis > 50.0, "{}", update.nis);
    assert!(
        filter.nominal().displacement().magnitude().metres() < 1e-9,
        "untouched"
    );
    assert!((filter.horizontal_error().semi_major().metres() - 10.0).abs() < 1e-9);
    // Without a gate the same fix is applied.
    assert!(
        filter
            .update_position(
                far,
                Distance::from_metres(5.0).unwrap(),
                GatingPolicy::none()
            )
            .unwrap()
            .accepted
    );
}

#[test]
fn a_gyro_bias_is_found_at_rest_with_a_compass_and_no_motion() {
    // 0.1°/s gyro bias about down: unaided, 30° of heading error after 5 min.
    // With heading and zero-velocity updates every second the bias is estimated
    // and the heading holds.
    let bias = [0.0, 0.0, 0.1_f64.to_radians()];
    let attitude = level(120.0);
    let mut filter = filter(120.0);
    let sample = at_rest(&attitude, bias, [0.0; 3]);
    for step in 0..3_000 {
        filter.predict(&sample).unwrap();
        if step % 10 == 9 {
            filter
                .update_heading(
                    TrueCourse::new(120.0).unwrap(),
                    Angle::from_degrees(0.5).unwrap(),
                    GatingPolicy::none(),
                )
                .unwrap();
            filter
                .update_zero_velocity(
                    Speed::from_metres_per_second(0.02).unwrap(),
                    GatingPolicy::none(),
                )
                .unwrap();
        }
    }
    let estimated = filter.nominal().gyro_bias();
    assert!(
        (estimated[2] - bias[2]).abs() < 0.1 * bias[2],
        "bias {estimated:?} vs {bias:?}"
    );
    let attitude = filter.attitude();
    assert!((attitude.yaw.degrees() - 120.0).abs() < 0.2, "{attitude}");
    assert!(attitude.roll.degrees().abs() < 0.05, "{attitude}");
    assert!(attitude.pitch.degrees().abs() < 0.05, "{attitude}");
    assert!(filter.velocity().magnitude().metres_per_second() < 0.05);
    assert!(filter.covariance().is_covariance());
    assert_eq!(
        filter.valid_at(),
        Instant::<Utc>::from_unix_seconds(1_789_000_000 + 300)
    );
}

#[test]
fn a_tilt_error_is_levelled_by_gravity_and_stillness() {
    // Initialised 1° off level: the accelerometers see a gravity component,
    // velocity grows, and zero-velocity updates turn it into a tilt correction.
    let truth = level(0.0);
    let wrong = Quaternion::from_euler(
        Angle::from_degrees(1.0).unwrap(),
        Angle::from_degrees(-0.5).unwrap(),
        TrueCourse::NORTH,
    );
    let nominal = Strapdown::new(
        Instant::<Utc>::from_unix_seconds(0),
        origin(),
        still(),
        wrong,
    )
    .unwrap();
    let mut filter = InsFilter::new(nominal, ImuNoise::mems(), &InsPriors::standard());
    let sample = at_rest(&truth, [0.0; 3], [0.0; 3]);
    for step in 0..1_200 {
        filter.predict(&sample).unwrap();
        if step % 10 == 9 {
            filter
                .update_zero_velocity(
                    Speed::from_metres_per_second(0.02).unwrap(),
                    GatingPolicy::none(),
                )
                .unwrap();
        }
    }
    let attitude = filter.attitude();
    assert!(attitude.roll.degrees().abs() < 0.05, "{attitude}");
    assert!(attitude.pitch.degrees().abs() < 0.05, "{attitude}");
    // At rest, tilt and accelerometer bias are indistinguishable, so the tilt
    // sigma cannot fall below bias prior / g: 2 mg ≈ 0.11°.
    assert!(filter.tilt_sigma().degrees() < 0.2);
    assert!(filter.tilt_sigma().degrees() > 0.05);
}

#[test]
fn a_velocity_update_is_three_dimensional() {
    let mut filter = filter(0.0);
    let update = filter
        .update_velocity(
            Vector3::new(
                Speed::from_metres_per_second(1.0).unwrap(),
                Speed::ZERO,
                Speed::ZERO,
            ),
            Speed::from_metres_per_second(0.1).unwrap(),
            GatingPolicy::none(),
        )
        .unwrap();
    assert_eq!(update.dof, 3);
    // Prior 0.5, σ 0.1: 25/26 of the way.
    let north = filter.velocity().north().metres_per_second();
    assert!((north - 25.0 / 26.0).abs() < 1e-6, "{north}");
    assert!(filter.velocity_sigma().metres_per_second() < 0.1);

    let update = filter
        .update_point(
            origin(),
            Distance::from_metres(5.0).unwrap(),
            Distance::from_metres(10.0).unwrap(),
            GatingPolicy::none(),
        )
        .unwrap();
    assert_eq!(update.dof, 3);
    assert!(filter.height_sigma().metres() < 10.0);
}

#[test]
fn sigmas_that_are_not_positive_are_refused() {
    let mut filter = filter(0.0);
    assert!(filter
        .update_position(origin().position(), Distance::ZERO, GatingPolicy::none())
        .is_err());
    assert!(filter
        .update_velocity(still(), Speed::ZERO, GatingPolicy::none())
        .is_err());
    assert!(filter
        .update_heading(TrueCourse::NORTH, Angle::ZERO, GatingPolicy::none())
        .is_err());
    assert!(InsPriors::new(
        Distance::ZERO,
        Distance::from_metres(1.0).unwrap(),
        Speed::from_metres_per_second(1.0).unwrap(),
        Angle::from_degrees(1.0).unwrap(),
        Angle::from_degrees(1.0).unwrap(),
        1e-3,
        1e-2
    )
    .is_err());
    assert!(InsPriors::new(
        Distance::from_metres(1.0).unwrap(),
        Distance::from_metres(1.0).unwrap(),
        Speed::from_metres_per_second(1.0).unwrap(),
        Angle::from_degrees(1.0).unwrap(),
        Angle::from_degrees(1.0).unwrap(),
        1e-3,
        1e-2
    )
    .is_ok());
}
