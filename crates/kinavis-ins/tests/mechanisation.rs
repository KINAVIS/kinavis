//! Mechanisation against motions with known solutions: at rest, steady turn,
//! constant acceleration. IMU readings are synthesised from the true motion
//! (Earth rate in the body frame, gravity, Coriolis) and the integrated result
//! is compared with the truth.

#![allow(clippy::unwrap_used, clippy::float_cmp, clippy::indexing_slicing)]

use core::time::Duration;

use kinavis_ins::{gravity_down, ImuSample, Quaternion, Strapdown, EARTH_RATE};
use kinavis_kernel::{
    Angle, Distance, GeodeticPoint, Height, Instant, Ned, Position, Speed, TrueCourse, Utc, Vector3,
};

const LATITUDE: f64 = 50.0;
const DT: Duration = Duration::from_millis(10);

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

fn at_rest(yaw_degrees: f64) -> Strapdown {
    Strapdown::new(
        Instant::<Utc>::from_unix_seconds(1_789_000_000),
        origin(),
        Vector3::<Ned, Speed>::new(Speed::ZERO, Speed::ZERO, Speed::ZERO),
        level(yaw_degrees),
    )
    .unwrap()
}

/// Earth rate in the navigation frame at the test latitude.
fn earth_rate() -> [f64; 3] {
    let latitude = LATITUDE.to_radians();
    [
        EARTH_RATE * latitude.cos(),
        0.0,
        -EARTH_RATE * latitude.sin(),
    ]
}

fn gravity() -> f64 {
    gravity_down(LATITUDE.to_radians().sin(), 0.0)
}

fn add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// Transport rate on a sphere; close enough to the ellipsoid for these
/// tolerances.
fn transport_rate(velocity: [f64; 3]) -> [f64; 3] {
    const RADIUS: f64 = 6_371_000.0;
    [
        velocity[1] / RADIUS,
        -velocity[0] / RADIUS,
        -velocity[1] * LATITUDE.to_radians().tan() / RADIUS,
    ]
}

/// IMU reading for a body with this attitude, velocity, acceleration and
/// rotation rate relative to the navigation frame: Earth rate and transport
/// rate in the body frame, specific force `a − g + (2ω_ie + ω_en) × v`.
fn reading(
    attitude: &Quaternion,
    velocity: [f64; 3],
    acceleration: [f64; 3],
    body_rate: [f64; 3],
) -> ImuSample {
    let earth = earth_rate();
    let transport = transport_rate(velocity);
    let angular_rate = add(attitude.rotate_back(add(earth, transport)), body_rate);
    let coriolis = cross(
        add([2.0 * earth[0], 0.0, 2.0 * earth[2]], transport),
        velocity,
    );
    let force_n = add(add(acceleration, [0.0, 0.0, -gravity()]), coriolis);
    ImuSample::new(angular_rate, attitude.rotate_back(force_n), DT).unwrap()
}

#[test]
fn at_rest_nothing_moves_for_ten_minutes() {
    let mut ins = at_rest(37.0);
    let sample = reading(&level(37.0), [0.0; 3], [0.0; 3], [0.0; 3]);
    for _ in 0..60_000 {
        ins.step(&sample).unwrap();
    }
    let velocity = ins.velocity();
    assert!(
        velocity.magnitude().metres_per_second() < 1e-6,
        "{velocity:?}"
    );
    assert!(ins.displacement().magnitude().metres() < 1e-3);
    let attitude = ins.attitude();
    assert!(attitude.roll.degrees().abs() < 1e-7);
    assert!(attitude.pitch.degrees().abs() < 1e-7);
    assert!((attitude.yaw.degrees() - 37.0).abs() < 1e-7);
    assert_eq!(
        ins.valid_at(),
        Instant::<Utc>::from_unix_seconds(1_789_000_000 + 600)
    );
}

#[test]
fn a_steady_turn_integrates_to_the_angle_turned() {
    // 1°/s to starboard for 90 s, from 350°.
    let rate = 1.0_f64.to_radians();
    let mut ins = at_rest(350.0);
    let mut truth = level(350.0);
    for _ in 0..9_000 {
        let sample = reading(&truth, [0.0; 3], [0.0; 3], [0.0, 0.0, rate]);
        ins.step(&sample).unwrap();
        truth = truth.rotated_by_body([0.0, 0.0, rate * DT.as_secs_f64()]);
    }
    let attitude = ins.attitude();
    assert!((attitude.yaw.degrees() - 80.0).abs() < 1e-4, "{attitude}");
    assert!(attitude.roll.degrees().abs() < 1e-4);
    assert!(ins.quaternion().angle_to(&truth).to_degrees() < 1e-4);
    assert!(ins.velocity().magnitude().metres_per_second() < 1e-4);
}

#[test]
fn a_steady_acceleration_east_makes_the_velocity_and_distance_it_should() {
    // Heading east, 0.1 m/s² for 60 s: 6 m/s and 180 m.
    let mut ins = at_rest(90.0);
    let attitude = level(90.0);
    let mut velocity = [0.0; 3];
    let acceleration = [0.0, 0.1, 0.0];
    for _ in 0..6_000 {
        let sample = reading(&attitude, velocity, acceleration, [0.0; 3]);
        ins.step(&sample).unwrap();
        velocity = add(velocity, [0.0, 0.1 * DT.as_secs_f64(), 0.0]);
    }
    // The sphere used for the test transport rate differs from the ellipsoid by
    // ~1/300: a tilt of a few nrad/s, leaking gravity at a few hundredths of a
    // mm/s.
    let got = ins.velocity();
    assert!(
        (got.east().metres_per_second() - 6.0).abs() < 1e-4,
        "{got:?}"
    );
    assert!(got.north().metres_per_second().abs() < 1e-4);
    let displacement = ins.displacement();
    assert!(
        (displacement.east().metres() - 180.0).abs() < 5e-3,
        "{displacement:?}"
    );
    assert!(displacement.north().metres().abs() < 5e-3);
    assert!(displacement.down().metres().abs() < 5e-3);

    // Chart position 180 m east of the origin: Δλ × radius of the parallel.
    let position = ins.position().unwrap();
    let sin_lat = LATITUDE.to_radians().sin();
    let prime_vertical = 6_378_137.0 / (1.0 - 0.006_694_379_990_14 * sin_lat * sin_lat).sqrt();
    let east_metres = origin().position().longitude_difference(position).radians()
        * LATITUDE.to_radians().cos()
        * prime_vertical;
    assert!((east_metres - 180.0).abs() < 0.05, "{east_metres}");
    assert!((position.latitude().degrees() - LATITUDE).abs() < 1e-7);
}

#[test]
fn the_coriolis_term_is_applied_when_the_reading_leaves_it_out() {
    // Same eastward acceleration, but IMU readings without the Coriolis term:
    // the mechanisation still removes it, deflecting the vessel to the right of
    // its motion (south, northern hemisphere).
    let mut ins = at_rest(90.0);
    let attitude = level(90.0);
    for _ in 0..6_000 {
        let sample = reading(&attitude, [0.0; 3], [0.0, 0.1, 0.0], [0.0; 3]);
        ins.step(&sample).unwrap();
    }
    let north = ins.displacement().north().metres();
    // 2 Ω sin φ ∫∫ v = 2 Ω sin φ × 0.1 × t³/6 ≈ 0.4 m.
    let expected = -2.0 * EARTH_RATE * LATITUDE.to_radians().sin() * 0.1 * 60.0_f64.powi(3) / 6.0;
    assert!((north - expected).abs() < 0.01, "{north} vs {expected}");
}

#[test]
fn gravity_is_the_wgs84_figure() {
    assert!((gravity_down(0.0, 0.0) - 9.780_325_3).abs() < 1e-6);
    assert!((gravity_down(1.0, 0.0) - 9.832_184_9).abs() < 1e-6);
    assert!((gravity_down(0.0, 1000.0) - (9.780_325_3 - 3.086e-3)).abs() < 1e-6);
}

#[test]
fn the_state_reads_back_in_the_kernels_types() {
    let ins = Strapdown::new(
        Instant::<Utc>::from_unix_seconds(0),
        origin(),
        Vector3::<Ned, Speed>::new(Speed::from_knots(10.0).unwrap(), Speed::ZERO, Speed::ZERO),
        level(0.0),
    )
    .unwrap();
    assert_eq!(ins.velocity().north().knots(), 10.0);
    assert_eq!(ins.gyro_bias(), [0.0; 3]);
    assert_eq!(ins.accel_bias(), [0.0; 3]);
    let position = ins.point().unwrap().position();
    assert!((position.latitude().degrees() - LATITUDE).abs() < 1e-9);
    assert!((position.longitude().degrees() + 1.0).abs() < 1e-9);
    assert_eq!(ins.frame().origin(), origin());
}
