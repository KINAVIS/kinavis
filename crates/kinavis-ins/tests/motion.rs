//! INS as the six-state estimator's process model.

#![allow(clippy::unwrap_used, clippy::float_cmp)]

use core::time::Duration;

use kinavis_ins::{ImuNoise, InsFilter, InsMotion, InsPriors, Quaternion, Strapdown};
use kinavis_kernel::estimation::ProcessModel;
use kinavis_kernel::{
    Angle, Distance, GeodeticPoint, GnssFix, Height, Instant, NavigationState, Ned, Position,
    RateOfTurn, Speed, StateComponent, TrueCourse, Utc, Vector3,
};

fn start() -> Instant<Utc> {
    Instant::from_unix_seconds(1_789_000_000)
}

fn state() -> NavigationState {
    let fix = GnssFix::builder(start(), Position::from_degrees(50.0, -1.0).unwrap())
        .course_over_ground(TrueCourse::new(90.0).unwrap())
        .speed_over_ground(Speed::from_metres_per_second(5.0).unwrap())
        .build();
    NavigationState::initialised_from(&fix, None).unwrap()
}

fn motion() -> InsMotion {
    InsMotion::new(
        Vector3::<Ned, Speed>::new(
            Speed::from_metres_per_second(3.0).unwrap(),
            Speed::from_metres_per_second(4.0).unwrap(),
            Speed::ZERO,
        ),
        RateOfTurn::from_degrees_per_minute(60.0).unwrap(),
        Speed::from_metres_per_second(0.1).unwrap(),
        Angle::from_degrees(0.5).unwrap(),
    )
    .unwrap()
}

#[test]
fn the_position_and_heading_follow_what_the_system_reports() {
    let model = motion();
    let moved = model.propagate(&state(), Duration::from_secs(10)).unwrap();
    // 30 m north, 40 m east; heading 90° + 10°.
    let displacement = moved
        .frame()
        .ned_of(GeodeticPoint::new(
            moved.position(),
            Height::above_ellipsoid(Distance::ZERO),
        ))
        .unwrap();
    assert!((displacement.north().metres() - 30.0).abs() < 1e-6);
    assert!((displacement.east().metres() - 40.0).abs() < 1e-6);
    assert!((moved.heading().degrees() - 100.0).abs() < 1e-9);
    assert_eq!(moved.speed_through_water().metres_per_second(), 5.0);
    assert_eq!(
        moved.valid_at(),
        start().saturating_add(Duration::from_secs(10))
    );
    assert_eq!(model.velocity().east().metres_per_second(), 4.0);
    assert!((model.yaw_rate().degrees_per_minute() - 60.0).abs() < 1e-9);
}

#[test]
fn the_jacobian_is_the_identity_and_the_noise_adds_up() {
    let model = motion();
    let state = state();
    let jacobian = model.jacobian(&state, Duration::from_secs(5)).unwrap();
    for row in StateComponent::ALL {
        for column in StateComponent::ALL {
            assert_eq!(jacobian.derivative(row, column), f64::from(row == column));
        }
    }
    // Q(a + b) = Q(a) + Q(b): required for exact late-observation handling.
    let whole = model.noise(&state, Duration::from_secs(10));
    let first = model.noise(&state, Duration::from_secs(3));
    let second = model.noise(&state, Duration::from_secs(7));
    for component in StateComponent::ALL {
        assert!(
            (whole.variance(component) - first.variance(component) - second.variance(component))
                .abs()
                < 1e-12
        );
    }
    assert!((whole.variance(StateComponent::North) - 0.1).abs() < 1e-12);
    assert!(
        (whole.variance(StateComponent::Heading) - 10.0 * 0.5_f64.to_radians().powi(2)).abs()
            < 1e-12
    );
    assert_eq!(whole.variance(StateComponent::SpeedThroughWater), 0.0);
}

#[test]
fn a_model_comes_from_a_filter_and_refuses_a_walk_of_nothing() {
    let nominal = Strapdown::new(
        start(),
        GeodeticPoint::new(
            Position::from_degrees(50.0, -1.0).unwrap(),
            Height::above_ellipsoid(Distance::ZERO),
        ),
        Vector3::<Ned, Speed>::new(
            Speed::from_metres_per_second(1.0).unwrap(),
            Speed::ZERO,
            Speed::ZERO,
        ),
        Quaternion::IDENTITY,
    )
    .unwrap();
    let filter = InsFilter::new(nominal, ImuNoise::mems(), &InsPriors::standard());
    let model = InsMotion::from_filter(&filter, RateOfTurn::ZERO).unwrap();
    assert_eq!(model.velocity().north().metres_per_second(), 1.0);
    assert!(InsMotion::new(
        model.velocity(),
        RateOfTurn::ZERO,
        Speed::ZERO,
        Angle::from_degrees(0.5).unwrap()
    )
    .is_err());
}
