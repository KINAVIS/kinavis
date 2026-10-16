//! The kinematic bicycle as the estimator's process model: propagation
//! against fine integration through changes of the wheel angle, the Jacobian
//! against finite differences, the noise of a step against its halves, and
//! the estimator through a headland turn with the receiver silent.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::indexing_slicing
)]

use core::time::Duration;

use kinavis::estimator::{Estimator, EstimatorConfig, LatePolicy, SteadyMotion};
use kinavis::observations::PositionObservation;
use kinavis_agri::{BicycleMotion, VehicleGeometry};
use kinavis_kernel::estimation::ProcessModel;
use kinavis_kernel::matrix::{Matrix, Vector};
use kinavis_kernel::state::{NavigationState, StatePriors};
use kinavis_kernel::{
    Angle, Distance, GeodeticPoint, GnssFix, Height, Instant, LocalFrame, Ned, Position,
    RateOfTurn, Speed, TrueCourse, Utc, Vector3,
};

fn vehicle() -> VehicleGeometry {
    VehicleGeometry::new(
        Distance::from_metres(2.9).unwrap(),
        Angle::from_degrees(40.0).unwrap(),
        RateOfTurn::from_radians_per_second(0.5).unwrap(),
    )
    .unwrap()
}

fn at(millis: u64) -> Instant<Utc> {
    Instant::from_unix_seconds(1_760_000_000).saturating_add(Duration::from_millis(millis))
}

fn anchor() -> Position {
    Position::from_degrees(52.0, 5.6).unwrap()
}

fn frame() -> LocalFrame {
    LocalFrame::at(
        GeodeticPoint::new(anchor(), Height::above_ellipsoid(Distance::ZERO)),
        &kinavis_kernel::geodesy::Ellipsoid::WGS84,
    )
    .unwrap()
}

/// A state with the given components and a unit covariance.
fn state(x: [f64; 6], when: Instant<Utc>) -> NavigationState {
    NavigationState::from_parts(when, frame(), Vector::from_column(x), Matrix::identity()).unwrap()
}

/// Wheel angles of a headland turn: straight, 25° right for 9 s, straight.
fn turning() -> BicycleMotion<16> {
    let mut model = BicycleMotion::<16>::standard(&vehicle()).unwrap();
    model.steer(at(0), Angle::ZERO).unwrap();
    model
        .steer(at(1_300), Angle::from_degrees(25.0).unwrap())
        .unwrap();
    model
        .steer(at(10_300), Angle::from_degrees(-3.0).unwrap())
        .unwrap();
    model
}

/// Euler integration of the bicycle in 10 µs steps.
fn integrate(model: &BicycleMotion<16>, x: [f64; 6], from: u64, millis: u64) -> [f64; 6] {
    let [mut north, mut east, mut heading, speed, slip_north, slip_east] = x;
    let h = 1e-5;
    let steps = (millis as f64 * 1e-3 / h).round() as u64;
    for step in 0..steps {
        let when = at(from).saturating_add(Duration::from_nanos(step * 10_000 + 5_000));
        let curvature = model.curvature_at(when);
        north += (speed * heading.cos() + slip_north) * h;
        east += (speed * heading.sin() + slip_east) * h;
        heading += speed * curvature * h;
    }
    [north, east, heading, speed, slip_north, slip_east]
}

#[test]
fn propagation_follows_the_arcs_through_changes_of_wheel_angle() {
    let model = turning();
    let x = [3.0, -2.0, 0.4, 2.5, 0.02, -0.01];
    for (from, millis) in [(0, 400), (1_100, 600), (1_000, 12_000), (10_000, 500)] {
        let ours = model
            .propagate(&state(x, at(from)), Duration::from_millis(millis))
            .unwrap();
        let fine = integrate(&model, x, from, millis);
        for (index, expected) in fine.iter().enumerate() {
            let got = ours.vector().element(index).unwrap();
            assert!(
                (got - expected).abs() < 2e-4,
                "component {index} over {from}+{millis}: {got} against {expected}"
            );
        }
    }
    // A full turn of 9 s at 25°: the heading turns by v·t·tan δ / L.
    let ours = model
        .propagate(&state(x, at(1_300)), Duration::from_secs(9))
        .unwrap();
    let expected = 0.4 + 2.5 * 9.0 * 25f64.to_radians().tan() / 2.9;
    assert!((ours.vector().element(2).unwrap() - expected).abs() < 1e-12);
}

#[test]
fn the_jacobian_is_the_derivative_of_the_propagation() {
    let model = turning();
    let x = [3.0, -2.0, 0.4, 2.5, 0.02, -0.01];
    let over = Duration::from_millis(1_500);
    let from = at(500);
    let jacobian = model.jacobian(&state(x, from), over).unwrap();
    for column in 0..6 {
        let step = 1e-6;
        let mut plus = x;
        let mut minus = x;
        plus[column] += step;
        minus[column] -= step;
        let forward = model.propagate(&state(plus, from), over).unwrap();
        let backward = model.propagate(&state(minus, from), over).unwrap();
        for row in 0..6 {
            let numeric = (forward.vector().element(row).unwrap()
                - backward.vector().element(row).unwrap())
                / (2.0 * step);
            let analytic = jacobian.matrix().get(row, column).unwrap();
            assert!(
                (numeric - analytic).abs() < 1e-6,
                "∂{row}/∂{column}: {analytic} against {numeric}"
            );
        }
    }
}

#[test]
fn the_noise_of_a_step_is_the_noise_of_its_halves() {
    let model = turning();
    let x = [0.0, 0.0, 1.0, 3.0, 0.0, 0.0];
    for (from, millis) in [(0, 1_000), (2_000, 1_000), (900, 1_000)] {
        let whole = *model
            .noise(&state(x, at(from)), Duration::from_millis(millis))
            .matrix();
        let half = Duration::from_millis(millis / 2);
        let first = model.noise(&state(x, at(from)), half);
        let middle = model.propagate(&state(x, at(from)), half).unwrap();
        let jacobian = model.jacobian(&middle, half).unwrap();
        let second = model.noise(&middle, half);
        let composed =
            *jacobian.matrix() * *first.matrix() * jacobian.matrix().transpose() + *second.matrix();
        let scale = whole.max_abs();
        assert!(
            whole.cholesky().is_some(),
            "not positive definite at {from}"
        );
        let difference = (whole - composed).max_abs() / scale;
        assert!(
            difference < 1e-5,
            "halves differ by {difference:e} at {from}"
        );
    }
}

#[test]
fn the_steering_log_keeps_its_order_and_its_capacity() {
    let mut model = BicycleMotion::<2>::standard(&vehicle()).unwrap();
    assert_eq!(model.curvature_at(at(0)), 0.0);
    model
        .steer(at(1_000), Angle::from_degrees(10.0).unwrap())
        .unwrap();
    model
        .steer(at(2_000), Angle::from_degrees(20.0).unwrap())
        .unwrap();
    model
        .steer(at(3_000), Angle::from_degrees(30.0).unwrap())
        .unwrap();
    // The oldest went; before the oldest kept angle that angle holds.
    let twenty = 20f64.to_radians().tan() / 2.9;
    assert!((model.curvature_at(at(500)) - twenty).abs() < 1e-15);
    assert!(model.steer(at(2_500), Angle::ZERO).is_err());
}

/// Truth of the headland turn: rear axle north/east and heading.
struct Turn {
    north: f64,
    east: f64,
    heading: f64,
}

fn wheel_angle(millis: u64) -> f64 {
    if (8_000..17_000).contains(&millis) {
        25f64.to_radians()
    } else {
        0.0
    }
}

/// Worst position error in the outage and after settling, and the worst
/// error in the outage over the reported semi-major axis.
fn run<P: ProcessModel>(
    process: P,
    lag_ticks: usize,
    feed: impl Fn(&mut P, Instant<Utc>, f64),
) -> (f64, f64, f64) {
    let frame = frame();
    let speed = 2.5;
    let fix = GnssFix::builder(at(0), anchor())
        .course_over_ground(TrueCourse::new(0.0).unwrap())
        .speed_over_ground(Speed::from_metres_per_second(speed).unwrap())
        .build();
    let initial = NavigationState::initialised_with(&fix, None, &StatePriors::standard()).unwrap();
    let config = EstimatorConfig::standard().with_late(LatePolicy::Smooth {
        max_lag: Duration::from_secs(2),
    });
    let mut estimator = Estimator::new(initial, process, config);
    let mut truth = Turn {
        north: 0.0,
        east: 0.0,
        heading: 0.0,
    };
    let mut noise = 0x9E37_79B9_7F4A_7C15u64;
    let mut normal = move || {
        // Sum of twelve uniforms, centred: unit variance, enough here.
        let mut sum = -6.0;
        for _ in 0..12 {
            noise ^= noise << 13;
            noise ^= noise >> 7;
            noise ^= noise << 17;
            sum += (noise >> 11) as f64 / (1u64 << 53) as f64;
        }
        sum
    };
    let mut worst_in_outage: f64 = 0.0;
    let mut worst_settled: f64 = 0.0;
    let mut worst_ratio: f64 = 0.0;
    let mut history: Vec<(u64, f64, f64)> = Vec::new();
    // 25 s at 5 Hz; the receiver is silent from 9 s to 15 s, mid-turn.
    for tick in 1..=125u64 {
        let millis = tick * 200;
        // Truth in 1 ms steps; the steering is fed as it changes.
        for ms in (millis - 200)..millis {
            let curvature = wheel_angle(ms).tan() / 2.9;
            truth.north += speed * truth.heading.cos() * 1e-3;
            truth.east += speed * truth.heading.sin() * 1e-3;
            truth.heading += speed * curvature * 1e-3;
        }
        feed(estimator.process_mut(), at(millis), wheel_angle(millis));
        history.push((millis, truth.north, truth.east));
        let _ = estimator.advance_to(at(millis)).unwrap();
        let silent = (9_000..15_000).contains(&millis);
        // A late receiver reports `lag_ticks` ticks after the fix was taken.
        let reported = history.len().checked_sub(1 + lag_ticks).map(|i| history[i]);
        if let (false, Some((taken, north, east))) = (silent, reported) {
            let measured: Vector3<Ned, Distance> = Vector3::new(
                Distance::from_metres(north + 0.02 * normal()).unwrap(),
                Distance::from_metres(east + 0.02 * normal()).unwrap(),
                Distance::ZERO,
            );
            let position = frame.point_from_ned(measured).unwrap().position();
            let _ = estimator
                .ingest(&PositionObservation::new(
                    at(taken),
                    position,
                    Distance::from_metres(0.02).unwrap(),
                ))
                .unwrap();
        }
        let state = estimator.state();
        let error = (state.vector().element(0).unwrap() - truth.north)
            .hypot(state.vector().element(1).unwrap() - truth.east);
        if silent {
            worst_in_outage = worst_in_outage.max(error);
            worst_ratio = worst_ratio.max(error / state.horizontal_error().semi_major().metres());
        } else if millis > 5_000 {
            worst_settled = worst_settled.max(error);
        }
    }
    (worst_in_outage, worst_settled, worst_ratio)
}

#[test]
fn the_estimator_holds_a_turn_through_an_outage_when_it_knows_the_wheels() {
    // The wheel angle is fed as it is commanded, before the step that uses it.
    let (bicycle_outage, bicycle_settled, bicycle_ratio) = run(
        BicycleMotion::<64>::standard(&vehicle()).unwrap(),
        0,
        |model, when, angle| {
            model
                .steer(when, Angle::from_radians(angle).unwrap())
                .unwrap();
        },
    );
    let (steady_outage, _, _) = run(SteadyMotion::standard(), 0, |_, _, _| {});
    assert!(bicycle_settled < 0.06, "settled {bicycle_settled}");
    // Inside the reported 99.9 % ellipse (3.7σ in two dimensions) all through
    // six seconds of a 25° turn without a fix.
    assert!(bicycle_ratio < 3.7, "error {bicycle_ratio}σ");
    // A straight-line model loses the turn: tens of times the error.
    assert!(
        steady_outage > 20.0 * bicycle_outage,
        "steady {steady_outage} against {bicycle_outage}"
    );
}

#[test]
fn a_late_receiver_is_folded_in_with_the_wheel_angles_of_its_time() {
    // Fixes 0.4 s late through the turn: the history is re-propagated with
    // the angles then in force. A model that remembers only the latest angle
    // replays the start and the end of the turn with the wrong one.
    let (_, in_time, _) = run(
        BicycleMotion::<64>::standard(&vehicle()).unwrap(),
        0,
        |model, when, angle| {
            model
                .steer(when, Angle::from_radians(angle).unwrap())
                .unwrap();
        },
    );
    let (_, late, ratio) = run(
        BicycleMotion::<64>::standard(&vehicle()).unwrap(),
        2,
        |model, when, angle| {
            model
                .steer(when, Angle::from_radians(angle).unwrap())
                .unwrap();
        },
    );
    let (_, forgetful, _) = run(
        BicycleMotion::<1>::standard(&vehicle()).unwrap(),
        2,
        |model, when, angle| {
            model
                .steer(when, Angle::from_radians(angle).unwrap())
                .unwrap();
        },
    );
    // A forecast 0.4 s long costs a few centimetres: 3.4 cm in time, 8.3 late.
    assert!(
        late < 0.12 && late < 4.0 * in_time,
        "late {late} against in time {in_time}"
    );
    assert!(ratio < 3.7, "error {ratio}σ");
    // Without the log the replay steers the wrong way: tens of metres.
    assert!(forgetful > 50.0 * late, "forgetful {forgetful}");
}
