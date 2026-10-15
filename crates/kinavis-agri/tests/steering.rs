//! Pass trackers: one step against `PythonRobotics`, and closed loops on a
//! straight pass, a curve and a change of pass at a range of speeds.

#![allow(
    clippy::expect_used,
    clippy::panic,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss
)]

use core::time::Duration;

use kinavis::region::PlanarPoint;
use kinavis_agri::{
    bicycle_step, locate, AbLine, FieldPlane, PassIndex, PassPattern, PathTracker, PivotLine,
    PurePursuit, ReferenceLine, Stanley, VehicleGeometry, VehicleState,
};
use kinavis_kernel::{Angle, Distance, GridCourse, Height, Position, RateOfTurn, Speed};

const STEERING: &str = include_str!("data/steering_reference.txt");

fn metres(value: f64) -> Distance {
    Distance::from_metres(value).expect("finite")
}

fn speed(value: f64) -> Speed {
    Speed::from_metres_per_second(value).expect("finite")
}

fn plane() -> FieldPlane {
    FieldPlane::centred_at(
        Position::from_degrees(47.5, 19.0).expect("on Earth"),
        Height::above_ellipsoid(metres(110.0)),
    )
    .expect("a plane")
}

fn point(east: f64, north: f64) -> PlanarPoint {
    PlanarPoint::from_metres(east, north).expect("finite")
}

fn vehicle(max_degrees: f64, rate_degrees: f64) -> VehicleGeometry {
    VehicleGeometry::new(
        metres(2.9),
        Angle::from_degrees(max_degrees).expect("finite"),
        RateOfTurn::from_radians_per_second(rate_degrees.to_radians()).expect("finite"),
    )
    .expect("a vehicle")
}

/// Pass 0 of either reference path of the data file.
enum Path {
    Line(AbLine),
    Circle(PivotLine),
}

impl Path {
    fn steer<T: PathTracker>(
        &self,
        tracker: &T,
        vehicle: &VehicleGeometry,
        state: &VehicleState,
    ) -> f64 {
        let pattern = PassPattern::new(metres(6.0), Distance::ZERO).expect("a pattern");
        let command = match self {
            Self::Line(line) => tracker.steer(line, &pattern, PassIndex::REFERENCE, vehicle, state),
            Self::Circle(circle) => {
                tracker.steer(circle, &pattern, PassIndex::REFERENCE, vehicle, state)
            }
        };
        command.expect("a command").angle.radians()
    }
}

#[test]
fn one_step_agrees_with_python_robotics() {
    let line = Path::Line(AbLine::from_point_and_course(
        plane(),
        PlanarPoint::ORIGIN,
        GridCourse::new(0.0).expect("north"),
    ));
    let circle =
        Path::Circle(PivotLine::new(plane(), PlanarPoint::ORIGIN, metres(40.0)).expect("a circle"));
    let pursuit = PurePursuit::new(metres(2.0), Duration::from_millis(100), metres(100.0))
        .expect("pure pursuit");
    let stanley = Stanley::new(0.5, Speed::ZERO).expect("Stanley");
    // Both clip at 45°: pure pursuit in the controller, Stanley in the model.
    let vehicle = vehicle(45.0, 30.0);
    let limit = 45f64.to_radians();
    let (mut checked, mut worst) = (0, 0.0f64);
    for row in STEERING.lines().filter(|line| !line.starts_with('#')) {
        let fields: Vec<&str> = row.split_ascii_whitespace().collect();
        let [controller, path, x, y, yaw, v, delta] = fields[..] else {
            panic!("seven columns: {row}");
        };
        let number = |field: &str| field.parse::<f64>().expect("a number");
        let state = VehicleState {
            position: point(number(x), number(y)),
            heading: GridCourse::wrap(90.0 - number(yaw).to_degrees()).expect("finite"),
            speed: speed(number(v)),
        };
        let path = if path == "line" { &line } else { &circle };
        let ours = match controller {
            "pure_pursuit" => path.steer(&pursuit, &vehicle, &state),
            _ => path.steer(&stanley, &vehicle, &state),
        };
        // Theirs turns left for a positive angle.
        let theirs = -number(delta).clamp(-limit, limit);
        let error = (ours - theirs).abs();
        // PythonRobotics' Stanley takes the front axle's offset along the
        // machine's own lateral axis, e·cos ψₑ; this crate, as Hoffmann et
        // al., the distance to the pass. The gap is at most k·|e|·(1 − cos ψₑ)/v.
        let allowed = if controller == "stanley" {
            let front = vehicle.front_axle(state.position, state.heading);
            let (e, psi) = match path {
                Path::Line(_) => (front.east_metres(), state.heading.radians()),
                Path::Circle(_) => {
                    let r = front.east_metres().hypot(front.north_metres());
                    let tangent = front.east_metres().atan2(front.north_metres())
                        + core::f64::consts::FRAC_PI_2;
                    (r - 40.0, state.heading.radians() - tangent)
                }
            };
            2e-3 + 0.5 * e.abs() * (1.0 - psi.cos()) / number(v)
        } else {
            2e-3
        };
        assert!(error < allowed, "{row}: {ours} against {theirs}");
        worst = worst.max(error - allowed + 2e-3);
        checked += 1;
    }
    assert_eq!(checked, 600);
    // The paths are sampled every millimetre; that is the whole difference.
    assert!(worst < 2e-3, "worst {worst}");
}

/// Closed loop: steps of 50 ms with the wheels slewing at their rate.
struct Run {
    worst_settled: f64,
    overshoot: f64,
}

fn run<L: ReferenceLine, T: PathTracker>(
    line: &L,
    tracker: &T,
    pass: PassIndex,
    start: VehicleState,
    seconds: f64,
    settle_after: f64,
) -> Run {
    let pattern = PassPattern::new(metres(6.0), Distance::ZERO).expect("a pattern");
    let vehicle = vehicle(35.0, 25.0);
    let dt = Duration::from_millis(50);
    let mut state = start;
    let mut wheels = Angle::ZERO;
    let start_side = locate_on(line, &pattern, pass, &state).signum();
    let mut result = Run {
        worst_settled: 0.0,
        overshoot: 0.0,
    };
    let steps = (seconds / 0.05) as usize;
    for step in 0..steps {
        let command = tracker
            .steer(line, &pattern, pass, &vehicle, &state)
            .expect("a command");
        wheels = vehicle.slew(wheels, command.angle, dt);
        state = bicycle_step(&state, &vehicle, wheels, dt).expect("a step");
        let error = locate_on(line, &pattern, pass, &state);
        // Overshoot: how far it went past the pass to the other side.
        if error * start_side < 0.0 {
            result.overshoot = result.overshoot.max(error.abs());
        }
        if step as f64 * 0.05 >= settle_after {
            result.worst_settled = result.worst_settled.max(error.abs());
        }
    }
    result
}

/// Cross-track error of the rear axle from the given pass.
fn locate_on<L: ReferenceLine>(
    line: &L,
    pattern: &PassPattern,
    pass: PassIndex,
    state: &VehicleState,
) -> f64 {
    let offset = line.station(state.position).offset.metres() - pattern.offset_of(pass).metres();
    let view = locate(line, pattern, state.position, state.heading);
    if view.reversed {
        offset
    } else {
        -offset
    }
}

fn trackers() -> (PurePursuit, Stanley) {
    (
        PurePursuit::new(metres(3.0), Duration::from_millis(600), metres(15.0))
            .and_then(|p| p.with_intercept(Angle::from_degrees(30.0).expect("finite")))
            .expect("pure pursuit"),
        Stanley::new(1.2, speed(0.5)).expect("Stanley"),
    )
}

#[test]
fn a_straight_pass_is_acquired_and_held_at_every_working_speed() {
    let line = AbLine::from_point_and_course(
        plane(),
        PlanarPoint::ORIGIN,
        GridCourse::new(30.0).expect("a course"),
    );
    let (pursuit, stanley) = trackers();
    for v in [0.5, 1.5, 3.0, 5.0, 8.0] {
        // 2 m to the right of the pass, 10° off its course.
        let (s, c) = 30f64.to_radians().sin_cos();
        let start = VehicleState {
            position: point(2.0 * c, -2.0 * s),
            heading: GridCourse::new(40.0).expect("a course"),
            speed: speed(v),
        };
        let time = 60.0 + 200.0 / v;
        for (name, result) in [
            (
                "pure pursuit",
                run(
                    &line,
                    &pursuit,
                    PassIndex::REFERENCE,
                    start,
                    time,
                    time - 20.0,
                ),
            ),
            (
                "Stanley",
                run(
                    &line,
                    &stanley,
                    PassIndex::REFERENCE,
                    start,
                    time,
                    time - 20.0,
                ),
            ),
        ] {
            assert!(
                result.worst_settled < 0.01,
                "{name} at {v} m/s: settled {}",
                result.worst_settled
            );
            assert!(
                result.overshoot < 0.3,
                "{name} at {v} m/s: overshoot {}",
                result.overshoot
            );
        }
    }
}

#[test]
fn a_curved_pass_is_held_at_its_geometric_offset() {
    // Pass +2 of a 60 m pivot: a circle of 72 m, driven clockwise.
    let pivot = PivotLine::new(plane(), PlanarPoint::ORIGIN, metres(60.0)).expect("a pivot");
    let (pursuit, stanley) = trackers();
    for v in [1.0, 3.0, 5.0] {
        let start = VehicleState {
            position: point(0.0, 72.5),
            heading: GridCourse::new(90.0).expect("east"),
            speed: speed(v),
        };
        let time = 40.0 + 150.0 / v;
        for (name, result, expected) in [
            (
                "pure pursuit",
                run(
                    &pivot,
                    &pursuit,
                    PassIndex::new(2),
                    start,
                    time,
                    time - 15.0,
                ),
                0.0,
            ),
            // The front axle holds the pass; the rear runs inside by R − √(R² − L²).
            (
                "Stanley",
                run(
                    &pivot,
                    &stanley,
                    PassIndex::new(2),
                    start,
                    time,
                    time - 15.0,
                ),
                72.0 - (72f64.powi(2) - 2.9f64.powi(2)).sqrt(),
            ),
        ] {
            let settled = (result.worst_settled - expected).abs();
            assert!(
                settled < 0.01,
                "{name} at {v} m/s: settled {} against {expected}",
                result.worst_settled
            );
        }
    }
}

#[test]
fn moving_to_the_next_pass_overshoots_little() {
    let line = AbLine::from_point_and_course(
        plane(),
        PlanarPoint::ORIGIN,
        GridCourse::new(0.0).expect("north"),
    );
    let (pursuit, stanley) = trackers();
    for v in [1.0, 2.5, 4.0] {
        // On pass 0, then told to take pass +1, 6 m to the left.
        let start = VehicleState {
            position: PlanarPoint::ORIGIN,
            heading: GridCourse::new(0.0).expect("north"),
            speed: speed(v),
        };
        let time = 40.0 + 120.0 / v;
        for (name, result) in [
            (
                "pure pursuit",
                run(&line, &pursuit, PassIndex::new(1), start, time, time - 15.0),
            ),
            (
                "Stanley",
                run(&line, &stanley, PassIndex::new(1), start, time, time - 15.0),
            ),
        ] {
            assert!(
                result.worst_settled < 0.01,
                "{name} at {v} m/s: settled {}",
                result.worst_settled
            );
            assert!(
                result.overshoot < 0.15 * 6.0,
                "{name} at {v} m/s: overshoot {}",
                result.overshoot
            );
        }
    }
}
