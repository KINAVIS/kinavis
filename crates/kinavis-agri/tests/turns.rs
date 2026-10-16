//! Headland turns: Dubins paths against Walker's library, the U and omega
//! turns against their closed forms, the fit in a headland, the path as a
//! reference line, and a tracker driving a turn onto the next pass.

#![allow(
    clippy::expect_used,
    clippy::panic,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::indexing_slicing
)]

use core::f64::consts::PI;
use core::time::Duration;

use kinavis::region::{PlanarPoint, Polygon, Region};
use kinavis_agri::{
    bicycle_step, AbLine, DubinsWord, Field, FieldPlane, PassIndex, PassPattern, PathTracker, Pose,
    PurePursuit, ReferenceLine, TurnPath, VehicleGeometry, VehicleState,
};
use kinavis_kernel::{Angle, Distance, GridCourse, Height, Position, RateOfTurn, Speed};

const DUBINS: &str = include_str!("data/dubins_reference.txt");

fn metres(value: f64) -> Distance {
    Distance::from_metres(value).expect("finite")
}

fn point(east: f64, north: f64) -> PlanarPoint {
    PlanarPoint::from_metres(east, north).expect("finite")
}

fn plane() -> FieldPlane {
    FieldPlane::centred_at(
        Position::from_degrees(45.7, 4.8).expect("on Earth"),
        Height::above_ellipsoid(metres(200.0)),
    )
    .expect("a plane")
}

/// Pose from the library's frame: heading counter-clockwise from east.
fn pose(x: f64, y: f64, heading: f64) -> Pose {
    Pose {
        position: point(x, y),
        course: GridCourse::wrap(90.0 - heading.to_degrees()).expect("finite"),
    }
}

#[test]
fn the_shortest_path_agrees_with_walkers_library() {
    let words = [
        DubinsWord::Lsl,
        DubinsWord::Lsr,
        DubinsWord::Rsl,
        DubinsWord::Rsr,
        DubinsWord::Rlr,
        DubinsWord::Lrl,
    ];
    let (mut checked, mut worst) = (0, 0.0f64);
    for row in DUBINS.lines().filter(|line| !line.starts_with('#')) {
        let v: Vec<f64> = row
            .split_ascii_whitespace()
            .map(|field| field.parse().expect("a number"))
            .collect();
        let (from, to, radius) = (pose(v[0], v[1], v[2]), pose(v[3], v[4], v[5]), v[6]);
        let theirs = words[v[7] as usize];
        let path = TurnPath::dubins(plane(), from, to, metres(radius)).expect("a path");
        let total: f64 = v[8] + v[9] + v[10];
        let ours = path.length().metres();
        assert!((ours - total).abs() < 1e-7, "{row}: {ours} against {total}");
        if path.word() == theirs {
            for (piece, expected) in path.pieces().iter().zip(&v[8..11]) {
                let error = (piece.length.metres() - expected).abs();
                assert!(
                    error < 1e-7,
                    "{row}: piece {} against {expected}",
                    piece.length.metres()
                );
                worst = worst.max(error);
            }
        } else {
            // Two words of the same length: either is the shortest.
            let other = TurnPath::with_word(plane(), from, to, metres(radius), theirs)
                .expect("a radius")
                .expect("their word exists");
            assert!((other.length().metres() - ours).abs() < 1e-7, "{row}");
        }
        // The path ends where it was asked to.
        let end = path.end();
        assert!(
            end.position.distance_to(to.position).metres() < 1e-7,
            "{row}: ends at {}",
            end.position
        );
        assert!(end.course.angular_distance(to.course) < 1e-7, "{row}");
        checked += 1;
    }
    assert_eq!(checked, 3000);
    assert!(worst < 1e-7);
}

fn north_line() -> AbLine {
    AbLine::from_point_and_course(
        plane(),
        PlanarPoint::ORIGIN,
        GridCourse::new(0.0).expect("north"),
    )
}

#[test]
fn a_turn_onto_a_pass_two_radii_away_or_more_is_a_u() {
    let pattern = PassPattern::new(metres(15.0), Distance::ZERO).expect("a pattern");
    let exit = Pose {
        position: point(0.0, 300.0),
        course: GridCourse::new(0.0).expect("north"),
    };
    for radius in [4.0, 6.0, 7.5] {
        // Pass +1 is 15 m to the left (west).
        let turn = TurnPath::to_pass(
            &north_line(),
            &pattern,
            exit,
            PassIndex::new(1),
            metres(radius),
        )
        .expect("a turn");
        assert_eq!(turn.word(), DubinsWord::Lsl);
        let expected = PI * radius + (15.0 - 2.0 * radius);
        assert!(
            (turn.length().metres() - expected).abs() < 1e-9,
            "{} against {expected}",
            turn.length().metres()
        );
        let end = turn.end();
        assert!(end.position.distance_to(point(-15.0, 300.0)).metres() < 1e-9);
        assert!((end.course.degrees() - 180.0).abs() < 1e-9);
    }
}

#[test]
fn a_turn_onto_a_pass_closer_than_two_radii_is_an_omega() {
    for (width, radius) in [(6.0, 6.0), (9.0, 5.0), (3.0, 8.0)] {
        let pattern = PassPattern::new(metres(width), Distance::ZERO).expect("a pattern");
        let exit = Pose {
            position: point(0.0, 300.0),
            course: GridCourse::new(0.0).expect("north"),
        };
        let turn = TurnPath::to_pass(
            &north_line(),
            &pattern,
            exit,
            PassIndex::new(1),
            metres(radius),
        )
        .expect("a turn");
        assert_eq!(turn.word(), DubinsWord::Rlr);
        // Outer circles 2R + w apart, the middle one 2R from each: the
        // first and last arcs sweep γ, the middle π + 2γ.
        let h = ((2.0 * radius).powi(2) - f64::midpoint(width, 2.0 * radius).powi(2)).sqrt();
        let gamma = h.atan2(width / 2.0 + radius);
        let expected = radius * (4.0 * gamma + PI);
        assert!(
            (turn.length().metres() - expected).abs() < 1e-9,
            "{width} m at {radius} m"
        );
    }
}

fn field(headland: f64) -> Field<8, 8, 1> {
    let outer = Polygon::<8>::new(&[
        point(-60.0, -10.0),
        point(40.0, -10.0),
        point(40.0, 300.0 + headland),
        point(-60.0, 300.0 + headland),
    ])
    .expect("a field");
    Field::new(plane(), Region::without_holes(outer))
}

#[test]
fn a_turn_is_reported_clear_only_where_it_is_and_always_past_the_step() {
    let pattern = PassPattern::new(metres(12.0), Distance::ZERO).expect("a pattern");
    let exit = Pose {
        position: point(0.0, 300.0),
        course: GridCourse::new(0.0).expect("north"),
    };
    // A 12 m boom on a 6 m turn: a semicircle peaking 6 m past the pass end,
    // the boom 6 m beyond it. The least headland is exactly 12 m; checked
    // every 0.5 m, clear is reported from 12.25 m.
    let turn = TurnPath::to_pass(
        &north_line(),
        &pattern,
        exit,
        PassIndex::new(1),
        metres(6.0),
    )
    .expect("a turn");
    let (half, step) = (metres(6.0), metres(0.5));
    for quarter in 40..=60 {
        let headland = f64::from(quarter) * 0.25;
        let clear = turn
            .first_conflict(&field(headland), half, step)
            .expect("one plane")
            .is_none();
        if clear {
            assert!(headland >= 12.0, "clear at {headland} m");
        }
        if headland >= 12.25 + 1e-9 {
            assert!(clear, "not clear at {headland} m");
        }
    }
    let conflict = turn
        .first_conflict(&field(10.0), half, step)
        .expect("one plane")
        .expect("too narrow");
    assert!(conflict.metres() > 0.0 && conflict.metres() < turn.length().metres());
}

#[test]
fn degenerate_turns_have_their_obvious_lengths() {
    let here = pose(5.0, 5.0, 1.0);
    let still = TurnPath::dubins(plane(), here, here, metres(6.0)).expect("a path");
    assert!(still.length().metres() < 1e-9);
    // Straight ahead: all straight.
    let ahead = pose(5.0 + 30.0 * 1f64.cos(), 5.0 + 30.0 * 1f64.sin(), 1.0);
    let straight = TurnPath::dubins(plane(), here, ahead, metres(6.0)).expect("a path");
    assert!((straight.length().metres() - 30.0).abs() < 1e-9);
    // Turning round on the spot: a full circle less nothing, or a
    // three-arc path; never shorter than the half circle it must contain.
    let back = pose(5.0, 5.0, 1.0 + PI);
    let round = TurnPath::dubins(plane(), here, back, metres(6.0)).expect("a path");
    assert!(round.length().metres() > PI * 6.0);
    assert!(round.end().position.distance_to(here.position).metres() < 1e-9);
}

#[test]
fn the_path_is_a_reference_line_through_its_pieces() {
    let from = pose(3.0, -4.0, 0.3);
    let to = pose(-20.0, 11.0, 2.9);
    let (mut exact, mut total) = (0, 0);
    for word in DubinsWord::ALL {
        let Some(path) =
            TurnPath::with_word(plane(), from, to, metres(5.0), word).expect("a radius")
        else {
            continue;
        };
        let length = path.length().metres();
        for step in 0..=40 {
            let along = length * f64::from(step) / 40.0;
            let at = path.pose_at(metres(along));
            let station = path.station(at.position);
            assert!(station.offset.metres().abs() < 1e-9, "{word:?} at {along}");
            assert!(
                station.direction.angular_distance(at.course) < 1e-6,
                "{word:?} at {along}"
            );
            // Half a metre to the left of the path is half a metre left.
            let (north, east) = at.course.components(1.0);
            let left = point(
                at.position.east_metres() - 0.5 * north,
                at.position.north_metres() + 0.5 * east,
            );
            // Nearer than 0.5 m only where another part of the path loops
            // back close; exactly 0.5 m left everywhere else.
            let shifted = path.station(left).offset.metres();
            assert!(
                shifted.abs() <= 0.5 + 1e-9,
                "{word:?} at {along}: {shifted}"
            );
            total += 1;
            if (shifted - 0.5).abs() < 1e-9 {
                exact += 1;
            }
        }
    }
    assert!(exact * 10 >= total * 9, "{exact} of {total}");
}

#[test]
fn pure_pursuit_drives_the_turn_and_settles_on_the_next_pass() {
    let line = north_line();
    let pattern = PassPattern::new(metres(12.0), Distance::ZERO).expect("a pattern");
    let vehicle = VehicleGeometry::new(
        metres(2.9),
        Angle::from_degrees(40.0).expect("finite"),
        RateOfTurn::from_radians_per_second(40f64.to_radians()).expect("finite"),
    )
    .expect("a vehicle");
    let tracker =
        PurePursuit::new(metres(2.5), Duration::from_millis(500), metres(10.0)).expect("a tracker");
    let exit = Pose {
        position: point(0.0, 300.0),
        course: GridCourse::new(0.0).expect("north"),
    };
    let turn =
        TurnPath::to_pass(&line, &pattern, exit, PassIndex::new(1), metres(7.0)).expect("a turn");
    let dt = Duration::from_millis(50);
    let mut state = VehicleState {
        position: exit.position,
        heading: exit.course,
        speed: Speed::from_metres_per_second(2.0).expect("finite"),
    };
    let mut wheels = Angle::ZERO;
    let mut worst_in_turn: f64 = 0.0;
    let turn_seconds = turn.length().metres() / 2.0;
    let mut on_pass = false;
    for step in 0..((turn_seconds + 40.0) / 0.05) as usize {
        let seconds = step as f64 * 0.05;
        let command = if on_pass {
            tracker.steer(&line, &pattern, PassIndex::new(1), &vehicle, &state)
        } else {
            tracker.steer(&turn, &pattern, PassIndex::REFERENCE, &vehicle, &state)
        }
        .expect("a command");
        wheels = vehicle.slew(wheels, command.angle, dt);
        state = bicycle_step(&state, &vehicle, wheels, dt).expect("a step");
        if !on_pass {
            worst_in_turn = worst_in_turn.max(turn.station(state.position).offset.metres().abs());
            // Hand over to the pass once past the end of the turn.
            on_pass = seconds > turn_seconds;
        }
    }
    let settled = (line.station(state.position).offset.metres() - 12.0).abs();
    // Pure pursuit cuts the corner where an arc meets a straight: 0.42 m at
    // a 3.5 m look-ahead on a 7 m turn.
    assert!(worst_in_turn < 0.5, "{worst_in_turn} off the turn");
    assert!(settled < 0.01, "{settled} off the pass");
    assert!(
        state
            .heading
            .angular_distance(GridCourse::new(180.0).expect("south"))
            < 0.5
    );
}
