//! Turns that reverse: Reeds–Shepp paths against OMPL, never longer than
//! Dubins, the gear against the motion, the fishtail onto a close pass and
//! its fit in a headland the omega turn does not fit, its sections between
//! cusps as reference lines, driven by both trackers.

#![allow(
    clippy::expect_used,
    clippy::panic,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::indexing_slicing
)]

use core::f64::consts::{PI, TAU};
use core::time::Duration;

use kinavis::region::{PlanarPoint, Polygon, Region};
use kinavis_agri::{
    bicycle_step, AbLine, Field, FieldPlane, Gear, GearedSection, PassIndex, PassPattern,
    PathTracker, Pose, PurePursuit, ReedsSheppPath, ReferenceLine, Stanley, Steer, TurnPath,
    VehicleGeometry, VehicleState,
};
use kinavis_kernel::{Angle, Distance, GridCourse, Height, Position, RateOfTurn, Speed};
use proptest::prelude::*;

const REFERENCE: &str = include_str!("data/reeds_shepp_reference.txt");

/// OMPL's table of path types.
const TYPES: [&str; 18] = [
    "LRL", "RLR", "LRLR", "RLRL", "LRSL", "RLSR", "LSRL", "RSLR", "LRSR", "RLSL", "RSRL", "LSLR",
    "LSR", "RSL", "LSL", "RSR", "LRSLR", "RLSRL",
];

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

fn north() -> GridCourse {
    GridCourse::new(0.0).expect("north")
}

fn letter(steer: Steer) -> char {
    match steer {
        Steer::Left => 'L',
        Steer::Right => 'R',
        Steer::Straight => 'S',
    }
}

/// Signed pieces: negative in reverse.
fn signed(path: &ReedsSheppPath) -> Vec<(char, f64)> {
    path.pieces()
        .map(|piece| {
            let sign = if piece.gear == Gear::Reverse {
                -1.0
            } else {
                1.0
            };
            (letter(piece.steer), sign * piece.length.metres())
        })
        .collect()
}

/// OMPL's pieces with the zero ones dropped and like neighbours merged.
fn tidied(word: &str, lengths: &[f64]) -> Vec<(char, f64)> {
    let mut out: Vec<(char, f64)> = Vec::new();
    for (steer, &length) in word.chars().zip(lengths) {
        if length.abs() <= 1e-9 {
            continue;
        }
        match out.last_mut() {
            Some(last) if last.0 == steer && (last.1 < 0.0) == (length < 0.0) => last.1 += length,
            _ => out.push((steer, length)),
        }
    }
    out
}

fn assert_reaches(path: &ReedsSheppPath, to: Pose, context: &str) {
    let end = path.end();
    let miss = end.position.distance_to(to.position).metres();
    let turn = end.course.angular_distance(to.course);
    assert!(
        miss < 1e-7 && turn < 1e-7,
        "{context}: {miss} m, {turn}° off"
    );
}

#[test]
fn the_shortest_path_agrees_with_ompl() {
    let (mut checked, mut same_word, mut worst) = (0, 0, 0.0f64);
    for row in REFERENCE.lines().filter(|line| !line.starts_with('#')) {
        let v: Vec<f64> = row
            .split_ascii_whitespace()
            .map(|field| field.parse().expect("a number"))
            .collect();
        let (from, to, radius) = (pose(v[0], v[1], v[2]), pose(v[3], v[4], v[5]), v[6]);
        let path = ReedsSheppPath::shortest(plane(), from, to, metres(radius)).expect("a path");
        let theirs: f64 = v[8..13].iter().map(|l| l.abs()).sum();
        let ours = path.length().metres();
        assert!(
            (ours - theirs).abs() < 1e-7,
            "{row}: {ours} against {theirs}"
        );
        assert_reaches(&path, to, row);
        // Of equal lengths, never more in reverse.
        let reverse = |p: &[(char, f64)]| {
            p.iter()
                .filter(|(_, l)| *l < 0.0)
                .map(|(_, l)| -l)
                .sum::<f64>()
        };
        let theirs_back: f64 = v[8..13].iter().filter(|l| **l < 0.0).map(|l| -l).sum();
        assert!(
            reverse(&signed(&path)) <= theirs_back + 1e-7,
            "{row}: more in reverse"
        );
        let expected = tidied(TYPES[v[7] as usize], &v[8..13]);
        let pieces = signed(&path);
        let word = |p: &[(char, f64)]| p.iter().map(|(c, l)| (*c, *l < 0.0)).collect::<Vec<_>>();
        if word(&pieces) == word(&expected) {
            for ((_, mine), (_, their)) in pieces.iter().zip(&expected) {
                worst = worst.max((mine - their).abs());
            }
            same_word += 1;
        }
        checked += 1;
    }
    assert_eq!(checked, 3600);
    std::println!("{same_word} of {checked} in the same word");
    // Where two words tie the choice may differ: mirror images of one
    // another, or one with less in reverse preferred here.
    assert!(same_word * 10 >= checked * 8, "{same_word} of {checked}");
    assert!(worst < 1e-7, "{worst}");
}

fn line() -> AbLine {
    AbLine::from_point_and_course(plane(), PlanarPoint::ORIGIN, north())
}

fn exit() -> Pose {
    Pose {
        position: point(0.0, 300.0),
        course: north(),
    }
}

#[test]
fn onto_a_close_pass_the_turn_reverses_and_is_shorter_than_the_omega() {
    for (width, radius) in [(4.0, 6.0), (6.0, 6.0), (3.0, 8.0), (9.0, 5.0)] {
        let pattern = PassPattern::new(metres(width), Distance::ZERO).expect("a pattern");
        let to = PassIndex::new(1);
        let turn =
            ReedsSheppPath::to_pass(&line(), &pattern, exit(), to, metres(radius)).expect("a turn");
        let omega =
            TurnPath::to_pass(&line(), &pattern, exit(), to, metres(radius)).expect("an omega");
        assert!(turn.cusps().count() >= 1, "{width} m at {radius} m");
        assert!(turn.length() < omega.length(), "{width} m at {radius} m");
        assert_reaches(&turn, omega.end(), "onto the pass");
    }
}

#[test]
fn onto_a_pass_far_enough_away_the_turn_is_the_dubins_u() {
    let pattern = PassPattern::new(metres(15.0), Distance::ZERO).expect("a pattern");
    let to = PassIndex::new(1);
    let turn = ReedsSheppPath::to_pass(&line(), &pattern, exit(), to, metres(6.0)).expect("a turn");
    assert_eq!(turn.cusps().count(), 0);
    assert!(turn.pieces().all(|piece| piece.gear == Gear::Forward));
    let expected = PI * 6.0 + (15.0 - 12.0);
    assert!((turn.length().metres() - expected).abs() < 1e-9);
}

#[test]
fn degenerate_paths_have_their_obvious_pieces() {
    let here = pose(5.0, 5.0, 1.0);
    let still = ReedsSheppPath::shortest(plane(), here, here, metres(6.0)).expect("a path");
    assert_eq!(still.pieces().count(), 0);
    assert_eq!(still.gear_at(Distance::ZERO), Gear::Forward);
    assert!(still.length().metres() < 1e-12);

    let (c, s) = (1f64.cos(), 1f64.sin());
    let ahead = pose(5.0 + 30.0 * c, 5.0 + 30.0 * s, 1.0);
    let forward = ReedsSheppPath::shortest(plane(), here, ahead, metres(6.0)).expect("a path");
    let pieces = signed(&forward);
    assert_eq!(pieces.len(), 1);
    assert_eq!(pieces[0].0, 'S');
    assert!((pieces[0].1 - 30.0).abs() < 1e-9);

    // Straight back: one piece in reverse, the heading kept.
    let behind = pose(5.0 - 30.0 * c, 5.0 - 30.0 * s, 1.0);
    let back = ReedsSheppPath::shortest(plane(), here, behind, metres(6.0)).expect("a path");
    let pieces = signed(&back);
    assert_eq!(pieces.len(), 1);
    assert_eq!(pieces[0].0, 'S');
    assert!((pieces[0].1 + 30.0).abs() < 1e-9);
    assert_eq!(back.gear_at(metres(10.0)), Gear::Reverse);
    let half = back.pose_at(metres(15.0));
    assert!(half.course.angular_distance(here.course) < 1e-9);
    assert!(
        half.position
            .distance_to(point(5.0 - 15.0 * c, 5.0 - 15.0 * s))
            .metres()
            < 1e-9
    );
}

/// Steps along the path: continuous, moving the way the gear says, cusps
/// where the motion turns over.
fn check_motion(path: &ReedsSheppPath) {
    let total = path.length().metres();
    let cusps: Vec<f64> = path.cusps().map(Distance::metres).collect();
    let step = 0.05;
    let count = (total / step).ceil() as usize;
    let mut previous = path.pose_at(Distance::ZERO);
    for index in 1..=count {
        let along = (index as f64 * step).min(total);
        let here = path.pose_at(metres(along));
        let (de, dn) = (
            here.position.east_metres() - previous.position.east_metres(),
            here.position.north_metres() - previous.position.north_metres(),
        );
        let moved = de.hypot(dn);
        assert!(moved <= step + 1e-9, "a jump of {moved} m at {along}");
        // The heading against the motion of the step's middle.
        let middle = path.pose_at(metres(along - step / 2.0));
        let heading = middle.course.degrees().to_radians();
        let ahead = de * heading.sin() + dn * heading.cos();
        let near_cusp = cusps.iter().any(|c| (c - along).abs() < step * 1.5);
        if moved > step / 2.0 && !near_cusp {
            let gear = path.gear_at(metres(along - step / 2.0));
            assert_eq!(gear == Gear::Forward, ahead > 0.0, "at {along} of {total}");
        }
        previous = here;
    }
}

#[test]
fn the_machine_moves_the_way_the_gear_says() {
    for row in REFERENCE
        .lines()
        .filter(|line| !line.starts_with('#'))
        .step_by(37)
    {
        let v: Vec<f64> = row
            .split_ascii_whitespace()
            .map(|field| field.parse().expect("a number"))
            .collect();
        let path = ReedsSheppPath::shortest(
            plane(),
            pose(v[0], v[1], v[2]),
            pose(v[3], v[4], v[5]),
            metres(v[6]),
        )
        .expect("a path");
        check_motion(&path);
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
fn a_fishtail_fits_a_headland_the_omega_does_not() {
    // A 4 m implement onto the next pass on a 6 m turn.
    let pattern = PassPattern::new(metres(4.0), Distance::ZERO).expect("a pattern");
    let to = PassIndex::new(1);
    let fishtail =
        ReedsSheppPath::to_pass(&line(), &pattern, exit(), to, metres(6.0)).expect("a turn");
    let omega = TurnPath::to_pass(&line(), &pattern, exit(), to, metres(6.0)).expect("an omega");
    let (half, step) = (metres(2.0), metres(0.25));
    let least = |clear: &dyn Fn(&Field<8, 8, 1>) -> bool| {
        (0..=200)
            .map(|quarter| f64::from(quarter) * 0.25)
            .find(|&headland| clear(&field(headland)))
            .expect("clear within 50 m")
    };
    let fishtail_headland = least(&|f| {
        fishtail
            .first_conflict(f, half, step)
            .expect("one plane")
            .is_none()
    });
    let omega_headland = least(&|f| {
        omega
            .first_conflict(f, half, step)
            .expect("one plane")
            .is_none()
    });
    assert!(
        fishtail_headland + 2.0 < omega_headland,
        "{fishtail_headland} m against {omega_headland} m"
    );
}

proptest! {
    #[test]
    fn every_path_reaches_its_goal_and_is_never_longer_than_dubins(
        x0 in -200.0..200.0f64, y0 in -200.0..200.0f64, h0 in 0.0..TAU,
        x1 in -200.0..200.0f64, y1 in -200.0..200.0f64, h1 in 0.0..TAU,
        radius in 0.5..30.0f64,
    ) {
        let (from, to) = (pose(x0, y0, h0), pose(x1, y1, h1));
        let path = ReedsSheppPath::shortest(plane(), from, to, metres(radius)).expect("a path");
        assert_reaches(&path, to, "random poses");
        let dubins = TurnPath::dubins(plane(), from, to, metres(radius)).expect("a path");
        prop_assert!(path.length().metres() <= dubins.length().metres() + 1e-7);
        prop_assert!(path.pieces().count() <= kinavis_agri::MAX_REVERSING_PIECES);
        let cusps = path.cusps().count();
        prop_assert!(cusps < path.pieces().count().max(1));
    }
}

/// Course of the motion at a pose along a section.
fn travel(pose: Pose, gear: Gear) -> GridCourse {
    match gear {
        Gear::Forward => pose.course,
        Gear::Reverse => pose.course.reciprocal(),
    }
}

/// The sections against the path: one more than the cusps, end to end,
/// alternating gears, each a reference line through its own pieces with the
/// direction of travel.
fn check_sections(path: &ReedsSheppPath) {
    let sections: Vec<GearedSection> = path.sections().collect();
    if path.pieces().count() == 0 {
        assert_eq!(sections, []);
        return;
    }
    assert_eq!(sections.len(), path.cusps().count() + 1);
    let mut along = 0.0;
    for (index, section) in sections.iter().enumerate() {
        assert!((section.start().metres() - along).abs() < 1e-9);
        if let Some(next) = sections.get(index + 1) {
            assert_ne!(section.gear(), next.gear());
        }
        let length = section.length().metres();
        let start = path.pose_at(metres(along));
        assert!(
            start
                .position
                .distance_to(section.start_pose().position)
                .metres()
                < 1e-9
        );
        assert!(start.course.angular_distance(section.start_pose().course) < 1e-7);
        let end = path.pose_at(metres(along + length));
        assert!(
            end.position
                .distance_to(section.end_pose().position)
                .metres()
                < 1e-9
        );
        for step in 1..20 {
            let into = length * f64::from(step) / 20.0;
            let at = path.pose_at(metres(along + into));
            let station = section.station(at.position);
            assert!(station.offset.metres().abs() < 1e-9, "{into} into {index}");
            assert!(
                station
                    .direction
                    .angular_distance(travel(at, section.gear()))
                    < 1e-6
            );
            // Arcs of one section never come back near themselves.
            let remaining = section.remaining(at.position).metres();
            assert!((remaining - (length - into)).abs() < 1e-7, "{remaining}");
        }
        along += length;
    }
    assert!((along - path.length().metres()).abs() < 1e-9);
}

#[test]
fn a_fishtail_is_three_sections_and_its_reverse_runs_backwards() {
    let pattern = PassPattern::new(metres(4.0), Distance::ZERO).expect("a pattern");
    let turn = ReedsSheppPath::to_pass(&line(), &pattern, exit(), PassIndex::new(1), metres(6.0))
        .expect("a turn");
    check_sections(&turn);
    let gears: Vec<Gear> = turn.sections().map(|s| s.gear()).collect();
    assert_eq!(gears, [Gear::Forward, Gear::Reverse, Gear::Forward]);
    let backing = turn.sections().nth(1).expect("a reverse section");
    // Half a metre left of the travel is half a metre right of the heading.
    let middle = turn.pose_at(metres(
        backing.start().metres() + backing.length().metres() / 2.0,
    ));
    let (north, east) = middle.course.components(1.0);
    let right_of_heading = point(
        middle.position.east_metres() + 0.5 * north,
        middle.position.north_metres() - 0.5 * east,
    );
    let offset = backing.station(right_of_heading).offset.metres();
    assert!((offset - 0.5).abs() < 1e-9, "{offset}");
}

/// Worst cross-track error of the rear axle in each section, and its miss
/// at the end of the turn, driven at 1 m/s with the wheels slewing and a
/// stop at each cusp while the wheels turn over.
fn drive<T: PathTracker>(
    tracker: &T,
    turn: &ReedsSheppPath,
    vehicle: &VehicleGeometry,
) -> (Vec<f64>, f64, f64) {
    let pattern = PassPattern::new(metres(1.0), Distance::ZERO).expect("a pattern");
    let dt = Duration::from_millis(20);
    let start = turn.start();
    let mut state = VehicleState {
        position: start.position,
        heading: start.course,
        speed: Speed::ZERO,
    };
    let mut wheels = Angle::ZERO;
    let mut worst = Vec::new();
    for section in turn.sections() {
        let speed = match section.gear() {
            Gear::Forward => 1.0,
            Gear::Reverse => -1.0,
        };
        state.speed = Speed::from_metres_per_second(speed).expect("finite");
        // At a standstill, turn the wheels to the first command.
        for _ in 0..100 {
            let command = tracker
                .steer(&section, &pattern, PassIndex::REFERENCE, vehicle, &state)
                .expect("a command");
            wheels = vehicle.slew(wheels, command.angle, dt);
        }
        let mut error: f64 = 0.0;
        let steps = (section.length().metres() / 0.02 * 2.0) as usize;
        for _ in 0..steps {
            if section.remaining(state.position).metres() <= 0.0 {
                break;
            }
            let command = tracker
                .steer(&section, &pattern, PassIndex::REFERENCE, vehicle, &state)
                .expect("a command");
            wheels = vehicle.slew(wheels, command.angle, dt);
            state = bicycle_step(&state, vehicle, wheels, dt).expect("a step");
            error = error.max(section.station(state.position).offset.metres().abs());
        }
        worst.push(error);
    }
    let end = turn.end();
    (
        worst,
        state.position.distance_to(end.position).metres(),
        state.heading.angular_distance(end.course),
    )
}

fn turn_vehicle() -> VehicleGeometry {
    VehicleGeometry::new(
        metres(2.9),
        Angle::from_degrees(40.0).expect("finite"),
        RateOfTurn::from_radians_per_second(30f64.to_radians()).expect("finite"),
    )
    .expect("a vehicle")
}

fn fishtail(width: f64, radius: f64) -> ReedsSheppPath {
    let pattern = PassPattern::new(metres(width), Distance::ZERO).expect("a pattern");
    ReedsSheppPath::to_pass(&line(), &pattern, exit(), PassIndex::new(1), metres(radius))
        .expect("a turn")
}

#[test]
fn pure_pursuit_drives_a_fishtail_through_its_cusps() {
    let pursuit =
        PurePursuit::new(metres(2.0), Duration::from_millis(500), metres(8.0)).expect("a tracker");
    for (width, radius) in [(4.0, 6.0), (6.0, 6.0), (3.0, 8.0)] {
        let turn = fishtail(width, radius);
        let (worst, miss, heading) = drive(&pursuit, &turn, &turn_vehicle());
        assert_eq!(worst.len(), 3);
        // Cutting in where an arc meets the run-on at a cusp: 9 cm at a 2.5 m
        // look-ahead on a 6 m turn.
        for error in &worst {
            assert!(*error < 0.12, "{width} m at {radius} m: {worst:?}");
        }
        assert!(miss < 0.1, "{width} m at {radius} m: {miss} m");
        assert!(heading < 5.0, "{width} m at {radius} m: {heading}°");
    }
}

#[test]
fn stanley_backs_the_rear_axle_round_the_reverse_arc_of_a_fishtail() {
    let vehicle = turn_vehicle();
    let pattern = PassPattern::new(metres(1.0), Distance::ZERO).expect("a pattern");
    let dt = Duration::from_millis(20);
    for gain in [0.5, 1.0, 2.0] {
        let stanley = Stanley::new(gain, Speed::from_metres_per_second(0.3).expect("finite"))
            .expect("a tracker");
        let section = fishtail(4.0, 6.0)
            .sections()
            .nth(1)
            .expect("a reverse section");
        let start = section.start_pose();
        let mut state = VehicleState {
            position: start.position,
            heading: start.course,
            speed: Speed::from_metres_per_second(-1.0).expect("finite"),
        };
        let mut wheels = Angle::ZERO;
        for _ in 0..200 {
            let command = stanley
                .steer(&section, &pattern, PassIndex::REFERENCE, &vehicle, &state)
                .expect("a command");
            wheels = vehicle.slew(wheels, command.angle, dt);
        }
        let mut worst: f64 = 0.0;
        for _ in 0..5000 {
            if section.remaining(state.position).metres() <= 0.0 {
                break;
            }
            let command = stanley
                .steer(&section, &pattern, PassIndex::REFERENCE, &vehicle, &state)
                .expect("a command");
            wheels = vehicle.slew(wheels, command.angle, dt);
            state = bicycle_step(&state, &vehicle, wheels, dt).expect("a step");
            worst = worst.max(section.station(state.position).offset.metres().abs());
        }
        assert!(section.remaining(state.position).metres() <= 0.0);
        // Without the curvature fed forward the rear axle would settle
        // (|v| + v_s)·L / (k·R) off the arc: 0.63 m at k = 1.
        assert!(worst < 0.04, "gain {gain}: {worst} m");
    }
}

proptest! {
    #[test]
    fn sections_split_the_path_at_its_cusps(
        x1 in -40.0..40.0f64, y1 in -40.0..40.0f64, h1 in 0.0..TAU,
        radius in 0.5..15.0f64,
    ) {
        let path = ReedsSheppPath::shortest(plane(), pose(0.0, 0.0, 0.0), pose(x1, y1, h1), metres(radius))
            .expect("a path");
        check_sections(&path);
    }
}
