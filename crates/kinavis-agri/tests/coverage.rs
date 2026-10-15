//! Coverage: worked area against the exact geometry of the strip on straight
//! and curved passes, passes edge to edge without gap or overlap, overlap
//! measured, and section control with valve delays at worked ground and at
//! the field's edge.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::float_cmp,
    clippy::indexing_slicing,
    clippy::cast_lossless
)]

use core::time::Duration;

use kinavis::region::{PlanarPoint, Polygon, Region};
use kinavis_agri::{Boom, CoverageGrid, Field, FieldPlane, Pose, SectionControl};
use kinavis_kernel::{Distance, GridCourse, Height, Position, Speed};

fn metres(value: f64) -> Distance {
    Distance::from_metres(value).unwrap()
}

fn point(east: f64, north: f64) -> PlanarPoint {
    PlanarPoint::from_metres(east, north).unwrap()
}

fn plane() -> FieldPlane {
    FieldPlane::centred_at(
        Position::from_degrees(52.5, 13.4).unwrap(),
        Height::above_ellipsoid(metres(40.0)),
    )
    .unwrap()
}

/// A 51.2 m square at 10 cm.
type Grid = CoverageGrid<8, 512>;

fn grid() -> Box<Grid> {
    Box::new(Grid::new(plane(), PlanarPoint::ORIGIN, metres(0.1)).unwrap())
}

fn pose(east: f64, north: f64, course: f64) -> Pose {
    Pose {
        position: point(east, north),
        course: GridCourse::wrap(course).unwrap(),
    }
}

/// Drives a straight pass in steps, marking with every section on.
fn drive<const S: usize>(
    grid: &mut Grid,
    boom: &Boom<S>,
    start: (f64, f64),
    course: f64,
    length: f64,
    steps: usize,
) {
    let (s, c) = course.to_radians().sin_cos();
    for k in 0..steps {
        let a = length * k as f64 / steps as f64;
        let b = length * (k + 1) as f64 / steps as f64;
        let _ = grid.mark(
            boom,
            pose(start.0 + a * s, start.1 + a * c, course),
            pose(start.0 + b * s, start.1 + b * c, course),
            [true; S],
        );
    }
}

#[test]
fn a_straight_pass_works_its_rectangle() {
    for course in [0.0_f64, 30.0, 47.0, 90.0, 213.0] {
        let mut grid = grid();
        let boom = Boom::new([metres(6.0)], Distance::ZERO).unwrap();
        let (s, c) = course.to_radians().sin_cos();
        let start = (25.6 - 20.0 * s, 25.6 - 20.0 * c);
        drive(&mut grid, &boom, start, course, 40.0, 80);
        let exact = 6.0 * 40.0;
        // A cell is mis-counted only where the edge crosses it.
        let bound = 2.0 * (40.0 + 6.0) * 0.1;
        assert!(
            (grid.worked_area() - exact).abs() < bound,
            "{course}°: {} against {exact}",
            grid.worked_area()
        );
        assert_eq!(
            grid.overlap_area(),
            0.0,
            "{course}°: a pass does not overlap itself"
        );
    }
}

#[test]
fn a_curved_pass_works_its_annulus() {
    let mut grid = grid();
    let boom = Boom::new([metres(2.0), metres(2.0), metres(2.0)], Distance::ZERO).unwrap();
    let (radius, centre) = (15.0, (25.6, 25.6));
    // Half a circle clockwise, in steps of half a degree.
    let at = |angle: f64| {
        let (s, c) = angle.to_radians().sin_cos();
        pose(centre.0 + radius * s, centre.1 + radius * c, angle + 90.0)
    };
    let steps = 360;
    for k in 0..steps {
        let _ = grid.mark(
            &boom,
            at(k as f64 * 0.5),
            at((k + 1) as f64 * 0.5),
            [true; 3],
        );
    }
    // θ R w, less the chords' sagittas on the outside and plus them inside.
    let exact = core::f64::consts::PI * radius * 6.0;
    let chord = 2.0 * 0.25f64.to_radians().sin();
    let polygon = 0.5
        * (18.0f64.powi(2) - 12.0f64.powi(2))
        * steps as f64
        * chord
        * 0.25f64.to_radians().cos();
    assert!(
        (polygon - exact).abs() < 0.01,
        "the polygon of the strip is the annulus to 1 cm²"
    );
    let bound = (core::f64::consts::PI * (18.0 + 12.0) + 12.0) * 0.1;
    assert!(
        (grid.worked_area() - polygon).abs() < bound,
        "{} against {polygon}",
        grid.worked_area()
    );
    assert_eq!(grid.overlap_area(), 0.0);
}

#[test]
fn passes_edge_to_edge_leave_no_gap_and_no_overlap() {
    for (course, offset) in [(0.0_f64, 0.0), (17.0, 0.031), (63.5, 0.077), (90.0, 0.05)] {
        let mut grid = grid();
        let width = 4.0;
        let boom = Boom::new([metres(width)], Distance::ZERO).unwrap();
        let (s, c) = course.to_radians().sin_cos();
        // Four passes back and forth, each `width` to the right of the last.
        let origin = (10.0 + offset, 8.0 + offset);
        let length = 25.0;
        for pass in 0..4 {
            let across = width * (pass as f64 + 0.5);
            let start = (origin.0 + across * c, origin.1 - across * s);
            if pass % 2 == 0 {
                drive(&mut grid, &boom, start, course, length, 50);
            } else {
                let end = (start.0 + length * s, start.1 + length * c);
                drive(&mut grid, &boom, end, course + 180.0, length, 50);
            }
        }
        assert_eq!(grid.overlap_area(), 0.0, "{course}°");
        // Every cell whose centre lies in the union is worked; centres on its
        // outer edge may go either way.
        let (mut inside, mut on_edge) = (0usize, 0usize);
        for row in 0..512 {
            for column in 0..512 {
                let (x, y) = (
                    (column as f64 + 0.5) * 0.1 - origin.0,
                    (row as f64 + 0.5) * 0.1 - origin.1,
                );
                let along = x * s + y * c;
                let across = x * c - y * s;
                let near = |value: f64, end: f64| value.abs() < 1e-9 || (value - end).abs() < 1e-9;
                let within = (-1e-9..=length + 1e-9).contains(&along)
                    && (-1e-9..=4.0 * width + 1e-9).contains(&across);
                if within && (near(along, length) || near(across, 4.0 * width)) {
                    on_edge += 1;
                    continue;
                }
                if (0.0..length).contains(&along) && (0.0..4.0 * width).contains(&across) {
                    inside += 1;
                    assert!(
                        grid.is_worked(point(
                            (column as f64 + 0.5) * 0.1,
                            (row as f64 + 0.5) * 0.1
                        )),
                        "a gap at {column}, {row}"
                    );
                }
            }
        }
        let worked = (grid.worked_area() / 0.01).round() as usize;
        assert!(
            (inside..=inside + on_edge).contains(&worked),
            "{course}°: {worked} worked, {inside} inside, {on_edge} on the edge"
        );
    }
}

#[test]
fn an_overlap_between_passes_is_measured() {
    let mut grid = grid();
    let boom = Boom::new([metres(6.0)], Distance::ZERO).unwrap();
    // Three passes north, 5.5 m apart: two strips of 0.5 m by 30 m overlap.
    for pass in 0..3 {
        drive(
            &mut grid,
            &boom,
            (10.0 + 5.5 * pass as f64, 10.0),
            0.0,
            30.0,
            60,
        );
    }
    let exact = 2.0 * 0.5 * 30.0;
    assert!(
        (grid.overlap_area() - exact).abs() < 2.0 * 2.0 * 30.0 * 0.1 / 2.0,
        "{}",
        grid.overlap_area()
    );
}

/// Drives north at 2 m/s in steps of 0.1 s from `from` to `to`, the
/// sections commanded and their valves acting after the delays `on` and
/// `off`. Returns, for each section, where along the pass it switched and
/// to what.
fn sections_over(
    grid: &mut Grid,
    field: Option<&Field<8, 8, 1>>,
    control: &SectionControl,
    on: f64,
    off: f64,
    from: f64,
    to: f64,
) -> [Vec<(f64, bool)>; 3] {
    let boom = Boom::new([metres(4.0), metres(4.0), metres(4.0)], Distance::ZERO).unwrap();
    let speed = Speed::from_metres_per_second(2.0).unwrap();
    let dt = 0.1;
    let mut actual = [true; 3];
    let mut pending: [Option<(bool, f64)>; 3] = [None; 3];
    let mut switches: [Vec<(f64, bool)>; 3] = Default::default();
    let mut y = from;
    let mut time = 0.0;
    while y < to {
        let here = pose(25.6, y, 0.0);
        let commands = control
            .commands(grid, field, &boom, here, speed, actual)
            .unwrap();
        for section in 0..3 {
            if commands[section] == actual[section] {
                pending[section] = None;
            } else {
                let delay = if commands[section] { on } else { off };
                if pending[section].is_none_or(|(state, _)| state != commands[section]) {
                    pending[section] = Some((commands[section], time + delay));
                }
            }
            if let Some((state, due)) = pending[section] {
                if time >= due - 1e-9 {
                    actual[section] = state;
                    pending[section] = None;
                    switches[section].push((y, state));
                }
            }
        }
        let next = pose(25.6, y + 2.0 * dt, 0.0);
        let _ = grid.mark(&boom, here, next, actual);
        y += 2.0 * dt;
        time += dt;
    }
    switches
}

#[test]
fn sections_stop_at_worked_ground_and_start_again_past_it() {
    let mut grid = grid();
    // A headland band already worked, 30 m to 38 m north.
    let wide = Boom::new([metres(20.0)], Distance::ZERO).unwrap();
    let _ = grid.mark(&wide, pose(25.6, 30.0, 0.0), pose(25.6, 38.0, 0.0), [true]);
    let before = grid.worked_area();
    let control =
        SectionControl::new(Duration::from_millis(600), Duration::from_millis(300), 0.5).unwrap();
    let switches = sections_over(&mut grid, None, &control, 0.6, 0.3, 5.0, 50.0);
    for section in &switches {
        // Off where the band begins, on where it ends, each within a step.
        let off = section
            .iter()
            .find(|(_, state)| !state)
            .expect("switched off")
            .0;
        let on = section
            .iter()
            .find(|(_, state)| *state)
            .expect("switched on")
            .0;
        assert!((off - 30.0).abs() <= 0.2 + 1e-9, "off at {off}");
        assert!((on - 38.0).abs() <= 0.2 + 1e-9, "on at {on}");
    }
    // Overlap into the band, and ground missed before or after it, each
    // within a step of the 12 m boom.
    let overlap = grid.overlap_area();
    assert!(overlap <= 12.0 * 0.4 + 0.5, "overlap {overlap}");
    let worked_by_pass = grid.worked_area() - before;
    let expected = 12.0 * (50.0 - 5.0 - 8.0);
    assert!(
        (worked_by_pass - expected).abs() <= 12.0 * 0.4 + 1.0,
        "{worked_by_pass} against {expected}"
    );
}

#[test]
fn sections_stop_one_by_one_at_a_slanting_edge() {
    let mut grid = grid();
    // A band whose southern edge runs south-west to north-east.
    let wide = Boom::new([metres(30.0)], Distance::ZERO).unwrap();
    let _ = grid.mark(
        &wide,
        pose(10.0, 20.0, 45.0),
        pose(30.0, 40.0, 45.0),
        [true],
    );
    let control =
        SectionControl::new(Duration::from_millis(400), Duration::from_millis(400), 0.5).unwrap();
    let switches = sections_over(&mut grid, None, &control, 0.4, 0.4, 2.0, 30.0);
    let offs: Vec<f64> = switches
        .iter()
        .map(|s| s.iter().find(|(_, on)| !on).expect("off").0)
        .collect();
    // The left section meets the band first.
    assert!(offs[0] < offs[1] && offs[1] < offs[2], "{offs:?}");
}

#[test]
fn sections_stop_at_the_edge_of_the_field() {
    let mut grid = grid();
    let outer = Polygon::<8>::new(&[
        point(0.0, 0.0),
        point(51.2, 0.0),
        point(51.2, 40.0),
        point(0.0, 40.0),
    ])
    .unwrap();
    let field = Field::new(plane(), Region::<8, 8, 1>::without_holes(outer));
    let control =
        SectionControl::new(Duration::from_millis(500), Duration::from_millis(500), 0.5).unwrap();
    let switches = sections_over(&mut grid, Some(&field), &control, 0.5, 0.5, 5.0, 48.0);
    for section in &switches {
        let off = section.iter().find(|(_, state)| !state).expect("off").0;
        assert!((off - 40.0).abs() <= 0.2 + 1e-9, "off at {off}");
    }
    let outside = grid.worked_area() - 12.0 * (40.0 - 5.0);
    assert!(outside.abs() <= 12.0 * 0.4, "{outside} m² worked outside");
}
