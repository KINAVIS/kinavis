//! Reference lines and passes: cross-track against closed formulas, pass
//! numbers at the boundaries between passes, parallel curves against the
//! exact offset and there and back, the tracker's hysteresis and events.

#![allow(clippy::expect_used, clippy::panic, clippy::float_cmp)]

use kinavis::error::NavigationError;
use kinavis::region::{PlanarPoint, Polygon, Region};
use kinavis_agri::{
    locate, AbLine, AgriError, AgriEvent, CurveLine, Field, FieldPlane, PassIndex, PassPattern,
    PassTracker, PivotLine, ReferenceLine, TrackerConfig,
};
use kinavis_kernel::{Civil, Distance, GridCourse, Height, Instant, KernelError, Position, Utc};
use proptest::prelude::*;

fn metres(value: f64) -> Distance {
    Distance::from_metres(value).expect("finite")
}

fn point(east: f64, north: f64) -> PlanarPoint {
    PlanarPoint::from_metres(east, north).expect("finite")
}

fn plane() -> FieldPlane {
    FieldPlane::centred_at(
        Position::from_degrees(50.08, 14.42).expect("on Earth"),
        Height::above_ellipsoid(metres(280.0)),
    )
    .expect("a field plane")
}

fn noon() -> Instant<Utc> {
    Instant::<Utc>::from_civil(Civil::date(2026, 5, 4)).expect("a date")
}

proptest! {
    #[test]
    fn ab_cross_track_is_the_perpendicular_distance(
        ax in -5e3..5e3f64, ay in -5e3..5e3f64,
        bearing in 0.0..360.0f64, length in 1.0..2e3f64,
        px in -8e3..8e3f64, py in -8e3..8e3f64,
    ) {
        let (ux, uy) = (bearing.to_radians().sin(), bearing.to_radians().cos());
        let a = point(ax, ay);
        let b = point(ax + length * ux, ay + length * uy);
        let line = AbLine::from_points(plane(), a, b).expect("a line");
        let station = line.station(point(px, py));
        // Closed form: the left normal (−u_y, u_x) dotted with P − A.
        let expected = -uy * (px - ax) + ux * (py - ay);
        prop_assert!((station.offset.metres() - expected).abs() < 1e-9 * (1.0 + expected.abs()));
        prop_assert!(station.direction.angular_distance(GridCourse::wrap(bearing).expect("finite")) < 1e-9);
    }

    #[test]
    fn pivot_offset_is_the_radial_distance(
        cx in -5e3..5e3f64, cy in -5e3..5e3f64, radius in 1.0..800.0f64,
        bearing in 0.0..360.0f64, r in 0.5..1600.0f64,
    ) {
        let pivot = PivotLine::new(plane(), point(cx, cy), metres(radius)).expect("a pivot");
        let (s, c) = (bearing.to_radians().sin(), bearing.to_radians().cos());
        let station = pivot.station(point(cx + r * s, cy + r * c));
        prop_assert!((station.offset.metres() - (r - radius)).abs() < 1e-9 * (1.0 + r));
        // Clockwise: a quarter turn right of the bearing from the centre.
        let tangent = GridCourse::wrap(bearing + 90.0).expect("finite");
        prop_assert!(station.direction.angular_distance(tangent) < 1e-7);
    }

    #[test]
    fn a_pass_runs_from_one_midpoint_to_the_next(
        width in 0.5..60.0f64, overlap_share in 0.0..0.5f64, shift in -30.0..30.0f64,
        k in -400i32..400, within in 0.0..1.0f64,
    ) {
        let pattern = PassPattern::new(metres(width), metres(width * overlap_share))
            .and_then(|p| p.with_shift(metres(shift)))
            .expect("a pattern");
        let spacing = pattern.spacing().metres();
        let centre = shift + f64::from(k) * spacing;
        let inside = centre + (within - 0.5) * spacing * (1.0 - 1e-9);
        prop_assert_eq!(pattern.pass_at(metres(inside)), PassIndex::new(k));
        prop_assert!((pattern.offset_of(PassIndex::new(k)).metres() - centre).abs() < 1e-9 * (1.0 + centre.abs()));
    }

    #[test]
    fn a_gentle_curve_offset_there_and_back_is_itself(
        turns in prop::collection::vec(-4.0..4.0f64, 2..30),
        lengths in prop::collection::vec(5.0..25.0f64, 30),
        start in 0.0..360.0f64,
        k in -3i32..=3, width in 3.0..12.0f64,
    ) {
        // Total turning below 180°: the curve and every offset stay simple.
        let curve = walk(start, &turns, &lengths);
        let tolerance = metres(0.005);
        let d = f64::from(k) * width;
        let there: CurveLine<512> = curve.offset(metres(d), tolerance).expect("an offset");
        let back: CurveLine<1024> = there.offset(metres(-d), tolerance).expect("an offset back");
        // Inside a bend and back out again rounds the corner of a polyline:
        // the arc about the inner corner passes d·(1/cos(θ/2) − 1) inside the
        // vertex. A smooth curve has no corner and comes back exactly.
        let sharpest = turns.iter().fold(0.0f64, |m, t| m.max(t.abs())).to_radians();
        let bound = 0.011 + d.abs() * (1.0 / (sharpest / 2.0).cos() - 1.0) * 1.01;
        for vertex in back.vertices() {
            prop_assert!(curve.station(*vertex).offset.metres().abs() < bound, "{vertex}");
        }
        for vertex in curve.vertices() {
            prop_assert!(back.station(*vertex).offset.metres().abs() < bound, "{vertex}");
        }
    }

    #[test]
    fn every_vertex_of_a_parallel_curve_is_on_the_exact_offset(
        turns in prop::collection::vec(-50.0..50.0f64, 2..14),
        lengths in prop::collection::vec(2.0..30.0f64, 14),
        start in 0.0..360.0f64,
        d in -40.0..40.0f64,
    ) {
        let Ok(curve) = CurveLine::<16>::new(plane(), &walk_points(start, &turns, &lengths)) else {
            // The walk crossed itself; such curves are refused at construction.
            return Ok(());
        };
        let tolerance = 0.01;
        match curve.offset::<1024>(metres(d), metres(tolerance)) {
            Ok(parallel) => {
                for vertex in parallel.vertices() {
                    let error = (curve.station(*vertex).offset.metres() - d).abs();
                    prop_assert!(error <= 2.0 * tolerance + 1e-5, "{vertex} off by {error}");
                }
            }
            Err(AgriError::Navigation(NavigationError::OffsetCollapses { .. })) => {}
            Err(other) => prop_assert!(false, "{other}"),
        }
    }
}

/// Curve from a start bearing, a turn at each vertex (clockwise positive)
/// and segment lengths.
fn walk(start: f64, turns: &[f64], lengths: &[f64]) -> CurveLine<64> {
    CurveLine::new(plane(), &walk_points(start, turns, lengths)).expect("a simple curve")
}

fn walk_points(start: f64, turns: &[f64], lengths: &[f64]) -> Vec<PlanarPoint> {
    let mut points = vec![point(0.0, 0.0)];
    let (mut x, mut y, mut bearing) = (0.0, 0.0, start);
    for (i, length) in lengths.iter().take(turns.len() + 1).enumerate() {
        if i > 0 {
            bearing += turns.get(i - 1).copied().unwrap_or(0.0);
        }
        x += length * bearing.to_radians().sin();
        y += length * bearing.to_radians().cos();
        points.push(point(x, y));
    }
    points
}

#[test]
fn passes_of_a_curve_hold_the_machine_at_the_spacing() {
    // A quarter circle of 200 m drawn as a polyline, and the third pass inside it.
    let mut points = Vec::new();
    for step in 0..=30 {
        let angle = f64::from(step) * 3f64.to_radians();
        points.push(point(200.0 * angle.sin(), 200.0 * angle.cos()));
    }
    let curve = CurveLine::<64>::new(plane(), &points).expect("a curve");
    let pattern = PassPattern::new(metres(9.0), Distance::ZERO).expect("a pattern");
    // The polyline runs clockwise, so its right is the inside: pass −3 at 173 m.
    for step in 1..30 {
        let angle = (f64::from(step) * 3.0 + 1.5).to_radians();
        let at = point(173.0 * angle.sin(), 173.0 * angle.cos());
        let heading = GridCourse::wrap(angle.to_degrees() + 90.0).expect("finite");
        let view = locate(&curve, &pattern, at, heading);
        assert_eq!(view.pass, PassIndex::new(-3));
        assert!(!view.reversed);
        // Chords of 10.5 m sit 6.9 cm inside the arc at their middle.
        assert!(
            view.cross_track.metres().abs() < 0.08,
            "{}",
            view.cross_track
        );
    }
}

#[test]
fn the_tracker_holds_its_pass_past_the_midpoint_by_the_hysteresis() {
    let line = AbLine::from_point_and_course(
        plane(),
        PlanarPoint::ORIGIN,
        GridCourse::new(0.0).expect("north"),
    );
    let pattern = PassPattern::new(metres(10.0), Distance::ZERO).expect("a pattern");
    let config = TrackerConfig::new(metres(0.5), metres(20.0), metres(1.0)).expect("a config");
    let mut tracker = PassTracker::new(config);
    let north = GridCourse::new(0.0).expect("north");
    // West is left: pass +1 at 10 m west.
    let (view, events) = tracker.update(&line, &pattern, point(-10.2, 0.0), north, noon());
    assert_eq!(view.pass, PassIndex::new(1));
    assert!((view.cross_track.metres() + 0.2).abs() < 1e-9);
    assert!(events.is_empty());
    // Drifting east past the midpoint, but not past the hysteresis.
    let (view, events) = tracker.update(&line, &pattern, point(-4.2, 0.0), north, noon());
    assert_eq!(view.pass, PassIndex::new(1));
    assert!(matches!(events[..], [AgriEvent::OffLine { .. }]));
    // Still off line: reported once.
    let (_, events) = tracker.update(&line, &pattern, point(-4.1, 0.0), north, noon());
    assert!(events.is_empty());
    // Past the hysteresis: the reference pass, and nothing off line there.
    let (view, events) = tracker.update(&line, &pattern, point(-3.9, 0.0), north, noon());
    assert_eq!(view.pass, PassIndex::REFERENCE);
    assert!(matches!(
        events[..],
        [AgriEvent::PassChanged { from, to, .. }, AgriEvent::OffLine { .. }]
            if from == PassIndex::new(1) && to == PassIndex::REFERENCE
    ));
    // Locked, the tracker does not move whatever the offset.
    tracker.lock(PassIndex::new(2));
    let (view, _) = tracker.update(&line, &pattern, point(30.0, 0.0), north, noon());
    assert_eq!(view.pass, PassIndex::new(2));
    assert!((view.cross_track.metres() - 50.0).abs() < 1e-9);
}

#[test]
fn the_end_of_a_pass_is_announced_once_inside_a_field() {
    let plane = plane();
    let outer = Polygon::<8>::new(&[
        point(0.0, 0.0),
        point(300.0, 0.0),
        point(300.0, 500.0),
        point(0.0, 500.0),
    ])
    .expect("a field");
    let field = Field::new(plane, Region::<8, 8, 1>::without_holes(outer));
    let line = AbLine::from_points(plane, point(6.0, 0.0), point(6.0, 500.0)).expect("a line");
    let pattern = PassPattern::new(metres(12.0), Distance::ZERO).expect("a pattern");
    let config = TrackerConfig::new(metres(0.3), metres(25.0), metres(0.5)).expect("a config");
    let mut tracker = PassTracker::new(config);
    let south = GridCourse::new(180.0).expect("south");
    let mut announced = 0;
    for step in 0..48 {
        let at = point(42.0, 480.0 - f64::from(step) * 10.0);
        let (view, events) = tracker
            .update_within(&line, &pattern, &field, at, south, noon())
            .expect("one plane");
        assert_eq!(view.pass, PassIndex::new(-3));
        assert!(view.reversed);
        let remaining = view.to_end.expect("inside the field").metres();
        assert!((remaining - at.north_metres()).abs() < 1e-9);
        for event in &events {
            if let AgriEvent::EndOfPassApproaching { remaining, .. } = event {
                assert!(remaining.metres() <= 25.0);
                announced += 1;
            }
        }
    }
    assert_eq!(announced, 1);
    let elsewhere = FieldPlane::centred_at(
        Position::from_degrees(50.0, 14.0).expect("on Earth"),
        Height::above_ellipsoid(Distance::ZERO),
    )
    .expect("a plane");
    let foreign = AbLine::from_points(elsewhere, point(0.0, 0.0), point(0.0, 1.0)).expect("a line");
    assert_eq!(
        tracker
            .update_within(&foreign, &pattern, &field, point(1.0, 1.0), south, noon())
            .map(|_| ()),
        Err(AgriError::PlaneMismatch)
    );
}

#[test]
fn an_offset_that_does_not_fit_says_so() {
    let curve = walk(0.0, &[60.0, 60.0], &[50.0, 50.0, 50.0]);
    assert!(matches!(
        curve.offset::<4>(metres(20.0), metres(0.001)),
        Err(AgriError::Navigation(NavigationError::Kernel(
            KernelError::CapacityExceeded { .. }
        )))
    ));
}
