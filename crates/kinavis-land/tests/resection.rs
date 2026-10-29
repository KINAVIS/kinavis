//! Intersection and resection on constructed figures with known answers:
//! bearings and readings computed from a chosen observer, inside and outside
//! the triangle of landmarks, both orientations, near the danger circle; and
//! a resection from ellipsoidal azimuths against the true point and the
//! marine fix of `kinavis`.

#![allow(
    clippy::expect_used,
    clippy::panic,
    clippy::float_cmp,
    clippy::indexing_slicing
)]

use kinavis::fix::{bearing_fix, PositionLine};
use kinavis::sailings::geodesic;
use kinavis_kernel::{Angle, Distance, GridBearing, Position};
use kinavis_land::{
    grid_bearing, intersection, resection, three_point_resection, GridPoint, LandError, MapGrid,
    Sighting,
};
use proptest::prelude::*;

fn point(e: f64, n: f64) -> GridPoint {
    GridPoint::from_metres(e, n).expect("finite")
}

fn metres(p: GridPoint) -> (f64, f64) {
    (p.easting.metres(), p.northing.metres())
}

fn close(a: GridPoint, b: GridPoint, tolerance: f64) -> bool {
    let (ae, an) = metres(a);
    let (be, bn) = metres(b);
    (ae - be).hypot(an - bn) < tolerance
}

/// Readings to `landmarks` from `observer` on a circle whose zero lies at
/// grid bearing `zero`.
fn readings(observer: GridPoint, landmarks: [GridPoint; 3], zero: f64) -> [Angle; 3] {
    landmarks.map(|landmark| {
        let bearing = grid_bearing(observer, landmark).expect("apart").degrees();
        Angle::from_degrees((bearing - zero).rem_euclid(360.0)).expect("finite")
    })
}

#[test]
fn an_intersection_lands_on_the_point_both_bearings_show() {
    let target = point(5_230.0, 7_410.0);
    for (a, b) in [
        (point(1_000.0, 2_000.0), point(9_000.0, 1_500.0)),
        (point(5_230.0, 0.0), point(0.0, 7_410.0)),
        (point(-3_000.0, 9_000.0), point(4_000.0, 12_000.0)),
    ] {
        let found = intersection(
            a,
            grid_bearing(a, target).expect("apart"),
            b,
            grid_bearing(b, target).expect("apart"),
        )
        .expect("a cut");
        assert!(close(found, target, 1e-6), "{found:?}");
    }
}

#[test]
fn an_intersection_behind_or_parallel_is_refused() {
    let (a, b) = (point(0.0, 0.0), point(1_000.0, 0.0));
    let north = GridBearing::new(0.0).expect("000");
    assert!(matches!(
        intersection(a, north, b, north),
        Err(LandError::Navigation(
            kinavis::error::NavigationError::Parallel { .. }
        ))
    ));
    // Lines that meet south of the base while the bearings point north.
    let in_ = GridBearing::new(10.0).expect("010");
    let out = GridBearing::new(350.0).expect("350");
    assert!(matches!(
        intersection(a, out, b, in_),
        Err(LandError::Navigation(
            kinavis::error::NavigationError::NoSolution { .. }
        ))
    ));
}

#[test]
fn a_resection_by_bearings_is_exact_and_reports_its_misfit() {
    let observer = point(3_000.0, 4_000.0);
    let landmarks = [
        point(500.0, 6_000.0),
        point(6_500.0, 5_500.0),
        point(4_000.0, 0.0),
    ];
    let sightings = landmarks.map(|landmark| Sighting {
        landmark,
        bearing: grid_bearing(observer, landmark).expect("apart"),
    });
    let fix = resection(&sightings).expect("a fix");
    assert!(close(fix.point, observer, 1e-6));
    assert!(fix.rms_residual.metres() < 1e-6);
    let two = resection(&sightings[..2]).expect("two lines");
    assert!(close(two.point, observer, 1e-6));
    // One bearing a degree off: the point moves, the misfit shows it.
    let mut off = sightings;
    off[2].bearing = GridBearing::new(off[2].bearing.degrees() + 1.0).expect("bearing");
    let rough = resection(&off).expect("a fix");
    assert!(rough.greatest_residual.metres() > 10.0);
    assert!(resection(&sightings[..1]).is_err());
    assert!(resection(&[]).is_err());
}

#[test]
fn the_three_point_resection_recovers_the_observer_and_the_compass_error() {
    let landmarks = [
        point(1_000.0, 3_000.0),
        point(4_000.0, 2_500.0),
        point(2_500.0, -500.0),
    ];
    let reversed = [landmarks[0], landmarks[2], landmarks[1]];
    // Inside the triangle, outside beyond each side and beyond a corner.
    for observer in [
        point(2_500.0, 1_800.0),
        point(2_000.0, 1_500.0),
        point(-2_000.0, 1_000.0),
        point(6_000.0, 5_000.0),
        point(2_500.0, -4_000.0),
        point(2_400.0, 6_000.0),
    ] {
        for set in [landmarks, reversed] {
            for zero in [0.0, 3.0, -17.5, 179.0] {
                let fix = three_point_resection(set, readings(observer, set, zero))
                    .expect("away from the danger circle");
                assert!(
                    close(fix.point, observer, 1e-6),
                    "{observer:?}: {:?}",
                    fix.point
                );
                let error = (fix.orientation.degrees() - zero + 540.0).rem_euclid(360.0) - 180.0;
                assert!(
                    error.abs() < 1e-9,
                    "zero {zero}: {}",
                    fix.orientation.degrees()
                );
                assert!(fix.danger > 0.0);
            }
        }
    }
}

#[test]
fn the_danger_circle_is_refused_and_its_neighbourhood_reported() {
    // Landmarks on the circle of radius 1000 about the origin.
    let at = |degrees: f64| {
        let r = degrees.to_radians();
        point(1_000.0 * r.sin(), 1_000.0 * r.cos())
    };
    let landmarks = [at(0.0), at(100.0), at(220.0)];
    let on_circle = at(300.0);
    assert!(matches!(
        three_point_resection(landmarks, readings(on_circle, landmarks, 0.0)),
        Err(LandError::DangerCircle { .. })
    ));
    // Inside and outside the circle by 5 % and by 10⁻⁵, away from the
    // landmarks: still solved, the offset reported.
    for (radius, offset, tolerance) in [
        (950.0, 0.05, 1e-6),
        (1_050.0, 0.05, 1e-6),
        (999.99, 1e-5, 1e-2),
    ] {
        let r = 300f64.to_radians();
        let near = point(radius * r.sin(), radius * r.cos());
        let fix = three_point_resection(landmarks, readings(near, landmarks, 0.0))
            .expect("off the circle");
        assert!((fix.danger - offset).abs() < 1e-6, "{}", fix.danger);
        assert!(
            close(fix.point, near, tolerance),
            "{radius}: {:?}",
            fix.point
        );
    }
    // Every point of the circle, refused.
    for degrees in [30.0, 160.0, 300.0, 350.0] {
        let on = at(degrees);
        assert!(
            matches!(
                three_point_resection(landmarks, readings(on, landmarks, 7.0)),
                Err(LandError::DangerCircle { .. })
            ),
            "{degrees}°"
        );
    }
    // Collinear and coincident landmarks.
    let line = [point(0.0, 0.0), point(1.0, 1.0), point(2.0, 2.0)];
    assert!(three_point_resection(line, [Angle::ZERO; 3]).is_err());
    let same = [point(5.0, 5.0); 3];
    assert!(three_point_resection(same, [Angle::ZERO; 3]).is_err());
}

#[test]
fn a_resection_from_field_bearings_lands_within_centimetres_and_the_marine_fix_agrees() {
    // Landmarks 2–4 km from an observer near Tver, 175 km from the central
    // meridian of UTM 36: bearings are the ellipsoidal azimuths at the
    // observer, as a compass takes them, made grid with the convergence
    // there. The plane is off by the arc-to-chord angle: 2.6 cm here; the marine fix
    // by its rhumb lines: 4.7 m.
    let observer = Position::from_degrees(56.8587, 35.9176).expect("on Earth");
    let grid = MapGrid::utm_at(observer).expect("UTM");
    let here = grid.project(observer).expect("projected");
    let marks = [(0.03, -0.02), (-0.01, 0.05), (-0.025, -0.03)].map(|(dlat, dlon)| {
        Position::from_degrees(56.8587 + dlat, 35.9176 + dlon).expect("on Earth")
    });
    let mut sightings = Vec::new();
    let mut lines = Vec::new();
    for mark in marks {
        let azimuth = geodesic(observer, mark).expect("apart").initial_course;
        sightings.push(Sighting {
            landmark: grid.project(mark).expect("projected").point,
            bearing: azimuth.to_grid(here.convergence),
        });
        lines.push(PositionLine::from_bearing_of(mark, azimuth));
    }
    let plane = resection(&sightings).expect("a fix");
    assert!(
        close(plane.point, here.point, 0.05),
        "plane fix {:?} against {:?}",
        metres(plane.point),
        metres(here.point)
    );
    // The marine fix draws rhumb lines; at these ranges within metres.
    let marine = grid
        .project(bearing_fix(&lines).expect("a fix").position)
        .expect("projected");
    assert!(
        close(marine.point, here.point, 5.0),
        "{:?}",
        metres(marine.point)
    );
}

proptest! {
    #[test]
    fn any_three_point_figure_away_from_its_circle_comes_back(
        coords in proptest::array::uniform8(-5_000.0f64..5_000.0),
        zero in -180.0f64..180.0,
    ) {
        let [ae, an, be, bn, ce, cn, pe, pn] = coords;
        let landmarks = [point(ae, an), point(be, bn), point(ce, cn)];
        let observer = point(pe, pn);
        let area = ((be - ae) * (cn - an) - (ce - ae) * (bn - an)).abs();
        let sides = [(be - ae).hypot(bn - an), (ce - be).hypot(cn - bn), (ae - ce).hypot(an - cn)];
        let nearest = landmarks.iter().map(|l| {
            let (e, n) = metres(*l);
            (e - pe).hypot(n - pn)
        }).fold(f64::INFINITY, f64::min);
        // A proper triangle, the observer clear of every landmark.
        prop_assume!(area > 0.05 * sides.iter().fold(0.0f64, |m, s| m.max(*s)).powi(2));
        prop_assume!(nearest > 100.0);
        match three_point_resection(landmarks, readings(observer, landmarks, zero)) {
            Ok(fix) if fix.danger > 0.05 => {
                prop_assert!(close(fix.point, observer, 1e-4), "{:?}", fix.point);
            }
            // Near the circle the figure is weak; it may refuse or drift.
            _ => {}
        }
    }
}

#[test]
fn grid_distances_and_bearings_are_plane_geometry() {
    let (a, b) = (point(0.0, 0.0), point(3.0, 4.0));
    assert_eq!(
        kinavis_land::grid_distance(a, b),
        Distance::from_metres(5.0).expect("5 m")
    );
    assert!((grid_bearing(a, b).expect("apart").degrees() - 36.869_897_645_844_02).abs() < 1e-12);
    assert!(grid_bearing(a, a).is_err());
}
