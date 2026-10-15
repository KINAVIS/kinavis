//! The field plane against PROJ, and its distances against the ground:
//! straight lines between points at the field's height (ECEF) and the
//! geodesic on the ellipsoid.

#![allow(clippy::expect_used, clippy::panic, clippy::float_cmp)]

use kinavis::region::PlanarPoint;
use kinavis::sailings::geodesic;
use kinavis_agri::{AgriError, FieldPlane, FIELD_RADIUS_METRES};
use kinavis_kernel::geodesy::{EcefPoint, Ellipsoid, GeodeticPoint};
use kinavis_kernel::{Distance, GridCourse, Height, Position};

const PLANE: &str = include_str!("data/plane_reference.txt");

/// Mean Earth radius, metres, for the stated distortion `x²/2R²`.
const R: f64 = 6_371_000.0;

fn rows(text: &str) -> impl Iterator<Item = Vec<f64>> + '_ {
    text.lines()
        .filter(|line| !line.starts_with('#'))
        .map(|line| {
            line.split_ascii_whitespace()
                .map(|field| field.parse().expect("a number"))
                .collect()
        })
}

fn metres(value: f64) -> Distance {
    Distance::from_metres(value).expect("finite")
}

fn plane(latitude: f64, longitude: f64, height: f64) -> FieldPlane {
    FieldPlane::centred_at(
        Position::from_degrees(latitude, longitude).expect("on Earth"),
        Height::above_ellipsoid(metres(height)),
    )
    .expect("a field plane")
}

#[test]
fn the_centred_plane_agrees_with_proj_to_a_micrometre() {
    let mut checked = 0;
    for row in rows(PLANE) {
        let [lat0, lon0, h, lat, lon, easting, northing, convergence, scale] = row[..] else {
            panic!("nine columns: {row:?}");
        };
        let plane = plane(lat0, lon0, h);
        let point = plane
            .to_plane(Position::from_degrees(lat, lon).expect("on Earth"))
            .expect("within the radius");
        assert!(
            (point.east_metres() - easting).abs() < 1e-6
                && (point.north_metres() - northing).abs() < 1e-6,
            "{point} against E {easting} N {northing} at {lat} {lon}"
        );
        let gamma = plane.convergence_at(point).expect("within the radius");
        assert!(
            (gamma.degrees() - convergence).abs() < 1e-9,
            "convergence at {lat} {lon}"
        );
        let k = plane.scale_at(point).expect("within the radius");
        assert!((k - scale).abs() < 1e-9, "scale at {lat} {lon}");
        let back = plane.to_position(point).expect("within the radius");
        assert!((back.latitude().degrees() - lat).abs() < 1e-10);
        assert!((back.longitude().degrees() - lon).abs() < 1e-10);
        checked += 1;
    }
    assert_eq!(checked, 660);
}

/// Points of a field around an origin, on rings to the radius.
fn ring_points(plane: &FieldPlane) -> Vec<PlanarPoint> {
    let mut points = vec![PlanarPoint::ORIGIN];
    for radius in [500.0, 2_000.0, 5_000.0, 10_000.0, 19_990.0] {
        for step in 0..12 {
            let angle = f64::from(step) * 30f64.to_radians() + 0.1;
            points.push(
                PlanarPoint::from_metres(radius * angle.sin(), radius * angle.cos())
                    .expect("finite"),
            );
        }
    }
    assert!(points.iter().all(|p| plane.to_position(*p).is_ok()));
    points
}

/// The stated relative error of a plane distance between two points at
/// these distances from the origin, for a field at height `h` and latitude
/// `latitude`.
fn stated(a: PlanarPoint, b: PlanarPoint, h: f64, latitude: f64) -> f64 {
    let x = |p: PlanarPoint| p.east_metres().hypot(p.north_metres());
    let e2 = 0.006_694_379_990_14;
    let height = h * e2 * latitude.to_radians().cos().powi(2) / (2.0 * 6_378_137.0);
    x(a).max(x(b)).powi(2) / (2.0 * R * R) * 1.02 + height * 1.05 + 2e-8
}

#[test]
fn plane_distances_are_ground_distances_at_the_fields_height() {
    let mut worst: f64 = 0.0;
    for (lat0, lon0) in [(0.5, -47.9), (45.0, 30.0), (60.0, 10.0), (-35.0, 150.0)] {
        for h in [0.0, 500.0, 3_000.0] {
            let plane = plane(lat0, lon0, h);
            let points = ring_points(&plane);
            let ground = |p: PlanarPoint| {
                let position = plane.to_position(p).expect("within the radius");
                EcefPoint::from_geodetic(
                    GeodeticPoint::new(position, Height::above_ellipsoid(metres(h))),
                    &Ellipsoid::WGS84,
                )
                .expect("on Earth")
            };
            for (i, a) in points.iter().enumerate() {
                for b in points.iter().skip(i + 1).step_by(7) {
                    let (ga, gb) = (ground(*a), ground(*b));
                    let chord = ((ga.x().metres() - gb.x().metres()).powi(2)
                        + (ga.y().metres() - gb.y().metres()).powi(2)
                        + (ga.z().metres() - gb.z().metres()).powi(2))
                    .sqrt();
                    // Ground distance along the surface at height h, not the chord.
                    let arc = chord * (1.0 + chord * chord / (24.0 * R * R));
                    let planar = a.distance_to(*b).metres();
                    let relative = (planar - arc).abs() / arc;
                    assert!(
                        relative <= stated(*a, *b, h, lat0),
                        "{relative:e} between {a} and {b} at {lat0} {lon0} h {h}"
                    );
                    if h == 0.0 {
                        worst = worst.max(relative);
                    }
                }
            }
        }
    }
    // The radius is where the error of the plane itself reaches its bound.
    assert!(worst < 5e-6 && worst > 3e-6, "worst {worst:e}");
}

#[test]
fn at_sea_level_plane_distances_are_geodesics() {
    let plane = plane(52.26, 21.01, 0.0);
    let points = ring_points(&plane);
    for (i, a) in points.iter().enumerate() {
        for b in points.iter().skip(i + 1).step_by(5) {
            let line = geodesic(
                plane.to_position(*a).expect("inside"),
                plane.to_position(*b).expect("inside"),
            )
            .expect("a geodesic");
            let relative = (a.distance_to(*b).metres() - line.distance.metres()).abs()
                / line.distance.metres();
            assert!(
                relative <= stated(*a, *b, 0.0, 52.26),
                "{relative:e} between {a} and {b}"
            );
        }
    }
}

#[test]
fn courses_turn_through_the_convergence_both_ways() {
    let plane = plane(59.9, 10.75, 90.0);
    let east = PlanarPoint::from_metres(15_000.0, 3_000.0).expect("finite");
    let gamma = plane.convergence_at(east).expect("inside").degrees();
    // East of the central meridian in the north grid north lies east of true
    // north by about Δλ·sin φ.
    let position = plane.to_position(east).expect("inside");
    let expected = (position.longitude().degrees() - 10.75) * position.latitude().radians().sin();
    assert!(
        (gamma - expected).abs() < 1e-3 * expected.abs(),
        "{gamma} against {expected}"
    );
    let grid = GridCourse::new(37.0).expect("a course");
    let back = plane
        .grid_course(plane.true_course(grid, east).expect("inside"), east)
        .expect("inside");
    assert!((back.degrees() - 37.0).abs() < 1e-12);
}

#[test]
fn the_radius_is_enforced_only_for_a_centred_plane() {
    let plane = plane(45.0, 30.0, 120.0);
    let beyond = PlanarPoint::from_metres(FIELD_RADIUS_METRES + 1.0, 0.0).expect("finite");
    assert!(matches!(
        plane.to_position(beyond),
        Err(AgriError::BeyondFieldRadius { .. })
    ));
    let grid = FieldPlane::grid(*plane.crs());
    assert!(!grid.is_centred());
    assert!(grid.to_position(beyond).is_ok());
}
