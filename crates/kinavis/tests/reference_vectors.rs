//! Agreement with external reference values and geometric identities.
//!
//! Round-trip properties prove functions are mutual inverses, not that either
//! is correct. These compare with external sources: Vincenty's published test
//! line, the WGS-84 defining constants, and cases where spherical geometry
//! fixes the answer.

// Tests may panic on a failed expectation; the library may not.
#![allow(clippy::expect_used, clippy::unwrap_used)]

use core::f64::consts::PI;
use kinavis::sailings::{geodesic, great_circle, rhumb_destination, rhumb_line, EARTH_RADIUS};
use kinavis::{Distance, Latitude, Longitude, Position, TrueCourse};

/// WGS-84 semi-major axis, m (defining constant).
const WGS84_A: f64 = 6_378_137.0;

/// Degrees, minutes, seconds to signed decimal degrees.
fn dms(sign: f64, degrees: f64, minutes: f64, seconds: f64) -> f64 {
    sign * (degrees + minutes / 60.0 + seconds / 3600.0)
}

fn at(latitude: f64, longitude: f64) -> Position {
    Position::new(
        Latitude::from_degrees(latitude).expect("in range"),
        Longitude::from_degrees(longitude).expect("in range"),
    )
}

/// Vincenty's worked example (1975 paper): Flinders Peak to Buninyong,
/// Victoria.
#[test]
fn vincentys_published_line() {
    let flinders = at(
        dms(-1.0, 37.0, 57.0, 3.720_30),
        dms(1.0, 144.0, 25.0, 29.524_40),
    );
    let buninyong = at(
        dms(-1.0, 37.0, 39.0, 10.156_10),
        dms(1.0, 143.0, 55.0, 35.383_90),
    );

    let sailing = geodesic(flinders, buninyong).expect("a short, ordinary line");

    // Published: 54972.271 m, to the quoted millimetre.
    assert!(
        (sailing.distance.metres() - 54_972.271).abs() < 5e-4,
        "{} m",
        sailing.distance.metres()
    );

    // Published: α₁ = 306°52'05.37", quoted to 0.01″ (~3e-6°).
    let alpha1 = dms(1.0, 306.0, 52.0, 5.37);
    assert!(
        (sailing.initial_course.degrees() - alpha1).abs() < 3e-6,
        "{} vs {alpha1}",
        sailing.initial_course.degrees()
    );

    // Published: α₂ = 127°10'25.07", the reverse azimuth at the far end. This
    // crate reports the course on arrival, its reciprocal; both conventions
    // exist, so the difference is stated here.
    let alpha2 = dms(1.0, 127.0, 10.0, 25.07);
    assert!(
        (sailing.final_course.reciprocal().degrees() - alpha2).abs() < 3e-6,
        "{} vs {alpha2}",
        sailing.final_course.reciprocal().degrees()
    );

    // The reciprocal equals the reverse line's initial course.
    let back = geodesic(buninyong, flinders).expect("the same line, reversed");
    assert!(
        back.initial_course
            .angular_distance(sailing.final_course.reciprocal())
            < 1e-9
    );
}

/// Ellipsoid defining values.
#[test]
fn wgs84_arcs_match_the_published_figures() {
    // 1° of longitude on the equator is exactly a·π/180.
    let equator = geodesic(at(0.0, 0.0), at(0.0, 1.0)).expect("along the equator");
    let exact = WGS84_A * PI / 180.0;
    assert!(
        (equator.distance.metres() - exact).abs() < 1e-6,
        "{} vs {exact}",
        equator.distance.metres()
    );

    // Quarter meridian, equator to pole: 10001965.729 m on WGS-84.
    let quarter = geodesic(at(0.0, 0.0), at(90.0, 0.0)).expect("up the meridian");
    assert!(
        (quarter.distance.metres() - 10_001_965.729).abs() < 1e-3,
        "{} m",
        quarter.distance.metres()
    );

    // 1° of latitude is shortest at the equator and longest at the pole
    // (flattening): 110574.4 m and 111694.0 m.
    let first = geodesic(at(0.0, 0.0), at(1.0, 0.0)).expect("up the meridian");
    assert!(
        (first.distance.metres() - 110_574.389).abs() < 1e-2,
        "{} m",
        first.distance.metres()
    );
    let last = geodesic(at(89.0, 0.0), at(90.0, 0.0)).expect("up the meridian");
    assert!(
        (last.distance.metres() - 111_693.865).abs() < 1e-2,
        "{} m",
        last.distance.metres()
    );
}

/// Cases fixed by spherical geometry.
#[test]
fn the_sphere_gives_exact_answers_in_places() {
    let quarter = EARTH_RADIUS.metres() * PI / 2.0;

    // Quarter of the equator.
    let along = great_circle(at(0.0, 0.0), at(0.0, 90.0)).expect("along the equator");
    assert!((along.distance.metres() - quarter).abs() < 1e-6);

    // Equator to pole.
    let up = great_circle(at(0.0, 0.0), at(90.0, 0.0)).expect("up the meridian");
    assert!((up.distance.metres() - quarter).abs() < 1e-6);

    // Off-axis case: by the spherical law of cosines, cos c = sin0·sin45 +
    // cos0·cos45·cos90 = 0, so the points are exactly 90° apart regardless of
    // the second latitude.
    let across = great_circle(at(0.0, 0.0), at(45.0, 90.0)).expect("a general line");
    assert!(
        (across.distance.metres() - quarter).abs() < 1e-6,
        "{} vs {quarter}",
        across.distance.metres()
    );
}

/// Exact rhumb-line cases in the model's own terms.
///
/// The spherical sailings use a mean radius of 6371.0088 km, not the radius on
/// which 1′ is exactly 1 NM; 1° of latitude is 60.04 NM here, so the identities
/// are written against the radius.
#[test]
fn rhumb_lines_agree_with_plane_trigonometry() {
    /// 10° of arc in NM on the sailings' sphere.
    fn ten_degrees() -> f64 {
        EARTH_RADIUS.metres() * (10.0_f64).to_radians() / 1852.0
    }

    // Due east on the equator: the parallel is a great circle, so the rhumb
    // distance is the arc.
    let east = rhumb_line(at(0.0, 0.0), at(0.0, 10.0)).expect("along the equator");
    assert!((east.initial_course.degrees() - 90.0).abs() < 1e-9);
    assert!(
        (east.distance.nautical_miles() - ten_degrees()).abs() < 1e-9,
        "{} vs {}",
        east.distance.nautical_miles(),
        ten_degrees()
    );

    // Due north: the meridian arc, course exactly 000°.
    let north = rhumb_line(at(0.0, 0.0), at(10.0, 0.0)).expect("up the meridian");
    assert!(north.initial_course.degrees().abs() < 1e-9);
    assert!((north.distance.nautical_miles() - ten_degrees()).abs() < 1e-9);

    // On 045° northing is distance·cos45 exactly: on Mercator the rhumb line is
    // straight and departure is the other leg of a right triangle.
    let run = 100.0;
    let arrival = rhumb_destination(
        at(0.0, 0.0),
        TrueCourse::new(45.0).expect("in range"),
        Distance::from_nautical_miles(run).expect("finite"),
    )
    .expect("a short leg from the equator");

    let expected_radians = run * 1852.0 * (45.0_f64).to_radians().cos() / EARTH_RADIUS.metres();
    let northing = arrival.latitude().degrees().to_radians();
    assert!(
        (northing - expected_radians).abs() < 1e-12,
        "{northing} vs {expected_radians}"
    );
}
