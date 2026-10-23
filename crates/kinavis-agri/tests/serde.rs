//! Serialisation round-trips exactly and cannot bypass invariants: a job kept
//! between sessions reads back equal to itself, and stored values the
//! constructors refuse are refused on reading.

#![cfg(all(feature = "serde", feature = "std"))]
#![allow(clippy::expect_used, clippy::panic, clippy::float_cmp)]

use kinavis::region::{PlanarPoint, Polygon, Region};
use kinavis_agri::{
    AbLine, Boom, CurveLine, Field, FieldPlane, LeverArm, PassIndex, PassPattern, PivotLine,
    TrackerConfig, VehicleGeometry, MAX_PASS,
};
use kinavis_grid::ProjectedCrs;
use kinavis_kernel::geodesy::{Datum, Ellipsoid, Helmert};
use kinavis_kernel::{Angle, Distance, GridCourse, Height, Position, RateOfTurn};
use proptest::prelude::*;

fn metres(value: f64) -> Distance {
    Distance::from_metres(value).expect("finite")
}

fn point(east: f64, north: f64) -> PlanarPoint {
    PlanarPoint::from_metres(east, north).expect("finite")
}

fn centred(latitude: f64, longitude: f64, height: f64) -> FieldPlane {
    FieldPlane::centred_at(
        Position::from_degrees(latitude, longitude).expect("valid"),
        Height::above_ellipsoid(metres(height)),
    )
    .expect("a field plane")
}

fn round_trip<T>(value: &T) -> T
where
    T: serde::Serialize + serde::de::DeserializeOwned,
{
    let text = serde_json::to_string(value).expect("serialises");
    serde_json::from_str(&text).expect("deserialises")
}

fn refused<T: serde::de::DeserializeOwned>(text: &str) -> bool {
    serde_json::from_str::<T>(text).is_err()
}

#[test]
fn a_job_reads_back_equal_to_itself() {
    let plane = centred(51.985, 5.66, 10.0);
    let boundary = Polygon::<8>::new(&[
        point(0.0, 0.0),
        point(400.0, 0.0),
        point(400.0, 300.0),
        point(0.0, 300.0),
    ])
    .expect("a boundary");
    let pond = Polygon::<8>::new(&[
        point(100.0, 100.0),
        point(150.0, 100.0),
        point(150.0, 140.0),
    ])
    .expect("a pond");
    let field = Field::new(
        plane,
        Region::<8, 8, 2>::new(boundary, &[pond]).expect("a region"),
    );
    assert_eq!(round_trip(&field), field);

    let ab = AbLine::from_point_and_course(
        plane,
        point(10.0, 20.0),
        GridCourse::new(33.3).expect("valid"),
    );
    assert_eq!(round_trip(&ab), ab);
    let ab = AbLine::from_points(plane, point(1.0, 2.0), point(311.7, -43.9)).expect("a line");
    assert_eq!(round_trip(&ab), ab);

    let pivot = PivotLine::new(plane, point(200.0, 150.0), metres(123.4)).expect("a circle");
    assert_eq!(round_trip(&pivot), pivot);

    let curve = CurveLine::<16>::new(
        plane,
        &[
            point(0.0, 0.0),
            point(50.0, 10.0),
            point(90.0, 45.0),
            point(110.0, 120.0),
        ],
    )
    .expect("a curve");
    assert_eq!(round_trip(&curve), curve);

    let pattern = PassPattern::new(metres(12.0), metres(0.3))
        .and_then(|pattern| pattern.with_shift(metres(0.1)))
        .and_then(|pattern| pattern.nudge(metres(-0.07)))
        .expect("a pattern");
    assert_eq!(round_trip(&pattern), pattern);

    let boom = Boom::new([metres(3.0), metres(6.1), metres(3.0)], metres(2.5))
        .and_then(|boom| boom.offset_right(metres(-0.2)))
        .expect("a boom");
    assert_eq!(round_trip(&boom), boom);

    let geometry = VehicleGeometry::new(
        metres(2.9),
        Angle::from_degrees(37.0).expect("valid"),
        RateOfTurn::from_radians_per_second(0.44).expect("valid"),
    )
    .expect("a geometry")
    .with_antenna(LeverArm {
        forward: metres(1.2),
        right: metres(-0.05),
        up: metres(3.1),
    });
    assert_eq!(round_trip(&geometry), geometry);

    let config = TrackerConfig::new(metres(0.3), metres(15.0), metres(0.5)).expect("a config");
    assert_eq!(round_trip(&config), config);

    assert_eq!(round_trip(&PassIndex::new(-7)), PassIndex::new(-7));
}

#[test]
fn a_centred_plane_is_stored_as_its_origin_and_scale() {
    let text = serde_json::to_string(&centred(52.0, 5.5, 0.0)).expect("serialises");
    assert_eq!(
        text,
        r#"{"centred":{"origin":{"latitude":52.0,"longitude":5.5},"scale":1.0}}"#
    );
}

#[test]
fn a_grid_plane_is_stored_by_its_datum_name() {
    let plane = FieldPlane::grid(ProjectedCrs::dhdn(3).expect("a zone"));
    let text = serde_json::to_string(&plane).expect("serialises");
    assert!(text.starts_with(r#"{"grid":{"datum":"DHDN","#), "{text}");
    assert_eq!(round_trip(&plane), plane);
    let plane = FieldPlane::grid(ProjectedCrs::british_national_grid().expect("a grid"));
    assert_eq!(round_trip(&plane), plane);
}

#[test]
fn a_grid_on_a_datum_of_its_own_does_not_serialise() {
    let datum = Datum::new(
        "Local survey",
        Ellipsoid::GRS80,
        Helmert::translation(1.0, 2.0, 3.0),
        metres(0.5),
    );
    let crs = ProjectedCrs::gauss_kruger_6(datum, 32).expect("a zone");
    assert!(serde_json::to_string(&FieldPlane::grid(crs)).is_err());
}

#[test]
fn stored_values_the_constructors_refuse_are_refused() {
    let plane = r#"{"centred":{"origin":{"latitude":52.0,"longitude":5.5},"scale":1.0}}"#;
    // Scales standing for heights of 63 km and −64 km.
    assert!(refused::<FieldPlane>(
        r#"{"centred":{"origin":{"latitude":52.0,"longitude":5.5},"scale":1.01}}"#
    ));
    assert!(refused::<FieldPlane>(
        r#"{"centred":{"origin":{"latitude":52.0,"longitude":5.5},"scale":0.99}}"#
    ));
    assert!(refused::<FieldPlane>(
        r#"{"centred":{"origin":{"latitude":89.5,"longitude":5.5},"scale":1.0}}"#
    ));
    assert!(refused::<FieldPlane>(
        r#"{"grid":{"datum":"Atlantis","central_meridian":9.0,"latitude_of_origin":0.0,"scale":1.0,"false_easting_metres":500000.0,"false_northing_metres":0.0}}"#
    ));
    assert!(refused::<FieldPlane>(
        r#"{"grid":{"datum":"WGS 84","central_meridian":9.0,"latitude_of_origin":0.0,"scale":0.0,"false_easting_metres":500000.0,"false_northing_metres":0.0}}"#
    ));

    // Not a unit direction.
    assert!(refused::<AbLine>(&format!(
        r#"{{"plane":{plane},"origin":[0.0,0.0],"direction":[1.0,1.0]}}"#
    )));
    assert!(refused::<PivotLine>(&format!(
        r#"{{"plane":{plane},"centre":[0.0,0.0],"radius_metres":0.5}}"#
    )));
    // Crossing itself; one vertex; one more than the capacity.
    assert!(refused::<CurveLine<8>>(&format!(
        r#"{{"plane":{plane},"vertices":[[0,0],[10,0],[10,10],[5,-5]]}}"#
    )));
    assert!(refused::<CurveLine<8>>(&format!(
        r#"{{"plane":{plane},"vertices":[[0,0]]}}"#
    )));
    assert!(refused::<CurveLine<2>>(&format!(
        r#"{{"plane":{plane},"vertices":[[0,0],[10,0],[20,5]]}}"#
    )));

    assert!(refused::<PassPattern>(
        r#"{"spacing_metres":0.0,"shift_metres":0.0}"#
    ));
    assert!(refused::<PassPattern>(
        r#"{"spacing_metres":12.0,"shift_metres":2e6}"#
    ));
    assert!(refused::<Boom<3>>(
        r#"{"widths_metres":[3.0,3.0],"behind_metres":1.0,"right_metres":0.0}"#
    ));
    assert!(refused::<Boom<1>>(
        r#"{"widths_metres":[3.0,3.0],"behind_metres":1.0,"right_metres":0.0}"#
    ));
    assert!(refused::<Boom<1>>(
        r#"{"widths_metres":[0.001],"behind_metres":1.0,"right_metres":0.0}"#
    ));
    assert!(refused::<Boom<1>>(
        r#"{"widths_metres":[3.0],"behind_metres":1.0,"right_metres":101.0}"#
    ));
    assert!(refused::<VehicleGeometry>(
        r#"{"wheelbase_metres":2.9,"max_wheel_angle_radians":1.6,"max_steering_rate_radians_per_second":0.4,"antenna":{"forward_metres":0.0,"right_metres":0.0,"up_metres":3.0}}"#
    ));
    assert!(refused::<VehicleGeometry>(
        r#"{"wheelbase_metres":2.9,"max_wheel_angle_radians":0.6,"max_steering_rate_radians_per_second":0.0,"antenna":{"forward_metres":0.0,"right_metres":0.0,"up_metres":3.0}}"#
    ));
    assert!(refused::<TrackerConfig>(
        r#"{"off_line_limit_metres":0.0,"end_warning_metres":15.0,"hysteresis_metres":0.5}"#
    ));
}

#[test]
fn a_pass_number_beyond_the_limit_reads_as_the_limit() {
    let far: PassIndex = serde_json::from_str("2000000000").expect("a number");
    assert_eq!(far, PassIndex::new(MAX_PASS));
}

#[test]
fn a_plane_at_the_height_limits_reads_back() {
    for height in [-1_000.0, 9_000.0] {
        let plane = centred(-33.9, 151.2, height);
        assert_eq!(round_trip(&plane), plane);
    }
}

proptest! {
    #[test]
    fn every_centred_plane_reads_back_equal_to_itself(
        latitude in -89.0..=89.0_f64,
        longitude in -180.0..180.0_f64,
        height in -1_000.0..=9_000.0_f64,
    ) {
        let plane = centred(latitude, longitude, height);
        prop_assert_eq!(round_trip(&plane), plane);
    }
}
