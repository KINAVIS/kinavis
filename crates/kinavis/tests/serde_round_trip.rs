//! Serialisation round-trips and cannot bypass invariants.
//!
//! The second property matters more: a constructor rejecting latitude 500° is
//! useless if `serde` accepts it.

#![cfg(all(feature = "serde", feature = "std"))]
#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::needless_pass_by_value
)]

use kinavis::anchor::AnchorWatch;
use kinavis::clearance::{ClearancePolicy, Hull, Waterway};
use kinavis::composite::composite_sailing;
use kinavis::estimator::{EstimatorConfig, LatePolicy};
use kinavis::guidance::GuidanceConfig;
use kinavis::mob::ManOverboard;
use kinavis::relative_motion::{Contact, Vessel};
use kinavis::route::{LegCursor, LegKind, Route};
use kinavis::sailings::Sailing;
use kinavis::schedule::RouteSchedule;
use kinavis::turning::{TurnMode, TurnParameters};
use kinavis::{
    Angle, CompassCourse, Deviation, DeviationTable, Distance, Instant, Latitude, Longitude,
    MagneticCourse, Position, RelativeBearing, Speed, TrueBearing, TrueCourse, Variation,
};

fn round_trip<T>(value: T) -> T
where
    T: serde::Serialize + serde::de::DeserializeOwned,
{
    let text = serde_json::to_string(&value).expect("serialises");
    serde_json::from_str(&text).expect("deserialises")
}

#[test]
fn scalars_round_trip() {
    assert_eq!(
        round_trip(Latitude::from_degrees(50.755).unwrap()),
        Latitude::from_degrees(50.755).unwrap()
    );
    assert_eq!(
        round_trip(Longitude::from_degrees(-1.2967).unwrap()),
        Longitude::from_degrees(-1.2967).unwrap()
    );
    assert_eq!(
        round_trip(Distance::from_nautical_miles(12.5).unwrap()).nautical_miles(),
        12.5
    );
    assert_eq!(round_trip(Speed::from_knots(8.25).unwrap()).knots(), 8.25);
    assert_eq!(
        round_trip(Angle::from_degrees(-1.5).unwrap()).degrees(),
        -1.5
    );
    assert_eq!(round_trip(Variation::new(-2.7).unwrap()).degrees(), -2.7);
    assert_eq!(round_trip(Deviation::new(1.5).unwrap()).degrees(), 1.5);
    assert_eq!(
        round_trip(RelativeBearing::new(315.0).unwrap()).degrees(),
        315.0
    );
}

#[test]
fn directions_keep_their_frame_through_the_type_not_the_data() {
    let compass = CompassCourse::new(123.4).unwrap();
    assert_eq!(round_trip(compass), compass);

    // The frame is not serialised: a course is a plain number on the wire.
    let text = serde_json::to_string(&compass).unwrap();
    assert_eq!(text, "123.4");

    // The same text deserialises into any frame; the target type decides.
    let as_magnetic: MagneticCourse = serde_json::from_str(&text).unwrap();
    assert_eq!(as_magnetic.degrees(), 123.4);
}

#[test]
fn positions_round_trip() {
    let position = Position::from_degrees(50.755, -1.2967).unwrap();
    assert_eq!(round_trip(position), position);
}

#[test]
fn deviation_tables_round_trip() {
    let table = DeviationTable::from_deviations(&[
        -2.5, -0.5, 1.6, 4.4, -1.7, 0.0, 1.0, 0.3, -0.9, 0.5, -1.2, 0.8, -0.3, 1.7, -2.1, 0.4,
        -0.6, 1.2, -1.3, 0.0, 0.9, -1.1, 1.5, -0.7, -13.2, -15.7, -17.9, -19.2, -18.1, 1.8, -0.4,
        0.7, -0.2, 1.4, -4.4, -2.9,
    ])
    .unwrap();
    assert_eq!(round_trip(table), table);
}

#[test]
fn routes_round_trip() {
    let route = Route::new(
        &[
            Position::from_degrees(50.1, -1.5).unwrap(),
            Position::from_degrees(49.9, -2.0).unwrap(),
            Position::from_degrees(49.7, -2.75).unwrap(),
        ],
        LegKind::GreatCircle,
    )
    .unwrap();
    let back = round_trip(route);
    assert_eq!(back, route);
    assert_eq!(back.kind(), LegKind::GreatCircle);
}

#[test]
fn result_types_round_trip() {
    let sailing = Sailing {
        initial_course: TrueCourse::new(282.8).unwrap(),
        final_course: TrueCourse::new(246.1).unwrap(),
        distance: Distance::from_nautical_miles(1889.1).unwrap(),
    };
    assert_eq!(round_trip(sailing), sailing);

    let vessel = Vessel {
        course: TrueCourse::new(35.0).unwrap(),
        speed: Speed::from_knots(12.0).unwrap(),
    };
    assert_eq!(round_trip(vessel), vessel);

    let contact = Contact {
        bearing: TrueBearing::new(80.0).unwrap(),
        range: Distance::from_nautical_miles(9.0).unwrap(),
    };
    assert_eq!(round_trip(contact), contact);
}

// ---------------------------------------------------------------------------
// Deserialisation is not a back door
// ---------------------------------------------------------------------------

#[test]
fn an_impossible_latitude_is_refused_on_the_way_in() {
    assert!(serde_json::from_str::<Latitude>("500.0").is_err());
    assert!(serde_json::from_str::<Latitude>("-91.0").is_err());
    assert!(serde_json::from_str::<Latitude>("null").is_err());
    // Longitude wraps, as at construction.
    assert_eq!(
        serde_json::from_str::<Longitude>("190.0")
            .unwrap()
            .degrees(),
        -170.0
    );
}

#[test]
fn an_impossible_direction_is_refused_on_the_way_in() {
    assert!(serde_json::from_str::<TrueCourse>("400.0").is_err());
    assert!(serde_json::from_str::<TrueCourse>("-1.0").is_err());
    assert!(serde_json::from_str::<CompassCourse>("\"north\"").is_err());
    assert_eq!(
        serde_json::from_str::<TrueCourse>("360.0")
            .unwrap()
            .degrees(),
        0.0
    );
}

#[test]
fn an_impossible_correction_is_refused_on_the_way_in() {
    assert!(serde_json::from_str::<Variation>("181.0").is_err());
    assert!(serde_json::from_str::<Deviation>("-181.0").is_err());
    assert!(serde_json::from_str::<RelativeBearing>("361.0").is_err());
}

#[test]
fn a_degenerate_deviation_table_is_refused_on_the_way_in() {
    // Too few nodes.
    assert!(serde_json::from_str::<DeviationTable>("[]").is_err());
    assert!(serde_json::from_str::<DeviationTable>("[[0, 1.0]]").is_err());
    // Duplicate heading.
    assert!(serde_json::from_str::<DeviationTable>("[[0, 1.0], [360, 2.0]]").is_err());
    // Impossible deviation.
    assert!(serde_json::from_str::<DeviationTable>("[[0, 1.0], [180, 900.0]]").is_err());

    // A valid table is accepted and its courses normalised.
    let table: DeviationTable =
        serde_json::from_str("[[-350, 1.0], [180, -2.0]]").expect("a valid table");
    assert_eq!(table.nodes().first().unwrap().course(), 10);
}

#[test]
fn schedules_round_trip() {
    let route = Route::new(
        &[
            Position::from_degrees(50.1, -1.5).unwrap(),
            Position::from_degrees(49.9, -2.0).unwrap(),
            Position::from_degrees(49.7, -2.75).unwrap(),
        ],
        LegKind::RhumbLine,
    )
    .unwrap();
    let schedule = RouteSchedule::with_leg_speeds(
        &route,
        Instant::from_unix_seconds(1_789_000_000),
        &[
            Speed::from_knots(12.0).unwrap(),
            Speed::from_knots(9.5).unwrap(),
        ],
    )
    .unwrap();
    let back = round_trip(schedule);
    assert_eq!(back, schedule);
    assert_eq!(back.arrival(), schedule.arrival());
}

#[test]
fn a_schedule_that_cannot_be_kept_is_refused_on_the_way_in() {
    // No legs.
    assert!(serde_json::from_str::<RouteSchedule>(
        r#"{"departure":{"seconds":1789000000,"nanos":0},"legs":[]}"#
    )
    .is_err());
    // Zero-speed leg, negative-length leg.
    assert!(serde_json::from_str::<RouteSchedule>(
        r#"{"departure":{"seconds":1789000000,"nanos":0},
            "legs":[{"distance":22.7,"speed":0.0}]}"#
    )
    .is_err());
    assert!(serde_json::from_str::<RouteSchedule>(
        r#"{"departure":{"seconds":1789000000,"nanos":0},
            "legs":[{"distance":-22.7,"speed":12.0}]}"#
    )
    .is_err());
    // The same values in the right order form a valid schedule.
    assert!(serde_json::from_str::<RouteSchedule>(
        r#"{"departure":{"seconds":1789000000,"nanos":0},
            "legs":[{"distance":22.7,"speed":12.0}]}"#
    )
    .is_ok());
}

#[test]
fn hulls_and_waterways_round_trip_and_a_hull_is_checked_on_the_way_in() {
    let hull = Hull::new(Distance::from_metres(12.0).unwrap(), 0.85).unwrap();
    assert_eq!(round_trip(hull), hull);
    let policy = ClearancePolicy {
        minimum: Distance::from_metres(1.0).unwrap(),
        fraction_of_draught: 0.1,
    };
    assert_eq!(round_trip(policy), policy);
    assert_eq!(
        round_trip(Waterway::Blockage(0.15)),
        Waterway::Blockage(0.15)
    );

    // Block coefficient above 1 or zero draught: invalid hull.
    assert!(serde_json::from_str::<Hull>(r#"{"draught":12.0,"block_coefficient":1.5}"#).is_err());
    assert!(serde_json::from_str::<Hull>(r#"{"draught":0.0,"block_coefficient":0.8}"#).is_err());
}

#[test]
fn the_phase_six_projections_round_trip() {
    let here = Position::from_degrees(50.0, -1.0).unwrap();
    let watch = AnchorWatch::new(here, Distance::from_metres(120.0).unwrap())
        .unwrap()
        .with_allowance(Distance::from_metres(15.0).unwrap())
        .unwrap();
    assert_eq!(round_trip(watch), watch);
    // Negative radius.
    assert!(serde_json::from_str::<AnchorWatch>(
        r#"{"anchor":{"latitude":50.0,"longitude":-1.0},"swinging_radius":-0.1,"allowance":0.0}"#
    )
    .is_err());

    let mob = ManOverboard::new(here, Instant::from_unix_seconds(1_789_000_000));
    assert_eq!(round_trip(mob), mob);

    let track = composite_sailing(
        Position::from_degrees(34.87, 139.7).unwrap(),
        Position::from_degrees(37.8, -122.5).unwrap(),
        Latitude::from_degrees(45.0).unwrap(),
    )
    .unwrap();
    // Courses round-trip to the last printed decimal, not the last bit.
    let back = round_trip(track);
    assert_eq!(
        back.parallel_run().unwrap().first_vertex,
        track.parallel_run().unwrap().first_vertex
    );
    assert!(
        (back.total_distance().nautical_miles() - track.total_distance().nautical_miles()).abs()
            < 1e-9
    );
}

#[test]
fn a_route_that_goes_nowhere_is_refused_on_the_way_in() {
    assert!(serde_json::from_str::<Route>(r#"{"waypoints":[],"kind":"RhumbLine"}"#).is_err());
    assert!(serde_json::from_str::<Route>(
        r#"{"waypoints":[{"latitude":50.0,"longitude":-1.0}],"kind":"RhumbLine"}"#
    )
    .is_err());
    // One invalid waypoint rejects the whole route.
    assert!(serde_json::from_str::<Route>(
        r#"{"waypoints":[{"latitude":500.0,"longitude":-1.0},
                         {"latitude":50.0,"longitude":-1.0}],"kind":"RhumbLine"}"#
    )
    .is_err());
}

#[test]
fn non_finite_numbers_do_not_survive_the_trip() {
    // JSON has no NaN; these arrive as text and must be rejected.
    for text in ["\"NaN\"", "\"inf\"", "1e400"] {
        assert!(
            serde_json::from_str::<Distance>(text).is_err(),
            "{text} should not deserialise"
        );
        assert!(serde_json::from_str::<Speed>(text).is_err(), "{text}");
        assert!(serde_json::from_str::<Angle>(text).is_err(), "{text}");
    }
}

#[test]
fn the_earth_and_its_datums_cannot_be_deformed_on_the_way_in() {
    use kinavis::{EcefPoint, Ellipsoid, Helmert};

    assert_eq!(round_trip(Ellipsoid::WGS84), Ellipsoid::WGS84);
    // A zero axis breaks every geodesic formula.
    assert!(serde_json::from_str::<Ellipsoid>(
        r#"{"semi_major_metres":0.0,"inverse_flattening":298.257}"#
    )
    .is_err());
    assert!(serde_json::from_str::<Ellipsoid>(
        r#"{"semi_major_metres":6378137.0,"inverse_flattening":0.5}"#
    )
    .is_err());
    assert!(serde_json::from_str::<Ellipsoid>(
        r#"{"semi_major_metres":"NaN","inverse_flattening":298.257}"#
    )
    .is_err());

    let shift = Helmert::position_vector([1.0, 2.0, 3.0], [0.1, 0.2, 0.3], 1.5).unwrap();
    assert_eq!(round_trip(shift), shift);
    assert!(serde_json::from_str::<Helmert>(
        r#"{"translation":[1.0,2.0,3.0],"rotation":[0.1,0.2,0.3],"scale_ppm":-2e6}"#
    )
    .is_err());
    assert!(serde_json::from_str::<Helmert>(
        r#"{"translation":[1.0,"inf",3.0],"rotation":[0.1,0.2,0.3],"scale_ppm":0.0}"#
    )
    .is_err());

    let point = EcefPoint::new(
        Distance::from_metres(1.0).unwrap(),
        Distance::from_metres(-2.0).unwrap(),
        Distance::from_metres(3.0).unwrap(),
    );
    assert_eq!(round_trip(point), point);
    assert!(serde_json::from_str::<EcefPoint>(r#"{"x":1.0,"y":"NaN","z":3.0}"#).is_err());
}

#[test]
fn an_error_ellipse_keeps_its_axes_in_order_on_the_way_in() {
    use kinavis::ErrorEllipse;

    let ellipse = ErrorEllipse::from_covariance(25.0, 5.0, 9.0).unwrap();
    assert_eq!(round_trip(ellipse), ellipse);
    // Minor axis longer than the major.
    assert!(serde_json::from_str::<ErrorEllipse>(
        r#"{"semi_major":0.001,"semi_minor":0.002,"orientation":45.0}"#
    )
    .is_err());
    assert!(serde_json::from_str::<ErrorEllipse>(
        r#"{"semi_major":-0.001,"semi_minor":-0.002,"orientation":45.0}"#
    )
    .is_err());
}

#[test]
fn a_tidal_cycle_that_runs_backwards_is_refused_on_the_way_in() {
    use kinavis::{TidalCycle, TideEvent};

    let low = TideEvent::new(
        Instant::from_unix_seconds(0),
        Distance::from_metres(0.5).unwrap(),
    );
    let high = TideEvent::new(
        Instant::from_unix_seconds(6 * 3600),
        Distance::from_metres(4.0).unwrap(),
    );
    let cycle = TidalCycle::new(low, high).unwrap();
    assert_eq!(round_trip(cycle), cycle);

    let text = serde_json::to_string(&cycle).unwrap();
    let backwards = text
        .replacen("\"from\"", "\"__\"", 1)
        .replacen("\"to\"", "\"from\"", 1)
        .replacen("\"__\"", "\"to\"", 1);
    assert!(serde_json::from_str::<TidalCycle>(&backwards).is_err());
}

#[test]
fn settings_round_trip_and_are_checked_on_the_way_in() {
    let turn = TurnParameters::new(
        TurnMode::Radius(Distance::from_nautical_miles(1.0).unwrap()),
        Distance::from_cables(9.0).unwrap(),
        Distance::from_cables(7.0).unwrap(),
    )
    .unwrap();
    assert_eq!(round_trip(turn), turn);
    let text = serde_json::to_string(&turn).unwrap();
    // Radius tighter than the transfer; transfer greater than advance.
    let tight = text.replace("\"Radius\":1.0", "\"Radius\":0.5");
    assert_ne!(tight, text);
    assert!(serde_json::from_str::<TurnParameters>(&tight).is_err());
    let inverted = text.replace("\"advance\":0.9", "\"advance\":0.5");
    assert_ne!(inverted, text);
    assert!(serde_json::from_str::<TurnParameters>(&inverted).is_err());

    let guidance = GuidanceConfig::new(
        Distance::from_cables(5.0).unwrap(),
        Distance::from_cables(2.0).unwrap(),
    )
    .unwrap()
    .with_leeway(Angle::from_degrees(3.0).unwrap())
    .anticipating_turns(turn);
    assert_eq!(round_trip(guidance), guidance);
    let text = serde_json::to_string(&guidance).unwrap();
    let negative = text.replace("\"arrival_radius\":0.2", "\"arrival_radius\":-0.2");
    assert_ne!(negative, text);
    assert!(serde_json::from_str::<GuidanceConfig>(&negative).is_err());

    let estimator = EstimatorConfig::standard()
        .with_late(LatePolicy::Reject)
        .with_suspect_after(0)
        .with_alert_limit(Distance::from_metres(50.0).unwrap())
        .unwrap();
    assert_eq!(round_trip(estimator), estimator);
    let text = serde_json::to_string(&estimator).unwrap();
    let no_limit = text.replace(
        "\"alert_limit\":0.026997840172786176",
        "\"alert_limit\":0.0",
    );
    assert_ne!(no_limit, text);
    assert!(serde_json::from_str::<EstimatorConfig>(&no_limit).is_err());

    // A deserialised cursor is checked only against the global maximum leg
    // count.
    let cursor = LegCursor::try_from(5).unwrap();
    assert_eq!(round_trip(cursor), cursor);
    assert_eq!(serde_json::to_string(&cursor).unwrap(), "5");
    assert!(serde_json::from_str::<LegCursor>("126").is_ok());
    assert!(serde_json::from_str::<LegCursor>("127").is_err());
}
