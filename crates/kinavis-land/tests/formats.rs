//! GPX and `GeoJSON`: the 97 reference GPX files of `GPSBabel` — from Garmin
//! `BaseCamp`, `ExpertGPS`, Lowrance, Humminbird, `SkyTraq` loggers and others
//! — read as `gpxpy` reads them (`tools/gpx_reference.py`); round trips;
//! refusals.
//!
//! The files are GPL-2.0 data, not in the repository: `GPSBABEL_DATA` names a
//! checkout of `reference/*.gpx` at the commit in CI.

#![cfg(feature = "formats")]
#![allow(
    clippy::expect_used,
    clippy::panic,
    clippy::indexing_slicing,
    clippy::float_cmp,
    clippy::cast_precision_loss,
    clippy::doc_markdown
)]

use std::path::PathBuf;

use kinavis::route::{LegKind, Route};
use kinavis_kernel::time::{Instant, Utc};
use kinavis_kernel::{Distance, Height, Position};
use kinavis_land::{
    parse_geojson, to_geojson, Feature, Geometry, Gpx, GpxPoint, GpxRoute, GpxTrack, LandError,
};
use proptest::prelude::*;

const EXPECTED: &str = include_str!("data/gpsbabel_gpx.expected");

fn points(gpx: &Gpx) -> impl Iterator<Item = &GpxPoint> {
    gpx.waypoints
        .iter()
        .chain(gpx.routes.iter().flat_map(|r| r.points.iter()))
        .chain(gpx.tracks.iter().flat_map(|t| t.segments.iter().flatten()))
}

#[test]
#[ignore = "needs GPSBabel's reference files in GPSBABEL_DATA"]
fn gpsbabel_reference_files_read_as_gpxpy_reads_them() {
    let directory = PathBuf::from(std::env::var("GPSBABEL_DATA").expect("GPSBABEL_DATA"));
    let mut checked = 0;
    let mut total = 0usize;
    for line in EXPECTED.lines().filter(|line| !line.starts_with('#')) {
        let fields: Vec<&str> = line.split_whitespace().collect();
        let name = fields[0];
        let bytes = std::fs::read(directory.join("reference").join(name)).expect("a file");
        let text = String::from_utf8_lossy(&bytes);
        let gpx = Gpx::parse(&text).unwrap_or_else(|error| panic!("{name}: {error}"));
        let count = |i: usize| fields[i].parse::<usize>().expect("a count");
        let float = |i: usize| fields[i].parse::<f64>().expect("a number");
        assert_eq!(gpx.waypoints.len(), count(1), "{name}: waypoints");
        assert_eq!(gpx.routes.len(), count(2), "{name}: routes");
        assert_eq!(
            gpx.routes.iter().map(|r| r.points.len()).sum::<usize>(),
            count(3),
            "{name}: route points"
        );
        assert_eq!(gpx.tracks.len(), count(4), "{name}: tracks");
        assert_eq!(
            gpx.tracks.iter().map(|t| t.segments.len()).sum::<usize>(),
            count(5),
            "{name}: segments"
        );
        let track_points: usize = gpx
            .tracks
            .iter()
            .flat_map(|t| t.segments.iter())
            .map(Vec::len)
            .sum();
        assert_eq!(track_points, count(6), "{name}: track points");
        let n = (count(1) + count(3) + count(6)).max(1) as f64;
        let lat: f64 = points(&gpx).map(|p| p.position.latitude().degrees()).sum();
        let lon: f64 = points(&gpx).map(|p| p.position.longitude().degrees()).sum();
        assert!(
            (lat - float(7)).abs() < 1e-9 * n,
            "{name}: latitudes {lat} against {}",
            float(7)
        );
        assert!(
            (lon - float(8)).abs() < 1e-9 * n,
            "{name}: longitudes {lon} against {}",
            float(8)
        );
        let elevations: Vec<f64> = points(&gpx)
            .filter_map(|p| p.elevation)
            .map(|h| h.value().metres())
            .collect();
        assert_eq!(elevations.len(), count(9), "{name}: elevations");
        assert!(
            (elevations.iter().sum::<f64>() - float(10)).abs() < 1e-6 * n,
            "{name}: elevation sum"
        );
        let times = points(&gpx).filter(|p| p.time.is_some()).count();
        assert_eq!(times, count(11), "{name}: times");
        let seconds: f64 = points(&gpx)
            .filter_map(|p| p.time)
            .map(|t| {
                t.checked_duration_since(Instant::<Utc>::UNIX_EPOCH)
                    .map_or_else(
                        || {
                            -Instant::<Utc>::UNIX_EPOCH
                                .checked_duration_since(t)
                                .expect("before")
                                .as_secs_f64()
                        },
                        |d| d.as_secs_f64(),
                    )
            })
            .sum();
        assert!(
            (seconds - float(12)).abs() < 1e-3 * n,
            "{name}: times {seconds} against {}",
            float(12)
        );
        // What is read is written and read back the same.
        let again = Gpx::parse(&gpx.to_xml()).expect("our own output");
        assert_eq!(again.waypoints.len(), gpx.waypoints.len());
        for (a, b) in points(&gpx).zip(points(&again)) {
            assert!(
                (a.position.latitude().degrees() - b.position.latitude().degrees()).abs() < 1e-9
            );
            assert_eq!(a.time, b.time, "{name}");
            assert_eq!(a.name, b.name, "{name}");
        }
        total += count(1) + count(3) + count(6);
        checked += 1;
    }
    assert_eq!(checked, 97);
    assert_eq!(total, 2201 + 1586 + 248_170);
}

fn time(text: &str) -> Instant<Utc> {
    let gpx = Gpx::parse(&format!(
        "<gpx><wpt lat=\"0\" lon=\"0\"><time>{text}</time></wpt></gpx>"
    ))
    .expect("a time");
    gpx.waypoints[0].time.expect("read")
}

#[test]
fn times_in_every_form_gpx_files_use() {
    let base = time("2009-01-17T20:27:37Z");
    assert_eq!(base.civil().hour, 20);
    assert_eq!(time("2009-01-17T20:27:37"), base);
    assert_eq!(time("2009-01-17T21:27:37+01:00"), base);
    assert_eq!(time("2009-01-17T15:27:37-0500"), base);
    assert_eq!(time("2009-01-17T20:27:37.250Z").civil().nanos, 250_000_000);
    assert_eq!(time("2016-12-31T23:59:60Z").civil().day, 1);
    for bad in [
        "2009-01-17",
        "2009-13-17T20:27:37Z",
        "yesterday",
        "2009-01-17T20:27:37Q",
    ] {
        assert!(
            Gpx::parse(&format!(
                "<gpx><wpt lat=\"0\" lon=\"0\"><time>{bad}</time></wpt></gpx>"
            ))
            .is_err(),
            "{bad}"
        );
    }
}

#[test]
fn malformed_files_are_refused_with_their_line() {
    for (text, what) in [
        ("<kml/>", "root element gpx"),
        ("", "root element gpx"),
        ("<gpx>\n<wpt lat=\"91\" lon=\"0\"/></gpx>", "lat"),
        ("<gpx>\n\n<wpt lon=\"0\"/></gpx>", "lat"),
        ("<gpx><wpt lat=\"1\" lon=\"x\"/></gpx>", "lon"),
        (
            "<gpx><wpt lat=\"1\" lon=\"2\"><ele>high</ele></wpt></gpx>",
            "ele",
        ),
        ("<gpx><wpt lat=\"1\"", "XML"),
    ] {
        match Gpx::parse(text) {
            Err(LandError::Format { what: found, .. }) => assert_eq!(found, what, "{text:?}"),
            other => panic!("{text:?}: {other:?}"),
        }
    }
    let Err(LandError::Format { line, .. }) = Gpx::parse("<gpx>\n\n<wpt lon=\"0\"/></gpx>") else {
        panic!("refused");
    };
    assert_eq!(line, 3);
}

#[test]
fn names_with_markup_and_entities_survive_the_round_trip() {
    let text = "<gpx><wpt lat=\"1\" lon=\"2\"><name>R&amp;D &lt;north&gt; &#x41;&#66; &quot;x&quot;</name></wpt>\
                <rte><name><![CDATA[a < b]]></name><rtept lat=\"1\" lon=\"2\"/></rte></gpx>";
    let gpx = Gpx::parse(text).expect("a file");
    assert_eq!(
        gpx.waypoints[0].name.as_deref(),
        Some("R&D <north> AB \"x\"")
    );
    assert_eq!(gpx.routes[0].name.as_deref(), Some("a < b"));
    let written = Gpx {
        creator: Some(String::from("KINAVIS")),
        ..gpx.clone()
    };
    assert_eq!(Gpx::parse(&gpx.to_xml()).expect("again"), written);
}

prop_compose! {
    fn a_point()(lat in -90.0f64..90.0, lon in -180.0f64..180.0, ele in proptest::option::of(-400.0f64..9000.0),
                 secs in proptest::option::of(0i64..4_000_000_000), name in proptest::option::of("[ -~]{0,12}"))
                -> GpxPoint {
        GpxPoint {
            position: Position::from_degrees(lat, lon).expect("on Earth"),
            elevation: ele.map(|e| Height::above_mean_sea_level(Distance::from_metres(e).expect("finite"))),
            time: secs.map(|s| Instant::<Utc>::new(s, 0).expect("a time")),
            name,
        }
    }
}

proptest! {
    #[test]
    fn any_file_written_reads_back_equal(
        waypoints in proptest::collection::vec(a_point(), 0..5),
        route in proptest::collection::vec(a_point(), 0..5),
        segment in proptest::collection::vec(a_point(), 0..8),
    ) {
        let gpx = Gpx {
            creator: Some(String::from("test & co")),
            waypoints,
            routes: vec![GpxRoute { name: Some(String::from("route")), points: route }],
            tracks: vec![GpxTrack { name: None, segments: vec![segment] }],
        };
        let again = Gpx::parse(&gpx.to_xml()).expect("our own output");
        prop_assert_eq!(again.creator.as_deref(), Some("test & co"));
        for (a, b) in points(&gpx).zip(points(&again)) {
            prop_assert!((a.position.latitude().degrees() - b.position.latitude().degrees()).abs() < 1e-9);
            prop_assert!((a.position.longitude().degrees() - b.position.longitude().degrees()).abs() < 1e-9);
            prop_assert_eq!(a.elevation, b.elevation);
            prop_assert_eq!(a.time, b.time);
            prop_assert_eq!(a.name.as_deref().map(str::trim), b.name.as_deref());
        }
    }
}

#[test]
fn a_route_and_a_track_turn_into_a_route_and_fixes() {
    let positions: Vec<Position> = (0..300)
        .map(|k| Position::from_degrees(55.0 + f64::from(k) * 1e-4, 37.0).expect("on Earth"))
        .collect();
    let route = GpxRoute {
        name: None,
        points: positions.iter().map(|p| GpxPoint::at(*p)).collect(),
    };
    // 300 points on a meridian: a straight route of two.
    assert_eq!(
        route
            .to_route(Distance::from_metres(1.0).expect("1 m"))
            .expect("a route")
            .waypoints()
            .len(),
        2
    );
    let kinavis = Route::new(&positions[..10], LegKind::RhumbLine).expect("a route");
    assert_eq!(GpxRoute::from_route(&kinavis, Some("ten")).points.len(), 10);
    let mut timed = GpxPoint::at(positions[0]);
    timed.time = Some(Instant::<Utc>::new(1_232_224_057, 0).expect("a time"));
    let track = GpxTrack {
        name: None,
        segments: vec![vec![timed.clone(), GpxPoint::at(positions[1]), timed]],
    };
    assert_eq!(track.fixes().count(), 2);
}

#[test]
fn geojson_features_read_back_and_polygons_turn_the_right_way() {
    let p = |lat: f64, lon: f64| Position::from_degrees(lat, lon).expect("on Earth");
    // An outer ring given clockwise and a hole given anticlockwise.
    let outer = vec![p(0.0, 0.0), p(1.0, 0.0), p(1.0, 1.0), p(0.0, 1.0)];
    let hole = vec![p(0.2, 0.2), p(0.2, 0.4), p(0.4, 0.4), p(0.4, 0.2)];
    let route = Route::new(&[p(55.0, 37.0), p(55.1, 37.2)], LegKind::RhumbLine).expect("a route");
    let features = vec![
        Feature {
            geometry: Geometry::Point(p(55.75, 37.62)),
            name: Some(String::from("Moscow")),
        },
        Feature::route(&route, None),
        Feature {
            geometry: Geometry::Polygon(vec![outer, hole]),
            name: Some(String::from("field")),
        },
    ];
    let text = to_geojson(&features);
    let back = parse_geojson(&text).expect("our own output");
    assert_eq!(back.len(), 3);
    assert_eq!(back[0], features[0]);
    assert_eq!(back[1].geometry, features[1].geometry);
    let Geometry::Polygon(rings) = &back[2].geometry else {
        panic!("a polygon")
    };
    let signed = |ring: &[Position]| -> f64 {
        ring.windows(2)
            .map(|w| {
                w[0].longitude().degrees() * w[1].latitude().degrees()
                    - w[1].longitude().degrees() * w[0].latitude().degrees()
            })
            .sum()
    };
    assert!(signed(&rings[0]) > 0.0, "outer anticlockwise");
    assert!(signed(&rings[1]) < 0.0, "hole clockwise");
    assert_eq!(rings[0].first(), rings[0].last(), "closed");
    // Multi-geometries split, a collection read, a bare geometry read.
    let multi = r#"{"type":"Feature","properties":{"name":"two"},"geometry":{"type":"MultiPoint","coordinates":[[37.0,55.0],[38.0,56.0,120.0]]}}"#;
    let parts = parse_geojson(multi).expect("a feature");
    assert_eq!(parts.len(), 2);
    assert!(parts.iter().all(|f| f.name.as_deref() == Some("two")));
    assert_eq!(
        parse_geojson(
            r#"{"type":"GeometryCollection","geometries":[{"type":"Point","coordinates":[1,2]}]}"#
        )
        .expect("ok")
        .len(),
        1
    );
    for bad in [
        r#"{"type":"Point","coordinates":[1]}"#,
        r#"{"type":"Point","coordinates":[0,95]}"#,
        "{",
        r#"{"type":"Circle","coordinates":[]}"#,
    ] {
        assert!(parse_geojson(bad).is_err(), "{bad}");
    }
}
