//! The watch over deltas as a Signal K server sends them.

#![allow(
    clippy::unwrap_used,
    clippy::panic,
    clippy::indexing_slicing,
    clippy::float_cmp
)]

use kinavis_kernel::time::{Instant, Utc};
use kinavis_signalk::{parse_timestamp, Emission, Level, Watch, WatchConfig};
use serde_json::{json, Value};

const TARGET: &str = "vessels.urn:mrn:imo:mmsi:244000001";
const KNOTS_6: f64 = 3.086_666;
const WEST: f64 = 3.0 * std::f64::consts::FRAC_PI_2;

fn at(time: &str) -> Instant<Utc> {
    parse_timestamp(time).unwrap()
}

fn delta(context: &str, time: &str, values: &Value) -> String {
    json!({ "context": context, "updates": [{ "timestamp": time, "values": values }] }).to_string()
}

/// Own vessel at 53°N 5°E steering north at 6 knots.
fn own(watch: &mut Watch, context: &str, time: &str) {
    watch
        .ingest(&delta(
            context,
            time,
            &json!([
                { "path": "navigation.position", "value": { "latitude": 53.0, "longitude": 5.0 } },
                { "path": "navigation.courseOverGroundTrue", "value": 0.0 },
                { "path": "navigation.speedOverGround", "value": KNOTS_6 },
            ]),
        ))
        .unwrap();
}

/// A target at a position, steering west at 6 knots, with extra values.
fn target(watch: &mut Watch, time: &str, latitude: f64, longitude: f64, extra: &[Value]) {
    let mut values = vec![
        json!({ "path": "navigation.position", "value": { "latitude": latitude, "longitude": longitude } }),
        json!({ "path": "navigation.courseOverGroundTrue", "value": WEST }),
        json!({ "path": "navigation.speedOverGround", "value": KNOTS_6 }),
    ];
    values.extend_from_slice(extra);
    watch
        .ingest(&delta(TARGET, time, &Value::Array(values)))
        .unwrap();
}

fn notifications(emissions: &[Emission]) -> Vec<(String, Value)> {
    emissions
        .iter()
        .filter_map(|emission| match emission {
            Emission::Notification { path, value } => Some((path.clone(), value.clone())),
            _ => None,
        })
        .collect()
}

#[test]
fn a_crossing_target_on_the_starboard_bow_raises_one_alarm_until_it_clears() {
    let mut watch = Watch::new(WatchConfig::default());
    own(&mut watch, "vessels.self", "2026-09-25T12:00:00Z");
    target(&mut watch, "2026-09-25T12:00:00Z", 53.025, 5.041_55, &[]);

    let (reports, emissions) = watch.assess(at("2026-09-25T12:00:01Z"));
    assert_eq!(reports.len(), 1);
    assert_eq!(reports[0].level, Level::Alarm);
    let raised = notifications(&emissions);
    assert_eq!(raised.len(), 1);
    assert_eq!(
        raised[0].0,
        "notifications.navigation.closestApproach.urn:mrn:imo:mmsi:244000001"
    );
    assert_eq!(raised[0].1["state"], "alarm");
    assert_eq!(raised[0].1["method"], json!(["visual", "sound"]));
    // A silent target counts as power-driven: Rule 15, own vessel gives way.
    let message = raised[0].1["message"].as_str().unwrap();
    assert!(
        message.contains("crossing, target on starboard, give way (Rule 15)"),
        "{message}"
    );
    // CPA and TCPA in the target's context, in metres and seconds.
    let Some(Emission::Delta(cpa)) = emissions.first() else {
        panic!("no closestApproach delta")
    };
    assert_eq!(cpa.context.as_deref(), Some(TARGET));
    let value = &cpa.updates[0].values[0].value;
    assert!(
        (value["timeTo"].as_f64().unwrap() - 900.0).abs() < 5.0,
        "{value}"
    );

    // The same picture again: the alarm stands, nothing new is notified.
    let (_, emissions) = watch.assess(at("2026-09-25T12:00:01Z"));
    assert!(notifications(&emissions).is_empty());

    // The target turns away north-east, opening: the alarm is cleared.
    watch
        .ingest(&delta(
            TARGET,
            "2026-09-25T12:00:10Z",
            &json!([{ "path": "navigation.courseOverGroundTrue", "value": 0.8 }]),
        ))
        .unwrap();
    let (_, emissions) = watch.assess(at("2026-09-25T12:00:11Z"));
    let cleared = notifications(&emissions);
    assert_eq!(cleared.len(), 1);
    assert_eq!(cleared[0].1["state"], "normal");
}

#[test]
fn a_target_gone_silent_is_cleared_once_stale() {
    let mut watch = Watch::new(WatchConfig::default());
    own(&mut watch, "vessels.self", "2026-09-25T12:00:00Z");
    target(&mut watch, "2026-09-25T12:00:00Z", 53.025, 5.041_55, &[]);
    assert_eq!(
        notifications(&watch.assess(at("2026-09-25T12:00:01Z")).1)[0].1["state"],
        "alarm"
    );

    // Own vessel reports for four minutes; the target does not.
    own(&mut watch, "vessels.self", "2026-09-25T12:04:00Z");
    let (reports, emissions) = watch.assess(at("2026-09-25T12:04:00Z"));
    assert!(reports.is_empty());
    let cleared = notifications(&emissions);
    assert_eq!(cleared[0].1["state"], "normal");
    assert_eq!(cleared[0].1["message"], "no longer assessed");
}

#[test]
fn a_sailing_vessel_by_its_ais_ship_type_is_given_way_to_under_rule_18() {
    let mut watch = Watch::new(WatchConfig::default());
    own(&mut watch, "vessels.self", "2026-09-25T12:00:00Z");
    // On the port bow: by Rule 15 alone own vessel would stand on.
    target(
        &mut watch,
        "2026-09-25T12:00:00Z",
        53.025,
        4.958_45,
        &[json!({ "path": "design.aisShipType", "value": { "id": 36, "name": "Sailing" } })],
    );
    watch
        .ingest(&delta(
            TARGET,
            "2026-09-25T12:00:00Z",
            &json!([{ "path": "navigation.courseOverGroundTrue", "value": std::f64::consts::FRAC_PI_2 }]),
        ))
        .unwrap();
    let (reports, _) = watch.assess(at("2026-09-25T12:00:01Z"));
    let message = kinavis_signalk::message(&reports[0]);
    assert!(message.contains("give way (Rule 18)"), "{message}");
}

#[test]
fn a_vessel_at_anchor_gets_a_cpa_but_no_ruling() {
    let mut watch = Watch::new(WatchConfig::default());
    own(&mut watch, "vessels.self", "2026-09-25T12:00:00Z");
    // Dead ahead, a mile off, at anchor.
    watch
        .ingest(&delta(
            TARGET,
            "2026-09-25T12:00:00Z",
            &json!([
                { "path": "navigation.position", "value": { "latitude": 53.016_67, "longitude": 5.0 } },
                { "path": "navigation.speedOverGround", "value": 0.0 },
                { "path": "navigation.state", "value": "anchored" },
            ]),
        ))
        .unwrap();
    let (reports, _) = watch.assess(at("2026-09-25T12:00:01Z"));
    assert_eq!(reports[0].level, Level::Alarm);
    assert_eq!(reports[0].ruling, None);
}

#[test]
fn own_vessel_is_known_by_its_full_context_once_named() {
    let own_context = "vessels.urn:mrn:imo:mmsi:230000001";
    let mut watch = Watch::new(WatchConfig::default());
    own(&mut watch, "vessels.self", "2026-09-25T12:00:00Z");
    watch.picture_mut().set_own_context(own_context);
    // The server writes own vessel's deltas under its full context.
    own(&mut watch, own_context, "2026-09-25T12:00:01Z");
    target(&mut watch, "2026-09-25T12:00:01Z", 53.025, 5.041_55, &[]);
    let (reports, _) = watch.assess(at("2026-09-25T12:00:02Z"));
    assert_eq!(reports.len(), 1, "own vessel is not its own target");
    assert_eq!(watch.picture().len(), 2);
}

#[test]
fn nothing_is_assessed_without_own_motion_and_bad_input_is_an_error() {
    let mut watch = Watch::new(WatchConfig::default());
    target(&mut watch, "2026-09-25T12:00:00Z", 53.025, 5.041_55, &[]);
    assert_eq!(
        watch.assess(at("2026-09-25T12:00:01Z")),
        (Vec::new(), Vec::new())
    );
    assert!(watch.ingest("not json").is_err());
    // Paths the watch does not use, and unreadable values, are ignored.
    watch
        .ingest(&delta(
            TARGET,
            "2026-09-25T12:00:01Z",
            &json!([
                { "path": "environment.wind.speedApparent", "value": 5.0 },
                { "path": "navigation.speedOverGround", "value": "fast" },
            ]),
        ))
        .unwrap();
}

#[test]
fn the_configuration_reads_from_the_plugin_settings() {
    let config: WatchConfig =
        serde_json::from_str(r#"{"cpaLimitNm": 0.5, "restrictedVisibility": true}"#).unwrap();
    assert_eq!(config.cpa_limit_nm, 0.5);
    assert!(config.restricted_visibility);
    assert_eq!(config.tcpa_limit_min, WatchConfig::default().tcpa_limit_min);
}

#[test]
fn own_vessel_alongside_with_noise_and_no_course_is_at_rest() {
    let mut watch = Watch::new(WatchConfig::default());
    // A receiver at rest: 5 mm/s of noise, course null.
    watch
        .ingest(&delta(
            "vessels.self",
            "2026-09-25T12:00:00Z",
            &json!([
                { "path": "navigation.position", "value": { "latitude": 53.0, "longitude": 5.0 } },
                { "path": "navigation.courseOverGroundTrue", "value": null },
                { "path": "navigation.speedOverGround", "value": 0.0046 },
            ]),
        ))
        .unwrap();
    target(&mut watch, "2026-09-25T12:00:00Z", 53.0, 5.02, &[]);
    let (reports, _) = watch.assess(at("2026-09-25T12:00:01Z"));
    assert_eq!(reports.len(), 1);
    assert!(reports[0].assessment.cpa_distance().is_some());
}

#[test]
fn own_vessel_is_learnt_from_its_own_sensors() {
    let own_context = "vessels.urn:mrn:signalk:uuid:cdf30fb9-28bf-4d10-8817-04a6b3d96e33";
    let mut watch = Watch::new(WatchConfig::default());
    // A target first, from AIS: not own vessel.
    watch
        .ingest(
            &json!({ "context": TARGET, "updates": [{
                "timestamp": "2026-09-25T12:00:00Z",
                "source": { "type": "NMEA0183", "sentence": "VDM", "talker": "AI" },
                "values": [
                    { "path": "navigation.position", "value": { "latitude": 53.025, "longitude": 5.041_55 } },
                    { "path": "navigation.courseOverGroundTrue", "value": WEST },
                    { "path": "navigation.speedOverGround", "value": KNOTS_6 },
                ]}]})
            .to_string(),
        )
        .unwrap();
    assert!(!watch.picture().own_is_named());
    // Then the GNSS receiver: its RMC is own vessel, under the server's
    // identity for it.
    watch
        .ingest(
            &json!({ "context": own_context, "updates": [{
            "timestamp": "2026-09-25T12:00:00Z",
            "source": { "type": "NMEA0183", "sentence": "RMC", "talker": "GP" },
            "values": [
                { "path": "navigation.position", "value": { "latitude": 53.0, "longitude": 5.0 } },
                { "path": "navigation.courseOverGroundTrue", "value": 0.0 },
                { "path": "navigation.speedOverGround", "value": KNOTS_6 },
            ]}]})
            .to_string(),
        )
        .unwrap();
    assert_eq!(watch.picture().own_context(), own_context);
    let (reports, _) = watch.assess(at("2026-09-25T12:00:01Z"));
    assert_eq!(reports.len(), 1);
    assert_eq!(reports[0].context, TARGET);
}

#[test]
fn a_receiver_clock_ahead_of_the_watch_is_current_and_own_vessel_can_go_stale() {
    let mut watch = Watch::new(WatchConfig::default());
    // The GNSS receiver stamps by its fix, eleven hours ahead of the server.
    own(&mut watch, "vessels.self", "2026-09-25T23:00:00Z");
    target(&mut watch, "2026-09-25T12:00:00Z", 53.025, 5.041_55, &[]);
    assert_eq!(watch.assess(at("2026-09-25T12:00:01Z")).0.len(), 1);

    // A receiver silent for four minutes: nothing is assessed against it.
    let mut watch = Watch::new(WatchConfig::default());
    own(&mut watch, "vessels.self", "2026-09-25T12:00:00Z");
    target(&mut watch, "2026-09-25T12:04:00Z", 53.025, 5.041_55, &[]);
    assert!(watch.assess(at("2026-09-25T12:04:01Z")).0.is_empty());
}

#[test]
fn own_vessel_at_rest_gets_cpas_but_no_ruling_and_far_warnings_are_not_raised() {
    let mut watch = Watch::new(WatchConfig::default());
    watch
        .ingest(&delta(
            "vessels.self",
            "2026-09-25T12:00:00Z",
            &json!([
                { "path": "navigation.position", "value": { "latitude": 53.0, "longitude": 5.0 } },
                { "path": "navigation.courseOverGroundTrue", "value": 2.1 },
                { "path": "navigation.speedOverGround", "value": 0.05 },
            ]),
        ))
        .unwrap();
    // Two miles east, steering west at 6 knots: straight at own vessel.
    target(&mut watch, "2026-09-25T12:00:00Z", 53.0, 5.055_4, &[]);
    let (reports, _) = watch.assess(at("2026-09-25T12:00:01Z"));
    assert_eq!(
        reports[0].level,
        Level::Warn,
        "TCPA 20 min: beyond the alarm limit"
    );
    assert_eq!(reports[0].ruling, None);

    // The same approach from forty miles: CPA is dangerous, but not within
    // the hour.
    let mut watch = Watch::new(WatchConfig::default());
    own(&mut watch, "vessels.self", "2026-09-25T12:00:00Z");
    watch
        .ingest(&delta(
            TARGET,
            "2026-09-25T12:00:00Z",
            &json!([
                { "path": "navigation.position", "value": { "latitude": 53.666_7, "longitude": 5.0 } },
                { "path": "navigation.courseOverGroundTrue", "value": std::f64::consts::PI },
                { "path": "navigation.speedOverGround", "value": KNOTS_6 },
            ]),
        ))
        .unwrap();
    let (reports, emissions) = watch.assess(at("2026-09-25T12:00:01Z"));
    assert_eq!(reports[0].level, Level::Normal);
    assert!(notifications(&emissions).is_empty());
}
