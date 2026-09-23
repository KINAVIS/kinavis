//! Value types round-trip through `serde`.

#![cfg(feature = "serde")]
#![allow(
    clippy::unwrap_used,
    clippy::indexing_slicing,
    clippy::needless_pass_by_value
)]

use core::time::Duration;
use kinavis_kernel::snapshot::GroundTrack;
use kinavis_kernel::{
    Angle, Distance, Instant, Position, RateOfTurn, Speed, TargetId, TrueCourse, Utc,
};
use kinavis_traffic::{
    CpaPolicy, ManoeuvreConstraints, PermittedSides, TargetObservation, TrackingPolicy, Traffic,
    WhenFull,
};

fn round_trip<T>(value: T) -> T
where
    T: serde::Serialize + serde::de::DeserializeOwned,
{
    let text = serde_json::to_string(&value).unwrap();
    serde_json::from_str(&text).unwrap()
}

#[test]
fn observations_policies_and_views_round_trip() {
    let observation = TargetObservation::new(
        TargetId::new(235_000_123),
        Position::from_degrees(50.0, -1.0).unwrap(),
        Instant::<Utc>::from_unix_seconds(1_789_000_000),
    )
    .with_ground_track(GroundTrack {
        course_over_ground: TrueCourse::new(45.0).unwrap(),
        speed_over_ground: Speed::from_knots(12.0).unwrap(),
    })
    .with_heading(TrueCourse::new(43.0).unwrap());
    assert_eq!(round_trip(observation), observation);

    let policy = TrackingPolicy::new(
        3,
        Duration::from_secs(30),
        Duration::from_secs(180),
        Speed::from_knots(60.0).unwrap(),
    )
    .unwrap()
    .when_full(WhenFull::EvictStalest);
    assert_eq!(round_trip(policy), policy);
    // A stored policy is validated on deserialisation.
    let text = serde_json::to_string(&policy).unwrap();
    let broken = text.replace("\"fixes_to_acquire\":3", "\"fixes_to_acquire\":0");
    assert_ne!(broken, text);
    assert!(serde_json::from_str::<TrackingPolicy>(&broken).is_err());

    let mut traffic = Traffic::new(policy);
    let _ = traffic.ingest(observation).unwrap();
    let view = traffic.view(observation.at());
    let seen = view.targets()[0];
    assert_eq!(round_trip(seen), seen);
    assert_eq!(format!("{}", seen.target), "#235000123");
}

#[test]
fn the_vessels_settings_round_trip_and_are_checked_on_the_way_in() {
    let policy = CpaPolicy::new(
        Distance::from_nautical_miles(2.0).unwrap(),
        Duration::from_secs(1200),
    )
    .unwrap();
    assert_eq!(round_trip(policy), policy);
    let text = serde_json::to_string(&policy).unwrap();
    let negative = text.replace("\"cpa_limit\":2.0", "\"cpa_limit\":-2.0");
    assert_ne!(negative, text);
    assert!(serde_json::from_str::<CpaPolicy>(&negative).is_err());

    let constraints = ManoeuvreConstraints::new(
        PermittedSides::Starboard,
        Angle::from_degrees(30.0).unwrap(),
        Angle::from_degrees(90.0).unwrap(),
    )
    .unwrap()
    .with_rate_of_turn(RateOfTurn::from_degrees_per_minute(10.0).unwrap())
    .unwrap();
    assert_eq!(round_trip(constraints), constraints);
    let text = serde_json::to_string(&constraints).unwrap();
    // Bounds out of order, zero rate, invalid side.
    let inverted = text.replace("\"most_alteration\":90.0", "\"most_alteration\":20.0");
    assert_ne!(inverted, text);
    assert!(serde_json::from_str::<ManoeuvreConstraints>(&inverted).is_err());
    let stuck = text.replace("\"rate_of_turn\":10.0", "\"rate_of_turn\":0.0");
    assert_ne!(stuck, text);
    assert!(serde_json::from_str::<ManoeuvreConstraints>(&stuck).is_err());
    let neither = text.replace("\"Starboard\"", "\"Neither\"");
    assert_ne!(neither, text);
    assert!(serde_json::from_str::<ManoeuvreConstraints>(&neither).is_err());
}
