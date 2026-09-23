//! Serialisation round-trips and cannot bypass invariants.
//!
//! Non-unit quaternions, invalid IMU intervals and zero priors are rejected by
//! the constructors and must be rejected on deserialisation too.

#![cfg(all(feature = "serde", feature = "std"))]
#![allow(clippy::unwrap_used, clippy::float_cmp, clippy::needless_pass_by_value)]

use core::time::Duration;

use kinavis_ins::{ImuNoise, ImuSample, InsMotion, InsPriors, Quaternion};
use kinavis_kernel::local::{Ned, Vector3};
use kinavis_kernel::units::{Angle, RateOfTurn, Speed};
use kinavis_kernel::TrueCourse;

fn round_trip<T>(value: T) -> T
where
    T: serde::Serialize + serde::de::DeserializeOwned,
{
    let text = serde_json::to_string(&value).unwrap();
    serde_json::from_str(&text).unwrap()
}

#[test]
fn a_quaternion_comes_back_a_unit_or_not_at_all() {
    let rotation = Quaternion::from_euler(
        Angle::from_degrees(5.0).unwrap(),
        Angle::from_degrees(-3.0).unwrap(),
        TrueCourse::new(47.0).unwrap(),
    );
    assert_eq!(round_trip(rotation), rotation);
    // All zeros or NaN: no rotation.
    assert!(serde_json::from_str::<Quaternion>(r#"{"w":0.0,"x":0.0,"y":0.0,"z":0.0}"#).is_err());
    assert!(serde_json::from_str::<Quaternion>(r#"{"w":"NaN","x":0.0,"y":0.0,"z":0.0}"#).is_err());
    // A stored non-unit quaternion is normalised, as in `new`.
    let scaled =
        serde_json::from_str::<Quaternion>(r#"{"w":2.0,"x":0.0,"y":0.0,"z":0.0}"#).unwrap();
    assert_eq!(scaled, Quaternion::IDENTITY);
}

#[test]
fn an_imu_sample_keeps_its_interval_honest() {
    let sample = ImuSample::new(
        [0.01, 0.0, -0.02],
        [0.1, 0.0, -9.8],
        Duration::from_millis(10),
    )
    .unwrap();
    assert_eq!(round_trip(sample), sample);
    assert!(serde_json::from_str::<ImuSample>(
        r#"{"angular_rate":[0.0,0.0,0.0],"specific_force":[0.0,0.0,-9.8],"over":{"secs":0,"nanos":0}}"#
    )
    .is_err());
    assert!(serde_json::from_str::<ImuSample>(
        r#"{"angular_rate":[0.0,"inf",0.0],"specific_force":[0.0,0.0,-9.8],"over":{"secs":0,"nanos":10000000}}"#
    )
    .is_err());
}

#[test]
fn a_noise_model_and_the_priors_stay_positive() {
    assert_eq!(round_trip(ImuNoise::mems()), ImuNoise::mems());
    assert!(serde_json::from_str::<ImuNoise>(
        r#"{"arw":0.0,"vrw":0.001,"gyro_bias":1e-5,"accel_bias":1e-4}"#
    )
    .is_err());

    assert_eq!(round_trip(InsPriors::standard()), InsPriors::standard());
    assert!(serde_json::from_str::<InsPriors>(
        r#"{"horizontal":10.0,"vertical":20.0,"velocity":0.5,"tilt":0.01,"heading":0.05,"gyro_bias":0.0,"accel_bias":0.02}"#
    )
    .is_err());
}

#[test]
fn a_motion_model_cannot_walk_nowhere() {
    let motion = InsMotion::new(
        Vector3::<Ned, Speed>::new(
            Speed::from_metres_per_second(3.0).unwrap(),
            Speed::from_metres_per_second(-1.0).unwrap(),
            Speed::ZERO,
        ),
        RateOfTurn::from_degrees_per_minute(30.0).unwrap(),
        Speed::from_metres_per_second(0.1).unwrap(),
        Angle::from_degrees(0.5).unwrap(),
    )
    .unwrap();
    assert_eq!(round_trip(motion), motion);
    assert!(serde_json::from_str::<InsMotion>(
        r#"{"velocity":[3.0,-1.0],"yaw_rate":0.01,"position_walk":0.0,"heading_walk":0.01}"#
    )
    .is_err());
    assert!(serde_json::from_str::<InsMotion>(
        r#"{"velocity":["NaN",-1.0],"yaw_rate":0.01,"position_walk":0.1,"heading_walk":0.01}"#
    )
    .is_err());
}
