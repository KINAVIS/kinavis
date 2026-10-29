//! Pedestrian dead reckoning on constructed signals: the stance test, steps
//! and their length, the tilt-compensated magnetic heading.

#![allow(
    clippy::expect_used,
    clippy::panic,
    clippy::float_cmp,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::indexing_slicing
)]

use core::time::Duration;

use kinavis_ins::{
    tilt_compensated_heading, weinberg_length, ImuSample, Quaternion, StanceConfig, StanceDetector,
    Step, StepConfig, StepDetector, MAX_STANCE_WINDOW,
};
use kinavis_kernel::{Angle, TrueCourse};
use proptest::prelude::*;

const G: f64 = 9.81;

fn sample(rate: [f64; 3], force: [f64; 3], dt: f64) -> ImuSample {
    ImuSample::new(rate, force, Duration::from_secs_f64(dt)).expect("a sample")
}

#[test]
fn a_foot_at_rest_stands_and_a_swinging_one_does_not() {
    let config = StanceConfig::new(5, 0.01, 0.1f64.to_radians(), G, 1e5).expect("settings");
    let mut detector = StanceDetector::new(config);
    // Tilted and still: gravity in another direction, no rotation.
    let tilted = [3.0, -1.0, -(G * G - 10.0f64).sqrt()];
    let mut results = Vec::new();
    for _ in 0..10 {
        results.extend(detector.push(&sample([0.0; 3], tilted, 0.005)));
    }
    assert_eq!(results.len(), 2, "one result per window of five");
    assert!(results
        .iter()
        .all(|stance| stance.standing && stance.statistic < 1e-6));
    // Swinging: 2 rad/s and 3 m/s² of extra force.
    for _ in 0..5 {
        results.extend(detector.push(&sample([2.0, 0.0, 0.0], [3.0, 0.0, -G], 0.005)));
    }
    let swing = results.last().expect("a third window");
    assert!(!swing.standing && swing.statistic > 1e5);
    // The statistic of a window: noise variances weight force and rate.
    let mut one = StanceDetector::new(StanceConfig::new(1, 0.1, 0.2, G, 1.0).expect("settings"));
    let single = one
        .push(&sample([0.2, 0.0, 0.0], [0.0, 0.0, -G], 0.005))
        .expect("a window");
    assert!(
        (single.statistic - 1.0).abs() < 1e-12,
        "(0.2 / 0.2)²: {}",
        single.statistic
    );
    assert!(!single.standing, "below the threshold, not at it");
}

#[test]
fn stance_settings_out_of_range_are_refused() {
    assert!(StanceConfig::new(0, 0.01, 0.01, G, 1.0).is_err());
    assert!(StanceConfig::new(MAX_STANCE_WINDOW + 1, 0.01, 0.01, G, 1.0).is_err());
    assert!(StanceConfig::new(5, 0.0, 0.01, G, 1.0).is_err());
    assert!(StanceConfig::new(5, 0.01, 0.01, 1.62, 1.0).is_err());
    assert!(StanceConfig::new(5, 0.01, 0.01, G, f64::NAN).is_err());
    assert!(StanceConfig::new(MAX_STANCE_WINDOW, 0.01, 0.01, G, 1.0).is_ok());
}

/// A walk at `cadence` steps per second: the vertical specific force swings
/// `amplitude` about gravity, sampled at 100 Hz.
fn walk(cadence: f64, amplitude: f64, seconds: f64) -> Vec<ImuSample> {
    let dt = 0.01;
    (0..(seconds / dt) as usize)
        .map(|k| {
            let t = k as f64 * dt;
            let extra = amplitude * (2.0 * core::f64::consts::PI * cadence * t).sin();
            sample([0.0; 3], [0.0, 0.0, -(G + extra)], dt)
        })
        .collect()
}

#[test]
fn steps_are_counted_at_their_cadence() {
    let config = StepConfig::new(
        1.0,
        0.0,
        Duration::from_millis(250),
        Duration::from_secs(2),
        G,
    )
    .expect("settings");
    for cadence in [1.5, 1.8, 2.2] {
        let mut detector = StepDetector::new(config);
        let steps: Vec<Step> = walk(cadence, 2.5, 20.0)
            .iter()
            .filter_map(|s| detector.push(s))
            .collect();
        let expected = (20.0 * cadence) as usize;
        assert!(
            steps.len().abs_diff(expected) <= 1,
            "{cadence}/s: {} steps",
            steps.len()
        );
        assert_eq!(steps[0].since_previous, None);
        for step in &steps[2..] {
            let interval = step.since_previous.expect("in stride").as_secs_f64();
            assert!((interval - 1.0 / cadence).abs() < 0.015, "{interval}");
            assert!((step.peak - (G + 2.5)).abs() < 0.01);
            assert!((step.trough - (G - 2.5)).abs() < 0.01);
        }
    }
    // Too gentle a swing is no step; a pause starts a new walk.
    let mut detector = StepDetector::new(config);
    assert_eq!(
        walk(2.0, 0.5, 10.0)
            .iter()
            .filter_map(|s| detector.push(s))
            .count(),
        0
    );
    let mut detector = StepDetector::new(config);
    let mut samples = walk(2.0, 2.5, 3.0);
    samples.extend((0..300).map(|_| sample([0.0; 3], [0.0, 0.0, -G], 0.01)));
    samples.extend(walk(2.0, 2.5, 3.0));
    let restarts = samples
        .iter()
        .filter_map(|s| detector.push(s))
        .filter(|step| step.since_previous.is_none())
        .count();
    assert_eq!(restarts, 2);
}

#[test]
fn step_settings_out_of_range_are_refused() {
    let (short, long) = (Duration::from_millis(250), Duration::from_secs(2));
    assert!(StepConfig::new(0.01, 0.0, short, long, G).is_err());
    assert!(StepConfig::new(1.0, 1.5, short, long, G).is_err());
    assert!(StepConfig::new(1.0, 0.0, Duration::from_millis(50), long, G).is_err());
    assert!(StepConfig::new(1.0, 0.0, long, short, G).is_err());
}

#[test]
fn weinberg_length_is_the_fourth_root_of_the_swing() {
    let step = Step {
        since_previous: None,
        peak: G + 4.0,
        trough: G - 4.0,
    };
    let length = weinberg_length(&step, 0.41).expect("a length").metres();
    assert!((length - 0.41 * 8f64.powf(0.25)).abs() < 1e-12);
    assert!(weinberg_length(&step, 3.0).is_err());
    let inverted = Step {
        peak: 1.0,
        trough: 2.0,
        ..step
    };
    assert!(weinberg_length(&inverted, 0.41).is_err());
}

proptest! {
    #[test]
    fn the_heading_is_the_yaw_whatever_the_tilt(
        roll in -60.0f64..60.0,
        pitch in -60.0f64..60.0,
        yaw in 0.0f64..360.0,
        dip in -75.0f64..75.0,
    ) {
        let attitude = Quaternion::from_euler(
            Angle::from_degrees(roll).expect("roll"),
            Angle::from_degrees(pitch).expect("pitch"),
            TrueCourse::from_degrees_wrapped(yaw),
        );
        // The field in the magnetic north-east-down frame, 50 µT with a dip.
        let d = dip.to_radians();
        let field = [50.0 * d.cos(), 0.0, 50.0 * d.sin()];
        let force = attitude.rotate_back([0.0, 0.0, -G]);
        let heading = tilt_compensated_heading(force, attitude.rotate_back(field)).expect("a heading");
        let off = (heading.degrees() - yaw + 540.0).rem_euclid(360.0) - 180.0;
        prop_assert!(off.abs() < 1e-8, "{} against {yaw}", heading.degrees());
    }
}

#[test]
fn a_heading_needs_gravity_and_a_horizontal_field() {
    assert!(tilt_compensated_heading([0.0; 3], [20.0, 0.0, 40.0]).is_err());
    assert!(tilt_compensated_heading([0.0, 0.0, -G], [0.0, 0.0, 50.0]).is_err());
    let east = tilt_compensated_heading([0.0, 0.0, -G], [0.0, -20.0, 40.0]).expect("a heading");
    assert!((east.degrees() - 90.0).abs() < 1e-12);
}
