//! Closest approach and loss of separation in three dimensions, against the
//! plane CPA, against sampling in time, and under exchange and rotation.

#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::panic
)]

use core::time::Duration;

use kinavis::local::{Kinematics3, Ned, Vector3};
use kinavis::relative_motion::{
    closest_approach_3d, closest_point_of_approach, separation_loss, Approach, Approach3, Contact,
    SeparationLoss, SeparationVolume, Vessel,
};
use kinavis::{Distance, Speed, TrueBearing, TrueCourse};
use proptest::prelude::*;

fn metres(value: f64) -> Distance {
    Distance::from_metres(value).expect("finite")
}

fn mps(value: f64) -> Speed {
    Speed::from_metres_per_second(value).expect("finite")
}

fn state(position: [f64; 3], velocity: [f64; 3]) -> Kinematics3<Ned> {
    let [n, e, d] = position;
    let [vn, ve, vd] = velocity;
    Kinematics3::new(
        Vector3::new(metres(n), metres(e), metres(d)),
        Vector3::new(mps(vn), mps(ve), mps(vd)),
    )
}

fn any_state() -> impl Strategy<Value = Kinematics3<Ned>> {
    (
        -5_000.0_f64..=5_000.0,
        -5_000.0_f64..=5_000.0,
        -500.0_f64..=500.0,
        -40.0_f64..=40.0,
        -40.0_f64..=40.0,
        -8.0_f64..=8.0,
    )
        .prop_map(|(n, e, d, vn, ve, vd)| state([n, e, d], [vn, ve, vd]))
}

/// Distance between the two at `seconds` from now.
fn distance_at(own: Kinematics3<Ned>, other: Kinematics3<Ned>, seconds: f64) -> f64 {
    let elapsed = Duration::from_secs_f64(seconds);
    (other.after(elapsed).position() - own.after(elapsed).position())
        .magnitude()
        .metres()
}

/// Same state turned by `angle` about the down axis.
fn rotated(state: Kinematics3<Ned>, angle: f64) -> Kinematics3<Ned> {
    let (sin, cos) = angle.sin_cos();
    let turn = |[n, e, d]: [f64; 3]| [n * cos - e * sin, n * sin + e * cos, d];
    self::state(
        turn(state.position().components().map(Distance::metres)),
        turn(state.velocity().components().map(Speed::metres_per_second)),
    )
}

proptest! {
    #[test]
    fn level_motion_gives_the_plane_cpa(
        own_course in 0.0_f64..360.0,
        own_knots in 0.0_f64..=30.0,
        target_course in 0.0_f64..360.0,
        target_knots in 0.0_f64..=30.0,
        bearing in 0.0_f64..360.0,
        range_nm in 0.0_f64..=12.0,
    ) {
        let own = Vessel { course: TrueCourse::new(own_course).unwrap(), speed: Speed::from_knots(own_knots).unwrap() };
        let target = Vessel { course: TrueCourse::new(target_course).unwrap(), speed: Speed::from_knots(target_knots).unwrap() };
        let contact = Contact { bearing: TrueBearing::new(bearing).unwrap(), range: Distance::from_nautical_miles(range_nm).unwrap() };
        let to_ned = |course: TrueCourse, magnitude: f64| {
            let (north, east) = course.components(magnitude);
            [north, east, 0.0]
        };
        let own3 = state([0.0; 3], to_ned(own.course, own.speed.metres_per_second()));
        let target3 = state(
            to_ned(contact.bearing, contact.range.metres()),
            to_ned(target.course, target.speed.metres_per_second()),
        );
        match (closest_point_of_approach(own, contact, target).unwrap(), closest_approach_3d(own3, target3).unwrap()) {
            (Approach::Closing(plane), Approach3::Closing(space)) => {
                prop_assert!((plane.distance.nautical_miles() - space.distance.nautical_miles()).abs() < 1e-9);
                prop_assert!((plane.time_to_go.as_secs_f64() - space.time_to_go.as_secs_f64()).abs() < 1e-3);
                prop_assert!(space.vertical.metres().abs() < 1e-9);
            }
            (Approach::Opening { .. }, Approach3::Opening { .. })
            | (Approach::Stationary { .. }, Approach3::Stationary { .. }) => {}
            (plane, space) => prop_assert!(
                // Exactly abeam: the two may split a zero closing rate
                // differently.
                range_nm < 1e-6 || matches!(plane, Approach::Closing(cpa) if cpa.time_to_go < Duration::from_millis(1)),
                "plane {plane:?}, space {space:?}"
            ),
        }
    }

    #[test]
    fn no_sampled_moment_comes_closer_than_the_cpa(own in any_state(), other in any_state()) {
        let Approach3::Closing(cpa) = closest_approach_3d(own, other).unwrap() else {
            return Ok(());
        };
        let tcpa = cpa.time_to_go.as_secs_f64();
        prop_assert!((distance_at(own, other, tcpa) - cpa.distance.metres()).abs() < 1e-6);
        let horizon = 2.0 * tcpa + 60.0;
        for step in 0..=2_000 {
            let seconds = horizon * f64::from(step) / 2_000.0;
            prop_assert!(distance_at(own, other, seconds) >= cpa.distance.metres() - 1e-6);
        }
        prop_assert!(
            (cpa.horizontal.metres().hypot(cpa.vertical.metres()) - cpa.distance.metres()).abs() < 1e-6
        );
    }

    #[test]
    fn exchanging_the_two_mirrors_the_cpa(own in any_state(), other in any_state()) {
        match (closest_approach_3d(own, other).unwrap(), closest_approach_3d(other, own).unwrap()) {
            (Approach3::Closing(ab), Approach3::Closing(ba)) => {
                prop_assert!((ab.distance.metres() - ba.distance.metres()).abs() < 1e-6);
                prop_assert_eq!(ab.time_to_go, ba.time_to_go);
                prop_assert!((ab.vertical.metres() + ba.vertical.metres()).abs() < 1e-6);
            }
            (Approach3::Opening { current_range: a }, Approach3::Opening { current_range: b })
            | (Approach3::Stationary { range: a }, Approach3::Stationary { range: b }) => {
                prop_assert!((a.metres() - b.metres()).abs() < 1e-6);
            }
            (ab, ba) => prop_assert!(false, "{ab:?} against {ba:?}"),
        }
    }

    #[test]
    fn turning_the_frame_changes_only_the_bearing(
        own in any_state(),
        other in any_state(),
        angle in 0.0_f64..core::f64::consts::TAU,
    ) {
        let plain = closest_approach_3d(own, other).unwrap();
        let turned = closest_approach_3d(rotated(own, angle), rotated(other, angle)).unwrap();
        if let (Approach3::Closing(a), Approach3::Closing(b)) = (plain, turned) {
            prop_assert!((a.distance.metres() - b.distance.metres()).abs() < 1e-6);
            prop_assert!((a.time_to_go.as_secs_f64() - b.time_to_go.as_secs_f64()).abs() < 1e-6);
            prop_assert!((a.vertical.metres() - b.vertical.metres()).abs() < 1e-6);
        }
    }

    #[test]
    fn the_loss_interval_is_where_sampling_finds_the_object_inside(
        own in any_state(),
        other in any_state(),
        radius in 0.0_f64..=2_000.0,
        half_height in 0.0_f64..=300.0,
    ) {
        let volume = SeparationVolume::new(metres(radius), metres(half_height)).unwrap();
        let loss = separation_loss(own, other, volume).unwrap();
        let inside = |seconds: f64| {
            let elapsed = Duration::from_secs_f64(seconds);
            let relative = other.after(elapsed).position() - own.after(elapsed).position();
            (
                relative.horizontal_magnitude().metres() - radius,
                relative.down().metres().abs() - half_height,
            )
        };
        let horizon = 3_600.0;
        for step in 0..=3_600 {
            let seconds = horizon * f64::from(step) / 3_600.0;
            let (horizontal_excess, vertical_excess) = inside(seconds);
            let margin = horizontal_excess.max(vertical_excess);
            // Within 1 cm of the surface either answer is acceptable.
            if margin.abs() < 1e-2 {
                continue;
            }
            let expected = margin < 0.0;
            let reported = match loss {
                SeparationLoss::Clear => false,
                SeparationLoss::Loss { begins, ends } => {
                    seconds >= begins.as_secs_f64() && ends.is_none_or(|ends| seconds <= ends.as_secs_f64())
                }
                _ => unreachable!("no other outcome"),
            };
            prop_assert_eq!(reported, expected, "at {} s: {:?}", seconds, loss);
        }
    }
}

#[test]
fn a_cylinder_grazed_at_its_rim_is_entered_and_left_at_once() {
    let own = state([0.0; 3], [0.0; 3]);
    // Passing due east at 100 m north: tangent to a 100 m radius.
    let other = state([100.0, -500.0, 0.0], [0.0, 10.0, 0.0]);
    let volume = SeparationVolume::new(metres(100.0), metres(50.0)).unwrap();
    assert_eq!(
        separation_loss(own, other, volume).unwrap(),
        SeparationLoss::Loss {
            begins: Duration::from_secs(50),
            ends: Some(Duration::from_secs(50)),
        }
    );
}

#[test]
fn entering_through_the_top_and_the_side_at_one_instant() {
    let own = state([0.0; 3], [0.0; 3]);
    // 300 m out and 150 m up, closing at 10 m/s and descending at 5 m/s:
    // reaches the 100 m radius and the 50 m half-height together at 20 s.
    let other = state([300.0, 0.0, -150.0], [-10.0, 0.0, 5.0]);
    let volume = SeparationVolume::new(metres(100.0), metres(50.0)).unwrap();
    let SeparationLoss::Loss { begins, ends } = separation_loss(own, other, volume).unwrap() else {
        panic!("the object enters");
    };
    assert!((begins.as_secs_f64() - 20.0).abs() < 1e-9);
    // Leaves through the bottom and the far side together at 40 s.
    assert!((ends.unwrap().as_secs_f64() - 40.0).abs() < 1e-9);
}

#[test]
fn an_object_holding_station_inside_never_leaves() {
    let own = state([0.0; 3], [5.0, 0.0, 0.0]);
    let other = state([20.0, 0.0, -10.0], [5.0, 0.0, 0.0]);
    let volume = SeparationVolume::new(metres(100.0), metres(50.0)).unwrap();
    assert_eq!(
        separation_loss(own, other, volume).unwrap(),
        SeparationLoss::Loss {
            begins: Duration::ZERO,
            ends: None,
        }
    );
    assert!(matches!(
        closest_approach_3d(own, other).unwrap(),
        Approach3::Stationary { .. }
    ));
    let above = state([20.0, 0.0, -100.0], [5.0, 0.0, 0.0]);
    assert_eq!(
        separation_loss(own, above, volume).unwrap(),
        SeparationLoss::Clear
    );
}

#[test]
fn a_negative_volume_is_refused() {
    assert!(SeparationVolume::new(metres(-1.0), metres(10.0)).is_err());
    assert!(SeparationVolume::new(metres(10.0), metres(-1.0)).is_err());
}

#[test]
fn directly_overhead_at_cpa_has_no_bearing() {
    let own = state([0.0; 3], [0.0; 3]);
    let other = state([500.0, 0.0, -100.0], [-10.0, 0.0, 0.0]);
    let Approach3::Closing(cpa) = closest_approach_3d(own, other).unwrap() else {
        panic!("closing");
    };
    assert_eq!(cpa.bearing, None);
    assert!((cpa.vertical.metres() - 100.0).abs() < 1e-9);
    assert!(cpa.horizontal.metres() < 1e-9);
}
