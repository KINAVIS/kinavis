//! The picture under hostile input.
//!
//! Hostile numbers, poles and antimeridian, out-of-order observations and
//! extreme times: no panic, and the invariants hold — a track always has a fix,
//! the picture never exceeds capacity, no event is dropped silently.

#![allow(clippy::unwrap_used, clippy::indexing_slicing)]

use core::time::Duration;
use kinavis::relative_motion::{Contact, Vessel};
use kinavis_kernel::event::PositionSource;
use kinavis_kernel::observation::{ObservationStatus, Observed, Quality};
use kinavis_kernel::snapshot::GroundTrack;
use kinavis_kernel::{
    Angle, Distance, Instant, NavigationSnapshot, Position, RateOfTurn, Speed, TargetId,
    TrueBearing, TrueCourse, Utc,
};
use kinavis_traffic::{
    assess, assess_traffic, avoid, avoid_all, CollisionRisk, CpaPolicy, ManoeuvreConstraints,
    PermittedSides, TargetObservation, TrackingPolicy, Traffic, TrafficEvent, MAX_TARGETS,
};

/// Values known to break floating-point code.
const HOSTILE: [f64; 10] = [
    f64::NAN,
    f64::INFINITY,
    f64::NEG_INFINITY,
    f64::MAX,
    f64::MIN_POSITIVE,
    -0.0,
    0.0,
    1e-300,
    1e300,
    -1.0,
];

/// Positions known to break spherical code.
const AWKWARD: [(f64, f64); 9] = [
    (0.0, 0.0),
    (90.0, 0.0),
    (-90.0, 0.0),
    (89.9999, 179.9999),
    (0.0, 180.0),
    (0.0, -180.0),
    (50.0, -1.0),
    (50.0001, -0.9999),
    (-33.9, 18.4),
];

fn policy() -> TrackingPolicy {
    TrackingPolicy::new(
        2,
        Duration::from_secs(30),
        Duration::from_secs(180),
        Speed::from_knots(100.0).unwrap(),
    )
    .unwrap()
}

#[test]
fn a_policy_from_hostile_numbers_is_refused_not_obeyed() {
    for &seconds in &HOSTILE {
        let Ok(duration) = Duration::try_from_secs_f64(seconds) else {
            continue;
        };
        for &knots in &HOSTILE {
            let Ok(speed) = Speed::from_knots(knots) else {
                continue;
            };
            for fixes in [0, 1, 3, u8::MAX] {
                if let Ok(policy) = TrackingPolicy::new(fixes, duration, duration, speed) {
                    assert!(fixes >= 1);
                    assert!(speed.knots() > 0.0);
                    assert!(Traffic::new(policy).is_empty());
                }
            }
        }
    }
}

#[test]
fn observations_anywhere_and_anywhen_never_panic() {
    let positions = AWKWARD
        .iter()
        .filter_map(|&(latitude, longitude)| Position::from_degrees(latitude, longitude).ok())
        .collect::<Vec<_>>();
    let moments = [
        Instant::<Utc>::from_unix_seconds(i64::MIN),
        Instant::from_unix_seconds(-1),
        Instant::from_unix_seconds(0),
        Instant::from_unix_seconds(1_789_000_000),
        Instant::from_unix_seconds(1_789_000_001),
        Instant::from_unix_seconds(i64::MAX),
    ];

    let mut traffic = Traffic::new(policy());
    for (index, &here) in positions.iter().enumerate() {
        for &when in &moments {
            for &knots in &HOSTILE {
                let target = TargetId::new(u32::try_from(index).unwrap());
                let mut observation = TargetObservation::new(target, here, when);
                if let Ok(speed) = Speed::from_knots(knots) {
                    observation = observation.with_ground_track(GroundTrack {
                        course_over_ground: TrueCourse::EAST,
                        speed_over_ground: speed,
                    });
                }
                let events = traffic.ingest(observation).unwrap();
                assert!(!events.overflowed());
                assert!(traffic.len() <= MAX_TARGETS);
            }
        }
    }

    for &now in &moments {
        let view = traffic.view(now);
        assert_eq!(view.len(), traffic.len());
        for seen in view.targets() {
            assert!(seen.position.latitude().degrees().abs() <= 90.0);
            if let Some(motion) = seen.motion {
                assert!(motion.speed_over_ground.knots().is_finite());
            }
        }
        for track in traffic.tracks() {
            assert!(track.fix_count() >= 1);
            let _ = track.position_at(now);
            let _ = track.fitted_motion();
        }
    }
    // At the end of time, everything not seen there is lost.
    let end = Instant::from_unix_seconds(i64::MAX);
    let lost = traffic.sweep(end);
    assert!(lost
        .iter()
        .all(|event| matches!(event, TrafficEvent::TargetLost { .. })));
    assert!(traffic
        .tracks()
        .iter()
        .all(|track| track.age(end) <= policy().lost_after()));
}

#[test]
fn a_picture_full_of_targets_stays_within_its_capacity() {
    let mut traffic = Traffic::new(policy());
    let when = Instant::from_unix_seconds(1_789_000_000);
    let here = Position::from_degrees(50.0, -1.0).unwrap();
    for target in 0..(2 * MAX_TARGETS) {
        let observation =
            TargetObservation::new(TargetId::new(u32::try_from(target).unwrap()), here, when);
        let outcome = traffic.ingest(observation);
        assert_eq!(outcome.is_ok(), target < MAX_TARGETS);
        assert!(traffic.len() <= MAX_TARGETS);
    }
    // The sweep list is sized to the picture: one loss per target, none
    // dropped.
    let events = traffic.sweep(when.checked_add(Duration::from_secs(1000)).unwrap());
    assert!(!events.overflowed());
    assert_eq!(events.len(), MAX_TARGETS);
    assert!(traffic.is_empty());
}

#[test]
fn an_assessment_from_hostile_numbers_never_panics() {
    let policy = CpaPolicy::new(
        Distance::from_nautical_miles(1.0).unwrap(),
        Duration::from_secs(1200),
    )
    .unwrap();
    let speeds = HOSTILE
        .iter()
        .filter_map(|&knots| Speed::from_knots(knots).ok())
        .chain([Speed::from_knots(10.0).unwrap()])
        .collect::<Vec<_>>();
    let ranges = HOSTILE
        .iter()
        .filter_map(|&miles| Distance::from_nautical_miles(miles).ok())
        .chain([Distance::from_nautical_miles(5.0).unwrap()])
        .collect::<Vec<_>>();
    for &own_speed in &speeds {
        for &target_speed in &speeds {
            for &range in &ranges {
                for course in [0.0, 45.0, 90.0, 180.0, 270.0, 359.999] {
                    let own = Vessel {
                        course: TrueCourse::new(course).unwrap(),
                        speed: own_speed,
                    };
                    let target = Vessel {
                        course: TrueCourse::new(360.0 - course).unwrap(),
                        speed: target_speed,
                    };
                    let contact = Contact {
                        bearing: TrueBearing::new(course).unwrap(),
                        range,
                    };
                    if let Ok(assessment) = assess(own, contact, target, &policy) {
                        assert!(assessment.bearing_drift().degrees_per_minute().is_finite());
                        assert!(assessment.relative_speed().knots().is_finite());
                        assert_eq!(
                            assessment.cpa().is_none(),
                            assessment.risk() == CollisionRisk::Opening
                        );
                    }
                }
            }
        }
    }
}

#[test]
fn a_picture_of_targets_anywhere_is_assessed_without_panic() {
    let when = Instant::from_unix_seconds(1_789_000_000);
    let positions = AWKWARD
        .iter()
        .filter_map(|&(latitude, longitude)| Position::from_degrees(latitude, longitude).ok())
        .collect::<Vec<_>>();
    let mut traffic = Traffic::new(policy());
    for (index, &there) in positions.iter().enumerate() {
        let observation =
            TargetObservation::new(TargetId::new(u32::try_from(index).unwrap()), there, when)
                .with_ground_track(GroundTrack {
                    course_over_ground: TrueCourse::EAST,
                    speed_over_ground: Speed::from_knots(12.0).unwrap(),
                });
        let _ = traffic.ingest(observation).unwrap();
    }
    let policy = CpaPolicy::new(
        Distance::from_nautical_miles(2.0).unwrap(),
        Duration::from_secs(1800),
    )
    .unwrap();
    for &here in &positions {
        for knots in [0.0, 12.0, 1e6] {
            let state = NavigationSnapshot::EMPTY
                .with_position(
                    Observed::new(
                        here,
                        when,
                        Quality::<Distance>::new(ObservationStatus::Valid),
                    ),
                    PositionSource::Gnss,
                )
                .with_ground_track(GroundTrack {
                    course_over_ground: TrueCourse::NORTH,
                    speed_over_ground: Speed::from_knots(knots).unwrap(),
                });
            if let Ok((picture, events)) = assess_traffic(&state, &traffic, &policy) {
                assert!(picture.len() <= traffic.len());
                assert_eq!(
                    events.len(),
                    picture.at_least(CollisionRisk::Dangerous).count().min(8)
                );
            }
        }
    }
}

/// Constraints from hostile numbers: invalid bounds are rejected at
/// construction, so the search only sees valid ones. Port forbidden inside 1
/// NM; rate of turn equals the speed where that is a valid rate.
fn hostile_constraints(
    range: Distance,
    least: Angle,
    most: Angle,
    own_speed: Speed,
) -> Option<ManoeuvreConstraints> {
    let sides = if range.nautical_miles() > 1.0 {
        PermittedSides::Either
    } else {
        PermittedSides::Starboard
    };
    let constraints = ManoeuvreConstraints::new(sides, least, most).ok()?;
    Some(
        RateOfTurn::from_degrees_per_minute(own_speed.knots())
            .ok()
            .and_then(|rate| constraints.with_rate_of_turn(rate).ok())
            .unwrap_or(constraints),
    )
}

#[test]
fn an_avoiding_manoeuvre_from_hostile_numbers_never_panics() {
    let speeds = HOSTILE
        .iter()
        .filter_map(|&knots| Speed::from_knots(knots).ok())
        .chain([Speed::from_knots(10.0).unwrap()])
        .collect::<Vec<_>>();
    let distances = HOSTILE
        .iter()
        .filter_map(|&miles| Distance::from_nautical_miles(miles).ok())
        .chain([Distance::from_nautical_miles(4.0).unwrap()])
        .collect::<Vec<_>>();
    let angles = HOSTILE
        .iter()
        .chain(&[30.0, 90.0, 180.0])
        .filter_map(|&degrees| Angle::from_degrees(degrees).ok())
        .collect::<Vec<_>>();
    for &own_speed in &speeds {
        for &target_speed in &speeds {
            for &range in &distances {
                for &desired in &distances {
                    for &least in &angles {
                        for &most in &angles {
                            let Some(constraints) =
                                hostile_constraints(range, least, most, own_speed)
                            else {
                                continue;
                            };
                            let own = Vessel {
                                course: TrueCourse::NORTH,
                                speed: own_speed,
                            };
                            let target = Vessel {
                                course: TrueCourse::new(200.0).unwrap(),
                                speed: target_speed,
                            };
                            let contact = Contact {
                                bearing: TrueBearing::new(20.0).unwrap(),
                                range,
                            };
                            if let Ok(manoeuvre) =
                                avoid(own, contact, target, desired, &constraints)
                            {
                                let alteration = manoeuvre.alteration().degrees();
                                assert!(alteration.abs() >= least.degrees() - 1e-9);
                                assert!(alteration.abs() <= most.degrees() + 1e-9);
                                assert!(manoeuvre.least_passing().nautical_miles() >= 0.0);
                            }
                        }
                    }
                }
            }
        }
    }

    // The picture with own ship in awkward places.
    let when = Instant::from_unix_seconds(1_789_000_000);
    let positions = AWKWARD
        .iter()
        .filter_map(|&(latitude, longitude)| Position::from_degrees(latitude, longitude).ok())
        .collect::<Vec<_>>();
    let mut traffic = Traffic::new(policy());
    for (index, &there) in positions.iter().enumerate() {
        let observation =
            TargetObservation::new(TargetId::new(u32::try_from(index).unwrap()), there, when)
                .with_ground_track(GroundTrack {
                    course_over_ground: TrueCourse::EAST,
                    speed_over_ground: Speed::from_knots(12.0).unwrap(),
                });
        let _ = traffic.ingest(observation).unwrap();
    }
    let constraints = ManoeuvreConstraints::new(
        PermittedSides::Either,
        Angle::from_degrees(10.0).unwrap(),
        Angle::from_degrees(120.0).unwrap(),
    )
    .unwrap();
    for &here in &positions {
        let state = NavigationSnapshot::EMPTY
            .with_position(
                Observed::new(
                    here,
                    when,
                    Quality::<Distance>::new(ObservationStatus::Valid),
                ),
                PositionSource::Gnss,
            )
            .with_ground_track(GroundTrack {
                course_over_ground: TrueCourse::NORTH,
                speed_over_ground: Speed::from_knots(12.0).unwrap(),
            });
        let _ = avoid_all(
            &state,
            &traffic,
            Distance::from_nautical_miles(1.0).unwrap(),
            &constraints,
        );
    }
}
