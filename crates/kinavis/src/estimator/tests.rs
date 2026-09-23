//! Estimator tests.

#![allow(
    clippy::unwrap_used,
    clippy::indexing_slicing,
    clippy::cast_possible_wrap
)]

use core::time::Duration;

use super::*;
use crate::error::{KernelError, NavigationError};
use crate::estimation::GatingPolicy;
use crate::event::{NavigationIntegrity, SensorHealth, SensorId};
use crate::gnss::{Dop, GnssFix};
use crate::observations::{
    HeadingObservation, PositionObservation, SpeedThroughWaterObservation, VelocityObservation,
};
use crate::sailings::rhumb_destination;
use crate::units::{Angle, Speed};
use crate::{Position, TrueCourse};

fn at(seconds: i64) -> Instant<Utc> {
    Instant::from_unix_seconds(1_700_000_000 + seconds)
}

fn start() -> Position {
    "50°00.0'N 001°00.0'W".parse().unwrap()
}

/// Position after `seconds` at `knots` on `course` from the start, by spherical
/// rhumb line; differs from the model's flat-Earth-on-WGS-84 integration by
/// ~0.1 % (sphere vs ellipsoid minute of arc).
fn truth(seconds: i64, course: f64, knots: f64) -> Position {
    let hours = crate::units::hours(Duration::from_secs(seconds.unsigned_abs()));
    let distance = Distance::from_nautical_miles(knots * hours).unwrap();
    rhumb_destination(start(), TrueCourse::new(course).unwrap(), distance).unwrap()
}

fn estimator(course: f64, knots: f64) -> Estimator<SteadyMotion> {
    let fix = GnssFix::builder(at(0), start())
        .course_over_ground(TrueCourse::new(course).unwrap())
        .speed_over_ground(Speed::from_knots(knots).unwrap())
        .hdop(Dop::new(1.0).unwrap())
        .build();
    Estimator::new(
        NavigationState::initialised_from(&fix, None).unwrap(),
        SteadyMotion::standard(),
        EstimatorConfig::standard(),
    )
}

#[test]
fn prediction_moves_the_vessel_along_its_track_and_grows_the_ellipse() {
    let mut estimator = estimator(45.0, 12.0);
    let before = estimator.state().horizontal_error().semi_major().metres();
    let _ = estimator.advance_to(at(600)).unwrap();
    let expected = truth(600, 45.0, 12.0);
    let got = estimator.state().position();
    let error = crate::sailings::great_circle(expected, got)
        .unwrap()
        .distance;
    // 2 NM run: 0.1 % is under 4 m.
    assert!(error.metres() < 15.0, "{} m off", error.metres());
    assert!(estimator.state().horizontal_error().semi_major().metres() > before);
    assert_eq!(estimator.state().valid_at(), at(600));
}

#[test]
fn time_does_not_run_backwards() {
    let mut estimator = estimator(0.0, 10.0);
    let _ = estimator.advance_to(at(10)).unwrap();
    assert!(matches!(
        estimator.advance_to(at(5)),
        Err(NavigationError::Kernel(KernelError::TimeReversed { .. }))
    ));
    // An observation older than the late policy allows is a rejection, not an
    // error: 5 s behind with a 2 s limit.
    let old = PositionObservation::new(at(5), start(), Distance::from_metres(5.0).unwrap());
    let outcome = estimator.ingest(&old).unwrap();
    assert!(matches!(
        outcome.events()[0],
        NavigationEvent::ObservationRejected {
            reason: RejectionReason::OutOfOrder,
            ..
        }
    ));
    assert_eq!(estimator.state().valid_at(), at(10));
}

#[test]
fn a_position_observation_pulls_the_estimate_towards_it_and_tightens_it() {
    let mut estimator = estimator(90.0, 10.0);
    let _ = estimator.advance_to(at(60)).unwrap();
    let loose = estimator.state().horizontal_error().semi_major().metres();
    // Receiver puts the vessel 30 m north of the steady-motion prediction, σ 5
    // m.
    let predicted = estimator.state().position();
    let reported = rhumb_destination(
        predicted,
        TrueCourse::NORTH,
        Distance::from_metres(30.0).unwrap(),
    )
    .unwrap();
    let outcome = estimator
        .ingest(&PositionObservation::new(
            at(60),
            reported,
            Distance::from_metres(5.0).unwrap(),
        ))
        .unwrap();
    let report = outcome.report().unwrap();
    assert!(report.accepted());
    assert_eq!(report.degrees_of_freedom(), 2);
    assert!(report.normalised_innovation_squared() > 0.0);
    let after = crate::sailings::great_circle(reported, estimator.state().position())
        .unwrap()
        .distance
        .metres();
    assert!(after < 30.0, "moved only to {after} m from the report");
    assert!(estimator.state().horizontal_error().semi_major().metres() < loose);
    assert!(estimator.state().horizontal_error().semi_major().metres() <= 5.0);
}

#[test]
fn the_gate_refuses_a_wild_position_and_says_so() {
    let mut estimator = estimator(90.0, 10.0);
    // 2 km north of any reachable position, 1 s later.
    let wild = PositionObservation::new(
        at(1),
        rhumb_destination(
            start(),
            TrueCourse::NORTH,
            Distance::from_metres(2000.0).unwrap(),
        )
        .unwrap(),
        Distance::from_metres(5.0).unwrap(),
    );
    let outcome = estimator.ingest(&wild).unwrap();
    assert!(!outcome.report().unwrap().accepted());
    assert!(matches!(
        outcome.events()[0],
        NavigationEvent::ObservationRejected {
            reason: RejectionReason::Improbable { .. },
            ..
        }
    ));
    // Time advanced; position did not jump.
    assert_eq!(estimator.state().valid_at(), at(1));
    let moved = crate::sailings::great_circle(start(), estimator.state().position())
        .unwrap()
        .distance
        .metres();
    assert!(moved < 10.0, "{moved}");
    // Ungated, the observation is applied, and the correction implies a 13 m/s
    // current, which the state invariants reject: the last line of defence with
    // the gate off.
    let mut trusting = estimator.clone();
    assert!(matches!(
        trusting.ingest(&wild.with_gate(GatingPolicy::none())),
        Err(NavigationError::Kernel(KernelError::OutOfRange {
            parameter: "current",
            ..
        }))
    ));
}

#[test]
fn headings_are_compared_across_north() {
    let mut estimator = estimator(1.0, 10.0);
    let gyro = HeadingObservation::new(
        at(1),
        TrueCourse::new(359.0).unwrap(),
        Angle::from_degrees(0.5).unwrap(),
    );
    let outcome = estimator.ingest(&gyro).unwrap();
    let report = outcome.report().unwrap();
    assert!(report.accepted(), "{report:?}");
    // Innovation −2°, a fraction of σ², not 358°.
    assert!(report.normalised_innovation_squared() < 3.0);
    let heading = estimator.state().heading().degrees();
    assert!(heading > 359.0 || heading < 1.0, "{heading}");
}

#[test]
fn without_gnss_the_log_and_gyro_carry_the_position_and_the_current_holds() {
    // Vessel on 090° at 10 kn through the water, 2 kn northerly set. GNSS for
    // 10 min, then 10 min of gyro and log only.
    let set = Speed::from_knots(2.0).unwrap();
    let mut estimator = estimator(90.0, 10.0);
    let mut position = start();
    for second in 1..=1200_i64 {
        let through_water = rhumb_destination(
            position,
            TrueCourse::new(90.0).unwrap(),
            Distance::from_nautical_miles(10.0 / 3600.0).unwrap(),
        )
        .unwrap();
        position = rhumb_destination(
            through_water,
            TrueCourse::NORTH,
            Distance::from_nautical_miles(set.knots() / 3600.0).unwrap(),
        )
        .unwrap();
        let _ = estimator
            .ingest(&HeadingObservation::new(
                at(second),
                TrueCourse::new(90.0).unwrap(),
                Angle::from_degrees(0.5).unwrap(),
            ))
            .unwrap();
        let _ = estimator
            .ingest(&SpeedThroughWaterObservation::new(
                at(second),
                Speed::from_knots(10.0).unwrap(),
                Speed::from_knots(0.2).unwrap(),
            ))
            .unwrap();
        if second <= 600 {
            let _ = estimator
                .ingest(&PositionObservation::new(
                    at(second),
                    position,
                    Distance::from_metres(5.0).unwrap(),
                ))
                .unwrap();
            let _ = estimator
                .ingest(&VelocityObservation::from_track(
                    at(second),
                    crate::sailings::rhumb_line(start(), position)
                        .unwrap()
                        .initial_course,
                    Speed::from_knots(math::hypot(10.0, 2.0)).unwrap(),
                    Speed::from_knots(0.3).unwrap(),
                ))
                .unwrap();
        }
    }
    // The current was estimated while GNSS was available...
    let current = estimator.state().current();
    assert!(
        (current.north().knots() - 2.0).abs() < 0.3,
        "current north {} kn",
        current.north().knots()
    );
    assert!(
        current.east().knots().abs() < 0.3,
        "{}",
        current.east().knots()
    );
    // ...and carried the position through the outage.
    let error = crate::sailings::great_circle(position, estimator.state().position())
        .unwrap()
        .distance;
    assert!(
        error.metres() < 150.0,
        "{} m off after ten minutes of DR",
        error.metres()
    );
    // Uncertainty grew while unaided.
    assert!(estimator.state().horizontal_error().semi_major().metres() > 20.0);
    let snapshot = estimator.snapshot_at(at(1200));
    assert!(!snapshot.is_stale());
    assert!(snapshot.heading().is_some());
}

#[test]
fn the_anchor_follows_a_vessel_that_goes_far() {
    let mut estimator = estimator(0.0, 20.0);
    let _ = estimator.advance_to(at(3 * 3600)).unwrap();
    // 60 NM north, past the 50 km limit: re-anchored, northing small again.
    let north = estimator
        .state()
        .vector()
        .element(StateComponent::North.index())
        .unwrap();
    assert!(north.abs() < 1.0, "{north}");
    let expected = truth(3 * 3600, 0.0, 20.0);
    let error = crate::sailings::great_circle(expected, estimator.state().position())
        .unwrap()
        .distance;
    // 60 NM run: 0.1 % is 100 m.
    assert!(error.metres() < 150.0, "{} m", error.metres());
}

fn at_millis(millis: i64) -> Instant<Utc> {
    at(0).saturating_add(Duration::from_millis(millis.unsigned_abs()))
}

fn gyro(when: Instant<Utc>, degrees: f64) -> HeadingObservation {
    HeadingObservation::new(
        when,
        TrueCourse::wrap(degrees).unwrap(),
        Angle::from_degrees(0.5).unwrap(),
    )
}

fn log(when: Instant<Utc>, knots: f64) -> SpeedThroughWaterObservation {
    SpeedThroughWaterObservation::new(
        when,
        Speed::from_knots(knots).unwrap(),
        Speed::from_knots(0.2).unwrap(),
    )
}

fn fix(when: Instant<Utc>, position: Position) -> PositionObservation {
    PositionObservation::new(when, position, Distance::from_metres(5.0).unwrap())
}

fn metres_apart(a: Position, b: Position) -> f64 {
    crate::sailings::great_circle(a, b)
        .unwrap()
        .distance
        .metres()
}

fn element(state: &NavigationState, component: StateComponent) -> f64 {
    state.vector().element(component.index()).unwrap()
}

/// Vessel heading just west of north (smoother heading differences cross
/// north), gyro and log each second, fix at 2.4 s. One estimator takes the fix
/// in order; the other after the third second's gyro and log, 0.6 s late.
fn in_order_and_late(offset: Option<f64>) -> (Estimator<SteadyMotion>, Estimator<SteadyMotion>) {
    let mut in_order = estimator(359.9, 10.0);
    let mut late = estimator(359.9, 10.0);
    let headings = [0.1, 359.8, 0.2, 359.9];
    // Fix position: `offset` m north of the estimate, or exactly on it.
    let mut probe = in_order.clone();
    let _ = probe.ingest(&gyro(at(1), headings[0])).unwrap();
    let _ = probe.ingest(&log(at(1), 10.0)).unwrap();
    let _ = probe.ingest(&gyro(at(2), headings[1])).unwrap();
    let _ = probe.ingest(&log(at(2), 10.0)).unwrap();
    let expected = probe.predicted_at(at_millis(2400)).unwrap().position();
    let reported = match offset {
        Some(metres) => rhumb_destination(
            expected,
            TrueCourse::NORTH,
            Distance::from_metres(metres).unwrap(),
        )
        .unwrap(),
        None => expected,
    };
    let observed = fix(at_millis(2400), reported);
    for (second, heading) in (1..=4_i64).zip(headings) {
        if second == 3 {
            let outcome = in_order.ingest(&observed).unwrap();
            assert!(outcome.report().unwrap().accepted());
        }
        for estimator in [&mut in_order, &mut late] {
            let _ = estimator.ingest(&gyro(at(second), heading)).unwrap();
            let _ = estimator.ingest(&log(at(second), 10.0)).unwrap();
        }
        if second == 3 {
            let outcome = late.ingest(&observed).unwrap();
            let report = outcome.report().unwrap();
            assert!(report.accepted(), "{report:?}");
            assert!(report.fixes_position());
            // First fix: integrity becomes nominal as of the belief it was
            // folded into.
            assert_eq!(
                outcome.events().as_slice(),
                [NavigationEvent::IntegrityChanged {
                    from: NavigationIntegrity::DeadReckoning,
                    to: NavigationIntegrity::Nominal,
                    at: at(3),
                }]
            );
        }
    }
    (in_order, late)
}

#[test]
fn a_late_fix_is_folded_in_as_if_it_had_arrived_in_time() {
    // Fix exactly on the estimate: nothing to correct, both orders agree to
    // rounding — smoother and step-wise process noise are exact.
    let (in_order, late) = in_order_and_late(None);
    assert_eq!(in_order.state().valid_at(), late.state().valid_at());
    for component in StateComponent::ALL {
        let a = element(in_order.state(), component);
        let b = element(late.state(), component);
        assert!((a - b).abs() < 1e-9, "{component:?}: {a} against {b}");
    }
    let apart = (*in_order.state().covariance() - *late.state().covariance()).max_abs();
    assert!(apart < 1e-9, "covariances {apart} apart");

    // Fix 8 m off: orders differ only through linearisation, millimetres here.
    let (in_order, late) = in_order_and_late(Some(8.0));
    let apart = metres_apart(in_order.state().position(), late.state().position());
    assert!(apart < 0.01, "{apart} m between the two orders");
    for component in StateComponent::ALL {
        let a = element(in_order.state(), component);
        let b = element(late.state(), component);
        assert!((a - b).abs() < 1e-3, "{component:?}: {a} against {b}");
    }
    let apart = (*in_order.state().covariance() - *late.state().covariance()).max_abs();
    assert!(apart < 1e-3, "covariances {apart} apart");
    // Without the fix, the estimate follows steady motion, 8 m south of the
    // reported position.
    let mut without = estimator(359.9, 10.0);
    for (second, heading) in (1..=4_i64).zip([0.1, 359.8, 0.2, 359.9]) {
        let _ = without.ingest(&gyro(at(second), heading)).unwrap();
        let _ = without.ingest(&log(at(second), 10.0)).unwrap();
    }
    let pulled = metres_apart(without.state().position(), late.state().position());
    assert!(
        pulled > 3.0,
        "the late fix moved the estimate only {pulled} m"
    );
}

#[test]
fn the_noise_of_a_step_is_the_noise_of_its_halves() {
    // Q(a + b) = F(b) Q(a) F(b)ᵀ + Q(b), required by the smoother.
    let model = SteadyMotion::standard();
    let state = *estimator(37.0, 12.0).state();
    let (a, b) = (Duration::from_millis(400), Duration::from_millis(600));
    let whole = *model.noise(&state, a + b).matrix();
    let first = *model.noise(&state, a).matrix();
    let carried = *model.jacobian(&state, b).unwrap().matrix();
    let second = *model.noise(&state, b).matrix();
    let halves = carried * first * carried.transpose() + second;
    let apart = (whole - halves).max_abs();
    assert!(apart < 1e-12, "{apart}");
}

#[test]
fn a_late_fix_at_a_remembered_moment_needs_no_carrying_forward() {
    let mut estimator = estimator(90.0, 10.0);
    let _ = estimator.ingest(&gyro(at(1), 90.0)).unwrap();
    let _ = estimator.ingest(&gyro(at(2), 90.0)).unwrap();
    // Fix stamped at 1 s, arriving after 2 s.
    let outcome = estimator.ingest(&fix(at(1), truth(1, 90.0, 10.0))).unwrap();
    assert!(outcome.report().unwrap().accepted());
    assert_eq!(estimator.state().valid_at(), at(2));
    let error = metres_apart(estimator.state().position(), truth(2, 90.0, 10.0));
    assert!(error < 5.0, "{error} m");
}

#[test]
fn the_policy_decides_what_becomes_of_a_late_fix() {
    let config = EstimatorConfig::standard().with_late(LatePolicy::Reject);
    let mut folding = estimator(90.0, 10.0);
    let mut rejecting = Estimator::new(*folding.state(), SteadyMotion::standard(), config);
    let _ = rejecting.ingest(&gyro(at(1), 90.0)).unwrap();
    let _ = folding.ingest(&gyro(at(1), 90.0)).unwrap();
    let late = fix(at_millis(500), truth(0, 90.0, 10.0));
    let outcome = rejecting.ingest(&late).unwrap();
    assert!(outcome.report().is_none());
    assert!(matches!(
        outcome.events()[0],
        NavigationEvent::ObservationRejected {
            reason: RejectionReason::OutOfOrder,
            ..
        }
    ));
    let outcome = folding.ingest(&late).unwrap();
    assert!(outcome.report().unwrap().accepted());
    // Beyond the lag limit or the history, the folding estimator rejects it
    // too.
    for _ in 0..MAX_HISTORY {
        let next = folding
            .state()
            .valid_at()
            .saturating_add(Duration::from_millis(100));
        let _ = folding.advance_to(next).unwrap();
    }
    let outcome = folding.ingest(&late).unwrap();
    assert!(matches!(
        outcome.events()[0],
        NavigationEvent::ObservationRejected {
            reason: RejectionReason::OutOfOrder,
            ..
        }
    ));
}

#[test]
fn a_receiver_that_is_always_late_still_holds_the_position() {
    // Gyro and log on each second; the previous second's fix arrives 0.4 s
    // later. Under the rejecting policy the only absolute source never gets in,
    // leaving dead reckoning in an unobserved current.
    let config = EstimatorConfig::standard().with_late(LatePolicy::Reject);
    let mut folding = estimator(90.0, 10.0);
    let mut rejecting = Estimator::new(*folding.state(), SteadyMotion::standard(), config);
    let mut position = start();
    for second in 1..=120_i64 {
        let previous = position;
        // 10 kn east through the water, 2 kn northerly set.
        position = rhumb_destination(
            rhumb_destination(
                position,
                TrueCourse::new(90.0).unwrap(),
                Distance::from_nautical_miles(10.0 / 3600.0).unwrap(),
            )
            .unwrap(),
            TrueCourse::NORTH,
            Distance::from_nautical_miles(2.0 / 3600.0).unwrap(),
        )
        .unwrap();
        for estimator in [&mut rejecting, &mut folding] {
            let _ = estimator.ingest(&gyro(at(second), 90.0)).unwrap();
            let _ = estimator.ingest(&log(at(second), 10.0)).unwrap();
        }
        let late = fix(at_millis(second * 1000 - 400), previous);
        let _ = rejecting.ingest(&late).unwrap();
        let outcome = folding.ingest(&late).unwrap();
        assert!(outcome.report().unwrap().accepted(), "second {second}");
    }
    let held = metres_apart(folding.state().position(), position);
    let lost = metres_apart(rejecting.state().position(), position);
    assert!(held < 15.0, "folding: {held} m off");
    assert!(lost > 100.0, "rejecting: only {lost} m off");
    assert_eq!(folding.integrity(), NavigationIntegrity::Nominal);
    assert_eq!(rejecting.integrity(), NavigationIntegrity::DeadReckoning);
    // The current was estimated from late fixes alone.
    let set = folding.state().current().north().knots();
    assert!((set - 2.0).abs() < 0.3, "set {set} kn");
}

#[test]
fn integrity_follows_the_fixes_and_the_ellipse() {
    let mut estimator = estimator(90.0, 10.0);
    assert_eq!(estimator.integrity(), NavigationIntegrity::DeadReckoning);
    let outcome = estimator.ingest(&fix(at(1), truth(1, 90.0, 10.0))).unwrap();
    assert!(outcome
        .events()
        .contains(&NavigationEvent::IntegrityChanged {
            from: NavigationIntegrity::DeadReckoning,
            to: NavigationIntegrity::Nominal,
            at: at(1),
        }));
    // A heading does not fix position, so it does not count.
    let _ = estimator.ingest(&gyro(at(20), 90.0)).unwrap();
    assert_eq!(estimator.integrity(), NavigationIntegrity::Nominal);
    // The snapshot evaluates the requested instant; the estimator, its last
    // step.
    assert_eq!(
        estimator.snapshot_at(at(40)).integrity(),
        Some(NavigationIntegrity::DeadReckoning)
    );
    assert_eq!(estimator.integrity(), NavigationIntegrity::Nominal);
    let outcome = estimator.advance_to(at(40)).unwrap();
    assert!(outcome
        .events()
        .contains(&NavigationEvent::IntegrityChanged {
            from: NavigationIntegrity::Nominal,
            to: NavigationIntegrity::DeadReckoning,
            at: at(40),
        }));
    // Unaided, the ellipse grows past the limit.
    let outcome = estimator.advance_to(at(1800)).unwrap();
    assert!(outcome
        .events()
        .contains(&NavigationEvent::IntegrityChanged {
            from: NavigationIntegrity::DeadReckoning,
            to: NavigationIntegrity::Exceeded,
            at: at(1800),
        }));
    assert!(estimator.state().horizontal_error().semi_major().metres() > 100.0);
    // One fix brings it back.
    let outcome = estimator
        .ingest(&fix(
            at(1801),
            estimator.predicted_at(at(1801)).unwrap().position(),
        ))
        .unwrap();
    assert!(outcome.report().unwrap().accepted());
    assert!(outcome
        .events()
        .contains(&NavigationEvent::IntegrityChanged {
            from: NavigationIntegrity::Exceeded,
            to: NavigationIntegrity::Nominal,
            at: at(1801),
        }));
}

#[test]
fn a_source_turned_away_again_and_again_comes_under_suspicion() {
    let mut estimator = estimator(90.0, 10.0);
    assert_eq!(estimator.sensor_health(SensorId::named("position")), None);
    let wild = rhumb_destination(
        start(),
        TrueCourse::NORTH,
        Distance::from_metres(2000.0).unwrap(),
    )
    .unwrap();
    for second in 1..=4_i64 {
        let outcome = estimator.ingest(&fix(at(second), wild)).unwrap();
        assert!(!outcome.report().unwrap().accepted());
        assert_eq!(outcome.events().len(), 1, "{:?}", outcome.events());
        assert_eq!(
            estimator.sensor_health(SensorId::named("position")),
            Some(SensorHealth::Healthy)
        );
    }
    let outcome = estimator.ingest(&fix(at(5), wild)).unwrap();
    assert!(outcome
        .events()
        .contains(&NavigationEvent::SensorHealthChanged {
            sensor: SensorId::named("position"),
            from: SensorHealth::Healthy,
            to: SensorHealth::Suspect,
            at: at(5),
        }));
    assert_eq!(
        estimator.sensor_health(SensorId::named("position")),
        Some(SensorHealth::Suspect)
    );
    // The same number of consecutive acceptances clears it; one does not, so a
    // source rejected half the time is not trusted half the time.
    for second in 6..=9_i64 {
        let outcome = estimator
            .ingest(&fix(at(second), truth(second, 90.0, 10.0)))
            .unwrap();
        assert!(outcome.report().unwrap().accepted());
        assert_eq!(
            estimator.sensor_health(SensorId::named("position")),
            Some(SensorHealth::Suspect)
        );
    }
    let outcome = estimator
        .ingest(&fix(at(10), truth(10, 90.0, 10.0)))
        .unwrap();
    assert!(outcome
        .events()
        .contains(&NavigationEvent::SensorHealthChanged {
            sensor: SensorId::named("position"),
            from: SensorHealth::Suspect,
            to: SensorHealth::Healthy,
            at: at(10),
        }));
    // The count restarts: one rejection does not make it suspect again.
    let outcome = estimator.ingest(&fix(at(11), wild)).unwrap();
    assert_eq!(outcome.events().len(), 1, "{:?}", outcome.events());
    assert_eq!(
        estimator.sensor_health(SensorId::named("position")),
        Some(SensorHealth::Healthy)
    );
    // The gyro was never suspect.
    let _ = estimator.ingest(&gyro(at(12), 90.0)).unwrap();
    assert_eq!(
        estimator.sensor_health(SensorId::named("heading")),
        Some(SensorHealth::Healthy)
    );
}

#[test]
fn two_receivers_named_apart_are_judged_apart() {
    let mut estimator = estimator(90.0, 10.0);
    let (one, two) = (SensorId::named("GNSS 1"), SensorId::named("GNSS 2"));
    let wild = rhumb_destination(
        start(),
        TrueCourse::NORTH,
        Distance::from_metres(2000.0).unwrap(),
    )
    .unwrap();
    // First receiver wrong five times; second correct.
    for second in 1..=5_i64 {
        let outcome = estimator
            .ingest(&fix(at(second), wild).from_sensor(one))
            .unwrap();
        assert_eq!(outcome.report().unwrap().sensor(), one);
        let _ = estimator
            .ingest(&fix(at(second), truth(second, 90.0, 10.0)).from_sensor(two))
            .unwrap();
    }
    assert_eq!(estimator.sensor_health(one), Some(SensorHealth::Suspect));
    assert_eq!(estimator.sensor_health(two), Some(SensorHealth::Healthy));
    // The generic kind was never heard from: both fixes named their receiver.
    assert_eq!(estimator.sensor_health(SensorId::named("position")), None);
}

#[test]
fn more_sources_than_the_estimator_can_watch_are_refused_not_lost() {
    const NAMES: [&str; MAX_SENSORS + 1] = [
        "gyro 1", "gyro 2", "gyro 3", "gyro 4", "gyro 5", "gyro 6", "gyro 7", "gyro 8", "gyro 9",
    ];
    let mut estimator = estimator(90.0, 10.0);
    for (second, name) in (1..).zip(NAMES.iter().take(MAX_SENSORS)) {
        let _ = estimator
            .ingest(&gyro(at(second), 90.0).from_sensor(SensorId::named(name)))
            .unwrap();
    }
    let ninth = gyro(at(20), 90.0).from_sensor(SensorId::named(NAMES[MAX_SENSORS]));
    assert!(matches!(
        estimator.ingest(&ninth),
        Err(NavigationError::Kernel(KernelError::CapacityExceeded {
            context: "the estimator's sources",
            ..
        }))
    ));
    // Belief propagated to the observation time; correction not applied.
    assert_eq!(estimator.state().valid_at(), at(20));
}

#[test]
fn the_history_keeps_one_belief_per_moment_and_lets_the_oldest_go() {
    const HELD: i64 = MAX_HISTORY as i64;
    let mut estimator = estimator(90.0, 10.0);
    for second in 1..=(HELD + 4) {
        let _ = estimator.ingest(&gyro(at(second), 90.0)).unwrap();
        let _ = estimator.ingest(&log(at(second), 10.0)).unwrap();
    }
    // The last MAX_HISTORY instants are kept: a fix at the oldest is folded in,
    // one earlier is not.
    let oldest = HELD + 4 - (HELD - 1);
    let config = EstimatorConfig::standard().with_late(LatePolicy::Smooth {
        max_lag: Duration::from_secs(3600),
    });
    let mut generous = Estimator::new(*estimator.state(), SteadyMotion::standard(), config);
    generous.history = estimator.history.clone();
    let outcome = generous
        .ingest(&fix(at(oldest), truth(oldest, 90.0, 10.0)))
        .unwrap();
    assert!(outcome.report().unwrap().accepted());
    let outcome = generous
        .ingest(&fix(at(oldest - 1), truth(oldest - 1, 90.0, 10.0)))
        .unwrap();
    assert!(matches!(
        outcome.events()[0],
        NavigationEvent::ObservationRejected {
            reason: RejectionReason::OutOfOrder,
            ..
        }
    ));
}

#[test]
fn moving_the_anchor_starts_the_history_afresh() {
    // 20 kn north: 50 km in 4860 s.
    let config = EstimatorConfig::standard().with_late(LatePolicy::Smooth {
        max_lag: Duration::from_secs(3600),
    });
    let initial = *estimator(0.0, 20.0).state();
    let mut stayed = Estimator::new(initial, SteadyMotion::standard(), config);
    let mut moved = stayed.clone();
    let _ = stayed.advance_to(at(4000)).unwrap();
    let _ = moved.advance_to(at(4000)).unwrap();
    let _ = stayed.advance_to(at(4500)).unwrap();
    let _ = moved.advance_to(at(5000)).unwrap();
    // The estimator still in its first frame has the 4000 s belief and folds in
    // a fix stamped then; the one re-anchored on the last step does not (that
    // belief was in the old frame).
    let late = fix(at(4000), truth(4000, 0.0, 20.0));
    assert!(stayed.ingest(&late).unwrap().report().unwrap().accepted());
    let outcome = moved.ingest(&late).unwrap();
    assert!(matches!(
        outcome.events()[0],
        NavigationEvent::ObservationRejected {
            reason: RejectionReason::OutOfOrder,
            ..
        }
    ));
}

#[test]
fn the_pure_late_step_refuses_a_history_that_does_not_reach() {
    let estimator = estimator(90.0, 10.0);
    let history = [*estimator.state()];
    let observation = fix(at(1), start());
    assert!(matches!(
        pure::update_late(&history, estimator.process(), &observation),
        Err(NavigationError::Kernel(KernelError::Missing { .. }))
    ));
    assert!(matches!(
        pure::update_late(&[], estimator.process(), &observation),
        Err(NavigationError::Kernel(KernelError::Missing { .. }))
    ));
}

#[test]
fn the_configuration_is_checked_as_it_is_built() {
    let standard = EstimatorConfig::standard();
    assert!((standard.reanchor_after().kilometres() - 50.0).abs() < 1e-9);
    assert!((standard.alert_limit().metres() - 100.0).abs() < 1e-9);
    assert_eq!(standard.max_age(), Duration::from_secs(10));
    assert_eq!(standard.max_unaided(), Duration::from_secs(30));
    assert_eq!(standard.suspect_after(), 5);
    assert!(
        matches!(standard.late(), LatePolicy::Smooth { max_lag } if max_lag == Duration::from_secs(2))
    );

    let tuned = standard
        .with_reanchor_after(Distance::from_kilometres(20.0).unwrap())
        .unwrap()
        .with_alert_limit(Distance::from_metres(30.0).unwrap())
        .unwrap()
        .with_max_age(Duration::from_secs(5))
        .with_max_unaided(Duration::from_secs(60))
        .with_late(LatePolicy::Reject)
        .with_suspect_after(0);
    assert!((tuned.reanchor_after().kilometres() - 20.0).abs() < 1e-9);
    assert!((tuned.alert_limit().metres() - 30.0).abs() < 1e-9);
    assert_eq!(tuned.max_age(), Duration::from_secs(5));
    assert_eq!(tuned.max_unaided(), Duration::from_secs(60));
    assert_eq!(tuned.late(), LatePolicy::Reject);
    assert_eq!(tuned.suspect_after(), 0);

    // Zero would re-anchor every step or flag every position; negative is
    // invalid.
    assert!(matches!(
        standard.with_reanchor_after(Distance::ZERO).unwrap_err(),
        NavigationError::Kernel(KernelError::OutOfRange {
            parameter: "reanchor distance",
            ..
        })
    ));
    assert!(standard
        .with_reanchor_after(Distance::from_metres(-1.0).unwrap())
        .is_err());
    assert!(matches!(
        standard.with_alert_limit(Distance::ZERO).unwrap_err(),
        NavigationError::Kernel(KernelError::OutOfRange {
            parameter: "alert limit",
            ..
        })
    ));
    assert!(standard
        .with_alert_limit(Distance::from_nautical_miles_unchecked(f64::NAN))
        .is_err());
}
