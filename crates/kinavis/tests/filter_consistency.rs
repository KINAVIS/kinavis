//! Monte Carlo consistency of the estimator (NEES, NIS).
//!
//! A filter reporting a tight ellipse around the wrong position is worse than
//! none. The check: simulate a vessel moving as the process model assumes, feed
//! sensor readings with the claimed noise, and compare the actual error with
//! the reported covariance — `NEES = (x − x̂)ᵀ P⁻¹ (x − x̂)`, χ²(6) for a
//! consistent filter — and the innovations with their predicted covariance
//! (NIS, χ² with the observation dof). Averaged over runs, both must fall
//! within χ² intervals.
//!
//! The straight-passage scenario proves consistency, in order and with a
//! permanently late receiver. The others violate the model or a sensor — turn,
//! outage, spoofed receiver, noisy or biased gyro — and check what matters
//! then: the error stays inside the reported ellipse, the right source becomes
//! suspect, integrity is reported correctly.
//!
//! Random numbers are local (xorshift + Box–Muller): reproducible from the
//! seed, no dependency.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::cast_precision_loss,
    clippy::cast_possible_wrap,
    clippy::cast_sign_loss,
    clippy::cast_possible_truncation,
    clippy::too_many_lines,
    clippy::struct_excessive_bools
)]

use core::ops::Range;
use core::time::Duration;

use kinavis::estimator::{Estimator, EstimatorConfig, LatePolicy, SteadyMotion};
use kinavis::observations::{
    HeadingObservation, PositionObservation, SpeedThroughWaterObservation, VelocityObservation,
};
use kinavis::{
    wrap180, Angle, Distance, GeodeticPoint, GnssFix, Height, Instant, LocalFrame, NavigationEvent,
    NavigationIntegrity, NavigationState, Ned, Position, SensorHealth, Speed, StateComponent,
    StatePriors, TrueCourse, Utc, Vector3,
};
use kinavis_kernel::matrix::{Matrix, Vector};

// ---------------------------------------------------------------- randomness

/// xorshift64* with Box–Muller. Deterministic from the seed.
struct Rng {
    state: u64,
    spare: Option<f64>,
}

impl Rng {
    fn new(seed: u64) -> Self {
        Self {
            state: seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1,
            spare: None,
        }
    }

    fn next_u64(&mut self) -> u64 {
        let mut x = self.state;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.state = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    /// Uniform on (0, 1).
    fn uniform(&mut self) -> f64 {
        ((self.next_u64() >> 11) as f64 + 0.5) / (1u64 << 53) as f64
    }

    /// Standard normal.
    fn normal(&mut self) -> f64 {
        if let Some(spare) = self.spare.take() {
            return spare;
        }
        let radius = (-2.0 * self.uniform().ln()).sqrt();
        let angle = core::f64::consts::TAU * self.uniform();
        self.spare = Some(radius * angle.sin());
        radius * angle.cos()
    }

    fn normal_with(&mut self, sigma: f64) -> f64 {
        sigma * self.normal()
    }
}

// --------------------------------------------------------------- chi-square

/// χ² quantile, Wilson–Hilferty; ~1 % accurate for the dof here (hundreds).
fn chi_square_quantile(dof: f64, z: f64) -> f64 {
    let a = 2.0 / (9.0 * dof);
    dof * (1.0 - a + z * a.sqrt()).powi(3)
}

/// Interval containing the mean of `samples` χ²(`dof`) variables with
/// probability 1 − 2Φ(−z).
fn mean_interval(dof: usize, samples: usize, z: f64) -> Range<f64> {
    let total = (dof * samples) as f64;
    let lower = chi_square_quantile(total, -z) / samples as f64;
    let upper = chi_square_quantile(total, z) / samples as f64;
    lower..upper
}

// ------------------------------------------------------------------ the sea

const KNOT: f64 = 1852.0 / 3600.0;
const SUBSTEPS: usize = 10;

/// True state: six components in the estimator's anchored frame.
#[derive(Clone, Copy)]
struct Truth {
    north: f64,
    east: f64,
    heading: f64,
    speed: f64,
    current_north: f64,
    current_east: f64,
}

impl Truth {
    fn velocity(&self) -> (f64, f64) {
        (
            self.speed * self.heading.cos() + self.current_north,
            self.speed * self.heading.sin() + self.current_east,
        )
    }

    /// Advances one second in substeps: position follows velocity; heading,
    /// speed and current random-walk per the process model, plus any turn
    /// unknown to the model. Returns the truth `part_way` substeps in, for the
    /// late-reporting receiver.
    fn advance(&mut self, walk: &Walk, turn_rate: f64, rng: &mut Rng, part_way: usize) -> Self {
        let h = 1.0 / SUBSTEPS as f64;
        let mut snapshot = *self;
        for substep in 0..SUBSTEPS {
            if substep == part_way {
                snapshot = *self;
            }
            let (vn, ve) = self.velocity();
            self.north += vn * h;
            self.east += ve * h;
            self.heading += turn_rate * h + rng.normal_with(walk.heading * h.sqrt());
            self.speed += rng.normal_with(walk.speed * h.sqrt());
            self.current_north += rng.normal_with(walk.current * h.sqrt());
            self.current_east += rng.normal_with(walk.current * h.sqrt());
        }
        snapshot
    }

    fn position(&self, frame: &LocalFrame) -> Position {
        let displacement: Vector3<Ned, Distance> = Vector3::new(
            Distance::from_metres(self.north).unwrap(),
            Distance::from_metres(self.east).unwrap(),
            Distance::ZERO,
        );
        // Northings this small cannot fail: the frame is anchored nearby.
        frame.point_from_ned(displacement).unwrap().position()
    }

    fn vector(&self) -> Vector<6> {
        Vector::from_column([
            self.north,
            self.east,
            self.heading,
            self.speed,
            self.current_north,
            self.current_east,
        ])
    }
}

/// Random-walk intensities per √s, as in [`SteadyMotion::standard`].
struct Walk {
    heading: f64,
    speed: f64,
    current: f64,
}

impl Walk {
    fn standard() -> Self {
        Self {
            heading: 0.5_f64.to_radians(),
            speed: 0.05,
            current: 0.025,
        }
    }
}

// ---------------------------------------------------------------- scenarios

/// Sensor behaviour per second.
struct Scenario {
    name: &'static str,
    seed: u64,
    runs: usize,
    seconds: i64,
    course_degrees: f64,
    speed_knots: f64,
    current_knots_north: f64,
    /// Turn rate, °/s, unknown to the model.
    turn_rate: f64,
    /// Seconds with no receiver output.
    gnss_outage: Option<Range<i64>>,
    /// Seconds with receiver positions offset this far north of the truth.
    spoof: Option<(Range<i64>, f64)>,
    /// Receiver report latency, ms; zero is in time.
    gnss_lag_millis: u64,
    /// From this second, actual gyro noise is this multiple of the claimed
    /// noise.
    gyro_noisy_from: Option<(i64, f64)>,
    /// From this second, gyro drift in °/s.
    gyro_drift_from: Option<(i64, f64)>,
}

impl Scenario {
    fn straight(name: &'static str, seed: u64) -> Self {
        Self {
            name,
            seed,
            runs: 40,
            seconds: 600,
            course_degrees: 75.0,
            speed_knots: 12.0,
            current_knots_north: 1.5,
            turn_rate: 0.0,
            gnss_outage: None,
            spoof: None,
            gnss_lag_millis: 0,
            gyro_noisy_from: None,
            gyro_drift_from: None,
        }
    }
}

const SIGMA_POSITION: f64 = 5.0;
const SIGMA_VELOCITY: f64 = 0.3 * KNOT;
const SIGMA_HEADING: f64 = 0.5;
const SIGMA_LOG: f64 = 0.2 * KNOT;
/// Settling time after which the initial priors no longer matter.
const SETTLED: i64 = 120;

/// Per-run record.
#[derive(Default)]
struct Record {
    /// NEES at each sampled second.
    nees: Vec<f64>,
    /// NIS of accepted position observations after settling.
    position_nis: Vec<f64>,
    /// NIS of accepted heading observations after settling.
    heading_nis: Vec<f64>,
    /// Per second: true position inside the 99.9 % ellipse.
    covered: Vec<bool>,
    /// Position rejections after settling.
    position_rejected: usize,
    heading_rejected: usize,
    /// All events with their second.
    events: Vec<(i64, NavigationEvent)>,
    /// Final position error, m.
    final_error: f64,
    final_semi_major: f64,
    /// Final heading error, σ.
    final_heading_sigmas: f64,
    /// Final current error, m/s.
    final_current_error: f64,
}

fn at(seconds: i64) -> Instant<Utc> {
    Instant::from_unix_seconds(1_700_000_000 + seconds)
}

fn nees(truth: &Truth, state: &NavigationState) -> f64 {
    let mut delta = truth.vector() - *state.vector();
    let heading = StateComponent::Heading.index();
    let turn = delta.element(heading).unwrap();
    delta.set(heading, 0, wrap180(turn.to_degrees()).to_radians());
    let factor = state.covariance().cholesky().expect("a covariance");
    let weighted = factor
        .solve(&delta)
        .expect("a positive definite covariance");
    delta.dot(&weighted)
}

/// Whether the true position is inside the estimate's 99.9 % ellipse.
fn covered(truth: &Truth, state: &NavigationState) -> bool {
    let delta = Vector::<2>::from_column([
        truth.north - state.vector().element(0).unwrap(),
        truth.east - state.vector().element(1).unwrap(),
    ]);
    let p = state.covariance();
    let block = Matrix::<2, 2>::from_rows([
        [p.get(0, 0).unwrap(), p.get(0, 1).unwrap()],
        [p.get(1, 0).unwrap(), p.get(1, 1).unwrap()],
    ]);
    let weighted = block.cholesky().unwrap().solve(&delta).unwrap();
    delta.dot(&weighted) <= 13.82
}

fn run(scenario: &Scenario, index: usize) -> Record {
    let mut rng = Rng::new(scenario.seed.wrapping_add(index as u64 * 7919));
    let walk = Walk::standard();
    let priors = StatePriors::standard();
    let anchor: Position = "50°00.0'N 001°00.0'W".parse().unwrap();
    let frame = LocalFrame::at(
        GeodeticPoint::new(anchor, Height::above_ellipsoid(Distance::ZERO)),
        &kinavis::Ellipsoid::WGS84,
    )
    .unwrap();

    // The estimator starts from a fix at the anchor with COG and SOG; the truth
    // is drawn around it from the priors.
    let course = scenario.course_degrees.to_radians();
    let speed = scenario.speed_knots * KNOT;
    let current_north = scenario.current_knots_north * KNOT;
    let ground = (speed * course.cos() + current_north, speed * course.sin());
    let ground_course = ground.1.atan2(ground.0).to_degrees();
    let ground_speed = ground.0.hypot(ground.1);
    let fix = GnssFix::builder(at(0), anchor)
        .course_over_ground(TrueCourse::wrap(ground_course).unwrap())
        .speed_over_ground(Speed::from_metres_per_second(ground_speed).unwrap())
        .build();
    let initial = NavigationState::initialised_with(&fix, None, &priors).unwrap();
    let mut truth = Truth {
        north: rng.normal_with(priors.position().metres()),
        east: rng.normal_with(priors.position().metres()),
        heading: ground_course.to_radians() + rng.normal_with(priors.heading().radians()),
        speed: ground_speed + rng.normal_with(priors.speed().metres_per_second()),
        current_north: rng.normal_with(priors.current().metres_per_second()),
        current_east: rng.normal_with(priors.current().metres_per_second()),
    };
    let config = EstimatorConfig::standard().with_late(LatePolicy::Smooth {
        max_lag: Duration::from_secs(2),
    });
    let mut estimator = Estimator::new(initial, SteadyMotion::standard(), config);
    let mut record = Record::default();
    let turn_rate = scenario.turn_rate.to_radians();
    let mut gyro_bias = 0.0;

    for second in 1..=scenario.seconds {
        // The late receiver measures part way through the second, before the
        // truth advances.
        let part_way = ((1000 - scenario.gnss_lag_millis) as usize * SUBSTEPS) / 1000;
        let part_way_truth = truth.advance(&walk, turn_rate, &mut rng, part_way);
        let measured_at = (scenario.gnss_lag_millis > 0).then_some(part_way_truth);
        let settled = second > SETTLED;
        let note = |record: &mut Record, outcome: kinavis::Outcome, name: &str| {
            for event in outcome.events() {
                record.events.push((second, *event));
            }
            if let Some(report) = outcome.report() {
                if settled {
                    match (name, report.accepted()) {
                        ("position", true) => {
                            record
                                .position_nis
                                .push(report.normalised_innovation_squared());
                        }
                        ("position", false) => record.position_rejected += 1,
                        ("heading", true) => {
                            record
                                .heading_nis
                                .push(report.normalised_innovation_squared());
                        }
                        ("heading", false) => record.heading_rejected += 1,
                        _ => {}
                    }
                }
            }
        };

        // Gyro reading for this second.
        let gyro_sigma = match scenario.gyro_noisy_from {
            Some((from, factor)) if second >= from => SIGMA_HEADING * factor,
            _ => SIGMA_HEADING,
        };
        if let Some((from, rate)) = scenario.gyro_drift_from {
            if second >= from {
                gyro_bias += rate;
            }
        }
        let read = truth.heading.to_degrees() + gyro_bias + rng.normal_with(gyro_sigma);
        let outcome = estimator
            .ingest(&HeadingObservation::new(
                at(second),
                TrueCourse::wrap(read).unwrap(),
                Angle::from_degrees(SIGMA_HEADING).unwrap(),
            ))
            .unwrap();
        note(&mut record, outcome, "heading");

        // Log reading.
        let outcome = estimator
            .ingest(&SpeedThroughWaterObservation::new(
                at(second),
                Speed::from_metres_per_second(truth.speed + rng.normal_with(SIGMA_LOG)).unwrap(),
                Speed::from_metres_per_second(SIGMA_LOG).unwrap(),
            ))
            .unwrap();
        note(&mut record, outcome, "log");

        // Receiver: in time or late, honest or spoofed, or silent.
        let silent = scenario
            .gnss_outage
            .as_ref()
            .is_some_and(|outage| outage.contains(&second));
        if !silent {
            let (measured, stamp) = match measured_at {
                Some(part) => (
                    part,
                    at(second - 1)
                        .saturating_add(Duration::from_millis(1000 - scenario.gnss_lag_millis)),
                ),
                None => (truth, at(second)),
            };
            let offset = match &scenario.spoof {
                Some((during, metres)) if during.contains(&second) => *metres,
                _ => 0.0,
            };
            let reported = Truth {
                north: measured.north + offset + rng.normal_with(SIGMA_POSITION),
                east: measured.east + rng.normal_with(SIGMA_POSITION),
                ..measured
            };
            let outcome = estimator
                .ingest(&PositionObservation::new(
                    stamp,
                    reported.position(&frame),
                    Distance::from_metres(SIGMA_POSITION).unwrap(),
                ))
                .unwrap();
            note(&mut record, outcome, "position");
            let (vn, ve) = measured.velocity();
            let velocity: Vector3<Ned, Speed> = Vector3::new(
                Speed::from_metres_per_second(vn + rng.normal_with(SIGMA_VELOCITY)).unwrap(),
                Speed::from_metres_per_second(ve + rng.normal_with(SIGMA_VELOCITY)).unwrap(),
                Speed::ZERO,
            );
            let outcome = estimator
                .ingest(&VelocityObservation::new(
                    stamp,
                    velocity,
                    Speed::from_metres_per_second(SIGMA_VELOCITY).unwrap(),
                ))
                .unwrap();
            note(&mut record, outcome, "velocity");
        }

        // Belief at the whole second vs the truth.
        let state = *estimator.state();
        assert_eq!(state.valid_at(), at(second));
        assert_eq!(
            state.frame().origin().position(),
            frame.origin().position(),
            "the anchor moved"
        );
        if settled && second % 60 == 0 {
            record.nees.push(nees(&truth, &state));
        }
        if settled {
            record.covered.push(covered(&truth, &state));
        }
        if second == scenario.seconds {
            record.final_error = (truth.north - state.vector().element(0).unwrap())
                .hypot(truth.east - state.vector().element(1).unwrap());
            record.final_semi_major = state.horizontal_error().semi_major().metres();
            let heading_error =
                wrap180((truth.heading - state.vector().element(2).unwrap()).to_degrees());
            record.final_heading_sigmas = heading_error.abs() / state.heading_sigma().degrees();
            record.final_current_error = (truth.current_north
                - state.current().north().metres_per_second())
            .hypot(truth.current_east - state.current().east().metres_per_second());
        }
    }
    record
}

/// Aggregate over runs.
struct Summary {
    mean_nees: f64,
    nees_samples: usize,
    mean_position_nis: f64,
    position_nis_samples: usize,
    mean_heading_nis: f64,
    heading_nis: Vec<f64>,
    coverage: f64,
    position_rejected: usize,
    heading_rejected: usize,
    records: Vec<Record>,
}

fn simulate(scenario: &Scenario) -> Summary {
    let records: Vec<Record> = (0..scenario.runs)
        .map(|index| run(scenario, index))
        .collect();
    let mean = |values: &[f64]| values.iter().sum::<f64>() / values.len().max(1) as f64;
    let nees: Vec<f64> = records
        .iter()
        .flat_map(|r| r.nees.iter().copied())
        .collect();
    let position_nis: Vec<f64> = records
        .iter()
        .flat_map(|r| r.position_nis.iter().copied())
        .collect();
    let heading_nis: Vec<f64> = records
        .iter()
        .flat_map(|r| r.heading_nis.iter().copied())
        .collect();
    let covered: Vec<bool> = records
        .iter()
        .flat_map(|r| r.covered.iter().copied())
        .collect();
    let summary = Summary {
        mean_nees: mean(&nees),
        nees_samples: nees.len(),
        mean_position_nis: mean(&position_nis),
        position_nis_samples: position_nis.len(),
        mean_heading_nis: mean(&heading_nis),
        heading_nis,
        coverage: covered.iter().filter(|c| **c).count() as f64 / covered.len().max(1) as f64,
        position_rejected: records.iter().map(|r| r.position_rejected).sum(),
        heading_rejected: records.iter().map(|r| r.heading_rejected).sum(),
        records,
    };
    eprintln!(
        "{}: NEES {:.2} over {} samples, position NIS {:.3} over {}, heading NIS {:.3}, \
         coverage {:.3}, rejected {} positions {} headings, final error {:.1} m (σ {:.1} m)",
        scenario.name,
        summary.mean_nees,
        summary.nees_samples,
        summary.mean_position_nis,
        summary.position_nis_samples,
        summary.mean_heading_nis,
        summary.coverage,
        summary.position_rejected,
        summary.heading_rejected,
        mean(
            &summary
                .records
                .iter()
                .map(|r| r.final_error)
                .collect::<Vec<_>>()
        ),
        mean(
            &summary
                .records
                .iter()
                .map(|r| r.final_semi_major)
                .collect::<Vec<_>>()
        ),
    );
    summary
}

/// Consistency verdict: mean NEES and mean NIS within their χ² intervals.
///
/// NEES samples a minute apart are correlated (current error decorrelates
/// slowly), so the interval uses half the samples, at 99.9 %. Innovations of a
/// consistent filter are white, so the NIS interval is used as is, at 99.9 %,
/// widened 2 % for EKF linearisation.
fn assert_consistent(summary: &Summary) {
    let nees = mean_interval(6, summary.nees_samples / 2, 3.29);
    assert!(
        nees.contains(&summary.mean_nees),
        "NEES {:.2} outside {:.2}..{:.2}",
        summary.mean_nees,
        nees.start,
        nees.end
    );
    let nis = mean_interval(2, summary.position_nis_samples, 3.29);
    let nis = nis.start * 0.98..nis.end * 1.02;
    assert!(
        nis.contains(&summary.mean_position_nis),
        "position NIS {:.3} outside {:.3}..{:.3}",
        summary.mean_position_nis,
        nis.start,
        nis.end
    );
    // 0.1 % of readings, by the gate's own confidence.
    let readings = summary.position_nis_samples + summary.position_rejected;
    assert!(
        summary.position_rejected * 100 < readings,
        "{} of {} positions rejected",
        summary.position_rejected,
        readings
    );
}

/// Integrity at this second, from the events.
fn integrity_at(record: &Record, second: i64) -> NavigationIntegrity {
    record
        .events
        .iter()
        .filter(|(when, _)| *when <= second)
        .filter_map(|(_, event)| match event {
            NavigationEvent::IntegrityChanged { to, .. } => Some(*to),
            _ => None,
        })
        .next_back()
        .unwrap_or(NavigationIntegrity::DeadReckoning)
}

fn health_at(record: &Record, name: &str, second: i64) -> SensorHealth {
    record
        .events
        .iter()
        .filter(|(when, _)| *when <= second)
        .filter_map(|(_, event)| match event {
            NavigationEvent::SensorHealthChanged { sensor, to, .. } if *sensor == name => Some(*to),
            _ => None,
        })
        .next_back()
        .unwrap_or(SensorHealth::Healthy)
}

// -------------------------------------------------------------------- tests

#[test]
fn at_anchor_the_filter_is_consistent() {
    let mut scenario = Scenario::straight("at anchor", 1);
    scenario.speed_knots = 0.0;
    scenario.current_knots_north = 0.0;
    let summary = simulate(&scenario);
    assert_consistent(&summary);
    assert!(summary.coverage > 0.99, "{}", summary.coverage);
}

#[test]
fn on_a_straight_passage_the_filter_is_consistent() {
    let summary = simulate(&Scenario::straight("straight passage", 2));
    assert_consistent(&summary);
    assert!(summary.coverage > 0.99, "{}", summary.coverage);
    for record in &summary.records {
        assert_eq!(integrity_at(record, 600), NavigationIntegrity::Nominal);
        assert!(record.final_error < 15.0, "{} m", record.final_error);
    }
}

#[test]
fn with_the_receiver_always_late_the_filter_is_still_consistent() {
    let mut scenario = Scenario::straight("receiver 400 ms late", 3);
    scenario.gnss_lag_millis = 400;
    let summary = simulate(&scenario);
    assert_consistent(&summary);
    assert!(summary.coverage > 0.99, "{}", summary.coverage);
    // Every late report folded in; none rejected as out of order.
    for record in &summary.records {
        assert!(
            !record.events.iter().any(|(_, event)| matches!(
                event,
                NavigationEvent::ObservationRejected {
                    reason: kinavis::RejectionReason::OutOfOrder,
                    ..
                }
            )),
            "a late report was turned away"
        );
    }
}

#[test]
fn in_a_steady_turn_the_error_stays_inside_the_ellipse() {
    // 20°/min turn, treated by the model as heading random walk: NEES is not
    // χ², but the ellipse still covers the truth and the gyro is not suspected.
    let mut scenario = Scenario::straight("steady turn", 4);
    scenario.turn_rate = 20.0 / 60.0;
    scenario.runs = 30;
    let summary = simulate(&scenario);
    assert!(summary.coverage > 0.97, "{}", summary.coverage);
    // The turn shows in gyro innovations but closes the gate on only a few per
    // thousand.
    assert!(
        summary.heading_rejected * 100 < summary.heading_nis.len() + summary.heading_rejected,
        "{} headings rejected",
        summary.heading_rejected
    );
    assert!(
        summary.mean_nees < 3.0 * 6.0,
        "NEES {:.2}: the ellipse has lost the truth",
        summary.mean_nees
    );
    for record in &summary.records {
        assert!(record.final_error < 20.0, "{} m", record.final_error);
    }
}

#[test]
fn through_a_gnss_outage_the_uncertainty_grows_honestly() {
    let mut scenario = Scenario::straight("60 s without GNSS", 5);
    scenario.gnss_outage = Some(300..360);
    scenario.runs = 30;
    let summary = simulate(&scenario);
    assert!(summary.coverage > 0.99, "{}", summary.coverage);
    // NEES at sampled minutes (300 s: last fix; 360 s: first after) stays χ²
    // across the outage.
    assert_consistent(&summary);
    for record in &summary.records {
        // 30 s into the outage integrity is dead reckoning; the first fix after
        // restores it.
        assert_eq!(integrity_at(record, 329), NavigationIntegrity::Nominal);
        assert_eq!(
            integrity_at(record, 331),
            NavigationIntegrity::DeadReckoning
        );
        assert_eq!(integrity_at(record, 361), NavigationIntegrity::Nominal);
        assert!(record.events.iter().any(|(when, event)| *when == 360
            && matches!(
                event,
                NavigationEvent::IntegrityChanged {
                    to: NavigationIntegrity::Nominal,
                    ..
                }
            )));
    }
}

#[test]
fn a_spoofed_receiver_is_gated_and_then_doubted() {
    let mut scenario = Scenario::straight("spoofed 500 m for 40 s", 6);
    scenario.spoof = Some((300..340, 500.0));
    scenario.runs = 30;
    let summary = simulate(&scenario);
    assert!(summary.coverage > 0.99, "{}", summary.coverage);
    for record in &summary.records {
        // Every spoofed position rejected; the fifth makes the receiver
        // suspect; the fifth honest one after clears it.
        assert_eq!(health_at(record, "position", 303), SensorHealth::Healthy);
        assert_eq!(health_at(record, "position", 304), SensorHealth::Suspect);
        assert_eq!(health_at(record, "position", 343), SensorHealth::Suspect);
        assert_eq!(health_at(record, "position", 344), SensorHealth::Healthy);
        // 30 s without an honest fix: dead reckoning, reported.
        assert_eq!(integrity_at(record, 329), NavigationIntegrity::Nominal);
        assert_eq!(
            integrity_at(record, 331),
            NavigationIntegrity::DeadReckoning
        );
        assert_eq!(integrity_at(record, 341), NavigationIntegrity::Nominal);
        assert!(record.final_error < 15.0, "{} m", record.final_error);
    }
    // 40 spoofed positions per run, none accepted; occasional honest ones
    // rejected at the gate's 0.1 % rate.
    assert!(summary.position_rejected >= 40 * scenario.runs);
    assert!(summary.position_rejected <= 41 * scenario.runs);
}

#[test]
fn a_gyro_gone_noisy_is_doubted_and_the_position_holds() {
    // Gyro scatter 20× the claimed noise: the gate rejects most readings, the
    // gyro becomes suspect, position and integrity are held by the receiver.
    // Heading is inconsistent (accepted readings are still noisier than
    // modelled), which the suspicion flags; position is consistent.
    let mut scenario = Scenario::straight("gyro noisy from 300 s", 7);
    scenario.gyro_noisy_from = Some((300, 20.0));
    scenario.runs = 30;
    let summary = simulate(&scenario);
    assert!(summary.coverage > 0.99, "{}", summary.coverage);
    for record in &summary.records {
        assert_eq!(health_at(record, "heading", 299), SensorHealth::Healthy);
        assert_eq!(health_at(record, "heading", 600), SensorHealth::Suspect);
        assert_eq!(integrity_at(record, 600), NavigationIntegrity::Nominal);
        assert!(record.final_error < 15.0, "{} m", record.final_error);
    }
    // Most degraded gyro readings rejected.
    assert!(
        summary.heading_rejected > 210 * scenario.runs,
        "{}",
        summary.heading_rejected
    );
}

#[test]
fn a_gyro_that_drifts_is_absorbed_by_the_current_and_the_position_holds() {
    // Gyro drift of 3°/min, 15° by the end. The state cannot distinguish
    // heading error from a current that reconciles the ground track, so the
    // filter follows the gyro and the current absorbs the difference; position,
    // held by the receiver, stays inside its ellipse, heading does not. This is
    // the limit of a single heading reference.
    let mut scenario = Scenario::straight("gyro drifting from 300 s", 8);
    scenario.gyro_drift_from = Some((300, 3.0 / 60.0));
    scenario.runs = 30;
    let summary = simulate(&scenario);
    assert!(summary.coverage > 0.99, "{}", summary.coverage);
    let speed = scenario.speed_knots * KNOT;
    // 15° at 12 kn gives a spurious current of 2u sin 7.5° ≈ 1.6 m/s (3 kn).
    // Varies per run with the current's own walk; on average matches the
    // geometry.
    let order = 2.0 * speed * (7.5_f64.to_radians()).sin();
    let mean_current_error = summary
        .records
        .iter()
        .map(|r| r.final_current_error)
        .sum::<f64>()
        / summary.records.len() as f64;
    eprintln!(
        "gyro drift: current {mean_current_error:.2} m/s off on average, geometry says {order:.2}"
    );
    assert!(
        (mean_current_error - order).abs() < 0.3 * order,
        "current {mean_current_error:.2} m/s off on average, expected about {order:.2}"
    );
    for record in &summary.records {
        assert!(record.final_error < 15.0, "{} m", record.final_error);
        assert!(
            record.final_heading_sigmas > 3.0,
            "heading only {:.1}σ off: the drift was seen through",
            record.final_heading_sigmas
        );
        assert!(
            record.final_current_error > 0.2,
            "current only {:.2} m/s off",
            record.final_current_error
        );
        assert_eq!(integrity_at(record, 600), NavigationIntegrity::Nominal);
    }
}
