//! Monte Carlo consistency of the inertial filter (NEES, NIS).
//!
//! Simulates a passage with a turn, an IMU with the noise and bias walk the
//! filter assumes, GNSS and a compass with the sigmas the updates claim, and an
//! initial error drawn from the priors. Consistency: mean NEES over all 15
//! states follows χ²(15), mean position NIS follows χ²(2). Falling outside the
//! interval indicates a sign error in the error dynamics, a missing noise term
//! or a Jacobian that does not match the mechanisation.
//!
//! Random numbers are local (xorshift + Box–Muller): reproducible from the
//! seed, no dependency.

#![allow(
    clippy::unwrap_used,
    clippy::panic,
    clippy::cast_precision_loss,
    clippy::indexing_slicing,
    clippy::too_many_lines
)]

use core::ops::Range;
use core::time::Duration;

use kinavis_ins::{
    gravity_down, GatingPolicy, ImuNoise, ImuSample, InsFilter, InsPriors, Quaternion, Strapdown,
    EARTH_RATE, ERROR_STATE_DIM,
};
use kinavis_kernel::matrix::Matrix;
use kinavis_kernel::{
    Angle, Distance, GeodeticPoint, Height, Instant, Ned, Position, Speed, TrueCourse, Utc, Vector3,
};

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

    fn uniform(&mut self) -> f64 {
        ((self.next_u64() >> 11) as f64 + 0.5) / (1u64 << 53) as f64
    }

    fn normal(&mut self) -> f64 {
        if let Some(spare) = self.spare.take() {
            return spare;
        }
        let radius = (-2.0 * self.uniform().ln()).sqrt();
        let angle = core::f64::consts::TAU * self.uniform();
        self.spare = Some(radius * angle.sin());
        radius * angle.cos()
    }

    fn normal3(&mut self, sigma: f64) -> [f64; 3] {
        [
            sigma * self.normal(),
            sigma * self.normal(),
            sigma * self.normal(),
        ]
    }
}

// --------------------------------------------------------------- chi-square

/// χ² quantile, Wilson–Hilferty approximation.
fn chi_square_quantile(dof: f64, z: f64) -> f64 {
    let a = 2.0 / (9.0 * dof);
    dof * (1.0 - a + z * a.sqrt()).powi(3)
}

/// Interval containing the mean of `samples` χ²(`dof`) variables with
/// probability 1 − 2Φ(−z).
fn mean_interval(dof: usize, samples: usize, z: f64) -> Range<f64> {
    let total = (dof * samples) as f64;
    chi_square_quantile(total, -z) / samples as f64..chi_square_quantile(total, z) / samples as f64
}

// ------------------------------------------------------------------ the sea

const LATITUDE: f64 = 50.0;
const IMU_HZ: usize = 10;
const SECONDS: usize = 200;
const SETTLED_AFTER: usize = 60;
const RUNS: usize = 24;
const SPEED: f64 = 6.0;

const GNSS_POSITION_SIGMA: f64 = 3.0;
const GNSS_VELOCITY_SIGMA: f64 = 0.1;
const HEADING_SIGMA_DEGREES: f64 = 0.5;

fn add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// True vessel state in the filter frame.
struct Truth {
    position: [f64; 3],
    heading: f64,
    gyro_bias: [f64; 3],
    accel_bias: [f64; 3],
}

impl Truth {
    fn attitude(&self) -> Quaternion {
        Quaternion::from_euler(
            Angle::ZERO,
            Angle::ZERO,
            TrueCourse::from_degrees_wrapped(self.heading.to_degrees()),
        )
    }

    fn velocity(&self) -> [f64; 3] {
        [SPEED * self.heading.cos(), SPEED * self.heading.sin(), 0.0]
    }

    /// Rate of turn over the passage: straight, 0.5°/s to starboard for 60 s,
    /// straight.
    fn yaw_rate(second: f64) -> f64 {
        if (90.0..150.0).contains(&second) {
            0.5_f64.to_radians()
        } else {
            0.0
        }
    }

    fn latitude(&self) -> f64 {
        LATITUDE.to_radians() + self.position[0] / 6_371_000.0
    }

    /// IMU reading over the next interval, and the propagated truth.
    fn step(&mut self, second: f64, dt: f64, noise: &ImuNoise, rng: &mut Rng) -> ImuSample {
        let rate = Self::yaw_rate(second);
        let latitude = self.latitude();
        let earth = [
            EARTH_RATE * latitude.cos(),
            0.0,
            -EARTH_RATE * latitude.sin(),
        ];
        let velocity = self.velocity();
        let transport = [
            velocity[1] / 6_371_000.0,
            -velocity[0] / 6_371_000.0,
            -velocity[1] * latitude.tan() / 6_371_000.0,
        ];
        let acceleration = [
            -SPEED * rate * self.heading.sin(),
            SPEED * rate * self.heading.cos(),
            0.0,
        ];
        let coriolis = cross(
            add([2.0 * earth[0], 0.0, 2.0 * earth[2]], transport),
            velocity,
        );
        let force_n = add(
            add(acceleration, [0.0, 0.0, -gravity_down(latitude.sin(), 0.0)]),
            coriolis,
        );
        let attitude = self.attitude();
        let angular_rate = add(
            add(
                attitude.rotate_back(add(earth, transport)),
                [0.0, 0.0, rate],
            ),
            add(
                self.gyro_bias,
                rng.normal3(noise.angle_random_walk() / dt.sqrt()),
            ),
        );
        let specific_force = add(
            attitude.rotate_back(force_n),
            add(
                self.accel_bias,
                rng.normal3(noise.velocity_random_walk() / dt.sqrt()),
            ),
        );

        // Propagate: heading turns, position follows the mean velocity, biases
        // walk.
        let before = velocity;
        self.heading += rate * dt;
        let after = self.velocity();
        self.position = add(
            self.position,
            [
                (before[0] + after[0]) * dt / 2.0,
                (before[1] + after[1]) * dt / 2.0,
                0.0,
            ],
        );
        self.gyro_bias = add(
            self.gyro_bias,
            rng.normal3(noise.gyro_bias_walk() * dt.sqrt()),
        );
        self.accel_bias = add(
            self.accel_bias,
            rng.normal3(noise.accel_bias_walk() * dt.sqrt()),
        );
        ImuSample::new(angular_rate, specific_force, Duration::from_secs_f64(dt)).unwrap()
    }
}

// ----------------------------------------------------------------- one run

#[derive(Default)]
struct Summary {
    nees_sum: f64,
    nees_samples: usize,
    nis_sum: f64,
    nis_samples: usize,
    rejected: usize,
    final_position_error: f64,
    final_heading_error: f64,
    final_ellipse: f64,
}

fn run(seed: u64, noise: ImuNoise, summary: &mut Summary) {
    let mut rng = Rng::new(seed);
    let priors = InsPriors::standard();
    let dt = 1.0 / IMU_HZ as f64;
    let start = Instant::<Utc>::from_unix_seconds(1_789_000_000);
    let origin = GeodeticPoint::new(
        Position::from_degrees(LATITUDE, -1.0).unwrap(),
        Height::above_ellipsoid(Distance::ZERO),
    );

    // Truth, and a filter initialised with errors drawn from the priors:
    // nominal = truth − error.
    let heading = 45.0_f64.to_radians();
    let mut truth = Truth {
        position: [
            10.0 * rng.normal(),
            10.0 * rng.normal(),
            20.0 * rng.normal(),
        ],
        heading,
        gyro_bias: rng.normal3(0.05_f64.to_radians()),
        accel_bias: rng.normal3(2e-3 * 9.806_65),
    };
    let velocity_error = rng.normal3(0.5);
    let misalignment = [
        1.0_f64.to_radians() * rng.normal(),
        1.0_f64.to_radians() * rng.normal(),
        3.0_f64.to_radians() * rng.normal(),
    ];
    let nominal_velocity = sub(truth.velocity(), velocity_error);
    let nominal_attitude = Quaternion::from_rotation_vector(misalignment).then(&truth.attitude());
    let nominal = Strapdown::new(
        start,
        origin,
        Vector3::<Ned, Speed>::new(
            Speed::from_metres_per_second(nominal_velocity[0]).unwrap(),
            Speed::from_metres_per_second(nominal_velocity[1]).unwrap(),
            Speed::from_metres_per_second(nominal_velocity[2]).unwrap(),
        ),
        nominal_attitude,
    )
    .unwrap();
    let mut filter = InsFilter::new(nominal, noise, &priors);
    let frame = *filter.nominal().frame();

    for second in 0..SECONDS {
        for tick in 0..IMU_HZ {
            let sample = truth.step(second as f64 + tick as f64 * dt, dt, &noise, &mut rng);
            filter.predict(&sample).unwrap();
        }
        let now = second + 1;

        // GNSS position and velocity, compass.
        let measured = add(truth.position, rng.normal3(GNSS_POSITION_SIGMA));
        let point = frame
            .point_from_ned(Vector3::<Ned, Distance>::new(
                Distance::from_metres(measured[0]).unwrap(),
                Distance::from_metres(measured[1]).unwrap(),
                Distance::from_metres(measured[2]).unwrap(),
            ))
            .unwrap();
        let update = filter
            .update_point(
                point,
                Distance::from_metres(GNSS_POSITION_SIGMA).unwrap(),
                Distance::from_metres(GNSS_POSITION_SIGMA).unwrap(),
                GatingPolicy::reject_above(16.27),
            )
            .unwrap();
        if now > SETTLED_AFTER {
            if update.accepted {
                summary.nis_sum += update.nis;
                summary.nis_samples += 1;
            } else {
                summary.rejected += 1;
            }
        }
        let measured = add(truth.velocity(), rng.normal3(GNSS_VELOCITY_SIGMA));
        filter
            .update_velocity(
                Vector3::<Ned, Speed>::new(
                    Speed::from_metres_per_second(measured[0]).unwrap(),
                    Speed::from_metres_per_second(measured[1]).unwrap(),
                    Speed::from_metres_per_second(measured[2]).unwrap(),
                ),
                Speed::from_metres_per_second(GNSS_VELOCITY_SIGMA).unwrap(),
                GatingPolicy::none(),
            )
            .unwrap();
        let measured = truth.heading.to_degrees() + HEADING_SIGMA_DEGREES * rng.normal();
        filter
            .update_heading(
                TrueCourse::from_degrees_wrapped(measured),
                Angle::from_degrees(HEADING_SIGMA_DEGREES).unwrap(),
                GatingPolicy::none(),
            )
            .unwrap();

        if now > SETTLED_AFTER {
            summary.nees_sum += nees(&filter, &truth);
            summary.nees_samples += 1;
        }
    }

    let position_error = sub(truth.position, ned(&filter));
    summary.final_position_error += position_error[0].hypot(position_error[1]);
    summary.final_heading_error += (filter.attitude().yaw.degrees() - truth.heading.to_degrees())
        .rem_euclid(360.0)
        .min(
            360.0
                - (filter.attitude().yaw.degrees() - truth.heading.to_degrees()).rem_euclid(360.0),
        );
    summary.final_ellipse += filter.horizontal_error().semi_major().metres();
}

fn ned(filter: &InsFilter) -> [f64; 3] {
    let displacement = filter.nominal().displacement();
    [
        displacement.north().metres(),
        displacement.east().metres(),
        displacement.down().metres(),
    ]
}

/// `(x − x̂)ᵀ P⁻¹ (x − x̂)` over the 15 error states.
fn nees(filter: &InsFilter, truth: &Truth) -> f64 {
    let position = sub(truth.position, ned(filter));
    let velocity = filter.velocity();
    let velocity = sub(
        truth.velocity(),
        [
            velocity.north().metres_per_second(),
            velocity.east().metres_per_second(),
            velocity.down().metres_per_second(),
        ],
    );
    let misalignment = filter.quaternion().misalignment_from(&truth.attitude());
    let gyro = sub(truth.gyro_bias, filter.nominal().gyro_bias());
    let accel = sub(truth.accel_bias, filter.nominal().accel_bias());
    let mut error = [0.0; ERROR_STATE_DIM];
    for (block, values) in [position, velocity, misalignment, gyro, accel]
        .iter()
        .enumerate()
    {
        for axis in 0..3 {
            error[3 * block + axis] = values[axis];
        }
    }
    let column = Matrix::<ERROR_STATE_DIM, 1>::from_fn(|row, _| error[row]);
    let solved = filter
        .covariance()
        .cholesky()
        .unwrap()
        .solve(&column)
        .unwrap();
    (column.transpose() * solved).get(0, 0).unwrap()
}

// -------------------------------------------------------------- the verdict

fn consistent(noise: ImuNoise, seed: u64) -> Summary {
    let mut summary = Summary::default();
    for run_index in 0..RUNS {
        run(seed + run_index as u64, noise, &mut summary);
    }
    summary
}

fn assert_consistent(summary: &Summary, label: &str) {
    let mean_nees = summary.nees_sum / summary.nees_samples as f64;
    // NEES samples 1 s apart are strongly correlated (biases decorrelate over
    // minutes), so the interval uses a tenth of the samples, at 99.9 %.
    let nees = mean_interval(ERROR_STATE_DIM, summary.nees_samples / 10, 3.29);
    assert!(
        nees.contains(&mean_nees),
        "{label}: NEES {mean_nees:.2} outside {:.2}..{:.2}",
        nees.start,
        nees.end
    );
    let mean_nis = summary.nis_sum / summary.nis_samples as f64;
    let nis = mean_interval(3, summary.nis_samples, 3.29);
    let nis = nis.start * 0.98..nis.end * 1.02;
    assert!(
        nis.contains(&mean_nis),
        "{label}: position NIS {mean_nis:.3} outside {:.3}..{:.3}",
        nis.start,
        nis.end
    );
    // A 99.9 % gate on 3 dof rejects 0.1 %.
    assert!(
        summary.rejected * 100 < summary.nis_samples + summary.rejected,
        "{label}: {} of {} fixes refused",
        summary.rejected,
        summary.nis_samples + summary.rejected
    );
    let position_error = summary.final_position_error / RUNS as f64;
    let ellipse = summary.final_ellipse / RUNS as f64;
    assert!(
        position_error < 3.0 * ellipse,
        "{label}: final position error {position_error:.2} m against an ellipse of {ellipse:.2} m"
    );
    assert!(
        ellipse < GNSS_POSITION_SIGMA,
        "{label}: the fix is not helping: {ellipse:.2} m"
    );
    let heading_error = summary.final_heading_error / RUNS as f64;
    assert!(
        heading_error < HEADING_SIGMA_DEGREES,
        "{label}: heading error {heading_error:.3}°"
    );
}

#[test]
fn the_filter_is_consistent_with_a_mems_unit() {
    let summary = consistent(ImuNoise::mems(), 1);
    assert_consistent(&summary, "MEMS");
}

#[test]
fn the_filter_is_consistent_with_a_tactical_unit() {
    let summary = consistent(ImuNoise::tactical(), 2);
    assert_consistent(&summary, "tactical");
}
