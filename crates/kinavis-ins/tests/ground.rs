//! A tractor in a field: Monte Carlo consistency of the ground updates (NEES,
//! NIS) and the drift through a loss of GNSS with and without the wheels.
//!
//! The tractor works passes 12 m apart at 2.5 m/s with U-turns between them,
//! rolled on a side slope that changes with the pass. A MEMS IMU sits over
//! the rear axle; the RTK antenna is on the cab roof, 3.1 m up and 0.6 m
//! ahead. Updates: the antenna fix, the wheel speed and the non-holonomic
//! constraint at 5 Hz, the IMU at 10 Hz. NEES over the 15 states follows
//! χ²(15) and the fix's NIS χ²(3) — in the field with RTK, and through a
//! 30 s outage carried by the wheels; without the wheels the same outage
//! drifts many times further. The wheels once more as a dead-reckoning
//! receiver passes them on: the rear wheels' ticks in UBX `ESF-MEAS` frames,
//! read back by `kinavis-ubx`'s odometer, carry the same outage as well.

#![allow(
    clippy::unwrap_used,
    clippy::panic,
    clippy::cast_precision_loss,
    clippy::indexing_slicing,
    clippy::too_many_lines,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss
)]

use core::ops::Range;
use core::time::Duration;

use kinavis_ins::{
    gravity_down, GatingPolicy, ImuNoise, ImuSample, InsFilter, InsPriors, Quaternion, Strapdown,
    EARTH_RATE, ERROR_STATE_DIM,
};
use kinavis_kernel::matrix::Matrix;
use kinavis_kernel::{
    Angle, Body, Distance, GeodeticPoint, Height, Instant, Ned, Position, Speed, TrueCourse, Utc,
    Vector3,
};
use kinavis_ubx::{Deframer, Event, Message, Odometer, SpeedSource};

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

fn chi_square_quantile(dof: f64, z: f64) -> f64 {
    let a = 2.0 / (9.0 * dof);
    dof * (1.0 - a + z * a.sqrt()).powi(3)
}

fn mean_interval(dof: usize, samples: usize, z: f64) -> Range<f64> {
    let total = (dof * samples) as f64;
    chi_square_quantile(total, -z) / samples as f64..chi_square_quantile(total, z) / samples as f64
}

// ---------------------------------------------------------------- the field

const LATITUDE: f64 = 52.0;
const IMU_HZ: usize = 10;
const SECONDS: usize = 240;
const SETTLED_AFTER: usize = 60;
const RUNS: usize = 12;
const SPEED: f64 = 2.5;
/// Pass of 60 s, U-turn of radius 6 m; 12 m between passes.
const PASS_SECONDS: f64 = 60.0;
const TURN_RADIUS: f64 = 6.0;

const RTK_SIGMA: f64 = 0.02;
const RTK_HEIGHT_SIGMA: f64 = 0.04;
const WHEEL_SIGMA: f64 = 0.02;
const NHC_SIGMA: f64 = 0.03;
/// Wheel ticks: 5 mm of travel a tick, rear track 1.8 m. The speed from two
/// counts 200 ms apart is quantised to 25 mm/s: about 7 mm/s of noise.
const TICK: f64 = 0.005;
const TRACK: f64 = 1.8;
const TICK_SIGMA: f64 = 0.01;
const LEVER: [f64; 3] = [0.6, 0.0, -3.1];

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

/// Yaw rate: straight passes, then a half turn at `SPEED / TURN_RADIUS`,
/// alternately right and left.
fn yaw_rate(second: f64) -> f64 {
    let turn = core::f64::consts::PI * TURN_RADIUS / SPEED;
    let cycle = PASS_SECONDS + turn;
    let within = second.rem_euclid(cycle);
    let index = (second / cycle).floor() as i64;
    if within < PASS_SECONDS {
        0.0
    } else if index % 2 == 0 {
        SPEED / TURN_RADIUS
    } else {
        -SPEED / TURN_RADIUS
    }
}

/// Roll: the slope seen one way on the pass out and the other way back,
/// with the rocking of the wheel tracks.
fn roll(second: f64) -> (f64, f64) {
    let slope = 6f64.to_radians();
    let heading_factor = (0.03 * second).sin();
    let rock = 1.5f64.to_radians();
    let value = slope * heading_factor + rock * (1.3 * second).sin();
    let rate = slope * 0.03 * (0.03 * second).cos() + rock * 1.3 * (1.3 * second).cos();
    (value, rate)
}

struct Truth {
    position: [f64; 3],
    /// Travel of the rear wheels, left and right, metres.
    wheels: [f64; 2],
    heading: f64,
    gyro_bias: [f64; 3],
    accel_bias: [f64; 3],
    second: f64,
}

impl Truth {
    fn attitude(&self) -> Quaternion {
        Quaternion::from_euler(
            Angle::from_radians(roll(self.second).0).unwrap(),
            Angle::ZERO,
            TrueCourse::from_degrees_wrapped(self.heading.to_degrees()),
        )
    }

    fn velocity(&self) -> [f64; 3] {
        [SPEED * self.heading.cos(), SPEED * self.heading.sin(), 0.0]
    }

    fn antenna(&self) -> [f64; 3] {
        add(self.position, self.attitude().rotate(LEVER))
    }

    fn latitude(&self) -> f64 {
        LATITUDE.to_radians() + self.position[0] / 6_371_000.0
    }

    /// IMU reading over the next interval, and the propagated truth.
    fn step(&mut self, dt: f64, noise: &ImuNoise, rng: &mut Rng) -> ImuSample {
        // Rates at the middle of the interval: the mechanisation integrates a
        // constant rate over it, and so it is exact to second order.
        let rate = yaw_rate(self.second + dt / 2.0);
        let (phi, phi_rate) = roll(self.second + dt / 2.0);
        let latitude = self.latitude();
        let earth = [
            EARTH_RATE * latitude.cos(),
            0.0,
            -EARTH_RATE * latitude.sin(),
        ];
        // Heading, velocity and attitude at the middle of the interval too.
        let middle = Truth {
            position: self.position,
            wheels: self.wheels,
            heading: self.heading + rate * dt / 2.0,
            gyro_bias: self.gyro_bias,
            accel_bias: self.accel_bias,
            second: self.second + dt / 2.0,
        };
        let velocity = middle.velocity();
        let transport = [
            velocity[1] / 6_371_000.0,
            -velocity[0] / 6_371_000.0,
            -velocity[1] * latitude.tan() / 6_371_000.0,
        ];
        let acceleration = [
            -SPEED * rate * middle.heading.sin(),
            SPEED * rate * middle.heading.cos(),
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
        // Euler rates at zero pitch: ω = (φ̇, ψ̇ sin φ, ψ̇ cos φ) in the body.
        let attitude = middle.attitude();
        let angular_rate = add(
            add(
                attitude.rotate_back(add(earth, transport)),
                [phi_rate, rate * phi.sin(), rate * phi.cos()],
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
        let before = self.velocity();
        // The rear axle's middle runs at SPEED; a turn speeds the outer wheel.
        self.wheels[0] += (SPEED + rate * TRACK / 2.0) * dt;
        self.wheels[1] += (SPEED - rate * TRACK / 2.0) * dt;
        self.heading += rate * dt;
        self.second += dt;
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

#[derive(Default)]
struct Summary {
    nees_sum: f64,
    nees_samples: usize,
    nis_sum: f64,
    nis_samples: usize,
    outage_error: f64,
    outage_ellipse: f64,
}

#[derive(Clone, Copy, PartialEq)]
enum Wheels {
    None,
    /// The wheel speed as a number, with noise.
    Speed,
    /// The rear wheels' ticks in `ESF-MEAS` frames.
    Ticks,
}

struct Setup {
    wheels: Wheels,
    outage: Option<Range<usize>>,
}

/// A UBX frame of `ESF-MEAS` with the rear wheels' tick counts. The counts
/// start 3000 ticks short of their 2²³ wrap and the time tag 20 s short of
/// its 2³² wrap, so both wrap within the run.
fn esf_meas(truth: &Truth, tick: usize) -> Vec<u8> {
    let count = |metres: f64| {
        let ticks = (metres / TICK).floor() as u32;
        ((1 << 23) - 3000 + ticks) & 0x007F_FFFF
    };
    let time_tag = (u32::MAX - 20_000).wrapping_add(tick as u32 * 100);
    let mut payload = Vec::new();
    payload.extend_from_slice(&time_tag.to_le_bytes());
    payload.extend_from_slice(&(2u16 << 11).to_le_bytes());
    payload.extend_from_slice(&0u16.to_le_bytes());
    // Data types 8 and 9: rear left, rear right; forward.
    for (data_type, metres) in [(8u32, truth.wheels[0]), (9, truth.wheels[1])] {
        payload.extend_from_slice(&((data_type << 24) | count(metres)).to_le_bytes());
    }
    let (a, b) = kinavis_ubx::checksum(0x10, 0x02, &payload);
    let mut frame = vec![kinavis_ubx::SYNC_1, kinavis_ubx::SYNC_2, 0x10, 0x02];
    frame.extend_from_slice(&(payload.len() as u16).to_le_bytes());
    frame.extend_from_slice(&payload);
    frame.extend_from_slice(&[a, b]);
    frame
}

fn run(seed: u64, setup: &Setup, summary: &mut Summary) {
    let mut rng = Rng::new(seed);
    let noise = ImuNoise::mems();
    let priors = InsPriors::standard();
    let dt = 1.0 / IMU_HZ as f64;
    let start = Instant::<Utc>::from_unix_seconds(1_789_000_000);
    let origin = GeodeticPoint::new(
        Position::from_degrees(LATITUDE, 5.6).unwrap(),
        Height::above_ellipsoid(Distance::ZERO),
    );
    let mut deframer: Deframer<256> = Deframer::new();
    let mut odometer = Odometer::new(
        SpeedSource::RearAxle,
        Distance::from_metres(TICK).unwrap(),
        Duration::from_millis(500),
    )
    .unwrap();
    let mut truth = Truth {
        wheels: [0.0; 2],
        position: [
            10.0 * rng.normal(),
            10.0 * rng.normal(),
            20.0 * rng.normal(),
        ],
        heading: 20f64.to_radians(),
        gyro_bias: rng.normal3(0.05_f64.to_radians()),
        accel_bias: rng.normal3(2e-3 * 9.806_65),
        second: 0.0,
    };
    let velocity_error = rng.normal3(0.5);
    let misalignment = [
        1.0_f64.to_radians() * rng.normal(),
        1.0_f64.to_radians() * rng.normal(),
        3.0_f64.to_radians() * rng.normal(),
    ];
    let nominal_velocity = sub(truth.velocity(), velocity_error);
    let nominal = Strapdown::new(
        start,
        origin,
        Vector3::<Ned, Speed>::new(
            Speed::from_metres_per_second(nominal_velocity[0]).unwrap(),
            Speed::from_metres_per_second(nominal_velocity[1]).unwrap(),
            Speed::from_metres_per_second(nominal_velocity[2]).unwrap(),
        ),
        Quaternion::from_rotation_vector(misalignment).then(&truth.attitude()),
    )
    .unwrap();
    let mut filter = InsFilter::new(nominal, noise, &priors);
    let frame = *filter.nominal().frame();
    let lever = Vector3::<Body, Distance>::new(
        Distance::from_metres(LEVER[0]).unwrap(),
        Distance::from_metres(LEVER[1]).unwrap(),
        Distance::from_metres(LEVER[2]).unwrap(),
    );

    for tick in 1..=SECONDS * IMU_HZ {
        let sample = truth.step(dt, &noise, &mut rng);
        filter.predict(&sample).unwrap();
        let second = tick / IMU_HZ;
        let silent = setup
            .outage
            .as_ref()
            .is_some_and(|outage| outage.contains(&second));
        if setup.wheels != Wheels::None && tick % 2 == 0 {
            let (measured, sigma) = if setup.wheels == Wheels::Speed {
                let speed = SPEED + WHEEL_SIGMA * rng.normal();
                (
                    Some(Speed::from_metres_per_second(speed).unwrap()),
                    WHEEL_SIGMA,
                )
            } else {
                let mut speed = None;
                for byte in esf_meas(&truth, tick) {
                    if let Some(Event::Frame(frame)) = deframer.push(byte) {
                        if let Ok(Message::EsfMeas(meas)) = Message::decode(&frame) {
                            speed = odometer.push(&meas).map(|reading| reading.speed);
                        }
                    }
                }
                (speed, TICK_SIGMA)
            };
            if let Some(measured) = measured {
                filter
                    .update_wheel_speed(
                        measured,
                        1.0,
                        Speed::from_metres_per_second(sigma).unwrap(),
                        GatingPolicy::none(),
                    )
                    .unwrap();
            }
            filter
                .update_non_holonomic(
                    Speed::from_metres_per_second(NHC_SIGMA).unwrap(),
                    Speed::from_metres_per_second(NHC_SIGMA).unwrap(),
                    GatingPolicy::none(),
                )
                .unwrap();
        }
        if !silent && tick % (IMU_HZ / 5) == 0 {
            let antenna = truth.antenna();
            let measured = [
                antenna[0] + RTK_SIGMA * rng.normal(),
                antenna[1] + RTK_SIGMA * rng.normal(),
                antenna[2] + RTK_HEIGHT_SIGMA * rng.normal(),
            ];
            let point = frame
                .point_from_ned(Vector3::<Ned, Distance>::new(
                    Distance::from_metres(measured[0]).unwrap(),
                    Distance::from_metres(measured[1]).unwrap(),
                    Distance::from_metres(measured[2]).unwrap(),
                ))
                .unwrap();
            let update = filter
                .update_antenna_point(
                    point,
                    lever,
                    Distance::from_metres(RTK_SIGMA).unwrap(),
                    Distance::from_metres(RTK_HEIGHT_SIGMA).unwrap(),
                    GatingPolicy::none(),
                )
                .unwrap();
            if second > SETTLED_AFTER {
                summary.nis_sum += update.nis;
                summary.nis_samples += 1;
            }
        }
        if tick % IMU_HZ == 0 && second > SETTLED_AFTER {
            summary.nees_sum += nees(&filter, &truth);
            summary.nees_samples += 1;
        }
        if let Some(outage) = &setup.outage {
            if tick % IMU_HZ == 0 && second + 1 == outage.end {
                let error = sub(truth.position, ned(&filter));
                summary.outage_error += error[0].hypot(error[1]);
                summary.outage_ellipse += filter.horizontal_error().semi_major().metres();
            }
        }
    }
}

fn ned(filter: &InsFilter) -> [f64; 3] {
    let displacement = filter.nominal().displacement();
    [
        displacement.north().metres(),
        displacement.east().metres(),
        displacement.down().metres(),
    ]
}

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

fn summarise(setup: &Setup, seed: u64) -> Summary {
    let mut summary = Summary::default();
    for index in 0..RUNS {
        run(seed + index as u64, setup, &mut summary);
    }
    summary
}

fn assert_consistent(summary: &Summary, label: &str) {
    let mean_nees = summary.nees_sum / summary.nees_samples as f64;
    // The biases decorrelate over minutes: one independent sample per run
    // and minute, at 99.9 %.
    let independent = RUNS * (SECONDS - SETTLED_AFTER) / 60;
    let nees = mean_interval(ERROR_STATE_DIM, independent, 3.29);
    assert!(
        nees.contains(&mean_nees),
        "{label}: NEES {mean_nees:.2} outside {:.2}..{:.2}",
        nees.start,
        nees.end
    );
    let mean_nis = summary.nis_sum / summary.nis_samples as f64;
    let nis = mean_interval(3, summary.nis_samples / 5, 3.29);
    assert!(
        nis.contains(&mean_nis),
        "{label}: fix NIS {mean_nis:.3} outside {:.3}..{:.3}",
        nis.start,
        nis.end
    );
}

#[test]
fn in_the_field_with_rtk_the_filter_is_consistent() {
    let summary = summarise(
        &Setup {
            wheels: Wheels::Speed,
            outage: None,
        },
        11,
    );
    assert_consistent(&summary, "field");
}

#[test]
fn through_thirty_seconds_without_gnss_the_wheels_carry_it() {
    let outage = 150..180;
    let with = summarise(
        &Setup {
            wheels: Wheels::Speed,
            outage: Some(outage.clone()),
        },
        21,
    );
    assert_consistent(&with, "outage with the wheels");
    let without = summarise(
        &Setup {
            wheels: Wheels::None,
            outage: Some(outage),
        },
        21,
    );
    let (with_error, without_error) = (
        with.outage_error / RUNS as f64,
        without.outage_error / RUNS as f64,
    );
    assert!(
        with_error < 3.0 * with.outage_ellipse / RUNS as f64,
        "error {with_error} against the ellipse"
    );
    // 7 cm with the wheels against 1.7 m without, after 30 s.
    assert!(
        without_error > 10.0 * with_error,
        "{without_error} without against {with_error} with"
    );
}

#[test]
fn the_wheel_ticks_of_esf_meas_carry_the_same_outage() {
    let outage = 150..180;
    let ticks = summarise(
        &Setup {
            wheels: Wheels::Ticks,
            outage: Some(outage.clone()),
        },
        21,
    );
    assert_consistent(&ticks, "outage with the wheel ticks");
    let without = summarise(
        &Setup {
            wheels: Wheels::None,
            outage: Some(outage),
        },
        21,
    );
    let (ticks_error, without_error) = (
        ticks.outage_error / RUNS as f64,
        without.outage_error / RUNS as f64,
    );
    assert!(
        ticks_error < 3.0 * ticks.outage_ellipse / RUNS as f64,
        "error {ticks_error} against the ellipse"
    );
    // 6.6 cm from the ticks, as from the speed with its noise (7 cm),
    // against 1.7 m without the wheels.
    assert!(
        without_error > 10.0 * ticks_error,
        "{without_error} without against {ticks_error} with the ticks"
    );
}
