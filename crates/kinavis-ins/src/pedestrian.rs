//! Pedestrian dead reckoning: the stance phase of a foot for zero-velocity
//! updates, steps and their length, and the magnetic heading of a tilted
//! sensor.
//!
//! Two ways to walk without GNSS:
//!
//! - **Foot-mounted IMU**: the full strapdown solution of [`crate::InsFilter`],
//!   held to the ground by a zero-velocity update whenever the foot stands.
//!   [`StanceDetector`] finds those moments by the SHOE test (Skog et al.,
//!   2010): the specific force close to gravity and the angular rate close
//!   to zero over a window, as the `PyShoe` implementation computes it.
//! - **Body-worn or hand-held IMU**: [`StepDetector`] counts steps on the
//!   specific force, [`weinberg_length`] gives the length of each from its
//!   swing, and [`tilt_compensated_heading`] the direction from a
//!   magnetometer.
//!
//! Every threshold is a setting: they depend on the sensor, its mounting and
//! the walker, and are calibrated, not guessed.

use core::time::Duration;

use kinavis_kernel::error::{ensure_range, KernelError, Result};
use kinavis_kernel::{math, Distance, MagneticCourse};

use crate::imu::ImuSample;

/// Longest window of the stance test, samples.
pub const MAX_STANCE_WINDOW: usize = 64;

/// Settings of the SHOE stance test.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StanceConfig {
    window: usize,
    inverse_accel_variance: f64,
    inverse_gyro_variance: f64,
    gravity: f64,
    threshold: f64,
}

impl StanceConfig {
    /// Test over `window` samples, with the accelerometer and gyro noise
    /// `accel_sigma` (m/s²) and `gyro_sigma` (rad/s), local gravity
    /// `gravity` (m/s²), standing when the statistic is below `threshold`.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] for a window outside 1 to
    /// [`MAX_STANCE_WINDOW`], a sigma outside 10⁻⁷ to 10, a gravity outside 9
    /// to 10.5 or a negative or non-finite threshold.
    pub fn new(
        window: usize,
        accel_sigma: f64,
        gyro_sigma: f64,
        gravity: f64,
        threshold: f64,
    ) -> Result<Self> {
        ensure_range(
            "stance window, samples",
            math::count_to_f64(window),
            1.0,
            math::count_to_f64(MAX_STANCE_WINDOW),
        )?;
        ensure_range("accelerometer sigma, m/s²", accel_sigma, 1e-7, 10.0)?;
        ensure_range("gyro sigma, rad/s", gyro_sigma, 1e-7, 10.0)?;
        ensure_range("gravity, m/s²", gravity, 9.0, 10.5)?;
        ensure_range("stance threshold", threshold, 0.0, f64::MAX)?;
        Ok(Self {
            window,
            inverse_accel_variance: 1.0 / (accel_sigma * accel_sigma),
            inverse_gyro_variance: 1.0 / (gyro_sigma * gyro_sigma),
            gravity,
            threshold,
        })
    }

    /// Samples per window.
    #[must_use]
    pub const fn window(&self) -> usize {
        self.window
    }
}

/// One window of the stance test.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Stance {
    /// The SHOE statistic of the window.
    pub statistic: f64,
    /// Whether the foot stood throughout: the statistic under the threshold.
    pub standing: bool,
}

/// Stance detector over consecutive, non-overlapping windows of samples.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StanceDetector {
    config: StanceConfig,
    samples: [[f64; 6]; MAX_STANCE_WINDOW],
    count: usize,
}

impl StanceDetector {
    /// Detector with its settings.
    #[must_use]
    pub const fn new(config: StanceConfig) -> Self {
        Self {
            config,
            samples: [[0.0; 6]; MAX_STANCE_WINDOW],
            count: 0,
        }
    }

    /// Takes a sample; at the end of each window, the test over it, which
    /// holds for each of its samples.
    pub fn push(&mut self, sample: &ImuSample) -> Option<Stance> {
        let [fx, fy, fz] = sample.specific_force();
        let [wx, wy, wz] = sample.angular_rate();
        if let Some(slot) = self.samples.get_mut(self.count) {
            *slot = [fx, fy, fz, wx, wy, wz];
        }
        self.count = self.count.saturating_add(1);
        if self.count < self.config.window {
            return None;
        }
        self.count = 0;
        let window = self.samples.get(..self.config.window).unwrap_or(&[]);
        let n = math::count_to_f64(window.len());
        let mut mean = [0.0; 3];
        for sample in window {
            for (sum, value) in mean.iter_mut().zip(sample) {
                *sum += value / n;
            }
        }
        let norm = math::sqrt(mean.iter().map(|value| value * value).sum());
        if norm.is_nan() || norm <= 0.0 {
            return Some(Stance {
                statistic: f64::INFINITY,
                standing: false,
            });
        }
        let gravity = mean.map(|value| self.config.gravity * value / norm);
        let mut statistic = 0.0;
        for sample in window {
            let [fx, fy, fz, wx, wy, wz] = *sample;
            let accel = squared([fx - gravity[0], fy - gravity[1], fz - gravity[2]]);
            let gyro = squared([wx, wy, wz]);
            statistic += self.config.inverse_accel_variance * accel
                + self.config.inverse_gyro_variance * gyro;
        }
        statistic /= n;
        Some(Stance {
            statistic,
            standing: statistic < self.config.threshold,
        })
    }
}

/// Settings of the step detector.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StepConfig {
    rise: f64,
    fall: f64,
    shortest: Duration,
    longest: Duration,
    gravity: f64,
}

impl StepConfig {
    /// A step starts when the specific force exceeds gravity by `rise` (m/s²)
    /// and ends when it falls back below gravity plus `fall`; steps closer
    /// than `shortest` are one, and a stride longer than `longest` is a
    /// stop, not a step.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] for `rise` outside 0.05 to 20 m/s², `fall`
    /// not below `rise`, `shortest` outside 0.1 to 2 s or `longest` not above
    /// it and at most 10 s, gravity outside 9 to 10.5.
    pub fn new(
        rise: f64,
        fall: f64,
        shortest: Duration,
        longest: Duration,
        gravity: f64,
    ) -> Result<Self> {
        ensure_range("step rise, m/s²", rise, 0.05, 20.0)?;
        ensure_range("step fall, m/s²", fall, -20.0, rise)?;
        ensure_range("shortest step, s", shortest.as_secs_f64(), 0.1, 2.0)?;
        ensure_range(
            "longest step, s",
            longest.as_secs_f64(),
            shortest.as_secs_f64(),
            10.0,
        )?;
        ensure_range("gravity, m/s²", gravity, 9.0, 10.5)?;
        Ok(Self {
            rise,
            fall,
            shortest,
            longest,
            gravity,
        })
    }
}

/// A step, with the extremes of the specific force over it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Step {
    /// Time since the previous step; `None` for the first after a stop.
    pub since_previous: Option<Duration>,
    /// Greatest specific force over the step, m/s².
    pub peak: f64,
    /// Least specific force since the previous step, m/s².
    pub trough: f64,
}

/// Step detector on the magnitude of the specific force.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StepDetector {
    config: StepConfig,
    rising: bool,
    peak: f64,
    trough: f64,
    since_step: Duration,
    stepped: bool,
}

impl StepDetector {
    /// Detector with its settings.
    #[must_use]
    pub const fn new(config: StepConfig) -> Self {
        Self {
            config,
            rising: false,
            peak: 0.0,
            trough: f64::MAX,
            since_step: Duration::ZERO,
            stepped: false,
        }
    }

    /// Takes a sample; a step when one is complete.
    pub fn push(&mut self, sample: &ImuSample) -> Option<Step> {
        let magnitude = math::sqrt(squared(sample.specific_force()));
        self.since_step = self.since_step.saturating_add(sample.over());
        if self.since_step > self.config.longest {
            self.stepped = false;
        }
        let excess = magnitude - self.config.gravity;
        if self.rising {
            self.peak = self.peak.max(magnitude);
            if excess >= self.config.fall {
                return None;
            }
            self.rising = false;
            if self.stepped && self.since_step < self.config.shortest {
                return None;
            }
            let step = Step {
                since_previous: self.stepped.then_some(self.since_step),
                peak: self.peak,
                trough: self.trough,
            };
            self.stepped = true;
            self.since_step = Duration::ZERO;
            self.trough = magnitude;
            return Some(step);
        }
        self.trough = self.trough.min(magnitude);
        if excess > self.config.rise {
            self.rising = true;
            self.peak = magnitude;
        }
        None
    }
}

/// Length of a step from the swing of the specific force over it:
/// `k · (peak − trough)^¼` (Weinberg, 2002), `k` calibrated per walker and
/// mounting.
///
/// # Errors
///
/// [`KernelError::OutOfRange`] for `k` outside 0.01 to 2 or a trough above
/// the peak.
pub fn weinberg_length(step: &Step, k: f64) -> Result<Distance> {
    ensure_range("Weinberg k", k, 0.01, 2.0)?;
    let swing = step.peak - step.trough;
    ensure_range("step swing, m/s²", swing, 0.0, 1e3)?;
    Distance::from_metres(k * math::sqrt(math::sqrt(swing)))
}

/// Magnetic heading of the forward axis from the specific force (for the
/// tilt) and the magnetic field, both in the body frame (forward, right,
/// down), with the sensor at rest or moving steadily.
///
/// Roll and pitch come from the specific force, `φ = atan2(−f_y, −f_z)` and
/// `θ = atan2(f_x, √(f_y² + f_z²))`; the field is levelled with them and its
/// horizontal part gives the heading.
///
/// # Errors
///
/// [`KernelError::Indeterminate`] for a specific force or a horizontal field
/// too small to give a direction; [`KernelError::NotFinite`] otherwise.
pub fn tilt_compensated_heading(
    specific_force: [f64; 3],
    magnetic_field: [f64; 3],
) -> Result<MagneticCourse> {
    let [fx, fy, fz] = specific_force;
    let [mx, my, mz] = magnetic_field;
    if squared(specific_force) < 1e-6 {
        return Err(KernelError::Indeterminate {
            quantity: "tilt without gravity",
        });
    }
    let roll = math::atan2(-fy, -fz);
    let pitch = math::atan2(fx, math::hypot(fy, fz));
    let (sin_roll, cos_roll) = (math::sin(roll), math::cos(roll));
    let (sin_pitch, cos_pitch) = (math::sin(pitch), math::cos(pitch));
    // The field in a frame turned back by roll then pitch: north-ish, east-ish.
    let north = mx * cos_pitch + (my * sin_roll + mz * cos_roll) * sin_pitch;
    let east = my * cos_roll - mz * sin_roll;
    if math::hypot(north, east) < 1e-12 * math::sqrt(squared(magnetic_field)).max(1e-300) {
        return Err(KernelError::Indeterminate {
            quantity: "heading from a vertical field",
        });
    }
    let degrees = math::to_degrees(math::atan2(-east, north));
    Ok(MagneticCourse::from_degrees_wrapped(degrees))
}

fn squared(vector: [f64; 3]) -> f64 {
    vector.iter().map(|value| value * value).sum()
}
