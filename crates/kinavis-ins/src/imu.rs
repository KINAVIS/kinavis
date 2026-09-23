//! IMU samples and noise model.

use core::time::Duration;

use kinavis_kernel::error::{ensure_finite, ensure_range, Result};
use kinavis_kernel::math;

/// One IMU sample: angular rate and specific force in the body frame, averaged
/// over the interval since the previous sample.
///
/// Angular rate about forward, right, down in rad/s; specific force along them
/// in m/s². At rest the specific force is minus gravity, about `[0, 0, −9.8]`
/// (down positive).
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(try_from = "StoredImuSample", into = "StoredImuSample")
)]
pub struct ImuSample {
    angular_rate: [f64; 3],
    specific_force: [f64; 3],
    over: Duration,
}

impl ImuSample {
    /// Sample from angular rate, specific force and interval.
    ///
    /// # Errors
    ///
    /// [`kinavis_kernel::KernelError::NotFinite`] for a non-finite component;
    /// [`kinavis_kernel::KernelError::OutOfRange`] for an interval of zero or
    /// over one second.
    pub fn new(angular_rate: [f64; 3], specific_force: [f64; 3], over: Duration) -> Result<Self> {
        for value in angular_rate {
            ensure_finite("angular rate", value)?;
        }
        for value in specific_force {
            ensure_finite("specific force", value)?;
        }
        ensure_range("IMU interval", over.as_secs_f64(), 1e-6, 1.0)?;
        Ok(Self {
            angular_rate,
            specific_force,
            over,
        })
    }

    /// Angular rate about forward, right, down, rad/s.
    #[must_use]
    pub const fn angular_rate(&self) -> [f64; 3] {
        self.angular_rate
    }

    /// Specific force along forward, right, down, m/s².
    #[must_use]
    pub const fn specific_force(&self) -> [f64; 3] {
        self.specific_force
    }

    /// Sample interval.
    #[must_use]
    pub const fn over(&self) -> Duration {
        self.over
    }

    /// Sample interval, s.
    pub(crate) fn seconds(&self) -> f64 {
        self.over.as_secs_f64()
    }
}

/// IMU error model from the datasheet: white noise per channel and bias random
/// walk.
///
/// White noise is given as random walk (1σ angle or velocity accumulated over 1
/// s); bias instability is modelled as a random walk (1σ change per √s), as
/// used by the error-state filter.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(try_from = "StoredImuNoise", into = "StoredImuNoise")
)]
pub struct ImuNoise {
    /// rad/√s.
    arw: f64,
    /// m/s/√s.
    vrw: f64,
    /// rad/s/√s.
    gyro_bias: f64,
    /// m/s²/√s.
    accel_bias: f64,
}

impl ImuNoise {
    /// Noise model from four 1σ-per-√s figures: angle random walk (rad),
    /// velocity random walk (m/s), gyro bias walk (rad/s), accelerometer bias
    /// walk (m/s²).
    ///
    /// # Errors
    ///
    /// [`kinavis_kernel::KernelError::OutOfRange`] for a figure that is not
    /// positive and finite.
    pub fn new(
        angle_random_walk: f64,
        velocity_random_walk: f64,
        gyro_bias_walk: f64,
        accel_bias_walk: f64,
    ) -> Result<Self> {
        ensure_range(
            "angle random walk",
            angle_random_walk,
            f64::MIN_POSITIVE,
            1.0,
        )?;
        ensure_range(
            "velocity random walk",
            velocity_random_walk,
            f64::MIN_POSITIVE,
            10.0,
        )?;
        ensure_range("gyro bias walk", gyro_bias_walk, f64::MIN_POSITIVE, 1.0)?;
        ensure_range(
            "accelerometer bias walk",
            accel_bias_walk,
            f64::MIN_POSITIVE,
            10.0,
        )?;
        Ok(Self {
            arw: angle_random_walk,
            vrw: velocity_random_walk,
            gyro_bias: gyro_bias_walk,
            accel_bias: accel_bias_walk,
        })
    }

    /// Consumer MEMS: ARW 0.3°/√h, VRW 0.1 m/s/√h, bias walk 10°/h and 1 mg per
    /// hour.
    #[must_use]
    pub fn mems() -> Self {
        Self {
            arw: math::to_radians(0.3) / 60.0,
            vrw: 0.1 / 60.0,
            gyro_bias: math::to_radians(10.0) / 3600.0 / 60.0,
            accel_bias: 1e-3 * 9.80665 / 60.0,
        }
    }

    /// Tactical-grade FOG: ARW 0.05°/√h, VRW 0.03 m/s/√h, bias walk 1°/h and
    /// 0.1 mg per hour.
    #[must_use]
    pub fn tactical() -> Self {
        Self {
            arw: math::to_radians(0.05) / 60.0,
            vrw: 0.03 / 60.0,
            gyro_bias: math::to_radians(1.0) / 3600.0 / 60.0,
            accel_bias: 1e-4 * 9.80665 / 60.0,
        }
    }

    /// Angle random walk, rad/√s.
    #[must_use]
    pub const fn angle_random_walk(&self) -> f64 {
        self.arw
    }

    /// Velocity random walk, m/s/√s.
    #[must_use]
    pub const fn velocity_random_walk(&self) -> f64 {
        self.vrw
    }

    /// Gyro bias walk, rad/s/√s.
    #[must_use]
    pub const fn gyro_bias_walk(&self) -> f64 {
        self.gyro_bias
    }

    /// Accelerometer bias walk, m/s²/√s.
    #[must_use]
    pub const fn accel_bias_walk(&self) -> f64 {
        self.accel_bias
    }
}

/// Serialised form. Deserialisation goes through [`ImuSample::new`], rejecting
/// `NaN` rates and invalid intervals.
#[cfg(feature = "serde")]
#[derive(serde::Serialize, serde::Deserialize)]
struct StoredImuSample {
    angular_rate: [f64; 3],
    specific_force: [f64; 3],
    over: Duration,
}

#[cfg(feature = "serde")]
impl TryFrom<StoredImuSample> for ImuSample {
    type Error = kinavis_kernel::KernelError;

    fn try_from(stored: StoredImuSample) -> Result<Self> {
        Self::new(stored.angular_rate, stored.specific_force, stored.over)
    }
}

#[cfg(feature = "serde")]
impl From<ImuSample> for StoredImuSample {
    fn from(sample: ImuSample) -> Self {
        Self {
            angular_rate: sample.angular_rate,
            specific_force: sample.specific_force,
            over: sample.over,
        }
    }
}

/// Serialised form. Deserialisation goes through [`ImuNoise::new`], rejecting
/// zero or negative figures (singular noise matrix).
#[cfg(feature = "serde")]
#[derive(serde::Serialize, serde::Deserialize)]
struct StoredImuNoise {
    arw: f64,
    vrw: f64,
    gyro_bias: f64,
    accel_bias: f64,
}

#[cfg(feature = "serde")]
impl TryFrom<StoredImuNoise> for ImuNoise {
    type Error = kinavis_kernel::KernelError;

    fn try_from(stored: StoredImuNoise) -> Result<Self> {
        Self::new(stored.arw, stored.vrw, stored.gyro_bias, stored.accel_bias)
    }
}

#[cfg(feature = "serde")]
impl From<ImuNoise> for StoredImuNoise {
    fn from(noise: ImuNoise) -> Self {
        Self {
            arw: noise.arw,
            vrw: noise.vrw,
            gyro_bias: noise.gyro_bias,
            accel_bias: noise.accel_bias,
        }
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::float_cmp)]

    use super::*;

    #[test]
    fn a_sample_is_finite_and_of_a_sensible_interval() {
        let sample = ImuSample::new(
            [0.0, 0.0, 0.01],
            [0.0, 0.0, -9.8],
            Duration::from_millis(10),
        )
        .unwrap();
        assert_eq!(sample.angular_rate(), [0.0, 0.0, 0.01]);
        assert_eq!(sample.specific_force(), [0.0, 0.0, -9.8]);
        assert_eq!(sample.over(), Duration::from_millis(10));
        assert!(ImuSample::new([f64::NAN, 0.0, 0.0], [0.0; 3], Duration::from_millis(10)).is_err());
        assert!(ImuSample::new(
            [0.0; 3],
            [0.0, f64::INFINITY, 0.0],
            Duration::from_millis(10)
        )
        .is_err());
        assert!(ImuSample::new([0.0; 3], [0.0; 3], Duration::ZERO).is_err());
        assert!(ImuSample::new([0.0; 3], [0.0; 3], Duration::from_secs(2)).is_err());
    }

    #[test]
    fn the_presets_are_in_the_units_the_sheets_use() {
        let mems = ImuNoise::mems();
        // 0.3°/√h = 0.005°/√s.
        assert!((math::to_degrees(mems.angle_random_walk()) - 0.005).abs() < 1e-12);
        assert!(ImuNoise::tactical().angle_random_walk() < mems.angle_random_walk());
        assert!(ImuNoise::tactical().gyro_bias_walk() < mems.gyro_bias_walk());
        assert!(ImuNoise::new(1e-4, 1e-3, 1e-7, 1e-5).is_ok());
        assert!(ImuNoise::new(0.0, 1e-3, 1e-7, 1e-5).is_err());
        assert!(ImuNoise::new(1e-4, f64::NAN, 1e-7, 1e-5).is_err());
        let custom = ImuNoise::new(1e-4, 1e-3, 1e-7, 1e-5).unwrap();
        assert_eq!(custom.velocity_random_walk(), 1e-3);
        assert_eq!(custom.accel_bias_walk(), 1e-5);
    }
}
