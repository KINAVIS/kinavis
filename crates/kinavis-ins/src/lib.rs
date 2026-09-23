//! Strapdown inertial navigation over the KINAVIS kernel.
//!
//! Integrating IMU rate and specific force gives attitude, velocity and
//! position without external aids, but sensor biases integrate too: 1°/h of
//! gyro drift becomes about a mile of position error in half an hour. The
//! system therefore has two parts:
//!
//! 1. **Mechanisation**, [`Strapdown`]: NED integration including Earth rate,
//!    transport rate, gravity and Coriolis.
//! 2. **Error-state filter**, [`InsFilter`]: 15 states (position, velocity,
//!    attitude, gyro bias, accelerometer bias errors) estimated from available
//!    aiding — GNSS position or velocity, heading, zero-velocity — and fed back
//!    into the mechanisation to bound drift.
//!
//! Outputs are kernel types: [`Position`], a [`Vector3`] of [`Speed`], an
//! [`Attitude`] with yaw as [`TrueCourse`], plus sigmas and the error ellipse;
//! never the covariance matrix. [`InsMotion`] is a process model for the
//! six-state estimator in `kinavis`, propagating position between fixes with
//! the INS velocity.
//!
//! No allocation, no panics on any input; builds for bare-metal targets and CI
//! checks the strict-profile build for panic paths. Filter consistency is
//! verified by Monte Carlo tests: over a passage with a turn, NEES (15 states)
//! and NIS of position fixes fall within their χ² intervals.
//!
//! ```rust
//! use core::time::Duration;
//! use kinavis_ins::{
//!     gravity_down, GatingPolicy, ImuNoise, ImuSample, InsFilter, InsPriors, Quaternion,
//!     Strapdown, EARTH_RATE,
//! };
//! use kinavis_kernel::{
//!     Angle, Distance, GeodeticPoint, Height, Instant, Ned, Speed, TrueCourse, Utc, Vector3,
//! };
//!
//! // Alongside at 50°45.3'N, heading 037° by the gyrocompass, level.
//! let start = Instant::<Utc>::from_unix_seconds(1_789_000_000);
//! let berth = GeodeticPoint::new(
//!     "50°45.3'N 001°20.0'W".parse()?,
//!     Height::above_ellipsoid(Distance::ZERO),
//! );
//! let attitude = Quaternion::from_euler(Angle::ZERO, Angle::ZERO, TrueCourse::new(37.0)?);
//! let still = Vector3::<Ned, Speed>::new(Speed::ZERO, Speed::ZERO, Speed::ZERO);
//! let mut ins = InsFilter::new(
//!     Strapdown::new(start, berth, still, attitude)?,
//!     ImuNoise::mems(),
//!     &InsPriors::standard(),
//! );
//!
//! // What the IMU reads at rest: the Earth turning, and minus gravity.
//! let latitude = 50.755_f64.to_radians();
//! let earth = [EARTH_RATE * latitude.cos(), 0.0, -EARTH_RATE * latitude.sin()];
//! let gravity = [0.0, 0.0, -gravity_down(latitude.sin(), 0.0)];
//! let sample = ImuSample::new(
//!     attitude.rotate_back(earth),
//!     attitude.rotate_back(gravity),
//!     Duration::from_millis(100),
//! )?;
//!
//! // A second of samples; then the gyrocompass says 037.2°, and the
//! // mooring lines say the vessel is not moving.
//! for _ in 0..10 {
//!     ins.predict(&sample)?;
//! }
//! ins.update_heading(TrueCourse::new(37.2)?, Angle::from_degrees(0.5)?, GatingPolicy::none())?;
//! ins.update_zero_velocity(Speed::from_metres_per_second(0.02)?, GatingPolicy::none())?;
//!
//! assert!((ins.attitude().yaw.degrees() - 37.2).abs() < 0.1);
//! assert!(ins.heading_sigma().degrees() < 0.5);
//! assert!(ins.velocity().magnitude().metres_per_second() < 0.02);
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```
//!
//! [`Position`]: kinavis_kernel::Position
//! [`Vector3`]: kinavis_kernel::Vector3
//! [`Speed`]: kinavis_kernel::Speed
//! [`TrueCourse`]: kinavis_kernel::TrueCourse
//!
//! # Not implemented
//!
//! - Initial alignment (levelling on gravity, gyrocompassing on Earth rate):
//!   the caller supplies the initial attitude (gyrocompass and level, or a
//!   previous run) and the filter refines it.
//! - IMU-to-antenna lever arm.
//! - Coning and sculling corrections: a marine IMU at tens of hertz on a slow
//!   vessel is within tolerance without them.
//!
//! # Feature flags
//!
//! - `std` *(default)* — standard library maths in the kernel.
//! - `libm` — for `no_std` targets: `--no-default-features --features libm`.
//! - `serde` — serialisation of the value types.

#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(test)]
extern crate std;

mod attitude;
mod filter;
mod imu;
mod mechanisation;
mod motion;

pub use attitude::{Attitude, Quaternion};
pub use filter::{InsFilter, InsPriors, InsUpdate, ERROR_STATE_DIM};
pub use imu::{ImuNoise, ImuSample};
pub use mechanisation::{gravity_down, Strapdown, EARTH_RATE};
pub use motion::InsMotion;

// Re-exported so callers need not depend on the kernel for the gate type.
pub use kinavis_kernel::estimation::GatingPolicy;

/// Runs the `README.md` example as a doctest.
#[cfg(doctest)]
#[doc = include_str!("../README.md")]
pub struct ReadmeExamples;
