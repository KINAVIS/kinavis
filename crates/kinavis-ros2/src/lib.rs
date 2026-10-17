//! ROS 2 for the KINAVIS crates: the messages a navigation node publishes,
//! with the conversions between the kernel's frames and those of ROS.
//!
//! KINAVIS works in NED and FRD; ROS in ENU and FLU (REP-103), with the
//! frames `earth`, `map`, `odom` and `base_link` of REP-105. This crate does
//! the conversion and nothing else (ADR-0020): it mirrors `NavSatFix`, `Imu`,
//! `Odometry` and `GeoPoint` as plain structs that a node copies into its
//! generated message types, and turns kernel values into them and back. The
//! nodes, on `rclrs`, live in the separate repository
//! `KINAVIS/ros2-kinavis`; this crate has no ROS dependency.
//!
//! - **Vectors**: NED → ENU `(e, n, −d)`, FRD → FLU `(x, −y, −z)`.
//! - **Orientation**: FLU in ENU, `q(NED→ENU) ⊗ q(FRD→NED) ⊗ q(FLU→FRD)`;
//!   roll kept, pitch negated, yaw `90° − heading`.
//! - **Covariances** only from published quantities: an error ellipse, a
//!   height or heading sigma; `−1` where `Imu` asks for it, a large variance
//!   where the message has no such marker.
//! - **Height** above the ellipsoid only: a height above sea level is
//!   refused, never silently passed as one.
//! - **Time**: UTC instants as Unix seconds and nanoseconds.
//!
//! ```rust
//! use kinavis_kernel::{Angle, TrueCourse};
//! use kinavis_ros2::{attitude, orientation};
//!
//! // Heading east, level: in ROS, yaw 0 (x east).
//! let q = orientation(Angle::ZERO, Angle::ZERO, TrueCourse::new(90.0)?);
//! assert!(q.w > 0.999_999 && q.z.abs() < 1e-9);
//! let (_, _, heading) = attitude(q)?;
//! assert!((heading.degrees() - 90.0).abs() < 1e-9);
//! # Ok::<(), kinavis_kernel::KernelError>(())
//! ```
//!
//! # Feature flags
//!
//! - `std` *(default)* — standard library maths in the kernel.
//! - `libm` — for `no_std` targets: `--no-default-features --features libm`.
//!
//! No allocation, no panics on any input.

#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(test)]
extern crate std;

/// Runs the README example as a doctest.
#[cfg(doctest)]
#[doc = include_str!("../README.md")]
pub struct ReadmeExamples;

mod convert;
mod messages;

pub use convert::{
    attitude, enu, flu, frd, from_nav_sat_fix, geo_point, imu, instant, nav_sat_fix, odometry,
    orientation, stamp, ImuReading, OdometryReading, BASE_LINK, EARTH, MAP, ODOM, UNKNOWN_VARIANCE,
};
pub use messages::{
    covariance_type, nav_sat_status, GeoPoint, Imu, NavSatFix, Odometry, Quaternion, Stamp, Vector,
};
