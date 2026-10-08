//! GNSS integrity for the KINAVIS crates: receiver autonomous integrity
//! monitoring (RAIM), fault exclusion and protection levels.
//!
//! A GNSS position can be wrong without the receiver knowing: a satellite
//! broadcasting a bad clock or orbit pulls the fix by tens of metres while the
//! DOP looks fine. With more satellites than unknowns, the ranges overdetermine
//! the fix and disagree when one is faulty. [`assess`] tests that
//! disagreement — the weighted sum of squared range residuals against a
//! chi-square threshold set by the probability of false alarm — and, when the
//! test fails, finds the satellite whose exclusion restores consistency.
//! When it passes, the horizontal and vertical *protection levels* bound the
//! position error that an undetected single fault could cause, with the
//! chosen probability of missed detection.
//!
//! [`predict`] computes the protection levels from geometry alone, before
//! residuals exist: whether integrity will be available for an operation.
//! [`AlertLimits`] turn a report into the kernel's [`NavigationIntegrity`].
//!
//! The satellites come as the kernel's [`SatelliteSet`]: sky positions and
//! residuals from NMEA `GSV`/`GSA`/`GRS` or UBX `NAV-SAT`.
//!
//! # RTK
//!
//! RAIM on code residuals says little about an RTK solution, whose errors are
//! centimetres while the residual test resolves metres. The integrity of RTK
//! is the solution type (fixed or float), the age of the corrections, the
//! receiver's accuracy estimate and agreement with dead reckoning; these
//! protection levels are not an RTK integrity bound.
//!
//! [`NavigationIntegrity`]: kinavis_kernel::NavigationIntegrity
//! [`SatelliteSet`]: kinavis_kernel::SatelliteSet
//!
//! # Feature flags
//!
//! - `std` *(default)* — standard library maths in the kernel.
//! - `libm` — for `no_std` targets: `--no-default-features --features libm`.
//!
//! No allocation; builds for bare-metal targets.

#![cfg_attr(not(feature = "std"), no_std)]

mod raim;

/// Runs the README example as a doctest.
#[cfg(doctest)]
#[doc = include_str!("../README.md")]
pub struct ReadmeExamples;

pub mod stats;

pub use raim::{
    assess, navigation_integrity, predict, protection_from_ellipse, AlertLimits, IntegrityReport,
    Protection, RaimConfig, RangeError, Unavailability, MAX_RAIM_SATELLITES,
};
