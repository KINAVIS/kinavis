//! u-blox UBX binary protocol for the KINAVIS crates.
//!
//! u-blox receivers — the F9 and X20 RTK modules of agricultural guidance,
//! robots and drones, the M8 and M9 of everything else — report in UBX
//! alongside or instead of NMEA: more precise, with the receiver's own
//! accuracy estimates, satellite residuals, RTK status, dual-antenna heading
//! and, on dead-reckoning modules, the inertial and wheel measurements.
//!
//! 1. **Framing**: [`Deframer`] takes bytes from a serial port, one at a time,
//!    and returns checked [`Frame`]s; bytes between frames (NMEA on a mixed
//!    port) come back untouched.
//! 2. **Decoding**: [`Message::decode`] turns a frame into a typed message.
//! 3. **Translation** into the kernel: [`NavPvt::fix`] a [`GnssFix`],
//!    [`NavSat::satellite_set`] a [`SatelliteSet`],
//!    [`NavTimels::leap_seconds`] a [`LeapSeconds`] table.
//!
//! ```rust
//! use kinavis_ubx::{Deframer, Event, Message};
//!
//! # let bytes: &[u8] = &[];
//! let mut deframer: Deframer<1024> = Deframer::new();
//! for &byte in bytes {
//!     if let Some(Event::Frame(frame)) = deframer.push(byte) {
//!         if let Ok(Message::NavPvt(pvt)) = Message::decode(&frame) {
//!             if let Ok(fix) = pvt.fix() {
//!                 println!("{} ±{:.2} m", fix.position(), pvt.horizontal_accuracy.metres());
//!             }
//!         }
//!     }
//! }
//! ```
//!
//! 4. **Configuration**: [`config`] writes `CFG-VALSET`, `CFG-VALGET` and
//!    `CFG-RATE` frames into a caller's buffer and reads the answer.
//! 5. **Odometry**: [`Odometer`] turns the wheel ticks of `ESF-MEAS` into
//!    speed for an inertial filter's wheel-speed update.
//!
//! No allocation, no panics; builds for bare-metal targets.
//!
//! [`GnssFix`]: kinavis_kernel::GnssFix
//! [`SatelliteSet`]: kinavis_kernel::SatelliteSet
//! [`LeapSeconds`]: kinavis_kernel::time::LeapSeconds
//!
//! # Feature flags
//!
//! - `std` *(default)* — standard library maths in the kernel.
//! - `libm` — for `no_std` targets: `--no-default-features --features libm`.

#![cfg_attr(not(feature = "std"), no_std)]

pub mod config;
mod error;

/// Runs the README example as a doctest.
#[cfg(doctest)]
#[doc = include_str!("../README.md")]
pub struct ReadmeExamples;

pub mod esf;
mod frame;
mod message;
pub mod nav;
mod odometer;
mod read;

pub use config::{CfgValget, ConfigError, ConfigKey, Layers, PollLayer, TimeReference};
pub use error::{TranslationError, UbxError};
pub use esf::{EsfIns, EsfMeas, Measurement, Wheel};
pub use frame::{checksum, DeframeError, Deframer, Event, Frame, SYNC_1, SYNC_2};
pub use message::{Message, RxmRawx, RxmSfrbx};
pub use nav::{
    CarrierSolution, NavDop, NavHpposllh, NavPvt, NavRelposned, NavSat, NavStatus, NavTimels,
    PvtFix, ReceiverLeapSeconds, SatInfo,
};
pub use odometer::{Odometer, SpeedSource, WheelSpeed};
