//! NMEA 2000 parameter groups from CAN frames to the KINAVIS types.
//!
//! Every NMEA 2000 device (GNSS, compass, sounder, AIS) sends *parameter
//! groups*: fixed layouts of little-endian fields, carried in one 8-byte frame
//! or, as a *fast packet*, in up to 32. Starting from frames delivered by the
//! bus interface, this crate:
//!
//! 1. **Parses the identifier** — [`CanId`] — into the [`Pgn`] and source
//!    address.
//! 2. **Reassembles fast packets** in an [`Assembler`] with fixed slots and a
//!    timeout, so a lost frame costs one group, not a slot.
//! 3. **Decodes** the group into a [`Message`] with kernel field types
//!    ([`Position`], [`Speed`], [`TrueCourse`], [`Distance`], [`Instant`]).
//!    "Not available" is `None`, not the raw all-ones value.
//!
//! No allocation, no panics on any input; builds for bare-metal targets and CI
//! checks the strict-profile build for panic paths. The bus (CAN controller,
//! driver, address claim) is the caller's.
//!
//! ```rust
//! use kinavis_nmea2000::{Assembler, CanId, Frame, Message, Pgn};
//! use kinavis_kernel::{Instant, Utc};
//!
//! let mut assembler = Assembler::new();
//! let now = Instant::<Utc>::from_unix_seconds(1_789_000_000);
//!
//! // Position, rapid update, from address 35: 50.755°N 1.333°W in 1e-7°.
//! let id = CanId::new(0x09F8_0123)?;
//! assert_eq!(id.pgn(), Pgn::POSITION_RAPID_UPDATE);
//! let frame = Frame::new(id, &[0x30, 0x99, 0x40, 0x1E, 0xB0, 0x99, 0x34, 0xFF])?;
//!
//! let Some(payload) = assembler.push(&frame, now)? else { panic!("in frames") };
//! let Message::PositionRapidUpdate(update) = Message::decode(&payload)? else {
//!     panic!("not a position");
//! };
//! assert_eq!(format!("{:.3}", update.position.unwrap()), "50°45.300'N 001°19.980'W");
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```
//!
//! [`Position`]: kinavis_kernel::Position
//! [`Speed`]: kinavis_kernel::Speed
//! [`TrueCourse`]: kinavis_kernel::TrueCourse
//! [`Distance`]: kinavis_kernel::Distance
//! [`Instant`]: kinavis_kernel::Instant
//!
//! # Supported PGNs
//!
//! | Group | Decoded as |
//! |---|---|
//! | 129025 — position, rapid update | [`PositionRapidUpdate`] |
//! | 129026 — COG and SOG, rapid update | [`CourseAndSpeed`] |
//! | 129029 — GNSS position data | [`GnssPosition`], and a kernel `GnssFix` from it |
//! | 127250 — vessel heading | [`VesselHeading`] |
//! | 128267 — water depth | [`WaterDepth`] |
//! | 129038 — AIS class A position report | [`AisPositionReport`] |
//! | 129039 — AIS class B position report | [`AisPositionReport`] |
//!
//! Other PGNs pass the assembler only with an explicit transport
//! ([`Assembler::push_as`]) and decode as [`Message::Unsupported`] with the PGN
//! and the raw [`Payload`].
//!
//! # Feature flags
//!
//! - `std` *(default)* — standard library maths in the kernel.
//! - `libm` — for `no_std` targets: `--no-default-features --features libm`.
//! - `serde` — serialisation of the messages.

#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(test)]
extern crate std;

mod ais;
mod assembler;
mod error;
mod fields;
mod frame;
mod gnss;
mod id;
mod message;
mod navigation;

pub use ais::{AisPositionReport, StationClass};
pub use assembler::{Assembler, DEFAULT_TIMEOUT, MAX_ASSEMBLIES};
pub use error::Nmea2000Error;
pub use frame::{Frame, Payload, MAX_PAYLOAD_BYTES};
pub use gnss::{GnssPosition, GnssSystem, Integrity};
pub use id::{CanId, Pgn, Transport};
pub use message::Message;
pub use navigation::{CourseAndSpeed, PositionRapidUpdate, Referenced, VesselHeading, WaterDepth};

/// Runs the `README.md` example as a doctest.
#[cfg(doctest)]
#[doc = include_str!("../README.md")]
pub struct ReadmeExamples;
