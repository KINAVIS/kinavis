//! AIS messages from their NMEA sentences to the KINAVIS types.
//!
//! A receiver delivers an AIS message as one or more `!AIVDM` sentences, 6 bits
//! per character. [`kinavis-nmea0183`] validates each sentence and yields a
//! [`Vdm`] with the payload still armoured; this crate:
//!
//! 1. **Unarmours** the payload into [`Bits`], a fixed-capacity buffer read by
//!    bit offset and width as in the standard's tables.
//! 2. **Reassembles** multi-sentence messages in an [`Assembler`] with fixed
//!    slots and a timeout, so a lost fragment costs one message, not a slot.
//! 3. **Decodes** into a [`Message`] with kernel field types: [`Position`],
//!    [`Speed`], [`TrueCourse`], [`TargetId`] for the MMSI, [`Distance`] for
//!    draught, [`InlineStr`] for names. "Not available" is `None`, never a zero
//!    or a string of `@`.
//!
//! No allocation, no panics on any input; builds for bare-metal targets and CI
//! checks the strict-profile build for panic paths.
//!
//! ```rust
//! use kinavis_ais::{Assembler, Message};
//! use kinavis_kernel::{Instant, Utc};
//! use kinavis_nmea0183::{parse, Sentence};
//!
//! let mut assembler = Assembler::new();
//! let now = Instant::<Utc>::from_unix_seconds(1_789_000_000);
//!
//! let line = b"!AIVDM,1,1,,A,13aEOK?P00PD2wVMdLDRhgvL289?,0*26\r\n";
//! let Sentence::Vdm(vdm) = parse(line)? else { panic!("not an AIS sentence") };
//! let Some(bits) = assembler.push(&vdm, now)? else { panic!("in fragments") };
//! let Message::PositionReport(report) = Message::decode(&bits)? else { panic!("not a position") };
//!
//! assert_eq!(report.mmsi.number(), 244_670_316);
//! assert_eq!(format!("{:.3}", report.position.unwrap()), "51°53.685'N 004°22.757'E");
//! assert_eq!(report.course.map(|c| c.degrees()), Some(70.6));
//! assert_eq!(report.speed.map(|s| s.knots()), Some(0.0));
//! assert_eq!(report.heading, None, "not available, not north");
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```
//!
//! The caller builds the `TargetObservation` for the traffic picture from these
//! fields; this crate depends only on the kernel and the sentence crate.
//!
//! [`kinavis-nmea0183`]: kinavis_nmea0183
//! [`Position`]: kinavis_kernel::Position
//! [`Speed`]: kinavis_kernel::Speed
//! [`TrueCourse`]: kinavis_kernel::TrueCourse
//! [`TargetId`]: kinavis_kernel::TargetId
//! [`Distance`]: kinavis_kernel::Distance
//!
//! # Supported messages
//!
//! | Message | Decoded as |
//! |---|---|
//! | 1, 2, 3 — class A position report | [`PositionReport`] |
//! | 5 — class A static and voyage data | [`StaticAndVoyageData`] |
//! | 18 — class B position report | [`PositionReport`] |
//! | 19 — class B extended position report | [`PositionReport`], with name, type and dimensions |
//! | 21 — aid to navigation | [`AidToNavigation`] |
//! | 24 — class B static data report, part A or B | [`StaticDataReport`] |
//!
//! Other well-formed messages decode as [`Message::Unsupported`] with the
//! message type and the raw [`Bits`].
//!
//! # Feature flags
//!
//! - `std` *(default)* — standard library maths in the kernel.
//! - `libm` — for `no_std` targets: `--no-default-features --features libm`.
//! - `serde` — serialisation of the messages.

#![cfg_attr(not(feature = "std"), no_std)]

mod assembler;
mod aton;
mod bits;
mod error;
mod fields;
mod message;
mod static_data;
mod vessel;

pub use assembler::{Assembler, DEFAULT_TIMEOUT, MAX_ASSEMBLIES};
pub use aton::{AidToNavigation, AidType, Mark, Quadrant};
pub use bits::{Bits, MAX_MESSAGE_BITS};
pub use error::AisError;
pub use message::{Message, NavigationStatus, PositionReport, StationClass, Turn};
pub use static_data::{
    ClassBStaticData, Eta, StaticAndVoyageData, StaticDataPart, StaticDataReport,
};
pub use vessel::{Dimensions, HazardCategory, PositionFixingDevice, ShipCategory, ShipType};

// Re-exported so callers need not depend on the NMEA crate for the sentence
// type or on the kernel for the string type.
pub use kinavis_kernel::InlineStr;
pub use kinavis_nmea0183::{Channel, Vdm};

/// Runs the `README.md` example as a doctest.
#[cfg(doctest)]
#[doc = include_str!("../README.md")]
pub struct ReadmeExamples;
