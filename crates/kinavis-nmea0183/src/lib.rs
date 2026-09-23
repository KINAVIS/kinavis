//! NMEA 0183 sentences to and from the KINAVIS types.
//!
//! Anti-corruption layer between receiver sentences (`$GPRMC,...*hh`) and the
//! domain:
//!
//! 1. **Framing**: length, printable characters, start character and a
//!    mandatory checksum. Any failure is an [`NmeaError`].
//! 2. **Decoding** into typed records — [`Rmc`], [`Gga`], [`Gll`], [`Vtg`] —
//!    whose fields are kernel types ([`Position`], [`Speed`], [`TrueCourse`]);
//!    out-of-domain values (latitude 95°) are rejected here. AIS arrives as
//!    [`Vdm`] with the payload still armoured; decoding it is the AIS crate's
//!    job.
//! 3. **Translation** into a [`GnssFix`] via `TryFrom<Rmc>` or [`Gga::fix_on`].
//!
//! Every step works on the input in place: no copies, no allocation, no panics.
//! The crate builds for bare-metal targets without an allocator; CI checks the
//! strict-profile build for panic paths.
//!
//! [`encode`] and `Display` write a record back with its checksum, for
//! generating sentences and for round-trip tests.
//!
//! ```rust
//! use kinavis_nmea0183::{parse, Sentence};
//! use kinavis_kernel::GnssFix;
//!
//! let line = b"$GPRMC,225444.00,A,4916.4500,N,12311.1200,W,3.0,272.5,110926,5.0,W,D*30\r\n";
//! let Sentence::Rmc(rmc) = parse(line)? else { panic!("not an RMC") };
//!
//! let fix = GnssFix::try_from(rmc)?;
//! assert_eq!(format!("{}", fix.taken_at()), "2026-09-11T22:54:44.000 UTC");
//! assert_eq!(format!("{:.2}", fix.position()), "49°16.45'N 123°11.12'W");
//! assert_eq!(fix.speed_over_ground().map(|s| s.knots()), Some(3.0));
//! assert!(fix.quality().fix_type().is_position_fix());
//!
//! // And back out again, checksum recomputed.
//! assert_eq!(format!("{rmc}"), "$GPRMC,225444.00,A,4916.4500,N,12311.1200,W,3.0,272.5,110926,5.0,W,D*30");
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```
//!
//! [`Position`]: kinavis_kernel::Position
//! [`Speed`]: kinavis_kernel::Speed
//! [`TrueCourse`]: kinavis_kernel::TrueCourse
//! [`GnssFix`]: kinavis_kernel::GnssFix
//!
//! # Not supported
//!
//! Other sentences (satellites in view, DOP breakdowns, proprietary) return
//! [`Sentence::Unsupported`] with their address and a verified checksum, for
//! counting or logging.
//!
//! # Feature flags
//!
//! - `std` *(default)* — standard library maths in the kernel.
//! - `libm` — for `no_std` targets: `--no-default-features --features libm`.

#![cfg_attr(not(feature = "std"), no_std)]

mod encode;
mod error;
mod field;
mod frame;
mod sentence;

pub use encode::encode;
pub use error::{NmeaError, TranslationError};
pub use frame::MAX_SENTENCE_BYTES;
pub use sentence::{
    Address, Channel, Date, Gga, Gll, Mode, Payload, Rmc, Sentence, Status, Talker, TimeOfDay, Vdm,
    Vtg, MAX_FRAGMENTS, MAX_PAYLOAD_CHARS,
};

/// Parses one sentence.
///
/// Input: one line from `$` to the checksum, with or without `CR LF`.
///
/// # Errors
///
/// [`NmeaError`] for framing, checksum, unparseable fields or out-of-domain
/// values. A well-formed sentence of an unsupported type is not an error: it
/// returns [`Sentence::Unsupported`].
pub fn parse(sentence: &[u8]) -> Result<Sentence, NmeaError> {
    let frame = frame::Frame::parse(sentence)?;
    Sentence::decode(frame)
}
