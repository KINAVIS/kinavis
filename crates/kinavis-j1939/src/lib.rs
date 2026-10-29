//! SAE J1939 and ISO 11783-3 for the KINAVIS crates: the 29-bit identifier,
//! NAME and address claim, the transport protocols.
//!
//! J1939 is the CAN framing of trucks, tractors (ISO 11783, ISOBUS) and
//! ships (NMEA 2000): a 29-bit identifier with a priority, a *parameter
//! group number* and the sender's address. This crate covers what every
//! such network shares:
//!
//! 1. **Identifier and frame**: [`CanId`], [`Pgn`], [`Frame`].
//! 2. **NAME and address claim**: [`Name`], and [`AddressClaimer`], the
//!    claim of ISO 11783-5 as a state machine.
//! 3. **Transport protocol** for 9 to 1785 bytes: [`TransportReceiver`]
//!    (broadcast and connections, as a participant or a bus monitor),
//!    [`broadcast`] and [`TransportSender`]; **extended transport** for more,
//!    [`ExtendedReceiver`] and [`ExtendedSender`].
//!
//! Each is a state machine over frames and instants: frames from the bus go
//! in with the time they arrived, frames to send come out. The CAN
//! controller, the sending and the clock are the caller's; timeouts are
//! instants passed in, not "now".
//!
//! ```rust
//! use kinavis_j1939::{CanId, Frame, Name, Pgn, TransportReceiver};
//! use kinavis_kernel::{Instant, Utc};
//!
//! // A tractor's address claim, as candump prints it: 18EEFF47#4D74300400340220.
//! let claim = Frame::new(CanId::new(0x18EE_FF47)?, &[0x4D, 0x74, 0x30, 0x04, 0x00, 0x34, 0x02, 0x20])?;
//! assert_eq!(claim.id.pgn(), Pgn::ADDRESS_CLAIMED);
//! let name = Name::from_bytes(claim.data().try_into()?);
//! assert_eq!(name.fields().industry_group, 2); // agriculture and forestry
//!
//! // A broadcast of 9 bytes in two packets, heard by a bus monitor.
//! let mut receiver = TransportReceiver::<4, 1785>::new(None);
//! let now = Instant::<Utc>::from_unix_seconds(1_547_753_511);
//! let mut done = None;
//! kinavis_j1939::broadcast(Pgn::new(65_242), 0x00, b"KINAVIS 1", |frame| {
//!     done = done.or(receiver.push(&frame, now).transfer);
//! })?;
//! assert_eq!(done.map(|transfer| transfer.data().to_vec()), Some(b"KINAVIS 1".to_vec()));
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```
//!
//! # Feature flags
//!
//! - `std` *(default)* — standard library in the kernel.
//! - `libm` — for `no_std` targets: `--no-default-features --features libm`.
//! - `serde` — serialisation of identifiers, frames and NAMEs.
//!
//! No allocation, no panics on any input.

#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(test)]
extern crate std;

/// Runs the README example as a doctest.
#[cfg(doctest)]
#[doc = include_str!("../README.md")]
pub struct ReadmeExamples;

mod claim;
mod error;
mod extended;
mod frame;
mod id;
mod name;
mod transport;

pub use claim::{request, AddressClaimer, ClaimState, CONTENTION_TIME, FIRST_DYNAMIC_ADDRESS};
pub use error::J1939Error;
pub use extended::{ExtendedReceiver, ExtendedSender, ETP_MAX_BYTES};
pub use frame::Frame;
pub use id::{CanId, Pgn};
pub use name::{Name, NameFields};
pub use transport::{
    abort, broadcast, AbortReason, Received, SenderState, Transfer, TransportReceiver,
    TransportSender, T1, T2, T3, T4, TP_MAX_BYTES,
};
