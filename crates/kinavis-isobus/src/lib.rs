//! ISOBUS (ISO 11783-7) for the KINAVIS crates: the speed of the tractor and
//! the messages between a guidance system and the steering.
//!
//! ISOBUS runs on J1939 framing (`kinavis-j1939`). This crate decodes and
//! encodes the parameter groups a guidance system reads and sends:
//!
//! | PGN | Message |
//! |---|---|
//! | 65096 (`FE48`) | [`WheelBasedSpeed`] — speed and distance from the wheels or the tail shaft |
//! | 65097 (`FE49`) | [`GroundBasedSpeed`] — speed and distance from a radar or another ground sensor |
//! | 61474 (`F022`) | [`MachineSelectedSpeed`] — the speed the tractor chose as best, with its source |
//! | 64835 (`FD43`) | [`MachineSelectedSpeedCommand`] — a speed set point for the tractor |
//! | 44032 (`AC00`) | [`GuidanceMachineInfo`] — the steering's estimated curvature and readiness |
//! | 44288 (`AD00`) | [`GuidanceSystemCommand`] — the curvature the guidance system commands |
//!
//! Field layouts are those of ISO 11783-7, checked value for value against
//! `AgIsoStack++` (MIT) because the standard itself is not public; fields that
//! could not be checked are not decoded. "Not available" and "error" are
//! `None` or their own variant, not the raw all-ones values. Positions on
//! ISOBUS are NMEA 2000 groups (129025, 129029): `kinavis-nmea2000`.
//!
//! Curvature is right positive (turning to the driver's right when moving
//! forward), as in `kinavis-agri`; per metre here, km⁻¹ on the bus.
//!
//! ```rust
//! use kinavis_isobus::{Direction, Message};
//! use kinavis_j1939::{CanId, Frame};
//!
//! // Ground-based speed from address 0x47: 3.6 m/s forward, distance 0.
//! let frame = Frame::new(CanId::new(0x0CFE_4947)?, &[0x10, 0x0E, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFD])?;
//! let Some(Message::GroundBasedSpeed(speed)) = Message::decode(&frame)? else {
//!     panic!("a ground-based speed")
//! };
//! assert_eq!(speed.speed.map(|v| v.metres_per_second()), Some(3.6));
//! assert_eq!(speed.direction, Direction::Forward);
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```
//!
//! # Feature flags
//!
//! - `std` *(default)* — standard library in the kernel.
//! - `libm` — for `no_std` targets: `--no-default-features --features libm`.
//! - `serde` — serialisation of the messages.
//!
//! No allocation, no panics on any input.

#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(test)]
extern crate std;

/// Runs the README example as a doctest.
#[cfg(doctest)]
#[doc = include_str!("../README.md")]
pub struct ReadmeExamples;

mod guidance;
mod slots;
mod speed;

use core::fmt;

use kinavis_j1939::{Frame, Pgn};

pub use guidance::{GuidanceMachineInfo, GuidanceSystemCommand, LimitStatus, MAX_CURVATURE};
pub use slots::{Direction, Indicator};
pub use speed::{
    GroundBasedSpeed, MachineSelectedSpeed, MachineSelectedSpeedCommand, SpeedSource,
    WheelBasedSpeed,
};

/// Why a frame could not be decoded.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum IsobusError {
    /// A message of eight bytes in fewer.
    TooShort {
        /// PGN.
        pgn: Pgn,
        /// Bytes received.
        len: usize,
    },
}

impl fmt::Display for IsobusError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::TooShort { pgn, len } => write!(f, "{pgn}: {len} bytes where 8 are needed"),
        }
    }
}

impl core::error::Error for IsobusError {}

/// A decoded ISOBUS message.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Message {
    /// PGN 65096.
    WheelBasedSpeed(WheelBasedSpeed),
    /// PGN 65097.
    GroundBasedSpeed(GroundBasedSpeed),
    /// PGN 61474.
    MachineSelectedSpeed(MachineSelectedSpeed),
    /// PGN 64835.
    MachineSelectedSpeedCommand(MachineSelectedSpeedCommand),
    /// PGN 44032.
    GuidanceMachineInfo(GuidanceMachineInfo),
    /// PGN 44288.
    GuidanceSystemCommand(GuidanceSystemCommand),
}

impl Message {
    /// Decodes a frame; `None` for a parameter group this crate does not
    /// know.
    ///
    /// # Errors
    ///
    /// [`IsobusError::TooShort`] for a known group in fewer than 8 bytes.
    pub fn decode(frame: &Frame) -> Result<Option<Self>, IsobusError> {
        let pgn = frame.id.pgn();
        let known = matches!(
            pgn.number(),
            WheelBasedSpeed::PGN_NUMBER
                | GroundBasedSpeed::PGN_NUMBER
                | MachineSelectedSpeed::PGN_NUMBER
                | MachineSelectedSpeedCommand::PGN_NUMBER
                | GuidanceMachineInfo::PGN_NUMBER
                | GuidanceSystemCommand::PGN_NUMBER
        );
        if !known {
            return Ok(None);
        }
        let bytes: [u8; 8] = frame.data().try_into().map_err(|_| IsobusError::TooShort {
            pgn,
            len: frame.data().len(),
        })?;
        Ok(Some(match pgn.number() {
            WheelBasedSpeed::PGN_NUMBER => {
                Self::WheelBasedSpeed(WheelBasedSpeed::from_bytes(bytes))
            }
            GroundBasedSpeed::PGN_NUMBER => {
                Self::GroundBasedSpeed(GroundBasedSpeed::from_bytes(bytes))
            }
            MachineSelectedSpeed::PGN_NUMBER => {
                Self::MachineSelectedSpeed(MachineSelectedSpeed::from_bytes(bytes))
            }
            MachineSelectedSpeedCommand::PGN_NUMBER => {
                Self::MachineSelectedSpeedCommand(MachineSelectedSpeedCommand::from_bytes(bytes))
            }
            GuidanceMachineInfo::PGN_NUMBER => {
                Self::GuidanceMachineInfo(GuidanceMachineInfo::from_bytes(bytes))
            }
            _ => Self::GuidanceSystemCommand(GuidanceSystemCommand::from_bytes(bytes)),
        }))
    }
}
