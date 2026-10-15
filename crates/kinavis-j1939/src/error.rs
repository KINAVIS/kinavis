//! Identifier, frame and transport errors.

use core::fmt;

use crate::id::Pgn;

/// Why a frame or a transfer could not be accepted.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum J1939Error {
    /// Identifier wider than 29 bits.
    BadCanId {
        /// Identifier value.
        raw: u32,
    },
    /// Frame with more than 8 data bytes.
    BadFrameLength {
        /// Length.
        len: usize,
    },
    /// NAME field beyond its width.
    BadName {
        /// Field.
        field: &'static str,
        /// Value given.
        value: u32,
    },
    /// Transfer longer than the protocol carries or than the receiver holds.
    TooLong {
        /// PGN carried.
        pgn: Pgn,
        /// Declared size, bytes.
        size: u32,
        /// Largest accepted, bytes.
        limit: u32,
    },
    /// Connection management message too short or of an unknown kind.
    BadControl {
        /// Control byte.
        control: u8,
    },
}

impl fmt::Display for J1939Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::BadCanId { raw } => write!(f, "identifier {raw:#x} is wider than 29 bits"),
            Self::BadFrameLength { len } => {
                write!(f, "a frame of {len} bytes where at most 8 are possible")
            }
            Self::BadName { field, value } => write!(f, "NAME {field} of {value} is too wide"),
            Self::TooLong { pgn, size, limit } => {
                write!(f, "{pgn}: transfer of {size} bytes, limit {limit}")
            }
            Self::BadControl { control } => {
                write!(
                    f,
                    "connection management message {control:#04x} not understood"
                )
            }
        }
    }
}

impl core::error::Error for J1939Error {}
