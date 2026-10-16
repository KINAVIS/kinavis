//! Frame, reassembly and decoding errors.

use core::fmt;

use kinavis_j1939::J1939Error;
use kinavis_kernel::KernelError;

use crate::id::Pgn;

/// Why a frame could not be accepted, reassembled or decoded.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, PartialEq)]
pub enum Nmea2000Error {
    /// Identifier or frame refused by `kinavis-j1939`.
    J1939(J1939Error),
    /// PGN whose transport (single frame or fast packet) is unknown; needs an
    /// explicit transport.
    UnknownPgn {
        /// PGN.
        pgn: Pgn,
    },
    /// Fast-packet first frame declares more bytes than a fast packet can
    /// carry, or fewer than the frame holds.
    BadLength {
        /// Declared length.
        declared: u8,
        /// Fast-packet maximum.
        limit: usize,
    },
    /// Fast-packet frame not expected by the assembly: out of order, repeated,
    /// of another sequence, or without a first frame.
    UnexpectedFrame {
        /// Expected frame counter.
        expected: u8,
        /// Received frame counter.
        found: u8,
    },
    /// Assembler built with zero slots.
    NoSlot,
    /// Payload shorter than a field of the group.
    TooShort {
        /// PGN.
        pgn: Pgn,
        /// Payload length.
        bytes: usize,
        /// Required length.
        needed: usize,
    },
    /// Field parsed but its value is outside the domain (latitude 95°, NaN
    /// speed).
    Value {
        /// Field name: `"latitude"`, `"heading"`.
        field: &'static str,
        /// Domain error.
        error: KernelError,
    },
}

impl fmt::Display for Nmea2000Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::J1939(error) => fmt::Display::fmt(error, f),
            Self::UnknownPgn { pgn } => write!(f, "{pgn} is not a group this crate knows"),
            Self::BadLength { declared, limit } => {
                write!(f, "fast packet of {declared} bytes declared, limit {limit}")
            }
            Self::UnexpectedFrame { expected, found } => {
                write!(f, "frame {found} where frame {expected} was expected")
            }
            Self::NoSlot => f.write_str("the assembler has no slot for a packet in frames"),
            Self::TooShort { pgn, bytes, needed } => {
                write!(
                    f,
                    "{pgn}: payload of {bytes} bytes where {needed} are needed"
                )
            }
            Self::Value { field, error } => write!(f, "{field}: {error}"),
        }
    }
}

impl From<J1939Error> for Nmea2000Error {
    fn from(error: J1939Error) -> Self {
        Self::J1939(error)
    }
}

impl core::error::Error for Nmea2000Error {
    fn source(&self) -> Option<&(dyn core::error::Error + 'static)> {
        match self {
            Self::J1939(error) => Some(error),
            _ => None,
        }
    }
}
