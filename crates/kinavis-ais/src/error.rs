//! Unarmouring, reassembly and decoding errors.

use core::fmt;

use kinavis_kernel::KernelError;

/// Why a payload could not be unarmoured, reassembled or decoded.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, PartialEq)]
pub enum AisError {
    /// Payload character outside the armouring alphabet.
    BadArmouring {
        /// Character offset in the payload.
        offset: usize,
    },
    /// More fill bits than payload bits.
    BadFill {
        /// Fill bits declared.
        fill_bits: u8,
        /// Payload bits.
        payload_bits: usize,
    },
    /// Longer than any defined message.
    TooLong {
        /// Total bits of the fragments.
        bits: usize,
        /// Maximum.
        limit: usize,
    },
    /// Message shorter than a field of its type.
    TooShort {
        /// Message length in bits.
        bits: usize,
        /// Bits required by the type.
        needed: usize,
    },
    /// Fragment not expected by the assembly: out of order, repeated, or
    /// without a first fragment.
    UnexpectedFragment {
        /// Expected fragment number.
        expected: u8,
        /// Received fragment number.
        found: u8,
    },
    /// Assembler built with zero slots.
    NoSlot,
    /// Field parsed but its value is outside the domain (latitude 95°, heading
    /// 400°).
    Value {
        /// Field name: `"latitude"`, `"heading"`.
        field: &'static str,
        /// Domain error.
        error: KernelError,
    },
}

impl fmt::Display for AisError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::BadArmouring { offset } => {
                write!(f, "payload character at offset {offset} is not armoured")
            }
            Self::BadFill {
                fill_bits,
                payload_bits,
            } => write!(
                f,
                "{fill_bits} fill bits claimed of a payload of {payload_bits} bits"
            ),
            Self::TooLong { bits, limit } => {
                write!(f, "message of {bits} bits exceeds the limit of {limit}")
            }
            Self::TooShort { bits, needed } => {
                write!(f, "message of {bits} bits where {needed} are needed")
            }
            Self::UnexpectedFragment { expected, found } => {
                write!(f, "fragment {found} where fragment {expected} was expected")
            }
            Self::NoSlot => f.write_str("the assembler has no slot for a message in fragments"),
            Self::Value { field, error } => write!(f, "{field}: {error}"),
        }
    }
}

impl core::error::Error for AisError {}
