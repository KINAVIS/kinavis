//! Sentence parse and translation errors.

use core::fmt;

use kinavis_kernel::KernelError;

/// Why a sentence could not be read or written.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, PartialEq)]
pub enum NmeaError {
    /// Sentence, read or to be written, longer than the standard allows.
    TooLong {
        /// Sentence length.
        length: usize,
        /// Maximum length, terminator included.
        limit: usize,
    },
    /// Sentence does not start with `$` or `!`.
    NoStart,
    /// No `*hh` checksum; required by this parser.
    NoChecksum,
    /// Checksum does not match the body.
    BadChecksum {
        /// Computed checksum.
        computed: u8,
        /// Checksum in the sentence.
        claimed: u8,
    },
    /// Byte outside the permitted printable ASCII, or a reserved character
    /// inside a field.
    BadCharacter {
        /// Byte offset within the sentence.
        offset: usize,
    },
    /// Address field is not a talker + formatter.
    BadHeader,
    /// Fewer fields than the formatter requires.
    TooFewFields {
        /// Fields present after the address.
        found: usize,
        /// Fields required.
        required: usize,
    },
    /// Field could not be parsed as the expected type.
    BadField {
        /// Zero-based field index after the address.
        index: usize,
        /// Expected type: `"time"`, `"latitude"`, etc.
        expected: &'static str,
    },
    /// Field parsed but its value is outside the domain (latitude 95°, negative
    /// DOP) or the plausibility bounds (speed over 1000 kn).
    Value {
        /// Zero-based field index after the address.
        index: usize,
        /// Domain error.
        error: KernelError,
    },
    /// Output buffer too small.
    BufferTooSmall,
}

impl fmt::Display for NmeaError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::TooLong { length, limit } => {
                write!(f, "sentence of {length} bytes exceeds the limit of {limit}")
            }
            Self::NoStart => f.write_str("sentence does not begin with `$` or `!`"),
            Self::NoChecksum => f.write_str("sentence has no checksum"),
            Self::BadChecksum { computed, claimed } => write!(
                f,
                "checksum {claimed:02X} claimed, body sums to {computed:02X}"
            ),
            Self::BadCharacter { offset } => {
                write!(f, "byte at offset {offset} is not allowed in a sentence")
            }
            Self::BadHeader => f.write_str("address field is not a talker and a formatter"),
            Self::TooFewFields { found, required } => {
                write!(f, "{found} fields where {required} are required")
            }
            Self::BadField { index, expected } => {
                write!(f, "field {index} is not a {expected}")
            }
            Self::Value { index, error } => write!(f, "field {index}: {error}"),
            Self::BufferTooSmall => f.write_str("output buffer too small for the sentence"),
        }
    }
}

impl core::error::Error for NmeaError {}

/// Why a well-formed sentence could not become a [`GnssFix`].
///
/// `#[non_exhaustive]`; match with a wildcard arm.
///
/// [`GnssFix`]: kinavis_kernel::GnssFix
#[non_exhaustive]
#[derive(Debug, Clone, PartialEq)]
pub enum TranslationError {
    /// No position.
    NoPosition,
    /// No time of day.
    NoTime,
    /// No date in the sentence and none supplied.
    NoDate,
    /// Date and time do not form a valid instant (30 February, a leap second
    /// the representation cannot hold).
    BadMoment(KernelError),
}

impl fmt::Display for TranslationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NoPosition => f.write_str("sentence carries no position"),
            Self::NoTime => f.write_str("sentence carries no time of day"),
            Self::NoDate => f.write_str("sentence carries no date"),
            Self::BadMoment(error) => write!(f, "date and time name no moment: {error}"),
        }
    }
}

impl core::error::Error for TranslationError {}
