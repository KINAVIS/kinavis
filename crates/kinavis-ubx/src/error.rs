//! Decoding and translation errors.

use core::fmt;

use kinavis_kernel::KernelError;

/// Why a frame's payload could not be decoded.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, PartialEq)]
pub enum UbxError {
    /// The payload length does not match the message.
    Length {
        /// Class.
        class: u8,
        /// Identifier.
        id: u8,
        /// Length the message requires.
        expected: usize,
        /// Length received.
        found: usize,
    },
    /// A field holds a value outside its domain (latitude beyond 90°).
    Value {
        /// Field name.
        field: &'static str,
        /// Domain error.
        error: KernelError,
    },
    /// A version of the message this crate does not read.
    Version {
        /// Class.
        class: u8,
        /// Identifier.
        id: u8,
        /// Version found.
        version: u8,
    },
}

impl fmt::Display for UbxError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Length {
                class,
                id,
                expected,
                found,
            } => write!(
                f,
                "UBX {class:02X}-{id:02X} payload of {found} bytes, expected {expected}"
            ),
            Self::Value { field, error } => write!(f, "UBX field {field}: {error}"),
            Self::Version { class, id, version } => {
                write!(f, "UBX {class:02X}-{id:02X} version {version} is not read")
            }
        }
    }
}

impl core::error::Error for UbxError {}

/// Why a decoded message does not translate into a kernel value.
#[non_exhaustive]
#[derive(Debug, Clone, PartialEq)]
pub enum TranslationError {
    /// The receiver has not resolved the UTC date and time.
    NoTime,
    /// The date and time do not form a valid instant.
    BadMoment(KernelError),
}

impl fmt::Display for TranslationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NoTime => f.write_str("the receiver has no valid UTC date and time"),
            Self::BadMoment(error) => write!(f, "date and time form no instant: {error}"),
        }
    }
}

impl core::error::Error for TranslationError {}

/// Wraps a kernel domain error with the field it came from.
pub(crate) fn value(field: &'static str) -> impl Fn(KernelError) -> UbxError {
    move |error| UbxError::Value { field, error }
}
