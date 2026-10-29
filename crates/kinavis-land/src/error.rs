//! Land navigation errors.

use core::fmt;

use kinavis::error::NavigationError;
use kinavis_kernel::KernelError;

/// Result alias.
pub type Result<T> = core::result::Result<T, LandError>;

/// Land navigation failure.
///
/// `#[non_exhaustive]`: variants may be added in a minor release; match with a
/// wildcard arm.
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum LandError {
    /// Failure of the geometry or a kernel value, propagated unchanged.
    Navigation(NavigationError),
    /// The observer stands on the circle through the three landmarks of a
    /// resection, where every point of the circle sees them under the same
    /// angles.
    DangerCircle {
        /// Greatest `|cot A − cot α|` of the figure, from the readings and
        /// the landmarks alone: zero on the circle, about twice the
        /// observer's distance from it over its radius nearby.
        strength: f64,
    },
    /// A GPX or `GeoJSON` file that does not read: malformed, or missing
    /// what the format requires.
    Format {
        /// What was wrong or missing.
        what: &'static str,
        /// Line, from 1; 0 where the format gives none.
        line: u32,
    },
}

impl From<NavigationError> for LandError {
    fn from(error: NavigationError) -> Self {
        Self::Navigation(error)
    }
}

impl From<KernelError> for LandError {
    fn from(error: KernelError) -> Self {
        Self::Navigation(NavigationError::Kernel(error))
    }
}

impl fmt::Display for LandError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Navigation(error) => fmt::Display::fmt(error, f),
            Self::Format { what, line } => write!(f, "file not readable: {what} at line {line}"),
            Self::DangerCircle { strength } => write!(
                f,
                "resection on the danger circle: figure strength {strength:.1e}"
            ),
        }
    }
}

impl core::error::Error for LandError {}
