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
            Self::DangerCircle { strength } => write!(
                f,
                "resection on the danger circle: figure strength {strength:.1e}"
            ),
        }
    }
}

impl core::error::Error for LandError {}
