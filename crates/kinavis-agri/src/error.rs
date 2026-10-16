//! Field guidance errors.

use core::fmt;

use kinavis::error::NavigationError;
use kinavis_kernel::KernelError;

/// Result alias.
pub type Result<T> = core::result::Result<T, AgriError>;

/// Field guidance failure.
///
/// `#[non_exhaustive]`: variants may be added in a minor release; match with a
/// wildcard arm.
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum AgriError {
    /// Failure of the planar geometry or a kernel value, propagated unchanged.
    Navigation(NavigationError),
    /// Two values built in different field planes were combined.
    PlaneMismatch,
    /// Point farther from the origin of a centred field plane than the radius
    /// its accuracy is stated for.
    BeyondFieldRadius {
        /// Distance from the plane's origin, metres.
        distance: f64,
        /// Radius limit, metres.
        limit: f64,
    },
}

impl From<NavigationError> for AgriError {
    fn from(error: NavigationError) -> Self {
        Self::Navigation(error)
    }
}

impl From<KernelError> for AgriError {
    fn from(error: KernelError) -> Self {
        Self::Navigation(NavigationError::Kernel(error))
    }
}

impl fmt::Display for AgriError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Navigation(error) => fmt::Display::fmt(error, f),
            Self::PlaneMismatch => f.write_str("values from different field planes combined"),
            Self::BeyondFieldRadius { distance, limit } => write!(
                f,
                "point {distance:.0} m from the field plane's origin, beyond its {limit:.0} m radius"
            ),
        }
    }
}

impl core::error::Error for AgriError {
    fn source(&self) -> Option<&(dyn core::error::Error + 'static)> {
        match self {
            Self::Navigation(error) => Some(error),
            _ => None,
        }
    }
}
