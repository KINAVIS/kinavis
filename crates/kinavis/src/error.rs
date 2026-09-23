//! Navigation computation errors.
//!
//! [`KernelError`] covers value types and numeric primitives (out-of-range
//! numbers, small buffers, unsolvable systems). [`NavigationError`] adds
//! algorithm-level failures (duplicate deviation entries, parallel rhumb lines,
//! current stronger than the vessel) and wraps kernel errors as
//! [`NavigationError::Kernel`], so `?` propagates them unchanged and callers
//! can match either.
//!
//! `#[non_exhaustive]`: variants may be added in a minor release; match with a
//! wildcard arm. [`ensure_finite`] and [`ensure_range`] are re-exported for
//! consistent validation.

use core::fmt;

pub use kinavis_kernel::error::{ensure_finite, ensure_range, Excerpt, KernelError, EXCERPT_BYTES};

/// Result alias.
pub type Result<T> = core::result::Result<T, NavigationError>;

/// Navigation computation failure.
///
/// No operation panics on caller data; every failure is reported through this
/// type.
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum NavigationError {
    /// Kernel error, propagated unchanged.
    Kernel(KernelError),

    /// Deviation table step outside `1..=180` degrees.
    InvalidStep {
        /// Step supplied, degrees.
        step: i32,
    },

    /// Two entries normalise to the same compass course.
    DuplicateCourse {
        /// Duplicated course, degrees.
        course: i32,
    },

    /// Compass course is not a table node.
    CourseNotInTable {
        /// Normalised course looked up, degrees.
        course: i32,
    },

    /// Deviation slice has the wrong length.
    UnexpectedTableLength {
        /// Values supplied.
        found: usize,
        /// Values expected.
        expected: usize,
    },

    /// Lines or circles do not intersect.
    Parallel {
        /// Intersection attempted.
        context: &'static str,
    },

    /// Well-posed problem without a solution.
    ///
    /// Unlike [`KernelError::Indeterminate`] (quantity undefined), the answer
    /// provably does not exist: no course achieves the requested CPA, no circle
    /// passes through the points.
    NoSolution {
        /// Unsolvable quantity.
        context: &'static str,
    },

    /// Current too strong to make good the requested track.
    CurrentTooStrong {
        /// Current rate, kn.
        drift: f64,
        /// Speed through the water, kn.
        speed_through_water: f64,
    },
}

impl NavigationError {
    /// Inner kernel error, if any.
    #[must_use]
    pub const fn kernel(&self) -> Option<&KernelError> {
        match self {
            Self::Kernel(error) => Some(error),
            _ => None,
        }
    }

    /// Converts to [`KernelError`] for kernel ports implemented in this crate.
    ///
    /// Kernel ports (compass, current, process models) return [`KernelError`],
    /// and the algorithms behind this crate's implementations fail only in
    /// kernel terms (too few nodes, singular fit, instant out of range). Any
    /// other variant maps to [`KernelError::Indeterminate`] with its
    /// description, so nothing is lost.
    #[must_use]
    pub fn into_kernel(self) -> KernelError {
        match self {
            Self::Kernel(error) => error,
            Self::InvalidStep { .. } => KernelError::Indeterminate {
                quantity: "a deviation table with an invalid step",
            },
            Self::DuplicateCourse { .. } => KernelError::Indeterminate {
                quantity: "a deviation table with a duplicate course",
            },
            Self::CourseNotInTable { .. } => KernelError::Indeterminate {
                quantity: "a course that is not a node of the table",
            },
            Self::UnexpectedTableLength { .. } => KernelError::Indeterminate {
                quantity: "a deviation table of unexpected length",
            },
            Self::Parallel { context } | Self::NoSolution { context } => {
                KernelError::Indeterminate { quantity: context }
            }
            Self::CurrentTooStrong { .. } => KernelError::Indeterminate {
                quantity: "a track against a current too strong to stem",
            },
        }
    }
}

impl From<KernelError> for NavigationError {
    fn from(error: KernelError) -> Self {
        Self::Kernel(error)
    }
}

impl fmt::Display for NavigationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Kernel(error) => fmt::Display::fmt(error, f),
            Self::InvalidStep { step } => write!(
                f,
                "invalid deviation table step: {step}. Must be between 1 and 180 degrees"
            ),
            Self::DuplicateCourse { course } => {
                write!(f, "duplicate compass course in deviation table: {course}")
            }
            Self::CourseNotInTable { course } => write!(
                f,
                "compass course {course} is not a node of this deviation table"
            ),
            Self::UnexpectedTableLength { found, expected } => {
                write!(f, "expected {expected} deviation values, got {found}")
            }
            Self::Parallel { context } => write!(f, "{context} never meet"),
            Self::NoSolution { context } => {
                write!(f, "no solution exists for {context}")
            }
            Self::CurrentTooStrong {
                drift,
                speed_through_water,
            } => write!(
                f,
                "current of {drift} is too strong for a speed through water of {speed_through_water}"
            ),
        }
    }
}

impl core::error::Error for NavigationError {
    fn source(&self) -> Option<&(dyn core::error::Error + 'static)> {
        match self {
            Self::Kernel(error) => Some(error),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::string::ToString;

    #[test]
    fn every_variant_has_a_message() {
        let errors = [
            NavigationError::Kernel(KernelError::Indeterminate {
                quantity: "a rhumb line through a pole",
            }),
            NavigationError::InvalidStep { step: 0 },
            NavigationError::DuplicateCourse { course: 10 },
            NavigationError::CourseNotInTable { course: 50 },
            NavigationError::UnexpectedTableLength {
                found: 5,
                expected: 36,
            },
            NavigationError::Parallel {
                context: "the two great circles",
            },
            NavigationError::NoSolution {
                context: "a course achieving that closest approach",
            },
            NavigationError::CurrentTooStrong {
                drift: 5.0,
                speed_through_water: 2.0,
            },
        ];
        for error in errors {
            assert!(!error.to_string().is_empty(), "{error:?}");
        }
    }

    #[test]
    fn a_kernel_error_comes_through_unchanged() {
        let inner = KernelError::Missing {
            what: "the vessel's position",
        };
        let outer: NavigationError = inner.clone().into();
        assert_eq!(outer.kernel(), Some(&inner));
        assert_eq!(outer.to_string(), inner.to_string());
        assert!(NavigationError::InvalidStep { step: 0 }.kernel().is_none());
        assert_eq!(outer.into_kernel(), inner);
        assert!(matches!(
            NavigationError::Parallel { context: "x" }.into_kernel(),
            KernelError::Indeterminate { quantity: "x" }
        ));
    }
}
