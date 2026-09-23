//! Errors of the value types and numeric kernels.
//!
//! [`KernelError`] uses only the kernel's vocabulary: non-finite or
//! out-of-range numbers, undersized buffers or stores, unsolvable systems,
//! undefined quantities, missing inputs, reversed clocks. Algorithm-specific
//! errors (deviation tables, sailings) belong to the crate that owns the
//! algorithm, which wraps this type via `From`.
//!
//! `#[non_exhaustive]`: variants may be added in a minor release; match with a
//! wildcard arm.
//!
//! [`ensure_finite`] and [`ensure_range`] are public so adapters validate
//! sensor input the same way the value types do.

use crate::inline::InlineStr;
use core::fmt;

/// Result alias.
pub type Result<T> = core::result::Result<T, KernelError>;

/// Maximum bytes of offending input carried by an error.
///
/// Enough to identify the input; bounded so errors need no allocator and cannot
/// be inflated by input.
pub const EXCERPT_BYTES: usize = 32;

/// Excerpt of offending input.
pub type Excerpt = InlineStr<EXCERPT_BYTES>;

/// Kernel operation failure.
///
/// No operation panics on caller data; every failure is reported through this
/// type.
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum KernelError {
    /// Parameter is `NaN` or infinite.
    NotFinite {
        /// Parameter name.
        parameter: &'static str,
        /// Value supplied.
        value: f64,
    },

    /// Parameter is finite but out of range.
    OutOfRange {
        /// Parameter name.
        parameter: &'static str,
        /// Value supplied.
        value: f64,
        /// Minimum, inclusive.
        min: f64,
        /// Maximum, inclusive.
        max: f64,
    },

    /// Fewer items than required: interpolation nodes, route waypoints, fit
    /// samples.
    InsufficientData {
        /// Items present.
        found: usize,
        /// Items required.
        required: usize,
        /// Operation that required them.
        context: &'static str,
    },

    /// Not one of the eight cardinal/intercardinal abbreviations.
    UnknownCardinalDirection {
        /// Input, truncated.
        direction: Excerpt,
    },

    /// Caller-supplied output buffer too small.
    ///
    /// Returned by slice-writing calls so a short buffer is an error, not a
    /// silent truncation.
    BufferTooSmall {
        /// Values produced.
        needed: usize,
        /// Buffer length.
        found: usize,
    },

    /// Inline store capacity exceeded.
    ///
    /// Collections are fixed-capacity inline stores; their bounds are public
    /// constants, so callers can check beforehand.
    CapacityExceeded {
        /// What was being built.
        context: &'static str,
        /// Items required.
        needed: usize,
        /// Capacity.
        capacity: usize,
    },

    /// Linear system has no unique solution.
    ///
    /// For a parametric fit: the sample courses do not constrain the requested
    /// coefficients (e.g. five coefficients from nodes on one semicircle).
    SingularSystem {
        /// System being solved.
        context: &'static str,
    },

    /// Matrix is not a valid covariance (symmetric positive definite): e.g.
    /// zero observation variance, collapsed innovation covariance.
    NotCovariance {
        /// Intended role of the matrix.
        context: &'static str,
    },

    /// Iterative solver did not converge.
    ///
    /// The deviation inverse fails this way when deviation changes faster than
    /// 1° per degree of heading, so several compass courses map to one magnetic
    /// course; the compass needs re-swinging.
    NotConverged {
        /// Iterations run.
        iterations: u32,
        /// Final residual.
        residual: f64,
    },

    /// String could not be parsed.
    Parse {
        /// Expected value type.
        what: &'static str,
        /// Input, truncated.
        input: Excerpt,
    },

    /// Mathematically undefined for the inputs: rhumb line through a pole,
    /// direction of a zero vector, great circle between antipodes.
    Indeterminate {
        /// Undefined quantity.
        quantity: &'static str,
    },

    /// Required input unavailable: snapshot without position, environment
    /// sample without tide, empty estimator history.
    Missing {
        /// Missing input.
        what: &'static str,
    },

    /// Result exists but is not representable: instant beyond range, counter
    /// overflow.
    Unrepresentable {
        /// Unrepresentable quantity.
        what: &'static str,
    },

    /// Instants in the wrong order for the elapsed time requested (clock
    /// stepped back, or arguments swapped).
    TimeReversed {
        /// Amount by which the later instant precedes the earlier.
        by: core::time::Duration,
    },

    /// Height referred to a different vertical datum than required.
    VerticalDatumMismatch {
        /// Required datum.
        required: crate::geodesy::VerticalDatum,
        /// Datum of the height.
        found: crate::geodesy::VerticalDatum,
    },

    /// Data requested outside its validity interval (leap-second table past
    /// expiry, magnetic model past epoch).
    OutsideValidity {
        /// Requested data.
        data: &'static str,
    },
}

/// Checks that a value is finite.
///
/// # Errors
///
/// [`KernelError::NotFinite`] for `NaN` or infinity.
pub fn ensure_finite(parameter: &'static str, value: f64) -> Result<()> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(KernelError::NotFinite { parameter, value })
    }
}

/// Checks that a value is finite and within `[min, max]`.
///
/// # Errors
///
/// [`KernelError::NotFinite`] for `NaN` or infinity;
/// [`KernelError::OutOfRange`] outside the interval.
pub fn ensure_range(parameter: &'static str, value: f64, min: f64, max: f64) -> Result<()> {
    ensure_finite(parameter, value)?;
    if value < min || value > max {
        return Err(KernelError::OutOfRange {
            parameter,
            value,
            min,
            max,
        });
    }
    Ok(())
}

impl fmt::Display for KernelError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NotFinite { parameter, value } => {
                write!(f, "{parameter} must be a finite number, got {value}")
            }
            Self::OutOfRange {
                parameter,
                value,
                min,
                max,
            } => write!(
                f,
                "{parameter} out of range: {value}. Must be between {min} and {max}"
            ),
            Self::InsufficientData {
                found,
                required,
                context,
            } => write!(f, "{context} needs at least {required}, and has {found}"),
            Self::UnknownCardinalDirection { direction } => write!(
                f,
                "unknown cardinal direction: {direction}. Expected one of N, NE, E, SE, S, SW, W, NW"
            ),
            Self::CapacityExceeded {
                context,
                needed,
                capacity,
            } => write!(
                f,
                "{context} needs room for {needed}, and the limit is {capacity}"
            ),
            Self::BufferTooSmall { needed, found } => write!(
                f,
                "output buffer holds {found} values, {needed} are needed"
            ),
            Self::SingularSystem { context } => {
                write!(f, "singular system while solving {context}")
            }
            Self::NotCovariance { context } => {
                write!(f, "{context} is not a covariance matrix")
            }
            Self::NotConverged {
                iterations,
                residual,
            } => write!(
                f,
                "solver did not converge after {iterations} iterations, residual {residual}"
            ),
            Self::Parse { what, input } => {
                write!(f, "could not read {input:?} as a {what}")
            }
            Self::Indeterminate { quantity } => {
                write!(f, "{quantity} is indeterminate for these inputs")
            }
            Self::Missing { what } => write!(f, "{what} is not available"),
            Self::Unrepresentable { what } => write!(f, "{what} cannot be represented"),
            Self::TimeReversed { by } => {
                write!(f, "time ran backwards by {} s", by.as_secs_f64())
            }
            Self::VerticalDatumMismatch { required, found } => write!(
                f,
                "a height above {found:?} was given where one above {required:?} is required"
            ),
            Self::OutsideValidity { data } => {
                write!(f, "the {data} is not valid for the requested moment")
            }
        }
    }
}

// `std::error::Error` re-exports `core::error::Error` since Rust 1.81; one impl
// covers `std` and `no_std`.
impl core::error::Error for KernelError {}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::string::ToString;

    #[test]
    fn every_variant_has_a_message() {
        let errors = [
            KernelError::NotFinite {
                parameter: "course",
                value: f64::NAN,
            },
            KernelError::OutOfRange {
                parameter: "course",
                value: 400.0,
                min: 0.0,
                max: 360.0,
            },
            KernelError::InsufficientData {
                found: 1,
                required: 2,
                context: "interpolation",
            },
            KernelError::UnknownCardinalDirection {
                direction: Excerpt::new("XYZ"),
            },
            KernelError::BufferTooSmall {
                needed: 36,
                found: 8,
            },
            KernelError::CapacityExceeded {
                context: "a deviation table",
                needed: 90,
                capacity: 72,
            },
            KernelError::SingularSystem {
                context: "parametric fit",
            },
            KernelError::NotCovariance {
                context: "the observation noise",
            },
            KernelError::TimeReversed {
                by: core::time::Duration::from_secs(18),
            },
            KernelError::OutsideValidity {
                data: "leap second table",
            },
            KernelError::VerticalDatumMismatch {
                required: crate::geodesy::VerticalDatum::Ellipsoid,
                found: crate::geodesy::VerticalDatum::MeanSeaLevel,
            },
            KernelError::NotConverged {
                iterations: 64,
                residual: 1.0,
            },
            KernelError::Parse {
                what: "latitude",
                input: Excerpt::new("north-ish"),
            },
            KernelError::Indeterminate {
                quantity: "a rhumb line through a pole",
            },
            KernelError::Missing {
                what: "the vessel's position",
            },
            KernelError::Unrepresentable {
                what: "a moment beyond the end of time",
            },
        ];
        for error in errors {
            assert!(!error.to_string().is_empty(), "{error:?}");
        }
    }

    #[test]
    fn errors_compare_by_value() {
        let a = KernelError::OutOfRange {
            parameter: "x",
            value: 1.0,
            min: 0.0,
            max: 0.5,
        };
        assert_eq!(a, a.clone());
        assert_ne!(
            a,
            KernelError::Missing {
                what: "the vessel's position"
            }
        );
    }

    #[test]
    fn the_checks_report_what_they_reject() {
        assert!(ensure_finite("x", 1.0).is_ok());
        assert!(matches!(
            ensure_finite("x", f64::NAN),
            Err(KernelError::NotFinite { parameter: "x", .. })
        ));
        assert!(matches!(
            ensure_range("x", 2.0, 0.0, 1.0),
            Err(KernelError::OutOfRange { parameter: "x", .. })
        ));
        assert!(matches!(
            ensure_range("x", f64::INFINITY, 0.0, 1.0),
            Err(KernelError::NotFinite { .. })
        ));
    }
}
