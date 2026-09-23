//! Floating-point primitives routed to `std` or `libm`.
//!
//! Routing every transcendental call through this module lets the crate build
//! as `no_std`: with `default-features = false, features = ["libm"]` the same
//! code links against pure-Rust `libm`.
//!
//! Public because the rule applies to the whole crate family: magnetic models
//! and estimators call these and thereby build for bare metal and produce the
//! same results on every target. Also provides the conversions and guarded
//! casts numeric code needs.

#[cfg(feature = "std")]
mod imp {
    pub(crate) fn sin(x: f64) -> f64 {
        x.sin()
    }
    pub(crate) fn cos(x: f64) -> f64 {
        x.cos()
    }
    pub(crate) fn asin(x: f64) -> f64 {
        x.asin()
    }
    pub(crate) fn atan2(y: f64, x: f64) -> f64 {
        y.atan2(x)
    }
    pub(crate) fn sqrt(x: f64) -> f64 {
        x.sqrt()
    }
    pub(crate) fn abs(x: f64) -> f64 {
        x.abs()
    }
    pub(crate) fn copysign(magnitude: f64, sign: f64) -> f64 {
        magnitude.copysign(sign)
    }
    pub(crate) fn tan(x: f64) -> f64 {
        x.tan()
    }
    pub(crate) fn atan(x: f64) -> f64 {
        x.atan()
    }
    pub(crate) fn acos(x: f64) -> f64 {
        x.acos()
    }
    pub(crate) fn ln(x: f64) -> f64 {
        x.ln()
    }
    pub(crate) fn exp(x: f64) -> f64 {
        x.exp()
    }
    pub(crate) fn hypot(x: f64, y: f64) -> f64 {
        x.hypot(y)
    }
    pub(crate) fn round(x: f64) -> f64 {
        x.round()
    }
    pub(crate) fn ceil(x: f64) -> f64 {
        x.ceil()
    }
    pub(crate) fn trunc(x: f64) -> f64 {
        x.trunc()
    }
}

#[cfg(all(not(feature = "std"), feature = "libm"))]
mod imp {
    pub(crate) fn sin(x: f64) -> f64 {
        libm::sin(x)
    }
    pub(crate) fn cos(x: f64) -> f64 {
        libm::cos(x)
    }
    pub(crate) fn asin(x: f64) -> f64 {
        libm::asin(x)
    }
    pub(crate) fn atan2(y: f64, x: f64) -> f64 {
        libm::atan2(y, x)
    }
    pub(crate) fn sqrt(x: f64) -> f64 {
        libm::sqrt(x)
    }
    pub(crate) fn abs(x: f64) -> f64 {
        libm::fabs(x)
    }
    pub(crate) fn copysign(magnitude: f64, sign: f64) -> f64 {
        libm::copysign(magnitude, sign)
    }
    pub(crate) fn tan(x: f64) -> f64 {
        libm::tan(x)
    }
    pub(crate) fn atan(x: f64) -> f64 {
        libm::atan(x)
    }
    pub(crate) fn acos(x: f64) -> f64 {
        libm::acos(x)
    }
    pub(crate) fn ln(x: f64) -> f64 {
        libm::log(x)
    }
    pub(crate) fn exp(x: f64) -> f64 {
        libm::exp(x)
    }
    pub(crate) fn hypot(x: f64, y: f64) -> f64 {
        libm::hypot(x, y)
    }
    pub(crate) fn round(x: f64) -> f64 {
        libm::round(x)
    }
    pub(crate) fn ceil(x: f64) -> f64 {
        libm::ceil(x)
    }
    pub(crate) fn trunc(x: f64) -> f64 {
        libm::trunc(x)
    }
}

#[cfg(not(any(feature = "std", feature = "libm")))]
compile_error!(
    "kinavis-kernel needs floating point math: enable the default `std` feature, \
     or build with `--no-default-features --features libm` for `no_std` targets"
);

// One function per primitive, delegating to the feature-selected
// implementation. Downstream code calls these, never `f64` methods, so a stray
// `x.sin()` cannot break `no_std` builds.

/// Sine, radians.
#[must_use]
#[inline]
pub fn sin(x: f64) -> f64 {
    imp::sin(x)
}

/// Cosine, radians.
#[must_use]
#[inline]
pub fn cos(x: f64) -> f64 {
    imp::cos(x)
}

/// Tangent, radians.
#[must_use]
#[inline]
pub fn tan(x: f64) -> f64 {
    imp::tan(x)
}

/// Arcsine in `[-π/2, π/2]`; `NaN` outside `[-1, 1]`.
#[must_use]
#[inline]
pub fn asin(x: f64) -> f64 {
    imp::asin(x)
}

/// Arccosine in `[0, π]`; `NaN` outside `[-1, 1]`.
#[must_use]
#[inline]
pub fn acos(x: f64) -> f64 {
    imp::acos(x)
}

/// Arctangent in `[-π/2, π/2]`.
#[must_use]
#[inline]
pub fn atan(x: f64) -> f64 {
    imp::atan(x)
}

/// Four-quadrant arctangent of `y / x`, in `[-π, π]`.
///
/// Mathematical argument order `atan2(y, x)`: for a course from north/east
/// components, pass east first.
#[must_use]
#[inline]
pub fn atan2(y: f64, x: f64) -> f64 {
    imp::atan2(y, x)
}

/// Square root; `NaN` for negative input.
#[must_use]
#[inline]
pub fn sqrt(x: f64) -> f64 {
    imp::sqrt(x)
}

/// Absolute value.
#[must_use]
#[inline]
pub fn abs(x: f64) -> f64 {
    imp::abs(x)
}

/// `magnitude` with the sign of `sign`.
#[must_use]
#[inline]
pub fn copysign(magnitude: f64, sign: f64) -> f64 {
    imp::copysign(magnitude, sign)
}

/// Natural logarithm; `NaN` for negative input, `-∞` for zero.
#[must_use]
#[inline]
pub fn ln(x: f64) -> f64 {
    imp::ln(x)
}

/// `eˣ`.
#[must_use]
#[inline]
pub fn exp(x: f64) -> f64 {
    imp::exp(x)
}

/// `√(x² + y²)` without intermediate overflow.
#[must_use]
#[inline]
pub fn hypot(x: f64, y: f64) -> f64 {
    imp::hypot(x, y)
}

/// Round to nearest, halves away from zero.
#[must_use]
#[inline]
pub fn round(x: f64) -> f64 {
    imp::round(x)
}

/// Ceiling.
#[must_use]
#[inline]
pub fn ceil(x: f64) -> f64 {
    imp::ceil(x)
}

/// Truncate towards zero.
#[must_use]
#[inline]
pub fn trunc(x: f64) -> f64 {
    imp::trunc(x)
}

/// Degrees per radian (`to_radians` without `std`).
const DEGREES_PER_RADIAN: f64 = 180.0 / core::f64::consts::PI;

/// Degrees to radians.
#[must_use]
pub fn to_radians(degrees: f64) -> f64 {
    degrees / DEGREES_PER_RADIAN
}

/// Radians to degrees.
#[must_use]
pub fn to_degrees(radians: f64) -> f64 {
    radians * DEGREES_PER_RADIAN
}

/// Whether a value is an integer.
///
/// Exact comparison is intended: the question is whether there is any
/// fractional part.
#[allow(clippy::float_cmp)]
#[must_use]
pub fn is_integral(value: f64) -> bool {
    value == trunc(value)
}

/// Relative tolerance for comparing computed results.
///
/// Larger than `f64::EPSILON`: chained transcendental calls lose several ULP,
/// and cancellation leaves a residue proportional to the operands.
pub const RELATIVE_TOLERANCE: f64 = 1e-12;

/// Whether a value is zero relative to `scale`, the magnitude of its operands.
///
/// Two cancelling 5 kn vectors leave a residue well above `f64::EPSILON` that
/// still means "stationary"; the same residue from 5000 kn vectors is noise of
/// another order. No fixed constant handles both.
#[must_use]
pub fn is_effectively_zero(value: f64, scale: f64) -> bool {
    abs(value) <= abs(scale) * RELATIVE_TOLERANCE
}

/// Rounds a value known to be within `i32` range. Callers pass angles bounded
/// by 360.
#[allow(clippy::cast_possible_truncation)]
#[must_use]
pub fn round_to_i32(value: f64) -> i32 {
    let rounded = round(value);
    if rounded > f64::from(i32::MAX) || rounded < f64::from(i32::MIN) {
        return 0;
    }
    rounded as i32
}

/// Truncates a small non-negative value to `usize`. Callers bound it first;
/// out-of-range input yields zero.
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
#[must_use]
pub fn to_usize(value: f64) -> usize {
    // NaN fails the range test and hits the guard.
    if !(0.0..=1e9).contains(&value) {
        return 0;
    }
    value as usize
}

/// Count as `f64`. Counts here are far below 2^53, so the conversion is exact.
#[allow(clippy::cast_precision_loss)]
#[must_use]
pub fn count_to_f64(count: usize) -> f64 {
    count as f64
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn radian_conversion_round_trips() {
        for degrees in [0.0, 1.0, 45.0, 90.0, 180.0, 359.9] {
            let back = to_degrees(to_radians(degrees));
            assert!((back - degrees).abs() < 1e-12);
        }
    }

    #[test]
    fn trig_matches_known_values() {
        assert!(abs(sin(to_radians(90.0)) - 1.0) < 1e-12);
        assert!(abs(cos(to_radians(180.0)) + 1.0) < 1e-12);
        assert!(abs(to_degrees(atan2(1.0, 0.0)) - 90.0) < 1e-12);
        assert!(abs(hypot(3.0, 4.0) - 5.0) < 1e-12);
        assert!(abs(sqrt(9.0) - 3.0) < 1e-12);
        assert!(abs(to_degrees(asin(0.5)) - 30.0) < 1e-12);
    }
}
