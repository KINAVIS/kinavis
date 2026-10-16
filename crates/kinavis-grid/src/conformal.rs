//! Conformal latitude on the ellipsoid, through `τ = tan φ` and
//! `τ′ = tan χ = sinh ψ` (Karney 2011, eqs. 7–9), and its Newton inverse.

use kinavis_kernel::error::{KernelError, Result};
use kinavis_kernel::math;

/// Iterations of the Newton inverse; it converges in at most three for any
/// terrestrial ellipsoid.
pub(crate) const MAX_ITERATIONS: u32 = 5;

/// `e · atanh(e · x)`, continued to negative `e²` (prolate) as
/// `−|e| · atan(|e| · x)`.
pub(crate) fn eatanhe(x: f64, es: f64) -> f64 {
    if es > 0.0 {
        es * math::atanh(es * x)
    } else {
        -es * math::atan(es * x)
    }
}

/// `τ′` from `τ`.
pub(crate) fn taup(tau: f64, es: f64) -> f64 {
    if !tau.is_finite() {
        return tau;
    }
    let tau1 = math::hypot(1.0, tau);
    let sig = math::sinh(eatanhe(tau / tau1, es));
    math::hypot(1.0, sig) * tau - sig * tau1
}

/// `τ` from `τ′`, by Newton's method.
///
/// # Errors
///
/// [`KernelError::NotConverged`] if [`MAX_ITERATIONS`] do not settle it.
pub(crate) fn tau(taup_value: f64, es: f64) -> Result<f64> {
    let e2m = 1.0 - es * es;
    let tolerance = math::sqrt(f64::EPSILON) / 10.0 * math::abs(taup_value).max(1.0);
    let tau_max = 2.0 / math::sqrt(f64::EPSILON);
    let mut tau = if math::abs(taup_value) > 70.0 {
        taup_value * math::exp(eatanhe(1.0, es))
    } else {
        taup_value / e2m
    };
    if tau.is_nan() || math::abs(tau) >= tau_max {
        return Ok(tau);
    }
    let mut step = 0.0;
    for _ in 0..MAX_ITERATIONS {
        let estimate = taup(tau, es);
        step = (taup_value - estimate) * (1.0 + e2m * tau * tau)
            / (e2m * math::hypot(1.0, tau) * math::hypot(1.0, estimate));
        tau += step;
        if step.is_nan() || math::abs(step) < tolerance {
            return Ok(tau);
        }
    }
    Err(KernelError::NotConverged {
        iterations: MAX_ITERATIONS,
        residual: step,
    })
}
