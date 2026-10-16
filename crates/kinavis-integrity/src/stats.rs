//! Chi-square distributions for the detection threshold and the protection
//! levels, by bounded iteration: the regularised incomplete gamma function
//! (series below `a + 1`, Lentz's continued fraction above), quantiles by
//! bisection, the noncentral distribution as a Poisson mixture.

use kinavis_kernel::error::{KernelError, Result};
use kinavis_kernel::math;

/// Iterations of each series, continued fraction and bisection.
pub const MAX_ITERATIONS: u32 = 4000;

const EPSILON: f64 = 1e-15;

/// `ln Γ(x)` for `x > 0`, Lanczos approximation (`g = 7`, nine terms),
/// relative error below 10⁻¹⁵.
pub(crate) fn ln_gamma(x: f64) -> f64 {
    // Published coefficients, kept as printed.
    #[allow(clippy::unreadable_literal, clippy::excessive_precision)]
    const COEFFICIENTS: [f64; 9] = [
        0.99999999999980993,
        676.5203681218851,
        -1259.1392167224028,
        771.32342877765313,
        -176.61502916214059,
        12.507343278686905,
        -0.13857109526572012,
        9.9843695780195716e-6,
        1.5056327351493116e-7,
    ];
    let x = x - 1.0;
    let [first, rest @ ..] = COEFFICIENTS;
    let mut sum = first;
    let mut denominator = x;
    for coefficient in rest {
        denominator += 1.0;
        sum += coefficient / denominator;
    }
    let t = x + 7.5;
    0.5 * math::ln(2.0 * core::f64::consts::PI) + (x + 0.5) * math::ln(t) - t + math::ln(sum)
}

fn not_converged(residual: f64) -> KernelError {
    KernelError::NotConverged {
        iterations: MAX_ITERATIONS,
        residual,
    }
}

/// Regularised lower incomplete gamma `P(a, x)`, `a > 0`, `x ≥ 0`.
// Notation of the continued fraction as published (Numerical Recipes, 6.2).
#[allow(clippy::many_single_char_names)]
pub(crate) fn gamma_p(a: f64, x: f64) -> Result<f64> {
    if x <= 0.0 {
        return Ok(0.0);
    }
    let prefix = -x + a * math::ln(x) - ln_gamma(a);
    if x < a + 1.0 {
        let mut term = 1.0 / a;
        let mut sum = term;
        let mut denominator = a;
        for _ in 0..MAX_ITERATIONS {
            denominator += 1.0;
            term *= x / denominator;
            sum += term;
            if math::abs(term) < math::abs(sum) * EPSILON {
                return Ok((sum * math::exp(prefix)).clamp(0.0, 1.0));
            }
        }
        Err(not_converged(term))
    } else {
        // Q(a, x) by Lentz's continued fraction.
        let tiny = 1e-300;
        let mut b = x + 1.0 - a;
        let mut c = 1.0 / tiny;
        let mut d = 1.0 / b;
        let mut h = d;
        let mut n = 0.0;
        for _ in 0..MAX_ITERATIONS {
            n += 1.0;
            let an = -n * (n - a);
            b += 2.0;
            d = an * d + b;
            if math::abs(d) < tiny {
                d = tiny;
            }
            c = b + an / c;
            if math::abs(c) < tiny {
                c = tiny;
            }
            d = 1.0 / d;
            let delta = d * c;
            h *= delta;
            if math::abs(delta - 1.0) < EPSILON {
                return Ok((1.0 - math::exp(prefix) * h).clamp(0.0, 1.0));
            }
        }
        Err(not_converged(h))
    }
}

/// Chi-square distribution function with `dof` degrees of freedom.
///
/// # Errors
///
/// [`KernelError::NotConverged`]; not reached for `dof ≤ 100`.
pub fn chi_square_cdf(x: f64, dof: u32) -> Result<f64> {
    gamma_p(f64::from(dof) / 2.0, x / 2.0)
}

/// Bisection for the `x` at which a decreasing or increasing function
/// crosses `target`, from `[low, high]`.
fn bisect(
    mut low: f64,
    mut high: f64,
    target: f64,
    increasing: bool,
    f: impl Fn(f64) -> Result<f64>,
) -> Result<f64> {
    for _ in 0..MAX_ITERATIONS {
        let middle = f64::midpoint(low, high);
        // Converged, or at the resolution of `f64` where the midpoint is an
        // end of the interval.
        if high - low <= EPSILON * high.max(1.0) || !(low < middle && middle < high) {
            return Ok(middle);
        }
        let below = f(middle)? < target;
        if below == increasing {
            low = middle;
        } else {
            high = middle;
        }
    }
    Err(not_converged(high - low))
}

/// Quantile: the `x` with `P(χ²(dof) ≤ x) = probability`.
///
/// # Errors
///
/// [`KernelError::OutOfRange`] unless `0 < probability < 1` and `dof ≥ 1`;
/// [`KernelError::NotConverged`].
pub fn chi_square_quantile(probability: f64, dof: u32) -> Result<f64> {
    kinavis_kernel::error::ensure_range(
        "probability",
        probability,
        f64::MIN_POSITIVE,
        1.0 - f64::EPSILON,
    )?;
    kinavis_kernel::error::ensure_range(
        "degrees of freedom",
        f64::from(dof),
        1.0,
        f64::from(u32::MAX),
    )?;
    let mut high = f64::from(dof) + 10.0 * math::sqrt(2.0 * f64::from(dof)) + 50.0;
    for _ in 0..64 {
        if chi_square_cdf(high, dof)? >= probability {
            return bisect(0.0, high, probability, true, |x| chi_square_cdf(x, dof));
        }
        high *= 2.0;
    }
    Err(not_converged(high))
}

/// Noncentral chi-square distribution function: `dof` degrees of freedom,
/// noncentrality `lambda`, as the Poisson mixture of central ones.
///
/// # Errors
///
/// [`KernelError::NotConverged`] if the mixture does not settle.
pub fn noncentral_chi_square_cdf(x: f64, dof: u32, lambda: f64) -> Result<f64> {
    if lambda <= 0.0 {
        return chi_square_cdf(x, dof);
    }
    let half = lambda / 2.0;
    let mut sum = 0.0;
    let mut weights = 0.0;
    let mut j = 0.0_f64;
    let mut freedom = dof;
    for _ in 0..MAX_ITERATIONS {
        let weight = math::exp(-half + j * math::ln(half) - ln_gamma(j + 1.0));
        sum += weight * chi_square_cdf(x, freedom)?;
        weights += weight;
        // Past the Poisson peak the weights only fall; once one no longer
        // registers against the total, the rest of the tail cannot either.
        if j > half && weight <= f64::EPSILON * weights * 1e-2 {
            return Ok(sum.clamp(0.0, 1.0));
        }
        j += 1.0;
        freedom = freedom.saturating_add(2);
    }
    Err(not_converged(1.0 - weights))
}

/// The noncentrality that a fault must reach for the test statistic to stay
/// below `threshold_squared` with probability `missed_detection`: its square
/// root is the bias multiplier of the protection levels.
///
/// # Errors
///
/// [`KernelError::OutOfRange`] unless `0 < missed_detection < 1`;
/// [`KernelError::NotConverged`].
pub fn bias_noncentrality(threshold_squared: f64, dof: u32, missed_detection: f64) -> Result<f64> {
    kinavis_kernel::error::ensure_range(
        "probability of missed detection",
        missed_detection,
        f64::MIN_POSITIVE,
        1.0 - f64::EPSILON,
    )?;
    if chi_square_cdf(threshold_squared, dof)? <= missed_detection {
        return Ok(0.0);
    }
    let mut high = 1.0;
    for _ in 0..64 {
        if noncentral_chi_square_cdf(threshold_squared, dof, high)? <= missed_detection {
            return bisect(0.0, high, missed_detection, false, |lambda| {
                noncentral_chi_square_cdf(threshold_squared, dof, lambda)
            });
        }
        high *= 2.0;
    }
    Err(not_converged(high))
}
