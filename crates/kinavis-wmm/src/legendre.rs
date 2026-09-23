//! Schmidt semi-normalised associated Legendre functions and the derived tables
//! used by the field synthesis.
//!
//! For one geocentric latitude `φ′`, three tables are built and reused for
//! every coefficient:
//!
//! - `P̄ₙᵐ(sin φ′)` — vertical component;
//! - `dP̄ₙᵐ/dφ′` — north component;
//! - `P̄ₙᵐ / cos φ′` for `m ≥ 1` — east component. Dividing afterwards is `0/0`
//!   at the pole; since the functions carry a factor `cosᵐ φ′`, the quotient
//!   satisfies the same recurrences and is computed directly, so the pole is a
//!   regular point.
//!
//! The recurrences run on unnormalised functions without the Condon–Shortley
//! phase; the Schmidt factor `√((2 − δₘ₀)(n − m)!/(n + m)!)` is applied last.

use kinavis_kernel::math;

/// Maximum degree and order of the expansion.
pub const MAX_DEGREE: usize = 12;

/// Number of `(n, m)` pairs up to [`MAX_DEGREE`], `n = 0` included.
pub(crate) const TABLE_LEN: usize = (MAX_DEGREE + 1) * (MAX_DEGREE + 2) / 2;
/// Exclusive upper bound of the degree loops. A constant half-open bound lets
/// the compiler prove every index in range.
const DEGREES: usize = MAX_DEGREE + 1;

/// Index of degree `n`, order `m` in a triangular table.
///
/// `n, m ≤ MAX_DEGREE`, so nothing wraps; saturating arithmetic states this
/// where the compiler cannot carry the loop bound through recursion peeling.
const fn slot(n: usize, m: usize) -> usize {
    (n.saturating_mul(n.saturating_add(1)) / 2).saturating_add(m)
}

/// One value per `(n, m)`, stored inline.
#[derive(Debug, Clone, Copy)]
pub(crate) struct Table([f64; TABLE_LEN]);

impl Table {
    const ZERO: Self = Self([0.0; TABLE_LEN]);

    /// Value at degree `n`, order `m`; zero outside the table.
    pub(crate) fn at(&self, n: usize, m: usize) -> f64 {
        self.0.get(slot(n, m)).copied().unwrap_or(0.0)
    }

    fn set(&mut self, n: usize, m: usize, value: f64) {
        if let Some(entry) = self.0.get_mut(slot(n, m)) {
            *entry = value;
        }
    }
}

/// The three tables for one geocentric latitude.
#[derive(Debug, Clone, Copy)]
pub(crate) struct Legendre {
    /// `P̄ₙᵐ(sin φ′)`.
    pub(crate) value: Table,
    /// `dP̄ₙᵐ/dφ′`.
    pub(crate) derivative: Table,
    /// `P̄ₙᵐ / cos φ′`, defined for `m ≥ 1`; zero at `m = 0`.
    pub(crate) over_cosine: Table,
}

impl Legendre {
    /// Tables at geocentric latitude `phi`, in radians.
    // Notation follows the recurrences. Order loops are half-open: an inclusive
    // range carries a flag the compiler cannot see through, and the strict
    // profile would then flag the index arithmetic as wrapping.
    #[allow(clippy::many_single_char_names, clippy::range_plus_one)]
    pub(crate) fn at(phi: f64) -> Self {
        let x = math::sin(phi);
        let c = math::cos(phi);

        // Unnormalised functions, their derivative with respect to colatitude θ
        // (x = cos θ, c = sin θ, dx/dθ = −c, dc/dθ = x), and the quotient by c
        // for m ≥ 1. The recurrences are linear and c is a common factor, so
        // the quotient follows them with its own seed.
        let mut p = Table::ZERO;
        let mut dp = Table::ZERO;
        let mut q = Table::ZERO;
        p.set(0, 0, 1.0);

        for n in 1..DEGREES {
            // Previous degrees: `n ≥ 1` here; the three-term branch below needs
            // `n ≥ 2`, guaranteed by its guard.
            let (n_1, n_2) = (n - 1, n.saturating_sub(2));
            for m in 0..n + 1 {
                let (value, deriv, quot) = if m == n {
                    // Pₘᵐ = (2m − 1) · c · Pₘ₋₁ᵐ⁻¹, with m = n ≥ 1.
                    let k = 2.0 * count(m) - 1.0;
                    let seed = if m == 1 { 1.0 } else { k * c * q.at(n_1, n_1) };
                    (
                        k * c * p.at(n_1, n_1),
                        k * (c * dp.at(n_1, n_1) + x * p.at(n_1, n_1)),
                        seed,
                    )
                } else if m == n_1 {
                    // Pₘ₊₁ᵐ = (2m + 1) · x · Pₘᵐ
                    let k = 2.0 * count(m) + 1.0;
                    (
                        k * x * p.at(m, m),
                        k * (x * dp.at(m, m) - c * p.at(m, m)),
                        k * x * q.at(m, m),
                    )
                } else {
                    // Pₙᵐ = ((2n − 1) x Pₙ₋₁ᵐ − (n + m − 1) Pₙ₋₂ᵐ) / (n − m), m < n − 1
                    let a = 2.0 * count(n) - 1.0;
                    let b = count(n + m) - 1.0;
                    let d = count(n - m);
                    (
                        (a * x * p.at(n_1, m) - b * p.at(n_2, m)) / d,
                        (a * (x * dp.at(n_1, m) - c * p.at(n_1, m)) - b * dp.at(n_2, m)) / d,
                        (a * x * q.at(n_1, m) - b * q.at(n_2, m)) / d,
                    )
                };
                p.set(n, m, value);
                dp.set(n, m, deriv);
                if m >= 1 {
                    q.set(n, m, quot);
                }
            }
        }

        // Schmidt semi-normalisation, and dφ′ = −dθ.
        let mut value = Table::ZERO;
        let mut derivative = Table::ZERO;
        let mut over_cosine = Table::ZERO;
        value.set(0, 0, 1.0);
        for n in 1..DEGREES {
            for m in 0..n + 1 {
                let factor = schmidt_factor(n, m);
                value.set(n, m, factor * p.at(n, m));
                derivative.set(n, m, -factor * dp.at(n, m));
                over_cosine.set(n, m, factor * q.at(n, m));
            }
        }
        Self {
            value,
            derivative,
            over_cosine,
        }
    }
}

/// `√((2 − δₘ₀) (n − m)! / (n + m)!)`, computed as a product to avoid overflow.
fn schmidt_factor(n: usize, m: usize) -> f64 {
    if m == 0 {
        return 1.0;
    }
    // (n − m)! / (n + m)! = 1 / ((n − m + 1)(n − m + 2)…(n + m))
    let mut ratio = 2.0;
    for k in (n - m + 1)..=(n + m) {
        ratio /= count(k);
    }
    math::sqrt(ratio)
}

/// Small count as `f64`, for recurrence coefficients.
fn count(k: usize) -> f64 {
    math::count_to_f64(k)
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;

    #[test]
    fn low_degrees_match_the_closed_forms() {
        let phi = math::to_radians(37.0);
        let (x, c) = (math::sin(phi), math::cos(phi));
        let tables = Legendre::at(phi);

        // P̄₁⁰ = x, P̄₁¹ = c, P̄₂⁰ = (3x² − 1)/2, P̄₂¹ = √3·x·c, P̄₂² = (√3/2)·c².
        assert!((tables.value.at(1, 0) - x).abs() < 1e-15);
        assert!((tables.value.at(1, 1) - c).abs() < 1e-15);
        assert!((tables.value.at(2, 0) - (3.0 * x * x - 1.0) / 2.0).abs() < 1e-15);
        assert!((tables.value.at(2, 1) - math::sqrt(3.0) * x * c).abs() < 1e-15);
        assert!((tables.value.at(2, 2) - math::sqrt(3.0) / 2.0 * c * c).abs() < 1e-15);

        // Derivatives with respect to φ′.
        assert!((tables.derivative.at(1, 0) - c).abs() < 1e-15);
        assert!((tables.derivative.at(1, 1) + x).abs() < 1e-15);
        assert!((tables.derivative.at(2, 0) - 3.0 * x * c).abs() < 1e-15);
        assert!((tables.derivative.at(2, 1) - math::sqrt(3.0) * (c * c - x * x)).abs() < 1e-15);

        // The quotient is the function divided by the cosine.
        assert!((tables.over_cosine.at(2, 1) - tables.value.at(2, 1) / c).abs() < 1e-15);
        assert!((tables.over_cosine.at(2, 2) - tables.value.at(2, 2) / c).abs() < 1e-15);
    }

    #[test]
    fn the_quotient_is_finite_at_the_pole_and_agrees_just_short_of_it() {
        let pole = Legendre::at(math::to_radians(90.0));
        let near = Legendre::at(math::to_radians(89.999_999));
        for n in 1..=MAX_DEGREE {
            for m in 1..=n {
                assert!(pole.over_cosine.at(n, m).is_finite());
                let c = math::cos(math::to_radians(89.999_999));
                let by_division = near.value.at(n, m) / c;
                assert!(
                    (near.over_cosine.at(n, m) - by_division).abs() < 1e-9,
                    "n={n} m={m}"
                );
            }
        }
        // At the pole only m = 1 survives: P̄ₙ¹/cos φ′ → √(n(n+1)/2)·… is
        // finite and non-zero; every m ≥ 2 term vanishes.
        assert!(pole.over_cosine.at(1, 1).abs() > 0.5);
        assert!(pole.over_cosine.at(2, 2).abs() < 1e-12);
    }

    #[test]
    fn the_derivative_is_the_slope_of_the_function() {
        let phi = math::to_radians(-23.0);
        let h = 1e-6;
        let tables = Legendre::at(phi);
        let up = Legendre::at(phi + h);
        let down = Legendre::at(phi - h);
        for n in 1..=MAX_DEGREE {
            for m in 0..=n {
                let numerical = (up.value.at(n, m) - down.value.at(n, m)) / (2.0 * h);
                assert!(
                    (tables.derivative.at(n, m) - numerical).abs() < 1e-6,
                    "n={n} m={m}"
                );
            }
        }
    }
}
