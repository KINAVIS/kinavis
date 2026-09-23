//! Small dense matrices with checked access, for estimators.
//!
//! Dimensions are type parameters: shape mismatches do not compile, nothing
//! allocates. Elements are accessed through methods, not indexing: the
//! workspace denies `indexing_slicing`, since a bounds panic aborts the
//! process. Every loop is bounded by a compile-time dimension.
//!
//! Not a general linear-algebra library: only what a Kalman filter needs —
//! products, transposes, sums, and a Cholesky factorisation that reports
//! non-positive-definite input — panic-free and deterministic.

use core::array;
use core::fmt;
use core::ops::{Add, Mul, Sub};

use crate::math;

/// Dense `R × C` `f64` matrix.
#[derive(Clone, Copy, PartialEq)]
pub struct Matrix<const R: usize, const C: usize> {
    rows: [[f64; C]; R],
}

/// Column vector of `N` elements.
pub type Vector<const N: usize> = Matrix<N, 1>;

impl<const R: usize, const C: usize> Matrix<R, C> {
    /// Zero matrix.
    pub const ZERO: Self = Self {
        rows: [[0.0; C]; R],
    };

    /// From rows.
    #[must_use]
    pub const fn from_rows(rows: [[f64; C]; R]) -> Self {
        Self { rows }
    }

    /// Element `(row, column)` from a closure.
    #[must_use]
    pub fn from_fn(mut element: impl FnMut(usize, usize) -> f64) -> Self {
        Self {
            rows: array::from_fn(|row| array::from_fn(|column| element(row, column))),
        }
    }

    /// Rows.
    #[must_use]
    pub const fn rows(&self) -> &[[f64; C]; R] {
        &self.rows
    }

    /// Element at `(row, column)`; `None` outside.
    #[must_use]
    pub fn get(&self, row: usize, column: usize) -> Option<f64> {
        self.rows.get(row)?.get(column).copied()
    }

    /// Sets element `(row, column)`; returns `false` and does nothing outside
    /// the matrix.
    pub fn set(&mut self, row: usize, column: usize, value: f64) -> bool {
        match self.rows.get_mut(row).and_then(|r| r.get_mut(column)) {
            Some(slot) => {
                *slot = value;
                true
            }
            None => false,
        }
    }

    /// Element at an index known to be in range (from `array::from_fn` over the
    /// same dimensions).
    fn at(&self, row: usize, column: usize) -> f64 {
        self.get(row, column).unwrap_or(0.0)
    }

    /// Transpose.
    #[must_use]
    pub fn transpose(&self) -> Matrix<C, R> {
        Matrix::from_fn(|row, column| self.at(column, row))
    }

    /// Scalar multiple.
    #[must_use]
    pub fn scaled(&self, factor: f64) -> Self {
        Self::from_fn(|row, column| self.at(row, column) * factor)
    }

    /// Whether all elements are finite.
    #[must_use]
    pub fn is_finite(&self) -> bool {
        self.rows.iter().flatten().all(|value| value.is_finite())
    }

    /// Maximum absolute element.
    #[must_use]
    pub fn max_abs(&self) -> f64 {
        self.rows
            .iter()
            .flatten()
            .fold(0.0, |largest, &value| largest.max(math::abs(value)))
    }
}

impl<const N: usize> Matrix<N, N> {
    /// Identity.
    #[must_use]
    pub fn identity() -> Self {
        Self::from_fn(|row, column| if row == column { 1.0 } else { 0.0 })
    }

    /// Diagonal matrix.
    #[must_use]
    pub fn diagonal(values: [f64; N]) -> Self {
        Self::from_fn(|row, column| {
            if row == column {
                values.get(row).copied().unwrap_or(0.0)
            } else {
                0.0
            }
        })
    }

    /// Trace.
    #[must_use]
    pub fn trace(&self) -> f64 {
        (0..N).map(|index| self.at(index, index)).sum()
    }

    /// `(A + Aᵀ) / 2`: exactly symmetric; unchanged for a symmetric input.
    ///
    /// Removes the last-place asymmetries a covariance accumulates through
    /// products.
    #[must_use]
    pub fn symmetrised(&self) -> Self {
        Self::from_fn(|row, column| f64::midpoint(self.at(row, column), self.at(column, row)))
    }

    /// Cholesky factor `L` with `self = L Lᵀ`; `None` unless symmetric positive
    /// definite.
    ///
    /// A pivot ≤ 0 within a matrix-scaled tolerance (an unobserved state can
    /// have exactly zero variance) means not positive definite; the result is
    /// `None`, not a factor of `NaN`s. Non-iterative: N³/6 operations.
    #[must_use]
    pub fn cholesky(&self) -> Option<Cholesky<N>> {
        let tolerance = self.max_abs() * f64::EPSILON * math::count_to_f64(N);
        let mut lower = Self::ZERO;
        for j in 0..N {
            let mut diagonal = self.at(j, j);
            for k in 0..j {
                diagonal -= lower.at(j, k) * lower.at(j, k);
            }
            if !diagonal.is_finite() || diagonal < -tolerance {
                return None;
            }
            // Zero pivot: the state has zero variance; the column is zero and
            // the matrix is semi-definite.
            let pivot = if diagonal <= tolerance {
                0.0
            } else {
                math::sqrt(diagonal)
            };
            lower.set(j, j, pivot);
            for i in (j + 1)..N {
                let mut sum = self.at(i, j);
                for k in 0..j {
                    sum -= lower.at(i, k) * lower.at(j, k);
                }
                let value = if pivot > 0.0 { sum / pivot } else { 0.0 };
                if !value.is_finite() {
                    return None;
                }
                lower.set(i, j, value);
            }
        }
        Some(Cholesky { lower })
    }

    /// Whether symmetric (relative tolerance) and positive semi-definite, as
    /// judged by [`Matrix::cholesky`].
    #[must_use]
    pub fn is_covariance(&self) -> bool {
        let scale = self.max_abs().max(f64::MIN_POSITIVE);
        let symmetric = (0..N).all(|row| {
            (0..N).all(|column| {
                math::abs(self.at(row, column) - self.at(column, row)) <= scale * 1e-9
            })
        });
        symmetric && self.is_finite() && self.cholesky().is_some()
    }
}

/// Cholesky factor of a positive definite matrix, for solving.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Cholesky<const N: usize> {
    lower: Matrix<N, N>,
}

impl<const N: usize> Cholesky<N> {
    /// Lower-triangular factor `L`.
    #[must_use]
    pub const fn lower(&self) -> &Matrix<N, N> {
        &self.lower
    }

    /// Solves `A X = B` with `A = L Lᵀ`.
    ///
    /// `None` if a zero pivot lies along a direction `B` requires: singular in
    /// that direction.
    #[must_use]
    pub fn solve<const M: usize>(&self, rhs: &Matrix<N, M>) -> Option<Matrix<N, M>> {
        // Forward substitution: L Y = B.
        let mut y = Matrix::<N, M>::ZERO;
        for column in 0..M {
            for i in 0..N {
                let mut sum = rhs.at(i, column);
                for k in 0..i {
                    sum -= self.lower.at(i, k) * y.at(k, column);
                }
                let pivot = self.lower.at(i, i);
                if pivot == 0.0 {
                    if math::abs(sum) > 0.0 {
                        return None;
                    }
                    y.set(i, column, 0.0);
                } else {
                    y.set(i, column, sum / pivot);
                }
            }
        }
        // Back substitution: Lᵀ X = Y.
        let mut x = Matrix::<N, M>::ZERO;
        for column in 0..M {
            for i in (0..N).rev() {
                let mut sum = y.at(i, column);
                for k in (i + 1)..N {
                    sum -= self.lower.at(k, i) * x.at(k, column);
                }
                let pivot = self.lower.at(i, i);
                if pivot == 0.0 {
                    if math::abs(sum) > 0.0 {
                        return None;
                    }
                    x.set(i, column, 0.0);
                } else {
                    x.set(i, column, sum / pivot);
                }
            }
        }
        x.is_finite().then_some(x)
    }

    /// Determinant: squared product of the pivots.
    #[must_use]
    pub fn determinant(&self) -> f64 {
        let product: f64 = (0..N).map(|index| self.lower.at(index, index)).product();
        product * product
    }
}

impl<const N: usize> Vector<N> {
    /// From elements.
    #[must_use]
    pub fn from_column(values: [f64; N]) -> Self {
        Self::from_fn(|row, _| values.get(row).copied().unwrap_or(0.0))
    }

    /// Elements.
    #[must_use]
    pub fn to_column(&self) -> [f64; N] {
        array::from_fn(|row| self.at(row, 0))
    }

    /// Element at `row`; `None` outside.
    #[must_use]
    pub fn element(&self, row: usize) -> Option<f64> {
        self.get(row, 0)
    }

    /// Dot product.
    #[must_use]
    pub fn dot(&self, other: &Self) -> f64 {
        (0..N).map(|row| self.at(row, 0) * other.at(row, 0)).sum()
    }

    /// Euclidean norm.
    #[must_use]
    pub fn norm(&self) -> f64 {
        math::sqrt(self.dot(self))
    }
}

impl<const R: usize, const C: usize> Add for Matrix<R, C> {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        Self::from_fn(|row, column| self.at(row, column) + other.at(row, column))
    }
}

impl<const R: usize, const C: usize> Sub for Matrix<R, C> {
    type Output = Self;

    fn sub(self, other: Self) -> Self {
        Self::from_fn(|row, column| self.at(row, column) - other.at(row, column))
    }
}

impl<const R: usize, const K: usize, const C: usize> Mul<Matrix<K, C>> for Matrix<R, K> {
    type Output = Matrix<R, C>;

    fn mul(self, other: Matrix<K, C>) -> Matrix<R, C> {
        Matrix::from_fn(|row, column| (0..K).map(|k| self.at(row, k) * other.at(k, column)).sum())
    }
}

impl<const R: usize, const C: usize> fmt::Debug for Matrix<R, C> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_list().entries(self.rows.iter()).finish()
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;

    #[test]
    fn products_and_transposes_come_out_right() {
        let a = Matrix::from_rows([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]);
        let b = Matrix::from_rows([[7.0, 8.0], [9.0, 10.0], [11.0, 12.0]]);
        let product = a * b;
        assert_eq!(product.rows(), &[[58.0, 64.0], [139.0, 154.0]]);
        assert_eq!(a.transpose().rows(), &[[1.0, 4.0], [2.0, 5.0], [3.0, 6.0]]);
        assert_eq!((a + a).rows(), &a.scaled(2.0).rows().clone());
        assert_eq!((a - a), Matrix::ZERO);
        assert_eq!(a.get(1, 2), Some(6.0));
        assert_eq!(a.get(2, 0), None);
        let mut c = a;
        assert!(!c.set(5, 5, 1.0));
        assert!(c.set(0, 0, 9.0));
        assert_eq!(c.get(0, 0), Some(9.0));
        assert_eq!(Matrix::<3, 3>::identity().trace(), 3.0);
    }

    #[test]
    fn cholesky_factors_and_solves_a_positive_definite_system() {
        let a = Matrix::from_rows([
            [4.0, 12.0, -16.0],
            [12.0, 37.0, -43.0],
            [-16.0, -43.0, 98.0],
        ]);
        let factor = a.cholesky().unwrap();
        assert_eq!(
            factor.lower().rows(),
            &[[2.0, 0.0, 0.0], [6.0, 1.0, 0.0], [-8.0, 5.0, 3.0]]
        );
        assert!((factor.determinant() - 36.0).abs() < 1e-9);
        let b = Vector::from_column([1.0, 2.0, 3.0]);
        let x = factor.solve(&b).unwrap();
        let residual = a * x - b;
        assert!(residual.max_abs() < 1e-12);
        assert!(a.is_covariance());
    }

    #[test]
    fn an_indefinite_matrix_has_no_factor() {
        let indefinite = Matrix::from_rows([[1.0, 2.0], [2.0, 1.0]]);
        assert!(indefinite.cholesky().is_none());
        assert!(!indefinite.is_covariance());
        let asymmetric = Matrix::from_rows([[1.0, 0.5], [0.0, 1.0]]);
        assert!(!asymmetric.is_covariance());
        let nan = Matrix::from_rows([[f64::NAN, 0.0], [0.0, 1.0]]);
        assert!(nan.cholesky().is_none());
    }

    #[test]
    fn a_semidefinite_matrix_factors_with_a_zero_pivot_and_solves_where_it_can() {
        // Variance in the first state only.
        let semidefinite = Matrix::from_rows([[4.0, 0.0], [0.0, 0.0]]);
        let factor = semidefinite.cholesky().unwrap();
        assert_eq!(factor.lower().rows(), &[[2.0, 0.0], [0.0, 0.0]]);
        assert!(semidefinite.is_covariance());
        // Solvable in the first direction, not the second.
        assert!(factor.solve(&Vector::from_column([2.0, 0.0])).is_some());
        assert!(factor.solve(&Vector::from_column([0.0, 1.0])).is_none());
    }

    #[test]
    fn symmetrising_removes_round_off_asymmetry() {
        let slightly_off = Matrix::from_rows([[1.0, 0.5 + 1e-16], [0.5 - 1e-16, 1.0]]);
        let fixed = slightly_off.symmetrised();
        assert_eq!(fixed.get(0, 1), fixed.get(1, 0));
    }

    #[test]
    fn vectors_dot_and_measure() {
        let v = Vector::from_column([3.0, 4.0]);
        assert_eq!(v.norm(), 5.0);
        assert_eq!(v.dot(&v), 25.0);
        assert_eq!(v.to_column(), [3.0, 4.0]);
        assert_eq!(v.element(1), Some(4.0));
        assert_eq!(v.element(2), None);
        assert_eq!(Matrix::<2, 2>::diagonal([1.0, 2.0]).trace(), 3.0);
    }
}
