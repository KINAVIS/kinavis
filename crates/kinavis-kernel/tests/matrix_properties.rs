//! Cholesky factorisation of matrices constructed to be factorable.

#![allow(clippy::expect_used, clippy::indexing_slicing)]

use kinavis_kernel::matrix::{Matrix, Vector};
use proptest::prelude::*;

/// Lower-triangular factor with positive diagonal, so `L Lᵀ` is positive
/// definite by construction.
fn any_factor() -> impl Strategy<Value = Matrix<4, 4>> {
    proptest::collection::vec(-10.0_f64..10.0, 16).prop_map(|values| {
        Matrix::from_fn(|row, column| {
            let value = values.get(row * 4 + column).copied().unwrap_or(0.0);
            match row.cmp(&column) {
                core::cmp::Ordering::Equal => value.abs() + 0.5,
                core::cmp::Ordering::Greater => value,
                core::cmp::Ordering::Less => 0.0,
            }
        })
    })
}

proptest! {
    #[test]
    fn a_product_of_a_factor_and_its_transpose_factors_back_and_solves(
        factor in any_factor(),
        rhs in proptest::collection::vec(-100.0_f64..100.0, 4),
    ) {
        let matrix = factor * factor.transpose();
        prop_assert!(matrix.is_covariance());
        let cholesky = matrix.cholesky().expect("positive definite");
        // With a positive diagonal the factor is unique.
        prop_assert!((*cholesky.lower() - factor).max_abs() < 1e-6 * factor.max_abs().max(1.0));
        let b = Vector::from_column([rhs[0], rhs[1], rhs[2], rhs[3]]);
        let x = cholesky.solve(&b).expect("solvable");
        let residual = matrix * x - b;
        prop_assert!(residual.max_abs() < 1e-6 * b.max_abs().max(1.0), "{residual:?}");
    }

    #[test]
    fn the_transpose_of_a_product_is_the_product_of_transposes(
        a in proptest::collection::vec(-10.0_f64..10.0, 6),
        b in proptest::collection::vec(-10.0_f64..10.0, 6),
    ) {
        let left: Matrix<2, 3> = Matrix::from_fn(|r, c| a[r * 3 + c]);
        let right: Matrix<3, 2> = Matrix::from_fn(|r, c| b[r * 2 + c]);
        let direct = (left * right).transpose();
        let swapped = right.transpose() * left.transpose();
        prop_assert!((direct - swapped).max_abs() < 1e-9);
    }
}
