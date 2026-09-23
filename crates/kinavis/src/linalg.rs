//! Small dense and banded linear solvers.
//!
//! Numeric kernels for the periodic spline, the parametric deviation fit and
//! the least-squares fix. Systems have at most a few tens of unknowns, so
//! direct factorisation is simpler and faster than iteration.
//!
//! Each solver returns `false` on a singular system instead of panicking or
//! dividing by zero, and writes into caller-supplied storage: no allocation.

use crate::math;

/// Scratch slots per unknown for [`solve_cyclic_tridiagonal`].
pub(crate) const CYCLIC_SCRATCH_PER_UNKNOWN: usize = 5;

/// Tridiagonal system with two corner entries.
///
/// `sub[i]` multiplies `x[i-1]`, `diag[i]` `x[i]`, `sup[i]` `x[i+1]`;
/// `corner_top_right` is at `(0, n-1)`, `corner_bottom_left` at `(n-1, 0)`.
pub(crate) struct CyclicSystem<'a> {
    /// Sub-diagonal.
    pub(crate) sub: &'a [f64],
    /// Main diagonal.
    pub(crate) diag: &'a [f64],
    /// Super-diagonal.
    pub(crate) sup: &'a [f64],
    /// Entry `(0, n-1)`.
    pub(crate) corner_top_right: f64,
    /// Entry `(n-1, 0)`.
    pub(crate) corner_bottom_left: f64,
    /// Right-hand side.
    pub(crate) rhs: &'a [f64],
}

/// Solves a cyclic tridiagonal system (Sherman–Morrison).
///
/// Writes into `out` (system length); `scratch` needs
/// [`CYCLIC_SCRATCH_PER_UNKNOWN`] slots per unknown.
///
/// Returns `false`, `out` untouched, if singular or the slice lengths disagree.
pub(crate) fn solve_cyclic_tridiagonal(
    system: &CyclicSystem<'_>,
    out: &mut [f64],
    scratch: &mut [f64],
) -> bool {
    let CyclicSystem {
        sub,
        diag,
        sup,
        corner_top_right,
        corner_bottom_left,
        rhs,
    } = *system;
    let count = diag.len();
    if count < 3
        || sub.len() != count
        || sup.len() != count
        || rhs.len() != count
        || out.len() != count
        || scratch.len() < count * CYCLIC_SCRATCH_PER_UNKNOWN
    {
        return false;
    }

    let Some((modified_diag, rest)) = scratch.split_at_mut_checked(count) else {
        return false;
    };
    let Some((correction, rest)) = rest.split_at_mut_checked(count) else {
        return false;
    };
    let Some((solved_rhs, rest)) = rest.split_at_mut_checked(count) else {
        return false;
    };
    let Some((solved_correction, inner_scratch)) = rest.split_at_mut_checked(count) else {
        return false;
    };

    // Rank-one correction A = T + u·vᵀ keeping T strictly tridiagonal.
    let Some(&first_diag) = diag.first() else {
        return false;
    };
    let Some(&last_diag) = diag.last() else {
        return false;
    };
    let gamma = -first_diag;
    if math::abs(gamma) < f64::EPSILON {
        return false;
    }
    let ratio = corner_top_right / gamma;

    let last = count - 1;
    // Length-checked copy: `copy_from_slice` would panic on mismatch.
    for (slot, &value) in modified_diag.iter_mut().zip(diag.iter()) {
        *slot = value;
    }
    if !write(modified_diag, 0, first_diag - gamma)
        || !write(modified_diag, last, last_diag - corner_bottom_left * ratio)
    {
        return false;
    }

    correction.fill(0.0);
    if !write(correction, 0, gamma) || !write(correction, last, corner_bottom_left) {
        return false;
    }

    if !solve_tridiagonal(sub, modified_diag, sup, rhs, solved_rhs, inner_scratch) {
        return false;
    }
    if !solve_tridiagonal(
        sub,
        modified_diag,
        sup,
        correction,
        solved_correction,
        inner_scratch,
    ) {
        return false;
    }

    let (Some(&rhs_first), Some(&rhs_last)) = (solved_rhs.first(), solved_rhs.last()) else {
        return false;
    };
    let (Some(&correction_first), Some(&correction_last)) =
        (solved_correction.first(), solved_correction.last())
    else {
        return false;
    };

    let numerator = rhs_first + ratio * rhs_last;
    let denominator = 1.0 + correction_first + ratio * correction_last;
    if math::abs(denominator) < f64::EPSILON {
        return false;
    }
    let factor = numerator / denominator;

    for ((slot, &value), &adjustment) in out
        .iter_mut()
        .zip(solved_rhs.iter())
        .zip(solved_correction.iter())
    {
        *slot = value - factor * adjustment;
    }
    true
}

/// Writes one slot, reporting instead of panicking when out of bounds.
///
/// Indices are bounded by the caller's length checks; this avoids an indexing
/// panic path in the binary.
fn write(slice: &mut [f64], index: usize, value: f64) -> bool {
    match slice.get_mut(index) {
        Some(slot) => {
            *slot = value;
            true
        }
        None => false,
    }
}

/// Thomas algorithm for a strictly tridiagonal system.
///
/// Writes into `out`; `scratch` at least system length. Returns `false` on a
/// vanishing pivot or length mismatch.
///
/// All access is checked: the strict profile is verified free of
/// `core::panicking`, and indexing would add it.
pub(crate) fn solve_tridiagonal(
    sub: &[f64],
    diag: &[f64],
    sup: &[f64],
    rhs: &[f64],
    out: &mut [f64],
    scratch: &mut [f64],
) -> bool {
    let count = diag.len();
    if count == 0
        || sub.len() != count
        || sup.len() != count
        || rhs.len() != count
        || out.len() != count
        || scratch.len() < count
    {
        return false;
    }
    let Some(sweep) = scratch.get_mut(..count) else {
        return false;
    };

    let (Some(&first_diag), Some(&first_sup), Some(&first_rhs)) =
        (diag.first(), sup.first(), rhs.first())
    else {
        return false;
    };
    if math::abs(first_diag) < f64::EPSILON {
        return false;
    }
    if !write(sweep, 0, first_sup / first_diag) || !write(out, 0, first_rhs / first_diag) {
        return false;
    }

    for index in 1..count {
        let (Some(&this_diag), Some(&this_sub), Some(&this_sup), Some(&this_rhs)) = (
            diag.get(index),
            sub.get(index),
            sup.get(index),
            rhs.get(index),
        ) else {
            return false;
        };
        let (Some(&previous_sweep), Some(&previous_out)) =
            (sweep.get(index - 1), out.get(index - 1))
        else {
            return false;
        };

        let pivot = this_diag - this_sub * previous_sweep;
        if math::abs(pivot) < f64::EPSILON {
            return false;
        }
        if !write(sweep, index, this_sup / pivot)
            || !write(out, index, (this_rhs - this_sub * previous_out) / pivot)
        {
            return false;
        }
    }

    for index in (0..count - 1).rev() {
        let (Some(&this_sweep), Some(&next_out), Some(&this_out)) =
            (sweep.get(index), out.get(index + 1), out.get(index))
        else {
            return false;
        };
        if !write(out, index, this_out - this_sweep * next_out) {
            return false;
        }
    }

    true
}

/// Gaussian elimination with partial pivoting, row-major `size × size`.
///
/// Uses `matrix` and `rhs` as scratch; writes the solution into `out`. Returns
/// `false` if singular or on length mismatch.
///
/// Checked access throughout, as in [`solve_tridiagonal`].
pub(crate) fn solve_dense(
    matrix: &mut [f64],
    rhs: &mut [f64],
    size: usize,
    out: &mut [f64],
) -> bool {
    if size == 0 || matrix.len() != size * size || rhs.len() != size || out.len() != size {
        return false;
    }

    for column in 0..size {
        let mut pivot_row = column;
        let Some(&leading) = matrix.get(column * size + column) else {
            return false;
        };
        let mut best = math::abs(leading);
        for row in (column + 1)..size {
            let Some(&candidate) = matrix.get(row * size + column) else {
                return false;
            };
            if math::abs(candidate) > best {
                best = math::abs(candidate);
                pivot_row = row;
            }
        }
        if best < 1e-12 {
            return false;
        }
        if pivot_row != column {
            for index in 0..size {
                let (Some(here), Some(there)) =
                    (column.checked_mul(size), pivot_row.checked_mul(size))
                else {
                    return false;
                };
                if here + index >= matrix.len() || there + index >= matrix.len() {
                    return false;
                }
                matrix.swap(here + index, there + index);
            }
            if column >= rhs.len() || pivot_row >= rhs.len() {
                return false;
            }
            rhs.swap(column, pivot_row);
        }

        let Some(&pivot) = matrix.get(column * size + column) else {
            return false;
        };
        for row in (column + 1)..size {
            let Some(&leading) = matrix.get(row * size + column) else {
                return false;
            };
            let factor = leading / pivot;
            // Exact comparison on purpose: a skip, not a singularity test. A
            // zero factor leaves the row unchanged; any other value must be
            // applied. Singularity is decided by the pivot search with a real
            // tolerance.
            if factor == 0.0 {
                continue;
            }
            for index in column..size {
                let (Some(&pivot_cell), Some(&row_cell)) = (
                    matrix.get(column * size + index),
                    matrix.get(row * size + index),
                ) else {
                    return false;
                };
                if !write(matrix, row * size + index, row_cell - factor * pivot_cell) {
                    return false;
                }
            }
            let (Some(&pivot_rhs), Some(&row_rhs)) = (rhs.get(column), rhs.get(row)) else {
                return false;
            };
            if !write(rhs, row, row_rhs - factor * pivot_rhs) {
                return false;
            }
        }
    }

    for row in (0..size).rev() {
        let Some(&mut mut accumulator) = rhs.get_mut(row) else {
            return false;
        };
        for column in (row + 1)..size {
            let (Some(&cell), Some(&solved)) = (matrix.get(row * size + column), out.get(column))
            else {
                return false;
            };
            accumulator -= cell * solved;
        }
        let Some(&diagonal) = matrix.get(row * size + row) else {
            return false;
        };
        if !write(out, row, accumulator / diagonal) {
            return false;
        }
    }
    true
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp, clippy::indexing_slicing)]
mod tests {
    use super::*;

    /// Solves into a new buffer, for readable tests.
    fn dense(matrix: &mut [f64], rhs: &mut [f64], size: usize) -> Option<[f64; 4]> {
        let mut out = [0.0; 4];
        solve_dense(matrix, rhs, size, &mut out[..size]).then_some(out)
    }

    fn tridiagonal(sub: &[f64], diag: &[f64], sup: &[f64], rhs: &[f64]) -> Option<[f64; 4]> {
        let mut out = [0.0; 4];
        let mut scratch = [0.0; 4];
        let count = diag.len();
        solve_tridiagonal(sub, diag, sup, rhs, &mut out[..count], &mut scratch).then_some(out)
    }

    fn cyclic(
        sub: &[f64],
        diag: &[f64],
        sup: &[f64],
        corner_tr: f64,
        corner_bl: f64,
        rhs: &[f64],
    ) -> Option<[f64; 4]> {
        let mut out = [0.0; 4];
        let mut scratch = [0.0; 4 * CYCLIC_SCRATCH_PER_UNKNOWN];
        let count = diag.len();
        let system = CyclicSystem {
            sub,
            diag,
            sup,
            corner_top_right: corner_tr,
            corner_bottom_left: corner_bl,
            rhs,
        };
        solve_cyclic_tridiagonal(&system, &mut out[..count], &mut scratch).then_some(out)
    }

    #[test]
    fn dense_solver_matches_a_hand_solution() {
        // 2x +  y = 5 ; x + 3y = 10  =>  x = 1, y = 3
        let solution = dense(&mut [2.0, 1.0, 1.0, 3.0], &mut [5.0, 10.0], 2).unwrap();
        assert!((solution[0] - 1.0).abs() < 1e-12);
        assert!((solution[1] - 3.0).abs() < 1e-12);
    }

    #[test]
    fn dense_solver_pivots() {
        // A zero leading pivot must be swapped, not divided by.
        let solution = dense(&mut [0.0, 1.0, 1.0, 0.0], &mut [2.0, 3.0], 2).unwrap();
        assert!((solution[0] - 3.0).abs() < 1e-12);
        assert!((solution[1] - 2.0).abs() < 1e-12);
    }

    #[test]
    fn dense_solver_rejects_singular_systems() {
        assert!(dense(&mut [1.0, 2.0, 2.0, 4.0], &mut [1.0, 2.0], 2).is_none());
        assert!(dense(&mut [], &mut [], 0).is_none());
    }

    #[test]
    fn tridiagonal_solver_matches_a_hand_solution() {
        // 2x - y = 1 ; -x + 2y - z = 0 ; -y + 2z = 1  =>  x = y = z = 1
        let solution = tridiagonal(
            &[0.0, -1.0, -1.0],
            &[2.0, 2.0, 2.0],
            &[-1.0, -1.0, 0.0],
            &[1.0, 0.0, 1.0],
        )
        .unwrap();
        for value in solution.iter().take(3) {
            assert!((value - 1.0).abs() < 1e-12);
        }
    }

    #[test]
    fn solvers_reject_mismatched_lengths() {
        assert!(tridiagonal(&[0.0], &[1.0, 2.0], &[0.0], &[1.0]).is_none());
        assert!(cyclic(&[0.0], &[1.0], &[0.0], 1.0, 1.0, &[1.0]).is_none());
    }

    #[test]
    fn solvers_refuse_a_short_output_or_scratch() {
        let mut out = [0.0; 2];
        let mut scratch = [0.0; 3];
        assert!(!solve_tridiagonal(
            &[0.0, -1.0, -1.0],
            &[2.0, 2.0, 2.0],
            &[-1.0, -1.0, 0.0],
            &[1.0, 0.0, 1.0],
            &mut out,
            &mut scratch,
        ));

        let mut out = [0.0; 3];
        let mut too_small = [0.0; 3];
        let system = CyclicSystem {
            sub: &[0.0, 1.0, 1.0],
            diag: &[4.0, 4.0, 4.0],
            sup: &[1.0, 1.0, 0.0],
            corner_top_right: 1.0,
            corner_bottom_left: 1.0,
            rhs: &[1.0, 2.0, 3.0],
        };
        assert!(!solve_cyclic_tridiagonal(&system, &mut out, &mut too_small));
    }

    #[test]
    fn cyclic_solver_matches_a_dense_solution() {
        // 4 × 4 cyclic tridiagonal system, solved both ways.
        let sub = [0.0, 1.0, 1.0, 1.0];
        let diag = [4.0, 4.0, 4.0, 4.0];
        let sup = [1.0, 1.0, 1.0, 0.0];
        let (corner_tr, corner_bl) = (1.0, 1.0);
        let rhs = [1.0, 2.0, 3.0, 4.0];

        let banded = cyclic(&sub, &diag, &sup, corner_tr, corner_bl, &rhs).unwrap();

        let mut dense_matrix = [0.0; 16];
        for row in 0..4 {
            dense_matrix[row * 4 + row] = diag[row];
            if row > 0 {
                dense_matrix[row * 4 + row - 1] = sub[row];
            }
            if row < 3 {
                dense_matrix[row * 4 + row + 1] = sup[row];
            }
        }
        dense_matrix[3] = corner_tr;
        dense_matrix[12] = corner_bl;
        let reference = dense(&mut dense_matrix, &mut rhs.clone(), 4).unwrap();

        for (left, right) in banded.iter().zip(reference.iter()) {
            assert!((left - right).abs() < 1e-10, "{left} vs {right}");
        }
    }
}
