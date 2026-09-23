//! Classical five-coefficient deviation model and its least-squares fit.
//!
//! ```text
//! δ = A + B·sin(y) + C·cos(y) + D·sin(2y) + E·cos(2y)
//! ```
//!
//! `A`: constant (index) error; `B`, `C`: semicircular deviation from permanent
//! magnetism; `D`, `E`: quadrantal deviation from soft iron. Fitting is a
//! service over a table, not a table property.

use crate::angle::{CompassCourse, MAX_DEVIATION_DEG};
use crate::error::{ensure_range, KernelError, NavigationError, Result};
use crate::linalg::solve_dense;
use crate::math;

/// Coefficient count: A, B, C, D, E.
const COEFFICIENTS: usize = 5;

use super::node::DeviationNode;
use super::table::DeviationTable;

/// Parametric model coefficients.
///
/// `None` fields are fitted by least squares; `Some` fields are held fixed and
/// the rest fitted around them.
///
/// # Example
///
/// ```rust
/// use kinavis::{
///     CompassCourse, DeviationCoefficients, DeviationTable, Interpolation, InterpolationMethod,
/// };
///
/// let table = DeviationTable::from_deviations(&[0.0; 36])?;
///
/// // Force a constant 1° index error, fit the rest.
/// let coefficients = DeviationCoefficients {
///     a: Some(1.0),
///     ..DeviationCoefficients::default()
/// };
///
/// let deviation = table.deviation_at(
///     CompassCourse::new(250.0)?,
///     Interpolation {
///         method: InterpolationMethod::Parametric,
///         coefficients: Some(&coefficients),
///     },
/// )?;
/// assert!((deviation.degrees() - 1.0).abs() < 1e-9);
/// # Ok::<(), kinavis::NavigationError>(())
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct DeviationCoefficients {
    /// Constant deviation (index or alignment error).
    pub a: Option<f64>,
    /// Semicircular, `sin(course)`.
    pub b: Option<f64>,
    /// Semicircular, `cos(course)`.
    pub c: Option<f64>,
    /// Quadrantal, `sin(2·course)`.
    pub d: Option<f64>,
    /// Quadrantal, `cos(2·course)`.
    pub e: Option<f64>,
}

impl DeviationCoefficients {
    fn as_array(self) -> [Option<f64>; 5] {
        [self.a, self.b, self.c, self.d, self.e]
    }

    fn validate(self) -> Result<()> {
        for (name, value) in [
            ("coefficient A", self.a),
            ("coefficient B", self.b),
            ("coefficient C", self.c),
            ("coefficient D", self.d),
            ("coefficient E", self.e),
        ] {
            if let Some(value) = value {
                ensure_range(name, value, -MAX_DEVIATION_DEG, MAX_DEVIATION_DEG)?;
            }
        }
        Ok(())
    }
}

/// Fully determined coefficients.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SmithCoefficients {
    /// Constant.
    pub a: f64,
    /// Semicircular, `sin(course)`.
    pub b: f64,
    /// Semicircular, `cos(course)`.
    pub c: f64,
    /// Quadrantal, `sin(2·course)`.
    pub d: f64,
    /// Quadrantal, `cos(2·course)`.
    pub e: f64,
}

impl SmithCoefficients {
    /// Model deviation at a compass course, degrees.
    ///
    /// Returns a plain `f64`, not a [`Deviation`](crate::Deviation): a poorly
    /// conditioned fit may produce absurd values, and reporting them is more
    /// useful than refusing.
    #[must_use]
    pub fn deviation_at(&self, course: CompassCourse) -> f64 {
        self.deviation_at_degrees(course.degrees())
    }

    /// As [`Self::deviation_at`] for a course already in `[0.0, 360.0)`,
    /// avoiding re-wrapping per node.
    pub(crate) fn deviation_at_degrees(&self, course_degrees: f64) -> f64 {
        let basis = parametric_basis(course_degrees);
        self.a * basis[0]
            + self.b * basis[1]
            + self.c * basis[2]
            + self.d * basis[3]
            + self.e * basis[4]
    }

    /// Converts to the partially specified form used by the interpolator.
    #[must_use]
    pub const fn as_input(&self) -> DeviationCoefficients {
        DeviationCoefficients {
            a: Some(self.a),
            b: Some(self.b),
            c: Some(self.c),
            d: Some(self.d),
            e: Some(self.e),
        }
    }

    fn from_array(values: [f64; 5]) -> Self {
        Self {
            a: values[0],
            b: values[1],
            c: values[2],
            d: values[3],
            e: values[4],
        }
    }
}

/// Least-squares fit to a table's nodes.
///
/// Pinned coefficients are held fixed and their contribution subtracted from
/// the observations; the rest are fitted.
///
/// # Errors
///
/// - [`KernelError::OutOfRange`] for an invalid pinned coefficient.
/// - [`KernelError::InsufficientData`] if there are fewer nodes than free
///   coefficients.
/// - [`KernelError::SingularSystem`] if the nodes do not constrain the model
///   (e.g. all on one semicircle).
pub(crate) fn fit(
    nodes: &[DeviationNode],
    requested: &DeviationCoefficients,
) -> Result<SmithCoefficients> {
    requested.validate()?;
    let fixed = requested.as_array();
    // Five coefficients: fixed-size buffers, no allocation regardless of table
    // size.
    let mut free = [0_usize; COEFFICIENTS];
    let mut free_count = 0;
    for (index, value) in fixed.iter().enumerate() {
        if value.is_none() {
            if let Some(slot) = free.get_mut(free_count) {
                *slot = index;
            }
            free_count = free_count.saturating_add(1);
        }
    }
    let free = free.get(..free_count).unwrap_or(&[]);

    let mut resolved = [0.0_f64; 5];
    for (index, value) in fixed.iter().enumerate() {
        if let (Some(slot), Some(value)) = (resolved.get_mut(index), *value) {
            *slot = value;
        }
    }

    if free.is_empty() {
        return Ok(SmithCoefficients::from_array(resolved));
    }

    if nodes.len() < free.len() {
        return Err(NavigationError::Kernel(KernelError::InsufficientData {
            found: nodes.len(),
            required: free.len(),
            context: "a parametric deviation fit",
        }));
    }

    // Normal equations over the free basis functions, with the fixed
    // contribution subtracted first.
    let size = free.len();
    let mut normal = [0.0; COEFFICIENTS * COEFFICIENTS];
    let mut target = [0.0; COEFFICIENTS];

    for node in nodes {
        let basis = parametric_basis(node.course_degrees());
        let mut residual = node.deviation_degrees();
        for (index, value) in fixed.iter().enumerate() {
            if let Some(value) = *value {
                residual -= value * basis.get(index).copied().unwrap_or(0.0);
            }
        }
        for (row, &row_index) in free.iter().enumerate() {
            let row_basis = basis.get(row_index).copied().unwrap_or(0.0);
            for (column, &column_index) in free.iter().enumerate() {
                let column_basis = basis.get(column_index).copied().unwrap_or(0.0);
                if let Some(cell) = normal.get_mut(row.saturating_mul(size).saturating_add(column))
                {
                    *cell += row_basis * column_basis;
                }
            }
            if let Some(cell) = target.get_mut(row) {
                *cell += row_basis * residual;
            }
        }
    }

    let mut solution = [0.0; COEFFICIENTS];
    let solved = solve_dense(
        normal
            .get_mut(..size.saturating_mul(size))
            .unwrap_or(&mut []),
        target.get_mut(..size).unwrap_or(&mut []),
        size,
        solution.get_mut(..size).unwrap_or(&mut []),
    );
    if !solved {
        return Err(NavigationError::Kernel(KernelError::SingularSystem {
            context: "a parametric deviation fit",
        }));
    }

    for (position, &index) in free.iter().enumerate() {
        if let (Some(slot), Some(value)) = (resolved.get_mut(index), solution.get(position)) {
            *slot = *value;
        }
    }

    Ok(SmithCoefficients::from_array(resolved))
}

/// RMS and maximum residual of a fit against its nodes; shared by the swing
/// summary and the parametric error estimate.
pub(crate) fn residuals(nodes: &[DeviationNode], coefficients: &SmithCoefficients) -> Residuals {
    let mut sum_squares = 0.0;
    let mut largest: f64 = 0.0;
    for node in nodes {
        let residual =
            node.deviation_degrees() - coefficients.deviation_at_degrees(node.course_degrees());
        sum_squares += residual * residual;
        largest = largest.max(math::abs(residual));
    }
    // Defensive: a table always has at least two nodes.
    Residuals {
        rms: math::sqrt(sum_squares / math::count_to_f64(nodes.len().max(1))),
        largest,
    }
}

/// Fit residuals, degrees.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Residuals {
    /// RMS over all nodes.
    pub(crate) rms: f64,
    /// Maximum residual.
    pub(crate) largest: f64,
}

/// The five basis functions at a course in degrees.
fn parametric_basis(course_degrees: f64) -> [f64; 5] {
    let radians = math::to_radians(course_degrees);
    [
        1.0,
        math::sin(radians),
        math::cos(radians),
        math::sin(2.0 * radians),
        math::cos(2.0 * radians),
    ]
}

/// Least-squares fit of the five-coefficient model to a table.
///
/// # Errors
///
/// - [`KernelError::InsufficientData`] for fewer than five nodes.
/// - [`KernelError::SingularSystem`] if the nodes do not constrain the model
///   (e.g. all on one semicircle).
///
/// # Example
///
/// ```rust
/// use kinavis::deviation::{smith_coefficients, DeviationTable};
///
/// let table = DeviationTable::from_deviations(&[1.0; 36])?;
/// let coefficients = smith_coefficients(&table)?;
/// assert!((coefficients.a - 1.0).abs() < 1e-9);
/// # Ok::<(), kinavis::NavigationError>(())
/// ```
pub fn smith_coefficients(table: &DeviationTable) -> Result<SmithCoefficients> {
    fit(table.nodes(), &DeviationCoefficients::default())
}

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::indexing_slicing,
    clippy::cast_precision_loss
)]
mod tests {
    use super::super::analysis::analyze;
    use super::super::interpolation::{Interpolation, InterpolationMethod};
    use super::super::table::DeviationTable;
    use super::super::test_support::{heading, readme_table};
    use super::*;

    #[test]
    fn parametric_fit_recovers_known_coefficients() {
        // The fit must depend on the deviation values, not just return the
        // table mean.
        let truth = SmithCoefficients {
            a: 1.0,
            b: -2.0,
            c: 3.0,
            d: 0.5,
            e: -1.5,
        };
        let values: [f64; 36] =
            core::array::from_fn(|index| truth.deviation_at(heading(index as f64 * 10.0)));
        let table = DeviationTable::from_deviations(&values).unwrap();

        let fitted = smith_coefficients(&table).unwrap();
        assert!((fitted.a - truth.a).abs() < 1e-9);
        assert!((fitted.b - truth.b).abs() < 1e-9);
        assert!((fitted.c - truth.c).abs() < 1e-9);
        assert!((fitted.d - truth.d).abs() < 1e-9);
        assert!((fitted.e - truth.e).abs() < 1e-9);

        let analysis = analyze(&table).unwrap();
        assert!(analysis.rms_residual < 1e-9);
        assert_eq!(analysis.nodes, 36);
        assert_eq!(analysis.max_gap, 10.0);
    }

    #[test]
    fn parametric_depends_on_the_deviation_values() {
        let flat = DeviationTable::from_deviations(&[0.0; 36]).unwrap();
        let values: [f64; 36] =
            core::array::from_fn(|index| 5.0 * math::sin(math::to_radians(index as f64 * 10.0)));
        let sinusoid = DeviationTable::from_deviations(&values).unwrap();

        let flat_value = flat
            .deviation_at(heading(90.0), InterpolationMethod::Parametric)
            .unwrap()
            .degrees();
        let sinusoid_value = sinusoid
            .deviation_at(heading(90.0), InterpolationMethod::Parametric)
            .unwrap()
            .degrees();

        assert!(flat_value.abs() < 1e-9);
        assert!(
            (sinusoid_value - 5.0).abs() < 1e-9,
            "expected 5.0 at 090°, got {sinusoid_value}"
        );
    }

    #[test]
    fn parametric_is_not_constant_across_the_compass() {
        let table = readme_table();
        let north = table
            .deviation_at(heading(0.0), InterpolationMethod::Parametric)
            .unwrap()
            .degrees();
        let west = table
            .deviation_at(heading(270.0), InterpolationMethod::Parametric)
            .unwrap()
            .degrees();
        assert!((north - west).abs() > 1.0, "{north} vs {west}");
    }

    #[test]
    fn parametric_honours_fixed_coefficients() {
        let table = readme_table();
        let requested = DeviationCoefficients {
            a: Some(0.0),
            b: Some(0.0),
            c: Some(0.0),
            d: Some(0.0),
            e: Some(0.0),
        };
        let value = table
            .deviation_at(
                heading(123.0),
                Interpolation {
                    method: InterpolationMethod::Parametric,
                    coefficients: Some(&requested),
                },
            )
            .unwrap();
        assert_eq!(value.degrees(), 0.0);

        let partial = DeviationCoefficients {
            a: Some(2.0),
            ..DeviationCoefficients::default()
        };
        let fitted = fit(table.nodes(), &partial).unwrap();
        assert_eq!(fitted.a, 2.0);
        assert!(fitted.b.abs() > 0.0 || fitted.c.abs() > 0.0);
    }

    #[test]
    fn parametric_needs_enough_nodes() {
        let table = DeviationTable::from_pairs(&[(0, 1.0), (180, -1.0)]).unwrap();
        assert!(matches!(
            smith_coefficients(&table).unwrap_err(),
            NavigationError::Kernel(KernelError::InsufficientData { required: 5, .. })
        ));
    }

    #[test]
    fn parametric_rejects_absurd_fixed_coefficients() {
        let table = readme_table();
        let requested = DeviationCoefficients {
            a: Some(1e6),
            ..DeviationCoefficients::default()
        };
        assert!(table
            .deviation_at(
                heading(0.0),
                Interpolation {
                    method: InterpolationMethod::Parametric,
                    coefficients: Some(&requested)
                }
            )
            .is_err());
    }
}
