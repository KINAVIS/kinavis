//! Table and coefficients as a [`CompassModel`].
//!
//! Conversions need deviation on a course regardless of source: a swung table
//! read linearly or by spline, or fitted coefficients. The kernel's
//! [`CompassModel`] port abstracts this; implementations are here.

use crate::angle::{CompassCourse, Deviation};
use crate::error::NavigationError;
use kinavis_kernel::environment::CompassModel;
use kinavis_kernel::error::Result;

use super::interpolation::{self, Interpolation, Interpolator, Prepared};
use super::smith::SmithCoefficients;
use super::table::DeviationTable;

/// Table read with the default
/// [`InterpolationMethod::Linear`](super::InterpolationMethod::Linear).
///
/// Linear is exact at nodes and never overshoots, the safe default. For a
/// smoother curve, or to prepare a spline once instead of per call, use
/// [`DeviationTable::interpolated`].
impl CompassModel for DeviationTable {
    fn deviation(&self, course: CompassCourse) -> Result<Deviation> {
        // Ports use kernel errors; table interpolation only fails in kernel
        // terms.
        let interpolator = interpolation::prepare(self, Interpolation::default())
            .map_err(NavigationError::into_kernel)?;
        Deviation::new(interpolator.evaluate(course.degrees()))
    }
}

/// Deviation table with a prepared interpolation.
///
/// Spline solve or parametric fit happens once at construction; each
/// [`deviation`](CompassModel::deviation) is then a lookup. Borrows the table,
/// which therefore cannot change underneath it.
#[derive(Debug, Clone, Copy)]
pub struct InterpolatedTable<'a> {
    prepared: Prepared<'a>,
}

impl DeviationTable {
    /// Table as a [`CompassModel`] with `interpolation`.
    ///
    /// # Errors
    ///
    /// As [`DeviationTable::deviation_at`]: singular spline system, or a
    /// parametric fit the table cannot support.
    ///
    /// # Example
    ///
    /// ```rust
    /// use kinavis::{CompassCourse, CompassModel, DeviationTable, InterpolationMethod};
    ///
    /// let table = DeviationTable::from_deviations(&[
    ///     -2.5, -0.5, 1.6, 4.4, -1.7, 0.0, 1.0, 0.3, -0.9, 0.5, -1.2, 0.8, -0.3, 1.7, -2.1, 0.4,
    ///     -0.6, 1.2, -1.3, 0.0, 0.9, -1.1, 1.5, -0.7, -3.2, -5.7, -7.9, -9.2, -8.1, 1.8,
    ///     -0.4, 0.7, -0.2, 1.4, -4.4, -2.9,
    /// ])?;
    /// let smooth = table.interpolated(InterpolationMethod::ShapePreserving)?;
    ///
    /// // At a node every method agrees with the table.
    /// let at_node = smooth.deviation(CompassCourse::new(30.0)?)?;
    /// assert!((at_node.degrees() - 4.4).abs() < 1e-12);
    /// # Ok::<(), kinavis::NavigationError>(())
    /// ```
    pub fn interpolated<'a>(
        &'a self,
        interpolation: impl Into<Interpolation<'a>>,
    ) -> crate::error::Result<InterpolatedTable<'a>> {
        Ok(InterpolatedTable {
            prepared: interpolation::prepare(self, interpolation.into())?,
        })
    }
}

impl InterpolatedTable<'_> {
    /// Approximate uncertainty of the reading on `course`, degrees:
    /// interpolation error bound from the local second difference, or the RMS
    /// residual of a parametric fit. Covers the reading only, not the quality
    /// of the swing.
    #[must_use]
    pub fn uncertainty(&self, course: CompassCourse) -> f64 {
        self.prepared.uncertainty(course.degrees())
    }
}

impl CompassModel for InterpolatedTable<'_> {
    fn deviation(&self, course: CompassCourse) -> Result<Deviation> {
        Deviation::new(self.prepared.evaluate(course.degrees()))
    }
}

/// Five-coefficient model as a [`CompassModel`].
///
/// [`SmithCoefficients::deviation_at`] returns a plain `f64` because a fit can
/// produce absurd values; here a [`Deviation`] is required, so such values
/// yield [`KernelError::OutOfRange`](crate::KernelError::OutOfRange).
impl CompassModel for SmithCoefficients {
    fn deviation(&self, course: CompassCourse) -> Result<Deviation> {
        Deviation::new(self.deviation_at(course))
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use crate::deviation::{smith_coefficients, InterpolationMethod};
    use crate::error::KernelError;

    fn table() -> DeviationTable {
        DeviationTable::from_deviations(&[
            -2.5, -0.5, 1.6, 4.4, -1.7, 0.0, 1.0, 0.3, -0.9, 0.5, -1.2, 0.8, -0.3, 1.7, -2.1, 0.4,
            -0.6, 1.2, -1.3, 0.0, 0.9, -1.1, 1.5, -0.7, -3.2, -5.7, -7.9, -9.2, -8.1, 1.8, -0.4,
            0.7, -0.2, 1.4, -4.4, -2.9,
        ])
        .unwrap()
    }

    /// Only the port is needed.
    fn through_the_port(model: &impl CompassModel, degrees: f64) -> f64 {
        model
            .deviation(CompassCourse::new(degrees).unwrap())
            .unwrap()
            .degrees()
    }

    #[test]
    fn a_bare_table_answers_the_port_linearly() {
        let table = table();
        let expected = table
            .deviation_at(
                CompassCourse::new(35.0).unwrap(),
                InterpolationMethod::Linear,
            )
            .unwrap();
        assert_eq!(through_the_port(&table, 35.0), expected.degrees());
        assert_eq!(through_the_port(&table, 30.0), 4.4);
    }

    #[test]
    fn an_interpolated_table_answers_with_its_method() {
        let table = table();
        for method in [
            InterpolationMethod::Linear,
            InterpolationMethod::Cubic,
            InterpolationMethod::ShapePreserving,
            InterpolationMethod::Parametric,
        ] {
            let model = table.interpolated(method).unwrap();
            for degrees in [0.0, 35.0, 123.4, 270.0, 359.9] {
                let expected = table
                    .deviation_at(CompassCourse::new(degrees).unwrap(), method)
                    .unwrap();
                assert_eq!(
                    through_the_port(&model, degrees),
                    expected.degrees(),
                    "{method:?} at {degrees}"
                );
            }
        }
    }

    #[test]
    fn the_coefficients_answer_the_port_and_refuse_the_absurd() {
        let coefficients = smith_coefficients(&table()).unwrap();
        assert_eq!(
            through_the_port(&coefficients, 77.0),
            coefficients.deviation_at(CompassCourse::new(77.0).unwrap())
        );

        let absurd = SmithCoefficients {
            a: 200.0,
            ..SmithCoefficients::default()
        };
        assert!(matches!(
            absurd.deviation(CompassCourse::NORTH),
            Err(KernelError::OutOfRange { .. })
        ));
    }

    #[test]
    fn the_port_takes_a_trait_object_too() {
        let table = table();
        let coefficients = smith_coefficients(&table).unwrap();
        let models: [&dyn CompassModel; 2] = [&table, &coefficients];
        for model in models {
            assert!(model.deviation(CompassCourse::NORTH).is_ok());
        }
    }
}
