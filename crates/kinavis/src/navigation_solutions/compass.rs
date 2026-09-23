//! Compass ↔ magnetic ↔ true conversions through deviation.
//!
//! Two families, one computation. `*_by` functions take any [`CompassModel`]
//! and return a course; `convert_*` functions take a [`DeviationTable`] and an
//! interpolation, read the table through the same port, and add a
//! [`CourseSolution`] report. Towards true, deviation and variation are
//! applied; backwards, the compass course is solved for in
//! [`solver`](super::solver).

use crate::angle::{
    Compass, CompassCourse, Deviation, Direction, Frame, Magnetic, MagneticCourse, True,
    TrueCourse, Variation,
};
use crate::deviation::{DeviationTable, InterpolatedTable, Interpolation};
use crate::environment::CompassModel;
use crate::error::{NavigationError, Result};
use crate::math;

use super::corrections::{compass_to_magnetic, magnetic_to_true, true_to_magnetic};
use super::solver::solve_compass_course;
use super::{
    Advisories, CourseSolution, COARSE_TABLE_GAP_DEG, LARGE_DEVIATION_DEG, LARGE_VARIATION_DEG,
};

// ---------------------------------------------------------------------------
// Conversions through a compass model
// ---------------------------------------------------------------------------

/// Compass course → true course, with deviation from any [`CompassModel`].
///
/// Unlike `convert_*`, which require a [`DeviationTable`] to report read
/// quality, this accepts any model: the table, an
/// [`InterpolatedTable`](crate::InterpolatedTable),
/// [`SmithCoefficients`](crate::SmithCoefficients), a calibrated sensor.
///
/// # Errors
///
/// Any model error.
///
/// # Example
///
/// ```rust
/// use kinavis::{
///     navigation_solutions::compass_to_true_by, CompassCourse, Deviation, DeviationTable,
///     Variation,
/// };
///
/// let mut table = DeviationTable::default();
/// table.set_deviation(0, Deviation::new(-2.5)?)?;
/// table.set_deviation(10, Deviation::new(-1.5)?)?;
///
/// let course = compass_to_true_by(CompassCourse::new(5.0)?, &table, Variation::new(-10.0)?)?;
/// assert_eq!(format!("{:.2}", course.degrees()), "353.00");
/// # Ok::<(), kinavis::NavigationError>(())
/// ```
pub fn compass_to_true_by(
    compass_course: CompassCourse,
    model: &impl CompassModel,
    variation: Variation,
) -> Result<TrueCourse> {
    let deviation = model.deviation(compass_course)?;
    Ok(magnetic_to_true(
        compass_to_magnetic(compass_course, deviation),
        variation,
    ))
}

/// True course → compass course, solving for the compass course the model's
/// deviation depends on.
///
/// # Errors
///
/// - Any model error on any course tried by the solver.
/// - [`KernelError::NotConverged`](crate::KernelError::NotConverged) if the
///   model is not invertible near this heading; bounds as in
///   [`convert_true_course_to_compass_course`].
///
/// # Example
///
/// ```rust
/// use kinavis::{
///     deviation::smith_coefficients,
///     navigation_solutions::{compass_to_true_by, true_to_compass_by},
///     CompassCourse, DeviationTable, Variation,
/// };
///
/// let table = DeviationTable::from_deviations(&[
///     -2.5, -0.5, 1.6, 4.4, -1.7, 0.0, 1.0, 0.3, -0.9, 0.5, -1.2, 0.8, -0.3, 1.7, -2.1, 0.4,
///     -0.6, 1.2, -1.3, 0.0, 0.9, -1.1, 1.5, -0.7, -3.2, -5.7, -7.9, -9.2, -8.1, 1.8, -0.4,
///     0.7, -0.2, 1.4, -4.4, -2.9,
/// ])?;
/// let model = smith_coefficients(&table)?;
/// let variation = Variation::new(-2.7)?;
///
/// let compass = CompassCourse::new(250.0)?;
/// let out = compass_to_true_by(compass, &model, variation)?;
/// let back = true_to_compass_by(out, &model, variation)?;
/// assert!((back.degrees() - compass.degrees()).abs() < 1e-9);
/// # Ok::<(), kinavis::NavigationError>(())
/// ```
pub fn true_to_compass_by(
    true_course: TrueCourse,
    model: &impl CompassModel,
    variation: Variation,
) -> Result<CompassCourse> {
    let magnetic = true_to_magnetic(true_course, variation);
    let compass_degrees = solve_compass_course(
        |compass| {
            model
                .deviation(CompassCourse::from_degrees_wrapped(compass))
                .map(Deviation::degrees)
                .map_err(NavigationError::from)
        },
        magnetic.degrees(),
    )?;
    Ok(CompassCourse::from_degrees_wrapped(compass_degrees))
}

/// Compass course → magnetic course, with deviation from any [`CompassModel`].
///
/// # Errors
///
/// Any model error.
pub fn compass_to_magnetic_by(
    compass_course: CompassCourse,
    model: &impl CompassModel,
) -> Result<MagneticCourse> {
    let deviation = model.deviation(compass_course)?;
    Ok(compass_to_magnetic(compass_course, deviation))
}

/// Magnetic course → compass course, solving for the model's deviation.
///
/// # Errors
///
/// As [`true_to_compass_by`].
pub fn magnetic_to_compass_by(
    magnetic_course: MagneticCourse,
    model: &impl CompassModel,
) -> Result<CompassCourse> {
    true_to_compass_by(magnetic_course.relabel::<True>(), model, Variation::ZERO)
}

// ---------------------------------------------------------------------------
// Conversions that read a deviation table, and say how well
// ---------------------------------------------------------------------------

/// Compass course → true course, applying tabulated deviation and variation.
///
/// # Errors
///
/// Deviation table errors: an unsupported parametric fit, or an interpolated
/// deviation that is not a valid angle.
///
/// # Example
///
/// ```rust
/// use kinavis::{
///     navigation_solutions::convert_compass_course_to_true_course, CompassCourse,
///     Deviation, DeviationTable, InterpolationMethod, Variation,
/// };
///
/// let mut table = DeviationTable::default();
/// table.set_deviation(0, Deviation::new(-2.5)?)?;
/// table.set_deviation(10, Deviation::new(-1.5)?)?;
///
/// let solution = convert_compass_course_to_true_course(
///     CompassCourse::new(5.0)?,
///     Variation::new(-10.0)?,
///     &table,
///     InterpolationMethod::Linear,
/// )?;
///
/// assert_eq!(format!("{:.2}", solution.course.degrees()), "353.00");
/// assert_eq!(format!("{:.2}", solution.deviation.degrees()), "-2.00");
/// # Ok::<(), kinavis::NavigationError>(())
/// ```
pub fn convert_compass_course_to_true_course<'a>(
    compass_course: Direction<Compass>,
    variation: Variation,
    deviation_table: &DeviationTable,
    interpolation: impl Into<Interpolation<'a>>,
) -> Result<CourseSolution<True>> {
    let model = deviation_table.interpolated(interpolation.into())?;
    let course = compass_to_true_by(compass_course, &model, variation)?;
    solution(course, compass_course, variation, deviation_table, &model)
}

/// True course → compass course, solving for the compass course the table is
/// indexed by.
///
/// # Errors
///
/// - Deviation table errors.
/// - [`KernelError::NotConverged`](crate::KernelError::NotConverged) if the
///   deviation curve is not invertible near this heading (the compass needs
///   re-swinging). The iteration count is
///   [`MAX_ITERATIONS_INVERSE_DEVIATION`](super::MAX_ITERATIONS_INVERSE_DEVIATION)
///   at
///   [`TOLERANCE_INVERSE_DEVIATION_DEG`](super::TOLERANCE_INVERSE_DEVIATION_DEG);
///   the preceding bracketing fallback is bounded by
///   [`MAX_BISECTIONS_INVERSE_DEVIATION`](super::MAX_BISECTIONS_INVERSE_DEVIATION).
///   All loops are bounded.
///
/// # Example
///
/// ```rust
/// use kinavis::{
///     navigation_solutions::{
///         convert_compass_course_to_true_course, convert_true_course_to_compass_course,
///     },
///     CompassCourse, DeviationTable, InterpolationMethod, Variation,
/// };
///
/// let table = DeviationTable::from_deviations(&[
///     -2.5, -0.5, 1.6, 4.4, -1.7, 0.0, 1.0, 0.3, -0.9, 0.5, -1.2, 0.8, -0.3, 1.7, -2.1, 0.4,
///     -0.6, 1.2, -1.3, 0.0, 0.9, -1.1, 1.5, -0.7, -13.2, -15.7, -17.9, -19.2, -18.1, 1.8,
///     -0.4, 0.7, -0.2, 1.4, -4.4, -2.9,
/// ])?;
/// let variation = Variation::new(-2.7)?;
///
/// // Out and back, over the steepest part of the curve.
/// let compass = CompassCourse::new(250.0)?;
/// let out = convert_compass_course_to_true_course(
///     compass, variation, &table, InterpolationMethod::Linear,
/// )?;
/// let back = convert_true_course_to_compass_course(
///     out.course, variation, &table, InterpolationMethod::Linear,
/// )?;
///
/// // Reading the deviation at the magnetic course would be 9.63° adrift here.
/// assert!((back.course.degrees() - compass.degrees()).abs() < 1e-9);
/// # Ok::<(), kinavis::NavigationError>(())
/// ```
pub fn convert_true_course_to_compass_course<'a>(
    true_course: Direction<True>,
    variation: Variation,
    deviation_table: &DeviationTable,
    interpolation: impl Into<Interpolation<'a>>,
) -> Result<CourseSolution<Compass>> {
    let model = deviation_table.interpolated(interpolation.into())?;
    let compass_course = true_to_compass_by(true_course, &model, variation)?;
    solution(
        compass_course,
        compass_course,
        variation,
        deviation_table,
        &model,
    )
}

/// Compass course → magnetic course, applying tabulated deviation.
///
/// # Errors
///
/// As [`convert_compass_course_to_true_course`].
pub fn convert_compass_course_to_magnetic_course<'a>(
    compass_course: Direction<Compass>,
    deviation_table: &DeviationTable,
    interpolation: impl Into<Interpolation<'a>>,
) -> Result<CourseSolution<Magnetic>> {
    let model = deviation_table.interpolated(interpolation.into())?;
    let course = compass_to_magnetic_by(compass_course, &model)?;
    solution(
        course,
        compass_course,
        Variation::ZERO,
        deviation_table,
        &model,
    )
}

/// Magnetic course → compass course, solving for tabulated deviation.
///
/// # Errors
///
/// As [`convert_true_course_to_compass_course`].
pub fn convert_magnetic_course_to_compass_course<'a>(
    magnetic_course: Direction<Magnetic>,
    deviation_table: &DeviationTable,
    interpolation: impl Into<Interpolation<'a>>,
) -> Result<CourseSolution<Compass>> {
    convert_true_course_to_compass_course(
        magnetic_course.relabel::<True>(),
        Variation::ZERO,
        deviation_table,
        interpolation,
    )
}

/// Conversion report: course, deviation on the compass course, and read
/// quality.
///
/// The deviation lookup is repeated here; it is the same lookup the conversion
/// used.
fn solution<F: Frame>(
    course: Direction<F>,
    compass_course: CompassCourse,
    variation: Variation,
    table: &DeviationTable,
    model: &InterpolatedTable<'_>,
) -> Result<CourseSolution<F>> {
    let deviation = model.deviation(compass_course)?;
    Ok(CourseSolution {
        course,
        deviation,
        variation,
        total_correction: variation.degrees() + deviation.degrees(),
        estimated_error: model.uncertainty(compass_course),
        advisories: Advisories {
            large_variation: math::abs(variation.degrees()) > LARGE_VARIATION_DEG,
            large_deviation: math::abs(deviation.degrees()) > LARGE_DEVIATION_DEG,
            coarse_table: table.max_gap() > COARSE_TABLE_GAP_DEG,
            non_invertible_table: !table.is_invertible(),
        },
    })
}
