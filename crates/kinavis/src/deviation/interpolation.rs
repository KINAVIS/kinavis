//! Interpolation of a deviation table between nodes.
//!
//! Each method is a type implementing [`Interpolator`], not a branch in the
//! table. Preparation is separate from evaluation because two methods solve a
//! system first (spline moments, least-squares fit), which a batch should pay
//! once.
//!
//! All methods read nodes through a [`NodeRing`], so the arc from the last node
//! through `360°/0°` to the first is an ordinary segment.

use crate::error::{KernelError, NavigationError, Result};
use crate::linalg::{solve_cyclic_tridiagonal, CyclicSystem, CYCLIC_SCRATCH_PER_UNKNOWN};

use super::ring::NodeRing;
use super::smith::{self, DeviationCoefficients, SmithCoefficients};
use super::table::{DeviationTable, MAX_TABLE_NODES};

/// Interpolation method between table nodes.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[non_exhaustive]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum InterpolationMethod {
    /// Periodic linear interpolation.
    ///
    /// Default: exact at nodes, never overshoots, needs two nodes.
    #[default]
    Linear,
    /// Periodic cubic spline, C² continuous.
    ///
    /// Exact at nodes and smooth across `360°/0°`. Falls back to
    /// [`InterpolationMethod::Linear`] below three nodes.
    Cubic,
    /// Classical five-coefficient model, least-squares fit.
    ///
    /// `δ = A + B·sin(y) + C·cos(y) + D·sin(2y) + E·cos(2y)`
    ///
    /// A fit, not an interpolation: it does not reproduce the nodes, which
    /// smooths observation noise.
    Parametric,
    /// Periodic shape-preserving cubic (Fritsch–Carlson).
    ///
    /// Smooth like [`InterpolationMethod::Cubic`] but monotone between nodes:
    /// the curve stays between adjacent node values and adds no spurious
    /// extrema. A cubic spline can overshoot at an abrupt step, producing
    /// deviations outside any observed value.
    ///
    /// C¹ rather than C²; the better default when values matter more than
    /// curvature.
    ShapePreserving,
}

/// Interpolation method with optional pinned coefficients.
///
/// All conversions in [`crate::navigation_solutions`] take `impl
/// Into<Interpolation>`: pass a bare [`InterpolationMethod`] normally, this
/// struct to pin coefficients.
///
/// # Example
///
/// ```rust
/// use kinavis::{
///     navigation_solutions::convert_compass_course_to_true_course, CompassCourse,
///     DeviationCoefficients, DeviationTable, Interpolation, InterpolationMethod, Variation,
/// };
///
/// let table = DeviationTable::from_deviations(&[0.0; 36])?;
/// let coefficients = DeviationCoefficients {
///     a: Some(1.0),
///     ..DeviationCoefficients::default()
/// };
///
/// // A bare method...
/// let plain = convert_compass_course_to_true_course(
///     CompassCourse::new(10.0)?,
///     Variation::ZERO,
///     &table,
///     InterpolationMethod::Linear,
/// )?;
/// assert_eq!(plain.deviation.degrees(), 0.0);
///
/// // ...or a method with coefficients held fixed.
/// let pinned = convert_compass_course_to_true_course(
///     CompassCourse::new(10.0)?,
///     Variation::ZERO,
///     &table,
///     Interpolation {
///         method: InterpolationMethod::Parametric,
///         coefficients: Some(&coefficients),
///     },
/// )?;
/// assert!((pinned.deviation.degrees() - 1.0).abs() < 1e-9);
/// # Ok::<(), kinavis::NavigationError>(())
/// ```
#[derive(Debug, Clone, Copy, Default)]
pub struct Interpolation<'a> {
    /// Method.
    pub method: InterpolationMethod,
    /// Pinned coefficients, for [`InterpolationMethod::Parametric`].
    pub coefficients: Option<&'a DeviationCoefficients>,
}

impl From<InterpolationMethod> for Interpolation<'_> {
    fn from(method: InterpolationMethod) -> Self {
        Self {
            method,
            coefficients: None,
        }
    }
}

/// Prepared interpolation method, ready to evaluate.
///
/// New methods are new implementing types, not new branches in the table.
pub(crate) trait Interpolator {
    /// Deviation at `course`, which must be in `[0.0, 360.0)`.
    fn evaluate(&self, course: f64) -> f64;

    /// Estimated uncertainty, degrees.
    fn uncertainty(&self, course: f64) -> f64;
}

/// Periodic linear interpolation.
#[derive(Debug, Clone, Copy)]
pub(crate) struct Linear<'a> {
    ring: NodeRing<'a>,
}

impl Interpolator for Linear<'_> {
    fn evaluate(&self, course: f64) -> f64 {
        let segment = self.ring.locate(course);
        let start_value = self.ring.value(segment.index);
        let end_value = self.ring.value(self.ring.after(segment.index));
        start_value + (end_value - start_value) * segment.fraction()
    }

    fn uncertainty(&self, course: f64) -> f64 {
        self.ring.local_error_bound(course)
    }
}

/// One value per node, inline.
type PerNode = [f64; MAX_TABLE_NODES];

/// Periodic cubic spline with second derivatives at the nodes.
#[derive(Debug, Clone, Copy)]
pub(crate) struct CubicSpline<'a> {
    ring: NodeRing<'a>,
    moments: PerNode,
}

impl Interpolator for CubicSpline<'_> {
    fn evaluate(&self, course: f64) -> f64 {
        let segment = self.ring.locate(course);
        let count = self.ring.count().max(1);
        let start_value = self.ring.value(segment.index);
        let end_value = self.ring.value(self.ring.after(segment.index));
        let start_moment = self
            .moments
            .get(segment.index % count)
            .copied()
            .unwrap_or(0.0);
        let end_moment = self
            .moments
            .get(self.ring.after(segment.index))
            .copied()
            .unwrap_or(0.0);

        let span = segment.span;
        let slope =
            (end_value - start_value) / span - span * (2.0 * start_moment + end_moment) / 6.0;
        let offset = segment.offset;

        start_value
            + slope * offset
            + start_moment / 2.0 * offset * offset
            + (end_moment - start_moment) / (6.0 * span) * offset * offset * offset
    }

    fn uncertainty(&self, course: f64) -> f64 {
        self.ring.local_error_bound(course)
    }
}

/// Periodic shape-preserving cubic with tangents at the nodes.
#[derive(Debug, Clone, Copy)]
pub(crate) struct ShapePreserving<'a> {
    ring: NodeRing<'a>,
    slopes: PerNode,
}

impl Interpolator for ShapePreserving<'_> {
    fn evaluate(&self, course: f64) -> f64 {
        let segment = self.ring.locate(course);
        let count = self.ring.count().max(1);
        let start_value = self.ring.value(segment.index);
        let end_value = self.ring.value(self.ring.after(segment.index));
        let start_slope = self
            .slopes
            .get(segment.index % count)
            .copied()
            .unwrap_or(0.0);
        let end_slope = self
            .slopes
            .get(self.ring.after(segment.index))
            .copied()
            .unwrap_or(0.0);

        // Cubic Hermite basis on the unit interval.
        let span = segment.span;
        let t = segment.fraction();
        let complement = 1.0 - t;
        let start_weight = (1.0 + 2.0 * t) * complement * complement;
        let start_tangent = t * complement * complement;
        let end_weight = t * t * (3.0 - 2.0 * t);
        let end_tangent = t * t * (t - 1.0);

        start_value * start_weight
            + span * start_slope * start_tangent
            + end_value * end_weight
            + span * end_slope * end_tangent
    }

    fn uncertainty(&self, course: f64) -> f64 {
        self.ring.local_error_bound(course)
    }
}

/// Fitted five-coefficient model and its residual.
#[derive(Debug, Clone, Copy)]
pub(crate) struct Parametric {
    coefficients: SmithCoefficients,
    rms_residual: f64,
}

impl Interpolator for Parametric {
    fn evaluate(&self, course: f64) -> f64 {
        self.coefficients.deviation_at_degrees(course)
    }

    /// The fit does not pass through the nodes, so its residual is the
    /// uncertainty estimate, uniform over courses.
    fn uncertainty(&self, _course: f64) -> f64 {
        self.rms_residual
    }
}

/// Method selected at preparation.
///
/// An enum rather than a boxed trait object: no allocation and no indirect
/// call, as required by the allocator-free build.
#[derive(Debug, Clone, Copy)]
pub(crate) enum Prepared<'a> {
    /// See [`Linear`].
    Linear(Linear<'a>),
    /// See [`CubicSpline`].
    Cubic(CubicSpline<'a>),
    /// See [`ShapePreserving`].
    ShapePreserving(ShapePreserving<'a>),
    /// See [`Parametric`].
    Parametric(Parametric),
}

impl Interpolator for Prepared<'_> {
    fn evaluate(&self, course: f64) -> f64 {
        match self {
            Self::Linear(method) => method.evaluate(course),
            Self::Cubic(method) => method.evaluate(course),
            Self::ShapePreserving(method) => method.evaluate(course),
            Self::Parametric(method) => method.evaluate(course),
        }
    }

    fn uncertainty(&self, course: f64) -> f64 {
        match self {
            Self::Linear(method) => method.uncertainty(course),
            Self::Cubic(method) => method.uncertainty(course),
            Self::ShapePreserving(method) => method.uncertainty(course),
            Self::Parametric(method) => method.uncertainty(course),
        }
    }
}

/// Prepares the chosen method for evaluation.
///
/// # Errors
///
/// - [`KernelError::SingularSystem`] if the cyclic spline system is singular.
/// - Errors of [`smith::fit`] for the parametric method.
pub(crate) fn prepare<'a>(
    table: &'a DeviationTable,
    interpolation: Interpolation<'_>,
) -> Result<Prepared<'a>> {
    let ring = NodeRing::new(table.nodes());
    match interpolation.method {
        InterpolationMethod::Linear => Ok(Prepared::Linear(Linear { ring })),
        InterpolationMethod::Cubic => {
            // The cyclic spline system needs at least three nodes.
            if ring.count() < 3 {
                return Ok(Prepared::Linear(Linear { ring }));
            }
            match second_derivatives(ring) {
                Some(moments) => Ok(Prepared::Cubic(CubicSpline { ring, moments })),
                None => Err(NavigationError::Kernel(KernelError::SingularSystem {
                    context: "the periodic cubic spline",
                })),
            }
        }
        InterpolationMethod::Parametric => {
            let requested = interpolation.coefficients.copied().unwrap_or_default();
            let coefficients = smith::fit(table.nodes(), &requested)?;
            let rms_residual = smith::residuals(table.nodes(), &coefficients).rms;
            Ok(Prepared::Parametric(Parametric {
                coefficients,
                rms_residual,
            }))
        }
        InterpolationMethod::ShapePreserving => Ok(Prepared::ShapePreserving(ShapePreserving {
            ring,
            slopes: shape_preserving_slopes(ring),
        })),
    }
}

/// Node tangents for the shape-preserving cubic (Fritsch–Carlson).
///
/// Zero at extrema; elsewhere the weighted harmonic mean of the adjacent
/// secants, which prevents overshoot.
fn shape_preserving_slopes(ring: NodeRing<'_>) -> PerNode {
    let count = ring.count();
    let mut slopes = [0.0; MAX_TABLE_NODES];
    for (index, slot) in slopes.iter_mut().enumerate().take(count) {
        let previous = (index + count - 1) % count;
        let (before, after) = (ring.span(previous), ring.span(index));
        if before <= 0.0 || after <= 0.0 {
            continue;
        }
        let secant_before = (ring.value(index) - ring.value(previous)) / before;
        let secant_after = (ring.value(ring.after(index)) - ring.value(index)) / after;

        // Extremum or flat segment: zero tangent so the curve cannot overshoot
        // either neighbour.
        if secant_before * secant_after <= 0.0 {
            continue;
        }
        let weight_before = 2.0 * after + before;
        let weight_after = after + 2.0 * before;
        *slot = (weight_before + weight_after)
            / (weight_before / secant_before + weight_after / secant_after);
    }
    slopes
}

/// Second derivatives of the periodic cubic spline, one per node.
///
/// Solves the cyclic tridiagonal moment system; `None` if singular. All buffers
/// are inline, so no allocator is needed; the stack frame scales with
/// [`MAX_TABLE_NODES`], not with this table's node count.
fn second_derivatives(ring: NodeRing<'_>) -> Option<PerNode> {
    let count = ring.count();
    if !(3..=MAX_TABLE_NODES).contains(&count) {
        return None;
    }

    let mut sub = [0.0; MAX_TABLE_NODES];
    let mut diag = [0.0; MAX_TABLE_NODES];
    let mut sup = [0.0; MAX_TABLE_NODES];
    let mut rhs = [0.0; MAX_TABLE_NODES];

    for index in 0..count {
        let previous = (index + count - 1) % count;
        let gap_before = ring.span(previous);
        let gap_after = ring.span(index);

        let slope_before = (ring.value(index) - ring.value(previous)) / gap_before;
        let slope_after = (ring.value(ring.after(index)) - ring.value(index)) / gap_after;

        *sub.get_mut(index)? = gap_before;
        *diag.get_mut(index)? = 2.0 * (gap_before + gap_after);
        *sup.get_mut(index)? = gap_after;
        *rhs.get_mut(index)? = 6.0 * (slope_after - slope_before);
    }

    // Row 0 couples to node n-1 and row n-1 to node 0; those entries are the
    // matrix corners.
    let corner_top_right = *sub.first()?;
    let corner_bottom_left = *sup.get(count - 1)?;
    *sub.first_mut()? = 0.0;
    *sup.get_mut(count - 1)? = 0.0;

    let mut moments = [0.0; MAX_TABLE_NODES];
    let mut scratch = [0.0; MAX_TABLE_NODES * CYCLIC_SCRATCH_PER_UNKNOWN];
    let system = CyclicSystem {
        sub: sub.get(..count)?,
        diag: diag.get(..count)?,
        sup: sup.get(..count)?,
        corner_top_right,
        corner_bottom_left,
        rhs: rhs.get(..count)?,
    };
    let solved = solve_cyclic_tridiagonal(&system, moments.get_mut(..count)?, &mut scratch);
    solved.then_some(moments)
}
