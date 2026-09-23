//! Course and bearing conversions; current triangle.
//!
//! # Sign convention
//!
//! Corrections are applied in one direction and removed in the other:
//!
//! ```text
//! magnetic course = compass course  + deviation(compass course)
//! true course     = magnetic course + variation
//! ```
//!
//! Deviation is a function of the **compass** course, so converting true →
//! compass means solving the implicit equation `CC + δ(CC) = MC` for `CC`.
//! [`convert_true_course_to_compass_course`] solves it exactly, making both
//! directions true inverses. Reading the deviation at the *magnetic* course
//! instead is up to 10° wrong on a realistic swing.
//!
//! # Fallibility
//!
//! Adding or subtracting a known correction on validated types cannot fail and
//! returns a value. Only table reads and implicit solves return `Result`.
//!
//! Layout: infallible corrections and gyro error; compass conversions via a
//! [`CompassModel`](crate::CompassModel); the inverse solver; the current
//! triangle. `convert_*` are the `*_by` functions plus a report, sharing one
//! computation.

mod compass;
mod corrections;
mod current;
mod solver;
#[cfg(test)]
mod tests;

use crate::angle::{Deviation, Direction, Frame, TrueCourse, Variation};
use crate::units::{Angle, Speed};

pub use compass::{
    compass_to_magnetic_by, compass_to_true_by, convert_compass_course_to_magnetic_course,
    convert_compass_course_to_true_course, convert_magnetic_course_to_compass_course,
    convert_true_course_to_compass_course, magnetic_to_compass_by, true_to_compass_by,
};
pub use corrections::{
    bearing_from_relative, calculate_course_angle, compass_to_magnetic, gyro_error_from_transit,
    gyro_speed_error, gyro_to_true, magnetic_to_compass, magnetic_to_true, true_to_gyro,
    true_to_magnetic,
};
pub use current::{course_over_ground, course_to_steer, estimate_current};

/// Variation magnitude above which [`Advisories::large_variation`] is set.
///
/// Rare outside high latitudes; usually indicates an outdated chart, a sign
/// error, or a deviation entered as variation.
pub const LARGE_VARIATION_DEG: f64 = 15.0;

/// Deviation magnitude above which [`Advisories::large_deviation`] is set. Such
/// a compass should be adjusted, not just tabulated.
pub const LARGE_DEVIATION_DEG: f64 = 10.0;

/// Maximum node gap above which [`Advisories::coarse_table`] is set.
/// Interpolating across more than a quadrant is guesswork for any method.
pub const COARSE_TABLE_GAP_DEG: f64 = 45.0;

/// Maximum iterations of the compass-course solver.
///
/// Every loop is bounded by a named constant, and every constant reported in a
/// [`KernelError::NotConverged`](crate::KernelError::NotConverged) is public.
///
/// The damped fixed-point iteration reaches [`TOLERANCE_INVERSE_DEVIATION_DEG`]
/// in a few steps on any realistic swing; the budget leaves room for steep but
/// invertible curves.
pub const MAX_ITERATIONS_INVERSE_DEVIATION: u32 = 64;

/// Convergence tolerance of the compass-course solver, degrees; far below
/// steering resolution.
pub const TOLERANCE_INVERSE_DEVIATION_DEG: f64 = 1e-9;

/// Maximum bisections of the fallback for non-invertible curves.
///
/// Used only when the plain iteration oscillates. 80 halvings of a ≤ 360°
/// bracket exceed `f64` resolution, so the tolerance, not this count, is
/// limiting.
pub const MAX_BISECTIONS_INVERSE_DEVIATION: u32 = 80;

/// Latitude beyond which [`gyro_speed_error`] refuses.
///
/// The horizontal component of Earth rotation, which aligns the gyro, vanishes
/// at the pole; well before that settling is too sluggish and the speed error
/// too large for the correction to be meaningful.
pub const MAX_GYRO_LATITUDE_DEG: f64 = 85.0;

/// Conditions to review before acting on a result.
///
/// Not errors: the computation is exact for its input. They flag unusual data.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
// Independent flags by design.
#[allow(clippy::struct_excessive_bools)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Advisories {
    /// Variation magnitude exceeds [`LARGE_VARIATION_DEG`].
    pub large_variation: bool,
    /// Interpolated deviation magnitude exceeds [`LARGE_DEVIATION_DEG`].
    pub large_deviation: bool,
    /// Maximum node gap exceeds [`COARSE_TABLE_GAP_DEG`].
    ///
    /// An eight-point swing (45° spacing) does not set this; it is normal
    /// practice.
    pub coarse_table: bool,
    /// Table not uniquely invertible; see
    /// [`crate::DeviationTable::is_invertible`].
    ///
    /// A true course converted back to compass still produces the requested
    /// true course, but may not be the original compass course: several
    /// headings give the same result.
    pub non_invertible_table: bool,
}

impl Advisories {
    /// Whether any advisory is set.
    #[must_use]
    pub const fn any(self) -> bool {
        self.large_variation
            || self.large_deviation
            || self.coarse_table
            || self.non_invertible_table
    }
}

/// Converted course with its inputs.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(bound = "")
)]
pub struct CourseSolution<F: Frame> {
    /// Converted course.
    pub course: Direction<F>,
    /// Deviation used, interpolated at the compass course.
    pub deviation: Deviation,
    /// Variation used; zero for conversions not involving true north.
    pub variation: Variation,
    /// Total correction applied, `variation + deviation`, degrees.
    pub total_correction: f64,
    /// Approximate uncertainty of the interpolated deviation, degrees.
    ///
    /// For [`crate::InterpolationMethod::Linear`] and
    /// [`crate::InterpolationMethod::Cubic`]: the classical error bound from
    /// the local second difference; for
    /// [`crate::InterpolationMethod::Parametric`]: the RMS residual of the fit.
    /// Covers interpolation only, not the quality of the swing.
    pub estimated_error: f64,
    /// Conditions to review before acting on the result.
    pub advisories: Advisories,
}

impl<F: Frame> CourseSolution<F> {
    /// Whether any advisory is set.
    #[must_use]
    pub const fn check_data_required(&self) -> bool {
        self.advisories.any()
    }
}

// Defined in the kernel so the read model can carry it; re-exported here.
pub use kinavis_kernel::snapshot::GroundTrack;

/// Steering solution for a required track.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SteeringSolution {
    /// Heading to steer through the water.
    pub heading: TrueCourse,
    /// Speed made good along the track.
    pub speed_over_ground: Speed,
    /// Drift angle between heading and track, positive to starboard.
    pub drift_angle: Angle,
}

// Defined in the kernel so the environment ports can return it; re-exported
// here.
pub use kinavis_kernel::environment::Current;
