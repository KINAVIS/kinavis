//! Infallible corrections and gyro error.
//!
//! Applying or removing a known correction to a validated direction always
//! succeeds, so these return values, not `Result`. Exception: the gyro speed
//! error, which has a domain.

use crate::angle::{
    wrap180, Compass, Deviation, Direction, Frame, Gyro, GyroCourse, Magnetic, MagneticCourse,
    RelativeBearing, True, TrueCourse, Variation,
};
use crate::error::{ensure_range, KernelError, NavigationError, Result};
use crate::math;
use crate::position::Latitude;
use crate::units::{Angle, Speed};

use super::current::ensure_speed;
use super::MAX_GYRO_LATITUDE_DEG;

/// Eastward surface speed at the equator, kn: 15° of longitude per hour × 60 NM
/// per degree.
const EARTH_SURFACE_SPEED_KNOTS: f64 = 900.0;

/// Applies variation: magnetic → true. Courses and bearings alike.
#[must_use]
pub fn magnetic_to_true(direction: MagneticCourse, variation: Variation) -> TrueCourse {
    Direction::<True>::from_degrees_wrapped(direction.degrees() + variation.degrees())
}

/// Removes variation: true → magnetic.
#[must_use]
pub fn true_to_magnetic(direction: TrueCourse, variation: Variation) -> MagneticCourse {
    Direction::<Magnetic>::from_degrees_wrapped(direction.degrees() - variation.degrees())
}

/// Applies deviation: compass → magnetic.
#[must_use]
pub fn compass_to_magnetic(
    direction: Direction<Compass>,
    deviation: Deviation,
) -> Direction<Magnetic> {
    Direction::<Magnetic>::from_degrees_wrapped(direction.degrees() + deviation.degrees())
}

/// Removes a known deviation: magnetic → compass.
///
/// The deviation is given. To look it up in a table (which requires solving for
/// the compass course), use
/// [`convert_magnetic_course_to_compass_course`](super::convert_magnetic_course_to_compass_course).
#[must_use]
pub fn magnetic_to_compass(
    direction: Direction<Magnetic>,
    deviation: Deviation,
) -> Direction<Compass> {
    Direction::<Compass>::from_degrees_wrapped(direction.degrees() - deviation.degrees())
}

/// Applies gyro error: gyro → true.
///
/// East-positive, `true = gyro + error`, as for [`Variation`] and
/// [`Deviation`].
#[must_use]
pub fn gyro_to_true(direction: GyroCourse, error: Angle) -> TrueCourse {
    Direction::<True>::from_degrees_wrapped(direction.degrees() + error.degrees())
}

/// Removes gyro error: true → gyro.
#[must_use]
pub fn true_to_gyro(direction: TrueCourse, error: Angle) -> GyroCourse {
    Direction::<Gyro>::from_degrees_wrapped(direction.degrees() - error.degrees())
}

/// Gyro error from a bearing of an object with known true bearing (transit,
/// distant object, celestial azimuth).
#[must_use]
pub fn gyro_error_from_transit(observed: Direction<Gyro>, reference: Direction<True>) -> Angle {
    Angle::from_degrees_unchecked(wrap180(reference.degrees() - observed.degrees()))
}

/// Gyrocompass speed error due to the ship's own motion.
///
/// The gyro settles along the resultant of the Earth's eastward surface
/// velocity and the ship's velocity: northerly courses tilt its meridian west,
/// southerly east, in both hemispheres. East-positive, for [`gyro_to_true`].
///
/// Speed error only; residual instrument error is observed separately with
/// [`gyro_error_from_transit`].
///
/// # Errors
///
/// - [`KernelError::NotFinite`] or [`KernelError::OutOfRange`] for a negative
///   or non-finite speed.
/// - [`KernelError::OutOfRange`] beyond [`MAX_GYRO_LATITUDE_DEG`], where a gyro
///   does not settle usefully.
/// - [`KernelError::Indeterminate`] if the westward ship speed exceeds the
///   Earth's surface speed (no settling meridian).
///
/// # Example
///
/// ```rust
/// use kinavis::navigation_solutions::gyro_speed_error;
/// use kinavis::{Latitude, NavigationError, Speed, TrueCourse};
///
/// fn main() -> Result<(), NavigationError> {
///     // 20 knots due north in latitude 60°: a westerly error of about 2.5°.
///     let error = gyro_speed_error(
///         Latitude::from_degrees(60.0)?,
///         TrueCourse::new(0.0)?,
///         Speed::from_knots(20.0)?,
///     )?;
///     assert_eq!(format!("{:.2}", error.degrees()), "-2.54");
///
///     // Due south the error is easterly instead.
///     let southerly = gyro_speed_error(
///         Latitude::from_degrees(60.0)?,
///         TrueCourse::new(180.0)?,
///         Speed::from_knots(20.0)?,
///     )?;
///     assert!(southerly.degrees() > 0.0);
///     Ok(())
/// }
/// ```
pub fn gyro_speed_error(latitude: Latitude, course: TrueCourse, speed: Speed) -> Result<Angle> {
    ensure_speed("speed", speed)?;
    ensure_range(
        "latitude",
        latitude.degrees(),
        -MAX_GYRO_LATITUDE_DEG,
        MAX_GYRO_LATITUDE_DEG,
    )?;

    let knots = speed.knots();
    let course_radians = course.radians();
    // 900 kn: Earth's equatorial surface speed (15°/h, 60 NM per degree).
    let eastward = EARTH_SURFACE_SPEED_KNOTS * math::cos(latitude.radians())
        + knots * math::sin(course_radians);

    if eastward <= f64::EPSILON {
        return Err(NavigationError::Kernel(KernelError::Indeterminate {
            quantity: "the settling meridian of a gyrocompass at this latitude",
        }));
    }

    let displacement = math::atan2(knots * math::cos(course_radians), eastward);
    Ok(Angle::from_degrees_unchecked(-math::to_degrees(
        displacement,
    )))
}

/// Relative bearing: angle from the ship's head clockwise to a bearing.
///
/// Both arguments in the same frame, enforced by the types.
///
/// # Example
///
/// ```rust
/// use kinavis::{navigation_solutions::calculate_course_angle, TrueCourse};
///
/// let course = TrueCourse::new(90.0)?;
/// let bearing = TrueCourse::new(180.0)?;
/// assert_eq!(calculate_course_angle(course, bearing).degrees(), 90.0);
/// # Ok::<(), kinavis::NavigationError>(())
/// ```
#[must_use]
pub fn calculate_course_angle<F: Frame>(
    course: Direction<F>,
    bearing: Direction<F>,
) -> RelativeBearing {
    RelativeBearing::from_degrees_wrapped(bearing.degrees() - course.degrees())
}

/// Relative bearing back to a bearing in the course's frame.
#[must_use]
pub fn bearing_from_relative<F: Frame>(
    course: Direction<F>,
    relative: RelativeBearing,
) -> Direction<F> {
    Direction::<F>::from_degrees_wrapped(course.degrees() + relative.degrees())
}
