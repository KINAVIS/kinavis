//! Current triangle: heading and water speed, set and drift, course and speed
//! over ground; any two give the third.

use crate::angle::{Direction, True, TrueCourse};
use crate::error::{ensure_finite, ensure_range, KernelError, NavigationError, Result};
use crate::math;
use crate::units::{Angle, Speed};

use super::{Current, GroundTrack, SteeringSolution};

/// Course and speed over ground from heading, water speed and current.
///
/// All speeds in the same unit.
///
/// # Errors
///
/// - [`KernelError::NotFinite`] or [`KernelError::OutOfRange`] for a negative
///   or non-finite speed.
/// - [`KernelError::Indeterminate`] if water motion exactly cancels the
///   current.
///
/// # Example
///
/// ```rust
/// use kinavis::{navigation_solutions::course_over_ground, Speed, TrueCourse};
///
/// // Steering due north at 10 knots, with 2 knots setting due east.
/// let track = course_over_ground(
///     TrueCourse::new(0.0)?,
///     Speed::from_knots(10.0)?,
///     TrueCourse::new(90.0)?,
///     Speed::from_knots(2.0)?,
/// )?;
///
/// assert_eq!(format!("{:.2}", track.course_over_ground.degrees()), "11.31");
/// assert_eq!(format!("{:.2}", track.speed_over_ground.knots()), "10.20");
/// # Ok::<(), kinavis::NavigationError>(())
/// ```
pub fn course_over_ground(
    heading: TrueCourse,
    speed_through_water: Speed,
    set: TrueCourse,
    drift: Speed,
) -> Result<GroundTrack> {
    ensure_speed("speed through water", speed_through_water)?;
    ensure_speed("drift", drift)?;

    let (water_north, water_east) = heading.components(speed_through_water.knots());
    let (current_north, current_east) = set.components(drift.knots());

    let north = water_north + current_north;
    let east = water_east + current_east;
    let speed_over_ground = math::hypot(north, east);

    // Relative to input magnitude, not a constant: see
    // `math::is_effectively_zero`.
    if math::is_effectively_zero(
        speed_over_ground,
        speed_through_water.knots().max(drift.knots()),
    ) {
        return Err(NavigationError::Kernel(KernelError::Missing {
            what: "course over ground",
        }));
    }

    Ok(GroundTrack {
        course_over_ground: Direction::<True>::from_degrees_wrapped(math::to_degrees(math::atan2(
            east, north,
        ))),
        speed_over_ground: Speed::from_knots_unchecked(speed_over_ground),
    })
}

/// Heading to steer to make good a track against a known current.
///
/// # Errors
///
/// - [`KernelError::NotFinite`] or [`KernelError::OutOfRange`] for a negative
///   or non-finite speed.
/// - [`NavigationError::CurrentTooStrong`] if no heading makes the track good
///   (cross-track set exceeds the vessel's speed, or the current carries it
///   backwards along the track).
///
/// # Example
///
/// ```rust
/// use kinavis::{navigation_solutions::course_to_steer, Speed, TrueCourse};
///
/// // To make good due north at 10 knots through the water, against 2 knots east.
/// let steering = course_to_steer(
///     TrueCourse::new(0.0)?,
///     Speed::from_knots(10.0)?,
///     TrueCourse::new(90.0)?,
///     Speed::from_knots(2.0)?,
/// )?;
///
/// assert_eq!(format!("{:.2}", steering.heading.degrees()), "348.46");
/// assert_eq!(format!("{:.2}", steering.speed_over_ground.knots()), "9.80");
/// # Ok::<(), kinavis::NavigationError>(())
/// ```
pub fn course_to_steer(
    track: TrueCourse,
    speed_through_water: Speed,
    set: TrueCourse,
    drift: Speed,
) -> Result<SteeringSolution> {
    ensure_speed("speed through water", speed_through_water)?;
    ensure_speed("drift", drift)?;

    let through_water = speed_through_water.knots();
    let current = drift.knots();
    let too_strong = || NavigationError::CurrentTooStrong {
        drift: current,
        speed_through_water: through_water,
    };

    if through_water < f64::EPSILON {
        return Err(too_strong());
    }

    // Cross-track components cancel: V·sin(H − T) + D·sin(S − T) = 0.
    let current_offset = math::to_radians(track.signed_difference(set));
    let sine = -current * math::sin(current_offset) / through_water;

    if math::abs(sine) > 1.0 {
        return Err(too_strong());
    }

    let drift_angle = math::asin(sine);
    let speed_over_ground =
        through_water * math::cos(drift_angle) + current * math::cos(current_offset);

    if speed_over_ground <= 0.0 {
        return Err(too_strong());
    }

    Ok(SteeringSolution {
        heading: Direction::<True>::from_degrees_wrapped(
            track.degrees() + math::to_degrees(drift_angle),
        ),
        speed_over_ground: Speed::from_knots_unchecked(speed_over_ground),
        drift_angle: Angle::from_degrees_unchecked(math::to_degrees(drift_angle)),
    })
}

/// Current implied by the difference between water track and ground track.
///
/// # Errors
///
/// [`KernelError::NotFinite`] or [`KernelError::OutOfRange`] for a negative or
/// non-finite speed.
pub fn estimate_current(
    heading: TrueCourse,
    speed_through_water: Speed,
    course_over_ground: TrueCourse,
    speed_over_ground: Speed,
) -> Result<Current> {
    ensure_speed("speed through water", speed_through_water)?;
    ensure_speed("speed over ground", speed_over_ground)?;

    let (water_north, water_east) = heading.components(speed_through_water.knots());
    let (ground_north, ground_east) = course_over_ground.components(speed_over_ground.knots());

    let north = ground_north - water_north;
    let east = ground_east - water_east;
    let drift = math::hypot(north, east);

    let set = if math::is_effectively_zero(
        drift,
        speed_through_water.knots().max(speed_over_ground.knots()),
    ) {
        Direction::<True>::NORTH
    } else {
        Direction::<True>::from_degrees_wrapped(math::to_degrees(math::atan2(east, north)))
    };

    Ok(Current {
        set,
        drift: Speed::from_knots_unchecked(drift),
    })
}

/// Current-triangle speeds must be finite and non-negative.
pub(super) fn ensure_speed(parameter: &'static str, value: Speed) -> Result<()> {
    ensure_finite(parameter, value.knots())?;
    Ok(ensure_range(parameter, value.knots(), 0.0, f64::MAX)?)
}
