//! Dead reckoning and estimated position.
//!
//! A *DR* position uses course steered and distance run only; an *EP* also
//! allows for current and leeway. They are kept separate: DR records what was
//! done, EP what probably happened.
//!
//! Tracks are rhumb lines, as followed when steering one course.
//!
//! # Example
//!
//! ```rust
//! use kinavis::dead_reckoning::{dead_reckoning, estimated_position};
//! use kinavis::navigation_solutions::Current;
//! use kinavis::{NavigationError, Position, Speed, TrueCourse};
//! use core::time::Duration;
//!
//! fn main() -> Result<(), NavigationError> {
//!     let noon = Position::from_degrees(50.0, -5.0)?;
//!     let course = TrueCourse::new(270.0)?;
//!     let speed = Speed::from_knots(12.0)?;
//!     let four_hours = Duration::from_secs(4 * 3600);
//!
//!     // Steering due west at 12 knots for four hours: 48 miles of westing.
//!     let reckoned = dead_reckoning(noon, course, speed, four_hours)?;
//!     assert_eq!(format!("{reckoned}"), "50°00.0'N 006°14.6'W");
//!
//!     // With a knot of north-going current the ship also makes northing.
//!     let current = Current {
//!         set: TrueCourse::new(0.0)?,
//!         drift: Speed::from_knots(1.0)?,
//!     };
//!     let estimated = estimated_position(noon, course, speed, current, four_hours)?;
//!     assert!(estimated.position.latitude().degrees() > 50.0);
//!     Ok(())
//! }
//! ```

use core::time::Duration;

use crate::angle::{Direction, True, TrueCourse};
use crate::error::{KernelError, Result};
use crate::math;
use crate::navigation_solutions::{course_over_ground, Current, GroundTrack};
use crate::position::Position;
use crate::sailings::rhumb_destination;
use crate::units::{hours, Angle, Distance, Speed};

/// One leg of a traverse.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Leg {
    /// Course made good.
    pub course: TrueCourse,
    /// Distance run.
    pub distance: Distance,
}

/// Estimated position with its ground track.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct EstimatedPosition {
    /// Estimated position.
    pub position: Position,
    /// Course and speed made good.
    pub track: GroundTrack,
    /// Distance made good.
    pub distance_made_good: Distance,
}

/// DR position after steering one course at one speed for `duration`.
///
/// # Errors
///
/// Rhumb-line failures, notably [`crate::KernelError::Indeterminate`] if the
/// track crosses a pole.
pub fn dead_reckoning(
    from: Position,
    course: TrueCourse,
    speed: Speed,
    elapsed: Duration,
) -> Result<Position> {
    rhumb_destination(from, course, speed.distance_covered(elapsed))
}

/// DR position after running `distance` on one course.
///
/// # Errors
///
/// As [`dead_reckoning`].
pub fn dead_reckoning_by_distance(
    from: Position,
    course: TrueCourse,
    distance: Distance,
) -> Result<Position> {
    rhumb_destination(from, course, distance)
}

/// Estimated position allowing for current.
///
/// # Errors
///
/// - [`crate::KernelError::OutOfRange`] for sternway (not modelled by the
///   current triangle).
/// - [`crate::KernelError::Indeterminate`] if motion through the water exactly
///   cancels the current.
pub fn estimated_position(
    from: Position,
    heading: TrueCourse,
    speed: Speed,
    current: Current,
    elapsed: Duration,
) -> Result<EstimatedPosition> {
    let track = course_over_ground(heading, speed, current.set, current.drift)?;
    let distance_made_good = track.speed_over_ground.distance_covered(elapsed);
    Ok(EstimatedPosition {
        position: rhumb_destination(from, track.course_over_ground, distance_made_good)?,
        track,
        distance_made_good,
    })
}

/// End position of consecutive legs, each a rhumb line from the previous end
/// (traverse table).
///
/// # Errors
///
/// As [`dead_reckoning`], for the first leg crossing a pole.
pub fn traverse(from: Position, legs: &[Leg]) -> Result<Position> {
    let mut position = from;
    for leg in legs {
        position = rhumb_destination(position, leg.course, leg.distance)?;
    }
    Ok(position)
}

/// Water track after leeway.
///
/// Leeway sets the ship to leeward of the heading: to starboard with the wind
/// on the port side and vice versa.
///
/// `leeway` is the observed leeway angle; it is not derived from wind strength
/// because it depends on the hull. Its sign is ignored.
///
/// # Example
///
/// ```rust
/// use kinavis::dead_reckoning::water_track;
/// use kinavis::{Angle, NavigationError, TrueCourse};
///
/// fn main() -> Result<(), NavigationError> {
///     let heading = TrueCourse::new(0.0)?;
///     let leeway = Angle::from_degrees(5.0)?;
///
///     // Wind on the port bow: the ship is set to starboard.
///     let from_port = water_track(heading, leeway, TrueCourse::new(315.0)?);
///     assert_eq!(from_port.degrees(), 5.0);
///
///     // Wind on the starboard bow: the other way.
///     let from_starboard = water_track(heading, leeway, TrueCourse::new(45.0)?);
///     assert_eq!(from_starboard.degrees(), 355.0);
///     Ok(())
/// }
/// ```
#[must_use]
pub fn water_track(heading: TrueCourse, leeway: Angle, wind_from: TrueCourse) -> TrueCourse {
    // Positive: wind from the starboard side.
    let relative = heading.signed_difference(wind_from);
    let magnitude = math::abs(leeway.degrees());
    let applied = if relative > 0.0 {
        -magnitude
    } else {
        magnitude
    };
    Direction::<True>::from_degrees_wrapped(heading.degrees() + applied)
}

/// Passage time at `speed`.
///
/// # Errors
///
/// [`crate::KernelError::Indeterminate`] for zero or opposite-sign speed.
pub fn passage_time(distance: Distance, speed: Speed) -> Result<Duration> {
    Ok(speed.time_to_cover(distance)?)
}

/// Speed required to cover `distance` in `duration`.
///
/// # Errors
///
/// [`crate::KernelError::Indeterminate`] for zero duration.
pub fn speed_required(distance: Distance, available: Duration) -> Result<Speed> {
    let elapsed = hours(available);
    if elapsed <= 0.0 {
        return Err(crate::NavigationError::Kernel(KernelError::Indeterminate {
            quantity: "the speed required in no time at all",
        }));
    }
    Ok(Speed::from_knots(distance.nautical_miles() / elapsed)?)
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp, clippy::indexing_slicing)]
mod tests {
    use super::*;
    use crate::sailings::rhumb_line;
    use alloc::vec;

    fn at(latitude: f64, longitude: f64) -> Position {
        Position::from_degrees(latitude, longitude).unwrap()
    }

    #[test]
    fn dead_reckoning_runs_the_distance_it_should() {
        let from = at(50.0, -5.0);
        let course = TrueCourse::new(45.0).unwrap();
        let speed = Speed::from_knots(12.0).unwrap();
        let elapsed = Duration::from_secs(3 * 3600);

        let to = dead_reckoning(from, course, speed, elapsed).unwrap();
        let sailing = rhumb_line(from, to).unwrap();

        assert!((sailing.distance.nautical_miles() - 36.0).abs() < 1e-9);
        assert!(sailing.initial_course.angular_distance(course) < 1e-9);
    }

    #[test]
    fn northing_alone_changes_no_longitude() {
        let from = at(10.0, 20.0);
        let to = dead_reckoning(
            from,
            TrueCourse::NORTH,
            Speed::from_knots(10.0).unwrap(),
            Duration::from_secs(6 * 3600),
        )
        .unwrap();
        assert!((to.longitude().degrees() - 20.0).abs() < 1e-12);
        // 60 NM of northing ≈ 1° of latitude.
        assert!((to.latitude().degrees() - 11.0).abs() < 0.01);
    }

    #[test]
    fn a_traverse_adds_up_to_its_legs() {
        let from = at(0.0, 0.0);
        let legs = vec![
            Leg {
                course: TrueCourse::new(0.0).unwrap(),
                distance: Distance::from_nautical_miles(60.0).unwrap(),
            },
            Leg {
                course: TrueCourse::new(90.0).unwrap(),
                distance: Distance::from_nautical_miles(60.0).unwrap(),
            },
            Leg {
                course: TrueCourse::new(180.0).unwrap(),
                distance: Distance::from_nautical_miles(60.0).unwrap(),
            },
        ];

        let end = traverse(from, &legs).unwrap();
        // North, east, south: back on the equator, east of the start.
        assert!(end.latitude().degrees().abs() < 1e-9);
        assert!(end.longitude().degrees() > 0.9);

        // Running the legs one at a time gives the same result.
        let mut step = from;
        for leg in &legs {
            step = dead_reckoning_by_distance(step, leg.course, leg.distance).unwrap();
        }
        assert!(rhumb_line(step, end).unwrap().distance.nautical_miles() < 1e-9);

        // Empty traverse: no movement.
        assert_eq!(traverse(from, &[]).unwrap(), from);
    }

    #[test]
    fn a_traverse_that_closes_returns_to_its_start() {
        let from = at(45.0, 10.0);
        let there = Leg {
            course: TrueCourse::new(75.0).unwrap(),
            distance: Distance::from_nautical_miles(40.0).unwrap(),
        };
        let back = Leg {
            course: there.course.reciprocal(),
            distance: there.distance,
        };
        let end = traverse(from, &[there, back]).unwrap();
        assert!(rhumb_line(from, end).unwrap().distance.nautical_miles() < 1e-9);
    }

    #[test]
    fn the_current_moves_the_estimated_position_off_the_dead_reckoning() {
        let from = at(50.0, -5.0);
        let heading = TrueCourse::new(270.0).unwrap();
        let speed = Speed::from_knots(12.0).unwrap();
        let elapsed = Duration::from_secs(4 * 3600);

        let reckoned = dead_reckoning(from, heading, speed, elapsed).unwrap();
        let estimated = estimated_position(
            from,
            heading,
            speed,
            Current {
                set: TrueCourse::NORTH,
                drift: Speed::from_knots(1.0).unwrap(),
            },
            elapsed,
        )
        .unwrap();

        // 4 h at 1 kn north: 4 NM northing.
        let offset = rhumb_line(reckoned, estimated.position).unwrap();
        assert!((offset.distance.nautical_miles() - 4.0).abs() < 0.05);
        assert!(offset.initial_course.angular_distance(TrueCourse::NORTH) < 1.0);
    }

    #[test]
    fn with_no_current_the_estimate_is_the_reckoning() {
        let from = at(-20.0, 100.0);
        let heading = TrueCourse::new(200.0).unwrap();
        let speed = Speed::from_knots(15.0).unwrap();
        let elapsed = Duration::from_secs(7200);

        let reckoned = dead_reckoning(from, heading, speed, elapsed).unwrap();
        let estimated = estimated_position(
            from,
            heading,
            speed,
            Current {
                set: TrueCourse::NORTH,
                drift: Speed::ZERO,
            },
            elapsed,
        )
        .unwrap();

        assert!(
            rhumb_line(reckoned, estimated.position)
                .unwrap()
                .distance
                .nautical_miles()
                < 1e-9
        );
        assert!((estimated.distance_made_good.nautical_miles() - 30.0).abs() < 1e-9);
    }

    #[test]
    fn leeway_is_applied_away_from_the_wind() {
        let heading = TrueCourse::new(0.0).unwrap();
        let leeway = Angle::from_degrees(5.0).unwrap();

        // Wind from port sets to starboard.
        assert_eq!(
            water_track(heading, leeway, TrueCourse::new(270.0).unwrap()).degrees(),
            5.0
        );
        // Wind from starboard sets to port.
        assert_eq!(
            water_track(heading, leeway, TrueCourse::new(90.0).unwrap()).degrees(),
            355.0
        );
        // Negative leeway is treated as positive.
        assert_eq!(
            water_track(
                heading,
                Angle::from_degrees(-5.0).unwrap(),
                TrueCourse::new(270.0).unwrap()
            )
            .degrees(),
            5.0
        );
    }

    #[test]
    fn leeway_wraps_through_north() {
        let heading = TrueCourse::new(2.0).unwrap();
        let track = water_track(
            heading,
            Angle::from_degrees(10.0).unwrap(),
            TrueCourse::new(90.0).unwrap(),
        );
        assert!((track.degrees() - 352.0).abs() < 1e-12);
    }

    #[test]
    fn passage_time_and_speed_required_are_inverses() {
        let distance = Distance::from_nautical_miles(150.0).unwrap();
        let speed = Speed::from_knots(12.5).unwrap();
        let elapsed = passage_time(distance, speed).unwrap();
        assert_eq!(elapsed.as_secs(), 12 * 3600);
        assert!((speed_required(distance, elapsed).unwrap().knots() - 12.5).abs() < 1e-9);
    }

    #[test]
    fn impossible_passages_are_errors() {
        let distance = Distance::from_nautical_miles(150.0).unwrap();
        assert!(passage_time(distance, Speed::ZERO).is_err());
        assert!(speed_required(distance, Duration::ZERO).is_err());
    }

    #[test]
    fn a_track_over_the_pole_is_refused_not_fudged() {
        let from = at(89.0, 0.0);
        let result = dead_reckoning(
            from,
            TrueCourse::NORTH,
            Speed::from_knots(30.0).unwrap(),
            Duration::from_secs(10 * 3600),
        );
        assert!(result.is_err());
    }
}
