//! Relative motion: CPA, radar plotting, avoiding manoeuvre.
//!
//! Computed in the relative frame: own ship stationary at the origin, the
//! target moving — as on a radar display and in the plotting triangle.
//!
//! Plane geometry: curvature is negligible over the few miles of a collision
//! situation.
//!
//! # Example
//!
//! ```rust
//! use kinavis::relative_motion::{closest_point_of_approach, Approach, Contact, Vessel};
//! use kinavis::{Distance, NavigationError, Speed, TrueBearing, TrueCourse};
//!
//! fn main() -> Result<(), NavigationError> {
//!     let own = Vessel {
//!         course: TrueCourse::new(0.0)?,
//!         speed: Speed::from_knots(15.0)?,
//!     };
//!     // A ship 10 miles away on the starboard bow, crossing to port.
//!     let contact = Contact {
//!         bearing: TrueBearing::new(30.0)?,
//!         range: Distance::from_nautical_miles(10.0)?,
//!     };
//!     let target = Vessel {
//!         course: TrueCourse::new(270.0)?,
//!         speed: Speed::from_knots(15.0)?,
//!     };
//!
//!     match closest_point_of_approach(own, contact, target)? {
//!         Approach::Closing(cpa) => {
//!             assert_eq!(format!("{:.2}", cpa.distance.nautical_miles()), "2.59");
//!             assert_eq!(cpa.time_to_go.as_secs() / 60, 27);
//!         }
//!         other => panic!("expected a closing contact, got {other:?}"),
//!     }
//!     Ok(())
//! }
//! ```

use core::time::Duration;

use crate::angle::{wrap180, Direction, RelativeBearing, True, TrueBearing, TrueCourse};
use crate::error::{KernelError, NavigationError, Result};
use crate::math;
use crate::units::{hours, Angle, Distance, Speed};

/// Relative speed below which the contact is treated as stationary.
const STATIONARY_KNOTS: f64 = 1e-9;

/// Course and speed of a vessel.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Vessel {
    /// Course.
    pub course: TrueCourse,
    /// Speed.
    pub speed: Speed,
}

impl Vessel {
    /// Velocity as north and east components, kn.
    fn velocity(self) -> (f64, f64) {
        let radians = self.course.radians();
        (
            self.speed.knots() * math::cos(radians),
            self.speed.knots() * math::sin(radians),
        )
    }

    /// Vessel from north and east velocity components, kn.
    fn from_velocity(north: f64, east: f64) -> Self {
        Self {
            course: Direction::<True>::from_degrees_wrapped(math::to_degrees(math::atan2(
                east, north,
            ))),
            speed: Speed::from_knots_unchecked(math::hypot(north, east)),
        }
    }
}

/// Target position relative to own ship.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Contact {
    /// True bearing from own ship.
    pub bearing: TrueBearing,
    /// Range.
    pub range: Distance,
}

impl Contact {
    /// North and east offsets, NM.
    fn offset(self) -> (f64, f64) {
        let radians = self.bearing.radians();
        (
            self.range.nautical_miles() * math::cos(radians),
            self.range.nautical_miles() * math::sin(radians),
        )
    }
}

/// Closest point of approach.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Cpa {
    /// CPA distance.
    pub distance: Distance,
    /// TCPA.
    pub time_to_go: Duration,
    /// Bearing at CPA.
    pub bearing: TrueBearing,
}

/// Range trend.
///
/// Distinguishes past CPA from future CPA.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Approach {
    /// Closing; CPA ahead.
    Closing(Cpa),
    /// Opening; CPA passed.
    Opening {
        /// Current range, the minimum from now on.
        current_range: Distance,
    },
    /// No relative motion: constant bearing and range.
    Stationary {
        /// Constant range.
        range: Distance,
    },
}

/// Radar plot result.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct TargetSolution {
    /// Target true course and speed.
    pub vessel: Vessel,
    /// Relative course of the target.
    pub relative_course: TrueCourse,
    /// Relative speed of the target.
    pub relative_speed: Speed,
    /// Aspect: own ship's bearing relative to the target's head.
    ///
    /// Negative: own ship on the target's port side (red aspect); positive:
    /// starboard (green).
    pub aspect: Angle,
}

/// Courses giving the requested CPA.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Avoidance {
    /// Smallest starboard alteration, if any.
    pub starboard: Option<TrueCourse>,
    /// Smallest port alteration, if any.
    pub port: Option<TrueCourse>,
}

/// CPA at present courses and speeds.
///
/// # Errors
///
/// [`KernelError::OutOfRange`] for a negative range;
/// [`KernelError::Indeterminate`] if TCPA is too large to represent.
pub fn closest_point_of_approach(
    own: Vessel,
    contact: Contact,
    target: Vessel,
) -> Result<Approach> {
    crate::error::ensure_range("range", contact.range.nautical_miles(), 0.0, f64::MAX)?;

    let (offset_north, offset_east) = contact.offset();
    let (relative_north, relative_east) = relative_velocity(own, target);
    let relative_speed = math::hypot(relative_north, relative_east);

    if relative_speed < STATIONARY_KNOTS {
        return Ok(Approach::Stationary {
            range: contact.range,
        });
    }

    // CPA: point of the relative track nearest the origin.
    let closing_rate = offset_north * relative_north + offset_east * relative_east;
    let time = -closing_rate / (relative_speed * relative_speed);

    if time <= 0.0 {
        return Ok(Approach::Opening {
            current_range: contact.range,
        });
    }

    let (closest_north, closest_east) = (
        offset_north + relative_north * time,
        offset_east + relative_east * time,
    );

    Ok(Approach::Closing(Cpa {
        distance: Distance::from_nautical_miles_unchecked(math::hypot(closest_north, closest_east)),
        time_to_go: crate::units::duration_from_hours(time)?,
        bearing: Direction::<True>::from_degrees_wrapped(math::to_degrees(math::atan2(
            closest_east,
            closest_north,
        ))),
    }))
}

/// Target course and speed from two plots and the time between (plotting
/// triangle).
///
/// Relative movement between plots gives relative velocity; adding own velocity
/// gives the target's.
///
/// # Errors
///
/// - [`KernelError::OutOfRange`] for a negative range.
/// - [`KernelError::Indeterminate`] for zero elapsed time.
///
/// # Example
///
/// ```rust
/// use kinavis::relative_motion::{target_from_plot, Contact, Vessel};
/// use kinavis::{Distance, NavigationError, Speed, TrueBearing, TrueCourse};
/// use core::time::Duration;
///
/// fn main() -> Result<(), NavigationError> {
///     let own = Vessel {
///         course: TrueCourse::new(0.0)?,
///         speed: Speed::from_knots(10.0)?,
///     };
///     // Two plots six minutes apart.
///     let first = Contact {
///         bearing: TrueBearing::new(90.0)?,
///         range: Distance::from_nautical_miles(6.0)?,
///     };
///     let second = Contact {
///         bearing: TrueBearing::new(90.0)?,
///         range: Distance::from_nautical_miles(5.0)?,
///     };
///
///     let solution = target_from_plot(own, first, second, Duration::from_secs(360))?;
///
///     // Target closes down the bearing at 10 kn while own ship makes 10 kn
///     // north, so the target's course is 315 at just over 14 kn.
///     assert_eq!(format!("{:.1}", solution.vessel.course.degrees()), "315.0");
///     assert_eq!(format!("{:.2}", solution.vessel.speed.knots()), "14.14");
///
///     // Own ship is 45° on the target's port bow.
///     assert_eq!(format!("{:.1}", solution.aspect.degrees()), "-45.0");
///     Ok(())
/// }
/// ```
pub fn target_from_plot(
    own: Vessel,
    first: Contact,
    second: Contact,
    elapsed: Duration,
) -> Result<TargetSolution> {
    crate::error::ensure_range("range", first.range.nautical_miles(), 0.0, f64::MAX)?;
    crate::error::ensure_range("range", second.range.nautical_miles(), 0.0, f64::MAX)?;

    let interval = hours(elapsed);
    if interval <= 0.0 {
        return Err(NavigationError::Kernel(KernelError::Indeterminate {
            quantity: "a target's motion between two plots at the same moment",
        }));
    }

    let (first_north, first_east) = first.offset();
    let (second_north, second_east) = second.offset();
    let (relative_north, relative_east) = (
        (second_north - first_north) / interval,
        (second_east - first_east) / interval,
    );

    let (own_north, own_east) = own.velocity();
    let vessel = Vessel::from_velocity(relative_north + own_north, relative_east + own_east);

    // Aspect: own ship relative to the target's head.
    let own_bearing_from_target = second.bearing.reciprocal();
    let aspect = Angle::from_degrees_unchecked(wrap180(
        own_bearing_from_target.degrees() - vessel.course.degrees(),
    ));

    Ok(TargetSolution {
        vessel,
        relative_course: Direction::<True>::from_degrees_wrapped(math::to_degrees(math::atan2(
            relative_east,
            relative_north,
        ))),
        relative_speed: Speed::from_knots_unchecked(math::hypot(relative_north, relative_east)),
        aspect,
    })
}

/// Bow crossing range: distance ahead along own heading where the target
/// crosses.
///
/// # Errors
///
/// - [`KernelError::OutOfRange`] for a negative range.
/// - [`NavigationError::NoSolution`] if the target does not cross ahead (same
///   direction, or passes astern).
pub fn bow_crossing_range(own: Vessel, contact: Contact, target: Vessel) -> Result<Distance> {
    crate::error::ensure_range("range", contact.range.nautical_miles(), 0.0, f64::MAX)?;

    let (offset_north, offset_east) = contact.offset();
    let (relative_north, relative_east) = relative_velocity(own, target);
    let heading = own.course.radians();
    let (ahead_north, ahead_east) = (math::cos(heading), math::sin(heading));

    // Solve offset + relative·t = ahead·s for time t and range ahead s.
    let determinant = relative_north * (-ahead_east) - relative_east * (-ahead_north);
    if math::abs(determinant) < 1e-12 {
        return Err(NavigationError::NoSolution {
            context: "a crossing of own ship's track by a contact moving along it",
        });
    }

    let time = (-offset_north * -ahead_east + offset_east * -ahead_north) / determinant;
    let range_ahead = (relative_east * offset_north - relative_north * offset_east) / determinant;

    // Zero range ahead means crossing over own ship (still ahead); only
    // negative is astern. Tolerance scaled to the range, since an exact bow
    // crossing yields a tiny value of either sign.
    let tolerance = contact.range.nautical_miles() * 1e-9;
    if time <= 0.0 || range_ahead < -tolerance {
        return Err(NavigationError::NoSolution {
            context: "a crossing ahead of own ship",
        });
    }

    let range_ahead = range_ahead.max(0.0);

    Ok(Distance::from_nautical_miles_unchecked(range_ahead))
}

/// Courses at unchanged speed giving a CPA of at least `desired`.
///
/// Two limiting relative tracks are tangent to the passing-distance circle;
/// each needs its own alteration, and the smallest on each side is returned.
/// Under COLREGs a starboard alteration is normal: [`Avoidance::starboard`].
///
/// # Errors
///
/// - [`KernelError::OutOfRange`] for a negative range or distance.
/// - [`NavigationError::NoSolution`] if the target is already inside `desired`,
///   or no alteration at this speed achieves it.
///
/// # Example
///
/// ```rust
/// use kinavis::relative_motion::{closest_point_of_approach, course_for_cpa, Approach, Contact, Vessel};
/// use kinavis::{Distance, NavigationError, Speed, TrueBearing, TrueCourse};
///
/// fn main() -> Result<(), NavigationError> {
///     let own = Vessel {
///         course: TrueCourse::new(0.0)?,
///         speed: Speed::from_knots(15.0)?,
///     };
///     let contact = Contact {
///         bearing: TrueBearing::new(10.0)?,
///         range: Distance::from_nautical_miles(8.0)?,
///     };
///     let target = Vessel {
///         course: TrueCourse::new(190.0)?,
///         speed: Speed::from_knots(12.0)?,
///     };
///
///     // At present courses the target passes very close.
///     let Approach::Closing(before) = closest_point_of_approach(own, contact, target)? else {
///         panic!("target is closing");
///     };
///     assert!(before.distance.nautical_miles() < 1.5);
///
///     // Two miles is wanted instead.
///     let wanted = Distance::from_nautical_miles(2.0)?;
///     let avoidance = course_for_cpa(own, contact, target, wanted)?;
///     let altered = avoidance.starboard.expect("a starboard alteration exists");
///
///     let after = closest_point_of_approach(
///         Vessel { course: altered, speed: own.speed },
///         contact,
///         target,
///     )?;
///     let Approach::Closing(after) = after else {
///         panic!("still closing, just further off");
///     };
///     assert!((after.distance.nautical_miles() - 2.0).abs() < 1e-6);
///     Ok(())
/// }
/// ```
pub fn course_for_cpa(
    own: Vessel,
    contact: Contact,
    target: Vessel,
    desired: Distance,
) -> Result<Avoidance> {
    crate::error::ensure_range("range", contact.range.nautical_miles(), 0.0, f64::MAX)?;
    crate::error::ensure_range("desired distance", desired.nautical_miles(), 0.0, f64::MAX)?;

    let range = contact.range.nautical_miles();
    let wanted = desired.nautical_miles();
    if range <= wanted {
        return Err(NavigationError::NoSolution {
            context: "opening a contact that is already inside the distance asked for",
        });
    }
    if own.speed.knots() <= 0.0 {
        return Err(NavigationError::NoSolution {
            context: "a manoeuvre by a vessel that is not moving",
        });
    }

    // The relative track must be tangent to the circle of radius `wanted`, so
    // it deviates from the target bearing by this angle.
    let grazing = math::to_degrees(math::asin((wanted / range).clamp(-1.0, 1.0)));
    let away = contact.bearing.degrees() + 180.0;
    let (target_north, target_east) = target.velocity();
    let own_speed = own.speed.knots();

    let mut candidates: [Option<f64>; 4] = [None; 4];
    let mut count = 0;

    for relative_course in [away - grazing, away + grazing] {
        let radians = math::to_radians(relative_course);
        let (unit_north, unit_east) = (math::cos(radians), math::sin(radians));

        // Own velocity = target velocity − k·(required relative direction), k >
        // 0, with the right magnitude.
        let projection = target_north * unit_north + target_east * unit_east;
        let target_speed_squared = target_north * target_north + target_east * target_east;
        let discriminant = projection * projection - (target_speed_squared - own_speed * own_speed);
        if discriminant < 0.0 {
            continue;
        }
        let root = math::sqrt(discriminant);

        for relative_speed in [projection + root, projection - root] {
            if relative_speed <= 0.0 {
                continue;
            }
            let north = target_north - relative_speed * unit_north;
            let east = target_east - relative_speed * unit_east;
            if let Some(slot) = candidates.get_mut(count) {
                *slot = Some(math::to_degrees(math::atan2(east, north)));
            }
            count = count.saturating_add(1);
        }
    }

    let mut starboard: Option<(f64, f64)> = None;
    let mut port: Option<(f64, f64)> = None;
    for candidate in candidates.iter().flatten() {
        let alteration = wrap180(candidate - own.course.degrees());
        if alteration > 0.0 {
            if starboard.map_or(true, |(best, _)| alteration < best) {
                starboard = Some((alteration, *candidate));
            }
        } else if alteration < 0.0 && port.map_or(true, |(best, _)| alteration > best) {
            port = Some((alteration, *candidate));
        }
    }

    if starboard.is_none() && port.is_none() {
        return Err(NavigationError::NoSolution {
            context: "a course at this speed that opens the closest approach that far",
        });
    }

    Ok(Avoidance {
        starboard: starboard.map(|(_, course)| Direction::<True>::from_degrees_wrapped(course)),
        port: port.map(|(_, course)| Direction::<True>::from_degrees_wrapped(course)),
    })
}

/// Relative bearing of a contact from own heading.
#[must_use]
pub fn relative_bearing_of(own: Vessel, contact: Contact) -> RelativeBearing {
    RelativeBearing::from_degrees_wrapped(contact.bearing.degrees() - own.course.degrees())
}

/// Target velocity relative to own ship, kn north and east.
fn relative_velocity(own: Vessel, target: Vessel) -> (f64, f64) {
    let (own_north, own_east) = own.velocity();
    let (target_north, target_east) = target.velocity();
    (target_north - own_north, target_east - own_east)
}

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::indexing_slicing,
    clippy::panic
)]
mod tests {
    use super::*;
    use alloc::format;

    fn vessel(course: f64, knots: f64) -> Vessel {
        Vessel {
            course: TrueCourse::new(course).unwrap(),
            speed: Speed::from_knots(knots).unwrap(),
        }
    }

    fn contact(bearing: f64, range: f64) -> Contact {
        Contact {
            bearing: TrueBearing::new(bearing).unwrap(),
            range: Distance::from_nautical_miles(range).unwrap(),
        }
    }

    fn closing(approach: Approach) -> Cpa {
        match approach {
            Approach::Closing(cpa) => cpa,
            other => panic!("expected a closing contact, got {other:?}"),
        }
    }

    #[test]
    fn a_head_on_contact_closes_to_nothing() {
        // Dead ahead, head-on: CPA zero, soon.
        let own = vessel(0.0, 10.0);
        let target = vessel(180.0, 10.0);
        let cpa = closing(closest_point_of_approach(own, contact(0.0, 10.0), target).unwrap());

        assert!(cpa.distance.nautical_miles() < 1e-9);
        // 20 kn closing over 10 NM: 30 min.
        assert!((cpa.time_to_go.as_secs_f64() - 1800.0).abs() < 1e-6);
    }

    #[test]
    fn a_contact_abeam_on_a_parallel_course_never_closes() {
        let own = vessel(0.0, 12.0);
        let target = vessel(0.0, 12.0);
        let approach = closest_point_of_approach(own, contact(90.0, 3.0), target).unwrap();
        assert!(matches!(approach, Approach::Stationary { .. }));
    }

    #[test]
    fn a_contact_astern_and_slower_is_opening() {
        let own = vessel(0.0, 20.0);
        let target = vessel(0.0, 8.0);
        let approach = closest_point_of_approach(own, contact(180.0, 4.0), target).unwrap();
        match approach {
            Approach::Opening { current_range } => {
                assert_eq!(current_range.nautical_miles(), 4.0);
            }
            other => panic!("expected an opening contact, got {other:?}"),
        }
    }

    #[test]
    fn the_cpa_matches_a_hand_worked_crossing() {
        // Own 000 at 15 kn; contact 030 at 10 NM, heading 270 at 15 kn.
        let own = vessel(0.0, 15.0);
        let target = vessel(270.0, 15.0);
        let cpa = closing(closest_point_of_approach(own, contact(30.0, 10.0), target).unwrap());

        // By hand: relative velocity (−15, −15) from offset (8.660, 5.000)
        // gives CPA 2.5882 NM in 27.32 min.
        assert_eq!(format!("{:.4}", cpa.distance.nautical_miles()), "2.5882");
        assert!((cpa.time_to_go.as_secs_f64() / 60.0 - 27.3205).abs() < 1e-3);
        assert!((cpa.bearing.degrees() - 315.0).abs() < 1e-9);
    }

    #[test]
    fn the_cpa_is_really_the_closest_the_contact_comes() {
        let own = vessel(20.0, 14.0);
        let target = vessel(250.0, 9.0);
        let start = contact(65.0, 12.0);
        let cpa = closing(closest_point_of_approach(own, start, target).unwrap());

        // Step along the relative track; nothing beats the answer.
        let (north, east) = start.offset();
        let (relative_north, relative_east) = relative_velocity(own, target);
        let mut time = 0.0;
        while time < 3.0 {
            let range = math::hypot(north + relative_north * time, east + relative_east * time);
            assert!(
                range >= cpa.distance.nautical_miles() - 1e-9,
                "at {time} h the range is {range}, inside the CPA"
            );
            time += 0.001;
        }
    }

    #[test]
    fn a_plot_recovers_the_target_that_made_it() {
        let own = vessel(35.0, 16.0);
        let target = vessel(300.0, 11.0);
        let first = contact(80.0, 9.0);

        // Target position 12 min later.
        let elapsed = Duration::from_secs(720);
        let interval = hours(elapsed);
        let (north, east) = first.offset();
        let (relative_north, relative_east) = relative_velocity(own, target);
        let (later_north, later_east) = (
            north + relative_north * interval,
            east + relative_east * interval,
        );
        let second = Contact {
            bearing: TrueBearing::wrap(math::to_degrees(math::atan2(later_east, later_north)))
                .unwrap(),
            range: Distance::from_nautical_miles(math::hypot(later_north, later_east)).unwrap(),
        };

        let solution = target_from_plot(own, first, second, elapsed).unwrap();
        assert!(solution.vessel.course.angular_distance(target.course) < 1e-9);
        assert!((solution.vessel.speed.knots() - target.speed.knots()).abs() < 1e-9);
        assert!(solution.relative_speed.knots() > 0.0);
    }

    #[test]
    fn a_plot_needs_time_to_have_passed() {
        let own = vessel(0.0, 10.0);
        assert!(matches!(
            target_from_plot(own, contact(90.0, 6.0), contact(90.0, 5.0), Duration::ZERO)
                .unwrap_err(),
            NavigationError::Kernel(KernelError::Indeterminate { .. })
        ));
    }

    #[test]
    fn aspect_says_which_side_of_the_target_we_are_on() {
        // Closing straight down the bearing while own ship makes 10 kn north:
        // target course 315, own ship 45° on its port bow.
        let own = vessel(0.0, 10.0);
        let solution = target_from_plot(
            own,
            contact(90.0, 6.0),
            contact(90.0, 5.0),
            Duration::from_secs(360),
        )
        .unwrap();
        assert!((solution.vessel.course.degrees() - 315.0).abs() < 1e-9);
        assert!((solution.aspect.degrees() + 45.0).abs() < 1e-9);

        // Own ship stopped: the relative track is the true one, so a target
        // closing from due east heads due west and own ship is dead ahead of
        // it.
        let stopped = target_from_plot(
            vessel(0.0, 0.0),
            contact(90.0, 6.0),
            contact(90.0, 5.0),
            Duration::from_secs(360),
        )
        .unwrap();
        assert!((stopped.vessel.course.degrees() - 270.0).abs() < 1e-9);
        assert!(stopped.aspect.degrees().abs() < 1e-9);
    }

    #[test]
    fn a_crosser_cuts_ahead_at_a_computable_range() {
        // Target 10 NM due east heading west at 10 kn; own ship stopped.
        let own = vessel(0.0, 0.0);
        let target = vessel(270.0, 10.0);
        let range = bow_crossing_range(own, contact(90.0, 10.0), target).unwrap();
        // Crossing the heading line at the origin: passes over own ship.
        assert!(range.nautical_miles() < 1e-9);

        // Offset to the north-east: crosses ahead.
        let target = vessel(270.0, 10.0);
        let ahead = bow_crossing_range(own, contact(45.0, 10.0), target).unwrap();
        assert!((ahead.nautical_miles() - 10.0 * (45.0_f64).to_radians().cos()).abs() < 1e-9);
    }

    #[test]
    fn a_contact_that_never_crosses_ahead_is_reported() {
        let own = vessel(0.0, 10.0);
        // Astern, opposite course: does not cross ahead.
        let target = vessel(180.0, 10.0);
        assert!(matches!(
            bow_crossing_range(own, contact(180.0, 5.0), target).unwrap_err(),
            NavigationError::NoSolution { .. }
        ));

        // Moving along the heading line: no single crossing point.
        let same_way = vessel(0.0, 12.0);
        assert!(bow_crossing_range(own, contact(0.0, 5.0), same_way).is_err());
    }

    #[test]
    fn an_alteration_achieves_the_closest_approach_it_promises() {
        let own = vessel(0.0, 15.0);
        let target = vessel(190.0, 12.0);
        let start = contact(10.0, 8.0);
        let wanted = Distance::from_nautical_miles(2.0).unwrap();

        let avoidance = course_for_cpa(own, start, target, wanted).unwrap();
        for course in [avoidance.starboard, avoidance.port].into_iter().flatten() {
            let altered = Vessel {
                course,
                speed: own.speed,
            };
            let cpa = closing(closest_point_of_approach(altered, start, target).unwrap());
            assert!(
                (cpa.distance.nautical_miles() - 2.0).abs() < 1e-6,
                "steering {} gives a CPA of {}",
                course.degrees(),
                cpa.distance.nautical_miles()
            );
        }
    }

    #[test]
    fn alterations_are_found_on_both_bows_for_a_range_of_situations() {
        let own = vessel(0.0, 15.0);
        let wanted = Distance::from_nautical_miles(2.0).unwrap();

        for bearing in [5.0, 30.0, 60.0, 300.0, 330.0] {
            for target_course in [90.0, 180.0, 200.0, 270.0] {
                let target = vessel(target_course, 12.0);
                let start = contact(bearing, 8.0);
                let Ok(avoidance) = course_for_cpa(own, start, target, wanted) else {
                    continue;
                };
                assert!(
                    avoidance.starboard.is_some() || avoidance.port.is_some(),
                    "an empty avoidance should have been an error"
                );
                for course in [avoidance.starboard, avoidance.port].into_iter().flatten() {
                    let altered = Vessel {
                        course,
                        speed: own.speed,
                    };
                    match closest_point_of_approach(altered, start, target).unwrap() {
                        Approach::Closing(cpa) => assert!(
                            cpa.distance.nautical_miles() >= 2.0 - 1e-6,
                            "bearing {bearing}, target {target_course}: CPA {}",
                            cpa.distance.nautical_miles()
                        ),
                        // Already opening: better than requested.
                        Approach::Opening { .. } | Approach::Stationary { .. } => {}
                    }
                }
            }
        }
    }

    #[test]
    fn a_manoeuvre_that_cannot_work_is_reported() {
        let own = vessel(0.0, 15.0);
        let target = vessel(180.0, 12.0);

        // Already inside the requested distance.
        assert!(matches!(
            course_for_cpa(
                own,
                contact(0.0, 1.0),
                target,
                Distance::from_nautical_miles(2.0).unwrap()
            )
            .unwrap_err(),
            NavigationError::NoSolution { .. }
        ));

        // Stopped: no alteration possible.
        assert!(course_for_cpa(
            vessel(0.0, 0.0),
            contact(0.0, 8.0),
            target,
            Distance::from_nautical_miles(2.0).unwrap()
        )
        .is_err());

        // A slow ship cannot open a fast target arbitrarily.
        let slow = vessel(0.0, 2.0);
        let fast = vessel(0.0, 30.0);
        assert!(course_for_cpa(
            slow,
            contact(180.0, 3.0),
            fast,
            Distance::from_nautical_miles(2.9).unwrap()
        )
        .is_err());
    }

    #[test]
    fn relative_bearing_is_measured_from_the_head() {
        let own = vessel(45.0, 10.0);
        assert_eq!(relative_bearing_of(own, contact(90.0, 5.0)).degrees(), 45.0);
        assert_eq!(relative_bearing_of(own, contact(0.0, 5.0)).degrees(), 315.0);
    }

    #[test]
    fn hostile_input_is_refused_not_panicked_on() {
        let own = vessel(0.0, 10.0);
        let target = vessel(180.0, 10.0);
        let negative = Contact {
            bearing: TrueBearing::NORTH,
            range: Distance::from_nautical_miles(-1.0).unwrap(),
        };
        assert!(closest_point_of_approach(own, negative, target).is_err());
        assert!(bow_crossing_range(own, negative, target).is_err());
        assert!(course_for_cpa(own, negative, target, Distance::ZERO).is_err());
        assert!(course_for_cpa(
            own,
            contact(0.0, 5.0),
            target,
            Distance::from_nautical_miles(-1.0).unwrap()
        )
        .is_err());

        // An oversized range still gives an answer, no overflow.
        let far = contact(0.0, 1e6);
        assert!(closest_point_of_approach(own, far, target).is_ok());
    }
}
