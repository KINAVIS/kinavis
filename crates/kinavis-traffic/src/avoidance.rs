//! Collision-avoidance manoeuvre within COLREGs and ship constraints.
//!
//! [`course_for_cpa`] gives the minimum alteration that opens one target to a
//! passing distance (the radar-plot answer). A real manoeuvre must also: be to
//! a permitted side; be large enough to be readily apparent (Rule 8 (b)) and no
//! larger than the ship accepts; account for the rate of turn; and not close
//! one target while opening another. These constraints are
//! [`ManoeuvreConstraints`]; the search returns the smallest permitted
//! alteration that satisfies them — [`avoid`] for one target, [`avoid_all`] for
//! the whole picture.
//!
//! Course alterations only. Speed reduction depends on stopping
//! characteristics, which this crate does not model.

use core::time::Duration;

use kinavis::error::{ensure_range, KernelError, NavigationError, Result};
use kinavis::relative_motion::{
    closest_point_of_approach, course_for_cpa, Approach, Contact, Vessel,
};
use kinavis::sailings::rhumb_line;
use kinavis_kernel::angle::{wrap180, Direction, Side, True, TrueBearing, TrueCourse};
use kinavis_kernel::event::TargetId;
use kinavis_kernel::inline::Inline;
use kinavis_kernel::math;
use kinavis_kernel::snapshot::NavigationSnapshot;
use kinavis_kernel::units::{Angle, Distance, RateOfTurn};

use crate::Traffic;

/// Search step for permitted alterations, degrees.
///
/// The exact [`course_for_cpa`] answer is tried first where one exists; the
/// search covers the rest (least alteration overshooting it, multiple targets).
/// 1° is finer than any ordered alteration.
pub const ALTERATION_STEP_DEG: f64 = 1.0;

/// Maximum alteration: a reversal.
pub const MAX_ALTERATION_DEG: f64 = 180.0;

/// Permitted alteration side.
///
/// COLREGs usually leave one side open (a give-way vessel in a crossing alters
/// to starboard, not to port across the other's bow). Forbidding both is not
/// representable: it would admit no manoeuvre.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum PermittedSides {
    /// Either side; the smaller alteration is chosen.
    #[default]
    Either,
    /// Starboard only.
    Starboard,
    /// Port only.
    Port,
}

impl PermittedSides {
    /// Starboard alteration permitted.
    #[must_use]
    pub const fn starboard(self) -> bool {
        matches!(self, Self::Either | Self::Starboard)
    }

    /// Port alteration permitted.
    #[must_use]
    pub const fn port(self) -> bool {
        matches!(self, Self::Either | Self::Port)
    }
}

/// Manoeuvre constraints.
///
/// Validated once by [`ManoeuvreConstraints::new`].
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(
        try_from = "StoredManoeuvreConstraints",
        into = "StoredManoeuvreConstraints"
    )
)]
pub struct ManoeuvreConstraints {
    sides: PermittedSides,
    least_alteration: Angle,
    most_alteration: Angle,
    rate_of_turn: Option<RateOfTurn>,
}

impl ManoeuvreConstraints {
    /// Alterations to the permitted `sides`, between `least_alteration` and
    /// `most_alteration`.
    ///
    /// The least alteration is the minimum readily apparent to the other
    /// vessel, visually or by radar (30° is a common standing order); smaller
    /// answers are raised to it. The most is the ship's limit, at most a
    /// reversal.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] unless `0° ≤ least_alteration ≤
    /// most_alteration ≤` [`MAX_ALTERATION_DEG`].
    pub fn new(
        sides: PermittedSides,
        least_alteration: Angle,
        most_alteration: Angle,
    ) -> Result<Self> {
        ensure_range(
            "least alteration",
            least_alteration.degrees(),
            0.0,
            MAX_ALTERATION_DEG,
        )?;
        ensure_range(
            "most alteration",
            most_alteration.degrees(),
            least_alteration.degrees(),
            MAX_ALTERATION_DEG,
        )?;
        Ok(Self {
            sides,
            least_alteration,
            most_alteration,
            rate_of_turn: None,
        })
    }

    /// Adds the ship's rate of turn, so the result includes the alteration
    /// time. The sign is ignored.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] for a zero rate.
    pub fn with_rate_of_turn(mut self, rate: RateOfTurn) -> Result<Self> {
        let rate = rate.abs();
        ensure_range(
            "rate of turn",
            rate.degrees_per_minute(),
            f64::MIN_POSITIVE,
            f64::MAX,
        )?;
        self.rate_of_turn = Some(rate);
        Ok(self)
    }

    /// Permitted side.
    #[must_use]
    pub const fn sides(&self) -> PermittedSides {
        self.sides
    }

    /// Minimum alteration.
    #[must_use]
    pub const fn least_alteration(&self) -> Angle {
        self.least_alteration
    }

    /// Maximum alteration.
    #[must_use]
    pub const fn most_alteration(&self) -> Angle {
        self.most_alteration
    }

    /// Rate of turn, if known; always positive.
    #[must_use]
    pub const fn rate_of_turn(&self) -> Option<RateOfTurn> {
        self.rate_of_turn
    }
}

/// Serialised form; deserialisation goes through [`ManoeuvreConstraints::new`].
#[cfg(feature = "serde")]
#[derive(serde::Serialize, serde::Deserialize)]
struct StoredManoeuvreConstraints {
    sides: PermittedSides,
    least_alteration: Angle,
    most_alteration: Angle,
    rate_of_turn: Option<RateOfTurn>,
}

#[cfg(feature = "serde")]
impl TryFrom<StoredManoeuvreConstraints> for ManoeuvreConstraints {
    type Error = NavigationError;

    fn try_from(stored: StoredManoeuvreConstraints) -> Result<Self> {
        let constraints = Self::new(
            stored.sides,
            stored.least_alteration,
            stored.most_alteration,
        )?;
        match stored.rate_of_turn {
            Some(rate) => constraints.with_rate_of_turn(rate),
            None => Ok(constraints),
        }
    }
}

#[cfg(feature = "serde")]
impl From<ManoeuvreConstraints> for StoredManoeuvreConstraints {
    fn from(constraints: ManoeuvreConstraints) -> Self {
        Self {
            sides: constraints.sides,
            least_alteration: constraints.least_alteration,
            most_alteration: constraints.most_alteration,
            rate_of_turn: constraints.rate_of_turn,
        }
    }
}

/// Manoeuvre result.
///
/// Projection: cheap to copy, safe to log.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct AvoidanceManoeuvre {
    course: TrueCourse,
    alteration: Angle,
    turn_time: Option<Duration>,
    least_passing: Distance,
    limiting_target: Option<TargetId>,
}

impl AvoidanceManoeuvre {
    /// Course to steer.
    #[must_use]
    pub const fn course(&self) -> TrueCourse {
        self.course
    }

    /// Alteration from the current course, positive to starboard.
    #[must_use]
    pub const fn alteration(&self) -> Angle {
        self.alteration
    }

    /// Side of the alteration.
    #[must_use]
    pub fn side(&self) -> Side {
        if self.alteration.degrees() < 0.0 {
            Side::Port
        } else {
            Side::Starboard
        }
    }

    /// Time to complete the alteration at the given rate of turn; `None` if no
    /// rate was given.
    #[must_use]
    pub const fn turn_time(&self) -> Option<Duration> {
        self.turn_time
    }

    /// Minimum passing distance after the alteration: at least the requested
    /// distance, except for a target already opening from inside it (its
    /// current range).
    #[must_use]
    pub const fn least_passing(&self) -> Distance {
        self.least_passing
    }

    /// Target passing closest; set for picture-wide manoeuvres.
    #[must_use]
    pub const fn limiting_target(&self) -> Option<TargetId> {
        self.limiting_target
    }
}

/// One target as seen by the search.
#[derive(Clone, Copy)]
struct Encounter {
    target: Option<TargetId>,
    contact: Contact,
    vessel: Vessel,
}

/// Smallest permitted alteration giving `target` a CPA of at least `desired`.
///
/// # Errors
///
/// - [`KernelError::OutOfRange`] for a negative range or distance.
/// - [`NavigationError::NoSolution`] if no permitted alteration achieves it:
///   the target is already inside the distance and closing, or too fast
///   relative to own speed.
pub fn avoid(
    own: Vessel,
    contact: Contact,
    target: Vessel,
    desired: Distance,
    constraints: &ManoeuvreConstraints,
) -> Result<AvoidanceManoeuvre> {
    ensure_range("range", contact.range.nautical_miles(), 0.0, f64::MAX)?;
    ensure_range("desired distance", desired.nautical_miles(), 0.0, f64::MAX)?;

    let encounter = Encounter {
        target: None,
        contact,
        vessel: target,
    };
    // Exact plot answers seed the search.
    let exact = course_for_cpa(own, contact, target, desired).ok();
    let seeds = [
        exact.and_then(|avoidance| avoidance.starboard),
        exact.and_then(|avoidance| avoidance.port),
    ];
    search(own, &[encounter], desired, constraints, seeds)
}

/// Smallest permitted alteration giving every target a CPA of at least
/// `desired`, or leaving it opening.
///
/// Own ship from the snapshot; each target at its track's extrapolated position
/// at the snapshot time. Targets without motion are skipped.
///
/// # Errors
///
/// - [`KernelError::Indeterminate`] if the snapshot has no position or ground
///   track.
/// - As [`avoid`]; [`NavigationError::NoSolution`] if no permitted alteration
///   clears every target.
pub fn avoid_all<const N: usize>(
    state: &NavigationSnapshot,
    traffic: &Traffic<N>,
    desired: Distance,
    constraints: &ManoeuvreConstraints,
) -> Result<AvoidanceManoeuvre> {
    ensure_range("desired distance", desired.nautical_miles(), 0.0, f64::MAX)?;
    let observed = state
        .position()
        .ok_or(NavigationError::Kernel(KernelError::Missing {
            what: "the vessel's position",
        }))?;
    let ground = state
        .ground_track()
        .ok_or(NavigationError::Kernel(KernelError::Missing {
            what: "the vessel's course and speed over the ground",
        }))?;
    let own = Vessel {
        course: ground.course_over_ground,
        speed: ground.speed_over_ground,
    };
    let now = observed.taken_at();

    let placeholder = Encounter {
        target: None,
        contact: Contact {
            bearing: TrueBearing::NORTH,
            range: Distance::ZERO,
        },
        vessel: own,
    };
    let mut encounters = Inline::<Encounter, N>::new(placeholder);
    for track in traffic.tracks() {
        let Some(vessel) = track.as_vessel() else {
            continue;
        };
        let line = rhumb_line(*observed.value(), track.position_at(now)?)?;
        // The store has room for every track in the picture.
        let _ = encounters.push(Encounter {
            target: Some(track.target()),
            contact: Contact {
                bearing: TrueBearing::new(line.initial_course.degrees())?,
                range: line.distance,
            },
            vessel,
        });
    }
    search(own, &encounters, desired, constraints, [None, None])
}

/// Searches permitted alterations for the smallest that clears every encounter,
/// seeds first.
fn search(
    own: Vessel,
    encounters: &[Encounter],
    desired: Distance,
    constraints: &ManoeuvreConstraints,
    seeds: [Option<TrueCourse>; 2],
) -> Result<AvoidanceManoeuvre> {
    let least = constraints.least_alteration.degrees();
    let most = constraints.most_alteration.degrees();
    let mut best: Option<(f64, Distance, Option<TargetId>)> = None;

    for (sign, permitted, seed) in [
        (
            1.0,
            constraints.sides.starboard(),
            seeds.first().copied().flatten(),
        ),
        (
            -1.0,
            constraints.sides.port(),
            seeds.get(1).copied().flatten(),
        ),
    ] {
        if !permitted {
            continue;
        }
        // Exact alteration on this side, raised to the minimum.
        let seeded = seed
            .map(|course| wrap180(course.degrees() - own.course.degrees()))
            .filter(|alteration| alteration * sign > 0.0)
            .map(|alteration| math::abs(alteration).max(least));
        let found = seeded
            .filter(|&alteration| alteration <= most)
            .and_then(|alteration| clears(own, sign * alteration, encounters, desired))
            .map(|(passing, limiting)| (sign * seeded.unwrap_or(least), passing, limiting))
            .or_else(|| {
                let steps = math::to_usize((most - least) / ALTERATION_STEP_DEG);
                (0..=steps).find_map(|step| {
                    let alteration =
                        (least + math::count_to_f64(step) * ALTERATION_STEP_DEG).min(most);
                    clears(own, sign * alteration, encounters, desired)
                        .map(|(passing, limiting)| (sign * alteration, passing, limiting))
                })
            });
        if let Some((alteration, passing, limiting)) = found {
            // Smaller alteration wins; ties go to starboard (checked first).
            if best.is_none_or(|(current, _, _)| math::abs(alteration) < math::abs(current)) {
                best = Some((alteration, passing, limiting));
            }
        }
    }

    let (alteration, least_passing, limiting_target) = best.ok_or(NavigationError::NoSolution {
        context: "an alteration within the constraints that opens every target that far",
    })?;
    // Positive rate guaranteed by `with_rate_of_turn`.
    let turn_time = constraints.rate_of_turn.and_then(|rate| {
        Duration::try_from_secs_f64(math::abs(alteration) / rate.degrees_per_minute() * 60.0).ok()
    });
    Ok(AvoidanceManoeuvre {
        course: Direction::<True>::from_degrees_wrapped(own.course.degrees() + alteration),
        alteration: Angle::from_degrees(alteration)?,
        turn_time,
        least_passing,
        limiting_target,
    })
}

/// Whether an alteration clears every encounter: the minimum passing distance
/// and its target, or `None` if any is not cleared.
fn clears(
    own: Vessel,
    alteration: f64,
    encounters: &[Encounter],
    desired: Distance,
) -> Option<(Distance, Option<TargetId>)> {
    let altered = Vessel {
        course: Direction::<True>::from_degrees_wrapped(own.course.degrees() + alteration),
        speed: own.speed,
    };
    let mut least: Option<(Distance, Option<TargetId>)> = None;
    for encounter in encounters {
        let Ok(approach) = closest_point_of_approach(altered, encounter.contact, encounter.vessel)
        else {
            return None;
        };
        let passing = match approach {
            // Tolerance: the exact answer lands on the distance.
            Approach::Closing(cpa)
                if cpa.distance.nautical_miles() < desired.nautical_miles() - 1e-9 =>
            {
                return None;
            }
            Approach::Closing(cpa) => cpa.distance,
            // Opening or constant range: passes at the current range, which is
            // its minimum from here on; nothing to open, inside the distance or
            // not.
            Approach::Opening { current_range } => current_range,
            Approach::Stationary { range } => range,
            // `#[non_exhaustive]`: an unknown kind of approach cannot be
            // vouched for.
            _ => return None,
        };
        if least.is_none_or(|(current, _)| passing < current) {
            least = Some((passing, encounter.target));
        }
    }
    least.or(Some((
        Distance::from_nautical_miles_unchecked(f64::MAX),
        None,
    )))
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use crate::{TargetObservation, TrackingPolicy};
    use kinavis_kernel::event::PositionSource;
    use kinavis_kernel::observation::{ObservationStatus, Observed, Quality};
    use kinavis_kernel::position::Position;
    use kinavis_kernel::snapshot::GroundTrack;
    use kinavis_kernel::time::{Instant, Utc};
    use kinavis_kernel::units::Speed;

    fn knots(value: f64) -> Speed {
        Speed::from_knots(value).unwrap()
    }

    fn miles(value: f64) -> Distance {
        Distance::from_nautical_miles(value).unwrap()
    }

    fn degrees(value: f64) -> Angle {
        Angle::from_degrees(value).unwrap()
    }

    fn vessel(course: f64, speed: f64) -> Vessel {
        Vessel {
            course: TrueCourse::new(course).unwrap(),
            speed: knots(speed),
        }
    }

    fn contact(bearing: f64, range: f64) -> Contact {
        Contact {
            bearing: TrueBearing::new(bearing).unwrap(),
            range: miles(range),
        }
    }

    fn either_side(least: f64) -> ManoeuvreConstraints {
        ManoeuvreConstraints::new(PermittedSides::Either, degrees(least), degrees(120.0)).unwrap()
    }

    fn only(side: PermittedSides, least: f64) -> ManoeuvreConstraints {
        ManoeuvreConstraints::new(side, degrees(least), degrees(120.0)).unwrap()
    }

    /// Plot example: target fine on the starboard bow, nearly head-on, CPA
    /// under 1.5 NM.
    fn nearly_head_on() -> (Vessel, Contact, Vessel) {
        (vessel(0.0, 15.0), contact(10.0, 8.0), vessel(190.0, 12.0))
    }

    #[test]
    fn the_exact_answer_is_taken_when_it_is_large_enough() {
        let (own, contact, target) = nearly_head_on();
        let plot = course_for_cpa(own, contact, target, miles(2.0)).unwrap();
        let manoeuvre = avoid(own, contact, target, miles(2.0), &either_side(0.0)).unwrap();

        // Smaller of the two exact alterations: 16° to port vs 36° to
        // starboard. The side is the constraints' decision, not the search's.
        let exact = wrap180(plot.port.unwrap().degrees() - own.course.degrees());
        assert!(exact < 0.0 && exact > -20.0);
        assert!((manoeuvre.alteration().degrees() - exact).abs() < 1e-9);
        assert_eq!(manoeuvre.course(), plot.port.unwrap());
        assert_eq!(manoeuvre.side(), Side::Port);
        assert!((manoeuvre.least_passing().nautical_miles() - 2.0).abs() < 1e-6);
        assert_eq!(manoeuvre.limiting_target(), None);
        assert_eq!(manoeuvre.turn_time(), None);

        // Starboard only: the exact starboard answer.
        let starboard_only = only(PermittedSides::Starboard, 0.0);
        let manoeuvre = avoid(own, contact, target, miles(2.0), &starboard_only).unwrap();
        assert_eq!(manoeuvre.course(), plot.starboard.unwrap());
        assert!((manoeuvre.alteration().degrees() - 36.0).abs() < 0.1);
    }

    #[test]
    fn a_small_answer_is_made_readily_apparent() {
        let (own, contact, target) = nearly_head_on();
        let manoeuvre = avoid(own, contact, target, miles(2.0), &either_side(30.0)).unwrap();
        assert!((manoeuvre.alteration().degrees().abs() - 30.0).abs() < 1e-9);
        // A larger alteration opens the target further.
        assert!(manoeuvre.least_passing().nautical_miles() >= 2.0);
        let exact = avoid(own, contact, target, miles(2.0), &either_side(0.0)).unwrap();
        assert!(manoeuvre.least_passing() > exact.least_passing());

        // Minimum alteration beyond both exact answers: the search steps up
        // from it.
        let bold = avoid(own, contact, target, miles(2.0), &either_side(60.0)).unwrap();
        assert!(bold.alteration().degrees().abs() >= 60.0);
        assert!(bold.least_passing().nautical_miles() >= 2.0 - 1e-9);
    }

    #[test]
    fn only_a_permitted_side_is_taken() {
        let (own, contact, target) = nearly_head_on();
        let port_only = only(PermittedSides::Port, 0.0);
        let manoeuvre = avoid(own, contact, target, miles(2.0), &port_only).unwrap();
        assert_eq!(manoeuvre.side(), Side::Port);
        assert!(manoeuvre.alteration().degrees() < 0.0);
        assert!(manoeuvre.least_passing().nautical_miles() >= 2.0 - 1e-9);

        // "Neither side" is not representable.
        assert!(PermittedSides::Either.starboard() && PermittedSides::Either.port());
        assert!(PermittedSides::Starboard.starboard() && !PermittedSides::Starboard.port());
        assert!(!PermittedSides::Port.starboard() && PermittedSides::Port.port());
    }

    #[test]
    fn an_alteration_the_bounds_do_not_allow_is_no_solution() {
        let (own, contact, target) = nearly_head_on();
        let tiny =
            ManoeuvreConstraints::new(PermittedSides::Either, degrees(0.0), degrees(3.0)).unwrap();
        assert!(matches!(
            avoid(own, contact, target, miles(2.0), &tiny).unwrap_err(),
            NavigationError::NoSolution { .. }
        ));

        // Invalid bounds are rejected at construction.
        assert!(matches!(
            ManoeuvreConstraints::new(PermittedSides::Either, degrees(60.0), degrees(30.0))
                .unwrap_err(),
            NavigationError::Kernel(KernelError::OutOfRange {
                parameter: "most alteration",
                ..
            })
        ));
        assert!(
            ManoeuvreConstraints::new(PermittedSides::Either, degrees(0.0), degrees(200.0))
                .is_err()
        );
        assert!(
            ManoeuvreConstraints::new(PermittedSides::Either, degrees(-1.0), degrees(30.0))
                .is_err()
        );
        assert!(ManoeuvreConstraints::new(
            PermittedSides::Either,
            degrees(0.0),
            degrees(MAX_ALTERATION_DEG)
        )
        .is_ok());
    }

    #[test]
    fn a_target_too_fast_to_get_clear_of_is_no_solution() {
        // Target at 20 kn head-on; own 6 kn cannot shift the relative track to
        // pass 3 NM off.
        let own = vessel(0.0, 6.0);
        let contact = contact(0.0, 4.0);
        let target = vessel(180.0, 20.0);
        assert!(matches!(
            avoid(own, contact, target, miles(3.0), &either_side(0.0)).unwrap_err(),
            NavigationError::NoSolution { .. }
        ));
        // 1 NM is reachable with a large alteration.
        let manoeuvre = avoid(own, contact, target, miles(1.0), &either_side(0.0)).unwrap();
        assert!(manoeuvre.least_passing().nautical_miles() >= 1.0 - 1e-9);
    }

    #[test]
    fn the_turn_time_follows_the_rate_of_turn() {
        let (own, contact, target) = nearly_head_on();
        let ten_a_minute = RateOfTurn::from_degrees_per_minute(10.0).unwrap();
        let constraints = either_side(30.0).with_rate_of_turn(ten_a_minute).unwrap();
        let manoeuvre = avoid(own, contact, target, miles(2.0), &constraints).unwrap();
        assert_eq!(manoeuvre.turn_time(), Some(Duration::from_secs(180)));

        // A port rate is still a rate.
        let to_port = either_side(30.0).with_rate_of_turn(-ten_a_minute).unwrap();
        assert_eq!(to_port.rate_of_turn(), Some(ten_a_minute));

        // Zero rate is rejected.
        assert!(matches!(
            either_side(30.0)
                .with_rate_of_turn(RateOfTurn::ZERO)
                .unwrap_err(),
            NavigationError::Kernel(KernelError::OutOfRange {
                parameter: "rate of turn",
                ..
            })
        ));
        assert_eq!(either_side(30.0).rate_of_turn(), None);
    }

    #[test]
    fn a_target_already_inside_and_opening_is_left_alone() {
        // 0.5 NM on the quarter and opening: nothing to open; the answer is the
        // minimum alteration, which makes nothing worse.
        let manoeuvre = avoid(
            vessel(0.0, 12.0),
            contact(150.0, 0.5),
            vessel(150.0, 12.0),
            miles(2.0),
            &either_side(0.0),
        )
        .unwrap();
        assert!((manoeuvre.least_passing().nautical_miles() - 0.5).abs() < 1e-9);
    }

    fn noon() -> Instant<Utc> {
        Instant::from_unix_seconds(1_789_000_000)
    }

    fn own_ship() -> NavigationSnapshot {
        NavigationSnapshot::EMPTY
            .with_position(
                Observed::new(
                    Position::from_degrees(50.0, -1.0).unwrap(),
                    noon(),
                    Quality::<Distance>::new(ObservationStatus::Valid),
                ),
                PositionSource::Gnss,
            )
            .with_ground_track(GroundTrack {
                course_over_ground: TrueCourse::NORTH,
                speed_over_ground: knots(12.0),
            })
    }

    /// Target with reported course and speed, `range` NM off on the given
    /// bearing.
    fn reporting(id: u32, bearing: f64, range: f64, course: f64, speed: f64) -> TargetObservation {
        let there = kinavis::sailings::rhumb_destination(
            Position::from_degrees(50.0, -1.0).unwrap(),
            TrueCourse::new(bearing).unwrap(),
            miles(range),
        )
        .unwrap();
        TargetObservation::new(TargetId::new(id), there, noon()).with_ground_track(GroundTrack {
            course_over_ground: TrueCourse::new(course).unwrap(),
            speed_over_ground: knots(speed),
        })
    }

    fn traffic() -> Traffic {
        Traffic::new(
            TrackingPolicy::new(
                1,
                Duration::from_secs(30),
                Duration::from_secs(180),
                knots(60.0),
            )
            .unwrap(),
        )
    }

    #[test]
    fn a_manoeuvre_for_the_picture_clears_every_target() {
        let mut traffic = traffic();
        // Starboard bow, heading west at own speed: collision course.
        let _ = traffic
            .ingest(reporting(1, 45.0, 5.0, 270.0, 12.0))
            .unwrap();
        // Broad on the starboard bow, 3 NM, heading north with own ship: a
        // small starboard alteration would close it.
        let _ = traffic.ingest(reporting(2, 70.0, 3.0, 0.0, 12.0)).unwrap();
        // Plot without motion, excluded from the search.
        let _ = traffic
            .ingest(TargetObservation::new(
                TargetId::new(3),
                Position::from_degrees(50.2, -1.0).unwrap(),
                noon(),
            ))
            .unwrap();

        let alone = avoid(
            vessel(0.0, 12.0),
            contact(45.0, 5.0),
            vessel(270.0, 12.0),
            miles(1.0),
            &either_side(0.0),
        )
        .unwrap();
        let together = avoid_all(&own_ship(), &traffic, miles(1.0), &either_side(0.0)).unwrap();

        // Every target at ≥ 1 NM; the second one is limiting and needs more
        // than the first alone.
        assert!(together.least_passing().nautical_miles() >= 1.0 - 1e-6);
        assert!(together.limiting_target().is_some());
        assert!(together.alteration().degrees().abs() > alone.alteration().degrees().abs());

        for track in traffic.tracks() {
            let Some(target) = track.as_vessel() else {
                continue;
            };
            let line = rhumb_line(
                Position::from_degrees(50.0, -1.0).unwrap(),
                track.last_position(),
            )
            .unwrap();
            let contact = Contact {
                bearing: TrueBearing::new(line.initial_course.degrees()).unwrap(),
                range: line.distance,
            };
            if let Approach::Closing(cpa) = closest_point_of_approach(
                Vessel {
                    course: together.course(),
                    speed: knots(12.0),
                },
                contact,
                target,
            )
            .unwrap()
            {
                assert!(cpa.distance.nautical_miles() >= 1.0 - 1e-6);
            }
        }
    }

    #[test]
    fn an_empty_picture_needs_no_manoeuvre_and_says_so() {
        // Nothing to clear: the minimum alteration suffices.
        let manoeuvre = avoid_all(&own_ship(), &traffic(), miles(1.0), &either_side(30.0)).unwrap();
        assert!((manoeuvre.alteration().degrees() - 30.0).abs() < 1e-9);
        assert_eq!(manoeuvre.limiting_target(), None);

        assert!(matches!(
            avoid_all(
                &NavigationSnapshot::EMPTY,
                &traffic(),
                miles(1.0),
                &either_side(0.0)
            )
            .unwrap_err(),
            NavigationError::Kernel(KernelError::Missing { .. })
        ));
    }
}
