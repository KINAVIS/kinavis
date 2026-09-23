//! Passage schedule: ETD, ETA, and progress against plan.
//!
//! A [`Route`] gives distances; a [`RouteSchedule`] adds a departure time and a
//! speed per leg, from which every waypoint time follows. Underway,
//! [`RouteSchedule::estimate`] compares the schedule with the guidance position
//! and speed made good: distance and time to go, ETA at current speed, ahead or
//! behind, and the speed required to arrive on time.
//!
//! Validated at construction (every leg has a positive speed and the arrival is
//! representable), so reported times are plain values. An estimate is a
//! stateless projection like [`GuidanceView`].
//!
//! ```rust
//! use kinavis::route::{LegKind, Route};
//! use kinavis::schedule::RouteSchedule;
//! use kinavis::{Civil, Instant, Position, Speed, Utc};
//!
//! let route = Route::new(
//!     &[
//!         "50°06.0'N 001°30.0'W".parse::<Position>()?,
//!         "49°54.0'N 002°00.0'W".parse::<Position>()?,
//!         "49°42.0'N 002°45.0'W".parse::<Position>()?,
//!     ],
//!     LegKind::RhumbLine,
//! )?;
//! let departure = Instant::<Utc>::from_civil(Civil {
//!     hour: 6,
//!     ..Civil::date(2026, 9, 17)
//! })?;
//!
//! // Twelve knots on the first leg, ten on the second.
//! let schedule = RouteSchedule::with_leg_speeds(
//!     &route,
//!     departure,
//!     &[Speed::from_knots(12.0)?, Speed::from_knots(10.0)?],
//! )?;
//!
//! assert_eq!(schedule.leg_count(), 2);
//! // Twenty-three miles at twelve knots, then thirty-one at ten: five hours.
//! assert_eq!(format!("{:.0}", schedule.arrival()), "2026-09-17T11:02:19 UTC");
//! // The time at the first waypoint is the end of the first leg.
//! assert_eq!(schedule.eta(1), schedule.leg(0).map(|leg| leg.arrives));
//! # Ok::<(), kinavis::NavigationError>(())
//! ```

use core::time::Duration;

use crate::error::{KernelError, NavigationError, Result};
use crate::guidance::GuidanceView;
use crate::inline::Inline;
use crate::math;
use crate::route::{Route, MAX_WAYPOINTS};
use crate::snapshot::NavigationSnapshot;
use crate::time::{Instant, Utc};
use crate::units::{Distance, Speed};

/// Maximum legs per schedule (waypoints − 1).
const MAX_LEGS: usize = MAX_WAYPOINTS - 1;

/// Planned leg: distance and speed.
///
/// Stored instead of the route, so the schedule is unaffected by later route
/// edits.
#[derive(Debug, Clone, Copy, PartialEq)]
struct PlannedLeg {
    distance: Distance,
    speed: Speed,
}

impl PlannedLeg {
    /// Zero leg: store fill value, never read.
    const UNUSED: Self = Self {
        distance: Distance::ZERO,
        speed: Speed::ZERO,
    };

    /// Leg with a validated speed: positive, and covering the leg in a
    /// representable time.
    fn planned(distance: Distance, speed: Speed) -> Result<Self> {
        if speed.knots() <= 0.0 || !speed.knots().is_finite() {
            return Err(NavigationError::Kernel(KernelError::OutOfRange {
                parameter: "leg speed",
                value: speed.knots(),
                min: f64::MIN_POSITIVE,
                max: f64::MAX,
            }));
        }
        speed.time_to_cover(distance)?;
        Ok(Self { distance, speed })
    }

    /// Leg duration. Validated at planning, so failure is unreachable and
    /// yields zero.
    fn duration(self) -> Duration {
        self.speed.time_to_cover(self.distance).unwrap_or_default()
    }
}

/// Inline leg storage.
type PlannedLegs = Inline<PlannedLeg, MAX_LEGS>;

/// Scheduled leg with its times.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ScheduledLeg {
    /// Leg index, zero-based.
    pub index: u16,
    /// Length.
    pub distance: Distance,
    /// Planned speed.
    pub speed: Speed,
    /// Duration at that speed.
    pub duration: Duration,
    /// Departure from the leg start.
    pub departs: Instant<Utc>,
    /// Arrival at the leg end.
    pub arrives: Instant<Utc>,
}

/// Passage plan with times: a speed per leg and a departure.
///
/// Built from a [`Route`]; legs follow the route order, and
/// [`RouteSchedule::estimate`] expects a [`GuidanceView`] along the same route.
/// Stored inline and large: pass by reference.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(try_from = "StoredSchedule", into = "StoredSchedule")
)]
pub struct RouteSchedule {
    departure: Instant<Utc>,
    legs: PlannedLegs,
}

impl RouteSchedule {
    /// Whole route at one speed, departing at `departure`.
    ///
    /// # Errors
    ///
    /// - [`KernelError::OutOfRange`] if `speed` is not positive.
    /// - [`KernelError::Indeterminate`] if a leg duration or the arrival time
    ///   is unrepresentable.
    /// - Sailing failures from [`Route::legs`].
    pub fn new(route: &Route, departure: Instant<Utc>, speed: Speed) -> Result<Self> {
        let mut legs = PlannedLegs::new(PlannedLeg::UNUSED);
        for leg in route.legs() {
            push_leg(
                &mut legs,
                PlannedLeg::planned(leg?.sailing.distance, speed)?,
            )?;
        }
        Self::from_legs(departure, &legs)
    }

    /// Route with one speed per leg, in leg order.
    ///
    /// # Errors
    ///
    /// - [`KernelError::OutOfRange`] unless there is exactly one positive speed
    ///   per leg.
    /// - Otherwise as [`RouteSchedule::new`].
    pub fn with_leg_speeds(
        route: &Route,
        departure: Instant<Utc>,
        speeds: &[Speed],
    ) -> Result<Self> {
        if speeds.len() != route.leg_count() {
            return Err(NavigationError::Kernel(KernelError::OutOfRange {
                parameter: "leg speeds",
                value: math::count_to_f64(speeds.len()),
                min: math::count_to_f64(route.leg_count()),
                max: math::count_to_f64(route.leg_count()),
            }));
        }
        let mut legs = PlannedLegs::new(PlannedLeg::UNUSED);
        for (leg, &speed) in route.legs().zip(speeds) {
            push_leg(
                &mut legs,
                PlannedLeg::planned(leg?.sailing.distance, speed)?,
            )?;
        }
        Self::from_legs(departure, &legs)
    }

    /// Single speed that arrives at the end of the route at `arrival`,
    /// departing at `departure`.
    ///
    /// # Errors
    ///
    /// - [`KernelError::TimeReversed`] if `arrival` precedes `departure`.
    /// - [`KernelError::Indeterminate`] if they are equal.
    /// - Otherwise as [`RouteSchedule::new`].
    pub fn to_arrive_by(
        route: &Route,
        departure: Instant<Utc>,
        arrival: Instant<Utc>,
    ) -> Result<Self> {
        let available = arrival.duration_since(departure)?;
        Self::new(route, departure, route.speed_required(available)?)
    }

    /// Schedule from planned legs, validated as a whole: at least one leg,
    /// representable arrival.
    fn from_legs(departure: Instant<Utc>, legs: &PlannedLegs) -> Result<Self> {
        if legs.is_empty() {
            return Err(NavigationError::Kernel(KernelError::InsufficientData {
                found: 0,
                required: 1,
                context: "a schedule",
            }));
        }
        let mut passage = Duration::ZERO;
        for leg in legs.iter() {
            passage = passage
                .checked_add(leg.duration())
                .ok_or(NavigationError::Kernel(KernelError::Unrepresentable {
                    what: "the passage time",
                }))?;
        }
        departure
            .checked_add(passage)
            .ok_or(NavigationError::Kernel(KernelError::Unrepresentable {
                what: "the time of arrival",
            }))?;
        Ok(Self {
            departure,
            legs: *legs,
        })
    }

    /// Replans one leg at another speed; later times shift accordingly.
    ///
    /// # Errors
    ///
    /// - [`KernelError::OutOfRange`] for a nonexistent leg or non-positive
    ///   speed.
    /// - [`KernelError::Indeterminate`] as [`RouteSchedule::new`]. Unchanged on
    ///   error.
    pub fn set_leg_speed(&mut self, leg: u16, speed: Speed) -> Result<()> {
        let mut legs = self.legs;
        let slot = legs
            .as_mut_slice()
            .get_mut(usize::from(leg))
            .ok_or(NavigationError::Kernel(KernelError::OutOfRange {
                parameter: "leg",
                value: f64::from(leg),
                min: 0.0,
                max: math::count_to_f64(self.legs.len().saturating_sub(1)),
            }))?;
        *slot = PlannedLeg::planned(slot.distance, speed)?;
        *self = Self::from_legs(self.departure, &legs)?;
        Ok(())
    }

    /// Moves the departure and every later time.
    ///
    /// # Errors
    ///
    /// [`KernelError::Indeterminate`] if the arrival becomes unrepresentable.
    /// Unchanged on error.
    pub fn set_departure(&mut self, departure: Instant<Utc>) -> Result<()> {
        *self = Self::from_legs(departure, &self.legs)?;
        Ok(())
    }

    /// ETD at the first waypoint.
    #[must_use]
    pub const fn departure(&self) -> Instant<Utc> {
        self.departure
    }

    /// Number of legs.
    #[must_use]
    pub const fn leg_count(&self) -> usize {
        self.legs.len()
    }

    /// Legs with speeds and times.
    pub fn legs(&self) -> impl Iterator<Item = ScheduledLeg> + '_ {
        self.legs
            .iter()
            .scan((0_u16, self.departure), |(index, clock), leg| {
                let duration = leg.duration();
                let departs = *clock;
                let arrives = departs.saturating_add(duration);
                let scheduled = ScheduledLeg {
                    index: *index,
                    distance: leg.distance,
                    speed: leg.speed,
                    duration,
                    departs,
                    arrives,
                };
                *index = index.saturating_add(1);
                *clock = arrives;
                Some(scheduled)
            })
    }

    /// Leg with times; `None` past the last.
    #[must_use]
    pub fn leg(&self, index: u16) -> Option<ScheduledLeg> {
        self.legs().nth(usize::from(index))
    }

    /// Planned speed of a leg; `None` past the last.
    #[must_use]
    pub fn leg_speed(&self, index: u16) -> Option<Speed> {
        self.legs.get(usize::from(index)).map(|leg| leg.speed)
    }

    /// Planned time at a waypoint: departure for waypoint `0`, then each leg
    /// end. `None` past the last.
    #[must_use]
    pub fn eta(&self, waypoint: u16) -> Option<Instant<Utc>> {
        match waypoint.checked_sub(1) {
            None => Some(self.departure),
            Some(leg) => self.leg(leg).map(|leg| leg.arrives),
        }
    }

    /// ETA at the end of the route.
    #[must_use]
    pub fn arrival(&self) -> Instant<Utc> {
        self.departure.saturating_add(self.passage_time())
    }

    /// Planned passage duration.
    #[must_use]
    pub fn passage_time(&self) -> Duration {
        self.legs.iter().fold(Duration::ZERO, |sum, leg| {
            sum.saturating_add(leg.duration())
        })
    }

    /// Route length.
    #[must_use]
    pub fn total_distance(&self) -> Distance {
        let total = self
            .legs
            .iter()
            .fold(0.0, |sum, leg| sum + leg.distance.nautical_miles());
        Distance::from_nautical_miles_unchecked(total)
    }

    /// Schedule compared with the current position.
    ///
    /// `view` locates the vessel on the route (active leg, distance to its end
    /// and to the route end); `state` supplies time and ground track. Speed
    /// made good is SOG resolved along the desired track, so crabbing across
    /// the track counts only its along-track progress.
    ///
    /// `view` must come from the same route; a nonexistent leg index is
    /// rejected, but a different route of the same shape is not detected.
    ///
    /// # Errors
    ///
    /// - [`KernelError::Indeterminate`] if the snapshot has no position (no
    ///   time), or the planned time to go is unrepresentable.
    /// - [`KernelError::OutOfRange`] if the view's active leg is not in this
    ///   schedule.
    pub fn estimate(
        &self,
        state: &NavigationSnapshot,
        view: &GuidanceView,
    ) -> Result<RouteEstimate> {
        let observed = state
            .position()
            .ok_or(NavigationError::Kernel(KernelError::Missing {
                what: "the vessel's position",
            }))?;
        let at = observed.taken_at();

        let active = usize::from(view.active_leg().index());
        let leg =
            self.legs
                .get(active)
                .ok_or(NavigationError::Kernel(KernelError::OutOfRange {
                    parameter: "active leg",
                    value: f64::from(view.active_leg().index()),
                    min: 0.0,
                    max: math::count_to_f64(self.legs.len().saturating_sub(1)),
                }))?;

        // Plan from here: rest of this leg at its speed, then the following
        // legs as scheduled.
        let mut planned_time_to_go = leg.speed.time_to_cover(view.distance_to_waypoint())?;
        for later in self.legs.iter().skip(active + 1) {
            planned_time_to_go =
                planned_time_to_go
                    .checked_add(later.duration())
                    .ok_or(NavigationError::Kernel(KernelError::Unrepresentable {
                        what: "the planned time to go",
                    }))?;
        }
        let eta_at_planned_speed =
            at.checked_add(planned_time_to_go)
                .ok_or(NavigationError::Kernel(KernelError::Unrepresentable {
                    what: "the time of arrival",
                }))?;
        let scheduled_arrival = self.arrival();
        let scheduled_here =
            scheduled_arrival
                .checked_sub(planned_time_to_go)
                .ok_or(NavigationError::Kernel(KernelError::Unrepresentable {
                    what: "the scheduled time here",
                }))?;

        let distance_to_go = view.distance_to_end();
        let ground = state.ground_track();
        let speed_made_good = ground.map(|ground| {
            let off_track = ground.course_over_ground.degrees() - view.desired_track().degrees();
            Speed::from_knots_unchecked(
                ground.speed_over_ground.knots() * math::cos(off_track.to_radians()),
            )
        });
        // Abeam of the track, speed made good is only rounding: treated as no
        // way, not an enormous time.
        let closing = speed_made_good.filter(|speed| {
            let over_ground = ground.map_or(0.0, |ground| ground.speed_over_ground.knots());
            speed.knots() > 0.0 && !math::is_effectively_zero(speed.knots(), over_ground)
        });
        let time_to_go = closing.and_then(|speed| speed.time_to_cover(distance_to_go).ok());
        let eta = time_to_go.and_then(|time| at.checked_add(time));

        let speed_required = scheduled_arrival
            .checked_duration_since(at)
            .and_then(|available| {
                crate::dead_reckoning::speed_required(distance_to_go, available).ok()
            });

        Ok(RouteEstimate {
            at,
            distance_to_go,
            speed_made_good,
            time_to_go,
            eta,
            planned_time_to_go,
            eta_at_planned_speed,
            scheduled_arrival,
            scheduled_here,
            speed_required,
        })
    }
}

/// Progress against schedule at one instant.
///
/// Projection built by [`RouteSchedule::estimate`], valid as of
/// [`RouteEstimate::at`].
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct RouteEstimate {
    at: Instant<Utc>,
    distance_to_go: Distance,
    speed_made_good: Option<Speed>,
    time_to_go: Option<Duration>,
    eta: Option<Instant<Utc>>,
    planned_time_to_go: Duration,
    eta_at_planned_speed: Instant<Utc>,
    scheduled_arrival: Instant<Utc>,
    scheduled_here: Instant<Utc>,
    speed_required: Option<Speed>,
}

impl RouteEstimate {
    /// Time of the estimate (position time).
    #[must_use]
    pub const fn at(&self) -> Instant<Utc> {
        self.at
    }

    /// Distance to go: direct to the next waypoint, then leg by leg.
    #[must_use]
    pub const fn distance_to_go(&self) -> Distance {
        self.distance_to_go
    }

    /// Speed made good along the desired track (SOG resolved onto it); zero or
    /// negative when not closing the next waypoint. `None` without a ground
    /// track.
    #[must_use]
    pub const fn speed_made_good(&self) -> Option<Speed> {
        self.speed_made_good
    }

    /// Time to go at the current speed made good. `None` without a ground track
    /// or with no progress along track.
    #[must_use]
    pub const fn time_to_go(&self) -> Option<Duration> {
        self.time_to_go
    }

    /// ETA at the current speed made good. `None` as
    /// [`RouteEstimate::time_to_go`].
    #[must_use]
    pub const fn eta(&self) -> Option<Instant<Utc>> {
        self.eta
    }

    /// Time to go at planned speeds from here.
    #[must_use]
    pub const fn planned_time_to_go(&self) -> Duration {
        self.planned_time_to_go
    }

    /// ETA at planned speeds from here.
    #[must_use]
    pub const fn eta_at_planned_speed(&self) -> Instant<Utc> {
        self.eta_at_planned_speed
    }

    /// Scheduled ETA: [`RouteSchedule::arrival`].
    #[must_use]
    pub const fn scheduled_arrival(&self) -> Instant<Utc> {
        self.scheduled_arrival
    }

    /// Scheduled time for the current position: scheduled arrival minus planned
    /// time to go. Before [`RouteEstimate::at`] when behind, after it when
    /// ahead.
    #[must_use]
    pub const fn scheduled_here(&self) -> Instant<Utc> {
        self.scheduled_here
    }

    /// Time ahead of schedule; `Some` when on time or early (on time is zero
    /// ahead).
    #[must_use]
    pub fn ahead_of_schedule(&self) -> Option<Duration> {
        self.scheduled_here.checked_duration_since(self.at)
    }

    /// Time behind schedule; `Some` when late.
    #[must_use]
    pub fn behind_schedule(&self) -> Option<Duration> {
        if self.at > self.scheduled_here {
            self.at.checked_duration_since(self.scheduled_here)
        } else {
            None
        }
    }

    /// SOG required to arrive at the scheduled ETA. `None` once it has passed.
    #[must_use]
    pub const fn speed_required(&self) -> Option<Speed> {
        self.speed_required
    }
}

/// Serialised form; deserialisation checks the invariants.
#[cfg(feature = "serde")]
#[derive(serde::Serialize, serde::Deserialize)]
struct StoredSchedule {
    departure: Instant<Utc>,
    legs: alloc::vec::Vec<StoredLeg>,
}

/// Serialised leg.
#[cfg(feature = "serde")]
#[derive(serde::Serialize, serde::Deserialize)]
struct StoredLeg {
    distance: Distance,
    speed: Speed,
}

#[cfg(feature = "serde")]
impl TryFrom<StoredSchedule> for RouteSchedule {
    type Error = NavigationError;

    /// Deserialised through the planning checks; stopped legs or no legs are
    /// rejected.
    fn try_from(stored: StoredSchedule) -> Result<Self> {
        let mut legs = PlannedLegs::new(PlannedLeg::UNUSED);
        for leg in &stored.legs {
            if leg.distance.is_negative() {
                return Err(NavigationError::Kernel(KernelError::OutOfRange {
                    parameter: "leg distance",
                    value: leg.distance.nautical_miles(),
                    min: 0.0,
                    max: f64::MAX,
                }));
            }
            push_leg(&mut legs, PlannedLeg::planned(leg.distance, leg.speed)?)?;
        }
        Self::from_legs(stored.departure, &legs)
    }
}

#[cfg(feature = "serde")]
impl From<RouteSchedule> for StoredSchedule {
    fn from(schedule: RouteSchedule) -> Self {
        Self {
            departure: schedule.departure,
            legs: schedule
                .legs
                .iter()
                .map(|leg| StoredLeg {
                    distance: leg.distance,
                    speed: leg.speed,
                })
                .collect(),
        }
    }
}

/// Appends a planned leg, or reports the required capacity.
fn push_leg(legs: &mut PlannedLegs, leg: PlannedLeg) -> Result<()> {
    legs.push(leg).map_err(|full| {
        NavigationError::Kernel(KernelError::CapacityExceeded {
            context: "a schedule",
            needed: full.capacity + 1,
            capacity: full.capacity,
        })
    })
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use crate::angle::TrueCourse;
    use crate::environment::EnvironmentSample;
    use crate::event::PositionSource;
    use crate::geodesy::{GeodeticPoint, Height};
    use crate::guidance::{guide, GuidanceConfig};
    use crate::observation::{ObservationStatus, Observed, Quality};
    use crate::position::Position;
    use crate::route::LegKind;
    use crate::snapshot::GroundTrack;

    fn at(latitude: f64, longitude: f64) -> Position {
        Position::from_degrees(latitude, longitude).unwrap()
    }

    fn noon() -> Instant<Utc> {
        Instant::from_unix_seconds(1_789_000_000)
    }

    fn knots(value: f64) -> Speed {
        Speed::from_knots(value).unwrap()
    }

    fn hours(value: f64) -> Duration {
        Duration::from_secs_f64(value * 3600.0)
    }

    /// 60 NM north, 60 NM east, 60 NM south.
    fn square() -> Route {
        Route::new(
            &[at(0.0, 0.0), at(1.0, 0.0), at(1.0, 1.0), at(0.0, 1.0)],
            LegKind::RhumbLine,
        )
        .unwrap()
    }

    fn twelve_knots() -> RouteSchedule {
        RouteSchedule::new(&square(), noon(), knots(12.0)).unwrap()
    }

    fn snapshot(position: Position, when: Instant<Utc>) -> NavigationSnapshot {
        NavigationSnapshot::EMPTY.with_position(
            Observed::new(
                position,
                when,
                Quality::<Distance>::new(ObservationStatus::Valid),
            ),
            PositionSource::Gnss,
        )
    }

    fn under_way(
        position: Position,
        when: Instant<Utc>,
        course: f64,
        speed: f64,
    ) -> NavigationSnapshot {
        snapshot(position, when).with_ground_track(GroundTrack {
            course_over_ground: TrueCourse::new(course).unwrap(),
            speed_over_ground: knots(speed),
        })
    }

    fn view_at(state: &NavigationSnapshot, active: u16) -> GuidanceView {
        let here = *state.position().unwrap().value();
        let environment = EnvironmentSample::at(
            GeodeticPoint::new(here, Height::above_ellipsoid(Distance::ZERO)),
            noon(),
        );
        let config = GuidanceConfig::new(
            Distance::from_nautical_miles(0.5).unwrap(),
            Distance::from_nautical_miles(0.2).unwrap(),
        )
        .unwrap();
        guide(
            state,
            &square(),
            square().leg(active).unwrap(),
            &environment,
            &config,
        )
        .unwrap()
        .0
    }

    /// Scheduled time for 30 NM along the first leg at 12 kn.
    fn halfway_up_the_first_leg() -> (Position, Instant<Utc>) {
        let here = at(0.5, 0.0);
        let schedule = twelve_knots();
        let view = view_at(&snapshot(here, noon()), 0);
        let run = schedule.leg(0).unwrap().distance - view.distance_to_waypoint();
        let when = noon()
            .checked_add(knots(12.0).time_to_cover(run).unwrap())
            .unwrap();
        (here, when)
    }

    /// Equal to the millisecond: leg times are float quotients and may differ
    /// from their sum in the last nanosecond.
    fn same_length(left: Duration, right: Duration) -> bool {
        left.abs_diff(right) <= Duration::from_millis(1)
    }

    fn within_a_second(left: Instant<Utc>, right: Instant<Utc>) -> bool {
        let (earlier, later) = if left < right {
            (left, right)
        } else {
            (right, left)
        };
        later.checked_duration_since(earlier).unwrap() <= Duration::from_secs(1)
    }

    #[test]
    fn one_speed_for_the_whole_route_is_the_route_s_own_schedule() {
        let route = square();
        let schedule = twelve_knots();

        assert_eq!(schedule.leg_count(), route.leg_count());
        assert_eq!(schedule.departure(), noon());
        assert!(same_length(
            schedule.passage_time(),
            route.passage_time(knots(12.0)).unwrap()
        ));
        assert_eq!(
            schedule.arrival(),
            noon().checked_add(schedule.passage_time()).unwrap()
        );
        assert!(
            (schedule.total_distance().nautical_miles()
                - route.total_distance().unwrap().nautical_miles())
            .abs()
                < 1e-9
        );

        // Waypoint times run from departure to arrival.
        assert_eq!(schedule.eta(0), Some(noon()));
        assert_eq!(schedule.eta(3), Some(schedule.arrival()));
        assert_eq!(schedule.eta(4), None);
        assert_eq!(schedule.leg(3), None);
        assert_eq!(schedule.leg_speed(1), Some(knots(12.0)));
        assert_eq!(schedule.leg_speed(3), None);

        // Each leg starts where the previous ends.
        let mut clock = noon();
        for (index, leg) in schedule.legs().enumerate() {
            assert_eq!(usize::from(leg.index), index);
            assert_eq!(leg.departs, clock);
            assert_eq!(leg.arrives, clock.checked_add(leg.duration).unwrap());
            assert_eq!(leg.duration, leg.speed.time_to_cover(leg.distance).unwrap());
            clock = leg.arrives;
        }
        assert_eq!(clock, schedule.arrival());
    }

    #[test]
    fn a_slower_leg_arrives_later_and_only_from_then_on() {
        let uniform = twelve_knots();
        let slow_middle = RouteSchedule::with_leg_speeds(
            &square(),
            noon(),
            &[knots(12.0), knots(6.0), knots(12.0)],
        )
        .unwrap();

        assert_eq!(slow_middle.eta(1), uniform.eta(1));
        assert!(slow_middle.eta(2) > uniform.eta(2));
        let middle = slow_middle.leg(1).unwrap();
        let first = slow_middle.leg(0).unwrap();
        assert!((middle.duration.as_secs_f64() / first.duration.as_secs_f64() - 2.0).abs() < 0.01);
        // Halving the middle leg's speed adds its previous duration.
        assert!(same_length(
            slow_middle.passage_time(),
            uniform
                .passage_time()
                .checked_add(uniform.leg(1).unwrap().duration)
                .unwrap()
        ));
    }

    #[test]
    fn the_speeds_must_match_the_legs() {
        let two = [knots(12.0), knots(12.0)];
        let four = [knots(12.0); 4];
        for speeds in [&two[..], &four[..], &[]] {
            assert!(matches!(
                RouteSchedule::with_leg_speeds(&square(), noon(), speeds).unwrap_err(),
                NavigationError::Kernel(KernelError::OutOfRange {
                    parameter: "leg speeds",
                    min: 3.0,
                    max: 3.0,
                    ..
                })
            ));
        }
    }

    #[test]
    fn a_stopped_or_backing_leg_keeps_no_schedule() {
        for speed in [Speed::ZERO, knots(-5.0)] {
            assert!(matches!(
                RouteSchedule::new(&square(), noon(), speed).unwrap_err(),
                NavigationError::Kernel(KernelError::OutOfRange {
                    parameter: "leg speed",
                    ..
                })
            ));
        }
        assert!(RouteSchedule::with_leg_speeds(
            &square(),
            noon(),
            &[knots(12.0), Speed::ZERO, knots(12.0)]
        )
        .is_err());
    }

    #[test]
    fn to_arrive_by_plans_the_speed_that_gets_there() {
        let route = square();
        let arrival = noon().checked_add(hours(15.0)).unwrap();
        let schedule = RouteSchedule::to_arrive_by(&route, noon(), arrival).unwrap();

        assert!(within_a_second(schedule.arrival(), arrival));
        let expected = route.total_distance().unwrap().nautical_miles() / 15.0;
        assert!((schedule.leg_speed(0).unwrap().knots() - expected).abs() < 1e-9);
        assert_eq!(schedule.leg_speed(0), schedule.leg_speed(2));

        assert!(matches!(
            RouteSchedule::to_arrive_by(&route, arrival, noon()).unwrap_err(),
            NavigationError::Kernel(KernelError::TimeReversed { .. })
        ));
        assert!(matches!(
            RouteSchedule::to_arrive_by(&route, noon(), noon()).unwrap_err(),
            NavigationError::Kernel(KernelError::Indeterminate { .. })
        ));
    }

    #[test]
    fn replanning_a_leg_moves_only_the_times_after_it() {
        let original = twelve_knots();
        let mut schedule = original;
        schedule.set_leg_speed(1, knots(6.0)).unwrap();

        assert_eq!(schedule.eta(1), original.eta(1));
        assert!(schedule.eta(2) > original.eta(2));
        assert!(schedule.arrival() > original.arrival());
        assert_eq!(schedule.leg_speed(1), Some(knots(6.0)));
        assert_eq!(schedule.leg_speed(2), Some(knots(12.0)));

        // A nonexistent leg or invalid speed leaves the schedule unchanged.
        let before = schedule;
        assert!(matches!(
            schedule.set_leg_speed(3, knots(6.0)).unwrap_err(),
            NavigationError::Kernel(KernelError::OutOfRange {
                parameter: "leg",
                ..
            })
        ));
        assert!(schedule.set_leg_speed(1, Speed::ZERO).is_err());
        assert_eq!(schedule, before);
    }

    #[test]
    fn moving_the_departure_moves_every_time_with_it() {
        let original = twelve_knots();
        let mut schedule = original;
        let later = noon().checked_add(hours(2.0)).unwrap();
        schedule.set_departure(later).unwrap();

        assert_eq!(schedule.departure(), later);
        assert_eq!(schedule.passage_time(), original.passage_time());
        for (moved, kept) in schedule.legs().zip(original.legs()) {
            assert_eq!(moved.departs, kept.departs.checked_add(hours(2.0)).unwrap());
            assert_eq!(moved.arrives, kept.arrives.checked_add(hours(2.0)).unwrap());
        }
    }

    #[test]
    fn an_arrival_off_the_clock_is_refused() {
        let end_of_time = Instant::from_unix_seconds(i64::MAX - 10);
        assert!(matches!(
            RouteSchedule::new(&square(), end_of_time, knots(12.0)).unwrap_err(),
            NavigationError::Kernel(KernelError::Unrepresentable {
                what: "the time of arrival"
            })
        ));

        let mut schedule = twelve_knots();
        let before = schedule;
        assert!(schedule.set_departure(end_of_time).is_err());
        assert_eq!(schedule, before);
    }

    #[test]
    fn on_time_at_the_planned_speed_the_estimate_agrees_with_the_schedule() {
        let schedule = twelve_knots();
        let (here, when) = halfway_up_the_first_leg();
        let state = under_way(here, when, 0.0, 12.0);
        let estimate = schedule.estimate(&state, &view_at(&state, 0)).unwrap();

        assert_eq!(estimate.at(), when);
        assert!((estimate.distance_to_go().nautical_miles() - 150.0).abs() < 0.2);
        assert!((estimate.speed_made_good().unwrap().knots() - 12.0).abs() < 1e-9);
        assert!(same_length(
            estimate.time_to_go().unwrap(),
            estimate.planned_time_to_go()
        ));
        assert!(within_a_second(estimate.eta().unwrap(), schedule.arrival()));
        assert!(within_a_second(
            estimate.eta_at_planned_speed(),
            schedule.arrival()
        ));
        assert_eq!(estimate.scheduled_arrival(), schedule.arrival());
        assert!(within_a_second(estimate.scheduled_here(), when));

        // On time: zero ahead, not behind.
        assert!(estimate.ahead_of_schedule().unwrap() <= Duration::from_secs(1));
        assert_eq!(estimate.behind_schedule(), None);
        assert!((estimate.speed_required().unwrap().knots() - 12.0).abs() < 1e-3);
    }

    #[test]
    fn late_and_slow_she_is_behind_and_must_make_more_speed() {
        let schedule = twelve_knots();
        let (here, on_time) = halfway_up_the_first_leg();
        let late = on_time.checked_add(hours(1.0)).unwrap();
        let state = under_way(here, late, 0.0, 10.0);
        let estimate = schedule.estimate(&state, &view_at(&state, 0)).unwrap();

        let behind = estimate.behind_schedule().unwrap();
        assert!((behind.as_secs_f64() - 3600.0).abs() <= 1.0);
        assert_eq!(estimate.ahead_of_schedule(), None);

        // At 10 kn arrival is later still; at planned speeds 1 h late; on time
        // requires more than 12 kn.
        assert!(estimate.eta().unwrap() > estimate.eta_at_planned_speed());
        assert!(within_a_second(
            estimate.eta_at_planned_speed(),
            schedule.arrival().checked_add(hours(1.0)).unwrap()
        ));
        assert!(estimate.speed_required().unwrap().knots() > 12.0);
    }

    #[test]
    fn early_she_is_ahead_and_may_slow_down() {
        let schedule = twelve_knots();
        let (here, on_time) = halfway_up_the_first_leg();
        let early = on_time.checked_sub(hours(0.5)).unwrap();
        let state = under_way(here, early, 0.0, 12.0);
        let estimate = schedule.estimate(&state, &view_at(&state, 0)).unwrap();

        let ahead = estimate.ahead_of_schedule().unwrap();
        assert!((ahead.as_secs_f64() - 1800.0).abs() <= 1.0);
        assert_eq!(estimate.behind_schedule(), None);
        assert!(estimate.speed_required().unwrap().knots() < 12.0);
        assert!(estimate.eta().unwrap() < schedule.arrival());
    }

    #[test]
    fn crabbing_across_the_track_counts_only_the_way_made_along_it() {
        let schedule = twelve_knots();
        let (here, when) = halfway_up_the_first_leg();
        // 12 kn at 60° off a northbound track: 6 kn along it.
        let state = under_way(here, when, 60.0, 12.0);
        let estimate = schedule.estimate(&state, &view_at(&state, 0)).unwrap();

        assert!((estimate.speed_made_good().unwrap().knots() - 6.0).abs() < 1e-9);
        assert_eq!(
            estimate.time_to_go(),
            Some(knots(6.0).time_to_cover(estimate.distance_to_go()).unwrap())
        );
    }

    #[test]
    fn making_no_way_along_the_track_gives_no_eta() {
        let schedule = twelve_knots();
        let (here, when) = halfway_up_the_first_leg();
        // Abeam of the track and heading back along it.
        for course in [90.0, 180.0] {
            let state = under_way(here, when, course, 12.0);
            let estimate = schedule.estimate(&state, &view_at(&state, 0)).unwrap();
            assert!(estimate.speed_made_good().unwrap().knots() < 1e-9);
            assert_eq!(estimate.time_to_go(), None);
            assert_eq!(estimate.eta(), None);
            // The planned figures remain available.
            assert!(estimate.planned_time_to_go() > Duration::ZERO);
            assert!(estimate.speed_required().is_some());
        }
    }

    #[test]
    fn without_a_ground_track_there_is_a_plan_but_no_eta() {
        let schedule = twelve_knots();
        let (here, when) = halfway_up_the_first_leg();
        let state = snapshot(here, when);
        let estimate = schedule.estimate(&state, &view_at(&state, 0)).unwrap();

        assert_eq!(estimate.speed_made_good(), None);
        assert_eq!(estimate.time_to_go(), None);
        assert_eq!(estimate.eta(), None);
        assert!(within_a_second(
            estimate.eta_at_planned_speed(),
            schedule.arrival()
        ));
        assert!((estimate.speed_required().unwrap().knots() - 12.0).abs() < 1e-3);
    }

    #[test]
    fn once_the_scheduled_arrival_has_passed_no_speed_will_do() {
        let schedule = twelve_knots();
        let here = at(0.5, 0.0);
        let too_late = schedule.arrival().checked_add(hours(1.0)).unwrap();
        let state = under_way(here, too_late, 0.0, 12.0);
        let estimate = schedule.estimate(&state, &view_at(&state, 0)).unwrap();

        assert_eq!(estimate.speed_required(), None);
        assert!(estimate.behind_schedule().unwrap() > hours(1.0));
        // ETA at the current speed is still available.
        assert!(estimate.eta().is_some());
    }

    #[test]
    fn the_estimate_moves_along_the_legs_with_the_view() {
        let schedule = twelve_knots();
        // Halfway along the second leg, heading east.
        let here = at(1.0, 0.5);
        let state = under_way(here, noon(), 90.0, 12.0);
        let estimate = schedule.estimate(&state, &view_at(&state, 1)).unwrap();

        assert!((estimate.distance_to_go().nautical_miles() - 90.0).abs() < 0.2);
        // 30 NM of this leg and 60 NM of the last at 12 kn.
        assert!((estimate.planned_time_to_go().as_secs_f64() / 3600.0 - 7.5).abs() < 0.02);
    }

    #[test]
    fn an_estimate_needs_a_position() {
        let schedule = twelve_knots();
        let state = under_way(at(0.5, 0.0), noon(), 0.0, 12.0);
        let view = view_at(&state, 0);
        assert!(matches!(
            schedule
                .estimate(&NavigationSnapshot::EMPTY, &view)
                .unwrap_err(),
            NavigationError::Kernel(KernelError::Missing { .. })
        ));
    }

    #[test]
    fn a_view_from_a_longer_route_is_refused() {
        // A one-leg schedule queried for the third leg.
        let short = Route::new(&[at(0.0, 0.0), at(1.0, 0.0)], LegKind::RhumbLine).unwrap();
        let schedule = RouteSchedule::new(&short, noon(), knots(12.0)).unwrap();
        let state = under_way(at(0.5, 1.0), noon(), 180.0, 12.0);
        let view = view_at(&state, 2);
        assert!(matches!(
            schedule.estimate(&state, &view).unwrap_err(),
            NavigationError::Kernel(KernelError::OutOfRange {
                parameter: "active leg",
                value: 2.0,
                max: 0.0,
                ..
            })
        ));
    }
}
