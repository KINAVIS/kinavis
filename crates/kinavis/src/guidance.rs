//! Route guidance: what to steer now, and route events.
//!
//! Inputs: a [`Route`], the vessel's [`NavigationSnapshot`] (from the estimator
//! or intake), and an [`EnvironmentSample`]. Outputs: the track to make good,
//! the heading that makes it good against current and leeway, cross-track
//! error, and events on waypoint arrival or excessive XTE.
//!
//! [`guide`] is a pure function: snapshot in, [`GuidanceView`] and
//! [`EventList`] out, no state between calls. The caller owns the active leg
//! and advances it on [`GuidanceEvent::WaypointReached`] — or, with turns
//! planned via [`TurnParameters`], on the earlier
//! [`GuidanceEvent::WheelOverReached`]. Thresholds in [`GuidanceConfig`] are
//! vessel settings.
//!
//! ```rust
//! use kinavis::guidance::{guide, GuidanceConfig};
//! use kinavis::route::{LegKind, Route};
//! use kinavis::{
//!     Civil, Current, Distance, EnvironmentSample, GeodeticPoint, Height, Instant,
//!     GuidanceEvent, NavigationSnapshot, ObservationStatus, Observed, Position,
//!     PositionSource, Quality, Speed, TrueCourse, Utc,
//! };
//! use kinavis::snapshot::GroundTrack;
//!
//! let route = Route::new(
//!     &[
//!         "50°00.0'N 001°00.0'W".parse::<Position>()?,
//!         "50°30.0'N 001°00.0'W".parse::<Position>()?,
//!         "50°30.0'N 000°00.0'W".parse::<Position>()?,
//!     ],
//!     LegKind::RhumbLine,
//! )?;
//!
//! // A little east of the first leg, making good ten knots northward.
//! let here: Position = "50°10.0'N 000°59.0'W".parse()?;
//! let noon = Instant::<Utc>::from_civil(Civil::date(2026, 9, 17))?;
//! let snapshot = NavigationSnapshot::EMPTY
//!     .with_position(
//!         Observed::new(here, noon, Quality::<Distance>::new(ObservationStatus::Valid)),
//!         PositionSource::Gnss,
//!     )
//!     .with_ground_track(GroundTrack {
//!         course_over_ground: TrueCourse::new(0.0)?,
//!         speed_over_ground: Speed::from_knots(10.0)?,
//!     });
//!
//! // Two knots of current setting east.
//! let environment = EnvironmentSample::at(
//!     GeodeticPoint::new(here, Height::above_ellipsoid(Distance::ZERO)),
//!     noon,
//! )
//! .with_current(Current {
//!     set: TrueCourse::new(90.0)?,
//!     drift: Speed::from_knots(2.0)?,
//! });
//!
//! let config = GuidanceConfig::new(Distance::from_cables(5.0)?, Distance::from_cables(2.0)?)?;
//!
//! let (view, events) = guide(&snapshot, &route, route.first_leg(), &environment, &config)?;
//! assert_eq!(view.active_leg().index(), 0);
//! assert_eq!(format!("{}", view.desired_track()), "000.0°T");
//! // She must point into the current to make the track good.
//! assert!(view.course_to_steer().degrees() > 345.0 && view.course_to_steer().degrees() < 350.0);
//! // Six tenths of a mile to starboard of the track: over the limit.
//! assert!(view.cross_track_error().distance.nautical_miles() > 0.5);
//! assert!(matches!(events[0], GuidanceEvent::CrossTrackExceeded { .. }));
//! # Ok::<(), kinavis::NavigationError>(())
//! ```

use crate::angle::{Direction, True, TrueCourse};
use crate::environment::EnvironmentSample;
use crate::error::{ensure_range, KernelError, NavigationError, Result};
use crate::event::{EventList, GuidanceEvent};
use crate::math;
use crate::navigation_solutions::{course_to_steer, SteeringSolution};
use crate::position::Position;
use crate::route::{LegCursor, LegKind, Route, RouteLeg};
use crate::sailings::{
    cross_track, great_circle, great_circle_destination, rhumb_line, CrossTrack, Sailing,
};
use crate::snapshot::NavigationSnapshot;
use crate::turning::{wheel_over_point, Turn, TurnMode, TurnParameters};
use crate::units::{Angle, Distance, Speed};

/// Guidance thresholds and leeway.
///
/// XTE limit and arrival radius are vessel settings (a coaster in a buoyed
/// channel and a tanker offshore differ). Validated once by
/// [`GuidanceConfig::new`].
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(try_from = "StoredGuidanceConfig", into = "StoredGuidanceConfig")
)]
pub struct GuidanceConfig {
    cross_track_limit: Distance,
    arrival_radius: Distance,
    leeway: Angle,
    turn: Option<TurnParameters>,
}

impl GuidanceConfig {
    /// Configuration without leeway or turn anticipation.
    ///
    /// `cross_track_limit`: XTE beyond which
    /// [`GuidanceEvent::CrossTrackExceeded`] is reported. `arrival_radius`:
    /// distance counting as waypoint arrival. Passing abeam also counts, so a
    /// vessel missing the circle does not keep steering for a waypoint astern.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] if either distance is negative.
    pub fn new(cross_track_limit: Distance, arrival_radius: Distance) -> Result<Self> {
        ensure_range(
            "cross-track limit",
            cross_track_limit.nautical_miles(),
            0.0,
            f64::MAX,
        )?;
        ensure_range(
            "arrival radius",
            arrival_radius.nautical_miles(),
            0.0,
            f64::MAX,
        )?;
        Ok(Self {
            cross_track_limit,
            arrival_radius,
            leeway: Angle::ZERO,
            turn: None,
        })
    }

    /// Sets the leeway angle, positive to starboard, as given by the vessel's
    /// [`LeewayModel`] for current motion and wind. Stored here because it is
    /// the hull's response to the environment.
    ///
    /// [`LeewayModel`]: crate::environment::LeewayModel
    #[must_use]
    pub const fn with_leeway(mut self, leeway: Angle) -> Self {
        self.leeway = leeway;
        self
    }

    /// Enables turn anticipation: the view includes the turn at the end of the
    /// active leg, [`GuidanceEvent::WheelOverReached`] is reported, and the XTE
    /// limit is suspended inside a planned turn (the track is the arc). Without
    /// it the vessel is steered leg to leg through the waypoints.
    #[must_use]
    pub const fn anticipating_turns(mut self, turn: TurnParameters) -> Self {
        self.turn = Some(turn);
        self
    }

    /// XTE limit for [`GuidanceEvent::CrossTrackExceeded`].
    #[must_use]
    pub const fn cross_track_limit(&self) -> Distance {
        self.cross_track_limit
    }

    /// Arrival radius.
    #[must_use]
    pub const fn arrival_radius(&self) -> Distance {
        self.arrival_radius
    }

    /// Leeway angle, positive to starboard; [`Angle::ZERO`] when not modelled.
    #[must_use]
    pub const fn leeway(&self) -> Angle {
        self.leeway
    }

    /// Turn parameters, if turns are anticipated; `None` for leg-to-leg
    /// steering.
    #[must_use]
    pub const fn turn(&self) -> Option<TurnParameters> {
        self.turn
    }
}

/// Serialised form; deserialisation goes through [`GuidanceConfig::new`].
#[cfg(feature = "serde")]
#[derive(serde::Serialize, serde::Deserialize)]
struct StoredGuidanceConfig {
    cross_track_limit: Distance,
    arrival_radius: Distance,
    leeway: Angle,
    turn: Option<TurnParameters>,
}

#[cfg(feature = "serde")]
impl TryFrom<StoredGuidanceConfig> for GuidanceConfig {
    type Error = NavigationError;

    fn try_from(stored: StoredGuidanceConfig) -> Result<Self> {
        let config =
            Self::new(stored.cross_track_limit, stored.arrival_radius)?.with_leeway(stored.leeway);
        Ok(match stored.turn {
            Some(turn) => config.anticipating_turns(turn),
            None => config,
        })
    }
}

#[cfg(feature = "serde")]
impl From<GuidanceConfig> for StoredGuidanceConfig {
    fn from(config: GuidanceConfig) -> Self {
        Self {
            cross_track_limit: config.cross_track_limit,
            arrival_radius: config.arrival_radius,
            leeway: config.leeway,
            turn: config.turn,
        }
    }
}

/// Steering solution and position relative to the route.
///
/// Projection built by [`guide`]: cheap to copy, safe to log.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct GuidanceView {
    active_leg: LegCursor,
    next_waypoint: Position,
    desired_track: TrueCourse,
    course_to_steer: TrueCourse,
    steering: Option<SteeringSolution>,
    cross_track: CrossTrack,
    bearing_to_waypoint: TrueCourse,
    distance_to_waypoint: Distance,
    distance_to_end: Distance,
    next_turn: Option<Turn>,
    in_turn: bool,
}

impl GuidanceView {
    /// Active leg, as supplied.
    #[must_use]
    pub const fn active_leg(&self) -> LegCursor {
        self.active_leg
    }

    /// Waypoint at the end of the active leg.
    #[must_use]
    pub const fn next_waypoint(&self) -> Position {
        self.next_waypoint
    }

    /// Desired track over ground: active-leg course abeam of the vessel.
    /// Constant on a rhumb leg; varies along a great-circle leg.
    #[must_use]
    pub const fn desired_track(&self) -> TrueCourse {
        self.desired_track
    }

    /// Course to steer to make good the desired track.
    ///
    /// Corrected for current when the sample has one and the snapshot has a
    /// ground track (to recover speed through water), and for
    /// [`GuidanceConfig::leeway`]. If the current cannot be applied (unknown,
    /// no water speed, too strong), this is desired track minus leeway and
    /// [`GuidanceView::steering`] is `None`.
    #[must_use]
    pub const fn course_to_steer(&self) -> TrueCourse {
        self.course_to_steer
    }

    /// Current triangle behind [`GuidanceView::course_to_steer`], if solved:
    /// water track (before leeway), speed made good along track, and drift
    /// angle.
    #[must_use]
    pub const fn steering(&self) -> Option<SteeringSolution> {
        self.steering
    }

    /// Cross-track error, side and along-track distance for the active leg.
    #[must_use]
    pub const fn cross_track_error(&self) -> CrossTrack {
        self.cross_track
    }

    /// Along-track distance from the leg start; negative before it, above the
    /// leg length past its end.
    #[must_use]
    pub const fn along_track_distance(&self) -> Distance {
        self.cross_track.along_track
    }

    /// Direct bearing to the next waypoint.
    #[must_use]
    pub const fn bearing_to_waypoint(&self) -> TrueCourse {
        self.bearing_to_waypoint
    }

    /// Direct distance to the next waypoint.
    #[must_use]
    pub const fn distance_to_waypoint(&self) -> Distance {
        self.distance_to_waypoint
    }

    /// Distance to go: direct to the next waypoint, then along the remaining
    /// legs.
    #[must_use]
    pub const fn distance_to_end(&self) -> Distance {
        self.distance_to_end
    }

    /// Turn at the end of the active leg, if turns are planned; `None` on the
    /// last leg or without [`GuidanceConfig::turn`].
    #[must_use]
    pub const fn next_turn(&self) -> Option<Turn> {
        self.next_turn
    }

    /// Distance to the next wheel-over point; negative once past. `None`
    /// without a next turn.
    #[must_use]
    pub fn distance_to_wheel_over(&self) -> Option<Distance> {
        self.next_turn
            .map(|turn| self.cross_track.to_run - turn.lead())
    }

    /// Whether inside a planned turn (past the wheel-over at the end of the
    /// active leg, or before the end of the turn onto it). XTE is then measured
    /// against a leg the vessel is not meant to follow and is not reported as
    /// exceeded.
    #[must_use]
    pub const fn in_turn(&self) -> bool {
        self.in_turn
    }
}

/// Guidance along `route` for a vessel at `state` on leg `active`.
///
/// Pure: nothing is remembered, so events describe this evaluation, not
/// transitions. [`GuidanceEvent::WaypointReached`] — or, with turns, the
/// earlier [`GuidanceEvent::WheelOverReached`] — is the cue to advance `active`
/// with [`LegCursor::advance`]; until then every evaluation past the point
/// reports it again.
///
/// Rate-of-turn turns take their radius from SOG in the snapshot; radius turns
/// use speed only to derive the rate, and assume stopped without a ground
/// track.
///
/// # Errors
///
/// - [`KernelError::Missing`] if the snapshot has no position.
/// - [`KernelError::Indeterminate`] for a zero-length active leg, or a
///   rate-of-turn turn without a ground track.
/// - [`KernelError::OutOfRange`] if `active` belongs to a longer route.
/// - Any [`wheel_over_point`] error for the turns at either end of the active
///   leg: an unexecutable turn plan is reported, not steered.
/// - Sailing failures, notably a rhumb leg through a pole.
pub fn guide(
    state: &NavigationSnapshot,
    route: &Route,
    active: LegCursor,
    env: &EnvironmentSample,
    config: &GuidanceConfig,
) -> Result<(GuidanceView, EventList<GuidanceEvent>)> {
    let observed = state
        .position()
        .ok_or(NavigationError::Kernel(KernelError::Missing {
            what: "the vessel's position",
        }))?;
    let position = *observed.value();
    let at = observed.taken_at();

    let RouteLeg {
        index,
        from,
        to,
        sailing: leg,
    } = route.leg_at(active)?;
    let offset = cross_track(position, from, to)?;
    let desired_track = track_abeam(route.kind(), from, &leg, offset.along_track)?;
    let to_next = sail(route.kind(), position, to)?;

    // Remaining distance: direct to the next waypoint, then leg by leg.
    let mut remaining = to_next.distance.nautical_miles();
    for later in route.legs().skip(index.saturating_add(1)) {
        remaining += later?.sailing.distance.nautical_miles();
    }

    let steering = steer_against_current(desired_track, state, env);
    let through_water = steering.map_or(desired_track, |solution| solution.heading);
    let course_to_steer =
        Direction::<True>::from_degrees_wrapped(through_water.degrees() - config.leeway.degrees());

    let (next_turn, in_turn) =
        turns_of(route, active, state, config)?.map_or((None, false), |(behind, ahead)| {
            let still_turning_on = behind.is_some_and(|turn| {
                offset.along_track.nautical_miles() < turn.tangent().nautical_miles()
            });
            let wheel_over_passed = ahead
                .is_some_and(|turn| offset.to_run.nautical_miles() <= turn.lead().nautical_miles());
            (ahead, still_turning_on || wheel_over_passed)
        });

    let mut events = EventList::new();
    if !in_turn && offset.distance.nautical_miles() > config.cross_track_limit.nautical_miles() {
        events.push(GuidanceEvent::CrossTrackExceeded {
            error: offset.distance,
            limit: config.cross_track_limit,
            at,
        });
    }
    if let Some(turn) = next_turn {
        if offset.to_run.nautical_miles() <= turn.lead().nautical_miles() {
            events.push(GuidanceEvent::WheelOverReached {
                index: turn.waypoint_index(),
                wheel_over_at: turn.wheel_over(),
                at,
            });
        }
    }
    let inside_circle = to_next.distance.nautical_miles() <= config.arrival_radius.nautical_miles();
    let abeam_or_past = offset.to_run.nautical_miles() <= 0.0;
    if inside_circle || abeam_or_past {
        events.push(GuidanceEvent::WaypointReached {
            index: active.index().saturating_add(1),
            at,
        });
    }

    Ok((
        GuidanceView {
            active_leg: active,
            next_waypoint: to,
            desired_track,
            course_to_steer,
            steering,
            cross_track: offset,
            bearing_to_waypoint: to_next.initial_course,
            distance_to_waypoint: to_next.distance,
            distance_to_end: Distance::from_nautical_miles_unchecked(remaining),
            next_turn,
            in_turn,
        },
        events,
    ))
}

/// Turns at both ends of the active leg, if planned: the one leaving the start
/// (none on the first leg) and the one at the end (none on the last).
fn turns_of(
    route: &Route,
    active: LegCursor,
    state: &NavigationSnapshot,
    config: &GuidanceConfig,
) -> Result<Option<(Option<Turn>, Option<Turn>)>> {
    let Some(parameters) = config.turn.as_ref() else {
        return Ok(None);
    };
    let speed = match (state.ground_track(), parameters.mode()) {
        (Some(ground), _) => ground.speed_over_ground,
        (None, TurnMode::Radius(_)) => Speed::ZERO,
        (None, _) => {
            return Err(NavigationError::Kernel(KernelError::Indeterminate {
                quantity: "the speed for a turn by rate of turn",
            }))
        }
    };
    // Leg `i` runs from waypoint `i` to `i + 1`.
    let index = active.index();
    let behind = match index.checked_sub(1) {
        Some(_) => Some(wheel_over_point(route, index, parameters, speed)?),
        None => None,
    };
    let ahead = if active.is_last(route) {
        None
    } else {
        Some(wheel_over_point(
            route,
            index.saturating_add(1),
            parameters,
            speed,
        )?)
    };
    Ok(Some((behind, ahead)))
}

/// Sailing between two points, of the route's leg kind.
fn sail(kind: LegKind, from: Position, to: Position) -> Result<Sailing> {
    match kind {
        LegKind::RhumbLine => rhumb_line(from, to),
        LegKind::GreatCircle => great_circle(from, to),
    }
}

/// Leg course at the foot of the perpendicular from the vessel.
///
/// A rhumb leg has one course. On a great-circle leg, the course made good at
/// `along` from the start, with `along` clamped to the leg so positions before
/// or beyond it get the end course.
fn track_abeam(
    kind: LegKind,
    from: Position,
    leg: &Sailing,
    along: Distance,
) -> Result<TrueCourse> {
    match kind {
        LegKind::RhumbLine => Ok(leg.initial_course),
        LegKind::GreatCircle => {
            // Not `clamp`, which panics on an inverted interval; the leg length
            // is non-negative but the compiler cannot prove it.
            let within = along
                .nautical_miles()
                .max(0.0)
                .min(leg.distance.nautical_miles());
            if within <= 0.0 {
                return Ok(leg.initial_course);
            }
            Ok(great_circle_destination(
                from,
                leg.initial_course,
                Distance::from_nautical_miles_unchecked(within),
            )?
            .final_course)
        }
    }
}

/// Current triangle for `track`, if solvable.
///
/// Speed through water is not in the snapshot; it is recovered as ground
/// velocity minus current. `None` if the current is unknown or zero, the ground
/// track is unknown, the vessel is not moving through the water, or the current
/// is too strong; the caller then steers the track and XTE reflects the rest.
fn steer_against_current(
    track: TrueCourse,
    state: &NavigationSnapshot,
    env: &EnvironmentSample,
) -> Option<SteeringSolution> {
    let current = env.current()?;
    let ground = state.ground_track()?;
    if current.drift.knots() <= 0.0 {
        return None;
    }

    let (ground_north, ground_east) = ground
        .course_over_ground
        .components(ground.speed_over_ground.knots());
    let (set_north, set_east) = current.set.components(current.drift.knots());
    let through_water = math::hypot(ground_north - set_north, ground_east - set_east);
    if math::is_effectively_zero(
        through_water,
        ground.speed_over_ground.knots().max(current.drift.knots()),
    ) {
        return None;
    }

    course_to_steer(
        track,
        Speed::from_knots_unchecked(through_water),
        current.set,
        current.drift,
    )
    .ok()
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp, clippy::indexing_slicing)]
mod tests {
    use super::*;
    use crate::environment::Current;
    use crate::event::PositionSource;
    use crate::geodesy::{GeodeticPoint, Height};
    use crate::observation::{ObservationStatus, Observed, Quality};
    use crate::sailings::TrackSide;
    use crate::snapshot::GroundTrack;
    use crate::time::{Instant, Utc};

    fn at(latitude: f64, longitude: f64) -> Position {
        Position::from_degrees(latitude, longitude).unwrap()
    }

    fn noon() -> Instant<Utc> {
        Instant::from_unix_seconds(1_789_000_000)
    }

    /// 60 NM north, 60 NM east, 60 NM south.
    fn square() -> Route {
        Route::new(
            &[at(0.0, 0.0), at(1.0, 0.0), at(1.0, 1.0), at(0.0, 1.0)],
            LegKind::RhumbLine,
        )
        .unwrap()
    }

    fn snapshot(position: Position, course: f64, knots: f64) -> NavigationSnapshot {
        NavigationSnapshot::EMPTY
            .with_position(
                Observed::new(
                    position,
                    noon(),
                    Quality::<Distance>::new(ObservationStatus::Valid),
                ),
                PositionSource::Gnss,
            )
            .with_ground_track(GroundTrack {
                course_over_ground: TrueCourse::new(course).unwrap(),
                speed_over_ground: Speed::from_knots(knots).unwrap(),
            })
    }

    fn calm(position: Position) -> EnvironmentSample {
        EnvironmentSample::at(
            GeodeticPoint::new(position, Height::above_ellipsoid(Distance::ZERO)),
            noon(),
        )
    }

    fn current(position: Position, set: f64, drift: f64) -> EnvironmentSample {
        calm(position).with_current(Current {
            set: TrueCourse::new(set).unwrap(),
            drift: Speed::from_knots(drift).unwrap(),
        })
    }

    fn on_leg(index: u16) -> LegCursor {
        LegCursor::try_from(index).unwrap()
    }

    fn config() -> GuidanceConfig {
        GuidanceConfig::new(
            Distance::from_nautical_miles(0.5).unwrap(),
            Distance::from_nautical_miles(0.2).unwrap(),
        )
        .unwrap()
    }

    fn turning() -> GuidanceConfig {
        config().anticipating_turns(
            TurnParameters::new(
                TurnMode::Radius(Distance::from_nautical_miles(1.0).unwrap()),
                Distance::from_cables(9.0).unwrap(),
                Distance::from_cables(7.0).unwrap(),
            )
            .unwrap(),
        )
    }

    #[test]
    fn on_the_track_in_slack_water_the_course_to_steer_is_the_track() {
        let here = at(0.5, 0.0);
        let (view, events) = guide(
            &snapshot(here, 0.0, 10.0),
            &square(),
            on_leg(0),
            &calm(here),
            &config(),
        )
        .unwrap();

        assert_eq!(view.active_leg().index(), 0);
        assert_eq!(view.next_waypoint(), at(1.0, 0.0));
        assert!(view.desired_track().degrees().abs() < 1e-9);
        assert!(view.course_to_steer().degrees().abs() < 1e-9);
        assert!(view.steering().is_none());
        assert_eq!(view.cross_track_error().side, TrackSide::OnTrack);
        assert!((view.along_track_distance().nautical_miles() - 30.0).abs() < 0.1);
        assert!((view.distance_to_waypoint().nautical_miles() - 30.0).abs() < 0.1);
        assert!(view.bearing_to_waypoint().degrees().abs() < 1e-6);
        assert!((view.distance_to_end().nautical_miles() - 150.0).abs() < 0.2);
        assert!(events.is_empty());
        assert!(!events.overflowed());
    }

    #[test]
    fn a_current_from_abeam_is_met_by_pointing_into_it() {
        let here = at(0.5, 0.0);
        // 10 kn made good north with 2 kn setting east: 10 kn through the
        // water, heading slightly west of north.
        let (view, _) = guide(
            &snapshot(here, 0.0, 10.0),
            &square(),
            on_leg(0),
            &current(here, 90.0, 2.0),
            &config(),
        )
        .unwrap();

        // The triangle is solved for the desired track, not the actual one:
        // water speed is |ground − set|.
        let steering = view.steering().unwrap();
        let through_water = math::hypot(10.0, 2.0);
        let expected = course_to_steer(
            TrueCourse::NORTH,
            Speed::from_knots(through_water).unwrap(),
            TrueCourse::new(90.0).unwrap(),
            Speed::from_knots(2.0).unwrap(),
        )
        .unwrap();
        assert!((steering.heading.degrees() - expected.heading.degrees()).abs() < 1e-9);
        assert!(view.course_to_steer().degrees() > 345.0);
        assert!(view.course_to_steer().degrees() < 360.0);
        assert!(steering.speed_over_ground.knots() < through_water);
    }

    #[test]
    fn leeway_to_starboard_means_steering_to_port_of_the_water_track() {
        let here = at(0.5, 0.0);
        let config = config().with_leeway(Angle::from_degrees(5.0).unwrap());
        let (view, _) = guide(
            &snapshot(here, 0.0, 10.0),
            &square(),
            on_leg(0),
            &calm(here),
            &config,
        )
        .unwrap();
        assert!((view.course_to_steer().degrees() - 355.0).abs() < 1e-9);
        assert!(view.desired_track().degrees().abs() < 1e-9);
    }

    #[test]
    fn a_current_she_cannot_stem_leaves_her_steering_the_track() {
        let here = at(0.5, 0.0);
        // 4 kn over ground with a 12 kn easterly set: about 8 kn through the
        // water, insufficient to hold the track across a 12 kn set.
        let (view, _) = guide(
            &snapshot(here, 90.0, 4.0),
            &square(),
            on_leg(0),
            &current(here, 90.0, 12.0),
            &config(),
        )
        .unwrap();
        assert!(view.steering().is_none());
        assert!(view.course_to_steer().degrees().abs() < 1e-9);
    }

    #[test]
    fn without_a_ground_track_the_current_is_not_applied() {
        let here = at(0.5, 0.0);
        let state = NavigationSnapshot::EMPTY.with_position(
            Observed::new(
                here,
                noon(),
                Quality::<Distance>::new(ObservationStatus::Valid),
            ),
            PositionSource::DeadReckoning,
        );
        let (view, _) = guide(
            &state,
            &square(),
            on_leg(0),
            &current(here, 90.0, 2.0),
            &config(),
        )
        .unwrap();
        assert!(view.steering().is_none());
        assert!(view.course_to_steer().degrees().abs() < 1e-9);
    }

    #[test]
    fn off_the_track_beyond_the_limit_is_reported_with_the_numbers() {
        // 3 NM east of a northbound leg: starboard.
        let here = at(0.5, 0.05);
        let (view, events) = guide(
            &snapshot(here, 0.0, 10.0),
            &square(),
            on_leg(0),
            &calm(here),
            &config(),
        )
        .unwrap();
        assert_eq!(view.cross_track_error().side, TrackSide::Starboard);
        assert_eq!(events.len(), 1);
        let GuidanceEvent::CrossTrackExceeded {
            error,
            limit,
            at: when,
        } = events[0]
        else {
            unreachable!("expected CrossTrackExceeded, got {:?}", events[0]);
        };
        assert!((error.nautical_miles() - 3.0).abs() < 0.05);
        assert_eq!(limit, config().cross_track_limit());
        assert_eq!(when, noon());

        // Within the limit: no event.
        let near = at(0.5, 0.005);
        let (_, quiet) = guide(
            &snapshot(near, 0.0, 10.0),
            &square(),
            on_leg(0),
            &calm(near),
            &config(),
        )
        .unwrap();
        assert!(quiet.is_empty());
    }

    #[test]
    fn inside_the_arrival_circle_the_waypoint_is_reached() {
        // 0.1 NM short of the first waypoint.
        let here = at(1.0 - 0.1 / 60.0, 0.0);
        let (view, events) = guide(
            &snapshot(here, 0.0, 10.0),
            &square(),
            on_leg(0),
            &calm(here),
            &config(),
        )
        .unwrap();
        assert!(view.distance_to_waypoint().nautical_miles() < 0.2);
        assert_eq!(events.len(), 1);
        assert!(matches!(
            events[0],
            GuidanceEvent::WaypointReached { index: 1, .. }
        ));
    }

    #[test]
    fn abeam_of_the_waypoint_counts_as_reaching_it_whatever_the_radius() {
        // 2 NM east of the first waypoint and just past it: outside the circle,
        // but the waypoint is astern.
        let here = at(1.001, 2.0 / 60.0);
        let (view, events) = guide(
            &snapshot(here, 0.0, 10.0),
            &square(),
            on_leg(0),
            &calm(here),
            &config(),
        )
        .unwrap();
        assert!(view.distance_to_waypoint().nautical_miles() > 1.0);
        assert!(view.cross_track_error().to_run.nautical_miles() <= 0.0);
        assert!(events
            .iter()
            .any(|event| matches!(event, GuidanceEvent::WaypointReached { index: 1, .. })));
        // 2 NM off: over the limit too.
        assert!(events
            .iter()
            .any(|event| matches!(event, GuidanceEvent::CrossTrackExceeded { .. })));
    }

    #[test]
    fn the_last_waypoint_reached_names_the_end_of_the_route() {
        let here = at(0.05, 1.0);
        let (view, events) = guide(
            &snapshot(here, 180.0, 10.0),
            &square(),
            on_leg(2),
            &calm(here),
            &config(),
        )
        .unwrap();
        assert_eq!(view.active_leg().index(), 2);
        assert!((view.distance_to_end().nautical_miles() - 3.0).abs() < 0.05);
        assert!(events.is_empty());

        let there = at(0.0, 1.0);
        let (_, events) = guide(
            &snapshot(there, 180.0, 10.0),
            &square(),
            on_leg(2),
            &calm(there),
            &config(),
        )
        .unwrap();
        assert!(matches!(
            events[0],
            GuidanceEvent::WaypointReached { index: 3, .. }
        ));
    }

    #[test]
    fn a_great_circle_leg_turns_as_she_goes() {
        // Lizard to Cape Race: the great circle bows north of the rhumb line.
        let route =
            Route::new(&[at(49.95, -5.2), at(46.66, -53.07)], LegKind::GreatCircle).unwrap();
        let leg = great_circle(at(49.95, -5.2), at(46.66, -53.07)).unwrap();

        let start = at(49.95, -5.2);
        let (near_start, _) = guide(
            &snapshot(start, 270.0, 15.0),
            &route,
            on_leg(0),
            &calm(start),
            &config(),
        )
        .unwrap();
        assert!((near_start.desired_track().degrees() - leg.initial_course.degrees()).abs() < 1e-6);

        // Halfway, the course made good has swung south of west.
        let halfway =
            crate::sailings::great_circle_intermediate(start, at(46.66, -53.07), 0.5).unwrap();
        let (mid, events) = guide(
            &snapshot(halfway, 270.0, 15.0),
            &route,
            on_leg(0),
            &calm(halfway),
            &config(),
        )
        .unwrap();
        assert!(mid.desired_track().degrees() < leg.initial_course.degrees());
        assert!(mid.desired_track().degrees() > leg.final_course.degrees());
        assert_eq!(mid.cross_track_error().side, TrackSide::OnTrack);
        assert!(events.is_empty());
    }

    #[test]
    fn before_the_start_of_a_leg_she_is_given_its_first_course() {
        let here = at(-0.5, 0.0);
        let (view, _) = guide(
            &snapshot(here, 0.0, 10.0),
            &square(),
            on_leg(0),
            &calm(here),
            &config(),
        )
        .unwrap();
        assert!(view.along_track_distance().is_negative());
        assert!(view.desired_track().degrees().abs() < 1e-9);
        assert!((view.distance_to_waypoint().nautical_miles() - 90.0).abs() < 0.2);
    }

    #[test]
    fn a_snapshot_without_a_position_has_nothing_to_guide() {
        let here = at(0.5, 0.0);
        assert!(matches!(
            guide(
                &NavigationSnapshot::EMPTY,
                &square(),
                on_leg(0),
                &calm(here),
                &config()
            )
            .unwrap_err(),
            NavigationError::Kernel(KernelError::Missing { .. })
        ));
    }

    #[test]
    fn a_leg_the_route_does_not_have_is_refused() {
        // A route does not issue a cursor for a leg it lacks...
        let square = square();
        assert!(matches!(
            square.leg(3).unwrap_err(),
            NavigationError::Kernel(KernelError::OutOfRange {
                parameter: "active leg",
                max,
                ..
            }) if max == 2.0
        ));
        assert!(square.leg(u16::MAX).is_err());
        assert!(LegCursor::try_from(u16::MAX).is_err());
        assert!(LegCursor::try_from(126).is_ok());
        assert!(LegCursor::try_from(127).is_err());

        // ...and rejects one issued by a longer route.
        let here = at(0.5, 0.0);
        let error = guide(
            &snapshot(here, 0.0, 10.0),
            &square,
            on_leg(3),
            &calm(here),
            &config(),
        )
        .unwrap_err();
        assert!(matches!(
            error,
            NavigationError::Kernel(KernelError::OutOfRange {
                parameter: "active leg",
                ..
            })
        ));
    }

    #[test]
    fn the_cursor_walks_the_route_and_stops_at_its_end() {
        let square = square();
        let first = square.first_leg();
        assert_eq!(first.index(), 0);
        assert!(!first.is_last(&square));
        let second = first.advance(&square).unwrap();
        let third = second.advance(&square).unwrap();
        assert_eq!(third.index(), 2);
        assert!(third.is_last(&square));
        assert!(third.advance(&square).is_none());
        assert_eq!(square.leg(1).unwrap(), second);
    }

    #[test]
    fn a_negative_threshold_is_refused() {
        let minus_one = Distance::from_nautical_miles(-1.0).unwrap();
        let one = Distance::from_nautical_miles(1.0).unwrap();
        assert!(matches!(
            GuidanceConfig::new(minus_one, one).unwrap_err(),
            NavigationError::Kernel(KernelError::OutOfRange {
                parameter: "cross-track limit",
                ..
            })
        ));
        assert!(matches!(
            GuidanceConfig::new(one, minus_one).unwrap_err(),
            NavigationError::Kernel(KernelError::OutOfRange {
                parameter: "arrival radius",
                ..
            })
        ));
        assert!(GuidanceConfig::new(Distance::ZERO, Distance::ZERO).is_ok());
    }

    #[test]
    fn a_zero_length_leg_is_reported_not_divided_by_zero() {
        let route = Route::new(
            &[at(10.0, 10.0), at(10.0, 10.0), at(11.0, 10.0)],
            LegKind::RhumbLine,
        )
        .unwrap();
        let here = at(10.5, 10.0);
        assert!(guide(
            &snapshot(here, 0.0, 10.0),
            &route,
            on_leg(0),
            &calm(here),
            &config()
        )
        .is_err());
        assert!(guide(
            &snapshot(here, 0.0, 10.0),
            &route,
            on_leg(1),
            &calm(here),
            &config()
        )
        .is_ok());
    }

    #[test]
    fn with_turns_planned_the_wheel_over_comes_before_the_waypoint() {
        // 58 NM along a 60 NM leg: 2 NM short of the waypoint, 0.8 NM short of
        // the wheel-over (1.2 NM lead).
        let here = at(58.0 / 60.0, 0.0);
        let (view, events) = guide(
            &snapshot(here, 0.0, 10.0),
            &square(),
            on_leg(0),
            &calm(here),
            &turning(),
        )
        .unwrap();
        let turn = view.next_turn().unwrap();
        assert_eq!(turn.waypoint_index(), 1);
        assert!((turn.lead().nautical_miles() - 1.2).abs() < 1e-9);
        assert!((view.distance_to_wheel_over().unwrap().nautical_miles() - 0.8).abs() < 0.01);
        assert!(!view.in_turn());
        assert!(events.is_empty());

        // 1 NM short: past the wheel-over, in the turn.
        let here = at(59.0 / 60.0, 0.0);
        let (view, events) = guide(
            &snapshot(here, 0.0, 10.0),
            &square(),
            on_leg(0),
            &calm(here),
            &turning(),
        )
        .unwrap();
        assert!(view.in_turn());
        assert!(view.distance_to_wheel_over().unwrap().is_negative());
        assert_eq!(events.len(), 1);
        let GuidanceEvent::WheelOverReached {
            index,
            wheel_over_at,
            at: when,
        } = events[0]
        else {
            unreachable!("expected WheelOverReached, got {:?}", events[0]);
        };
        assert_eq!(index, 1);
        assert_eq!(wheel_over_at, turn.wheel_over());
        assert_eq!(when, noon());
    }

    #[test]
    fn in_a_turn_the_cross_track_error_is_not_an_alarm() {
        // 0.5 NM short of the waypoint, 0.5 NM east of the leg: on the turn
        // arc, off the straight leg.
        let here = at(59.5 / 60.0, 0.5 / 60.0);
        let (view, events) = guide(
            &snapshot(here, 45.0, 10.0),
            &square(),
            on_leg(0),
            &calm(here),
            &turning(),
        )
        .unwrap();
        assert!(view.in_turn());
        assert!(view.cross_track_error().distance.nautical_miles() > 0.4);
        assert!(!events
            .iter()
            .any(|event| matches!(event, GuidanceEvent::CrossTrackExceeded { .. })));

        // Without a planned turn the same position is simply off track.
        let (view, events) = guide(
            &snapshot(here, 45.0, 10.0),
            &square(),
            on_leg(0),
            &calm(here),
            &config(),
        )
        .unwrap();
        assert!(!view.in_turn());
        assert!(view.next_turn().is_none());
        assert!(view.distance_to_wheel_over().is_none());
        assert!(events
            .iter()
            .any(|event| matches!(event, GuidanceEvent::CrossTrackExceeded { .. })));
    }

    #[test]
    fn the_new_leg_is_still_a_turn_until_the_arc_joins_it() {
        // Leg advanced at the wheel-over: 1 NM short of the second leg's start,
        // well to port of it, turning onto it.
        let here = at(59.0 / 60.0, 0.0);
        let (view, events) = guide(
            &snapshot(here, 0.0, 10.0),
            &square(),
            on_leg(1),
            &calm(here),
            &turning(),
        )
        .unwrap();
        assert_eq!(view.active_leg().index(), 1);
        assert!(view.along_track_distance().is_negative());
        assert!(view.in_turn());
        assert!(!events
            .iter()
            .any(|event| matches!(event, GuidanceEvent::CrossTrackExceeded { .. })));

        // 2 NM along the second leg: out of the turn, on track.
        let here = at(1.0, 2.0 / 60.0);
        let (view, _) = guide(
            &snapshot(here, 90.0, 10.0),
            &square(),
            on_leg(1),
            &calm(here),
            &turning(),
        )
        .unwrap();
        assert!(!view.in_turn());
        assert_eq!(view.next_turn().unwrap().waypoint_index(), 2);
    }

    #[test]
    fn the_last_leg_has_no_turn_at_its_end() {
        let here = at(0.5, 1.0);
        let (view, _) = guide(
            &snapshot(here, 180.0, 10.0),
            &square(),
            on_leg(2),
            &calm(here),
            &turning(),
        )
        .unwrap();
        assert!(view.next_turn().is_none());
        assert!(view.distance_to_wheel_over().is_none());
        assert!(!view.in_turn());
    }

    #[test]
    fn a_turn_by_rate_needs_a_ground_track_and_a_turn_by_radius_does_not() {
        let here = at(0.5, 0.0);
        let state = NavigationSnapshot::EMPTY.with_position(
            Observed::new(
                here,
                noon(),
                Quality::<Distance>::new(ObservationStatus::Valid),
            ),
            PositionSource::DeadReckoning,
        );
        let by_radius = turning();
        let (view, _) = guide(&state, &square(), on_leg(0), &calm(here), &by_radius).unwrap();
        assert_eq!(
            view.next_turn().unwrap().rate_of_turn(),
            crate::units::RateOfTurn::ZERO
        );

        let by_rate = config().anticipating_turns(
            TurnParameters::new(
                TurnMode::RateOfTurn(
                    crate::units::RateOfTurn::from_degrees_per_minute(10.0).unwrap(),
                ),
                Distance::ZERO,
                Distance::ZERO,
            )
            .unwrap(),
        );
        assert!(matches!(
            guide(&state, &square(), on_leg(0), &calm(here), &by_rate).unwrap_err(),
            NavigationError::Kernel(KernelError::Indeterminate { .. })
        ));
        let (view, _) = guide(
            &snapshot(here, 0.0, 10.0),
            &square(),
            on_leg(0),
            &calm(here),
            &by_rate,
        )
        .unwrap();
        assert!(
            (view
                .next_turn()
                .unwrap()
                .rate_of_turn()
                .degrees_per_minute()
                - 10.0)
                .abs()
                < 1e-9
        );
    }

    #[test]
    fn a_route_that_cannot_be_turned_as_planned_is_refused() {
        let here = at(0.5, 0.0);
        let config = config().anticipating_turns(
            TurnParameters::new(
                TurnMode::Radius(Distance::from_nautical_miles(100.0).unwrap()),
                Distance::ZERO,
                Distance::ZERO,
            )
            .unwrap(),
        );
        assert!(matches!(
            guide(
                &snapshot(here, 0.0, 10.0),
                &square(),
                on_leg(0),
                &calm(here),
                &config
            )
            .unwrap_err(),
            NavigationError::Kernel(KernelError::OutOfRange {
                parameter: "turn lead",
                ..
            })
        ));
    }
}
