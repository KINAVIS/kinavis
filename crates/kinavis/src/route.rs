//! Passage planning: waypoint chains and progress along them.
//!
//! A [`Route`] is an ordered list of positions and the leg type between them.
//! It answers distance, time, and — underway — progress.
//!
//! # Example
//!
//! ```rust
//! use kinavis::route::{LegKind, Route};
//! use kinavis::{NavigationError, Position, Speed};
//!
//! fn main() -> Result<(), NavigationError> {
//!     let route = Route::new(
//!         &[
//!             "50°06.0'N 001°30.0'W".parse::<Position>()?,
//!             "49°54.0'N 002°00.0'W".parse::<Position>()?,
//!             "49°42.0'N 002°45.0'W".parse::<Position>()?,
//!         ],
//!         LegKind::RhumbLine,
//!     )?;
//!
//!     assert_eq!(route.legs().count(), 2);
//!     assert_eq!(format!("{:.1}", route.total_distance()?.nautical_miles()), "54.2");
//!
//!     // At ten knots, how long is the passage?
//!     let elapsed = route.passage_time(Speed::from_knots(10.0)?)?;
//!     assert_eq!(elapsed.as_secs() / 60, 325);
//!     Ok(())
//! }
//! ```

use core::time::Duration;

use crate::angle::TrueCourse;
use crate::error::{KernelError, NavigationError, Result};
use crate::inline::Inline;
use crate::math;
use crate::position::Position;
use crate::sailings::{
    cross_track, great_circle, great_circle_waypoints_into, pieces_of, rhumb_line, CrossTrack,
    Sailing,
};
use crate::units::{Distance, Speed};

/// Leg type.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[non_exhaustive]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum LegKind {
    /// Constant course between waypoints; what is steered.
    #[default]
    RhumbLine,
    /// Shortest track, course varying along it.
    ///
    /// For measuring passages; split into rhumb legs with [`Route::split_legs`]
    /// to steer.
    GreatCircle,
}

/// Route leg.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct RouteLeg {
    /// Leg index, zero-based.
    pub index: usize,
    /// Start.
    pub from: Position,
    /// End.
    pub to: Position,
    /// Course and distance.
    pub sailing: Sailing,
}

/// Active leg of a route.
///
/// Issued by the route ([`Route::first_leg`], [`Route::leg`]) and moved by
/// [`LegCursor::advance`], so a caller cannot name a nonexistent leg. Taken by
/// [`guide`] and returned by [`GuidanceView::active_leg`]; advanced on the
/// view's cue.
///
/// It stores only an index validated by the route, not a borrow; used with
/// another route it is re-checked and rejected if that route is too short.
///
/// [`guide`]: crate::guidance::guide
/// [`GuidanceView::active_leg`]: crate::guidance::GuidanceView::active_leg
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(try_from = "u16", into = "u16")
)]
pub struct LegCursor {
    index: u16,
}

/// Maximum leg index: [`MAX_WAYPOINTS`] − 2.
const MAX_LEG_INDEX: u16 = 126;
const _: () = assert!(MAX_LEG_INDEX as usize == MAX_WAYPOINTS - 2);

impl LegCursor {
    /// First leg.
    const FIRST: Self = Self { index: 0 };

    /// Leg index, zero-based.
    #[must_use]
    pub const fn index(self) -> u16 {
        self.index
    }

    /// Next leg of `route`; `None` on the last leg.
    #[must_use]
    pub fn advance(self, route: &Route) -> Option<Self> {
        let next = self.index.checked_add(1)?;
        route.leg(next).ok()
    }

    /// Whether this is the last leg of `route`.
    #[must_use]
    pub fn is_last(self, route: &Route) -> bool {
        self.advance(route).is_none()
    }
}

impl TryFrom<u16> for LegCursor {
    type Error = NavigationError;

    /// Cursor from a raw index, checked only against the global maximum;
    /// [`Route::leg`] checks against the route.
    fn try_from(index: u16) -> Result<Self> {
        if index > MAX_LEG_INDEX {
            return Err(NavigationError::Kernel(KernelError::OutOfRange {
                parameter: "leg index",
                value: f64::from(index),
                min: 0.0,
                max: f64::from(MAX_LEG_INDEX),
            }));
        }
        Ok(Self { index })
    }
}

impl From<LegCursor> for u16 {
    fn from(cursor: LegCursor) -> Self {
        cursor.index
    }
}

/// Position relative to a route.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Progress {
    /// Index of the current leg.
    pub leg: usize,
    /// Cross-track and along-track position on that leg.
    pub cross_track: CrossTrack,
    /// Direct course to the next waypoint.
    pub course_to_next: TrueCourse,
    /// Direct distance to the next waypoint.
    pub distance_to_next: Distance,
    /// Distance to go: rest of this leg plus the following legs.
    pub distance_to_end: Distance,
}

/// Maximum waypoints per route.
///
/// Routes are stored inline. [`Route::split_legs`] is bounded by the same
/// limit; an interval that would exceed it is an error.
pub const MAX_WAYPOINTS: usize = 128;

/// Inline waypoint storage.
type Waypoints = Inline<Position, MAX_WAYPOINTS>;

/// Planned passage.
///
/// Up to [`MAX_WAYPOINTS`] waypoints stored inline, so the type is large: pass
/// it by reference.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(try_from = "StoredRoute", into = "StoredRoute")
)]
pub struct Route {
    waypoints: Waypoints,
    kind: LegKind,
}

impl Route {
    /// Route from ordered waypoints.
    ///
    /// # Errors
    ///
    /// - [`KernelError::InsufficientData`] for fewer than two waypoints.
    /// - [`KernelError::CapacityExceeded`] beyond [`MAX_WAYPOINTS`].
    pub fn new(waypoints: &[Position], kind: LegKind) -> Result<Self> {
        let Some(&first) = waypoints.first() else {
            return Err(NavigationError::Kernel(KernelError::InsufficientData {
                found: 0,
                required: 2,
                context: "a route",
            }));
        };
        if waypoints.len() < 2 {
            return Err(NavigationError::Kernel(KernelError::InsufficientData {
                found: waypoints.len(),
                required: 2,
                context: "a route",
            }));
        }
        let mut stored = Waypoints::new(first);
        for &waypoint in waypoints {
            push_waypoint(&mut stored, waypoint, waypoints.len())?;
        }
        Ok(Self {
            waypoints: stored,
            kind,
        })
    }

    /// Waypoints.
    #[must_use]
    pub fn waypoints(&self) -> &[Position] {
        &self.waypoints
    }

    /// Leg type.
    #[must_use]
    pub const fn kind(&self) -> LegKind {
        self.kind
    }

    /// Number of legs (waypoints − 1).
    #[must_use]
    pub fn leg_count(&self) -> usize {
        self.waypoints.len().saturating_sub(1)
    }

    /// Legs with course and distance.
    ///
    /// An iterator, so reading a route does not allocate; collect with
    /// `.collect::<Result<Vec<_>>>()` if needed. Each item may carry a sailing
    /// failure (e.g. rhumb line through a pole).
    pub fn legs(&self) -> impl Iterator<Item = Result<RouteLeg>> + '_ {
        // Bounded range instead of `enumerate`, whose counter the compiler
        // cannot bound by the waypoint count.
        (0..MAX_WAYPOINTS)
            .zip(self.waypoints.windows(2))
            .map(|(index, pair)| {
                let (from, to) = pair_of(pair)?;
                Ok(RouteLeg {
                    index,
                    from,
                    to,
                    sailing: self.sail(from, to)?,
                })
            })
    }

    /// Total distance.
    ///
    /// # Errors
    ///
    /// As [`Route::legs`].
    pub fn total_distance(&self) -> Result<Distance> {
        let mut total = 0.0;
        for pair in self.waypoints.windows(2) {
            let (from, to) = pair_of(pair)?;
            total += self.sail(from, to)?.distance.nautical_miles();
        }
        Ok(Distance::from_nautical_miles_unchecked(total))
    }

    /// Passage time at `speed`.
    ///
    /// # Errors
    ///
    /// As [`Route::legs`]; [`KernelError::Indeterminate`] for zero or
    /// opposite-sign speed.
    pub fn passage_time(&self, speed: Speed) -> Result<Duration> {
        Ok(speed.time_to_cover(self.total_distance()?)?)
    }

    /// Speed required to complete the route in `duration`.
    ///
    /// # Errors
    ///
    /// As [`Route::legs`]; [`KernelError::Indeterminate`] for zero duration.
    pub fn speed_required(&self, available: Duration) -> Result<Speed> {
        crate::dead_reckoning::speed_required(self.total_distance()?, available)
    }

    /// Route with every leg split into pieces no longer than `interval`.
    ///
    /// Makes great-circle routes steerable: each piece is short enough that a
    /// constant course costs nothing significant.
    ///
    /// # Errors
    ///
    /// - [`KernelError::OutOfRange`] if `interval` is not positive or too small
    ///   to fit in [`MAX_WAYPOINTS`].
    /// - Sailing failures.
    pub fn split_legs(&self, interval: Distance) -> Result<Self> {
        let Some(&first) = self.waypoints.first() else {
            return Err(NavigationError::Kernel(KernelError::InsufficientData {
                found: 0,
                required: 2,
                context: "a route",
            }));
        };
        let mut waypoints = Waypoints::new(first);
        let mut piece_buffer = [first; MAX_WAYPOINTS];

        for (index, pair) in (0..MAX_WAYPOINTS).zip(self.waypoints.windows(2)) {
            let (from, to) = pair_of(pair)?;
            let room = MAX_WAYPOINTS
                .saturating_sub(waypoints.len())
                .saturating_add(usize::from(index > 0));
            let pieces = match self.kind {
                LegKind::GreatCircle => {
                    great_circle_waypoints_into(from, to, interval, &mut piece_buffer)?
                }
                // Rhumb legs are split too, for a uniform maximum leg length.
                LegKind::RhumbLine => rhumb_waypoints_into(from, to, interval, &mut piece_buffer)?,
            };
            if pieces > room {
                return Err(NavigationError::Kernel(KernelError::CapacityExceeded {
                    context: "a split route",
                    needed: waypoints.len().saturating_add(pieces),
                    capacity: MAX_WAYPOINTS,
                }));
            }
            // All pieces except the first of each leg, to avoid duplicate
            // joins.
            let skip = usize::from(index > 0);
            for &waypoint in piece_buffer.get(skip..pieces).unwrap_or(&[]) {
                push_waypoint(&mut waypoints, waypoint, MAX_WAYPOINTS)?;
            }
        }
        Self::new(waypoints.as_slice(), LegKind::RhumbLine)
    }

    /// Position relative to the route.
    ///
    /// The chosen leg is the first whose along-track range contains the
    /// position; if none does, the nearest leg is used.
    ///
    /// # Errors
    ///
    /// Sailing failures; [`KernelError::Indeterminate`] for a zero-length leg.
    pub fn progress(&self, position: Position) -> Result<Progress> {
        let mut best: Option<(usize, CrossTrack, bool)> = None;

        for (index, pair) in (0..MAX_WAYPOINTS).zip(self.waypoints.windows(2)) {
            let (from, to) = pair_of(pair)?;
            let offset = cross_track(position, from, to)?;
            let leg_length = self.sail(from, to)?.distance.nautical_miles();
            let within = offset.along_track.nautical_miles() >= 0.0
                && offset.along_track.nautical_miles() <= leg_length;

            let better = match &best {
                None => true,
                Some((_, previous, previous_within)) => {
                    // A leg containing the position wins over one that does
                    // not.
                    match (within, previous_within) {
                        (true, false) => true,
                        (false, true) => false,
                        _ => offset.distance.nautical_miles() < previous.distance.nautical_miles(),
                    }
                }
            };
            if better {
                best = Some((index, offset, within));
            }
        }

        let (leg, offset, _) =
            best.ok_or(NavigationError::Kernel(KernelError::InsufficientData {
                found: self.waypoints.len(),
                required: 2,
                context: "a route",
            }))?;

        let next =
            self.waypoints
                .get(leg.saturating_add(1))
                .copied()
                .ok_or(NavigationError::Kernel(KernelError::Missing {
                    what: "the waypoint the leg ends at",
                }))?;
        let to_next = self.sail(position, next)?;

        // Rest of this leg, then every following leg.
        let mut remaining = to_next.distance.nautical_miles();
        for pair in self.waypoints.windows(2).skip(leg.saturating_add(1)) {
            let (from, to) = pair_of(pair)?;
            remaining += self.sail(from, to)?.distance.nautical_miles();
        }

        Ok(Progress {
            leg,
            cross_track: offset,
            course_to_next: to_next.initial_course,
            distance_to_next: to_next.distance,
            distance_to_end: Distance::from_nautical_miles_unchecked(remaining),
        })
    }

    /// First leg; every route has one.
    #[must_use]
    pub const fn first_leg(&self) -> LegCursor {
        LegCursor::FIRST
    }

    /// Leg at `index`, zero-based.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] if the route has no such leg.
    pub fn leg(&self, index: u16) -> Result<LegCursor> {
        if usize::from(index) >= self.leg_count() {
            return Err(NavigationError::Kernel(KernelError::OutOfRange {
                parameter: "active leg",
                value: f64::from(index),
                min: 0.0,
                max: math::count_to_f64(self.leg_count().saturating_sub(1)),
            }));
        }
        Ok(LegCursor { index })
    }

    /// Leg named by a cursor, with course and distance.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] if this route lacks the leg (cursor from a
    /// longer route); sailing failures, notably a rhumb leg through a pole.
    pub fn leg_at(&self, cursor: LegCursor) -> Result<RouteLeg> {
        let index = usize::from(self.leg(cursor.index)?.index);
        let (from, to) = match (
            self.waypoints.get(index),
            self.waypoints.get(index.saturating_add(1)),
        ) {
            (Some(from), Some(to)) => (*from, *to),
            // `leg` has confirmed both waypoints exist.
            _ => {
                return Err(NavigationError::Kernel(KernelError::Missing {
                    what: "the waypoints of a leg",
                }))
            }
        };
        Ok(RouteLeg {
            index,
            from,
            to,
            sailing: self.sail(from, to)?,
        })
    }

    /// Sailing between two points of this route's leg type.
    fn sail(&self, from: Position, to: Position) -> Result<Sailing> {
        match self.kind {
            LegKind::RhumbLine => rhumb_line(from, to),
            LegKind::GreatCircle => great_circle(from, to),
        }
    }
}

/// Serialised form; deserialisation checks the invariant.
#[cfg(feature = "serde")]
#[derive(serde::Serialize, serde::Deserialize)]
struct StoredRoute {
    waypoints: alloc::vec::Vec<Position>,
    kind: LegKind,
}

#[cfg(feature = "serde")]
impl TryFrom<StoredRoute> for Route {
    type Error = NavigationError;

    /// Deserialised through [`Route::new`]; fewer than two waypoints is
    /// rejected.
    fn try_from(stored: StoredRoute) -> Result<Self> {
        Self::new(&stored.waypoints, stored.kind)
    }
}

#[cfg(feature = "serde")]
impl From<Route> for StoredRoute {
    fn from(route: Route) -> Self {
        Self {
            waypoints: route.waypoints.as_slice().to_vec(),
            kind: route.kind,
        }
    }
}

/// Splits a rhumb leg into pieces no longer than `interval`, into a buffer.
fn rhumb_waypoints_into(
    from: Position,
    to: Position,
    interval: Distance,
    out: &mut [Position],
) -> Result<usize> {
    let sailing = rhumb_line(from, to)?;
    let total = sailing.distance.nautical_miles();
    let pieces = pieces_of(total, interval, out.len())?;
    let count = crate::math::to_usize(pieces);
    // One more position than pieces; `pieces_of` bounded the pieces by the
    // buffer, so this cannot wrap.
    let positions = count.saturating_add(1);

    for step in 0..positions {
        let run = total * crate::math::count_to_f64(step) / pieces;
        let position = crate::sailings::rhumb_destination(
            from,
            sailing.initial_course,
            Distance::from_nautical_miles_unchecked(run),
        )?;
        match out.get_mut(step) {
            Some(slot) => *slot = position,
            None => {
                return Err(NavigationError::Kernel(KernelError::BufferTooSmall {
                    needed: positions,
                    found: out.len(),
                }))
            }
        }
    }
    Ok(positions)
}

/// Appends a waypoint, or reports the required capacity.
fn push_waypoint(waypoints: &mut Waypoints, waypoint: Position, needed: usize) -> Result<()> {
    waypoints.push(waypoint).map_err(|full| {
        NavigationError::Kernel(KernelError::CapacityExceeded {
            context: "a route",
            needed,
            capacity: full.capacity,
        })
    })
}

/// Extracts the two positions of a `windows(2)` slice.
fn pair_of(pair: &[Position]) -> Result<(Position, Position)> {
    match (pair.first(), pair.last()) {
        (Some(from), Some(to)) => Ok((*from, *to)),
        _ => Err(NavigationError::Kernel(KernelError::InsufficientData {
            found: pair.len(),
            required: 2,
            context: "a route leg",
        })),
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp, clippy::indexing_slicing)]
mod tests {
    use super::*;
    use crate::sailings::TrackSide;
    use alloc::vec;
    use alloc::vec::Vec;

    fn at(latitude: f64, longitude: f64) -> Position {
        Position::from_degrees(latitude, longitude).unwrap()
    }

    fn square() -> Route {
        // 60 NM north, 60 NM east, 60 NM south.
        Route::new(
            &[at(0.0, 0.0), at(1.0, 0.0), at(1.0, 1.0), at(0.0, 1.0)],
            LegKind::RhumbLine,
        )
        .unwrap()
    }

    #[test]
    fn a_route_needs_somewhere_to_go() {
        assert!(matches!(
            Route::new(&[], LegKind::RhumbLine).unwrap_err(),
            NavigationError::Kernel(KernelError::InsufficientData { found: 0, .. })
        ));
        assert!(Route::new(&[at(0.0, 0.0)], LegKind::RhumbLine).is_err());
        assert!(Route::new(&[at(0.0, 0.0), at(1.0, 0.0)], LegKind::RhumbLine).is_ok());
    }

    #[test]
    fn legs_and_distances_add_up() {
        let route = square();
        assert_eq!(route.leg_count(), 3);

        let legs = route.legs().collect::<Result<Vec<_>>>().unwrap();
        assert_eq!(legs.len(), 3);
        assert_eq!(legs[0].index, 0);
        assert!(legs[0].sailing.initial_course.degrees().abs() < 1e-9);
        assert!((legs[1].sailing.initial_course.degrees() - 90.0).abs() < 1e-9);
        assert!((legs[2].sailing.initial_course.degrees() - 180.0).abs() < 1e-9);

        let summed: f64 = legs
            .iter()
            .map(|leg| leg.sailing.distance.nautical_miles())
            .sum();
        assert!((summed - route.total_distance().unwrap().nautical_miles()).abs() < 1e-9);
        // 60 NM up, 60 across near the equator, 60 down.
        assert!((summed - 180.0).abs() < 0.2);
    }

    #[test]
    fn the_schedule_works_both_ways() {
        let route = square();
        let speed = Speed::from_knots(12.0).unwrap();
        let elapsed = route.passage_time(speed).unwrap();
        let required = route.speed_required(elapsed).unwrap();
        assert!((required.knots() - 12.0).abs() < 1e-6);

        assert!(route.passage_time(Speed::ZERO).is_err());
        assert!(route.speed_required(Duration::ZERO).is_err());
    }

    #[test]
    fn a_great_circle_route_is_shorter_than_the_rhumb_one() {
        let waypoints = vec![at(49.95, -5.2), at(46.66, -53.07)];
        let direct = Route::new(&waypoints, LegKind::GreatCircle).unwrap();
        let steered = Route::new(&waypoints, LegKind::RhumbLine).unwrap();
        assert!(
            direct.total_distance().unwrap().nautical_miles()
                < steered.total_distance().unwrap().nautical_miles()
        );
        assert_eq!(direct.kind(), LegKind::GreatCircle);
    }

    #[test]
    fn splitting_a_great_circle_keeps_its_length_and_shortens_its_legs() {
        let route =
            Route::new(&[at(49.95, -5.2), at(46.66, -53.07)], LegKind::GreatCircle).unwrap();
        let total = route.total_distance().unwrap().nautical_miles();

        let split = route
            .split_legs(Distance::from_nautical_miles(300.0).unwrap())
            .unwrap();
        assert_eq!(split.kind(), LegKind::RhumbLine);
        assert!(split.leg_count() > route.leg_count());

        for leg in split.legs().collect::<Result<Vec<_>>>().unwrap() {
            assert!(leg.sailing.distance.nautical_miles() <= 300.0 + 1e-6);
        }

        // Steering as rhumb legs costs little.
        let steered = split.total_distance().unwrap().nautical_miles();
        assert!(steered >= total - 1e-6);
        assert!((steered - total) / total < 0.001, "{steered} vs {total}");
    }

    #[test]
    fn splitting_keeps_the_ends_where_they_were() {
        let route = square();
        let split = route
            .split_legs(Distance::from_nautical_miles(10.0).unwrap())
            .unwrap();
        let first = split.waypoints().first().copied().unwrap();
        let last = split.waypoints().last().copied().unwrap();
        assert!(
            rhumb_line(route.waypoints()[0], first)
                .unwrap()
                .distance
                .nautical_miles()
                < 1e-9
        );
        assert!(
            rhumb_line(*route.waypoints().last().unwrap(), last)
                .unwrap()
                .distance
                .nautical_miles()
                < 1e-6
        );
    }

    #[test]
    fn splitting_refuses_a_useless_interval() {
        let route = square();
        assert!(route.split_legs(Distance::ZERO).is_err());
        assert!(route
            .split_legs(Distance::from_nautical_miles(-1.0).unwrap())
            .is_err());
        assert!(route
            .split_legs(Distance::from_nautical_miles(1e-9).unwrap())
            .is_err());
    }

    #[test]
    fn progress_on_the_track_is_all_zeros_off_track() {
        let route = square();
        // Halfway along the first leg.
        let position = at(0.5, 0.0);
        let progress = route.progress(position).unwrap();

        assert_eq!(progress.leg, 0);
        assert_eq!(progress.cross_track.side, TrackSide::OnTrack);
        assert!(progress.cross_track.distance.nautical_miles() < 1e-6);
        assert!((progress.distance_to_next.nautical_miles() - 30.0).abs() < 0.1);
        assert!(progress.course_to_next.degrees().abs() < 1e-6);
        // 30 NM left on this leg plus the next two.
        assert!((progress.distance_to_end.nautical_miles() - 150.0).abs() < 0.2);
    }

    #[test]
    fn progress_knows_which_side_of_the_track_the_ship_is_on() {
        let route = square();
        // Slightly east of a northbound leg: starboard.
        let east = route.progress(at(0.5, 0.05)).unwrap();
        assert_eq!(east.leg, 0);
        assert_eq!(east.cross_track.side, TrackSide::Starboard);
        assert!((east.cross_track.distance.nautical_miles() - 3.0).abs() < 0.05);

        let west = route.progress(at(0.5, -0.05)).unwrap();
        assert_eq!(west.cross_track.side, TrackSide::Port);
        assert!(west.cross_track.signed().is_negative());
    }

    #[test]
    fn progress_moves_from_leg_to_leg() {
        let route = square();
        assert_eq!(route.progress(at(0.2, 0.0)).unwrap().leg, 0);
        assert_eq!(route.progress(at(1.0, 0.5)).unwrap().leg, 1);
        assert_eq!(route.progress(at(0.5, 1.0)).unwrap().leg, 2);
    }

    #[test]
    fn the_distance_left_shrinks_all_the_way_along() {
        let route = square();
        let mut previous = f64::MAX;
        for step in 0..=20 {
            let latitude = f64::from(step) / 20.0;
            let progress = route.progress(at(latitude, 0.0)).unwrap();
            let remaining = progress.distance_to_end.nautical_miles();
            assert!(
                remaining <= previous + 1e-6,
                "at {latitude}° the distance left grew to {remaining}"
            );
            previous = remaining;
        }
    }

    #[test]
    fn a_ship_past_the_end_still_gets_an_answer() {
        let route = square();
        // Well beyond the last waypoint.
        let progress = route.progress(at(-1.0, 1.0)).unwrap();
        assert_eq!(progress.leg, 2);
        assert!(progress.distance_to_next.nautical_miles() > 0.0);
        assert!(progress.cross_track.along_track.nautical_miles() > 0.0);
    }

    #[test]
    fn a_ship_before_the_start_gets_a_negative_along_track() {
        let route = square();
        let progress = route.progress(at(-0.5, 0.0)).unwrap();
        assert_eq!(progress.leg, 0);
        assert!(progress.cross_track.along_track.is_negative());
    }

    #[test]
    fn a_route_with_a_repeated_waypoint_is_reported_not_divided_by_zero() {
        let route = Route::new(
            &[at(10.0, 10.0), at(10.0, 10.0), at(11.0, 10.0)],
            LegKind::RhumbLine,
        )
        .unwrap();
        // A zero-length leg has no track to be off.
        assert!(route.progress(at(10.5, 10.0)).is_err());
        // Its length is still defined.
        assert!(route.total_distance().unwrap().nautical_miles() > 59.0);
    }
}
