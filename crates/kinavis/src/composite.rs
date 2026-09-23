//! Composite sailing: great circle limited to a maximum latitude.
//!
//! On long east-west high-latitude passages the great circle climbs towards
//! ice, weather or a charter limit. The composite track is a great circle from
//! the departure tangent to the limiting parallel, a run along the parallel,
//! and a great circle from the parallel to the destination. It is the shortest
//! track that does not cross the parallel; the arcs touch it at their vertices,
//! so it is constructed directly, not searched.
//!
//! [`composite_sailing`] computes it, or reports that the great circle stays
//! within the limit; [`CompositeSailing::route`] converts it to rhumb legs.
//!
//! # Model
//!
//! Sphere of radius [`EARTH_RADIUS`]. For departure latitude `φ` and limit `L`
//! in the same hemisphere, the tangent circle's vertex is at `cos DLo = tan φ /
//! tan L` east or west of the departure, likewise for the destination; between
//! the vertices the track follows the parallel.
//!
//! ```rust
//! use kinavis::composite::composite_sailing;
//! use kinavis::{Latitude, Position};
//!
//! // Yokohama to San Francisco, held below 45°N.
//! let from: Position = "34°52.0'N 139°42.0'E".parse()?;
//! let to: Position = "37°48.0'N 122°30.0'W".parse()?;
//! let track = composite_sailing(from, to, Latitude::from_degrees(45.0)?)?;
//!
//! // The great circle would climb past 48°N, so the track runs the parallel.
//! assert!(track.is_limited());
//! let run = track.parallel_run().unwrap();
//! assert_eq!(format!("{:.1}", run.first_vertex.latitude().degrees()), "45.0");
//! // Holding to 45° costs about fifteen miles in four and a half thousand.
//! assert_eq!(format!("{:.0}", track.extra_distance().nautical_miles()), "15");
//! assert_eq!(format!("{:.0}", track.total_distance().nautical_miles()), "4506");
//! # Ok::<(), kinavis::NavigationError>(())
//! ```

use crate::angle::TrueCourse;
use crate::error::{ensure_range, KernelError, NavigationError, Result};
use crate::math;
use crate::position::{Latitude, Longitude, Position};
use crate::route::{LegKind, Route, MAX_WAYPOINTS};
use crate::sailings::{great_circle, great_circle_vertex, rhumb_line, Sailing};
use crate::units::Distance;

// Other sailings share `EARTH_RADIUS`; referenced in the docs.
#[allow(unused_imports)]
use crate::sailings::EARTH_RADIUS;

/// Non-great-circle part of a composite track: arc to the parallel, run along
/// it, arc from it.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ParallelRun {
    /// Great-circle arc from the departure to the parallel, arriving
    /// tangentially.
    pub to_parallel: Sailing,
    /// Tangent point of the first arc.
    pub first_vertex: Position,
    /// Run along the parallel: rhumb line due east or west.
    pub along_parallel: Sailing,
    /// Departure point from the parallel.
    pub second_vertex: Position,
    /// Great-circle arc from the parallel to the destination, leaving
    /// tangentially.
    pub from_parallel: Sailing,
}

impl ParallelRun {
    /// Total length of the three parts.
    #[must_use]
    pub fn total_distance(&self) -> Distance {
        self.to_parallel.distance + self.along_parallel.distance + self.from_parallel.distance
    }
}

/// Track between two positions within a limiting latitude.
///
/// Built by [`composite_sailing`]. If the great circle stays within the limit
/// there is no parallel run and the track is the great circle.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct CompositeSailing {
    from: Position,
    to: Position,
    limit: Latitude,
    great_circle: Sailing,
    run: Option<ParallelRun>,
}

impl CompositeSailing {
    /// Departure.
    #[must_use]
    pub const fn from(&self) -> Position {
        self.from
    }

    /// Destination.
    #[must_use]
    pub const fn to(&self) -> Position {
        self.to
    }

    /// Limiting latitude.
    #[must_use]
    pub const fn limit(&self) -> Latitude {
        self.limit
    }

    /// Unconstrained great circle, for comparison.
    #[must_use]
    pub const fn great_circle(&self) -> Sailing {
        self.great_circle
    }

    /// Whether the limit is active; `false` when the great circle stays within
    /// it.
    #[must_use]
    pub const fn is_limited(&self) -> bool {
        self.run.is_some()
    }

    /// Parallel run, if the limit is active.
    #[must_use]
    pub const fn parallel_run(&self) -> Option<&ParallelRun> {
        self.run.as_ref()
    }

    /// Track length: composite if the limit is active, else great circle.
    #[must_use]
    pub fn total_distance(&self) -> Distance {
        self.run
            .as_ref()
            .map_or(self.great_circle.distance, ParallelRun::total_distance)
    }

    /// Extra distance due to the limit; zero if inactive.
    #[must_use]
    pub fn extra_distance(&self) -> Distance {
        self.total_distance() - self.great_circle.distance
    }

    /// Initial course.
    #[must_use]
    pub fn initial_course(&self) -> TrueCourse {
        self.run
            .as_ref()
            .map_or(self.great_circle.initial_course, |run| {
                run.to_parallel.initial_course
            })
    }

    /// Track as rhumb legs no longer than `interval`.
    ///
    /// Great-circle parts are split as by [`Route::split_legs`]; the parallel
    /// run, already a rhumb line, is split to the same interval for uniformity.
    ///
    /// # Errors
    ///
    /// As [`Route::split_legs`]: non-positive interval, or too short for
    /// [`MAX_WAYPOINTS`].
    pub fn route(&self, interval: Distance) -> Result<Route> {
        let Some(run) = self.run.as_ref() else {
            return Route::new(&[self.from, self.to], LegKind::GreatCircle)?.split_legs(interval);
        };
        let up = Route::new(&[self.from, run.first_vertex], LegKind::GreatCircle)?
            .split_legs(interval)?;
        let along = Route::new(&[run.first_vertex, run.second_vertex], LegKind::RhumbLine)?
            .split_legs(interval)?;
        let down = Route::new(&[run.second_vertex, self.to], LegKind::GreatCircle)?
            .split_legs(interval)?;

        let mut waypoints = crate::inline::Inline::<Position, MAX_WAYPOINTS>::new(self.from);
        // The three parts share their two junctions.
        let needed = up
            .waypoints()
            .len()
            .saturating_add(along.waypoints().len())
            .saturating_add(down.waypoints().len())
            .saturating_sub(2);
        for (part, skip) in [(&up, 0), (&along, 1), (&down, 1)] {
            for &waypoint in part.waypoints().iter().skip(skip) {
                waypoints.push(waypoint).map_err(|full| {
                    NavigationError::Kernel(KernelError::CapacityExceeded {
                        context: "a composite route",
                        needed,
                        capacity: full.capacity,
                    })
                })?;
            }
        }
        Route::new(&waypoints, LegKind::RhumbLine)
    }
}

/// Shortest track from `from` to `to` not beyond `limit`: the great circle if
/// within, otherwise the composite track.
///
/// One parallel in one hemisphere; passages in the other hemisphere get the
/// great circle. Both ends must be within the limit.
///
/// # Errors
///
/// - [`KernelError::OutOfRange`] if the limit is the equator or a pole, or
///   either end is beyond it.
/// - [`KernelError::Indeterminate`] for antipodal ends (no unique great
///   circle).
pub fn composite_sailing(
    from: Position,
    to: Position,
    limit: Latitude,
) -> Result<CompositeSailing> {
    ensure_range(
        "limiting latitude",
        math::abs(limit.degrees()),
        f64::MIN_POSITIVE,
        90.0 - 1e-9,
    )?;
    let direct = great_circle(from, to)?;
    let sailing = CompositeSailing {
        from,
        to,
        limit,
        great_circle: direct,
        run: None,
    };

    // Hemisphere of the limit; all latitudes measured towards its pole.
    let sign = if limit.degrees() < 0.0 { -1.0 } else { 1.0 };
    let l = math::abs(limit.radians());
    let from_towards = sign * from.latitude().radians();
    let to_towards = sign * to.latitude().radians();
    for (name, value) in [("departure", from_towards), ("destination", to_towards)] {
        if value > l {
            return Err(NavigationError::Kernel(KernelError::OutOfRange {
                parameter: name,
                value: math::to_degrees(sign * value),
                min: -math::abs(limit.degrees()),
                max: math::abs(limit.degrees()),
            }));
        }
    }

    // Vertex on the limit's side, and whether the track passes through it.
    let northern = great_circle_vertex(from, direct.initial_course)?;
    let vertex = if sign < 0.0 {
        Position::new(
            Latitude::from_degrees_clamped(-northern.latitude().degrees()),
            Longitude::from_degrees_wrapped(northern.longitude().degrees() + 180.0),
        )
    } else {
        northern
    };
    if sign * vertex.latitude().radians() <= l || !passes_through(from, vertex, to, &direct)? {
        return Ok(sailing);
    }

    // Bowditch: tangent vertex at `cos DLo = tan φ / tan L`, towards the
    // destination.
    let direction = longitude_direction(from, to);
    let first_vertex = Position::new(
        limit,
        Longitude::from_degrees_wrapped(
            from.longitude().degrees()
                + direction * math::to_degrees(vertex_offset(from_towards, l)),
        ),
    );
    let second_vertex = Position::new(
        limit,
        Longitude::from_degrees_wrapped(
            to.longitude().degrees() - direction * math::to_degrees(vertex_offset(to_towards, l)),
        ),
    );

    // The run is due east or west by construction: a rhumb line along one
    // parallel.
    let along_parallel = rhumb_line(first_vertex, second_vertex)?;
    Ok(CompositeSailing {
        run: Some(ParallelRun {
            to_parallel: great_circle(from, first_vertex)?,
            first_vertex,
            along_parallel,
            second_vertex,
            from_parallel: great_circle(second_vertex, to)?,
        }),
        ..sailing
    })
}

/// Longitude difference from a point at latitude `towards` (measured towards
/// the limit's pole) to the vertex of the great circle through it tangent to
/// parallel `l`.
fn vertex_offset(towards: f64, l: f64) -> f64 {
    // `|towards| ≤ l < 90°`, so the ratio is in `[-1, 1]` up to rounding; the
    // constant-bound clamp handles rounding.
    math::acos((math::tan(towards) / math::tan(l)).clamp(-1.0, 1.0))
}

/// `+1` if the destination is east by the short way, `-1` if west; east if
/// neither.
fn longitude_direction(from: Position, to: Position) -> f64 {
    if from.longitude_difference(to).degrees() < 0.0 {
        -1.0
    } else {
        1.0
    }
}

/// Whether a point on the great circle through `from` and `to` lies between
/// them: the two partial arcs sum to the whole.
fn passes_through(from: Position, point: Position, to: Position, whole: &Sailing) -> Result<bool> {
    let first = great_circle(from, point)?.distance.nautical_miles();
    let second = great_circle(point, to)?.distance.nautical_miles();
    let total = whole.distance.nautical_miles();
    Ok(math::is_effectively_zero(
        first + second - total,
        total.max(1.0),
    ))
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;

    fn at(latitude: f64, longitude: f64) -> Position {
        Position::from_degrees(latitude, longitude).unwrap()
    }

    fn limit(degrees: f64) -> Latitude {
        Latitude::from_degrees(degrees).unwrap()
    }

    /// Yokohama to San Francisco, great circle reaching ~48°N.
    fn pacific() -> (Position, Position) {
        (at(34.87, 139.7), at(37.8, -122.5))
    }

    #[test]
    fn a_limit_the_great_circle_keeps_within_changes_nothing() {
        let (from, to) = pacific();
        let track = composite_sailing(from, to, limit(60.0)).unwrap();
        assert!(!track.is_limited());
        assert_eq!(track.parallel_run(), None);
        assert_eq!(track.total_distance(), track.great_circle().distance);
        assert_eq!(track.extra_distance(), Distance::ZERO);
        assert_eq!(track.initial_course(), track.great_circle().initial_course);
    }

    #[test]
    fn a_limit_in_the_other_hemisphere_is_never_reached() {
        let (from, to) = pacific();
        assert!(!composite_sailing(from, to, limit(-40.0))
            .unwrap()
            .is_limited());
    }

    #[test]
    fn the_track_touches_the_parallel_and_runs_along_it() {
        let (from, to) = pacific();
        let track = composite_sailing(from, to, limit(45.0)).unwrap();
        let run = track.parallel_run().unwrap();

        // Both vertices on the parallel, in eastward longitude order.
        assert!((run.first_vertex.latitude().degrees() - 45.0).abs() < 1e-9);
        assert!((run.second_vertex.latitude().degrees() - 45.0).abs() < 1e-9);
        assert!(
            run.first_vertex
                .longitude_difference(run.second_vertex)
                .degrees()
                > 0.0
        );
        assert!(from.longitude_difference(run.first_vertex).degrees() > 0.0);
        assert!(run.second_vertex.longitude_difference(to).degrees() > 0.0);

        // The arcs meet the parallel tangentially: due east.
        assert!((run.to_parallel.final_course.degrees() - 90.0).abs() < 0.01);
        assert!((run.from_parallel.initial_course.degrees() - 90.0).abs() < 0.01);
        assert!((run.along_parallel.initial_course.degrees() - 90.0).abs() < 1e-9);

        // Longer than the great circle, shorter than the rhumb line.
        let total = track.total_distance().nautical_miles();
        assert!(total > track.great_circle().distance.nautical_miles());
        assert!(total < rhumb_line(from, to).unwrap().distance.nautical_miles());
        assert!((run.total_distance().nautical_miles() - total).abs() < 1e-9);
    }

    #[test]
    fn the_parallel_run_shrinks_as_the_limit_rises() {
        let (from, to) = pacific();
        let mut previous = f64::MAX;
        for degrees in [40.0, 42.0, 44.0, 46.0, 47.5] {
            let track = composite_sailing(from, to, limit(degrees)).unwrap();
            let along = track
                .parallel_run()
                .unwrap()
                .along_parallel
                .distance
                .nautical_miles();
            assert!(along < previous, "{degrees}°: {along} vs {previous}");
            previous = along;
            // The cost decreases accordingly.
            assert!(track.extra_distance().nautical_miles() > 0.0);
        }
    }

    #[test]
    fn a_westbound_and_a_southern_passage_are_the_mirror_images() {
        let (from, to) = pacific();
        let east = composite_sailing(from, to, limit(45.0)).unwrap();
        let west = composite_sailing(to, from, limit(45.0)).unwrap();
        assert!(
            (east.total_distance().nautical_miles() - west.total_distance().nautical_miles()).abs()
                < 1e-6
        );
        let run = west.parallel_run().unwrap();
        assert!((run.along_parallel.initial_course.degrees() - 270.0).abs() < 1e-9);

        // Cape Town to Melbourne, limited to 50°S.
        let south = composite_sailing(at(-33.9, 18.4), at(-37.8, 144.9), limit(-50.0)).unwrap();
        assert!(south.is_limited());
        let run = south.parallel_run().unwrap();
        assert!((run.first_vertex.latitude().degrees() + 50.0).abs() < 1e-9);
        assert!(south.extra_distance().nautical_miles() > 0.0);
    }

    #[test]
    fn an_end_beyond_the_limit_is_refused() {
        let (from, to) = pacific();
        assert!(matches!(
            composite_sailing(from, to, limit(36.0)).unwrap_err(),
            NavigationError::Kernel(KernelError::OutOfRange {
                parameter: "destination",
                ..
            })
        ));
        assert!(matches!(
            composite_sailing(to, from, limit(36.0)).unwrap_err(),
            NavigationError::Kernel(KernelError::OutOfRange {
                parameter: "departure",
                ..
            })
        ));
        assert!(composite_sailing(from, to, limit(0.0)).is_err());
        assert!(composite_sailing(from, to, limit(90.0)).is_err());
    }

    #[test]
    fn a_route_along_the_track_stays_within_the_limit() {
        let (from, to) = pacific();
        let track = composite_sailing(from, to, limit(45.0)).unwrap();
        let route = track
            .route(Distance::from_nautical_miles(200.0).unwrap())
            .unwrap();

        assert_eq!(route.kind(), LegKind::RhumbLine);
        assert_eq!(route.waypoints().first(), Some(&from));
        assert!(
            great_circle(*route.waypoints().last().unwrap(), to)
                .unwrap()
                .distance
                .nautical_miles()
                < 1e-6
        );
        for waypoint in route.waypoints() {
            assert!(waypoint.latitude().degrees() <= 45.0 + 1e-9);
        }
        // Rhumb-leg approximation costs little over the track.
        let steered = route.total_distance().unwrap().nautical_miles();
        let planned = track.total_distance().nautical_miles();
        assert!(steered >= planned - 1e-6);
        assert!((steered - planned) / planned < 0.002);

        // Inactive limit: the split great circle.
        let free = composite_sailing(from, to, limit(60.0))
            .unwrap()
            .route(Distance::from_nautical_miles(200.0).unwrap())
            .unwrap();
        assert!(free.leg_count() > 20);
    }
}
