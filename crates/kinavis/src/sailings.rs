//! Sailings: course and distance between positions.
//!
//! | Function | Model | Path | Use when |
//! |---|---|---|---|
//! | [`rhumb_line`] | sphere | constant course | steering one course |
//! | [`great_circle`] | sphere | shortest on a sphere | ocean passages |
//! | [`geodesic`] | WGS-84 ellipsoid | shortest, exact | metre accuracy needed |
//!
//! Spherical results use the mean Earth radius [`EARTH_RADIUS`] (6371.0088 km),
//! up to ~0.5 % off the ellipsoid on a long leg.
//!
//! # Example
//!
//! ```rust
//! use kinavis::sailings::{great_circle, rhumb_line};
//! use kinavis::{NavigationError, Position};
//!
//! fn main() -> Result<(), NavigationError> {
//!     // The Lizard to Cape Race.
//!     let from = Position::from_degrees(49.95, -5.20)?;
//!     let to = Position::from_degrees(46.66, -53.07)?;
//!
//!     let direct = great_circle(from, to)?;
//!     let steered = rhumb_line(from, to)?;
//!
//!     // The great circle is shorter, but the course changes the whole way.
//!     assert!(direct.distance < steered.distance);
//!     assert!(direct.initial_course != direct.final_course);
//!     assert_eq!(steered.initial_course, steered.final_course);
//!     Ok(())
//! }
//! ```

use core::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI};

use crate::angle::{Direction, True, TrueCourse};
use crate::error::{KernelError, NavigationError, Result};
use crate::math;
use crate::position::{
    GeocentricUnit, Latitude, Longitude, Position, WGS84_FLATTENING, WGS84_SEMI_MAJOR_AXIS_METRES,
};
use crate::units::{Distance, METRES_PER_NAUTICAL_MILE};

/// Mean Earth radius for the spherical sailings.
pub const EARTH_RADIUS: Distance =
    Distance::from_nautical_miles_unchecked(6_371_008.8 / METRES_PER_NAUTICAL_MILE);

/// Vincenty convergence tolerance, radians.
///
/// 1 rad ≈ 3440 NM at the surface, so this is sub-micrometre: arithmetic is the
/// limit.
pub const TOLERANCE_GEODESIC_RAD: f64 = 1e-12;

/// Maximum Vincenty iterations.
///
/// Public so a caller receiving [`KernelError::NotConverged`] can interpret the
/// reported count. The inverse converges in a few steps for ordinary pairs,
/// slowly for nearly antipodal ones, and not at all for exactly antipodal ones,
/// hence the bound.
pub const MAX_ITERATIONS_GEODESIC: u32 = 200;
/// Angular separation below which two positions are the same place.
const COINCIDENT: f64 = 1e-12;

/// Course and distance between two positions.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Sailing {
    /// Initial course.
    pub initial_course: TrueCourse,
    /// Course on arrival.
    ///
    /// Equal to `initial_course` for a rhumb line; differs for great circles
    /// and geodesics, which are therefore steered in legs.
    pub final_course: TrueCourse,
    /// Track length.
    pub distance: Distance,
}

/// Arrival position and course.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Arrival {
    /// Arrival position.
    pub position: Position,
    /// Course on arrival.
    pub final_course: TrueCourse,
}

/// Side of a track.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum TrackSide {
    /// On the track, within rounding.
    OnTrack,
    /// Left of the track, looking along it.
    Port,
    /// Right of the track, looking along it.
    Starboard,
}

/// Cross-track and along-track distance of a position relative to a leg.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct CrossTrack {
    /// Perpendicular distance from the track, non-negative; see `side`.
    pub distance: Distance,
    /// Side of the track.
    pub side: TrackSide,
    /// Distance from the leg start to the foot of the perpendicular.
    ///
    /// Negative before the start; greater than the leg length beyond the end.
    pub along_track: Distance,
    /// Distance to go along the track to the end of the leg.
    pub to_run: Distance,
}

impl CrossTrack {
    /// Cross-track distance, positive to starboard.
    #[must_use]
    pub fn signed(&self) -> Distance {
        match self.side {
            TrackSide::Port => -self.distance,
            TrackSide::OnTrack | TrackSide::Starboard => self.distance,
        }
    }
}

// ---------------------------------------------------------------------------
// Rhumb line
// ---------------------------------------------------------------------------

/// Rhumb-line course and distance.
///
/// # Errors
///
/// [`KernelError::Indeterminate`] if either position is at a pole.
pub fn rhumb_line(from: Position, to: Position) -> Result<Sailing> {
    if from.latitude().is_polar() || to.latitude().is_polar() {
        return Err(NavigationError::Kernel(KernelError::Indeterminate {
            quantity: "a rhumb line through a pole",
        }));
    }

    let latitude_difference = to.latitude().radians() - from.latitude().radians();
    let longitude_difference = from.longitude_difference(to).radians();
    let stretched = stretched_difference(from.latitude(), to.latitude());

    // East-west scale factor dφ/dψ → cos φ as the latitudes converge (0/0
    // limit).
    let scale = if math::abs(stretched) > 1e-12 {
        latitude_difference / stretched
    } else {
        math::cos(from.latitude().radians())
    };

    let angular = math::sqrt(
        latitude_difference * latitude_difference
            + scale * scale * longitude_difference * longitude_difference,
    );
    let course = Direction::<True>::from_degrees_wrapped(math::to_degrees(math::atan2(
        longitude_difference,
        stretched,
    )));

    Ok(Sailing {
        initial_course: course,
        final_course: course,
        distance: from_angular(angular),
    })
}

/// Rhumb-line destination for a course and distance.
///
/// # Near the poles
///
/// A non-cardinal rhumb line spirals towards the pole. Long high-latitude
/// tracks can cross a meridian several times; the arrival position does not
/// record the windings, so measuring back with [`rhumb_line`] gives the shorter
/// unwound distance. This is a property of the rhumb line, and a reason
/// high-latitude passages use great-circle legs.
///
/// # Errors
///
/// - [`KernelError::NotFinite`] for a non-finite distance.
/// - [`KernelError::Indeterminate`] if the track would cross a pole.
pub fn rhumb_destination(
    from: Position,
    course: TrueCourse,
    distance: Distance,
) -> Result<Position> {
    let angular = to_angular(distance)?;
    let course_radians = course.radians();
    let latitude_difference = angular * math::cos(course_radians);
    let latitude_radians = from.latitude().radians() + latitude_difference;

    if math::abs(latitude_radians) > FRAC_PI_2 {
        return Err(NavigationError::Kernel(KernelError::Indeterminate {
            quantity: "a rhumb line beyond the pole",
        }));
    }

    let latitude = Latitude::from_degrees_clamped(math::to_degrees(latitude_radians));
    let stretched = stretched_difference(from.latitude(), latitude);
    let scale = if math::abs(stretched) > 1e-12 {
        latitude_difference / stretched
    } else {
        math::cos(from.latitude().radians())
    };

    let longitude_difference = if math::abs(scale) < 1e-12 {
        0.0
    } else {
        angular * math::sin(course_radians) / scale
    };

    Ok(Position::new(
        latitude,
        Longitude::from_degrees_wrapped(
            from.longitude().degrees() + math::to_degrees(longitude_difference),
        ),
    ))
}

/// Difference of spherical isometric latitude, radians.
fn stretched_difference(from: Latitude, to: Latitude) -> f64 {
    math::ln(math::tan(FRAC_PI_4 + to.radians() / 2.0))
        - math::ln(math::tan(FRAC_PI_4 + from.radians() / 2.0))
}

// ---------------------------------------------------------------------------
// Great circle
// ---------------------------------------------------------------------------

/// Great-circle course and distance: shortest track on a sphere.
///
/// Coincident positions: zero distance, courses `000°`. Antipodal positions:
/// every great circle has the same length; the course returned is arbitrary.
///
/// # Errors
///
/// Never fails; returns `Result` for consistency with the other sailings.
pub fn great_circle(from: Position, to: Position) -> Result<Sailing> {
    let (from_latitude, to_latitude) = (from.latitude().radians(), to.latitude().radians());
    let longitude_difference = from.longitude_difference(to).radians();
    let latitude_difference = to_latitude - from_latitude;

    let half_latitude = math::sin(latitude_difference / 2.0);
    let half_longitude = math::sin(longitude_difference / 2.0);
    let chord = half_latitude * half_latitude
        + math::cos(from_latitude) * math::cos(to_latitude) * half_longitude * half_longitude;
    let angular = 2.0 * math::asin(math::sqrt(chord).min(1.0));

    Ok(Sailing {
        initial_course: initial_course(from, to),
        final_course: initial_course(to, from).reciprocal(),
        distance: from_angular(angular),
    })
}

/// Great-circle destination for an initial course and distance.
///
/// # Errors
///
/// [`KernelError::NotFinite`] for a non-finite distance.
pub fn great_circle_destination(
    from: Position,
    course: TrueCourse,
    distance: Distance,
) -> Result<Arrival> {
    let angular = to_angular(distance)?;
    let latitude = from.latitude().radians();
    let course_radians = course.radians();

    let sine_latitude = math::sin(latitude) * math::cos(angular)
        + math::cos(latitude) * math::sin(angular) * math::cos(course_radians);
    let destination_latitude = math::asin(sine_latitude.clamp(-1.0, 1.0));
    let longitude_difference = math::atan2(
        math::sin(course_radians) * math::sin(angular) * math::cos(latitude),
        math::cos(angular) - math::sin(latitude) * sine_latitude,
    );

    let position = Position::new(
        Latitude::from_degrees_clamped(math::to_degrees(destination_latitude)),
        Longitude::from_degrees_wrapped(
            from.longitude().degrees() + math::to_degrees(longitude_difference),
        ),
    );

    Ok(Arrival {
        position,
        final_course: initial_course(position, from).reciprocal(),
    })
}

/// Point at `fraction` along a great circle; values outside `0..=1`
/// extrapolate.
///
/// # Errors
///
/// [`KernelError::NotFinite`] for a non-finite fraction;
/// [`KernelError::Indeterminate`] for antipodal positions (no unique great
/// circle).
pub fn great_circle_intermediate(from: Position, to: Position, fraction: f64) -> Result<Position> {
    crate::error::ensure_finite("fraction", fraction)?;
    let angular = to_angular(great_circle(from, to)?.distance)?;

    if angular < COINCIDENT {
        return Ok(from);
    }
    if math::abs(angular - PI) < COINCIDENT {
        return Err(NavigationError::Kernel(KernelError::Indeterminate {
            quantity: "a great circle between antipodal positions",
        }));
    }

    let sine = math::sin(angular);
    let start_weight = math::sin((1.0 - fraction) * angular) / sine;
    let end_weight = math::sin(fraction * angular) / sine;

    let [from_x, from_y, from_z] = from.to_geocentric_unit().components();
    let [to_x, to_y, to_z] = to.to_geocentric_unit().components();

    GeocentricUnit::from_finite(
        start_weight * from_x + end_weight * to_x,
        start_weight * from_y + end_weight * to_y,
        start_weight * from_z + end_weight * to_z,
    )
    .map(Position::from_geocentric_unit)
    .ok_or(NavigationError::Kernel(KernelError::Indeterminate {
        quantity: "a point on the great circle",
    }))
}

/// Splits a great circle into legs of at most `interval`, to steer as rhumb
/// lines.
///
/// Writes positions from `from` to `to` into `out` (`n` legs need `n + 1`
/// slots) and returns the count.
///
/// # Errors
///
/// - [`KernelError::OutOfRange`] if `interval` is not positive.
/// - [`KernelError::BufferTooSmall`] if `out` is too short.
/// - [`KernelError::Indeterminate`] for antipodal positions.
pub fn great_circle_waypoints_into(
    from: Position,
    to: Position,
    interval: Distance,
    out: &mut [Position],
) -> Result<usize> {
    let total = great_circle(from, to)?.distance.nautical_miles();
    let legs = pieces_of(total, interval, out.len())?;
    let count = math::to_usize(legs);
    // One more position than legs; `pieces_of` bounded the legs by the buffer,
    // so this cannot wrap.
    let positions = count.saturating_add(1);

    for step in 0..positions {
        let fraction = math::count_to_f64(step) / legs;
        let position = great_circle_intermediate(from, to, fraction)?;
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

/// As [`great_circle_waypoints_into`], collecting into a `Vec`.
///
/// # Errors
///
/// As [`great_circle_waypoints_into`], except the buffer error.
#[cfg(feature = "alloc")]
pub fn great_circle_waypoints(
    from: Position,
    to: Position,
    interval: Distance,
) -> Result<alloc::vec::Vec<Position>> {
    let total = great_circle(from, to)?.distance.nautical_miles();
    let legs = pieces_of(total, interval, usize::MAX)?;
    let count = math::to_usize(legs);

    let mut waypoints = alloc::vec::Vec::with_capacity(count + 1);
    for step in 0..=count {
        let fraction = math::count_to_f64(step) / legs;
        waypoints.push(great_circle_intermediate(from, to, fraction)?);
    }
    Ok(waypoints)
}

/// Number of pieces for a `total`-mile track at `interval`. Shared by the
/// great-circle and rhumb splitters so the absurd-interval guard lives in one
/// place.
pub(crate) fn pieces_of(total: f64, interval: Distance, room: usize) -> Result<f64> {
    if interval.nautical_miles() <= 0.0 {
        return Err(NavigationError::Kernel(KernelError::OutOfRange {
            parameter: "interval",
            value: interval.nautical_miles(),
            min: f64::MIN_POSITIVE,
            max: f64::MAX,
        }));
    }

    let pieces = math::ceil(total / interval.nautical_miles()).max(1.0);
    // Piece counts beyond this are rejected as unintended.
    if pieces > 1e6 {
        return Err(NavigationError::Kernel(KernelError::OutOfRange {
            parameter: "interval",
            value: interval.nautical_miles(),
            min: total / 1e6,
            max: f64::MAX,
        }));
    }

    // `pieces ≤ 1e6`, so +1 cannot wrap; saturating states it for the compiler.
    let needed = math::to_usize(pieces).saturating_add(1);
    if needed > room {
        return Err(NavigationError::Kernel(KernelError::BufferTooSmall {
            needed,
            found: room,
        }));
    }
    Ok(pieces)
}

/// Northern vertex of a great circle (highest latitude).
///
/// The southern vertex is its antipode. For the equator every point is a
/// vertex; the result is arbitrary.
///
/// # Errors
///
/// [`KernelError::Indeterminate`] if the great circle degenerates.
pub fn great_circle_vertex(from: Position, initial_course: TrueCourse) -> Result<Position> {
    let [pole_x, pole_y, pole_z] = great_circle_pole(from, initial_course);
    let position = GeocentricUnit::from_finite(pole_x, pole_y, pole_z)
        .map(Position::from_geocentric_unit)
        .ok_or(NavigationError::Kernel(KernelError::Indeterminate {
            quantity: "the pole of the great circle",
        }))?;

    let vertex_latitude = 90.0 - math::abs(position.latitude().degrees());
    let vertex_longitude = if position.latitude().degrees() >= 0.0 {
        position.longitude().degrees() + 180.0
    } else {
        position.longitude().degrees()
    };

    Ok(Position::new(
        Latitude::from_degrees_clamped(vertex_latitude),
        Longitude::from_degrees_wrapped(vertex_longitude),
    ))
}

/// Intersection of two great circles, each given by a position and course.
///
/// Great circles cross at two antipodal points; the one nearer the given
/// positions is returned (as a fix requires).
///
/// # Errors
///
/// [`NavigationError::Parallel`] if both define the same circle.
pub fn intersection(
    first: Position,
    first_course: TrueCourse,
    second: Position,
    second_course: TrueCourse,
) -> Result<Position> {
    let a = great_circle_pole(first, first_course);
    let b = great_circle_pole(second, second_course);
    let line = cross(a, b);
    let magnitude = math::sqrt(dot(line, line));

    if magnitude < 1e-12 {
        return Err(NavigationError::Parallel {
            context: "the two great circles",
        });
    }

    let candidate = [
        line[0] / magnitude,
        line[1] / magnitude,
        line[2] / magnitude,
    ];
    // Of the two antipodal crossings, take the one on the side of the input
    // positions.
    let midpoint = {
        let [x1, y1, z1] = first.to_geocentric_unit().components();
        let [x2, y2, z2] = second.to_geocentric_unit().components();
        [x1 + x2, y1 + y2, z1 + z2]
    };
    let chosen = if dot(candidate, midpoint) >= 0.0 {
        candidate
    } else {
        [-candidate[0], -candidate[1], -candidate[2]]
    };

    let [crossing_x, crossing_y, crossing_z] = chosen;
    GeocentricUnit::from_finite(crossing_x, crossing_y, crossing_z)
        .map(Position::from_geocentric_unit)
        .ok_or(NavigationError::Kernel(KernelError::Indeterminate {
            quantity: "the intersection of the two great circles",
        }))
}

/// Intersection of two rhumb lines, each given by a position and course.
///
/// As on a Mercator chart: rhumb lines are straight there, so the crossing is
/// an exact line intersection in meridional parts and longitude. Unlike
/// [`intersection`], the result is unique, which is why the fixes use it.
///
/// # Errors
///
/// - [`NavigationError::Parallel`] for equal or reciprocal courses.
/// - [`KernelError::Indeterminate`] if either position is at a pole.
pub fn rhumb_intersection(
    first: Position,
    first_course: TrueCourse,
    second: Position,
    second_course: TrueCourse,
) -> Result<Position> {
    if first.latitude().is_polar() || second.latitude().is_polar() {
        return Err(NavigationError::Kernel(KernelError::Indeterminate {
            quantity: "a rhumb line through a pole",
        }));
    }

    // Mercator coordinates in minutes: longitude east, meridional parts north.
    // Longitudes relative to the first position, so the antimeridian is not a
    // special case.
    let first_point = (0.0, first.latitude().isometric_minutes());
    let second_point = (
        first.longitude_difference(second).minutes(),
        second.latitude().isometric_minutes(),
    );

    // On Mercator a course C is the straight direction (sin C, cos C).
    let first_direction = (
        math::sin(first_course.radians()),
        math::cos(first_course.radians()),
    );
    let second_direction = (
        math::sin(second_course.radians()),
        math::cos(second_course.radians()),
    );

    let determinant =
        first_direction.0 * second_direction.1 - first_direction.1 * second_direction.0;
    if math::abs(determinant) < 1e-12 {
        return Err(NavigationError::Parallel {
            context: "the two rhumb lines",
        });
    }

    let offset = (
        second_point.0 - first_point.0,
        second_point.1 - first_point.1,
    );
    let along = (offset.0 * second_direction.1 - offset.1 * second_direction.0) / determinant;

    let crossing = (
        first_point.0 + along * first_direction.0,
        first_point.1 + along * first_direction.1,
    );

    if !crossing.0.is_finite() || !crossing.1.is_finite() {
        return Err(NavigationError::Kernel(KernelError::Indeterminate {
            quantity: "the crossing of these rhumb lines",
        }));
    }

    Ok(Position::new(
        Latitude::from_isometric_minutes(crossing.1),
        Longitude::from_degrees_wrapped(first.longitude().degrees() + crossing.0 / 60.0),
    ))
}

/// Cross-track and along-track position relative to a leg.
///
/// # Errors
///
/// [`KernelError::Indeterminate`] for a zero-length leg.
pub fn cross_track(
    position: Position,
    leg_start: Position,
    leg_end: Position,
) -> Result<CrossTrack> {
    let leg = great_circle(leg_start, leg_end)?;
    let leg_angular = to_angular(leg.distance)?;
    if leg_angular < COINCIDENT {
        return Err(NavigationError::Kernel(KernelError::Indeterminate {
            quantity: "the track of a zero-length leg",
        }));
    }

    let to_position = great_circle(leg_start, position)?;
    let position_angular = to_angular(to_position.distance)?;
    let offset = math::to_radians(
        leg.initial_course
            .signed_difference(to_position.initial_course),
    );

    let across = math::asin((math::sin(position_angular) * math::sin(offset)).clamp(-1.0, 1.0));
    let cosine = math::cos(across);
    let along = if math::abs(cosine) < f64::EPSILON {
        0.0
    } else {
        let ratio = (math::cos(position_angular) / cosine).clamp(-1.0, 1.0);
        math::acos(ratio) * if math::cos(offset) < 0.0 { -1.0 } else { 1.0 }
    };

    let side = if math::abs(across) < 1e-12 {
        TrackSide::OnTrack
    } else if across > 0.0 {
        TrackSide::Starboard
    } else {
        TrackSide::Port
    };

    Ok(CrossTrack {
        distance: from_angular(math::abs(across)),
        side,
        along_track: from_angular(along),
        to_run: from_angular(leg_angular - along),
    })
}

// ---------------------------------------------------------------------------
// Geodesic, on the WGS-84 ellipsoid
// ---------------------------------------------------------------------------

/// Geodesic course and distance: shortest track on the ellipsoid.
///
/// Vincenty inverse, sub-millimetre accuracy.
///
/// # Errors
///
/// [`KernelError::NotConverged`] for nearly antipodal positions, where the
/// iteration does not converge; use [`great_circle`] there (the model
/// difference is negligible at half the circumference). The loop is bounded by
/// [`MAX_ITERATIONS_GEODESIC`] and [`TOLERANCE_GEODESIC_RAD`], both reported in
/// the error.
pub fn geodesic(from: Position, to: Position) -> Result<Sailing> {
    let flattening = WGS84_FLATTENING;
    let semi_major = WGS84_SEMI_MAJOR_AXIS_METRES;
    let semi_minor = semi_major * (1.0 - flattening);

    let longitude_difference = from.longitude_difference(to).radians();
    let reduced_from = math::atan((1.0 - flattening) * math::tan(from.latitude().radians()));
    let reduced_to = math::atan((1.0 - flattening) * math::tan(to.latitude().radians()));
    let (sin_from, cos_from) = (math::sin(reduced_from), math::cos(reduced_from));
    let (sin_to, cos_to) = (math::sin(reduced_to), math::cos(reduced_to));

    let mut lambda = longitude_difference;
    let mut sin_sigma = 0.0;
    let mut cos_sigma = 0.0;
    let mut sigma = 0.0;
    let mut cos_squared_alpha = 0.0;
    let mut cos_two_sigma_m = 0.0;
    let mut converged = false;

    for _ in 0..MAX_ITERATIONS_GEODESIC {
        let (sin_lambda, cos_lambda) = (math::sin(lambda), math::cos(lambda));
        let first = cos_to * sin_lambda;
        let second = cos_from * sin_to - sin_from * cos_to * cos_lambda;
        sin_sigma = math::sqrt(first * first + second * second);

        if sin_sigma < COINCIDENT {
            // Coincident positions: zero distance, no course.
            return Ok(Sailing {
                initial_course: Direction::<True>::NORTH,
                final_course: Direction::<True>::NORTH,
                distance: Distance::ZERO,
            });
        }

        cos_sigma = sin_from * sin_to + cos_from * cos_to * cos_lambda;
        sigma = math::atan2(sin_sigma, cos_sigma);
        let sin_alpha = cos_from * cos_to * sin_lambda / sin_sigma;
        cos_squared_alpha = 1.0 - sin_alpha * sin_alpha;
        cos_two_sigma_m = if math::abs(cos_squared_alpha) < f64::EPSILON {
            0.0 // Equatorial track: the midpoint term drops out.
        } else {
            cos_sigma - 2.0 * sin_from * sin_to / cos_squared_alpha
        };

        let correction = flattening / 16.0
            * cos_squared_alpha
            * (4.0 + flattening * (4.0 - 3.0 * cos_squared_alpha));
        let previous = lambda;
        lambda = longitude_difference
            + (1.0 - correction)
                * flattening
                * sin_alpha
                * (sigma
                    + correction
                        * sin_sigma
                        * (cos_two_sigma_m
                            + correction
                                * cos_sigma
                                * (-1.0 + 2.0 * cos_two_sigma_m * cos_two_sigma_m)));

        if math::abs(lambda - previous) < TOLERANCE_GEODESIC_RAD {
            converged = true;
            break;
        }
    }

    if !converged {
        return Err(NavigationError::Kernel(KernelError::NotConverged {
            iterations: MAX_ITERATIONS_GEODESIC,
            residual: math::to_degrees(math::abs(lambda - longitude_difference)),
        }));
    }

    let u_squared = cos_squared_alpha * (semi_major * semi_major - semi_minor * semi_minor)
        / (semi_minor * semi_minor);
    let a_series = 1.0
        + u_squared / 16384.0
            * (4096.0 + u_squared * (-768.0 + u_squared * (320.0 - 175.0 * u_squared)));
    let b_series =
        u_squared / 1024.0 * (256.0 + u_squared * (-128.0 + u_squared * (74.0 - 47.0 * u_squared)));
    let delta_sigma = delta_sigma(b_series, sin_sigma, cos_sigma, cos_two_sigma_m);

    let (sin_lambda, cos_lambda) = (math::sin(lambda), math::cos(lambda));
    Ok(Sailing {
        initial_course: Direction::<True>::from_degrees_wrapped(math::to_degrees(math::atan2(
            cos_to * sin_lambda,
            cos_from * sin_to - sin_from * cos_to * cos_lambda,
        ))),
        final_course: Direction::<True>::from_degrees_wrapped(math::to_degrees(math::atan2(
            cos_from * sin_lambda,
            -sin_from * cos_to + cos_from * sin_to * cos_lambda,
        ))),
        distance: Distance::from_nautical_miles_unchecked(
            semi_minor * a_series * (sigma - delta_sigma) / METRES_PER_NAUTICAL_MILE,
        ),
    })
}

/// Geodesic destination for an initial course and distance (Vincenty direct).
///
/// # Errors
///
/// - [`KernelError::NotFinite`] for a non-finite distance.
/// - [`KernelError::NotConverged`] if the iteration does not settle within
///   [`MAX_ITERATIONS_GEODESIC`] steps of [`TOLERANCE_GEODESIC_RAD`].
pub fn geodesic_destination(
    from: Position,
    course: TrueCourse,
    distance: Distance,
) -> Result<Arrival> {
    crate::error::ensure_finite("distance", distance.nautical_miles())?;
    let flattening = WGS84_FLATTENING;
    let semi_major = WGS84_SEMI_MAJOR_AXIS_METRES;
    let semi_minor = semi_major * (1.0 - flattening);
    let metres = distance.metres();

    let course_radians = course.radians();
    let (sin_course, cos_course) = (math::sin(course_radians), math::cos(course_radians));
    let reduced = math::atan((1.0 - flattening) * math::tan(from.latitude().radians()));
    let (sin_reduced, cos_reduced) = (math::sin(reduced), math::cos(reduced));

    let sigma_one = math::atan2(math::tan(reduced), cos_course);
    let sin_alpha = cos_reduced * sin_course;
    let cos_squared_alpha = 1.0 - sin_alpha * sin_alpha;
    let u_squared = cos_squared_alpha * (semi_major * semi_major - semi_minor * semi_minor)
        / (semi_minor * semi_minor);
    let a_series = 1.0
        + u_squared / 16384.0
            * (4096.0 + u_squared * (-768.0 + u_squared * (320.0 - 175.0 * u_squared)));
    let b_series =
        u_squared / 1024.0 * (256.0 + u_squared * (-128.0 + u_squared * (74.0 - 47.0 * u_squared)));

    let mut sigma = metres / (semi_minor * a_series);
    let mut cos_two_sigma_m = math::cos(2.0 * sigma_one + sigma);
    let mut converged = false;

    for _ in 0..MAX_ITERATIONS_GEODESIC {
        cos_two_sigma_m = math::cos(2.0 * sigma_one + sigma);
        let (sin_sigma, cos_sigma) = (math::sin(sigma), math::cos(sigma));
        let correction = delta_sigma(b_series, sin_sigma, cos_sigma, cos_two_sigma_m);
        let previous = sigma;
        sigma = metres / (semi_minor * a_series) + correction;
        if math::abs(sigma - previous) < TOLERANCE_GEODESIC_RAD {
            converged = true;
            break;
        }
    }

    if !converged {
        return Err(NavigationError::Kernel(KernelError::NotConverged {
            iterations: MAX_ITERATIONS_GEODESIC,
            residual: math::to_degrees(sigma),
        }));
    }

    let (sin_sigma, cos_sigma) = (math::sin(sigma), math::cos(sigma));
    let numerator = sin_reduced * cos_sigma + cos_reduced * sin_sigma * cos_course;
    let denominator_term = sin_reduced * sin_sigma - cos_reduced * cos_sigma * cos_course;
    let latitude = math::atan2(
        numerator,
        (1.0 - flattening)
            * math::sqrt(sin_alpha * sin_alpha + denominator_term * denominator_term),
    );
    let lambda = math::atan2(
        sin_sigma * sin_course,
        cos_reduced * cos_sigma - sin_reduced * sin_sigma * cos_course,
    );
    let correction = flattening / 16.0
        * cos_squared_alpha
        * (4.0 + flattening * (4.0 - 3.0 * cos_squared_alpha));
    let longitude_difference = lambda
        - (1.0 - correction)
            * flattening
            * sin_alpha
            * (sigma
                + correction
                    * sin_sigma
                    * (cos_two_sigma_m
                        + correction
                            * cos_sigma
                            * (-1.0 + 2.0 * cos_two_sigma_m * cos_two_sigma_m)));

    Ok(Arrival {
        position: Position::new(
            Latitude::from_degrees_clamped(math::to_degrees(latitude)),
            Longitude::from_degrees_wrapped(
                from.longitude().degrees() + math::to_degrees(longitude_difference),
            ),
        ),
        final_course: Direction::<True>::from_degrees_wrapped(math::to_degrees(math::atan2(
            sin_alpha,
            -denominator_term,
        ))),
    })
}

/// `Δσ` correction shared by Vincenty's direct and inverse solutions.
fn delta_sigma(b_series: f64, sin_sigma: f64, cos_sigma: f64, cos_two_sigma_m: f64) -> f64 {
    let cos_squared = cos_two_sigma_m * cos_two_sigma_m;
    let sin_squared = sin_sigma * sin_sigma;
    b_series
        * sin_sigma
        * (cos_two_sigma_m
            + b_series / 4.0
                * (cos_sigma * (-1.0 + 2.0 * cos_squared)
                    - b_series / 6.0
                        * cos_two_sigma_m
                        * (-3.0 + 4.0 * sin_squared)
                        * (-3.0 + 4.0 * cos_squared)))
}

// ---------------------------------------------------------------------------
// Shared helpers
// ---------------------------------------------------------------------------

/// Initial great-circle course.
fn initial_course(from: Position, to: Position) -> TrueCourse {
    let (from_latitude, to_latitude) = (from.latitude().radians(), to.latitude().radians());
    let longitude_difference = from.longitude_difference(to).radians();
    Direction::<True>::from_degrees_wrapped(math::to_degrees(math::atan2(
        math::sin(longitude_difference) * math::cos(to_latitude),
        math::cos(from_latitude) * math::sin(to_latitude)
            - math::sin(from_latitude) * math::cos(to_latitude) * math::cos(longitude_difference),
    )))
}

/// Unit normal of the great circle through a position on a course.
fn great_circle_pole(from: Position, course: TrueCourse) -> [f64; 3] {
    let (latitude, longitude) = (from.latitude().radians(), from.longitude().radians());
    let (sin_latitude, cos_latitude) = (math::sin(latitude), math::cos(latitude));
    let (sin_longitude, cos_longitude) = (math::sin(longitude), math::cos(longitude));
    let course_radians = course.radians();
    let (sin_course, cos_course) = (math::sin(course_radians), math::cos(course_radians));

    // Unit tangent at `from`, east and north components in ECEF.
    let east = [-sin_longitude, cos_longitude, 0.0];
    let north = [
        -sin_latitude * cos_longitude,
        -sin_latitude * sin_longitude,
        cos_latitude,
    ];
    let tangent = [
        sin_course * east[0] + cos_course * north[0],
        sin_course * east[1] + cos_course * north[1],
        sin_course * east[2] + cos_course * north[2],
    ];
    cross(from.to_geocentric_unit().components(), tangent)
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// Central angle → surface distance.
fn from_angular(radians: f64) -> Distance {
    Distance::from_nautical_miles_unchecked(radians * EARTH_RADIUS.nautical_miles())
}

/// Surface distance → central angle.
fn to_angular(distance: Distance) -> Result<f64> {
    crate::error::ensure_finite("distance", distance.nautical_miles())?;
    Ok(distance.nautical_miles() / EARTH_RADIUS.nautical_miles())
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp, clippy::indexing_slicing)]
mod tests {
    use super::*;

    fn at(latitude: f64, longitude: f64) -> Position {
        Position::from_degrees(latitude, longitude).unwrap()
    }

    #[test]
    fn one_minute_of_latitude_is_one_mile() {
        let sailing = great_circle(at(0.0, 0.0), at(1.0 / 60.0, 0.0)).unwrap();
        // On the mean-radius sphere this is 1.0007 NM, not exactly 1.
        assert!((sailing.distance.nautical_miles() - 1.0).abs() < 0.001);
        assert!(sailing.initial_course.degrees().abs() < 1e-9);
    }

    #[test]
    fn a_degree_of_latitude_is_sixty_miles() {
        let sailing = great_circle(at(10.0, 30.0), at(11.0, 30.0)).unwrap();
        assert!((sailing.distance.nautical_miles() - 60.0).abs() < 0.1);
        assert!(sailing.initial_course.degrees().abs() < 1e-9);
        assert!(sailing.final_course.degrees().abs() < 1e-9);
    }

    #[test]
    fn quarter_of_the_globe_along_the_equator() {
        let sailing = great_circle(at(0.0, 0.0), at(0.0, 90.0)).unwrap();
        assert!((sailing.distance.nautical_miles() - 90.0 * 60.0).abs() < 10.0);
        assert!((sailing.initial_course.degrees() - 90.0).abs() < 1e-9);
    }

    #[test]
    fn great_circle_courses_change_along_the_track() {
        // High-latitude crossing: large course change.
        let sailing = great_circle(at(50.0, -5.0), at(50.0, -50.0)).unwrap();
        assert!(sailing.initial_course.degrees() > 270.0);
        assert!(sailing.final_course.degrees() < 270.0);
        // Symmetric about the middle meridian.
        let out = 360.0 - sailing.initial_course.degrees();
        let back = 270.0 - sailing.final_course.degrees();
        assert!((out - (90.0 - back) - 0.0).abs() < 30.0);
    }

    #[test]
    fn great_circle_is_never_longer_than_the_rhumb_line() {
        for (from, to) in [
            (at(49.95, -5.2), at(46.66, -53.07)),
            (at(35.0, 139.0), at(37.8, -122.4)),
            (at(-33.9, 151.2), at(-34.6, -58.4)),
            (at(10.0, 0.0), at(-10.0, 30.0)),
        ] {
            let direct = great_circle(from, to).unwrap();
            let steered = rhumb_line(from, to).unwrap();
            assert!(
                direct.distance.nautical_miles() <= steered.distance.nautical_miles() + 1e-6,
                "{:?} vs {:?}",
                direct.distance,
                steered.distance
            );
        }
    }

    #[test]
    fn rhumb_line_along_a_meridian_and_a_parallel() {
        let meridian = rhumb_line(at(10.0, 20.0), at(11.0, 20.0)).unwrap();
        assert!((meridian.distance.nautical_miles() - 60.0).abs() < 0.1);
        assert!(meridian.initial_course.degrees().abs() < 1e-9);

        // Along a parallel the rhumb distance equals the departure.
        let parallel = rhumb_line(at(60.0, 0.0), at(60.0, 2.0)).unwrap();
        assert!((parallel.distance.nautical_miles() - 60.0).abs() < 0.1);
        assert!((parallel.initial_course.degrees() - 90.0).abs() < 1e-9);
    }

    #[test]
    fn rhumb_line_round_trips_through_its_destination() {
        for (from, course, distance) in [
            (at(0.0, 0.0), 45.0, 1000.0),
            (at(50.0, -5.0), 250.0, 2000.0),
            (at(-20.0, 170.0), 300.0, 1500.0),
            (at(60.0, 10.0), 180.0, 600.0),
        ] {
            let course = TrueCourse::new(course).unwrap();
            let distance = Distance::from_nautical_miles(distance).unwrap();
            let destination = rhumb_destination(from, course, distance).unwrap();
            let back = rhumb_line(from, destination).unwrap();

            assert!(
                (back.distance.nautical_miles() - distance.nautical_miles()).abs() < 1e-6,
                "{:?} vs {distance:?}",
                back.distance
            );
            assert!(back.initial_course.angular_distance(course) < 1e-9);
        }
    }

    #[test]
    fn rhumb_line_refuses_to_cross_a_pole() {
        let result = rhumb_destination(
            at(80.0, 0.0),
            TrueCourse::NORTH,
            Distance::from_nautical_miles(1000.0).unwrap(),
        );
        assert!(result.is_err());
        assert!(rhumb_line(at(90.0, 0.0), at(45.0, 0.0)).is_err());
    }

    #[test]
    fn great_circle_round_trips_through_its_destination() {
        for (from, course, distance) in [
            (at(0.0, 0.0), 45.0, 1000.0),
            (at(50.0, -5.0), 250.0, 3000.0),
            (at(-20.0, 170.0), 300.0, 1500.0),
        ] {
            let course = TrueCourse::new(course).unwrap();
            let distance = Distance::from_nautical_miles(distance).unwrap();
            let arrival = great_circle_destination(from, course, distance).unwrap();
            let back = great_circle(from, arrival.position).unwrap();

            assert!((back.distance.nautical_miles() - distance.nautical_miles()).abs() < 1e-6);
            assert!(back.initial_course.angular_distance(course) < 1e-9);
            assert!(back.final_course.angular_distance(arrival.final_course) < 1e-9);
        }
    }

    #[test]
    fn vertex_is_the_highest_latitude_on_the_track() {
        // From the equator on 045: vertex at 45°N, 90° further east.
        let vertex = great_circle_vertex(at(0.0, 0.0), TrueCourse::new(45.0).unwrap()).unwrap();
        assert!((vertex.latitude().degrees() - 45.0).abs() < 1e-9);
        assert!((vertex.longitude().degrees() - 90.0).abs() < 1e-9);

        // On 315: same latitude, 90° to the west.
        let westward = great_circle_vertex(at(0.0, 0.0), TrueCourse::new(315.0).unwrap()).unwrap();
        assert!((westward.latitude().degrees() - 45.0).abs() < 1e-9);
        assert!((westward.longitude().degrees() + 90.0).abs() < 1e-9);
    }

    #[test]
    fn vertex_is_never_beaten_by_a_point_on_the_track() {
        let from = at(35.0, 139.0);
        let course = TrueCourse::new(50.0).unwrap();
        let vertex = great_circle_vertex(from, course).unwrap();

        let mut distance = 0.0;
        while distance < 10_000.0 {
            let point = great_circle_destination(
                from,
                course,
                Distance::from_nautical_miles(distance).unwrap(),
            )
            .unwrap();
            assert!(
                point.position.latitude().degrees() <= vertex.latitude().degrees() + 1e-6,
                "at {distance} miles the track reaches {}, above the vertex {}",
                point.position.latitude().degrees(),
                vertex.latitude().degrees()
            );
            distance += 50.0;
        }
    }

    #[test]
    fn waypoints_span_the_track() {
        let from = at(49.95, -5.2);
        let to = at(46.66, -53.07);
        let total = great_circle(from, to).unwrap().distance;
        let mut buffer = [from; 32];
        let written = great_circle_waypoints_into(
            from,
            to,
            Distance::from_nautical_miles(300.0).unwrap(),
            &mut buffer,
        )
        .unwrap();
        let waypoints = &buffer[..written];

        assert!(waypoints.len() >= 2);
        assert!(waypoints.first().unwrap().latitude().degrees() - from.latitude().degrees() < 1e-9);
        let last = waypoints.last().unwrap();
        assert!(great_circle(*last, to).unwrap().distance.nautical_miles() < 1e-6);

        // Every leg within the interval; lengths sum to the total.
        let mut sum = 0.0;
        for pair in waypoints.windows(2) {
            let leg = great_circle(pair[0], pair[1]).unwrap();
            assert!(leg.distance.nautical_miles() <= 300.0 + 1e-6);
            sum += leg.distance.nautical_miles();
        }
        assert!((sum - total.nautical_miles()).abs() < 1e-6);
    }

    #[test]
    fn waypoints_reject_a_useless_interval() {
        let from = at(0.0, 0.0);
        let to = at(10.0, 10.0);
        let mut buffer = [from; 32];
        for interval in [Distance::ZERO, nm(-5.0), nm(1e-9)] {
            assert!(great_circle_waypoints_into(from, to, interval, &mut buffer).is_err());
        }

        // A buffer too small is reported, not overrun.
        let mut cramped = [from; 2];
        assert!(matches!(
            great_circle_waypoints_into(from, to, nm(100.0), &mut cramped),
            Err(NavigationError::Kernel(KernelError::BufferTooSmall { .. }))
        ));
    }

    /// Distance in NM, test helper.
    fn nm(miles: f64) -> Distance {
        Distance::from_nautical_miles(miles).unwrap()
    }

    #[test]
    fn intermediate_points_lie_on_the_track() {
        let from = at(10.0, 20.0);
        let to = at(-30.0, 100.0);
        let total = great_circle(from, to).unwrap().distance.nautical_miles();

        for fraction in [0.0, 0.25, 0.5, 0.75, 1.0] {
            let point = great_circle_intermediate(from, to, fraction).unwrap();
            let travelled = great_circle(from, point).unwrap().distance.nautical_miles();
            assert!((travelled - total * fraction).abs() < 1e-6, "at {fraction}");
        }
    }

    #[test]
    fn intersection_of_two_meridians_is_a_pole() {
        // Two meridians going north meet at the north pole.
        let crossing = intersection(
            at(10.0, 0.0),
            TrueCourse::NORTH,
            at(10.0, 90.0),
            TrueCourse::NORTH,
        )
        .unwrap();
        assert!((crossing.latitude().degrees() - 90.0).abs() < 1e-6);
    }

    #[test]
    fn intersection_of_a_meridian_and_the_equator() {
        let crossing = intersection(
            at(-10.0, 30.0),
            TrueCourse::NORTH,
            at(0.0, 0.0),
            TrueCourse::EAST,
        )
        .unwrap();
        assert!(crossing.latitude().degrees().abs() < 1e-6);
        assert!((crossing.longitude().degrees() - 30.0).abs() < 1e-6);
    }

    #[test]
    fn parallel_great_circles_do_not_intersect() {
        let result = intersection(
            at(0.0, 0.0),
            TrueCourse::EAST,
            at(0.0, 40.0),
            TrueCourse::EAST,
        );
        assert!(matches!(
            result.unwrap_err(),
            NavigationError::Parallel { .. }
        ));
    }

    #[test]
    fn cross_track_is_zero_on_the_track() {
        let start = at(0.0, 0.0);
        let end = at(0.0, 10.0);
        let on_track = at(0.0, 5.0);
        let result = cross_track(on_track, start, end).unwrap();
        assert!(result.distance.nautical_miles() < 1e-6);
        assert_eq!(result.side, TrackSide::OnTrack);
        assert!((result.along_track.nautical_miles() - 300.4).abs() < 1.0);
    }

    #[test]
    fn cross_track_knows_which_side_it_is_on() {
        let start = at(0.0, 0.0);
        let end = at(0.0, 10.0); // steering due east
        let to_the_north = at(1.0, 5.0);
        let to_the_south = at(-1.0, 5.0);

        let north = cross_track(to_the_north, start, end).unwrap();
        // North of an eastbound track is port.
        assert_eq!(north.side, TrackSide::Port);
        assert!((north.distance.nautical_miles() - 60.0).abs() < 0.2);
        assert!(north.signed().nautical_miles() < 0.0);

        let south = cross_track(to_the_south, start, end).unwrap();
        assert_eq!(south.side, TrackSide::Starboard);
        assert!(south.signed().nautical_miles() > 0.0);
    }

    #[test]
    fn cross_track_along_and_to_run_add_up() {
        let start = at(50.0, -5.0);
        let end = at(50.0, -10.0);
        let leg = great_circle(start, end).unwrap().distance.nautical_miles();
        let position = at(50.2, -7.5);
        let result = cross_track(position, start, end).unwrap();
        assert!(
            (result.along_track.nautical_miles() + result.to_run.nautical_miles() - leg).abs()
                < 1e-6
        );
    }

    #[test]
    fn cross_track_can_be_behind_the_start() {
        let start = at(0.0, 0.0);
        let end = at(0.0, 10.0);
        let behind = at(0.0, -2.0);
        let result = cross_track(behind, start, end).unwrap();
        assert!(result.along_track.is_negative());
    }

    #[test]
    fn cross_track_needs_a_leg_with_length() {
        let point = at(1.0, 1.0);
        assert!(cross_track(point, at(0.0, 0.0), at(0.0, 0.0)).is_err());
    }

    #[test]
    fn geodesic_matches_the_published_vincenty_test_case() {
        // Vincenty's worked example: Flinders Peak 37°57'03.72030"S
        // 144°25'29.52440"E to Buninyong 37°39'10.15610"S 143°55'35.38390"E. On
        // WGS-84: 54 972.271 m, initial 306°52'05.37". Vincenty tabulates the
        // reverse azimuth 127°10'25.07" at the far end; the course on arrival
        // is its reciprocal, 307°10'25.07".
        let from = Position::from_degrees(-37.951_033_416_7, 144.424_867_888_9).unwrap();
        let to = Position::from_degrees(-37.652_821_138_9, 143.926_495_527_8).unwrap();
        let sailing = geodesic(from, to).unwrap();

        assert!(
            (sailing.distance.metres() - 54_972.271).abs() < 0.001,
            "{} m",
            sailing.distance.metres()
        );

        let initial = 306.0 + 52.0 / 60.0 + 5.37 / 3600.0;
        let reverse_azimuth = 127.0 + 10.0 / 60.0 + 25.07 / 3600.0;
        assert!(
            (sailing.initial_course.degrees() - initial).abs() < 1e-6,
            "initial {}",
            sailing.initial_course.degrees()
        );
        assert!(
            (sailing.final_course.reciprocal().degrees() - reverse_azimuth).abs() < 1e-6,
            "final {}",
            sailing.final_course.degrees()
        );
    }

    #[test]
    fn geodesic_round_trips_through_its_destination() {
        for (from, course, metres) in [
            (at(0.0, 0.0), 45.0, 1_000_000.0),
            (at(50.0, -5.0), 250.0, 3_000_000.0),
            (at(-20.0, 170.0), 300.0, 500_000.0),
            (at(60.0, 10.0), 180.0, 100_000.0),
        ] {
            let course = TrueCourse::new(course).unwrap();
            let distance = Distance::from_metres(metres).unwrap();
            let arrival = geodesic_destination(from, course, distance).unwrap();
            let back = geodesic(from, arrival.position).unwrap();

            assert!(
                (back.distance.metres() - metres).abs() < 1e-6,
                "{} vs {metres}",
                back.distance.metres()
            );
            assert!(back.initial_course.angular_distance(course) < 1e-9);
            assert!(back.final_course.angular_distance(arrival.final_course) < 1e-9);
        }
    }

    #[test]
    fn geodesic_and_great_circle_agree_to_within_the_flattening() {
        for (from, to) in [
            (at(49.95, -5.2), at(46.66, -53.07)),
            (at(0.0, 0.0), at(0.0, 40.0)),
            (at(-33.9, 151.2), at(35.0, 139.0)),
        ] {
            let sphere = great_circle(from, to).unwrap().distance.nautical_miles();
            let ellipsoid = geodesic(from, to).unwrap().distance.nautical_miles();
            let relative = (sphere - ellipsoid).abs() / ellipsoid;
            assert!(
                relative < 0.006,
                "{relative} between {sphere} and {ellipsoid}"
            );
        }
    }

    #[test]
    fn coincident_positions_have_no_distance() {
        let point = at(12.34, -56.78);
        assert_eq!(great_circle(point, point).unwrap().distance, Distance::ZERO);
        assert_eq!(rhumb_line(point, point).unwrap().distance, Distance::ZERO);
        assert_eq!(geodesic(point, point).unwrap().distance, Distance::ZERO);
    }

    #[test]
    fn tracks_across_the_antimeridian_take_the_short_way() {
        let from = at(0.0, 179.0);
        let to = at(0.0, -179.0);
        let sailing = great_circle(from, to).unwrap();
        assert!((sailing.distance.nautical_miles() - 120.0).abs() < 0.2);
        assert!((sailing.initial_course.degrees() - 90.0).abs() < 1e-9);

        let steered = rhumb_line(from, to).unwrap();
        assert!((steered.distance.nautical_miles() - 120.0).abs() < 0.2);

        let destination = rhumb_destination(
            from,
            TrueCourse::EAST,
            Distance::from_nautical_miles(120.0).unwrap(),
        )
        .unwrap();
        assert!(destination.longitude().degrees() < 0.0);
    }

    #[test]
    fn hostile_distances_are_errors_not_panics() {
        let from = at(10.0, 10.0);
        for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let distance = Distance::from_nautical_miles(value);
            assert!(distance.is_err());
        }
        // An oversized distance yields a position, not a panic.
        let far = Distance::from_nautical_miles(1e9).unwrap();
        assert!(great_circle_destination(from, TrueCourse::EAST, far).is_ok());
        assert!(geodesic_destination(from, TrueCourse::EAST, far).is_ok());
    }
}
