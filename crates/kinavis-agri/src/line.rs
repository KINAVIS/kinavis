//! Reference lines: AB lines and pivot circles; curves are in `curve`.

use kinavis::error::NavigationError;
use kinavis::region::PlanarPoint;
use kinavis_kernel::error::ensure_range;
use kinavis_kernel::{math, Distance, GridCourse, Position, TrueCourse};

use crate::error::Result;
use crate::plane::FieldPlane;

/// Shortest AB line and smallest pivot radius, metres.
pub const MIN_LINE_METRES: f64 = 1.0;

/// Where a point lies relative to a reference line.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Station {
    /// Signed distance from the reference line, left positive looking along
    /// its reference direction.
    pub offset: Distance,
    /// Reference direction of the line, and of every pass parallel to it, at
    /// the point.
    pub direction: GridCourse,
}

/// Line that parallel passes are laid out from.
///
/// A role, implemented by [`AbLine`], [`crate::CurveLine`] and [`PivotLine`]:
/// the passes of a line are the curves of constant [`Station::offset`] from
/// it, which is what the pass tracker follows.
pub trait ReferenceLine {
    /// Plane the line was laid out in.
    fn plane(&self) -> &FieldPlane;

    /// Offset and pass direction at a point.
    fn station(&self, point: PlanarPoint) -> Station;

    /// Whether passes are closed loops with no end.
    fn is_closed(&self) -> bool {
        false
    }
}

/// Straight line through A in a fixed direction, extended both ways.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(try_from = "StoredAbLine", into = "StoredAbLine")
)]
pub struct AbLine {
    plane: FieldPlane,
    origin: PlanarPoint,
    east: f64,
    north: f64,
}

impl AbLine {
    /// Line from A through B.
    ///
    /// # Errors
    ///
    /// [`NavigationError::Degenerate`] if A and B are closer than
    /// [`MIN_LINE_METRES`].
    pub fn from_points(plane: FieldPlane, a: PlanarPoint, b: PlanarPoint) -> Result<Self> {
        let (east, north) = (
            b.east_metres() - a.east_metres(),
            b.north_metres() - a.north_metres(),
        );
        let length = math::hypot(east, north);
        if !length.is_finite() || length < MIN_LINE_METRES {
            return Err(NavigationError::Degenerate {
                context: "an AB line shorter than 1 m",
            }
            .into());
        }
        Ok(Self {
            plane,
            origin: a,
            east: east / length,
            north: north / length,
        })
    }

    /// Line through A on a grid course.
    #[must_use]
    pub fn from_point_and_course(plane: FieldPlane, a: PlanarPoint, course: GridCourse) -> Self {
        let (north, east) = course.components(1.0);
        Self {
            plane,
            origin: a,
            east,
            north,
        }
    }

    /// Line from surveyed A and B positions.
    ///
    /// # Errors
    ///
    /// As [`FieldPlane::to_plane`] and [`AbLine::from_points`].
    pub fn from_positions(plane: FieldPlane, a: Position, b: Position) -> Result<Self> {
        Self::from_points(plane, plane.to_plane(a)?, plane.to_plane(b)?)
    }

    /// Line through position A on a true course, turned to grid at A.
    ///
    /// # Errors
    ///
    /// As [`FieldPlane::to_plane`].
    pub fn from_position_and_true_course(
        plane: FieldPlane,
        a: Position,
        course: TrueCourse,
    ) -> Result<Self> {
        let origin = plane.to_plane(a)?;
        Ok(Self::from_point_and_course(
            plane,
            origin,
            plane.grid_course(course, origin)?,
        ))
    }

    /// Point A.
    #[must_use]
    pub const fn origin(&self) -> PlanarPoint {
        self.origin
    }

    /// Reference direction, A towards B.
    #[must_use]
    pub fn course(&self) -> GridCourse {
        course_of(self.east, self.north)
    }
}

impl ReferenceLine for AbLine {
    fn plane(&self) -> &FieldPlane {
        &self.plane
    }

    fn station(&self, point: PlanarPoint) -> Station {
        let dx = point.east_metres() - self.origin.east_metres();
        let dy = point.north_metres() - self.origin.north_metres();
        Station {
            offset: metres(self.east * dy - self.north * dx),
            direction: self.course(),
        }
    }
}

/// Circle of a centre-pivot irrigator: passes are concentric circles, the
/// reference direction is clockwise, so the left side is outward and pass +1
/// lies outside the reference circle.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(try_from = "StoredPivotLine", into = "StoredPivotLine")
)]
pub struct PivotLine {
    plane: FieldPlane,
    centre: PlanarPoint,
    radius: f64,
}

impl PivotLine {
    /// Circle about a centre.
    ///
    /// # Errors
    ///
    /// [`kinavis_kernel::KernelError::OutOfRange`] for a radius below
    /// [`MIN_LINE_METRES`] or beyond 100 km.
    pub fn new(plane: FieldPlane, centre: PlanarPoint, radius: Distance) -> Result<Self> {
        Self::with_radius(plane, centre, radius.metres())
    }

    fn with_radius(plane: FieldPlane, centre: PlanarPoint, radius: f64) -> Result<Self> {
        ensure_range("pivot radius, metres", radius, MIN_LINE_METRES, 1e5)?;
        Ok(Self {
            plane,
            centre,
            radius,
        })
    }

    /// Circle about a surveyed centre through a surveyed point.
    ///
    /// # Errors
    ///
    /// As [`FieldPlane::to_plane`] and [`PivotLine::new`].
    pub fn from_positions(
        plane: FieldPlane,
        centre: Position,
        on_circle: Position,
    ) -> Result<Self> {
        let centre = plane.to_plane(centre)?;
        let radius = centre.distance_to(plane.to_plane(on_circle)?);
        Self::new(plane, centre, radius)
    }

    /// Centre.
    #[must_use]
    pub const fn centre(&self) -> PlanarPoint {
        self.centre
    }

    /// Radius of the reference circle.
    #[must_use]
    pub fn radius(&self) -> Distance {
        metres(self.radius)
    }
}

impl ReferenceLine for PivotLine {
    fn plane(&self) -> &FieldPlane {
        &self.plane
    }

    fn station(&self, point: PlanarPoint) -> Station {
        let vx = point.east_metres() - self.centre.east_metres();
        let vy = point.north_metres() - self.centre.north_metres();
        let r = math::hypot(vx, vy);
        // Clockwise tangent of the radius vector; due east at the centre.
        let direction = if r > 0.0 {
            course_of(vy, -vx)
        } else {
            course_of(1.0, 0.0)
        };
        Station {
            offset: metres(r - self.radius),
            direction,
        }
    }

    fn is_closed(&self) -> bool {
        true
    }
}

/// Grid course of a direction given by its east and north components.
pub(crate) fn course_of(east: f64, north: f64) -> GridCourse {
    GridCourse::from_degrees_wrapped(math::to_degrees(math::atan2(east, north)))
}

/// Metres as a distance; non-finite input, unreachable from validated
/// geometry, maps to zero.
pub(crate) fn metres(value: f64) -> Distance {
    Distance::from_metres(value).unwrap_or(Distance::ZERO)
}

/// Serialised form: A and the unit direction towards B.
#[cfg(feature = "serde")]
#[derive(Clone, Copy, serde::Serialize, serde::Deserialize)]
struct StoredAbLine {
    plane: FieldPlane,
    origin: PlanarPoint,
    /// East and north components.
    direction: [f64; 2],
}

#[cfg(feature = "serde")]
impl TryFrom<StoredAbLine> for AbLine {
    type Error = crate::error::AgriError;

    /// The direction must be a unit vector to a billionth.
    fn try_from(stored: StoredAbLine) -> Result<Self> {
        let [east, north] = stored.direction;
        ensure_range(
            "length of an AB line's direction",
            math::hypot(east, north),
            1.0 - 1e-9,
            1.0 + 1e-9,
        )?;
        Ok(Self {
            plane: stored.plane,
            origin: stored.origin,
            east,
            north,
        })
    }
}

#[cfg(feature = "serde")]
impl From<AbLine> for StoredAbLine {
    fn from(line: AbLine) -> Self {
        Self {
            plane: line.plane,
            origin: line.origin,
            direction: [line.east, line.north],
        }
    }
}

/// Serialised form, in metres.
#[cfg(feature = "serde")]
#[derive(Clone, Copy, serde::Serialize, serde::Deserialize)]
struct StoredPivotLine {
    plane: FieldPlane,
    centre: PlanarPoint,
    radius_metres: f64,
}

#[cfg(feature = "serde")]
impl TryFrom<StoredPivotLine> for PivotLine {
    type Error = crate::error::AgriError;

    /// Checked as [`PivotLine::new`] checks.
    fn try_from(stored: StoredPivotLine) -> Result<Self> {
        Self::with_radius(stored.plane, stored.centre, stored.radius_metres)
    }
}

#[cfg(feature = "serde")]
impl From<PivotLine> for StoredPivotLine {
    fn from(line: PivotLine) -> Self {
        Self {
            plane: line.plane,
            centre: line.centre,
            radius_metres: line.radius,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use kinavis_kernel::Height;

    fn plane() -> Result<FieldPlane> {
        FieldPlane::centred_at(
            Position::from_degrees(45.0, 30.0)?,
            Height::above_ellipsoid(Distance::from_metres(120.0)?),
        )
    }

    #[test]
    fn left_of_an_ab_line_is_positive() -> Result<()> {
        let line = AbLine::from_points(
            plane()?,
            PlanarPoint::from_metres(0.0, 0.0)?,
            PlanarPoint::from_metres(100.0, 100.0)?,
        )?;
        let station = line.station(PlanarPoint::from_metres(0.0, 10.0)?);
        assert!((station.offset.metres() - 10.0 / 2f64.sqrt()).abs() < 1e-9);
        assert!((station.direction.degrees() - 45.0).abs() < 1e-12);
        assert!(AbLine::from_points(plane()?, PlanarPoint::ORIGIN, PlanarPoint::ORIGIN).is_err());
        Ok(())
    }

    #[test]
    fn outward_of_a_pivot_is_positive_and_travel_clockwise() -> Result<()> {
        let pivot = PivotLine::new(plane()?, PlanarPoint::ORIGIN, Distance::from_metres(50.0)?)?;
        let north = pivot.station(PlanarPoint::from_metres(0.0, 60.0)?);
        assert!((north.offset.metres() - 10.0).abs() < 1e-12);
        assert!((north.direction.degrees() - 90.0).abs() < 1e-12);
        let west = pivot.station(PlanarPoint::from_metres(-45.0, 0.0)?);
        assert!((west.offset.metres() + 5.0).abs() < 1e-12);
        assert!(west.direction.degrees().abs() < 1e-12);
        assert!(pivot.is_closed());
        Ok(())
    }
}
