//! A field: a region with holes in a field plane.

use kinavis::region::{Containment, PlanarPoint, Polygon, Region};
use kinavis_kernel::inline::Inline;
use kinavis_kernel::{math, Distance, GridCourse, KernelError, Position};

use crate::error::Result;
use crate::plane::FieldPlane;

/// Field: a boundary of at most `N` vertices and up to `K` obstacles of at
/// most `H` vertices each, in the plane it was laid out in.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Field<const N: usize, const H: usize, const K: usize> {
    plane: FieldPlane,
    region: Region<N, H, K>,
}

impl<const N: usize, const H: usize, const K: usize> Field<N, H, K> {
    /// Field from a region already in `plane`.
    #[must_use]
    pub const fn new(plane: FieldPlane, region: Region<N, H, K>) -> Self {
        Self { plane, region }
    }

    /// Field from surveyed positions: the boundary and the obstacles, each a
    /// ring in either order.
    ///
    /// # Errors
    ///
    /// As [`FieldPlane::to_plane`], [`Polygon::new`] and [`Region::new`];
    /// [`KernelError::CapacityExceeded`] for more than `K` obstacles.
    pub fn from_positions(
        plane: FieldPlane,
        boundary: &[Position],
        obstacles: &[&[Position]],
    ) -> Result<Self> {
        let outer = polygon::<N>(&plane, boundary)?;
        let mut holes = Inline::<Polygon<H>, K>::new(placeholder::<H>()?);
        for obstacle in obstacles {
            holes.push(polygon::<H>(&plane, obstacle)?).map_err(|_| {
                KernelError::CapacityExceeded {
                    context: "the obstacles of a field",
                    needed: obstacles.len(),
                    capacity: K,
                }
            })?;
        }
        Ok(Self {
            plane,
            region: Region::new(outer, holes.as_slice())?,
        })
    }

    /// Plane.
    #[must_use]
    pub const fn plane(&self) -> &FieldPlane {
        &self.plane
    }

    /// Region in the plane.
    #[must_use]
    pub const fn region(&self) -> &Region<N, H, K> {
        &self.region
    }

    /// Distance from a point inside the field along a straight course to the
    /// first boundary or obstacle it meets; `None` from a point outside.
    #[must_use]
    pub fn distance_to_exit(&self, from: PlanarPoint, course: GridCourse) -> Option<Distance> {
        if self.region.contains(from) == Containment::Outside {
            return None;
        }
        let (north, east) = course.components(1.0);
        let mut nearest = f64::INFINITY;
        let rings = core::iter::once(self.region.outer().vertices())
            .chain(self.region.holes().iter().map(Polygon::vertices));
        for ring in rings {
            for (a, b) in ring.iter().zip(ring.iter().cycle().skip(1)) {
                if let Some(t) = ray_hits(from, east, north, *a, *b) {
                    nearest = nearest.min(t);
                }
            }
        }
        if nearest.is_finite() {
            Distance::from_metres(nearest).ok()
        } else {
            None
        }
    }
}

/// Distance along the unit ray from `origin` to segment `a`–`b`, if it is met
/// ahead.
fn ray_hits(
    origin: PlanarPoint,
    east: f64,
    north: f64,
    a: PlanarPoint,
    b: PlanarPoint,
) -> Option<f64> {
    let (ax, ay) = (
        a.east_metres() - origin.east_metres(),
        a.north_metres() - origin.north_metres(),
    );
    let (sx, sy) = (
        b.east_metres() - a.east_metres(),
        b.north_metres() - a.north_metres(),
    );
    let denominator = east * sy - north * sx;
    if math::abs(denominator) <= f64::EPSILON * math::hypot(sx, sy) {
        return None;
    }
    // origin + t·ray = a + u·segment
    let t = (ax * sy - ay * sx) / denominator;
    let u = (ax * north - ay * east) / denominator;
    (t >= 0.0 && (0.0..=1.0).contains(&u)).then_some(t)
}

fn polygon<const M: usize>(plane: &FieldPlane, ring: &[Position]) -> Result<Polygon<M>> {
    let mut points = Inline::<PlanarPoint, M>::new(PlanarPoint::ORIGIN);
    for position in ring {
        points
            .push(plane.to_plane(*position)?)
            .map_err(|_| KernelError::CapacityExceeded {
                context: "a field boundary",
                needed: ring.len(),
                capacity: M,
            })?;
    }
    Ok(Polygon::new(points.as_slice())?)
}

/// Fill value for the obstacle store; never handed out.
fn placeholder<const M: usize>() -> Result<Polygon<M>> {
    Ok(Polygon::new(&[
        PlanarPoint::ORIGIN,
        PlanarPoint::from_metres(1.0, 0.0)?,
        PlanarPoint::from_metres(0.0, 1.0)?,
    ])?)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn square() -> Result<Field<8, 8, 2>> {
        let plane = crate::FieldPlane::centred_at(
            Position::from_degrees(50.0, 10.0)?,
            kinavis_kernel::Height::above_ellipsoid(Distance::ZERO),
        )?;
        let corners = [(0.0, 0.0), (100.0, 0.0), (100.0, 100.0), (0.0, 100.0)];
        let mut outer = [PlanarPoint::ORIGIN; 4];
        for (slot, (e, n)) in outer.iter_mut().zip(corners) {
            *slot = PlanarPoint::from_metres(e, n)?;
        }
        let hole = Polygon::new(&[
            PlanarPoint::from_metres(40.0, 40.0)?,
            PlanarPoint::from_metres(60.0, 40.0)?,
            PlanarPoint::from_metres(60.0, 60.0)?,
            PlanarPoint::from_metres(40.0, 60.0)?,
        ])?;
        Ok(Field::new(
            plane,
            Region::new(Polygon::new(&outer)?, &[hole])?,
        ))
    }

    #[test]
    fn the_exit_is_the_nearest_edge_ahead() -> Result<()> {
        let field = square()?;
        let at = PlanarPoint::from_metres(10.0, 50.0)?;
        let east = GridCourse::new(90.0)?;
        let to_hole = field.distance_to_exit(at, east).map(Distance::metres);
        assert!(to_hole.is_some_and(|d| (d - 30.0).abs() < 1e-9));
        let north = GridCourse::new(0.0)?;
        let to_edge = field.distance_to_exit(at, north).map(Distance::metres);
        assert!(to_edge.is_some_and(|d| (d - 50.0).abs() < 1e-9));
        let outside = PlanarPoint::from_metres(-5.0, 50.0)?;
        assert_eq!(field.distance_to_exit(outside, east), None);
        Ok(())
    }
}
