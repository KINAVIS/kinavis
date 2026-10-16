//! Outer boundary with holes.

use super::offset::offset_rings;
use super::polygon::{
    checked_offset, offset_piece, offset_tolerance, rings_meet, signed_area, Polygon,
};
use super::{Area, Containment, PlanarPoint};
use crate::error::{KernelError, NavigationError, Result};
use crate::inline::Inline;
use crate::units::Distance;

/// Outer boundary of at most `N` vertices with up to `K` holes of at most `H`
/// vertices each: a field and its obstacles, a zone and its exclusions.
///
/// Validated by [`Region::new`]: every hole lies strictly inside the boundary
/// and the holes neither touch nor overlap one another.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(try_from = "StoredRegion", into = "StoredRegion")
)]
pub struct Region<const N: usize, const H: usize, const K: usize> {
    outer: Polygon<N>,
    holes: Inline<Polygon<H>, K>,
}

impl<const N: usize, const H: usize, const K: usize> Region<N, H, K> {
    /// Region of a boundary and its holes.
    ///
    /// # Errors
    ///
    /// - [`KernelError::CapacityExceeded`] for more than `K` holes;
    /// - [`NavigationError::HoleOutside`] if a hole reaches or crosses the
    ///   boundary;
    /// - [`NavigationError::HolesOverlap`] if two holes touch or overlap.
    pub fn new(outer: Polygon<N>, holes: &[Polygon<H>]) -> Result<Self> {
        let mut stored: Inline<Polygon<H>, K> = Inline::new(Polygon::PLACEHOLDER);
        for hole in holes {
            stored
                .push(*hole)
                .map_err(|_| KernelError::CapacityExceeded {
                    context: "the holes of a region",
                    needed: holes.len(),
                    capacity: K,
                })?;
        }
        for (index, hole) in holes.iter().enumerate() {
            let inside = hole
                .vertices()
                .iter()
                .all(|v| outer.contains(*v) == Containment::Inside);
            if !inside || rings_meet(outer.vertices(), hole.vertices()) {
                return Err(NavigationError::HoleOutside { hole: index });
            }
        }
        for (first, a) in holes.iter().enumerate() {
            for (second, b) in holes.iter().enumerate().skip(first.saturating_add(1)) {
                if rings_meet(a.vertices(), b.vertices())
                    || a.vertices()
                        .first()
                        .is_some_and(|v| b.contains(*v) != Containment::Outside)
                    || b.vertices()
                        .first()
                        .is_some_and(|v| a.contains(*v) != Containment::Outside)
                {
                    return Err(NavigationError::HolesOverlap { first, second });
                }
            }
        }
        Ok(Self {
            outer,
            holes: stored,
        })
    }

    /// Region without holes.
    #[must_use]
    pub fn without_holes(outer: Polygon<N>) -> Self {
        Self {
            outer,
            holes: Inline::new(Polygon::PLACEHOLDER),
        }
    }

    /// Outer boundary.
    #[must_use]
    pub const fn outer(&self) -> &Polygon<N> {
        &self.outer
    }

    /// Holes.
    #[must_use]
    pub fn holes(&self) -> &[Polygon<H>] {
        self.holes.as_slice()
    }

    /// Area inside the boundary and outside every hole.
    #[must_use]
    pub fn area(&self) -> Area {
        let holes: f64 = self.holes().iter().map(|h| h.area().square_metres()).sum();
        Area::from_square_metres_unchecked((self.outer.area().square_metres() - holes).max(0.0))
    }

    /// Length of the boundary and of every hole's edge.
    #[must_use]
    pub fn perimeter(&self) -> Distance {
        let metres: f64 = self.outer.perimeter().metres()
            + self
                .holes()
                .iter()
                .map(|h| h.perimeter().metres())
                .sum::<f64>();
        Distance::from_metres(metres).unwrap_or(Distance::ZERO)
    }

    /// Where a point lies; a point inside a hole is outside the region.
    #[must_use]
    pub fn contains(&self, point: PlanarPoint) -> Containment {
        match self.outer.contains(point) {
            Containment::Inside => {}
            other => return other,
        }
        for hole in self.holes() {
            match hole.contains(point) {
                Containment::Outside => {}
                Containment::OnBoundary => return Containment::OnBoundary,
                Containment::Inside => return Containment::Outside,
            }
        }
        Containment::Inside
    }

    /// Distance from a point to the nearest edge of the boundary or a hole.
    #[must_use]
    pub fn distance_to_boundary(&self, point: PlanarPoint) -> Distance {
        let nearest = self.closest_boundary_point(point);
        point.distance_to(nearest)
    }

    /// Point of the boundary or of a hole nearest to a point.
    #[must_use]
    pub fn closest_boundary_point(&self, point: PlanarPoint) -> PlanarPoint {
        let outer = self.outer.closest_boundary_point(point);
        self.holes()
            .iter()
            .map(|h| h.closest_boundary_point(point))
            .fold(outer, |best, candidate| {
                if point.distance_to(candidate) < point.distance_to(best) {
                    candidate
                } else {
                    best
                }
            })
    }

    /// Region shrunk by a distance: the boundary moved inward, every hole
    /// grown, all corners mitred. A headland of that width is what is
    /// removed.
    ///
    /// Boundary and holes move as one straight-skeleton wavefront, as
    /// [`Polygon::inset`]: a hole that reaches the boundary becomes a notch in
    /// it, two holes that meet become one, and parts that pinch off and
    /// vanish are gone.
    ///
    /// The front holds the boundary's and the holes' vertices together, and
    /// takes one more per parting or joining, all within `N`.
    ///
    /// # Errors
    ///
    /// - [`KernelError::OutOfRange`] for a negative distance;
    /// - [`NavigationError::OffsetCollapses`] if nothing is left;
    /// - [`NavigationError::OffsetSplits`] if more than one part is left;
    /// - [`KernelError::CapacityExceeded`] if the front outgrows `N`, a hole
    ///   left outgrows `H` or more than `K` holes are left.
    pub fn inset(&self, distance: Distance) -> Result<Self> {
        let metres = checked_offset(distance)?;
        if metres == 0.0 {
            return Ok(*self);
        }
        let rings = core::iter::once((self.outer.vertices(), false))
            .chain(self.holes().iter().map(|hole| (hole.vertices(), true)));
        let finished = offset_rings::<N>(rings, metres)?;
        let extent = self
            .holes()
            .iter()
            .map(Polygon::extent)
            .fold(self.outer.extent(), f64::max);
        let tolerance = offset_tolerance(metres, extent);
        let mut outer: Option<Polygon<N>> = None;
        let mut holes: Inline<Polygon<H>, K> = Inline::new(Polygon::PLACEHOLDER);
        finished.for_each_ring(|ring| {
            if signed_area(ring) > 0.0 {
                let Some(piece) = offset_piece::<N>(ring, metres, tolerance)? else {
                    return Ok(());
                };
                if outer.is_some() {
                    return Err(NavigationError::OffsetSplits { distance: metres });
                }
                outer = Some(piece);
            } else if let Some(hole) = offset_piece::<H>(ring, metres, tolerance)? {
                holes
                    .push(hole)
                    .map_err(|_| KernelError::CapacityExceeded {
                        context: "the holes of an inset region",
                        needed: K.saturating_add(1),
                        capacity: K,
                    })?;
            }
            Ok(())
        })?;
        let outer = outer.ok_or(NavigationError::OffsetCollapses { distance: metres })?;
        let result = Self::new(outer, holes.as_slice()).map_err(|error| match error {
            NavigationError::HoleOutside { .. } | NavigationError::HolesOverlap { .. } => {
                NavigationError::OffsetSplits { distance: metres }
            }
            other => other,
        })?;
        let keeps_clear = result.rings().flatten().all(|vertex| {
            self.contains(*vertex) == Containment::Inside
                && self.distance_to_boundary(*vertex).metres() >= metres - tolerance
        });
        // Vertices inside are not enough: a skeleton thrown off by rounding
        // can span a notch of the boundary with one edge.
        let crosses = result
            .rings()
            .any(|ring| self.rings().any(|own| rings_meet(ring, own)));
        if keeps_clear && !crosses {
            Ok(result)
        } else {
            Err(NavigationError::OffsetSplits { distance: metres })
        }
    }

    /// Outer ring, then the holes.
    fn rings(&self) -> impl Iterator<Item = &[PlanarPoint]> + '_ {
        core::iter::once(self.outer.vertices()).chain(self.holes().iter().map(Polygon::vertices))
    }
}

#[cfg(feature = "serde")]
#[derive(serde::Serialize, serde::Deserialize)]
struct StoredRegion {
    outer: alloc::vec::Vec<PlanarPoint>,
    holes: alloc::vec::Vec<alloc::vec::Vec<PlanarPoint>>,
}

#[cfg(feature = "serde")]
impl<const N: usize, const H: usize, const K: usize> TryFrom<StoredRegion> for Region<N, H, K> {
    type Error = NavigationError;

    /// Deserialised through [`Polygon::new`] and [`Region::new`].
    fn try_from(stored: StoredRegion) -> Result<Self> {
        let outer = Polygon::new(&stored.outer)?;
        let holes = stored
            .holes
            .iter()
            .map(|ring| Polygon::new(ring))
            .collect::<Result<alloc::vec::Vec<_>>>()?;
        Self::new(outer, &holes)
    }
}

#[cfg(feature = "serde")]
impl<const N: usize, const H: usize, const K: usize> From<Region<N, H, K>> for StoredRegion {
    fn from(region: Region<N, H, K>) -> Self {
        Self {
            outer: region.outer.vertices().to_vec(),
            holes: region
                .holes()
                .iter()
                .map(|h| h.vertices().to_vec())
                .collect(),
        }
    }
}
