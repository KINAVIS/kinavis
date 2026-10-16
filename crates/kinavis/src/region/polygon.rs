//! Simple polygon of at most `N` vertices.

use super::offset::{offset_rings, Side};
use super::{Area, Containment, PlanarPoint};
use crate::error::{KernelError, NavigationError, Result};
use crate::inline::Inline;
use crate::math;
use crate::units::Distance;

/// Simple polygon in a plane, at most `N` vertices.
///
/// Validated by [`Polygon::new`]: at least three vertices, no repeated
/// consecutive vertex, no edge crossing or touching a non-adjacent edge,
/// positive area. Vertices are stored counter-clockwise.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(try_from = "StoredPolygon", into = "StoredPolygon")
)]
pub struct Polygon<const N: usize> {
    vertices: Inline<PlanarPoint, N>,
}

impl<const N: usize> Polygon<N> {
    /// Fill value of fixed-capacity stores of polygons; never handed out.
    pub(super) const PLACEHOLDER: Self = Self {
        vertices: Inline::new(PlanarPoint::ORIGIN),
    };

    /// Polygon through the vertices, in either order.
    ///
    /// A closing vertex equal to the first, as `GeoJSON` and shapefiles write
    /// rings, is dropped.
    ///
    /// # Errors
    ///
    /// - [`KernelError::InsufficientData`] for fewer than three vertices;
    /// - [`KernelError::CapacityExceeded`] for more than `N`;
    /// - [`KernelError::OutOfRange`] for a coordinate beyond ±10⁹ m;
    /// - [`NavigationError::Degenerate`] for a repeated consecutive vertex or
    ///   zero area;
    /// - [`NavigationError::SelfIntersecting`] if two edges cross or touch.
    pub fn new(vertices: &[PlanarPoint]) -> Result<Self> {
        let ring = match vertices.split_last() {
            Some((last, rest)) if rest.first() == Some(last) => rest,
            _ => vertices,
        };
        if ring.len() < 3 {
            return Err(KernelError::InsufficientData {
                found: ring.len(),
                required: 3,
                context: "a polygon",
            }
            .into());
        }
        let mut stored = Inline::new(PlanarPoint::ORIGIN);
        for vertex in ring {
            for value in vertex.xy() {
                if math::abs(value) > MAX_COORDINATE {
                    return Err(KernelError::OutOfRange {
                        parameter: "planar coordinate, metres",
                        value,
                        min: -MAX_COORDINATE,
                        max: MAX_COORDINATE,
                    }
                    .into());
                }
            }
            stored
                .push(*vertex)
                .map_err(|_| KernelError::CapacityExceeded {
                    context: "a polygon",
                    needed: ring.len(),
                    capacity: N,
                })?;
        }
        if edges(&stored).any(|(a, b)| a == b) {
            return Err(NavigationError::Degenerate {
                context: "a polygon with a repeated vertex",
            });
        }
        // Orientation first, keeping the first vertex, so that every check
        // below runs on the ring as stored: reading back what was written
        // repeats the same arithmetic and reaches the same verdict.
        let signed = signed_area(&stored);
        let span = span(&stored);
        if math::abs(signed) <= MIN_RELATIVE_AREA * span * span {
            return Err(NavigationError::Degenerate {
                context: "a polygon of zero area",
            });
        }
        let reversed = signed < 0.0;
        if reversed {
            if let Some((_, rest)) = stored.as_mut_slice().split_first_mut() {
                rest.reverse();
            }
        }
        check_simple(&stored).map_err(|error| match error {
            // Edge k of the reversed ring is edge n − 1 − k of the input.
            NavigationError::SelfIntersecting {
                first_edge,
                second_edge,
            } if reversed => {
                let last = stored.len().saturating_sub(1);
                NavigationError::SelfIntersecting {
                    first_edge: last.saturating_sub(second_edge),
                    second_edge: last.saturating_sub(first_edge),
                }
            }
            other => other,
        })?;
        Ok(Self { vertices: stored })
    }

    /// Vertices, counter-clockwise.
    #[must_use]
    pub fn vertices(&self) -> &[PlanarPoint] {
        self.vertices.as_slice()
    }

    /// Edges as `(start, end)`, counter-clockwise, the last closing the ring.
    pub fn edges(&self) -> impl Iterator<Item = (PlanarPoint, PlanarPoint)> + '_ {
        edges(&self.vertices)
    }

    /// Enclosed area.
    #[must_use]
    pub fn area(&self) -> Area {
        Area::from_square_metres_unchecked(math::abs(signed_area(&self.vertices)))
    }

    /// Length of the boundary.
    #[must_use]
    pub fn perimeter(&self) -> Distance {
        let metres = self
            .edges()
            .map(|(a, b)| {
                math::hypot(
                    b.east_metres() - a.east_metres(),
                    b.north_metres() - a.north_metres(),
                )
            })
            .sum();
        Distance::from_metres(metres).unwrap_or(Distance::ZERO)
    }

    /// Where a point lies: inside, on the boundary or outside.
    ///
    /// Exact for the stored coordinates: a point is on the boundary only if
    /// it lies on an edge to the last bit, as in GEOS.
    #[must_use]
    pub fn contains(&self, point: PlanarPoint) -> Containment {
        ring_contains(&self.vertices, point.xy())
    }

    /// Distance from a point, inside or outside, to the nearest edge.
    #[must_use]
    pub fn distance_to_boundary(&self, point: PlanarPoint) -> Distance {
        let (distance, _) = self.nearest(point);
        Distance::from_metres(distance).unwrap_or(Distance::ZERO)
    }

    /// Point of the boundary nearest to a point.
    #[must_use]
    pub fn closest_boundary_point(&self, point: PlanarPoint) -> PlanarPoint {
        let (_, nearest) = self.nearest(point);
        PlanarPoint::from_xy(nearest)
    }

    /// Distance in metres and the nearest boundary point.
    pub(super) fn nearest(&self, point: PlanarPoint) -> (f64, [f64; 2]) {
        let p = point.xy();
        self.edges()
            .map(|(a, b)| {
                let q = closest_on_segment(p, a.xy(), b.xy());
                (math::hypot(q[0] - p[0], q[1] - p[1]), q)
            })
            .fold((f64::INFINITY, p), |best, candidate| {
                if candidate.0 < best.0 {
                    candidate
                } else {
                    best
                }
            })
    }

    /// Polygon shrunk by a distance: every edge moved inward, corners
    /// mitred (the edges' lines extended until they meet).
    ///
    /// The edges move as the wavefront of a straight skeleton: an edge that
    /// shrinks to nothing disappears, a narrow part that pinches off parts the
    /// polygon. A part that then vanishes is gone; if more than one part is
    /// left, that is reported, as the result is not one polygon. Matches
    /// CGAL's straight-skeleton offset to micrometres. GEOS's mitred buffer
    /// differs near reflex corners by up to metres: GEOS simplifies the
    /// outline by a hundredth of the distance first.
    ///
    /// Parting the outline takes a spare vertex slot: a polygon holding `N`
    /// vertices may fail with [`KernelError::CapacityExceeded`] where one
    /// with room to spare would not.
    ///
    /// # Errors
    ///
    /// - [`KernelError::OutOfRange`] for a negative distance;
    /// - [`NavigationError::OffsetCollapses`] if nothing is left;
    /// - [`NavigationError::OffsetSplits`] if more than one part is left;
    /// - [`KernelError::CapacityExceeded`] as above.
    pub fn inset(&self, distance: Distance) -> Result<Self> {
        self.offset(distance, Side::Inward)
    }

    /// Polygon grown by a distance: every edge moved outward, corners mitred.
    ///
    /// A sharp convex corner grows a long point (the mitre length is the
    /// distance over the sine of half the corner angle: 340 m for 3 m at a
    /// corner of 1°). An inlet whose sides meet fills; one that closes over
    /// a pocket leaves a hole, which a polygon cannot hold, and that is
    /// reported.
    ///
    /// # Errors
    ///
    /// - [`KernelError::OutOfRange`] for a negative distance;
    /// - [`NavigationError::OffsetSplits`] if the grown outline encloses a
    ///   hole;
    /// - [`KernelError::CapacityExceeded`] as for [`Polygon::inset`].
    pub fn outset(&self, distance: Distance) -> Result<Self> {
        self.offset(distance, Side::Outward)
    }

    fn offset(&self, distance: Distance, side: Side) -> Result<Self> {
        let metres = checked_offset(distance)?;
        if metres == 0.0 {
            return Ok(*self);
        }
        let finished = offset_rings::<N>(
            core::iter::once((self.vertices(), side == Side::Outward)),
            metres,
        )?;
        let tolerance = offset_tolerance(metres, self.extent());
        let mut result: Option<Self> = None;
        finished.for_each_ring(|ring| {
            let Some(piece) = offset_piece::<N>(ring, metres, tolerance)? else {
                return Ok(());
            };
            // Inward the polygon's own ring stays counter-clockwise; outward
            // it was walked backwards. A ring the other way round encloses a
            // hole or an island: not one polygon.
            let signed = signed_area(ring);
            let expected = match side {
                Side::Inward => signed > 0.0,
                Side::Outward => signed < 0.0,
            };
            if !expected || result.is_some() {
                return Err(NavigationError::OffsetSplits { distance: metres });
            }
            result = Some(piece);
            Ok(())
        })?;
        let result = result.ok_or(NavigationError::OffsetCollapses { distance: metres })?;
        let keeps_clear = result.vertices().iter().all(|vertex| {
            let (gap, _) = self.nearest(*vertex);
            let placed = self.contains(*vertex);
            let right_side = match side {
                Side::Inward => placed == Containment::Inside,
                Side::Outward => placed == Containment::Outside,
            };
            right_side && gap >= metres - tolerance
        });
        if keeps_clear {
            Ok(result)
        } else {
            Err(NavigationError::OffsetSplits { distance: metres })
        }
    }

    /// Largest coordinate magnitude, metres: scale of the rounding error.
    pub(super) fn extent(&self) -> f64 {
        self.vertices()
            .iter()
            .map(|v| math::abs(v.east_metres()).max(math::abs(v.north_metres())))
            .fold(0.0, f64::max)
    }
}

/// Largest planar coordinate, metres: a million kilometres, far beyond any
/// projection, and small enough that products of coordinates stay exact to
/// a millimetre squared.
const MAX_COORDINATE: f64 = 1e9;

/// Area below this fraction of the squared span is zero: the shoelace sum
/// cannot tell its sign.
const MIN_RELATIVE_AREA: f64 = 1e-12;

/// Diagonal of the bounding box, metres.
fn span(ring: &[PlanarPoint]) -> f64 {
    let (mut low, mut high) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
    for vertex in ring {
        for ((low, high), value) in low.iter_mut().zip(high.iter_mut()).zip(vertex.xy()) {
            *low = low.min(value);
            *high = high.max(value);
        }
    }
    math::hypot(high[0] - low[0], high[1] - low[1])
}

/// Relative tolerance of the offset's distance check.
const OFFSET_TOLERANCE: f64 = 1e-9;

/// Offset distance in metres, checked.
pub(super) fn checked_offset(distance: Distance) -> Result<f64> {
    let metres = distance.metres();
    if metres < 0.0 {
        return Err(KernelError::OutOfRange {
            parameter: "offset distance, metres",
            value: metres,
            min: 0.0,
            max: f64::MAX,
        }
        .into());
    }
    Ok(metres)
}

/// Slack of the check that every offset vertex keeps the full distance.
pub(super) fn offset_tolerance(metres: f64, extent: f64) -> f64 {
    OFFSET_TOLERANCE * (1.0 + metres + extent)
}

/// Polygon of a ring the offset left; `None` for a ring that shrank to
/// nothing exactly at the distance.
pub(super) fn offset_piece<const M: usize>(
    ring: &[PlanarPoint],
    metres: f64,
    tolerance: f64,
) -> Result<Option<Polygon<M>>> {
    match Polygon::new(ring) {
        Ok(piece) => Ok(Some(piece)),
        Err(NavigationError::Degenerate { .. })
            if math::abs(signed_area(ring)) <= tolerance * tolerance =>
        {
            Ok(None)
        }
        Err(error) => Err(offset_failure(error, metres)),
    }
}

/// A ring the offset produced that is not a valid polygon.
pub(super) fn offset_failure(error: NavigationError, metres: f64) -> NavigationError {
    match error {
        NavigationError::Degenerate { .. } => NavigationError::OffsetCollapses { distance: metres },
        NavigationError::SelfIntersecting { .. } => {
            NavigationError::OffsetSplits { distance: metres }
        }
        other => other,
    }
}

/// Ring edges, the last closing the ring.
pub(super) fn edges(ring: &[PlanarPoint]) -> impl Iterator<Item = (PlanarPoint, PlanarPoint)> + '_ {
    ring.iter()
        .copied()
        .zip(ring.iter().copied().cycle().skip(1))
}

/// Twice the signed area is avoided: shoelace about the first vertex, so the
/// products stay small for a field far from the plane's origin.
pub(super) fn signed_area(ring: &[PlanarPoint]) -> f64 {
    let Some(origin) = ring.first().map(|v| v.xy()) else {
        return 0.0;
    };
    let twice: f64 = edges(ring)
        .map(|(a, b)| {
            let a = sub(a.xy(), origin);
            let b = sub(b.xy(), origin);
            cross(a, b)
        })
        .sum();
    twice / 2.0
}

/// Checks that no two edges of a ring cross or touch, adjacent edges meeting
/// only at their shared vertex.
pub(super) fn check_simple(ring: &[PlanarPoint]) -> Result<()> {
    let count = ring.len();
    let edge = |k: usize| -> Option<([f64; 2], [f64; 2])> {
        let next = if k.saturating_add(1) == count {
            0
        } else {
            k.saturating_add(1)
        };
        Some((ring.get(k)?.xy(), ring.get(next)?.xy()))
    };
    for i in 0..count {
        let Some((a1, a2)) = edge(i) else { continue };
        for j in i.saturating_add(1)..count {
            let Some((b1, b2)) = edge(j) else { continue };
            let crossing = if j == i.saturating_add(1) {
                // a2 == b1: overlap if b2 doubles back along a.
                folds_back(a2, a1, b2)
            } else if i == 0 && j == count.saturating_sub(1) {
                // b2 == a1: overlap if b1 doubles back along a.
                folds_back(a1, a2, b1)
            } else {
                segments_meet(a1, a2, b1, b2)
            };
            if crossing {
                return Err(NavigationError::SelfIntersecting {
                    first_edge: i,
                    second_edge: j,
                });
            }
        }
    }
    Ok(())
}

/// Whether two edges sharing `shared` overlap: the other ends lie on one ray.
fn folds_back(shared: [f64; 2], a: [f64; 2], b: [f64; 2]) -> bool {
    let da = sub(a, shared);
    let db = sub(b, shared);
    cross(da, db) == 0.0 && dot(da, db) > 0.0
}

/// Whether two closed segments have a point in common.
pub(super) fn segments_meet(p1: [f64; 2], p2: [f64; 2], q1: [f64; 2], q2: [f64; 2]) -> bool {
    let d1 = orient(q1, q2, p1);
    let d2 = orient(q1, q2, p2);
    let d3 = orient(p1, p2, q1);
    let d4 = orient(p1, p2, q2);
    if ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0))
        && ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0))
    {
        return true;
    }
    (d1 == 0.0 && within_box(q1, q2, p1))
        || (d2 == 0.0 && within_box(q1, q2, p2))
        || (d3 == 0.0 && within_box(p1, p2, q1))
        || (d4 == 0.0 && within_box(p1, p2, q2))
}

/// Whether `p`, collinear with the segment, lies within its bounding box.
fn within_box(a: [f64; 2], b: [f64; 2], p: [f64; 2]) -> bool {
    p[0] >= a[0].min(b[0])
        && p[0] <= a[0].max(b[0])
        && p[1] >= a[1].min(b[1])
        && p[1] <= a[1].max(b[1])
}

/// Point-in-ring by crossing number, boundary checked first.
pub(super) fn ring_contains(ring: &[PlanarPoint], p: [f64; 2]) -> Containment {
    let mut inside = false;
    for (a, b) in edges(ring) {
        let (a, b) = (a.xy(), b.xy());
        if orient(a, b, p) == 0.0 && within_box(a, b, p) {
            return Containment::OnBoundary;
        }
        // Half-open in y, so a vertex on the ray is counted once.
        if (a[1] > p[1]) != (b[1] > p[1]) {
            let x = a[0] + (p[1] - a[1]) / (b[1] - a[1]) * (b[0] - a[0]);
            if p[0] < x {
                inside = !inside;
            }
        }
    }
    if inside {
        Containment::Inside
    } else {
        Containment::Outside
    }
}

/// Point of segment `ab` nearest to `p`.
pub(super) fn closest_on_segment(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> [f64; 2] {
    let ab = sub(b, a);
    let length_squared = dot(ab, ab);
    if length_squared <= 0.0 {
        return a;
    }
    let t = (dot(sub(p, a), ab) / length_squared).clamp(0.0, 1.0);
    [a[0] + t * ab[0], a[1] + t * ab[1]]
}

/// Twice the signed area of triangle `abc`: positive counter-clockwise.
pub(super) fn orient(a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> f64 {
    cross(sub(b, a), sub(c, a))
}

pub(super) fn sub(a: [f64; 2], b: [f64; 2]) -> [f64; 2] {
    [a[0] - b[0], a[1] - b[1]]
}

pub(super) fn dot(a: [f64; 2], b: [f64; 2]) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}

pub(super) fn cross(a: [f64; 2], b: [f64; 2]) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}

#[cfg(feature = "serde")]
#[derive(serde::Serialize, serde::Deserialize)]
struct StoredPolygon(alloc::vec::Vec<PlanarPoint>);

#[cfg(feature = "serde")]
impl<const N: usize> TryFrom<StoredPolygon> for Polygon<N> {
    type Error = NavigationError;

    /// Deserialised through [`Polygon::new`].
    fn try_from(stored: StoredPolygon) -> Result<Self> {
        Self::new(&stored.0)
    }
}

#[cfg(feature = "serde")]
impl<const N: usize> From<Polygon<N>> for StoredPolygon {
    fn from(polygon: Polygon<N>) -> Self {
        Self(polygon.vertices().to_vec())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn point(east: f64, north: f64) -> PlanarPoint {
        PlanarPoint::from_xy([east, north])
    }

    fn metres(value: f64) -> Distance {
        Distance::from_metres(value).unwrap_or(Distance::ZERO)
    }

    fn square(side: f64) -> Polygon<8> {
        let ring = [
            point(0.0, 0.0),
            point(side, 0.0),
            point(side, side),
            point(0.0, side),
        ];
        Polygon::new(&ring).unwrap_or(Polygon::PLACEHOLDER)
    }

    #[test]
    fn a_ring_is_stored_counter_clockwise() {
        let clockwise = [
            point(0.0, 0.0),
            point(0.0, 10.0),
            point(10.0, 10.0),
            point(10.0, 0.0),
        ];
        let polygon = Polygon::<4>::new(&clockwise);
        assert!(polygon.is_ok());
        let polygon = polygon.unwrap_or(Polygon::PLACEHOLDER);
        assert!(signed_area(polygon.vertices()) > 0.0);
        assert_eq!(polygon.area().square_metres(), 100.0);
        assert!((polygon.perimeter().metres() - 40.0).abs() < 1e-9);
    }

    #[test]
    fn a_closed_ring_drops_its_closing_vertex() {
        let ring = [
            point(0.0, 0.0),
            point(4.0, 0.0),
            point(0.0, 3.0),
            point(0.0, 0.0),
        ];
        let polygon = Polygon::<3>::new(&ring);
        assert_eq!(polygon.map(|p| p.vertices().len()), Ok(3));
    }

    #[test]
    fn bad_rings_are_refused() {
        let two = [point(0.0, 0.0), point(1.0, 0.0)];
        assert!(matches!(
            Polygon::<4>::new(&two),
            Err(NavigationError::Kernel(KernelError::InsufficientData {
                found: 2,
                ..
            }))
        ));
        let five = [
            point(0.0, 0.0),
            point(2.0, 0.0),
            point(3.0, 1.0),
            point(2.0, 2.0),
            point(0.0, 2.0),
        ];
        assert!(matches!(
            Polygon::<4>::new(&five),
            Err(NavigationError::Kernel(KernelError::CapacityExceeded {
                needed: 5,
                capacity: 4,
                ..
            }))
        ));
        // Clockwise as given, so the check runs on the reversed ring and the
        // edges are numbered back in the input's order.
        let bow_tie = [
            point(0.0, 0.0),
            point(4.0, 4.0),
            point(4.0, 0.0),
            point(0.0, 2.0),
        ];
        assert!(matches!(
            Polygon::<4>::new(&bow_tie),
            Err(NavigationError::SelfIntersecting {
                first_edge: 0,
                second_edge: 2
            })
        ));
        let balanced = [
            point(0.0, 0.0),
            point(2.0, 2.0),
            point(2.0, 0.0),
            point(0.0, 2.0),
        ];
        assert!(matches!(
            Polygon::<4>::new(&balanced),
            Err(NavigationError::Degenerate { .. })
        ));
        let far = [point(0.0, 0.0), point(2e9, 0.0), point(0.0, 1.0)];
        assert!(matches!(
            Polygon::<4>::new(&far),
            Err(NavigationError::Kernel(KernelError::OutOfRange { .. }))
        ));
        let repeated = [
            point(0.0, 0.0),
            point(1.0, 0.0),
            point(1.0, 0.0),
            point(0.0, 1.0),
        ];
        assert!(matches!(
            Polygon::<4>::new(&repeated),
            Err(NavigationError::Degenerate { .. })
        ));
        let flat = [point(0.0, 0.0), point(1.0, 0.0), point(2.0, 0.0)];
        assert!(Polygon::<4>::new(&flat).is_err());
        // Touches itself at a vertex: (2, 0) lies on the edge (0, 0)–(4, 0).
        let pinched = [
            point(0.0, 0.0),
            point(4.0, 0.0),
            point(4.0, 2.0),
            point(2.0, 0.0),
            point(0.0, 2.0),
        ];
        assert!(matches!(
            Polygon::<8>::new(&pinched),
            Err(NavigationError::SelfIntersecting { .. })
        ));
        let spike = [
            point(0.0, 0.0),
            point(4.0, 0.0),
            point(2.0, 0.0),
            point(2.0, 3.0),
        ];
        assert!(matches!(
            Polygon::<8>::new(&spike),
            Err(NavigationError::SelfIntersecting { .. })
        ));
    }

    #[test]
    fn points_are_placed_exactly() {
        let field = square(10.0);
        assert_eq!(field.contains(point(5.0, 5.0)), Containment::Inside);
        assert_eq!(field.contains(point(10.0, 5.0)), Containment::OnBoundary);
        assert_eq!(field.contains(point(0.0, 0.0)), Containment::OnBoundary);
        assert_eq!(
            field.contains(point(10.0 + 1e-12, 5.0)),
            Containment::Outside
        );
        assert_eq!(field.contains(point(-1.0, 0.0)), Containment::Outside);
        assert!((field.distance_to_boundary(point(3.0, 5.0)).metres() - 3.0).abs() < 1e-9);
        assert!((field.distance_to_boundary(point(13.0, 14.0)).metres() - 5.0).abs() < 1e-9);
        assert_eq!(
            field.closest_boundary_point(point(13.0, 14.0)),
            point(10.0, 10.0)
        );
    }

    #[test]
    fn a_square_insets_and_outsets_by_its_mitres() {
        let field = square(10.0);
        let inner = field.inset(metres(2.0));
        assert_eq!(inner.map(|p| p.area().square_metres().round()), Ok(36.0));
        let outer = field.outset(metres(2.0));
        assert_eq!(outer.map(|p| p.area().square_metres().round()), Ok(196.0));
        assert!(matches!(
            field.inset(metres(5.0)),
            Err(NavigationError::OffsetCollapses { .. })
        ));
        assert!(matches!(
            field.inset(metres(6.0)),
            Err(NavigationError::OffsetCollapses { .. })
        ));
        assert!(field.inset(metres(-1.0)).is_err());
        assert_eq!(field.inset(Distance::ZERO), Ok(field));
    }

    #[test]
    fn a_short_edge_vanishes_on_the_way_in() {
        // A pentagon whose top-right corner is cut by a 1 m chamfer: the
        // chamfer shrinks to nothing before 2 m and the result is a square.
        let ring = [
            point(0.0, 0.0),
            point(10.0, 0.0),
            point(10.0, 9.0),
            point(9.0, 10.0),
            point(0.0, 10.0),
        ];
        let polygon = Polygon::<8>::new(&ring);
        let inner = polygon.and_then(|p| p.inset(metres(2.0)));
        assert_eq!(inner.as_ref().map(|p| p.vertices().len()), Ok(4));
        assert_eq!(inner.map(|p| p.area().square_metres().round()), Ok(36.0));
    }

    #[test]
    fn a_waist_that_would_pinch_off_is_reported() {
        // Two 10 m squares joined by a 2 m wide neck: a 1.5 m inset parts them.
        let ring = [
            point(0.0, 0.0),
            point(10.0, 0.0),
            point(10.0, 4.0),
            point(14.0, 4.0),
            point(14.0, 0.0),
            point(24.0, 0.0),
            point(24.0, 10.0),
            point(14.0, 10.0),
            point(14.0, 6.0),
            point(10.0, 6.0),
            point(10.0, 10.0),
            point(0.0, 10.0),
        ];
        let polygon = Polygon::<16>::new(&ring);
        assert!(polygon.is_ok());
        let narrow = polygon.as_ref().map(|p| p.inset(metres(0.5)).is_ok());
        assert_eq!(narrow, Ok(true));
        let parted = polygon.map(|p| p.inset(metres(1.5)));
        assert!(matches!(
            parted,
            Ok(Err(NavigationError::OffsetSplits { .. }))
        ));
    }

    #[test]
    fn a_split_needs_a_spare_slot() {
        // The waist again at full capacity: parting it takes one more edge.
        let ring = [
            point(0.0, 0.0),
            point(10.0, 0.0),
            point(10.0, 4.0),
            point(14.0, 4.0),
            point(14.0, 0.0),
            point(24.0, 0.0),
            point(24.0, 10.0),
            point(14.0, 10.0),
            point(14.0, 6.0),
            point(10.0, 6.0),
            point(10.0, 10.0),
            point(0.0, 10.0),
        ];
        let parted = Polygon::<12>::new(&ring).map(|p| p.inset(metres(1.5)));
        assert!(matches!(
            parted,
            Ok(Err(NavigationError::Kernel(
                KernelError::CapacityExceeded { capacity: 12, .. }
            )))
        ));
    }

    #[test]
    fn an_inlet_that_closes_fills() {
        // A U shape whose 2 m gap closes when grown by more than 1 m.
        let ring = [
            point(0.0, 0.0),
            point(10.0, 0.0),
            point(10.0, 10.0),
            point(6.0, 10.0),
            point(6.0, 3.0),
            point(4.0, 3.0),
            point(4.0, 10.0),
            point(0.0, 10.0),
        ];
        let polygon = Polygon::<12>::new(&ring);
        let open = polygon
            .as_ref()
            .map(|p| p.outset(metres(0.5)).map(|g| g.vertices().len()));
        assert_eq!(open, Ok(Ok(8)));
        // The walls meet at 1 m and the inlet fills, as a union would: the
        // grown outline is the 13 m square.
        let closed = polygon.map(|p| {
            p.outset(metres(1.5))
                .map(|g| g.area().square_metres().round())
        });
        assert_eq!(closed, Ok(Ok(169.0)));
    }
}
