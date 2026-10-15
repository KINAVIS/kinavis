//! Curved reference lines: polylines, their parallel curves, recorded passes.

use kinavis::error::NavigationError;
use kinavis::region::PlanarPoint;
use kinavis_kernel::error::ensure_range;
use kinavis_kernel::inline::Inline;
use kinavis_kernel::{math, Distance, KernelError, Position};

use crate::error::Result;
use crate::line::{course_of, metres, ReferenceLine, Station};
use crate::plane::FieldPlane;

/// Shortest segment of a curve, metres.
pub const MIN_SEGMENT_METRES: f64 = 0.01;

/// Largest coordinate of a curve vertex, metres.
const MAX_COORDINATE: f64 = 1e9;

/// Sharpest turn at a vertex of a curve, degrees: a sharper one folds the
/// curve back over itself.
const MAX_TURN_DEGREES: f64 = 179.0;

/// Polyline of at most `N` vertices, extended straight beyond both ends.
///
/// Validated by [`CurveLine::new`]: at least two vertices, segments of at
/// least [`MIN_SEGMENT_METRES`], no turn sharper than 179°, no segment
/// crossing or touching a non-adjacent one.
///
/// Its passes are its exact parallel curves: the points at a constant
/// distance from it. On the outside of a bend that is an arc about the
/// vertex; on the inside the parallel curve is cut where it would come
/// closer to another part of the line, so a pass never loops back.
/// [`CurveLine::offset`] gives the same curve as a polyline.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CurveLine<const N: usize> {
    plane: FieldPlane,
    vertices: Inline<PlanarPoint, N>,
}

impl<const N: usize> CurveLine<N> {
    /// Curve through the vertices, in the direction of travel.
    ///
    /// # Errors
    ///
    /// - [`KernelError::InsufficientData`] for fewer than two vertices;
    /// - [`KernelError::CapacityExceeded`] for more than `N`;
    /// - [`KernelError::OutOfRange`] for a coordinate beyond ±10⁹ m;
    /// - [`NavigationError::Degenerate`] for a segment shorter than
    ///   [`MIN_SEGMENT_METRES`] or a turn sharper than 179°;
    /// - [`NavigationError::SelfIntersecting`] if two segments cross or touch.
    pub fn new(plane: FieldPlane, vertices: &[PlanarPoint]) -> Result<Self> {
        if vertices.len() < 2 {
            return Err(KernelError::InsufficientData {
                found: vertices.len(),
                required: 2,
                context: "a guidance curve",
            }
            .into());
        }
        let mut stored = Inline::new(PlanarPoint::ORIGIN);
        for vertex in vertices {
            for value in [vertex.east_metres(), vertex.north_metres()] {
                ensure_range(
                    "planar coordinate, metres",
                    value,
                    -MAX_COORDINATE,
                    MAX_COORDINATE,
                )?;
            }
            stored
                .push(*vertex)
                .map_err(|_| KernelError::CapacityExceeded {
                    context: "a guidance curve",
                    needed: vertices.len(),
                    capacity: N,
                })?;
        }
        let curve = Self {
            plane,
            vertices: stored,
        };
        curve.validate()?;
        Ok(curve)
    }

    /// Curve through surveyed positions.
    ///
    /// # Errors
    ///
    /// As [`FieldPlane::to_plane`] and [`CurveLine::new`].
    pub fn from_positions(plane: FieldPlane, positions: &[Position]) -> Result<Self> {
        let mut points = Inline::<PlanarPoint, N>::new(PlanarPoint::ORIGIN);
        for position in positions {
            points
                .push(plane.to_plane(*position)?)
                .map_err(|_| KernelError::CapacityExceeded {
                    context: "a guidance curve",
                    needed: positions.len(),
                    capacity: N,
                })?;
        }
        Self::new(plane, points.as_slice())
    }

    /// Vertices, in the direction of travel.
    #[must_use]
    pub fn vertices(&self) -> &[PlanarPoint] {
        self.vertices.as_slice()
    }

    /// Length from the first vertex to the last.
    #[must_use]
    pub fn length(&self) -> Distance {
        metres(
            segments(self.vertices())
                .map(|(a, b)| a.distance_to(b).metres())
                .sum(),
        )
    }

    /// Parallel curve at a signed distance, left positive, as a polyline of at
    /// most `M` vertices: arcs on the outside of bends drawn with chords no
    /// farther than `tolerance` from the arc, loops on the inside cut.
    ///
    /// Each vertex of the result lies within `tolerance` of the exact
    /// parallel curve that [`ReferenceLine::station`] measures from.
    ///
    /// # Errors
    ///
    /// - [`KernelError::OutOfRange`] for a tolerance outside 0.1 mm…1 m or a
    ///   distance beyond ±10 km;
    /// - [`KernelError::CapacityExceeded`] if the curve with its arcs, before
    ///   loops are cut, needs more than `M` vertices;
    /// - [`NavigationError::OffsetCollapses`] if nothing of the curve is left.
    pub fn offset<const M: usize>(
        &self,
        distance: Distance,
        tolerance: Distance,
    ) -> Result<CurveLine<M>> {
        let d = distance.metres();
        let tolerance = tolerance.metres();
        ensure_range("offset of a curve, metres", d, -1e4, 1e4)?;
        ensure_range("chord tolerance, metres", tolerance, 1e-4, 1.0)?;
        let mut raw = raw_offset::<N, M>(self.vertices(), d, tolerance)?;
        cut_loops(&mut raw);
        let mut kept = Inline::<PlanarPoint, M>::new(PlanarPoint::ORIGIN);
        for point in raw.as_slice() {
            let station = self.station(*point);
            if math::abs(station.offset.metres() - d)
                > 2.0 * tolerance + 1e-6 * (1.0 + math::abs(d))
            {
                continue;
            }
            let distinct = kept
                .as_slice()
                .last()
                .is_none_or(|last| last.distance_to(*point).metres() >= MIN_SEGMENT_METRES);
            if distinct {
                // `kept` is no longer than `raw`, which fits in `M`.
                let _ = kept.push(*point);
            }
        }
        if kept.len() < 2 {
            return Err(NavigationError::OffsetCollapses { distance: d }.into());
        }
        CurveLine::<M>::new(self.plane, kept.as_slice())
    }

    fn validate(&self) -> Result<()> {
        let max_turn = math::to_radians(MAX_TURN_DEGREES);
        let mut previous: Option<(f64, f64)> = None;
        for (a, b) in segments(self.vertices()) {
            let (sx, sy) = sub(b, a);
            let length = math::hypot(sx, sy);
            if !length.is_finite() || length < MIN_SEGMENT_METRES {
                return Err(NavigationError::Degenerate {
                    context: "a guidance curve with a segment shorter than 1 cm",
                }
                .into());
            }
            let direction = (sx / length, sy / length);
            if let Some(before) = previous {
                if math::abs(turn(before, direction)) > max_turn {
                    return Err(NavigationError::Degenerate {
                        context: "a guidance curve that folds back on itself",
                    }
                    .into());
                }
            }
            previous = Some(direction);
        }
        let points = self.vertices();
        let count = points.len().saturating_sub(1);
        for i in 0..count {
            let Some((a, b)) = segment(points, i) else {
                continue;
            };
            for j in i.saturating_add(2)..count {
                let Some((c, d)) = segment(points, j) else {
                    continue;
                };
                if segments_meet(a, b, c, d) {
                    return Err(NavigationError::SelfIntersecting {
                        first_edge: i,
                        second_edge: j,
                    }
                    .into());
                }
            }
        }
        Ok(())
    }
}

impl<const N: usize> ReferenceLine for CurveLine<N> {
    fn plane(&self) -> &FieldPlane {
        &self.plane
    }

    fn station(&self, point: PlanarPoint) -> Station {
        let points = self.vertices();
        let last = points.len().saturating_sub(2);
        let mut best = f64::INFINITY;
        let mut nearest = Nearest::Segment(0);
        for (i, (a, b)) in segments(points).enumerate() {
            let (sx, sy) = sub(b, a);
            let (wx, wy) = sub(point, a);
            let t = (wx * sx + wy * sy) / (sx * sx + sy * sy);
            // The first and last segments run on to infinity.
            let lower = if i == 0 { f64::NEG_INFINITY } else { 0.0 };
            let upper = if i == last { f64::INFINITY } else { 1.0 };
            let clamped = t.clamp(lower, upper);
            let (fx, fy) = (wx - clamped * sx, wy - clamped * sy);
            let distance = math::hypot(fx, fy);
            if distance < best {
                best = distance;
                nearest = if clamped > t {
                    Nearest::Vertex(i)
                } else if clamped < t {
                    Nearest::Vertex(i.saturating_add(1))
                } else {
                    Nearest::Segment(i)
                };
            }
        }
        match nearest {
            Nearest::Segment(i) => segment_station(points, i, point),
            Nearest::Vertex(v) => vertex_station(points, v, point),
        }
    }
}

/// Part of a curve nearest to a point.
#[derive(Clone, Copy)]
enum Nearest {
    /// Interior of segment `i`.
    Segment(usize),
    /// Interior vertex `v`.
    Vertex(usize),
}

fn segment_station(points: &[PlanarPoint], i: usize, point: PlanarPoint) -> Station {
    let (Some(a), Some(b)) = (points.get(i), points.get(i.saturating_add(1))) else {
        return Station {
            offset: Distance::ZERO,
            direction: course_of(0.0, 1.0),
        };
    };
    let (sx, sy) = sub(*b, *a);
    let (wx, wy) = sub(point, *a);
    let length = math::hypot(sx, sy);
    Station {
        offset: metres((sx * wy - sy * wx) / length),
        direction: course_of(sx, sy),
    }
}

fn vertex_station(points: &[PlanarPoint], v: usize, point: PlanarPoint) -> Station {
    let (Some(before), Some(vertex), Some(after)) = (
        v.checked_sub(1).and_then(|u| points.get(u)),
        points.get(v),
        points.get(v.saturating_add(1)),
    ) else {
        return segment_station(points, v.saturating_sub(1), point);
    };
    let incoming = unit(sub(*vertex, *before));
    let outgoing = unit(sub(*after, *vertex));
    let mut tangent = (incoming.0 + outgoing.0, incoming.1 + outgoing.1);
    if math::hypot(tangent.0, tangent.1) < 1e-9 {
        tangent = incoming;
    }
    let (wx, wy) = sub(point, *vertex);
    let distance = math::hypot(wx, wy);
    let side = tangent.0 * wy - tangent.1 * wx;
    // Pass direction: across the radius from the vertex, along the travel.
    let (mut ex, mut ey) = (wy, -wx);
    if ex * tangent.0 + ey * tangent.1 < 0.0 {
        (ex, ey) = (-ex, -ey);
    }
    if distance <= 0.0 {
        (ex, ey) = tangent;
    }
    Station {
        offset: metres(if side < 0.0 { -distance } else { distance }),
        direction: course_of(ex, ey),
    }
}

/// Offset segments joined by arcs outside bends and by a step back inside.
fn raw_offset<const N: usize, const M: usize>(
    points: &[PlanarPoint],
    d: f64,
    tolerance: f64,
) -> Result<Inline<PlanarPoint, M>> {
    let mut out = Inline::<PlanarPoint, M>::new(PlanarPoint::ORIGIN);
    let mut push = |x: f64, y: f64| -> Result<()> {
        let point = PlanarPoint::from_metres(x, y)?;
        out.push(point).map_err(|_| KernelError::CapacityExceeded {
            context: "a parallel curve before its loops are cut",
            needed: M.saturating_add(1),
            capacity: M,
        })?;
        Ok(())
    };
    // Largest angle an arc chord may span within the tolerance.
    let step = if tolerance >= math::abs(d) {
        core::f64::consts::PI
    } else {
        2.0 * math::acos(1.0 - tolerance / math::abs(d))
    };
    let mut previous: Option<(f64, f64)> = None;
    for (a, b) in segments(points) {
        let direction = unit(sub(b, a));
        let normal = (-direction.1, direction.0);
        if let Some(before) = previous {
            let normal_before = (-before.1, before.0);
            let angle = turn(before, direction);
            let (vx, vy) = (a.east_metres(), a.north_metres());
            push(vx + d * normal_before.0, vy + d * normal_before.1)?;
            if d * angle < 0.0 {
                // Outside of the bend: the arc about the vertex.
                let count = math::ceil(math::abs(angle) / step);
                let chords = math::to_usize(count).max(1);
                for k in 1..chords {
                    let phi = angle * math::count_to_f64(k) / count;
                    let (s, c) = (math::sin(phi), math::cos(phi));
                    let rotated = (
                        normal_before.0 * c - normal_before.1 * s,
                        normal_before.0 * s + normal_before.1 * c,
                    );
                    push(vx + d * rotated.0, vy + d * rotated.1)?;
                }
            }
            push(vx + d * normal.0, vy + d * normal.1)?;
        } else {
            push(
                a.east_metres() + d * normal.0,
                a.north_metres() + d * normal.1,
            )?;
        }
        previous = Some(direction);
    }
    if let (Some(last), Some(direction)) = (points.last(), previous) {
        push(
            last.east_metres() - d * direction.1,
            last.north_metres() + d * direction.0,
        )?;
    }
    Ok(out)
}

/// Cuts every loop: while segment `i` meets a later non-adjacent segment `j`,
/// the vertices between them give way to the meeting point, taking the
/// latest `j` for the earliest `i`.
fn cut_loops<const M: usize>(points: &mut Inline<PlanarPoint, M>) {
    // Each cut removes at least one vertex, so `M` passes suffice.
    for _ in 0..M {
        let Some((i, j, meeting)) = first_loop(points.as_slice()) else {
            return;
        };
        // Vertices i+1 ..= j go; the meeting point takes their place.
        let mut removed = j.saturating_sub(i);
        while removed > 0 {
            if points.remove(i.saturating_add(1)).is_none() {
                return;
            }
            removed -= 1;
        }
        if points.insert(i.saturating_add(1), meeting).is_err() {
            return;
        }
    }
}

fn first_loop(points: &[PlanarPoint]) -> Option<(usize, usize, PlanarPoint)> {
    let count = points.len().saturating_sub(1);
    for i in 0..count {
        let Some((a, b)) = segment(points, i) else {
            continue;
        };
        // The latest meeting segment, searched from the end.
        for j in (i.saturating_add(2)..count).rev() {
            let Some((c, d)) = segment(points, j) else {
                continue;
            };
            if let Some(meeting) = intersection(a, b, c, d) {
                return Some((i, j, meeting));
            }
        }
    }
    None
}

/// Segment `i`: vertices `i` and `i + 1`.
fn segment(points: &[PlanarPoint], i: usize) -> Option<(PlanarPoint, PlanarPoint)> {
    Some((*points.get(i)?, *points.get(i.checked_add(1)?)?))
}

/// Recorder of a driven pass, to be followed as a curve on the next ones.
///
/// Keeps a vertex every `spacing` of travel. When the store of `N` fills,
/// every other vertex is dropped and the spacing doubles, so a pass of any
/// length fits and stays evenly sampled.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ContourRecorder<const N: usize> {
    plane: FieldPlane,
    points: Inline<PlanarPoint, N>,
    spacing: f64,
}

impl<const N: usize> ContourRecorder<N> {
    /// Empty recorder.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] for a spacing outside 0.1…100 m;
    /// [`KernelError::CapacityExceeded`] for `N` below 3.
    pub fn new(plane: FieldPlane, spacing: Distance) -> Result<Self> {
        ensure_range("contour spacing, metres", spacing.metres(), 0.1, 100.0)?;
        if N < 3 {
            return Err(KernelError::CapacityExceeded {
                context: "a contour recorder",
                needed: 3,
                capacity: N,
            }
            .into());
        }
        Ok(Self {
            plane,
            points: Inline::new(PlanarPoint::ORIGIN),
            spacing: spacing.metres(),
        })
    }

    /// Plane.
    #[must_use]
    pub const fn plane(&self) -> &FieldPlane {
        &self.plane
    }

    /// Current spacing between kept vertices.
    #[must_use]
    pub fn spacing(&self) -> Distance {
        metres(self.spacing)
    }

    /// Vertices kept so far.
    #[must_use]
    pub fn vertices(&self) -> &[PlanarPoint] {
        self.points.as_slice()
    }

    /// Records the position of the control point; kept if it is at least the
    /// spacing from the last kept vertex. Returns whether it was kept.
    pub fn record(&mut self, point: PlanarPoint) -> bool {
        let far_enough = self
            .points
            .as_slice()
            .last()
            .is_none_or(|last| last.distance_to(point).metres() >= self.spacing);
        if !far_enough {
            return false;
        }
        if self.points.len() >= N {
            self.thin();
        }
        self.points.push(point).is_ok()
    }

    /// Recorded pass as a curve.
    ///
    /// # Errors
    ///
    /// As [`CurveLine::new`]: a pass that crosses itself is refused.
    pub fn finish(&self) -> Result<CurveLine<N>> {
        CurveLine::new(self.plane, self.points.as_slice())
    }

    /// Drops every vertex at an odd index and doubles the spacing.
    fn thin(&mut self) {
        let mut kept = Inline::<PlanarPoint, N>::new(PlanarPoint::ORIGIN);
        for point in self.points.as_slice().iter().step_by(2) {
            // Half of a store of `N` fits in `N`.
            let _ = kept.push(*point);
        }
        self.points = kept;
        self.spacing *= 2.0;
    }
}

/// Consecutive pairs of vertices.
fn segments(points: &[PlanarPoint]) -> impl Iterator<Item = (PlanarPoint, PlanarPoint)> + '_ {
    points.iter().copied().zip(points.iter().copied().skip(1))
}

fn sub(a: PlanarPoint, b: PlanarPoint) -> (f64, f64) {
    (
        a.east_metres() - b.east_metres(),
        a.north_metres() - b.north_metres(),
    )
}

fn unit((x, y): (f64, f64)) -> (f64, f64) {
    let length = math::hypot(x, y);
    if length > 0.0 {
        (x / length, y / length)
    } else {
        (0.0, 1.0)
    }
}

/// Signed angle from one unit direction to the next, counter-clockwise
/// positive, radians.
fn turn(from: (f64, f64), to: (f64, f64)) -> f64 {
    math::atan2(from.0 * to.1 - from.1 * to.0, from.0 * to.0 + from.1 * to.1)
}

/// Whether two closed segments share a point.
fn segments_meet(
    first_start: PlanarPoint,
    first_end: PlanarPoint,
    second_start: PlanarPoint,
    second_end: PlanarPoint,
) -> bool {
    let (a, b, c, d) = (first_start, first_end, second_start, second_end);
    let o1 = orientation(a, b, c);
    let o2 = orientation(a, b, d);
    let o3 = orientation(c, d, a);
    let o4 = orientation(c, d, b);
    if o1 * o2 < 0.0 && o3 * o4 < 0.0 {
        return true;
    }
    (o1 == 0.0 && within(a, b, c))
        || (o2 == 0.0 && within(a, b, d))
        || (o3 == 0.0 && within(c, d, a))
        || (o4 == 0.0 && within(c, d, b))
}

/// Meeting point of two segments, if they cross or touch at one point.
fn intersection(
    first_start: PlanarPoint,
    first_end: PlanarPoint,
    second_start: PlanarPoint,
    second_end: PlanarPoint,
) -> Option<PlanarPoint> {
    let (a, c) = (first_start, second_start);
    let (rx, ry) = sub(first_end, a);
    let (sx, sy) = sub(second_end, c);
    let denominator = rx * sy - ry * sx;
    if denominator == 0.0 {
        return None;
    }
    let (qx, qy) = sub(c, a);
    let t = (qx * sy - qy * sx) / denominator;
    let u = (qx * ry - qy * rx) / denominator;
    if (0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&u) {
        PlanarPoint::from_metres(a.east_metres() + t * rx, a.north_metres() + t * ry).ok()
    } else {
        None
    }
}

/// Twice the signed area of triangle `a`, `b`, `c`.
fn orientation(a: PlanarPoint, b: PlanarPoint, c: PlanarPoint) -> f64 {
    let (bx, by) = sub(b, a);
    let (cx, cy) = sub(c, a);
    bx * cy - by * cx
}

/// Whether `p`, collinear with `a`–`b`, lies on the segment.
fn within(a: PlanarPoint, b: PlanarPoint, p: PlanarPoint) -> bool {
    let (lo_x, hi_x) = min_max(a.east_metres(), b.east_metres());
    let (lo_y, hi_y) = min_max(a.north_metres(), b.north_metres());
    (lo_x..=hi_x).contains(&p.east_metres()) && (lo_y..=hi_y).contains(&p.north_metres())
}

fn min_max(a: f64, b: f64) -> (f64, f64) {
    if a <= b {
        (a, b)
    } else {
        (b, a)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use kinavis_kernel::Height;

    fn plane() -> Result<FieldPlane> {
        FieldPlane::centred_at(
            Position::from_degrees(48.0, 11.0)?,
            Height::above_ellipsoid(Distance::from_metres(500.0)?),
        )
    }

    fn points(xy: &[(f64, f64)]) -> Result<Inline<PlanarPoint, 16>> {
        let mut out = Inline::new(PlanarPoint::ORIGIN);
        for (x, y) in xy {
            let _ = out.push(PlanarPoint::from_metres(*x, *y)?);
        }
        Ok(out)
    }

    fn l_shape() -> Result<CurveLine<16>> {
        CurveLine::new(
            plane()?,
            points(&[(0.0, 0.0), (100.0, 0.0), (100.0, 100.0)])?.as_slice(),
        )
    }

    #[test]
    fn a_bend_is_measured_from_its_vertex_outside_and_cut_inside() -> Result<()> {
        let curve = l_shape()?;
        // Outside the bend, south-east of the vertex: on the arc of radius 5√2.
        let outside = curve.station(PlanarPoint::from_metres(105.0, -5.0)?);
        assert!((outside.offset.metres() + 50f64.sqrt()).abs() < 1e-9);
        assert!((outside.direction.degrees() - 45.0).abs() < 1e-9);
        // Inside: the nearer leg wins.
        let inside = curve.station(PlanarPoint::from_metres(97.0, 10.0)?);
        assert!((inside.offset.metres() - 3.0).abs() < 1e-9);
        assert!(inside.direction.degrees().abs() < 1e-9);
        // Beyond the ends the legs run on.
        let before = curve.station(PlanarPoint::from_metres(-50.0, 2.0)?);
        assert!((before.offset.metres() - 2.0).abs() < 1e-9);
        Ok(())
    }

    #[test]
    fn the_parallel_curve_rounds_outside_and_cuts_inside() -> Result<()> {
        let curve = l_shape()?;
        let tolerance = Distance::from_metres(0.01)?;
        let outside: CurveLine<64> = curve.offset(Distance::from_metres(-10.0)?, tolerance)?;
        assert!(outside.vertices().len() > 4);
        for vertex in outside.vertices() {
            assert!((curve.station(*vertex).offset.metres() + 10.0).abs() < 0.011);
        }
        let inside: CurveLine<64> = curve.offset(Distance::from_metres(10.0)?, tolerance)?;
        let corner = inside.vertices().get(1).copied();
        assert_eq!(corner, Some(PlanarPoint::from_metres(90.0, 10.0)?));
        assert_eq!(inside.vertices().len(), 3);
        Ok(())
    }

    #[test]
    fn crossing_and_folding_curves_are_refused() -> Result<()> {
        let crossing = points(&[(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (5.0, -5.0)])?;
        assert!(matches!(
            CurveLine::<16>::new(plane()?, crossing.as_slice()),
            Err(crate::AgriError::Navigation(
                NavigationError::SelfIntersecting { .. }
            ))
        ));
        let folding = points(&[(0.0, 0.0), (10.0, 0.0), (0.0, 0.0001)])?;
        assert!(CurveLine::<16>::new(plane()?, folding.as_slice()).is_err());
        Ok(())
    }

    #[test]
    fn the_recorder_thins_when_full() -> Result<()> {
        let mut recorder = ContourRecorder::<8>::new(plane()?, Distance::from_metres(1.0)?)?;
        for k in 0..40 {
            let _ = recorder.record(PlanarPoint::from_metres(f64::from(k) * 0.5, 0.0)?);
        }
        assert!(recorder.vertices().len() <= 8);
        assert!(recorder.spacing().metres() > 1.0);
        let curve = recorder.finish()?;
        assert!(curve.length().metres() > 15.0);
        Ok(())
    }
}
