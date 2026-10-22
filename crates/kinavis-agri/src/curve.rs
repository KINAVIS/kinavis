//! Curved reference lines: polylines, their parallel curves, recorded passes.

use kinavis::error::NavigationError;
use kinavis::region::PlanarPoint;
use kinavis_kernel::error::ensure_range;
use kinavis_kernel::inline::Inline;
use kinavis_kernel::{math, Distance, KernelError, Position};

use crate::error::{AgriError, Result};
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
    /// The shifted segments, joined by arcs and by steps back inside bends,
    /// are cut where they cross; the pieces nearer the line than the distance
    /// are dropped, the rest joined end to end at the crossings, the ends run
    /// on along the end segments. Each vertex of the result lies within twice
    /// `tolerance`, plus a millionth of the distance, of the exact parallel
    /// curve that [`ReferenceLine::station`] measures from: a loop cut next
    /// to an arc meets its chord, not the arc.
    ///
    /// # Errors
    ///
    /// - [`KernelError::OutOfRange`] for a tolerance outside 0.1 mm…1 m or a
    ///   distance beyond ±10 km;
    /// - [`KernelError::CapacityExceeded`] if the curve with its arcs, before
    ///   loops are cut, or its crossings need more than `M` vertices;
    /// - [`NavigationError::OffsetCollapses`] if nothing of the curve is left;
    /// - [`AgriError::PassSplits`](crate::AgriError::PassSplits) if the exact
    ///   parallel curve is in pieces: when a run-on meets the other or the
    ///   curve, so the line crosses itself, or inside a bend of more than half
    ///   a turn that closes a pocket.
    pub fn offset<const M: usize>(
        &self,
        distance: Distance,
        tolerance: Distance,
    ) -> Result<CurveLine<M>> {
        let d = distance.metres();
        let tolerance = tolerance.metres();
        ensure_range("offset of a curve, metres", d, -1e4, 1e4)?;
        ensure_range("chord tolerance, metres", tolerance, 1e-4, 1.0)?;
        let bound = 2.0 * tolerance + 1e-6 * (1.0 + math::abs(d));
        if math::abs(d) <= bound {
            // Within the bound of the curve: the curve itself.
            return CurveLine::<M>::new(self.plane, self.vertices());
        }
        if run_ons_cross(self.vertices()) {
            return Err(AgriError::PassSplits { distance: d });
        }
        let raw = raw_offset::<N, M>(self.vertices(), d, tolerance)?;
        let traced = trace::<N, M>(self, raw.as_slice(), d, bound, tolerance)?;
        let mut kept = Inline::<PlanarPoint, M>::new(PlanarPoint::ORIGIN);
        for point in traced.as_slice() {
            let station = self.station(*point);
            if math::abs(station.offset.metres() - d) > bound {
                continue;
            }
            let distinct = kept
                .as_slice()
                .last()
                .is_none_or(|last| last.distance_to(*point).metres() >= MIN_SEGMENT_METRES);
            if distinct {
                // `kept` is no longer than `traced`, which fits in `M`.
                let _ = kept.push(*point);
            }
        }
        // Folding back is a pass pinched off between two sides of the curve
        // twice the distance apart.
        if kept.len() < 2 || folds_back(kept.as_slice()) {
            return Err(NavigationError::OffsetCollapses { distance: d }.into());
        }
        CurveLine::<M>::new(self.plane, kept.as_slice())
    }

    fn validate(&self) -> Result<()> {
        for (a, b) in segments(self.vertices()) {
            let length = a.distance_to(b).metres();
            if !length.is_finite() || length < MIN_SEGMENT_METRES {
                return Err(NavigationError::Degenerate {
                    context: "a guidance curve with a segment shorter than 1 cm",
                }
                .into());
            }
        }
        if folds_back(self.vertices()) {
            return Err(NavigationError::Degenerate {
                context: "a guidance curve that folds back on itself",
            }
            .into());
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

/// Crossing of the raw parallel curve with itself, at `key` along it:
/// segment index plus the fraction of the segment, below 0 on the run-in and
/// beyond the last vertex on the run-out.
#[derive(Debug, Clone, Copy)]
struct Split {
    key: f64,
    at: (f64, f64),
    /// Index of the crossing; each appears twice.
    crossing: usize,
}

const NO_SPLIT: Split = Split {
    key: 0.0,
    at: (0.0, 0.0),
    crossing: 0,
};

/// The piece of the exact parallel curve that runs on to both ends, traced
/// along the raw parallel curve `raw` (see [`CurveLine::offset`]).
fn trace<const N: usize, const M: usize>(
    curve: &CurveLine<N>,
    raw: &[PlanarPoint],
    d: f64,
    bound: f64,
    tolerance: f64,
) -> Result<Inline<PlanarPoint, M>> {
    let collapses = || NavigationError::OffsetCollapses { distance: d };
    let full = |context| KernelError::CapacityExceeded {
        context,
        needed: M.saturating_add(1),
        capacity: M,
    };
    let path = without_repeats::<M>(raw);
    let path = path.as_slice();
    let last = path.len().checked_sub(2).ok_or_else(collapses)?;
    let on_curve = |at: (f64, f64)| -> Result<bool> {
        let offset = curve
            .station(PlanarPoint::from_metres(at.0, at.1)?)
            .offset
            .metres();
        Ok(math::abs(offset - d) <= bound)
    };
    let splits = crossings::<M>(path, last, on_curve)?;
    let splits = splits.as_slice();
    // Slice `s` runs from split `s − 1` (the run-in for 0) to split `s` (the
    // run-out for the last).
    let slices = splits.len().saturating_add(1);
    let mut used = Inline::<bool, M>::new(false);
    for _ in 0..slices {
        used.push(false)
            .map_err(|_| full("the crossings of a parallel curve"))?;
    }
    let valid = |slice: usize| -> Result<bool> {
        let from = slice
            .checked_sub(1)
            .and_then(|s| splits.get(s))
            .map(|s| s.key);
        let to = splits.get(slice).map(|s| s.key);
        slice_is_valid(curve, path, last, from, to, d, bound)
    };
    let mut out = Inline::<PlanarPoint, M>::new(PlanarPoint::ORIGIN);
    let mut emit = |xy: (f64, f64)| -> Result<()> {
        out.push(PlanarPoint::from_metres(xy.0, xy.1)?)
            .map_err(|_| full("a parallel curve after its loops are cut"))?;
        Ok(())
    };
    if !valid(0)? {
        return Err(collapses().into());
    }
    let run = math::abs(d).max(1.0);
    let mut slice = 0;
    for _ in 0..slices {
        if let Some(flag) = used.as_mut_slice().get_mut(slice) {
            *flag = true;
        }
        let from = slice.checked_sub(1).and_then(|s| splits.get(s));
        let to = splits.get(slice);
        let from_key = match from {
            Some(split) => {
                emit(split.at)?;
                split.key
            }
            // From the first vertex, or the run-in if crossed before it.
            None => match to {
                Some(first) if first.key < 0.0 => {
                    emit(along(path, last, first.key - run / segment_length(path, 0)))?;
                    first.key
                }
                _ => -1.0,
            },
        };
        let to_key = to.map_or(f64::INFINITY, |s| s.key);
        for (m, vertex) in path.iter().enumerate() {
            let key = math::count_to_f64(m);
            if key > from_key && key <= to_key {
                emit(*vertex)?;
            }
        }
        let Some(end) = to else {
            // To the last vertex, or the run-out if crossed beyond it.
            if from_key >= math::count_to_f64(last.saturating_add(1)) {
                emit(along(
                    path,
                    last,
                    from_key + run / segment_length(path, last),
                ))?;
            }
            break;
        };
        slice = onward(splits, slice, end, used.as_slice(), &valid)?
            .ok_or(AgriError::PassSplits { distance: d })?;
    }
    if closes_a_pocket(splits, path, used.as_mut_slice(), &valid, bound, tolerance)? {
        return Err(AgriError::PassSplits { distance: d });
    }
    Ok(out)
}

/// Vertices without repeats, so every segment has a direction.
fn without_repeats<const M: usize>(raw: &[PlanarPoint]) -> Inline<(f64, f64), M> {
    let mut path = Inline::<(f64, f64), M>::new((0.0, 0.0));
    for point in raw {
        let xy = (point.east_metres(), point.north_metres());
        let repeat = path
            .as_slice()
            .last()
            .is_some_and(|last| math::hypot(xy.0 - last.0, xy.1 - last.1) < 1e-9);
        if !repeat {
            // No longer than `raw`, which fits in `M`.
            let _ = path.push(xy);
        }
    }
    path
}

/// Whether what is left of the parallel curve after the traced piece holds a
/// pocket: a loop that closes at the crossing it starts from and encloses
/// ground. A loop within the bound throughout but thin is a step back inside
/// a gentle bend; a chain that does not close is a crumb where the curve
/// pinches off.
fn closes_a_pocket(
    splits: &[Split],
    path: &[(f64, f64)],
    used: &mut [bool],
    valid: &impl Fn(usize) -> Result<bool>,
    bound: f64,
    tolerance: f64,
) -> Result<bool> {
    for first in 1..splits.len() {
        if used.get(first).is_none_or(|u| *u) || !valid(first)? {
            continue;
        }
        let (mut length, mut twice_area) = (0.0, 0.0);
        let start = first.checked_sub(1).and_then(|s| splits.get(s));
        let mut closes = false;
        let mut slice = first;
        for _ in 0..used.len() {
            if let Some(flag) = used.get_mut(slice) {
                *flag = true;
            }
            let (Some(from), Some(end)) = (
                slice.checked_sub(1).and_then(|s| splits.get(s)),
                splits.get(slice),
            ) else {
                break;
            };
            let mut previous = from.at;
            let vertices = path.iter().enumerate().filter_map(|(m, vertex)| {
                let key = math::count_to_f64(m);
                (key > from.key && key < end.key).then_some(*vertex)
            });
            for point in vertices.chain(core::iter::once(end.at)) {
                length += math::hypot(point.0 - previous.0, point.1 - previous.1);
                twice_area += previous.0 * point.1 - point.0 * previous.1;
                previous = point;
            }
            if start.is_some_and(|s| s.crossing == end.crossing) {
                closes = true;
                break;
            }
            match onward(splits, slice, end, used, valid)? {
                Some(next) => slice = next,
                None => break,
            }
        }
        if closes && length > 10.0 * tolerance && math::abs(twice_area) > 4.0 * bound * length {
            return Ok(true);
        }
    }
    Ok(false)
}

/// Slice that the parallel curve takes on from the end of `slice` at the
/// crossing `end`: across the loop if that piece holds, else straight on;
/// `None` if neither is free and valid.
fn onward(
    splits: &[Split],
    slice: usize,
    end: &Split,
    used: &[bool],
    valid: &impl Fn(usize) -> Result<bool>,
) -> Result<Option<usize>> {
    let partner = splits
        .iter()
        .enumerate()
        .position(|(index, s)| index != slice && s.crossing == end.crossing);
    let across = partner.map(|p| p.saturating_add(1));
    let straight = slice.saturating_add(1);
    for candidate in [across, Some(straight)].into_iter().flatten() {
        let free = used.get(candidate).is_some_and(|u| !u);
        if free && valid(candidate)? {
            return Ok(Some(candidate));
        }
    }
    Ok(None)
}

/// Crossings of the path with itself, the first segment extended back and the
/// last on, sorted along the path: those on the exact parallel curve, where
/// it turns from one side's offset to another's. Elsewhere both pieces are
/// off it, and whether a piece holds changes only at such a crossing.
fn crossings<const M: usize>(
    path: &[(f64, f64)],
    last: usize,
    on_curve: impl Fn((f64, f64)) -> Result<bool>,
) -> Result<Inline<Split, M>> {
    let mut splits = Inline::<Split, M>::new(NO_SPLIT);
    let mut count = 0;
    for i in 0..=last {
        for j in i.saturating_add(2)..=last {
            let (Some(start), Some(stop), Some(other), Some(other_stop)) = (
                path.get(i),
                path.get(i.saturating_add(1)),
                path.get(j),
                path.get(j.saturating_add(1)),
            ) else {
                continue;
            };
            let (rx, ry) = (stop.0 - start.0, stop.1 - start.1);
            let (sx, sy) = (other_stop.0 - other.0, other_stop.1 - other.1);
            let denominator = rx * sy - ry * sx;
            if denominator == 0.0 {
                continue;
            }
            let (qx, qy) = (other.0 - start.0, other.1 - start.1);
            let on_first = (qx * sy - qy * sx) / denominator;
            let on_second = (qx * ry - qy * rx) / denominator;
            // A vertex belongs to the segment it starts.
            let first_in = (i == 0 || on_first >= 0.0) && on_first < 1.0;
            let second_in = on_second >= 0.0 && (j == last || on_second < 1.0);
            if !(first_in && second_in && on_first.is_finite() && on_second.is_finite()) {
                continue;
            }
            let at = (start.0 + on_first * rx, start.1 + on_first * ry);
            if !on_curve(at)? {
                continue;
            }
            for key in [
                math::count_to_f64(i) + on_first,
                math::count_to_f64(j) + on_second,
            ] {
                splits
                    .push(Split {
                        key,
                        at,
                        crossing: count,
                    })
                    .map_err(|_| KernelError::CapacityExceeded {
                        context: "the crossings of a parallel curve",
                        needed: M.saturating_add(1),
                        capacity: M,
                    })?;
            }
            count += 1;
        }
    }
    // Insertion sort by key: a few crossings.
    let sorted = splits.as_mut_slice();
    for k in 1..sorted.len() {
        let mut m = k;
        while m > 0 {
            let (Some(before), Some(here)) = (sorted.get(m - 1), sorted.get(m)) else {
                break;
            };
            if before.key <= here.key {
                break;
            }
            sorted.swap(m - 1, m);
            m -= 1;
        }
    }
    Ok(splits)
}

/// Point at `key` along the path, the end segments extended.
fn along(path: &[(f64, f64)], last: usize, key: f64) -> (f64, f64) {
    let k = if key <= 0.0 {
        0
    } else {
        math::to_usize(math::trunc(key)).min(last)
    };
    let t = key - math::count_to_f64(k);
    match (path.get(k), path.get(k.saturating_add(1))) {
        (Some(a), Some(b)) => (a.0 + t * (b.0 - a.0), a.1 + t * (b.1 - a.1)),
        _ => (0.0, 0.0),
    }
}

fn segment_length(path: &[(f64, f64)], k: usize) -> f64 {
    match (path.get(k), path.get(k.saturating_add(1))) {
        (Some(a), Some(b)) => math::hypot(b.0 - a.0, b.1 - a.1).max(f64::MIN_POSITIVE),
        _ => 1.0,
    }
}

/// Breaks of a slice: its ends and the vertices between, `None` for an end
/// that runs on; consecutive breaks lie on one segment.
fn breaks(
    last: usize,
    from: Option<f64>,
    to: Option<f64>,
) -> impl Iterator<Item = (Option<f64>, Option<f64>)> {
    let lo = from.unwrap_or(f64::NEG_INFINITY);
    let hi = to.unwrap_or(f64::INFINITY);
    let vertices = (0..=last.saturating_add(1))
        .map(math::count_to_f64)
        .filter(move |&key| key > lo && key < hi);
    let points = core::iter::once(from)
        .chain(vertices.map(Some))
        .chain(core::iter::once(to));
    let ahead = points.clone().skip(1);
    points.zip(ahead)
}

/// Whether a slice lies on the exact parallel curve: the middle of each of
/// its pieces between breaks, a point one segment out on a run-on.
fn slice_is_valid<const N: usize>(
    curve: &CurveLine<N>,
    path: &[(f64, f64)],
    last: usize,
    from: Option<f64>,
    to: Option<f64>,
    d: f64,
    bound: f64,
) -> Result<bool> {
    for piece in breaks(last, from, to) {
        let key = match piece {
            (Some(a), Some(b)) => f64::midpoint(a, b),
            (None, Some(b)) => b - 1.0,
            (Some(a), None) => a + 1.0,
            (None, None) => 0.5,
        };
        let (x, y) = along(path, last, key);
        let offset = curve
            .station(PlanarPoint::from_metres(x, y)?)
            .offset
            .metres();
        if math::abs(offset - d) > bound {
            return Ok(false);
        }
    }
    Ok(true)
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

/// Whether the run-in (the first segment extended back) or the run-out (the
/// last extended on) meets the other or a segment it does not adjoin.
fn run_ons_cross(points: &[PlanarPoint]) -> bool {
    let xy = |p: &PlanarPoint| (p.east_metres(), p.north_metres());
    let (Some(first), Some(second)) = (points.first(), points.get(1)) else {
        return false;
    };
    let (Some(end), Some(before_end)) = (
        points.last(),
        points.len().checked_sub(2).and_then(|i| points.get(i)),
    ) else {
        return false;
    };
    let (p0, p1, pn, pm) = (xy(first), xy(second), xy(end), xy(before_end));
    let run_in = (p0, (p0.0 - p1.0, p0.1 - p1.1));
    let run_out = (pn, (pn.0 - pm.0, pn.1 - pm.1));
    // Ray from `origin` along `ray` against segment `a`–`b`, or another ray
    // when `unbounded`; the ray's own origin excluded.
    let meets =
        |(origin, ray): ((f64, f64), (f64, f64)), a: (f64, f64), b: (f64, f64), unbounded: bool| {
            let (sx, sy) = (b.0 - a.0, b.1 - a.1);
            let denominator = ray.0 * sy - ray.1 * sx;
            if denominator == 0.0 {
                return false;
            }
            let (qx, qy) = (a.0 - origin.0, a.1 - origin.1);
            let t = (qx * sy - qy * sx) / denominator;
            let u = (qx * ray.1 - qy * ray.0) / denominator;
            t > 0.0 && u >= 0.0 && (unbounded || u <= 1.0)
        };
    let count = points.len().saturating_sub(1);
    if count >= 2 && meets(run_in, pn, (pn.0 + run_out.1 .0, pn.1 + run_out.1 .1), true) {
        return true;
    }
    for i in 0..count {
        let Some((a, b)) = segment(points, i) else {
            continue;
        };
        let (a, b) = (xy(&a), xy(&b));
        if i > 0 && meets(run_in, a, b, false) {
            return true;
        }
        if i.saturating_add(1) < count && meets(run_out, a, b, false) {
            return true;
        }
    }
    false
}

/// Whether a vertex turns sharper than [`MAX_TURN_DEGREES`].
fn folds_back(points: &[PlanarPoint]) -> bool {
    let max_turn = math::to_radians(MAX_TURN_DEGREES);
    points.windows(3).any(|three| match three {
        [a, b, c] => math::abs(turn(unit(sub(*b, *a)), unit(sub(*c, *b)))) > max_turn,
        _ => false,
    })
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
