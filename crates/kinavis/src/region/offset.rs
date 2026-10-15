//! Mitred offset of a ring to the left of its edges: the wavefront of a
//! straight skeleton.
//!
//! Each edge lies on a line `n · x = c`, `n` the unit normal to the left. At
//! offset `t` the line is `n · x = c + t`; a vertex is the meeting point of its
//! two edges' lines and moves linearly in `t` with velocity
//! `(nₐ + n_b) / (1 + nₐ · n_b)`. Each edge keeps its start vertex as of the
//! last event that touched it and moves it on from there: intersecting two
//! nearly parallel lines afresh would lose millimetres. The front advances from
//! event to event:
//!
//! - edge event: an edge shrinks to nothing and is removed, its neighbours
//!   meet directly;
//! - split event: a reflex vertex runs into another edge, which is cut in two,
//!   and the ring parts into two rings that move on independently.
//!
//! Neighbours on one line facing opposite ways enclose nothing and both go; a
//! ring of fewer than three edges is gone. Of two simultaneous events the
//! edge event goes first. The front lives in a fixed arena of `N` edges linked
//! into rings; a split takes a slot freed by an earlier edge event, or a spare
//! one.

use super::polygon::{cross, dot, sub};
use super::PlanarPoint;
use crate::error::{KernelError, NavigationError, Result};
use crate::inline::Inline;
use crate::math;

/// Which way the polygon's edges move.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum Side {
    Inward,
    Outward,
}

/// Edge line: unit direction `u`, left normal `n`, `n · x = c`.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Line {
    u: [f64; 2],
    n: [f64; 2],
    c: f64,
}

impl Line {
    fn through(start: [f64; 2], end: [f64; 2]) -> Option<Self> {
        let delta = sub(end, start);
        let length = math::hypot(delta[0], delta[1]);
        if length <= 0.0 {
            return None;
        }
        let u = [delta[0] / length, delta[1] / length];
        let n = [-u[1], u[0]];
        Some(Self {
            u,
            n,
            c: dot(n, start),
        })
    }
}

/// Edge of the front, linked into its ring, with its start vertex at offset
/// `since`.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Edge {
    line: Line,
    previous: usize,
    next: usize,
    alive: bool,
    start: [f64; 2],
    since: f64,
}

impl Edge {
    const UNUSED: Self = Self {
        line: Line {
            u: [1.0, 0.0],
            n: [0.0, 1.0],
            c: 0.0,
        },
        previous: 0,
        next: 0,
        alive: false,
        start: [0.0, 0.0],
        since: 0.0,
    };
}

/// Lines closer to parallel than this (sine of the angle) are parallel.
const PARALLEL: f64 = 1e-12;

/// Neighbours facing opposite ways closer than this (one plus the cosine of
/// the angle between them, about half its square) enclose nothing: their
/// meeting point would move faster than a million times the offset.
const OPPOSITE: f64 = 1e-12;

/// Slack, metres, for a split point at the very end of the edge it hits.
const ON_EDGE: f64 = 1e-9;

/// Relative offset within which two events count as simultaneous.
const TIE: f64 = 1e-12;

/// What a tidying step removes.
#[derive(Debug, Clone, Copy)]
enum Tidy {
    /// A ring of fewer than three edges.
    Ring,
    /// The second of two neighbours on one line facing the same way (`true`),
    /// or both facing opposite ways.
    Parallel(bool),
    /// An edge of no length that does not grow.
    Empty,
}

/// Next event of the front.
#[derive(Debug, Clone, Copy)]
enum Event {
    /// Edge at this slot shrinks to nothing.
    Collapse(usize),
    /// Start vertex of the edge at `vertex` runs into the edge at `edge`.
    Split { vertex: usize, edge: usize },
}

/// The front: edges in an arena, linked into rings.
struct Front<const N: usize> {
    edges: Inline<Edge, N>,
}

impl<const N: usize> Front<N> {
    fn edge(&self, slot: usize) -> Option<&Edge> {
        self.edges.get(slot)
    }

    fn edge_mut(&mut self, slot: usize) -> Option<&mut Edge> {
        self.edges.as_mut_slice().get_mut(slot)
    }

    fn alive(&self) -> impl Iterator<Item = (usize, &Edge)> + '_ {
        self.edges.iter().enumerate().filter(|(_, e)| e.alive)
    }

    /// Unlinks an edge from its ring.
    fn remove(&mut self, slot: usize) -> Option<()> {
        let edge = *self.edge(slot)?;
        self.edge_mut(edge.previous)?.next = edge.next;
        self.edge_mut(edge.next)?.previous = edge.previous;
        self.edge_mut(slot)?.alive = false;
        Some(())
    }

    /// Number of edges in the ring through `slot`, at most `N`.
    fn ring_len(&self, slot: usize) -> usize {
        let mut count = 0_usize;
        let mut at = slot;
        for _ in 0..N {
            count = count.saturating_add(1);
            match self.edge(at) {
                Some(edge) if edge.next != slot => at = edge.next,
                _ => break,
            }
        }
        count
    }

    /// Start vertex of the edge at `slot` at offset `t`, and its velocity.
    fn vertex(&self, slot: usize, t: f64) -> Option<([f64; 2], [f64; 2])> {
        let edge = self.edge(slot)?;
        let velocity = velocity(&self.edge(edge.previous)?.line, &edge.line)?;
        let elapsed = t - edge.since;
        Some((
            [
                edge.start[0] + elapsed * velocity[0],
                edge.start[1] + elapsed * velocity[1],
            ],
            velocity,
        ))
    }

    /// Records the start vertex of the edge at `slot` as of offset `t`, before
    /// its previous edge changes.
    fn anchor(&mut self, slot: usize, t: f64) -> Option<()> {
        let (start, _) = self.vertex(slot, t)?;
        let edge = self.edge_mut(slot)?;
        edge.start = start;
        edge.since = t;
        Some(())
    }

    /// Removes an edge that shrank to nothing at `t`; its successor starts
    /// where it vanished.
    fn collapse(&mut self, slot: usize, t: f64) -> Option<()> {
        let next = self.edge(slot)?.next;
        self.anchor(next, t)?;
        self.remove(slot)
    }

    /// Length of the edge at `slot` at offset `t`, and its rate of change.
    fn length(&self, slot: usize, t: f64) -> Option<(f64, f64)> {
        let edge = self.edge(slot)?;
        let (start, start_velocity) = self.vertex(slot, t)?;
        let (end, end_velocity) = self.vertex(edge.next, t)?;
        Some((
            dot(edge.line.u, sub(end, start)),
            dot(edge.line.u, sub(end_velocity, start_velocity)),
        ))
    }

    /// One tidying step at offset `t`: drops a ring of fewer than three
    /// edges, one pair of neighbours on one line, or an edge of no length
    /// that does not grow (left by a vertex meeting a vertex). Returns
    /// whether anything changed.
    fn tidy_once(&mut self, t: f64) -> Option<bool> {
        let found = self.alive().find_map(|(slot, edge)| {
            if self.ring_len(slot) < 3 {
                return Some((slot, Tidy::Ring));
            }
            let next = self.edge(edge.next)?;
            if 1.0 + dot(edge.line.u, next.line.u) < OPPOSITE {
                return Some((slot, Tidy::Parallel(false)));
            }
            if math::abs(cross(edge.line.u, next.line.u)) < PARALLEL
                && dot(edge.line.u, next.line.u) > 0.0
            {
                return Some((slot, Tidy::Parallel(true)));
            }
            let (length, rate) = self.length(slot, t)?;
            if length <= ON_EDGE * (1.0 + t) && rate <= TIE {
                return Some((slot, Tidy::Empty));
            }
            None
        });
        let Some((slot, kind)) = found else {
            return Some(false);
        };
        let next = self.edge(slot)?.next;
        match kind {
            Tidy::Empty => self.collapse(slot, t)?,
            Tidy::Ring => {
                // Ring too small: drop it whole.
                let mut at = slot;
                for _ in 0..N {
                    let following = self.edge(at)?.next;
                    self.edge_mut(at)?.alive = false;
                    if following == slot || !self.edge(following)?.alive {
                        break;
                    }
                    at = following;
                }
            }
            // Straight vertex: one line, one edge.
            Tidy::Parallel(true) => {
                let after = self.edge(next)?.next;
                self.anchor(after, t)?;
                self.remove(next)?;
            }
            // Zero-width spike or inlet: nothing between them. The edges on
            // either side meet afresh.
            Tidy::Parallel(false) => {
                let before = self.edge(slot)?.previous;
                let after = self.edge(next)?.next;
                let (from, _) = self.vertex(slot, t)?;
                let (to, _) = self.vertex(after, t)?;
                let start = meeting(&self.edge(before)?.line, &self.edge(after)?.line, t)
                    .unwrap_or([f64::midpoint(from[0], to[0]), f64::midpoint(from[1], to[1])]);
                self.remove(next)?;
                self.remove(slot)?;
                let edge = self.edge_mut(after)?;
                edge.start = start;
                edge.since = t;
            }
        }
        Some(true)
    }

    fn tidy(&mut self, t: f64) -> Option<()> {
        for _ in 0..=N {
            if !self.tidy_once(t)? {
                return Some(());
            }
        }
        Some(())
    }

    /// Earliest event after offset `t`, and how much further it lies.
    fn next_event(&self, t: f64) -> Option<(Event, f64)> {
        let mut collapse: Option<(Event, f64)> = None;
        let mut split: Option<(Event, f64)> = None;
        for (slot, edge) in self.alive() {
            let (start, start_velocity) = self.vertex(slot, t)?;
            let (length, rate) = self.length(slot, t)?;
            let length = length.max(0.0);
            if rate < 0.0 {
                let step = length / -rate;
                if collapse.is_none_or(|(_, s)| step < s) {
                    collapse = Some((Event::Collapse(slot), step));
                }
            }
            // A start vertex where the front turns right is reflex.
            let previous = self.edge(edge.previous)?;
            if cross(previous.line.u, edge.line.u) >= 0.0 {
                continue;
            }
            for (other, target) in self.alive() {
                if other == slot || other == edge.previous {
                    continue;
                }
                let closing = dot(target.line.n, start_velocity) - 1.0;
                if closing >= 0.0 {
                    continue;
                }
                let step = (target.line.c + t - dot(target.line.n, start)) / closing;
                if step < 0.0 || split.is_some_and(|(_, s)| step >= s) {
                    continue;
                }
                let at = t + step;
                let hit = [
                    start[0] + step * start_velocity[0],
                    start[1] + step * start_velocity[1],
                ];
                let (from, _) = self.vertex(other, at)?;
                let (to, _) = self.vertex(target.next, at)?;
                let u = target.line.u;
                if dot(u, sub(hit, from)) >= -ON_EDGE && dot(u, sub(to, hit)) >= -ON_EDGE {
                    split = Some((
                        Event::Split {
                            vertex: slot,
                            edge: other,
                        },
                        step,
                    ));
                }
            }
        }
        // An edge event at the same moment goes first: it may leave nothing
        // to split, and it needs no slot.
        let slack = TIE * (1.0 + t);
        Some(match (collapse, split) {
            (Some(c), Some(s)) if s.1 < c.1 - slack => s,
            (Some(c), _) => c,
            (None, Some(s)) => s,
            (None, None) => (Event::Collapse(0), f64::INFINITY),
        })
    }

    /// Cuts the ring where the start vertex of `vertex` meets `edge`: one
    /// ring keeps `edge` and runs on to `vertex`, the other takes a copy of
    /// `edge` after the edge before `vertex`.
    fn split(&mut self, vertex: usize, edge: usize, t: f64) -> Result<Option<()>> {
        let (Some(at_vertex), Some(hit), Some((point, _))) =
            (self.edge(vertex), self.edge(edge), self.vertex(vertex, t))
        else {
            return Ok(None);
        };
        let before = at_vertex.previous;
        let hit = *hit;
        let copy = Edge {
            line: hit.line,
            previous: before,
            next: hit.next,
            alive: true,
            start: point,
            since: t,
        };
        let slot = if let Some(free) = self.edges.iter().position(|e| !e.alive) {
            free
        } else {
            self.edges
                .push(Edge::UNUSED)
                .map_err(|_| KernelError::CapacityExceeded {
                    context: "the front of a polygon offset",
                    needed: N.saturating_add(1),
                    capacity: N,
                })?;
            self.edges.len().saturating_sub(1)
        };
        let linked = (|| {
            *self.edge_mut(slot)? = copy;
            self.edge_mut(before)?.next = slot;
            self.edge_mut(hit.next)?.previous = slot;
            self.edge_mut(edge)?.next = vertex;
            let cut = self.edge_mut(vertex)?;
            cut.previous = edge;
            cut.start = point;
            cut.since = t;
            Some(())
        })();
        Ok(linked)
    }
}

/// Velocity in `t` of the meeting point of lines `a` and `b`: the vector
/// whose projection on either normal is one.
fn velocity(a: &Line, b: &Line) -> Option<[f64; 2]> {
    let scale = 1.0 + dot(a.n, b.n);
    if scale < OPPOSITE {
        return None;
    }
    Some([(a.n[0] + b.n[0]) / scale, (a.n[1] + b.n[1]) / scale])
}

/// Meeting point of lines `a` and `b` at offset `t`.
fn meeting(a: &Line, b: &Line, t: f64) -> Option<[f64; 2]> {
    let det = cross(a.n, b.n);
    if math::abs(det) < PARALLEL {
        return None;
    }
    let (ra, rb) = (a.c + t, b.c + t);
    Some([
        (ra * b.n[1] - a.n[1] * rb) / det,
        (a.n[0] * rb - ra * b.n[0]) / det,
    ])
}

/// Rings offset together by `distance` metres to the left of their edges,
/// in a front of at most `N` edges. A ring given as `(vertices, true)` is
/// walked backwards.
///
/// Rings meet as the front moves: a vertex of one running into an edge of
/// another joins them into one ring (a hole reaching the boundary becomes a
/// notch), as a vertex running into an edge of its own ring parts it.
///
/// # Errors
///
/// - [`KernelError::CapacityExceeded`] if the rings, or a split, need more
///   than `N` edges;
/// - [`NavigationError::OffsetCollapses`] for a ring with a repeated vertex;
/// - [`KernelError::NotConverged`] if the events do not run out.
pub(super) fn offset_rings<'a, const N: usize>(
    rings: impl Iterator<Item = (&'a [PlanarPoint], bool)> + Clone,
    distance: f64,
) -> Result<Finished<N>> {
    let collapses = NavigationError::OffsetCollapses { distance };
    let splits = NavigationError::OffsetSplits { distance };
    // Relative to the first vertex: a field 450 km from the plane's origin
    // would otherwise lose millimetres in `c` and in every vertex.
    let origin = rings
        .clone()
        .find_map(|(ring, _)| ring.first())
        .map_or([0.0, 0.0], |p| p.xy());
    let mut front: Front<N> = Front {
        edges: Inline::new(Edge::UNUSED),
    };
    let full = |needed: usize| KernelError::CapacityExceeded {
        context: "the front of a polygon offset",
        needed,
        capacity: N,
    };
    for (ring, backwards) in rings {
        let first = front.edges.len();
        let count = ring.len();
        let at = |k: usize| {
            let index = if backwards {
                count.saturating_sub(1).saturating_sub(k)
            } else {
                k
            };
            ring.get(index).map(|p| sub(p.xy(), origin))
        };
        for k in 0..count {
            let following = if k.saturating_add(1) == count {
                0
            } else {
                k.saturating_add(1)
            };
            let (Some(a), Some(b)) = (at(k), at(following)) else {
                return Err(collapses);
            };
            let edge = Edge {
                line: Line::through(a, b).ok_or(collapses.clone())?,
                previous: first.saturating_add(if k == 0 {
                    count.saturating_sub(1)
                } else {
                    k.saturating_sub(1)
                }),
                next: first.saturating_add(following),
                alive: true,
                start: a,
                since: 0.0,
            };
            front
                .edges
                .push(edge)
                .map_err(|_| full(front.edges.len().saturating_add(count.saturating_sub(k))))?;
        }
    }
    let mut travelled = 0.0;
    // Every event removes an edge or adds a ring, and a ring holds at least
    // three edges, so the events are bounded.
    let events = N.saturating_mul(3).saturating_add(3);
    for _ in 0..events {
        front.tidy(travelled).ok_or(splits.clone())?;
        let (event, step) = front.next_event(travelled).ok_or(splits.clone())?;
        if travelled + step >= distance {
            return Ok(Finished {
                front,
                t: distance,
                origin,
            });
        }
        travelled += step;
        match event {
            Event::Collapse(slot) => front.collapse(slot, travelled).ok_or(splits.clone())?,
            Event::Split { vertex, edge } => front
                .split(vertex, edge, travelled)?
                .ok_or(splits.clone())?,
        }
    }
    Err(KernelError::NotConverged {
        iterations: u32::try_from(events).unwrap_or(u32::MAX),
        residual: distance - travelled,
    }
    .into())
}

/// Front at the requested offset.
pub(super) struct Finished<const N: usize> {
    front: Front<N>,
    t: f64,
    origin: [f64; 2],
}

impl<const N: usize> Finished<N> {
    /// Calls `visit` with the vertices of every ring left, in plane
    /// coordinates; stops at the first error.
    pub(super) fn for_each_ring(
        &self,
        mut visit: impl FnMut(&[PlanarPoint]) -> Result<()>,
    ) -> Result<()> {
        let lost = NavigationError::OffsetSplits { distance: self.t };
        let mut seen = [false; N];
        for (first, _) in self.front.alive() {
            if seen.get(first).copied().unwrap_or(true) {
                continue;
            }
            let mut ring: Inline<PlanarPoint, N> = Inline::new(PlanarPoint::ORIGIN);
            let mut at = first;
            for _ in 0..N {
                if let Some(mark) = seen.get_mut(at) {
                    *mark = true;
                }
                let (local, _) = self.front.vertex(at, self.t).ok_or(lost.clone())?;
                let xy = [local[0] + self.origin[0], local[1] + self.origin[1]];
                if !(xy[0].is_finite() && xy[1].is_finite()) {
                    return Err(lost);
                }
                ring.push(PlanarPoint::from_xy(xy))
                    .map_err(|_| lost.clone())?;
                at = self.front.edge(at).ok_or(lost.clone())?.next;
                if at == first {
                    break;
                }
            }
            visit(&ring)?;
        }
        Ok(())
    }
}
