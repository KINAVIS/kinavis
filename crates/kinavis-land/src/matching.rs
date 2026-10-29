//! Map matching: which road each fix of a vehicle lies on, by the hidden
//! Markov model of Newson and Krumm (2009).
//!
//! Each fix has candidates, the nearest points of the roads around it. A
//! candidate is likely if it is close to the fix (Gaussian, σ of the fix's
//! scatter) and if the drive to it from a candidate of the previous fix is
//! about as long as the straight line between the fixes (exponential in the
//! difference, scale β). The most likely sequence is found by Viterbi over a
//! fixed window: a fix's road is decided once `W` fixes have followed it, in
//! bounded memory, and a fix that no candidate reaches breaks the chain and
//! starts a new one.
//!
//! The road network is the application's: its index and its routing sit
//! behind [`RoadNetwork`]. The matcher holds `K` candidates a fix and `W`
//! fixes, nothing else.

use kinavis::sailings::geodesic;
use kinavis_kernel::error::ensure_range;
use kinavis_kernel::{Distance, Position};

use crate::error::Result;

/// Identifier of a road (an edge of the network), the application's.
pub type RoadId = u64;

/// A point on a road near a fix.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Candidate {
    /// Road.
    pub road: RoadId,
    /// Whether it is driven from its start to its end; a two-way road gives
    /// a candidate each way.
    pub forward: bool,
    /// Nearest point of the road to the fix.
    pub point: Position,
    /// Distance along the road in its direction of travel to the point.
    pub offset: Distance,
    /// Distance from the fix to the point.
    pub distance: Distance,
}

/// The road network a matcher works on: the application's index and router.
pub trait RoadNetwork {
    /// Candidates within `radius` of `position`, nearest first, into `out`;
    /// returns how many were written.
    fn candidates(&self, position: Position, radius: Distance, out: &mut [Candidate]) -> usize;

    /// Driving distance along the network from one candidate to another, or
    /// `None` if it is longer than `limit` or there is no way.
    fn route_distance(&self, from: &Candidate, to: &Candidate, limit: Distance)
        -> Option<Distance>;
}

/// Settings of a matcher.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MatcherConfig {
    sigma: f64,
    beta: f64,
    radius: Distance,
    detour: f64,
}

impl MatcherConfig {
    /// Fix scatter `sigma`, transition scale `beta`, candidate search
    /// `radius`, and the longest `detour` a drive between two fixes may make
    /// beyond the straight line between them.
    ///
    /// Newson and Krumm's values for their 1 Hz drive: σ 4.07 m, β 3 m
    /// (at 1 s; β grows with the interval), radius 200 m, detour 2 km.
    ///
    /// # Errors
    ///
    /// [`kinavis_kernel::KernelError::OutOfRange`] for σ or β outside 0.1 m
    /// to 1 km, a radius outside 1 m to 10 km or a detour outside 0 to 100 km.
    pub fn new(
        sigma: Distance,
        beta: Distance,
        radius: Distance,
        detour: Distance,
    ) -> Result<Self> {
        ensure_range("fix scatter, m", sigma.metres(), 0.1, 1_000.0)?;
        ensure_range("transition scale, m", beta.metres(), 0.1, 1_000.0)?;
        ensure_range("candidate radius, m", radius.metres(), 1.0, 10_000.0)?;
        ensure_range("detour, m", detour.metres(), 0.0, 100_000.0)?;
        Ok(Self {
            sigma: sigma.metres(),
            beta: beta.metres(),
            radius,
            detour: detour.metres(),
        })
    }
}

/// A fix matched to a road.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Matched {
    /// Index of the fix among those pushed, from 0.
    pub index: u64,
    /// The candidate chosen.
    pub candidate: Candidate,
    /// Whether the chain broke just before this fix: no candidate of it is
    /// reachable from the previous fix's.
    pub after_break: bool,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct State {
    candidate: Candidate,
    score: f64,
    back: usize,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct Column<const K: usize> {
    index: u64,
    position: Position,
    states: [Option<State>; K],
    after_break: bool,
}

/// Map matcher holding `K` candidates a fix and deciding a fix once `W`
/// fixes have followed it.
#[derive(Debug, Clone, PartialEq)]
pub struct MapMatcher<const K: usize, const W: usize> {
    config: MatcherConfig,
    columns: [Option<Column<K>>; W],
    /// Oldest column in the ring.
    head: usize,
    len: usize,
    pushed: u64,
}

impl<const K: usize, const W: usize> MapMatcher<K, W> {
    /// Matcher with its settings.
    #[must_use]
    pub const fn new(config: MatcherConfig) -> Self {
        Self {
            config,
            columns: [None; W],
            head: 0,
            len: 0,
            pushed: 0,
        }
    }

    /// Fixes waiting for their decision.
    #[must_use]
    pub const fn pending(&self) -> usize {
        self.len
    }

    /// Takes the next fix; the fixes it decides go to `matched`, oldest
    /// first. A fix with no road within the radius, or within 2σ of the last
    /// fix kept, is passed over: it is never matched.
    ///
    /// # Errors
    ///
    /// The network's positions, if they are not on Earth.
    pub fn push(
        &mut self,
        network: &impl RoadNetwork,
        position: Position,
        mut matched: impl FnMut(Matched),
    ) -> Result<()> {
        let index = self.pushed;
        self.pushed = self.pushed.saturating_add(1);
        // Newson and Krumm: a fix within 2σ of the last one kept says nothing
        // the scatter does not, and a stopped vehicle's jitter would drive it
        // backwards along its road.
        if let Some(previous) = self.newest() {
            if geodesic(previous.position, position)?.distance.metres() < 2.0 * self.config.sigma {
                return Ok(());
            }
        }
        let mut found = [None; K];
        let count = gather(network, position, self.config.radius, &mut found);
        if count == 0 {
            return Ok(());
        }
        let mut column = Column {
            index,
            position,
            states: [None; K],
            after_break: false,
        };
        let previous = self.newest();
        let mut reached = false;
        if let Some(previous) = previous {
            let straight = geodesic(previous.position, position)?.distance.metres();
            let limit = Distance::from_metres(straight + self.config.detour)?;
            for (slot, candidate) in column.states.iter_mut().zip(found.iter().flatten()) {
                let emission = self.emission(candidate);
                let mut best: Option<(f64, usize)> = None;
                for (back, state) in previous.states.iter().enumerate() {
                    let Some(state) = state else {
                        continue;
                    };
                    let Some(route) = network.route_distance(&state.candidate, candidate, limit)
                    else {
                        continue;
                    };
                    let transition = -(route.metres() - straight).abs() / self.config.beta;
                    let score = state.score + transition;
                    if best.is_none_or(|(top, _)| score > top) {
                        best = Some((score, back));
                    }
                }
                if let Some((score, back)) = best {
                    reached = true;
                    *slot = Some(State {
                        candidate: *candidate,
                        score: score + emission,
                        back,
                    });
                }
            }
        }
        if !reached {
            // A new chain: decide the whole window on the old one first.
            while self.len > 0 {
                self.decide(&mut matched);
            }
            column.after_break = previous.is_some();
            for (slot, candidate) in column.states.iter_mut().zip(found.iter().flatten()) {
                *slot = Some(State {
                    candidate: *candidate,
                    score: self.emission(candidate),
                    back: 0,
                });
            }
        }
        renormalise(&mut column);
        if self.len == W {
            self.decide(&mut matched);
        }
        let tail = (self.head + self.len) % W.max(1);
        if let Some(slot) = self.columns.get_mut(tail) {
            *slot = Some(column);
            self.len = self.len.saturating_add(1);
        }
        Ok(())
    }

    /// Decides every fix still waiting, as at the end of a drive.
    pub fn finish(&mut self, mut matched: impl FnMut(Matched)) {
        while self.len > 0 {
            self.decide(&mut matched);
        }
    }

    fn emission(&self, candidate: &Candidate) -> f64 {
        let z = candidate.distance.metres() / self.config.sigma;
        -0.5 * z * z
    }

    fn newest(&self) -> Option<Column<K>> {
        if self.len == 0 {
            return None;
        }
        let at = (self.head + self.len - 1) % W.max(1);
        self.columns.get(at).copied().flatten()
    }

    /// Decides the oldest fix by tracing back from the best state of the
    /// newest, and drops it.
    fn decide(&mut self, matched: &mut impl FnMut(Matched)) {
        let size = W.max(1);
        let newest = (self.head + self.len.saturating_sub(1)) % size;
        let Some(Some(last)) = self.columns.get(newest) else {
            return;
        };
        let mut choice = best_state(last);
        // Back from the newest to the oldest column.
        for step in (1..self.len).rev() {
            let at = (self.head + step) % size;
            let Some(Some(column)) = self.columns.get(at) else {
                break;
            };
            choice = column
                .states
                .get(choice)
                .copied()
                .flatten()
                .map_or(0, |state| state.back);
        }
        if let Some(Some(oldest)) = self.columns.get(self.head) {
            if let Some(Some(state)) = oldest.states.get(choice) {
                matched(Matched {
                    index: oldest.index,
                    candidate: state.candidate,
                    after_break: oldest.after_break,
                });
            }
        }
        if let Some(slot) = self.columns.get_mut(self.head) {
            *slot = None;
        }
        self.head = (self.head + 1) % size;
        self.len = self.len.saturating_sub(1);
    }
}

/// Keeps scores near zero over a long drive: subtracts the best.
fn renormalise<const K: usize>(column: &mut Column<K>) {
    let top = column
        .states
        .iter()
        .flatten()
        .map(|state| state.score)
        .fold(f64::NEG_INFINITY, f64::max);
    if top.is_finite() {
        for state in column.states.iter_mut().flatten() {
            state.score -= top;
        }
    }
}

fn best_state<const K: usize>(column: &Column<K>) -> usize {
    let mut best = (f64::NEG_INFINITY, 0);
    for (index, state) in column.states.iter().enumerate() {
        if let Some(state) = state {
            if state.score > best.0 {
                best = (state.score, index);
            }
        }
    }
    best.1
}

/// The `K` nearest candidates the network gives, nearest first.
fn gather<const K: usize>(
    network: &impl RoadNetwork,
    position: Position,
    radius: Distance,
    out: &mut [Option<Candidate>; K],
) -> usize {
    let blank = Candidate {
        road: 0,
        forward: true,
        point: position,
        offset: Distance::ZERO,
        distance: Distance::ZERO,
    };
    let mut buffer = [blank; K];
    let count = network.candidates(position, radius, &mut buffer).min(K);
    let given = buffer.get_mut(..count).unwrap_or(&mut []);
    given.sort_unstable_by(|a, b| a.distance.metres().total_cmp(&b.distance.metres()));
    for (slot, candidate) in out.iter_mut().zip(given.iter()) {
        *slot = Some(*candidate);
    }
    count
}
