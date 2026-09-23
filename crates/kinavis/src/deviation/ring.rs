//! Table nodes as a closed ring.
//!
//! Deviation is periodic, so the arc from the last node through `360°/0°` to
//! the first is an ordinary segment. Interpolation, slope and spacing all use
//! this view.

use crate::math;

use super::node::DeviationNode;

/// Table nodes as a closed ring.
///
/// Indices are taken modulo the node count, so the segment crossing north needs
/// no special case.
#[derive(Debug, Clone, Copy)]
pub(crate) struct NodeRing<'a> {
    nodes: &'a [DeviationNode],
}

impl<'a> NodeRing<'a> {
    /// Wraps a table's nodes (sorted, unique, at least two).
    pub(crate) const fn new(nodes: &'a [DeviationNode]) -> Self {
        Self { nodes }
    }

    /// Node count.
    pub(crate) const fn count(self) -> usize {
        self.nodes.len()
    }

    /// Deviation at a node, degrees.
    pub(crate) fn value(self, index: usize) -> f64 {
        let count = self.nodes.len().max(1);
        self.nodes
            .get(index % count)
            .map_or(0.0, DeviationNode::deviation_degrees)
    }

    /// Compass course of a node, degrees.
    pub(crate) fn course(self, index: usize) -> f64 {
        let count = self.nodes.len().max(1);
        self.nodes
            .get(index % count)
            .map_or(0.0, DeviationNode::course_degrees)
    }

    /// Index after `index` on the ring. `index` is below the count, so the
    /// step cannot wrap; saturating arithmetic states it.
    pub(crate) fn after(self, index: usize) -> usize {
        index.saturating_add(1) % self.count().max(1)
    }

    /// Index before `index` on the ring.
    pub(crate) fn before(self, index: usize) -> usize {
        let count = self.count().max(1);
        index.saturating_add(count).saturating_sub(1) % count
    }

    /// Angular width of the segment starting at `index`; the last one closes
    /// the circle through `360°/0°`.
    pub(crate) fn span(self, index: usize) -> f64 {
        if index.saturating_add(1) < self.count() {
            self.course(self.after(index)) - self.course(index)
        } else {
            360.0 - self.course(index) + self.course(0)
        }
    }

    /// Segment containing `course`, on the closed circle.
    pub(crate) fn locate(self, course: f64) -> Segment {
        let count = self.count();
        let last_index = count.saturating_sub(1);
        let first = self.course(0);
        let last = self.course(last_index);
        let wrap_span = self.span(last_index);

        if course < first {
            // Between the last and first node, past 360°/0°.
            return Segment {
                index: last_index,
                span: wrap_span,
                offset: course + 360.0 - last,
            };
        }

        let index = self
            .nodes
            .partition_point(|node| node.course_degrees() <= course)
            .saturating_sub(1);

        if index >= last_index {
            Segment {
                index: last_index,
                span: wrap_span,
                offset: course - last,
            }
        } else {
            let start = self.course(index);
            Segment {
                index,
                span: self.course(index + 1) - start,
                offset: course - start,
            }
        }
    }

    /// Classical `h²·|f''|/8` interpolation error bound.
    ///
    /// `f''` is approximated by the local second difference. Valid for every
    /// method passing through the nodes; the parametric fit reports its
    /// residual instead.
    pub(crate) fn local_error_bound(self, course: f64) -> f64 {
        let segment = self.locate(course);
        let second_difference = |centre: usize| {
            let previous = self.value(self.before(centre));
            let current = self.value(centre);
            let next = self.value(self.after(centre));
            math::abs(previous - 2.0 * current + next)
        };
        let left = second_difference(segment.index);
        let right = second_difference(self.after(segment.index));
        left.max(right) / 8.0
    }
}

/// Position of a course within the ring's segments.
pub(crate) struct Segment {
    /// Start node index.
    pub(crate) index: usize,
    /// Segment width, degrees; always positive.
    pub(crate) span: f64,
    /// Offset from the segment start, degrees.
    pub(crate) offset: f64,
}

impl Segment {
    pub(crate) fn fraction(&self) -> f64 {
        self.offset / self.span
    }
}
