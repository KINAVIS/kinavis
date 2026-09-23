//! Deviation table storage: sorted, unique, validated nodes.
//!
//! Numerical methods live in [`super::interpolation`] (reading),
//! [`super::smith`] (fitting) and [`super::analysis`] (assessment). Here:
//! storage, invariants, access.

use crate::angle::{CardinalPoint, CompassCourse, Deviation, Variation};
use crate::error::{KernelError, NavigationError, Result};
use crate::math;

use crate::inline::Inline;

use super::analysis;
use super::interpolation::{self, Interpolation, Interpolator};
use super::node::DeviationNode;
use super::swing::SwingObservation;

/// Values expected by [`DeviationTable::from_deviations`]: 0° to 350° in 10°
/// steps.
pub const STANDARD_TABLE_LEN: usize = 36;

/// Maximum nodes per table.
///
/// Stored inline (no allocation), so the size is fixed at compile time. Twice
/// the standard swing — one node per 5° — finer than any practical swing.
pub const MAX_TABLE_NODES: usize = STANDARD_TABLE_LEN * 2;

/// Inline node storage.
type Nodes = Inline<DeviationNode, MAX_TABLE_NODES>;

/// Deviation as a function of compass course.
///
/// Nodes are kept sorted and unique (binary-search lookup, deterministic
/// iteration). Every constructor validates input, so any existing table is
/// usable.
///
/// Up to [`MAX_TABLE_NODES`] nodes stored inline: large, pass by reference;
/// usable without an allocator.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(try_from = "Vec<(i32, f64)>", into = "Vec<(i32, f64)>")
)]
pub struct DeviationTable {
    nodes: Nodes,
}

impl Default for DeviationTable {
    /// Zero-deviation table, 0° to 350° in 10° steps.
    fn default() -> Self {
        let mut nodes = Nodes::new(DeviationNode::zero_at(0));
        for index in 0..STANDARD_TABLE_LEN {
            // `index < 36`: neither multiplication nor push can fail.
            let _ = nodes.push(DeviationNode::zero_at(
                i32::try_from(index).unwrap_or(0) * 10,
            ));
        }
        Self { nodes }
    }
}

impl DeviationTable {
    /// Zero-deviation table with a fixed step.
    ///
    /// # Errors
    ///
    /// - [`NavigationError::InvalidStep`] unless `step` is in `1..=180` (`0`
    ///   would divide by zero; negative would yield a single node).
    /// - [`KernelError::CapacityExceeded`] if the step yields more than
    ///   [`MAX_TABLE_NODES`] headings (step below 5°).
    pub fn from_step(step: i32) -> Result<Self> {
        if !(1..=180).contains(&step) {
            return Err(NavigationError::InvalidStep { step });
        }
        let stride = usize::try_from(step).unwrap_or(1);
        let mut nodes = Nodes::new(DeviationNode::zero_at(0));
        for course in (0..360).step_by(stride) {
            push_node(&mut nodes, DeviationNode::zero_at(course), 360 / stride)?;
        }
        Ok(Self { nodes })
    }

    /// Zero-deviation table on the eight cardinal and intercardinal points.
    #[must_use]
    pub fn from_cardinal_directions() -> Self {
        let mut nodes = Nodes::new(DeviationNode::zero_at(0));
        for point in CardinalPoint::ALL {
            // Eight nodes always fit.
            let _ = nodes.push(DeviationNode::zero_at(point.whole_degrees()));
        }
        nodes
            .as_mut_slice()
            .sort_unstable_by_key(DeviationNode::course);
        Self { nodes }
    }

    /// Table from `(compass course, deviation)` pairs.
    ///
    /// Courses are normalised into `0..360` with Euclidean remainder (`-350` →
    /// `10`); plain `%` would leave an unmatchable negative key.
    ///
    /// # Errors
    ///
    /// - [`KernelError::InsufficientData`] for fewer than two pairs.
    /// - [`NavigationError::DuplicateCourse`] if two pairs normalise to the
    ///   same course.
    /// - [`KernelError::CapacityExceeded`] beyond [`MAX_TABLE_NODES`] pairs.
    /// - [`KernelError::NotFinite`] or [`KernelError::OutOfRange`] for an
    ///   invalid deviation.
    pub fn from_pairs(deviations: &[(i32, f64)]) -> Result<Self> {
        let mut nodes = Nodes::new(DeviationNode::zero_at(0));
        for &(course, deviation) in deviations {
            push_node(
                &mut nodes,
                DeviationNode::new(course, deviation)?,
                deviations.len(),
            )?;
        }
        Self::from_nodes(nodes)
    }

    /// Table from 36 deviations for headings 0°, 10°, … 350°.
    ///
    /// # Errors
    ///
    /// [`NavigationError::UnexpectedTableLength`] unless exactly
    /// [`STANDARD_TABLE_LEN`] values are given (no zero-fill, no truncation).
    pub fn from_deviations(deviations: &[f64]) -> Result<Self> {
        if deviations.len() != STANDARD_TABLE_LEN {
            return Err(NavigationError::UnexpectedTableLength {
                found: deviations.len(),
                expected: STANDARD_TABLE_LEN,
            });
        }
        let mut nodes = Nodes::new(DeviationNode::zero_at(0));
        for (index, &deviation) in deviations.iter().enumerate() {
            // 36 nodes always fit; `index < 36` cannot overflow.
            let node = DeviationNode::new(i32::try_from(index).unwrap_or(0) * 10, deviation)?;
            push_node(&mut nodes, node, STANDARD_TABLE_LEN)?;
        }
        Self::from_nodes(nodes)
    }

    /// Table from raw swing observations.
    ///
    /// A swing yields bearings, not deviations; converting by hand is
    /// error-prone. Headings are rounded to whole degrees (089.6° becomes the
    /// 090° node).
    ///
    /// # Errors
    ///
    /// - [`KernelError::InsufficientData`] for fewer than two observations.
    /// - [`NavigationError::DuplicateCourse`] if two observations round to the
    ///   same heading.
    /// - [`KernelError::CapacityExceeded`] beyond [`MAX_TABLE_NODES`]
    ///   observations.
    /// - [`KernelError::OutOfRange`] if an observation implies an impossible
    ///   deviation.
    pub fn from_swing(observations: &[SwingObservation], variation: Variation) -> Result<Self> {
        let mut nodes = Nodes::new(DeviationNode::zero_at(0));
        for observation in observations {
            let deviation = observation.deviation(variation)?;
            // Validated direction in `[0, 360)`: cannot overflow.
            let heading = math::round_to_i32(observation.compass_heading.degrees());
            push_node(
                &mut nodes,
                DeviationNode::new(heading, deviation.degrees())?,
                observations.len(),
            )?;
        }
        Self::from_nodes(nodes)
    }

    fn from_nodes(mut nodes: Nodes) -> Result<Self> {
        nodes
            .as_mut_slice()
            .sort_unstable_by_key(DeviationNode::course);
        if let Some(duplicate) = nodes
            .windows(2)
            .find(|pair| {
                pair.first().map(DeviationNode::course) == pair.last().map(DeviationNode::course)
            })
            .and_then(|pair| pair.first())
        {
            return Err(NavigationError::DuplicateCourse {
                course: duplicate.course(),
            });
        }
        if nodes.len() < 2 {
            return Err(NavigationError::Kernel(KernelError::InsufficientData {
                found: nodes.len(),
                required: 2,
                context: "a deviation table",
            }));
        }
        Ok(Self { nodes })
    }

    /// Nodes, sorted by compass course.
    #[must_use]
    pub fn nodes(&self) -> &[DeviationNode] {
        &self.nodes
    }

    /// Node count, at least two.
    #[must_use]
    pub fn len(&self) -> usize {
        self.nodes.len()
    }

    /// Always `false`: tables are never empty.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.nodes.is_empty()
    }

    /// Replaces the deviation at an existing node.
    ///
    /// The course is normalised first; the value is a validated [`Deviation`],
    /// so only the course can fail.
    ///
    /// # Errors
    ///
    /// [`NavigationError::CourseNotInTable`] if the course is not a node; use
    /// [`DeviationTable::insert_deviation`] to add one.
    pub fn set_deviation(&mut self, course: i32, deviation: Deviation) -> Result<()> {
        let course = course.rem_euclid(360);
        match self
            .nodes
            .binary_search_by_key(&course, DeviationNode::course)
        {
            Ok(index) => {
                if let Some(node) = self.nodes.as_mut_slice().get_mut(index) {
                    node.set_deviation_degrees(deviation.degrees());
                }
                Ok(())
            }
            Err(_) => Err(NavigationError::CourseNotInTable { course }),
        }
    }

    /// Sets the deviation at a course, adding the node if absent.
    ///
    /// # Errors
    ///
    /// [`KernelError::CapacityExceeded`] if the table is full and the course is
    /// new.
    pub fn insert_deviation(&mut self, course: i32, deviation: Deviation) -> Result<()> {
        let course = course.rem_euclid(360);
        match self
            .nodes
            .binary_search_by_key(&course, DeviationNode::course)
        {
            Ok(index) => {
                if let Some(node) = self.nodes.as_mut_slice().get_mut(index) {
                    node.set_deviation_degrees(deviation.degrees());
                }
            }
            Err(index) => {
                let node = DeviationNode::with_deviation(course, deviation);
                let needed = self.nodes.len().saturating_add(1);
                self.nodes.insert(index, node).map_err(|full| {
                    NavigationError::Kernel(KernelError::CapacityExceeded {
                        context: "a deviation table",
                        needed,
                        capacity: full.capacity,
                    })
                })?;
            }
        }
        Ok(())
    }

    /// Replaces the deviation at a cardinal point.
    ///
    /// # Errors
    ///
    /// [`NavigationError::CourseNotInTable`] if the point is not a node; use
    /// [`DeviationTable::insert_deviation`] to add one.
    ///
    /// # Example
    ///
    /// ```rust
    /// use kinavis::{CardinalPoint, Deviation, DeviationTable};
    ///
    /// let mut table = DeviationTable::from_cardinal_directions();
    /// table.set_deviation_at(CardinalPoint::N, Deviation::new(-2.5)?)?;
    /// assert_eq!(table.deviation_at_point(CardinalPoint::N)?.degrees(), -2.5);
    /// # Ok::<(), kinavis::NavigationError>(())
    /// ```
    pub fn set_deviation_at(&mut self, point: CardinalPoint, deviation: Deviation) -> Result<()> {
        self.set_deviation(point.whole_degrees(), deviation)
    }

    /// Tabulated deviation at a cardinal point.
    ///
    /// # Errors
    ///
    /// [`NavigationError::CourseNotInTable`] if the point is not a node (steps
    /// not dividing 45°); use [`DeviationTable::deviation_at`] to interpolate.
    pub fn deviation_at_point(&self, point: CardinalPoint) -> Result<Deviation> {
        let course = point.whole_degrees();
        self.deviation_at_node(course)
            .ok_or(NavigationError::CourseNotInTable { course })
    }

    /// Tabulated deviation at an exact node, no interpolation.
    #[must_use]
    pub fn deviation_at_node(&self, course: i32) -> Option<Deviation> {
        let course = course.rem_euclid(360);
        self.nodes
            .binary_search_by_key(&course, DeviationNode::course)
            .ok()
            .and_then(|index| self.nodes.get(index))
            .map(DeviationNode::deviation)
    }

    /// Maximum gap between adjacent nodes, degrees, around the full circle.
    #[must_use]
    pub fn max_gap(&self) -> f64 {
        analysis::max_gap(&self.nodes)
    }

    /// Maximum node-to-node rate of change of deviation, degrees per degree.
    ///
    /// Determines invertibility: at 1° of deviation per degree of heading, two
    /// compass courses give the same magnetic course and the inverse is no
    /// longer unique. See [`DeviationTable::is_invertible`].
    #[must_use]
    pub fn max_slope(&self) -> f64 {
        analysis::max_slope(&self.nodes)
    }

    /// Whether each magnetic course maps back to exactly one compass course.
    ///
    /// `false`: the compass is unsteerable over part of the circle and should
    /// be re-swung or adjusted. Conversions still work —
    /// [`crate::navigation_solutions::convert_true_course_to_compass_course`]
    /// returns *a* compass course producing the requested true course — but not
    /// necessarily the original one.
    #[must_use]
    pub fn is_invertible(&self) -> bool {
        analysis::is_invertible(&self.nodes)
    }

    /// Maximum tabulated deviation magnitude, degrees.
    #[must_use]
    pub fn max_abs_deviation(&self) -> f64 {
        analysis::max_abs_deviation(&self.nodes)
    }

    /// Interpolated deviation at one compass course.
    ///
    /// The course is a [`CompassCourse`], so no range check is needed.
    ///
    /// `interpolation` is a single parameter: a bare
    /// [`InterpolationMethod`](super::InterpolationMethod) normally, or an
    /// [`Interpolation`] with pinned coefficients.
    ///
    /// # Errors
    ///
    /// - [`KernelError::InsufficientData`] or [`KernelError::SingularSystem`]
    ///   if a parametric fit is impossible.
    /// - [`KernelError::OutOfRange`] if the result is not a valid deviation
    ///   (not possible for a table within range).
    ///
    /// # Example
    ///
    /// ```rust
    /// use kinavis::{CompassCourse, Deviation, DeviationTable, InterpolationMethod};
    ///
    /// let mut table = DeviationTable::from_step(90)?;
    /// table.set_deviation(0, Deviation::new(10.0)?)?;
    ///
    /// let deviation = table.deviation_at(
    ///     CompassCourse::new(315.0)?,
    ///     InterpolationMethod::Linear,
    /// )?;
    /// assert!((deviation.degrees() - 5.0).abs() < 1e-12);
    /// # Ok::<(), kinavis::NavigationError>(())
    /// ```
    pub fn deviation_at<'a>(
        &self,
        course: CompassCourse,
        interpolation: impl Into<Interpolation<'a>>,
    ) -> Result<Deviation> {
        let interpolator = interpolation::prepare(self, interpolation.into())?;
        Ok(Deviation::new(interpolator.evaluate(course.degrees()))?)
    }

    /// Interpolated deviation for several compass courses.
    ///
    /// Prepares the spline or fit once for the batch. Results go into
    /// caller-owned `out`, so no allocator is needed.
    ///
    /// # Errors
    ///
    /// - [`KernelError::BufferTooSmall`] if `out` is shorter than `courses`.
    /// - Otherwise as [`DeviationTable::deviation_at`].
    ///
    /// # Example
    ///
    /// ```rust
    /// use kinavis::{CompassCourse, Deviation, DeviationTable, InterpolationMethod};
    ///
    /// let table = DeviationTable::from_deviations(&[1.0; 36])?;
    /// let courses = [CompassCourse::new(5.0)?, CompassCourse::new(185.0)?];
    /// let mut deviations = [Deviation::ZERO; 2];
    ///
    /// table.interpolate_deviation(&courses, InterpolationMethod::Linear, &mut deviations)?;
    /// assert!((deviations[0].degrees() - 1.0).abs() < 1e-12);
    /// # Ok::<(), kinavis::NavigationError>(())
    /// ```
    pub fn interpolate_deviation<'a>(
        &self,
        courses: &[CompassCourse],
        interpolation: impl Into<Interpolation<'a>>,
        out: &mut [Deviation],
    ) -> Result<()> {
        if out.len() < courses.len() {
            return Err(NavigationError::Kernel(KernelError::BufferTooSmall {
                needed: courses.len(),
                found: out.len(),
            }));
        }
        let interpolator = interpolation::prepare(self, interpolation.into())?;
        for (course, slot) in courses.iter().zip(out.iter_mut()) {
            *slot = Deviation::new(interpolator.evaluate(course.degrees()))?;
        }
        Ok(())
    }
}

#[cfg(feature = "serde")]
use alloc::vec::Vec;

#[cfg(feature = "serde")]
impl TryFrom<Vec<(i32, f64)>> for DeviationTable {
    type Error = NavigationError;

    /// Deserialised through [`DeviationTable::from_pairs`]: duplicates,
    /// non-finite values, too few nodes and more than [`MAX_TABLE_NODES`] are
    /// rejected.
    fn try_from(nodes: Vec<(i32, f64)>) -> Result<Self> {
        Self::from_pairs(&nodes)
    }
}

#[cfg(feature = "serde")]
impl From<DeviationTable> for Vec<(i32, f64)> {
    fn from(table: DeviationTable) -> Self {
        table
            .nodes
            .iter()
            .map(|node| (node.course(), node.deviation_degrees()))
            .collect()
    }
}

#[cfg(test)]
#[path = "table_tests.rs"]
mod tests;

/// Appends a node, or reports the required capacity.
fn push_node(nodes: &mut Nodes, node: DeviationNode, needed: usize) -> Result<()> {
    nodes.push(node).map_err(|full| {
        NavigationError::Kernel(KernelError::CapacityExceeded {
            context: "a deviation table",
            needed,
            capacity: full.capacity,
        })
    })
}
