//! Deviation table row.

use crate::angle::{Deviation, MAX_DEVIATION_DEG};
#[cfg(feature = "serde")]
use crate::error::NavigationError;
use crate::error::{ensure_range, Result};

/// Deviation table row.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(try_from = "(i32, f64)", into = "(i32, f64)")
)]
pub struct DeviationNode {
    course: i32,
    deviation: f64,
}

impl DeviationNode {
    /// Compass course, `0..360` degrees.
    #[must_use]
    pub const fn course(&self) -> i32 {
        self.course
    }

    /// Tabulated deviation.
    #[must_use]
    pub fn deviation(&self) -> Deviation {
        // Validated on insertion into the table.
        Deviation::new(self.deviation).unwrap_or(Deviation::ZERO)
    }

    /// Tabulated deviation, degrees.
    #[must_use]
    pub const fn deviation_degrees(&self) -> f64 {
        self.deviation
    }
}

impl DeviationNode {
    /// Node with the course normalised into `0..360`.
    ///
    /// # Errors
    ///
    /// [`KernelError::NotFinite`](crate::KernelError::NotFinite) or
    /// [`KernelError::OutOfRange`](crate::KernelError::OutOfRange) for an
    /// invalid deviation.
    pub(crate) fn new(course: i32, deviation: f64) -> Result<Self> {
        ensure_range(
            "deviation",
            deviation,
            -MAX_DEVIATION_DEG,
            MAX_DEVIATION_DEG,
        )?;
        Ok(Self {
            course: course.rem_euclid(360),
            deviation,
        })
    }

    /// Node from an already validated deviation.
    pub(crate) fn with_deviation(course: i32, deviation: Deviation) -> Self {
        Self {
            course: course.rem_euclid(360),
            deviation: deviation.degrees(),
        }
    }

    /// Zero-deviation node, for initialising a table.
    pub(crate) fn zero_at(course: i32) -> Self {
        Self {
            course: course.rem_euclid(360),
            deviation: 0.0,
        }
    }

    /// Compass course, degrees.
    #[must_use]
    pub fn course_degrees(&self) -> f64 {
        f64::from(self.course)
    }

    /// Overwrites the value; the caller has validated it.
    pub(crate) fn set_deviation_degrees(&mut self, deviation: f64) {
        self.deviation = deviation;
    }
}

#[cfg(feature = "serde")]
impl TryFrom<(i32, f64)> for DeviationNode {
    type Error = NavigationError;

    /// Validated on deserialisation: no invalid deviation or course outside
    /// `0..360`.
    fn try_from((course, deviation): (i32, f64)) -> Result<Self> {
        Self::new(course, deviation)
    }
}

#[cfg(feature = "serde")]
impl From<DeviationNode> for (i32, f64) {
    fn from(node: DeviationNode) -> Self {
        (node.course, node.deviation)
    }
}
