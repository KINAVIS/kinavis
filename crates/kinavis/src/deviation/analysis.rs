//! Swing analysis: fit quality of the classical model, deviation from it,
//! invertibility.
//!
//! Derived on demand from the nodes, not maintained by the table.

use crate::math;

use super::node::DeviationNode;
use super::ring::NodeRing;
use super::smith::{self, SmithCoefficients};
use super::table::DeviationTable;
use crate::error::Result;

/// Swing summary from [`analyze`].
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct DeviationAnalysis {
    /// Least-squares five-coefficient fit.
    pub coefficients: SmithCoefficients,
    /// RMS residual of the fit, degrees.
    ///
    /// Large values indicate deviation not described by the classical model, or
    /// a bad observation in the swing.
    pub rms_residual: f64,
    /// Maximum residual, degrees.
    pub max_residual: f64,
    /// Maximum tabulated deviation magnitude, degrees.
    pub max_abs_deviation: f64,
    /// Maximum gap between adjacent nodes, degrees, periodic.
    pub max_gap: f64,
    /// Maximum node-to-node slope, degrees of deviation per degree of heading.
    ///
    /// See [`DeviationTable::max_slope`]; at `1.0` or above the table is not
    /// uniquely invertible.
    pub max_slope: f64,
    /// Node count.
    pub nodes: usize,
}

/// Table summary: fitted coefficients, residuals, extremes, node spacing.
///
/// # Errors
///
/// As [`smith::smith_coefficients`]: at least five nodes that constrain the
/// model.
///
/// # Example
///
/// ```rust
/// use kinavis::deviation::{analyze, DeviationTable};
///
/// let table = DeviationTable::from_deviations(&[0.0; 36])?;
/// let summary = analyze(&table)?;
/// assert_eq!(summary.nodes, 36);
/// assert!(summary.max_gap - 10.0 < 1e-12);
/// # Ok::<(), kinavis::NavigationError>(())
/// ```
pub fn analyze(table: &DeviationTable) -> Result<DeviationAnalysis> {
    let nodes = table.nodes();
    let coefficients = smith::smith_coefficients(table)?;
    let residuals = smith::residuals(nodes, &coefficients);
    Ok(DeviationAnalysis {
        coefficients,
        rms_residual: residuals.rms,
        max_residual: residuals.largest,
        max_abs_deviation: max_abs_deviation(nodes),
        max_gap: max_gap(nodes),
        max_slope: max_slope(nodes),
        nodes: nodes.len(),
    })
}

/// Maximum gap between adjacent nodes, degrees, around the full circle.
#[must_use]
pub(crate) fn max_gap(nodes: &[DeviationNode]) -> f64 {
    let ring = NodeRing::new(nodes);
    (0..nodes.len()).fold(0.0_f64, |widest, index| widest.max(ring.span(index)))
}

/// Maximum node-to-node rate of change of deviation, degrees per degree.
#[must_use]
pub(crate) fn max_slope(nodes: &[DeviationNode]) -> f64 {
    let ring = NodeRing::new(nodes);
    (0..nodes.len()).fold(0.0_f64, |steepest, index| {
        let span = ring.span(index);
        if span > 0.0 {
            let rise = ring.value(ring.after(index)) - ring.value(index);
            steepest.max(math::abs(rise) / span)
        } else {
            steepest
        }
    })
}

/// Maximum tabulated deviation magnitude, degrees.
#[must_use]
pub(crate) fn max_abs_deviation(nodes: &[DeviationNode]) -> f64 {
    nodes.iter().fold(0.0_f64, |largest, node| {
        largest.max(math::abs(node.deviation_degrees()))
    })
}

/// Whether each magnetic course maps back to exactly one compass course.
#[must_use]
pub(crate) fn is_invertible(nodes: &[DeviationNode]) -> bool {
    max_slope(nodes) < 1.0
}
