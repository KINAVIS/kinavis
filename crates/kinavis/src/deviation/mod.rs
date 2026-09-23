//! Deviation tables and their interpolation.
//!
//! A deviation table gives, for a set of compass headings, the displacement of
//! compass north from magnetic north caused by the ship's magnetism:
//!
//! ```text
//! magnetic course = compass course + deviation(compass course)
//! ```
//!
//! Deviation is a *periodic* function of the **compass** course, and is treated
//! as such throughout.
//!
//! # Interpolation methods
//!
//! | Method | Continuity | Nodes needed | Use when |
//! |---|---|---|---|
//! | [`InterpolationMethod::Linear`] | C⁰ | 2 | results must never overshoot tabulated values |
//! | [`InterpolationMethod::Cubic`] | C² | 3 | dense swing, smooth curve wanted |
//! | [`InterpolationMethod::Parametric`] | analytic | 5 | classical A–E model, or smoothing a noisy swing |
//!
//! All are periodic: the arc from the last node through `360°/0°` to the first
//! is a real interval, not a flat extrapolation.
//!
//! # Example
//!
//! ```rust
//! use kinavis::{CompassCourse, Deviation, DeviationTable, InterpolationMethod};
//!
//! let mut table = DeviationTable::from_step(90)?;
//! table.set_deviation(0, Deviation::new(10.0)?)?;
//! table.set_deviation(180, Deviation::new(-10.0)?)?;
//!
//! // Halfway between the 270° node (0.0) and the 0° node (10.0), the long way
//! // round through north — a segment a naive lookup does not see.
//! let deviation = table.deviation_at(CompassCourse::new(315.0)?, InterpolationMethod::Linear)?;
//! assert!((deviation.degrees() - 5.0).abs() < 1e-12);
//! # Ok::<(), kinavis::NavigationError>(())
//! ```
//! # Structure
//!
//! The table stores observations; numerical methods are services on it, so new
//! reading methods never modify the storage type.
//!
//! - `table` — [`DeviationTable`]: storage, invariants, mutators.
//! - `node`, `swing` — a table row ([`DeviationNode`]) and an observed heading
//!   ([`SwingObservation`]).
//! - `interpolation` — the four reading methods.
//! - `ring` — nodes as a closed circle.
//! - `smith` — five-coefficient model and least-squares fit,
//!   [`smith_coefficients`].
//! - `analysis` — residuals, extremes, spacing, invertibility, [`analyze`].
//! - `model` — table and coefficients as the kernel's
//!   [`CompassModel`](kinavis_kernel::environment::CompassModel);
//!   [`InterpolatedTable`] for a table with a prepared reading.
//!
//! Submodules are private; types are re-exported flat
//! (`deviation::DeviationTable`).

mod analysis;
mod interpolation;
mod model;
mod node;
mod ring;
mod smith;
mod swing;
mod table;

pub use analysis::{analyze, DeviationAnalysis};
pub use interpolation::{Interpolation, InterpolationMethod};
pub use model::InterpolatedTable;
pub use node::DeviationNode;
pub use smith::{smith_coefficients, DeviationCoefficients, SmithCoefficients};
pub use swing::SwingObservation;
pub use table::{DeviationTable, MAX_TABLE_NODES, STANDARD_TABLE_LEN};

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod test_support {

    use super::DeviationTable;
    use crate::angle::CompassCourse;

    /// Course input type.
    ///
    /// Out-of-range values are rejected at construction, so they cannot reach a
    /// table.
    pub(super) fn heading(degrees: f64) -> CompassCourse {
        CompassCourse::new(degrees).unwrap()
    }

    pub(super) fn readme_table() -> DeviationTable {
        DeviationTable::from_deviations(&[
            -2.5, -0.5, 1.6, 4.4, -1.7, 0.0, 1.0, 0.3, -0.9, 0.5, -1.2, 0.8, -0.3, 1.7, -2.1, 0.4,
            -0.6, 1.2, -1.3, 0.0, 0.9, -1.1, 1.5, -0.7, -13.2, -15.7, -17.9, -19.2, -18.1, 1.8,
            -0.4, 0.7, -0.2, 1.4, -4.4, -2.9,
        ])
        .unwrap()
    }
}
