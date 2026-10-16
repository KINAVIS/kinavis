//! Snapshot RAIM by the residual method: weighted least squares over the
//! satellites used, a chi-square test of the residuals, protection levels
//! from the largest slope, and exclusion of one faulty satellite.
//!
//! References: Brown (1992), "A baseline GPS RAIM scheme and a note on the
//! equivalence of three RAIM methods", Navigation 39(3); Walter & Enge
//! (1995), "Weighted RAIM for precision approach", ION GPS-95.

use kinavis_kernel::error::{ensure_range, Result};
use kinavis_kernel::{math, Constellation, Distance, SatelliteId, SatelliteSet, SatelliteView};

use crate::stats::{bias_noncentrality, chi_square_quantile};

/// Most satellites taken into one solution.
pub const MAX_RAIM_SATELLITES: usize = 64;

/// Unknowns: east, north, up, and one receiver clock per constellation.
const MAX_UNKNOWNS: usize = 3 + 7;

/// Elevation below which the elevation weighting stops growing, degrees.
const WEIGHTING_FLOOR_DEGREES: f64 = 5.0;

/// Range error model: the standard deviation of each satellite's range.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RangeError {
    /// The same σ for every satellite.
    Uniform(Distance),
    /// `σ / sin(elevation)`, elevation floored at 5°: the low satellites
    /// through more atmosphere and multipath.
    ElevationWeighted(Distance),
}

impl RangeError {
    fn sigma(self, elevation_degrees: f64) -> f64 {
        match self {
            Self::Uniform(sigma) => sigma.metres(),
            Self::ElevationWeighted(sigma) => {
                let elevation = elevation_degrees.max(WEIGHTING_FLOOR_DEGREES);
                sigma.metres() / math::sin(math::to_radians(elevation))
            }
        }
    }
}

/// Risk allocation and error model of the monitor.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RaimConfig {
    probability_false_alarm: f64,
    probability_missed_detection: f64,
    range_error: RangeError,
}

impl RaimConfig {
    /// Monitor with a probability of false alarm per test, a probability of
    /// missed detection, and a range error model.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`](kinavis_kernel::KernelError::OutOfRange) for a probability outside `(0, 1)` or a
    /// non-positive σ.
    pub fn new(
        probability_false_alarm: f64,
        probability_missed_detection: f64,
        range_error: RangeError,
    ) -> Result<Self> {
        ensure_range(
            "probability of false alarm",
            probability_false_alarm,
            f64::MIN_POSITIVE,
            0.5,
        )?;
        ensure_range(
            "probability of missed detection",
            probability_missed_detection,
            f64::MIN_POSITIVE,
            0.5,
        )?;
        let (RangeError::Uniform(sigma) | RangeError::ElevationWeighted(sigma)) = range_error;
        ensure_range("range error", sigma.metres(), f64::MIN_POSITIVE, f64::MAX)?;
        Ok(Self {
            probability_false_alarm,
            probability_missed_detection,
            range_error,
        })
    }
}

/// Horizontal and vertical protection levels.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Protection {
    /// Horizontal protection level.
    pub horizontal: Distance,
    /// Vertical protection level.
    pub vertical: Distance,
    /// Satellites in the solution.
    pub satellites: u8,
}

/// Why integrity could not be assessed.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Unavailability {
    /// Fewer satellites than unknowns plus one (plus two for exclusion).
    TooFewSatellites {
        /// Satellites with geometry (and residual, for detection).
        usable: u8,
        /// Satellites needed.
        needed: u8,
    },
    /// The geometry does not determine the position (all satellites on one
    /// azimuth, or a constellation's clock not separable).
    Geometry,
}

/// Outcome of one integrity test.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum IntegrityReport {
    /// No fault detected; the protection levels hold.
    Available {
        /// Protection levels.
        protection: Protection,
        /// Test statistic: root of the weighted sum of squared residuals.
        statistic: f64,
        /// Detection threshold.
        threshold: f64,
    },
    /// The test detected a fault.
    FaultDetected {
        /// Satellite whose exclusion leaves a consistent solution, if one does.
        excluded: Option<SatelliteId>,
        /// Protection levels after the exclusion.
        after_exclusion: Option<Protection>,
        /// Test statistic.
        statistic: f64,
        /// Detection threshold.
        threshold: f64,
    },
    /// Integrity cannot be assessed.
    Unavailable(Unavailability),
}

/// One satellite of the solution.
#[derive(Debug, Clone, Copy)]
struct Row {
    id: Option<SatelliteId>,
    /// Partial derivatives: −line of sight east, north, up, then clocks.
    geometry: [f64; MAX_UNKNOWNS],
    weight: f64,
    sigma: f64,
    residual: f64,
}

const EMPTY_ROW: Row = Row {
    id: None,
    geometry: [0.0; MAX_UNKNOWNS],
    weight: 0.0,
    sigma: 0.0,
    residual: 0.0,
};

/// The satellites of a solution, at most [`MAX_RAIM_SATELLITES`].
#[derive(Clone, Copy)]
struct Solution {
    rows: [Row; MAX_RAIM_SATELLITES],
    count: usize,
    unknowns: usize,
}

/// Weighted least-squares quantities of a geometry.
struct Fit {
    /// `S = (Gᵀ W G)⁻¹ Gᵀ W`, rows east, north, up; one column per satellite.
    slopes: [[f64; MAX_RAIM_SATELLITES]; 3],
    /// Diagonal of `I − G S`.
    parity: [f64; MAX_RAIM_SATELLITES],
    /// Normal-matrix inverse.
    inverse: [[f64; MAX_UNKNOWNS]; MAX_UNKNOWNS],
}

impl Solution {
    /// The used satellites with sky position (and residual, when `residuals`).
    fn from_set<const N: usize>(set: &SatelliteSet<N>, error: RangeError, residuals: bool) -> Self {
        let mut solution = Self {
            rows: [EMPTY_ROW; MAX_RAIM_SATELLITES],
            count: 0,
            unknowns: 3,
        };
        let mut clocks: [Option<Constellation>; MAX_UNKNOWNS - 3] = [None; MAX_UNKNOWNS - 3];
        for view in set.iter().filter(|view| view.is_used()) {
            let Some(row) = row(view, error, residuals, &mut clocks) else {
                continue;
            };
            if let Some(slot) = solution.rows.get_mut(solution.count) {
                *slot = row;
                solution.count = solution.count.saturating_add(1);
            }
        }
        solution.unknowns = 3 + clocks.iter().flatten().count();
        solution
    }

    fn rows(&self) -> &[Row] {
        self.rows.get(..self.count).unwrap_or(&[])
    }

    /// The same solution without one satellite.
    fn without(&self, excluded: usize) -> Self {
        let mut solution = Self {
            rows: [EMPTY_ROW; MAX_RAIM_SATELLITES],
            count: 0,
            unknowns: self.unknowns,
        };
        for (index, row) in self.rows().iter().enumerate() {
            if index != excluded {
                if let Some(slot) = solution.rows.get_mut(solution.count) {
                    *slot = *row;
                    solution.count = solution.count.saturating_add(1);
                }
            }
        }
        solution
    }

    fn degrees_of_freedom(&self) -> Option<u32> {
        let dof = self.count.checked_sub(self.unknowns)?;
        u32::try_from(dof).ok().filter(|dof| *dof >= 1)
    }

    fn fit(&self) -> Option<Fit> {
        let m = self.unknowns;
        let mut normal = [[0.0; MAX_UNKNOWNS]; MAX_UNKNOWNS];
        for row in self.rows() {
            for (i, line) in normal.iter_mut().enumerate().take(m) {
                for (j, cell) in line.iter_mut().enumerate().take(m) {
                    *cell += row.weight * at(&row.geometry, i) * at(&row.geometry, j);
                }
            }
        }
        let inverse = invert(normal, m)?;
        let mut fit = Fit {
            slopes: [[0.0; MAX_RAIM_SATELLITES]; 3],
            parity: [0.0; MAX_RAIM_SATELLITES],
            inverse,
        };
        for (k, row) in self.rows().iter().enumerate() {
            // Column k of S: N⁻¹ g_k w_k.
            let mut column = [0.0; MAX_UNKNOWNS];
            for (i, value) in column.iter_mut().enumerate().take(m) {
                *value = (0..m)
                    .map(|j| at2(&fit.inverse, i, j) * at(&row.geometry, j))
                    .sum::<f64>()
                    * row.weight;
            }
            for (axis, line) in fit.slopes.iter_mut().enumerate() {
                if let Some(cell) = line.get_mut(k) {
                    *cell = at(&column, axis);
                }
            }
            let hat = (0..m)
                .map(|i| at(&row.geometry, i) * at(&column, i))
                .sum::<f64>();
            if let Some(cell) = fit.parity.get_mut(k) {
                *cell = 1.0 - hat;
            }
        }
        Some(fit)
    }

    /// `√(Σ w r²)` of the residuals projected onto this geometry's parity
    /// space: `r − G S r`.
    fn statistic(&self, fit: &Fit) -> f64 {
        let m = self.unknowns;
        // x = S r, extended to the clocks through N⁻¹ Gᵀ W r.
        let mut gtwr = [0.0; MAX_UNKNOWNS];
        for row in self.rows() {
            for (i, value) in gtwr.iter_mut().enumerate().take(m) {
                *value += at(&row.geometry, i) * row.weight * row.residual;
            }
        }
        let x: [f64; MAX_UNKNOWNS] =
            core::array::from_fn(|i| (0..m).map(|j| at2(&fit.inverse, i, j) * at(&gtwr, j)).sum());
        let sum = self
            .rows()
            .iter()
            .map(|row| {
                let projected = row.residual
                    - (0..m)
                        .map(|i| at(&row.geometry, i) * at(&x, i))
                        .sum::<f64>();
                row.weight * projected * projected
            })
            .sum::<f64>();
        math::sqrt(sum)
    }

    /// Protection levels: the largest horizontal and vertical slopes times
    /// the bias multiplier `√λ`.
    fn protection(&self, fit: &Fit, multiplier: f64) -> Protection {
        let mut horizontal: f64 = 0.0;
        let mut vertical: f64 = 0.0;
        let [east, north, up] = &fit.slopes;
        let columns = self
            .rows()
            .iter()
            .zip(&fit.parity)
            .zip(east.iter().zip(north).zip(up));
        for ((row, &parity), ((&east, &north), &up)) in columns {
            if parity <= 1e-12 {
                continue;
            }
            let scale = row.sigma / math::sqrt(parity);
            horizontal = horizontal.max(math::hypot(east, north) * scale);
            vertical = vertical.max(math::abs(up) * scale);
        }
        Protection {
            horizontal: Distance::from_metres(horizontal * multiplier).unwrap_or(Distance::ZERO),
            vertical: Distance::from_metres(vertical * multiplier).unwrap_or(Distance::ZERO),
            satellites: u8::try_from(self.count).unwrap_or(u8::MAX),
        }
    }
}

fn at(values: &[f64; MAX_UNKNOWNS], index: usize) -> f64 {
    values.get(index).copied().unwrap_or(0.0)
}

fn at2(values: &[[f64; MAX_UNKNOWNS]; MAX_UNKNOWNS], row: usize, column: usize) -> f64 {
    values
        .get(row)
        .and_then(|line| line.get(column))
        .copied()
        .unwrap_or(0.0)
}

/// The solution row of a satellite; registers its constellation's clock.
fn row(
    view: &SatelliteView,
    error: RangeError,
    residuals: bool,
    clocks: &mut [Option<Constellation>; MAX_UNKNOWNS - 3],
) -> Option<Row> {
    let elevation = view.elevation()?.degrees();
    let azimuth = view.azimuth()?.radians();
    let residual = match (residuals, view.residual()) {
        (true, Some(residual)) => residual.metres(),
        (true, None) => return None,
        (false, _) => 0.0,
    };
    let constellation = view.id().constellation();
    let clock = if let Some(index) = clocks.iter().position(|slot| *slot == Some(constellation)) {
        index
    } else {
        let index = clocks.iter().position(Option::is_none)?;
        if let Some(slot) = clocks.get_mut(index) {
            *slot = Some(constellation);
        }
        index
    };
    let elevation_radians = math::to_radians(elevation);
    let (sin_el, cos_el) = (math::sin(elevation_radians), math::cos(elevation_radians));
    let mut geometry = [0.0; MAX_UNKNOWNS];
    for (cell, value) in geometry.iter_mut().zip([
        -cos_el * math::sin(azimuth),
        -cos_el * math::cos(azimuth),
        -sin_el,
    ]) {
        *cell = value;
    }
    if let Some(cell) = geometry.get_mut(clock.saturating_add(3)) {
        *cell = 1.0;
    }
    let sigma = error.sigma(elevation);
    Some(Row {
        id: Some(view.id()),
        geometry,
        weight: 1.0 / (sigma * sigma),
        sigma,
        residual,
    })
}

/// Gauss–Jordan inverse of the leading `m × m` block, partial pivoting;
/// `None` if singular.
fn invert(
    mut a: [[f64; MAX_UNKNOWNS]; MAX_UNKNOWNS],
    m: usize,
) -> Option<[[f64; MAX_UNKNOWNS]; MAX_UNKNOWNS]> {
    let mut inverse = [[0.0; MAX_UNKNOWNS]; MAX_UNKNOWNS];
    for (i, line) in inverse.iter_mut().enumerate().take(m) {
        if let Some(cell) = line.get_mut(i) {
            *cell = 1.0;
        }
    }
    let scale = (0..m).map(|i| math::abs(at2(&a, i, i))).fold(0.0, f64::max);
    for column in 0..m {
        let pivot = (column..m).max_by(|&x, &y| {
            math::abs(at2(&a, x, column)).total_cmp(&math::abs(at2(&a, y, column)))
        })?;
        if math::abs(at2(&a, pivot, column)) <= 1e-12 * scale.max(f64::MIN_POSITIVE) {
            return None;
        }
        a.swap(column, pivot);
        inverse.swap(column, pivot);
        let divisor = at2(&a, column, column);
        for value in a
            .get_mut(column)?
            .iter_mut()
            .chain(inverse.get_mut(column)?.iter_mut())
        {
            *value /= divisor;
        }
        let pivot_row = *a.get(column)?;
        let pivot_inverse = *inverse.get(column)?;
        for row in 0..m {
            if row == column {
                continue;
            }
            let factor = at2(&a, row, column);
            if let (Some(target), Some(target_inverse)) = (a.get_mut(row), inverse.get_mut(row)) {
                for (cell, pivot) in target.iter_mut().zip(pivot_row) {
                    *cell -= factor * pivot;
                }
                for (cell, pivot) in target_inverse.iter_mut().zip(pivot_inverse) {
                    *cell -= factor * pivot;
                }
            }
        }
    }
    Some(inverse)
}

/// Threshold and bias multiplier for `dof` degrees of freedom.
fn bounds(config: &RaimConfig, dof: u32) -> Result<(f64, f64)> {
    let threshold_squared = chi_square_quantile(1.0 - config.probability_false_alarm, dof)?;
    let lambda = bias_noncentrality(threshold_squared, dof, config.probability_missed_detection)?;
    Ok((math::sqrt(threshold_squared), math::sqrt(lambda)))
}

fn too_few(solution: &Solution, extra: usize) -> IntegrityReport {
    IntegrityReport::Unavailable(Unavailability::TooFewSatellites {
        usable: u8::try_from(solution.count).unwrap_or(u8::MAX),
        needed: u8::try_from(solution.unknowns.saturating_add(extra)).unwrap_or(u8::MAX),
    })
}

/// Protection levels from geometry alone, for a satellite set without
/// residuals: whether integrity would be available, before the fact.
///
/// # Errors
///
/// [`KernelError::NotConverged`](kinavis_kernel::KernelError::NotConverged) if a chi-square bound does not settle; not
/// reached for at most 64 satellites.
pub fn predict<const N: usize>(
    set: &SatelliteSet<N>,
    config: &RaimConfig,
) -> Result<IntegrityReport> {
    let solution = Solution::from_set(set, config.range_error, false);
    let Some(dof) = solution.degrees_of_freedom() else {
        return Ok(too_few(&solution, 1));
    };
    let Some(fit) = solution.fit() else {
        return Ok(IntegrityReport::Unavailable(Unavailability::Geometry));
    };
    let (threshold, multiplier) = bounds(config, dof)?;
    Ok(IntegrityReport::Available {
        protection: solution.protection(&fit, multiplier),
        statistic: 0.0,
        threshold,
    })
}

/// Fault detection and exclusion on the used satellites of a set, from
/// their sky positions and range residuals.
///
/// A satellite without residual or sky position is left out. When the test
/// fails, each satellite in turn is excluded; the exclusion whose solution
/// passes with the most margin is reported, with its protection levels.
///
/// # Errors
///
/// [`KernelError::NotConverged`](kinavis_kernel::KernelError::NotConverged) if a chi-square bound does not settle; not
/// reached for at most 64 satellites.
pub fn assess<const N: usize>(
    set: &SatelliteSet<N>,
    config: &RaimConfig,
) -> Result<IntegrityReport> {
    let solution = Solution::from_set(set, config.range_error, true);
    let Some(dof) = solution.degrees_of_freedom() else {
        return Ok(too_few(&solution, 1));
    };
    let Some(fit) = solution.fit() else {
        return Ok(IntegrityReport::Unavailable(Unavailability::Geometry));
    };
    let (threshold, multiplier) = bounds(config, dof)?;
    let statistic = solution.statistic(&fit);
    if statistic <= threshold {
        return Ok(IntegrityReport::Available {
            protection: solution.protection(&fit, multiplier),
            statistic,
            threshold,
        });
    }
    let mut best: Option<(f64, SatelliteId, Protection)> = None;
    if dof >= 2 {
        for index in 0..solution.count {
            let subset = solution.without(index);
            let (Some(sub_dof), Some(sub_fit)) = (subset.degrees_of_freedom(), subset.fit()) else {
                continue;
            };
            let (sub_threshold, sub_multiplier) = bounds(config, sub_dof)?;
            let margin = subset.statistic(&sub_fit) / sub_threshold;
            if margin <= 1.0 && best.is_none_or(|(held, _, _)| margin < held) {
                let id = solution.rows().get(index).and_then(|row| row.id);
                if let Some(id) = id {
                    best = Some((margin, id, subset.protection(&sub_fit, sub_multiplier)));
                }
            }
        }
    }
    Ok(IntegrityReport::FaultDetected {
        excluded: best.map(|(_, id, _)| id),
        after_exclusion: best.map(|(_, _, protection)| protection),
        statistic,
        threshold,
    })
}

/// Protection level of a receiver that reports only its error ellipse:
/// `K σ_major`, `K = √(−2 ln risk)`, so that a two-dimensional Gaussian
/// error leaves the circle with at most `risk`. An estimate, not RAIM: it
/// does not detect a faulty satellite.
///
/// # Errors
///
/// [`KernelError::OutOfRange`](kinavis_kernel::KernelError::OutOfRange) unless `0 < risk < 1`.
pub fn protection_from_ellipse(semi_major: Distance, risk: f64) -> Result<Distance> {
    ensure_range(
        "integrity risk",
        risk,
        f64::MIN_POSITIVE,
        1.0 - f64::EPSILON,
    )?;
    Distance::from_metres(math::sqrt(-2.0 * math::ln(risk)) * semi_major.metres())
}

/// Bound the kernel's integrity state on alert limits: `Exceeded` when a
/// protection level passes its limit or integrity is not available.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AlertLimits {
    /// Horizontal alert limit.
    pub horizontal: Distance,
    /// Vertical alert limit.
    pub vertical: Distance,
}

impl AlertLimits {
    /// Whether the report supports navigation within these limits.
    #[must_use]
    pub fn supports(&self, report: &IntegrityReport) -> bool {
        let (IntegrityReport::Available { protection, .. }
        | IntegrityReport::FaultDetected {
            after_exclusion: Some(protection),
            ..
        }) = report
        else {
            return false;
        };
        protection.horizontal.metres() <= self.horizontal.metres()
            && protection.vertical.metres() <= self.vertical.metres()
    }
}

/// The kernel's navigation integrity for a report and alert limits.
#[must_use]
pub fn navigation_integrity(
    report: &IntegrityReport,
    limits: &AlertLimits,
) -> kinavis_kernel::NavigationIntegrity {
    if limits.supports(report) {
        kinavis_kernel::NavigationIntegrity::Nominal
    } else {
        kinavis_kernel::NavigationIntegrity::Exceeded
    }
}
