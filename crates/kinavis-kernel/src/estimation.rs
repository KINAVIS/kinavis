//! Estimator ports: process model and observation.
//!
//! An estimator fuses observations into a [`NavigationState`] without knowing
//! their source. A filter with `update_gnss`, `update_gyro`, `update_log`
//! methods must change for every new sensor and cannot be tested without faking
//! one; instead each observation describes itself — measurement, prediction,
//! Jacobian, noise — through the [`Observation`] port. The [`ProcessModel`]
//! port describes motion between observations.
//!
//! Neither port exposes the state layout: an observation names
//! [`StateComponent::Heading`] and the crate maps names to columns. Standard
//! observations and process models live in `kinavis`; an adapter with an
//! unusual sensor implements [`Observation`] without touching the estimator.

use core::time::Duration;

use crate::error::{ensure_finite, KernelError, Result};
use crate::event::SensorId;
use crate::inline::Inline;
use crate::matrix::Matrix;
use crate::state::{NavigationState, StateComponent, STATE_DIM};
use crate::time::{Instant, Utc};

/// Maximum observation dimension.
///
/// Position 2, velocity 2, heading 1, fix with velocity 4. Larger observations
/// should be split.
pub const MAX_OBSERVATION_DIM: usize = 4;

/// Observation elements, measured or predicted.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ObservationVector {
    elements: Inline<f64, MAX_OBSERVATION_DIM>,
}

impl ObservationVector {
    /// Empty vector; rejected by estimators.
    pub const EMPTY: Self = Self {
        elements: Inline::new(0.0),
    };

    /// Vector from elements.
    ///
    /// # Errors
    ///
    /// [`KernelError::CapacityExceeded`] beyond [`MAX_OBSERVATION_DIM`];
    /// [`KernelError::NotFinite`] for a non-finite element.
    pub fn new(elements: &[f64]) -> Result<Self> {
        let mut inline = Inline::new(0.0);
        for &element in elements {
            ensure_finite("observation element", element)?;
            inline
                .push(element)
                .map_err(|full| KernelError::CapacityExceeded {
                    context: "an observation",
                    needed: elements.len(),
                    capacity: full.capacity,
                })?;
        }
        Ok(Self { elements: inline })
    }

    /// Elements.
    #[must_use]
    pub fn as_slice(&self) -> &[f64] {
        self.elements.as_slice()
    }

    /// Length.
    #[must_use]
    pub const fn len(&self) -> usize {
        self.elements.len()
    }

    /// Whether empty (degenerate observation).
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.elements.is_empty()
    }

    /// Element at `index`; `None` past the end.
    #[must_use]
    pub fn get(&self, index: usize) -> Option<f64> {
        self.elements.get(index).copied()
    }

    /// Element-wise `self − other`; `None` for different lengths.
    #[must_use]
    pub fn minus(&self, other: &Self) -> Option<Self> {
        if self.len() != other.len() {
            return None;
        }
        let mut elements = Inline::new(0.0);
        for (a, b) in self.as_slice().iter().zip(other.as_slice()) {
            elements.push(a - b).ok()?;
        }
        Some(Self { elements })
    }
}

/// One Jacobian row: derivatives of one element with respect to each state
/// component.
///
/// Built by naming components; unnamed entries are zero.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JacobianRow {
    entries: [f64; STATE_DIM],
}

impl JacobianRow {
    /// Zero row.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            entries: [0.0; STATE_DIM],
        }
    }

    /// Sets the derivative with respect to one component.
    #[must_use]
    pub fn with(mut self, component: StateComponent, derivative: f64) -> Self {
        if let Some(slot) = self.entries.get_mut(component.index()) {
            *slot = derivative;
        }
        self
    }

    /// Derivative with respect to a component.
    #[must_use]
    pub fn derivative(&self, component: StateComponent) -> f64 {
        self.entries.get(component.index()).copied().unwrap_or(0.0)
    }

    /// Row in state-vector order.
    ///
    /// Internal to the crate family: hidden, not covered by the stability
    /// guarantee. See [hidden items](crate#hidden-items).
    #[doc(hidden)]
    #[must_use]
    pub const fn entries(&self) -> &[f64; STATE_DIM] {
        &self.entries
    }
}

impl Default for JacobianRow {
    fn default() -> Self {
        Self::new()
    }
}

/// Observation Jacobian: one row per element, in [`ObservationVector`] order.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ObservationJacobian {
    rows: Inline<JacobianRow, MAX_OBSERVATION_DIM>,
}

impl ObservationJacobian {
    /// Empty Jacobian.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            rows: Inline::new(JacobianRow::new()),
        }
    }

    /// Appends a row.
    ///
    /// # Errors
    ///
    /// [`KernelError::CapacityExceeded`] beyond [`MAX_OBSERVATION_DIM`] rows;
    /// [`KernelError::NotFinite`] for a non-finite derivative.
    pub fn with_row(mut self, row: JacobianRow) -> Result<Self> {
        for &entry in row.entries() {
            ensure_finite("jacobian entry", entry)?;
        }
        let needed = self.rows.len().saturating_add(1);
        self.rows
            .push(row)
            .map_err(|full| KernelError::CapacityExceeded {
                context: "an observation jacobian",
                needed,
                capacity: full.capacity,
            })?;
        Ok(self)
    }

    /// Row count.
    #[must_use]
    pub const fn len(&self) -> usize {
        self.rows.len()
    }

    /// Whether empty.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.rows.is_empty()
    }

    /// Rows.
    #[must_use]
    pub fn rows(&self) -> &[JacobianRow] {
        self.rows.as_slice()
    }
}

impl Default for ObservationJacobian {
    fn default() -> Self {
        Self::new()
    }
}

/// Observation noise covariance `R`.
///
/// Usually diagonal; a full matrix is allowed for correlated elements.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ObservationNoise {
    entries: [[f64; MAX_OBSERVATION_DIM]; MAX_OBSERVATION_DIM],
    len: usize,
}

impl ObservationNoise {
    /// Unit variance, one element.
    pub const UNIT: Self = {
        let mut entries = [[0.0; MAX_OBSERVATION_DIM]; MAX_OBSERVATION_DIM];
        entries[0][0] = 1.0;
        Self { entries, len: 1 }
    };

    /// Independent elements with the given variances.
    ///
    /// # Errors
    ///
    /// [`KernelError::CapacityExceeded`] beyond [`MAX_OBSERVATION_DIM`];
    /// [`KernelError::OutOfRange`] for a non-positive variance (a noiseless
    /// observation makes the update singular).
    pub fn diagonal(variances: &[f64]) -> Result<Self> {
        if variances.len() > MAX_OBSERVATION_DIM {
            return Err(KernelError::CapacityExceeded {
                context: "an observation noise",
                needed: variances.len(),
                capacity: MAX_OBSERVATION_DIM,
            });
        }
        let mut entries = [[0.0; MAX_OBSERVATION_DIM]; MAX_OBSERVATION_DIM];
        for (index, &variance) in variances.iter().enumerate() {
            ensure_finite("observation variance", variance)?;
            if variance <= 0.0 {
                return Err(KernelError::OutOfRange {
                    parameter: "observation variance",
                    value: variance,
                    min: f64::MIN_POSITIVE,
                    max: f64::MAX,
                });
            }
            if let Some(slot) = entries.get_mut(index).and_then(|row| row.get_mut(index)) {
                *slot = variance;
            }
        }
        Ok(Self {
            entries,
            len: variances.len(),
        })
    }

    /// Independent elements with the given standard deviations.
    ///
    /// # Errors
    ///
    /// As [`ObservationNoise::diagonal`].
    pub fn sigmas(sigmas: &[f64]) -> Result<Self> {
        let mut variances = Inline::<f64, MAX_OBSERVATION_DIM>::new(0.0);
        for &sigma in sigmas {
            variances
                .push(sigma * sigma)
                .map_err(|full| KernelError::CapacityExceeded {
                    context: "an observation noise",
                    needed: sigmas.len(),
                    capacity: full.capacity,
                })?;
        }
        Self::diagonal(variances.as_slice())
    }

    /// Full covariance for correlated elements.
    ///
    /// # Errors
    ///
    /// [`KernelError::CapacityExceeded`] beyond [`MAX_OBSERVATION_DIM`] rows;
    /// [`KernelError::NotCovariance`] unless symmetric positive definite.
    pub fn full(rows: &[&[f64]]) -> Result<Self> {
        let len = rows.len();
        if len > MAX_OBSERVATION_DIM {
            return Err(KernelError::CapacityExceeded {
                context: "an observation noise",
                needed: len,
                capacity: MAX_OBSERVATION_DIM,
            });
        }
        let mut entries = [[0.0; MAX_OBSERVATION_DIM]; MAX_OBSERVATION_DIM];
        for (i, row) in rows.iter().enumerate() {
            if row.len() != len {
                return Err(KernelError::NotCovariance {
                    context: "an observation noise with rows of unequal length",
                });
            }
            for (j, &value) in row.iter().enumerate() {
                ensure_finite("observation covariance", value)?;
                if let Some(slot) = entries.get_mut(i).and_then(|row| row.get_mut(j)) {
                    *slot = value;
                }
            }
        }
        let noise = Self { entries, len };
        // Positive definite on the used block, identity-padded so the unused
        // block does not affect the check.
        let padded = Matrix::<MAX_OBSERVATION_DIM, MAX_OBSERVATION_DIM>::from_fn(|i, j| {
            if i < len && j < len {
                noise.get(i, j).unwrap_or(0.0)
            } else if i == j {
                1.0
            } else {
                0.0
            }
        });
        if !padded.is_covariance() || (0..len).any(|i| noise.get(i, i).unwrap_or(0.0) <= 0.0) {
            return Err(KernelError::NotCovariance {
                context: "an observation noise that is not positive definite",
            });
        }
        Ok(noise)
    }

    /// Dimension.
    #[must_use]
    pub const fn len(&self) -> usize {
        self.len
    }

    /// Whether empty.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Covariance entry; `None` outside.
    #[must_use]
    pub fn get(&self, row: usize, column: usize) -> Option<f64> {
        if row >= self.len || column >= self.len {
            return None;
        }
        self.entries.get(row)?.get(column).copied()
    }
}

/// Innovation gating policy.
///
/// The normalised innovation squared is χ²-distributed with as many degrees of
/// freedom as the observation has elements when the model holds. A value far in
/// the tail is more likely a fault than a surprise.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GatingPolicy {
    threshold: Option<f64>,
}

impl GatingPolicy {
    /// No gate.
    #[must_use]
    pub const fn none() -> Self {
        Self { threshold: None }
    }

    /// Reject when NIS exceeds the threshold.
    ///
    /// 2 dof: 5.99 rejects 5 %, 9.21 rejects 1 %. 1 dof: 3.84 and 6.63.
    #[must_use]
    pub const fn reject_above(threshold: f64) -> Self {
        Self {
            threshold: Some(threshold),
        }
    }

    /// Threshold, if any.
    #[must_use]
    pub const fn threshold(&self) -> Option<f64> {
        self.threshold
    }
}

/// Observation from the estimator's view: `z`, `h(x)`, `H`, `R`, no source
/// details.
///
/// Vectors and Jacobian must agree in length; the estimator rejects the
/// observation otherwise.
pub trait Observation {
    /// Source identifier, for reporting and per-source health: the estimator
    /// tracks accept/reject runs per [`SensorId`], so differently named
    /// receivers are judged separately.
    fn sensor(&self) -> SensorId;

    /// Time of the measurement.
    fn taken_at(&self) -> Instant<Utc>;

    /// Measurement `z`.
    fn measured(&self) -> ObservationVector;

    /// Predicted measurement `h(x)`.
    ///
    /// # Errors
    ///
    /// Any error preventing the prediction from this state.
    fn predict(&self, state: &NavigationState) -> Result<ObservationVector>;

    /// Jacobian `H = ∂h/∂x` at `x`.
    ///
    /// # Errors
    ///
    /// As [`Observation::predict`].
    fn jacobian(&self, state: &NavigationState) -> Result<ObservationJacobian>;

    /// Measurement noise `R`.
    fn noise(&self) -> ObservationNoise;

    /// Gating policy.
    fn gate(&self) -> GatingPolicy {
        GatingPolicy::none()
    }

    /// Innovation `z − h(x)`.
    ///
    /// Plain difference by default. Angle observations override it to wrap into
    /// `[−π, π)`: 359° measured vs 1° predicted is −2°, not 358°.
    ///
    /// # Errors
    ///
    /// [`KernelError::BufferTooSmall`] if measured and predicted vectors differ
    /// in length.
    fn innovation(&self, predicted: &ObservationVector) -> Result<ObservationVector> {
        let measured = self.measured();
        measured
            .minus(predicted)
            .ok_or(KernelError::BufferTooSmall {
                needed: measured.len(),
                found: predicted.len(),
            })
    }
}

/// Process Jacobian `F = ∂f/∂x` over a step.
///
/// Built from the identity by setting non-identity entries by name.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StateJacobian {
    matrix: Matrix<STATE_DIM, STATE_DIM>,
}

impl StateJacobian {
    /// Identity.
    #[must_use]
    pub fn identity() -> Self {
        Self {
            matrix: Matrix::identity(),
        }
    }

    /// Sets `∂(row)/∂(column)`.
    #[must_use]
    pub fn with(mut self, row: StateComponent, column: StateComponent, derivative: f64) -> Self {
        self.matrix.set(row.index(), column.index(), derivative);
        self
    }

    /// `∂(row)/∂(column)`.
    #[must_use]
    pub fn derivative(&self, row: StateComponent, column: StateComponent) -> f64 {
        self.matrix.get(row.index(), column.index()).unwrap_or(0.0)
    }

    /// Matrix form.
    ///
    /// Internal to the crate family: hidden, not covered by the stability
    /// guarantee. See [hidden items](crate#hidden-items).
    #[doc(hidden)]
    #[must_use]
    pub const fn matrix(&self) -> &Matrix<STATE_DIM, STATE_DIM> {
        &self.matrix
    }
}

/// Process noise `Q`: unmodelled state change over a step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ProcessNoise {
    matrix: Matrix<STATE_DIM, STATE_DIM>,
}

impl ProcessNoise {
    /// Zero noise.
    #[must_use]
    pub const fn zero() -> Self {
        Self {
            matrix: Matrix::ZERO,
        }
    }

    /// Sets one component's variance.
    #[must_use]
    pub fn with_variance(mut self, component: StateComponent, variance: f64) -> Self {
        self.matrix
            .set(component.index(), component.index(), variance);
        self
    }

    /// Sets the covariance of two components, symmetrically.
    #[must_use]
    pub fn with_covariance(
        mut self,
        a: StateComponent,
        b: StateComponent,
        covariance: f64,
    ) -> Self {
        self.matrix.set(a.index(), b.index(), covariance);
        self.matrix.set(b.index(), a.index(), covariance);
        self
    }

    /// Variance of one component.
    #[must_use]
    pub fn variance(&self, component: StateComponent) -> f64 {
        self.matrix
            .get(component.index(), component.index())
            .unwrap_or(0.0)
    }

    /// Matrix form.
    ///
    /// Internal to the crate family: hidden, not covered by the stability
    /// guarantee. See [hidden items](crate#hidden-items).
    #[doc(hidden)]
    #[must_use]
    pub const fn matrix(&self) -> &Matrix<STATE_DIM, STATE_DIM> {
        &self.matrix
    }
}

/// State propagation between observations; injected into the estimator.
pub trait ProcessModel {
    /// State after a step, `f(x)`, with the covariance unchanged; the estimator
    /// propagates the covariance from `F` and `Q`.
    ///
    /// # Errors
    ///
    /// Any error the model raises: negative step, unpropagatable state.
    fn propagate(&self, state: &NavigationState, over: Duration) -> Result<NavigationState>;

    /// Process Jacobian `F`.
    ///
    /// # Errors
    ///
    /// As [`ProcessModel::propagate`].
    fn jacobian(&self, state: &NavigationState, over: Duration) -> Result<StateJacobian>;

    /// Process noise `Q` at the start state.
    ///
    /// State-dependent because noise may enter through the state (heading noise
    /// moving position across track at the speed made good). A self-consistent
    /// model gives the same `Q` for one step as for its two halves;
    /// late-observation handling relies on it.
    fn noise(&self, state: &NavigationState, over: Duration) -> ProcessNoise;
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;

    #[test]
    fn observation_vectors_are_bounded_and_finite() {
        let vector = ObservationVector::new(&[1.0, 2.0]).unwrap();
        assert_eq!(vector.as_slice(), &[1.0, 2.0]);
        assert_eq!(vector.get(1), Some(2.0));
        assert_eq!(vector.get(2), None);
        assert!(ObservationVector::new(&[1.0; 5]).is_err());
        assert!(ObservationVector::new(&[f64::NAN]).is_err());
        let difference = vector
            .minus(&ObservationVector::new(&[0.5, 0.5]).unwrap())
            .unwrap();
        assert_eq!(difference.as_slice(), &[0.5, 1.5]);
        assert!(vector
            .minus(&ObservationVector::new(&[1.0]).unwrap())
            .is_none());
    }

    #[test]
    fn jacobians_are_built_by_name() {
        let row = JacobianRow::new().with(StateComponent::Heading, 2.0);
        assert_eq!(row.derivative(StateComponent::Heading), 2.0);
        assert_eq!(row.derivative(StateComponent::North), 0.0);
        let jacobian = ObservationJacobian::new().with_row(row).unwrap();
        assert_eq!(jacobian.len(), 1);
        assert_eq!(jacobian.rows().first(), Some(&row));
        let mut full = jacobian;
        for _ in 1..MAX_OBSERVATION_DIM {
            full = full.with_row(row).unwrap();
        }
        assert!(full.with_row(row).is_err());
        assert!(ObservationJacobian::new()
            .with_row(JacobianRow::new().with(StateComponent::East, f64::INFINITY))
            .is_err());

        let state_jacobian = StateJacobian::identity().with(
            StateComponent::North,
            StateComponent::SpeedThroughWater,
            0.5,
        );
        assert_eq!(
            state_jacobian.derivative(StateComponent::North, StateComponent::SpeedThroughWater),
            0.5
        );
        assert_eq!(
            state_jacobian.derivative(StateComponent::North, StateComponent::North),
            1.0
        );
    }

    #[test]
    fn noise_must_be_positive_definite() {
        let diagonal = ObservationNoise::sigmas(&[2.0, 3.0]).unwrap();
        assert_eq!(diagonal.get(0, 0), Some(4.0));
        assert_eq!(diagonal.get(1, 1), Some(9.0));
        assert_eq!(diagonal.get(0, 1), Some(0.0));
        assert_eq!(diagonal.get(2, 2), None);
        assert!(ObservationNoise::diagonal(&[0.0]).is_err());
        assert!(ObservationNoise::diagonal(&[-1.0]).is_err());
        assert!(ObservationNoise::diagonal(&[1.0; 5]).is_err());
        let full = ObservationNoise::full(&[&[2.0, 0.5], &[0.5, 2.0]]).unwrap();
        assert_eq!(full.get(1, 0), Some(0.5));
        assert!(ObservationNoise::full(&[&[1.0, 2.0], &[2.0, 1.0]]).is_err());
        assert!(ObservationNoise::full(&[&[1.0, 0.0], &[0.0]]).is_err());

        let process = ProcessNoise::zero()
            .with_variance(StateComponent::Heading, 0.01)
            .with_covariance(
                StateComponent::CurrentNorth,
                StateComponent::CurrentEast,
                0.1,
            );
        assert_eq!(process.variance(StateComponent::Heading), 0.01);
        assert_eq!(process.variance(StateComponent::North), 0.0);
        assert_eq!(
            process.matrix().get(
                StateComponent::CurrentEast.index(),
                StateComponent::CurrentNorth.index()
            ),
            Some(0.1)
        );
    }

    #[test]
    fn gating_is_optional() {
        assert_eq!(GatingPolicy::none().threshold(), None);
        assert_eq!(GatingPolicy::reject_above(5.99).threshold(), Some(5.99));
    }

    /// One-element observation for exercising the trait defaults.
    struct Constant(f64);

    impl Observation for Constant {
        fn sensor(&self) -> SensorId {
            SensorId::named("constant")
        }
        fn taken_at(&self) -> Instant<Utc> {
            Instant::UNIX_EPOCH
        }
        fn measured(&self) -> ObservationVector {
            ObservationVector::new(&[self.0]).unwrap()
        }
        fn predict(&self, _: &NavigationState) -> Result<ObservationVector> {
            ObservationVector::new(&[0.0])
        }
        fn jacobian(&self, _: &NavigationState) -> Result<ObservationJacobian> {
            ObservationJacobian::new().with_row(JacobianRow::new())
        }
        fn noise(&self) -> ObservationNoise {
            ObservationNoise::sigmas(&[1.0]).unwrap()
        }
    }

    #[test]
    fn the_default_innovation_is_the_difference() {
        let observation = Constant(3.0);
        let predicted = ObservationVector::new(&[1.0]).unwrap();
        assert_eq!(
            observation.innovation(&predicted).unwrap().as_slice(),
            &[2.0]
        );
        assert!(observation
            .innovation(&ObservationVector::new(&[1.0, 1.0]).unwrap())
            .is_err());
        assert_eq!(observation.gate(), GatingPolicy::none());
    }
}
