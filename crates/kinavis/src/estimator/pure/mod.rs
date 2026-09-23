//! Filter steps as pure functions.
//!
//! [`predict`] and [`update`] are the Kalman filter; [`update_late`] handles an
//! observation older than the belief, via the intermediate history.

mod late;

use core::time::Duration;

use crate::error::{KernelError, NavigationError, Result};
use crate::estimation::{
    Observation, ObservationJacobian, ObservationNoise, ObservationVector, ProcessModel,
};
use crate::event::SensorId;
use crate::math;
use crate::matrix::{Matrix, Vector};
use crate::state::{NavigationState, StateComponent, STATE_DIM};

pub use late::update_late;

/// Update result: whether the observation was applied, and its normalised
/// innovation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct UpdateReport {
    sensor: SensorId,
    accepted: bool,
    normalised_innovation_squared: f64,
    degrees_of_freedom: usize,
    fixes_position: bool,
}

impl UpdateReport {
    /// Source.
    #[must_use]
    pub const fn sensor(&self) -> SensorId {
        self.sensor
    }

    /// Whether the state was corrected.
    #[must_use]
    pub const fn accepted(&self) -> bool {
        self.accepted
    }

    /// Normalised innovation squared, `νᵀ S⁻¹ ν`.
    ///
    /// χ² with [`UpdateReport::degrees_of_freedom`] when the model holds: mean
    /// equals the dof, rarely above three times it.
    #[must_use]
    pub const fn normalised_innovation_squared(&self) -> f64 {
        self.normalised_innovation_squared
    }

    /// Observation dimension.
    #[must_use]
    pub const fn degrees_of_freedom(&self) -> usize {
        self.degrees_of_freedom
    }

    /// Whether the observation constrains position directly (a fix), as opposed
    /// to heading, speed or velocity.
    ///
    /// Determined from the Jacobian: any row with a northing or easting
    /// derivative.
    #[must_use]
    pub const fn fixes_position(&self) -> bool {
        self.fixes_position
    }
}

/// Whether a Jacobian bears on position.
fn fixes_position(jacobian: &ObservationJacobian) -> bool {
    jacobian.rows().iter().any(|row| {
        math::abs(row.derivative(StateComponent::North)) > 0.0
            || math::abs(row.derivative(StateComponent::East)) > 0.0
    })
}

/// Prediction: `x ← f(x)`, `P ← F P Fᵀ + Q`.
///
/// # Errors
///
/// Model errors; state invariant violations by the propagated covariance.
pub fn predict(
    state: &NavigationState,
    model: &impl ProcessModel,
    over: Duration,
) -> Result<NavigationState> {
    let moved = model.propagate(state, over)?;
    let transition = *model.jacobian(state, over)?.matrix();
    let noise = *model.noise(state, over).matrix();
    let covariance =
        (transition * *state.covariance() * transition.transpose() + noise).symmetrised();
    Ok(NavigationState::from_parts(
        moved.valid_at(),
        *moved.frame(),
        *moved.vector(),
        covariance,
    )?)
}

/// Corrected state and update report.
///
/// Joseph-form Kalman update, keeping the covariance valid under rounding. A
/// gate rejection returns the state unchanged with `accepted` false.
///
/// # Errors
///
/// Observation errors; [`KernelError::BufferTooSmall`] if vector, Jacobian and
/// noise lengths disagree; [`KernelError::SingularSystem`] if the innovation
/// covariance cannot be factored (prevented by positive definite noise).
pub fn update(
    state: &NavigationState,
    observation: &dyn Observation,
) -> Result<(NavigationState, UpdateReport)> {
    let predicted = observation.predict(state)?;
    let jacobian = observation.jacobian(state)?;
    let noise = observation.noise();
    let innovation = observation.innovation(&predicted)?;
    let dimension = innovation.len();
    if jacobian.len() != dimension || noise.len() != dimension || dimension == 0 {
        return Err(NavigationError::Kernel(KernelError::BufferTooSmall {
            needed: dimension,
            found: jacobian.len().min(noise.len()),
        }));
    }
    let step = Step {
        state,
        jacobian: &jacobian,
        noise: &noise,
        innovation: &innovation,
        gate: observation.gate().threshold(),
        sensor: observation.sensor(),
    };
    match dimension {
        1 => step.run::<1>(),
        2 => step.run::<2>(),
        3 => step.run::<3>(),
        4 => step.run::<4>(),
        _ => Err(NavigationError::Kernel(KernelError::CapacityExceeded {
            context: "an observation",
            needed: dimension,
            capacity: 4,
        })),
    }
}

/// Update with the observation's parts gathered, dimension still generic.
struct Step<'a> {
    state: &'a NavigationState,
    jacobian: &'a ObservationJacobian,
    noise: &'a ObservationNoise,
    innovation: &'a ObservationVector,
    gate: Option<f64>,
    sensor: SensorId,
}

impl Step<'_> {
    fn run<const M: usize>(&self) -> Result<(NavigationState, UpdateReport)> {
        let h = Matrix::<M, STATE_DIM>::from_fn(|row, column| {
            self.jacobian
                .rows()
                .get(row)
                .and_then(|r| r.entries().get(column))
                .copied()
                .unwrap_or(0.0)
        });
        let r = Matrix::<M, M>::from_fn(|row, column| self.noise.get(row, column).unwrap_or(0.0));
        let nu = Vector::<M>::from_fn(|row, _| self.innovation.get(row).unwrap_or(0.0));
        let p = *self.state.covariance();

        let s = (h * p * h.transpose() + r).symmetrised();
        let factor = s
            .cholesky()
            .ok_or(NavigationError::Kernel(KernelError::SingularSystem {
                context: "innovation covariance",
            }))?;
        let weighted =
            factor
                .solve(&nu)
                .ok_or(NavigationError::Kernel(KernelError::SingularSystem {
                    context: "innovation covariance",
                }))?;
        let normalised_innovation_squared = nu.dot(&weighted);

        let mut report = UpdateReport {
            sensor: self.sensor,
            accepted: true,
            normalised_innovation_squared,
            degrees_of_freedom: M,
            fixes_position: fixes_position(self.jacobian),
        };
        if self
            .gate
            .is_some_and(|threshold| normalised_innovation_squared > threshold)
        {
            report.accepted = false;
            return Ok((*self.state, report));
        }

        // K = P Hᵀ S⁻¹, computed as (S⁻¹ H P)ᵀ since S is symmetric.
        let gain = factor
            .solve(&(h * p))
            .ok_or(NavigationError::Kernel(KernelError::SingularSystem {
                context: "innovation covariance",
            }))?
            .transpose();
        let corrected = *self.state.vector() + gain * nu;
        let shrink = Matrix::<STATE_DIM, STATE_DIM>::identity() - gain * h;
        let covariance =
            (shrink * p * shrink.transpose() + gain * r * gain.transpose()).symmetrised();
        let state = NavigationState::from_parts(
            self.state.valid_at(),
            *self.state.frame(),
            corrected,
            covariance,
        )?;
        Ok((state, report))
    }
}
