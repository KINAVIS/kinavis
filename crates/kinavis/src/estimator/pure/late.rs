//! Out-of-sequence measurement update.
//!
//! A GNSS fix is timestamped at signal reception and delivered tenths of a
//! second later, after further gyro updates. Discarding it loses the only
//! absolute position source; applying it as current puts the vessel where it
//! was. The goal is the correction the fix would have made in time, propagated
//! to the current belief without losing the intervening updates.
//!
//! The belief history gives this exactly for a linear model. Running the RTS
//! smoother backwards from the present to the observation time yields the
//! smoothed belief at that time and the product of smoother gains, i.e. its
//! cross-covariance with the present belief. The innovation against the
//! smoothed belief, weighted by that cross-covariance, corrects the present.
//! This is the exact out-of-sequence measurement (OOSM) solution for the
//! linear-Gaussian case (Bar-Shalom) and the standard first-order form for the
//! EKF. Intermediate observations need not be stored or replayed: their effect
//! is in the beliefs.
//!
//! Approximation: history entries are not updated retroactively, so a second
//! late observation is folded in against a history unaware of the first. The
//! present belief includes it and the correction goes through it, so the error
//! is second-order; systems where lateness is the norm should use a reorder
//! buffer upstream.

use core::time::Duration;

use super::{predict, UpdateReport};
use crate::angle::wrap180;
use crate::error::{KernelError, NavigationError, Result};
use crate::estimation::{
    Observation, ObservationJacobian, ObservationNoise, ObservationVector, ProcessModel,
};
use crate::event::SensorId;
use crate::math;
use crate::matrix::{Matrix, Vector};
use crate::state::{NavigationState, StateComponent, STATE_DIM};

/// Present belief corrected by an observation older than it, through the
/// intermediate beliefs.
///
/// `history` is oldest first and ends at the present belief; its first entry is
/// at or before the observation time, all in one local frame. The result is at
/// the present time. As in [`update`](super::update), a gated innovation leaves
/// the belief unchanged with `accepted` false.
///
/// # Errors
///
/// [`KernelError::Indeterminate`] if the history does not bracket the
/// observation time; [`KernelError::SingularSystem`] if a predicted or
/// innovation covariance cannot be factored; model, observation and
/// state-invariant errors.
pub fn update_late(
    history: &[NavigationState],
    model: &impl ProcessModel,
    observation: &dyn Observation,
) -> Result<(NavigationState, UpdateReport)> {
    let when = observation.taken_at();
    let (Some(first), Some(present)) = (history.first(), history.last()) else {
        return Err(NavigationError::Kernel(KernelError::Missing {
            what: "a late update with no history",
        }));
    };
    if first.valid_at() > when || present.valid_at() <= when {
        return Err(NavigationError::Kernel(KernelError::Missing {
            what: "a late update outside its history",
        }));
    }

    // Belief at the observation time as it was then: the first entry propagated
    // forward, or the entry itself.
    let at_moment = if first.valid_at() == when {
        *first
    } else {
        predict(first, model, when.duration_since(first.valid_at())?)?
    };

    // Backwards from the present to the observation time.
    let mut smoothed = Smoothed {
        vector: *present.vector(),
        covariance: *present.covariance(),
        gain_chain: Matrix::identity(),
    };
    // Entries strictly between the first and the present, newest first; then
    // the belief at the observation time, standing in for the first entry.
    let mut next = *present;
    let between = history.len().saturating_sub(2);
    for node in history.iter().rev().skip(1).take(between) {
        smoothed.step_back(node, &next, model)?;
        next = *node;
    }
    smoothed.step_back(&at_moment, &next, model)?;
    let then =
        NavigationState::from_parts(when, *present.frame(), smoothed.vector, smoothed.covariance)?;

    let predicted = observation.predict(&then)?;
    let jacobian = observation.jacobian(&then)?;
    let noise = observation.noise();
    let innovation = observation.innovation(&predicted)?;
    let dimension = innovation.len();
    if jacobian.len() != dimension || noise.len() != dimension || dimension == 0 {
        return Err(NavigationError::Kernel(KernelError::BufferTooSmall {
            needed: dimension,
            found: jacobian.len().min(noise.len()),
        }));
    }
    let step = LateStep {
        present,
        then: &then,
        gain_chain: &smoothed.gain_chain,
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

/// Smoothed belief during the backward pass, and the product of smoother gains
/// from the present down to it.
struct Smoothed {
    vector: Vector<STATE_DIM>,
    covariance: Matrix<STATE_DIM, STATE_DIM>,
    /// `C_j C_{j+1} ⋯ C_{k−1}`: cross-covariance with the present belief is
    /// this times the present covariance.
    gain_chain: Matrix<STATE_DIM, STATE_DIM>,
}

impl Smoothed {
    /// One Rauch–Tung–Striebel backward step, from the smoothed belief at
    /// `next` to the smoothed belief at `node`.
    ///
    /// `C = P Fᵀ P⁻¹₊`, `x̂ₛ = x̂ + C (x̂ₛ₊ − x̂₊)`, `Pₛ = P + C (Pₛ₊ − P₊)
    /// Cᵀ`; `+` denotes `node` predicted to `next`'s time, `s` smoothed values.
    fn step_back(
        &mut self,
        node: &NavigationState,
        next: &NavigationState,
        model: &impl ProcessModel,
    ) -> Result<()> {
        let over: Duration = next.valid_at().duration_since(node.valid_at())?;
        let predicted = predict(node, model, over)?;
        let transition = *model.jacobian(node, over)?.matrix();
        let factor = predicted
            .covariance()
            .cholesky()
            .ok_or(NavigationError::Kernel(KernelError::SingularSystem {
                context: "predicted covariance",
            }))?;
        // C = P Fᵀ P₊⁻¹, computed as (P₊⁻¹ F P)ᵀ since both P are symmetric.
        let gain = factor
            .solve(&(transition * *node.covariance()))
            .ok_or(NavigationError::Kernel(KernelError::SingularSystem {
                context: "predicted covariance",
            }))?
            .transpose();
        let residual = difference(&self.vector, predicted.vector());
        self.vector = *node.vector() + gain * residual;
        self.covariance = (*node.covariance()
            + gain * (self.covariance - *predicted.covariance()) * gain.transpose())
        .symmetrised();
        self.gain_chain = gain * self.gain_chain;
        Ok(())
    }
}

/// `a − b` for state vectors, heading difference wrapped into `[−π, π)`: 0.01
/// rad vs 6.27 rad differ by ~0.02 rad, not six.
fn difference(a: &Vector<STATE_DIM>, b: &Vector<STATE_DIM>) -> Vector<STATE_DIM> {
    let mut result = *a - *b;
    let heading = StateComponent::Heading.index();
    if let Some(turn) = result.element(heading) {
        let wrapped = math::to_radians(wrap180(math::to_degrees(turn)));
        result.set(heading, 0, wrapped);
    }
    result
}

/// Late update with the observation's parts gathered, dimension still generic.
struct LateStep<'a> {
    present: &'a NavigationState,
    then: &'a NavigationState,
    gain_chain: &'a Matrix<STATE_DIM, STATE_DIM>,
    jacobian: &'a ObservationJacobian,
    noise: &'a ObservationNoise,
    innovation: &'a ObservationVector,
    gate: Option<f64>,
    sensor: SensorId,
}

impl LateStep<'_> {
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
        let p_then = *self.then.covariance();
        let p_now = *self.present.covariance();

        // The innovation is evaluated against the belief at the observation
        // time, as if in time.
        let s = (h * p_then * h.transpose() + r).symmetrised();
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
            fixes_position: super::fixes_position(self.jacobian),
        };
        if self
            .gate
            .is_some_and(|threshold| normalised_innovation_squared > threshold)
        {
            report.accepted = false;
            return Ok((*self.present, report));
        }

        // Correction applied to the present belief via its cross-covariance
        // with the belief then: P_xz = P_now Gᵀ Hᵀ, W = P_xz S⁻¹, computed as
        // (S⁻¹ P_xzᵀ)ᵀ.
        let cross = p_now * self.gain_chain.transpose() * h.transpose();
        let gain = factor
            .solve(&cross.transpose())
            .ok_or(NavigationError::Kernel(KernelError::SingularSystem {
                context: "innovation covariance",
            }))?
            .transpose();
        let corrected = *self.present.vector() + gain * nu;
        let covariance = (p_now - gain * s * gain.transpose()).symmetrised();
        let state = NavigationState::from_parts(
            self.present.valid_at(),
            *self.present.frame(),
            corrected,
            covariance,
        )?;
        Ok((state, report))
    }
}
