//! Standard process model.

use core::time::Duration;

use crate::estimation::{ProcessModel, ProcessNoise, StateJacobian};
use crate::math;
use crate::state::StateComponent::{
    CurrentEast, CurrentNorth, East, Heading, North, SpeedThroughWater,
};
use crate::state::{NavigationState, StateComponent};
use crate::units::{hours, Angle, Speed};
use kinavis_kernel::error::{KernelError, Result};

/// Standard process model: heading, speed through the water and current held
/// constant with random-walk noise; position integrates the resulting ground
/// velocity.
///
/// Intensities are the 1σ change over one second (variance per step `σ² ×
/// seconds`). Hand steering varies heading faster than autopilot; tidal streams
/// vary faster than ocean currents. Standard values suit a merchant vessel on
/// passage.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SteadyMotion {
    /// rad/√s.
    heading: f64,
    /// m/s/√s.
    speed: f64,
    /// m/s/√s per component.
    current: f64,
}

impl SteadyMotion {
    /// Model from 1σ-per-√s intensities for heading, water speed and each
    /// current component.
    #[must_use]
    pub fn new(heading: Angle, speed: Speed, current: Speed) -> Self {
        Self {
            heading: heading.radians(),
            speed: speed.metres_per_second(),
            current: current.metres_per_second(),
        }
    }

    /// Merchant vessel on passage: 0.5°, 0.1 kn and 0.05 kn per √s.
    #[must_use]
    pub fn standard() -> Self {
        Self {
            heading: math::to_radians(0.5),
            speed: 0.05,
            current: 0.025,
        }
    }
}

impl ProcessModel for SteadyMotion {
    /// Moves position by the ground velocity over the step; the rest of the
    /// state is held.
    fn propagate(&self, state: &NavigationState, over: Duration) -> Result<NavigationState> {
        let seconds = hours(over) * 3600.0;
        let vector = state.vector();
        let get = |component: StateComponent| vector.element(component.index()).unwrap_or(0.0);
        let heading = get(StateComponent::Heading);
        let speed = get(StateComponent::SpeedThroughWater);
        let velocity_north = speed * math::cos(heading) + get(StateComponent::CurrentNorth);
        let velocity_east = speed * math::sin(heading) + get(StateComponent::CurrentEast);
        let mut moved = *vector;
        moved.set(
            StateComponent::North.index(),
            0,
            get(StateComponent::North) + velocity_north * seconds,
        );
        moved.set(
            StateComponent::East.index(),
            0,
            get(StateComponent::East) + velocity_east * seconds,
        );
        let when = state
            .valid_at()
            .checked_add(over)
            .ok_or(KernelError::Unrepresentable {
                what: "a moment beyond the end of time",
            })?;
        NavigationState::from_parts(when, *state.frame(), moved, *state.covariance())
    }

    fn jacobian(&self, state: &NavigationState, over: Duration) -> Result<StateJacobian> {
        let seconds = hours(over) * 3600.0;
        let vector = state.vector();
        let get = |component: StateComponent| vector.element(component.index()).unwrap_or(0.0);
        let heading = get(StateComponent::Heading);
        let speed = get(StateComponent::SpeedThroughWater);
        let (sin, cos) = (math::sin(heading), math::cos(heading));
        Ok(StateJacobian::identity()
            .with(North, Heading, -speed * sin * seconds)
            .with(North, SpeedThroughWater, cos * seconds)
            .with(North, CurrentNorth, seconds)
            .with(East, Heading, speed * cos * seconds)
            .with(East, SpeedThroughWater, sin * seconds)
            .with(East, CurrentEast, seconds))
    }

    /// Random-walk variance over the step and its effect on position: a
    /// velocity wandering by `σ√t` moves position by `σ t^{3/2}/√3`, correlated
    /// by `σ² t²/2`. Speed wanders along the heading, heading (times speed)
    /// across it, current per component. Constructed so a step's noise equals
    /// the two halves' noise propagated through the Jacobian, as exact
    /// late-observation handling requires.
    fn noise(&self, state: &NavigationState, over: Duration) -> ProcessNoise {
        let t = hours(over) * 3600.0;
        let vector = state.vector();
        let get = |component: StateComponent| vector.element(component.index()).unwrap_or(0.0);
        let heading = get(StateComponent::Heading);
        let speed = get(StateComponent::SpeedThroughWater);
        let (sin, cos) = (math::sin(heading), math::cos(heading));

        let heading_variance = self.heading * self.heading * t;
        let speed_variance = self.speed * self.speed * t;
        let current_variance = self.current * self.current * t;
        // Position variance induced by each walk, and its correlation with that
        // walk.
        let along = speed_variance * t * t / 3.0;
        let across = heading_variance * speed * speed * t * t / 3.0;
        let drift = current_variance * t * t / 3.0;
        let position_speed = speed_variance * t / 2.0;
        let position_heading = heading_variance * speed * t / 2.0;
        let position_current = current_variance * t / 2.0;
        ProcessNoise::zero()
            .with_variance(North, along * cos * cos + across * sin * sin + drift)
            .with_variance(East, along * sin * sin + across * cos * cos + drift)
            .with_covariance(North, East, (along - across) * sin * cos)
            .with_variance(Heading, heading_variance)
            .with_variance(SpeedThroughWater, speed_variance)
            .with_variance(CurrentNorth, current_variance)
            .with_variance(CurrentEast, current_variance)
            .with_covariance(North, SpeedThroughWater, position_speed * cos)
            .with_covariance(East, SpeedThroughWater, position_speed * sin)
            .with_covariance(North, Heading, -position_heading * sin)
            .with_covariance(East, Heading, position_heading * cos)
            .with_covariance(North, CurrentNorth, position_current)
            .with_covariance(East, CurrentEast, position_current)
    }
}
