//! The INS as a process model for the six-state estimator.
//!
//! `kinavis::estimator` fuses GNSS, gyrocompass and log, propagating between
//! observations with a [`ProcessModel`]. The default model holds heading and
//! speed as random walks; with an INS, this model uses the measured ground
//! velocity and rate of turn, held constant over the step.
//!
//! Position and heading are propagated at those rates; speed through the water
//! and current are held. Noise is a random walk of constant intensity, so the
//! noise of a step equals the sum over its halves, as late-observation handling
//! in the estimator requires.

use core::time::Duration;

use kinavis_kernel::error::{ensure_finite, ensure_range, KernelError, Result};
use kinavis_kernel::estimation::{ProcessModel, ProcessNoise, StateJacobian};
use kinavis_kernel::local::{Ned, Vector3};
use kinavis_kernel::state::StateComponent::{East, Heading, North};
use kinavis_kernel::state::{NavigationState, StateComponent};
use kinavis_kernel::units::{Angle, RateOfTurn, Speed};

use crate::filter::InsFilter;

/// INS-reported motion as a process model.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(try_from = "StoredInsMotion", into = "StoredInsMotion")
)]
pub struct InsMotion {
    /// m/s, north and east.
    velocity: [f64; 2],
    /// rad/s, positive to starboard.
    yaw_rate: f64,
    /// Position random walk, m/√s (1σ over 1 s).
    position_walk: f64,
    /// Heading random walk, rad/√s.
    heading_walk: f64,
}

impl InsMotion {
    /// Model from the reported ground velocity and rate of turn, and the
    /// position and heading random-walk intensities (1σ over 1 s).
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] for a non-positive walk.
    pub fn new(
        velocity: Vector3<Ned, Speed>,
        yaw_rate: RateOfTurn,
        position_walk: Speed,
        heading_walk: Angle,
    ) -> Result<Self> {
        Self::from_raw(Self {
            velocity: [
                velocity.north().metres_per_second(),
                velocity.east().metres_per_second(),
            ],
            yaw_rate: yaw_rate.radians_per_second(),
            position_walk: position_walk.metres_per_second(),
            heading_walk: heading_walk.radians(),
        })
    }

    /// Single check shared by the typed constructor and deserialisation. Typed
    /// arguments are finite by construction; stored ones are checked here too.
    fn from_raw(motion: Self) -> Result<Self> {
        for value in motion.velocity {
            ensure_finite("velocity", value)?;
        }
        ensure_finite("yaw rate", motion.yaw_rate)?;
        ensure_range(
            "position walk",
            motion.position_walk,
            f64::MIN_POSITIVE,
            1e3,
        )?;
        ensure_range("heading walk", motion.heading_walk, f64::MIN_POSITIVE, 10.0)?;
        Ok(motion)
    }

    /// Model from a filter's current velocity and its sigma, and a gyro rate of
    /// turn; heading walk 0.5°/√s as in the steady-motion model.
    ///
    /// # Errors
    ///
    /// As [`InsMotion::new`]: a collapsed velocity sigma is rejected.
    pub fn from_filter(filter: &InsFilter, yaw_rate: RateOfTurn) -> Result<Self> {
        Self::new(
            filter.velocity(),
            yaw_rate,
            filter.velocity_sigma(),
            Angle::from_degrees(0.5)?,
        )
    }

    /// Propagation velocity, north and east.
    #[must_use]
    pub fn velocity(&self) -> Vector3<Ned, Speed> {
        let speed = |value: f64| Speed::from_metres_per_second(value).unwrap_or(Speed::ZERO);
        Vector3::new(
            speed(self.velocity[0]),
            speed(self.velocity[1]),
            Speed::ZERO,
        )
    }

    /// Heading rate.
    #[must_use]
    pub fn yaw_rate(&self) -> RateOfTurn {
        RateOfTurn::from_radians_per_second(self.yaw_rate).unwrap_or(RateOfTurn::ZERO)
    }
}

/// Serialised form; deserialisation applies the [`InsMotion::new`] checks.
#[cfg(feature = "serde")]
#[derive(serde::Serialize, serde::Deserialize)]
struct StoredInsMotion {
    velocity: [f64; 2],
    yaw_rate: f64,
    position_walk: f64,
    heading_walk: f64,
}

#[cfg(feature = "serde")]
impl TryFrom<StoredInsMotion> for InsMotion {
    type Error = KernelError;

    fn try_from(stored: StoredInsMotion) -> Result<Self> {
        Self::from_raw(Self {
            velocity: stored.velocity,
            yaw_rate: stored.yaw_rate,
            position_walk: stored.position_walk,
            heading_walk: stored.heading_walk,
        })
    }
}

#[cfg(feature = "serde")]
impl From<InsMotion> for StoredInsMotion {
    fn from(motion: InsMotion) -> Self {
        Self {
            velocity: motion.velocity,
            yaw_rate: motion.yaw_rate,
            position_walk: motion.position_walk,
            heading_walk: motion.heading_walk,
        }
    }
}

impl ProcessModel for InsMotion {
    /// Moves position by the reported velocity and heading by the reported
    /// rate; the rest of the state is held.
    fn propagate(&self, state: &NavigationState, over: Duration) -> Result<NavigationState> {
        let seconds = over.as_secs_f64();
        let vector = state.vector();
        let get = |component: StateComponent| vector.element(component.index()).unwrap_or(0.0);
        let mut moved = *vector;
        moved.set(North.index(), 0, get(North) + self.velocity[0] * seconds);
        moved.set(East.index(), 0, get(East) + self.velocity[1] * seconds);
        moved.set(Heading.index(), 0, get(Heading) + self.yaw_rate * seconds);
        let when = state
            .valid_at()
            .checked_add(over)
            .ok_or(KernelError::Indeterminate {
                quantity: "a moment beyond the end of time",
            })?;
        NavigationState::from_parts(when, *state.frame(), moved, *state.covariance())
    }

    /// Identity: the step depends on the reported motion, not on the state.
    fn jacobian(&self, _: &NavigationState, _: Duration) -> Result<StateJacobian> {
        Ok(StateJacobian::identity())
    }

    /// Random-walk variance over the step, `σ² t`, on position and heading.
    fn noise(&self, _: &NavigationState, over: Duration) -> ProcessNoise {
        let seconds = over.as_secs_f64();
        let position = self.position_walk * self.position_walk * seconds;
        let heading = self.heading_walk * self.heading_walk * seconds;
        ProcessNoise::zero()
            .with_variance(North, position)
            .with_variance(East, position)
            .with_variance(Heading, heading)
    }
}
