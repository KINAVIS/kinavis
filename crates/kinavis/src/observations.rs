//! Standard observations for the estimator.
//!
//! Each type implements [`Observation`] for one reading kind: position, ground
//! velocity, heading, speed through water. A receiver fix becomes a
//! [`PositionObservation`] and optionally a [`VelocityObservation`] on the
//! caller's side via [`PositionObservation::from_fix`] and
//! [`VelocityObservation::from_fix`]; the estimator never sees the fix.
//!
//! A position is observed in its own local frame: the observation anchors a
//! frame at the reported point and predicts the state's displacement from it,
//! so the measurement is zero and the innovation is the vector from estimate to
//! report, in metres. The observation needs no knowledge of the state anchor
//! and works in metres, not degrees.

use crate::angle::{wrap180, TrueCourse};
use crate::estimation::{
    GatingPolicy, JacobianRow, Observation, ObservationJacobian, ObservationNoise,
    ObservationVector,
};
use crate::event::SensorId;
use crate::geodesy::{Ellipsoid, GeodeticPoint, Height};
use crate::gnss::GnssFix;
use crate::local::{LocalFrame, Ned, Vector3};
use crate::math;
use crate::position::Position;
use crate::state::StateComponent::{CurrentEast, CurrentNorth, Heading, SpeedThroughWater};
use crate::state::{NavigationState, StateComponent};
use crate::time::{Instant, Utc};
use crate::units::{Angle, Distance, Speed};
use kinavis_kernel::error::Result;

/// χ² threshold exceeded with probability 0.1 % for one element: default 1-dof
/// gate.
pub const GATE_ONE_IN_A_THOUSAND_1DOF: f64 = 10.83;

/// Same for two elements.
pub const GATE_ONE_IN_A_THOUSAND_2DOF: f64 = 13.82;

/// Position from a receiver or a manual fix.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PositionObservation {
    taken_at: Instant<Utc>,
    position: Position,
    sigma_north: Distance,
    sigma_east: Distance,
    gate: GatingPolicy,
    sensor: SensorId,
}

impl PositionObservation {
    /// Position with isotropic 1σ uncertainty.
    #[must_use]
    pub fn new(taken_at: Instant<Utc>, position: Position, sigma: Distance) -> Self {
        Self {
            taken_at,
            position,
            sigma_north: sigma,
            sigma_east: sigma,
            gate: GatingPolicy::reject_above(GATE_ONE_IN_A_THOUSAND_2DOF),
            sensor: SensorId::named("position"),
        }
    }

    /// Position with separate north and east uncertainties.
    #[must_use]
    pub fn with_sigmas(mut self, north: Distance, east: Distance) -> Self {
        self.sigma_north = north;
        self.sigma_east = east;
        self
    }

    /// Sets the gate.
    #[must_use]
    pub const fn with_gate(mut self, gate: GatingPolicy) -> Self {
        self.gate = gate;
        self
    }

    /// Sets the source identifier; differently named receivers are judged
    /// separately.
    #[must_use]
    pub const fn from_sensor(mut self, sensor: SensorId) -> Self {
        self.sensor = sensor;
        self
    }

    /// Position of a fix, with its horizontal accuracy as sigma, or `fallback`
    /// if none.
    #[must_use]
    pub fn from_fix(fix: &GnssFix, fallback: Distance) -> Self {
        Self::new(
            fix.taken_at(),
            fix.position(),
            fix.horizontal_accuracy().unwrap_or(fallback),
        )
    }

    /// Reported position.
    #[must_use]
    pub const fn position(&self) -> Position {
        self.position
    }
}

impl Observation for PositionObservation {
    fn sensor(&self) -> SensorId {
        self.sensor
    }

    fn taken_at(&self) -> Instant<Utc> {
        self.taken_at
    }

    /// Zero: the observation is its own origin.
    fn measured(&self) -> ObservationVector {
        ObservationVector::new(&[0.0, 0.0]).unwrap_or(ObservationVector::EMPTY)
    }

    /// State displacement from the reported position, north and east, m.
    fn predict(&self, state: &NavigationState) -> Result<ObservationVector> {
        let here = GeodeticPoint::new(self.position, Height::above_ellipsoid(Distance::ZERO));
        let frame = LocalFrame::at(here, &Ellipsoid::WGS84)?;
        let there = GeodeticPoint::new(state.position(), Height::above_ellipsoid(Distance::ZERO));
        let displacement: Vector3<Ned, Distance> = frame.ned_of(there)?;
        ObservationVector::new(&[displacement.north().metres(), displacement.east().metres()])
    }

    /// Identity on the position components: the observation frame and the state
    /// frame are parallel well within noise for any displacement the gate
    /// passes.
    fn jacobian(&self, _: &NavigationState) -> Result<ObservationJacobian> {
        ObservationJacobian::new()
            .with_row(JacobianRow::new().with(StateComponent::North, 1.0))?
            .with_row(JacobianRow::new().with(StateComponent::East, 1.0))
    }

    fn noise(&self) -> ObservationNoise {
        sigmas(&[self.sigma_north.metres(), self.sigma_east.metres()])
    }

    fn gate(&self) -> GatingPolicy {
        self.gate
    }
}

/// Ground velocity (receiver course and speed).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VelocityObservation {
    taken_at: Instant<Utc>,
    velocity: Vector3<Ned, Speed>,
    sigma: Speed,
    gate: GatingPolicy,
    sensor: SensorId,
}

impl VelocityObservation {
    /// Velocity with the same 1σ on each component.
    #[must_use]
    pub fn new(taken_at: Instant<Utc>, velocity: Vector3<Ned, Speed>, sigma: Speed) -> Self {
        Self {
            taken_at,
            velocity,
            sigma,
            gate: GatingPolicy::reject_above(GATE_ONE_IN_A_THOUSAND_2DOF),
            sensor: SensorId::named("velocity over ground"),
        }
    }

    /// Course and speed over ground.
    #[must_use]
    pub fn from_track(
        taken_at: Instant<Utc>,
        course: TrueCourse,
        speed: Speed,
        sigma: Speed,
    ) -> Self {
        let radians = math::to_radians(course.degrees());
        Self::new(
            taken_at,
            Vector3::new(
                speed * math::cos(radians),
                speed * math::sin(radians),
                Speed::ZERO,
            ),
            sigma,
        )
    }

    /// Velocity of a fix, if it reports both course and speed.
    #[must_use]
    pub fn from_fix(fix: &GnssFix, sigma: Speed) -> Option<Self> {
        Some(Self::from_track(
            fix.taken_at(),
            fix.course_over_ground()?,
            fix.speed_over_ground()?,
            sigma,
        ))
    }

    /// Sets the gate.
    #[must_use]
    pub const fn with_gate(mut self, gate: GatingPolicy) -> Self {
        self.gate = gate;
        self
    }

    /// Sets the source identifier; differently named receivers are judged
    /// separately.
    #[must_use]
    pub const fn from_sensor(mut self, sensor: SensorId) -> Self {
        self.sensor = sensor;
        self
    }
}

impl Observation for VelocityObservation {
    fn sensor(&self) -> SensorId {
        self.sensor
    }

    fn taken_at(&self) -> Instant<Utc> {
        self.taken_at
    }

    fn measured(&self) -> ObservationVector {
        vector(&[
            self.velocity.north().metres_per_second(),
            self.velocity.east().metres_per_second(),
        ])
    }

    /// `u (cos ψ, sin ψ) + c`.
    fn predict(&self, state: &NavigationState) -> Result<ObservationVector> {
        let velocity = state.velocity_over_ground();
        ObservationVector::new(&[
            velocity.north().metres_per_second(),
            velocity.east().metres_per_second(),
        ])
    }

    fn jacobian(&self, state: &NavigationState) -> Result<ObservationJacobian> {
        let heading = math::to_radians(state.heading().degrees());
        let speed = state.speed_through_water().metres_per_second();
        let (sin, cos) = (math::sin(heading), math::cos(heading));
        ObservationJacobian::new()
            .with_row(
                JacobianRow::new()
                    .with(Heading, -speed * sin)
                    .with(SpeedThroughWater, cos)
                    .with(CurrentNorth, 1.0),
            )?
            .with_row(
                JacobianRow::new()
                    .with(Heading, speed * cos)
                    .with(SpeedThroughWater, sin)
                    .with(CurrentEast, 1.0),
            )
    }

    fn noise(&self) -> ObservationNoise {
        let sigma = self.sigma.metres_per_second();
        sigmas(&[sigma, sigma])
    }

    fn gate(&self) -> GatingPolicy {
        self.gate
    }
}

/// Heading from a gyrocompass or corrected magnetic compass.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HeadingObservation {
    taken_at: Instant<Utc>,
    heading: TrueCourse,
    sigma: Angle,
    gate: GatingPolicy,
    sensor: SensorId,
}

impl HeadingObservation {
    /// Heading with 1σ uncertainty.
    #[must_use]
    pub fn new(taken_at: Instant<Utc>, heading: TrueCourse, sigma: Angle) -> Self {
        Self {
            taken_at,
            heading,
            sigma,
            gate: GatingPolicy::reject_above(GATE_ONE_IN_A_THOUSAND_1DOF),
            sensor: SensorId::named("heading"),
        }
    }

    /// Sets the gate.
    #[must_use]
    pub const fn with_gate(mut self, gate: GatingPolicy) -> Self {
        self.gate = gate;
        self
    }

    /// Sets the source identifier; differently named sensors are judged
    /// separately.
    #[must_use]
    pub const fn from_sensor(mut self, sensor: SensorId) -> Self {
        self.sensor = sensor;
        self
    }
}

impl Observation for HeadingObservation {
    fn sensor(&self) -> SensorId {
        self.sensor
    }

    fn taken_at(&self) -> Instant<Utc> {
        self.taken_at
    }

    fn measured(&self) -> ObservationVector {
        vector(&[math::to_radians(self.heading.degrees())])
    }

    fn predict(&self, state: &NavigationState) -> Result<ObservationVector> {
        ObservationVector::new(&[math::to_radians(state.heading().degrees())])
    }

    fn jacobian(&self, _: &NavigationState) -> Result<ObservationJacobian> {
        ObservationJacobian::new().with_row(JacobianRow::new().with(StateComponent::Heading, 1.0))
    }

    fn noise(&self) -> ObservationNoise {
        sigmas(&[self.sigma.radians()])
    }

    fn gate(&self) -> GatingPolicy {
        self.gate
    }

    /// Difference wrapped to `[−π, π)`: 359° vs predicted 1° is −2°, not 358°.
    fn innovation(&self, predicted: &ObservationVector) -> Result<ObservationVector> {
        let measured = math::to_radians(self.heading.degrees());
        let predicted = predicted.get(0).unwrap_or(0.0);
        let difference = math::to_radians(wrap180(math::to_degrees(measured - predicted)));
        ObservationVector::new(&[difference])
    }
}

/// Speed through the water from a log.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SpeedThroughWaterObservation {
    taken_at: Instant<Utc>,
    speed: Speed,
    sigma: Speed,
    gate: GatingPolicy,
    sensor: SensorId,
}

impl SpeedThroughWaterObservation {
    /// Speed with 1σ uncertainty.
    #[must_use]
    pub fn new(taken_at: Instant<Utc>, speed: Speed, sigma: Speed) -> Self {
        Self {
            taken_at,
            speed,
            sigma,
            gate: GatingPolicy::reject_above(GATE_ONE_IN_A_THOUSAND_1DOF),
            sensor: SensorId::named("speed through water"),
        }
    }

    /// Sets the gate.
    #[must_use]
    pub const fn with_gate(mut self, gate: GatingPolicy) -> Self {
        self.gate = gate;
        self
    }

    /// Sets the source identifier; differently named sensors are judged
    /// separately.
    #[must_use]
    pub const fn from_sensor(mut self, sensor: SensorId) -> Self {
        self.sensor = sensor;
        self
    }
}

impl Observation for SpeedThroughWaterObservation {
    fn sensor(&self) -> SensorId {
        self.sensor
    }

    fn taken_at(&self) -> Instant<Utc> {
        self.taken_at
    }

    fn measured(&self) -> ObservationVector {
        vector(&[self.speed.metres_per_second()])
    }

    fn predict(&self, state: &NavigationState) -> Result<ObservationVector> {
        ObservationVector::new(&[state.speed_through_water().metres_per_second()])
    }

    fn jacobian(&self, _: &NavigationState) -> Result<ObservationJacobian> {
        ObservationJacobian::new()
            .with_row(JacobianRow::new().with(StateComponent::SpeedThroughWater, 1.0))
    }

    fn noise(&self) -> ObservationNoise {
        sigmas(&[self.sigma.metres_per_second()])
    }

    fn gate(&self) -> GatingPolicy {
        self.gate
    }
}

/// Observation vector from values already finite and few; cannot fail for them.
/// Falls back to empty (rejected by the estimator) instead of panicking.
fn vector(values: &[f64]) -> ObservationVector {
    ObservationVector::new(values).unwrap_or(ObservationVector::EMPTY)
}

/// Noise from validated sigmas. A zero sigma (allowed by the units) becomes the
/// smallest positive variance, since a noiseless observation makes the update
/// singular.
fn sigmas(values: &[f64]) -> ObservationNoise {
    let floor = |sigma: &f64| sigma.max(1e-9);
    let mut floored = [0.0; 4];
    for (slot, sigma) in floored.iter_mut().zip(values) {
        *slot = floor(sigma);
    }
    ObservationNoise::sigmas(floored.get(..values.len().min(4)).unwrap_or(&[]))
        .unwrap_or(ObservationNoise::UNIT)
}
