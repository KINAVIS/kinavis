//! The kinematic bicycle as a process model for the `kinavis` estimator: a
//! machine on land without an inertial system, steered by a known wheel
//! angle.

use core::time::Duration;

use kinavis_kernel::estimation::{ProcessModel, ProcessNoise, StateJacobian};
use kinavis_kernel::inline::Inline;
use kinavis_kernel::matrix::Matrix;
use kinavis_kernel::state::{NavigationState, StateComponent, STATE_DIM};
use kinavis_kernel::{math, Angle, Instant, KernelError, Speed, Utc};

use crate::error::Result;
use crate::steering::VehicleGeometry;

use StateComponent::{CurrentEast, CurrentNorth, East, Heading, North, SpeedThroughWater};

/// Process model of a front-steered machine on land: the rear axle moves on
/// arcs of the curvature `tan δ / L` of the wheel angle in force.
///
/// The estimator's state is read the land way: position is the rear axle,
/// `SpeedThroughWater` is the forward speed over the ground (what the wheels
/// or the radar measure), `CurrentNorth`/`CurrentEast` the side slip of the
/// machine over the ground, near zero on firm ground.
///
/// Wheel angles are fed with [`BicycleMotion::steer`] as they are commanded
/// or measured, stamped, through [`kinavis::estimator::Estimator::process_mut`]. The
/// model keeps the last `N` and propagates each step through the angles in
/// force during it, so the re-propagation of a late observation uses the
/// angles of its own time. Before the oldest kept angle that angle is
/// assumed; with none the machine runs straight.
///
/// Heading, speed and slip random-walk with the given intensities (1σ change
/// over one second); the noise follows them to second order over sub-steps
/// in which the heading turns at most 0.01 rad, so a step's noise equals its
/// halves' to 10⁻⁵.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BicycleMotion<const N: usize> {
    wheelbase: f64,
    log: Inline<(Instant<Utc>, f64), N>,
    heading: f64,
    speed: f64,
    slip: f64,
}

impl<const N: usize> BicycleMotion<N> {
    /// Model of a machine with intensities per √s for heading, forward speed
    /// and each component of slip.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] for a negative intensity;
    /// [`KernelError::CapacityExceeded`] for `N` of zero.
    pub fn new(
        vehicle: &VehicleGeometry,
        heading: Angle,
        speed: Speed,
        slip: Speed,
    ) -> Result<Self> {
        for (name, value) in [
            ("heading intensity, rad/√s", heading.radians()),
            ("speed intensity, m/s/√s", speed.metres_per_second()),
            ("slip intensity, m/s/√s", slip.metres_per_second()),
        ] {
            kinavis_kernel::error::ensure_range(name, value, 0.0, 10.0)?;
        }
        if N == 0 {
            return Err(KernelError::CapacityExceeded {
                context: "a log of wheel angles",
                needed: 1,
                capacity: 0,
            }
            .into());
        }
        Ok(Self {
            wheelbase: vehicle.wheelbase().metres(),
            log: Inline::new((Instant::UNIX_EPOCH, 0.0)),
            heading: heading.radians(),
            speed: speed.metres_per_second(),
            slip: slip.metres_per_second(),
        })
    }

    /// Tractor in field work: heading 0.3°, speed 0.05 m/s and slip
    /// 0.01 m/s per √s.
    ///
    /// # Errors
    ///
    /// As [`BicycleMotion::new`].
    pub fn standard(vehicle: &VehicleGeometry) -> Result<Self> {
        Self::new(
            vehicle,
            Angle::from_degrees(0.3)?,
            Speed::from_metres_per_second(0.05)?,
            Speed::from_metres_per_second(0.01)?,
        )
    }

    /// Records the wheel angle from `at` on, right positive. When the log is
    /// full the oldest angle goes.
    ///
    /// # Errors
    ///
    /// [`KernelError::TimeReversed`] for an instant before the last recorded.
    pub fn steer(&mut self, at: Instant<Utc>, wheel_angle: Angle) -> Result<()> {
        let curvature = math::tan(wheel_angle.radians()) / self.wheelbase;
        if let Some((last, _)) = self.log.as_slice().last() {
            if at < *last {
                return Err(KernelError::TimeReversed {
                    by: last.checked_duration_since(at).unwrap_or(Duration::ZERO),
                }
                .into());
            }
        }
        if self.log.len() >= N {
            let _ = self.log.remove(0);
        }
        // Room was made above, so the push succeeds.
        let _ = self.log.push((at, curvature));
        Ok(())
    }

    /// Curvature in force at an instant, 1/m, right positive.
    #[must_use]
    pub fn curvature_at(&self, at: Instant<Utc>) -> f64 {
        let log = self.log.as_slice();
        log.iter()
            .rev()
            .find(|(from, _)| *from <= at)
            .or_else(|| log.first())
            .map_or(0.0, |(_, curvature)| *curvature)
    }

    /// Pieces of a step from `start` over `over`: duration and curvature of
    /// each, in order, at most `N + 1`, passed to `each`.
    fn pieces(&self, start: Instant<Utc>, over: Duration, mut each: impl FnMut(f64, f64)) {
        let end = start.saturating_add(over);
        let mut from = start;
        for (at, _) in self.log.as_slice() {
            if *at > from && *at < end {
                let length = at.checked_duration_since(from).unwrap_or(Duration::ZERO);
                each(length.as_secs_f64(), self.curvature_at(from));
                from = *at;
            }
        }
        let rest = end.checked_duration_since(from).unwrap_or(Duration::ZERO);
        each(rest.as_secs_f64(), self.curvature_at(from));
    }

    /// Noise of one piece of constant curvature, linearised at its start.
    fn piece_noise(
        &self,
        x: &[f64; STATE_DIM],
        t: f64,
        curvature: f64,
    ) -> Matrix<STATE_DIM, STATE_DIM> {
        let [_, _, heading, speed, _, _] = *x;
        let middle = heading + 0.5 * curvature * speed * t;
        let (sin, cos) = (math::sin(middle), math::cos(middle));
        let heading_variance = self.heading * self.heading * t;
        let speed_variance = self.speed * self.speed * t;
        let slip_variance = self.slip * self.slip * t;
        // A walk of σ√t moves its integral by σ t^{3/2}/√3, correlated σ² t²/2.
        let along = speed_variance * t * t / 3.0;
        let across = heading_variance * speed * speed * t * t / 3.0;
        let drift = slip_variance * t * t / 3.0;
        let along_speed = speed_variance * t / 2.0;
        let across_heading = heading_variance * speed * t / 2.0;
        let drift_slip = slip_variance * t / 2.0;
        // A speed error turns the heading by the curvature times the distance
        // it adds: δψ = κ · δs.
        let noise = ProcessNoise::zero()
            .with_variance(North, along * cos * cos + across * sin * sin + drift)
            .with_variance(East, along * sin * sin + across * cos * cos + drift)
            .with_covariance(North, East, (along - across) * sin * cos)
            .with_variance(Heading, heading_variance + curvature * curvature * along)
            .with_variance(SpeedThroughWater, speed_variance)
            .with_variance(CurrentNorth, slip_variance)
            .with_variance(CurrentEast, slip_variance)
            .with_covariance(North, SpeedThroughWater, along_speed * cos)
            .with_covariance(East, SpeedThroughWater, along_speed * sin)
            .with_covariance(Heading, SpeedThroughWater, curvature * along_speed)
            .with_covariance(
                North,
                Heading,
                -across_heading * sin + curvature * along * cos,
            )
            .with_covariance(
                East,
                Heading,
                across_heading * cos + curvature * along * sin,
            )
            .with_covariance(North, CurrentNorth, drift_slip)
            .with_covariance(East, CurrentEast, drift_slip);
        *noise.matrix()
    }
}

/// State after one piece of constant curvature, and the piece's Jacobian.
fn piece(
    x: &[f64; STATE_DIM],
    t: f64,
    curvature: f64,
) -> ([f64; STATE_DIM], Matrix<STATE_DIM, STATE_DIM>) {
    let [north, east, heading, speed, slip_north, slip_east] = *x;
    let turn = curvature * speed * t;
    let half = 0.5 * turn;
    let chord = speed * t * sinc(half);
    let middle = heading + half;
    let (sin, cos) = (math::sin(middle), math::cos(middle));
    let next = [
        north + chord * cos + slip_north * t,
        east + chord * sin + slip_east * t,
        heading + turn,
        speed,
        slip_north,
        slip_east,
    ];
    let chord_speed = t * math::cos(half);
    let middle_speed = 0.5 * curvature * t;
    let jacobian = StateJacobian::identity()
        .with(North, Heading, -chord * sin)
        .with(East, Heading, chord * cos)
        .with(
            North,
            SpeedThroughWater,
            chord_speed * cos - chord * sin * middle_speed,
        )
        .with(
            East,
            SpeedThroughWater,
            chord_speed * sin + chord * cos * middle_speed,
        )
        .with(Heading, SpeedThroughWater, curvature * t)
        .with(North, CurrentNorth, t)
        .with(East, CurrentEast, t);
    (next, *jacobian.matrix())
}

/// Largest heading turn of one sub-piece of the noise, radians.
const MAX_SUB_TURN: f64 = 0.01;

/// Most sub-pieces of one piece of the noise.
const MAX_SUB_PIECES: f64 = 64.0;

fn sinc(x: f64) -> f64 {
    if math::abs(x) < 1e-4 {
        1.0 - x * x / 6.0
    } else {
        math::sin(x) / x
    }
}

fn components(state: &NavigationState) -> [f64; STATE_DIM] {
    state.vector().to_column()
}

impl<const N: usize> ProcessModel for BicycleMotion<N> {
    /// Moves the rear axle along the arcs of the wheel angles in force, plus
    /// the slip; heading turns with the distance run; speed and slip are
    /// held.
    fn propagate(
        &self,
        state: &NavigationState,
        over: Duration,
    ) -> kinavis_kernel::error::Result<NavigationState> {
        let mut x = components(state);
        self.pieces(state.valid_at(), over, |t, curvature| {
            x = piece(&x, t, curvature).0;
        });
        let when = state
            .valid_at()
            .checked_add(over)
            .ok_or(KernelError::Unrepresentable {
                what: "a moment beyond the end of time",
            })?;
        let mut vector = *state.vector();
        for (index, value) in x.iter().enumerate() {
            vector.set(index, 0, *value);
        }
        NavigationState::from_parts(when, *state.frame(), vector, *state.covariance())
    }

    /// Product of the pieces' Jacobians.
    fn jacobian(
        &self,
        state: &NavigationState,
        over: Duration,
    ) -> kinavis_kernel::error::Result<StateJacobian> {
        let mut x = components(state);
        let mut total = Matrix::<STATE_DIM, STATE_DIM>::identity();
        self.pieces(state.valid_at(), over, |t, curvature| {
            let (next, jacobian) = piece(&x, t, curvature);
            total = jacobian * total;
            x = next;
        });
        Ok(from_matrix(&total))
    }

    /// The pieces' noise, each carried through the pieces after it.
    fn noise(&self, state: &NavigationState, over: Duration) -> ProcessNoise {
        let mut x = components(state);
        let mut total = Matrix::<STATE_DIM, STATE_DIM>::ZERO;
        self.pieces(state.valid_at(), over, |t, curvature| {
            // Short enough sub-pieces that the heading barely turns in each,
            // so the linearisation holds and a step equals its halves.
            let turn = math::abs(curvature * x[3] * t);
            let count = math::ceil(turn / MAX_SUB_TURN).clamp(1.0, MAX_SUB_PIECES);
            let sub = t / count;
            for _ in 0..math::to_usize(count) {
                let (next, jacobian) = piece(&x, sub, curvature);
                total =
                    jacobian * total * jacobian.transpose() + self.piece_noise(&x, sub, curvature);
                x = next;
            }
        });
        let mut noise = ProcessNoise::zero();
        for i in 0..STATE_DIM {
            for j in i..STATE_DIM {
                let (Some(a), Some(b)) = (StateComponent::ALL.get(i), StateComponent::ALL.get(j))
                else {
                    continue;
                };
                let value = f64::midpoint(
                    total.get(i, j).unwrap_or(0.0),
                    total.get(j, i).unwrap_or(0.0),
                );
                noise = noise.with_covariance(*a, *b, value);
            }
        }
        noise
    }
}

fn from_matrix(matrix: &Matrix<STATE_DIM, STATE_DIM>) -> StateJacobian {
    let mut jacobian = StateJacobian::identity();
    for (i, row) in StateComponent::ALL.iter().enumerate() {
        for (j, column) in StateComponent::ALL.iter().enumerate() {
            jacobian = jacobian.with(*row, *column, matrix.get(i, j).unwrap_or(0.0));
        }
    }
    jacobian
}
