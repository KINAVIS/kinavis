//! Closed-loop 15-state error-state Kalman filter around the mechanisation.
//!
//! The mechanisation integrates; the filter estimates its errors — position,
//! velocity, attitude, gyro bias, accelerometer bias, three components each.
//! After each aiding update the error estimate is fed back into the nominal
//! state and reset to zero (closed loop), keeping the linearisation valid.
//!
//! Error dynamics between updates follow the standard navigation-frame model:
//! position error from velocity error, tilt coupling specific force into
//! velocity error, gyro bias into tilt, Earth rate coupling the tilts.
//! Observations (position, velocity, heading, zero velocity) are linear in the
//! error state; the update is the standard Kalman update in Joseph form to keep
//! the covariance symmetric positive semi-definite.
//!
//! Outputs match the six-state estimator: position with an error ellipse,
//! velocity, heading with sigma. The covariance matrix is not exposed.

use kinavis_kernel::angle::{wrap180, TrueCourse};
use kinavis_kernel::error::{ensure_range, KernelError, Result};
use kinavis_kernel::estimation::GatingPolicy;
use kinavis_kernel::geodesy::GeodeticPoint;
use kinavis_kernel::local::{Body, Ned, Vector3};
use kinavis_kernel::math;
use kinavis_kernel::matrix::Matrix;
use kinavis_kernel::position::Position;
use kinavis_kernel::snapshot::ErrorEllipse;
use kinavis_kernel::time::{Instant, Utc};
use kinavis_kernel::units::{Angle, Distance, Speed};

use crate::attitude::{skew, Attitude, Quaternion};
use crate::imu::{ImuNoise, ImuSample};
use crate::mechanisation::Strapdown;

/// Error-state dimension: position, velocity, attitude, gyro bias,
/// accelerometer bias; three each.
pub const ERROR_STATE_DIM: usize = 15;

/// Start index of each error-state block.
const POSITION: usize = 0;
const VELOCITY: usize = 3;
const ATTITUDE: usize = 6;
const GYRO_BIAS: usize = 9;
const ACCEL_BIAS: usize = 12;

/// Initial 1σ uncertainty of each part of the state.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(try_from = "StoredInsPriors", into = "StoredInsPriors")
)]
pub struct InsPriors {
    /// Horizontal position component, m.
    horizontal: f64,
    /// Height, m.
    vertical: f64,
    /// Velocity component, m/s.
    velocity: f64,
    /// Roll and pitch, rad.
    tilt: f64,
    /// Heading, rad.
    heading: f64,
    /// Gyro bias per axis, rad/s.
    gyro_bias: f64,
    /// Accelerometer bias per axis, m/s².
    accel_bias: f64,
}

impl InsPriors {
    /// Priors from 1σ values: horizontal and vertical position, velocity, tilt,
    /// heading, gyro bias (rad/s), accelerometer bias (m/s²).
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] for a sigma that is not positive and finite.
    pub fn new(
        horizontal: Distance,
        vertical: Distance,
        velocity: Speed,
        tilt: Angle,
        heading: Angle,
        gyro_bias: f64,
        accel_bias: f64,
    ) -> Result<Self> {
        Self::from_raw(Self {
            horizontal: horizontal.metres(),
            vertical: vertical.metres(),
            velocity: velocity.metres_per_second(),
            tilt: tilt.radians(),
            heading: heading.radians(),
            gyro_bias,
            accel_bias,
        })
    }

    /// Single range check shared by the typed constructor and deserialisation.
    fn from_raw(priors: Self) -> Result<Self> {
        for (name, value) in [
            ("horizontal position prior", priors.horizontal),
            ("vertical position prior", priors.vertical),
            ("velocity prior", priors.velocity),
            ("tilt prior", priors.tilt),
            ("heading prior", priors.heading),
            ("gyro bias prior", priors.gyro_bias),
            ("accelerometer bias prior", priors.accel_bias),
        ] {
            ensure_range(name, value, f64::MIN_POSITIVE, 1e6)?;
        }
        Ok(priors)
    }

    /// Initialisation from a GNSS fix and a gyrocompass: 10 m / 20 m position,
    /// 0.5 m/s, 1° tilt, 3° heading, 0.05°/s gyro bias, 2 mg accelerometer
    /// bias.
    #[must_use]
    pub fn standard() -> Self {
        Self {
            horizontal: 10.0,
            vertical: 20.0,
            velocity: 0.5,
            tilt: math::to_radians(1.0),
            heading: math::to_radians(3.0),
            gyro_bias: math::to_radians(0.05),
            accel_bias: 2e-3 * 9.806_65,
        }
    }

    fn covariance(&self) -> Matrix<ERROR_STATE_DIM, ERROR_STATE_DIM> {
        let sigmas = [
            self.horizontal,
            self.horizontal,
            self.vertical,
            self.velocity,
            self.velocity,
            self.velocity,
            self.tilt,
            self.tilt,
            self.heading,
            self.gyro_bias,
            self.gyro_bias,
            self.gyro_bias,
            self.accel_bias,
            self.accel_bias,
            self.accel_bias,
        ];
        let mut variances = [0.0; ERROR_STATE_DIM];
        for (variance, sigma) in variances.iter_mut().zip(sigmas) {
            *variance = sigma * sigma;
        }
        Matrix::diagonal(variances)
    }
}

/// Serialised form: seven raw sigmas. Deserialisation applies the
/// [`InsPriors::new`] range check, so a zero prior (singular initial
/// covariance) is rejected.
#[cfg(feature = "serde")]
#[derive(serde::Serialize, serde::Deserialize)]
struct StoredInsPriors {
    horizontal: f64,
    vertical: f64,
    velocity: f64,
    tilt: f64,
    heading: f64,
    gyro_bias: f64,
    accel_bias: f64,
}

#[cfg(feature = "serde")]
impl TryFrom<StoredInsPriors> for InsPriors {
    type Error = KernelError;

    fn try_from(stored: StoredInsPriors) -> Result<Self> {
        Self::from_raw(Self {
            horizontal: stored.horizontal,
            vertical: stored.vertical,
            velocity: stored.velocity,
            tilt: stored.tilt,
            heading: stored.heading,
            gyro_bias: stored.gyro_bias,
            accel_bias: stored.accel_bias,
        })
    }
}

#[cfg(feature = "serde")]
impl From<InsPriors> for StoredInsPriors {
    fn from(priors: InsPriors) -> Self {
        Self {
            horizontal: priors.horizontal,
            vertical: priors.vertical,
            velocity: priors.velocity,
            tilt: priors.tilt,
            heading: priors.heading,
            gyro_bias: priors.gyro_bias,
            accel_bias: priors.accel_bias,
        }
    }
}

/// Update result.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct InsUpdate {
    /// Normalised innovation squared; χ² with `dof` degrees of freedom for a
    /// consistent filter.
    pub nis: f64,
    /// Observation dimension.
    pub dof: usize,
    /// Whether the observation was applied or rejected by the gate.
    pub accepted: bool,
}

/// Inertial navigation filter: a [`Strapdown`] mechanisation with its 15-state
/// error filter.
#[derive(Debug, Clone)]
pub struct InsFilter {
    nominal: Strapdown,
    covariance: Matrix<ERROR_STATE_DIM, ERROR_STATE_DIM>,
    noise: ImuNoise,
}

impl InsFilter {
    /// Filter around an initialised mechanisation, with IMU noise and initial
    /// uncertainty.
    #[must_use]
    pub fn new(nominal: Strapdown, noise: ImuNoise, priors: &InsPriors) -> Self {
        Self {
            nominal,
            covariance: priors.covariance(),
            noise,
        }
    }

    /// Integrates one IMU sample and propagates the covariance.
    ///
    /// # Errors
    ///
    /// As [`Strapdown::step`].
    pub fn predict(&mut self, sample: &ImuSample) -> Result<()> {
        let dt = sample.over().as_secs_f64();
        let force = self.nominal.navigation_force(sample);
        let rotation = self.nominal.quaternion().to_matrix();
        let earth_rate = self.nominal.earth_rate();
        self.nominal.step(sample)?;

        // Φ = I + A dt, with the blocks of A significant over a few-millisecond
        // step: position from velocity, tilt × specific force into velocity,
        // biases into velocity and tilt via the attitude, Earth rate coupling
        // the tilts.
        let mut transition = Matrix::<ERROR_STATE_DIM, ERROR_STATE_DIM>::identity();
        let force_skew = skew(force);
        let earth_skew = skew(earth_rate);
        for row in 0..3 {
            transition.set(POSITION + row, VELOCITY + row, dt);
            for column in 0..3 {
                let c = rotation.get(row, column).unwrap_or(0.0);
                transition.set(
                    VELOCITY + row,
                    ATTITUDE + column,
                    force_skew.get(row, column).unwrap_or(0.0) * dt,
                );
                transition.set(VELOCITY + row, ACCEL_BIAS + column, -c * dt);
                transition.set(ATTITUDE + row, GYRO_BIAS + column, c * dt);
                let earth = -earth_skew.get(row, column).unwrap_or(0.0) * dt;
                let existing = transition
                    .get(ATTITUDE + row, ATTITUDE + column)
                    .unwrap_or(0.0);
                transition.set(ATTITUDE + row, ATTITUDE + column, existing + earth);
            }
        }

        // Q: white noise per channel over the step, velocity noise propagated
        // into position, bias random walks.
        let square = |value: f64| value * value;
        let velocity_variance = square(self.noise.velocity_random_walk()) * dt;
        let attitude_variance = square(self.noise.angle_random_walk()) * dt;
        let gyro_variance = square(self.noise.gyro_bias_walk()) * dt;
        let accel_variance = square(self.noise.accel_bias_walk()) * dt;
        let mut noise = Matrix::<ERROR_STATE_DIM, ERROR_STATE_DIM>::ZERO;
        for axis in 0..3 {
            noise.set(
                POSITION + axis,
                POSITION + axis,
                velocity_variance * dt * dt / 3.0,
            );
            noise.set(
                POSITION + axis,
                VELOCITY + axis,
                velocity_variance * dt / 2.0,
            );
            noise.set(
                VELOCITY + axis,
                POSITION + axis,
                velocity_variance * dt / 2.0,
            );
            noise.set(VELOCITY + axis, VELOCITY + axis, velocity_variance);
            noise.set(ATTITUDE + axis, ATTITUDE + axis, attitude_variance);
            noise.set(GYRO_BIAS + axis, GYRO_BIAS + axis, gyro_variance);
            noise.set(ACCEL_BIAS + axis, ACCEL_BIAS + axis, accel_variance);
        }

        self.covariance =
            (transition * self.covariance * transition.transpose() + noise).symmetrised();
        Ok(())
    }

    /// Applies an observation: `innovation = z − h(x̂)`, `jacobian` = `H` over
    /// the error state, `noise` = `R`.
    fn update<const M: usize>(
        &mut self,
        innovation: [f64; M],
        jacobian: &Matrix<M, ERROR_STATE_DIM>,
        noise: &Matrix<M, M>,
        gate: GatingPolicy,
    ) -> Result<InsUpdate> {
        let innovation_column =
            Matrix::<M, 1>::from_fn(|row, _| innovation.get(row).copied().unwrap_or(0.0));
        let cross = self.covariance * jacobian.transpose();
        let innovation_covariance = (*jacobian * cross + *noise).symmetrised();
        let factor = innovation_covariance
            .cholesky()
            .ok_or(KernelError::Indeterminate {
                quantity: "the innovation covariance is not positive definite",
            })?;
        let whitened = factor
            .solve(&innovation_column)
            .ok_or(KernelError::Indeterminate {
                quantity: "the innovation covariance is singular",
            })?;
        let nis = (innovation_column.transpose() * whitened)
            .get(0, 0)
            .unwrap_or(0.0);
        if gate.threshold().is_some_and(|threshold| nis > threshold) {
            return Ok(InsUpdate {
                nis,
                dof: M,
                accepted: false,
            });
        }

        // K = P Hᵀ S⁻¹, correction K y, Joseph-form covariance update.
        let inverse =
            factor
                .solve(&Matrix::<M, M>::identity())
                .ok_or(KernelError::Indeterminate {
                    quantity: "the innovation covariance is singular",
                })?;
        let gain = cross * inverse;
        let correction = gain * innovation_column;
        let identity = Matrix::<ERROR_STATE_DIM, ERROR_STATE_DIM>::identity();
        let shrink = identity - gain * *jacobian;
        self.covariance = (shrink * self.covariance * shrink.transpose()
            + gain * *noise * gain.transpose())
        .symmetrised();

        let block = |start: usize| {
            [
                correction.get(start, 0).unwrap_or(0.0),
                correction.get(start + 1, 0).unwrap_or(0.0),
                correction.get(start + 2, 0).unwrap_or(0.0),
            ]
        };
        self.nominal.correct(
            block(POSITION),
            block(VELOCITY),
            block(ATTITUDE),
            block(GYRO_BIAS),
            block(ACCEL_BIAS),
        );
        Ok(InsUpdate {
            nis,
            dof: M,
            accepted: true,
        })
    }

    /// Horizontal position fix (no height), same sigma on both axes.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] for a non-positive sigma;
    /// [`KernelError::Indeterminate`] if the innovation covariance is not
    /// positive definite (prevented by a finite sigma).
    pub fn update_position(
        &mut self,
        position: Position,
        sigma: Distance,
        gate: GatingPolicy,
    ) -> Result<InsUpdate> {
        ensure_range("position sigma", sigma.metres(), f64::MIN_POSITIVE, 1e6)?;
        let height = self.nominal.point()?.height();
        let measured = self
            .nominal
            .frame()
            .ned_of(GeodeticPoint::new(position, height))?;
        let (estimated, _, _, _) = self.nominal.raw();
        let innovation = [
            measured.north().metres() - estimated[0],
            measured.east().metres() - estimated[1],
        ];
        let jacobian = Matrix::<2, ERROR_STATE_DIM>::from_fn(|row, column| {
            f64::from(column == POSITION + row)
        });
        let variance = sigma.metres() * sigma.metres();
        self.update(
            innovation,
            &jacobian,
            &Matrix::diagonal([variance, variance]),
            gate,
        )
    }

    /// Position fix with height above the ellipsoid.
    ///
    /// # Errors
    ///
    /// As [`InsFilter::update_position`].
    pub fn update_point(
        &mut self,
        point: GeodeticPoint,
        horizontal_sigma: Distance,
        vertical_sigma: Distance,
        gate: GatingPolicy,
    ) -> Result<InsUpdate> {
        ensure_range(
            "position sigma",
            horizontal_sigma.metres(),
            f64::MIN_POSITIVE,
            1e6,
        )?;
        ensure_range(
            "height sigma",
            vertical_sigma.metres(),
            f64::MIN_POSITIVE,
            1e6,
        )?;
        let measured = self.nominal.frame().ned_of(point)?;
        let (estimated, _, _, _) = self.nominal.raw();
        let innovation = [
            measured.north().metres() - estimated[0],
            measured.east().metres() - estimated[1],
            measured.down().metres() - estimated[2],
        ];
        let jacobian = Matrix::<3, ERROR_STATE_DIM>::from_fn(|row, column| {
            f64::from(column == POSITION + row)
        });
        let horizontal = horizontal_sigma.metres() * horizontal_sigma.metres();
        let vertical = vertical_sigma.metres() * vertical_sigma.metres();
        self.update(
            innovation,
            &jacobian,
            &Matrix::diagonal([horizontal, horizontal, vertical]),
            gate,
        )
    }

    /// Ground velocity, NED, same sigma on each component.
    ///
    /// # Errors
    ///
    /// As [`InsFilter::update_position`].
    pub fn update_velocity(
        &mut self,
        velocity: Vector3<Ned, Speed>,
        sigma: Speed,
        gate: GatingPolicy,
    ) -> Result<InsUpdate> {
        ensure_range(
            "velocity sigma",
            sigma.metres_per_second(),
            f64::MIN_POSITIVE,
            1e3,
        )?;
        let (_, estimated, _, _) = self.nominal.raw();
        let innovation = [
            velocity.north().metres_per_second() - estimated[0],
            velocity.east().metres_per_second() - estimated[1],
            velocity.down().metres_per_second() - estimated[2],
        ];
        let jacobian = Matrix::<3, ERROR_STATE_DIM>::from_fn(|row, column| {
            f64::from(column == VELOCITY + row)
        });
        let variance = sigma.metres_per_second() * sigma.metres_per_second();
        self.update(
            innovation,
            &jacobian,
            &Matrix::diagonal([variance, variance, variance]),
            gate,
        )
    }

    /// Zero-velocity update; `sigma` bounds the residual motion.
    ///
    /// # Errors
    ///
    /// As [`InsFilter::update_position`].
    pub fn update_zero_velocity(&mut self, sigma: Speed, gate: GatingPolicy) -> Result<InsUpdate> {
        self.update_velocity(
            Vector3::new(Speed::ZERO, Speed::ZERO, Speed::ZERO),
            sigma,
            gate,
        )
    }

    /// Heading from a gyrocompass or satellite compass.
    ///
    /// The heading error is the misalignment about down, to first order in roll
    /// and pitch; the innovation is wrapped (359° vs 1° → −2°).
    ///
    /// # Errors
    ///
    /// As [`InsFilter::update_position`].
    pub fn update_heading(
        &mut self,
        heading: TrueCourse,
        sigma: Angle,
        gate: GatingPolicy,
    ) -> Result<InsUpdate> {
        ensure_range(
            "heading sigma",
            sigma.radians(),
            f64::MIN_POSITIVE,
            core::f64::consts::PI,
        )?;
        let estimated = self.nominal.attitude().yaw;
        let innovation = [math::to_radians(wrap180(
            heading.degrees() - estimated.degrees(),
        ))];
        // yaw_true = yaw_estimated − ψ_down.
        let jacobian =
            Matrix::<1, ERROR_STATE_DIM>::from_fn(|_, column| -f64::from(column == ATTITUDE + 2));
        let variance = sigma.radians() * sigma.radians();
        self.update(innovation, &jacobian, &Matrix::diagonal([variance]), gate)
    }

    /// Horizontal fix of an antenna at a lever arm from the IMU, in body axes
    /// (forward, right, down).
    ///
    /// The fix is `p + C·l`; its Jacobian carries the attitude through
    /// `[(C·l)×]`, so a mast leaning with the machine corrects the tilt
    /// instead of pulling the position.
    ///
    /// # Errors
    ///
    /// As [`InsFilter::update_position`].
    pub fn update_antenna_position(
        &mut self,
        position: Position,
        lever: Vector3<Body, Distance>,
        sigma: Distance,
        gate: GatingPolicy,
    ) -> Result<InsUpdate> {
        ensure_range("position sigma", sigma.metres(), f64::MIN_POSITIVE, 1e6)?;
        let arm = self.lever_in_navigation(lever);
        let height = self.nominal.point()?.height();
        let measured = self
            .nominal
            .frame()
            .ned_of(GeodeticPoint::new(position, height))?;
        let (estimated, _, _, _) = self.nominal.raw();
        let innovation = [
            measured.north().metres() - (estimated[0] + arm[0]),
            measured.east().metres() - (estimated[1] + arm[1]),
        ];
        let jacobian = antenna_jacobian::<2>(arm);
        let variance = sigma.metres() * sigma.metres();
        self.update(
            innovation,
            &jacobian,
            &Matrix::diagonal([variance, variance]),
            gate,
        )
    }

    /// Fix with height of an antenna at a lever arm from the IMU, in body
    /// axes.
    ///
    /// # Errors
    ///
    /// As [`InsFilter::update_point`].
    pub fn update_antenna_point(
        &mut self,
        point: GeodeticPoint,
        lever: Vector3<Body, Distance>,
        horizontal_sigma: Distance,
        vertical_sigma: Distance,
        gate: GatingPolicy,
    ) -> Result<InsUpdate> {
        ensure_range(
            "position sigma",
            horizontal_sigma.metres(),
            f64::MIN_POSITIVE,
            1e6,
        )?;
        ensure_range(
            "height sigma",
            vertical_sigma.metres(),
            f64::MIN_POSITIVE,
            1e6,
        )?;
        let arm = self.lever_in_navigation(lever);
        let measured = self.nominal.frame().ned_of(point)?;
        let (estimated, _, _, _) = self.nominal.raw();
        let innovation = [
            measured.north().metres() - (estimated[0] + arm[0]),
            measured.east().metres() - (estimated[1] + arm[1]),
            measured.down().metres() - (estimated[2] + arm[2]),
        ];
        let jacobian = antenna_jacobian::<3>(arm);
        let horizontal = horizontal_sigma.metres() * horizontal_sigma.metres();
        let vertical = vertical_sigma.metres() * vertical_sigma.metres();
        self.update(
            innovation,
            &jacobian,
            &Matrix::diagonal([horizontal, horizontal, vertical]),
            gate,
        )
    }

    /// Wheel speed: the forward speed of the body, read by an odometer whose
    /// scale (measured over true speed, 1 for a calibrated wheel) is given.
    ///
    /// The scale is the caller's calibration, not a state of the filter
    /// (ADR-0021): a 1 % error at 2.5 m/s is 2.5 cm/s, 0.75 m over 30 s
    /// without GNSS. The odometer is taken to read at the IMU; mount the IMU
    /// over the rear axle, or the turn adds `ω × l` across the reading.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] for a non-positive sigma or a scale outside
    /// 0.5…2; as [`InsFilter::update_position`] otherwise.
    pub fn update_wheel_speed(
        &mut self,
        speed: Speed,
        scale: f64,
        sigma: Speed,
        gate: GatingPolicy,
    ) -> Result<InsUpdate> {
        ensure_range(
            "wheel speed sigma",
            sigma.metres_per_second(),
            f64::MIN_POSITIVE,
            1e3,
        )?;
        ensure_range("odometer scale", scale, 0.5, 2.0)?;
        let (rows, body) = self.body_velocity_rows();
        let [forward, _, _] = rows;
        let innovation = [speed.metres_per_second() / scale - body[0]];
        let jacobian = Matrix::<1, ERROR_STATE_DIM>::from_fn(|_, column| {
            forward.get(column).copied().unwrap_or(0.0)
        });
        let variance = square(sigma.metres_per_second() / scale);
        self.update(innovation, &jacobian, &Matrix::diagonal([variance]), gate)
    }

    /// Non-holonomic constraint: no sideways and no vertical velocity of the
    /// body.
    ///
    /// Holds only while the wheels are on the ground and do not slide, for an
    /// IMU on the axle the machine turns about (the rear axle of a tractor):
    /// the caller decides when that is so. `sigma_lateral` and
    /// `sigma_vertical` bound the residual motion: skidding, suspension,
    /// a lever arm left out.
    ///
    /// # Errors
    ///
    /// As [`InsFilter::update_position`].
    pub fn update_non_holonomic(
        &mut self,
        sigma_lateral: Speed,
        sigma_vertical: Speed,
        gate: GatingPolicy,
    ) -> Result<InsUpdate> {
        for (name, sigma) in [
            ("lateral sigma", sigma_lateral),
            ("vertical sigma", sigma_vertical),
        ] {
            ensure_range(name, sigma.metres_per_second(), f64::MIN_POSITIVE, 1e3)?;
        }
        let (rows, body) = self.body_velocity_rows();
        let [_, right, down] = rows;
        let innovation = [-body[1], -body[2]];
        let jacobian = Matrix::<2, ERROR_STATE_DIM>::from_fn(|row, column| {
            let source = if row == 0 { &right } else { &down };
            source.get(column).copied().unwrap_or(0.0)
        });
        self.update(
            innovation,
            &jacobian,
            &Matrix::diagonal([
                square(sigma_lateral.metres_per_second()),
                square(sigma_vertical.metres_per_second()),
            ]),
            gate,
        )
    }

    /// Heading of a two-antenna baseline, in body axes from the first antenna
    /// to the second, with its measured length checked first.
    ///
    /// A baseline whose measured length differs from `baseline`'s by more
    /// than `tolerance` is refused (`accepted: false`, `nis` infinite): a
    /// wrongly resolved ambiguity shows in the length before it shows in the
    /// heading. The heading is that of `C·b`, so a baseline across the
    /// machine or tilted with it is handled exactly.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] for a baseline shorter than 10 cm in
    /// plan, a negative tolerance or a non-positive sigma.
    pub fn update_baseline_heading(
        &mut self,
        heading: TrueCourse,
        measured_length: Distance,
        baseline: Vector3<Body, Distance>,
        tolerance: Distance,
        sigma: Angle,
        gate: GatingPolicy,
    ) -> Result<InsUpdate> {
        ensure_range(
            "heading sigma",
            sigma.radians(),
            f64::MIN_POSITIVE,
            core::f64::consts::PI,
        )?;
        ensure_range("baseline tolerance", tolerance.metres(), 0.0, 1e3)?;
        let body = [
            baseline.forward().metres(),
            baseline.right().metres(),
            baseline.down().metres(),
        ];
        let length = math::sqrt(body.iter().map(|v| v * v).sum());
        if math::abs(measured_length.metres() - length) > tolerance.metres() {
            return Ok(InsUpdate {
                nis: f64::INFINITY,
                dof: 1,
                accepted: false,
            });
        }
        let direction = self.nominal.quaternion().rotate(body);
        let [north, east, _] = direction;
        let plan = north * north + east * east;
        ensure_range(
            "baseline length in plan, metres",
            math::sqrt(plan),
            0.1,
            1e3,
        )?;
        let estimated = math::to_degrees(math::atan2(east, north));
        let innovation = [math::to_radians(wrap180(heading.degrees() - estimated))];
        let jacobian = baseline_jacobian(direction);
        let variance = sigma.radians() * sigma.radians();
        self.update(innovation, &jacobian, &Matrix::diagonal([variance]), gate)
    }

    /// Lever arm in navigation axes, `C·l`.
    fn lever_in_navigation(&self, lever: Vector3<Body, Distance>) -> [f64; 3] {
        self.nominal.quaternion().rotate([
            lever.forward().metres(),
            lever.right().metres(),
            lever.down().metres(),
        ])
    }

    /// Body velocity `Cᵀ v` and the Jacobian row of each component over the
    /// error state: `Cᵀ` on the velocity error, `−Cᵀ[v×]` on the
    /// misalignment.
    fn body_velocity_rows(&self) -> ([[f64; ERROR_STATE_DIM]; 3], [f64; 3]) {
        let (_, velocity, _, _) = self.nominal.raw();
        let rotation = self.nominal.quaternion().to_matrix();
        let body = self.nominal.quaternion().rotate_back(velocity);
        let transposed = rotation.transpose();
        let attitude = transposed * skew(velocity).scaled(-1.0);
        let mut rows = [[0.0; ERROR_STATE_DIM]; 3];
        for (axis, row) in rows.iter_mut().enumerate() {
            for column in 0..3 {
                if let Some(slot) = row.get_mut(VELOCITY + column) {
                    *slot = transposed.get(axis, column).unwrap_or(0.0);
                }
                if let Some(slot) = row.get_mut(ATTITUDE + column) {
                    *slot = attitude.get(axis, column).unwrap_or(0.0);
                }
            }
        }
        (rows, body)
    }

    /// Corrected mechanisation state.
    #[must_use]
    pub const fn nominal(&self) -> &Strapdown {
        &self.nominal
    }

    /// Time of the estimate.
    #[must_use]
    pub const fn valid_at(&self) -> Instant<Utc> {
        self.nominal.valid_at()
    }

    /// Position.
    ///
    /// # Errors
    ///
    /// As [`Strapdown::position`].
    pub fn position(&self) -> Result<Position> {
        self.nominal.position()
    }

    /// Ground velocity.
    #[must_use]
    pub fn velocity(&self) -> Vector3<Ned, Speed> {
        self.nominal.velocity()
    }

    /// Roll, pitch and yaw.
    #[must_use]
    pub fn attitude(&self) -> Attitude {
        self.nominal.attitude()
    }

    /// Attitude quaternion.
    #[must_use]
    pub const fn quaternion(&self) -> &Quaternion {
        self.nominal.quaternion()
    }

    /// 1σ horizontal position error ellipse.
    #[must_use]
    pub fn horizontal_error(&self) -> ErrorEllipse {
        ErrorEllipse::from_covariance(
            self.variance(POSITION),
            self.covariance.get(POSITION, POSITION + 1).unwrap_or(0.0),
            self.variance(POSITION + 1),
        )
        .unwrap_or_else(|| ErrorEllipse::circular(Distance::ZERO))
    }

    /// 1σ height error.
    #[must_use]
    pub fn height_sigma(&self) -> Distance {
        Distance::from_metres(math::sqrt(self.variance(POSITION + 2))).unwrap_or(Distance::ZERO)
    }

    /// 1σ horizontal velocity error, larger component.
    #[must_use]
    pub fn velocity_sigma(&self) -> Speed {
        let largest = self.variance(VELOCITY).max(self.variance(VELOCITY + 1));
        Speed::from_metres_per_second(math::sqrt(largest)).unwrap_or(Speed::ZERO)
    }

    /// 1σ heading error.
    #[must_use]
    pub fn heading_sigma(&self) -> Angle {
        Angle::from_radians(math::sqrt(self.variance(ATTITUDE + 2))).unwrap_or(Angle::ZERO)
    }

    /// 1σ tilt error, larger of roll and pitch.
    #[must_use]
    pub fn tilt_sigma(&self) -> Angle {
        let largest = self.variance(ATTITUDE).max(self.variance(ATTITUDE + 1));
        Angle::from_radians(math::sqrt(largest)).unwrap_or(Angle::ZERO)
    }

    fn variance(&self, index: usize) -> f64 {
        self.covariance.get(index, index).unwrap_or(0.0)
    }

    /// Error-state covariance. Hidden, not covered by the stability guarantee;
    /// used by the consistency tests.
    #[doc(hidden)]
    #[must_use]
    pub const fn covariance(&self) -> &Matrix<ERROR_STATE_DIM, ERROR_STATE_DIM> {
        &self.covariance
    }
}

/// Jacobian of an antenna fix `p + C·l` over the error state, first `M`
/// navigation axes: identity on position, `[(C·l)×]` on the misalignment
/// (`C_true l = (I − [ψ×]) C l = C l + [(C l)×] ψ`).
fn antenna_jacobian<const M: usize>(arm: [f64; 3]) -> Matrix<M, ERROR_STATE_DIM> {
    let cross = skew(arm);
    Matrix::<M, ERROR_STATE_DIM>::from_fn(|row, column| {
        if column == POSITION + row {
            1.0
        } else if (ATTITUDE..ATTITUDE + 3).contains(&column) {
            cross.get(row, column - ATTITUDE).unwrap_or(0.0)
        } else {
            0.0
        }
    })
}

/// Jacobian of the heading of a baseline `d = C·b` over the error state:
/// `δd = [d×] ψ`, `δα = (d_n δd_e − d_e δd_n) / (d_n² + d_e²)`.
fn baseline_jacobian(direction: [f64; 3]) -> Matrix<1, ERROR_STATE_DIM> {
    let [north, east, _] = direction;
    let plan = north * north + east * east;
    let cross = skew(direction);
    Matrix::<1, ERROR_STATE_DIM>::from_fn(|_, column| {
        if !(ATTITUDE..ATTITUDE + 3).contains(&column) {
            return 0.0;
        }
        let axis = column - ATTITUDE;
        let row_north = cross.get(0, axis).unwrap_or(0.0);
        let row_east = cross.get(1, axis).unwrap_or(0.0);
        (north * row_east - east * row_north) / plan
    })
}

fn square(value: f64) -> f64 {
    value * value
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::indexing_slicing)]
mod tests {
    use super::*;
    use kinavis_kernel::geodesy::Height;
    use kinavis_kernel::position::Position;

    /// A filter rolled, pitched and turned, moving.
    fn filter() -> InsFilter {
        let nominal = Strapdown::new(
            Instant::<Utc>::from_unix_seconds(1_789_000_000),
            GeodeticPoint::new(
                Position::from_degrees(48.0, 11.0).unwrap(),
                Height::above_ellipsoid(Distance::from_metres(500.0).unwrap()),
            ),
            Vector3::<Ned, Speed>::new(
                Speed::from_metres_per_second(1.7).unwrap(),
                Speed::from_metres_per_second(-2.1).unwrap(),
                Speed::from_metres_per_second(0.2).unwrap(),
            ),
            Quaternion::from_euler(
                Angle::from_degrees(8.0).unwrap(),
                Angle::from_degrees(-3.0).unwrap(),
                TrueCourse::new(305.0).unwrap(),
            ),
        )
        .unwrap();
        InsFilter::new(nominal, ImuNoise::mems(), &InsPriors::standard())
    }

    /// The filter with an error-state correction applied.
    fn moved(filter: &InsFilter, column: usize, step: f64) -> InsFilter {
        let mut moved = filter.clone();
        let mut blocks = [[0.0; 3]; 5];
        blocks[column / 3][column % 3] = step;
        moved
            .nominal
            .correct(blocks[0], blocks[1], blocks[2], blocks[3], blocks[4]);
        moved
    }

    /// Central differences of `h` against the analytic Jacobian rows.
    fn check<const M: usize>(
        h: impl Fn(&InsFilter) -> [f64; M],
        jacobian: &Matrix<M, ERROR_STATE_DIM>,
    ) {
        let base = filter();
        for column in 0..ERROR_STATE_DIM {
            let step = 1e-6;
            let plus = h(&moved(&base, column, step));
            let minus = h(&moved(&base, column, -step));
            for row in 0..M {
                let numeric = (plus[row] - minus[row]) / (2.0 * step);
                let analytic = jacobian.get(row, column).unwrap();
                assert!(
                    (numeric - analytic).abs() < 1e-6 * (1.0 + analytic.abs()),
                    "∂{row}/∂{column}: {analytic} against {numeric}"
                );
            }
        }
    }

    const LEVER: [f64; 3] = [0.6, -0.3, -3.1];

    #[test]
    fn the_antenna_jacobian_is_the_derivative_of_the_antenna_position() {
        let base = filter();
        let arm = base.nominal.quaternion().rotate(LEVER);
        let h = |f: &InsFilter| {
            let (p, _, _, _) = f.nominal.raw();
            let arm = f.nominal.quaternion().rotate(LEVER);
            [p[0] + arm[0], p[1] + arm[1], p[2] + arm[2]]
        };
        check(h, &antenna_jacobian::<3>(arm));
    }

    #[test]
    fn the_body_velocity_rows_are_the_derivative_of_the_body_velocity() {
        let base = filter();
        let (rows, _) = base.body_velocity_rows();
        let jacobian = Matrix::<3, ERROR_STATE_DIM>::from_fn(|row, column| rows[row][column]);
        let h = |f: &InsFilter| f.body_velocity_rows().1;
        check(h, &jacobian);
    }

    #[test]
    fn the_baseline_jacobian_is_the_derivative_of_its_heading() {
        let baseline = [0.3, 1.4, 0.05];
        let base = filter();
        let jacobian = baseline_jacobian(base.nominal.quaternion().rotate(baseline));
        let h = |f: &InsFilter| {
            let [n, e, _] = f.nominal.quaternion().rotate(baseline);
            [math::atan2(e, n)]
        };
        check(h, &jacobian);
    }

    #[test]
    fn a_baseline_of_the_wrong_length_is_refused() {
        let mut filter = filter();
        let baseline = Vector3::<Body, Distance>::new(
            Distance::from_metres(0.0).unwrap(),
            Distance::from_metres(1.5).unwrap(),
            Distance::ZERO,
        );
        let update = filter
            .update_baseline_heading(
                TrueCourse::new(35.0).unwrap(),
                Distance::from_metres(1.62).unwrap(),
                baseline,
                Distance::from_metres(0.05).unwrap(),
                Angle::from_degrees(0.3).unwrap(),
                GatingPolicy::none(),
            )
            .unwrap();
        assert!(!update.accepted);
    }
}
