//! Strapdown mechanisation: IMU samples integrated into attitude, velocity and
//! position in the local navigation frame.
//!
//! Gyros measure rotation relative to inertial space; Earth rate and transport
//! rate are removed before updating the attitude. Accelerometers measure
//! specific force (acceleration minus gravity); gravity is restored from a
//! model, Coriolis is removed, and the result is integrated into velocity and
//! position. Standard NED formulation (Titterton & Weston, Groves).
//!
//! Unaided, the mechanisation drifts: 1°/h gyro bias is 1°/h of heading error;
//! 1 mg of accelerometer bias is about 1 km in 20 minutes. The [`InsFilter`]
//! estimates the biases and corrects this state; the mechanisation runs between
//! corrections.
//!
//! [`InsFilter`]: crate::InsFilter

use kinavis_kernel::error::{KernelError, Result};
use kinavis_kernel::geodesy::{Ellipsoid, GeodeticPoint};
use kinavis_kernel::local::{LocalFrame, Ned, Vector3};
use kinavis_kernel::math;
use kinavis_kernel::position::Position;
use kinavis_kernel::time::{Instant, Utc};
use kinavis_kernel::units::{Distance, Speed};

use crate::attitude::{cross, Attitude, Quaternion};
use crate::imu::ImuSample;

/// Earth rotation rate, rad/s (WGS 84).
pub const EARTH_RATE: f64 = 7.292_115e-5;

/// Nominal INS state: position, velocity, attitude and estimated sensor biases.
///
/// Position is a NED displacement from a [`LocalFrame`] fixed at
/// initialisation, so arithmetic is in metres; the geodetic position is
/// computed on request.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Strapdown {
    frame: LocalFrame,
    /// m, NED from the frame origin.
    position: [f64; 3],
    /// m/s, NED.
    velocity: [f64; 3],
    attitude: Quaternion,
    /// rad/s, body axes.
    gyro_bias: [f64; 3],
    /// m/s², body axes.
    accel_bias: [f64; 3],
    valid_at: Instant<Utc>,
}

impl Strapdown {
    /// System initialised at a point, stationary or moving, with known attitude
    /// (alignment, gyrocompass and level, or a previous run) and zero biases.
    ///
    /// # Errors
    ///
    /// As [`LocalFrame::at`]: the point must be above the ellipsoid.
    pub fn new(
        at: Instant<Utc>,
        origin: GeodeticPoint,
        velocity: Vector3<Ned, Speed>,
        attitude: Quaternion,
    ) -> Result<Self> {
        let frame = LocalFrame::at(origin, &Ellipsoid::WGS84)?;
        Ok(Self {
            frame,
            position: [0.0; 3],
            velocity: [
                velocity.north().metres_per_second(),
                velocity.east().metres_per_second(),
                velocity.down().metres_per_second(),
            ],
            attitude,
            gyro_bias: [0.0; 3],
            accel_bias: [0.0; 3],
            valid_at: at,
        })
    }

    /// Integrates one IMU sample to the end of its interval.
    ///
    /// # Errors
    ///
    /// [`KernelError::Indeterminate`] if the time would overflow [`Instant`].
    pub fn step(&mut self, sample: &ImuSample) -> Result<()> {
        let dt = sample.seconds();
        let (latitude, height) = self.latitude_and_height();
        let (sin_lat, cos_lat) = (math::sin(latitude), math::cos(latitude));
        let (radius_north, radius_east) = curvature_radii(sin_lat);

        // Earth rate and transport rate as seen by the gyros.
        let earth_rate = [EARTH_RATE * cos_lat, 0.0, -EARTH_RATE * sin_lat];
        let [vn, ve, _] = self.velocity;
        let transport_rate = [
            ve / (radius_east + height),
            -vn / (radius_north + height),
            -ve * sin_lat / (cos_lat.max(1e-9) * (radius_east + height)),
        ];
        let frame_rate = add(earth_rate, transport_rate);

        // Body rotation relative to the navigation frame.
        let measured = sample.angular_rate();
        let corrected = sub(measured, self.gyro_bias);
        let body_rate = sub(corrected, self.attitude.rotate_back(frame_rate));
        let rotation = scale(body_rate, dt);
        let midway = self.attitude.rotated_by_body(scale(body_rate, dt / 2.0));
        let attitude = self.attitude.rotated_by_body(rotation);

        // Specific force in the navigation frame at mid-step, plus gravity,
        // minus Coriolis.
        let force = midway.rotate(sub(sample.specific_force(), self.accel_bias));
        let gravity = [0.0, 0.0, gravity_down(sin_lat, height)];
        let coriolis = cross(add(scale(earth_rate, 2.0), transport_rate), self.velocity);
        let acceleration = sub(add(force, gravity), coriolis);
        let velocity = add(self.velocity, scale(acceleration, dt));
        let position = add(self.position, scale(add(self.velocity, velocity), dt / 2.0));

        self.valid_at =
            self.valid_at
                .checked_add(sample.over())
                .ok_or(KernelError::Indeterminate {
                    quantity: "a moment beyond the end of time",
                })?;
        self.attitude = attitude;
        self.velocity = velocity;
        self.position = position;
        Ok(())
    }

    /// Latitude (rad) and ellipsoidal height (m) of the current position, from
    /// the origin and the displacement; accurate enough for the Earth-rate and
    /// gravity terms within the frame's range.
    fn latitude_and_height(&self) -> (f64, f64) {
        let origin = self.frame.origin();
        let latitude = origin.position().latitude().radians();
        let height = origin.height().value().metres() - self.position[2];
        let (radius_north, _) = curvature_radii(math::sin(latitude));
        (
            latitude + self.position[0] / (radius_north + height),
            height,
        )
    }

    /// Time of the state.
    #[must_use]
    pub const fn valid_at(&self) -> Instant<Utc> {
        self.valid_at
    }

    /// Frame of the displacement.
    #[must_use]
    pub const fn frame(&self) -> &LocalFrame {
        &self.frame
    }

    /// NED displacement from the frame origin.
    #[must_use]
    pub fn displacement(&self) -> Vector3<Ned, Distance> {
        let [n, e, d] = self.position;
        Vector3::new(metres(n), metres(e), metres(d))
    }

    /// Position with height above the ellipsoid.
    ///
    /// # Errors
    ///
    /// As [`LocalFrame::point_from_ned`].
    pub fn point(&self) -> Result<GeodeticPoint> {
        self.frame.point_from_ned(self.displacement())
    }

    /// Position.
    ///
    /// # Errors
    ///
    /// As [`Strapdown::point`].
    pub fn position(&self) -> Result<Position> {
        Ok(self.point()?.position())
    }

    /// Ground velocity, NED.
    #[must_use]
    pub fn velocity(&self) -> Vector3<Ned, Speed> {
        let [n, e, d] = self.velocity;
        Vector3::new(
            metres_per_second(n),
            metres_per_second(e),
            metres_per_second(d),
        )
    }

    /// Roll, pitch and yaw.
    #[must_use]
    pub fn attitude(&self) -> Attitude {
        self.attitude.to_euler()
    }

    /// Attitude quaternion, body to navigation.
    #[must_use]
    pub const fn quaternion(&self) -> &Quaternion {
        &self.attitude
    }

    /// Estimated gyro biases, rad/s, body axes.
    #[must_use]
    pub const fn gyro_bias(&self) -> [f64; 3] {
        self.gyro_bias
    }

    /// Estimated accelerometer biases, m/s², body axes.
    #[must_use]
    pub const fn accel_bias(&self) -> [f64; 3] {
        self.accel_bias
    }

    /// Position, velocity and biases as raw arrays, for the filter.
    pub(crate) const fn raw(&self) -> ([f64; 3], [f64; 3], [f64; 3], [f64; 3]) {
        (
            self.position,
            self.velocity,
            self.gyro_bias,
            self.accel_bias,
        )
    }

    /// Navigation-frame specific force at the current attitude, for the filter
    /// Jacobian.
    pub(crate) fn navigation_force(&self, sample: &ImuSample) -> [f64; 3] {
        self.attitude
            .rotate(sub(sample.specific_force(), self.accel_bias))
    }

    /// Earth rate in the navigation frame at the current latitude.
    pub(crate) fn earth_rate(&self) -> [f64; 3] {
        let (latitude, _) = self.latitude_and_height();
        [
            EARTH_RATE * math::cos(latitude),
            0.0,
            -EARTH_RATE * math::sin(latitude),
        ]
    }

    /// Applies an error-state correction: errors are truth minus state and are
    /// added; the attitude is rotated by the misalignment.
    pub(crate) fn correct(
        &mut self,
        position: [f64; 3],
        velocity: [f64; 3],
        misalignment: [f64; 3],
        gyro_bias: [f64; 3],
        accel_bias: [f64; 3],
    ) {
        self.position = add(self.position, position);
        self.velocity = add(self.velocity, velocity);
        self.attitude = self.attitude.corrected_by(misalignment);
        self.gyro_bias = add(self.gyro_bias, gyro_bias);
        self.accel_bias = add(self.accel_bias, accel_bias);
    }
}

/// WGS 84 meridional and prime-vertical radii of curvature at a latitude, m.
fn curvature_radii(sin_lat: f64) -> (f64, f64) {
    let a = Ellipsoid::WGS84.semi_major_axis().metres();
    let e2 = Ellipsoid::WGS84.first_eccentricity_squared();
    let denominator = 1.0 - e2 * sin_lat * sin_lat;
    let east = a / math::sqrt(denominator);
    let north = east * (1.0 - e2) / denominator;
    (north, east)
}

/// Gravity at latitude and height, m/s², positive down: Somigliana on WGS 84
/// with free-air correction.
#[must_use]
pub fn gravity_down(sin_lat: f64, height: f64) -> f64 {
    let sin2 = sin_lat * sin_lat;
    let surface = 9.780_325_335_9 * (1.0 + 0.001_931_852_652_41 * sin2)
        / math::sqrt(1.0 - 0.006_694_379_990_14 * sin2);
    surface - 3.086e-6 * height
}

fn add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn scale(a: [f64; 3], by: f64) -> [f64; 3] {
    [a[0] * by, a[1] * by, a[2] * by]
}

fn metres(value: f64) -> Distance {
    Distance::from_metres(value).unwrap_or(Distance::ZERO)
}

fn metres_per_second(value: f64) -> Speed {
    Speed::from_metres_per_second(value).unwrap_or(Speed::ZERO)
}
