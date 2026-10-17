//! KINAVIS (NED, FRD) to ROS (ENU, FLU) and back (ADR-0020).
//!
//! Vectors: NED → ENU `(e, n, u) = (n_e, n_n, −n_d)`, FRD → FLU
//! `(x, −y, −z)`. Orientation: `q_ros = q(NED→ENU) ⊗ q(FRD→NED) ⊗
//! q(FLU→FRD)`; in angles roll is kept, pitch changes sign and yaw is
//! `90° − heading`. Covariances only from published quantities: an error
//! ellipse gives the horizontal block, a height sigma the vertical.

use kinavis_kernel::error::{KernelError, Result};
use kinavis_kernel::geodesy::VerticalDatum;
use kinavis_kernel::{
    math, Angle, Distance, ErrorEllipse, FixType, GeodeticPoint, GnssFix, Height, Instant,
    Latitude, Longitude, Ned, Position, TrueCourse, Utc, Vector3, VectorUnit,
};

use crate::messages::{
    covariance_type, nav_sat_status, GeoPoint, Imu, NavSatFix, Odometry, Quaternion, Stamp, Vector,
};

/// ECEF frame, REP-105.
pub const EARTH: &str = "earth";
/// World-fixed ENU frame of `Odometry` poses.
pub const MAP: &str = "map";
/// Continuous odometry frame.
pub const ODOM: &str = "odom";
/// Body frame, FLU.
pub const BASE_LINK: &str = "base_link";

/// Variance written where a quantity is not known and the message has no
/// "unknown" marker, m² or rad²: large enough that a filter ignores it.
pub const UNKNOWN_VARIANCE: f64 = 1e9;

/// ROS time of an instant.
///
/// # Errors
///
/// [`KernelError::OutOfRange`] beyond the `i32` seconds of ROS time.
pub fn stamp(at: Instant<Utc>) -> Result<Stamp> {
    // Beyond ±2³¹ s only the sign of the value matters to the message.
    let sec = i32::try_from(at.seconds()).map_err(|_| KernelError::OutOfRange {
        parameter: "ROS time, seconds",
        value: if at.seconds() < 0 { f64::MIN } else { f64::MAX },
        min: f64::from(i32::MIN),
        max: f64::from(i32::MAX),
    })?;
    Ok(Stamp {
        sec,
        nanosec: at.subsec_nanos(),
    })
}

/// Instant of a ROS time.
#[must_use]
pub fn instant(stamp: Stamp) -> Instant<Utc> {
    Instant::<Utc>::from_unix_seconds(i64::from(stamp.sec)).saturating_add(
        core::time::Duration::from_nanos(u64::from(stamp.nanosec.min(999_999_999))),
    )
}

/// NED vector to ENU components.
#[must_use]
pub fn enu<U: VectorUnit>(vector: Vector3<Ned, U>) -> Vector {
    Vector {
        x: vector.east().si(),
        y: vector.north().si(),
        z: -vector.down().si(),
    }
}

/// FRD components to FLU.
#[must_use]
pub const fn flu(frd: [f64; 3]) -> Vector {
    Vector {
        x: frd[0],
        y: -frd[1],
        z: -frd[2],
    }
}

/// FLU components to FRD.
#[must_use]
pub const fn frd(flu: Vector) -> [f64; 3] {
    [flu.x, -flu.y, -flu.z]
}

/// Hamilton product.
const fn product(a: Quaternion, b: Quaternion) -> Quaternion {
    Quaternion {
        w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    }
}

/// NED to ENU: half a turn about (1, 1, 0)/√2.
const NED_TO_ENU: Quaternion = Quaternion {
    w: 0.0,
    x: core::f64::consts::FRAC_1_SQRT_2,
    y: core::f64::consts::FRAC_1_SQRT_2,
    z: 0.0,
};

/// FLU to FRD: half a turn about x.
const FLU_TO_FRD: Quaternion = Quaternion {
    w: 0.0,
    x: 1.0,
    y: 0.0,
    z: 0.0,
};

const fn conjugate(q: Quaternion) -> Quaternion {
    Quaternion {
        w: q.w,
        x: -q.x,
        y: -q.y,
        z: -q.z,
    }
}

/// Orientation of FLU in ENU from roll, pitch and true heading of FRD in NED.
#[must_use]
pub fn orientation(roll: Angle, pitch: Angle, heading: TrueCourse) -> Quaternion {
    let half = |angle: f64| (math::sin(angle / 2.0), math::cos(angle / 2.0));
    let (sr, cr) = half(roll.radians());
    let (sp, cp) = half(pitch.radians());
    let (sy, cy) = half(heading.radians());
    let body_to_ned = Quaternion {
        w: cr * cp * cy + sr * sp * sy,
        x: sr * cp * cy - cr * sp * sy,
        y: cr * sp * cy + sr * cp * sy,
        z: cr * cp * sy - sr * sp * cy,
    };
    normalised(product(product(NED_TO_ENU, body_to_ned), FLU_TO_FRD))
}

/// Roll, pitch and true heading of an orientation of FLU in ENU.
///
/// # Errors
///
/// [`KernelError::Indeterminate`] for a quaternion of zero length.
pub fn attitude(orientation: Quaternion) -> Result<(Angle, Angle, TrueCourse)> {
    let length = math::sqrt(
        orientation.w * orientation.w
            + orientation.x * orientation.x
            + orientation.y * orientation.y
            + orientation.z * orientation.z,
    );
    if length.is_nan() || length <= 1e-12 || length.is_infinite() {
        return Err(KernelError::Indeterminate {
            quantity: "an orientation of zero length",
        });
    }
    let q = normalised(product(
        product(conjugate(NED_TO_ENU), orientation),
        conjugate(FLU_TO_FRD),
    ));
    let roll = math::atan2(
        2.0 * (q.w * q.x + q.y * q.z),
        1.0 - 2.0 * (q.x * q.x + q.y * q.y),
    );
    let pitch = math::asin((2.0 * (q.w * q.y - q.z * q.x)).clamp(-1.0, 1.0));
    let yaw = math::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    );
    Ok((
        Angle::from_radians(roll)?,
        Angle::from_radians(pitch)?,
        TrueCourse::wrap(math::to_degrees(yaw))?,
    ))
}

fn normalised(q: Quaternion) -> Quaternion {
    let length = math::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if length.is_nan() || length <= 0.0 {
        return Quaternion {
            w: 1.0,
            ..Quaternion::default()
        };
    }
    // One sign for one rotation: w non-negative.
    let sign = if q.w < 0.0 { -1.0 } else { 1.0 } / length;
    Quaternion {
        w: q.w * sign,
        x: q.x * sign,
        y: q.y * sign,
        z: q.z * sign,
    }
}

/// East, north and east-north variances of a 1σ error ellipse, m².
fn horizontal(ellipse: ErrorEllipse) -> (f64, f64, f64) {
    let (a, b) = (ellipse.semi_major().metres(), ellipse.semi_minor().metres());
    let (s, c) = (
        math::sin(ellipse.orientation().radians()),
        math::cos(ellipse.orientation().radians()),
    );
    let (a2, b2) = (a * a, b * b);
    (
        a2 * s * s + b2 * c * c,
        a2 * c * c + b2 * s * s,
        (a2 - b2) * s * c,
    )
}

fn ellipsoidal(height: Height) -> Result<f64> {
    if height.datum() != VerticalDatum::Ellipsoid {
        return Err(KernelError::VerticalDatumMismatch {
            required: VerticalDatum::Ellipsoid,
            found: height.datum(),
        });
    }
    Ok(height.value().metres())
}

/// `NavSatFix` of a GNSS fix at a height above the ellipsoid.
///
/// Status: no fix, a fix, SBAS for differential and precise point
/// positioning, GBAS for RTK; service GPS (the constellations of the fix
/// are not in the kernel's `GnssFix`). Covariance: the fix's error ellipse
/// as the horizontal block and `height_sigma` (or [`UNKNOWN_VARIANCE`]) as
/// the vertical, `APPROXIMATED`; `UNKNOWN` without an ellipse.
///
/// # Errors
///
/// [`KernelError::VerticalDatumMismatch`] for a height not above the
/// ellipsoid: convert one above sea level with a geoid model; as [`stamp`].
pub fn nav_sat_fix(
    fix: &GnssFix,
    height: Height,
    height_sigma: Option<Distance>,
) -> Result<NavSatFix> {
    let altitude = ellipsoidal(height)?;
    let status = match fix.quality().fix_type() {
        FixType::None => nav_sat_status::STATUS_NO_FIX,
        FixType::Differential | FixType::Precise => nav_sat_status::STATUS_SBAS_FIX,
        FixType::RtkFixed | FixType::RtkFloat => nav_sat_status::STATUS_GBAS_FIX,
        _ => nav_sat_status::STATUS_FIX,
    };
    let mut covariance = [0.0; 9];
    let mut kind = covariance_type::UNKNOWN;
    if let Some(ellipse) = fix.horizontal_error() {
        let (east, north, cross) = horizontal(ellipse);
        let vertical =
            height_sigma.map_or(UNKNOWN_VARIANCE, |sigma| sigma.metres() * sigma.metres());
        covariance = [east, cross, 0.0, cross, north, 0.0, 0.0, 0.0, vertical];
        kind = covariance_type::APPROXIMATED;
    }
    Ok(NavSatFix {
        stamp: stamp(fix.taken_at())?,
        status,
        service: nav_sat_status::SERVICE_GPS,
        latitude: fix.position().latitude().degrees(),
        longitude: fix.position().longitude().degrees(),
        altitude,
        position_covariance: covariance,
        position_covariance_type: kind,
    })
}

/// Point above the ellipsoid and error ellipse of a `NavSatFix`; `None` for
/// the ellipse when its covariance is unknown or not one.
///
/// # Errors
///
/// [`KernelError::OutOfRange`] for a position off the Earth.
pub fn from_nav_sat_fix(fix: &NavSatFix) -> Result<(GeodeticPoint, Option<ErrorEllipse>)> {
    let point = GeodeticPoint::new(
        Position::new(
            Latitude::from_degrees(fix.latitude)?,
            Longitude::from_degrees(fix.longitude)?,
        ),
        Height::above_ellipsoid(Distance::from_metres(fix.altitude)?),
    );
    let [east, cross, _, _, north, ..] = fix.position_covariance;
    let ellipse = (fix.position_covariance_type != covariance_type::UNKNOWN)
        .then(|| ErrorEllipse::from_covariance(north, cross, east))
        .flatten();
    Ok((point, ellipse))
}

/// `GeoPoint` of a point above the ellipsoid.
///
/// # Errors
///
/// As [`nav_sat_fix`] for the height.
pub fn geo_point(point: GeodeticPoint) -> Result<GeoPoint> {
    Ok(GeoPoint {
        latitude: point.position().latitude().degrees(),
        longitude: point.position().longitude().degrees(),
        altitude: ellipsoidal(point.height())?,
    })
}

/// What an `Imu` carries, in the kernel's frames.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuReading {
    /// Roll, pitch and true heading, if known.
    pub attitude: Option<(Angle, Angle, TrueCourse)>,
    /// Their sigmas, if known.
    pub attitude_sigma: Option<[Angle; 3]>,
    /// Angular rate, FRD, rad/s, if known.
    pub angular_rate: Option<[f64; 3]>,
    /// Specific force, FRD, m/s², if known.
    pub acceleration: Option<[f64; 3]>,
}

/// `Imu` of a reading at an instant: orientation FLU in ENU, rates and
/// acceleration in FLU; the first covariance element −1 for what is not
/// known, as `sensor_msgs/Imu` requires.
///
/// # Errors
///
/// As [`stamp`].
pub fn imu(at: Instant<Utc>, reading: &ImuReading) -> Result<Imu> {
    let unknown = {
        let mut c = [0.0; 9];
        c[0] = -1.0;
        c
    };
    let (orientation, orientation_covariance) = match reading.attitude {
        Some((roll, pitch, heading)) => {
            let covariance = reading.attitude_sigma.map_or(unknown, |[r, p, y]| {
                let s = |a: Angle| a.radians() * a.radians();
                [s(r), 0.0, 0.0, 0.0, s(p), 0.0, 0.0, 0.0, s(y)]
            });
            (orientation(roll, pitch, heading), covariance)
        }
        None => (
            Quaternion {
                w: 1.0,
                ..Quaternion::default()
            },
            unknown,
        ),
    };
    Ok(Imu {
        stamp: stamp(at)?,
        orientation,
        orientation_covariance,
        angular_velocity: reading.angular_rate.map_or_else(Vector::default, flu),
        angular_velocity_covariance: if reading.angular_rate.is_some() {
            [0.0; 9]
        } else {
            unknown
        },
        linear_acceleration: reading.acceleration.map_or_else(Vector::default, flu),
        linear_acceleration_covariance: if reading.acceleration.is_some() {
            [0.0; 9]
        } else {
            unknown
        },
    })
}

/// What an `Odometry` carries, in the kernel's frames.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OdometryReading {
    /// Position from the origin of `map` (an application's `LocalFrame`).
    pub displacement: Vector3<Ned, Distance>,
    /// Roll, pitch and true heading.
    pub attitude: (Angle, Angle, TrueCourse),
    /// Velocity of the body, FRD, m/s.
    pub velocity: [f64; 3],
    /// Angular rate of the body, FRD, rad/s.
    pub angular_rate: [f64; 3],
    /// Horizontal 1σ ellipse, if known.
    pub horizontal_error: Option<ErrorEllipse>,
    /// Height sigma, if known.
    pub height_sigma: Option<Distance>,
    /// Heading sigma, if known.
    pub heading_sigma: Option<Angle>,
}

/// `Odometry` of a reading: pose of `base_link` in `map`, twist in
/// `base_link`; covariances from the ellipse and sigmas given,
/// [`UNKNOWN_VARIANCE`] on the diagonal for the rest.
///
/// # Errors
///
/// As [`stamp`].
pub fn odometry(at: Instant<Utc>, reading: &OdometryReading) -> Result<Odometry> {
    let (roll, pitch, heading) = reading.attitude;
    let mut pose = [0.0; 36];
    for index in 0..6 {
        if let Some(slot) = pose.get_mut(index * 7) {
            *slot = UNKNOWN_VARIANCE;
        }
    }
    if let Some(ellipse) = reading.horizontal_error {
        let (east, north, cross) = horizontal(ellipse);
        pose[0] = east;
        pose[1] = cross;
        pose[6] = cross;
        pose[7] = north;
    }
    if let Some(sigma) = reading.height_sigma {
        pose[14] = sigma.metres() * sigma.metres();
    }
    if let Some(sigma) = reading.heading_sigma {
        // Yaw of ROS is about up: the same angle, the opposite sense.
        pose[35] = sigma.radians() * sigma.radians();
    }
    let mut twist = [0.0; 36];
    for index in 0..6 {
        if let Some(slot) = twist.get_mut(index * 7) {
            *slot = UNKNOWN_VARIANCE;
        }
    }
    Ok(Odometry {
        stamp: stamp(at)?,
        position: enu(reading.displacement),
        orientation: orientation(roll, pitch, heading),
        pose_covariance: pose,
        linear: flu(reading.velocity),
        angular: flu(reading.angular_rate),
        twist_covariance: twist,
    })
}
