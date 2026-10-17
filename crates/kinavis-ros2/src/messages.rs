//! Mirrors of the ROS 2 messages: the fields a node copies into the
//! generated message types, in ROS units and frames.

/// `builtin_interfaces/Time`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Stamp {
    /// Seconds since the Unix epoch.
    pub sec: i32,
    /// Nanoseconds within the second.
    pub nanosec: u32,
}

/// `geometry_msgs/Quaternion`.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Quaternion {
    /// x.
    pub x: f64,
    /// y.
    pub y: f64,
    /// z.
    pub z: f64,
    /// w.
    pub w: f64,
}

/// `geometry_msgs/Vector3` and `Point`.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Vector {
    /// x.
    pub x: f64,
    /// y.
    pub y: f64,
    /// z.
    pub z: f64,
}

/// `sensor_msgs/NavSatStatus` values.
pub mod nav_sat_status {
    /// No fix.
    pub const STATUS_NO_FIX: i8 = -1;
    /// Unaugmented fix.
    pub const STATUS_FIX: i8 = 0;
    /// Satellite-based augmentation.
    pub const STATUS_SBAS_FIX: i8 = 1;
    /// Ground-based augmentation (RTK, DGPS).
    pub const STATUS_GBAS_FIX: i8 = 2;
    /// GPS.
    pub const SERVICE_GPS: u16 = 1;
}

/// `sensor_msgs/NavSatFix` covariance types.
pub mod covariance_type {
    /// Unknown.
    pub const UNKNOWN: u8 = 0;
    /// Approximated (from an error ellipse, no cross terms with height).
    pub const APPROXIMATED: u8 = 1;
    /// Diagonal known.
    pub const DIAGONAL_KNOWN: u8 = 2;
    /// Fully known.
    pub const KNOWN: u8 = 3;
}

/// `sensor_msgs/NavSatFix`: its `frame_id` is the antenna's frame on the
/// vehicle, not `earth`; position on the WGS 84 ellipsoid.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct NavSatFix {
    /// Time of the fix.
    pub stamp: Stamp,
    /// `NavSatStatus.status`.
    pub status: i8,
    /// `NavSatStatus.service`.
    pub service: u16,
    /// Latitude, degrees.
    pub latitude: f64,
    /// Longitude, degrees.
    pub longitude: f64,
    /// Height above the WGS 84 ellipsoid, metres.
    pub altitude: f64,
    /// ENU covariance, m², row-major.
    pub position_covariance: [f64; 9],
    /// One of [`covariance_type`].
    pub position_covariance_type: u8,
}

/// `sensor_msgs/Imu`, orientation of FLU in ENU.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Imu {
    /// Time of the sample.
    pub stamp: Stamp,
    /// Orientation.
    pub orientation: Quaternion,
    /// Its covariance, rad²; first element −1 when there is no orientation.
    pub orientation_covariance: [f64; 9],
    /// Angular velocity, FLU, rad/s.
    pub angular_velocity: Vector,
    /// Its covariance; first element −1 when unknown.
    pub angular_velocity_covariance: [f64; 9],
    /// Linear acceleration, FLU, m/s².
    pub linear_acceleration: Vector,
    /// Its covariance; first element −1 when unknown.
    pub linear_acceleration_covariance: [f64; 9],
}

/// `nav_msgs/Odometry`: pose of `base_link` in `map`, twist in `base_link`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Odometry {
    /// Time of the estimate.
    pub stamp: Stamp,
    /// Position in `map` (ENU), metres.
    pub position: Vector,
    /// Orientation of `base_link` (FLU) in `map`.
    pub orientation: Quaternion,
    /// Pose covariance, 6 × 6 row-major (x, y, z, rotations about x, y, z).
    pub pose_covariance: [f64; 36],
    /// Linear velocity in `base_link`, m/s.
    pub linear: Vector,
    /// Angular velocity in `base_link`, rad/s.
    pub angular: Vector,
    /// Twist covariance, 6 × 6.
    pub twist_covariance: [f64; 36],
}

/// `geographic_msgs/GeoPoint`: latitude, longitude, height above the
/// ellipsoid.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct GeoPoint {
    /// Latitude, degrees.
    pub latitude: f64,
    /// Longitude, degrees.
    pub longitude: f64,
    /// Height above the WGS 84 ellipsoid, metres.
    pub altitude: f64,
}
