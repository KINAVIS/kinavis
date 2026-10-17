//! The conventions of ADR-0020: the orientation against the rotation matrix
//! built from the definitions of the frames, the angle rule of REP-103,
//! vectors, covariances, heights and times there and back.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::indexing_slicing
)]

use kinavis_kernel::geodesy::VerticalDatum;
use kinavis_kernel::{
    Angle, Distance, ErrorEllipse, FixType, GeodeticPoint, GnssFix, Height, Instant, Ned, Position,
    TrueCourse, Utc, Vector3,
};
use kinavis_ros2::{
    attitude, covariance_type, enu, flu, frd, from_nav_sat_fix, geo_point, imu, instant,
    nav_sat_fix, nav_sat_status, odometry, orientation, stamp, ImuReading, OdometryReading,
    Quaternion, UNKNOWN_VARIANCE,
};
use proptest::prelude::*;

type Matrix = [[f64; 3]; 3];

fn multiply(a: Matrix, b: Matrix) -> Matrix {
    let mut out = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            out[i][j] = (0..3).map(|k| a[i][k] * b[k][j]).sum();
        }
    }
    out
}

/// Body FRD to NED: Rz(ψ) Ry(θ) Rx(φ).
fn body_to_ned(roll: f64, pitch: f64, yaw: f64) -> Matrix {
    let (sr, cr) = roll.sin_cos();
    let (sp, cp) = pitch.sin_cos();
    let (sy, cy) = yaw.sin_cos();
    let rz = [[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]];
    let ry = [[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]];
    let rx = [[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]];
    multiply(rz, multiply(ry, rx))
}

fn quaternion_matrix(q: Quaternion) -> Matrix {
    let Quaternion { w, x, y, z } = q;
    [
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - w * z),
            2.0 * (x * z + w * y),
        ],
        [
            2.0 * (x * y + w * z),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - w * x),
        ],
        [
            2.0 * (x * z - w * y),
            2.0 * (y * z + w * x),
            1.0 - 2.0 * (x * x + y * y),
        ],
    ]
}

/// ZYX angles of a rotation matrix.
fn euler(m: Matrix) -> (f64, f64, f64) {
    (
        m[2][1].atan2(m[2][2]),
        (-m[2][0]).asin(),
        m[1][0].atan2(m[0][0]),
    )
}

fn wrap(degrees: f64) -> f64 {
    let wrapped = degrees.rem_euclid(360.0);
    if wrapped > 180.0 {
        wrapped - 360.0
    } else {
        wrapped
    }
}

proptest! {
    #[test]
    fn the_orientation_is_the_composed_rotation_and_follows_the_angle_rule(
        roll in -80.0..80.0f64, pitch in -85.0..85.0f64, heading in 0.0..360.0f64,
    ) {
        let q = orientation(
            Angle::from_degrees(roll).unwrap(),
            Angle::from_degrees(pitch).unwrap(),
            TrueCourse::wrap(heading).unwrap(),
        );
        // ENU ← NED swaps the first two axes and negates the third; FRD ← FLU
        // negates the last two.
        let ned_to_enu = [[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]];
        let flu_to_frd = [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]];
        let expected = multiply(
            ned_to_enu,
            multiply(body_to_ned(roll.to_radians(), pitch.to_radians(), heading.to_radians()), flu_to_frd),
        );
        let got = quaternion_matrix(q);
        for i in 0..3 {
            for j in 0..3 {
                prop_assert!((got[i][j] - expected[i][j]).abs() < 1e-12, "{i},{j}");
            }
        }
        // REP-103 angles: roll kept, pitch negated, yaw 90° − heading.
        let (r, p, y) = euler(got);
        prop_assert!((r.to_degrees() - roll).abs() < 1e-9);
        prop_assert!((p.to_degrees() + pitch).abs() < 1e-9);
        prop_assert!((wrap(y.to_degrees()) - wrap(90.0 - heading)).abs() < 1e-9);
        // And back.
        let (r, p, h) = attitude(q).unwrap();
        prop_assert!((r.degrees() - roll).abs() < 1e-9);
        prop_assert!((p.degrees() - pitch).abs() < 1e-9);
        prop_assert!(h.angular_distance(TrueCourse::wrap(heading).unwrap()) < 1e-9);
    }
}

#[test]
fn vectors_change_axes() {
    let ned = Vector3::<Ned, Distance>::new(
        Distance::from_metres(10.0).unwrap(),
        Distance::from_metres(5.0).unwrap(),
        Distance::from_metres(-2.0).unwrap(),
    );
    let v = enu(ned);
    assert_eq!((v.x, v.y, v.z), (5.0, 10.0, 2.0));
    let f = flu([0.1, 0.2, 0.3]);
    assert_eq!((f.x, f.y, f.z), (0.1, -0.2, -0.3));
    assert_eq!(frd(f), [0.1, 0.2, 0.3]);
}

fn fix(fix_type: FixType, ellipse: Option<ErrorEllipse>) -> GnssFix {
    let mut builder = GnssFix::builder(
        Instant::<Utc>::from_unix_seconds(1_760_000_000),
        Position::from_degrees(52.1, 4.3).unwrap(),
    )
    .fix_type(fix_type);
    if let Some(ellipse) = ellipse {
        builder = builder.horizontal_error(ellipse);
    }
    builder.build()
}

#[test]
fn a_fix_carries_its_ellipse_and_comes_back_with_it() {
    let ellipse = ErrorEllipse::new(
        Distance::from_metres(3.0).unwrap(),
        Distance::from_metres(1.0).unwrap(),
        TrueCourse::new(30.0).unwrap(),
    )
    .unwrap();
    let height = Height::above_ellipsoid(Distance::from_metres(43.2).unwrap());
    let message = nav_sat_fix(
        &fix(FixType::RtkFixed, Some(ellipse)),
        height,
        Some(Distance::from_metres(0.5).unwrap()),
    )
    .unwrap();
    assert_eq!(message.status, nav_sat_status::STATUS_GBAS_FIX);
    assert_eq!(
        message.position_covariance_type,
        covariance_type::APPROXIMATED
    );
    assert_eq!(message.position_covariance[8], 0.25);
    // The major axis 30° east of north: east variance 9 sin² + cos².
    let s = 30f64.to_radians().sin();
    assert!((message.position_covariance[0] - (9.0 * s * s + (1.0 - s * s))).abs() < 1e-12);
    let (point, back) = from_nav_sat_fix(&message).unwrap();
    assert_eq!(point.height().datum(), VerticalDatum::Ellipsoid);
    let back = back.unwrap();
    assert!((back.semi_major().metres() - 3.0).abs() < 1e-9);
    assert!((back.semi_minor().metres() - 1.0).abs() < 1e-9);
    assert!(
        back.orientation()
            .angular_distance(TrueCourse::new(30.0).unwrap())
            < 1e-6
    );
    // No ellipse: unknown; no fix: no fix.
    let unknown = nav_sat_fix(&fix(FixType::None, None), height, None).unwrap();
    assert_eq!(unknown.position_covariance_type, covariance_type::UNKNOWN);
    assert_eq!(unknown.status, nav_sat_status::STATUS_NO_FIX);
}

#[test]
fn a_height_above_sea_level_is_never_passed_as_ellipsoidal() {
    let msl = Height::above_mean_sea_level(Distance::from_metres(0.0).unwrap());
    assert!(nav_sat_fix(&fix(FixType::Autonomous, None), msl, None).is_err());
    let point = GeodeticPoint::new(Position::from_degrees(1.0, 2.0).unwrap(), msl);
    assert!(geo_point(point).is_err());
}

#[test]
fn an_unknown_imu_quantity_is_marked_minus_one() {
    let at = Instant::<Utc>::from_unix_seconds(1_760_000_000);
    let message = imu(
        at,
        &ImuReading {
            attitude: None,
            attitude_sigma: None,
            angular_rate: Some([0.1, 0.2, 0.3]),
            acceleration: None,
        },
    )
    .unwrap();
    assert_eq!(message.orientation_covariance[0], -1.0);
    assert_eq!(message.linear_acceleration_covariance[0], -1.0);
    assert_eq!(message.angular_velocity_covariance[0], 0.0);
    assert_eq!(
        (message.angular_velocity.y, message.angular_velocity.z),
        (-0.2, -0.3)
    );
}

#[test]
fn odometry_puts_the_pose_in_map_and_the_twist_in_base_link() {
    let at = Instant::<Utc>::from_unix_seconds(1_760_000_000);
    let reading = OdometryReading {
        displacement: Vector3::new(
            Distance::from_metres(10.0).unwrap(),
            Distance::from_metres(5.0).unwrap(),
            Distance::from_metres(-2.0).unwrap(),
        ),
        attitude: (Angle::ZERO, Angle::ZERO, TrueCourse::new(90.0).unwrap()),
        velocity: [2.0, 0.1, 0.0],
        angular_rate: [0.0, 0.0, 0.2],
        horizontal_error: Some(ErrorEllipse::circular(Distance::from_metres(0.02).unwrap())),
        height_sigma: None,
        heading_sigma: Some(Angle::from_degrees(1.0).unwrap()),
    };
    let message = odometry(at, &reading).unwrap();
    assert_eq!(
        (message.position.x, message.position.y, message.position.z),
        (5.0, 10.0, 2.0)
    );
    assert_eq!((message.linear.x, message.linear.y), (2.0, -0.1));
    // A right turn in FRD is a negative yaw rate about up.
    assert_eq!(message.angular.z, -0.2);
    assert!((message.pose_covariance[0] - 4e-4).abs() < 1e-15);
    assert_eq!(message.pose_covariance[14], UNKNOWN_VARIANCE);
    assert!((message.pose_covariance[35] - 1f64.to_radians().powi(2)).abs() < 1e-15);
    assert!(message.orientation.w > 0.999_999);
}

#[test]
fn times_go_to_ros_and_back() {
    let at = Instant::<Utc>::from_unix_seconds(1_760_000_000)
        .saturating_add(core::time::Duration::from_nanos(123_456_789));
    let ros = stamp(at).unwrap();
    assert_eq!((ros.sec, ros.nanosec), (1_760_000_000, 123_456_789));
    assert_eq!(instant(ros), at);
    assert!(stamp(Instant::<Utc>::from_unix_seconds(1 << 33)).is_err());
}
