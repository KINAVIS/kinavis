//! Messages to kernel types: positions and heights on their datum, NED
//! velocities, GNSS fixes, attitude; waypoints and targets refuse heights
//! not above sea level; RTCM in fragments.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::indexing_slicing
)]

use kinavis_kernel::geodesy::VerticalDatum;
use kinavis_kernel::{Distance, FixType, GeodeticPoint, Height, Instant, Position, Utc};
use kinavis_mavlink::{
    Attitude, GlobalPositionInt, GpsRawInt, GpsRtcmData, MissionItemInt,
    SetPositionTargetGlobalInt, FRAME_GLOBAL_INT, RTCM_MAX_BYTES,
};

#[test]
fn a_global_position_keeps_its_height_above_sea_level() {
    let message = GlobalPositionInt {
        lat: -337_000_000,
        lon: 1_512_000_000,
        alt: 12_500,
        relative_alt: -300,
        vx: 150,
        vy: -20,
        vz: 5,
        hdg: u16::MAX,
        ..GlobalPositionInt::ZERO
    };
    let point = message.point().unwrap();
    assert_eq!(point.height().datum(), VerticalDatum::MeanSeaLevel);
    assert!((point.height().value().metres() - 12.5).abs() < 1e-12);
    assert!((point.position().latitude().degrees() + 33.7).abs() < 1e-12);
    assert_eq!(message.heading(), None);
    let velocity = message.velocity().unwrap();
    assert!((velocity.north().metres_per_second() - 1.5).abs() < 1e-12);
    assert!((velocity.down().metres_per_second() - 0.05).abs() < 1e-12);
    assert!((message.relative_height().unwrap().metres() + 0.3).abs() < 1e-12);
}

#[test]
fn a_raw_gps_message_is_a_kernel_fix_when_it_has_one() {
    let at = Instant::<Utc>::from_unix_seconds(1_760_000_000);
    let rtk = GpsRawInt {
        fix_type: 6,
        lat: 520_000_000,
        lon: 50_000_000,
        eph: 80,
        epv: u16::MAX,
        vel: 250,
        cog: 9_000,
        satellites_visible: 17,
        alt_ellipsoid: 45_000,
        h_acc: 14,
        ..GpsRawInt::ZERO
    };
    let fix = rtk.gnss_fix(at).unwrap().expect("a fix");
    assert_eq!(fix.quality().fix_type(), FixType::RtkFixed);
    assert_eq!(fix.quality().satellites(), Some(17));
    assert!(fix
        .quality()
        .hdop()
        .is_some_and(|d| (d.value() - 0.8).abs() < 1e-12));
    assert_eq!(fix.quality().vdop(), None);
    assert!(fix
        .speed_over_ground()
        .is_some_and(|s| (s.metres_per_second() - 2.5).abs() < 1e-12));
    assert_eq!(
        rtk.ellipsoidal_height().unwrap().map(|h| h.datum()),
        Some(VerticalDatum::Ellipsoid)
    );
    assert_eq!(
        rtk.horizontal_accuracy()
            .map(|d| (d.metres() * 1e3).round()),
        Some(14.0)
    );
    let none = GpsRawInt { fix_type: 1, ..rtk };
    assert_eq!(none.gnss_fix(at).unwrap(), None);
}

#[test]
fn attitude_reads_in_the_kernels_signs() {
    let attitude = Attitude {
        roll: 0.1,
        pitch: -0.2,
        yaw: -1.570_796_4,
        yawspeed: 0.05,
        ..Attitude::ZERO
    };
    assert!((attitude.roll_angle().unwrap().radians() - 0.1).abs() < 1e-7);
    assert!((attitude.heading().unwrap().degrees() - 270.0).abs() < 1e-4);
    assert!((attitude.yaw_rate().unwrap().radians_per_second() - 0.05).abs() < 1e-7);
}

#[test]
fn a_target_or_waypoint_takes_a_height_above_sea_level_only() {
    let position = Position::from_degrees(48.1, 11.6).unwrap();
    let amsl = GeodeticPoint::new(
        position,
        Height::above_mean_sea_level(Distance::from_metres(520.0).unwrap()),
    );
    let target = SetPositionTargetGlobalInt::position(1, 1, amsl).unwrap();
    assert_eq!(target.coordinate_frame, FRAME_GLOBAL_INT);
    assert_eq!(target.type_mask, 0x0FF8);
    assert_eq!(target.lat_int, 481_000_000);
    assert_eq!(target.alt, 520.0);
    let waypoint = MissionItemInt::waypoint(1, 1, 4, amsl).unwrap();
    assert_eq!(
        (waypoint.seq, waypoint.command, waypoint.y),
        (4, 16, 116_000_000)
    );
    let ellipsoidal = GeodeticPoint::new(
        position,
        Height::above_ellipsoid(Distance::from_metres(567.0).unwrap()),
    );
    assert!(SetPositionTargetGlobalInt::position(1, 1, ellipsoidal).is_err());
    assert!(MissionItemInt::waypoint(1, 1, 0, ellipsoidal).is_err());
}

#[test]
fn rtcm_goes_in_one_message_or_up_to_four_fragments() {
    let mut sent = Vec::new();
    GpsRtcmData::fragments(&[7; 100], 9, |m| sent.push(m)).unwrap();
    assert_eq!(sent.len(), 1);
    assert_eq!((sent[0].flags, sent[0].len), (9 << 3, 100));
    sent.clear();
    let rtcm: Vec<u8> = (0..500).map(|i| u8::try_from(i % 251).unwrap()).collect();
    GpsRtcmData::fragments(&rtcm, 33, |m| sent.push(m)).unwrap();
    assert_eq!(sent.len(), 3);
    for (index, message) in sent.iter().enumerate() {
        assert_eq!(message.flags & 1, 1);
        assert_eq!(usize::from(message.flags >> 1 & 3), index);
        assert_eq!(message.flags >> 3, 1, "the sequence keeps five bits");
    }
    let joined: Vec<u8> = sent
        .iter()
        .flat_map(|m| m.data[..usize::from(m.len)].to_vec())
        .collect();
    assert_eq!(joined, rtcm);
    assert!(GpsRtcmData::fragments(&vec![0; RTCM_MAX_BYTES + 1], 0, |_| {}).is_err());
}
