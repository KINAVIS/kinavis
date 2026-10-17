//! Grids over their whole domain: there and back, the hemisphere and zone
//! rules, the sign of the convergence, refused inputs.

#![allow(clippy::expect_used)]

use kinavis_grid::{grid_magnetic_angle, Hemisphere, ProjectedCrs, UtmUps, UtmZone};
use kinavis_kernel::{Angle, Distance, Position, Variation};
use proptest::prelude::*;

proptest! {
    #[test]
    fn utm_and_ups_there_and_back_close_to_a_tenth_of_a_millimetre(
        latitude in -90.0_f64..=90.0,
        longitude in -180.0_f64..180.0,
    ) {
        let position = Position::from_degrees(latitude, longitude).expect("on Earth");
        let gridded = UtmUps::from_position(position).expect("a standard zone");
        let (back, convergence, scale) = gridded.coordinate.to_position().expect("inside its limits");
        // 0.1 mm of latitude; of longitude, scaled by the parallel's radius.
        prop_assert!((back.latitude().degrees() - latitude).abs() < 1e-9);
        let parallel = latitude.to_radians().cos();
        prop_assert!(back.longitude_difference(position).degrees().abs() * parallel < 1e-9);
        prop_assert!((convergence.degrees() - gridded.convergence.degrees()).abs() < 1e-6 || latitude.abs() > 89.999);
        prop_assert!((scale - gridded.scale).abs() < 1e-9);
    }

    #[test]
    fn the_hemisphere_follows_the_sign_of_the_latitude(
        latitude in -79.0_f64..=83.0,
        longitude in -180.0_f64..180.0,
    ) {
        let gridded = UtmUps::from_position(Position::from_degrees(latitude, longitude).expect("on Earth"))
            .expect("UTM");
        let UtmUps::Utm { zone, hemisphere, easting, .. } = gridded.coordinate else {
            return Err(TestCaseError::fail("UTM expected"));
        };
        prop_assert_eq!(hemisphere == Hemisphere::North, latitude >= 0.0);
        // A standard zone holds 3° either side of its central meridian; the
        // zones widened for Norway (32V) and Svalbard (31X–37X) reach 6°.
        let off = kinavis_kernel::wrap180(longitude - zone.central_meridian()).abs();
        let half = if off <= 3.0 { 3.0 } else { 6.0 };
        prop_assert!(off <= half, "{off}° from the central meridian of zone {}", zone.number());
        // The easting within that, at the parallel's radius and a little
        // more for the projection's growth away from the meridian.
        let reach = 1.01 * 0.9996 * 6_378_137.0 * latitude.to_radians().cos() * half.to_radians();
        prop_assert!((easting.metres() - 500_000.0).abs() <= reach, "easting {}", easting.metres());
    }

    #[test]
    fn east_of_the_central_meridian_grid_north_is_east_of_true_north(
        latitude in 1.0_f64..=80.0,
        offset in 0.1_f64..=2.9,
        zone in 1_u8..=60,
    ) {
        let zone = UtmZone::new(zone).expect("1-60");
        let longitude = kinavis_kernel::wrap180(zone.central_meridian() + offset);
        let gridded = UtmUps::in_zone(Position::from_degrees(latitude, longitude).expect("on Earth"), zone)
            .expect("in the zone");
        prop_assert!(gridded.convergence.degrees() > 0.0);
        // Spherical approximation γ ≈ Δλ sin φ, within a few hundredths.
        let approximate = offset * latitude.to_radians().sin();
        prop_assert!((gridded.convergence.degrees() - approximate).abs() < 0.05);
    }

    #[test]
    fn a_national_grid_there_and_back_closes_through_the_datum_shift(
        latitude in 42.0_f64..=70.0,
        offset in -3.0_f64..=3.0,
        zone in 6_u8..=20,
    ) {
        let crs = ProjectedCrs::pulkovo_1942(zone).expect("a zone");
        let fix = Position::from_degrees(latitude, f64::from(zone) * 6.0 - 3.0 + offset).expect("on Earth");
        let projected = crs.project(fix).expect("projects");
        let back = crs.unproject(projected.easting, projected.northing).expect("unprojects");
        // The datum shift drops the height at each end: about a centimetre.
        prop_assert!((back.latitude().degrees() - latitude).abs() < 1e-7);
        prop_assert!(back.longitude_difference(fix).degrees().abs() < 2e-7);
    }
}

#[test]
fn norway_and_svalbard_have_their_own_zones() {
    let zone = |latitude, longitude| {
        UtmZone::standard(Position::from_degrees(latitude, longitude).expect("on Earth"))
            .map(UtmZone::number)
    };
    assert_eq!(zone(60.0, 5.0), Some(32), "Bergen");
    assert_eq!(zone(60.0, 2.0), Some(31));
    assert_eq!(zone(78.0, 8.0), Some(31), "west Svalbard");
    assert_eq!(zone(78.0, 15.0), Some(33), "Longyearbyen");
    assert_eq!(zone(78.0, 25.0), Some(35));
    assert_eq!(zone(78.0, 40.0), Some(37));
    assert_eq!(zone(84.0, 0.0), None, "UPS from 84°N");
    assert_eq!(zone(-80.5, 0.0), None, "UPS south of 80°S");
}

#[test]
fn out_of_place_requests_are_refused() {
    assert!(UtmZone::new(0).is_err());
    assert!(UtmZone::new(61).is_err());
    let equator = Position::from_degrees(0.0, 0.0).expect("on Earth");
    assert!(UtmUps::polar(equator).is_err(), "UPS at the equator");
    assert!(
        UtmUps::in_zone(equator, UtmZone::new(30).expect("zone")).is_ok(),
        "the next zone"
    );
    assert!(
        UtmUps::in_zone(equator, UtmZone::new(10).expect("zone")).is_err(),
        "120° away"
    );
    let wild = UtmUps::Utm {
        zone: UtmZone::new(31).expect("zone"),
        hemisphere: Hemisphere::North,
        easting: Distance::from_metres(1_200_000.0).expect("finite"),
        northing: Distance::from_metres(5_000_000.0).expect("finite"),
    };
    assert!(wild.to_position().is_err(), "easting off the grid");
    assert!(ProjectedCrs::pulkovo_1942(0).is_err());
    assert!(ProjectedCrs::dhdn(121).is_err());
}

#[test]
fn the_grid_magnetic_angle_is_variation_less_convergence() {
    let angle = grid_magnetic_angle(
        Variation::new(11.5).expect("in range"),
        Angle::from_degrees(-1.2).expect("finite"),
    )
    .expect("finite");
    assert!((angle.degrees() - 12.7).abs() < 1e-12);
}
