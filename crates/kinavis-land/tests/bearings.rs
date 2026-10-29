//! Bearings, mils, the pace count, the map grid and routes by bearing.

#![allow(
    clippy::expect_used,
    clippy::panic,
    clippy::float_cmp,
    clippy::indexing_slicing
)]

use kinavis::route::Route;
use kinavis::sailings::geodesic;
use kinavis_grid::{grid_magnetic_angle, Hemisphere, ProjectedCrs, UtmUps, UtmZone};
use kinavis_kernel::{
    Angle, Distance, GridBearing, Magnetic, MagneticBearing, Position, TrueBearing, Variation,
};
use kinavis_land::{
    legs_of, route_from_legs, BearingFrame, GridPoint, Leg, MapGrid, Mils, PaceCount,
};
use proptest::prelude::*;

fn degrees(value: f64) -> Angle {
    Angle::from_degrees(value).expect("finite")
}

fn position(lat: f64, lon: f64) -> Position {
    Position::from_degrees(lat, lon).expect("on Earth")
}

fn metres(value: f64) -> Distance {
    Distance::from_metres(value).expect("finite")
}

fn same_direction(a: f64, b: f64, tolerance: f64) -> bool {
    ((a - b + 540.0).rem_euclid(360.0) - 180.0).abs() < tolerance
}

#[test]
fn the_frame_is_the_map_margins_rule() {
    // East declination 11.5°, convergence 1.5°: grid magnetic angle +10°, a
    // magnetic bearing plus 10° is the grid bearing.
    let frame = BearingFrame::new(degrees(1.5), Variation::new(11.5).expect("11.5°E"));
    assert!((frame.grid_magnetic_angle().degrees() - 10.0).abs() < 1e-12);
    let magnetic = MagneticBearing::new(350.0).expect("350");
    let grid = frame.magnetic_to_grid(magnetic);
    assert!((grid.degrees() - 0.0).abs() < 1e-12 || (grid.degrees() - 360.0).abs() < 1e-12);
    assert_eq!(frame.grid_to_magnetic(grid).degrees(), 350.0);
    // The same through true north.
    let truth: TrueBearing = frame.magnetic_to_true(magnetic);
    assert!((truth.degrees() - 1.5).abs() < 1e-12);
    assert!(same_direction(
        frame.true_to_grid(truth).degrees(),
        grid.degrees(),
        1e-12
    ));
    assert!(same_direction(
        frame.grid_to_true(grid).degrees(),
        truth.degrees(),
        1e-12
    ));
    assert!(same_direction(
        frame.true_to_magnetic(truth).degrees(),
        350.0,
        1e-12
    ));
    // As kinavis-grid prints it.
    let printed = grid_magnetic_angle(frame.declination(), frame.convergence()).expect("finite");
    assert!((printed.degrees() - frame.grid_magnetic_angle().degrees()).abs() < 1e-12);
    // West declination and west convergence: δ − γ = −7.2 − (−2.0).
    let west = BearingFrame::new(degrees(-2.0), Variation::new(-7.2).expect("7.2°W"));
    assert!((west.grid_magnetic_angle().degrees() + 5.2).abs() < 1e-12);
}

#[test]
fn the_frame_at_a_point_takes_the_grids_convergence() {
    let moscow = position(55.75, 37.62);
    let grid = MapGrid::utm_at(moscow).expect("UTM");
    let frame =
        BearingFrame::at(&grid, moscow, Variation::new(11.0).expect("11°E")).expect("frame");
    let utm = UtmUps::from_position(moscow).expect("UTM");
    assert_eq!(frame.convergence(), utm.convergence);
}

proptest! {
    #[test]
    fn every_conversion_has_its_inverse(
        convergence in -10.0f64..10.0,
        declination in -40.0f64..40.0,
        bearing in 0.0f64..360.0,
    ) {
        let frame = BearingFrame::new(degrees(convergence), Variation::new(declination).expect("δ"));
        let grid = GridBearing::from_degrees_wrapped(bearing);
        let back = frame.magnetic_to_grid(frame.grid_to_magnetic(grid));
        prop_assert!(same_direction(back.degrees(), bearing, 1e-9));
        let through_true = frame.true_to_magnetic(frame.grid_to_true(grid));
        prop_assert!(same_direction(through_true.degrees(), frame.grid_to_magnetic(grid).degrees(), 1e-9));
    }
}

#[test]
fn mils_of_both_systems() {
    // 60-00 is a full circle, 15-00 a right angle, 0-01 the тысячная.
    assert_eq!(Mils::Soviet.of(degrees(90.0)), 1500.0);
    assert_eq!(Mils::Soviet.angle(6000.0).expect("circle").degrees(), 360.0);
    assert!((Mils::Soviet.angle(1.0).expect("0-01").degrees() - 0.06).abs() < 1e-15);
    assert_eq!(Mils::Nato.of(degrees(90.0)), 1600.0);
    assert_eq!(Mils::Nato.angle(3200.0).expect("half").degrees(), 180.0);
    let west: MagneticBearing = Mils::Soviet.direction::<Magnetic>(4500.0).expect("45-00");
    assert_eq!(west.degrees(), 270.0);
    let wrapped: MagneticBearing = Mils::Nato.direction::<Magnetic>(-1600.0).expect("−1600");
    assert_eq!(wrapped.degrees(), 270.0);
    assert_eq!(Mils::Soviet.of_direction(west), 4500.0);
    assert!(Mils::Soviet.angle(f64::NAN).is_err());
    assert!(Mils::Soviet.angle(60_001.0).is_err());
    // The rule of thousandths: 1 mil (6000) subtends 1 m at about 955 m.
    let subtends = 955.0 * Mils::Soviet.angle(1.0).expect("0-01").radians();
    assert!((subtends - 1.0).abs() < 1e-3);
}

#[test]
fn the_pace_count_is_calibrated_and_reduced_on_a_slope() {
    let pace = PaceCount::calibrate(65.0, metres(100.0)).expect("65 pairs per 100 m");
    assert!((pace.pair().metres() - 100.0 / 65.0).abs() < 1e-12);
    let level = pace.distance(650.0, Angle::ZERO).expect("level");
    assert!((level.metres() - 1000.0).abs() < 1e-9);
    let steep = pace.distance(650.0, degrees(20.0)).expect("20°");
    assert!((steep.metres() - 1000.0 * 20f64.to_radians().cos()).abs() < 1e-9);
    let pairs = pace
        .pairs_for(metres(1000.0), degrees(-20.0))
        .expect("down");
    assert!((pairs - 650.0 / 20f64.to_radians().cos()).abs() < 1e-9);
    assert!(PaceCount::new(metres(0.3)).is_err());
    assert!(PaceCount::calibrate(0.0, metres(100.0)).is_err());
    assert!(pace.distance(-1.0, Angle::ZERO).is_err());
    assert!(pace.distance(10.0, degrees(61.0)).is_err());
}

#[test]
fn a_utm_grid_takes_points_of_the_next_zone_and_hemisphere() {
    // A sheet of zone 36 running over into 37, and across the equator.
    let grid = MapGrid::Utm {
        zone: UtmZone::new(36).expect("36"),
        hemisphere: Hemisphere::North,
    };
    let beyond = position(55.0, 36.5);
    let on = grid.project(beyond).expect("in 36 widened");
    let (back, again) = grid.unproject(on.point).expect("back");
    assert!((back.latitude().degrees() - 55.0).abs() < 1e-11);
    assert!((back.longitude().degrees() - 36.5).abs() < 1e-11);
    assert!((again.scale - on.scale).abs() < 1e-12);
    let south = position(-0.01, 33.0);
    let below = grid.project(south).expect("south of the equator");
    assert!(below.point.northing.metres() < 0.0);
    let (back, _) = grid.unproject(below.point).expect("back");
    assert!((back.latitude().degrees() + 0.01).abs() < 1e-11);
    let southern = MapGrid::Utm {
        zone: UtmZone::new(36).expect("36"),
        hemisphere: Hemisphere::South,
    };
    let north = southern
        .project(position(0.01, 33.0))
        .expect("north of the equator");
    assert!(north.point.northing.metres() > 10_000_000.0);
    assert!(MapGrid::utm_at(position(85.0, 0.0)).is_err());
}

#[test]
fn an_sk42_sheet_is_a_map_grid() {
    let crs = ProjectedCrs::pulkovo_1942(7).expect("zone 7");
    let grid = MapGrid::Projected(crs);
    let fix = position(55.7539, 37.6208);
    let on = grid.project(fix).expect("projected");
    assert_eq!(
        on.point.easting,
        crs.project(fix).expect("projected").easting
    );
    let (back, again) = grid.unproject(on.point).expect("back");
    assert!((back.latitude().degrees() - 55.7539).abs() < 1e-9);
    assert!((again.convergence.degrees() - on.convergence.degrees()).abs() < 1e-9);
    assert!((again.scale - on.scale).abs() < 1e-12);
}

#[test]
fn a_route_by_bearings_is_walked_and_read_back() {
    let start = position(56.8587, 35.9176);
    let grid = MapGrid::utm_at(start).expect("UTM");
    let frame =
        BearingFrame::at(&grid, start, Variation::new(10.4).expect("10.4°E")).expect("frame");
    let legs = [
        Leg {
            bearing: GridBearing::new(45.0).expect("045"),
            distance: metres(1_200.0),
        },
        Leg::magnetic(
            &frame,
            MagneticBearing::new(120.0).expect("120"),
            metres(800.0),
        ),
        Leg {
            bearing: GridBearing::new(270.0).expect("270"),
            distance: metres(2_500.0),
        },
        Leg {
            bearing: GridBearing::new(0.0).expect("000"),
            distance: metres(350.0),
        },
    ];
    let route: Route = route_from_legs(&grid, start, &legs).expect("a route");
    assert_eq!(route.waypoints().len(), 5);
    let mut card = [Leg {
        bearing: GridBearing::NORTH,
        distance: Distance::ZERO,
    }; 4];
    assert_eq!(legs_of(&grid, &route, &mut card).expect("the card"), 4);
    for (walked, read) in legs.iter().zip(&card) {
        assert!(same_direction(
            walked.bearing.degrees(),
            read.bearing.degrees(),
            1e-9
        ));
        assert!((walked.distance.metres() - read.distance.metres()).abs() < 1e-6);
        // The ground distance is the ellipsoid's to the arc-to-chord.
    }
    for (pair, leg) in route.waypoints().windows(2).zip(&legs) {
        let ground = geodesic(pair[0], pair[1]).expect("apart").distance.metres();
        assert!(
            (ground - leg.distance.metres()).abs() < 1e-6 * leg.distance.metres(),
            "{ground} against {}",
            leg.distance.metres()
        );
    }
    assert!(same_direction(
        card[1].magnetic_bearing(&frame).degrees(),
        120.0,
        1e-9
    ));
    let mut short = [card[0]; 3];
    assert!(legs_of(&grid, &route, &mut short).is_err());
    assert!(route_from_legs(&grid, start, &[legs[0]; 128]).is_err());
}

#[test]
fn grid_points_are_finite() {
    assert!(GridPoint::from_metres(f64::NAN, 0.0).is_err());
    assert!(GridPoint::from_metres(0.0, f64::INFINITY).is_err());
}
