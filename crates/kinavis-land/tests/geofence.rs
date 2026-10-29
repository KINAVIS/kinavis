//! Geofences on a square field of the UTM plane: crossings each way for
//! both kinds, the approach warning and its rearming, a hole, and the
//! hysteresis that keeps a scattering fix from chattering at the boundary.

#![allow(
    clippy::expect_used,
    clippy::panic,
    clippy::indexing_slicing,
    clippy::cast_precision_loss
)]

use core::time::Duration;

use kinavis::region::{PlanarPoint, Polygon, Region};
use kinavis_kernel::time::{Instant, Utc};
use kinavis_kernel::{Distance, Position};
use kinavis_land::{FenceConfig, FenceKind, Geofence, GeofenceEvent, GridPoint, MapGrid};
use proptest::prelude::*;

fn metres(value: f64) -> Distance {
    Distance::from_metres(value).expect("finite")
}

fn at(second: u64) -> Instant<Utc> {
    Instant::<Utc>::from_unix_seconds(1_760_000_000).saturating_add(Duration::from_secs(second))
}

/// A field near Tver: the grid, its centre, and a 200 m square about it with
/// an optional 40 m square hole in the middle.
fn field(hole: bool) -> (MapGrid, GridPoint, Region<4, 4, 1>) {
    let centre = Position::from_degrees(56.86, 35.92).expect("on Earth");
    let grid = MapGrid::utm_at(centre).expect("UTM");
    let c = grid.project(centre).expect("projected").point;
    let square = |half: f64| -> Polygon<4> {
        let (e, n) = (c.easting.metres(), c.northing.metres());
        Polygon::new(&[
            PlanarPoint::from_metres(e - half, n - half).expect("finite"),
            PlanarPoint::from_metres(e + half, n - half).expect("finite"),
            PlanarPoint::from_metres(e + half, n + half).expect("finite"),
            PlanarPoint::from_metres(e - half, n + half).expect("finite"),
        ])
        .expect("a square")
    };
    let region = if hole {
        Region::new(square(100.0), &[square(20.0)]).expect("a region")
    } else {
        Region::without_holes(square(100.0))
    };
    (grid, c, region)
}

/// The position `east` metres east of the field's centre.
fn east_of(grid: &MapGrid, centre: GridPoint, east: f64, north: f64) -> Position {
    let point = GridPoint::from_metres(
        centre.easting.metres() + east,
        centre.northing.metres() + north,
    )
    .expect("finite");
    grid.unproject(point).expect("on Earth").0
}

fn fence(
    kind: FenceKind,
    hysteresis: f64,
    approach: f64,
    hole: bool,
) -> (Geofence<4, 4, 1>, MapGrid, GridPoint) {
    let (grid, centre, region) = field(hole);
    let config = FenceConfig::new(metres(hysteresis), metres(approach)).expect("settings");
    (Geofence::new(7, kind, region, grid, config), grid, centre)
}

#[test]
fn a_drive_across_a_keep_in_field_enters_warns_and_leaves() {
    let (mut fence, grid, centre) = fence(FenceKind::KeepIn, 2.0, 15.0, false);
    let mut events = Vec::new();
    // West to east through the field, 1 m a second, from 150 m west.
    for k in 0..300u64 {
        let east = -150.0 + k as f64;
        if let Some(event) = fence
            .update(east_of(&grid, centre, east, 0.0), at(k))
            .expect("an update")
        {
            events.push((east, event));
        }
    }
    let names: Vec<&str> = events
        .iter()
        .map(|(_, e)| match e {
            GeofenceEvent::Entered { .. } => "entered",
            GeofenceEvent::Exited { .. } => "exited",
            GeofenceEvent::ApproachingBoundary { .. } => "approaching",
        })
        .collect();
    // Outside at the start, in past the band, a warning near the far side,
    // out past the band.
    assert_eq!(names, ["exited", "entered", "approaching", "exited"]);
    assert!(
        (events[1].0 - (-100.0 + 2.0)).abs() <= 1.0,
        "entered at {}",
        events[1].0
    );
    let GeofenceEvent::ApproachingBoundary {
        distance,
        fence: id,
        ..
    } = events[2].1
    else {
        panic!("a warning");
    };
    assert_eq!(id, 7);
    assert!((distance.metres() - 15.0).abs() <= 1.0 + 1e-6);
    assert!(
        (events[3].0 - (100.0 + 2.0)).abs() <= 1.0,
        "left at {}",
        events[3].0
    );
    assert!(events[3].1.is_breach());
    assert!(!events[1].1.is_breach());
}

#[test]
fn a_keep_out_zone_warns_from_outside_and_breaches_on_entry() {
    let (mut fence, grid, centre) = fence(FenceKind::KeepOut, 1.0, 20.0, false);
    let first = fence
        .update(east_of(&grid, centre, -200.0, 0.0), at(0))
        .expect("an update");
    assert!(matches!(first, Some(GeofenceEvent::Exited { .. })));
    assert!(!first.expect("an event").is_breach());
    let mut warned = None;
    let mut breached = None;
    for k in 1..150u64 {
        let east = -200.0 + k as f64;
        match fence
            .update(east_of(&grid, centre, east, 0.0), at(k))
            .expect("an update")
        {
            Some(GeofenceEvent::ApproachingBoundary { .. }) => warned = warned.or(Some(east)),
            Some(event @ GeofenceEvent::Entered { .. }) => {
                assert!(event.is_breach());
                breached = breached.or(Some(east));
            }
            _ => {}
        }
    }
    assert!((warned.expect("a warning") - (-120.0)).abs() <= 1.0);
    assert!((breached.expect("a breach") - (-99.0)).abs() <= 1.0);
}

#[test]
fn a_warning_rearms_only_beyond_the_band() {
    let (mut fence, grid, centre) = fence(FenceKind::KeepIn, 3.0, 10.0, false);
    let point = |east: f64| east_of(&grid, centre, east, 0.0);
    assert!(matches!(
        fence.update(point(0.0), at(0)).expect("ok"),
        Some(GeofenceEvent::Entered { .. })
    ));
    // 8 m from the east edge: a warning; staying near, no more.
    assert!(matches!(
        fence.update(point(92.0), at(1)).expect("ok"),
        Some(GeofenceEvent::ApproachingBoundary { .. })
    ));
    assert_eq!(fence.update(point(91.0), at(2)).expect("ok"), None);
    // Back to 12 m: inside the approach plus band, still quiet.
    assert_eq!(fence.update(point(88.0), at(3)).expect("ok"), None);
    assert_eq!(fence.update(point(93.0), at(4)).expect("ok"), None);
    // Back to 14 m, beyond 10 + 3: rearmed; near again, warned again.
    assert_eq!(fence.update(point(86.0), at(5)).expect("ok"), None);
    assert!(matches!(
        fence.update(point(95.0), at(6)).expect("ok"),
        Some(GeofenceEvent::ApproachingBoundary { .. })
    ));
}

#[test]
fn the_hole_of_a_field_is_outside_it() {
    let (mut fence, grid, centre) = fence(FenceKind::KeepIn, 1.0, 0.0, true);
    assert!(
        fence
            .signed_distance(east_of(&grid, centre, 0.0, 0.0))
            .expect("ok")
            .metres()
            < -19.0
    );
    assert!(
        (fence
            .signed_distance(east_of(&grid, centre, 60.0, 0.0))
            .expect("ok")
            .metres()
            - 40.0)
            .abs()
            < 1e-6
    );
    assert!(matches!(
        fence
            .update(east_of(&grid, centre, 60.0, 0.0), at(0))
            .expect("ok"),
        Some(GeofenceEvent::Entered { .. })
    ));
    // Into the hole.
    assert!(matches!(
        fence
            .update(east_of(&grid, centre, 10.0, 0.0), at(1))
            .expect("ok"),
        Some(GeofenceEvent::Exited { .. })
    ));
    assert_eq!(fence.inside(), Some(false));
    assert!(FenceConfig::new(metres(-1.0), metres(0.0)).is_err());
    assert!(FenceConfig::new(metres(1.0), metres(1e6)).is_err());
}

proptest! {
    #[test]
    fn a_fix_scattering_about_the_boundary_crosses_once(
        noise in proptest::collection::vec(-1.0f64..1.0, 200),
        scatter in 0.5f64..4.0,
    ) {
        // Hysteresis wider than the scatter: one way across the east edge,
        // at 0.2 m a fix, the fix jumping up to `scatter` either side.
        let (mut fence, grid, centre) = fence(FenceKind::KeepIn, scatter * 1.01, 0.0, false);
        let mut crossings = 0;
        for (k, jitter) in noise.iter().enumerate() {
            let east = 80.0 + 0.2 * k as f64 + scatter * jitter;
            if let Some(event) = fence.update(east_of(&grid, centre, east, 3.0), at(k as u64)).expect("ok") {
                if k > 0 {
                    prop_assert!(matches!(event, GeofenceEvent::Exited { .. }), "{event:?}");
                    crossings += 1;
                }
            }
        }
        // 80 + 40 m: well past the edge and the band.
        prop_assert_eq!(crossings, 1);
    }
}
