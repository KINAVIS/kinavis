//! Geofence and field guidance events on the alert board.

#![cfg(all(feature = "land", feature = "agri"))]
#![allow(clippy::expect_used, clippy::indexing_slicing)]

use core::time::Duration;

use kinavis_agri::{AgriEvent, PassIndex};
use kinavis_alerts::{AlertKind, AlertManager, AlertPriority, AlertState, StandardPolicy};
use kinavis_kernel::time::{Instant, Utc};
use kinavis_kernel::Distance;
use kinavis_land::{FenceKind, GeofenceEvent};

fn at(second: u64) -> Instant<Utc> {
    Instant::<Utc>::from_unix_seconds(1_760_000_000).saturating_add(Duration::from_secs(second))
}

fn standing(board: &AlertManager<StandardPolicy>) -> Vec<(AlertKind, AlertPriority)> {
    board
        .alerts()
        .iter()
        .filter(|alert| alert.state() != AlertState::Rectified)
        .map(|alert| (alert.kind(), alert.priority()))
        .collect()
}

#[test]
fn a_geofence_warns_breaches_and_clears_when_back() {
    let mut board = AlertManager::new(StandardPolicy::default());
    let fence = 3;
    let kind = FenceKind::KeepIn;
    let near = GeofenceEvent::ApproachingBoundary {
        fence,
        kind,
        distance: Distance::from_metres(8.0).expect("8 m"),
        at: at(1),
    };
    let _ = board.ingest(
        &[
            GeofenceEvent::Entered {
                fence,
                kind,
                at: at(0),
            },
            near,
        ],
        at(1),
    );
    assert_eq!(
        standing(&board),
        [(
            AlertKind::GeofenceApproaching { fence },
            AlertPriority::Caution
        )]
    );
    let _ = board.ingest(
        &[GeofenceEvent::Exited {
            fence,
            kind,
            at: at(5),
        }],
        at(5),
    );
    assert!(standing(&board).contains(&(
        AlertKind::GeofenceBreached { fence },
        AlertPriority::Warning
    )));
    // Back inside: both conditions of the fence end, another fence's stay.
    let other = GeofenceEvent::Entered {
        fence: 9,
        kind: FenceKind::KeepOut,
        at: at(6),
    };
    let _ = board.ingest(
        &[
            other,
            GeofenceEvent::Entered {
                fence,
                kind,
                at: at(7),
            },
        ],
        at(7),
    );
    assert_eq!(
        standing(&board),
        [(
            AlertKind::GeofenceBreached { fence: 9 },
            AlertPriority::Warning
        )]
    );
}

#[test]
fn field_guidance_reports_off_line_and_the_end_of_the_pass() {
    let mut board = AlertManager::new(StandardPolicy::default());
    let off = AgriEvent::OffLine {
        cross_track: Distance::from_metres(0.6).expect("0.6 m"),
        limit: Distance::from_metres(0.3).expect("0.3 m"),
        at: at(0),
    };
    let end = AgriEvent::EndOfPassApproaching {
        remaining: Distance::from_metres(25.0).expect("25 m"),
        at: at(0),
    };
    let _ = board.ingest(&[off, end], at(0));
    let now = standing(&board);
    assert!(now.contains(&(AlertKind::CrossTrackExceeded, AlertPriority::Warning)));
    assert!(now.contains(&(AlertKind::EndOfPassApproaching, AlertPriority::Caution)));
    let turned = AgriEvent::PassChanged {
        from: PassIndex::REFERENCE,
        to: PassIndex::new(1),
        at: at(9),
    };
    let _ = board.ingest(&[turned], at(9));
    assert!(!standing(&board)
        .iter()
        .any(|(kind, _)| *kind == AlertKind::EndOfPassApproaching));
}
