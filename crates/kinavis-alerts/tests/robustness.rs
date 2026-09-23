//! The board stays consistent under any sequence of events at any times.

#![allow(clippy::unwrap_used, clippy::indexing_slicing)]

use core::time::Duration;
use kinavis::event::{AnchorEvent, ClearanceEvent, GuidanceEvent};
use kinavis_alerts::{
    AlertChange, AlertKind, AlertManager, AlertPolicy, AlertPriority, AlertState, Ended,
    Reportable, StandardPolicy, MAX_ALERTS,
};
use kinavis_kernel::event::{Event, NavigationIntegrity, PositionSource, SensorHealth, SensorId};
use kinavis_kernel::{Distance, EventList, Instant, NavigationEvent, Position, TargetId, Utc};
use kinavis_traffic::TrafficEvent;

/// Application-style union of every context's events, reportable by delegation.
#[derive(Debug, Clone, Copy, PartialEq)]
enum AnyEvent {
    Navigation(NavigationEvent),
    Guidance(GuidanceEvent),
    Clearance(ClearanceEvent),
    Anchor(AnchorEvent),
    Traffic(TrafficEvent),
}

impl Event for AnyEvent {
    const PLACEHOLDER: Self = Self::Navigation(NavigationEvent::PLACEHOLDER);

    fn at(&self) -> Instant<Utc> {
        match self {
            Self::Navigation(event) => event.at(),
            Self::Guidance(event) => event.at(),
            Self::Clearance(event) => event.at(),
            Self::Anchor(event) => event.at(),
            Self::Traffic(event) => event.at(),
        }
    }
}

impl Reportable for AnyEvent {
    fn condition(&self) -> Option<AlertKind> {
        match self {
            Self::Navigation(event) => event.condition(),
            Self::Guidance(event) => event.condition(),
            Self::Clearance(event) => event.condition(),
            Self::Anchor(event) => event.condition(),
            Self::Traffic(event) => event.condition(),
        }
    }

    fn ends(&self) -> Ended {
        match self {
            Self::Navigation(event) => event.ends(),
            Self::Guidance(event) => event.ends(),
            Self::Clearance(event) => event.ends(),
            Self::Anchor(event) => event.ends(),
            Self::Traffic(event) => event.ends(),
        }
    }
}

/// Every event kind, about a few subjects, at a given time.
fn every_event(at: Instant<Utc>, subject: u32) -> Vec<AnyEvent> {
    let target = TargetId::new(subject);
    let sensor = SensorId::named(if subject % 2 == 0 { "GNSS 1" } else { "GNSS 2" });
    let source = if subject % 2 == 0 {
        PositionSource::Gnss
    } else {
        PositionSource::DeadReckoning
    };
    let distance = Distance::from_cables(3.0).unwrap();
    vec![
        AnyEvent::Navigation(NavigationEvent::FixAcquired { source, at }),
        AnyEvent::Navigation(NavigationEvent::FixLost {
            source,
            at,
            last_good: at,
        }),
        AnyEvent::Navigation(NavigationEvent::ObservationRejected {
            reason: kinavis_kernel::event::RejectionReason::Stale,
            at,
        }),
        AnyEvent::Navigation(NavigationEvent::IntegrityChanged {
            from: NavigationIntegrity::Nominal,
            to: if subject % 3 == 0 {
                NavigationIntegrity::Nominal
            } else if subject % 3 == 1 {
                NavigationIntegrity::DeadReckoning
            } else {
                NavigationIntegrity::Exceeded
            },
            at,
        }),
        AnyEvent::Navigation(NavigationEvent::SensorHealthChanged {
            sensor,
            from: SensorHealth::Healthy,
            to: if subject % 2 == 0 {
                SensorHealth::Suspect
            } else {
                SensorHealth::Healthy
            },
            at,
        }),
        AnyEvent::Guidance(GuidanceEvent::WaypointReached { index: 1, at }),
        AnyEvent::Guidance(GuidanceEvent::WheelOverReached {
            index: 1,
            wheel_over_at: Position::from_degrees(50.0, -1.0).unwrap(),
            at,
        }),
        AnyEvent::Guidance(GuidanceEvent::CrossTrackExceeded {
            error: distance,
            limit: distance,
            at,
        }),
        AnyEvent::Clearance(ClearanceEvent::UnderKeelClearanceLow {
            clearance: distance,
            required: distance,
            at,
        }),
        AnyEvent::Anchor(AnchorEvent::AnchorDragging {
            distance,
            radius: distance,
            at,
        }),
        AnyEvent::Traffic(TrafficEvent::TargetAcquired { target, at }),
        AnyEvent::Traffic(TrafficEvent::TargetLost {
            target,
            last_seen: at,
        }),
        AnyEvent::Traffic(TrafficEvent::TargetEvicted {
            target,
            last_seen: at,
            for_target: TargetId::new(subject.wrapping_add(1)),
        }),
        AnyEvent::Traffic(TrafficEvent::CpaAlarm {
            target,
            cpa: distance,
            tcpa: Duration::from_secs(60),
            at,
        }),
    ]
}

#[test]
fn any_sequence_of_events_keeps_the_board_coherent() {
    let moments = [
        Instant::<Utc>::from_unix_seconds(i64::MIN),
        Instant::from_unix_seconds(0),
        Instant::from_unix_seconds(1_789_000_000),
        Instant::from_unix_seconds(1_789_000_031),
        Instant::from_unix_seconds(1_789_000_000),
        Instant::from_unix_seconds(i64::MAX),
    ];
    let mut alerts = AlertManager::new(StandardPolicy::default());
    let mut raised = 0_u32;
    for (round, &now) in moments.iter().enumerate() {
        for subject in 0..40_u32 {
            let mut events = EventList::<AnyEvent, 16>::with_capacity();
            for event in every_event(now, subject + u32::try_from(round).unwrap() * 7) {
                events.push(event);
            }
            assert!(!events.overflowed());
            let changes = alerts.ingest(&events, now);
            // A later event in the batch can reverse an earlier change to the
            // same alert (target lost, then too close again); only the final
            // state per alert is checked.
            for (position, change) in changes.iter().enumerate() {
                let id = match change {
                    AlertChange::Raised(id)
                    | AlertChange::Cleared(id)
                    | AlertChange::Rectified(id)
                    | AlertChange::Acknowledged(id)
                    | AlertChange::Dropped(id) => *id,
                    _ => continue,
                };
                if matches!(change, AlertChange::Raised(_)) {
                    raised += 1;
                }
                let last_word = changes.iter().skip(position + 1).all(|later| match later {
                    AlertChange::Raised(other)
                    | AlertChange::Cleared(other)
                    | AlertChange::Rectified(other)
                    | AlertChange::Acknowledged(other)
                    | AlertChange::Dropped(other) => *other != id,
                    _ => true,
                });
                if !last_word {
                    continue;
                }
                match change {
                    AlertChange::Raised(_) => {
                        assert_eq!(alerts.alert(id).unwrap().state(), AlertState::Active);
                    }
                    AlertChange::Cleared(_) | AlertChange::Dropped(_) => {
                        assert!(alerts.alert(id).is_none());
                    }
                    AlertChange::Rectified(_) => {
                        assert_eq!(alerts.alert(id).unwrap().state(), AlertState::Rectified);
                    }
                    AlertChange::Acknowledged(_) => {
                        assert_eq!(alerts.alert(id).unwrap().state(), AlertState::Acknowledged);
                    }
                    _ => {}
                }
            }
            assert!(alerts.len() <= MAX_ALERTS);
            // Ordered, one alert per kind.
            let standing = alerts.alerts();
            for pair in standing.windows(2) {
                assert!(pair[0].priority() >= pair[1].priority());
            }
            for (index, alert) in standing.iter().enumerate() {
                assert!(standing
                    .iter()
                    .skip(index + 1)
                    .all(|other| other.kind() != alert.kind()));
                assert!(alert.occurrences() >= 1);
                assert_eq!(
                    Some(alert.priority()),
                    StandardPolicy::default().classify(&alert.kind())
                );
            }
        }
        let _ = alerts.tick(now);
        if round % 2 == 1 {
            let _ = alerts.acknowledge_all(now);
            assert_eq!(alerts.unacknowledged().count(), 0);
        }
    }
    assert!(raised > 0);
    assert!(alerts
        .highest()
        .map_or(true, |alert| alert.priority() >= AlertPriority::Caution));
}

#[cfg(feature = "serde")]
#[test]
fn the_value_types_round_trip() {
    let mut alerts = AlertManager::new(StandardPolicy::default());
    let now = Instant::<Utc>::from_unix_seconds(1_789_000_000);
    let mut events = EventList::<TrafficEvent>::new();
    events.push(TrafficEvent::CpaAlarm {
        target: TargetId::new(7),
        cpa: Distance::from_cables(3.0).unwrap(),
        tcpa: Duration::from_secs(600),
        at: now,
    });
    let _ = alerts.ingest(&events, now);
    let alert = alerts.alerts()[0];
    let text = serde_json::to_string(&alert).unwrap();
    let back: kinavis_alerts::Alert = serde_json::from_str(&text).unwrap();
    assert_eq!(back, alert);
    let policy: StandardPolicy =
        serde_json::from_str(&serde_json::to_string(&StandardPolicy::default()).unwrap()).unwrap();
    assert_eq!(policy, StandardPolicy::default());
}
