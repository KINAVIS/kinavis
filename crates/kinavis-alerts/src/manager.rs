//! Alert manager: standing alerts and their state transitions.

use core::ops::Deref;

use kinavis_kernel::inline::Inline;
use kinavis_kernel::time::{Instant, Utc};
use kinavis_kernel::{KernelError, Result};

use crate::{Alert, AlertId, AlertKind, AlertPolicy, AlertPriority, AlertState, Ended, Reportable};

/// Maximum number of standing alerts.
///
/// More than a watch can handle; reaching it indicates a policy that is too
/// loud. On a full board, a new alert outranking the lowest-priority standing
/// one replaces it ([`AlertChange::Dropped`]); otherwise it is not raised and
/// [`AlertChanges::lost`] is set. Either way the overflow is reported and the
/// most pressing alerts are kept.
pub const MAX_ALERTS: usize = 32;

/// Maximum number of changes reported per call.
pub const MAX_CHANGES: usize = 16;

/// Alert state change.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum AlertChange {
    /// New alert, to be annunciated.
    Raised(AlertId),
    /// Standing alert acknowledged.
    Acknowledged(AlertId),
    /// Condition of an unacknowledged alert ended; awaiting acknowledgement.
    Rectified(AlertId),
    /// Alert removed: acknowledged and condition ended, in either order.
    Cleared(AlertId),
    /// Alert removed with its condition still present, to make room for a
    /// higher-priority alert. A later report raises it anew.
    Dropped(AlertId),
}

/// Changes made by one call, in order.
///
/// Fixed capacity, like [`EventList`](kinavis_kernel::event::EventList);
/// dereferences to a slice.
#[must_use = "an unread change is an alert nobody annunciated"]
#[derive(Debug, Clone, Copy)]
pub struct AlertChanges {
    changes: Inline<AlertChange, MAX_CHANGES>,
    overflowed: bool,
    lost: bool,
}

impl AlertChanges {
    const fn new() -> Self {
        Self {
            changes: Inline::new(AlertChange::Raised(AlertId(0))),
            overflowed: false,
            lost: false,
        }
    }

    fn push(&mut self, change: AlertChange) {
        if self.changes.push(change).is_err() {
            self.overflowed = true;
        }
    }

    /// Whether a change did not fit in this list. Alert state is correct; only
    /// the report is incomplete.
    #[must_use]
    pub const fn overflowed(&self) -> bool {
        self.overflowed
    }

    /// Whether an alert could not be raised because [`MAX_ALERTS`] were
    /// standing. Requires action in itself.
    #[must_use]
    pub const fn lost(&self) -> bool {
        self.lost
    }
}

impl Deref for AlertChanges {
    type Target = [AlertChange];

    fn deref(&self) -> &[AlertChange] {
        &self.changes
    }
}

/// Standing alerts under a policy.
///
/// Stores up to [`MAX_ALERTS`] alerts inline, ordered by priority then by time
/// raised, so [`AlertManager::alerts`] starts with the most pressing. Several
/// kilobytes: keep it behind a reference or in a `static`. Deliberately not
/// `Copy`, so the board cannot be duplicated by accident:
///
/// ```compile_fail
/// fn is_copy<T: Copy>() {}
/// is_copy::<kinavis_alerts::AlertManager<kinavis_alerts::StandardPolicy>>();
/// ```
#[derive(Debug, Clone)]
pub struct AlertManager<P: AlertPolicy> {
    policy: P,
    alerts: Inline<Alert, MAX_ALERTS>,
    next: u32,
}

impl<P: AlertPolicy> AlertManager<P> {
    /// Empty board under `policy`.
    pub fn new(policy: P) -> Self {
        Self {
            policy,
            alerts: Inline::new(Alert {
                id: AlertId(0),
                kind: AlertKind::ObservationRejected,
                priority: AlertPriority::Caution,
                state: AlertState::Active,
                raised_at: Instant::UNIX_EPOCH,
                last_reported_at: Instant::UNIX_EPOCH,
                occurrences: 0,
            }),
            next: 1,
        }
    }

    /// Policy.
    pub const fn policy(&self) -> &P {
        &self.policy
    }

    /// Standing alerts, most pressing first (priority, then time raised).
    #[must_use]
    pub fn alerts(&self) -> &[Alert] {
        &self.alerts
    }

    /// Number of standing alerts.
    #[must_use]
    pub const fn len(&self) -> usize {
        self.alerts.len()
    }

    /// Whether no alert is standing.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.alerts.is_empty()
    }

    /// Alert by identifier, if standing.
    #[must_use]
    pub fn alert(&self, id: AlertId) -> Option<&Alert> {
        self.alerts.iter().find(|alert| alert.id == id)
    }

    /// Unacknowledged alerts, most pressing first.
    pub fn unacknowledged(&self) -> impl Iterator<Item = &Alert> + '_ {
        self.alerts
            .iter()
            .filter(|alert| alert.state != AlertState::Acknowledged)
    }

    /// Most pressing standing alert, if any.
    #[must_use]
    pub fn highest(&self) -> Option<&Alert> {
        self.alerts.first()
    }

    /// Processes the events of one operation at `now`.
    ///
    /// Each event first ends the conditions it ends, then raises or repeats its
    /// own alert; conditions not reported within `rectify_after` are then
    /// rectified as in [`AlertManager::tick`]. Accepts a slice of any
    /// [`Reportable`] type — any
    /// [`EventList`](kinavis_kernel::event::EventList) dereferences to one.
    /// Typically one call per source list: estimator, guidance, traffic.
    pub fn ingest<E: Reportable>(&mut self, events: &[E], now: Instant<Utc>) -> AlertChanges {
        let mut changes = AlertChanges::new();
        for event in events {
            self.end(event.ends(), &mut changes);
            if let Some(kind) = event.condition() {
                if let Some(priority) = self.policy.classify(&kind) {
                    self.raise_or_repeat(kind, priority, now, &mut changes);
                }
            }
        }
        self.rectify_silent(now, &mut changes);
        changes
    }

    /// Advances time: conditions not reported within `rectify_after` are
    /// rectified — removed if acknowledged, otherwise left as
    /// [`AlertState::Rectified`].
    pub fn tick(&mut self, now: Instant<Utc>) -> AlertChanges {
        let mut changes = AlertChanges::new();
        self.rectify_silent(now, &mut changes);
        changes
    }

    /// Acknowledges an alert: active becomes acknowledged, rectified is
    /// removed.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] if no alert has that identifier.
    pub fn acknowledge(&mut self, id: AlertId, at: Instant<Utc>) -> Result<AlertChanges> {
        let mut changes = AlertChanges::new();
        let index =
            self.alerts
                .iter()
                .position(|alert| alert.id == id)
                .ok_or(KernelError::OutOfRange {
                    parameter: "alert id",
                    value: f64::from(id.0),
                    min: 1.0,
                    max: f64::from(self.next.saturating_sub(1)),
                })?;
        self.acknowledge_at(index, at, &mut changes);
        Ok(changes)
    }

    /// Acknowledges every standing alert.
    pub fn acknowledge_all(&mut self, at: Instant<Utc>) -> AlertChanges {
        let mut changes = AlertChanges::new();
        // Removal shifts later entries down; iterate from the end.
        let mut index = self.alerts.len();
        while index > 0 {
            index -= 1;
            self.acknowledge_at(index, at, &mut changes);
        }
        changes
    }

    fn acknowledge_at(&mut self, index: usize, _at: Instant<Utc>, changes: &mut AlertChanges) {
        let Some(alert) = self.alerts.get(index).copied() else {
            return;
        };
        match alert.state {
            AlertState::Active => {
                if let Some(slot) = self.alerts.as_mut_slice().get_mut(index) {
                    slot.state = AlertState::Acknowledged;
                }
                changes.push(AlertChange::Acknowledged(alert.id));
            }
            AlertState::Rectified => {
                self.remove(index);
                changes.push(AlertChange::Cleared(alert.id));
            }
            AlertState::Acknowledged => {}
        }
    }

    /// Raises an alert of `kind`, or repeats the standing one.
    fn raise_or_repeat(
        &mut self,
        kind: AlertKind,
        priority: AlertPriority,
        now: Instant<Utc>,
        changes: &mut AlertChanges,
    ) {
        if let Some(standing) = self
            .alerts
            .as_mut_slice()
            .iter_mut()
            .find(|alert| alert.kind == kind)
        {
            standing.occurrences = standing.occurrences.saturating_add(1);
            standing.last_reported_at = now;
            // A rectified condition that recurs becomes active; an acknowledged
            // one stays acknowledged.
            if standing.state == AlertState::Rectified {
                standing.state = AlertState::Active;
                changes.push(AlertChange::Raised(standing.id));
            }
            return;
        }

        let alert = Alert {
            id: AlertId(self.next),
            kind,
            priority,
            state: AlertState::Active,
            raised_at: now,
            last_reported_at: now,
            occurrences: 1,
        };
        // Insert after every alert of higher or equal priority.
        let index = self
            .alerts
            .iter()
            .position(|standing| standing.priority < priority)
            .unwrap_or(self.alerts.len());
        if self.alerts.len() >= MAX_ALERTS {
            // Board full. The last entry is the least pressing; a newcomer that
            // outranks it replaces it, otherwise the newcomer is lost.
            if index >= self.alerts.len() {
                changes.lost = true;
                return;
            }
            if let Some(dropped) = self.alerts.remove(self.alerts.len().saturating_sub(1)) {
                changes.push(AlertChange::Dropped(dropped.id));
            }
        }
        if self.alerts.insert(index, alert).is_err() {
            changes.lost = true;
            return;
        }
        self.next = self.next.wrapping_add(1).max(1);
        changes.push(AlertChange::Raised(alert.id));
    }

    /// Applies the endings of an event.
    fn end(&mut self, ended: Ended, changes: &mut AlertChanges) {
        let mut index = self.alerts.len();
        while index > 0 {
            index -= 1;
            let Some(alert) = self.alerts.get(index).copied() else {
                continue;
            };
            let ends = match ended {
                Ended::One(kind) => alert.kind == kind,
                Ended::EveryIntegrityDegradation => {
                    matches!(alert.kind, AlertKind::IntegrityDegraded { .. })
                }
                Ended::Nothing => false,
            };
            if ends {
                self.rectify_at(index, changes);
            }
        }
    }

    /// Rectifies every alert whose condition has gone silent.
    fn rectify_silent(&mut self, now: Instant<Utc>, changes: &mut AlertChanges) {
        let limit = self.policy.rectify_after();
        let mut index = self.alerts.len();
        while index > 0 {
            index -= 1;
            let Some(alert) = self.alerts.get(index).copied() else {
                continue;
            };
            if alert.state == AlertState::Rectified {
                continue;
            }
            let silent = now
                .checked_duration_since(alert.last_reported_at)
                .is_some_and(|silence| silence > limit);
            if silent {
                self.rectify_at(index, changes);
            }
        }
    }

    /// Rectifies one alert: removed if acknowledged, otherwise left as
    /// rectified.
    fn rectify_at(&mut self, index: usize, changes: &mut AlertChanges) {
        let Some(alert) = self.alerts.get(index).copied() else {
            return;
        };
        match alert.state {
            AlertState::Acknowledged => {
                self.remove(index);
                changes.push(AlertChange::Cleared(alert.id));
            }
            AlertState::Active => {
                if let Some(slot) = self.alerts.as_mut_slice().get_mut(index) {
                    slot.state = AlertState::Rectified;
                }
                changes.push(AlertChange::Rectified(alert.id));
            }
            AlertState::Rectified => {}
        }
    }

    /// Removes the alert at `index`, preserving order.
    fn remove(&mut self, index: usize) {
        let Some(first) = self.alerts.first().copied() else {
            return;
        };
        let mut kept = Inline::<Alert, MAX_ALERTS>::new(first);
        for (position, alert) in self.alerts.iter().enumerate() {
            if position != index {
                // The new store has the capacity of the old one.
                let _ = kept.push(*alert);
            }
        }
        self.alerts = kept;
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::indexing_slicing)]
mod tests {
    use super::*;
    use crate::StandardPolicy;
    use core::time::Duration;
    use kinavis::event::GuidanceEvent;
    use kinavis_kernel::event::{
        Event, EventList, NavigationEvent, NavigationIntegrity, PositionSource, SensorHealth,
        SensorId, TargetId,
    };
    use kinavis_kernel::units::Distance;
    use kinavis_traffic::TrafficEvent;

    fn start() -> Instant<Utc> {
        Instant::from_unix_seconds(1_789_000_000)
    }

    fn after(seconds: u64) -> Instant<Utc> {
        start().checked_add(Duration::from_secs(seconds)).unwrap()
    }

    fn manager() -> AlertManager<StandardPolicy> {
        AlertManager::new(StandardPolicy::default())
    }

    fn one<E: Event>(event: E) -> EventList<E> {
        let mut events = EventList::new();
        events.push(event);
        events
    }

    fn off_track(at: Instant<Utc>) -> GuidanceEvent {
        GuidanceEvent::CrossTrackExceeded {
            error: Distance::from_cables(7.0).unwrap(),
            limit: Distance::from_cables(5.0).unwrap(),
            at,
        }
    }

    fn cpa(target: u32, at: Instant<Utc>) -> TrafficEvent {
        TrafficEvent::CpaAlarm {
            target: TargetId::new(target),
            cpa: Distance::from_cables(3.0).unwrap(),
            tcpa: Duration::from_secs(600),
            at,
        }
    }

    fn fix_lost(at: Instant<Utc>) -> NavigationEvent {
        NavigationEvent::FixLost {
            source: PositionSource::Gnss,
            at,
            last_good: at,
        }
    }

    #[test]
    fn a_condition_reported_many_times_is_one_alert() {
        let mut alerts = manager();
        let first = alerts.ingest(&one(off_track(start())), start());
        assert_eq!(first.len(), 1);
        assert!(matches!(first[0], AlertChange::Raised(id) if id.number() == 1));

        for second in 1..10 {
            let changes = alerts.ingest(&one(off_track(after(second))), after(second));
            assert!(changes.is_empty());
        }
        assert_eq!(alerts.len(), 1);
        let alert = alerts.alerts()[0];
        assert_eq!(alert.occurrences(), 10);
        assert_eq!(alert.raised_at(), start());
        assert_eq!(alert.last_reported_at(), after(9));
        assert_eq!(alert.kind(), AlertKind::CrossTrackExceeded);
        assert_eq!(alert.priority(), AlertPriority::Warning);
        assert_eq!(alert.state(), AlertState::Active);
        assert_eq!(alloc::format!("{}", alert.id()), "A1");
        assert_eq!(alloc::format!("{}", alert.priority()), "warning");
    }

    #[test]
    fn the_alerts_stand_in_order_of_priority_then_of_raising() {
        let mut alerts = manager();
        let _ = alerts.ingest(&one(off_track(start())), start());
        let _ = alerts.ingest(
            &one(TrafficEvent::TargetLost {
                target: TargetId::new(3),
                last_seen: start(),
            }),
            after(1),
        );
        let _ = alerts.ingest(&one(cpa(7, after(2))), after(2));
        let _ = alerts.ingest(&one(cpa(8, after(3))), after(3));
        let _ = alerts.ingest(&one(fix_lost(after(4))), after(4));

        let order = alerts
            .alerts()
            .iter()
            .map(|alert| (alert.priority(), alert.id().number()))
            .collect::<alloc::vec::Vec<_>>();
        assert_eq!(
            order,
            [
                (AlertPriority::Alarm, 3),
                (AlertPriority::Alarm, 4),
                (AlertPriority::Warning, 1),
                (AlertPriority::Warning, 5),
                (AlertPriority::Caution, 2),
            ]
        );
        assert_eq!(alerts.highest().unwrap().id().number(), 3);
        assert_eq!(alerts.unacknowledged().count(), 5);
        assert!(alerts.alert(AlertId(4)).is_some());
        assert!(alerts.alert(AlertId(9)).is_none());
    }

    #[test]
    fn acknowledged_it_stands_quietly_and_goes_when_the_condition_does() {
        let mut alerts = manager();
        let _ = alerts.ingest(&one(off_track(start())), start());
        let id = alerts.alerts()[0].id();

        let changes = alerts.acknowledge(id, after(1)).unwrap();
        assert!(matches!(changes[0], AlertChange::Acknowledged(acked) if acked == id));
        assert_eq!(alerts.alerts()[0].state(), AlertState::Acknowledged);
        assert_eq!(alerts.unacknowledged().count(), 0);

        // Still reported: stays acknowledged, no new annunciation.
        let changes = alerts.ingest(&one(off_track(after(5))), after(5));
        assert!(changes.is_empty());
        assert_eq!(alerts.alerts()[0].state(), AlertState::Acknowledged);

        // Silent beyond the policy limit: rectified and removed.
        let changes = alerts.tick(after(5 + 31));
        assert!(matches!(changes[0], AlertChange::Cleared(cleared) if cleared == id));
        assert!(alerts.is_empty());
    }

    #[test]
    fn unacknowledged_it_is_rectified_and_waits_and_comes_back_if_reported_again() {
        let mut alerts = manager();
        let _ = alerts.ingest(&one(off_track(start())), start());
        let id = alerts.alerts()[0].id();

        // Exactly at the 30 s limit: not yet rectified.
        assert!(alerts.tick(after(30)).is_empty());
        let changes = alerts.tick(after(31));
        assert!(matches!(changes[0], AlertChange::Rectified(rectified) if rectified == id));
        assert_eq!(alerts.alerts()[0].state(), AlertState::Rectified);
        assert_eq!(alerts.unacknowledged().count(), 1);

        // Reported again: active again, same identifier.
        let changes = alerts.ingest(&one(off_track(after(40))), after(40));
        assert!(matches!(changes[0], AlertChange::Raised(raised) if raised == id));
        assert_eq!(alerts.alerts()[0].state(), AlertState::Active);

        // Rectified again, then acknowledged: removed.
        let _ = alerts.tick(after(100));
        let changes = alerts.acknowledge(id, after(101)).unwrap();
        assert!(matches!(changes[0], AlertChange::Cleared(cleared) if cleared == id));
        assert!(alerts.is_empty());
    }

    #[test]
    // Four conditions raised and ended, each checked.
    #[allow(clippy::too_many_lines)]
    fn an_event_ends_the_condition_it_is_the_end_of() {
        let mut alerts = manager();
        let _ = alerts.ingest(&one(fix_lost(start())), start());
        let _ = alerts.ingest(&one(cpa(7, start())), start());
        let _ = alerts.ingest(
            &one(NavigationEvent::SensorHealthChanged {
                sensor: SensorId::named("GNSS 1"),
                from: SensorHealth::Healthy,
                to: SensorHealth::Suspect,
                at: start(),
            }),
            start(),
        );
        let _ = alerts.ingest(
            &one(NavigationEvent::IntegrityChanged {
                from: NavigationIntegrity::Nominal,
                to: NavigationIntegrity::DeadReckoning,
                at: start(),
            }),
            start(),
        );
        assert_eq!(alerts.len(), 4);

        let mut ends = EventList::new();
        ends.push(NavigationEvent::FixAcquired {
            source: PositionSource::Gnss,
            at: after(1),
        });
        ends.push(NavigationEvent::SensorHealthChanged {
            sensor: SensorId::named("GNSS 1"),
            from: SensorHealth::Suspect,
            to: SensorHealth::Healthy,
            at: after(1),
        });
        ends.push(NavigationEvent::IntegrityChanged {
            from: NavigationIntegrity::DeadReckoning,
            to: NavigationIntegrity::Nominal,
            at: after(1),
        });
        let mut changes = alerts.ingest(&ends, after(1));
        let lost = alerts.ingest(
            &one(TrafficEvent::TargetLost {
                target: TargetId::new(7),
                last_seen: after(1),
            }),
            after(1),
        );
        for change in lost.iter() {
            changes.push(*change);
        }
        assert_eq!(
            changes
                .iter()
                .filter(|change| matches!(change, AlertChange::Rectified(_)))
                .count(),
            4
        );
        // Target lost is its own caution; the others stand rectified.
        assert!(changes
            .iter()
            .any(|change| matches!(change, AlertChange::Raised(_))));
        assert_eq!(alerts.len(), 5);
        assert_eq!(
            alerts
                .alerts()
                .iter()
                .filter(|alert| alert.state() == AlertState::Rectified)
                .count(),
            4
        );

        // An integrity degradation supersedes the previous one.
        let _ = alerts.acknowledge_all(after(2));
        assert_eq!(alerts.len(), 1);
        let _ = alerts.ingest(
            &one(NavigationEvent::IntegrityChanged {
                from: NavigationIntegrity::Nominal,
                to: NavigationIntegrity::DeadReckoning,
                at: after(3),
            }),
            after(3),
        );
        let _ = alerts.ingest(
            &one(NavigationEvent::IntegrityChanged {
                from: NavigationIntegrity::DeadReckoning,
                to: NavigationIntegrity::Exceeded,
                at: after(4),
            }),
            after(4),
        );
        let degradations = alerts
            .alerts()
            .iter()
            .filter(|alert| matches!(alert.kind(), AlertKind::IntegrityDegraded { .. }))
            .collect::<alloc::vec::Vec<_>>();
        assert_eq!(degradations.len(), 2);
        assert!(degradations
            .iter()
            .any(|alert| alert.state() == AlertState::Rectified
                && alert.kind()
                    == AlertKind::IntegrityDegraded {
                        to: NavigationIntegrity::DeadReckoning
                    }));
        assert!(degradations
            .iter()
            .any(|alert| alert.state() == AlertState::Active
                && alert.priority() == AlertPriority::Warning));
    }

    #[test]
    fn news_is_not_an_alert() {
        let mut alerts = manager();
        let reached = GuidanceEvent::WaypointReached {
            index: 2,
            at: start(),
        };
        let mut events = EventList::new();
        events.push(NavigationEvent::FixAcquired {
            source: PositionSource::Gnss,
            at: start(),
        });
        events.push(NavigationEvent::ObservationRejected {
            reason: kinavis_kernel::event::RejectionReason::Stale,
            at: start(),
        });
        let acquired = TrafficEvent::TargetAcquired {
            target: TargetId::new(1),
            at: start(),
        };
        assert!(alerts.ingest(&one(reached), start()).is_empty());
        assert!(alerts.ingest(&events, start()).is_empty());
        assert!(alerts.ingest(&one(acquired), start()).is_empty());
        assert!(alerts.is_empty());
        assert_eq!(reached.condition(), None);
        assert_eq!(acquired.condition(), None);
        assert_eq!(events[1].condition(), Some(AlertKind::ObservationRejected));
    }

    #[test]
    fn acknowledging_what_does_not_stand_is_refused() {
        let mut alerts = manager();
        assert!(matches!(
            alerts.acknowledge(AlertId(1), start()).unwrap_err(),
            KernelError::OutOfRange {
                parameter: "alert id",
                ..
            }
        ));
        let _ = alerts.ingest(&one(off_track(start())), start());
        assert!(alerts.acknowledge(AlertId(2), start()).is_err());
        assert!(alerts.acknowledge(AlertId(1), start()).is_ok());
        // Repeated acknowledgement is a no-op.
        assert!(alerts.acknowledge(AlertId(1), start()).unwrap().is_empty());
    }

    #[test]
    fn a_full_board_loses_nothing_in_silence() {
        let mut alerts = manager();
        for target in 0..MAX_ALERTS {
            let changes =
                alerts.ingest(&one(cpa(u32::try_from(target).unwrap(), start())), start());
            assert!(!changes.lost());
        }
        assert_eq!(alerts.len(), MAX_ALERTS);
        // Board full of alarms: an equal-priority alarm is lost and reported.
        let changes = alerts.ingest(&one(cpa(u32::MAX, start())), start());
        assert!(changes.lost());
        assert!(changes.is_empty());
        assert_eq!(alerts.len(), MAX_ALERTS);
        // A known condition still repeats.
        let changes = alerts.ingest(&one(cpa(3, after(1))), after(1));
        assert!(!changes.lost());

        // Clearing all produces more changes than the list holds; overflow is
        // reported.
        let changes = alerts.acknowledge_all(after(2));
        assert!(changes.overflowed());
        assert_eq!(changes.len(), MAX_CHANGES);
        assert_eq!(alerts.unacknowledged().count(), 0);
    }

    #[test]
    fn a_full_board_makes_room_for_an_alert_that_outranks_its_least() {
        let mut alerts = manager();
        // 31 alarms and one caution: board full.
        for target in 0..MAX_ALERTS - 1 {
            let _ = alerts.ingest(&one(cpa(u32::try_from(target).unwrap(), start())), start());
        }
        let lost = TrafficEvent::TargetLost {
            target: TargetId::new(500),
            last_seen: start(),
        };
        let caution = alerts.ingest(&one(lost), start());
        let caution_id = match caution[0] {
            AlertChange::Raised(id) => id,
            other => unreachable!("the caution was raised, not {other:?}"),
        };
        assert_eq!(alerts.len(), MAX_ALERTS);
        // A warning outranks the caution and replaces it.
        let changes = alerts.ingest(&one(fix_lost(after(1))), after(1));
        assert!(!changes.lost());
        assert_eq!(changes[0], AlertChange::Dropped(caution_id));
        assert!(matches!(changes[1], AlertChange::Raised(_)));
        assert_eq!(alerts.len(), MAX_ALERTS);
        assert!(alerts
            .alerts()
            .iter()
            .all(|alert| alert.priority() >= AlertPriority::Warning));
        // Another caution has nothing to outrank and is lost.
        let changes = alerts.ingest(&one(lost), after(2));
        assert!(changes.lost());
    }
}
