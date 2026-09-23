//! Mapping from events to alert conditions.
//!
//! Each context reports in its own enum ([`NavigationEvent`],
//! [`GuidanceEvent`], [`TrafficEvent`]) and none knows about alerts.
//! [`Reportable`] maps one event to the condition it reports and the conditions
//! it ends; the board accepts any list of reportable events. Implementations
//! for the workspace's events are here; applications implement it for their own
//! event unions.

use kinavis::event::{AnchorEvent, ClearanceEvent, GuidanceEvent};
use kinavis_kernel::event::{Event, NavigationEvent, NavigationIntegrity, SensorHealth};
use kinavis_traffic::TrafficEvent;

use crate::{AlertKind, Ended};

/// Event accepted by the alert board: the condition it reports and the
/// conditions it ends.
///
/// Informational events (waypoint reached, fix acquired) report [`None`] but
/// may still end a condition.
pub trait Reportable: Event {
    /// Condition reported by the event, if any.
    fn condition(&self) -> Option<AlertKind>;

    /// Conditions ended by the event.
    fn ends(&self) -> Ended;
}

impl Reportable for NavigationEvent {
    /// Fix lost, integrity degradation, suspect sensor and rejected observation
    /// are conditions; fix acquired is informational.
    fn condition(&self) -> Option<AlertKind> {
        match self {
            Self::FixLost { source, .. } => Some(AlertKind::FixLost { source: *source }),
            Self::IntegrityChanged { to, .. } if *to != NavigationIntegrity::Nominal => {
                Some(AlertKind::IntegrityDegraded { to: *to })
            }
            Self::SensorHealthChanged { sensor, to, .. } if *to != SensorHealth::Healthy => {
                Some(AlertKind::SensorSuspect { sensor: *sensor })
            }
            Self::ObservationRejected { .. } => Some(AlertKind::ObservationRejected),
            _ => None,
        }
    }

    /// Fix acquired ends the loss of that source; a healthy sensor ends its
    /// suspicion; any integrity change ends every standing degradation and
    /// raises its own if it is one.
    fn ends(&self) -> Ended {
        match self {
            Self::FixAcquired { source, .. } => Ended::One(AlertKind::FixLost { source: *source }),
            Self::SensorHealthChanged { sensor, to, .. } if *to == SensorHealth::Healthy => {
                Ended::One(AlertKind::SensorSuspect { sensor: *sensor })
            }
            Self::IntegrityChanged { .. } => Ended::EveryIntegrityDegradation,
            _ => Ended::Nothing,
        }
    }
}

impl Reportable for GuidanceEvent {
    /// XTE exceeded is a condition; waypoint or wheel-over reached is
    /// informational.
    fn condition(&self) -> Option<AlertKind> {
        match self {
            Self::CrossTrackExceeded { .. } => Some(AlertKind::CrossTrackExceeded),
            _ => None,
        }
    }

    fn ends(&self) -> Ended {
        Ended::Nothing
    }
}

impl Reportable for ClearanceEvent {
    fn condition(&self) -> Option<AlertKind> {
        match self {
            Self::UnderKeelClearanceLow { .. } => Some(AlertKind::UnderKeelClearanceLow),
            _ => None,
        }
    }

    fn ends(&self) -> Ended {
        Ended::Nothing
    }
}

impl Reportable for AnchorEvent {
    fn condition(&self) -> Option<AlertKind> {
        match self {
            Self::AnchorDragging { .. } => Some(AlertKind::AnchorDragging),
            _ => None,
        }
    }

    fn ends(&self) -> Ended {
        Ended::Nothing
    }
}

impl Reportable for TrafficEvent {
    /// CPA alarm, target lost (silence or eviction) and rejected observation
    /// are conditions; target acquired is informational.
    fn condition(&self) -> Option<AlertKind> {
        match self {
            Self::CpaAlarm { target, .. } => Some(AlertKind::Cpa { target: *target }),
            Self::TargetLost { target, .. } | Self::TargetEvicted { target, .. } => {
                Some(AlertKind::TargetLost { target: *target })
            }
            Self::ObservationRejected { .. } => Some(AlertKind::ObservationRejected),
            _ => None,
        }
    }

    /// Target lost ends its CPA alarm; target acquired ends its loss.
    fn ends(&self) -> Ended {
        match self {
            Self::TargetLost { target, .. } | Self::TargetEvicted { target, .. } => {
                Ended::One(AlertKind::Cpa { target: *target })
            }
            Self::TargetAcquired { target, .. } => {
                Ended::One(AlertKind::TargetLost { target: *target })
            }
            _ => Ended::Nothing,
        }
    }
}
