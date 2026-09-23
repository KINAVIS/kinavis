//! Alert policy: which conditions raise alerts, and at what priority.

use core::time::Duration;

use kinavis_kernel::event::NavigationIntegrity;

use crate::{AlertKind, AlertPriority};

/// Priority assignment: a vessel decision, not a domain one.
///
/// The domain reports conditions via [`Reportable`](crate::Reportable); the
/// policy prioritises them. As in BAM, priority belongs to the condition: a CPA
/// alarm is always an alarm, not an alarm at 3 cables and a warning at 5.
pub trait AlertPolicy {
    /// Priority of an alert for `condition`, or `None` if it raises no alert.
    fn classify(&self, condition: &AlertKind) -> Option<AlertPriority>;

    /// Time without reports after which a condition is considered ended.
    /// Conditions are reported on every evaluation, so a few evaluation
    /// periods.
    fn rectify_after(&self) -> Duration;
}

/// Default policy for a vessel in open water.
///
/// Alarm: target too close, insufficient UKC, anchor dragging. Warning: off
/// track, position source lost, integrity exceeded. Caution: dead reckoning,
/// suspect sensor, target lost. Rejected observations are not alerts; a series
/// of them surfaces as a suspect sensor.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct StandardPolicy {
    /// Time without reports after which a condition has ended.
    pub rectify_after: Duration,
}

impl Default for StandardPolicy {
    /// 30 s of silence ends a condition.
    fn default() -> Self {
        Self {
            rectify_after: Duration::from_secs(30),
        }
    }
}

impl AlertPolicy for StandardPolicy {
    fn classify(&self, condition: &AlertKind) -> Option<AlertPriority> {
        match condition {
            AlertKind::Cpa { .. }
            | AlertKind::UnderKeelClearanceLow
            | AlertKind::AnchorDragging => Some(AlertPriority::Alarm),
            AlertKind::CrossTrackExceeded | AlertKind::FixLost { .. } => {
                Some(AlertPriority::Warning)
            }
            AlertKind::IntegrityDegraded { to } => match to {
                NavigationIntegrity::Exceeded => Some(AlertPriority::Warning),
                NavigationIntegrity::DeadReckoning => Some(AlertPriority::Caution),
                _ => None,
            },
            AlertKind::SensorSuspect { .. } | AlertKind::TargetLost { .. } => {
                Some(AlertPriority::Caution)
            }
            _ => None,
        }
    }

    fn rectify_after(&self) -> Duration {
        self.rectify_after
    }
}
