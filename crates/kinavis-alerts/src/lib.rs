//! Bridge alert management (BAM): aggregation, prioritisation and
//! acknowledgement of conditions reported as events.
//!
//! Domain operations report a condition on every evaluation that finds it
//! (guidance reports XTE exceeded on each call, traffic raises a CPA alarm on
//! each assessment). [`AlertManager`] turns that stream into one alert per
//! condition, with a priority, acknowledgement and clearance.
//!
//! # Alert lifecycle
//!
//! An event declares its condition via [`Reportable`]; the policy assigns a
//! priority. A classified condition raises an alert keyed by [`AlertKind`]
//! (condition + target, source or sensor). If an alert of that kind is already
//! standing, it is repeated: its count increments and nothing new is
//! annunciated. States follow IMO BAM:
//!
//! - [`AlertState::Active`] — raised, not acknowledged;
//! - [`AlertState::Acknowledged`] — acknowledged, condition present;
//! - [`AlertState::Rectified`] — condition gone, not yet acknowledged;
//! - normal — gone and acknowledged; removed from the list.
//!
//! A condition ends either by an ending event (fix acquired ends fix lost, a
//! healthy sensor ends its suspicion, target lost ends its CPA alarm) or by
//! silence: an alert not repeated within [`AlertPolicy::rectify_after`] is
//! rectified on the next [`AlertManager::tick`].
//!
//! # Policy
//!
//! Priority is the vessel's decision: [`AlertPolicy`] is the port,
//! [`StandardPolicy`] a default. XTE exceeded may be a warning in open water
//! and an alarm in a channel. What counts as a condition is the domain's:
//! [`Reportable`] is implemented for every event type in the workspace;
//! applications implement it for their own event unions.
//!
//! ```rust
//! use kinavis_alerts::{AlertChange, AlertManager, AlertPriority, AlertState, StandardPolicy};
//! use kinavis_kernel::{Distance, EventList, Instant, TargetId, Utc};
//! use kinavis_traffic::TrafficEvent;
//! use core::time::Duration;
//!
//! let mut alerts = AlertManager::new(StandardPolicy::default());
//! let start = Instant::<Utc>::from_unix_seconds(1_789_000_000);
//!
//! // Every assessment reports the same target too close; one alert stands.
//! for second in 0..3 {
//!     let now = start.checked_add(Duration::from_secs(second)).unwrap();
//!     let mut events = EventList::<TrafficEvent>::new();
//!     events.push(TrafficEvent::CpaAlarm {
//!         target: TargetId::new(7),
//!         cpa: Distance::from_cables(3.0)?,
//!         tcpa: Duration::from_secs(600),
//!         at: now,
//!     });
//!     let changes = alerts.ingest(&events, now);
//!     if second == 0 {
//!         assert!(matches!(changes[0], AlertChange::Raised(_)));
//!     } else {
//!         assert!(changes.is_empty());
//!     }
//! }
//! assert_eq!(alerts.len(), 1);
//! let alarm = alerts.alerts()[0];
//! assert_eq!(alarm.priority(), AlertPriority::Alarm);
//! assert_eq!(alarm.occurrences(), 3);
//!
//! // Acknowledged, it stands quietly; when the target is lost it is over.
//! alerts.acknowledge(alarm.id(), start)?;
//! assert_eq!(alerts.alerts()[0].state(), AlertState::Acknowledged);
//! let mut events = EventList::<TrafficEvent>::new();
//! events.push(TrafficEvent::TargetLost { target: TargetId::new(7), last_seen: start });
//! let changes = alerts.ingest(&events, start.checked_add(Duration::from_secs(60)).unwrap());
//! assert!(changes.iter().any(|change| matches!(change, AlertChange::Cleared(_))));
//! // The lost target is itself a caution, standing on its own.
//! assert_eq!(alerts.len(), 1);
//! assert_eq!(alerts.alerts()[0].priority(), AlertPriority::Caution);
//! # Ok::<(), kinavis_kernel::KernelError>(())
//! ```
//!
//! # Feature flags
//!
//! - `std` *(default)* — standard library maths in the kernel.
//! - `libm` — for `no_std` targets: `--no-default-features --features libm`.
//! - `serde` — serialisation of the value types.
//!
//! No allocation: the manager holds at most [`MAX_ALERTS`] alerts inline and
//! reports overflow instead of dropping silently.

#![cfg_attr(not(feature = "std"), no_std)]

// The crate does not allocate; tests use `format!`.
#[cfg(test)]
extern crate alloc;

mod manager;
mod policy;
mod reportable;

use core::fmt;

use kinavis_kernel::event::{NavigationIntegrity, PositionSource, SensorId, TargetId};
use kinavis_kernel::time::{Instant, Utc};

pub use manager::{AlertChange, AlertChanges, AlertManager, MAX_ALERTS, MAX_CHANGES};
pub use policy::{AlertPolicy, StandardPolicy};
pub use reportable::Reportable;

/// Runs the `README.md` example as a doctest.
#[cfg(doctest)]
#[doc = include_str!("../README.md")]
pub struct ReadmeExamples;

/// Alert priority per BAM, in ascending order.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
#[non_exhaustive]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum AlertPriority {
    /// Awareness; no immediate attention required.
    Caution,
    /// Attention required to prevent a hazard.
    Warning,
    /// Immediate attention and action required.
    Alarm,
    /// Immediate danger to life or the ship.
    EmergencyAlarm,
}

impl fmt::Display for AlertPriority {
    /// Formats as `alarm`, `warning`, `caution`, `emergency alarm`.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::Caution => "caution",
            Self::Warning => "warning",
            Self::Alarm => "alarm",
            Self::EmergencyAlarm => "emergency alarm",
        })
    }
}

/// Alert state.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum AlertState {
    /// Raised, not acknowledged; annunciating.
    Active,
    /// Acknowledged; condition present.
    Acknowledged,
    /// Condition gone; removed once acknowledged.
    Rectified,
}

/// Alert key: condition and subject.
///
/// Events of the same kind about the same subject aggregate into one alert.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum AlertKind {
    /// Cross-track error beyond the guidance limit.
    CrossTrackExceeded,
    /// Under-keel clearance below policy.
    UnderKeelClearanceLow,
    /// Outside the swinging circle.
    AnchorDragging,
    /// Target CPA/TCPA inside the limits.
    Cpa {
        /// Target.
        target: TargetId,
    },
    /// Target removed from the picture.
    TargetLost {
        /// Target.
        target: TargetId,
    },
    /// Position source stopped delivering.
    FixLost {
        /// Source.
        source: PositionSource,
    },
    /// Navigation integrity below nominal.
    IntegrityDegraded {
        /// Integrity level.
        to: NavigationIntegrity,
    },
    /// Sensor observations are suspect.
    SensorSuspect {
        /// Sensor.
        sensor: SensorId,
    },
    /// An observation was rejected.
    ObservationRejected,
}

/// Conditions an event ends.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum Ended {
    /// None.
    Nothing,
    /// One condition.
    One(AlertKind),
    /// Every integrity degradation: an integrity change supersedes the standing
    /// degradation and raises its own if it is one.
    EveryIntegrityDegradation,
}

/// Alert identifier, unique while the alert stands.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct AlertId(u32);

impl AlertId {
    /// Numeric value.
    #[must_use]
    pub const fn number(self) -> u32 {
        self.0
    }
}

impl fmt::Display for AlertId {
    /// Formats as `A17`.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "A{}", self.0)
    }
}

/// Standing alert.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Alert {
    id: AlertId,
    kind: AlertKind,
    priority: AlertPriority,
    state: AlertState,
    raised_at: Instant<Utc>,
    last_reported_at: Instant<Utc>,
    occurrences: u32,
}

impl Alert {
    /// Identifier.
    #[must_use]
    pub const fn id(&self) -> AlertId {
        self.id
    }

    /// Key.
    #[must_use]
    pub const fn kind(&self) -> AlertKind {
        self.kind
    }

    /// Priority.
    #[must_use]
    pub const fn priority(&self) -> AlertPriority {
        self.priority
    }

    /// State.
    #[must_use]
    pub const fn state(&self) -> AlertState {
        self.state
    }

    /// Time raised.
    #[must_use]
    pub const fn raised_at(&self) -> Instant<Utc> {
        self.raised_at
    }

    /// Time the condition was last reported.
    #[must_use]
    pub const fn last_reported_at(&self) -> Instant<Utc> {
        self.last_reported_at
    }

    /// Number of reports, the first included.
    #[must_use]
    pub const fn occurrences(&self) -> u32 {
        self.occurrences
    }
}
