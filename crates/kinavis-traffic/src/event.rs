//! Traffic picture events.
//!
//! Returned in an [`EventList`] of [`TrafficEvent`], as in the kernel's
//! [`event`](kinavis_kernel::event) module: no subscriptions, no allocation,
//! nothing dropped silently. The enum is this crate's own (the kernel has no
//! notion of a target track); an alert board or application that aggregates
//! contexts defines the union.

use core::time::Duration;

use kinavis_kernel::event::{Event, RejectionReason, TargetId};
use kinavis_kernel::time::{Instant, Utc};
use kinavis_kernel::units::Distance;

/// Traffic picture event.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum TrafficEvent {
    /// Target observed enough times to be tracked; from now on it has course,
    /// speed and an extrapolated position.
    TargetAcquired {
        /// Target.
        target: TargetId,
        /// Time of the observation that completed acquisition.
        at: Instant<Utc>,
    },
    /// Target not observed within the lost timeout; removed from the picture.
    TargetLost {
        /// Target.
        target: TargetId,
        /// Last observation time.
        last_seen: Instant<Utc>,
    },
    /// Least recently observed target removed from a full picture to admit a
    /// new one. Same meaning as [`TrafficEvent::TargetLost`] for observers,
    /// with the reason.
    TargetEvicted {
        /// Removed target.
        target: TargetId,
        /// Last observation time.
        last_seen: Instant<Utc>,
        /// Admitted target.
        for_target: TargetId,
    },
    /// Observation rejected.
    ObservationRejected {
        /// Target.
        target: TargetId,
        /// Reason.
        reason: RejectionReason,
        /// Time of the rejected observation.
        at: Instant<Utc>,
    },
    /// CPA within the CPA limit and TCPA within the TCPA limit.
    ///
    /// Reported on every assessment that finds it; the assessment is stateless.
    CpaAlarm {
        /// Target.
        target: TargetId,
        /// CPA.
        cpa: Distance,
        /// TCPA.
        tcpa: Duration,
        /// Time of the assessed picture.
        at: Instant<Utc>,
    },
}

impl TrafficEvent {
    /// Target the event refers to.
    #[must_use]
    pub const fn target(&self) -> TargetId {
        match self {
            Self::TargetAcquired { target, .. }
            | Self::TargetLost { target, .. }
            | Self::TargetEvicted { target, .. }
            | Self::ObservationRejected { target, .. }
            | Self::CpaAlarm { target, .. } => *target,
        }
    }
}

impl Event for TrafficEvent {
    const PLACEHOLDER: Self = Self::TargetAcquired {
        target: TargetId::new(0),
        at: Instant::UNIX_EPOCH,
    };

    fn at(&self) -> Instant<Utc> {
        match self {
            Self::TargetAcquired { at, .. }
            | Self::ObservationRejected { at, .. }
            | Self::CpaAlarm { at, .. } => *at,
            Self::TargetLost { last_seen, .. } | Self::TargetEvicted { last_seen, .. } => {
                *last_seen
            }
        }
    }
}
