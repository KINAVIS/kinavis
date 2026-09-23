//! Events of this crate's use cases.
//!
//! As in the kernel's [`event`](kinavis_kernel::event) module, events are
//! returned in a list, not published. Each context has its own enum — guidance,
//! anchor watch, under-keel clearance — returned in an [`EventList`] of that
//! type.
//!
//! Every enum is `#[non_exhaustive]`.

pub use kinavis_kernel::event::{
    Event, EventList, NavigationEvent, NavigationIntegrity, PositionSource, RejectionReason,
    SensorHealth, SensorId, TargetId, MAX_EVENTS, SENSOR_NAME_BYTES,
};

use kinavis_kernel::position::Position;
use kinavis_kernel::time::{Instant, Utc};
use kinavis_kernel::units::Distance;

/// Guidance events.
///
/// Guidance is stateless, so each event is reported on every evaluation that
/// finds the condition; the caller reacts (advances the leg, raises an alarm)
/// and the report stops or repeats accordingly.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum GuidanceEvent {
    /// Waypoint reached: inside the arrival circle, abeam or past it.
    ///
    /// The caller advances the active leg; legs are numbered by their start
    /// waypoint, so the next leg is `index`.
    WaypointReached {
        /// Waypoint index, zero-based.
        index: u16,
        /// Time of the position.
        at: Instant<Utc>,
    },
    /// Wheel-over point for the turn at the next waypoint reached. With turns
    /// planned, this is the cue to advance the active leg.
    WheelOverReached {
        /// Waypoint index of the turn, zero-based.
        index: u16,
        /// Wheel-over position.
        wheel_over_at: Position,
        /// Time of the position.
        at: Instant<Utc>,
    },
    /// Cross-track error beyond the limit.
    CrossTrackExceeded {
        /// Cross-track error.
        error: Distance,
        /// Limit exceeded.
        limit: Distance,
        /// Time of the position.
        at: Instant<Utc>,
    },
}

impl Event for GuidanceEvent {
    const PLACEHOLDER: Self = Self::WaypointReached {
        index: 0,
        at: Instant::UNIX_EPOCH,
    };

    fn at(&self) -> Instant<Utc> {
        match self {
            Self::WaypointReached { at, .. }
            | Self::WheelOverReached { at, .. }
            | Self::CrossTrackExceeded { at, .. } => *at,
        }
    }
}

/// Under-keel clearance events.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum ClearanceEvent {
    /// Under-keel clearance below policy: depth of water minus draught and
    /// squat is less than the required margin.
    ///
    /// Reported on every evaluation that finds it. Negative clearance (aground)
    /// is the same report.
    UnderKeelClearanceLow {
        /// Under-keel clearance; negative when aground.
        clearance: Distance,
        /// Minimum required.
        required: Distance,
        /// Time of the depth of water.
        at: Instant<Utc>,
    },
}

impl Event for ClearanceEvent {
    const PLACEHOLDER: Self = Self::UnderKeelClearanceLow {
        clearance: Distance::ZERO,
        required: Distance::ZERO,
        at: Instant::UNIX_EPOCH,
    };

    fn at(&self) -> Instant<Utc> {
        match self {
            Self::UnderKeelClearanceLow { at, .. } => *at,
        }
    }
}

/// Anchor watch events.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum AnchorEvent {
    /// Distance from the anchor exceeds swinging radius + allowance: the anchor
    /// is dragging, or the circle is wrong.
    ///
    /// Reported on every evaluation that finds it.
    AnchorDragging {
        /// Distance from the anchor.
        distance: Distance,
        /// Allowed radius: swinging radius + allowance.
        radius: Distance,
        /// Time of the position.
        at: Instant<Utc>,
    },
}

impl Event for AnchorEvent {
    const PLACEHOLDER: Self = Self::AnchorDragging {
        distance: Distance::ZERO,
        radius: Distance::ZERO,
        at: Instant::UNIX_EPOCH,
    };

    fn at(&self) -> Instant<Utc> {
        match self {
            Self::AnchorDragging { at, .. } => *at,
        }
    }
}

#[cfg(test)]
#[allow(clippy::indexing_slicing)]
mod tests {
    use super::*;

    #[test]
    fn each_context_lists_its_own_events() {
        let at = Instant::from_unix_seconds(1_789_000_000);
        let mut passage = EventList::<GuidanceEvent>::new();
        passage.push(GuidanceEvent::WaypointReached { index: 3, at });
        assert_eq!(passage.len(), 1);
        assert_eq!(passage[0].at(), at);

        let mut anchor = EventList::<AnchorEvent, 2>::with_capacity();
        anchor.push(AnchorEvent::AnchorDragging {
            distance: Distance::ZERO,
            radius: Distance::ZERO,
            at,
        });
        assert_eq!(anchor.as_slice()[0].at(), at);
        assert!(!anchor.overflowed());
    }
}
