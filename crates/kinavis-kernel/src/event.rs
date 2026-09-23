//! Events returned as values.
//!
//! Publish/subscribe needs listener storage (allocation), runs foreign code
//! inside a calculation (unbounded time, possible panic, half-updated state
//! visible) and makes delivery order incidental. Instead, a state-changing
//! operation *returns* what happened in an [`EventList`] alongside its result;
//! the caller alarms, logs or forwards. The list is `#[must_use]` and has fixed
//! capacity with an explicit [`EventList::overflowed`] flag: a lost navigation
//! event is worse than a late one.
//!
//! The kernel's events (fix acquired or lost, observation rejected, integrity
//! or sensor health changed) are [`NavigationEvent`]. Other contexts define
//! their own enums (guidance: waypoint reached; traffic: target lost) in their
//! own [`EventList`]. The [`Event`] trait defines an event: a timestamp and a
//! placeholder for unused slots. The kernel does not know outer contexts'
//! events; the integrating crate (alert board, application) defines the union.

use core::fmt;
use core::ops::Deref;

use crate::inline::{Inline, InlineStr};
use crate::time::{Instant, Utc};
use crate::units::Speed;

/// Position source.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum PositionSource {
    /// Satellite fix.
    Gnss,
    /// Dead reckoning from the last fix.
    DeadReckoning,
    /// Estimator output.
    Estimated,
}

/// Observation rejection reason.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum RejectionReason {
    /// Marked invalid by the source.
    Invalid,
    /// Older than the use case accepts.
    Stale,
    /// Timestamped before an already accepted observation and not applicable
    /// retroactively: late observations are rejected by policy, or it is older
    /// than the policy or history allows.
    OutOfOrder,
    /// Implied speed exceeds the plausible maximum.
    ImplausibleJump {
        /// Implied speed.
        implied_speed: Speed,
    },
    /// Innovation outside the estimator's gate.
    Improbable {
        /// Normalised innovation squared.
        normalised_innovation_squared: f64,
    },
}

/// Integrity of the navigation solution as a whole.
///
/// Not per sensor (see [`SensorHealth`]) but combined: whether the position is
/// held by an absolute source and whether its uncertainty is within the
/// vessel's alert limit. Ordered best to worst.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum NavigationIntegrity {
    /// Absolute position aiding present; uncertainty within the alert limit.
    Nominal,
    /// No absolute position for longer than allowed: dead reckoning,
    /// uncertainty growing, still within the alert limit.
    DeadReckoning,
    /// Uncertainty beyond the alert limit. The position is still reported as
    /// the best available, but must not be relied on for the purpose the limit
    /// protects.
    Exceeded,
}

/// Health of one observation source, as judged by its consumer.
///
/// Judged by content, not by presence: detecting silence is the intake's job,
/// since only it knows the expected rate.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum SensorHealth {
    /// Observations accepted.
    Healthy,
    /// Several consecutive observations rejected. Either the source is faulty
    /// or the estimate has drifted and this source is right; the consumer
    /// cannot tell which, and the operator should check.
    Suspect,
}

/// Maximum sensor name length in an event, bytes.
pub const SENSOR_NAME_BYTES: usize = 32;

/// Observation source identifier: the source's name, truncated to
/// [`SENSOR_NAME_BYTES`] so events stay plain values.
#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SensorId(InlineStr<SENSOR_NAME_BYTES>);

impl SensorId {
    /// Source with this name.
    #[must_use]
    pub fn named(name: &str) -> Self {
        Self(InlineStr::new(name))
    }

    /// Name.
    #[must_use]
    pub fn as_str(&self) -> &str {
        self.0.as_str()
    }
}

impl fmt::Debug for SensorId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt::Debug::fmt(&self.0, f)
    }
}

impl fmt::Display for SensorId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt::Display::fmt(&self.0, f)
    }
}

/// Target identifier: radar track number or AIS MMSI.
///
/// A plain number, as both sources provide and events can carry without
/// allocation. Disambiguating sources that reuse numbers is the intake's job,
/// before the traffic picture.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct TargetId(u32);

impl TargetId {
    /// Target with this number.
    #[must_use]
    pub const fn new(number: u32) -> Self {
        Self(number)
    }

    /// Number.
    #[must_use]
    pub const fn number(self) -> u32 {
        self.0
    }
}

impl fmt::Display for TargetId {
    /// Formats as `#123456789`.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "#{}", self.0)
    }
}

impl core::hash::Hash for SensorId {
    fn hash<H: core::hash::Hasher>(&self, state: &mut H) {
        self.as_str().hash(state);
    }
}

impl PartialEq<str> for SensorId {
    fn eq(&self, other: &str) -> bool {
        self.as_str() == other
    }
}

impl PartialEq<&str> for SensorId {
    fn eq(&self, other: &&str) -> bool {
        self.as_str() == *other
    }
}

/// Kernel event.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum NavigationEvent {
    /// Position source started delivering, or resumed after a gap.
    FixAcquired {
        /// Source.
        source: PositionSource,
        /// Time of the acquiring fix.
        at: Instant<Utc>,
    },
    /// Position source stopped delivering usable fixes.
    FixLost {
        /// Source.
        source: PositionSource,
        /// Time the loss was established.
        at: Instant<Utc>,
        /// Time of the last usable fix.
        last_good: Instant<Utc>,
    },
    /// Observation rejected.
    ObservationRejected {
        /// Reason.
        reason: RejectionReason,
        /// Time of the rejected observation.
        at: Instant<Utc>,
    },
    /// Solution integrity changed.
    IntegrityChanged {
        /// Previous level.
        from: NavigationIntegrity,
        /// New level.
        to: NavigationIntegrity,
        /// Time of the estimate that crossed the threshold.
        at: Instant<Utc>,
    },
    /// Source health changed.
    SensorHealthChanged {
        /// Source.
        sensor: SensorId,
        /// Previous health.
        from: SensorHealth,
        /// New health.
        to: SensorHealth,
        /// Time of the deciding observation.
        at: Instant<Utc>,
    },
}

/// Type storable in an [`EventList`]: something that happened at an instant.
///
/// Implemented by every context's event enum and by application unions. The
/// placeholder fills unused slots so the store needs no allocator and no
/// `unsafe`; it is never read.
pub trait Event: Copy + PartialEq + fmt::Debug {
    /// Unused-slot value; never observed.
    const PLACEHOLDER: Self;

    /// Time of the event.
    fn at(&self) -> Instant<Utc>;
}

impl Event for NavigationEvent {
    const PLACEHOLDER: Self = Self::FixAcquired {
        source: PositionSource::Gnss,
        at: Instant::UNIX_EPOCH,
    };

    fn at(&self) -> Instant<Utc> {
        match self {
            Self::FixAcquired { at, .. }
            | Self::FixLost { at, .. }
            | Self::ObservationRejected { at, .. }
            | Self::IntegrityChanged { at, .. }
            | Self::SensorHealthChanged { at, .. } => *at,
        }
    }
}

/// Default per-operation event capacity.
///
/// A step produces a few events at most (loss and acquisition, one or two
/// rejections). Operations that can produce more (sweeping a traffic picture)
/// return an [`EventList`] sized to their maximum, so nothing is lost by
/// construction; overflow is still reported via [`EventList::overflowed`].
pub const MAX_EVENTS: usize = 8;

/// Events of one operation, in order.
///
/// List of `E` (default [`NavigationEvent`]) with fixed capacity `N`, no
/// allocation. Dereferences to a slice.
#[must_use = "an unread event list is a navigation event nobody acted on"]
#[derive(Clone, Copy)]
pub struct EventList<E: Event = NavigationEvent, const N: usize = MAX_EVENTS> {
    events: Inline<E, N>,
    overflowed: bool,
}

impl<E: Event> EventList<E, MAX_EVENTS> {
    /// Empty list with capacity [`MAX_EVENTS`].
    pub const fn new() -> Self {
        Self::with_capacity()
    }
}

impl<E: Event, const N: usize> EventList<E, N> {
    /// Empty list with capacity `N`, for operations that can report more than
    /// [`MAX_EVENTS`].
    pub const fn with_capacity() -> Self {
        Self {
            events: Inline::new(E::PLACEHOLDER),
            overflowed: false,
        }
    }

    /// Records an event.
    ///
    /// When full the event is dropped and [`EventList::overflowed`] is set,
    /// rather than failing the operation: the work is done and the caller needs
    /// the result plus the fact that the report is incomplete.
    pub fn push(&mut self, event: E) {
        if self.events.push(event).is_err() {
            self.overflowed = true;
        }
    }

    /// Whether an event was dropped for lack of capacity.
    ///
    /// Requires action: the events present are the earliest; at least one later
    /// one is missing.
    #[must_use]
    pub const fn overflowed(&self) -> bool {
        self.overflowed
    }

    /// Events, in order.
    #[must_use]
    pub fn as_slice(&self) -> &[E] {
        self.events.as_slice()
    }
}

impl<E: Event> Default for EventList<E, MAX_EVENTS> {
    fn default() -> Self {
        Self::new()
    }
}

impl<E: Event, const N: usize> Deref for EventList<E, N> {
    type Target = [E];

    fn deref(&self) -> &[E] {
        self.as_slice()
    }
}

impl<'a, E: Event, const N: usize> IntoIterator for &'a EventList<E, N> {
    type Item = &'a E;
    type IntoIter = core::slice::Iter<'a, E>;

    fn into_iter(self) -> Self::IntoIter {
        self.as_slice().iter()
    }
}

impl<E: Event, const N: usize> fmt::Debug for EventList<E, N> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("EventList")
            .field("events", &self.events)
            .field("overflowed", &self.overflowed)
            .finish()
    }
}

impl<E: Event, const N: usize, const M: usize> PartialEq<EventList<E, M>> for EventList<E, N> {
    /// Equal when they hold the same events and the same overflow flag,
    /// regardless of capacity.
    fn eq(&self, other: &EventList<E, M>) -> bool {
        self.overflowed == other.overflowed && self.as_slice() == other.as_slice()
    }
}

#[cfg(test)]
#[allow(clippy::cast_possible_wrap)]
mod tests {
    use super::*;

    /// Capacity as a timestamp, for numbering test events.
    const CAPACITY: i64 = MAX_EVENTS as i64;

    fn acquired(seconds: i64) -> NavigationEvent {
        NavigationEvent::FixAcquired {
            source: PositionSource::Gnss,
            at: Instant::from_unix_seconds(seconds),
        }
    }

    #[test]
    fn events_come_back_in_order() {
        let mut list = EventList::new();
        assert!(list.is_empty());
        list.push(acquired(1));
        list.push(NavigationEvent::ObservationRejected {
            reason: RejectionReason::Stale,
            at: Instant::from_unix_seconds(2),
        });
        assert_eq!(list.len(), 2);
        assert_eq!(list.first(), Some(&acquired(1)));
        assert!(matches!(
            list.last(),
            Some(NavigationEvent::ObservationRejected {
                reason: RejectionReason::Stale,
                ..
            })
        ));
        assert_eq!((&list).into_iter().count(), 2);
        assert!(!list.overflowed());
    }

    #[test]
    fn a_full_list_keeps_the_earliest_and_says_it_lost_the_rest() {
        let mut list = EventList::new();
        for second in 0..CAPACITY {
            list.push(acquired(second));
        }
        assert!(!list.overflowed());
        list.push(acquired(99));
        assert!(list.overflowed());
        assert_eq!(list.len(), MAX_EVENTS);
        assert_eq!(list.last(), Some(&acquired(CAPACITY - 1)));
    }

    #[test]
    fn lists_compare_by_events_and_by_loss() {
        let mut first = EventList::new();
        let mut second = EventList::default();
        first.push(acquired(1));
        second.push(acquired(1));
        assert_eq!(first, second);
        for second_number in 0..=CAPACITY {
            second.push(acquired(second_number));
        }
        assert_ne!(first, second);
    }
}
