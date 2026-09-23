//! Target tracking from radar plots and AIS reports, collision assessment and
//! avoidance.
//!
//! An observation gives a numbered target's position at an instant. A
//! [`TargetTrack`] derives course and speed (least-squares over recent fixes,
//! or as reported by the target), an extrapolated position and an age.
//! [`Traffic`] holds the tracks, ingests observations, drops silent targets and
//! produces a [`TrafficView`] plus events: [`TrafficEvent::TargetAcquired`],
//! [`TrafficEvent::TargetLost`], [`TrafficEvent::ObservationRejected`] (out of
//! order or implausible speed).
//!
//! Thresholds are vessel settings in a [`TrackingPolicy`]: fixes to acquire,
//! stale and lost timeouts, maximum plausible speed.
//!
//! Collision assessment: [`assess`] for one encounter (CPA/TCPA, bearing at
//! CPA, bearing rate, bow crossing range, [`CollisionRisk`] against a
//! [`CpaPolicy`]); [`assess_track`] for a tracked target; [`assess_traffic`]
//! for the whole picture against own ship's snapshot, raising
//! [`TrafficEvent::CpaAlarm`]. Give-way rules live in `kinavis-colregs`; their
//! permitted manoeuvre comes back as [`ManoeuvreConstraints`], within which
//! [`avoid`] and [`avoid_all`] find the smallest course alteration that opens
//! one or all targets to the requested passing distance.
//!
//! ```rust
//! use kinavis_kernel::{Instant, Position, Speed, TargetId, Utc};
//! use kinavis_traffic::{TargetObservation, Traffic, TrackingPolicy, TrafficEvent, WhenFull};
//! use core::time::Duration;
//!
//! // Three plots to acquire, stale after half a minute, dropped after three,
//! // nothing faster than sixty knots; and when the picture is full, the
//! // target that has gone quietest makes way for the newcomer.
//! let policy = TrackingPolicy::new(
//!     3,
//!     Duration::from_secs(30),
//!     Duration::from_secs(180),
//!     Speed::from_knots(60.0)?,
//! )?
//! .when_full(WhenFull::EvictStalest);
//! let mut traffic = Traffic::new(policy);
//!
//! // Three radar plots of one target, a minute apart, heading north at
//! // twelve knots: a fifth of a mile a minute.
//! let start = Instant::<Utc>::from_unix_seconds(1_789_000_000);
//! let target = TargetId::new(7);
//! let mut acquired = false;
//! for minute in 0_u32..3 {
//!     let position = Position::from_degrees(50.0 + 0.2 * f64::from(minute) / 60.0, -1.0)?;
//!     let at = start.checked_add(Duration::from_secs(60 * u64::from(minute))).unwrap();
//!     let events = traffic.ingest(TargetObservation::new(target, position, at))?;
//!     acquired |= events
//!         .iter()
//!         .any(|event| matches!(event, TrafficEvent::TargetAcquired { .. }));
//! }
//! assert!(acquired);
//!
//! // Two and a half minutes in, the picture has it half a mile up the track.
//! let now = start.checked_add(Duration::from_secs(150)).unwrap();
//! let view = traffic.view(now);
//! let seen = &view.targets()[0];
//! let motion = seen.motion.unwrap();
//! assert_eq!(format!("{:.0}", motion.course_over_ground), "000°T");
//! assert_eq!(format!("{:.1}", motion.speed_over_ground.knots()), "12.0");
//! assert_eq!(format!("{:.3}", seen.position.latitude().degrees()), "50.008");
//! assert!(!seen.stale);
//!
//! // Ten minutes of silence and it is gone.
//! let later = now.checked_add(Duration::from_secs(600)).unwrap();
//! let events = traffic.sweep(later);
//! assert!(matches!(events[0], TrafficEvent::TargetLost { target, .. } if target == TargetId::new(7)));
//! assert!(traffic.is_empty());
//! # Ok::<(), kinavis::NavigationError>(())
//! ```
//!
//! # Feature flags
//!
//! - `std` *(default)* — standard library maths in the kernel.
//! - `libm` — for `no_std` targets: `--no-default-features --features libm`.
//! - `serde` — serialisation of observations, policies and view entries.
//!
//! No allocation; builds for bare-metal targets. The picture is stored inline
//! ([`MAX_TARGETS`] tracks × [`MAX_TRACK_HISTORY`] fixes), so [`Traffic`] is
//! large: keep it behind a reference or in a `static`.
//!

#![cfg_attr(not(feature = "std"), no_std)]

// The crate does not allocate; tests use `format!`.
#[cfg(test)]
extern crate alloc;

mod assessment;
mod avoidance;
mod event;
mod observation;
mod track;

/// Runs the `README.md` example as a doctest.
#[cfg(doctest)]
#[doc = include_str!("../README.md")]
pub struct ReadmeExamples;

use core::time::Duration;

use kinavis::error::{ensure_range, KernelError, NavigationError, Result};
use kinavis::sailings::great_circle;
use kinavis_kernel::angle::TrueCourse;
use kinavis_kernel::event::{EventList, RejectionReason, TargetId};
use kinavis_kernel::inline::Inline;
#[cfg(test)]
use kinavis_kernel::math;
use kinavis_kernel::position::Position;
use kinavis_kernel::snapshot::GroundTrack;
use kinavis_kernel::time::{Instant, Utc};
use kinavis_kernel::units::Speed;

pub use assessment::{
    assess, assess_track, assess_traffic, CollisionAssessment, CollisionPicture, CollisionRisk,
    CpaPolicy, TargetAssessment,
};
pub use avoidance::{
    avoid, avoid_all, AvoidanceManoeuvre, ManoeuvreConstraints, PermittedSides,
    ALTERATION_STEP_DEG, MAX_ALTERATION_DEG,
};
pub use event::TrafficEvent;
pub use observation::TargetObservation;
pub use track::{TargetTrack, TrackStatus, MAX_TRACK_HISTORY};

/// Default picture capacity.
///
/// ARPA tracks a few dozen targets; a dense AIS picture should be
/// range-filtered first or use a larger capacity ([`Traffic`] takes it as a
/// parameter). Behaviour when full is set by [`WhenFull`].
pub const MAX_TARGETS: usize = 32;

/// Behaviour for a new target when the picture is full.
///
/// A full picture may be caused by spoofed identities (fake AIS, corrupt feed)
/// as well as by traffic density; refusing every newcomer would then hide real
/// targets. The policy decides.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum WhenFull {
    /// Reject the newcomer with [`KernelError::CapacityExceeded`]; the picture
    /// is unchanged.
    #[default]
    Refuse,
    /// Evict the least recently observed target, reporting
    /// [`TrafficEvent::TargetEvicted`], and track the newcomer. Active targets
    /// are kept; silent ones go first.
    EvictStalest,
}

/// Track acquisition and loss thresholds: vessel settings.
///
/// Validated by [`TrackingPolicy::new`]: at least one fix to acquire, lost
/// timeout ≥ stale timeout, positive maximum speed.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(try_from = "StoredTrackingPolicy", into = "StoredTrackingPolicy")
)]
pub struct TrackingPolicy {
    fixes_to_acquire: u8,
    stale_after: Duration,
    lost_after: Duration,
    max_speed: Speed,
    when_full: WhenFull,
}

impl TrackingPolicy {
    /// Policy from four thresholds; full picture refuses newcomers.
    ///
    /// - `fixes_to_acquire`: observations before
    ///   [`TrafficEvent::TargetAcquired`] (3 is the radar convention; AIS can
    ///   be trusted from the first).
    /// - `stale_after`: time since the last observation after which the track
    ///   is marked stale in the view (still shown and extrapolated).
    /// - `lost_after` (≥ `stale_after`): time after which [`Traffic::sweep`]
    ///   drops the track with [`TrafficEvent::TargetLost`].
    /// - `max_speed`: maximum plausible target speed; an observation implying
    ///   more is rejected as an implausible jump (swapped track, corrupt
    ///   report).
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] for zero fixes, `lost_after < stale_after`,
    /// or a non-positive speed.
    pub fn new(
        fixes_to_acquire: u8,
        stale_after: Duration,
        lost_after: Duration,
        max_speed: Speed,
    ) -> Result<Self> {
        ensure_range(
            "fixes to acquire",
            f64::from(fixes_to_acquire),
            1.0,
            f64::from(u8::MAX),
        )?;
        ensure_range(
            "lost after",
            lost_after.as_secs_f64(),
            stale_after.as_secs_f64(),
            f64::MAX,
        )?;
        ensure_range("max speed", max_speed.knots(), f64::MIN_POSITIVE, f64::MAX)?;
        Ok(Self {
            fixes_to_acquire,
            stale_after,
            lost_after,
            max_speed,
            when_full: WhenFull::Refuse,
        })
    }

    /// Sets the full-picture behaviour.
    #[must_use]
    pub const fn when_full(mut self, when_full: WhenFull) -> Self {
        self.when_full = when_full;
        self
    }

    /// Observations needed to acquire.
    #[must_use]
    pub const fn fixes_to_acquire(&self) -> u8 {
        self.fixes_to_acquire
    }

    /// Stale timeout.
    #[must_use]
    pub const fn stale_after(&self) -> Duration {
        self.stale_after
    }

    /// Lost timeout.
    #[must_use]
    pub const fn lost_after(&self) -> Duration {
        self.lost_after
    }

    /// Maximum plausible target speed.
    #[must_use]
    pub const fn max_speed(&self) -> Speed {
        self.max_speed
    }

    /// Full-picture behaviour.
    #[must_use]
    pub const fn on_full(&self) -> WhenFull {
        self.when_full
    }
}

/// Serialised form; deserialisation goes through [`TrackingPolicy::new`].
#[cfg(feature = "serde")]
#[derive(serde::Serialize, serde::Deserialize)]
struct StoredTrackingPolicy {
    fixes_to_acquire: u8,
    stale_after: Duration,
    lost_after: Duration,
    max_speed: Speed,
    when_full: WhenFull,
}

#[cfg(feature = "serde")]
impl TryFrom<StoredTrackingPolicy> for TrackingPolicy {
    type Error = NavigationError;

    fn try_from(stored: StoredTrackingPolicy) -> Result<Self> {
        Ok(Self::new(
            stored.fixes_to_acquire,
            stored.stale_after,
            stored.lost_after,
            stored.max_speed,
        )?
        .when_full(stored.when_full))
    }
}

#[cfg(feature = "serde")]
impl From<TrackingPolicy> for StoredTrackingPolicy {
    fn from(policy: TrackingPolicy) -> Self {
        Self {
            fixes_to_acquire: policy.fixes_to_acquire,
            stale_after: policy.stale_after,
            lost_after: policy.lost_after,
            max_speed: policy.max_speed,
            when_full: policy.when_full,
        }
    }
}

/// Traffic picture: up to `N` tracked targets.
///
/// Aggregate with its policy: observations via [`Traffic::ingest`], time via
/// [`Traffic::sweep`], output via [`Traffic::view`]. Stored inline and large
/// (each track holds [`MAX_TRACK_HISTORY`] fixes): keep it behind a reference
/// or in a `static`. Deliberately not `Copy`, so the picture cannot be
/// duplicated by accident:
///
/// ```compile_fail
/// fn is_copy<T: Copy>() {}
/// is_copy::<kinavis_traffic::Traffic>();
/// ```
#[derive(Debug, Clone, PartialEq)]
pub struct Traffic<const N: usize = MAX_TARGETS> {
    policy: TrackingPolicy,
    tracks: Inline<TargetTrack, N>,
}

impl Traffic<MAX_TARGETS> {
    /// Empty picture with [`MAX_TARGETS`] capacity.
    #[must_use]
    pub fn new(policy: TrackingPolicy) -> Self {
        Self::with_capacity(policy)
    }
}

impl<const N: usize> Traffic<N> {
    /// Empty picture with capacity `N`, for dense pictures without range
    /// filtering.
    #[must_use]
    pub fn with_capacity(policy: TrackingPolicy) -> Self {
        Self {
            policy,
            tracks: Inline::new(TargetTrack::placeholder()),
        }
    }

    /// Policy.
    #[must_use]
    pub const fn policy(&self) -> TrackingPolicy {
        self.policy
    }

    /// Capacity.
    #[must_use]
    pub const fn capacity() -> usize {
        N
    }

    /// Number of tracks, including those still acquiring.
    #[must_use]
    pub const fn len(&self) -> usize {
        self.tracks.len()
    }

    /// Whether empty.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.tracks.is_empty()
    }

    /// Tracks, in order of first detection.
    #[must_use]
    pub fn tracks(&self) -> &[TargetTrack] {
        &self.tracks
    }

    /// Track for a target, if present.
    #[must_use]
    pub fn track(&self, target: TargetId) -> Option<&TargetTrack> {
        self.tracks.iter().find(|track| track.target() == target)
    }

    /// Ingests an observation.
    ///
    /// A new target starts a track; a known target's track is extended unless
    /// the observation is not newer than the last or implies a speed above
    /// `max_speed`, in which case it is rejected with
    /// [`TrafficEvent::ObservationRejected`]. [`TrafficEvent::TargetAcquired`]
    /// is reported when the track reaches `fixes_to_acquire`.
    ///
    /// A new target in a full picture is handled per [`WhenFull`]: rejected, or
    /// admitted by evicting the least recently observed target
    /// ([`TrafficEvent::TargetEvicted`]).
    ///
    /// # Errors
    ///
    /// - [`KernelError::CapacityExceeded`] for a new target in a full picture
    ///   under [`WhenFull::Refuse`]; the picture is unchanged.
    /// - A sailing failure in the plausibility check.
    pub fn ingest(&mut self, observation: TargetObservation) -> Result<EventList<TrafficEvent>> {
        let mut events = EventList::new();
        let index = self
            .tracks
            .iter()
            .position(|track| track.target() == observation.target());

        let Some(index) = index else {
            if self.tracks.len() >= N {
                self.make_room(&observation, &mut events)?;
            }
            let mut track = TargetTrack::started_by(&observation);
            if self.policy.fixes_to_acquire <= 1 {
                track.acquire();
                events.push(TrafficEvent::TargetAcquired {
                    target: observation.target(),
                    at: observation.at(),
                });
            }
            self.tracks.push(track).map_err(|full| {
                NavigationError::Kernel(KernelError::CapacityExceeded {
                    context: "the traffic picture",
                    needed: full.capacity.saturating_add(1),
                    capacity: full.capacity,
                })
            })?;
            return Ok(events);
        };

        let Some(track) = self.tracks.as_mut_slice().get_mut(index) else {
            return Ok(events);
        };
        let Some(elapsed) = observation
            .at()
            .checked_duration_since(track.last_seen())
            .filter(|elapsed| !elapsed.is_zero())
        else {
            events.push(TrafficEvent::ObservationRejected {
                target: observation.target(),
                reason: RejectionReason::OutOfOrder,
                at: observation.at(),
            });
            return Ok(events);
        };
        let jump = great_circle(track.last_position(), observation.position())?.distance;
        let implied = jump.nautical_miles() / (elapsed.as_secs_f64() / 3600.0);
        if !implied.is_finite() || implied > self.policy.max_speed.knots() {
            events.push(TrafficEvent::ObservationRejected {
                target: observation.target(),
                reason: RejectionReason::ImplausibleJump {
                    implied_speed: Speed::from_knots_unchecked(implied.min(f64::MAX)),
                },
                at: observation.at(),
            });
            return Ok(events);
        }

        track.extend(&observation);
        if track.status() == TrackStatus::Acquiring
            && track.fix_count() >= usize::from(self.policy.fixes_to_acquire)
        {
            track.acquire();
            events.push(TrafficEvent::TargetAcquired {
                target: observation.target(),
                at: observation.at(),
            });
        }
        Ok(events)
    }

    /// Frees a slot for `newcomer` in a full picture, per policy.
    fn make_room(
        &mut self,
        newcomer: &TargetObservation,
        events: &mut EventList<TrafficEvent>,
    ) -> Result<()> {
        match self.policy.when_full {
            WhenFull::Refuse => Err(NavigationError::Kernel(KernelError::CapacityExceeded {
                context: "the traffic picture",
                needed: N.saturating_add(1),
                capacity: N,
            })),
            WhenFull::EvictStalest => {
                let stalest = self
                    .tracks
                    .iter()
                    .enumerate()
                    .min_by_key(|(_, track)| track.last_seen())
                    .map(|(index, _)| index);
                if let Some(evicted) = stalest.and_then(|index| self.tracks.remove(index)) {
                    events.push(TrafficEvent::TargetEvicted {
                        target: evicted.target(),
                        last_seen: evicted.last_seen(),
                        for_target: newcomer.target(),
                    });
                }
                Ok(())
            }
        }
    }

    /// Drops tracks not observed within `lost_after`, reporting
    /// [`TrafficEvent::TargetLost`] for each.
    ///
    /// The picture has no clock; call this as often as targets should expire.
    /// The returned list has room for one loss per target.
    pub fn sweep(&mut self, now: Instant<Utc>) -> EventList<TrafficEvent, N> {
        let mut events = EventList::with_capacity();
        let mut kept = Inline::<TargetTrack, N>::new(TargetTrack::placeholder());
        for track in self.tracks.iter() {
            if track.age(now) > self.policy.lost_after {
                events.push(TrafficEvent::TargetLost {
                    target: track.target(),
                    last_seen: track.last_seen(),
                });
            } else {
                // The new store has the capacity of the old one.
                let _ = kept.push(*track);
            }
        }
        self.tracks = kept;
        events
    }

    /// Picture at `now`: each target at its extrapolated position, with motion,
    /// age and staleness.
    #[must_use]
    pub fn view(&self, now: Instant<Utc>) -> TrafficView<N> {
        let mut targets = Inline::<TargetView, N>::new(TargetView::PLACEHOLDER);
        for track in self.tracks.iter() {
            let age = track.age(now);
            // If extrapolation fails (across a pole), fall back to the last
            // observed position instead of dropping the target.
            let position = track.position_at(now).unwrap_or(track.last_position());
            let _ = targets.push(TargetView {
                target: track.target(),
                status: track.status(),
                position,
                motion: track.motion(),
                heading: track.heading(),
                age,
                stale: age > self.policy.stale_after,
            });
        }
        TrafficView { at: now, targets }
    }
}

/// One target in the view.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct TargetView {
    /// Target.
    pub target: TargetId,
    /// Whether acquired.
    pub status: TrackStatus,
    /// Extrapolated current position.
    pub position: Position,
    /// Course and speed over ground, if known.
    pub motion: Option<GroundTrack>,
    /// Reported heading, if any.
    pub heading: Option<TrueCourse>,
    /// Time since the last observation.
    pub age: Duration,
    /// Whether older than the stale timeout.
    pub stale: bool,
}

impl TargetView {
    /// Fill value for the view's store; never read.
    const PLACEHOLDER: Self = Self {
        target: TargetId::new(0),
        status: TrackStatus::Acquiring,
        position: Position::new(
            kinavis_kernel::position::Latitude::EQUATOR,
            kinavis_kernel::position::Longitude::GREENWICH,
        ),
        motion: None,
        heading: None,
        age: Duration::ZERO,
        stale: false,
    };
}

/// Traffic picture at one instant: read model built by [`Traffic::view`], with
/// the source picture's capacity.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TrafficView<const N: usize = MAX_TARGETS> {
    at: Instant<Utc>,
    targets: Inline<TargetView, N>,
}

impl<const N: usize> TrafficView<N> {
    /// Time of the view.
    #[must_use]
    pub const fn at(&self) -> Instant<Utc> {
        self.at
    }

    /// Targets, in order of first detection.
    #[must_use]
    pub fn targets(&self) -> &[TargetView] {
        &self.targets
    }

    /// Number of targets.
    #[must_use]
    pub const fn len(&self) -> usize {
        self.targets.len()
    }

    /// Whether empty.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.targets.is_empty()
    }

    /// Entry for a target, if present.
    #[must_use]
    pub fn target(&self, target: TargetId) -> Option<&TargetView> {
        self.targets.iter().find(|view| view.target == target)
    }

    /// Targets that are not stale.
    pub fn current(&self) -> impl Iterator<Item = &TargetView> + '_ {
        self.targets.iter().filter(|view| !view.stale)
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp, clippy::indexing_slicing)]
mod tests {
    use super::*;
    use kinavis::relative_motion::Contact;
    use kinavis_kernel::angle::TrueBearing;
    use kinavis_kernel::units::Distance;

    fn at(latitude: f64, longitude: f64) -> Position {
        Position::from_degrees(latitude, longitude).unwrap()
    }

    fn start() -> Instant<Utc> {
        Instant::from_unix_seconds(1_789_000_000)
    }

    fn after(seconds: u64) -> Instant<Utc> {
        start().checked_add(Duration::from_secs(seconds)).unwrap()
    }

    fn knots(value: f64) -> Speed {
        Speed::from_knots(value).unwrap()
    }

    fn policy() -> TrackingPolicy {
        TrackingPolicy::new(
            3,
            Duration::from_secs(30),
            Duration::from_secs(180),
            knots(60.0),
        )
        .unwrap()
    }

    /// Target heading north at 12 kn from 50°N 1°W, plotted every `interval`
    /// seconds; fix `n` at `n × interval`.
    fn northbound(target: u32, n: u64, interval: u64) -> TargetObservation {
        let miles = 12.0 * f64::from(u32::try_from(n * interval).unwrap()) / 3600.0;
        TargetObservation::new(
            TargetId::new(target),
            at(50.0 + miles / 60.0, -1.0),
            after(n * interval),
        )
    }

    fn kinds(events: &[TrafficEvent]) -> alloc::vec::Vec<&'static str> {
        events
            .iter()
            .map(|event| match event {
                TrafficEvent::TargetAcquired { .. } => "acquired",
                TrafficEvent::TargetLost { .. } => "lost",
                TrafficEvent::ObservationRejected { .. } => "rejected",
                _ => "other",
            })
            .collect()
    }

    #[test]
    fn a_target_is_acquired_on_the_policys_fix_and_not_before() {
        let mut traffic = Traffic::new(policy());
        assert!(traffic.is_empty());

        assert!(traffic.ingest(northbound(7, 0, 60)).unwrap().is_empty());
        assert_eq!(traffic.len(), 1);
        assert_eq!(
            traffic.track(TargetId::new(7)).unwrap().status(),
            TrackStatus::Acquiring
        );
        assert!(traffic.ingest(northbound(7, 1, 60)).unwrap().is_empty());
        let events = traffic.ingest(northbound(7, 2, 60)).unwrap();
        assert_eq!(kinds(&events), ["acquired"]);
        assert!(matches!(
            events[0],
            TrafficEvent::TargetAcquired { target, at } if target == TargetId::new(7) && at == after(120)
        ));
        assert_eq!(
            traffic.track(TargetId::new(7)).unwrap().status(),
            TrackStatus::Tracking
        );
        // Acquired once; no second event.
        assert!(traffic.ingest(northbound(7, 3, 60)).unwrap().is_empty());
    }

    #[test]
    fn a_policy_of_one_fix_acquires_on_sight() {
        let mut traffic = Traffic::new(
            TrackingPolicy::new(
                1,
                policy().stale_after(),
                policy().lost_after(),
                policy().max_speed(),
            )
            .unwrap(),
        );
        let events = traffic.ingest(northbound(9, 0, 10)).unwrap();
        assert_eq!(kinds(&events), ["acquired"]);
    }

    #[test]
    fn the_fitted_motion_is_the_targets_course_and_speed() {
        let mut traffic = Traffic::new(policy());
        for n in 0..5 {
            let _ = traffic.ingest(northbound(7, n, 60)).unwrap();
        }
        let track = traffic.track(TargetId::new(7)).unwrap();
        assert_eq!(track.fix_count(), 5);
        assert_eq!(track.first_seen(), start());
        assert_eq!(track.last_seen(), after(240));
        let motion = track.fitted_motion().unwrap();
        assert!(
            motion.course_over_ground.degrees() < 0.01
                || motion.course_over_ground.degrees() > 359.99
        );
        assert!((motion.speed_over_ground.knots() - 12.0).abs() < 0.01);
        assert_eq!(track.motion(), Some(motion));
        assert_eq!(track.reported_ground_track(), None);
        assert_eq!(track.as_vessel().unwrap().speed, motion.speed_over_ground);

        // +1 min: 0.2 NM further north; −1 min: less.
        let ahead = track.position_at(after(300)).unwrap();
        assert!((ahead.latitude().degrees() - (50.0 + 1.0 / 60.0)).abs() < 1e-4);
        let behind = track.position_at(after(180)).unwrap();
        assert!((behind.latitude().degrees() - (50.0 + 0.6 / 60.0)).abs() < 1e-4);
    }

    #[test]
    fn the_fit_smooths_a_noisy_plot() {
        let mut traffic = Traffic::new(policy());
        // True track, every other plot offset 1 cable east.
        for n in 0..8 {
            let mut observation = northbound(7, n, 60);
            if n % 2 == 1 {
                observation = TargetObservation::new(
                    observation.target(),
                    at(
                        observation.position().latitude().degrees(),
                        -1.0 + 0.1 / 60.0 / math::cos(50.0_f64.to_radians()),
                    ),
                    observation.at(),
                );
            }
            let _ = traffic.ingest(observation).unwrap();
        }
        let motion = traffic
            .track(TargetId::new(7))
            .unwrap()
            .fitted_motion()
            .unwrap();
        // Northbound at 12 kn, with small scatter.
        let course = motion.course_over_ground.degrees();
        assert!(course < 2.0 || course > 358.0, "{course}");
        assert!((motion.speed_over_ground.knots() - 12.0).abs() < 0.5);
    }

    #[test]
    fn a_reported_track_beats_the_fitted_one() {
        let mut traffic = Traffic::new(policy());
        let reported = GroundTrack {
            course_over_ground: TrueCourse::new(3.0).unwrap(),
            speed_over_ground: knots(11.5),
        };
        let _ = traffic
            .ingest(northbound(7, 0, 60).with_ground_track(reported))
            .unwrap();
        // One fix with a report: motion available immediately.
        let track = traffic.track(TargetId::new(7)).unwrap();
        assert_eq!(track.fitted_motion(), None);
        assert_eq!(track.motion(), Some(reported));

        let _ = traffic
            .ingest(northbound(7, 1, 60).with_heading(TrueCourse::new(5.0).unwrap()))
            .unwrap();
        let track = traffic.track(TargetId::new(7)).unwrap();
        // Course/speed report kept from the earlier observation, heading from
        // the later one.
        assert_eq!(track.motion(), Some(reported));
        assert_eq!(track.heading(), Some(TrueCourse::new(5.0).unwrap()));
        assert!(track.fitted_motion().is_some());
    }

    #[test]
    fn the_window_keeps_the_latest_fixes() {
        let mut traffic = Traffic::new(policy());
        for n in 0..20 {
            let _ = traffic.ingest(northbound(7, n, 30)).unwrap();
        }
        let track = traffic.track(TargetId::new(7)).unwrap();
        assert_eq!(track.fix_count(), MAX_TRACK_HISTORY);
        assert_eq!(track.last_seen(), after(19 * 30));
        assert_eq!(
            track.first_seen(),
            after((20 - MAX_TRACK_HISTORY as u64) * 30)
        );
        let motion = track.fitted_motion().unwrap();
        assert!((motion.speed_over_ground.knots() - 12.0).abs() < 0.01);
    }

    #[test]
    fn an_observation_out_of_order_or_too_fast_is_turned_away() {
        let mut traffic = Traffic::new(policy());
        let _ = traffic.ingest(northbound(7, 1, 60)).unwrap();

        // Same time again, and an earlier time.
        let same = traffic.ingest(northbound(7, 1, 60)).unwrap();
        assert!(matches!(
            same[0],
            TrafficEvent::ObservationRejected {
                reason: RejectionReason::OutOfOrder,
                ..
            }
        ));
        let earlier = traffic.ingest(northbound(7, 0, 60)).unwrap();
        assert_eq!(kinds(&earlier), ["rejected"]);

        // 10 NM in 1 min: 600 kn.
        let jump = traffic
            .ingest(TargetObservation::new(
                TargetId::new(7),
                at(50.0 + 10.0 / 60.0, -1.0),
                after(120),
            ))
            .unwrap();
        assert!(matches!(
            jump[0],
            TrafficEvent::ObservationRejected {
                target,
                reason: RejectionReason::ImplausibleJump { implied_speed },
                at
            } if implied_speed.knots() > 500.0 && at == after(120) && target == TargetId::new(7)
        ));
        // Track unchanged.
        let track = traffic.track(TargetId::new(7)).unwrap();
        assert_eq!(track.fix_count(), 1);
        assert_eq!(track.last_seen(), after(60));
    }

    #[test]
    fn a_silent_target_goes_stale_and_then_is_lost() {
        let mut traffic = Traffic::new(policy());
        for n in 0..3 {
            let _ = traffic.ingest(northbound(7, n, 60)).unwrap();
        }
        let _ = traffic.ingest(northbound(8, 0, 60)).unwrap();

        let fresh = traffic.view(after(130));
        assert_eq!(fresh.len(), 2);
        assert!(!fresh.target(TargetId::new(7)).unwrap().stale);
        assert_eq!(fresh.current().count(), 1);
        // Target 8 last seen at the start, more than 30 s ago.
        assert!(fresh.target(TargetId::new(8)).unwrap().stale);
        assert_eq!(fresh.at(), after(130));

        let stale = traffic.view(after(160));
        assert!(stale.target(TargetId::new(7)).unwrap().stale);
        assert_eq!(
            stale.target(TargetId::new(7)).unwrap().age,
            Duration::from_secs(40)
        );
        assert_eq!(stale.current().count(), 0);

        // Nothing is dropped before a sweep, nor before the policy timeout.
        assert!(traffic.sweep(after(170)).is_empty());
        assert_eq!(traffic.len(), 2);
        let events = traffic.sweep(after(181));
        assert_eq!(traffic.len(), 1);
        assert!(matches!(
            events[0],
            TrafficEvent::TargetLost { target, last_seen }
                if target == TargetId::new(8) && last_seen == start()
        ));
        let events = traffic.sweep(after(120 + 181));
        assert_eq!(kinds(&events), ["lost"]);
        assert!(traffic.is_empty());
        assert!(traffic.view(after(1000)).is_empty());
    }

    #[test]
    fn the_view_carries_the_target_forward() {
        let mut traffic = Traffic::new(policy());
        for n in 0..3 {
            let _ = traffic.ingest(northbound(7, n, 60)).unwrap();
        }
        let view = traffic.view(after(180));
        let seen = view.targets()[0];
        assert_eq!(seen.target, TargetId::new(7));
        assert_eq!(seen.status, TrackStatus::Tracking);
        // 3 min at 12 kn: 0.6 NM north.
        assert!((seen.position.latitude().degrees() - (50.0 + 0.6 / 60.0)).abs() < 1e-4);
        assert!(seen.motion.is_some());
        assert_eq!(seen.heading, None);
        assert_eq!(seen.age, Duration::from_secs(60));
    }

    #[test]
    fn a_full_picture_refuses_a_new_target_and_keeps_the_old() {
        let mut traffic = Traffic::new(policy());
        for target in 0..MAX_TARGETS {
            let _ = traffic
                .ingest(northbound(u32::try_from(target).unwrap(), 0, 60))
                .unwrap();
        }
        assert_eq!(traffic.len(), MAX_TARGETS);
        assert!(matches!(
            traffic
                .ingest(northbound(u32::try_from(MAX_TARGETS).unwrap(), 0, 60))
                .unwrap_err(),
            NavigationError::Kernel(KernelError::CapacityExceeded {
                context: "the traffic picture",
                ..
            })
        ));
        assert_eq!(traffic.len(), MAX_TARGETS);
        // A known target is still accepted.
        assert!(traffic.ingest(northbound(3, 1, 60)).is_ok());
    }

    #[test]
    fn a_radar_contact_becomes_a_position() {
        let own = at(50.0, -1.0);
        let observation = TargetObservation::from_contact(
            TargetId::new(4),
            own,
            Contact {
                bearing: TrueBearing::new(90.0).unwrap(),
                range: Distance::from_nautical_miles(6.0).unwrap(),
            },
            start(),
        )
        .unwrap();
        assert_eq!(observation.target(), TargetId::new(4));
        assert_eq!(observation.at(), start());
        assert!((observation.position().latitude().degrees() - 50.0).abs() < 1e-9);
        assert!(observation.position().longitude().degrees() > -1.0 + 0.15);
        assert_eq!(observation.ground_track(), None);
        assert_eq!(observation.heading(), None);
    }

    #[test]
    fn a_policy_must_make_sense() {
        let (stale, lost) = (Duration::from_secs(30), Duration::from_secs(180));
        assert!(TrackingPolicy::new(0, stale, lost, knots(60.0)).is_err());
        assert!(TrackingPolicy::new(3, stale, Duration::from_secs(10), knots(60.0)).is_err());
        assert!(TrackingPolicy::new(3, stale, lost, Speed::ZERO).is_err());
        assert_eq!(Traffic::new(policy()).policy(), policy());
        assert_eq!(policy().on_full(), WhenFull::Refuse);
        assert_eq!(
            policy().when_full(WhenFull::EvictStalest).on_full(),
            WhenFull::EvictStalest
        );
    }

    #[test]
    fn a_full_picture_told_to_evict_drops_the_stalest_and_says_so() {
        let mut traffic = Traffic::<4>::with_capacity(policy().when_full(WhenFull::EvictStalest));
        // Four targets first seen at 0, 10, 20, 30 s; target 1 seen again at 40
        // s, so target 0 is the stalest.
        for target in 0..4_u64 {
            let observation = TargetObservation::new(
                TargetId::new(u32::try_from(target).unwrap()),
                at(50.0, -1.0),
                after(10 * target),
            );
            let _ = traffic.ingest(observation).unwrap();
        }
        let _ = traffic
            .ingest(TargetObservation::new(
                TargetId::new(1),
                at(50.01, -1.0),
                after(40),
            ))
            .unwrap();
        let events = traffic
            .ingest(TargetObservation::new(
                TargetId::new(9),
                at(50.0, -1.0),
                after(50),
            ))
            .unwrap();
        assert_eq!(traffic.len(), 4);
        assert!(traffic.track(TargetId::new(0)).is_none());
        assert!(traffic.track(TargetId::new(9)).is_some());
        assert_eq!(
            events.as_slice(),
            [TrafficEvent::TargetEvicted {
                target: TargetId::new(0),
                last_seen: start(),
                for_target: TargetId::new(9),
            }]
        );
        // A flood of newcomers cycles through the free slots and never evicts
        // the target that keeps reporting.
        for flood in 100..200_u64 {
            let seen = after(60 + flood);
            let _ = traffic
                .ingest(TargetObservation::new(
                    TargetId::new(1),
                    at(50.02, -1.0),
                    seen,
                ))
                .unwrap();
            let newcomer = TargetId::new(u32::try_from(flood).unwrap());
            let _ = traffic
                .ingest(TargetObservation::new(newcomer, at(50.0, -1.0), seen))
                .unwrap();
        }
        assert_eq!(traffic.len(), 4);
        assert!(traffic.track(TargetId::new(1)).is_some());
    }
}
