//! Position from a GNSS fix stream.
//!
//! Receivers emit non-fixes (sky lost), out-of-order or late fixes, and fixes
//! implying impossible jumps. From the rest the system needs the current
//! position, its quality, and notification when the position is acquired or
//! lost.
//!
//! [`GnssIntake`] keeps the last accepted fix, validates each new one against
//! it using the vessel thresholds in [`IntakeConfig`], and returns
//! [`NavigationEvent`]s in an [`IntakeOutcome`]; no callbacks.
//! [`GnssIntake::snapshot_at`] answers at a caller-chosen instant and decides
//! staleness, which depends on the evaluation time, not the fix.
//!
//! ```rust
//! use core::time::Duration;
//! use kinavis::gnss_intake::{GnssIntake, IntakeConfig};
//! use kinavis::{Civil, GnssFix, Instant, NavigationEvent, Position, Speed, Utc};
//!
//! let mut intake = GnssIntake::new(IntakeConfig {
//!     max_age: Duration::from_secs(10),
//!     max_speed: Speed::from_knots(40.0)?,
//! });
//!
//! let noon = Instant::<Utc>::from_civil(Civil::date(2026, 9, 11))?;
//! let here: Position = "50°45.3'N 001°20.0'W".parse()?;
//! let outcome = intake.accept(GnssFix::builder(noon, here).build());
//! assert!(outcome.accepted());
//! assert!(matches!(outcome.events()[0], NavigationEvent::FixAcquired { .. }));
//!
//! let snapshot = intake.snapshot_at(noon.saturating_add(Duration::from_secs(3)));
//! assert_eq!(snapshot.position().map(|p| *p.value()), Some(here));
//! assert!(!snapshot.is_stale());
//! # Ok::<(), kinavis::NavigationError>(())
//! ```

use core::time::Duration;

use crate::error::Result;
use crate::event::{EventList, NavigationEvent, PositionSource, RejectionReason};
use crate::gnss::GnssFix;
use crate::navigation_solutions::GroundTrack;
use crate::position::Position;
use crate::sailings::great_circle;
use crate::snapshot::ErrorEllipse;
use crate::time::{Instant, Utc};
use crate::units::{hours, Speed};

/// Intake thresholds: vessel settings.
///
/// A launch and a container ship differ in plausible speed; a chart display and
/// an autopilot differ in acceptable position age.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct IntakeConfig {
    /// Maximum age of the last fix before the position is lost.
    pub max_age: Duration,
    /// Maximum plausible speed; a fix requiring more from the last one is
    /// rejected as an implausible jump.
    pub max_speed: Speed,
}

/// Intake state.
#[derive(Debug, Clone, Copy)]
pub struct GnssIntake {
    config: IntakeConfig,
    /// Last two accepted fixes, for a ground track when the receiver reports
    /// none.
    last: Option<GnssFix>,
    previous: Option<GnssFix>,
    /// Whether the position is held: set by an accepted fix, cleared when it
    /// goes stale; transitions raise `FixAcquired` and `FixLost`.
    delivering: bool,
}

/// Result of offering a fix.
#[must_use = "the events say what happened; an unread outcome is a lost event"]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct IntakeOutcome {
    accepted: bool,
    events: EventList,
}

impl IntakeOutcome {
    /// Whether the fix was accepted.
    #[must_use]
    pub const fn accepted(&self) -> bool {
        self.accepted
    }

    /// Events in order: loss, acquisition, rejection with reason.
    pub const fn events(&self) -> &EventList {
        &self.events
    }
}

// The read model is defined in the kernel so estimator and intake project into
// the same type; re-exported here.
pub use kinavis_kernel::snapshot::NavigationSnapshot;

impl GnssIntake {
    /// Empty intake.
    #[must_use]
    pub const fn new(config: IntakeConfig) -> Self {
        Self {
            config,
            last: None,
            previous: None,
            delivering: false,
        }
    }

    /// Thresholds.
    #[must_use]
    pub const fn config(&self) -> &IntakeConfig {
        &self.config
    }

    /// Last accepted fix, if any.
    #[must_use]
    pub const fn last_fix(&self) -> Option<&GnssFix> {
        self.last.as_ref()
    }

    /// Offers a fix.
    ///
    /// Rejected (and reported) if the receiver does not vouch for it, if it is
    /// older than the last accepted fix, or if reaching it would exceed
    /// [`IntakeConfig::max_speed`]. First, a last fix that is stale at the new
    /// fix's time is dropped, so a fix after a long gap reports loss, then
    /// acquisition.
    pub fn accept(&mut self, fix: GnssFix) -> IntakeOutcome {
        let mut events = EventList::new();
        self.expire(fix.taken_at(), &mut events);

        if let Some(reason) = self.reason_to_reject(&fix) {
            events.push(NavigationEvent::ObservationRejected {
                reason,
                at: fix.taken_at(),
            });
            return IntakeOutcome {
                accepted: false,
                events,
            };
        }

        if !self.delivering {
            events.push(NavigationEvent::FixAcquired {
                source: PositionSource::Gnss,
                at: fix.taken_at(),
            });
            self.delivering = true;
        }
        self.previous = self.last;
        self.last = Some(fix);
        IntakeOutcome {
            accepted: true,
            events,
        }
    }

    /// Checks for loss without a new fix.
    ///
    /// A silent receiver sends nothing, so [`GnssIntake::accept`] never runs.
    /// Call this from the caller's clock (e.g. every second) to report the loss
    /// when it occurs.
    pub fn check_at(&mut self, now: Instant<Utc>) -> EventList {
        let mut events = EventList::new();
        self.expire(now, &mut events);
        events
    }

    /// Position at an instant.
    ///
    /// Does not mutate the intake; call as often as the display refreshes.
    /// Losses it reveals are reported by [`GnssIntake::check_at`].
    #[must_use]
    pub fn snapshot_at(&self, now: Instant<Utc>) -> NavigationSnapshot {
        let Some(last) = self.last else {
            return NavigationSnapshot::EMPTY;
        };
        let age = now.checked_duration_since(last.taken_at());
        let mut snapshot = NavigationSnapshot::EMPTY
            .with_position(last.observed_position(), PositionSource::Gnss)
            .with_age(age, age.is_some_and(|age| age > self.config.max_age));
        if let Some(track) = self.ground_track(&last) {
            snapshot = snapshot.with_ground_track(track);
        }
        if let Some(sigma) = last.horizontal_accuracy() {
            snapshot = snapshot.with_horizontal_error(ErrorEllipse::circular(sigma));
        }
        snapshot
    }

    /// Drops a held position older than the limit at `now`.
    fn expire(&mut self, now: Instant<Utc>, events: &mut EventList) {
        let Some(last) = self.last else { return };
        if !self.delivering {
            return;
        }
        let stale = now
            .checked_duration_since(last.taken_at())
            .is_some_and(|age| age > self.config.max_age);
        if stale {
            self.delivering = false;
            events.push(NavigationEvent::FixLost {
                source: PositionSource::Gnss,
                // The time the position became stale, not when it was noticed.
                at: last.taken_at().saturating_add(self.config.max_age),
                last_good: last.taken_at(),
            });
        }
    }

    /// Rejection reason, if any.
    fn reason_to_reject(&self, fix: &GnssFix) -> Option<RejectionReason> {
        if !fix.quality().fix_type().is_position_fix() {
            return Some(RejectionReason::Invalid);
        }
        let last = self.last?;
        if fix.taken_at() < last.taken_at() {
            return Some(RejectionReason::OutOfOrder);
        }
        // Jumps are judged only against a held position: after a loss, any
        // position reachable in the gap is plausible.
        if !self.delivering {
            return None;
        }
        let elapsed = fix.taken_at().checked_duration_since(last.taken_at())?;
        let implied = implied_speed(last.position(), fix.position(), elapsed).ok()??;
        (implied > self.config.max_speed).then_some(RejectionReason::ImplausibleJump {
            implied_speed: implied,
        })
    }

    /// Ground track for the last fix: from the receiver, or derived from the
    /// previous fix.
    fn ground_track(&self, last: &GnssFix) -> Option<GroundTrack> {
        if let (Some(course_over_ground), Some(speed_over_ground)) =
            (last.course_over_ground(), last.speed_over_ground())
        {
            return Some(GroundTrack {
                course_over_ground,
                speed_over_ground,
            });
        }
        let previous = self.previous?;
        let elapsed = last
            .taken_at()
            .checked_duration_since(previous.taken_at())?;
        let sailing = great_circle(previous.position(), last.position()).ok()?;
        let speed_over_ground =
            implied_speed(previous.position(), last.position(), elapsed).ok()??;
        Some(GroundTrack {
            course_over_ground: sailing.initial_course,
            speed_over_ground,
        })
    }
}

/// Speed required between two positions in `elapsed`; `None` for zero time.
fn implied_speed(from: Position, to: Position, elapsed: Duration) -> Result<Option<Speed>> {
    let elapsed_hours = hours(elapsed);
    if elapsed_hours <= 0.0 {
        return Ok(None);
    }
    let distance = great_circle(from, to)?.distance;
    Ok(Speed::from_knots(distance.nautical_miles() / elapsed_hours).map(Some)?)
}

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::indexing_slicing,
    clippy::panic,
    clippy::float_cmp
)]
mod tests {
    use super::*;
    use crate::gnss::FixType;
    use crate::TrueCourse;

    fn config() -> IntakeConfig {
        IntakeConfig {
            max_age: Duration::from_secs(10),
            max_speed: Speed::from_knots(30.0).unwrap(),
        }
    }

    fn at(seconds: i64) -> Instant<Utc> {
        Instant::from_unix_seconds(seconds)
    }

    /// Fix `seconds` after the epoch, `minutes_north` of 50°N on the Greenwich
    /// meridian.
    fn fix(seconds: i64, minutes_north: f64) -> GnssFix {
        let position = Position::new(
            crate::Latitude::from_degrees(50.0 + minutes_north / 60.0).unwrap(),
            crate::Longitude::from_degrees(0.0).unwrap(),
        );
        GnssFix::builder(at(seconds), position).build()
    }

    #[test]
    fn the_first_fix_acquires_and_the_next_does_not_repeat_it() {
        let mut intake = GnssIntake::new(config());
        let first = intake.accept(fix(0, 0.0));
        assert!(first.accepted());
        assert_eq!(first.events().len(), 1);
        assert!(matches!(
            first.events()[0],
            NavigationEvent::FixAcquired {
                source: PositionSource::Gnss,
                ..
            }
        ));
        let second = intake.accept(fix(1, 0.0));
        assert!(second.accepted());
        assert!(second.events().is_empty());
    }

    #[test]
    fn a_fix_the_receiver_does_not_vouch_for_is_rejected() {
        let mut intake = GnssIntake::new(config());
        let position = fix(0, 0.0).position();
        let outcome = intake.accept(
            GnssFix::builder(at(0), position)
                .fix_type(FixType::None)
                .build(),
        );
        assert!(!outcome.accepted());
        assert!(matches!(
            outcome.events()[0],
            NavigationEvent::ObservationRejected {
                reason: RejectionReason::Invalid,
                ..
            }
        ));
        assert!(intake.snapshot_at(at(1)).position().is_none());
    }

    #[test]
    fn out_of_order_fixes_are_rejected_and_equal_times_are_not() {
        let mut intake = GnssIntake::new(config());
        let _ = intake.accept(fix(5, 0.0));
        let late = intake.accept(fix(4, 0.0));
        assert!(matches!(
            late.events()[0],
            NavigationEvent::ObservationRejected {
                reason: RejectionReason::OutOfOrder,
                ..
            }
        ));
        assert!(intake.accept(fix(5, 0.0)).accepted());
    }

    #[test]
    fn a_jump_faster_than_the_vessel_is_rejected_with_the_speed_it_needed() {
        let mut intake = GnssIntake::new(config());
        let _ = intake.accept(fix(0, 0.0));
        // 1′ of latitude in 1 s: ~3600 kn (on the spherical model 1′ = 1853 m).
        let outcome = intake.accept(fix(1, 1.0));
        assert!(!outcome.accepted());
        let NavigationEvent::ObservationRejected {
            reason: RejectionReason::ImplausibleJump { implied_speed },
            ..
        } = outcome.events()[0]
        else {
            panic!("expected a jump: {:?}", outcome.events());
        };
        assert!((implied_speed.knots() - 3600.0).abs() < 5.0);
        // The held position is still the first.
        assert_eq!(intake.last_fix().unwrap().taken_at(), at(0));
    }

    #[test]
    fn a_gap_longer_than_the_limit_is_a_loss_then_an_acquisition() {
        let mut intake = GnssIntake::new(config());
        let _ = intake.accept(fix(0, 0.0));
        // Far away but after a long gap: no jump check.
        let outcome = intake.accept(fix(100, 30.0));
        assert!(outcome.accepted());
        assert_eq!(outcome.events().len(), 2);
        assert!(matches!(
            outcome.events()[0],
            NavigationEvent::FixLost {
                source: PositionSource::Gnss,
                at,
                last_good,
            } if at == at_secs(10) && last_good == at_secs(0)
        ));
        assert!(matches!(
            outcome.events()[1],
            NavigationEvent::FixAcquired { .. }
        ));
    }

    fn at_secs(seconds: i64) -> Instant<Utc> {
        at(seconds)
    }

    #[test]
    fn silence_is_noticed_by_the_clock() {
        let mut intake = GnssIntake::new(config());
        let _ = intake.accept(fix(0, 0.0));
        assert!(intake.check_at(at(10)).is_empty());
        let events = intake.check_at(at(11));
        assert!(matches!(events[0], NavigationEvent::FixLost { .. }));
        // Reported once, not every tick.
        assert!(intake.check_at(at(12)).is_empty());
        // The snapshot still provides the old position, marked stale.
        let snapshot = intake.snapshot_at(at(12));
        assert!(snapshot.position().is_some());
        assert!(snapshot.is_stale());
        assert_eq!(snapshot.age(), Some(Duration::from_secs(12)));
        assert_eq!(snapshot.source(), Some(PositionSource::Gnss));
    }

    #[test]
    fn the_ground_track_comes_from_the_receiver_or_from_two_fixes() {
        let mut intake = GnssIntake::new(config());
        assert!(intake.snapshot_at(at(0)).ground_track().is_none());
        let _ = intake.accept(fix(0, 0.0));
        // One fix: no derived track.
        assert!(intake.snapshot_at(at(0)).ground_track().is_none());
        // 1 NM north in 6 min: 10 kn on 000°.
        let _ = intake.accept(fix(360, 1.0));
        let track = intake.snapshot_at(at(360)).ground_track().unwrap();
        assert!((track.speed_over_ground.knots() - 10.0).abs() < 0.01);
        assert!(track.course_over_ground.degrees() < 0.01);

        // The receiver's own track takes precedence.
        let position = fix(361, 1.0).position();
        let _ = intake.accept(
            GnssFix::builder(at(361), position)
                .course_over_ground(TrueCourse::new(90.0).unwrap())
                .speed_over_ground(Speed::from_knots(7.0).unwrap())
                .build(),
        );
        let track = intake.snapshot_at(at(361)).ground_track().unwrap();
        assert_eq!(track.course_over_ground.degrees(), 90.0);
        assert_eq!(track.speed_over_ground.knots(), 7.0);
    }

    #[test]
    fn a_snapshot_before_the_fix_has_no_age_and_is_not_stale() {
        let mut intake = GnssIntake::new(config());
        let _ = intake.accept(fix(100, 0.0));
        let snapshot = intake.snapshot_at(at(50));
        assert_eq!(snapshot.age(), None);
        assert!(!snapshot.is_stale());
    }
}
