//! Extended Kalman filter over the navigation state.
//!
//! The mathematics is in [`pure`]: side-effect-free functions from state to
//! state — [`pure::predict`], [`pure::update`], and [`pure::update_late`] for
//! out-of-sequence observations. Steps are reproducible from a log and
//! comparable with a reference, as the consistency tests require. [`Estimator`]
//! is a thin mutable shell: it holds the current belief and a short history,
//! enforces monotonic time, applies observation gates, moves the local-frame
//! anchor with the vessel, tracks per-source and overall verdicts, and reports
//! events.
//!
//! The estimator knows no sensors: it takes any [`Observation`] and any
//! [`ProcessModel`]. Standard implementations are in [`crate::observations`]
//! and [`SteadyMotion`]; adapters implement the port for their own sensors.
//!
//! ```rust
//! use core::time::Duration;
//! use kinavis::estimator::{Estimator, EstimatorConfig, SteadyMotion};
//! use kinavis::observations::PositionObservation;
//! use kinavis::{Distance, GnssFix, Instant, NavigationState, Position, Speed, TrueCourse, Utc};
//!
//! let start = Instant::<Utc>::from_unix_seconds(1_700_000_000);
//! let here: Position = "50°45.3'N 001°20.0'W".parse()?;
//! let fix = GnssFix::builder(start, here)
//!     .course_over_ground(TrueCourse::new(90.0)?)
//!     .speed_over_ground(Speed::from_knots(10.0)?)
//!     .build();
//!
//! let mut estimator = Estimator::new(
//!     NavigationState::initialised_from(&fix, None)?,
//!     SteadyMotion::standard(),
//!     EstimatorConfig::standard(),
//! );
//!
//! // Ten seconds on, the receiver reports the vessel where steady motion
//! // would have put it: the update barely moves the estimate.
//! let later = start.saturating_add(Duration::from_secs(10));
//! let expected = estimator.predicted_at(later)?.position();
//! let outcome = estimator.ingest(&PositionObservation::new(
//!     later, expected, Distance::from_metres(5.0)?,
//! ))?;
//! assert!(outcome.report().map_or(false, |report| report.accepted()));
//! assert!(estimator.state().horizontal_error().semi_major().metres() < 5.0);
//! # Ok::<(), kinavis::NavigationError>(())
//! ```

mod health;
mod history;
mod motion;
pub mod pure;
#[cfg(test)]
mod tests;

use core::time::Duration;

use crate::error::{ensure_range, Result};
use crate::estimation::{Observation, ProcessModel};
use crate::event::{
    EventList, NavigationEvent, NavigationIntegrity, RejectionReason, SensorHealth, SensorId,
};
use crate::geodesy::{Ellipsoid, GeodeticPoint, Height};
use crate::local::LocalFrame;
use crate::math;
use crate::snapshot::NavigationSnapshot;
use crate::state::{NavigationState, StateComponent};
use crate::time::{Instant, Utc};
use crate::units::{Distance, METRES_PER_NAUTICAL_MILE};

use health::{IntegrityLimits, Sensors};
use history::History;

pub use health::MAX_SENSORS;
pub use history::MAX_HISTORY;
pub use motion::SteadyMotion;
pub use pure::UpdateReport;

/// Handling of observations older than the belief.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum LatePolicy {
    /// Reject as [`RejectionReason::OutOfOrder`].
    Reject,
    /// Fold in through the belief history ([`pure::update_late`]) if no older
    /// than this and within the history; otherwise reject.
    Smooth {
        /// Maximum lag behind the belief.
        max_lag: Duration,
    },
}

/// Estimator settings.
///
/// Start from [`EstimatorConfig::standard`] and adjust with `with_*`. Distances
/// are validated on input, so no limit can move the anchor every step or flag
/// every position.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(try_from = "StoredEstimatorConfig", into = "StoredEstimatorConfig")
)]
pub struct EstimatorConfig {
    reanchor_after: Distance,
    max_age: Duration,
    late: LatePolicy,
    alert_limit: Distance,
    max_unaided: Duration,
    suspect_after: u8,
}

impl EstimatorConfig {
    /// 50 km anchor distance; 10 s staleness; late observations folded in up to
    /// 2 s; 100 m alert limit; 30 s unaided; 5 consecutive rejections.
    #[must_use]
    pub const fn standard() -> Self {
        Self {
            reanchor_after: Distance::from_nautical_miles_unchecked(
                50_000.0 / METRES_PER_NAUTICAL_MILE,
            ),
            max_age: Duration::from_secs(10),
            late: LatePolicy::Smooth {
                max_lag: Duration::from_secs(2),
            },
            alert_limit: Distance::from_nautical_miles_unchecked(100.0 / METRES_PER_NAUTICAL_MILE),
            max_unaided: Duration::from_secs(30),
            suspect_after: 5,
        }
    }

    /// Sets the distance from the anchor at which the local frame is
    /// re-anchored under the vessel. The state is linearised about the anchor;
    /// errors appear at tens of kilometres.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] unless positive.
    ///
    /// [`KernelError::OutOfRange`]: crate::KernelError::OutOfRange
    pub fn with_reanchor_after(mut self, distance: Distance) -> Result<Self> {
        ensure_range(
            "reanchor distance",
            distance.metres(),
            f64::MIN_POSITIVE,
            f64::MAX,
        )?;
        self.reanchor_after = distance;
        Ok(self)
    }

    /// Sets the state age at which snapshots are marked stale.
    #[must_use]
    pub const fn with_max_age(mut self, max_age: Duration) -> Self {
        self.max_age = max_age;
        self
    }

    /// Sets the late-observation policy.
    #[must_use]
    pub const fn with_late(mut self, late: LatePolicy) -> Self {
        self.late = late;
        self
    }

    /// Sets the 1σ semi-major axis beyond which integrity is
    /// [`NavigationIntegrity::Exceeded`].
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] unless positive.
    ///
    /// [`KernelError::OutOfRange`]: crate::KernelError::OutOfRange
    pub fn with_alert_limit(mut self, limit: Distance) -> Result<Self> {
        ensure_range("alert limit", limit.metres(), f64::MIN_POSITIVE, f64::MAX)?;
        self.alert_limit = limit;
        Ok(self)
    }

    /// Sets the time without a position fix after which integrity is
    /// [`NavigationIntegrity::DeadReckoning`].
    #[must_use]
    pub const fn with_max_unaided(mut self, max_unaided: Duration) -> Self {
        self.max_unaided = max_unaided;
        self
    }

    /// Sets the number of consecutive gate rejections that make a source
    /// [`SensorHealth::Suspect`]; the same number of acceptances clears it.
    /// Zero disables.
    #[must_use]
    pub const fn with_suspect_after(mut self, rejections: u8) -> Self {
        self.suspect_after = rejections;
        self
    }

    /// Re-anchoring distance.
    #[must_use]
    pub const fn reanchor_after(&self) -> Distance {
        self.reanchor_after
    }

    /// Staleness limit.
    #[must_use]
    pub const fn max_age(&self) -> Duration {
        self.max_age
    }

    /// Late-observation policy.
    #[must_use]
    pub const fn late(&self) -> LatePolicy {
        self.late
    }

    /// 1σ semi-major axis limit for [`NavigationIntegrity::Exceeded`].
    #[must_use]
    pub const fn alert_limit(&self) -> Distance {
        self.alert_limit
    }

    /// Unaided time limit for [`NavigationIntegrity::DeadReckoning`].
    #[must_use]
    pub const fn max_unaided(&self) -> Duration {
        self.max_unaided
    }

    /// Consecutive rejections for [`SensorHealth::Suspect`]; zero if disabled.
    #[must_use]
    pub const fn suspect_after(&self) -> u8 {
        self.suspect_after
    }

    const fn integrity_limits(&self) -> IntegrityLimits {
        IntegrityLimits {
            alert_limit: self.alert_limit,
            max_unaided: self.max_unaided,
        }
    }
}

/// Serialised form; deserialisation goes through the `with_*` validation.
#[cfg(feature = "serde")]
#[derive(serde::Serialize, serde::Deserialize)]
struct StoredEstimatorConfig {
    reanchor_after: Distance,
    max_age: Duration,
    late: LatePolicy,
    alert_limit: Distance,
    max_unaided: Duration,
    suspect_after: u8,
}

#[cfg(feature = "serde")]
impl TryFrom<StoredEstimatorConfig> for EstimatorConfig {
    type Error = crate::error::NavigationError;

    fn try_from(stored: StoredEstimatorConfig) -> Result<Self> {
        Self::standard()
            .with_reanchor_after(stored.reanchor_after)?
            .with_alert_limit(stored.alert_limit)
            .map(|config| {
                config
                    .with_max_age(stored.max_age)
                    .with_late(stored.late)
                    .with_max_unaided(stored.max_unaided)
                    .with_suspect_after(stored.suspect_after)
            })
    }
}

#[cfg(feature = "serde")]
impl From<EstimatorConfig> for StoredEstimatorConfig {
    fn from(config: EstimatorConfig) -> Self {
        Self {
            reanchor_after: config.reanchor_after,
            max_age: config.max_age,
            late: config.late,
            alert_limit: config.alert_limit,
            max_unaided: config.max_unaided,
            suspect_after: config.suspect_after,
        }
    }
}

/// Step result: events and the update report, if any.
#[must_use = "the events say what happened; an unread outcome is a lost event"]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Outcome {
    events: EventList,
    report: Option<UpdateReport>,
}

impl Outcome {
    /// Events.
    pub const fn events(&self) -> &EventList {
        &self.events
    }

    /// Update report, for observation steps.
    #[must_use]
    pub const fn report(&self) -> Option<&UpdateReport> {
        self.report.as_ref()
    }
}

/// Running estimator: current belief and history, process model, and
/// bookkeeping around the pure steps.
#[derive(Debug, Clone)]
pub struct Estimator<P: ProcessModel> {
    history: History,
    process: P,
    config: EstimatorConfig,
    sensors: Sensors,
    /// Time of the last position-fixing observation.
    last_fix: Option<Instant<Utc>>,
    integrity: NavigationIntegrity,
}

impl<P: ProcessModel> Estimator<P> {
    /// Estimator from an initial state.
    ///
    /// No position fix yet, so integrity starts at
    /// [`NavigationIntegrity::DeadReckoning`] (or `Exceeded` if the initial
    /// uncertainty is beyond the limit); the first fixing observation raises
    /// [`NavigationEvent::IntegrityChanged`].
    pub fn new(initial: NavigationState, process: P, config: EstimatorConfig) -> Self {
        let integrity = health::assess(
            &initial,
            None,
            initial.valid_at(),
            &config.integrity_limits(),
        );
        Self {
            history: History::starting_with(&initial),
            process,
            config,
            sensors: Sensors::new(),
            last_fix: None,
            integrity,
        }
    }

    /// Current belief.
    #[must_use]
    pub fn state(&self) -> &NavigationState {
        self.history.current()
    }

    /// Settings.
    #[must_use]
    pub const fn config(&self) -> &EstimatorConfig {
        &self.config
    }

    /// Process model.
    #[must_use]
    pub const fn process(&self) -> &P {
        &self.process
    }

    /// Overall integrity as of the current belief.
    ///
    /// Re-evaluated only on steps; an unstepped estimator does not notice time
    /// passing. Use [`Estimator::advance_to`] to step on a clock;
    /// [`Estimator::snapshot_at`] evaluates the requested instant.
    #[must_use]
    pub const fn integrity(&self) -> NavigationIntegrity {
        self.integrity
    }

    /// Verdict on a source; `None` if not heard from.
    #[must_use]
    pub fn sensor_health(&self, sensor: SensorId) -> Option<SensorHealth> {
        self.sensors.health(sensor)
    }

    /// Belief propagated to `at`, without mutating the estimator.
    ///
    /// # Errors
    ///
    /// [`KernelError::TimeReversed`](crate::KernelError::TimeReversed) for an
    /// instant before the state; process model errors.
    pub fn predicted_at(&self, when: Instant<Utc>) -> Result<NavigationState> {
        let state = self.state();
        let over = when.duration_since(state.valid_at())?;
        if over.is_zero() {
            return Ok(*state);
        }
        pure::predict(state, &self.process, over)
    }

    /// Propagates the belief to `at`.
    ///
    /// # Errors
    ///
    /// As [`Estimator::predicted_at`]. Unchanged on error.
    pub fn advance_to(&mut self, when: Instant<Utc>) -> Result<Outcome> {
        let mut events = EventList::new();
        self.history.record(&self.predicted_at(when)?);
        self.reanchor_if_far()?;
        self.judge(&mut events);
        Ok(Outcome {
            events,
            report: None,
        })
    }

    /// Ingests an observation: propagates to its time, then corrects.
    ///
    /// Observations older than the belief follow [`EstimatorConfig::late`]:
    /// folded in via the history, or rejected as
    /// [`RejectionReason::OutOfOrder`]. Gate rejections are
    /// [`RejectionReason::Improbable`]. In both cases the belief is
    /// uncorrected, but a gated observation still propagates it in time.
    ///
    /// # Errors
    ///
    /// Process model or observation errors; a singular innovation covariance
    /// (not possible for a well-formed observation); state invariant violations
    /// when an ungated observation demands an impossible correction;
    /// [`KernelError::CapacityExceeded`](crate::KernelError::CapacityExceeded)
    /// for a source beyond [`MAX_SENSORS`]. On error the belief is at the
    /// observation time, uncorrected.
    pub fn ingest(&mut self, observation: &dyn Observation) -> Result<Outcome> {
        let mut events = EventList::new();
        let when = observation.taken_at();
        let stepped = if when < self.state().valid_at() {
            self.fold_in_late(observation).transpose()?
        } else {
            self.history.record(&self.predicted_at(when)?);
            Some(pure::update(self.state(), observation)?)
        };
        let Some((updated, report)) = stepped else {
            events.push(NavigationEvent::ObservationRejected {
                reason: RejectionReason::OutOfOrder,
                at: when,
            });
            return Ok(Outcome {
                events,
                report: None,
            });
        };
        if report.accepted() {
            self.history.record(&updated);
            if report.fixes_position() {
                self.last_fix = Some(when.max(self.last_fix.unwrap_or(when)));
            }
        } else {
            events.push(NavigationEvent::ObservationRejected {
                reason: RejectionReason::Improbable {
                    normalised_innovation_squared: report.normalised_innovation_squared(),
                },
                at: when,
            });
        }
        self.sensors.note(
            observation.sensor(),
            report.accepted(),
            self.config.suspect_after,
            when,
            &mut events,
        )?;
        self.reanchor_if_far()?;
        self.judge(&mut events);
        Ok(Outcome {
            events,
            report: Some(report),
        })
    }

    /// Read model at `at`: projected belief with age and integrity verdict as
    /// of that instant.
    #[must_use]
    pub fn snapshot_at(&self, now: Instant<Utc>) -> NavigationSnapshot {
        let state = self.state();
        let age = now.checked_duration_since(state.valid_at());
        let integrity = health::assess(state, self.last_fix, now, &self.config.integrity_limits());
        state
            .project()
            .with_integrity(integrity)
            .with_age(age, age.is_some_and(|age| age > self.config.max_age))
    }

    /// Late update under the current policy: corrected belief and report, or
    /// `None` to reject.
    fn fold_in_late(
        &self,
        observation: &dyn Observation,
    ) -> Option<Result<(NavigationState, UpdateReport)>> {
        let LatePolicy::Smooth { max_lag } = self.config.late else {
            return None;
        };
        let lag = self
            .state()
            .valid_at()
            .checked_duration_since(observation.taken_at())?;
        if lag > max_lag {
            return None;
        }
        let history = self.history.reaching(observation.taken_at())?;
        Some(pure::update_late(history, &self.process, observation))
    }

    /// Re-evaluates integrity after a step and reports changes.
    fn judge(&mut self, events: &mut EventList) {
        let state = self.state();
        let now = health::assess(
            state,
            self.last_fix,
            state.valid_at(),
            &self.config.integrity_limits(),
        );
        if now != self.integrity {
            events.push(NavigationEvent::IntegrityChanged {
                from: self.integrity,
                to: now,
                at: state.valid_at(),
            });
            self.integrity = now;
        }
    }

    /// Re-anchors the local frame under the vessel when it has moved far
    /// enough.
    ///
    /// The covariance is carried over unchanged: the frame rotates by the tiny
    /// angle between nearby meridians, far below its precision. The history is
    /// reset, since its entries are in the old frame.
    fn reanchor_if_far(&mut self) -> Result<()> {
        let state = self.state();
        let vector = state.vector();
        let north = vector.element(StateComponent::North.index()).unwrap_or(0.0);
        let east = vector.element(StateComponent::East.index()).unwrap_or(0.0);
        if math::hypot(north, east) <= self.config.reanchor_after.metres() {
            return Ok(());
        }
        let anchor = GeodeticPoint::new(state.position(), Height::above_ellipsoid(Distance::ZERO));
        let frame = LocalFrame::at(anchor, &Ellipsoid::WGS84)?;
        let mut moved = *vector;
        moved.set(StateComponent::North.index(), 0, 0.0);
        moved.set(StateComponent::East.index(), 0, 0.0);
        let moved =
            NavigationState::from_parts(state.valid_at(), frame, moved, *state.covariance())?;
        self.history.restart_with(&moved);
        Ok(())
    }
}
