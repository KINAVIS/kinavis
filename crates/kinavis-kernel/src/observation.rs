//! Observed value: time and quality.
//!
//! A sensor reading is a statement about the world at an instant, with an
//! uncertainty, from a source that may be degraded. Displays, estimators and
//! alarms all need the same three facts, so they are kept in one wrapper
//! instead of repeated on every reading type.
//!
//! Age is not stored: it depends on the evaluation time and would be wrong
//! immediately. Use [`Observed::age_at`].
//!
//! ```rust
//! use core::time::Duration;
//! use kinavis_kernel::observation::{Observed, ObservationStatus, Quality};
//! use kinavis_kernel::time::{Civil, Instant, Utc};
//! use kinavis_kernel::{Distance, Position};
//!
//! let taken_at = Instant::<Utc>::from_civil(Civil::date(2026, 9, 11))?;
//! let fix = Observed::new(
//!     "50°45.3'N 001°20.0'W".parse::<Position>()?,
//!     taken_at,
//!     Quality::new(ObservationStatus::Valid).with_sigma(Distance::from_metres(5.0)?),
//! );
//!
//! let now = taken_at.saturating_add(Duration::from_secs(90));
//! assert_eq!(fix.age_at(now)?, Duration::from_secs(90));
//! assert!(fix.is_stale_at(now, Duration::from_secs(60)));
//! assert!(!fix.is_stale_at(now, Duration::from_secs(120)));
//! # Ok::<(), kinavis_kernel::KernelError>(())
//! ```

use core::time::Duration;

use crate::error::Result;
use crate::time::{Instant, Utc};

/// Source-reported validity of a reading.
///
/// What the source reports, not a judgement of the value (a GNSS fix the
/// receiver distrusts, a settling gyro, a stopped log impeller). Handling of
/// `Suspect` is the consumer's decision; `Invalid` carries no information.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum ObservationStatus {
    /// Valid.
    Valid,
    /// Flagged by the source: use with caution or not at all.
    Suspect,
    /// No usable reading; the value is a placeholder.
    Invalid,
}

impl ObservationStatus {
    /// Whether the reading carries information: anything but
    /// [`Invalid`](ObservationStatus::Invalid).
    #[must_use]
    pub const fn is_usable(self) -> bool {
        !matches!(self, Self::Invalid)
    }
}

/// Reading quality: status and optional 1σ uncertainty.
///
/// The uncertainty has the type of the quantity it bounds — [`Distance`] for a
/// position, [`Angle`] for a course, [`Speed`] for a speed — so units cannot be
/// confused. `U = ()` for sources without an error estimate.
///
/// [`Distance`]: crate::Distance
/// [`Angle`]: crate::Angle
/// [`Speed`]: crate::Speed
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Quality<U = ()> {
    status: ObservationStatus,
    sigma: Option<U>,
}

impl<U> Quality<U> {
    /// Quality with status only.
    #[must_use]
    pub const fn new(status: ObservationStatus) -> Self {
        Self {
            status,
            sigma: None,
        }
    }

    /// Adds a 1σ uncertainty.
    #[must_use]
    pub fn with_sigma(self, sigma: U) -> Self {
        Self {
            sigma: Some(sigma),
            ..self
        }
    }

    /// Status.
    #[must_use]
    pub const fn status(&self) -> ObservationStatus {
        self.status
    }

    /// 1σ uncertainty, if given.
    #[must_use]
    pub const fn sigma(&self) -> Option<&U> {
        self.sigma.as_ref()
    }
}

impl<U> Default for Quality<U> {
    /// `Valid` without uncertainty: the default for sources that report
    /// neither.
    fn default() -> Self {
        Self::new(ObservationStatus::Valid)
    }
}

/// Value with its observation time and quality.
///
/// `T` is the value; `U` the uncertainty type (see [`Quality`]). Transparent to
/// the value ([`Observed::value`], [`Observed::map`]); answers age and
/// staleness.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Observed<T, U = ()> {
    value: T,
    taken_at: Instant<Utc>,
    quality: Quality<U>,
}

impl<T, U> Observed<T, U> {
    /// Wraps a value with its time and quality.
    #[must_use]
    pub const fn new(value: T, taken_at: Instant<Utc>, quality: Quality<U>) -> Self {
        Self {
            value,
            taken_at,
            quality,
        }
    }

    /// Value.
    #[must_use]
    pub const fn value(&self) -> &T {
        &self.value
    }

    /// Unwraps the value.
    #[must_use]
    pub fn into_value(self) -> T {
        self.value
    }

    /// Observation time.
    #[must_use]
    pub const fn taken_at(&self) -> Instant<Utc> {
        self.taken_at
    }

    /// Quality.
    #[must_use]
    pub const fn quality(&self) -> &Quality<U> {
        &self.quality
    }

    /// Age at `now`.
    ///
    /// Computed, never stored.
    ///
    /// # Errors
    ///
    /// [`KernelError::TimeReversed`](crate::KernelError::TimeReversed) if `now`
    /// precedes the observation (clock stepped back, or a future timestamp).
    pub fn age_at(&self, now: Instant<Utc>) -> Result<Duration> {
        now.duration_since(self.taken_at)
    }

    /// Whether older than `limit` at `now`.
    ///
    /// `false` for a reading timestamped after `now`; that error is reported by
    /// [`Observed::age_at`].
    #[must_use]
    pub fn is_stale_at(&self, now: Instant<Utc>, limit: Duration) -> bool {
        now.checked_duration_since(self.taken_at)
            .is_some_and(|age| age > limit)
    }

    /// Derives another value, keeping time and quality.
    ///
    /// For quantities computed from the value alone (e.g. course from a fix
    /// velocity). A derivation that changes the uncertainty should build a new
    /// [`Observed`].
    #[must_use]
    pub fn map<V>(self, derive: impl FnOnce(T) -> V) -> Observed<V, U> {
        Observed {
            value: derive(self.value),
            taken_at: self.taken_at,
            quality: self.quality,
        }
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use crate::error::KernelError;
    use crate::units::Speed;

    fn at(seconds: i64) -> Instant<Utc> {
        Instant::from_unix_seconds(seconds)
    }

    #[test]
    fn age_is_computed_from_two_moments() {
        let reading = Observed::<_, ()>::new(12.5_f64, at(100), Quality::default());
        assert_eq!(reading.age_at(at(160)).unwrap(), Duration::from_secs(60));
        assert_eq!(reading.age_at(at(100)).unwrap(), Duration::ZERO);
        assert_eq!(
            reading.age_at(at(90)),
            Err(KernelError::TimeReversed {
                by: Duration::from_secs(10)
            })
        );
    }

    #[test]
    fn staleness_is_age_beyond_a_limit_and_never_from_the_future() {
        let reading = Observed::<_, ()>::new((), at(100), Quality::default());
        let limit = Duration::from_secs(30);
        assert!(!reading.is_stale_at(at(130), limit));
        assert!(reading.is_stale_at(at(131), limit));
        assert!(!reading.is_stale_at(at(50), limit));
    }

    #[test]
    fn quality_carries_a_typed_uncertainty() {
        let sigma = Speed::from_knots(0.2).unwrap();
        let quality = Quality::new(ObservationStatus::Suspect).with_sigma(sigma);
        assert_eq!(quality.status(), ObservationStatus::Suspect);
        assert_eq!(quality.sigma(), Some(&sigma));
        assert!(quality.status().is_usable());
        assert!(!ObservationStatus::Invalid.is_usable());
        assert_eq!(Quality::<Speed>::default().sigma(), None);
    }

    #[test]
    fn map_keeps_the_moment_and_the_quality() {
        let quality =
            Quality::new(ObservationStatus::Valid).with_sigma(Speed::from_knots(0.2).unwrap());
        let speed = Observed::new(Speed::from_knots(10.0).unwrap(), at(100), quality);
        let doubled = speed.map(|s| s * 2.0);
        assert_eq!(doubled.value().knots(), 20.0);
        assert_eq!(doubled.taken_at(), at(100));
        assert_eq!(doubled.quality(), &quality);
        assert_eq!(doubled.into_value().knots(), 20.0);
    }

    #[cfg(feature = "serde")]
    #[test]
    fn serde_round_trips() {
        let quality =
            Quality::new(ObservationStatus::Valid).with_sigma(Speed::from_knots(0.2).unwrap());
        let speed = Observed::new(Speed::from_knots(10.0).unwrap(), at(100), quality);
        let json = serde_json::to_string(&speed).unwrap();
        assert_eq!(
            serde_json::from_str::<Observed<Speed, Speed>>(&json).unwrap(),
            speed
        );
    }
}
