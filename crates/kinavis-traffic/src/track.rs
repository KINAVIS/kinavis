//! Target track: recent observations and derived motion.

use core::time::Duration;

use kinavis::error::Result;
use kinavis::relative_motion::Vessel;
use kinavis::sailings::rhumb_destination;
use kinavis_kernel::angle::{Direction, True, TrueCourse};
use kinavis_kernel::event::TargetId;
use kinavis_kernel::inline::Inline;
use kinavis_kernel::math;
use kinavis_kernel::position::{Latitude, Longitude, Position};
use kinavis_kernel::snapshot::GroundTrack;
use kinavis_kernel::time::{Instant, Utc};
use kinavis_kernel::units::{Distance, Speed};

use crate::observation::TargetObservation;

/// Maximum fixes per track.
///
/// Enough to smooth a radar plot over a few minutes; older fixes are discarded.
/// A sliding window, not a log.
pub const MAX_TRACK_HISTORY: usize = 12;

/// Position and time of a sighting.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Fix {
    position: Position,
    at: Instant<Utc>,
}

/// Inline fix storage.
type Fixes = Inline<Fix, MAX_TRACK_HISTORY>;

/// Track acquisition status.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum TrackStatus {
    /// Fewer fixes than the policy requires; course and speed are provisional.
    Acquiring,
    /// Acquired.
    Tracking,
}

/// Target track held by the traffic picture.
///
/// Aggregate of recent fixes, last reported motion and status. Mutated only by
/// [`Traffic`].
///
/// [`Traffic`]: crate::Traffic
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TargetTrack {
    target: TargetId,
    /// Fixes in time order, latest last; never empty.
    fixes: Fixes,
    /// Copy of the latest fix, so reading it needs no non-empty proof.
    latest: Fix,
    reported: Option<GroundTrack>,
    heading: Option<TrueCourse>,
    status: TrackStatus,
}

/// Course and speed fitted to the fixes, and the fitted position at the latest
/// fix.
struct Fit {
    north_knots: f64,
    east_knots: f64,
    /// Offset of the fitted position from the latest fix, NM.
    north_offset: f64,
    east_offset: f64,
}

impl TargetTrack {
    /// Fill value for a store of tracks; never read.
    pub(crate) fn placeholder() -> Self {
        Self::started_by(&TargetObservation::new(
            TargetId::new(0),
            Position::new(Latitude::EQUATOR, Longitude::GREENWICH),
            Instant::from_unix_seconds(0),
        ))
    }

    /// Track started by one observation.
    pub(crate) fn started_by(observation: &TargetObservation) -> Self {
        let fix = Fix {
            position: observation.position(),
            at: observation.at(),
        };
        let mut fixes = Fixes::new(fix);
        // The store was made with room for one; the push cannot fail.
        let _ = fixes.push(fix);
        Self {
            target: observation.target(),
            fixes,
            latest: fix,
            reported: observation.ground_track(),
            heading: observation.heading(),
            status: TrackStatus::Acquiring,
        }
    }

    /// Appends an observation, discarding the oldest fix when full. The caller
    /// has checked it is newer than the last.
    pub(crate) fn extend(&mut self, observation: &TargetObservation) {
        let fix = Fix {
            position: observation.position(),
            at: observation.at(),
        };
        if self.fixes.push(fix).is_err() {
            let mut kept = Fixes::new(fix);
            for &old in self.fixes.iter().skip(1) {
                let _ = kept.push(old);
            }
            let _ = kept.push(fix);
            self.fixes = kept;
        }
        self.latest = fix;
        if observation.ground_track().is_some() {
            self.reported = observation.ground_track();
        }
        if observation.heading().is_some() {
            self.heading = observation.heading();
        }
    }

    /// Marks the track acquired.
    pub(crate) fn acquire(&mut self) {
        self.status = TrackStatus::Tracking;
    }

    /// Target.
    #[must_use]
    pub const fn target(&self) -> TargetId {
        self.target
    }

    /// Whether acquired.
    #[must_use]
    pub const fn status(&self) -> TrackStatus {
        self.status
    }

    /// Number of fixes, up to [`MAX_TRACK_HISTORY`].
    #[must_use]
    pub const fn fix_count(&self) -> usize {
        self.fixes.len()
    }

    /// Time of the last sighting.
    #[must_use]
    pub fn last_seen(&self) -> Instant<Utc> {
        self.last().at
    }

    /// Last observed position, unsmoothed.
    #[must_use]
    pub fn last_position(&self) -> Position {
        self.last().position
    }

    /// Time of the earliest fix in the window.
    #[must_use]
    pub fn first_seen(&self) -> Instant<Utc> {
        self.fixes
            .first()
            .map_or_else(|| self.last().at, |fix| fix.at)
    }

    /// Last course and speed reported by the target, if any.
    #[must_use]
    pub const fn reported_ground_track(&self) -> Option<GroundTrack> {
        self.reported
    }

    /// Last heading reported by the target, if any.
    #[must_use]
    pub const fn heading(&self) -> Option<TrueCourse> {
        self.heading
    }

    /// Time since the last sighting; zero if `now` is earlier.
    #[must_use]
    pub fn age(&self, now: Instant<Utc>) -> Duration {
        now.checked_duration_since(self.last_seen())
            .unwrap_or_default()
    }

    /// Course and speed over ground: as reported, otherwise fitted from the
    /// fixes.
    ///
    /// `None` for a single fix without a report.
    #[must_use]
    pub fn motion(&self) -> Option<GroundTrack> {
        self.reported.or_else(|| self.fitted_motion())
    }

    /// Course and speed fitted to the fixes, ignoring reports (the radar plot).
    ///
    /// Least-squares line of position against time, robust to a single noisy
    /// plot. `None` with fewer than two fixes or no movement.
    #[must_use]
    pub fn fitted_motion(&self) -> Option<GroundTrack> {
        let fit = self.fit()?;
        let speed = math::hypot(fit.north_knots, fit.east_knots);
        if !speed.is_finite() || speed <= 0.0 {
            return None;
        }
        Some(GroundTrack {
            course_over_ground: Direction::<True>::from_degrees_wrapped(math::to_degrees(
                math::atan2(fit.east_knots, fit.north_knots),
            )),
            speed_over_ground: Speed::from_knots_unchecked(speed),
        })
    }

    /// Extrapolated position at `when`: the smoothed position at the last fix,
    /// propagated by [`TargetTrack::motion`] forwards or backwards. Without
    /// motion, the last position.
    ///
    /// # Errors
    ///
    /// As [`rhumb_destination`]: extrapolation across a pole.
    pub fn position_at(&self, when: Instant<Utc>) -> Result<Position> {
        let base = self.smoothed_position()?;
        let Some(motion) = self.motion() else {
            return Ok(base);
        };
        let last = self.last_seen();
        let hours = match when.checked_duration_since(last) {
            Some(ahead) => ahead.as_secs_f64() / 3600.0,
            None => {
                -last
                    .checked_duration_since(when)
                    .unwrap_or_default()
                    .as_secs_f64()
                    / 3600.0
            }
        };
        let run = Distance::from_nautical_miles(motion.speed_over_ground.knots() * hours)?;
        rhumb_destination(base, motion.course_over_ground, run)
    }

    /// Target as a vessel for relative-motion calculations, if it has motion.
    #[must_use]
    pub fn as_vessel(&self) -> Option<Vessel> {
        self.motion().map(|motion| Vessel {
            course: motion.course_over_ground,
            speed: motion.speed_over_ground,
        })
    }

    /// Latest fix; a track always has one.
    const fn last(&self) -> Fix {
        self.latest
    }

    /// Fitted position at the last fix; the fix itself if there is no fit or it
    /// cannot be placed on the globe.
    fn smoothed_position(&self) -> Result<Position> {
        let last = self.last();
        let Some(fit) = self.fit() else {
            return Ok(last.position);
        };
        let latitude = last.position.latitude().degrees() + fit.north_offset / 60.0;
        let stretch = math::cos(last.position.latitude().radians());
        if stretch < 1e-6 {
            return Ok(last.position);
        }
        let longitude = last.position.longitude().degrees() + fit.east_offset / (60.0 * stretch);
        if !(-90.0..=90.0).contains(&latitude) {
            return Ok(last.position);
        }
        Ok(Position::from_degrees(latitude, longitude)?)
    }

    /// Least-squares line through the fixes in a local plane about the latest:
    /// NM north and east vs hours before it.
    fn fit(&self) -> Option<Fit> {
        if self.fixes.len() < 2 {
            return None;
        }
        let last = self.last();
        let count = math::count_to_f64(self.fixes.len());
        let stretch = math::cos(last.position.latitude().radians());

        let mut sum_t = 0.0;
        let mut sum_n = 0.0;
        let mut sum_e = 0.0;
        for fix in self.fixes.iter() {
            let (t, n, e) = local(fix, last, stretch);
            sum_t += t;
            sum_n += n;
            sum_e += e;
        }
        let (mean_t, mean_n, mean_e) = (sum_t / count, sum_n / count, sum_e / count);

        let mut s_tt = 0.0;
        let mut s_tn = 0.0;
        let mut s_te = 0.0;
        for fix in self.fixes.iter() {
            let (t, n, e) = local(fix, last, stretch);
            s_tt += (t - mean_t) * (t - mean_t);
            s_tn += (t - mean_t) * (n - mean_n);
            s_te += (t - mean_t) * (e - mean_e);
        }
        if s_tt <= 0.0 || !s_tt.is_finite() {
            return None;
        }
        let north_knots = s_tn / s_tt;
        let east_knots = s_te / s_tt;
        Some(Fit {
            north_knots,
            east_knots,
            north_offset: mean_n - north_knots * mean_t,
            east_offset: mean_e - east_knots * mean_t,
        })
    }
}

/// Fix in the local plane: hours (negative, before the latest), NM north, NM
/// east.
fn local(fix: &Fix, last: Fix, stretch: f64) -> (f64, f64, f64) {
    let hours = -last
        .at
        .checked_duration_since(fix.at)
        .unwrap_or_default()
        .as_secs_f64()
        / 3600.0;
    let north = last.position.latitude_difference(fix.position).degrees() * 60.0;
    let east = last.position.longitude_difference(fix.position).degrees() * 60.0 * stretch;
    (hours, north, east)
}
