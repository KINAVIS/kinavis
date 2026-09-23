//! Read model of the navigation solution.
//!
//! Estimator (state + covariance) or intake (last fix): displays, autopilots
//! and alarms need the same answers — position, speed, direction, uncertainty,
//! age. [`NavigationSnapshot`] projects them from either producer, with no
//! internal representation: uncertainty is an [`ErrorEllipse`], not a
//! covariance matrix.

use core::fmt;
use core::time::Duration;

use crate::angle::TrueCourse;
use crate::error::{ensure_range, Result};
use crate::event::{NavigationIntegrity, PositionSource};
use crate::math;
use crate::observation::Observed;
use crate::position::Position;
use crate::units::{Angle, Distance, Speed};

/// Course and speed over ground.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct GroundTrack {
    /// Course over ground.
    pub course_over_ground: TrueCourse,
    /// Speed over ground.
    pub speed_over_ground: Speed,
}

/// 1σ horizontal position error ellipse.
///
/// Semi-axes and true direction of the major axis of a 2 × 2 covariance. Scale
/// axes by 2.45 for 95 %, by 1.18 for an equal-probability circle when the axes
/// are nearly equal.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(try_from = "StoredErrorEllipse", into = "StoredErrorEllipse")
)]
pub struct ErrorEllipse {
    semi_major: Distance,
    semi_minor: Distance,
    orientation: TrueCourse,
}

impl ErrorEllipse {
    /// Ellipse of a north/east covariance, m².
    ///
    /// `None` unless a valid covariance: finite, non-negative variances,
    /// |correlation| ≤ 1.
    #[must_use]
    pub fn from_covariance(north: f64, north_east: f64, east: f64) -> Option<Self> {
        if !(north.is_finite() && north_east.is_finite() && east.is_finite())
            || north < 0.0
            || east < 0.0
            || north_east * north_east > north * east * (1.0 + 1e-9)
        {
            return None;
        }
        // Eigenvalues of the symmetric 2 × 2 matrix.
        let half_trace = f64::midpoint(north, east);
        let half_difference = 0.5 * (north - east);
        let radius = math::hypot(half_difference, north_east);
        let major = (half_trace + radius).max(0.0);
        let minor = (half_trace - radius).max(0.0);
        // Major-axis direction, from north towards east.
        let orientation = if radius < f64::MIN_POSITIVE {
            0.0
        } else {
            0.5 * math::atan2(2.0 * north_east, north - east)
        };
        Some(Self {
            semi_major: Distance::from_metres(math::sqrt(major)).ok()?,
            semi_minor: Distance::from_metres(math::sqrt(minor)).ok()?,
            orientation: TrueCourse::wrap(math::to_degrees(orientation)).ok()?,
        })
    }

    /// Ellipse from semi-axes and major-axis direction.
    ///
    /// # Errors
    ///
    /// [`crate::error::KernelError::OutOfRange`] for a negative axis or minor >
    /// major.
    pub fn new(
        semi_major: Distance,
        semi_minor: Distance,
        orientation: TrueCourse,
    ) -> Result<Self> {
        ensure_range("semi-major axis", semi_major.metres(), 0.0, f64::MAX)?;
        ensure_range(
            "semi-minor axis",
            semi_minor.metres(),
            0.0,
            semi_major.metres(),
        )?;
        Ok(Self {
            semi_major,
            semi_minor,
            orientation,
        })
    }

    /// Circle of the given radius; a negative radius is taken by magnitude.
    #[must_use]
    pub fn circular(radius: Distance) -> Self {
        let radius = Distance::from_metres(math::abs(radius.metres())).unwrap_or(radius);
        Self {
            semi_major: radius,
            semi_minor: radius,
            orientation: TrueCourse::NORTH,
        }
    }

    /// 1σ semi-major axis.
    #[must_use]
    pub const fn semi_major(&self) -> Distance {
        self.semi_major
    }

    /// 1σ semi-minor axis.
    #[must_use]
    pub const fn semi_minor(&self) -> Distance {
        self.semi_minor
    }

    /// True direction of the major axis, in `[0°, 180°)` (axes have no sense).
    #[must_use]
    pub fn orientation(&self) -> TrueCourse {
        let degrees = self.orientation.degrees();
        if degrees >= 180.0 {
            TrueCourse::wrap(degrees - 180.0).unwrap_or(self.orientation)
        } else {
            self.orientation
        }
    }

    /// Radius of the circle of equal area, for displays with no room for an
    /// ellipse.
    #[must_use]
    pub fn equivalent_radius(&self) -> Distance {
        Distance::from_metres(math::sqrt(
            self.semi_major.metres() * self.semi_minor.metres(),
        ))
        .unwrap_or(Distance::ZERO)
    }
}

/// Serialised form; deserialisation goes through [`ErrorEllipse::new`].
#[cfg(feature = "serde")]
#[derive(serde::Serialize, serde::Deserialize)]
struct StoredErrorEllipse {
    semi_major: Distance,
    semi_minor: Distance,
    orientation: TrueCourse,
}

#[cfg(feature = "serde")]
impl TryFrom<StoredErrorEllipse> for ErrorEllipse {
    type Error = crate::error::KernelError;

    fn try_from(stored: StoredErrorEllipse) -> Result<Self> {
        Self::new(stored.semi_major, stored.semi_minor, stored.orientation)
    }
}

#[cfg(feature = "serde")]
impl From<ErrorEllipse> for StoredErrorEllipse {
    fn from(ellipse: ErrorEllipse) -> Self {
        Self {
            semi_major: ellipse.semi_major,
            semi_minor: ellipse.semi_minor,
            orientation: ellipse.orientation,
        }
    }
}

impl fmt::Display for ErrorEllipse {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{:.1} m × {:.1} m at {:.0}",
            self.semi_major.metres(),
            self.semi_minor.metres(),
            self.orientation()
        )
    }
}

/// Vessel state at an instant, as far as the producer knows.
///
/// Every field except the stale flag is optional: before the first fix nothing
/// is known; an intake provides no heading. Consumers show missing fields as
/// unknown.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NavigationSnapshot {
    position: Option<Observed<Position, Distance>>,
    source: Option<PositionSource>,
    ground_track: Option<GroundTrack>,
    heading: Option<TrueCourse>,
    heading_sigma: Option<Angle>,
    horizontal_error: Option<ErrorEllipse>,
    integrity: Option<NavigationIntegrity>,
    age: Option<Duration>,
    stale: bool,
}

impl NavigationSnapshot {
    /// Empty snapshot.
    pub const EMPTY: Self = Self {
        position: None,
        source: None,
        ground_track: None,
        heading: None,
        heading_sigma: None,
        horizontal_error: None,
        integrity: None,
        age: None,
        stale: false,
    };

    /// Sets position and source.
    #[must_use]
    pub const fn with_position(
        mut self,
        position: Observed<Position, Distance>,
        source: PositionSource,
    ) -> Self {
        self.position = Some(position);
        self.source = Some(source);
        self
    }

    /// Sets course and speed over ground.
    #[must_use]
    pub const fn with_ground_track(mut self, track: GroundTrack) -> Self {
        self.ground_track = Some(track);
        self
    }

    /// Sets heading and its uncertainty.
    #[must_use]
    pub const fn with_heading(mut self, heading: TrueCourse, sigma: Option<Angle>) -> Self {
        self.heading = Some(heading);
        self.heading_sigma = sigma;
        self
    }

    /// Sets the position error ellipse.
    #[must_use]
    pub const fn with_horizontal_error(mut self, ellipse: ErrorEllipse) -> Self {
        self.horizontal_error = Some(ellipse);
        self
    }

    /// Sets the producer's integrity verdict.
    #[must_use]
    pub const fn with_integrity(mut self, integrity: NavigationIntegrity) -> Self {
        self.integrity = Some(integrity);
        self
    }

    /// Sets position age and staleness.
    #[must_use]
    pub const fn with_age(mut self, age: Option<Duration>, stale: bool) -> Self {
        self.age = age;
        self.stale = stale;
        self
    }

    /// Position with time and quality; `None` if unknown.
    #[must_use]
    pub const fn position(&self) -> Option<&Observed<Position, Distance>> {
        self.position.as_ref()
    }

    /// Position source, if any.
    #[must_use]
    pub const fn source(&self) -> Option<PositionSource> {
        self.source
    }

    /// Course and speed over ground, if known.
    #[must_use]
    pub const fn ground_track(&self) -> Option<GroundTrack> {
        self.ground_track
    }

    /// Heading (bow direction, distinct from COG), if known.
    #[must_use]
    pub const fn heading(&self) -> Option<TrueCourse> {
        self.heading
    }

    /// 1σ heading uncertainty, if known.
    #[must_use]
    pub const fn heading_sigma(&self) -> Option<Angle> {
        self.heading_sigma
    }

    /// 1σ position error ellipse, if known.
    #[must_use]
    pub const fn horizontal_error(&self) -> Option<ErrorEllipse> {
        self.horizontal_error
    }

    /// Integrity, if assessed by the producer (an intake reports a source, an
    /// estimator reports integrity).
    #[must_use]
    pub const fn integrity(&self) -> Option<NavigationIntegrity> {
        self.integrity
    }

    /// Position age at snapshot time; `None` without a position or if the
    /// position is later than the snapshot.
    #[must_use]
    pub const fn age(&self) -> Option<Duration> {
        self.age
    }

    /// Whether the position is older than the producer's limit.
    ///
    /// A stale position is still reported as the best available, but must not
    /// be presented as current.
    #[must_use]
    pub const fn is_stale(&self) -> bool {
        self.stale
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use alloc::format;

    #[test]
    fn a_diagonal_covariance_gives_axes_along_north_and_east() {
        let ellipse = ErrorEllipse::from_covariance(16.0, 0.0, 4.0).unwrap();
        assert_eq!(ellipse.semi_major().metres(), 4.0);
        assert_eq!(ellipse.semi_minor().metres(), 2.0);
        assert_eq!(ellipse.orientation().degrees(), 0.0);
        let across = ErrorEllipse::from_covariance(4.0, 0.0, 16.0).unwrap();
        assert_eq!(across.orientation().degrees(), 90.0);
        assert!((across.equivalent_radius().metres() - math::sqrt(8.0)).abs() < 1e-12);
        assert_eq!(format!("{across}"), "4.0 m × 2.0 m at 090°T");
    }

    #[test]
    fn a_correlated_covariance_tilts_the_ellipse() {
        // Equal variances, full correlation: a line at 45°.
        let ellipse = ErrorEllipse::from_covariance(1.0, 1.0, 1.0).unwrap();
        assert!((ellipse.semi_major().metres() - math::sqrt(2.0)).abs() < 1e-12);
        assert!(ellipse.semi_minor().metres() < 1e-9);
        assert!((ellipse.orientation().degrees() - 45.0).abs() < 1e-9);
        // Negative correlation tilts the other way, still in [0°, 180°).
        let other = ErrorEllipse::from_covariance(1.0, -0.5, 1.0).unwrap();
        assert!((other.orientation().degrees() - 135.0).abs() < 1e-9);
    }

    #[test]
    fn what_is_not_a_covariance_is_refused() {
        assert!(ErrorEllipse::from_covariance(-1.0, 0.0, 1.0).is_none());
        assert!(ErrorEllipse::from_covariance(1.0, 2.0, 1.0).is_none());
        assert!(ErrorEllipse::from_covariance(f64::NAN, 0.0, 1.0).is_none());
        assert!(ErrorEllipse::from_covariance(0.0, 0.0, 0.0).is_some());
    }

    #[test]
    fn an_empty_snapshot_knows_nothing_and_is_not_stale() {
        let empty = NavigationSnapshot::EMPTY;
        assert!(empty.position().is_none());
        assert!(empty.source().is_none());
        assert!(empty.ground_track().is_none());
        assert!(empty.heading().is_none());
        assert!(empty.horizontal_error().is_none());
        assert!(empty.integrity().is_none());
        assert_eq!(empty.age(), None);
        assert!(!empty.is_stale());
    }
}
