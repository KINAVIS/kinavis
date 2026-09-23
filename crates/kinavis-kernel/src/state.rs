//! Navigation state: the estimator's belief as one aggregate.
//!
//! An estimator fuses fixes, headings and speeds into one belief — position,
//! heading, speed through the water, current — with its uncertainty.
//! [`NavigationState`] is a DDD aggregate: private fields, constructed only
//! through invariant-checking constructors, exposed through the projection
//! [`NavigationState::project`].
//!
//! Internally a six-element state vector and its covariance
//! ([`StateComponent`]). The dimension is not public: consumers see an
//! [`ErrorEllipse`] and sigmas for heading and speed. The estimator uses hidden
//! accessors for the vector and covariance and rebuilds the state through a
//! hidden, re-validating constructor. Six components because each is directly
//! observed by a standard bridge sensor — GNSS position, gyro heading, log
//! speed — with current reconciling them.
//!
//! ```rust
//! use kinavis_kernel::gnss::{Dop, GnssFix};
//! use kinavis_kernel::state::NavigationState;
//! use kinavis_kernel::time::{Civil, Instant, Utc};
//! use kinavis_kernel::{Position, Speed, TrueCourse};
//!
//! let fix = GnssFix::builder(
//!     Instant::<Utc>::from_civil(Civil::date(2026, 9, 11))?,
//!     "50°45.3'N 001°20.0'W".parse::<Position>()?,
//! )
//! .course_over_ground(TrueCourse::new(272.5)?)
//! .speed_over_ground(Speed::from_knots(11.3)?)
//! .hdop(Dop::new(1.0)?)
//! .build();
//!
//! let state = NavigationState::initialised_from(&fix, None)?;
//! assert_eq!(state.heading().degrees(), 272.5);
//! assert!((state.speed_through_water().knots() - 11.3).abs() < 1e-9);
//! // Four metres one-sigma from the HDOP, in every direction.
//! assert!((state.horizontal_error().semi_major().metres() - 4.0).abs() < 1e-9);
//! # Ok::<(), kinavis_kernel::KernelError>(())
//! ```

use crate::angle::TrueCourse;
use crate::error::{ensure_range, KernelError, Result};
use crate::event::PositionSource;
use crate::geodesy::{Ellipsoid, GeodeticPoint, Height};
use crate::gnss::GnssFix;
use crate::local::{LocalFrame, Ned, Vector3};
use crate::math;
use crate::matrix::{Matrix, Vector};
use crate::observation::{ObservationStatus, Observed, Quality};
use crate::position::Position;
use crate::snapshot::{ErrorEllipse, GroundTrack, NavigationSnapshot};
use crate::time::{Instant, Utc};
use crate::units::{Angle, Distance, Speed, METRES_PER_NAUTICAL_MILE};

/// Number of state components.
///
/// Internal to the crate family: hidden, not covered by the stability
/// guarantee. See [hidden items](crate#hidden-items).
#[doc(hidden)]
pub const STATE_DIM: usize = 6;

/// State vector component, by name.
///
/// Observations reference components by name, never by index, so the vector
/// layout stays internal. `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum StateComponent {
    /// Northing from the anchor, m.
    North,
    /// Easting from the anchor, m.
    East,
    /// True heading, rad.
    Heading,
    /// Speed through the water along the heading, m/s.
    SpeedThroughWater,
    /// Current, north component, m/s.
    CurrentNorth,
    /// Current, east component, m/s.
    CurrentEast,
}

impl StateComponent {
    /// All components, in vector order.
    pub const ALL: [Self; STATE_DIM] = [
        Self::North,
        Self::East,
        Self::Heading,
        Self::SpeedThroughWater,
        Self::CurrentNorth,
        Self::CurrentEast,
    ];

    /// Vector index of the component.
    ///
    /// Internal to the crate family: hidden, not covered by the stability
    /// guarantee. See [hidden items](crate#hidden-items).
    #[doc(hidden)]
    #[must_use]
    pub const fn index(self) -> usize {
        match self {
            Self::North => 0,
            Self::East => 1,
            Self::Heading => 2,
            Self::SpeedThroughWater => 3,
            Self::CurrentNorth => 4,
            Self::CurrentEast => 5,
        }
    }
}

/// Initial 1σ priors where the fix provides none.
///
/// A fix usually gives horizontal accuracy, rarely course accuracy, never speed
/// accuracy, and nothing about current. The defaults are deliberately loose: an
/// estimator recovers from a loose prior within a few observations, never from
/// a tight wrong one.
///
/// Start from [`StatePriors::standard`] and adjust with `with_*`; each rejects
/// non-positive sigmas, so no component starts as exactly known.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StatePriors {
    position: Distance,
    heading: Angle,
    heading_unknown: Angle,
    speed: Speed,
    speed_unknown: Speed,
    current: Speed,
}

impl StatePriors {
    /// Merchant vessel with standard GNSS, gyro and log: 10 m; heading 3°, or
    /// 104° (uniform over the circle) if unknown; speed 1 kn, or 10 kn if
    /// unknown; current 1 kn.
    #[must_use]
    pub const fn standard() -> Self {
        Self {
            position: Distance::from_nautical_miles_unchecked(10.0 / METRES_PER_NAUTICAL_MILE),
            heading: Angle::from_degrees_unchecked(3.0),
            heading_unknown: Angle::from_degrees_unchecked(104.0),
            speed: Speed::from_knots_unchecked(1.0),
            speed_unknown: Speed::from_knots_unchecked(10.0),
            current: Speed::from_knots_unchecked(1.0),
        }
    }

    /// Sets the horizontal position sigma used when the fix gives no accuracy.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] unless positive.
    pub fn with_position(mut self, sigma: Distance) -> Result<Self> {
        ensure_positive("position prior", sigma.metres())?;
        self.position = sigma;
        Ok(self)
    }

    /// Sets the heading sigmas: `known` for a heading without sigma or taken
    /// from COG; `unknown` when neither is available.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] unless both positive.
    pub fn with_heading(mut self, known: Angle, unknown: Angle) -> Result<Self> {
        ensure_positive("heading prior", known.degrees())?;
        ensure_positive("unknown-heading prior", unknown.degrees())?;
        self.heading = known;
        self.heading_unknown = unknown;
        Ok(self)
    }

    /// Sets the speed-through-water sigmas: `known` when taken from SOG;
    /// `unknown` when the fix has no speed.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] unless both positive.
    pub fn with_speed(mut self, known: Speed, unknown: Speed) -> Result<Self> {
        ensure_positive("speed prior", known.knots())?;
        ensure_positive("unknown-speed prior", unknown.knots())?;
        self.speed = known;
        self.speed_unknown = unknown;
        Ok(self)
    }

    /// Sets the sigma of each current component (not directly observed).
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] unless positive.
    pub fn with_current(mut self, sigma: Speed) -> Result<Self> {
        ensure_positive("current prior", sigma.knots())?;
        self.current = sigma;
        Ok(self)
    }

    /// Horizontal position, when the fix gives no accuracy.
    #[must_use]
    pub const fn position(&self) -> Distance {
        self.position
    }

    /// Heading, when given without sigma or taken from COG.
    #[must_use]
    pub const fn heading(&self) -> Angle {
        self.heading
    }

    /// Heading, when neither heading nor course is available.
    #[must_use]
    pub const fn heading_unknown(&self) -> Angle {
        self.heading_unknown
    }

    /// Speed through the water, taken from SOG.
    #[must_use]
    pub const fn speed(&self) -> Speed {
        self.speed
    }

    /// Speed through the water, when the fix has no speed.
    #[must_use]
    pub const fn speed_unknown(&self) -> Speed {
        self.speed_unknown
    }

    /// Each current component.
    #[must_use]
    pub const fn current(&self) -> Speed {
        self.current
    }
}

/// A sigma must be finite and positive: zero means exactly known; negative is
/// invalid.
fn ensure_positive(parameter: &'static str, value: f64) -> Result<()> {
    ensure_range(parameter, value, f64::MIN_POSITIVE, f64::MAX)
}

/// Maximum speed through the water, m/s (100 kn).
const MAX_SPEED_METRES_PER_SECOND: f64 = 51.4;

/// Maximum current, m/s (20 kn).
const MAX_CURRENT_METRES_PER_SECOND: f64 = 10.3;

/// Estimator belief about the vessel at one instant.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NavigationState {
    valid_at: Instant<Utc>,
    /// Local frame of northing and easting.
    frame: LocalFrame,
    /// `[north, east, heading, speed through water, current north, current east]`.
    vector: Vector<STATE_DIM>,
    covariance: Matrix<STATE_DIM, STATE_DIM>,
}

impl NavigationState {
    /// State from a first fix and an optional heading.
    ///
    /// The fix position anchors the local frame, so northing and easting start
    /// at zero. Heading comes from the observation, else from COG, else it is
    /// unknown with the corresponding sigma. Speed through the water starts as
    /// SOG; current starts at zero with [`StatePriors::current`] uncertainty.
    ///
    /// # Errors
    ///
    /// As [`LocalFrame::at`] (not reachable from a fix);
    /// [`KernelError::OutOfRange`] for a speed above 100 kn.
    pub fn initialised_from(
        fix: &GnssFix,
        heading: Option<Observed<TrueCourse, Angle>>,
    ) -> Result<Self> {
        Self::initialised_with(fix, heading, &StatePriors::standard())
    }

    /// As [`NavigationState::initialised_from`], with custom priors.
    ///
    /// # Errors
    ///
    /// As [`NavigationState::initialised_from`].
    pub fn initialised_with(
        fix: &GnssFix,
        heading: Option<Observed<TrueCourse, Angle>>,
        priors: &StatePriors,
    ) -> Result<Self> {
        let anchor = GeodeticPoint::new(fix.position(), Height::above_ellipsoid(Distance::ZERO));
        let frame = LocalFrame::at(anchor, &Ellipsoid::WGS84)?;

        let (heading_radians, heading_sigma) = match (heading, fix.course_over_ground()) {
            (Some(observed), _) => (
                math::to_radians(observed.value().degrees()),
                observed
                    .quality()
                    .sigma()
                    .map_or(priors.heading, |sigma| *sigma),
            ),
            (None, Some(course)) => (math::to_radians(course.degrees()), priors.heading),
            (None, None) => (0.0, priors.heading_unknown),
        };
        let (speed, speed_sigma) = match fix.speed_over_ground() {
            Some(speed) => (speed, priors.speed),
            None => (Speed::ZERO, priors.speed_unknown),
        };
        let position_sigma = fix.horizontal_accuracy().unwrap_or(priors.position);

        let vector = Vector::from_column([
            0.0,
            0.0,
            heading_radians,
            speed.metres_per_second(),
            0.0,
            0.0,
        ]);
        let square = |value: f64| value * value;
        let covariance = Matrix::diagonal([
            square(position_sigma.metres()),
            square(position_sigma.metres()),
            square(heading_sigma.radians()),
            square(speed_sigma.metres_per_second()),
            square(priors.current.metres_per_second()),
            square(priors.current.metres_per_second()),
        ]);
        Self::from_parts(fix.taken_at(), frame, vector, covariance)
    }

    /// State from parts, validated; used by the estimator to rebuild a state
    /// after a step.
    ///
    /// # Errors
    ///
    /// [`KernelError::NotFinite`] for a non-finite element;
    /// [`KernelError::NotCovariance`] unless the covariance is symmetric
    /// positive semi-definite; [`KernelError::OutOfRange`] for speed or current
    /// beyond physical limits.
    ///
    /// Internal to the crate family: hidden, not covered by the stability
    /// guarantee. See [hidden items](crate#hidden-items).
    #[doc(hidden)]
    pub fn from_parts(
        valid_at: Instant<Utc>,
        frame: LocalFrame,
        vector: Vector<STATE_DIM>,
        covariance: Matrix<STATE_DIM, STATE_DIM>,
    ) -> Result<Self> {
        if !vector.is_finite() {
            return Err(KernelError::NotFinite {
                parameter: "navigation state",
                value: f64::NAN,
            });
        }
        if !covariance.is_finite() {
            return Err(KernelError::NotFinite {
                parameter: "navigation state covariance",
                value: f64::NAN,
            });
        }
        if !covariance.is_covariance() {
            return Err(KernelError::NotCovariance {
                context: "a navigation state's covariance",
            });
        }
        let element = |component: StateComponent| vector.element(component.index()).unwrap_or(0.0);
        let speed = element(StateComponent::SpeedThroughWater);
        if math::abs(speed) > MAX_SPEED_METRES_PER_SECOND {
            return Err(KernelError::OutOfRange {
                parameter: "speed through water",
                value: speed,
                min: -MAX_SPEED_METRES_PER_SECOND,
                max: MAX_SPEED_METRES_PER_SECOND,
            });
        }
        let current = math::hypot(
            element(StateComponent::CurrentNorth),
            element(StateComponent::CurrentEast),
        );
        if current > MAX_CURRENT_METRES_PER_SECOND {
            return Err(KernelError::OutOfRange {
                parameter: "current",
                value: current,
                min: 0.0,
                max: MAX_CURRENT_METRES_PER_SECOND,
            });
        }
        // Keep heading in `[0, 2π)` so equal headings compare equal and the
        // projection needs no wrapping.
        let heading = element(StateComponent::Heading);
        let mut vector = vector;
        vector.set(
            StateComponent::Heading.index(),
            0,
            math::to_radians(crate::angle::wrap360(math::to_degrees(heading))),
        );
        Ok(Self {
            valid_at,
            frame,
            vector,
            covariance: covariance.symmetrised(),
        })
    }

    /// Time of the state.
    #[must_use]
    pub const fn valid_at(&self) -> Instant<Utc> {
        self.valid_at
    }

    /// Local frame of the state.
    ///
    /// Internal to the crate family: hidden, not covered by the stability
    /// guarantee. See [hidden items](crate#hidden-items).
    #[doc(hidden)]
    #[must_use]
    pub const fn frame(&self) -> &LocalFrame {
        &self.frame
    }

    /// State vector.
    ///
    /// Internal to the crate family: hidden, not covered by the stability
    /// guarantee. See [hidden items](crate#hidden-items).
    #[doc(hidden)]
    #[must_use]
    pub const fn vector(&self) -> &Vector<STATE_DIM> {
        &self.vector
    }

    /// Covariance.
    ///
    /// Internal to the crate family: hidden, not covered by the stability
    /// guarantee. See [hidden items](crate#hidden-items).
    #[doc(hidden)]
    #[must_use]
    pub const fn covariance(&self) -> &Matrix<STATE_DIM, STATE_DIM> {
        &self.covariance
    }

    /// Vector component by name.
    fn component(&self, component: StateComponent) -> f64 {
        self.vector.element(component.index()).unwrap_or(0.0)
    }

    /// Variance of a component.
    fn variance(&self, component: StateComponent) -> f64 {
        self.covariance
            .get(component.index(), component.index())
            .unwrap_or(0.0)
            .max(0.0)
    }

    /// Estimated position.
    #[must_use]
    pub fn position(&self) -> Position {
        let (north, east) = (
            self.component(StateComponent::North),
            self.component(StateComponent::East),
        );
        // Zero displacement returns the anchor exactly, avoiding round-trip
        // noise.
        if north == 0.0 && east == 0.0 {
            return self.frame.origin().position();
        }
        let displacement: Vector3<Ned, Distance> = Vector3::new(
            Distance::from_metres(north).unwrap_or(Distance::ZERO),
            Distance::from_metres(east).unwrap_or(Distance::ZERO),
            Distance::ZERO,
        );
        // Fails only for planetary-scale displacements, excluded by the speed
        // and step bounds; the anchor is the fallback.
        self.frame
            .point_from_ned(displacement)
            .map_or(self.frame.origin().position(), |point| point.position())
    }

    /// Estimated heading.
    #[must_use]
    pub fn heading(&self) -> TrueCourse {
        TrueCourse::wrap(math::to_degrees(self.component(StateComponent::Heading)))
            .unwrap_or(TrueCourse::NORTH)
    }

    /// 1σ heading uncertainty.
    #[must_use]
    pub fn heading_sigma(&self) -> Angle {
        Angle::from_radians(math::sqrt(self.variance(StateComponent::Heading)))
            .unwrap_or(Angle::ZERO)
    }

    /// Estimated speed through the water along the heading.
    #[must_use]
    pub fn speed_through_water(&self) -> Speed {
        Speed::from_metres_per_second(self.component(StateComponent::SpeedThroughWater))
            .unwrap_or(Speed::ZERO)
    }

    /// 1σ speed uncertainty.
    #[must_use]
    pub fn speed_sigma(&self) -> Speed {
        Speed::from_metres_per_second(math::sqrt(self.variance(StateComponent::SpeedThroughWater)))
            .unwrap_or(Speed::ZERO)
    }

    /// Estimated current.
    #[must_use]
    pub fn current(&self) -> Vector3<Ned, Speed> {
        Vector3::new(
            Speed::from_metres_per_second(self.component(StateComponent::CurrentNorth))
                .unwrap_or(Speed::ZERO),
            Speed::from_metres_per_second(self.component(StateComponent::CurrentEast))
                .unwrap_or(Speed::ZERO),
            Speed::ZERO,
        )
    }

    /// Ground velocity: water velocity plus current.
    #[must_use]
    pub fn velocity_over_ground(&self) -> Vector3<Ned, Speed> {
        let heading = self.component(StateComponent::Heading);
        let speed = self.component(StateComponent::SpeedThroughWater);
        let through_water: Vector3<Ned, Speed> = Vector3::new(
            Speed::from_metres_per_second(speed * math::cos(heading)).unwrap_or(Speed::ZERO),
            Speed::from_metres_per_second(speed * math::sin(heading)).unwrap_or(Speed::ZERO),
            Speed::ZERO,
        );
        through_water + self.current()
    }

    /// Course and speed made good; `None` if the vessel is not moving.
    #[must_use]
    pub fn ground_track(&self) -> Option<GroundTrack> {
        let velocity = self.velocity_over_ground();
        Some(GroundTrack {
            course_over_ground: velocity.horizontal_direction()?,
            speed_over_ground: velocity.horizontal_magnitude(),
        })
    }

    /// 1σ position error ellipse.
    #[must_use]
    pub fn horizontal_error(&self) -> ErrorEllipse {
        let north = StateComponent::North.index();
        let east = StateComponent::East.index();
        ErrorEllipse::from_covariance(
            self.covariance.get(north, north).unwrap_or(0.0),
            self.covariance.get(north, east).unwrap_or(0.0),
            self.covariance.get(east, east).unwrap_or(0.0),
        )
        .unwrap_or_else(|| ErrorEllipse::circular(Distance::ZERO))
    }

    /// Read model for displays and alarms.
    ///
    /// Position status is `Valid` (the estimator's committed belief), sigma is
    /// the ellipse's equivalent radius. Age and staleness are left to the
    /// caller, who knows the current time.
    #[must_use]
    pub fn project(&self) -> NavigationSnapshot {
        let ellipse = self.horizontal_error();
        let position = Observed::new(
            self.position(),
            self.valid_at,
            Quality::new(ObservationStatus::Valid).with_sigma(ellipse.equivalent_radius()),
        );
        let mut snapshot = NavigationSnapshot::EMPTY
            .with_position(position, PositionSource::Estimated)
            .with_heading(self.heading(), Some(self.heading_sigma()))
            .with_horizontal_error(ellipse);
        if let Some(track) = self.ground_track() {
            snapshot = snapshot.with_ground_track(track);
        }
        snapshot
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use crate::gnss::Dop;

    fn fix() -> GnssFix {
        GnssFix::builder(
            Instant::from_unix_seconds(1_000),
            "50°45.3'N 001°20.0'W".parse().unwrap(),
        )
        .course_over_ground(TrueCourse::new(90.0).unwrap())
        .speed_over_ground(Speed::from_metres_per_second(5.0).unwrap())
        .hdop(Dop::new(2.0).unwrap())
        .build()
    }

    #[test]
    fn a_state_from_a_fix_starts_at_the_fix() {
        let state = NavigationState::initialised_from(&fix(), None).unwrap();
        assert_eq!(state.valid_at(), Instant::from_unix_seconds(1_000));
        assert_eq!(state.position(), fix().position());
        assert_eq!(state.heading().degrees(), 90.0);
        assert_eq!(state.speed_through_water().metres_per_second(), 5.0);
        assert_eq!(state.current().magnitude().metres_per_second(), 0.0);
        let velocity = state.velocity_over_ground();
        assert!(velocity.north().metres_per_second().abs() < 1e-12);
        assert!((velocity.east().metres_per_second() - 5.0).abs() < 1e-12);
        let track = state.ground_track().unwrap();
        assert!((track.course_over_ground.degrees() - 90.0).abs() < 1e-9);
        // HDOP 2.0 × 4 m: 8 m circle.
        assert!((state.horizontal_error().semi_major().metres() - 8.0).abs() < 1e-9);
        assert!((state.horizontal_error().semi_minor().metres() - 8.0).abs() < 1e-9);
        assert!((state.heading_sigma().degrees() - 3.0).abs() < 1e-9);
    }

    #[test]
    fn a_supplied_heading_beats_the_course_over_ground() {
        let heading = Observed::new(
            TrueCourse::new(80.0).unwrap(),
            Instant::from_unix_seconds(1_000),
            Quality::new(ObservationStatus::Valid).with_sigma(Angle::from_degrees(0.5).unwrap()),
        );
        let state = NavigationState::initialised_from(&fix(), Some(heading)).unwrap();
        assert_eq!(state.heading().degrees(), 80.0);
        assert!((state.heading_sigma().degrees() - 0.5).abs() < 1e-9);
    }

    #[test]
    fn a_bare_fix_leaves_heading_and_speed_unknown_and_says_so() {
        let bare = GnssFix::builder(Instant::from_unix_seconds(0), fix().position()).build();
        let state = NavigationState::initialised_from(&bare, None).unwrap();
        assert_eq!(state.speed_through_water().metres_per_second(), 0.0);
        assert!(state.ground_track().is_none());
        assert!(state.heading_sigma().degrees() > 100.0);
        assert!(state.speed_sigma().knots() > 5.0);
        // No HDOP: the 10 m prior.
        assert!((state.horizontal_error().semi_major().metres() - 10.0).abs() < 1e-9);
    }

    #[test]
    fn priors_of_your_own_are_used_and_a_sigma_of_nothing_is_refused() {
        let bare = GnssFix::builder(Instant::from_unix_seconds(0), fix().position()).build();
        let priors = StatePriors::standard()
            .with_position(Distance::from_metres(25.0).unwrap())
            .unwrap()
            .with_heading(
                Angle::from_degrees(1.0).unwrap(),
                Angle::from_degrees(90.0).unwrap(),
            )
            .unwrap()
            .with_speed(
                Speed::from_knots(0.5).unwrap(),
                Speed::from_knots(4.0).unwrap(),
            )
            .unwrap()
            .with_current(Speed::from_knots(2.0).unwrap())
            .unwrap();
        assert_eq!(priors.position().metres(), 25.0);
        assert_eq!(priors.heading_unknown().degrees(), 90.0);
        assert_eq!(priors.speed_unknown().knots(), 4.0);
        assert_eq!(priors.current().knots(), 2.0);
        let state = NavigationState::initialised_with(&bare, None, &priors).unwrap();
        assert!((state.horizontal_error().semi_major().metres() - 25.0).abs() < 1e-9);
        assert!((state.heading_sigma().degrees() - 90.0).abs() < 1e-9);
        assert!((state.speed_sigma().knots() - 4.0).abs() < 1e-9);

        // Zero sigma means exactly known; negative is invalid.
        let standard = StatePriors::standard();
        assert!(standard.with_position(Distance::ZERO).is_err());
        assert!(standard
            .with_position(Distance::from_metres(-1.0).unwrap())
            .is_err());
        assert!(standard
            .with_heading(Angle::ZERO, Angle::from_degrees(90.0).unwrap())
            .is_err());
        assert!(standard
            .with_heading(Angle::from_degrees(1.0).unwrap(), Angle::ZERO)
            .is_err());
        assert!(standard
            .with_speed(Speed::ZERO, Speed::from_knots(4.0).unwrap())
            .is_err());
        assert!(standard
            .with_speed(Speed::from_knots(0.5).unwrap(), Speed::ZERO)
            .is_err());
        assert!(standard.with_current(Speed::ZERO).is_err());
        assert!(standard
            .with_current(Speed::from_knots_unchecked(f64::NAN))
            .is_err());
        // Standard figures match the documentation.
        assert!((standard.position().metres() - 10.0).abs() < 1e-9);
        assert_eq!(standard.heading().degrees(), 3.0);
        assert_eq!(standard.heading_unknown().degrees(), 104.0);
        assert_eq!(standard.speed().knots(), 1.0);
        assert_eq!(standard.speed_unknown().knots(), 10.0);
        assert_eq!(standard.current().knots(), 1.0);
    }

    #[test]
    fn the_projection_is_a_snapshot_the_display_can_use() {
        let state = NavigationState::initialised_from(&fix(), None).unwrap();
        let snapshot = state.project();
        assert_eq!(snapshot.source(), Some(PositionSource::Estimated));
        assert_eq!(*snapshot.position().unwrap().value(), fix().position());
        assert_eq!(snapshot.heading().unwrap().degrees(), 90.0);
        assert!(snapshot.horizontal_error().is_some());
        assert!(snapshot.ground_track().is_some());
        assert_eq!(snapshot.age(), None);
    }

    #[test]
    fn the_invariants_are_checked_on_the_way_in() {
        let good = NavigationState::initialised_from(&fix(), None).unwrap();
        let frame = *good.frame();
        let at = good.valid_at();
        let mut vector = *good.vector();
        let covariance = *good.covariance();

        // Not a covariance.
        let mut bad = covariance;
        bad.set(0, 1, 1e9);
        bad.set(1, 0, 1e9);
        assert!(NavigationState::from_parts(at, frame, vector, bad).is_err());
        // Impossible speed.
        vector.set(StateComponent::SpeedThroughWater.index(), 0, 200.0);
        assert!(NavigationState::from_parts(at, frame, vector, covariance).is_err());
        // Heading 370° normalises to 10°.
        let mut wrapped = *good.vector();
        wrapped.set(StateComponent::Heading.index(), 0, math::to_radians(370.0));
        let state = NavigationState::from_parts(at, frame, wrapped, covariance).unwrap();
        assert!((state.heading().degrees() - 10.0).abs() < 1e-9);
        // NaN anywhere.
        let mut nan = *good.vector();
        nan.set(0, 0, f64::NAN);
        assert!(NavigationState::from_parts(at, frame, nan, covariance).is_err());
    }
}
