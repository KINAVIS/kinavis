//! Current, wind and leeway models implementing the kernel's environment ports.
//!
//! - [`Constant`]: one value everywhere and always (current from the last fix,
//!   observed wind); implements every port its value fits.
//! - [`Timetable`]: values at instants, linearly interpolated by components
//!   (e.g. a wind forecast at 0600, 1200, 1800).
//! - [`FixedLeeway`]: observed leeway, applied on the side the wind dictates.
//!
//! Tidal streams are in [`tides`](crate::tides).
//!
//! ```rust
//! use kinavis::conditions::{Constant, Timetable};
//! use kinavis::environment::{CurrentModel, Wind, WindModel};
//! use kinavis::{Civil, Instant, Position, Speed, TrueCourse, Utc};
//! use kinavis::navigation_solutions::Current;
//!
//! let here = Position::from_degrees(50.0, -1.0)?;
//! let at = |hour| Instant::<Utc>::from_civil(Civil { hour, ..Civil::date(2026, 9, 15) });
//!
//! // The current from the last fix, taken as holding.
//! let current = Constant(Current { set: TrueCourse::new(90.0)?, drift: Speed::from_knots(1.5)? });
//! assert_eq!(current.current_at(here, at(9)?)?.drift.knots(), 1.5);
//!
//! // The forecast: the wind backs and freshens through the morning.
//! let mut forecast = Timetable::<Wind, 8>::new();
//! forecast.push(at(6)?, Wind::new(TrueCourse::new(270.0)?, Speed::from_knots(10.0)?)?)?;
//! forecast.push(at(12)?, Wind::new(TrueCourse::new(180.0)?, Speed::from_knots(20.0)?)?)?;
//! // Halfway, as vectors: not the mean of the angles.
//! let at_nine = forecast.wind_at(here, at(9)?)?;
//! assert_eq!(format!("{:.0}", at_nine.from().degrees()), "207");
//! assert_eq!(format!("{:.1}", at_nine.speed()), "11.2 kn");
//! # Ok::<(), kinavis::NavigationError>(())
//! ```

use core::time::Duration;

use crate::angle::{wrap180, TrueCourse};
use crate::environment::{
    Current, CurrentModel, LeewayModel, MagneticField, MagneticModel, TideModel, VesselMotion,
    Wind, WindModel,
};
use crate::geodesy::GeodeticPoint;
use crate::inline::Inline;
use crate::math;
use crate::position::Position;
use crate::time::{Instant, Utc};
use crate::units::{Angle, Distance, Speed};
use kinavis_kernel::error::{KernelError, Result};

// ---------------------------------------------------------------------------
// One value, everywhere and always
// ---------------------------------------------------------------------------

/// Model returning the same value at every place and time.
///
/// The simplest port implementation: an estimated current, the last observed
/// wind, a charted field. Implements each port its value fits.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Constant<T>(pub T);

impl CurrentModel for Constant<Current> {
    fn current_at(&self, _: Position, _: Instant<Utc>) -> Result<Current> {
        Ok(self.0)
    }
}

impl WindModel for Constant<Wind> {
    fn wind_at(&self, _: Position, _: Instant<Utc>) -> Result<Wind> {
        Ok(self.0)
    }
}

impl TideModel for Constant<Distance> {
    fn height_of_tide(&self, _: Position, _: Instant<Utc>) -> Result<Distance> {
        Ok(self.0)
    }
}

impl MagneticModel for Constant<MagneticField> {
    fn field_at(&self, _: GeodeticPoint, _: Instant<Utc>) -> Result<MagneticField> {
        Ok(self.0)
    }
}

// ---------------------------------------------------------------------------
// Values at instants, read between them
// ---------------------------------------------------------------------------

/// Value storable in a [`Timetable`]: interpolated by components, so currents
/// setting `350°` and `010°` average to north, not south.
pub trait Interpolable: Copy {
    /// North and east components, in the value's unit.
    fn components(self) -> (f64, f64);
    /// Value from components.
    ///
    /// # Errors
    ///
    /// Any error of the value type; components from two valid values are always
    /// accepted.
    fn from_components(north: f64, east: f64) -> Result<Self>;

    /// Value `fraction` of the way from `self` to `other`.
    ///
    /// # Errors
    ///
    /// As [`from_components`](Self::from_components).
    fn interpolate(self, other: Self, fraction: f64) -> Result<Self> {
        let (n0, e0) = self.components();
        let (n1, e1) = other.components();
        Self::from_components(n0 + (n1 - n0) * fraction, e0 + (e1 - e0) * fraction)
    }
}

impl Interpolable for Current {
    fn components(self) -> (f64, f64) {
        let radians = self.set.radians();
        let knots = self.drift.knots();
        (knots * math::cos(radians), knots * math::sin(radians))
    }

    fn from_components(north: f64, east: f64) -> Result<Self> {
        Ok(Self {
            set: TrueCourse::from_degrees_wrapped(direction_of(north, east)),
            drift: Speed::from_knots(math::hypot(north, east))?,
        })
    }
}

impl Interpolable for Wind {
    fn components(self) -> (f64, f64) {
        // Wind blows towards the reciprocal of its "from" direction.
        let radians = self.towards().radians();
        let knots = self.speed().knots();
        (knots * math::cos(radians), knots * math::sin(radians))
    }

    fn from_components(north: f64, east: f64) -> Result<Self> {
        let towards = TrueCourse::from_degrees_wrapped(direction_of(north, east));
        Self::new(
            towards.reciprocal(),
            Speed::from_knots(math::hypot(north, east))?,
        )
    }
}

/// Direction of a vector in degrees; north for the zero vector (zero current).
fn direction_of(north: f64, east: f64) -> f64 {
    if north == 0.0 && east == 0.0 {
        0.0
    } else {
        math::to_degrees(math::atan2(east, north))
    }
}

/// Time-ordered values, interpolated between entries.
///
/// Forecasts, observation logs, hourly tidal streams. At most `N` entries
/// inline. Between entries values are interpolated by components; outside the
/// covered interval the result is [`KernelError::OutsideValidity`] (no
/// extrapolation).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Timetable<T: Interpolable, const N: usize> {
    // `Option` so unused slots need no `T` value.
    entries: Inline<Option<Entry<T>>, N>,
}

/// [`Timetable`] row.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Entry<T> {
    at: Instant<Utc>,
    value: T,
}

impl<T: Interpolable, const N: usize> Default for Timetable<T, N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T: Interpolable, const N: usize> Timetable<T, N> {
    /// Empty timetable.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            entries: Inline::new(None),
        }
    }

    /// Entries.
    fn entries(&self) -> impl Iterator<Item = &Entry<T>> {
        self.entries.as_slice().iter().flatten()
    }

    /// Appends a value, strictly later than the last.
    ///
    /// # Errors
    ///
    /// [`KernelError::TimeReversed`] if `at` is not after the last entry;
    /// [`KernelError::CapacityExceeded`] if full.
    pub fn push(&mut self, at: Instant<Utc>, value: T) -> Result<()> {
        if let Some(last) = self.entries().last() {
            if at <= last.at {
                return Err(KernelError::TimeReversed {
                    by: last.at.checked_duration_since(at).unwrap_or_default(),
                });
            }
        }
        self.entries
            .push(Some(Entry { at, value }))
            .map_err(|full| KernelError::CapacityExceeded {
                context: "a timetable",
                needed: full.capacity + 1,
                capacity: full.capacity,
            })
    }

    /// Number of entries.
    #[must_use]
    pub const fn len(&self) -> usize {
        self.entries.len()
    }

    /// Whether empty.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// Interval covered, first to last entry.
    #[must_use]
    pub fn span(&self) -> Option<(Instant<Utc>, Instant<Utc>)> {
        Some((self.entries().next()?.at, self.entries().last()?.at))
    }

    /// Value at `when`, interpolated between the surrounding entries.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutsideValidity`] before the first entry, after the last,
    /// or when empty.
    pub fn at(&self, when: Instant<Utc>) -> Result<T> {
        let outside = || KernelError::OutsideValidity { data: "timetable" };
        let (first, last) = self.span().ok_or_else(outside)?;
        if when < first || when > last {
            return Err(outside());
        }
        // Last entry at or before `when`, and the next one.
        let before = self
            .entries()
            .take_while(|entry| entry.at <= when)
            .last()
            .ok_or_else(outside)?;
        // At an entry, return it exactly, not via components.
        let Some(after) = self.entries().find(|entry| entry.at > when) else {
            return Ok(before.value);
        };
        if before.at == when {
            return Ok(before.value);
        }
        let elapsed = when.checked_duration_since(before.at).unwrap_or_default();
        let gap = after
            .at
            .checked_duration_since(before.at)
            .unwrap_or_default();
        let fraction = fraction_of(elapsed, gap);
        before.value.interpolate(after.value, fraction)
    }
}

/// `part / whole` in `[0, 1]`; zero for an empty whole.
fn fraction_of(part: Duration, whole: Duration) -> f64 {
    if whole.is_zero() {
        0.0
    } else {
        (part.as_secs_f64() / whole.as_secs_f64()).clamp(0.0, 1.0)
    }
}

impl<const N: usize> CurrentModel for Timetable<Current, N> {
    fn current_at(&self, _: Position, when: Instant<Utc>) -> Result<Current> {
        self.at(when)
    }
}

impl<const N: usize> WindModel for Timetable<Wind, N> {
    fn wind_at(&self, _: Position, when: Instant<Utc>) -> Result<Wind> {
        self.at(when)
    }
}

// ---------------------------------------------------------------------------
// Leeway
// ---------------------------------------------------------------------------

/// Observed leeway angle, applied downwind.
///
/// Magnitude as given; sign from the wind side: wind from port sets to
/// starboard (positive). Wind dead ahead or astern, or calm, gives zero.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FixedLeeway(pub Angle);

impl LeewayModel for FixedLeeway {
    fn leeway(&self, motion: VesselMotion, wind: Wind) -> Result<Angle> {
        if wind.speed() == Speed::ZERO {
            return Ok(Angle::ZERO);
        }
        let relative = wrap180(wind.from().degrees() - motion.heading().degrees());
        let magnitude = math::abs(self.0.degrees());
        Angle::from_degrees(if relative < 0.0 {
            magnitude
        } else if relative > 0.0 {
            -magnitude
        } else {
            0.0
        })
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use crate::time::Civil;

    fn here() -> Position {
        Position::from_degrees(50.0, -1.0).unwrap()
    }

    fn at(hour: u8) -> Instant<Utc> {
        Instant::from_civil(Civil {
            hour,
            ..Civil::date(2026, 9, 15)
        })
        .unwrap()
    }

    fn current(set: f64, knots: f64) -> Current {
        Current {
            set: TrueCourse::new(set).unwrap(),
            drift: Speed::from_knots(knots).unwrap(),
        }
    }

    fn wind(from: f64, knots: f64) -> Wind {
        Wind::new(
            TrueCourse::new(from).unwrap(),
            Speed::from_knots(knots).unwrap(),
        )
        .unwrap()
    }

    #[test]
    fn a_constant_answers_every_port_its_value_fits() {
        assert_eq!(
            Constant(current(90.0, 1.5))
                .current_at(here(), at(9))
                .unwrap(),
            current(90.0, 1.5)
        );
        assert_eq!(
            Constant(wind(270.0, 10.0)).wind_at(here(), at(9)).unwrap(),
            wind(270.0, 10.0)
        );
        let field = MagneticField::from_ned_nanotesla(20_000.0, -1_000.0, 40_000.0).unwrap();
        let point = GeodeticPoint::new(
            here(),
            crate::geodesy::Height::above_mean_sea_level(crate::units::Distance::ZERO),
        );
        assert_eq!(Constant(field).field_at(point, at(9)).unwrap(), field);
    }

    #[test]
    fn a_timetable_interpolates_through_the_components_not_the_angle() {
        let mut table = Timetable::<Current, 4>::new();
        table.push(at(6), current(350.0, 2.0)).unwrap();
        table.push(at(8), current(10.0, 2.0)).unwrap();
        // Midway between 350° and 010° is north, with a slightly reduced drift
        // — not 180°, not 2 kn.
        let midway = table.at(at(7)).unwrap();
        assert!(midway.set.angular_distance(TrueCourse::NORTH) < 1e-9);
        assert!((midway.drift.knots() - 2.0 * math::cos(math::to_radians(10.0))).abs() < 1e-9);
        // At the entries, the entries.
        assert_eq!(table.at(at(6)).unwrap(), current(350.0, 2.0));
        assert_eq!(table.at(at(8)).unwrap(), current(10.0, 2.0));
        assert_eq!(table.span(), Some((at(6), at(8))));
        assert_eq!(table.len(), 2);
    }

    #[test]
    fn a_timetable_does_not_extend_itself() {
        let mut table = Timetable::<Wind, 4>::new();
        assert!(matches!(
            table.at(at(7)),
            Err(KernelError::OutsideValidity { data: "timetable" })
        ));
        table.push(at(6), wind(270.0, 10.0)).unwrap();
        table.push(at(12), wind(180.0, 20.0)).unwrap();
        assert!(table.at(at(5)).is_err());
        assert!(table.at(at(13)).is_err());
        assert!(table.wind_at(here(), at(12)).is_ok());
    }

    #[test]
    fn entries_must_come_in_order_and_fit() {
        let mut table = Timetable::<Wind, 2>::new();
        table.push(at(6), wind(270.0, 10.0)).unwrap();
        assert!(matches!(
            table.push(at(6), wind(270.0, 10.0)),
            Err(KernelError::TimeReversed { .. })
        ));
        assert!(matches!(
            table.push(at(5), wind(270.0, 10.0)),
            Err(KernelError::TimeReversed { .. })
        ));
        table.push(at(7), wind(270.0, 10.0)).unwrap();
        assert!(matches!(
            table.push(at(8), wind(270.0, 10.0)),
            Err(KernelError::CapacityExceeded { .. })
        ));
    }

    #[test]
    fn a_wind_is_interpolated_as_the_vector_it_is() {
        let mut table = Timetable::<Wind, 4>::new();
        table.push(at(6), wind(270.0, 10.0)).unwrap();
        table.push(at(12), wind(180.0, 20.0)).unwrap();
        let at_nine = table.at(at(9)).unwrap();
        // From 270° at 10 kn → towards east (0, 10); from 180° at 20 kn →
        // towards north (20, 0). Midway (10, 5): towards 026.6°, i.e. from
        // 206.6° — not the 225° of an angle mean.
        assert!((at_nine.from().degrees() - 206.565).abs() < 1e-2);
        assert!((at_nine.speed().knots() - math::hypot(10.0, 5.0)).abs() < 1e-9);
    }

    #[test]
    fn fixed_leeway_falls_away_from_the_wind() {
        let model = FixedLeeway(Angle::from_degrees(5.0).unwrap());
        let motion = VesselMotion::new(TrueCourse::NORTH, Speed::from_knots(6.0).unwrap());
        assert_eq!(
            model.leeway(motion, wind(270.0, 15.0)).unwrap().degrees(),
            5.0
        );
        assert_eq!(
            model.leeway(motion, wind(90.0, 15.0)).unwrap().degrees(),
            -5.0
        );
        assert_eq!(
            model.leeway(motion, wind(0.0, 15.0)).unwrap().degrees(),
            0.0
        );
        assert_eq!(model.leeway(motion, Wind::CALM).unwrap().degrees(), 0.0);
        // Negative angle equals positive.
        let negative = FixedLeeway(Angle::from_degrees(-5.0).unwrap());
        assert_eq!(
            negative
                .leeway(motion, wind(270.0, 15.0))
                .unwrap()
                .degrees(),
            5.0
        );
    }
}
