//! Angles, distances, speeds and rates of turn as types.
//!
//! A function taking `(f64, f64)` cannot distinguish knots from m/s; these
//! types can.
//!
//! Internally distances are nautical miles, speeds knots, rates of turn °/min;
//! construct and read in any unit.
//!
//! # Example
//!
//! ```rust
//! use kinavis_kernel::{Distance, KernelError, Speed};
//! use core::time::Duration;
//!
//! let leg = Distance::from_nautical_miles(12.0)?;
//! assert_eq!(format!("{:.0}", leg.metres()), "22224");
//! assert_eq!(format!("{:.0}", leg.cables()), "120");
//!
//! let speed = Speed::from_knots(8.0)?;
//! let elapsed = speed.time_to_cover(leg)?;
//! assert_eq!(elapsed.as_secs(), 5400); // an hour and a half
//!
//! assert_eq!(speed.distance_covered(elapsed).nautical_miles(), 12.0);
//! # Ok::<(), KernelError>(())
//! ```

use core::fmt;
use core::ops::{Add, Div, Mul, Neg, Sub};
use core::str::FromStr;
use core::time::Duration;

use crate::angle::wrap180;
use crate::error::{ensure_finite, KernelError, Result};
use crate::math;

/// Metres per international nautical mile.
pub const METRES_PER_NAUTICAL_MILE: f64 = 1852.0;
/// Metres per foot.
pub const METRES_PER_FOOT: f64 = 0.3048;
/// Metres per fathom (6 ft).
pub const METRES_PER_FATHOM: f64 = 6.0 * METRES_PER_FOOT;
/// Cables per nautical mile.
pub const CABLES_PER_NAUTICAL_MILE: f64 = 10.0;
/// Seconds per hour.
const SECONDS_PER_HOUR: f64 = 3600.0;
/// Seconds per minute.
const SECONDS_PER_MINUTE: f64 = 60.0;
/// Minutes per hour.
const MINUTES_PER_HOUR: f64 = 60.0;

/// Angular magnitude in degrees.
///
/// Not a compass direction and not wrapped: a difference, error or subtended
/// angle whose sign matters (gyro error, sextant angle, leeway).
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(try_from = "f64", into = "f64")
)]
pub struct Angle(f64);

impl Angle {
    /// Zero.
    pub const ZERO: Self = Self(0.0);

    /// From degrees.
    ///
    /// # Errors
    ///
    /// [`KernelError::NotFinite`] for `NaN` or infinity.
    pub fn from_degrees(value: f64) -> Result<Self> {
        ensure_finite("angle", value)?;
        Ok(Self(value))
    }

    /// From minutes of arc.
    ///
    /// # Errors
    ///
    /// [`KernelError::NotFinite`] for `NaN` or infinity.
    pub fn from_minutes(value: f64) -> Result<Self> {
        ensure_finite("angle", value)?;
        Ok(Self(value / 60.0))
    }

    /// From degrees, minutes and seconds.
    ///
    /// # Errors
    ///
    /// [`KernelError::NotFinite`] for `NaN` or infinity.
    pub fn from_degrees_minutes_seconds(degrees: f64, minutes: f64, seconds: f64) -> Result<Self> {
        ensure_finite("angle", degrees)?;
        ensure_finite("angle", minutes)?;
        ensure_finite("angle", seconds)?;
        Ok(Self(degrees + minutes / 60.0 + seconds / 3600.0))
    }

    /// From radians.
    ///
    /// # Errors
    ///
    /// [`KernelError::NotFinite`] for `NaN` or infinity.
    pub fn from_radians(value: f64) -> Result<Self> {
        ensure_finite("angle", value)?;
        Ok(Self(math::to_degrees(value)))
    }

    /// From a value known to be finite.
    ///
    /// A non-finite argument breaks the invariant; use [`Angle::from_degrees`]
    /// otherwise.
    ///
    /// Internal to the crate family: hidden, not covered by the stability
    /// guarantee. See [hidden items](crate#hidden-items).
    #[doc(hidden)]
    #[must_use]
    pub const fn from_degrees_unchecked(value: f64) -> Self {
        Self(value)
    }

    /// Degrees.
    #[must_use]
    pub const fn degrees(self) -> f64 {
        self.0
    }

    /// Minutes of arc.
    #[must_use]
    pub fn minutes(self) -> f64 {
        self.0 * 60.0
    }

    /// Radians.
    #[must_use]
    pub fn radians(self) -> f64 {
        math::to_radians(self.0)
    }

    /// Absolute value.
    #[must_use]
    pub fn abs(self) -> Self {
        Self(math::abs(self.0))
    }

    /// Folded into `[-180°, 180°)`.
    #[must_use]
    pub fn normalised(self) -> Self {
        Self(wrap180(self.0))
    }
}

impl fmt::Display for Angle {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let precision = f.precision().unwrap_or(1);
        write!(f, "{:.precision$}°", self.0)
    }
}

impl Neg for Angle {
    type Output = Self;
    fn neg(self) -> Self {
        Self(-self.0)
    }
}

impl Add for Angle {
    type Output = Self;
    fn add(self, other: Self) -> Self {
        Self(self.0 + other.0)
    }
}

impl Sub for Angle {
    type Output = Self;
    fn sub(self, other: Self) -> Self {
        Self(self.0 - other.0)
    }
}

impl FromStr for Angle {
    type Err = KernelError;

    /// Parses `1°30.5'`, `1 30 30`, `-2.7` etc.
    ///
    /// Sign only; hemisphere letters are rejected.
    ///
    /// # Errors
    ///
    /// [`KernelError::Parse`] for unreadable input, including a hemisphere
    /// letter.
    fn from_str(input: &str) -> Result<Self> {
        let parsed = crate::parse::sexagesimal("angle", input)?;
        if parsed.hemisphere.is_some() {
            return Err(crate::parse::parse_error("angle", input));
        }
        Self::from_degrees(parsed.signed(""))
    }
}

/// Distance, stored in nautical miles.
///
/// Signed: e.g. along-track distance is negative before the start of a leg.
/// Functions requiring a positive distance check it.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(try_from = "f64", into = "f64")
)]
pub struct Distance(f64);

impl Distance {
    /// Zero.
    pub const ZERO: Self = Self(0.0);

    /// From nautical miles.
    ///
    /// # Errors
    ///
    /// [`KernelError::NotFinite`] for `NaN` or infinity.
    pub fn from_nautical_miles(value: f64) -> Result<Self> {
        ensure_finite("distance", value)?;
        Ok(Self(value))
    }

    /// From cables (10 per NM).
    ///
    /// # Errors
    ///
    /// [`KernelError::NotFinite`] for `NaN` or infinity.
    pub fn from_cables(value: f64) -> Result<Self> {
        ensure_finite("distance", value)?;
        Ok(Self(value / CABLES_PER_NAUTICAL_MILE))
    }

    /// From metres.
    ///
    /// # Errors
    ///
    /// [`KernelError::NotFinite`] for `NaN` or infinity.
    pub fn from_metres(value: f64) -> Result<Self> {
        ensure_finite("distance", value)?;
        Ok(Self(value / METRES_PER_NAUTICAL_MILE))
    }

    /// From kilometres.
    ///
    /// # Errors
    ///
    /// [`KernelError::NotFinite`] for `NaN` or infinity.
    pub fn from_kilometres(value: f64) -> Result<Self> {
        ensure_finite("distance", value)?;
        Ok(Self(value * 1000.0 / METRES_PER_NAUTICAL_MILE))
    }

    /// From feet (e.g. charted heights of lights).
    ///
    /// # Errors
    ///
    /// [`KernelError::NotFinite`] for `NaN` or infinity.
    pub fn from_feet(value: f64) -> Result<Self> {
        ensure_finite("distance", value)?;
        Ok(Self(value * METRES_PER_FOOT / METRES_PER_NAUTICAL_MILE))
    }

    /// From fathoms.
    ///
    /// # Errors
    ///
    /// [`KernelError::NotFinite`] for `NaN` or infinity.
    pub fn from_fathoms(value: f64) -> Result<Self> {
        ensure_finite("distance", value)?;
        Ok(Self(value * METRES_PER_FATHOM / METRES_PER_NAUTICAL_MILE))
    }

    /// From minutes of arc of a great circle (1′ = 1 NM).
    ///
    /// # Errors
    ///
    /// [`KernelError::NotFinite`] for `NaN` or infinity.
    pub fn from_arc_minutes(value: f64) -> Result<Self> {
        Self::from_nautical_miles(value)
    }

    /// From a value known to be finite.
    ///
    /// A non-finite argument breaks the invariant; use
    /// [`Distance::from_nautical_miles`] otherwise.
    ///
    /// Internal to the crate family: hidden, not covered by the stability
    /// guarantee. See [hidden items](crate#hidden-items).
    #[doc(hidden)]
    #[must_use]
    pub const fn from_nautical_miles_unchecked(value: f64) -> Self {
        Self(value)
    }

    /// Nautical miles.
    #[must_use]
    pub const fn nautical_miles(self) -> f64 {
        self.0
    }

    /// Cables.
    #[must_use]
    pub fn cables(self) -> f64 {
        self.0 * CABLES_PER_NAUTICAL_MILE
    }

    /// Metres.
    #[must_use]
    pub fn metres(self) -> f64 {
        self.0 * METRES_PER_NAUTICAL_MILE
    }

    /// Kilometres.
    #[must_use]
    pub fn kilometres(self) -> f64 {
        self.0 * METRES_PER_NAUTICAL_MILE / 1000.0
    }

    /// Feet.
    #[must_use]
    pub fn feet(self) -> f64 {
        self.0 * METRES_PER_NAUTICAL_MILE / METRES_PER_FOOT
    }

    /// Absolute value.
    #[must_use]
    pub fn abs(self) -> Self {
        Self(math::abs(self.0))
    }

    /// Whether negative.
    #[must_use]
    pub fn is_negative(self) -> bool {
        self.0 < 0.0
    }

    /// Time to cover this distance at `speed`.
    ///
    /// # Errors
    ///
    /// [`KernelError::Indeterminate`] if the speed is zero or of opposite sign.
    pub fn time_at(self, speed: Speed) -> Result<Duration> {
        speed.time_to_cover(self)
    }
}

impl fmt::Display for Distance {
    /// Formats in nautical miles: `12.0 M`.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let precision = f.precision().unwrap_or(1);
        write!(f, "{:.precision$} M", self.0)
    }
}

impl Neg for Distance {
    type Output = Self;
    fn neg(self) -> Self {
        Self(-self.0)
    }
}

impl Add for Distance {
    type Output = Self;
    fn add(self, other: Self) -> Self {
        Self(self.0 + other.0)
    }
}

impl Sub for Distance {
    type Output = Self;
    fn sub(self, other: Self) -> Self {
        Self(self.0 - other.0)
    }
}

impl Mul<f64> for Distance {
    type Output = Self;
    fn mul(self, factor: f64) -> Self {
        Self(self.0 * factor)
    }
}

impl Div<f64> for Distance {
    type Output = Self;
    fn div(self, divisor: f64) -> Self {
        Self(self.0 / divisor)
    }
}

/// Speed, stored in knots. Negative is sternway.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(try_from = "f64", into = "f64")
)]
pub struct Speed(f64);

impl Speed {
    /// Zero.
    pub const ZERO: Self = Self(0.0);

    /// From knots.
    ///
    /// # Errors
    ///
    /// [`KernelError::NotFinite`] for `NaN` or infinity.
    pub fn from_knots(value: f64) -> Result<Self> {
        ensure_finite("speed", value)?;
        Ok(Self(value))
    }

    /// From m/s.
    ///
    /// # Errors
    ///
    /// [`KernelError::NotFinite`] for `NaN` or infinity.
    pub fn from_metres_per_second(value: f64) -> Result<Self> {
        ensure_finite("speed", value)?;
        Ok(Self(value * SECONDS_PER_HOUR / METRES_PER_NAUTICAL_MILE))
    }

    /// From km/h.
    ///
    /// # Errors
    ///
    /// [`KernelError::NotFinite`] for `NaN` or infinity.
    pub fn from_kilometres_per_hour(value: f64) -> Result<Self> {
        ensure_finite("speed", value)?;
        Ok(Self(value * 1000.0 / METRES_PER_NAUTICAL_MILE))
    }

    /// From a value known to be finite.
    ///
    /// A non-finite argument breaks the invariant; use [`Speed::from_knots`]
    /// otherwise.
    ///
    /// Internal to the crate family: hidden, not covered by the stability
    /// guarantee. See [hidden items](crate#hidden-items).
    #[doc(hidden)]
    #[must_use]
    pub const fn from_knots_unchecked(value: f64) -> Self {
        Self(value)
    }

    /// Knots.
    #[must_use]
    pub const fn knots(self) -> f64 {
        self.0
    }

    /// m/s.
    #[must_use]
    pub fn metres_per_second(self) -> f64 {
        self.0 * METRES_PER_NAUTICAL_MILE / SECONDS_PER_HOUR
    }

    /// km/h.
    #[must_use]
    pub fn kilometres_per_hour(self) -> f64 {
        self.0 * METRES_PER_NAUTICAL_MILE / 1000.0
    }

    /// Absolute value.
    #[must_use]
    pub fn abs(self) -> Self {
        Self(math::abs(self.0))
    }

    /// Whether negative (sternway).
    #[must_use]
    pub fn is_negative(self) -> bool {
        self.0 < 0.0
    }

    /// Distance covered in `duration`.
    #[must_use]
    pub fn distance_covered(self, elapsed: Duration) -> Distance {
        Distance(self.0 * elapsed.as_secs_f64() / SECONDS_PER_HOUR)
    }

    /// Time to cover `distance`.
    ///
    /// # Errors
    ///
    /// [`KernelError::Indeterminate`] if the speed is zero or of opposite sign
    /// to the distance.
    pub fn time_to_cover(self, distance: Distance) -> Result<Duration> {
        let hours = distance.0 / self.0;
        if !hours.is_finite() || hours < 0.0 {
            return Err(KernelError::Indeterminate {
                quantity: "time to cover the distance",
            });
        }
        Duration::try_from_secs_f64(hours * SECONDS_PER_HOUR).map_err(|_| {
            KernelError::Unrepresentable {
                what: "the time to cover the distance",
            }
        })
    }
}

impl fmt::Display for Speed {
    /// Formats in knots: `8.0 kn`.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let precision = f.precision().unwrap_or(1);
        write!(f, "{:.precision$} kn", self.0)
    }
}

impl Neg for Speed {
    type Output = Self;
    fn neg(self) -> Self {
        Self(-self.0)
    }
}

impl Add for Speed {
    type Output = Self;
    fn add(self, other: Self) -> Self {
        Self(self.0 + other.0)
    }
}

impl Sub for Speed {
    type Output = Self;
    fn sub(self, other: Self) -> Self {
        Self(self.0 - other.0)
    }
}

impl Mul<f64> for Speed {
    type Output = Self;
    fn mul(self, factor: f64) -> Self {
        Self(self.0 * factor)
    }
}

/// Rate of turn in °/min, as shown by the ROT indicator.
///
/// Positive to starboard. Keeps °/min distinct from rad/s.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(try_from = "f64", into = "f64")
)]
pub struct RateOfTurn(f64);

impl RateOfTurn {
    /// Zero.
    pub const ZERO: Self = Self(0.0);

    /// From °/min.
    ///
    /// # Errors
    ///
    /// [`KernelError::NotFinite`] for `NaN` or infinity.
    pub fn from_degrees_per_minute(value: f64) -> Result<Self> {
        ensure_finite("rate of turn", value)?;
        Ok(Self(value))
    }

    /// From rad/s.
    ///
    /// # Errors
    ///
    /// [`KernelError::NotFinite`] for `NaN` or infinity.
    pub fn from_radians_per_second(value: f64) -> Result<Self> {
        ensure_finite("rate of turn", value)?;
        Ok(Self(math::to_degrees(value) * SECONDS_PER_MINUTE))
    }

    /// °/min.
    #[must_use]
    pub const fn degrees_per_minute(self) -> f64 {
        self.0
    }

    /// rad/s.
    #[must_use]
    pub fn radians_per_second(self) -> f64 {
        math::to_radians(self.0) / SECONDS_PER_MINUTE
    }

    /// Absolute value.
    #[must_use]
    pub fn abs(self) -> Self {
        Self(math::abs(self.0))
    }

    /// Whether turning to port.
    #[must_use]
    pub fn is_to_port(self) -> bool {
        self.0 < 0.0
    }

    /// Turning radius at `speed`: speed / rate (rad/h).
    ///
    /// # Errors
    ///
    /// [`KernelError::Indeterminate`] for a zero rate.
    pub fn radius_at(self, speed: Speed) -> Result<Distance> {
        let radians_per_hour = math::to_radians(self.0) * MINUTES_PER_HOUR;
        let radius = speed.0 / radians_per_hour;
        if !radius.is_finite() {
            return Err(KernelError::Indeterminate {
                quantity: "the radius of a turn at no rate",
            });
        }
        Ok(Distance(math::abs(radius)))
    }

    /// Rate to starboard for a circle of `radius` at `speed`.
    ///
    /// # Errors
    ///
    /// [`KernelError::Indeterminate`] for a zero radius.
    pub fn around(radius: Distance, speed: Speed) -> Result<Self> {
        let radians_per_hour = speed.0 / radius.0;
        if !radians_per_hour.is_finite() {
            return Err(KernelError::Indeterminate {
                quantity: "the rate of a turn of no radius",
            });
        }
        Ok(Self(
            math::to_degrees(math::abs(radians_per_hour)) / MINUTES_PER_HOUR,
        ))
    }
}

impl fmt::Display for RateOfTurn {
    /// Formats in °/min: `12.0°/min`.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let precision = f.precision().unwrap_or(1);
        write!(f, "{:.precision$}°/min", self.0)
    }
}

impl Neg for RateOfTurn {
    type Output = Self;
    fn neg(self) -> Self {
        Self(-self.0)
    }
}

/// Duration in hours.
#[must_use]
pub fn hours(elapsed: Duration) -> f64 {
    elapsed.as_secs_f64() / SECONDS_PER_HOUR
}

/// Duration from hours.
///
/// # Errors
///
/// [`KernelError::Indeterminate`] for negative, non-finite or unrepresentable
/// values.
pub fn duration_from_hours(value: f64) -> Result<Duration> {
    if !value.is_finite() || value < 0.0 {
        return Err(KernelError::Indeterminate {
            quantity: "elapsed time",
        });
    }
    Duration::try_from_secs_f64(value * SECONDS_PER_HOUR).map_err(|_| {
        KernelError::Unrepresentable {
            what: "the elapsed time",
        }
    })
}

#[cfg(feature = "serde")]
impl TryFrom<f64> for Angle {
    type Error = KernelError;

    /// Validated on deserialisation.
    fn try_from(value: f64) -> Result<Self> {
        Self::from_degrees(value)
    }
}

#[cfg(feature = "serde")]
impl From<Angle> for f64 {
    fn from(value: Angle) -> Self {
        value.0
    }
}

#[cfg(feature = "serde")]
impl TryFrom<f64> for Distance {
    type Error = KernelError;

    /// Validated on deserialisation.
    fn try_from(value: f64) -> Result<Self> {
        Self::from_nautical_miles(value)
    }
}

#[cfg(feature = "serde")]
impl From<Distance> for f64 {
    fn from(value: Distance) -> Self {
        value.0
    }
}

#[cfg(feature = "serde")]
impl TryFrom<f64> for Speed {
    type Error = KernelError;

    /// Validated on deserialisation.
    fn try_from(value: f64) -> Result<Self> {
        Self::from_knots(value)
    }
}

#[cfg(feature = "serde")]
impl From<Speed> for f64 {
    fn from(value: Speed) -> Self {
        value.0
    }
}

#[cfg(feature = "serde")]
impl TryFrom<f64> for RateOfTurn {
    type Error = KernelError;

    /// Validated on deserialisation.
    fn try_from(value: f64) -> Result<Self> {
        Self::from_degrees_per_minute(value)
    }
}

#[cfg(feature = "serde")]
impl From<RateOfTurn> for f64 {
    fn from(value: RateOfTurn) -> Self {
        value.0
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp, clippy::indexing_slicing)]
mod tests {
    use super::*;
    use alloc::format;

    #[test]
    fn a_rate_of_turn_and_a_radius_are_two_views_of_one_circle() {
        // 10°/min at 10 kn: the circle takes 36 min and is 6 NM round, radius 6
        // / 2π.
        let rate = RateOfTurn::from_degrees_per_minute(10.0).unwrap();
        let speed = Speed::from_knots(10.0).unwrap();
        let radius = rate.radius_at(speed).unwrap();
        assert!((radius.nautical_miles() - 6.0 / core::f64::consts::TAU).abs() < 1e-12);
        let back = RateOfTurn::around(radius, speed).unwrap();
        assert!((back.degrees_per_minute() - 10.0).abs() < 1e-12);

        // A port turn has the same radius, with its direction.
        assert_eq!((-rate).radius_at(speed).unwrap(), radius);
        assert!((-rate).is_to_port());
        assert!(!rate.is_to_port());
        assert_eq!((-rate).abs(), rate);

        assert!((rate.radians_per_second() - 10.0_f64.to_radians() / 60.0).abs() < 1e-15);
        assert!(
            (RateOfTurn::from_radians_per_second(rate.radians_per_second())
                .unwrap()
                .degrees_per_minute()
                - 10.0)
                .abs()
                < 1e-12
        );
        assert_eq!(format!("{rate}"), "10.0°/min");
        assert_eq!(format!("{}", RateOfTurn::ZERO), "0.0°/min");
    }

    #[test]
    fn a_turn_at_no_rate_or_of_no_radius_has_no_circle() {
        let speed = Speed::from_knots(10.0).unwrap();
        assert!(RateOfTurn::ZERO.radius_at(speed).is_err());
        assert!(RateOfTurn::around(Distance::ZERO, speed).is_err());
        assert!(RateOfTurn::from_degrees_per_minute(f64::NAN).is_err());
        assert!(RateOfTurn::from_radians_per_second(f64::INFINITY).is_err());
        // Stopped: zero radius, not an error.
        assert_eq!(
            RateOfTurn::from_degrees_per_minute(5.0)
                .unwrap()
                .radius_at(Speed::ZERO)
                .unwrap(),
            Distance::ZERO
        );
    }

    #[test]
    fn distance_units_round_trip() {
        let distance = Distance::from_nautical_miles(1.0).unwrap();
        assert_eq!(distance.metres(), 1852.0);
        assert_eq!(distance.cables(), 10.0);
        assert!((distance.kilometres() - 1.852).abs() < 1e-12);
        assert!((distance.feet() - 6076.115).abs() < 1e-3);

        for constructor in [
            Distance::from_metres(1852.0),
            Distance::from_cables(10.0),
            Distance::from_kilometres(1.852),
            Distance::from_arc_minutes(1.0),
        ] {
            assert!((constructor.unwrap().nautical_miles() - 1.0).abs() < 1e-12);
        }

        assert!(
            (Distance::from_feet(6.0).unwrap().nautical_miles()
                - Distance::from_fathoms(1.0).unwrap().nautical_miles())
            .abs()
                < 1e-15
        );
    }

    #[test]
    fn speed_units_round_trip() {
        let speed = Speed::from_knots(1.0).unwrap();
        assert!((speed.metres_per_second() - 0.514_444_444).abs() < 1e-9);
        assert!((speed.kilometres_per_hour() - 1.852).abs() < 1e-12);
        assert!(
            (Speed::from_metres_per_second(0.514_444_444_444_444_4)
                .unwrap()
                .knots()
                - 1.0)
                .abs()
                < 1e-12
        );
    }

    #[test]
    fn distance_and_time_are_consistent() {
        let speed = Speed::from_knots(8.0).unwrap();
        let distance = Distance::from_nautical_miles(12.0).unwrap();
        let elapsed = speed.time_to_cover(distance).unwrap();
        assert_eq!(elapsed.as_secs(), 5400);
        assert!((speed.distance_covered(elapsed).nautical_miles() - 12.0).abs() < 1e-12);
        assert!((distance.time_at(speed).unwrap().as_secs_f64() - 5400.0).abs() < 1e-9);
    }

    #[test]
    fn impossible_times_are_errors_not_panics() {
        let distance = Distance::from_nautical_miles(10.0).unwrap();
        assert!(Speed::ZERO.time_to_cover(distance).is_err());
        assert!(Speed::from_knots(-5.0)
            .unwrap()
            .time_to_cover(distance)
            .is_err());
        assert!(Speed::from_knots(1e-300)
            .unwrap()
            .time_to_cover(Distance::from_nautical_miles(1e300).unwrap())
            .is_err());
    }

    #[test]
    fn non_finite_input_is_rejected() {
        for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(Distance::from_nautical_miles(value).is_err());
            assert!(Distance::from_metres(value).is_err());
            assert!(Speed::from_knots(value).is_err());
            assert!(Angle::from_degrees(value).is_err());
            assert!(Angle::from_minutes(value).is_err());
            assert!(Angle::from_radians(value).is_err());
        }
    }

    #[test]
    fn angle_conversions() {
        let angle = Angle::from_degrees_minutes_seconds(1.0, 30.0, 0.0).unwrap();
        assert_eq!(angle.degrees(), 1.5);
        assert_eq!(angle.minutes(), 90.0);
        assert!((Angle::from_minutes(90.0).unwrap().degrees() - 1.5).abs() < 1e-12);
        assert!(
            (Angle::from_radians(core::f64::consts::PI)
                .unwrap()
                .degrees()
                - 180.0)
                .abs()
                < 1e-12
        );
        assert_eq!((-angle).abs(), angle);
        assert_eq!(
            Angle::from_degrees(370.0).unwrap().normalised().degrees(),
            10.0
        );
    }

    #[test]
    fn angles_parse_from_how_they_are_written() {
        assert_eq!("1.5".parse::<Angle>().unwrap().degrees(), 1.5);
        assert!(("1°30'".parse::<Angle>().unwrap().degrees() - 1.5).abs() < 1e-12);
        assert!(("1 30 00".parse::<Angle>().unwrap().degrees() - 1.5).abs() < 1e-12);
        assert!(("-2.7".parse::<Angle>().unwrap().degrees() + 2.7).abs() < 1e-12);
        // Plain angles take no hemisphere.
        assert!("1°30'N".parse::<Angle>().is_err());
        assert!("".parse::<Angle>().is_err());
        assert!("1 60".parse::<Angle>().is_err());
    }

    #[test]
    fn arithmetic_behaves() {
        let a = Distance::from_nautical_miles(3.0).unwrap();
        let b = Distance::from_nautical_miles(4.0).unwrap();
        assert_eq!((a + b).nautical_miles(), 7.0);
        assert_eq!((b - a).nautical_miles(), 1.0);
        assert_eq!((a * 2.0).nautical_miles(), 6.0);
        assert_eq!((b / 2.0).nautical_miles(), 2.0);
        assert!((a - b).is_negative());
        assert_eq!((a - b).abs().nautical_miles(), 1.0);
    }

    #[test]
    fn display_is_readable() {
        assert_eq!(
            format!("{}", Distance::from_nautical_miles(12.0).unwrap()),
            "12.0 M"
        );
        assert_eq!(format!("{:.2}", Speed::from_knots(8.5).unwrap()), "8.50 kn");
        assert_eq!(format!("{}", Angle::from_degrees(-1.25).unwrap()), "-1.2°");
    }
}
