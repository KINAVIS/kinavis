//! Geographic position: latitude, longitude, position.
//!
//! Both coordinates enforce their range: a [`Latitude`] is in `[-90°, 90°]`; a
//! [`Longitude`] is in `[-180°, 180°)` and wraps, so crossing the antimeridian
//! cannot produce an invalid value.
//!
//! # Example
//!
//! ```rust
//! use kinavis_kernel::{Latitude, Longitude, KernelError, NorthSouth, Position};
//!
//! // Decimal degrees, or the degrees-and-minutes a chart is marked in.
//! let position = Position::new(
//!     Latitude::from_degrees_minutes(50, 45.3, NorthSouth::North)?,
//!     Longitude::from_degrees(-1.296_667)?,
//! );
//!
//! assert_eq!(format!("{position}"), "50°45.3'N 001°17.8'W");
//! assert_eq!(position.latitude().degrees(), 50.755);
//!
//! // Out of range is rejected; longitude wraps.
//! assert!(Latitude::from_degrees(91.0).is_err());
//! assert_eq!(Longitude::from_degrees(180.0)?.degrees(), -180.0);
//! # Ok::<(), KernelError>(())
//! ```

use core::fmt;
use core::str::FromStr;

use crate::angle::wrap180;
use crate::error::{ensure_finite, ensure_range, KernelError, Result};
use crate::math;
use crate::units::{Angle, Distance};

/// WGS 84 flattening.
pub const WGS84_FLATTENING: f64 = 1.0 / 298.257_223_563;
/// WGS 84 semi-major axis, m.
pub const WGS84_SEMI_MAJOR_AXIS_METRES: f64 = 6_378_137.0;
/// WGS 84 first eccentricity squared, `f·(2 − f)`.
pub const WGS84_ECCENTRICITY_SQUARED: f64 = WGS84_FLATTENING * (2.0 - WGS84_FLATTENING);
/// WGS 84 first eccentricity.
///
/// A literal because `sqrt` is not `const`; a test checks it equals the square
/// root.
const WGS84_ECCENTRICITY: f64 = 0.081_819_190_842_621_49;

/// Inverse hyperbolic tangent via the natural logarithm.
fn artanh(value: f64) -> f64 {
    0.5 * math::ln((1.0 + value) / (1.0 - value))
}

/// Latitude hemisphere.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum NorthSouth {
    /// North of the equator.
    North,
    /// South of the equator.
    South,
}

impl NorthSouth {
    /// `1.0` north, `-1.0` south.
    #[must_use]
    pub const fn sign(self) -> f64 {
        match self {
            Self::North => 1.0,
            Self::South => -1.0,
        }
    }

    /// Chart letter.
    #[must_use]
    pub const fn letter(self) -> char {
        match self {
            Self::North => 'N',
            Self::South => 'S',
        }
    }
}

/// Longitude hemisphere.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum EastWest {
    /// East of Greenwich.
    East,
    /// West of Greenwich.
    West,
}

impl EastWest {
    /// `1.0` east, `-1.0` west.
    #[must_use]
    pub const fn sign(self) -> f64 {
        match self {
            Self::East => 1.0,
            Self::West => -1.0,
        }
    }

    /// Chart letter.
    #[must_use]
    pub const fn letter(self) -> char {
        match self {
            Self::East => 'E',
            Self::West => 'W',
        }
    }
}

/// Latitude in `[-90°, 90°]`, north positive.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(try_from = "f64", into = "f64")
)]
pub struct Latitude(f64);

impl Latitude {
    /// The equator.
    pub const EQUATOR: Self = Self(0.0);
    /// The north pole.
    pub const NORTH_POLE: Self = Self(90.0);
    /// The south pole.
    pub const SOUTH_POLE: Self = Self(-90.0);

    /// Latitude from decimal degrees, north positive.
    ///
    /// # Errors
    ///
    /// [`crate::KernelError::NotFinite`] for `NaN` or infinity;
    /// [`crate::KernelError::OutOfRange`] outside `[-90.0, 90.0]`.
    pub fn from_degrees(value: f64) -> Result<Self> {
        ensure_range("latitude", value, -90.0, 90.0)?;
        Ok(Self(value))
    }

    /// Latitude from whole degrees and decimal minutes.
    ///
    /// # Errors
    ///
    /// As [`Latitude::from_degrees`], after combining.
    pub fn from_degrees_minutes(
        degrees: u16,
        minutes: f64,
        hemisphere: NorthSouth,
    ) -> Result<Self> {
        ensure_finite("latitude minutes", minutes)?;
        let magnitude = f64::from(degrees) + minutes / 60.0;
        Self::from_degrees(magnitude * hemisphere.sign())
    }

    /// Latitude from a value known to be finite, clamping rounding overshoot
    /// past ±90°.
    ///
    /// `NaN` survives the clamp and breaks the invariant; use
    /// [`Latitude::from_degrees`] otherwise.
    ///
    /// Internal to the crate family: hidden, not covered by the stability
    /// guarantee. See [hidden items](crate#hidden-items).
    #[doc(hidden)]
    #[must_use]
    pub fn from_degrees_clamped(value: f64) -> Self {
        Self(value.clamp(-90.0, 90.0))
    }

    /// Degrees, north positive.
    #[must_use]
    pub const fn degrees(self) -> f64 {
        self.0
    }

    /// Radians.
    #[must_use]
    pub fn radians(self) -> f64 {
        math::to_radians(self.0)
    }

    /// Hemisphere; the equator counts as north.
    #[must_use]
    pub fn hemisphere(self) -> NorthSouth {
        if self.0 < 0.0 {
            NorthSouth::South
        } else {
            NorthSouth::North
        }
    }

    /// Whole degrees, decimal minutes and hemisphere.
    #[must_use]
    pub fn to_degrees_minutes(self) -> (u16, f64, NorthSouth) {
        split_degrees_minutes(self.0, 90)
            .map_or((0, 0.0, self.hemisphere()), |(degrees, minutes)| {
                (degrees, minutes, self.hemisphere())
            })
    }

    /// Whether at a pole, where longitude is undefined.
    #[must_use]
    pub fn is_polar(self) -> bool {
        math::abs(math::abs(self.0) - 90.0) < 1e-9
    }

    /// Meridional parts: Mercator y-coordinate in minutes of arc, on the WGS 84
    /// ellipsoid.
    ///
    /// At 45° it gives 3013.6 (a sphere gives 3029.9).
    ///
    /// `kinavis::sailings::rhumb_line` uses a spherical model and will not
    /// match a Mercator sailing computed from these values exactly; use
    /// `kinavis::sailings::geodesic` when the ellipsoid matters.
    ///
    /// # Errors
    ///
    /// [`crate::KernelError::Indeterminate`] at a pole (infinite).
    pub fn meridional_parts(self) -> Result<f64> {
        if self.is_polar() {
            return Err(crate::KernelError::Indeterminate {
                quantity: "meridional parts at the pole",
            });
        }
        // Exact ellipsoidal correction, not the truncated series: the classic
        // 23.268932·sinφ − … coefficients are for Clarke 1866 and are 1.4′ off
        // at low latitudes on WGS 84.
        let eccentricity = WGS84_ECCENTRICITY;
        let sine = math::sin(self.radians());
        let correction = eccentricity * artanh(eccentricity * sine);
        Ok(self.isometric_minutes() - math::to_degrees(correction) * 60.0)
    }

    /// Latitude whose spherical isometric latitude is `minutes`: the
    /// Gudermannian, inverse of [`Latitude::isometric_minutes`].
    ///
    /// `NaN` breaks the invariant; callers pass values derived from valid
    /// latitudes.
    ///
    /// Internal to the crate family: hidden, not covered by the stability
    /// guarantee. See [hidden items](crate#hidden-items).
    #[doc(hidden)]
    #[must_use]
    pub fn from_isometric_minutes(minutes: f64) -> Self {
        let radians = math::to_radians(minutes / 60.0);
        let latitude = 2.0 * math::atan(math::exp(radians)) - core::f64::consts::FRAC_PI_2;
        Self::from_degrees_clamped(math::to_degrees(latitude))
    }

    /// Spherical isometric latitude, minutes of arc.
    ///
    /// The Mercator y-coordinate on a sphere, used by the rhumb-line sailings
    /// in `kinavis::sailings`. Infinite at the poles.
    #[must_use]
    pub fn isometric_minutes(self) -> f64 {
        math::to_degrees(math::ln(math::tan(
            core::f64::consts::FRAC_PI_4 + self.radians() / 2.0,
        ))) * 60.0
    }
}

impl fmt::Display for Latitude {
    /// Formats as `50°45.3'N`.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let precision = f.precision().unwrap_or(1);
        let (degrees, minutes, hemisphere) = self.to_degrees_minutes();
        write!(
            f,
            "{degrees:02}°{minutes:0>width$.precision$}'{}",
            hemisphere.letter(),
            width = if precision == 0 { 2 } else { precision + 3 }
        )
    }
}

impl FromStr for Latitude {
    type Err = crate::KernelError;

    /// Parses `50°45.3'N`, `N50 45 18`, `-33.9` etc.
    ///
    /// Hemisphere letter or sign, not both. See [`crate::position`] for
    /// accepted forms.
    ///
    /// # Errors
    ///
    /// [`crate::KernelError::Parse`] for unreadable input or a non-N/S
    /// hemisphere; [`crate::KernelError::OutOfRange`] beyond the poles.
    fn from_str(input: &str) -> Result<Self> {
        let parsed = crate::parse::sexagesimal("latitude", input)?;
        if let Some(letter) = parsed.hemisphere {
            if !matches!(letter, 'N' | 'S') {
                return Err(crate::parse::parse_error("latitude", input));
            }
        }
        Self::from_degrees(parsed.signed("S"))
    }
}

/// Longitude in `[-180°, 180°)`, east positive.
///
/// Out-of-range values wrap: adding a longitude difference across the
/// antimeridian is routine.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(try_from = "f64", into = "f64")
)]
pub struct Longitude(f64);

impl Longitude {
    /// Prime meridian.
    pub const GREENWICH: Self = Self(0.0);

    /// Longitude from decimal degrees, east positive, wrapped.
    ///
    /// # Errors
    ///
    /// [`crate::KernelError::NotFinite`] for `NaN` or infinity.
    pub fn from_degrees(value: f64) -> Result<Self> {
        ensure_finite("longitude", value)?;
        Ok(Self(wrap180(value)))
    }

    /// Longitude from whole degrees and decimal minutes.
    ///
    /// # Errors
    ///
    /// [`crate::KernelError::NotFinite`] for `NaN` or infinity.
    pub fn from_degrees_minutes(degrees: u16, minutes: f64, hemisphere: EastWest) -> Result<Self> {
        ensure_finite("longitude minutes", minutes)?;
        let magnitude = f64::from(degrees) + minutes / 60.0;
        Self::from_degrees(magnitude * hemisphere.sign())
    }

    /// Longitude from a value known to be finite.
    ///
    /// A non-finite argument breaks the invariant; use
    /// [`Longitude::from_degrees`] otherwise.
    ///
    /// Internal to the crate family: hidden, not covered by the stability
    /// guarantee. See [hidden items](crate#hidden-items).
    #[doc(hidden)]
    #[must_use]
    pub fn from_degrees_wrapped(value: f64) -> Self {
        Self(wrap180(value))
    }

    /// Degrees, east positive.
    #[must_use]
    pub const fn degrees(self) -> f64 {
        self.0
    }

    /// Radians.
    #[must_use]
    pub fn radians(self) -> f64 {
        math::to_radians(self.0)
    }

    /// Hemisphere; Greenwich counts as east.
    #[must_use]
    pub fn hemisphere(self) -> EastWest {
        if self.0 < 0.0 {
            EastWest::West
        } else {
            EastWest::East
        }
    }

    /// Whole degrees, decimal minutes and hemisphere.
    #[must_use]
    pub fn to_degrees_minutes(self) -> (u16, f64, EastWest) {
        split_degrees_minutes(self.0, 180)
            .map_or((0, 0.0, self.hemisphere()), |(degrees, minutes)| {
                (degrees, minutes, self.hemisphere())
            })
    }

    /// Shortest signed longitude difference to `other`, in `[-180°, 180°)`,
    /// east positive.
    #[must_use]
    pub fn difference_to(self, other: Self) -> Angle {
        Angle::from_degrees_unchecked(wrap180(other.0 - self.0))
    }
}

impl fmt::Display for Longitude {
    /// Formats as `001°17.8'W`.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let precision = f.precision().unwrap_or(1);
        let (degrees, minutes, hemisphere) = self.to_degrees_minutes();
        write!(
            f,
            "{degrees:03}°{minutes:0>width$.precision$}'{}",
            hemisphere.letter(),
            width = if precision == 0 { 2 } else { precision + 3 }
        )
    }
}

impl FromStr for Longitude {
    type Err = crate::KernelError;

    /// Parses `001°17.8'W`, `W001 17 48`, `151.2` etc.
    ///
    /// # Errors
    ///
    /// [`crate::KernelError::Parse`] for unreadable input or a non-E/W
    /// hemisphere.
    fn from_str(input: &str) -> Result<Self> {
        let parsed = crate::parse::sexagesimal("longitude", input)?;
        if let Some(letter) = parsed.hemisphere {
            if !matches!(letter, 'E' | 'W') {
                return Err(crate::parse::parse_error("longitude", input));
            }
        }
        Self::from_degrees(parsed.signed("W"))
    }
}

/// Position on the Earth's surface (WGS 84).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Position {
    latitude: Latitude,
    longitude: Longitude,
}

impl Position {
    /// 0°N 0°E.
    pub const ORIGIN: Self = Self {
        latitude: Latitude::EQUATOR,
        longitude: Longitude::GREENWICH,
    };

    /// Position from latitude and longitude.
    #[must_use]
    pub const fn new(latitude: Latitude, longitude: Longitude) -> Self {
        Self {
            latitude,
            longitude,
        }
    }

    /// Position from decimal degrees.
    ///
    /// # Errors
    ///
    /// As [`Latitude::from_degrees`] and [`Longitude::from_degrees`].
    pub fn from_degrees(latitude: f64, longitude: f64) -> Result<Self> {
        Ok(Self::new(
            Latitude::from_degrees(latitude)?,
            Longitude::from_degrees(longitude)?,
        ))
    }

    /// Latitude.
    #[must_use]
    pub const fn latitude(self) -> Latitude {
        self.latitude
    }

    /// Longitude.
    #[must_use]
    pub const fn longitude(self) -> Longitude {
        self.longitude
    }

    /// Latitude difference to `other`, north positive.
    #[must_use]
    pub fn latitude_difference(self, other: Self) -> Angle {
        Angle::from_degrees_unchecked(other.latitude.0 - self.latitude.0)
    }

    /// Longitude difference to `other`, east positive, short way.
    #[must_use]
    pub fn longitude_difference(self, other: Self) -> Angle {
        self.longitude.difference_to(other.longitude)
    }

    /// Departure: east-west distance between the meridians at the mean
    /// latitude.
    ///
    /// Plane-sailing approximation, for short distances.
    #[must_use]
    pub fn departure(self, other: Self) -> Distance {
        let mean_latitude = f64::midpoint(self.latitude.radians(), other.latitude.radians());
        Distance::from_nautical_miles_unchecked(
            self.longitude_difference(other).minutes() * math::cos(mean_latitude),
        )
    }

    /// Position as a geocentric direction; the form spherical geometry is
    /// computed in. See [`GeocentricUnit`] for the axes.
    #[must_use]
    pub fn to_geocentric_unit(self) -> GeocentricUnit {
        let (latitude, longitude) = (self.latitude.radians(), self.longitude.radians());
        let cos_latitude = math::cos(latitude);
        GeocentricUnit {
            x: cos_latitude * math::cos(longitude),
            y: cos_latitude * math::sin(longitude),
            z: math::sin(latitude),
        }
    }

    /// Position of a geocentric direction.
    ///
    /// Infallible: the zero vector cannot be a [`GeocentricUnit`].
    #[must_use]
    pub fn from_geocentric_unit(unit: GeocentricUnit) -> Self {
        let horizontal = math::hypot(unit.x, unit.y);
        Self::new(
            Latitude::from_degrees_clamped(math::to_degrees(math::atan2(unit.z, horizontal))),
            Longitude::from_degrees_wrapped(math::to_degrees(math::atan2(unit.y, unit.x))),
        )
    }
}

/// Unit vector from the Earth's centre.
///
/// x towards 0°N 0°E, y towards 0°N 90°E, z towards the north pole. Direction
/// only, no radius.
///
/// Components are private, finite and of unit length. As a bare `[f64; 3]` a
/// caller could pass the zero vector, a vector in other axes, or metres, and
/// the signature would not prevent it.
///
/// # Example
///
/// ```rust
/// use kinavis_kernel::{GeocentricUnit, Latitude, Longitude, Position};
///
/// let greenwich = Position::new(Latitude::from_degrees(0.0)?, Longitude::from_degrees(0.0)?);
/// let unit = greenwich.to_geocentric_unit();
/// assert!((unit.x() - 1.0).abs() < 1e-12);
///
/// // Round trips, and the length is normalised on the way in.
/// let doubled = GeocentricUnit::new(2.0, 0.0, 0.0)?;
/// assert_eq!(Position::from_geocentric_unit(doubled), greenwich);
///
/// // The zero vector points nowhere and is refused.
/// assert!(GeocentricUnit::new(0.0, 0.0, 0.0).is_err());
/// # Ok::<(), kinavis_kernel::KernelError>(())
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GeocentricUnit {
    x: f64,
    y: f64,
    z: f64,
}

impl GeocentricUnit {
    /// Normalises an Earth-centred vector; any positive multiple gives the same
    /// direction.
    ///
    /// # Errors
    ///
    /// - [`KernelError::NotFinite`] for a `NaN` or infinite component.
    /// - [`KernelError::Indeterminate`] for the zero vector or one too small to
    ///   normalise meaningfully.
    pub fn new(x: f64, y: f64, z: f64) -> Result<Self> {
        ensure_finite("geocentric x", x)?;
        ensure_finite("geocentric y", y)?;
        ensure_finite("geocentric z", z)?;
        Self::from_finite(x, y, z).ok_or(KernelError::Indeterminate {
            quantity: "a direction from a vector of zero length",
        })
    }

    /// As [`GeocentricUnit::new`], for components known to be finite.
    ///
    /// `None` only for the zero vector, so the caller can name the
    /// indeterminate quantity. A non-finite component breaks the invariant.
    ///
    /// Internal to the crate family: hidden, not covered by the stability
    /// guarantee. See [hidden items](crate#hidden-items).
    #[doc(hidden)]
    #[must_use]
    pub fn from_finite(x: f64, y: f64, z: f64) -> Option<Self> {
        let magnitude = math::hypot(math::hypot(x, y), z);
        if magnitude < f64::MIN_POSITIVE {
            return None;
        }
        Some(Self {
            x: x / magnitude,
            y: y / magnitude,
            z: z / magnitude,
        })
    }

    /// Component towards 0°N 0°E.
    #[must_use]
    pub const fn x(self) -> f64 {
        self.x
    }

    /// Component towards 0°N 90°E.
    #[must_use]
    pub const fn y(self) -> f64 {
        self.y
    }

    /// Component towards the north pole.
    #[must_use]
    pub const fn z(self) -> f64 {
        self.z
    }

    /// Components `[x, y, z]`.
    #[must_use]
    pub const fn components(self) -> [f64; 3] {
        [self.x, self.y, self.z]
    }

    /// Cosine of the angle between two directions.
    #[must_use]
    pub fn dot(self, other: Self) -> f64 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    /// Antipodal direction.
    #[must_use]
    pub fn antipode(self) -> Self {
        Self {
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }
}

impl FromStr for Position {
    type Err = crate::KernelError;

    /// Parses latitude then longitude.
    ///
    /// `50°45.3'N 001°17.8'W`, `N50 45.3 W001 17.8`, `50.755, -1.2967`.
    ///
    /// # Errors
    ///
    /// [`crate::KernelError::Parse`] if the halves cannot be separated or
    /// either is unreadable.
    fn from_str(input: &str) -> Result<Self> {
        let (latitude, longitude) = crate::parse::split_position(input)?;
        Ok(Self::new(latitude.parse()?, longitude.parse()?))
    }
}

impl fmt::Display for Position {
    /// Formats as `50°45.3'N 001°17.8'W`.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let precision = f.precision().unwrap_or(1);
        write!(
            f,
            "{:.precision$} {:.precision$}",
            self.latitude, self.longitude
        )
    }
}

/// Splits signed decimal degrees into whole degrees and decimal minutes.
///
/// `None` if the magnitude exceeds the bound; unreachable for a valid latitude
/// or longitude.
fn split_degrees_minutes(value: f64, maximum: u16) -> Option<(u16, f64)> {
    let magnitude = math::abs(value);
    let mut degrees = u16::try_from(math::to_usize(magnitude)).ok()?;
    let mut minutes = (magnitude - f64::from(degrees)) * 60.0;
    // Rounding guard: 10.99999' must not print as `10°60.0'`.
    if minutes >= 59.999_95 {
        minutes = 0.0;
        degrees = degrees.checked_add(1)?;
    }
    if degrees > maximum {
        return None;
    }
    Some((degrees, minutes))
}

#[cfg(feature = "serde")]
impl TryFrom<f64> for Latitude {
    type Error = KernelError;

    /// Validated on deserialisation.
    fn try_from(value: f64) -> Result<Self> {
        Self::from_degrees(value)
    }
}

#[cfg(feature = "serde")]
impl From<Latitude> for f64 {
    fn from(value: Latitude) -> Self {
        value.0
    }
}

#[cfg(feature = "serde")]
impl TryFrom<f64> for Longitude {
    type Error = KernelError;

    /// Validated on deserialisation.
    fn try_from(value: f64) -> Result<Self> {
        Self::from_degrees(value)
    }
}

#[cfg(feature = "serde")]
impl From<Longitude> for f64 {
    fn from(value: Longitude) -> Self {
        value.0
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp, clippy::indexing_slicing)]
mod tests {
    use super::*;
    use alloc::format;

    #[test]
    fn latitude_validates() {
        assert!(Latitude::from_degrees(90.0).is_ok());
        assert!(Latitude::from_degrees(-90.0).is_ok());
        assert!(Latitude::from_degrees(90.000_001).is_err());
        assert!(Latitude::from_degrees(f64::NAN).is_err());
        assert!(Latitude::from_degrees(f64::INFINITY).is_err());
    }

    #[test]
    fn longitude_wraps_instead_of_failing() {
        assert_eq!(Longitude::from_degrees(180.0).unwrap().degrees(), -180.0);
        assert_eq!(Longitude::from_degrees(-180.0).unwrap().degrees(), -180.0);
        assert_eq!(Longitude::from_degrees(190.0).unwrap().degrees(), -170.0);
        assert_eq!(Longitude::from_degrees(-190.0).unwrap().degrees(), 170.0);
        assert_eq!(Longitude::from_degrees(720.5).unwrap().degrees(), 0.5);
        assert!(Longitude::from_degrees(f64::NAN).is_err());
    }

    #[test]
    fn degrees_and_minutes_round_trip() {
        let latitude = Latitude::from_degrees_minutes(50, 45.3, NorthSouth::North).unwrap();
        assert!((latitude.degrees() - 50.755).abs() < 1e-12);
        let (degrees, minutes, hemisphere) = latitude.to_degrees_minutes();
        assert_eq!(degrees, 50);
        assert!((minutes - 45.3).abs() < 1e-9);
        assert_eq!(hemisphere, NorthSouth::North);

        let longitude = Longitude::from_degrees_minutes(1, 17.8, EastWest::West).unwrap();
        assert!((longitude.degrees() + 1.296_666_667).abs() < 1e-9);
    }

    #[test]
    fn display_matches_chart_convention() {
        let position = Position::from_degrees(50.755, -1.296_666_667).unwrap();
        assert_eq!(format!("{position}"), "50°45.3'N 001°17.8'W");

        let southern = Position::from_degrees(-33.9, 151.2).unwrap();
        assert_eq!(format!("{southern}"), "33°54.0'S 151°12.0'E");

        // Rounding must not produce 60.0 minutes.
        let boundary = Latitude::from_degrees(10.999_999_9).unwrap();
        assert_eq!(format!("{boundary}"), "11°00.0'N");
    }

    #[test]
    fn isometric_latitude_round_trips() {
        for degrees in [-85.0, -45.0, -0.5, 0.0, 0.5, 10.0, 45.0, 80.0, 89.0] {
            let latitude = Latitude::from_degrees(degrees).unwrap();
            let back = Latitude::from_isometric_minutes(latitude.isometric_minutes());
            assert!(
                (back.degrees() - degrees).abs() < 1e-9,
                "{degrees} came back as {}",
                back.degrees()
            );
        }
        // Poles are the limits; values clamp rather than overflow.
        assert!(Latitude::from_isometric_minutes(f64::INFINITY).is_polar());
        assert!(Latitude::from_isometric_minutes(f64::NEG_INFINITY).is_polar());
    }

    #[test]
    fn eccentricity_constant_is_the_square_root_it_claims_to_be() {
        let expected = WGS84_ECCENTRICITY_SQUARED.sqrt();
        assert!((WGS84_ECCENTRICITY - expected).abs() < 1e-15);
    }

    #[test]
    fn positions_read_back_from_what_they_print() {
        for (latitude, longitude) in [
            (50.755, -1.296_666_667),
            (-33.9, 151.2),
            (0.0, 0.0),
            (89.5, -179.5),
        ] {
            let position = Position::from_degrees(latitude, longitude).unwrap();
            let printed = alloc::format!("{position:.4}");
            let read: Position = printed.parse().unwrap();
            assert!(
                (read.latitude().degrees() - latitude).abs() < 1e-6,
                "{printed}"
            );
            assert!(
                read.longitude()
                    .difference_to(position.longitude())
                    .degrees()
                    .abs()
                    < 1e-6,
                "{printed}"
            );
        }
    }

    #[test]
    fn positions_parse_from_the_usual_forms() {
        let expected = Position::from_degrees(50.755, -1.296_666_667).unwrap();
        for input in [
            "50°45.3'N 001°17.8'W",
            "50 45.3 N 001 17.8 W",
            "N50°45.3' W001°17.8'",
            "50.755, -1.2966667",
            "50.755 -1.2966667",
        ] {
            let parsed: Position = input.parse().unwrap();
            assert!(
                (parsed.latitude().degrees() - expected.latitude().degrees()).abs() < 1e-6,
                "{input}"
            );
            assert!(
                (parsed.longitude().degrees() - expected.longitude().degrees()).abs() < 1e-6,
                "{input}"
            );
        }
    }

    #[test]
    fn the_wrong_hemisphere_letter_is_refused() {
        assert!("50°45.3'E".parse::<Latitude>().is_err());
        assert!("001°17.8'N".parse::<Longitude>().is_err());
        assert!("50°45.3'N".parse::<Longitude>().is_err());
        // A latitude beyond the pole is out of range.
        assert!("91 00.0 N".parse::<Latitude>().is_err());
        // A longitude past the antimeridian wraps.
        assert_eq!("190".parse::<Longitude>().unwrap().degrees(), -170.0);
    }

    #[test]
    fn unreadable_input_is_an_error_not_a_panic() {
        for input in ["", "   ", "north", "50°45.3'N", "a b", "50 45.3 60.0"] {
            assert!(input.parse::<Position>().is_err(), "{input}");
        }

        // Two bare numbers are latitude and longitude, not degrees and minutes
        // of one coordinate: a position needs both halves.
        let pair: Position = "50 45.3".parse().unwrap();
        assert_eq!(pair.latitude().degrees(), 50.0);
        assert_eq!(pair.longitude().degrees(), 45.3);
        for input in ["", "  ", "fifty", "50 60.0", "50 45 18 12"] {
            assert!(input.parse::<Latitude>().is_err(), "{input}");
            assert!(input.parse::<Longitude>().is_err(), "{input}");
        }
    }

    #[test]
    fn hemispheres() {
        assert_eq!(
            Latitude::from_degrees(0.0).unwrap().hemisphere(),
            NorthSouth::North
        );
        assert_eq!(
            Latitude::from_degrees(-0.1).unwrap().hemisphere(),
            NorthSouth::South
        );
        assert_eq!(
            Longitude::from_degrees(0.0).unwrap().hemisphere(),
            EastWest::East
        );
        assert_eq!(
            Longitude::from_degrees(-0.1).unwrap().hemisphere(),
            EastWest::West
        );
        assert!(Latitude::NORTH_POLE.is_polar());
        assert!(!Latitude::from_degrees(89.0).unwrap().is_polar());
    }

    #[test]
    fn longitude_difference_takes_the_short_way() {
        let west = Longitude::from_degrees(-179.0).unwrap();
        let east = Longitude::from_degrees(179.0).unwrap();
        assert!((west.difference_to(east).degrees() + 2.0).abs() < 1e-12);
        assert!((east.difference_to(west).degrees() - 2.0).abs() < 1e-12);
    }

    #[test]
    fn meridional_parts_match_the_tables() {
        // Meridional parts on WGS 84 from an independent evaluation of a·[ln
        // tan(π/4 + φ/2) − e·artanh(e sin φ)], minutes of arc.
        for (degrees, expected) in [
            (10.0, 599.073),
            (30.0, 1876.862),
            (45.0, 3013.648),
            (60.0, 4507.404),
            (75.0, 6948.063),
        ] {
            let latitude = Latitude::from_degrees(degrees).unwrap();
            let parts = latitude.meridional_parts().unwrap();
            assert!(
                (parts - expected).abs() < 0.001,
                "at {degrees}°: {parts} vs {expected}"
            );
        }

        let latitude = Latitude::from_degrees(45.0).unwrap();
        // The spherical value used by the rhumb sailings differs measurably.
        assert!((latitude.isometric_minutes() - 3029.9).abs() < 0.1);
        assert!(Latitude::EQUATOR.meridional_parts().unwrap().abs() < 1e-9);
        assert!(Latitude::NORTH_POLE.meridional_parts().is_err());

        // Symmetric about the equator.
        let south = Latitude::from_degrees(-45.0).unwrap();
        assert!((south.meridional_parts().unwrap() + 3013.648).abs() < 0.001);
    }

    #[test]
    fn geocentric_units_round_trip() {
        for (latitude, longitude) in [
            (0.0, 0.0),
            (45.0, 90.0),
            (-33.9, 151.2),
            (89.0, -179.0),
            (0.0, -180.0),
        ] {
            let position = Position::from_degrees(latitude, longitude).unwrap();
            let unit = position.to_geocentric_unit();
            // Invariant: unit length.
            assert!((unit.dot(unit) - 1.0).abs() < 1e-12);

            let back = Position::from_geocentric_unit(unit);
            assert!((back.latitude().degrees() - latitude).abs() < 1e-9);
            assert!(
                back.longitude()
                    .difference_to(position.longitude())
                    .degrees()
                    .abs()
                    < 1e-9
            );

            // The antipode is 180° away.
            let opposite = Position::from_geocentric_unit(unit.antipode());
            assert!((opposite.latitude().degrees() + latitude).abs() < 1e-9);
            assert!((unit.dot(unit.antipode()) + 1.0).abs() < 1e-12);
        }
    }

    #[test]
    fn a_geocentric_unit_normalises_and_refuses_what_points_nowhere() {
        // Any positive multiple gives the same direction.
        let one = GeocentricUnit::new(1.0, 0.0, 0.0).unwrap();
        let many = GeocentricUnit::new(1_000.0, 0.0, 0.0).unwrap();
        assert_eq!(one, many);
        assert!((one.x() - 1.0).abs() < 1e-12);
        assert_eq!(one.components(), [1.0, 0.0, 0.0]);

        // Length is normalised, not just checked.
        let diagonal = GeocentricUnit::new(3.0, 4.0, 0.0).unwrap();
        assert!((diagonal.x() - 0.6).abs() < 1e-12);
        assert!((diagonal.y() - 0.8).abs() < 1e-12);

        assert!(matches!(
            GeocentricUnit::new(0.0, 0.0, 0.0),
            Err(KernelError::Indeterminate { .. })
        ));
        assert!(matches!(
            GeocentricUnit::new(f64::NAN, 0.0, 0.0),
            Err(KernelError::NotFinite { .. })
        ));
        assert!(GeocentricUnit::new(f64::INFINITY, 0.0, 1.0).is_err());
    }

    #[test]
    fn departure_matches_plane_sailing() {
        // 1° of longitude at 60°: 30 NM departure.
        let from = Position::from_degrees(60.0, 0.0).unwrap();
        let to = Position::from_degrees(60.0, 1.0).unwrap();
        assert!((from.departure(to).nautical_miles() - 30.0).abs() < 0.01);
        assert!((from.latitude_difference(to).degrees()).abs() < 1e-12);
    }
}
