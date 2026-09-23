//! Angles tagged with their reference frame.
//!
//! With bare `f64`, `calculate_true_bearing(bearing, variation)` and
//! `calculate_true_bearing(variation, bearing)` both compile and both yield a
//! plausible course. Here a [`CompassCourse`] cannot be passed where a
//! [`MagneticCourse`] is expected, nor a [`Variation`] where a course is
//! expected.
//!
//! The types also enforce range: a [`Direction`] is always finite and in `[0°,
//! 360°)`, so functions consuming one cannot fail on range and return no
//! `Result`.
//!
//! # Example
//!
//! ```rust
//! use kinavis_kernel::{CompassCourse, Deviation, MagneticCourse, Variation};
//!
//! let cc = CompassCourse::new(3.0)?;
//! let variation = Variation::new(-2.7)?;
//!
//! // Out-of-range and non-finite inputs are rejected at construction.
//! assert!(CompassCourse::new(400.0).is_err());
//! assert!(Variation::new(f64::NAN).is_err());
//!
//! // 360° is accepted and normalised to 0°.
//! assert_eq!(CompassCourse::new(360.0)?.degrees(), 0.0);
//!
//! // Arbitrary values are wrapped explicitly.
//! assert_eq!(MagneticCourse::wrap(-10.0)?.degrees(), 350.0);
//! # Ok::<(), kinavis_kernel::KernelError>(())
//! ```

use core::fmt;
use core::marker::PhantomData;

use crate::error::{ensure_finite, ensure_range, KernelError, Result};
use crate::math;

/// Maximum magnitude of a magnetic variation, degrees.
pub const MAX_VARIATION_DEG: f64 = 180.0;

/// Maximum magnitude of a compass deviation, degrees.
pub const MAX_DEVIATION_DEG: f64 = 180.0;

/// Normalises a finite angle into `[0.0, 360.0)`.
///
/// Correct for every finite input, including values below `-360°`, unlike
/// `(angle + 360.0) % 360.0`.
#[must_use]
pub fn wrap360(degrees: f64) -> f64 {
    let remainder = degrees % 360.0;
    if remainder < 0.0 {
        let shifted = remainder + 360.0;
        // A remainder such as -1e-16 rounds to exactly 360.0 on addition and
        // would leave the half-open interval; map it to 0.
        if shifted >= 360.0 {
            0.0
        } else {
            shifted
        }
    } else {
        // Adding zero turns a `-0.0` remainder into `+0.0`.
        remainder + 0.0
    }
}

/// Normalises a finite angle into `[-180.0, 180.0)`.
#[must_use]
pub fn wrap180(degrees: f64) -> f64 {
    let wrapped = wrap360(degrees);
    if wrapped >= 180.0 {
        wrapped - 360.0
    } else {
        wrapped
    }
}

mod sealed {
    pub trait Sealed {}
}

/// Reference frame of a [`Direction`].
///
/// Sealed: only the frames below exist.
pub trait Frame: sealed::Sealed + Copy + Clone + fmt::Debug + 'static {
    /// Frame name for errors and display.
    const NAME: &'static str;
    /// Single-letter suffix, as in `045.0°M`.
    const SUFFIX: char;
}

/// True (geographic) north.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct True;

/// Magnetic north.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Magnetic;

/// Compass north of the ship's magnetic compass.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Compass;

/// Gyrocompass north.
///
/// A gyrocompass has no deviation but has a gyro error, hence a separate frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Gyro;

impl sealed::Sealed for True {}
impl sealed::Sealed for Magnetic {}
impl sealed::Sealed for Compass {}
impl sealed::Sealed for Gyro {}

impl Frame for True {
    const NAME: &'static str = "true";
    const SUFFIX: char = 'T';
}

impl Frame for Magnetic {
    const NAME: &'static str = "magnetic";
    const SUFFIX: char = 'M';
}

impl Frame for Compass {
    const NAME: &'static str = "compass";
    const SUFFIX: char = 'C';
}

impl Frame for Gyro {
    const NAME: &'static str = "gyro";
    const SUFFIX: char = 'G';
}

/// Direction in `[0°, 360°)`, tagged with its reference frame.
///
/// Courses and bearings share this type: within a frame they are the same
/// quantity. The type prevents mixing *frames*.
#[derive(Clone, Copy, PartialEq, PartialOrd, Default)]
pub struct Direction<F: Frame> {
    degrees: f64,
    frame: PhantomData<F>,
}

/// Course or bearing, true.
pub type TrueCourse = Direction<True>;
/// Bearing, true. Alias of [`TrueCourse`].
pub type TrueBearing = Direction<True>;
/// Course or bearing, magnetic.
pub type MagneticCourse = Direction<Magnetic>;
/// Bearing, magnetic. Alias of [`MagneticCourse`].
pub type MagneticBearing = Direction<Magnetic>;
/// Course or bearing, compass.
pub type CompassCourse = Direction<Compass>;
/// Bearing, compass. Alias of [`CompassCourse`].
pub type CompassBearing = Direction<Compass>;
/// Course, gyro.
pub type GyroCourse = Direction<Gyro>;
/// Bearing, gyro. Alias of [`GyroCourse`].
pub type GyroBearing = Direction<Gyro>;

impl<F: Frame> Direction<F> {
    /// Due north, `000°`.
    pub const NORTH: Self = Self::from_wrapped(0.0);
    /// Due east, `090°`.
    pub const EAST: Self = Self::from_wrapped(90.0);
    /// Due south, `180°`.
    pub const SOUTH: Self = Self::from_wrapped(180.0);
    /// Due west, `270°`.
    pub const WEST: Self = Self::from_wrapped(270.0);

    /// Direction from a value already in `[0.0, 360.0)`.
    const fn from_wrapped(degrees: f64) -> Self {
        Self {
            degrees,
            frame: PhantomData,
        }
    }

    /// Wraps a value known to be finite.
    ///
    /// For algorithms whose inputs are validated newtypes, so the result cannot
    /// be `NaN`. A non-finite argument breaks the type invariant; use
    /// [`Direction::wrap`] otherwise.
    ///
    /// Internal to the crate family: hidden, not covered by the stability
    /// guarantee. See [hidden items](crate#hidden-items).
    #[doc(hidden)]
    #[must_use]
    pub fn from_degrees_wrapped(degrees: f64) -> Self {
        Self::from_wrapped(wrap360(degrees))
    }

    /// Direction from a value in `[0.0, 360.0]`; `360.0` maps to `0.0`.
    ///
    /// Out-of-range values are rejected, not wrapped: `400°` is more likely a
    /// typo or unit error than `040°`. Use [`Direction::wrap`] to wrap.
    ///
    /// # Errors
    ///
    /// [`KernelError::NotFinite`] for `NaN` or infinity;
    /// [`KernelError::OutOfRange`] outside `[0.0, 360.0]`.
    pub fn new(degrees: f64) -> Result<Self> {
        ensure_range("course", degrees, 0.0, 360.0)?;
        Ok(Self::from_wrapped(wrap360(degrees)))
    }

    /// Direction from any finite value, normalised into `[0.0, 360.0)`.
    ///
    /// # Errors
    ///
    /// [`KernelError::NotFinite`] for `NaN` or infinity.
    pub fn wrap(degrees: f64) -> Result<Self> {
        ensure_finite("course", degrees)?;
        Ok(Self::from_wrapped(wrap360(degrees)))
    }

    /// Degrees, in `[0.0, 360.0)`.
    #[must_use]
    pub const fn degrees(self) -> f64 {
        self.degrees
    }

    /// Radians, in `[0.0, 2π)`.
    #[must_use]
    pub fn radians(self) -> f64 {
        math::to_radians(self.degrees)
    }

    /// Reciprocal direction.
    #[must_use]
    pub fn reciprocal(self) -> Self {
        Self::from_wrapped(wrap360(self.degrees + 180.0))
    }

    /// North and east components of a vector of `magnitude` in this direction,
    /// in the magnitude's unit.
    ///
    /// The single place where velocity triangles (current triangle, relative
    /// motion, drift) resolve their sides.
    #[must_use]
    pub fn components(self, magnitude: f64) -> (f64, f64) {
        let radians = self.radians();
        (
            magnitude * math::cos(radians),
            magnitude * math::sin(radians),
        )
    }

    /// Rotates by `delta` degrees, wrapping.
    ///
    /// # Errors
    ///
    /// [`KernelError::NotFinite`] if `delta` is not finite.
    pub fn offset(self, delta: f64) -> Result<Self> {
        ensure_finite("delta", delta)?;
        Ok(Self::from_wrapped(wrap360(self.degrees + delta)))
    }

    /// Signed shortest angle from `self` to `other`, in `[-180.0, 180.0)`;
    /// positive clockwise.
    #[must_use]
    pub fn signed_difference(self, other: Self) -> f64 {
        wrap180(other.degrees - self.degrees)
    }

    /// Unsigned shortest angle between two directions, in `[0.0, 180.0]`.
    #[must_use]
    pub fn angular_distance(self, other: Self) -> f64 {
        math::abs(self.signed_difference(other))
    }

    /// Changes the frame tag without changing the value.
    ///
    /// Hidden on purpose: a frame change must go through a conversion in
    /// `kinavis::navigation_solutions` that applies variation or deviation.
    /// This is the primitive those conversions use.
    ///
    /// Internal to the crate family: hidden, not covered by the stability
    /// guarantee. See [hidden items](crate#hidden-items).
    #[doc(hidden)]
    #[must_use]
    pub const fn relabel<G: Frame>(self) -> Direction<G> {
        Direction::from_wrapped(self.degrees)
    }
}

impl<F: Frame> fmt::Display for Direction<F> {
    /// Formats as `045.0°T`.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let precision = f.precision().unwrap_or(1);
        write!(
            f,
            "{:0>width$.precision$}°{}",
            self.degrees,
            F::SUFFIX,
            width = if precision == 0 { 3 } else { precision + 4 },
        )
    }
}

impl<F: Frame> fmt::Debug for Direction<F> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}({}°)", F::NAME, self.degrees)
    }
}

/// Magnetic variation: angle from true north to magnetic north.
///
/// East positive. True = magnetic + variation.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(try_from = "f64", into = "f64")
)]
pub struct Variation(f64);

impl Variation {
    /// No variation.
    pub const ZERO: Self = Self(0.0);

    /// Variation from a value in `[-180.0, 180.0]` degrees.
    ///
    /// # Errors
    ///
    /// [`KernelError::NotFinite`] for `NaN` or infinity;
    /// [`KernelError::OutOfRange`] outside `[-180.0, 180.0]`.
    pub fn new(degrees: f64) -> Result<Self> {
        ensure_range("variation", degrees, -MAX_VARIATION_DEG, MAX_VARIATION_DEG)?;
        Ok(Self(degrees))
    }

    /// Degrees, east positive.
    #[must_use]
    pub const fn degrees(self) -> f64 {
        self.0
    }
}

impl fmt::Display for Variation {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let precision = f.precision().unwrap_or(1);
        let hemisphere = if self.0 < 0.0 { 'W' } else { 'E' };
        write!(f, "{:.precision$}°{hemisphere}", math::abs(self.0))
    }
}

/// Compass deviation: angle from magnetic north to compass north.
///
/// East positive. Magnetic = compass + deviation.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(try_from = "f64", into = "f64")
)]
pub struct Deviation(f64);

impl Deviation {
    /// No deviation.
    pub const ZERO: Self = Self(0.0);

    /// Deviation from a value in `[-180.0, 180.0]` degrees.
    ///
    /// # Errors
    ///
    /// [`KernelError::NotFinite`] for `NaN` or infinity;
    /// [`KernelError::OutOfRange`] outside `[-180.0, 180.0]`.
    pub fn new(degrees: f64) -> Result<Self> {
        ensure_range("deviation", degrees, -MAX_DEVIATION_DEG, MAX_DEVIATION_DEG)?;
        Ok(Self(degrees))
    }

    /// Degrees, east positive.
    #[must_use]
    pub const fn degrees(self) -> f64 {
        self.0
    }
}

impl fmt::Display for Deviation {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let precision = f.precision().unwrap_or(1);
        let hemisphere = if self.0 < 0.0 { 'W' } else { 'E' };
        write!(f, "{:.precision$}°{hemisphere}", math::abs(self.0))
    }
}

/// Cardinal or intercardinal compass point.
///
/// A type rather than a string, so deviation-table points cannot be misspelled:
/// a typo is a compile error, or [`KernelError::UnknownCardinalDirection`] when
/// parsed, never a silent failed lookup.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
///
/// # Example
///
/// ```rust
/// use kinavis_kernel::{CardinalPoint, CompassCourse};
///
/// assert_eq!(CardinalPoint::SW.degrees(), 225.0);
/// assert_eq!("sw".parse::<CardinalPoint>()?, CardinalPoint::SW);
/// assert_eq!(CompassCourse::from(CardinalPoint::SW).degrees(), 225.0);
/// # Ok::<(), kinavis_kernel::KernelError>(())
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
#[non_exhaustive]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum CardinalPoint {
    /// North, `000°`.
    #[default]
    N,
    /// North-east, `045°`.
    NE,
    /// East, `090°`.
    E,
    /// South-east, `135°`.
    SE,
    /// South, `180°`.
    S,
    /// South-west, `225°`.
    SW,
    /// West, `270°`.
    W,
    /// North-west, `315°`.
    NW,
}

impl CardinalPoint {
    /// All eight points, N to NW.
    pub const ALL: [Self; 8] = [
        Self::N,
        Self::NE,
        Self::E,
        Self::SE,
        Self::S,
        Self::SW,
        Self::W,
        Self::NW,
    ];

    /// Course of the point in whole degrees, `0..360`.
    ///
    /// The points fall on multiples of 45°, and deviation tables are indexed by
    /// whole degrees.
    #[must_use]
    pub const fn whole_degrees(self) -> i32 {
        match self {
            Self::N => 0,
            Self::NE => 45,
            Self::E => 90,
            Self::SE => 135,
            Self::S => 180,
            Self::SW => 225,
            Self::W => 270,
            Self::NW => 315,
        }
    }

    /// Course of the point in degrees, `[0.0, 360.0)`.
    #[must_use]
    pub fn degrees(self) -> f64 {
        f64::from(self.whole_degrees())
    }

    /// Abbreviation, `"N"` to `"NW"`.
    #[must_use]
    pub const fn abbreviation(self) -> &'static str {
        match self {
            Self::N => "N",
            Self::NE => "NE",
            Self::E => "E",
            Self::SE => "SE",
            Self::S => "S",
            Self::SW => "SW",
            Self::W => "W",
            Self::NW => "NW",
        }
    }
}

impl fmt::Display for CardinalPoint {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.abbreviation())
    }
}

impl core::str::FromStr for CardinalPoint {
    type Err = KernelError;

    /// Parses an abbreviation, case-insensitive, trimming whitespace.
    ///
    /// # Errors
    ///
    /// [`KernelError::UnknownCardinalDirection`] for anything else.
    fn from_str(text: &str) -> Result<Self> {
        let trimmed = text.trim();
        Self::ALL
            .into_iter()
            .find(|point| point.abbreviation().eq_ignore_ascii_case(trimmed))
            .ok_or_else(|| KernelError::UnknownCardinalDirection {
                direction: trimmed.into(),
            })
    }
}

impl<F: Frame> From<CardinalPoint> for Direction<F> {
    /// The point as a direction in any frame; the point itself has no frame.
    fn from(point: CardinalPoint) -> Self {
        Self::from_wrapped(point.degrees())
    }
}

/// Side relative to the bow.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Side {
    /// Dead ahead, within tolerance.
    Ahead,
    /// Starboard: `000°` to `180°` relative.
    Starboard,
    /// Dead astern, within tolerance.
    Astern,
    /// Port: `180°` to `360°` relative.
    Port,
}

/// Bearing clockwise from the ship's head, in `[0°, 360°)`.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(try_from = "f64", into = "f64")
)]
pub struct RelativeBearing(f64);

impl RelativeBearing {
    /// Dead ahead.
    pub const AHEAD: Self = Self(0.0);
    /// Starboard beam.
    pub const ABEAM_STARBOARD: Self = Self(90.0);
    /// Dead astern.
    pub const ASTERN: Self = Self(180.0);
    /// Port beam.
    pub const ABEAM_PORT: Self = Self(270.0);

    /// Relative bearing from a value in `[0.0, 360.0]`; `360.0` maps to `0.0`.
    ///
    /// # Errors
    ///
    /// [`KernelError::NotFinite`] for `NaN` or infinity;
    /// [`KernelError::OutOfRange`] outside `[0.0, 360.0]`.
    pub fn new(degrees: f64) -> Result<Self> {
        ensure_range("relative bearing", degrees, 0.0, 360.0)?;
        Ok(Self(wrap360(degrees)))
    }

    /// Relative bearing from any finite value, normalised into `[0.0, 360.0)`.
    ///
    /// # Errors
    ///
    /// [`KernelError::NotFinite`] for `NaN` or infinity.
    pub fn wrap(degrees: f64) -> Result<Self> {
        ensure_finite("relative bearing", degrees)?;
        Ok(Self(wrap360(degrees)))
    }

    /// Wraps a value known to be finite.
    ///
    /// A non-finite argument breaks the type invariant; use
    /// [`RelativeBearing::wrap`] otherwise.
    ///
    /// Internal to the crate family: hidden, not covered by the stability
    /// guarantee. See [hidden items](crate#hidden-items).
    #[doc(hidden)]
    #[must_use]
    pub fn from_degrees_wrapped(degrees: f64) -> Self {
        Self(wrap360(degrees))
    }

    /// Degrees, in `[0.0, 360.0)`.
    #[must_use]
    pub const fn degrees(self) -> f64 {
        self.0
    }

    /// Signed angle in `[-180.0, 180.0)`; positive starboard, negative port.
    #[must_use]
    pub fn signed_degrees(self) -> f64 {
        wrap180(self.0)
    }

    /// Side relative to the bow.
    #[must_use]
    pub fn side(self) -> Side {
        const TOLERANCE: f64 = 1e-9;
        let signed = self.signed_degrees();
        if math::abs(signed) < TOLERANCE {
            Side::Ahead
        } else if math::abs(math::abs(signed) - 180.0) < TOLERANCE {
            Side::Astern
        } else if signed > 0.0 {
            Side::Starboard
        } else {
            Side::Port
        }
    }
}

impl fmt::Display for RelativeBearing {
    /// Formats as `030.0° green` / `045.0° red`.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let precision = f.precision().unwrap_or(1);
        let magnitude = math::abs(self.signed_degrees());
        match self.side() {
            Side::Ahead => write!(f, "dead ahead"),
            Side::Astern => write!(f, "dead astern"),
            Side::Starboard => write!(f, "{magnitude:.precision$}° green"),
            Side::Port => write!(f, "{magnitude:.precision$}° red"),
        }
    }
}

#[cfg(feature = "serde")]
impl TryFrom<f64> for Variation {
    type Error = KernelError;

    /// Validated on deserialisation.
    fn try_from(value: f64) -> Result<Self> {
        Self::new(value)
    }
}

#[cfg(feature = "serde")]
impl From<Variation> for f64 {
    fn from(value: Variation) -> Self {
        value.0
    }
}

#[cfg(feature = "serde")]
impl TryFrom<f64> for Deviation {
    type Error = KernelError;

    /// Validated on deserialisation.
    fn try_from(value: f64) -> Result<Self> {
        Self::new(value)
    }
}

#[cfg(feature = "serde")]
impl From<Deviation> for f64 {
    fn from(value: Deviation) -> Self {
        value.0
    }
}

#[cfg(feature = "serde")]
impl TryFrom<f64> for RelativeBearing {
    type Error = KernelError;

    /// Validated on deserialisation.
    fn try_from(value: f64) -> Result<Self> {
        Self::new(value)
    }
}

#[cfg(feature = "serde")]
impl From<RelativeBearing> for f64 {
    fn from(value: RelativeBearing) -> Self {
        value.0
    }
}

#[cfg(feature = "serde")]
impl<F: Frame> serde::Serialize for Direction<F> {
    /// Serialised as plain degrees; the frame is in the type.
    fn serialize<S: serde::Serializer>(
        &self,
        serializer: S,
    ) -> core::result::Result<S::Ok, S::Error> {
        serializer.serialize_f64(self.degrees)
    }
}

#[cfg(feature = "serde")]
impl<'de, F: Frame> serde::Deserialize<'de> for Direction<F> {
    /// Deserialised through [`Direction::new`]; values outside `[0°, 360°]` are
    /// rejected.
    fn deserialize<D: serde::Deserializer<'de>>(
        deserializer: D,
    ) -> core::result::Result<Self, D::Error> {
        let degrees = f64::deserialize(deserializer)?;
        Self::new(degrees).map_err(serde::de::Error::custom)
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp, clippy::indexing_slicing)]
mod tests {
    use super::*;
    use alloc::format;

    #[test]
    fn wrap360_is_correct_below_minus_360() {
        // `(x + 360.0) % 360.0` gives -40.0 here.
        assert_eq!(wrap360(-400.0), 320.0);
        assert_eq!(wrap360(-720.0), 0.0);
        assert_eq!(wrap360(-0.0), 0.0);
        assert_eq!(wrap360(0.0), 0.0);
        assert_eq!(wrap360(360.0), 0.0);
        assert_eq!(wrap360(725.0), 5.0);
        assert!(wrap360(-1e15).is_finite());
    }

    #[test]
    fn wrap360_keeps_tiny_negatives_off_the_far_end() {
        // These round to exactly 360.0 under naive addition.
        for value in [-1e-16, -1e-18, -f64::MIN_POSITIVE, -1e-14] {
            let wrapped = wrap360(value);
            assert!(
                (0.0..360.0).contains(&wrapped),
                "{value} wrapped to {wrapped}"
            );
        }
        assert_eq!(wrap360(-1e-16), 0.0);
        // A representable large negative still lands just below 360.
        assert!(wrap360(-1e-10) < 360.0);
        assert!(wrap360(-1e-10) > 359.999);
    }

    #[test]
    fn wrap360_never_leaves_the_interval() {
        let mut value = -2000.0;
        while value < 2000.0 {
            let wrapped = wrap360(value);
            assert!((0.0..360.0).contains(&wrapped), "{value} -> {wrapped}");
            value += 0.37;
        }
    }

    #[test]
    fn wrap180_is_symmetric() {
        assert_eq!(wrap180(0.0), 0.0);
        assert_eq!(wrap180(90.0), 90.0);
        assert_eq!(wrap180(180.0), -180.0);
        assert_eq!(wrap180(190.0), -170.0);
        assert_eq!(wrap180(-190.0), 170.0);
    }

    #[test]
    fn direction_rejects_bad_input() {
        assert!(TrueCourse::new(f64::NAN).is_err());
        assert!(TrueCourse::new(f64::INFINITY).is_err());
        assert!(TrueCourse::new(-0.1).is_err());
        assert!(TrueCourse::new(400.0).is_err());
        assert!(TrueCourse::wrap(f64::NAN).is_err());
    }

    #[test]
    fn direction_normalises() {
        assert_eq!(TrueCourse::new(360.0).unwrap().degrees(), 0.0);
        assert_eq!(TrueCourse::wrap(-10.0).unwrap().degrees(), 350.0);
        assert_eq!(TrueCourse::wrap(730.0).unwrap().degrees(), 10.0);
    }

    #[test]
    fn reciprocal_round_trips() {
        for degrees in [0.0, 45.0, 179.0, 180.0, 359.9] {
            let direction = TrueCourse::new(degrees).unwrap();
            assert!((direction.reciprocal().reciprocal().degrees() - degrees).abs() < 1e-12);
        }
    }

    #[test]
    fn signed_difference_takes_the_short_way() {
        let a = TrueCourse::new(350.0).unwrap();
        let b = TrueCourse::new(10.0).unwrap();
        assert!((a.signed_difference(b) - 20.0).abs() < 1e-12);
        assert!((b.signed_difference(a) + 20.0).abs() < 1e-12);
        assert!((a.angular_distance(b) - 20.0).abs() < 1e-12);
    }

    #[test]
    fn variation_and_deviation_validate() {
        assert!(Variation::new(-181.0).is_err());
        assert!(Variation::new(f64::NAN).is_err());
        assert!(Variation::new(180.0).is_ok());
        assert!(Deviation::new(f64::INFINITY).is_err());
        assert_eq!(Deviation::ZERO.degrees(), 0.0);
    }

    #[test]
    fn relative_bearing_sides() {
        assert_eq!(RelativeBearing::new(0.0).unwrap().side(), Side::Ahead);
        assert_eq!(RelativeBearing::new(90.0).unwrap().side(), Side::Starboard);
        assert_eq!(RelativeBearing::new(180.0).unwrap().side(), Side::Astern);
        assert_eq!(RelativeBearing::new(270.0).unwrap().side(), Side::Port);
        assert!((RelativeBearing::new(270.0).unwrap().signed_degrees() + 90.0).abs() < 1e-12);
    }

    #[test]
    fn display_is_chart_style() {
        assert_eq!(format!("{}", TrueCourse::new(45.0).unwrap()), "045.0°T");
        assert_eq!(
            format!("{}", MagneticCourse::new(357.89).unwrap()),
            "357.9°M"
        );
        assert_eq!(format!("{}", Variation::new(-2.7).unwrap()), "2.7°W");
        assert_eq!(format!("{}", Deviation::new(1.5).unwrap()), "1.5°E");
        assert_eq!(
            format!("{}", RelativeBearing::new(300.0).unwrap()),
            "60.0° red"
        );
    }

    #[test]
    fn cardinal_points_are_multiples_of_forty_five_degrees() {
        for (index, point) in CardinalPoint::ALL.into_iter().enumerate() {
            assert_eq!(point.whole_degrees(), i32::try_from(index).unwrap() * 45);
            assert_eq!(point.degrees(), f64::from(point.whole_degrees()));
        }
    }

    #[test]
    fn cardinal_points_parse_regardless_of_case_and_padding() {
        for point in CardinalPoint::ALL {
            let name = point.abbreviation();
            assert_eq!(name.parse::<CardinalPoint>().unwrap(), point);
            assert_eq!(name.to_lowercase().parse::<CardinalPoint>().unwrap(), point);
            assert_eq!(
                format!("  {name} ").parse::<CardinalPoint>().unwrap(),
                point
            );
            assert_eq!(format!("{point}"), name);
        }
    }

    #[test]
    fn an_unknown_point_names_itself_in_the_error() {
        assert!(matches!(
            "NNE".parse::<CardinalPoint>(),
            Err(KernelError::UnknownCardinalDirection { direction }) if direction == "NNE"
        ));
        assert!("north".parse::<CardinalPoint>().is_err());
        assert!("".parse::<CardinalPoint>().is_err());
    }

    #[test]
    fn a_cardinal_point_becomes_a_direction_in_any_frame() {
        assert_eq!(CompassCourse::from(CardinalPoint::SW).degrees(), 225.0);
        assert_eq!(TrueCourse::from(CardinalPoint::N).degrees(), 0.0);
        assert_eq!(
            MagneticCourse::from(CardinalPoint::NW),
            Direction::from(CardinalPoint::NW)
        );
    }
}
