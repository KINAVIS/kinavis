//! Planar regions: polygons, regions with holes, inward and outward offsets.
//!
//! Geometry in a plane, in metres: field boundaries and obstacles, geofences,
//! airspace footprints. The plane itself is not known here. Points come from a
//! projection chosen by the caller (`kinavis-agri`'s field plane, a UTM zone, a
//! tangent plane), and every point of one region must come from the same one;
//! `north` is the plane's grid north, not true north.
//!
//! Polygons are simple (no edge crosses or touches another), have positive
//! area and are stored counter-clockwise whatever the input order. Capacity is
//! a type parameter: [`Polygon<N>`] holds at most `N` vertices, [`Region`] up to
//! `K` holes of at most `H` vertices each.
//!
//! ```rust
//! use kinavis::region::{Containment, PlanarPoint, Polygon};
//! use kinavis::Distance;
//!
//! // A 400 m × 250 m field, corners clockwise as surveyed.
//! let field = Polygon::<8>::new(&[
//!     PlanarPoint::from_metres(0.0, 0.0)?,
//!     PlanarPoint::from_metres(0.0, 250.0)?,
//!     PlanarPoint::from_metres(400.0, 250.0)?,
//!     PlanarPoint::from_metres(400.0, 0.0)?,
//! ])?;
//! assert_eq!(field.area().hectares(), 10.0);
//!
//! // A 12 m headland leaves a 376 m × 226 m work area.
//! let work = field.inset(Distance::from_metres(12.0)?)?;
//! assert_eq!(format!("{:.4}", work.area().hectares()), "8.4976");
//! let gate = PlanarPoint::from_metres(5.0, 125.0)?;
//! assert_eq!(field.contains(gate), Containment::Inside);
//! assert_eq!(work.contains(gate), Containment::Outside);
//! # Ok::<(), kinavis::NavigationError>(())
//! ```

mod holes;
mod offset;
mod polygon;

use core::fmt;

pub use holes::Region;
pub use polygon::Polygon;

#[cfg(feature = "serde")]
use crate::error::NavigationError;
use crate::error::{ensure_finite, KernelError, Result};
use crate::geodesy::Height;
use crate::math;
use crate::units::Distance;

/// Point of a plane, metres east and north of the plane's origin.
///
/// Axes are the plane's: `north` is grid north of the projection the point
/// came from.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(try_from = "[f64; 2]", into = "[f64; 2]")
)]
pub struct PlanarPoint {
    east: f64,
    north: f64,
}

impl PlanarPoint {
    /// Origin of the plane.
    pub const ORIGIN: Self = Self {
        east: 0.0,
        north: 0.0,
    };

    /// From easting and northing.
    #[must_use]
    pub fn new(east: Distance, north: Distance) -> Self {
        Self {
            east: east.metres(),
            north: north.metres(),
        }
    }

    /// From easting and northing in metres.
    ///
    /// # Errors
    ///
    /// [`KernelError::NotFinite`] for `NaN` or infinity.
    pub fn from_metres(east: f64, north: f64) -> Result<Self> {
        ensure_finite("easting", east)?;
        ensure_finite("northing", north)?;
        Ok(Self { east, north })
    }

    /// Easting.
    #[must_use]
    pub fn east(self) -> Distance {
        Distance::from_metres(self.east).unwrap_or(Distance::ZERO)
    }

    /// Northing.
    #[must_use]
    pub fn north(self) -> Distance {
        Distance::from_metres(self.north).unwrap_or(Distance::ZERO)
    }

    /// Easting, metres.
    #[must_use]
    pub const fn east_metres(self) -> f64 {
        self.east
    }

    /// Northing, metres.
    #[must_use]
    pub const fn north_metres(self) -> f64 {
        self.north
    }

    /// Straight-line distance in the plane.
    #[must_use]
    pub fn distance_to(self, other: Self) -> Distance {
        let metres = math::hypot(other.east - self.east, other.north - self.north);
        Distance::from_metres(metres).unwrap_or(Distance::ZERO)
    }

    pub(crate) const fn xy(self) -> [f64; 2] {
        [self.east, self.north]
    }

    /// From coordinates already known to be finite.
    pub(crate) const fn from_xy(xy: [f64; 2]) -> Self {
        Self {
            east: xy[0],
            north: xy[1],
        }
    }
}

impl fmt::Display for PlanarPoint {
    /// Formats as `E 412.30 m N -18.00 m`; precision defaults to 2.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let precision = f.precision().unwrap_or(2);
        write!(
            f,
            "E {:.*} m N {:.*} m",
            precision, self.east, precision, self.north
        )
    }
}

#[cfg(feature = "serde")]
impl TryFrom<[f64; 2]> for PlanarPoint {
    type Error = NavigationError;

    fn try_from([east, north]: [f64; 2]) -> Result<Self> {
        Self::from_metres(east, north)
    }
}

#[cfg(feature = "serde")]
impl From<PlanarPoint> for [f64; 2] {
    fn from(point: PlanarPoint) -> Self {
        point.xy()
    }
}

/// Area in the plane, never negative.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(try_from = "f64", into = "f64")
)]
pub struct Area(f64);

impl Area {
    /// Zero.
    pub const ZERO: Self = Self(0.0);

    /// From square metres.
    ///
    /// # Errors
    ///
    /// [`KernelError::NotFinite`] for `NaN` or infinity,
    /// [`KernelError::OutOfRange`] if negative.
    pub fn from_square_metres(value: f64) -> Result<Self> {
        ensure_finite("area", value)?;
        if value < 0.0 {
            return Err(KernelError::OutOfRange {
                parameter: "area",
                value,
                min: 0.0,
                max: f64::MAX,
            }
            .into());
        }
        Ok(Self(value))
    }

    /// From hectares.
    ///
    /// # Errors
    ///
    /// As [`Area::from_square_metres`].
    pub fn from_hectares(value: f64) -> Result<Self> {
        Self::from_square_metres(value * SQUARE_METRES_PER_HECTARE)
    }

    /// Square metres.
    #[must_use]
    pub const fn square_metres(self) -> f64 {
        self.0
    }

    /// Hectares.
    #[must_use]
    pub fn hectares(self) -> f64 {
        self.0 / SQUARE_METRES_PER_HECTARE
    }

    /// From a value known to be finite and non-negative.
    pub(crate) const fn from_square_metres_unchecked(value: f64) -> Self {
        Self(value)
    }
}

const SQUARE_METRES_PER_HECTARE: f64 = 10_000.0;

impl fmt::Display for Area {
    /// Formats as `12.50 ha`; precision defaults to 2.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let precision = f.precision().unwrap_or(2);
        write!(f, "{:.*} ha", precision, self.hectares())
    }
}

#[cfg(feature = "serde")]
impl TryFrom<f64> for Area {
    type Error = NavigationError;

    fn try_from(value: f64) -> Result<Self> {
        Self::from_square_metres(value)
    }
}

#[cfg(feature = "serde")]
impl From<Area> for f64 {
    fn from(area: Area) -> Self {
        area.0
    }
}

/// Where a point lies relative to a polygon or region.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Containment {
    /// Strictly inside.
    Inside,
    /// Exactly on an edge or a vertex.
    OnBoundary,
    /// Strictly outside, or inside a hole.
    Outside,
}

/// Lower and upper height limits of a prism over a region.
///
/// Either limit may be absent (from the ground, unlimited). Both must share
/// one vertical datum.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(try_from = "StoredVerticalLimits", into = "StoredVerticalLimits")
)]
pub struct VerticalLimits {
    lower: Option<Height>,
    upper: Option<Height>,
}

impl VerticalLimits {
    /// No limit either way.
    pub const UNLIMITED: Self = Self {
        lower: None,
        upper: None,
    };

    /// Limits from a lower and an upper height.
    ///
    /// # Errors
    ///
    /// [`KernelError::VerticalDatumMismatch`] if the datums differ,
    /// [`KernelError::OutOfRange`] if the lower limit is above the upper.
    pub fn new(lower: Option<Height>, upper: Option<Height>) -> Result<Self> {
        if let (Some(low), Some(high)) = (lower, upper) {
            let thickness = high.difference(low)?.metres();
            if thickness < 0.0 {
                return Err(KernelError::OutOfRange {
                    parameter: "lower limit above the upper, metres",
                    value: -thickness,
                    min: 0.0,
                    max: 0.0,
                }
                .into());
            }
        }
        Ok(Self { lower, upper })
    }

    /// Lower limit.
    #[must_use]
    pub const fn lower(&self) -> Option<Height> {
        self.lower
    }

    /// Upper limit.
    #[must_use]
    pub const fn upper(&self) -> Option<Height> {
        self.upper
    }

    /// Whether a height lies within the limits, both inclusive.
    ///
    /// # Errors
    ///
    /// [`KernelError::VerticalDatumMismatch`] if the height is on another
    /// datum than a limit.
    pub fn contains(&self, height: Height) -> Result<bool> {
        if let Some(low) = self.lower {
            if height.difference(low)?.metres() < 0.0 {
                return Ok(false);
            }
        }
        if let Some(high) = self.upper {
            if high.difference(height)?.metres() < 0.0 {
                return Ok(false);
            }
        }
        Ok(true)
    }
}

#[cfg(feature = "serde")]
#[derive(serde::Serialize, serde::Deserialize)]
struct StoredVerticalLimits {
    lower: Option<Height>,
    upper: Option<Height>,
}

#[cfg(feature = "serde")]
impl TryFrom<StoredVerticalLimits> for VerticalLimits {
    type Error = NavigationError;

    fn try_from(stored: StoredVerticalLimits) -> Result<Self> {
        Self::new(stored.lower, stored.upper)
    }
}

#[cfg(feature = "serde")]
impl From<VerticalLimits> for StoredVerticalLimits {
    fn from(limits: VerticalLimits) -> Self {
        Self {
            lower: limits.lower,
            upper: limits.upper,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geodesy::VerticalDatum;
    use alloc::format;

    fn metres(value: f64) -> Distance {
        Distance::from_metres(value).unwrap_or(Distance::ZERO)
    }

    #[test]
    fn a_point_keeps_its_metres() {
        let point = PlanarPoint::new(metres(412.3), metres(-18.0));
        assert!((point.east_metres() - 412.3).abs() < 1e-9);
        assert!((point.north().metres() + 18.0).abs() < 1e-9);
        assert_eq!(format!("{point:.1}"), "E 412.3 m N -18.0 m");
        assert!(PlanarPoint::from_metres(f64::NAN, 0.0).is_err());
        let far = PlanarPoint::from_metres(3.0, 4.0).unwrap_or_default();
        assert!((PlanarPoint::ORIGIN.distance_to(far).metres() - 5.0).abs() < 1e-9);
    }

    #[test]
    fn an_area_is_never_negative() {
        assert!(Area::from_square_metres(-1.0).is_err());
        assert!(Area::from_square_metres(f64::INFINITY).is_err());
        let area = Area::from_hectares(2.5).unwrap_or_default();
        assert_eq!(area.square_metres(), 25_000.0);
        assert_eq!(format!("{area}"), "2.50 ha");
    }

    #[test]
    fn vertical_limits_hold_one_datum_in_order() {
        let low = Height::above_mean_sea_level(metres(100.0));
        let high = Height::above_mean_sea_level(metres(300.0));
        let limits = VerticalLimits::new(Some(low), Some(high));
        assert!(limits.is_ok());
        let limits = limits.unwrap_or(VerticalLimits::UNLIMITED);
        assert_eq!(limits.contains(low), Ok(true));
        assert_eq!(
            limits.contains(Height::above_mean_sea_level(metres(301.0))),
            Ok(false)
        );
        assert!(limits
            .contains(Height::new(metres(200.0), VerticalDatum::Ellipsoid))
            .is_err());
        assert!(VerticalLimits::new(Some(high), Some(low)).is_err());
        assert!(
            VerticalLimits::new(Some(low), Some(Height::above_ellipsoid(metres(1.0)))).is_err()
        );
        assert_eq!(
            VerticalLimits::UNLIMITED.contains(Height::above_ellipsoid(metres(9e3))),
            Ok(true)
        );
    }
}
