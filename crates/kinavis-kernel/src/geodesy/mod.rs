//! Earth figure, points with height, Earth-centred Cartesian coordinates.
//!
//! A [`Position`] is latitude and longitude only (the chart view used by the
//! sailings). Adding height raises "above what?", and adding a Cartesian frame
//! raises "on which ellipsoid?". Both are explicit here: a [`Height`] carries
//! its [`VerticalDatum`], a [`GeodeticPoint`] is a position with such a height,
//! and an [`EcefPoint`] is reached only through a named [`Ellipsoid`].
//!
//! A [`Position`] is on WGS 84 by definition. A position from an older chart is
//! on that chart's [`Datum`] and may be hundreds of metres off;
//! [`Datum::to_wgs84`] and [`Datum::from_wgs84`] make the shift explicit via a
//! [`Helmert`] transformation.
//!
//! ```rust
//! use kinavis_kernel::geodesy::{EcefPoint, Ellipsoid, GeodeticPoint, Height};
//! use kinavis_kernel::{Distance, Position};
//!
//! let here: Position = "50°45.3'N 001°20.0'W".parse()?;
//! let point = GeodeticPoint::new(here, Height::above_ellipsoid(Distance::from_metres(48.0)?));
//!
//! let ecef = EcefPoint::from_geodetic(point, &Ellipsoid::WGS84)?;
//! let back = ecef.to_geodetic(&Ellipsoid::WGS84)?;
//!
//! assert!((back.position().latitude().degrees() - here.latitude().degrees()).abs() < 1e-9);
//! assert!((back.height().value().metres() - 48.0).abs() < 1e-3);
//! # Ok::<(), kinavis_kernel::KernelError>(())
//! ```

mod datum;

use core::fmt;

use crate::error::{KernelError, Result};
use crate::math;
use crate::position::{Latitude, Longitude, Position};
use crate::units::Distance;

pub use datum::{Datum, Helmert};

/// Reference ellipsoid.
///
/// Defined by equatorial radius and flattening; polar radius and eccentricities
/// are derived. Common chart ellipsoids are provided; others via
/// [`Ellipsoid::new`].
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(try_from = "StoredEllipsoid", into = "StoredEllipsoid")
)]
pub struct Ellipsoid {
    semi_major_metres: f64,
    inverse_flattening: f64,
}

impl Ellipsoid {
    /// WGS 84, the GNSS reference ellipsoid.
    pub const WGS84: Self = Self {
        semi_major_metres: 6_378_137.0,
        inverse_flattening: 298.257_223_563,
    };

    /// GRS 80 (ITRF and most national datums); differs from WGS 84 by 0.1 mm at
    /// the pole.
    pub const GRS80: Self = Self {
        semi_major_metres: 6_378_137.0,
        inverse_flattening: 298.257_222_101,
    };

    /// International 1924 (Hayford): ED50 and many older charts.
    pub const INTERNATIONAL_1924: Self = Self {
        semi_major_metres: 6_378_388.0,
        inverse_flattening: 297.0,
    };

    /// Clarke 1866: NAD27.
    ///
    /// Defined by both axes; inverse flattening derived as `a / (a − b)`.
    pub const CLARKE_1866: Self = Self {
        semi_major_metres: 6_378_206.4,
        inverse_flattening: 294.978_698_213_898,
    };

    /// Airy 1830: OSGB36.
    pub const AIRY_1830: Self = Self {
        semi_major_metres: 6_377_563.396,
        inverse_flattening: 299.324_964_6,
    };

    /// Krassowsky 1940: Pulkovo 1942, charts of the former USSR.
    pub const KRASSOWSKY_1940: Self = Self {
        semi_major_metres: 6_378_245.0,
        inverse_flattening: 298.3,
    };

    /// Bessel 1841: Tokyo datum, DHDN.
    pub const BESSEL_1841: Self = Self {
        semi_major_metres: 6_377_397.155,
        inverse_flattening: 299.152_812_8,
    };

    /// Australian National Spheroid: AGD66; same figure as GRS 1967 Modified
    /// (SAD69).
    pub const AUSTRALIAN_NATIONAL: Self = Self {
        semi_major_metres: 6_378_160.0,
        inverse_flattening: 298.25,
    };

    /// Ellipsoid from equatorial radius and inverse flattening.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] for a non-positive radius or an inverse
    /// flattening below 1 (`f64::INFINITY`, a sphere, is accepted).
    pub fn new(semi_major_axis: Distance, inverse_flattening: f64) -> Result<Self> {
        Self::from_raw(semi_major_axis.metres(), inverse_flattening)
    }

    /// Single check shared by the typed constructor and deserialisation. `NaN`
    /// fails every comparison, so it is rejected explicitly.
    fn from_raw(semi_major_metres: f64, inverse_flattening: f64) -> Result<Self> {
        if semi_major_metres.is_nan() || semi_major_metres <= 0.0 {
            return Err(KernelError::OutOfRange {
                parameter: "semi-major axis",
                value: semi_major_metres,
                min: f64::MIN_POSITIVE,
                max: f64::MAX,
            });
        }
        if inverse_flattening.is_nan() || inverse_flattening < 1.0 {
            return Err(KernelError::OutOfRange {
                parameter: "inverse flattening",
                value: inverse_flattening,
                min: 1.0,
                max: f64::INFINITY,
            });
        }
        Ok(Self {
            semi_major_metres,
            inverse_flattening,
        })
    }

    /// Equatorial radius `a`.
    #[must_use]
    pub fn semi_major_axis(&self) -> Distance {
        // Finite positive metres; cannot fail.
        Distance::from_metres(self.semi_major_metres).unwrap_or(Distance::ZERO)
    }

    /// Polar radius `b = a (1 − f)`.
    #[must_use]
    pub fn semi_minor_axis(&self) -> Distance {
        Distance::from_metres(self.semi_minor_metres()).unwrap_or(Distance::ZERO)
    }

    /// Flattening `f = (a − b) / a`.
    #[must_use]
    pub fn flattening(&self) -> f64 {
        1.0 / self.inverse_flattening
    }

    /// Inverse flattening `1 / f`.
    #[must_use]
    pub const fn inverse_flattening(&self) -> f64 {
        self.inverse_flattening
    }

    /// First eccentricity squared `e² = 2f − f²`.
    #[must_use]
    pub fn first_eccentricity_squared(&self) -> f64 {
        let f = self.flattening();
        f * (2.0 - f)
    }

    fn semi_minor_metres(&self) -> f64 {
        self.semi_major_metres * (1.0 - self.flattening())
    }

    /// Second eccentricity squared `e′² = e² / (1 − e²)`.
    fn second_eccentricity_squared(&self) -> f64 {
        let e2 = self.first_eccentricity_squared();
        e2 / (1.0 - e2)
    }

    /// Prime vertical radius of curvature `N` at a latitude.
    fn prime_vertical_radius(&self, sin_latitude: f64) -> f64 {
        self.semi_major_metres
            / math::sqrt(1.0 - self.first_eccentricity_squared() * sin_latitude * sin_latitude)
    }
}

/// Vertical datum.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum VerticalDatum {
    /// Reference ellipsoid surface (GNSS).
    Ellipsoid,
    /// Mean sea level / geoid (barometric, topographic).
    MeanSeaLevel,
    /// Chart datum, usually LAT: soundings are below it, tide heights above it.
    ChartDatum,
}

/// Serialised form; deserialisation applies the [`Ellipsoid::new`] checks,
/// rejecting a zero axis.
#[cfg(feature = "serde")]
#[derive(serde::Serialize, serde::Deserialize)]
struct StoredEllipsoid {
    semi_major_metres: f64,
    inverse_flattening: f64,
}

#[cfg(feature = "serde")]
impl TryFrom<StoredEllipsoid> for Ellipsoid {
    type Error = KernelError;

    fn try_from(stored: StoredEllipsoid) -> Result<Self> {
        Self::from_raw(stored.semi_major_metres, stored.inverse_flattening)
    }
}

#[cfg(feature = "serde")]
impl From<Ellipsoid> for StoredEllipsoid {
    fn from(ellipsoid: Ellipsoid) -> Self {
        Self {
            semi_major_metres: ellipsoid.semi_major_metres,
            inverse_flattening: ellipsoid.inverse_flattening,
        }
    }
}

impl VerticalDatum {
    const fn name(self) -> &'static str {
        match self {
            Self::Ellipsoid => "the ellipsoid",
            Self::MeanSeaLevel => "mean sea level",
            Self::ChartDatum => "chart datum",
        }
    }
}

/// Height with its vertical datum.
///
/// Positive up; a depth is a negative height. Converting between datums needs a
/// geoid or tide model, hence the datum is part of the value.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Height {
    value: Distance,
    datum: VerticalDatum,
}

impl Height {
    /// Height above the ellipsoid.
    #[must_use]
    pub const fn above_ellipsoid(value: Distance) -> Self {
        Self {
            value,
            datum: VerticalDatum::Ellipsoid,
        }
    }

    /// Height above MSL.
    #[must_use]
    pub const fn above_mean_sea_level(value: Distance) -> Self {
        Self {
            value,
            datum: VerticalDatum::MeanSeaLevel,
        }
    }

    /// Height above chart datum.
    #[must_use]
    pub const fn above_chart_datum(value: Distance) -> Self {
        Self {
            value,
            datum: VerticalDatum::ChartDatum,
        }
    }

    /// Height on any datum.
    #[must_use]
    pub const fn new(value: Distance, datum: VerticalDatum) -> Self {
        Self { value, datum }
    }

    /// Height, positive up.
    #[must_use]
    pub const fn value(&self) -> Distance {
        self.value
    }

    /// Vertical datum.
    #[must_use]
    pub const fn datum(&self) -> VerticalDatum {
        self.datum
    }
}

impl fmt::Display for Height {
    /// Formats as `48.0 m above the ellipsoid`.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let precision = f.precision().unwrap_or(1);
        write!(
            f,
            "{:.*} m above {}",
            precision,
            self.value.metres(),
            self.datum.name()
        )
    }
}

/// Position with height.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct GeodeticPoint {
    position: Position,
    height: Height,
}

impl GeodeticPoint {
    /// Point from horizontal position and height.
    #[must_use]
    pub const fn new(position: Position, height: Height) -> Self {
        Self { position, height }
    }

    /// Horizontal position.
    #[must_use]
    pub const fn position(&self) -> Position {
        self.position
    }

    /// Height with datum.
    #[must_use]
    pub const fn height(&self) -> Height {
        self.height
    }
}

impl fmt::Display for GeodeticPoint {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}, {}", self.position, self.height)
    }
}

/// Earth-centred, Earth-fixed Cartesian point, metres.
///
/// Origin at the centre of mass; `x` towards 0°N 0°E, `z` towards the north
/// pole, `y` towards 0°N 90°E. Reached from a [`GeodeticPoint`] via an
/// [`Ellipsoid`], and only from an ellipsoidal height: MSL heights need a geoid
/// model, which this crate does not provide.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(try_from = "StoredEcefPoint", into = "StoredEcefPoint")
)]
pub struct EcefPoint {
    x: f64,
    y: f64,
    z: f64,
}

impl EcefPoint {
    /// Point from three coordinates.
    #[must_use]
    pub fn new(x: Distance, y: Distance, z: Distance) -> Self {
        Self {
            x: x.metres(),
            y: y.metres(),
            z: z.metres(),
        }
    }

    /// Towards 0°N 0°E.
    #[must_use]
    pub fn x(&self) -> Distance {
        Distance::from_metres(self.x).unwrap_or(Distance::ZERO)
    }

    /// Towards 0°N 90°E.
    #[must_use]
    pub fn y(&self) -> Distance {
        Distance::from_metres(self.y).unwrap_or(Distance::ZERO)
    }

    /// Towards the north pole.
    #[must_use]
    pub fn z(&self) -> Distance {
        Distance::from_metres(self.z).unwrap_or(Distance::ZERO)
    }

    /// ECEF coordinates of a geodetic point on an ellipsoid.
    ///
    /// # Errors
    ///
    /// [`KernelError::VerticalDatumMismatch`] unless the height is ellipsoidal;
    /// MSL heights need a geoid model supplied by an adapter.
    pub fn from_geodetic(point: GeodeticPoint, ellipsoid: &Ellipsoid) -> Result<Self> {
        if point.height.datum != VerticalDatum::Ellipsoid {
            return Err(KernelError::VerticalDatumMismatch {
                required: VerticalDatum::Ellipsoid,
                found: point.height.datum,
            });
        }
        let (sin_lat, cos_lat) = sin_cos(point.position.latitude().radians());
        let (sin_lon, cos_lon) = sin_cos(point.position.longitude().radians());
        let n = ellipsoid.prime_vertical_radius(sin_lat);
        let h = point.height.value.metres();
        let e2 = ellipsoid.first_eccentricity_squared();
        Ok(Self {
            x: (n + h) * cos_lat * cos_lon,
            y: (n + h) * cos_lat * sin_lon,
            z: (n * (1.0 - e2) + h) * sin_lat,
        })
    }

    /// Geodetic point and ellipsoidal height on an ellipsoid.
    ///
    /// Bowring (1985) closed form: no iteration, sub-millimetre from the
    /// surface to satellite altitudes. Round trip with
    /// [`EcefPoint::from_geodetic`] holds to `1e-9°` and `1e-3 m`, poles and
    /// antimeridian included (property-tested).
    ///
    /// # Errors
    ///
    /// [`KernelError::Indeterminate`] at the Earth's centre, and for points so
    /// close to the polar axis inside the Earth that the geodetic height is
    /// undefined.
    // Notation follows Bowring's paper.
    #[allow(clippy::many_single_char_names)]
    pub fn to_geodetic(self, ellipsoid: &Ellipsoid) -> Result<GeodeticPoint> {
        let a = ellipsoid.semi_major_metres;
        let b = ellipsoid.semi_minor_metres();
        let e2 = ellipsoid.first_eccentricity_squared();
        let ep2 = ellipsoid.second_eccentricity_squared();

        let p = math::hypot(self.x, self.y);
        let r = math::hypot(p, self.z);
        if r < f64::MIN_POSITIVE {
            return Err(KernelError::Indeterminate {
                quantity: "the geodetic position of the Earth's centre",
            });
        }
        let longitude = Longitude::from_degrees(math::to_degrees(math::atan2(self.y, self.x)))?;

        // On the polar axis the parametric latitude is 90° and the height is
        // the distance along the axis from the pole.
        if p < f64::MIN_POSITIVE * a {
            let latitude = if self.z < 0.0 {
                Latitude::SOUTH_POLE
            } else {
                Latitude::NORTH_POLE
            };
            let height = Distance::from_metres(math::abs(self.z) - b)?;
            return Ok(GeodeticPoint::new(
                Position::new(latitude, longitude),
                Height::above_ellipsoid(height),
            ));
        }

        // Bowring (1985): parametric latitude u from the geocentric latitude,
        // then geodetic latitude from u in closed form.
        let tan_u = (b * self.z / (a * p)) * (1.0 + ep2 * b / r);
        let cos_u = 1.0 / math::sqrt(1.0 + tan_u * tan_u);
        let sin_u = tan_u * cos_u;
        let latitude_radians = math::atan2(
            self.z + ep2 * b * sin_u * sin_u * sin_u,
            p - e2 * a * cos_u * cos_u * cos_u,
        );
        let (sin_lat, cos_lat) = sin_cos(latitude_radians);
        let n = ellipsoid.prime_vertical_radius(sin_lat);
        let height_metres = p * cos_lat + self.z * sin_lat - a * a / n;

        let latitude = Latitude::from_degrees(math::to_degrees(latitude_radians))?;
        let height = Distance::from_metres(height_metres)?;
        Ok(GeodeticPoint::new(
            Position::new(latitude, longitude),
            Height::above_ellipsoid(height),
        ))
    }

    /// Straight-line (chord) distance.
    #[must_use]
    pub fn chord_to(&self, other: Self) -> Distance {
        let chord = math::hypot(
            math::hypot(other.x - self.x, other.y - self.y),
            other.z - self.z,
        );
        Distance::from_metres(chord).unwrap_or(Distance::ZERO)
    }
}

/// Serialised form: three metres; deserialisation checks each is finite.
#[cfg(feature = "serde")]
#[derive(serde::Serialize, serde::Deserialize)]
struct StoredEcefPoint {
    x: f64,
    y: f64,
    z: f64,
}

#[cfg(feature = "serde")]
impl TryFrom<StoredEcefPoint> for EcefPoint {
    type Error = KernelError;

    fn try_from(stored: StoredEcefPoint) -> Result<Self> {
        Ok(Self::new(
            Distance::from_metres(stored.x)?,
            Distance::from_metres(stored.y)?,
            Distance::from_metres(stored.z)?,
        ))
    }
}

#[cfg(feature = "serde")]
impl From<EcefPoint> for StoredEcefPoint {
    fn from(point: EcefPoint) -> Self {
        Self {
            x: point.x,
            y: point.y,
            z: point.z,
        }
    }
}

fn sin_cos(radians: f64) -> (f64, f64) {
    (math::sin(radians), math::cos(radians))
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use alloc::format;

    fn point(latitude: f64, longitude: f64, height: f64) -> GeodeticPoint {
        GeodeticPoint::new(
            Position::new(
                Latitude::from_degrees(latitude).unwrap(),
                Longitude::from_degrees(longitude).unwrap(),
            ),
            Height::above_ellipsoid(Distance::from_metres(height).unwrap()),
        )
    }

    #[test]
    fn wgs84_derived_constants_match_the_published_ones() {
        let e = Ellipsoid::WGS84;
        assert!((e.semi_minor_axis().metres() - 6_356_752.314_245).abs() < 1e-6);
        assert!((e.first_eccentricity_squared() - 6.694_379_990_14e-3).abs() < 1e-14);
        assert!((e.second_eccentricity_squared() - 6.739_496_742_28e-3).abs() < 1e-14);
        assert_eq!(e.inverse_flattening(), 298.257_223_563);
        assert!((Ellipsoid::GRS80.semi_minor_axis().metres() - 6_356_752.314_140).abs() < 1e-6);
    }

    #[test]
    fn an_ellipsoid_must_be_a_plausible_shape() {
        assert!(Ellipsoid::new(Distance::from_metres(0.0).unwrap(), 300.0).is_err());
        assert!(Ellipsoid::new(Distance::from_metres(6.4e6).unwrap(), 0.5).is_err());
        assert!(Ellipsoid::new(Distance::from_metres(6.4e6).unwrap(), f64::NAN).is_err());
        // A sphere: zero flattening.
        let sphere =
            Ellipsoid::new(Distance::from_metres(6_371_000.0).unwrap(), f64::INFINITY).unwrap();
        assert_eq!(sphere.flattening(), 0.0);
        assert_eq!(sphere.semi_minor_axis().metres(), 6_371_000.0);
    }

    #[test]
    fn ecef_of_reference_points_matches_the_textbook() {
        // On the equator at Greenwich: (a, 0, 0).
        let origin = EcefPoint::from_geodetic(point(0.0, 0.0, 0.0), &Ellipsoid::WGS84).unwrap();
        assert!((origin.x().metres() - 6_378_137.0).abs() < 1e-6);
        assert!(origin.y().metres().abs() < 1e-9);
        assert!(origin.z().metres().abs() < 1e-9);
        // The north pole: (0, 0, b).
        let pole = EcefPoint::from_geodetic(point(90.0, 0.0, 0.0), &Ellipsoid::WGS84).unwrap();
        assert!(pole.x().metres().abs() < 1e-6);
        assert!((pole.z().metres() - 6_356_752.314_245).abs() < 1e-6);
        // 34°N 117°W, 251 m, computed independently from the defining formulas.
        let inland =
            EcefPoint::from_geodetic(point(34.0, -117.0, 251.0), &Ellipsoid::WGS84).unwrap();
        assert!((inland.x().metres() - -2_403_183.467).abs() < 1e-3);
        assert!((inland.y().metres() - -4_716_513.119).abs() < 1e-3);
        assert!((inland.z().metres() - 3_546_586.921).abs() < 1e-3);
        // 45°N 45°E, 1000 m: x and y equal by symmetry.
        let diagonal =
            EcefPoint::from_geodetic(point(45.0, 45.0, 1000.0), &Ellipsoid::WGS84).unwrap();
        assert!((diagonal.x().metres() - 3_194_919.145).abs() < 1e-3);
        assert!((diagonal.x().metres() - diagonal.y().metres()).abs() < 1e-6);
        assert!((diagonal.z().metres() - 4_488_055.516).abs() < 1e-3);
    }

    #[test]
    fn the_round_trip_holds_at_the_awkward_places() {
        let places = [
            (0.0, 0.0, 0.0),
            (90.0, 0.0, 0.0),
            (-90.0, 45.0, 1000.0),
            (89.999_999, 179.999_999, -50.0),
            (-45.0, -180.0, 20_200_000.0),
            (50.755, -1.333, 48.0),
            (1e-9, 1e-9, 0.0),
        ];
        for (latitude, longitude, height) in places {
            let there = point(latitude, longitude, height);
            let back = EcefPoint::from_geodetic(there, &Ellipsoid::WGS84)
                .unwrap()
                .to_geodetic(&Ellipsoid::WGS84)
                .unwrap();
            assert!(
                (back.position().latitude().degrees() - latitude).abs() < 1e-9,
                "latitude at {latitude} {longitude} {height}: {}",
                back.position().latitude().degrees()
            );
            assert!(
                back.position()
                    .longitude_difference(there.position())
                    .degrees()
                    .abs()
                    < 1e-9
                    || latitude.abs() == 90.0,
                "longitude at {latitude} {longitude} {height}"
            );
            assert!(
                (back.height().value().metres() - height).abs() < 1e-3,
                "height at {latitude} {longitude} {height}: {}",
                back.height().value().metres()
            );
        }
    }

    #[test]
    fn a_sea_level_height_does_not_pretend_to_be_ellipsoidal() {
        let msl = GeodeticPoint::new(
            point(50.0, 0.0, 0.0).position(),
            Height::above_mean_sea_level(Distance::from_metres(10.0).unwrap()),
        );
        assert_eq!(
            EcefPoint::from_geodetic(msl, &Ellipsoid::WGS84),
            Err(KernelError::VerticalDatumMismatch {
                required: VerticalDatum::Ellipsoid,
                found: VerticalDatum::MeanSeaLevel,
            })
        );
        assert_eq!(format!("{}", msl.height()), "10.0 m above mean sea level");
    }

    #[test]
    fn the_centre_of_the_earth_has_no_position() {
        let centre = EcefPoint::new(Distance::ZERO, Distance::ZERO, Distance::ZERO);
        assert!(matches!(
            centre.to_geodetic(&Ellipsoid::WGS84),
            Err(KernelError::Indeterminate { .. })
        ));
    }

    #[test]
    fn the_chord_is_the_straight_line_through_the_earth() {
        let north = EcefPoint::from_geodetic(point(90.0, 0.0, 0.0), &Ellipsoid::WGS84).unwrap();
        let south = EcefPoint::from_geodetic(point(-90.0, 0.0, 0.0), &Ellipsoid::WGS84).unwrap();
        assert!((north.chord_to(south).metres() - 2.0 * 6_356_752.314_245).abs() < 1e-6);
    }
}
