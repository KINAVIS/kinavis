//! UTM and UPS on WGS 84, with the standard zone rules and the coordinate
//! ranges of GeographicLib's `UTMUPS` (C. Karney, MIT licence).

use core::fmt;

use kinavis_kernel::error::{ensure_range, KernelError, Result};
use kinavis_kernel::geodesy::Ellipsoid;
use kinavis_kernel::{math, Angle, Distance, Latitude, Longitude, Position};

use crate::tm::{TmParameters, TransverseMercator};
use crate::ups::PolarStereographic;

/// Scale on the central meridian of UTM.
pub const UTM_SCALE: f64 = 0.9996;
/// Scale at the pole of UPS.
pub const UPS_SCALE: f64 = 0.994;
/// UTM false easting, m.
const UTM_FALSE_EASTING: f64 = 500_000.0;
/// UTM false northing in the southern hemisphere, m.
const UTM_SOUTH_FALSE_NORTHING: f64 = 10_000_000.0;
/// UPS false easting and northing, m.
const UPS_FALSE_ORIGIN: f64 = 2_000_000.0;
/// MGRS square, m; coordinate limits are multiples of it.
const TILE: f64 = 100_000.0;

/// Hemisphere of a UTM or UPS coordinate: which false northing applies.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Hemisphere {
    /// Northern.
    North,
    /// Southern.
    South,
}

/// UTM zone, 1 to 60.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct UtmZone(u8);

impl UtmZone {
    /// Zone from its number.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] outside 1–60.
    pub fn new(number: u8) -> Result<Self> {
        ensure_range("UTM zone", f64::from(number), 1.0, 60.0)?;
        Ok(Self(number))
    }

    /// Number, 1–60.
    #[must_use]
    pub const fn number(self) -> u8 {
        self.0
    }

    /// Central meridian, degrees east: `6 · zone − 183`.
    #[must_use]
    pub fn central_meridian(self) -> f64 {
        6.0 * f64::from(self.0) - 183.0
    }

    /// Standard zone of a position between 80°S and 84°N, with the Norway and
    /// Svalbard exceptions; `None` in the polar caps.
    #[must_use]
    pub fn standard(position: Position) -> Option<Self> {
        let latitude = position.latitude().degrees();
        if !(-80.0..84.0).contains(&latitude) {
            return None;
        }
        let ilon = floor(position.longitude().degrees());
        let mut zone = ilon.saturating_add(186) / 6;
        let band = latitude_band_index(latitude);
        if band == 7 && zone == 31 && ilon >= 3 {
            zone = 32;
        } else if band == 9 && (0..42).contains(&ilon) {
            zone = (ilon.saturating_add(183) / 12)
                .saturating_mul(2)
                .saturating_add(1);
        }
        u8::try_from(zone).ok().map(Self)
    }
}

impl fmt::Display for UtmZone {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.0)
    }
}

/// Coordinate on the UTM or UPS grid.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum UtmUps {
    /// Universal Transverse Mercator.
    Utm {
        /// Zone.
        zone: UtmZone,
        /// Hemisphere.
        hemisphere: Hemisphere,
        /// Easting.
        easting: Distance,
        /// Northing.
        northing: Distance,
    },
    /// Universal Polar Stereographic.
    Ups {
        /// Hemisphere: which pole.
        hemisphere: Hemisphere,
        /// Easting.
        easting: Distance,
        /// Northing.
        northing: Distance,
    },
}

/// A position on the UTM/UPS grid, with the meridian convergence and point
/// scale there.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Gridded {
    /// Coordinate.
    pub coordinate: UtmUps,
    /// Angle from true north to grid north, clockwise positive.
    pub convergence: Angle,
    /// Point scale.
    pub scale: f64,
}

impl UtmUps {
    /// Coordinate of a WGS 84 position in its standard zone: UTM between
    /// 80°S and 84°N, UPS beyond.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] if the result lies outside the zone's
    /// coordinate limits; not reachable for a standard zone.
    pub fn from_position(position: Position) -> Result<Gridded> {
        match UtmZone::standard(position) {
            Some(zone) => Self::in_zone(position, zone),
            None => Self::polar(position),
        }
    }

    /// Coordinate of a WGS 84 position in a given UTM zone, as when a map
    /// sheet or a survey extends into the next zone.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] more than 60° from the zone's central
    /// meridian, or outside the zone's coordinate limits widened by 100 km.
    pub fn in_zone(position: Position, zone: UtmZone) -> Result<Gridded> {
        let latitude = position.latitude().degrees();
        let difference =
            kinavis_kernel::wrap180(position.longitude().degrees() - zone.central_meridian());
        ensure_range(
            "longitude from the central meridian",
            difference,
            -60.0,
            60.0,
        )?;
        let hemisphere = hemisphere_of(latitude);
        let projected = utm(zone)?.project(position.latitude(), position.longitude())?;
        let northing = projected.northing.metres()
            + if hemisphere == Hemisphere::South {
                UTM_SOUTH_FALSE_NORTHING
            } else {
                0.0
            };
        let coordinate = Self::Utm {
            zone,
            hemisphere,
            easting: projected.easting,
            northing: Distance::from_metres(northing)?,
        };
        coordinate.check()?;
        Ok(Gridded {
            coordinate,
            convergence: projected.convergence,
            scale: projected.scale,
        })
    }

    /// Coordinate of a WGS 84 position on UPS, about the nearer pole.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] within 70° of the equator, where UPS is
    /// not used.
    pub fn polar(position: Position) -> Result<Gridded> {
        let latitude = position.latitude().degrees();
        if math::abs(latitude) < 70.0 {
            return Err(KernelError::OutOfRange {
                parameter: "UPS latitude",
                value: latitude,
                min: 70.0,
                max: 90.0,
            });
        }
        let hemisphere = hemisphere_of(latitude);
        let (x, y, gamma, k) = ups().forward(
            hemisphere == Hemisphere::North,
            latitude,
            position.longitude().degrees(),
        );
        let coordinate = Self::Ups {
            hemisphere,
            easting: Distance::from_metres(x + UPS_FALSE_ORIGIN)?,
            northing: Distance::from_metres(y + UPS_FALSE_ORIGIN)?,
        };
        coordinate.check()?;
        Ok(Gridded {
            coordinate,
            convergence: Angle::from_degrees(gamma)?,
            scale: k,
        })
    }

    /// WGS 84 position of the coordinate, with the convergence and scale
    /// there.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] outside the coordinate limits widened by
    /// 100 km; [`KernelError::NotConverged`] not reachable within them.
    pub fn to_position(&self) -> Result<(Position, Angle, f64)> {
        self.check()?;
        match *self {
            Self::Utm {
                zone,
                hemisphere,
                easting,
                northing,
            } => {
                let northing = northing.metres()
                    - if hemisphere == Hemisphere::South {
                        UTM_SOUTH_FALSE_NORTHING
                    } else {
                        0.0
                    };
                let (latitude, longitude, gamma, k) =
                    utm(zone)?.unproject(easting, Distance::from_metres(northing)?)?;
                Ok((Position::new(latitude, longitude), gamma, k))
            }
            Self::Ups {
                hemisphere,
                easting,
                northing,
            } => {
                let (latitude, longitude, gamma, k) = ups().reverse(
                    hemisphere == Hemisphere::North,
                    easting.metres() - UPS_FALSE_ORIGIN,
                    northing.metres() - UPS_FALSE_ORIGIN,
                )?;
                Ok((
                    Position::new(
                        Latitude::from_degrees(latitude)?,
                        Longitude::from_degrees(longitude)?,
                    ),
                    Angle::from_degrees(gamma)?,
                    k,
                ))
            }
        }
    }

    /// Checks the coordinate against the limits of its grid, widened by one
    /// 100 km square.
    fn check(&self) -> Result<()> {
        let (easting, northing, [min_e, max_e, min_n, max_n]) = match *self {
            Self::Utm {
                hemisphere,
                easting,
                northing,
                ..
            } => (
                easting,
                northing,
                match hemisphere {
                    Hemisphere::North => [1.0, 9.0, -90.0, 95.0],
                    Hemisphere::South => [1.0, 9.0, 10.0, 195.0],
                },
            ),
            Self::Ups {
                hemisphere,
                easting,
                northing,
            } => (
                easting,
                northing,
                match hemisphere {
                    Hemisphere::North => [13.0, 27.0, 13.0, 27.0],
                    Hemisphere::South => [8.0, 32.0, 8.0, 32.0],
                },
            ),
        };
        ensure_range(
            "easting",
            easting.metres(),
            (min_e - 1.0) * TILE,
            (max_e + 1.0) * TILE,
        )?;
        ensure_range(
            "northing",
            northing.metres(),
            (min_n - 1.0) * TILE,
            (max_n + 1.0) * TILE,
        )
    }
}

/// UTM projection of a zone, no hemisphere offset.
fn utm(zone: UtmZone) -> Result<TransverseMercator> {
    TransverseMercator::new(
        &Ellipsoid::WGS84,
        TmParameters {
            central_meridian: Longitude::from_degrees(zone.central_meridian())?,
            latitude_of_origin: Latitude::EQUATOR,
            scale: UTM_SCALE,
            false_easting: Distance::from_metres(UTM_FALSE_EASTING)?,
            false_northing: Distance::ZERO,
        },
    )
}

fn ups() -> PolarStereographic {
    PolarStereographic::new(&Ellipsoid::WGS84, UPS_SCALE)
}

/// North unless the latitude carries a minus sign, `-0.0` included.
fn hemisphere_of(latitude: f64) -> Hemisphere {
    if latitude.is_sign_negative() {
        Hemisphere::South
    } else {
        Hemisphere::North
    }
}

/// MGRS latitude band, −10 (80°S–72°S) to 9 (72°N–84°N).
pub(crate) fn latitude_band_index(latitude: f64) -> i32 {
    (floor(latitude)
        .saturating_add(80)
        .div_euclid(8)
        .saturating_sub(10))
    .clamp(-10, 9)
}

/// Largest integer not above a finite, bounded value.
pub(crate) fn floor(value: f64) -> i32 {
    let whole = math::trunc(value);
    let truncated = math::round_to_i32(whole);
    if value < whole {
        truncated.saturating_sub(1)
    } else {
        truncated
    }
}
