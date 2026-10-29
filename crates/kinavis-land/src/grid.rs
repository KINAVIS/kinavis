//! The map grid of a land navigation problem: one UTM zone or one projected
//! system, and points on its plane.

use kinavis_grid::{Hemisphere, ProjectedCrs, UtmUps, UtmZone};
use kinavis_kernel::{Angle, Distance, Position};

use crate::error::Result;

/// UTM false northing of the southern hemisphere, m.
const SOUTH_FALSE_NORTHING: f64 = 10_000_000.0;

/// The grid a problem is solved on: every point is projected into it, even
/// one whose standard zone is the next one, as on a map sheet that runs over
/// a zone boundary.
// No allocator to box the projected system in; a problem holds one grid.
#[allow(clippy::large_enum_variant)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MapGrid {
    /// One UTM zone and hemisphere on WGS 84.
    Utm {
        /// Zone.
        zone: UtmZone,
        /// Hemisphere, which fixes the false northing.
        hemisphere: Hemisphere,
    },
    /// A projected system: Gauss–Krüger on СК-42, a national grid.
    Projected(ProjectedCrs),
}

/// A point on the plane of a [`MapGrid`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GridPoint {
    /// Easting.
    pub easting: Distance,
    /// Northing.
    pub northing: Distance,
}

/// A position projected on a [`MapGrid`], with the meridian convergence and
/// point scale there.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OnGrid {
    /// Point.
    pub point: GridPoint,
    /// Angle from true north to grid north, clockwise positive.
    pub convergence: Angle,
    /// Grid distance over ground distance.
    pub scale: f64,
}

impl GridPoint {
    /// Point from its easting and northing in metres.
    ///
    /// # Errors
    ///
    /// [`kinavis_kernel::KernelError::NotFinite`] for a non-finite value.
    pub fn from_metres(easting: f64, northing: f64) -> Result<Self> {
        Ok(Self {
            easting: Distance::from_metres(easting)?,
            northing: Distance::from_metres(northing)?,
        })
    }

    pub(crate) fn metres(self) -> (f64, f64) {
        (self.easting.metres(), self.northing.metres())
    }
}

impl MapGrid {
    /// The UTM grid of a position's standard zone and hemisphere.
    ///
    /// # Errors
    ///
    /// [`kinavis_kernel::KernelError::OutOfRange`] in the polar caps, beyond
    /// UTM.
    pub fn utm_at(position: Position) -> Result<Self> {
        let gridded = UtmUps::from_position(position)?;
        match gridded.coordinate {
            UtmUps::Utm {
                zone, hemisphere, ..
            } => Ok(Self::Utm { zone, hemisphere }),
            UtmUps::Ups { .. } => Err(kinavis_kernel::KernelError::OutOfRange {
                parameter: "UTM latitude",
                value: position.latitude().degrees(),
                min: -80.0,
                max: 84.0,
            }
            .into()),
        }
    }

    /// A WGS 84 position on the grid.
    ///
    /// # Errors
    ///
    /// As [`UtmUps::in_zone`] or [`ProjectedCrs::project`].
    pub fn project(&self, position: Position) -> Result<OnGrid> {
        match *self {
            Self::Utm { zone, hemisphere } => {
                let gridded = UtmUps::in_zone(position, zone)?;
                let UtmUps::Utm {
                    hemisphere: own,
                    easting,
                    northing,
                    ..
                } = gridded.coordinate
                else {
                    return Err(kinavis_kernel::KernelError::Indeterminate {
                        quantity: "UTM coordinate of a polar position",
                    }
                    .into());
                };
                let shift = match (own, hemisphere) {
                    (Hemisphere::South, Hemisphere::North) => -SOUTH_FALSE_NORTHING,
                    (Hemisphere::North, Hemisphere::South) => SOUTH_FALSE_NORTHING,
                    _ => 0.0,
                };
                Ok(OnGrid {
                    point: GridPoint::from_metres(easting.metres(), northing.metres() + shift)?,
                    convergence: gridded.convergence,
                    scale: gridded.scale,
                })
            }
            Self::Projected(crs) => {
                let projected = crs.project(position)?;
                Ok(OnGrid {
                    point: GridPoint {
                        easting: projected.easting,
                        northing: projected.northing,
                    },
                    convergence: projected.convergence,
                    scale: projected.scale,
                })
            }
        }
    }

    /// The WGS 84 position of a grid point, with the convergence and scale
    /// there.
    ///
    /// # Errors
    ///
    /// As [`UtmUps::to_position`] or [`ProjectedCrs::unproject`].
    pub fn unproject(&self, point: GridPoint) -> Result<(Position, OnGrid)> {
        match *self {
            Self::Utm { zone, hemisphere } => {
                let (easting, northing) = point.metres();
                // A northing past the equator belongs to the other hemisphere.
                let (own, northing) = match hemisphere {
                    Hemisphere::North if northing < 0.0 => {
                        (Hemisphere::South, northing + SOUTH_FALSE_NORTHING)
                    }
                    Hemisphere::South if northing >= SOUTH_FALSE_NORTHING => {
                        (Hemisphere::North, northing - SOUTH_FALSE_NORTHING)
                    }
                    _ => (hemisphere, northing),
                };
                let coordinate = UtmUps::Utm {
                    zone,
                    hemisphere: own,
                    easting: Distance::from_metres(easting)?,
                    northing: Distance::from_metres(northing)?,
                };
                let (position, convergence, scale) = coordinate.to_position()?;
                Ok((
                    position,
                    OnGrid {
                        point,
                        convergence,
                        scale,
                    },
                ))
            }
            Self::Projected(crs) => {
                let (latitude, longitude, convergence, scale) =
                    crs.projection().unproject(point.easting, point.northing)?;
                let position = crs.datum().to_wgs84(Position::new(latitude, longitude))?;
                Ok((
                    position,
                    OnGrid {
                        point,
                        convergence,
                        scale,
                    },
                ))
            }
        }
    }
}
