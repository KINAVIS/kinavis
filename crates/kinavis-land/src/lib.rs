//! Land navigation for the KINAVIS crates: bearings on a map grid, mils,
//! intersection and resection, pace counting, routes by bearing.
//!
//! The problems of the topographic map: a point is known by its grid
//! coordinates (UTM, MGRS, a Gauss–Krüger sheet), a direction by its grid
//! bearing from grid north or its magnetic bearing from the compass.
//!
//! 1. **Bearings**: [`BearingFrame`] turns grid, true and magnetic bearings
//!    into one another from the meridian convergence and the magnetic
//!    declination; [`Mils`] reads angles in the 6000 of Soviet maps or the
//!    6400 of NATO.
//! 2. **Grid**: [`MapGrid`] — one UTM zone or a projected system — holds the
//!    plane a problem is solved on.
//! 3. **Intersection and resection**: [`intersection`], [`resection`] by
//!    bearings, [`three_point_resection`] from readings with an unknown zero.
//! 4. **Pace count and route by bearings**: [`PaceCount`], [`Leg`],
//!    [`route_from_legs`] and the card back, [`legs_of`].
//!
//! ```rust
//! use kinavis_land::{intersection, BearingFrame, GridPoint, MapGrid, Mils};
//! use kinavis_kernel::{Angle, GridBearing, MagneticBearing, Position, Variation};
//!
//! // Two observers 2 km apart sight a tower; their compasses read 0-50 and
//! // 58-50 (Soviet mils) on a sheet where the margin gives a grid magnetic
//! // angle of +10°.
//! let frame = BearingFrame::new(Angle::from_degrees(1.5)?, Variation::new(11.5)?);
//! let to_grid = |mils: f64| -> Result<GridBearing, kinavis_land::LandError> {
//!     Ok(frame.magnetic_to_grid(Mils::Soviet.direction::<kinavis_kernel::Magnetic>(mils)?))
//! };
//! let west = GridPoint::from_metres(410_000.0, 6_180_000.0)?;
//! let east = GridPoint::from_metres(412_000.0, 6_180_000.0)?;
//! let tower = intersection(west, to_grid(50.0)?, east, to_grid(5850.0)?)?;
//! assert!(tower.northing.metres() > 6_180_000.0);
//!
//! // The grid of a fix in Moscow, UTM zone 37N: west of its central
//! // meridian, 39°E, grid north lies west of true north.
//! let grid = MapGrid::utm_at(Position::from_degrees(55.75, 37.62)?)?;
//! let on = grid.project(Position::from_degrees(55.76, 37.63)?)?;
//! assert!(on.convergence.degrees() < 0.0);
//! # let _: MagneticBearing = frame.grid_to_magnetic(to_grid(50.0)?);
//! # Ok::<(), kinavis_land::LandError>(())
//! ```
//!
//! # Feature flags
//!
//! - `std` *(default)* — standard library maths.
//! - `libm` — for `no_std` targets: `--no-default-features --features libm`.
//!
//! No allocation; builds for bare-metal targets.

#![cfg_attr(not(feature = "std"), no_std)]

/// Runs the README example as a doctest.
#[cfg(doctest)]
#[doc = include_str!("../README.md")]
pub struct ReadmeExamples;

mod bearings;
mod card;
mod error;
mod grid;
mod pace;
mod resection;

pub use bearings::{BearingFrame, Mils};
pub use card::{legs_of, route_from_legs, Leg};
pub use error::{LandError, Result};
pub use grid::{GridPoint, MapGrid, OnGrid};
pub use pace::{PaceCount, MAX_SLOPE_DEGREES};
pub use resection::{
    grid_bearing, grid_distance, intersection, resection, three_point_resection, PlaneFix,
    Sighting, ThreePointFix, DANGER_LIMIT,
};
