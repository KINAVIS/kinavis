//! Map grids for the KINAVIS crates: transverse Mercator, UTM and UPS,
//! Gauss–Krüger zones and national grids, grid north.
//!
//! A grid turns latitude and longitude into eastings and northings in metres
//! on a plane, the coordinates of topographic maps, surveys and field
//! machinery. Every point of a grid also has a *meridian convergence*, the
//! angle between grid north and true north, and a *point scale*, the ratio of
//! grid to ground distance; both are returned with each coordinate.
//!
//! ```rust
//! use kinavis_grid::{ProjectedCrs, UtmUps};
//! use kinavis_kernel::{GridCourse, Position, TrueCourse};
//!
//! let fix = Position::from_degrees(55.7539, 37.6208)?;
//!
//! // UTM, the grid of GNSS receivers.
//! let utm = UtmUps::from_position(fix)?;
//! let UtmUps::Utm { zone, easting, northing, .. } = utm.coordinate else {
//!     unreachable!("Moscow is far from the poles")
//! };
//! assert_eq!(zone.number(), 37);
//! assert_eq!(format!("{:.0} {:.0}", easting.metres(), northing.metres()), "413440 6179551");
//!
//! // The same fix on СК-42, Gauss–Krüger zone 7, as on Soviet topographic maps.
//! let sk42 = ProjectedCrs::pulkovo_1942(7)?.project(fix)?;
//! assert!(sk42.easting.metres() > 7_000_000.0);
//!
//! // A true course of 090° read off the UTM sheet: grid = true − convergence.
//! let on_the_sheet: GridCourse = TrueCourse::new(90.0)?.to_grid(utm.convergence);
//! assert!(on_the_sheet.degrees() > 90.0);
//! # Ok::<(), kinavis_kernel::KernelError>(())
//! ```
//!
//! # Accuracy
//!
//! The transverse Mercator uses Krüger's series to sixth order in the third
//! flattening (Karney 2011), transcribed from GeographicLib: about 5 nm within
//! 3900 km of the central meridian, far beyond any zone. Polar stereographic
//! is exact. Datum shifts are the kernel's, with their stated accuracy.
//!
//! # Feature flags
//!
//! - `std` *(default)* — standard library maths in the kernel.
//! - `libm` — for `no_std` targets: `--no-default-features --features libm`.
//!
//! No allocation; builds for bare-metal targets.

#![cfg_attr(not(feature = "std"), no_std)]

mod conformal;

/// Runs the README example as a doctest.
#[cfg(doctest)]
#[doc = include_str!("../README.md")]
pub struct ReadmeExamples;

mod crs;
mod tm;
mod ups;
mod utm;

use kinavis_kernel::{Angle, Variation};

pub use crs::ProjectedCrs;
pub use tm::{Projected, TmParameters, TransverseMercator};
pub use utm::{Gridded, Hemisphere, UtmUps, UtmZone, UPS_SCALE, UTM_SCALE};

/// Angle from grid north to magnetic north, east positive: the grid magnetic
/// angle printed in a map's margin, `variation − convergence`.
///
/// A magnetic bearing becomes a grid bearing by adding it.
///
/// # Errors
///
/// [`kinavis_kernel::KernelError::NotFinite`]; not reachable for validated
/// inputs.
pub fn grid_magnetic_angle(
    variation: Variation,
    convergence: Angle,
) -> kinavis_kernel::Result<Angle> {
    Angle::from_degrees(kinavis_kernel::wrap180(
        variation.degrees() - convergence.degrees(),
    ))
}
