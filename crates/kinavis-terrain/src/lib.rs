//! Terrain for the KINAVIS crates: SRTM height tiles over borrowed bytes,
//! slope and aspect, profiles, line of sight, walking time.
//!
//! 1. **Tiles**: [`HgtTile`] reads an `.hgt` file the application holds —
//!    memory-mapped, in flash, embedded — without copying it, and
//!    interpolates heights bilinearly; voids are refused, not guessed.
//! 2. **Many tiles**: a [`TileSource`] finds the tile of a corner;
//!    [`Terrain`] over it is a [`kinavis_kernel::ElevationModel`], the
//!    kernel's port, as one tile is.
//! 3. **Analysis** over any elevation model: [`slope_aspect`] by Horn's
//!    method, [`profile`] and its [`summarise`]d ascent and descent,
//!    [`line_of_sight`] with the Earth's curvature and refraction.
//! 4. **Walking time**: [`WalkingPace::NAISMITH`] and paces of one's own.
//!
//! ```rust
//! use kinavis_kernel::{Distance, ElevationModel, Position};
//! use kinavis_terrain::{HgtTile, TileCorner, WalkingPace};
//!
//! // A 3 × 3 tile rising 100 m a sample eastward: bytes as a file holds them.
//! let rows: [[i16; 3]; 3] = [[0, 100, 200]; 3];
//! let bytes: Vec<u8> = rows.iter().flatten().flat_map(|h| h.to_be_bytes()).collect();
//! let tile = HgtTile::new(&bytes, "N46E007".parse::<TileCorner>()?)?;
//! let height = tile.elevation(Position::from_degrees(46.5, 7.25)?)?;
//! assert_eq!(height.value().metres().round(), 50.0);
//!
//! // Two hours on the level and an hour of climbing.
//! let time = WalkingPace::NAISMITH.time(Distance::from_kilometres(10.0)?, Distance::from_metres(600.0)?);
//! assert_eq!(time.as_secs(), 3 * 3600);
//! # Ok::<(), kinavis_kernel::KernelError>(())
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

mod analysis;
mod hgt;
mod source;
mod walk;

pub use analysis::{
    line_of_sight, profile, slope_aspect, summarise, ProfilePoint, ProfileSummary, Sight,
    SlopeAspect, MAX_SIGHT_STEPS, STANDARD_REFRACTION,
};
pub use hgt::{HgtTile, TileCorner, MAX_SAMPLES_PER_SIDE, VOID};
pub use source::{Terrain, TileSource};
pub use walk::WalkingPace;
