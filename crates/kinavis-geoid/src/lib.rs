//! Geoid heights from a grid, as a [`GeoidModel`] for the KINAVIS crates.
//!
//! The geoid is the equipotential surface that mean sea level follows; its
//! height `N` above the WGS-84 ellipsoid converts a GNSS ellipsoidal height `h`
//! into a height above mean sea level `H = h − N`. Over the Earth `N` ranges
//! from about −107 m (south of India) to +85 m (New Guinea).
//!
//! [`GridGeoid`] reads a global grid of `N` in the PGM layout of GeographicLib
//! from a borrowed byte slice — embedded, read from a file, or memory-mapped by
//! the application — without copying or allocating, and interpolates it
//! bilinearly or with GeographicLib's twelve-point cubic fit. The `egm96`
//! feature embeds EGM96 on a 15′ grid as [`GridGeoid::EGM96`].
//!
//! ```rust
//! # #[cfg(feature = "egm96")]
//! # fn main() -> Result<(), kinavis_kernel::KernelError> {
//! use kinavis_geoid::GridGeoid;
//! use kinavis_kernel::{Distance, GeodeticPoint, Height, Position};
//!
//! // An antenna 52.0 m above the ellipsoid on the Solent.
//! let antenna = GeodeticPoint::new(
//!     Position::from_degrees(50.76, -1.3)?,
//!     Height::above_ellipsoid(Distance::from_metres(52.0)?),
//! );
//! let above_sea = antenna.to_mean_sea_level(&GridGeoid::EGM96)?;
//! // The geoid lies 46.4 m above the ellipsoid there.
//! assert_eq!(format!("{:.1}", above_sea.height().value().metres()), "5.6");
//! # Ok(())
//! # }
//! # #[cfg(not(feature = "egm96"))]
//! # fn main() {}
//! ```
//!
//! # Accuracy
//!
//! The embedded grid reproduces NGA's 15′ EGM96 grid to its 3 mm storage
//! resolution. Interpolation adds at most 1.2 m (bilinear) or 0.17 m (cubic)
//! against the spherical-harmonic model, RMS 4 cm and 7 mm (GeographicLib's
//! figures for this grid). EGM96 itself differs from the true geoid by up to a
//! metre or two; EGM2008 or a national model, supplied as a grid through
//! [`GridGeoid::from_pgm`], is better where it matters.
//!
//! # Feature flags
//!
//! - `std` *(default)* — standard library maths in the kernel.
//! - `libm` — for `no_std` targets: `--no-default-features --features libm`.
//! - `egm96` *(default)* — embeds the EGM96 15′ grid (2 MB) as
//!   [`GridGeoid::EGM96`]. Without it, only [`GridGeoid::from_pgm`].
//!
//! No allocation; builds for bare-metal targets.

#![cfg_attr(not(feature = "std"), no_std)]

mod pgm;

/// Runs the README example as a doctest.
#[cfg(all(doctest, feature = "egm96"))]
#[doc = include_str!("../README.md")]
pub struct ReadmeExamples;

use kinavis_kernel::error::{KernelError, Result};
use kinavis_kernel::geodesy::GeoidModel;
use kinavis_kernel::{math, Distance, Position};

/// Interpolation between grid nodes.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum Interpolation {
    /// Bilinear in the four surrounding nodes: continuous, kinks at grid
    /// lines.
    Bilinear,
    /// GeographicLib's least-squares cubic over twelve nodes: several times
    /// more accurate, three times the reads. Fitted per cell, so values step
    /// across cell edges: by about a millimetre typically, by up to 12 cm on
    /// the EGM96 15′ grid.
    #[default]
    Cubic,
}

/// Global geoid grid over a borrowed PGM image.
///
/// Rows run from 90°N to 90°S, columns eastwards from 0°E; `height` rows
/// include both poles, `width` columns close the circle without repeating
/// 0°E. Sample `s` stands for `N = offset + scale · s` metres.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GridGeoid<'a> {
    samples: &'a [u8],
    width: usize,
    height: usize,
    offset: f64,
    scale: f64,
    interpolation: Interpolation,
}

#[cfg(feature = "egm96")]
impl GridGeoid<'static> {
    /// EGM96 on a 15′ grid, cubic interpolation.
    ///
    /// NGA's grid (public domain), as distributed by PROJ, stored in 3 mm
    /// steps; see `tools/egm96_pgm.py`.
    pub const EGM96: Self = Self {
        samples: pgm::EGM96_SAMPLES,
        width: 1440,
        height: 721,
        offset: -108.0,
        scale: 0.003,
        interpolation: Interpolation::Cubic,
    };
}

impl<'a> GridGeoid<'a> {
    /// Grid from a PGM image: `P5` header with `# Offset` and `# Scale`
    /// comments, width and height, maximum value 65535, then 16-bit
    /// big-endian samples. Cubic interpolation.
    ///
    /// The bytes are untrusted: the header is checked and the sample count
    /// must match it exactly.
    ///
    /// # Errors
    ///
    /// - [`KernelError::Parse`] for a malformed header: no `P5`, missing or
    ///   non-finite offset, non-positive scale, odd width, even height, a grid
    ///   smaller than 2 × 3, a maximum value other than 65535.
    /// - [`KernelError::InsufficientData`] when the sample bytes differ from
    ///   width × height × 2, in either direction.
    pub fn from_pgm(bytes: &'a [u8]) -> Result<Self> {
        let header = pgm::Header::parse(bytes)?;
        Ok(Self {
            samples: header.samples,
            width: header.width,
            height: header.height,
            offset: header.offset,
            scale: header.scale,
            interpolation: Interpolation::Cubic,
        })
    }

    /// Same grid with another interpolation.
    #[must_use]
    pub const fn with_interpolation(mut self, interpolation: Interpolation) -> Self {
        self.interpolation = interpolation;
        self
    }

    /// Interpolation in use.
    #[must_use]
    pub const fn interpolation(&self) -> Interpolation {
        self.interpolation
    }

    /// Columns, rows.
    #[must_use]
    pub const fn dimensions(&self) -> (usize, usize) {
        (self.width, self.height)
    }

    /// Geoid height above the ellipsoid at a latitude and longitude, degrees.
    fn height_at(&self, latitude: f64, longitude: f64) -> Result<f64> {
        let rows = self.height.saturating_sub(1);
        let half = index(rows / 2);
        let longitude_resolution = math::count_to_f64(self.width) / 360.0;
        let latitude_resolution = math::count_to_f64(rows) / 180.0;
        let east = longitude * longitude_resolution;
        let south = -latitude * latitude_resolution;
        let column = floor(east);
        // The last cell above the south pole: the stencil never starts below
        // it.
        let row = floor(south).min(half.saturating_sub(1));
        let fx = east - f64::from(column);
        let fy = south - f64::from(row);
        let row = row.saturating_add(half);

        let value = match self.interpolation {
            Interpolation::Bilinear => {
                let v00 = self.sample(column, row)?;
                let v01 = self.sample(column.saturating_add(1), row)?;
                let v10 = self.sample(column, row.saturating_add(1))?;
                let v11 = self.sample(column.saturating_add(1), row.saturating_add(1))?;
                let upper = (1.0 - fx) * v00 + fx * v01;
                let lower = (1.0 - fx) * v10 + fx * v11;
                (1.0 - fy) * upper + fy * lower
            }
            Interpolation::Cubic => {
                let stencil = [
                    (0, -1),
                    (1, -1),
                    (-1, 0),
                    (0, 0),
                    (1, 0),
                    (2, 0),
                    (-1, 1),
                    (0, 1),
                    (1, 1),
                    (2, 1),
                    (0, 2),
                    (1, 2),
                ];
                let mut values = [0.0; cubic::STENCIL];
                for (value, (dx, dy)) in values.iter_mut().zip(stencil) {
                    *value = self.sample(column.saturating_add(dx), row.saturating_add(dy))?;
                }
                let (coefficients, denominator) = if row == 0 {
                    (&cubic::NORTH, cubic::NORTH_DENOMINATOR)
                } else if row == index(self.height).saturating_sub(2) {
                    (&cubic::SOUTH, cubic::SOUTH_DENOMINATOR)
                } else {
                    (&cubic::INTERIOR, cubic::INTERIOR_DENOMINATOR)
                };
                let mut t = [0.0; cubic::TERMS];
                for (term, slot) in t.iter_mut().enumerate() {
                    *slot = values
                        .iter()
                        .zip(coefficients)
                        .map(|(value, row)| value * f64::from(row.get(term).copied().unwrap_or(0)))
                        .sum::<f64>()
                        / denominator;
                }
                let [t0, t1, t2, t3, t4, t5, t6, t7, t8, t9] = t;
                t0 + fx * (t1 + fx * (t3 + fx * t6))
                    + fy * (t2 + fx * (t4 + fx * t7) + fy * (t5 + fx * t8 + fy * t9))
            }
        };
        Ok(self.offset + self.scale * value)
    }

    /// Sample at a column and row, wrapping in longitude and reflecting rows
    /// beyond a pole onto the opposite meridian.
    fn sample(&self, column: i32, row: i32) -> Result<f64> {
        let width = index(self.width);
        let last = index(self.height).saturating_sub(1);
        let (column, row) = if row < 0 {
            (column.saturating_add(width / 2), row.saturating_neg())
        } else if row > last {
            (
                column.saturating_add(width / 2),
                last.saturating_mul(2).saturating_sub(row),
            )
        } else {
            (column, row)
        };
        let column = column.checked_rem_euclid(width);
        let position = usize::try_from(row)
            .ok()
            .and_then(|row| row.checked_mul(self.width))
            .and_then(|start| start.checked_add(usize::try_from(column?).ok()?))
            .and_then(|cell| cell.checked_mul(2));
        let bytes = position
            .and_then(|at| self.samples.get(at..at.checked_add(2)?))
            .ok_or(KernelError::Indeterminate {
                quantity: "geoid grid sample",
            })?;
        match bytes {
            [high, low] => Ok(f64::from(u16::from_be_bytes([*high, *low]))),
            _ => Err(KernelError::Indeterminate {
                quantity: "geoid grid sample",
            }),
        }
    }
}

impl GeoidModel for GridGeoid<'_> {
    fn undulation(&self, position: Position) -> Result<Distance> {
        let height = self.height_at(
            position.latitude().degrees(),
            position.longitude().degrees(),
        )?;
        Distance::from_metres(height)
    }
}

/// Grid dimension as a signed index; dimensions are checked against
/// [`pgm::MAX_DIMENSION`] when the grid is read, so this does not saturate.
fn index(count: usize) -> i32 {
    i32::try_from(count).unwrap_or(i32::MAX)
}

/// Largest integer not above a finite, bounded value.
fn floor(value: f64) -> i32 {
    let whole = math::trunc(value);
    let truncated = math::round_to_i32(whole);
    if value < whole {
        truncated.saturating_sub(1)
    } else {
        truncated
    }
}

/// GeographicLib's cubic fit: coefficients of the ten terms of a bivariate
/// cubic from the twelve stencil values, as integer matrices over a common
/// denominator. At the first and last row the terms in `x` alone are held at
/// zero, so the height at the pole does not depend on longitude.
///
/// Values from GeographicLib `Geoid.cpp` (C. Karney, MIT licence).
mod cubic {
    /// Stencil points.
    pub(crate) const STENCIL: usize = 12;
    /// Polynomial terms: 1, x, y, x², xy, y², x³, x²y, xy², y³.
    pub(crate) const TERMS: usize = 10;

    pub(crate) const INTERIOR_DENOMINATOR: f64 = 240.0;
    pub(crate) const INTERIOR: [[i16; TERMS]; STENCIL] = [
        [9, -18, -88, 0, 96, 90, 0, 0, -60, -20],
        [-9, 18, 8, 0, -96, 30, 0, 0, 60, -20],
        [9, -88, -18, 90, 96, 0, -20, -60, 0, 0],
        [186, -42, -42, -150, -96, -150, 60, 60, 60, 60],
        [54, 162, -78, 30, -24, -90, -60, 60, -60, 60],
        [-9, -32, 18, 30, 24, 0, 20, -60, 0, 0],
        [-9, 8, 18, 30, -96, 0, -20, 60, 0, 0],
        [54, -78, 162, -90, -24, 30, 60, -60, 60, -60],
        [-54, 78, 78, 90, 144, 90, -60, -60, -60, -60],
        [9, -8, -18, -30, -24, 0, 20, 60, 0, 0],
        [-9, 18, -32, 0, 24, 30, 0, 0, -60, 20],
        [9, -18, -8, 0, -24, -30, 0, 0, 60, 20],
    ];

    pub(crate) const NORTH_DENOMINATOR: f64 = 372.0;
    pub(crate) const NORTH: [[i16; TERMS]; STENCIL] = [
        [0, 0, -131, 0, 138, 144, 0, 0, -102, -31],
        [0, 0, 7, 0, -138, 42, 0, 0, 102, -31],
        [62, 0, -31, 0, 0, -62, 0, 0, 0, 31],
        [124, 0, -62, 0, 0, -124, 0, 0, 0, 62],
        [124, 0, -62, 0, 0, -124, 0, 0, 0, 62],
        [62, 0, -31, 0, 0, -62, 0, 0, 0, 31],
        [0, 0, 45, 0, -183, -9, 0, 93, 18, 0],
        [0, 0, 216, 0, 33, 87, 0, -93, 12, -93],
        [0, 0, 156, 0, 153, 99, 0, -93, -12, -93],
        [0, 0, -45, 0, -3, 9, 0, 93, -18, 0],
        [0, 0, -55, 0, 48, 42, 0, 0, -84, 31],
        [0, 0, -7, 0, -48, -42, 0, 0, 84, 31],
    ];

    pub(crate) const SOUTH_DENOMINATOR: f64 = 372.0;
    pub(crate) const SOUTH: [[i16; TERMS]; STENCIL] = [
        [18, -36, -122, 0, 120, 135, 0, 0, -84, -31],
        [-18, 36, -2, 0, -120, 51, 0, 0, 84, -31],
        [36, -165, -27, 93, 147, -9, 0, -93, 18, 0],
        [210, 45, -111, -93, -57, -192, 0, 93, 12, 93],
        [162, 141, -75, -93, -129, -180, 0, 93, -12, 93],
        [-36, -21, 27, 93, 39, 9, 0, -93, -18, 0],
        [0, 0, 62, 0, 0, 31, 0, 0, 0, -31],
        [0, 0, 124, 0, 0, 62, 0, 0, 0, -62],
        [0, 0, 124, 0, 0, 62, 0, 0, 0, -62],
        [0, 0, 62, 0, 0, 31, 0, 0, 0, -31],
        [-18, 36, -64, 0, 66, 51, 0, 0, -102, 31],
        [18, -36, 2, 0, -66, -51, 0, 0, 102, 31],
    ];
}
