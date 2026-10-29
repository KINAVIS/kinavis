//! SRTM height tiles (`.hgt`): one degree square of big-endian `i16`
//! heights in metres above the EGM96 geoid, rows from the north edge
//! southward, columns from the west edge eastward. Both edges are included,
//! so neighbouring tiles share their border samples: 3601 per side at 1″,
//! 1201 at 3″. −32768 marks a void.

use core::fmt;
use core::str::FromStr;

use kinavis_kernel::error::{ensure_range, Excerpt, KernelError, Result};
use kinavis_kernel::geodesy::ElevationModel;
use kinavis_kernel::{math, Distance, Height, Position};

/// Value of a void sample.
pub const VOID: i16 = i16::MIN;

/// Most samples per side accepted: 1/3″ tiles.
pub const MAX_SAMPLES_PER_SIDE: usize = 10_801;

/// The south-west corner of a tile: whole degrees, as its name gives it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct TileCorner {
    south: i16,
    west: i16,
}

impl TileCorner {
    /// Corner at latitude `south` and longitude `west`.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] outside −90…89 or −180…179.
    pub fn new(south: i16, west: i16) -> Result<Self> {
        ensure_range("tile south edge", f64::from(south), -90.0, 89.0)?;
        ensure_range("tile west edge", f64::from(west), -180.0, 179.0)?;
        Ok(Self { south, west })
    }

    /// The corner of the tile holding a position: a position on a tile
    /// border belongs to the tile north or east of it.
    #[must_use]
    pub fn of(position: Position) -> Self {
        let south = floor(position.latitude().degrees()).clamp(-90, 89);
        let west = floor(position.longitude().degrees()).clamp(-180, 179);
        Self { south, west }
    }

    /// Southern edge, degrees.
    #[must_use]
    pub const fn south(self) -> i16 {
        self.south
    }

    /// Western edge, degrees.
    #[must_use]
    pub const fn west(self) -> i16 {
        self.west
    }
}

impl FromStr for TileCorner {
    type Err = KernelError;

    /// Reads a tile name: `N55E037`, `s01w072`, with or without `.hgt`.
    fn from_str(text: &str) -> Result<Self> {
        let bad = || KernelError::Parse {
            what: "HGT tile name",
            input: Excerpt::new(text),
        };
        let stem = text
            .strip_suffix(".hgt")
            .or_else(|| text.strip_suffix(".HGT"))
            .unwrap_or(text);
        let bytes = stem.as_bytes();
        let [ns, la, lb, ew, oa, ob, oc] = bytes else {
            return Err(bad());
        };
        let digits = |values: &[u8]| -> Option<i16> {
            values.iter().try_fold(0i16, |sum, &byte| {
                byte.is_ascii_digit().then(|| {
                    sum.saturating_mul(10)
                        .saturating_add(i16::from(byte - b'0'))
                })
            })
        };
        let latitude = digits(&[*la, *lb]).ok_or_else(bad)?;
        let longitude = digits(&[*oa, *ob, *oc]).ok_or_else(bad)?;
        let south = match ns.to_ascii_uppercase() {
            b'N' => latitude,
            b'S' => -latitude,
            _ => return Err(bad()),
        };
        let west = match ew.to_ascii_uppercase() {
            b'E' => longitude,
            b'W' => -longitude,
            _ => return Err(bad()),
        };
        Self::new(south, west).map_err(|_| bad())
    }
}

impl fmt::Display for TileCorner {
    /// Formats as the tile name, `N55E037`.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let ns = if self.south < 0 { 'S' } else { 'N' };
        let ew = if self.west < 0 { 'W' } else { 'E' };
        write!(
            f,
            "{ns}{:02}{ew}{:03}",
            self.south.unsigned_abs(),
            self.west.unsigned_abs()
        )
    }
}

/// A height tile over borrowed bytes: no copy, no allocation.
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct HgtTile<'a> {
    bytes: &'a [u8],
    side: usize,
    corner: TileCorner,
}

impl<'a> HgtTile<'a> {
    /// The tile at `corner` from the bytes of its file: `2 · n²` of them for
    /// `n` samples per side, any `n` from 2 to [`MAX_SAMPLES_PER_SIDE`].
    ///
    /// # Errors
    ///
    /// [`KernelError::Parse`] for a length that is not of a square tile.
    pub fn new(bytes: &'a [u8], corner: TileCorner) -> Result<Self> {
        let side = side_of(bytes.len()).ok_or_else(|| KernelError::Parse {
            what: "HGT tile (2·n² bytes)",
            input: Excerpt::new(""),
        })?;
        Ok(Self {
            bytes,
            side,
            corner,
        })
    }

    /// Corner.
    #[must_use]
    pub const fn corner(&self) -> TileCorner {
        self.corner
    }

    /// Samples along each side.
    #[must_use]
    pub const fn samples_per_side(&self) -> usize {
        self.side
    }

    /// Spacing of the samples, arc seconds: 1 for SRTM1, 3 for SRTM3.
    #[must_use]
    pub fn spacing_arc_seconds(&self) -> f64 {
        3600.0 / math::count_to_f64(self.side.saturating_sub(1))
    }

    /// Whether the position lies in the tile, borders included.
    #[must_use]
    pub fn contains(&self, position: Position) -> bool {
        self.fraction(position).is_some()
    }

    /// The sample at `row` from the north edge and `column` from the west;
    /// `None` outside the tile or for a void.
    #[must_use]
    pub fn sample(&self, row: usize, column: usize) -> Option<i16> {
        if row >= self.side || column >= self.side {
            return None;
        }
        let index = row
            .checked_mul(self.side)?
            .checked_add(column)?
            .checked_mul(2)?;
        let pair = self.bytes.get(index..index.checked_add(2)?)?;
        let value = i16::from_be_bytes([*pair.first()?, *pair.get(1)?]);
        (value != VOID).then_some(value)
    }

    /// Height at a position, metres above the geoid, interpolated
    /// bilinearly between the four samples around it.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutsideValidity`] outside the tile or next to a void.
    pub fn height_metres(&self, position: Position) -> Result<f64> {
        let (row, column) = self.fraction(position).ok_or(OUTSIDE)?;
        let last = self.side.saturating_sub(1);
        let (r0, fr) = split(row, last);
        let (c0, fc) = split(column, last);
        let (r1, c1) = (
            r0.saturating_add(1).min(last),
            c0.saturating_add(1).min(last),
        );
        let at = |r, c| self.sample(r, c).map(f64::from).ok_or(VOIDED);
        let north = at(r0, c0)? * (1.0 - fc) + at(r0, c1)? * fc;
        let south = at(r1, c0)? * (1.0 - fc) + at(r1, c1)? * fc;
        Ok(north * (1.0 - fr) + south * fr)
    }

    /// Fractional row and column of a position, or `None` outside.
    fn fraction(&self, position: Position) -> Option<(f64, f64)> {
        let last = math::count_to_f64(self.side.saturating_sub(1));
        let north = f64::from(self.corner.south) + 1.0;
        let west = f64::from(self.corner.west);
        let row = (north - position.latitude().degrees()) * last;
        let mut east = position.longitude().degrees() - west;
        // A tile ending on the antimeridian holds 180°, read as −180°.
        if east < 0.0 && self.corner.west == 179 {
            east += 360.0;
        }
        let column = east * last;
        ((0.0..=last).contains(&row) && (0.0..=last).contains(&column)).then_some((row, column))
    }
}

impl ElevationModel for HgtTile<'_> {
    fn elevation(&self, position: Position) -> Result<Height> {
        Ok(Height::above_mean_sea_level(Distance::from_metres(
            self.height_metres(position)?,
        )?))
    }
}

impl fmt::Debug for HgtTile<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "HgtTile({}, {}²)", self.corner, self.side)
    }
}

const OUTSIDE: KernelError = KernelError::OutsideValidity {
    data: "terrain tile at the position",
};

const VOIDED: KernelError = KernelError::OutsideValidity {
    data: "terrain next to a void",
};

/// Samples per side of a tile of `len` bytes.
fn side_of(len: usize) -> Option<usize> {
    if len % 2 != 0 {
        return None;
    }
    let samples = len / 2;
    let side = math::to_usize(math::round(math::sqrt(math::count_to_f64(samples))));
    ((2..=MAX_SAMPLES_PER_SIDE).contains(&side) && side.checked_mul(side) == Some(samples))
        .then_some(side)
}

/// A fractional index split into the lower whole index, kept below `last`
/// so that its neighbour exists, and the fraction past it.
fn split(value: f64, last: usize) -> (usize, f64) {
    let below = last.saturating_sub(1);
    let whole = math::to_usize(math::trunc(value)).min(below);
    (whole, value - math::count_to_f64(whole))
}

/// Largest integer not above `value`, for degrees within ±180.
fn floor(value: f64) -> i16 {
    let whole = math::trunc(value);
    let truncated = i16::try_from(math::round_to_i32(whole)).unwrap_or(0);
    if value < whole {
        truncated.saturating_sub(1)
    } else {
        truncated
    }
}
