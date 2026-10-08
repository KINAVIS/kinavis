//! Military Grid Reference System: UTM/UPS coordinates as a grid zone, a
//! 100 km square and digits, as in GeographicLib's `MGRS` (C. Karney, MIT
//! licence) and NGA.SIG.0012. Lettering of WGS 84 and other modern
//! ellipsoids (the "AA" scheme).

use core::fmt;
use core::str::FromStr;

use kinavis_kernel::error::{Excerpt, KernelError, Result};
use kinavis_kernel::{math, Distance, Position};

use crate::utm::{floor, latitude_band_index, Hemisphere, UtmUps, UtmZone};

/// Most digits per coordinate: 11, a micrometre.
pub const MAX_MGRS_PRECISION: u8 = 11;

/// Longest reference: zone, three letters, two runs of digits.
const MAX_LENGTH: usize = 2 + 3 + 2 * MAX_MGRS_PRECISION as usize;

const TILE: f64 = 100_000.0;
const ROW_PERIOD: i32 = 20;
const EVEN_ROW_SHIFT: i32 = 5;
const UPS_EASTING: i32 = 20;
const MIN_UPS_SOUTH: i32 = 8;
const MIN_UPS_NORTH: i32 = 13;
const MAX_UTM_SOUTH_ROW: i32 = 100;
const MIN_UTM_COLUMN: i32 = 1;
/// Units of the digit grid: a micrometre in metres is 10⁻⁶.
const MULTIPLIER: f64 = 1e6;
/// `MULTIPLIER · TILE`: digit-grid units per 100 km square.
const SQUARE_UNITS: i64 = 100_000_000_000;
/// Powers of ten up to `10^MAX_MGRS_PRECISION`.
const POWERS_OF_TEN: [i64; 12] = [
    1,
    10,
    100,
    1_000,
    10_000,
    100_000,
    1_000_000,
    10_000_000,
    100_000_000,
    1_000_000_000,
    10_000_000_000,
    100_000_000_000,
];

const UTM_COLUMNS: [&[u8]; 3] = [b"ABCDEFGH", b"JKLMNPQR", b"STUVWXYZ"];
const UTM_ROWS: &[u8] = b"ABCDEFGHJKLMNPQRSTUV";
const UPS_COLUMNS: [&[u8]; 4] = [b"JKLPQRSTUXYZ", b"ABCFGHJKLPQR", b"RSTUXYZ", b"ABCFGHJ"];
const UPS_ROWS: [&[u8]; 2] = [b"ABCDEFGHJKLMNPQRSTUVWXYZ", b"ABCDEFGHJKLMNP"];
const LATITUDE_BANDS: &[u8] = b"CDEFGHJKLMNPQRSTUVWX";
const UPS_BANDS: &[u8] = b"ABYZ";

/// An MGRS reference, at most 27 ASCII characters, stored inline.
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub struct Mgrs {
    text: [u8; MAX_LENGTH],
    len: u8,
}

impl Mgrs {
    /// Reference of a WGS 84 position with `precision` digits per coordinate:
    /// 0 for the 100 km square, 5 for a metre, up to 11.
    ///
    /// Digits truncate, as MGRS requires: the reference names the square
    /// that contains the point.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] for a precision above
    /// [`MAX_MGRS_PRECISION`]; as [`UtmUps::from_position`].
    pub fn from_position(position: Position, precision: u8) -> Result<Self> {
        let gridded = UtmUps::from_position(position)?;
        Self::from_coordinate(gridded.coordinate, position.latitude().degrees(), precision)
    }

    /// Reference of a UTM/UPS coordinate; `latitude` picks the band where a
    /// 100 km square straddles two.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] for a precision above
    /// [`MAX_MGRS_PRECISION`] or a coordinate outside the MGRS limits;
    /// [`KernelError::Indeterminate`] when the latitude does not lie in the
    /// coordinate's band.
    pub fn from_coordinate(coordinate: UtmUps, latitude: f64, precision: u8) -> Result<Self> {
        kinavis_kernel::error::ensure_range(
            "MGRS precision",
            f64::from(precision),
            0.0,
            f64::from(MAX_MGRS_PRECISION),
        )?;
        let (zone, mut north, mut x, mut y) = match coordinate {
            UtmUps::Utm {
                zone,
                hemisphere,
                easting,
                northing,
            } => (
                Some(zone),
                hemisphere == Hemisphere::North,
                easting.metres(),
                northing.metres(),
            ),
            UtmUps::Ups {
                hemisphere,
                easting,
                northing,
            } => (
                None,
                hemisphere == Hemisphere::North,
                easting.metres(),
                northing.metres(),
            ),
        };
        check(zone.is_some(), &mut north, &mut x, &mut y)?;
        let mut out = Self {
            text: [0; MAX_LENGTH],
            len: 0,
        };
        let ix = floor_units(x * MULTIPLIER);
        let iy = floor_units(y * MULTIPLIER);
        let xh = to_i32(ix.checked_div(SQUARE_UNITS).unwrap_or(0));
        let yh = to_i32(iy.checked_div(SQUARE_UNITS).unwrap_or(0));
        match zone {
            Some(zone) => out.push_utm_letters(zone, north, latitude, xh, yh)?,
            None => out.push_ups_letters(north, xh, yh)?,
        }
        if precision > 0 {
            let divisor = POWERS_OF_TEN
                .get(usize::from(MAX_MGRS_PRECISION - precision))
                .copied()
                .unwrap_or(1);
            let square_x = ix.saturating_sub(SQUARE_UNITS.saturating_mul(i64::from(xh)));
            let square_y = iy.saturating_sub(SQUARE_UNITS.saturating_mul(i64::from(yh)));
            let digits_x = square_x.checked_div(divisor).unwrap_or(0);
            let digits_y = square_y.checked_div(divisor).unwrap_or(0);
            out.push_number(digits_x, precision);
            out.push_number(digits_y, precision);
        }
        Ok(out)
    }

    /// The reference as text.
    #[must_use]
    pub fn as_str(&self) -> &str {
        self.text
            .get(..usize::from(self.len))
            .and_then(|bytes| core::str::from_utf8(bytes).ok())
            .unwrap_or("")
    }

    fn push(&mut self, byte: u8) {
        if let Some(slot) = self.text.get_mut(usize::from(self.len)) {
            *slot = byte;
            self.len = self.len.saturating_add(1);
        }
    }

    fn push_digit(&mut self, digit: i64) {
        self.push(b'0'.saturating_add(u8::try_from(digit.rem_euclid(10)).unwrap_or(0)));
    }

    /// `value` as exactly `width` digits, leading zeros kept.
    fn push_number(&mut self, value: i64, width: u8) {
        for place in (0..width).rev() {
            let power = POWERS_OF_TEN.get(usize::from(place)).copied().unwrap_or(1);
            self.push_digit(value.checked_div(power).unwrap_or(0));
        }
    }
}

impl Mgrs {
    /// Zone digits, band, column and row letters of a UTM square.
    fn push_utm_letters(
        &mut self,
        zone: UtmZone,
        north: bool,
        latitude: f64,
        xh: i32,
        yh: i32,
    ) -> Result<()> {
        self.push_digit(i64::from(zone.number() / 10));
        self.push_digit(i64::from(zone.number() % 10));
        // Within rounding of the equator the hemisphere decides the band.
        let band = if math::abs(latitude) < ANGLE_EPSILON {
            if north {
                0
            } else {
                -1
            }
        } else {
            latitude_band_index(latitude)
        };
        let column = xh.saturating_sub(MIN_UTM_COLUMN);
        let row = utm_row(band, column, yh.rem_euclid(ROW_PERIOD));
        let expected = yh.saturating_sub(if north { 0 } else { MAX_UTM_SOUTH_ROW });
        if row != expected {
            return Err(KernelError::Indeterminate {
                quantity: "MGRS band of a latitude outside the coordinate's band",
            });
        }
        let zone_index = usize::from(zone.number().saturating_sub(1));
        self.push(letter(LATITUDE_BANDS, band.saturating_add(10))?);
        self.push(letter(
            UTM_COLUMNS.get(zone_index % 3).copied().unwrap_or(b""),
            column,
        )?);
        let shift = if zone_index % 2 == 1 {
            EVEN_ROW_SHIFT
        } else {
            0
        };
        self.push(letter(
            UTM_ROWS,
            yh.saturating_add(shift).rem_euclid(ROW_PERIOD),
        )?);
        Ok(())
    }

    /// Band, column and row letters of a UPS square.
    fn push_ups_letters(&mut self, north: bool, xh: i32, yh: i32) -> Result<()> {
        let east = xh >= UPS_EASTING;
        let band = usize::from(north) * 2 + usize::from(east);
        self.push(*UPS_BANDS.get(band).ok_or(out_of_range())?);
        let first_column = if east {
            UPS_EASTING
        } else if north {
            MIN_UPS_NORTH
        } else {
            MIN_UPS_SOUTH
        };
        self.push(letter(
            UPS_COLUMNS.get(band).copied().unwrap_or(b""),
            xh.saturating_sub(first_column),
        )?);
        let first_row = if north { MIN_UPS_NORTH } else { MIN_UPS_SOUTH };
        self.push(letter(
            UPS_ROWS.get(usize::from(north)).copied().unwrap_or(b""),
            yh.saturating_sub(first_row),
        )?);
        Ok(())
    }
}

impl fmt::Display for Mgrs {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

impl fmt::Debug for Mgrs {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Mgrs({})", self.as_str())
    }
}

/// A parsed MGRS reference: the centre of the square it names.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MgrsSquare {
    /// Centre of the square.
    pub centre: UtmUps,
    /// Digits per coordinate; `None` for a grid zone alone (`38S`), whose
    /// centre is a representative point of the band.
    pub precision: Option<u8>,
}

impl FromStr for MgrsSquare {
    type Err = KernelError;

    /// Reads `38SMB4488`, `YZG`, `38S`: zone digits (none for UPS), band,
    /// column and row letters, an even number of digits. Letters in either
    /// case; no spaces.
    fn from_str(text: &str) -> Result<Self> {
        parse(text.as_bytes()).ok_or_else(|| KernelError::Parse {
            what: "MGRS reference",
            input: Excerpt::new(text),
        })
    }
}

/// The smallest latitude distinguishable from zero near the equator, as
/// GeographicLib: `2^-(53 − 7)` degrees.
const ANGLE_EPSILON: f64 = 1.0 / 70_368_744_177_664.0;
/// The smallest step below `2 · 10⁷` m, as GeographicLib: `2^-(53 − 25)` m.
const NORTHING_EPSILON: f64 = 1.0 / 268_435_456.0;

/// MGRS coordinate limits in 100 km squares, `[min, max)`, in the order
/// UPS south, UPS north, UTM south, UTM north.
const MIN_EASTING: [i32; 4] = [8, 13, 1, 1];
const MAX_EASTING: [i32; 4] = [32, 27, 9, 9];
const MIN_NORTHING: [i32; 4] = [8, 13, 10, -90];
const MAX_NORTHING: [i32; 4] = [32, 27, 195, 95];

/// Checks a coordinate against the MGRS limits, nudging one on an excluded
/// upper edge just inside and folding a UTM northing into the right
/// hemisphere.
fn check(utm: bool, north: &mut bool, x: &mut f64, y: &mut f64) -> Result<()> {
    let index = usize::from(utm) * 2 + usize::from(*north);
    let (min_e, max_e, min_n, max_n) = (
        MIN_EASTING.get(index).copied().unwrap_or(0),
        MAX_EASTING.get(index).copied().unwrap_or(0),
        MIN_NORTHING.get(index).copied().unwrap_or(0),
        MAX_NORTHING.get(index).copied().unwrap_or(0),
    );
    let ix = floor(*x / TILE);
    let iy = floor(*y / TILE);
    if !(min_e..max_e).contains(&ix) {
        if ix == max_e && *x <= f64::from(max_e) * TILE {
            *x -= NORTHING_EPSILON;
        } else {
            return Err(range("MGRS easting", *x, min_e, max_e));
        }
    }
    if !(min_n..max_n).contains(&iy) {
        if iy == max_n && *y <= f64::from(max_n) * TILE {
            *y -= NORTHING_EPSILON;
        } else {
            return Err(range("MGRS northing", *y, min_n, max_n));
        }
    }
    if utm {
        if *north && iy < 0 {
            *north = false;
            *y += f64::from(MAX_UTM_SOUTH_ROW) * TILE;
        } else if !*north && iy >= MAX_UTM_SOUTH_ROW {
            if *y <= f64::from(MAX_UTM_SOUTH_ROW) * TILE {
                *y -= NORTHING_EPSILON;
            } else {
                *north = true;
                *y -= f64::from(MAX_UTM_SOUTH_ROW) * TILE;
            }
        }
    }
    Ok(())
}

/// True row index in `[-90, 95)` of a UTM square from its band, column and
/// periodic row; [`MAX_UTM_SOUTH_ROW`] when they are incompatible.
fn utm_row(band: i32, column: i32, periodic_row: i32) -> i32 {
    let centre = 100.0 * (8.0 * f64::from(band) + 4.0) / 90.0;
    let north = if band >= 0 { 0.1 } else { 0.0 };
    let min_row = if band > -10 {
        floor(centre - 4.3 - north)
    } else {
        -90
    };
    let max_row = if band < 9 {
        floor(centre + 4.4 - north)
    } else {
        94
    };
    let base_row = (min_row.saturating_add(max_row) / 2).saturating_sub(ROW_PERIOD / 2);
    let row = periodic_row
        .saturating_sub(base_row)
        .saturating_add(MAX_UTM_SOUTH_ROW)
        .rem_euclid(ROW_PERIOD)
        .saturating_add(base_row);
    if (min_row..=max_row).contains(&row) {
        return row;
    }
    // Northings 7100 km and 8000 km cross band edges in some columns.
    let folded_band = if band >= 0 {
        band
    } else {
        band.saturating_neg().saturating_sub(1)
    };
    let folded_row = if row >= 0 {
        row
    } else {
        row.saturating_neg().saturating_sub(1)
    };
    let folded_column = if column < 4 {
        column
    } else {
        7_i32.saturating_sub(column)
    };
    let allowed = (folded_row == 70 && folded_band == 8 && folded_column >= 2)
        || (folded_row == 71 && folded_band == 7 && folded_column <= 2)
        || (folded_row == 79 && folded_band == 9 && folded_column >= 1)
        || (folded_row == 80 && folded_band == 8 && folded_column <= 1);
    if allowed {
        row
    } else {
        MAX_UTM_SOUTH_ROW
    }
}

fn parse(text: &[u8]) -> Option<MgrsSquare> {
    let mut rest = text;
    let mut zone_number: u8 = 0;
    let mut zone_digits = 0;
    while let Some((&byte, after)) = rest.split_first() {
        if !byte.is_ascii_digit() {
            break;
        }
        zone_number = zone_number.checked_mul(10)?.checked_add(byte - b'0')?;
        zone_digits += 1;
        rest = after;
        if zone_digits > 2 {
            return None;
        }
    }
    let zone = if zone_digits > 0 {
        Some(UtmZone::new(zone_number).ok()?)
    } else {
        None
    };
    let (&band_letter, after) = rest.split_first()?;
    rest = after;
    let bands = if zone.is_some() {
        LATITUDE_BANDS
    } else {
        UPS_BANDS
    };
    let band = index_of(bands, band_letter)?;
    let north = band >= if zone.is_some() { 10 } else { 2 };
    let hemisphere = if north {
        Hemisphere::North
    } else {
        Hemisphere::South
    };
    if rest.is_empty() {
        return grid_zone_only(zone, band, hemisphere);
    }
    let [column_letter, row_letter, digits @ ..] = rest else {
        return None;
    };
    let (column, row) = square_indices(zone, band, north, *column_letter, *row_letter)?;
    if digits.len() % 2 != 0 || digits.len() > 2 * usize::from(MAX_MGRS_PRECISION) {
        return None;
    }
    let precision = digits.len() / 2;
    let (x_digits, y_digits) = digits.split_at(precision);
    let mut unit = 1.0;
    let (mut x, mut y) = (f64::from(column), f64::from(row));
    for (&dx, &dy) in x_digits.iter().zip(y_digits) {
        if !(dx.is_ascii_digit() && dy.is_ascii_digit()) {
            return None;
        }
        unit *= 10.0;
        x = 10.0 * x + f64::from(dx - b'0');
        y = 10.0 * y + f64::from(dy - b'0');
    }
    // The centre of the square.
    unit *= 2.0;
    x = 2.0 * x + 1.0;
    y = 2.0 * y + 1.0;
    let easting = Distance::from_metres(TILE * x / unit).ok()?;
    let northing = Distance::from_metres(TILE * y / unit).ok()?;
    let centre = match zone {
        Some(zone) => UtmUps::Utm {
            zone,
            hemisphere,
            easting,
            northing,
        },
        None => UtmUps::Ups {
            hemisphere,
            easting,
            northing,
        },
    };
    Some(MgrsSquare {
        centre,
        precision: u8::try_from(precision).ok(),
    })
}

/// Column and row of a 100 km square, in squares from the false origin.
fn square_indices(
    zone: Option<UtmZone>,
    band: i32,
    north: bool,
    column_letter: u8,
    row_letter: u8,
) -> Option<(i32, i32)> {
    if let Some(zone) = zone {
        let zone_index = usize::from(zone.number().saturating_sub(1));
        let column = index_of(UTM_COLUMNS.get(zone_index % 3)?, column_letter)?;
        let mut row = index_of(UTM_ROWS, row_letter)?;
        if zone_index % 2 == 1 {
            row = row
                .saturating_add(ROW_PERIOD - EVEN_ROW_SHIFT)
                .rem_euclid(ROW_PERIOD);
        }
        let row = utm_row(band.saturating_sub(10), column, row);
        if row == MAX_UTM_SOUTH_ROW {
            return None;
        }
        Some((
            column.saturating_add(MIN_UTM_COLUMN),
            if north {
                row
            } else {
                row.saturating_add(MAX_UTM_SOUTH_ROW)
            },
        ))
    } else {
        let column = index_of(UPS_COLUMNS.get(usize::try_from(band).ok()?)?, column_letter)?;
        let row = index_of(UPS_ROWS.get(usize::from(north))?, row_letter)?;
        let first = if north { MIN_UPS_NORTH } else { MIN_UPS_SOUTH };
        let east = band % 2 == 1;
        Some((
            column.saturating_add(if east { UPS_EASTING } else { first }),
            row.saturating_add(first),
        ))
    }
}

/// A representative point of a grid zone given alone: the central meridian
/// (4° east in 31V) at the middle of the band; for UPS, 86° on the 90°E or
/// 90°W meridian.
fn grid_zone_only(zone: Option<UtmZone>, band: i32, hemisphere: Hemisphere) -> Option<MgrsSquare> {
    // Tiles per degree of meridian, roughly: 10⁷ m over 90°.
    let degree = 100.0 / 90.0;
    let (x, y) = if let Some(zone) = zone {
        let x = if zone.number() == 31 && band == 17 {
            4.0
        } else {
            5.0
        };
        let y = f64::from(floor(8.0 * (f64::from(band) - 9.5) * degree + 0.5));
        (
            x * TILE,
            y * TILE
                + if hemisphere == Hemisphere::North {
                    0.0
                } else {
                    f64::from(MAX_UTM_SOUTH_ROW) * TILE
                },
        )
    } else {
        let side = if band % 2 == 1 { 1.0 } else { -1.0 };
        (
            (side * f64::from(floor(4.0 * degree + 0.5)) + f64::from(UPS_EASTING)) * TILE,
            f64::from(UPS_EASTING) * TILE,
        )
    };
    let easting = Distance::from_metres(x).ok()?;
    let northing = Distance::from_metres(y).ok()?;
    let centre = match zone {
        Some(zone) => UtmUps::Utm {
            zone,
            hemisphere,
            easting,
            northing,
        },
        None => UtmUps::Ups {
            hemisphere,
            easting,
            northing,
        },
    };
    Some(MgrsSquare {
        centre,
        precision: None,
    })
}

/// Position of an upper-case or lower-case letter in a letter set.
fn index_of(set: &[u8], letter: u8) -> Option<i32> {
    let upper = letter.to_ascii_uppercase();
    set.iter()
        .position(|&candidate| candidate == upper)
        .and_then(|index| i32::try_from(index).ok())
}

fn letter(set: &[u8], index: i32) -> Result<u8> {
    usize::try_from(index)
        .ok()
        .and_then(|index| set.get(index))
        .copied()
        .ok_or(out_of_range())
}

fn out_of_range() -> KernelError {
    KernelError::Indeterminate {
        quantity: "MGRS letter for a square outside the grid",
    }
}

fn range(parameter: &'static str, value: f64, min: i32, max: i32) -> KernelError {
    KernelError::OutOfRange {
        parameter,
        value,
        min: f64::from(min) * TILE,
        max: f64::from(max) * TILE,
    }
}

/// `floor` of a coordinate in digit-grid units; coordinates are below
/// 4·10⁶ m, so the product stays far inside `i64`.
#[allow(clippy::cast_possible_truncation)]
fn floor_units(value: f64) -> i64 {
    let whole = math::trunc(value);
    let truncated = whole as i64;
    if value < whole {
        truncated.saturating_sub(1)
    } else {
        truncated
    }
}

fn to_i32(value: i64) -> i32 {
    i32::try_from(value).unwrap_or(i32::MAX)
}
