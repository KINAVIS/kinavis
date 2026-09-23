//! Typed field readers.
//!
//! Each reader takes the field bytes and index and returns a typed value or an
//! [`NmeaError`] carrying the index. `optional_*` readers return `None` for an
//! empty field (the standard's "no data"); an empty required field is an error.

use core::str::FromStr;

use kinavis_kernel::gnss::Dop;
use kinavis_kernel::time::Civil;
use kinavis_kernel::{
    Distance, EastWest, KernelError, Latitude, Longitude, NorthSouth, Position, Speed, TrueCourse,
    Variation,
};

use crate::error::NmeaError;
use crate::sentence::{Date, Mode, Status, TimeOfDay};

/// Field with its index, for error reporting.
#[derive(Debug, Clone, Copy)]
pub(crate) struct Field<'a> {
    pub(crate) bytes: &'a [u8],
    pub(crate) index: usize,
}

impl<'a> Field<'a> {
    pub(crate) fn is_empty(&self) -> bool {
        self.bytes.is_empty()
    }

    fn bad(&self, expected: &'static str) -> NmeaError {
        NmeaError::BadField {
            index: self.index,
            expected,
        }
    }

    fn value(&self, error: KernelError) -> NmeaError {
        NmeaError::Value {
            index: self.index,
            error,
        }
    }

    /// Field as `&str`; input is ASCII, so this cannot fail.
    fn text(&self) -> &'a str {
        core::str::from_utf8(self.bytes).unwrap_or("")
    }

    /// Decimal number: optional sign, digits, at most one point.
    ///
    /// `inf`, `nan` and exponents are rejected, so the result is always finite.
    pub(crate) fn decimal(&self, expected: &'static str) -> Result<f64, NmeaError> {
        // Optional leading sign, then digits and at most one point.
        let body = match self.bytes {
            [b'+' | b'-', rest @ ..] => rest,
            all => all,
        };
        let (mut digits, mut points) = (0_usize, 0_usize);
        for &byte in body {
            match byte {
                b'0'..=b'9' => digits = digits.saturating_add(1),
                b'.' => points = points.saturating_add(1),
                _ => return Err(self.bad(expected)),
            }
        }
        if digits == 0 || points > 1 {
            return Err(self.bad(expected));
        }
        let value = f64::from_str(self.text()).map_err(|_| self.bad(expected))?;
        if value.is_finite() {
            Ok(value)
        } else {
            Err(self.bad(expected))
        }
    }

    /// Unsigned integer, digits only.
    pub(crate) fn unsigned<T: FromStr>(&self, expected: &'static str) -> Result<T, NmeaError> {
        if self.bytes.is_empty() || !self.bytes.iter().all(u8::is_ascii_digit) {
            return Err(self.bad(expected));
        }
        self.text().parse().map_err(|_| self.bad(expected))
    }

    /// Single character from a set.
    fn letter(&self, expected: &'static str) -> Result<u8, NmeaError> {
        match self.bytes {
            [byte] => Ok(*byte),
            _ => Err(self.bad(expected)),
        }
    }

    /// `hhmmss` with optional fractional seconds.
    pub(crate) fn time_of_day(&self) -> Result<TimeOfDay, NmeaError> {
        let (whole, fraction) = split_fraction(self.bytes);
        if whole.len() != 6 || !whole.iter().all(u8::is_ascii_digit) {
            return Err(self.bad("time"));
        }
        let nanos = fraction_nanos(fraction).ok_or_else(|| self.bad("time"))?;
        Ok(TimeOfDay {
            hour: two_digits(whole, 0),
            minute: two_digits(whole, 2),
            second: two_digits(whole, 4),
            nanos,
        })
    }

    /// `ddmmyy`; century 20xx.
    pub(crate) fn date(&self) -> Result<Date, NmeaError> {
        if self.bytes.len() != 6 || !self.bytes.iter().all(u8::is_ascii_digit) {
            return Err(self.bad("date"));
        }
        Ok(Date {
            year: 2000 + i32::from(two_digits(self.bytes, 4)),
            month: two_digits(self.bytes, 2),
            day: two_digits(self.bytes, 0),
        })
    }

    /// `ddmm.mmmm`: two degree digits, then minutes.
    pub(crate) fn latitude(&self, hemisphere: Field<'_>) -> Result<Latitude, NmeaError> {
        let (degrees, minutes) = self.degrees_minutes(2, "latitude")?;
        let hemisphere = match hemisphere.letter("hemisphere")? {
            b'N' => NorthSouth::North,
            b'S' => NorthSouth::South,
            _ => return Err(hemisphere.bad("hemisphere")),
        };
        Latitude::from_degrees_minutes(degrees, minutes, hemisphere).map_err(|e| self.value(e))
    }

    /// `dddmm.mmmm`: three degree digits, then minutes.
    pub(crate) fn longitude(&self, hemisphere: Field<'_>) -> Result<Longitude, NmeaError> {
        let (degrees, minutes) = self.degrees_minutes(3, "longitude")?;
        let hemisphere = match hemisphere.letter("hemisphere")? {
            b'E' => EastWest::East,
            b'W' => EastWest::West,
            _ => return Err(hemisphere.bad("hemisphere")),
        };
        Longitude::from_degrees_minutes(degrees, minutes, hemisphere).map_err(|e| self.value(e))
    }

    /// Splits `d…dmm.mmmm` exactly into whole degrees and decimal minutes,
    /// without an intermediate float.
    fn degrees_minutes(
        &self,
        degree_digits: usize,
        expected: &'static str,
    ) -> Result<(u16, f64), NmeaError> {
        let (whole, fraction) = split_fraction(self.bytes);
        if whole.len() != degree_digits.saturating_add(2) || !whole.iter().all(u8::is_ascii_digit) {
            return Err(self.bad(expected));
        }
        let (degrees, minutes) = whole.split_at(degree_digits);
        let degrees = core::str::from_utf8(degrees)
            .ok()
            .and_then(|text| text.parse::<u16>().ok())
            .ok_or_else(|| self.bad(expected))?;
        let whole_minutes = f64::from(two_digits(minutes, 0));
        let fraction = fraction_of_unit(fraction).ok_or_else(|| self.bad(expected))?;
        Ok((degrees, whole_minutes + fraction))
    }

    /// Latitude, hemisphere, longitude, hemisphere: all four present or all
    /// four empty.
    pub(crate) fn position(
        latitude: Field<'a>,
        north_south: Field<'a>,
        longitude: Field<'a>,
        east_west: Field<'a>,
    ) -> Result<Option<Position>, NmeaError> {
        let present = [latitude, north_south, longitude, east_west].map(|field| !field.is_empty());
        if present == [false; 4] {
            return Ok(None);
        }
        if present != [true; 4] {
            let missing = [latitude, north_south, longitude, east_west]
                .into_iter()
                .find(Field::is_empty)
                .unwrap_or(latitude);
            return Err(missing.bad("position"));
        }
        Ok(Some(Position::new(
            latitude.latitude(north_south)?,
            longitude.longitude(east_west)?,
        )))
    }

    pub(crate) fn optional_speed_knots(&self) -> Result<Option<Speed>, NmeaError> {
        if self.is_empty() {
            return Ok(None);
        }
        Speed::from_knots(self.decimal("speed")?)
            .map(Some)
            .map_err(|e| self.value(e))
    }

    pub(crate) fn optional_speed_kmh(&self) -> Result<Option<Speed>, NmeaError> {
        if self.is_empty() {
            return Ok(None);
        }
        Speed::from_kilometres_per_hour(self.decimal("speed")?)
            .map(Some)
            .map_err(|e| self.value(e))
    }

    pub(crate) fn optional_course(&self) -> Result<Option<TrueCourse>, NmeaError> {
        if self.is_empty() {
            return Ok(None);
        }
        TrueCourse::new(self.decimal("course")?)
            .map(Some)
            .map_err(|e| self.value(e))
    }

    pub(crate) fn optional_magnetic_course(
        &self,
    ) -> Result<Option<kinavis_kernel::MagneticCourse>, NmeaError> {
        if self.is_empty() {
            return Ok(None);
        }
        kinavis_kernel::MagneticCourse::new(self.decimal("course")?)
            .map(Some)
            .map_err(|e| self.value(e))
    }

    /// Variation with its `E`/`W` field; west is negative.
    pub(crate) fn optional_variation(
        &self,
        east_west: Field<'_>,
    ) -> Result<Option<Variation>, NmeaError> {
        if self.is_empty() && east_west.is_empty() {
            return Ok(None);
        }
        let magnitude = self.decimal("variation")?;
        let sign = match east_west.letter("variation direction")? {
            b'E' => 1.0,
            b'W' => -1.0,
            _ => return Err(east_west.bad("variation direction")),
        };
        Variation::new(magnitude * sign)
            .map(Some)
            .map_err(|e| self.value(e))
    }

    pub(crate) fn optional_metres(&self) -> Result<Option<Distance>, NmeaError> {
        if self.is_empty() {
            return Ok(None);
        }
        Distance::from_metres(self.decimal("distance")?)
            .map(Some)
            .map_err(|e| self.value(e))
    }

    pub(crate) fn optional_dop(&self) -> Result<Option<Dop>, NmeaError> {
        if self.is_empty() {
            return Ok(None);
        }
        Dop::new(self.decimal("dilution of precision")?)
            .map(Some)
            .map_err(|e| self.value(e))
    }

    pub(crate) fn optional_unsigned<T: FromStr>(
        &self,
        expected: &'static str,
    ) -> Result<Option<T>, NmeaError> {
        if self.is_empty() {
            return Ok(None);
        }
        self.unsigned(expected).map(Some)
    }

    pub(crate) fn optional_time_of_day(&self) -> Result<Option<TimeOfDay>, NmeaError> {
        if self.is_empty() {
            return Ok(None);
        }
        self.time_of_day().map(Some)
    }

    pub(crate) fn optional_date(&self) -> Result<Option<Date>, NmeaError> {
        if self.is_empty() {
            return Ok(None);
        }
        self.date().map(Some)
    }

    /// `A` valid, `V` warning.
    pub(crate) fn status(&self) -> Result<Status, NmeaError> {
        match self.letter("status")? {
            b'A' => Ok(Status::Valid),
            b'V' => Ok(Status::Warning),
            _ => Err(self.bad("status")),
        }
    }

    /// Mode indicator; absent before NMEA 2.3.
    pub(crate) fn optional_mode(&self) -> Result<Option<Mode>, NmeaError> {
        if self.is_empty() {
            return Ok(None);
        }
        Mode::from_letter(self.letter("mode")?)
            .map(Some)
            .ok_or_else(|| self.bad("mode"))
    }

    /// GGA fix quality digit.
    pub(crate) fn fix_quality(&self) -> Result<u8, NmeaError> {
        match self.bytes {
            [digit @ b'0'..=b'9'] => Ok(digit - b'0'),
            _ => Err(self.bad("fix quality")),
        }
    }

    /// Field that must contain exactly the given letter (e.g. `M` for metres).
    pub(crate) fn expect_letter(
        &self,
        letter: u8,
        expected: &'static str,
    ) -> Result<(), NmeaError> {
        if self.is_empty() || self.bytes == [letter] {
            Ok(())
        } else {
            Err(self.bad(expected))
        }
    }
}

/// Splits at the decimal point into integer part and fraction digits.
fn split_fraction(bytes: &[u8]) -> (&[u8], &[u8]) {
    match bytes.iter().position(|&byte| byte == b'.') {
        Some(point) => (
            bytes.get(..point).unwrap_or(&[]),
            bytes.get(point.saturating_add(1)..).unwrap_or(&[]),
        ),
        None => (bytes, &[]),
    }
}

/// Two decimal digits at an offset already bounds-checked by the caller.
///
/// A non-digit counts as zero, keeping each digit below 10 and the pair below
/// 100 without further checks.
fn two_digits(bytes: &[u8], at: usize) -> u8 {
    let digit = |offset: usize| {
        bytes
            .get(at.saturating_add(offset))
            .filter(|byte| byte.is_ascii_digit())
            .map_or(0, |byte| byte - b'0')
    };
    digit(0) * 10 + digit(1)
}

/// Fraction digits as a value in `[0, 1)`; `None` on a non-digit.
///
/// Accumulated digit by digit, so `.45` is the exact sum of its digits.
fn fraction_of_unit(digits: &[u8]) -> Option<f64> {
    let mut value = 0.0;
    let mut scale = 1.0;
    for &byte in digits {
        if !byte.is_ascii_digit() {
            return None;
        }
        scale /= 10.0;
        value += f64::from(byte - b'0') * scale;
    }
    Some(value)
}

/// Fraction digits as nanoseconds, truncated after the ninth.
fn fraction_nanos(digits: &[u8]) -> Option<u32> {
    let mut nanos = 0_u32;
    let mut place = 100_000_000_u32;
    for &byte in digits {
        if !byte.is_ascii_digit() {
            return None;
        }
        if place > 0 {
            // Each place is used once, so the sum stays below one second;
            // saturating arithmetic states this without an induction proof.
            nanos = nanos.saturating_add(u32::from(byte - b'0').saturating_mul(place));
            place /= 10;
        }
    }
    Some(nanos)
}

/// Combines date and time of day into a calendar value.
pub(crate) fn civil(date: Date, time: TimeOfDay) -> Civil {
    Civil {
        year: date.year,
        month: date.month,
        day: date.day,
        hour: time.hour,
        minute: time.minute,
        second: time.second,
        nanos: time.nanos,
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;

    fn field(bytes: &[u8]) -> Field<'_> {
        Field { bytes, index: 0 }
    }

    #[test]
    fn decimals_accept_only_the_plain_shape() {
        assert_eq!(field(b"12.5").decimal("x").unwrap(), 12.5);
        assert_eq!(field(b"-0.5").decimal("x").unwrap(), -0.5);
        assert_eq!(field(b".5").decimal("x").unwrap(), 0.5);
        assert_eq!(field(b"5.").decimal("x").unwrap(), 5.0);
        for bad in [
            &b""[..],
            b".",
            b"1.2.3",
            b"1e5",
            b"inf",
            b"nan",
            b"1-",
            b"--1",
            b" 1",
        ] {
            assert!(field(bad).decimal("x").is_err(), "{bad:?}");
        }
    }

    #[test]
    fn coordinates_split_degrees_from_minutes_exactly() {
        let latitude = field(b"4916.45").latitude(field(b"N")).unwrap();
        assert!((latitude.degrees() - (49.0 + 16.45 / 60.0)).abs() < 1e-12);
        let longitude = field(b"12311.12").longitude(field(b"W")).unwrap();
        assert!((longitude.degrees() + (123.0 + 11.12 / 60.0)).abs() < 1e-12);
        assert!(field(b"916.45").latitude(field(b"N")).is_err());
        assert!(field(b"4916.45").latitude(field(b"E")).is_err());
        assert!(matches!(
            field(b"9916.45").latitude(field(b"N")).unwrap_err(),
            NmeaError::Value { .. }
        ));
    }

    #[test]
    fn times_and_dates_read_their_fixed_layout() {
        assert_eq!(
            field(b"225444.25").time_of_day().unwrap(),
            TimeOfDay {
                hour: 22,
                minute: 54,
                second: 44,
                nanos: 250_000_000
            }
        );
        assert_eq!(field(b"225444").time_of_day().unwrap().nanos, 0);
        assert!(field(b"22544").time_of_day().is_err());
        assert!(field(b"225444.2x").time_of_day().is_err());
        assert_eq!(
            field(b"110926").date().unwrap(),
            Date {
                year: 2026,
                month: 9,
                day: 11
            }
        );
        assert!(field(b"1109").date().is_err());
    }

    #[test]
    fn a_position_is_all_or_nothing() {
        let empty = field(b"");
        assert_eq!(Field::position(empty, empty, empty, empty).unwrap(), None);
        assert!(Field::position(field(b"4916.45"), field(b"N"), empty, empty).is_err());
        assert!(Field::position(
            field(b"4916.45"),
            field(b"N"),
            field(b"12311.12"),
            field(b"W")
        )
        .unwrap()
        .is_some());
    }
}
