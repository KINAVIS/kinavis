//! Navigation time with the time scale in the type.
//!
//! Navigation uses several scales: UTC (logs, tide tables; steps with leap
//! seconds), GPS time (receivers; continuous) and TAI (continuous, reference).
//! They differ by whole seconds — 18 s UTC→GPS since 2017, 185 m at 20 kn — so
//! mixing them must not compile.
//!
//! An [`Instant`] carries its scale as a type parameter, like the frame of a
//! [`Direction`](crate::Direction). Instants on the same scale compare and
//! subtract; conversion between scales is an explicit function using the
//! leap-second table:
//!
//! ```compile_fail
//! use kinavis_kernel::time::{Gps, Instant, Utc};
//!
//! let utc: Instant<Utc> = Instant::from_unix_seconds(1_700_000_000);
//! let gps: Instant<Gps> = Instant::from_unix_seconds(1_700_000_000);
//! let _ = utc.duration_since(gps); // mismatched types: `Instant<Utc>` is not `Instant<Gps>`
//! ```
//!
//! The leap-second table expires (IERS announces each leap second about six
//! months ahead), so it is a port, [`LeapSeconds`], not a constant.
//!
//! # Representation
//!
//! Every scale counts seconds and nanoseconds since `1970-01-01T00:00:00` *on
//! that scale*. For UTC this is Unix time (a leap second has no distinct
//! value); GPS and TAI count continuously through leap seconds, which makes
//! them suitable for intervals. [`Instant::civil`] gives the calendar of the
//! instant's own scale, e.g. the date a GPS receiver would display.

use core::fmt;
use core::hash::Hash;
use core::marker::PhantomData;
use core::time::Duration;

use crate::error::{KernelError, Result};

mod sealed {
    pub trait Sealed {}
}

/// Time scale of an [`Instant`].
///
/// Sealed: only the scales below exist.
pub trait TimeScale:
    sealed::Sealed + Copy + Clone + fmt::Debug + Eq + Ord + Hash + Default + 'static
{
    /// Scale suffix: `"UTC"`, `"GPS"`, `"TAI"`.
    const NAME: &'static str;
}

/// Coordinated Universal Time, with leap seconds.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Utc;

/// GPS time: continuous, 19 s behind TAI.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Gps;

/// International Atomic Time: continuous reference scale.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Tai;

impl sealed::Sealed for Utc {}
impl sealed::Sealed for Gps {}
impl sealed::Sealed for Tai {}

impl TimeScale for Utc {
    const NAME: &'static str = "UTC";
}

impl TimeScale for Gps {
    const NAME: &'static str = "GPS";
}

impl TimeScale for Tai {
    const NAME: &'static str = "TAI";
}

/// Nanoseconds per second; upper bound of [`Instant::subsec_nanos`].
const NANOS_PER_SECOND: u32 = 1_000_000_000;

/// Splits nanoseconds below 2 s into a carry second and remainder. Summing two
/// sub-second parts in `u64` cannot wrap; the remainder fits `u32` by
/// construction.
fn split_nanos(total: u64) -> (i64, u32) {
    let carry = i64::from(total >= u64::from(NANOS_PER_SECOND));
    let remainder = total % u64::from(NANOS_PER_SECOND);
    (carry, u32::try_from(remainder).unwrap_or(0))
}

/// Subtracts sub-second parts with borrow, in a wider type and with each part
/// reduced below one second first (a no-op for valid parts), so the compiler
/// can prove nothing wraps.
fn borrow_nanos(from: u32, subtract: u32) -> (i64, u32) {
    let (from, subtract) = (
        u64::from(from) % u64::from(NANOS_PER_SECOND),
        u64::from(subtract) % u64::from(NANOS_PER_SECOND),
    );
    if from >= subtract {
        (0, u32::try_from(from - subtract).unwrap_or(0))
    } else {
        (
            1,
            u32::try_from(from + u64::from(NANOS_PER_SECOND) - subtract).unwrap_or(0),
        )
    }
}

/// TAI − GPS, fixed at the GPS epoch.
pub const TAI_MINUS_GPS: Duration = Duration::from_secs(19);

/// Instant on one time scale, nanosecond resolution.
///
/// Comparison and subtraction only within one scale (see the [module
/// docs](self)). Stored as seconds and nanoseconds since the scale's
/// `1970-01-01T00:00:00`: range far beyond any calendar, and any `i64` of Unix
/// nanoseconds fits.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Instant<S: TimeScale> {
    // Field order matters: derived `Ord` compares seconds first.
    seconds: i64,
    /// Always below `NANOS_PER_SECOND`.
    nanos: u32,
    scale: PhantomData<S>,
}

impl<S: TimeScale> Instant<S> {
    /// `1970-01-01T00:00:00` on this scale.
    pub const UNIX_EPOCH: Self = Self::from_parts(0, 0);

    /// Instant from already normalised parts.
    const fn from_parts(seconds: i64, nanos: u32) -> Self {
        Self {
            seconds,
            nanos,
            scale: PhantomData,
        }
    }

    /// Instant from whole seconds and nanoseconds within the second.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] if `nanos` ≥ 1 s.
    pub fn new(seconds: i64, nanos: u32) -> Result<Self> {
        if nanos >= NANOS_PER_SECOND {
            return Err(KernelError::OutOfRange {
                parameter: "nanoseconds",
                value: f64::from(nanos),
                min: 0.0,
                max: f64::from(NANOS_PER_SECOND - 1),
            });
        }
        Ok(Self::from_parts(seconds, nanos))
    }

    /// Instant a whole number of seconds from the epoch.
    #[must_use]
    pub const fn from_unix_seconds(seconds: i64) -> Self {
        Self::from_parts(seconds, 0)
    }

    /// Instant `nanos` from the epoch (negative before it). Every `i64` is
    /// representable.
    #[must_use]
    pub const fn from_unix_nanos(nanos: i64) -> Self {
        let seconds = nanos.div_euclid(NANOS_PER_SECOND as i64);
        // The Euclidean remainder by a positive divisor that fits `u32` is
        // non-negative and below it.
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let nanos = nanos.rem_euclid(NANOS_PER_SECOND as i64) as u32;
        Self::from_parts(seconds, nanos)
    }

    /// Whole seconds since the epoch, floored.
    #[must_use]
    pub const fn seconds(self) -> i64 {
        self.seconds
    }

    /// Nanoseconds past the whole second, `0..1_000_000_000`.
    #[must_use]
    pub const fn subsec_nanos(self) -> u32 {
        self.nanos
    }

    /// Nanoseconds since the epoch, if within ~292 years (the `i64` range).
    #[must_use]
    pub const fn checked_unix_nanos(self) -> Option<i64> {
        match self.seconds.checked_mul(NANOS_PER_SECOND as i64) {
            Some(whole) => whole.checked_add(self.nanos as i64),
            None => None,
        }
    }

    /// `self + duration`; `None` on overflow.
    #[must_use]
    pub fn checked_add(self, duration: Duration) -> Option<Self> {
        let seconds = i64::try_from(duration.as_secs()).ok()?;
        // Both parts below 1 s so the sum is below 2 s; computed in a wider
        // type so the compiler can prove it.
        let (carry, nanos) =
            split_nanos(u64::from(self.nanos) + u64::from(duration.subsec_nanos()));
        let seconds = self.seconds.checked_add(seconds)?.checked_add(carry)?;
        Some(Self::from_parts(seconds, nanos))
    }

    /// `self − duration`; `None` on underflow.
    #[must_use]
    pub fn checked_sub(self, duration: Duration) -> Option<Self> {
        let seconds = i64::try_from(duration.as_secs()).ok()?;
        let (borrow, nanos) = borrow_nanos(self.nanos, duration.subsec_nanos());
        let seconds = self.seconds.checked_sub(seconds)?.checked_sub(borrow)?;
        Some(Self::from_parts(seconds, nanos))
    }

    /// `self + duration`, saturating.
    #[must_use]
    pub fn saturating_add(self, duration: Duration) -> Self {
        self.checked_add(duration)
            .unwrap_or(Self::from_parts(i64::MAX, NANOS_PER_SECOND - 1))
    }

    /// `self − duration`, saturating.
    #[must_use]
    pub fn saturating_sub(self, duration: Duration) -> Self {
        self.checked_sub(duration)
            .unwrap_or(Self::from_parts(i64::MIN, 0))
    }

    /// Duration since `earlier`; `None` unless `self ≥ earlier`.
    #[must_use]
    pub fn checked_duration_since(self, earlier: Self) -> Option<Duration> {
        if self < earlier {
            return None;
        }
        let (borrow, nanos) = borrow_nanos(self.nanos, earlier.nanos);
        // The difference can exceed `i64` across the full range; computed via
        // `u64`.
        let seconds = u64::try_from(
            i128::from(self.seconds) - i128::from(earlier.seconds) - i128::from(borrow),
        )
        .ok()?;
        Some(Duration::new(seconds, nanos))
    }

    /// Duration since `earlier`.
    ///
    /// # Errors
    ///
    /// [`KernelError::TimeReversed`] if `earlier` is later (clock stepped back
    /// or arguments swapped), with the amount.
    pub fn duration_since(self, earlier: Self) -> Result<Duration> {
        match self.checked_duration_since(earlier) {
            Some(elapsed) => Ok(elapsed),
            None => Err(KernelError::TimeReversed {
                // `earlier > self`, so `Some`.
                by: earlier.checked_duration_since(self).unwrap_or_default(),
            }),
        }
    }

    /// Instant for a calendar reading on this scale.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] for an invalid month, day, hour, minute,
    /// second or nanosecond. Second `60` (leap second) is rejected: the count
    /// has no place for it; the parser decides which side of the step it
    /// belongs to.
    pub fn from_civil(civil: Civil) -> Result<Self> {
        civil.validate()?;
        let days = days_from_civil(civil.year, civil.month, civil.day);
        let seconds = days
            .checked_mul(SECONDS_PER_DAY)
            .and_then(|day_start| {
                day_start.checked_add(
                    i64::from(civil.hour) * 3600
                        + i64::from(civil.minute) * 60
                        + i64::from(civil.second),
                )
            })
            .ok_or(KernelError::OutOfRange {
                parameter: "year",
                value: f64::from(civil.year),
                min: f64::from(i32::MIN),
                max: f64::from(i32::MAX),
            })?;
        Ok(Self::from_parts(seconds, civil.nanos))
    }

    /// Calendar reading on this instant's own scale.
    #[must_use]
    pub fn civil(self) -> Civil {
        let days = self.seconds.div_euclid(SECONDS_PER_DAY);
        let of_day = self.seconds.rem_euclid(SECONDS_PER_DAY);
        let (year, month, day) = civil_from_days(days);
        // Each quotient is bounded by the next divisor, so the narrowing casts
        // cannot truncate.
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        Civil {
            year,
            month,
            day,
            hour: (of_day / 3600) as u8,
            minute: (of_day % 3600 / 60) as u8,
            second: (of_day % 60) as u8,
            nanos: self.nanos,
        }
    }

    /// Same count on another scale, for the conversions below.
    const fn relabel<T: TimeScale>(self) -> Instant<T> {
        Instant::from_parts(self.seconds, self.nanos)
    }
}

impl<S: TimeScale> fmt::Debug for Instant<S> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{self:.9}")
    }
}

impl<S: TimeScale> fmt::Display for Instant<S> {
    /// Formats as `2026-09-11T10:15:30.250 UTC` (ISO 8601 plus scale).
    /// Precision: decimals of the second, default 3, max 9.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let civil = self.civil();
        write!(
            f,
            "{:04}-{:02}-{:02}T{:02}:{:02}:{:02}",
            civil.year, civil.month, civil.day, civil.hour, civil.minute, civil.second
        )?;
        let precision = f.precision().unwrap_or(3).min(9);
        if precision > 0 {
            let mut scaled = civil.nanos;
            for _ in precision..9 {
                scaled /= 10;
            }
            write!(f, ".{scaled:0precision$}")?;
        }
        write!(f, " {}", S::NAME)
    }
}

#[cfg(feature = "serde")]
#[derive(serde::Serialize, serde::Deserialize)]
struct RawInstant {
    seconds: i64,
    nanos: u32,
}

#[cfg(feature = "serde")]
impl<S: TimeScale> serde::Serialize for Instant<S> {
    /// Serialised as `{ "seconds", "nanos" }`; the scale is in the type.
    fn serialize<Z: serde::Serializer>(
        &self,
        serializer: Z,
    ) -> core::result::Result<Z::Ok, Z::Error> {
        RawInstant {
            seconds: self.seconds,
            nanos: self.nanos,
        }
        .serialize(serializer)
    }
}

#[cfg(feature = "serde")]
impl<'de, S: TimeScale> serde::Deserialize<'de> for Instant<S> {
    /// Deserialised through [`Instant::new`]; `nanos` ≥ 1 s is rejected.
    fn deserialize<D: serde::Deserializer<'de>>(
        deserializer: D,
    ) -> core::result::Result<Self, D::Error> {
        let raw = RawInstant::deserialize(deserializer)?;
        Self::new(raw.seconds, raw.nanos).map_err(serde::de::Error::custom)
    }
}

/// Seconds per day. No scale here has other day lengths: a UTC leap second has
/// no distinct count.
const SECONDS_PER_DAY: i64 = 86_400;

/// Calendar reading.
///
/// Public fields so a parser can fill it and pass it to
/// [`Instant::from_civil`], which validates. A `Civil` from [`Instant::civil`]
/// is always valid.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Civil {
    /// Proleptic Gregorian year; `0` is 1 BC.
    pub year: i32,
    /// Month, `1..=12`.
    pub month: u8,
    /// Day, `1..=31` as the month allows.
    pub day: u8,
    /// Hour, `0..=23`.
    pub hour: u8,
    /// Minute, `0..=59`.
    pub minute: u8,
    /// Second, `0..=59`.
    pub second: u8,
    /// Nanoseconds, `0..1_000_000_000`.
    pub nanos: u32,
}

impl Civil {
    /// Midnight on a date.
    #[must_use]
    pub const fn date(year: i32, month: u8, day: u8) -> Self {
        Self {
            year,
            month,
            day,
            hour: 0,
            minute: 0,
            second: 0,
            nanos: 0,
        }
    }

    /// Whether every field is valid.
    fn validate(self) -> Result<()> {
        check("month", u32::from(self.month), 1, 12)?;
        check(
            "day",
            u32::from(self.day),
            1,
            u32::from(days_in_month(self.year, self.month)),
        )?;
        check("hour", u32::from(self.hour), 0, 23)?;
        check("minute", u32::from(self.minute), 0, 59)?;
        check("second", u32::from(self.second), 0, 59)?;
        check("nanoseconds", self.nanos, 0, NANOS_PER_SECOND - 1)
    }
}

/// Calendar field range check.
fn check(parameter: &'static str, value: u32, min: u32, max: u32) -> Result<()> {
    if value < min || value > max {
        return Err(KernelError::OutOfRange {
            parameter,
            value: f64::from(value),
            min: f64::from(min),
            max: f64::from(max),
        });
    }
    Ok(())
}

/// Whether a proleptic Gregorian year is a leap year.
const fn is_leap_year(year: i32) -> bool {
    year % 4 == 0 && (year % 100 != 0 || year % 400 == 0)
}

/// Days in a month; `0` for an invalid month, so the day check fails too.
const fn days_in_month(year: i32, month: u8) -> u8 {
    match month {
        1 | 3 | 5 | 7 | 8 | 10 | 12 => 31,
        4 | 6 | 9 | 11 => 30,
        2 if is_leap_year(year) => 29,
        2 => 28,
        _ => 0,
    }
}

/// Days from `1970-01-01` to a date, negative before.
///
/// Hinnant's algorithm: years start on 1 March (February last) and days are
/// counted in 400-year eras, over which the leap pattern repeats exactly. Valid
/// for every `i32` year.
fn days_from_civil(year: i32, month: u8, day: u8) -> i64 {
    let year = i64::from(year) - i64::from(month <= 2);
    let era = year.div_euclid(400);
    let year_of_era = year.rem_euclid(400);
    let shifted_month = i64::from(if month > 2 { month - 3 } else { month + 9 });
    let day_of_year = (153 * shifted_month + 2) / 5 + i64::from(day) - 1;
    let day_of_era = year_of_era * 365 + year_of_era / 4 - year_of_era / 100 + day_of_year;
    era * 146_097 + day_of_era - 719_468
}

/// Inverse of [`days_from_civil`]. The year is clamped to `i32`, which no `i64`
/// second count can exceed anyway.
fn civil_from_days(days: i64) -> (i32, u8, u8) {
    // Days derive from seconds / 86 400, so the era shift cannot wrap;
    // saturating arithmetic states it.
    let days = days.saturating_add(719_468);
    let era = days.div_euclid(146_097);
    let day_of_era = days.rem_euclid(146_097);
    let year_of_era =
        (day_of_era - day_of_era / 1460 + day_of_era / 36_524 - day_of_era / 146_096) / 365;
    let day_of_year = day_of_era - (365 * year_of_era + year_of_era / 4 - year_of_era / 100);
    let shifted_month = (5 * day_of_year + 2) / 153;
    let day = day_of_year - (153 * shifted_month + 2) / 5 + 1;
    let month = if shifted_month < 10 {
        shifted_month + 3
    } else {
        shifted_month - 9
    };
    let year = year_of_era + era * 400 + i64::from(month <= 2);
    // Month `1..=12`, day `1..=31` by construction; the year is clamped, not
    // truncated.
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    (
        year.clamp(i64::from(i32::MIN), i64::from(i32::MAX)) as i32,
        month as u8,
        day as u8,
    )
}

/// Leap-second table: TAI − UTC at a given instant.
///
/// A port because any table expires (IERS announces about six months ahead);
/// past its validity an implementation returns [`KernelError::OutsideValidity`]
/// rather than the last known value. Implementations: compiled-in table with
/// expiry, file, or GNSS receiver-reported offset.
pub trait LeapSeconds {
    /// `TAI − UTC` at a UTC instant: 37 s since 2017 until the next leap
    /// second.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutsideValidity`] outside the table's validity.
    fn tai_minus_utc(&self, at: Instant<Utc>) -> Result<Duration>;
}

/// UTC → TAI.
///
/// # Errors
///
/// As [`LeapSeconds::tai_minus_utc`].
pub fn utc_to_tai(at: Instant<Utc>, table: &impl LeapSeconds) -> Result<Instant<Tai>> {
    let offset = table.tai_minus_utc(at)?;
    Ok(at.saturating_add(offset).relabel())
}

/// TAI → UTC.
///
/// The table is indexed by UTC (the unknown), so it is read at a first guess
/// and again at the result. Exact except during a leap second, where UTC is
/// ambiguous and the later reading is returned.
///
/// # Errors
///
/// As [`LeapSeconds::tai_minus_utc`].
pub fn tai_to_utc(at: Instant<Tai>, table: &impl LeapSeconds) -> Result<Instant<Utc>> {
    let guess: Instant<Utc> = at.relabel();
    let first = table.tai_minus_utc(guess)?;
    let utc = at.saturating_sub(first).relabel::<Utc>();
    let second = table.tai_minus_utc(utc)?;
    Ok(at.saturating_sub(second).relabel())
}

/// TAI → GPS: fixed 19 s.
#[must_use]
pub fn tai_to_gps(at: Instant<Tai>) -> Instant<Gps> {
    at.saturating_sub(TAI_MINUS_GPS).relabel()
}

/// GPS → TAI: fixed 19 s.
#[must_use]
pub fn gps_to_tai(at: Instant<Gps>) -> Instant<Tai> {
    at.saturating_add(TAI_MINUS_GPS).relabel()
}

/// UTC → GPS.
///
/// # Errors
///
/// As [`LeapSeconds::tai_minus_utc`].
pub fn utc_to_gps(at: Instant<Utc>, table: &impl LeapSeconds) -> Result<Instant<Gps>> {
    utc_to_tai(at, table).map(tai_to_gps)
}

/// GPS → UTC.
///
/// # Errors
///
/// As [`LeapSeconds::tai_minus_utc`].
pub fn gps_to_utc(at: Instant<Gps>, table: &impl LeapSeconds) -> Result<Instant<Utc>> {
    tai_to_utc(gps_to_tai(at), table)
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;
    use alloc::format;

    /// Table since 2017: 37 s, no expiry.
    struct SinceTwentySeventeen;

    impl LeapSeconds for SinceTwentySeventeen {
        fn tai_minus_utc(&self, _at: Instant<Utc>) -> Result<Duration> {
            Ok(Duration::from_secs(37))
        }
    }

    #[test]
    fn nanoseconds_split_correctly_either_side_of_the_epoch() {
        let after = Instant::<Utc>::from_unix_nanos(1_500_000_000);
        assert_eq!((after.seconds(), after.subsec_nanos()), (1, 500_000_000));
        let before = Instant::<Utc>::from_unix_nanos(-1);
        assert_eq!((before.seconds(), before.subsec_nanos()), (-1, 999_999_999));
        assert_eq!(after.checked_unix_nanos(), Some(1_500_000_000));
        assert_eq!(before.checked_unix_nanos(), Some(-1));
        assert_eq!(
            Instant::<Utc>::from_unix_seconds(i64::MAX).checked_unix_nanos(),
            None
        );
    }

    #[test]
    fn a_full_second_of_nanoseconds_is_rejected() {
        assert!(Instant::<Utc>::new(0, NANOS_PER_SECOND).is_err());
        assert!(Instant::<Utc>::new(0, NANOS_PER_SECOND - 1).is_ok());
    }

    #[test]
    fn arithmetic_carries_and_borrows() {
        let start = Instant::<Gps>::new(10, 900_000_000).unwrap();
        let later = start.checked_add(Duration::from_millis(250)).unwrap();
        assert_eq!((later.seconds(), later.subsec_nanos()), (11, 150_000_000));
        assert_eq!(later.checked_sub(Duration::from_millis(250)), Some(start));
        assert_eq!(
            later.duration_since(start).unwrap(),
            Duration::from_millis(250)
        );
        assert_eq!(
            start.duration_since(later),
            Err(KernelError::TimeReversed {
                by: Duration::from_millis(250)
            })
        );
    }

    #[test]
    fn the_ends_of_the_range_saturate_rather_than_wrap() {
        let end = Instant::<Tai>::from_unix_seconds(i64::MAX);
        assert_eq!(end.checked_add(Duration::from_secs(1)), None);
        assert!(end.saturating_add(Duration::from_secs(1)) >= end);
        let start = Instant::<Tai>::from_unix_seconds(i64::MIN);
        assert_eq!(start.checked_sub(Duration::from_nanos(1)), None);
        assert_eq!(start.saturating_sub(Duration::from_secs(1)), start);
        // The full range exceeds an `i64` second count but fits `Duration`
        // (`u64` seconds).
        assert_eq!(
            end.checked_duration_since(start),
            Some(Duration::from_secs(u64::MAX))
        );
    }

    #[test]
    fn the_calendar_round_trips_on_known_dates() {
        let cases = [
            (Civil::date(1970, 1, 1), 0),
            (Civil::date(2000, 3, 1), 951_868_800),
            (Civil::date(2017, 1, 1), 1_483_228_800),
            (Civil::date(1969, 12, 31), -86_400),
            (Civil::date(1600, 2, 29), -11_670_998_400),
            (Civil::date(2400, 2, 29), 13_574_563_200),
        ];
        for (civil, seconds) in cases {
            let instant = Instant::<Utc>::from_civil(civil).unwrap();
            assert_eq!(instant.seconds(), seconds, "{civil:?}");
            assert_eq!(instant.civil(), civil);
        }
        let reading = Civil {
            year: 2026,
            month: 9,
            day: 11,
            hour: 10,
            minute: 15,
            second: 30,
            nanos: 250_000_000,
        };
        let instant = Instant::<Utc>::from_civil(reading).unwrap();
        assert_eq!(instant.civil(), reading);
        assert_eq!(format!("{instant}"), "2026-09-11T10:15:30.250 UTC");
        assert_eq!(format!("{instant:.0}"), "2026-09-11T10:15:30 UTC");
        assert_eq!(format!("{instant:?}"), "2026-09-11T10:15:30.250000000 UTC");
    }

    #[test]
    fn impossible_dates_are_errors() {
        assert!(Instant::<Utc>::from_civil(Civil::date(2023, 2, 29)).is_err());
        assert!(Instant::<Utc>::from_civil(Civil::date(2024, 2, 29)).is_ok());
        assert!(Instant::<Utc>::from_civil(Civil::date(2024, 13, 1)).is_err());
        assert!(Instant::<Utc>::from_civil(Civil::date(2024, 4, 31)).is_err());
        let leap = Civil {
            second: 60,
            ..Civil::date(2016, 12, 31)
        };
        assert!(Instant::<Utc>::from_civil(leap).is_err());
    }

    #[test]
    fn scales_convert_through_the_table_and_back() {
        let utc = Instant::<Utc>::from_civil(Civil::date(2026, 9, 11)).unwrap();
        let tai = utc_to_tai(utc, &SinceTwentySeventeen).unwrap();
        assert_eq!(
            tai.duration_since(utc.relabel()).unwrap(),
            Duration::from_secs(37)
        );
        let gps = utc_to_gps(utc, &SinceTwentySeventeen).unwrap();
        assert_eq!(
            gps.duration_since(utc.relabel()).unwrap(),
            Duration::from_secs(18)
        );
        assert_eq!(tai_to_utc(tai, &SinceTwentySeventeen).unwrap(), utc);
        assert_eq!(gps_to_utc(gps, &SinceTwentySeventeen).unwrap(), utc);
        assert_eq!(format!("{gps}"), "2026-09-11T00:00:18.000 GPS");
    }

    #[test]
    fn a_stepping_table_is_read_at_the_answer_not_the_guess() {
        // One-step table: 36 s before 2017, 37 s after.
        struct Stepping;
        impl LeapSeconds for Stepping {
            fn tai_minus_utc(&self, at: Instant<Utc>) -> Result<Duration> {
                let step = Instant::<Utc>::from_civil(Civil::date(2017, 1, 1)).unwrap();
                Ok(Duration::from_secs(if at < step { 36 } else { 37 }))
            }
        }
        // 1 s before the step (UTC): 36 s applies, whichever side the initial
        // guess lands.
        let utc = Instant::<Utc>::from_civil(Civil::date(2017, 1, 1))
            .unwrap()
            .checked_sub(Duration::from_secs(1))
            .unwrap();
        let tai = utc_to_tai(utc, &Stepping).unwrap();
        assert_eq!(tai_to_utc(tai, &Stepping).unwrap(), utc);
    }

    #[test]
    fn an_expired_table_is_an_error_not_a_guess() {
        struct Expired;
        impl LeapSeconds for Expired {
            fn tai_minus_utc(&self, _at: Instant<Utc>) -> Result<Duration> {
                Err(KernelError::OutsideValidity {
                    data: "leap second table",
                })
            }
        }
        assert!(utc_to_gps(Instant::UNIX_EPOCH, &Expired).is_err());
    }

    #[cfg(feature = "serde")]
    #[test]
    fn serde_round_trips_and_validates() {
        let instant = Instant::<Utc>::new(1_700_000_000, 5).unwrap();
        let json = serde_json::to_string(&instant).unwrap();
        assert_eq!(json, r#"{"seconds":1700000000,"nanos":5}"#);
        assert_eq!(
            serde_json::from_str::<Instant<Utc>>(&json).unwrap(),
            instant
        );
        assert!(
            serde_json::from_str::<Instant<Utc>>(r#"{"seconds":0,"nanos":1000000000}"#).is_err()
        );
    }
}
