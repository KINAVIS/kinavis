//! `GGA` — global positioning system fix data.
//!
//! ```text
//! $GPGGA,hhmmss.ss,ddmm.mmmm,N,dddmm.mmmm,E,q,sats,hdop,alt,M,geoid,M,age,station*hh
//! ```
//!
//! Position with fix quality, satellites in use and HDOP. No date: conversion
//! to [`GnssFix`] needs one supplied — [`Gga::fix_on`].

use core::fmt::{self, Write};
use core::time::Duration;

use kinavis_kernel::gnss::{Dop, FixType, GnssFix};
use kinavis_kernel::{Distance, Position};

use crate::encode;
use crate::error::{NmeaError, TranslationError};
use crate::field::{bounds, Field};
use crate::frame::Frame;
use crate::sentence::{Date, Talker, TimeOfDay};

/// Fix data: position, time, quality, satellites, HDOP, altitude.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Gga {
    /// Talker.
    pub talker: Talker,
    /// Time of the fix, UTC.
    pub time: Option<TimeOfDay>,
    /// Position, if available.
    pub position: Option<Position>,
    /// Fix quality.
    pub fix_type: FixType,
    /// Satellites in use.
    pub satellites: Option<u8>,
    /// Horizontal dilution of precision.
    pub hdop: Option<Dop>,
    /// Antenna altitude above mean sea level.
    pub altitude: Option<Distance>,
    /// Geoid separation: mean sea level above the WGS-84 ellipsoid.
    pub geoid_separation: Option<Distance>,
    /// Age of the differential correction.
    pub differential_age: Option<Duration>,
    /// Identifier of the differential reference station.
    pub station: Option<u16>,
}

impl Gga {
    /// Minimum field count: through fix quality.
    const REQUIRED: usize = 6;

    pub(crate) fn decode(talker: Talker, frame: Frame<'_>) -> Result<Self, NmeaError> {
        let found = frame.fields().remaining();
        if found < Self::REQUIRED {
            return Err(NmeaError::TooFewFields {
                found,
                required: Self::REQUIRED,
            });
        }
        let mut fields = frame.fields().indexed();
        let mut next = || {
            fields.next().unwrap_or(Field {
                bytes: &[],
                index: usize::MAX,
            })
        };

        let time = next().optional_time_of_day()?;
        let position = Field::position(next(), next(), next(), next())?;
        let quality = next();
        let fix_type =
            fix_type_from_quality(quality.fix_quality()?).ok_or(NmeaError::BadField {
                index: quality.index,
                expected: "fix quality",
            })?;
        let satellites = next().optional_unsigned("satellite count")?;
        let hdop = next().optional_dop()?;
        let altitude = next().optional_metres("altitude", bounds::ALTITUDE_METRES)?;
        next().expect_letter(b'M', "altitude unit")?;
        let geoid_separation =
            next().optional_metres("geoid separation", bounds::GEOID_SEPARATION_METRES)?;
        next().expect_letter(b'M', "geoid separation unit")?;
        let age = next();
        let differential_age = if age.is_empty() {
            None
        } else {
            let seconds =
                age.bounded_decimal("differential age", bounds::DIFFERENTIAL_AGE_SECONDS)?;
            Some(
                Duration::try_from_secs_f64(seconds).map_err(|_| NmeaError::BadField {
                    index: age.index,
                    expected: "differential age",
                })?,
            )
        };
        let station = next().optional_unsigned("station identifier")?;

        Ok(Self {
            talker,
            time,
            position,
            fix_type,
            satellites,
            hdop,
            altitude,
            geoid_separation,
            differential_age,
            station,
        })
    }

    /// Fix from the sentence on a supplied date.
    ///
    /// # Errors
    ///
    /// [`TranslationError::NoPosition`] or [`TranslationError::NoTime`] if
    /// missing; [`TranslationError::BadMoment`] if date and time do not form a
    /// valid instant.
    pub fn fix_on(&self, date: Date) -> Result<GnssFix, TranslationError> {
        let position = self.position.ok_or(TranslationError::NoPosition)?;
        let time = self.time.ok_or(TranslationError::NoTime)?;
        let mut fix = GnssFix::builder(date.at(time)?, position).fix_type(self.fix_type);
        if let Some(satellites) = self.satellites {
            fix = fix.satellites(satellites);
        }
        if let Some(hdop) = self.hdop {
            fix = fix.hdop(hdop);
        }
        Ok(fix.build())
    }
}

/// GGA quality digit as a fix type.
const fn fix_type_from_quality(digit: u8) -> Option<FixType> {
    Some(match digit {
        0 => FixType::None,
        1 => FixType::Autonomous,
        2 => FixType::Differential,
        3 => FixType::Precise,
        4 => FixType::RtkFixed,
        5 => FixType::RtkFloat,
        6 => FixType::Estimated,
        7 => FixType::Manual,
        8 => FixType::Simulated,
        _ => return None,
    })
}

/// GGA quality digit for a fix type.
const fn quality_from_fix_type(fix_type: FixType) -> u8 {
    match fix_type {
        FixType::Autonomous => 1,
        FixType::Differential => 2,
        FixType::Precise => 3,
        FixType::RtkFixed => 4,
        FixType::RtkFloat => 5,
        FixType::Estimated => 6,
        FixType::Manual => 7,
        FixType::Simulated => 8,
        // No fix, or a fix type without a GGA digit: encoded as no fix, which
        // never overstates quality.
        FixType::None | _ => 0,
    }
}

impl fmt::Display for Gga {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        encode::sentence(f, |out| {
            write!(out, "{}GGA,", self.talker)?;
            encode::optional(out, self.time)?;
            out.write_str(",")?;
            encode::position(out, self.position)?;
            write!(out, ",{},", quality_from_fix_type(self.fix_type))?;
            encode::optional(out, self.satellites.map(TwoDigits))?;
            out.write_str(",")?;
            encode::optional(out, self.hdop.map(|d| DopField(d.value())))?;
            out.write_str(",")?;
            encode::optional(out, self.altitude.map(|a| OneDecimal(a.metres())))?;
            out.write_str(",M,")?;
            encode::optional(out, self.geoid_separation.map(|g| OneDecimal(g.metres())))?;
            out.write_str(",M,")?;
            encode::optional(
                out,
                self.differential_age.map(|a| OneDecimal(a.as_secs_f64())),
            )?;
            out.write_str(",")?;
            encode::optional(out, self.station.map(FourDigits))
        })
    }
}

struct TwoDigits(u8);

impl fmt::Display for TwoDigits {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:02}", self.0)
    }
}

struct FourDigits(u16);

impl fmt::Display for FourDigits {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:04}", self.0)
    }
}

/// DOP with one decimal. A DOP is strictly positive, so values that would
/// round to `0.0` are written as `0.1`, the smallest positive value at this
/// precision; the written sentence always parses.
struct DopField(f64);

impl fmt::Display for DopField {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.0 < 0.05 {
            f.write_str("0.1")
        } else {
            write!(f, "{:.1}", self.0)
        }
    }
}

pub(crate) struct OneDecimal(pub(crate) f64);

impl fmt::Display for OneDecimal {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:.1}", self.0)
    }
}
