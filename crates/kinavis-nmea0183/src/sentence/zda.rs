//! `ZDA` — time and date.
//!
//! ```text
//! $GPZDA,hhmmss.ss,dd,mm,yyyy,zh,zm*hh
//! ```
//!
//! The date a `GGA` lacks, with the four-digit year, and the local zone.

use core::fmt::{self, Write};

use kinavis_kernel::time::{Instant, Utc};

use crate::encode;
use crate::error::{NmeaError, TranslationError};
use crate::field::Field;
use crate::frame::Frame;
use crate::sentence::{Date, Talker, TimeOfDay};

/// Local time zone of a `ZDA`: hours and minutes to add to UTC.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct LocalZone {
    /// Hours, −13 to 13.
    pub hours: i8,
    /// Minutes, 0 to 59, with the sign of the hours.
    pub minutes: u8,
}

/// Time and date, UTC.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Zda {
    /// Talker.
    pub talker: Talker,
    /// Time of day, UTC.
    pub time: Option<TimeOfDay>,
    /// Date, UTC.
    pub date: Option<Date>,
    /// Local zone, if reported.
    pub local_zone: Option<LocalZone>,
}

impl Zda {
    /// Minimum field count: through the year.
    const REQUIRED: usize = 4;

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
        let day = next().optional_bounded_integer("day", (1, 31))?;
        let month = next().optional_bounded_integer("month", (1, 12))?;
        let year = next().optional_bounded_integer("year", (1980, 9999))?;
        let date = match (day, month, year) {
            (Some(day), Some(month), Some(year)) => Some(Date {
                year,
                month: u8::try_from(month).unwrap_or(1),
                day: u8::try_from(day).unwrap_or(1),
            }),
            _ => None,
        };
        let hours = next().optional_bounded_integer("local zone hours", (-13, 13))?;
        let minutes = next().optional_bounded_integer("local zone minutes", (0, 59))?;
        let local_zone = hours.map(|hours| LocalZone {
            hours: i8::try_from(hours).unwrap_or(0),
            minutes: minutes.and_then(|m| u8::try_from(m).ok()).unwrap_or(0),
        });
        Ok(Self {
            talker,
            time,
            date,
            local_zone,
        })
    }

    /// The UTC instant of the sentence.
    ///
    /// # Errors
    ///
    /// [`TranslationError::NoTime`] without time or date;
    /// [`TranslationError::BadMoment`] if they do not form a valid instant.
    pub fn instant(&self) -> Result<Instant<Utc>, TranslationError> {
        let time = self.time.ok_or(TranslationError::NoTime)?;
        let date = self.date.ok_or(TranslationError::NoTime)?;
        date.at(time)
    }
}

impl fmt::Display for Zda {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        encode::sentence(f, |out| {
            write!(out, "{}ZDA,", self.talker)?;
            encode::optional(out, self.time)?;
            match self.date {
                Some(date) => write!(out, ",{:02},{:02},{:04}", date.day, date.month, date.year)?,
                None => out.write_str(",,,")?,
            }
            match self.local_zone {
                Some(zone) => write!(out, ",{:02},{:02}", zone.hours, zone.minutes),
                None => out.write_str(",,"),
            }
        })
    }
}
