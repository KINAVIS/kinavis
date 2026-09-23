//! `GLL` — geographic position, latitude and longitude.
//!
//! ```text
//! $GPGLL,ddmm.mmmm,N,dddmm.mmmm,E,hhmmss.ss,A[,mode]*hh
//! ```

use core::fmt::{self, Write};

use kinavis_kernel::Position;

use crate::encode;
use crate::error::NmeaError;
use crate::field::Field;
use crate::frame::Frame;
use crate::sentence::{Mode, Status, Talker, TimeOfDay};

/// Geographic position: latitude, longitude, time.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Gll {
    /// Talker.
    pub talker: Talker,
    /// Position, if available.
    pub position: Option<Position>,
    /// Time of the position, UTC.
    pub time: Option<TimeOfDay>,
    /// `A`/`V`: data valid or warning.
    pub status: Status,
    /// Mode indicator; absent before NMEA 2.3.
    pub mode: Option<Mode>,
}

impl Gll {
    /// Minimum field count: through status.
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

        let position = Field::position(next(), next(), next(), next())?;
        let time = next().optional_time_of_day()?;
        let status = next().status()?;
        let mode = next().optional_mode()?;

        Ok(Self {
            talker,
            position,
            time,
            status,
            mode,
        })
    }
}

impl fmt::Display for Gll {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        encode::sentence(f, |out| {
            write!(out, "{}GLL,", self.talker)?;
            encode::position(out, self.position)?;
            out.write_str(",")?;
            encode::optional(out, self.time)?;
            write!(out, ",{}", self.status)?;
            if let Some(mode) = self.mode {
                write!(out, ",{mode}")?;
            }
            Ok(())
        })
    }
}
