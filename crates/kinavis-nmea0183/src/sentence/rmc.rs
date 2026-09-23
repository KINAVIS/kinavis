//! `RMC` — recommended minimum specific GNSS data.
//!
//! ```text
//! $GPRMC,hhmmss.ss,A,ddmm.mmmm,N,dddmm.mmmm,E,sog,cog,ddmmyy,var,E[,mode[,nav]]*hh
//! ```
//!
//! The only sentence with position, time *and* date, hence the only one that
//! converts to a [`GnssFix`] on its own.

use core::fmt::{self, Write};

use kinavis_kernel::gnss::{FixType, GnssFix};
use kinavis_kernel::{Position, Speed, TrueCourse, Variation};

use crate::encode;
use crate::error::{NmeaError, TranslationError};
use crate::field::Field;
use crate::frame::Frame;
use crate::sentence::{Date, Mode, Status, Talker, TimeOfDay};

/// Recommended minimum: position, velocity, time and date.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Rmc {
    /// Talker.
    pub talker: Talker,
    /// Time of the fix, UTC.
    pub time: Option<TimeOfDay>,
    /// `A`/`V`: data valid or warning.
    pub status: Status,
    /// Position, if available.
    pub position: Option<Position>,
    /// Speed over the ground.
    pub speed_over_ground: Option<Speed>,
    /// Course over the ground, true.
    pub course_over_ground: Option<TrueCourse>,
    /// Date of the fix, UTC.
    pub date: Option<Date>,
    /// Magnetic variation, east positive.
    pub variation: Option<Variation>,
    /// Mode indicator; absent before NMEA 2.3.
    pub mode: Option<Mode>,
}

impl Rmc {
    /// Minimum field count: through the variation `E`/`W`.
    const REQUIRED: usize = 11;

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
        let status = next().status()?;
        let position = Field::position(next(), next(), next(), next())?;
        let speed_over_ground = next().optional_speed_knots()?;
        let course_over_ground = next().optional_course()?;
        let date = next().optional_date()?;
        let variation = next().optional_variation(next())?;
        let mode = next().optional_mode()?;

        Ok(Self {
            talker,
            time,
            status,
            position,
            speed_over_ground,
            course_over_ground,
            date,
            variation,
            mode,
        })
    }

    /// Fix type: from the mode indicator if present, else `Autonomous` for
    /// status `A` and `None` for `V`.
    #[must_use]
    pub fn fix_type(&self) -> FixType {
        match (self.mode, self.status) {
            (Some(mode), _) => mode.fix_type(),
            (None, Status::Valid) => FixType::Autonomous,
            (None, _) => FixType::None,
        }
    }
}

impl TryFrom<Rmc> for GnssFix {
    type Error = TranslationError;

    /// Fix from position, date and time.
    ///
    /// Status `V` or mode `N` still yields a fix, of type [`FixType::None`];
    /// rejecting it is the use case's decision.
    fn try_from(rmc: Rmc) -> Result<Self, TranslationError> {
        let position = rmc.position.ok_or(TranslationError::NoPosition)?;
        let time = rmc.time.ok_or(TranslationError::NoTime)?;
        let date = rmc.date.ok_or(TranslationError::NoDate)?;
        let mut fix = GnssFix::builder(date.at(time)?, position).fix_type(rmc.fix_type());
        if let Some(course) = rmc.course_over_ground {
            fix = fix.course_over_ground(course);
        }
        if let Some(speed) = rmc.speed_over_ground {
            fix = fix.speed_over_ground(speed);
        }
        Ok(fix.build())
    }
}

impl fmt::Display for Rmc {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        encode::sentence(f, |out| {
            write!(out, "{}RMC,", self.talker)?;
            encode::optional(out, self.time)?;
            write!(out, ",{},", self.status)?;
            encode::position(out, self.position)?;
            out.write_str(",")?;
            encode::optional(out, self.speed_over_ground.map(Knots))?;
            out.write_str(",")?;
            encode::optional(out, self.course_over_ground.map(|c| Degrees(c.degrees())))?;
            out.write_str(",")?;
            encode::optional(out, self.date)?;
            out.write_str(",")?;
            match self.variation {
                Some(variation) => {
                    let side = if variation.degrees() < 0.0 { 'W' } else { 'E' };
                    write!(out, "{},{side}", Degrees(variation.degrees().abs()))?;
                }
                None => out.write_str(",")?,
            }
            if let Some(mode) = self.mode {
                write!(out, ",{mode}")?;
            }
            Ok(())
        })
    }
}

/// Speed as knots, one decimal.
pub(crate) struct Knots(pub(crate) Speed);

impl fmt::Display for Knots {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:.1}", self.0.knots())
    }
}

/// Angle, one decimal.
pub(crate) struct Degrees(pub(crate) f64);

impl fmt::Display for Degrees {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:.1}", self.0)
    }
}
