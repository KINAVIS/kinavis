//! `VTG` — course over ground and ground speed.
//!
//! ```text
//! $GPVTG,cog,T,cogm,M,sog,N,sogk,K[,mode]*hh
//! ```

use core::fmt::{self, Write};

use kinavis_kernel::{MagneticCourse, Speed, TrueCourse};

use crate::encode;
use crate::error::NmeaError;
use crate::field::Field;
use crate::frame::Frame;
use crate::sentence::gga::OneDecimal;
use crate::sentence::{Mode, Talker};

/// Course and speed over the ground.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vtg {
    /// Talker.
    pub talker: Talker,
    /// Course over the ground, true.
    pub course_true: Option<TrueCourse>,
    /// Course over the ground, magnetic.
    pub course_magnetic: Option<MagneticCourse>,
    /// Speed over ground, knots field.
    pub speed_knots: Option<Speed>,
    /// Speed over ground, km/h field.
    ///
    /// Redundant with `speed_knots`; kept as received so the sentence
    /// re-encodes unchanged.
    pub speed_kmh: Option<Speed>,
    /// Mode indicator; absent before NMEA 2.3.
    pub mode: Option<Mode>,
}

impl Vtg {
    /// Minimum field count: through the km/h unit.
    const REQUIRED: usize = 8;

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

        let course_true = next().optional_course()?;
        next().expect_letter(b'T', "true course marker")?;
        let course_magnetic = next().optional_magnetic_course()?;
        next().expect_letter(b'M', "magnetic course marker")?;
        let speed_knots = next().optional_speed_knots()?;
        next().expect_letter(b'N', "knots marker")?;
        let speed_kmh = next().optional_speed_kmh()?;
        next().expect_letter(b'K', "kilometres per hour marker")?;
        let mode = next().optional_mode()?;

        Ok(Self {
            talker,
            course_true,
            course_magnetic,
            speed_knots,
            speed_kmh,
            mode,
        })
    }

    /// Speed over ground from whichever field is present.
    #[must_use]
    pub fn speed_over_ground(&self) -> Option<Speed> {
        self.speed_knots.or(self.speed_kmh)
    }
}

impl fmt::Display for Vtg {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        encode::sentence(f, |out| {
            write!(out, "{}VTG,", self.talker)?;
            encode::optional(out, self.course_true.map(|c| OneDecimal(c.degrees())))?;
            out.write_str(",T,")?;
            encode::optional(out, self.course_magnetic.map(|c| OneDecimal(c.degrees())))?;
            out.write_str(",M,")?;
            encode::optional(out, self.speed_knots.map(|s| OneDecimal(s.knots())))?;
            out.write_str(",N,")?;
            encode::optional(
                out,
                self.speed_kmh.map(|s| OneDecimal(s.kilometres_per_hour())),
            )?;
            out.write_str(",K")?;
            if let Some(mode) = self.mode {
                write!(out, ",{mode}")?;
            }
            Ok(())
        })
    }
}
