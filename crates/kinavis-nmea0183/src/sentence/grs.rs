//! `GRS` — range residuals.
//!
//! ```text
//! $GPGRS,hhmmss.ss,m,r1,…,r12[,s,g]*hh
//! ```
//!
//! The residual of each satellite's range after the fix, in metres, in the
//! order of the satellites of the matching `GSA`. Mode `0`: the residuals of
//! the position in the `GGA` of the same time; `1`: recomputed after it. Since
//! NMEA 4.10, system (`s`) and signal (`g`) identifiers follow.

use core::fmt::{self, Write};

use kinavis_kernel::{Distance, SatelliteSet, SatelliteView};

use crate::encode;
use crate::error::NmeaError;
use crate::field::{bounds, Field};
use crate::frame::Frame;
use crate::sentence::{Gsa, Talker, TimeOfDay};

/// Residual fields of a `GRS`.
const SLOTS: usize = 12;

/// Range residuals.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Grs {
    /// Talker.
    pub talker: Talker,
    /// Time of the fix, UTC.
    pub time: Option<TimeOfDay>,
    /// `true` for mode 1: residuals recomputed after the `GGA` position.
    pub recomputed: bool,
    /// Residuals in `GSA` order; `None` for an empty field.
    pub residuals: [Option<Distance>; SLOTS],
    /// NMEA 4.10 system identifier.
    pub system: Option<u8>,
    /// NMEA 4.10 signal identifier.
    pub signal: Option<u8>,
}

impl Grs {
    /// Minimum field count: time, mode and one residual.
    const REQUIRED: usize = 3;

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
        let mode = next();
        let recomputed = match mode.bytes {
            b"0" => false,
            b"1" => true,
            _ => {
                return Err(NmeaError::BadField {
                    index: mode.index,
                    expected: "residual mode",
                })
            }
        };
        let mut residuals = [None; SLOTS];
        for residual in &mut residuals {
            *residual = next().optional_metres("range residual", bounds::RESIDUAL_METRES)?;
        }
        let system = next().optional_hex_digit("system identifier")?;
        let signal = next().optional_hex_digit("signal identifier")?;
        Ok(Self {
            talker,
            time,
            recomputed,
            residuals,
            system,
            signal,
        })
    }

    /// Puts each residual on its satellite: the `GSA`'s slots in order, paired
    /// with the residuals. A satellite not yet in `set` is added as
    /// used in the fix, without sky position.
    ///
    /// # Errors
    ///
    /// [`kinavis_kernel::KernelError::CapacityExceeded`] if `set` is full.
    pub fn apply<const N: usize>(
        &self,
        gsa: &Gsa,
        set: &mut SatelliteSet<N>,
    ) -> Result<(), kinavis_kernel::KernelError> {
        for (id, residual) in gsa.slot_satellites().zip(self.residuals) {
            let (Some(id), Some(residual)) = (id, residual) else {
                continue;
            };
            let view = match set.get(id) {
                Some(view) => *view,
                None => SatelliteView::new(id, None, None)?.used_in_fix(true),
            };
            set.insert(view.with_residual(residual))?;
        }
        Ok(())
    }
}

impl fmt::Display for Grs {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        encode::sentence(f, |out| {
            write!(out, "{}GRS,", self.talker)?;
            encode::optional(out, self.time)?;
            write!(out, ",{}", u8::from(self.recomputed))?;
            for residual in self.residuals {
                out.write_char(',')?;
                encode::optional(out, residual.map(|r| OneDecimal(r.metres())))?;
            }
            if self.system.is_some() || self.signal.is_some() {
                out.write_char(',')?;
                encode::optional(out, self.system.map(Hex))?;
                out.write_char(',')?;
                encode::optional(out, self.signal.map(Hex))?;
            }
            Ok(())
        })
    }
}

struct OneDecimal(f64);

impl fmt::Display for OneDecimal {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:.1}", self.0)
    }
}

struct Hex(u8);

impl fmt::Display for Hex {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:X}", self.0)
    }
}
