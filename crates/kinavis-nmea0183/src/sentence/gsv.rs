//! `GSV` — satellites in view, and the assembler of its multi-line messages.
//!
//! ```text
//! $GPGSV,n,i,t,id,el,az,snr,…[,g]*hh
//! ```
//!
//! Line `i` of `n` of one talker's report of `t` satellites in view, up to four
//! per line: number, elevation, azimuth and signal-to-noise ratio, each
//! possibly empty. Since NMEA 4.10 a signal identifier (`g`) follows; some
//! receivers add a system identifier after it.
//!
//! A receiver sends one report per talker (`GP`, `GL`, `GA`, `GB`, …) each
//! epoch. [`GsvAssembler`] collects the lines of each, in order, into one
//! [`SatelliteSet`].

use core::fmt::{self, Write};

use kinavis_kernel::inline::Inline;
use kinavis_kernel::{Angle, KernelError, SatelliteSet, SatelliteView, TrueBearing};

use crate::encode;
use crate::error::NmeaError;
use crate::field::Field;
use crate::frame::Frame;
use crate::sentence::Talker;

/// Most satellites on one `GSV` line.
pub const MAX_GSV_SATELLITES: usize = 4;

/// One satellite as a `GSV` line gives it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct GsvSatellite {
    /// Number as written.
    pub number: u16,
    /// Elevation, degrees, −90 to 90.
    pub elevation: Option<i8>,
    /// True azimuth, degrees, 0 to 359.
    pub azimuth: Option<u16>,
    /// Signal-to-noise ratio, dB-Hz, 0 to 99.
    pub snr: Option<u8>,
}

/// One line of a satellites-in-view report.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Gsv {
    /// Talker.
    pub talker: Talker,
    /// Lines in the report.
    pub lines: u8,
    /// This line's number, from 1.
    pub line: u8,
    /// Satellites in view, the whole report.
    pub in_view: u8,
    /// Satellites on this line.
    pub satellites: Inline<GsvSatellite, MAX_GSV_SATELLITES>,
    /// NMEA 4.10 signal identifier.
    pub signal: Option<u8>,
    /// System identifier written after the signal by some receivers.
    pub system: Option<u8>,
}

impl Gsv {
    /// Minimum field count: lines, line number and satellites in view.
    const REQUIRED: usize = 3;

    pub(crate) fn decode(talker: Talker, frame: Frame<'_>) -> Result<Self, NmeaError> {
        let found = frame.fields().remaining();
        if found < Self::REQUIRED {
            return Err(NmeaError::TooFewFields {
                found,
                required: Self::REQUIRED,
            });
        }
        let groups = (found - Self::REQUIRED) / 4;
        let extra = (found - Self::REQUIRED) % 4;
        let mut fields = frame.fields().indexed();
        let mut next = || {
            fields.next().unwrap_or(Field {
                bytes: &[],
                index: usize::MAX,
            })
        };
        let lines = next().unsigned::<u8>("number of lines")?;
        let line_field = next();
        let line = line_field.unsigned::<u8>("line number")?;
        if line == 0 || line > lines {
            return Err(NmeaError::BadField {
                index: line_field.index,
                expected: "line number within the number of lines",
            });
        }
        let in_view = next().unsigned::<u8>("satellites in view")?;
        let mut satellites = Inline::new(GsvSatellite {
            number: 0,
            elevation: None,
            azimuth: None,
            snr: None,
        });
        for _ in 0..groups {
            let number = next();
            let elevation = next().optional_bounded_integer("elevation", (-90, 90))?;
            let azimuth = next().optional_bounded_integer("azimuth", (0, 360))?;
            let snr = next().optional_bounded_integer("signal-to-noise ratio", (0, 99))?;
            // A group with no number is padding.
            let Some(number) = number.optional_unsigned::<u16>("satellite number")? else {
                continue;
            };
            let satellite = GsvSatellite {
                number,
                elevation: elevation.and_then(|e| i8::try_from(e).ok()),
                azimuth: azimuth.and_then(|a| u16::try_from(a % 360).ok()),
                snr: snr.and_then(|s| u8::try_from(s).ok()),
            };
            satellites
                .push(satellite)
                .map_err(|_| NmeaError::BadField {
                    index: number_index(found),
                    expected: "at most four satellites per line",
                })?;
        }
        let (signal, system) = match extra {
            0 => (None, None),
            1 => (next().optional_hex_digit("signal identifier")?, None),
            2 => (
                next().optional_hex_digit("signal identifier")?,
                next().optional_hex_digit("system identifier")?,
            ),
            _ => {
                return Err(NmeaError::BadField {
                    index: found.saturating_sub(1),
                    expected: "satellites in groups of four fields",
                })
            }
        };
        Ok(Self {
            talker,
            lines,
            line,
            in_view,
            satellites,
            signal,
            system,
        })
    }

    /// The satellites of this line as kernel views, not marked as used in the
    /// fix; satellites this crate cannot name are left out.
    pub fn views(&self) -> impl Iterator<Item = SatelliteView> + '_ {
        self.satellites.iter().filter_map(|satellite| {
            let id =
                crate::sentence::satellites::satellite(self.talker, self.system, satellite.number)?;
            let elevation = satellite
                .elevation
                .and_then(|e| Angle::from_degrees(f64::from(e)).ok());
            let azimuth = satellite
                .azimuth
                .and_then(|a| TrueBearing::new(f64::from(a)).ok());
            let mut view = SatelliteView::new(id, elevation, azimuth).ok()?;
            if let Some(snr) = satellite.snr {
                view = view.with_carrier_to_noise(f64::from(snr)).ok()?;
            }
            Some(view)
        })
    }
}

/// Field index reported when a line holds too many satellites: the last one.
fn number_index(found: usize) -> usize {
    found.saturating_sub(1)
}

impl fmt::Display for Gsv {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        encode::sentence(f, |out| {
            write!(
                out,
                "{}GSV,{},{},{:02}",
                self.talker, self.lines, self.line, self.in_view
            )?;
            for satellite in self.satellites.iter() {
                write!(out, ",{:02},", satellite.number)?;
                encode::optional(out, satellite.elevation.map(|e| TwoDigits(i32::from(e))))?;
                out.write_char(',')?;
                encode::optional(out, satellite.azimuth.map(ThreeDigits))?;
                out.write_char(',')?;
                encode::optional(out, satellite.snr.map(|s| TwoDigits(i32::from(s))))?;
            }
            if let Some(signal) = self.signal {
                write!(out, ",{signal:X}")?;
                if let Some(system) = self.system {
                    write!(out, ",{system:X}")?;
                }
            }
            Ok(())
        })
    }
}

struct TwoDigits(i32);

impl fmt::Display for TwoDigits {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:02}", self.0)
    }
}

struct ThreeDigits(u16);

impl fmt::Display for ThreeDigits {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:03}", self.0)
    }
}

/// Talkers whose reports one assembler follows at once.
pub const MAX_GSV_TALKERS: usize = 8;

/// Why a `GSV` line could not be assembled.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, PartialEq)]
pub enum GsvError {
    /// A line came out of order: one was lost or the report restarted. The
    /// talker's report is dropped; its next line 1 starts a new one.
    OutOfOrder {
        /// Talker.
        talker: Talker,
        /// Line expected next.
        expected: u8,
        /// Line received.
        found: u8,
    },
    /// More talkers than [`MAX_GSV_TALKERS`] in one epoch.
    TooManyTalkers,
    /// The satellite set is full.
    Full(KernelError),
}

impl fmt::Display for GsvError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::OutOfOrder {
                talker,
                expected,
                found,
            } => write!(f, "{talker}GSV line {found} where line {expected} was due"),
            Self::TooManyTalkers => {
                write!(f, "more than {MAX_GSV_TALKERS} talkers report satellites")
            }
            Self::Full(error) => write!(f, "satellites in view: {error}"),
        }
    }
}

impl core::error::Error for GsvError {}

/// Progress of a talker's report after a line.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GsvProgress {
    /// More lines of this talker's report are due.
    Partial,
    /// The talker's report is complete.
    Complete(Talker),
}

/// Collects `GSV` lines of every talker into one set of satellites in view.
///
/// Lines of each talker must arrive in order from 1; a gap drops that
/// talker's report and is reported. The set accumulates across talkers until
/// [`GsvAssembler::clear`], which the caller calls once per epoch (for
/// example on each `GGA`).
#[derive(Clone)]
pub struct GsvAssembler<const N: usize = { kinavis_kernel::MAX_SATELLITES }> {
    talkers: Inline<(Talker, u8, u8), MAX_GSV_TALKERS>,
    satellites: SatelliteSet<N>,
}

impl<const N: usize> GsvAssembler<N> {
    /// Empty assembler.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            talkers: Inline::new((Talker::GP, 0, 0)),
            satellites: SatelliteSet::new(),
        }
    }

    /// Takes one line.
    ///
    /// # Errors
    ///
    /// [`GsvError`]: a line out of order, too many talkers, a full set.
    pub fn push(&mut self, gsv: &Gsv) -> Result<GsvProgress, GsvError> {
        let slot = self
            .talkers
            .iter()
            .position(|(talker, _, _)| *talker == gsv.talker);
        let expected = slot
            .and_then(|index| self.talkers.get(index))
            .map_or(1, |(_, next, _)| *next);
        if gsv.line != 1 && gsv.line != expected {
            if let Some(index) = slot {
                let _ = self.talkers.remove(index);
            }
            return Err(GsvError::OutOfOrder {
                talker: gsv.talker,
                expected,
                found: gsv.line,
            });
        }
        let state = (gsv.talker, gsv.line.saturating_add(1), gsv.lines);
        match slot.and_then(|index| self.talkers.as_mut_slice().get_mut(index)) {
            Some(entry) => *entry = state,
            None => self
                .talkers
                .push(state)
                .map_err(|_| GsvError::TooManyTalkers)?,
        }
        for view in gsv.views() {
            self.satellites.insert(view).map_err(GsvError::Full)?;
        }
        if gsv.line == gsv.lines {
            if let Some(index) = self
                .talkers
                .iter()
                .position(|(talker, _, _)| *talker == gsv.talker)
            {
                let _ = self.talkers.remove(index);
            }
            Ok(GsvProgress::Complete(gsv.talker))
        } else {
            Ok(GsvProgress::Partial)
        }
    }

    /// Satellites collected since the last [`GsvAssembler::clear`].
    #[must_use]
    pub const fn satellites(&self) -> &SatelliteSet<N> {
        &self.satellites
    }

    /// Mutable access, to mark satellites used ([`crate::Gsa`]) or add
    /// residuals ([`crate::Grs::apply`]).
    pub fn satellites_mut(&mut self) -> &mut SatelliteSet<N> {
        &mut self.satellites
    }

    /// Starts a new epoch: forgets the satellites and any partial reports.
    pub fn clear(&mut self) {
        *self = Self::new();
    }
}

impl<const N: usize> Default for GsvAssembler<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> fmt::Debug for GsvAssembler<N> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("GsvAssembler")
            .field("talkers", &self.talkers)
            .field("satellites", &self.satellites)
            .finish()
    }
}
