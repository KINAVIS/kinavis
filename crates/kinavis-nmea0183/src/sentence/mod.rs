//! Supported sentences and their shared field types.

use core::fmt;

use kinavis_kernel::gnss::FixType;
use kinavis_kernel::time::{Instant, Utc};

use crate::error::{NmeaError, TranslationError};
use crate::frame::Frame;

mod gga;
mod gll;
mod rmc;
mod vdm;
mod vtg;

pub use gga::Gga;
pub use gll::Gll;
pub use rmc::Rmc;
pub use vdm::{Channel, Payload, Vdm, MAX_FRAGMENTS, MAX_PAYLOAD_CHARS};
pub use vtg::Vtg;

/// Two-letter talker identifier: `GP` GPS, `GN` multi-constellation, `GL`
/// GLONASS, etc.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Talker([u8; 2]);

impl Talker {
    /// GPS receiver.
    pub const GP: Self = Self(*b"GP");
    /// Multi-constellation receiver.
    pub const GN: Self = Self(*b"GN");

    /// Talker from two characters.
    ///
    /// # Errors
    ///
    /// [`NmeaError::BadHeader`] unless both are upper-case letters or digits.
    pub fn new(letters: [u8; 2]) -> Result<Self, NmeaError> {
        if letters
            .iter()
            .all(|byte| byte.is_ascii_uppercase() || byte.is_ascii_digit())
        {
            Ok(Self(letters))
        } else {
            Err(NmeaError::BadHeader)
        }
    }

    /// The two characters.
    #[must_use]
    pub fn as_str(&self) -> &str {
        core::str::from_utf8(&self.0).unwrap_or("??")
    }
}

impl fmt::Display for Talker {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

/// UTC time of day as carried by a sentence, at the talker's resolution.
///
/// Not an instant: that requires a date, which only some sentences carry.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct TimeOfDay {
    /// Hour, `0..=23`.
    pub hour: u8,
    /// Minute, `0..=59`.
    pub minute: u8,
    /// Second, `0..=60` (a leap second may be reported).
    pub second: u8,
    /// Nanoseconds.
    pub nanos: u32,
}

impl fmt::Display for TimeOfDay {
    /// `hhmmss.ss` wire form.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{:02}{:02}{:02}.{:02}",
            self.hour,
            self.minute,
            self.second,
            self.nanos / 10_000_000
        )
    }
}

/// Date as carried by a sentence.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Date {
    /// Year; `ddmmyy` is read as 20yy.
    pub year: i32,
    /// Month, `1..=12`.
    pub month: u8,
    /// Day of the month.
    pub day: u8,
}

impl Date {
    /// UTC instant from date and time of day.
    ///
    /// # Errors
    ///
    /// [`TranslationError::BadMoment`] if they do not form a valid instant: 30
    /// February, or second 60, which the kernel's second count cannot
    /// represent.
    pub fn at(self, time: TimeOfDay) -> Result<Instant<Utc>, TranslationError> {
        Instant::from_civil(crate::field::civil(self, time)).map_err(TranslationError::BadMoment)
    }
}

impl fmt::Display for Date {
    /// `ddmmyy` wire form.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{:02}{:02}{:02}",
            self.day,
            self.month,
            self.year.rem_euclid(100)
        )
    }
}

/// Data status of a position sentence: `A` or `V`.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Status {
    /// `A`: valid.
    Valid,
    /// `V`: warning, data not valid.
    Warning,
}

impl Status {
    const fn letter(self) -> char {
        match self {
            Self::Valid => 'A',
            Self::Warning => 'V',
        }
    }
}

/// Mode indicator (NMEA 2.3+): how the position was obtained.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Mode {
    /// `A`: autonomous.
    Autonomous,
    /// `D`: differential.
    Differential,
    /// `E`: estimated (receiver dead reckoning).
    Estimated,
    /// `F`: RTK float.
    FloatRtk,
    /// `M`: manual input.
    Manual,
    /// `N`: no fix.
    NoFix,
    /// `P`: precise.
    Precise,
    /// `R`: RTK fixed.
    Rtk,
    /// `S`: simulator.
    Simulator,
}

impl Mode {
    pub(crate) const fn from_letter(letter: u8) -> Option<Self> {
        Some(match letter {
            b'A' => Self::Autonomous,
            b'D' => Self::Differential,
            b'E' => Self::Estimated,
            b'F' => Self::FloatRtk,
            b'M' => Self::Manual,
            b'N' => Self::NoFix,
            b'P' => Self::Precise,
            b'R' => Self::Rtk,
            b'S' => Self::Simulator,
            _ => return None,
        })
    }

    const fn letter(self) -> char {
        match self {
            Self::Autonomous => 'A',
            Self::Differential => 'D',
            Self::Estimated => 'E',
            Self::FloatRtk => 'F',
            Self::Manual => 'M',
            Self::NoFix => 'N',
            Self::Precise => 'P',
            Self::Rtk => 'R',
            Self::Simulator => 'S',
        }
    }

    /// Fix type for this mode.
    #[must_use]
    pub const fn fix_type(self) -> FixType {
        match self {
            Self::Autonomous => FixType::Autonomous,
            Self::Differential => FixType::Differential,
            Self::Estimated => FixType::Estimated,
            Self::FloatRtk => FixType::RtkFloat,
            Self::Manual => FixType::Manual,
            Self::NoFix => FixType::None,
            Self::Precise => FixType::Precise,
            Self::Rtk => FixType::RtkFixed,
            Self::Simulator => FixType::Simulated,
        }
    }
}

impl fmt::Display for Mode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.letter())
    }
}

impl fmt::Display for Status {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.letter())
    }
}

/// Parsed sentence, supported or not.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Sentence {
    /// Recommended minimum: position, velocity, time and date.
    Rmc(Rmc),
    /// Fix data: position, time, quality, satellites, HDOP, altitude.
    Gga(Gga),
    /// Geographic position: latitude, longitude, time.
    Gll(Gll),
    /// Course and speed over the ground.
    Vtg(Vtg),
    /// AIS message or fragment (`VDM`/`VDO`), payload still armoured.
    Vdm(Vdm),
    /// Well-formed sentence (checksum verified) of an unsupported type. Not an
    /// error; the caller decides whether to log or ignore it.
    Unsupported {
        /// Address field as received: talker + formatter, or proprietary code.
        address: Address,
    },
}

/// Sentence address field; at most 10 characters (proprietary maximum).
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub struct Address {
    bytes: [u8; 10],
    len: u8,
}

impl Address {
    fn new(address: &[u8]) -> Self {
        let mut bytes = [0; 10];
        let len = address.len().min(bytes.len());
        for (slot, byte) in bytes.iter_mut().zip(address) {
            *slot = *byte;
        }
        // `len ≤ 10`.
        #[allow(clippy::cast_possible_truncation)]
        Self {
            bytes,
            len: len as u8,
        }
    }

    /// Address as `&str`.
    #[must_use]
    pub fn as_str(&self) -> &str {
        self.bytes
            .get(..usize::from(self.len))
            .and_then(|bytes| core::str::from_utf8(bytes).ok())
            .unwrap_or("")
    }
}

impl fmt::Debug for Address {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt::Debug::fmt(self.as_str(), f)
    }
}

impl fmt::Display for Address {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

impl Sentence {
    /// Decodes a framed sentence by formatter.
    pub(crate) fn decode(frame: Frame<'_>) -> Result<Self, NmeaError> {
        let (talker, formatter) = match frame.address {
            [t0, t1, f0, f1, f2] => (Talker::new([*t0, *t1])?, [*f0, *f1, *f2]),
            _ => {
                return Ok(Self::Unsupported {
                    address: Address::new(frame.address),
                })
            }
        };
        if frame.encapsulated {
            return Ok(match &formatter {
                b"VDM" => Self::Vdm(Vdm::decode(talker, false, frame)?),
                b"VDO" => Self::Vdm(Vdm::decode(talker, true, frame)?),
                _ => Self::Unsupported {
                    address: Address::new(frame.address),
                },
            });
        }
        Ok(match &formatter {
            b"RMC" => Self::Rmc(Rmc::decode(talker, frame)?),
            b"GGA" => Self::Gga(Gga::decode(talker, frame)?),
            b"GLL" => Self::Gll(Gll::decode(talker, frame)?),
            b"VTG" => Self::Vtg(Vtg::decode(talker, frame)?),
            _ => Self::Unsupported {
                address: Address::new(frame.address),
            },
        })
    }
}

impl fmt::Display for Sentence {
    /// Wire form, `$` to checksum, without `CR LF`.
    ///
    /// [`Unsupported`](Sentence::Unsupported) cannot be re-encoded (its fields
    /// are not kept) and formats as an error.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Rmc(rmc) => rmc.fmt(f),
            Self::Gga(gga) => gga.fmt(f),
            Self::Gll(gll) => gll.fmt(f),
            Self::Vtg(vtg) => vtg.fmt(f),
            Self::Vdm(vdm) => vdm.fmt(f),
            Self::Unsupported { .. } => Err(fmt::Error),
        }
    }
}
