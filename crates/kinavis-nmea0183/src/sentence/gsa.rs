//! `GSA` — DOP and active satellites.
//!
//! ```text
//! $GPGSA,A,3,04,05,,09,12,,,24,,,,,2.5,1.3,2.1[,s]*hh
//! ```
//!
//! Selection mode, fix dimension, the satellites used in the fix, PDOP,
//! HDOP and VDOP; since NMEA 4.10 a system identifier (`s`) names the
//! constellation of a `GN` sentence (a decimal number in its place, as
//! SiRF-TriG writes, is ignored). Twelve satellite fields is the standard;
//! some receivers write more, and the count follows the sentence: the last
//! three fields are the DOPs, preceded by the satellites, except that exactly
//! twelve satellites and four trailing fields mean a system identifier.

use core::fmt::{self, Write};

use kinavis_kernel::gnss::Dop;
use kinavis_kernel::inline::Inline;
use kinavis_kernel::{KernelError, SatelliteId, SatelliteSet, SatelliteView};

use crate::encode;
use crate::error::NmeaError;
use crate::field::Field;
use crate::frame::Frame;
use crate::sentence::satellites::satellite;
use crate::sentence::Talker;

/// Most satellite fields read from one `GSA`.
pub const MAX_GSA_SATELLITES: usize = 32;

/// Standard number of satellite fields.
const STANDARD_SLOTS: usize = 12;

/// How the fix dimension was chosen.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Selection {
    /// `A`: automatic 2D/3D.
    Automatic,
    /// `M`: forced by the operator.
    Manual,
}

/// Fix dimension.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Dimension {
    /// `1`: no fix.
    NoFix,
    /// `2`: horizontal only.
    TwoD,
    /// `3`: horizontal and vertical.
    ThreeD,
}

/// DOP and active satellites.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Gsa {
    /// Talker.
    pub talker: Talker,
    /// Selection mode.
    pub selection: Selection,
    /// Fix dimension.
    pub dimension: Dimension,
    /// Satellite fields in order, `None` for an empty one: a `GRS` residual
    /// belongs to the satellite in the same position.
    pub slots: Inline<Option<u16>, MAX_GSA_SATELLITES>,
    /// Position dilution of precision.
    pub pdop: Option<Dop>,
    /// Horizontal dilution of precision.
    pub hdop: Option<Dop>,
    /// Vertical dilution of precision.
    pub vdop: Option<Dop>,
    /// NMEA 4.10 system identifier.
    pub system: Option<u8>,
}

impl Gsa {
    /// Minimum field count: mode, dimension and the three DOPs.
    const REQUIRED: usize = 5;

    pub(crate) fn decode(talker: Talker, frame: Frame<'_>) -> Result<Self, NmeaError> {
        let found = frame.fields().remaining();
        if found < Self::REQUIRED {
            return Err(NmeaError::TooFewFields {
                found,
                required: Self::REQUIRED,
            });
        }
        let trailing = found.saturating_sub(2);
        let (slots, has_system) = if trailing == STANDARD_SLOTS + 4 {
            (STANDARD_SLOTS, true)
        } else {
            (trailing.saturating_sub(3), false)
        };
        let mut fields = frame.fields().indexed();
        let mut next = || {
            fields.next().unwrap_or(Field {
                bytes: &[],
                index: usize::MAX,
            })
        };
        let selection = next();
        let selection = match selection.bytes {
            b"A" => Selection::Automatic,
            b"M" => Selection::Manual,
            _ => {
                return Err(NmeaError::BadField {
                    index: selection.index,
                    expected: "selection mode",
                })
            }
        };
        let dimension = next();
        let dimension = match dimension.bytes {
            b"1" => Dimension::NoFix,
            b"2" => Dimension::TwoD,
            b"3" => Dimension::ThreeD,
            _ => {
                return Err(NmeaError::BadField {
                    index: dimension.index,
                    expected: "fix dimension",
                })
            }
        };
        let mut numbers = Inline::new(None);
        for _ in 0..slots {
            let field = next();
            let number = field.optional_unsigned::<u16>("satellite number")?;
            numbers.push(number).map_err(|_| NmeaError::BadField {
                index: field.index,
                expected: "at most 32 satellite fields",
            })?;
        }
        let pdop = next().optional_dop()?;
        let hdop = next().optional_dop()?;
        let vdop = next().optional_dop()?;
        // SiRF-TriG writes a decimal number in place of the system
        // identifier; it is not one and is left out.
        let system = match next() {
            field if !has_system || field.bytes.contains(&b'.') => None,
            field => field.optional_hex_digit("system identifier")?,
        };
        Ok(Self {
            talker,
            selection,
            dimension,
            slots: numbers,
            pdop,
            hdop,
            vdop,
            system,
        })
    }

    /// The satellites used in the fix that this crate can name, in order.
    pub fn satellites(&self) -> impl Iterator<Item = SatelliteId> + '_ {
        self.slot_satellites().flatten()
    }

    /// Each slot's satellite: `None` for an empty slot or a number this crate
    /// cannot name.
    pub fn slot_satellites(&self) -> impl Iterator<Item = Option<SatelliteId>> + '_ {
        self.slots
            .iter()
            .map(|slot| slot.and_then(|number| satellite(self.talker, self.system, number)))
    }
}

impl Gsa {
    /// Marks this sentence's satellites as used in the fix; a satellite not
    /// yet in `set` is added without sky position.
    ///
    /// # Errors
    ///
    /// [`KernelError::CapacityExceeded`] if `set` is full.
    pub fn apply<const N: usize>(&self, set: &mut SatelliteSet<N>) -> Result<(), KernelError> {
        for id in self.satellites() {
            let view = match set.get(id) {
                Some(view) => *view,
                None => SatelliteView::new(id, None, None)?,
            };
            set.insert(view.used_in_fix(true))?;
        }
        Ok(())
    }
}

impl fmt::Display for Gsa {
    /// The slots as read; twelve, all empty, when there are none.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        encode::sentence(f, |out| {
            let selection = match self.selection {
                Selection::Automatic => 'A',
                Selection::Manual => 'M',
            };
            let dimension = match self.dimension {
                Dimension::NoFix => 1,
                Dimension::TwoD => 2,
                Dimension::ThreeD => 3,
            };
            write!(out, "{}GSA,{selection},{dimension}", self.talker)?;
            let count = self.slots.len().max(STANDARD_SLOTS);
            for slot in 0..count {
                out.write_char(',')?;
                encode::optional(out, self.slots.get(slot).copied().flatten().map(TwoDigits))?;
            }
            for dop in [self.pdop, self.hdop, self.vdop] {
                out.write_char(',')?;
                encode::optional(out, dop.map(|d| OneDecimal(d.value())))?;
            }
            if let Some(system) = self.system {
                write!(out, ",{system:X}")?;
            }
            Ok(())
        })
    }
}

struct TwoDigits(u16);

impl fmt::Display for TwoDigits {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:02}", self.0)
    }
}

/// DOP with one decimal, at least `0.1` so the field reads back as a DOP.
struct OneDecimal(f64);

impl fmt::Display for OneDecimal {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.0 < 0.05 {
            f.write_str("0.1")
        } else {
            write!(f, "{:.1}", self.0)
        }
    }
}
