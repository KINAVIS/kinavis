//! Class A static and voyage data (message 5) and class B static data report
//! (message 24).

use core::fmt;

use kinavis_kernel::event::TargetId;
use kinavis_kernel::units::Distance;
use kinavis_kernel::InlineStr;

use crate::bits::Bits;
use crate::error::AisError;
use crate::fields::{byte, counts_per, value_of, Fields};
use crate::vessel::{Dimensions, PositionFixingDevice, ShipType};

/// Character counts of the text fields.
const NAME_CHARS: usize = 20;
const CALLSIGN_CHARS: usize = 7;
const VENDOR_CHARS: usize = 3;

/// Field values meaning "not available".
const NO_IMO: u32 = 0;
const NO_DRAUGHT: u32 = 0;

/// Class A static and voyage data: message 5.
///
/// Every field that can be "not available" is an `Option`; text fields are
/// `None`, not empty, when entirely padding.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct StaticAndVoyageData {
    /// MMSI.
    pub mmsi: TargetId,
    /// AIS version: 0 = M.1371-1, 1 = M.1371-3, 2 = M.1371-5, 3 = later.
    pub ais_version: u8,
    /// IMO number.
    pub imo: Option<u32>,
    /// Call sign, up to 7 characters.
    pub callsign: Option<InlineStr<CALLSIGN_CHARS>>,
    /// Name, up to 20 characters.
    pub name: Option<InlineStr<NAME_CHARS>>,
    /// Ship and cargo type.
    pub ship_type: Option<ShipType>,
    /// Dimensions around the reported position.
    pub dimensions: Option<Dimensions>,
    /// Position fixing device.
    pub fixing_device: Option<PositionFixingDevice>,
    /// ETA, UTC.
    pub eta: Eta,
    /// Maximum present static draught, 0.1 m resolution. 25.5 m means 25.5 m or
    /// more.
    pub draught: Option<Distance>,
    /// Destination, up to 20 characters.
    pub destination: Option<InlineStr<NAME_CHARS>>,
    /// DTE ready; stations without a DTE report not ready.
    pub data_terminal_ready: bool,
}

impl StaticAndVoyageData {
    /// Message length: 424 bits.
    const NEEDED: usize = 424;

    pub(crate) fn decode(bits: &Bits) -> Result<Self, AisError> {
        let field = Fields::of(bits, Self::NEEDED)?;
        Ok(Self {
            mmsi: TargetId::new(field.unsigned(8, 30)),
            ais_version: byte(field.unsigned(38, 2)),
            imo: imo(field.unsigned(40, 30)),
            callsign: field.text(70, CALLSIGN_CHARS),
            name: field.text(112, NAME_CHARS),
            ship_type: ShipType::from_code(byte(field.unsigned(232, 8))),
            dimensions: Dimensions::from_fields(
                field.unsigned(240, 9),
                field.unsigned(249, 9),
                field.unsigned(258, 6),
                field.unsigned(264, 6),
            )?,
            fixing_device: PositionFixingDevice::from_code(byte(field.unsigned(270, 4))),
            eta: Eta::from_fields(
                field.unsigned(274, 4),
                field.unsigned(278, 5),
                field.unsigned(283, 5),
                field.unsigned(288, 6),
            ),
            draught: draught(field.unsigned(294, 8))?,
            destination: field.text(302, NAME_CHARS),
            data_terminal_ready: !field.bit(422),
        })
    }
}

/// ETA, UTC, as far as given.
///
/// No year; each part may be absent independently (day known, hour not). An
/// out-of-range part (month 13) is also "not available", since the standard
/// defines no meaning for it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Eta {
    /// Month, `1..=12`.
    pub month: Option<u8>,
    /// Day, `1..=31`.
    pub day: Option<u8>,
    /// Hour, `0..=23`.
    pub hour: Option<u8>,
    /// Minute, `0..=59`.
    pub minute: Option<u8>,
}

impl Eta {
    /// ETA with no part given.
    pub const NONE: Self = Self {
        month: None,
        day: None,
        hour: None,
        minute: None,
    };

    fn from_fields(month: u32, day: u32, hour: u32, minute: u32) -> Self {
        Self {
            month: ((1..=12).contains(&month)).then_some(byte(month)),
            day: ((1..=31).contains(&day)).then_some(byte(day)),
            hour: (hour <= 23).then_some(byte(hour)),
            minute: (minute <= 59).then_some(byte(minute)),
        }
    }

    /// Whether any part is given.
    #[must_use]
    pub const fn is_given(&self) -> bool {
        self.month.is_some() || self.day.is_some() || self.hour.is_some() || self.minute.is_some()
    }
}

impl fmt::Display for Eta {
    /// Formats as `MM-DD HH:MM`, `??` for missing parts.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let part = |f: &mut fmt::Formatter<'_>, value: Option<u8>| match value {
            Some(value) => write!(f, "{value:02}"),
            None => f.write_str("??"),
        };
        part(f, self.month)?;
        f.write_str("-")?;
        part(f, self.day)?;
        f.write_str(" ")?;
        part(f, self.hour)?;
        f.write_str(":")?;
        part(f, self.minute)
    }
}

/// Class B static data report: message 24, sent as two parts.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct StaticDataReport {
    /// MMSI.
    pub mmsi: TargetId,
    /// Part and its content.
    pub part: StaticDataPart,
}

impl StaticDataReport {
    /// Part A: 160 bits; part B: 168.
    const PART_A_NEEDED: usize = 160;
    const PART_B_NEEDED: usize = 168;

    /// `None` for a reserved part number.
    pub(crate) fn decode(bits: &Bits) -> Result<Option<Self>, AisError> {
        let field = Fields::of(bits, 40)?;
        let mmsi = TargetId::new(field.unsigned(8, 30));
        let part = match field.unsigned(38, 2) {
            0 => {
                let field = Fields::of(bits, Self::PART_A_NEEDED)?;
                StaticDataPart::A {
                    name: field.text(40, NAME_CHARS),
                }
            }
            1 => {
                let field = Fields::of(bits, Self::PART_B_NEEDED)?;
                StaticDataPart::B(ClassBStaticData::decode(&field, mmsi)?)
            }
            _ => return Ok(None),
        };
        Ok(Some(Self { mmsi, part }))
    }
}

/// Parts of a class B static data report.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum StaticDataPart {
    /// Part A: name.
    A {
        /// Name, up to 20 characters.
        name: Option<InlineStr<NAME_CHARS>>,
    },
    /// Part B: remaining static data.
    B(ClassBStaticData),
}

/// Class B static data, part B.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ClassBStaticData {
    /// Ship type.
    pub ship_type: Option<ShipType>,
    /// Vendor ID, 3 characters.
    pub vendor_id: Option<InlineStr<VENDOR_CHARS>>,
    /// Vendor model code, `0..=15`.
    pub unit_model: u8,
    /// Vendor serial number, 20 bits.
    pub serial_number: u32,
    /// Call sign, up to 7 characters.
    pub callsign: Option<InlineStr<CALLSIGN_CHARS>>,
    /// Dimensions around the reported position; for an auxiliary craft this
    /// field carries the mothership MMSI instead.
    pub dimensions: Option<Dimensions>,
    /// Mothership MMSI, for an auxiliary craft (MMSI starting 98).
    pub mother_ship: Option<TargetId>,
    /// Position fixing device.
    pub fixing_device: Option<PositionFixingDevice>,
}

impl ClassBStaticData {
    fn decode(field: &Fields<'_>, mmsi: TargetId) -> Result<Self, AisError> {
        let carried = is_carried_craft(mmsi);
        Ok(Self {
            ship_type: ShipType::from_code(byte(field.unsigned(40, 8))),
            vendor_id: field.text(48, VENDOR_CHARS),
            unit_model: byte(field.unsigned(66, 4)),
            serial_number: field.unsigned(70, 20),
            callsign: field.text(90, CALLSIGN_CHARS),
            dimensions: if carried {
                None
            } else {
                Dimensions::from_fields(
                    field.unsigned(132, 9),
                    field.unsigned(141, 9),
                    field.unsigned(150, 6),
                    field.unsigned(156, 6),
                )?
            },
            mother_ship: carried.then(|| TargetId::new(field.unsigned(132, 30))),
            fixing_device: PositionFixingDevice::from_code(byte(field.unsigned(162, 4))),
        })
    }
}

/// Whether an MMSI is an auxiliary craft of a parent ship: `98`, MID, four
/// digits.
const fn is_carried_craft(mmsi: TargetId) -> bool {
    mmsi.number() / 10_000_000 == 98
}

fn imo(field: u32) -> Option<u32> {
    (field != NO_IMO).then_some(field)
}

fn draught(field: u32) -> Result<Option<Distance>, AisError> {
    if field == NO_DRAUGHT {
        return Ok(None);
    }
    // Draught in 0.1 m.
    Distance::from_metres(f64::from(field) / counts_per::TENTH)
        .map(Some)
        .map_err(value_of("draught"))
}
