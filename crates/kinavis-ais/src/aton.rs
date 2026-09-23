//! Aid-to-navigation report, message 21: buoys, beacons, lights, and virtual
//! aids broadcast by a shore station.

use core::fmt;

use kinavis_kernel::event::TargetId;
use kinavis_kernel::position::Position;
use kinavis_kernel::InlineStr;

use crate::bits::Bits;
use crate::error::AisError;
use crate::fields::{byte, position, second, Fields};
use crate::vessel::{Dimensions, PositionFixingDevice};

/// Name field: 20 characters; name extension at the end of the message: up to
/// 14 more.
const NAME_CHARS: usize = 20;
const EXTENSION_CHARS: usize = 14;
const ALL_NAME_CHARS: usize = NAME_CHARS + EXTENSION_CHARS;

/// Aid-to-navigation report.
// Independent flags, each a field of the standard's table; not a state machine.
#[allow(clippy::struct_excessive_bools)]
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct AidToNavigation {
    /// MMSI.
    pub mmsi: TargetId,
    /// Aid type.
    pub aid_type: Option<AidType>,
    /// Name, up to 34 characters: 20 in the name field plus the extension.
    pub name: Option<InlineStr<ALL_NAME_CHARS>>,
    /// Position accuracy flag: DGNSS, better than 10 m.
    pub accurate: bool,
    /// Position, 1/10 000 minute resolution.
    pub position: Option<Position>,
    /// Dimensions around the position; none for a virtual aid.
    pub dimensions: Option<Dimensions>,
    /// Position fixing device; `Surveyed` for a fixed aid.
    pub fixing_device: Option<PositionFixingDevice>,
    /// UTC second of the report, `0..=59`; `None` if no time is available or
    /// the position is dead-reckoned or manual.
    pub second: Option<u8>,
    /// Off-position flag for a floating aid. Defined only when the second is
    /// valid; otherwise `None`.
    pub off_position: Option<bool>,
    /// 8 bits reserved for regional use.
    pub regional: u8,
    /// RAIM flag.
    pub raim: bool,
    /// Virtual aid: broadcast from elsewhere, nothing physical at the position.
    pub is_virtual: bool,
    /// Assigned mode: transmitting on a schedule set by a competent authority.
    pub assigned: bool,
}

impl AidToNavigation {
    /// Fixed part: 272 bits, followed by up to 14 characters of name extension.
    const NEEDED: usize = 272;

    pub(crate) fn decode(bits: &Bits) -> Result<Self, AisError> {
        let field = Fields::of(bits, Self::NEEDED)?;
        let second = second(field.unsigned(253, 6));
        Ok(Self {
            mmsi: TargetId::new(field.unsigned(8, 30)),
            aid_type: AidType::from_code(byte(field.unsigned(38, 5))),
            name: name(&field),
            accurate: field.bit(163),
            position: position(field.signed(164, 28), field.signed(192, 27))?,
            dimensions: Dimensions::from_fields(
                field.unsigned(219, 9),
                field.unsigned(228, 9),
                field.unsigned(237, 6),
                field.unsigned(243, 6),
            )?,
            fixing_device: PositionFixingDevice::from_code(byte(field.unsigned(249, 4))),
            second,
            off_position: second.map(|_| field.bit(259)),
            regional: byte(field.unsigned(260, 8)),
            raim: field.bit(268),
            is_virtual: field.bit(269),
            assigned: field.bit(270),
        })
    }
}

/// Name from the name field plus any whole characters past the fixed part.
fn name(field: &Fields<'_>) -> Option<InlineStr<ALL_NAME_CHARS>> {
    let extension = ((field.len() - AidToNavigation::NEEDED) / 6).min(EXTENSION_CHARS);
    let in_field = (0..NAME_CHARS).map(|index| 43 + index * 6);
    let in_extension = (0..extension).map(|index| AidToNavigation::NEEDED + index * 6);
    field.text_at(in_field.chain(in_extension))
}

/// Aid-to-navigation type.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum AidType {
    /// Reference point, 1.
    ReferencePoint,
    /// RACON, 2.
    Racon,
    /// Fixed offshore structure (platform, wind farm), 3.
    FixedStructureOffshore,
    /// Light, with or without sectors, 5 and 6.
    Light {
        /// Sector light.
        sectored: bool,
    },
    /// Leading light, front or rear, 7 and 8.
    LeadingLight {
        /// Rear light of the pair.
        rear: bool,
    },
    /// Fixed beacon, `9..=19`.
    Beacon(Mark),
    /// Floating mark (buoy), `20..=30`.
    Floating(Mark),
    /// Light vessel, LANBY or rig, 31.
    LightVessel,
    /// Reserved code, kept as received.
    Reserved(u8),
}

impl AidType {
    /// Type for a code; `None` for 0 ("not specified").
    #[must_use]
    pub const fn from_code(code: u8) -> Option<Self> {
        Some(match code {
            0 => return None,
            1 => Self::ReferencePoint,
            2 => Self::Racon,
            3 => Self::FixedStructureOffshore,
            5 => Self::Light { sectored: false },
            6 => Self::Light { sectored: true },
            7 => Self::LeadingLight { rear: false },
            8 => Self::LeadingLight { rear: true },
            9..=19 => Self::Beacon(Mark::from_offset(code - 9)),
            20..=30 => Self::Floating(Mark::from_offset(code - 20)),
            31 => Self::LightVessel,
            _ => Self::Reserved(code),
        })
    }

    /// Code for the type.
    #[must_use]
    pub const fn code(self) -> u8 {
        match self {
            Self::ReferencePoint => 1,
            Self::Racon => 2,
            Self::FixedStructureOffshore => 3,
            Self::Light { sectored: false } => 5,
            Self::Light { sectored: true } => 6,
            Self::LeadingLight { rear: false } => 7,
            Self::LeadingLight { rear: true } => 8,
            Self::Beacon(mark) => 9 + mark.offset(),
            Self::Floating(mark) => 20 + mark.offset(),
            Self::LightVessel => 31,
            Self::Reserved(code) => code,
        }
    }
}

impl fmt::Display for AidType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ReferencePoint => f.write_str("reference point"),
            Self::Racon => f.write_str("RACON"),
            Self::FixedStructureOffshore => f.write_str("fixed structure off shore"),
            Self::Light { sectored: false } => f.write_str("light without sectors"),
            Self::Light { sectored: true } => f.write_str("light with sectors"),
            Self::LeadingLight { rear: false } => f.write_str("leading light, front"),
            Self::LeadingLight { rear: true } => f.write_str("leading light, rear"),
            Self::Beacon(mark) => write!(f, "beacon, {mark}"),
            Self::Floating(mark) => write!(f, "floating mark, {mark}"),
            Self::LightVessel => f.write_str("light vessel"),
            Self::Reserved(code) => write!(f, "reserved aid type {code}"),
        }
    }
}

/// IALA mark type of a beacon or buoy.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Mark {
    /// Cardinal mark: safe water on the named side.
    Cardinal(Quadrant),
    /// Port-hand lateral mark.
    PortHand,
    /// Starboard-hand lateral mark.
    StarboardHand,
    /// Preferred channel to starboard: pass as a port-hand mark.
    PreferredChannelPortHand,
    /// Preferred channel to port: pass as a starboard-hand mark.
    PreferredChannelStarboardHand,
    /// Isolated danger mark.
    IsolatedDanger,
    /// Safe water mark: fairway or landfall.
    SafeWater,
    /// Special mark.
    Special,
}

impl Mark {
    /// Mark at an offset into either code range, `0..=10`.
    const fn from_offset(offset: u8) -> Self {
        match offset {
            0 => Self::Cardinal(Quadrant::North),
            1 => Self::Cardinal(Quadrant::East),
            2 => Self::Cardinal(Quadrant::South),
            3 => Self::Cardinal(Quadrant::West),
            4 => Self::PortHand,
            5 => Self::StarboardHand,
            6 => Self::PreferredChannelPortHand,
            7 => Self::PreferredChannelStarboardHand,
            8 => Self::IsolatedDanger,
            9 => Self::SafeWater,
            _ => Self::Special,
        }
    }

    const fn offset(self) -> u8 {
        match self {
            Self::Cardinal(Quadrant::North) => 0,
            Self::Cardinal(Quadrant::East) => 1,
            Self::Cardinal(Quadrant::South) => 2,
            Self::Cardinal(Quadrant::West) => 3,
            Self::PortHand => 4,
            Self::StarboardHand => 5,
            Self::PreferredChannelPortHand => 6,
            Self::PreferredChannelStarboardHand => 7,
            Self::IsolatedDanger => 8,
            Self::SafeWater => 9,
            Self::Special => 10,
        }
    }
}

impl fmt::Display for Mark {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Cardinal(quadrant) => write!(f, "cardinal {quadrant}"),
            Self::PortHand => f.write_str("port hand"),
            Self::StarboardHand => f.write_str("starboard hand"),
            Self::PreferredChannelPortHand => f.write_str("preferred channel, port hand"),
            Self::PreferredChannelStarboardHand => f.write_str("preferred channel, starboard hand"),
            Self::IsolatedDanger => f.write_str("isolated danger"),
            Self::SafeWater => f.write_str("safe water"),
            Self::Special => f.write_str("special"),
        }
    }
}

/// Cardinal quadrant: the side of the danger where safe water lies.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Quadrant {
    /// Pass north of the mark.
    North,
    /// Pass east.
    East,
    /// Pass south.
    South,
    /// Pass west.
    West,
}

impl fmt::Display for Quadrant {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::North => "north",
            Self::East => "east",
            Self::South => "south",
            Self::West => "west",
        })
    }
}
