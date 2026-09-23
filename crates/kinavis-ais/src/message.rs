//! Message decoding into kernel types.

use core::fmt;

use kinavis_kernel::angle::TrueCourse;
use kinavis_kernel::event::TargetId;
use kinavis_kernel::position::Position;
use kinavis_kernel::snapshot::GroundTrack;
use kinavis_kernel::units::{RateOfTurn, Speed};
use kinavis_kernel::InlineStr;

use crate::aton::AidToNavigation;
use crate::bits::Bits;
use crate::error::AisError;
use crate::fields::{byte, course, heading, position, second, speed, value_of, Fields};
use crate::static_data::{StaticAndVoyageData, StaticDataReport};
use crate::vessel::{Dimensions, PositionFixingDevice, ShipType};

/// Decoded AIS message.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Message {
    /// Position report: messages 1, 2, 3 (class A), 18, 19 (class B).
    PositionReport(PositionReport),
    /// Class A static and voyage data: message 5.
    StaticAndVoyageData(StaticAndVoyageData),
    /// Class B static data: message 24, part A or B.
    StaticDataReport(StaticDataReport),
    /// Aid to navigation: message 21.
    AidToNavigation(AidToNavigation),
    /// Well-formed message of an unsupported type, for counting or logging.
    Unsupported {
        /// Message type, `0..=63`.
        kind: u8,
    },
}

impl Message {
    /// Decodes a message from its bits.
    ///
    /// # Errors
    ///
    /// [`AisError::TooShort`] if the bits end before a field of the type;
    /// [`AisError::Value`] if a field value is outside the domain.
    pub fn decode(bits: &Bits) -> Result<Self, AisError> {
        let kind = bits.unsigned(0, 6).ok_or(AisError::TooShort {
            bits: bits.len(),
            needed: 6,
        })?;
        let kind = byte(kind);
        match kind {
            1..=3 => PositionReport::class_a(bits, kind).map(Self::PositionReport),
            5 => StaticAndVoyageData::decode(bits).map(Self::StaticAndVoyageData),
            18 => PositionReport::class_b(bits, ClassB::STANDARD).map(Self::PositionReport),
            19 => PositionReport::class_b(bits, ClassB::EXTENDED).map(Self::PositionReport),
            21 => AidToNavigation::decode(bits).map(Self::AidToNavigation),
            // A reserved part number is an undefined message.
            24 => Ok(StaticDataReport::decode(bits)?
                .map_or(Self::Unsupported { kind }, Self::StaticDataReport)),
            _ => Ok(Self::Unsupported { kind }),
        }
    }

    /// Source MMSI; `None` for unsupported messages (not decoded).
    #[must_use]
    pub const fn mmsi(&self) -> Option<TargetId> {
        match self {
            Self::PositionReport(report) => Some(report.mmsi),
            Self::StaticAndVoyageData(data) => Some(data.mmsi),
            Self::StaticDataReport(report) => Some(report.mmsi),
            Self::AidToNavigation(aid) => Some(aid.mmsi),
            Self::Unsupported { .. } => None,
        }
    }
}

/// Station class.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum StationClass {
    /// Class A: SOLAS carriage requirement; reports navigational status and
    /// rate of turn.
    A,
    /// Class B: reports neither.
    B,
}

impl fmt::Display for StationClass {
    /// Formats as `class A` or `class B`.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::A => "class A",
            Self::B => "class B",
        })
    }
}

/// Class A navigational status.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum NavigationStatus {
    /// Under way using engine.
    UnderWayUsingEngine,
    /// At anchor.
    AtAnchor,
    /// Not under command.
    NotUnderCommand,
    /// Restricted in ability to manoeuvre.
    RestrictedManoeuvrability,
    /// Constrained by draught.
    ConstrainedByDraught,
    /// Moored.
    Moored,
    /// Aground.
    Aground,
    /// Engaged in fishing.
    Fishing,
    /// Under way sailing.
    UnderWaySailing,
    /// Power-driven vessel towing astern.
    TowingAstern,
    /// Power-driven vessel pushing ahead or towing alongside.
    PushingAheadOrTowingAlongside,
    /// AIS-SART, MOB-AIS or EPIRB-AIS active.
    SearchAndRescueTransmitter,
    /// Reserved code, kept as received.
    Reserved(u8),
}

impl NavigationStatus {
    /// Status for a code; `None` for 15 ("not defined").
    #[must_use]
    pub const fn from_code(code: u8) -> Option<Self> {
        Some(match code {
            0 => Self::UnderWayUsingEngine,
            1 => Self::AtAnchor,
            2 => Self::NotUnderCommand,
            3 => Self::RestrictedManoeuvrability,
            4 => Self::ConstrainedByDraught,
            5 => Self::Moored,
            6 => Self::Aground,
            7 => Self::Fishing,
            8 => Self::UnderWaySailing,
            11 => Self::TowingAstern,
            12 => Self::PushingAheadOrTowingAlongside,
            14 => Self::SearchAndRescueTransmitter,
            9 | 10 | 13 => Self::Reserved(code),
            _ => return None,
        })
    }

    /// Code for the status.
    #[must_use]
    pub const fn code(self) -> u8 {
        match self {
            Self::UnderWayUsingEngine => 0,
            Self::AtAnchor => 1,
            Self::NotUnderCommand => 2,
            Self::RestrictedManoeuvrability => 3,
            Self::ConstrainedByDraught => 4,
            Self::Moored => 5,
            Self::Aground => 6,
            Self::Fishing => 7,
            Self::UnderWaySailing => 8,
            Self::TowingAstern => 11,
            Self::PushingAheadOrTowingAlongside => 12,
            Self::SearchAndRescueTransmitter => 14,
            Self::Reserved(code) => code,
        }
    }
}

impl fmt::Display for NavigationStatus {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnderWayUsingEngine => f.write_str("under way using engine"),
            Self::AtAnchor => f.write_str("at anchor"),
            Self::NotUnderCommand => f.write_str("not under command"),
            Self::RestrictedManoeuvrability => f.write_str("restricted manoeuvrability"),
            Self::ConstrainedByDraught => f.write_str("constrained by draught"),
            Self::Moored => f.write_str("moored"),
            Self::Aground => f.write_str("aground"),
            Self::Fishing => f.write_str("engaged in fishing"),
            Self::UnderWaySailing => f.write_str("under way sailing"),
            Self::TowingAstern => f.write_str("towing astern"),
            Self::PushingAheadOrTowingAlongside => f.write_str("pushing ahead or towing alongside"),
            Self::SearchAndRescueTransmitter => f.write_str("search and rescue transmitter"),
            Self::Reserved(code) => write!(f, "reserved status {code}"),
        }
    }
}

/// Class A rate of turn.
///
/// The field encodes a scaled square root of the rate: fine resolution for slow
/// turns, coarse for fast ones; above 5°/s only the direction is given.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Turn {
    /// Rate from the turn indicator.
    Rate(RateOfTurn),
    /// Faster than 5°/s (field saturated).
    OffScale {
        /// Turning to port; otherwise starboard.
        to_port: bool,
    },
}

impl Turn {
    /// Field scale: `ROT_AIS = 4.733 √ROT`, ROT in °/min.
    const SCALE: f64 = 4.733;

    /// Turn for a field value; `None` for `-128` ("not available").
    fn from_field(value: i32) -> Result<Option<Self>, AisError> {
        Ok(Some(match value {
            -128 => return Ok(None),
            127 => Self::OffScale { to_port: false },
            -127 => Self::OffScale { to_port: true },
            _ => {
                let scaled = f64::from(value) / Self::SCALE;
                let magnitude = scaled * scaled;
                let rate = if value < 0 { -magnitude } else { magnitude };
                Self::Rate(
                    RateOfTurn::from_degrees_per_minute(rate).map_err(value_of("rate of turn"))?,
                )
            }
        }))
    }
}

/// Position report.
///
/// Every field that can be "not available" is an `Option`, `None` when
/// unavailable — never a zero that reads as a valid course or a stopped ship.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PositionReport {
    /// Message type: 1, 2 or 3 (class A), 18 or 19 (class B).
    pub kind: u8,
    /// MMSI.
    pub mmsi: TargetId,
    /// Navigational status; class A only.
    pub status: Option<NavigationStatus>,
    /// Rate of turn; class A only.
    pub turn: Option<Turn>,
    /// Speed over ground, 0.1 kn resolution. 102.2 kn means 102.2 kn or more.
    pub speed: Option<Speed>,
    /// Position accuracy flag: DGNSS, better than 10 m.
    pub accurate: bool,
    /// Position, 1/10 000 minute resolution.
    pub position: Option<Position>,
    /// Course over ground, 0.1° resolution.
    pub course: Option<TrueCourse>,
    /// True heading, 1° resolution.
    pub heading: Option<TrueCourse>,
    /// UTC second of the report, `0..=59`; `None` if no time is available or
    /// the position is dead-reckoned or manual.
    pub second: Option<u8>,
    /// RAIM flag.
    pub raim: bool,
    /// Name, up to 20 characters; message 19 only (others use static data).
    pub name: Option<InlineStr<20>>,
    /// Ship type; message 19 only.
    pub ship_type: Option<ShipType>,
    /// Dimensions around the position; message 19 only.
    pub dimensions: Option<Dimensions>,
    /// Position fixing device; message 19 only.
    pub fixing_device: Option<PositionFixingDevice>,
}

/// Class B field layout; messages 18 and 19 differ only at the end.
#[derive(Clone, Copy)]
struct ClassB {
    needed: usize,
    raim: usize,
}

impl ClassB {
    const STANDARD: Self = Self {
        needed: 168,
        raim: 147,
    };
    const EXTENDED: Self = Self {
        needed: 312,
        raim: 305,
    };
}

impl PositionReport {
    /// Messages 1, 2 and 3.
    fn class_a(bits: &Bits, kind: u8) -> Result<Self, AisError> {
        let field = Fields::of(bits, 168)?;
        Ok(Self {
            kind,
            mmsi: TargetId::new(field.unsigned(8, 30)),
            status: NavigationStatus::from_code(byte(field.unsigned(38, 4))),
            turn: Turn::from_field(field.signed(42, 8))?,
            speed: speed(field.unsigned(50, 10))?,
            accurate: field.bit(60),
            position: position(field.signed(61, 28), field.signed(89, 27))?,
            course: course(field.unsigned(116, 12))?,
            heading: heading(field.unsigned(128, 9))?,
            second: second(field.unsigned(137, 6)),
            raim: field.bit(148),
            name: None,
            ship_type: None,
            dimensions: None,
            fixing_device: None,
        })
    }

    /// Messages 18 and 19.
    fn class_b(bits: &Bits, layout: ClassB) -> Result<Self, AisError> {
        let field = Fields::of(bits, layout.needed)?;
        let extended = layout.needed == ClassB::EXTENDED.needed;
        Ok(Self {
            kind: byte(field.unsigned(0, 6)),
            mmsi: TargetId::new(field.unsigned(8, 30)),
            status: None,
            turn: None,
            speed: speed(field.unsigned(46, 10))?,
            accurate: field.bit(56),
            position: position(field.signed(57, 28), field.signed(85, 27))?,
            course: course(field.unsigned(112, 12))?,
            heading: heading(field.unsigned(124, 9))?,
            second: second(field.unsigned(133, 6)),
            raim: field.bit(layout.raim),
            name: extended.then(|| field.text(143, 20)).flatten(),
            ship_type: extended
                .then(|| ShipType::from_code(byte(field.unsigned(263, 8))))
                .flatten(),
            dimensions: if extended {
                Dimensions::from_fields(
                    field.unsigned(271, 9),
                    field.unsigned(280, 9),
                    field.unsigned(289, 6),
                    field.unsigned(295, 6),
                )?
            } else {
                None
            },
            fixing_device: extended
                .then(|| PositionFixingDevice::from_code(byte(field.unsigned(301, 4))))
                .flatten(),
        })
    }

    /// Station class.
    #[must_use]
    pub const fn station_class(&self) -> StationClass {
        match self.kind {
            18 | 19 => StationClass::B,
            _ => StationClass::A,
        }
    }

    /// Course and speed over ground, if both are reported.
    #[must_use]
    pub fn ground_track(&self) -> Option<GroundTrack> {
        Some(GroundTrack {
            course_over_ground: self.course?,
            speed_over_ground: self.speed?,
        })
    }
}
