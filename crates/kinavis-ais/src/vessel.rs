//! Static vessel data shared by several messages: ship type, dimensions,
//! position fixing device.
//!
//! Used by messages 5, 24 and 19; dimensions and fixing device also by message
//! 21.

use core::fmt;

use kinavis_kernel::units::Distance;

use crate::error::AisError;
use crate::fields::value_of;

/// Ship and cargo type, as the standard's two-digit code.
///
/// First digit: category (3 fishing/special craft, 6 passenger, 7 cargo, 8
/// tanker). For cargo-carrying categories the second digit is the cargo hazard
/// category. The raw code is kept so codes added later are not lost;
/// [`ShipType::category`] and [`ShipType::hazard`] interpret it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ShipType(u8);

impl ShipType {
    /// Type for a code; `None` for 0 ("not available").
    #[must_use]
    pub const fn from_code(code: u8) -> Option<Self> {
        if code == 0 {
            None
        } else {
            Some(Self(code))
        }
    }

    /// Code, `1..=255`.
    #[must_use]
    pub const fn code(self) -> u8 {
        self.0
    }

    /// Ship category.
    #[must_use]
    pub const fn category(self) -> ShipCategory {
        match self.0 {
            20..=29 => ShipCategory::WingInGround,
            30 => ShipCategory::Fishing,
            31 => ShipCategory::Towing,
            32 => ShipCategory::TowingLarge,
            33 => ShipCategory::DredgingOrUnderwaterOperations,
            34 => ShipCategory::DivingOperations,
            35 => ShipCategory::MilitaryOperations,
            36 => ShipCategory::Sailing,
            37 => ShipCategory::PleasureCraft,
            40..=49 => ShipCategory::HighSpeedCraft,
            50 => ShipCategory::PilotVessel,
            51 => ShipCategory::SearchAndRescue,
            52 => ShipCategory::Tug,
            53 => ShipCategory::PortTender,
            54 => ShipCategory::AntiPollution,
            55 => ShipCategory::LawEnforcement,
            58 => ShipCategory::MedicalTransport,
            59 => ShipCategory::Noncombatant,
            60..=69 => ShipCategory::Passenger,
            70..=79 => ShipCategory::Cargo,
            80..=89 => ShipCategory::Tanker,
            90..=99 => ShipCategory::Other,
            _ => ShipCategory::Reserved,
        }
    }

    /// Cargo hazard category, where defined: second digit 1–4 for WIG, HSC,
    /// passenger, cargo, tanker or other.
    #[must_use]
    pub const fn hazard(self) -> Option<HazardCategory> {
        let grouped = matches!(self.0, 20..=29 | 40..=49 | 60..=99);
        if !grouped {
            return None;
        }
        Some(match self.0 % 10 {
            1 => HazardCategory::A,
            2 => HazardCategory::B,
            3 => HazardCategory::C,
            4 => HazardCategory::D,
            _ => return None,
        })
    }
}

impl fmt::Display for ShipType {
    /// Formats as category and hazard if any: `tanker, hazard category A`.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.category() {
            ShipCategory::Reserved => write!(f, "ship type {}", self.0)?,
            category => write!(f, "{category}")?,
        }
        if let Some(hazard) = self.hazard() {
            write!(f, ", hazard category {hazard}")?;
        }
        Ok(())
    }
}

/// Ship categories.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum ShipCategory {
    /// Wing-in-ground craft, `20..=29`.
    WingInGround,
    /// Fishing, 30.
    Fishing,
    /// Towing, 31.
    Towing,
    /// Towing, tow over 200 m long or 25 m wide, 32.
    TowingLarge,
    /// Dredging or underwater operations, 33.
    DredgingOrUnderwaterOperations,
    /// Diving operations, 34.
    DivingOperations,
    /// Military operations, 35.
    MilitaryOperations,
    /// Sailing, 36.
    Sailing,
    /// Pleasure craft, 37.
    PleasureCraft,
    /// High speed craft, `40..=49`.
    HighSpeedCraft,
    /// Pilot vessel, 50.
    PilotVessel,
    /// Search and rescue vessel, 51.
    SearchAndRescue,
    /// Tug, 52.
    Tug,
    /// Port tender, 53.
    PortTender,
    /// Anti-pollution equipment, 54.
    AntiPollution,
    /// Law enforcement, 55.
    LawEnforcement,
    /// Medical transport, 58.
    MedicalTransport,
    /// Noncombatant ship per RR Resolution 18, 59.
    Noncombatant,
    /// Passenger, `60..=69`.
    Passenger,
    /// Cargo, `70..=79`.
    Cargo,
    /// Tanker, `80..=89`.
    Tanker,
    /// Other, `90..=99`.
    Other,
    /// Reserved or regional-use code.
    Reserved,
}

impl fmt::Display for ShipCategory {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::WingInGround => "wing in ground",
            Self::Fishing => "fishing",
            Self::Towing => "towing",
            Self::TowingLarge => "towing a large tow",
            Self::DredgingOrUnderwaterOperations => "dredging or underwater operations",
            Self::DivingOperations => "diving operations",
            Self::MilitaryOperations => "military operations",
            Self::Sailing => "sailing",
            Self::PleasureCraft => "pleasure craft",
            Self::HighSpeedCraft => "high speed craft",
            Self::PilotVessel => "pilot vessel",
            Self::SearchAndRescue => "search and rescue",
            Self::Tug => "tug",
            Self::PortTender => "port tender",
            Self::AntiPollution => "anti-pollution",
            Self::LawEnforcement => "law enforcement",
            Self::MedicalTransport => "medical transport",
            Self::Noncombatant => "noncombatant",
            Self::Passenger => "passenger",
            Self::Cargo => "cargo",
            Self::Tanker => "tanker",
            Self::Other => "other",
            Self::Reserved => "reserved",
        })
    }
}

/// Cargo hazard category (A most hazardous, D least).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum HazardCategory {
    /// MARPOL Annex II category X, or the most hazardous under the applicable
    /// code.
    A,
    /// Category Y.
    B,
    /// Category Z.
    C,
    /// Other substances.
    D,
}

impl fmt::Display for HazardCategory {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::A => "A",
            Self::B => "B",
            Self::C => "C",
            Self::D => "D",
        })
    }
}

/// Dimensions from the reported position to bow, stern, port and starboard.
///
/// The reference point is the position antenna, so the fields locate the hull
/// relative to it as well as giving its size. Whole metres; fore-aft fields
/// saturate at 511 m, athwartships at 63 m (meaning that value or more).
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Dimensions {
    /// Reference point to bow.
    pub to_bow: Distance,
    /// Reference point to stern.
    pub to_stern: Distance,
    /// Reference point to port side.
    pub to_port: Distance,
    /// Reference point to starboard side.
    pub to_starboard: Distance,
}

impl Dimensions {
    /// From the four fields in metres; `None` if all zero ("not available").
    pub(crate) fn from_fields(
        to_bow: u32,
        to_stern: u32,
        to_port: u32,
        to_starboard: u32,
    ) -> Result<Option<Self>, AisError> {
        if to_bow == 0 && to_stern == 0 && to_port == 0 && to_starboard == 0 {
            return Ok(None);
        }
        let metres =
            |field: u32| Distance::from_metres(f64::from(field)).map_err(value_of("dimensions"));
        Ok(Some(Self {
            to_bow: metres(to_bow)?,
            to_stern: metres(to_stern)?,
            to_port: metres(to_port)?,
            to_starboard: metres(to_starboard)?,
        }))
    }

    /// Length overall.
    #[must_use]
    pub fn length(&self) -> Distance {
        self.to_bow + self.to_stern
    }

    /// Beam.
    #[must_use]
    pub fn beam(&self) -> Distance {
        self.to_port + self.to_starboard
    }
}

/// Position fixing device type.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum PositionFixingDevice {
    /// GPS.
    Gps,
    /// GLONASS.
    Glonass,
    /// GPS and GLONASS together.
    GpsAndGlonass,
    /// Loran-C.
    LoranC,
    /// Chayka.
    Chayka,
    /// Integrated navigation system.
    IntegratedNavigationSystem,
    /// Surveyed position: fixed station or aid to navigation.
    Surveyed,
    /// Galileo.
    Galileo,
    /// Internal GNSS (class B).
    InternalGnss,
    /// Reserved code, kept as received.
    Reserved(u8),
}

impl PositionFixingDevice {
    /// Device for a code; `None` for 0 ("undefined").
    #[must_use]
    pub const fn from_code(code: u8) -> Option<Self> {
        Some(match code {
            0 => return None,
            1 => Self::Gps,
            2 => Self::Glonass,
            3 => Self::GpsAndGlonass,
            4 => Self::LoranC,
            5 => Self::Chayka,
            6 => Self::IntegratedNavigationSystem,
            7 => Self::Surveyed,
            8 => Self::Galileo,
            15 => Self::InternalGnss,
            _ => Self::Reserved(code),
        })
    }

    /// Code for the device.
    #[must_use]
    pub const fn code(self) -> u8 {
        match self {
            Self::Gps => 1,
            Self::Glonass => 2,
            Self::GpsAndGlonass => 3,
            Self::LoranC => 4,
            Self::Chayka => 5,
            Self::IntegratedNavigationSystem => 6,
            Self::Surveyed => 7,
            Self::Galileo => 8,
            Self::InternalGnss => 15,
            Self::Reserved(code) => code,
        }
    }
}

impl fmt::Display for PositionFixingDevice {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Gps => f.write_str("GPS"),
            Self::Glonass => f.write_str("GLONASS"),
            Self::GpsAndGlonass => f.write_str("GPS and GLONASS"),
            Self::LoranC => f.write_str("Loran-C"),
            Self::Chayka => f.write_str("Chayka"),
            Self::IntegratedNavigationSystem => f.write_str("integrated navigation system"),
            Self::Surveyed => f.write_str("surveyed"),
            Self::Galileo => f.write_str("Galileo"),
            Self::InternalGnss => f.write_str("internal GNSS"),
            Self::Reserved(code) => write!(f, "reserved device {code}"),
        }
    }
}
