//! Speed and distance of the tractor: wheel-based, ground-based, machine
//! selected, and the machine selected speed command.

use kinavis_j1939::Pgn;
use kinavis_kernel::{Distance, Speed};

use crate::slots::{
    distance, distance_raw, speed, speed_raw, u16_at, u32_at_2, Direction, Indicator, MAX_U8,
};

/// Wheel-based speed and distance, PGN 65096: from the wheels or the tail
/// shaft, so it slips with the wheels.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct WheelBasedSpeed {
    /// Speed, mm/s resolution, up to 64.255 m/s.
    pub speed: Option<Speed>,
    /// Distance travelled, mm resolution; wraps to zero after 4 211 081.215 m.
    pub distance: Option<Distance>,
    /// Remaining time of tractor-supplied electrical power at the present
    /// load, minutes.
    pub power_time_minutes: Option<u8>,
    /// Direction of travel; at a standstill, the last one.
    pub direction: Direction,
    /// Key switch: `Off`, or `On` for "not off".
    pub key_switch: Indicator,
    /// The operator's switch to start or enable implement operations.
    pub implement_operations: Indicator,
    /// Whether the operator faces against the direction reported.
    pub operator_reversed: Indicator,
}

impl WheelBasedSpeed {
    pub(crate) const PGN_NUMBER: u32 = 65_096;
    /// PGN 65096.
    pub const PGN: Pgn = Pgn::new(Self::PGN_NUMBER);

    /// From the 8 data bytes.
    #[must_use]
    pub fn from_bytes(bytes: [u8; 8]) -> Self {
        let [_, _, _, _, _, _, power, flags] = bytes;
        Self {
            speed: speed(u16_at(bytes, 0)),
            distance: distance(u32_at_2(bytes)),
            power_time_minutes: (power <= MAX_U8).then_some(power),
            direction: Direction::from_bits(flags),
            key_switch: Indicator::from_bits(flags >> 2),
            implement_operations: Indicator::from_bits(flags >> 4),
            operator_reversed: Indicator::from_bits(flags >> 6),
        }
    }

    /// The 8 data bytes.
    #[must_use]
    pub fn to_bytes(&self) -> [u8; 8] {
        let [s0, s1] = speed_raw(self.speed).to_le_bytes();
        let [d0, d1, d2, d3] = distance_raw(self.distance).to_le_bytes();
        let power = self
            .power_time_minutes
            .map_or(0xFF, |minutes| minutes.min(MAX_U8));
        let flags = self.direction.bits()
            | self.key_switch.bits() << 2
            | self.implement_operations.bits() << 4
            | self.operator_reversed.bits() << 6;
        [s0, s1, d0, d1, d2, d3, power, flags]
    }
}

/// Ground-based speed and distance, PGN 65097: from a radar or another
/// sensor of the ground, so it does not slip.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct GroundBasedSpeed {
    /// Speed, mm/s resolution.
    pub speed: Option<Speed>,
    /// Distance travelled, mm resolution.
    pub distance: Option<Distance>,
    /// Direction of travel.
    pub direction: Direction,
}

impl GroundBasedSpeed {
    pub(crate) const PGN_NUMBER: u32 = 65_097;
    /// PGN 65097.
    pub const PGN: Pgn = Pgn::new(Self::PGN_NUMBER);

    /// From the 8 data bytes.
    #[must_use]
    pub fn from_bytes(bytes: [u8; 8]) -> Self {
        let [_, _, _, _, _, _, _, flags] = bytes;
        Self {
            speed: speed(u16_at(bytes, 0)),
            distance: distance(u32_at_2(bytes)),
            direction: Direction::from_bits(flags),
        }
    }

    /// The 8 data bytes; the reserved bits are ones.
    #[must_use]
    pub fn to_bytes(&self) -> [u8; 8] {
        let [s0, s1] = speed_raw(self.speed).to_le_bytes();
        let [d0, d1, d2, d3] = distance_raw(self.distance).to_le_bytes();
        [s0, s1, d0, d1, d2, d3, 0xFF, 0xFC | self.direction.bits()]
    }
}

/// Source of the machine selected speed.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum SpeedSource {
    /// Wheel-based, usually a wheel encoder.
    WheelBased,
    /// Ground-based, usually a radar.
    GroundBased,
    /// Navigation-based, usually GNSS.
    NavigationBased,
    /// A blend of sources.
    Blended,
    /// Simulated, for tests.
    Simulated,
    /// A reserved code.
    Reserved(u8),
    /// Not available.
    NotAvailable,
}

impl SpeedSource {
    const fn from_bits(bits: u8) -> Self {
        match bits & 0x07 {
            0 => Self::WheelBased,
            1 => Self::GroundBased,
            2 => Self::NavigationBased,
            3 => Self::Blended,
            4 => Self::Simulated,
            7 => Self::NotAvailable,
            other => Self::Reserved(other),
        }
    }

    const fn bits(self) -> u8 {
        match self {
            Self::WheelBased => 0,
            Self::GroundBased => 1,
            Self::NavigationBased => 2,
            Self::Blended => 3,
            Self::Simulated => 4,
            Self::Reserved(code) => code & 0x07,
            Self::NotAvailable => 7,
        }
    }
}

/// Machine selected speed, PGN 61474: the speed the tractor judges best,
/// with where it came from.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct MachineSelectedSpeed {
    /// Speed, mm/s resolution.
    pub speed: Option<Speed>,
    /// Distance travelled, mm resolution.
    pub distance: Option<Distance>,
    /// Why remote control of the speed ended, as its code (0 all clear,
    /// 5 command timeout, 62 error); `None` when not available.
    pub exit_reason: Option<u8>,
    /// Direction of travel.
    pub direction: Direction,
    /// Source of the speed.
    pub source: SpeedSource,
    /// Limit on remote commands of the speed.
    pub limit: crate::guidance::LimitStatus,
}

impl MachineSelectedSpeed {
    pub(crate) const PGN_NUMBER: u32 = 61_474;
    /// PGN 61474.
    pub const PGN: Pgn = Pgn::new(Self::PGN_NUMBER);

    /// From the 8 data bytes.
    #[must_use]
    pub fn from_bytes(bytes: [u8; 8]) -> Self {
        let [_, _, _, _, _, _, exit, flags] = bytes;
        let exit = exit & 0x3F;
        Self {
            speed: speed(u16_at(bytes, 0)),
            distance: distance(u32_at_2(bytes)),
            exit_reason: (exit != 0x3F).then_some(exit),
            direction: Direction::from_bits(flags),
            source: SpeedSource::from_bits(flags >> 2),
            limit: crate::guidance::LimitStatus::from_bits(flags >> 5),
        }
    }

    /// The 8 data bytes; the reserved bits are ones.
    #[must_use]
    pub fn to_bytes(&self) -> [u8; 8] {
        let [s0, s1] = speed_raw(self.speed).to_le_bytes();
        let [d0, d1, d2, d3] = distance_raw(self.distance).to_le_bytes();
        let exit = 0xC0 | self.exit_reason.map_or(0x3F, |code| code.min(0x3F));
        let flags = self.direction.bits() | self.source.bits() << 2 | self.limit.bits() << 5;
        [s0, s1, d0, d1, d2, d3, exit, flags]
    }
}

/// Machine selected speed command, PGN 64835: a set point for the
/// tractor's speed and the limit it may go to.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct MachineSelectedSpeedCommand {
    /// Speed set point, mm/s resolution.
    pub set_point: Option<Speed>,
    /// Highest speed allowed, mm/s resolution.
    pub limit: Option<Speed>,
    /// Direction commanded.
    pub direction: Direction,
}

impl MachineSelectedSpeedCommand {
    pub(crate) const PGN_NUMBER: u32 = 64_835;
    /// PGN 64835.
    pub const PGN: Pgn = Pgn::new(Self::PGN_NUMBER);

    /// From the 8 data bytes.
    #[must_use]
    pub fn from_bytes(bytes: [u8; 8]) -> Self {
        let [_, _, _, _, _, _, _, flags] = bytes;
        Self {
            set_point: speed(u16_at(bytes, 0)),
            limit: speed(u16_at(bytes, 2)),
            direction: Direction::from_bits(flags),
        }
    }

    /// The 8 data bytes; the reserved bytes and bits are ones.
    #[must_use]
    pub fn to_bytes(&self) -> [u8; 8] {
        let [s0, s1] = speed_raw(self.set_point).to_le_bytes();
        let [l0, l1] = speed_raw(self.limit).to_le_bytes();
        [
            s0,
            s1,
            l0,
            l1,
            0xFF,
            0xFF,
            0xFF,
            0xFC | self.direction.bits(),
        ]
    }
}
