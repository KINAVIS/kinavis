//! Value slots of J1939: two-bit states and the ranges of scaled values.

use kinavis_kernel::{Distance, Speed};

/// Largest valid one-byte value; above it are error and not-available codes.
pub(crate) const MAX_U8: u8 = 250;
/// Largest valid two-byte value.
pub(crate) const MAX_U16: u16 = 0xFAFF;
/// Largest valid four-byte value.
pub(crate) const MAX_U32: u32 = 0xFAFF_FFFF;

/// Direction of travel.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Direction {
    /// Reverse.
    Reverse,
    /// Forward.
    Forward,
    /// Sensor error.
    Error,
    /// Not available.
    NotAvailable,
}

impl Direction {
    pub(crate) const fn from_bits(bits: u8) -> Self {
        match bits & 0x03 {
            0 => Self::Reverse,
            1 => Self::Forward,
            2 => Self::Error,
            _ => Self::NotAvailable,
        }
    }

    pub(crate) const fn bits(self) -> u8 {
        match self {
            Self::Reverse => 0,
            Self::Forward => 1,
            Self::Error => 2,
            Self::NotAvailable => 3,
        }
    }
}

/// A two-bit state of J1939: off, on, error, not available.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Indicator {
    /// Off, disabled, passive, not required.
    Off,
    /// On, enabled, active, required.
    On,
    /// Error.
    Error,
    /// Not available.
    NotAvailable,
}

impl Indicator {
    pub(crate) const fn from_bits(bits: u8) -> Self {
        match bits & 0x03 {
            0 => Self::Off,
            1 => Self::On,
            2 => Self::Error,
            _ => Self::NotAvailable,
        }
    }

    pub(crate) const fn bits(self) -> u8 {
        match self {
            Self::Off => 0,
            Self::On => 1,
            Self::Error => 2,
            Self::NotAvailable => 3,
        }
    }
}

/// Speed of a two-byte field in mm/s.
pub(crate) fn speed(raw: u16) -> Option<Speed> {
    (raw <= MAX_U16)
        .then(|| Speed::from_metres_per_second(f64::from(raw) / 1000.0).ok())
        .flatten()
}

/// Two-byte field in mm/s of a speed; not available for none, the largest
/// valid value for one beyond it, zero for a negative one.
pub(crate) fn speed_raw(speed: Option<Speed>) -> u16 {
    speed.map_or(u16::MAX, |speed| {
        let millimetres = kinavis_kernel::math::round(speed.metres_per_second() * 1000.0);
        let bounded = millimetres.max(0.0).min(f64::from(MAX_U16));
        u16::try_from(kinavis_kernel::math::to_usize(bounded)).unwrap_or(MAX_U16)
    })
}

/// Distance of a four-byte field in mm.
pub(crate) fn distance(raw: u32) -> Option<Distance> {
    (raw <= MAX_U32)
        .then(|| Distance::from_metres(f64::from(raw) / 1000.0).ok())
        .flatten()
}

/// Four-byte field in mm of a distance, as [`speed_raw`].
pub(crate) fn distance_raw(distance: Option<Distance>) -> u32 {
    distance.map_or(u32::MAX, |distance| {
        let millimetres = kinavis_kernel::math::round(distance.metres() * 1000.0);
        let bounded = millimetres.max(0.0).min(f64::from(MAX_U32));
        // Within u32 by the bound; `to_usize` saturates at 10⁹, so split.
        let high = kinavis_kernel::math::trunc(bounded / 65_536.0);
        let low = bounded - high * 65_536.0;
        let high = u32::try_from(kinavis_kernel::math::to_usize(high)).unwrap_or(0);
        let low = u32::try_from(kinavis_kernel::math::to_usize(low)).unwrap_or(0);
        high.saturating_mul(65_536).saturating_add(low).min(MAX_U32)
    })
}

pub(crate) const fn u16_at(bytes: [u8; 8], at: usize) -> u16 {
    let (low, high) = match at {
        0 => (bytes[0], bytes[1]),
        _ => (bytes[2], bytes[3]),
    };
    u16::from_le_bytes([low, high])
}

pub(crate) const fn u32_at_2(bytes: [u8; 8]) -> u32 {
    u32::from_le_bytes([bytes[2], bytes[3], bytes[4], bytes[5]])
}
