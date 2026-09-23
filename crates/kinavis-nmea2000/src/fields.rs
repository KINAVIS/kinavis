//! Field readers over a length-checked payload, with conversion to kernel
//! types.
//!
//! The standard reserves the top of each field's range: all ones is "not
//! available", one less is "error/out of range". Both read as `None`.

use kinavis_kernel::angle::{Direction, Frame as AngleFrame};
use kinavis_kernel::math;
use kinavis_kernel::position::Position;
use kinavis_kernel::units::{Distance, Speed};
use kinavis_kernel::KernelError;

use crate::error::Nmea2000Error;
use crate::frame::Payload;

/// Field resolutions (unit per count), named once so PGN definitions read like
/// the standard and a power-of-ten error is a one-line fix.
pub(crate) mod resolution {
    /// Position in rapid-update and AIS PGNs: 1e-7° per count.
    pub(crate) const POSITION_DEG: f64 = 1e-7;
    /// Position in GNSS position data: 1e-16° per count.
    pub(crate) const PRECISE_POSITION_DEG: f64 = 1e-16;
    /// Altitude in GNSS position data: 1e-6 m per count.
    pub(crate) const ALTITUDE_M: f64 = 1e-6;
    /// Angles: 1e-4 rad per count.
    pub(crate) const ANGLE_RAD: f64 = 1e-4;
    /// Speeds: 0.01 m/s per count.
    pub(crate) const SPEED_M_PER_S: f64 = 0.01;
    /// Centimetre distances: depth, geoidal separation.
    pub(crate) const CENTIMETRE_M: f64 = 0.01;
    /// Millimetre distances: transducer offset.
    pub(crate) const MILLIMETRE_M: f64 = 0.001;
    /// Depth range: 10 m per count.
    pub(crate) const DEPTH_RANGE_M: f64 = 10.0;
    /// Dilution of precision: 0.01 per count.
    pub(crate) const DOP: f64 = 0.01;
    /// AIS rate of turn: 3.125e-5 rad/s per count.
    pub(crate) const RATE_OF_TURN_RAD_PER_S: f64 = 3.125e-5;
}

/// Payload of at least `needed` bytes, so fields are read without per-field
/// length checks.
pub(crate) struct Fields<'a> {
    payload: &'a Payload,
}

impl<'a> Fields<'a> {
    pub(crate) fn of(payload: &'a Payload, needed: usize) -> Result<Self, Nmea2000Error> {
        if payload.len() < needed {
            return Err(Nmea2000Error::TooShort {
                pgn: payload.pgn(),
                bytes: payload.len(),
                needed,
            });
        }
        Ok(Self { payload })
    }

    /// Byte; `None` for 0xFF and 0xFE.
    pub(crate) fn u8(&self, offset: usize) -> Option<u8> {
        self.payload.u8(offset).filter(|&value| value < 0xFE)
    }

    /// Unsigned field; `None` for the top two values.
    pub(crate) fn unsigned(&self, offset: usize, width: usize) -> Option<u64> {
        let value = self.payload.unsigned(offset, width)?;
        // `payload.unsigned` rejects widths above 8, so the shift is below 64;
        // bound repeated for the compiler.
        let max = if width >= 8 {
            u64::MAX
        } else {
            (1_u64 << (8 * width.min(7))) - 1
        };
        (value < max - 1).then_some(value)
    }

    /// Signed field; `None` for the top two values.
    pub(crate) fn signed(&self, offset: usize, width: usize) -> Option<i64> {
        let value = self.payload.signed(offset, width)?;
        // `payload.signed` rejects widths of 0 or above 8.
        let max = if width >= 8 {
            i64::MAX
        } else {
            (1_i64 << (8 * width.clamp(1, 7) - 1)) - 1
        };
        (value < max - 1).then_some(value)
    }

    /// Bit field; `None` when all ones.
    pub(crate) fn bits(&self, bit: usize, width: usize) -> Option<u32> {
        let value = self.payload.bits(bit, width)?;
        // `payload.bits` rejects widths of 0 or above 32. A 32-bit mask cannot
        // be built with a shift by 32, so it is derived from the width minus
        // one.
        let all_ones = u32::MAX >> (32 - width.clamp(1, 32));
        (value != all_ones).then_some(value)
    }

    /// Bit field, raw.
    pub(crate) fn raw_bits(&self, bit: usize, width: usize) -> u32 {
        self.payload.bits(bit, width).unwrap_or(0)
    }

    /// One-bit flag.
    pub(crate) fn flag(&self, bit: usize) -> bool {
        self.payload.bits(bit, 1) == Some(1)
    }
}

pub(crate) fn value_of(field: &'static str) -> impl Fn(KernelError) -> Nmea2000Error {
    move |error| Nmea2000Error::Value { field, error }
}

/// Latitude and longitude in 1e-7° (4 bytes each) or 1e-16° (8 bytes each).
pub(crate) fn position(
    latitude: Option<i64>,
    longitude: Option<i64>,
    scale: f64,
) -> Result<Option<Position>, Nmea2000Error> {
    let (Some(latitude), Some(longitude)) = (latitude, longitude) else {
        return Ok(None);
    };
    // i64 → f64: precision loss far below the field resolution.
    #[allow(clippy::cast_precision_loss)]
    Position::from_degrees(latitude as f64 * scale, longitude as f64 * scale)
        .map(Some)
        .map_err(value_of("position"))
}

/// Angle in 1e-4 rad as a direction in a frame.
///
/// Range 0..6.5534 rad, slightly over a full turn; values past 360° wrap as
/// intended by the standard, so no value is rejected.
pub(crate) fn direction<F: AngleFrame>(field: Option<u64>) -> Option<Direction<F>> {
    // At most 16 bits.
    #[allow(clippy::cast_precision_loss)]
    let degrees = math::to_degrees(field? as f64 * resolution::ANGLE_RAD);
    Some(Direction::from_degrees_wrapped(degrees))
}

/// Speed in 0.01 m/s.
pub(crate) fn speed(field: Option<u64>) -> Result<Option<Speed>, Nmea2000Error> {
    let Some(field) = field else {
        return Ok(None);
    };
    // u64 → f64: precision loss far below the field resolution.
    #[allow(clippy::cast_precision_loss)]
    Speed::from_metres_per_second(field as f64 * resolution::SPEED_M_PER_S)
        .map(Some)
        .map_err(value_of("speed"))
}

/// Distance in units of `resolution` metres.
pub(crate) fn distance(
    field: Option<i64>,
    resolution: f64,
    name: &'static str,
) -> Result<Option<Distance>, Nmea2000Error> {
    let Some(field) = field else {
        return Ok(None);
    };
    // i64 → f64: precision loss far below the field resolution.
    #[allow(clippy::cast_precision_loss)]
    Distance::from_metres(field as f64 * resolution)
        .map(Some)
        .map_err(value_of(name))
}

/// AIS 6-bit UTC second: `0..=59`; `None` for 60+ (no time available).
pub(crate) fn second(field: u32) -> Option<u8> {
    // Six bits.
    #[allow(clippy::cast_possible_truncation)]
    (field <= 59).then_some(field as u8)
}
