//! Field readers over a length-checked message, with conversion to kernel
//! types.

use kinavis_kernel::angle::TrueCourse;
use kinavis_kernel::position::Position;
use kinavis_kernel::units::Speed;
use kinavis_kernel::{InlineStr, KernelError};

use crate::bits::Bits;
use crate::error::AisError;

/// Field values meaning "not available".
const NO_SPEED: u32 = 1023;
const NO_LONGITUDE: i32 = 181 * 600_000;
const NO_LATITUDE: i32 = 91 * 600_000;
const NO_COURSE: u32 = 3600;
const NO_HEADING: u32 = 511;

/// Message of at least `needed` bits, so fields are read without per-field
/// length checks.
pub(crate) struct Fields<'a> {
    bits: &'a Bits,
}

impl<'a> Fields<'a> {
    pub(crate) fn of(bits: &'a Bits, needed: usize) -> Result<Self, AisError> {
        if bits.len() < needed {
            return Err(AisError::TooShort {
                bits: bits.len(),
                needed,
            });
        }
        Ok(Self { bits })
    }

    /// Message length in bits, for optional fields beyond `needed`.
    pub(crate) fn len(&self) -> usize {
        self.bits.len()
    }

    pub(crate) fn unsigned(&self, offset: usize, width: usize) -> u32 {
        self.bits.unsigned(offset, width).unwrap_or(0)
    }

    pub(crate) fn signed(&self, offset: usize, width: usize) -> i32 {
        self.bits.signed(offset, width).unwrap_or(0)
    }

    pub(crate) fn bit(&self, offset: usize) -> bool {
        self.bits.bit(offset).unwrap_or(false)
    }

    /// Text field; `None` if entirely padding (unset, not empty).
    pub(crate) fn text<const N: usize>(&self, offset: usize, chars: usize) -> Option<InlineStr<N>> {
        self.bits
            .text::<N>(offset, chars)
            .filter(|text| !text.as_str().is_empty())
    }

    /// Text field at `offsets`; `None` if entirely padding or past the end.
    pub(crate) fn text_at<const N: usize>(
        &self,
        offsets: impl Iterator<Item = usize>,
    ) -> Option<InlineStr<N>> {
        self.bits
            .text_at::<N>(offsets)
            .filter(|text| !text.as_str().is_empty())
    }
}

/// Counts per unit, per M.1371. Named once so a power-of-ten error is a
/// one-line fix.
pub(crate) mod counts_per {
    /// SOG and COG: tenths.
    pub(crate) const TENTH: f64 = 10.0;
    /// Latitude and longitude: 1/10 000 minute, 600 000 per degree.
    pub(crate) const DEGREE: f64 = 600_000.0;
}

pub(crate) fn value_of(field: &'static str) -> impl Fn(KernelError) -> AisError {
    move |error| AisError::Value { field, error }
}

pub(crate) fn speed(field: u32) -> Result<Option<Speed>, AisError> {
    if field == NO_SPEED {
        return Ok(None);
    }
    Speed::from_knots(f64::from(field) / counts_per::TENTH)
        .map(Some)
        .map_err(value_of("speed"))
}

pub(crate) fn position(longitude: i32, latitude: i32) -> Result<Option<Position>, AisError> {
    if longitude == NO_LONGITUDE || latitude == NO_LATITUDE {
        return Ok(None);
    }
    Position::from_degrees(
        f64::from(latitude) / counts_per::DEGREE,
        f64::from(longitude) / counts_per::DEGREE,
    )
    .map(Some)
    .map_err(value_of("position"))
}

pub(crate) fn course(field: u32) -> Result<Option<TrueCourse>, AisError> {
    if field >= NO_COURSE {
        return Ok(None);
    }
    TrueCourse::new(f64::from(field) / counts_per::TENTH)
        .map(Some)
        .map_err(value_of("course"))
}

pub(crate) fn heading(field: u32) -> Result<Option<TrueCourse>, AisError> {
    if field == NO_HEADING {
        return Ok(None);
    }
    TrueCourse::new(f64::from(field))
        .map(Some)
        .map_err(value_of("heading"))
}

pub(crate) fn second(field: u32) -> Option<u8> {
    // Six bits.
    #[allow(clippy::cast_possible_truncation)]
    (field <= 59).then_some(field as u8)
}

/// 6-bit code as `u8`.
pub(crate) const fn byte(field: u32) -> u8 {
    // Callers read at most 8 bits.
    #[allow(clippy::cast_possible_truncation)]
    {
        field as u8
    }
}
