//! Sentence encoding with checksum.
//!
//! Each sentence type implements `Display` as its wire form. The body is
//! written through a [`Checksum`] adapter that XORs every byte, so the checksum
//! is computed in the same pass with no intermediate buffer.

use core::fmt::{self, Write};

use kinavis_kernel::{Latitude, Longitude, Position};

use crate::error::NmeaError;
use crate::frame::MAX_SENTENCE_BYTES;

/// A `fmt::Write` that XORs everything passing through it.
pub(crate) struct Checksum<'a, 'b> {
    inner: &'a mut fmt::Formatter<'b>,
    sum: u8,
}

impl Write for Checksum<'_, '_> {
    fn write_str(&mut self, text: &str) -> fmt::Result {
        for byte in text.bytes() {
            self.sum ^= byte;
        }
        self.inner.write_str(text)
    }
}

/// Writes `$`, the body produced by the closure, then `*hh`.
pub(crate) fn sentence(
    f: &mut fmt::Formatter<'_>,
    body: impl FnOnce(&mut Checksum<'_, '_>) -> fmt::Result,
) -> fmt::Result {
    framed(f, '$', body)
}

/// Writes `!`, the body produced by the closure, then `*hh` (encapsulation
/// sentence).
pub(crate) fn encapsulated(
    f: &mut fmt::Formatter<'_>,
    body: impl FnOnce(&mut Checksum<'_, '_>) -> fmt::Result,
) -> fmt::Result {
    framed(f, '!', body)
}

fn framed(
    f: &mut fmt::Formatter<'_>,
    start: char,
    body: impl FnOnce(&mut Checksum<'_, '_>) -> fmt::Result,
) -> fmt::Result {
    f.write_char(start)?;
    let mut checksum = Checksum { inner: f, sum: 0 };
    body(&mut checksum)?;
    let sum = checksum.sum;
    write!(f, "*{sum:02X}")
}

/// Writes an optional value, or nothing for `None`; the caller writes the
/// comma.
pub(crate) fn optional<T: fmt::Display>(out: &mut impl Write, value: Option<T>) -> fmt::Result {
    match value {
        Some(value) => write!(out, "{value}"),
        None => Ok(()),
    }
}

/// `ddmm.mmmm,N`, or two empty fields.
pub(crate) fn latitude(out: &mut impl Write, latitude: Option<Latitude>) -> fmt::Result {
    match latitude {
        Some(latitude) => {
            let (degrees, minutes) = degrees_minutes(latitude.degrees());
            let hemisphere = if latitude.degrees() < 0.0 { 'S' } else { 'N' };
            write!(out, "{degrees:02}{minutes:07.4},{hemisphere}")
        }
        None => out.write_char(','),
    }
}

/// `dddmm.mmmm,E`, or two empty fields.
pub(crate) fn longitude(out: &mut impl Write, longitude: Option<Longitude>) -> fmt::Result {
    match longitude {
        Some(longitude) => {
            let (degrees, minutes) = degrees_minutes(longitude.degrees());
            let hemisphere = if longitude.degrees() < 0.0 { 'W' } else { 'E' };
            write!(out, "{degrees:03}{minutes:07.4},{hemisphere}")
        }
        None => out.write_char(','),
    }
}

/// The four position fields; empty when there is no position.
pub(crate) fn position(out: &mut impl Write, position: Option<Position>) -> fmt::Result {
    latitude(out, position.map(Position::latitude))?;
    out.write_char(',')?;
    longitude(out, position.map(Position::longitude))
}

/// Whole degrees and decimal minutes of a magnitude, minutes rounded to four
/// places with carry at 60.
fn degrees_minutes(degrees: f64) -> (u32, f64) {
    let magnitude = kinavis_kernel::math::abs(degrees);
    let whole = kinavis_kernel::math::trunc(magnitude);
    let mut minutes = (magnitude - whole) * 60.0;
    // Round here so that 59.99996 carries instead of writing `60.0000`.
    minutes = kinavis_kernel::math::round(minutes * 10_000.0) / 10_000.0;
    let mut whole = kinavis_kernel::math::to_usize(whole);
    if minutes >= 60.0 {
        minutes = 0.0;
        whole = whole.saturating_add(1);
    }
    (u32::try_from(whole).unwrap_or(u32::MAX), minutes)
}

/// `fmt::Write` over a byte slice, used by [`encode`].
struct SliceWriter<'a> {
    out: &'a mut [u8],
    len: usize,
}

impl Write for SliceWriter<'_> {
    fn write_str(&mut self, text: &str) -> fmt::Result {
        let end = self.len.checked_add(text.len()).ok_or(fmt::Error)?;
        let slot = self.out.get_mut(self.len..end).ok_or(fmt::Error)?;
        slot.copy_from_slice(text.as_bytes());
        self.len = end;
        Ok(())
    }
}

/// `fmt::Write` that only counts bytes, used by [`encode`].
struct Counter(usize);

impl Write for Counter {
    fn write_str(&mut self, text: &str) -> fmt::Result {
        self.0 = self.0.saturating_add(text.len());
        Ok(())
    }
}

/// Writes a sentence (`$` to `CR LF`) into `out` and returns its length.
///
/// A sentence longer than [`MAX_SENTENCE_BYTES`] is refused, never written.
/// Of the sentences [`parse`](crate::parse) accepts, only a GGA with several
/// fields near their plausibility bounds can exceed it.
///
/// # Errors
///
/// [`NmeaError::TooLong`] if the sentence exceeds [`MAX_SENTENCE_BYTES`];
/// [`NmeaError::BufferTooSmall`] if it does not fit in `out`.
pub fn encode(sentence: &impl fmt::Display, out: &mut [u8]) -> Result<usize, NmeaError> {
    let mut counter = Counter(0);
    write!(counter, "{sentence}\r\n").map_err(|_| NmeaError::BufferTooSmall)?;
    if counter.0 > MAX_SENTENCE_BYTES {
        return Err(NmeaError::TooLong {
            length: counter.0,
            limit: MAX_SENTENCE_BYTES,
        });
    }
    let mut writer = SliceWriter { out, len: 0 };
    write!(writer, "{sentence}\r\n").map_err(|_| NmeaError::BufferTooSmall)?;
    Ok(writer.len)
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;

    #[test]
    fn minutes_that_round_to_sixty_carry_into_the_degrees() {
        assert_eq!(degrees_minutes(49.0 + 59.99996 / 60.0), (50, 0.0));
        assert_eq!(degrees_minutes(49.0 + 16.45 / 60.0), (49, 16.45));
    }
}
