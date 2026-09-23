//! CAN frame and reassembled payload.

use core::fmt;

use crate::error::Nmea2000Error;
use crate::id::{CanId, Pgn};

/// Maximum fast-packet payload: 6 bytes in the first frame + 7 in each of up to
/// 31 more.
pub const MAX_PAYLOAD_BYTES: usize = 223;

/// CAN frame: identifier and up to 8 data bytes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(try_from = "StoredFrame", into = "StoredFrame")
)]
pub struct Frame {
    /// 29-bit identifier.
    pub id: CanId,
    data: [u8; 8],
    len: u8,
}

impl Frame {
    /// Frame from identifier and data.
    ///
    /// # Errors
    ///
    /// [`Nmea2000Error::BadFrameLength`] for more than 8 bytes.
    pub fn new(id: CanId, data: &[u8]) -> Result<Self, Nmea2000Error> {
        if data.len() > 8 {
            return Err(Nmea2000Error::BadFrameLength { len: data.len() });
        }
        let mut bytes = [0xFF; 8];
        for (slot, byte) in bytes.iter_mut().zip(data) {
            *slot = *byte;
        }
        // At most 8.
        #[allow(clippy::cast_possible_truncation)]
        Ok(Self {
            id,
            data: bytes,
            len: data.len() as u8,
        })
    }

    /// Data bytes.
    #[must_use]
    pub fn data(&self) -> &[u8] {
        self.data.get(..usize::from(self.len)).unwrap_or(&[])
    }
}

/// Complete PGN payload (single frame or reassembled fast packet), fixed
/// capacity.
///
/// Fields are read by byte offset, little-endian; a field past the end reads as
/// `None`.
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub struct Payload {
    pgn: Pgn,
    source: u8,
    bytes: [u8; MAX_PAYLOAD_BYTES],
    len: usize,
}

/// Serialised frame: identifier, 8 bytes and the length. Deserialisation goes
/// through [`Frame::new`], so a log cannot carry an invalid length.
#[cfg(feature = "serde")]
#[derive(serde::Serialize, serde::Deserialize)]
struct StoredFrame {
    id: CanId,
    data: [u8; 8],
    len: u8,
}

#[cfg(feature = "serde")]
impl TryFrom<StoredFrame> for Frame {
    type Error = Nmea2000Error;

    fn try_from(stored: StoredFrame) -> Result<Self, Nmea2000Error> {
        let data =
            stored
                .data
                .get(..usize::from(stored.len))
                .ok_or(Nmea2000Error::BadFrameLength {
                    len: usize::from(stored.len),
                })?;
        Self::new(stored.id, data)
    }
}

#[cfg(feature = "serde")]
impl From<Frame> for StoredFrame {
    fn from(frame: Frame) -> Self {
        Self {
            id: frame.id,
            data: frame.data,
            len: frame.len,
        }
    }
}

impl Payload {
    /// Empty payload for a PGN from a source.
    #[must_use]
    pub const fn new(pgn: Pgn, source: u8) -> Self {
        Self {
            pgn,
            source,
            bytes: [0; MAX_PAYLOAD_BYTES],
            len: 0,
        }
    }

    /// Payload with the given bytes.
    ///
    /// # Errors
    ///
    /// [`Nmea2000Error::BadLength`] for more than [`MAX_PAYLOAD_BYTES`].
    pub fn from_bytes(pgn: Pgn, source: u8, bytes: &[u8]) -> Result<Self, Nmea2000Error> {
        let mut payload = Self::new(pgn, source);
        payload.append(bytes)?;
        Ok(payload)
    }

    /// Appends bytes; on error nothing is appended.
    ///
    /// # Errors
    ///
    /// [`Nmea2000Error::BadLength`] beyond [`MAX_PAYLOAD_BYTES`].
    pub fn append(&mut self, bytes: &[u8]) -> Result<(), Nmea2000Error> {
        let end = self.len.saturating_add(bytes.len());
        let Some(slots) = self.bytes.get_mut(self.len..end) else {
            // The declared length is at most one byte; this is reached only
            // through it or a caller's slice.
            #[allow(clippy::cast_possible_truncation)]
            return Err(Nmea2000Error::BadLength {
                declared: end.min(255) as u8,
                limit: MAX_PAYLOAD_BYTES,
            });
        };
        slots.copy_from_slice(bytes);
        self.len = end;
        Ok(())
    }

    /// Truncates to `len` bytes.
    pub(crate) fn truncate(&mut self, len: usize) {
        if len < self.len {
            for byte in self.bytes.iter_mut().skip(len) {
                *byte = 0;
            }
            self.len = len;
        }
    }

    /// PGN.
    #[must_use]
    pub const fn pgn(&self) -> Pgn {
        self.pgn
    }

    /// Source address.
    #[must_use]
    pub const fn source(&self) -> u8 {
        self.source
    }

    /// Bytes.
    #[must_use]
    pub fn bytes(&self) -> &[u8] {
        self.bytes.get(..self.len).unwrap_or(&[])
    }

    /// Length.
    #[must_use]
    pub const fn len(&self) -> usize {
        self.len
    }

    /// Whether empty.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Byte at `offset`; `None` past the end.
    #[must_use]
    pub fn u8(&self, offset: usize) -> Option<u8> {
        self.bytes().get(offset).copied()
    }

    /// Unsigned little-endian field of `width ≤ 8` bytes at `offset`; `None`
    /// past the end.
    #[must_use]
    pub fn unsigned(&self, offset: usize, width: usize) -> Option<u64> {
        if width > 8 {
            return None;
        }
        let end = offset.checked_add(width)?;
        let bytes = self.bytes().get(offset..end)?;
        let mut value = 0_u64;
        for &byte in bytes.iter().rev() {
            value = (value << 8) | u64::from(byte);
        }
        Some(value)
    }

    /// Two's-complement little-endian field of `width ≤ 8` bytes at `offset`;
    /// `None` past the end.
    #[must_use]
    pub fn signed(&self, offset: usize, width: usize) -> Option<i64> {
        if width == 0 {
            return None;
        }
        let raw = self.unsigned(offset, width)?;
        // `unsigned` rejects widths above 8; bound repeated so the compiler
        // sees the shift stays in range.
        let shift = 64 - 8 * width.min(8);
        // Shift to the top and back to sign-extend.
        #[allow(clippy::cast_possible_wrap)]
        Some(((raw << shift) as i64) >> shift)
    }

    /// Bit field of `width` bits at bit `bit`, counted from the LSB of byte 0
    /// upward as the standard packs them; `None` past the end or if wider than
    /// 32 bits.
    #[must_use]
    pub fn bits(&self, bit: usize, width: usize) -> Option<u32> {
        if width > 32 || width == 0 {
            return None;
        }
        let end = bit.checked_add(width)?;
        // Length ≤ `MAX_PAYLOAD_BYTES`; bound repeated so the compiler sees the
        // bit count cannot wrap.
        if end > self.len.min(MAX_PAYLOAD_BYTES) * 8 {
            return None;
        }
        let mut value = 0_u32;
        for index in (bit..end).rev() {
            let byte = self.u8(index / 8)?;
            value = (value << 1) | u32::from((byte >> (index % 8)) & 1);
        }
        Some(value)
    }
}

impl fmt::Debug for Payload {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Payload({} from {}, {} bytes)",
            self.pgn, self.source, self.len
        )
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use super::*;

    fn payload(bytes: &[u8]) -> Payload {
        Payload::from_bytes(Pgn::new(1), 0, bytes).unwrap()
    }

    #[test]
    fn fields_are_little_endian() {
        let payload = payload(&[0x01, 0x02, 0x03, 0x04, 0xFF, 0xFF, 0xFE, 0x7F]);
        assert_eq!(payload.u8(0), Some(1));
        assert_eq!(payload.unsigned(0, 2), Some(0x0201));
        assert_eq!(payload.unsigned(0, 4), Some(0x0403_0201));
        assert_eq!(payload.signed(4, 2), Some(-1));
        assert_eq!(payload.signed(6, 2), Some(0x7FFE));
        assert_eq!(payload.unsigned(0, 8), Some(0x7FFE_FFFF_0403_0201));
        assert_eq!(payload.signed(0, 8), Some(0x7FFE_FFFF_0403_0201));
        assert_eq!(payload.unsigned(7, 2), None);
        assert_eq!(payload.unsigned(0, 9), None);
        assert_eq!(payload.signed(0, 0), None);
        assert_eq!(payload.u8(8), None);
    }

    #[test]
    fn bit_fields_run_from_the_low_bit_of_the_first_byte() {
        // 0b1011_0110: bits 0..2 = 0b10 (2), bits 2..6 = 0b1101 (13),
        // bits 6..8 = 0b10 (2); the next byte's bits 8..12 = 0x5.
        let payload = payload(&[0b1011_0110, 0x35]);
        assert_eq!(payload.bits(0, 2), Some(2));
        assert_eq!(payload.bits(2, 4), Some(13));
        assert_eq!(payload.bits(6, 2), Some(2));
        assert_eq!(payload.bits(8, 4), Some(5));
        // Across a byte boundary: bits 6..12 = 0b0101_10 = 22.
        assert_eq!(payload.bits(6, 6), Some(22));
        assert_eq!(payload.bits(15, 2), None);
        assert_eq!(payload.bits(0, 33), None);
        assert_eq!(payload.bits(0, 0), None);
    }

    #[test]
    fn a_frame_and_a_payload_refuse_more_than_they_hold() {
        let id = CanId::from_parts(2, Pgn::new(1), 0, 0);
        assert!(Frame::new(id, &[0; 8]).is_ok());
        assert_eq!(
            Frame::new(id, &[0; 9]),
            Err(Nmea2000Error::BadFrameLength { len: 9 })
        );
        assert_eq!(Frame::new(id, &[1, 2]).unwrap().data(), &[1, 2]);

        let mut payload = Payload::new(Pgn::new(1), 0);
        payload.append(&[0xAA; MAX_PAYLOAD_BYTES]).unwrap();
        assert_eq!(payload.len(), MAX_PAYLOAD_BYTES);
        assert_eq!(
            payload.append(&[0]),
            Err(Nmea2000Error::BadLength {
                declared: 224,
                limit: MAX_PAYLOAD_BYTES
            })
        );
        assert_eq!(
            payload.len(),
            MAX_PAYLOAD_BYTES,
            "nothing appended on an error"
        );
        payload.truncate(3);
        assert_eq!(payload.bytes(), &[0xAA; 3]);
        assert_eq!(payload.u8(3), None);
        assert_eq!(
            std::format!("{payload:?}"),
            "Payload(PGN 1 from 0, 3 bytes)"
        );
    }
}
