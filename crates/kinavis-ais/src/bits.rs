//! AIS payload unarmouring and bit-field access.
//!
//! Each payload character carries 6 bits: ASCII code − 48, minus a further 8 if
//! the result exceeds 40 (skipping characters a sentence cannot carry). The
//! message is the concatenation of the characters' bits, MSB first, minus the
//! fill bits of the last fragment.
//!
//! Text uses the standard's 6-bit alphabet: `@` and `A`–`_` for codes 0–31,
//! space to `?` for 32–63; `@` pads fields shorter than their width.

use core::fmt;

use kinavis_kernel::InlineStr;

use crate::error::AisError;

/// Maximum message length in bits.
///
/// The longest defined message (multi-slot binary) is 1064 bits; the buffer
/// rounds up to whole bytes.
pub const MAX_MESSAGE_BITS: usize = MAX_MESSAGE_BYTES * 8;

const MAX_MESSAGE_BYTES: usize = 136;

/// Message bits in a fixed-capacity buffer.
///
/// Fields are read by bit offset and width as in the standard's tables; a field
/// past the end reads as `None`.
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub struct Bits {
    /// Bits past `len` are always zero, so derived comparisons see only the
    /// message.
    bytes: [u8; MAX_MESSAGE_BYTES],
    len: usize,
}

impl Default for Bits {
    fn default() -> Self {
        Self::new()
    }
}

impl Bits {
    /// Empty.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            bytes: [0; MAX_MESSAGE_BYTES],
            len: 0,
        }
    }

    /// Bits of one armoured payload, minus fill bits.
    ///
    /// # Errors
    ///
    /// [`AisError::BadArmouring`] for a character outside the alphabet;
    /// [`AisError::BadFill`] for more fill bits than bits;
    /// [`AisError::TooLong`] beyond [`MAX_MESSAGE_BITS`].
    pub fn unarmour(payload: &[u8], fill_bits: u8) -> Result<Self, AisError> {
        let mut bits = Self::new();
        bits.append(payload, fill_bits)?;
        Ok(bits)
    }

    /// Appends one armoured payload, minus fill bits. On error nothing is
    /// appended.
    ///
    /// # Errors
    ///
    /// As [`Bits::unarmour`].
    pub fn append(&mut self, payload: &[u8], fill_bits: u8) -> Result<(), AisError> {
        // 6 bits per character; a payload long enough to wrap is rejected as
        // too long immediately after, so saturation loses nothing.
        let payload_bits = payload.len().saturating_mul(6);
        if usize::from(fill_bits) > payload_bits {
            return Err(AisError::BadFill {
                fill_bits,
                payload_bits,
            });
        }
        let added = payload_bits - usize::from(fill_bits);
        let end = self.len.saturating_add(added);
        if end > MAX_MESSAGE_BITS {
            return Err(AisError::TooLong {
                bits: end,
                limit: MAX_MESSAGE_BITS,
            });
        }
        if let Some(offset) = payload.iter().position(|&byte| unarmour(byte).is_none()) {
            return Err(AisError::BadArmouring { offset });
        }

        let mut at = self.len;
        for value in payload.iter().map(|&byte| unarmour(byte).unwrap_or(0)) {
            for shift in (0..6).rev() {
                if at == end {
                    break;
                }
                if (value >> shift) & 1 == 1 {
                    if let Some(byte) = self.bytes.get_mut(at / 8) {
                        *byte |= 0x80 >> (at % 8);
                    }
                }
                // Below `end ≤ MAX_MESSAGE_BITS`.
                at = at.saturating_add(1);
            }
        }
        self.len = end;
        Ok(())
    }

    /// Length in bits.
    #[must_use]
    pub const fn len(&self) -> usize {
        self.len
    }

    /// Whether empty.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Bit at `offset`; `None` past the end.
    #[must_use]
    pub fn bit(&self, offset: usize) -> Option<bool> {
        if offset >= self.len {
            return None;
        }
        self.bytes
            .get(offset / 8)
            .map(|byte| (byte >> (7 - offset % 8)) & 1 == 1)
    }

    /// Unsigned field of `width` bits at `offset`, MSB first.
    ///
    /// `None` past the end or if wider than 32 bits.
    #[must_use]
    pub fn unsigned(&self, offset: usize, width: usize) -> Option<u32> {
        if width > 32 || offset.checked_add(width)? > self.len {
            return None;
        }
        let mut value = 0;
        for at in offset..offset + width {
            value = (value << 1) | u32::from(self.bit(at)?);
        }
        Some(value)
    }

    /// Two's-complement field of `width` bits at `offset`.
    ///
    /// `None` past the end, or if wider than 32 or narrower than 1 bit.
    #[must_use]
    pub fn signed(&self, offset: usize, width: usize) -> Option<i32> {
        if width == 0 {
            return None;
        }
        let raw = self.unsigned(offset, width)?;
        let sign = 1u32 << (width - 1);
        // Sign extension: a set sign bit subtracts `2^width`.
        #[allow(clippy::cast_possible_wrap)]
        Some(if raw & sign == 0 {
            raw as i32
        } else {
            (raw as i32).wrapping_sub((sign as i32).wrapping_mul(2))
        })
    }

    /// Text field of `chars` 6-bit characters at `offset`, in the AIS alphabet.
    ///
    /// Terminated at the first `@` (padding); trailing spaces are trimmed;
    /// truncated to `N` bytes (one byte per character). Empty text is an empty
    /// string; `None` only if the field runs past the end.
    #[must_use]
    pub fn text<const N: usize>(&self, offset: usize, chars: usize) -> Option<InlineStr<N>> {
        let end = offset.checked_add(chars.checked_mul(6)?)?;
        if end > self.len {
            return None;
        }
        self.text_at((0..chars).map(|index| offset + index * 6))
    }

    /// Text from 6-bit characters at possibly non-contiguous `offsets` (e.g.
    /// name plus extension).
    ///
    /// `None` if any character runs past the end.
    pub(crate) fn text_at<const N: usize>(
        &self,
        offsets: impl Iterator<Item = usize>,
    ) -> Option<InlineStr<N>> {
        let mut buffer = [0_u8; N];
        let mut len = 0;
        for (slot, offset) in buffer.iter_mut().zip(offsets) {
            // 6 bits, 0..64: exact cast.
            #[allow(clippy::cast_possible_truncation)]
            let code = self.unsigned(offset, 6)? as u8;
            if code == 0 {
                break;
            }
            *slot = if code < 32 { code + 64 } else { code };
            len += 1;
        }
        while len > 0 && buffer.get(len - 1) == Some(&b' ') {
            len -= 1;
        }
        let text = core::str::from_utf8(buffer.get(..len)?).ok()?;
        Some(InlineStr::new(text))
    }
}

/// 6-bit value of a payload character; `None` outside the alphabet.
const fn unarmour(byte: u8) -> Option<u8> {
    match byte {
        b'0'..=b'W' => Some(byte - 48),
        b'`'..=b'w' => Some(byte - 56),
        _ => None,
    }
}

impl fmt::Debug for Bits {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Bits({} bits)", self.len)
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use super::*;

    #[test]
    fn the_alphabet_is_the_standards() {
        assert_eq!(unarmour(b'0'), Some(0));
        assert_eq!(unarmour(b'W'), Some(39));
        assert_eq!(unarmour(b'`'), Some(40));
        assert_eq!(unarmour(b'w'), Some(63));
        assert_eq!(unarmour(b'X'), None);
        assert_eq!(unarmour(b'x'), None);
        assert_eq!(unarmour(b' '), None);
    }

    #[test]
    fn fields_read_most_significant_bit_first() {
        // `1` is 000001, `w` is 111111.
        let bits = Bits::unarmour(b"1w", 0).unwrap();
        assert_eq!(bits.len(), 12);
        assert_eq!(bits.unsigned(0, 6), Some(1));
        assert_eq!(bits.unsigned(6, 6), Some(63));
        assert_eq!(bits.unsigned(5, 2), Some(3));
        assert_eq!(bits.unsigned(0, 12), Some(0b0000_0111_1111));
        assert_eq!(bits.unsigned(7, 6), None);
        assert_eq!(bits.unsigned(0, 33), None);
    }

    #[test]
    fn signed_fields_are_twos_complement() {
        let bits = Bits::unarmour(b"1w", 0).unwrap();
        assert_eq!(bits.signed(6, 6), Some(-1));
        assert_eq!(bits.signed(0, 6), Some(1));
        assert_eq!(bits.signed(5, 2), Some(-1));
        assert_eq!(bits.signed(0, 0), None);
        // Full-width field: sign bit is bit 31.
        let bits = Bits::unarmour(b"w00000", 0).unwrap();
        assert_eq!(bits.signed(0, 32), Some(-67_108_864));
    }

    #[test]
    fn fill_bits_come_off_the_end_and_stay_zero() {
        let bits = Bits::unarmour(b"ww", 2).unwrap();
        assert_eq!(bits.len(), 10);
        assert_eq!(bits.unsigned(0, 10), Some(0x3FF));
        assert_eq!(bits.bit(10), None);
        assert_eq!(bits, Bits::unarmour(b"wt", 2).unwrap());
        assert_eq!(
            Bits::unarmour(b"", 1),
            Err(AisError::BadFill {
                fill_bits: 1,
                payload_bits: 0
            })
        );
    }

    #[test]
    fn appending_continues_where_the_last_fragment_ended() {
        let mut bits = Bits::unarmour(b"1", 0).unwrap();
        bits.append(b"w", 0).unwrap();
        assert_eq!(bits, Bits::unarmour(b"1w", 0).unwrap());
        assert_eq!(
            bits.append(b"1X", 0),
            Err(AisError::BadArmouring { offset: 1 })
        );
        assert_eq!(bits.len(), 12, "nothing appended on an error");
    }

    #[test]
    fn text_is_the_standards_alphabet_cut_at_the_padding() {
        // `@` is 0, `A` is 1 … `_` is 31; space is 32 … `?` is 63.
        let mut bits = Bits::new();
        for code in 0..64_u8 {
            let armoured = if code < 40 { code + 48 } else { code + 56 };
            bits.append(&[armoured], 0).unwrap();
        }
        let alphabet = bits.text::<64>(0, 64).unwrap();
        assert_eq!(alphabet, "");
        let alphabet = bits.text::<64>(6, 63).unwrap();
        assert_eq!(
            alphabet,
            "ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_ !\"#$%&'()*+,-./0123456789:;<=>?"
        );

        // `MT.MITCHELL@@@@@@@@@`, as in a type 5 name field.
        let bits = Bits::unarmour(b"5P5TL01VIaAL@7WKO@mBplU@<PDhh000000001S", 0).unwrap();
        assert_eq!(bits.text::<20>(106, 20).unwrap(), "MT.MITCHELL");
        // Truncated to the requested capacity.
        assert_eq!(bits.text::<4>(106, 20).unwrap(), "MT.M");
        // Past the end.
        assert_eq!(bits.text::<20>(106, 22), None);
        assert_eq!(bits.text::<20>(usize::MAX, 1), None);
    }

    #[test]
    fn trailing_spaces_come_off_and_inner_ones_stay() {
        // `A B  ` then `@`: codes 1, 32, 2, 32, 32, 0.
        let mut bits = Bits::new();
        for code in [1_u8, 32, 2, 32, 32, 0, 3] {
            let armoured = if code < 40 { code + 48 } else { code + 56 };
            bits.append(&[armoured], 0).unwrap();
        }
        assert_eq!(bits.text::<8>(0, 7).unwrap(), "A B");
        assert_eq!(bits.text::<8>(18, 2).unwrap(), "");
    }

    #[test]
    fn a_message_cannot_outgrow_the_buffer() {
        let mut bits = Bits::new();
        let fragment = [b'w'; 62];
        for _ in 0..2 {
            bits.append(&fragment, 0).unwrap();
        }
        assert_eq!(bits.len(), 744);
        assert_eq!(
            bits.append(&fragment, 0),
            Err(AisError::TooLong {
                bits: 1116,
                limit: MAX_MESSAGE_BITS
            })
        );
        let exact = [b'w'; 58];
        bits.append(&exact, 4).unwrap();
        assert_eq!(bits.len(), MAX_MESSAGE_BITS);
        assert_eq!(bits.unsigned(MAX_MESSAGE_BITS - 8, 8), Some(0xFF));
    }
}
