//! Sentence framing: start character, address, fields, checksum.
//!
//! Checks what every sentence must satisfy — at most 82 bytes, printable ASCII,
//! `$` or `!` start, matching `*hh` checksum — and returns the address and
//! fields as slices of the input. No copying.

use crate::error::NmeaError;
use crate::field::Field;

/// Maximum sentence length, `$` and `CR LF` included.
pub const MAX_SENTENCE_BYTES: usize = 82;

/// Sentence that passed framing checks.
#[derive(Debug, Clone, Copy)]
pub(crate) struct Frame<'a> {
    /// Address field: talker + formatter, or a proprietary code.
    pub(crate) address: &'a [u8],
    /// Everything after the address and its comma, up to `*`.
    pub(crate) fields: &'a [u8],
    /// Whether the sentence starts with `!` (encapsulation) rather than `$`.
    pub(crate) encapsulated: bool,
}

impl<'a> Frame<'a> {
    /// Validates framing and splits the sentence.
    pub(crate) fn parse(sentence: &'a [u8]) -> Result<Self, NmeaError> {
        if sentence.len() > MAX_SENTENCE_BYTES {
            return Err(NmeaError::TooLong {
                length: sentence.len(),
                limit: MAX_SENTENCE_BYTES,
            });
        }
        let trimmed = trim_terminator(sentence);
        let (start, rest) = trimmed.split_first().ok_or(NmeaError::NoStart)?;
        let encapsulated = match start {
            b'$' => false,
            b'!' => true,
            _ => return Err(NmeaError::NoStart),
        };

        let star = rest
            .iter()
            .position(|&byte| byte == b'*')
            .ok_or(NmeaError::NoChecksum)?;
        let body = rest.get(..star).ok_or(NmeaError::NoChecksum)?;
        let claimed = rest
            .get(star.saturating_add(1)..)
            .and_then(hex_byte)
            .ok_or(NmeaError::NoChecksum)?;
        let computed = checksum(body);
        if computed != claimed {
            return Err(NmeaError::BadChecksum { computed, claimed });
        }

        // Offsets within the sentence: +1 for the start character.
        for (offset, &byte) in (1..=MAX_SENTENCE_BYTES).zip(body) {
            if !is_allowed(byte) {
                return Err(NmeaError::BadCharacter { offset });
            }
        }

        let (address, fields) = match body.iter().position(|&byte| byte == b',') {
            Some(comma) => (
                body.get(..comma).unwrap_or(&[]),
                body.get(comma.saturating_add(1)..).unwrap_or(&[]),
            ),
            None => (body, &[][..]),
        };
        // Talker + formatter (5 characters) or proprietary code (≥ 4);
        // upper-case letters and digits only.
        if address.len() < 4
            || !address
                .iter()
                .all(|byte| byte.is_ascii_uppercase() || byte.is_ascii_digit())
        {
            return Err(NmeaError::BadHeader);
        }
        Ok(Self {
            address,
            fields,
            encapsulated,
        })
    }

    /// Iterator over the fields.
    pub(crate) fn fields(&self) -> Fields<'a> {
        Fields {
            rest: Some(self.fields),
        }
    }
}

/// Strips a trailing `CR LF`, `LF` or `CR`.
fn trim_terminator(sentence: &[u8]) -> &[u8] {
    let mut end = sentence.len();
    while end > 0 && matches!(sentence.get(end - 1), Some(b'\r' | b'\n')) {
        end -= 1;
    }
    sentence.get(..end).unwrap_or(&[])
}

/// XOR of every byte between the start character and `*`.
pub(crate) fn checksum(body: &[u8]) -> u8 {
    body.iter().fold(0, |sum, &byte| sum ^ byte)
}

/// Two hex digits as a byte; `None` otherwise, including longer input.
fn hex_byte(text: &[u8]) -> Option<u8> {
    match text {
        [high, low] => Some(hex_digit(*high)? << 4 | hex_digit(*low)?),
        _ => None,
    }
}

fn hex_digit(byte: u8) -> Option<u8> {
    match byte {
        b'0'..=b'9' => Some(byte - b'0'),
        b'A'..=b'F' => Some(byte - b'A' + 10),
        b'a'..=b'f' => Some(byte - b'a' + 10),
        _ => None,
    }
}

/// Whether a byte is allowed in a sentence body: printable ASCII minus reserved
/// characters.
fn is_allowed(byte: u8) -> bool {
    byte.is_ascii_graphic() && !matches!(byte, b'$' | b'!' | b'*' | b'\\' | b'^' | b'~')
        || byte == b' '
}

/// Sentence fields in order, as slices of the input.
///
/// An empty field (no data) is an empty slice. Past the end the iterator yields
/// `None`, so omitted trailing fields (added in later editions) read as empty
/// where the decoder allows it.
#[derive(Debug, Clone)]
pub(crate) struct Fields<'a> {
    rest: Option<&'a [u8]>,
}

impl<'a> Fields<'a> {
    /// Remaining fields, empty ones included.
    pub(crate) fn remaining(&self) -> usize {
        match self.rest {
            Some(rest) => bytecount(rest, b',').saturating_add(1),
            None => 0,
        }
    }

    /// Fields with their indices. A sentence has fewer fields than bytes, so
    /// the bounded range is never exhausted first.
    pub(crate) fn indexed(self) -> impl Iterator<Item = Field<'a>> {
        self.zip(0..MAX_SENTENCE_BYTES)
            .map(|(bytes, index)| Field { bytes, index })
    }
}

/// Occurrences of a byte. Sentences are ≤ 82 bytes, so a naive count suffices.
///
/// A fold rather than `count`, whose overflow-checked counter the strict
/// profile would flag.
fn bytecount(haystack: &[u8], needle: u8) -> usize {
    haystack
        .iter()
        .filter(|&&byte| byte == needle)
        .fold(0, |count, _| count.saturating_add(1))
}

impl<'a> Iterator for Fields<'a> {
    type Item = &'a [u8];

    fn next(&mut self) -> Option<&'a [u8]> {
        let rest = self.rest?;
        if let Some(comma) = rest.iter().position(|&byte| byte == b',') {
            self.rest = rest.get(comma.saturating_add(1)..);
            rest.get(..comma)
        } else {
            self.rest = None;
            Some(rest)
        }
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::indexing_slicing)]
mod tests {
    use super::*;

    #[test]
    fn a_good_sentence_splits_into_address_and_fields() {
        let frame = Frame::parse(b"$GPGLL,4916.45,N,12311.12,W,225444,A,A*5C\r\n").unwrap();
        assert_eq!(frame.address, b"GPGLL");
        assert!(!frame.encapsulated);
        let fields: [&[u8]; 7] = [b"4916.45", b"N", b"12311.12", b"W", b"225444", b"A", b"A"];
        assert!(frame.fields().eq(fields));
        assert_eq!(frame.fields().remaining(), 7);
    }

    #[test]
    fn empty_fields_are_empty_slices_and_still_counted() {
        let frame = Frame::parse(b"$GPRMC,,V,,,,,,,,,N*7F").unwrap();
        assert_eq!(frame.fields().remaining(), 11);
        assert_eq!(frame.fields().nth(1), Some(&b"V"[..]));
        assert_eq!(frame.fields().nth(2), Some(&b""[..]));
    }

    #[test]
    fn every_framing_fault_is_named() {
        assert_eq!(Frame::parse(b"").unwrap_err(), NmeaError::NoStart);
        assert_eq!(Frame::parse(b"GPGLL,A*00").unwrap_err(), NmeaError::NoStart);
        assert_eq!(
            Frame::parse(b"$GPGLL,A").unwrap_err(),
            NmeaError::NoChecksum
        );
        assert_eq!(
            Frame::parse(b"$GPGLL,A*5").unwrap_err(),
            NmeaError::NoChecksum
        );
        assert_eq!(
            Frame::parse(b"$GPGLL,A*ZZ").unwrap_err(),
            NmeaError::NoChecksum
        );
        assert!(matches!(
            Frame::parse(b"$GPGLL,A*00").unwrap_err(),
            NmeaError::BadChecksum { .. }
        ));
        assert!(matches!(
            Frame::parse(&[b'$'; 83]).unwrap_err(),
            NmeaError::TooLong { length: 83, .. }
        ));
        // Control character inside an otherwise valid sentence.
        let mut bad = *b"$GPGLL,\x01*00";
        let sum = checksum(&bad[1..8]);
        bad[9] = b"0123456789ABCDEF"[usize::from(sum >> 4)];
        bad[10] = b"0123456789ABCDEF"[usize::from(sum & 0xF)];
        assert_eq!(
            Frame::parse(&bad).unwrap_err(),
            NmeaError::BadCharacter { offset: 7 }
        );
        assert_eq!(Frame::parse(b"$GP*17").unwrap_err(), NmeaError::BadHeader);
    }
}
