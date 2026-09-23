//! `VDM` / `VDO` — AIS message or fragment, as encapsulated by the receiver.
//!
//! ```text
//! !AIVDM,n,i,seq,ch,payload,fill*hh
//! ```
//!
//! `VDM`: received from another station; `VDO`: own station's transmission. The
//! payload is the message in 6-bit AIS armouring; decoding is the AIS crate's
//! job. This sentence carries the payload plus reassembly data: fragment number
//! and count, sequence identifier, and fill bits in the last character.

use core::fmt::{self, Write};

use crate::encode;
use crate::error::NmeaError;
use crate::field::Field;
use crate::frame::Frame;
use crate::sentence::Talker;

/// Maximum payload characters per fragment.
///
/// 82 bytes minus address, five short fields, commas, checksum and terminator
/// leaves 62; receivers send at most that.
pub const MAX_PAYLOAD_CHARS: usize = 62;

/// Maximum number of fragments per message.
pub const MAX_FRAGMENTS: u8 = 9;

/// AIS radio channel.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Channel {
    /// AIS 1, 161.975 MHz; written `A` or `1`.
    A,
    /// AIS 2, 162.025 MHz; written `B` or `2`.
    B,
}

impl Channel {
    const fn from_letter(letter: u8) -> Option<Self> {
        Some(match letter {
            b'A' | b'1' => Self::A,
            b'B' | b'2' => Self::B,
            _ => return None,
        })
    }

    const fn letter(self) -> char {
        match self {
            Self::A => 'A',
            Self::B => 'B',
        }
    }
}

/// Armoured payload of one fragment.
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub struct Payload {
    bytes: [u8; MAX_PAYLOAD_CHARS],
    len: u8,
}

impl Payload {
    fn new(bytes: &[u8]) -> Option<Self> {
        if bytes.len() > MAX_PAYLOAD_CHARS {
            return None;
        }
        // Armouring characters are `0`..=`w`, excluding the gap between `W` and
        // `` ` ``.
        if !bytes
            .iter()
            .all(|&byte| matches!(byte, b'0'..=b'W' | b'`'..=b'w'))
        {
            return None;
        }
        let mut stored = [0; MAX_PAYLOAD_CHARS];
        for (slot, byte) in stored.iter_mut().zip(bytes) {
            *slot = *byte;
        }
        // At most `MAX_PAYLOAD_CHARS`, well below `u8::MAX`.
        #[allow(clippy::cast_possible_truncation)]
        Some(Self {
            bytes: stored,
            len: bytes.len() as u8,
        })
    }

    /// Payload characters, 6 bits each.
    #[must_use]
    pub fn as_bytes(&self) -> &[u8] {
        self.bytes.get(..usize::from(self.len)).unwrap_or(&[])
    }

    /// Payload as `&str`.
    #[must_use]
    pub fn as_str(&self) -> &str {
        core::str::from_utf8(self.as_bytes()).unwrap_or("")
    }
}

impl fmt::Debug for Payload {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt::Debug::fmt(self.as_str(), f)
    }
}

impl fmt::Display for Payload {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

/// AIS message or one fragment of it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Vdm {
    /// Talker: `AI` for an AIS receiver; others for base stations etc.
    pub talker: Talker,
    /// `true` for own station (`VDO`), `false` for received (`VDM`).
    pub own: bool,
    /// Fragment count, `1..=9`.
    pub fragments: u8,
    /// Fragment number, `1..=fragments`.
    pub fragment: u8,
    /// Sequence identifier shared by fragments of one message, `0..=9`; empty
    /// for single-fragment messages.
    pub sequence: Option<u8>,
    /// Receive channel; empty if not reported.
    pub channel: Option<Channel>,
    /// Armoured payload.
    pub payload: Payload,
    /// Fill bits in the last payload character, `0..=5`.
    pub fill_bits: u8,
}

impl Vdm {
    /// Minimum field count.
    const REQUIRED: usize = 6;

    pub(crate) fn decode(talker: Talker, own: bool, frame: Frame<'_>) -> Result<Self, NmeaError> {
        let found = frame.fields().remaining();
        if found < Self::REQUIRED {
            return Err(NmeaError::TooFewFields {
                found,
                required: Self::REQUIRED,
            });
        }
        let mut fields = frame.fields().indexed();
        let mut next = || {
            fields.next().unwrap_or(Field {
                bytes: &[],
                index: usize::MAX,
            })
        };

        let fragments = next().unsigned::<u8>("fragment count")?;
        let fragment_field = next();
        let fragment = fragment_field.unsigned::<u8>("fragment number")?;
        if !(1..=MAX_FRAGMENTS).contains(&fragments) {
            return Err(NmeaError::BadField {
                index: 0,
                expected: "fragment count",
            });
        }
        if fragment == 0 || fragment > fragments {
            return Err(NmeaError::BadField {
                index: 1,
                expected: "fragment number",
            });
        }
        let sequence = next().optional_unsigned::<u8>("sequence id")?;
        if sequence.is_some_and(|sequence| sequence > 9) {
            return Err(NmeaError::BadField {
                index: 2,
                expected: "sequence id",
            });
        }
        if fragments > 1 && sequence.is_none() {
            return Err(NmeaError::BadField {
                index: 2,
                expected: "sequence id",
            });
        }
        let channel = next().optional_channel()?;
        let payload_field = next();
        let payload = Payload::new(payload_field.bytes).ok_or(NmeaError::BadField {
            index: 4,
            expected: "armoured payload",
        })?;
        let fill_bits = next().unsigned::<u8>("fill bits")?;
        if fill_bits > 5 {
            return Err(NmeaError::BadField {
                index: 5,
                expected: "fill bits",
            });
        }

        Ok(Self {
            talker,
            own,
            fragments,
            fragment,
            sequence,
            channel,
            payload,
            fill_bits,
        })
    }

    /// Whether the message fits in this single fragment.
    #[must_use]
    pub const fn is_whole(&self) -> bool {
        self.fragments == 1
    }
}

impl Field<'_> {
    fn optional_channel(&self) -> Result<Option<Channel>, NmeaError> {
        match self.bytes {
            [] => Ok(None),
            [letter] => Channel::from_letter(*letter)
                .map(Some)
                .ok_or(NmeaError::BadField {
                    index: self.index,
                    expected: "channel",
                }),
            _ => Err(NmeaError::BadField {
                index: self.index,
                expected: "channel",
            }),
        }
    }
}

impl fmt::Display for Vdm {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        encode::encapsulated(f, |out| {
            write!(
                out,
                "{}{},{},{},",
                self.talker,
                if self.own { "VDO" } else { "VDM" },
                self.fragments,
                self.fragment
            )?;
            encode::optional(out, self.sequence)?;
            out.write_char(',')?;
            encode::optional(out, self.channel.map(Channel::letter))?;
            write!(out, ",{},{}", self.payload, self.fill_bits)
        })
    }
}
