//! MAVLink frames: v2 and v1 on the way in, v2 on the way out; checksum with
//! `CRC_EXTRA`, signature carried; a byte-by-byte deframer.

use core::fmt;

use crate::messages::{crc_extra, Message};
use crate::wire::Crc;

/// Longest payload.
pub const MAX_PAYLOAD: usize = 255;
/// Longest frame: header, payload, checksum, signature.
pub const MAX_FRAME: usize = 10 + MAX_PAYLOAD + 2 + 13;

const STX_V2: u8 = 0xFD;
const STX_V1: u8 = 0xFE;
/// Incompatibility flag: the frame is signed.
pub const SIGNED: u8 = 0x01;

/// Protocol version of a frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Version {
    /// MAVLink 1: one-byte message id, no flags, no signature.
    V1,
    /// MAVLink 2.
    V2,
}

/// Signature of a MAVLink 2 frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Signature {
    /// Link id.
    pub link: u8,
    /// Time, units of 10 µs since 2015-01-01 00:00:00 UTC, 48 bits.
    pub timestamp: u64,
    /// The first 48 bits of the SHA-256 over key, frame and the two above.
    pub value: [u8; 6],
}

/// Why bytes are not a frame.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MavlinkError {
    /// A frame whose checksum does not match.
    BadChecksum {
        /// Message id.
        id: u32,
    },
    /// A frame of a message this crate does not know: without its
    /// `CRC_EXTRA` the checksum cannot be checked.
    UnknownMessage {
        /// Message id.
        id: u32,
    },
    /// A MAVLink 2 frame with incompatibility flags this crate does not
    /// support.
    UnsupportedFlags {
        /// The flags.
        flags: u8,
    },
}

impl fmt::Display for MavlinkError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::BadChecksum { id } => write!(f, "message {id}: bad checksum"),
            Self::UnknownMessage { id } => write!(f, "message {id} is not known"),
            Self::UnsupportedFlags { flags } => {
                write!(f, "incompatibility flags {flags:#04x} not supported")
            }
        }
    }
}

impl core::error::Error for MavlinkError {}

/// One MAVLink frame, its checksum checked.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Frame {
    /// Protocol version.
    pub version: Version,
    /// Sequence number of the sender.
    pub sequence: u8,
    /// Sending system.
    pub system: u8,
    /// Sending component.
    pub component: u8,
    /// Message id.
    pub id: u32,
    /// Compatibility flags (MAVLink 2).
    pub compat: u8,
    /// Signature, if signed.
    pub signature: Option<Signature>,
    incompat: u8,
    len: u8,
    payload: [u8; MAX_PAYLOAD],
}

impl Frame {
    /// MAVLink 2 frame of a message, trailing zeros of its payload dropped.
    #[must_use]
    pub fn new(system: u8, component: u8, sequence: u8, message: &Message) -> Self {
        let mut payload = [0u8; MAX_PAYLOAD];
        let full = message.encode_into(&mut payload);
        let kept = payload
            .get(..full)
            .and_then(|bytes| bytes.iter().rposition(|&byte| byte != 0))
            .map_or(1, |last| last.saturating_add(1));
        Self {
            version: Version::V2,
            sequence,
            system,
            component,
            id: message.id(),
            compat: 0,
            signature: None,
            incompat: 0,
            len: u8::try_from(kept).unwrap_or(u8::MAX),
            payload,
        }
    }

    /// Payload as on the wire.
    #[must_use]
    pub fn payload(&self) -> &[u8] {
        self.payload.get(..usize::from(self.len)).unwrap_or(&[])
    }

    /// Incompatibility flags (MAVLink 2): [`SIGNED`] when signed.
    #[must_use]
    pub const fn incompat(&self) -> u8 {
        self.incompat
    }

    /// The message, if this crate knows it.
    #[must_use]
    pub fn message(&self) -> Option<Message> {
        Message::decode(self.id, self.payload())
    }

    /// The bytes of the frame, MAVLink 2 (MAVLink 1 for a frame read as
    /// one), signature included if set; returns their number.
    pub fn encode(&self, out: &mut [u8; MAX_FRAME]) -> usize {
        let header = self.header();
        let header_len = match self.version {
            Version::V1 => 6,
            Version::V2 => 10,
        };
        let mut at = 0usize;
        let mut put = |bytes: &[u8]| {
            for byte in bytes {
                if let Some(slot) = out.get_mut(at) {
                    *slot = *byte;
                }
                at = at.saturating_add(1);
            }
        };
        put(header.get(..header_len).unwrap_or(&[]));
        put(self.payload());
        put(&self.checksum().to_le_bytes());
        if let (Version::V2, Some(signature)) = (self.version, self.signature) {
            put(&[signature.link]);
            put(&signature
                .timestamp
                .to_le_bytes()
                .get(..6)
                .unwrap_or(&[0; 6])
                .to_owned_array());
            put(&signature.value);
        }
        at.min(MAX_FRAME)
    }

    /// Header bytes, STX first; ten for MAVLink 2, six of them for 1.
    fn header(&self) -> [u8; 10] {
        let [id0, id1, id2, _] = self.id.to_le_bytes();
        match self.version {
            Version::V1 => [
                STX_V1,
                self.len,
                self.sequence,
                self.system,
                self.component,
                id0,
                0,
                0,
                0,
                0,
            ],
            Version::V2 => [
                STX_V2,
                self.len,
                self.incompat,
                self.compat,
                self.sequence,
                self.system,
                self.component,
                id0,
                id1,
                id2,
            ],
        }
    }

    /// Checksum of header (after STX), payload and `CRC_EXTRA`.
    fn checksum(&self) -> u16 {
        let header = self.header();
        let header_len = match self.version {
            Version::V1 => 6,
            Version::V2 => 10,
        };
        let mut crc = Crc::new();
        crc.add(header.get(1..header_len).unwrap_or(&[]));
        crc.add(self.payload());
        crc.add(&[crc_extra(self.id).unwrap_or(0)]);
        crc.value()
    }

    /// Bytes the signature covers: header, payload, checksum, link id and
    /// timestamp.
    #[cfg(feature = "signing")]
    pub(crate) fn signed_bytes(&self, link: u8, timestamp: u64, mut add: impl FnMut(&[u8])) {
        add(&self.header());
        add(self.payload());
        add(&self.checksum().to_le_bytes());
        add(&[link]);
        add(timestamp.to_le_bytes().get(..6).unwrap_or(&[0; 6]));
    }

    #[cfg(feature = "signing")]
    pub(crate) fn set_signed(&mut self, signature: Signature) {
        self.incompat |= SIGNED;
        self.signature = Some(signature);
    }
}

trait ToOwnedArray {
    fn to_owned_array(&self) -> [u8; 6];
}

impl ToOwnedArray for [u8] {
    fn to_owned_array(&self) -> [u8; 6] {
        let mut out = [0u8; 6];
        for (slot, byte) in out.iter_mut().zip(self) {
            *slot = *byte;
        }
        out
    }
}

/// Frames out of a byte stream, one byte at a time.
///
/// Bytes before a start marker are skipped. A frame whose checksum fails is
/// reported and the search resumes one byte after its marker, so a marker
/// inside a corrupt frame is not lost.
#[derive(Debug, Clone)]
pub struct Deframer {
    buffer: [u8; MAX_FRAME],
    len: usize,
}

impl Default for Deframer {
    fn default() -> Self {
        Self::new()
    }
}

impl Deframer {
    /// Empty deframer.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            buffer: [0; MAX_FRAME],
            len: 0,
        }
    }

    /// Takes one byte; returns a frame or an error when one ends.
    pub fn push(&mut self, byte: u8) -> Option<Result<Frame, MavlinkError>> {
        if let Some(slot) = self.buffer.get_mut(self.len) {
            *slot = byte;
            self.len = self.len.saturating_add(1);
        }
        self.scan()
    }

    fn scan(&mut self) -> Option<Result<Frame, MavlinkError>> {
        // Drop bytes before a marker.
        let start = self
            .buffer
            .get(..self.len)
            .and_then(|bytes| bytes.iter().position(|&b| b == STX_V2 || b == STX_V1));
        match start {
            None => {
                self.len = 0;
                return None;
            }
            Some(skip) => self.consume(skip),
        }
        let bytes = self.buffer.get(..self.len).unwrap_or(&[]);
        let needed = frame_length(bytes)?;
        if self.len < needed {
            return None;
        }
        match parse(bytes.get(..needed).unwrap_or(&[])) {
            Ok(frame) => {
                self.consume(needed);
                Some(Ok(frame))
            }
            Err(error @ MavlinkError::BadChecksum { .. }) => {
                // Resume one byte after the marker; a marker inside the
                // corrupt frame is found by the next byte's scan.
                self.consume(1);
                Some(Err(error))
            }
            Err(error) => {
                self.consume(needed);
                Some(Err(error))
            }
        }
    }

    fn consume(&mut self, count: usize) {
        let count = count.min(self.len);
        let mut to = 0usize;
        for from in count..self.len {
            if let Some(byte) = self.buffer.get(from).copied() {
                if let Some(slot) = self.buffer.get_mut(to) {
                    *slot = byte;
                }
            }
            to = to.saturating_add(1);
        }
        self.len = self.len.saturating_sub(count);
    }
}

/// Length of the frame at the start of `bytes`, once its header is in.
fn frame_length(bytes: &[u8]) -> Option<usize> {
    match bytes {
        [STX_V2, len, incompat, ..] => {
            let signature = if incompat & SIGNED != 0 { 13 } else { 0 };
            Some(10 + usize::from(*len) + 2 + signature)
        }
        [STX_V1, len, ..] => Some(6 + usize::from(*len) + 2),
        _ => None,
    }
}

/// One whole frame.
fn parse(bytes: &[u8]) -> Result<Frame, MavlinkError> {
    let (version, header_len) = match bytes.first() {
        Some(&STX_V2) => (Version::V2, 10),
        _ => (Version::V1, 6),
    };
    let header: [u8; 10] = {
        let mut h = [0u8; 10];
        for (slot, byte) in h.iter_mut().zip(bytes) {
            *slot = *byte;
        }
        h
    };
    let len = usize::from(header[1]);
    let (incompat, compat, sequence, system, component, id) = match version {
        Version::V2 => (
            header[2],
            header[3],
            header[4],
            header[5],
            header[6],
            u32::from_le_bytes([header[7], header[8], header[9], 0]),
        ),
        Version::V1 => (0, 0, header[2], header[3], header[4], u32::from(header[5])),
    };
    let mut payload = [0u8; MAX_PAYLOAD];
    for (slot, byte) in payload
        .iter_mut()
        .zip(bytes.iter().skip(header_len).take(len))
    {
        *slot = *byte;
    }
    let at = header_len.saturating_add(len);
    let received = u16::from_le_bytes([
        bytes.get(at).copied().unwrap_or(0),
        bytes.get(at.saturating_add(1)).copied().unwrap_or(0),
    ]);
    if crc_extra(id).is_none() {
        return Err(MavlinkError::UnknownMessage { id });
    }
    if incompat & !SIGNED != 0 {
        return Err(MavlinkError::UnsupportedFlags { flags: incompat });
    }
    let signature = (incompat & SIGNED != 0).then(|| {
        let signed = bytes.get(at.saturating_add(2)..).unwrap_or(&[]);
        let mut timestamp = [0u8; 8];
        for (slot, byte) in timestamp.iter_mut().zip(signed.iter().skip(1).take(6)) {
            *slot = *byte;
        }
        let mut value = [0u8; 6];
        for (slot, byte) in value.iter_mut().zip(signed.iter().skip(7).take(6)) {
            *slot = *byte;
        }
        Signature {
            link: signed.first().copied().unwrap_or(0),
            timestamp: u64::from_le_bytes(timestamp),
            value,
        }
    });
    let frame = Frame {
        version,
        sequence,
        system,
        component,
        id,
        compat,
        signature,
        incompat,
        len: u8::try_from(len).unwrap_or(u8::MAX),
        payload,
    };
    if frame.checksum() != received {
        return Err(MavlinkError::BadChecksum { id });
    }
    Ok(frame)
}
