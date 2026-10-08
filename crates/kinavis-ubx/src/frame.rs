//! UBX framing: `B5 62`, class, identifier, little-endian length, payload,
//! 8-bit Fletcher checksum over class to payload.

use core::fmt;

/// A checked UBX frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Frame<'a> {
    /// Message class.
    pub class: u8,
    /// Message identifier within the class.
    pub id: u8,
    /// Payload.
    pub payload: &'a [u8],
}

/// First synchronisation byte.
pub const SYNC_1: u8 = 0xB5;
/// Second synchronisation byte.
pub const SYNC_2: u8 = 0x62;

/// Fletcher checksum `(CK_A, CK_B)` of class, identifier, length and payload.
#[must_use]
pub fn checksum(class: u8, id: u8, payload: &[u8]) -> (u8, u8) {
    let length = u16::try_from(payload.len())
        .unwrap_or(u16::MAX)
        .to_le_bytes();
    [class, id, length[0], length[1]]
        .iter()
        .chain(payload)
        .fold((0_u8, 0_u8), |(a, b), &byte| {
            let a = a.wrapping_add(byte);
            (a, b.wrapping_add(a))
        })
}

/// What a byte fed to the [`Deframer`] produced.
#[derive(Debug, PartialEq, Eq)]
pub enum Event<'a> {
    /// A complete frame with a valid checksum.
    Frame(Frame<'a>),
    /// A byte outside any UBX frame: NMEA text on a mixed port, or noise.
    Passthrough(u8),
    /// A frame that failed; the deframer is looking for the next one.
    Error(DeframeError),
}

/// Why a frame was dropped.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DeframeError {
    /// The checksum did not match.
    BadChecksum {
        /// Class of the dropped frame.
        class: u8,
        /// Identifier of the dropped frame.
        id: u8,
    },
    /// The length exceeds the deframer's capacity.
    TooLong {
        /// Announced payload length.
        length: u16,
        /// Capacity of the deframer.
        capacity: usize,
    },
}

impl fmt::Display for DeframeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::BadChecksum { class, id } => {
                write!(f, "UBX {class:02X}-{id:02X} failed its checksum")
            }
            Self::TooLong { length, capacity } => write!(
                f,
                "UBX payload of {length} bytes exceeds the capacity of {capacity}"
            ),
        }
    }
}

impl core::error::Error for DeframeError {}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum State {
    Sync1,
    Sync2,
    Class,
    Id,
    Length1,
    Length2,
    Payload,
    ChecksumA,
    ChecksumB,
}

/// Streaming UBX deframer with a payload capacity of `N` bytes.
///
/// Fed one byte at a time from a serial port that may also carry NMEA: bytes
/// outside frames come back as [`Event::Passthrough`], a bad frame as
/// [`Event::Error`] after which the deframer resynchronises on the next
/// `B5 62`. A frame longer than `N` is an error, never truncated.
#[derive(Clone)]
pub struct Deframer<const N: usize> {
    state: State,
    class: u8,
    id: u8,
    length: u16,
    received: usize,
    checksum_a: u8,
    buffer: [u8; N],
}

impl<const N: usize> Deframer<N> {
    /// Deframer waiting for a frame.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            state: State::Sync1,
            class: 0,
            id: 0,
            length: 0,
            received: 0,
            checksum_a: 0,
            buffer: [0; N],
        }
    }

    /// Takes one byte.
    pub fn push(&mut self, byte: u8) -> Option<Event<'_>> {
        match self.state {
            State::Sync1 => {
                if byte == SYNC_1 {
                    self.state = State::Sync2;
                    None
                } else {
                    Some(Event::Passthrough(byte))
                }
            }
            State::Sync2 => {
                if byte == SYNC_2 {
                    self.state = State::Class;
                    None
                } else if byte == SYNC_1 {
                    Some(Event::Passthrough(SYNC_1))
                } else {
                    self.state = State::Sync1;
                    Some(Event::Passthrough(byte))
                }
            }
            State::Class => {
                self.class = byte;
                self.state = State::Id;
                None
            }
            State::Id => {
                self.id = byte;
                self.state = State::Length1;
                None
            }
            State::Length1 => {
                self.length = u16::from(byte);
                self.state = State::Length2;
                None
            }
            State::Length2 => {
                self.length |= u16::from(byte) << 8;
                if usize::from(self.length) > N {
                    self.state = State::Sync1;
                    return Some(Event::Error(DeframeError::TooLong {
                        length: self.length,
                        capacity: N,
                    }));
                }
                self.received = 0;
                self.state = if self.length == 0 {
                    State::ChecksumA
                } else {
                    State::Payload
                };
                None
            }
            State::Payload => {
                if let Some(slot) = self.buffer.get_mut(self.received) {
                    *slot = byte;
                }
                self.received = self.received.saturating_add(1);
                if self.received >= usize::from(self.length) {
                    self.state = State::ChecksumA;
                }
                None
            }
            State::ChecksumA => {
                self.checksum_a = byte;
                self.state = State::ChecksumB;
                None
            }
            State::ChecksumB => {
                self.state = State::Sync1;
                let payload = self.buffer.get(..usize::from(self.length)).unwrap_or(&[]);
                if checksum(self.class, self.id, payload) == (self.checksum_a, byte) {
                    Some(Event::Frame(Frame {
                        class: self.class,
                        id: self.id,
                        payload,
                    }))
                } else {
                    Some(Event::Error(DeframeError::BadChecksum {
                        class: self.class,
                        id: self.id,
                    }))
                }
            }
        }
    }
}

impl<const N: usize> Default for Deframer<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> fmt::Debug for Deframer<N> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Deframer")
            .field("state", &self.state)
            .field("capacity", &N)
            .finish_non_exhaustive()
    }
}
