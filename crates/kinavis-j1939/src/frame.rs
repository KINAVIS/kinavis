//! CAN frame: identifier and up to 8 data bytes.

use crate::error::J1939Error;
use crate::id::CanId;

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
    /// [`J1939Error::BadFrameLength`] for more than 8 bytes.
    pub fn new(id: CanId, data: &[u8]) -> Result<Self, J1939Error> {
        if data.len() > 8 {
            return Err(J1939Error::BadFrameLength { len: data.len() });
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

impl Frame {
    /// Frame of at most the first 8 bytes of `data`; for frames this crate
    /// builds, whose length is known.
    pub(crate) fn truncated(id: CanId, data: &[u8]) -> Self {
        let mut bytes = [0xFF; 8];
        let mut len = 0u8;
        for (slot, byte) in bytes.iter_mut().zip(data) {
            *slot = *byte;
            len = len.saturating_add(1);
        }
        Self {
            id,
            data: bytes,
            len,
        }
    }

    /// Frame of a parameter group from a source, broadcast or to a
    /// destination (ignored for a broadcast group).
    ///
    /// # Errors
    ///
    /// [`J1939Error::BadFrameLength`] for more than 8 bytes.
    pub fn of(
        priority: u8,
        pgn: crate::id::Pgn,
        destination: u8,
        source: u8,
        data: &[u8],
    ) -> Result<Self, J1939Error> {
        Self::new(CanId::from_parts(priority, pgn, destination, source), data)
    }
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
    type Error = J1939Error;

    fn try_from(stored: StoredFrame) -> Result<Self, J1939Error> {
        let data =
            stored
                .data
                .get(..usize::from(stored.len))
                .ok_or(J1939Error::BadFrameLength {
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

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use super::*;

    #[test]
    fn a_frame_holds_at_most_eight_bytes() {
        let id = CanId::new(0x18EE_FF47).unwrap();
        let frame = Frame::new(id, &[1, 2, 3]).unwrap();
        assert_eq!(frame.data(), &[1, 2, 3]);
        assert_eq!(
            Frame::new(id, &[0; 9]),
            Err(J1939Error::BadFrameLength { len: 9 })
        );
    }
}
