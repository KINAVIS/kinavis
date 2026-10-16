//! 29-bit CAN identifier and the parameter group number (PGN).
//!
//! SAE J1939, ISO 11783 (ISOBUS) and NMEA 2000 share one framing on a
//! 250 kbit/s CAN bus. The extended identifier carries the sender, priority
//! and the *parameter group* the 8 data bytes belong to:
//!
//! ```text
//! bit 28  26 25 24 23    16 15     8 7      0
//!    ┌──────┬──┬──┬────────┬────────┬────────┐
//!    │ prio │R │DP│  PF    │  PS    │ source │
//!    └──────┴──┴──┴────────┴────────┴────────┘
//! ```
//!
//! The PGN is `R`, `DP`, `PF`, plus `PS` when `PF ≥ 240` (*PDU2*, broadcast).
//! For `PF < 240` the frame is *PDU1*, addressed to one node, and `PS` is the
//! destination.

use core::fmt;

/// Parameter group number.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Pgn(u32);

impl Pgn {
    /// Acknowledgement: ACK or NACK of a request or command.
    pub const ACKNOWLEDGEMENT: Self = Self(59_392);
    /// Request for a parameter group.
    pub const REQUEST: Self = Self(59_904);
    /// Transport protocol, data transfer (TP.DT).
    pub const TP_DATA: Self = Self(60_160);
    /// Transport protocol, connection management (TP.CM).
    pub const TP_CONNECTION: Self = Self(60_416);
    /// Address claimed, and cannot claim from the null address.
    pub const ADDRESS_CLAIMED: Self = Self(60_928);
    /// Extended transport protocol, data transfer (ETP.DT).
    pub const ETP_DATA: Self = Self(50_944);
    /// Extended transport protocol, connection management (ETP.CM).
    pub const ETP_CONNECTION: Self = Self(51_200);

    /// PGN from its number; bits above the 18 of a PGN are dropped.
    #[must_use]
    pub const fn new(number: u32) -> Self {
        Self(number & 0x3_FFFF)
    }

    /// Numeric value.
    #[must_use]
    pub const fn number(self) -> u32 {
        self.0
    }

    /// Whether the group is addressed to one node (PDU1, PF < 240), so the
    /// low byte of its identifier is a destination.
    #[must_use]
    pub const fn is_addressed(self) -> bool {
        (self.0 >> 8) & 0xFF < 240
    }

    /// The three little-endian bytes of the PGN, as transport and request
    /// messages carry it.
    #[must_use]
    pub const fn to_bytes(self) -> [u8; 3] {
        // Masked to bytes.
        #[allow(clippy::cast_possible_truncation)]
        [
            (self.0 & 0xFF) as u8,
            ((self.0 >> 8) & 0xFF) as u8,
            ((self.0 >> 16) & 0x03) as u8,
        ]
    }

    /// PGN from three little-endian bytes.
    #[must_use]
    pub const fn from_bytes(bytes: [u8; 3]) -> Self {
        Self::new(bytes[0] as u32 | (bytes[1] as u32) << 8 | (bytes[2] as u32) << 16)
    }
}

impl fmt::Display for Pgn {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "PGN {}", self.0)
    }
}

/// 29-bit extended CAN identifier, J1939 layout.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct CanId(u32);

impl CanId {
    /// Source address of a node that has not claimed one.
    pub const NULL_ADDRESS: u8 = 254;
    /// Destination address of every node.
    pub const GLOBAL_ADDRESS: u8 = 255;

    /// Identifier from 29 bits.
    ///
    /// # Errors
    ///
    /// [`crate::J1939Error::BadCanId`] if any of the top three bits is set.
    pub const fn new(raw: u32) -> Result<Self, crate::J1939Error> {
        if raw >> 29 != 0 {
            return Err(crate::J1939Error::BadCanId { raw });
        }
        Ok(Self(raw))
    }

    /// Identifier from its parts (inverse of the readers), for tests and
    /// transmitters. For PDU1 the PGN low byte is ignored: `PS` is the
    /// destination.
    #[must_use]
    pub const fn from_parts(priority: u8, pgn: Pgn, destination: u8, source: u8) -> Self {
        let pgn = pgn.0 & 0x3_FFFF;
        let pf = (pgn >> 8) & 0xFF;
        let ps = if pf < 240 {
            destination as u32
        } else {
            pgn & 0xFF
        };
        Self(
            ((priority as u32 & 0x7) << 26)
                | ((pgn >> 16) << 24)
                | (pf << 16)
                | (ps << 8)
                | source as u32,
        )
    }

    /// Raw 29 bits.
    #[must_use]
    pub const fn raw(self) -> u32 {
        self.0
    }

    /// Priority `0..=7`, 0 highest.
    #[must_use]
    pub const fn priority(self) -> u8 {
        // Three bits.
        ((self.0 >> 26) & 0x7) as u8
    }

    /// PGN.
    #[must_use]
    pub const fn pgn(self) -> Pgn {
        let bits = (self.0 >> 8) & 0x3_FFFF;
        if Self::is_pdu1(bits) {
            Pgn(bits & 0x3_FF00)
        } else {
            Pgn(bits)
        }
    }

    /// Destination address for PDU1; `None` for PDU2 (broadcast).
    #[must_use]
    pub const fn destination(self) -> Option<u8> {
        let bits = (self.0 >> 8) & 0x3_FFFF;
        if Self::is_pdu1(bits) {
            // Low byte.
            Some((bits & 0xFF) as u8)
        } else {
            None
        }
    }

    /// Source address `0..=253`; 254 = no address claimed, 255 = broadcast.
    #[must_use]
    pub const fn source(self) -> u8 {
        // Low byte.
        (self.0 & 0xFF) as u8
    }

    const fn is_pdu1(pgn_bits: u32) -> bool {
        (pgn_bits >> 8) & 0xFF < 240
    }
}

impl fmt::Display for CanId {
    /// Formats as `PGN 129025 from 35, priority 2`, with `to 12` after the
    /// source for addressed frames.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{} from {}", self.pgn(), self.source())?;
        if let Some(destination) = self.destination() {
            write!(f, " to {destination}")?;
        }
        write!(f, ", priority {}", self.priority())
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use super::*;

    #[test]
    fn a_broadcast_group_takes_the_whole_low_word() {
        // Ground-based speed and distance, PGN 65097 = 0xFE49, from 0x47.
        let id = CanId::new(0x0CFE_4947).unwrap();
        assert_eq!(id.priority(), 3);
        assert_eq!(id.pgn(), Pgn::new(65_097));
        assert_eq!(id.destination(), None);
        assert_eq!(id.source(), 0x47);
        assert_eq!(id, CanId::from_parts(3, Pgn::new(65_097), 0, 0x47));
        assert_eq!(std::format!("{id}"), "PGN 65097 from 71, priority 3");
    }

    #[test]
    fn an_addressed_group_keeps_its_destination_out_of_the_number() {
        let id = CanId::new(0x18EA_0C21).unwrap();
        assert_eq!(id.pgn(), Pgn::REQUEST);
        assert!(id.pgn().is_addressed());
        assert_eq!(id.destination(), Some(12));
        assert_eq!(id, CanId::from_parts(6, Pgn::REQUEST, 12, 0x21));
        assert_eq!(std::format!("{id}"), "PGN 59904 from 33 to 12, priority 6");
    }

    #[test]
    fn the_data_page_bit_reaches_the_group_number() {
        let id = CanId::from_parts(2, Pgn::new(130_306), 0, 1);
        assert_eq!(id.pgn().number(), 130_306);
        assert_eq!(id.raw() >> 24 & 0x1, 1);
        assert_eq!(
            Pgn::from_bytes(Pgn::new(130_306).to_bytes()),
            Pgn::new(130_306)
        );
    }

    #[test]
    fn thirty_bits_are_refused() {
        assert_eq!(
            CanId::new(1 << 29),
            Err(crate::J1939Error::BadCanId { raw: 1 << 29 })
        );
        assert!(CanId::new((1 << 29) - 1).is_ok());
    }
}
