//! 29-bit CAN identifier and the parameter group number (PGN).
//!
//! NMEA 2000 uses SAE J1939 framing on a 250 kbit/s CAN bus. The extended
//! identifier carries the sender, priority and the *parameter group* the 8 data
//! bytes belong to:
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
    /// Position, rapid update: latitude and longitude, several times per
    /// second.
    pub const POSITION_RAPID_UPDATE: Self = Self(129_025);
    /// Course and speed over ground, rapid update.
    pub const COG_SOG_RAPID_UPDATE: Self = Self(129_026);
    /// GNSS position data: full fix, once per second.
    pub const GNSS_POSITION_DATA: Self = Self(129_029);
    /// Vessel heading, with deviation and variation if known.
    pub const VESSEL_HEADING: Self = Self(127_250);
    /// Water depth below the transducer.
    pub const WATER_DEPTH: Self = Self(128_267);
    /// AIS class A position report.
    pub const AIS_CLASS_A_POSITION_REPORT: Self = Self(129_038);
    /// AIS class B position report.
    pub const AIS_CLASS_B_POSITION_REPORT: Self = Self(129_039);

    /// PGN from its number.
    #[must_use]
    pub const fn new(number: u32) -> Self {
        Self(number)
    }

    /// Numeric value.
    #[must_use]
    pub const fn number(self) -> u32 {
        self.0
    }

    /// Transport for known PGNs; `None` otherwise — supply it via
    /// [`Assembler::push_as`](crate::Assembler::push_as).
    #[must_use]
    pub const fn transport(self) -> Option<Transport> {
        match self {
            Self::POSITION_RAPID_UPDATE
            | Self::COG_SOG_RAPID_UPDATE
            | Self::VESSEL_HEADING
            | Self::WATER_DEPTH => Some(Transport::SingleFrame),
            Self::GNSS_POSITION_DATA
            | Self::AIS_CLASS_A_POSITION_REPORT
            | Self::AIS_CLASS_B_POSITION_REPORT => Some(Transport::FastPacket),
            _ => None,
        }
    }
}

impl fmt::Display for Pgn {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "PGN {}", self.0)
    }
}

/// PGN transport on the bus.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Transport {
    /// Single 8-byte frame.
    SingleFrame,
    /// Fast packet: up to 32 frames, first with the length, rest with a
    /// counter; reassembled by an [`Assembler`](crate::Assembler).
    FastPacket,
}

/// 29-bit extended CAN identifier, J1939 layout.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct CanId(u32);

impl CanId {
    /// Identifier from 29 bits.
    ///
    /// # Errors
    ///
    /// [`crate::Nmea2000Error::BadCanId`] if any of the top three bits is set.
    pub const fn new(raw: u32) -> Result<Self, crate::Nmea2000Error> {
        if raw >> 29 != 0 {
            return Err(crate::Nmea2000Error::BadCanId { raw });
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
        // Position rapid update, source 35, priority 2: 0x09F80123 = 2 << 26 |
        // 0x1F801 << 8 | 0x23.
        let id = CanId::new(0x09F8_0123).unwrap();
        assert_eq!(id.priority(), 2);
        assert_eq!(id.pgn(), Pgn::POSITION_RAPID_UPDATE);
        assert_eq!(id.destination(), None);
        assert_eq!(id.source(), 35);
        assert_eq!(id, CanId::from_parts(2, Pgn::POSITION_RAPID_UPDATE, 0, 35));
        assert_eq!(std::format!("{id}"), "PGN 129025 from 35, priority 2");
    }

    #[test]
    fn an_addressed_group_keeps_its_destination_out_of_the_number() {
        // ISO request, PGN 59904 = 0xEA00, PDU1: PS is the destination.
        let id = CanId::new((6 << 26) | (0xEA << 16) | (0x0C << 8) | 0x21).unwrap();
        assert_eq!(id.pgn(), Pgn::new(59_904));
        assert_eq!(id.destination(), Some(12));
        assert_eq!(id.source(), 0x21);
        assert_eq!(id, CanId::from_parts(6, Pgn::new(59_904), 12, 0x21));
        assert_eq!(std::format!("{id}"), "PGN 59904 from 33 to 12, priority 6");
    }

    #[test]
    fn the_data_page_bit_reaches_the_group_number() {
        // PGN 130306 (wind) has DP set: 0x1FD02.
        let id = CanId::from_parts(2, Pgn::new(130_306), 0, 1);
        assert_eq!(id.pgn().number(), 130_306);
        assert_eq!(id.raw() >> 24 & 0x1, 1);
    }

    #[test]
    fn thirty_bits_are_refused() {
        assert_eq!(
            CanId::new(1 << 29),
            Err(crate::Nmea2000Error::BadCanId { raw: 1 << 29 })
        );
        assert!(CanId::new((1 << 29) - 1).is_ok());
    }

    #[test]
    fn the_known_groups_know_their_transport() {
        assert_eq!(Pgn::WATER_DEPTH.transport(), Some(Transport::SingleFrame));
        assert_eq!(
            Pgn::GNSS_POSITION_DATA.transport(),
            Some(Transport::FastPacket)
        );
        assert_eq!(Pgn::new(130_306).transport(), None);
        assert_eq!(std::format!("{}", Pgn::WATER_DEPTH), "PGN 128267");
    }
}
