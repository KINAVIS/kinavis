//! The identifier and the parameter group number, from `kinavis-j1939`, and
//! the groups of NMEA 2000.
//!
//! NMEA 2000 uses the framing of SAE J1939 (ISO 11783-3) on a 250 kbit/s CAN
//! bus; the identifier, its parameter group and the frame are
//! `kinavis-j1939`'s, re-exported here.

pub use kinavis_j1939::{CanId, Pgn};

/// The parameter groups of NMEA 2000 this crate decodes, and their transport:
/// associated constants on [`Pgn`], in scope with this trait.
///
/// ```rust
/// use kinavis_nmea2000::{Nmea2000Pgn, Pgn, Transport};
/// assert_eq!(Pgn::GNSS_POSITION_DATA.transport(), Some(Transport::FastPacket));
/// ```
pub trait Nmea2000Pgn: Sized {
    /// Position, rapid update: latitude and longitude, several times per
    /// second.
    const POSITION_RAPID_UPDATE: Self;
    /// Course and speed over ground, rapid update.
    const COG_SOG_RAPID_UPDATE: Self;
    /// GNSS position data: full fix, once per second.
    const GNSS_POSITION_DATA: Self;
    /// Vessel heading, with deviation and variation if known.
    const VESSEL_HEADING: Self;
    /// Water depth below the transducer.
    const WATER_DEPTH: Self;
    /// AIS class A position report.
    const AIS_CLASS_A_POSITION_REPORT: Self;
    /// AIS class B position report.
    const AIS_CLASS_B_POSITION_REPORT: Self;

    /// Transport for the groups this crate knows; `None` otherwise — supply
    /// it via [`Assembler::push_as`](crate::Assembler::push_as).
    fn transport(self) -> Option<Transport>;
}

impl Nmea2000Pgn for Pgn {
    const POSITION_RAPID_UPDATE: Self = Self::new(129_025);
    const COG_SOG_RAPID_UPDATE: Self = Self::new(129_026);
    const GNSS_POSITION_DATA: Self = Self::new(129_029);
    const VESSEL_HEADING: Self = Self::new(127_250);
    const WATER_DEPTH: Self = Self::new(128_267);
    const AIS_CLASS_A_POSITION_REPORT: Self = Self::new(129_038);
    const AIS_CLASS_B_POSITION_REPORT: Self = Self::new(129_039);

    fn transport(self) -> Option<Transport> {
        match self.number() {
            129_025 | 129_026 | 127_250 | 128_267 => Some(Transport::SingleFrame),
            129_029 | 129_038 | 129_039 => Some(Transport::FastPacket),
            _ => None,
        }
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

#[cfg(test)]
mod tests {
    use super::*;

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
