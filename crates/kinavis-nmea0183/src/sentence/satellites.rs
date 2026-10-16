//! Satellite numbers as NMEA writes them, to the kernel's [`SatelliteId`].
//!
//! NMEA 0183 numbers satellites per talker, with ranges inherited from
//! older versions: `GP` 1–32 GPS, 33–64 SBAS (PRN − 87), 65–96 GLONASS
//! (slot + 64), 193–202 QZSS (PRN); receivers also write Galileo as 301–336
//! and BeiDou as 201–263, and since NMEA 4.10 a system identifier names the
//! constellation outright. Where the last two meet, 201 and 202 under `GP`
//! or `GN` are read as BeiDou: QZSS has no satellite there, BeiDou has. A number that fits no rule is not a satellite this
//! crate can name; it is skipped, not an error.

use kinavis_kernel::{Constellation, SatelliteId};

use crate::sentence::Talker;

/// NMEA 4.10 system identifier: 1 GPS, 2 GLONASS, 3 Galileo, 4 BeiDou,
/// 5 QZSS, 6 NavIC.
fn by_system(system: u8, number: u16) -> Option<SatelliteId> {
    let constellation = match system {
        1 => return gps_family(number),
        2 => Constellation::Glonass,
        3 => Constellation::Galileo,
        4 => Constellation::BeiDou,
        5 => Constellation::Qzss,
        6 => Constellation::NavIc,
        _ => return None,
    };
    in_constellation(constellation, number)
}

/// Numbering of `GP` and `GN` without a system identifier.
fn gps_family(number: u16) -> Option<SatelliteId> {
    match number {
        1..=32 => id(Constellation::Gps, number),
        33..=64 => id(Constellation::Sbas, number + 87),
        65..=96 => id(Constellation::Glonass, number - 64),
        120..=158 => id(Constellation::Sbas, number),
        193..=200 => id(Constellation::Qzss, number - 192),
        201..=263 => id(Constellation::BeiDou, number - 200),
        301..=336 => id(Constellation::Galileo, number - 300),
        _ => None,
    }
}

/// A number in a constellation's own talker: its own range, or the offset
/// range some receivers use.
fn in_constellation(constellation: Constellation, number: u16) -> Option<SatelliteId> {
    let (offset, upper) = match constellation {
        Constellation::Glonass => (64, 96),
        Constellation::Galileo => (300, 336),
        Constellation::BeiDou => (200, 263),
        Constellation::Qzss => (192, 202),
        _ => (0, 0),
    };
    if number > offset && number <= upper {
        id(constellation, number - offset)
    } else {
        id(constellation, number)
    }
}

fn id(constellation: Constellation, number: u16) -> Option<SatelliteId> {
    SatelliteId::new(constellation, u8::try_from(number).ok()?).ok()
}

/// Satellite named by `number` in a sentence from `talker`, with the NMEA
/// 4.10 system identifier when the sentence carries one.
pub(crate) fn satellite(talker: Talker, system: Option<u8>, number: u16) -> Option<SatelliteId> {
    if let Some(system) = system {
        return by_system(system, number);
    }
    match talker.as_str() {
        "GL" => in_constellation(Constellation::Glonass, number),
        "GA" => in_constellation(Constellation::Galileo, number),
        "GB" | "BD" => in_constellation(Constellation::BeiDou, number),
        "GQ" | "QZ" | "PQ" => in_constellation(Constellation::Qzss, number),
        "GI" => in_constellation(Constellation::NavIc, number),
        _ => gps_family(number),
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;

    fn talker(letters: [u8; 2]) -> Talker {
        Talker::new(letters).unwrap()
    }

    #[test]
    fn each_talker_numbers_its_own_satellites() {
        let gp = talker(*b"GP");
        let named = |talker, system, number| {
            satellite(talker, system, number).map(|id| (id.constellation(), id.number()))
        };
        assert_eq!(named(gp, None, 7), Some((Constellation::Gps, 7)));
        assert_eq!(named(gp, None, 46), Some((Constellation::Sbas, 133)));
        assert_eq!(named(gp, None, 70), Some((Constellation::Glonass, 6)));
        assert_eq!(named(gp, None, 194), Some((Constellation::Qzss, 2)));
        assert_eq!(
            named(talker(*b"GL"), None, 79),
            Some((Constellation::Glonass, 15))
        );
        assert_eq!(
            named(talker(*b"GA"), None, 36),
            Some((Constellation::Galileo, 36))
        );
        assert_eq!(
            named(talker(*b"GB"), None, 205),
            Some((Constellation::BeiDou, 5))
        );
        assert_eq!(
            named(talker(*b"BD"), None, 13),
            Some((Constellation::BeiDou, 13))
        );
        assert_eq!(
            named(talker(*b"GN"), Some(3), 4),
            Some((Constellation::Galileo, 4))
        );
        assert_eq!(
            named(talker(*b"GN"), Some(2), 68),
            Some((Constellation::Glonass, 4))
        );
        assert_eq!(named(gp, None, 0), None);
        assert_eq!(named(gp, None, 400), None);
    }
}
