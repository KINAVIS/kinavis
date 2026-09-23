//! GNSS position data, PGN 129029: the full fix, once per second.

use core::fmt;

use kinavis_kernel::geodesy::Height;
use kinavis_kernel::gnss::{Dop, FixType, GnssFix};
use kinavis_kernel::position::Position;
use kinavis_kernel::time::{Instant, Utc};
use kinavis_kernel::units::Distance;

use crate::error::Nmea2000Error;
use crate::fields::{distance, position, resolution, value_of, Fields};
use crate::frame::Payload;

/// Satellite system(s) of a fix.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum GnssSystem {
    /// GPS.
    Gps,
    /// GLONASS.
    Glonass,
    /// GPS and GLONASS together.
    GpsAndGlonass,
    /// GPS with SBAS (WAAS, EGNOS) corrections.
    GpsWithSbas,
    /// GPS with SBAS, and GLONASS.
    GpsWithSbasAndGlonass,
    /// Chayka.
    Chayka,
    /// Integrated navigation system.
    Integrated,
    /// Surveyed position.
    Surveyed,
    /// Galileo.
    Galileo,
    /// Reserved code, kept as received.
    Reserved(u8),
}

impl GnssSystem {
    /// System for a code; the field has no "not available" value.
    #[must_use]
    pub const fn from_code(code: u8) -> Self {
        match code {
            0 => Self::Gps,
            1 => Self::Glonass,
            2 => Self::GpsAndGlonass,
            3 => Self::GpsWithSbas,
            4 => Self::GpsWithSbasAndGlonass,
            5 => Self::Chayka,
            6 => Self::Integrated,
            7 => Self::Surveyed,
            8 => Self::Galileo,
            _ => Self::Reserved(code),
        }
    }
}

impl fmt::Display for GnssSystem {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Gps => f.write_str("GPS"),
            Self::Glonass => f.write_str("GLONASS"),
            Self::GpsAndGlonass => f.write_str("GPS and GLONASS"),
            Self::GpsWithSbas => f.write_str("GPS with SBAS"),
            Self::GpsWithSbasAndGlonass => f.write_str("GPS with SBAS, and GLONASS"),
            Self::Chayka => f.write_str("Chayka"),
            Self::Integrated => f.write_str("integrated navigation"),
            Self::Surveyed => f.write_str("surveyed"),
            Self::Galileo => f.write_str("Galileo"),
            Self::Reserved(code) => write!(f, "reserved system {code}"),
        }
    }
}

/// Receiver integrity monitoring status.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Integrity {
    /// No integrity checking.
    NotChecked,
    /// Checked, safe.
    Safe,
    /// Checked, caution.
    Caution,
}

/// GNSS position data, PGN 129029.
///
/// Position, time, constellation, method and quality — the content of the
/// kernel's [`GnssFix`], built by [`GnssPosition::fix`] when time and position
/// are both present.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct GnssPosition {
    /// Sequence identifier, if sent.
    pub sid: Option<u8>,
    /// Fix time, from the date in days and time of day in 1e-4 s.
    pub taken_at: Option<Instant<Utc>>,
    /// Position, 1e-16° resolution.
    pub position: Option<Position>,
    /// Height above the WGS 84 ellipsoid, 1e-6 m resolution.
    pub altitude: Option<Height>,
    /// Satellite system.
    pub system: GnssSystem,
    /// Fix method; `None` for a code without a kernel equivalent.
    pub method: Option<FixType>,
    /// Integrity status; `None` for a reserved code.
    pub integrity: Option<Integrity>,
    /// Satellites in use.
    pub satellites: Option<u8>,
    /// HDOP, 0.01 resolution.
    pub hdop: Option<Dop>,
    /// PDOP, 0.01 resolution.
    pub pdop: Option<Dop>,
    /// Geoidal separation, 0.01 m resolution; altitude minus this is height
    /// above MSL.
    pub geoidal_separation: Option<Distance>,
    /// Number of reference stations that follow (not read).
    pub reference_stations: Option<u8>,
}

impl GnssPosition {
    /// Fixed part of the group, before the reference stations.
    const NEEDED: usize = 43;

    pub(crate) fn decode(payload: &Payload) -> Result<Self, Nmea2000Error> {
        let field = Fields::of(payload, Self::NEEDED)?;
        // Four bits.
        #[allow(clippy::cast_possible_truncation)]
        let system = GnssSystem::from_code(field.raw_bits(31 * 8, 4) as u8);
        let method = match field.raw_bits(31 * 8 + 4, 4) {
            0 => Some(FixType::None),
            1 => Some(FixType::Autonomous),
            2 => Some(FixType::Differential),
            3 => Some(FixType::Precise),
            4 => Some(FixType::RtkFixed),
            5 => Some(FixType::RtkFloat),
            6 => Some(FixType::Estimated),
            7 => Some(FixType::Manual),
            8 => Some(FixType::Simulated),
            _ => None,
        };
        let integrity = match field.raw_bits(32 * 8, 2) {
            0 => Some(Integrity::NotChecked),
            1 => Some(Integrity::Safe),
            2 => Some(Integrity::Caution),
            _ => None,
        };
        Ok(Self {
            sid: field.u8(0),
            taken_at: taken_at(field.unsigned(1, 2), field.unsigned(3, 4)),
            position: position(
                field.signed(7, 8),
                field.signed(15, 8),
                resolution::PRECISE_POSITION_DEG,
            )?,
            altitude: distance(field.signed(23, 8), resolution::ALTITUDE_M, "altitude")?
                .map(Height::above_ellipsoid),
            system,
            method,
            integrity,
            satellites: field.u8(33),
            hdop: dop(field.signed(34, 2))?,
            pdop: dop(field.signed(36, 2))?,
            geoidal_separation: distance(
                field.signed(38, 4),
                resolution::CENTIMETRE_M,
                "geoidal separation",
            )?,
            reference_stations: field.u8(42),
        })
    }

    /// Kernel fix, if time and position are both present.
    #[must_use]
    pub fn fix(&self) -> Option<GnssFix> {
        let mut builder = GnssFix::builder(self.taken_at?, self.position?);
        if let Some(method) = self.method {
            builder = builder.fix_type(method);
        }
        if let Some(satellites) = self.satellites {
            builder = builder.satellites(satellites);
        }
        if let Some(hdop) = self.hdop {
            builder = builder.hdop(hdop);
        }
        if let Some(pdop) = self.pdop {
            builder = builder.pdop(pdop);
        }
        Some(builder.build())
    }
}

/// Date in days since 1970 and time of day in 1e-4 s.
fn taken_at(days: Option<u64>, time: Option<u64>) -> Option<Instant<Utc>> {
    let days = i64::try_from(days?).ok()?;
    let time = i64::try_from(time?).ok()?;
    // 1e-4 s = 100 000 ns.
    let nanos = days
        .checked_mul(86_400 * 1_000_000_000)?
        .checked_add(time.checked_mul(100_000)?)?;
    Some(Instant::from_unix_nanos(nanos))
}

/// Dilution of precision in 0.01 units.
fn dop(field: Option<i64>) -> Result<Option<Dop>, Nmea2000Error> {
    let Some(field) = field else {
        return Ok(None);
    };
    // At most 16 bits.
    #[allow(clippy::cast_precision_loss)]
    Dop::new(field as f64 * resolution::DOP)
        .map(Some)
        .map_err(value_of("dilution of precision"))
}
