//! Navigation results, class `NAV` (0x01).

use core::time::Duration;

use kinavis_kernel::geodesy::{GeodeticPoint, Height};
use kinavis_kernel::gnss::{Dop, FixType, GnssFix};
use kinavis_kernel::local::{Ned, Vector3};
use kinavis_kernel::snapshot::ErrorEllipse;
use kinavis_kernel::time::{Civil, Instant, LeapSeconds, Utc};
use kinavis_kernel::{
    Angle, Constellation, Distance, KernelError, Latitude, Longitude, Position, SatelliteId,
    SatelliteSet, SatelliteView, Speed, TrueBearing, TrueCourse, Variation,
};

use crate::error::{value, TranslationError, UbxError};
use crate::read::{i16_at, i32_at, i8_at, u16_at, u32_at, u8_at};

/// Class `NAV`.
pub const CLASS: u8 = 0x01;
/// `NAV-STATUS`.
pub const STATUS: u8 = 0x03;
/// `NAV-DOP`.
pub const DOP: u8 = 0x04;
/// `NAV-PVT`.
pub const PVT: u8 = 0x07;
/// `NAV-HPPOSLLH`.
pub const HPPOSLLH: u8 = 0x14;
/// `NAV-TIMELS`.
pub const TIMELS: u8 = 0x26;
/// `NAV-SAT`.
pub const SAT: u8 = 0x35;
/// `NAV-RELPOSNED`.
pub const RELPOSNED: u8 = 0x3C;

/// Payload reader that reports a short payload as a length error.
struct Payload<'a> {
    bytes: &'a [u8],
    id: u8,
    expected: usize,
}

impl<'a> Payload<'a> {
    fn new(bytes: &'a [u8], id: u8, lengths: &[usize]) -> Result<Self, UbxError> {
        if lengths.contains(&bytes.len()) {
            Ok(Self {
                bytes,
                id,
                expected: bytes.len(),
            })
        } else {
            Err(UbxError::Length {
                class: CLASS,
                id,
                expected: lengths.first().copied().unwrap_or(0),
                found: bytes.len(),
            })
        }
    }

    fn short(&self) -> UbxError {
        UbxError::Length {
            class: CLASS,
            id: self.id,
            expected: self.expected,
            found: self.bytes.len(),
        }
    }

    fn u8(&self, offset: usize) -> Result<u8, UbxError> {
        u8_at(self.bytes, offset).ok_or_else(|| self.short())
    }
    fn i8(&self, offset: usize) -> Result<i8, UbxError> {
        i8_at(self.bytes, offset).ok_or_else(|| self.short())
    }
    fn u16(&self, offset: usize) -> Result<u16, UbxError> {
        u16_at(self.bytes, offset).ok_or_else(|| self.short())
    }
    fn i16(&self, offset: usize) -> Result<i16, UbxError> {
        i16_at(self.bytes, offset).ok_or_else(|| self.short())
    }
    fn u32(&self, offset: usize) -> Result<u32, UbxError> {
        u32_at(self.bytes, offset).ok_or_else(|| self.short())
    }
    fn i32(&self, offset: usize) -> Result<i32, UbxError> {
        i32_at(self.bytes, offset).ok_or_else(|| self.short())
    }

    /// Millimetres as a distance.
    fn millimetres(&self, offset: usize, field: &'static str) -> Result<Distance, UbxError> {
        Distance::from_metres(f64::from(self.i32(offset)?) / 1000.0).map_err(value(field))
    }

    /// Unsigned millimetres as a distance.
    fn accuracy(&self, offset: usize, field: &'static str) -> Result<Distance, UbxError> {
        Distance::from_metres(f64::from(self.u32(offset)?) / 1000.0).map_err(value(field))
    }

    /// Millimetres per second as a speed.
    fn velocity(&self, offset: usize, field: &'static str) -> Result<Speed, UbxError> {
        Speed::from_metres_per_second(f64::from(self.i32(offset)?) / 1000.0).map_err(value(field))
    }

    /// DOP in hundredths; zero reads as not available.
    fn dop(&self, offset: usize, field: &'static str) -> Result<Option<Dop>, UbxError> {
        let raw = self.u16(offset)?;
        if raw == 0 {
            return Ok(None);
        }
        Dop::new(f64::from(raw) / 100.0)
            .map(Some)
            .map_err(value(field))
    }
}

/// Position from latitude and longitude in degrees.
fn position(latitude: f64, longitude: f64) -> Result<Position, UbxError> {
    Ok(Position::new(
        Latitude::from_degrees(latitude).map_err(value("latitude"))?,
        Longitude::from_degrees(longitude).map_err(value("longitude"))?,
    ))
}

/// Carrier-phase range solution.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CarrierSolution {
    /// No carrier-phase solution.
    None,
    /// RTK float: ambiguities not resolved.
    Float,
    /// RTK fixed: ambiguities resolved.
    Fixed,
}

impl CarrierSolution {
    const fn from_bits(bits: u8) -> Self {
        match bits & 0b11 {
            1 => Self::Float,
            2 => Self::Fixed,
            _ => Self::None,
        }
    }
}

/// `NAV-PVT` fix type.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PvtFix {
    /// No fix.
    NoFix,
    /// Dead reckoning only.
    DeadReckoning,
    /// 2D.
    TwoD,
    /// 3D.
    ThreeD,
    /// GNSS combined with dead reckoning.
    GnssDeadReckoning,
    /// Time only (fixed position).
    TimeOnly,
    /// A value the protocol does not define.
    Other(u8),
}

impl PvtFix {
    const fn from_byte(byte: u8) -> Self {
        match byte {
            0 => Self::NoFix,
            1 => Self::DeadReckoning,
            2 => Self::TwoD,
            3 => Self::ThreeD,
            4 => Self::GnssDeadReckoning,
            5 => Self::TimeOnly,
            other => Self::Other(other),
        }
    }
}

/// `NAV-PVT`: position, velocity and time in one message.
// The protocol's flags, one field each.
#[allow(clippy::struct_excessive_bools)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NavPvt {
    /// GPS time of week of the epoch, ms.
    pub itow: u32,
    /// UTC date and time as written: year, month, day, hour, minute, second.
    pub date_time: (u16, u8, u8, u8, u8, u8),
    /// Fraction of the second, ns, −10⁹ to 10⁹.
    pub nanos: i32,
    /// UTC date valid.
    pub valid_date: bool,
    /// UTC time of day valid.
    pub valid_time: bool,
    /// Time accuracy estimate, ns.
    pub time_accuracy: u32,
    /// Fix type.
    pub fix: PvtFix,
    /// Fix within the receiver's DOP and accuracy masks.
    pub fix_ok: bool,
    /// Differential corrections applied.
    pub differential: bool,
    /// Carrier-phase solution.
    pub carrier: CarrierSolution,
    /// Satellites used.
    pub satellites: u8,
    /// Position.
    pub position: Position,
    /// Height above the ellipsoid.
    pub height: Distance,
    /// Height above mean sea level, by the receiver's geoid model.
    pub height_msl: Distance,
    /// Horizontal accuracy estimate.
    pub horizontal_accuracy: Distance,
    /// Vertical accuracy estimate.
    pub vertical_accuracy: Distance,
    /// Velocity, NED.
    pub velocity: Vector3<Ned, Speed>,
    /// Ground speed.
    pub ground_speed: Speed,
    /// Heading of motion.
    pub heading_of_motion: TrueCourse,
    /// Speed accuracy estimate.
    pub speed_accuracy: Speed,
    /// Heading accuracy estimate.
    pub heading_accuracy: Angle,
    /// Position DOP.
    pub pdop: Option<Dop>,
    /// Latitude, longitude and height are not valid.
    pub invalid_llh: bool,
    /// Heading of the vehicle (dual antenna or sensor fusion), when valid.
    pub vehicle_heading: Option<TrueCourse>,
    /// Magnetic declination, when the receiver has a valid model.
    pub declination: Option<Variation>,
}

impl NavPvt {
    /// Payload of 92 bytes; 84 before protocol 15, without vehicle heading
    /// and declination.
    pub(crate) fn decode(bytes: &[u8]) -> Result<Self, UbxError> {
        let p = Payload::new(bytes, PVT, &[92, 84])?;
        let valid = p.u8(11)?;
        let flags = p.u8(21)?;
        let full = bytes.len() == 92;
        let latitude = f64::from(p.i32(28)?) * 1e-7;
        let longitude = f64::from(p.i32(24)?) * 1e-7;
        let vehicle_heading = if full && flags & 0x20 != 0 {
            Some(TrueCourse::wrap(f64::from(p.i32(84)?) * 1e-5).map_err(value("vehicle heading"))?)
        } else {
            None
        };
        let declination = if full && valid & 0x08 != 0 {
            Some(Variation::new(f64::from(p.i16(88)?) * 0.01).map_err(value("declination"))?)
        } else {
            None
        };
        Ok(Self {
            itow: p.u32(0)?,
            date_time: (p.u16(4)?, p.u8(6)?, p.u8(7)?, p.u8(8)?, p.u8(9)?, p.u8(10)?),
            nanos: p.i32(16)?,
            valid_date: valid & 0x01 != 0,
            valid_time: valid & 0x02 != 0,
            time_accuracy: p.u32(12)?,
            fix: PvtFix::from_byte(p.u8(20)?),
            fix_ok: flags & 0x01 != 0,
            differential: flags & 0x02 != 0,
            carrier: CarrierSolution::from_bits(flags >> 6),
            satellites: p.u8(23)?,
            position: position(latitude, longitude)?,
            height: p.millimetres(32, "height")?,
            height_msl: p.millimetres(36, "height above mean sea level")?,
            horizontal_accuracy: p.accuracy(40, "horizontal accuracy")?,
            vertical_accuracy: p.accuracy(44, "vertical accuracy")?,
            velocity: Vector3::new(
                p.velocity(48, "north velocity")?,
                p.velocity(52, "east velocity")?,
                p.velocity(56, "down velocity")?,
            ),
            ground_speed: p.velocity(60, "ground speed")?,
            heading_of_motion: TrueCourse::wrap(f64::from(p.i32(64)?) * 1e-5)
                .map_err(value("heading of motion"))?,
            speed_accuracy: Speed::from_metres_per_second(f64::from(p.u32(68)?) / 1000.0)
                .map_err(value("speed accuracy"))?,
            heading_accuracy: Angle::from_degrees(f64::from(p.u32(72)?) * 1e-5)
                .map_err(value("heading accuracy"))?,
            pdop: p.dop(76, "position DOP")?,
            invalid_llh: full && p.u16(78)? & 0x01 != 0,
            vehicle_heading,
            declination,
        })
    }

    /// UTC instant of the epoch.
    ///
    /// # Errors
    ///
    /// [`TranslationError::NoTime`] unless the receiver marks both date and
    /// time valid; [`TranslationError::BadMoment`] if they form no instant.
    pub fn instant(&self) -> Result<Instant<Utc>, TranslationError> {
        if !(self.valid_date && self.valid_time) {
            return Err(TranslationError::NoTime);
        }
        let (year, month, day, hour, minute, second) = self.date_time;
        let whole = Instant::<Utc>::from_civil(Civil {
            year: i32::from(year),
            month,
            day,
            hour,
            minute,
            second,
            nanos: 0,
        })
        .map_err(TranslationError::BadMoment)?;
        let fraction = Duration::from_nanos(u64::from(self.nanos.unsigned_abs()));
        let instant = if self.nanos < 0 {
            whole.checked_sub(fraction)
        } else {
            whole.checked_add(fraction)
        };
        instant.ok_or(TranslationError::BadMoment(KernelError::Unrepresentable {
            what: "the epoch",
        }))
    }

    /// Fix type in the kernel's terms: none unless the receiver marks the fix
    /// within its masks; RTK from the carrier-phase solution.
    #[must_use]
    pub fn fix_type(&self) -> FixType {
        match self.fix {
            PvtFix::DeadReckoning => FixType::Estimated,
            PvtFix::TwoD | PvtFix::ThreeD | PvtFix::GnssDeadReckoning
                if self.fix_ok && !self.invalid_llh =>
            {
                match self.carrier {
                    CarrierSolution::Fixed => FixType::RtkFixed,
                    CarrierSolution::Float => FixType::RtkFloat,
                    CarrierSolution::None if self.differential => FixType::Differential,
                    CarrierSolution::None => FixType::Autonomous,
                }
            }
            _ => FixType::None,
        }
    }

    /// The fix as a kernel [`GnssFix`]: position, course and speed over
    /// ground, satellites, PDOP, and the horizontal accuracy as a circular
    /// error ellipse.
    ///
    /// `hAcc` is read as the root-sum-square horizontal error, so each axis of
    /// the circle is `hAcc / √2`.
    ///
    /// # Errors
    ///
    /// As [`NavPvt::instant`].
    pub fn fix(&self) -> Result<GnssFix, TranslationError> {
        let per_axis = self.horizontal_accuracy * core::f64::consts::FRAC_1_SQRT_2;
        let mut fix = GnssFix::builder(self.instant()?, self.position)
            .fix_type(self.fix_type())
            .course_over_ground(self.heading_of_motion)
            .speed_over_ground(self.ground_speed)
            .satellites(self.satellites)
            .horizontal_error(ErrorEllipse::circular(per_axis));
        if let Some(pdop) = self.pdop {
            fix = fix.pdop(pdop);
        }
        Ok(fix.build())
    }

    /// Position with its height above the ellipsoid.
    #[must_use]
    pub fn geodetic_point(&self) -> GeodeticPoint {
        GeodeticPoint::new(self.position, Height::above_ellipsoid(self.height))
    }
}

/// `NAV-HPPOSLLH`: position to a tenth of a millimetre.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NavHpposllh {
    /// GPS time of week, ms.
    pub itow: u32,
    /// Position, to 10⁻⁹°.
    pub position: Position,
    /// Height above the ellipsoid.
    pub height: Distance,
    /// Height above mean sea level.
    pub height_msl: Distance,
    /// Horizontal accuracy estimate.
    pub horizontal_accuracy: Distance,
    /// Vertical accuracy estimate.
    pub vertical_accuracy: Distance,
    /// Latitude, longitude and heights are not valid.
    pub invalid_llh: bool,
}

impl NavHpposllh {
    pub(crate) fn decode(bytes: &[u8]) -> Result<Self, UbxError> {
        let p = Payload::new(bytes, HPPOSLLH, &[36])?;
        // Whole part in 1e-7° or mm, fine part in 1e-9° or 0.1 mm.
        let tenth_mm =
            |whole: i32, fine: i8| f64::from(whole) / 1000.0 + f64::from(fine) / 10_000.0;
        let latitude = f64::from(p.i32(12)?) * 1e-7 + f64::from(p.i8(25)?) * 1e-9;
        let longitude = f64::from(p.i32(8)?) * 1e-7 + f64::from(p.i8(24)?) * 1e-9;
        Ok(Self {
            itow: p.u32(4)?,
            position: position(latitude, longitude)?,
            height: Distance::from_metres(tenth_mm(p.i32(16)?, p.i8(26)?))
                .map_err(value("height"))?,
            height_msl: Distance::from_metres(tenth_mm(p.i32(20)?, p.i8(27)?))
                .map_err(value("height above mean sea level"))?,
            horizontal_accuracy: Distance::from_metres(f64::from(p.u32(28)?) / 10_000.0)
                .map_err(value("horizontal accuracy"))?,
            vertical_accuracy: Distance::from_metres(f64::from(p.u32(32)?) / 10_000.0)
                .map_err(value("vertical accuracy"))?,
            invalid_llh: p.u8(3)? & 0x01 != 0,
        })
    }
}

/// `NAV-RELPOSNED`: the vector from a reference station or moving base to
/// this receiver, and with two antennas the heading between them.
// The protocol's flags, one field each.
#[allow(clippy::struct_excessive_bools)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NavRelposned {
    /// Reference station identifier.
    pub reference_station: u16,
    /// GPS time of week, ms.
    pub itow: u32,
    /// Base to rover, NED, to 0.1 mm.
    pub relative: Vector3<Ned, Distance>,
    /// Accuracy of each component.
    pub accuracy: Vector3<Ned, Distance>,
    /// Length of the vector (version 1).
    pub length: Option<Distance>,
    /// Heading of the vector, when the receiver marks it valid (version 1).
    pub heading: Option<TrueCourse>,
    /// Heading accuracy (version 1).
    pub heading_accuracy: Option<Angle>,
    /// A valid fix.
    pub fix_ok: bool,
    /// Differential corrections applied.
    pub differential: bool,
    /// The relative position is valid.
    pub valid: bool,
    /// Carrier-phase solution.
    pub carrier: CarrierSolution,
    /// The base is moving (moving-base mode).
    pub moving_base: bool,
}

impl NavRelposned {
    /// Version 0 (40 bytes, M8P) or 1 (64 bytes, F9).
    pub(crate) fn decode(bytes: &[u8]) -> Result<Self, UbxError> {
        let p = Payload::new(bytes, RELPOSNED, &[64, 40])?;
        let version = p.u8(0)?;
        let v1 = match (version, bytes.len()) {
            (1, 64) => true,
            (0, 40) => false,
            _ => {
                return Err(UbxError::Version {
                    class: CLASS,
                    id: RELPOSNED,
                    version,
                })
            }
        };
        let (fine, accuracy, flags) = if v1 { (32, 36, 60) } else { (20, 24, 36) };
        let component = |coarse: usize, fine_at: usize, field| -> Result<Distance, UbxError> {
            Distance::from_metres(
                f64::from(p.i32(coarse)?) / 100.0 + f64::from(p.i8(fine_at)?) / 10_000.0,
            )
            .map_err(value(field))
        };
        let tenth_mm = |offset: usize, field| -> Result<Distance, UbxError> {
            Distance::from_metres(f64::from(p.u32(offset)?) / 10_000.0).map_err(value(field))
        };
        let flags = p.u32(flags)?;
        let (length, heading, heading_accuracy) = if v1 {
            let heading = if flags & 0x100 != 0 {
                Some(TrueCourse::wrap(f64::from(p.i32(24)?) * 1e-5).map_err(value("heading"))?)
            } else {
                None
            };
            (
                Some(component(20, 35, "length")?),
                heading,
                Some(
                    Angle::from_degrees(f64::from(p.u32(52)?) * 1e-5)
                        .map_err(value("heading accuracy"))?,
                ),
            )
        } else {
            (None, None, None)
        };
        Ok(Self {
            reference_station: p.u16(2)?,
            itow: p.u32(4)?,
            relative: Vector3::new(
                component(8, fine, "north")?,
                component(12, fine + 1, "east")?,
                component(16, fine + 2, "down")?,
            ),
            accuracy: Vector3::new(
                tenth_mm(accuracy, "north accuracy")?,
                tenth_mm(accuracy + 4, "east accuracy")?,
                tenth_mm(accuracy + 8, "down accuracy")?,
            ),
            length,
            heading,
            heading_accuracy,
            fix_ok: flags & 0x01 != 0,
            differential: flags & 0x02 != 0,
            valid: flags & 0x04 != 0,
            carrier: CarrierSolution::from_bits(u8::try_from((flags >> 3) & 0b11).unwrap_or(0)),
            moving_base: flags & 0x20 != 0,
        })
    }
}

/// One satellite of a `NAV-SAT`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct SatInfo {
    /// u-blox GNSS identifier: 0 GPS, 1 SBAS, 2 Galileo, 3 BeiDou, 4 IMES,
    /// 5 QZSS, 6 GLONASS, 7 NavIC.
    pub gnss: u8,
    /// Satellite number within the GNSS.
    pub sv: u8,
    /// Carrier-to-noise density, dB-Hz.
    pub cno: u8,
    /// Elevation, degrees; outside ±90 when unknown.
    pub elevation: i8,
    /// Azimuth, degrees; outside 0–360 when unknown.
    pub azimuth: i16,
    /// Pseudorange residual, decimetres.
    pub residual_decimetres: i16,
    /// Flags as sent.
    pub flags: u32,
}

impl SatInfo {
    /// Used in the navigation solution (flag bit 3).
    #[must_use]
    pub const fn used(&self) -> bool {
        self.flags & 0x08 != 0
    }

    /// The kernel's identifier; `None` for IMES and unknown systems.
    #[must_use]
    pub fn id(&self) -> Option<SatelliteId> {
        let constellation = match self.gnss {
            0 => Constellation::Gps,
            1 => Constellation::Sbas,
            2 => Constellation::Galileo,
            3 => Constellation::BeiDou,
            5 => Constellation::Qzss,
            6 => Constellation::Glonass,
            7 => Constellation::NavIc,
            _ => return None,
        };
        SatelliteId::new(constellation, self.sv).ok()
    }

    /// The kernel's view: sky position where known, signal, use, and the
    /// residual of a used satellite.
    #[must_use]
    pub fn view(&self) -> Option<SatelliteView> {
        let elevation = (-90..=90)
            .contains(&self.elevation)
            .then(|| Angle::from_degrees(f64::from(self.elevation)).ok())
            .flatten();
        let azimuth = (0..360)
            .contains(&self.azimuth)
            .then(|| TrueBearing::new(f64::from(self.azimuth)).ok())
            .flatten();
        let mut view = SatelliteView::new(self.id()?, elevation, azimuth)
            .ok()?
            .with_carrier_to_noise(f64::from(self.cno))
            .ok()?
            .used_in_fix(self.used());
        if self.used() {
            let residual =
                Distance::from_metres(f64::from(self.residual_decimetres) / 10.0).ok()?;
            view = view.with_residual(residual);
        }
        Some(view)
    }
}

/// `NAV-SAT`: every satellite tracked.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NavSat<'a> {
    /// GPS time of week, ms.
    pub itow: u32,
    blocks: &'a [u8],
}

impl<'a> NavSat<'a> {
    pub(crate) fn decode(bytes: &'a [u8]) -> Result<Self, UbxError> {
        let count = usize::from(u8_at(bytes, 5).unwrap_or(0));
        let expected = count.saturating_mul(12).saturating_add(8);
        if bytes.len() != expected {
            return Err(UbxError::Length {
                class: CLASS,
                id: SAT,
                expected,
                found: bytes.len(),
            });
        }
        Ok(Self {
            itow: u32_at(bytes, 0).unwrap_or(0),
            blocks: bytes.get(8..).unwrap_or(&[]),
        })
    }

    /// The satellites, as sent.
    pub fn satellites(&self) -> impl Iterator<Item = SatInfo> + 'a {
        self.blocks.chunks_exact(12).map(|block| SatInfo {
            gnss: u8_at(block, 0).unwrap_or(0),
            sv: u8_at(block, 1).unwrap_or(0),
            cno: u8_at(block, 2).unwrap_or(0),
            elevation: i8_at(block, 3).unwrap_or(i8::MIN),
            azimuth: i16_at(block, 4).unwrap_or(i16::MIN),
            residual_decimetres: i16_at(block, 6).unwrap_or(0),
            flags: u32_at(block, 8).unwrap_or(0),
        })
    }

    /// The satellites the kernel can name, as a set.
    ///
    /// # Errors
    ///
    /// [`KernelError::CapacityExceeded`] if more than `N`.
    pub fn satellite_set<const N: usize>(&self) -> Result<SatelliteSet<N>, KernelError> {
        let mut set = SatelliteSet::new();
        for view in self.satellites().filter_map(|info| info.view()) {
            set.insert(view)?;
        }
        Ok(set)
    }
}

/// `NAV-DOP`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NavDop {
    /// GPS time of week, ms.
    pub itow: u32,
    /// Geometric, position, time, vertical, horizontal, north and east DOP.
    pub gdop: Option<Dop>,
    /// Position DOP.
    pub pdop: Option<Dop>,
    /// Time DOP.
    pub tdop: Option<Dop>,
    /// Vertical DOP.
    pub vdop: Option<Dop>,
    /// Horizontal DOP.
    pub hdop: Option<Dop>,
    /// North DOP.
    pub ndop: Option<Dop>,
    /// East DOP.
    pub edop: Option<Dop>,
}

impl NavDop {
    pub(crate) fn decode(bytes: &[u8]) -> Result<Self, UbxError> {
        let p = Payload::new(bytes, DOP, &[18])?;
        Ok(Self {
            itow: p.u32(0)?,
            gdop: p.dop(4, "geometric DOP")?,
            pdop: p.dop(6, "position DOP")?,
            tdop: p.dop(8, "time DOP")?,
            vdop: p.dop(10, "vertical DOP")?,
            hdop: p.dop(12, "horizontal DOP")?,
            ndop: p.dop(14, "north DOP")?,
            edop: p.dop(16, "east DOP")?,
        })
    }
}

/// `NAV-STATUS`.
// The protocol's flags, one field each.
#[allow(clippy::struct_excessive_bools)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NavStatus {
    /// GPS time of week, ms.
    pub itow: u32,
    /// Fix type, as `NAV-PVT`.
    pub fix: PvtFix,
    /// Fix within the receiver's masks.
    pub fix_ok: bool,
    /// Differential corrections applied.
    pub differential: bool,
    /// GPS week number valid.
    pub week_valid: bool,
    /// Time of week valid.
    pub time_of_week_valid: bool,
    /// Carrier-phase solution.
    pub carrier: CarrierSolution,
    /// Time to first fix, ms.
    pub time_to_first_fix: u32,
    /// Milliseconds since start-up or reset.
    pub since_startup: u32,
}

impl NavStatus {
    pub(crate) fn decode(bytes: &[u8]) -> Result<Self, UbxError> {
        let p = Payload::new(bytes, STATUS, &[16])?;
        let flags = p.u8(5)?;
        Ok(Self {
            itow: p.u32(0)?,
            fix: PvtFix::from_byte(p.u8(4)?),
            fix_ok: flags & 0x01 != 0,
            differential: flags & 0x02 != 0,
            week_valid: flags & 0x04 != 0,
            time_of_week_valid: flags & 0x08 != 0,
            carrier: CarrierSolution::from_bits(p.u8(7)? >> 6),
            time_to_first_fix: p.u32(8)?,
            since_startup: p.u32(12)?,
        })
    }
}

/// `NAV-TIMELS`: the leap second count and the next change.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NavTimels {
    /// GPS time of week, ms.
    pub itow: u32,
    /// `GPS − UTC`, s, when valid.
    pub current: Option<i8>,
    /// Change at the next event, s: −1, 0 or +1.
    pub change: i8,
    /// Seconds from this epoch to the next event, when valid.
    pub time_to_event: Option<i32>,
}

impl NavTimels {
    pub(crate) fn decode(bytes: &[u8]) -> Result<Self, UbxError> {
        let p = Payload::new(bytes, TIMELS, &[24])?;
        let version = p.u8(4)?;
        if version != 0 {
            return Err(UbxError::Version {
                class: CLASS,
                id: TIMELS,
                version,
            });
        }
        let valid = p.u8(23)?;
        Ok(Self {
            itow: p.u32(0)?,
            current: (valid & 0x01 != 0).then_some(p.i8(9)?),
            change: p.i8(11)?,
            time_to_event: (valid & 0x02 != 0).then_some(p.i32(12)?),
        })
    }

    /// The receiver's leap second table, from `received` (the UTC instant of
    /// this epoch) onwards; `None` without a valid count.
    #[must_use]
    pub fn leap_seconds(&self, received: Instant<Utc>) -> Option<ReceiverLeapSeconds> {
        let current = self.current?;
        let next = match self.time_to_event {
            Some(seconds) if self.change != 0 && seconds >= 0 => received
                .checked_add(Duration::from_secs(u64::from(seconds.unsigned_abs())))
                .map(|at| (at, self.change)),
            _ => None,
        };
        Some(ReceiverLeapSeconds {
            from: received,
            gps_minus_utc: current,
            next,
        })
    }
}

/// `TAI − UTC` as the receiver knows it: valid from the instant of the
/// `NAV-TIMELS` it came from, including the next announced change.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ReceiverLeapSeconds {
    from: Instant<Utc>,
    gps_minus_utc: i8,
    next: Option<(Instant<Utc>, i8)>,
}

impl LeapSeconds for ReceiverLeapSeconds {
    fn tai_minus_utc(&self, at: Instant<Utc>) -> kinavis_kernel::Result<Duration> {
        if at < self.from {
            return Err(KernelError::OutsideValidity {
                data: "receiver leap second count",
            });
        }
        let tai_minus_gps =
            i64::try_from(kinavis_kernel::time::TAI_MINUS_GPS.as_secs()).unwrap_or(19);
        let mut seconds = tai_minus_gps.saturating_add(i64::from(self.gps_minus_utc));
        if let Some((event, change)) = self.next {
            if at >= event {
                seconds = seconds.saturating_add(i64::from(change));
            }
        }
        Ok(Duration::from_secs(u64::try_from(seconds).unwrap_or(0)))
    }
}
