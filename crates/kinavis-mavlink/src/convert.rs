//! Messages to and from kernel types: units and the datum of height only,
//! never axes (ADR-0020). MAVLink is NED like the kernel.

use kinavis_kernel::error::{KernelError, Result};
use kinavis_kernel::geodesy::VerticalDatum;
use kinavis_kernel::{
    Angle, Distance, FixType, GeodeticPoint, GnssFix, Height, Instant, Latitude, Longitude, Ned,
    Position, RateOfTurn, Speed, TrueCourse, Utc, Vector3,
};

use crate::messages::{
    Attitude, GlobalPositionInt, GpsRawInt, GpsRtcmData, LocalPositionNed, MissionItemInt,
    SetPositionTargetGlobalInt,
};

/// `MAV_FRAME_GLOBAL_INT`: latitude and longitude in 1e-7°, altitude above
/// mean sea level.
pub const FRAME_GLOBAL_INT: u8 = 5;

/// `MAV_CMD_NAV_WAYPOINT`.
pub const CMD_NAV_WAYPOINT: u16 = 16;

/// Most RTCM bytes one `GPS_RTCM_DATA` carries.
pub const RTCM_FRAGMENT_BYTES: usize = 180;

/// Most RTCM bytes of one correction message: four fragments.
pub const RTCM_MAX_BYTES: usize = 4 * RTCM_FRAGMENT_BYTES;

fn position(lat: i32, lon: i32) -> Result<Position> {
    Ok(Position::new(
        Latitude::from_degrees(f64::from(lat) * 1e-7)?,
        Longitude::from_degrees(f64::from(lon) * 1e-7)?,
    ))
}

/// Degrees in 1e-7°; within ±1.8·10⁹ for valid angles.
fn degrees_e7(degrees: f64) -> i32 {
    kinavis_kernel::math::round_to_i32(degrees * 1e7)
}

fn millimetres(value: i32) -> Result<Distance> {
    Distance::from_metres(f64::from(value) * 1e-3)
}

fn centimetres_per_second(value: i16) -> Result<Speed> {
    Speed::from_metres_per_second(f64::from(value) * 1e-2)
}

impl GlobalPositionInt {
    /// Position and height above mean sea level; convert the height with a
    /// geoid model (`GeodeticPoint::to_ellipsoidal`) where the ellipsoid is
    /// wanted.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] for a latitude or longitude off the Earth.
    pub fn point(&self) -> Result<GeodeticPoint> {
        Ok(GeodeticPoint::new(
            position(self.lat, self.lon)?,
            Height::above_mean_sea_level(millimetres(self.alt)?),
        ))
    }

    /// Height above the home position.
    ///
    /// # Errors
    ///
    /// Not reachable: every `i32` of millimetres is a distance.
    pub fn relative_height(&self) -> Result<Distance> {
        millimetres(self.relative_alt)
    }

    /// Ground velocity, NED.
    ///
    /// # Errors
    ///
    /// Not reachable for any `i16` of cm/s.
    pub fn velocity(&self) -> Result<Vector3<Ned, Speed>> {
        Ok(Vector3::new(
            centimetres_per_second(self.vx)?,
            centimetres_per_second(self.vy)?,
            centimetres_per_second(self.vz)?,
        ))
    }

    /// Heading, if known.
    #[must_use]
    pub fn heading(&self) -> Option<TrueCourse> {
        (self.hdg != u16::MAX).then(|| TrueCourse::from_degrees_wrapped(f64::from(self.hdg) * 0.01))
    }
}

impl GpsRawInt {
    /// Fix type, as the kernel names it; `None` for no fix.
    #[must_use]
    pub const fn fix(&self) -> FixType {
        match self.fix_type {
            2 | 3 => FixType::Autonomous,
            4 => FixType::Differential,
            5 => FixType::RtkFloat,
            6 => FixType::RtkFixed,
            8 => FixType::Precise,
            _ => FixType::None,
        }
    }

    /// The fix as the kernel's `GnssFix`, taken at `at` (the message's
    /// `time_usec` may be time since boot); `None` without a position fix.
    ///
    /// Course and speed, HDOP and VDOP, satellites when known.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] for a position off the Earth.
    pub fn gnss_fix(&self, at: Instant<Utc>) -> Result<Option<GnssFix>> {
        let fix = self.fix();
        if !fix.is_position_fix() {
            return Ok(None);
        }
        let mut builder = GnssFix::builder(at, position(self.lat, self.lon)?).fix_type(fix);
        if self.vel != u16::MAX {
            builder = builder
                .speed_over_ground(Speed::from_metres_per_second(f64::from(self.vel) * 0.01)?);
        }
        if self.cog != u16::MAX {
            builder = builder
                .course_over_ground(TrueCourse::from_degrees_wrapped(f64::from(self.cog) * 0.01));
        }
        if self.satellites_visible != u8::MAX {
            builder = builder.satellites(self.satellites_visible);
        }
        if self.eph != u16::MAX {
            builder = builder.hdop(kinavis_kernel::Dop::new(f64::from(self.eph) * 0.01)?);
        }
        if self.epv != u16::MAX {
            builder = builder.vdop(kinavis_kernel::Dop::new(f64::from(self.epv) * 0.01)?);
        }
        Ok(Some(builder.build()))
    }

    /// Height above the ellipsoid, if the receiver sends it (an extension:
    /// zero from those without it reads as `None`).
    ///
    /// # Errors
    ///
    /// Not reachable.
    pub fn ellipsoidal_height(&self) -> Result<Option<Height>> {
        if self.alt_ellipsoid == 0 {
            return Ok(None);
        }
        Ok(Some(Height::above_ellipsoid(millimetres(
            self.alt_ellipsoid,
        )?)))
    }

    /// Height above mean sea level.
    ///
    /// # Errors
    ///
    /// Not reachable.
    pub fn height(&self) -> Result<Height> {
        Ok(Height::above_mean_sea_level(millimetres(self.alt)?))
    }

    /// Horizontal accuracy, if sent (extension).
    #[must_use]
    pub fn horizontal_accuracy(&self) -> Option<Distance> {
        (self.h_acc != 0)
            .then(|| Distance::from_metres(f64::from(self.h_acc) * 1e-3).ok())
            .flatten()
    }
}

impl Attitude {
    /// Roll, right side down positive.
    ///
    /// # Errors
    ///
    /// [`KernelError::NotFinite`] for a non-finite angle.
    pub fn roll_angle(&self) -> Result<Angle> {
        Angle::from_radians(f64::from(self.roll))
    }

    /// Pitch, nose up positive.
    ///
    /// # Errors
    ///
    /// As [`Attitude::roll_angle`].
    pub fn pitch_angle(&self) -> Result<Angle> {
        Angle::from_radians(f64::from(self.pitch))
    }

    /// Yaw from true north, as the autopilot's NED frame has it.
    ///
    /// # Errors
    ///
    /// As [`Attitude::roll_angle`].
    pub fn heading(&self) -> Result<TrueCourse> {
        TrueCourse::wrap(kinavis_kernel::math::to_degrees(f64::from(self.yaw)))
    }

    /// Yaw rate.
    ///
    /// # Errors
    ///
    /// As [`Attitude::roll_angle`].
    pub fn yaw_rate(&self) -> Result<RateOfTurn> {
        RateOfTurn::from_radians_per_second(f64::from(self.yawspeed))
    }
}

impl LocalPositionNed {
    /// Position from the autopilot's EKF origin, NED. That origin is the
    /// autopilot's, not the application's `LocalFrame`.
    ///
    /// # Errors
    ///
    /// [`KernelError::NotFinite`] for a non-finite coordinate.
    pub fn displacement(&self) -> Result<Vector3<Ned, Distance>> {
        Ok(Vector3::new(
            Distance::from_metres(f64::from(self.x))?,
            Distance::from_metres(f64::from(self.y))?,
            Distance::from_metres(f64::from(self.z))?,
        ))
    }

    /// Velocity, NED.
    ///
    /// # Errors
    ///
    /// As [`LocalPositionNed::displacement`].
    pub fn velocity(&self) -> Result<Vector3<Ned, Speed>> {
        Ok(Vector3::new(
            Speed::from_metres_per_second(f64::from(self.vx))?,
            Speed::from_metres_per_second(f64::from(self.vy))?,
            Speed::from_metres_per_second(f64::from(self.vz))?,
        ))
    }
}

/// Height above mean sea level of a point, refusing any other datum.
fn amsl(point: GeodeticPoint) -> Result<Distance> {
    let height = point.height();
    if height.datum() != VerticalDatum::MeanSeaLevel {
        return Err(KernelError::VerticalDatumMismatch {
            required: VerticalDatum::MeanSeaLevel,
            found: height.datum(),
        });
    }
    Ok(height.value())
}

impl SetPositionTargetGlobalInt {
    /// Target position, the rest ignored (`type_mask` 0x0FF8), in
    /// `MAV_FRAME_GLOBAL_INT`.
    ///
    /// # Errors
    ///
    /// [`KernelError::VerticalDatumMismatch`] for a height not above mean
    /// sea level: convert it with a geoid model first.
    pub fn position(system: u8, component: u8, target: GeodeticPoint) -> Result<Self> {
        let alt = amsl(target)?;
        Ok(Self {
            target_system: system,
            target_component: component,
            coordinate_frame: FRAME_GLOBAL_INT,
            type_mask: 0x0FF8,
            lat_int: degrees_e7(target.position().latitude().degrees()),
            lon_int: degrees_e7(target.position().longitude().degrees()),
            alt: f32::from_bits(alt_bits(alt.metres())),
            ..Self::default()
        })
    }
}

/// Bits of an `f32` altitude in metres; rounding to the nearest `f32` is the
/// message's own resolution.
#[allow(clippy::cast_possible_truncation)]
fn alt_bits(metres: f64) -> u32 {
    (metres as f32).to_bits()
}

impl MissionItemInt {
    /// Waypoint `sequence` at a point, in `MAV_FRAME_GLOBAL_INT`, continuing
    /// to the next.
    ///
    /// # Errors
    ///
    /// As [`SetPositionTargetGlobalInt::position`].
    pub fn waypoint(
        system: u8,
        component: u8,
        sequence: u16,
        point: GeodeticPoint,
    ) -> Result<Self> {
        let alt = amsl(point)?;
        Ok(Self {
            target_system: system,
            target_component: component,
            seq: sequence,
            frame: FRAME_GLOBAL_INT,
            command: CMD_NAV_WAYPOINT,
            autocontinue: 1,
            x: degrees_e7(point.position().latitude().degrees()),
            y: degrees_e7(point.position().longitude().degrees()),
            z: f32::from_bits(alt_bits(alt.metres())),
            ..Self::default()
        })
    }
}

impl GpsRtcmData {
    /// The messages carrying an RTCM correction message of up to
    /// [`RTCM_MAX_BYTES`]: one if it fits in [`RTCM_FRAGMENT_BYTES`], else
    /// up to four fragments; `sequence` (five bits) tells corrections apart.
    ///
    /// # Errors
    ///
    /// [`KernelError::CapacityExceeded`] for more than [`RTCM_MAX_BYTES`].
    pub fn fragments(rtcm: &[u8], sequence: u8, mut send: impl FnMut(Self)) -> Result<()> {
        if rtcm.len() > RTCM_MAX_BYTES {
            return Err(KernelError::CapacityExceeded {
                context: "an RTCM message in GPS_RTCM_DATA",
                needed: rtcm.len(),
                capacity: RTCM_MAX_BYTES,
            });
        }
        let fragmented = rtcm.len() > RTCM_FRAGMENT_BYTES;
        let sequence = (sequence & 0x1F) << 3;
        for (index, chunk) in rtcm.chunks(RTCM_FRAGMENT_BYTES).enumerate() {
            let mut data = [0u8; RTCM_FRAGMENT_BYTES];
            for (slot, byte) in data.iter_mut().zip(chunk) {
                *slot = *byte;
            }
            let fragment = u8::try_from(index).unwrap_or(0) & 0x03;
            send(Self {
                flags: u8::from(fragmented) | fragment << 1 | sequence,
                len: u8::try_from(chunk.len()).unwrap_or(u8::MAX),
                data,
            });
        }
        if rtcm.is_empty() {
            send(Self {
                flags: sequence,
                ..Self::default()
            });
        }
        Ok(())
    }
}
