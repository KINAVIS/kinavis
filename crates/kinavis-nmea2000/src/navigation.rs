//! Single-frame navigation PGNs: position, course and speed, heading, depth.

use core::fmt;

use kinavis_kernel::angle::{Deviation, MagneticCourse, TrueCourse, Variation};
use kinavis_kernel::math;
use kinavis_kernel::position::Position;
use kinavis_kernel::units::{Distance, Speed};

use crate::error::Nmea2000Error;
use crate::fields::{direction, distance, position, resolution, speed, value_of, Fields};
use crate::frame::Payload;

/// Direction with its reference: true or magnetic north.
///
/// The reference field also has "error" and "not available"; both read as
/// `None` at the field level.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Referenced {
    /// From true north.
    True(TrueCourse),
    /// From magnetic north.
    Magnetic(MagneticCourse),
}

impl Referenced {
    /// Direction in degrees, regardless of reference.
    #[must_use]
    pub fn degrees(&self) -> f64 {
        match self {
            Self::True(course) => course.degrees(),
            Self::Magnetic(course) => course.degrees(),
        }
    }

    /// Direction, if true-referenced.
    #[must_use]
    pub const fn as_true(&self) -> Option<TrueCourse> {
        match self {
            Self::True(course) => Some(*course),
            Self::Magnetic(_) => None,
        }
    }

    /// Direction, if magnetic-referenced.
    #[must_use]
    pub const fn as_magnetic(&self) -> Option<MagneticCourse> {
        match self {
            Self::Magnetic(course) => Some(*course),
            Self::True(_) => None,
        }
    }

    /// Reference field (`0` true, `1` magnetic) applied to an angle in 1e-4
    /// rad.
    pub(crate) fn from_fields(reference: u32, angle: Option<u64>) -> Option<Self> {
        match reference {
            0 => direction(angle).map(Self::True),
            1 => direction(angle).map(Self::Magnetic),
            _ => None,
        }
    }
}

impl fmt::Display for Referenced {
    /// Formats as `271.5°T` or `268.0°M`.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let precision = f.precision().unwrap_or(1);
        match self {
            Self::True(course) => write!(f, "{:.*}°T", precision, course.degrees()),
            Self::Magnetic(course) => write!(f, "{:.*}°M", precision, course.degrees()),
        }
    }
}

/// Position, rapid update: PGN 129025.
///
/// Latitude and longitude at 1e-7°, several times per second; time and quality
/// come once per second in GNSS position data.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PositionRapidUpdate {
    /// Position, WGS 84.
    pub position: Option<Position>,
}

impl PositionRapidUpdate {
    const NEEDED: usize = 8;

    pub(crate) fn decode(payload: &Payload) -> Result<Self, Nmea2000Error> {
        let field = Fields::of(payload, Self::NEEDED)?;
        Ok(Self {
            position: position(
                field.signed(0, 4),
                field.signed(4, 4),
                resolution::POSITION_DEG,
            )?,
        })
    }
}

/// Course and speed over ground, rapid update: PGN 129026.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct CourseAndSpeed {
    /// Sequence identifier linking groups of the same fix, if sent.
    pub sid: Option<u8>,
    /// Course over ground, 1e-4 rad resolution, with reference.
    pub course: Option<Referenced>,
    /// Speed over ground, 0.01 m/s resolution.
    pub speed: Option<Speed>,
}

impl CourseAndSpeed {
    const NEEDED: usize = 8;

    pub(crate) fn decode(payload: &Payload) -> Result<Self, Nmea2000Error> {
        let field = Fields::of(payload, Self::NEEDED)?;
        Ok(Self {
            sid: field.u8(0),
            course: Referenced::from_fields(field.raw_bits(8, 2), field.unsigned(2, 2)),
            speed: speed(field.unsigned(4, 2))?,
        })
    }
}

/// Vessel heading: PGN 127250.
///
/// Sensor heading plus the deviation and variation applied or known by the
/// sender, for converting magnetic to true and back.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct VesselHeading {
    /// Sequence identifier, if sent.
    pub sid: Option<u8>,
    /// Heading, 1e-4 rad resolution, with reference.
    pub heading: Option<Referenced>,
    /// Deviation, 1e-4 rad resolution, if known.
    pub deviation: Option<Deviation>,
    /// Variation, 1e-4 rad resolution, if known.
    pub variation: Option<Variation>,
}

impl VesselHeading {
    const NEEDED: usize = 8;

    pub(crate) fn decode(payload: &Payload) -> Result<Self, Nmea2000Error> {
        let field = Fields::of(payload, Self::NEEDED)?;
        Ok(Self {
            sid: field.u8(0),
            heading: Referenced::from_fields(field.raw_bits(56, 2), field.unsigned(1, 2)),
            deviation: field
                .signed(3, 2)
                .map(|value| {
                    Deviation::new(small_angle_degrees(value)).map_err(value_of("deviation"))
                })
                .transpose()?,
            variation: field
                .signed(5, 2)
                .map(|value| {
                    Variation::new(small_angle_degrees(value)).map_err(value_of("variation"))
                })
                .transpose()?,
        })
    }
}

/// Signed angle in 1e-4 rad, as degrees.
fn small_angle_degrees(field: i64) -> f64 {
    // At most 16 bits.
    #[allow(clippy::cast_precision_loss)]
    math::to_degrees(field as f64 * resolution::ANGLE_RAD)
}

/// Water depth: PGN 128267.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct WaterDepth {
    /// Sequence identifier, if sent.
    pub sid: Option<u8>,
    /// Depth below the transducer, 0.01 m resolution.
    pub depth: Option<Distance>,
    /// Transducer offset, 0.001 m resolution: positive to the waterline,
    /// negative to the keel; depth + offset is depth below the surface or the
    /// keel.
    pub offset: Option<Distance>,
    /// Sounder range setting, 10 m resolution.
    pub range: Option<Distance>,
}

impl WaterDepth {
    const NEEDED: usize = 8;

    pub(crate) fn decode(payload: &Payload) -> Result<Self, Nmea2000Error> {
        let field = Fields::of(payload, Self::NEEDED)?;
        Ok(Self {
            sid: field.u8(0),
            depth: distance(
                field.unsigned(1, 4).and_then(|v| i64::try_from(v).ok()),
                resolution::CENTIMETRE_M,
                "depth",
            )?,
            offset: distance(
                field.signed(5, 2),
                resolution::MILLIMETRE_M,
                "transducer offset",
            )?,
            range: distance(
                field.u8(7).map(i64::from),
                resolution::DEPTH_RANGE_M,
                "range",
            )?,
        })
    }

    /// Depth below the surface or the keel (depth + offset), if both are
    /// present.
    #[must_use]
    pub fn depth_from_reference(&self) -> Option<Distance> {
        Some(self.depth? + self.offset?)
    }
}
