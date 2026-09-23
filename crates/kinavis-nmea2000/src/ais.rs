//! AIS position reports as put on the bus by an AIS receiver: PGN 129038 (class
//! A), 129039 (class B).
//!
//! The receiver has already decoded the radio message; the fields arrive
//! byte-aligned in standard units. Field names match `kinavis-ais`.

use kinavis_kernel::angle::TrueCourse;
use kinavis_kernel::event::TargetId;
use kinavis_kernel::position::Position;
use kinavis_kernel::snapshot::GroundTrack;
use kinavis_kernel::units::{RateOfTurn, Speed};

use crate::error::Nmea2000Error;
use crate::fields::{direction, position, resolution, second, speed, value_of, Fields};
use crate::frame::Payload;

/// Station class.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum StationClass {
    /// Class A: reports navigational status and rate of turn.
    A,
    /// Class B: reports neither.
    B,
}

/// AIS position report, class A or B.
///
/// Every field that can be "not available" is an `Option`, `None` when
/// unavailable — never a zero that reads as a valid value.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct AisPositionReport {
    /// Station class.
    pub class: StationClass,
    /// AIS message type: 1, 2 or 3 for class A, 18 for class B.
    pub message_id: u8,
    /// Station MMSI.
    pub mmsi: TargetId,
    /// Position, 1e-7° resolution.
    pub position: Option<Position>,
    /// Position accuracy flag: DGNSS, better than 10 m.
    pub accurate: bool,
    /// RAIM flag.
    pub raim: bool,
    /// UTC second of the report, `0..=59`.
    pub second: Option<u8>,
    /// Course over ground, 1e-4 rad resolution.
    pub course: Option<TrueCourse>,
    /// Speed over ground, 0.01 m/s resolution.
    pub speed: Option<Speed>,
    /// True heading, 1e-4 rad resolution.
    pub heading: Option<TrueCourse>,
    /// Rate of turn, 3.125e-5 rad/s resolution; class A only.
    pub turn: Option<RateOfTurn>,
    /// Navigational status code `0..=14` (ITU-R M.1371; named in
    /// `kinavis-ais`); class A only.
    pub status: Option<u8>,
}

impl AisPositionReport {
    /// Payload length: class A, class B.
    const CLASS_A_NEEDED: usize = 28;
    const CLASS_B_NEEDED: usize = 26;

    /// PGN 129038.
    pub(crate) fn class_a(payload: &Payload) -> Result<Self, Nmea2000Error> {
        let field = Fields::of(payload, Self::CLASS_A_NEEDED)?;
        let mut report = Self::common(&field, StationClass::A)?;
        report.turn = field
            .signed(23, 2)
            .map(|value| {
                // At most 16 bits.
                #[allow(clippy::cast_precision_loss)]
                RateOfTurn::from_radians_per_second(
                    value as f64 * resolution::RATE_OF_TURN_RAD_PER_S,
                )
                .map_err(value_of("rate of turn"))
            })
            .transpose()?;
        // Four bits; 15 is "not defined".
        #[allow(clippy::cast_possible_truncation)]
        let status = field.bits(25 * 8, 4).map(|code| code as u8);
        report.status = status;
        Ok(report)
    }

    /// PGN 129039.
    pub(crate) fn class_b(payload: &Payload) -> Result<Self, Nmea2000Error> {
        let field = Fields::of(payload, Self::CLASS_B_NEEDED)?;
        Self::common(&field, StationClass::B)
    }

    /// Fields common to both PGNs, at identical offsets.
    fn common(field: &Fields<'_>, class: StationClass) -> Result<Self, Nmea2000Error> {
        Ok(Self {
            class,
            // Six bits.
            #[allow(clippy::cast_possible_truncation)]
            message_id: field.raw_bits(0, 6) as u8,
            // 32 bits.
            #[allow(clippy::cast_possible_truncation)]
            mmsi: TargetId::new(field.unsigned(1, 4).unwrap_or(0) as u32),
            position: position(
                field.signed(9, 4),
                field.signed(5, 4),
                resolution::POSITION_DEG,
            )?,
            accurate: field.flag(13 * 8),
            raim: field.flag(13 * 8 + 1),
            second: second(field.raw_bits(13 * 8 + 2, 6)),
            course: direction(field.unsigned(14, 2)),
            speed: speed(field.unsigned(16, 2))?,
            heading: direction(field.unsigned(21, 2)),
            turn: None,
            status: None,
        })
    }

    /// Course and speed over ground, if both are reported.
    #[must_use]
    pub fn ground_track(&self) -> Option<GroundTrack> {
        Some(GroundTrack {
            course_over_ground: self.course?,
            speed_over_ground: self.speed?,
        })
    }
}
