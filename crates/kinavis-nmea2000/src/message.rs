//! PGN decoding into kernel types.

use crate::ais::AisPositionReport;
use crate::error::Nmea2000Error;
use crate::frame::Payload;
use crate::gnss::GnssPosition;
use crate::id::Pgn;
use crate::navigation::{CourseAndSpeed, PositionRapidUpdate, VesselHeading, WaterDepth};

/// Decoded parameter group.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Message {
    /// PGN 129025.
    PositionRapidUpdate(PositionRapidUpdate),
    /// PGN 129026.
    CourseAndSpeed(CourseAndSpeed),
    /// PGN 129029.
    GnssPosition(GnssPosition),
    /// PGN 127250.
    VesselHeading(VesselHeading),
    /// PGN 128267.
    WaterDepth(WaterDepth),
    /// PGN 129038 and 129039.
    AisPositionReport(AisPositionReport),
    /// Payload of an unsupported PGN, for counting or logging.
    Unsupported {
        /// PGN.
        pgn: Pgn,
    },
}

impl Message {
    /// Decodes a group from its payload.
    ///
    /// # Errors
    ///
    /// [`Nmea2000Error::TooShort`] if the payload ends before a field;
    /// [`Nmea2000Error::Value`] if a field value is outside the domain.
    pub fn decode(payload: &Payload) -> Result<Self, Nmea2000Error> {
        let pgn = payload.pgn();
        Ok(match pgn {
            Pgn::POSITION_RAPID_UPDATE => {
                Self::PositionRapidUpdate(PositionRapidUpdate::decode(payload)?)
            }
            Pgn::COG_SOG_RAPID_UPDATE => Self::CourseAndSpeed(CourseAndSpeed::decode(payload)?),
            Pgn::GNSS_POSITION_DATA => Self::GnssPosition(GnssPosition::decode(payload)?),
            Pgn::VESSEL_HEADING => Self::VesselHeading(VesselHeading::decode(payload)?),
            Pgn::WATER_DEPTH => Self::WaterDepth(WaterDepth::decode(payload)?),
            Pgn::AIS_CLASS_A_POSITION_REPORT => {
                Self::AisPositionReport(AisPositionReport::class_a(payload)?)
            }
            Pgn::AIS_CLASS_B_POSITION_REPORT => {
                Self::AisPositionReport(AisPositionReport::class_b(payload)?)
            }
            _ => Self::Unsupported { pgn },
        })
    }
}
