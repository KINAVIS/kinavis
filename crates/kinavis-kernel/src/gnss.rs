//! Satellite fix, independent of the sentence it arrived in.
//!
//! Receivers report fixes in many formats (NMEA `RMC`/`GGA`, binary protocols,
//! proprietary), all carrying the same facts: position, time, method, quality.
//! [`GnssFix`] holds those facts; parsers translate into it and use cases read
//! only it (anti-corruption layer).
//!
//! No vertical channel: a height without datum, vertical velocity and source
//! would be incomplete.
//!
//! ```rust
//! use kinavis_kernel::gnss::{Dop, FixType, GnssFix};
//! use kinavis_kernel::time::{Civil, Instant, Utc};
//! use kinavis_kernel::{Position, Speed, TrueCourse};
//!
//! let taken_at = Instant::<Utc>::from_civil(Civil::date(2026, 9, 11))?;
//! let fix = GnssFix::builder(taken_at, "50°45.3'N 001°20.0'W".parse::<Position>()?)
//!     .fix_type(FixType::Differential)
//!     .course_over_ground(TrueCourse::new(272.5)?)
//!     .speed_over_ground(Speed::from_knots(11.3)?)
//!     .satellites(9)
//!     .hdop(Dop::new(0.9)?)
//!     .build();
//!
//! assert!(fix.quality().fix_type().is_position_fix());
//! // HDOP times the nominal one-sigma range error for a differential fix.
//! assert_eq!(format!("{:.2}", fix.horizontal_accuracy().unwrap().metres()), "0.90");
//! # Ok::<(), kinavis_kernel::KernelError>(())
//! ```

use crate::angle::TrueCourse;
use crate::error::{ensure_finite, KernelError, Result};
use crate::observation::{ObservationStatus, Observed, Quality};
use crate::position::Position;
use crate::time::{Instant, Utc};
use crate::units::{Distance, Speed};

/// Fix method.
///
/// Follows the NMEA mode indicators. Downstream, [`FixType::is_position_fix`]
/// distinguishes satellite-measured positions from carried-forward, manual or
/// simulated ones.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum FixType {
    /// No fix; the position is the last known one.
    None,
    /// Autonomous single-receiver fix.
    Autonomous,
    /// Differential (DGPS, SBAS).
    Differential,
    /// Precise Positioning Service (military).
    Precise,
    /// RTK, integer ambiguity fixed.
    RtkFixed,
    /// RTK float.
    RtkFloat,
    /// Receiver dead reckoning bridging a coverage gap.
    Estimated,
    /// Manual input.
    Manual,
    /// Simulator.
    Simulated,
}

impl FixType {
    /// Whether the position was measured from satellites.
    ///
    /// `Estimated`, `Manual` and `Simulated` may be displayed but are not
    /// fixes.
    #[must_use]
    pub const fn is_position_fix(self) -> bool {
        matches!(
            self,
            Self::Autonomous | Self::Differential | Self::Precise | Self::RtkFixed | Self::RtkFloat
        )
    }

    /// Nominal 1σ UERE for this fix type, m; `None` where undefined.
    ///
    /// Round figures from published performance standards, for converting DOP
    /// to distance. A receiver's own accuracy estimate takes precedence.
    const fn nominal_uere_metres(self) -> Option<f64> {
        match self {
            Self::Autonomous => Some(4.0),
            Self::Differential => Some(1.0),
            Self::Precise => Some(3.0),
            Self::RtkFixed => Some(0.02),
            Self::RtkFloat => Some(0.5),
            Self::None | Self::Estimated | Self::Manual | Self::Simulated => None,
        }
    }
}

/// Dilution of precision: geometry factor from range error to position error.
///
/// Dimensionless, positive, finite; ~1 is good geometry, above ~6 poor.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(try_from = "f64", into = "f64")
)]
pub struct Dop(f64);

impl Dop {
    /// DOP from a value.
    ///
    /// # Errors
    ///
    /// [`KernelError::NotFinite`] for `NaN` or infinity;
    /// [`KernelError::OutOfRange`] for zero or negative.
    pub fn new(value: f64) -> Result<Self> {
        ensure_finite("dilution of precision", value)?;
        if value <= 0.0 {
            return Err(KernelError::OutOfRange {
                parameter: "dilution of precision",
                value,
                min: f64::MIN_POSITIVE,
                max: f64::MAX,
            });
        }
        Ok(Self(value))
    }

    /// Factor.
    #[must_use]
    pub const fn value(self) -> f64 {
        self.0
    }
}

impl TryFrom<f64> for Dop {
    type Error = KernelError;

    fn try_from(value: f64) -> Result<Self> {
        Self::new(value)
    }
}

impl From<Dop> for f64 {
    fn from(dop: Dop) -> Self {
        dop.0
    }
}

/// Receiver-reported fix quality.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct GnssQuality {
    fix_type: FixType,
    satellites: Option<u8>,
    hdop: Option<Dop>,
    vdop: Option<Dop>,
    pdop: Option<Dop>,
}

impl GnssQuality {
    /// Fix method.
    #[must_use]
    pub const fn fix_type(&self) -> FixType {
        self.fix_type
    }

    /// Satellites used, if reported.
    #[must_use]
    pub const fn satellites(&self) -> Option<u8> {
        self.satellites
    }

    /// HDOP, if reported.
    #[must_use]
    pub const fn hdop(&self) -> Option<Dop> {
        self.hdop
    }

    /// VDOP, if reported.
    #[must_use]
    pub const fn vdop(&self) -> Option<Dop> {
        self.vdop
    }

    /// PDOP, if reported.
    #[must_use]
    pub const fn pdop(&self) -> Option<Dop> {
        self.pdop
    }
}

/// Satellite fix: position, time, method and other receiver data.
///
/// Built via [`GnssFix::builder`]: most fields are optional and filled
/// differently by different sentences.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct GnssFix {
    taken_at: Instant<Utc>,
    position: Position,
    course_over_ground: Option<TrueCourse>,
    speed_over_ground: Option<Speed>,
    quality: GnssQuality,
}

impl GnssFix {
    /// Starts a fix from time and position, the two mandatory fields.
    ///
    /// With nothing else set it is an autonomous fix.
    pub const fn builder(taken_at: Instant<Utc>, position: Position) -> GnssFixBuilder {
        GnssFixBuilder {
            fix: Self {
                taken_at,
                position,
                course_over_ground: None,
                speed_over_ground: None,
                quality: GnssQuality {
                    fix_type: FixType::Autonomous,
                    satellites: None,
                    hdop: None,
                    vdop: None,
                    pdop: None,
                },
            },
        }
    }

    /// Fix time, receiver clock, UTC.
    #[must_use]
    pub const fn taken_at(&self) -> Instant<Utc> {
        self.taken_at
    }

    /// Position.
    #[must_use]
    pub const fn position(&self) -> Position {
        self.position
    }

    /// Course over ground, if reported.
    ///
    /// Receivers omit it when stationary, where it is noise.
    #[must_use]
    pub const fn course_over_ground(&self) -> Option<TrueCourse> {
        self.course_over_ground
    }

    /// Speed over ground, if reported.
    #[must_use]
    pub const fn speed_over_ground(&self) -> Option<Speed> {
        self.speed_over_ground
    }

    /// Quality.
    #[must_use]
    pub const fn quality(&self) -> &GnssQuality {
        &self.quality
    }

    /// Estimated 1σ horizontal error: HDOP × nominal UERE for the fix type.
    ///
    /// `None` without HDOP or for non-satellite positions. A nominal estimate,
    /// not a measurement; see [`GnssFix::horizontal_accuracy_with`] to supply a
    /// UERE.
    #[must_use]
    pub fn horizontal_accuracy(&self) -> Option<Distance> {
        let uere = self.quality.fix_type.nominal_uere_metres()?;
        // The UERE table holds finite constants; cannot fail.
        self.horizontal_accuracy_with(Distance::from_metres(uere).ok()?)
    }

    /// 1σ horizontal error for a given 1σ range error: `HDOP × UERE`.
    ///
    /// `None` without HDOP.
    #[must_use]
    pub fn horizontal_accuracy_with(&self, uere: Distance) -> Option<Distance> {
        self.quality.hdop.map(|hdop| uere * hdop.value())
    }

    /// Position as an [`Observed`] value, for source-agnostic use cases.
    ///
    /// Status by fix type: satellite fix `Valid`; receiver estimate, manual or
    /// simulated `Suspect`; no fix `Invalid`. Uncertainty is
    /// [`GnssFix::horizontal_accuracy`] where available.
    #[must_use]
    pub fn observed_position(&self) -> Observed<Position, Distance> {
        let status = match self.quality.fix_type {
            FixType::None => ObservationStatus::Invalid,
            FixType::Estimated | FixType::Manual | FixType::Simulated => ObservationStatus::Suspect,
            _ => ObservationStatus::Valid,
        };
        let mut quality = Quality::new(status);
        if let Some(sigma) = self.horizontal_accuracy() {
            quality = quality.with_sigma(sigma);
        }
        Observed::new(self.position, self.taken_at, quality)
    }
}

/// Builder for the optional parts of a [`GnssFix`].
///
/// Setters take already-validated types, so [`GnssFixBuilder::build`] cannot
/// fail. The only builder in the kernel: many optional fields, filled per
/// sentence.
#[derive(Debug, Clone, Copy)]
#[must_use = "a builder does nothing until `build` is called"]
pub struct GnssFixBuilder {
    fix: GnssFix,
}

impl GnssFixBuilder {
    /// Fix method; default `Autonomous`.
    pub const fn fix_type(mut self, fix_type: FixType) -> Self {
        self.fix.quality.fix_type = fix_type;
        self
    }

    /// Course over ground.
    pub const fn course_over_ground(mut self, course: TrueCourse) -> Self {
        self.fix.course_over_ground = Some(course);
        self
    }

    /// Speed over ground.
    pub const fn speed_over_ground(mut self, speed: Speed) -> Self {
        self.fix.speed_over_ground = Some(speed);
        self
    }

    /// Satellites used.
    pub const fn satellites(mut self, count: u8) -> Self {
        self.fix.quality.satellites = Some(count);
        self
    }

    /// HDOP.
    pub const fn hdop(mut self, hdop: Dop) -> Self {
        self.fix.quality.hdop = Some(hdop);
        self
    }

    /// VDOP.
    pub const fn vdop(mut self, vdop: Dop) -> Self {
        self.fix.quality.vdop = Some(vdop);
        self
    }

    /// PDOP.
    pub const fn pdop(mut self, pdop: Dop) -> Self {
        self.fix.quality.pdop = Some(pdop);
        self
    }

    /// Builds the fix.
    #[must_use]
    pub const fn build(self) -> GnssFix {
        self.fix
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;

    fn somewhere() -> Position {
        "50°45.3'N 001°20.0'W".parse().unwrap()
    }

    fn at(seconds: i64) -> Instant<Utc> {
        Instant::from_unix_seconds(seconds)
    }

    #[test]
    fn a_bare_fix_is_autonomous_with_nothing_else_known() {
        let fix = GnssFix::builder(at(100), somewhere()).build();
        assert_eq!(fix.taken_at(), at(100));
        assert_eq!(fix.position(), somewhere());
        assert_eq!(fix.quality().fix_type(), FixType::Autonomous);
        assert_eq!(fix.course_over_ground(), None);
        assert_eq!(fix.speed_over_ground(), None);
        assert_eq!(fix.quality().satellites(), None);
        assert_eq!(fix.horizontal_accuracy(), None);
    }

    #[test]
    fn accuracy_is_hdop_times_the_range_error() {
        let fix = GnssFix::builder(at(0), somewhere())
            .hdop(Dop::new(2.0).unwrap())
            .build();
        // Autonomous: 4 m nominal.
        assert!((fix.horizontal_accuracy().unwrap().metres() - 8.0).abs() < 1e-9);
        let own = Distance::from_metres(1.5).unwrap();
        assert!((fix.horizontal_accuracy_with(own).unwrap().metres() - 3.0).abs() < 1e-9);
        // Estimated position: no meaningful range error.
        let estimated = GnssFix::builder(at(0), somewhere())
            .fix_type(FixType::Estimated)
            .hdop(Dop::new(2.0).unwrap())
            .build();
        assert_eq!(estimated.horizontal_accuracy(), None);
        assert!(estimated.horizontal_accuracy_with(own).is_some());
    }

    #[test]
    fn the_observed_position_follows_the_fix_type() {
        let cases = [
            (FixType::RtkFixed, ObservationStatus::Valid),
            (FixType::Autonomous, ObservationStatus::Valid),
            (FixType::Estimated, ObservationStatus::Suspect),
            (FixType::Simulated, ObservationStatus::Suspect),
            (FixType::None, ObservationStatus::Invalid),
        ];
        for (fix_type, status) in cases {
            let fix = GnssFix::builder(at(7), somewhere())
                .fix_type(fix_type)
                .hdop(Dop::new(1.0).unwrap())
                .build();
            let observed = fix.observed_position();
            assert_eq!(observed.quality().status(), status, "{fix_type:?}");
            assert_eq!(observed.taken_at(), at(7));
            assert_eq!(*observed.value(), somewhere());
            assert_eq!(
                observed.quality().sigma().is_some(),
                fix_type.is_position_fix(),
                "{fix_type:?}"
            );
        }
    }

    #[test]
    fn a_dilution_of_precision_is_positive_and_finite() {
        assert!(Dop::new(0.0).is_err());
        assert!(Dop::new(-1.0).is_err());
        assert!(Dop::new(f64::NAN).is_err());
        assert!(Dop::new(f64::INFINITY).is_err());
        assert_eq!(Dop::new(1.5).unwrap().value(), 1.5);
    }

    #[cfg(feature = "serde")]
    #[test]
    fn serde_round_trips_and_validates_the_dop() {
        let fix = GnssFix::builder(at(0), somewhere())
            .fix_type(FixType::Differential)
            .speed_over_ground(Speed::from_knots(3.0).unwrap())
            .satellites(12)
            .pdop(Dop::new(1.7).unwrap())
            .build();
        let json = serde_json::to_string(&fix).unwrap();
        assert_eq!(serde_json::from_str::<GnssFix>(&json).unwrap(), fix);
        assert!(serde_json::from_str::<Dop>("-1.0").is_err());
    }
}
