//! Environment ports and the resolved sample.
//!
//! Magnetic model, current atlas, wind forecast and deviation table all answer
//! a question for a place and time. The choice of model (WMM or IGRF, tidal
//! atlas or GRIB, swung table or Smith coefficients) is not the kernel's; the
//! kernel defines the ports ([`MagneticModel`], [`CurrentModel`],
//! [`WindModel`], [`TideModel`], [`LeewayModel`], [`CompassModel`]) and the
//! answer types ([`MagneticField`], [`Current`], [`Wind`], height of tide as a
//! [`Distance`] above chart datum).
//!
//! An [`EnvironmentSample`] holds the answers resolved for one point and
//! instant, for calculations that must not know the source. No port is
//! implemented here; implementations live in `kinavis` or in satellite crates
//! with the model data.
//!
//! The compass model sits here because it is used together with the magnetic
//! one: variation from the field, deviation from the ship's magnetism, together
//! converting compass to true.

use core::fmt;

use crate::angle::{CompassCourse, Deviation, TrueCourse, Variation};
use crate::error::{ensure_range, KernelError, Result};
use crate::geodesy::GeodeticPoint;
use crate::math;
use crate::position::Position;
use crate::time::{Instant, Utc};
use crate::units::{Angle, Distance, Speed};

/// Maximum magnitude of a field component, nT.
///
/// Total intensity is below 70 000 nT everywhere at the surface and decreases
/// with height; larger values indicate a unit error (gauss, µT) and are
/// rejected.
pub const MAX_FIELD_NANOTESLA: f64 = 100_000.0;

// ---------------------------------------------------------------------------
// The value types
// ---------------------------------------------------------------------------

/// Earth's magnetic field at a point: north, east, down components in nT.
///
/// Derived quantities — [`declination`], [`inclination`], horizontal and total
/// intensity — are computed from the components, not stored.
///
/// [`declination`]: Self::declination
/// [`inclination`]: Self::inclination
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(try_from = "MagneticFieldComponents", into = "MagneticFieldComponents")
)]
pub struct MagneticField {
    north: f64,
    east: f64,
    down: f64,
}

impl MagneticField {
    /// Field from north, east, down components in nT.
    ///
    /// # Errors
    ///
    /// [`KernelError::NotFinite`] for a non-finite component;
    /// [`KernelError::OutOfRange`] beyond ±[`MAX_FIELD_NANOTESLA`].
    pub fn from_ned_nanotesla(north: f64, east: f64, down: f64) -> Result<Self> {
        ensure_range(
            "field north component",
            north,
            -MAX_FIELD_NANOTESLA,
            MAX_FIELD_NANOTESLA,
        )?;
        ensure_range(
            "field east component",
            east,
            -MAX_FIELD_NANOTESLA,
            MAX_FIELD_NANOTESLA,
        )?;
        ensure_range(
            "field down component",
            down,
            -MAX_FIELD_NANOTESLA,
            MAX_FIELD_NANOTESLA,
        )?;
        Ok(Self { north, east, down })
    }

    /// North component, nT.
    #[must_use]
    pub const fn north_nanotesla(&self) -> f64 {
        self.north
    }

    /// East component, nT.
    #[must_use]
    pub const fn east_nanotesla(&self) -> f64 {
        self.east
    }

    /// Down component, nT; positive in the northern magnetic hemisphere.
    #[must_use]
    pub const fn down_nanotesla(&self) -> f64 {
        self.down
    }

    /// Horizontal intensity `H`, nT: the component that aligns a compass.
    ///
    /// Small near the magnetic poles, where a compass becomes sluggish and then
    /// unusable.
    #[must_use]
    pub fn horizontal_intensity_nanotesla(&self) -> f64 {
        math::hypot(self.north, self.east)
    }

    /// Total intensity `F`, nT.
    #[must_use]
    pub fn total_intensity_nanotesla(&self) -> f64 {
        math::hypot(self.horizontal_intensity_nanotesla(), self.down)
    }

    /// Declination `D` (magnetic variation), east positive.
    ///
    /// Undefined where `H` is zero and reported as zero; check
    /// [`horizontal_intensity_nanotesla`](Self::horizontal_intensity_nanotesla)
    /// before relying on a compass there.
    #[must_use]
    pub fn declination(&self) -> Variation {
        let degrees = math::to_degrees(math::atan2(self.east, self.north));
        // `atan2` is within ±180°, the variation range.
        Variation::new(degrees).unwrap_or(Variation::ZERO)
    }

    /// Inclination `I` (dip): angle below the horizontal, positive down.
    #[must_use]
    pub fn inclination(&self) -> Angle {
        let radians = math::atan2(self.down, self.horizontal_intensity_nanotesla());
        // `atan2` of finite arguments is finite.
        Angle::from_radians(radians).unwrap_or(Angle::ZERO)
    }
}

impl fmt::Display for MagneticField {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "D {}, I {:.1}°, H {:.0} nT, F {:.0} nT",
            self.declination(),
            self.inclination().degrees(),
            self.horizontal_intensity_nanotesla(),
            self.total_intensity_nanotesla()
        )
    }
}

/// Serialised form of [`MagneticField`]: named components, validated on
/// deserialisation. Units are part of the key names.
#[cfg(feature = "serde")]
#[derive(serde::Serialize, serde::Deserialize)]
// The unit suffix is part of the wire format.
#[allow(clippy::struct_field_names)]
struct MagneticFieldComponents {
    north_nanotesla: f64,
    east_nanotesla: f64,
    down_nanotesla: f64,
}

#[cfg(feature = "serde")]
impl TryFrom<MagneticFieldComponents> for MagneticField {
    type Error = KernelError;

    fn try_from(components: MagneticFieldComponents) -> Result<Self> {
        Self::from_ned_nanotesla(
            components.north_nanotesla,
            components.east_nanotesla,
            components.down_nanotesla,
        )
    }
}

#[cfg(feature = "serde")]
impl From<MagneticField> for MagneticFieldComponents {
    fn from(field: MagneticField) -> Self {
        Self {
            north_nanotesla: field.north,
            east_nanotesla: field.east,
            down_nanotesla: field.down,
        }
    }
}

/// Current: set and drift.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Current {
    /// Direction the current flows towards.
    ///
    /// Undefined when `drift` is zero; reported as `000°`.
    pub set: TrueCourse,
    /// Current speed.
    pub drift: Speed,
}

impl Current {
    /// Slack water.
    pub const SLACK: Self = Self {
        set: TrueCourse::NORTH,
        drift: Speed::ZERO,
    };
}

/// True wind: direction from and speed.
///
/// Named by the direction it blows *from*, speed never negative. Apparent wind
/// is a different quantity.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(try_from = "WindComponents", into = "WindComponents")
)]
pub struct Wind {
    from: TrueCourse,
    speed: Speed,
}

impl Wind {
    /// Calm.
    pub const CALM: Self = Self {
        from: TrueCourse::NORTH,
        speed: Speed::ZERO,
    };

    /// Wind from `from` at `speed`.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] for a negative speed.
    pub fn new(from: TrueCourse, speed: Speed) -> Result<Self> {
        if speed.is_negative() {
            return Err(KernelError::OutOfRange {
                parameter: "wind speed",
                value: speed.knots(),
                min: 0.0,
                max: f64::INFINITY,
            });
        }
        Ok(Self { from, speed })
    }

    /// Direction the wind blows from.
    ///
    /// Undefined at zero speed; reported as `000°`.
    #[must_use]
    pub const fn from(&self) -> TrueCourse {
        self.from
    }

    /// Direction the wind blows towards: reciprocal of [`from`](Self::from).
    #[must_use]
    pub fn towards(&self) -> TrueCourse {
        self.from.reciprocal()
    }

    /// Wind speed, non-negative.
    #[must_use]
    pub const fn speed(&self) -> Speed {
        self.speed
    }
}

impl fmt::Display for Wind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{} from {}", self.speed, self.from)
    }
}

/// Serialised form of [`Wind`], validated on deserialisation.
#[cfg(feature = "serde")]
#[derive(serde::Serialize, serde::Deserialize)]
struct WindComponents {
    from: TrueCourse,
    speed: Speed,
}

#[cfg(feature = "serde")]
impl TryFrom<WindComponents> for Wind {
    type Error = KernelError;

    fn try_from(components: WindComponents) -> Result<Self> {
        Self::new(components.from, components.speed)
    }
}

#[cfg(feature = "serde")]
impl From<Wind> for WindComponents {
    fn from(wind: Wind) -> Self {
        Self {
            from: wind.from,
            speed: wind.speed,
        }
    }
}

/// Motion through the water, as input to a leeway model.
///
/// Heading and speed through the water, not over the ground: leeway acts
/// relative to the water; current is added afterwards.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct VesselMotion {
    heading: TrueCourse,
    speed_through_water: Speed,
}

impl VesselMotion {
    /// Vessel on `heading` at `speed_through_water`.
    #[must_use]
    pub const fn new(heading: TrueCourse, speed_through_water: Speed) -> Self {
        Self {
            heading,
            speed_through_water,
        }
    }

    /// True heading.
    #[must_use]
    pub const fn heading(&self) -> TrueCourse {
        self.heading
    }

    /// Speed through the water; negative for sternway.
    #[must_use]
    pub const fn speed_through_water(&self) -> Speed {
        self.speed_through_water
    }
}

// ---------------------------------------------------------------------------
// The ports
// ---------------------------------------------------------------------------

/// Earth's magnetic field: WMM, IGRF, compass rose, constant.
///
/// Outside its validity interval an implementation returns
/// [`KernelError::OutsideValidity`] rather than extrapolating. The point
/// includes height because the field decreases with it.
pub trait MagneticModel {
    /// Field at `at`, time `when`.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutsideValidity`] outside the model's validity; any other
    /// error the model data raises.
    fn field_at(&self, at: GeodeticPoint, when: Instant<Utc>) -> Result<MagneticField>;
}

/// Ship's magnetism as compass deviation.
///
/// Swung table, Smith coefficients or calibrated fluxgate. Deviation is a
/// function of the compass course (what is read on the card), not the magnetic
/// course; the inverse is a solve performed by the algorithms.
pub trait CompassModel {
    /// Deviation on `course`.
    ///
    /// # Errors
    ///
    /// Any error the model raises: table too sparse, course not covered.
    fn deviation(&self, course: CompassCourse) -> Result<Deviation>;
}

/// Current: constant, tidal, GRIB, ocean model.
pub trait CurrentModel {
    /// Current at `at`, time `when`.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutsideValidity`] outside the model's coverage; any other
    /// error the model data raises.
    fn current_at(&self, at: Position, when: Instant<Utc>) -> Result<Current>;
}

/// True wind: forecast, observation, constant.
pub trait WindModel {
    /// Wind at `at`, time `when`.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutsideValidity`] outside the model's coverage; any other
    /// error the model data raises.
    fn wind_at(&self, at: Position, when: Instant<Utc>) -> Result<Wind>;
}

/// Tide: tables, harmonic prediction, gauge.
///
/// Returns the height of water above chart datum, so depth = charted depth +
/// height of tide.
pub trait TideModel {
    /// Height of tide at `at`, time `when`.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutsideValidity`] outside the model's coverage; any other
    /// error the model data raises.
    fn height_of_tide(&self, at: Position, when: Instant<Utc>) -> Result<Distance>;
}

/// Leeway model for a hull.
///
/// Leeway depends on windage, draught and speed, hence a port rather than a
/// formula. Returns the angle between heading and water track.
pub trait LeewayModel {
    /// Leeway for `motion` under `wind`, positive when set to starboard of the
    /// heading.
    ///
    /// # Errors
    ///
    /// Any error the model raises, e.g. speed outside its fitted range.
    fn leeway(&self, motion: VesselMotion, wind: Wind) -> Result<Angle>;
}

// ---------------------------------------------------------------------------
// The sample
// ---------------------------------------------------------------------------

/// Environment resolved for one point and instant.
///
/// Values, not sources: built from whichever models are available and passed to
/// calculations that need no port. Unresolved quantities are `None`, never
/// zero.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct EnvironmentSample {
    point: GeodeticPoint,
    when: Instant<Utc>,
    magnetic: Option<MagneticField>,
    current: Option<Current>,
    wind: Option<Wind>,
    /// Optional in the serialised form, for samples stored without it.
    #[cfg_attr(feature = "serde", serde(default))]
    tide: Option<Distance>,
}

impl EnvironmentSample {
    /// Empty sample for `point` at `when`.
    #[must_use]
    pub const fn at(point: GeodeticPoint, when: Instant<Utc>) -> Self {
        Self {
            point,
            when,
            magnetic: None,
            current: None,
            wind: None,
            tide: None,
        }
    }

    /// Sets the magnetic field.
    #[must_use]
    pub const fn with_magnetic(mut self, field: MagneticField) -> Self {
        self.magnetic = Some(field);
        self
    }

    /// Sets the current.
    #[must_use]
    pub const fn with_current(mut self, current: Current) -> Self {
        self.current = Some(current);
        self
    }

    /// Sets the wind.
    #[must_use]
    pub const fn with_wind(mut self, wind: Wind) -> Self {
        self.wind = Some(wind);
        self
    }

    /// Sets the height of tide above chart datum.
    ///
    /// Where there is no tide, set [`Distance::ZERO`]: an unset tide means "not
    /// asked", and calculations needing it report that instead of assuming
    /// zero.
    #[must_use]
    pub const fn with_tide(mut self, height: Distance) -> Self {
        self.tide = Some(height);
        self
    }

    /// Point of the sample.
    #[must_use]
    pub const fn point(&self) -> GeodeticPoint {
        self.point
    }

    /// Time of the sample.
    #[must_use]
    pub const fn when(&self) -> Instant<Utc> {
        self.when
    }

    /// Magnetic field, if resolved.
    #[must_use]
    pub const fn magnetic(&self) -> Option<MagneticField> {
        self.magnetic
    }

    /// Current, if resolved.
    #[must_use]
    pub const fn current(&self) -> Option<Current> {
        self.current
    }

    /// Wind, if resolved.
    #[must_use]
    pub const fn wind(&self) -> Option<Wind> {
        self.wind
    }

    /// Height of tide above chart datum, if resolved.
    #[must_use]
    pub const fn tide(&self) -> Option<Distance> {
        self.tide
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use alloc::format;

    use super::*;
    use crate::geodesy::Height;
    use crate::units::Distance;

    fn somewhere() -> GeodeticPoint {
        GeodeticPoint::new(
            Position::from_degrees(50.0, -4.0).unwrap(),
            Height::above_ellipsoid(Distance::ZERO),
        )
    }

    fn noon() -> Instant<Utc> {
        Instant::from_unix_seconds(1_700_000_000)
    }

    #[test]
    fn the_navigators_quantities_follow_from_the_components() {
        // Field 30° east of north, dip 60°.
        let horizontal = 20_000.0;
        let field = MagneticField::from_ned_nanotesla(
            horizontal * math::cos(math::to_radians(30.0)),
            horizontal * math::sin(math::to_radians(30.0)),
            horizontal * math::tan(math::to_radians(60.0)),
        )
        .unwrap();
        assert!((field.declination().degrees() - 30.0).abs() < 1e-9);
        assert!((field.inclination().degrees() - 60.0).abs() < 1e-9);
        assert!((field.horizontal_intensity_nanotesla() - horizontal).abs() < 1e-6);
        assert!((field.total_intensity_nanotesla() - 40_000.0).abs() < 1e-6);
    }

    #[test]
    fn declination_is_west_negative_and_dip_is_up_negative_in_the_south() {
        let field = MagneticField::from_ned_nanotesla(20_000.0, -5_000.0, -30_000.0).unwrap();
        assert!(field.declination().degrees() < 0.0);
        assert!(field.inclination().degrees() < 0.0);
        assert_eq!(format!("{}", field.declination()), "14.0°W");
    }

    #[test]
    fn a_field_that_cannot_be_the_earths_is_refused() {
        assert!(matches!(
            MagneticField::from_ned_nanotesla(f64::NAN, 0.0, 0.0),
            Err(KernelError::NotFinite { .. })
        ));
        // µT/gauss confusion.
        assert!(matches!(
            MagneticField::from_ned_nanotesla(0.0, 0.0, 500_000.0),
            Err(KernelError::OutOfRange {
                parameter: "field down component",
                ..
            })
        ));
    }

    #[test]
    fn a_vanishing_horizontal_field_has_no_declination_and_says_so_quietly() {
        let field = MagneticField::from_ned_nanotesla(0.0, 0.0, 50_000.0).unwrap();
        assert_eq!(field.horizontal_intensity_nanotesla(), 0.0);
        assert_eq!(field.declination(), Variation::ZERO);
        assert!((field.inclination().degrees() - 90.0).abs() < 1e-9);
    }

    #[test]
    fn the_field_is_displayed_as_a_navigator_would_write_it() {
        let field = MagneticField::from_ned_nanotesla(17_320.508, 10_000.0, 40_000.0).unwrap();
        assert_eq!(
            format!("{field}"),
            "D 30.0°E, I 63.4°, H 20000 nT, F 44721 nT"
        );
    }

    #[test]
    fn a_wind_is_named_by_where_it_comes_from() {
        let wind = Wind::new(
            TrueCourse::new(270.0).unwrap(),
            Speed::from_knots(15.0).unwrap(),
        )
        .unwrap();
        assert_eq!(wind.from().degrees(), 270.0);
        assert_eq!(wind.towards().degrees(), 90.0);
        assert_eq!(wind.speed().knots(), 15.0);
        assert_eq!(format!("{wind}"), "15.0 kn from 270.0°T");
        assert_eq!(Wind::CALM.speed(), Speed::ZERO);
    }

    #[test]
    fn a_wind_cannot_blow_backwards() {
        assert!(matches!(
            Wind::new(TrueCourse::NORTH, Speed::from_knots(-1.0).unwrap()),
            Err(KernelError::OutOfRange {
                parameter: "wind speed",
                ..
            })
        ));
    }

    #[test]
    fn slack_water_is_no_current() {
        assert_eq!(Current::SLACK.drift, Speed::ZERO);
    }

    /// Constant implementation of every port: minimal stand-ins, and proof that
    /// ports can be implemented outside the crate.
    struct Constant;

    impl MagneticModel for Constant {
        fn field_at(&self, _: GeodeticPoint, when: Instant<Utc>) -> Result<MagneticField> {
            if when < noon() {
                return Err(KernelError::OutsideValidity {
                    data: "magnetic model",
                });
            }
            MagneticField::from_ned_nanotesla(19_000.0, -1_000.0, 45_000.0)
        }
    }

    impl CompassModel for Constant {
        fn deviation(&self, course: CompassCourse) -> Result<Deviation> {
            Deviation::new(2.0 * math::sin(course.radians()))
        }
    }

    impl CurrentModel for Constant {
        fn current_at(&self, _: Position, _: Instant<Utc>) -> Result<Current> {
            Ok(Current {
                set: TrueCourse::new(45.0)?,
                drift: Speed::from_knots(1.5)?,
            })
        }
    }

    impl WindModel for Constant {
        fn wind_at(&self, _: Position, _: Instant<Utc>) -> Result<Wind> {
            Wind::new(TrueCourse::new(200.0)?, Speed::from_knots(20.0)?)
        }
    }

    impl LeewayModel for Constant {
        fn leeway(&self, motion: VesselMotion, wind: Wind) -> Result<Angle> {
            // Toy model: 5° downwind, whichever side.
            let relative =
                crate::angle::wrap180(wind.from().degrees() - motion.heading().degrees());
            Angle::from_degrees(if relative < 0.0 { 5.0 } else { -5.0 })
        }
    }

    #[test]
    fn the_ports_are_implementable_and_a_sample_is_built_from_their_answers() {
        let point = somewhere();
        let when = noon();
        let sample = EnvironmentSample::at(point, when)
            .with_magnetic(Constant.field_at(point, when).unwrap())
            .with_current(Constant.current_at(point.position(), when).unwrap())
            .with_wind(Constant.wind_at(point.position(), when).unwrap());
        assert_eq!(sample.point(), point);
        assert_eq!(sample.when(), when);
        assert!((sample.magnetic().unwrap().declination().degrees() + 3.0128).abs() < 1e-3);
        assert_eq!(sample.current().unwrap().drift.knots(), 1.5);
        assert_eq!(sample.wind().unwrap().from().degrees(), 200.0);
    }

    #[test]
    fn a_sample_is_honest_about_what_was_not_resolved() {
        let sample = EnvironmentSample::at(somewhere(), noon());
        assert_eq!(sample.magnetic(), None);
        assert_eq!(sample.current(), None);
        assert_eq!(sample.wind(), None);
        assert_eq!(sample.tide(), None);

        // A tide resolved as zero is an answer, not a gap.
        let slack = sample.with_tide(Distance::ZERO);
        assert_eq!(slack.tide(), Some(Distance::ZERO));
    }

    #[test]
    fn a_model_refuses_rather_than_guesses_outside_its_validity() {
        let earlier = Instant::from_unix_seconds(1_600_000_000);
        assert!(matches!(
            Constant.field_at(somewhere(), earlier),
            Err(KernelError::OutsideValidity {
                data: "magnetic model"
            })
        ));
    }

    #[test]
    fn deviation_and_leeway_ports_answer_for_a_course() {
        let deviation = Constant
            .deviation(CompassCourse::new(90.0).unwrap())
            .unwrap();
        assert!((deviation.degrees() - 2.0).abs() < 1e-9);

        let motion = VesselMotion::new(TrueCourse::NORTH, Speed::from_knots(6.0).unwrap());
        assert_eq!(motion.heading(), TrueCourse::NORTH);
        assert_eq!(motion.speed_through_water().knots(), 6.0);
        // Wind on the port beam sets the vessel to starboard.
        let from_port = Wind::new(
            TrueCourse::new(270.0).unwrap(),
            Speed::from_knots(20.0).unwrap(),
        )
        .unwrap();
        assert_eq!(Constant.leeway(motion, from_port).unwrap().degrees(), 5.0);
        let from_starboard = Wind::new(
            TrueCourse::new(90.0).unwrap(),
            Speed::from_knots(20.0).unwrap(),
        )
        .unwrap();
        assert_eq!(
            Constant.leeway(motion, from_starboard).unwrap().degrees(),
            -5.0
        );
    }

    #[cfg(feature = "serde")]
    #[test]
    fn serde_round_trips_and_validates_on_the_way_in() {
        let field = MagneticField::from_ned_nanotesla(19_000.0, -1_000.0, 45_000.0).unwrap();
        let json = serde_json::to_string(&field).unwrap();
        assert_eq!(
            json,
            r#"{"north_nanotesla":19000.0,"east_nanotesla":-1000.0,"down_nanotesla":45000.0}"#
        );
        assert_eq!(serde_json::from_str::<MagneticField>(&json).unwrap(), field);
        assert!(serde_json::from_str::<MagneticField>(
            r#"{"north_nanotesla":1e9,"east_nanotesla":0.0,"down_nanotesla":0.0}"#
        )
        .is_err());

        let wind = Wind::new(
            TrueCourse::new(200.0).unwrap(),
            Speed::from_knots(20.0).unwrap(),
        )
        .unwrap();
        let json = serde_json::to_string(&wind).unwrap();
        assert_eq!(serde_json::from_str::<Wind>(&json).unwrap(), wind);
        assert!(serde_json::from_str::<Wind>(r#"{"from":200.0,"speed":-1.0}"#).is_err());

        let sample = EnvironmentSample::at(somewhere(), noon())
            .with_magnetic(field)
            .with_wind(wind)
            .with_tide(Distance::from_metres(2.5).unwrap());
        let json = serde_json::to_string(&sample).unwrap();
        assert_eq!(
            serde_json::from_str::<EnvironmentSample>(&json).unwrap(),
            sample
        );

        // A sample stored without a tide reads back without one.
        let older = json.replace(",\"tide\":", ",\"ignored\":");
        assert_ne!(older, json);
        let read = serde_json::from_str::<EnvironmentSample>(&older).unwrap();
        assert_eq!(read.tide(), None);
        assert_eq!(read.wind(), Some(wind));
    }
}
