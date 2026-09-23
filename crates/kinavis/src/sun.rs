//! Solar position, sunrise, sunset and twilights.
//!
//! Solar azimuth is the standard compass check: compare a bearing of the sun
//! with the computed azimuth; [`gyro_error_from_transit`] computes the error.
//! Sunrise, sunset and twilight times govern navigation lights, sextant
//! horizon, and "arrive in daylight" planning.
//!
//! Solar position follows NOAA's solar calculator (Meeus, *Astronomical
//! Algorithms*, ch. 25): ~`0.01°` in apparent longitude for 1800–2200, a few
//! tenths of a minute in azimuth. UT is used in place of TT; the ~1 min
//! difference moves the azimuth by ~0.01° and is not corrected. Rise and set
//! are found by iterating the hour angle at which the sun crosses the chosen
//! [`Horizon`], to a few seconds; published tables round to the minute.
//!
//! ```rust
//! use kinavis::sun::{solar_position, sun_over_horizon, Crossing, Horizon};
//! use kinavis::{Civil, Instant, Position, Utc};
//!
//! // Off the Lizard, midsummer morning.
//! let here = Position::from_degrees(49.9, -5.2)?;
//! let at = Instant::<Utc>::from_civil(Civil { hour: 6, ..Civil::date(2026, 6, 21) })?;
//! let sun = solar_position(here, at)?;
//! assert_eq!(format!("{:.0}", sun.azimuth().degrees()), "70");
//! assert!(sun.altitude().degrees() > 10.0);
//!
//! // The day's sunrise and sunset, and when the lights can go off.
//! let Crossing::Rises { up, down } = sun_over_horizon(here, Civil::date(2026, 6, 21), Horizon::Visible)? else {
//!     panic!("the sun rises and sets at this latitude")
//! };
//! assert_eq!(format!("{}", up), "2026-06-21T04:11:58.455 UTC");
//! assert_eq!(format!("{}", down), "2026-06-21T20:33:16.544 UTC");
//! # Ok::<(), kinavis::NavigationError>(())
//! ```
//!
//! [`gyro_error_from_transit`]: crate::navigation_solutions::gyro_error_from_transit

use core::time::Duration;

use crate::angle::{wrap180, wrap360, TrueBearing};
use crate::error::{KernelError, NavigationError, Result};
use crate::math;
use crate::position::Position;
use crate::time::{Civil, Instant, Utc};
use crate::units::Angle;

/// Julian day of the Unix epoch.
const JULIAN_DAY_OF_UNIX_EPOCH: f64 = 2_440_587.5;
/// Julian day of J2000.0.
const J2000: f64 = 2_451_545.0;
/// Seconds per day.
const SECONDS_PER_DAY: f64 = 86_400.0;

/// Solar position from a place at an instant.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SolarPosition {
    azimuth: TrueBearing,
    altitude: Angle,
    declination: Angle,
    equation_of_time_minutes: f64,
}

impl SolarPosition {
    /// True bearing of the sun's centre.
    #[must_use]
    pub const fn azimuth(&self) -> TrueBearing {
        self.azimuth
    }

    /// Geometric altitude of the centre, negative below the horizon; no
    /// refraction.
    #[must_use]
    pub const fn altitude(&self) -> Angle {
        self.altitude
    }

    /// Declination.
    #[must_use]
    pub const fn declination(&self) -> Angle {
        self.declination
    }

    /// Equation of time, minutes: apparent minus mean solar time. Positive in
    /// autumn.
    #[must_use]
    pub const fn equation_of_time_minutes(&self) -> f64 {
        self.equation_of_time_minutes
    }
}

/// Altitude defining rise/set.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum Horizon {
    /// Upper limb on the sea horizon: centre at `−0°50′` (refraction +
    /// semi-diameter). Almanac sunrise and sunset.
    Visible,
    /// Centre `6°` below: civil twilight.
    Civil,
    /// Centre `12°` below: nautical twilight (star sights with a visible
    /// horizon).
    Nautical,
    /// Centre `18°` below: astronomical twilight; fully dark afterwards.
    Astronomical,
}

impl Horizon {
    /// Altitude of the centre at this horizon, degrees.
    #[must_use]
    pub const fn altitude_degrees(self) -> f64 {
        match self {
            Self::Visible => -0.833,
            Self::Civil => -6.0,
            Self::Nautical => -12.0,
            Self::Astronomical => -18.0,
        }
    }
}

/// Whether and when the sun crosses a horizon on a given day.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Crossing {
    /// Rises at `up`, sets at `down`.
    Rises {
        /// Rise time of the centre through the horizon.
        up: Instant<Utc>,
        /// Set time.
        down: Instant<Utc>,
    },
    /// Above the horizon all day (midnight sun, or continuous twilight).
    AlwaysAbove,
    /// Below the horizon all day (polar night).
    AlwaysBelow,
}

/// Solar position from `at` at `when`.
///
/// # Errors
///
/// [`KernelError::OutsideValidity`] outside 1800–2200.
pub fn solar_position(at: Position, when: Instant<Utc>) -> Result<SolarPosition> {
    let sun = Sun::at(julian_day(when)?);
    let hour_angle = sun.hour_angle(at, when);
    let (azimuth, altitude) = horizontal(at, sun.declination_radians, hour_angle);
    Ok(SolarPosition {
        azimuth: TrueBearing::from_degrees_wrapped(math::to_degrees(azimuth)),
        altitude: Angle::from_radians(altitude)?,
        declination: Angle::from_radians(sun.declination_radians)?,
        equation_of_time_minutes: sun.equation_of_time_minutes,
    })
}

/// Crossings of `horizon` on the local day `date` at `at`.
///
/// The local day runs midnight to midnight in the longitude's mean solar time,
/// so a vessel at 170°E gets its own sunrise, not one 12 h off. Instants are
/// UTC.
///
/// # Errors
///
/// [`KernelError::OutsideValidity`] outside 1800–2200; calendar errors in
/// `date`.
pub fn sun_over_horizon(at: Position, date: Civil, horizon: Horizon) -> Result<Crossing> {
    let noon = solar_noon(at, date)?;
    let latitude = at.latitude().radians();
    let target = math::to_radians(horizon.altitude_degrees());

    // Hour angle at which the centre is on the horizon for the current
    // declination; `None` if never.
    let half_day = |sun: &Sun| -> Option<f64> {
        let declination = sun.declination_radians;
        let cos_hour_angle = (math::sin(target) - math::sin(latitude) * math::sin(declination))
            / (math::cos(latitude) * math::cos(declination));
        (cos_hour_angle.abs() <= 1.0).then(|| math::acos(cos_hour_angle))
    };

    let Some(arc) = half_day(&Sun::at(julian_day(noon)?)) else {
        return polar(at, noon, target);
    };
    // Morning and evening are refined separately (declination and equation of
    // time change over half a day), iterating until the hour angle matches the
    // arc.
    let refine = |sign: f64| -> Result<Option<Instant<Utc>>> {
        let mut guess = shift(noon, sign * arc / core::f64::consts::TAU * SECONDS_PER_DAY)?;
        for _ in 0..4 {
            let sun = Sun::at(julian_day(guess)?);
            let Some(arc) = half_day(&sun) else {
                return Ok(None);
            };
            let error = sign * arc - sun.hour_angle(at, guess);
            guess = shift(guess, error / core::f64::consts::TAU * SECONDS_PER_DAY)?;
        }
        Ok(Some(guess))
    };
    match (refine(-1.0)?, refine(1.0)?) {
        (Some(up), Some(down)) => Ok(Crossing::Rises { up, down }),
        _ => polar(at, noon, target),
    }
}

/// Meridian transit at `at` on the local day.
///
/// # Errors
///
/// As [`sun_over_horizon`].
pub fn solar_noon(at: Position, date: Civil) -> Result<Instant<Utc>> {
    let midnight = Instant::<Utc>::from_civil(Civil::date(date.year, date.month, date.day))?;
    // Mean noon at this longitude: 12:00 UTC minus longitude in hours.
    let longitude_seconds = at.longitude().degrees() / 360.0 * SECONDS_PER_DAY;
    let mean_noon = shift(midnight, 12.0 * 3600.0 - longitude_seconds)?;
    // Apparent noon = mean noon − equation of time, iterated twice so it is
    // evaluated at the result.
    let mut noon = mean_noon;
    for _ in 0..2 {
        let equation = Sun::at(julian_day(noon)?).equation_of_time_minutes;
        noon = shift(mean_noon, -equation * 60.0)?;
    }
    Ok(noon)
}

/// Which way the crossing fails, judged at noon (maximum altitude).
fn polar(at: Position, noon: Instant<Utc>, target: f64) -> Result<Crossing> {
    let sun = Sun::at(julian_day(noon)?);
    let (_, altitude) = horizontal(at, sun.declination_radians, 0.0);
    Ok(if altitude > target {
        Crossing::AlwaysAbove
    } else {
        Crossing::AlwaysBelow
    })
}

/// Apparent solar position at a Julian day (Meeus ch. 25).
#[derive(Debug, Clone, Copy)]
struct Sun {
    declination_radians: f64,
    equation_of_time_minutes: f64,
}

impl Sun {
    fn at(julian_day: f64) -> Self {
        let t = (julian_day - J2000) / 36_525.0;
        // Geometric mean longitude and mean anomaly, degrees.
        let mean_longitude = wrap360(280.466_46 + t * (36_000.769_83 + t * 0.000_303_2));
        let mean_anomaly = math::to_radians(357.529_11 + t * (35_999.050_29 - t * 0.000_153_7));
        let eccentricity = 0.016_708_634 - t * (0.000_042_037 + t * 0.000_000_126_7);
        let centre = math::sin(mean_anomaly) * (1.914_602 - t * (0.004_817 + t * 0.000_014))
            + math::sin(2.0 * mean_anomaly) * (0.019_993 - t * 0.000_101)
            + math::sin(3.0 * mean_anomaly) * 0.000_289;
        let true_longitude = mean_longitude + centre;
        let ascending_node = math::to_radians(125.04 - 1934.136 * t);
        let apparent_longitude =
            math::to_radians(true_longitude - 0.005_69 - 0.004_78 * math::sin(ascending_node));
        let mean_obliquity =
            23.0 + (26.0 + (21.448 - t * (46.815 + t * (0.000_59 - t * 0.001_813))) / 60.0) / 60.0;
        let obliquity = math::to_radians(mean_obliquity + 0.002_56 * math::cos(ascending_node));

        let declination_radians = math::asin(math::sin(obliquity) * math::sin(apparent_longitude));

        let y = math::tan(obliquity / 2.0);
        let y = y * y;
        let l0 = math::to_radians(mean_longitude);
        let equation = y * math::sin(2.0 * l0) - 2.0 * eccentricity * math::sin(mean_anomaly)
            + 4.0 * eccentricity * y * math::sin(mean_anomaly) * math::cos(2.0 * l0)
            - 0.5 * y * y * math::sin(4.0 * l0)
            - 1.25 * eccentricity * eccentricity * math::sin(2.0 * mean_anomaly);
        Self {
            declination_radians,
            equation_of_time_minutes: 4.0 * math::to_degrees(equation),
        }
    }

    /// Local hour angle at `at`, `when`, radians, west positive: zero at
    /// apparent noon, negative in the morning.
    fn hour_angle(&self, at: Position, when: Instant<Utc>) -> f64 {
        let seconds_of_day = when.seconds().rem_euclid(86_400);
        // `seconds_of_day` in `[0, 86 400)`: exact conversion.
        #[allow(clippy::cast_precision_loss)]
        let utc_minutes = seconds_of_day as f64 / 60.0 + f64::from(when.subsec_nanos()) / 6e10;
        let true_solar_minutes =
            utc_minutes + self.equation_of_time_minutes + 4.0 * at.longitude().degrees();
        math::to_radians(wrap180(true_solar_minutes / 4.0 - 180.0))
    }
}

/// Azimuth (from north through east) and altitude, radians, of a body at
/// `declination` and `hour_angle` seen from `at`.
fn horizontal(at: Position, declination: f64, hour_angle: f64) -> (f64, f64) {
    let latitude = at.latitude().radians();
    let (sin_lat, cos_lat) = (math::sin(latitude), math::cos(latitude));
    let (sin_dec, cos_dec) = (math::sin(declination), math::cos(declination));
    let (sin_ha, cos_ha) = (math::sin(hour_angle), math::cos(hour_angle));
    let altitude = math::asin((sin_lat * sin_dec + cos_lat * cos_dec * cos_ha).clamp(-1.0, 1.0));
    // East and north components of the direction to the body.
    let east = -sin_ha * cos_dec;
    let north = cos_lat * sin_dec - sin_lat * cos_dec * cos_ha;
    let azimuth = if east == 0.0 && north == 0.0 {
        0.0
    } else {
        math::atan2(east, north)
    };
    (azimuth, altitude)
}

/// Julian day of an instant within the theory's validity.
fn julian_day(when: Instant<Utc>) -> Result<f64> {
    let year = when.civil().year;
    if !(1800..=2200).contains(&year) {
        return Err(NavigationError::Kernel(KernelError::OutsideValidity {
            data: "solar theory",
        }));
    }
    // The year check keeps seconds well inside `f64`'s exact range.
    #[allow(clippy::cast_precision_loss)]
    let seconds = when.seconds() as f64 + f64::from(when.subsec_nanos()) / 1e9;
    Ok(seconds / SECONDS_PER_DAY + JULIAN_DAY_OF_UNIX_EPOCH)
}

/// Non-negative seconds as a `Duration`.
fn seconds(value: f64) -> Result<Duration> {
    Duration::try_from_secs_f64(value).map_err(|_| {
        NavigationError::Kernel(KernelError::Indeterminate {
            quantity: "sun's hour angle",
        })
    })
}

/// `from` shifted by signed seconds.
fn shift(from: Instant<Utc>, by_seconds: f64) -> Result<Instant<Utc>> {
    let magnitude = seconds(math::abs(by_seconds))?;
    Ok(if by_seconds < 0.0 {
        from.saturating_sub(magnitude)
    } else {
        from.saturating_add(magnitude)
    })
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp, clippy::panic)]
mod tests {
    use super::*;

    fn at(civil: Civil) -> Instant<Utc> {
        Instant::from_civil(civil).unwrap()
    }

    fn position(latitude: f64, longitude: f64) -> Position {
        Position::from_degrees(latitude, longitude).unwrap()
    }

    #[test]
    fn meeus_example_25a_the_suns_place_on_1992_october_13() {
        // Meeus example 25.a: 1992 October 13.0 TD, apparent declination
        // −7.78507° (low-accuracy method, as here); equation of time 13.71 min
        // from example 28.a, same date.
        let sun = Sun::at(2_448_908.5);
        assert!((math::to_degrees(sun.declination_radians) + 7.785_07).abs() < 0.001);
        assert!((sun.equation_of_time_minutes - 13.71).abs() < 0.02);
    }

    #[test]
    fn the_equation_of_time_has_its_known_extremes() {
        let november = Sun::at(julian_day(at(Civil::date(2026, 11, 3))).unwrap());
        assert!((november.equation_of_time_minutes - 16.4).abs() < 0.2);
        let february = Sun::at(julian_day(at(Civil::date(2026, 2, 11))).unwrap());
        assert!((february.equation_of_time_minutes + 14.2).abs() < 0.2);
    }

    #[test]
    fn at_noon_on_the_solstice_the_sun_is_where_the_geometry_says() {
        // 50°N, midsummer: transit altitude 90 − 50 + 23.44.
        let here = position(50.0, 0.0);
        let noon = solar_noon(here, Civil::date(2026, 6, 21)).unwrap();
        let sun = solar_position(here, noon).unwrap();
        assert!((sun.altitude().degrees() - 63.44).abs() < 0.05);
        assert!(
            sun.azimuth()
                .angular_distance(TrueBearing::new(180.0).unwrap())
                < 0.05
        );
        assert!((sun.declination().degrees() - 23.44).abs() < 0.02);
    }

    #[test]
    fn on_the_equinox_at_the_equator_the_sun_rises_due_east_and_the_day_is_twelve_hours() {
        let here = position(0.0, 0.0);
        let Crossing::Rises { up, down } =
            sun_over_horizon(here, Civil::date(2026, 3, 20), Horizon::Visible).unwrap()
        else {
            panic!("the sun rises at the equator")
        };
        // Refraction and semi-diameter lengthen the day by minutes.
        let length = down.duration_since(up).unwrap();
        assert!(length > Duration::from_secs(12 * 3600 + 5 * 60));
        assert!(length < Duration::from_secs(12 * 3600 + 9 * 60));
        let rising = solar_position(here, up).unwrap();
        assert!(
            rising
                .azimuth()
                .angular_distance(TrueBearing::new(90.0).unwrap())
                < 1.0
        );
        assert!(
            (rising.altitude().degrees() + 0.833).abs() < 0.01,
            "altitude at rising: {}",
            rising.altitude().degrees()
        );
        let setting = solar_position(here, down).unwrap();
        assert!(
            setting
                .azimuth()
                .angular_distance(TrueBearing::new(270.0).unwrap())
                < 1.0
        );
    }

    #[test]
    fn the_morning_sun_is_in_the_east_and_the_afternoon_sun_in_the_west() {
        let here = position(49.9, -5.2);
        let morning = solar_position(
            here,
            at(Civil {
                hour: 8,
                ..Civil::date(2026, 6, 21)
            }),
        )
        .unwrap();
        assert!(morning.azimuth().degrees() > 60.0 && morning.azimuth().degrees() < 120.0);
        let afternoon = solar_position(
            here,
            at(Civil {
                hour: 17,
                ..Civil::date(2026, 6, 21)
            }),
        )
        .unwrap();
        assert!(afternoon.azimuth().degrees() > 240.0 && afternoon.azimuth().degrees() < 300.0);
    }

    #[test]
    fn the_twilights_come_in_order_and_last_longer_in_the_north() {
        let here = position(49.9, -5.2);
        let date = Civil::date(2026, 9, 16);
        let up = |horizon| match sun_over_horizon(here, date, horizon).unwrap() {
            Crossing::Rises { up, .. } => up,
            other => panic!("{other:?}"),
        };
        assert!(up(Horizon::Astronomical) < up(Horizon::Nautical));
        assert!(up(Horizon::Nautical) < up(Horizon::Civil));
        assert!(up(Horizon::Civil) < up(Horizon::Visible));
        // Civil twilight lasts ~30 min at this latitude in September, ~20 min
        // at the equator.
        let civil_here = up(Horizon::Visible)
            .duration_since(up(Horizon::Civil))
            .unwrap();
        assert!(
            civil_here > Duration::from_secs(28 * 60) && civil_here < Duration::from_secs(40 * 60)
        );
    }

    #[test]
    fn beyond_the_arctic_circle_the_sun_neither_rises_nor_sets() {
        let north = position(75.0, 20.0);
        assert_eq!(
            sun_over_horizon(north, Civil::date(2026, 6, 21), Horizon::Visible).unwrap(),
            Crossing::AlwaysAbove
        );
        assert_eq!(
            sun_over_horizon(north, Civil::date(2026, 12, 21), Horizon::Visible).unwrap(),
            Crossing::AlwaysBelow
        );
        // Polar night at 75°N still has nautical twilight.
        assert!(matches!(
            sun_over_horizon(north, Civil::date(2026, 12, 21), Horizon::Nautical).unwrap(),
            Crossing::Rises { .. }
        ));
    }

    #[test]
    fn the_local_day_follows_the_longitude() {
        // At 170°E sunrise is ~18:00 UTC the previous day; that is the answer,
        // not one 12 h off.
        let east = position(-40.0, 170.0);
        let Crossing::Rises { up, .. } =
            sun_over_horizon(east, Civil::date(2026, 3, 20), Horizon::Visible).unwrap()
        else {
            panic!()
        };
        let civil = up.civil();
        assert_eq!((civil.month, civil.day), (3, 19));
        assert!(civil.hour >= 17 && civil.hour <= 19);
    }

    /// USNO "Rise/Set/Transit Times" (`aa.usno.navy.mil/api/rstt/oneday`),
    /// queried 2026-09-16, published to the minute.
    #[test]
    fn agrees_with_the_naval_observatory_to_the_minute() {
        let cases = [
            // (lat, lon, y, m, d), (civil begin, rise, transit, set, civil end)
            (
                (49.9, -5.2, 2026, 6, 21),
                ((3, 27), (4, 12), (12, 23), (20, 33), (21, 18)),
            ),
            (
                (0.0, 0.0, 2026, 3, 20),
                ((5, 44), (6, 4), (12, 7), (18, 11), (18, 31)),
            ),
        ];
        let hm = |instant: Instant<Utc>| {
            // Rounded to the minute, as published.
            let rounded = instant.saturating_add(Duration::from_secs(30)).civil();
            (rounded.hour, rounded.minute)
        };
        for ((lat, lon, y, m, d), (civil_begin, rise, transit, set, civil_end)) in cases {
            let here = position(lat, lon);
            let date = Civil::date(y, m, d);
            let Crossing::Rises { up, down } =
                sun_over_horizon(here, date, Horizon::Visible).unwrap()
            else {
                panic!()
            };
            let Crossing::Rises {
                up: civil_up,
                down: civil_down,
            } = sun_over_horizon(here, date, Horizon::Civil).unwrap()
            else {
                panic!()
            };
            assert_eq!(hm(civil_up), civil_begin);
            assert_eq!(hm(up), rise);
            assert_eq!(hm(solar_noon(here, date).unwrap()), transit);
            assert_eq!(hm(down), set);
            assert_eq!(hm(civil_down), civil_end);
        }
    }

    #[test]
    fn outside_the_theorys_years_there_is_no_answer() {
        assert!(matches!(
            solar_position(position(0.0, 0.0), at(Civil::date(1750, 1, 1))),
            Err(NavigationError::Kernel(KernelError::OutsideValidity {
                data: "solar theory"
            }))
        ));
        assert!(sun_over_horizon(
            position(0.0, 0.0),
            Civil::date(2300, 1, 1),
            Horizon::Visible
        )
        .is_err());
    }
}
