//! Man overboard datum.
//!
//! The alarm fixes a position and time. The casualty then drifts with the full
//! current and with a small fraction of the wind speed depending on the object
//! (person, person in lifejacket, liferaft). The *datum* is the drop position
//! propagated by both; it centres the search and the return manoeuvre.
//!
//! [`ManOverboard::datum`] computes it from an [`EnvironmentSample`] resolved
//! at the drop position. One sample for the whole interval is appropriate for
//! the first hour; longer searches re-resolve the environment and propagate
//! from the last datum. Components missing from the sample are not applied and
//! the datum reports it: a datum without current is still better than the drop
//! position.
//!
//! ```rust
//! use kinavis::mob::ManOverboard;
//! use kinavis::{
//!     Current, Distance, EnvironmentSample, GeodeticPoint, Height, Instant, Position, Speed,
//!     TrueCourse, Utc, Wind,
//! };
//! use core::time::Duration;
//!
//! let overboard: Position = "50°00.0'N 001°00.0'W".parse()?;
//! let alarm = Instant::<Utc>::from_unix_seconds(1_789_000_000);
//! let mob = ManOverboard::new(overboard, alarm);
//!
//! // A knot of current setting east, twenty knots of wind from the north.
//! let environment = EnvironmentSample::at(
//!     GeodeticPoint::new(overboard, Height::above_ellipsoid(Distance::ZERO)),
//!     alarm,
//! )
//! .with_current(Current { set: TrueCourse::EAST, drift: Speed::from_knots(1.0)? })
//! .with_wind(Wind::new(TrueCourse::NORTH, Speed::from_knots(20.0)?)?);
//!
//! // Half an hour later, a person in the water drifting at 2% of the wind.
//! let now = alarm.checked_add(Duration::from_secs(1800)).unwrap();
//! let datum = mob.datum(&environment, now, 0.02)?;
//! // Half a mile east by the current, a fifth of a mile south by the wind.
//! assert_eq!(format!("{:.2}", datum.drift().nautical_miles()), "0.54");
//! assert_eq!(format!("{:.0}", datum.set().unwrap().degrees()), "112");
//! assert!(datum.current_applied() && datum.wind_applied());
//! # Ok::<(), kinavis::NavigationError>(())
//! ```

use core::time::Duration;

use crate::angle::{Direction, True, TrueCourse};
use crate::environment::EnvironmentSample;
use crate::error::{ensure_range, Result};
use crate::math;
use crate::position::Position;
use crate::sailings::great_circle_destination;
use crate::time::{Instant, Utc};
use crate::units::Distance;

/// Time and position of a person entering the water.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ManOverboard {
    position: Position,
    at: Instant<Utc>,
}

impl ManOverboard {
    /// Alarm at the vessel's position when raised.
    #[must_use]
    pub const fn new(position: Position, at: Instant<Utc>) -> Self {
        Self { position, at }
    }

    /// Drop position.
    #[must_use]
    pub const fn position(&self) -> Position {
        self.position
    }

    /// Drop time.
    #[must_use]
    pub const fn at(&self) -> Instant<Utc> {
        self.at
    }

    /// Datum at `now`.
    ///
    /// Current in `env` applies at full rate; wind at `wind_factor` of its
    /// speed, downwind. Typical factors (IAMSAR leeway tables): ~1 % for a
    /// person, a few percent for a liferaft. Zero applies no wind.
    ///
    /// # Errors
    ///
    /// - [`KernelError::TimeReversed`] if `now` precedes the alarm.
    /// - [`KernelError::OutOfRange`] if the factor is outside `[0, 1]`.
    ///
    /// [`KernelError::TimeReversed`]: crate::KernelError::TimeReversed
    /// [`KernelError::OutOfRange`]: crate::KernelError::OutOfRange
    pub fn datum(
        &self,
        env: &EnvironmentSample,
        now: Instant<Utc>,
        wind_factor: f64,
    ) -> Result<MobDatum> {
        ensure_range("wind factor", wind_factor, 0.0, 1.0)?;
        let elapsed = now.duration_since(self.at)?;
        let hours = elapsed.as_secs_f64() / 3600.0;

        let mut north = 0.0;
        let mut east = 0.0;
        let current = env.current().filter(|current| current.drift.knots() > 0.0);
        if let Some(current) = current {
            let (n, e) = current.set.components(current.drift.knots() * hours);
            north += n;
            east += e;
        }
        let wind = env.wind().filter(|wind| wind.speed().knots() > 0.0);
        if let Some(wind) = wind {
            let (n, e) = wind
                .towards()
                .components((wind.speed() * wind_factor).knots() * hours);
            north += n;
            east += e;
        }

        let drift = Distance::from_nautical_miles(math::hypot(north, east))?;
        let (set, position) = if drift.nautical_miles() > 0.0 {
            let set =
                Direction::<True>::from_degrees_wrapped(math::to_degrees(math::atan2(east, north)));
            (
                Some(set),
                great_circle_destination(self.position, set, drift)?.position,
            )
        } else {
            (None, self.position)
        };

        Ok(MobDatum {
            at: now,
            position,
            elapsed,
            drift,
            set,
            current_applied: current.is_some(),
            wind_applied: wind.is_some() && wind_factor > 0.0,
        })
    }
}

/// Datum at one instant.
///
/// Projection built by [`ManOverboard::datum`], stating which drifts were
/// applied.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct MobDatum {
    at: Instant<Utc>,
    position: Position,
    elapsed: Duration,
    drift: Distance,
    set: Option<TrueCourse>,
    current_applied: bool,
    wind_applied: bool,
}

impl MobDatum {
    /// Time of the datum.
    #[must_use]
    pub const fn at(&self) -> Instant<Utc> {
        self.at
    }

    /// Datum position.
    #[must_use]
    pub const fn position(&self) -> Position {
        self.position
    }

    /// Time in the water.
    #[must_use]
    pub const fn elapsed(&self) -> Duration {
        self.elapsed
    }

    /// Drift distance from the drop position.
    #[must_use]
    pub const fn drift(&self) -> Distance {
        self.drift
    }

    /// Drift direction; `None` if zero.
    #[must_use]
    pub const fn set(&self) -> Option<TrueCourse> {
        self.set
    }

    /// Whether current was present and applied.
    #[must_use]
    pub const fn current_applied(&self) -> bool {
        self.current_applied
    }

    /// Whether wind was present and applied with a non-zero factor.
    #[must_use]
    pub const fn wind_applied(&self) -> bool {
        self.wind_applied
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use crate::environment::{Current, Wind};
    use crate::error::{KernelError, NavigationError};
    use crate::geodesy::{GeodeticPoint, Height};
    use crate::units::Speed;

    fn at(latitude: f64, longitude: f64) -> Position {
        Position::from_degrees(latitude, longitude).unwrap()
    }

    fn alarm() -> Instant<Utc> {
        Instant::from_unix_seconds(1_789_000_000)
    }

    fn later(minutes: u64) -> Instant<Utc> {
        alarm()
            .checked_add(Duration::from_secs(minutes * 60))
            .unwrap()
    }

    fn knots(value: f64) -> Speed {
        Speed::from_knots(value).unwrap()
    }

    fn sample(position: Position) -> EnvironmentSample {
        EnvironmentSample::at(
            GeodeticPoint::new(position, Height::above_ellipsoid(Distance::ZERO)),
            alarm(),
        )
    }

    #[test]
    fn with_nothing_resolved_the_datum_is_where_they_went_in() {
        let here = at(50.0, -1.0);
        let mob = ManOverboard::new(here, alarm());
        let datum = mob.datum(&sample(here), later(30), 0.02).unwrap();
        assert_eq!(datum.position(), here);
        assert_eq!(datum.drift(), Distance::ZERO);
        assert_eq!(datum.set(), None);
        assert!(!datum.current_applied());
        assert!(!datum.wind_applied());
        assert_eq!(datum.elapsed(), Duration::from_secs(1800));
        assert_eq!(datum.at(), later(30));
        assert_eq!(mob.position(), here);
        assert_eq!(mob.at(), alarm());
    }

    #[test]
    fn the_current_carries_the_datum_at_its_full_rate() {
        let here = at(50.0, -1.0);
        let mob = ManOverboard::new(here, alarm());
        let environment = sample(here).with_current(Current {
            set: TrueCourse::EAST,
            drift: knots(2.0),
        });
        let datum = mob.datum(&environment, later(30), 0.02).unwrap();
        assert!((datum.drift().nautical_miles() - 1.0).abs() < 1e-9);
        assert!((datum.set().unwrap().degrees() - 90.0).abs() < 1e-9);
        assert!(datum.current_applied());
        assert!(!datum.wind_applied());
        assert!(datum.position().longitude().degrees() > -1.0);
        assert!((datum.position().latitude().degrees() - 50.0).abs() < 1e-4);
    }

    #[test]
    fn the_wind_carries_it_downwind_by_its_factor() {
        let here = at(50.0, -1.0);
        let mob = ManOverboard::new(here, alarm());
        let environment =
            sample(here).with_wind(Wind::new(TrueCourse::NORTH, knots(20.0)).unwrap());

        // 2 % of 20 kn for 1 h: 0.4 NM south.
        let datum = mob.datum(&environment, later(60), 0.02).unwrap();
        assert!((datum.drift().nautical_miles() - 0.4).abs() < 1e-9);
        assert!((datum.set().unwrap().degrees() - 180.0).abs() < 1e-9);
        assert!(datum.wind_applied());

        // Zero factor: no wind applied, reported as such.
        let still = mob.datum(&environment, later(60), 0.0).unwrap();
        assert_eq!(still.drift(), Distance::ZERO);
        assert!(!still.wind_applied());
    }

    #[test]
    fn the_two_drifts_add_as_vectors() {
        let here = at(50.0, -1.0);
        let mob = ManOverboard::new(here, alarm());
        let environment = sample(here)
            .with_current(Current {
                set: TrueCourse::EAST,
                drift: knots(1.0),
            })
            .with_wind(Wind::new(TrueCourse::NORTH, knots(20.0)).unwrap());
        // 1 h: 1 NM east, 0.4 NM south: 1.077 NM on 112°.
        let datum = mob.datum(&environment, later(60), 0.02).unwrap();
        assert!((datum.drift().nautical_miles() - math::hypot(1.0, 0.4)).abs() < 1e-9);
        assert!((datum.set().unwrap().degrees() - 111.8).abs() < 0.1);
    }

    #[test]
    fn the_datum_grows_with_time_and_refuses_to_go_backwards() {
        let here = at(50.0, -1.0);
        let mob = ManOverboard::new(here, alarm());
        let environment = sample(here).with_current(Current {
            set: TrueCourse::EAST,
            drift: knots(1.0),
        });
        let mut previous = 0.0;
        for minutes in [10, 20, 60, 180] {
            let drift = mob
                .datum(&environment, later(minutes), 0.0)
                .unwrap()
                .drift()
                .nautical_miles();
            assert!(drift > previous);
            previous = drift;
        }
        assert!(matches!(
            mob.datum(
                &environment,
                alarm().checked_sub(Duration::from_secs(1)).unwrap(),
                0.0
            )
            .unwrap_err(),
            NavigationError::Kernel(KernelError::TimeReversed { .. })
        ));
        // At the alarm time: zero drift.
        let now = mob.datum(&environment, alarm(), 0.0).unwrap();
        assert_eq!(now.drift(), Distance::ZERO);
        assert!(now.current_applied());
    }

    #[test]
    fn the_wind_factor_is_a_fraction() {
        let here = at(50.0, -1.0);
        let mob = ManOverboard::new(here, alarm());
        for factor in [-0.1, 1.5, f64::NAN] {
            assert!(mob.datum(&sample(here), later(10), factor).is_err());
        }
    }
}
