//! Tidal stream from a tidal diamond, as a [`CurrentModel`].

use core::time::Duration;

use crate::conditions::Interpolable;
use crate::environment::{Current, CurrentModel};
use crate::error::{ensure_range, KernelError, NavigationError, Result};
use crate::math;
use crate::position::Position;
use crate::time::{Instant, Utc};
use crate::units::Distance;

/// Hours either side of HW tabulated by a diamond.
pub const HOURS_EITHER_SIDE: usize = 6;

/// Diamond rows: one per hour, `−6` to `+6`.
const ROWS: usize = 2 * HOURS_EITHER_SIDE + 1;

/// Diamond row: spring and neap streams at one hour relative to HW at the
/// reference port.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct StreamHour {
    /// Spring stream.
    pub springs: Current,
    /// Neap stream.
    pub neaps: Current,
}

/// Tidal diamond for one day: hourly streams relative to reference-port HW,
/// springs and neaps, evaluated for the day's range and the requested time.
///
/// Thirteen rows, HW−6 to HW+6. Between rows the stream is interpolated as a
/// vector; between neaps and springs by the range factor (`0` neaps, `1`
/// springs) from [`range_factor`](Self::range_factor), as in the tables'
/// computation-of-rates diagram. Position is ignored: the caller chose the
/// diamond.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TidalStream {
    rows: [StreamHour; ROWS],
    high_water: Instant<Utc>,
    range_factor: f64,
}

impl TidalStream {
    /// Diamond from rows HW−6 to HW+6, for the day's reference-port HW and
    /// range factor.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] for a range factor outside `[0, 2]` (tables
    /// extrapolate slightly beyond springs, not to twice).
    pub fn new(
        rows: [StreamHour; ROWS],
        high_water: Instant<Utc>,
        range_factor: f64,
    ) -> Result<Self> {
        ensure_range("range factor", range_factor, 0.0, 2.0)?;
        Ok(Self {
            rows,
            high_water,
            range_factor,
        })
    }

    /// Range factor from the day's reference-port range: `0` at mean neap
    /// range, `1` at mean spring range, linear between and slightly beyond.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] unless spring range > neap range and the
    /// factor is within `[0, 2]`.
    pub fn range_factor(today: Distance, neaps: Distance, springs: Distance) -> Result<f64> {
        let spread = (springs - neaps).metres();
        ensure_range(
            "spring range above neap",
            spread,
            f64::MIN_POSITIVE,
            f64::MAX,
        )?;
        let factor = (today - neaps).metres() / spread;
        ensure_range("range factor", factor, 0.0, 2.0)?;
        Ok(factor)
    }

    /// Reference-port HW of the diamond.
    #[must_use]
    pub const fn high_water(&self) -> Instant<Utc> {
        self.high_water
    }

    /// Stream at `hours` from HW (negative before).
    ///
    /// # Errors
    ///
    /// [`KernelError::OutsideValidity`] beyond ±6 h or for non-finite time.
    pub fn at_hour(&self, hours: f64) -> Result<Current> {
        let limit = math::count_to_f64(HOURS_EITHER_SIDE);
        if !hours.is_finite() || hours < -limit || hours > limit {
            return Err(NavigationError::Kernel(KernelError::OutsideValidity {
                data: "tidal diamond",
            }));
        }
        let shifted = hours + limit;
        // `shifted` in `[0, 12]`: safe to truncate.
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let index = (shifted as usize).min(ROWS - 2);
        let fraction = shifted - math::count_to_f64(index);
        let outside = || {
            NavigationError::Kernel(KernelError::OutsideValidity {
                data: "tidal diamond",
            })
        };
        let before = self.rows.get(index).ok_or_else(outside)?;
        let after = self.rows.get(index + 1).ok_or_else(outside)?;
        let springs = before.springs.interpolate(after.springs, fraction)?;
        let neaps = before.neaps.interpolate(after.neaps, fraction)?;
        // Linear blend of neap and spring vectors by range factor, as in the
        // rates diagram.
        Ok(neaps.interpolate(springs, self.range_factor)?)
    }
}

impl CurrentModel for TidalStream {
    fn current_at(
        &self,
        _: Position,
        when: Instant<Utc>,
    ) -> kinavis_kernel::error::Result<Current> {
        let hours = match when.checked_duration_since(self.high_water) {
            Some(after) => hours_of(after),
            None => -hours_of(
                self.high_water
                    .checked_duration_since(when)
                    .unwrap_or_default(),
            ),
        };
        self.at_hour(hours).map_err(NavigationError::into_kernel)
    }
}

fn hours_of(duration: Duration) -> f64 {
    duration.as_secs_f64() / 3600.0
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use crate::angle::TrueCourse;
    use crate::time::Civil;
    use crate::units::Speed;

    fn current(set: f64, knots: f64) -> Current {
        Current {
            set: TrueCourse::new(set).unwrap(),
            drift: Speed::from_knots(knots).unwrap(),
        }
    }

    fn high_water() -> Instant<Utc> {
        Instant::from_civil(Civil {
            hour: 10,
            ..Civil::date(2026, 9, 15)
        })
        .unwrap()
    }

    /// Channel diamond: floods east, ebbs west, slack 1 h after HW, springs
    /// twice neaps.
    fn diamond() -> TidalStream {
        let rates = [
            0.4, 1.2, 1.8, 2.0, 1.6, 0.8, 0.4, 0.2, 0.6, 1.4, 1.9, 1.5, 0.5,
        ];
        let rows: [StreamHour; ROWS] = core::array::from_fn(|index| {
            // Flood east through HW+1, ebb west after.
            let set = if index <= HOURS_EITHER_SIDE + 1 {
                90.0
            } else {
                270.0
            };
            let springs = rates.get(index).copied().unwrap();
            StreamHour {
                springs: current(set, springs),
                neaps: current(set, springs / 2.0),
            }
        });
        TidalStream::new(rows, high_water(), 0.5).unwrap()
    }

    #[test]
    fn the_rows_are_read_at_the_hours_and_blended_between_springs_and_neaps() {
        let stream = diamond();
        // At HW, midway between springs (0.4) and neaps (0.2).
        let at_high_water = stream.at_hour(0.0).unwrap();
        assert!(
            at_high_water
                .set
                .angular_distance(TrueCourse::new(90.0).unwrap())
                < 1e-9
        );
        assert!((at_high_water.drift.knots() - 0.3).abs() < 1e-12);
        // HW−3, maximum flood: springs 2.0, neaps 1.0.
        assert!((stream.at_hour(-3.0).unwrap().drift.knots() - 1.5).abs() < 1e-12);
        // ±6 h: table ends.
        assert!((stream.at_hour(-6.0).unwrap().drift.knots() - 0.3).abs() < 1e-12);
        let last = stream.at_hour(6.0).unwrap();
        assert!((last.drift.knots() - 0.375).abs() < 1e-12);
        assert!(last.set.angular_distance(TrueCourse::new(270.0).unwrap()) < 1e-9);
    }

    #[test]
    fn between_the_hours_the_stream_turns_through_slack_not_through_the_side() {
        let stream = diamond();
        // HW+1 floods east at 0.15 kn (blended), HW+2 ebbs west at 0.45 kn.
        // Midway the vectors sum to 0.15 kn westerly: slack and turning, not
        // swinging through north at speed.
        let turning = stream.at_hour(1.5).unwrap();
        assert!(
            turning
                .set
                .angular_distance(TrueCourse::new(270.0).unwrap())
                < 1e-9
        );
        assert!((turning.drift.knots() - 0.15).abs() < 1e-12);
        // One fifth of the way: weak flood.
        let early = stream.at_hour(1.2).unwrap();
        assert!(early.set.angular_distance(TrueCourse::new(90.0).unwrap()) < 1e-9);
        assert!((early.drift.knots() - 0.03).abs() < 1e-12);
    }

    #[test]
    fn as_a_current_model_the_clock_is_the_reference_ports_high_water() {
        let stream = diamond();
        let anywhere = Position::from_degrees(50.0, -1.0).unwrap();
        let hw = high_water();
        let three_after = hw.saturating_add(Duration::from_secs(3 * 3600));
        let two_before = hw.saturating_sub(Duration::from_secs(2 * 3600));
        assert_eq!(
            stream.current_at(anywhere, three_after).unwrap(),
            stream.at_hour(3.0).unwrap()
        );
        assert_eq!(
            stream.current_at(anywhere, two_before).unwrap(),
            stream.at_hour(-2.0).unwrap()
        );
        assert_eq!(stream.high_water(), hw);
        let seven_after = hw.saturating_add(Duration::from_secs(7 * 3600));
        assert!(matches!(
            stream.current_at(anywhere, seven_after),
            Err(KernelError::OutsideValidity {
                data: "tidal diamond"
            })
        ));
        assert!(stream.at_hour(f64::NAN).is_err());
    }

    #[test]
    fn the_range_factor_places_the_day_between_neaps_and_springs() {
        let metres = |value: f64| Distance::from_metres(value).unwrap();
        let factor = TidalStream::range_factor(metres(3.5), metres(2.0), metres(5.0)).unwrap();
        assert!((factor - 0.5).abs() < 1e-12);
        assert_eq!(
            TidalStream::range_factor(metres(2.0), metres(2.0), metres(5.0)).unwrap(),
            0.0
        );
        assert_eq!(
            TidalStream::range_factor(metres(5.0), metres(2.0), metres(5.0)).unwrap(),
            1.0
        );
        // Slightly beyond springs is allowed; neap range above spring range is
        // not.
        assert!(TidalStream::range_factor(metres(6.0), metres(2.0), metres(5.0)).is_ok());
        assert!(TidalStream::range_factor(metres(3.0), metres(5.0), metres(2.0)).is_err());
        assert!(TidalStream::range_factor(metres(1.0), metres(2.0), metres(5.0)).is_err());
        assert!(TidalStream::new(diamond().rows, high_water(), 2.5).is_err());
    }
}
