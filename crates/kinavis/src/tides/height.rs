//! Tidal height between turning points; secondary port corrections.

use core::fmt;
use core::time::Duration;

use crate::environment::TideModel;
use crate::error::{ensure_range, KernelError, NavigationError, Result};
use crate::position::Position;
use crate::time::{Instant, Utc};
use crate::units::Distance;

/// High or low water: time and height above chart datum.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct TideEvent {
    at: Instant<Utc>,
    height: Distance,
}

impl TideEvent {
    /// Tidal turning point, as tabulated.
    #[must_use]
    pub const fn new(at: Instant<Utc>, height: Distance) -> Self {
        Self { at, height }
    }

    /// Time.
    #[must_use]
    pub const fn at(&self) -> Instant<Utc> {
        self.at
    }

    /// Height above chart datum.
    #[must_use]
    pub const fn height(&self) -> Distance {
        self.height
    }
}

impl fmt::Display for TideEvent {
    /// Formats as `4.6 m at 2026-09-15T10:20:00.000 UTC` (metres, not NM).
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let precision = f.precision().unwrap_or(1);
        write!(f, "{:.precision$} m at {}", self.height.metres(), self.at)
    }
}

/// Secondary port differences.
///
/// Admiralty tables give, per secondary port, time and height differences of
/// its high and low waters relative to the standard port. This is the simple
/// form — one time and one height difference each for HW and LW — used where
/// the differences vary little. Where the tables give two values to
/// interpolate, the caller selects the one for the day.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SecondaryPort {
    high_water_minutes: i32,
    high_water_height: Distance,
    low_water_minutes: i32,
    low_water_height: Distance,
}

impl SecondaryPort {
    /// Zero differences: the standard port itself.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            high_water_minutes: 0,
            high_water_height: Distance::ZERO,
            low_water_minutes: 0,
            low_water_height: Distance::ZERO,
        }
    }

    /// HW differences: `minutes` later (negative earlier), `height` higher
    /// (negative lower).
    #[must_use]
    pub const fn high_water(mut self, minutes: i32, height: Distance) -> Self {
        self.high_water_minutes = minutes;
        self.high_water_height = height;
        self
    }

    /// LW differences.
    #[must_use]
    pub const fn low_water(mut self, minutes: i32, height: Distance) -> Self {
        self.low_water_minutes = minutes;
        self.low_water_height = height;
        self
    }

    /// Standard port HW at the secondary port.
    #[must_use]
    pub fn adjust_high_water(&self, standard: TideEvent) -> TideEvent {
        adjust(standard, self.high_water_minutes, self.high_water_height)
    }

    /// Standard port LW at the secondary port.
    #[must_use]
    pub fn adjust_low_water(&self, standard: TideEvent) -> TideEvent {
        adjust(standard, self.low_water_minutes, self.low_water_height)
    }

    /// A standard-port cycle transferred to the secondary port.
    ///
    /// # Errors
    ///
    /// [`KernelError::TimeReversed`] if the differences invert the cycle (not
    /// possible with plausible tables).
    pub fn adjust(&self, cycle: TidalCycle) -> Result<TidalCycle> {
        let (from, to) = if cycle.is_rising() {
            (
                self.adjust_low_water(cycle.from),
                self.adjust_high_water(cycle.to),
            )
        } else {
            (
                self.adjust_high_water(cycle.from),
                self.adjust_low_water(cycle.to),
            )
        };
        TidalCycle::new(from, to)
    }
}

fn adjust(event: TideEvent, minutes: i32, height: Distance) -> TideEvent {
    let shift = Duration::from_secs(60 * u64::from(minutes.unsigned_abs()));
    let at = if minutes < 0 {
        event.at.saturating_sub(shift)
    } else {
        event.at.saturating_add(shift)
    };
    TideEvent::new(at, event.height + height)
}

/// Cumulative twelfths at the end of each sixth of the cycle.
const CUMULATIVE_TWELFTHS: [f64; 7] = [0.0, 1.0, 3.0, 6.0, 9.0, 11.0, 12.0];

/// Twelfths per sixth of the cycle.
const TWELFTHS_PER_SIXTH: [f64; 6] = [1.0, 2.0, 3.0, 3.0, 2.0, 1.0];

/// One rise or fall: LW to the next HW, or HW to the next LW.
///
/// Heights between follow the rule of twelfths, scaled to the cycle duration.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(try_from = "StoredTidalCycle", into = "StoredTidalCycle")
)]
pub struct TidalCycle {
    from: TideEvent,
    to: TideEvent,
}

impl TidalCycle {
    /// Cycle between two turning points.
    ///
    /// # Errors
    ///
    /// [`KernelError::TimeReversed`] unless `to` is after `from`;
    /// [`KernelError::OutOfRange`] for a cycle over one day;
    /// [`KernelError::Indeterminate`] for equal heights (no range).
    pub fn new(from: TideEvent, to: TideEvent) -> Result<Self> {
        let duration = to.at.duration_since(from.at)?;
        if duration.is_zero() {
            return Err(NavigationError::Kernel(KernelError::TimeReversed {
                by: Duration::ZERO,
            }));
        }
        ensure_range(
            "tidal cycle duration",
            duration.as_secs_f64() / 3600.0,
            0.0,
            24.0,
        )?;
        if to.height == from.height {
            return Err(NavigationError::Kernel(KernelError::Indeterminate {
                quantity: "tidal range",
            }));
        }
        Ok(Self { from, to })
    }

    /// Starting turning point.
    #[must_use]
    pub const fn from(&self) -> TideEvent {
        self.from
    }

    /// Ending turning point.
    #[must_use]
    pub const fn to(&self) -> TideEvent {
        self.to
    }

    /// Whether rising.
    #[must_use]
    pub fn is_rising(&self) -> bool {
        self.to.height > self.from.height
    }

    /// Range, non-negative.
    #[must_use]
    pub fn range(&self) -> Distance {
        (self.to.height - self.from.height).abs()
    }

    /// Duration.
    #[must_use]
    pub fn duration(&self) -> Duration {
        self.to
            .at
            .checked_duration_since(self.from.at)
            .unwrap_or_default()
    }

    /// Height at `when` by the rule of twelfths.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutsideValidity`] outside the cycle.
    pub fn height_at(&self, when: Instant<Utc>) -> Result<Distance> {
        if when < self.from.at || when > self.to.at {
            return Err(NavigationError::Kernel(KernelError::OutsideValidity {
                data: "tidal cycle",
            }));
        }
        let elapsed = when
            .checked_duration_since(self.from.at)
            .unwrap_or_default();
        let fraction = elapsed.as_secs_f64() / self.duration().as_secs_f64();
        let twelfths = twelfths_at(fraction);
        let signed_range = (self.to.height - self.from.height).metres();
        Ok(Distance::from_metres(
            self.from.height.metres() + signed_range * twelfths / 12.0,
        )?)
    }

    /// First time the tide reaches `height` (inverse rule of twelfths).
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] if `height` is not between the turning
    /// points.
    pub fn time_of_height(&self, height: Distance) -> Result<Instant<Utc>> {
        let (low, high) = if self.is_rising() {
            (self.from.height, self.to.height)
        } else {
            (self.to.height, self.from.height)
        };
        ensure_range("tide height", height.metres(), low.metres(), high.metres())?;
        let progress =
            (height - self.from.height).metres() / (self.to.height - self.from.height).metres();
        let fraction = fraction_at(progress * 12.0);
        // Finite and non-negative by construction; the error branch is
        // unreachable.
        let elapsed = Duration::try_from_secs_f64(fraction * self.duration().as_secs_f64())
            .map_err(|_| {
                NavigationError::Kernel(KernelError::Indeterminate {
                    quantity: "time of height",
                })
            })?;
        Ok(self.from.at.saturating_add(elapsed))
    }
}

/// Serialised form: two turning points. Deserialisation goes through
/// [`TidalCycle::new`], rejecting reversed or zero-range cycles.
#[cfg(feature = "serde")]
#[derive(serde::Serialize, serde::Deserialize)]
struct StoredTidalCycle {
    from: TideEvent,
    to: TideEvent,
}

#[cfg(feature = "serde")]
impl TryFrom<StoredTidalCycle> for TidalCycle {
    type Error = NavigationError;

    fn try_from(stored: StoredTidalCycle) -> Result<Self> {
        Self::new(stored.from, stored.to)
    }
}

#[cfg(feature = "serde")]
impl From<TidalCycle> for StoredTidalCycle {
    fn from(cycle: TidalCycle) -> Self {
        Self {
            from: cycle.from,
            to: cycle.to,
        }
    }
}

impl TideModel for TidalCycle {
    /// Position ignored: a cycle is valid for the place its turning points were
    /// predicted for.
    fn height_of_tide(
        &self,
        _: Position,
        when: Instant<Utc>,
    ) -> kinavis_kernel::error::Result<Distance> {
        self.height_at(when).map_err(NavigationError::into_kernel)
    }
}

/// Twelfths of the range elapsed at `fraction` of the cycle.
fn twelfths_at(fraction: f64) -> f64 {
    let sixths = (fraction.clamp(0.0, 1.0) * 6.0).min(6.0 - f64::EPSILON * 8.0);
    // `sixths` in `[0, 6)`: safe to truncate.
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let index = sixths as usize;
    let within = sixths - crate::math::count_to_f64(index);
    let start = CUMULATIVE_TWELFTHS.get(index).copied().unwrap_or(12.0);
    let rate = TWELFTHS_PER_SIXTH.get(index).copied().unwrap_or(0.0);
    start + rate * within
}

/// Inverse of [`twelfths_at`]: cycle fraction at which `twelfths` have elapsed.
fn fraction_at(twelfths: f64) -> f64 {
    let twelfths = twelfths.clamp(0.0, 12.0);
    let index = CUMULATIVE_TWELFTHS
        .iter()
        .skip(1)
        .position(|&boundary| twelfths <= boundary)
        .unwrap_or(5);
    let start = CUMULATIVE_TWELFTHS.get(index).copied().unwrap_or(12.0);
    let rate = TWELFTHS_PER_SIXTH.get(index).copied().unwrap_or(1.0);
    (crate::math::count_to_f64(index) + (twelfths - start) / rate) / 6.0
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use crate::time::Civil;
    use alloc::format;

    fn at(hour: u8, minute: u8) -> Instant<Utc> {
        Instant::from_civil(Civil {
            hour,
            minute,
            ..Civil::date(2026, 9, 15)
        })
        .unwrap()
    }

    fn metres(value: f64) -> Distance {
        Distance::from_metres(value).unwrap()
    }

    /// LW 0400 at 1 m, HW 1000 at 5 m: 6 h, 4 m range.
    fn textbook_rise() -> TidalCycle {
        TidalCycle::new(
            TideEvent::new(at(4, 0), metres(1.0)),
            TideEvent::new(at(10, 0), metres(5.0)),
        )
        .unwrap()
    }

    #[test]
    fn the_rule_of_twelfths_hour_by_hour() {
        let rise = textbook_rise();
        // 4 m range: one twelfth is 1/3 m.
        let expected = [1.0, 4.0 / 3.0, 2.0, 3.0, 4.0, 14.0 / 3.0, 5.0];
        for (hour, want) in (4..=10).zip(expected) {
            let got = rise.height_at(at(hour, 0)).unwrap().metres();
            assert!((got - want).abs() < 1e-9, "at {hour}00: {got} vs {want}");
        }
        // Mid third hour: 3 + 1.5 twelfths.
        let got = rise.height_at(at(6, 30)).unwrap().metres();
        assert!((got - (1.0 + 4.0 * 4.5 / 12.0)).abs() < 1e-9);
        assert!(rise.is_rising());
        assert!((rise.range().metres() - 4.0).abs() < 1e-9);
        assert_eq!(rise.duration(), Duration::from_secs(6 * 3600));
    }

    #[test]
    fn a_falling_tide_is_the_rise_read_backwards() {
        let fall = TidalCycle::new(
            TideEvent::new(at(10, 0), metres(5.0)),
            TideEvent::new(at(16, 0), metres(1.0)),
        )
        .unwrap();
        assert!(!fall.is_rising());
        assert!((fall.height_at(at(11, 0)).unwrap().metres() - 14.0 / 3.0).abs() < 1e-9);
        assert!((fall.height_at(at(13, 0)).unwrap().metres() - 3.0).abs() < 1e-9);
        assert_eq!(fall.time_of_height(metres(3.0)).unwrap(), at(13, 0));
    }

    #[test]
    fn the_inverse_finds_the_moment_of_a_height() {
        let rise = textbook_rise();
        assert_eq!(rise.time_of_height(metres(1.0)).unwrap(), at(4, 0));
        assert_eq!(rise.time_of_height(metres(2.0)).unwrap(), at(6, 0));
        assert_eq!(rise.time_of_height(metres(3.0)).unwrap(), at(7, 0));
        assert_eq!(rise.time_of_height(metres(5.0)).unwrap(), at(10, 0));
        // Round trip at arbitrary points.
        for minute in [7, 61, 143, 200, 299, 359] {
            let when = at(4, 0).saturating_add(Duration::from_secs(60 * minute));
            let height = rise.height_at(when).unwrap();
            let back = rise.time_of_height(height).unwrap();
            let error = back
                .checked_duration_since(when)
                .or_else(|| when.checked_duration_since(back))
                .unwrap();
            assert!(error < Duration::from_millis(1), "{minute}: {error:?}");
        }
    }

    #[test]
    fn outside_the_cycle_there_is_no_answer() {
        let rise = textbook_rise();
        assert!(matches!(
            rise.height_at(at(3, 59)),
            Err(NavigationError::Kernel(KernelError::OutsideValidity {
                data: "tidal cycle"
            }))
        ));
        assert!(rise.height_at(at(10, 1)).is_err());
        assert!(matches!(
            rise.time_of_height(metres(5.5)),
            Err(NavigationError::Kernel(KernelError::OutOfRange { .. }))
        ));
    }

    #[test]
    fn an_impossible_cycle_is_refused() {
        let event = TideEvent::new(at(4, 0), metres(1.0));
        assert!(matches!(
            TidalCycle::new(event, TideEvent::new(at(3, 0), metres(5.0))),
            Err(NavigationError::Kernel(KernelError::TimeReversed { .. }))
        ));
        assert!(matches!(
            TidalCycle::new(event, TideEvent::new(at(4, 0), metres(5.0))),
            Err(NavigationError::Kernel(KernelError::TimeReversed { .. }))
        ));
        assert!(matches!(
            TidalCycle::new(event, TideEvent::new(at(10, 0), metres(1.0))),
            Err(NavigationError::Kernel(KernelError::Indeterminate { .. }))
        ));
        let next_week = at(4, 0).saturating_add(Duration::from_secs(7 * 86_400));
        assert!(matches!(
            TidalCycle::new(event, TideEvent::new(next_week, metres(5.0))),
            Err(NavigationError::Kernel(KernelError::OutOfRange { .. }))
        ));
    }

    #[test]
    fn a_secondary_port_shifts_the_turning_points() {
        let port = SecondaryPort::new()
            .high_water(20, metres(-0.4))
            .low_water(-35, metres(0.1));
        let high = port.adjust_high_water(TideEvent::new(at(10, 0), metres(5.0)));
        assert_eq!(high, TideEvent::new(at(10, 20), metres(4.6)));
        let low = port.adjust_low_water(TideEvent::new(at(4, 0), metres(1.0)));
        assert_eq!(low, TideEvent::new(at(3, 25), metres(1.1)));

        let carried = port.adjust(textbook_rise()).unwrap();
        assert_eq!(carried.from(), low);
        assert_eq!(carried.to(), high);
        assert_eq!(
            SecondaryPort::default().adjust(textbook_rise()).unwrap(),
            textbook_rise()
        );
        assert_eq!(format!("{high}"), "4.6 m at 2026-09-15T10:20:00.000 UTC");
    }
}
