//! Speed from the wheel ticks of `ESF-MEAS`, for an inertial filter's
//! wheel-speed update.
//!
//! A dead-reckoning receiver passes on the odometer it is wired to as tick
//! counts: 23 bits, accumulated and wrapping, counted up whichever way the
//! wheel turns, with a direction bit beside them, and stamped with the
//! sensor's millisecond time tag. [`Odometer`] differences two readings into
//! a signed [`Speed`] over the interval between them; a gap longer than the
//! caller allows, or a time tag that does not advance, starts it afresh.

use core::time::Duration;

use kinavis_kernel::error::ensure_range;
use kinavis_kernel::{Distance, KernelError, Speed};

use crate::esf::{EsfMeas, Measurement, Wheel};

/// Ticks wrap at 2²³.
const TICK_MASK: u32 = 0x007F_FFFF;

/// What the speed is read from.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SpeedSource {
    /// The ticks of one wheel.
    Wheel(Wheel),
    /// The mean of the two rear wheels' ticks: the speed at the middle of the
    /// rear axle, the point a tractor turns about. Both must be in the same
    /// message.
    RearAxle,
    /// The ticks of a single speed sensor (data type 10): a gearbox or
    /// driveshaft pickup.
    SpeedTicks,
    /// A speed the vehicle sends as such (data type 11), passed through; the
    /// tick size is not used.
    Speed,
}

/// Speed over the interval up to a measurement's time tag.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WheelSpeed {
    /// Sensor time tag of the later reading, ms.
    pub time_tag: u32,
    /// Interval the speed is averaged over; zero for a speed sent as such.
    pub interval: Duration,
    /// Speed, negative backwards.
    pub speed: Speed,
}

/// Tick counts of the last reading.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Reading {
    time_tag: u32,
    /// One count, or two for the rear axle: left, right.
    ticks: [u32; 2],
}

/// Differences successive `ESF-MEAS` tick counts into speed.
///
/// ```rust
/// use core::time::Duration;
/// use kinavis_kernel::Distance;
/// use kinavis_ubx::{Odometer, SpeedSource};
///
/// // A 1.9 m rear tyre and 100 ticks a turn: 19 mm a tick.
/// let odometer = Odometer::new(
///     SpeedSource::RearAxle,
///     Distance::from_metres(0.019)?,
///     Duration::from_millis(500),
/// )?;
/// # let _ = odometer;
/// # Ok::<(), kinavis_kernel::KernelError>(())
/// ```
///
/// Each [`Odometer::push`] that yields a [`WheelSpeed`] is one measurement
/// for `kinavis-ins`'s `InsFilter::update_wheel_speed`. The ticks are those
/// of the wheels, not of the IMU: with the IMU over the rear axle and
/// [`SpeedSource::RearAxle`], the two are the same point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Odometer {
    source: SpeedSource,
    metres_per_tick: f64,
    max_interval_ms: u32,
    last: Option<Reading>,
}

impl Odometer {
    /// Odometer reading `source`, `per_tick` of travel a tick, refusing to
    /// difference readings more than `max_interval` apart.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] for a tick outside 0.01 mm…10 m or a
    /// longest interval outside 1 ms…60 s.
    pub fn new(
        source: SpeedSource,
        per_tick: Distance,
        max_interval: Duration,
    ) -> Result<Self, KernelError> {
        let metres_per_tick = per_tick.metres();
        ensure_range("distance a tick, metres", metres_per_tick, 1e-5, 10.0)?;
        let millis = max_interval.as_secs_f64() * 1e3;
        ensure_range("longest interval between ticks, ms", millis, 1.0, 60_000.0)?;
        Ok(Self {
            source,
            metres_per_tick,
            // Within 1…60 000 after the check above.
            max_interval_ms: u32::try_from(max_interval.as_millis()).unwrap_or(60_000),
            last: None,
        })
    }

    /// Source the speed is read from.
    #[must_use]
    pub const fn source(&self) -> SpeedSource {
        self.source
    }

    /// Forgets the last reading: the next one starts afresh.
    pub fn reset(&mut self) {
        self.last = None;
    }

    /// Takes a measurement message; the speed since the last reading, when
    /// this one has the source's ticks and follows that within the longest
    /// interval.
    ///
    /// A message without the source's data changes nothing. One with it but
    /// too long after the last, or with a time tag that does not advance,
    /// becomes the new start and yields nothing.
    pub fn push(&mut self, message: &EsfMeas<'_>) -> Option<WheelSpeed> {
        let time_tag = message.time_tag;
        let (ticks, backward) = match self.source {
            SpeedSource::Speed => {
                return message
                    .measurements()
                    .find_map(|measurement| match measurement {
                        Measurement::Speed(speed) => Some(WheelSpeed {
                            time_tag,
                            interval: Duration::ZERO,
                            speed,
                        }),
                        _ => None,
                    });
            }
            SpeedSource::Wheel(wheel) => {
                let (count, backward) = wheel_ticks(message, wheel)?;
                ([count, 0], [backward, false])
            }
            SpeedSource::RearAxle => {
                let (left, left_back) = wheel_ticks(message, Wheel::RearLeft)?;
                let (right, right_back) = wheel_ticks(message, Wheel::RearRight)?;
                ([left, right], [left_back, right_back])
            }
            SpeedSource::SpeedTicks => {
                let (count, backward) = message.measurements().find_map(|m| match m {
                    Measurement::SpeedTicks { ticks, backward } => Some((ticks, backward)),
                    _ => None,
                })?;
                ([count, 0], [backward, false])
            }
        };
        let reading = Reading { time_tag, ticks };
        let last = self.last.replace(reading)?;
        let elapsed = time_tag.wrapping_sub(last.time_tag);
        if elapsed == 0 || elapsed > self.max_interval_ms {
            return None;
        }
        let travel = |index: usize| {
            let now = ticks.get(index).copied().unwrap_or(0);
            let before = last.ticks.get(index).copied().unwrap_or(0);
            let count = f64::from(now.wrapping_sub(before) & TICK_MASK);
            let sign = if backward.get(index).copied().unwrap_or(false) {
                -1.0
            } else {
                1.0
            };
            sign * count * self.metres_per_tick
        };
        let metres = match self.source {
            SpeedSource::RearAxle => f64::midpoint(travel(0), travel(1)),
            _ => travel(0),
        };
        let seconds = f64::from(elapsed) / 1e3;
        Some(WheelSpeed {
            time_tag,
            interval: Duration::from_millis(u64::from(elapsed)),
            speed: Speed::from_metres_per_second(metres / seconds).ok()?,
        })
    }
}

/// Tick count and direction of one wheel in a message.
fn wheel_ticks(message: &EsfMeas<'_>, wheel: Wheel) -> Option<(u32, bool)> {
    message
        .measurements()
        .find_map(|measurement| match measurement {
            Measurement::WheelTicks {
                wheel: which,
                ticks,
                backward,
            } if which == wheel => Some((ticks, backward)),
            _ => None,
        })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// An `ESF-MEAS` payload of up to four words: time tag, then
    /// `(data type, field)` words; the buffer and its length.
    struct Payload([u8; 24], usize);

    impl core::ops::Deref for Payload {
        type Target = [u8];

        fn deref(&self) -> &[u8] {
            self.0.get(..self.1).unwrap_or(&[])
        }
    }

    fn payload(time_tag: u32, words: &[(u8, u32)]) -> Payload {
        let mut bytes = [0u8; 24];
        let mut put = |at: usize, data: &[u8]| {
            for (slot, byte) in bytes.iter_mut().skip(at).zip(data) {
                *slot = *byte;
            }
        };
        put(0, &time_tag.to_le_bytes());
        let flags = u16::try_from(words.len()).unwrap_or(0) << 11;
        put(4, &flags.to_le_bytes());
        for (index, &(data_type, field)) in words.iter().enumerate() {
            let word = (u32::from(data_type) << 24) | (field & 0x00FF_FFFF);
            put(8 + 4 * index, &word.to_le_bytes());
        }
        Payload(bytes, 8 + 4 * words.len())
    }

    fn ticks(count: u32, backward: bool) -> u32 {
        (count & TICK_MASK) | if backward { 0x0080_0000 } else { 0 }
    }

    fn odometer(source: SpeedSource) -> Odometer {
        Odometer::new(
            source,
            Distance::from_metres(0.02).unwrap_or(Distance::ZERO),
            Duration::from_millis(500),
        )
        .unwrap_or_else(|_| unreachable!())
    }

    /// Pushes a payload; the speed in m/s, if any.
    fn push(odometer: &mut Odometer, bytes: &[u8]) -> Option<f64> {
        let message = EsfMeas::decode(bytes).ok()?;
        odometer
            .push(&message)
            .map(|reading| reading.speed.metres_per_second())
    }

    fn close(found: Option<f64>, expected: f64) -> bool {
        found.is_some_and(|found| (found - expected).abs() < 1e-12)
    }

    #[test]
    fn ticks_over_an_interval_are_a_speed() {
        let mut odometer = odometer(SpeedSource::Wheel(Wheel::RearLeft));
        assert_eq!(
            push(&mut odometer, &payload(1000, &[(8, ticks(500, false))])),
            None
        );
        // 13 ticks of 2 cm in 100 ms.
        let speed = push(&mut odometer, &payload(1100, &[(8, ticks(513, false))]));
        assert!(close(speed, 2.6), "{speed:?}");
    }

    #[test]
    fn the_counts_and_the_time_tag_wrap() {
        let mut odometer = odometer(SpeedSource::SpeedTicks);
        let start = u32::MAX - 49;
        assert_eq!(
            push(
                &mut odometer,
                &payload(start, &[(10, ticks(TICK_MASK - 9, false))])
            ),
            None
        );
        // 20 ticks across the 2²³ wrap, in 100 ms across the 2³² wrap.
        let speed = push(&mut odometer, &payload(50, &[(10, ticks(10, false))]));
        assert!(close(speed, 4.0), "{speed:?}");
    }

    #[test]
    fn backwards_is_negative() {
        let mut odometer = odometer(SpeedSource::Wheel(Wheel::FrontRight));
        push(&mut odometer, &payload(0, &[(7, ticks(40, true))]));
        let speed = push(&mut odometer, &payload(200, &[(7, ticks(50, true))]));
        assert!(close(speed, -1.0), "{speed:?}");
    }

    #[test]
    fn the_rear_axle_is_the_mean_of_its_wheels() {
        let mut odometer = odometer(SpeedSource::RearAxle);
        push(
            &mut odometer,
            &payload(0, &[(8, ticks(0, false)), (9, ticks(0, false))]),
        );
        // Turning left: the right wheel runs further.
        let words = [(8, ticks(10, false)), (9, ticks(14, false))];
        assert!(close(push(&mut odometer, &payload(100, &words)), 2.4));
        // Only one rear wheel: nothing, and the start stays.
        assert_eq!(
            push(&mut odometer, &payload(150, &[(8, ticks(15, false))])),
            None
        );
        let words = [(8, ticks(20, false)), (9, ticks(24, false))];
        assert!(close(push(&mut odometer, &payload(200, &words)), 2.0));
    }

    #[test]
    fn a_gap_or_a_stalled_clock_starts_afresh() {
        let mut odometer = odometer(SpeedSource::SpeedTicks);
        push(&mut odometer, &payload(0, &[(10, ticks(0, false))]));
        assert_eq!(
            push(&mut odometer, &payload(600, &[(10, ticks(60, false))])),
            None
        );
        assert_eq!(
            push(&mut odometer, &payload(600, &[(10, ticks(61, false))])),
            None
        );
        assert!(close(
            push(&mut odometer, &payload(700, &[(10, ticks(71, false))])),
            2.0
        ));
        odometer.reset();
        assert_eq!(
            push(&mut odometer, &payload(800, &[(10, ticks(81, false))])),
            None
        );
    }

    #[test]
    fn messages_without_the_source_change_nothing() {
        let mut odometer = odometer(SpeedSource::Wheel(Wheel::RearRight));
        push(&mut odometer, &payload(0, &[(9, ticks(0, false))]));
        // A gyro message between two tick messages.
        assert_eq!(push(&mut odometer, &payload(50, &[(5, 4096)])), None);
        assert!(close(
            push(&mut odometer, &payload(100, &[(9, ticks(5, false))])),
            1.0
        ));
    }

    #[test]
    fn a_speed_sent_as_such_passes_through() {
        let mut odometer = odometer(SpeedSource::Speed);
        // −1.25 m/s in mm/s, 24-bit two's complement.
        let field = u32::from_le_bytes((-1250i32).to_le_bytes()) & 0x00FF_FFFF;
        assert!(close(
            push(&mut odometer, &payload(10, &[(11, field)])),
            -1.25
        ));
    }

    #[test]
    fn a_tick_and_an_interval_out_of_range_are_refused() {
        let tick = |metres| Distance::from_metres(metres).unwrap_or(Distance::ZERO);
        let second = Duration::from_secs(1);
        assert!(Odometer::new(SpeedSource::SpeedTicks, tick(0.0), second).is_err());
        assert!(Odometer::new(SpeedSource::SpeedTicks, tick(11.0), second).is_err());
        assert!(Odometer::new(SpeedSource::SpeedTicks, tick(0.02), Duration::ZERO).is_err());
        assert!(
            Odometer::new(SpeedSource::SpeedTicks, tick(0.02), Duration::from_secs(61)).is_err()
        );
    }
}
