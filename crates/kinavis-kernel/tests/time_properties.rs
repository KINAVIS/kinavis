//! [`Instant`] properties over its full range.
//!
//! Unit tests pin known dates; these check invariants for all inputs, `i64`
//! extremes included: arithmetic never wraps, the calendar is a bijection,
//! order is preserved.

#![allow(clippy::expect_used)]

use core::time::Duration;

use kinavis_kernel::time::{Civil, Instant, Utc};
use proptest::prelude::*;

/// Any instant, range ends included.
fn any_instant() -> impl Strategy<Value = Instant<Utc>> {
    (any::<i64>(), 0_u32..1_000_000_000)
        .prop_map(|(seconds, nanos)| Instant::new(seconds, nanos).expect("nanos in range"))
}

/// Any duration.
fn any_duration() -> impl Strategy<Value = Duration> {
    (any::<u64>(), 0_u32..1_000_000_000).prop_map(|(seconds, nanos)| Duration::new(seconds, nanos))
}

proptest! {
    #[test]
    fn unix_nanos_round_trip(nanos in any::<i64>()) {
        let instant = Instant::<Utc>::from_unix_nanos(nanos);
        prop_assert_eq!(instant.checked_unix_nanos(), Some(nanos));
    }

    #[test]
    fn adding_never_wraps_and_never_goes_backwards(
        instant in any_instant(),
        duration in any_duration(),
    ) {
        let later = instant.saturating_add(duration);
        prop_assert!(later >= instant);
        if let Some(checked) = instant.checked_add(duration) {
            prop_assert_eq!(checked, later);
            prop_assert_eq!(checked.checked_duration_since(instant), Some(duration));
            prop_assert_eq!(checked.checked_sub(duration), Some(instant));
        }
    }

    #[test]
    fn subtracting_never_wraps_and_never_goes_forwards(
        instant in any_instant(),
        duration in any_duration(),
    ) {
        let earlier = instant.saturating_sub(duration);
        prop_assert!(earlier <= instant);
        if let Some(checked) = instant.checked_sub(duration) {
            prop_assert_eq!(checked, earlier);
            prop_assert_eq!(instant.checked_duration_since(checked), Some(duration));
        }
    }

    #[test]
    fn exactly_one_of_two_instants_is_later(a in any_instant(), b in any_instant()) {
        let forward = a.checked_duration_since(b);
        let backward = b.checked_duration_since(a);
        match (forward, backward) {
            (Some(zero), Some(also_zero)) => {
                prop_assert_eq!(a, b);
                prop_assert_eq!(zero, Duration::ZERO);
                prop_assert_eq!(also_zero, Duration::ZERO);
            }
            (Some(_), None) => prop_assert!(a > b),
            (None, Some(_)) => prop_assert!(a < b),
            (None, None) => prop_assert!(false, "neither order worked"),
        }
    }

    #[test]
    fn the_calendar_is_a_bijection_and_keeps_order(
        // About ±100 000 years around the epoch; the full `i32` year range is
        // unnecessary.
        a in -4_000_000_000_000_i64..4_000_000_000_000,
        b in -4_000_000_000_000_i64..4_000_000_000_000,
        nanos in 0_u32..1_000_000_000,
    ) {
        let first = Instant::<Utc>::new(a, nanos).expect("nanos in range");
        let second = Instant::<Utc>::new(b, nanos).expect("nanos in range");
        let civil = first.civil();
        prop_assert_eq!(Instant::<Utc>::from_civil(civil), Ok(first));
        prop_assert!((1..=12).contains(&civil.month));
        prop_assert!((1..=31).contains(&civil.day));
        prop_assert!(civil.hour < 24 && civil.minute < 60 && civil.second < 60);
        prop_assert_eq!(first.cmp(&second), civil.cmp(&second.civil()));
    }

    #[test]
    fn every_valid_calendar_reading_names_an_instant(
        year in -100_000_i32..100_000,
        month in 1_u8..=12,
        day in 1_u8..=31,
        hour in 0_u8..24,
        minute in 0_u8..60,
        second in 0_u8..60,
    ) {
        let civil = Civil { year, month, day, hour, minute, second, nanos: 0 };
        match Instant::<Utc>::from_civil(civil) {
            Ok(instant) => prop_assert_eq!(instant.civil(), civil),
            // The only valid rejection: a day the month lacks.
            Err(_) => prop_assert!(day > 28),
        }
    }
}
