//! Position from GNSS end to end: a recorded NMEA log in, snapshots and events
//! out.
//!
//! The log contains typical faults — a garbage line, a bad checksum, an
//! unrequested sentence, a 60 NM jump, an unvouched fix, a 30 s silence —
//! around a vessel at 10 kn due north. The test checks the event sequence and
//! the snapshots between events. Allocation and panic freedom are checked by
//! the compiler and `ci/panic-free.py`; this test checks outputs.

#![allow(
    clippy::unwrap_used,
    clippy::indexing_slicing,
    clippy::panic,
    clippy::float_cmp
)]

use core::time::Duration;

use kinavis::gnss_intake::{GnssIntake, IntakeConfig};
use kinavis::{
    Civil, GnssFix, Instant, NavigationEvent, PositionSource, RejectionReason, Speed, Utc,
};
use kinavis_nmea0183::{parse, NmeaError, Sentence, TranslationError};

const LOG: &str = include_str!("fixtures/gnss_log.nmea");

/// Per-line tally.
#[derive(Debug, Default)]
struct Tally {
    unreadable: usize,
    unsupported: usize,
    untranslatable: usize,
    accepted: usize,
    rejected: usize,
    events: Vec<NavigationEvent>,
}

fn at(hour: u8, minute: u8, second: u8) -> Instant<Utc> {
    Instant::from_civil(Civil {
        year: 2026,
        month: 9,
        day: 11,
        hour,
        minute,
        second,
        nanos: 0,
    })
    .unwrap()
}

#[test]
fn a_recorded_log_produces_the_expected_events_and_snapshots() {
    let mut intake = GnssIntake::new(IntakeConfig {
        max_age: Duration::from_secs(10),
        max_speed: Speed::from_knots(40.0).unwrap(),
    });
    let mut tally = Tally::default();

    for line in LOG.lines() {
        let sentence = match parse(line.as_bytes()) {
            Ok(sentence) => sentence,
            Err(NmeaError::NoStart | NmeaError::BadChecksum { .. }) => {
                tally.unreadable += 1;
                continue;
            }
            Err(other) => panic!("unexpected parse error {other} on {line}"),
        };
        let fix = match sentence {
            Sentence::Rmc(rmc) => match GnssFix::try_from(rmc) {
                Ok(fix) => fix,
                Err(TranslationError::NoPosition) => {
                    // Receiver "no fix" sentence: still reported to the intake.
                    // Date and time are present; only the position is missing.
                    tally.untranslatable += 1;
                    continue;
                }
                Err(other) => panic!("unexpected translation error {other} on {line}"),
            },
            Sentence::Unsupported { .. } => {
                tally.unsupported += 1;
                continue;
            }
            other => panic!("unexpected sentence {other:?}"),
        };

        let outcome = intake.accept(fix);
        if outcome.accepted() {
            tally.accepted += 1;
        } else {
            tally.rejected += 1;
        }
        assert!(!outcome.events().overflowed());
        tally.events.extend(outcome.events().iter().copied());

        // Every accepted fix becomes the snapshot's current GNSS position.
        if outcome.accepted() {
            let snapshot = intake.snapshot_at(fix.taken_at());
            assert_eq!(*snapshot.position().unwrap().value(), fix.position());
            assert_eq!(snapshot.source(), Some(PositionSource::Gnss));
            assert!(!snapshot.is_stale());
            let track = snapshot.ground_track().unwrap();
            assert_eq!(track.speed_over_ground.knots(), 10.0);
            assert_eq!(track.course_over_ground.degrees(), 0.0);
        }
    }

    assert_eq!(tally.unreadable, 2, "the garbage line and the bad checksum");
    assert_eq!(tally.unsupported, 1, "the GSV");
    assert_eq!(tally.untranslatable, 1, "the RMC with no position");
    assert_eq!(tally.accepted, 7);
    assert_eq!(tally.rejected, 1, "the sixty-mile jump");

    let expected = [
        NavigationEvent::FixAcquired {
            source: PositionSource::Gnss,
            at: at(12, 0, 0),
        },
        NavigationEvent::ObservationRejected {
            reason: RejectionReason::ImplausibleJump {
                // Matched by pattern below; the exact value depends on the
                // sphere.
                implied_speed: Speed::from_knots(0.0).unwrap(),
            },
            at: at(12, 0, 3),
        },
        NavigationEvent::FixLost {
            source: PositionSource::Gnss,
            at: at(12, 0, 14),
            last_good: at(12, 0, 4),
        },
        NavigationEvent::FixAcquired {
            source: PositionSource::Gnss,
            at: at(12, 0, 35),
        },
    ];
    assert_eq!(tally.events.len(), expected.len(), "{:?}", tally.events);
    for (index, (actual, wanted)) in tally.events.iter().zip(&expected).enumerate() {
        match (actual, wanted) {
            (
                NavigationEvent::ObservationRejected {
                    reason: RejectionReason::ImplausibleJump { implied_speed },
                    at: actual_at,
                },
                NavigationEvent::ObservationRejected { at: wanted_at, .. },
            ) => {
                assert_eq!(actual_at, wanted_at, "event {index}");
                // 60 NM in one second.
                assert!(implied_speed.knots() > 200_000.0, "event {index}");
            }
            _ => assert_eq!(actual, wanted, "event {index}"),
        }
    }

    // After the log ends the last position is still provided, marked stale once
    // older than 10 s.
    assert!(!intake.snapshot_at(at(12, 0, 46)).is_stale());
    let stale = intake.snapshot_at(at(12, 0, 47));
    assert_eq!(
        *stale.position().unwrap().value(),
        intake.last_fix().unwrap().position()
    );
    assert!(stale.is_stale());
}

#[test]
fn the_silence_is_reported_by_the_clock_when_no_fix_arrives() {
    let mut intake = GnssIntake::new(IntakeConfig {
        max_age: Duration::from_secs(10),
        max_speed: Speed::from_knots(40.0).unwrap(),
    });
    let first = LOG.lines().next().unwrap();
    let Sentence::Rmc(rmc) = parse(first.as_bytes()).unwrap() else {
        panic!("not RMC")
    };
    let _ = intake.accept(GnssFix::try_from(rmc).unwrap());

    assert!(intake.check_at(at(12, 0, 10)).is_empty());
    let events = intake.check_at(at(12, 0, 11));
    assert_eq!(
        events.as_slice(),
        &[NavigationEvent::FixLost {
            source: PositionSource::Gnss,
            at: at(12, 0, 10),
            last_good: at(12, 0, 0),
        }]
    );
}
