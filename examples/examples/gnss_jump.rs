//! A position that jumps forty miles in a second is refused.
//!
//! The fixes are a real track: a B&G Zeus² 9 chartplotter on a yacht off
//! Stavoren, NL, 20 August 2018, one fix a second (gpsd regression log
//! `bundg_zeus_9.log`). One fix halfway through is moved forty miles north,
//! as a receiver fault or a spoofed signal would. The intake accepts the real
//! track and refuses the jump; the position the system holds never jumps.
//!
//! ```text
//! cargo run -p kinavis-examples --example gnss_jump
//! ```

use core::time::Duration;

use kinavis::gnss_intake::{GnssIntake, IntakeConfig};
use kinavis::{GnssFix, NavigationError, NavigationEvent, Position, RejectionReason, Speed};
use kinavis_nmea0183::{parse, Sentence};

const LOG: &str = include_str!("../data/bundg_zeus_9.nmea");

/// Distance the injected fix is moved north, degrees of latitude.
const JUMP_DEGREES: f64 = 40.0 / 60.0;

fn main() -> Result<(), NavigationError> {
    let mut fixes: Vec<GnssFix> = LOG
        .lines()
        .filter_map(|line| match parse(line.as_bytes()) {
            Ok(Sentence::Rmc(rmc)) => GnssFix::try_from(rmc).ok(),
            _ => None,
        })
        .collect();
    let injected = fixes.len() / 2;
    if let Some(fix) = fixes.get_mut(injected) {
        let here = fix.position();
        let there = Position::from_degrees(
            here.latitude().degrees() + JUMP_DEGREES,
            here.longitude().degrees(),
        )?;
        *fix = GnssFix::builder(fix.taken_at(), there).build();
    }

    // A sailing yacht: nothing faster than 40 kn, nothing older than 10 s.
    let mut intake = GnssIntake::new(IntakeConfig {
        max_age: Duration::from_secs(10),
        max_speed: Speed::from_knots(40.0)?,
    });

    let (mut accepted, mut refused) = (0_u32, 0_u32);
    let last = fixes.len().saturating_sub(1);
    let mut skipped = false;
    for (index, fix) in fixes.iter().enumerate() {
        let outcome = intake.accept(*fix);
        if outcome.accepted() {
            accepted += 1;
        } else {
            refused += 1;
        }
        let shown = index < 3 || index.abs_diff(injected) <= 2 || index == last;
        if !shown {
            skipped = true;
            continue;
        }
        if skipped {
            println!("  ...");
            skipped = false;
        }
        let verdict = if outcome.accepted() {
            "accepted"
        } else {
            "REFUSED "
        };
        let mut notes: Vec<String> = outcome.events().iter().map(describe).collect();
        if index == injected {
            notes.push("injected".to_owned());
        }
        let line = format!(
            "{:.0}  {verdict}  {:.3}  {}",
            fix.taken_at(),
            fix.position(),
            notes.join("; ")
        );
        println!("{}", line.trim_end());
    }

    println!("\n{accepted} fixes accepted, {refused} refused.");
    if let Some(position) = intake.last_fix().map(GnssFix::position) {
        println!("Position held: {position:.3}");
    }
    Ok(())
}

fn describe(event: &NavigationEvent) -> String {
    match event {
        NavigationEvent::FixAcquired { .. } => "fix acquired".to_owned(),
        NavigationEvent::FixLost { .. } => "fix lost".to_owned(),
        NavigationEvent::ObservationRejected {
            reason: RejectionReason::ImplausibleJump { implied_speed },
            ..
        } => format!("implausible jump: {implied_speed:.0} implied"),
        NavigationEvent::ObservationRejected { reason, .. } => format!("{reason:?}"),
        other => format!("{other:?}"),
    }
}
