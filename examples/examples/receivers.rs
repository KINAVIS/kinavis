//! What real receivers send, and what KINAVIS makes of it.
//!
//! One sentence from each of seven receivers, from the gpsd regression logs:
//! a chartplotter, an RTK receiver writing past the standard's 82 bytes, an
//! AIS transponder that drops a field, a receiver that writes 999.9 for an
//! unknown variation, a cold start, a receiver without a fix and a line that
//! lost bytes in transit.
//!
//! ```text
//! cargo run -p kinavis-examples --example receivers
//! ```

use kinavis::GnssFix;
use kinavis_nmea0183::{parse, Sentence};

const LOG: &str = include_str!("../data/receivers.nmea");

fn main() {
    let mut receiver = "";
    for line in LOG.lines() {
        if let Some(name) = line.strip_prefix("## ") {
            receiver = name;
            continue;
        }
        if line.starts_with('#') || line.is_empty() {
            continue;
        }
        println!("{receiver}");
        println!("  {line}");
        println!("  -> {}\n", read(line));
    }
}

/// One line, as the navigation system would see it.
fn read(line: &str) -> String {
    match parse(line.as_bytes()) {
        Err(error) => format!("refused: {error}"),
        Ok(Sentence::Rmc(rmc)) => match GnssFix::try_from(rmc) {
            Ok(fix) => {
                let motion = match (fix.speed_over_ground(), fix.course_over_ground()) {
                    (Some(speed), Some(course)) => format!(", {speed} over the ground on {course}"),
                    _ => String::new(),
                };
                format!("fix {:.4} at {:.0}{motion}", fix.position(), fix.taken_at())
            }
            Err(error) => format!("no fix: {error}"),
        },
        // GGA has no date, so it gives a position and its quality, not a fix.
        Ok(Sentence::Gga(gga)) => match gga.position {
            Some(position) if gga.fix_type.is_position_fix() => {
                let hdop = gga
                    .hdop
                    .map_or_else(String::new, |hdop| format!(", HDOP {}", hdop.value()));
                format!("position {position:.7} ({:?}{hdop})", gga.fix_type)
            }
            _ => {
                let hdop = gga.hdop.map_or_else(
                    || "not available".to_owned(),
                    |hdop| hdop.value().to_string(),
                );
                format!("no fix yet: fix type {:?}, HDOP {hdop}", gga.fix_type)
            }
        },
        Ok(other) => format!("{other:?}"),
    }
}
