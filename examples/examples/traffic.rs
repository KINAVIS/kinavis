//! AIS traffic off Harlingen, and the closest point of approach of each ship.
//!
//! The sentences are real: an AIS receiver off Harlingen in the Netherlands,
//! 16 April 2014 (Signal K server sample `gofree-merrimac.log`). They are
//! reassembled and decoded; the last report of each ship gives its position,
//! course and speed. One of them, a ship making 21 knots to the north-east, is
//! taken as own ship, and every other ship is assessed against it: CPA, TCPA
//! and the risk against a 2-mile, 30-minute policy.
//!
//! The recording lasts under four minutes and carries no timestamps, so the
//! last reports are taken as simultaneous.
//!
//! ```text
//! cargo run -p kinavis-examples --example traffic
//! ```

use std::collections::BTreeMap;

use core::time::Duration;

use kinavis::sailings::rhumb_line;
use kinavis::{Contact, Distance, Instant, Position, Speed, TrueCourse, Utc, Vessel};
use kinavis_ais::{Assembler, Message, StaticDataPart};
use kinavis_nmea0183::{parse, Sentence};
use kinavis_traffic::{assess, CollisionAssessment, CollisionRisk, CpaPolicy};

const LOG: &str = include_str!("../data/gofree-merrimac-ais.nmea");

/// The ship taken as own ship.
const OWN_SHIP: u32 = 319_014_600;

/// Last known state of one ship.
#[derive(Default)]
struct Ship {
    name: Option<String>,
    position: Option<Position>,
    course: Option<TrueCourse>,
    speed: Option<Speed>,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let (ships, messages) = decode();
    let own = ships
        .get(&OWN_SHIP)
        .ok_or("own ship not in the recording")?;
    let (Some(own_position), Some(own_motion)) = (own.position, motion(own)) else {
        return Err("own ship has no position or motion".into());
    };
    let policy = CpaPolicy::new(
        Distance::from_nautical_miles(2.0)?,
        Duration::from_secs(30 * 60),
    )?;

    let mut picture = Vec::new();
    for (&mmsi, ship) in &ships {
        let (Some(position), Some(target)) = (ship.position, motion(ship)) else {
            continue;
        };
        if mmsi == OWN_SHIP {
            continue;
        }
        let line = rhumb_line(own_position, position)?;
        let contact = Contact {
            bearing: line.initial_course,
            range: line.distance,
        };
        picture.push((mmsi, ship, assess(own_motion, contact, target, &policy)?));
    }
    picture.sort_by(|a, b| order(&a.2).total_cmp(&order(&b.2)));

    println!(
        "{messages} AIS messages, {} ships; own ship {} ({OWN_SHIP}), {} on {}, at {:.3}\n",
        ships.len(),
        own.name.as_deref().unwrap_or("unnamed"),
        own_motion.speed,
        own_motion.course,
        own_position
    );
    println!(
        "{:<22} {:>9} {:>8} {:>8} {:>7}  risk",
        "ship", "bearing", "range", "CPA", "TCPA"
    );
    for (mmsi, ship, assessment) in picture.iter().take(10) {
        let name = ship.name.clone().unwrap_or_else(|| mmsi.to_string());
        let (cpa, tcpa) = match (assessment.cpa_distance(), assessment.tcpa()) {
            (Some(cpa), Some(tcpa)) => (format!("{cpa:.2}"), minutes(tcpa)),
            _ => ("-".to_owned(), "-".to_owned()),
        };
        println!(
            "{name:<22} {:>9} {:>8} {cpa:>8} {tcpa:>7}  {}",
            assessment.bearing().to_string(),
            format!("{:.2}", assessment.range()),
            risk(assessment.risk())
        );
    }
    let count = |wanted: CollisionRisk| {
        picture
            .iter()
            .filter(|(_, _, a)| a.risk() == wanted)
            .count()
    };
    println!(
        "\n{} ships assessed: {} dangerous, {} developing, {} passing clear, {} opening.",
        picture.len(),
        count(CollisionRisk::Dangerous),
        count(CollisionRisk::Developing),
        count(CollisionRisk::Passing),
        count(CollisionRisk::Opening)
    );
    Ok(())
}

/// Reassembles and decodes the recording: the last state of each ship, and
/// the number of messages decoded.
fn decode() -> (BTreeMap<u32, Ship>, usize) {
    let mut assembler = Assembler::new();
    let now = Instant::<Utc>::from_unix_seconds(1_397_678_400);
    let mut ships: BTreeMap<u32, Ship> = BTreeMap::new();
    let mut messages = 0;
    for line in LOG.lines().filter(|line| !line.starts_with('#')) {
        let Ok(Sentence::Vdm(vdm)) = parse(line.as_bytes()) else {
            continue;
        };
        let Ok(Some(bits)) = assembler.push(&vdm, now) else {
            continue;
        };
        let Ok(message) = Message::decode(&bits) else {
            continue;
        };
        messages += 1;
        match message {
            Message::PositionReport(report) => {
                let ship = ships.entry(report.mmsi.number()).or_default();
                ship.position = report.position.or(ship.position);
                ship.course = report.course;
                ship.speed = report.speed;
                if let Some(name) = report.name {
                    ship.name = Some(name.as_str().trim().to_owned());
                }
            }
            Message::StaticAndVoyageData(data) => {
                if let Some(name) = data.name {
                    ships.entry(data.mmsi.number()).or_default().name =
                        Some(name.as_str().trim().to_owned());
                }
            }
            Message::StaticDataReport(report) => {
                if let StaticDataPart::A { name: Some(name) } = report.part {
                    ships.entry(report.mmsi.number()).or_default().name =
                        Some(name.as_str().trim().to_owned());
                }
            }
            _ => {}
        }
    }
    (ships, messages)
}

/// Course and speed; a ship at rest may report no course.
fn motion(ship: &Ship) -> Option<Vessel> {
    let speed = ship.speed?;
    let course = match ship.course {
        Some(course) => course,
        None if speed.knots() == 0.0 => TrueCourse::NORTH,
        None => return None,
    };
    Some(Vessel { course, speed })
}

/// Closest first: CPA of a closing ship, and opening ships last.
fn order(assessment: &CollisionAssessment) -> f64 {
    assessment
        .cpa_distance()
        .map_or(f64::INFINITY, Distance::nautical_miles)
}

fn minutes(duration: Duration) -> String {
    let seconds = duration.as_secs();
    format!("{}:{:02}", seconds / 60, seconds % 60)
}

fn risk(risk: CollisionRisk) -> &'static str {
    match risk {
        CollisionRisk::Dangerous => "DANGEROUS",
        CollisionRisk::Developing => "developing",
        CollisionRisk::Passing => "passing clear",
        CollisionRisk::Opening => "opening",
        _ => "unknown",
    }
}
