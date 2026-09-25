//! Course to steer and cross-track error along a real track.
//!
//! The fixes are a B&G Zeus² 9 chartplotter on a yacht off Stavoren, NL,
//! 20 August 2018 (gpsd regression log `bundg_zeus_9.log`). The passage plan
//! is drawn for this example: one leg towards Medemblik. Every thirty seconds
//! the guidance gives the desired track, the course to steer to make it good
//! (the track corrected for current and leeway, neither known here), the
//! cross-track error and the distance to go, and raises an alarm while the
//! yacht is past the cross-track limit of one cable.
//!
//! ```text
//! cargo run -p kinavis-examples --example course_to_steer
//! ```

use core::time::Duration;

use kinavis::gnss_intake::{GnssIntake, IntakeConfig};
use kinavis::guidance::{guide, GuidanceConfig};
use kinavis::route::{LegKind, Route};
use kinavis::{
    Distance, EnvironmentSample, GeodeticPoint, GnssFix, GuidanceEvent, Height, NavigationError,
    Position, Speed,
};
use kinavis_nmea0183::{parse, Sentence};

const LOG: &str = include_str!("../data/bundg_zeus_9.nmea");

fn main() -> Result<(), NavigationError> {
    let start: Position = "52°51.40'N 005°19.20'E".parse()?;
    let medemblik: Position = "52°46.20'N 005°06.60'E".parse()?;
    let route = Route::new(&[start, medemblik], LegKind::RhumbLine)?;
    // Cross-track limit one cable, arrival radius two cables.
    let config = GuidanceConfig::new(Distance::from_cables(1.0)?, Distance::from_cables(2.0)?)?;
    let mut intake = GnssIntake::new(IntakeConfig {
        max_age: Duration::from_secs(10),
        max_speed: Speed::from_knots(40.0)?,
    });

    println!("Leg from {start:.2} to {medemblik:.2}\n");
    println!(
        "{:<9} {:<25} {:>8} {:>8} {:>8} {:>16} {:>7}  alarm",
        "UTC", "position", "COG", "track", "steer", "off track", "to go"
    );
    let fixes = LOG.lines().filter_map(|line| match parse(line.as_bytes()) {
        Ok(Sentence::Rmc(rmc)) => GnssFix::try_from(rmc).ok(),
        _ => None,
    });
    for (index, fix) in fixes.enumerate() {
        let _ = intake.accept(fix);
        if index % 30 != 0 {
            continue;
        }
        let now = fix.taken_at();
        let here = GeodeticPoint::new(fix.position(), Height::above_ellipsoid(Distance::ZERO));
        let sea = EnvironmentSample::at(here, now);
        let snapshot = intake.snapshot_at(now);
        let (view, events) = guide(&snapshot, &route, route.first_leg(), &sea, &config)?;

        let error = view.cross_track_error();
        let clock = format!("{now:.0}");
        let cog = fix
            .course_over_ground()
            .map_or_else(|| "-".to_owned(), |cog| cog.to_string());
        let alarm = events
            .iter()
            .filter_map(|event| match event {
                GuidanceEvent::CrossTrackExceeded { error, limit, .. } => {
                    Some(format!("off track: {error:.2} > {limit:.2}"))
                }
                _ => None,
            })
            .collect::<Vec<_>>()
            .join("; ");
        println!(
            "{:<9} {:<25} {cog:>8} {:>8} {:>8} {:>16} {:>7}  {alarm}",
            clock.get(11..19).unwrap_or(&clock),
            format!("{:.3}", fix.position()),
            view.desired_track().to_string(),
            view.course_to_steer().to_string(),
            format!("{:.2} {:?}", error.distance, error.side),
            format!("{:.2}", view.distance_to_waypoint()),
        );
    }
    Ok(())
}
