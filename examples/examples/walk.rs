//! A walk up Elbrus on the map: MGRS references of the stops, the card of
//! bearings to follow by compass, the profile of each leg on SRTM, the time
//! by Naismith's rule, a resection on three summits, a crevasse field kept
//! out of, and the route as GPX.
//!
//! The terrain is real: the SRTM 1″ tile N43E042 (public domain, USGS; from
//! the Terrain Tiles on AWS, not in the repository). `SRTM_TILES` names the
//! directory that holds it. The stops are those of the southern route, at
//! positions read off a map to a few hundred metres; the compass readings
//! of the resection are constructed from a known point with a known
//! compass error, and said to be.
//!
//! ```text
//! SRTM_TILES=… cargo run -p kinavis-examples --example walk
//! ```

use core::time::Duration;
use std::error::Error;
use std::path::PathBuf;

use kinavis::region::{PlanarPoint, Polygon, Region};
use kinavis::route::{LegKind, Route};
use kinavis_grid::{Mgrs, MgrsSquare};
use kinavis_kernel::environment::MagneticModel;
use kinavis_kernel::time::{Civil, Instant, Utc};
use kinavis_kernel::{
    Angle, Distance, ElevationModel, GeodeticPoint, GridBearing, Height, Position,
};
use kinavis_land::{
    grid_bearing, legs_of, three_point_resection, BearingFrame, FenceConfig, FenceKind, Geofence,
    GeofenceEvent, Gpx, GpxPoint, GpxRoute, GridPoint, Leg, MapGrid, Mils,
};
use kinavis_terrain::{
    line_of_sight, profile, summarise, HgtTile, ProfilePoint, Terrain, TileSource, WalkingPace,
    STANDARD_REFRACTION,
};
use kinavis_wmm::Wmm;

type Outcome<T = ()> = Result<T, Box<dyn Error>>;

/// The stops: the cable car's lower station, the Garabashi huts, the
/// Pastukhov rocks, the saddle, the eastern summit.
const STOPS: [(&str, f64, f64); 5] = [
    ("Azau", 43.2647, 42.4790),
    ("Garabashi", 43.3015, 42.4650),
    ("Pastukhov rocks", 43.3273, 42.4560),
    ("Saddle", 43.3483, 42.4434),
    ("East summit", 43.3528, 42.4392),
];

/// Summits seen from the hut for the resection.
const SUMMITS: [(&str, f64, f64); 3] = [
    ("West summit", 43.3497, 42.4258),
    ("East summit", 43.3528, 42.4392),
    ("Donguz-Orun", 43.2150, 42.5050),
];

/// The compass's error in the constructed readings, degrees.
const COMPASS_ERROR: f64 = 2.5;

fn main() -> Outcome {
    let directory = PathBuf::from(std::env::var("SRTM_TILES").map_err(|_| "set SRTM_TILES")?);
    let bytes = std::fs::read(directory.join("N43E042.hgt"))?;
    let tiles = [HgtTile::new(&bytes, "N43E042".parse()?)?];
    let terrain = Terrain::new(&tiles);

    let stops = read_stops(&terrain)?;
    let [_, hut, rocks, _, summit] = stops;
    let grid = MapGrid::utm_at(hut)?;
    let when = Instant::<Utc>::from_civil(Civil::date(2026, 7, 15))?;
    let frame = compass(&grid, hut, when)?;
    card(&terrain, &grid, &frame, &stops)?;
    resection(&grid, &frame, hut)?;
    let sight = line_of_sight(
        &terrain,
        hut,
        Distance::from_metres(1.7)?,
        summit,
        Distance::ZERO,
        4000,
        STANDARD_REFRACTION,
    )?;
    println!(
        "The summit from the hut: {} (least clearance {:.0} m, {:.1} km out)",
        if sight.visible { "in sight" } else { "hidden" },
        sight.clearance.metres(),
        sight.at.metres() / 1000.0
    );
    crevasses(&grid, hut, rocks, when)?;
    gpx(&stops)
}

/// The stops as a guide gives them, MGRS references to the metre, read back
/// to the centres of their squares.
fn read_stops<S: TileSource>(terrain: &Terrain<S>) -> Outcome<[Position; 5]> {
    println!("Stops (MGRS, height on SRTM):");
    let mut stops = [Position::from_degrees(0.0, 0.0)?; 5];
    for (slot, &(name, lat, lon)) in stops.iter_mut().zip(&STOPS) {
        let reference = Mgrs::from_position(Position::from_degrees(lat, lon)?, 5)?;
        let square: MgrsSquare = reference.as_str().parse()?;
        let (centre, ..) = square.centre.to_position()?;
        let height = terrain.elevation(centre)?;
        println!(
            "  {name:<16} {reference}  {:>5.0} m",
            height.value().metres()
        );
        *slot = centre;
    }
    Ok(stops)
}

/// The compass of 2026: declination by WMM2025 at the hut, convergence of
/// the hut's UTM zone.
fn compass(grid: &MapGrid, hut: Position, when: Instant<Utc>) -> Outcome<BearingFrame> {
    let point = GeodeticPoint::new(
        hut,
        Height::above_mean_sea_level(Distance::from_metres(3850.0)?),
    );
    let declination = Wmm::WMM2025.field_at(point, when)?.declination();
    let frame = BearingFrame::at(grid, hut, declination)?;
    println!(
        "\nDeclination {declination}, convergence {:+.2}°: grid magnetic angle {:+.2}°",
        frame.convergence().degrees(),
        frame.grid_magnetic_angle().degrees()
    );
    Ok(frame)
}

/// The card: bearing and distance of each leg, its climb and its time.
fn card<S: TileSource>(
    terrain: &Terrain<S>,
    grid: &MapGrid,
    frame: &BearingFrame,
    stops: &[Position; 5],
) -> Outcome {
    let [start, ..] = *stops;
    let route = Route::new(stops, LegKind::RhumbLine)?;
    let blank = Leg {
        bearing: GridBearing::NORTH,
        distance: Distance::ZERO,
    };
    let mut legs = [blank; 4];
    legs_of(grid, &route, &mut legs)?;
    let mut samples = vec![
        ProfilePoint {
            distance: Distance::ZERO,
            position: start,
            elevation: Height::above_mean_sea_level(Distance::ZERO),
        };
        400
    ];
    let (mut total, mut climbed, mut walked) = (Duration::ZERO, 0.0, 0.0);
    println!("\n  leg                              magnetic  mils    ground   climb   time");
    let ends = stops.windows(2).zip(STOPS.windows(2));
    for (leg, (pair, names)) in legs.iter().zip(ends) {
        let ([from, to], [(a, ..), (b, ..)]) = (pair, names) else {
            continue;
        };
        profile(terrain, *from, *to, &mut samples)?;
        // SRTM's noise of a few metres is not climbing.
        let summary = summarise(&samples, Distance::from_metres(5.0)?)?;
        let time = WalkingPace::NAISMITH.time_over(&summary);
        total += time;
        climbed += summary.ascent.metres();
        walked += summary.length.metres();
        let magnetic = leg.magnetic_bearing(frame);
        println!(
            "  {a:<15} → {b:<15} {:>6.1}°  {:>5.0}  {:>6.0} m  {:>4.0} m  {}",
            magnetic.degrees(),
            Mils::Soviet.of_direction(magnetic),
            leg.distance.metres(),
            summary.ascent.metres(),
            hours(time)
        );
    }
    println!(
        "  {walked:.0} m and {climbed:.0} m of climbing: {} by Naismith's rule",
        hours(total)
    );
    Ok(())
}

/// Lost in cloud above the hut, the cloud parting on three summits: the
/// readings are constructed from the hut with the compass off by
/// [`COMPASS_ERROR`].
fn resection(grid: &MapGrid, frame: &BearingFrame, hut: Position) -> Outcome {
    let at = grid.project(hut)?.point;
    let mut points = [at; 3];
    let mut readings = [Angle::ZERO; 3];
    for ((point, reading), &(_, lat, lon)) in points.iter_mut().zip(&mut readings).zip(&SUMMITS) {
        *point = grid.project(Position::from_degrees(lat, lon)?)?.point;
        let bearing = grid_bearing(at, *point)?.degrees();
        let magnetic = bearing - frame.grid_magnetic_angle().degrees() - COMPASS_ERROR;
        *reading = Angle::from_degrees(magnetic.rem_euclid(360.0))?;
    }
    let fix = three_point_resection(points, readings)?;
    println!(
        "\nResection on {}: {:.3} m from the hut; compass {:+.2}° from grid north (the grid magnetic angle and the {COMPASS_ERROR}° error)",
        SUMMITS.map(|(name, ..)| name).join(", "),
        apart(fix.point, at),
        fix.orientation.degrees()
    );
    Ok(())
}

/// A 200 × 300 m crevasse field east of the climb from the hut to the
/// rocks, kept out of with 30 m of warning.
fn crevasses(grid: &MapGrid, hut: Position, rocks: Position, when: Instant<Utc>) -> Outcome {
    let c = grid.project(rocks)?.point;
    let (e, n) = (c.easting.metres() + 220.0, c.northing.metres() - 400.0);
    let corner = |de: f64, dn: f64| PlanarPoint::from_metres(e + de, n + dn);
    let square = Polygon::<4>::new(&[
        corner(-100.0, -150.0)?,
        corner(100.0, -150.0)?,
        corner(100.0, 150.0)?,
        corner(-100.0, 150.0)?,
    ])?;
    let config = FenceConfig::new(Distance::from_metres(3.0)?, Distance::from_metres(30.0)?)?;
    let region = Region::<4, 4, 1>::without_holes(square);
    let mut fence = Geofence::new(1, FenceKind::KeepOut, region, *grid, config);
    let mut warned = None;
    let mut clock = when;
    for step in 0..200u16 {
        let t = f64::from(step) / 199.0;
        let lat =
            hut.latitude().degrees() + t * (rocks.latitude().degrees() - hut.latitude().degrees());
        let lon = hut.longitude().degrees()
            + t * (rocks.longitude().degrees() - hut.longitude().degrees());
        clock = clock.saturating_add(Duration::from_secs(10));
        if let Some(GeofenceEvent::ApproachingBoundary { distance, .. }) =
            fence.update(Position::from_degrees(lat, lon)?, clock)?
        {
            warned = warned.or(Some(distance.metres()));
        }
    }
    match warned {
        Some(distance) => println!("Crevasse field: warned {distance:.0} m from its edge"),
        None => println!("Crevasse field: never within 30 m"),
    }
    Ok(())
}

/// The route for a GPS receiver.
fn gpx(stops: &[Position; 5]) -> Outcome {
    let points = stops
        .iter()
        .zip(&STOPS)
        .map(|(position, &(name, ..))| GpxPoint {
            name: Some(String::from(name)),
            ..GpxPoint::at(*position)
        })
        .collect();
    let file = Gpx {
        creator: Some(String::from("KINAVIS walk")),
        routes: vec![GpxRoute {
            name: Some(String::from("Elbrus, southern route")),
            points,
        }],
        ..Gpx::default()
    };
    let text = file.to_xml();
    println!("\nGPX, {} bytes:", text.len());
    for line in text.lines().take(5) {
        println!("  {line}");
    }
    let back = Gpx::parse(&text)?;
    let read = back.routes.first().map_or(0, |route| route.points.len());
    if read != STOPS.len() {
        return Err("the GPX does not read back".into());
    }
    Ok(())
}

fn apart(a: GridPoint, b: GridPoint) -> f64 {
    (a.easting.metres() - b.easting.metres()).hypot(a.northing.metres() - b.northing.metres())
}

fn hours(time: Duration) -> String {
    let minutes = time.as_secs() / 60 + u64::from(time.as_secs() % 60 >= 30);
    format!("{}h{:02}", minutes / 60, minutes % 60)
}
