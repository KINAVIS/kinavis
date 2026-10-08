//! A u-blox receiver's own words, checked: fix, integrity, grids, height.
//!
//! A NEO-M8P in Prague (gpsd regression log `ublox-neo-m8p.log`), UBX
//! binary. For each epoch: the `NAV-PVT` fix with the receiver's accuracy,
//! a RAIM test of the `NAV-SAT` residuals with protection levels, the
//! position on UTM, MGRS and СК-42 (Gauss–Krüger zone 3), and the height
//! above mean sea level by EGM96 against the receiver's own.
//!
//! In the third epoch a 40 m error is injected into the range of the highest
//! satellite, and RAIM finds it.
//!
//! ```text
//! cargo run -p kinavis-examples --example integrity
//! ```

use kinavis_geoid::GridGeoid;
use kinavis_grid::{Mgrs, ProjectedCrs, UtmUps};
use kinavis_integrity::{assess, IntegrityReport, RaimConfig, RangeError};
use kinavis_kernel::{Distance, SatelliteSet};
use kinavis_ubx::{Deframer, Event, Message, NavPvt};

const LOG: &[u8] = include_bytes!("../data/ublox-neo-m8p.ubx");

/// Epochs shown.
const EPOCHS: usize = 5;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // A single-frequency code receiver: 3 m at the zenith, more low down.
    let raim = RaimConfig::new(
        1e-3,
        1e-3,
        RangeError::ElevationWeighted(Distance::from_metres(3.0)?),
    )?;
    let sk42 = ProjectedCrs::pulkovo_1942(3)?;
    let mut deframer: Deframer<2048> = Deframer::new();
    let mut fix: Option<NavPvt> = None;
    let mut shown = 0;
    for &byte in LOG {
        let Some(Event::Frame(frame)) = deframer.push(byte) else {
            continue;
        };
        match Message::decode(&frame) {
            Ok(Message::NavPvt(pvt)) => fix = Some(pvt),
            Ok(Message::NavSat(sat)) => {
                let Some(pvt) = fix.filter(|pvt| pvt.itow == sat.itow) else {
                    continue;
                };
                let mut sky: SatelliteSet = sat.satellite_set()?;
                shown += 1;
                if shown == 3 {
                    inject(&mut sky, 40.0)?;
                }
                epoch(&pvt, &sky, &raim, &sk42)?;
                if shown == EPOCHS {
                    break;
                }
            }
            _ => {}
        }
    }
    Ok(())
}

/// Adds `metres` to the residual of the highest satellite used.
fn inject(sky: &mut SatelliteSet, metres: f64) -> Result<(), Box<dyn std::error::Error>> {
    let highest = sky
        .iter()
        .filter(|view| view.is_used())
        .max_by(|a, b| {
            let elevation = |v: &&kinavis_kernel::SatelliteView| {
                v.elevation().map_or(-90.0, kinavis_kernel::Angle::degrees)
            };
            elevation(a).total_cmp(&elevation(b))
        })
        .copied()
        .ok_or("no satellite used")?;
    let residual = highest.residual().map_or(0.0, Distance::metres) + metres;
    println!("  (injected: {metres:.0} m on {})", highest.id());
    sky.insert(highest.with_residual(Distance::from_metres(residual)?))?;
    Ok(())
}

fn epoch(
    pvt: &NavPvt,
    sky: &SatelliteSet,
    raim: &RaimConfig,
    sk42: &ProjectedCrs,
) -> Result<(), Box<dyn std::error::Error>> {
    let fix = pvt.fix()?;
    println!(
        "{:.1}  {:?}, {} satellites, ±{:.2} m (receiver)",
        fix.taken_at(),
        fix.quality().fix_type(),
        pvt.satellites,
        fix.horizontal_accuracy().map_or(f64::NAN, Distance::metres),
    );
    match assess(sky, raim)? {
        IntegrityReport::Available {
            protection,
            statistic,
            threshold,
        } => println!(
            "  RAIM   consistent (test {statistic:.2} ≤ {threshold:.2}), HPL {:.1} m, VPL {:.1} m",
            protection.horizontal.metres(),
            protection.vertical.metres()
        ),
        IntegrityReport::FaultDetected {
            excluded,
            after_exclusion,
            statistic,
            threshold,
        } => println!(
            "  RAIM   FAULT (test {statistic:.2} > {threshold:.2}), excluded {}{}",
            excluded.map_or_else(|| "none".to_owned(), |id| id.to_string()),
            after_exclusion.map_or_else(String::new, |p| format!(
                ", then HPL {:.1} m",
                p.horizontal.metres()
            ))
        ),
        other => println!("  RAIM   {other:?}"),
    }
    let position = fix.position();
    if let UtmUps::Utm {
        zone,
        hemisphere,
        easting,
        northing,
    } = UtmUps::from_position(position)?.coordinate
    {
        println!(
            "  UTM    {zone}{} {:.2} E {:.2} N",
            if matches!(hemisphere, kinavis_grid::Hemisphere::North) {
                'N'
            } else {
                'S'
            },
            easting.metres(),
            northing.metres()
        );
    }
    println!("  MGRS   {}", Mgrs::from_position(position, 5)?);
    let gk = sk42.project(position)?;
    println!(
        "  СК-42  x {:.2}  y {:.2}  (zone 3)",
        gk.northing.metres(),
        gk.easting.metres()
    );
    let msl = pvt.geodetic_point().to_mean_sea_level(&GridGeoid::EGM96)?;
    println!(
        "  height {:.2} m above the ellipsoid, {:.2} m above mean sea level by EGM96, {:.2} m by the receiver\n",
        pvt.height.metres(),
        msl.height().value().metres(),
        pvt.height_msl.metres()
    );
    Ok(())
}
