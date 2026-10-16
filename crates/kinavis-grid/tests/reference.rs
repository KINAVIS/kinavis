//! The grids against two other implementations, from `tools/reference.py`:
//! UTM/UPS against GeographicLib's `GeoConvert` (4649 points over the whole
//! Earth, zone edges, the Norway and Svalbard exceptions, the poles), and
//! the Gauss–Krüger presets and the British National Grid against PROJ
//! (10 000 points, projection only, no datum shift).

#![allow(clippy::expect_used, clippy::panic, clippy::float_cmp)]

use kinavis_grid::{Hemisphere, ProjectedCrs, UtmUps};
use kinavis_kernel::{Distance, Latitude, Longitude, Position};

const UTM_UPS: &str = include_str!("data/utm_ups_reference.txt");
const GAUSS_KRUGER: &str = include_str!("data/gauss_kruger_reference.txt");

fn rows(text: &str) -> impl Iterator<Item = Vec<&str>> {
    text.lines()
        .filter(|line| !line.starts_with('#'))
        .map(|line| line.split_ascii_whitespace().collect())
}

fn number(field: &str) -> f64 {
    field.parse().expect("a number")
}

fn metres(value: f64) -> Distance {
    Distance::from_metres(value).expect("finite")
}

#[test]
fn utm_and_ups_agree_with_geographiclib_to_ten_micrometres() {
    let mut checked = 0;
    for row in rows(UTM_UPS) {
        let [lat, lon, zone, hemisphere, easting, northing, gamma, k] = row[..] else {
            panic!("eight columns: {row:?}");
        };
        let (lat, lon) = (number(lat), number(lon));
        let position = Position::from_degrees(lat, lon).expect("on Earth");
        let gridded = UtmUps::from_position(position).expect("a standard zone");
        let hemisphere = if hemisphere == "n" {
            Hemisphere::North
        } else {
            Hemisphere::South
        };
        let (ours_zone, ours_hemisphere, ours_e, ours_n) = match gridded.coordinate {
            UtmUps::Utm {
                zone,
                hemisphere,
                easting,
                northing,
            } => (zone.number(), hemisphere, easting, northing),
            UtmUps::Ups {
                hemisphere,
                easting,
                northing,
            } => (0, hemisphere, easting, northing),
        };
        assert_eq!(
            u32::from(ours_zone),
            zone.parse::<u32>().expect("zone"),
            "zone at {lat} {lon}"
        );
        assert_eq!(ours_hemisphere, hemisphere, "hemisphere at {lat} {lon}");
        assert!(
            (ours_e.metres() - number(easting)).abs() < 1e-5,
            "easting at {lat} {lon}: {ours_e:?}"
        );
        assert!(
            (ours_n.metres() - number(northing)).abs() < 1e-5,
            "northing at {lat} {lon}: {ours_n:?}"
        );
        assert!(
            (gridded.convergence.degrees() - number(gamma)).abs() < 1e-9 || (lat.abs() == 90.0),
            "convergence at {lat} {lon}: {}",
            gridded.convergence.degrees()
        );
        assert!(
            (gridded.scale - number(k)).abs() < 1e-12,
            "scale at {lat} {lon}: {}",
            gridded.scale
        );

        // And back.
        let (back, _, _) = gridded.coordinate.to_position().expect("inside its limits");
        assert!(
            (back.latitude().degrees() - lat).abs() < 1e-9,
            "latitude back at {lat} {lon}"
        );
        if lat.abs() < 90.0 {
            assert!(
                back.longitude_difference(position).degrees().abs()
                    < 1e-9 / (lat.to_radians().cos()).max(1e-3),
                "longitude back at {lat} {lon}: {back}"
            );
        }
        checked += 1;
    }
    assert_eq!(checked, 4649);
}

#[test]
fn national_grids_agree_with_proj_to_a_millimetre() {
    let mut checked = 0;
    for row in rows(GAUSS_KRUGER) {
        let [preset, zone, lat, lon, easting, northing] = row[..] else {
            panic!("six columns: {row:?}");
        };
        let zone: u8 = zone.parse().expect("zone");
        let crs = match preset {
            "pulkovo_1942" => ProjectedCrs::pulkovo_1942(zone),
            "pulkovo_1995" => ProjectedCrs::pulkovo_1995(zone),
            "gsk_2011" => ProjectedCrs::gsk_2011(zone),
            "dhdn" => ProjectedCrs::dhdn(zone),
            "bng" => ProjectedCrs::british_national_grid(),
            other => panic!("unknown preset {other}"),
        }
        .expect("a valid zone");
        let (lat, lon) = (number(lat), number(lon));
        let projected = crs
            .projection()
            .project(
                Latitude::from_degrees(lat).expect("in range"),
                Longitude::from_degrees(lon).expect("in range"),
            )
            .expect("projects");
        assert!(
            (projected.easting.metres() - number(easting)).abs() < 1e-3
                && (projected.northing.metres() - number(northing)).abs() < 1e-3,
            "{preset} zone {zone} at {lat} {lon}: {:.6} {:.6}",
            projected.easting.metres(),
            projected.northing.metres()
        );
        let (back_lat, back_lon, _, _) = crs
            .projection()
            .unproject(metres(number(easting)), metres(number(northing)))
            .expect("unprojects");
        let longitude_error = kinavis_kernel::wrap180(back_lon.degrees() - lon).abs();
        assert!(
            (back_lat.degrees() - lat).abs() < 1e-8 && longitude_error < 1e-8,
            "{preset} back at {lat} {lon}: {} {}",
            back_lat.degrees(),
            back_lon.degrees()
        );
        checked += 1;
    }
    assert_eq!(checked, 10_000);
}
