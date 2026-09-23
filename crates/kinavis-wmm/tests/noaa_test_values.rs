//! NOAA's hundred WMM2025 test values (`WMM2025_TestValues.txt`), unchanged,
//! all checked.
//!
//! Each line: decimal year, height above the WGS-84 ellipsoid in km, geodetic
//! latitude and longitude, then `D`, `I`, `H`, `X`, `Y`, `Z`, `F` and their
//! annual rates. Rates are not checked: the port does not expose secular
//! variation.

#![cfg(feature = "wmm2025")]
#![allow(clippy::unwrap_used, clippy::indexing_slicing)]

use kinavis_kernel::{Distance, GeodeticPoint, Height, Position};
use kinavis_wmm::{DecimalYear, Wmm};

const TEST_VALUES: &str = include_str!("WMM2025_TestValues.txt");

/// Published to six decimals of a nT; the synthesis must match to 0.01 nT.
const FIELD_TOLERANCE_NT: f64 = 0.01;
/// Angles are published to 0.01°; half a unit in the last place is rounding.
const ANGLE_TOLERANCE_DEG: f64 = 0.005 + 1e-9;

struct Row {
    year: f64,
    height_km: f64,
    latitude: f64,
    longitude: f64,
    declination: f64,
    inclination: f64,
    horizontal: f64,
    north: f64,
    east: f64,
    down: f64,
    total: f64,
}

fn rows() -> Vec<Row> {
    TEST_VALUES
        .lines()
        .filter(|line| !line.starts_with('#') && !line.trim().is_empty())
        .map(|line| {
            let fields: Vec<f64> = line
                .split_whitespace()
                .map(|field| field.parse().unwrap())
                .collect();
            assert_eq!(fields.len(), 18, "{line}");
            Row {
                year: fields[0],
                height_km: fields[1],
                latitude: fields[2],
                longitude: fields[3],
                declination: fields[4],
                inclination: fields[5],
                horizontal: fields[6],
                north: fields[7],
                east: fields[8],
                down: fields[9],
                total: fields[10],
            }
        })
        .collect()
}

#[test]
fn every_published_test_value_is_reproduced() {
    let rows = rows();
    assert_eq!(rows.len(), 100);
    for row in rows {
        let point = GeodeticPoint::new(
            Position::from_degrees(row.latitude, row.longitude).unwrap(),
            Height::above_ellipsoid(Distance::from_kilometres(row.height_km).unwrap()),
        );
        let field = Wmm::WMM2025
            .field_in(point, DecimalYear::new(row.year).unwrap())
            .unwrap();
        let context = format!(
            "{} at {}°, {}°, {} km",
            row.year, row.latitude, row.longitude, row.height_km
        );

        let close = |got: f64, want: f64, what: &str| {
            assert!(
                (got - want).abs() < FIELD_TOLERANCE_NT,
                "{what}: got {got}, published {want} — {context}"
            );
        };
        close(field.north_nanotesla(), row.north, "X");
        close(field.east_nanotesla(), row.east, "Y");
        close(field.down_nanotesla(), row.down, "Z");
        close(field.horizontal_intensity_nanotesla(), row.horizontal, "H");
        close(field.total_intensity_nanotesla(), row.total, "F");

        let declination = field.declination().degrees();
        assert!(
            (declination - row.declination).abs() < ANGLE_TOLERANCE_DEG,
            "D: got {declination}, published {} — {context}",
            row.declination
        );
        let inclination = field.inclination().degrees();
        assert!(
            (inclination - row.inclination).abs() < ANGLE_TOLERANCE_DEG,
            "I: got {inclination}, published {} — {context}",
            row.inclination
        );
    }
}
