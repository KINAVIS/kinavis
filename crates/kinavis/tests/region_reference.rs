//! Polygons and regions against GEOS and CGAL on real field boundaries.
//!
//! `fixtures/region_reference.txt` is written by `tools/region_reference.py`
//! from parcels of the Dutch crop parcel register (CC0) in RD New: area,
//! perimeter and where 1100 points per parcel fall from GEOS, and the inward
//! offset by 3, 12 and 24 m along the straight skeleton from CGAL.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use kinavis::region::{Containment, PlanarPoint, Polygon, Region};
use kinavis::{Distance, NavigationError};

const REFERENCE: &str = include_str!("fixtures/region_reference.txt");

/// Outer boundary capacity: the largest parcel has 452 vertices.
type Field = Region<512, 512, 4>;

/// Same generator as the script: splitmix64.
struct SplitMix64(u64);

impl SplitMix64 {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    #[allow(clippy::cast_precision_loss)]
    fn unit(&mut self) -> f64 {
        (self.next() >> 11) as f64 * 2f64.powi(-53)
    }
}

/// The script's points: uniform over the bounding box widened by a tenth.
fn points(seed: u64, count: usize, bounds: [f64; 4]) -> Vec<PlanarPoint> {
    let [west, south, east, north] = bounds;
    let pad_x = (east - west) / 10.0;
    let pad_y = (north - south) / 10.0;
    let mut rng = SplitMix64(seed);
    (0..count)
        .map(|_| {
            let x = west - pad_x + rng.unit() * (east - west + 2.0 * pad_x);
            let y = south - pad_y + rng.unit() * (north - south + 2.0 * pad_y);
            PlanarPoint::from_metres(x, y).unwrap()
        })
        .collect()
}

fn code(containment: Containment) -> char {
    match containment {
        Containment::Inside => 'I',
        Containment::OnBoundary => 'B',
        Containment::Outside => 'O',
    }
}

struct Parcel {
    index: usize,
    rings: Vec<Vec<PlanarPoint>>,
    area: f64,
    perimeter: f64,
    seed: u64,
    expected: String,
    insets: Vec<(f64, Inset)>,
}

enum Inset {
    Empty,
    Multi,
    Area {
        area: f64,
        seed: u64,
        expected: String,
    },
}

fn parcels() -> Vec<Parcel> {
    let mut out: Vec<Parcel> = Vec::new();
    for line in REFERENCE.lines().filter(|l| !l.starts_with('#')) {
        let mut words = line.split_whitespace();
        let tag = words.next().unwrap();
        let words: Vec<&str> = words.collect();
        if tag == "parcel" {
            out.push(Parcel {
                index: words[0].parse().unwrap(),
                rings: Vec::new(),
                area: 0.0,
                perimeter: 0.0,
                seed: 0,
                expected: String::new(),
                insets: Vec::new(),
            });
            continue;
        }
        let parcel = out.last_mut().unwrap();
        match tag {
            "ring" => {
                let numbers: Vec<f64> = words.iter().map(|w| w.parse().unwrap()).collect();
                let ring = numbers
                    .chunks(2)
                    .map(|xy| PlanarPoint::from_metres(xy[0], xy[1]).unwrap())
                    .collect();
                parcel.rings.push(ring);
            }
            "area" => parcel.area = words[0].parse().unwrap(),
            "perimeter" => parcel.perimeter = words[0].parse().unwrap(),
            "points" => {
                parcel.seed = words[0].parse().unwrap();
                words[2].clone_into(&mut parcel.expected);
            }
            "inset" => {
                let distance = words[0].parse().unwrap();
                let inset = match words[1] {
                    "EMPTY" => Inset::Empty,
                    "MULTI" => Inset::Multi,
                    _ => Inset::Area {
                        area: words[2].parse().unwrap(),
                        seed: words[3].parse().unwrap(),
                        expected: words[5].to_owned(),
                    },
                };
                parcel.insets.push((distance, inset));
            }
            other => unreachable!("unknown record {other}"),
        }
    }
    out
}

fn region(parcel: &Parcel) -> Field {
    let outer = Polygon::new(&parcel.rings[0]).unwrap();
    let holes: Vec<Polygon<512>> = parcel.rings[1..]
        .iter()
        .map(|r| Polygon::new(r).unwrap())
        .collect();
    Region::new(outer, &holes).unwrap()
}

fn bounds(parcel: &Parcel) -> [f64; 4] {
    let outer = &parcel.rings[0];
    let xs = outer.iter().map(|p| p.east_metres());
    let ys = outer.iter().map(|p| p.north_metres());
    [
        xs.clone().fold(f64::INFINITY, f64::min),
        ys.clone().fold(f64::INFINITY, f64::min),
        xs.fold(f64::NEG_INFINITY, f64::max),
        ys.fold(f64::NEG_INFINITY, f64::max),
    ]
}

fn placed(region: &Field, seed: u64, count: usize, bounds: [f64; 4]) -> String {
    points(seed, count, bounds)
        .into_iter()
        .map(|p| code(region.contains(p)))
        .collect()
}

#[test]
fn area_and_perimeter_match_geos() {
    for parcel in parcels() {
        let region = region(&parcel);
        let area = region.area().square_metres();
        let perimeter = region.perimeter().metres();
        assert!(
            ((area - parcel.area) / parcel.area).abs() < 1e-9,
            "parcel {}: area {area} against {}",
            parcel.index,
            parcel.area
        );
        assert!(
            ((perimeter - parcel.perimeter) / parcel.perimeter).abs() < 1e-9,
            "parcel {}: perimeter {perimeter} against {}",
            parcel.index,
            parcel.perimeter
        );
    }
}

#[test]
fn a_hundred_thousand_points_fall_where_geos_puts_them() {
    let mut total = 0;
    for parcel in parcels() {
        let region = region(&parcel);
        let got = placed(&region, parcel.seed, parcel.expected.len(), bounds(&parcel));
        assert_eq!(got, parcel.expected, "parcel {}", parcel.index);
        total += got.len();
    }
    assert!(total >= 100_000, "{total} points");
}

#[test]
fn insets_match_the_straight_skeleton() {
    let mut matched = 0;
    let mut refused = Vec::new();
    for parcel in parcels() {
        let region = region(&parcel);
        for (distance, expected) in &parcel.insets {
            let result = region.inset(Distance::from_metres(*distance).unwrap());
            match (expected, result) {
                (Inset::Empty, Err(NavigationError::OffsetCollapses { .. }))
                | (Inset::Multi, Err(NavigationError::OffsetSplits { .. })) => matched += 1,
                (
                    Inset::Area {
                        area,
                        seed,
                        expected,
                    },
                    Ok(inset),
                ) => {
                    let got_area = inset.area().square_metres();
                    let got = placed(&inset, *seed, expected.len(), bounds(&parcel));
                    let differing = got
                        .chars()
                        .zip(expected.chars())
                        .filter(|(a, b)| a != b)
                        .count();
                    if
                    // Relative, and the 1e-6 m² the file is written to.
                    (got_area - area).abs() <= 1e-8 * area + 1e-5 && differing == 0 {
                        matched += 1;
                    } else {
                        refused.push(format!(
                            "parcel {} inset {distance}: area {got_area:.3} against {area:.3}, {differing} points differ",
                            parcel.index
                        ));
                    }
                }
                (expected, result) => refused.push(format!(
                    "parcel {} inset {distance}: CGAL {}, got {:?}",
                    parcel.index,
                    match expected {
                        Inset::Empty => "EMPTY".to_owned(),
                        Inset::Multi => "MULTI".to_owned(),
                        Inset::Area { area, .. } => format!("area {area:.1}"),
                    },
                    result.map(|r| r.area().square_metres())
                )),
            }
        }
    }
    assert!(refused.is_empty(), "{}", refused.join("\n"));
    assert_eq!(matched, 276);
}
