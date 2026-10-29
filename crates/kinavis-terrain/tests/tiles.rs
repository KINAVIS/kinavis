//! Tiles built in the test: the byte layout, names, borders shared with
//! neighbours, voids, the antimeridian, many tiles as one terrain.

#![allow(
    clippy::expect_used,
    clippy::panic,
    clippy::float_cmp,
    clippy::indexing_slicing,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::identity_op
)]

use kinavis_kernel::{ElevationModel, KernelError, Position, VerticalDatum};
use kinavis_terrain::{HgtTile, Terrain, TileCorner, VOID};

/// File bytes of a tile whose sample at row `r`, column `c` is `f(r, c)`.
fn bytes(side: usize, f: impl Fn(usize, usize) -> i16) -> Vec<u8> {
    (0..side)
        .flat_map(|r| (0..side).map(move |c| (r, c)))
        .flat_map(|(r, c)| f(r, c).to_be_bytes())
        .collect()
}

fn position(lat: f64, lon: f64) -> Position {
    Position::from_degrees(lat, lon).expect("on Earth")
}

fn corner(name: &str) -> TileCorner {
    name.parse().expect("a tile name")
}

#[test]
fn names_read_and_print_as_the_files_have_them() {
    for (name, south, west) in [
        ("N55E037", 55, 37),
        ("s01w072.hgt", -1, -72),
        ("N00W180", 0, -180),
        ("S90E179.HGT", -90, 179),
    ] {
        let c = corner(name);
        assert_eq!((c.south(), c.west()), (south, west));
        assert_eq!(c.to_string(), name[..7].to_uppercase());
    }
    for bad in [
        "N55E37",
        "X55E037",
        "N55E180",
        "N90E000",
        "N5aE037",
        "",
        "N55E037.tif",
    ] {
        assert!(bad.parse::<TileCorner>().is_err(), "{bad:?}");
    }
    assert_eq!(TileCorner::of(position(55.0, 37.0)), corner("N55E037"));
    assert_eq!(TileCorner::of(position(-0.5, -71.2)), corner("S01W072"));
}

#[test]
fn rows_run_south_from_the_north_edge() {
    // 3″ resolution: 1201 per side; sample (r, c) = r·10 + c, mod.
    let file = bytes(1201, |r, c| {
        i16::try_from((r * 7 + c * 3) % 30_000).expect("i16")
    });
    let tile = HgtTile::new(&file, corner("N46E007")).expect("a tile");
    assert_eq!(tile.samples_per_side(), 1201);
    assert!((tile.spacing_arc_seconds() - 3.0).abs() < 1e-12);
    // North-west corner is the first sample, south-east the last.
    let nw = tile.elevation(position(47.0, 7.0)).expect("corner");
    assert_eq!(nw.value().metres(), 0.0);
    assert_eq!(nw.datum(), VerticalDatum::MeanSeaLevel);
    let se = tile.height_metres(position(46.0, 8.0)).expect("corner");
    assert_eq!(se, f64::from(((1200 * 7 + 1200 * 3) % 30_000) as i16));
    // Half way between two samples of a row: their mean.
    let lat = 47.0 - 10.0 / 1200.0;
    let lon = 7.0 + 20.5 / 1200.0;
    let mid = tile.height_metres(position(lat, lon)).expect("inside");
    let mean = f64::from((10 * 7 + 20 * 3 + 10 * 7 + 21 * 3) as i16) / 2.0;
    assert!((mid - mean).abs() < 1e-9, "{mid} against {mean}");
    assert_eq!(tile.sample(0, 1), Some(3));
    assert_eq!(tile.sample(1201, 0), None);
    assert!(tile.height_metres(position(45.999, 7.5)).is_err());
    assert!(tile.height_metres(position(46.5, 8.001)).is_err());
}

#[test]
fn lengths_that_are_not_square_tiles_are_refused() {
    for len in [0, 1, 2, 6, 7, 2 * 3601 * 3601 - 2, 2 * 10_802 * 10_802] {
        let data = vec![0u8; len];
        assert!(HgtTile::new(&data, corner("N00E000")).is_err(), "{len}");
    }
    let small = vec![0u8; 8];
    assert_eq!(
        HgtTile::new(&small, corner("N00E000"))
            .expect("2 × 2")
            .samples_per_side(),
        2
    );
}

#[test]
fn a_void_refuses_the_heights_next_to_it_and_only_those() {
    let file = bytes(5, |r, c| if (r, c) == (2, 2) { VOID } else { 100 });
    let tile = HgtTile::new(&file, corner("N10E010")).expect("a tile");
    assert_eq!(tile.sample(2, 2), None);
    let next = position(10.5 + 0.1, 10.5 - 0.1);
    assert!(matches!(
        tile.elevation(next),
        Err(KernelError::OutsideValidity { .. })
    ));
    let away = position(10.1, 10.1);
    assert_eq!(tile.height_metres(away).expect("clear of the void"), 100.0);
}

#[test]
fn neighbours_share_their_borders_and_serve_as_one_terrain() {
    // Height = 1000·latitude + longitude, sampled exactly on each tile.
    let side = 11;
    let make = |south: i16, west: i16| {
        bytes(side, move |r, c| {
            let lat = f64::from(south) + 1.0 - r as f64 / 10.0;
            let lon = f64::from(west) + c as f64 / 10.0;
            (1000.0 * (lat - 40.0) + 10.0 * (lon - 10.0)).round() as i16
        })
    };
    let files = [make(45, 10), make(45, 11), make(46, 10), make(46, 11)];
    let corners = ["N45E010", "N45E011", "N46E010", "N46E011"];
    let tiles: Vec<HgtTile<'_>> = files
        .iter()
        .zip(corners)
        .map(|(file, name)| HgtTile::new(file, corner(name)).expect("a tile"))
        .collect();
    let terrain = Terrain::new(tiles.as_slice());
    // A plane is reproduced exactly by bilinear interpolation, across borders.
    for (lat, lon) in [
        (45.37, 10.91),
        (46.0, 11.0),
        (46.999, 11.5),
        (45.5, 11.0),
        (46.0, 10.25),
    ] {
        let height = terrain
            .elevation(position(lat, lon))
            .expect("held")
            .value()
            .metres();
        let plane = 1000.0 * (lat - 40.0) + 10.0 * (lon - 10.0);
        assert!(
            (height - plane).abs() < 1e-6,
            "{lat} {lon}: {height} against {plane}"
        );
    }
    // Both tiles of a border give the same height there.
    let border = position(45.63, 11.0);
    let west = tiles[0]
        .height_metres(border)
        .expect("east edge of the west tile");
    let east = tiles[1]
        .height_metres(border)
        .expect("west edge of the east tile");
    assert_eq!(west, east);
    assert!(terrain.elevation(position(44.5, 10.5)).is_err());
}

#[test]
fn the_tile_on_the_antimeridian_holds_180_degrees() {
    let file = bytes(3, |_, c| i16::try_from(c * 10).expect("i16"));
    let tile = HgtTile::new(&file, corner("N10E179")).expect("a tile");
    let east_edge = position(10.5, 180.0);
    assert_eq!(east_edge.longitude().degrees(), -180.0);
    assert_eq!(tile.height_metres(east_edge).expect("the east edge"), 20.0);
    assert!((tile.height_metres(position(10.5, 179.5)).expect("inside") - 10.0).abs() < 1e-9);
}
