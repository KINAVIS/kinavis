//! Heights of two SRTM 1″ tiles, Elbrus (N43E042) and Moscow (N55E037),
//! against GDAL's bilinear interpolation on its own georeferencing of the
//! files, from `tools/reference.py`: 10 000 points, corners, edges and
//! sample nodes. The tiles are not in the repository; `SRTM_TILES` names
//! the directory that holds them.

#![allow(clippy::expect_used, clippy::panic)]

use std::path::PathBuf;

use kinavis_kernel::{ElevationModel, Position};
use kinavis_terrain::{HgtTile, Terrain, TileCorner};

const REFERENCE: &str = include_str!("data/srtm_reference.txt");

#[test]
#[ignore = "needs the SRTM tiles in SRTM_TILES"]
fn heights_agree_with_gdal() {
    let directory = PathBuf::from(std::env::var("SRTM_TILES").expect("SRTM_TILES"));
    let names = ["N43E042", "N55E037"];
    let files: Vec<Vec<u8>> = names
        .iter()
        .map(|name| std::fs::read(directory.join(format!("{name}.hgt"))).expect("a tile"))
        .collect();
    let tiles: Vec<HgtTile<'_>> = names
        .iter()
        .zip(&files)
        .map(|(name, bytes)| HgtTile::new(bytes, name.parse().expect("a name")).expect("a tile"))
        .collect();
    assert!(tiles.iter().all(|tile| tile.samples_per_side() == 3601));
    let terrain = Terrain::new(tiles.as_slice());
    let mut worst = 0.0_f64;
    let mut checked = 0;
    for line in REFERENCE.lines().filter(|line| !line.starts_with('#')) {
        let [name, lat, lon, height] = line.split_ascii_whitespace().collect::<Vec<_>>()[..] else {
            panic!("four columns: {line}");
        };
        let position = Position::from_degrees(lat.parse().expect("lat"), lon.parse().expect("lon"))
            .expect("on Earth");
        let expected: f64 = height.parse().expect("height");
        let corner: TileCorner = name.parse().expect("a name");
        let tile = tiles
            .iter()
            .find(|tile| tile.corner() == corner)
            .expect("held");
        let ours = tile.height_metres(position).expect("in the tile");
        worst = worst.max((ours - expected).abs());
        // Through the many-tile terrain, where the tile is found by position;
        // on the north and east edges that is the neighbour, not held.
        if let Ok(height) = terrain.elevation(position) {
            assert!((height.value().metres() - ours).abs() < 1e-9);
        }
        checked += 1;
    }
    assert_eq!(checked, 10_000);
    assert!(worst < 1e-6, "worst {worst} m");
}
