//! Fuzz target: a quadkey read from arbitrary text, and tiles of arbitrary
//! positions and Web Mercator points.
//!
//! Input: a zoom byte, two `f64` (latitude and longitude, or x and y), then
//! text. Invariants: no panic; a quadkey that reads names a tile whose key is
//! the same text; a position on the map lands in a tile whose edges hold it,
//! at a pixel within the tile, and comes back from that pixel.
#![no_main]

use libfuzzer_sys::fuzz_target;

use kinavis_grid::{Quadkey, Tile, WebMercator, TILE_PIXELS};
use kinavis_kernel::{Distance, Position};

fuzz_target!(|data: &[u8]| {
    let [zoom, rest @ ..] = data else {
        return;
    };
    let (Some(a), Some(b)) = (rest.get(..8), rest.get(8..16)) else {
        return;
    };
    let a = f64::from_le_bytes(a.try_into().unwrap_or([0; 8]));
    let b = f64::from_le_bytes(b.try_into().unwrap_or([0; 8]));
    let zoom = zoom % 32;
    if let Ok(text) = core::str::from_utf8(rest.get(16..).unwrap_or(&[])) {
        if let Ok(key) = text.parse::<Quadkey>() {
            assert_eq!(key.tile().quadkey().as_str(), text);
        }
    }
    if let Ok(position) = Position::from_degrees(a, b) {
        if let Ok(at) = Tile::containing(position, zoom) {
            let size = f64::from(TILE_PIXELS);
            assert!((0.0..=size).contains(&at.pixel_x) && (0.0..=size).contains(&at.pixel_y));
            let bounds = at.tile.bounds();
            let (lat, lon) = (position.latitude().degrees(), position.longitude().degrees());
            assert!(bounds.west - 1e-9 <= lon && lon <= bounds.east + 1e-9);
            assert!(bounds.south - 1e-9 <= lat && lat <= bounds.north + 1e-9);
            let back = at.to_position();
            assert!((back.latitude().degrees() - lat).abs() < 1e-6);
        }
    }
    if let (Ok(x), Ok(y)) = (Distance::from_metres(a), Distance::from_metres(b)) {
        if let Ok(point) = WebMercator::new(x, y) {
            let _ = Tile::at(point, zoom);
            let _ = WebMercator::from_position(point.to_position());
        }
    }
});
