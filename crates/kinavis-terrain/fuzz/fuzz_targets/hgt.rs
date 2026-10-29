//! Fuzz target: a tile name and an HGT tile from arbitrary bytes, read at
//! arbitrary positions.
//!
//! Input: a name length byte, the name, two `f64` (latitude and longitude
//! fractions), then the tile bytes. Invariants: no panic; a name that reads
//! prints back the same, upper case; a tile that reads gives heights within
//! the range of `i16` or an error, and a profile and a sight line over it
//! end without panic.
#![no_main]

use libfuzzer_sys::fuzz_target;

use kinavis_kernel::{Distance, ElevationModel, Height, Position};
use kinavis_terrain::{line_of_sight, profile, HgtTile, ProfilePoint, TileCorner};

fuzz_target!(|data: &[u8]| {
    let [length, rest @ ..] = data else {
        return;
    };
    let (name, rest) = rest.split_at(usize::from(*length % 16).min(rest.len()));
    if let Ok(text) = core::str::from_utf8(name) {
        if let Ok(corner) = text.parse::<TileCorner>() {
            let printed = corner.to_string();
            assert_eq!(printed.parse::<TileCorner>().ok(), Some(corner));
        }
    }
    let (Some(a), Some(b)) = (rest.get(..8), rest.get(8..16)) else {
        return;
    };
    let u = f64::from_le_bytes(a.try_into().unwrap_or([0; 8]));
    let v = f64::from_le_bytes(b.try_into().unwrap_or([0; 8]));
    let bytes = rest.get(16..).unwrap_or(&[]);
    let Ok(corner) = TileCorner::new(46, 7) else {
        return;
    };
    let Ok(tile) = HgtTile::new(bytes, corner) else {
        return;
    };
    let Ok(at) = Position::from_degrees(46.0 + u.fract().abs(), 7.0 + v.fract().abs()) else {
        return;
    };
    if let Ok(height) = tile.elevation(at) {
        let metres = height.value().metres();
        assert!((-32_768.0..=32_767.0).contains(&metres), "{metres}");
    }
    let Ok(other) = Position::from_degrees(46.0 + v.fract().abs(), 7.0 + u.fract().abs()) else {
        return;
    };
    let mut points = [ProfilePoint {
        distance: Distance::ZERO,
        position: at,
        elevation: Height::above_mean_sea_level(Distance::ZERO),
    }; 16];
    let _ = profile(&tile, at, other, &mut points);
    let two = Distance::from_metres(2.0).unwrap_or(Distance::ZERO);
    let _ = line_of_sight(&tile, at, two, other, two, 64, 0.13);
});
