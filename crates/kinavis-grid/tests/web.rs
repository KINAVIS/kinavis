//! Web Mercator and tiles against two other implementations, from
//! `tools/reference.py`: EPSG:3857 both ways against PROJ (5030 points), and
//! tiles, quadkeys and tile edges against mercantile (4769 points, zooms 0
//! to 30); then the edges, the round trips and the tile tree.

#![allow(clippy::expect_used, clippy::panic, clippy::float_cmp)]

use kinavis_grid::{
    metres_per_pixel, Quadkey, Tile, TilePoint, WebMercator, MAX_ZOOM, TILE_PIXELS,
    WEB_MERCATOR_MAX_LATITUDE, WEB_MERCATOR_RADIUS,
};
use kinavis_kernel::{Distance, KernelError, Latitude, Position};
use proptest::prelude::*;

const WEB_MERCATOR: &str = include_str!("data/web_mercator_reference.txt");
const TILES: &str = include_str!("data/tiles_reference.txt");

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

fn position(lat: f64, lon: f64) -> Position {
    Position::from_degrees(lat, lon).expect("on Earth")
}

#[test]
fn web_mercator_agrees_with_proj_both_ways() {
    let (mut forward, mut inverse) = (0, 0);
    for row in rows(WEB_MERCATOR) {
        match row[..] {
            ["forward", lat, lon, x, y] => {
                let (lat, lon) = (number(lat), number(lon));
                let point = WebMercator::from_position(position(lat, lon)).expect("on the map");
                assert!(
                    (point.x.metres() - number(x)).abs() < 1e-6,
                    "x at {lat} {lon}: {}",
                    point.x.metres()
                );
                assert!(
                    (point.y.metres() - number(y)).abs() < 1e-6,
                    "y at {lat} {lon}: {}",
                    point.y.metres()
                );
                forward += 1;
            }
            ["inverse", x, y, lat, lon] => {
                let point =
                    WebMercator::new(metres(number(x)), metres(number(y))).expect("on the map");
                let back = point.to_position();
                assert!(
                    (back.latitude().degrees() - number(lat)).abs() < 1e-12,
                    "latitude at {x} {y}"
                );
                assert!(
                    (back.longitude().degrees() - number(lon)).abs() < 1e-12,
                    "longitude at {x} {y}"
                );
                inverse += 1;
            }
            _ => panic!("a forward or inverse row: {row:?}"),
        }
    }
    assert_eq!((forward, inverse), (3030, 2000));
}

#[test]
fn tiles_quadkeys_and_edges_agree_with_mercantile() {
    let mut checked = 0;
    for row in rows(TILES) {
        let [lat, lon, zoom, x, y, key, west, south, east, north] = row[..] else {
            panic!("ten columns: {row:?}");
        };
        let (lat, lon) = (number(lat), number(lon));
        let zoom: u8 = zoom.parse().expect("zoom");
        let at = Tile::containing(position(lat, lon), zoom).expect("on the map");
        let expected =
            Tile::new(zoom, x.parse().expect("x"), y.parse().expect("y")).expect("a tile");
        assert_eq!(at.tile, expected, "tile of {lat} {lon} at zoom {zoom}");
        let key = if key == "-" { "" } else { key };
        assert_eq!(at.tile.quadkey().as_str(), key);
        assert_eq!(key.parse::<Quadkey>().expect("a quadkey").tile(), expected);
        let bounds = expected.bounds();
        for (ours, theirs, edge) in [
            (bounds.west, west, "west"),
            (bounds.south, south, "south"),
            (bounds.east, east, "east"),
            (bounds.north, north, "north"),
        ] {
            assert!(
                (ours - number(theirs)).abs() < 1e-12,
                "{edge} edge of {expected:?}: {ours} against {theirs}"
            );
        }
        assert!(
            bounds.west <= lon && lon < bounds.east && bounds.south < lat && lat <= bounds.north,
            "{lat} {lon} outside {expected:?}: {bounds:?}"
        );
        checked += 1;
    }
    assert_eq!(checked, 4769);
}

#[test]
fn the_map_is_square_and_ends_at_its_edges() {
    let top = WebMercator::from_position(position(WEB_MERCATOR_MAX_LATITUDE, -180.0))
        .expect("the corner");
    let half = core::f64::consts::PI * WEB_MERCATOR_RADIUS;
    assert!((top.y.metres() - half).abs() < 1e-6);
    assert_eq!(top.x.metres(), -half);
    for latitude in [85.06, -85.06, 90.0] {
        assert!(matches!(
            WebMercator::from_position(position(latitude, 0.0)),
            Err(KernelError::OutOfRange { .. })
        ));
        let clamped = WebMercator::from_position_clamped(position(latitude, 0.0));
        assert!((clamped.y.metres().abs() - half).abs() < 1e-6);
        assert!(Tile::containing(position(latitude, 0.0), 3).is_err());
    }
    assert!(WebMercator::new(metres(half + 1.0), metres(0.0)).is_err());
    assert!(WebMercator::new(metres(0.0), metres(-half - 1.0)).is_err());

    // The corners of the map at zoom 2: the top left tile, the bottom right
    // one with its pixel on the far edge.
    let corner =
        Tile::containing(position(WEB_MERCATOR_MAX_LATITUDE, -180.0), 2).expect("a corner");
    assert_eq!((corner.tile.x(), corner.tile.y()), (0, 0));
    assert_eq!(corner.pixel_x, 0.0);
    assert!(corner.pixel_y < 1e-9);
    let bottom = Tile::containing(position(-WEB_MERCATOR_MAX_LATITUDE, 179.999_999_999), 2)
        .expect("a corner");
    assert_eq!((bottom.tile.x(), bottom.tile.y()), (3, 3));
    assert!((bottom.pixel_y - f64::from(TILE_PIXELS)).abs() < 1e-6);
    let last = Tile::new(2, 3, 3).expect("a tile").bounds();
    assert_eq!(last.east, 180.0);
    assert!((last.south + WEB_MERCATOR_MAX_LATITUDE).abs() < 1e-12);
}

#[test]
fn indices_and_zooms_beyond_the_scheme_are_refused() {
    assert!(Tile::new(MAX_ZOOM, (1 << MAX_ZOOM) - 1, 0).is_ok());
    assert!(Tile::new(MAX_ZOOM + 1, 0, 0).is_err());
    assert!(Tile::new(3, 8, 0).is_err());
    assert!(Tile::new(3, 0, 8).is_err());
    assert!(Tile::containing(position(0.0, 0.0), MAX_ZOOM + 1).is_err());
    assert!(metres_per_pixel(Latitude::EQUATOR, MAX_ZOOM + 1).is_err());
    for bad in ["4", "0124", "a", " 0", &"0".repeat(31)] {
        assert!(bad.parse::<Quadkey>().is_err(), "{bad:?}");
    }
    let longest = "3".repeat(30).parse::<Quadkey>().expect("30 digits");
    assert_eq!(
        longest.tile(),
        Tile::new(30, (1 << 30) - 1, (1 << 30) - 1).expect("a tile")
    );
    assert_eq!(
        "".parse::<Quadkey>().expect("empty").tile(),
        Tile::new(0, 0, 0).expect("the world")
    );
}

#[test]
fn the_ground_resolution_halves_per_level_and_with_the_cosine() {
    // 2π · 6 378 137 / 256 m per pixel at zoom 0 on the equator.
    let equator = metres_per_pixel(Latitude::EQUATOR, 0).expect("zoom 0");
    assert!((equator.metres() - 156_543.033_928).abs() < 1e-5);
    let fifteen = metres_per_pixel(Latitude::EQUATOR, 15).expect("zoom 15");
    assert!((fifteen.metres() - 4.777_314).abs() < 1e-6);
    let sixty = metres_per_pixel(Latitude::from_degrees(60.0).expect("60°"), 15).expect("zoom 15");
    assert!((sixty.metres() - fifteen.metres() / 2.0).abs() < 1e-9);
}

#[test]
fn the_tile_tree_nests() {
    let tile = Tile::new(5, 17, 10).expect("a tile");
    let children = tile.children().expect("children");
    for (index, child) in children.iter().enumerate() {
        assert_eq!(child.parent(), Some(tile));
        assert_eq!(
            child.quadkey().as_str(),
            format!("{}{index}", tile.quadkey())
        );
        let (outer, inner) = (tile.bounds(), child.bounds());
        assert!(outer.west <= inner.west && inner.east <= outer.east);
        assert!(outer.south <= inner.south && inner.north <= outer.north);
    }
    assert_eq!(Tile::new(0, 0, 0).expect("the world").parent(), None);
    assert_eq!(Tile::new(MAX_ZOOM, 0, 0).expect("deepest").children(), None);
}

proptest! {
    #[test]
    fn a_position_comes_back_through_its_tile_pixel(
        lat in -WEB_MERCATOR_MAX_LATITUDE..WEB_MERCATOR_MAX_LATITUDE,
        lon in -180.0f64..180.0,
        zoom in 0u8..=MAX_ZOOM,
    ) {
        let start = position(lat, lon);
        let at = Tile::containing(start, zoom).expect("on the map");
        let size = f64::from(TILE_PIXELS);
        prop_assert!((0.0..=size).contains(&at.pixel_x) && (0.0..=size).contains(&at.pixel_y));
        let back = at.to_position();
        prop_assert!((back.latitude().degrees() - lat).abs() < 1e-9);
        prop_assert!((back.longitude().degrees() - lon).abs() < 1e-9);
        // The same point read from a pixel of the parent lands in that parent.
        if let Some(parent) = at.tile.parent() {
            let up = Tile::containing(start, zoom - 1).expect("on the map");
            prop_assert_eq!(up.tile, parent);
        }
        let again = TilePoint { ..at }.to_web_mercator();
        let direct = WebMercator::from_position(start).expect("on the map");
        prop_assert!((again.x.metres() - direct.x.metres()).abs() < 1e-6);
        prop_assert!((again.y.metres() - direct.y.metres()).abs() < 1e-6);
    }

    #[test]
    fn a_quadkey_reads_back_as_its_tile(zoom in 0u8..=MAX_ZOOM, x in any::<u32>(), y in any::<u32>()) {
        let mask = (1u32 << zoom) - 1;
        let tile = Tile::new(zoom, x & mask, y & mask).expect("a tile");
        let key = tile.quadkey();
        prop_assert_eq!(key.as_str().len(), usize::from(zoom));
        prop_assert_eq!(key.as_str().parse::<Quadkey>().expect("a quadkey").tile(), tile);
    }
}
