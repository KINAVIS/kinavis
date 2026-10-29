# kinavis-terrain

Terrain for the [KINAVIS](https://github.com/KINAVIS/kinavis) crates: SRTM
height tiles over borrowed bytes, slope and aspect, profiles, line of sight
and walking time.

```rust
use kinavis_kernel::{Distance, ElevationModel, Position};
use kinavis_terrain::{line_of_sight, HgtTile, TileCorner, STANDARD_REFRACTION};

// The bytes of an .hgt file, read or mapped by the application.
let rows: [[i16; 3]; 3] = [[0, 0, 0], [0, 500, 0], [0, 0, 0]];
let bytes: Vec<u8> = rows.iter().flatten().flat_map(|h| h.to_be_bytes()).collect();
let tile = HgtTile::new(&bytes, "N46E007".parse::<TileCorner>()?)?;

// A 500 m hill between two walkers 2 m tall.
let west = Position::from_degrees(46.5, 7.05)?;
let east = Position::from_degrees(46.5, 7.95)?;
let two = Distance::from_metres(2.0)?;
let sight = line_of_sight(&tile, west, two, east, two, 1000, STANDARD_REFRACTION)?;
assert!(!sight.visible);
# Ok::<(), kinavis_kernel::KernelError>(())
```

- **Tiles**: SRTM `.hgt` at 1″ (3601²), 3″ (1201²) or any square
  resolution, read in place from a borrowed slice; the corner from the file
  name; bilinear heights above the EGM96 geoid; voids (−32768) refused.
- **Many tiles**: `TileSource`, a role the application fills, and
  `Terrain` over it — an `ElevationModel`, the kernel's port, which any
  other model can fill as well.
- **Slope and aspect** by Horn's method, the weighting of `gdaldem`.
- **Profiles** along the chord between two points: ascent, descent,
  extremes, with a noise threshold.
- **Line of sight** through the Earth-fixed frame — the Earth's curvature
  exact — with standard refraction, and where the line comes closest to the
  ground.
- **Walking time**: Naismith's rule, or one's own level speed and climbing
  rate.

Checked against GDAL 3.12 (through rasterio) on two SRTM 1″ tiles, Elbrus
and Moscow, 10 000 points, bilinear on GDAL's own georeferencing, to 2·10⁻⁷ m
(the test reads the tiles from `SRTM_TILES`; they are not in the repository);
slope and aspect on analytic surfaces; line of sight against dense
sampling. The tile reader is fuzzed. `no_std`, no allocation.

Data: SRTM is in the public domain (USGS). The 1″ tiles come, for example,
from the Terrain Tiles of Mapzen on AWS, `s3://elevation-tiles-prod/skadi/`.

## Feature flags

- `std` *(default)* — standard library maths.
- `libm` — for `no_std` targets: `--no-default-features --features libm`.

## License

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or [MIT license](LICENSE-MIT) at your option.

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in this crate by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.
