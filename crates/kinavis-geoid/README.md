# kinavis-geoid

Geoid heights from a global grid, as a geoid model port for the
[KINAVIS](https://github.com/KINAVIS/kinavis) crates.

Implements `kinavis_kernel::geodesy::GeoidModel`, so a GNSS height above the
ellipsoid becomes a height above mean sea level and back:

```rust
use kinavis_geoid::GridGeoid;
use kinavis_kernel::{Distance, GeodeticPoint, Height, Position};

let antenna = GeodeticPoint::new(
    Position::from_degrees(50.76, -1.3)?,
    Height::above_ellipsoid(Distance::from_metres(52.0)?),
);
let above_sea = antenna.to_mean_sea_level(&GridGeoid::EGM96)?;
println!("{}", above_sea.height());
# Ok::<(), kinavis_kernel::KernelError>(())
```

`GridGeoid` reads any global grid in GeographicLib's PGM layout from a borrowed
byte slice — embedded, read from a file or memory-mapped — without copying or
allocating, and interpolates it bilinearly or with GeographicLib's twelve-point
cubic fit. The `egm96` feature embeds EGM96 on a 15′ grid (2 MB).

Checked against GeographicLib's `GeoidEval` (both interpolations, to its 0.1 mm
output) and PROJ (bilinear, on NGA's unquantised grid) at 2020 points; the
header parser is fuzzed. `no_std`, no allocation.

## Data

`data/egm96-15.pgm` is NGA's EGM96 15-minute geoid grid (WW15MGH), public
domain, taken from PROJ-data as `us_nga_egm96_15.tif` and stored in 3 mm steps
by `tools/egm96_pgm.py`, which checks the source's SHA-256. The cubic
coefficients are GeographicLib's (C. Karney, MIT licence).

## Feature flags

- `std` *(default)* — standard library maths in the kernel.
- `libm` — for `no_std` targets: `--no-default-features --features libm`.
- `egm96` *(default)* — embeds the EGM96 15′ grid as `GridGeoid::EGM96`.

## License

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or [MIT license](LICENSE-MIT) at your option.

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in this crate by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.
