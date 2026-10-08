# kinavis-grid

Map grids for the [KINAVIS](https://github.com/KINAVIS/kinavis) crates:
transverse Mercator, UTM and UPS, Gauss–Krüger zones and national grids, with
the meridian convergence and point scale at every point.

```rust
use kinavis_grid::{ProjectedCrs, UtmUps};
use kinavis_kernel::Position;

let fix = Position::from_degrees(55.7539, 37.6208)?;
let utm = UtmUps::from_position(fix)?;          // zone 37N
let sk42 = ProjectedCrs::pulkovo_1942(7)?.project(fix)?; // СК-42, zone 7
println!("{:?} {:.0} {:.0}", utm.coordinate, sk42.easting.metres(), sk42.northing.metres());
# Ok::<(), kinavis_kernel::KernelError>(())
```

- **Transverse Mercator**: Krüger's series to sixth order (Karney 2011),
  transcribed from GeographicLib; about 5 nm within 3900 km of the central
  meridian. Forward and inverse, convergence and scale.
- **UTM/UPS** on WGS 84: standard zones with the Norway and Svalbard
  exceptions, UPS beyond 84°N and 80°S, coordinate limits checked.
- **Gauss–Krüger** in 6° and 3° zones on any datum, and presets: СК-42, СК-95,
  ГСК-2011, DHDN, the British National Grid. A WGS 84 fix goes through the
  kernel's datum shift first.
- **MGRS**: references at 0 to 11 digits (100 km to 1 µm), truncating as the
  system requires, and read back to the centre of the square; UPS lettering
  near the poles. Lettering of WGS 84 and other modern ellipsoids.
- **Grid north**: the convergence turns a true course into a grid course
  (`TrueCourse::to_grid`); `grid_magnetic_angle` gives the angle in a map's
  margin.

Checked against GeographicLib's `GeoConvert` (UTM/UPS, convergence and scale,
to 10 µm, 4649 points; MGRS references and squares, 5400 points) and PROJ
(every preset, to 1 mm, 10 000 points); the MGRS reader is fuzzed. `no_std`,
no allocation.

## Feature flags

- `std` *(default)* — standard library maths in the kernel.
- `libm` — for `no_std` targets: `--no-default-features --features libm`.

## License

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or [MIT license](LICENSE-MIT) at your option.

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in this crate by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.
