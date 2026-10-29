# kinavis-land

Land navigation for the [KINAVIS](https://github.com/KINAVIS/kinavis) crates:
bearings on a map grid, mils, intersection and resection, pace counting and
routes by bearing.

```rust
use kinavis_land::{three_point_resection, GridPoint};
use kinavis_kernel::Angle;

// Directions to three landmarks read on a compass 3° off grid north, from
// the point 2000 E 1500 N.
let landmarks = [
    GridPoint::from_metres(1_000.0, 3_000.0)?,
    GridPoint::from_metres(4_000.0, 2_500.0)?,
    GridPoint::from_metres(2_500.0, -500.0)?,
];
let readings = [323.3099, 60.4349, 162.9638].map(|deg| Angle::from_degrees(deg).unwrap());
let fix = three_point_resection(landmarks, readings)?;
assert_eq!(format!("{:.0} {:.0}", fix.point.easting.metres(), fix.point.northing.metres()), "2000 1500");
assert_eq!(format!("{:.1}", fix.orientation.degrees()), "3.0");
# Ok::<(), kinavis_land::LandError>(())
```

- **Bearings**: grid, true and magnetic at a point from the meridian
  convergence (`kinavis-grid`) and the declination (`kinavis-wmm`); the grid
  magnetic angle of a map's margin; mils of 6000 (Soviet and Russian) and
  6400 (NATO).
- **Map grid**: one UTM zone, points of the next zone projected into it, or
  a projected system — Gauss–Krüger on СК-42, a national grid.
- **Intersection** from two known points, **resection** by bearings to two
  or more landmarks (least squares, with residuals), **three-point
  resection** (Snellius–Pothenot, Tienstra's formula) from directions with
  an unknown zero, which also yields the compass error; the danger circle
  is refused.
- **Pace count**: pairs of steps calibrated on a known course, reduced to
  the horizontal on a slope.
- **Route by bearings**: legs of grid or magnetic bearing and ground distance
  laid out on the grid as a `kinavis` route, and the card of a route back.
- **Map matching**: the road of each fix of a drive by the hidden Markov
  model of Newson and Krumm — `K` candidates a fix, Viterbi over a fixed
  window of `W` fixes, a break and a new chain where no candidate is
  reachable — over a `RoadNetwork` the application provides (its index and
  its router).

The plane problems are checked on constructed figures — inside and outside
the triangle, both orientations, near the danger circle — against their
known solutions, and the resection against the marine fix of `kinavis`.
The map matcher follows Newson and Krumm's own drive through Seattle (7531
fixes at 1 Hz; data from Microsoft Research, fetched, not in the
repository): 99.83 % of the fixes it keeps on the road driven, every one
the right way, no break.
`no_std`, no allocation.

## Feature flags

- `std` *(default)* — standard library maths.
- `libm` — for `no_std` targets: `--no-default-features --features libm`.

## License

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or [MIT license](LICENSE-MIT) at your option.

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in this crate by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.
