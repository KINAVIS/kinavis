# kinavis-agri

Field guidance for the [KINAVIS](https://github.com/KINAVIS/kinavis) crates:
a low-distortion field plane, AB lines, curves and pivot circles, parallel
passes and tilt compensation.

```rust
use kinavis::region::PlanarPoint;
use kinavis_agri::{AbLine, FieldPlane, PassPattern, ReferenceLine};
use kinavis_kernel::{Distance, Height, Position};

let a = Position::from_degrees(52.2560, 21.0100)?;
let b = Position::from_degrees(52.2600, 21.0110)?;
let plane = FieldPlane::centred_at(a, Height::above_ellipsoid(Distance::from_metres(110.0)?))?;
let line = AbLine::from_positions(plane, a, b)?;
let passes = PassPattern::new(Distance::from_metres(24.0)?, Distance::from_metres(0.3)?)?;

let here = plane.to_plane(Position::from_degrees(52.2580, 21.0120)?)?;
let offset = line.station(here).offset;
println!("{} at {:.2} m from the line", passes.pass_at(offset), offset.metres());
# Ok::<(), kinavis_agri::AgriError>(())
```

- **Field plane**: a transverse Mercator through the field, scaled to the
  field's height above the ellipsoid; plane distances equal ground distances
  to 3·10⁻⁷ at 5 km, 5·10⁻⁶ at the 20 km limit. Zoned grids (UTM,
  Gauss–Krüger) for data that arrives in them. Grid and true directions
  through the meridian convergence.
- **Reference lines**: AB lines (two points, or a point and a course),
  curves of up to `N` vertices, pivot circles; recorded passes become curves.
- **Passes**: working width, overlap, shift and nudge; the pass at a point,
  cross-track error with its side, heading error, distance to the field
  boundary ahead. Passes of a curve are its exact parallel curves: arcs
  outside bends, loops inside cut.
- **Tracker**: hysteresis between passes, a lock, and events: pass changed,
  end of pass approaching, off line.
- **Tilt compensation**: the control point under a rolled and pitched
  antenna, with a full lever arm.

Checked: plane distances against the geodesic and against straight lines
between points at the field's height (ECEF); the centred plane against PROJ
(pyproj); cross-track for lines and circles against closed formulas; pass
numbers at the boundaries between passes; parallel curves against the exact
offset they stand for, and offsetting there and back; a pass on a side slope
with a noisy roll sensor. `no_std`, no allocation.

## Feature flags

- `std` *(default)* — standard library maths.
- `libm` — for `no_std` targets: `--no-default-features --features libm`.

## License

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or [MIT license](LICENSE-MIT) at your option.

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in this crate by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.
