# kinavis-agri

Field guidance for the [KINAVIS](https://github.com/KINAVIS/kinavis) crates:
a low-distortion field plane, AB lines, curves and pivot circles, parallel
passes, tilt compensation and pass trackers.

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
  outside bends, loops inside cut; a pass that parts into pieces, round a
  bend of more than half a turn, is reported, not joined up.
- **Tracker**: hysteresis between passes, a lock, and events: pass changed,
  end of pass approaching, off line.
- **Tilt compensation**: the control point under a rolled and pitched
  antenna, with a full lever arm.
- **Steering**: pure pursuit (look-ahead growing with speed, an optional
  intercept angle for distant passes) and Stanley, both behind one
  `PathTracker` role, forward and in reverse; wheelbase, wheel-angle and
  steering-rate limits; the kinematic bicycle. The output is a wheel angle to command, not control of
  the machine.
- **Turns**: Dubins paths, all six words, as U-turns or omega turns onto
  the next pass; followed by the same trackers; checked against the field
  for the machine's width. Reeds–Shepp paths, all 48 words, for turns that
  reverse: the three-point turn onto a close pass, with its cusps and the
  gear of each piece; each stretch between cusps a reference line the
  trackers drive in its gear.
- **Coverage and sections**: a two-bit raster of worked and overlapped
  ground, marked by the sections that are on; section control ahead of
  worked ground and the field's edge, with the valves' on and off delays.
- **Estimation without an INS**: `BicycleMotion`, the kinematic bicycle as
  the `kinavis` estimator's process model, with a log of wheel angles so a
  late fix is replayed with the angles of its time.

Checked: plane distances against the geodesic and against straight lines
between points at the field's height (ECEF); the centred plane against PROJ
(pyproj); cross-track for lines and circles against closed formulas; pass
numbers at the boundaries between passes; parallel curves against the exact
offset they stand for, and offsetting there and back; a pass on a side slope
with a noisy roll sensor; both trackers step for step against PythonRobotics,
and in closed loop on straight and curved passes and a change of pass at
working speeds, forward and in reverse; worked area against the exact strip on straight and curved passes, passes
edge to edge without gap or overlap, sections switching within a step of
worked ground and the field's edge; Dubins paths against Walker's library (3000 cases) and the U and omega
turns against their closed forms; Reeds–Shepp paths against OMPL (3600 cases, 600 of them headland turns),
never longer than Dubins, the machine moving the way each piece's gear says,
a fishtail driven through its cusps; the bicycle model against fine integration and finite
differences, and the estimator through a headland turn with six seconds
without fixes and with a late receiver; the same results with `std` and
`libm`; stored jobs read back equal to themselves, and what the
constructors refuse is refused on reading, fuzzed. `no_std`, no
allocation without `serde`.

## Keeping a job

With the `serde` feature a field, its guidance lines, the pass pattern, the
boom, the machine's geometry and the tracker's thresholds serialise, so a job
outlasts the session. Lengths are in metres, planar points `[east, north]`;
a centred plane is its origin and scale:

```json
{"centred":{"origin":{"latitude":51.985,"longitude":5.66},"scale":1.0000015}}
```

Reading goes through the constructors' checks: a curve that crosses itself
or a boom with the wrong number of sections is an error, not a value. A grid
plane is stored by its datum's name and reads only on one of the `Datum`
presets.

## Feature flags

- `std` *(default)* — standard library maths.
- `libm` — for `no_std` targets: `--no-default-features --features libm`.
- `serde` — serialisation, as above. Needs an allocator.

## License

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or [MIT license](LICENSE-MIT) at your option.

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in this crate by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.
