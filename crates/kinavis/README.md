# kinavis

The navigation algorithms of [KINAVIS](https://github.com/KINAVIS/kinavis):
compass corrections, the sailings, dead reckoning, position fixing, passage
planning and guidance, tides, the sun, and a Kalman filter over the
navigation state — with no panics, no `unsafe` and no allocation.

```toml
[dependencies]
kinavis = "1"
```

```rust
use kinavis::navigation_solutions::convert_compass_course_to_true_course;
use kinavis::{CompassCourse, DeviationTable, InterpolationMethod, Variation};

// A swing: deviation observed on every tenth of the compass, 000° to 350°.
let table = DeviationTable::from_deviations(&[
    -2.5, -0.5, 1.6, 4.4, -1.7, 0.0, 1.0, 0.3, -0.9, // 000°..080°
    0.5, -1.2, 0.8, -0.3, 1.7, -2.1, 0.4, -0.6, 1.2, // 090°..170°
    -1.3, 0.0, 0.9, -1.1, 1.5, -0.7, -13.2, -15.7, -17.9, // 180°..260°
    -19.2, -18.1, 1.8, -0.4, 0.7, -0.2, 1.4, -4.4, -2.9, // 270°..350°
])?;

// True course made good steering 003° by compass, variation 2.7°W.
let solution = convert_compass_course_to_true_course(
    CompassCourse::new(3.0)?,
    Variation::new(-2.7)?,
    &table,
    InterpolationMethod::Cubic,
)?;
assert_eq!(format!("{}", solution.course), "358.2°T");
# Ok::<(), kinavis::NavigationError>(())
```

A compass course cannot be passed where a true one belongs, and knots cannot
be mistaken for metres per second: every angle carries its reference frame in
the type, and every quantity its unit.

## What is inside

| Area | Modules |
|---|---|
| Value types (from `kinavis-kernel`) | `angle`, `units`, `position`, `time`, `geodesy`, `local`, `observation`, `gnss`, `event`, `environment` |
| The compass | `deviation`, `navigation_solutions` |
| Position | `sailings`, `dead_reckoning`, `fix`, `gnss_intake` |
| Passage | `route`, `turning`, `guidance`, `schedule`, `composite`, `clearance` |
| Surroundings | `relative_motion`, `anchor`, `mob`, `conditions`, `tides`, `sun` |
| Estimation | `estimator`, `observations`, `state`, `snapshot` |

The [guide](https://docs.rs/kinavis/latest/kinavis/guide/index.html) walks
through all of it with examples that are compiled and run as tests.

## Features

| Feature | Default | What it does |
|---|---|---|
| `std` | yes | the standard library's floating point maths; implies `alloc` |
| `alloc` | via `std` | `Vec`-returning companions of the `*_into` calls |
| `libm` | | pure-Rust maths for `no_std` targets |
| `serde` | | serialisation, with deserialisation through the constructors |

For a bare-metal target with no allocator:

```toml
kinavis = { version = "1", default-features = false, features = ["libm"] }
```

## License

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or [MIT license](LICENSE-MIT) at your option.

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in this crate by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.
