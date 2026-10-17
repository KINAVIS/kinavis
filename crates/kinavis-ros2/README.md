# kinavis-ros2

ROS 2 for the [KINAVIS](https://github.com/KINAVIS/kinavis) crates:
`NavSatFix`, `Imu`, `Odometry` and `GeoPoint` mirrors with the conversions
between the kernel's NED/FRD frames and ROS's ENU/FLU, and covariances.

```rust
use kinavis_kernel::{Angle, TrueCourse};
use kinavis_ros2::orientation;

// Heading north, level: in ROS, a quarter turn about up (yaw 90°).
let q = orientation(Angle::ZERO, Angle::ZERO, TrueCourse::new(0.0)?);
assert!((q.z - core::f64::consts::FRAC_1_SQRT_2).abs() < 1e-12);
# Ok::<(), kinavis_kernel::KernelError>(())
```

- Vectors NED → ENU and FRD → FLU; orientation of FLU in ENU (REP-103).
- Frames `earth`, `map`, `odom`, `base_link` (REP-105).
- Covariances from an error ellipse and sigmas only; `−1` for an unknown
  `Imu` orientation; height above the ellipsoid only.
- No ROS dependency: plain structs a node copies into its messages. The
  nodes, on `rclrs`, are in `KINAVIS/ros2-kinavis`.

Checked: orientation against the rotation matrix built from the frames'
definitions and the angle rule roll, −pitch, 90° − heading; every
conversion there and back. `no_std`, no allocation.

## Feature flags

- `std` *(default)* — standard library maths in the kernel.
- `libm` — for `no_std` targets: `--no-default-features --features libm`.

## License

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or [MIT license](LICENSE-MIT) at your option.

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in this crate by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.
