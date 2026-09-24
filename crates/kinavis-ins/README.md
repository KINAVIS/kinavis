# kinavis-ins

Strapdown inertial navigation over
[kinavis-kernel](https://crates.io/crates/kinavis-kernel): NED mechanisation of
an IMU and a 15-state error-state Kalman filter.

- `Strapdown` — attitude, velocity and position integrated from gyros and
  accelerometers, including Earth rate, transport rate, gravity and Coriolis.
- `InsFilter` — position, velocity, attitude, gyro-bias and accelerometer-bias
  errors estimated from GNSS position or velocity, heading, or zero-velocity
  updates, and fed back into the mechanisation. Outputs position with error
  ellipse, velocity, roll/pitch/yaw with sigmas; no covariance matrix.
- `InsMotion` — the INS as a process model for the six-state estimator in
  `kinavis`.
- No allocation, no panics. Filter consistency verified by Monte Carlo tests:
  NEES and NIS within χ² intervals.

```rust
use core::time::Duration;
use kinavis_ins::{
    gravity_down, GatingPolicy, ImuNoise, ImuSample, InsFilter, InsPriors, Quaternion,
    Strapdown, EARTH_RATE,
};
use kinavis_kernel::{
    Angle, Distance, GeodeticPoint, Height, Instant, Ned, Speed, TrueCourse, Utc, Vector3,
};

// Alongside at 50°45.3'N, heading 037° by the gyrocompass, level.
let start = Instant::<Utc>::from_unix_seconds(1_789_000_000);
let berth = GeodeticPoint::new(
    "50°45.3'N 001°20.0'W".parse()?,
    Height::above_ellipsoid(Distance::ZERO),
);
let attitude = Quaternion::from_euler(Angle::ZERO, Angle::ZERO, TrueCourse::new(37.0)?);
let still = Vector3::<Ned, Speed>::new(Speed::ZERO, Speed::ZERO, Speed::ZERO);
let mut ins = InsFilter::new(
    Strapdown::new(start, berth, still, attitude)?,
    ImuNoise::mems(),
    &InsPriors::standard(),
);

// What the IMU reads at rest: the Earth turning, and minus gravity.
let latitude = 50.755_f64.to_radians();
let earth = [EARTH_RATE * latitude.cos(), 0.0, -EARTH_RATE * latitude.sin()];
let gravity = [0.0, 0.0, -gravity_down(latitude.sin(), 0.0)];
let sample = ImuSample::new(
    attitude.rotate_back(earth),
    attitude.rotate_back(gravity),
    Duration::from_millis(100),
)?;

for _ in 0..10 {
    ins.predict(&sample)?;
}
ins.update_heading(TrueCourse::new(37.2)?, Angle::from_degrees(0.5)?, GatingPolicy::none())?;
ins.update_zero_velocity(Speed::from_metres_per_second(0.02)?, GatingPolicy::none())?;

assert!((ins.attitude().yaw.degrees() - 37.2).abs() < 0.1);
assert!(ins.heading_sigma().degrees() < 0.5);
# Ok::<(), Box<dyn std::error::Error>>(())
```

The navigation algorithms and the list of the other KINAVIS crates are in
[`kinavis`](https://crates.io/crates/kinavis).

## Feature flags

- `std` *(default)* — standard library maths in the kernel.
- `libm` — for `no_std` targets: `--no-default-features --features libm`.
- `serde` — serialisation of the value types.

## License

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or [MIT license](LICENSE-MIT) at your option.

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in this crate by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.
