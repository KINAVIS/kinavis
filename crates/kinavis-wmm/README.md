# kinavis-wmm

World Magnetic Model as a magnetic model port for the
[KINAVIS](https://github.com/KINAVIS/kinavis) crates.

Embeds WMM2025 (valid 2025.0–2030.0) behind the `wmm2025` feature and implements
`kinavis_kernel::environment::MagneticModel`: position, height and time in;
north, east and down field components out, with declination derived. Outside the
validity interval it returns an error instead of extrapolating.

Verified against NOAA's hundred published test values. `no_std`, no allocation.

```rust
use kinavis_kernel::environment::MagneticModel;
use kinavis_kernel::{Civil, Distance, GeodeticPoint, Height, Instant, Position, Utc};
use kinavis_wmm::Wmm;

let point = GeodeticPoint::new(
    Position::from_degrees(48.5, -5.5)?,
    Height::above_mean_sea_level(Distance::ZERO),
);
let when = Instant::<Utc>::from_civil(Civil::date(2026, 6, 21))?;
let field = Wmm::WMM2025.field_at(point, when)?;
println!("variation {}", field.declination());
# Ok::<(), kinavis_kernel::KernelError>(())
```

The navigation algorithms and the list of the other KINAVIS crates are in
[`kinavis`](https://crates.io/crates/kinavis).

## Feature flags

- `std` *(default)* — standard library maths in the kernel.
- `libm` — for `no_std` targets: `--no-default-features --features libm`.
- `wmm2025` *(default)* — embeds the WMM2025 coefficients.

## License

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or [MIT license](LICENSE-MIT) at your option.

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in this crate by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.
