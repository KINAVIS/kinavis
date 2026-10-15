# kinavis-isobus

ISOBUS (ISO 11783-7) for the [KINAVIS](https://github.com/KINAVIS/kinavis)
crates: the speed of the tractor and the messages between a guidance system
and the steering.

```rust
use kinavis_isobus::{GuidanceSystemCommand, Indicator};
use kinavis_j1939::Frame;

// Command a right turn of radius 20 m: 0.05 per metre.
let command = GuidanceSystemCommand { curvature: Some(0.05), intends_to_steer: Indicator::On };
let frame = Frame::of(3, GuidanceSystemCommand::PGN, 0x13, 0x1C, &command.to_bytes())?;
assert_eq!(GuidanceSystemCommand::from_bytes(command.to_bytes()), command);
# let _ = frame;
# Ok::<(), kinavis_j1939::J1939Error>(())
```

- **Speed**: wheel-based (65096) and ground-based (65097) speed and
  distance, the machine selected speed with its source (61474), the machine
  selected speed command (64835).
- **Guidance**: the steering's estimated curvature, readiness and limits
  (44032), and the curvature commanded (44288); right positive, per metre.
- Decoded and encoded; "not available" and "error" are `None` or their own
  variant.

Field layouts of ISO 11783-7, checked value for value against AgIsoStack++
(MIT), as the standard is not public; fields that could not be checked are
not decoded. Positions on ISOBUS are NMEA 2000 groups: `kinavis-nmea2000`.
Frames, address claim and transport are `kinavis-j1939`'s. `no_std`, no
allocation.

## Feature flags

- `std` *(default)* — standard library in the kernel.
- `libm` — for `no_std` targets: `--no-default-features --features libm`.
- `serde` — serialisation of the messages.

## License

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or [MIT license](LICENSE-MIT) at your option.

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in this crate by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.
