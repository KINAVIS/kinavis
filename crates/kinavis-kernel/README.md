# kinavis-kernel

Value types of the [KINAVIS](https://github.com/KINAVIS/kinavis) crates:
frame-tagged angles, units, positions, time scales, geodesy and chart datums,
events, the error type, and the floating-point primitives.

The kernel holds what has exactly one correct implementation. Algorithms
(sailings, fixes, dead reckoning, deviation) are in `kinavis`; every satellite
crate (sensor adapters, environment models, INS) depends on the kernel alone, so
adapters change without touching the core.

Most users should depend on [`kinavis`](https://crates.io/crates/kinavis),
which re-exports these types and lists the other KINAVIS crates. Depend on
`kinavis-kernel` directly for an adapter that must not pull in the algorithms.

## Feature flags

- `std` *(default)* — standard library maths.
- `libm` — for `no_std` targets: `--no-default-features --features libm`.
- `serde` — serialisation; deserialisation applies construction-time validation.

No dependencies by default; no allocation.

## License

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or [MIT license](LICENSE-MIT) at your option.

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in this crate by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.
