# kinavis-nmea0183

NMEA 0183 sentences parsed into and written from the
[kinavis-kernel](https://crates.io/crates/kinavis-kernel) value types.

Anti-corruption layer between receivers and the navigation domain: bytes in,
checksum and length validated, fields decoded into typed records, records
translated into a kernel `GnssFix`. The sentence never reaches the domain.

- Sentences: `RMC`, `GGA`, `GLL`, `VTG`; `VDM`/`VDO` with the AIS payload still
  armoured (decoded by `kinavis-ais`). Others are returned as
  `Sentence::Unsupported` with a verified checksum.
- Plausibility bounds on numeric fields (speed, altitude, DOP, differential
  age): a value outside is rejected as a corrupt field.
- Encoding as well as parsing, for generation and round-trip tests; a sentence
  longer than the standard's 82 bytes is refused, never written.
- No allocation: parses `&[u8]` in place.
- No panics: every failure is an `NmeaError`.

The navigation algorithms and the list of the other KINAVIS crates are in
[`kinavis`](https://crates.io/crates/kinavis).

## Feature flags

- `std` *(default)* — standard library maths in the kernel.
- `libm` — for `no_std` targets: `--no-default-features --features libm`.

## License

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or [MIT license](LICENSE-MIT) at your option.

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in this crate by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.
