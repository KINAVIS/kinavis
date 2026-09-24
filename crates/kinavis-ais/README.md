# kinavis-ais

AIS messages from NMEA 0183 encapsulation sentences to the
[kinavis-kernel](https://crates.io/crates/kinavis-kernel) value types.

AIS messages arrive as `!AIVDM` sentences, 6 bits per character, split across
sentences when long.
[kinavis-nmea0183](https://crates.io/crates/kinavis-nmea0183) validates each
sentence and yields the armoured payload; this crate unarmours it, reassembles
fragments and decodes the message into kernel types — `Position`, `Speed`,
`TrueCourse`, `TargetId` (MMSI), `Distance` (draught), `InlineStr` (names). "Not
available" is `None`, never zero.

- Position reports — messages 1, 2, 3 (class A), 18, 19 (class B) — as
  `PositionReport`; message 19 adds name, ship type and dimensions.
- Static and voyage data — message 5 — as `StaticAndVoyageData`: IMO, call sign,
  name, ship and cargo type, dimensions, draught, destination, ETA.
- Class B static data — message 24, part A or B — as `StaticDataReport`.
- Aids to navigation — message 21 — as `AidToNavigation`: mark type, name,
  off-position and virtual flags.
- Other types: `Message::Unsupported` with the type and raw bits.
- No allocation: fixed-capacity bit buffer; assembler with fixed slots and a
  timeout.
- No panics: every failure is an `AisError`.

```rust
use kinavis_ais::{Assembler, Message, StationClass};
use kinavis_kernel::{Instant, Utc};
use kinavis_nmea0183::{parse, Sentence};

let mut assembler = Assembler::new();
let now = Instant::<Utc>::from_unix_seconds(1_789_000_000);

let line = b"!AIVDM,1,1,,A,13aEOK?P00PD2wVMdLDRhgvL289?,0*26\r\n";
let Sentence::Vdm(vdm) = parse(line)? else { panic!("not an AIS sentence") };
let Some(bits) = assembler.push(&vdm, now)? else { panic!("in fragments") };
let Message::PositionReport(report) = Message::decode(&bits)? else { panic!("not a position") };

assert_eq!(report.station_class(), StationClass::A);
assert_eq!(report.mmsi.number(), 244_670_316);
assert_eq!(format!("{:.3}", report.position.unwrap()), "51°53.685'N 004°22.757'E");
assert_eq!(report.course.map(|c| c.degrees()), Some(70.6));
assert_eq!(report.heading, None);
# Ok::<(), Box<dyn std::error::Error>>(())
```

The navigation algorithms and the list of the other KINAVIS crates are in
[`kinavis`](https://crates.io/crates/kinavis).

## Feature flags

- `std` *(default)* — standard library maths in the kernel.
- `libm` — for `no_std` targets: `--no-default-features --features libm`.
- `serde` — serialisation of the messages.

## License

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or [MIT license](LICENSE-MIT) at your option.

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in this crate by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.
