# kinavis-nmea2000

NMEA 2000 parameter groups from CAN frames to the
[kinavis-kernel](https://crates.io/crates/kinavis-kernel) value types.

NMEA 2000 devices send numbered *parameter groups* — fixed little-endian layouts
in one 8-byte frame or, as a *fast packet*, up to 32 frames. From frames
delivered by the bus interface, this crate parses the 29-bit identifier (PGN,
source), reassembles fast packets and decodes the group into kernel types:
`Position`, `Speed`, `TrueCourse`, `Distance`, `Instant`. "Not available" is
`None`, never the raw all-ones value.

- Position rapid update (129025); COG/SOG rapid update (129026); GNSS position
  data (129029), with a kernel `GnssFix`.
- Vessel heading (127250) with reference, deviation and variation; water depth
  (128267).
- AIS position reports from an AIS receiver: class A (129038), class B (129039).
- Other PGNs: `Message::Unsupported` with the PGN and raw payload.
- No allocation: fixed-capacity payload buffer; assembler with fixed slots and a
  timeout.
- No panics: every failure is an `Nmea2000Error`.

The CAN controller, driver and address claim are the caller's.

```rust
use kinavis_nmea2000::{Assembler, CanId, Frame, Message, Pgn};
use kinavis_kernel::{Instant, Utc};

let mut assembler = Assembler::new();
let now = Instant::<Utc>::from_unix_seconds(1_789_000_000);

// Water depth from address 9: 12.34 m below the transducer, which is
// 1.5 m above the keel.
let id = CanId::from_parts(3, Pgn::WATER_DEPTH, 255, 9);
let frame = Frame::new(id, &[0x01, 0xD2, 0x04, 0x00, 0x00, 0x24, 0xFA, 0x0A])?;

let Some(payload) = assembler.push(&frame, now)? else { panic!("in frames") };
let Message::WaterDepth(depth) = Message::decode(&payload)? else { panic!("not a depth") };

assert_eq!(payload.source(), 9);
assert_eq!(format!("{:.2}", depth.depth.unwrap().metres()), "12.34");
assert_eq!(format!("{:.2}", depth.depth_from_reference().unwrap().metres()), "10.84");
# Ok::<(), Box<dyn std::error::Error>>(())
```

## Feature flags

- `std` *(default)* — standard library maths in the kernel.
- `libm` — for `no_std` targets: `--no-default-features --features libm`.
- `serde` — serialisation of the messages.

## License

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or [MIT license](LICENSE-MIT) at your option.

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in this crate by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.
