# kinavis-j1939

SAE J1939 and ISO 11783-3 for the [KINAVIS](https://github.com/KINAVIS/kinavis)
crates: the 29-bit identifier, NAME and address claim, the transport
protocols.

```rust
use kinavis_j1939::{CanId, Frame, Pgn};

let frame = Frame::new(CanId::new(0x0CFE_4947)?, &[0x10, 0x0E, 0, 0, 0, 0, 0xFF, 0xFD])?;
assert_eq!(frame.id.pgn(), Pgn::new(65_097)); // ground-based speed
assert_eq!(frame.id.source(), 0x47);
# Ok::<(), kinavis_j1939::J1939Error>(())
```

- **Identifier and frame**: priority, PGN, source and destination of a
  29-bit identifier; frames of up to 8 bytes.
- **NAME and address claim**: the 64-bit NAME and its fields; the claim of
  ISO 11783-5 as a state machine — request, contention time, preferred or
  dynamic address, conflicts settled by NAME, cannot-claim.
- **Transport protocol** (9 to 1785 bytes): broadcast (BAM) and connections
  with flow control (RTS/CTS), received as a participant or followed as a
  bus monitor; broadcast frames and a connection sender.
- **Extended transport** (up to 117 MB): reception with data packet
  offsets, and a connection sender over borrowed data.

Every part is a state machine over frames and instants passed in; the CAN
controller, the sending and the clock are the caller's. Fast packet stays
in `kinavis-nmea2000`.

Checked against the frames of a John Deere tractor (TractorHacking/PolyCAN,
not in the repository: no licence) and constructed exchanges; fuzzed.
`no_std`, no allocation.

## Feature flags

- `std` *(default)* — standard library in the kernel.
- `libm` — for `no_std` targets: `--no-default-features --features libm`.
- `serde` — serialisation of identifiers, frames and NAMEs.

## License

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or [MIT license](LICENSE-MIT) at your option.

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in this crate by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.
