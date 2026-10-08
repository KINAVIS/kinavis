# kinavis-ubx

The u-blox UBX binary protocol for the [KINAVIS](https://github.com/KINAVIS/kinavis)
crates: what F9, X20, M8 and M9 receivers report beyond NMEA, read into the
kernel's types.

```rust
use kinavis_ubx::{Deframer, Event, Message};

# let serial_port: &[u8] = &[];
let mut deframer: Deframer<1024> = Deframer::new();
for &byte in serial_port {
    match deframer.push(byte) {
        Some(Event::Frame(frame)) => {
            if let Ok(Message::NavPvt(pvt)) = Message::decode(&frame) {
                println!("{:?} {:?}", pvt.fix_type(), pvt.carrier);
            }
        }
        Some(Event::Passthrough(_nmea_byte)) => {} // NMEA on the same port
        _ => {}
    }
}
```

- **Framing**: a streaming deframer of fixed capacity, one byte at a time,
  resynchronising after noise; NMEA bytes on a mixed port are passed through;
  a frame longer than the capacity is an error, never truncated.
- **Navigation**: `NAV-PVT` (to a kernel `GnssFix` with the receiver's
  accuracy, RTK float/fixed from the carrier solution), `NAV-HPPOSLLH`
  (0.1 mm), `NAV-RELPOSNED` (moving base and dual-antenna heading),
  `NAV-SAT` (to a kernel `SatelliteSet` with residuals), `NAV-DOP`,
  `NAV-STATUS`, `NAV-TIMELS` (a `LeapSeconds` table from the receiver).
- **Sensor fusion**: `ESF-MEAS` (gyroscopes, accelerometers, wheel ticks,
  speed) and `ESF-INS`.
- **Raw measurements**: `RXM-RAWX` and `RXM-SFRBX` headers, the measurements
  left as bytes. `ACK-ACK`/`ACK-NAK`.

Checked field by field against pyubx2 on the UBX messages of 29 receiver logs
of the gpsd project (ZED-F9P, ZED-F9R, X20P, the M8 family); fuzzed. Reading
only. `no_std`, no allocation, no panics.

## Feature flags

- `std` *(default)* — standard library maths in the kernel.
- `libm` — for `no_std` targets: `--no-default-features --features libm`.

## License

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or [MIT license](LICENSE-MIT) at your option.

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in this crate by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.
