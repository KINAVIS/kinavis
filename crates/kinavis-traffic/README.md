# kinavis-traffic

Target tracking, collision assessment and avoidance for the
[KINAVIS](https://github.com/KINAVIS/kinavis) crates.

Radar plots and AIS reports are observations of numbered targets. `Traffic`
turns them into tracks — extrapolated position, course, speed, age — and reports
acquisitions and losses as events, returning a `TrafficView`. Tracks are stored
inline with a fixed history: no allocation, bare-metal capable. The picture is
~15 KiB regardless of target count and deliberately not `Copy`: keep it behind a
reference or in a `static`.

`assess_traffic` computes each target's CPA/TCPA against own ship's snapshot and
raises `CpaAlarm` inside the vessel's `CpaPolicy`; `avoid_all` finds the
smallest course alteration, within COLREGs and ship constraints, that clears
every target.

```rust
use kinavis_kernel::{Instant, Position, Speed, TargetId, Utc};
use kinavis_traffic::{TargetObservation, Traffic, TrackingPolicy, WhenFull};
use core::time::Duration;

// Three plots to acquire, stale after half a minute, dropped after three,
// nothing faster than sixty knots; and when the picture is full, the
// target that has gone quietest makes way for the newcomer.
let policy = TrackingPolicy::new(
    3,
    Duration::from_secs(30),
    Duration::from_secs(180),
    Speed::from_knots(60.0)?,
)?
.when_full(WhenFull::EvictStalest);
let mut traffic = Traffic::new(policy);

// Three radar plots of one target, a minute apart, heading north at 12 knots.
let start = Instant::<Utc>::from_unix_seconds(1_789_000_000);
for minute in 0_u32..3 {
    let position = Position::from_degrees(50.0 + 0.2 * f64::from(minute) / 60.0, -1.0)?;
    let at = start.checked_add(Duration::from_secs(60 * u64::from(minute))).unwrap();
    traffic.ingest(TargetObservation::new(TargetId::new(7), position, at))?;
}

let view = traffic.view(start.checked_add(Duration::from_secs(150)).unwrap());
let target = &view.targets()[0];
println!("{} {} at {}", target.target, target.motion.unwrap().course_over_ground, target.position);
# Ok::<(), kinavis::NavigationError>(())
```

## Feature flags

- `std` *(default)* — standard library maths in the kernel.
- `libm` — for `no_std` targets: `--no-default-features --features libm`.
- `serde` — serialisation of observations, policies and view entries.

## License

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or [MIT license](LICENSE-MIT) at your option.

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in this crate by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.
