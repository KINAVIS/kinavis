# kinavis-alerts

Bridge alert management (BAM) for the
[KINAVIS](https://github.com/KINAVIS/kinavis) crates.

Each context reports events in its own enum: guidance (track exceeded), traffic
(CPA alarm), anchor watch (dragging). Which condition an event represents is
defined by the `Reportable` trait, implemented for every workspace event type
and implementable for application unions. Priority is a vessel policy
(`AlertPolicy`). `AlertManager` classifies conditions into alarms, warnings and
cautions, keeps one alert per condition regardless of repetition, tracks active
/ acknowledged / rectified states, and lists standing alerts by priority.

No allocation; builds for bare-metal targets. The board is several kilobytes and
deliberately not `Copy`: keep it behind a reference or in a `static`.

```rust
use kinavis::event::GuidanceEvent;
use kinavis_alerts::{AlertManager, AlertPriority, AlertState, StandardPolicy, MAX_ALERTS};
use kinavis_kernel::{Distance, EventList, Instant, Utc};

let mut alerts = AlertManager::new(StandardPolicy::default());
let now = Instant::<Utc>::from_unix_seconds(1_789_000_000);

// Guidance reports XTE beyond the limit.
let mut events = EventList::<GuidanceEvent>::new();
events.push(GuidanceEvent::CrossTrackExceeded {
    error: Distance::from_cables(7.0)?,
    limit: Distance::from_cables(5.0)?,
    at: now,
});
let changes = alerts.ingest(&events, now);
assert_eq!(changes.len(), 1);

let standing = alerts.alerts()[0];
assert_eq!(standing.priority(), AlertPriority::Warning);
assert_eq!(standing.state(), AlertState::Active);
alerts.acknowledge(standing.id(), now)?;
assert_eq!(alerts.alerts()[0].state(), AlertState::Acknowledged);
assert!(alerts.len() <= MAX_ALERTS);
# Ok::<(), kinavis_kernel::KernelError>(())
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
