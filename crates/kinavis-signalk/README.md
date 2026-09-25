# kinavis-signalk

Signal K deltas in and out of the [KINAVIS](https://crates.io/crates/kinavis)
navigation crates.

A [Signal K](https://signalk.org) server merges a vessel's NMEA 0183,
NMEA 2000 and AIS into one JSON model and streams changes as deltas. This
crate reads them into a picture of own vessel and every target, and assesses
it: CPA and TCPA of each target, and for a dangerous or developing approach
the COLREGs ruling — the encounter, who gives way, and what each may do. The
results go back as `navigation.closestApproach` deltas in each target's
context and as notifications on own vessel:

```text
notifications.navigation.closestApproach.urn:mrn:imo:mmsi:244000001
  state: alarm
  message: ANNA: CPA 0.00 NM in 15:00; crossing, target on starboard, give way (Rule 15): alter to starboard, or slow down
```

- Units converted at the boundary: Signal K's radians and metres per second
  in, KINAVIS types inside.
- COLREGs category from `navigation.state`, else the AIS ship type; a vessel
  at anchor, moored or aground gets a CPA but no ruling.
- An alarm for a dangerous CPA within the TCPA limit, a warning for one within
  the warning horizon; a notification is sent when a target's level changes,
  and cleared when it is no longer dangerous, stale or gone.
- No ruling while own vessel is at rest: the steering rules are for vessels
  under way.

An aid to the watch, not a substitute for it: a proper lookout (COLREGs
Rule 5) comes first, the ruling takes every target to be what AIS says it is,
and Rules 9 and 10 are not applied.

Requires the standard library. The Signal K server plugin built on it is
[`signalk-kinavis`](https://github.com/KINAVIS/signalk-kinavis).

The navigation algorithms and the list of the other KINAVIS crates are in
[`kinavis`](https://crates.io/crates/kinavis).

## License

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or [MIT license](LICENSE-MIT) at your option.

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in this crate by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.
