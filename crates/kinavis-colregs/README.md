# kinavis-colregs

COLREGs steering and sailing rules (International Regulations for Preventing
Collisions at Sea, 1972) for the [KINAVIS](https://github.com/KINAVIS/kinavis)
crates.

For two vessels — positions, motion, categories — returns the encounter type
(overtaking, head-on, crossing), which vessel gives way under which rule, and
the permitted manoeuvre: Rules 12–19 as data, Rule 19 in restricted visibility.
CPA/TCPA kinematics are in `kinavis-traffic`; the two are used together.

Sector widths are conventions and are configurable in `ColregsConfig`. Rules 9
and 10 (narrow channels, TSS) are not applied. Every ruling names its deciding
rule.

```rust
use kinavis::relative_motion::{Contact, Vessel};
use kinavis_colregs::{rule_of_the_road, ColregsConfig, Party, Responsibility, Situation, VesselCategory, Visibility};
use kinavis_kernel::{Distance, Speed, TrueBearing, TrueCourse};

// Own ship heading north; power-driven target on the starboard bow heading west.
let own = Party::new(
    Vessel { course: TrueCourse::new(0.0)?, speed: Speed::from_knots(12.0)? },
    VesselCategory::PowerDriven,
);
let target = Party::new(
    Vessel { course: TrueCourse::new(270.0)?, speed: Speed::from_knots(12.0)? },
    VesselCategory::PowerDriven,
);
let contact = Contact { bearing: TrueBearing::new(45.0)?, range: Distance::from_nautical_miles(5.0)? };

let ruling = rule_of_the_road(&Situation::new(own, target, contact, Visibility::InSight), &ColregsConfig::STANDARD)?;
assert_eq!(ruling.responsibility(), Responsibility::GiveWay);
assert_eq!(format!("{}", ruling.rule()), "Rule 15");
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
