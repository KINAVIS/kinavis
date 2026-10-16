# kinavis-integrity

GNSS integrity for the [KINAVIS](https://github.com/KINAVIS/kinavis) crates:
receiver autonomous integrity monitoring (RAIM), fault exclusion and
protection levels.

```rust
use kinavis_integrity::{assess, IntegrityReport, RaimConfig, RangeError};
use kinavis_kernel::{Distance, SatelliteSet};

# fn sky() -> SatelliteSet { SatelliteSet::new() }
let config = RaimConfig::new(1e-3, 1e-3, RangeError::ElevationWeighted(Distance::from_metres(0.5)?))?;
match assess(&sky(), &config)? {
    IntegrityReport::Available { protection, .. } => {
        println!("HPL {:.1} m, VPL {:.1} m", protection.horizontal.metres(), protection.vertical.metres());
    }
    IntegrityReport::FaultDetected { excluded, .. } => println!("fault, excluded {excluded:?}"),
    _ => println!("integrity not available"),
}
# Ok::<(), kinavis_kernel::KernelError>(())
```

- **Detection**: weighted least squares over the satellites used, with a
  receiver clock per constellation; the chi-square test of the range residuals
  at a chosen probability of false alarm.
- **Exclusion**: each satellite in turn is left out; the exclusion that
  restores consistency with the most margin is reported, with its protection
  levels.
- **Protection levels**: the largest horizontal and vertical slopes times the
  bias that the noncentral chi-square leaves undetected with the chosen
  probability of missed detection. `predict` gives them from geometry alone.
- **Receivers without residuals**: `protection_from_ellipse`, a bound from
  the error ellipse — an estimate, not RAIM.
- **Alert limits** turn a report into the kernel's `NavigationIntegrity`.

The satellites come as the kernel's `SatelliteSet`, filled from NMEA
(`kinavis-nmea0183`: `GSV`, `GSA`, `GRS`) or UBX (`kinavis-ubx`: `NAV-SAT`).

Checked: thresholds and bias noncentralities against mpmath (the noncentral
distribution by integration of its density, not by the Poisson mixture used
here); statistic and protection levels against numpy over real skies of u-blox
receivers; false-alarm rate and fault exclusion by Monte Carlo. RAIM on code
residuals is not an integrity bound for RTK. `no_std`, no allocation; a test
takes about 20 KiB of stack.

## Feature flags

- `std` *(default)* — standard library maths in the kernel.
- `libm` — for `no_std` targets: `--no-default-features --features libm`.

## License

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or [MIT license](LICENSE-MIT) at your option.

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in this crate by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.
