//! Identical results with `std` and `libm` maths.
//!
//! `std` uses the platform libm; the `libm` feature uses the pure-Rust crate.
//! The snapshot below comes from the `std` build; CI runs this test in both
//! configurations.
//!
//! To regenerate after an intentional change to the maths:
//!
//! ```text
//! cargo test --test determinism -- --ignored --nocapture
//! ```
//!
//! and state it in the commit message.

// The snapshot covers the embedded coefficients only.
#![cfg(feature = "wmm2025")]
// Tests may panic on a failed expectation; the library may not. The snapshot is
// machine-written at round-trip precision.
#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::panic,
    clippy::unreadable_literal
)]

use kinavis_kernel::environment::MagneticModel;
use kinavis_kernel::{Civil, Distance, GeodeticPoint, Height, Instant, Position, Utc};
use kinavis_wmm::Wmm;

/// Allowed relative difference between the two implementations.
///
/// The synthesis sums a few hundred Legendre and trigonometric terms; 1000 ε
/// covers the rounding and is 10⁶ times below the model error.
const RELATIVE_TOLERANCE: f64 = 1e-13;

/// Output of the `std` build; see the module docs.
const EXPECTED: &[(&str, f64)] = &[
    ("ushant 2025 north", 2.129049733575729e4),
    ("ushant 2025 east", -2.2624602562493078e2),
    ("ushant 2025 down", 4.29755684689266e4),
    ("ushant 2025 declination", -6.088375617331039e-1),
    ("ushant 2025 inclination", 6.364446924899117e1),
    ("ushant 2028 north", 2.133252112956178e4),
    ("ushant 2028 east", -6.10258380543635e-2),
    ("ushant 2028 down", 4.307361161066139e4),
    ("ushant 2028 declination", -1.6390575406059228e-4),
    ("ushant 2028 inclination", 6.365274973839997e1),
    ("resolute 2025 north", 3.120268892503185e3),
    ("resolute 2025 east", -8.971523820744528e2),
    ("resolute 2025 down", 5.7213122557745315e4),
    ("resolute 2025 declination", -1.6041216619612616e1),
    ("resolute 2025 inclination", 8.675210827800944e1),
    ("resolute 2028 north", 3.3781688100702568e3),
    ("resolute 2028 east", -8.346650746144451e2),
    ("resolute 2028 down", 5.7110308997563814e4),
    ("resolute 2028 declination", -1.3878467517903053e1),
    ("resolute 2028 inclination", 8.651325546157595e1),
    ("hobart 2025 north", 1.7826177076529315e4),
    ("hobart 2025 east", 4.952172668275565e3),
    ("hobart 2025 down", -5.909059975242223e4),
    ("hobart 2025 declination", 1.5525474073266201e1),
    ("hobart 2025 inclination", -7.261466816741775e1),
    ("hobart 2028 north", 1.7790355448616836e4),
    ("hobart 2028 east", 5.013899005793707e3),
    ("hobart 2028 down", -5.903121830644197e4),
    ("hobart 2028 declination", 1.5739557428365382e1),
    ("hobart 2028 inclination", -7.261402387311065e1),
];

#[test]
fn the_two_maths_libraries_agree() {
    let measured = measurements();
    assert_eq!(
        measured.len(),
        EXPECTED.len(),
        "the snapshot is stale: regenerate it"
    );

    let mut worst = (0.0_f64, "");
    for ((name, found), (stored, expected)) in measured.iter().zip(EXPECTED) {
        assert_eq!(name, stored, "the snapshot is out of order: regenerate it");
        let error = (found - expected).abs() / expected.abs().max(1.0);
        if error > worst.0 {
            worst = (error, name);
        }
        assert!(
            error <= RELATIVE_TOLERANCE,
            "{name}: {found:e} vs {expected:e}, relative error {error:e}"
        );
    }
    println!("worst relative difference {:e}, at {}", worst.0, worst.1);
}

/// Prints the snapshot in the form of the constant above.
#[test]
#[ignore = "regenerates the snapshot rather than checking it"]
fn print_the_snapshot() {
    for (name, value) in measurements() {
        println!("    (\"{name}\", {value:e}),");
    }
}

/// Field at three points and two dates: mid-latitude, high latitude near the
/// dip pole (weak horizontal field), and southern hemisphere.
fn measurements() -> Vec<(String, f64)> {
    let mut out = Vec::new();
    let mut record = |name: &str, value: f64| out.push((name.to_owned(), value));

    let places = [
        ("ushant", 48.5, -5.5, 0.0),
        ("resolute", 74.7, -94.8, 50.0),
        ("hobart", -42.9, 147.3, 1000.0),
    ];
    let dates = [
        ("2025", Civil::date(2025, 1, 1)),
        ("2028", Civil::date(2028, 7, 1)),
    ];
    for (place, latitude, longitude, metres) in places {
        let point = GeodeticPoint::new(
            Position::from_degrees(latitude, longitude).unwrap(),
            Height::above_ellipsoid(Distance::from_metres(metres).unwrap()),
        );
        for (year, date) in dates {
            let when = Instant::<Utc>::from_civil(date).unwrap();
            let field = Wmm::WMM2025.field_at(point, when).unwrap();
            record(&format!("{place} {year} north"), field.north_nanotesla());
            record(&format!("{place} {year} east"), field.east_nanotesla());
            record(&format!("{place} {year} down"), field.down_nanotesla());
            record(
                &format!("{place} {year} declination"),
                field.declination().degrees(),
            );
            record(
                &format!("{place} {year} inclination"),
                field.inclination().degrees(),
            );
        }
    }

    out
}
