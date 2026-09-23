//! World Magnetic Model as a [`MagneticModel`] for the KINAVIS crates.
//!
//! WMM is the standard main-field model: a spherical harmonic expansion to
//! degree 12 with linear secular variation, issued every five years by
//! NOAA/NCEI and the British Geological Survey. Magnetic variation
//! (declination) is the angle between its horizontal component and true north.
//!
//! The `wmm2025` feature embeds the WMM2025 coefficients (valid 2025.0–2030.0)
//! and implements the kernel's [`MagneticModel`] port. A date outside the
//! validity interval returns [`KernelError::OutsideValidity`] instead of
//! extrapolating: coefficients five years out of date can be a degree off.
//!
//! ```rust
//! use kinavis_kernel::environment::MagneticModel;
//! use kinavis_kernel::{Civil, Distance, GeodeticPoint, Height, Instant, Position, Utc};
//! use kinavis_wmm::Wmm;
//!
//! // Off Ushant, midsummer 2026.
//! let point = GeodeticPoint::new(
//!     Position::from_degrees(48.5, -5.5)?,
//!     Height::above_mean_sea_level(Distance::ZERO),
//! );
//! let when = Instant::<Utc>::from_civil(Civil::date(2026, 6, 21))?;
//!
//! let field = Wmm::WMM2025.field_at(point, when)?;
//! assert_eq!(format!("{:.1}", field.declination()), "0.4°W");
//! assert!(field.horizontal_intensity_nanotesla() > 20_000.0);
//!
//! // Before the epoch the model has nothing to say.
//! let too_early = Instant::<Utc>::from_civil(Civil::date(2024, 12, 31))?;
//! assert!(Wmm::WMM2025.field_at(point, too_early).is_err());
//! # Ok::<(), kinavis_kernel::KernelError>(())
//! ```
//!
//! # Accuracy
//!
//! The synthesis reproduces NOAA's hundred published test values to better than
//! `0.01 nT` per component and `0.01°` in declination and inclination — the
//! resolution of the published values. The model itself has a global RMS error
//! of about `0.5°` in declination, larger near the magnetic poles where the
//! horizontal field is weak. Local anomalies, ship's magnetism and space
//! weather are not modelled.
//!
//! # Height
//!
//! The model takes height above the WGS-84 ellipsoid. Heights above MSL or
//! chart datum differ by the geoid undulation (≤ ~100 m), which changes the
//! field by less than `3 nT`, two orders below the model error; every
//! [`Height`] is therefore used as is, whatever its datum.
//!
//! # Feature flags
//!
//! - `std` *(default)* — standard library maths in the kernel.
//! - `libm` — for `no_std` targets: `--no-default-features --features libm`.
//! - `wmm2025` *(default)* — embeds the WMM2025 coefficients as
//!   [`Wmm::WMM2025`]. Without it, only the synthesis and [`Wmm::new`] for
//!   caller-supplied coefficients.
//!
//! No allocation; builds for bare-metal targets.
//!
//! [`Height`]: kinavis_kernel::Height

#![cfg_attr(not(feature = "std"), no_std)]

// The crate does not allocate; tests use `format!`.
#[cfg(test)]
extern crate alloc;

mod legendre;
#[cfg(feature = "wmm2025")]
mod wmm2025;

use core::fmt;

use kinavis_kernel::environment::{MagneticField, MagneticModel};
use kinavis_kernel::error::{ensure_finite, ensure_range, KernelError, Result};
use kinavis_kernel::geodesy::{Ellipsoid, GeodeticPoint};
use kinavis_kernel::math;
use kinavis_kernel::time::{Civil, Instant, Utc};

use legendre::Legendre;
pub use legendre::MAX_DEGREE;

/// Geomagnetic reference radius `a`, in metres.
///
/// Not the ellipsoid semi-major axis: the model is defined on a sphere of this
/// radius.
pub const REFERENCE_RADIUS_METRES: f64 = 6_371_200.0;

/// Validity span of one model issue, in years.
pub const VALIDITY_YEARS: f64 = 5.0;

/// Maximum number of coefficients: every `(n, m)` with `1 ≤ m ≤ n` up to
/// [`MAX_DEGREE`].
pub const MAX_COEFFICIENTS: usize = MAX_DEGREE * (MAX_DEGREE + 3) / 2;

/// Time in the model's coordinate: decimal year.
///
/// `2025.5` is halfway through 2025 by calendar days, as in the coefficient
/// files and test values. Built from an [`Instant`] in use, or from a number to
/// check against published values.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct DecimalYear(f64);

impl DecimalYear {
    /// Decimal year from a number.
    ///
    /// # Errors
    ///
    /// [`KernelError::NotFinite`] for `NaN` or infinity;
    /// [`KernelError::OutOfRange`] outside `[-10_000, 10_000]`.
    pub fn new(year: f64) -> Result<Self> {
        ensure_range("decimal year", year, -10_000.0, 10_000.0)?;
        Ok(Self(year))
    }

    /// Decimal year of an instant: calendar year plus the elapsed fraction of
    /// that year (leap years included).
    #[must_use]
    pub fn from_instant(at: Instant<Utc>) -> Self {
        let year = at.civil().year;
        // Both dates are valid by construction; the fallbacks are unreachable.
        let start = Instant::<Utc>::from_civil(Civil::date(year, 1, 1)).unwrap_or(at);
        let end =
            Instant::<Utc>::from_civil(Civil::date(year.saturating_add(1), 1, 1)).unwrap_or(start);
        let elapsed = at.checked_duration_since(start).unwrap_or_default();
        let length = end.checked_duration_since(start).unwrap_or_default();
        let fraction = if length.is_zero() {
            0.0
        } else {
            elapsed.as_secs_f64() / length.as_secs_f64()
        };
        Self(f64::from(year) + fraction)
    }

    /// Year as `f64`.
    #[must_use]
    pub const fn value(self) -> f64 {
        self.0
    }
}

impl fmt::Display for DecimalYear {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let precision = f.precision().unwrap_or(3);
        write!(f, "{:.precision$}", self.0)
    }
}

/// Gauss coefficient pair with its secular variation.
///
/// Degree `n`, order `m`; `g`, `h` in nT at the epoch and their rates in
/// nT/year. `h` is zero for `m = 0`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Coefficient {
    n: u8,
    m: u8,
    g: f64,
    h: f64,
    g_dot: f64,
    h_dot: f64,
}

impl Coefficient {
    /// Coefficient as a row of the coefficient file.
    ///
    /// Validated when the [`Wmm`] is built, so a table can be a `const`.
    #[must_use]
    pub const fn new(n: u8, m: u8, g: f64, h: f64, g_dot: f64, h_dot: f64) -> Self {
        Self {
            n,
            m,
            g,
            h,
            g_dot,
            h_dot,
        }
    }

    /// Degree.
    #[must_use]
    pub const fn degree(&self) -> u8 {
        self.n
    }

    /// Order.
    #[must_use]
    pub const fn order(&self) -> u8 {
        self.m
    }

    /// `g` and `h` at `years` past the epoch.
    fn at(&self, years: f64) -> (f64, f64) {
        (self.g + years * self.g_dot, self.h + years * self.h_dot)
    }
}

/// One issue of the World Magnetic Model: coefficients, epoch, name.
///
/// [`Wmm::WMM2025`] is the embedded issue; [`Wmm::new`] accepts a
/// caller-supplied coefficient table, e.g. a newer issue.
#[derive(Debug, Clone, Copy)]
pub struct Wmm {
    name: &'static str,
    epoch: f64,
    coefficients: &'static [Coefficient],
}

impl Wmm {
    /// WMM2025, valid 2025.0–2030.0; coefficient file dated 2024-11-13.
    #[cfg(feature = "wmm2025")]
    pub const WMM2025: Self = Self {
        name: wmm2025::NAME,
        epoch: wmm2025::EPOCH,
        coefficients: &wmm2025::COEFFICIENTS,
    };

    /// Model from a coefficient table.
    ///
    /// The table must hold the full expansion, degrees 1 to [`MAX_DEGREE`], in
    /// coefficient-file order: every `(n, m)` with `0 ≤ m ≤ n`, degree by
    /// degree.
    ///
    /// # Errors
    ///
    /// [`KernelError::NotFinite`] for a non-finite coefficient or epoch;
    /// [`KernelError::OutOfRange`] for an epoch outside the calendar;
    /// [`KernelError::InsufficientData`] or [`KernelError::CapacityExceeded`]
    /// unless the table has [`MAX_COEFFICIENTS`] rows; [`KernelError::Parse`]
    /// for a row out of order.
    pub fn new(
        name: &'static str,
        epoch: DecimalYear,
        coefficients: &'static [Coefficient],
    ) -> Result<Self> {
        if coefficients.len() < MAX_COEFFICIENTS {
            return Err(KernelError::InsufficientData {
                found: coefficients.len(),
                required: MAX_COEFFICIENTS,
                context: "a coefficient table",
            });
        }
        if coefficients.len() > MAX_COEFFICIENTS {
            return Err(KernelError::CapacityExceeded {
                context: "a coefficient table",
                needed: coefficients.len(),
                capacity: MAX_COEFFICIENTS,
            });
        }
        let mut expected = (1_u8, 0_u8);
        for coefficient in coefficients {
            if (coefficient.n, coefficient.m) != expected {
                return Err(KernelError::Parse {
                    what: "coefficient table",
                    input: kinavis_kernel::Excerpt::new(name),
                });
            }
            ensure_finite("g", coefficient.g)?;
            ensure_finite("h", coefficient.h)?;
            ensure_finite("g_dot", coefficient.g_dot)?;
            ensure_finite("h_dot", coefficient.h_dot)?;
            // The length check bounds the degree by `MAX_DEGREE`; saturating
            // arithmetic states that the count cannot wrap.
            expected = if expected.1 == expected.0 {
                (expected.0.saturating_add(1), 0)
            } else {
                (expected.0, expected.1.saturating_add(1))
            };
        }
        Ok(Self {
            name,
            epoch: epoch.value(),
            coefficients,
        })
    }

    /// Model name from the coefficient file, e.g. `WMM-2025`.
    #[must_use]
    pub const fn name(&self) -> &'static str {
        self.name
    }

    /// Epoch of the coefficients.
    #[must_use]
    pub const fn epoch(&self) -> DecimalYear {
        DecimalYear(self.epoch)
    }

    /// End of validity (exclusive): epoch + [`VALIDITY_YEARS`].
    #[must_use]
    pub fn expires(&self) -> DecimalYear {
        DecimalYear(self.epoch + VALIDITY_YEARS)
    }

    /// Whether `year` is within the validity interval.
    #[must_use]
    pub fn is_valid_at(&self, year: DecimalYear) -> bool {
        year.0 >= self.epoch && year.0 <= self.epoch + VALIDITY_YEARS
    }

    /// Field at `at` in decimal year `year`.
    ///
    /// [`MagneticModel::field_at`] converts an instant and calls this.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutsideValidity`] if `year` is outside `[epoch, epoch +
    /// 5)`.
    // Notation follows the WMM technical report.
    #[allow(clippy::many_single_char_names)]
    pub fn field_in(&self, at: GeodeticPoint, year: DecimalYear) -> Result<MagneticField> {
        if !self.is_valid_at(year) {
            return Err(KernelError::OutsideValidity {
                data: "magnetic model",
            });
        }
        let years = year.0 - self.epoch;

        // Geodetic to geocentric spherical coordinates on WGS-84.
        let geodetic = at.position();
        let phi = geodetic.latitude().radians();
        let lambda = geodetic.longitude().radians();
        let height = at.height().value().metres();
        let wgs84 = Ellipsoid::WGS84;
        let e2 = wgs84.first_eccentricity_squared();
        let (sin_phi, cos_phi) = (math::sin(phi), math::cos(phi));
        let prime_vertical =
            wgs84.semi_major_axis().metres() / math::sqrt(1.0 - e2 * sin_phi * sin_phi);
        let p = (prime_vertical + height) * cos_phi;
        let z = (prime_vertical * (1.0 - e2) + height) * sin_phi;
        let radius = math::hypot(p, z);
        let phi_prime = math::atan2(z, p);

        // The expansion, in the geocentric frame.
        let tables = Legendre::at(phi_prime);
        let ratio = REFERENCE_RADIUS_METRES / radius;
        let (mut north, mut east, mut down) = (0.0, 0.0, 0.0);
        for coefficient in self.coefficients {
            // `new` validated every degree and order; this bound lets the
            // compiler prove the indexing cannot wrap.
            let n = usize::from(coefficient.n).min(MAX_DEGREE);
            let m = usize::from(coefficient.m).min(n);
            let (g, h) = coefficient.at(years);
            let m_lambda = math::count_to_f64(m) * lambda;
            let (sin_m, cos_m) = (math::sin(m_lambda), math::cos(m_lambda));
            let in_phase = g * cos_m + h * sin_m;
            let quadrature = g * sin_m - h * cos_m;
            let scale = power(ratio, n + 2);
            north -= scale * in_phase * tables.derivative.at(n, m);
            east += scale * math::count_to_f64(m) * quadrature * tables.over_cosine.at(n, m);
            down -= scale * math::count_to_f64(n + 1) * in_phase * tables.value.at(n, m);
        }

        // Rotate from geocentric to geodetic vertical.
        let delta = phi_prime - phi;
        let (sin_delta, cos_delta) = (math::sin(delta), math::cos(delta));
        let x = north * cos_delta - down * sin_delta;
        let y = east;
        let z = north * sin_delta + down * cos_delta;
        MagneticField::from_ned_nanotesla(x, y, z)
    }
}

impl MagneticModel for Wmm {
    fn field_at(&self, at: GeodeticPoint, when: Instant<Utc>) -> Result<MagneticField> {
        self.field_in(at, DecimalYear::from_instant(when))
    }
}

/// `base` to a small integer power by repeated multiplication: exact enough for
/// `(a/r)ⁿ⁺²`, `n ≤ 12`, and identical on every target.
fn power(base: f64, exponent: usize) -> f64 {
    let mut result = 1.0;
    for _ in 0..exponent {
        result *= base;
    }
    result
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use alloc::format;

    use super::*;

    #[test]
    fn a_decimal_year_counts_the_calendars_own_days() {
        let midyear = |year: i32| {
            // A common year has 365 days; halfway is 182.5 days in.
            let days = if (year % 4 == 0 && year % 100 != 0) || year % 400 == 0 {
                366.0
            } else {
                365.0
            };
            let start = Instant::<Utc>::from_civil(Civil::date(year, 1, 1)).unwrap();
            start.saturating_add(core::time::Duration::from_secs_f64(days / 2.0 * 86_400.0))
        };
        assert!((DecimalYear::from_instant(midyear(2025)).value() - 2025.5).abs() < 1e-9);
        assert!((DecimalYear::from_instant(midyear(2028)).value() - 2028.5).abs() < 1e-9);

        let new_year = Instant::<Utc>::from_civil(Civil::date(2027, 1, 1)).unwrap();
        assert_eq!(DecimalYear::from_instant(new_year).value(), 2027.0);
        assert_eq!(
            format!("{}", DecimalYear::new(2026.25).unwrap()),
            "2026.250"
        );
        assert!(DecimalYear::new(f64::NAN).is_err());
    }

    #[cfg(feature = "wmm2025")]
    #[test]
    fn the_embedded_model_knows_its_span() {
        let model = Wmm::WMM2025;
        assert_eq!(model.name(), "WMM-2025");
        assert_eq!(model.epoch().value(), 2025.0);
        assert_eq!(model.expires().value(), 2030.0);
        assert!(model.is_valid_at(DecimalYear::new(2025.0).unwrap()));
        assert!(model.is_valid_at(DecimalYear::new(2030.0).unwrap()));
        assert!(!model.is_valid_at(DecimalYear::new(2024.999).unwrap()));
        assert!(!model.is_valid_at(DecimalYear::new(2030.001).unwrap()));
    }

    #[cfg(feature = "wmm2025")]
    #[test]
    fn the_embedded_table_passes_the_checks_a_supplied_one_must() {
        let rebuilt = Wmm::new(
            "again",
            DecimalYear::new(2025.0).unwrap(),
            &wmm2025::COEFFICIENTS,
        )
        .unwrap();
        assert_eq!(rebuilt.name(), "again");
    }

    #[test]
    fn a_supplied_table_is_checked_for_length_and_order() {
        static SHORT: [Coefficient; 2] = [
            Coefficient::new(1, 0, 1.0, 0.0, 0.0, 0.0),
            Coefficient::new(1, 1, 1.0, 1.0, 0.0, 0.0),
        ];
        static DISORDERED: [Coefficient; MAX_COEFFICIENTS] =
            [Coefficient::new(12, 12, 0.0, 0.0, 0.0, 0.0); MAX_COEFFICIENTS];

        assert!(matches!(
            Wmm::new("short", DecimalYear::new(2025.0).unwrap(), &SHORT),
            Err(KernelError::InsufficientData { .. })
        ));
        assert!(matches!(
            Wmm::new("disordered", DecimalYear::new(2025.0).unwrap(), &DISORDERED),
            Err(KernelError::Parse { .. })
        ));
    }

    #[cfg(feature = "wmm2025")]
    #[test]
    fn outside_the_span_the_model_refuses() {
        use kinavis_kernel::{Distance, Height, Position};
        let point = GeodeticPoint::new(
            Position::from_degrees(50.0, 0.0).unwrap(),
            Height::above_ellipsoid(Distance::ZERO),
        );
        assert!(matches!(
            Wmm::WMM2025.field_in(point, DecimalYear::new(2031.0).unwrap()),
            Err(KernelError::OutsideValidity {
                data: "magnetic model"
            })
        ));
        assert!(Wmm::WMM2025
            .field_in(point, DecimalYear::new(2027.3).unwrap())
            .is_ok());
    }
}
