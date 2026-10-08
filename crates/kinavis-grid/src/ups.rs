//! Polar stereographic projection, as GeographicLib's `PolarStereographic`
//! (C. Karney, MIT licence): exact formulas through the conformal latitude.

use kinavis_kernel::error::Result;
use kinavis_kernel::geodesy::Ellipsoid;
use kinavis_kernel::math;

use crate::conformal;
use crate::tm::sin_cos_degrees;

/// Polar stereographic projection centred on one pole.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct PolarStereographic {
    a: f64,
    e2: f64,
    es: f64,
    e2m: f64,
    c: f64,
    k0: f64,
}

impl PolarStereographic {
    pub(crate) fn new(ellipsoid: &Ellipsoid, k0: f64) -> Self {
        let f = ellipsoid.flattening();
        let e2 = f * (2.0 - f);
        let es = math::copysign(math::sqrt(math::abs(e2)), f);
        Self {
            a: ellipsoid.semi_major_axis().metres(),
            e2,
            es,
            e2m: 1.0 - e2,
            c: (1.0 - f) * math::exp(conformal::eatanhe(1.0, es)),
            k0,
        }
    }

    /// `(x, y, γ°, k)` about the pole, no false origin.
    pub(crate) fn forward(
        &self,
        north: bool,
        latitude: f64,
        longitude: f64,
    ) -> (f64, f64, f64, f64) {
        let lat = if north { latitude } else { -latitude };
        let (sphi, cphi) = sin_cos_degrees(lat);
        let (tau, secphi) = if lat < 90.0 {
            (sphi / cphi, math::hypot(1.0, sphi / cphi))
        } else {
            (f64::INFINITY, f64::INFINITY)
        };
        let taup = conformal::taup(tau, self.es);
        let mut rho = math::hypot(1.0, taup) + math::abs(taup);
        rho = if taup >= 0.0 {
            if lat < 90.0 {
                1.0 / rho
            } else {
                0.0
            }
        } else {
            rho
        };
        rho *= 2.0 * self.k0 * self.a / self.c;
        let k = if lat < 90.0 {
            (rho / self.a) * secphi * math::sqrt(self.e2m + self.e2 / (secphi * secphi))
        } else {
            self.k0
        };
        let (slam, clam) = sin_cos_degrees(longitude);
        let x = rho * slam;
        let y = if north { -rho * clam } else { rho * clam };
        let gamma = kinavis_kernel::wrap180(if north { longitude } else { -longitude });
        (x, y, gamma, k)
    }

    /// `(φ°, λ°, γ°, k)` from coordinates about the pole.
    pub(crate) fn reverse(&self, north: bool, x: f64, y: f64) -> Result<(f64, f64, f64, f64)> {
        let rho = math::hypot(x, y);
        let t = if rho > 0.0 {
            rho / (2.0 * self.k0 * self.a / self.c)
        } else {
            f64::EPSILON * f64::EPSILON
        };
        let taup = (1.0 / t - t) / 2.0;
        let tau = conformal::tau(taup, self.es)?;
        let secphi = math::hypot(1.0, tau);
        let k = if rho > 0.0 {
            (rho / self.a) * secphi * math::sqrt(self.e2m + self.e2 / (secphi * secphi))
        } else {
            self.k0
        };
        let latitude = math::to_degrees(math::atan(tau));
        let longitude = math::to_degrees(math::atan2(x, if north { -y } else { y }));
        let gamma = kinavis_kernel::wrap180(if north { longitude } else { -longitude });
        Ok((
            if north { latitude } else { -latitude },
            kinavis_kernel::wrap180(longitude),
            gamma,
            k,
        ))
    }
}
