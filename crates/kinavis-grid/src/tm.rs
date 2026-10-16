//! Transverse Mercator: Krüger's series to sixth order in the third
//! flattening `n`, after Karney (2011), "Transverse Mercator with an accuracy
//! of a few nanometers", J. Geodesy 85(8), 475–485.
//!
//! A transcription of GeographicLib's `TransverseMercator` (C. Karney, MIT
//! licence) at its default order: conformal latitude, Gauss–Schreiber
//! coordinates, then the series summed by Clenshaw's method on complex
//! arguments, which also yields the meridian convergence and the scale. Error
//! about 5 nm within 3900 km of the central meridian.

use core::ops::{Add, Mul, Sub};

use kinavis_kernel::error::{ensure_finite, ensure_range, Result};
use kinavis_kernel::geodesy::Ellipsoid;
use kinavis_kernel::{math, wrap180, Distance, Latitude, Longitude};

use crate::conformal;

/// Order of the series.
const ORDER: usize = 6;

/// Rectifying radius factor: `(1 + n²/4 + n⁴/64 + n⁶/256) / (1 + n)`, as a
/// polynomial in `n²` over its denominator.
const B1: [f64; 5] = [1.0, 4.0, 64.0, 256.0, 256.0];

/// `α_l / n^l` as polynomials in `n` of order `6 − l`, each followed by its
/// denominator.
// Integers as GeographicLib publishes them, so the table can be compared.
#[allow(clippy::unreadable_literal)]
const ALPHA: [f64; 27] = [
    31564.0,
    -66675.0,
    34440.0,
    47250.0,
    -100800.0,
    75600.0,
    151200.0, //
    -1983433.0,
    863232.0,
    748608.0,
    -1161216.0,
    524160.0,
    1935360.0, //
    670412.0,
    406647.0,
    -533952.0,
    184464.0,
    725760.0, //
    6601661.0,
    -7732800.0,
    2230245.0,
    7257600.0, //
    -13675556.0,
    3438171.0,
    7983360.0, //
    212378941.0,
    319334400.0,
];

/// `β_l / n^l`, laid out as [`ALPHA`].
// Integers as GeographicLib publishes them, so the table can be compared.
#[allow(clippy::unreadable_literal)]
const BETA: [f64; 27] = [
    384796.0,
    -382725.0,
    -6720.0,
    932400.0,
    -1612800.0,
    1209600.0,
    2419200.0, //
    -1118711.0,
    1695744.0,
    -1174656.0,
    258048.0,
    80640.0,
    3870720.0, //
    22276.0,
    -16929.0,
    -15984.0,
    12852.0,
    362880.0, //
    -830251.0,
    -158400.0,
    197865.0,
    7257600.0, //
    -435388.0,
    453717.0,
    15966720.0, //
    20648693.0,
    638668800.0,
];

/// Parameters of a transverse Mercator projection.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TmParameters {
    /// Central meridian.
    pub central_meridian: Longitude,
    /// Latitude of the origin of northings.
    pub latitude_of_origin: Latitude,
    /// Scale on the central meridian.
    pub scale: f64,
    /// Added to every easting.
    pub false_easting: Distance,
    /// Added to every northing.
    pub false_northing: Distance,
}

/// Grid coordinates with the projection's distortion at the point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Projected {
    /// Easting.
    pub easting: Distance,
    /// Northing.
    pub northing: Distance,
    /// Meridian convergence: angle from true north to grid north, clockwise
    /// positive; grid = true − convergence.
    pub convergence: kinavis_kernel::Angle,
    /// Point scale: grid distance over ellipsoidal distance.
    pub scale: f64,
}

/// Transverse Mercator projection on one ellipsoid.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TransverseMercator {
    parameters: TmParameters,
    es: f64,
    e2: f64,
    e2m: f64,
    c: f64,
    a1: f64,
    b1: f64,
    alpha: [f64; ORDER + 1],
    beta: [f64; ORDER + 1],
    origin_northing: f64,
}

impl TransverseMercator {
    /// Projection on an ellipsoid with the given parameters.
    ///
    /// # Errors
    ///
    /// [`kinavis_kernel::KernelError::OutOfRange`] for a scale that is not
    /// positive; [`kinavis_kernel::KernelError::NotFinite`] for a non-finite
    /// one.
    pub fn new(ellipsoid: &Ellipsoid, parameters: TmParameters) -> Result<Self> {
        ensure_finite("scale on the central meridian", parameters.scale)?;
        ensure_range(
            "scale on the central meridian",
            parameters.scale,
            f64::MIN_POSITIVE,
            f64::MAX,
        )?;
        let f = ellipsoid.flattening();
        let e2 = f * (2.0 - f);
        let es = math::copysign(math::sqrt(math::abs(e2)), f);
        let e2m = 1.0 - e2;
        let n = f / (2.0 - f);
        let n2 = n * n;
        let b1 = polynomial(B1.get(..4).unwrap_or(&[]), n2) / (B1[4] * (1.0 + n));
        let mut alpha = [0.0; ORDER + 1];
        let mut beta = [0.0; ORDER + 1];
        let mut offset: usize = 0;
        let mut power = n;
        for l in 1..=ORDER {
            let order = ORDER - l;
            let last = offset.saturating_add(order);
            let alpha_terms = ALPHA.get(offset..=last).unwrap_or(&[]);
            let beta_terms = BETA.get(offset..=last).unwrap_or(&[]);
            let denominator = last.saturating_add(1);
            let alpha_denominator = ALPHA.get(denominator).copied().unwrap_or(1.0);
            let beta_denominator = BETA.get(denominator).copied().unwrap_or(1.0);
            if let (Some(a), Some(b)) = (alpha.get_mut(l), beta.get_mut(l)) {
                *a = power * polynomial(alpha_terms, n) / alpha_denominator;
                *b = power * polynomial(beta_terms, n) / beta_denominator;
            }
            offset = denominator.saturating_add(1);
            power *= n;
        }
        let mut projection = Self {
            parameters,
            es,
            e2,
            e2m,
            c: math::sqrt(e2m) * math::exp(conformal::eatanhe(1.0, es)),
            a1: b1 * ellipsoid.semi_major_axis().metres(),
            b1,
            alpha,
            beta,
            origin_northing: 0.0,
        };
        let (_, origin, _, _) = projection.forward_raw(
            parameters.latitude_of_origin.degrees(),
            parameters.central_meridian.degrees(),
        );
        projection.origin_northing = origin;
        Ok(projection)
    }

    /// Parameters.
    #[must_use]
    pub const fn parameters(&self) -> &TmParameters {
        &self.parameters
    }

    /// Grid coordinates of a latitude and longitude on the projection's
    /// ellipsoid.
    ///
    /// # Errors
    ///
    /// [`kinavis_kernel::KernelError::NotFinite`] if the result overflows;
    /// unreachable for terrestrial input.
    pub fn project(&self, latitude: Latitude, longitude: Longitude) -> Result<Projected> {
        let (x, y, gamma, k) = self.forward_raw(latitude.degrees(), longitude.degrees());
        Ok(Projected {
            easting: Distance::from_metres(x + self.parameters.false_easting.metres())?,
            northing: Distance::from_metres(
                y - self.origin_northing + self.parameters.false_northing.metres(),
            )?,
            convergence: kinavis_kernel::Angle::from_degrees(gamma)?,
            scale: k,
        })
    }

    /// Latitude and longitude of grid coordinates, with the convergence and
    /// scale there.
    ///
    /// # Errors
    ///
    /// [`kinavis_kernel::KernelError::NotConverged`] if the latitude does not
    /// settle; [`kinavis_kernel::KernelError::OutOfRange`] for coordinates
    /// that map outside the Earth.
    pub fn unproject(
        &self,
        easting: Distance,
        northing: Distance,
    ) -> Result<(Latitude, Longitude, kinavis_kernel::Angle, f64)> {
        let x = easting.metres() - self.parameters.false_easting.metres();
        let y = northing.metres() - self.parameters.false_northing.metres() + self.origin_northing;
        let (latitude, longitude, gamma, k) = self.reverse_raw(x, y)?;
        Ok((
            Latitude::from_degrees(latitude)?,
            Longitude::from_degrees(longitude)?,
            kinavis_kernel::Angle::from_degrees(gamma)?,
            k,
        ))
    }

    /// Forward projection about the central meridian, no false origin:
    /// `(x, y, γ°, k)`.
    fn forward_raw(&self, latitude: f64, longitude: f64) -> (f64, f64, f64, f64) {
        let k0 = self.parameters.scale;
        let mut lon = wrap180(longitude - self.parameters.central_meridian.degrees());
        let mut lat = latitude;
        let mut latsign = if lat.is_sign_negative() { -1.0 } else { 1.0 };
        let lonsign = if lon.is_sign_negative() { -1.0 } else { 1.0 };
        lon *= lonsign;
        lat *= latsign;
        let backside = lon > 90.0;
        if backside {
            if lat == 0.0 {
                latsign = -1.0;
            }
            lon = 180.0 - lon;
        }
        let (sphi, cphi) = sin_cos_degrees(lat);
        let (slam, clam) = sin_cos_degrees(lon);
        let (xip, etap, mut gamma, mut k) = if lat < 90.0 {
            let tau = sphi / cphi;
            let taup = conformal::taup(tau, self.es);
            (
                math::atan2(taup, clam),
                math::asinh(slam / math::hypot(taup, clam)),
                math::to_degrees(math::atan2(slam * taup, clam * math::hypot(1.0, taup))),
                math::sqrt(self.e2m + self.e2 * cphi * cphi) * math::hypot(1.0, tau)
                    / math::hypot(taup, clam),
            )
        } else {
            (core::f64::consts::FRAC_PI_2, 0.0, lon, self.c)
        };
        let (c0, s0) = (math::cos(2.0 * xip), math::sin(2.0 * xip));
        let (ch0, sh0) = (math::cosh(2.0 * etap), math::sinh(2.0 * etap));
        let twice_cos = Complex(2.0 * c0 * ch0, -2.0 * s0 * sh0);
        let (sum, derivative) = clenshaw(twice_cos, &self.alpha, 1.0);
        let derivative =
            Complex(1.0, 0.0) - derivative.1 + Complex(c0 * ch0, -s0 * sh0) * derivative.0;
        let zeta = Complex(xip, etap) + Complex(s0 * ch0, c0 * sh0) * sum;
        gamma -= math::to_degrees(math::atan2(derivative.1, derivative.0));
        k *= self.b1 * math::hypot(derivative.0, derivative.1);
        let (xi, eta) = (zeta.0, zeta.1);
        let y = self.a1
            * k0
            * (if backside {
                core::f64::consts::PI - xi
            } else {
                xi
            })
            * latsign;
        let x = self.a1 * k0 * eta * lonsign;
        if backside {
            gamma = 180.0 - gamma;
        }
        gamma = wrap180(gamma * latsign * lonsign);
        (x, y, gamma, k * k0)
    }

    /// Inverse of [`TransverseMercator::forward_raw`]: `(φ°, λ°, γ°, k)`.
    // Notation follows Karney (2011) and GeographicLib.
    #[allow(clippy::many_single_char_names)]
    fn reverse_raw(&self, x: f64, y: f64) -> Result<(f64, f64, f64, f64)> {
        let k0 = self.parameters.scale;
        let mut xi = y / (self.a1 * k0);
        let mut eta = x / (self.a1 * k0);
        let xisign = if xi.is_sign_negative() { -1.0 } else { 1.0 };
        let etasign = if eta.is_sign_negative() { -1.0 } else { 1.0 };
        xi *= xisign;
        eta *= etasign;
        let backside = xi > core::f64::consts::FRAC_PI_2;
        if backside {
            xi = core::f64::consts::PI - xi;
        }
        let (c0, s0) = (math::cos(2.0 * xi), math::sin(2.0 * xi));
        let (ch0, sh0) = (math::cosh(2.0 * eta), math::sinh(2.0 * eta));
        let twice_cos = Complex(2.0 * c0 * ch0, -2.0 * s0 * sh0);
        let (sum, derivative) = clenshaw(twice_cos, &self.beta, -1.0);
        let derivative =
            Complex(1.0, 0.0) - derivative.1 + Complex(c0 * ch0, -s0 * sh0) * derivative.0;
        let zeta = Complex(xi, eta) + Complex(s0 * ch0, c0 * sh0) * sum;
        let mut gamma = math::to_degrees(math::atan2(derivative.1, derivative.0));
        let mut k = self.b1 / math::hypot(derivative.0, derivative.1);
        let (xip, etap) = (zeta.0, zeta.1);
        let s = math::sinh(etap);
        let c = math::cos(xip).max(0.0);
        let r = math::hypot(s, c);
        let (mut lat, mut lon) = if r > 0.0 {
            let sxip = math::sin(xip);
            let tau = conformal::tau(sxip / r, self.es)?;
            gamma += math::to_degrees(math::atan2(sxip * math::tanh(etap), c));
            k *= math::sqrt(self.e2m + self.e2 / (1.0 + tau * tau)) * math::hypot(1.0, tau) * r;
            (
                math::to_degrees(math::atan(tau)),
                math::to_degrees(math::atan2(s, c)),
            )
        } else {
            k *= self.c;
            (90.0, 0.0)
        };
        lat *= xisign;
        if backside {
            lon = 180.0 - lon;
        }
        lon *= etasign;
        lon = wrap180(lon + self.parameters.central_meridian.degrees());
        if backside {
            gamma = 180.0 - gamma;
        }
        gamma = wrap180(gamma * xisign * etasign);
        Ok((lat, lon, gamma, k * k0))
    }
}

/// Sine and cosine of an angle in degrees, exact at 90°.
// Exact comparison: only an exact 90° has a cosine of exactly zero.
#[allow(clippy::float_cmp)]
pub(crate) fn sin_cos_degrees(degrees: f64) -> (f64, f64) {
    if degrees == 90.0 {
        return (1.0, 0.0);
    }
    let radians = math::to_radians(degrees);
    (math::sin(radians), math::cos(radians))
}

/// `p[0]·x^N + … + p[N]` by Horner's rule.
fn polynomial(coefficients: &[f64], x: f64) -> f64 {
    coefficients.iter().fold(0.0, |sum, c| sum * x + c)
}

/// Clenshaw sums of `Σ c_l sin(2lζ)` and of its derivative series
/// `Σ 2l c_l cos(2lζ)` with `twice_cos = 2 cos 2ζ`, coefficients `sign · c_l`;
/// returns `(y0, (z0, z1))` as GeographicLib's loop leaves them.
fn clenshaw(
    twice_cos: Complex,
    coefficients: &[f64; ORDER + 1],
    sign: f64,
) -> (Complex, (Complex, Complex)) {
    let (mut y0, mut y1) = (Complex(0.0, 0.0), Complex(0.0, 0.0));
    let (mut z0, mut z1) = (Complex(0.0, 0.0), Complex(0.0, 0.0));
    let mut l = ORDER;
    while l > 0 {
        let c = sign * coefficients.get(l).copied().unwrap_or(0.0);
        let weight = 2.0 * math::count_to_f64(l);
        y1 = twice_cos * y0 - y1 + Complex(c, 0.0);
        z1 = twice_cos * z0 - z1 + Complex(weight * c, 0.0);
        l -= 1;
        let c = sign * coefficients.get(l).copied().unwrap_or(0.0);
        let weight = 2.0 * math::count_to_f64(l);
        y0 = twice_cos * y1 - y0 + Complex(c, 0.0);
        z0 = twice_cos * z1 - z0 + Complex(weight * c, 0.0);
        l -= 1;
    }
    (y0, (z0, z1))
}

/// Complex number `(re, im)`.
#[derive(Debug, Clone, Copy)]
struct Complex(f64, f64);

impl Add for Complex {
    type Output = Self;
    fn add(self, other: Self) -> Self {
        Self(self.0 + other.0, self.1 + other.1)
    }
}

impl Sub for Complex {
    type Output = Self;
    fn sub(self, other: Self) -> Self {
        Self(self.0 - other.0, self.1 - other.1)
    }
}

impl Mul for Complex {
    type Output = Self;
    fn mul(self, other: Self) -> Self {
        Self(
            self.0 * other.0 - self.1 * other.1,
            self.0 * other.1 + self.1 * other.0,
        )
    }
}
