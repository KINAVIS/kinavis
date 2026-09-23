//! Horizontal datums and datum transformation.
//!
//! Pre-satellite charts are referred to national datums: a regional ellipsoid
//! fixed at a fundamental point. The same coordinates on such a datum and on
//! WGS 84 are different places — tens of metres in North America, hundreds in
//! Japan — so a chart position must be shifted before plotting a GNSS fix
//! against it, as an explicit step.
//!
//! ```rust
//! use kinavis_kernel::geodesy::Datum;
//! use kinavis_kernel::Position;
//!
//! // A light, as an old chart of Tokyo Bay gives it.
//! let charted: Position = "35°39.0'N 139°45.0'E".parse()?;
//! let wgs84 = Datum::TOKYO.to_wgs84(charted)?;
//!
//! // Some 460 m away: a quarter of a mile, on a chart that says "Tokyo".
//! assert_eq!(format!("{wgs84:.3}"), "35°39.196'N 139°44.807'E");
//! assert_eq!(Datum::TOKYO.accuracy().metres().round(), 29.0);
//!
//! // And a GNSS fix, as it is to be plotted on that chart.
//! let plotted = Datum::TOKYO.from_wgs84(wgs84)?;
//! assert_eq!(format!("{plotted:.3}"), "35°39.000'N 139°45.000'E");
//! # Ok::<(), kinavis_kernel::KernelError>(())
//! ```

use core::fmt;

use super::{EcefPoint, Ellipsoid, GeodeticPoint, Height};
use crate::error::{ensure_finite, KernelError, Result};
use crate::math;
use crate::position::Position;
use crate::units::Distance;

/// Radians per arc-second.
const RADIANS_PER_ARC_SECOND: f64 = core::f64::consts::PI / (180.0 * 3600.0);

/// Seven-parameter Helmert transformation between geocentric frames:
/// translation, small rotation, scale.
///
/// Parameters come from EPSG, a national survey or a chart note, in one of two
/// rotation sign conventions: *position vector* (rotates the point) and
/// *coordinate frame* (rotates the axes; opposite sign). Each has its own
/// constructor so parameters are entered as published.
///
/// The rotation matrix is the small-angle `I + [r]×` used by the definitions,
/// not an exact rotation: the published parameters were fitted with it.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(try_from = "StoredHelmert", into = "StoredHelmert")
)]
pub struct Helmert {
    /// Metres.
    translation: [f64; 3],
    /// Arc-seconds, position vector convention.
    rotation: [f64; 3],
    /// Parts per million.
    scale_ppm: f64,
}

impl Helmert {
    /// Identity.
    pub const IDENTITY: Self = Self {
        translation: [0.0; 3],
        rotation: [0.0; 3],
        scale_ppm: 0.0,
    };

    /// Translation only, metres: the three-parameter form most shifts are
    /// published in.
    #[must_use]
    pub const fn translation(dx: f64, dy: f64, dz: f64) -> Self {
        Self {
            translation: [dx, dy, dz],
            rotation: [0.0; 3],
            scale_ppm: 0.0,
        }
    }

    /// Seven parameters, position vector convention (EPSG 9606, Bursa-Wolf as
    /// used in Europe): translations in m, rotations in arc-seconds, scale in
    /// ppm.
    ///
    /// # Errors
    ///
    /// [`KernelError::NotFinite`] for a non-finite parameter;
    /// [`KernelError::OutOfRange`] for a scale ≤ −1 000 000 ppm (collapses the
    /// frame).
    pub fn position_vector(
        translation: [f64; 3],
        rotation: [f64; 3],
        scale_ppm: f64,
    ) -> Result<Self> {
        for value in translation {
            ensure_finite("Helmert translation", value)?;
        }
        for value in rotation {
            ensure_finite("Helmert rotation", value)?;
        }
        ensure_finite("Helmert scale difference", scale_ppm)?;
        if scale_ppm <= -1e6 {
            return Err(KernelError::OutOfRange {
                parameter: "Helmert scale difference",
                value: scale_ppm,
                min: -1e6,
                max: f64::INFINITY,
            });
        }
        Ok(Self {
            translation,
            rotation,
            scale_ppm,
        })
    }

    /// Seven parameters, coordinate frame convention (EPSG 9607, as used in the
    /// US and Australia): as [`Helmert::position_vector`] with rotation signs
    /// reversed.
    ///
    /// # Errors
    ///
    /// As [`Helmert::position_vector`].
    pub fn coordinate_frame(
        translation: [f64; 3],
        rotation: [f64; 3],
        scale_ppm: f64,
    ) -> Result<Self> {
        Self::position_vector(
            translation,
            [-rotation[0], -rotation[1], -rotation[2]],
            scale_ppm,
        )
    }

    /// Translation, m.
    #[must_use]
    pub const fn translation_metres(&self) -> [f64; 3] {
        self.translation
    }

    /// Rotation, arc-seconds, position vector convention.
    #[must_use]
    pub const fn rotation_arc_seconds(&self) -> [f64; 3] {
        self.rotation
    }

    /// Scale difference, ppm.
    #[must_use]
    pub const fn scale_ppm(&self) -> f64 {
        self.scale_ppm
    }

    /// Whether every parameter is zero.
    #[must_use]
    pub fn is_identity(&self) -> bool {
        *self == Self::IDENTITY
    }

    fn rotation_radians(&self) -> [f64; 3] {
        [
            self.rotation[0] * RADIANS_PER_ARC_SECOND,
            self.rotation[1] * RADIANS_PER_ARC_SECOND,
            self.rotation[2] * RADIANS_PER_ARC_SECOND,
        ]
    }

    fn scale(&self) -> f64 {
        1.0 + self.scale_ppm * 1e-6
    }

    /// Forward transform: `X′ = T + (1 + s) (I + [r]×) X`.
    #[must_use]
    pub fn apply(&self, point: EcefPoint) -> EcefPoint {
        let [rx, ry, rz] = self.rotation_radians();
        let [tx, ty, tz] = self.translation;
        let scale = self.scale();
        let (x, y, z) = (point.x, point.y, point.z);
        EcefPoint {
            x: tx + scale * (x - rz * y + ry * z),
            y: ty + scale * (rz * x + y - rx * z),
            z: tz + scale * (-ry * x + rx * y + z),
        }
    }

    /// Inverse transform, exact (not the usual sign-reversal approximation), so
    /// a round trip closes to rounding precision.
    ///
    /// For `M = I + [r]×`, `M⁻¹ = (I − [r]× + r rᵀ) / (1 + |r|²)`.
    #[must_use]
    pub fn apply_inverse(&self, point: EcefPoint) -> EcefPoint {
        let [rx, ry, rz] = self.rotation_radians();
        let [tx, ty, tz] = self.translation;
        let scale = self.scale();
        let (x, y, z) = (
            (point.x - tx) / scale,
            (point.y - ty) / scale,
            (point.z - tz) / scale,
        );
        let along = rx * x + ry * y + rz * z;
        let norm = 1.0 + rx * rx + ry * ry + rz * rz;
        EcefPoint {
            x: (x + rz * y - ry * z + rx * along) / norm,
            y: (-rz * x + y + rx * z + ry * along) / norm,
            z: (ry * x - rx * y + z + rz * along) / norm,
        }
    }
}

/// Horizontal geodetic datum: ellipsoid and its transformation to WGS 84.
///
/// Each provided datum uses the EPSG transformation for its home area, with the
/// accuracy EPSG states: a few metres for a modern seven-parameter fit, tens of
/// metres for a continental mean translation. This is the accuracy of the
/// shift, not of the chart; a chart's own datum note, when present, takes
/// precedence over this table.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Datum {
    name: &'static str,
    ellipsoid: Ellipsoid,
    to_wgs84: Helmert,
    accuracy_metres: f64,
}

/// Serialised form, position vector convention. Deserialisation goes through
/// [`Helmert::position_vector`], rejecting `NaN` and frame-inverting scales.
#[cfg(feature = "serde")]
#[derive(serde::Serialize, serde::Deserialize)]
struct StoredHelmert {
    translation: [f64; 3],
    rotation: [f64; 3],
    scale_ppm: f64,
}

#[cfg(feature = "serde")]
impl TryFrom<StoredHelmert> for Helmert {
    type Error = KernelError;

    fn try_from(stored: StoredHelmert) -> Result<Self> {
        Self::position_vector(stored.translation, stored.rotation, stored.scale_ppm)
    }
}

#[cfg(feature = "serde")]
impl From<Helmert> for StoredHelmert {
    fn from(helmert: Helmert) -> Self {
        Self {
            translation: helmert.translation,
            rotation: helmert.rotation,
            scale_ppm: helmert.scale_ppm,
        }
    }
}

impl Datum {
    /// WGS 84: the GNSS datum, and the datum of every [`Position`].
    pub const WGS84: Self = Self {
        name: "WGS 84",
        ellipsoid: Ellipsoid::WGS84,
        to_wgs84: Helmert::IDENTITY,
        accuracy_metres: 0.0,
    };

    /// NAD83, on GRS 80. Equivalent to WGS 84 at chart accuracy: EPSG 1188
    /// (identity), 4 m.
    pub const NAD83: Self = Self {
        name: "NAD83",
        ellipsoid: Ellipsoid::GRS80,
        to_wgs84: Helmert::IDENTITY,
        accuracy_metres: 4.0,
    };

    /// ED50, on International 1924: western European charts before ETRS 89.
    /// EPSG 1133 (Europe mean), 10 m.
    pub const ED50: Self = Self {
        name: "ED50",
        ellipsoid: Ellipsoid::INTERNATIONAL_1924,
        to_wgs84: Helmert::translation(-87.0, -98.0, -121.0),
        accuracy_metres: 10.0,
    };

    /// NAD27, on Clarke 1866: older US charts. EPSG 1173 (CONUS mean), 10 m.
    pub const NAD27: Self = Self {
        name: "NAD27",
        ellipsoid: Ellipsoid::CLARKE_1866,
        to_wgs84: Helmert::translation(-8.0, 160.0, 176.0),
        accuracy_metres: 10.0,
    };

    /// OSGB36, on Airy 1830. EPSG 1314 (seven-parameter, Great Britain), 2 m.
    pub const OSGB36: Self = Self {
        name: "OSGB36",
        ellipsoid: Ellipsoid::AIRY_1830,
        to_wgs84: Helmert {
            translation: [446.448, -125.157, 542.06],
            rotation: [0.15, 0.247, 0.842],
            scale_ppm: -20.489,
        },
        accuracy_metres: 2.0,
    };

    /// Pulkovo 1942, on Krassowsky 1940: Russian and former-USSR charts. EPSG
    /// 5044 (GOST R 51794-2001, Russia), 3 m. Published in the coordinate frame
    /// convention; stored here in the position vector convention.
    pub const PULKOVO_1942: Self = Self {
        name: "Pulkovo 1942",
        ellipsoid: Ellipsoid::KRASSOWSKY_1940,
        to_wgs84: Helmert {
            translation: [23.57, -140.95, -79.8],
            rotation: [0.0, 0.35, 0.79],
            scale_ppm: -0.22,
        },
        accuracy_metres: 3.0,
    };

    /// Tokyo, on Bessel 1841: Japanese and Korean charts before JGD 2000. EPSG
    /// 1230 (Japan and South Korea mean), 29 m; the shift itself exceeds 400 m.
    pub const TOKYO: Self = Self {
        name: "Tokyo",
        ellipsoid: Ellipsoid::BESSEL_1841,
        to_wgs84: Helmert::translation(-148.0, 507.0, 685.0),
        accuracy_metres: 29.0,
    };

    /// DHDN (Potsdam), on Bessel 1841: German charts before ETRS 89. EPSG 1777
    /// (Germany), 3 m.
    pub const DHDN: Self = Self {
        name: "DHDN",
        ellipsoid: Ellipsoid::BESSEL_1841,
        to_wgs84: Helmert {
            translation: [598.1, 73.7, 418.2],
            rotation: [0.202, 0.045, -2.455],
            scale_ppm: 6.7,
        },
        accuracy_metres: 3.0,
    };

    /// AGD66, on the Australian National Spheroid. EPSG 15788 (Australia mean),
    /// 5 m.
    pub const AGD66: Self = Self {
        name: "AGD66",
        ellipsoid: Ellipsoid::AUSTRALIAN_NATIONAL,
        to_wgs84: Helmert::translation(-127.8, -52.3, 152.9),
        accuracy_metres: 5.0,
    };

    /// SAD69, on GRS 1967 Modified. EPSG 1864 (continental mean), 19 m.
    pub const SAD69: Self = Self {
        name: "SAD69",
        ellipsoid: Ellipsoid::AUSTRALIAN_NATIONAL,
        to_wgs84: Helmert::translation(-57.0, 1.0, -41.0),
        accuracy_metres: 19.0,
    };

    /// Datum from its ellipsoid, transformation to WGS 84 and stated accuracy:
    /// for datums not in the table, or with parameters from a chart note.
    #[must_use]
    pub fn new(
        name: &'static str,
        ellipsoid: Ellipsoid,
        to_wgs84: Helmert,
        accuracy: Distance,
    ) -> Self {
        Self {
            name,
            ellipsoid,
            to_wgs84,
            accuracy_metres: math::abs(accuracy.metres()),
        }
    }

    /// Datum name, as in a chart note.
    #[must_use]
    pub const fn name(&self) -> &'static str {
        self.name
    }

    /// Reference ellipsoid.
    #[must_use]
    pub const fn ellipsoid(&self) -> &Ellipsoid {
        &self.ellipsoid
    }

    /// Transformation from this datum's geocentric frame to WGS 84.
    #[must_use]
    pub const fn to_wgs84_helmert(&self) -> &Helmert {
        &self.to_wgs84
    }

    /// Stated accuracy of the shift; zero for WGS 84.
    #[must_use]
    pub fn accuracy(&self) -> Distance {
        Distance::from_metres(self.accuracy_metres).unwrap_or(Distance::ZERO)
    }

    /// Chart position on this datum → WGS 84.
    ///
    /// Placed on the datum ellipsoid at zero height, transformed via ECEF, read
    /// back on WGS 84. A ship is within ~100 m of the ellipsoid and the shift
    /// tilts the normal by ≤ 0.1 mrad, so dropping height costs about 1 cm.
    ///
    /// # Errors
    ///
    /// [`KernelError::Indeterminate`] if the transformed point has no geodetic
    /// position; impossible with published parameters, possible with arbitrary
    /// [`Helmert`] values.
    pub fn to_wgs84(&self, position: Position) -> Result<Position> {
        if self.to_wgs84.is_identity() && self.ellipsoid == Ellipsoid::WGS84 {
            return Ok(position);
        }
        let point = GeodeticPoint::new(position, Height::above_ellipsoid(Distance::ZERO));
        let geocentric = EcefPoint::from_geodetic(point, &self.ellipsoid)?;
        let shifted = self.to_wgs84.apply(geocentric);
        Ok(shifted.to_geodetic(&Ellipsoid::WGS84)?.position())
    }

    /// WGS 84 position (GNSS fix) → position on this datum, for plotting on its
    /// chart.
    ///
    /// Exact inverse of [`Datum::to_wgs84`] apart from the height dropped at
    /// each end: round trip closes to ~1 cm in the datum's home area, a
    /// decimetre or two on the far side of the Earth where its ellipsoid is
    /// kilometres from WGS 84.
    ///
    /// # Errors
    ///
    /// As [`Datum::to_wgs84`].
    pub fn from_wgs84(&self, position: Position) -> Result<Position> {
        if self.to_wgs84.is_identity() && self.ellipsoid == Ellipsoid::WGS84 {
            return Ok(position);
        }
        let point = GeodeticPoint::new(position, Height::above_ellipsoid(Distance::ZERO));
        let geocentric = EcefPoint::from_geodetic(point, &Ellipsoid::WGS84)?;
        let shifted = self.to_wgs84.apply_inverse(geocentric);
        Ok(shifted.to_geodetic(&self.ellipsoid)?.position())
    }
}

impl fmt::Display for Datum {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name)
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use crate::position::{Latitude, Longitude};

    fn position(latitude: f64, longitude: f64) -> Position {
        Position::new(
            Latitude::from_degrees(latitude).unwrap(),
            Longitude::from_degrees(longitude).unwrap(),
        )
    }

    /// Metres between two nearby positions, on a sphere.
    fn metres_apart(a: Position, b: Position) -> f64 {
        let dlat = math::to_radians(b.latitude().degrees() - a.latitude().degrees());
        let dlon = b.longitude_difference(a).radians();
        let cos = math::cos(math::to_radians(a.latitude().degrees()));
        6_371_000.0 * math::hypot(dlat, dlon * cos)
    }

    #[test]
    fn the_identity_leaves_a_point_alone_and_a_translation_moves_it() {
        let point = EcefPoint::new(
            Distance::from_metres(1.0).unwrap(),
            Distance::from_metres(2.0).unwrap(),
            Distance::from_metres(3.0).unwrap(),
        );
        assert_eq!(Helmert::IDENTITY.apply(point), point);
        assert!(Helmert::IDENTITY.is_identity());
        let moved = Helmert::translation(10.0, -20.0, 30.0).apply(point);
        assert_eq!(moved.x().metres(), 11.0);
        assert_eq!(moved.y().metres(), -18.0);
        assert_eq!(moved.z().metres(), 33.0);
    }

    #[test]
    fn the_two_conventions_differ_by_the_sign_of_the_rotation() {
        let pv = Helmert::position_vector([1.0, 2.0, 3.0], [0.1, -0.2, 0.3], 1.5).unwrap();
        let cf = Helmert::coordinate_frame([1.0, 2.0, 3.0], [-0.1, 0.2, -0.3], 1.5).unwrap();
        assert_eq!(pv, cf);
        assert_eq!(pv.rotation_arc_seconds(), [0.1, -0.2, 0.3]);
        assert_eq!(pv.translation_metres(), [1.0, 2.0, 3.0]);
        assert_eq!(pv.scale_ppm(), 1.5);
    }

    #[test]
    fn wild_parameters_are_refused() {
        assert!(Helmert::position_vector([f64::NAN, 0.0, 0.0], [0.0; 3], 0.0).is_err());
        assert!(Helmert::position_vector([0.0; 3], [0.0, f64::INFINITY, 0.0], 0.0).is_err());
        assert!(Helmert::position_vector([0.0; 3], [0.0; 3], f64::NAN).is_err());
        assert!(matches!(
            Helmert::coordinate_frame([0.0; 3], [0.0; 3], -1e6),
            Err(KernelError::OutOfRange { .. })
        ));
    }

    #[test]
    fn the_inverse_is_exact_not_the_reversed_parameters() {
        let helmert = Datum::OSGB36.to_wgs84;
        let point = EcefPoint::new(
            Distance::from_metres(3_874_938.849).unwrap(),
            Distance::from_metres(116_218.624).unwrap(),
            Distance::from_metres(5_047_168.208).unwrap(),
        );
        let back = helmert.apply_inverse(helmert.apply(point));
        assert!(back.chord_to(point).metres() < 1e-9, "{back:?}");

        // The sign-reversal approximation used in textbooks is off by a
        // fraction of a millimetre here: second-order in a 20 ppm scale and 1″
        // rotation.
        let reversed = Helmert {
            translation: [-446.448, 125.157, -542.06],
            rotation: [-0.15, -0.247, -0.842],
            scale_ppm: 20.489,
        };
        let approximate = reversed.apply(helmert.apply(point));
        assert!(approximate.chord_to(point).metres() > 1e-6);
    }

    #[test]
    fn wgs84_and_nad83_shift_nothing() {
        let here = position(38.9, -77.0);
        assert_eq!(Datum::WGS84.to_wgs84(here).unwrap(), here);
        assert_eq!(Datum::WGS84.from_wgs84(here).unwrap(), here);
        assert_eq!(Datum::WGS84.accuracy(), Distance::ZERO);
        // NAD83 is on GRS 80; its 0.1 mm polar difference is below position
        // resolution.
        let shifted = Datum::NAD83.to_wgs84(here).unwrap();
        assert!(metres_apart(here, shifted) < 1e-3);
    }

    #[test]
    fn the_datums_are_named_and_carry_their_accuracy() {
        assert_eq!(Datum::OSGB36.name(), "OSGB36");
        assert_eq!(alloc::format!("{}", Datum::PULKOVO_1942), "Pulkovo 1942");
        assert!((Datum::TOKYO.accuracy().metres() - 29.0).abs() < 1e-9);
        assert_eq!(*Datum::ED50.ellipsoid(), Ellipsoid::INTERNATIONAL_1924);
        assert_eq!(
            Datum::NAD27.to_wgs84_helmert().translation_metres(),
            [-8.0, 160.0, 176.0]
        );
        let own = Datum::new(
            "chart note",
            Ellipsoid::INTERNATIONAL_1924,
            Helmert::translation(-84.0, -97.0, -117.0),
            Distance::from_metres(-5.0).unwrap(),
        );
        assert_eq!(own.name(), "chart note");
        assert!((own.accuracy().metres() - 5.0).abs() < 1e-9);
    }
}
