//! The plane a field is laid out in: positions to planar points and back,
//! grid directions to true directions and back.

use kinavis::region::PlanarPoint;
use kinavis_grid::{ProjectedCrs, TmParameters};
use kinavis_kernel::error::ensure_range;
use kinavis_kernel::geodesy::{Datum, Ellipsoid, VerticalDatum};
use kinavis_kernel::{
    math, Angle, Distance, GridCourse, Height, KernelError, Position, TrueCourse,
};

use crate::error::{AgriError, Result};

/// Radius around the origin of a centred plane within which its scale error is
/// stated, metres: ≤ 5·10⁻⁶ there.
pub const FIELD_RADIUS_METRES: f64 = 20_000.0;

/// Lowest field height accepted by [`FieldPlane::centred_at`], metres above
/// the ellipsoid.
const MIN_HEIGHT_METRES: f64 = -1_000.0;

/// Highest field height accepted by [`FieldPlane::centred_at`], metres above
/// the ellipsoid.
const MAX_HEIGHT_METRES: f64 = 9_000.0;

/// Plane of a field: a transverse Mercator projection with its accuracy.
///
/// Every planar quantity of this crate — field boundaries, guidance lines,
/// passes — lives in one `FieldPlane`, and values from different planes are
/// not combined: that is [`AgriError::PlaneMismatch`].
///
/// - [`FieldPlane::centred_at`] *(the default)*: central meridian and origin
///   of northings through a point of the field, scale `(R + h)/R` for the
///   field's height `h` and the Gaussian radius `R = √(MN)` there. Distances
///   in the plane equal distances on the ground to `x²/2R²`: 3·10⁻⁷ at 5 km
///   from the origin, 5·10⁻⁶ at the 20 km radius ([`FIELD_RADIUS_METRES`])
///   beyond which points are refused. Height adds up to
///   `h · e² cos²φ / 2a` (5·10⁻⁷ at 1000 m on the equator, half that at
///   45°): the ground at height `h` is scaled differently north–south and
///   east–west, and a conformal plane has one scale in every direction.
/// - [`FieldPlane::grid`]: a zoned grid (UTM, Gauss–Krüger), for boundaries
///   and lines that arrive in it and must match it to the millimetre. Its
///   scale error is the grid's: up to 10⁻³ at the edge of a UTM zone, that is
///   3.6 cm between two passes of a 36 m boom; [`FieldPlane::scale_at`] gives
///   it at a point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FieldPlane {
    crs: ProjectedCrs,
    centred: bool,
}

impl FieldPlane {
    /// Plane centred on a point of the field at the field's mean height above
    /// the ellipsoid, on WGS 84.
    ///
    /// # Errors
    ///
    /// - [`KernelError::VerticalDatumMismatch`] for a height not above the
    ///   ellipsoid: convert a height above sea level with a geoid model first;
    /// - [`KernelError::OutOfRange`] for a height outside −1000…9000 m or an
    ///   origin closer than 1° to a pole.
    pub fn centred_at(origin: Position, height: Height) -> Result<Self> {
        if height.datum() != VerticalDatum::Ellipsoid {
            return Err(KernelError::VerticalDatumMismatch {
                required: VerticalDatum::Ellipsoid,
                found: height.datum(),
            }
            .into());
        }
        let h = height.value().metres();
        ensure_range(
            "field height above the ellipsoid, metres",
            h,
            MIN_HEIGHT_METRES,
            MAX_HEIGHT_METRES,
        )?;
        ensure_range(
            "latitude of a field plane's origin",
            origin.latitude().degrees(),
            -89.0,
            89.0,
        )?;
        let radius = gaussian_radius(Datum::WGS84.ellipsoid(), origin.latitude().radians());
        let crs = ProjectedCrs::new(
            Datum::WGS84,
            TmParameters {
                central_meridian: origin.longitude(),
                latitude_of_origin: origin.latitude(),
                scale: (radius + h) / radius,
                false_easting: Distance::ZERO,
                false_northing: Distance::ZERO,
            },
        )?;
        Ok(Self { crs, centred: true })
    }

    /// Plane of a zoned grid, for data that arrives in it.
    #[must_use]
    pub const fn grid(crs: ProjectedCrs) -> Self {
        Self {
            crs,
            centred: false,
        }
    }

    /// Projection.
    #[must_use]
    pub const fn crs(&self) -> &ProjectedCrs {
        &self.crs
    }

    /// Whether this is a centred plane, limited to [`FIELD_RADIUS_METRES`].
    #[must_use]
    pub const fn is_centred(&self) -> bool {
        self.centred
    }

    /// Planar point of a WGS 84 position.
    ///
    /// # Errors
    ///
    /// [`AgriError::BeyondFieldRadius`] for a centred plane and a position
    /// beyond its radius; as [`ProjectedCrs::project`] otherwise.
    pub fn to_plane(&self, position: Position) -> Result<PlanarPoint> {
        let projected = self.crs.project(position)?;
        let point = PlanarPoint::new(projected.easting, projected.northing);
        self.check_radius(point)?;
        Ok(point)
    }

    /// WGS 84 position of a planar point.
    ///
    /// # Errors
    ///
    /// [`AgriError::BeyondFieldRadius`] as [`FieldPlane::to_plane`]; as
    /// [`ProjectedCrs::unproject`] otherwise.
    pub fn to_position(&self, point: PlanarPoint) -> Result<Position> {
        self.check_radius(point)?;
        Ok(self.crs.unproject(point.east(), point.north())?)
    }

    /// Meridian convergence at a planar point: the angle from true north to
    /// grid north, clockwise positive.
    ///
    /// # Errors
    ///
    /// As [`FieldPlane::to_position`].
    pub fn convergence_at(&self, point: PlanarPoint) -> Result<Angle> {
        Ok(self.distortion_at(point)?.0)
    }

    /// Point scale at a planar point: plane distance over distance on the
    /// datum's ellipsoid. For a centred plane this includes the height
    /// factor `(R + h)/R`, so that plane distances are ground distances.
    ///
    /// # Errors
    ///
    /// As [`FieldPlane::to_position`].
    pub fn scale_at(&self, point: PlanarPoint) -> Result<f64> {
        Ok(self.distortion_at(point)?.1)
    }

    /// Grid direction of a true one at a planar point.
    ///
    /// # Errors
    ///
    /// As [`FieldPlane::to_position`].
    pub fn grid_course(&self, course: TrueCourse, at: PlanarPoint) -> Result<GridCourse> {
        Ok(course.to_grid(self.convergence_at(at)?))
    }

    /// True direction of a grid one at a planar point.
    ///
    /// # Errors
    ///
    /// As [`FieldPlane::to_position`].
    pub fn true_course(&self, course: GridCourse, at: PlanarPoint) -> Result<TrueCourse> {
        Ok(course.to_true(self.convergence_at(at)?))
    }

    /// `Ok` if `other` is this plane.
    pub(crate) fn ensure_same(&self, other: &Self) -> Result<()> {
        if self == other {
            Ok(())
        } else {
            Err(AgriError::PlaneMismatch)
        }
    }

    fn distortion_at(&self, point: PlanarPoint) -> Result<(Angle, f64)> {
        self.check_radius(point)?;
        let (_, _, convergence, scale) = self
            .crs
            .projection()
            .unproject(point.east(), point.north())?;
        Ok((convergence, scale))
    }

    fn check_radius(&self, point: PlanarPoint) -> Result<()> {
        if !self.centred {
            return Ok(());
        }
        let distance = math::hypot(point.east_metres(), point.north_metres());
        if distance > FIELD_RADIUS_METRES {
            return Err(AgriError::BeyondFieldRadius {
                distance,
                limit: FIELD_RADIUS_METRES,
            });
        }
        Ok(())
    }
}

/// Gaussian mean radius of curvature `√(MN)` at a latitude, metres.
fn gaussian_radius(ellipsoid: &Ellipsoid, latitude: f64) -> f64 {
    let a = ellipsoid.semi_major_axis().metres();
    let f = ellipsoid.flattening();
    let e2 = f * (2.0 - f);
    let s = math::sin(latitude);
    a * math::sqrt(1.0 - e2) / (1.0 - e2 * s * s)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn plane() -> Result<FieldPlane> {
        FieldPlane::centred_at(
            Position::from_degrees(52.0, 5.5)?,
            Height::above_ellipsoid(Distance::from_metres(40.0)?),
        )
    }

    #[test]
    fn the_origin_is_the_centre() -> Result<()> {
        let plane = plane()?;
        let origin = plane.to_plane(Position::from_degrees(52.0, 5.5)?)?;
        assert!(origin.east_metres().abs() < 1e-9 && origin.north_metres().abs() < 1e-9);
        assert!(plane.convergence_at(origin)?.degrees().abs() < 1e-12);
        // √(MN) at 52° N.
        let radius = 6_383_287.311;
        assert!((plane.scale_at(origin)? - (radius + 40.0) / radius).abs() < 1e-12);
        Ok(())
    }

    #[test]
    fn a_height_on_sea_level_is_refused() -> Result<()> {
        let error = FieldPlane::centred_at(
            Position::from_degrees(52.0, 5.5)?,
            Height::above_mean_sea_level(Distance::from_metres(40.0)?),
        );
        assert!(matches!(
            error,
            Err(AgriError::Navigation(
                kinavis::error::NavigationError::Kernel(KernelError::VerticalDatumMismatch { .. })
            ))
        ));
        Ok(())
    }

    #[test]
    fn points_beyond_the_radius_are_refused() -> Result<()> {
        let plane = plane()?;
        let far = Position::from_degrees(52.0, 5.9)?;
        assert!(matches!(
            plane.to_plane(far),
            Err(AgriError::BeyondFieldRadius { .. })
        ));
        let grid = FieldPlane::grid(*plane.crs());
        assert!(grid.to_plane(far).is_ok());
        assert_eq!(grid.ensure_same(&plane), Err(AgriError::PlaneMismatch));
        Ok(())
    }
}
