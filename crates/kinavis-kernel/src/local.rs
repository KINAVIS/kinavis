//! Local Cartesian frames and vectors typed by frame and unit.
//!
//! Near a point, positions and velocities are naturally expressed in a local
//! Cartesian frame — NED, ENU, or the vessel's forward-right-down. Mixing
//! frames is the classic sign error; adding a velocity to a displacement the
//! classic unit error.
//!
//! [`Vector3<F, U>`] carries both frame `F` and unit `U` in its type:
//! `Vector3<Ned, Distance>` and `Vector3<Enu, Distance>` cannot be added, nor
//! `Vector3<Ned, Distance>` and `Vector3<Ned, Speed>`. The same approach as
//! [`Direction<F>`](crate::Direction), in three dimensions:
//!
//! ```compile_fail
//! use kinavis_kernel::local::{Enu, Ned, Vector3};
//! use kinavis_kernel::Distance;
//!
//! let ned: Vector3<Ned, Distance> = Vector3::new(Distance::ZERO, Distance::ZERO, Distance::ZERO);
//! let enu: Vector3<Enu, Distance> = Vector3::new(Distance::ZERO, Distance::ZERO, Distance::ZERO);
//! let _ = ned + enu; // mismatched types: `Ned` is not `Enu`
//! ```
//!
//! A [`LocalFrame`] is a NED frame anchored at a point: it converts a
//! [`GeodeticPoint`] to a NED displacement from its origin and back, via
//! [`EcefPoint`] on a named [`Ellipsoid`].
//!
//! ```rust
//! use kinavis_kernel::geodesy::{Ellipsoid, GeodeticPoint, Height};
//! use kinavis_kernel::local::LocalFrame;
//! use kinavis_kernel::{Distance, Position};
//!
//! let origin = GeodeticPoint::new(
//!     "50°45.3'N 001°20.0'W".parse::<Position>()?,
//!     Height::above_ellipsoid(Distance::ZERO),
//! );
//! let frame = LocalFrame::at(origin, &Ellipsoid::WGS84)?;
//!
//! // A point one minute of latitude north of the origin.
//! let north = GeodeticPoint::new(
//!     "50°46.3'N 001°20.0'W".parse::<Position>()?,
//!     Height::above_ellipsoid(Distance::ZERO),
//! );
//! let ned = frame.ned_of(north)?;
//! // A minute of latitude on the ellipsoid at 50°N is 1854 m, not the
//! // sphere's 1852.
//! assert!((ned.north().metres() - 1854.1).abs() < 0.5);
//! assert!(ned.east().metres().abs() < 1e-6);
//! // The Earth curves away under a straight line: the point is slightly
//! // below the origin's horizontal plane.
//! assert!(ned.down().metres() > 0.0 && ned.down().metres() < 0.3);
//! # Ok::<(), kinavis_kernel::KernelError>(())
//! ```

use core::fmt;
use core::marker::PhantomData;
use core::ops::{Add, Div, Mul, Neg, Sub};

use crate::angle::TrueCourse;
use crate::error::Result;
use crate::geodesy::{EcefPoint, Ellipsoid, GeodeticPoint};
use crate::math;
use crate::units::{Distance, Speed};

mod sealed {
    pub trait Sealed {}
}

/// Frame of a [`Vector3`].
///
/// Sealed: only the frames below exist.
pub trait VectorFrame:
    sealed::Sealed + Copy + Clone + fmt::Debug + Eq + core::hash::Hash + Default + 'static
{
    /// Frame name.
    const NAME: &'static str;
    /// Axis names, in order.
    const AXES: [&'static str; 3];
}

/// North, east, down: the navigation frame.
///
/// Down is positive towards the Earth's centre, so height is a negative third
/// component (standard inertial/estimation convention).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Ned;

/// East, north, up: surveying and mapping frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Enu;

/// Forward, right, down: the vessel body frame.
///
/// Forward along the keel to the bow, right to starboard, down through the
/// keel. Conversion to a level frame needs the attitude, which is the
/// estimator's concern.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Body;

impl sealed::Sealed for Ned {}
impl sealed::Sealed for Enu {}
impl sealed::Sealed for Body {}

impl VectorFrame for Ned {
    const NAME: &'static str = "NED";
    const AXES: [&'static str; 3] = ["north", "east", "down"];
}

impl VectorFrame for Enu {
    const NAME: &'static str = "ENU";
    const AXES: [&'static str; 3] = ["east", "north", "up"];
}

impl VectorFrame for Body {
    const NAME: &'static str = "body";
    const AXES: [&'static str; 3] = ["forward", "right", "down"];
}

/// Quantity carried by each [`Vector3`] component.
///
/// Sealed: [`Distance`] and [`Speed`] only. Arithmetic is done in SI (m, m/s).
pub trait VectorUnit:
    sealed::Sealed
    + Copy
    + fmt::Debug
    + PartialEq
    + Add<Output = Self>
    + Sub<Output = Self>
    + Neg<Output = Self>
    + Mul<f64, Output = Self>
{
    /// Value in SI.
    fn si(self) -> f64;
    /// From an SI value known to be finite.
    fn from_si(value: f64) -> Self;
}

impl sealed::Sealed for Distance {}
impl sealed::Sealed for Speed {}

impl VectorUnit for Distance {
    fn si(self) -> f64 {
        self.metres()
    }

    fn from_si(value: f64) -> Self {
        Self::from_metres(value).unwrap_or(Self::ZERO)
    }
}

impl VectorUnit for Speed {
    fn si(self) -> f64 {
        self.metres_per_second()
    }

    fn from_si(value: f64) -> Self {
        Self::from_metres_per_second(value).unwrap_or(Self::ZERO)
    }
}

/// Three components of one quantity in one frame.
///
/// Components are in the frame's axis order ([`VectorFrame::AXES`]); each frame
/// has accessors named after its axes.
#[derive(Clone, Copy, PartialEq)]
pub struct Vector3<F: VectorFrame, U: VectorUnit> {
    components: [U; 3],
    frame: PhantomData<F>,
}

impl<F: VectorFrame, U: VectorUnit> Vector3<F, U> {
    /// Vector from components in axis order.
    #[must_use]
    pub const fn new(first: U, second: U, third: U) -> Self {
        Self {
            components: [first, second, third],
            frame: PhantomData,
        }
    }

    /// Components in axis order.
    #[must_use]
    pub const fn components(&self) -> [U; 3] {
        self.components
    }

    /// Length.
    #[must_use]
    pub fn magnitude(&self) -> U {
        let [a, b, c] = self.si();
        U::from_si(math::hypot(math::hypot(a, b), c))
    }

    /// Length of the projection on the first two axes (horizontal, for a level
    /// frame).
    #[must_use]
    pub fn horizontal_magnitude(&self) -> U {
        let [a, b, _] = self.si();
        U::from_si(math::hypot(a, b))
    }

    fn si(&self) -> [f64; 3] {
        self.components.map(U::si)
    }

    fn from_si(components: [f64; 3]) -> Self {
        Self {
            components: components.map(U::from_si),
            frame: PhantomData,
        }
    }
}

impl<U: VectorUnit> Vector3<Ned, U> {
    /// North component.
    #[must_use]
    pub const fn north(&self) -> U {
        self.components[0]
    }

    /// East component.
    #[must_use]
    pub const fn east(&self) -> U {
        self.components[1]
    }

    /// Down component.
    #[must_use]
    pub const fn down(&self) -> U {
        self.components[2]
    }

    /// Same vector in ENU.
    #[must_use]
    pub fn to_enu(self) -> Vector3<Enu, U> {
        Vector3::new(self.east(), self.north(), -self.down())
    }

    /// True direction of the horizontal component; `None` if negligible.
    ///
    /// For a velocity: course over ground.
    #[must_use]
    pub fn horizontal_direction(&self) -> Option<TrueCourse> {
        horizontal_direction(self.north().si(), self.east().si())
    }
}

impl<U: VectorUnit> Vector3<Enu, U> {
    /// East component.
    #[must_use]
    pub const fn east(&self) -> U {
        self.components[0]
    }

    /// North component.
    #[must_use]
    pub const fn north(&self) -> U {
        self.components[1]
    }

    /// Up component.
    #[must_use]
    pub const fn up(&self) -> U {
        self.components[2]
    }

    /// Same vector in NED.
    #[must_use]
    pub fn to_ned(self) -> Vector3<Ned, U> {
        Vector3::new(self.north(), self.east(), -self.up())
    }

    /// True direction of the horizontal component; `None` if negligible.
    #[must_use]
    pub fn horizontal_direction(&self) -> Option<TrueCourse> {
        horizontal_direction(self.north().si(), self.east().si())
    }
}

impl<U: VectorUnit> Vector3<Body, U> {
    /// Forward component.
    #[must_use]
    pub const fn forward(&self) -> U {
        self.components[0]
    }

    /// Starboard component.
    #[must_use]
    pub const fn right(&self) -> U {
        self.components[1]
    }

    /// Down component.
    #[must_use]
    pub const fn down(&self) -> U {
        self.components[2]
    }
}

/// Direction of a north/east pair; `None` when both are zero relative to the
/// larger.
fn horizontal_direction(north: f64, east: f64) -> Option<TrueCourse> {
    let scale = math::abs(north).max(math::abs(east));
    if scale < f64::MIN_POSITIVE {
        return None;
    }
    TrueCourse::wrap(math::to_degrees(math::atan2(east, north))).ok()
}

impl<F: VectorFrame, U: VectorUnit> Add for Vector3<F, U> {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        let [first, second, third] = self.components;
        let [x, y, z] = other.components;
        Self::new(first + x, second + y, third + z)
    }
}

impl<F: VectorFrame, U: VectorUnit> Sub for Vector3<F, U> {
    type Output = Self;

    fn sub(self, other: Self) -> Self {
        let [first, second, third] = self.components;
        let [x, y, z] = other.components;
        Self::new(first - x, second - y, third - z)
    }
}

impl<F: VectorFrame, U: VectorUnit> Neg for Vector3<F, U> {
    type Output = Self;

    fn neg(self) -> Self {
        let [a, b, c] = self.components;
        Self::new(-a, -b, -c)
    }
}

impl<F: VectorFrame, U: VectorUnit> Mul<f64> for Vector3<F, U> {
    type Output = Self;

    fn mul(self, factor: f64) -> Self {
        let [a, b, c] = self.components;
        Self::new(a * factor, b * factor, c * factor)
    }
}

impl<F: VectorFrame, U: VectorUnit> Div<f64> for Vector3<F, U> {
    type Output = Self;

    /// Division by zero yields infinite components, which `from_si` maps to
    /// zero; callers dividing by elapsed time check it first.
    fn div(self, divisor: f64) -> Self {
        Self::from_si(self.si().map(|value| value / divisor))
    }
}

impl<F: VectorFrame, U: VectorUnit> fmt::Debug for Vector3<F, U> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut debug = f.debug_struct(F::NAME);
        for (axis, component) in F::AXES.iter().zip(&self.components) {
            debug.field(axis, component);
        }
        debug.finish()
    }
}

impl<F: VectorFrame, U: VectorUnit + fmt::Display> fmt::Display for Vector3<F, U> {
    /// Formats as `(north 200.0 m, east 50.0 m, down -3.0 m)`.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("(")?;
        for (index, (axis, component)) in F::AXES.iter().zip(&self.components).enumerate() {
            if index > 0 {
                f.write_str(", ")?;
            }
            write!(f, "{axis} {component}")?;
        }
        f.write_str(")")
    }
}

#[cfg(feature = "serde")]
impl<F: VectorFrame, U: VectorUnit + serde::Serialize> serde::Serialize for Vector3<F, U> {
    /// Serialised as three components; the frame is in the type.
    fn serialize<S: serde::Serializer>(
        &self,
        serializer: S,
    ) -> core::result::Result<S::Ok, S::Error> {
        self.components.serialize(serializer)
    }
}

#[cfg(feature = "serde")]
impl<'de, F: VectorFrame, U: VectorUnit + serde::Deserialize<'de>> serde::Deserialize<'de>
    for Vector3<F, U>
{
    fn deserialize<D: serde::Deserializer<'de>>(
        deserializer: D,
    ) -> core::result::Result<Self, D::Error> {
        let [a, b, c] = <[U; 3]>::deserialize(deserializer)?;
        Ok(Self::new(a, b, c))
    }
}

/// NED frame anchored at a point.
///
/// Displacements are computed via ECEF, exact at any distance: no flat-Earth
/// approximation; curvature appears as a growing `down` component for level
/// points further from the origin.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LocalFrame {
    origin: GeodeticPoint,
    origin_ecef: EcefPoint,
    ellipsoid: Ellipsoid,
    /// Rows of the ECEF → NED rotation.
    rotation: [[f64; 3]; 3],
}

impl LocalFrame {
    /// Frame with origin at a point on an ellipsoid.
    ///
    /// # Errors
    ///
    /// As [`EcefPoint::from_geodetic`]: the origin height must be ellipsoidal.
    pub fn at(origin: GeodeticPoint, ellipsoid: &Ellipsoid) -> Result<Self> {
        let origin_ecef = EcefPoint::from_geodetic(origin, ellipsoid)?;
        let (sin_lat, cos_lat) = sin_cos(origin.position().latitude().radians());
        let (sin_lon, cos_lon) = sin_cos(origin.position().longitude().radians());
        Ok(Self {
            origin,
            origin_ecef,
            ellipsoid: *ellipsoid,
            rotation: [
                [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat],
                [-sin_lon, cos_lon, 0.0],
                [-cos_lat * cos_lon, -cos_lat * sin_lon, -sin_lat],
            ],
        })
    }

    /// Origin.
    #[must_use]
    pub const fn origin(&self) -> GeodeticPoint {
        self.origin
    }

    /// Ellipsoid.
    #[must_use]
    pub const fn ellipsoid(&self) -> &Ellipsoid {
        &self.ellipsoid
    }

    /// NED displacement of a point from the origin.
    ///
    /// # Errors
    ///
    /// As [`EcefPoint::from_geodetic`]: the height must be ellipsoidal.
    pub fn ned_of(&self, point: GeodeticPoint) -> Result<Vector3<Ned, Distance>> {
        let ecef = EcefPoint::from_geodetic(point, &self.ellipsoid)?;
        let delta = [
            ecef.x().metres() - self.origin_ecef.x().metres(),
            ecef.y().metres() - self.origin_ecef.y().metres(),
            ecef.z().metres() - self.origin_ecef.z().metres(),
        ];
        Ok(Vector3::from_si(self.rotation.map(|row| dot(row, delta))))
    }

    /// ENU displacement of a point from the origin.
    ///
    /// # Errors
    ///
    /// As [`LocalFrame::ned_of`].
    pub fn enu_of(&self, point: GeodeticPoint) -> Result<Vector3<Enu, Distance>> {
        self.ned_of(point).map(Vector3::to_enu)
    }

    /// Point at a NED displacement from the origin.
    ///
    /// Inverse of [`LocalFrame::ned_of`]; round trip holds to 0.1 mm for
    /// terrestrial displacements (property-tested).
    ///
    /// # Errors
    ///
    /// As [`EcefPoint::to_geodetic`]; not reachable for terrestrial
    /// displacements.
    pub fn point_from_ned(&self, displacement: Vector3<Ned, Distance>) -> Result<GeodeticPoint> {
        let local = displacement.si();
        // Orthonormal rotation: its transpose maps NED back.
        let column = |index: usize| {
            self.rotation
                .iter()
                .zip(local)
                .map(|(row, value)| row.get(index).copied().unwrap_or(0.0) * value)
                .sum::<f64>()
        };
        let ecef = EcefPoint::new(
            Distance::from_si(self.origin_ecef.x().metres() + column(0)),
            Distance::from_si(self.origin_ecef.y().metres() + column(1)),
            Distance::from_si(self.origin_ecef.z().metres() + column(2)),
        );
        ecef.to_geodetic(&self.ellipsoid)
    }

    /// Point at an ENU displacement from the origin.
    ///
    /// # Errors
    ///
    /// As [`LocalFrame::point_from_ned`].
    pub fn point_from_enu(&self, displacement: Vector3<Enu, Distance>) -> Result<GeodeticPoint> {
        self.point_from_ned(displacement.to_ned())
    }
}

fn sin_cos(radians: f64) -> (f64, f64) {
    (math::sin(radians), math::cos(radians))
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use crate::geodesy::Height;
    use crate::position::{Latitude, Longitude, Position};
    use alloc::format;

    fn metres(value: f64) -> Distance {
        Distance::from_metres(value).unwrap()
    }

    fn point(latitude: f64, longitude: f64, height: f64) -> GeodeticPoint {
        GeodeticPoint::new(
            Position::new(
                Latitude::from_degrees(latitude).unwrap(),
                Longitude::from_degrees(longitude).unwrap(),
            ),
            Height::above_ellipsoid(metres(height)),
        )
    }

    #[test]
    fn vectors_add_scale_and_measure_within_one_frame_and_unit() {
        let a: Vector3<Ned, Distance> = Vector3::new(metres(3.0), metres(4.0), metres(12.0));
        let b = Vector3::new(metres(1.0), metres(1.0), metres(1.0));
        let close = |vector: Vector3<Ned, Distance>, wanted: [f64; 3]| {
            vector
                .components()
                .iter()
                .zip(wanted)
                .all(|(got, wanted)| (got.metres() - wanted).abs() < 1e-9)
        };
        assert!(close(a + b, [4.0, 5.0, 13.0]));
        assert!(close(a - b, [2.0, 3.0, 11.0]));
        assert!(close(-a, [-3.0, -4.0, -12.0]));
        assert!(close(a * 2.0, [6.0, 8.0, 24.0]));
        assert!(close(a / 2.0, [1.5, 2.0, 6.0]));
        assert!((a.magnitude().metres() - 13.0).abs() < 1e-9);
        assert!((a.horizontal_magnitude().metres() - 5.0).abs() < 1e-9);
        let printed = format!("{a:?}");
        assert!(printed.starts_with("NED { north: "), "{printed}");
        assert!(format!("{a}").starts_with("(north "), "{a}");
    }

    #[test]
    fn ned_and_enu_are_the_same_vector_written_differently() {
        let ned: Vector3<Ned, Speed> = Vector3::new(
            Speed::from_metres_per_second(4.0).unwrap(),
            Speed::from_metres_per_second(1.0).unwrap(),
            Speed::from_metres_per_second(-0.5).unwrap(),
        );
        let enu = ned.to_enu();
        assert_eq!(enu.east(), ned.east());
        assert_eq!(enu.north(), ned.north());
        assert_eq!(enu.up().metres_per_second(), 0.5);
        assert_eq!(enu.to_ned(), ned);
        let course = ned.horizontal_direction().unwrap();
        assert!((course.degrees() - 14.036_243_467_926_479).abs() < 1e-9);
        assert_eq!(enu.horizontal_direction(), Some(course));
        let still: Vector3<Ned, Speed> = Vector3::new(Speed::ZERO, Speed::ZERO, Speed::ZERO);
        assert_eq!(still.horizontal_direction(), None);
    }

    #[test]
    fn a_local_frame_measures_displacements_from_its_origin() {
        let frame = LocalFrame::at(point(50.0, 0.0, 0.0), &Ellipsoid::WGS84).unwrap();
        // Origin is at zero.
        let zero = frame.ned_of(point(50.0, 0.0, 0.0)).unwrap();
        assert!(zero.magnitude().metres() < 1e-6);
        // A point straight up is straight up.
        let above = frame.ned_of(point(50.0, 0.0, 100.0)).unwrap();
        assert!(above.north().metres().abs() < 1e-6);
        assert!(above.east().metres().abs() < 1e-6);
        assert!((above.down().metres() + 100.0).abs() < 1e-6);
        // A point to the east lies east and slightly below the horizon.
        let east = frame.ned_of(point(50.0, 0.01, 0.0)).unwrap();
        assert!(east.east().metres() > 700.0 && east.east().metres() < 720.0);
        assert!(east.north().metres().abs() < 0.1);
        assert!(east.down().metres() > 0.0);
        assert_eq!(east.horizontal_direction().unwrap().degrees().round(), 90.0);
    }

    #[test]
    fn displacements_round_trip_through_the_frame() {
        let origins = [
            point(50.0, 0.0, 0.0),
            point(89.99, 179.99, 10.0),
            point(-33.9, 151.2, 50.0),
            point(0.0, -180.0, -30.0),
        ];
        for origin in origins {
            let frame = LocalFrame::at(origin, &Ellipsoid::WGS84).unwrap();
            let displacement: Vector3<Ned, Distance> =
                Vector3::new(metres(12_345.6), metres(-9_876.5), metres(432.1));
            let there = frame.point_from_ned(displacement).unwrap();
            let back = frame.ned_of(there).unwrap();
            let error = (back - displacement).magnitude().metres();
            assert!(error < 1e-4, "{origin}: off by {error} m, {back:?}");
            let enu_back = frame.enu_of(there).unwrap();
            let again = frame.point_from_enu(enu_back).unwrap();
            assert!(
                (again.position().latitude().degrees() - there.position().latitude().degrees())
                    .abs()
                    < 1e-9
            );
        }
    }

    #[test]
    fn the_frame_wants_an_ellipsoidal_origin() {
        let msl = GeodeticPoint::new(
            point(50.0, 0.0, 0.0).position(),
            Height::above_mean_sea_level(Distance::ZERO),
        );
        assert!(LocalFrame::at(msl, &Ellipsoid::WGS84).is_err());
    }
}
