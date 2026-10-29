//! Intersection and resection on the plane of a map grid.
//!
//! On a conformal grid a short sight line is straight and its bearing is the
//! grid bearing, so the problems of the topographic map are plane geometry:
//!
//! - **intersection** (прямая засечка): bearings to an unknown point from two
//!   known ones;
//! - **resection by bearings** (обратная засечка): bearings from the unknown
//!   observer to two or more landmarks, solved by least squares;
//! - **three-point resection** (задача Потенота): directions to three
//!   landmarks read on any circle with an unknown zero — a theodolite, or a
//!   compass of unknown error — solved by Tienstra's formula, which also
//!   yields that zero.
//!
//! Points are given on one grid ([`crate::MapGrid`]); a landmark known in
//! another (an MGRS reference, an СК-42 sheet) is brought to it through its
//! position first. The plane is exact to the curvature of a sight line on
//! the grid, centimetres over ten kilometres in a UTM zone.

// A, B, C, P and their angles as in Tienstra's formula; t, s, u, v as the
// parameters and directions of two lines.
#![allow(clippy::many_single_char_names)]

use kinavis::error::NavigationError;
use kinavis_kernel::{math, wrap180, wrap360, Angle, Distance, GridBearing};

use crate::error::{LandError, Result};
use crate::grid::GridPoint;

/// Sine of the smallest angle at which two lines are taken to cross: 10⁻⁹ rad.
const MIN_CUT: f64 = 1e-9;

/// Figure strength below which a three-point resection is refused: the
/// greatest `|cot A − cot α|`, about twice the observer's distance from the
/// danger circle over its radius.
pub const DANGER_LIMIT: f64 = 1e-6;

/// Bearing from the observer to a landmark of known grid coordinates.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sighting {
    /// Landmark.
    pub landmark: GridPoint,
    /// Grid bearing from the observer to it.
    pub bearing: GridBearing,
}

/// A resection by bearings, with the agreement of its lines.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlaneFix {
    /// Most probable point: least squares of the distances to the lines.
    pub point: GridPoint,
    /// Number of lines.
    pub lines: usize,
    /// RMS distance from the point to the lines; zero for two.
    pub rms_residual: Distance,
    /// Greatest distance from the point to a line.
    pub greatest_residual: Distance,
}

/// A three-point resection.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ThreePointFix {
    /// The observer.
    pub point: GridPoint,
    /// Grid bearing of the reading circle's zero: a reading plus it is a grid
    /// bearing. For a compass, its total error against grid north.
    pub orientation: Angle,
    /// Distance from the circle through the landmarks over its radius: the
    /// strength of the fix, poor below about 0.2.
    pub danger: f64,
}

/// Grid bearing from one point to another.
///
/// # Errors
///
/// [`kinavis_kernel::KernelError::Indeterminate`] for coincident points.
pub fn grid_bearing(from: GridPoint, to: GridPoint) -> Result<GridBearing> {
    let (east, north) = difference(from, to);
    if east == 0.0 && north == 0.0 {
        return Err(kinavis_kernel::KernelError::Indeterminate {
            quantity: "bearing between coincident points",
        }
        .into());
    }
    Ok(GridBearing::from_degrees_wrapped(math::to_degrees(
        math::atan2(east, north),
    )))
}

/// Grid distance between two points.
#[must_use]
pub fn grid_distance(from: GridPoint, to: GridPoint) -> Distance {
    let (east, north) = difference(from, to);
    Distance::from_metres(math::hypot(east, north)).unwrap_or(Distance::ZERO)
}

/// Intersection: the point seen on `from_first` from `first` and on
/// `from_second` from `second`.
///
/// # Errors
///
/// [`NavigationError::Parallel`] for lines crossing at under 10⁻⁹ rad;
/// [`NavigationError::NoSolution`] if they cross behind either known point.
pub fn intersection(
    first: GridPoint,
    from_first: GridBearing,
    second: GridPoint,
    from_second: GridBearing,
) -> Result<GridPoint> {
    let u = unit(from_first);
    let v = unit(from_second);
    let cross = u.0 * v.1 - u.1 * v.0;
    if math::abs(cross) < MIN_CUT {
        return Err(NavigationError::Parallel {
            context: "intersection lines",
        }
        .into());
    }
    let (east, north) = difference(first, second);
    // first + t·u = second + s·v
    let t = (east * v.1 - north * v.0) / cross;
    let s = (east * u.1 - north * u.0) / cross;
    if t <= 0.0 || s <= 0.0 {
        return Err(NavigationError::NoSolution {
            context: "intersection ahead of both points",
        }
        .into());
    }
    let (e, n) = first.metres();
    GridPoint::from_metres(e + t * u.0, n + t * u.1)
}

/// Resection by bearings to two or more landmarks.
///
/// Least squares of the perpendicular distances to the lines: the most
/// probable point for independent bearings of equal accuracy. A common
/// error in all of them — an uncorrected compass — moves the true point out
/// of the figure, which least squares cannot see; [`three_point_resection`]
/// solves for it.
///
/// # Errors
///
/// [`kinavis_kernel::KernelError::InsufficientData`] for fewer than two
/// sightings; [`NavigationError::Parallel`] if all lines are parallel.
pub fn resection(sightings: &[Sighting]) -> Result<PlaneFix> {
    let Some(first) = sightings.first() else {
        return Err(too_few(0));
    };
    if sightings.len() < 2 {
        return Err(too_few(sightings.len()));
    }
    // Relative to the first landmark, for conditioning.
    let origin = first.landmark;
    let (mut ee, mut en, mut nn, mut eo, mut no) = (0.0, 0.0, 0.0, 0.0, 0.0);
    for sighting in sightings {
        let (normal, offset) = normal_form(origin, sighting);
        ee += normal.0 * normal.0;
        en += normal.0 * normal.1;
        nn += normal.1 * normal.1;
        eo += normal.0 * offset;
        no += normal.1 * offset;
    }
    let determinant = ee * nn - en * en;
    let count = kinavis_kernel::math::count_to_f64(sightings.len());
    if determinant < MIN_CUT * MIN_CUT * count * count {
        return Err(NavigationError::Parallel {
            context: "resection lines",
        }
        .into());
    }
    let east = (nn * eo - en * no) / determinant;
    let north = (ee * no - en * eo) / determinant;
    let (mut squares, mut greatest) = (0.0, 0.0_f64);
    for sighting in sightings {
        let (normal, offset) = normal_form(origin, sighting);
        let residual = math::abs(normal.0 * east + normal.1 * north - offset);
        squares += residual * residual;
        greatest = greatest.max(residual);
    }
    let (e0, n0) = origin.metres();
    Ok(PlaneFix {
        point: GridPoint::from_metres(e0 + east, n0 + north)?,
        lines: sightings.len(),
        rms_residual: Distance::from_metres(math::sqrt(squares / count))?,
        greatest_residual: Distance::from_metres(greatest)?,
    })
}

/// Three-point resection from directions to three landmarks read clockwise
/// on a circle with an unknown zero.
///
/// Tienstra's formula: with the triangle's angles `A`, `B`, `C` and the
/// angles `α`, `β`, `γ` the observer sees between the other two landmarks,
/// the observer is the barycentre of the landmarks weighted by
/// `1 / (cot A − cot α)` and its likes. Valid inside the triangle and out.
///
/// # Errors
///
/// [`NavigationError::Parallel`] for collinear landmarks;
/// [`LandError::DangerCircle`] for a figure strength under [`DANGER_LIMIT`]:
/// the observer on the circle through the landmarks, where the readings fix
/// no point; [`kinavis_kernel::KernelError::Indeterminate`]
/// for coincident landmarks or two equal readings.
pub fn three_point_resection(
    landmarks: [GridPoint; 3],
    readings: [Angle; 3],
) -> Result<ThreePointFix> {
    let [a, b, c] = landmarks.map(GridPoint::metres);
    let [ra, rb, rc] = readings.map(Angle::degrees);
    // Signed area: positive when A, B, C run anticlockwise.
    let area = (b.0 - a.0) * (c.1 - a.1) - (c.0 - a.0) * (b.1 - a.1);
    let scale = squared(b.0 - a.0, b.1 - a.1)
        .max(squared(c.0 - a.0, c.1 - a.1))
        .max(squared(c.0 - b.0, c.1 - b.1));
    if scale.is_nan() || scale <= 0.0 {
        return Err(kinavis_kernel::KernelError::Indeterminate {
            quantity: "resection on coincident landmarks",
        }
        .into());
    }
    if math::abs(area) < MIN_CUT * scale {
        return Err(NavigationError::Parallel {
            context: "resection landmarks in a line",
        }
        .into());
    }
    // Tienstra wants the landmarks clockwise and each observed angle the
    // clockwise sweep between the other two.
    let ((p, q, r), (rp, rq, rr)) = if area > 0.0 {
        ((a, c, b), (ra, rc, rb))
    } else {
        ((a, b, c), (ra, rb, rc))
    };
    let sweep = |from: f64, to: f64| math::to_radians(wrap360(to - from));
    let observed = [sweep(rq, rr), sweep(rr, rp), sweep(rp, rq)];
    if observed.contains(&0.0) {
        return Err(kinavis_kernel::KernelError::Indeterminate {
            quantity: "resection with two equal readings",
        }
        .into());
    }
    let interior = [
        corner_angle(p, q, r),
        corner_angle(q, r, p),
        corner_angle(r, p, q),
    ];
    let mut differences = [0.0; 3];
    for ((difference, &corner), &angle) in differences.iter_mut().zip(&interior).zip(&observed) {
        *difference = cot(corner) - cot(angle);
    }
    // On the circle every difference vanishes and the weights below are
    // undefined; the test needs the figure alone, not the computed point.
    let strength = differences.iter().fold(0.0_f64, |greatest, difference| {
        greatest.max(math::abs(*difference))
    });
    if strength.is_nan() || strength < DANGER_LIMIT {
        return Err(LandError::DangerCircle { strength });
    }
    let [kp, kq, kr] = differences.map(|difference| 1.0 / difference);
    let total = kp + kq + kr;
    let east = (kp * p.0 + kq * q.0 + kr * r.0) / total;
    let north = (kp * p.1 + kq * q.1 + kr * r.1) / total;
    if !east.is_finite() || !north.is_finite() {
        return Err(LandError::DangerCircle { strength });
    }
    let danger = danger_offset(a, b, c, (east, north));
    let point = GridPoint::from_metres(east, north)?;
    // The zero of the circle: the mean of grid bearing minus reading.
    let (mut sin_sum, mut cos_sum) = (0.0, 0.0);
    for (landmark, reading) in landmarks.iter().zip(readings) {
        let bearing = grid_bearing(point, *landmark)?;
        let zero = math::to_radians(bearing.degrees() - reading.degrees());
        sin_sum += math::sin(zero);
        cos_sum += math::cos(zero);
    }
    Ok(ThreePointFix {
        point,
        orientation: Angle::from_degrees(wrap180(math::to_degrees(math::atan2(sin_sum, cos_sum))))?,
        danger,
    })
}

fn too_few(found: usize) -> LandError {
    kinavis_kernel::KernelError::InsufficientData {
        found,
        required: 2,
        context: "a resection",
    }
    .into()
}

/// `to − from`, metres east and north.
fn difference(from: GridPoint, to: GridPoint) -> (f64, f64) {
    let (e0, n0) = from.metres();
    let (e1, n1) = to.metres();
    (e1 - e0, n1 - n0)
}

/// Unit vector east and north of a grid bearing.
fn unit(bearing: GridBearing) -> (f64, f64) {
    let radians = bearing.radians();
    (math::sin(radians), math::cos(radians))
}

/// The line of a sighting as `n · x = offset`, relative to `origin`.
fn normal_form(origin: GridPoint, sighting: &Sighting) -> ((f64, f64), f64) {
    let (east, north) = difference(origin, sighting.landmark);
    let (u_east, u_north) = unit(sighting.bearing);
    let normal = (u_north, -u_east);
    (normal, normal.0 * east + normal.1 * north)
}

fn squared(east: f64, north: f64) -> f64 {
    east * east + north * north
}

/// Interior angle at `at` of the triangle with `left` and `right`.
fn corner_angle(at: (f64, f64), left: (f64, f64), right: (f64, f64)) -> f64 {
    let u = (left.0 - at.0, left.1 - at.1);
    let v = (right.0 - at.0, right.1 - at.1);
    math::abs(math::atan2(u.0 * v.1 - u.1 * v.0, u.0 * v.0 + u.1 * v.1))
}

fn cot(angle: f64) -> f64 {
    math::cos(angle) / math::sin(angle)
}

/// `| |P − O| − R | / R` for the circle through `a`, `b`, `c`.
fn danger_offset(a: (f64, f64), b: (f64, f64), c: (f64, f64), p: (f64, f64)) -> f64 {
    let (bx, by) = (b.0 - a.0, b.1 - a.1);
    let (cx, cy) = (c.0 - a.0, c.1 - a.1);
    let d = 2.0 * (bx * cy - by * cx);
    let ux = (cy * squared(bx, by) - by * squared(cx, cy)) / d;
    let uy = (bx * squared(cx, cy) - cx * squared(bx, by)) / d;
    let radius = math::hypot(ux, uy);
    let distance = math::hypot(p.0 - a.0 - ux, p.1 - a.1 - uy);
    math::abs(distance - radius) / radius
}
