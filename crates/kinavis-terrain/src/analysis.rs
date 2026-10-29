//! Slope and aspect, profiles and line of sight over any elevation model.
//!
//! Heights of the model are taken as they come, above mean sea level for
//! SRTM. Where the geometry needs heights above the ellipsoid — a sight line
//! through the Earth-fixed frame — the same values serve, the geoid's slope
//! over a line of sight being centimetres per kilometre.

use kinavis_kernel::error::{ensure_range, KernelError, Result};
use kinavis_kernel::geodesy::{EcefPoint, ElevationModel, Ellipsoid, GeodeticPoint};
use kinavis_kernel::{math, Angle, Distance, Height, Latitude, Longitude, Position, TrueBearing};

/// Mean Earth radius for the refraction of a sight line, m.
const EARTH_RADIUS: f64 = 6_371_008.8;

/// Most steps of a line of sight.
pub const MAX_SIGHT_STEPS: usize = 65_536;

/// Standard coefficient of terrestrial refraction: a sight line bends with
/// 0.13 of the Earth's curvature.
pub const STANDARD_REFRACTION: f64 = 0.13;

/// Slope and aspect of the ground at a point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SlopeAspect {
    /// Angle of the steepest slope from the horizontal.
    pub slope: Angle,
    /// Direction the ground falls in; `None` on level ground.
    pub aspect: Option<TrueBearing>,
    /// Rise eastward, metres per metre.
    pub east_gradient: f64,
    /// Rise northward, metres per metre.
    pub north_gradient: f64,
}

/// A point of a profile.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ProfilePoint {
    /// Ground distance from the start.
    pub distance: Distance,
    /// Position.
    pub position: Position,
    /// Height of the ground.
    pub elevation: Height,
}

/// What a profile climbs and falls.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ProfileSummary {
    /// Ground length.
    pub length: Distance,
    /// Total ascent.
    pub ascent: Distance,
    /// Total descent.
    pub descent: Distance,
    /// Highest ground.
    pub highest: Height,
    /// Lowest ground.
    pub lowest: Height,
}

/// A line of sight.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sight {
    /// Whether the line clears the ground at every step.
    pub visible: bool,
    /// Least height of the line above the ground; negative where blocked.
    pub clearance: Distance,
    /// Distance from the observer where the clearance is least.
    pub at: Distance,
    /// Position there.
    pub position: Position,
}

/// Slope and aspect at a position by Horn's method on the 3 × 3 heights
/// `spacing` apart, the weighting of `gdaldem slope`.
///
/// # Errors
///
/// [`KernelError::OutOfRange`] for a spacing outside 0.1 m to 10 km; the
/// model's errors at any of the nine points.
// The window's heights a…i as Horn and gdaldem name them.
#[allow(clippy::many_single_char_names)]
pub fn slope_aspect(
    model: &impl ElevationModel,
    position: Position,
    spacing: Distance,
) -> Result<SlopeAspect> {
    let d = spacing.metres();
    ensure_range("slope spacing, m", d, 0.1, 10_000.0)?;
    let mut z = [[0.0; 3]; 3];
    for (row, north) in z.iter_mut().zip([1.0, 0.0, -1.0]) {
        for (value, east) in row.iter_mut().zip([-1.0, 0.0, 1.0]) {
            let at = offset(position, east * d, north * d)?;
            *value = model.elevation(at)?.value().metres();
        }
    }
    let [[a, b, c], [l, _, r], [g, h, i]] = z;
    let east_gradient = ((c + 2.0 * r + i) - (a + 2.0 * l + g)) / (8.0 * d);
    let north_gradient = ((a + 2.0 * b + c) - (g + 2.0 * h + i)) / (8.0 * d);
    let rise = math::hypot(east_gradient, north_gradient);
    let aspect = (rise > 0.0).then(|| {
        TrueBearing::from_degrees_wrapped(math::to_degrees(math::atan2(
            -east_gradient,
            -north_gradient,
        )))
    });
    Ok(SlopeAspect {
        slope: Angle::from_radians(math::atan(rise))?,
        aspect,
        east_gradient,
        north_gradient,
    })
}

/// The profile from `from` to `to`, `points.len()` points evenly spaced
/// along the chord through the Earth, which on the ground is the normal
/// section, within millimetres of the geodesic over tens of kilometres.
///
/// # Errors
///
/// [`KernelError::InsufficientData`] for fewer than two points; the
/// model's errors along the way.
pub fn profile(
    model: &impl ElevationModel,
    from: Position,
    to: Position,
    points: &mut [ProfilePoint],
) -> Result<ProfileSummary> {
    let count = points.len();
    if count < 2 {
        return Err(KernelError::InsufficientData {
            found: count,
            required: 2,
            context: "a terrain profile",
        });
    }
    let (start, end) = (surface(from)?, surface(to)?);
    let last = math::count_to_f64(count.saturating_sub(1));
    let mut travelled = 0.0;
    let mut previous = start;
    let final_index = count.saturating_sub(1);
    for (index, point) in points.iter_mut().enumerate() {
        let t = math::count_to_f64(index) / last;
        // The ends exactly as given; between, the chord brought down.
        let (position, ground) = match index {
            0 => (from, start),
            i if i == final_index => (to, end),
            _ => down_to_ground(lerp(start, end, t))?,
        };
        travelled += chord(previous, ground);
        previous = ground;
        *point = ProfilePoint {
            distance: Distance::from_metres(travelled)?,
            position,
            elevation: model.elevation(position)?,
        };
    }
    summarise(points, Distance::ZERO)
}

/// Length, ascent, descent and extremes of a profile; rises and falls under
/// `threshold` between turning points are ignored, as noise of the model.
///
/// # Errors
///
/// [`KernelError::InsufficientData`] for an empty profile;
/// [`KernelError::VerticalDatumMismatch`] for heights on different datums.
pub fn summarise(points: &[ProfilePoint], threshold: Distance) -> Result<ProfileSummary> {
    let Some(first) = points.first() else {
        return Err(KernelError::InsufficientData {
            found: 0,
            required: 1,
            context: "a terrain profile",
        });
    };
    let threshold = threshold.metres().max(0.0);
    let (mut ascent, mut descent) = (0.0, 0.0);
    let (mut highest, mut lowest) = (first.elevation, first.elevation);
    // The last turning point counted, and the extreme reached since.
    let mut anchor = first.elevation.value().metres();
    for point in points {
        let height = point.elevation.value().metres();
        if point.elevation.difference(highest)? > Distance::ZERO {
            highest = point.elevation;
        }
        if point.elevation.difference(lowest)? < Distance::ZERO {
            lowest = point.elevation;
        }
        let change = height - anchor;
        if change > threshold {
            ascent += change;
            anchor = height;
        } else if -change > threshold {
            descent -= change;
            anchor = height;
        }
    }
    Ok(ProfileSummary {
        length: points.last().map_or(Distance::ZERO, |point| point.distance),
        ascent: Distance::from_metres(ascent)?,
        descent: Distance::from_metres(descent)?,
        highest,
        lowest,
    })
}

/// Line of sight from an eye `eye` above the ground at `observer` to a point
/// `target_height` above the ground at `target`, checked at `steps` points
/// between them; `refraction` is the coefficient that bends the line with
/// the Earth ([`STANDARD_REFRACTION`], or 0 for a straight line).
///
/// # Errors
///
/// [`KernelError::OutOfRange`] for steps outside 1 to [`MAX_SIGHT_STEPS`],
/// a refraction outside −1 to 1, or negative heights; the model's errors.
pub fn line_of_sight(
    model: &impl ElevationModel,
    observer: Position,
    eye: Distance,
    target: Position,
    target_height: Distance,
    steps: usize,
    refraction: f64,
) -> Result<Sight> {
    ensure_range(
        "sight steps",
        math::count_to_f64(steps),
        1.0,
        math::count_to_f64(MAX_SIGHT_STEPS),
    )?;
    ensure_range("refraction coefficient", refraction, -1.0, 1.0)?;
    ensure_range("eye height, m", eye.metres(), 0.0, 1e5)?;
    ensure_range("target height, m", target_height.metres(), 0.0, 1e5)?;
    let from = raised(observer, model.elevation(observer)?.value() + eye)?;
    let to = raised(target, model.elevation(target)?.value() + target_height)?;
    let length = chord(from, to);
    let mut least = Sight {
        visible: true,
        clearance: Distance::from_metres(f64::MAX)?,
        at: Distance::ZERO,
        position: observer,
    };
    let parts = math::count_to_f64(steps.saturating_add(1));
    for step in 1..=steps {
        let t = math::count_to_f64(step) / parts;
        let along = lerp(from, to, t).to_geodetic(&Ellipsoid::WGS84)?;
        let position = along.position();
        let lift = refraction * (t * length) * ((1.0 - t) * length) / (2.0 * EARTH_RADIUS);
        let ray = along.height().value().metres() + lift;
        let ground = model.elevation(position)?.value().metres();
        let clearance = ray - ground;
        if clearance < least.clearance.metres() {
            least = Sight {
                visible: clearance >= 0.0,
                clearance: Distance::from_metres(clearance)?,
                at: Distance::from_metres(t * length)?,
                position,
            };
        }
    }
    least.visible = least.clearance.metres() >= 0.0;
    Ok(least)
}

/// A position `east` and `north` metres from another, on WGS 84's radii of
/// curvature there.
fn offset(position: Position, east: f64, north: f64) -> Result<Position> {
    let ellipsoid = Ellipsoid::WGS84;
    let a = ellipsoid.semi_major_axis().metres();
    let e2 = ellipsoid.first_eccentricity_squared();
    let phi = position.latitude().radians();
    let w = math::sqrt(1.0 - e2 * math::sin(phi) * math::sin(phi));
    let meridian = a * (1.0 - e2) / (w * w * w);
    let normal = a / w;
    let cos_phi = math::cos(phi);
    if cos_phi < 1e-9 {
        return Err(KernelError::Indeterminate {
            quantity: "east offset at a pole",
        });
    }
    let latitude = position.latitude().degrees() + math::to_degrees(north / meridian);
    let longitude = position.longitude().degrees() + math::to_degrees(east / (normal * cos_phi));
    Ok(Position::new(
        Latitude::from_degrees(latitude)?,
        Longitude::from_degrees_wrapped(longitude),
    ))
}

fn surface(position: Position) -> Result<EcefPoint> {
    raised(position, Distance::ZERO)
}

fn raised(position: Position, height: Distance) -> Result<EcefPoint> {
    EcefPoint::from_geodetic(
        GeodeticPoint::new(position, Height::above_ellipsoid(height)),
        &Ellipsoid::WGS84,
    )
}

/// The ground position below a point of the chord, and the surface point.
fn down_to_ground(point: EcefPoint) -> Result<(Position, EcefPoint)> {
    let position = point.to_geodetic(&Ellipsoid::WGS84)?.position();
    Ok((position, surface(position)?))
}

fn lerp(from: EcefPoint, to: EcefPoint, t: f64) -> EcefPoint {
    let mix = |a: Distance, b: Distance| {
        Distance::from_metres(a.metres() + (b.metres() - a.metres()) * t).unwrap_or(Distance::ZERO)
    };
    EcefPoint::new(
        mix(from.x(), to.x()),
        mix(from.y(), to.y()),
        mix(from.z(), to.z()),
    )
}

fn chord(from: EcefPoint, to: EcefPoint) -> f64 {
    let dx = to.x().metres() - from.x().metres();
    let dy = to.y().metres() - from.y().metres();
    let dz = to.z().metres() - from.z().metres();
    math::sqrt(dx * dx + dy * dy + dz * dz)
}
