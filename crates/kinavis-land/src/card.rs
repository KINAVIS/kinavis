//! A route by bearings (маршрут по азимутам): legs of a grid bearing and a
//! ground distance from a start, laid out on the map grid, and the card of
//! such legs read back from a route.
//!
//! Each leg is a straight line on the grid. Its grid length is the ground
//! distance times the point scale at the leg's middle; over legs of a few
//! kilometres the scale changes by under 10⁻⁶ in a UTM zone. The waypoints
//! make a [`Route`] of rhumb-line legs, which on such legs lie within
//! centimetres of the grid lines.

use kinavis::route::{LegKind, Route, MAX_WAYPOINTS};
use kinavis_kernel::{math, Distance, GridBearing, KernelError, MagneticBearing, Position};

use crate::bearings::BearingFrame;
use crate::error::Result;
use crate::grid::{GridPoint, MapGrid};
use crate::resection::{grid_bearing, grid_distance};

/// Iterations for the scale at a leg's middle.
const SCALE_ITERATIONS: usize = 3;

/// One leg of a route by bearings.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Leg {
    /// Grid bearing to follow.
    pub bearing: GridBearing,
    /// Ground distance.
    pub distance: Distance,
}

impl Leg {
    /// Leg on a magnetic bearing, as a compass is set, in the frame of the
    /// area.
    #[must_use]
    pub fn magnetic(frame: &BearingFrame, bearing: MagneticBearing, distance: Distance) -> Self {
        Self {
            bearing: frame.magnetic_to_grid(bearing),
            distance,
        }
    }

    /// The magnetic bearing of the leg in the frame of the area.
    #[must_use]
    pub fn magnetic_bearing(&self, frame: &BearingFrame) -> MagneticBearing {
        frame.grid_to_magnetic(self.bearing)
    }
}

/// The route walked from `start` along `legs` on `grid`.
///
/// # Errors
///
/// [`KernelError::CapacityExceeded`] for more legs than a [`Route`] holds;
/// as [`MapGrid::project`] and [`MapGrid::unproject`] for a leg that leaves
/// the grid.
pub fn route_from_legs(grid: &MapGrid, start: Position, legs: &[Leg]) -> Result<Route> {
    let capacity = MAX_WAYPOINTS.saturating_sub(1);
    if legs.len() > capacity {
        return Err(KernelError::CapacityExceeded {
            context: "legs of a route",
            needed: legs.len(),
            capacity,
        }
        .into());
    }
    let mut waypoints = [start; MAX_WAYPOINTS];
    let mut here = grid.project(start)?;
    for (index, leg) in legs.iter().enumerate() {
        let (sin, cos) = (
            math::sin(leg.bearing.radians()),
            math::cos(leg.bearing.radians()),
        );
        let (east, north) = here.point.metres();
        let mut scale = here.scale;
        let mut end = (start, here);
        for _ in 0..SCALE_ITERATIONS {
            let length = leg.distance.metres() * scale;
            let point = GridPoint::from_metres(east + length * sin, north + length * cos)?;
            end = grid.unproject(point)?;
            let middle =
                GridPoint::from_metres(east + 0.5 * length * sin, north + 0.5 * length * cos)?;
            scale = grid.unproject(middle)?.1.scale;
        }
        if let Some(slot) = waypoints.get_mut(index.saturating_add(1)) {
            *slot = end.0;
        }
        here = end.1;
    }
    let count = legs.len().saturating_add(1);
    Ok(Route::new(
        waypoints.get(..count).unwrap_or(&[]),
        LegKind::RhumbLine,
    )?)
}

/// The card of a route on `grid`: each leg's grid bearing and ground
/// distance, into `legs`. Returns the number of legs.
///
/// # Errors
///
/// [`KernelError::BufferTooSmall`] if `legs` is shorter than the route's
/// legs; as [`MapGrid::project`]; [`KernelError::Indeterminate`] for two
/// equal waypoints in a row.
pub fn legs_of(grid: &MapGrid, route: &Route, legs: &mut [Leg]) -> Result<usize> {
    let needed = route.leg_count();
    if legs.len() < needed {
        return Err(KernelError::BufferTooSmall {
            needed,
            found: legs.len(),
        }
        .into());
    }
    for (slot, pair) in legs.iter_mut().zip(route.waypoints().windows(2)) {
        let [from, to] = pair else {
            continue;
        };
        let (a, b) = (grid.project(*from)?, grid.project(*to)?);
        let (ea, na) = a.point.metres();
        let (eb, nb) = b.point.metres();
        let middle = GridPoint::from_metres(f64::midpoint(ea, eb), f64::midpoint(na, nb))?;
        let scale = grid.unproject(middle)?.1.scale;
        *slot = Leg {
            bearing: grid_bearing(a.point, b.point)?,
            distance: Distance::from_metres(grid_distance(a.point, b.point).metres() / scale)?,
        };
    }
    Ok(needed)
}
