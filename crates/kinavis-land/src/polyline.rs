//! A route from the polyline of an external router: the points that keep
//! within a tolerance of it, at most what a [`Route`] holds.

use kinavis::route::{LegKind, Route, MAX_WAYPOINTS};
use kinavis_kernel::error::{ensure_range, KernelError};
use kinavis_kernel::{math, Distance, Position};

use crate::error::Result;

/// A route made from a polyline, with how far the polyline strays from it.
#[derive(Debug, Clone, PartialEq)]
pub struct Simplified {
    /// The route.
    pub route: Route,
    /// Greatest distance of a point of the polyline from the route.
    pub deviation: Distance,
}

/// The route through the points of `polyline` that keeps every other point
/// within `tolerance`, or as close as [`MAX_WAYPOINTS`] allow: the ends are
/// kept, then the point farthest from the route so far, until the tolerance
/// is met or the route is full. Distances are on the local plane of each
/// leg, exact to the curvature over a leg of a few kilometres.
///
/// # Errors
///
/// [`KernelError::InsufficientData`] for fewer than two points;
/// [`KernelError::OutOfRange`] for a negative tolerance; as [`Route::new`].
pub fn route_from_polyline(
    polyline: &[Position],
    tolerance: Distance,
    kind: LegKind,
) -> Result<Simplified> {
    if polyline.len() < 2 {
        return Err(KernelError::InsufficientData {
            found: polyline.len(),
            required: 2,
            context: "a polyline",
        }
        .into());
    }
    ensure_range("tolerance, m", tolerance.metres(), 0.0, 1e7)?;
    let last = polyline.len().saturating_sub(1);
    // Indices kept, in order.
    let mut kept = [0usize; MAX_WAYPOINTS];
    let mut count: usize = 2;
    if let Some(slot) = kept.get_mut(1) {
        *slot = last;
    }
    let mut deviation;
    loop {
        // The point farthest from its leg, and where it would go.
        let mut worst = (0.0_f64, 0usize, 0usize);
        for leg in 0..count.saturating_sub(1) {
            let (Some(&from), Some(&to)) = (kept.get(leg), kept.get(leg + 1)) else {
                continue;
            };
            for index in from.saturating_add(1)..to {
                let (Some(a), Some(b), Some(p)) =
                    (polyline.get(from), polyline.get(to), polyline.get(index))
                else {
                    continue;
                };
                let off = off_leg(*a, *b, *p);
                if off > worst.0 {
                    worst = (off, index, leg + 1);
                }
            }
        }
        deviation = worst.0;
        if deviation <= tolerance.metres() || count == MAX_WAYPOINTS {
            break;
        }
        let (_, index, at) = worst;
        // Make room at `at`, keeping the order.
        for slot in (at.saturating_add(1)..=count).rev() {
            let moved = kept.get(slot.saturating_sub(1)).copied();
            if let (Some(moved), Some(target)) = (moved, kept.get_mut(slot)) {
                *target = moved;
            }
        }
        if let Some(target) = kept.get_mut(at) {
            *target = index;
        }
        count = count.saturating_add(1);
    }
    let Some(&first) = polyline.first() else {
        return Err(KernelError::InsufficientData {
            found: 0,
            required: 2,
            context: "a polyline",
        }
        .into());
    };
    let mut waypoints = [first; MAX_WAYPOINTS];
    for (slot, &index) in waypoints.iter_mut().zip(kept.iter().take(count)) {
        if let Some(point) = polyline.get(index) {
            *slot = *point;
        }
    }
    Ok(Simplified {
        route: Route::new(waypoints.get(..count).unwrap_or(&[]), kind)?,
        deviation: Distance::from_metres(deviation)?,
    })
}

/// Distance of `p` from the segment `a`–`b`, on the plane tangent at `a`.
fn off_leg(a: Position, b: Position, p: Position) -> f64 {
    let (north, east) = metres_per_degree(a.latitude().degrees());
    let flat = |q: Position| {
        (
            kinavis_kernel::wrap180(q.longitude().degrees() - a.longitude().degrees()) * east,
            (q.latitude().degrees() - a.latitude().degrees()) * north,
        )
    };
    let (bx, by) = flat(b);
    let (px, py) = flat(p);
    let span = bx * bx + by * by;
    let t = if span > 0.0 {
        ((px * bx + py * by) / span).clamp(0.0, 1.0)
    } else {
        0.0
    };
    math::hypot(px - t * bx, py - t * by)
}

/// Metres per degree north and east at a latitude, on WGS 84.
fn metres_per_degree(latitude: f64) -> (f64, f64) {
    let a = 6_378_137.0;
    let e2 = 0.006_694_379_990_14;
    let phi = math::to_radians(latitude);
    let w = math::sqrt(1.0 - e2 * math::sin(phi) * math::sin(phi));
    (
        math::to_radians(a * (1.0 - e2) / (w * w * w)),
        math::to_radians(a / w * math::cos(phi)),
    )
}
