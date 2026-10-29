//! Geofences: an area to keep in or keep out of, and the events of crossing
//! its boundary or coming near it.
//!
//! The area is a [`Region`] on the plane of a [`MapGrid`]. The signed
//! distance of a fix from the boundary — positive inside — moves the fence's
//! state only past a hysteresis band: a fix scattering about the boundary
//! does not report a crossing at every update. A warning is raised once when
//! the fix comes within the approach distance on the side it should stay
//! on; it is armed when the fix is deeper than that distance and the band,
//! so not straight after a crossing, and again after each warning.

use kinavis::region::{Containment, PlanarPoint, Region};
use kinavis_kernel::error::ensure_range;
use kinavis_kernel::event::Event;
use kinavis_kernel::time::{Instant, Utc};
use kinavis_kernel::{Distance, Position};

use crate::error::Result;
use crate::grid::MapGrid;

/// Identifier of a geofence, the application's.
pub type FenceId = u16;

/// Which side of the boundary the vehicle should be on.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum FenceKind {
    /// Keep inside: a field, a work site, a permitted area.
    KeepIn,
    /// Keep outside: a hazard, a protected zone.
    KeepOut,
}

/// Something that happened at a geofence.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GeofenceEvent {
    /// The fix is inside, from outside or from the start.
    Entered {
        /// Fence.
        fence: FenceId,
        /// Its kind.
        kind: FenceKind,
        /// Time of the fix.
        at: Instant<Utc>,
    },
    /// The fix is outside, from inside or from the start.
    Exited {
        /// Fence.
        fence: FenceId,
        /// Its kind.
        kind: FenceKind,
        /// Time of the fix.
        at: Instant<Utc>,
    },
    /// The fix came within the approach distance of the boundary from the
    /// side it should stay on.
    ApproachingBoundary {
        /// Fence.
        fence: FenceId,
        /// Its kind.
        kind: FenceKind,
        /// Distance to the boundary.
        distance: Distance,
        /// Time of the fix.
        at: Instant<Utc>,
    },
}

impl Event for GeofenceEvent {
    const PLACEHOLDER: Self = Self::Entered {
        fence: 0,
        kind: FenceKind::KeepIn,
        at: Instant::UNIX_EPOCH,
    };

    fn at(&self) -> Instant<Utc> {
        match self {
            Self::Entered { at, .. }
            | Self::Exited { at, .. }
            | Self::ApproachingBoundary { at, .. } => *at,
        }
    }
}

impl GeofenceEvent {
    /// Whether the event breaks the fence's rule: leaving a keep-in fence,
    /// entering a keep-out one.
    #[must_use]
    pub const fn is_breach(&self) -> bool {
        matches!(
            self,
            Self::Exited {
                kind: FenceKind::KeepIn,
                ..
            } | Self::Entered {
                kind: FenceKind::KeepOut,
                ..
            }
        )
    }
}

/// Settings of a geofence.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FenceConfig {
    hysteresis: f64,
    approach: f64,
}

impl FenceConfig {
    /// A crossing counts once the fix is `hysteresis` beyond the boundary; a
    /// warning when it comes within `approach` of it on the safe side, zero
    /// for none.
    ///
    /// # Errors
    ///
    /// [`kinavis_kernel::KernelError::OutOfRange`] for a hysteresis outside 0
    /// to 1 km or an approach outside 0 to 100 km.
    pub fn new(hysteresis: Distance, approach: Distance) -> Result<Self> {
        ensure_range("geofence hysteresis, m", hysteresis.metres(), 0.0, 1_000.0)?;
        ensure_range("geofence approach, m", approach.metres(), 0.0, 100_000.0)?;
        Ok(Self {
            hysteresis: hysteresis.metres(),
            approach: approach.metres(),
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Side {
    Unknown,
    Inside,
    Outside,
}

/// A geofence over a region of `N` vertices with up to `K` holes of `H`.
#[derive(Debug, Clone, PartialEq)]
pub struct Geofence<const N: usize, const H: usize, const K: usize> {
    id: FenceId,
    kind: FenceKind,
    region: Region<N, H, K>,
    grid: MapGrid,
    config: FenceConfig,
    side: Side,
    warned: bool,
}

impl<const N: usize, const H: usize, const K: usize> Geofence<N, H, K> {
    /// A fence over `region`, given on the plane of `grid`.
    #[must_use]
    pub const fn new(
        id: FenceId,
        kind: FenceKind,
        region: Region<N, H, K>,
        grid: MapGrid,
        config: FenceConfig,
    ) -> Self {
        Self {
            id,
            kind,
            region,
            grid,
            config,
            side: Side::Unknown,
            warned: false,
        }
    }

    /// Identifier.
    #[must_use]
    pub const fn id(&self) -> FenceId {
        self.id
    }

    /// Whether the last fix left the fence inside; `None` before the first.
    #[must_use]
    pub const fn inside(&self) -> Option<bool> {
        match self.side {
            Side::Unknown => None,
            Side::Inside => Some(true),
            Side::Outside => Some(false),
        }
    }

    /// Signed distance of a position from the boundary: positive inside.
    ///
    /// # Errors
    ///
    /// As [`MapGrid::project`].
    pub fn signed_distance(&self, position: Position) -> Result<Distance> {
        let on = self.grid.project(position)?.point;
        let point = PlanarPoint::new(on.easting, on.northing);
        let distance = self.region.distance_to_boundary(point).metres();
        let signed = match self.region.contains(point) {
            Containment::Inside => distance,
            Containment::OnBoundary => 0.0,
            Containment::Outside => -distance,
        };
        Ok(Distance::from_metres(signed)?)
    }

    /// Takes a fix; the event it causes, if any. The first fix reports the
    /// side it is on.
    ///
    /// # Errors
    ///
    /// As [`MapGrid::project`].
    pub fn update(
        &mut self,
        position: Position,
        at: Instant<Utc>,
    ) -> Result<Option<GeofenceEvent>> {
        let signed = self.signed_distance(position)?.metres();
        let band = self.config.hysteresis;
        let (id, kind) = (self.id, self.kind);
        let crossed = match self.side {
            Side::Unknown => Some(if signed >= 0.0 {
                Side::Inside
            } else {
                Side::Outside
            }),
            Side::Inside if signed < -band => Some(Side::Outside),
            Side::Outside if signed > band => Some(Side::Inside),
            _ => None,
        };
        if let Some(side) = crossed {
            self.side = side;
            // Just across onto the safe side the fix is near the boundary by
            // necessity: the warning arms once it is deeper.
            let safe = match (kind, side) {
                (FenceKind::KeepIn, Side::Inside) => signed,
                (FenceKind::KeepOut, Side::Outside) => -signed,
                _ => f64::INFINITY,
            };
            self.warned = safe <= self.config.approach + band;
            return Ok(Some(match side {
                Side::Inside => GeofenceEvent::Entered {
                    fence: id,
                    kind,
                    at,
                },
                _ => GeofenceEvent::Exited {
                    fence: id,
                    kind,
                    at,
                },
            }));
        }
        // The distance on the side the vehicle should keep to.
        let safe = match (kind, self.side) {
            (FenceKind::KeepIn, Side::Inside) => signed,
            (FenceKind::KeepOut, Side::Outside) => -signed,
            _ => return Ok(None),
        };
        let approach = self.config.approach;
        if approach > 0.0 && !self.warned && safe < approach {
            self.warned = true;
            return Ok(Some(GeofenceEvent::ApproachingBoundary {
                fence: id,
                kind,
                distance: Distance::from_metres(safe.max(0.0))?,
                at,
            }));
        }
        if self.warned && safe > approach + band {
            self.warned = false;
        }
        Ok(None)
    }
}
