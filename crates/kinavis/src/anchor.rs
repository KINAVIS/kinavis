//! Anchor watch: swinging circle and dragging detection.
//!
//! Swinging radius = horizontal scope of the cable + hawse-to-antenna distance.
//! A position outside the circle plus an allowance (position error, catenary
//! slack) raises [`AnchorEvent::AnchorDragging`]. An error in the recorded
//! anchor position is indistinguishable from dragging.
//!
//! - [`swinging_radius`] — circle radius.
//! - [`anchor_position`] — anchor position from the antenna position at let-go.
//! - [`AnchorWatch::check`] — snapshot → [`AnchorView`] + event; stateless.
//!
//! ```rust
//! use kinavis::anchor::{anchor_position, swinging_radius, AnchorWatch};
//! use kinavis::{
//!     Distance, Instant, AnchorEvent, NavigationSnapshot, ObservationStatus, Observed,
//!     Position, PositionSource, Quality, TrueCourse, Utc,
//! };
//!
//! // Let go heading north, the antenna 40 m abaft the bow, in 10 m of
//! // water with the hawse 3 m above it, four shackles (110 m) of cable.
//! let antenna: Position = "50°45.00'N 001°20.00'W".parse()?;
//! let anchor = anchor_position(antenna, TrueCourse::NORTH, Distance::from_metres(40.0)?)?;
//! let radius = swinging_radius(
//!     Distance::from_metres(110.0)?,
//!     Distance::from_metres(13.0)?,
//!     Distance::from_metres(40.0)?,
//! )?;
//! // Roughly 109 m of horizontal scope, plus the 40 m of ship.
//! assert_eq!(format!("{:.0}", radius.metres()), "149");
//!
//! let watch = AnchorWatch::new(anchor, radius)?.with_allowance(Distance::from_metres(15.0)?)?;
//!
//! // Later, swung to the south: 0.1' of latitude is 185 m from the anchor.
//! let now = Instant::<Utc>::from_unix_seconds(1_789_003_600);
//! let here: Position = "50°44.92'N 001°20.00'W".parse()?;
//! let state = NavigationSnapshot::EMPTY.with_position(
//!     Observed::new(here, now, Quality::<Distance>::new(ObservationStatus::Valid)),
//!     PositionSource::Gnss,
//! );
//! let (view, events) = watch.check(&state)?;
//! assert!(view.is_dragging());
//! assert!(matches!(events[0], AnchorEvent::AnchorDragging { .. }));
//! # Ok::<(), kinavis::NavigationError>(())
//! ```

use crate::angle::TrueCourse;
use crate::error::{ensure_range, KernelError, NavigationError, Result};
use crate::event::{AnchorEvent, EventList};
use crate::math;
use crate::position::Position;
use crate::sailings::{great_circle, rhumb_destination};
use crate::snapshot::NavigationSnapshot;
use crate::time::{Instant, Utc};
use crate::units::Distance;

/// Anchor position: `antenna_to_bow` ahead of the antenna position at let-go,
/// along the heading.
///
/// # Errors
///
/// [`KernelError::OutOfRange`] if `antenna_to_bow` is negative; otherwise as
/// [`rhumb_destination`].
pub fn anchor_position(
    antenna: Position,
    heading: TrueCourse,
    antenna_to_bow: Distance,
) -> Result<Position> {
    ensure_range("antenna to bow", antenna_to_bow.metres(), 0.0, f64::MAX)?;
    rhumb_destination(antenna, heading, antenna_to_bow)
}

/// Swinging circle radius about the anchor, at the measured position.
///
/// `√(cable² − depth_at_hawse²) + antenna_to_bow`; `depth_at_hawse` is the
/// water depth plus the hawse height above the surface.
///
/// # Errors
///
/// [`KernelError::OutOfRange`] if a length is negative or the cable is shorter
/// than `depth_at_hawse`.
pub fn swinging_radius(
    cable: Distance,
    depth_at_hawse: Distance,
    antenna_to_bow: Distance,
) -> Result<Distance> {
    ensure_range("depth at hawse", depth_at_hawse.metres(), 0.0, f64::MAX)?;
    ensure_range("antenna to bow", antenna_to_bow.metres(), 0.0, f64::MAX)?;
    ensure_range("cable", cable.metres(), depth_at_hawse.metres(), f64::MAX)?;
    let scope = math::sqrt(
        cable.metres() * cable.metres() - depth_at_hawse.metres() * depth_at_hawse.metres(),
    );
    Ok(Distance::from_metres(scope)? + antenna_to_bow)
}

/// Anchor position and the circle the vessel may swing on.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(
    feature = "serde",
    serde(try_from = "StoredWatch", into = "StoredWatch")
)]
pub struct AnchorWatch {
    anchor: Position,
    swinging_radius: Distance,
    allowance: Distance,
}

impl AnchorWatch {
    /// Watch on `anchor` with the given swinging radius and zero allowance.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] if the radius is negative.
    pub fn new(anchor: Position, swinging_radius: Distance) -> Result<Self> {
        ensure_range("swinging radius", swinging_radius.metres(), 0.0, f64::MAX)?;
        Ok(Self {
            anchor,
            swinging_radius,
            allowance: Distance::ZERO,
        })
    }

    /// Sets the allowance beyond the swinging circle before dragging is
    /// reported: position error, cable slack, false-alarm margin.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] if the allowance is negative.
    pub fn with_allowance(mut self, allowance: Distance) -> Result<Self> {
        ensure_range("allowance", allowance.metres(), 0.0, f64::MAX)?;
        self.allowance = allowance;
        Ok(self)
    }

    /// Anchor position.
    #[must_use]
    pub const fn anchor(&self) -> Position {
        self.anchor
    }

    /// Swinging circle radius.
    #[must_use]
    pub const fn swinging_radius(&self) -> Distance {
        self.swinging_radius
    }

    /// Allowance beyond the swinging circle.
    #[must_use]
    pub const fn allowance(&self) -> Distance {
        self.allowance
    }

    /// Dragging threshold: swinging radius + allowance.
    #[must_use]
    pub fn alarm_radius(&self) -> Distance {
        self.swinging_radius + self.allowance
    }

    /// Checks the position in `state` against the watch.
    ///
    /// # Errors
    ///
    /// [`KernelError::Indeterminate`] if the snapshot has no position.
    pub fn check(
        &self,
        state: &NavigationSnapshot,
    ) -> Result<(AnchorView, EventList<AnchorEvent>)> {
        let observed = state
            .position()
            .ok_or(NavigationError::Kernel(KernelError::Missing {
                what: "the vessel's position",
            }))?;
        let from_anchor = great_circle(self.anchor, *observed.value())?;
        let view = AnchorView {
            at: observed.taken_at(),
            distance_from_anchor: from_anchor.distance,
            bearing_from_anchor: from_anchor.initial_course,
            alarm_radius: self.alarm_radius(),
        };

        let mut events = EventList::new();
        if view.is_dragging() {
            events.push(AnchorEvent::AnchorDragging {
                distance: view.distance_from_anchor,
                radius: view.alarm_radius,
                at: view.at,
            });
        }
        Ok((view, events))
    }
}

/// Serialised form; deserialisation goes through the constructors.
#[cfg(feature = "serde")]
#[derive(serde::Serialize, serde::Deserialize)]
struct StoredWatch {
    anchor: Position,
    swinging_radius: Distance,
    allowance: Distance,
}

#[cfg(feature = "serde")]
impl TryFrom<StoredWatch> for AnchorWatch {
    type Error = NavigationError;

    fn try_from(stored: StoredWatch) -> Result<Self> {
        Self::new(stored.anchor, stored.swinging_radius)?.with_allowance(stored.allowance)
    }
}

#[cfg(feature = "serde")]
impl From<AnchorWatch> for StoredWatch {
    fn from(watch: AnchorWatch) -> Self {
        Self {
            anchor: watch.anchor,
            swinging_radius: watch.swinging_radius,
            allowance: watch.allowance,
        }
    }
}

/// Vessel position relative to the anchor at one instant.
///
/// Projection returned by [`AnchorWatch::check`].
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct AnchorView {
    at: Instant<Utc>,
    distance_from_anchor: Distance,
    bearing_from_anchor: TrueCourse,
    alarm_radius: Distance,
}

impl AnchorView {
    /// Time of the position.
    #[must_use]
    pub const fn at(&self) -> Instant<Utc> {
        self.at
    }

    /// Distance from the anchor.
    #[must_use]
    pub const fn distance_from_anchor(&self) -> Distance {
        self.distance_from_anchor
    }

    /// Bearing of the position from the anchor.
    #[must_use]
    pub const fn bearing_from_anchor(&self) -> TrueCourse {
        self.bearing_from_anchor
    }

    /// Dragging threshold: swinging radius + allowance.
    #[must_use]
    pub const fn alarm_radius(&self) -> Distance {
        self.alarm_radius
    }

    /// Margin to the threshold; negative when beyond it.
    #[must_use]
    pub fn slack(&self) -> Distance {
        self.alarm_radius - self.distance_from_anchor
    }

    /// Whether the position is beyond the threshold.
    #[must_use]
    pub fn is_dragging(&self) -> bool {
        self.slack().is_negative()
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp, clippy::indexing_slicing)]
mod tests {
    use super::*;
    use crate::event::PositionSource;
    use crate::observation::{ObservationStatus, Observed, Quality};

    fn metres(value: f64) -> Distance {
        Distance::from_metres(value).unwrap()
    }

    fn at(latitude: f64, longitude: f64) -> Position {
        Position::from_degrees(latitude, longitude).unwrap()
    }

    fn noon() -> Instant<Utc> {
        Instant::from_unix_seconds(1_789_000_000)
    }

    fn snapshot(position: Position) -> NavigationSnapshot {
        NavigationSnapshot::EMPTY.with_position(
            Observed::new(
                position,
                noon(),
                Quality::<Distance>::new(ObservationStatus::Valid),
            ),
            PositionSource::Gnss,
        )
    }

    #[test]
    fn the_circle_is_the_horizontal_scope_plus_the_ship() {
        // A 5-12-13 triangle: 130 m of cable in 50 m reaches 120 m along.
        let radius = swinging_radius(metres(130.0), metres(50.0), metres(30.0)).unwrap();
        assert!((radius.metres() - 150.0).abs() < 1e-6);

        // Cable straight down: no scope, only the ship.
        let plumb = swinging_radius(metres(50.0), metres(50.0), metres(30.0)).unwrap();
        assert!((plumb.metres() - 30.0).abs() < 1e-6);

        // Cable that does not reach the bottom, or nonsense lengths.
        assert!(swinging_radius(metres(40.0), metres(50.0), metres(30.0)).is_err());
        assert!(swinging_radius(metres(130.0), metres(-1.0), metres(30.0)).is_err());
        assert!(swinging_radius(metres(130.0), metres(50.0), metres(-1.0)).is_err());
    }

    #[test]
    fn the_anchor_lies_ahead_of_the_antenna_by_the_length_of_the_bow() {
        let antenna = at(50.0, -1.0);
        let anchor = anchor_position(antenna, TrueCourse::EAST, metres(50.0)).unwrap();
        let back = great_circle(antenna, anchor).unwrap();
        assert!((back.distance.metres() - 50.0).abs() < 1e-3);
        assert!((back.initial_course.degrees() - 90.0).abs() < 1e-3);
        assert!(anchor_position(antenna, TrueCourse::EAST, metres(-1.0)).is_err());
    }

    #[test]
    fn inside_the_circle_is_quiet_and_outside_is_an_alarm() {
        let anchor = at(50.0, -1.0);
        let watch = AnchorWatch::new(anchor, metres(100.0))
            .unwrap()
            .with_allowance(metres(20.0))
            .unwrap();
        assert!((watch.alarm_radius().metres() - 120.0).abs() < 1e-9);

        // 0.05' of latitude south: 93 m, inside.
        let (inside, events) = watch
            .check(&snapshot(at(50.0 - 0.05 / 60.0, -1.0)))
            .unwrap();
        assert!(!inside.is_dragging());
        assert!(inside.slack().metres() > 20.0);
        assert!((inside.bearing_from_anchor().degrees() - 180.0).abs() < 1e-6);
        assert_eq!(inside.at(), noon());
        assert!(events.is_empty());

        // 0.08' north: 148 m, outside.
        let (outside, events) = watch
            .check(&snapshot(at(50.0 + 0.08 / 60.0, -1.0)))
            .unwrap();
        assert!(outside.is_dragging());
        assert!(outside.slack().is_negative());
        assert!(matches!(
            events[0],
            AnchorEvent::AnchorDragging { distance, radius, at }
                if distance == outside.distance_from_anchor()
                    && radius == outside.alarm_radius()
                    && at == noon()
        ));
    }

    #[test]
    fn the_allowance_is_the_difference_between_a_swing_and_a_drag() {
        let anchor = at(50.0, -1.0);
        let here = at(50.0, -1.0 + 0.09 / 60.0 / math::cos(50.0_f64.to_radians()));
        let tight = AnchorWatch::new(anchor, metres(150.0)).unwrap();
        let easy = tight.with_allowance(metres(30.0)).unwrap();
        assert!(tight.check(&snapshot(here)).unwrap().0.is_dragging());
        assert!(!easy.check(&snapshot(here)).unwrap().0.is_dragging());
    }

    #[test]
    fn a_watch_needs_a_position_and_sensible_radii() {
        let watch = AnchorWatch::new(at(50.0, -1.0), metres(100.0)).unwrap();
        assert!(matches!(
            watch.check(&NavigationSnapshot::EMPTY).unwrap_err(),
            NavigationError::Kernel(KernelError::Missing { .. })
        ));
        assert!(AnchorWatch::new(at(50.0, -1.0), metres(-1.0)).is_err());
        assert!(watch.with_allowance(metres(-1.0)).is_err());
        assert_eq!(watch.allowance(), Distance::ZERO);
        assert_eq!(watch.anchor(), at(50.0, -1.0));
        assert_eq!(watch.swinging_radius(), metres(100.0));
    }
}
