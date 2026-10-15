//! Field guidance for the KINAVIS crates: a low-distortion field plane, AB
//! lines, curves and pivot circles, parallel passes, tilt compensation, pass
//! trackers that command a wheel angle.
//!
//! A field is worked in a plane. [`FieldPlane::centred_at`] lays one through
//! the field — a transverse Mercator with its central meridian and origin on
//! the field, scaled to the field's height — so that a metre in the plane is a
//! metre on the ground to a few parts in a million. Boundaries
//! ([`Field`]), reference lines ([`AbLine`], [`CurveLine`], [`PivotLine`]) and
//! passes all live in it; directions in it are grid directions
//! ([`GridCourse`](kinavis_kernel::GridCourse)).
//!
//! ```rust
//! use kinavis::region::PlanarPoint;
//! use kinavis_agri::{
//!     ground_point, AbLine, FieldPlane, LeverArm, PassIndex, PassPattern, PassTracker, Tilt,
//!     TrackerConfig,
//! };
//! use kinavis_kernel::{Angle, Civil, Distance, GridCourse, Height, Instant, Position, Utc};
//!
//! // A field near Wageningen at 10 m above the ellipsoid.
//! let a = Position::from_degrees(51.9850, 5.6600)?;
//! let plane = FieldPlane::centred_at(a, Height::above_ellipsoid(Distance::from_metres(10.0)?))?;
//!
//! // An AB line due grid north through A; a 12 m boom with 20 cm overlap.
//! let line = AbLine::from_point_and_course(plane, plane.to_plane(a)?, GridCourse::new(0.0)?);
//! let passes = PassPattern::new(Distance::from_metres(12.0)?, Distance::from_metres(0.2)?)?;
//!
//! // The antenna, 3 m up, reads 46.9 m east of A; the tractor heads south
//! // rolled 4° to the right on a side slope, so the antenna leans 21 cm west.
//! let antenna = PlanarPoint::from_metres(46.9, 120.0)?;
//! let heading = GridCourse::new(180.0)?;
//! let tilt = Tilt::new(Angle::from_degrees(4.0)?, Angle::ZERO)?;
//! let ground = ground_point(antenna, heading, tilt, LeverArm::above(Distance::from_metres(3.0)?))?;
//!
//! let config = TrackerConfig::new(
//!     Distance::from_metres(0.3)?,
//!     Distance::from_metres(15.0)?,
//!     Distance::from_metres(0.5)?,
//! )?;
//! let mut tracker = PassTracker::new(config);
//! let noon = Instant::<Utc>::from_civil(Civil::date(2026, 4, 20))?;
//! let (view, _events) = tracker.update(&line, &passes, ground, heading, noon);
//! assert_eq!(view.pass, PassIndex::new(-4)); // 4 × 11.8 m to the right of the line
//! assert!(view.reversed); // driving it south
//! // Ground 47.109 m east, the pass at 47.2 m: 9 cm right (west) of it, southbound.
//! assert_eq!(format!("{:+.3}", view.cross_track.metres()), "+0.091");
//! # Ok::<(), kinavis_agri::AgriError>(())
//! ```
//!
//! # Steering
//!
//! [`PurePursuit`] and [`Stanley`] turn a pass and a machine state into a
//! wheel angle ([`SteeringCommand`]) for the machine's [`VehicleGeometry`];
//! both are [`PathTracker`]s, so code that steers is written once.
//! [`bicycle_step`] is the kinematic model they are tested against. The
//! output is a command, not control: the actuator and its safety are the
//! caller's.
//!
//! # Turns
//!
//! [`TurnPath`] is the shortest path of bounded curvature between two poses
//! (Dubins): a U-turn onto a pass two turning radii away or more, an omega
//! turn onto a closer one. It is a [`ReferenceLine`], so the same trackers
//! drive it, and [`TurnPath::first_conflict`] checks it against the field
//! for a machine of a given width.
//!
//! Without an inertial system, [`BicycleMotion`] is the process model of the
//! `kinavis` estimator for a steered machine: fed the wheel angles, it holds
//! a turn through a gap in the fixes.
//!
//! # Accuracy
//!
//! The centred plane's scale error is about `x²/2R²` at `x` from its origin:
//! 3·10⁻⁷ at 5 km, 5·10⁻⁶ at the 20 km radius beyond which points are
//! refused. A zoned grid ([`FieldPlane::grid`]) is accepted for data that
//! arrives in it, with its own error: up to 10⁻³ at the edge of a UTM zone.
//!
//! # Events
//!
//! [`PassTracker`] reports [`AgriEvent`]s as values. `kinavis-alerts` does
//! not know them; an application that runs an alert board wraps them in its
//! own event union and implements `Reportable` there.
//!
//! # Feature flags
//!
//! - `std` *(default)* — standard library maths.
//! - `libm` — for `no_std` targets: `--no-default-features --features libm`.
//!
//! No allocation; builds for bare-metal targets.

#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(test)]
extern crate std;

/// Runs the README example as a doctest.
#[cfg(doctest)]
#[doc = include_str!("../README.md")]
pub struct ReadmeExamples;

mod bicycle;
mod curve;
mod error;
mod field;
mod line;
mod pass;
mod plane;
mod steering;
mod tilt;
mod tracker;
mod turn;

pub use bicycle::BicycleMotion;
pub use curve::{ContourRecorder, CurveLine, MIN_SEGMENT_METRES};
pub use error::{AgriError, Result};
pub use field::Field;
pub use line::{AbLine, PivotLine, ReferenceLine, Station, MIN_LINE_METRES};
pub use pass::{PassIndex, PassPattern, MAX_PASS};
pub use plane::{FieldPlane, FIELD_RADIUS_METRES};
pub use steering::{
    bicycle_step, PathTracker, PurePursuit, Stanley, SteeringCommand, VehicleGeometry, VehicleState,
};
pub use tilt::{ground_point, LeverArm, Tilt};
pub use tracker::{locate, AgriEvent, PassTracker, PassView, TrackerConfig, MAX_AGRI_EVENTS};
pub use turn::{DubinsWord, Pose, Steer, TurnPath, TurnPiece};
