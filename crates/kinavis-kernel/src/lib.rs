//! Value types of the KINAVIS crates.
//!
//! The kernel holds what has exactly one correct implementation: units, angles,
//! positions, time, the error type. Anything with a choice of method (sailing,
//! interpolation, fix) lives in `kinavis`; every satellite crate (parsers,
//! magnetic model, INS) depends on the kernel alone. This is the workspace
//! dependency rule: adapters change without touching the core.
//!
//! Most users should depend on `kinavis`, which re-exports these modules.
//! Depend on `kinavis-kernel` directly for an adapter that must not pull in the
//! algorithms.
//!
//! # Type guarantees
//!
//! Angles carry their reference frame: [`CompassCourse`], [`MagneticCourse`],
//! [`TrueCourse`], [`GyroCourse`], [`Variation`], [`Deviation`],
//! [`RelativeBearing`]. Passing a magnetic course where a true one is expected
//! does not compile. [`Distance`] and [`Speed`] are types, so knots cannot be
//! passed as m/s.
//!
//! Each type enforces its range: a [`Direction`] is always finite and in `[0°,
//! 360°)`, a [`Latitude`] in `[-90°, 90°]`. Existence implies validity, so
//! corrections built on them return values, not `Result`. Invalid input is
//! rejected at construction with a [`KernelError`]; nothing panics on caller
//! data.
//!
//! # Modules
//!
//! - [`angle`] — frame-tagged angles.
//! - [`units`] — angles, distances, speeds, rate of turn.
//! - [`position`] — latitude, longitude, notation.
//! - [`error`] — the error type and validation helpers.
//! - [`time`] — instants with the time scale in the type, calendar, leap-second
//!   port.
//! - [`observation`] — a value with its time and quality; age is computed.
//! - [`geodesy`] — ellipsoids, heights with datum, ECEF, chart datums and their
//!   transformation to WGS 84.
//! - [`local`] — NED and other local frames; vectors typed by frame and unit; a
//!   frame anchored at a point.
//! - [`gnss`] — a satellite fix, independent of the sentence format.
//! - [`state`] — navigation state aggregate behind invariants, projected to the
//!   read model.
//! - [`estimation`] — estimator ports: process model and observation.
//! - [`environment`] — environment ports (magnetic field, current, wind, tide,
//!   leeway, deviation) and the sample they return.
//! - [`snapshot`] — read model: position, motion, uncertainty and age, for
//!   displays and alarms.
//! - [`event`] — events returned as values in a fixed-capacity list, never via
//!   callbacks.
//! - [`matrix`] — small dense matrices with checked access and Cholesky
//!   factorisation.
//! - [`math`] — floating-point primitives routed to `std` or `libm`. Every
//!   transcendental call in the crate family goes through here, for
//!   reproducibility across targets.
//! - [`inline`] — fixed-capacity storage; no allocator required.
//!
//! # Feature flags
//!
//! - `std` *(default)* — standard library floating-point maths.
//! - `libm` — for `no_std` targets: `--no-default-features --features libm`.
//! - `serde` — serialisation of the value types; deserialisation applies the
//!   same validation as construction.
//! - `alloc` — unused here; lets dependent crates name one flag for the whole
//!   family.
//!
//! No dependencies in the default configuration.
//!
//! # Hidden items
//!
//! Some public items are `#[doc(hidden)]`: constructors that accept a raw value
//! unchecked (`from_degrees_unchecked`, `from_knots_unchecked`,
//! `from_degrees_wrapped`, `relabel`) and accessors exposing internal
//! representation (`NavigationState` vector, covariance and `from_parts`, the
//! state dimension, Jacobian entries). The algorithm crates need them — a
//! solver wrapping an already-bounded angle should not pay for a check, an
//! estimator must see the covariance — and Rust has no visibility between
//! crates other than `pub`. They form an internal contract of the crate family:
//! not covered by the stability guarantee, may change in a minor release, and
//! marked as such in their docs. Use them only from crates released together
//! with the kernel.

#![cfg_attr(not(feature = "std"), no_std)]

// The crate does not allocate; test modules use `format!`, hence `test`. The
// bare-metal CI build enables neither.
#[cfg(any(feature = "alloc", test))]
extern crate alloc;

pub mod angle;
pub mod environment;
pub mod error;
pub mod estimation;
pub mod event;
pub mod geodesy;
pub mod gnss;
pub mod inline;
pub mod local;
pub mod math;
pub mod matrix;
pub mod observation;
mod parse;
pub mod position;
pub mod snapshot;
pub mod state;
pub mod time;
pub mod units;

pub use angle::{
    wrap180, wrap360, CardinalPoint, Compass, CompassBearing, CompassCourse, Deviation, Direction,
    Frame, Gyro, GyroBearing, GyroCourse, Magnetic, MagneticBearing, MagneticCourse,
    RelativeBearing, Side, True, TrueBearing, TrueCourse, Variation, MAX_DEVIATION_DEG,
    MAX_VARIATION_DEG,
};
pub use environment::{
    CompassModel, Current, CurrentModel, EnvironmentSample, LeewayModel, MagneticField,
    MagneticModel, TideModel, VesselMotion, Wind, WindModel, MAX_FIELD_NANOTESLA,
};
pub use error::{Excerpt, KernelError, Result, EXCERPT_BYTES};
pub use estimation::{
    GatingPolicy, JacobianRow, Observation, ObservationJacobian, ObservationNoise,
    ObservationVector, ProcessModel, ProcessNoise, StateJacobian, MAX_OBSERVATION_DIM,
};
pub use event::{
    Event, EventList, NavigationEvent, NavigationIntegrity, PositionSource, RejectionReason,
    SensorHealth, SensorId, TargetId, MAX_EVENTS, SENSOR_NAME_BYTES,
};
pub use geodesy::{Datum, EcefPoint, Ellipsoid, GeodeticPoint, Height, Helmert, VerticalDatum};
pub use gnss::{Dop, FixType, GnssFix, GnssFixBuilder, GnssQuality};
pub use inline::InlineStr;
pub use local::{Body, Enu, LocalFrame, Ned, Vector3, VectorFrame, VectorUnit};
pub use observation::{ObservationStatus, Observed, Quality};
pub use position::{EastWest, GeocentricUnit, Latitude, Longitude, NorthSouth, Position};
pub use snapshot::{ErrorEllipse, GroundTrack, NavigationSnapshot};
pub use state::{NavigationState, StateComponent, StatePriors};
pub use time::{Civil, Gps, Instant, LeapSeconds, Tai, TimeScale, Utc};
pub use units::{Angle, Distance, RateOfTurn, Speed};
