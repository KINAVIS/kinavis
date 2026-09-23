//! Marine navigation algorithms: compass and deviation, sailings, dead
//! reckoning, position fixing, passage planning and guidance, tides, the sun,
//! and state estimation.
//!
//! ```text
//! magnetic course = compass course  + deviation(compass course)
//! true course     = magnetic course + variation
//! ```
//!
//! # Type guarantees
//!
//! Angles carry their reference frame: [`CompassCourse`], [`MagneticCourse`],
//! [`TrueCourse`], [`GyroCourse`], [`Variation`], [`Deviation`],
//! [`RelativeBearing`]. Passing a magnetic course where a true one is expected,
//! or a variation where a course is expected, does not compile. [`Distance`]
//! and [`Speed`] are types, so knots cannot be passed as m/s.
//!
//! Each type enforces its range: a [`Direction`] is finite and in `[0°, 360°)`,
//! a [`Latitude`] in `[-90°, 90°]`, so pure corrections return values, not
//! `Result`.
//!
//! No panics on caller data; invalid input returns a [`NavigationError`].
//!
//! # Example
//!
//! ```rust
//! use kinavis::{
//!     navigation_solutions::{
//!         convert_compass_course_to_true_course, convert_true_course_to_compass_course,
//!     },
//!     CompassCourse, DeviationTable, InterpolationMethod, TrueCourse, Variation,
//! };
//!
//! // A swing: deviation observed on every tenth of the compass, 000° to 350°.
//! let table = DeviationTable::from_deviations(&[
//!     -2.5, -0.5, 1.6, 4.4, -1.7, 0.0, 1.0, 0.3, -0.9, // 000°..080°
//!     0.5, -1.2, 0.8, -0.3, 1.7, -2.1, 0.4, -0.6, 1.2, // 090°..170°
//!     -1.3, 0.0, 0.9, -1.1, 1.5, -0.7, -13.2, -15.7, -17.9, // 180°..260°
//!     -19.2, -18.1, 1.8, -0.4, 0.7, -0.2, 1.4, -4.4, -2.9, // 270°..350°
//! ])?;
//!
//! let variation = Variation::new(-2.7)?;
//!
//! // What is the ship actually making good, steering 003° by the compass?
//! let solution = convert_compass_course_to_true_course(
//!     CompassCourse::new(3.0)?,
//!     variation,
//!     &table,
//!     InterpolationMethod::Cubic,
//! )?;
//! assert_eq!(format!("{}", solution.course), "358.2°T");
//!
//! // And back again: the inverse solves for the compass course the table is
//! // indexed by, so the two directions agree.
//! let back = convert_true_course_to_compass_course(
//!     solution.course,
//!     variation,
//!     &table,
//!     InterpolationMethod::Cubic,
//! )?;
//! assert!((back.course.degrees() - 3.0).abs() < 1e-9);
//!
//! // This swing jumps 12.5° between 230° and 240°, steeper than a compass
//! // can be steered by; the result flags it.
//! assert!(solution.advisories.non_invertible_table);
//! # Ok::<(), kinavis::NavigationError>(())
//! ```
//!
//! # Modules
//!
//! Value types come from [`kinavis_kernel`] and are re-exported under this
//! crate's paths; the algorithms are this crate's own. Adapters that must not
//! pull in the algorithms depend on the kernel alone.
//!
//! - [`angle`] — frame-tagged angles.
//! - [`units`] — angles, distances, speeds, rate of turn.
//! - [`position`] — latitude, longitude, notation.
//! - [`time`] — instants with the time scale in the type; leap-second port.
//! - [`observation`] — a value with its time and quality.
//! - [`gnss`] — satellite fix.
//! - [`geodesy`] — ellipsoids, heights with datum, ECEF, chart datums and their
//!   transformation to WGS 84.
//! - [`local`] — NED and other local frames; vectors typed by frame and unit.
//! - [`snapshot`] — read model: position, motion, uncertainty, age.
//! - [`state`] — navigation state aggregate.
//! - [`estimation`] — estimator ports.
//! - [`environment`] — environment ports and the resolved sample.
//! - [`conditions`] — constant and timetabled current and wind, fixed leeway.
//! - [`tides`] — rule of twelfths, secondary ports, tidal diamonds as a current
//!   model.
//! - [`sun`] — solar azimuth and altitude, sunrise, sunset, twilights.
//! - [`event`] — events of this crate's use cases.
//! - [`deviation`] — deviation tables, periodic interpolation, coefficient
//!   fitting.
//! - [`navigation_solutions`] — course and bearing conversions, gyro error,
//!   current triangle.
//! - [`sailings`] — rhumb line, great circle, WGS 84 geodesic, cross-track
//!   error.
//! - [`dead_reckoning`] — DR and EP, traverses, leeway.
//! - [`fix`] — position lines, fixes, cocked hats, distance off.
//! - [`relative_motion`] — CPA, radar plotting, avoiding manoeuvre.
//! - [`route`] — passage plans: legs, distances, progress along track.
//! - [`turning`] — leg-to-leg turns: radius or rate of turn, advance and
//!   transfer, wheel-over point.
//! - [`guidance`] — what to steer now: track, course to steer, XTE, next
//!   wheel-over, events.
//! - [`schedule`] — speed per leg, ETD and ETA, time to go, ahead/behind,
//!   required speed.
//! - [`clearance`] — squat (Barrass) and under-keel clearance against the
//!   vessel's policy.
//! - [`composite`] — composite great-circle sailing below a limiting latitude,
//!   as rhumb legs.
//! - [`anchor`] — anchor watch: swinging circle and dragging detection.
//! - [`mob`] — man overboard datum drifted by current and wind.
//! - [`gnss_intake`] — position from a GNSS fix stream: rejection, loss,
//!   acquisition, snapshot.
//! - [`estimator`] — extended Kalman filter over the navigation state: pure
//!   steps and a thin shell.
//! - [`observations`] — standard observations: position, velocity, heading,
//!   speed through water.
//! - [`error`] — the error type.
//!
//! # Models
//!
//! Spherical sailings use a mean Earth radius of 6371.0088 km;
//! [`sailings::geodesic`] uses the WGS 84 ellipsoid. Position lines are rhumb
//! lines and intersect exactly on a Mercator chart; range fixes and relative
//! motion are planar. Each function documents its model.
//!
//! # Memory
//!
//! No allocation. Deviation tables, routes and error excerpts are stored inline
//! with bounds [`MAX_TABLE_NODES`], [`MAX_WAYPOINTS`] and [`EXCERPT_BYTES`];
//! exceeding them returns [`KernelError::CapacityExceeded`]. Batch computations
//! write into caller-owned buffers (`*_into`). The crate runs on bare metal
//! without an allocator, with memory use known at compile time.
//!
//! Aggregates are therefore large: pass [`DeviationTable`] and [`Route`] by
//! reference.
//!
//! # Feature flags
//!
//! - `std` *(default)* — standard library floating-point maths; implies
//!   `alloc`.
//! - `alloc` — `Vec`-returning companions of the `*_into` calls.
//! - `libm` — for `no_std` targets: `--no-default-features --features libm`.
//! - `serde` — serialisation of the value types; deserialisation applies
//!   construction-time validation (no latitude of 500°, no duplicate headings
//!   in a table). Implies `alloc`.
//!
//! No dependencies in the default configuration.

#![cfg_attr(not(feature = "std"), no_std)]

// An allocator is needed only by the `alloc` convenience functions; test
// modules use `vec!` and `format!`, hence `test`. The bare-metal CI build
// enables neither.
#[cfg(any(feature = "alloc", test))]
extern crate alloc;

// Kernel modules under this crate's paths, inlined so the docs show one crate.
#[doc(inline)]
pub use kinavis_kernel::{
    angle, environment, estimation, geodesy, gnss, local, observation, position, snapshot, state,
    time, units,
};

// Crate-private: numeric primitives and fixed-capacity storage. Public in the
// kernel for adapters, not part of this crate's API.
use kinavis_kernel::{inline, math, matrix};

pub mod anchor;
pub mod clearance;
pub mod composite;
pub mod conditions;
pub mod dead_reckoning;
pub mod deviation;
pub mod error;
pub mod estimator;
pub mod event;
pub mod fix;
pub mod gnss_intake;
pub mod guidance;
mod linalg;
pub mod mob;
pub mod navigation_solutions;
pub mod observations;
pub mod relative_motion;
pub mod route;
pub mod sailings;
pub mod schedule;
pub mod sun;
pub mod tides;
pub mod turning;

/// Runs the crate and project README examples as doctests, so documented
/// numbers cannot drift from the code.
#[cfg(doctest)]
#[doc = include_str!("../README.md")]
pub struct ReadmeExamples;

#[cfg(doctest)]
#[doc = include_str!("../../../README.md")]
pub struct ProjectReadmeExamples;

#[doc = include_str!("guide.md")]
pub mod guide {}

pub use anchor::{anchor_position, swinging_radius, AnchorView, AnchorWatch};
pub use angle::{
    wrap180, wrap360, CardinalPoint, Compass, CompassBearing, CompassCourse, Deviation, Direction,
    Frame, Gyro, GyroBearing, GyroCourse, Magnetic, MagneticBearing, MagneticCourse,
    RelativeBearing, Side, True, TrueBearing, TrueCourse, Variation, MAX_DEVIATION_DEG,
    MAX_VARIATION_DEG,
};
pub use clearance::{
    height_of_tide_needed, squat, under_keel_clearance, Clearance, ClearancePolicy, Hull, Waterway,
};
pub use composite::{composite_sailing, CompositeSailing, ParallelRun};
pub use conditions::{Constant, FixedLeeway, Timetable};
pub use dead_reckoning::{EstimatedPosition, Leg};
pub use deviation::{
    DeviationAnalysis, DeviationCoefficients, DeviationNode, DeviationTable, InterpolatedTable,
    Interpolation, InterpolationMethod, SmithCoefficients, SwingObservation, MAX_TABLE_NODES,
    STANDARD_TABLE_LEN,
};
pub use environment::{
    CompassModel, CurrentModel, EnvironmentSample, LeewayModel, MagneticField, MagneticModel,
    TideModel, VesselMotion, Wind, WindModel, MAX_FIELD_NANOTESLA,
};
pub use error::{Excerpt, KernelError, NavigationError, Result, EXCERPT_BYTES};
pub use estimator::{
    Estimator, EstimatorConfig, LatePolicy, Outcome, SteadyMotion, UpdateReport, MAX_HISTORY,
    MAX_SENSORS,
};
pub use event::{
    AnchorEvent, ClearanceEvent, Event, EventList, GuidanceEvent, NavigationEvent,
    NavigationIntegrity, PositionSource, RejectionReason, SensorHealth, SensorId, TargetId,
    MAX_EVENTS, SENSOR_NAME_BYTES,
};
pub use fix::{CockedHat, Fix, PositionLine, TwoBearingDistance};
pub use geodesy::{Datum, EcefPoint, Ellipsoid, GeodeticPoint, Height, Helmert, VerticalDatum};
pub use gnss::{Dop, FixType, GnssFix, GnssFixBuilder, GnssQuality};
pub use gnss_intake::{GnssIntake, IntakeConfig, IntakeOutcome};
pub use guidance::{guide, GuidanceConfig, GuidanceView};
pub use inline::InlineStr;
pub use local::{Body, Enu, LocalFrame, Ned, Vector3, VectorFrame, VectorUnit};
pub use mob::{ManOverboard, MobDatum};
pub use navigation_solutions::{
    Advisories, CourseSolution, Current, GroundTrack, SteeringSolution, COARSE_TABLE_GAP_DEG,
    LARGE_DEVIATION_DEG, LARGE_VARIATION_DEG, MAX_BISECTIONS_INVERSE_DEVIATION,
    MAX_ITERATIONS_INVERSE_DEVIATION, TOLERANCE_INVERSE_DEVIATION_DEG,
};
pub use observation::{ObservationStatus, Observed, Quality};
pub use observations::{
    HeadingObservation, PositionObservation, SpeedThroughWaterObservation, VelocityObservation,
};
pub use position::{EastWest, GeocentricUnit, Latitude, Longitude, NorthSouth, Position};
pub use relative_motion::{Approach, Avoidance, Contact, Cpa, TargetSolution, Vessel};
pub use route::{LegCursor, LegKind, Progress, Route, RouteLeg, MAX_WAYPOINTS};
pub use sailings::{
    Arrival, CrossTrack, Sailing, TrackSide, EARTH_RADIUS, MAX_ITERATIONS_GEODESIC,
    TOLERANCE_GEODESIC_RAD,
};
pub use schedule::{RouteEstimate, RouteSchedule, ScheduledLeg};
pub use snapshot::{ErrorEllipse, NavigationSnapshot};
pub use state::{NavigationState, StateComponent, StatePriors};
pub use sun::{Crossing, Horizon, SolarPosition};
pub use tides::{SecondaryPort, StreamHour, TidalCycle, TidalStream, TideEvent};
pub use time::{Civil, Gps, Instant, LeapSeconds, Tai, TimeScale, Utc};
pub use turning::{wheel_over_point, Turn, TurnMode, TurnParameters};
pub use units::{Angle, Distance, RateOfTurn, Speed};
