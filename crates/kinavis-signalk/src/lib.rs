//! Signal K deltas in and out of the KINAVIS navigation crates.
//!
//! A [Signal K](https://signalk.org) server merges a vessel's NMEA 0183,
//! NMEA 2000 and AIS into one JSON model and streams changes as deltas. This
//! crate reads those deltas into a [`Picture`] of own vessel and every
//! target, and a [`Watch`] assesses the picture: CPA and TCPA of each target
//! (`kinavis-traffic`), and for a dangerous or developing approach the
//! COLREGs ruling (`kinavis-colregs`): the encounter, who gives way and what
//! each may do. The results go back to the server as
//! `navigation.closestApproach` deltas in each target's context and as
//! notifications on own vessel.
//!
//! Signal K units are SI: radians, metres, metres per second. The crate
//! converts at the boundary; everything inside is KINAVIS types.
//!
//! ```rust
//! use kinavis_signalk::{parse_timestamp, Emission, Level, Watch, WatchConfig};
//!
//! let mut watch = Watch::new(WatchConfig::default());
//! // Own vessel steering north at 6 knots (3.087 m/s)...
//! watch.ingest(r#"{"context":"vessels.self","updates":[{"timestamp":"2026-09-25T12:00:00Z","values":[
//!     {"path":"navigation.position","value":{"latitude":53.0,"longitude":5.0}},
//!     {"path":"navigation.courseOverGroundTrue","value":0.0},
//!     {"path":"navigation.speedOverGround","value":3.087}]}]}"#)?;
//! // ...and a motor vessel a mile and a half north and east, steering west at 6 knots.
//! watch.ingest(r#"{"context":"vessels.urn:mrn:imo:mmsi:244000001","updates":[{"timestamp":"2026-09-25T12:00:00Z","values":[
//!     {"path":"","value":{"name":"ANNA"}},
//!     {"path":"navigation.position","value":{"latitude":53.025,"longitude":5.04155}},
//!     {"path":"navigation.courseOverGroundTrue","value":4.712389},
//!     {"path":"navigation.speedOverGround","value":3.087},
//!     {"path":"navigation.state","value":"motoring"}]}]}"#)?;
//!
//! let now = parse_timestamp("2026-09-25T12:00:01Z").unwrap();
//! let (reports, emissions) = watch.assess(now);
//! assert_eq!(reports[0].level, Level::Alarm);
//! let Some(Emission::Notification { path, value }) = emissions.last() else { panic!() };
//! assert_eq!(path, "notifications.navigation.closestApproach.urn:mrn:imo:mmsi:244000001");
//! assert_eq!(value["state"], "alarm");
//! assert!(value["message"].as_str().unwrap().contains("give way (Rule 15)"));
//! # Ok::<(), serde_json::Error>(())
//! ```
//!
//! Requires the standard library: JSON, and a picture as large as the server's.

mod delta;
mod picture;
mod time;
mod watch;

/// Runs the `README.md` example as a doctest.
#[cfg(doctest)]
#[doc = include_str!("../README.md")]
pub struct ReadmeExamples;

pub use delta::{Delta, PathValue, Update};
/// The clock [`Watch::assess`] takes.
pub use kinavis_kernel::time::{Instant, Utc};
pub use picture::{Category, Picture, VesselState, SELF_CONTEXT};
pub use time::parse_timestamp;
pub use watch::{message, Emission, Level, TargetReport, Watch, WatchConfig, NOTIFICATION_PATH};
