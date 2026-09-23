//! Tides: height between high and low water, secondary port corrections, tidal
//! stream as a [`CurrentModel`].
//!
//! From tide-table high and low waters at a standard port to the height or time
//! needed at a nearby place, in three steps:
//!
//! 1. [`SecondaryPort`] — Admiralty time and height differences applied to the
//!    standard port's high and low waters.
//! 2. [`TidalCycle`] — one rise or fall between turning points: height at any
//!    time by the rule of twelfths, and time of any height by its inverse.
//! 3. [`TidalStream`] — tidal diamond: set and rate hour by hour relative to
//!    HW, springs and neaps, interpolated for the day's range and the requested
//!    time, as a [`CurrentModel`].
//!
//! # Rule of twelfths
//!
//! Over a six-hour rise the tide gains 1, 2, 3, 3, 2, 1 twelfths of the range
//! per hour: a piecewise-linear approximation of a cosine, accurate to a few
//! centimetres for regular semi-diurnal tides. [`TidalCycle::height_at`]
//! applies it scaled to the actual cycle duration. Irregular tides (Solent
//! double high waters, diurnal tides) are out of scope; their tables give
//! dedicated curves.
//!
//! Heights are above chart datum, as tabulated; no datum conversion.
//!
//! ```rust
//! use kinavis::tides::{SecondaryPort, TidalCycle, TideEvent};
//! use kinavis::{Civil, Distance, Instant, Utc};
//!
//! let at = |hour, minute| Instant::<Utc>::from_civil(Civil { hour, minute, ..Civil::date(2026, 9, 15) });
//!
//! // The standard port: low water 0412 0.8 m, high water 1030 4.6 m.
//! let low = TideEvent::new(at(4, 12)?, Distance::from_metres(0.8)?);
//! let high = TideEvent::new(at(10, 30)?, Distance::from_metres(4.6)?);
//!
//! // The secondary port, from the tables: HW +0020, LW +0035, heights −0.4 and −0.1.
//! let port = SecondaryPort::new()
//!     .high_water(20, Distance::from_metres(-0.4)?)
//!     .low_water(35, Distance::from_metres(-0.1)?);
//! let rise = TidalCycle::new(port.adjust_low_water(low), port.adjust_high_water(high))?;
//!
//! // The height at 0800, and when there is first 3 m of water.
//! assert_eq!(format!("{:.2}", rise.height_at(at(8, 0)?)?.metres()), "2.62");
//! let enough = rise.time_of_height(Distance::from_metres(3.0)?)?;
//! assert_eq!(format!("{}", enough), "2026-09-15T08:26:31.714 UTC");
//! # Ok::<(), kinavis::NavigationError>(())
//! ```

mod height;
mod stream;

pub use height::{SecondaryPort, TidalCycle, TideEvent};
pub use stream::{StreamHour, TidalStream, HOURS_EITHER_SIDE};

// Imported for the module docs link.
#[allow(unused_imports)]
use crate::environment::CurrentModel;
