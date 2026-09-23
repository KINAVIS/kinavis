//! Compass swing observations.

use crate::angle::{wrap180, Compass, Deviation, Direction, True, Variation};
use crate::error::Result;

/// One observed heading of a swing.
///
/// Deviation is not measured directly: a bearing of an object with known true
/// bearing (transit, distant mark, celestial body) is taken by compass on each
/// heading. Deviation follows from:
///
/// ```text
/// deviation = reference bearing − variation − observed bearing
/// ```
///
/// # Example
///
/// ```rust
/// use kinavis::{
///     CompassBearing, CompassCourse, DeviationTable, NavigationError, SwingObservation,
///     TrueBearing, Variation,
/// };
///
/// fn main() -> Result<(), NavigationError> {
///     let variation = Variation::new(-2.0)?;
///     // A transit whose charted direction is 045°T, observed from four headings.
///     let transit = TrueBearing::new(45.0)?;
///     let observations = [
///         (0.0, 48.5),
///         (90.0, 46.0),
///         (180.0, 45.5),
///         (270.0, 48.0),
///     ]
///     .into_iter()
///     .map(|(heading, observed)| {
///         Ok(SwingObservation {
///             compass_heading: CompassCourse::new(heading)?,
///             observed_bearing: CompassBearing::new(observed)?,
///             reference_bearing: transit,
///         })
///     })
///     .collect::<Result<Vec<_>, NavigationError>>()?;
///
///     let table = DeviationTable::from_swing(&observations, variation)?;
///
///     // On north the compass read 048.5 for something that is really 045.0,
///     // with 2°W variation: deviation is 045.0 − (−2.0) − 048.5 = −1.5°.
///     assert_eq!(table.deviation_at_node(0).unwrap().degrees(), -1.5);
///     Ok(())
/// }
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SwingObservation {
    /// Compass heading the ship was steadied on.
    pub compass_heading: Direction<Compass>,
    /// Compass bearing of the reference.
    pub observed_bearing: Direction<Compass>,
    /// True bearing of the reference.
    pub reference_bearing: Direction<True>,
}

impl SwingObservation {
    /// Deviation implied by the observation for the given variation.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`](crate::KernelError::OutOfRange) if the
    /// bearings imply a deviation beyond ±180° (an input is wrong).
    pub fn deviation(&self, variation: Variation) -> Result<Deviation> {
        Ok(Deviation::new(wrap180(
            self.reference_bearing.degrees()
                - variation.degrees()
                - self.observed_bearing.degrees(),
        ))?)
    }
}
