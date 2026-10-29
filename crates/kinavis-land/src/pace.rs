//! Distance by pace count: pairs of steps, calibrated on a known course, and
//! reduced to the horizontal on a slope.

use kinavis_kernel::error::ensure_range;
use kinavis_kernel::{math, Angle, Distance};

use crate::error::Result;

/// Shortest and longest pair of steps accepted, m.
const PAIR_RANGE: (f64, f64) = (0.5, 4.0);

/// Steepest slope a pace count is reduced for, degrees.
pub const MAX_SLOPE_DEGREES: f64 = 60.0;

/// A walker's pace: the ground distance covered by one pair of steps (two
/// steps, the count of one foot).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PaceCount {
    pair_metres: f64,
}

impl PaceCount {
    /// Pace of `pair` per pair of steps.
    ///
    /// # Errors
    ///
    /// [`kinavis_kernel::KernelError::OutOfRange`] outside 0.5–4 m.
    pub fn new(pair: Distance) -> Result<Self> {
        let metres = pair.metres();
        ensure_range("pair of steps, m", metres, PAIR_RANGE.0, PAIR_RANGE.1)?;
        Ok(Self {
            pair_metres: metres,
        })
    }

    /// Pace from `pairs` counted over a measured course of `length`, walked
    /// on level ground.
    ///
    /// # Errors
    ///
    /// As [`PaceCount::new`] for the resulting pair; a count of zero or less
    /// is out of range too.
    pub fn calibrate(pairs: f64, length: Distance) -> Result<Self> {
        ensure_range("pairs of steps", pairs, 1.0, 1e7)?;
        Self::new(Distance::from_metres(length.metres() / pairs)?)
    }

    /// Ground distance of one pair of steps.
    #[must_use]
    pub fn pair(&self) -> Distance {
        Distance::from_metres(self.pair_metres).unwrap_or(Distance::ZERO)
    }

    /// Horizontal distance of `pairs` walked up or down `slope`: the ground
    /// distance times `cos slope`. The pace itself shortens on a climb; that
    /// is calibrated, not modelled.
    ///
    /// # Errors
    ///
    /// [`kinavis_kernel::KernelError::OutOfRange`] for a negative count or a
    /// slope beyond ±[`MAX_SLOPE_DEGREES`].
    pub fn distance(&self, pairs: f64, slope: Angle) -> Result<Distance> {
        ensure_range("pairs of steps", pairs, 0.0, 1e7)?;
        ensure_range(
            "slope, degrees",
            slope.degrees(),
            -MAX_SLOPE_DEGREES,
            MAX_SLOPE_DEGREES,
        )?;
        Ok(Distance::from_metres(
            pairs * self.pair_metres * math::cos(slope.radians()),
        )?)
    }

    /// Pairs of steps to cover a horizontal distance up or down `slope`.
    ///
    /// # Errors
    ///
    /// As [`PaceCount::distance`], and for a negative distance.
    pub fn pairs_for(&self, horizontal: Distance, slope: Angle) -> Result<f64> {
        ensure_range("horizontal distance, m", horizontal.metres(), 0.0, 1e8)?;
        let per_pair = self.distance(1.0, slope)?.metres();
        Ok(horizontal.metres() / per_pair)
    }
}
