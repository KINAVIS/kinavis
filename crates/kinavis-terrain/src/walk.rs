//! Walking time over a profile: Naismith's rule and its kin.

use core::time::Duration;

use kinavis_kernel::error::{ensure_range, Result};
use kinavis_kernel::{Distance, Speed};

use crate::analysis::ProfileSummary;

/// A walker's pace: speed on the level and the climb that costs an hour.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WalkingPace {
    level: Speed,
    climb_per_hour_metres: f64,
}

impl WalkingPace {
    /// Naismith's rule (1892): 5 km/h on the level, an hour more for every
    /// 600 m climbed; descent costs nothing.
    pub const NAISMITH: Self = Self {
        level: Speed::from_knots_unchecked(5.0 / 1.852),
        climb_per_hour_metres: 600.0,
    };

    /// Pace of `level` on the level and an hour per `climb_per_hour` of
    /// ascent.
    ///
    /// # Errors
    ///
    /// [`kinavis_kernel::KernelError::OutOfRange`] for a level speed outside
    /// 0.5–30 km/h or a climb outside 50–3000 m per hour.
    pub fn new(level: Speed, climb_per_hour: Distance) -> Result<Self> {
        ensure_range(
            "walking speed, km/h",
            level.metres_per_second() * 3.6,
            0.5,
            30.0,
        )?;
        ensure_range("climb per hour, m", climb_per_hour.metres(), 50.0, 3_000.0)?;
        Ok(Self {
            level,
            climb_per_hour_metres: climb_per_hour.metres(),
        })
    }

    /// Time to walk `length` with `ascent` of climbing.
    #[must_use]
    pub fn time(&self, length: Distance, ascent: Distance) -> Duration {
        let level = length.metres().max(0.0) / self.level.metres_per_second();
        let climb = ascent.metres().max(0.0) / self.climb_per_hour_metres * 3600.0;
        Duration::try_from_secs_f64((level + climb).clamp(0.0, 1e9)).unwrap_or(Duration::ZERO)
    }

    /// Time to walk a profile.
    #[must_use]
    pub fn time_over(&self, profile: &ProfileSummary) -> Duration {
        self.time(profile.length, profile.ascent)
    }
}
