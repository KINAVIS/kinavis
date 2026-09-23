//! Inverse deviation: compass course whose deviation yields a given magnetic
//! course.

use crate::angle::{wrap180, wrap360};
use crate::error::{KernelError, NavigationError, Result};
use crate::math;

use super::{
    MAX_BISECTIONS_INVERSE_DEVIATION, MAX_ITERATIONS_INVERSE_DEVIATION,
    TOLERANCE_INVERSE_DEVIATION_DEG,
};

/// Solves `compass + δ(compass) = magnetic` for the compass course.
///
/// Deviation is tabulated against compass course, so the equation is implicit.
/// A damped fixed-point iteration converges in a few steps for realistic
/// swings; a bracketing fallback handles curves steep enough to make it
/// oscillate.
///
/// `deviation` returns δ in degrees for a course in `[0°, 360°)`; errors (as
/// from a [`CompassModel`]) are propagated.
pub(super) fn solve_compass_course(
    deviation: impl Fn(f64) -> Result<f64>,
    magnetic_degrees: f64,
) -> Result<f64> {
    let residual = |compass: f64| -> Result<f64> {
        Ok(wrap180(compass + deviation(compass)? - magnetic_degrees))
    };

    let mut compass = magnetic_degrees;
    let mut damping = 1.0_f64;
    let mut previous_step = f64::MAX;

    for _ in 0..MAX_ITERATIONS_INVERSE_DEVIATION {
        let step = -residual(compass)?;
        let magnitude = math::abs(step);

        if magnitude < TOLERANCE_INVERSE_DEVIATION_DEG {
            return Ok(wrap360(compass));
        }
        // Oscillating: reduce the step.
        if magnitude >= previous_step {
            damping *= 0.5;
            if damping < 0.05 {
                break;
            }
        }
        previous_step = magnitude;
        compass = wrap360(compass + damping * step);
    }

    if let Some(bracketed) = bracket_and_bisect(&residual)? {
        return Ok(bracketed);
    }

    Err(NavigationError::Kernel(KernelError::NotConverged {
        iterations: MAX_ITERATIONS_INVERSE_DEVIATION,
        residual: math::abs(residual(compass)?),
    }))
}

/// Scans the circle for a residual sign change, then bisects.
///
/// `Ok(None)` if no root is bracketed; `Err` if the residual function fails.
fn bracket_and_bisect(residual: &impl Fn(f64) -> Result<f64>) -> Result<Option<f64>> {
    /// Samples per degree; enough to bracket any physically plausible curve.
    const SAMPLES: usize = 1440;
    /// Ignore sign changes caused by the ±180° wrap rather than a root.
    const WRAP_GUARD_DEG: f64 = 90.0;

    let step = 360.0 / math::count_to_f64(SAMPLES);
    let mut low = 0.0;
    let mut low_value = residual(low)?;

    for index in 1..=SAMPLES {
        let high = step * math::count_to_f64(index) % 360.0;
        let high_value = residual(high)?;

        let brackets_root =
            (low_value <= 0.0 && high_value >= 0.0) || (low_value >= 0.0 && high_value <= 0.0);
        let near_a_root = math::abs(low_value) + math::abs(high_value) < WRAP_GUARD_DEG;

        if brackets_root && near_a_root {
            let (mut left, mut right) = (low, if high < low { high + 360.0 } else { high });
            let mut left_value = low_value;

            for _ in 0..MAX_BISECTIONS_INVERSE_DEVIATION {
                let middle = f64::midpoint(left, right);
                let middle_value = residual(wrap360(middle))?;
                if math::abs(middle_value) < TOLERANCE_INVERSE_DEVIATION_DEG {
                    return Ok(Some(wrap360(middle)));
                }
                if (left_value <= 0.0) == (middle_value <= 0.0) {
                    left = middle;
                    left_value = middle_value;
                } else {
                    right = middle;
                }
            }
            return Ok(Some(wrap360(f64::midpoint(left, right))));
        }

        low = high;
        low_value = high_value;
    }

    Ok(None)
}
