//! Parallel passes: swath width, overlap, shift, pass numbers.

use core::fmt;

use kinavis_kernel::error::{ensure_finite, ensure_range};
use kinavis_kernel::{math, Distance};

use crate::error::Result;

/// Largest pass number either side of the reference line.
pub const MAX_PASS: i32 = 1_000_000;

/// Number of a pass: 0 is the reference line, positive passes lie to its
/// left, negative to its right (looking along the line's reference
/// direction).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(from = "i32", into = "i32")
)]
pub struct PassIndex(i32);

impl PassIndex {
    /// The reference line itself.
    pub const REFERENCE: Self = Self(0);

    /// Pass with this number, clamped to ±[`MAX_PASS`].
    #[must_use]
    pub const fn new(number: i32) -> Self {
        Self(if number > MAX_PASS {
            MAX_PASS
        } else if number < -MAX_PASS {
            -MAX_PASS
        } else {
            number
        })
    }

    /// Number.
    #[must_use]
    pub const fn number(self) -> i32 {
        self.0
    }

    /// Next pass to the left.
    #[must_use]
    pub const fn left(self) -> Self {
        Self::new(self.0.saturating_add(1))
    }

    /// Next pass to the right.
    #[must_use]
    pub const fn right(self) -> Self {
        Self::new(self.0.saturating_sub(1))
    }
}

impl fmt::Display for PassIndex {
    /// Formats as `pass +3`, `pass 0`, `pass -1`.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.0 > 0 {
            write!(f, "pass +{}", self.0)
        } else {
            write!(f, "pass {}", self.0)
        }
    }
}

/// Layout of parallel passes: spacing `width − overlap` and a shift of the
/// whole pattern.
///
/// Pass `k` lies at the signed offset `shift + k · spacing` from the reference
/// line, left positive. The shift moves every pass at once: set once to
/// place the pattern, or nudged during work to take out GNSS drift since the
/// line was recorded.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(try_from = "StoredPattern", into = "StoredPattern")
)]
pub struct PassPattern {
    spacing: f64,
    shift: f64,
}

impl PassPattern {
    /// Passes of a working width with an overlap between neighbours.
    ///
    /// # Errors
    ///
    /// [`kinavis_kernel::KernelError::OutOfRange`] for a width not above
    /// 1 cm or beyond 1 km, or an overlap below zero or not below the width by
    /// at least 1 cm.
    pub fn new(width: Distance, overlap: Distance) -> Result<Self> {
        let width = width.metres();
        ensure_range("working width, metres", width, MIN_SPACING, MAX_SPACING)?;
        ensure_range(
            "overlap between passes, metres",
            overlap.metres(),
            0.0,
            width - MIN_SPACING,
        )?;
        Ok(Self {
            spacing: width - overlap.metres(),
            shift: 0.0,
        })
    }

    /// Pattern shifted by `shift`, left positive.
    ///
    /// # Errors
    ///
    /// [`kinavis_kernel::KernelError::OutOfRange`] for a shift beyond 10⁶ m.
    pub fn with_shift(self, shift: Distance) -> Result<Self> {
        self.shifted(shift.metres())
    }

    fn shifted(self, shift: f64) -> Result<Self> {
        ensure_finite("pass shift", shift)?;
        ensure_range("pass shift, metres", shift, -MAX_SHIFT, MAX_SHIFT)?;
        Ok(Self { shift, ..self })
    }

    /// Pattern moved by `delta`, left positive, on top of its shift.
    ///
    /// # Errors
    ///
    /// As [`PassPattern::with_shift`] for the resulting shift.
    pub fn nudge(self, delta: Distance) -> Result<Self> {
        self.with_shift(Distance::from_metres(self.shift + delta.metres())?)
    }

    /// Distance between neighbouring passes.
    #[must_use]
    pub fn spacing(&self) -> Distance {
        Distance::from_metres(self.spacing).unwrap_or(Distance::ZERO)
    }

    /// Shift of the whole pattern, left positive.
    #[must_use]
    pub fn shift(&self) -> Distance {
        Distance::from_metres(self.shift).unwrap_or(Distance::ZERO)
    }

    /// Signed offset of a pass from the reference line, left positive.
    #[must_use]
    pub fn offset_of(&self, pass: PassIndex) -> Distance {
        Distance::from_metres(self.offset_metres(pass)).unwrap_or(Distance::ZERO)
    }

    /// Pass nearest to a signed offset from the reference line; a point
    /// exactly midway between two passes belongs to the one on the left.
    #[must_use]
    pub fn pass_at(&self, offset: Distance) -> PassIndex {
        self.pass_at_metres(offset.metres())
    }

    pub(crate) fn offset_metres(&self, pass: PassIndex) -> f64 {
        self.shift + f64::from(pass.number()) * self.spacing
    }

    pub(crate) fn pass_at_metres(&self, offset: f64) -> PassIndex {
        let k = ((offset - self.shift) / self.spacing + 0.5).floor_toward();
        let bound = f64::from(MAX_PASS);
        PassIndex::new(math::round_to_i32(k.clamp(-bound, bound)))
    }

    pub(crate) const fn spacing_metres(&self) -> f64 {
        self.spacing
    }
}

/// Smallest pass spacing, metres.
const MIN_SPACING: f64 = 0.01;

/// Largest working width, metres.
const MAX_SPACING: f64 = 1_000.0;

/// Largest pattern shift, metres.
const MAX_SHIFT: f64 = 1e6;

/// `floor` without the standard library.
trait FloorToward {
    fn floor_toward(self) -> f64;
}

impl FloorToward for f64 {
    fn floor_toward(self) -> f64 {
        let truncated = math::trunc(self);
        if truncated > self {
            truncated - 1.0
        } else {
            truncated
        }
    }
}

impl From<i32> for PassIndex {
    /// As [`PassIndex::new`]: clamped to ±[`MAX_PASS`].
    fn from(number: i32) -> Self {
        Self::new(number)
    }
}

impl From<PassIndex> for i32 {
    fn from(pass: PassIndex) -> Self {
        pass.0
    }
}

/// Serialised form, in metres: the spacing, not the width and overlap it
/// came from.
#[cfg(feature = "serde")]
#[derive(Clone, Copy, serde::Serialize, serde::Deserialize)]
struct StoredPattern {
    spacing_metres: f64,
    shift_metres: f64,
}

#[cfg(feature = "serde")]
impl TryFrom<StoredPattern> for PassPattern {
    type Error = crate::error::AgriError;

    /// The spacing within the widths [`PassPattern::new`] accepts, the shift
    /// as [`PassPattern::with_shift`] checks it.
    fn try_from(stored: StoredPattern) -> Result<Self> {
        let spacing = stored.spacing_metres;
        ensure_range("pass spacing, metres", spacing, MIN_SPACING, MAX_SPACING)?;
        Self {
            spacing,
            shift: 0.0,
        }
        .shifted(stored.shift_metres)
    }
}

#[cfg(feature = "serde")]
impl From<PassPattern> for StoredPattern {
    fn from(pattern: PassPattern) -> Self {
        Self {
            spacing_metres: pattern.spacing,
            shift_metres: pattern.shift,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn passes_change_midway() -> Result<()> {
        let pattern = PassPattern::new(Distance::from_metres(12.0)?, Distance::from_metres(0.5)?)?
            .with_shift(Distance::from_metres(1.0)?)?;
        assert_eq!(pattern.spacing().metres(), 11.5);
        assert_eq!(pattern.pass_at_metres(1.0), PassIndex::REFERENCE);
        assert_eq!(pattern.pass_at_metres(1.0 + 5.75), PassIndex::new(1));
        assert_eq!(
            pattern.pass_at_metres(1.0 + 5.75 - 1e-9),
            PassIndex::REFERENCE
        );
        assert_eq!(pattern.pass_at_metres(1.0 - 5.75), PassIndex::REFERENCE);
        assert_eq!(
            pattern.pass_at_metres(1.0 - 5.75 - 1e-9),
            PassIndex::new(-1)
        );
        assert_eq!(pattern.pass_at_metres(1e12), PassIndex::new(MAX_PASS));
        assert!((pattern.offset_metres(PassIndex::new(-3)) - (1.0 - 34.5)).abs() < 1e-12);
        let nudged = pattern.nudge(Distance::from_metres(-0.25)?)?;
        assert!((nudged.shift().metres() - 0.75).abs() < 1e-12);
        Ok(())
    }

    #[test]
    fn an_overlap_must_leave_a_spacing() -> Result<()> {
        let width = Distance::from_metres(6.0)?;
        assert!(PassPattern::new(width, Distance::from_metres(6.0)?).is_err());
        assert!(PassPattern::new(width, Distance::from_metres(-0.1)?).is_err());
        assert!(PassPattern::new(Distance::ZERO, Distance::ZERO).is_err());
        Ok(())
    }

    #[test]
    fn pass_numbers_print_with_their_side() {
        use std::string::ToString;

        assert_eq!(PassIndex::new(3).to_string(), "pass +3");
        assert_eq!(PassIndex::new(-1).to_string(), "pass -1");
        assert_eq!(PassIndex::new(i32::MAX), PassIndex::new(MAX_PASS));
    }
}
