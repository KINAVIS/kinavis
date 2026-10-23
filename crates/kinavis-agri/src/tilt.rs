//! Tilt compensation: the ground point under a tilted antenna.
//!
//! A GNSS antenna on a cab roof `h` above the ground leans with the machine:
//! at roll `φ` its position is `h · sin φ` to the side of the point the
//! machine steers by — 52 cm for a 3 m mast at 10°, against a 2 cm RTK fix.
//! [`ground_point`] takes the antenna's planar position, the heading and the
//! tilt and returns the control point.

use kinavis::region::PlanarPoint;
use kinavis_kernel::error::ensure_range;
use kinavis_kernel::{math, Angle, Distance, GridCourse};

use crate::error::Result;

/// Antenna position relative to the control point (the point the machine
/// steers by, usually on the ground under the rear axle), in body axes.
///
/// With the `serde` feature it is stored in metres.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(try_from = "StoredLeverArm", into = "StoredLeverArm")
)]
pub struct LeverArm {
    /// Ahead of the control point.
    pub forward: Distance,
    /// To the right of it.
    pub right: Distance,
    /// Above it.
    pub up: Distance,
}

impl LeverArm {
    /// Antenna straight above the control point.
    #[must_use]
    pub const fn above(height: Distance) -> Self {
        Self {
            forward: Distance::ZERO,
            right: Distance::ZERO,
            up: height,
        }
    }
}

/// Serialised form, in metres.
#[cfg(feature = "serde")]
#[derive(Clone, Copy, serde::Serialize, serde::Deserialize)]
// The unit suffix is part of the wire format.
#[allow(clippy::struct_field_names)]
struct StoredLeverArm {
    forward_metres: f64,
    right_metres: f64,
    up_metres: f64,
}

#[cfg(feature = "serde")]
impl TryFrom<StoredLeverArm> for LeverArm {
    type Error = crate::error::AgriError;

    fn try_from(stored: StoredLeverArm) -> Result<Self> {
        Ok(Self {
            forward: Distance::from_metres(stored.forward_metres)?,
            right: Distance::from_metres(stored.right_metres)?,
            up: Distance::from_metres(stored.up_metres)?,
        })
    }
}

#[cfg(feature = "serde")]
impl From<LeverArm> for StoredLeverArm {
    fn from(arm: LeverArm) -> Self {
        Self {
            forward_metres: arm.forward.metres(),
            right_metres: arm.right.metres(),
            up_metres: arm.up.metres(),
        }
    }
}

/// Roll and pitch of the machine, as from an inertial system or an
/// inclinometer.
///
/// Signs are those of the body frame (forward, right, down): roll positive
/// right side down, pitch positive nose up. From `kinavis-ins`, an
/// `Attitude`'s `roll` and `pitch` fields are these angles.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Tilt {
    roll: Angle,
    pitch: Angle,
}

impl Tilt {
    /// Level.
    pub const LEVEL: Self = Self {
        roll: Angle::ZERO,
        pitch: Angle::ZERO,
    };

    /// Tilt from roll and pitch.
    ///
    /// # Errors
    ///
    /// [`kinavis_kernel::KernelError::OutOfRange`] for an angle beyond ±60°:
    /// no machine works there, and a reading that large is a sensor fault.
    pub fn new(roll: Angle, pitch: Angle) -> Result<Self> {
        ensure_range("roll, degrees", roll.degrees(), -60.0, 60.0)?;
        ensure_range("pitch, degrees", pitch.degrees(), -60.0, 60.0)?;
        Ok(Self { roll, pitch })
    }

    /// Roll, right side down positive.
    #[must_use]
    pub const fn roll(&self) -> Angle {
        self.roll
    }

    /// Pitch, nose up positive.
    #[must_use]
    pub const fn pitch(&self) -> Angle {
        self.pitch
    }
}

/// Control point under an antenna: the antenna's planar position less the
/// lever arm turned by tilt and heading.
///
/// `heading` is the machine's grid heading (where its nose points, not its
/// track); a true heading becomes grid with
/// [`crate::FieldPlane::grid_course`].
///
/// # Errors
///
/// [`kinavis_kernel::KernelError::NotFinite`]; not reachable for validated
/// inputs.
pub fn ground_point(
    antenna: PlanarPoint,
    heading: GridCourse,
    tilt: Tilt,
    lever: LeverArm,
) -> Result<PlanarPoint> {
    let (sr, cr) = (
        math::sin(tilt.roll.radians()),
        math::cos(tilt.roll.radians()),
    );
    let (sp, cp) = (
        math::sin(tilt.pitch.radians()),
        math::cos(tilt.pitch.radians()),
    );
    // Lever arm in body axes (forward, right, down), rotated by roll then pitch
    // into the level frame aligned with the heading.
    let (x, y, z) = (
        lever.forward.metres(),
        lever.right.metres(),
        -lever.up.metres(),
    );
    let (y, z) = (y * cr - z * sr, y * sr + z * cr);
    let x = x * cp + z * sp;
    let (sh, ch) = (math::sin(heading.radians()), math::cos(heading.radians()));
    let north = x * ch - y * sh;
    let east = x * sh + y * ch;
    Ok(PlanarPoint::from_metres(
        antenna.east_metres() - east,
        antenna.north_metres() - north,
    )?)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_mast_rolled_to_the_right_leans_right() -> Result<()> {
        let antenna = PlanarPoint::ORIGIN;
        let mast = LeverArm::above(Distance::from_metres(3.0)?);
        let rolled = Tilt::new(Angle::from_degrees(10.0)?, Angle::ZERO)?;
        // Heading north: right is east, so the ground point is west of the antenna.
        let ground = ground_point(antenna, GridCourse::new(0.0)?, rolled, mast)?;
        assert!((ground.east_metres() + 3.0 * 10f64.to_radians().sin()).abs() < 1e-12);
        assert!(ground.north_metres().abs() < 1e-12);
        // Nose up: the antenna leans back, the ground point is ahead.
        let pitched = Tilt::new(Angle::ZERO, Angle::from_degrees(5.0)?)?;
        let ground = ground_point(antenna, GridCourse::new(90.0)?, pitched, mast)?;
        assert!((ground.east_metres() - 3.0 * 5f64.to_radians().sin()).abs() < 1e-12);
        Ok(())
    }

    #[test]
    fn level_ground_removes_only_the_horizontal_arm() -> Result<()> {
        let arm = LeverArm {
            forward: Distance::from_metres(1.5)?,
            right: Distance::from_metres(-0.5)?,
            up: Distance::from_metres(3.0)?,
        };
        let ground = ground_point(
            PlanarPoint::ORIGIN,
            GridCourse::new(180.0)?,
            Tilt::LEVEL,
            arm,
        )?;
        // Heading south: forward is south, right is west.
        assert!((ground.north_metres() - 1.5).abs() < 1e-12);
        assert!((ground.east_metres() + 0.5).abs() < 1e-12);
        assert!(Tilt::new(Angle::from_degrees(70.0)?, Angle::ZERO).is_err());
        Ok(())
    }
}
