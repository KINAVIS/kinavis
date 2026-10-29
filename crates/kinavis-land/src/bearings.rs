//! Grid, true and magnetic bearings at a point, and angles in mils.
//!
//! A map gives bearings from grid north (the *grid bearing*, дирекционный
//! угол), a compass from magnetic north. Between them lie the meridian
//! convergence γ (true to grid north, from the grid) and the magnetic
//! declination δ (true to magnetic north, from a magnetic model): grid =
//! true − γ, magnetic = true − δ, so magnetic = grid − (δ − γ). The angle
//! δ − γ is the *grid magnetic angle* printed in a map's margin (поправка
//! направления).

use kinavis_kernel::error::ensure_range;
use kinavis_kernel::{
    wrap360, Angle, Direction, GridBearing, MagneticBearing, Position, TrueBearing, Variation,
};

use crate::error::Result;
use crate::grid::MapGrid;

/// Convergence and declination at one point: the conversions between grid,
/// true and magnetic bearings there.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BearingFrame {
    convergence: Angle,
    declination: Variation,
}

impl BearingFrame {
    /// Frame from the convergence (true to grid north, clockwise positive)
    /// and the declination (true to magnetic north, east positive).
    #[must_use]
    pub const fn new(convergence: Angle, declination: Variation) -> Self {
        Self {
            convergence,
            declination,
        }
    }

    /// Frame at a position: the convergence of `grid` there and the
    /// declination given, as `kinavis-wmm` computes it.
    ///
    /// # Errors
    ///
    /// As [`MapGrid::project`].
    pub fn at(grid: &MapGrid, position: Position, declination: Variation) -> Result<Self> {
        Ok(Self::new(grid.project(position)?.convergence, declination))
    }

    /// Convergence.
    #[must_use]
    pub const fn convergence(&self) -> Angle {
        self.convergence
    }

    /// Declination.
    #[must_use]
    pub const fn declination(&self) -> Variation {
        self.declination
    }

    /// Grid magnetic angle, `δ − γ` within ±180°: a magnetic bearing plus it
    /// is the grid bearing.
    #[must_use]
    pub fn grid_magnetic_angle(&self) -> Angle {
        Angle::from_degrees_unchecked(kinavis_kernel::wrap180(
            self.declination.degrees() - self.convergence.degrees(),
        ))
    }

    /// Magnetic bearing of a grid bearing.
    #[must_use]
    pub fn grid_to_magnetic(&self, bearing: GridBearing) -> MagneticBearing {
        Direction::from_degrees_wrapped(bearing.degrees() - self.grid_magnetic_angle().degrees())
    }

    /// Grid bearing of a magnetic bearing.
    #[must_use]
    pub fn magnetic_to_grid(&self, bearing: MagneticBearing) -> GridBearing {
        Direction::from_degrees_wrapped(bearing.degrees() + self.grid_magnetic_angle().degrees())
    }

    /// Magnetic bearing of a true bearing.
    #[must_use]
    pub fn true_to_magnetic(&self, bearing: TrueBearing) -> MagneticBearing {
        Direction::from_degrees_wrapped(bearing.degrees() - self.declination.degrees())
    }

    /// True bearing of a magnetic bearing.
    #[must_use]
    pub fn magnetic_to_true(&self, bearing: MagneticBearing) -> TrueBearing {
        Direction::from_degrees_wrapped(bearing.degrees() + self.declination.degrees())
    }

    /// Grid bearing of a true bearing.
    #[must_use]
    pub fn true_to_grid(&self, bearing: TrueBearing) -> GridBearing {
        bearing.to_grid(self.convergence)
    }

    /// True bearing of a grid bearing.
    #[must_use]
    pub fn grid_to_true(&self, bearing: GridBearing) -> TrueBearing {
        bearing.to_true(self.convergence)
    }
}

/// How many mils make a full circle.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Mils {
    /// 6000, the угломер divisions of Soviet and Russian maps and artillery:
    /// a bearing of 45-30 is 4530 of them.
    Soviet,
    /// 6400, NATO.
    Nato,
}

impl Mils {
    /// Mils in a full circle.
    #[must_use]
    pub const fn per_circle(self) -> f64 {
        match self {
            Self::Soviet => 6000.0,
            Self::Nato => 6400.0,
        }
    }

    /// An angle in mils.
    #[must_use]
    pub fn of(self, angle: Angle) -> f64 {
        angle.degrees() * self.per_circle() / 360.0
    }

    /// A direction in mils, from 0 to a full circle.
    #[must_use]
    pub fn of_direction<F: kinavis_kernel::Frame>(self, direction: Direction<F>) -> f64 {
        direction.degrees() * self.per_circle() / 360.0
    }

    /// Angle of a number of mils.
    ///
    /// # Errors
    ///
    /// [`kinavis_kernel::KernelError::OutOfRange`] beyond ±10 full circles,
    /// or a non-finite value.
    pub fn angle(self, mils: f64) -> Result<Angle> {
        let circle = self.per_circle();
        ensure_range("mils", mils, -10.0 * circle, 10.0 * circle)?;
        Ok(Angle::from_degrees(mils * 360.0 / circle)?)
    }

    /// Direction of a number of mils, wrapped into a full circle.
    ///
    /// # Errors
    ///
    /// As [`Mils::angle`].
    pub fn direction<F: kinavis_kernel::Frame>(self, mils: f64) -> Result<Direction<F>> {
        let degrees = self.angle(mils)?.degrees();
        Ok(Direction::from_degrees_wrapped(wrap360(degrees)))
    }
}
