//! Coverage: what the implement has worked, as a raster of the field plane,
//! and section control from it.
//!
//! A boom of `S` sections leaves a strip behind the machine. Each step from
//! one pose to the next marks, for each section that is on, the
//! quadrilateral its two ends swept; a cell is worked when its centre lies
//! inside. Edges shared by two strips are owned by one of them (the top-left
//! rule of raster graphics), so passes laid edge to edge leave no gap and
//! count no overlap.

use core::time::Duration;

use kinavis::region::{Containment, PlanarPoint};
use kinavis_kernel::error::ensure_range;
use kinavis_kernel::{math, Distance, KernelError, Speed};

use crate::error::Result;
use crate::field::Field;
use crate::line::metres;
use crate::plane::FieldPlane;
use crate::turn::Pose;

/// Boom of `S` sections, left to right, behind the control point.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(try_from = "StoredBoom", into = "StoredBoom")
)]
pub struct Boom<const S: usize> {
    /// Section widths, metres, left to right.
    widths: [f64; S],
    /// Distance of the boom behind the control point, metres.
    behind: f64,
    /// Offset of the boom's centre to the right of the control point, metres.
    right: f64,
}

impl<const S: usize> Boom<S> {
    /// Boom of the given section widths, left to right, `behind` the control
    /// point (negative for a front-mounted implement), centred on it.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] for a section narrower than 1 cm or wider
    /// than 100 m, or a boom more than 100 m from the control point;
    /// [`KernelError::InsufficientData`] for no sections.
    pub fn new(widths: [Distance; S], behind: Distance) -> Result<Self> {
        if S == 0 {
            return Err(KernelError::InsufficientData {
                found: 0,
                required: 1,
                context: "a boom",
            }
            .into());
        }
        Self::checked(widths.map(Distance::metres), behind.metres(), 0.0)
    }

    fn checked(widths: [f64; S], behind: f64, right: f64) -> Result<Self> {
        for width in widths {
            ensure_range("section width, metres", width, 0.01, 100.0)?;
        }
        ensure_range("boom distance behind, metres", behind, -100.0, 100.0)?;
        ensure_range("boom offset, metres", right, -100.0, 100.0)?;
        Ok(Self {
            widths,
            behind,
            right,
        })
    }

    /// The same boom with its centre `right` of the control point.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] beyond ±100 m.
    pub fn offset_right(self, right: Distance) -> Result<Self> {
        Self::checked(self.widths, self.behind, right.metres())
    }

    /// Working width.
    #[must_use]
    pub fn width(&self) -> Distance {
        metres(self.widths.iter().sum())
    }

    /// Left and right ends of each section for the machine at `pose`.
    fn ends(&self, pose: Pose) -> [(PlanarPoint, PlanarPoint); S] {
        let (north, east) = pose.course.components(1.0);
        // A quarter turn clockwise of the course.
        let (right_east, right_north) = (north, -east);
        let total: f64 = self.widths.iter().sum();
        let centre_east =
            pose.position.east_metres() - self.behind * east + self.right * right_east;
        let centre_north =
            pose.position.north_metres() - self.behind * north + self.right * right_north;
        let point = |across: f64| {
            PlanarPoint::from_metres(
                centre_east + across * right_east,
                centre_north + across * right_north,
            )
            .unwrap_or(PlanarPoint::ORIGIN)
        };
        let mut ends = [(PlanarPoint::ORIGIN, PlanarPoint::ORIGIN); S];
        let mut left = -total / 2.0;
        for (slot, width) in ends.iter_mut().zip(self.widths) {
            *slot = (point(left), point(left + width));
            left += width;
        }
        ends
    }
}

/// Area newly worked and worked again by one step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Marked {
    /// Area worked for the first time.
    pub new: f64,
    /// Area worked before and again now, square metres.
    pub overlap: f64,
}

/// Raster of what has been worked: `64 · W` cells east by `H` north, from
/// a south-west corner, in a field plane.
///
/// Two bits a cell — worked, worked more than once — so `16 · W · H` bytes:
/// a 500 m square at 25 cm (`W = 32`, `H = 2000`) is 1 MB.
#[derive(Debug, Clone, PartialEq)]
pub struct CoverageGrid<const W: usize, const H: usize> {
    plane: FieldPlane,
    corner: PlanarPoint,
    cell: f64,
    worked: [[u64; W]; H],
    again: [[u64; W]; H],
    worked_cells: usize,
    again_cells: usize,
}

impl<const W: usize, const H: usize> CoverageGrid<W, H> {
    /// Empty raster of cells of `cell` with its south-west corner at
    /// `corner`.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] for a cell outside 1 mm…100 m.
    pub fn new(plane: FieldPlane, corner: PlanarPoint, cell: Distance) -> Result<Self> {
        ensure_range("coverage cell, metres", cell.metres(), 1e-3, 100.0)?;
        Ok(Self {
            plane,
            corner,
            cell: cell.metres(),
            worked: [[0; W]; H],
            again: [[0; W]; H],
            worked_cells: 0,
            again_cells: 0,
        })
    }

    /// Plane.
    #[must_use]
    pub const fn plane(&self) -> &FieldPlane {
        &self.plane
    }

    /// Cell side.
    #[must_use]
    pub fn cell(&self) -> Distance {
        metres(self.cell)
    }

    /// Area worked, square metres.
    #[must_use]
    pub fn worked_area(&self) -> f64 {
        math::count_to_f64(self.worked_cells) * self.cell * self.cell
    }

    /// Area worked more than once, square metres.
    #[must_use]
    pub fn overlap_area(&self) -> f64 {
        math::count_to_f64(self.again_cells) * self.cell * self.cell
    }

    /// Whether the cell holding a point is worked; `false` off the raster.
    #[must_use]
    pub fn is_worked(&self, point: PlanarPoint) -> bool {
        self.index(point)
            .is_some_and(|(column, row)| bit(&self.worked, column, row))
    }

    /// Marks the strip the sections that are `on` swept from one pose to the
    /// next.
    pub fn mark<const S: usize>(
        &mut self,
        boom: &Boom<S>,
        from: Pose,
        to: Pose,
        on: [bool; S],
    ) -> Marked {
        let before = (self.worked_cells, self.again_cells);
        let start = boom.ends(from);
        let end = boom.ends(to);
        for (((left_a, right_a), (left_b, right_b)), on) in start.into_iter().zip(end).zip(on) {
            if on {
                self.fill_triangle(left_a, right_a, right_b);
                self.fill_triangle(left_a, right_b, left_b);
            }
        }
        let area = self.cell * self.cell;
        Marked {
            new: math::count_to_f64(self.worked_cells.saturating_sub(before.0)) * area,
            overlap: math::count_to_f64(self.again_cells.saturating_sub(before.1)) * area,
        }
    }

    /// Fraction of a segment's sample points already worked or outside the
    /// field.
    fn done_fraction<const N: usize, const HH: usize, const K: usize>(
        &self,
        field: Option<&Field<N, HH, K>>,
        left: PlanarPoint,
        right: PlanarPoint,
    ) -> f64 {
        const SAMPLES: usize = 8;
        let mut done = 0usize;
        for index in 0..SAMPLES {
            let t = (math::count_to_f64(index) + 0.5) / math::count_to_f64(SAMPLES);
            let point = PlanarPoint::from_metres(
                left.east_metres() + t * (right.east_metres() - left.east_metres()),
                left.north_metres() + t * (right.north_metres() - left.north_metres()),
            )
            .unwrap_or(left);
            let outside =
                field.is_some_and(|field| field.region().contains(point) != Containment::Inside);
            if outside || self.is_worked(point) {
                done = done.saturating_add(1);
            }
        }
        math::count_to_f64(done) / math::count_to_f64(SAMPLES)
    }

    fn index(&self, point: PlanarPoint) -> Option<(usize, usize)> {
        let x = (point.east_metres() - self.corner.east_metres()) / self.cell;
        let y = (point.north_metres() - self.corner.north_metres()) / self.cell;
        let columns = math::count_to_f64(W.saturating_mul(64));
        let rows = math::count_to_f64(H);
        if !(0.0..columns).contains(&x) || !(0.0..rows).contains(&y) {
            return None;
        }
        Some((math::to_usize(x), math::to_usize(y)))
    }

    /// Marks the cells whose centres lie in a triangle, with the top-left
    /// rule on its edges.
    #[allow(clippy::many_single_char_names)]
    fn fill_triangle(&mut self, a: PlanarPoint, b: PlanarPoint, c: PlanarPoint) {
        // Vertices on a lattice of 2⁻²⁰ cell, so that the ends of two strips
        // computed along different paths, a few ulps apart, are one point and
        // their shared edge is shared exactly.
        let snap = |value: f64| math::round(value * LATTICE) / LATTICE;
        let to_cells = |p: PlanarPoint| {
            (
                snap((p.east_metres() - self.corner.east_metres()) / self.cell),
                snap((p.north_metres() - self.corner.north_metres()) / self.cell),
            )
        };
        let (mut a, mut b, c) = (to_cells(a), to_cells(b), to_cells(c));
        let area = cross(a, b, c);
        if area == 0.0 || !area.is_finite() {
            return;
        }
        if area < 0.0 {
            core::mem::swap(&mut a, &mut b);
        }
        let columns = math::count_to_f64(W.saturating_mul(64));
        let rows = math::count_to_f64(H);
        // Cells whose centres i + ½ lie between the least and the greatest
        // coordinate, within the raster.
        let low = |u: f64, v: f64, w: f64, limit: f64| {
            math::to_usize(math::ceil(u.min(v).min(w) - 0.5).max(0.0).min(limit))
        };
        let high = |u: f64, v: f64, w: f64, limit: f64| {
            math::to_usize((floor(u.max(v).max(w) - 0.5) + 1.0).max(0.0).min(limit))
        };
        let (x0, x1) = (low(a.0, b.0, c.0, columns), high(a.0, b.0, c.0, columns));
        let (y0, y1) = (low(a.1, b.1, c.1, rows), high(a.1, b.1, c.1, rows));
        for row in y0..y1 {
            let y = math::count_to_f64(row) + 0.5;
            for column in x0..x1 {
                let p = (math::count_to_f64(column) + 0.5, y);
                if owns(a, b, p) && owns(b, c, p) && owns(c, a, p) {
                    self.set(column, row);
                }
            }
        }
    }

    fn set(&mut self, column: usize, row: usize) {
        let (word, mask) = (column / 64, 1u64 << (column % 64));
        let Some(slot) = self.worked.get_mut(row).and_then(|r| r.get_mut(word)) else {
            return;
        };
        if *slot & mask == 0 {
            *slot |= mask;
            self.worked_cells = self.worked_cells.saturating_add(1);
            return;
        }
        let Some(again) = self.again.get_mut(row).and_then(|r| r.get_mut(word)) else {
            return;
        };
        if *again & mask == 0 {
            *again |= mask;
            self.again_cells = self.again_cells.saturating_add(1);
        }
    }
}

/// `floor` from the kernel's `trunc`.
fn floor(x: f64) -> f64 {
    let truncated = math::trunc(x);
    if truncated > x {
        truncated - 1.0
    } else {
        truncated
    }
}

fn bit<const W: usize, const H: usize>(bits: &[[u64; W]; H], column: usize, row: usize) -> bool {
    bits.get(row)
        .and_then(|r| r.get(column / 64))
        .is_some_and(|word| word >> (column % 64) & 1 != 0)
}

/// Twice the signed area of `a`, `b`, `c`.
fn cross(a: (f64, f64), b: (f64, f64), c: (f64, f64)) -> f64 {
    (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0)
}

/// Vertex lattice, per cell.
const LATTICE: f64 = 1_048_576.0;

/// Whether `p` is on the inner side of the counter-clockwise edge `a`→`b`,
/// points on the edge belonging to it only for a top or left edge.
///
/// The side is computed from the endpoints in a fixed order, so the two
/// triangles sharing an edge, which run it in opposite directions, get
/// exactly opposite values and one of them owns each point on it.
fn owns(a: (f64, f64), b: (f64, f64), p: (f64, f64)) -> bool {
    let side = if a <= b {
        cross(a, b, p)
    } else {
        -cross(b, a, p)
    };
    if side != 0.0 {
        return side > 0.0;
    }
    // On the edge: counter-clockwise, a left edge runs downwards, a top edge
    // westwards.
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    dy < 0.0 || (dy == 0.0 && dx < 0.0)
}

/// Section control: delays of the valves and the share of a section's
/// footprint already worked or outside the field that switches it off.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SectionControl {
    on_delay: f64,
    off_delay: f64,
    threshold: f64,
}

impl SectionControl {
    /// Control with the time a section takes to start and to stop applying
    /// once commanded, and the share of its footprint, ahead by that time,
    /// worked or outside the field at which it is switched off (0 for no
    /// overlap tolerated, 1 for no gap).
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] for a delay beyond 30 s or a share outside
    /// 0…1.
    pub fn new(on_delay: Duration, off_delay: Duration, threshold: f64) -> Result<Self> {
        let (on, off) = (on_delay.as_secs_f64(), off_delay.as_secs_f64());
        ensure_range("section on delay, seconds", on, 0.0, 30.0)?;
        ensure_range("section off delay, seconds", off, 0.0, 30.0)?;
        ensure_range("section switching share", threshold, 0.0, 1.0)?;
        Ok(Self {
            on_delay: on,
            off_delay: off,
            threshold,
        })
    }

    /// Commands for each section: on where the ground the section will
    /// reach once its valve has acted is mostly unworked and inside the
    /// field.
    ///
    /// A section now off is judged where it will be after the on delay; one
    /// now on, after the off delay.
    ///
    /// # Errors
    ///
    /// [`crate::AgriError::PlaneMismatch`] for a grid or field in another
    /// plane.
    pub fn commands<
        const S: usize,
        const W: usize,
        const H: usize,
        const N: usize,
        const HH: usize,
        const K: usize,
    >(
        &self,
        grid: &CoverageGrid<W, H>,
        field: Option<&Field<N, HH, K>>,
        boom: &Boom<S>,
        pose: Pose,
        speed: Speed,
        current: [bool; S],
    ) -> Result<[bool; S]> {
        if let Some(field) = field {
            grid.plane.ensure_same(field.plane())?;
        }
        let ahead = |seconds: f64| {
            let run = speed.metres_per_second().max(0.0) * seconds;
            let (north, east) = pose.course.components(run);
            boom.ends(Pose {
                position: PlanarPoint::from_metres(
                    pose.position.east_metres() + east,
                    pose.position.north_metres() + north,
                )
                .unwrap_or(pose.position),
                course: pose.course,
            })
        };
        let when_on = ahead(self.on_delay);
        let when_off = ahead(self.off_delay);
        let mut commands = [false; S];
        for (index, command) in commands.iter_mut().enumerate() {
            let is_on = current.get(index).copied().unwrap_or(false);
            let ends = if is_on {
                when_off.get(index)
            } else {
                when_on.get(index)
            };
            let Some((left, right)) = ends else {
                continue;
            };
            let done = grid.done_fraction(field, *left, *right);
            // Strictly below the share: at a threshold of 0 any worked ground
            // ahead stops the section.
            *command = if self.threshold == 0.0 {
                done == 0.0
            } else {
                done < self.threshold
            };
        }
        Ok(commands)
    }
}

/// Serialised form, in metres: the section widths as a list.
#[cfg(feature = "serde")]
#[derive(serde::Serialize, serde::Deserialize)]
// The unit suffix is part of the wire format.
#[allow(clippy::struct_field_names)]
struct StoredBoom {
    widths_metres: alloc::vec::Vec<f64>,
    behind_metres: f64,
    right_metres: f64,
}

#[cfg(feature = "serde")]
impl<const S: usize> TryFrom<StoredBoom> for Boom<S> {
    type Error = crate::error::AgriError;

    /// Exactly `S` widths, checked as [`Boom::new`] and [`Boom::offset_right`]
    /// check.
    fn try_from(stored: StoredBoom) -> Result<Self> {
        let found = stored.widths_metres.len();
        if found < S {
            return Err(KernelError::InsufficientData {
                found,
                required: S,
                context: "the sections of a boom",
            }
            .into());
        }
        if found > S {
            return Err(KernelError::CapacityExceeded {
                context: "the sections of a boom",
                needed: found,
                capacity: S,
            }
            .into());
        }
        let mut widths = [0.0; S];
        for (slot, width) in widths.iter_mut().zip(stored.widths_metres) {
            *slot = width;
        }
        Self::checked(widths, stored.behind_metres, stored.right_metres)
    }
}

#[cfg(feature = "serde")]
impl<const S: usize> From<Boom<S>> for StoredBoom {
    fn from(boom: Boom<S>) -> Self {
        Self {
            widths_metres: boom.widths.to_vec(),
            behind_metres: boom.behind,
            right_metres: boom.right,
        }
    }
}
