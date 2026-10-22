//! Turns at the end of a pass: the shortest path of bounded curvature from
//! one pose to another (Dubins), followed as a reference line, checked
//! against the field.
//!
//! A turn from the end of one pass onto the next is three pieces, each an arc
//! of the turning radius or a straight: left–straight–left and its kin when
//! the passes are far enough apart for a U-turn, right–left–right (the omega,
//! or bulb, turn) when the spacing is under twice the radius. Turns that
//! reverse (fishtail) are [`ReedsSheppPath`](crate::ReedsSheppPath).

use kinavis::region::{Containment, PlanarPoint};
use kinavis_kernel::error::ensure_range;
use kinavis_kernel::{math, Distance, GridCourse, KernelError};

use crate::error::Result;
use crate::field::Field;
use crate::line::{course_of, metres, ReferenceLine, Station};
use crate::pass::{PassIndex, PassPattern};
use crate::plane::FieldPlane;
use crate::steering::project;

const TAU: f64 = core::f64::consts::TAU;

/// Position and grid course in a field plane.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pose {
    /// Position of the control point.
    pub position: PlanarPoint,
    /// Grid course.
    pub course: GridCourse,
}

/// Direction of one piece of a turn.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Steer {
    /// Arc turning left, counter-clockwise.
    Left,
    /// Straight.
    Straight,
    /// Arc turning right, clockwise.
    Right,
}

/// The six Dubins words, by their pieces.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum DubinsWord {
    /// Left, straight, left.
    Lsl,
    /// Left, straight, right.
    Lsr,
    /// Right, straight, left.
    Rsl,
    /// Right, straight, right.
    Rsr,
    /// Right, left, right.
    Rlr,
    /// Left, right, left.
    Lrl,
}

impl DubinsWord {
    /// All six.
    pub const ALL: [Self; 6] = [
        Self::Lsl,
        Self::Lsr,
        Self::Rsl,
        Self::Rsr,
        Self::Rlr,
        Self::Lrl,
    ];

    /// Pieces of the word.
    #[must_use]
    pub const fn steers(self) -> [Steer; 3] {
        use Steer::{Left, Right, Straight};
        match self {
            Self::Lsl => [Left, Straight, Left],
            Self::Lsr => [Left, Straight, Right],
            Self::Rsl => [Right, Straight, Left],
            Self::Rsr => [Right, Straight, Right],
            Self::Rlr => [Right, Left, Right],
            Self::Lrl => [Left, Right, Left],
        }
    }
}

/// One piece of a turn.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TurnPiece {
    /// Direction.
    pub steer: Steer,
    /// Length along the path.
    pub length: Distance,
}

/// Shortest path of bounded curvature between two poses: three pieces of a
/// Dubins word.
///
/// As a [`ReferenceLine`] the path is followed by the trackers like a pass;
/// beyond its ends it runs on straight along the end courses.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TurnPath {
    plane: FieldPlane,
    start: Pose,
    radius: f64,
    word: DubinsWord,
    /// Piece lengths in units of the radius (arcs in radians).
    lengths: [f64; 3],
}

impl TurnPath {
    /// Shortest path from one pose to another with turns no tighter than
    /// `radius`.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] for a radius outside 0.5…1000 m;
    /// [`KernelError::Indeterminate`] if no word connects the poses (not
    /// reachable for finite input: LSL and RSR always exist).
    pub fn dubins(plane: FieldPlane, from: Pose, to: Pose, radius: Distance) -> Result<Self> {
        let radius = radius.metres();
        ensure_range("turning radius, metres", radius, 0.5, 1_000.0)?;
        let problem = Problem::new(from, to, radius);
        let mut best: Option<(DubinsWord, [f64; 3])> = None;
        for word in DubinsWord::ALL {
            if let Some(lengths) = problem.solve(word) {
                let total: f64 = lengths.iter().sum();
                if best.is_none_or(|(_, b)| total < b.iter().sum::<f64>()) {
                    best = Some((word, lengths));
                }
            }
        }
        let (word, lengths) = best.ok_or(KernelError::Indeterminate {
            quantity: "a Dubins path between the poses",
        })?;
        Ok(Self {
            plane,
            start: from,
            radius,
            word,
            lengths,
        })
    }

    /// The path of one word, if that word connects the poses.
    ///
    /// # Errors
    ///
    /// As [`TurnPath::dubins`].
    pub fn with_word(
        plane: FieldPlane,
        from: Pose,
        to: Pose,
        radius: Distance,
        word: DubinsWord,
    ) -> Result<Option<Self>> {
        let radius = radius.metres();
        ensure_range("turning radius, metres", radius, 0.5, 1_000.0)?;
        Ok(Problem::new(from, to, radius)
            .solve(word)
            .map(|lengths| Self {
                plane,
                start: from,
                radius,
                word,
                lengths,
            }))
    }

    /// Turn from a pose at the end of a pass onto another pass of the same
    /// line, joining it abreast of the exit point and heading back.
    ///
    /// # Errors
    ///
    /// As [`TurnPath::dubins`].
    pub fn to_pass<L: ReferenceLine>(
        line: &L,
        pattern: &PassPattern,
        exit: Pose,
        to: PassIndex,
        radius: Distance,
    ) -> Result<Self> {
        Self::dubins(
            *line.plane(),
            exit,
            pass_entry(line, pattern, exit, to),
            radius,
        )
    }

    /// Word.
    #[must_use]
    pub const fn word(&self) -> DubinsWord {
        self.word
    }

    /// Turning radius.
    #[must_use]
    pub fn radius(&self) -> Distance {
        metres(self.radius)
    }

    /// Start pose.
    #[must_use]
    pub const fn start(&self) -> Pose {
        self.start
    }

    /// The three pieces.
    #[must_use]
    pub fn pieces(&self) -> [TurnPiece; 3] {
        let steers = self.word.steers();
        let mut pieces = [TurnPiece {
            steer: Steer::Straight,
            length: Distance::ZERO,
        }; 3];
        for ((piece, steer), length) in pieces.iter_mut().zip(steers).zip(self.lengths) {
            *piece = TurnPiece {
                steer,
                length: metres(length * self.radius),
            };
        }
        pieces
    }

    /// Length.
    #[must_use]
    pub fn length(&self) -> Distance {
        metres(self.lengths.iter().sum::<f64>() * self.radius)
    }

    /// Pose at a distance along the path, clamped to its ends.
    #[must_use]
    pub fn pose_at(&self, along: Distance) -> Pose {
        let mut left = along.metres().max(0.0) / self.radius;
        let mut state = MathPose::of(self.start, self.radius);
        for (steer, length) in self.word.steers().into_iter().zip(self.lengths) {
            let run = left.min(length);
            state = state.advance(steer, run);
            left -= run;
            if left <= 0.0 {
                break;
            }
        }
        state.pose(self.radius)
    }

    /// End pose.
    #[must_use]
    pub fn end(&self) -> Pose {
        self.pose_at(self.length())
    }

    /// First distance along the path at which a machine of the given half
    /// width would leave the field or touch an obstacle, checked every
    /// `step`; `None` if it stays clear throughout.
    ///
    /// Between two checked points the path is within `step / 2` of one, so
    /// a clearance of `half_width + step / 2` is required at each: a turn
    /// reported clear is clear.
    ///
    /// # Errors
    ///
    /// [`crate::AgriError::PlaneMismatch`] for a field in another plane;
    /// [`KernelError::OutOfRange`] for a step outside 1 cm…10 m or a negative
    /// half width.
    pub fn first_conflict<const N: usize, const H: usize, const K: usize>(
        &self,
        field: &Field<N, H, K>,
        half_width: Distance,
        step: Distance,
    ) -> Result<Option<Distance>> {
        self.plane.ensure_same(field.plane())?;
        first_conflict(field, half_width, step, self.length(), |along| {
            self.pose_at(along).position
        })
    }
}

/// Pose that joins pass `to` abreast of the exit point, heading back.
pub(crate) fn pass_entry<L: ReferenceLine>(
    line: &L,
    pattern: &PassPattern,
    exit: Pose,
    to: PassIndex,
) -> Pose {
    let entry = project(line, pattern.offset_metres(to), exit.position);
    let course = line.station(entry).direction;
    // Heading back: against the exit course.
    let course = if course.angular_distance(exit.course) <= 90.0 {
        course.reciprocal()
    } else {
        course
    };
    Pose {
        position: entry,
        course,
    }
}

/// First checked distance along a path of length `total` at which the point
/// `at(along)` is closer than `half_width + step / 2` to the field's edge or
/// outside it.
pub(crate) fn first_conflict<const N: usize, const H: usize, const K: usize>(
    field: &Field<N, H, K>,
    half_width: Distance,
    step: Distance,
    total: Distance,
    at: impl Fn(Distance) -> PlanarPoint,
) -> Result<Option<Distance>> {
    let step = step.metres();
    ensure_range("check step, metres", step, 0.01, 10.0)?;
    ensure_range("half width, metres", half_width.metres(), 0.0, 1e3)?;
    let needed = half_width.metres() + step / 2.0;
    let total = total.metres();
    let count = math::to_usize(math::ceil(total / step));
    let region = field.region();
    for index in 0..=count {
        let along = (math::count_to_f64(index) * step).min(total);
        let point = at(metres(along));
        let clear = region.contains(point) == Containment::Inside
            && region.distance_to_boundary(point).metres() >= needed;
        if !clear {
            return Ok(Some(metres(along)));
        }
    }
    Ok(None)
}

impl ReferenceLine for TurnPath {
    fn plane(&self) -> &FieldPlane {
        &self.plane
    }

    /// Nearest point of the three pieces, or of the straight run-ons beyond
    /// the ends.
    fn station(&self, point: PlanarPoint) -> Station {
        let start = MathPose::of(self.start, self.radius);
        let pieces = self.word.steers().into_iter().zip(self.lengths);
        path_foot(start, pieces, point, self.radius).station(self.radius)
    }
}

/// Pose in the normalised mathematical frame: x east, y north, in units of
/// the radius, heading counter-clockwise from east.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct MathPose {
    pub(crate) x: f64,
    pub(crate) y: f64,
    pub(crate) heading: f64,
}

impl MathPose {
    pub(crate) fn of(pose: Pose, radius: f64) -> Self {
        Self {
            x: pose.position.east_metres() / radius,
            y: pose.position.north_metres() / radius,
            heading: math::to_radians(90.0 - pose.course.degrees()),
        }
    }

    pub(crate) fn pose(self, radius: f64) -> Pose {
        Pose {
            position: PlanarPoint::from_metres(self.x * radius, self.y * radius)
                .unwrap_or(PlanarPoint::ORIGIN),
            course: course_of(math::cos(self.heading), math::sin(self.heading)),
        }
    }

    /// Pose after `length` (negative in reverse) of a piece.
    pub(crate) fn advance(self, steer: Steer, length: f64) -> Self {
        let (s, c) = (math::sin(self.heading), math::cos(self.heading));
        match steer {
            Steer::Straight => Self {
                x: self.x + length * c,
                y: self.y + length * s,
                ..self
            },
            Steer::Left => {
                let heading = self.heading + length;
                Self {
                    x: self.x + math::sin(heading) - s,
                    y: self.y - math::cos(heading) + c,
                    heading,
                }
            }
            Steer::Right => {
                let heading = self.heading - length;
                Self {
                    x: self.x - math::sin(heading) + s,
                    y: self.y + math::cos(heading) - c,
                    heading,
                }
            }
        }
    }
}

/// Nearest point of a path, in units of the radius.
#[derive(Debug, Clone, Copy)]
pub(crate) struct Foot {
    /// Distance from the point.
    distance: f64,
    /// Signed offset of the point, left positive.
    offset: f64,
    /// Heading of the path at the foot, counter-clockwise from east.
    heading: f64,
    /// Distance along the path, negative on the run-in.
    pub(crate) along: f64,
}

impl Foot {
    pub(crate) fn station(self, radius: f64) -> Station {
        Station {
            offset: metres(self.offset * radius),
            direction: course_of(math::cos(self.heading), math::sin(self.heading)),
        }
    }
}

/// Nearest point to `point` of the pieces driven forward from `start`, or of
/// the straight run-ons beyond the ends.
pub(crate) fn path_foot(
    start: MathPose,
    pieces: impl Iterator<Item = (Steer, f64)>,
    point: PlanarPoint,
    radius: f64,
) -> Foot {
    let target = (point.east_metres() / radius, point.north_metres() / radius);
    let mut state = start;
    // Run-in: the start course extended backwards.
    let mut best = straight_foot(state, f64::NEG_INFINITY, 0.0, target);
    let mut along = 0.0;
    for (steer, length) in pieces {
        let mut candidate = match steer {
            Steer::Straight => straight_foot(state, 0.0, length, target),
            Steer::Left | Steer::Right => arc_foot(state, steer, length, target),
        };
        candidate.along += along;
        if candidate.distance < best.distance {
            best = candidate;
        }
        state = state.advance(steer, length);
        along += length;
    }
    let mut run_out = straight_foot(state, 0.0, f64::INFINITY, target);
    run_out.along += along;
    if run_out.distance < best.distance {
        best = run_out;
    }
    best
}

/// Foot on a straight of parameter range `from..to` starting at `pose`.
fn straight_foot(pose: MathPose, from: f64, to: f64, target: (f64, f64)) -> Foot {
    let (s, c) = (math::sin(pose.heading), math::cos(pose.heading));
    let (dx, dy) = (target.0 - pose.x, target.1 - pose.y);
    let along = (dx * c + dy * s).max(from).min(to);
    let (fx, fy) = (dx - along * c, dy - along * s);
    let distance = math::hypot(fx, fy);
    let side = c * dy - s * dx;
    Foot {
        distance,
        offset: if side < 0.0 { -distance } else { distance },
        heading: pose.heading,
        along,
    }
}

/// As [`straight_foot`] for an arc of unit radius and the given swept angle.
fn arc_foot(pose: MathPose, steer: Steer, sweep: f64, target: (f64, f64)) -> Foot {
    let sign = if steer == Steer::Left { 1.0 } else { -1.0 };
    let (s, c) = (math::sin(pose.heading), math::cos(pose.heading));
    // Centre on the inside of the turn.
    let (cx, cy) = (pose.x - sign * s, pose.y + sign * c);
    let (dx, dy) = (target.0 - cx, target.1 - cy);
    let r = math::hypot(dx, dy);
    // Angle swept from the start radius to the target, in the turn's sense.
    let start = math::atan2(pose.y - cy, pose.x - cx);
    let angle = math::atan2(dy, dx);
    let turned = (sign * (angle - start)).wrap_tau();
    if turned <= sweep {
        // Left arc: the centre is left, inside is left.
        return Foot {
            distance: math::abs(r - 1.0),
            offset: sign * (1.0 - r),
            heading: pose.heading + sign * turned,
            along: turned,
        };
    }
    // Nearer end of the arc.
    let to_start = straight_foot(pose, 0.0, 0.0, target);
    let mut to_end = straight_foot(pose.advance(steer, sweep), 0.0, 0.0, target);
    to_end.along = sweep;
    if to_start.distance <= to_end.distance {
        to_start
    } else {
        to_end
    }
}

/// Angle reduced to `[0, 2π)`.
pub(crate) trait WrapTau {
    fn wrap_tau(self) -> f64;
}

impl WrapTau for f64 {
    fn wrap_tau(self) -> f64 {
        let turns = math::trunc(self / TAU);
        let mut wrapped = self - turns * TAU;
        if wrapped < 0.0 {
            wrapped += TAU;
        }
        if wrapped >= TAU {
            wrapped -= TAU;
        }
        wrapped
    }
}

/// Normalised Dubins problem (Shkel and Lumelsky): distance `d` in radii,
/// start and end headings `α`, `β` relative to the line joining them.
struct Problem {
    d: f64,
    alpha: f64,
    beta: f64,
}

impl Problem {
    fn new(from: Pose, to: Pose, radius: f64) -> Self {
        let start = MathPose::of(from, radius);
        let end = MathPose::of(to, radius);
        let (dx, dy) = (end.x - start.x, end.y - start.y);
        let d = math::hypot(dx, dy);
        let theta = if d > 0.0 {
            math::atan2(dy, dx).wrap_tau()
        } else {
            0.0
        };
        Self {
            d,
            alpha: (start.heading - theta).wrap_tau(),
            beta: (end.heading - theta).wrap_tau(),
        }
    }

    /// Lengths of the three pieces of `word`, in radii, if it exists.
    fn solve(&self, word: DubinsWord) -> Option<[f64; 3]> {
        let Self { d, alpha, beta } = *self;
        let (sa, sb) = (math::sin(alpha), math::sin(beta));
        let (ca, cb) = (math::cos(alpha), math::cos(beta));
        let cab = math::cos(alpha - beta);
        let wrap = |x: f64| x.wrap_tau();
        let root = |p2: f64| (p2 >= 0.0).then(|| math::sqrt(p2));
        match word {
            DubinsWord::Lsl => {
                let p = root(2.0 + d * d - 2.0 * cab + 2.0 * d * (sa - sb))?;
                let tmp = math::atan2(cb - ca, d + sa - sb);
                Some([wrap(tmp - alpha), p, wrap(beta - tmp)])
            }
            DubinsWord::Rsr => {
                let p = root(2.0 + d * d - 2.0 * cab + 2.0 * d * (sb - sa))?;
                let tmp = math::atan2(ca - cb, d - sa + sb);
                Some([wrap(alpha - tmp), p, wrap(tmp - beta)])
            }
            DubinsWord::Lsr => {
                let p = root(-2.0 + d * d + 2.0 * cab + 2.0 * d * (sa + sb))?;
                let tmp = math::atan2(-ca - cb, d + sa + sb) - math::atan2(-2.0, p);
                Some([wrap(tmp - alpha), p, wrap(tmp - beta)])
            }
            DubinsWord::Rsl => {
                let p = root(-2.0 + d * d + 2.0 * cab - 2.0 * d * (sa + sb))?;
                let tmp = math::atan2(ca + cb, d - sa - sb) - math::atan2(2.0, p);
                Some([wrap(alpha - tmp), p, wrap(beta - tmp)])
            }
            DubinsWord::Rlr => {
                let cosine = (6.0 - d * d + 2.0 * cab + 2.0 * d * (sa - sb)) / 8.0;
                if math::abs(cosine) > 1.0 {
                    return None;
                }
                let phi = math::atan2(ca - cb, d - sa + sb);
                let p = wrap(TAU - math::acos(cosine));
                let t = wrap(alpha - phi + wrap(p / 2.0));
                Some([t, p, wrap(alpha - beta - t + p)])
            }
            DubinsWord::Lrl => {
                let cosine = (6.0 - d * d + 2.0 * cab + 2.0 * d * (sb - sa)) / 8.0;
                if math::abs(cosine) > 1.0 {
                    return None;
                }
                let phi = math::atan2(ca - cb, d + sa - sb);
                let p = wrap(TAU - math::acos(cosine));
                let t = wrap(-alpha - phi + p / 2.0);
                Some([t, p, wrap(beta - alpha - t + p)])
            }
        }
    }
}
