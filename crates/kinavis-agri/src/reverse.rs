//! Turns that reverse: the shortest path of bounded curvature between two
//! poses for a machine that may back up (Reeds and Shepp).
//!
//! Up to five pieces, each an arc of the turning radius or a straight, each
//! driven forward or in reverse; a change of gear is a cusp. Onto a pass
//! closer than two turning radii this is the three-point (fishtail) turn,
//! shorter than the omega turn of [`TurnPath`](crate::TurnPath), which
//! never reverses.

// t, u, v, x, y as in the paper.
#![allow(clippy::many_single_char_names)]

use kinavis_kernel::error::ensure_range;
use kinavis_kernel::{math, Distance, KernelError};

use crate::error::Result;
use crate::field::Field;
use crate::line::{metres, ReferenceLine};
use crate::pass::{PassIndex, PassPattern};
use crate::plane::FieldPlane;
use crate::turn::{first_conflict, pass_entry, MathPose, Pose, Steer};

const PI: f64 = core::f64::consts::PI;
const TAU: f64 = core::f64::consts::TAU;
const HALF_PI: f64 = core::f64::consts::FRAC_PI_2;

/// Slack on the sign conditions of the formulas, in radii.
const ZERO: f64 = 10.0 * f64::EPSILON;

/// Pieces shorter than this, in radii, are dropped.
const NEGLIGIBLE: f64 = 1e-12;

/// Most pieces in a Reeds–Shepp path.
pub const MAX_REVERSING_PIECES: usize = 5;

/// Direction of travel along a piece.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Gear {
    /// Forward.
    Forward,
    /// Reverse.
    Reverse,
}

/// One piece of a turn that reverses.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GearedPiece {
    /// Direction of the wheels: an arc turns left or right in the machine's
    /// frame, in either gear.
    pub steer: Steer,
    /// Direction of travel.
    pub gear: Gear,
    /// Length along the path, positive in either gear.
    pub length: Distance,
}

/// Piece in units of the radius, negative in reverse.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Segment {
    steer: Steer,
    length: f64,
}

impl Segment {
    const NONE: Self = Self {
        steer: Steer::Straight,
        length: 0.0,
    };

    fn gear(self) -> Gear {
        if self.length < 0.0 {
            Gear::Reverse
        } else {
            Gear::Forward
        }
    }
}

/// Shortest path of bounded curvature between two poses with reversing
/// allowed: up to five pieces of one of the 48 Reeds–Shepp words. Of paths
/// equally short, the one with least driven in reverse.
///
/// Poses along the path give the machine's heading, which is against the
/// direction of travel on a piece driven in reverse.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ReedsSheppPath {
    plane: FieldPlane,
    start: Pose,
    radius: f64,
    segments: [Segment; MAX_REVERSING_PIECES],
    count: usize,
}

impl ReedsSheppPath {
    /// Shortest path from one pose to another with turns no tighter than
    /// `radius`, forward or in reverse.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] for a radius outside 0.5…1000 m;
    /// [`KernelError::Indeterminate`] if no word connects the poses (not
    /// reachable for finite input).
    pub fn shortest(plane: FieldPlane, from: Pose, to: Pose, radius: Distance) -> Result<Self> {
        let radius = radius.metres();
        ensure_range("turning radius, metres", radius, 0.5, 1_000.0)?;
        let start = MathPose::of(from, radius);
        let end = MathPose::of(to, radius);
        // End pose in the start frame.
        let (dx, dy) = (end.x - start.x, end.y - start.y);
        let (s, c) = (math::sin(start.heading), math::cos(start.heading));
        let word = shortest_word(
            c * dx + s * dy,
            c * dy - s * dx,
            mod2pi(end.heading - start.heading),
        )
        .ok_or(KernelError::Indeterminate {
            quantity: "a Reeds-Shepp path between the poses",
        })?;
        let (segments, count) = tidy(word);
        Ok(Self {
            plane,
            start: from,
            radius,
            segments,
            count,
        })
    }

    /// Turn from a pose at the end of a pass onto another pass of the same
    /// line, joining it abreast of the exit point and heading back.
    ///
    /// # Errors
    ///
    /// As [`ReedsSheppPath::shortest`].
    pub fn to_pass<L: ReferenceLine>(
        line: &L,
        pattern: &PassPattern,
        exit: Pose,
        to: PassIndex,
        radius: Distance,
    ) -> Result<Self> {
        Self::shortest(
            *line.plane(),
            exit,
            pass_entry(line, pattern, exit, to),
            radius,
        )
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

    fn segments(&self) -> &[Segment] {
        self.segments.get(..self.count).unwrap_or(&[])
    }

    /// The pieces in order, none of zero length and no two neighbours alike
    /// in steer and gear; none for a path between equal poses.
    pub fn pieces(&self) -> impl Iterator<Item = GearedPiece> + '_ {
        self.segments().iter().map(|segment| GearedPiece {
            steer: segment.steer,
            gear: segment.gear(),
            length: metres(math::abs(segment.length) * self.radius),
        })
    }

    /// Length, both gears counted.
    #[must_use]
    pub fn length(&self) -> Distance {
        metres(self.length_radii() * self.radius)
    }

    fn length_radii(&self) -> f64 {
        self.segments().iter().map(|s| math::abs(s.length)).sum()
    }

    /// Distances along the path at which the gear changes.
    pub fn cusps(&self) -> impl Iterator<Item = Distance> + '_ {
        self.segments()
            .windows(2)
            .scan(0.0, |along, pair| {
                let (first, second) = (pair.first()?, pair.get(1)?);
                *along += math::abs(first.length);
                Some((*along, first.gear() != second.gear()))
            })
            .filter(|&(_, cusp)| cusp)
            .map(|(along, _)| metres(along * self.radius))
    }

    /// Gear at a distance along the path: of the piece being driven, the
    /// next one at a cusp; forward for a path of no length.
    #[must_use]
    pub fn gear_at(&self, along: Distance) -> Gear {
        let mut left = along.metres() / self.radius;
        let mut gear = Gear::Forward;
        for segment in self.segments() {
            gear = segment.gear();
            left -= math::abs(segment.length);
            if left < 0.0 {
                break;
            }
        }
        gear
    }

    /// Pose at a distance along the path, clamped to its ends.
    #[must_use]
    pub fn pose_at(&self, along: Distance) -> Pose {
        let mut left = along.metres().max(0.0) / self.radius;
        let mut state = MathPose::of(self.start, self.radius);
        for segment in self.segments() {
            if left <= 0.0 {
                break;
            }
            let run = left.min(math::abs(segment.length));
            state = state.advance(segment.steer, math::copysign(run, segment.length));
            left -= run;
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
    /// `step`; `None` if it stays clear throughout. As
    /// [`TurnPath::first_conflict`](crate::TurnPath::first_conflict): a turn
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

/// Angle reduced to `(-π, π]` the way C's `fmod` leaves it.
fn mod2pi(x: f64) -> f64 {
    let v = x - math::trunc(x / TAU) * TAU;
    if v < -PI {
        v + TAU
    } else if v > PI {
        v - TAU
    } else {
        v
    }
}

fn polar(x: f64, y: f64) -> (f64, f64) {
    (math::hypot(x, y), math::atan2(y, x))
}

/// Base formula: lengths `t, u, v` of its word, if it connects the origin
/// heading east to `(x, y)` heading `φ`, in radii.
type Formula = fn(f64, f64, f64) -> Option<(f64, f64, f64)>;

/// A family of words: a base word, the lengths of its pieces from the
/// formula's `t, u, v`, and whether the word read backwards is tried too.
/// Each is tried as is, time-flipped (gears swapped), reflected (left and
/// right swapped) and both.
struct Family {
    formula: Formula,
    steers: &'static [Steer],
    lengths: fn(f64, f64, f64) -> [f64; MAX_REVERSING_PIECES],
    backwards: bool,
}

use Steer::{Left as L, Right as R, Straight as S};

/// Formulas 8.1–8.11 of Reeds and Shepp (1990), with the corrections to
/// 8.3/8.4 and 8.11 made in OMPL's `ReedsSheppStateSpace`.
const FAMILIES: [Family; 8] = [
    // 8.1, CSC
    Family {
        formula: lp_sp_lp,
        steers: &[L, S, L],
        lengths: |t, u, v| [t, u, v, 0.0, 0.0],
        backwards: false,
    },
    // 8.2, CSC
    Family {
        formula: lp_sp_rp,
        steers: &[L, S, R],
        lengths: |t, u, v| [t, u, v, 0.0, 0.0],
        backwards: false,
    },
    // 8.3, C|C|C and CC|C
    Family {
        formula: lp_rm_l,
        steers: &[L, R, L],
        lengths: |t, u, v| [t, u, v, 0.0, 0.0],
        backwards: true,
    },
    // 8.7, CCu|CuC
    Family {
        formula: lp_rup_lum_rm,
        steers: &[L, R, L, R],
        lengths: |t, u, v| [t, u, -u, v, 0.0],
        backwards: false,
    },
    // 8.8, C|CuCu|C
    Family {
        formula: lp_rum_lum_rp,
        steers: &[L, R, L, R],
        lengths: |t, u, v| [t, u, u, v, 0.0],
        backwards: false,
    },
    // 8.9, C|C(π/2)SC
    Family {
        formula: lp_rm_sm_lm,
        steers: &[L, R, S, L],
        lengths: |t, u, v| [t, -HALF_PI, u, v, 0.0],
        backwards: true,
    },
    // 8.10, C|C(π/2)SC
    Family {
        formula: lp_rm_sm_rm,
        steers: &[L, R, S, R],
        lengths: |t, u, v| [t, -HALF_PI, u, v, 0.0],
        backwards: true,
    },
    // 8.11, C|C(π/2)SC(π/2)|C
    Family {
        formula: lp_rm_s_lm_rp,
        steers: &[L, R, S, L, R],
        lengths: |t, u, v| [t, -HALF_PI, u, -HALF_PI, v],
        backwards: false,
    },
];

/// Shortest word to `(x, y, φ)` from the origin heading east, in radii.
fn shortest_word(x: f64, y: f64, phi: f64) -> Option<[Segment; MAX_REVERSING_PIECES]> {
    // The goal seen from the end, for the words read backwards.
    let (sp, cp) = (math::sin(phi), math::cos(phi));
    let (xb, yb) = (x * cp + y * sp, x * sp - y * cp);
    let mut best: Option<([Segment; MAX_REVERSING_PIECES], Cost)> = None;
    for family in &FAMILIES {
        let goals: &[(f64, f64, bool)] = if family.backwards {
            &[(x, y, false), (xb, yb, true)]
        } else {
            &[(x, y, false)]
        };
        for &(gx, gy, backwards) in goals {
            for (flip, reflect) in [(false, false), (true, false), (false, true), (true, true)] {
                let gx = if flip { -gx } else { gx };
                let gy = if reflect { -gy } else { gy };
                let gphi = if flip == reflect { phi } else { -phi };
                let Some((t, u, v)) = (family.formula)(gx, gy, gphi) else {
                    continue;
                };
                let mut word = [Segment::NONE; MAX_REVERSING_PIECES];
                let pieces = family.steers.iter().zip((family.lengths)(t, u, v));
                for (slot, (&steer, length)) in word.iter_mut().zip(pieces) {
                    *slot = Segment {
                        steer: if reflect { mirrored(steer) } else { steer },
                        length: if flip { -length } else { length },
                    };
                }
                if backwards {
                    if let Some(used) = word.get_mut(..family.steers.len()) {
                        used.reverse();
                    }
                }
                let cost = Cost::of(&word);
                if best.is_none_or(|(_, b)| cost.beats(b)) {
                    best = Some((word, cost));
                }
            }
        }
    }
    best.map(|(word, _)| word)
}

/// Length of a word and the part of it in reverse, in radii.
#[derive(Debug, Clone, Copy)]
struct Cost {
    total: f64,
    reverse: f64,
}

impl Cost {
    fn of(word: &[Segment]) -> Self {
        let total = word.iter().map(|s| math::abs(s.length)).sum();
        let reverse = word
            .iter()
            .filter(|s| s.gear() == Gear::Reverse)
            .map(|s| -s.length)
            .sum();
        Self { total, reverse }
    }

    /// Shorter, or as short (to rounding) with less in reverse: of a U-turn
    /// ahead and its mirror image driven backwards, the one ahead.
    fn beats(self, other: Self) -> bool {
        let tie = 1e-9 * other.total.max(1.0);
        self.total < other.total - tie
            || (self.total <= other.total + tie && self.reverse < other.reverse - tie)
    }
}

const fn mirrored(steer: Steer) -> Steer {
    match steer {
        Steer::Left => Steer::Right,
        Steer::Right => Steer::Left,
        Steer::Straight => Steer::Straight,
    }
}

/// Word without pieces of no length, neighbours alike in steer and gear
/// merged.
fn tidy(word: [Segment; MAX_REVERSING_PIECES]) -> ([Segment; MAX_REVERSING_PIECES], usize) {
    let mut out = [Segment::NONE; MAX_REVERSING_PIECES];
    let mut count: usize = 0;
    for segment in word {
        if math::abs(segment.length) <= NEGLIGIBLE {
            continue;
        }
        let last = count.checked_sub(1).and_then(|i| out.get_mut(i));
        match last {
            Some(last) if last.steer == segment.steer && last.gear() == segment.gear() => {
                last.length += segment.length;
            }
            _ => {
                if let Some(slot) = out.get_mut(count) {
                    *slot = segment;
                    count += 1;
                }
            }
        }
    }
    (out, count)
}

/// `τ, ω` of formulas 8.7 and 8.8.
fn tau_omega(u: f64, v: f64, xi: f64, eta: f64, phi: f64) -> (f64, f64) {
    let delta = mod2pi(u - v);
    let a = math::sin(u) - math::sin(delta);
    let b = math::cos(u) - math::cos(delta) - 1.0;
    let t1 = math::atan2(eta * a - xi * b, xi * a + eta * b);
    let t2 = 2.0 * (math::cos(delta) - math::cos(v) - math::cos(u)) + 3.0;
    let tau = if t2 < 0.0 {
        mod2pi(t1 + PI)
    } else {
        mod2pi(t1)
    };
    (tau, mod2pi(tau - u + v - phi))
}

/// 8.1: L⁺S⁺L⁺.
fn lp_sp_lp(x: f64, y: f64, phi: f64) -> Option<(f64, f64, f64)> {
    let (u, t) = polar(x - math::sin(phi), y - 1.0 + math::cos(phi));
    let v = mod2pi(phi - t);
    (t >= -ZERO && v >= -ZERO).then_some((t, u, v))
}

/// 8.2: L⁺S⁺R⁺.
fn lp_sp_rp(x: f64, y: f64, phi: f64) -> Option<(f64, f64, f64)> {
    let (u1, t1) = polar(x + math::sin(phi), y - 1.0 - math::cos(phi));
    let u1 = u1 * u1;
    if u1 < 4.0 {
        return None;
    }
    let u = math::sqrt(u1 - 4.0);
    let t = mod2pi(t1 + math::atan2(2.0, u));
    let v = mod2pi(t - phi);
    (t >= -ZERO && v >= -ZERO).then_some((t, u, v))
}

/// 8.3/8.4: L⁺R⁻L.
fn lp_rm_l(x: f64, y: f64, phi: f64) -> Option<(f64, f64, f64)> {
    let (u1, theta) = polar(x - math::sin(phi), y - 1.0 + math::cos(phi));
    if u1 > 4.0 {
        return None;
    }
    let u = -2.0 * math::asin(0.25 * u1);
    let t = mod2pi(theta + 0.5 * u + PI);
    let v = mod2pi(phi - t + u);
    (t >= -ZERO && u <= ZERO).then_some((t, u, v))
}

/// 8.7: L⁺R⁺ᵤL⁻ᵤR⁻.
fn lp_rup_lum_rm(x: f64, y: f64, phi: f64) -> Option<(f64, f64, f64)> {
    let (xi, eta) = (x + math::sin(phi), y - 1.0 - math::cos(phi));
    let rho = 0.25 * (2.0 + math::hypot(xi, eta));
    if rho > 1.0 {
        return None;
    }
    let u = math::acos(rho);
    let (t, v) = tau_omega(u, -u, xi, eta, phi);
    (t >= -ZERO && v <= ZERO).then_some((t, u, v))
}

/// 8.8: L⁺R⁻ᵤL⁻ᵤR⁺.
fn lp_rum_lum_rp(x: f64, y: f64, phi: f64) -> Option<(f64, f64, f64)> {
    let (xi, eta) = (x + math::sin(phi), y - 1.0 - math::cos(phi));
    let rho = (20.0 - xi * xi - eta * eta) / 16.0;
    if !(0.0..=1.0).contains(&rho) {
        return None;
    }
    let u = -math::acos(rho);
    if u < -HALF_PI {
        return None;
    }
    let (t, v) = tau_omega(u, u, xi, eta, phi);
    (t >= -ZERO && v >= -ZERO).then_some((t, u, v))
}

/// 8.9: L⁺R⁻(π/2)S⁻L⁻.
fn lp_rm_sm_lm(x: f64, y: f64, phi: f64) -> Option<(f64, f64, f64)> {
    let (rho, theta) = polar(x - math::sin(phi), y - 1.0 + math::cos(phi));
    if rho < 2.0 {
        return None;
    }
    let r = math::sqrt(rho * rho - 4.0);
    let u = 2.0 - r;
    let t = mod2pi(theta + math::atan2(r, -2.0));
    let v = mod2pi(phi - HALF_PI - t);
    (t >= -ZERO && u <= ZERO && v <= ZERO).then_some((t, u, v))
}

/// 8.10: L⁺R⁻(π/2)S⁻R⁻.
fn lp_rm_sm_rm(x: f64, y: f64, phi: f64) -> Option<(f64, f64, f64)> {
    let (xi, eta) = (x + math::sin(phi), y - 1.0 - math::cos(phi));
    let (rho, theta) = polar(-eta, xi);
    if rho < 2.0 {
        return None;
    }
    let t = theta;
    let u = 2.0 - rho;
    let v = mod2pi(t + HALF_PI - phi);
    (t >= -ZERO && u <= ZERO && v <= ZERO).then_some((t, u, v))
}

/// 8.11: L⁺R⁻(π/2)S⁻L⁻(π/2)R⁺.
fn lp_rm_s_lm_rp(x: f64, y: f64, phi: f64) -> Option<(f64, f64, f64)> {
    let (xi, eta) = (x + math::sin(phi), y - 1.0 - math::cos(phi));
    let (rho, _) = polar(xi, eta);
    if rho < 2.0 {
        return None;
    }
    let u = 4.0 - math::sqrt(rho * rho - 4.0);
    if u > ZERO {
        return None;
    }
    let t = mod2pi(math::atan2(
        (4.0 - u) * xi - 2.0 * eta,
        -2.0 * xi + (u - 4.0) * eta,
    ));
    let v = mod2pi(t - phi);
    (t >= -ZERO && v >= -ZERO).then_some((t, u, v))
}
