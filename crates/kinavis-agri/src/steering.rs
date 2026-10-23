//! Steering a machine along a pass: geometry, the commanded wheel angle, the
//! pure pursuit and Stanley trackers, the kinematic bicycle.
//!
//! The output is a wheel angle to command, not control of the machine: the
//! actuator, its loop and its safety are the caller's. Angles follow the body
//! frame (forward, right, down): a positive wheel angle and a positive
//! curvature turn the machine to the right going forward, to the left in
//! reverse.

use core::time::Duration;

use kinavis::region::PlanarPoint;
use kinavis_kernel::error::ensure_range;
use kinavis_kernel::{math, Angle, Distance, GridCourse, RateOfTurn, Speed};

use crate::error::Result;
use crate::line::{metres, ReferenceLine};
use crate::pass::{PassIndex, PassPattern};
use crate::tilt::LeverArm;

/// Geometry of a front-steered machine: wheelbase, the limits of its
/// steering, the antenna's place.
///
/// The control point is the middle of the rear axle: the point that moves
/// along the heading without side slip in the kinematic bicycle.
///
/// With the `serde` feature the wheel angle and steering rate are stored in
/// radians, as held, so that a geometry reads back equal to itself.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(try_from = "StoredGeometry", into = "StoredGeometry")
)]
pub struct VehicleGeometry {
    wheelbase: f64,
    max_angle: f64,
    max_rate: f64,
    antenna: LeverArm,
}

impl VehicleGeometry {
    /// Geometry with the antenna above the rear axle at zero height.
    ///
    /// # Errors
    ///
    /// [`kinavis_kernel::KernelError::OutOfRange`] for a wheelbase outside
    /// 0.1…30 m, a maximum wheel angle outside 1°…80°, or a steering rate
    /// not above zero or beyond 360°/s.
    pub fn new(wheelbase: Distance, max_angle: Angle, max_rate: RateOfTurn) -> Result<Self> {
        ensure_range("wheelbase, metres", wheelbase.metres(), 0.1, 30.0)?;
        ensure_range(
            "maximum wheel angle, degrees",
            max_angle.degrees(),
            1.0,
            80.0,
        )?;
        let rate = max_rate.radians_per_second();
        ensure_range(
            "maximum steering rate, rad/s",
            rate,
            1e-6,
            math::to_radians(360.0),
        )?;
        Ok(Self {
            wheelbase: wheelbase.metres(),
            max_angle: max_angle.radians(),
            max_rate: rate,
            antenna: LeverArm::above(Distance::ZERO),
        })
    }

    /// Geometry with the antenna at a lever arm from the rear axle.
    #[must_use]
    pub const fn with_antenna(self, antenna: LeverArm) -> Self {
        Self { antenna, ..self }
    }

    /// Wheelbase.
    #[must_use]
    pub fn wheelbase(&self) -> Distance {
        metres(self.wheelbase)
    }

    /// Largest wheel angle either way.
    #[must_use]
    pub fn max_angle(&self) -> Angle {
        Angle::from_radians(self.max_angle).unwrap_or(Angle::ZERO)
    }

    /// Fastest change of the wheel angle.
    #[must_use]
    pub fn max_rate(&self) -> RateOfTurn {
        RateOfTurn::from_radians_per_second(self.max_rate).unwrap_or(RateOfTurn::ZERO)
    }

    /// Antenna relative to the rear axle.
    #[must_use]
    pub const fn antenna(&self) -> LeverArm {
        self.antenna
    }

    /// Smallest turning radius of the rear axle, `L / tan(δ_max)`.
    #[must_use]
    pub fn min_turning_radius(&self) -> Distance {
        metres(self.wheelbase / math::tan(self.max_angle))
    }

    /// Command for a wheel angle, clamped to the maximum.
    #[must_use]
    pub fn command(&self, wheel_angle: f64) -> SteeringCommand {
        let angle = wheel_angle.max(-self.max_angle).min(self.max_angle);
        SteeringCommand {
            angle: Angle::from_radians(angle).unwrap_or(Angle::ZERO),
            curvature: math::tan(angle) / self.wheelbase,
        }
    }

    /// Wheel angle reached from `current` towards `wanted` in `over`, at the
    /// steering rate and within the maximum angle.
    #[must_use]
    pub fn slew(&self, current: Angle, wanted: Angle, over: Duration) -> Angle {
        let step = self.max_rate * over.as_secs_f64();
        let current = current.radians();
        let reached = current + (wanted.radians() - current).max(-step).min(step);
        Angle::from_radians(reached.max(-self.max_angle).min(self.max_angle)).unwrap_or(Angle::ZERO)
    }

    /// Front axle middle of a machine whose rear axle is at `rear`.
    #[must_use]
    pub fn front_axle(&self, rear: PlanarPoint, heading: GridCourse) -> PlanarPoint {
        let (north, east) = heading.components(self.wheelbase);
        offset_point(rear, east, north)
    }
}

/// Wheel angle to command, with the curvature it gives the rear axle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SteeringCommand {
    /// Wheel angle, right positive.
    pub angle: Angle,
    /// Curvature of the rear axle's path, 1/m, right positive.
    pub curvature: f64,
}

/// Rear axle position, heading and speed of a machine.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VehicleState {
    /// Middle of the rear axle, in the field plane.
    pub position: PlanarPoint,
    /// Grid heading.
    pub heading: GridCourse,
    /// Speed along the heading, negative in reverse.
    pub speed: Speed,
}

impl VehicleState {
    /// Whether the machine is reversing.
    #[must_use]
    pub fn is_reversing(&self) -> bool {
        self.speed.metres_per_second() < 0.0
    }

    /// Grid course of the motion: the heading, its reciprocal in reverse.
    #[must_use]
    pub fn travel(&self) -> GridCourse {
        if self.is_reversing() {
            self.heading.reciprocal()
        } else {
            self.heading
        }
    }
}

/// Role of a pass tracker: the wheel angle that brings a machine onto a pass
/// and holds it there.
///
/// The pass is followed in the direction along it nearest the direction of
/// motion, [`VehicleState::travel`], as [`crate::locate`] takes it for that
/// course: the machine reverses along a pass when its speed is negative.
pub trait PathTracker {
    /// Command for one step.
    ///
    /// # Errors
    ///
    /// [`kinavis_kernel::KernelError::OutOfRange`] for a speed beyond
    /// 100 m/s either way.
    fn steer<L: ReferenceLine>(
        &self,
        line: &L,
        pattern: &PassPattern,
        pass: PassIndex,
        vehicle: &VehicleGeometry,
        state: &VehicleState,
    ) -> Result<SteeringCommand>;
}

/// Pure pursuit: steer the rear axle on the circle through the point of the
/// pass a look-ahead distance away.
///
/// The look-ahead grows with speed, `base + gain · v`, up to a maximum. A
/// longer look-ahead is smoother and slower to converge; on a pass of
/// constant curvature the rear axle settles on the pass exactly.
///
/// Classic pure pursuit aims at the foot of the perpendicular when the pass
/// is farther than the look-ahead, and so meets it nearly square and
/// overshoots, the more so with the wheels at their stop. With an intercept
/// angle θ ([`PurePursuit::with_intercept`]) the look-ahead is stretched to
/// `|e| / sin θ`, so the pass is approached at θ at most.
///
/// In reverse the goal is behind the machine and the arc the same: the rear
/// axle moves on a circle tangent to the heading in either gear, and the
/// wheel angle that gives it is of the opposite sign.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PurePursuit {
    base: f64,
    gain: f64,
    max: f64,
    intercept: Option<f64>,
}

impl PurePursuit {
    /// Tracker with look-ahead `base + gain · v`, at most `max`.
    ///
    /// # Errors
    ///
    /// [`kinavis_kernel::KernelError::OutOfRange`] for a base outside
    /// 0.1…100 m, a gain outside 0…10 s, or a maximum below the base or
    /// beyond 100 m.
    pub fn new(base: Distance, gain: Duration, max: Distance) -> Result<Self> {
        ensure_range(
            "look-ahead base, metres",
            base.metres(),
            0.1,
            MAX_LOOK_AHEAD,
        )?;
        let gain = gain.as_secs_f64();
        ensure_range("look-ahead gain, seconds", gain, 0.0, 10.0)?;
        ensure_range(
            "longest look-ahead, metres",
            max.metres(),
            base.metres(),
            MAX_LOOK_AHEAD,
        )?;
        Ok(Self {
            base: base.metres(),
            gain,
            max: max.metres(),
            intercept: None,
        })
    }

    /// The same tracker approaching a distant pass at `angle` at most.
    ///
    /// # Errors
    ///
    /// [`kinavis_kernel::KernelError::OutOfRange`] for an angle outside
    /// 5°…85°.
    pub fn with_intercept(self, angle: Angle) -> Result<Self> {
        ensure_range("intercept angle, degrees", angle.degrees(), 5.0, 85.0)?;
        Ok(Self {
            intercept: Some(math::sin(angle.radians())),
            ..self
        })
    }

    /// Look-ahead distance at a speed.
    #[must_use]
    pub fn look_ahead(&self, speed: Speed) -> Distance {
        metres(self.look_ahead_metres(speed.metres_per_second()))
    }

    fn look_ahead_metres(&self, speed: f64) -> f64 {
        (self.base + self.gain * math::abs(speed)).min(self.max)
    }
}

impl PathTracker for PurePursuit {
    fn steer<L: ReferenceLine>(
        &self,
        line: &L,
        pattern: &PassPattern,
        pass: PassIndex,
        vehicle: &VehicleGeometry,
        state: &VehicleState,
    ) -> Result<SteeringCommand> {
        let speed = checked_speed(state)?;
        let target = pattern.offset_metres(pass);
        let travel = state.travel();
        let forward = travels_forward(line, state.position, travel);
        let rear = state.position;
        let foot = project(line, target, rear);
        let mut look_ahead = self.look_ahead_metres(speed);
        if let Some(sine) = self.intercept {
            look_ahead = look_ahead.max(distance(rear, foot) / sine);
        }
        // Walk the pass until the look-ahead circle is crossed.
        let step = (look_ahead / 16.0).min(0.5);
        let mut previous = foot;
        let mut goal = foot;
        if distance(rear, foot) < look_ahead {
            for _ in 0..MAX_STEPS {
                let next = advance(line, target, previous, step, forward);
                if distance(rear, next) >= look_ahead {
                    goal = crossing(line, target, rear, previous, next, look_ahead);
                    break;
                }
                previous = next;
                goal = next;
            }
        }
        let bearing = math::atan2(
            goal.east_metres() - rear.east_metres(),
            goal.north_metres() - rear.north_metres(),
        );
        let alpha = wrap_pi(bearing - travel.radians());
        // Curvature of the motion, right positive.
        let curvature = 2.0 * math::sin(alpha) / look_ahead;
        Ok(vehicle.command(math::atan(vehicle.wheelbase * gear_sign(state) * curvature)))
    }
}

/// Stanley: turn the wheels by the heading error plus the angle that closes
/// the cross-track error of the front axle.
///
/// `δ = −ψₑ − atan(k·e / (v + v_s))`, with ψₑ and e of the front axle against
/// the pass (right positive, e the distance to the pass) and the softening
/// speed `v_s` bounding the gain at a standstill. The heading error alone
/// supplies the wheel angle a curve needs, so the front axle settles on a
/// pass of constant curvature; the rear axle then runs `R − √(R² − L²)`
/// inside it, 5.8 cm for a 2.9 m wheelbase on a 72 m circle.
///
/// In reverse the steered axle trails and holding it on the pass would
/// leave the heading free to jackknife. The rear axle, which leads, is
/// guided instead, its heading turned towards the angle that closes the
/// cross-track error, with the wheel angle of the pass's curvature fed
/// forward: `δ = −atan(L·κ) + ψₑ + atan(k·e / (|v| + v_s))`, ψₑ and e of
/// the rear axle against the motion, κ of the pass, right positive. On a
/// straight pass the error then rings as a second-order system of natural
/// frequency `√(k·|v| / L)` and damping `½·√(|v| / (k·L))`: settled
/// reversing wants a smaller gain than driving forward.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Stanley {
    gain: f64,
    softening: f64,
}

impl Stanley {
    /// Tracker with gain `k`, 1/s, and a softening speed.
    ///
    /// # Errors
    ///
    /// [`kinavis_kernel::KernelError::OutOfRange`] for a gain outside
    /// 0.01…20 /s or a softening speed outside 0…10 m/s.
    pub fn new(gain: f64, softening: Speed) -> Result<Self> {
        ensure_range("Stanley gain, 1/s", gain, 0.01, 20.0)?;
        ensure_range(
            "Stanley softening speed, m/s",
            softening.metres_per_second(),
            0.0,
            10.0,
        )?;
        Ok(Self {
            gain,
            softening: softening.metres_per_second(),
        })
    }
}

impl PathTracker for Stanley {
    fn steer<L: ReferenceLine>(
        &self,
        line: &L,
        pattern: &PassPattern,
        pass: PassIndex,
        vehicle: &VehicleGeometry,
        state: &VehicleState,
    ) -> Result<SteeringCommand> {
        let speed = math::abs(checked_speed(state)?) + self.softening;
        let travel = state.travel();
        let guided = if state.is_reversing() {
            state.position
        } else {
            vehicle.front_axle(state.position, state.heading)
        };
        let view = crate::tracker::locate_pass(line, pattern, guided, travel, pass);
        let cross_track = view.cross_track.metres();
        let heading_error = view.heading_error.radians();
        let correction = if speed > 0.0 {
            math::atan(self.gain * cross_track / speed)
        } else {
            math::copysign(core::f64::consts::FRAC_PI_2, cross_track)
        };
        if state.is_reversing() {
            let target = pattern.offset_metres(pass);
            let curvature = pass_curvature(line, target, guided, !view.reversed);
            let feed_forward = -math::atan(vehicle.wheelbase * curvature);
            return Ok(vehicle.command(feed_forward + heading_error + correction));
        }
        Ok(vehicle.command(-heading_error - correction))
    }
}

/// Rear axle state after `over` at constant speed and wheel angle: the exact
/// kinematic bicycle, an arc of curvature `tan δ / L`.
///
/// # Errors
///
/// [`kinavis_kernel::KernelError::NotFinite`]; not reachable for validated
/// inputs.
pub fn bicycle_step(
    state: &VehicleState,
    vehicle: &VehicleGeometry,
    wheel_angle: Angle,
    over: Duration,
) -> Result<VehicleState> {
    let curvature = math::tan(wheel_angle.radians()) / vehicle.wheelbase;
    let length = state.speed.metres_per_second() * over.as_secs_f64();
    let turn = curvature * length;
    // Chord of the arc and its direction: half the turn on.
    let chord = length * sinc(turn / 2.0);
    let direction = state.heading.radians() + turn / 2.0;
    let east = chord * math::sin(direction);
    let north = chord * math::cos(direction);
    Ok(VehicleState {
        position: PlanarPoint::from_metres(
            state.position.east_metres() + east,
            state.position.north_metres() + north,
        )?,
        heading: GridCourse::wrap(math::to_degrees(state.heading.radians() + turn))?,
        speed: state.speed,
    })
}

/// Longest look-ahead, metres.
const MAX_LOOK_AHEAD: f64 = 100.0;

/// Most steps of a walk along a pass: 100 m at 0.5 m with margin.
const MAX_STEPS: usize = 512;

/// Distance either side of a point over which the curvature of a pass is
/// taken, metres.
const CURVATURE_SPAN: f64 = 0.1;

fn checked_speed(state: &VehicleState) -> Result<f64> {
    let speed = state.speed.metres_per_second();
    ensure_range("speed, m/s", speed, -100.0, 100.0)?;
    Ok(speed)
}

/// +1 forward, −1 in reverse: the sign that turns the curvature of the
/// motion into that of the body.
fn gear_sign(state: &VehicleState) -> f64 {
    if state.is_reversing() {
        -1.0
    } else {
        1.0
    }
}

/// Curvature of the pass at `point`, 1/m, right positive along the travel:
/// the turn of its direction over [`CURVATURE_SPAN`] either side.
fn pass_curvature<L: ReferenceLine>(
    line: &L,
    target: f64,
    point: PlanarPoint,
    forward: bool,
) -> f64 {
    let foot = project(line, target, point);
    let behind = advance(line, target, foot, CURVATURE_SPAN, !forward);
    let ahead = advance(line, target, foot, CURVATURE_SPAN, forward);
    let span = distance(behind, ahead);
    if span <= 0.0 {
        return 0.0;
    }
    // Reference directions: their difference is that of the travel when it
    // runs either way along them.
    let turn = line
        .station(behind)
        .direction
        .signed_difference(line.station(ahead).direction);
    math::to_radians(turn) / span
}

fn travels_forward<L: ReferenceLine>(line: &L, at: PlanarPoint, heading: GridCourse) -> bool {
    line.station(at).direction.angular_distance(heading) <= 90.0
}

/// Point of the pass at signed offset `target` nearest `point`, by moving
/// across the pass direction until the offset matches.
pub(crate) fn project<L: ReferenceLine>(line: &L, target: f64, point: PlanarPoint) -> PlanarPoint {
    let mut p = point;
    for _ in 0..4 {
        let station = line.station(p);
        let error = station.offset.metres() - target;
        if math::abs(error) < 1e-9 {
            break;
        }
        // Left normal of the direction (east, north) = (sin θ, cos θ).
        let theta = station.direction.radians();
        p = offset_point(p, error * math::cos(theta), -error * math::sin(theta));
    }
    p
}

/// Next point of the pass `step` on from `from`, in or against the line's
/// reference direction.
fn advance<L: ReferenceLine>(
    line: &L,
    target: f64,
    from: PlanarPoint,
    step: f64,
    forward: bool,
) -> PlanarPoint {
    let theta = line.station(from).direction.radians();
    let signed = if forward { step } else { -step };
    project(
        line,
        target,
        offset_point(from, signed * math::sin(theta), signed * math::cos(theta)),
    )
}

/// Point between two points of the pass where the distance from `centre`
/// reaches `radius`, by bisection on the pass.
fn crossing<L: ReferenceLine>(
    line: &L,
    target: f64,
    centre: PlanarPoint,
    inside: PlanarPoint,
    outside: PlanarPoint,
    radius: f64,
) -> PlanarPoint {
    let (mut near, mut far) = (inside, outside);
    for _ in 0..40 {
        let middle = project(
            line,
            target,
            offset_point(
                near,
                0.5 * (far.east_metres() - near.east_metres()),
                0.5 * (far.north_metres() - near.north_metres()),
            ),
        );
        if distance(centre, middle) < radius {
            near = middle;
        } else {
            far = middle;
        }
    }
    far
}

fn offset_point(point: PlanarPoint, east: f64, north: f64) -> PlanarPoint {
    PlanarPoint::from_metres(point.east_metres() + east, point.north_metres() + north)
        .unwrap_or(point)
}

fn distance(a: PlanarPoint, b: PlanarPoint) -> f64 {
    math::hypot(
        b.east_metres() - a.east_metres(),
        b.north_metres() - a.north_metres(),
    )
}

/// Angle wrapped to `(−π, π]`.
fn wrap_pi(angle: f64) -> f64 {
    math::atan2(math::sin(angle), math::cos(angle))
}

/// `sin x / x`, 1 at 0.
fn sinc(x: f64) -> f64 {
    if math::abs(x) < 1e-4 {
        1.0 - x * x / 6.0
    } else {
        math::sin(x) / x
    }
}

/// Serialised form, in the units held.
#[cfg(feature = "serde")]
#[derive(Clone, Copy, serde::Serialize, serde::Deserialize)]
struct StoredGeometry {
    wheelbase_metres: f64,
    max_wheel_angle_radians: f64,
    max_steering_rate_radians_per_second: f64,
    antenna: LeverArm,
}

#[cfg(feature = "serde")]
impl TryFrom<StoredGeometry> for VehicleGeometry {
    type Error = crate::error::AgriError;

    /// The ranges of [`VehicleGeometry::new`].
    fn try_from(stored: StoredGeometry) -> Result<Self> {
        ensure_range("wheelbase, metres", stored.wheelbase_metres, 0.1, 30.0)?;
        ensure_range(
            "maximum wheel angle, radians",
            stored.max_wheel_angle_radians,
            math::to_radians(1.0),
            math::to_radians(80.0),
        )?;
        ensure_range(
            "maximum steering rate, rad/s",
            stored.max_steering_rate_radians_per_second,
            1e-6,
            math::to_radians(360.0),
        )?;
        Ok(Self {
            wheelbase: stored.wheelbase_metres,
            max_angle: stored.max_wheel_angle_radians,
            max_rate: stored.max_steering_rate_radians_per_second,
            antenna: stored.antenna,
        })
    }
}

#[cfg(feature = "serde")]
impl From<VehicleGeometry> for StoredGeometry {
    fn from(geometry: VehicleGeometry) -> Self {
        Self {
            wheelbase_metres: geometry.wheelbase,
            max_wheel_angle_radians: geometry.max_angle,
            max_steering_rate_radians_per_second: geometry.max_rate,
            antenna: geometry.antenna,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{AbLine, FieldPlane};
    use kinavis_kernel::{Height, Position};

    fn geometry() -> Result<VehicleGeometry> {
        VehicleGeometry::new(
            Distance::from_metres(2.9)?,
            Angle::from_degrees(35.0)?,
            RateOfTurn::from_radians_per_second(math::to_radians(30.0))?,
        )
    }

    #[test]
    fn the_bicycle_turns_on_its_arc() -> Result<()> {
        let vehicle = geometry()?;
        let state = VehicleState {
            position: PlanarPoint::ORIGIN,
            heading: GridCourse::new(0.0)?,
            speed: Speed::from_metres_per_second(2.0)?,
        };
        let wheel = Angle::from_degrees(20.0)?;
        let radius = 2.9 / math::tan(math::to_radians(20.0));
        // A quarter turn right about the centre `radius` to the east.
        let quarter = core::f64::consts::FRAC_PI_2 * radius / 2.0;
        let after = bicycle_step(&state, &vehicle, wheel, Duration::from_secs_f64(quarter))?;
        assert!((after.position.east_metres() - radius).abs() < 1e-9);
        assert!((after.position.north_metres() - radius).abs() < 1e-9);
        // The step is whole nanoseconds: 1 ns at 2 m/s turns 1.4·10⁻⁸°.
        assert!((after.heading.degrees() - 90.0).abs() < 1e-7);
        Ok(())
    }

    #[test]
    fn the_wheels_slew_at_their_rate_within_their_limit() -> Result<()> {
        let vehicle = geometry()?;
        let reached = vehicle.slew(
            Angle::ZERO,
            Angle::from_degrees(30.0)?,
            Duration::from_millis(500),
        );
        assert!((reached.degrees() - 15.0).abs() < 1e-9);
        let held = vehicle.command(math::to_radians(50.0));
        assert!((held.angle.degrees() - 35.0).abs() < 1e-9);
        assert!(
            (vehicle.min_turning_radius().metres() - 2.9 / math::tan(math::to_radians(35.0))).abs()
                < 1e-9
        );
        Ok(())
    }

    #[test]
    fn both_trackers_steer_towards_the_pass() -> Result<()> {
        let plane = FieldPlane::centred_at(
            Position::from_degrees(50.0, 10.0)?,
            Height::above_ellipsoid(Distance::ZERO),
        )?;
        let line = AbLine::from_point_and_course(plane, PlanarPoint::ORIGIN, GridCourse::new(0.0)?);
        let pattern = PassPattern::new(Distance::from_metres(10.0)?, Distance::ZERO)?;
        // Heading north 1 m east of the reference pass: steer left.
        let state = VehicleState {
            position: PlanarPoint::from_metres(1.0, 0.0)?,
            heading: GridCourse::new(0.0)?,
            speed: Speed::from_metres_per_second(3.0)?,
        };
        let vehicle = geometry()?;
        let pursuit = PurePursuit::new(
            Distance::from_metres(4.0)?,
            Duration::from_millis(500),
            Distance::from_metres(12.0)?,
        )?;
        let stanley = Stanley::new(1.0, Speed::from_metres_per_second(0.5)?)?;
        for command in [
            pursuit.steer(&line, &pattern, PassIndex::REFERENCE, &vehicle, &state)?,
            stanley.steer(&line, &pattern, PassIndex::REFERENCE, &vehicle, &state)?,
        ] {
            assert!(command.angle.degrees() < 0.0 && command.curvature < 0.0);
        }
        // Pure pursuit, closed form: look-ahead 5.5 m, sin α = −1/5.5.
        let command = pursuit.steer(&line, &pattern, PassIndex::REFERENCE, &vehicle, &state)?;
        let expected = math::atan(2.9 * 2.0 * (-1.0 / 5.5) / 5.5);
        assert!((command.angle.radians() - expected).abs() < 1e-9);
        // Facing south and backing north from the same place: the mirror
        // image, the wheels turned the other way.
        let reversing = VehicleState {
            heading: GridCourse::new(180.0)?,
            speed: Speed::from_metres_per_second(-3.0)?,
            ..state
        };
        for command in [
            pursuit.steer(&line, &pattern, PassIndex::REFERENCE, &vehicle, &reversing)?,
            stanley.steer(&line, &pattern, PassIndex::REFERENCE, &vehicle, &reversing)?,
        ] {
            assert!(command.angle.degrees() > 0.0 && command.curvature > 0.0);
        }
        let command = pursuit.steer(&line, &pattern, PassIndex::REFERENCE, &vehicle, &reversing)?;
        assert!((command.angle.radians() + expected).abs() < 1e-9);
        let too_fast = VehicleState {
            speed: Speed::from_metres_per_second(-101.0)?,
            ..state
        };
        assert!(stanley
            .steer(&line, &pattern, PassIndex::REFERENCE, &vehicle, &too_fast)
            .is_err());
        Ok(())
    }
}
