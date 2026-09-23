//! Leg-to-leg turns: the wheel-over point.
//!
//! A vessel turns on a circle tangent to both legs, so to be on the new leg
//! when the turn ends it must start before the waypoint. The *lead* is the
//! tangent length `R · tan(Δ/2)` for alteration `Δ`, plus the *reach* — the
//! straight run between helm order and response.
//!
//! The circle is planned by radius or by rate of turn (radius derived from the
//! turning speed). The reach comes from the pilot card: the transfer is the
//! turning radius at that rudder, advance − transfer is the run before the turn
//! takes effect, and the transfer is also the tightest plannable radius.
//!
//! ```rust
//! use kinavis::route::{LegKind, Route};
//! use kinavis::turning::{wheel_over_point, TurnMode, TurnParameters};
//! use kinavis::{Distance, Position, Speed};
//!
//! // North for thirty miles, then east: a ninety-degree turn to starboard.
//! let route = Route::new(
//!     &[
//!         "50°00.0'N 001°00.0'W".parse::<Position>()?,
//!         "50°30.0'N 001°00.0'W".parse::<Position>()?,
//!         "50°30.0'N 000°00.0'W".parse::<Position>()?,
//!     ],
//!     LegKind::RhumbLine,
//! )?;
//! // A pilot card: 0.9 M advance and 0.7 M transfer for a 90° turn.
//! let parameters = TurnParameters::new(
//!     TurnMode::Radius(Distance::from_nautical_miles(1.0)?),
//!     Distance::from_cables(9.0)?,
//!     Distance::from_cables(7.0)?,
//! )?;
//!
//! let turn = wheel_over_point(&route, 1, &parameters, Speed::from_knots(12.0)?)?;
//! assert_eq!(format!("{:.0}", turn.alteration().degrees()), "90");
//! // Tangent of a 1 NM circle through 90° is 1 NM, plus 0.2 NM reach:
//! // wheel over 1.2 NM before the waypoint.
//! assert_eq!(format!("{:.1}", turn.lead().nautical_miles()), "1.2");
//! assert_eq!(format!("{:.1}", turn.rate_of_turn()), "11.5°/min");
//! assert!(turn.wheel_over().latitude().degrees() < 50.5);
//! # Ok::<(), kinavis::NavigationError>(())
//! ```

use crate::angle::TrueCourse;
use crate::error::{ensure_range, KernelError, NavigationError, Result};
use crate::math;
use crate::position::Position;
use crate::route::{LegKind, Route};
use crate::sailings::{
    great_circle, great_circle_destination, rhumb_destination, rhumb_line, Sailing,
};
use crate::units::{Angle, Distance, RateOfTurn, Speed};

/// Turn planning mode: radius or rate.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[derive(Debug, Clone, Copy, PartialEq)]
#[non_exhaustive]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum TurnMode {
    /// Fixed radius regardless of speed, as specified on a chart-planned route.
    Radius(Distance),
    /// Fixed rate of turn; radius follows from speed, so the circle widens at
    /// higher speed.
    RateOfTurn(RateOfTurn),
}

/// Turn plan and pilot-card figures.
///
/// Validated by [`TurnParameters::new`]: no circle tighter than the ship can
/// make, no self-contradictory pilot card.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(try_from = "StoredTurnParameters", into = "StoredTurnParameters")
)]
pub struct TurnParameters {
    mode: TurnMode,
    advance: Distance,
    transfer: Distance,
}

impl TurnParameters {
    /// Turn planned in `mode` for a ship whose 90° turning circle at the
    /// intended rudder has this `advance` (distance along the original course)
    /// and `transfer` (sideways displacement). The transfer is the turning
    /// radius and the tightest plannable radius. Use [`Distance::ZERO`] for
    /// both without a pilot card: the turn is then a pure arc.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] if either figure is negative, advance <
    /// transfer, or a planned radius is not positive or tighter than the
    /// transfer. A rate of turn is checked against the transfer at turning
    /// speed by [`wheel_over_point`].
    pub fn new(mode: TurnMode, advance: Distance, transfer: Distance) -> Result<Self> {
        ensure_range("transfer", transfer.nautical_miles(), 0.0, f64::MAX)?;
        ensure_range(
            "advance",
            advance.nautical_miles(),
            transfer.nautical_miles(),
            f64::MAX,
        )?;
        if let TurnMode::Radius(radius) = mode {
            ensure_range(
                "turn radius",
                radius.nautical_miles(),
                tightest_radius(transfer),
                f64::MAX,
            )?;
        }
        Ok(Self {
            mode,
            advance,
            transfer,
        })
    }

    /// Planned circle.
    #[must_use]
    pub const fn mode(&self) -> TurnMode {
        self.mode
    }

    /// 90° advance.
    #[must_use]
    pub const fn advance(&self) -> Distance {
        self.advance
    }

    /// 90° transfer: the turning radius.
    #[must_use]
    pub const fn transfer(&self) -> Distance {
        self.transfer
    }
}

/// Minimum plannable radius: the transfer, never zero.
fn tightest_radius(transfer: Distance) -> f64 {
    transfer.nautical_miles().max(f64::MIN_POSITIVE)
}

/// Serialised form; deserialisation goes through [`TurnParameters::new`].
#[cfg(feature = "serde")]
#[derive(serde::Serialize, serde::Deserialize)]
struct StoredTurnParameters {
    mode: TurnMode,
    advance: Distance,
    transfer: Distance,
}

#[cfg(feature = "serde")]
impl TryFrom<StoredTurnParameters> for TurnParameters {
    type Error = NavigationError;

    fn try_from(stored: StoredTurnParameters) -> Result<Self> {
        Self::new(stored.mode, stored.advance, stored.transfer)
    }
}

#[cfg(feature = "serde")]
impl From<TurnParameters> for StoredTurnParameters {
    fn from(parameters: TurnParameters) -> Self {
        Self {
            mode: parameters.mode,
            advance: parameters.advance,
            transfer: parameters.transfer,
        }
    }
}

/// Computed turn at a waypoint: wheel-over point and where the new leg is
/// joined.
///
/// Built by [`wheel_over_point`].
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Turn {
    waypoint_index: u16,
    waypoint: Position,
    inbound: TrueCourse,
    outbound: TrueCourse,
    alteration: Angle,
    radius: Distance,
    rate: RateOfTurn,
    reach: Distance,
    tangent: Distance,
    wheel_over: Position,
    joins_at: Position,
}

impl Turn {
    /// Waypoint index, zero-based.
    #[must_use]
    pub const fn waypoint_index(&self) -> u16 {
        self.waypoint_index
    }

    /// Waypoint position (leg junction).
    #[must_use]
    pub const fn waypoint(&self) -> Position {
        self.waypoint
    }

    /// Inbound leg course on arrival.
    #[must_use]
    pub const fn inbound(&self) -> TrueCourse {
        self.inbound
    }

    /// Outbound leg course on departure.
    #[must_use]
    pub const fn outbound(&self) -> TrueCourse {
        self.outbound
    }

    /// Alteration, positive to starboard.
    #[must_use]
    pub const fn alteration(&self) -> Angle {
        self.alteration
    }

    /// Turning radius.
    #[must_use]
    pub const fn radius(&self) -> Distance {
        self.radius
    }

    /// Rate of turn at the computed speed, signed by direction.
    #[must_use]
    pub const fn rate_of_turn(&self) -> RateOfTurn {
        self.rate
    }

    /// Reach: straight run after wheel-over before the turn takes (advance −
    /// transfer).
    #[must_use]
    pub const fn reach(&self) -> Distance {
        self.reach
    }

    /// Tangent length: distance before the waypoint where the arc leaves the
    /// inbound leg, and after it where it joins the outbound leg.
    #[must_use]
    pub const fn tangent(&self) -> Distance {
        self.tangent
    }

    /// Wheel-over distance before the waypoint along the inbound leg: tangent +
    /// reach.
    #[must_use]
    pub fn lead(&self) -> Distance {
        self.tangent + self.reach
    }

    /// Wheel-over position.
    #[must_use]
    pub const fn wheel_over(&self) -> Position {
        self.wheel_over
    }

    /// End of turn: where the arc joins the outbound leg.
    #[must_use]
    pub const fn end_of_turn(&self) -> Position {
        self.joins_at
    }

    /// Arc length.
    #[must_use]
    pub fn arc_length(&self) -> Distance {
        Distance::from_nautical_miles_unchecked(
            self.radius.nautical_miles() * math::to_radians(math::abs(self.alteration.degrees())),
        )
    }

    /// Distance saved versus the two tangents (the cut corner), for schedule
    /// corrections.
    #[must_use]
    pub fn distance_saved(&self) -> Distance {
        self.tangent + self.tangent - self.arc_length()
    }
}

/// Turn at `waypoint` of `route` for a vessel making `speed` over ground in the
/// turn.
///
/// Speed determines the radius in [`TurnMode::RateOfTurn`] and the rate in
/// [`TurnMode::Radius`]; a stationary vessel has zero rate.
///
/// # Errors
///
/// - [`KernelError::OutOfRange`] for the first or last waypoint (nothing to
///   turn onto); if a rate-of-turn radius at this speed is tighter than the
///   transfer; or if the lead or tangent exceeds the leg it lies on (the turn
///   cannot be executed as planned).
/// - [`KernelError::Indeterminate`] for a 180° turn (zero-length tangent) or a
///   zero rate of turn.
/// - Sailing failures, notably a rhumb leg through a pole.
pub fn wheel_over_point(
    route: &Route,
    waypoint: u16,
    parameters: &TurnParameters,
    speed: Speed,
) -> Result<Turn> {
    let index = usize::from(waypoint);
    let waypoints = route.waypoints();
    let (before, at, after) = match (
        index.checked_sub(1).and_then(|i| waypoints.get(i)),
        waypoints.get(index),
        waypoints.get(index + 1),
    ) {
        (Some(before), Some(at), Some(after)) => (*before, *at, *after),
        _ => {
            return Err(NavigationError::Kernel(KernelError::OutOfRange {
                parameter: "turn waypoint",
                value: f64::from(waypoint),
                min: 1.0,
                max: math::count_to_f64(waypoints.len().saturating_sub(2)),
            }))
        }
    };

    let (radius, reach) = circle(parameters, speed)?;

    let kind = route.kind();
    let inbound_leg = sail(kind, before, at)?;
    let outbound_leg = sail(kind, at, after)?;
    let inbound = inbound_leg.final_course;
    let outbound = outbound_leg.initial_course;
    let alteration = inbound.signed_difference(outbound);
    if math::abs(alteration) >= 180.0 - 1e-6 {
        return Err(NavigationError::Kernel(KernelError::Indeterminate {
            quantity: "the wheel-over point of a turn through 180°",
        }));
    }

    let tangent = radius.nautical_miles() * math::tan(math::to_radians(alteration / 2.0));
    let tangent = math::abs(tangent);
    let lead = tangent + reach.nautical_miles();
    if lead > inbound_leg.distance.nautical_miles() {
        return Err(NavigationError::Kernel(KernelError::OutOfRange {
            parameter: "turn lead",
            value: lead,
            min: 0.0,
            max: inbound_leg.distance.nautical_miles(),
        }));
    }
    if tangent > outbound_leg.distance.nautical_miles() {
        return Err(NavigationError::Kernel(KernelError::OutOfRange {
            parameter: "turn tangent",
            value: tangent,
            min: 0.0,
            max: outbound_leg.distance.nautical_miles(),
        }));
    }

    let unsigned_rate = RateOfTurn::around(radius, speed)?;
    let rate = if alteration < 0.0 {
        -unsigned_rate
    } else {
        unsigned_rate
    };

    Ok(Turn {
        waypoint_index: waypoint,
        waypoint: at,
        inbound,
        outbound,
        alteration: Angle::from_degrees_unchecked(alteration),
        radius,
        rate,
        reach,
        tangent: Distance::from_nautical_miles_unchecked(tangent),
        wheel_over: destination(
            kind,
            at,
            inbound.reciprocal(),
            Distance::from_nautical_miles_unchecked(lead),
        )?,
        joins_at: destination(
            kind,
            at,
            outbound,
            Distance::from_nautical_miles_unchecked(tangent),
        )?,
    })
}

/// Turning radius and reach from the parameters and speed; the radius is
/// checked against the pilot card where it depends on speed.
fn circle(parameters: &TurnParameters, speed: Speed) -> Result<(Distance, Distance)> {
    // Both figures non-negative, advance ≥ transfer: guaranteed by
    // `TurnParameters::new`.
    let reach = parameters.advance.nautical_miles() - parameters.transfer.nautical_miles();
    let radius = match parameters.mode {
        // Checked against the transfer at construction.
        TurnMode::Radius(radius) => radius,
        TurnMode::RateOfTurn(rate) => {
            let radius = rate.radius_at(speed)?;
            ensure_range(
                "turn radius",
                radius.nautical_miles(),
                tightest_radius(parameters.transfer),
                f64::MAX,
            )?;
            radius
        }
    };
    Ok((radius, Distance::from_nautical_miles_unchecked(reach)))
}

/// Sailing between two points of the route's leg type.
fn sail(kind: LegKind, from: Position, to: Position) -> Result<Sailing> {
    match kind {
        LegKind::RhumbLine => rhumb_line(from, to),
        LegKind::GreatCircle => great_circle(from, to),
    }
}

/// Point at a distance along a course, on the route's leg type. Back along the
/// reciprocal of a leg's final course is the same leg for either type.
fn destination(
    kind: LegKind,
    from: Position,
    course: TrueCourse,
    distance: Distance,
) -> Result<Position> {
    match kind {
        LegKind::RhumbLine => rhumb_destination(from, course, distance),
        LegKind::GreatCircle => Ok(great_circle_destination(from, course, distance)?.position),
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use crate::sailings::cross_track;

    fn at(latitude: f64, longitude: f64) -> Position {
        Position::from_degrees(latitude, longitude).unwrap()
    }

    /// 60 NM north, 60 NM east, 60 NM south.
    fn square() -> Route {
        Route::new(
            &[at(0.0, 0.0), at(1.0, 0.0), at(1.0, 1.0), at(0.0, 1.0)],
            LegKind::RhumbLine,
        )
        .unwrap()
    }

    fn radius(miles: f64) -> TurnParameters {
        TurnParameters::new(
            TurnMode::Radius(Distance::from_nautical_miles(miles).unwrap()),
            Distance::ZERO,
            Distance::ZERO,
        )
        .unwrap()
    }

    fn miles(value: f64) -> Distance {
        Distance::from_nautical_miles(value).unwrap()
    }

    fn ten_knots() -> Speed {
        Speed::from_knots(10.0).unwrap()
    }

    #[test]
    fn a_right_angle_on_a_one_mile_circle_has_a_one_mile_tangent() {
        let turn = wheel_over_point(&square(), 1, &radius(1.0), ten_knots()).unwrap();
        assert_eq!(turn.waypoint_index(), 1);
        assert_eq!(turn.waypoint(), at(1.0, 0.0));
        assert!((turn.alteration().degrees() - 90.0).abs() < 1e-6);
        assert!((turn.tangent().nautical_miles() - 1.0).abs() < 1e-9);
        assert_eq!(turn.reach(), Distance::ZERO);
        assert!((turn.lead().nautical_miles() - 1.0).abs() < 1e-9);
        assert!((turn.arc_length().nautical_miles() - core::f64::consts::FRAC_PI_2).abs() < 1e-6);
        assert!(
            (turn.distance_saved().nautical_miles() - (2.0 - core::f64::consts::FRAC_PI_2)).abs()
                < 1e-6
        );

        // Wheel-over 1 NM south of the waypoint on the leg; turn ends 1 NM east
        // of it on the next.
        let wheel_over = turn.wheel_over();
        let back = rhumb_line(wheel_over, at(1.0, 0.0)).unwrap();
        assert!((back.distance.nautical_miles() - 1.0).abs() < 1e-9);
        assert!(back.initial_course.degrees().abs() < 1e-9);
        assert!(wheel_over.longitude().degrees().abs() < 1e-9);
        let end = turn.end_of_turn();
        let on = rhumb_line(at(1.0, 0.0), end).unwrap();
        assert!((on.distance.nautical_miles() - 1.0).abs() < 1e-9);
        assert!((on.initial_course.degrees() - 90.0).abs() < 1e-9);
    }

    #[test]
    fn the_reach_moves_the_wheel_over_point_back_but_not_the_arc() {
        let mut parameters = radius(1.0);
        parameters.advance = Distance::from_cables(9.0).unwrap();
        parameters.transfer = Distance::from_cables(7.0).unwrap();
        let turn = wheel_over_point(&square(), 1, &parameters, ten_knots()).unwrap();
        assert!((turn.reach().nautical_miles() - 0.2).abs() < 1e-9);
        assert!((turn.tangent().nautical_miles() - 1.0).abs() < 1e-9);
        assert!((turn.lead().nautical_miles() - 1.2).abs() < 1e-9);
        let back = rhumb_line(turn.wheel_over(), at(1.0, 0.0)).unwrap();
        assert!((back.distance.nautical_miles() - 1.2).abs() < 1e-9);
    }

    #[test]
    fn a_turn_by_rate_widens_with_speed() {
        let parameters = TurnParameters::new(
            TurnMode::RateOfTurn(RateOfTurn::from_degrees_per_minute(10.0).unwrap()),
            Distance::ZERO,
            Distance::ZERO,
        )
        .unwrap();
        let slow = wheel_over_point(&square(), 1, &parameters, ten_knots()).unwrap();
        let fast =
            wheel_over_point(&square(), 1, &parameters, Speed::from_knots(20.0).unwrap()).unwrap();
        assert!((slow.radius().nautical_miles() - 6.0 / core::f64::consts::TAU).abs() < 1e-9);
        assert!(
            (fast.radius().nautical_miles() - 2.0 * slow.radius().nautical_miles()).abs() < 1e-9
        );
        assert!((slow.rate_of_turn().degrees_per_minute() - 10.0).abs() < 1e-9);
        assert!((fast.rate_of_turn().degrees_per_minute() - 10.0).abs() < 1e-9);
    }

    #[test]
    fn a_turn_by_radius_reports_the_rate_it_needs() {
        let turn = wheel_over_point(&square(), 1, &radius(1.0), ten_knots()).unwrap();
        // 10 kn on a 1 NM circle: 10 rad/h = 9.55°/min.
        assert!(
            (turn.rate_of_turn().degrees_per_minute() - 10.0_f64.to_degrees() / 60.0).abs() < 1e-9
        );
        assert!(!turn.rate_of_turn().is_to_port());

        // Stopped: zero rate, a value, not an error.
        let stopped = wheel_over_point(&square(), 1, &radius(1.0), Speed::ZERO).unwrap();
        assert_eq!(stopped.rate_of_turn(), RateOfTurn::ZERO);
    }

    #[test]
    fn a_port_turn_is_signed_to_port() {
        // East then north: 90° to port.
        let route = Route::new(
            &[at(0.0, 0.0), at(0.0, 1.0), at(1.0, 1.0)],
            LegKind::RhumbLine,
        )
        .unwrap();
        let turn = wheel_over_point(&route, 1, &radius(1.0), ten_knots()).unwrap();
        assert!((turn.alteration().degrees() + 90.0).abs() < 1e-6);
        assert!(turn.rate_of_turn().is_to_port());
        assert!((turn.tangent().nautical_miles() - 1.0).abs() < 1e-9);
        // Wheel-over 1 NM west of the waypoint.
        let back = rhumb_line(turn.wheel_over(), at(0.0, 1.0)).unwrap();
        assert!((back.distance.nautical_miles() - 1.0).abs() < 1e-9);
        assert!((back.initial_course.degrees() - 90.0).abs() < 1e-9);
    }

    #[test]
    fn a_small_alteration_has_a_short_tangent() {
        let route = Route::new(
            &[at(0.0, 0.0), at(1.0, 0.0), at(2.0, 0.2)],
            LegKind::RhumbLine,
        )
        .unwrap();
        let turn = wheel_over_point(&route, 1, &radius(2.0), ten_knots()).unwrap();
        let expected = 2.0 * (turn.alteration().degrees() / 2.0).to_radians().tan();
        assert!((turn.tangent().nautical_miles() - expected).abs() < 1e-9);
        assert!(turn.tangent().nautical_miles() < 0.25);
    }

    #[test]
    fn a_great_circle_route_turns_on_the_courses_at_the_waypoint() {
        let route = Route::new(
            &[at(40.0, -70.0), at(50.0, -30.0), at(50.0, 0.0)],
            LegKind::GreatCircle,
        )
        .unwrap();
        let turn = wheel_over_point(&route, 1, &radius(2.0), ten_knots()).unwrap();
        let inbound = great_circle(at(40.0, -70.0), at(50.0, -30.0)).unwrap();
        let outbound = great_circle(at(50.0, -30.0), at(50.0, 0.0)).unwrap();
        assert_eq!(turn.inbound(), inbound.final_course);
        assert_eq!(turn.outbound(), outbound.initial_course);
        // Wheel-over lies on the inbound great circle.
        assert!(
            cross_track(turn.wheel_over(), at(40.0, -70.0), at(50.0, -30.0))
                .unwrap()
                .distance
                .nautical_miles()
                < 1e-6
        );
    }

    #[test]
    fn the_ends_of_the_route_have_nothing_to_turn_onto() {
        assert!(matches!(
            wheel_over_point(&square(), 0, &radius(1.0), ten_knots()).unwrap_err(),
            NavigationError::Kernel(KernelError::OutOfRange {
                parameter: "turn waypoint",
                ..
            })
        ));
        assert!(wheel_over_point(&square(), 3, &radius(1.0), ten_knots()).is_err());
        assert!(wheel_over_point(&square(), u16::MAX, &radius(1.0), ten_knots()).is_err());
    }

    #[test]
    fn a_circle_tighter_than_she_can_turn_is_refused() {
        // A planned radius is checked at construction...
        let half_mile = TurnMode::Radius(miles(0.5));
        assert!(matches!(
            TurnParameters::new(half_mile, miles(0.8), miles(0.7)).unwrap_err(),
            NavigationError::Kernel(KernelError::OutOfRange {
                parameter: "turn radius",
                ..
            })
        ));
        assert!(
            TurnParameters::new(TurnMode::Radius(Distance::ZERO), miles(0.0), miles(0.0)).is_err()
        );
        assert!(
            TurnParameters::new(TurnMode::Radius(miles(-1.0)), miles(0.0), miles(0.0)).is_err()
        );

        // ...and a rate-derived radius at turning speed: 10°/min at 10 kn is
        // 0.95 NM, tighter than a 1 NM transfer.
        let by_rate = TurnParameters::new(
            TurnMode::RateOfTurn(RateOfTurn::from_degrees_per_minute(10.0).unwrap()),
            miles(1.2),
            miles(1.0),
        )
        .unwrap();
        assert!(matches!(
            wheel_over_point(&square(), 1, &by_rate, ten_knots()).unwrap_err(),
            NavigationError::Kernel(KernelError::OutOfRange {
                parameter: "turn radius",
                ..
            })
        ));
        // At higher speed the circle is wider and the same parameters are
        // valid.
        let twenty_knots = Speed::from_knots(20.0).unwrap();
        assert!(wheel_over_point(&square(), 1, &by_rate, twenty_knots).is_ok());
    }

    #[test]
    fn a_pilot_card_with_more_transfer_than_advance_is_refused() {
        let one_mile = TurnMode::Radius(miles(1.0));
        assert!(matches!(
            TurnParameters::new(one_mile, miles(0.5), miles(0.7)).unwrap_err(),
            NavigationError::Kernel(KernelError::OutOfRange {
                parameter: "advance",
                ..
            })
        ));
        assert!(TurnParameters::new(one_mile, miles(-0.5), miles(-0.7)).is_err());
        assert!(TurnParameters::new(one_mile, miles(0.5), miles(-0.7)).is_err());
        let nan = Distance::from_nautical_miles_unchecked(f64::NAN);
        assert!(TurnParameters::new(one_mile, nan, miles(0.7)).is_err());
    }

    #[test]
    fn a_turn_longer_than_its_legs_is_a_route_that_cannot_be_turned() {
        // A 60 NM leg cannot hold the tangent of a 100 NM circle.
        assert!(matches!(
            wheel_over_point(&square(), 1, &radius(100.0), ten_knots()).unwrap_err(),
            NavigationError::Kernel(KernelError::OutOfRange {
                parameter: "turn lead",
                ..
            })
        ));
        // Nor can a short outbound leg.
        let route = Route::new(
            &[at(0.0, 0.0), at(1.0, 0.0), at(1.0, 0.01)],
            LegKind::RhumbLine,
        )
        .unwrap();
        assert!(matches!(
            wheel_over_point(&route, 1, &radius(1.0), ten_knots()).unwrap_err(),
            NavigationError::Kernel(KernelError::OutOfRange {
                parameter: "turn tangent",
                ..
            })
        ));
    }

    #[test]
    fn a_turn_by_rate_needs_a_rate_and_a_reversal_has_no_tangent() {
        let parameters = TurnParameters::new(
            TurnMode::RateOfTurn(RateOfTurn::ZERO),
            Distance::ZERO,
            Distance::ZERO,
        )
        .unwrap();
        assert!(wheel_over_point(&square(), 1, &parameters, ten_knots()).is_err());

        let back_and_forth = Route::new(
            &[at(0.0, 0.0), at(1.0, 0.0), at(0.0, 0.0)],
            LegKind::RhumbLine,
        )
        .unwrap();
        assert!(matches!(
            wheel_over_point(&back_and_forth, 1, &radius(1.0), ten_knots()).unwrap_err(),
            NavigationError::Kernel(KernelError::Indeterminate { .. })
        ));
    }
}
