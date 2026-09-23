//! Squat and under-keel clearance.
//!
//! Depth of water = charted depth + height of tide. Subtract the static draught
//! and the squat — the bodily sinkage and trim of a hull moving in shallow
//! water, growing with the square of speed — to get the under-keel clearance
//! (UKC), which the vessel's policy bounds from below.
//!
//! # Model
//!
//! Barrass squat, speed through the water `V` in kn, block coefficient `Cb`,
//! result in m:
//!
//! - open water: `Cb · V² / 100`;
//! - confined channel: `Cb · V² / 50`;
//! - general: `Cb · S^0.81 · V^2.08 / 20`, blockage factor `S` = midship
//!   section / channel cross-section; the two rules above correspond to `S ≈
//!   0.1` and `S ≈ 0.25`.
//!
//! These are maximum squats (bow or stern, whichever is deeper), as UKC
//! requires. Empirical, accurate to about 0.1 m for conventional hulls and
//! speeds; a computed metre of UKC is real only if chart and tide are also
//! correct.
//!
//! ```rust
//! use kinavis::clearance::{under_keel_clearance, ClearancePolicy, Hull, Waterway};
//! use kinavis::{
//!     Civil, Distance, EnvironmentSample, GeodeticPoint, Height, Instant, ClearanceEvent,
//!     Position, Speed, Utc,
//! };
//!
//! // Laden tanker: 12 m draught, block coefficient 0.85. Policy: UKC at
//! // least 10 % of draught and at least 1 m.
//! let hull = Hull::new(Distance::from_metres(12.0)?, 0.85)?;
//! let policy = ClearancePolicy {
//!     minimum: Distance::from_metres(1.0)?,
//!     fraction_of_draught: 0.1,
//! };
//!
//! // Fourteen metres on the chart, a metre and a half of tide.
//! let here: Position = "51°22.0'N 003°00.0'E".parse()?;
//! let when = Instant::<Utc>::from_civil(Civil { hour: 9, ..Civil::date(2026, 9, 17) })?;
//! let environment = EnvironmentSample::at(
//!     GeodeticPoint::new(here, Height::above_ellipsoid(Distance::ZERO)),
//!     when,
//! )
//! .with_tide(Distance::from_metres(1.5)?);
//! let charted = Distance::from_metres(14.0)?;
//!
//! // At 12 kn in open water: squat 1.2 m, 2.3 m margin.
//! let (open, events) = under_keel_clearance(
//!     &hull, Speed::from_knots(12.0)?, Waterway::OpenWater, charted, &environment, &policy,
//! )?;
//! assert_eq!(format!("{:.2}", open.squat().metres()), "1.22");
//! assert_eq!(format!("{:.2}", open.clearance().metres()), "2.28");
//! assert!(open.is_sufficient());
//! assert!(events.is_empty());
//!
//! // The same speed in the channel doubles the squat, and that is too close.
//! let (channel, events) = under_keel_clearance(
//!     &hull, Speed::from_knots(12.0)?, Waterway::ConfinedChannel, charted, &environment, &policy,
//! )?;
//! assert_eq!(format!("{:.2}", channel.clearance().metres()), "1.05");
//! assert!(!channel.is_sufficient());
//! assert!(matches!(events[0], ClearanceEvent::UnderKeelClearanceLow { .. }));
//! # Ok::<(), kinavis::NavigationError>(())
//! ```

use crate::environment::EnvironmentSample;
use crate::error::{ensure_range, KernelError, NavigationError, Result};
use crate::event::{ClearanceEvent, EventList};
use crate::math;
use crate::time::{Instant, Utc};
use crate::units::{Distance, Speed};

/// Hull draught and fullness.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(try_from = "StoredHull", into = "StoredHull"))]
pub struct Hull {
    draught: Distance,
    block_coefficient: f64,
}

impl Hull {
    /// Hull with static draught (deepest, at rest) and block coefficient (≈ 0.4
    /// fine yacht to ≥ 0.85 laden tanker).
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] if the draught is not positive or the block
    /// coefficient is not in `(0, 1]`.
    pub fn new(draught: Distance, block_coefficient: f64) -> Result<Self> {
        ensure_range("draught", draught.metres(), f64::MIN_POSITIVE, f64::MAX)?;
        ensure_range(
            "block coefficient",
            block_coefficient,
            f64::MIN_POSITIVE,
            1.0,
        )?;
        Ok(Self {
            draught,
            block_coefficient,
        })
    }

    /// Static draught.
    #[must_use]
    pub const fn draught(&self) -> Distance {
        self.draught
    }

    /// Block coefficient.
    #[must_use]
    pub const fn block_coefficient(&self) -> f64 {
        self.block_coefficient
    }
}

/// Serialised form; deserialisation checks the invariants.
#[cfg(feature = "serde")]
#[derive(serde::Serialize, serde::Deserialize)]
struct StoredHull {
    draught: Distance,
    block_coefficient: f64,
}

#[cfg(feature = "serde")]
impl TryFrom<StoredHull> for Hull {
    type Error = NavigationError;

    fn try_from(stored: StoredHull) -> Result<Self> {
        Self::new(stored.draught, stored.block_coefficient)
    }
}

#[cfg(feature = "serde")]
impl From<Hull> for StoredHull {
    fn from(hull: Hull) -> Self {
        Self {
            draught: hull.draught,
            block_coefficient: hull.block_coefficient,
        }
    }
}

/// Waterway confinement.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[derive(Debug, Clone, Copy, PartialEq)]
#[non_exhaustive]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Waterway {
    /// Shallow open water: only the bottom restricts the flow.
    OpenWater,
    /// Confined channel: dredged channel, canal, river.
    ConfinedChannel,
    /// Intermediate, by blockage factor: midship section / channel
    /// cross-section, in `(0, 1)`.
    Blockage(f64),
}

/// Minimum UKC policy.
///
/// Set by the vessel, owner or port as a fixed margin, a fraction of draught,
/// or the greater of both (the usual form). The required UKC is the greater.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ClearancePolicy {
    /// Fixed minimum.
    pub minimum: Distance,
    /// Minimum as a fraction of static draught (commonly 0.1).
    pub fraction_of_draught: f64,
}

impl ClearancePolicy {
    /// Required UKC for a hull: max(fixed margin, fraction × draught).
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] if either margin is negative;
    /// [`KernelError::NotFinite`] if the fraction is not finite.
    pub fn required(&self, hull: &Hull) -> Result<Distance> {
        ensure_range("minimum clearance", self.minimum.metres(), 0.0, f64::MAX)?;
        ensure_range(
            "fraction of draught",
            self.fraction_of_draught,
            0.0,
            f64::MAX,
        )?;
        let by_draught = hull.draught.metres() * self.fraction_of_draught;
        Ok(Distance::from_metres(
            self.minimum.metres().max(by_draught),
        )?)
    }
}

/// Barrass squat at a speed through the water.
///
/// Sign of speed is ignored (squat also occurs going astern). Formulas in the
/// module docs.
///
/// # Errors
///
/// - [`KernelError::OutOfRange`] for a blockage factor outside `(0, 1)`.
/// - [`KernelError::NotFinite`] if the speed makes the squat unrepresentable.
pub fn squat(hull: &Hull, speed: Speed, waterway: Waterway) -> Result<Distance> {
    let v = math::abs(speed.knots());
    let cb = hull.block_coefficient;
    let metres = match waterway {
        Waterway::OpenWater => cb * v * v / 100.0,
        Waterway::ConfinedChannel => cb * v * v / 50.0,
        Waterway::Blockage(factor) => {
            ensure_range("blockage factor", factor, f64::MIN_POSITIVE, 1.0)?;
            cb * power(factor, 0.81) * power(v, 2.08) / 20.0
        }
    };
    Ok(Distance::from_metres(metres)?)
}

/// UKC at one instant with its components.
///
/// Projection built by [`under_keel_clearance`]. Metres; negative UKC means
/// aground on paper.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Clearance {
    at: Instant<Utc>,
    charted_depth: Distance,
    height_of_tide: Distance,
    draught: Distance,
    squat: Distance,
    required: Distance,
}

impl Clearance {
    /// Time of the depth of water (the sample's).
    #[must_use]
    pub const fn at(&self) -> Instant<Utc> {
        self.at
    }

    /// Charted depth below chart datum; negative for a drying height.
    #[must_use]
    pub const fn charted_depth(&self) -> Distance {
        self.charted_depth
    }

    /// Height of tide above chart datum, from the sample.
    #[must_use]
    pub const fn height_of_tide(&self) -> Distance {
        self.height_of_tide
    }

    /// Depth of water: charted depth + height of tide.
    #[must_use]
    pub fn depth_of_water(&self) -> Distance {
        self.charted_depth + self.height_of_tide
    }

    /// Static draught.
    #[must_use]
    pub const fn draught(&self) -> Distance {
        self.draught
    }

    /// Squat at the assessed speed.
    #[must_use]
    pub const fn squat(&self) -> Distance {
        self.squat
    }

    /// Dynamic draught: static draught + squat.
    #[must_use]
    pub fn dynamic_draught(&self) -> Distance {
        self.draught + self.squat
    }

    /// UKC: depth of water − dynamic draught. Negative when aground.
    #[must_use]
    pub fn clearance(&self) -> Distance {
        self.depth_of_water() - self.dynamic_draught()
    }

    /// Required minimum.
    #[must_use]
    pub const fn required(&self) -> Distance {
        self.required
    }

    /// Margin over the requirement; negative when short.
    #[must_use]
    pub fn margin(&self) -> Distance {
        self.clearance() - self.required
    }

    /// Whether UKC meets the policy.
    #[must_use]
    pub fn is_sufficient(&self) -> bool {
        !self.margin().is_negative()
    }
}

/// UKC of `hull` at `speed` through the water over `charted_depth`, with the
/// sample's height of tide.
///
/// Pure: [`ClearanceEvent::UnderKeelClearanceLow`] is reported on every
/// evaluation below policy.
///
/// # Errors
///
/// - [`KernelError::Indeterminate`] if the sample has no height of tide (set it
///   to zero explicitly for non-tidal waters).
/// - Errors of [`squat`] and [`ClearancePolicy::required`].
pub fn under_keel_clearance(
    hull: &Hull,
    speed: Speed,
    waterway: Waterway,
    charted_depth: Distance,
    env: &EnvironmentSample,
    policy: &ClearancePolicy,
) -> Result<(Clearance, EventList<ClearanceEvent>)> {
    let height_of_tide = env
        .tide()
        .ok_or(NavigationError::Kernel(KernelError::Missing {
            what: "the height of tide",
        }))?;
    let clearance = Clearance {
        at: env.when(),
        charted_depth,
        height_of_tide,
        draught: hull.draught,
        squat: squat(hull, speed, waterway)?,
        required: policy.required(hull)?,
    };

    let mut events = EventList::new();
    if !clearance.is_sufficient() {
        events.push(ClearanceEvent::UnderKeelClearanceLow {
            clearance: clearance.clearance(),
            required: clearance.required,
            at: clearance.at,
        });
    }
    Ok((clearance, events))
}

/// Height of tide giving exactly the required UKC for `hull` at `speed` over
/// `charted_depth`; look it up with [`TidalCycle::time_of_height`] for the
/// earliest passing time. Negative when the charted depth alone suffices.
///
/// # Errors
///
/// Errors of [`squat`] and [`ClearancePolicy::required`].
///
/// [`TidalCycle::time_of_height`]: crate::tides::TidalCycle::time_of_height
pub fn height_of_tide_needed(
    hull: &Hull,
    speed: Speed,
    waterway: Waterway,
    charted_depth: Distance,
    policy: &ClearancePolicy,
) -> Result<Distance> {
    let needed = hull.draught + squat(hull, speed, waterway)? + policy.required(hull)?;
    Ok(needed - charted_depth)
}

/// `base^power` for a non-negative base.
///
/// No kernel power function; the exponents are non-integer, and `exp(p · ln x)`
/// is accurate enough for an empirical formula.
fn power(base: f64, exponent: f64) -> f64 {
    if base <= 0.0 {
        return 0.0;
    }
    math::exp(exponent * math::ln(base))
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp, clippy::indexing_slicing)]
mod tests {
    use super::*;
    use crate::conditions::Constant;
    use crate::environment::TideModel;
    use crate::geodesy::{GeodeticPoint, Height};
    use crate::position::Position;
    use crate::tides::{TidalCycle, TideEvent};

    fn metres(value: f64) -> Distance {
        Distance::from_metres(value).unwrap()
    }

    fn knots(value: f64) -> Speed {
        Speed::from_knots(value).unwrap()
    }

    fn noon() -> Instant<Utc> {
        Instant::from_unix_seconds(1_789_000_000)
    }

    /// Equal to the micrometre: distances are stored in NM and metres do not
    /// round-trip to the last bit.
    fn about(distance: Distance, metres: f64) -> bool {
        (distance.metres() - metres).abs() < 1e-6
    }

    fn tanker() -> Hull {
        Hull::new(metres(12.0), 0.85).unwrap()
    }

    fn tenth_of_draught() -> ClearancePolicy {
        ClearancePolicy {
            minimum: metres(1.0),
            fraction_of_draught: 0.1,
        }
    }

    fn sample(tide: Option<f64>) -> EnvironmentSample {
        let here = Position::from_degrees(51.0, 3.0).unwrap();
        let sample = EnvironmentSample::at(
            GeodeticPoint::new(here, Height::above_ellipsoid(Distance::ZERO)),
            noon(),
        );
        match tide {
            Some(height) => sample.with_tide(metres(height)),
            None => sample,
        }
    }

    #[test]
    fn squat_grows_with_the_square_of_the_speed_and_doubles_in_a_channel() {
        let hull = tanker();
        let open_six = squat(&hull, knots(6.0), Waterway::OpenWater).unwrap();
        let open_twelve = squat(&hull, knots(12.0), Waterway::OpenWater).unwrap();
        let channel_twelve = squat(&hull, knots(12.0), Waterway::ConfinedChannel).unwrap();

        assert!((open_twelve.metres() - 1.224).abs() < 1e-9);
        assert!((open_twelve.metres() / open_six.metres() - 4.0).abs() < 1e-9);
        assert!((channel_twelve.metres() / open_twelve.metres() - 2.0).abs() < 1e-9);
        assert_eq!(
            squat(&hull, Speed::ZERO, Waterway::OpenWater).unwrap(),
            Distance::ZERO
        );
    }

    #[test]
    fn sternway_squats_as_much_as_headway() {
        let hull = tanker();
        assert_eq!(
            squat(&hull, knots(-8.0), Waterway::OpenWater).unwrap(),
            squat(&hull, knots(8.0), Waterway::OpenWater).unwrap()
        );
    }

    #[test]
    fn the_general_formula_meets_the_two_rules_at_their_ends() {
        let hull = tanker();
        for v in [6.0, 10.0, 14.0] {
            let open = squat(&hull, knots(v), Waterway::OpenWater).unwrap();
            let at_tenth = squat(&hull, knots(v), Waterway::Blockage(0.1)).unwrap();
            assert!(
                (at_tenth.metres() / open.metres() - 1.0).abs() < 0.15,
                "{v} kn: {} vs {}",
                at_tenth.metres(),
                open.metres()
            );

            let confined = squat(&hull, knots(v), Waterway::ConfinedChannel).unwrap();
            let at_quarter = squat(&hull, knots(v), Waterway::Blockage(0.25)).unwrap();
            assert!(
                (at_quarter.metres() / confined.metres() - 1.0).abs() < 0.15,
                "{v} kn: {} vs {}",
                at_quarter.metres(),
                confined.metres()
            );
        }
    }

    #[test]
    fn a_blockage_factor_must_be_a_fraction_of_the_channel() {
        let hull = tanker();
        for factor in [0.0, -0.1, 1.5, f64::NAN, f64::INFINITY] {
            assert!(
                squat(&hull, knots(10.0), Waterway::Blockage(factor)).is_err(),
                "{factor}"
            );
        }
        assert!(squat(&hull, knots(10.0), Waterway::Blockage(1.0)).is_ok());
    }

    #[test]
    fn a_hull_has_a_draught_and_a_block_coefficient_a_hull_can_have() {
        assert!(Hull::new(Distance::ZERO, 0.8).is_err());
        assert!(Hull::new(metres(-3.0), 0.8).is_err());
        assert!(Hull::new(metres(3.0), 0.0).is_err());
        assert!(Hull::new(metres(3.0), 1.2).is_err());
        assert!(Hull::new(metres(3.0), f64::NAN).is_err());
        let hull = Hull::new(metres(3.0), 1.0).unwrap();
        assert_eq!(hull.draught(), metres(3.0));
        assert_eq!(hull.block_coefficient(), 1.0);
    }

    #[test]
    fn the_policy_keeps_the_greater_of_its_two_margins() {
        let hull = tanker();
        // 10 % of 12 m exceeds 1 m.
        assert!(about(tenth_of_draught().required(&hull).unwrap(), 1.2));
        // A 2 m floor exceeds 10 %.
        let floor = ClearancePolicy {
            minimum: metres(2.0),
            fraction_of_draught: 0.1,
        };
        assert!(about(floor.required(&hull).unwrap(), 2.0));

        for bad in [
            ClearancePolicy {
                minimum: metres(-1.0),
                fraction_of_draught: 0.1,
            },
            ClearancePolicy {
                minimum: metres(1.0),
                fraction_of_draught: -0.1,
            },
            ClearancePolicy {
                minimum: metres(1.0),
                fraction_of_draught: f64::NAN,
            },
        ] {
            assert!(bad.required(&hull).is_err());
        }
    }

    #[test]
    fn enough_water_is_no_event_and_too_little_is_one() {
        let hull = tanker();
        let (open, events) = under_keel_clearance(
            &hull,
            knots(12.0),
            Waterway::OpenWater,
            metres(14.0),
            &sample(Some(1.5)),
            &tenth_of_draught(),
        )
        .unwrap();
        assert_eq!(open.at(), noon());
        assert!(about(open.depth_of_water(), 15.5));
        assert!(about(open.dynamic_draught(), 13.224));
        assert!(about(open.clearance(), 2.276));
        assert!(about(open.required(), 1.2));
        assert!(about(open.margin(), 1.076));
        assert!(open.is_sufficient());
        assert!(events.is_empty());

        let (channel, events) = under_keel_clearance(
            &hull,
            knots(12.0),
            Waterway::ConfinedChannel,
            metres(14.0),
            &sample(Some(1.5)),
            &tenth_of_draught(),
        )
        .unwrap();
        assert!(!channel.is_sufficient());
        assert!(channel.margin().is_negative());
        assert!(matches!(
            events[0],
            ClearanceEvent::UnderKeelClearanceLow { clearance, required, at }
                if clearance == channel.clearance() && about(required, 1.2) && at == noon()
        ));
    }

    #[test]
    fn aground_is_a_negative_clearance_not_an_error() {
        let (stuck, events) = under_keel_clearance(
            &tanker(),
            knots(4.0),
            Waterway::OpenWater,
            metres(10.0),
            &sample(Some(0.5)),
            &tenth_of_draught(),
        )
        .unwrap();
        assert!(stuck.clearance().is_negative());
        assert_eq!(events.len(), 1);
    }

    #[test]
    fn a_drying_height_is_a_negative_charted_depth() {
        let dinghy = Hull::new(metres(0.3), 0.5).unwrap();
        let policy = ClearancePolicy {
            minimum: metres(0.2),
            fraction_of_draught: 0.0,
        };
        // Dries 1.0 m; 3 m of tide leaves 2 m of water.
        let (over, _) = under_keel_clearance(
            &dinghy,
            knots(3.0),
            Waterway::OpenWater,
            metres(-1.0),
            &sample(Some(3.0)),
            &policy,
        )
        .unwrap();
        assert!(about(over.depth_of_water(), 2.0));
        assert!(over.is_sufficient());
    }

    #[test]
    fn without_the_tide_resolved_there_is_no_answer() {
        assert!(matches!(
            under_keel_clearance(
                &tanker(),
                knots(12.0),
                Waterway::OpenWater,
                metres(14.0),
                &sample(None),
                &tenth_of_draught(),
            )
            .unwrap_err(),
            NavigationError::Kernel(KernelError::Missing {
                what: "the height of tide"
            })
        ));
    }

    #[test]
    fn the_tide_needed_makes_the_clearance_exactly_the_policy() {
        let hull = tanker();
        let policy = tenth_of_draught();
        let needed = height_of_tide_needed(
            &hull,
            knots(12.0),
            Waterway::ConfinedChannel,
            metres(14.0),
            &policy,
        )
        .unwrap();
        // 12 + 2.448 + 1.2 − 14.
        assert!((needed.metres() - 1.648).abs() < 1e-9);

        // 1 mm more tide suffices; 1 mm less does not.
        for (extra, enough) in [(0.001, true), (-0.001, false)] {
            let (at_the_edge, events) = under_keel_clearance(
                &hull,
                knots(12.0),
                Waterway::ConfinedChannel,
                metres(14.0),
                &sample(Some(needed.metres() + extra)),
                &policy,
            )
            .unwrap();
            assert!(about(at_the_edge.margin(), extra));
            assert_eq!(at_the_edge.is_sufficient(), enough);
            assert_eq!(events.is_empty(), enough);
        }

        // Deep water needs no tide: negative result.
        let deep = height_of_tide_needed(
            &hull,
            knots(12.0),
            Waterway::OpenWater,
            metres(30.0),
            &policy,
        )
        .unwrap();
        assert!(deep.is_negative());
    }

    #[test]
    fn a_tidal_cycle_and_a_constant_answer_the_tide_port() {
        let here = Position::from_degrees(51.0, 3.0).unwrap();
        let low = TideEvent::new(noon(), metres(0.8));
        let high = TideEvent::new(
            noon()
                .checked_add(core::time::Duration::from_secs(6 * 3600))
                .unwrap(),
            metres(4.6),
        );
        let rise = TidalCycle::new(low, high).unwrap();
        let later = noon()
            .checked_add(core::time::Duration::from_secs(2 * 3600))
            .unwrap();
        assert_eq!(
            rise.height_of_tide(here, later).unwrap(),
            rise.height_at(later).unwrap()
        );
        assert!(rise
            .height_of_tide(
                here,
                noon()
                    .checked_sub(core::time::Duration::from_secs(1))
                    .unwrap()
            )
            .is_err());

        assert_eq!(
            Constant(metres(2.0)).height_of_tide(here, later).unwrap(),
            metres(2.0)
        );

        // The sample carries the answer.
        let sample = sample(None).with_tide(rise.height_at(later).unwrap());
        assert_eq!(sample.tide(), Some(rise.height_at(later).unwrap()));
    }
}
