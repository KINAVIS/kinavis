//! COLREGs steering and sailing rules: encounter type, responsibility,
//! permitted manoeuvre.
//!
//! CPA/TCPA is kinematics and lives in the collision assessment; this crate
//! holds the rules. Their sector widths are conventions and their application
//! depends on the waters (TSS, narrow channels), so they are versioned
//! separately, the conventions are explicit in [`ColregsConfig`], and every
//! answer names the deciding [`Rule`].
//!
//! # Rules applied
//!
//! Vessels in sight of one another ([`Visibility::InSight`]):
//!
//! - **Rule 13**, overtaking: a vessel coming up from more than 22.5° abaft the
//!   other's beam keeps out of the way, whatever the categories. *Coming up*
//!   requires a closing range; astern and opening is treated as crossing
//!   geometry.
//! - **Rule 18**, responsibilities: power-driven gives way to sailing, fishing,
//!   RAM and NUC, in that order of precedence; a vessel constrained by her
//!   draught is not to be impeded.
//! - **Rule 14**, head-on: two power-driven vessels on reciprocal or nearly
//!   reciprocal courses both alter to starboard.
//! - **Rule 15**, crossing: the power-driven vessel with the other on her
//!   starboard side gives way.
//! - **Rule 12**, sailing vessels: port tack keeps clear of starboard tack; on
//!   the same tack, windward keeps clear of leeward.
//! - **Rule 17**, stand-on vessel: keeps course and speed; may act if the
//!   give-way vessel does not; does not alter to port for a vessel on her port
//!   side.
//!
//! Restricted visibility ([`Visibility::Restricted`]), **Rule 19**: no stand-on
//! vessel; no alteration to port for a vessel forward of the beam, none towards
//! a vessel abeam or abaft the beam.
//!
//! Rules 9 and 10 (narrow channels, TSS) are **not** applied. Between two
//! vessels of the same category other than power-driven or sailing (e.g. two
//! fishing vessels) the rules are silent; the head-on and crossing geometry is
//! applied and reported as Rule 14 or 15.
//!
//! ```rust
//! use kinavis::relative_motion::{Contact, Vessel};
//! use kinavis_colregs::{
//!     rule_of_the_road, ColregsConfig, Encounter, Party, Responsibility, Situation,
//!     VesselCategory, Visibility,
//! };
//! use kinavis_kernel::{Distance, Side, Speed, TrueBearing, TrueCourse};
//!
//! // Steering north at twelve knots; a power-driven vessel five miles off
//! // on the starboard bow, steering west at the same speed.
//! let own = Party::new(
//!     Vessel { course: TrueCourse::new(0.0)?, speed: Speed::from_knots(12.0)? },
//!     VesselCategory::PowerDriven,
//! );
//! let target = Party::new(
//!     Vessel { course: TrueCourse::new(270.0)?, speed: Speed::from_knots(12.0)? },
//!     VesselCategory::PowerDriven,
//! );
//! let contact = Contact {
//!     bearing: TrueBearing::new(45.0)?,
//!     range: Distance::from_nautical_miles(5.0)?,
//! };
//! let situation = Situation::new(own, target, contact, Visibility::InSight);
//!
//! let ruling = rule_of_the_road(&situation, &ColregsConfig::STANDARD)?;
//! assert_eq!(ruling.encounter(), Encounter::Crossing { target_on: Side::Starboard });
//! assert_eq!(ruling.responsibility(), Responsibility::GiveWay);
//! assert_eq!(format!("{}", ruling.rule()), "Rule 15");
//! // Give way by altering to starboard and passing astern; not to port,
//! // which would cross ahead.
//! let may = ruling.manoeuvre();
//! assert!(may.starboard && !may.port && may.slow_down && !may.hold);
//!
//! // A sailing vessel would stand on regardless of geometry.
//! let sailing = Situation::new(
//!     own,
//!     Party::new(target.motion(), VesselCategory::Sailing { tack: None }),
//!     contact,
//!     Visibility::InSight,
//! );
//! let ruling = rule_of_the_road(&sailing, &ColregsConfig::STANDARD)?;
//! assert_eq!(ruling.responsibility(), Responsibility::GiveWay);
//! assert_eq!(format!("{}", ruling.rule()), "Rule 18");
//! # Ok::<(), kinavis_kernel::KernelError>(())
//! ```
//!
//! # Feature flags
//!
//! - `std` *(default)* — standard library maths in the kernel.
//! - `libm` — for `no_std` targets: `--no-default-features --features libm`.
//! - `serde` — serialisation of the value types.
//!
//! No allocation; builds for bare-metal targets.

#![cfg_attr(not(feature = "std"), no_std)]

// The crate does not allocate; tests use `format!`.
#[cfg(test)]
extern crate alloc;

mod rules;

use core::fmt;

use kinavis::relative_motion::{Contact, Vessel};
use kinavis_kernel::angle::{Side, TrueCourse};
use kinavis_kernel::units::Angle;

pub use rules::rule_of_the_road;

/// Runs the `README.md` example as a doctest.
#[cfg(doctest)]
#[doc = include_str!("../README.md")]
pub struct ReadmeExamples;

/// Side of a sailing vessel the wind is on.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Tack {
    /// Wind on the port side; mainsail carried to starboard.
    Port,
    /// Wind on the starboard side.
    Starboard,
}

/// Vessel category per Rule 3, ordered as in Rule 18.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum VesselCategory {
    /// Propelled by machinery; the case Rules 14 and 15 address.
    PowerDriven,
    /// Under sail, no machinery in use. The tack matters only between two
    /// sailing vessels (Rule 12); `None` if unknown.
    Sailing {
        /// Wind side, if known.
        tack: Option<Tack>,
    },
    /// Engaged in fishing with gear that restricts manoeuvrability.
    Fishing,
    /// Constrained by her draught: not to be impeded, but ranks below Rule 18
    /// (a)–(c).
    ConstrainedByDraught,
    /// Restricted in her ability to manoeuvre (RAM).
    RestrictedInAbilityToManoeuvre,
    /// Not under command (NUC).
    NotUnderCommand,
}

impl VesselCategory {
    /// Rule 18 precedence: higher stands on, lower gives way; equal means Rule
    /// 18 does not decide.
    #[must_use]
    pub const fn precedence(self) -> u8 {
        match self {
            Self::PowerDriven => 0,
            Self::Sailing { .. } => 1,
            Self::Fishing => 2,
            Self::ConstrainedByDraught => 3,
            Self::RestrictedInAbilityToManoeuvre => 4,
            Self::NotUnderCommand => 5,
        }
    }
}

/// One vessel in an encounter: motion and category.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Party {
    motion: Vessel,
    heading: Option<TrueCourse>,
    category: VesselCategory,
}

impl Party {
    /// Vessel of `category` with ground motion `motion`; heading defaults to
    /// the course over ground until set by [`Party::with_heading`].
    #[must_use]
    pub const fn new(motion: Vessel, category: VesselCategory) -> Self {
        Self {
            motion,
            heading: None,
            category,
        }
    }

    /// Sets the heading. The rules refer to the vessel's head, which differs
    /// from the course over ground under current or leeway.
    #[must_use]
    pub const fn with_heading(mut self, heading: TrueCourse) -> Self {
        self.heading = Some(heading);
        self
    }

    /// Course and speed over the ground.
    #[must_use]
    pub const fn motion(&self) -> Vessel {
        self.motion
    }

    /// Heading: as set, or the course over ground.
    #[must_use]
    pub fn heading(&self) -> TrueCourse {
        self.heading.unwrap_or(self.motion.course)
    }

    /// Category.
    #[must_use]
    pub const fn category(&self) -> VesselCategory {
        self.category
    }
}

/// Whether the vessels are in sight of one another.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Visibility {
    /// Visual contact: Section II applies.
    InSight,
    /// Fog, mist, falling snow, heavy rain: Rule 19 applies; no stand-on
    /// vessel.
    Restricted,
}

/// Encounter: own ship, the other vessel, its bearing and range, and
/// conditions.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Situation {
    own: Party,
    target: Party,
    contact: Contact,
    visibility: Visibility,
    wind_from: Option<TrueCourse>,
}

impl Situation {
    /// Own ship and the other vessel at `contact` from own ship.
    #[must_use]
    pub const fn new(own: Party, target: Party, contact: Contact, visibility: Visibility) -> Self {
        Self {
            own,
            target,
            contact,
            visibility,
            wind_from: None,
        }
    }

    /// Sets the true wind; Rule 12 (a)(ii) needs it to tell windward from
    /// leeward.
    #[must_use]
    pub const fn with_wind_from(mut self, from: TrueCourse) -> Self {
        self.wind_from = Some(from);
        self
    }

    /// Own ship.
    #[must_use]
    pub const fn own(&self) -> Party {
        self.own
    }

    /// Other vessel.
    #[must_use]
    pub const fn target(&self) -> Party {
        self.target
    }

    /// Bearing and range of the other vessel from own ship.
    #[must_use]
    pub const fn contact(&self) -> Contact {
        self.contact
    }

    /// Visibility.
    #[must_use]
    pub const fn visibility(&self) -> Visibility {
        self.visibility
    }

    /// Direction the true wind blows from, if known.
    #[must_use]
    pub const fn wind_from(&self) -> Option<TrueCourse> {
        self.wind_from
    }
}

/// Conventions used to apply the rules.
///
/// Rule 13 fixes the overtaking sector; "nearly reciprocal" is left to
/// judgement. These values encode that judgement and may be set per vessel or
/// area.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ColregsConfig {
    /// Start of the overtaking sector abaft the beam: 22.5° per Rule 13 (b),
    /// the arc of the stern light.
    pub overtaking_abaft_beam: Angle,
    /// Half-width either side of dead ahead, for both relative bearings, within
    /// which courses are nearly reciprocal under Rule 14. Customary value 6°.
    pub head_on_half_width: Angle,
}

impl ColregsConfig {
    /// Customary conventions: 22.5° and 6°.
    pub const STANDARD: Self = Self {
        overtaking_abaft_beam: Angle::from_degrees_unchecked(22.5),
        head_on_half_width: Angle::from_degrees_unchecked(6.0),
    };
}

/// Encounter type from own ship's side.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Encounter {
    /// Own ship is overtaking the other vessel.
    Overtaking,
    /// The other vessel is overtaking own ship.
    BeingOvertaken,
    /// Reciprocal or nearly reciprocal courses, each ahead of the other.
    HeadOn,
    /// Crossing courses; the other vessel on the given side.
    Crossing {
        /// Side of own ship the other vessel bears on.
        target_on: Side,
    },
}

/// Who keeps out of the way.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Responsibility {
    /// Own ship gives way: early and substantial action.
    GiveWay,
    /// Own ship stands on and monitors.
    StandOn,
    /// Both act: head-on, or restricted visibility.
    Both,
    /// Not decidable from the inputs: two sailing vessels on the same tack, no
    /// wind given.
    Undetermined,
}

/// Deciding rule.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Rule {
    /// Sailing vessels.
    Rule12,
    /// Overtaking.
    Rule13,
    /// Head-on situation.
    Rule14,
    /// Crossing situation.
    Rule15,
    /// Action by stand-on vessel.
    Rule17,
    /// Responsibilities between vessels.
    Rule18,
    /// Conduct of vessels in restricted visibility.
    Rule19,
}

impl fmt::Display for Rule {
    /// Formats as `Rule 15`.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let number = match self {
            Self::Rule12 => 12,
            Self::Rule13 => 13,
            Self::Rule14 => 14,
            Self::Rule15 => 15,
            Self::Rule17 => 17,
            Self::Rule18 => 18,
            Self::Rule19 => 19,
        };
        write!(f, "Rule {number}")
    }
}

/// Constraints the rules place on own ship's manoeuvre.
///
/// Not a manoeuvre (that is computed from the geometry): permitted alteration
/// sides, speed reduction, and whether to hold course and speed.
// Four independent permissions, not a state machine; hence the lint allowance.
#[allow(clippy::struct_excessive_bools)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PermittedManoeuvre {
    /// Alteration to starboard permitted.
    pub starboard: bool,
    /// Alteration to port permitted.
    pub port: bool,
    /// Speed reduction permitted.
    pub slow_down: bool,
    /// Hold course and speed; the permitted alterations apply only if the other
    /// vessel fails to act.
    pub hold: bool,
}

/// Result of applying the rules to a situation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Ruling {
    encounter: Encounter,
    responsibility: Responsibility,
    rule: Rule,
    manoeuvre: PermittedManoeuvre,
}

impl Ruling {
    /// Encounter type.
    #[must_use]
    pub const fn encounter(&self) -> Encounter {
        self.encounter
    }

    /// Who keeps out of the way.
    #[must_use]
    pub const fn responsibility(&self) -> Responsibility {
        self.responsibility
    }

    /// Rule that decided the responsibility.
    #[must_use]
    pub const fn rule(&self) -> Rule {
        self.rule
    }

    /// Permitted manoeuvre for own ship.
    #[must_use]
    pub const fn manoeuvre(&self) -> PermittedManoeuvre {
        self.manoeuvre
    }
}
