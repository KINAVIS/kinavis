//! Rule evaluation.

use kinavis::relative_motion::Vessel;
use kinavis_kernel::angle::{wrap180, Side};
use kinavis_kernel::error::{ensure_range, Result};
use kinavis_kernel::math;

use crate::{
    ColregsConfig, Encounter, PermittedManoeuvre, Responsibility, Rule, Ruling, Situation, Tack,
    VesselCategory, Visibility,
};

/// Half-width of the dead-ahead sector that counts as neither bow.
const DEAD_AHEAD_DEG: f64 = 1e-9;

/// Encounter geometry, in degrees from each vessel's head.
struct Geometry {
    /// Relative bearing of the other vessel from own ship's head, positive to
    /// starboard.
    relative: f64,
    /// Relative bearing of own ship from the other vessel's head, positive to
    /// its starboard (aspect).
    aspect: f64,
}

/// Applies the rules to a situation.
///
/// # Errors
///
/// [`KernelError::OutOfRange`] if the overtaking sector or the head-on
/// half-width in the configuration is outside `[0°, 90°]`.
///
/// [`KernelError::OutOfRange`]: kinavis_kernel::KernelError::OutOfRange
pub fn rule_of_the_road(situation: &Situation, config: &ColregsConfig) -> Result<Ruling> {
    ensure_range(
        "overtaking sector",
        config.overtaking_abaft_beam.degrees(),
        0.0,
        90.0,
    )?;
    ensure_range(
        "head-on half-width",
        config.head_on_half_width.degrees(),
        0.0,
        90.0,
    )?;

    let geometry = geometry_of(situation);
    let encounter = encounter_of(&geometry, situation, config);

    if situation.visibility() == Visibility::Restricted {
        return Ok(Ruling {
            encounter,
            responsibility: Responsibility::Both,
            rule: Rule::Rule19,
            manoeuvre: restricted_visibility(encounter, geometry.relative),
        });
    }

    let (responsibility, rule) = responsibility_of(encounter, situation);
    let manoeuvre = in_sight(encounter, responsibility, rule);
    Ok(Ruling {
        encounter,
        responsibility,
        rule,
        manoeuvre,
    })
}

/// Relative bearings of each vessel from the other's head.
fn geometry_of(situation: &Situation) -> Geometry {
    let contact = situation.contact();
    let relative = wrap180(contact.bearing.degrees() - situation.own().heading().degrees());
    let from_target = contact.bearing.degrees() + 180.0;
    let aspect = wrap180(from_target - situation.target().heading().degrees());
    Geometry { relative, aspect }
}

/// Encounter type from the geometry.
fn encounter_of(geometry: &Geometry, situation: &Situation, config: &ColregsConfig) -> Encounter {
    let own = situation.own().motion();
    let target = situation.target().motion();
    let overtaking_from = 90.0 + config.overtaking_abaft_beam.degrees();
    // Rule 13 "coming up with": from abaft the beam and closing. Astern and
    // opening is not overtaking; it falls through to crossing geometry.
    let closing = is_closing(situation);
    let own_astern_of_target = closing && math::abs(geometry.aspect) > overtaking_from;
    let target_astern_of_own = closing && math::abs(geometry.relative) > overtaking_from;

    match (own_astern_of_target, target_astern_of_own) {
        (true, false) => return Encounter::Overtaking,
        (false, true) => return Encounter::BeingOvertaken,
        // Each astern of the other and closing: the faster one is overtaking.
        (true, true) => {
            return if own.speed.knots() > target.speed.knots() {
                Encounter::Overtaking
            } else {
                Encounter::BeingOvertaken
            };
        }
        (false, false) => {}
    }

    let half = config.head_on_half_width.degrees();
    if math::abs(geometry.relative) <= half && math::abs(geometry.aspect) <= half {
        return Encounter::HeadOn;
    }
    // Dead ahead is neither side; the other vessel's side decides, so both
    // vessels' readings agree.
    let starboard = if math::abs(geometry.relative) < DEAD_AHEAD_DEG {
        geometry.aspect < 0.0
    } else {
        geometry.relative > 0.0
    };
    Encounter::Crossing {
        target_on: if starboard {
            Side::Starboard
        } else {
            Side::Port
        },
    }
}

/// Whether the range is closing: relative velocity has a component towards own
/// ship.
fn is_closing(situation: &Situation) -> bool {
    let contact = situation.contact();
    let (north, east) = (
        math::cos(contact.bearing.radians()),
        math::sin(contact.bearing.radians()),
    );
    let (own_north, own_east) = components(situation.own().motion());
    let (target_north, target_east) = components(situation.target().motion());
    let closing_rate = north * (target_north - own_north) + east * (target_east - own_east);
    // A rate within rounding of zero is tangential, not closing.
    let scale = math::hypot(own_north, own_east).max(math::hypot(target_north, target_east));
    closing_rate < 0.0 && !math::is_effectively_zero(closing_rate, scale)
}

/// Velocity as north and east components, in knots.
fn components(vessel: Vessel) -> (f64, f64) {
    let radians = vessel.course.radians();
    (
        vessel.speed.knots() * math::cos(radians),
        vessel.speed.knots() * math::sin(radians),
    )
}

/// Responsibility and deciding rule, vessels in sight.
fn responsibility_of(encounter: Encounter, situation: &Situation) -> (Responsibility, Rule) {
    // Rule 13 first: it applies notwithstanding Rule 18.
    match encounter {
        Encounter::Overtaking => return (Responsibility::GiveWay, Rule::Rule13),
        Encounter::BeingOvertaken => return (Responsibility::StandOn, Rule::Rule13),
        _ => {}
    }

    let own = situation.own().category();
    let target = situation.target().category();
    match own.precedence().cmp(&target.precedence()) {
        core::cmp::Ordering::Less => return (Responsibility::GiveWay, Rule::Rule18),
        core::cmp::Ordering::Greater => return (Responsibility::StandOn, Rule::Rule18),
        core::cmp::Ordering::Equal => {}
    }

    if let (
        VesselCategory::Sailing { tack: own_tack },
        VesselCategory::Sailing { tack: target_tack },
    ) = (own, target)
    {
        return (
            sailing_vessels(own_tack, target_tack, situation),
            Rule::Rule12,
        );
    }

    match encounter {
        Encounter::HeadOn => (Responsibility::Both, Rule::Rule14),
        Encounter::Crossing {
            target_on: Side::Starboard,
        } => (Responsibility::GiveWay, Rule::Rule15),
        Encounter::Crossing { .. } => (Responsibility::StandOn, Rule::Rule17),
        // Handled above; arm needed for exhaustiveness.
        Encounter::Overtaking | Encounter::BeingOvertaken => {
            (Responsibility::Undetermined, Rule::Rule13)
        }
    }
}

/// Rule 12: two sailing vessels.
fn sailing_vessels(
    own: Option<Tack>,
    target: Option<Tack>,
    situation: &Situation,
) -> Responsibility {
    match (own, target) {
        (Some(Tack::Port), Some(Tack::Starboard)) => Responsibility::GiveWay,
        (Some(Tack::Starboard), Some(Tack::Port)) => Responsibility::StandOn,
        (Some(_), Some(_)) => match situation.wind_from() {
            // Own ship is to windward when the other vessel lies within 90° of
            // the downwind direction.
            Some(from) => {
                let towards = from.degrees() + 180.0;
                let downwind = wrap180(situation.contact().bearing.degrees() - towards);
                if math::abs(downwind) < 90.0 {
                    Responsibility::GiveWay
                } else {
                    Responsibility::StandOn
                }
            }
            None => Responsibility::Undetermined,
        },
        _ => Responsibility::Undetermined,
    }
}

/// Permitted manoeuvre, vessels in sight.
fn in_sight(
    encounter: Encounter,
    responsibility: Responsibility,
    rule: Rule,
) -> PermittedManoeuvre {
    match (responsibility, rule, encounter) {
        // Rule 15 with Rule 16: give way and avoid crossing ahead (alter to
        // starboard, pass astern). Rule 14: both alter to starboard. Same
        // constraints.
        (Responsibility::GiveWay, Rule::Rule15, _) | (Responsibility::Both, Rule::Rule14, _) => {
            PermittedManoeuvre {
                starboard: true,
                port: false,
                slow_down: true,
                hold: false,
            }
        }
        // Rule 17: hold course and speed; may act if the give-way vessel does
        // not, but not to port for a vessel on the port side.
        (Responsibility::StandOn, _, Encounter::Crossing { .. }) => PermittedManoeuvre {
            starboard: true,
            port: false,
            slow_down: true,
            hold: true,
        },
        (Responsibility::StandOn, _, _) => PermittedManoeuvre {
            starboard: true,
            port: true,
            slow_down: true,
            hold: true,
        },
        // Overtaking, Rule 18, Rule 12: give way to either side. Undetermined:
        // no constraints.
        (Responsibility::GiveWay | Responsibility::Both | Responsibility::Undetermined, _, _) => {
            PermittedManoeuvre {
                starboard: true,
                port: true,
                slow_down: true,
                hold: false,
            }
        }
    }
}

/// Rule 19 (d): permitted manoeuvre in restricted visibility, from the other
/// vessel's bearing.
fn restricted_visibility(encounter: Encounter, relative: f64) -> PermittedManoeuvre {
    let forward_of_beam = math::abs(relative) < 90.0;
    let (starboard, port) = if forward_of_beam {
        // No alteration to port for a vessel forward of the beam, except one
        // being overtaken.
        (true, encounter == Encounter::Overtaking)
    } else if relative >= 0.0 {
        // No alteration towards a vessel abeam or abaft the beam: it is to
        // starboard, so no alteration to starboard.
        (false, true)
    } else {
        (true, false)
    };
    PermittedManoeuvre {
        starboard,
        port,
        slow_down: true,
        hold: false,
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;
    use crate::Party;
    use kinavis::relative_motion::Contact;
    use kinavis_kernel::angle::{TrueBearing, TrueCourse};
    use kinavis_kernel::units::{Angle, Distance, Speed};

    fn vessel(course: f64, knots: f64) -> Vessel {
        Vessel {
            course: TrueCourse::new(course).unwrap(),
            speed: Speed::from_knots(knots).unwrap(),
        }
    }

    fn power(course: f64, knots: f64) -> Party {
        Party::new(vessel(course, knots), VesselCategory::PowerDriven)
    }

    fn bearing(degrees: f64) -> Contact {
        Contact {
            bearing: TrueBearing::new(degrees).unwrap(),
            range: Distance::from_nautical_miles(5.0).unwrap(),
        }
    }

    fn in_sight(own: Party, target: Party, contact: Contact) -> Ruling {
        rule_of_the_road(
            &Situation::new(own, target, contact, Visibility::InSight),
            &ColregsConfig::STANDARD,
        )
        .unwrap()
    }

    #[test]
    fn crossing_the_vessel_with_the_other_to_starboard_gives_way() {
        // Own ship heading north; other vessel bears 045, heading west: on the
        // starboard side.
        let ruling = in_sight(power(0.0, 12.0), power(270.0, 12.0), bearing(45.0));
        assert_eq!(
            ruling.encounter(),
            Encounter::Crossing {
                target_on: Side::Starboard
            }
        );
        assert_eq!(ruling.responsibility(), Responsibility::GiveWay);
        assert_eq!(ruling.rule(), Rule::Rule15);
        let may = ruling.manoeuvre();
        assert!(may.starboard && !may.port && may.slow_down && !may.hold);

        // Other vessel bears 315, heading east: on the port side; own ship
        // stands on.
        let ruling = in_sight(power(0.0, 12.0), power(90.0, 12.0), bearing(315.0));
        assert_eq!(
            ruling.encounter(),
            Encounter::Crossing {
                target_on: Side::Port
            }
        );
        assert_eq!(ruling.responsibility(), Responsibility::StandOn);
        assert_eq!(ruling.rule(), Rule::Rule17);
        let may = ruling.manoeuvre();
        assert!(may.hold && may.starboard && !may.port);
    }

    #[test]
    fn head_on_is_nearly_reciprocal_and_both_go_to_starboard() {
        // Dead ahead on the reciprocal course.
        let ruling = in_sight(power(0.0, 12.0), power(180.0, 12.0), bearing(0.0));
        assert_eq!(ruling.encounter(), Encounter::HeadOn);
        assert_eq!(ruling.responsibility(), Responsibility::Both);
        assert_eq!(ruling.rule(), Rule::Rule14);
        assert!(ruling.manoeuvre().starboard && !ruling.manoeuvre().port);

        // 5° on the bow, courses 5° off reciprocal: nearly reciprocal.
        let ruling = in_sight(power(0.0, 12.0), power(185.0, 12.0), bearing(5.0));
        assert_eq!(ruling.encounter(), Encounter::HeadOn);

        // 10° on the bow: crossing.
        let ruling = in_sight(power(0.0, 12.0), power(190.0, 12.0), bearing(10.0));
        assert!(matches!(ruling.encounter(), Encounter::Crossing { .. }));

        // A wider convention makes it head-on.
        let wide = ColregsConfig {
            head_on_half_width: Angle::from_degrees(12.0).unwrap(),
            ..ColregsConfig::STANDARD
        };
        let ruling = rule_of_the_road(
            &Situation::new(
                power(0.0, 12.0),
                power(190.0, 12.0),
                bearing(10.0),
                Visibility::InSight,
            ),
            &wide,
        )
        .unwrap();
        assert_eq!(ruling.encounter(), Encounter::HeadOn);
    }

    #[test]
    fn overtaking_keeps_clear_whatever_the_vessels_are() {
        // Both heading north, other vessel ahead and slower: own ship overtakes
        // from right astern.
        let sailing = Party::new(vessel(0.0, 15.0), VesselCategory::Sailing { tack: None });
        let ruling = in_sight(sailing, power(0.0, 8.0), bearing(0.0));
        assert_eq!(ruling.encounter(), Encounter::Overtaking);
        assert_eq!(ruling.responsibility(), Responsibility::GiveWay);
        assert_eq!(ruling.rule(), Rule::Rule13);
        assert!(ruling.manoeuvre().starboard && ruling.manoeuvre().port);

        // Reversed: the other vessel overtakes on the starboard quarter.
        let ruling = in_sight(power(0.0, 8.0), power(0.0, 15.0), bearing(150.0));
        assert_eq!(ruling.encounter(), Encounter::BeingOvertaken);
        assert_eq!(ruling.responsibility(), Responsibility::StandOn);
        assert_eq!(ruling.rule(), Rule::Rule13);
        assert!(ruling.manoeuvre().hold);

        // Just outside the sector (20° abaft the beam): crossing.
        let ruling = in_sight(power(0.0, 8.0), power(0.0, 15.0), bearing(110.0));
        assert!(matches!(ruling.encounter(), Encounter::Crossing { .. }));

        // Opening, each astern of the other: not overtaking.
        let ruling = in_sight(power(0.0, 15.0), power(180.0, 8.0), bearing(180.0));
        assert!(matches!(ruling.encounter(), Encounter::Crossing { .. }));
        // Astern but slower, so not closing: not overtaking.
        let ruling = in_sight(power(0.0, 6.0), power(0.0, 15.0), bearing(0.0));
        assert!(matches!(ruling.encounter(), Encounter::Crossing { .. }));
    }

    #[test]
    fn rule_18_ranks_the_vessels() {
        let fishing = Party::new(vessel(90.0, 4.0), VesselCategory::Fishing);
        // Port bow, crossing: a power-driven vessel would stand on here.
        let ruling = in_sight(power(0.0, 12.0), fishing, bearing(315.0));
        assert_eq!(ruling.responsibility(), Responsibility::GiveWay);
        assert_eq!(ruling.rule(), Rule::Rule18);
        assert!(ruling.manoeuvre().port && ruling.manoeuvre().starboard);

        // Own ship RAM: the other vessel gives way.
        let ram = Party::new(
            vessel(0.0, 3.0),
            VesselCategory::RestrictedInAbilityToManoeuvre,
        );
        let ruling = in_sight(ram, fishing, bearing(45.0));
        assert_eq!(ruling.responsibility(), Responsibility::StandOn);
        assert_eq!(ruling.rule(), Rule::Rule18);

        // NUC has the highest precedence.
        let nuc = Party::new(vessel(0.0, 0.0), VesselCategory::NotUnderCommand);
        assert_eq!(
            in_sight(ram, nuc, bearing(45.0)).responsibility(),
            Responsibility::GiveWay
        );
        assert!(
            VesselCategory::NotUnderCommand.precedence()
                > VesselCategory::ConstrainedByDraught.precedence()
        );

        // Two fishing vessels: Rule 18 is silent, crossing rule applies.
        let ruling = in_sight(
            Party::new(vessel(0.0, 4.0), VesselCategory::Fishing),
            fishing,
            bearing(45.0),
        );
        assert_eq!(ruling.responsibility(), Responsibility::GiveWay);
        assert_eq!(ruling.rule(), Rule::Rule15);
    }

    #[test]
    fn rule_12_between_sailing_vessels() {
        let on = |tack| VesselCategory::Sailing { tack: Some(tack) };
        let port_tack = Party::new(vessel(0.0, 6.0), on(Tack::Port));
        let starboard_tack = Party::new(vessel(90.0, 6.0), on(Tack::Starboard));
        let southbound = Party::new(vessel(180.0, 6.0), on(Tack::Port));

        // Opposite tacks: port tack keeps clear, whichever side the other bears
        // on.
        let ruling = in_sight(port_tack, starboard_tack, bearing(315.0));
        assert_eq!(ruling.responsibility(), Responsibility::GiveWay);
        assert_eq!(ruling.rule(), Rule::Rule12);
        let ruling = in_sight(starboard_tack, southbound, bearing(45.0));
        assert_eq!(ruling.responsibility(), Responsibility::StandOn);

        // Same tack: windward keeps clear. Wind from the west, other vessel
        // bears east: own ship is to windward.
        let same = Party::new(vessel(90.0, 6.0), on(Tack::Port));
        let situation = Situation::new(port_tack, same, bearing(90.0), Visibility::InSight)
            .with_wind_from(TrueCourse::WEST);
        let ruling = rule_of_the_road(&situation, &ColregsConfig::STANDARD).unwrap();
        assert_eq!(ruling.responsibility(), Responsibility::GiveWay);
        // Wind from the east: the other vessel is to windward.
        let situation = situation.with_wind_from(TrueCourse::EAST);
        let ruling = rule_of_the_road(&situation, &ColregsConfig::STANDARD).unwrap();
        assert_eq!(ruling.responsibility(), Responsibility::StandOn);

        // No wind or no tack: undetermined.
        let ruling = in_sight(port_tack, same, bearing(90.0));
        assert_eq!(ruling.responsibility(), Responsibility::Undetermined);
        let unknown = Party::new(vessel(90.0, 6.0), VesselCategory::Sailing { tack: None });
        assert_eq!(
            in_sight(port_tack, unknown, bearing(90.0)).responsibility(),
            Responsibility::Undetermined
        );
        assert!(!in_sight(port_tack, unknown, bearing(90.0)).manoeuvre().hold);
    }

    #[test]
    fn in_restricted_visibility_nobody_stands_on() {
        let situation = |contact| {
            Situation::new(
                power(0.0, 12.0),
                power(90.0, 12.0),
                contact,
                Visibility::Restricted,
            )
        };
        // Port bow; in sight own ship would stand on.
        let ruling =
            rule_of_the_road(&situation(bearing(315.0)), &ColregsConfig::STANDARD).unwrap();
        assert_eq!(ruling.responsibility(), Responsibility::Both);
        assert_eq!(ruling.rule(), Rule::Rule19);
        let may = ruling.manoeuvre();
        assert!(!may.hold && may.starboard && !may.port && may.slow_down);

        // Starboard quarter: not towards it, so not to starboard.
        let ruling =
            rule_of_the_road(&situation(bearing(150.0)), &ColregsConfig::STANDARD).unwrap();
        assert!(!ruling.manoeuvre().starboard && ruling.manoeuvre().port);
        // Port quarter: not to port.
        let ruling =
            rule_of_the_road(&situation(bearing(210.0)), &ColregsConfig::STANDARD).unwrap();
        assert!(ruling.manoeuvre().starboard && !ruling.manoeuvre().port);

        // Overtaking a vessel forward of the beam: either side.
        let overtaking = Situation::new(
            power(0.0, 15.0),
            power(0.0, 8.0),
            bearing(0.0),
            Visibility::Restricted,
        );
        let ruling = rule_of_the_road(&overtaking, &ColregsConfig::STANDARD).unwrap();
        assert_eq!(ruling.encounter(), Encounter::Overtaking);
        assert!(ruling.manoeuvre().port && ruling.manoeuvre().starboard);
    }

    #[test]
    fn the_head_is_the_heading_when_it_is_known() {
        // COG 000, heading 020 into a current: a vessel bearing 010 is on the
        // port bow by the head.
        let own = power(0.0, 12.0).with_heading(TrueCourse::new(20.0).unwrap());
        let ruling = in_sight(own, power(180.0, 12.0), bearing(10.0));
        assert_eq!(
            ruling.encounter(),
            Encounter::Crossing {
                target_on: Side::Port
            }
        );
        assert_eq!(own.heading(), TrueCourse::new(20.0).unwrap());
        assert_eq!(power(0.0, 12.0).heading(), TrueCourse::NORTH);
    }

    #[test]
    fn a_configuration_the_rules_could_not_mean_is_refused() {
        let situation = Situation::new(
            power(0.0, 12.0),
            power(270.0, 12.0),
            bearing(45.0),
            Visibility::InSight,
        );
        for (abaft, half) in [(100.0, 6.0), (-1.0, 6.0), (22.5, 91.0), (22.5, -6.0)] {
            let config = ColregsConfig {
                overtaking_abaft_beam: Angle::from_degrees(abaft).unwrap(),
                head_on_half_width: Angle::from_degrees(half).unwrap(),
            };
            assert!(rule_of_the_road(&situation, &config).is_err());
        }
        assert_eq!(alloc::format!("{}", Rule::Rule19), "Rule 19");
    }
}
