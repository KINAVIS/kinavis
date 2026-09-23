//! Exhaustive sweep of geometries, category pairs and conditions: every
//! situation gets a coherent ruling.

#![allow(clippy::unwrap_used)]

use kinavis::relative_motion::{closest_point_of_approach, Approach, Contact, Vessel};
use kinavis_colregs::{
    rule_of_the_road, ColregsConfig, Encounter, Party, Responsibility, Rule, Situation, Tack,
    VesselCategory, Visibility,
};
use kinavis_kernel::{Distance, Speed, TrueBearing, TrueCourse};

const CATEGORIES: [VesselCategory; 8] = [
    VesselCategory::PowerDriven,
    VesselCategory::Sailing { tack: None },
    VesselCategory::Sailing {
        tack: Some(Tack::Port),
    },
    VesselCategory::Sailing {
        tack: Some(Tack::Starboard),
    },
    VesselCategory::Fishing,
    VesselCategory::ConstrainedByDraught,
    VesselCategory::RestrictedInAbilityToManoeuvre,
    VesselCategory::NotUnderCommand,
];

#[test]
fn the_rules_always_answer_and_the_answer_is_coherent() {
    let speeds = [0.0, 3.0, 12.0, 30.0];
    let mut count = 0_u32;
    for own_category in CATEGORIES {
        for target_category in CATEGORIES {
            for own_course in (0..360).step_by(45) {
                for target_course in (0..360).step_by(45) {
                    for bearing in (0..360).step_by(30) {
                        for (own_speed, target_speed) in speeds.iter().zip(speeds.iter().rev()) {
                            for visibility in [Visibility::InSight, Visibility::Restricted] {
                                let own = Party::new(
                                    Vessel {
                                        course: TrueCourse::new(f64::from(own_course)).unwrap(),
                                        speed: Speed::from_knots(*own_speed).unwrap(),
                                    },
                                    own_category,
                                );
                                let target = Party::new(
                                    Vessel {
                                        course: TrueCourse::new(f64::from(target_course)).unwrap(),
                                        speed: Speed::from_knots(*target_speed).unwrap(),
                                    },
                                    target_category,
                                );
                                let contact = Contact {
                                    bearing: TrueBearing::new(f64::from(bearing)).unwrap(),
                                    range: Distance::from_nautical_miles(4.0).unwrap(),
                                };
                                let situation = Situation::new(own, target, contact, visibility)
                                    .with_wind_from(TrueCourse::WEST);
                                let ruling =
                                    rule_of_the_road(&situation, &ColregsConfig::STANDARD).unwrap();
                                count += 1;

                                let may = ruling.manoeuvre();
                                // At least one action is always permitted.
                                assert!(may.starboard || may.port || may.slow_down);
                                match ruling.responsibility() {
                                    // A give-way vessel never holds course.
                                    Responsibility::GiveWay => assert!(!may.hold),
                                    // A stand-on vessel always holds course
                                    // initially.
                                    Responsibility::StandOn => assert!(may.hold),
                                    _ => {}
                                }
                                match visibility {
                                    Visibility::Restricted => {
                                        assert_eq!(ruling.rule(), Rule::Rule19);
                                        assert_eq!(ruling.responsibility(), Responsibility::Both);
                                        assert!(!may.hold);
                                    }
                                    Visibility::InSight => {
                                        assert_ne!(ruling.rule(), Rule::Rule19);
                                        // Overtaking is decided by Rule 13
                                        // only.
                                        match ruling.encounter() {
                                            Encounter::Overtaking => {
                                                assert_eq!(ruling.rule(), Rule::Rule13);
                                                assert_eq!(
                                                    ruling.responsibility(),
                                                    Responsibility::GiveWay
                                                );
                                            }
                                            Encounter::BeingOvertaken => {
                                                assert_eq!(ruling.rule(), Rule::Rule13);
                                                assert_eq!(
                                                    ruling.responsibility(),
                                                    Responsibility::StandOn
                                                );
                                            }
                                            _ => {}
                                        }
                                        // Rule 18 decides only between
                                        // different categories.
                                        if ruling.rule() == Rule::Rule18 {
                                            assert_ne!(
                                                own_category.precedence(),
                                                target_category.precedence()
                                            );
                                        }
                                        // Undetermined only under Rule 12.
                                        if ruling.responsibility() == Responsibility::Undetermined {
                                            assert_eq!(ruling.rule(), Rule::Rule12);
                                        }
                                    }
                                    _ => {}
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    assert!(count > 100_000);
}

#[test]
fn the_ruling_is_symmetric_where_the_rules_are() {
    // Two closing power-driven vessels: if own ship gives way, the other stands
    // on from its side. Non-closing pairs are not an encounter and are skipped.
    let contact = |bearing: f64| Contact {
        bearing: TrueBearing::new(bearing).unwrap(),
        range: Distance::from_nautical_miles(4.0).unwrap(),
    };
    let power = |course: f64| {
        Party::new(
            Vessel {
                course: TrueCourse::new(course).unwrap(),
                speed: Speed::from_knots(12.0).unwrap(),
            },
            VesselCategory::PowerDriven,
        )
    };
    let mut compared = 0_u32;
    for own_course in (0..360).step_by(15) {
        for target_course in (0..360).step_by(15) {
            for bearing in (0..360).step_by(15) {
                let (own, target) = (
                    power(f64::from(own_course)),
                    power(f64::from(target_course)),
                );
                // Real risk of collision: CPA ahead within 0.5 NM and range
                // closing faster than 1 kn. Looser geometry is not a Rule 15
                // encounter (both vessels can have the other to starboard and
                // pass clear), and the two sides may disagree.
                let approach = closest_point_of_approach(
                    own.motion(),
                    contact(f64::from(bearing)),
                    target.motion(),
                )
                .unwrap();
                let Approach::Closing(cpa) = approach else {
                    continue;
                };
                let closing_miles = 4.0 - cpa.distance.nautical_miles();
                let hours = cpa.time_to_go.as_secs_f64() / 3600.0;
                if hours <= 0.0
                    || closing_miles / hours < 1.0
                    || cpa.distance.nautical_miles() > 0.5
                {
                    continue;
                }
                let ours = rule_of_the_road(
                    &Situation::new(
                        own,
                        target,
                        contact(f64::from(bearing)),
                        Visibility::InSight,
                    ),
                    &ColregsConfig::STANDARD,
                )
                .unwrap();
                let theirs = rule_of_the_road(
                    &Situation::new(
                        target,
                        own,
                        contact(f64::from((bearing + 180) % 360)),
                        Visibility::InSight,
                    ),
                    &ColregsConfig::STANDARD,
                )
                .unwrap();
                compared += 1;
                match ours.responsibility() {
                    Responsibility::GiveWay => {
                        assert_eq!(
                            theirs.responsibility(),
                            Responsibility::StandOn,
                            "own {own_course} target {target_course} bearing {bearing}: {ours:?} / {theirs:?}"
                        );
                    }
                    Responsibility::StandOn => {
                        assert_eq!(
                            theirs.responsibility(),
                            Responsibility::GiveWay,
                            "own {own_course} target {target_course} bearing {bearing}: {ours:?} / {theirs:?}"
                        );
                    }
                    Responsibility::Both => {
                        assert_eq!(theirs.responsibility(), Responsibility::Both);
                    }
                    _ => {}
                }
            }
        }
    }
    assert!(compared > 200, "{compared}");
}

#[cfg(feature = "serde")]
#[test]
fn the_value_types_round_trip() {
    let situation = Situation::new(
        Party::new(
            Vessel {
                course: TrueCourse::new(10.0).unwrap(),
                speed: Speed::from_knots(12.0).unwrap(),
            },
            VesselCategory::Sailing {
                tack: Some(Tack::Port),
            },
        )
        .with_heading(TrueCourse::new(12.0).unwrap()),
        Party::new(
            Vessel {
                course: TrueCourse::new(200.0).unwrap(),
                speed: Speed::from_knots(6.0).unwrap(),
            },
            VesselCategory::Fishing,
        ),
        Contact {
            bearing: TrueBearing::new(30.0).unwrap(),
            range: Distance::from_nautical_miles(3.0).unwrap(),
        },
        Visibility::InSight,
    )
    .with_wind_from(TrueCourse::WEST);
    let text = serde_json::to_string(&situation).unwrap();
    let back: Situation = serde_json::from_str(&text).unwrap();
    assert_eq!(back, situation);

    let ruling = rule_of_the_road(&situation, &ColregsConfig::STANDARD).unwrap();
    let text = serde_json::to_string(&ruling).unwrap();
    let back: kinavis_colregs::Ruling = serde_json::from_str(&text).unwrap();
    assert_eq!(back, ruling);
    let config: ColregsConfig =
        serde_json::from_str(&serde_json::to_string(&ColregsConfig::STANDARD).unwrap()).unwrap();
    assert_eq!(config, ColregsConfig::STANDARD);
}
