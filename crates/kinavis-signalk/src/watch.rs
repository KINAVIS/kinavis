//! Collision watch over a Signal K picture: CPA and TCPA of every target, the
//! COLREGs ruling for the ones that matter, as deltas and notifications.

use std::collections::BTreeMap;
use std::fmt::Write as _;
use std::time::Duration;

use kinavis::relative_motion::{Contact, Vessel};
use kinavis::sailings::rhumb_line;
use kinavis_colregs::{
    rule_of_the_road, ColregsConfig, Encounter, Party, Responsibility, Ruling, Situation,
    VesselCategory, Visibility,
};
use kinavis_kernel::angle::{Side, TrueCourse};
use kinavis_kernel::position::Position;
use kinavis_kernel::time::{Instant, Utc};
use kinavis_kernel::units::{Distance, Speed};
use kinavis_traffic::{assess, CollisionAssessment, CollisionRisk, CpaPolicy};
use serde::Deserialize;
use serde_json::{json, Value};

use crate::delta::Delta;
use crate::picture::{Category, Picture, VesselState};

/// Path of the notification for a target, followed by `.` and the target's
/// context without `vessels.`.
pub const NOTIFICATION_PATH: &str = "notifications.navigation.closestApproach";

/// Watch settings: vessel decisions, with defaults for a small vessel in
/// open water. Read from the plugin configuration, `camelCase`.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct WatchConfig {
    /// CPA inside which an approach is dangerous, nautical miles.
    pub cpa_limit_nm: f64,
    /// TCPA inside which a dangerous approach is an alarm, not yet a
    /// warning, minutes.
    pub tcpa_limit_min: f64,
    /// Age after which a target's last report is too old to assess, seconds.
    pub stale_after_s: f64,
    /// Visibility: the steering rules differ in restricted visibility.
    pub restricted_visibility: bool,
    /// Horizon of warnings, minutes: a dangerous CPA further off than this
    /// is not reported.
    pub warn_within_min: f64,
}

impl Default for WatchConfig {
    fn default() -> Self {
        Self {
            cpa_limit_nm: 1.0,
            tcpa_limit_min: 20.0,
            stale_after_s: 180.0,
            restricted_visibility: false,
            warn_within_min: 60.0,
        }
    }
}

/// Severity of a target's approach, as a Signal K notification state.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Level {
    /// Nothing to report.
    Normal,
    /// Dangerous CPA, but beyond the TCPA limit: developing.
    Warn,
    /// Dangerous CPA within the TCPA limit.
    Alarm,
}

impl Level {
    /// Signal K notification state.
    #[must_use]
    pub const fn state(self) -> &'static str {
        match self {
            Self::Normal => "normal",
            Self::Warn => "warn",
            Self::Alarm => "alarm",
        }
    }

    fn of(assessment: &CollisionAssessment, horizon: Duration) -> Self {
        match assessment.risk() {
            CollisionRisk::Dangerous => Self::Alarm,
            CollisionRisk::Developing if assessment.tcpa().is_some_and(|tcpa| tcpa <= horizon) => {
                Self::Warn
            }
            _ => Self::Normal,
        }
    }
}

/// One target's assessment.
#[derive(Debug, Clone, PartialEq)]
pub struct TargetReport {
    /// Target's context.
    pub context: String,
    /// Name, MMSI or context.
    pub label: String,
    /// CPA, TCPA and risk.
    pub assessment: CollisionAssessment,
    /// COLREGs ruling, for a dangerous or developing approach between two
    /// vessels under way.
    pub ruling: Option<Ruling>,
    /// Level of the approach.
    pub level: Level,
}

/// What to send to the server after an assessment.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, PartialEq)]
pub enum Emission {
    /// A delta, `navigation.closestApproach` in a target's context.
    Delta(Delta),
    /// A notification on own vessel: its path and value.
    Notification {
        /// Full path, [`NOTIFICATION_PATH`]`.<id>`.
        path: String,
        /// `state`, `method` and `message`.
        value: Value,
    },
}

/// The watch: the picture, and the level each target's notification stands at.
#[derive(Debug, Clone)]
pub struct Watch {
    config: WatchConfig,
    picture: Picture,
    standing: BTreeMap<String, Level>,
}

impl Watch {
    /// Empty watch.
    #[must_use]
    pub fn new(config: WatchConfig) -> Self {
        Self {
            config,
            picture: Picture::new(),
            standing: BTreeMap::new(),
        }
    }

    /// Settings.
    #[must_use]
    pub const fn config(&self) -> &WatchConfig {
        &self.config
    }

    /// The picture, to name own vessel's context or read the vessels.
    #[must_use]
    pub const fn picture(&self) -> &Picture {
        &self.picture
    }

    /// The picture, to name own vessel's context.
    pub fn picture_mut(&mut self) -> &mut Picture {
        &mut self.picture
    }

    /// Reads a delta into the picture.
    ///
    /// # Errors
    ///
    /// The JSON error if the text is not a delta.
    pub fn ingest(&mut self, json: &str) -> Result<(), serde_json::Error> {
        let delta: Delta = serde_json::from_str(json)?;
        self.picture.apply(&delta);
        Ok(())
    }

    /// Assesses every target against own vessel at `now`.
    ///
    /// `now` is the caller's clock, not a timestamp from the deltas: sources
    /// stamp their updates by different clocks (a GNSS receiver by its fix,
    /// the server by reception). A vessel is assessed if its last update is
    /// no older than [`WatchConfig::stale_after_s`] at `now`; one stamped
    /// ahead of `now` counts as current.
    ///
    /// Returns the reports, closest CPA first, and what to send: a
    /// `navigation.closestApproach` delta for every closing target, and a
    /// notification wherever a target's level changed. Empty while own
    /// vessel has no current position and motion.
    pub fn assess(&mut self, now: Instant<Utc>) -> (Vec<TargetReport>, Vec<Emission>) {
        let reports = self.reports(now);
        let mut emissions = Vec::new();
        let mut levels = BTreeMap::new();
        for report in &reports {
            if let (Some(cpa), Some(tcpa)) =
                (report.assessment.cpa_distance(), report.assessment.tcpa())
            {
                emissions.push(Emission::Delta(Delta::single(
                    Some(report.context.clone()),
                    "navigation.closestApproach",
                    json!({ "distance": cpa.metres(), "timeTo": tcpa.as_secs_f64() }),
                )));
            }
            levels.insert(report.context.clone(), report.level);
            if self
                .standing
                .get(&report.context)
                .copied()
                .unwrap_or(Level::Normal)
                != report.level
            {
                emissions.push(notification(
                    report.context.as_str(),
                    report.level,
                    &message(report),
                ));
            }
        }
        // Targets that stood above normal and are no longer assessed: stale,
        // gone or opening.
        for (context, level) in &self.standing {
            if *level != Level::Normal && !levels.contains_key(context) {
                emissions.push(notification(context, Level::Normal, "no longer assessed"));
            }
        }
        levels.retain(|_, level| *level != Level::Normal);
        self.standing = levels;
        (reports, emissions)
    }

    fn reports(&self, now: Instant<Utc>) -> Vec<TargetReport> {
        let stale = Duration::try_from_secs_f64(self.config.stale_after_s).unwrap_or(Duration::MAX);
        let Some(own) = self.picture.own().filter(|own| is_fresh(own, now, stale)) else {
            return Vec::new();
        };
        let (Some(own_position), Some(own_motion)) = (own.position, motion(own)) else {
            return Vec::new();
        };
        let Some(policy) = self.policy() else {
            return Vec::new();
        };
        let mut reports: Vec<TargetReport> = self
            .picture
            .targets()
            .filter(|(_, target)| is_fresh(target, now, stale))
            .filter_map(|(context, target)| {
                let report =
                    self.report(own, own_position, own_motion, &policy, context, target)?;
                Some(report)
            })
            .collect();
        reports.sort_by(|a, b| closeness(&a.assessment).total_cmp(&closeness(&b.assessment)));
        reports
    }

    fn report(
        &self,
        own: &VesselState,
        own_position: Position,
        own_motion: Vessel,
        policy: &CpaPolicy,
        context: &str,
        target: &VesselState,
    ) -> Option<TargetReport> {
        let position = target.position?;
        let target_motion = motion(target)?;
        let line = rhumb_line(own_position, position).ok()?;
        let contact = Contact {
            bearing: line.initial_course,
            range: line.distance,
        };
        let assessment = assess(own_motion, contact, target_motion, policy).ok()?;
        let horizon = Duration::try_from_secs_f64(self.config.warn_within_min * 60.0)
            .unwrap_or(Duration::MAX);
        let level = Level::of(&assessment, horizon);
        // The steering rules are for vessels under way: none for own vessel
        // at rest (see `AT_REST_KNOTS`).
        let ruling = if level == Level::Normal || own_motion.speed == Speed::ZERO {
            None
        } else {
            self.ruling(own, own_motion, target, target_motion, contact)
        };
        Some(TargetReport {
            context: context.to_owned(),
            label: target.label(context).to_owned(),
            assessment,
            ruling,
            level,
        })
    }

    fn ruling(
        &self,
        own: &VesselState,
        own_motion: Vessel,
        target: &VesselState,
        target_motion: Vessel,
        contact: Contact,
    ) -> Option<Ruling> {
        // Own vessel under way unless it says otherwise; a target must say
        // nothing contrary either, and counts as power-driven if silent.
        let own_category = under_way(own)?;
        let target_category = under_way(target)?;
        let mut own_party = Party::new(own_motion, own_category);
        if let Some(heading) = own.heading {
            own_party = own_party.with_heading(heading);
        }
        let mut target_party = Party::new(target_motion, target_category);
        if let Some(heading) = target.heading {
            target_party = target_party.with_heading(heading);
        }
        let visibility = if self.config.restricted_visibility {
            Visibility::Restricted
        } else {
            Visibility::InSight
        };
        let situation = Situation::new(own_party, target_party, contact, visibility);
        rule_of_the_road(&situation, &ColregsConfig::STANDARD).ok()
    }

    fn policy(&self) -> Option<CpaPolicy> {
        let limit = Distance::from_nautical_miles(self.config.cpa_limit_nm).ok()?;
        let tcpa = Duration::try_from_secs_f64(self.config.tcpa_limit_min * 60.0).ok()?;
        CpaPolicy::new(limit, tcpa).ok()
    }
}

/// Speed below which a vessel is taken as at rest, knots: a receiver at rest
/// reports a few centimetres a second of noise, and a course that is noise
/// too, or none.
const AT_REST_KNOTS: f64 = 0.5;

/// Course and speed; a vessel at rest may report no course.
fn motion(vessel: &VesselState) -> Option<Vessel> {
    let speed = vessel.speed?;
    if speed.knots() < AT_REST_KNOTS {
        return Some(Vessel {
            course: TrueCourse::NORTH,
            speed: Speed::ZERO,
        });
    }
    Some(Vessel {
        course: vessel.course?,
        speed,
    })
}

/// Category under way: the vessel's own, power-driven if it says nothing,
/// `None` if it is not under way.
fn under_way(vessel: &VesselState) -> Option<VesselCategory> {
    match vessel.category() {
        Some(Category::UnderWay(category)) => Some(category),
        Some(Category::NotUnderWay) => None,
        None => Some(VesselCategory::PowerDriven),
    }
}

/// Whether the last update is no older than `stale`; one stamped ahead of
/// `now` is current.
fn is_fresh(vessel: &VesselState, now: Instant<Utc>, stale: Duration) -> bool {
    vessel.last_seen.is_some_and(|seen| {
        now.checked_duration_since(seen)
            .is_none_or(|age| age <= stale)
    })
}

/// Closest first; opening targets last.
fn closeness(assessment: &CollisionAssessment) -> f64 {
    assessment
        .cpa_distance()
        .map_or(f64::INFINITY, Distance::nautical_miles)
}

fn notification(context: &str, level: Level, message: &str) -> Emission {
    let id = context.strip_prefix("vessels.").unwrap_or(context);
    let method: &[&str] = match level {
        Level::Alarm => &["visual", "sound"],
        Level::Warn => &["visual"],
        Level::Normal => &[],
    };
    Emission::Notification {
        path: format!("{NOTIFICATION_PATH}.{id}"),
        value: json!({ "state": level.state(), "method": method, "message": message }),
    }
}

/// Notification text: the approach, and the ruling if there is one.
#[must_use]
pub fn message(report: &TargetReport) -> String {
    let mut out = report.label.clone();
    match (report.assessment.cpa_distance(), report.assessment.tcpa()) {
        (Some(cpa), Some(tcpa)) => {
            let seconds = tcpa.as_secs();
            let _ = write!(
                out,
                ": CPA {:.2} NM in {}:{:02}",
                cpa.nautical_miles(),
                seconds / 60,
                seconds % 60
            );
        }
        _ => out.push_str(": opening"),
    }
    if let Some(ruling) = &report.ruling {
        let _ = write!(
            out,
            "; {}, {} ({})",
            encounter(ruling.encounter()),
            duty(ruling.responsibility()),
            ruling.rule()
        );
        let may = ruling.manoeuvre();
        let turn = match (may.starboard, may.port) {
            (true, true) => Some("alter to starboard or port"),
            (true, false) => Some("alter to starboard"),
            (false, true) => Some("alter to port"),
            (false, false) => None,
        };
        let options: Vec<&str> = [
            turn,
            may.slow_down.then_some("slow down"),
            may.hold.then_some("hold course and speed"),
        ]
        .into_iter()
        .flatten()
        .collect();
        if !options.is_empty() {
            let _ = write!(out, ": {}", options.join(", or "));
        }
    }
    out
}

fn encounter(encounter: Encounter) -> String {
    match encounter {
        Encounter::Overtaking => "overtaking".to_owned(),
        Encounter::BeingOvertaken => "being overtaken".to_owned(),
        Encounter::HeadOn => "head-on".to_owned(),
        Encounter::Crossing { target_on } => format!("crossing, target on {}", side(target_on)),
        other => format!("{other:?}"),
    }
}

const fn side(side: Side) -> &'static str {
    match side {
        Side::Port => "port",
        Side::Starboard => "starboard",
        Side::Ahead => "ahead",
        Side::Astern => "astern",
        _ => "abeam",
    }
}

const fn duty(responsibility: Responsibility) -> &'static str {
    match responsibility {
        Responsibility::GiveWay => "give way",
        Responsibility::StandOn => "stand on",
        Responsibility::Both => "both alter",
        _ => "no rule decides",
    }
}
