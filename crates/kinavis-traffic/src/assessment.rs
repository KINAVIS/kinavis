//! Collision assessment: CPA, TCPA, bearing rate, bow crossing range and risk.
//!
//! Kinematics only: CPA/TCPA from [`closest_point_of_approach`], bearing at
//! CPA, current bearing rate (a steady bearing on a closing range indicates a
//! collision course), bow crossing range, and the resulting risk against the
//! vessel's [`CpaPolicy`]. Give-way responsibility is a rule and lives in
//! `kinavis-colregs`.

use core::time::Duration;

use kinavis::error::{ensure_range, KernelError, NavigationError, Result};
use kinavis::relative_motion::{
    bow_crossing_range, closest_point_of_approach, Approach, Contact, Cpa, Vessel,
};
use kinavis::sailings::rhumb_line;
use kinavis_kernel::angle::{Direction, True, TrueBearing, TrueCourse};
use kinavis_kernel::event::{EventList, TargetId};

use crate::event::TrafficEvent;
use kinavis_kernel::inline::Inline;
use kinavis_kernel::math;
use kinavis_kernel::position::Position;
use kinavis_kernel::snapshot::NavigationSnapshot;
use kinavis_kernel::time::{Instant, Utc};
use kinavis_kernel::units::{Distance, RateOfTurn, Speed};

use crate::track::TargetTrack;
use crate::{Traffic, MAX_TARGETS};

/// CPA and TCPA limits: vessel settings.
///
/// Validated once by [`CpaPolicy::new`].
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(try_from = "StoredCpaPolicy", into = "StoredCpaPolicy")
)]
pub struct CpaPolicy {
    cpa_limit: Distance,
    tcpa_limit: Duration,
}

impl CpaPolicy {
    /// A CPA within `cpa_limit` is dangerous; a dangerous CPA with TCPA beyond
    /// `tcpa_limit` is developing, not yet an alarm.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] for a negative CPA limit.
    pub fn new(cpa_limit: Distance, tcpa_limit: Duration) -> Result<Self> {
        ensure_range("CPA limit", cpa_limit.nautical_miles(), 0.0, f64::MAX)?;
        Ok(Self {
            cpa_limit,
            tcpa_limit,
        })
    }

    /// CPA limit.
    #[must_use]
    pub const fn cpa_limit(&self) -> Distance {
        self.cpa_limit
    }

    /// TCPA limit; beyond it a dangerous approach is developing, not an alarm.
    #[must_use]
    pub const fn tcpa_limit(&self) -> Duration {
        self.tcpa_limit
    }
}

/// Serialised form; deserialisation goes through [`CpaPolicy::new`].
#[cfg(feature = "serde")]
#[derive(serde::Serialize, serde::Deserialize)]
struct StoredCpaPolicy {
    cpa_limit: Distance,
    tcpa_limit: Duration,
}

#[cfg(feature = "serde")]
impl TryFrom<StoredCpaPolicy> for CpaPolicy {
    type Error = NavigationError;

    fn try_from(stored: StoredCpaPolicy) -> Result<Self> {
        Self::new(stored.cpa_limit, stored.tcpa_limit)
    }
}

#[cfg(feature = "serde")]
impl From<CpaPolicy> for StoredCpaPolicy {
    fn from(policy: CpaPolicy) -> Self {
        Self {
            cpa_limit: policy.cpa_limit,
            tcpa_limit: policy.tcpa_limit,
        }
    }
}

/// Risk level of an encounter.
///
/// Ordered by increasing concern, so assessments can be compared.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
#[non_exhaustive]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum CollisionRisk {
    /// Range opening, or no relative motion: CPA is in the past.
    Opening,
    /// Closing, CPA outside the limit.
    Passing,
    /// Closing, CPA inside the limit, TCPA beyond the limit: monitor.
    Developing,
    /// Closing, CPA and TCPA inside the limits: alarm.
    Dangerous,
}

/// Assessed encounter.
///
/// Projection returned by [`assess`]. CPA is `None` when the range is opening
/// or the contact holds bearing and range.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct CollisionAssessment {
    range: Distance,
    bearing: TrueBearing,
    relative_course: TrueCourse,
    relative_speed: Speed,
    bearing_drift: RateOfTurn,
    cpa: Option<Cpa>,
    bow_crossing: Option<Distance>,
    risk: CollisionRisk,
}

impl CollisionAssessment {
    /// Current range.
    #[must_use]
    pub const fn range(&self) -> Distance {
        self.range
    }

    /// Current bearing.
    #[must_use]
    pub const fn bearing(&self) -> TrueBearing {
        self.bearing
    }

    /// Relative course of the target (direction of its radar echo trail).
    #[must_use]
    pub const fn relative_course(&self) -> TrueCourse {
        self.relative_course
    }

    /// Relative speed of the target.
    #[must_use]
    pub const fn relative_speed(&self) -> Speed {
        self.relative_speed
    }

    /// Bearing rate, positive drawing right. Zero on a closing range means a
    /// collision course.
    #[must_use]
    pub const fn bearing_drift(&self) -> RateOfTurn {
        self.bearing_drift
    }

    /// Closest point of approach: distance, time, bearing. `None` unless the
    /// range is closing.
    #[must_use]
    pub const fn cpa(&self) -> Option<Cpa> {
        self.cpa
    }

    /// CPA distance, if ahead.
    #[must_use]
    pub fn cpa_distance(&self) -> Option<Distance> {
        self.cpa.map(|cpa| cpa.distance)
    }

    /// TCPA, if ahead.
    #[must_use]
    pub fn tcpa(&self) -> Option<Duration> {
        self.cpa.map(|cpa| cpa.time_to_go)
    }

    /// Bearing at CPA.
    #[must_use]
    pub fn bearing_at_cpa(&self) -> Option<TrueBearing> {
        self.cpa.map(|cpa| cpa.bearing)
    }

    /// Bow crossing range; `None` if the target will not cross ahead.
    #[must_use]
    pub const fn bow_crossing(&self) -> Option<Distance> {
        self.bow_crossing
    }

    /// Risk against the policy.
    #[must_use]
    pub const fn risk(&self) -> CollisionRisk {
        self.risk
    }
}

/// Assesses one encounter from own course and speed, the contact's bearing and
/// range, and the target's course and speed.
///
/// # Errors
///
/// - [`KernelError::OutOfRange`] for a negative range.
/// - [`KernelError::Indeterminate`] if TCPA is too large to represent.
pub fn assess(
    own: Vessel,
    contact: Contact,
    target: Vessel,
    policy: &CpaPolicy,
) -> Result<CollisionAssessment> {
    ensure_range("range", contact.range.nautical_miles(), 0.0, f64::MAX)?;

    let (relative_north, relative_east) = relative_velocity(own, target);
    let relative_speed = math::hypot(relative_north, relative_east);
    let relative_course = Direction::<True>::from_degrees_wrapped(math::to_degrees(math::atan2(
        relative_east,
        relative_north,
    )));

    // Bearing rate: transverse relative velocity over range, rad/h, converted
    // to °/min.
    let (offset_north, offset_east) = (
        contact.range.nautical_miles() * math::cos(contact.bearing.radians()),
        contact.range.nautical_miles() * math::sin(contact.bearing.radians()),
    );
    let range_squared = contact.range.nautical_miles() * contact.range.nautical_miles();
    let drift_per_hour = if range_squared > 0.0 {
        (offset_north * relative_east - offset_east * relative_north) / range_squared
    } else {
        0.0
    };
    let bearing_drift = RateOfTurn::from_degrees_per_minute(if drift_per_hour.is_finite() {
        math::to_degrees(drift_per_hour) / 60.0
    } else {
        0.0
    })?;

    let cpa = match closest_point_of_approach(own, contact, target)? {
        Approach::Closing(cpa) => Some(cpa),
        _ => None,
    };
    let bow_crossing = bow_crossing_range(own, contact, target).ok();

    let risk = match cpa {
        None => CollisionRisk::Opening,
        Some(cpa) if cpa.distance > policy.cpa_limit => CollisionRisk::Passing,
        Some(cpa) if cpa.time_to_go > policy.tcpa_limit => CollisionRisk::Developing,
        Some(_) => CollisionRisk::Dangerous,
    };

    Ok(CollisionAssessment {
        range: contact.range,
        bearing: contact.bearing,
        relative_course,
        relative_speed: Speed::from_knots_unchecked(relative_speed),
        bearing_drift,
        cpa,
        bow_crossing,
        risk,
    })
}

/// Assesses a tracked target against own ship at `now`, using the track's
/// extrapolated position and motion.
///
/// `None` if the track has no motion yet (single radar plot): no course, so no
/// CPA.
///
/// # Errors
///
/// As [`assess`], plus a sailing failure placing the target.
pub fn assess_track(
    own_position: Position,
    own: Vessel,
    track: &TargetTrack,
    now: Instant<Utc>,
    policy: &CpaPolicy,
) -> Result<Option<CollisionAssessment>> {
    let Some(target) = track.as_vessel() else {
        return Ok(None);
    };
    let line = rhumb_line(own_position, track.position_at(now)?)?;
    let contact = Contact {
        bearing: TrueBearing::new(line.initial_course.degrees())?,
        range: line.distance,
    };
    assess(own, contact, target, policy).map(Some)
}

/// One target's assessment.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct TargetAssessment {
    /// Target.
    pub target: TargetId,
    /// Encounter with own ship.
    pub assessment: CollisionAssessment,
}

/// All tracked targets assessed against own ship at one instant.
///
/// Read model built by [`assess_traffic`]. Targets without motion are omitted.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CollisionPicture<const N: usize = MAX_TARGETS> {
    at: Instant<Utc>,
    targets: Inline<TargetAssessment, N>,
}

impl<const N: usize> CollisionPicture<N> {
    /// Time of the picture.
    #[must_use]
    pub const fn at(&self) -> Instant<Utc> {
        self.at
    }

    /// Assessments, in order of first detection.
    #[must_use]
    pub fn targets(&self) -> &[TargetAssessment] {
        &self.targets
    }

    /// Number of assessed targets.
    #[must_use]
    pub const fn len(&self) -> usize {
        self.targets.len()
    }

    /// Whether no target was assessed.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.targets.is_empty()
    }

    /// Assessment for one target, if present.
    #[must_use]
    pub fn target(&self, target: TargetId) -> Option<&TargetAssessment> {
        self.targets.iter().find(|entry| entry.target == target)
    }

    /// Most dangerous target: highest risk, then earliest CPA. `None` if empty.
    #[must_use]
    pub fn most_dangerous(&self) -> Option<&TargetAssessment> {
        self.targets.iter().max_by(|left, right| {
            left.assessment
                .risk
                .cmp(&right.assessment.risk)
                .then_with(|| {
                    // Earlier CPA ranks higher: greater TCPA sorts lower.
                    right
                        .assessment
                        .tcpa()
                        .unwrap_or(Duration::MAX)
                        .cmp(&left.assessment.tcpa().unwrap_or(Duration::MAX))
                })
        })
    }

    /// Targets with risk ≥ `least`, in order of first detection.
    pub fn at_least(&self, least: CollisionRisk) -> impl Iterator<Item = &TargetAssessment> + '_ {
        self.targets
            .iter()
            .filter(move |entry| entry.assessment.risk >= least)
    }
}

/// Assesses every target against own ship from the snapshot, reporting
/// [`TrafficEvent::CpaAlarm`] for each dangerous one.
///
/// Pure function of snapshot and picture: alarms are repeated on every call
/// while the condition holds.
///
/// # Errors
///
/// - [`KernelError::Indeterminate`] if the snapshot has no position or ground
///   track (a stopped ship has a ground track with zero speed).
/// - As [`assess_track`] for any target.
pub fn assess_traffic<const N: usize>(
    state: &NavigationSnapshot,
    traffic: &Traffic<N>,
    policy: &CpaPolicy,
) -> Result<(CollisionPicture<N>, EventList<TrafficEvent, N>)> {
    let observed = state
        .position()
        .ok_or(NavigationError::Kernel(KernelError::Missing {
            what: "the vessel's position",
        }))?;
    let ground = state
        .ground_track()
        .ok_or(NavigationError::Kernel(KernelError::Missing {
            what: "the vessel's course and speed over the ground",
        }))?;
    let now = observed.taken_at();
    let own = Vessel {
        course: ground.course_over_ground,
        speed: ground.speed_over_ground,
    };

    let mut events = EventList::with_capacity();
    let mut targets = Inline::<TargetAssessment, N>::new(TargetAssessment {
        target: TargetId::new(0),
        assessment: CollisionAssessment {
            range: Distance::ZERO,
            bearing: TrueBearing::NORTH,
            relative_course: TrueCourse::NORTH,
            relative_speed: Speed::ZERO,
            bearing_drift: RateOfTurn::ZERO,
            cpa: None,
            bow_crossing: None,
            risk: CollisionRisk::Opening,
        },
    });
    for track in traffic.tracks() {
        let Some(assessment) = assess_track(*observed.value(), own, track, now, policy)? else {
            continue;
        };
        if assessment.risk == CollisionRisk::Dangerous {
            if let Some(cpa) = assessment.cpa {
                events.push(TrafficEvent::CpaAlarm {
                    target: track.target(),
                    cpa: cpa.distance,
                    tcpa: cpa.time_to_go,
                    at: now,
                });
            }
        }
        // The store has room for every track in the picture.
        let _ = targets.push(TargetAssessment {
            target: track.target(),
            assessment,
        });
    }
    Ok((CollisionPicture { at: now, targets }, events))
}

/// Target velocity minus own velocity, north and east, knots.
fn relative_velocity(own: Vessel, target: Vessel) -> (f64, f64) {
    let (own_north, own_east) = own.course.components(own.speed.knots());
    let (target_north, target_east) = target.course.components(target.speed.knots());
    (target_north - own_north, target_east - own_east)
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp, clippy::indexing_slicing)]
mod tests {
    use super::*;
    use crate::{TargetObservation, TrackingPolicy};
    use kinavis_kernel::event::PositionSource;
    use kinavis_kernel::observation::{ObservationStatus, Observed, Quality};
    use kinavis_kernel::snapshot::GroundTrack;

    fn knots(value: f64) -> Speed {
        Speed::from_knots(value).unwrap()
    }

    fn miles(value: f64) -> Distance {
        Distance::from_nautical_miles(value).unwrap()
    }

    fn vessel(course: f64, speed: f64) -> Vessel {
        Vessel {
            course: TrueCourse::new(course).unwrap(),
            speed: knots(speed),
        }
    }

    fn contact(bearing: f64, range: f64) -> Contact {
        Contact {
            bearing: TrueBearing::new(bearing).unwrap(),
            range: miles(range),
        }
    }

    fn policy(cpa: f64, tcpa_minutes: u64) -> CpaPolicy {
        CpaPolicy::new(miles(cpa), Duration::from_secs(tcpa_minutes * 60)).unwrap()
    }

    fn noon() -> Instant<Utc> {
        Instant::from_unix_seconds(1_789_000_000)
    }

    #[test]
    fn a_crossing_target_is_assessed_as_the_plotting_triangle_says() {
        // The `relative_motion` example: 2.59 NM in 27 min.
        let own = vessel(0.0, 15.0);
        let target = vessel(270.0, 15.0);
        let assessment = assess(own, contact(30.0, 10.0), target, &policy(2.0, 30)).unwrap();

        assert!((assessment.cpa_distance().unwrap().nautical_miles() - 2.59).abs() < 0.01);
        assert_eq!(assessment.tcpa().unwrap().as_secs() / 60, 27);
        assert!(assessment.bearing_at_cpa().is_some());
        assert_eq!(assessment.range(), miles(10.0));
        assert_eq!(assessment.bearing(), TrueBearing::new(30.0).unwrap());
        // Relative motion: target 15 kn west minus own 15 kn north = 225° at
        // 21.2 kn.
        assert!((assessment.relative_course().degrees() - 225.0).abs() < 1e-9);
        assert!((assessment.relative_speed().knots() - 21.21).abs() < 0.01);
        // Crosses ahead, passes outside 2 NM.
        assert!(assessment.bow_crossing().unwrap().nautical_miles() > 0.0);
        assert_eq!(assessment.risk(), CollisionRisk::Passing);

        // With a 3 NM limit: dangerous with a 30 min TCPA limit, developing
        // with 20 min.
        let tight = assess(own, contact(30.0, 10.0), target, &policy(3.0, 30)).unwrap();
        assert_eq!(tight.risk(), CollisionRisk::Dangerous);
        let far = assess(own, contact(30.0, 10.0), target, &policy(3.0, 20)).unwrap();
        assert_eq!(far.risk(), CollisionRisk::Developing);
    }

    #[test]
    fn a_steady_bearing_on_a_closing_range_is_a_collision_course() {
        // Both at 10 kn, target on the starboard bow heading west: steady
        // bearing, CPA zero.
        let assessment = assess(
            vessel(0.0, 10.0),
            contact(45.0, 5.0),
            vessel(270.0, 10.0),
            &policy(1.0, 30),
        )
        .unwrap();
        assert!(assessment.bearing_drift().degrees_per_minute().abs() < 1e-9);
        assert!(assessment.cpa_distance().unwrap().nautical_miles() < 1e-9);
        assert_eq!(assessment.risk(), CollisionRisk::Dangerous);
    }

    #[test]
    fn the_bearing_drift_is_signed_the_way_the_bearing_draws() {
        // Dead ahead crossing left to right: bearing draws right.
        let right = assess(
            vessel(0.0, 10.0),
            contact(0.0, 5.0),
            vessel(90.0, 10.0),
            &policy(1.0, 30),
        )
        .unwrap();
        assert!(right.bearing_drift().degrees_per_minute() > 0.0);
        // 10 kn across at 5 NM: 2 rad/h = 1.9°/min.
        assert!((right.bearing_drift().degrees_per_minute() - 1.91).abs() < 0.01);

        let left = assess(
            vessel(0.0, 10.0),
            contact(0.0, 5.0),
            vessel(270.0, 10.0),
            &policy(1.0, 30),
        )
        .unwrap();
        assert!(left.bearing_drift().degrees_per_minute() < 0.0);
    }

    #[test]
    fn an_opening_range_has_no_closest_approach_ahead() {
        // Astern, opposite course.
        let assessment = assess(
            vessel(0.0, 10.0),
            contact(180.0, 3.0),
            vessel(180.0, 10.0),
            &policy(1.0, 30),
        )
        .unwrap();
        assert_eq!(assessment.cpa(), None);
        assert_eq!(assessment.tcpa(), None);
        assert_eq!(assessment.bow_crossing(), None);
        assert_eq!(assessment.risk(), CollisionRisk::Opening);

        // No relative motion counts as opening.
        let still = assess(
            vessel(0.0, 10.0),
            contact(90.0, 3.0),
            vessel(0.0, 10.0),
            &policy(1.0, 30),
        )
        .unwrap();
        assert_eq!(still.risk(), CollisionRisk::Opening);
        assert_eq!(still.relative_speed(), Speed::ZERO);
    }

    #[test]
    fn a_policy_with_a_negative_limit_is_refused() {
        assert!(matches!(
            CpaPolicy::new(miles(-1.0), Duration::from_secs(1800)).unwrap_err(),
            NavigationError::Kernel(KernelError::OutOfRange {
                parameter: "CPA limit",
                ..
            })
        ));
        assert!(CpaPolicy::new(Distance::ZERO, Duration::ZERO).is_ok());
        assert!(assess(
            vessel(0.0, 10.0),
            contact(90.0, -3.0),
            vessel(0.0, 10.0),
            &policy(1.0, 30),
        )
        .is_err());
    }

    /// Own ship at 50°N 1°W, heading north at 10 kn.
    fn own_ship() -> NavigationSnapshot {
        NavigationSnapshot::EMPTY
            .with_position(
                Observed::new(
                    Position::from_degrees(50.0, -1.0).unwrap(),
                    noon(),
                    Quality::<Distance>::new(ObservationStatus::Valid),
                ),
                PositionSource::Gnss,
            )
            .with_ground_track(GroundTrack {
                course_over_ground: TrueCourse::NORTH,
                speed_over_ground: knots(10.0),
            })
    }

    /// Target with reported course and speed, 5 NM off on the given bearing.
    fn reporting(id: u32, bearing: f64, course: f64, speed: f64) -> TargetObservation {
        let here = Position::from_degrees(50.0, -1.0).unwrap();
        let there = kinavis::sailings::rhumb_destination(
            here,
            TrueCourse::new(bearing).unwrap(),
            miles(5.0),
        )
        .unwrap();
        TargetObservation::new(TargetId::new(id), there, noon()).with_ground_track(GroundTrack {
            course_over_ground: TrueCourse::new(course).unwrap(),
            speed_over_ground: knots(speed),
        })
    }

    fn traffic() -> Traffic {
        Traffic::new(
            TrackingPolicy::new(
                1,
                Duration::from_secs(30),
                Duration::from_secs(180),
                knots(60.0),
            )
            .unwrap(),
        )
    }

    #[test]
    fn the_picture_assesses_every_moving_target_and_alarms_on_the_dangerous() {
        let mut traffic = traffic();
        // Starboard bow, heading west at own speed: collision course.
        let _ = traffic.ingest(reporting(1, 45.0, 270.0, 10.0)).unwrap();
        // Ahead, same course, faster: opening.
        let _ = traffic.ingest(reporting(2, 0.0, 0.0, 15.0)).unwrap();
        // Abeam to port, heading north at own speed: no relative motion.
        let _ = traffic.ingest(reporting(3, 270.0, 0.0, 10.0)).unwrap();
        // Single plot, nothing reported: no motion, not assessed.
        let _ = traffic
            .ingest(TargetObservation::new(
                TargetId::new(4),
                Position::from_degrees(50.1, -0.9).unwrap(),
                noon(),
            ))
            .unwrap();

        let (picture, events) = assess_traffic(&own_ship(), &traffic, &policy(1.0, 30)).unwrap();
        assert_eq!(picture.at(), noon());
        assert_eq!(picture.len(), 3);
        assert!(picture.target(TargetId::new(4)).is_none());

        let first = picture.target(TargetId::new(1)).unwrap();
        assert_eq!(first.assessment.risk(), CollisionRisk::Dangerous);
        assert!(first.assessment.cpa_distance().unwrap().nautical_miles() < 0.05);
        assert_eq!(
            picture.target(TargetId::new(2)).unwrap().assessment.risk(),
            CollisionRisk::Opening
        );
        assert_eq!(
            picture.target(TargetId::new(3)).unwrap().assessment.risk(),
            CollisionRisk::Opening
        );
        assert_eq!(picture.most_dangerous().unwrap().target, TargetId::new(1));
        assert_eq!(picture.at_least(CollisionRisk::Passing).count(), 1);
        assert_eq!(picture.at_least(CollisionRisk::Opening).count(), 3);

        assert_eq!(events.len(), 1);
        assert!(matches!(
            events[0],
            TrafficEvent::CpaAlarm { target, cpa, tcpa, at }
                if target == TargetId::new(1)
                    && cpa.nautical_miles() < 0.05
                    && tcpa.as_secs() / 60 == 21
                    && at == noon()
        ));
    }

    #[test]
    fn a_track_is_assessed_where_it_is_reckoned_to_be() {
        let mut traffic = traffic();
        let _ = traffic.ingest(reporting(1, 90.0, 270.0, 10.0)).unwrap();
        let track = traffic.track(TargetId::new(1)).unwrap();
        let own = vessel(0.0, 10.0);
        let here = Position::from_degrees(50.0, -1.0).unwrap();

        let now = assess_track(here, own, track, noon(), &policy(1.0, 30))
            .unwrap()
            .unwrap();
        assert!((now.range().nautical_miles() - 5.0).abs() < 1e-6);
        // 12 min later: 2 NM closer and still closing.
        let later = noon().checked_add(Duration::from_secs(720)).unwrap();
        let then = assess_track(here, own, track, later, &policy(1.0, 30))
            .unwrap()
            .unwrap();
        assert!((then.range().nautical_miles() - 3.0).abs() < 0.01);
        assert!(then.tcpa().unwrap() < now.tcpa().unwrap());
    }

    #[test]
    fn own_ship_must_be_somewhere_and_moving() {
        let traffic = traffic();
        assert!(matches!(
            assess_traffic(&NavigationSnapshot::EMPTY, &traffic, &policy(1.0, 30)).unwrap_err(),
            NavigationError::Kernel(KernelError::Missing {
                what: "the vessel's position"
            })
        ));
        let anchored = NavigationSnapshot::EMPTY.with_position(
            Observed::new(
                Position::from_degrees(50.0, -1.0).unwrap(),
                noon(),
                Quality::<Distance>::new(ObservationStatus::Valid),
            ),
            PositionSource::Gnss,
        );
        assert!(matches!(
            assess_traffic(&anchored, &traffic, &policy(1.0, 30)).unwrap_err(),
            NavigationError::Kernel(KernelError::Missing { .. })
        ));
        // Empty picture yields an empty result, not an error.
        let (picture, events) = assess_traffic(&own_ship(), &traffic, &policy(1.0, 30)).unwrap();
        assert!(picture.is_empty());
        assert!(picture.most_dangerous().is_none());
        assert!(events.is_empty());
    }
}
