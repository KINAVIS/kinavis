//! The vessels a Signal K server knows, as the last value of each path this
//! crate uses.

use std::collections::BTreeMap;

use kinavis_colregs::VesselCategory;
use kinavis_kernel::angle::TrueCourse;
use kinavis_kernel::position::Position;
use kinavis_kernel::time::{Instant, Utc};
use kinavis_kernel::units::Speed;
use serde_json::Value;

use crate::delta::Delta;
use crate::time::parse_timestamp;

/// Context of own vessel in Signal K.
pub const SELF_CONTEXT: &str = "vessels.self";

/// Last known state of one vessel.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct VesselState {
    /// Name, from the vessel's root `name`.
    pub name: Option<String>,
    /// MMSI, from the vessel's root `mmsi`.
    pub mmsi: Option<String>,
    /// `navigation.position`.
    pub position: Option<Position>,
    /// `navigation.courseOverGroundTrue`.
    pub course: Option<TrueCourse>,
    /// `navigation.speedOverGround`.
    pub speed: Option<Speed>,
    /// `navigation.headingTrue`.
    pub heading: Option<TrueCourse>,
    /// `navigation.state`, as sent.
    pub state: Option<String>,
    /// `design.aisShipType.id`.
    pub ship_type: Option<u32>,
    /// Latest timestamp of any update.
    pub last_seen: Option<Instant<Utc>>,
}

impl VesselState {
    /// Name, else MMSI, else the context it is known by.
    #[must_use]
    pub fn label<'a>(&'a self, context: &'a str) -> &'a str {
        self.name
            .as_deref()
            .or(self.mmsi.as_deref())
            .unwrap_or(context)
    }

    /// COLREGs category from `navigation.state`, else from the AIS ship type;
    /// `None` if neither says. A vessel at anchor, moored or aground is not
    /// under way and has no category under the steering rules.
    #[must_use]
    pub fn category(&self) -> Option<Category> {
        if let Some(state) = self.state.as_deref() {
            if let Some(category) = category_of_state(state) {
                return Some(category);
            }
        }
        match self.ship_type? {
            30 => Some(Category::UnderWay(VesselCategory::Fishing)),
            36 => Some(Category::UnderWay(VesselCategory::Sailing { tack: None })),
            _ => None,
        }
    }

    fn apply(&mut self, path: &str, value: &Value) {
        match path {
            "" => {
                if let Some(name) = value.get("name").and_then(Value::as_str) {
                    self.name = Some(name.trim().to_owned());
                }
                if let Some(mmsi) = value.get("mmsi").and_then(Value::as_str) {
                    self.mmsi = Some(mmsi.to_owned());
                }
            }
            "name" => self.name = value.as_str().map(|name| name.trim().to_owned()),
            "mmsi" => self.mmsi = value.as_str().map(str::to_owned),
            "navigation.position" => {
                self.position = value
                    .get("latitude")
                    .and_then(Value::as_f64)
                    .zip(value.get("longitude").and_then(Value::as_f64))
                    .and_then(|(latitude, longitude)| {
                        Position::from_degrees(latitude, longitude).ok()
                    });
            }
            "navigation.courseOverGroundTrue" => self.course = value.as_f64().and_then(direction),
            "navigation.headingTrue" => self.heading = value.as_f64().and_then(direction),
            "navigation.speedOverGround" => {
                self.speed = value
                    .as_f64()
                    .filter(|speed| *speed >= 0.0)
                    .and_then(|speed| Speed::from_metres_per_second(speed).ok());
            }
            "navigation.state" => self.state = value.as_str().map(str::to_owned),
            "design.aisShipType" => {
                self.ship_type = value
                    .get("id")
                    .and_then(Value::as_u64)
                    .and_then(|id| u32::try_from(id).ok());
            }
            _ => {}
        }
    }
}

/// What a vessel is, for the steering rules.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Category {
    /// Under way, of this category.
    UnderWay(VesselCategory),
    /// At anchor, moored, aground or otherwise not under way.
    NotUnderWay,
}

/// Category of a Signal K `navigation.state`; `None` for states that do not
/// say (motoring counts as power-driven).
fn category_of_state(state: &str) -> Option<Category> {
    let category = match state {
        "motoring" | "towing < 200m" | "towing > 200m" | "pushing" | "pilotage" => {
            VesselCategory::PowerDriven
        }
        "sailing" => VesselCategory::Sailing { tack: None },
        "fishing" | "fishing-hampered" | "trawling" | "trawling-shooting" | "trawling-hauling" => {
            VesselCategory::Fishing
        }
        "constrained by draft" => VesselCategory::ConstrainedByDraught,
        "not under command" => VesselCategory::NotUnderCommand,
        "anchored" | "moored" | "aground" | "not-under-way" => return Some(Category::NotUnderWay),
        state if state.starts_with("restricted manouverability") || state == "mine clearance" => {
            VesselCategory::RestrictedInAbilityToManoeuvre
        }
        _ => return None,
    };
    Some(Category::UnderWay(category))
}

/// True direction of an angle in radians, wrapped to `[0, 360)`.
fn direction(radians: f64) -> Option<TrueCourse> {
    radians
        .is_finite()
        .then(|| TrueCourse::from_degrees_wrapped(radians.to_degrees()))
}

/// Every vessel seen, by context.
#[derive(Debug, Clone, Default)]
pub struct Picture {
    vessels: BTreeMap<String, VesselState>,
    own: Option<String>,
}

impl Picture {
    /// Empty picture; own vessel is known as [`SELF_CONTEXT`] until
    /// [`Picture::set_own_context`] names it.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Names own vessel's full context, `vessels.urn:mrn:imo:mmsi:…`, as the
    /// server writes it in deltas. Values already held under either name are
    /// merged.
    pub fn set_own_context(&mut self, context: &str) {
        self.own = Some(context.to_owned());
        if let Some(held) = self.vessels.remove(SELF_CONTEXT) {
            self.vessels.entry(context.to_owned()).or_insert(held);
        }
    }

    /// Whether own vessel's full context is known, named or learnt.
    #[must_use]
    pub const fn own_is_named(&self) -> bool {
        self.own.is_some()
    }

    /// Context own vessel is held under.
    #[must_use]
    pub fn own_context(&self) -> &str {
        self.own.as_deref().unwrap_or(SELF_CONTEXT)
    }

    /// Whether a context is own vessel.
    #[must_use]
    pub fn is_own(&self, context: &str) -> bool {
        context == SELF_CONTEXT || context == self.own_context()
    }

    /// Applies a delta. Paths this crate does not use, and values it cannot
    /// read, are ignored.
    ///
    /// Until [`Picture::set_own_context`] names own vessel, the first vessel
    /// context with a position from own sensors (see
    /// [`Update::is_own_sensor`]) is taken as own vessel.
    ///
    /// [`Update::is_own_sensor`]: crate::Update::is_own_sensor
    pub fn apply(&mut self, delta: &Delta) {
        if self.own.is_none() {
            if let Some(context) = delta.context.as_deref() {
                let own_position = delta.updates.iter().any(|update| {
                    update.is_own_sensor()
                        && update
                            .values
                            .iter()
                            .any(|value| value.path == "navigation.position")
                });
                if own_position && context.starts_with("vessels.") && context != SELF_CONTEXT {
                    self.set_own_context(context);
                }
            }
        }
        let context = match delta.context.as_deref() {
            None => self.own_context().to_owned(),
            Some(context) if self.is_own(context) => self.own_context().to_owned(),
            Some(context) => context.to_owned(),
        };
        if !context.starts_with("vessels.") {
            return;
        }
        let vessel = self.vessels.entry(context).or_default();
        for update in &delta.updates {
            let at = update.timestamp.as_deref().and_then(parse_timestamp);
            if let Some(at) = at {
                vessel.last_seen = Some(vessel.last_seen.map_or(at, |seen| seen.max(at)));
            }
            for value in &update.values {
                vessel.apply(&value.path, &value.value);
            }
        }
    }

    /// Own vessel, if seen.
    #[must_use]
    pub fn own(&self) -> Option<&VesselState> {
        self.vessels.get(self.own_context())
    }

    /// Other vessels, by context.
    pub fn targets(&self) -> impl Iterator<Item = (&str, &VesselState)> + '_ {
        self.vessels
            .iter()
            .filter(|(context, _)| !self.is_own(context))
            .map(|(context, vessel)| (context.as_str(), vessel))
    }

    /// Number of vessels, own included.
    #[must_use]
    pub fn len(&self) -> usize {
        self.vessels.len()
    }

    /// Whether no vessel has been seen.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.vessels.is_empty()
    }
}
