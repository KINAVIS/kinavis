//! The Signal K delta: a context, and updates of paths within it.
//!
//! Only the fields this crate reads or writes are modelled; anything else in
//! a delta is ignored on reading.

use serde::{Deserialize, Serialize};
use serde_json::Value;

/// One delta message.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Delta {
    /// Vessel the updates are about, `vessels.<id>`; absent means own vessel.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub context: Option<String>,
    /// Updates, each from one source at one time.
    #[serde(default)]
    pub updates: Vec<Update>,
}

/// Values from one source at one time.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Update {
    /// RFC 3339 time of the values; the server fills it in on emission.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub timestamp: Option<String>,
    /// Where the values came from: for a device, `type` with `sentence`
    /// (NMEA 0183) or `pgn` (NMEA 2000). The server fills it in.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub source: Option<Value>,
    /// Path and value pairs.
    #[serde(default)]
    pub values: Vec<PathValue>,
}

/// A value at a path.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PathValue {
    /// Dotted path, `navigation.position`; empty for the vessel's root
    /// properties (`name`, `mmsi`).
    pub path: String,
    /// JSON value; `null` clears it.
    pub value: Value,
}

impl Delta {
    /// Delta of one value at one path in a context.
    #[must_use]
    pub fn single(context: Option<String>, path: &str, value: Value) -> Self {
        Self {
            context,
            updates: vec![Update {
                timestamp: None,
                source: None,
                values: vec![PathValue {
                    path: path.to_owned(),
                    value,
                }],
            }],
        }
    }
}

/// NMEA 2000 parameter groups that carry other vessels: the AIS messages.
const AIS_PGNS: [u64; 11] = [
    129_038, 129_039, 129_040, 129_041, 129_793, 129_794, 129_798, 129_801, 129_802, 129_809,
    129_810,
];

impl Update {
    /// Whether the values come from own vessel's own sensors: an NMEA 0183
    /// sentence other than AIS `VDM`, or an NMEA 2000 parameter group other
    /// than AIS. Signal K puts such values in own vessel's context; only AIS
    /// describes other vessels. `false` when the source does not say.
    #[must_use]
    pub fn is_own_sensor(&self) -> bool {
        let Some(source) = &self.source else {
            return false;
        };
        if let Some(sentence) = source.get("sentence").and_then(Value::as_str) {
            return sentence != "VDM";
        }
        source
            .get("pgn")
            .and_then(Value::as_u64)
            .is_some_and(|pgn| !AIS_PGNS.contains(&pgn))
    }
}
