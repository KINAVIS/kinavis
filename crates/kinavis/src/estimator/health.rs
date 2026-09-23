//! Source health and overall integrity verdicts.

use core::time::Duration;

use crate::error::{KernelError, NavigationError, Result};
use crate::event::{EventList, NavigationEvent, NavigationIntegrity, SensorHealth, SensorId};
use crate::inline::Inline;
use crate::state::NavigationState;
use crate::time::{Instant, Utc};
use crate::units::Distance;

/// Maximum number of distinct sources tracked.
///
/// Sources are distinguished by the [`SensorId`] of their observations. A
/// bridge has a handful (two receivers, gyro, log, radar); beyond this limit a
/// new source is refused rather than dropping an existing one.
pub const MAX_SENSORS: usize = 8;

#[derive(Debug, Clone, Copy, PartialEq)]
struct Record {
    sensor: SensorId,
    health: SensorHealth,
    /// Consecutive rejections while healthy, or consecutive acceptances while
    /// suspect: the run that changes the verdict.
    run: u8,
}

/// One verdict per source.
#[derive(Debug, Clone)]
pub(super) struct Sensors {
    records: Inline<Record, MAX_SENSORS>,
}

impl Sensors {
    pub(super) fn new() -> Self {
        Self {
            records: Inline::new(Record {
                sensor: SensorId::named(""),
                health: SensorHealth::Healthy,
                run: 0,
            }),
        }
    }

    /// Verdict on a source, if heard from.
    pub(super) fn health(&self, sensor: SensorId) -> Option<SensorHealth> {
        self.records
            .iter()
            .find(|record| record.sensor == sensor)
            .map(|record| record.health)
    }

    /// Records a gate acceptance or rejection for a source and reports any
    /// verdict change.
    ///
    /// `suspect_after` consecutive rejections make a source suspect; the same
    /// number of consecutive acceptances clear it, so a source rejected half
    /// the time does not flap. Zero disables the assessment.
    ///
    /// # Errors
    ///
    /// [`KernelError::CapacityExceeded`] for a source beyond [`MAX_SENSORS`].
    pub(super) fn note(
        &mut self,
        sensor: SensorId,
        accepted: bool,
        suspect_after: u8,
        at: Instant<Utc>,
        events: &mut EventList,
    ) -> Result<()> {
        let known = self
            .records
            .iter()
            .position(|record| record.sensor == sensor);
        let index = if let Some(index) = known {
            index
        } else {
            self.records
                .push(Record {
                    sensor,
                    health: SensorHealth::Healthy,
                    run: 0,
                })
                .map_err(|_| {
                    NavigationError::Kernel(KernelError::CapacityExceeded {
                        context: "the estimator's sources",
                        needed: MAX_SENSORS.saturating_add(1),
                        capacity: MAX_SENSORS,
                    })
                })?;
            self.records.len().saturating_sub(1)
        };
        let Some(record) = self.records.as_mut_slice().get_mut(index) else {
            return Ok(());
        };
        let (against, towards) = match record.health {
            SensorHealth::Healthy => (accepted, SensorHealth::Suspect),
            _ => (!accepted, SensorHealth::Healthy),
        };
        record.run = if against {
            0
        } else {
            record.run.saturating_add(1)
        };
        if suspect_after > 0 && record.run >= suspect_after {
            events.push(NavigationEvent::SensorHealthChanged {
                sensor: record.sensor,
                from: record.health,
                to: towards,
                at,
            });
            record.health = towards;
            record.run = 0;
        }
        Ok(())
    }
}

/// Vessel limits on overall integrity.
#[derive(Debug, Clone, Copy)]
pub(super) struct IntegrityLimits {
    pub(super) alert_limit: Distance,
    pub(super) max_unaided: Duration,
}

/// Integrity verdict for a state, given the time the position was last fixed by
/// an absolute observation.
pub(super) fn assess(
    state: &NavigationState,
    last_fix: Option<Instant<Utc>>,
    now: Instant<Utc>,
    limits: &IntegrityLimits,
) -> NavigationIntegrity {
    if state.horizontal_error().semi_major() > limits.alert_limit {
        return NavigationIntegrity::Exceeded;
    }
    let aided = last_fix.is_some_and(|fix| {
        now.checked_duration_since(fix)
            .is_some_and(|since| since <= limits.max_unaided)
    });
    if aided {
        NavigationIntegrity::Nominal
    } else {
        NavigationIntegrity::DeadReckoning
    }
}
