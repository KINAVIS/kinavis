//! Recent beliefs, kept for late observations.

use crate::state::NavigationState;
use crate::time::{Instant, Utc};

/// Number of distinct instants kept.
///
/// A late observation can be folded in only if the history reaches its time.
/// Beliefs at one instant share an entry: at 10 Hz the history covers 1.6 s, at
/// 1 Hz 16 s — longer than receiver latency, shorter than an outage.
pub const MAX_HISTORY: usize = 16;

/// Beliefs at the last [`MAX_HISTORY`] distinct instants, oldest first, last is
/// current.
///
/// Fixed capacity, no allocation, never empty. All entries share one local
/// frame; moving the anchor resets the history.
#[derive(Debug, Clone)]
pub(super) struct History {
    states: [NavigationState; MAX_HISTORY],
    len: usize,
}

impl History {
    /// History with one belief.
    pub(super) const fn starting_with(state: &NavigationState) -> Self {
        Self {
            states: [*state; MAX_HISTORY],
            len: 1,
        }
    }

    /// Current belief.
    pub(super) fn current(&self) -> &NavigationState {
        // Never empty: last index is `len − 1`, `len ≥ 1`.
        self.states
            .get(self.len.saturating_sub(1))
            .unwrap_or(&self.states[0])
    }

    /// Makes a belief current: replaces the current one at the same instant,
    /// otherwise appends, evicting the oldest when full.
    ///
    /// An earlier belief is a caller error and still replaces the current one,
    /// since the history must end with the current belief.
    pub(super) fn record(&mut self, state: &NavigationState) {
        if state.valid_at() <= self.current().valid_at() {
            self.replace_current(state);
            return;
        }
        if self.len == MAX_HISTORY {
            for index in 1..MAX_HISTORY {
                if let Some(&next) = self.states.get(index) {
                    if let Some(slot) = self.states.get_mut(index.wrapping_sub(1)) {
                        *slot = next;
                    }
                }
            }
            self.len = MAX_HISTORY.saturating_sub(1);
        }
        if let Some(slot) = self.states.get_mut(self.len) {
            *slot = *state;
            self.len = self.len.saturating_add(1);
        }
    }

    /// Clears all but this belief.
    pub(super) fn restart_with(&mut self, state: &NavigationState) {
        if let Some(slot) = self.states.get_mut(0) {
            *slot = *state;
        }
        self.len = 1;
    }

    /// Beliefs from the last one at or before `moment` to the current one; `None`
    /// if the history does not reach that far.
    pub(super) fn reaching(&self, moment: Instant<Utc>) -> Option<&[NavigationState]> {
        let held = self.states.get(..self.len)?;
        let start = held.iter().rposition(|state| state.valid_at() <= moment)?;
        held.get(start..)
    }

    fn replace_current(&mut self, state: &NavigationState) {
        if let Some(slot) = self.states.get_mut(self.len.saturating_sub(1)) {
            *slot = *state;
        }
    }
}
