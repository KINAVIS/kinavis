//! Size budgets of the aggregates.
//!
//! Aggregates store everything inline, so their size is fixed at compile time:
//! memory use is known before run time, at the cost of kilobyte-sized values
//! that belong behind a reference or in a `static`. These are the budgets
//! quoted in the documentation; exceeding one changes what callers must
//! provision and must be done here deliberately.

#![allow(clippy::unwrap_used)]

use core::mem::size_of;

/// Fails with the type, its size and its budget.
fn within<T>(budget: usize) {
    let size = size_of::<T>();
    assert!(
        size <= budget,
        "{} is {size} bytes, over its budget of {budget}",
        core::any::type_name::<T>()
    );
}

#[test]
fn the_board_stays_within_its_budget() {
    within::<kinavis_alerts::AlertManager<kinavis_alerts::StandardPolicy>>(3584);
    within::<kinavis_alerts::Alert>(96);
    within::<kinavis_alerts::AlertChange>(128);
}
