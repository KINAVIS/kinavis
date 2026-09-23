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
fn the_aggregates_stay_within_their_budgets() {
    // Route or schedule: ~2 KiB, MAX_WAYPOINTS positions.
    within::<kinavis::Route>(2560);
    within::<kinavis::schedule::RouteSchedule>(2560);
    // Deviation table: MAX_TABLE_NODES nodes.
    within::<kinavis::DeviationTable>(1280);
    // Estimator: MAX_HISTORY beliefs for late observations.
    within::<kinavis::estimator::Estimator<kinavis::estimator::SteadyMotion>>(9216);
    within::<kinavis::state::NavigationState>(512);
    within::<kinavis::gnss_intake::GnssIntake>(320);
}

#[test]
fn the_values_that_travel_stay_small() {
    // Snapshot: read by displays every tick.
    within::<kinavis::NavigationSnapshot>(192);
    // Event list: a few events, inline.
    within::<kinavis::EventList>(640);
    // Error or event: one cache line.
    within::<kinavis::NavigationEvent>(64);
    within::<kinavis::KernelError>(64);
    within::<kinavis::NavigationError>(64);
    within::<kinavis::event::GuidanceEvent>(64);
    within::<kinavis::event::ClearanceEvent>(64);
    within::<kinavis::event::AnchorEvent>(64);
}
