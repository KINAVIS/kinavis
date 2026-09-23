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
fn the_picture_is_large_and_its_projections_are_not() {
    // MAX_TARGETS tracks × MAX_TRACK_HISTORY fixes: keep behind a reference or
    // in a `static`.
    within::<kinavis_traffic::Traffic>(16 * 1024);
    within::<kinavis_traffic::TargetTrack>(512);
    // Projections passed to displays and assessments are much smaller.
    within::<kinavis_traffic::TrafficView>(3072);
    within::<kinavis_traffic::CollisionPicture>(4096);
    within::<kinavis_traffic::TrafficEvent>(64);
}
