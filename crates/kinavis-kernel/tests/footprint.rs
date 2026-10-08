//! Size budgets of the kernel's inline collections and of the values parsers
//! hand on.
//!
//! Sizes are fixed at compile time; these are the budgets quoted in the
//! documentation, and exceeding one is a deliberate change made here.

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
fn the_satellites_in_view_fit_their_budget() {
    // MAX_SATELLITES views of 72 bytes.
    within::<kinavis_kernel::SatelliteSet>(5120);
    within::<kinavis_kernel::SatelliteView>(72);
}

#[test]
fn a_fix_stays_small() {
    within::<kinavis_kernel::GnssFix>(160);
    within::<kinavis_kernel::Kinematics3<kinavis_kernel::Ned>>(48);
}
