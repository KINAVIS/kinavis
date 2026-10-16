//! Size budgets: fixed at compile time, quoted in the documentation;
//! exceeding one is a deliberate change made here.

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
fn a_grid_is_a_view_not_a_copy() {
    // The samples stay where they are; the grid holds a slice and a header.
    within::<kinavis_geoid::GridGeoid<'static>>(64);
}
