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
fn grids_fixes_and_legs_stay_small() {
    within::<kinavis_land::MapGrid>(328);
    within::<kinavis_land::GridPoint>(16);
    within::<kinavis_land::OnGrid>(32);
    within::<kinavis_land::BearingFrame>(16);
    within::<kinavis_land::PlaneFix>(48);
    within::<kinavis_land::ThreePointFix>(40);
    within::<kinavis_land::Leg>(16);
    within::<kinavis_land::PaceCount>(8);
    // 8 candidates a fix over a window of 16 fixes.
    within::<kinavis_land::MapMatcher<8, 16>>(10_496);
    within::<kinavis_land::Candidate>(56);
}
