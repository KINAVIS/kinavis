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
fn sentences_and_the_sky_assembler_stay_within_budget() {
    within::<kinavis_nmea0183::Sentence>(224);
    // The satellite set, MAX_SATELLITES views, and the talkers in progress.
    within::<kinavis_nmea0183::GsvAssembler>(5120);
}
