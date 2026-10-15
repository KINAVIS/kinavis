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
fn frames_and_messages_stay_small() {
    within::<kinavis_mavlink::Frame>(296);
    within::<kinavis_mavlink::Deframer>(296);
    within::<kinavis_mavlink::Message>(192);
}
