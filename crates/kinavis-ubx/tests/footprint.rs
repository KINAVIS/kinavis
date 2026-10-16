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
fn messages_stay_small_and_the_deframer_is_its_buffer() {
    within::<kinavis_ubx::NavPvt>(192);
    within::<kinavis_ubx::Message<'static>>(192);
    // N bytes of payload and a few words of state.
    within::<kinavis_ubx::Deframer<1024>>(1024 + 32);
}
