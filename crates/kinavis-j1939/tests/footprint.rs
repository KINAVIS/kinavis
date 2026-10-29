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
fn identifiers_claims_and_transport_stay_small() {
    within::<kinavis_j1939::CanId>(4);
    within::<kinavis_j1939::Frame>(16);
    within::<kinavis_j1939::Name>(8);
    within::<kinavis_j1939::AddressClaimer>(96);
    // A session is its buffer and a few dozen bytes of state.
    within::<kinavis_j1939::TransportReceiver<4, 1785>>(7_320);
    within::<kinavis_j1939::TransportSender<1785>>(1_840);
    within::<kinavis_j1939::ExtendedReceiver<1, 65_536>>(65_600);
    // The data is borrowed.
    within::<kinavis_j1939::ExtendedSender<'static>>(48);
}
