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
fn tiles_and_results_stay_small() {
    // A tile is a borrowed slice, its side and its corner.
    within::<kinavis_terrain::HgtTile<'static>>(32);
    within::<kinavis_terrain::TileCorner>(4);
    within::<kinavis_terrain::SlopeAspect>(48);
    within::<kinavis_terrain::ProfilePoint>(40);
    within::<kinavis_terrain::ProfileSummary>(80);
    within::<kinavis_terrain::Sight>(40);
    within::<kinavis_terrain::WalkingPace>(16);
}
