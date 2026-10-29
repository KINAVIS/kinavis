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
fn projections_and_references_stay_small() {
    within::<kinavis_grid::TransverseMercator>(256);
    within::<kinavis_grid::ProjectedCrs>(320);
    within::<kinavis_grid::Mgrs>(32);
    within::<kinavis_grid::Gridded>(48);
    within::<kinavis_grid::Tile>(12);
    within::<kinavis_grid::TilePoint>(32);
    within::<kinavis_grid::Quadkey>(31);
}
