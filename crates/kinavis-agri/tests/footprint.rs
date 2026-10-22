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
fn planes_lines_and_trackers_stay_small() {
    // A plane is its projection: a transverse Mercator with its series.
    within::<kinavis_agri::FieldPlane>(320);
    within::<kinavis_agri::AbLine>(352);
    within::<kinavis_agri::PivotLine>(352);
    within::<kinavis_agri::CurveLine<64>>(1360);
    within::<kinavis_agri::ContourRecorder<256>>(4440);
    within::<kinavis_agri::Field<64, 32, 4>>(3456);
    within::<kinavis_agri::PassPattern>(16);
    within::<kinavis_agri::PassTracker>(48);
    within::<kinavis_agri::PassView>(48);
    within::<kinavis_agri::VehicleGeometry>(64);
    within::<kinavis_agri::PurePursuit>(40);
    within::<kinavis_agri::Stanley>(16);
    within::<kinavis_agri::TurnPath>(384);
    within::<kinavis_agri::ReedsSheppPath>(432);
    within::<kinavis_agri::GearedSection>(448);
    // Two bits a cell: a 500 m square at 25 cm is 1 MB.
    within::<kinavis_agri::CoverageGrid<32, 2000>>(1_024_400);
    within::<kinavis_agri::Boom<16>>(152);
    within::<kinavis_agri::SectionControl>(24);
    // 24 bytes an angle: instant and curvature.
    within::<kinavis_agri::BicycleMotion<64>>(1600);
}
