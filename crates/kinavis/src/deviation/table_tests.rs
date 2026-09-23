//! Tests for [`super`], kept separate to bound the size of `table.rs`.

#![allow(clippy::unwrap_used, clippy::float_cmp, clippy::indexing_slicing)]

use super::*;

#[test]
fn default_table_has_thirty_six_nodes() {
    let table = DeviationTable::default();
    assert_eq!(table.len(), STANDARD_TABLE_LEN);
    assert_eq!(table.deviation_at_node(0).unwrap().degrees(), 0.0);
    assert_eq!(table.deviation_at_node(350).unwrap().degrees(), 0.0);
    assert!(table.deviation_at_node(5).is_none());
}

#[test]
fn from_step_rejects_zero_and_negative() {
    // Neither may abort the process or silently build a one-node table.
    assert_eq!(
        DeviationTable::from_step(0).unwrap_err(),
        NavigationError::InvalidStep { step: 0 }
    );
    assert_eq!(
        DeviationTable::from_step(-10).unwrap_err(),
        NavigationError::InvalidStep { step: -10 }
    );
    assert!(DeviationTable::from_step(181).is_err());
    assert_eq!(DeviationTable::from_step(180).unwrap().len(), 2);
}

#[test]
fn from_step_reports_a_swing_too_fine_to_store() {
    // 5° is the finest step that fits, and fills the table exactly.
    assert_eq!(DeviationTable::from_step(5).unwrap().len(), MAX_TABLE_NODES);
    assert!(matches!(
        DeviationTable::from_step(4).unwrap_err(),
        NavigationError::Kernel(KernelError::CapacityExceeded {
            needed: 90,
            capacity: MAX_TABLE_NODES,
            ..
        })
    ));
    assert!(matches!(
        DeviationTable::from_step(1).unwrap_err(),
        NavigationError::Kernel(KernelError::CapacityExceeded { needed: 360, .. })
    ));
}

#[test]
fn from_step_never_duplicates_north() {
    let table = DeviationTable::from_step(45).unwrap();
    assert_eq!(table.len(), 8);
    assert_eq!(table.nodes().last().unwrap().course(), 315);
}

#[test]
fn empty_and_tiny_tables_are_rejected_not_paniced() {
    // An empty table must not reach a subtract overflow.
    assert!(matches!(
        DeviationTable::from_pairs(&[]).unwrap_err(),
        NavigationError::Kernel(KernelError::InsufficientData { found: 0, .. })
    ));
    assert!(matches!(
        DeviationTable::from_pairs(&[(0, 1.0)]).unwrap_err(),
        NavigationError::Kernel(KernelError::InsufficientData { found: 1, .. })
    ));
}

#[test]
fn negative_courses_normalise_the_euclidean_way() {
    // `%` would leave this as the unreachable key -350.
    let table = DeviationTable::from_pairs(&[(-350, 1.0), (180, 2.0)]).unwrap();
    assert_eq!(table.nodes().first().unwrap().course(), 10);
    assert_eq!(table.deviation_at_node(10).unwrap().degrees(), 1.0);
}

#[test]
fn duplicate_courses_are_rejected() {
    assert_eq!(
        DeviationTable::from_pairs(&[(10, 1.0), (370, 2.0), (180, 0.0)]).unwrap_err(),
        NavigationError::DuplicateCourse { course: 10 }
    );
}

#[test]
fn non_finite_deviations_are_rejected() {
    assert!(DeviationTable::from_pairs(&[(0, f64::NAN), (10, 0.0)]).is_err());
    assert!(DeviationTable::from_pairs(&[(0, f64::INFINITY), (10, 0.0)]).is_err());
    // The point setter takes a `Deviation`, so an invalid value cannot be
    // passed.
    assert!(Deviation::new(f64::NAN).is_err());
    assert!(Deviation::new(1e9).is_err());
}

#[test]
fn from_deviations_demands_the_full_swing() {
    // Short slices are not zero-filled, long ones are not truncated.
    assert_eq!(
        DeviationTable::from_deviations(&[-2.5, -0.5]).unwrap_err(),
        NavigationError::UnexpectedTableLength {
            found: 2,
            expected: 36
        }
    );
    assert!(DeviationTable::from_deviations(&[0.0; 37]).is_err());
    assert!(DeviationTable::from_deviations(&[0.0; 36]).is_ok());
}

#[test]
fn set_deviation_reports_unknown_nodes() {
    let mut table = DeviationTable::from_cardinal_directions();
    let westerly = Deviation::new(-1.0).unwrap();
    assert_eq!(
        table.set_deviation(50, westerly).unwrap_err(),
        NavigationError::CourseNotInTable { course: 50 }
    );
    table.set_deviation(90, westerly).unwrap();
    assert_eq!(table.deviation_at_node(90).unwrap().degrees(), -1.0);

    // ...but `insert_deviation` adds it, keeping the table sorted.
    table.insert_deviation(50, westerly).unwrap();
    assert_eq!(table.deviation_at_node(50).unwrap().degrees(), -1.0);
    assert!(table
        .nodes()
        .windows(2)
        .all(|pair| pair[0].course() < pair[1].course()));
}

#[test]
fn cardinal_points_round_trip() {
    let mut table = DeviationTable::from_cardinal_directions();
    table
        .set_deviation_at(CardinalPoint::N, Deviation::new(-2.5).unwrap())
        .unwrap();
    table
        .set_deviation_at(CardinalPoint::E, Deviation::new(1.0).unwrap())
        .unwrap();
    assert_eq!(
        table
            .deviation_at_point(CardinalPoint::N)
            .unwrap()
            .degrees(),
        -2.5
    );
    assert_eq!(
        table
            .deviation_at_point(CardinalPoint::E)
            .unwrap()
            .degrees(),
        1.0
    );
    assert_eq!(
        table
            .deviation_at_point(CardinalPoint::SW)
            .unwrap()
            .degrees(),
        0.0
    );
}

#[test]
fn a_point_absent_from_the_table_is_reported_as_such() {
    // A 100° step has nodes at 0°, 100°, 200°, 300°: no cardinal point other
    // than north.
    let table = DeviationTable::from_step(100).unwrap();
    assert!(table.deviation_at_point(CardinalPoint::N).is_ok());
    assert!(matches!(
        table.deviation_at_point(CardinalPoint::E),
        Err(NavigationError::CourseNotInTable { course: 90 })
    ));
}
