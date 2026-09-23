//! Tests for [`super`], kept separate to bound the size of `interpolation.rs`.

#![allow(clippy::unwrap_used, clippy::float_cmp, clippy::indexing_slicing)]

use super::super::table::DeviationTable;
use super::super::test_support::{heading, readme_table};
use super::*;
use crate::angle::{CompassCourse, Deviation};
use crate::error::{KernelError, NavigationError};
use crate::math;
use alloc::vec;

/// Batch read into a new buffer, so tests compare whole sweeps directly.
fn batch(table: &DeviationTable, courses: &[f64], method: InterpolationMethod) -> Vec<f64> {
    let courses: Vec<CompassCourse> = courses.iter().copied().map(heading).collect();
    let mut out = vec![Deviation::ZERO; courses.len()];
    table
        .interpolate_deviation(&courses, method, &mut out)
        .unwrap();
    out.iter().map(|deviation| deviation.degrees()).collect()
}

#[test]
fn a_bad_angle_never_reaches_the_table() {
    // The range check lives in the course type, not in `deviation_at`, which
    // is why these calls do not compile in any other form.
    assert!(CompassCourse::new(400.0).is_err());
    assert!(CompassCourse::new(f64::NAN).is_err());
    assert!(CompassCourse::new(-1.0).is_err());
    assert_eq!(CompassCourse::new(360.0).unwrap().degrees(), 0.0);
}

#[test]
fn a_short_output_buffer_is_reported_not_truncated() {
    let table = DeviationTable::default();
    let courses = [heading(0.0), heading(90.0), heading(180.0)];
    let mut out = [Deviation::ZERO; 2];
    assert!(matches!(
        table.interpolate_deviation(&courses, InterpolationMethod::Linear, &mut out),
        Err(NavigationError::Kernel(KernelError::BufferTooSmall {
            needed: 3,
            found: 2
        }))
    ));

    // A longer buffer is accepted; the tail is untouched.
    let mut out = [Deviation::ZERO; 4];
    assert!(table
        .interpolate_deviation(&courses, InterpolationMethod::Linear, &mut out)
        .is_ok());
}

#[test]
fn linear_interpolation_is_exact_at_nodes() {
    let table = readme_table();
    for node in table.nodes() {
        let value = table
            .deviation_at(
                heading(f64::from(node.course())),
                InterpolationMethod::Linear,
            )
            .unwrap();
        assert!((value.degrees() - node.deviation_degrees()).abs() < 1e-12);
    }
}

#[test]
fn cubic_interpolation_is_exact_at_nodes() {
    let table = readme_table();
    for node in table.nodes() {
        let value = table
            .deviation_at(
                heading(f64::from(node.course())),
                InterpolationMethod::Cubic,
            )
            .unwrap();
        assert!(
            (value.degrees() - node.deviation_degrees()).abs() < 1e-9,
            "node {}: {} vs {}",
            node.course(),
            value.degrees(),
            node.deviation_degrees()
        );
    }
}

#[test]
fn cubic_interpolates_across_the_first_segment() {
    // A flat extrapolation would give -2.5 across the whole 0°..10° segment.
    let mut table = DeviationTable::from_step(10).unwrap();
    table.set_deviation(0, Deviation::new(-2.5).unwrap()).unwrap();
    table.set_deviation(10, Deviation::new(-1.5).unwrap()).unwrap();

    let midpoint = table
        .deviation_at(heading(5.0), InterpolationMethod::Cubic)
        .unwrap()
        .degrees();
    assert!(
        midpoint > -2.5 && midpoint < -1.5,
        "midpoint should lie between the nodes, got {midpoint}"
    );
}

#[test]
fn linear_interpolation_wraps_through_north() {
    // Values past the last node wrap to the first, not clamp.
    let mut table = DeviationTable::from_step(10).unwrap();
    table.set_deviation(350, Deviation::new(10.0).unwrap()).unwrap();
    table.set_deviation(0, Deviation::new(-10.0).unwrap()).unwrap();

    let midpoint = table
        .deviation_at(heading(355.0), InterpolationMethod::Linear)
        .unwrap();
    assert!((midpoint.degrees() - 0.0).abs() < 1e-12);

    let quarter = table
        .deviation_at(heading(352.5), InterpolationMethod::Linear)
        .unwrap();
    assert!((quarter.degrees() - 5.0).abs() < 1e-12);
}

#[test]
fn cubic_spline_is_smooth_across_north() {
    let table = readme_table();
    let before = table
        .deviation_at(heading(359.9), InterpolationMethod::Cubic)
        .unwrap()
        .degrees();
    let after = table
        .deviation_at(heading(0.1), InterpolationMethod::Cubic)
        .unwrap()
        .degrees();
    assert!(
        (before - after).abs() < 0.05,
        "spline jumps across north: {before} vs {after}"
    );
}

#[test]
fn cubic_spline_reproduces_a_sinusoid() {
    let values: Vec<f64> = (0..36)
        .map(|index| 5.0 * math::sin(math::to_radians(f64::from(index) * 10.0)))
        .collect();
    let table = DeviationTable::from_deviations(&values).unwrap();

    for course in [5.0, 17.5, 123.4, 250.0, 355.0] {
        let expected = 5.0 * math::sin(math::to_radians(course));
        let actual = table
            .deviation_at(heading(course), InterpolationMethod::Cubic)
            .unwrap()
            .degrees();
        assert!(
            (actual - expected).abs() < 1e-3,
            "at {course}: {actual} vs {expected}"
        );
    }
}

#[test]
fn linear_never_overshoots_its_nodes() {
    let table = readme_table();
    let low = table
        .nodes()
        .iter()
        .fold(f64::MAX, |acc, node| acc.min(node.deviation_degrees()));
    let high = table
        .nodes()
        .iter()
        .fold(f64::MIN, |acc, node| acc.max(node.deviation_degrees()));

    let mut course = 0.0;
    while course < 360.0 {
        let value = table
            .deviation_at(heading(course), InterpolationMethod::Linear)
            .unwrap()
            .degrees();
        assert!(value >= low - 1e-12 && value <= high + 1e-12);
        course += 0.25;
    }
}

#[test]
fn two_node_table_falls_back_from_cubic_to_linear() {
    let table = DeviationTable::from_pairs(&[(0, 0.0), (180, 4.0)]).unwrap();
    let value = table
        .deviation_at(heading(90.0), InterpolationMethod::Cubic)
        .unwrap();
    assert!((value.degrees() - 2.0).abs() < 1e-12);
}

#[test]
fn uneven_node_spacing_still_interpolates() {
    let table = DeviationTable::from_pairs(&[
        (0, 1.0),
        (7, -2.0),
        (93, 0.5),
        (200, -3.0),
        (201, -3.1),
        (355, 2.0),
    ])
    .unwrap();

    for method in [
        InterpolationMethod::Linear,
        InterpolationMethod::Cubic,
        InterpolationMethod::Parametric,
        InterpolationMethod::ShapePreserving,
    ] {
        let mut course = 0.0;
        while course < 360.0 {
            let value = table.deviation_at(heading(course), method).unwrap();
            assert!(value.degrees().is_finite(), "{method:?} at {course}");
            course += 0.5;
        }
    }
}

#[test]
fn shape_preserving_is_exact_at_nodes() {
    let table = readme_table();
    for node in table.nodes() {
        let value = table
            .deviation_at(
                heading(f64::from(node.course())),
                InterpolationMethod::ShapePreserving,
            )
            .unwrap();
        assert!((value.degrees() - node.deviation_degrees()).abs() < 1e-12);
    }
}

#[test]
fn shape_preserving_never_overshoots_where_the_spline_does() {
    // The swing has a 12.5° step, where a natural cubic spline overshoots the
    // data.
    let table = readme_table();
    let low = table
        .nodes()
        .iter()
        .fold(f64::MAX, |acc, node| acc.min(node.deviation_degrees()));
    let high = table
        .nodes()
        .iter()
        .fold(f64::MIN, |acc, node| acc.max(node.deviation_degrees()));

    let mut spline_overshot = false;
    let mut course = 0.0;
    while course < 360.0 {
        let shaped = table
            .deviation_at(heading(course), InterpolationMethod::ShapePreserving)
            .unwrap()
            .degrees();
        assert!(
            shaped >= low - 1e-12 && shaped <= high + 1e-12,
            "shape-preserving bulged to {shaped} at {course}"
        );

        let spline = table
            .deviation_at(heading(course), InterpolationMethod::Cubic)
            .unwrap()
            .degrees();
        if spline < low - 1e-9 || spline > high + 1e-9 {
            spline_overshot = true;
        }
        course += 0.25;
    }

    assert!(
        spline_overshot,
        "the cubic spline was expected to overshoot on this swing"
    );
}

#[test]
fn shape_preserving_stays_between_neighbouring_nodes() {
    // Stronger property: within each segment the curve stays between that
    // segment's two values.
    let table = readme_table();
    let nodes = table.nodes();
    for pair in nodes.windows(2) {
        let (start, end) = (pair[0], pair[1]);
        let (low, high) = if start.deviation_degrees() <= end.deviation_degrees() {
            (start.deviation_degrees(), end.deviation_degrees())
        } else {
            (end.deviation_degrees(), start.deviation_degrees())
        };

        let mut course = f64::from(start.course());
        while course <= f64::from(end.course()) {
            let value = table
                .deviation_at(heading(course), InterpolationMethod::ShapePreserving)
                .unwrap()
                .degrees();
            assert!(
                value >= low - 1e-12 && value <= high + 1e-12,
                "between {}° and {}° the curve reached {value}, outside [{low}, {high}]",
                start.course(),
                end.course()
            );
            course += 0.1;
        }
    }
}

#[test]
fn shape_preserving_is_smooth_across_north() {
    let table = readme_table();
    let before = table
        .deviation_at(heading(359.9), InterpolationMethod::ShapePreserving)
        .unwrap()
        .degrees();
    let after = table
        .deviation_at(heading(0.1), InterpolationMethod::ShapePreserving)
        .unwrap()
        .degrees();
    assert!((before - after).abs() < 0.05, "{before} vs {after}");
}

#[test]
fn shape_preserving_reproduces_a_gentle_curve() {
    let values: Vec<f64> = (0..36)
        .map(|index| 5.0 * math::sin(math::to_radians(f64::from(index) * 10.0)))
        .collect();
    let table = DeviationTable::from_deviations(&values).unwrap();

    for course in [5.0, 17.5, 123.4, 250.0, 355.0] {
        let expected = 5.0 * math::sin(math::to_radians(course));
        let actual = table
            .deviation_at(heading(course), InterpolationMethod::ShapePreserving)
            .unwrap()
            .degrees();
        assert!(
            (actual - expected).abs() < 0.02,
            "at {course}: {actual} vs {expected}"
        );
    }
}

#[test]
fn interpolate_batch_matches_single_lookups() {
    let table = readme_table();
    let courses = [0.0, 3.0, 45.5, 180.0, 259.9, 360.0];
    for method in [
        InterpolationMethod::Linear,
        InterpolationMethod::Cubic,
        InterpolationMethod::Parametric,
        InterpolationMethod::ShapePreserving,
    ] {
        let whole = batch(&table, &courses, method);
        for (index, &course) in courses.iter().enumerate() {
            let single = table
                .deviation_at(heading(course), method)
                .unwrap()
                .degrees();
            assert!((whole[index] - single).abs() < 1e-12);
        }
    }
}
