//! The same results with the standard library's maths and with `libm`
//! (R-08): values computed once with `std`, checked in both builds. The
//! actual difference is a nanometre in a northing, 1.6·10⁻¹⁶ of the meridian
//! arc it is the difference of, and nothing elsewhere.

#![allow(clippy::expect_used, clippy::unreadable_literal)]

use core::time::Duration;

use kinavis::region::PlanarPoint;
use kinavis_agri::{
    bicycle_step, ground_point, AbLine, CurveLine, FieldPlane, LeverArm, PassIndex, PassPattern,
    PathTracker, PivotLine, PurePursuit, ReferenceLine, Stanley, Tilt, VehicleGeometry,
    VehicleState,
};
use kinavis_kernel::{Angle, Distance, GridCourse, Height, Position, RateOfTurn, Speed};

fn values() -> Vec<f64> {
    let m = |v: f64| Distance::from_metres(v).expect("finite");
    let plane = FieldPlane::centred_at(
        Position::from_degrees(52.3, 4.9).expect("on Earth"),
        Height::above_ellipsoid(m(-2.0)),
    )
    .expect("a plane");
    let corner = plane
        .to_plane(Position::from_degrees(52.31, 4.915).expect("on Earth"))
        .expect("inside");
    let vehicle = VehicleGeometry::new(
        m(2.9),
        Angle::from_degrees(35.0).expect("finite"),
        RateOfTurn::from_radians_per_second(0.4).expect("finite"),
    )
    .expect("a vehicle");
    let state = VehicleState {
        position: PlanarPoint::from_metres(3.7, 12.0).expect("finite"),
        heading: GridCourse::new(17.0).expect("finite"),
        speed: Speed::from_metres_per_second(2.6).expect("finite"),
    };
    let pattern = PassPattern::new(m(9.0), m(0.25)).expect("a pattern");
    let line = AbLine::from_point_and_course(
        plane,
        PlanarPoint::ORIGIN,
        GridCourse::new(12.5).expect("finite"),
    );
    let pivot = PivotLine::new(
        plane,
        PlanarPoint::from_metres(-50.0, 0.0).expect("finite"),
        m(54.5),
    )
    .expect("a pivot");
    let curve = CurveLine::<8>::new(
        plane,
        &[
            PlanarPoint::from_metres(0.0, 0.0).expect("finite"),
            PlanarPoint::from_metres(5.0, 20.0).expect("finite"),
            PlanarPoint::from_metres(14.0, 41.0).expect("finite"),
        ],
    )
    .expect("a curve");
    let pursuit = PurePursuit::new(m(3.0), Duration::from_millis(600), m(15.0)).expect("pursuit");
    let stanley =
        Stanley::new(1.2, Speed::from_metres_per_second(0.5).expect("finite")).expect("Stanley");
    let after = bicycle_step(
        &state,
        &vehicle,
        Angle::from_degrees(-12.0).expect("finite"),
        Duration::from_millis(700),
    )
    .expect("a step");
    let ground = ground_point(
        state.position,
        state.heading,
        Tilt::new(
            Angle::from_degrees(6.0).expect("finite"),
            Angle::from_degrees(-2.0).expect("finite"),
        )
        .expect("a tilt"),
        LeverArm {
            forward: m(0.4),
            right: m(-0.2),
            up: m(3.1),
        },
    )
    .expect("finite");
    let parallel: CurveLine<64> = curve.offset(m(-7.5), m(0.01)).expect("an offset");
    vec![
        corner.east_metres(),
        corner.north_metres(),
        plane.convergence_at(corner).expect("inside").degrees(),
        plane.scale_at(corner).expect("inside"),
        line.station(state.position).offset.metres(),
        pivot.station(state.position).offset.metres(),
        curve.station(state.position).offset.metres(),
        pursuit
            .steer(&line, &pattern, PassIndex::REFERENCE, &vehicle, &state)
            .expect("a command")
            .angle
            .radians(),
        stanley
            .steer(&curve, &pattern, PassIndex::REFERENCE, &vehicle, &state)
            .expect("a command")
            .angle
            .radians(),
        pivot.station(state.position).direction.degrees(),
        after.position.east_metres(),
        after.position.north_metres(),
        after.heading.degrees(),
        ground.east_metres(),
        ground.north_metres(),
        parallel.length().metres(),
    ]
}

const EXPECTED: [f64; 16] = [
    1023.0388174029464,
    1112.8370310058817,
    0.011869953889921262,
    0.9999996995345687,
    -1.015019859086519,
    0.5244491112814345,
    -0.6790997501017323,
    -0.36274456869972543,
    -0.36226412727690155,
    167.4034267641815,
    4.114624248789645,
    13.770756238585232,
    9.356882454297192,
    3.4317851842633313,
    11.550704651884054,
    42.26093663703454,
];

#[test]
fn std_and_libm_agree() {
    let values = values();
    for (index, (ours, expected)) in values.iter().zip(EXPECTED).enumerate() {
        // Plane coordinates are differences of meridian arcs of thousands of
        // kilometres from the equator: relative to those.
        let scale = if index < 2 {
            1e7
        } else {
            expected.abs().max(1.0)
        };
        assert!(
            (ours - expected).abs() <= 1e-13 * scale,
            "value {index}: {ours:?} against {expected:?}"
        );
    }
}
