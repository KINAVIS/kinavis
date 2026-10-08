//! Local frames over the whole Earth: displacement round trip, consistency with
//! the ellipsoid, velocities of distant objects, body-frame conventions.

#![allow(clippy::expect_used)]

use core::time::Duration;
use kinavis_kernel::geodesy::{Ellipsoid, GeodeticPoint, Height};

use kinavis_kernel::local::{Body, LocalFrame, Ned, Vector3};
use kinavis_kernel::{Distance, Latitude, Longitude, Position, Speed};
use proptest::prelude::*;

fn any_origin() -> impl Strategy<Value = GeodeticPoint> {
    (-89.9_f64..=89.9, -180.0_f64..180.0, -500.0_f64..=10_000.0).prop_map(
        |(latitude, longitude, height)| {
            GeodeticPoint::new(
                Position::new(
                    Latitude::from_degrees(latitude).expect("in range"),
                    Longitude::from_degrees(longitude).expect("in range"),
                ),
                Height::above_ellipsoid(Distance::from_metres(height).expect("finite")),
            )
        },
    )
}

fn any_displacement() -> impl Strategy<Value = Vector3<Ned, Distance>> {
    // Up to a few hundred km each way (coastal chart extent).
    let component = -300_000.0_f64..=300_000.0;
    (
        component.clone(),
        component.clone(),
        -10_000.0_f64..=10_000.0,
    )
        .prop_map(|(n, e, d)| {
            Vector3::new(
                Distance::from_metres(n).expect("finite"),
                Distance::from_metres(e).expect("finite"),
                Distance::from_metres(d).expect("finite"),
            )
        })
}

proptest! {
    #[test]
    fn ned_to_point_and_back_is_the_identity(
        origin in any_origin(),
        displacement in any_displacement(),
    ) {
        let frame = LocalFrame::at(origin, &Ellipsoid::WGS84).expect("ellipsoidal origin");
        let there = frame.point_from_ned(displacement).expect("terrestrial");
        let back = frame.ned_of(there).expect("ellipsoidal height");
        let error = (back - displacement).magnitude().metres();
        prop_assert!(error < 1e-4, "off by {error} m from {origin}");
    }

    #[test]
    fn straight_up_from_the_origin_is_straight_up(
        origin in any_origin(),
        climb in 0.0_f64..=10_000.0,
    ) {
        let frame = LocalFrame::at(origin, &Ellipsoid::WGS84).expect("ellipsoidal origin");
        let raised = GeodeticPoint::new(
            origin.position(),
            Height::above_ellipsoid(
                Distance::from_metres(origin.height().value().metres() + climb).expect("finite"),
            ),
        );
        let ned = frame.ned_of(raised).expect("ellipsoidal height");
        prop_assert!(ned.horizontal_magnitude().metres() < 1e-6);
        prop_assert!((ned.down().metres() + climb).abs() < 1e-6);
    }

    #[test]
    fn body_to_flu_and_back_is_the_identity(
        forward in -1e6_f64..=1e6,
        right in -1e6_f64..=1e6,
        down in -1e6_f64..=1e6,
    ) {
        let metres = |value| Distance::from_metres(value).expect("finite");
        let body: Vector3<Body, Distance> = Vector3::new(metres(forward), metres(right), metres(down));
        let flu = body.to_flu();
        prop_assert_eq!(flu.forward(), body.forward());
        prop_assert_eq!(flu.left(), -body.right());
        prop_assert_eq!(flu.up(), -body.down());
        prop_assert_eq!(flu.to_body(), body);
    }

    #[test]
    fn a_distant_velocity_is_what_the_origin_sees_it_move(
        origin in any_origin(),
        displacement in any_displacement(),
        north in -30.0_f64..=30.0,
        east in -30.0_f64..=30.0,
        down in -5.0_f64..=5.0,
    ) {
        let speed = |value| Speed::from_metres_per_second(value).expect("finite");
        let frame = LocalFrame::at(origin, &Ellipsoid::WGS84).expect("ellipsoidal origin");
        let object = frame.point_from_ned(displacement).expect("terrestrial");
        let velocity: Vector3<Ned, Speed> = Vector3::new(speed(north), speed(east), speed(down));
        let state = frame.kinematics_of(object, velocity).expect("ellipsoidal height");

        // One second along the velocity in the object's own frame, measured
        // from the origin: the finite difference is the rotated velocity.
        let own = LocalFrame::at(object, &Ellipsoid::WGS84).expect("ellipsoidal");
        let step = velocity.displacement_in(Duration::from_secs(1));
        let later = frame.ned_of(own.point_from_ned(step).expect("terrestrial")).expect("ellipsoidal");
        let moved = later - state.position();
        for (seen, wanted) in moved.components().iter().zip(state.velocity().components()) {
            prop_assert!(
                (seen.metres() - wanted.metres_per_second()).abs() < 1e-3,
                "{moved:?} against {:?}", state.velocity()
            );
        }
        prop_assert!((state.velocity().magnitude().metres_per_second()
            - velocity.magnitude().metres_per_second()).abs() < 1e-9);
    }
}
