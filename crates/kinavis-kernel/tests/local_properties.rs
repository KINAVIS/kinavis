//! Local frames over the whole Earth: displacement round trip and consistency
//! with the ellipsoid.

#![allow(clippy::expect_used)]

use kinavis_kernel::geodesy::{Ellipsoid, GeodeticPoint, Height};
use kinavis_kernel::local::{LocalFrame, Ned, Vector3};
use kinavis_kernel::{Distance, Latitude, Longitude, Position};
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
}
