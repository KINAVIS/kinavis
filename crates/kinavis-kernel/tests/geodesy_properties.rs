//! Geodetic ↔ ECEF round trip over the whole Earth and beyond.

#![allow(clippy::expect_used)]

use kinavis_kernel::geodesy::{Datum, EcefPoint, Ellipsoid, GeodeticPoint, Height, Helmert};
use kinavis_kernel::{Distance, Latitude, Longitude, Position};
use proptest::prelude::*;

fn any_point() -> impl Strategy<Value = GeodeticPoint> {
    // Latitudes concentrated towards the poles; heights from the deepest trench
    // to GNSS orbit.
    let latitude = prop_oneof![
        -90.0_f64..=90.0,
        (89.0_f64..=90.0),
        (-90.0_f64..=-89.0),
        Just(90.0),
        Just(-90.0),
    ];
    let longitude = prop_oneof![-180.0_f64..180.0, Just(-180.0), Just(179.999_999_9)];
    let height = -11_000.0_f64..=20_200_000.0;
    (latitude, longitude, height).prop_map(|(latitude, longitude, height)| {
        GeodeticPoint::new(
            Position::new(
                Latitude::from_degrees(latitude).expect("in range"),
                Longitude::from_degrees(longitude).expect("in range"),
            ),
            Height::above_ellipsoid(Distance::from_metres(height).expect("finite")),
        )
    })
}

proptest! {
    #[test]
    fn geodetic_to_ecef_and_back_is_the_identity(point in any_point()) {
        for ellipsoid in [Ellipsoid::WGS84, Ellipsoid::GRS80] {
            let ecef = EcefPoint::from_geodetic(point, &ellipsoid).expect("ellipsoidal height");
            let back = ecef.to_geodetic(&ellipsoid).expect("not the centre");

            let latitude = point.position().latitude().degrees();
            prop_assert!(
                (back.position().latitude().degrees() - latitude).abs() < 1e-9,
                "latitude {latitude} came back as {}",
                back.position().latitude().degrees()
            );
            // Longitude is arbitrary at a pole.
            if latitude.abs() < 90.0 - 1e-6 {
                let difference = back
                    .position()
                    .longitude_difference(point.position())
                    .degrees()
                    .abs();
                prop_assert!(difference < 1e-9, "longitude off by {difference}°");
            }
            let height = point.height().value().metres();
            prop_assert!(
                (back.height().value().metres() - height).abs() < 1e-3,
                "height {height} came back as {}",
                back.height().value().metres()
            );
        }
    }

    #[test]
    fn a_point_on_the_surface_is_a_semi_axis_from_the_centre(
        latitude in -90.0_f64..=90.0,
        longitude in -180.0_f64..180.0,
    ) {
        let point = GeodeticPoint::new(
            Position::new(
                Latitude::from_degrees(latitude).expect("in range"),
                Longitude::from_degrees(longitude).expect("in range"),
            ),
            Height::above_ellipsoid(Distance::ZERO),
        );
        let ecef = EcefPoint::from_geodetic(point, &Ellipsoid::WGS84).expect("ellipsoidal height");
        let centre = EcefPoint::new(Distance::ZERO, Distance::ZERO, Distance::ZERO);
        let radius = ecef.chord_to(centre).metres();
        let a = Ellipsoid::WGS84.semi_major_axis().metres();
        let b = Ellipsoid::WGS84.semi_minor_axis().metres();
        prop_assert!(radius <= a + 1e-6 && radius >= b - 1e-6, "radius {radius}");
    }

    #[test]
    fn a_datum_shift_and_its_reverse_close_anywhere_on_earth(point in any_point()) {
        // Height is dropped at each end and measured along a normal tilted by
        // the shift, so the round trip does not close exactly: ~1 cm in the
        // datum's home area, a decimetre or two on the far side of the Earth. 1
        // m required here.
        let there = point.position();
        for datum in [
            Datum::WGS84, Datum::NAD83, Datum::ED50, Datum::NAD27, Datum::OSGB36,
            Datum::PULKOVO_1942, Datum::TOKYO, Datum::DHDN, Datum::AGD66, Datum::SAD69,
        ] {
            let shifted = datum.to_wgs84(there).expect("a place on Earth");
            let back = datum.from_wgs84(shifted).expect("a place on Earth");
            // Chord distance, so the pole (where 1 mm is a degree of longitude)
            // is not a special case.
            let on_ellipsoid = |position| {
                let point = GeodeticPoint::new(position, Height::above_ellipsoid(Distance::ZERO));
                EcefPoint::from_geodetic(point, datum.ellipsoid()).expect("ellipsoidal height")
            };
            let apart = on_ellipsoid(back).chord_to(on_ellipsoid(there)).metres();
            prop_assert!(apart < 1.0, "{datum}: came back {apart} m away");
        }
    }

    #[test]
    fn a_helmert_transformation_and_its_inverse_are_the_identity(
        point in any_point(),
        translation in prop::array::uniform3(-1000.0_f64..=1000.0),
        rotation in prop::array::uniform3(-10.0_f64..=10.0),
        scale in -100.0_f64..=100.0,
    ) {
        let helmert = Helmert::position_vector(translation, rotation, scale).expect("finite");
        let ecef = EcefPoint::from_geodetic(point, &Ellipsoid::WGS84).expect("ellipsoidal height");
        let back = helmert.apply_inverse(helmert.apply(ecef));
        prop_assert!(back.chord_to(ecef).metres() < 1e-6, "off by {} m", back.chord_to(ecef).metres());
        let forth = helmert.apply(helmert.apply_inverse(ecef));
        prop_assert!(forth.chord_to(ecef).metres() < 1e-6);
    }
}
