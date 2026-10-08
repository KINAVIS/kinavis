//! EGM96 over the whole Earth: no seam at the antimeridian, a single
//! height at each pole, the two interpolations within their stated errors of
//! each other.

#![cfg(feature = "egm96")]
#![allow(clippy::expect_used)]

use kinavis_geoid::{GridGeoid, Interpolation};
use kinavis_kernel::geodesy::GeoidModel;
use kinavis_kernel::Position;
use proptest::prelude::*;

fn height(geoid: &GridGeoid<'_>, latitude: f64, longitude: f64) -> f64 {
    geoid
        .undulation(Position::from_degrees(latitude, longitude).expect("on Earth"))
        .expect("a global grid")
        .metres()
}

fn both() -> [GridGeoid<'static>; 2] {
    [
        GridGeoid::EGM96,
        GridGeoid::EGM96.with_interpolation(Interpolation::Bilinear),
    ]
}

proptest! {
    #[test]
    fn the_antimeridian_is_no_seam(latitude in -90.0_f64..=90.0) {
        // Bilinear is continuous. The cubic is fitted per cell and steps at
        // every cell edge, the antimeridian included: by 1 mm typically, 12 cm
        // at most over 40 000 edges with GeographicLib's GeoidEval; each side
        // is within 0.169 m of the model, so the step is within twice that.
        for (geoid, step) in both().into_iter().zip([2.0 * 0.169, 1e-4]) {
            let west = height(&geoid, latitude, -180.0);
            let east = height(&geoid, latitude, 179.999_999_9);
            prop_assert!((west - east).abs() < step, "{west} against {east}");
        }
    }

    #[test]
    fn each_pole_has_one_height(longitude in -180.0_f64..180.0) {
        for geoid in both() {
            prop_assert!((height(&geoid, 90.0, longitude) - height(&geoid, 90.0, 0.0)).abs() < 1e-9);
            prop_assert!((height(&geoid, -90.0, longitude) - height(&geoid, -90.0, 0.0)).abs() < 1e-9);
        }
    }

    #[test]
    fn the_interpolations_agree_within_their_errors(
        latitude in -90.0_f64..=90.0,
        longitude in -180.0_f64..180.0,
    ) {
        let [cubic, bilinear] = both();
        let cubic = height(&cubic, latitude, longitude);
        let bilinear = height(&bilinear, latitude, longitude);
        // 1.152 m and 0.169 m: the largest errors of each against the model.
        prop_assert!((cubic - bilinear).abs() <= 1.152 + 0.169);
        prop_assert!((-108.0..=86.0).contains(&cubic));
    }
}
