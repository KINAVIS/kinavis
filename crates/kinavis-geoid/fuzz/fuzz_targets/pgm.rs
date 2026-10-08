//! Fuzz target: a geoid grid read from arbitrary bytes and evaluated.
//!
//! Invariants: no panic; a grid that reads evaluates everywhere, with both
//! interpolations, to a finite height within its sample range.
#![no_main]

use libfuzzer_sys::fuzz_target;

use kinavis_geoid::{GridGeoid, Interpolation};
use kinavis_kernel::geodesy::GeoidModel;
use kinavis_kernel::Position;

fuzz_target!(|data: &[u8]| {
    let Ok(grid) = GridGeoid::from_pgm(data) else {
        return;
    };
    for interpolation in [Interpolation::Bilinear, Interpolation::Cubic] {
        let grid = grid.with_interpolation(interpolation);
        for (latitude, longitude) in [(90.0, 0.0), (-90.0, 179.0), (0.0, -180.0), (33.3, 77.7)] {
            let position = Position::from_degrees(latitude, longitude).expect("in range");
            let height = grid.undulation(position).expect("a grid that reads evaluates");
            assert!(height.metres().is_finite());
        }
    }
});
