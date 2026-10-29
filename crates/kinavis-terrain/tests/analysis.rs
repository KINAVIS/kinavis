//! Slope and aspect on analytic surfaces, profiles against the geodesic,
//! line of sight against the horizon and against dense sampling, walking
//! time.

#![allow(
    clippy::expect_used,
    clippy::panic,
    clippy::float_cmp,
    clippy::indexing_slicing,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::identity_op
)]

use kinavis::sailings::geodesic;
use kinavis_kernel::geodesy::{EcefPoint, Ellipsoid, GeodeticPoint};
use kinavis_kernel::{Distance, ElevationModel, Height, KernelError, Position, Result, Speed};
use kinavis_terrain::{
    line_of_sight, profile, slope_aspect, summarise, ProfilePoint, WalkingPace, STANDARD_REFRACTION,
};

fn position(lat: f64, lon: f64) -> Position {
    Position::from_degrees(lat, lon).expect("on Earth")
}

fn metres(value: f64) -> Distance {
    Distance::from_metres(value).expect("finite")
}

/// A surface given in metres east and north of an origin, on a local plane.
struct Surface<F> {
    origin: Position,
    height: F,
}

impl<F: Fn(f64, f64) -> f64> ElevationModel for Surface<F> {
    fn elevation(&self, at: Position) -> Result<Height> {
        // Mean radius: the local plane of the test, not the crate's radii.
        let r = 6_371_000.0;
        let phi = self.origin.latitude().radians();
        let east = (at.longitude().degrees() - self.origin.longitude().degrees()).to_radians()
            * r
            * phi.cos();
        let north = (at.latitude().degrees() - self.origin.latitude().degrees()).to_radians() * r;
        Ok(Height::above_mean_sea_level(Distance::from_metres((self
            .height)(
            east, north,
        ))?))
    }
}

/// The same surface on the crate's own radii, so a plane is a plane.
struct Ellipsoidal<F> {
    origin: Position,
    height: F,
}

impl<F: Fn(f64, f64) -> f64> ElevationModel for Ellipsoidal<F> {
    fn elevation(&self, at: Position) -> Result<Height> {
        let a = 6_378_137.0;
        let e2 = Ellipsoid::WGS84.first_eccentricity_squared();
        let phi = self.origin.latitude().radians();
        let w = (1.0 - e2 * phi.sin().powi(2)).sqrt();
        let (meridian, normal) = (a * (1.0 - e2) / w.powi(3), a / w);
        let east = (at.longitude().degrees() - self.origin.longitude().degrees()).to_radians()
            * normal
            * phi.cos();
        let north =
            (at.latitude().degrees() - self.origin.latitude().degrees()).to_radians() * meridian;
        Ok(Height::above_mean_sea_level(Distance::from_metres((self
            .height)(
            east, north,
        ))?))
    }
}

#[test]
fn a_plane_has_its_slope_and_aspect_at_any_spacing() {
    let origin = position(46.5, 7.5);
    // Rising 0.3 m per metre east and 0.4 north: slope atan 0.5, falling
    // toward the south-west, 216.87°.
    let plane = Ellipsoidal {
        origin,
        height: |e: f64, n: f64| 1000.0 + 0.3 * e + 0.4 * n,
    };
    for spacing in [1.0, 30.0, 500.0] {
        let at = slope_aspect(&plane, origin, metres(spacing)).expect("a slope");
        assert!(
            (at.east_gradient - 0.3).abs() < 1e-6,
            "{spacing}: {}",
            at.east_gradient
        );
        assert!((at.north_gradient - 0.4).abs() < 1e-6);
        assert!((at.slope.degrees() - 0.5f64.atan().to_degrees()).abs() < 1e-4);
        let aspect = at.aspect.expect("sloping").degrees();
        assert!(
            (aspect - (180.0 + 0.75f64.atan().to_degrees())).abs() < 1e-4,
            "{aspect}"
        );
    }
    let level = Ellipsoidal {
        origin,
        height: |_: f64, _: f64| 250.0,
    };
    let flat = slope_aspect(&level, origin, metres(30.0)).expect("level");
    assert_eq!(flat.aspect, None);
    assert_eq!(flat.slope.degrees(), 0.0);
    assert!(slope_aspect(&level, origin, metres(0.01)).is_err());
}

#[test]
fn a_hill_falls_away_from_its_top_in_every_direction() {
    let origin = position(43.35, 42.44);
    // A Gaussian hill 500 m high, 1 km wide, in metres on the crate's own
    // radii: on a mean radius the east would be 0.27 % long.
    let hill = Ellipsoidal {
        origin,
        height: |e: f64, n: f64| 500.0 * (-(e * e + n * n) / (2.0 * 1000.0f64.powi(2))).exp(),
    };
    for bearing in [0.0f64, 45.0, 135.0, 200.0, 290.0] {
        let (e, n) = (
            800.0 * bearing.to_radians().sin(),
            800.0 * bearing.to_radians().cos(),
        );
        let e2 = Ellipsoid::WGS84.first_eccentricity_squared();
        let phi = 43.35f64.to_radians();
        let w = (1.0 - e2 * phi.sin().powi(2)).sqrt();
        let (meridian, normal) = (6_378_137.0 * (1.0 - e2) / w.powi(3), 6_378_137.0 / w);
        let at = Position::from_degrees(
            43.35 + (n / meridian).to_degrees(),
            42.44 + (e / (normal * phi.cos())).to_degrees(),
        )
        .expect("near");
        let found = slope_aspect(&hill, at, metres(10.0)).expect("a slope");
        // Analytic gradient magnitude at 800 m: 500 · r / σ² · e^(−r²/2σ²).
        let rise = 500.0 * 800.0 / 1e6 * (-0.32f64).exp();
        assert!(
            (found.slope.radians().tan() - rise).abs() < 2e-3 * rise,
            "{bearing}"
        );
        let aspect = found.aspect.expect("sloping").degrees();
        let off = (aspect - bearing + 540.0).rem_euclid(360.0) - 180.0;
        assert!(off.abs() < 0.5, "{bearing}: {aspect}");
    }
}

#[test]
fn a_profile_is_as_long_as_the_geodesic_and_climbs_what_it_crosses() {
    let start = position(55.70, 37.40);
    for end in [
        position(55.71, 37.42),
        position(55.95, 37.90),
        position(55.30, 37.10),
    ] {
        let flat = Surface {
            origin: start,
            height: |_: f64, _: f64| 150.0,
        };
        let mut points = [ProfilePoint {
            distance: Distance::ZERO,
            position: start,
            elevation: Height::above_mean_sea_level(Distance::ZERO),
        }; 401];
        let summary = profile(&flat, start, end, &mut points).expect("a profile");
        let truth = geodesic(start, end).expect("apart").distance.metres();
        assert!(
            (summary.length.metres() - truth).abs() < 1e-6 * truth,
            "{} against {truth}",
            summary.length.metres()
        );
        assert_eq!(summary.ascent, Distance::ZERO);
        assert_eq!(points[0].position, start);
        assert!(
            (points[400].position.latitude().degrees() - end.latitude().degrees()).abs() < 1e-9
        );
    }
    // Over the hill: up 500 m and down again, give or take the sampling.
    let origin = position(43.35, 42.44);
    let hill = Surface {
        origin,
        height: |e: f64, n: f64| 500.0 * (-(e * e + n * n) / (2.0 * 1000.0f64.powi(2))).exp(),
    };
    let mut points = vec![
        ProfilePoint {
            distance: Distance::ZERO,
            position: origin,
            elevation: Height::above_mean_sea_level(Distance::ZERO),
        };
        1001
    ];
    let summary = profile(
        &hill,
        position(43.35, 42.39),
        position(43.35, 42.49),
        &mut points,
    )
    .expect("a profile");
    assert!((summary.ascent.metres() - summary.descent.metres()).abs() < 1.0);
    assert!(summary.ascent.metres() > 495.0 && summary.ascent.metres() < 500.0);
    assert!((summary.highest.value().metres() - 500.0).abs() < 0.5);
    assert!(profile(&hill, origin, origin, &mut points[..1]).is_err());
}

#[test]
fn a_threshold_ignores_the_noise_of_a_model() {
    let make = |heights: &[f64]| -> Vec<ProfilePoint> {
        heights
            .iter()
            .enumerate()
            .map(|(i, &h)| ProfilePoint {
                distance: metres(i as f64 * 30.0),
                position: position(0.0, 0.0),
                elevation: Height::above_mean_sea_level(metres(h)),
            })
            .collect()
    };
    // A climb of 100 m with ±2 m of noise on the way.
    let noisy = make(&[
        0.0, 2.0, 0.0, 2.0, 0.0, 50.0, 48.0, 50.0, 100.0, 98.0, 100.0,
    ]);
    let raw = summarise(&noisy, Distance::ZERO).expect("raw");
    assert_eq!(raw.ascent.metres(), 2.0 + 2.0 + 50.0 + 2.0 + 50.0 + 2.0);
    let clean = summarise(&noisy, metres(5.0)).expect("thresholded");
    assert_eq!(clean.ascent.metres(), 100.0);
    assert_eq!(clean.descent.metres(), 0.0);
    assert_eq!(clean.length.metres(), 300.0);
    assert!(summarise(&[], Distance::ZERO).is_err());
}

#[test]
fn two_masts_see_each_other_to_the_horizon_and_no_further() {
    // Level sea-level ground; two 50 m masts. With refraction k the horizon
    // of each is √(2·R·h / (1 − k)).
    let flat = Surface {
        origin: position(0.0, 0.0),
        height: |_: f64, _: f64| 0.0,
    };
    let h = 50.0;
    for k in [0.0, STANDARD_REFRACTION] {
        let reach = 2.0 * (2.0 * 6_371_008.8 * h / (1.0 - k)).sqrt();
        for (factor, seen) in [(0.98, true), (1.02, false)] {
            let distance = reach * factor;
            let east = (distance / 6_378_137.0).to_degrees();
            let sight = line_of_sight(
                &flat,
                position(0.0, 10.0),
                metres(h),
                position(0.0, 10.0 + east),
                metres(h),
                2000,
                k,
            )
            .expect("a sight");
            assert_eq!(sight.visible, seen, "k {k}, {factor} of the horizon");
            // The line comes lowest half way.
            assert!((sight.at.metres() / distance - 0.5).abs() < 0.01);
        }
    }
}

#[test]
fn the_sight_line_agrees_with_dense_sampling_of_the_same_line() {
    let origin = position(43.35, 42.44);
    let ridge = Surface {
        origin,
        height: |e: f64, n: f64| {
            800.0 + 300.0 * (-((e - 400.0).powi(2)) / (2.0 * 300.0f64.powi(2))).exp() + 0.02 * n
        },
    };
    let (from, to) = (position(43.35, 42.40), position(43.36, 42.50));
    for (eye, tall) in [(2.0, 2.0), (2.0, 450.0), (1.7, 900.0)] {
        let sight = line_of_sight(&ridge, from, metres(eye), to, metres(tall), 65_536, 0.0)
            .expect("a sight");
        // Brute force: 200 000 points of the same chord, geometric.
        let ground = |p: Position| ridge.elevation(p).expect("defined").value().metres();
        let ecef = |p: Position, h: f64| {
            EcefPoint::from_geodetic(
                GeodeticPoint::new(p, Height::above_ellipsoid(metres(h))),
                &Ellipsoid::WGS84,
            )
            .expect("on Earth")
        };
        let (a, b) = (ecef(from, ground(from) + eye), ecef(to, ground(to) + tall));
        let mut least = f64::MAX;
        for i in 1..200_000 {
            let t = f64::from(i) / 200_000.0;
            let mix = |p: Distance, q: Distance| metres(p.metres() + (q.metres() - p.metres()) * t);
            let point = EcefPoint::new(mix(a.x(), b.x()), mix(a.y(), b.y()), mix(a.z(), b.z()))
                .to_geodetic(&Ellipsoid::WGS84)
                .expect("above ground");
            least = least.min(point.height().value().metres() - ground(point.position()));
        }
        assert_eq!(
            sight.visible,
            least >= 0.0,
            "eye {eye}, target {tall}: {least}"
        );
        assert!(
            (sight.clearance.metres() - least).abs() < 0.05,
            "{} against {least}",
            sight.clearance.metres()
        );
    }
}

#[test]
fn a_sight_outside_the_model_or_with_bad_parameters_is_refused() {
    struct Nowhere;
    impl ElevationModel for Nowhere {
        fn elevation(&self, _: Position) -> Result<Height> {
            Err(KernelError::OutsideValidity { data: "terrain" })
        }
    }
    let (a, b) = (position(0.0, 0.0), position(0.0, 0.1));
    assert!(line_of_sight(&Nowhere, a, metres(2.0), b, metres(2.0), 10, 0.0).is_err());
    let flat = Surface {
        origin: a,
        height: |_: f64, _: f64| 0.0,
    };
    assert!(line_of_sight(&flat, a, metres(2.0), b, metres(2.0), 0, 0.0).is_err());
    assert!(line_of_sight(&flat, a, metres(2.0), b, metres(2.0), 70_000, 0.0).is_err());
    assert!(line_of_sight(&flat, a, metres(-1.0), b, metres(2.0), 10, 0.0).is_err());
    assert!(line_of_sight(&flat, a, metres(2.0), b, metres(2.0), 10, 1.5).is_err());
}

#[test]
fn naismith_and_paces_of_ones_own() {
    let rule = WalkingPace::NAISMITH;
    let time = rule.time(metres(10_000.0), metres(600.0));
    assert!((time.as_secs_f64() - 3.0 * 3600.0).abs() < 1e-6);
    assert!((rule.time(metres(5_000.0), Distance::ZERO).as_secs_f64() - 3600.0).abs() < 1e-6);
    let slow = WalkingPace::new(
        Speed::from_kilometres_per_hour(4.0).expect("4 km/h"),
        metres(400.0),
    )
    .expect("a pace");
    assert!((slow.time(metres(4_000.0), metres(200.0)).as_secs_f64() - 1.5 * 3600.0).abs() < 1e-6);
    assert!(WalkingPace::new(
        Speed::from_kilometres_per_hour(40.0).expect("40"),
        metres(400.0)
    )
    .is_err());
    assert!(WalkingPace::new(
        Speed::from_kilometres_per_hour(4.0).expect("4"),
        metres(10.0)
    )
    .is_err());
}
