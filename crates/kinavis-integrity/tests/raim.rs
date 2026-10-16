//! RAIM against an independent implementation and by simulation.
//!
//! - The chi-square threshold and the bias noncentrality against mpmath
//!   (`tests/data/statistics_reference.txt`).
//! - Test statistic and protection levels over real skies (the used
//!   satellites of u-blox `NAV-SAT` records) against numpy
//!   (`tests/data/raim_reference.txt`).
//! - Monte Carlo with a deterministic generator: the false-alarm rate is the
//!   one asked for; a gross fault on one satellite is detected and that
//!   satellite excluded.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::float_cmp,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::indexing_slicing
)]

use kinavis_integrity::stats::{bias_noncentrality, chi_square_quantile};
use kinavis_integrity::{
    assess, navigation_integrity, predict, protection_from_ellipse, AlertLimits, IntegrityReport,
    RaimConfig, RangeError, Unavailability,
};
use kinavis_kernel::{
    Angle, Constellation, Distance, NavigationIntegrity, SatelliteId, SatelliteSet, SatelliteView,
    TrueBearing,
};

const STATISTICS: &str = include_str!("data/statistics_reference.txt");
const SKIES: &str = include_str!("data/raim_reference.txt");

fn metres(value: f64) -> Distance {
    Distance::from_metres(value).unwrap()
}

fn relative(ours: f64, theirs: f64) -> f64 {
    (ours - theirs).abs() / theirs.abs().max(1e-12)
}

#[test]
fn thresholds_and_bias_noncentrality_agree_with_mpmath() {
    let mut checked = 0;
    for line in STATISTICS.lines().filter(|line| !line.starts_with('#')) {
        let fields: Vec<f64> = line
            .split(' ')
            .map(|field| field.parse().unwrap())
            .collect();
        let [dof, pfa, pmd, threshold_squared, lambda] = fields[..] else {
            panic!("{line}")
        };
        let dof = dof as u32;
        let ours = chi_square_quantile(1.0 - pfa, dof).unwrap();
        assert!(
            relative(ours, threshold_squared) < 1e-9,
            "{line}: threshold² {ours}"
        );
        let ours = bias_noncentrality(threshold_squared, dof, pmd).unwrap();
        assert!(
            relative(ours, lambda) < 1e-7 || (lambda == 0.0 && ours == 0.0),
            "{line}: λ {ours}"
        );
        checked += 1;
    }
    assert!(checked > 100);
}

fn constellation(gnss: u8) -> Constellation {
    match gnss {
        0 => Constellation::Gps,
        1 => Constellation::Sbas,
        2 => Constellation::Galileo,
        3 => Constellation::BeiDou,
        5 => Constellation::Qzss,
        6 => Constellation::Glonass,
        7 => Constellation::NavIc,
        other => panic!("gnss {other}"),
    }
}

/// One satellite per `gnss,sv,elev,azim,residual`.
fn sky(satellites: &str) -> SatelliteSet {
    let mut set = SatelliteSet::new();
    for satellite in satellites.split(' ') {
        let fields: Vec<&str> = satellite.split(',').collect();
        let [gnss, sv, elevation, azimuth, residual] = fields[..] else {
            panic!("{satellite}")
        };
        let id =
            SatelliteId::new(constellation(gnss.parse().unwrap()), sv.parse().unwrap()).unwrap();
        let view = SatelliteView::new(
            id,
            Some(Angle::from_degrees(elevation.parse().unwrap()).unwrap()),
            Some(TrueBearing::new(azimuth.parse().unwrap()).unwrap()),
        )
        .unwrap()
        .used_in_fix(true)
        .with_residual(metres(residual.parse().unwrap()));
        set.insert(view).unwrap();
    }
    set
}

fn config(pfa: f64, pmd: f64) -> RaimConfig {
    RaimConfig::new(pfa, pmd, RangeError::ElevationWeighted(metres(0.5))).unwrap()
}

#[test]
fn statistic_and_protection_levels_agree_with_numpy_over_real_skies() {
    let config = config(1e-3, 1e-3);
    let mut checked = 0;
    for line in SKIES.lines().filter(|line| !line.starts_with('#')) {
        let (satellites, expected) = line.split_once(" => ").unwrap();
        let (_, satellites) = satellites.split_once(' ').unwrap();
        let expected: Vec<f64> = expected.split(' ').map(|v| v.parse().unwrap()).collect();
        let [statistic, threshold, hpl, vpl] = expected[..] else {
            panic!("{line}")
        };
        let set = sky(satellites);
        let IntegrityReport::Available {
            protection,
            threshold: ours_threshold,
            ..
        } = predict(&set, &config).unwrap()
        else {
            panic!("geometry of {line}")
        };
        assert!(relative(ours_threshold, threshold) < 1e-9);
        assert!(
            relative(protection.horizontal.metres(), hpl) < 1e-6,
            "HPL {} against {hpl}",
            protection.horizontal.metres()
        );
        assert!(
            relative(protection.vertical.metres(), vpl) < 1e-6,
            "VPL {} against {vpl}",
            protection.vertical.metres()
        );
        let ours_statistic = match assess(&set, &config).unwrap() {
            IntegrityReport::Available { statistic, .. }
            | IntegrityReport::FaultDetected { statistic, .. } => statistic,
            other => panic!("{other:?}"),
        };
        assert!(
            relative(ours_statistic, statistic) < 1e-6,
            "statistic {ours_statistic} against {statistic}"
        );
        checked += 1;
    }
    assert!(checked >= 20, "{checked} skies");
}

/// The first sky of the reference, without its residuals.
fn first_sky() -> SatelliteSet {
    let line = SKIES.lines().find(|line| !line.starts_with('#')).unwrap();
    let (satellites, _) = line.split_once(" => ").unwrap();
    sky(satellites.split_once(' ').unwrap().1)
}

/// xorshift64* and Box–Muller: deterministic Gaussian draws.
struct Gaussian(u64);

impl Gaussian {
    fn uniform(&mut self) -> f64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        ((self.0.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 11) as f64 + 0.5) / (1_u64 << 53) as f64
    }

    fn next(&mut self) -> f64 {
        let (u, v) = (self.uniform(), self.uniform());
        (-2.0 * u.ln()).sqrt() * (core::f64::consts::TAU * v).cos()
    }
}

/// The sky with residuals drawn from the error model, plus `bias` metres on
/// satellite `faulty`.
fn draw(
    template: &SatelliteSet,
    random: &mut Gaussian,
    faulty: Option<usize>,
    bias: f64,
) -> SatelliteSet {
    let mut set = SatelliteSet::new();
    for (index, view) in template.iter().enumerate() {
        let elevation = view.elevation().unwrap().degrees().max(5.0);
        let sigma = 0.5 / elevation.to_radians().sin();
        let fault = if Some(index) == faulty { bias } else { 0.0 };
        set.insert(view.with_residual(metres(sigma * random.next() + fault)))
            .unwrap();
    }
    set
}

#[test]
fn the_false_alarm_rate_is_the_one_asked_for() {
    let template = first_sky();
    let config = config(0.02, 1e-3);
    let mut random = Gaussian(0x9E37_79B9_7F4A_7C15);
    let trials = 5_000;
    let alarms = (0..trials)
        .filter(|_| {
            matches!(
                assess(&draw(&template, &mut random, None, 0.0), &config).unwrap(),
                IntegrityReport::FaultDetected { .. }
            )
        })
        .count();
    // 100 expected; binomial σ ≈ 10.
    assert!((60..=140).contains(&alarms), "{alarms} alarms in {trials}");
}

#[test]
fn a_gross_fault_is_detected_and_its_satellite_excluded() {
    let template = first_sky();
    let config = config(1e-3, 1e-3);
    let mut random = Gaussian(42);
    let trials = 200;
    let mut excluded_right = 0;
    // A constellation of one satellite has its own clock, which absorbs any
    // error of that satellite: it neither moves the fix nor can be detected.
    let detectable: Vec<usize> = (0..template.len())
        .filter(|&i| {
            let constellation = template[i].id().constellation();
            template
                .iter()
                .filter(|view| view.id().constellation() == constellation)
                .count()
                > 1
        })
        .collect();
    for trial in 0..trials {
        let faulty = detectable[trial % detectable.len()];
        let set = draw(&template, &mut random, Some(faulty), 60.0);
        match assess(&set, &config).unwrap() {
            IntegrityReport::FaultDetected {
                excluded,
                after_exclusion,
                ..
            } => {
                if excluded == Some(template[faulty].id()) {
                    excluded_right += 1;
                    assert!(after_exclusion.is_some());
                }
            }
            other => panic!("a 60 m fault passed: {other:?}"),
        }
    }
    assert!(
        excluded_right * 100 >= trials * 95,
        "{excluded_right} of {trials} excluded right"
    );
}

#[test]
fn degenerate_skies_are_unavailable() {
    let id = |n| SatelliteId::new(Constellation::Gps, n).unwrap();
    let view = |n, elevation: f64, azimuth: f64| {
        SatelliteView::new(
            id(n),
            Some(Angle::from_degrees(elevation).unwrap()),
            Some(TrueBearing::new(azimuth).unwrap()),
        )
        .unwrap()
        .used_in_fix(true)
        .with_residual(Distance::ZERO)
    };
    let mut few: SatelliteSet = SatelliteSet::new();
    for n in 1..=4 {
        few.insert(view(n, 30.0 + f64::from(n), 90.0 * f64::from(n)))
            .unwrap();
    }
    assert_eq!(
        assess(&few, &config(1e-3, 1e-3)).unwrap(),
        IntegrityReport::Unavailable(Unavailability::TooFewSatellites {
            usable: 4,
            needed: 5
        })
    );
    let mut aligned: SatelliteSet = SatelliteSet::new();
    for n in 1..=8 {
        aligned.insert(view(n, 45.0, 120.0)).unwrap();
    }
    assert_eq!(
        assess(&aligned, &config(1e-3, 1e-3)).unwrap(),
        IntegrityReport::Unavailable(Unavailability::Geometry)
    );
}

#[test]
fn alert_limits_turn_a_report_into_the_kernels_integrity() {
    let report = predict(&first_sky(), &config(1e-3, 1e-3)).unwrap();
    let IntegrityReport::Available { protection, .. } = report else {
        panic!("{report:?}")
    };
    let wide = AlertLimits {
        horizontal: metres(protection.horizontal.metres() + 1.0),
        vertical: metres(protection.vertical.metres() + 1.0),
    };
    let narrow = AlertLimits {
        horizontal: metres(protection.horizontal.metres() - 0.1),
        vertical: wide.vertical,
    };
    assert_eq!(
        navigation_integrity(&report, &wide),
        NavigationIntegrity::Nominal
    );
    assert_eq!(
        navigation_integrity(&report, &narrow),
        NavigationIntegrity::Exceeded
    );
    // K = √(−2 ln 10⁻⁷) ≈ 5.68.
    let level = protection_from_ellipse(metres(1.0), 1e-7).unwrap();
    assert!((level.metres() - 5.677_692_427_555).abs() < 1e-9);
    assert!(RaimConfig::new(0.0, 1e-3, RangeError::Uniform(metres(1.0))).is_err());
}
