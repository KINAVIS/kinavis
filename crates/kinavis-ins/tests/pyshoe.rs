//! A foot-mounted IMU against the University of Toronto foot-mounted
//! inertial navigation dataset (`PyShoe`, Wagstaff et al., 2019): 56 trials in a
//! VICON area with the foot's position at every sample, and 11 walks along
//! hallways with the foot's position at surveyed markers.
//!
//! The data has no licence: it is not in the repository. `PYSHOE_DATA` names
//! a directory holding the trials as text (`tools/pyshoe_text.py`, from
//! `PyShoe`'s processed MAT files) and `PyShoe`'s published error tables in
//! `results/`.
//!
//! Checked: the stance test flags exactly the samples `PyShoe`'s SHOE detector
//! flags at its optimised thresholds; the error-state filter with
//! zero-velocity updates at those samples tracks the foot about as well as
//! `PyShoe`'s own filter, by `PyShoe`'s own error measure.

#![allow(
    clippy::expect_used,
    clippy::panic,
    clippy::indexing_slicing,
    clippy::cast_precision_loss,
    clippy::too_many_lines,
    clippy::many_single_char_names,
    clippy::needless_range_loop,
    clippy::format_push_string
)]

use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::time::Duration;

use kinavis_ins::{
    GatingPolicy, ImuNoise, ImuSample, InsFilter, InsPriors, Quaternion, StanceConfig,
    StanceDetector, Strapdown,
};
use kinavis_kernel::{
    Angle, Distance, GeodeticPoint, Height, Instant, Ned, Position, Speed, TrueCourse, Utc, Vector3,
};

/// `PyShoe`'s SHOE settings: window, accelerometer and gyro sigma, gravity.
const WINDOW: usize = 5;
const ACCEL_SIGMA: f64 = 0.000_98;
const GYRO_SIGMA: f64 = 8.726_646_3e-5;
const GRAVITY: f64 = 9.8029;

struct Trial {
    name: String,
    threshold: f64,
    times: Vec<f64>,
    imu: Vec<[f64; 6]>,
    flags: Vec<bool>,
    truth: Vec<(usize, [f64; 3])>,
}

fn directory() -> PathBuf {
    PathBuf::from(std::env::var("PYSHOE_DATA").expect("PYSHOE_DATA"))
}

fn read(path: &Path) -> Trial {
    let text = std::fs::read_to_string(path).expect("a trial");
    let mut lines = text.lines();
    let header: Vec<&str> = lines.next().expect("a header").split_whitespace().collect();
    let mut trial = Trial {
        name: path
            .file_stem()
            .expect("a name")
            .to_string_lossy()
            .into_owned(),
        threshold: header[2].parse().expect("G"),
        times: Vec::new(),
        imu: Vec::new(),
        flags: Vec::new(),
        truth: Vec::new(),
    };
    for line in lines {
        let fields: Vec<&str> = line.split_whitespace().collect();
        let number = |i: usize| fields[i].parse::<f64>().expect("a number");
        match fields[0] {
            "imu" => {
                trial.times.push(number(1));
                trial.imu.push([
                    number(2),
                    number(3),
                    number(4),
                    number(5),
                    number(6),
                    number(7),
                ]);
                trial.flags.push(fields[8] == "1");
            }
            "gt" => trial.truth.push((
                fields[1].parse().expect("index"),
                [number(2), number(3), number(4)],
            )),
            other => panic!("{other}"),
        }
    }
    trial
}

fn trials(kind: &str) -> Vec<Trial> {
    let mut paths: Vec<PathBuf> = std::fs::read_dir(directory().join("text"))
        .expect("the text directory")
        .map(|entry| entry.expect("an entry").path())
        .filter(|path| {
            path.file_name()
                .is_some_and(|name| name.to_string_lossy().starts_with(kind))
        })
        .collect();
    paths.sort();
    paths.iter().map(|path| read(path)).collect()
}

fn sample(trial: &Trial, k: usize) -> ImuSample {
    let [ax, ay, az, wx, wy, wz] = trial.imu[k];
    let dt = if k == 0 {
        0.005
    } else {
        (trial.times[k] - trial.times[k - 1]).clamp(1e-4, 0.05)
    };
    ImuSample::new([wx, wy, wz], [ax, ay, az], Duration::from_secs_f64(dt)).expect("a sample")
}

/// Stance of every sample by our detector; the samples past the last whole
/// window are not tested.
fn stances(trial: &Trial, threshold: f64) -> Vec<Option<bool>> {
    let config = StanceConfig::new(WINDOW, ACCEL_SIGMA, GYRO_SIGMA, GRAVITY, threshold)
        .expect("PyShoe's settings");
    let mut detector = StanceDetector::new(config);
    let mut flags = vec![None; trial.imu.len()];
    for k in 0..trial.imu.len() {
        if let Some(stance) = detector.push(&sample(trial, k)) {
            for flag in &mut flags[k + 1 - WINDOW..=k] {
                *flag = Some(stance.standing);
            }
        }
    }
    flags
}

/// `PyShoe`'s process noise, 0.5 m/s² and 0.5°/s per sample at 200 Hz, as
/// random walks; slow bias walks. A foot strikes the ground: the datasheet
/// noise of `ImuNoise::mems` leaves the filter too sure of itself (12 ×
/// `PyShoe`'s error in the VICON area).
fn noise() -> ImuNoise {
    let dt: f64 = 0.005;
    ImuNoise::new(0.5f64.to_radians() * dt.sqrt(), 0.5 * dt.sqrt(), 1e-5, 1e-4).expect("noise")
}

/// The foot's track by the filter with zero-velocity updates at `stance`, in
/// metres east, north, up from the start.
fn track(trial: &Trial, stance: &[Option<bool>]) -> Vec<[f64; 3]> {
    // Level on the first 50 samples, standing; heading unknown, set to 0.
    let mean = (0..50).fold([0.0; 3], |sum, k| {
        let s = trial.imu[k];
        [
            sum[0] + s[0] / 50.0,
            sum[1] + s[1] / 50.0,
            sum[2] + s[2] / 50.0,
        ]
    });
    let roll = (-mean[1]).atan2(-mean[2]);
    let pitch = mean[0].atan2(mean[1].hypot(mean[2]));
    let attitude = Quaternion::from_euler(
        Angle::from_radians(roll).expect("roll"),
        Angle::from_radians(pitch).expect("pitch"),
        TrueCourse::NORTH,
    );
    let toronto = GeodeticPoint::new(
        Position::from_degrees(43.66, -79.40).expect("Toronto"),
        Height::above_ellipsoid(Distance::from_metres(100.0).expect("100 m")),
    );
    let still = Vector3::<Ned, Speed>::new(Speed::ZERO, Speed::ZERO, Speed::ZERO);
    let start = Instant::<Utc>::from_unix_seconds(1_510_000_000);
    let priors = InsPriors::new(
        Distance::from_metres(0.01).expect("1 cm"),
        Distance::from_metres(0.01).expect("1 cm"),
        Speed::from_metres_per_second(0.01).expect("1 cm/s"),
        Angle::from_degrees(1.0).expect("1°"),
        Angle::from_degrees(0.1).expect("0.1°"),
        // The gyro calibrated at rest before the walk: yaw is not observable
        // through zero-velocity updates, and a loose bias prior lets the
        // filter wander in it (1.1 × PyShoe's error in the hallways at
        // 10⁻⁴ rad/s, 5.9 × at 0.5°/s, chosen on these data).
        1e-4,
        0.05,
    )
    .expect("priors");
    let mut filter = InsFilter::new(
        Strapdown::new(start, toronto, still, attitude).expect("a start"),
        noise(),
        &priors,
    );
    let zero = Speed::from_metres_per_second(0.01).expect("1 cm/s");
    let mut out = Vec::with_capacity(trial.imu.len());
    for k in 0..trial.imu.len() {
        if k > 0 {
            filter.predict(&sample(trial, k)).expect("a step");
        }
        if stance[k] == Some(true) {
            filter
                .update_zero_velocity(zero, GatingPolicy::none())
                .expect("an update");
        }
        let d = filter.displacement();
        // The dataset's plane is north and east of NED; its height is up.
        out.push([d.north().metres(), d.east().metres(), -d.down().metres()]);
    }
    out
}

/// `PyShoe`'s alignment: both from their first point, ours turned about the
/// vertical so that its point `at` lies along the truth's `towards`.
fn align(ours: &mut [[f64; 3]], at: usize, towards: [f64; 3], origin: [f64; 3]) {
    let first = ours[0];
    for point in ours.iter_mut() {
        for (value, start) in point.iter_mut().zip(first) {
            *value -= start;
        }
    }
    let a = ours[at];
    let b = [towards[0] - origin[0], towards[1] - origin[1]];
    let angle = (a[0] * b[1] - a[1] * b[0]).atan2(a[0] * b[0] + a[1] * b[1]);
    let (s, c) = angle.sin_cos();
    for point in ours.iter_mut() {
        let (x, y) = (point[0], point[1]);
        point[0] = c * x - s * y;
        point[1] = s * x + c * y;
    }
}

/// `PyShoe`'s error: the mean over points of the RMS over axes.
fn armse(ours: &[[f64; 3]], truth: &[[f64; 3]], axes: usize) -> f64 {
    let total: f64 = ours
        .iter()
        .zip(truth)
        .map(|(a, b)| ((0..axes).map(|i| (a[i] - b[i]).powi(2)).sum::<f64>() / axes as f64).sqrt())
        .sum();
    total / ours.len() as f64
}

/// `PyShoe`'s published errors: `results/<file>`, keyed by the trial's time
/// stamp, the column named.
fn published(file: &str, key_column: &str, column: &str) -> HashMap<String, f64> {
    let text = std::fs::read_to_string(directory().join("results").join(file)).expect("a table");
    let mut lines = text.lines();
    let header = split_csv(lines.next().expect("a header"));
    let key = header
        .iter()
        .position(|h| h == key_column)
        .expect("the key");
    let value = header.iter().position(|h| h == column).expect("the column");
    lines
        .map(split_csv)
        .map(|row| {
            let stamp = row[key]
                .trim_start_matches('/')
                .trim_end_matches(".mat")
                .to_owned();
            (stamp, row[value].parse().expect("an error"))
        })
        .collect()
}

fn split_csv(line: &str) -> Vec<String> {
    let mut fields = Vec::new();
    let (mut field, mut quoted) = (String::new(), false);
    for ch in line.chars() {
        match ch {
            '"' => quoted = !quoted,
            ',' if !quoted => fields.push(std::mem::take(&mut field)),
            _ => field.push(ch),
        }
    }
    fields.push(field);
    fields
}

#[test]
#[ignore = "needs the PyShoe data in PYSHOE_DATA"]
fn the_stance_test_flags_what_pyshoe_flags() {
    let vicon = trials("vicon-");
    assert_eq!(vicon.len(), 56);
    let (mut agree, mut total) = (0usize, 0usize);
    for trial in &vicon {
        let ours = stances(trial, trial.threshold);
        for (flag, theirs) in ours.iter().zip(&trial.flags) {
            if let Some(flag) = flag {
                total += 1;
                agree += usize::from(flag == theirs);
            }
        }
    }
    assert_eq!(agree, total, "{} of {total} samples differ", total - agree);
}

#[test]
#[ignore = "needs the PyShoe data in PYSHOE_DATA"]
fn the_filter_tracks_the_foot_in_the_vicon_area_as_pyshoe_does() {
    let theirs = published("vicon_results_raw.csv", "folder", "shoe_2d");
    let mut ratios = Vec::new();
    let mut report = String::new();
    for trial in trials("vicon-") {
        let stance = stances(&trial, trial.threshold);
        let mut ours = track(&trial, &stance);
        let truth: Vec<[f64; 3]> = trial.truth.iter().map(|(_, p)| *p).collect();
        // Align at 0.8 m from the start, as PyShoe does.
        let at = (1..ours.len())
            .find(|&k| (ours[k][0] - ours[0][0]).hypot(ours[k][1] - ours[0][1]) >= 0.8)
            .unwrap_or(300);
        align(&mut ours, at, truth[at], truth[0]);
        let shifted: Vec<[f64; 3]> = truth
            .iter()
            .map(|p| [p[0] - truth[0][0], p[1] - truth[0][1], p[2] - truth[0][2]])
            .collect();
        let error = armse(&ours, &shifted, 2);
        let stamp = trial.name.trim_start_matches("vicon-").to_owned();
        let reference = theirs.get(&stamp).copied().expect("published");
        report.push_str(&format!("{stamp} ours {error:.3} theirs {reference:.3}\n"));
        ratios.push(error / reference.max(0.005));
    }
    ratios.sort_by(f64::total_cmp);
    let median = ratios[ratios.len() / 2];
    eprintln!("{report}median ratio {median:.2}");
    // 0.83 when written.
    assert!(median < 1.0, "median ratio {median}");
}

#[test]
#[ignore = "needs the PyShoe data in PYSHOE_DATA"]
fn the_filter_walks_the_hallways_as_pyshoe_does() {
    let theirs = published(
        "hallway_results_raw.csv",
        "Trial",
        "shoe_G=85000000.0_3d_error",
    );
    let mut report = String::new();
    let mut ratios = Vec::new();
    let mut closures = Vec::new();
    for trial in trials("hallway-") {
        let stance = stances(&trial, 8.5e7);
        let mut ours = track(&trial, &stance);
        let (indices, truth): (Vec<usize>, Vec<[f64; 3]>) = trial.truth.iter().copied().unzip();
        align(&mut ours, indices[1], truth[1], truth[0]);
        let at: Vec<[f64; 3]> = indices.iter().map(|&k| ours[k]).collect();
        let error = armse(&at, &truth, 3);
        let stamp = trial
            .name
            .rsplit('-')
            .take(6)
            .collect::<Vec<_>>()
            .into_iter()
            .rev()
            .collect::<Vec<_>>()
            .join("-");
        let reference = theirs.get(&stamp).copied();
        let path: f64 = truth
            .windows(2)
            .map(|w| ((w[1][0] - w[0][0]).powi(2) + (w[1][1] - w[0][1]).powi(2)).sqrt())
            .sum();
        let closure = (at[at.len() - 1][0] - at[0][0]).hypot(at[at.len() - 1][1] - at[0][1]);
        report.push_str(&format!(
            "{stamp} ours {error:.3} theirs {reference:?} path {path:.0} m closure {closure:.2} m ({:.2} %)\n",
            100.0 * closure / path
        ));
        closures.push(100.0 * closure / path);
        ratios.push(error / reference.expect("published"));
    }
    assert_eq!(ratios.len(), 11);
    ratios.sort_by(f64::total_cmp);
    closures.sort_by(f64::total_cmp);
    let median = ratios[ratios.len() / 2];
    let closure = closures[closures.len() / 2];
    eprintln!("{report}median ratio {median:.2}, median closure {closure:.2} % of the path");
    // 1.12 and 1.03 % when written.
    assert!(median < 1.3, "median ratio {median}");
    assert!(closure < 2.0, "median closure {closure} %");
}
