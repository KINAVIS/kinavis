//! A pass across a side slope: the antenna leans with the tractor, the fix is
//! the antenna's, and tilt compensation brings the cross-track error back to
//! the error of the fix and the roll sensor.

#![allow(clippy::expect_used, clippy::panic, clippy::cast_precision_loss)]

use kinavis::region::PlanarPoint;
use kinavis_agri::{
    ground_point, locate, AbLine, FieldPlane, LeverArm, PassIndex, PassPattern, Tilt,
};
use kinavis_kernel::{Angle, Distance, GridCourse, Height, Position};

fn metres(value: f64) -> Distance {
    Distance::from_metres(value).expect("finite")
}

/// splitmix64, as in the reference scripts.
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> f64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        ((z ^ (z >> 31)) >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Standard normal by Box–Muller.
    fn normal(&mut self) -> f64 {
        let u = self.next().max(f64::MIN_POSITIVE);
        let v = self.next();
        (-2.0 * u.ln()).sqrt() * (std::f64::consts::TAU * v).cos()
    }
}

/// Antenna position of a tractor whose control point is `ground`, as the
/// body-to-level rotation (roll, then pitch, then heading) places it.
fn antenna(
    ground: PlanarPoint,
    heading: f64,
    roll: f64,
    pitch: f64,
    lever: [f64; 3],
) -> PlanarPoint {
    let [forward, right, up] = lever;
    let (sr, cr) = roll.to_radians().sin_cos();
    let (sp, cp) = pitch.to_radians().sin_cos();
    let (x, y, z) = (forward, right, -up);
    let (y, z) = (y * cr - z * sr, y * sr + z * cr);
    let x = x * cp + z * sp;
    let (sh, ch) = heading.to_radians().sin_cos();
    PlanarPoint::from_metres(
        ground.east_metres() + x * sh + y * ch,
        ground.north_metres() + x * ch - y * sh,
    )
    .expect("finite")
}

#[test]
fn tilt_compensation_takes_out_the_lean_of_the_mast() {
    let plane = FieldPlane::centred_at(
        Position::from_degrees(46.2, 20.1).expect("on Earth"),
        Height::above_ellipsoid(metres(85.0)),
    )
    .expect("a plane");
    // Passes run along the contour, grid bearing 070°; the slope falls to the
    // right of the travel, so the tractor rolls right side down.
    let line = AbLine::from_point_and_course(
        plane,
        PlanarPoint::ORIGIN,
        GridCourse::new(70.0).expect("a course"),
    );
    let pattern = PassPattern::new(metres(18.0), metres(0.2)).expect("a pattern");
    let pass = PassIndex::new(-5);
    let offset = pattern.offset_of(pass).metres();
    let lever = [0.4, -0.1, 3.2];
    let arm = LeverArm {
        forward: metres(lever[0]),
        right: metres(lever[1]),
        up: metres(lever[2]),
    };

    let (sigma_fix, sigma_angle, sigma_heading) = (0.01, 0.1, 0.2);
    let mut rng = Rng(20_261_015);
    let (mut raw_sum, mut worst, mut squares) = (0.0, 0.0f64, 0.0);
    let steps = 3000;
    for step in 0..steps {
        let s = f64::from(step) * 0.2;
        // On the pass exactly, with the slope and the wheel tracks rocking it.
        let (ux, uy) = (70f64.to_radians().sin(), 70f64.to_radians().cos());
        let ground =
            PlanarPoint::from_metres(s * ux - offset * uy, s * uy + offset * ux).expect("finite");
        let roll = 9.0 + 3.0 * (s / 7.0).sin();
        let pitch = 2.0 * (s / 11.0).sin();
        let heading = 70.0 + 1.5 * (s / 5.0).sin();
        let true_antenna = antenna(ground, heading, roll, pitch, lever);
        let fix = PlanarPoint::from_metres(
            true_antenna.east_metres() + sigma_fix * rng.normal(),
            true_antenna.north_metres() + sigma_fix * rng.normal(),
        )
        .expect("finite");
        let measured_heading =
            GridCourse::wrap(heading + sigma_heading * rng.normal()).expect("finite");
        let tilt = Tilt::new(
            Angle::from_degrees(roll + sigma_angle * rng.normal()).expect("finite"),
            Angle::from_degrees(pitch + sigma_angle * rng.normal()).expect("finite"),
        )
        .expect("a tilt");

        let raw = locate(&line, &pattern, fix, measured_heading);
        assert_eq!(raw.pass, pass);
        raw_sum += raw.cross_track.metres();

        let control = ground_point(fix, measured_heading, tilt, arm).expect("finite");
        let compensated = locate(&line, &pattern, control, measured_heading);
        assert_eq!(compensated.pass, pass);
        let error = compensated.cross_track.metres();
        worst = worst.max(error.abs());
        squares += error * error;
    }
    let raw_mean = raw_sum / f64::from(steps);
    let rms = (squares / f64::from(steps)).sqrt();
    // Without compensation the fix sits h·sin φ − 0.1 m to the right: 40 cm.
    assert!(
        raw_mean > 0.38 && raw_mean < 0.46,
        "uncompensated mean {raw_mean}"
    );
    // Compensated: the fix's error and h·σ_roll, about 1.2 cm, nothing more.
    let expected = (sigma_fix.powi(2) + (lever[2] * sigma_angle.to_radians()).powi(2)).sqrt();
    assert!(rms < 1.15 * expected, "rms {rms} against {expected}");
    assert!(worst < 4.5 * expected, "worst {worst}");
}
