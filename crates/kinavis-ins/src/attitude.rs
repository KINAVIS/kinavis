//! Attitude: rotation from the body frame to the local navigation frame, as a
//! unit quaternion.
//!
//! Body frame: forward, right, down. Navigation frame: north, east, down. `v^n
//! = C v^b`. Euler angles use the aerospace convention: yaw about down, pitch
//! about the new right, roll about the new forward; yaw is the heading,
//! positive roll is starboard down.

use core::fmt;

use kinavis_kernel::angle::TrueCourse;
use kinavis_kernel::error::{ensure_finite, Result};
use kinavis_kernel::math;
use kinavis_kernel::matrix::Matrix;
use kinavis_kernel::units::Angle;

/// Unit quaternion `[w, x, y, z]`, body to navigation frame.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(try_from = "StoredQuaternion", into = "StoredQuaternion")
)]
pub struct Quaternion {
    w: f64,
    x: f64,
    y: f64,
    z: f64,
}

impl Quaternion {
    /// Identity: body frame aligned with north, east, down.
    pub const IDENTITY: Self = Self {
        w: 1.0,
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    /// Quaternion from four components, normalised.
    ///
    /// # Errors
    ///
    /// [`kinavis_kernel::KernelError::NotFinite`] for a non-finite component or
    /// all zeros.
    pub fn new(w: f64, x: f64, y: f64, z: f64) -> Result<Self> {
        for value in [w, x, y, z] {
            ensure_finite("quaternion component", value)?;
        }
        let norm = math::sqrt(w * w + x * x + y * y + z * z);
        ensure_finite("quaternion norm", 1.0 / norm)?;
        Ok(Self {
            w: w / norm,
            x: x / norm,
            y: y / norm,
            z: z / norm,
        })
    }

    /// Rotation from roll, pitch and yaw.
    #[must_use]
    pub fn from_euler(roll: Angle, pitch: Angle, yaw: TrueCourse) -> Self {
        let (sr, cr) = sin_cos(roll.radians() / 2.0);
        let (sp, cp) = sin_cos(pitch.radians() / 2.0);
        let (sy, cy) = sin_cos(yaw.radians() / 2.0);
        Self {
            w: cr * cp * cy + sr * sp * sy,
            x: sr * cp * cy - cr * sp * sy,
            y: cr * sp * cy + sr * cp * sy,
            z: cr * cp * sy - sr * sp * cy,
        }
        .normalised()
    }

    /// Roll, pitch and yaw.
    ///
    /// At pitch ±90° roll and yaw are not separable; roll is set to zero and
    /// yaw takes the remainder.
    #[must_use]
    pub fn to_euler(&self) -> Attitude {
        let (w, x, y, z) = (self.w, self.x, self.y, self.z);
        let sin_pitch = (2.0 * (w * y - z * x)).clamp(-1.0, 1.0);
        let pitch = math::asin(sin_pitch);
        let (roll, yaw) = if 1.0 - math::abs(sin_pitch) < 1e-12 {
            (
                0.0,
                math::atan2(2.0 * (x * y + w * z), 1.0 - 2.0 * (y * y + z * z)),
            )
        } else {
            (
                math::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)),
                math::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)),
            )
        };
        Attitude {
            roll: Angle::from_radians(roll).unwrap_or(Angle::ZERO),
            pitch: Angle::from_radians(pitch).unwrap_or(Angle::ZERO),
            yaw: TrueCourse::from_degrees_wrapped(math::to_degrees(yaw)),
        }
    }

    /// Components `[w, x, y, z]`.
    #[must_use]
    pub const fn components(&self) -> [f64; 4] {
        [self.w, self.x, self.y, self.z]
    }

    /// Rotation matrix `C`, body to navigation.
    #[must_use]
    pub fn to_matrix(&self) -> Matrix<3, 3> {
        let (w, x, y, z) = (self.w, self.x, self.y, self.z);
        Matrix::from_rows([
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - w * z),
                2.0 * (x * z + w * y),
            ],
            [
                2.0 * (x * y + w * z),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - w * x),
            ],
            [
                2.0 * (x * z - w * y),
                2.0 * (y * z + w * x),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ])
    }

    /// Body-frame vector expressed in the navigation frame.
    #[must_use]
    pub fn rotate(&self, body: [f64; 3]) -> [f64; 3] {
        apply(&self.to_matrix(), body)
    }

    /// Navigation-frame vector expressed in the body frame.
    #[must_use]
    pub fn rotate_back(&self, navigation: [f64; 3]) -> [f64; 3] {
        apply(&self.to_matrix().transpose(), navigation)
    }

    /// Quaternion product `self ⊗ other`: `other` applied first, then `self`.
    #[must_use]
    pub fn then(&self, other: &Self) -> Self {
        // `(self ⊗ other).rotate(v) == self.rotate(other.rotate(v))`.
        let (aw, ax, ay, az) = (self.w, self.x, self.y, self.z);
        let (bw, bx, by, bz) = (other.w, other.x, other.y, other.z);
        Self {
            w: aw * bw - ax * bx - ay * by - az * bz,
            x: aw * bx + ax * bw + ay * bz - az * by,
            y: aw * by - ax * bz + ay * bw + az * bx,
            z: aw * bz + ax * by - ay * bx + az * bw,
        }
    }

    /// Attitude propagated by a body rotation vector: `|rotation|` radians
    /// about `rotation / |rotation|`.
    ///
    /// Exact for a constant rate over the step; no small-angle approximation,
    /// so large steps stay unit quaternions.
    #[must_use]
    pub fn rotated_by_body(&self, rotation: [f64; 3]) -> Self {
        self.then(&Self::from_rotation_vector(rotation))
            .normalised()
    }

    /// Removes a small navigation-frame misalignment: `C ← (I − [ψ×]) C`, the
    /// error-state feedback.
    #[must_use]
    pub fn corrected_by(&self, misalignment: [f64; 3]) -> Self {
        let [a, b, c] = misalignment;
        Self::from_rotation_vector([-a, -b, -c])
            .then(self)
            .normalised()
    }

    /// Quaternion of a rotation vector: angle = length, axis = direction.
    #[must_use]
    pub fn from_rotation_vector(rotation: [f64; 3]) -> Self {
        let [a, b, c] = rotation;
        let angle = math::sqrt(a * a + b * b + c * c);
        if angle < 1e-12 {
            // sin(θ/2)/θ → 1/2.
            return Self {
                w: 1.0,
                x: a / 2.0,
                y: b / 2.0,
                z: c / 2.0,
            }
            .normalised();
        }
        let (sin, cos) = sin_cos(angle / 2.0);
        let scale = sin / angle;
        Self {
            w: cos,
            x: a * scale,
            y: b * scale,
            z: c * scale,
        }
    }

    /// Renormalised to remove accumulated rounding.
    #[must_use]
    pub fn normalised(&self) -> Self {
        let norm =
            math::sqrt(self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z);
        if norm < f64::MIN_POSITIVE || !norm.is_finite() {
            return Self::IDENTITY;
        }
        Self {
            w: self.w / norm,
            x: self.x / norm,
            y: self.y / norm,
            z: self.z / norm,
        }
    }

    /// Misalignment of this (estimated) attitude relative to `truth`: the
    /// navigation-frame rotation `ψ` with `C_true = (I − [ψ×]) C_estimate`,
    /// i.e. the error an error-state filter estimates. Exact for any error
    /// size, as a rotation vector.
    #[must_use]
    pub fn misalignment_from(&self, truth: &Self) -> [f64; 3] {
        // self = R ⊗ truth ⇒ R = self ⊗ truth⁻¹, and I − [ψ×] ≈ Rᵀ: ψ is the
        // rotation vector of R.
        let inverse = Self {
            w: truth.w,
            x: -truth.x,
            y: -truth.y,
            z: -truth.z,
        };
        let relative = self.then(&inverse).normalised();
        // Shortest rotation.
        let sign = if relative.w < 0.0 { -1.0 } else { 1.0 };
        let (w, x, y, z) = (
            relative.w * sign,
            relative.x * sign,
            relative.y * sign,
            relative.z * sign,
        );
        let sin_half = math::sqrt(x * x + y * y + z * z);
        if sin_half < 1e-12 {
            return [2.0 * x, 2.0 * y, 2.0 * z];
        }
        let angle = 2.0 * math::atan2(sin_half, w);
        let scale = angle / sin_half;
        [x * scale, y * scale, z * scale]
    }

    /// Angle between two attitudes, in radians.
    #[must_use]
    pub fn angle_to(&self, other: &Self) -> f64 {
        let dot = self.w * other.w + self.x * other.x + self.y * other.y + self.z * other.z;
        2.0 * math::acos(math::abs(dot).clamp(0.0, 1.0))
    }
}

/// Roll, pitch and yaw.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Attitude {
    /// About the forward axis, positive starboard down.
    pub roll: Angle,
    /// About the right axis, positive bow up.
    pub pitch: Angle,
    /// Heading from true north.
    pub yaw: TrueCourse,
}

impl Attitude {
    /// Level, heading north.
    pub const LEVEL_NORTH: Self = Self {
        roll: Angle::ZERO,
        pitch: Angle::ZERO,
        yaw: TrueCourse::NORTH,
    };
}

impl fmt::Display for Attitude {
    /// Formats as `roll 2.0° pitch -1.0° yaw 271.5°`.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let precision = f.precision().unwrap_or(1);
        write!(
            f,
            "roll {:.*}° pitch {:.*}° yaw {:.*}°",
            precision,
            self.roll.degrees(),
            precision,
            self.pitch.degrees(),
            precision,
            self.yaw.degrees()
        )
    }
}

/// `M v`.
pub(crate) fn apply(matrix: &Matrix<3, 3>, vector: [f64; 3]) -> [f64; 3] {
    let rows = matrix.rows();
    let mut out = [0.0; 3];
    for (slot, row) in out.iter_mut().zip(rows) {
        *slot = row[0] * vector[0] + row[1] * vector[1] + row[2] * vector[2];
    }
    out
}

/// Cross-product matrix `[v×]`: `[v×] u = v × u`.
pub(crate) fn skew(v: [f64; 3]) -> Matrix<3, 3> {
    Matrix::from_rows([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])
}

/// `a × b`.
pub(crate) fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn sin_cos(radians: f64) -> (f64, f64) {
    (math::sin(radians), math::cos(radians))
}

/// Serialised form: four raw components. Deserialisation goes through
/// [`Quaternion::new`], so zero or `NaN` rotations are rejected.
#[cfg(feature = "serde")]
#[derive(serde::Serialize, serde::Deserialize)]
struct StoredQuaternion {
    w: f64,
    x: f64,
    y: f64,
    z: f64,
}

#[cfg(feature = "serde")]
impl TryFrom<StoredQuaternion> for Quaternion {
    type Error = kinavis_kernel::KernelError;

    fn try_from(stored: StoredQuaternion) -> Result<Self> {
        Self::new(stored.w, stored.x, stored.y, stored.z)
    }
}

#[cfg(feature = "serde")]
impl From<Quaternion> for StoredQuaternion {
    fn from(quaternion: Quaternion) -> Self {
        Self {
            w: quaternion.w,
            x: quaternion.x,
            y: quaternion.y,
            z: quaternion.z,
        }
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::float_cmp, clippy::indexing_slicing)]

    use super::*;

    fn degrees(value: f64) -> Angle {
        Angle::from_degrees(value).unwrap()
    }

    #[test]
    fn euler_angles_round_trip_and_yaw_is_the_heading() {
        let q = Quaternion::from_euler(
            degrees(10.0),
            degrees(-5.0),
            TrueCourse::new(271.5).unwrap(),
        );
        let attitude = q.to_euler();
        assert!((attitude.roll.degrees() - 10.0).abs() < 1e-9);
        assert!((attitude.pitch.degrees() + 5.0).abs() < 1e-9);
        assert!((attitude.yaw.degrees() - 271.5).abs() < 1e-9);
        assert_eq!(
            std::format!("{attitude}"),
            "roll 10.0° pitch -5.0° yaw 271.5°"
        );
        assert_eq!(Quaternion::IDENTITY.to_euler(), Attitude::LEVEL_NORTH);
    }

    #[test]
    fn a_heading_of_east_carries_forward_to_east() {
        let q = Quaternion::from_euler(Angle::ZERO, Angle::ZERO, TrueCourse::new(90.0).unwrap());
        let forward = q.rotate([1.0, 0.0, 0.0]);
        assert!(forward[0].abs() < 1e-12);
        assert!((forward[1] - 1.0).abs() < 1e-12);
        let back = q.rotate_back(forward);
        assert!((back[0] - 1.0).abs() < 1e-12);
        // Down is invariant under yaw.
        assert!((q.rotate([0.0, 0.0, 1.0])[2] - 1.0).abs() < 1e-12);
    }

    #[test]
    fn a_body_rotation_about_down_adds_to_the_yaw() {
        let q = Quaternion::from_euler(Angle::ZERO, Angle::ZERO, TrueCourse::new(350.0).unwrap());
        let turned = q.rotated_by_body([0.0, 0.0, math::to_radians(20.0)]);
        assert!((turned.to_euler().yaw.degrees() - 10.0).abs() < 1e-9);
        // Zero rotation, no division by zero.
        assert!(q.rotated_by_body([0.0; 3]).angle_to(&q) < 1e-12);
        // Tiny rotation stays tiny.
        let tiny = q.rotated_by_body([1e-13, 0.0, 0.0]);
        assert!(tiny.angle_to(&q) < 1e-12);
    }

    #[test]
    fn a_navigation_frame_correction_takes_the_misalignment_out() {
        // Estimate with yaw ε too large: misalignment about down is ε; the
        // correction reduces yaw by ε.
        let estimate =
            Quaternion::from_euler(Angle::ZERO, Angle::ZERO, TrueCourse::new(45.0).unwrap());
        let epsilon = math::to_radians(0.5);
        let corrected = estimate.corrected_by([0.0, 0.0, epsilon]);
        assert!((corrected.to_euler().yaw.degrees() - 44.5).abs() < 1e-9);
        // Matrix form: (I − [ψ×]) C, first order.
        let expected =
            (Matrix::<3, 3>::identity() - skew([0.0, 0.0, epsilon])) * estimate.to_matrix();
        let got = corrected.to_matrix();
        for row in 0..3 {
            for column in 0..3 {
                assert!(
                    (expected.get(row, column).unwrap() - got.get(row, column).unwrap()).abs()
                        < 1e-4
                );
            }
        }
    }

    #[test]
    fn the_misalignment_is_what_the_correction_takes_out() {
        let truth =
            Quaternion::from_euler(degrees(3.0), degrees(-2.0), TrueCourse::new(200.0).unwrap());
        let misalignment = [0.01, -0.02, 0.03];
        // Estimate with that misalignment: truth = (I − [ψ×]) estimate ⇒
        // estimate = q(ψ) ⊗ truth.
        let estimate = Quaternion::from_rotation_vector(misalignment).then(&truth);
        let found = estimate.misalignment_from(&truth);
        for axis in 0..3 {
            assert!(
                (found[axis] - misalignment[axis]).abs() < 1e-12,
                "{found:?}"
            );
        }
        let corrected = estimate.corrected_by(found);
        assert!(corrected.angle_to(&truth) < 1e-12);
        assert_eq!(truth.misalignment_from(&truth), [0.0; 3]);
    }

    #[test]
    fn products_compose_rotations_and_the_matrix_agrees() {
        let a = Quaternion::from_euler(degrees(20.0), degrees(0.0), TrueCourse::new(0.0).unwrap());
        let b = Quaternion::from_euler(degrees(0.0), degrees(30.0), TrueCourse::new(0.0).unwrap());
        let v = [0.3, -0.4, 0.5];
        let composed = a.then(&b).rotate(v);
        let stepwise = a.rotate(b.rotate(v));
        let by_matrix = apply(&(a.to_matrix() * b.to_matrix()), v);
        for axis in 0..3 {
            assert!((composed[axis] - stepwise[axis]).abs() < 1e-12);
            assert!((composed[axis] - by_matrix[axis]).abs() < 1e-12);
        }
        assert!((a.angle_to(&Quaternion::IDENTITY) - math::to_radians(20.0)).abs() < 1e-9);
    }

    #[test]
    fn a_quaternion_is_normalised_and_finite_or_refused() {
        let q = Quaternion::new(2.0, 0.0, 0.0, 0.0).unwrap();
        assert_eq!(q.components(), [1.0, 0.0, 0.0, 0.0]);
        assert!(Quaternion::new(0.0, 0.0, 0.0, 0.0).is_err());
        assert!(Quaternion::new(f64::NAN, 0.0, 0.0, 0.0).is_err());
        assert_eq!(
            Quaternion {
                w: 0.0,
                x: 0.0,
                y: 0.0,
                z: 0.0
            }
            .normalised(),
            Quaternion::IDENTITY
        );
    }

    #[test]
    fn the_gimbal_lock_case_keeps_the_yaw() {
        let q = Quaternion::from_euler(degrees(0.0), degrees(90.0), TrueCourse::new(30.0).unwrap());
        let attitude = q.to_euler();
        assert!((attitude.pitch.degrees() - 90.0).abs() < 1e-6);
        assert_eq!(attitude.roll, Angle::ZERO);
    }

    #[test]
    fn cross_and_skew_agree() {
        let a = [1.0, 2.0, 3.0];
        let b = [-2.0, 0.5, 4.0];
        assert_eq!(cross(a, b), apply(&skew(a), b));
        assert_eq!(cross(a, b), [6.5, -10.0, 4.5]);
    }
}
