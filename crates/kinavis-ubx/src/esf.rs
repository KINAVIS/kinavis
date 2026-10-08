//! External sensor fusion, class `ESF` (0x10): the inertial and odometry
//! measurements of dead-reckoning receivers (ZED-F9R, NEO-M8L, NEO-M8U).

use kinavis_kernel::local::{Body, Vector3};
use kinavis_kernel::{Acceleration, Speed};

use crate::error::UbxError;
use crate::read::{i32_at, u16_at, u32_at};

/// Class `ESF`.
pub const CLASS: u8 = 0x10;
/// `ESF-MEAS`.
pub const MEAS: u8 = 0x02;
/// `ESF-INS`.
pub const INS: u8 = 0x15;

/// Wheel of a wheel-tick measurement.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Wheel {
    /// Front left.
    FrontLeft,
    /// Front right.
    FrontRight,
    /// Rear left.
    RearLeft,
    /// Rear right.
    RearRight,
}

/// One `ESF-MEAS` measurement, by data type.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Measurement {
    /// Angular rate about a sensor axis, deg/s (types 14, 13, 5: x, y, z).
    AngularRate {
        /// Axis: 0 x, 1 y, 2 z.
        axis: u8,
        /// Rate, deg/s.
        degrees_per_second: f64,
    },
    /// Specific force along a sensor axis (types 16, 17, 18: x, y, z).
    Acceleration {
        /// Axis: 0 x, 1 y, 2 z.
        axis: u8,
        /// Specific force.
        value: Acceleration,
    },
    /// Gyroscope temperature, °C (type 12).
    GyroTemperature(f64),
    /// Accumulated ticks of one wheel (types 6–9).
    WheelTicks {
        /// Wheel.
        wheel: Wheel,
        /// Ticks, 23 bits, wrapping.
        ticks: u32,
        /// Moving backwards.
        backward: bool,
    },
    /// Accumulated ticks of a single speed sensor (type 10).
    SpeedTicks {
        /// Ticks, 23 bits, wrapping.
        ticks: u32,
        /// Moving backwards.
        backward: bool,
    },
    /// Speed (type 11), signed: negative backwards.
    Speed(Speed),
    /// A data type this crate does not read: type and the 24-bit field.
    Other {
        /// Data type, 0–63.
        data_type: u8,
        /// Field as sent.
        field: u32,
    },
}

/// 24-bit two's complement field as a signed integer.
fn signed_24(field: u32) -> i32 {
    let shifted = (field << 8).to_le_bytes();
    i32::from_le_bytes(shifted) >> 8
}

impl Measurement {
    fn from_word(word: u32) -> Self {
        let field = word & 0x00FF_FFFF;
        let data_type = u8::try_from((word >> 24) & 0x3F).unwrap_or(0);
        let ticks = |wheel| Self::WheelTicks {
            wheel,
            ticks: field & 0x007F_FFFF,
            backward: field & 0x0080_0000 != 0,
        };
        let rate = |axis| Self::AngularRate {
            axis,
            degrees_per_second: f64::from(signed_24(field)) / 4096.0,
        };
        let force = |axis| Self::Acceleration {
            axis,
            value: Acceleration::from_metres_per_second_squared(
                f64::from(signed_24(field)) / 1024.0,
            )
            .unwrap_or(Acceleration::ZERO),
        };
        match data_type {
            5 => rate(2),
            13 => rate(1),
            14 => rate(0),
            16 => force(0),
            17 => force(1),
            18 => force(2),
            12 => Self::GyroTemperature(f64::from(signed_24(field)) / 100.0),
            6 => ticks(Wheel::FrontLeft),
            7 => ticks(Wheel::FrontRight),
            8 => ticks(Wheel::RearLeft),
            9 => ticks(Wheel::RearRight),
            10 => Self::SpeedTicks {
                ticks: field & 0x007F_FFFF,
                backward: field & 0x0080_0000 != 0,
            },
            11 => Self::Speed(
                Speed::from_metres_per_second(f64::from(signed_24(field)) / 1000.0)
                    .unwrap_or(Speed::ZERO),
            ),
            _ => Self::Other { data_type, field },
        }
    }
}

/// `ESF-MEAS`: measurements of one sensor provider at one time tag.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EsfMeas<'a> {
    /// Sensor time tag, ms.
    pub time_tag: u32,
    /// Data provider.
    pub provider: u16,
    /// Calibrated time tag, ms, when present.
    pub calibrated_time_tag: Option<u32>,
    words: &'a [u8],
}

impl<'a> EsfMeas<'a> {
    pub(crate) fn decode(bytes: &'a [u8]) -> Result<Self, UbxError> {
        let flags = u16_at(bytes, 4).unwrap_or(0);
        let count = usize::from((flags >> 11) & 0x1F);
        let calibrated = flags & 0x08 != 0;
        let expected = 8 + 4 * count + if calibrated { 4 } else { 0 };
        if bytes.len() != expected {
            return Err(UbxError::Length {
                class: CLASS,
                id: MEAS,
                expected,
                found: bytes.len(),
            });
        }
        Ok(Self {
            time_tag: u32_at(bytes, 0).unwrap_or(0),
            provider: u16_at(bytes, 6).unwrap_or(0),
            calibrated_time_tag: calibrated.then(|| u32_at(bytes, 8 + 4 * count)).flatten(),
            words: bytes.get(8..8 + 4 * count).unwrap_or(&[]),
        })
    }

    /// The measurements, in order.
    pub fn measurements(&self) -> impl Iterator<Item = Measurement> + 'a {
        self.words
            .chunks_exact(4)
            .filter_map(|word| u32_at(word, 0))
            .map(Measurement::from_word)
    }
}

/// `ESF-INS`: compensated angular rates and accelerations in the vehicle
/// frame, gravity removed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EsfIns {
    /// GPS time of week, ms.
    pub itow: u32,
    /// Angular rates about x, y, z, deg/s; `None` where not valid.
    pub angular_rate: [Option<f64>; 3],
    /// Accelerations along forward, right, down; `None` where not valid.
    pub acceleration: [Option<Acceleration>; 3],
}

impl EsfIns {
    pub(crate) fn decode(bytes: &[u8]) -> Result<Self, UbxError> {
        if bytes.len() != 36 {
            return Err(UbxError::Length {
                class: CLASS,
                id: INS,
                expected: 36,
                found: bytes.len(),
            });
        }
        let flags = u32_at(bytes, 0).unwrap_or(0);
        let valid = |bit: u32| flags & (1 << bit) != 0;
        let field = |offset: usize| f64::from(i32_at(bytes, offset).unwrap_or(0));
        Ok(Self {
            itow: u32_at(bytes, 8).unwrap_or(0),
            angular_rate: [
                valid(8).then(|| field(12) * 1e-3),
                valid(9).then(|| field(16) * 1e-3),
                valid(10).then(|| field(20) * 1e-3),
            ],
            acceleration: [(11, 24), (12, 28), (13, 32)].map(|(bit, offset)| {
                valid(bit)
                    .then(|| {
                        Acceleration::from_metres_per_second_squared(field(offset) * 0.01).ok()
                    })
                    .flatten()
            }),
        })
    }

    /// Accelerations as a body-frame vector, when all three are valid.
    #[must_use]
    pub fn acceleration_vector(&self) -> Option<Vector3<Body, Acceleration>> {
        let [x, y, z] = self.acceleration;
        Some(Vector3::new(x?, y?, z?))
    }
}
