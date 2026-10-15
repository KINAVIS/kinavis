//! Agricultural guidance: the steering's state and the curvature commanded.

use kinavis_j1939::Pgn;

use crate::slots::{Indicator, MAX_U16};

/// Largest curvature the bus carries, per metre: 8031.75 km⁻¹, a radius of
/// 12.5 cm.
pub const MAX_CURVATURE: f64 = 8.031_75;

/// Curvature resolution on the bus, km⁻¹ per bit.
const RESOLUTION: f64 = 0.25;
/// Curvature offset on the bus, km⁻¹.
const OFFSET: f64 = 8_032.0;

/// Limit on remote commands: whether the machine follows them.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum LimitStatus {
    /// Not limited.
    NotLimited,
    /// The operator has control; commands cannot be carried out.
    OperatorLimited,
    /// At the upper limit: only lower commands change anything.
    LimitedHigh,
    /// At the lower limit: only higher commands change anything.
    LimitedLow,
    /// A reserved code.
    Reserved(u8),
    /// A fault that does not clear.
    NonRecoverableFault,
    /// Not available.
    NotAvailable,
}

impl LimitStatus {
    pub(crate) const fn from_bits(bits: u8) -> Self {
        match bits & 0x07 {
            0 => Self::NotLimited,
            1 => Self::OperatorLimited,
            2 => Self::LimitedHigh,
            3 => Self::LimitedLow,
            6 => Self::NonRecoverableFault,
            7 => Self::NotAvailable,
            other => Self::Reserved(other),
        }
    }

    pub(crate) const fn bits(self) -> u8 {
        match self {
            Self::NotLimited => 0,
            Self::OperatorLimited => 1,
            Self::LimitedHigh => 2,
            Self::LimitedLow => 3,
            Self::Reserved(code) => code & 0x07,
            Self::NonRecoverableFault => 6,
            Self::NotAvailable => 7,
        }
    }
}

/// Curvature per metre of a two-byte field.
fn curvature(raw: u16) -> Option<f64> {
    (raw <= MAX_U16).then(|| (f64::from(raw) * RESOLUTION - OFFSET) / 1000.0)
}

/// Two-byte field of a curvature per metre, clamped to the range the bus
/// carries; not available for none.
fn curvature_raw(curvature: Option<f64>) -> u16 {
    curvature.map_or(u16::MAX, |per_metre| {
        let steps = kinavis_kernel::math::round((per_metre * 1000.0 + OFFSET) / RESOLUTION);
        let bounded = steps.max(0.0).min(f64::from(MAX_U16));
        u16::try_from(kinavis_kernel::math::to_usize(bounded)).unwrap_or(MAX_U16)
    })
}

/// Guidance machine info, PGN 44032: what the steering estimates and whether
/// it is ready to be steered.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct GuidanceMachineInfo {
    /// Estimated curvature of the machine's path, per metre, right positive.
    pub curvature: Option<f64>,
    /// Mechanical system lockout: `On` when locked out.
    pub mechanical_lockout: Indicator,
    /// Whether the steering system is ready to be steered.
    pub steering_ready: Indicator,
    /// Steering input position status.
    pub steering_input_position: Indicator,
    /// Whether the steering asks for a reset of the command.
    pub reset_required: Indicator,
    /// Limit on guidance commands.
    pub limit: LimitStatus,
    /// Why guidance ended, as its code (0 all clear, 3 operator override,
    /// 62 error); `None` when not available.
    pub exit_reason: Option<u8>,
    /// Remote engage switch.
    pub remote_engage: Indicator,
}

impl GuidanceMachineInfo {
    pub(crate) const PGN_NUMBER: u32 = 44_032;
    /// PGN 44032.
    pub const PGN: Pgn = Pgn::new(Self::PGN_NUMBER);

    /// From the 8 data bytes.
    #[must_use]
    pub fn from_bytes(bytes: [u8; 8]) -> Self {
        let [c0, c1, states, limit, exit, _, _, _] = bytes;
        let code = exit & 0x3F;
        Self {
            curvature: curvature(u16::from_le_bytes([c0, c1])),
            mechanical_lockout: Indicator::from_bits(states),
            steering_ready: Indicator::from_bits(states >> 2),
            steering_input_position: Indicator::from_bits(states >> 4),
            reset_required: Indicator::from_bits(states >> 6),
            limit: LimitStatus::from_bits(limit >> 5),
            exit_reason: (code != 0x3F).then_some(code),
            remote_engage: Indicator::from_bits(exit >> 6),
        }
    }

    /// The 8 data bytes; the reserved bits are ones.
    #[must_use]
    pub fn to_bytes(&self) -> [u8; 8] {
        let [c0, c1] = curvature_raw(self.curvature).to_le_bytes();
        let states = self.mechanical_lockout.bits()
            | self.steering_ready.bits() << 2
            | self.steering_input_position.bits() << 4
            | self.reset_required.bits() << 6;
        let exit =
            self.exit_reason.map_or(0x3F, |code| code.min(0x3F)) | self.remote_engage.bits() << 6;
        [
            c0,
            c1,
            states,
            0x1F | self.limit.bits() << 5,
            exit,
            0xFF,
            0xFF,
            0xFF,
        ]
    }
}

/// Guidance system command, PGN 44288: the curvature the guidance system
/// asks the steering to follow.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct GuidanceSystemCommand {
    /// Commanded curvature, per metre, right positive.
    pub curvature: Option<f64>,
    /// `On` when the guidance system intends to steer.
    pub intends_to_steer: Indicator,
}

impl GuidanceSystemCommand {
    pub(crate) const PGN_NUMBER: u32 = 44_288;
    /// PGN 44288.
    pub const PGN: Pgn = Pgn::new(Self::PGN_NUMBER);

    /// From the 8 data bytes.
    #[must_use]
    pub fn from_bytes(bytes: [u8; 8]) -> Self {
        let [c0, c1, status, _, _, _, _, _] = bytes;
        Self {
            curvature: curvature(u16::from_le_bytes([c0, c1])),
            intends_to_steer: Indicator::from_bits(status),
        }
    }

    /// The 8 data bytes; the reserved bits are ones.
    #[must_use]
    pub fn to_bytes(&self) -> [u8; 8] {
        let [c0, c1] = curvature_raw(self.curvature).to_le_bytes();
        [
            c0,
            c1,
            0xFC | self.intends_to_steer.bits(),
            0xFF,
            0xFF,
            0xFF,
            0xFF,
            0xFF,
        ]
    }
}
