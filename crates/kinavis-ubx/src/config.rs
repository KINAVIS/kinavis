//! Receiver configuration, class `CFG` (0x06): the configuration database of
//! protocol 23.01 and later (F9, X20, M10) through `CFG-VALSET` and
//! `CFG-VALGET`, and `CFG-RATE` for older receivers.
//!
//! Messages are written into a caller's buffer as complete frames, checksum
//! included, ready for the serial port.

use core::fmt;

use crate::frame::{checksum, SYNC_1, SYNC_2};
use crate::read::{u16_at, u32_at, u8_at};

/// Class `CFG`.
pub const CLASS: u8 = 0x06;
/// `CFG-RATE`.
pub const RATE: u8 = 0x08;
/// `CFG-VALSET`.
pub const VALSET: u8 = 0x8A;
/// `CFG-VALGET`.
pub const VALGET: u8 = 0x8B;

/// Most keys in one `CFG-VALSET` or `CFG-VALGET`.
pub const MAX_KEYS: usize = 64;

/// A configuration key: group, item and the storage size of its value, which
/// bits 28–30 encode.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ConfigKey(u32);

impl ConfigKey {
    /// Measurement period, ms (`CFG-RATE-MEAS`).
    pub const RATE_MEAS: Self = Self(0x3021_0001);
    /// Measurements per navigation solution (`CFG-RATE-NAV`).
    pub const RATE_NAV: Self = Self(0x3021_0002);
    /// Time system of the measurement rate (`CFG-RATE-TIMEREF`).
    pub const RATE_TIMEREF: Self = Self(0x2021_0003);
    /// Dynamic platform model (`CFG-NAVSPG-DYNMODEL`).
    pub const NAVSPG_DYNMODEL: Self = Self(0x2011_0021);
    /// UART1 baud rate (`CFG-UART1-BAUDRATE`).
    pub const UART1_BAUDRATE: Self = Self(0x4052_0001);
    /// UBX output on UART1 (`CFG-UART1OUTPROT-UBX`).
    pub const UART1_OUTPUT_UBX: Self = Self(0x1074_0001);
    /// NMEA output on UART1 (`CFG-UART1OUTPROT-NMEA`).
    pub const UART1_OUTPUT_NMEA: Self = Self(0x1074_0002);
    /// `NAV-PVT` rate on UART1, per navigation solution.
    pub const MSGOUT_NAV_PVT_UART1: Self = Self(0x2091_0007);
    /// `NAV-PVT` rate on USB.
    pub const MSGOUT_NAV_PVT_USB: Self = Self(0x2091_0009);
    /// `NAV-SAT` rate on UART1.
    pub const MSGOUT_NAV_SAT_UART1: Self = Self(0x2091_0016);
    /// `NAV-HPPOSLLH` rate on UART1.
    pub const MSGOUT_NAV_HPPOSLLH_UART1: Self = Self(0x2091_0034);
    /// `NAV-RELPOSNED` rate on UART1.
    pub const MSGOUT_NAV_RELPOSNED_UART1: Self = Self(0x2091_008E);
    /// `NAV-TIMELS` rate on UART1.
    pub const MSGOUT_NAV_TIMELS_UART1: Self = Self(0x2091_0061);
    /// `ESF-MEAS` rate on UART1.
    pub const MSGOUT_ESF_MEAS_UART1: Self = Self(0x2091_0278);

    /// Key from its 32-bit identifier.
    ///
    /// # Errors
    ///
    /// [`ConfigError::BadKey`] if bits 28–30 name no storage size.
    pub const fn new(raw: u32) -> Result<Self, ConfigError> {
        match (raw >> 28) & 0x7 {
            1..=5 => Ok(Self(raw)),
            _ => Err(ConfigError::BadKey(raw)),
        }
    }

    /// The 32-bit identifier.
    #[must_use]
    pub const fn raw(self) -> u32 {
        self.0
    }

    /// Bytes of the value: 1 (a bit or a byte), 2, 4 or 8.
    #[must_use]
    pub const fn value_size(self) -> usize {
        match (self.0 >> 28) & 0x7 {
            3 => 2,
            4 => 4,
            5 => 8,
            _ => 1,
        }
    }
}

/// Layers a `CFG-VALSET` writes: any union of RAM, battery-backed RAM and
/// flash.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Layers(u8);

impl Layers {
    /// RAM: effective at once, lost at power-off.
    pub const RAM: Self = Self(0x01);
    /// Battery-backed RAM.
    pub const BBR: Self = Self(0x02);
    /// Flash, where fitted.
    pub const FLASH: Self = Self(0x04);

    /// Both layers.
    #[must_use]
    pub const fn and(self, other: Self) -> Self {
        Self(self.0 | other.0)
    }
}

/// Layer a `CFG-VALGET` reads.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PollLayer {
    /// Current configuration.
    Ram,
    /// Battery-backed RAM.
    Bbr,
    /// Flash.
    Flash,
    /// Factory defaults.
    Default,
}

impl PollLayer {
    const fn byte(self) -> u8 {
        match self {
            Self::Ram => 0,
            Self::Bbr => 1,
            Self::Flash => 2,
            Self::Default => 7,
        }
    }
}

/// Time system of `CFG-RATE`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum TimeReference {
    /// UTC.
    Utc,
    /// GPS time.
    Gps,
    /// GLONASS time.
    Glonass,
    /// BeiDou time.
    BeiDou,
    /// Galileo time.
    Galileo,
}

/// Why a configuration message could not be written or read.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConfigError {
    /// The buffer is shorter than the frame.
    BufferTooSmall {
        /// Bytes the frame needs.
        needed: usize,
    },
    /// More than [`MAX_KEYS`] keys.
    TooManyKeys(usize),
    /// A value does not fit its key's storage size.
    ValueTooLarge {
        /// Key.
        key: ConfigKey,
        /// Value.
        value: u64,
    },
    /// A key whose bits 28–30 name no storage size.
    BadKey(u32),
    /// A `CFG-VALGET` response that ends inside a value.
    Truncated,
}

impl fmt::Display for ConfigError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::BufferTooSmall { needed } => write!(f, "the frame needs {needed} bytes"),
            Self::TooManyKeys(count) => write!(f, "{count} keys, at most {MAX_KEYS}"),
            Self::ValueTooLarge { key, value } => write!(
                f,
                "value {value} does not fit the {} bytes of key {:#010X}",
                key.value_size(),
                key.raw()
            ),
            Self::BadKey(raw) => write!(f, "key {raw:#010X} names no value size"),
            Self::Truncated => f.write_str("configuration response ends inside a value"),
        }
    }
}

impl core::error::Error for ConfigError {}

/// Writes a complete frame — sync, class, identifier, length, payload,
/// checksum — and returns its length. The payload is produced in place by
/// `body`, which returns its length.
fn frame(
    out: &mut [u8],
    class: u8,
    id: u8,
    payload_length: usize,
    body: impl FnOnce(&mut [u8]),
) -> Result<usize, ConfigError> {
    let needed = payload_length.saturating_add(8);
    let length =
        u16::try_from(payload_length).map_err(|_| ConfigError::BufferTooSmall { needed })?;
    let frame = out
        .get_mut(..needed)
        .ok_or(ConfigError::BufferTooSmall { needed })?;
    let [low, high] = length.to_le_bytes();
    for (slot, byte) in frame.iter_mut().zip([SYNC_1, SYNC_2, class, id, low, high]) {
        *slot = byte;
    }
    let end = needed.saturating_sub(2);
    let payload = frame
        .get_mut(6..end)
        .ok_or(ConfigError::BufferTooSmall { needed })?;
    body(payload);
    let (a, b) = checksum(class, id, payload);
    for (slot, byte) in frame.iter_mut().skip(end).zip([a, b]) {
        *slot = byte;
    }
    Ok(needed)
}

/// `CFG-VALSET` version 0: sets each key to its value in `layers`.
///
/// # Errors
///
/// [`ConfigError::TooManyKeys`], [`ConfigError::ValueTooLarge`],
/// [`ConfigError::BufferTooSmall`].
pub fn valset(
    layers: Layers,
    items: &[(ConfigKey, u64)],
    out: &mut [u8],
) -> Result<usize, ConfigError> {
    if items.len() > MAX_KEYS {
        return Err(ConfigError::TooManyKeys(items.len()));
    }
    for &(key, value) in items {
        let bits = 8 * u32::try_from(key.value_size()).unwrap_or(8);
        if value.checked_shr(bits).is_some_and(|rest| rest != 0) {
            return Err(ConfigError::ValueTooLarge { key, value });
        }
    }
    let length = items.iter().fold(4_usize, |sum, (key, _)| {
        sum.saturating_add(4).saturating_add(key.value_size())
    });
    frame(out, CLASS, VALSET, length, |payload| {
        let mut at: usize = 0;
        let mut put = |bytes: &[u8]| {
            let start = at;
            at = at.saturating_add(bytes.len());
            for (slot, byte) in payload.iter_mut().skip(start).zip(bytes) {
                *slot = *byte;
            }
        };
        put(&[0, layers.0, 0, 0]);
        for &(key, value) in items {
            put(&key.raw().to_le_bytes());
            put(value.to_le_bytes().get(..key.value_size()).unwrap_or(&[]));
        }
    })
}

/// `CFG-VALGET` poll version 0: asks for the values of `keys` in `layer`,
/// from `position` in the receiver's answer.
///
/// # Errors
///
/// [`ConfigError::TooManyKeys`], [`ConfigError::BufferTooSmall`].
pub fn valget(
    layer: PollLayer,
    position: u16,
    keys: &[ConfigKey],
    out: &mut [u8],
) -> Result<usize, ConfigError> {
    if keys.len() > MAX_KEYS {
        return Err(ConfigError::TooManyKeys(keys.len()));
    }
    frame(
        out,
        CLASS,
        VALGET,
        keys.len().saturating_mul(4).saturating_add(4),
        |payload| {
            let position = position.to_le_bytes();
            for (slot, byte) in payload
                .iter_mut()
                .zip([0, layer.byte(), position[0], position[1]])
            {
                *slot = byte;
            }
            for (chunk, key) in payload
                .get_mut(4..)
                .unwrap_or(&mut [])
                .chunks_exact_mut(4)
                .zip(keys)
            {
                for (slot, byte) in chunk.iter_mut().zip(key.raw().to_le_bytes()) {
                    *slot = byte;
                }
            }
        },
    )
}

/// `CFG-RATE`: a measurement every `measurement_ms`, a navigation solution
/// every `navigation_cycles` measurements, aligned to `reference`.
///
/// # Errors
///
/// [`ConfigError::BufferTooSmall`].
pub fn rate(
    measurement_ms: u16,
    navigation_cycles: u16,
    reference: TimeReference,
    out: &mut [u8],
) -> Result<usize, ConfigError> {
    let reference: u16 = match reference {
        TimeReference::Utc => 0,
        TimeReference::Gps => 1,
        TimeReference::Glonass => 2,
        TimeReference::BeiDou => 3,
        TimeReference::Galileo => 4,
    };
    frame(out, CLASS, RATE, 6, |payload| {
        let bytes = [
            measurement_ms.to_le_bytes(),
            navigation_cycles.to_le_bytes(),
            reference.to_le_bytes(),
        ];
        for (chunk, value) in payload.chunks_exact_mut(2).zip(bytes) {
            for (slot, byte) in chunk.iter_mut().zip(value) {
                *slot = byte;
            }
        }
    })
}

/// A `CFG-VALGET` response: the layer it reports and its key–value pairs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CfgValget<'a> {
    /// Layer, as polled.
    pub layer: u8,
    /// Position of the first pair in the receiver's full answer.
    pub position: u16,
    pairs: &'a [u8],
}

impl<'a> CfgValget<'a> {
    /// Version 1 (a response) only; version 0 is a poll sent to the receiver.
    pub(crate) fn decode(payload: &'a [u8]) -> Option<Self> {
        if u8_at(payload, 0)? != 1 {
            return None;
        }
        Some(Self {
            layer: u8_at(payload, 1)?,
            position: u16_at(payload, 2)?,
            pairs: payload.get(4..)?,
        })
    }

    /// The key–value pairs, in order; a value is little-endian, zero-extended.
    pub fn values(&self) -> impl Iterator<Item = Result<(ConfigKey, u64), ConfigError>> + 'a {
        let mut rest = self.pairs;
        core::iter::from_fn(move || {
            if rest.is_empty() {
                return None;
            }
            let Some(raw) = u32_at(rest, 0) else {
                rest = &[];
                return Some(Err(ConfigError::Truncated));
            };
            let key = match ConfigKey::new(raw) {
                Ok(key) => key,
                Err(error) => {
                    rest = &[];
                    return Some(Err(error));
                }
            };
            let end = key.value_size().saturating_add(4);
            let Some(bytes) = rest.get(4..end) else {
                rest = &[];
                return Some(Err(ConfigError::Truncated));
            };
            let mut value = [0_u8; 8];
            for (slot, byte) in value.iter_mut().zip(bytes) {
                *slot = *byte;
            }
            rest = rest.get(end..).unwrap_or(&[]);
            Some(Ok((key, u64::from_le_bytes(value))))
        })
    }
}
