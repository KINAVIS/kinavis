//! A frame decoded by class and identifier.

use crate::config::{self, CfgValget};
use crate::error::UbxError;
use crate::esf::{self, EsfIns, EsfMeas};
use crate::frame::Frame;
use crate::nav::{self, NavDop, NavHpposllh, NavPvt, NavRelposned, NavSat, NavStatus, NavTimels};
use crate::read::{f64_at, i8_at, u16_at, u8_at};

/// Class `ACK`.
pub(crate) const ACK_CLASS: u8 = 0x05;
/// Class `RXM`.
pub(crate) const RXM_CLASS: u8 = 0x02;
/// `RXM-SFRBX`.
pub(crate) const RXM_SFRBX: u8 = 0x13;
/// `RXM-RAWX`.
pub(crate) const RXM_RAWX: u8 = 0x15;

/// `RXM-RAWX` header; the measurements are left as bytes, 32 per
/// measurement, for a solver to interpret.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RxmRawx<'a> {
    /// Receiver time of week, s.
    pub receiver_time_of_week: f64,
    /// GPS week.
    pub week: u16,
    /// `GPS − UTC`, s, as the receiver believes it.
    pub leap_seconds: i8,
    /// Receiver status flags.
    pub status: u8,
    /// Measurement blocks, 32 bytes each.
    pub measurements: &'a [u8],
}

/// `RXM-SFRBX` header; the navigation data words are left as bytes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RxmSfrbx<'a> {
    /// u-blox GNSS identifier.
    pub gnss: u8,
    /// Satellite number.
    pub sv: u8,
    /// Signal identifier.
    pub signal: u8,
    /// GLONASS frequency slot + 7.
    pub frequency: u8,
    /// Tracking channel.
    pub channel: u8,
    /// Data words, 4 bytes each, little-endian.
    pub words: &'a [u8],
}

/// A decoded UBX message.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Message<'a> {
    /// `NAV-PVT`.
    NavPvt(NavPvt),
    /// `NAV-HPPOSLLH`.
    NavHpposllh(NavHpposllh),
    /// `NAV-RELPOSNED`.
    NavRelposned(NavRelposned),
    /// `NAV-SAT`.
    NavSat(NavSat<'a>),
    /// `NAV-DOP`.
    NavDop(NavDop),
    /// `NAV-STATUS`.
    NavStatus(NavStatus),
    /// `NAV-TIMELS`.
    NavTimels(NavTimels),
    /// `ESF-MEAS`.
    EsfMeas(EsfMeas<'a>),
    /// `ESF-INS`.
    EsfIns(EsfIns),
    /// `ACK-ACK`: the receiver accepted a command.
    Ack {
        /// Class of the command.
        class: u8,
        /// Identifier of the command.
        id: u8,
    },
    /// `ACK-NAK`: the receiver refused a command.
    Nak {
        /// Class of the command.
        class: u8,
        /// Identifier of the command.
        id: u8,
    },
    /// `CFG-VALGET` response.
    CfgValget(CfgValget<'a>),
    /// `RXM-RAWX`.
    RxmRawx(RxmRawx<'a>),
    /// `RXM-SFRBX`.
    RxmSfrbx(RxmSfrbx<'a>),
    /// A message this crate does not decode; the frame was valid.
    Unsupported {
        /// Class.
        class: u8,
        /// Identifier.
        id: u8,
    },
}

impl<'a> Message<'a> {
    /// Decodes a frame.
    ///
    /// # Errors
    ///
    /// [`UbxError`] for a payload of the wrong length or version, or a field
    /// outside its domain. An unknown message is not an error: it returns
    /// [`Message::Unsupported`].
    pub fn decode(frame: &Frame<'a>) -> Result<Self, UbxError> {
        let payload = frame.payload;
        Ok(match (frame.class, frame.id) {
            (nav::CLASS, nav::PVT) => Self::NavPvt(NavPvt::decode(payload)?),
            (nav::CLASS, nav::HPPOSLLH) => Self::NavHpposllh(NavHpposllh::decode(payload)?),
            (nav::CLASS, nav::RELPOSNED) => Self::NavRelposned(NavRelposned::decode(payload)?),
            (nav::CLASS, nav::SAT) => Self::NavSat(NavSat::decode(payload)?),
            (nav::CLASS, nav::DOP) => Self::NavDop(NavDop::decode(payload)?),
            (nav::CLASS, nav::STATUS) => Self::NavStatus(NavStatus::decode(payload)?),
            (nav::CLASS, nav::TIMELS) => Self::NavTimels(NavTimels::decode(payload)?),
            (esf::CLASS, esf::MEAS) => Self::EsfMeas(EsfMeas::decode(payload)?),
            (esf::CLASS, esf::INS) => Self::EsfIns(EsfIns::decode(payload)?),
            (ACK_CLASS, 0x00 | 0x01) => {
                let [class, id] = <[u8; 2]>::try_from(payload).map_err(|_| UbxError::Length {
                    class: ACK_CLASS,
                    id: frame.id,
                    expected: 2,
                    found: payload.len(),
                })?;
                if frame.id == 0x01 {
                    Self::Ack { class, id }
                } else {
                    Self::Nak { class, id }
                }
            }
            (RXM_CLASS, RXM_RAWX) => {
                let count = usize::from(u8_at(payload, 11).unwrap_or(0));
                let expected = 16 + 32 * count;
                if payload.len() != expected {
                    return Err(UbxError::Length {
                        class: RXM_CLASS,
                        id: RXM_RAWX,
                        expected,
                        found: payload.len(),
                    });
                }
                Self::RxmRawx(RxmRawx {
                    receiver_time_of_week: f64_at(payload, 0).unwrap_or(0.0),
                    week: u16_at(payload, 8).unwrap_or(0),
                    leap_seconds: i8_at(payload, 10).unwrap_or(0),
                    status: u8_at(payload, 12).unwrap_or(0),
                    measurements: payload.get(16..).unwrap_or(&[]),
                })
            }
            (RXM_CLASS, RXM_SFRBX) => {
                let words = usize::from(u8_at(payload, 4).unwrap_or(0));
                let expected = 8 + 4 * words;
                if payload.len() != expected {
                    return Err(UbxError::Length {
                        class: RXM_CLASS,
                        id: RXM_SFRBX,
                        expected,
                        found: payload.len(),
                    });
                }
                Self::RxmSfrbx(RxmSfrbx {
                    gnss: u8_at(payload, 0).unwrap_or(0),
                    sv: u8_at(payload, 1).unwrap_or(0),
                    signal: u8_at(payload, 2).unwrap_or(0),
                    frequency: u8_at(payload, 3).unwrap_or(0),
                    channel: u8_at(payload, 5).unwrap_or(0),
                    words: payload.get(8..).unwrap_or(&[]),
                })
            }
            (config::CLASS, config::VALGET) => match CfgValget::decode(payload) {
                Some(answer) => Self::CfgValget(answer),
                None => Self::Unsupported {
                    class: frame.class,
                    id: frame.id,
                },
            },
            (class, id) => Self::Unsupported { class, id },
        })
    }
}
