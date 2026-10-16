//! Transport protocol (TP, J1939-21 and ISO 11783-3): parameter groups of 9
//! to 1785 bytes in frames of 7, broadcast (BAM) or to one node with flow
//! control (RTS/CTS).
//!
//! Every type here is a state machine over frames and instants: frames from
//! the bus go in, completed transfers and frames to send come out. Timing —
//! when to call [`TransportReceiver::expire`], the gap between broadcast
//! packets — is the caller's.

use core::time::Duration;

use kinavis_kernel::time::{Instant, Utc};

use crate::error::J1939Error;
use crate::frame::Frame;
use crate::id::{CanId, Pgn};

/// Longest transfer of the transport protocol, bytes.
pub const TP_MAX_BYTES: usize = 1785;

/// Longest gap between two data packets.
pub const T1: Duration = Duration::from_millis(750);
/// Longest wait for data after a clear-to-send.
pub const T2: Duration = Duration::from_millis(1250);
/// Longest wait of a sender for a clear-to-send or the acknowledgement.
pub const T3: Duration = Duration::from_millis(1250);
/// Longest hold of a sender told to wait (clear-to-send of zero packets).
pub const T4: Duration = Duration::from_millis(1050);

/// Priority of transport frames.
const PRIORITY: u8 = 7;

const RTS: u8 = 16;
const CTS: u8 = 17;
const END_OF_MESSAGE: u8 = 19;
const BAM: u8 = 32;
const ABORT: u8 = 255;

/// Why a connection was aborted, ISO 11783-3.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum AbortReason {
    /// Already in a connection with this node.
    AlreadyInSession,
    /// No resources for the transfer: too large for the receiver, or no slot.
    ResourcesNeeded,
    /// A timeout.
    Timeout,
    /// A clear-to-send came while packets were being sent.
    ClearToSendDuringTransfer,
    /// The retransmit limit was reached.
    RetransmitLimit,
    /// A data packet came outside a connection.
    UnexpectedData,
    /// A data packet out of sequence.
    BadSequence,
    /// A data packet seen twice.
    DuplicateSequence,
    /// More than 1785 bytes announced.
    TooLong,
    /// Any other reason, as its code.
    Other(u8),
}

impl AbortReason {
    /// Code on the bus.
    #[must_use]
    pub const fn code(self) -> u8 {
        match self {
            Self::AlreadyInSession => 1,
            Self::ResourcesNeeded => 2,
            Self::Timeout => 3,
            Self::ClearToSendDuringTransfer => 4,
            Self::RetransmitLimit => 5,
            Self::UnexpectedData => 6,
            Self::BadSequence => 7,
            Self::DuplicateSequence => 8,
            Self::TooLong => 9,
            Self::Other(code) => code,
        }
    }

    /// Reason of a code.
    #[must_use]
    pub const fn from_code(code: u8) -> Self {
        match code {
            1 => Self::AlreadyInSession,
            2 => Self::ResourcesNeeded,
            3 => Self::Timeout,
            4 => Self::ClearToSendDuringTransfer,
            5 => Self::RetransmitLimit,
            6 => Self::UnexpectedData,
            7 => Self::BadSequence,
            8 => Self::DuplicateSequence,
            9 => Self::TooLong,
            other => Self::Other(other),
        }
    }
}

/// A parameter group carried by the transport protocol, at most `B` bytes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Transfer<const B: usize> {
    /// PGN carried.
    pub pgn: Pgn,
    /// Sender.
    pub source: u8,
    /// Receiver; `None` for a broadcast.
    pub destination: Option<u8>,
    len: usize,
    bytes: [u8; B],
}

impl<const B: usize> Transfer<B> {
    pub(crate) const fn new(
        pgn: Pgn,
        source: u8,
        destination: Option<u8>,
        len: usize,
        bytes: [u8; B],
    ) -> Self {
        Self {
            pgn,
            source,
            destination,
            len,
            bytes,
        }
    }

    /// Data bytes.
    #[must_use]
    pub fn data(&self) -> &[u8] {
        self.bytes.get(..self.len).unwrap_or(&[])
    }
}

/// Outcome of one frame given to a [`TransportReceiver`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Received<const B: usize> {
    /// A transfer completed by the frame.
    pub transfer: Option<Transfer<B>>,
    /// Frame to send in answer: a clear-to-send, the end-of-message
    /// acknowledgement, or an abort.
    pub reply: Option<Frame>,
    /// A session dropped by the frame, and why.
    pub dropped: Option<AbortReason>,
}

impl<const B: usize> Received<B> {
    const NOTHING: Self = Self {
        transfer: None,
        reply: None,
        dropped: None,
    };
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Role {
    /// A broadcast, heard.
    Broadcast,
    /// A connection to this node: clear-to-send and acknowledgement are ours.
    Participant {
        /// Last packet of the window granted.
        window_end: u8,
        /// Whether the last thing done was granting a window.
        after_cts: bool,
    },
    /// A connection between two other nodes, followed.
    Observer,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Session<const B: usize> {
    role: Role,
    pgn: Pgn,
    source: u8,
    destination: u8,
    size: usize,
    packets: u8,
    /// Sequence number expected next, from 1.
    next: u8,
    /// Most packets per clear-to-send the sender takes.
    sender_window: u8,
    last: Instant<Utc>,
    bytes: [u8; B],
}

/// Receiver of transport-protocol transfers of up to `B` bytes, `S` at a time.
///
/// Broadcasts are always received. Connections addressed to the local
/// address are answered: clear-to-send in windows, the end-of-message
/// acknowledgement, aborts. Without a local address every connection on the
/// bus is followed silently, as a bus monitor does.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TransportReceiver<const S: usize, const B: usize> {
    local: Option<u8>,
    window: u8,
    sessions: [Option<Session<B>>; S],
}

impl<const S: usize, const B: usize> TransportReceiver<S, B> {
    /// Receiver for a node at `local`, or a monitor of the whole bus for
    /// `None`, granting up to 16 packets per clear-to-send.
    #[must_use]
    pub const fn new(local: Option<u8>) -> Self {
        Self {
            local,
            window: 16,
            sessions: [None; S],
        }
    }

    /// The same receiver granting up to `packets` per clear-to-send, at least
    /// one.
    #[must_use]
    pub const fn with_window(self, packets: u8) -> Self {
        Self {
            window: if packets == 0 { 1 } else { packets },
            ..self
        }
    }

    /// Sessions in progress.
    #[must_use]
    pub fn pending(&self) -> usize {
        self.sessions.iter().flatten().count()
    }

    /// Takes in a frame from the bus; frames of other parameter groups are
    /// passed over.
    pub fn push(&mut self, frame: &Frame, at: Instant<Utc>) -> Received<B> {
        match frame.id.pgn() {
            Pgn::TP_CONNECTION => self.connection(frame, at),
            Pgn::TP_DATA => self.data(frame, at),
            _ => Received::NOTHING,
        }
    }

    /// Drops sessions silent beyond their timeout; the aborts owed for
    /// connections to this node go to `send`. Returns how many were dropped.
    pub fn expire(&mut self, at: Instant<Utc>, mut send: impl FnMut(Frame)) -> usize {
        let local = self.local;
        let mut dropped = 0usize;
        for slot in &mut self.sessions {
            let Some(session) = slot else {
                continue;
            };
            let limit = match session.role {
                Role::Participant {
                    after_cts: true, ..
                } => T2,
                _ => T1,
            };
            let silent = at
                .checked_duration_since(session.last)
                .is_some_and(|waited| waited > limit);
            if !silent {
                continue;
            }
            if let (Role::Participant { .. }, Some(local)) = (session.role, local) {
                send(abort(
                    local,
                    session.source,
                    session.pgn,
                    AbortReason::Timeout,
                ));
            }
            *slot = None;
            dropped = dropped.saturating_add(1);
        }
        dropped
    }

    fn connection(&mut self, frame: &Frame, at: Instant<Utc>) -> Received<B> {
        let Ok(bytes) = <[u8; 8]>::try_from(frame.data()) else {
            return Received::NOTHING;
        };
        let [control, size_low, size_high, packets, _, p0, p1, p2] = bytes;
        let pgn = Pgn::from_bytes([p0, p1, p2]);
        let source = frame.id.source();
        let destination = frame.id.destination().unwrap_or(CanId::GLOBAL_ADDRESS);
        let size = usize::from(u16::from_le_bytes([size_low, size_high]));
        match control {
            BAM if destination == CanId::GLOBAL_ADDRESS => {
                self.remove(source, destination);
                if !announced(size, packets) || size > B {
                    return Received {
                        dropped: Some(AbortReason::ResourcesNeeded),
                        ..Received::NOTHING
                    };
                }
                self.open(
                    Role::Broadcast,
                    pgn,
                    source,
                    destination,
                    size,
                    packets,
                    0,
                    at,
                )
            }
            RTS => self.request_to_send(bytes, pgn, source, destination, size, at),
            // A clear-to-send between other nodes may ask for packets again.
            CTS => {
                // [17, packets, next, …]
                let (count, next) = (size_low, size_high);
                if let Some(session) = self.find(destination, source) {
                    if session.role == Role::Observer && count > 0 && next > 0 {
                        session.next = next;
                        session.last = at;
                    }
                }
                Received::NOTHING
            }
            ABORT => {
                let removed = self.remove(source, destination) || self.remove(destination, source);
                Received {
                    dropped: removed.then_some(AbortReason::from_code(size_low)),
                    ..Received::NOTHING
                }
            }
            _ => Received::NOTHING,
        }
    }

    fn request_to_send(
        &mut self,
        bytes: [u8; 8],
        pgn: Pgn,
        source: u8,
        destination: u8,
        size: usize,
        at: Instant<Utc>,
    ) -> Received<B> {
        let [_, _, _, packets, window, _, _, _] = bytes;
        let participant = Some(destination) == self.local;
        if !participant && self.local.is_some() {
            return Received::NOTHING;
        }
        self.remove(source, destination);
        if participant {
            let refusal = if size > TP_MAX_BYTES {
                Some(AbortReason::TooLong)
            } else if size > B || !announced(size, packets) {
                Some(AbortReason::ResourcesNeeded)
            } else {
                None
            };
            if let Some(reason) = refusal {
                return Received {
                    reply: Some(abort(destination, source, pgn, reason)),
                    dropped: Some(reason),
                    ..Received::NOTHING
                };
            }
            let granted = self.window.min(window).min(packets);
            let opened = self.open(
                Role::Participant {
                    window_end: granted,
                    after_cts: true,
                },
                pgn,
                source,
                destination,
                size,
                packets,
                window,
                at,
            );
            if opened.dropped.is_some() {
                return Received {
                    reply: Some(abort(
                        destination,
                        source,
                        pgn,
                        AbortReason::ResourcesNeeded,
                    )),
                    ..opened
                };
            }
            return Received {
                reply: Some(clear_to_send(destination, source, pgn, granted, 1)),
                ..opened
            };
        }
        if !announced(size, packets) || size > B {
            return Received {
                dropped: Some(AbortReason::ResourcesNeeded),
                ..Received::NOTHING
            };
        }
        self.open(
            Role::Observer,
            pgn,
            source,
            destination,
            size,
            packets,
            window,
            at,
        )
    }

    fn data(&mut self, frame: &Frame, at: Instant<Utc>) -> Received<B> {
        let source = frame.id.source();
        let destination = frame.id.destination().unwrap_or(CanId::GLOBAL_ADDRESS);
        let local = self.local;
        let window = self.window;
        let Some(index) = self.position(source, destination) else {
            return Received::NOTHING;
        };
        let Some(Some(session)) = self.sessions.get_mut(index) else {
            return Received::NOTHING;
        };
        let Some((&sequence, chunk)) = frame.data().split_first() else {
            return Received::NOTHING;
        };
        if sequence != session.next {
            let reason = if sequence < session.next {
                AbortReason::DuplicateSequence
            } else {
                AbortReason::BadSequence
            };
            let reply = match (session.role, local) {
                (Role::Participant { .. }, Some(local)) => {
                    Some(abort(local, session.source, session.pgn, reason))
                }
                _ => None,
            };
            if let Some(slot) = self.sessions.get_mut(index) {
                *slot = None;
            }
            return Received {
                reply,
                dropped: Some(reason),
                ..Received::NOTHING
            };
        }
        let offset = usize::from(sequence.saturating_sub(1)).saturating_mul(7);
        let end = offset.saturating_add(7).min(session.size);
        if let (Some(slots), Some(chunk)) = (
            session.bytes.get_mut(offset..end),
            chunk.get(..end.saturating_sub(offset)),
        ) {
            slots.copy_from_slice(chunk);
        }
        session.next = sequence.saturating_add(1);
        session.last = at;
        if sequence >= session.packets {
            return self.complete(index, local);
        }
        if let (
            Role::Participant {
                window_end,
                after_cts,
            },
            Some(local),
        ) = (session.role, local)
        {
            if sequence == window_end {
                let remaining = session.packets.saturating_sub(sequence);
                let granted = window.min(session.sender_window).min(remaining);
                session.role = Role::Participant {
                    window_end: sequence.saturating_add(granted),
                    after_cts: true,
                };
                let next = session.next;
                return Received {
                    reply: Some(clear_to_send(
                        local,
                        session.source,
                        session.pgn,
                        granted,
                        next,
                    )),
                    ..Received::NOTHING
                };
            }
            if after_cts {
                session.role = Role::Participant {
                    window_end,
                    after_cts: false,
                };
            }
        }
        Received::NOTHING
    }

    /// Hands over the completed session at `index`, with the acknowledgement
    /// owed if it was a connection to this node.
    fn complete(&mut self, index: usize, local: Option<u8>) -> Received<B> {
        let Some(slot) = self.sessions.get_mut(index) else {
            return Received::NOTHING;
        };
        let Some(done) = slot.take() else {
            return Received::NOTHING;
        };
        let reply = match (done.role, local) {
            (Role::Participant { .. }, Some(local)) => Some(end_of_message(
                local,
                done.source,
                done.pgn,
                done.size,
                done.packets,
            )),
            _ => None,
        };
        Received {
            transfer: Some(Transfer::new(
                done.pgn,
                done.source,
                (done.destination != CanId::GLOBAL_ADDRESS).then_some(done.destination),
                done.size,
                done.bytes,
            )),
            reply,
            dropped: None,
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn open(
        &mut self,
        role: Role,
        pgn: Pgn,
        source: u8,
        destination: u8,
        size: usize,
        packets: u8,
        sender_window: u8,
        at: Instant<Utc>,
    ) -> Received<B> {
        let Some(slot) = self.sessions.iter_mut().find(|slot| slot.is_none()) else {
            return Received {
                dropped: Some(AbortReason::ResourcesNeeded),
                ..Received::NOTHING
            };
        };
        *slot = Some(Session {
            role,
            pgn,
            source,
            destination,
            size,
            packets,
            next: 1,
            sender_window: if sender_window == 0 {
                u8::MAX
            } else {
                sender_window
            },
            last: at,
            bytes: [0; B],
        });
        Received::NOTHING
    }

    fn position(&self, source: u8, destination: u8) -> Option<usize> {
        self.sessions.iter().position(|slot| {
            slot.is_some_and(|session| {
                session.source == source && session.destination == destination
            })
        })
    }

    fn find(&mut self, source: u8, destination: u8) -> Option<&mut Session<B>> {
        self.sessions
            .iter_mut()
            .flatten()
            .find(|session| session.source == source && session.destination == destination)
    }

    fn remove(&mut self, source: u8, destination: u8) -> bool {
        let Some(index) = self.position(source, destination) else {
            return false;
        };
        if let Some(slot) = self.sessions.get_mut(index) {
            *slot = None;
        }
        true
    }
}

/// Whether a size and a packet count agree: `⌈size / 7⌉` packets, 9 to 1785
/// bytes.
fn announced(size: usize, packets: u8) -> bool {
    (9..=TP_MAX_BYTES).contains(&size) && usize::from(packets) == size.div_ceil(7)
}

fn frame(source: u8, destination: u8, data: [u8; 8]) -> Frame {
    Frame::truncated(
        CanId::from_parts(PRIORITY, Pgn::TP_CONNECTION, destination, source),
        &data,
    )
}

fn pgn_bytes(pgn: Pgn) -> [u8; 3] {
    pgn.to_bytes()
}

fn clear_to_send(source: u8, destination: u8, pgn: Pgn, packets: u8, next: u8) -> Frame {
    let [p0, p1, p2] = pgn_bytes(pgn);
    frame(
        source,
        destination,
        [CTS, packets, next, 0xFF, 0xFF, p0, p1, p2],
    )
}

fn end_of_message(source: u8, destination: u8, pgn: Pgn, size: usize, packets: u8) -> Frame {
    let [low, high] = u16::try_from(size).unwrap_or(u16::MAX).to_le_bytes();
    let [p0, p1, p2] = pgn_bytes(pgn);
    frame(
        source,
        destination,
        [END_OF_MESSAGE, low, high, packets, 0xFF, p0, p1, p2],
    )
}

/// Connection abort from `source` to `destination`.
#[must_use]
pub fn abort(source: u8, destination: u8, pgn: Pgn, reason: AbortReason) -> Frame {
    let [p0, p1, p2] = pgn_bytes(pgn);
    frame(
        source,
        destination,
        [ABORT, reason.code(), 0xFF, 0xFF, 0xFF, p0, p1, p2],
    )
}

/// The frames of a broadcast (BAM) of `data` from `source`: the
/// announcement, then the data packets, to `send` in order. The gap of 50 to
/// 200 ms between packets is the caller's to keep.
///
/// # Errors
///
/// [`J1939Error::TooLong`] for fewer than 9 bytes (a single frame carries
/// them) or more than [`TP_MAX_BYTES`].
pub fn broadcast(
    pgn: Pgn,
    source: u8,
    data: &[u8],
    mut send: impl FnMut(Frame),
) -> Result<(), J1939Error> {
    let size = data.len();
    let packets = packets_of(pgn, size)?;
    let [low, high] = u16::try_from(size).unwrap_or(u16::MAX).to_le_bytes();
    let [p0, p1, p2] = pgn_bytes(pgn);
    send(frame(
        source,
        CanId::GLOBAL_ADDRESS,
        [BAM, low, high, packets, 0xFF, p0, p1, p2],
    ));
    for (index, chunk) in data.chunks(7).enumerate() {
        send(data_packet(source, CanId::GLOBAL_ADDRESS, index, chunk));
    }
    Ok(())
}

fn packets_of(pgn: Pgn, size: usize) -> Result<u8, J1939Error> {
    let fits = (9..=TP_MAX_BYTES).contains(&size);
    let limit = u32::try_from(TP_MAX_BYTES).unwrap_or(u32::MAX);
    if !fits {
        return Err(J1939Error::TooLong {
            pgn,
            size: u32::try_from(size).unwrap_or(u32::MAX),
            limit,
        });
    }
    Ok(u8::try_from(size.div_ceil(7)).unwrap_or(u8::MAX))
}

fn data_packet(source: u8, destination: u8, index: usize, chunk: &[u8]) -> Frame {
    let mut bytes = [0xFF; 8];
    if let Some((sequence, rest)) = bytes.split_first_mut() {
        *sequence = u8::try_from(index.saturating_add(1)).unwrap_or(u8::MAX);
        for (slot, byte) in rest.iter_mut().zip(chunk) {
            *slot = *byte;
        }
    }
    Frame::truncated(
        CanId::from_parts(PRIORITY, Pgn::TP_DATA, destination, source),
        &bytes,
    )
}

/// Where a [`TransportSender`] stands.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SenderState {
    /// Not started.
    Idle,
    /// Waiting for a clear-to-send.
    Waiting,
    /// Told to hold by a clear-to-send of zero packets.
    Holding,
    /// The receiver acknowledged the whole transfer.
    Done,
    /// Aborted, by either side.
    Aborted(AbortReason),
}

/// Sender of one transfer of up to `B` bytes to one node, with flow control
/// (RTS/CTS).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TransportSender<const B: usize> {
    pgn: Pgn,
    source: u8,
    destination: u8,
    len: usize,
    packets: u8,
    bytes: [u8; B],
    state: SenderState,
    since: Instant<Utc>,
}

impl<const B: usize> TransportSender<B> {
    /// Sender of `data` from `source` to `destination`.
    ///
    /// # Errors
    ///
    /// [`J1939Error::TooLong`] for fewer than 9 bytes, more than
    /// [`TP_MAX_BYTES`] or more than `B`.
    pub fn new(pgn: Pgn, source: u8, destination: u8, data: &[u8]) -> Result<Self, J1939Error> {
        let packets = packets_of(pgn, data.len())?;
        let mut bytes = [0xFF; B];
        let slots = bytes.get_mut(..data.len()).ok_or(J1939Error::TooLong {
            pgn,
            size: u32::try_from(data.len()).unwrap_or(u32::MAX),
            limit: u32::try_from(B).unwrap_or(u32::MAX),
        })?;
        slots.copy_from_slice(data);
        Ok(Self {
            pgn,
            source,
            destination,
            len: data.len(),
            packets,
            bytes,
            state: SenderState::Idle,
            since: Instant::UNIX_EPOCH,
        })
    }

    /// State.
    #[must_use]
    pub const fn state(&self) -> SenderState {
        self.state
    }

    /// Starts: the request to send, taking any number of packets per
    /// clear-to-send.
    #[must_use]
    pub fn start(&mut self, at: Instant<Utc>) -> Frame {
        self.state = SenderState::Waiting;
        self.since = at;
        let [low, high] = u16::try_from(self.len).unwrap_or(u16::MAX).to_le_bytes();
        let [p0, p1, p2] = pgn_bytes(self.pgn);
        frame(
            self.source,
            self.destination,
            [RTS, low, high, self.packets, 0xFF, p0, p1, p2],
        )
    }

    /// Takes in a frame from the bus; the packets a clear-to-send asks for go
    /// to `send`.
    pub fn handle(&mut self, frame: &Frame, at: Instant<Utc>, mut send: impl FnMut(Frame)) {
        let id = frame.id;
        let ours = id.pgn() == Pgn::TP_CONNECTION
            && id.source() == self.destination
            && id.destination() == Some(self.source);
        let Ok(bytes) = <[u8; 8]>::try_from(frame.data()) else {
            return;
        };
        let [control, count, next, _, _, p0, p1, p2] = bytes;
        if !ours || Pgn::from_bytes([p0, p1, p2]) != self.pgn {
            return;
        }
        if matches!(
            self.state,
            SenderState::Done | SenderState::Aborted(_) | SenderState::Idle
        ) {
            return;
        }
        match control {
            CTS if count == 0 => {
                self.state = SenderState::Holding;
                self.since = at;
            }
            CTS => {
                let first = usize::from(next.max(1));
                let last = first
                    .saturating_add(usize::from(count))
                    .saturating_sub(1)
                    .min(usize::from(self.packets));
                for sequence in first..=last {
                    let start = sequence.saturating_sub(1).saturating_mul(7);
                    let end = start.saturating_add(7).min(self.len);
                    let chunk = self.bytes.get(start..end).unwrap_or(&[]);
                    send(data_packet(
                        self.source,
                        self.destination,
                        sequence.saturating_sub(1),
                        chunk,
                    ));
                }
                self.state = SenderState::Waiting;
                self.since = at;
            }
            END_OF_MESSAGE => self.state = SenderState::Done,
            ABORT => self.state = SenderState::Aborted(AbortReason::from_code(count)),
            _ => {}
        }
    }

    /// Gives up after T3 without an answer or T4 on hold; the abort goes to
    /// `send`.
    pub fn poll(&mut self, at: Instant<Utc>, mut send: impl FnMut(Frame)) {
        let limit = match self.state {
            SenderState::Waiting => T3,
            SenderState::Holding => T4,
            _ => return,
        };
        let expired = at
            .checked_duration_since(self.since)
            .is_some_and(|waited| waited > limit);
        if expired {
            self.state = SenderState::Aborted(AbortReason::Timeout);
            send(abort(
                self.source,
                self.destination,
                self.pgn,
                AbortReason::Timeout,
            ));
        }
    }
}
