//! Extended transport protocol (ETP, ISO 11783-3): transfers of 1786 bytes
//! up to 117 440 505 to one node, in windows placed by a data packet offset.
//!
//! Reception as a participant or a silent observer, and the sender of one
//! transfer; the frames to send come out, timing is the caller's (the T1 to
//! T4 of the transport protocol apply).

use kinavis_kernel::time::{Instant, Utc};

use crate::error::J1939Error;
use crate::frame::Frame;
use crate::id::{CanId, Pgn};
use crate::transport::{AbortReason, Received, SenderState, Transfer, T1, T2, T3, T4};

/// Longest extended transfer, bytes: 2²⁴ − 1 packets of 7.
pub const ETP_MAX_BYTES: u32 = 117_440_505;

/// Priority of extended transport frames.
const PRIORITY: u8 = 7;

const RTS: u8 = 20;
const CTS: u8 = 21;
const DPO: u8 = 22;
const END_OF_MESSAGE: u8 = 23;
const ABORT: u8 = 255;

/// Abort code of a clear-to-send asking for packets past the end of the
/// transfer, ISO 11783-3.
const CTS_PAST_END: u8 = 15;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Session<const B: usize> {
    participant: bool,
    pgn: Pgn,
    source: u8,
    destination: u8,
    size: usize,
    /// Packets of the transfer.
    packets: u32,
    /// Packets before the current window.
    offset: u32,
    /// Packets in the current window, announced by the offset message.
    window: u8,
    /// Sequence number within the window expected next, from 1.
    next: u8,
    /// Whether the last thing done was granting a window.
    after_cts: bool,
    last: Instant<Utc>,
    bytes: [u8; B],
}

/// Receiver of extended transfers of up to `B` bytes, `S` at a time, for a
/// node at a local address or, without one, a monitor of every connection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExtendedReceiver<const S: usize, const B: usize> {
    local: Option<u8>,
    window: u8,
    sessions: [Option<Session<B>>; S],
}

impl<const S: usize, const B: usize> ExtendedReceiver<S, B> {
    /// Receiver for a node at `local`, or a monitor for `None`, granting up
    /// to 255 packets per clear-to-send.
    #[must_use]
    pub const fn new(local: Option<u8>) -> Self {
        Self {
            local,
            window: u8::MAX,
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
            Pgn::ETP_CONNECTION => self.connection(frame, at),
            Pgn::ETP_DATA => self.data(frame, at),
            _ => nothing(),
        }
    }

    /// Drops sessions silent beyond their timeout; the aborts owed go to
    /// `send`. Returns how many were dropped.
    pub fn expire(&mut self, at: Instant<Utc>, mut send: impl FnMut(Frame)) -> usize {
        let mut dropped = 0usize;
        for slot in &mut self.sessions {
            let Some(session) = slot else {
                continue;
            };
            let limit = if session.after_cts { T2 } else { T1 };
            let silent = at
                .checked_duration_since(session.last)
                .is_some_and(|waited| waited > limit);
            if !silent {
                continue;
            }
            if session.participant {
                send(abort(
                    session.destination,
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
            return nothing();
        };
        let [control, b1, b2, b3, b4, p0, p1, p2] = bytes;
        let pgn = Pgn::from_bytes([p0, p1, p2]);
        let source = frame.id.source();
        let Some(destination) = frame.id.destination() else {
            return nothing();
        };
        match control {
            RTS => {
                let participant = Some(destination) == self.local;
                if !participant && self.local.is_some() {
                    return nothing();
                }
                self.remove(source, destination);
                let size = u32::from_le_bytes([b1, b2, b3, b4]);
                let refusal = if !(1786..=ETP_MAX_BYTES).contains(&size) {
                    Some(AbortReason::Other(250))
                } else if usize::try_from(size).map_or(true, |size| size > B) {
                    Some(AbortReason::ResourcesNeeded)
                } else {
                    None
                };
                if let Some(reason) = refusal {
                    return Received {
                        reply: participant.then(|| abort(destination, source, pgn, reason)),
                        dropped: Some(reason),
                        transfer: None,
                    };
                }
                let Some(slot) = self.sessions.iter_mut().find(|slot| slot.is_none()) else {
                    return Received {
                        reply: participant
                            .then(|| abort(destination, source, pgn, AbortReason::ResourcesNeeded)),
                        dropped: Some(AbortReason::ResourcesNeeded),
                        transfer: None,
                    };
                };
                let packets = size.div_ceil(7);
                *slot = Some(Session {
                    participant,
                    pgn,
                    source,
                    destination,
                    size: usize::try_from(size).unwrap_or(0),
                    packets,
                    offset: 0,
                    window: 0,
                    next: 1,
                    after_cts: true,
                    last: at,
                    bytes: [0; B],
                });
                let granted = grant(self.window, packets, 0);
                Received {
                    reply: participant.then(|| clear_to_send(destination, source, pgn, granted, 1)),
                    ..nothing()
                }
            }
            DPO => {
                if let Some(session) = self.find(source, destination) {
                    session.window = b1;
                    session.offset = u32::from_le_bytes([b2, b3, b4, 0]);
                    session.next = 1;
                    session.after_cts = false;
                    session.last = at;
                }
                nothing()
            }
            ABORT => {
                let removed = self.remove(source, destination) || self.remove(destination, source);
                Received {
                    dropped: removed.then_some(AbortReason::from_code(b1)),
                    ..nothing()
                }
            }
            // A clear-to-send or acknowledgement between others changes nothing
            // the data packets do not show.
            _ => nothing(),
        }
    }

    fn data(&mut self, frame: &Frame, at: Instant<Utc>) -> Received<B> {
        let source = frame.id.source();
        let Some(destination) = frame.id.destination() else {
            return nothing();
        };
        let local_window = self.window;
        let Some(index) = self.sessions.iter().position(|slot| {
            slot.is_some_and(|s| s.source == source && s.destination == destination)
        }) else {
            return nothing();
        };
        let Some(Some(session)) = self.sessions.get_mut(index) else {
            return nothing();
        };
        let Some((&sequence, chunk)) = frame.data().split_first() else {
            return nothing();
        };
        if sequence != session.next || sequence > session.window {
            let reason = if sequence < session.next {
                AbortReason::DuplicateSequence
            } else {
                AbortReason::BadSequence
            };
            let reply = session
                .participant
                .then(|| abort(session.destination, session.source, session.pgn, reason));
            if let Some(slot) = self.sessions.get_mut(index) {
                *slot = None;
            }
            return Received {
                reply,
                dropped: Some(reason),
                transfer: None,
            };
        }
        let packet = session.offset.saturating_add(u32::from(sequence));
        let start = usize::try_from(packet.saturating_sub(1))
            .unwrap_or(usize::MAX)
            .saturating_mul(7);
        let end = start.saturating_add(7).min(session.size);
        if let (Some(slots), Some(chunk)) = (
            session.bytes.get_mut(start..end),
            chunk.get(..end.saturating_sub(start)),
        ) {
            slots.copy_from_slice(chunk);
        }
        session.next = sequence.saturating_add(1);
        session.last = at;
        if packet >= session.packets {
            let done = *session;
            if let Some(slot) = self.sessions.get_mut(index) {
                *slot = None;
            }
            return Received {
                transfer: Some(Transfer::new(
                    done.pgn,
                    done.source,
                    Some(done.destination),
                    done.size,
                    done.bytes,
                )),
                reply: done
                    .participant
                    .then(|| end_of_message(done.destination, done.source, done.pgn, done.size)),
                dropped: None,
            };
        }
        if sequence == session.window && session.participant {
            let granted = grant(local_window, session.packets, packet);
            session.after_cts = true;
            let next = packet.saturating_add(1);
            return Received {
                reply: Some(clear_to_send(
                    session.destination,
                    session.source,
                    session.pgn,
                    granted,
                    next,
                )),
                ..nothing()
            };
        }
        nothing()
    }

    fn find(&mut self, source: u8, destination: u8) -> Option<&mut Session<B>> {
        self.sessions
            .iter_mut()
            .flatten()
            .find(|session| session.source == source && session.destination == destination)
    }

    fn remove(&mut self, source: u8, destination: u8) -> bool {
        let Some(slot) = self
            .sessions
            .iter_mut()
            .find(|slot| slot.is_some_and(|s| s.source == source && s.destination == destination))
        else {
            return false;
        };
        *slot = None;
        true
    }
}

const fn nothing<const B: usize>() -> Received<B> {
    Received {
        transfer: None,
        reply: None,
        dropped: None,
    }
}

/// Packets to grant: the window, no more than remain.
fn grant(window: u8, packets: u32, sent: u32) -> u8 {
    let remaining = packets.saturating_sub(sent);
    u8::try_from(remaining.min(u32::from(window))).unwrap_or(window)
}

fn frame(source: u8, destination: u8, data: [u8; 8]) -> Frame {
    Frame::truncated(
        CanId::from_parts(PRIORITY, Pgn::ETP_CONNECTION, destination, source),
        &data,
    )
}

fn clear_to_send(source: u8, destination: u8, pgn: Pgn, packets: u8, next: u32) -> Frame {
    let [n0, n1, n2, _] = next.to_le_bytes();
    let [p0, p1, p2] = pgn.to_bytes();
    frame(source, destination, [CTS, packets, n0, n1, n2, p0, p1, p2])
}

fn end_of_message(source: u8, destination: u8, pgn: Pgn, size: usize) -> Frame {
    let [s0, s1, s2, s3] = u32::try_from(size).unwrap_or(u32::MAX).to_le_bytes();
    let [p0, p1, p2] = pgn.to_bytes();
    frame(
        source,
        destination,
        [END_OF_MESSAGE, s0, s1, s2, s3, p0, p1, p2],
    )
}

/// Sender of one extended transfer from `source` to `destination`, the data
/// borrowed.
///
/// Each clear-to-send is answered with the offset message and the packets it
/// grants, at most 255 in one call of [`ExtendedSender::handle`]; the pace at
/// which they reach the bus is the caller's.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExtendedSender<'a> {
    pgn: Pgn,
    source: u8,
    destination: u8,
    data: &'a [u8],
    packets: u32,
    state: SenderState,
    since: Instant<Utc>,
}

impl<'a> ExtendedSender<'a> {
    /// Sender of `data` from `source` to `destination`.
    ///
    /// # Errors
    ///
    /// [`J1939Error::TooLong`] for fewer than 1786 bytes (the transport
    /// protocol carries them) or more than [`ETP_MAX_BYTES`].
    pub fn new(pgn: Pgn, source: u8, destination: u8, data: &'a [u8]) -> Result<Self, J1939Error> {
        let size = u32::try_from(data.len()).unwrap_or(u32::MAX);
        if !(1786..=ETP_MAX_BYTES).contains(&size) {
            return Err(J1939Error::TooLong {
                pgn,
                size,
                limit: ETP_MAX_BYTES,
            });
        }
        Ok(Self {
            pgn,
            source,
            destination,
            data,
            packets: size.div_ceil(7),
            state: SenderState::Idle,
            since: Instant::UNIX_EPOCH,
        })
    }

    /// State.
    #[must_use]
    pub const fn state(&self) -> SenderState {
        self.state
    }

    /// Starts: the request to send.
    #[must_use]
    pub fn start(&mut self, at: Instant<Utc>) -> Frame {
        self.state = SenderState::Waiting;
        self.since = at;
        let [s0, s1, s2, s3] = u32::try_from(self.data.len())
            .unwrap_or(u32::MAX)
            .to_le_bytes();
        let [p0, p1, p2] = self.pgn.to_bytes();
        frame(
            self.source,
            self.destination,
            [RTS, s0, s1, s2, s3, p0, p1, p2],
        )
    }

    /// Takes in a frame from the bus; the offset message and the packets a
    /// clear-to-send asks for, or the abort it calls for, go to `send`.
    ///
    /// A clear-to-send may ask for any packets again, also after the last
    /// one; one starting past the end aborts the transfer.
    pub fn handle(&mut self, frame: &Frame, at: Instant<Utc>, mut send: impl FnMut(Frame)) {
        let id = frame.id;
        let ours = id.pgn() == Pgn::ETP_CONNECTION
            && id.source() == self.destination
            && id.destination() == Some(self.source);
        let Ok(bytes) = <[u8; 8]>::try_from(frame.data()) else {
            return;
        };
        let [control, count, n0, n1, n2, p0, p1, p2] = bytes;
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
                let next = u32::from_le_bytes([n0, n1, n2, 0]);
                if next == 0 || next > self.packets {
                    let reason = AbortReason::from_code(CTS_PAST_END);
                    self.state = SenderState::Aborted(reason);
                    send(abort(self.source, self.destination, self.pgn, reason));
                    return;
                }
                let offset = next.saturating_sub(1);
                let window = grant(count, self.packets, offset);
                send(data_packet_offset(
                    self.source,
                    self.destination,
                    self.pgn,
                    window,
                    offset,
                ));
                for sequence in 1..=window {
                    let packet = offset.saturating_add(u32::from(sequence));
                    let start = usize::try_from(packet.saturating_sub(1))
                        .unwrap_or(usize::MAX)
                        .saturating_mul(7);
                    let end = start.saturating_add(7).min(self.data.len());
                    let chunk = self.data.get(start..end).unwrap_or(&[]);
                    send(data_packet(self.source, self.destination, sequence, chunk));
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

fn data_packet_offset(source: u8, destination: u8, pgn: Pgn, packets: u8, offset: u32) -> Frame {
    let [o0, o1, o2, _] = offset.to_le_bytes();
    let [p0, p1, p2] = pgn.to_bytes();
    frame(source, destination, [DPO, packets, o0, o1, o2, p0, p1, p2])
}

fn data_packet(source: u8, destination: u8, sequence: u8, chunk: &[u8]) -> Frame {
    let mut bytes = [0xFF; 8];
    if let Some((first, rest)) = bytes.split_first_mut() {
        *first = sequence;
        for (slot, byte) in rest.iter_mut().zip(chunk) {
            *slot = *byte;
        }
    }
    Frame::truncated(
        CanId::from_parts(PRIORITY, Pgn::ETP_DATA, destination, source),
        &bytes,
    )
}

fn abort(source: u8, destination: u8, pgn: Pgn, reason: AbortReason) -> Frame {
    let [p0, p1, p2] = pgn.to_bytes();
    frame(
        source,
        destination,
        [ABORT, reason.code(), 0xFF, 0xFF, 0xFF, p0, p1, p2],
    )
}
