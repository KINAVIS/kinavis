//! Extended transport protocol (ETP, ISO 11783-3): transfers of 1786 bytes
//! up to 117 440 505 to one node, in windows placed by a data packet offset.
//!
//! Reception only, as a participant or a silent observer; the frames to send
//! come out, timing is the caller's (the T1 and T2 of the transport
//! protocol apply).

use kinavis_kernel::time::{Instant, Utc};

use crate::frame::Frame;
use crate::id::{CanId, Pgn};
use crate::transport::{AbortReason, Received, Transfer, T1, T2};

/// Longest extended transfer, bytes: 2²⁴ − 1 packets of 7.
pub const ETP_MAX_BYTES: u32 = 117_440_505;

/// Priority of extended transport frames.
const PRIORITY: u8 = 7;

const RTS: u8 = 20;
const CTS: u8 = 21;
const DPO: u8 = 22;
const END_OF_MESSAGE: u8 = 23;
const ABORT: u8 = 255;

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

fn abort(source: u8, destination: u8, pgn: Pgn, reason: AbortReason) -> Frame {
    let [p0, p1, p2] = pgn.to_bytes();
    frame(
        source,
        destination,
        [ABORT, reason.code(), 0xFF, 0xFF, 0xFF, p0, p1, p2],
    )
}
