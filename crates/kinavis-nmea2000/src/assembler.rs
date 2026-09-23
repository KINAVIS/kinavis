//! Fast-packet reassembly.
//!
//! A group longer than 8 bytes is sent as a *fast packet*: the first frame
//! carries a sequence counter, the length and 6 bytes; each subsequent frame
//! the counter, a frame number and 7 bytes; up to 32 frames. Reassembly uses a
//! fixed number of slots keyed by (PGN, source); an incomplete packet is
//! dropped on timeout.

use core::time::Duration;

use kinavis_kernel::time::{Instant, Utc};

use crate::error::Nmea2000Error;
use crate::frame::{Frame, Payload, MAX_PAYLOAD_BYTES};
use crate::id::{Pgn, Transport};

/// Default number of concurrent fast-packet assemblies.
///
/// A GNSS and an AIS receiver each send one group at a time; a busy bus has a
/// few of each. [`Assembler`] takes the count as a parameter for larger buses.
/// With every slot busy, the oldest assembly is evicted, so a source that only
/// ever sends first frames cannot starve packets that complete.
pub const MAX_ASSEMBLIES: usize = 8;

/// Default timeout for an incomplete packet. Frames of one packet follow within
/// milliseconds; the standard specifies 750 ms.
pub const DEFAULT_TIMEOUT: Duration = Duration::from_millis(750);

/// Assembly key.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Key {
    pgn: Pgn,
    source: u8,
}

/// Packet being assembled.
#[derive(Debug, Clone, Copy)]
struct Assembly {
    key: Key,
    sequence: u8,
    /// Next expected frame counter.
    next: u8,
    declared: usize,
    started: Instant<Utc>,
    payload: Payload,
}

/// Fast-packet assembler with `N` slots.
#[derive(Debug, Clone)]
pub struct Assembler<const N: usize = MAX_ASSEMBLIES> {
    slots: [Option<Assembly>; N],
    timeout: Duration,
}

impl Default for Assembler<MAX_ASSEMBLIES> {
    fn default() -> Self {
        Self::new()
    }
}

impl Assembler<MAX_ASSEMBLIES> {
    /// [`MAX_ASSEMBLIES`] slots, [`DEFAULT_TIMEOUT`].
    #[must_use]
    pub const fn new() -> Self {
        Self::with_slots(DEFAULT_TIMEOUT)
    }

    /// [`MAX_ASSEMBLIES`] slots with the given timeout.
    #[must_use]
    pub const fn with_timeout(timeout: Duration) -> Self {
        Self::with_slots(timeout)
    }
}

impl<const N: usize> Assembler<N> {
    /// `N` slots with the given timeout, for buses with more than
    /// [`MAX_ASSEMBLIES`] concurrent fast-packet sources.
    #[must_use]
    pub const fn with_slots(timeout: Duration) -> Self {
        Self {
            slots: [None; N],
            timeout,
        }
    }

    /// Accepts one frame; returns the group once complete.
    ///
    /// A single-frame group is returned immediately. A fast-packet first frame
    /// starts an assembly, replacing any incomplete one for the same (PGN,
    /// source); subsequent frames are appended in order and the one reaching
    /// the declared length completes it. `now` is the receiver clock;
    /// assemblies older than the timeout are dropped before the frame is
    /// processed.
    ///
    /// # Errors
    ///
    /// [`Nmea2000Error::UnknownPgn`] if the transport of the PGN is unknown
    /// (see [`Assembler::push_as`]); otherwise as [`Assembler::push_as`].
    pub fn push(
        &mut self,
        frame: &Frame,
        now: Instant<Utc>,
    ) -> Result<Option<Payload>, Nmea2000Error> {
        let pgn = frame.id.pgn();
        let transport = pgn.transport().ok_or(Nmea2000Error::UnknownPgn { pgn })?;
        self.push_as(frame, transport, now)
    }

    /// Accepts one frame with an explicit transport.
    ///
    /// # Errors
    ///
    /// [`Nmea2000Error::UnexpectedFrame`] for a fast-packet frame that is out
    /// of order, repeated, of another sequence, or has no first frame (the
    /// assembly is dropped); [`Nmea2000Error::BadLength`] if a first frame
    /// declares more than a fast packet can carry. A first frame with every
    /// slot busy evicts the oldest assembly; [`Nmea2000Error::NoSlot`] only for
    /// an assembler with zero slots.
    pub fn push_as(
        &mut self,
        frame: &Frame,
        transport: Transport,
        now: Instant<Utc>,
    ) -> Result<Option<Payload>, Nmea2000Error> {
        self.expire(now);
        let key = Key {
            pgn: frame.id.pgn(),
            source: frame.id.source(),
        };
        match transport {
            Transport::SingleFrame => {
                Payload::from_bytes(key.pgn, key.source, frame.data()).map(Some)
            }
            Transport::FastPacket => self.push_fast(frame, key, now),
        }
    }

    fn push_fast(
        &mut self,
        frame: &Frame,
        key: Key,
        now: Instant<Utc>,
    ) -> Result<Option<Payload>, Nmea2000Error> {
        let data = frame.data();
        let Some(&counter) = data.first() else {
            return Err(Nmea2000Error::UnexpectedFrame {
                expected: 0,
                found: 0,
            });
        };
        let sequence = counter >> 5;
        let number = counter & 0x1F;
        let slot = self
            .slots
            .iter_mut()
            .find(|slot| slot.is_some_and(|assembly| assembly.key == key));

        if number == 0 {
            let declared = usize::from(data.get(1).copied().unwrap_or(0));
            if declared > MAX_PAYLOAD_BYTES {
                return Err(Nmea2000Error::BadLength {
                    declared: data.get(1).copied().unwrap_or(0),
                    limit: MAX_PAYLOAD_BYTES,
                });
            }
            let mut assembly = Assembly {
                key,
                sequence,
                next: 1,
                declared,
                started: now,
                payload: Payload::new(key.pgn, key.source),
            };
            assembly.payload.append(data.get(2..).unwrap_or(&[]))?;
            if assembly.payload.len() >= declared {
                assembly.payload.truncate(declared);
                if let Some(slot) = slot {
                    *slot = None;
                }
                return Ok(Some(assembly.payload));
            }
            let slot = match slot {
                Some(slot) => slot,
                None => self.free_or_oldest_slot().ok_or(Nmea2000Error::NoSlot)?,
            };
            *slot = Some(assembly);
            return Ok(None);
        }

        let Some(slot) = slot else {
            return Err(Nmea2000Error::UnexpectedFrame {
                expected: 0,
                found: number,
            });
        };
        let Some(assembly) = slot.as_mut() else {
            // Slot found by key, so it is occupied.
            return Ok(None);
        };
        if number != assembly.next || sequence != assembly.sequence {
            let expected = assembly.next;
            *slot = None;
            return Err(Nmea2000Error::UnexpectedFrame {
                expected,
                found: number,
            });
        }
        if let Err(error) = assembly.payload.append(data.get(1..).unwrap_or(&[])) {
            *slot = None;
            return Err(error);
        }
        if assembly.payload.len() >= assembly.declared {
            let mut payload = assembly.payload;
            payload.truncate(assembly.declared);
            *slot = None;
            return Ok(Some(payload));
        }
        assembly.next = number.saturating_add(1);
        Ok(None)
    }

    /// A free slot, or else the oldest assembly (evicted). `None` only for zero
    /// slots.
    fn free_or_oldest_slot(&mut self) -> Option<&mut Option<Assembly>> {
        let index = self.slots.iter().position(Option::is_none).or_else(|| {
            self.slots
                .iter()
                .enumerate()
                .min_by_key(|(_, slot)| slot.map(|assembly| assembly.started))
                .map(|(index, _)| index)
        })?;
        self.slots.get_mut(index)
    }

    /// Drops assemblies older than the timeout at `now`, and any started after
    /// `now` (clock stepped back).
    pub fn expire(&mut self, now: Instant<Utc>) {
        for slot in &mut self.slots {
            let stale = slot.is_some_and(|assembly| {
                now.checked_duration_since(assembly.started)
                    .is_none_or(|age| age > self.timeout)
            });
            if stale {
                *slot = None;
            }
        }
    }

    /// Number of assemblies in progress.
    #[must_use]
    pub fn pending(&self) -> usize {
        self.slots.iter().filter(|slot| slot.is_some()).count()
    }

    /// Assembly timeout.
    #[must_use]
    pub const fn timeout(&self) -> Duration {
        self.timeout
    }
}
