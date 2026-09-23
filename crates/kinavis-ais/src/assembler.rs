//! Multi-sentence message reassembly.
//!
//! A message longer than one sentence is split into fragments carrying the
//! fragment count, fragment number and a shared sequence identifier. Reassembly
//! uses a fixed number of slots; an incomplete message is dropped on timeout so
//! a lost fragment cannot hold a slot indefinitely.

use core::time::Duration;

use kinavis_kernel::time::{Instant, Utc};
use kinavis_nmea0183::{Channel, Vdm};

use crate::bits::Bits;
use crate::error::AisError;

/// Default number of concurrent assemblies.
///
/// The sequence identifier is one digit and a receiver has two channels; four
/// interleaved messages covers practice. [`Assembler`] takes the count as a
/// parameter. With every slot busy, the oldest assembly is evicted, so a flood
/// of first fragments cannot starve messages that complete.
pub const MAX_ASSEMBLIES: usize = 4;

/// Default timeout for an incomplete message.
pub const DEFAULT_TIMEOUT: Duration = Duration::from_secs(10);

/// Assembly key.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Key {
    own: bool,
    sequence: Option<u8>,
    channel: Option<Channel>,
}

impl Key {
    const fn of(vdm: &Vdm) -> Self {
        Self {
            own: vdm.own,
            sequence: vdm.sequence,
            channel: vdm.channel,
        }
    }
}

/// Message being assembled.
#[derive(Debug, Clone, Copy)]
struct Assembly {
    key: Key,
    fragments: u8,
    /// Next expected fragment number.
    next: u8,
    started: Instant<Utc>,
    bits: Bits,
}

/// Fragment assembler with `N` slots.
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
    /// `N` slots with the given timeout, for feeds with more than
    /// [`MAX_ASSEMBLIES`] interleaved messages.
    #[must_use]
    pub const fn with_slots(timeout: Duration) -> Self {
        Self {
            slots: [None; N],
            timeout,
        }
    }

    /// Accepts one sentence; returns the message once complete.
    ///
    /// A single-fragment message is returned immediately. A first fragment
    /// starts an assembly, replacing any incomplete one with the same sequence
    /// and channel; subsequent fragments are appended in order and the last
    /// completes it. `now` is the receiver clock; assemblies older than the
    /// timeout are dropped before the sentence is processed.
    ///
    /// # Errors
    ///
    /// [`AisError::UnexpectedFragment`] for a fragment out of order, repeated
    /// or without a first fragment (the assembly is dropped); errors of
    /// [`Bits::append`] for the payload. A first fragment with every slot busy
    /// evicts the oldest assembly; [`AisError::NoSlot`] only for an assembler
    /// with zero slots.
    pub fn push(&mut self, vdm: &Vdm, now: Instant<Utc>) -> Result<Option<Bits>, AisError> {
        self.expire(now);
        if vdm.is_whole() {
            return Bits::unarmour(vdm.payload.as_bytes(), vdm.fill_bits).map(Some);
        }
        let key = Key::of(vdm);
        let slot = self
            .slots
            .iter_mut()
            .find(|slot| slot.is_some_and(|assembly| assembly.key == key));

        if vdm.fragment == 1 {
            let mut assembly = Assembly {
                key,
                fragments: vdm.fragments,
                next: 2,
                started: now,
                bits: Bits::new(),
            };
            assembly
                .bits
                .append(vdm.payload.as_bytes(), vdm.fill_bits)?;
            let slot = match slot {
                Some(slot) => slot,
                None => self.free_or_oldest_slot().ok_or(AisError::NoSlot)?,
            };
            *slot = Some(assembly);
            return Ok(None);
        }

        let Some(slot) = slot else {
            return Err(AisError::UnexpectedFragment {
                expected: 1,
                found: vdm.fragment,
            });
        };
        let Some(assembly) = slot.as_mut() else {
            // Slot found by key, so it is occupied.
            return Ok(None);
        };
        if vdm.fragment != assembly.next || vdm.fragments != assembly.fragments {
            let expected = assembly.next;
            *slot = None;
            return Err(AisError::UnexpectedFragment {
                expected,
                found: vdm.fragment,
            });
        }
        if let Err(error) = assembly.bits.append(vdm.payload.as_bytes(), vdm.fill_bits) {
            *slot = None;
            return Err(error);
        }
        if vdm.fragment == assembly.fragments {
            let bits = assembly.bits;
            *slot = None;
            return Ok(Some(bits));
        }
        assembly.next = vdm.fragment.saturating_add(1);
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
                // `Option::is_none_or` requires a newer Rust than the MSRV.
                now.checked_duration_since(assembly.started)
                    .map_or(true, |age| age > self.timeout)
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
