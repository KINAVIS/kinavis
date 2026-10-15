//! Address claim (ISO 11783-5, J1939-81) as a state machine: frames in,
//! frames to send out. Sending them, and when, is the caller's.

use core::time::Duration;

use kinavis_kernel::time::{Instant, Utc};

use crate::error::J1939Error;
use crate::frame::Frame;
use crate::id::{CanId, Pgn};
use crate::name::Name;

/// How long claims are collected before an address is chosen, and how long a
/// claim stands unchallenged before it holds.
pub const CONTENTION_TIME: Duration = Duration::from_millis(250);

/// First address a self-configurable node may take for itself.
pub const FIRST_DYNAMIC_ADDRESS: u8 = 128;

/// Priority of address claim and request frames.
const PRIORITY: u8 = 6;

/// Where a claimer stands.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClaimState {
    /// Not started.
    Idle,
    /// The request for claims is out; claims are being collected.
    Listening {
        /// When the request went out.
        since: Instant<Utc>,
    },
    /// A claim is out and not yet held.
    Claiming {
        /// Address claimed.
        address: u8,
        /// When the claim went out.
        since: Instant<Utc>,
    },
    /// The address is held: the node may send from it.
    Claimed {
        /// Address.
        address: u8,
    },
    /// No address could be had; the node sends nothing but the cannot-claim.
    CannotClaim,
}

/// Address claim of one control function.
///
/// [`AddressClaimer::start`] sends a request for claims; claims seen in the
/// next [`CONTENTION_TIME`] fill the table of taken addresses; then
/// [`AddressClaimer::poll`] claims the preferred address if it is free or held
/// by a lower-priority NAME, else, for a self-configurable NAME, the first
/// free address from [`FIRST_DYNAMIC_ADDRESS`] to the end of the industry
/// group's range (235 for agriculture, 158 on highway, 207 for construction,
/// marine and industry, 247 otherwise), else gives up with a cannot-claim. A
/// claim unchallenged for [`CONTENTION_TIME`] holds. A conflicting claim
/// later is settled by NAME: the lower keeps the address.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AddressClaimer {
    name: Name,
    preferred: u8,
    state: ClaimState,
    /// Addresses claimed by other NAMEs, one bit each.
    taken: [u64; 4],
    /// NAME holding the preferred address, if seen.
    holder: Option<Name>,
}

impl AddressClaimer {
    /// Claimer for a NAME wanting an address.
    ///
    /// # Errors
    ///
    /// [`J1939Error::BadControl`] for a preferred address of 254 or 255,
    /// which no node may hold.
    pub const fn new(name: Name, preferred: u8) -> Result<Self, J1939Error> {
        if preferred >= CanId::NULL_ADDRESS {
            return Err(J1939Error::BadControl { control: preferred });
        }
        Ok(Self {
            name,
            preferred,
            state: ClaimState::Idle,
            taken: [0; 4],
            holder: None,
        })
    }

    /// NAME.
    #[must_use]
    pub const fn name(&self) -> Name {
        self.name
    }

    /// State.
    #[must_use]
    pub const fn state(&self) -> ClaimState {
        self.state
    }

    /// Address the node may send from, once held.
    #[must_use]
    pub const fn address(&self) -> Option<u8> {
        match self.state {
            ClaimState::Claimed { address } => Some(address),
            _ => None,
        }
    }

    /// Starts: the request for every node's claim, to send now.
    #[must_use]
    pub fn start(&mut self, at: Instant<Utc>) -> Frame {
        self.state = ClaimState::Listening { since: at };
        request(Pgn::ADDRESS_CLAIMED, CanId::NULL_ADDRESS)
    }

    /// Moves on with the clock: chooses and claims an address after the
    /// listening time, holds a claim after the contention time. Returns the
    /// frame to send, if any.
    #[must_use]
    pub fn poll(&mut self, at: Instant<Utc>) -> Option<Frame> {
        match self.state {
            ClaimState::Listening { since } if elapsed(since, at) => Some(self.choose(at)),
            ClaimState::Claiming { address, since } if elapsed(since, at) => {
                self.state = ClaimState::Claimed { address };
                None
            }
            _ => None,
        }
    }

    /// Takes in a frame from the bus. Returns the frame to send in answer, if
    /// any: our claim again after a request or a conflict we win, a claim of
    /// another address or a cannot-claim after one we lose.
    #[must_use]
    pub fn handle(&mut self, frame: &Frame, at: Instant<Utc>) -> Option<Frame> {
        let id = frame.id;
        match id.pgn() {
            Pgn::REQUEST => {
                let mut requested = [0u8; 3];
                for (slot, byte) in requested.iter_mut().zip(frame.data()) {
                    *slot = *byte;
                }
                if frame.data().len() < 3 || Pgn::from_bytes(requested) != Pgn::ADDRESS_CLAIMED {
                    return None;
                }
                let to_us = match id.destination() {
                    Some(CanId::GLOBAL_ADDRESS) | None => true,
                    Some(destination) => Some(destination) == self.current(),
                };
                if !to_us {
                    return None;
                }
                match self.state {
                    ClaimState::Claiming { address, .. } | ClaimState::Claimed { address } => {
                        Some(self.claim_frame(address))
                    }
                    ClaimState::CannotClaim => Some(self.claim_frame(CanId::NULL_ADDRESS)),
                    _ => None,
                }
            }
            Pgn::ADDRESS_CLAIMED => {
                let bytes: [u8; 8] = frame.data().try_into().ok()?;
                let other = Name::from_bytes(bytes);
                let source = id.source();
                if other == self.name || source >= CanId::NULL_ADDRESS {
                    return None;
                }
                self.mark(source);
                if source == self.preferred && self.holder.is_none_or(|held| other < held) {
                    self.holder = Some(other);
                }
                let current = self.current()?;
                if source != current {
                    return None;
                }
                if self.name < other {
                    // Ours is the higher priority: the address stays ours.
                    return Some(self.claim_frame(current));
                }
                // Lost: find another, or give up.
                Some(self.choose_after_loss(at))
            }
            _ => None,
        }
    }

    fn current(&self) -> Option<u8> {
        match self.state {
            ClaimState::Claiming { address, .. } | ClaimState::Claimed { address } => Some(address),
            _ => None,
        }
    }

    fn choose(&mut self, at: Instant<Utc>) -> Frame {
        let preferred_free = self.holder.is_none_or(|held| self.name < held);
        if preferred_free {
            return self.claim(self.preferred, at);
        }
        self.choose_after_loss(at)
    }

    fn choose_after_loss(&mut self, at: Instant<Utc>) -> Frame {
        if self.name.is_self_configurable() {
            let last = last_dynamic_address(self.name.fields().industry_group);
            let current = self.current();
            for address in FIRST_DYNAMIC_ADDRESS..=last {
                if !self.is_taken(address) && Some(address) != current {
                    return self.claim(address, at);
                }
            }
        }
        self.state = ClaimState::CannotClaim;
        self.claim_frame(CanId::NULL_ADDRESS)
    }

    fn claim(&mut self, address: u8, at: Instant<Utc>) -> Frame {
        self.state = ClaimState::Claiming { address, since: at };
        self.claim_frame(address)
    }

    fn claim_frame(&self, source: u8) -> Frame {
        let id = CanId::from_parts(
            PRIORITY,
            Pgn::ADDRESS_CLAIMED,
            CanId::GLOBAL_ADDRESS,
            source,
        );
        Frame::truncated(id, &self.name.to_bytes())
    }

    fn mark(&mut self, address: u8) {
        if let Some(word) = self.taken.get_mut(usize::from(address / 64)) {
            *word |= 1 << (address % 64);
        }
    }

    fn is_taken(&self, address: u8) -> bool {
        self.taken
            .get(usize::from(address / 64))
            .is_some_and(|word| word >> (address % 64) & 1 != 0)
    }
}

/// Request for a parameter group, to every node, from a source.
#[must_use]
pub fn request(pgn: Pgn, source: u8) -> Frame {
    let id = CanId::from_parts(PRIORITY, Pgn::REQUEST, CanId::GLOBAL_ADDRESS, source);
    Frame::truncated(id, &pgn.to_bytes())
}

fn elapsed(since: Instant<Utc>, at: Instant<Utc>) -> bool {
    at.checked_duration_since(since)
        .is_some_and(|waited| waited >= CONTENTION_TIME)
}

/// Last address of the dynamic range of an industry group (isobus.net).
const fn last_dynamic_address(industry_group: u8) -> u8 {
    match industry_group {
        1 => 158,
        2 => 235,
        3..=5 => 207,
        _ => 247,
    }
}
