//! Address claim: a node alone, conflicts won and lost, the self-configurable
//! node moving on, the node that cannot claim, answers to requests.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use core::time::Duration;

use kinavis_j1939::{
    AddressClaimer, CanId, ClaimState, Frame, Name, NameFields, Pgn, CONTENTION_TIME,
};
use kinavis_kernel::{Instant, Utc};

fn at(millis: u64) -> Instant<Utc> {
    Instant::<Utc>::from_unix_seconds(1_760_000_000).saturating_add(Duration::from_millis(millis))
}

fn name(identity: u32, self_configurable: bool) -> Name {
    Name::new(NameFields {
        self_configurable,
        industry_group: 2,
        device_class: 4,
        function: 130,
        manufacturer_code: 1407,
        identity_number: identity,
        ..NameFields::default()
    })
    .unwrap()
}

fn claim_from(name: Name, source: u8) -> Frame {
    Frame::of(
        6,
        Pgn::ADDRESS_CLAIMED,
        CanId::GLOBAL_ADDRESS,
        source,
        &name.to_bytes(),
    )
    .unwrap()
}

#[test]
fn a_node_alone_claims_its_preferred_address() {
    let mut claimer = AddressClaimer::new(name(100, false), 0x80).unwrap();
    let request = claimer.start(at(0));
    assert_eq!(request.id.pgn(), Pgn::REQUEST);
    assert_eq!(request.id.source(), CanId::NULL_ADDRESS);
    assert_eq!(request.id.destination(), Some(CanId::GLOBAL_ADDRESS));
    assert_eq!(request.data(), &Pgn::ADDRESS_CLAIMED.to_bytes());
    assert_eq!(claimer.poll(at(100)), None);
    let claim = claimer
        .poll(at(250))
        .expect("a claim after the contention time");
    assert_eq!(claim.id.pgn(), Pgn::ADDRESS_CLAIMED);
    assert_eq!(claim.id.source(), 0x80);
    assert_eq!(claim.data(), &claimer.name().to_bytes());
    assert_eq!(claimer.address(), None);
    assert_eq!(claimer.poll(at(250).saturating_add(CONTENTION_TIME)), None);
    assert_eq!(claimer.address(), Some(0x80));
}

#[test]
fn the_lower_name_keeps_the_address() {
    let ours = name(100, false);
    let mut claimer = AddressClaimer::new(ours, 0x80).unwrap();
    let _ = claimer.start(at(0));
    let _ = claimer.poll(at(300));
    let _ = claimer.poll(at(600));
    assert_eq!(claimer.address(), Some(0x80));
    // A higher NAME claims our address: we answer with our claim and keep it.
    let answer = claimer
        .handle(&claim_from(name(200, false), 0x80), at(700))
        .expect("an answer");
    assert_eq!(answer.id.source(), 0x80);
    assert_eq!(claimer.address(), Some(0x80));
    // A lower NAME takes it: we cannot move, so we cannot claim.
    let answer = claimer
        .handle(&claim_from(name(50, false), 0x80), at(800))
        .expect("an answer");
    assert_eq!(answer.id.source(), CanId::NULL_ADDRESS);
    assert_eq!(answer.data(), &ours.to_bytes());
    assert_eq!(claimer.state(), ClaimState::CannotClaim);
}

#[test]
fn a_self_configurable_node_moves_to_the_first_free_dynamic_address() {
    let mut claimer = AddressClaimer::new(name(100, true), 0x1C).unwrap();
    let _ = claimer.start(at(0));
    // While listening: a lower NAME holds 0x1C, others hold 128 and 129.
    for (identity, source) in [(10, 0x1C), (300, 128), (301, 129)] {
        assert_eq!(
            claimer.handle(&claim_from(name(identity, false), source), at(50)),
            None
        );
    }
    let claim = claimer.poll(at(250)).expect("a claim");
    assert_eq!(claim.id.source(), 130);
    // A lower NAME on 130 later: on to 131.
    let claim = claimer
        .handle(&claim_from(name(5, false), 130), at(300))
        .expect("a claim");
    assert_eq!(claim.id.source(), 131);
    let _ = claimer.poll(at(600));
    assert_eq!(claimer.address(), Some(131));
}

#[test]
fn a_request_is_answered_only_when_it_is_for_us() {
    let mut claimer = AddressClaimer::new(name(100, false), 0x80).unwrap();
    let ask = |destination: u8| {
        Frame::of(
            6,
            Pgn::REQUEST,
            destination,
            0x26,
            &Pgn::ADDRESS_CLAIMED.to_bytes(),
        )
        .unwrap()
    };
    // Not started: silent.
    assert_eq!(claimer.handle(&ask(CanId::GLOBAL_ADDRESS), at(0)), None);
    let _ = claimer.start(at(0));
    let _ = claimer.poll(at(250));
    let _ = claimer.poll(at(500));
    assert_eq!(
        claimer
            .handle(&ask(CanId::GLOBAL_ADDRESS), at(600))
            .map(|f| f.id.source()),
        Some(0x80)
    );
    assert_eq!(
        claimer.handle(&ask(0x80), at(600)).map(|f| f.id.source()),
        Some(0x80)
    );
    assert_eq!(claimer.handle(&ask(0x81), at(600)), None);
    // A request for another group is not ours to answer.
    let other = Frame::of(
        6,
        Pgn::REQUEST,
        CanId::GLOBAL_ADDRESS,
        0x26,
        &Pgn::new(65_242).to_bytes(),
    )
    .unwrap();
    assert_eq!(claimer.handle(&other, at(600)), None);
}

#[test]
fn reserved_addresses_cannot_be_preferred() {
    assert!(AddressClaimer::new(name(1, true), 254).is_err());
    assert!(AddressClaimer::new(name(1, true), 255).is_err());
}
