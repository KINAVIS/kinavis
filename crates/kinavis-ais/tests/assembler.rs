//! Fragment reassembly: ordering, timeouts, slots.

#![allow(
    clippy::unwrap_used,
    clippy::panic,
    clippy::redundant_closure_for_method_calls
)]

use core::time::Duration;

use kinavis_ais::{AisError, Assembler, Bits, Message, MAX_ASSEMBLIES};
use kinavis_kernel::{Instant, Utc};
use kinavis_nmea0183::{parse, Sentence, Vdm};

fn vdm(line: &str) -> Vdm {
    let Sentence::Vdm(vdm) = parse(line.as_bytes()).unwrap() else {
        panic!("not an AIS sentence");
    };
    vdm
}

fn at(seconds: i64) -> Instant<Utc> {
    Instant::from_unix_seconds(1_789_000_000 + seconds)
}

const FIRST: &str = "!AIVDM,2,1,7,B,C5N3SRP0EnJGEBT>NhMLeLl0`:,0*38";
const SECOND: &str = "!AIVDM,2,2,7,B,Va04N2`00000000000S0`:1R30,0*54";
const WHOLE: &str = "!AIVDM,1,1,,A,C5N3SRP0EnJGEBT>NhMLeLl0`:Va04N2`00000000000S0`:1R30,0*49";

#[test]
fn two_fragments_make_the_message_the_one_sentence_carries() {
    let mut assembler = Assembler::new();
    assert_eq!(assembler.push(&vdm(FIRST), at(0)), Ok(None));
    assert_eq!(assembler.pending(), 1);
    let bits = assembler.push(&vdm(SECOND), at(1)).unwrap().unwrap();
    assert_eq!(assembler.pending(), 0);

    let whole = assembler.push(&vdm(WHOLE), at(1)).unwrap().unwrap();
    assert_eq!(bits, whole);
    assert!(matches!(
        Message::decode(&bits),
        Ok(Message::PositionReport(_))
    ));
}

#[test]
fn fill_bits_of_the_last_fragment_come_off() {
    let mut assembler = Assembler::new();
    let first = "!AIVDM,2,1,3,B,55P5TL01VIaAL@7WKO@mBplU@<PDhh000000001S;AJ::4A80?4i@E53,0*3E";
    let second = "!AIVDM,2,2,3,B,1@0000000000000,2*55";
    assert_eq!(assembler.push(&vdm(first), at(0)), Ok(None));
    let bits = assembler.push(&vdm(second), at(0)).unwrap().unwrap();
    assert_eq!(bits.len(), 424);
    let Ok(Message::StaticAndVoyageData(data)) = Message::decode(&bits) else {
        panic!("not a type 5");
    };
    assert_eq!(data.mmsi.number(), 369_190_000);
    assert_eq!(data.name.unwrap(), "MT.MITCHELL");
}

#[test]
fn fragments_of_different_messages_interleave() {
    let mut assembler = Assembler::new();
    let other_first =
        "!AIVDM,2,1,3,B,55P5TL01VIaAL@7WKO@mBplU@<PDhh000000001S;AJ::4A80?4i@E53,0*3E";
    let other_second = "!AIVDM,2,2,3,B,1@0000000000000,2*55";
    assert_eq!(assembler.push(&vdm(FIRST), at(0)), Ok(None));
    assert_eq!(assembler.push(&vdm(other_first), at(0)), Ok(None));
    assert_eq!(assembler.pending(), 2);
    let other = assembler.push(&vdm(other_second), at(0)).unwrap().unwrap();
    assert!(matches!(
        Message::decode(&other),
        Ok(Message::StaticAndVoyageData(_))
    ));
    let this = assembler.push(&vdm(SECOND), at(0)).unwrap().unwrap();
    assert!(matches!(
        Message::decode(&this),
        Ok(Message::PositionReport(_))
    ));
    assert_eq!(assembler.pending(), 0);
}

#[test]
fn a_fragment_out_of_order_is_refused_and_the_assembly_given_up() {
    let mut assembler = Assembler::new();
    // Second fragment without a first.
    assert_eq!(
        assembler.push(&vdm(SECOND), at(0)),
        Err(AisError::UnexpectedFragment {
            expected: 1,
            found: 2
        })
    );
    // A repeated first fragment restarts the assembly.
    assert_eq!(assembler.push(&vdm(FIRST), at(0)), Ok(None));
    assert_eq!(assembler.push(&vdm(FIRST), at(0)), Ok(None));
    assert_eq!(assembler.pending(), 1);
    // Third fragment of a two-fragment message.
    let third = vdm("!AIVDM,3,3,7,B,Va04N2`00000000000S0`:1R30,0*54");
    assert_eq!(
        assembler.push(&third, at(0)),
        Err(AisError::UnexpectedFragment {
            expected: 2,
            found: 3
        })
    );
    assert_eq!(assembler.pending(), 0, "the assembly is given up");
    assert_eq!(
        assembler.push(&vdm(SECOND), at(0)),
        Err(AisError::UnexpectedFragment {
            expected: 1,
            found: 2
        })
    );
}

#[test]
fn an_assembly_that_does_not_finish_in_time_is_given_up() {
    let mut assembler = Assembler::with_timeout(Duration::from_secs(5));
    assert_eq!(assembler.timeout(), Duration::from_secs(5));
    assert_eq!(assembler.push(&vdm(FIRST), at(0)), Ok(None));
    assert_eq!(
        assembler.push(&vdm(SECOND), at(6)),
        Err(AisError::UnexpectedFragment {
            expected: 1,
            found: 2
        })
    );
    // Would have completed within the timeout.
    assert_eq!(assembler.push(&vdm(FIRST), at(10)), Ok(None));
    assert!(assembler.push(&vdm(SECOND), at(15)).unwrap().is_some());
    // A clock step backwards also drops the assembly.
    assert_eq!(assembler.push(&vdm(FIRST), at(20)), Ok(None));
    assembler.expire(at(19));
    assert_eq!(assembler.pending(), 0);
}

#[test]
fn a_full_assembler_gives_the_oldest_assembly_up_for_a_new_message() {
    let mut assembler = Assembler::new();
    for sequence in 0..MAX_ASSEMBLIES {
        let line = format!("AIVDM,2,1,{sequence},B,C5N3SRP0EnJGEBT>NhMLeLl0`:,0");
        let checksum = line.bytes().fold(0, |sum, byte| sum ^ byte);
        // Started 1 s apart; sequence 0 is the oldest.
        assert_eq!(
            assembler.push(
                &vdm(&format!("!{line}*{checksum:02X}")),
                at(i64::try_from(sequence).unwrap())
            ),
            Ok(None)
        );
    }
    assert_eq!(assembler.pending(), MAX_ASSEMBLIES);
    // A fifth message evicts sequence 0; its second fragment has no assembly.
    let fifth = vdm("!AIVDM,2,1,9,B,C5N3SRP0EnJGEBT>NhMLeLl0`:,0*36");
    assert_eq!(assembler.push(&fifth, at(5)), Ok(None));
    assert_eq!(assembler.pending(), MAX_ASSEMBLIES);
    let orphan = vdm("!AIVDM,2,2,0,B,Va04N2`00000000000S0`:1R30,0*53");
    assert!(matches!(
        assembler.push(&orphan, at(5)),
        Err(AisError::UnexpectedFragment { .. })
    ));
    // A single-fragment message needs no slot.
    assert!(assembler.push(&vdm(WHOLE), at(5)).unwrap().is_some());
    // The remaining messages and the newcomer complete.
    let second = vdm("!AIVDM,2,2,9,B,Va04N2`00000000000S0`:1R30,0*5A");
    let bits = assembler.push(&second, at(5)).unwrap().unwrap();
    assert_eq!(
        bits,
        Bits::unarmour(vdm(WHOLE).payload.as_bytes(), 0).unwrap()
    );
}

#[test]
fn the_same_sequence_on_another_channel_is_another_message() {
    let mut assembler = Assembler::new();
    assert_eq!(assembler.push(&vdm(FIRST), at(0)), Ok(None));
    let on_a = vdm("!AIVDM,2,1,7,A,C5N3SRP0EnJGEBT>NhMLeLl0`:,0*3B");
    assert_eq!(assembler.push(&on_a, at(0)), Ok(None));
    assert_eq!(assembler.pending(), 2);
}
