//! Fuzz target: frames of the ISOBUS groups to messages and back.
//!
//! Input: one byte picking the group and up to 8 data bytes. Invariants: no
//! panic; a message decoded and encoded decodes to itself.
#![no_main]

use libfuzzer_sys::fuzz_target;

use kinavis_isobus::{
    GroundBasedSpeed, GuidanceMachineInfo, GuidanceSystemCommand, MachineSelectedSpeed,
    MachineSelectedSpeedCommand, Message, WheelBasedSpeed,
};
use kinavis_j1939::{CanId, Frame};

fuzz_target!(|data: &[u8]| {
    let Some((pick, bytes)) = data.split_first() else {
        return;
    };
    let pgns = [
        WheelBasedSpeed::PGN,
        GroundBasedSpeed::PGN,
        MachineSelectedSpeed::PGN,
        MachineSelectedSpeedCommand::PGN,
        GuidanceMachineInfo::PGN,
        GuidanceSystemCommand::PGN,
    ];
    let Some(pgn) = pgns.get(usize::from(*pick) % pgns.len()) else {
        return;
    };
    let Ok(frame) = Frame::new(CanId::from_parts(3, *pgn, 0xFF, 0x47), bytes.get(..bytes.len().min(8)).unwrap_or(&[])) else {
        return;
    };
    let Ok(Some(message)) = Message::decode(&frame) else {
        return;
    };
    let encoded = match message {
        Message::WheelBasedSpeed(m) => m.to_bytes(),
        Message::GroundBasedSpeed(m) => m.to_bytes(),
        Message::MachineSelectedSpeed(m) => m.to_bytes(),
        Message::MachineSelectedSpeedCommand(m) => m.to_bytes(),
        Message::GuidanceMachineInfo(m) => m.to_bytes(),
        Message::GuidanceSystemCommand(m) => m.to_bytes(),
        _ => return,
    };
    let again = Frame::new(frame.id, &encoded).unwrap_or(frame);
    assert_eq!(Message::decode(&again).ok().flatten(), Some(message));
});
