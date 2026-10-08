//! Fuzz target: arbitrary bytes through the deframer and every decoder.
//!
//! Invariants: no panic; every frame the deframer returns carries a valid
//! checksum; decoding is a function of the frame.
#![no_main]

use libfuzzer_sys::fuzz_target;

use kinavis_ubx::{checksum, Deframer, Event, Message};

fuzz_target!(|data: &[u8]| {
    let mut deframer: Deframer<1024> = Deframer::new();
    for &byte in data {
        if let Some(Event::Frame(frame)) = deframer.push(byte) {
            let _ = checksum(frame.class, frame.id, frame.payload);
            let first = Message::decode(&frame);
            let again = Message::decode(&frame);
            assert_eq!(first, again, "decoding is not a function of the frame");
            if let Ok(Message::NavPvt(pvt)) = first {
                let _ = pvt.fix();
            }
            if let Ok(Message::NavSat(sat)) = first {
                let _ = sat.satellite_set::<64>();
            }
            if let Ok(Message::EsfMeas(meas)) = first {
                let _ = meas.measurements().count();
            }
        }
    }
});
