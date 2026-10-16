//! Configuration frames byte for byte as pyubx2 writes them, and a
//! `CFG-VALGET` answer pyubx2 reads as the values below.

#![allow(clippy::unwrap_used, clippy::panic)]

use kinavis_ubx::config::{rate, valget, valset};
use kinavis_ubx::{
    ConfigError, ConfigKey, Deframer, Event, Layers, Message, PollLayer, TimeReference,
};

fn hex(text: &str) -> Vec<u8> {
    (0..text.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(text.get(i..i + 2).unwrap(), 16).unwrap())
        .collect()
}

#[test]
fn valset_valget_and_rate_match_pyubx2() {
    let mut out = [0_u8; 128];
    let n = valset(
        Layers::RAM.and(Layers::BBR),
        &[
            (ConfigKey::RATE_MEAS, 100),
            (ConfigKey::MSGOUT_NAV_PVT_UART1, 1),
            (ConfigKey::UART1_BAUDRATE, 115_200),
        ],
        &mut out,
    )
    .unwrap();
    assert_eq!(
        out.get(..n).unwrap(),
        hex("b562068a17000003000001002130640007009120010100524000c201006fa2")
    );

    let n = valset(
        Layers::FLASH,
        &[
            (ConfigKey::UART1_OUTPUT_NMEA, 0),
            (ConfigKey::NAVSPG_DYNMODEL, 4),
        ],
        &mut out,
    )
    .unwrap();
    assert_eq!(
        out.get(..n).unwrap(),
        hex("b562068a0e0000040000020074100021001120047eea")
    );

    let n = valget(
        PollLayer::Ram,
        0,
        &[ConfigKey::RATE_MEAS, ConfigKey::UART1_BAUDRATE],
        &mut out,
    )
    .unwrap();
    assert_eq!(
        out.get(..n).unwrap(),
        hex("b562068b0c0000000000010021300100524082d3")
    );

    let n = rate(200, 1, TimeReference::Gps, &mut out).unwrap();
    assert_eq!(out.get(..n).unwrap(), hex("b56206080600c80001000100de6a"));
}

#[test]
fn a_valget_answer_reads_back_its_values() {
    let mut deframer: Deframer<64> = Deframer::new();
    let mut values = Vec::new();
    for byte in hex("b562068b1700010000000100213064000100524000c2010002007410013ce7") {
        if let Some(Event::Frame(frame)) = deframer.push(byte) {
            let Message::CfgValget(answer) = Message::decode(&frame).unwrap() else {
                panic!("CFG-VALGET")
            };
            values = answer.values().collect::<Result<Vec<_>, _>>().unwrap();
        }
    }
    assert_eq!(
        values,
        vec![
            (ConfigKey::RATE_MEAS, 100),
            (ConfigKey::UART1_BAUDRATE, 115_200),
            (ConfigKey::UART1_OUTPUT_NMEA, 1),
        ]
    );
}

#[test]
fn what_cannot_be_written_is_refused() {
    let mut out = [0_u8; 16];
    assert_eq!(
        valset(Layers::RAM, &[(ConfigKey::RATE_MEAS, 70_000)], &mut [0; 64]),
        Err(ConfigError::ValueTooLarge {
            key: ConfigKey::RATE_MEAS,
            value: 70_000
        })
    );
    assert!(matches!(
        valset(Layers::RAM, &[(ConfigKey::UART1_BAUDRATE, 9600)], &mut out),
        Err(ConfigError::BufferTooSmall { needed: 20 })
    ));
    let many = [ConfigKey::RATE_MEAS; 65];
    assert_eq!(
        valget(PollLayer::Ram, 0, &many, &mut [0; 512]),
        Err(ConfigError::TooManyKeys(65))
    );
    assert!(ConfigKey::new(0x0000_0001).is_err());
    assert_eq!(ConfigKey::new(0x5000_0001).unwrap().value_size(), 8);
}
