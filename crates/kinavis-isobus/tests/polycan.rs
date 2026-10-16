//! Every frame of a John Deere tractor's bus, read as a bus monitor would
//! read it, against a recorded tally.
//!
//! The logs are candump recordings of TractorHacking/PolyCAN (commit
//! f4f43254, `CANable/logs/oldLogs/*.log`); they carry no licence and are not
//! in this repository. CI fetches them at the pinned commit and names the
//! directory in `POLYCAN_LOGS`. Ignored by default; run with
//!
//! ```text
//! POLYCAN_LOGS=path/to/polycan/CANable/logs/oldLogs cargo test -p kinavis-isobus --test polycan -- --ignored
//! ```
//!
//! The tally counts frames by what they are and what became of them: address
//! claims and their NAMEs, requests, transport sessions completed or dropped,
//! ISOBUS messages and whether their values were available. A changed count
//! fails the test and prints the new tally; with `KINAVIS_BLESS=1` the new
//! tally is written as the expected one, to be reviewed in the diff.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::indexing_slicing
)]

use std::collections::{BTreeMap, BTreeSet};
use std::fmt::Write as _;
use std::fs;
use std::path::{Path, PathBuf};
use std::time::Duration;

use kinavis_isobus::Message;
use kinavis_j1939::{CanId, ExtendedReceiver, Frame, Name, Pgn, TransportReceiver};
use kinavis_kernel::{Instant, Utc};

const EXPECTED: &str = "tests/polycan.expected";

#[derive(Default)]
struct Tally {
    counts: BTreeMap<String, usize>,
    names: BTreeSet<(u8, u64)>,
    sources: BTreeSet<u8>,
}

impl Tally {
    fn count(&mut self, what: impl Into<String>) {
        *self.counts.entry(what.into()).or_default() += 1;
    }
}

#[test]
#[ignore = "needs the PolyCAN logs; see the module documentation"]
fn a_tractors_bus_reads_as_recorded() {
    let dir = std::env::var_os("POLYCAN_LOGS").expect("POLYCAN_LOGS names CANable/logs/oldLogs");
    let mut logs: Vec<PathBuf> = fs::read_dir(&dir)
        .unwrap()
        .map(|entry| entry.unwrap().path())
        .filter(|path| path.extension().is_some_and(|extension| extension == "log"))
        .collect();
    logs.sort();
    assert_eq!(logs.len(), 14, "logs in {dir:?}");

    let mut tally = Tally::default();
    for log in &logs {
        read_log(&fs::read_to_string(log).unwrap(), &mut tally);
    }
    let mut actual = format!("# Tally of the PolyCAN logs ({} logs).\n", logs.len());
    for (what, count) in &tally.counts {
        let _ = writeln!(actual, "{count:>7} {what}");
    }
    let _ = writeln!(actual, "{:>7} distinct sources", tally.sources.len());
    for (source, name) in &tally.names {
        let _ = writeln!(
            actual,
            "  claim from {source:#04x}: {}",
            Name::from_raw(*name)
        );
    }

    let expected_path = Path::new(env!("CARGO_MANIFEST_DIR")).join(EXPECTED);
    if std::env::var_os("KINAVIS_BLESS").is_some() {
        fs::write(&expected_path, &actual).unwrap();
        return;
    }
    let expected = fs::read_to_string(&expected_path).unwrap_or_default();
    assert!(
        actual == expected,
        "the tally changed; review it and rerun with KINAVIS_BLESS=1 to record it\n\n{actual}"
    );
}

/// `(1547753511.478075) can0 18EEFF47#4D74300400340220`
fn parse(line: &str) -> Option<(Instant<Utc>, Frame)> {
    let mut fields = line.split_ascii_whitespace();
    let stamp = fields.next()?.strip_prefix('(')?.strip_suffix(')')?;
    let (seconds, micros) = stamp.split_once('.')?;
    let at = Instant::<Utc>::from_unix_seconds(seconds.parse().ok()?)
        .saturating_add(Duration::from_micros(micros.parse().ok()?));
    let (id, data) = fields.nth(1)?.split_once('#')?;
    let id = CanId::new(u32::from_str_radix(id, 16).ok()?).ok()?;
    let bytes: Vec<u8> = (0..data.len() / 2)
        .map(|i| u8::from_str_radix(data.get(2 * i..2 * i + 2)?, 16).ok())
        .collect::<Option<_>>()?;
    Some((at, Frame::new(id, &bytes).ok()?))
}

fn read_log(text: &str, tally: &mut Tally) {
    let mut transport = TransportReceiver::<8, 1785>::new(None);
    let mut extended = ExtendedReceiver::<2, 65_536>::new(None);
    for line in text.lines().filter(|line| !line.trim().is_empty()) {
        let Some((at, frame)) = parse(line) else {
            tally.count("line not a frame");
            continue;
        };
        tally.count("frames");
        tally.sources.insert(frame.id.source());
        let _ = transport.expire(at, |_| panic!("a monitor owes no abort"));
        let pgn = frame.id.pgn();
        match pgn {
            Pgn::ADDRESS_CLAIMED => {
                tally.count("J1939: address claimed");
                if let Ok(bytes) = <[u8; 8]>::try_from(frame.data()) {
                    tally
                        .names
                        .insert((frame.id.source(), Name::from_bytes(bytes).raw()));
                }
            }
            Pgn::REQUEST => {
                let requested = frame
                    .data()
                    .get(..3)
                    .map(|b| Pgn::from_bytes([b[0], b[1], b[2]]));
                tally.count(format!(
                    "J1939: request for {}",
                    requested.map_or("?".into(), |p| p.to_string())
                ));
            }
            Pgn::TP_CONNECTION => {
                let kind = match frame.data().first() {
                    Some(16) => "RTS",
                    Some(17) => "CTS",
                    Some(19) => "end of message",
                    Some(32) => "BAM",
                    Some(255) => "abort",
                    _ => "other",
                };
                tally.count(format!("TP.CM: {kind}"));
            }
            Pgn::TP_DATA => tally.count("TP.DT"),
            Pgn::ETP_CONNECTION | Pgn::ETP_DATA => tally.count("ETP frame"),
            _ => {}
        }
        let received = transport.push(&frame, at);
        assert_eq!(received.reply, None, "a monitor never answers");
        if let Some(transfer) = received.transfer {
            let kind = if transfer.destination.is_some() {
                "connection"
            } else {
                "broadcast"
            };
            tally.count(format!(
                "TP {kind} completed: {} ({} bytes)",
                transfer.pgn,
                transfer.data().len()
            ));
        }
        if let Some(reason) = received.dropped {
            tally.count(format!("TP dropped: {reason:?}"));
        }
        let received = extended.push(&frame, at);
        if received.transfer.is_some() {
            tally.count("ETP completed");
        }
        match Message::decode(&frame) {
            Ok(Some(message)) => {
                let (kind, available) = match message {
                    Message::WheelBasedSpeed(m) => ("wheel-based speed", m.speed.is_some()),
                    Message::GroundBasedSpeed(m) => ("ground-based speed", m.speed.is_some()),
                    Message::MachineSelectedSpeed(m) => {
                        ("machine selected speed", m.speed.is_some())
                    }
                    Message::MachineSelectedSpeedCommand(m) => {
                        ("speed command", m.set_point.is_some())
                    }
                    Message::GuidanceMachineInfo(m) => {
                        ("guidance machine info", m.curvature.is_some())
                    }
                    Message::GuidanceSystemCommand(m) => {
                        ("guidance command", m.curvature.is_some())
                    }
                    _ => ("other", false),
                };
                let state = if available {
                    "with a value"
                } else {
                    "not available"
                };
                tally.count(format!("ISOBUS: {kind}, {state}"));
            }
            Ok(None) => {}
            Err(error) => tally.count(format!("ISOBUS refused: {error}")),
        }
    }
    // A recording that ends inside a transfer leaves it open.
    for _ in 0..transport.pending() {
        tally.count("TP open when the log ends");
    }
}
