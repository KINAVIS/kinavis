//! Every sentence of the gpsd regression logs, read as the navigation system
//! would read it, against a recorded tally.
//!
//! The logs, about two hundred, are recorded from real receivers and are not
//! in this repository: CI fetches `test/daemon/` of gpsd at a pinned commit
//! and names the directory in `GPSD_LOGS`. Ignored by default; run with
//!
//! ```text
//! GPSD_LOGS=path/to/gpsd/test/daemon cargo test -p kinavis-examples --test gpsd_corpus -- --ignored
//! ```
//!
//! The tally counts sentences by what became of them: read as which sentence,
//! refused for which error, translated into a fix or not, decoded as which AIS
//! message. A changed count fails the test and prints the new tally; with
//! `KINAVIS_BLESS=1` the new tally is written as the expected one, to be
//! reviewed in the diff.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::BTreeMap;
use std::fmt::{Debug, Write as _};
use std::fs;
use std::path::{Path, PathBuf};

use kinavis::{GnssFix, Instant, Utc};
use kinavis_ais::{Assembler, Message};
use kinavis_nmea0183::{parse, Sentence};

const EXPECTED: &str = "tests/gpsd_corpus.expected";

#[test]
#[ignore = "needs the gpsd logs; see the module documentation"]
fn the_gpsd_logs_read_as_recorded() {
    let dir = std::env::var_os("GPSD_LOGS").expect("GPSD_LOGS names gpsd's test/daemon directory");
    let logs = logs(Path::new(&dir));
    assert!(logs.len() > 100, "{} logs in {dir:?}", logs.len());

    let mut tally = Tally::default();
    for log in &logs {
        read_log(&fs::read(log).unwrap(), &mut tally);
    }
    let actual = tally.to_string(logs.len());

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

/// `*.log` files of a directory, in name order.
fn logs(dir: &Path) -> Vec<PathBuf> {
    let mut logs: Vec<PathBuf> = fs::read_dir(dir)
        .unwrap()
        .map(|entry| entry.unwrap().path())
        .filter(|path| path.extension().is_some_and(|extension| extension == "log"))
        .collect();
    logs.sort();
    logs
}

/// Counts by outcome.
#[derive(Default)]
struct Tally(BTreeMap<String, u64>);

impl Tally {
    fn count(&mut self, outcome: String) {
        *self.0.entry(outcome).or_default() += 1;
    }

    fn to_string(&self, logs: usize) -> String {
        let mut out = format!("# Tally of the gpsd regression logs ({logs} logs).\n");
        for (outcome, count) in &self.0 {
            writeln!(out, "{count:>7} {outcome}").unwrap();
        }
        out
    }
}

/// One log, line by line; only lines that start a sentence count. Binary logs
/// and comments are skipped by that rule.
fn read_log(bytes: &[u8], tally: &mut Tally) {
    let mut assembler = Assembler::new();
    let now = Instant::<Utc>::from_unix_seconds(0);
    for line in bytes.split(|&byte| byte == b'\n') {
        let line = line.strip_suffix(b"\r").unwrap_or(line);
        if !matches!(line.first(), Some(b'$' | b'!')) {
            continue;
        }
        match parse(line) {
            Err(error) => tally.count(format!("refused: {}", kind(&error))),
            Ok(Sentence::Rmc(rmc)) => match GnssFix::try_from(rmc) {
                Ok(_) => tally.count("RMC: fix".to_owned()),
                Err(error) => tally.count(format!("RMC: no fix, {}", kind(&error))),
            },
            Ok(Sentence::Gga(gga)) => {
                let fix = if gga.fix_type.is_position_fix() {
                    "position"
                } else {
                    "no fix"
                };
                tally.count(format!("GGA: {fix}"));
            }
            Ok(Sentence::Gll(_)) => tally.count("GLL".to_owned()),
            Ok(Sentence::Vtg(_)) => tally.count("VTG".to_owned()),
            Ok(Sentence::Vdm(vdm)) => match assembler.push(&vdm, now) {
                Ok(None) => tally.count("AIS: fragment held".to_owned()),
                Ok(Some(bits)) => match Message::decode(&bits) {
                    Ok(message) => tally.count(format!("AIS: {}", message_kind(&message))),
                    Err(error) => tally.count(format!("AIS: refused, {}", kind(&error))),
                },
                Err(error) => tally.count(format!("AIS: refused, {}", kind(&error))),
            },
            Ok(Sentence::Unsupported { .. }) => tally.count("unsupported sentence".to_owned()),
            Ok(other) => tally.count(format!("other: {}", kind(&other))),
        }
    }
}

/// Variant name of an enum value: its `Debug` form up to the first field.
fn kind(value: &impl Debug) -> String {
    let debug = format!("{value:?}");
    debug
        .split([' ', '(', '{'])
        .next()
        .unwrap_or_default()
        .to_owned()
}

fn message_kind(message: &Message) -> String {
    match message {
        Message::PositionReport(report) => format!("position report, type {}", report.kind),
        other => kind(other),
    }
}
