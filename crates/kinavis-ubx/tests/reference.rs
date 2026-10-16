//! UBX messages of the gpsd regression logs against pyubx2's decoding
//! (`tools/reference.py`): the first 40 of each type in each of the 29 logs
//! that carry UBX.
//!
//! The logs are not in this repository; CI fetches them for the real-data
//! job. Ignored by default; run with
//!
//! ```text
//! GPSD_LOGS=path/to/gpsd/test/daemon cargo test -p kinavis-ubx --test reference -- --ignored
//! ```

#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::panic,
    clippy::float_cmp,
    clippy::too_many_lines,
    clippy::cast_precision_loss,
    clippy::cast_possible_wrap,
    clippy::match_same_arms,
    clippy::semicolon_if_nothing_returned
)]

use std::collections::{BTreeMap, HashMap};
use std::path::Path;

use kinavis_ubx::{CarrierSolution, Deframer, Event, Measurement, Message, PvtFix};

const REFERENCE: &str = include_str!("data/ubx_reference.txt");

type Key = (String, &'static str, usize);

/// (log, message, occurrence) → field values as written by pyubx2.
fn reference() -> HashMap<Key, Vec<(String, String)>> {
    let names = [
        "NAV-PVT",
        "NAV-HPPOSLLH",
        "NAV-RELPOSNED",
        "NAV-SAT",
        "NAV-DOP",
        "NAV-STATUS",
        "NAV-TIMELS",
        "ESF-INS",
        "ESF-MEAS",
    ];
    let mut map = HashMap::new();
    for line in REFERENCE.lines().filter(|line| !line.starts_with('#')) {
        let mut words = line.split(' ');
        let log = words.next().unwrap().to_owned();
        let occurrence: usize = words.next().unwrap().parse().unwrap();
        let name = words.next().unwrap();
        let name = names.iter().find(|n| **n == name).copied().unwrap();
        let fields = words
            .map(|word| {
                let (key, value) = word.split_once('=').unwrap();
                (key.to_owned(), value.to_owned())
            })
            .collect();
        map.insert((log, name, occurrence), fields);
    }
    map
}

fn field<'a>(fields: &'a [(String, String)], key: &str) -> &'a str {
    &fields
        .iter()
        .find(|(k, _)| k == key)
        .unwrap_or_else(|| panic!("{key}"))
        .1
}

fn number(fields: &[(String, String)], key: &str) -> f64 {
    field(fields, key)
        .parse()
        .unwrap_or_else(|_| panic!("{key}"))
}

fn close(ours: f64, theirs: f64, tolerance: f64, what: &str) {
    assert!(
        (ours - theirs).abs() <= tolerance,
        "{what}: ours {ours}, pyubx2 {theirs}"
    );
}

fn carrier(solution: CarrierSolution) -> f64 {
    match solution {
        CarrierSolution::Float => 1.0,
        CarrierSolution::Fixed => 2.0,
        _ => 0.0,
    }
}

fn fix_number(fix: PvtFix) -> f64 {
    match fix {
        PvtFix::NoFix => 0.0,
        PvtFix::DeadReckoning => 1.0,
        PvtFix::TwoD => 2.0,
        PvtFix::ThreeD => 3.0,
        PvtFix::GnssDeadReckoning => 4.0,
        PvtFix::TimeOnly => 5.0,
        PvtFix::Other(other) => f64::from(other),
        _ => -1.0,
    }
}

fn flag(value: bool) -> f64 {
    if value {
        1.0
    } else {
        0.0
    }
}

#[test]
#[ignore = "needs the gpsd logs; see the module documentation"]
fn every_message_agrees_with_pyubx2() {
    let dir = std::env::var_os("GPSD_LOGS").expect("GPSD_LOGS names gpsd's test/daemon");
    let reference = reference();
    let mut checked: BTreeMap<&str, usize> = BTreeMap::new();
    let mut logs: Vec<String> = reference.keys().map(|(log, _, _)| log.clone()).collect();
    logs.sort();
    logs.dedup();
    for log in &logs {
        let bytes = std::fs::read(Path::new(&dir).join(log)).unwrap();
        let mut deframer: Deframer<8192> = Deframer::new();
        let mut counts: HashMap<&'static str, usize> = HashMap::new();
        for &byte in &bytes {
            let Some(Event::Frame(frame)) = deframer.push(byte) else {
                continue;
            };
            let Ok(message) = Message::decode(&frame) else {
                continue;
            };
            let name = match message {
                Message::NavPvt(_) => "NAV-PVT",
                Message::NavHpposllh(_) => "NAV-HPPOSLLH",
                Message::NavRelposned(_) => "NAV-RELPOSNED",
                Message::NavSat(_) => "NAV-SAT",
                Message::NavDop(_) => "NAV-DOP",
                Message::NavStatus(_) => "NAV-STATUS",
                Message::NavTimels(_) => "NAV-TIMELS",
                Message::EsfIns(_) => "ESF-INS",
                Message::EsfMeas(_) => "ESF-MEAS",
                _ => continue,
            };
            let occurrence = counts.entry(name).or_default();
            *occurrence += 1;
            let Some(f) = reference.get(&(log.clone(), name, *occurrence)) else {
                continue;
            };
            let at = format!("{log} {name} #{occurrence}");
            match message {
                Message::NavPvt(pvt) => {
                    close(f64::from(pvt.itow), number(f, "iTOW"), 0.0, &at);
                    close(fix_number(pvt.fix), number(f, "fixType"), 0.0, &at);
                    close(flag(pvt.fix_ok), number(f, "gnssFixOk"), 0.0, &at);
                    close(flag(pvt.differential), number(f, "diffSoln"), 0.0, &at);
                    close(carrier(pvt.carrier), number(f, "carrSoln"), 0.0, &at);
                    close(f64::from(pvt.satellites), number(f, "numSV"), 0.0, &at);
                    close(
                        pvt.position.longitude().degrees(),
                        number(f, "lon"),
                        1e-9,
                        &at,
                    );
                    close(
                        pvt.position.latitude().degrees(),
                        number(f, "lat"),
                        1e-9,
                        &at,
                    );
                    for (ours, key) in [
                        (pvt.height.metres(), "height"),
                        (pvt.height_msl.metres(), "hMSL"),
                        (pvt.horizontal_accuracy.metres(), "hAcc"),
                        (pvt.vertical_accuracy.metres(), "vAcc"),
                        (pvt.velocity.north().metres_per_second(), "velN"),
                        (pvt.velocity.east().metres_per_second(), "velE"),
                        (pvt.velocity.down().metres_per_second(), "velD"),
                        (pvt.ground_speed.metres_per_second(), "gSpeed"),
                    ] {
                        close(ours * 1000.0, number(f, key), 1e-6, &format!("{at} {key}"));
                    }
                    let heading = kinavis_kernel::wrap360(number(f, "headMot"));
                    close(pvt.heading_of_motion.degrees(), heading, 1e-9, &at);
                    close(
                        pvt.pdop.map_or(0.0, kinavis_kernel::gnss::Dop::value),
                        number(f, "pDOP"),
                        1e-9,
                        &at,
                    );
                    close(flag(pvt.valid_date), number(f, "validDate"), 0.0, &at);
                }
                Message::NavHpposllh(hp) => {
                    close(
                        hp.position.longitude().degrees(),
                        number(f, "lon"),
                        1e-9,
                        &at,
                    );
                    close(
                        hp.position.latitude().degrees(),
                        number(f, "lat"),
                        1e-9,
                        &at,
                    );
                    for (ours, key) in [
                        (hp.height.metres(), "height"),
                        (hp.height_msl.metres(), "hMSL"),
                        (hp.horizontal_accuracy.metres(), "hAcc"),
                        (hp.vertical_accuracy.metres(), "vAcc"),
                    ] {
                        close(ours * 1000.0, number(f, key), 1e-6, &format!("{at} {key}"));
                    }
                    close(flag(hp.invalid_llh), number(f, "invalidLlh"), 0.0, &at);
                }
                Message::NavRelposned(rel) => {
                    close(
                        rel.relative.north().metres() * 100.0,
                        number(f, "relPosN"),
                        1e-6,
                        &at,
                    );
                    close(
                        rel.relative.east().metres() * 100.0,
                        number(f, "relPosE"),
                        1e-6,
                        &at,
                    );
                    close(
                        rel.relative.down().metres() * 100.0,
                        number(f, "relPosD"),
                        1e-6,
                        &at,
                    );
                    close(
                        rel.accuracy.north().metres() * 1000.0,
                        number(f, "accN"),
                        1e-6,
                        &at,
                    );
                    close(flag(rel.valid), number(f, "relPosValid"), 0.0, &at);
                    close(carrier(rel.carrier), number(f, "carrSoln"), 0.0, &at);
                    if let Some(heading) = rel.heading {
                        let theirs = kinavis_kernel::wrap360(number(f, "relPosHeading"));
                        close(heading.degrees(), theirs, 1e-9, &at);
                    }
                }
                Message::NavSat(sat) => {
                    let theirs: Vec<&str> = f
                        .iter()
                        .filter(|(k, _)| k == "sv")
                        .map(|(_, v)| v.as_str())
                        .collect();
                    let ours: Vec<String> = sat
                        .satellites()
                        .map(|s| {
                            format!(
                                "{},{},{},{},{},{:?},{}",
                                s.gnss,
                                s.sv,
                                s.cno,
                                s.elevation,
                                s.azimuth,
                                f64::from(s.residual_decimetres) / 10.0,
                                u8::from(s.used())
                            )
                        })
                        .collect();
                    assert_eq!(ours, theirs, "{at}");
                    close(ours.len() as f64, number(f, "numSvs"), 0.0, &at);
                }
                Message::NavDop(dop) => {
                    for (ours, key) in [
                        (dop.gdop, "gDOP"),
                        (dop.pdop, "pDOP"),
                        (dop.hdop, "hDOP"),
                        (dop.vdop, "vDOP"),
                    ] {
                        close(
                            ours.map_or(0.0, kinavis_kernel::gnss::Dop::value),
                            number(f, key),
                            1e-9,
                            &format!("{at} {key}"),
                        );
                    }
                }
                Message::NavStatus(status) => {
                    close(fix_number(status.fix), number(f, "gpsFix"), 0.0, &at);
                    close(flag(status.fix_ok), number(f, "gpsFixOk"), 0.0, &at);
                    close(
                        f64::from(status.time_to_first_fix),
                        number(f, "ttff"),
                        0.0,
                        &at,
                    );
                    close(f64::from(status.since_startup), number(f, "msss"), 0.0, &at);
                }
                Message::NavTimels(timels) => {
                    let valid = number(f, "validCurrLs") == 1.0;
                    assert_eq!(timels.current.is_some(), valid, "{at}");
                    if let Some(current) = timels.current {
                        close(f64::from(current), number(f, "currLs"), 0.0, &at);
                    }
                }
                Message::EsfIns(ins) => {
                    for (ours, key) in [
                        (ins.acceleration[0], "xAccel"),
                        (ins.acceleration[1], "yAccel"),
                        (ins.acceleration[2], "zAccel"),
                    ] {
                        if let Some(ours) = ours {
                            close(
                                ours.metres_per_second_squared(),
                                number(f, key),
                                1e-9,
                                &format!("{at} {key}"),
                            );
                        }
                    }
                    if let Some(rate) = ins.angular_rate[2] {
                        close(rate, number(f, "zAngRate"), 1e-9, &at);
                    }
                }
                Message::EsfMeas(meas) => {
                    close(f64::from(meas.provider), number(f, "id"), 0.0, &at);
                    let theirs: Vec<(u8, u32)> = f
                        .iter()
                        .filter(|(k, _)| k == "meas")
                        .map(|(_, v)| {
                            let (t, d) = v.split_once(',').unwrap();
                            (t.parse().unwrap(), d.parse().unwrap())
                        })
                        .collect();
                    let ours: Vec<Measurement> = meas.measurements().collect();
                    assert_eq!(ours.len(), theirs.len(), "{at}");
                    for (ours, (data_type, field)) in ours.iter().zip(theirs) {
                        let signed = f64::from(((field << 8) as i32) >> 8);
                        match (data_type, ours) {
                            (16..=18, Measurement::Acceleration { value, .. }) => {
                                close(
                                    value.metres_per_second_squared(),
                                    signed / 1024.0,
                                    1e-12,
                                    &at,
                                );
                            }
                            (
                                5 | 13 | 14,
                                Measurement::AngularRate {
                                    degrees_per_second, ..
                                },
                            ) => {
                                close(*degrees_per_second, signed / 4096.0, 1e-12, &at);
                            }
                            (12, Measurement::GyroTemperature(t)) => {
                                close(*t, signed / 100.0, 1e-12, &at)
                            }
                            (
                                6..=9,
                                Measurement::WheelTicks {
                                    ticks, backward, ..
                                },
                            ) => {
                                assert_eq!(*ticks, field & 0x7F_FFFF, "{at}");
                                assert_eq!(*backward, field & 0x80_0000 != 0, "{at}");
                            }
                            (10, Measurement::SpeedTicks { ticks, backward }) => {
                                assert_eq!(*ticks, field & 0x7F_FFFF, "{at}");
                                assert_eq!(*backward, field & 0x80_0000 != 0, "{at}");
                            }
                            (11, Measurement::Speed(speed)) => {
                                close(speed.metres_per_second(), signed / 1000.0, 1e-12, &at);
                            }
                            (other, Measurement::Other { data_type, .. }) => {
                                assert_eq!(other, *data_type, "{at}")
                            }
                            (t, m) => panic!("{at}: type {t} read as {m:?}"),
                        }
                    }
                }
                _ => unreachable!(),
            }
            *checked.entry(name).or_default() += 1;
        }
    }
    let total: usize = checked.values().sum();
    assert_eq!(
        total,
        reference.len(),
        "every reference message found: {checked:?}"
    );
}
