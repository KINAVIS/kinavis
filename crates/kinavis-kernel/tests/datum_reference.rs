//! Datum shifts against PROJ 9.5.1, using the same EPSG transformations written
//! out as pipelines so no grid or registry default substitutes for the
//! parameters:
//!
//! ```text
//! +proj=pipeline +step +proj=unitconvert +xy_in=deg +xy_out=rad
//!   +step +proj=cart +ellps=<datum's>
//!   +step +proj=helmert <the datum's parameters>
//!   +step +inv +proj=cart +ellps=WGS84
//!   +step +proj=unitconvert +xy_in=rad +xy_out=deg
//! ```
//!
//! Required agreement: 1e-8° (~1 mm). Same arithmetic, same parameters; a
//! looser tolerance would hide a sign or convention error.

#![allow(clippy::expect_used)]

use kinavis_kernel::geodesy::Datum;
use kinavis_kernel::Position;

/// Degrees of latitude and longitude.
type Degrees = (f64, f64);

/// (datum, position on the datum, WGS 84 position by PROJ).
const REFERENCE: &[(&Datum, Degrees, Degrees)] = &[
    (
        &Datum::OSGB36,
        (52.657_570_3, 1.717_921_6),
        (52.657_978_596, 1.716_052_006),
    ),
    (
        &Datum::OSGB36,
        (57.15, -2.1),
        (57.149_814_646, -2.101_622_400),
    ),
    (
        &Datum::OSGB36,
        (50.0, -5.5),
        (50.000_613_771, -5.500_948_254),
    ),
    (&Datum::ED50, (52.0, 4.5), (51.999_201_775, 4.498_676_873)),
    (&Datum::ED50, (36.5, -6.0), (36.498_745_746, -6.001_189_337)),
    (
        &Datum::NAD27,
        (40.0, -100.0),
        (40.000_009_483, -100.000_417_622),
    ),
    (
        &Datum::NAD27,
        (30.0, -88.0),
        (30.000_241_943, -88.000_024_990),
    ),
    (
        &Datum::PULKOVO_1942,
        (59.95, 30.3),
        (59.949_973_156, 30.297_744_801),
    ),
    (
        &Datum::PULKOVO_1942,
        (43.1, 131.9),
        (43.100_306_854, 131.901_092_975),
    ),
    (
        &Datum::TOKYO,
        (35.65, 139.75),
        (35.653_269_620, 139.746_782_892),
    ),
    (
        &Datum::TOKYO,
        (34.0, 132.0),
        (34.003_276_100, 131.997_518_309),
    ),
    (&Datum::DHDN, (53.55, 9.99), (53.548_450_695, 9.988_770_637)),
    (
        &Datum::DHDN,
        (48.14, 11.58),
        (48.139_078_487, 11.578_611_372),
    ),
    (
        &Datum::AGD66,
        (-33.86, 151.21),
        (-33.858_414_269, 151.211_160_430),
    ),
    (
        &Datum::AGD66,
        (-12.46, 130.84),
        (-12.458_562_334, 130.841_203_940),
    ),
    (
        &Datum::SAD69,
        (-22.9, -43.2),
        (-22.900_485_611, -43.200_373_208),
    ),
    (
        &Datum::SAD69,
        (-34.6, -58.4),
        (-34.600_456_456, -58.400_523_542),
    ),
];

/// 1 mm and 1 cm, in degrees of latitude.
const MILLIMETRE_DEGREES: f64 = 1e-8;
const CENTIMETRE_DEGREES: f64 = 1e-7;

fn position((latitude, longitude): Degrees) -> Position {
    Position::from_degrees(latitude, longitude).expect("in range")
}

#[test]
fn every_datum_agrees_with_proj_to_a_millimetre() {
    for &(datum, on_datum, on_wgs84) in REFERENCE {
        let shifted = datum
            .to_wgs84(position(on_datum))
            .expect("a place on Earth");
        let expected = position(on_wgs84);
        let dlat = (shifted.latitude().degrees() - expected.latitude().degrees()).abs();
        let dlon = shifted.longitude_difference(expected).degrees().abs();
        assert!(
            dlat < MILLIMETRE_DEGREES && dlon < MILLIMETRE_DEGREES,
            "{datum} at {on_datum:?}: got {shifted}, PROJ says {expected}"
        );
    }
}

#[test]
fn the_shift_back_returns_to_the_chart_within_the_height_dropped() {
    // PROJ also drops height at both ends and closes to 3e-8° at worst (Tokyo:
    // 60 m of height dropped along a normal tilted 0.1 mrad by the 700 m
    // shift). 1 cm required here.
    for &(datum, on_datum, _) in REFERENCE {
        let there = position(on_datum);
        let back = datum
            .from_wgs84(datum.to_wgs84(there).expect("forward"))
            .expect("back");
        let dlat = (back.latitude().degrees() - there.latitude().degrees()).abs();
        let dlon = back.longitude_difference(there).degrees().abs();
        assert!(
            dlat < CENTIMETRE_DEGREES && dlon < CENTIMETRE_DEGREES,
            "{datum} at {on_datum:?}: came back as {back}"
        );
    }
}

#[test]
fn the_shifts_are_the_size_the_charts_warn_of() {
    // Tokyo > 400 m, Great Britain ~100 m, US tens of metres: a wrong ellipsoid
    // or sign would not produce these.
    let metres = |datum: &Datum, place: Degrees| {
        let there = position(place);
        let shifted = datum.to_wgs84(there).expect("a place on Earth");
        let dlat = (shifted.latitude().degrees() - there.latitude().degrees()).to_radians();
        let dlon = shifted.longitude_difference(there).radians()
            * there.latitude().degrees().to_radians().cos();
        6_371_000.0 * dlat.hypot(dlon)
    };
    let tokyo = metres(&Datum::TOKYO, (35.65, 139.75));
    assert!((440.0..480.0).contains(&tokyo), "{tokyo}");
    let london = metres(&Datum::OSGB36, (51.5, -0.1));
    assert!((100.0..140.0).contains(&london), "{london}");
    let kansas = metres(&Datum::NAD27, (40.0, -100.0));
    assert!((30.0..40.0).contains(&kansas), "{kansas}");
    let baltic = metres(&Datum::PULKOVO_1942, (59.95, 30.3));
    assert!((100.0..140.0).contains(&baltic), "{baltic}");
}
