//! MGRS against GeographicLib's `GeoConvert` (`tools/reference.py`): the
//! reference of 5400 points at precisions 0 to 11, and the centre of each
//! referenced square read back.

#![allow(clippy::expect_used, clippy::panic)]

use kinavis_grid::{Hemisphere, Mgrs, MgrsSquare, UtmUps};
use kinavis_kernel::Position;

const REFERENCE: &str = include_str!("data/mgrs_reference.txt");

fn number(field: &str) -> f64 {
    field.parse().expect("a number")
}

#[test]
fn references_and_centres_agree_with_geographiclib() {
    let mut checked = 0;
    for line in REFERENCE.lines().filter(|line| !line.starts_with('#')) {
        let fields: Vec<&str> = line.split_ascii_whitespace().collect();
        let [lat, lon, precision, reference, zone, hemisphere, easting, northing] = fields[..]
        else {
            panic!("eight columns: {line}");
        };
        let position = Position::from_degrees(number(lat), number(lon)).expect("on Earth");
        let precision: u8 = precision.parse().expect("precision");
        let ours = Mgrs::from_position(position, precision).expect("a reference");
        assert_eq!(
            ours.as_str(),
            reference,
            "at {lat} {lon}, precision {precision}"
        );

        let square: MgrsSquare = reference.parse().expect("parses");
        assert_eq!(square.precision, Some(precision));
        let (ours_zone, ours_hemisphere, ours_e, ours_n) = match square.centre {
            UtmUps::Utm {
                zone,
                hemisphere,
                easting,
                northing,
            } => (zone.number(), hemisphere, easting, northing),
            UtmUps::Ups {
                hemisphere,
                easting,
                northing,
            } => (0, hemisphere, easting, northing),
        };
        assert_eq!(ours_zone, zone.parse::<u8>().expect("zone"), "{reference}");
        assert_eq!(
            ours_hemisphere == Hemisphere::North,
            hemisphere == "n",
            "{reference}"
        );
        assert!(
            (ours_e.metres() - number(easting)).abs() < 1e-6,
            "{reference}: {ours_e:?}"
        );
        assert!(
            (ours_n.metres() - number(northing)).abs() < 1e-6,
            "{reference}: {ours_n:?}"
        );
        checked += 1;
    }
    assert_eq!(checked, 5400);
}

#[test]
fn malformed_references_are_refused() {
    for text in [
        "",
        "3",
        "61SMB",
        "0SMB",
        "38I",
        "38SMB1",
        "38SMB12345678901234567890123",
        "38SIB",
        "38SMB44x8",
        "123SMB",
        "38SMBB",
        "CAA",
        "YZ",
        "38 SMB",
    ] {
        assert!(text.parse::<MgrsSquare>().is_err(), "{text:?}");
    }
    assert!("38smb4488".parse::<MgrsSquare>().is_ok(), "lower case");
    let zone_only: MgrsSquare = "38S".parse().expect("a grid zone");
    assert_eq!(zone_only.precision, None);
}

#[test]
fn a_precision_beyond_a_micrometre_is_refused() {
    let position = Position::from_degrees(10.0, 10.0).expect("on Earth");
    assert!(Mgrs::from_position(position, 12).is_err());
}

mod properties {
    use super::*;
    use proptest::prelude::*;

    proptest! {
        #[test]
        fn a_reference_names_the_square_its_centre_lies_in(
            latitude in -90.0_f64..=90.0,
            longitude in -180.0_f64..180.0,
            precision in 1_u8..=6,
        ) {
            let position = Position::from_degrees(latitude, longitude).expect("on Earth");
            let reference = Mgrs::from_position(position, precision).expect("a reference");
            let square: MgrsSquare = reference.as_str().parse().expect("reads back");
            // Re-encoded in its own zone and band: a square of a widened
            // Svalbard zone may have its centre in the next standard zone, and
            // one across a band edge its centre in the next band.
            let again = Mgrs::from_coordinate(square.centre, latitude, precision)
                .expect("a reference");
            prop_assert_eq!(again.as_str(), reference.as_str());
        }
    }
}
