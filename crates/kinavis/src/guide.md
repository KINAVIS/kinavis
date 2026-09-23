Guide to the crate: frame-tagged angles, sailings, fixing, deviation tables and
the inverse problem, errors, serialisation, and memory footprint.

All examples are compiled and run as tests.

# Compass to true, and back

```rust
use kinavis::navigation_solutions::{
    convert_compass_course_to_true_course, convert_true_course_to_compass_course,
};
use kinavis::{CompassCourse, DeviationTable, InterpolationMethod, NavigationError, Variation};

fn main() -> Result<(), NavigationError> {
    // A swing: deviation observed on every tenth of the compass, 000° to 350°.
    let table = DeviationTable::from_deviations(&[
        -2.5, -0.5, 1.6, 4.4, -1.7, 0.0, 1.0, 0.3, -0.9,      // 000°..080°
        0.5, -1.2, 0.8, -0.3, 1.7, -2.1, 0.4, -0.6, 1.2,      // 090°..170°
        -1.3, 0.0, 0.9, -1.1, 1.5, -0.7, -13.2, -15.7, -17.9, // 180°..260°
        -19.2, -18.1, 1.8, -0.4, 0.7, -0.2, 1.4, -4.4, -2.9,  // 270°..350°
    ])?;

    let variation = Variation::new(-2.7)?; // 2.7° west

    // Steering 003° by compass: true course made good.
    let solution = convert_compass_course_to_true_course(
        CompassCourse::new(3.0)?,
        variation,
        &table,
        InterpolationMethod::Cubic,
    )?;

    assert_eq!(format!("{}", solution.course), "358.2°T");
    assert_eq!(format!("{:.4}", solution.deviation.degrees()), "-2.0665");

    // And back again.
    let back = convert_true_course_to_compass_course(
        solution.course,
        variation,
        &table,
        InterpolationMethod::Cubic,
    )?;
    assert!((back.course.degrees() - 3.0).abs() < 1e-9);

    Ok(())
}
```

# Frame-tagged angles

Every angle is a newtype tagged with its reference frame; mixing frames does not
compile:

```rust,compile_fail
use kinavis::navigation_solutions::magnetic_to_true;
use kinavis::{NavigationError, TrueCourse, Variation};

fn main() -> Result<(), NavigationError> {
    let true_course = TrueCourse::new(90.0)?;
    let variation = Variation::new(-3.0)?;

    // `magnetic_to_true` takes a MagneticCourse. This is a compile error rather
    // than a plausible-looking wrong answer.
    let _ = magnetic_to_true(true_course, variation);
    Ok(())
}
```

Types also enforce range — a direction is finite and in `[0°, 360°)` — so
corrections that add or subtract a known angle return a value, not a `Result`:

```rust
use kinavis::navigation_solutions::{compass_to_magnetic, magnetic_to_true};
use kinavis::{CompassCourse, Deviation, NavigationError, Variation};

fn main() -> Result<(), NavigationError> {
    let compass = CompassCourse::new(357.0)?;
    let deviation = Deviation::new(5.5)?;
    let variation = Variation::new(-2.0)?;

    // No `?` on these two: they cannot fail.
    let magnetic = compass_to_magnetic(compass, deviation);
    let true_course = magnetic_to_true(magnetic, variation);

    assert_eq!(format!("{}", magnetic), "002.5°M");
    assert_eq!(format!("{}", true_course), "000.5°T");

    // Out-of-range and non-finite input is rejected at construction instead.
    assert!(CompassCourse::new(400.0).is_err());
    assert!(Variation::new(f64::NAN).is_err());

    // ...`wrap` wraps explicitly.
    assert_eq!(CompassCourse::wrap(-10.0)?.degrees(), 350.0);
    Ok(())
}
```

| Type | Meaning | Range |
|---|---|---|
| `CompassCourse` / `CompassBearing` | ship's magnetic compass | `[0°, 360°)` |
| `MagneticCourse` / `MagneticBearing` | magnetic north | `[0°, 360°)` |
| `TrueCourse` / `TrueBearing` | true north | `[0°, 360°)` |
| `GyroCourse` / `GyroBearing` | gyrocompass | `[0°, 360°)` |
| `Variation` | true → magnetic north, east positive | `[-180°, 180°]` |
| `Deviation` | magnetic → compass north, east positive | `[-180°, 180°]` |
| `RelativeBearing` | clockwise from the ship's head | `[0°, 360°)` |
| `Angle` | plain magnitude: gyro error, sextant angle, leeway | any finite |
| `Latitude` / `Longitude` | north and east positive | `[-90°, 90°]` / `[-180°, 180°)` |
| `Distance` / `Speed` | stored in NM and kn | any finite |

Within a frame, courses and bearings share a type; the types prevent mixing
*frames*.

The gyrocompass has its own frame: its error is a single value, not a curve, and
depends on the ship's speed:

```rust
use kinavis::navigation_solutions::{gyro_error_from_transit, gyro_speed_error, gyro_to_true};
use kinavis::{GyroBearing, Latitude, NavigationError, Speed, TrueBearing, TrueCourse};

fn main() -> Result<(), NavigationError> {
    // Twenty knots due north in latitude 60°: the meridian is dragged west.
    let speed_error = gyro_speed_error(
        Latitude::from_degrees(60.0)?,
        TrueCourse::new(0.0)?,
        Speed::from_knots(20.0)?,
    )?;
    assert_eq!(format!("{speed_error:.2}"), "-2.54°");

    // And the total error, checked against a transit of known direction.
    let observed = GyroBearing::new(46.5)?;
    let charted = TrueBearing::new(45.0)?;
    let error = gyro_error_from_transit(observed, charted);
    assert_eq!(format!("{error:.1}"), "-1.5°");
    assert_eq!(gyro_to_true(observed, error).degrees(), 45.0);
    Ok(())
}
```

# Positions and sailings

```rust
use kinavis::sailings::{cross_track, geodesic, great_circle, great_circle_vertex, rhumb_line};
use kinavis::{NavigationError, Position, TrackSide};

fn main() -> Result<(), NavigationError> {
    // The Lizard to Cape Race.
    let from = Position::from_degrees(49.95, -5.20)?;
    let to = Position::from_degrees(46.66, -53.07)?;

    // One course the whole way, or the shortest track?
    let steered = rhumb_line(from, to)?;
    let direct = great_circle(from, to)?;

    assert_eq!(format!("{:.1}", steered.distance.nautical_miles()), "1921.0");
    assert_eq!(format!("{:.1}", direct.distance.nautical_miles()), "1889.1");

    // The rhumb line holds one course; the great circle does not.
    assert_eq!(format!("{:.1}", steered.initial_course.degrees()), "264.1");
    assert_eq!(format!("{:.1}", direct.initial_course.degrees()), "282.8");
    assert_eq!(format!("{:.1}", direct.final_course.degrees()), "246.1");

    // Its highest latitude, which is what limits a winter passage.
    let vertex = great_circle_vertex(from, direct.initial_course)?;
    assert_eq!(format!("{vertex}"), "51°08.1'N 021°43.1'W");

    // On the ellipsoid the same track is a little longer.
    assert_eq!(format!("{:.1}", geodesic(from, to)?.distance.nautical_miles()), "1894.6");

    // Cross-track error and distance to go on the great-circle track.
    let ship = Position::from_degrees(48.5, -30.0)?;
    let off = cross_track(ship, from, to)?;
    assert_eq!(off.side, TrackSide::Port);
    assert_eq!(format!("{:.1}", off.distance.nautical_miles()), "139.7");
    assert_eq!(format!("{:.0}", off.to_run.nautical_miles()), "927");
    Ok(())
}
```

Distances and speeds are types, so units are explicit:

```rust
use kinavis::{Distance, NavigationError, Speed};
use core::time::Duration;

fn main() -> Result<(), NavigationError> {
    let leg = Distance::from_nautical_miles(12.0)?;
    assert_eq!(format!("{:.0}", leg.metres()), "22224");
    assert_eq!(format!("{:.0}", leg.cables()), "120");

    let speed = Speed::from_knots(8.0)?;
    assert_eq!(speed.time_to_cover(leg)?, Duration::from_secs(5400));
    assert_eq!(speed.distance_covered(Duration::from_secs(3600)).nautical_miles(), 8.0);
    Ok(())
}
```

Positions parse from and format to standard notation:

```rust
use kinavis::{Latitude, NavigationError, Position};

fn main() -> Result<(), NavigationError> {
    let expected = Position::from_degrees(50.755, -1.2966667)?;

    for text in [
        "50°45.3'N 001°17.8'W",
        "50 45.3 N 001 17.8 W",
        "N50°45.3' W001°17.8'",
        "50.755, -1.2966667",
    ] {
        let position: Position = text.parse()?;
        assert!(position.latitude().degrees() - expected.latitude().degrees() < 1e-6);
    }

    assert_eq!(format!("{expected}"), "50°45.3'N 001°17.8'W");

    // Seconds work too, and a hemisphere that does not belong is refused.
    assert_eq!("50°45'18\"N".parse::<Latitude>()?.degrees(), 50.755);
    assert!("50°45.3'E".parse::<Latitude>().is_err());
    assert!("50 60.0 N".parse::<Latitude>().is_err()); // sixty minutes is the next degree
    Ok(())
}
```

# Passage planning

```rust
use kinavis::route::{LegKind, Route};
use kinavis::sailings::TrackSide;
use kinavis::{Distance, NavigationError, Position, Speed};

fn main() -> Result<(), NavigationError> {
    let route = Route::new(
        &[
            "50°06.0'N 001°30.0'W".parse::<Position>()?,
            "49°54.0'N 002°00.0'W".parse::<Position>()?,
            "49°42.0'N 002°45.0'W".parse::<Position>()?,
        ],
        LegKind::RhumbLine,
    )?;

    assert_eq!(route.leg_count(), 2);
    assert_eq!(format!("{:.1}", route.total_distance()?.nautical_miles()), "54.2");
    assert_eq!(route.passage_time(Speed::from_knots(10.0)?)?.as_secs() / 60, 325);

    // Underway: current leg, cross-track error, distance to go.
    let progress = route.progress("50°00.0'N 001°43.0'W".parse::<Position>()?)?;
    assert_eq!(progress.leg, 0);
    assert_eq!(progress.cross_track.side, TrackSide::Port);
    assert_eq!(format!("{:.1}", progress.cross_track.distance.nautical_miles()), "0.7");
    assert_eq!(format!("{:.0}", progress.distance_to_end.nautical_miles()), "44");

    // A great-circle route, broken into legs a ship can actually steer.
    let ocean = Route::new(
        &[
            Position::from_degrees(49.95, -5.20)?,
            Position::from_degrees(46.66, -53.07)?,
        ],
        LegKind::GreatCircle,
    )?;
    let steerable = ocean.split_legs(Distance::from_nautical_miles(300.0)?)?;
    assert_eq!(steerable.kind(), LegKind::RhumbLine);
    for leg in steerable.legs() {
        assert!(leg?.sailing.distance.nautical_miles() <= 300.0);
    }
    Ok(())
}
```

# Dead reckoning

```rust
use kinavis::dead_reckoning::{dead_reckoning, estimated_position};
use kinavis::navigation_solutions::Current;
use kinavis::{NavigationError, Position, Speed, TrueCourse};
use core::time::Duration;

fn main() -> Result<(), NavigationError> {
    let noon = Position::from_degrees(50.0, -5.0)?;
    let heading = TrueCourse::new(270.0)?;
    let speed = Speed::from_knots(12.0)?;
    let watch = Duration::from_secs(4 * 3600);

    // Course and distance alone.
    let reckoned = dead_reckoning(noon, heading, speed, watch)?;
    assert_eq!(format!("{reckoned}"), "50°00.0'N 006°14.6'W");

    // Allowing for a knot of north-going stream.
    let current = Current {
        set: TrueCourse::new(0.0)?,
        drift: Speed::from_knots(1.0)?,
    };
    let estimated = estimated_position(noon, heading, speed, current, watch)?;

    assert_eq!(format!("{}", estimated.position), "50°04.0'N 006°14.7'W");
    assert_eq!(format!("{:.1}", estimated.track.course_over_ground.degrees()), "274.8");
    assert_eq!(format!("{:.2}", estimated.track.speed_over_ground.knots()), "12.04");
    Ok(())
}
```

# Position fixing

Three bearings, one 3° off: the cocked hat opens and the residual shows it:

```rust
use kinavis::fix::{bearing_fix, cocked_hat, PositionLine};
use kinavis::{NavigationError, Position, TrueBearing};

fn main() -> Result<(), NavigationError> {
    let lighthouse = Position::from_degrees(50.20, -4.00)?;
    let headland = Position::from_degrees(50.20, -4.40)?;
    let buoy = Position::from_degrees(49.95, -4.15)?;

    let good = [
        PositionLine::from_bearing_of(lighthouse, TrueBearing::new(52.0)?),
        PositionLine::from_bearing_of(headland, TrueBearing::new(308.0)?),
        PositionLine::from_bearing_of(buoy, TrueBearing::new(167.9)?),
    ];
    let fix = bearing_fix(&good)?;
    assert_eq!(format!("{}", fix.position), "50°06.0'N 004°12.0'W");
    assert!(fix.rms_residual.nautical_miles() < 0.01);

    // Now spoil the third bearing.
    let mut spoiled = good;
    spoiled[2] = PositionLine::from_bearing_of(buoy, TrueBearing::new(170.9)?);

    let hat = cocked_hat(spoiled)?;
    assert_eq!(format!("{:.2}", hat.greatest_side.nautical_miles()), "0.78");
    assert!(bearing_fix(&spoiled)?.rms_residual.nautical_miles() > 0.1);
    Ok(())
}
```

Distance off without a rangefinder:

```rust
use kinavis::fix::{dipping_distance, distance_by_two_bearings, distance_by_vertical_angle};
use kinavis::{Angle, Distance, NavigationError, RelativeBearing};

fn main() -> Result<(), NavigationError> {
    // A light 80 m high subtending half a degree.
    let by_sextant = distance_by_vertical_angle(
        Distance::from_metres(80.0)?,
        Angle::from_minutes(30.0)?,
    )?;
    assert_eq!(format!("{by_sextant:.2}"), "4.95 M");

    // Doubling the angle on the bow: the run gives the distance off.
    let by_bearings = distance_by_two_bearings(
        RelativeBearing::new(30.0)?,
        RelativeBearing::new(60.0)?,
        Distance::from_nautical_miles(6.0)?,
    )?;
    assert_eq!(format!("{:.2}", by_bearings.at_second_bearing.nautical_miles()), "6.00");
    assert_eq!(format!("{:.2}", by_bearings.abeam.nautical_miles()), "5.20");

    // A 100 m light seen from a bridge 10 m up rises at 27.4 miles.
    let rising = dipping_distance(Distance::from_metres(10.0)?, Distance::from_metres(100.0)?)?;
    assert_eq!(format!("{rising:.2}"), "27.38 M");
    Ok(())
}
```

# Position from GNSS

Receiver sentences become a position with a quality estimate and events — fix
acquired, fix lost, observation rejected. Parsing is done by
[`kinavis-nmea0183`](https://crates.io/crates/kinavis-nmea0183), which depends
on the kernel only. Thresholds are vessel settings.

```rust
use std::time::Duration;
use kinavis::gnss_intake::{GnssIntake, IntakeConfig};
use kinavis::{GnssFix, NavigationEvent, Speed};
use kinavis_nmea0183::{parse, Sentence};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut intake = GnssIntake::new(IntakeConfig {
        max_age: Duration::from_secs(10),
        max_speed: Speed::from_knots(40.0)?,
    });

    let log = [
        "$GPRMC,120000.00,A,5045.3000,N,00120.0000,W,11.3,272.5,110926,,,D*72",
        "$GPRMC,120001.00,A,5045.3000,N,00120.0050,W,11.3,272.5,110926,,,D*76",
    ];
    for line in log {
        let Sentence::Rmc(rmc) = parse(line.as_bytes())? else { continue };
        let fix = GnssFix::try_from(rmc)?;
        let outcome = intake.accept(fix);
        for event in outcome.events() {
            if let NavigationEvent::FixAcquired { at, .. } = event {
                assert_eq!(format!("{at}"), "2026-09-11T12:00:00.000 UTC");
            }
        }
    }

    // Current position and staleness.
    let now = intake.last_fix().unwrap().taken_at().saturating_add(Duration::from_secs(2));
    let snapshot = intake.snapshot_at(now);
    let position = snapshot.position().unwrap();
    assert_eq!(format!("{:.1}", position.value()), "50°45.3'N 001°20.0'W");
    assert_eq!(snapshot.ground_track().unwrap().speed_over_ground.knots(), 11.3);
    assert!(!snapshot.is_stale());
    // A differential fix with no HDOP reported: the status is known, the
    // error bar is not.
    assert!(position.quality().status().is_usable());
    assert!(position.quality().sigma().is_none());
    Ok(())
}
```

# Collision avoidance

```rust
use kinavis::relative_motion::{
    closest_point_of_approach, course_for_cpa, Approach, Contact, Vessel,
};
use kinavis::{Distance, NavigationError, Speed, TrueBearing, TrueCourse};

fn main() -> Result<(), NavigationError> {
    let own = Vessel {
        course: TrueCourse::new(0.0)?,
        speed: Speed::from_knots(15.0)?,
    };
    // Fine on the starboard bow at nine miles, coming the other way.
    let contact = Contact {
        bearing: TrueBearing::new(15.0)?,
        range: Distance::from_nautical_miles(9.0)?,
    };
    let target = Vessel {
        course: TrueCourse::new(200.0)?,
        speed: Speed::from_knots(12.0)?,
    };

    let Approach::Closing(cpa) = closest_point_of_approach(own, contact, target)? else {
        panic!("she is closing");
    };
    assert_eq!(format!("{:.2}", cpa.distance.nautical_miles()), "0.96");
    assert_eq!(format!("{:.0}", cpa.time_to_go.as_secs_f64() / 60.0), "20");

    // Two miles would be more comfortable. What course gives it?
    let avoidance = course_for_cpa(own, contact, target, Distance::from_nautical_miles(2.0)?)?;
    let starboard = avoidance.starboard.expect("an alteration to starboard exists");
    assert_eq!(format!("{:.1}", starboard.degrees()), "34.1");

    // And it really does: the answer is checked, not asserted.
    let after = closest_point_of_approach(
        Vessel { course: starboard, speed: own.speed },
        contact,
        target,
    )?;
    let Approach::Closing(after) = after else {
        panic!("still closing, just further off");
    };
    assert!((after.distance.nautical_miles() - 2.0).abs() < 1e-9);
    Ok(())
}
```

# Deviation tables

A table can be built from a full swing, arbitrary headings, or a fixed step:

```rust
use kinavis::{CardinalPoint, Deviation, DeviationTable, NavigationError};

fn main() -> Result<(), NavigationError> {
    // 36 values, 000° to 350°.
    let _swing = DeviationTable::from_deviations(&[0.0; 36])?;

    // Arbitrary headings. Negative and over-360 courses normalise properly.
    let _sparse = DeviationTable::from_pairs(&[(0, -2.5), (-270, 1.0), (180, 0.4)])?;

    // A fixed step, or the eight cardinal points.
    let _every_ten = DeviationTable::from_step(10)?;
    let mut cardinal = DeviationTable::from_cardinal_directions();

    // A compass point is a type, so it cannot be misspelled.
    cardinal.set_deviation_at(CardinalPoint::NE, Deviation::new(1.5)?)?;
    assert_eq!(cardinal.deviation_at_point(CardinalPoint::NE)?.degrees(), 1.5);

    // Bad input is rejected rather than silently patched up.
    assert!(DeviationTable::from_step(0).is_err());
    assert!(DeviationTable::from_pairs(&[]).is_err());
    assert!(DeviationTable::from_deviations(&[0.0; 12]).is_err()); // not a full swing
    Ok(())
}
```

Or from the swing observations directly: deviation is not measured, but derived
from compass bearings of an object with known true bearing, taken on each
heading.

```rust
use kinavis::{
    CompassBearing, CompassCourse, DeviationTable, NavigationError, SwingObservation,
    TrueBearing, Variation,
};

fn main() -> Result<(), NavigationError> {
    let variation = Variation::new(-2.0)?;
    let transit = TrueBearing::new(45.0)?; // charted direction of the transit

    let observations = [(0.0, 48.5), (90.0, 46.0), (180.0, 45.5), (270.0, 48.0)]
        .into_iter()
        .map(|(heading, observed)| {
            Ok(SwingObservation {
                compass_heading: CompassCourse::new(heading)?,
                observed_bearing: CompassBearing::new(observed)?,
                reference_bearing: transit,
            })
        })
        .collect::<Result<Vec<_>, NavigationError>>()?;

    let table = DeviationTable::from_swing(&observations, variation)?;

    // On north the compass called the transit 048.5 when it is really 045.0,
    // with 2°W variation: deviation is 045.0 − (−2.0) − 048.5 = −1.5°.
    assert_eq!(table.deviation_at_node(0).unwrap().degrees(), -1.5);
    assert_eq!(table.deviation_at_node(90).unwrap().degrees(), 1.0);
    Ok(())
}
```

## Interpolation

| Method | Continuity | Nodes needed | Use when |
|---|---|---|---|
| `Linear` (default) | C⁰ | 2 | values must never overshoot the table |
| `ShapePreserving` | C¹ | 2 | smooth curve without overshoot |
| `Cubic` | C² | 3 | dense swing, smoothest curve |
| `Parametric` | analytic | 5 | classical A–E model, or smoothing a noisy swing |

A cubic spline can overshoot the data at an abrupt step, producing deviations
never observed; `ShapePreserving` (Fritsch–Carlson) trades a little smoothness
to prevent it:

```rust
use kinavis::{CompassCourse, DeviationTable, InterpolationMethod, NavigationError};

fn main() -> Result<(), NavigationError> {
    let table = DeviationTable::from_deviations(&[
        -2.5, -0.5, 1.6, 4.4, -1.7, 0.0, 1.0, 0.3, -0.9,
        0.5, -1.2, 0.8, -0.3, 1.7, -2.1, 0.4, -0.6, 1.2,
        -1.3, 0.0, 0.9, -1.1, 1.5, -0.7, -13.2, -15.7, -17.9,
        -19.2, -18.1, 1.8, -0.4, 0.7, -0.2, 1.4, -4.4, -2.9,
    ])?;

    // Between the 270° node (−19.2) and the 280° node (−18.1) the spline dips to
    // −20.3, a deviation this compass was never observed to have.
    let course = CompassCourse::new(273.0)?;
    let spline = table.deviation_at(course, InterpolationMethod::Cubic)?;
    let shaped = table.deviation_at(course, InterpolationMethod::ShapePreserving)?;

    assert_eq!(format!("{:.2}", spline.degrees()), "-20.31");
    assert_eq!(format!("{:.2}", shaped.degrees()), "-19.09");

    // The shape-preserving curve stays between the two nodes, as it must.
    assert!(shaped.degrees() >= -19.2 && shaped.degrees() <= -18.1);
    Ok(())
}
```

All four are **periodic**: the arc from the last node through 360°/0° to the
first is a real interval, not a flat extrapolation:

```rust
use kinavis::{CompassCourse, Deviation, DeviationTable, InterpolationMethod, NavigationError};

fn main() -> Result<(), NavigationError> {
    let mut table = DeviationTable::from_step(10)?;
    table.set_deviation(350, Deviation::new(10.0)?)?;
    table.set_deviation(0, Deviation::new(-10.0)?)?;

    // Halfway round the 350°->000° arc, the deviation is halfway between.
    let midpoint = table.deviation_at(CompassCourse::new(355.0)?, InterpolationMethod::Linear)?;
    assert!(midpoint.degrees().abs() < 1e-12);
    Ok(())
}
```

`Parametric` fits

```text
δ = A + B·sin(y) + C·cos(y) + D·sin(2y) + E·cos(2y)
```

by least squares. Supplied coefficients are held fixed and the rest fitted
around them:

```rust
use kinavis::{
    CompassCourse, DeviationCoefficients, DeviationTable, Interpolation, InterpolationMethod,
    NavigationError,
};

fn main() -> Result<(), NavigationError> {
    // A swing that is exactly 5°·sin(course).
    let values: Vec<f64> = (0..36)
        .map(|index| 5.0 * (f64::from(index) * 10.0).to_radians().sin())
        .collect();
    let table = DeviationTable::from_deviations(&values)?;

    let east = CompassCourse::new(90.0)?;
    let fitted = table.deviation_at(east, InterpolationMethod::Parametric)?;
    assert!((fitted.degrees() - 5.0).abs() < 1e-9);

    // Hold the constant term at 1° and fit B..E around it.
    let coefficients = DeviationCoefficients { a: Some(1.0), ..Default::default() };
    let pinned = table.deviation_at(
        east,
        Interpolation {
            method: InterpolationMethod::Parametric,
            coefficients: Some(&coefficients),
        },
    )?;
    assert!((pinned.degrees() - 6.0).abs() < 1e-9);
    Ok(())
}
```

# Inverse problem

Deviation is tabulated against the **compass** course, so converting true to
compass requires solving

```text
CC + δ(CC) = MC
```

for `CC` — an implicit equation, not a subtraction. `kinavis` solves it, so both
directions agree to the solver tolerance:

```rust
use kinavis::navigation_solutions::{
    convert_compass_course_to_true_course, convert_true_course_to_compass_course,
};
use kinavis::{
    CompassCourse, DeviationTable, InterpolationMethod, NavigationError, SmithCoefficients,
    Variation,
};

fn main() -> Result<(), NavigationError> {
    // A smooth, well-behaved swing.
    let model = SmithCoefficients { a: 2.0, b: 3.0, c: -4.0, d: 1.5, e: -0.5 };
    let values: Vec<f64> = (0..36)
        .map(|index| {
            let course = CompassCourse::new(f64::from(index) * 10.0)?;
            Ok(model.deviation_at(course))
        })
        .collect::<Result<_, NavigationError>>()?;
    let table = DeviationTable::from_deviations(&values)?;
    let variation = Variation::new(-2.7)?;

    let mut worst: f64 = 0.0;
    let mut course = 0.0;
    while course < 360.0 {
        let compass = CompassCourse::new(course)?;
        let out = convert_compass_course_to_true_course(
            compass, variation, &table, InterpolationMethod::Cubic,
        )?;
        let back = convert_true_course_to_compass_course(
            out.course, variation, &table, InterpolationMethod::Cubic,
        )?;
        worst = worst.max(back.course.angular_distance(compass));
        course += 0.5;
    }

    assert!(worst < 1e-8, "worst round-trip error was {worst}");
    Ok(())
}
```

## Non-invertible swings

If deviation changes by more than 1° per degree of heading, two compass courses
map to the same magnetic course and the inverse is not unique. This is detected
and reported:

```rust
use kinavis::navigation_solutions::convert_true_course_to_compass_course;
use kinavis::{DeviationTable, InterpolationMethod, NavigationError, TrueCourse, Variation};

fn main() -> Result<(), NavigationError> {
    // This sample swing jumps 12.5° between 230° and 240°.
    let table = DeviationTable::from_deviations(&[
        -2.5, -0.5, 1.6, 4.4, -1.7, 0.0, 1.0, 0.3, -0.9,
        0.5, -1.2, 0.8, -0.3, 1.7, -2.1, 0.4, -0.6, 1.2,
        -1.3, 0.0, 0.9, -1.1, 1.5, -0.7, -13.2, -15.7, -17.9,
        -19.2, -18.1, 1.8, -0.4, 0.7, -0.2, 1.4, -4.4, -2.9,
    ])?;

    assert!(!table.is_invertible());
    assert!((table.max_slope() - 1.99).abs() < 0.01); // degrees of δ per degree of heading

    let solution = convert_true_course_to_compass_course(
        TrueCourse::new(256.0)?,
        Variation::new(0.7)?,
        &table,
        InterpolationMethod::Linear,
    )?;

    // The answer is still correct — steering it does make 256°T good — but the
    // advisory says it is not the only compass course that would.
    assert_eq!(format!("{:.2}", solution.course.degrees()), "274.05");
    assert!(solution.advisories.non_invertible_table);
    Ok(())
}
```

# Swing analysis

```rust
use kinavis::deviation::analyze;
use kinavis::{DeviationTable, NavigationError};

fn main() -> Result<(), NavigationError> {
    let table = DeviationTable::from_deviations(&[
        -2.5, -0.5, 1.6, 4.4, -1.7, 0.0, 1.0, 0.3, -0.9,
        0.5, -1.2, 0.8, -0.3, 1.7, -2.1, 0.4, -0.6, 1.2,
        -1.3, 0.0, 0.9, -1.1, 1.5, -0.7, -13.2, -15.7, -17.9,
        -19.2, -18.1, 1.8, -0.4, 0.7, -0.2, 1.4, -4.4, -2.9,
    ])?;

    let analysis = analyze(&table)?;

    assert_eq!(format!("{:.4}", analysis.coefficients.a), "-2.4083");
    assert_eq!(format!("{:.4}", analysis.coefficients.b), "4.5557");
    assert_eq!(analysis.nodes, 36);
    assert_eq!(format!("{:.1}", analysis.max_gap), "10.0");

    // An RMS residual this large means the classical five-coefficient model does
    // not describe this compass — which, for a swing with a 12.5° step in it, is
    // exactly the right conclusion.
    assert!(analysis.rms_residual > 4.0);
    Ok(())
}
```

# Current triangle

```rust
use kinavis::navigation_solutions::{course_over_ground, course_to_steer, estimate_current};
use kinavis::{NavigationError, Speed, TrueCourse};

fn main() -> Result<(), NavigationError> {
    let heading = TrueCourse::new(0.0)?; // steering due north
    let set = TrueCourse::new(90.0)?;    // current setting due east
    let speed = Speed::from_knots(10.0)?;
    let drift = Speed::from_knots(2.0)?;

    // Course and speed made good.
    let track = course_over_ground(heading, speed, set, drift)?;
    assert_eq!(format!("{:.2}", track.course_over_ground.degrees()), "11.31");
    assert_eq!(format!("{:.2}", track.speed_over_ground.knots()), "10.20");

    // Course to steer to make good due north.
    let steering = course_to_steer(TrueCourse::new(0.0)?, speed, set, drift)?;
    assert_eq!(format!("{:.2}", steering.heading.degrees()), "348.46");
    assert_eq!(format!("{:.2}", steering.speed_over_ground.knots()), "9.80");

    // What current explains the difference between water track and ground track?
    let current = estimate_current(
        heading,
        speed,
        track.course_over_ground,
        track.speed_over_ground,
    )?;
    assert!(current.set.angular_distance(set) < 1e-9);
    assert!((current.drift.knots() - drift.knots()).abs() < 1e-9);

    // A current the ship cannot outrun is reported, not approximated.
    assert!(course_to_steer(
        TrueCourse::new(0.0)?,
        Speed::from_knots(2.0)?,
        set,
        Speed::from_knots(10.0)?,
    )
    .is_err());
    Ok(())
}
```

# Advisories and error estimates

Every conversion returns its inputs, an interpolation uncertainty estimate, and
advisories with documented thresholds:

```rust
use kinavis::navigation_solutions::convert_compass_course_to_true_course;
use kinavis::{CompassCourse, DeviationTable, InterpolationMethod, NavigationError, Variation};

fn main() -> Result<(), NavigationError> {
    let table = DeviationTable::from_deviations(&[
        -2.5, -0.5, 1.6, 4.4, -1.7, 0.0, 1.0, 0.3, -0.9,
        0.5, -1.2, 0.8, -0.3, 1.7, -2.1, 0.4, -0.6, 1.2,
        -1.3, 0.0, 0.9, -1.1, 1.5, -0.7, -13.2, -15.7, -17.9,
        -19.2, -18.1, 1.8, -0.4, 0.7, -0.2, 1.4, -4.4, -2.9,
    ])?;

    let solution = convert_compass_course_to_true_course(
        CompassCourse::new(270.0)?,
        Variation::new(-20.0)?,
        &table,
        InterpolationMethod::Linear,
    )?;

    assert!(solution.advisories.large_variation);      // |variation| > 15°
    assert!(solution.advisories.large_deviation);      // |deviation| > 10°
    assert!(solution.advisories.non_invertible_table); // δ changes faster than 1°/1°
    assert!(!solution.advisories.coarse_table);        // widest node gap > 45°
    assert!(solution.check_data_required());

    // Estimated interpolation uncertainty, in degrees.
    assert!(solution.estimated_error.is_finite());
    Ok(())
}
```

`estimated_error` is the classical interpolation error bound for `Linear` and
`Cubic`, and the RMS fit residual for `Parametric`. It covers interpolation
only, not the quality of the swing.

# Errors

Two nested error types. Value types and numeric primitives return `KernelError`
(out of range, buffer too small, undefined quantity, missing input). Algorithms
return `NavigationError`, with their own variants (duplicate deviation entry,
non-intersecting rhumb lines) and kernel errors wrapped in
`NavigationError::Kernel`. Both are `#[non_exhaustive]`: match with a wildcard
arm.

```rust
use kinavis::{CompassCourse, DeviationTable, KernelError, NavigationError};

fn main() {
    let error = DeviationTable::from_step(0).unwrap_err();
    assert_eq!(error, NavigationError::InvalidStep { step: 0 });
    assert_eq!(
        error.to_string(),
        "invalid deviation table step: 0. Must be between 1 and 180 degrees"
    );

    // A value type refuses in the kernel's vocabulary.
    match CompassCourse::new(400.0) {
        Err(KernelError::OutOfRange { value, max, .. }) => {
            assert_eq!(value, 400.0);
            assert_eq!(max, 360.0);
        }
        _ => panic!("400° is out of range"),
    }

    // Inside an algorithm the same refusal comes back wrapped, and `?`
    // does the wrapping.
    fn table_at(degrees: f64) -> Result<(), NavigationError> {
        let course = CompassCourse::new(degrees)?;
        let table = DeviationTable::from_step(10)?;
        let _ = table.deviation_at(course, kinavis::InterpolationMethod::Linear)?;
        Ok(())
    }
    assert!(matches!(
        table_at(400.0),
        Err(NavigationError::Kernel(KernelError::OutOfRange { .. }))
    ));
}
```

No panics on caller data: `NaN`, infinities, degenerate tables and extreme
magnitudes return errors. Enforced by lints (`clippy::unwrap_used`,
`expect_used`, `panic`, `indexing_slicing` denied; `unsafe_code` forbidden),
tested with hostile input in `tests/robustness.rs`, and verified by
`ci/panic-free.py`, which inspects the bare-metal LLVM IR twice: as shipped, and
with integer overflow checks on, so an unchecked `+` on a length or offset
cannot hide as a wrong value in release. Threat model and vulnerability
reporting:
[SECURITY.md](https://github.com/KINAVIS/kinavis/blob/main/SECURITY.md).

# Serialisation

With the `serde` feature, deserialisation applies construction-time validation,
so stored data cannot contain a latitude of 500° or duplicate deviation
headings:

```toml
kinavis = { version = "1", features = ["serde"] }
```

```rust,ignore
let route: Route = serde_json::from_str(&plan)?;      // checked on the way in
assert!(serde_json::from_str::<Latitude>("500.0").is_err());
assert!(serde_json::from_str::<DeviationTable>("[]").is_err());
```

This applies to every type with an invariant: a quaternion deserialises as a
unit or fails, an ellipsoid with a zero axis is rejected, a CAN frame cannot
claim nine bytes. `ci/serde-guard.py` fails the build if a type with a private
numeric field derives `Deserialize` without going through its constructor.

# `no_std` without an allocator

```toml
kinavis = { version = "1", default-features = false, features = ["libm"] }
```

This configuration has no `extern crate alloc`, so the compiler guarantees no
heap use. All storage is inline with public bounds: `MAX_TABLE_NODES` nodes per
deviation table, `MAX_WAYPOINTS` per route, `EXCERPT_BYTES` of input per error.
Exceeding them returns `CapacityExceeded`; memory use is known at compile time.
Aggregates are therefore large: pass `DeviationTable` and `Route` by reference.

Functions that would return a `Vec` write into a caller-owned slice
(`interpolate_deviation`, `great_circle_waypoints_into`); the `alloc` feature
(default, implied by `std`) adds `Vec`-returning companions.

CI builds for `thumbv7em-none-eabihf` on every commit and checks the LLVM IR for
`core::panicking` calls.

# Memory footprint

Inline storage is paid at declaration, not at use: a `Traffic` picture is ~15
KiB whether it tracks one target or 32. Budgets, enforced by
`tests/footprint.rs` in each crate (exceeding one fails the build until raised
deliberately):

| Type | Budget | Holds |
|---|---|---|
| `kinavis_traffic::Traffic` | 16 KiB | `MAX_TARGETS` tracks of `MAX_TRACK_HISTORY` fixes |
| `kinavis::estimator::Estimator<_>` | 9 KiB | `MAX_HISTORY` beliefs, for late observations |
| `kinavis_traffic::CollisionPicture` | 4 KiB | one assessment per target |
| `kinavis_alerts::AlertManager<_>` | 3.5 KiB | `MAX_ALERTS` standing alerts |
| `kinavis_traffic::TrafficView` | 3 KiB | one entry per target |
| `kinavis::Route`, `RouteSchedule` | 2.5 KiB | `MAX_WAYPOINTS` positions |
| `kinavis_nmea2000::Assembler`, `kinavis_ins::InsFilter` | 2.5 KiB | fast packets in flight; INS state and covariance |
| `kinavis::DeviationTable` | 1.25 KiB | `MAX_TABLE_NODES` nodes |
| `kinavis::NavigationSnapshot` | 192 B | what a display reads every tick |
| `NavigationEvent`, `NavigationError`, every event and error | 64 B | one cache line |

Rule: types over 1 KiB live behind a reference or in a `static` and are not
passed by value on hot paths; `Traffic` and `AlertManager` are deliberately not
`Copy`. Projections (`TrafficView`, `GuidanceView`, `NavigationSnapshot`) cross
boundaries; aggregates stay put. On bare metal, size task stacks from this table
or place aggregates in `static` storage.

# Models

Models agreeing to three figures and differing in the fourth are documented per
function:

| Computation | Model |
|---|---|
| [`sailings::rhumb_line`], [`sailings::great_circle`] | sphere of mean radius 6371.0088 km |
| [`sailings::geodesic`] | WGS-84 ellipsoid, Vincenty |
| [`Latitude::meridional_parts`] | WGS-84 ellipsoid |
| Position lines and their crossings | rhumb lines, exact on a Mercator chart |
| Bearing fixes | least squares in Mercator coordinates |
| Range fixes | azimuthal equidistant plane about the observer |
| Relative motion | plane |

[`sailings::rhumb_line`]: https://docs.rs/kinavis/latest/kinavis/sailings/fn.rhumb_line.html
[`sailings::great_circle`]: https://docs.rs/kinavis/latest/kinavis/sailings/fn.great_circle.html
[`sailings::geodesic`]: https://docs.rs/kinavis/latest/kinavis/sailings/fn.geodesic.html
[`Latitude::meridional_parts`]: https://docs.rs/kinavis/latest/kinavis/position/struct.Latitude.html#method.meridional_parts
