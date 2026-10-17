//! A tractor's work, read back from its log: the AB line, the passes, where
//! the implement was, what it worked, how it turned.
//!
//! A Fendt 820 with a 4.2 m cultivator (Kerner Komet K420), deep
//! cultivation near Munich: an excerpt of TUM's "Agricultural Load Cycles"
//! (doi:10.5281/zenodo.14619787, CC BY 4.0), 28 minutes at 5 Hz, twelve
//! passes back and forth with the turns between them.
//!
//! The AB line is the first pass. The logged position steps across
//! alternately about 2.4 m and 5.3 m from pass to pass: the implement is not
//! under it. The passes themselves give the implement's place, the one
//! offset to the side that spaces them equally. Laid out from the logged
//! position, the coverage raster leaves a strip unworked at every other
//! seam; from the implement's place, almost none. Every headland turn backs
//! up once: shorter than any turn without reversing.
//!
//! ```text
//! cargo run -p kinavis-examples --example field
//! ```

use core::time::Duration as Elapsed;
use std::error::Error;

use kinavis::region::PlanarPoint;
use kinavis_agri::{
    ground_point, AbLine, Boom, CoverageGrid, FieldPlane, LeverArm, PassIndex, PassPattern,
    PassTracker, Pose, ReferenceLine, Tilt, TrackerConfig, TurnPath,
};
use kinavis_kernel::{Civil, Distance, GridCourse, Height, Instant, Position, Utc};

const LOG: &str = include_str!("../data/tum-fendt820-field5.csv");

/// Working width of the K420.
const WIDTH: f64 = 4.2;

/// Height of the field above the ellipsoid: 571 m above sea level in the
/// log's altitude column, and about 47 m of geoid near Munich.
const ELLIPSOIDAL_HEIGHT: f64 = 618.0;

/// Turning radius of the shortest forward-only turn to compare with: a
/// tractor of this size with a mounted implement.
const TURN_RADIUS: f64 = 6.0;

/// Fixes dropped at each end of a pass, 5 s at 5 Hz: the implement going in
/// and out.
const SETTLE: usize = 25;

/// Raster cell, metres.
const CELL: f64 = 0.2;

/// Raster: `64 · 12` cells east by 1800 north, 154 m by 360 m at 20 cm.
type Raster = CoverageGrid<12, 1800>;

type Outcome<T> = Result<T, Box<dyn Error>>;

/// One line of the log, in the field plane.
struct Fix {
    seconds: f64,
    point: PlanarPoint,
    working: bool,
}

/// A run of the implement in work: the fixes `first..=last`.
#[derive(Clone, Copy)]
struct Pass {
    first: usize,
    last: usize,
}

/// Where a pass ran, by the logged position.
#[derive(Clone, Copy)]
struct Run {
    /// Mean offset from the AB line, left positive, metres.
    offset: f64,
    /// Driven from A towards B.
    forward: bool,
}

/// Spacing of the passes and the implement's place, fitted to the runs.
struct Fit {
    /// Offset from one pass to the next, metres; negative when the work
    /// moves right of the line.
    step: f64,
    /// Implement right of the logged position, metres.
    right: f64,
    /// Offset of the implement on the first pass, metres.
    shift: f64,
}

fn main() -> Outcome<()> {
    let (plane, fixes) = read(LOG)?;
    let passes = passes(&fixes)?;
    let minutes = fixes.last().map_or(0.0, |fix| fix.seconds) / 60.0;
    println!(
        "{} fixes over {minutes:.0} min, {} passes with the implement in work",
        fixes.len(),
        passes.len()
    );

    // The AB line: the first pass, start to end.
    let reference = *passes.first().ok_or("no pass")?;
    let interior = inside(&fixes, reference)?;
    let a = interior.first().ok_or("a short pass")?.point;
    let b = interior.last().ok_or("a short pass")?.point;
    let line = AbLine::from_points(plane, a, b)?;
    println!(
        "AB line on the first pass: {:.1} m, grid course {:.2}°\n",
        a.distance_to(b).metres(),
        line.course().degrees()
    );

    let runs = runs(&fixes, &passes, &line)?;
    let fit = fit(&runs).ok_or("too few passes to fit")?;
    let side = if fit.right >= 0.0 { "right" } else { "left" };
    println!(
        "\nSpacing {:.2} m: the {WIDTH} m implement overlaps {:.2} m. The logged steps \
         alternate because the implement works {:.2} m {side} of the logged position.",
        fit.step.abs(),
        WIDTH - fit.step.abs(),
        fit.right.abs()
    );

    let pattern = PassPattern::new(
        Distance::from_metres(WIDTH)?,
        Distance::from_metres(WIDTH - fit.step.abs())?,
    )?
    .with_shift(Distance::from_metres(fit.shift)?)?;
    track(&fixes, &passes, &line, &pattern, &fit)?;
    coverage(&fixes, &passes, &line, &pattern, &fit, (a, b))?;
    turns(&plane, &fixes, &passes)
}

/// The field plane, centred on the first fix, and every fix in it.
fn read(log: &str) -> Outcome<(FieldPlane, Vec<Fix>)> {
    let mut plane = None;
    let mut fixes = Vec::new();
    for line in log.lines().filter(|line| !line.starts_with('#')) {
        let fields: Vec<&str> = line.split(',').collect();
        let [seconds, latitude, longitude, _speed, working] = fields[..] else {
            return Err(format!("not five fields: {line}").into());
        };
        let position = Position::from_degrees(latitude.parse()?, longitude.parse()?)?;
        let plane = match plane {
            Some(plane) => plane,
            None => *plane.insert(FieldPlane::centred_at(
                position,
                Height::above_ellipsoid(Distance::from_metres(ELLIPSOIDAL_HEIGHT)?),
            )?),
        };
        fixes.push(Fix {
            seconds: seconds.parse()?,
            point: plane.to_plane(position)?,
            working: working == "1",
        });
    }
    Ok((plane.ok_or("an empty log")?, fixes))
}

/// Runs of the implement in work longer than 100 m.
fn passes(fixes: &[Fix]) -> Outcome<Vec<Pass>> {
    let mut passes = Vec::new();
    let mut start = None;
    for (i, fix) in fixes.iter().enumerate() {
        match (fix.working, start) {
            (true, None) => start = Some(i),
            (false, Some(first)) => {
                let last = i.saturating_sub(1);
                let ends = (at(fixes, first)?.point, at(fixes, last)?.point);
                if ends.0.distance_to(ends.1).metres() > 100.0 {
                    passes.push(Pass { first, last });
                }
                start = None;
            }
            _ => {}
        }
    }
    Ok(passes)
}

/// Where each pass ran by the logged position, printed with the step from
/// the one before.
fn runs(fixes: &[Fix], passes: &[Pass], line: &AbLine) -> Outcome<Vec<Run>> {
    println!("pass  course   length   offset   sd    step");
    let mut runs: Vec<Run> = Vec::new();
    for (k, pass) in passes.iter().enumerate() {
        let interior = inside(fixes, *pass)?;
        let offsets: Vec<f64> = interior
            .iter()
            .map(|fix| line.station(fix.point).offset.metres())
            .collect();
        let (mean, sd) = mean_sd(&offsets);
        let start = interior.first().ok_or("a short pass")?.point;
        let end = interior.last().ok_or("a short pass")?.point;
        let forward = line
            .station(start)
            .direction
            .angular_distance(course(start, end)?)
            < 90.0;
        let step = runs.last().map_or(String::new(), |before| {
            format!("{:+6.2}", mean - before.offset)
        });
        println!(
            "{k:4}  {:>6}  {:5.0} m  {mean:+6.2}  {sd:4.2}  {step}",
            if forward { "AB" } else { "BA" },
            start.distance_to(end).metres(),
        );
        runs.push(Run {
            offset: mean,
            forward,
        });
    }
    Ok(runs)
}

/// Equal steps `w` for the implement at `r` right of the logged position:
/// the logged offset of pass `k` is `s + k·w + r` on a pass driven along the
/// line, `s + k·w − r` against it. `w` from passes two apart, driven the same
/// way; then `s + r` and `s − r` from the passes of each direction.
fn fit(runs: &[Run]) -> Option<Fit> {
    let pairs: Vec<f64> = runs
        .iter()
        .zip(runs.iter().skip(2))
        .map(|(before, after)| (after.offset - before.offset) / 2.0)
        .collect();
    let step = mean_sd(&pairs).0;
    let level = |forward: bool| {
        let rest: Vec<f64> = runs
            .iter()
            .zip(0u32..)
            .filter(|(run, _)| run.forward == forward)
            .map(|(run, k)| run.offset - f64::from(k) * step)
            .collect();
        (!rest.is_empty()).then(|| mean_sd(&rest).0)
    };
    let (along, against) = (level(true)?, level(false)?);
    step.is_finite().then_some(Fit {
        step,
        right: (along - against) / 2.0,
        shift: f64::midpoint(along, against),
    })
}

/// Pass by pass, the implement's place against its pass.
fn track(
    fixes: &[Fix],
    passes: &[Pass],
    line: &AbLine,
    pattern: &PassPattern,
    fit: &Fit,
) -> Outcome<()> {
    let config = TrackerConfig::new(
        Distance::from_metres(0.5)?,
        Distance::from_metres(20.0)?,
        Distance::from_metres(0.5)?,
    )?;
    // The antenna relative to the implement, the control point here.
    let lever = LeverArm {
        forward: Distance::ZERO,
        right: Distance::from_metres(-fit.right)?,
        up: Distance::ZERO,
    };
    let mut tracker = PassTracker::new(config);
    let start = Instant::<Utc>::from_civil(Civil::date(2026, 1, 1))?;
    println!("\npass  number  off its pass   sd");
    for (k, pass) in passes.iter().enumerate() {
        let mut errors = Vec::new();
        let mut number = 0;
        for i in pass.first + SETTLE..=pass.last.saturating_sub(SETTLE) {
            let here = pose(fixes, i)?;
            let implement = ground_point(here.position, here.course, Tilt::LEVEL, lever)?;
            let now = start
                .checked_add(Elapsed::from_secs_f64(at(fixes, i)?.seconds))
                .ok_or("a time past the end of the calendar")?;
            let (view, _) = tracker.update(line, pattern, implement, here.course, now);
            errors.push(view.cross_track.metres());
            number = view.pass.number();
        }
        let (mean, sd) = mean_sd(&errors);
        println!("{k:4}  {number:6}  {mean:+9.2} m  {sd:4.2}");
    }
    Ok(())
}

/// What was worked, laid out from the logged position and from the
/// implement's place.
fn coverage(
    fixes: &[Fix],
    passes: &[Pass],
    line: &AbLine,
    pattern: &PassPattern,
    fit: &Fit,
    ends: (PlanarPoint, PlanarPoint),
) -> Outcome<()> {
    let low = |coordinate: fn(PlanarPoint) -> f64| {
        fixes
            .iter()
            .map(|fix| coordinate(fix.point))
            .fold(f64::INFINITY, f64::min)
            - 10.0
    };
    let corner = PlanarPoint::from_metres(
        low(PlanarPoint::east_metres),
        low(PlanarPoint::north_metres),
    )?;
    let boom = Boom::new([Distance::from_metres(WIDTH)?], Distance::ZERO)?;
    let towards = if fit.step < 0.0 { -1 } else { 1 };
    println!("\ncoverage              worked    twice    left unworked between passes");
    for (label, boom) in [
        ("from the logged fix", boom),
        (
            "from the implement",
            boom.offset_right(Distance::from_metres(fit.right)?)?,
        ),
    ] {
        let mut raster = Box::new(Raster::new(
            *line.plane(),
            corner,
            Distance::from_metres(CELL)?,
        )?);
        for pass in passes {
            for i in pass.first + 1..=pass.last {
                raster.mark(&boom, pose(fixes, i - 1)?, pose(fixes, i)?, [true]);
            }
        }
        let unworked = seams_unworked(&raster, line, pattern, passes.len(), towards, ends);
        println!(
            "{label:20}  {:6.0} m²  {:5.0} m²  {unworked:5} m of seam",
            raster.worked_area(),
            raster.overlap_area(),
        );
    }
    Ok(())
}

/// Metres of seam between neighbouring passes left unworked, sampled every
/// metre from A to B less 10 m at each end; `towards` is the step in pass
/// number from one pass to the next.
fn seams_unworked(
    raster: &Raster,
    line: &AbLine,
    pattern: &PassPattern,
    count: usize,
    towards: i32,
    (a, b): (PlanarPoint, PlanarPoint),
) -> u32 {
    let first = pattern.pass_at(line.station(a).offset).number();
    let length = a.distance_to(b).metres();
    let (north, east) = line.course().components(1.0);
    let mut unworked = 0;
    let mut pass = first;
    for _ in 1..count {
        let seam = f64::midpoint(
            pattern.offset_of(PassIndex::new(pass)).metres(),
            pattern.offset_of(PassIndex::new(pass + towards)).metres(),
        );
        let mut along = 10.0;
        while along < length - 10.0 {
            // Left of the line is a quarter turn anticlockwise of its course.
            let point = PlanarPoint::from_metres(
                a.east_metres() + along * east - seam * north,
                a.north_metres() + along * north + seam * east,
            );
            if point.is_ok_and(|point| !raster.is_worked(point)) {
                unworked += 1;
            }
            along += 1.0;
        }
        pass += towards;
    }
    unworked
}

/// The headland turns, against the shortest forward-only turn; a change of
/// direction is the track turning back on itself within a second.
fn turns(plane: &FieldPlane, fixes: &[Fix], passes: &[Pass]) -> Outcome<()> {
    let mut driven = 0.0;
    let mut shortest = 0.0;
    let mut seconds = 0.0;
    let mut reversals = 0u32;
    let mut count = 0u32;
    for (from, to) in passes.iter().zip(passes.iter().skip(1)) {
        let (out, back) = (from.last, to.first);
        let path: &[Fix] = fixes.get(out..=back).ok_or("a turn outside the log")?;
        driven += path
            .iter()
            .zip(path.iter().skip(1))
            .map(|(p, q)| p.point.distance_to(q.point).metres())
            .sum::<f64>();
        seconds += at(fixes, back)?.seconds - at(fixes, out)?.seconds;
        let mut i = out + 2;
        while i + 2 < back {
            let before = course(at(fixes, i - 2)?.point, at(fixes, i)?.point)?;
            let after = course(at(fixes, i)?.point, at(fixes, i + 2)?.point)?;
            if before.angular_distance(after) > 120.0 {
                reversals += 1;
                i += 5;
            } else {
                i += 1;
            }
        }
        let turn = TurnPath::dubins(
            *plane,
            pose(fixes, out)?,
            pose(fixes, back)?,
            Distance::from_metres(TURN_RADIUS)?,
        )?;
        shortest += turn.length().metres();
        count += 1;
    }
    let turns = f64::from(count.max(1));
    println!(
        "\n{count} headland turns: {:.0} m and {:.0} s a turn, changing direction {:.0} times \
         in each.\nThe shortest turn without reversing, at a {TURN_RADIUS} m radius: {:.0} m.",
        driven / turns,
        seconds / turns,
        f64::from(reversals) / turns,
        shortest / turns
    );
    Ok(())
}

/// The fixes of a pass less [`SETTLE`] at each end.
fn inside(fixes: &[Fix], pass: Pass) -> Outcome<&[Fix]> {
    Ok(fixes
        .get(pass.first + SETTLE..=pass.last.saturating_sub(SETTLE))
        .ok_or("a pass shorter than its settling")?)
}

fn at(fixes: &[Fix], i: usize) -> Outcome<&Fix> {
    Ok(fixes.get(i).ok_or("a fix outside the log")?)
}

/// Grid course from one point to another.
fn course(from: PlanarPoint, to: PlanarPoint) -> Outcome<GridCourse> {
    let east = to.east_metres() - from.east_metres();
    let north = to.north_metres() - from.north_metres();
    Ok(GridCourse::wrap(east.atan2(north).to_degrees())?)
}

/// The machine at fix `i`, its course from the fixes two either side.
fn pose(fixes: &[Fix], i: usize) -> Outcome<Pose> {
    let before = at(fixes, i.saturating_sub(2))?.point;
    let after = at(fixes, (i + 2).min(fixes.len().saturating_sub(1)))?.point;
    Ok(Pose {
        position: at(fixes, i)?.point,
        course: course(before, after)?,
    })
}

fn mean_sd(values: &[f64]) -> (f64, f64) {
    let n = u32::try_from(values.len()).map_or(f64::NAN, f64::from);
    let mean = values.iter().sum::<f64>() / n;
    let variance = values.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / n;
    (mean, variance.sqrt())
}
