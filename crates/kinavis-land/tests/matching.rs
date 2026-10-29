//! The map matcher on small constructed networks: a road beside another,
//! a fork, a gap in the network, fixes passed over, the window.

#![allow(
    clippy::expect_used,
    clippy::panic,
    clippy::indexing_slicing,
    clippy::cast_precision_loss,
    clippy::float_cmp,
    clippy::type_complexity
)]

use kinavis_kernel::{Distance, Position};
use kinavis_land::{Candidate, MapMatcher, Matched, MatcherConfig, RoadId, RoadNetwork};

/// Metres per degree near the origin used here (0°N 0°E), on WGS 84.
const NORTH: f64 = 110_574.4;
const EAST: f64 = 111_319.5;

fn at(east: f64, north: f64) -> Position {
    Position::from_degrees(north / NORTH, east / EAST).expect("near the origin")
}

fn metres(value: f64) -> Distance {
    Distance::from_metres(value).expect("finite")
}

/// One-way straight roads on a local plane: id, start, end (metres east and
/// north), and which roads follow which.
struct Roads {
    roads: Vec<(RoadId, (f64, f64), (f64, f64))>,
    next: Vec<(RoadId, RoadId)>,
}

impl Roads {
    fn road(&self, id: RoadId) -> ((f64, f64), (f64, f64)) {
        let (_, a, b) = self.roads.iter().find(|road| road.0 == id).expect("a road");
        (*a, *b)
    }

    fn length(&self, id: RoadId) -> f64 {
        let (a, b) = self.road(id);
        (b.0 - a.0).hypot(b.1 - a.1)
    }
}

impl RoadNetwork for Roads {
    fn candidates(&self, position: Position, radius: Distance, out: &mut [Candidate]) -> usize {
        let (x, y) = (
            position.longitude().degrees() * EAST,
            position.latitude().degrees() * NORTH,
        );
        let mut found: Vec<Candidate> = self
            .roads
            .iter()
            .filter_map(|&(road, a, b)| {
                let (dx, dy) = (b.0 - a.0, b.1 - a.1);
                let t = (((x - a.0) * dx + (y - a.1) * dy) / (dx * dx + dy * dy)).clamp(0.0, 1.0);
                let (px, py) = (a.0 + t * dx, a.1 + t * dy);
                let distance = (x - px).hypot(y - py);
                (distance <= radius.metres()).then(|| Candidate {
                    road,
                    forward: true,
                    point: at(px, py),
                    offset: metres(t * dx.hypot(dy)),
                    distance: metres(distance),
                })
            })
            .collect();
        found.sort_by(|a, b| a.distance.metres().total_cmp(&b.distance.metres()));
        let count = found.len().min(out.len());
        out[..count].copy_from_slice(&found[..count]);
        count
    }

    fn route_distance(
        &self,
        from: &Candidate,
        to: &Candidate,
        limit: Distance,
    ) -> Option<Distance> {
        let distance = if from.road == to.road {
            (to.offset.metres() >= from.offset.metres())
                .then(|| to.offset.metres() - from.offset.metres())?
        } else if self.next.contains(&(from.road, to.road)) {
            self.length(from.road) - from.offset.metres() + to.offset.metres()
        } else {
            return None;
        };
        (distance <= limit.metres()).then(|| metres(distance))
    }
}

fn config() -> MatcherConfig {
    MatcherConfig::new(metres(4.0), metres(3.0), metres(60.0), metres(500.0)).expect("settings")
}

fn run<const K: usize, const W: usize>(roads: &Roads, fixes: &[Position]) -> Vec<Matched> {
    let mut matcher = MapMatcher::<K, W>::new(config());
    let mut out = Vec::new();
    for fix in fixes {
        matcher.push(roads, *fix, |m| out.push(m)).expect("a step");
        assert!(matcher.pending() <= W);
    }
    matcher.finish(|m| out.push(m));
    assert_eq!(matcher.pending(), 0);
    out
}

#[test]
fn a_drive_on_a_road_beside_another_stays_on_it() {
    // Two parallel roads 12 m apart; the drive is on the southern one but
    // its fixes stray up to 7 m north, nearer the other now and then.
    let roads = Roads {
        roads: vec![
            (1, (0.0, 0.0), (1000.0, 0.0)),
            (2, (0.0, 12.0), (1000.0, 12.0)),
        ],
        next: vec![],
    };
    let fixes: Vec<Position> = (0..60)
        .map(|k| {
            at(
                10.0 + 15.0 * f64::from(k),
                if k % 7 == 3 {
                    7.0
                } else {
                    1.5 * f64::from(k % 3)
                },
            )
        })
        .collect();
    let matched = run::<4, 8>(&roads, &fixes);
    assert_eq!(matched.len(), 60);
    assert!(matched.iter().all(|m| m.candidate.road == 1), "{matched:?}");
    let indices: Vec<u64> = matched.iter().map(|m| m.index).collect();
    assert_eq!(indices, (0..60).collect::<Vec<_>>());
}

#[test]
fn a_fork_is_taken_the_way_the_fixes_go_though_they_start_ambiguous() {
    // Road 1 east to a fork at 500 m; road 2 bears north-east, road 3
    // south-east. Near the fork both are close; later only road 3.
    let roads = Roads {
        roads: vec![
            (1, (0.0, 0.0), (500.0, 0.0)),
            (2, (500.0, 0.0), (900.0, 120.0)),
            (3, (500.0, 0.0), (900.0, -120.0)),
        ],
        next: vec![(1, 2), (1, 3)],
    };
    let fixes: Vec<Position> = (0..40)
        .map(|k| {
            let x = 380.0 + 12.0 * f64::from(k);
            let y = if x > 500.0 { -(x - 500.0) * 0.3 } else { 0.0 };
            at(x, y + 1.0)
        })
        .collect();
    let matched = run::<4, 12>(&roads, &fixes);
    for m in &matched {
        let x = 380.0 + 12.0 * m.index as f64;
        let road = if x <= 500.0 { 1 } else { 3 };
        // Right at the fork either road's start is the same point.
        if (x - 500.0).abs() > 12.0 {
            assert_eq!(m.candidate.road, road, "fix {} at {x} m", m.index);
        }
    }
}

#[test]
fn a_gap_in_the_network_breaks_the_chain_and_a_new_one_starts() {
    let roads = Roads {
        roads: vec![
            (1, (0.0, 0.0), (300.0, 0.0)),
            (2, (400.0, 0.0), (800.0, 0.0)),
        ],
        next: vec![],
    };
    let fixes: Vec<Position> = (0..50).map(|k| at(16.0 * f64::from(k), 1.0)).collect();
    let matched = run::<4, 6>(&roads, &fixes);
    // Fixes between 300 and 400 m are within the radius of the road ends,
    // and none of them connects 1 to 2.
    let breaks: Vec<&Matched> = matched.iter().filter(|m| m.after_break).collect();
    assert_eq!(breaks.len(), 1, "{breaks:?}");
    assert_eq!(breaks[0].candidate.road, 2);
    assert!(matched
        .iter()
        .filter(|m| m.index < breaks[0].index)
        .all(|m| m.candidate.road == 1));
}

#[test]
fn fixes_within_two_sigma_or_far_from_any_road_are_passed_over() {
    let roads = Roads {
        roads: vec![(1, (0.0, 0.0), (1000.0, 0.0))],
        next: vec![],
    };
    let fixes = [
        at(0.0, 1.0),
        at(3.0, 1.0),
        at(20.0, 1.0),
        at(20.0, 500.0),
        at(40.0, -1.0),
    ];
    let matched = run::<2, 4>(&roads, &fixes);
    let indices: Vec<u64> = matched.iter().map(|m| m.index).collect();
    // 1: 3 m from fix 0, within 2σ = 8 m; 3: 500 m from the road.
    assert_eq!(indices, vec![0, 2, 4]);
    assert!((matched[1].candidate.offset.metres() - 20.0).abs() < 0.01);
}

#[test]
fn a_window_of_one_decides_each_fix_as_the_next_arrives() {
    let roads = Roads {
        roads: vec![(1, (0.0, 0.0), (1000.0, 0.0))],
        next: vec![],
    };
    let mut matcher = MapMatcher::<2, 1>::new(config());
    let mut out = Vec::new();
    matcher
        .push(&roads, at(0.0, 0.0), |m| out.push(m))
        .expect("a step");
    assert_eq!(out, Vec::new());
    matcher
        .push(&roads, at(20.0, 0.0), |m| out.push(m))
        .expect("a step");
    assert_eq!(out.len(), 1);
    assert_eq!(out[0].index, 0);
}

#[test]
fn settings_out_of_range_are_refused() {
    assert!(MatcherConfig::new(metres(0.01), metres(3.0), metres(200.0), metres(2000.0)).is_err());
    assert!(MatcherConfig::new(metres(4.0), metres(0.0), metres(200.0), metres(2000.0)).is_err());
    assert!(MatcherConfig::new(metres(4.0), metres(3.0), metres(0.5), metres(2000.0)).is_err());
    assert!(MatcherConfig::new(metres(4.0), metres(3.0), metres(200.0), metres(-1.0)).is_err());
}

#[test]
fn a_routers_polyline_becomes_a_route_within_its_tolerance() {
    use kinavis::route::{LegKind, MAX_WAYPOINTS};
    use kinavis_land::route_from_polyline;
    // A gentle S-bend of 2001 points over 2 km: within 0.5 m of a few legs.
    let bend: Vec<Position> = (0..=2000)
        .map(|k| {
            let x = f64::from(k);
            at(x, 40.0 * (x / 2000.0 * core::f64::consts::TAU).sin())
        })
        .collect();
    let simple = route_from_polyline(&bend, metres(0.5), LegKind::RhumbLine).expect("a route");
    assert!(simple.deviation.metres() <= 0.5);
    let legs = simple.route.waypoints().len();
    assert!((5..60).contains(&legs), "{legs} waypoints");
    assert_eq!(simple.route.waypoints()[0], bend[0]);
    assert_eq!(
        *simple.route.waypoints().last().expect("an end"),
        bend[2000]
    );
    // A zigzag no route of 128 points can follow to a millimetre: full, and
    // how far off it stays is said.
    let zigzag: Vec<Position> = (0..1000)
        .map(|k| at(10.0 * f64::from(k), if k % 2 == 0 { 0.0 } else { 5.0 }))
        .collect();
    let full = route_from_polyline(&zigzag, metres(0.001), LegKind::RhumbLine).expect("a route");
    assert_eq!(full.route.waypoints().len(), MAX_WAYPOINTS);
    assert!(full.deviation.metres() > 1.0);
    // A straight line: its ends.
    let straight: Vec<Position> = (0..100).map(|k| at(f64::from(k), 0.0)).collect();
    let two = route_from_polyline(&straight, metres(0.01), LegKind::RhumbLine).expect("a route");
    assert_eq!(two.route.waypoints().len(), 2);
    assert!(route_from_polyline(&straight[..1], metres(1.0), LegKind::RhumbLine).is_err());
    assert!(route_from_polyline(&straight, metres(-1.0), LegKind::RhumbLine).is_err());
}
