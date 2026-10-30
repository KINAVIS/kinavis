//! A drive through Seattle with a tunnel in it: Newson and Krumm's own
//! drive (7531 fixes at 1 Hz, Microsoft Research, 2009; not in the
//! repository), with 40 fixes cut out as a tunnel would. The map matcher
//! keeps the drive on its roads across the gap; dead reckoning on the last
//! course and speed carries the position through it, and the fix at the
//! exit says how far off it came out.
//!
//! The road network is the application's here: the roads as polylines
//! joined at equal end points, a grid index, Dijkstra for the driving
//! distance. `SEATTLE_DATA` names the directory with `gps_data.txt`,
//! `road_network.txt` and `ground_truth_route.txt`.
//!
//! ```text
//! SEATTLE_DATA=… cargo run --release -p kinavis-examples --example drive
//! ```

use std::collections::HashMap;
use std::error::Error;
use std::path::PathBuf;

use kinavis::sailings::{great_circle, rhumb_destination};
use kinavis_kernel::{Distance, Position};
use kinavis_land::{MapMatcher, Matched, MatcherConfig, RoadId};

use network::Network;

/// The fixes cut out, first and past the last: 40 s at about 75 km/h on a
/// road that bends, chosen in the data for that.
const TUNNEL: (usize, usize) = (2840, 2880);

/// The application's side: the road network of the data set, its index and
/// its router. Not the crates' code, and written as a test harness is.
mod network {
    #![allow(
        clippy::expect_used,
        clippy::indexing_slicing,
        clippy::cast_possible_truncation,
        clippy::many_single_char_names
    )]

    use std::cell::RefCell;
    use std::cmp::Ordering;
    use std::collections::{BinaryHeap, HashMap, HashSet};

    use kinavis_kernel::{Distance, Latitude, Longitude, Position};
    use kinavis_land::{Candidate, RoadId, RoadNetwork};

    const A: f64 = 6_378_137.0;
    const E2: f64 = 0.006_694_379_990_14;
    /// Grid cell of the segment index, degrees.
    const CELL: f64 = 0.002;

    /// Metres per degree north and east at a latitude, on WGS 84.
    fn scales(latitude: f64) -> (f64, f64) {
        let phi = latitude.to_radians();
        let w = (1.0 - E2 * phi.sin().powi(2)).sqrt();
        let meridian = A * (1.0 - E2) / w.powi(3);
        let normal = A / w;
        (meridian.to_radians(), (normal * phi.cos()).to_radians())
    }

    struct Road {
        id: RoadId,
        from: u64,
        to: u64,
        two_way: bool,
        /// Vertices, latitude and longitude.
        points: Vec<(f64, f64)>,
        /// Distance along the road to each vertex, m.
        along: Vec<f64>,
    }

    pub(super) struct Network {
        roads: Vec<Road>,
        /// Directed edges leaving each node: road index, forward.
        leaving: HashMap<u64, Vec<(usize, bool)>>,
        /// Segments in each cell: road index, segment index.
        cells: HashMap<(i32, i32), Vec<(usize, usize)>>,
        index: HashMap<RoadId, usize>,
        /// Dijkstra from a node, to a limit: distances found.
        cache: RefCell<HashMap<(u64, u64), HashMap<u64, f64>>>,
    }

    fn cell(lat: f64, lon: f64) -> (i32, i32) {
        ((lat / CELL).floor() as i32, (lon / CELL).floor() as i32)
    }

    impl Network {
        pub(super) fn load(path: &std::path::Path) -> Self {
            let text = std::fs::read_to_string(path).expect("road_network.txt");
            let mut roads = Vec::new();
            for line in text.lines().skip(1) {
                let fields: Vec<&str> = line.splitn(7, '\t').collect();
                let geometry = fields[6]
                    .trim()
                    .trim_start_matches("LINESTRING(")
                    .trim_end_matches(')');
                let points: Vec<(f64, f64)> = geometry
                    .split(',')
                    .map(|pair| {
                        let mut xy = pair.split_whitespace();
                        let lon: f64 = xy.next().expect("lon").parse().expect("lon");
                        let lat: f64 = xy.next().expect("lat").parse().expect("lat");
                        (lat, lon)
                    })
                    .collect();
                let mut along = vec![0.0];
                for pair in points.windows(2) {
                    let (north, east) = scales(f64::midpoint(pair[0].0, pair[1].0));
                    let d = ((pair[1].0 - pair[0].0) * north).hypot((pair[1].1 - pair[0].1) * east);
                    along.push(along.last().expect("a start") + d);
                }
                // The network is cut into tiles whose border nodes carry
                // different identifiers on either side (39 469 ends touch one
                // road): nodes are joined by their coordinates.
                let node = |(lat, lon): (f64, f64)| lat.to_bits() ^ lon.to_bits().rotate_left(32);
                let (first, last) = (points[0], points[points.len() - 1]);
                roads.push(Road {
                    id: fields[0].parse().expect("an id"),
                    from: node(first),
                    to: node(last),
                    two_way: fields[3] == "1",
                    points,
                    along,
                });
            }
            let mut leaving: HashMap<u64, Vec<(usize, bool)>> = HashMap::new();
            let mut cells: HashMap<(i32, i32), Vec<(usize, usize)>> = HashMap::new();
            let mut index = HashMap::new();
            for (r, road) in roads.iter().enumerate() {
                index.insert(road.id, r);
                leaving.entry(road.from).or_default().push((r, true));
                if road.two_way {
                    leaving.entry(road.to).or_default().push((r, false));
                }
                for (s, pair) in road.points.windows(2).enumerate() {
                    let (a, b) = (cell(pair[0].0, pair[0].1), cell(pair[1].0, pair[1].1));
                    for i in a.0.min(b.0)..=a.0.max(b.0) {
                        for j in a.1.min(b.1)..=a.1.max(b.1) {
                            cells.entry((i, j)).or_default().push((r, s));
                        }
                    }
                }
            }
            Self {
                roads,
                leaving,
                cells,
                index,
                cache: RefCell::new(HashMap::new()),
            }
        }

        fn length(&self, r: usize) -> f64 {
            *self.roads[r].along.last().expect("vertices")
        }

        /// Shortest distances from a node, up to `limit`.
        fn reach(&self, start: u64, limit: f64) -> HashMap<u64, f64> {
            #[derive(PartialEq)]
            struct Item(f64, u64);
            impl Eq for Item {}
            impl Ord for Item {
                fn cmp(&self, other: &Self) -> Ordering {
                    other.0.total_cmp(&self.0)
                }
            }
            impl PartialOrd for Item {
                fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
                    Some(self.cmp(other))
                }
            }
            let mut done: HashMap<u64, f64> = HashMap::new();
            let mut heap = BinaryHeap::from([Item(0.0, start)]);
            while let Some(Item(d, node)) = heap.pop() {
                if d > limit || done.contains_key(&node) {
                    continue;
                }
                done.insert(node, d);
                for &(r, forward) in self.leaving.get(&node).map_or(&[][..], Vec::as_slice) {
                    let road = &self.roads[r];
                    let next = if forward { road.to } else { road.from };
                    if !done.contains_key(&next) {
                        heap.push(Item(d + self.length(r), next));
                    }
                }
            }
            done
        }
    }

    impl RoadNetwork for Network {
        fn candidates(&self, position: Position, radius: Distance, out: &mut [Candidate]) -> usize {
            let (lat, lon) = (
                position.latitude().degrees(),
                position.longitude().degrees(),
            );
            let (north, east) = scales(lat);
            let r = radius.metres();
            let reach_lat = (r / north / CELL).ceil() as i32 + 1;
            let reach_lon = (r / east / CELL).ceil() as i32 + 1;
            let (ci, cj) = cell(lat, lon);
            // Nearest point of each road: distance, offset from its start, point.
            let mut best: HashMap<usize, (f64, f64, (f64, f64))> = HashMap::new();
            let mut seen = HashSet::new();
            for i in ci - reach_lat..=ci + reach_lat {
                for j in cj - reach_lon..=cj + reach_lon {
                    for &(road, segment) in self.cells.get(&(i, j)).map_or(&[][..], Vec::as_slice) {
                        if !seen.insert((road, segment)) {
                            continue;
                        }
                        let p = &self.roads[road].points;
                        let (a, b) = (p[segment], p[segment + 1]);
                        // On the plane of the fix.
                        let (ax, ay) = ((a.1 - lon) * east, (a.0 - lat) * north);
                        let (bx, by) = ((b.1 - lon) * east, (b.0 - lat) * north);
                        let (dx, dy) = (bx - ax, by - ay);
                        let span = dx * dx + dy * dy;
                        let t = if span > 0.0 {
                            (-(ax * dx + ay * dy) / span).clamp(0.0, 1.0)
                        } else {
                            0.0
                        };
                        let (px, py) = (ax + t * dx, ay + t * dy);
                        let distance = px.hypot(py);
                        if distance > r {
                            continue;
                        }
                        let along = &self.roads[road].along;
                        let offset = along[segment] + t * (along[segment + 1] - along[segment]);
                        let point = (lat + py / north, lon + px / east);
                        let entry = best.entry(road).or_insert((f64::MAX, 0.0, point));
                        if distance < entry.0 {
                            *entry = (distance, offset, point);
                        }
                    }
                }
            }
            let mut found: Vec<Candidate> = Vec::new();
            for (road, (distance, offset, (plat, plon))) in best {
                let length = self.length(road);
                let point = Position::new(
                    Latitude::from_degrees(plat).expect("latitude"),
                    Longitude::from_degrees_wrapped(plon),
                );
                let mut push = |forward: bool, along: f64| {
                    found.push(Candidate {
                        road: self.roads[road].id,
                        forward,
                        point,
                        offset: Distance::from_metres(along).expect("finite"),
                        distance: Distance::from_metres(distance).expect("finite"),
                    });
                };
                push(true, offset);
                if self.roads[road].two_way {
                    push(false, length - offset);
                }
            }
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
            let limit = limit.metres();
            let (a, b) = (self.index[&from.road], self.index[&to.road]);
            if a == b && from.forward == to.forward && to.offset.metres() >= from.offset.metres() {
                return Distance::from_metres(to.offset.metres() - from.offset.metres()).ok();
            }
            let leave = self.length(a) - from.offset.metres();
            let start = if from.forward {
                self.roads[a].to
            } else {
                self.roads[a].from
            };
            let target = if to.forward {
                self.roads[b].from
            } else {
                self.roads[b].to
            };
            let key = (start, limit.to_bits());
            let mut cache = self.cache.borrow_mut();
            if cache.len() > 50_000 {
                cache.clear();
            }
            let reached = cache.entry(key).or_insert_with(|| self.reach(start, limit));
            let total = leave + reached.get(&target)? + to.offset.metres();
            (total <= limit).then(|| Distance::from_metres(total).expect("finite"))
        }
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    let directory = PathBuf::from(std::env::var("SEATTLE_DATA").map_err(|_| "set SEATTLE_DATA")?);
    let network = Network::load(&directory.join("road_network.txt"));
    let mut fixes = Vec::new();
    for line in std::fs::read_to_string(directory.join("gps_data.txt"))?
        .lines()
        .skip(1)
    {
        let mut fields = line.split('\t').skip(2);
        let (Some(lat), Some(lon)) = (fields.next(), fields.next()) else {
            return Err("a fix without its position".into());
        };
        fixes.push(Position::from_degrees(lat.parse()?, lon.parse()?)?);
    }
    let truth: HashMap<RoadId, bool> =
        std::fs::read_to_string(directory.join("ground_truth_route.txt"))?
            .lines()
            .skip(1)
            .filter_map(|line| {
                let mut fields = line.split('\t');
                Some((
                    fields.next()?.trim().parse().ok()?,
                    fields.next()?.trim() == "1",
                ))
            })
            .collect();
    let config = MatcherConfig::new(
        Distance::from_metres(4.07)?,
        Distance::from_metres(3.0)?,
        Distance::from_metres(200.0)?,
        Distance::from_metres(2_000.0)?,
    )?;
    let mut matcher = MapMatcher::<8, 16>::new(config);
    let mut matched: Vec<Matched> = Vec::new();
    // The matcher numbers the fixes it is given; the drive's numbers of them.
    let given: Vec<(usize, Position)> = fixes
        .iter()
        .copied()
        .enumerate()
        .filter(|(index, _)| !(TUNNEL.0..TUNNEL.1).contains(index))
        .collect();
    for (_, fix) in &given {
        matcher.push(&network, *fix, |m| matched.push(m))?;
    }
    matcher.finish(|m| matched.push(m));
    for m in &mut matched {
        let number = usize::try_from(m.index)
            .ok()
            .and_then(|i| given.get(i))
            .map_or(0, |g| g.0);
        m.index = u64::try_from(number)?;
    }
    let on_route = matched
        .iter()
        .filter(|m| truth.contains_key(&m.candidate.road))
        .count();
    let breaks = matched.iter().filter(|m| m.after_break).count();
    println!(
        "{} fixes, {} cut for the tunnel; {} matched, {:.2} % on the road driven, {breaks} breaks",
        fixes.len(),
        TUNNEL.1 - TUNNEL.0,
        matched.len(),
        percent(on_route, matched.len())
    );
    let first = u64::try_from(TUNNEL.0)?;
    let last = u64::try_from(TUNNEL.1)?;
    let before = matched
        .iter()
        .rev()
        .find(|m| m.index < first)
        .ok_or("a fix before")?;
    let after = matched
        .iter()
        .find(|m| m.index >= last)
        .ok_or("a fix after")?;
    println!(
        "Into the tunnel on road {}, out of it on road {} ({}), after {} s",
        before.candidate.road,
        after.candidate.road,
        if truth.contains_key(&after.candidate.road) {
            "the road driven"
        } else {
            "another road"
        },
        after.index - before.index
    );

    reckon(&fixes)
}

/// Dead reckoning through the gap on the course and speed of the last ten
/// seconds before it, against the fix at the exit.
fn reckon(fixes: &[Position]) -> Result<(), Box<dyn Error>> {
    let at = |index: usize| fixes.get(index).copied().ok_or("a fix of the drive");
    let (a, b, exit) = (at(TUNNEL.0 - 11)?, at(TUNNEL.0 - 1)?, at(TUNNEL.1)?);
    let leg = great_circle(a, b)?;
    let speed = leg.distance.metres() / 10.0;
    let seconds = f64::from(u32::try_from(TUNNEL.1 - TUNNEL.0 + 1)?);
    let reckoned = rhumb_destination(
        b,
        leg.initial_course,
        Distance::from_metres(speed * seconds)?,
    )?;
    let miss = great_circle(reckoned, exit)?.distance.metres();
    let travelled = great_circle(b, exit)?.distance.metres();
    println!(
        "Dead reckoning at {speed:.1} m/s on {:.0}°: {miss:.0} m from the exit fix after {travelled:.0} m",
        leg.initial_course.degrees()
    );
    Ok(())
}

fn percent(part: usize, whole: usize) -> f64 {
    let (part, whole) = (
        u32::try_from(part).unwrap_or(0),
        u32::try_from(whole).unwrap_or(1).max(1),
    );
    100.0 * f64::from(part) / f64::from(whole)
}
