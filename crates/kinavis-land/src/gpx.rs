//! GPX 1.1 (and 1.0): waypoints, routes and tracks, read and written.
//!
//! Reading takes the elements the navigation needs — the position of every
//! point, its elevation, time and name, the names of routes and tracks — and
//! passes over the rest (extensions, links, descriptions). Writing produces
//! GPX 1.1 that reads back equal.

use std::fmt::Write as _;
use std::string::String;
use std::vec::Vec;

use kinavis::route::{LegKind, Route};
use kinavis_kernel::time::{Civil, Instant, Utc};
use kinavis_kernel::{Distance, GnssFix, Height, Latitude, Longitude, Position};
use xmlparser::{ElementEnd, Token, Tokenizer};

use crate::error::{LandError, Result};
use crate::polyline::route_from_polyline;

/// A point of a GPX file: a waypoint, a route point or a track point.
#[derive(Debug, Clone, PartialEq)]
pub struct GpxPoint {
    /// Position, WGS 84.
    pub position: Position,
    /// Elevation, metres; GPX does not say above what, in practice the geoid.
    pub elevation: Option<Height>,
    /// Time, UTC.
    pub time: Option<Instant<Utc>>,
    /// Name.
    pub name: Option<String>,
}

/// A route: an ordered list of points to go to.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct GpxRoute {
    /// Name.
    pub name: Option<String>,
    /// Points.
    pub points: Vec<GpxPoint>,
}

/// A track: where someone went, in segments of continuous recording.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct GpxTrack {
    /// Name.
    pub name: Option<String>,
    /// Segments.
    pub segments: Vec<Vec<GpxPoint>>,
}

/// A GPX file.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Gpx {
    /// Program that wrote it.
    pub creator: Option<String>,
    /// Waypoints.
    pub waypoints: Vec<GpxPoint>,
    /// Routes.
    pub routes: Vec<GpxRoute>,
    /// Tracks.
    pub tracks: Vec<GpxTrack>,
}

impl GpxPoint {
    /// A point at a position, nothing else known.
    #[must_use]
    pub const fn at(position: Position) -> Self {
        Self {
            position,
            elevation: None,
            time: None,
            name: None,
        }
    }

    /// The point as a satellite fix, if it has a time: a track point is one.
    #[must_use]
    pub fn to_fix(&self) -> Option<GnssFix> {
        Some(GnssFix::builder(self.time?, self.position).build())
    }
}

impl GpxRoute {
    /// The route as a `kinavis` route of rhumb lines through its points, or
    /// within `tolerance` of them when it has more than a route holds.
    ///
    /// # Errors
    ///
    /// As [`route_from_polyline`].
    pub fn to_route(&self, tolerance: Distance) -> Result<Route> {
        let positions: Vec<Position> = self.points.iter().map(|point| point.position).collect();
        Ok(route_from_polyline(&positions, tolerance, LegKind::RhumbLine)?.route)
    }

    /// A route of the waypoints of a `kinavis` route.
    #[must_use]
    pub fn from_route(route: &Route, name: Option<&str>) -> Self {
        Self {
            name: name.map(String::from),
            points: route.waypoints().iter().map(|p| GpxPoint::at(*p)).collect(),
        }
    }
}

impl GpxTrack {
    /// The fixes of the track, segment after segment; points without a time
    /// are left out.
    pub fn fixes(&self) -> impl Iterator<Item = GnssFix> + '_ {
        self.segments.iter().flatten().filter_map(GpxPoint::to_fix)
    }
}

/// Where a point being read belongs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Owner {
    Waypoint,
    Route,
    Track,
}

/// The text field being read.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Field {
    Elevation,
    Time,
    Name,
    RouteName,
    TrackName,
}

impl Gpx {
    /// Reads a GPX file.
    ///
    /// # Errors
    ///
    /// [`LandError::Format`] for XML that does not parse, a missing or
    /// malformed coordinate, elevation or time, or a root that is not `gpx`.
    pub fn parse(text: &str) -> Result<Self> {
        let mut gpx = Self::default();
        let mut depth = 0usize;
        let mut point: Option<(GpxPoint, Owner)> = None;
        let mut field: Option<Field> = None;
        let mut buffer = String::new();
        let mut open: Option<&str> = None;
        let mut attributes: Vec<(&str, String)> = Vec::new();
        let mut seen_root = false;
        for token in Tokenizer::from(text) {
            let token = token.map_err(|error| format_error("XML", error.pos().row))?;
            match token {
                Token::ElementStart { local, .. } => {
                    open = Some(local.as_str());
                    attributes.clear();
                }
                Token::Attribute { local, value, .. } => {
                    attributes.push((local.as_str(), unescape(value.as_str())));
                }
                Token::ElementEnd { end, span } => {
                    // Lines are counted only for an error: counting them at
                    // every element is quadratic in the file.
                    let at = |error: LandError| at_line(error, text, span.start());
                    let row = 0;
                    match end {
                        ElementEnd::Open | ElementEnd::Empty => {
                            let name = open.take().unwrap_or("");
                            if depth == 0 {
                                if name != "gpx" {
                                    return Err(at(format_error("root element gpx", row)));
                                }
                                seen_root = true;
                                gpx.creator = attribute(&attributes, "creator");
                            }
                            start(
                                &mut gpx,
                                name,
                                depth,
                                &attributes,
                                &mut point,
                                &mut field,
                                row,
                            )
                            .map_err(at)?;
                            buffer.clear();
                            if end == ElementEnd::Open {
                                depth = depth.saturating_add(1);
                            } else {
                                finish(&mut gpx, name, &mut point, &mut field, &buffer, row)
                                    .map_err(at)?;
                            }
                        }
                        ElementEnd::Close(_, local) => {
                            depth = depth.saturating_sub(1);
                            finish(
                                &mut gpx,
                                local.as_str(),
                                &mut point,
                                &mut field,
                                &buffer,
                                row,
                            )
                            .map_err(at)?;
                            buffer.clear();
                        }
                    }
                }
                Token::Text { text: content } if field.is_some() => {
                    buffer.push_str(&unescape(content.as_str()));
                }
                Token::Cdata { text: content, .. } if field.is_some() => {
                    buffer.push_str(content.as_str());
                }
                _ => {}
            }
        }
        if !seen_root {
            return Err(format_error("root element gpx", 1));
        }
        if depth != 0 || open.is_some() {
            return Err(at_line(format_error("XML", 0), text, text.len()));
        }
        Ok(gpx)
    }

    /// The file as GPX 1.1.
    #[must_use]
    pub fn to_xml(&self) -> String {
        let mut out = String::from("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
        let creator = self.creator.as_deref().unwrap_or("KINAVIS");
        let _ = writeln!(
            out,
            "<gpx version=\"1.1\" creator=\"{}\" xmlns=\"http://www.topografix.com/GPX/1/1\">",
            escape(creator)
        );
        for point in &self.waypoints {
            write_point(&mut out, "wpt", point, "  ");
        }
        for route in &self.routes {
            out.push_str("  <rte>\n");
            if let Some(name) = &route.name {
                let _ = writeln!(out, "    <name>{}</name>", escape(name));
            }
            for point in &route.points {
                write_point(&mut out, "rtept", point, "    ");
            }
            out.push_str("  </rte>\n");
        }
        for track in &self.tracks {
            out.push_str("  <trk>\n");
            if let Some(name) = &track.name {
                let _ = writeln!(out, "    <name>{}</name>", escape(name));
            }
            for segment in &track.segments {
                out.push_str("    <trkseg>\n");
                for point in segment {
                    write_point(&mut out, "trkpt", point, "      ");
                }
                out.push_str("    </trkseg>\n");
            }
            out.push_str("  </trk>\n");
        }
        out.push_str("</gpx>\n");
        out
    }
}

#[allow(clippy::too_many_arguments)]
fn start(
    gpx: &mut Gpx,
    name: &str,
    depth: usize,
    attributes: &[(&str, String)],
    point: &mut Option<(GpxPoint, Owner)>,
    field: &mut Option<Field>,
    row: u32,
) -> Result<()> {
    let owner = match name {
        "wpt" if depth == 1 => Some(Owner::Waypoint),
        "rtept" => Some(Owner::Route),
        "trkpt" => Some(Owner::Track),
        _ => None,
    };
    if let Some(owner) = owner {
        let latitude = number(attribute(attributes, "lat").as_deref(), "lat", row)?;
        let longitude = number(attribute(attributes, "lon").as_deref(), "lon", row)?;
        let position = Position::new(
            Latitude::from_degrees(latitude).map_err(|_| format_error("lat", row))?,
            Longitude::from_degrees(longitude).map_err(|_| format_error("lon", row))?,
        );
        *point = Some((GpxPoint::at(position), owner));
        return Ok(());
    }
    match name {
        "rte" if depth == 1 => gpx.routes.push(GpxRoute::default()),
        "trk" if depth == 1 => gpx.tracks.push(GpxTrack::default()),
        "trkseg" => {
            if let Some(track) = gpx.tracks.last_mut() {
                track.segments.push(Vec::new());
            }
        }
        "ele" if point.is_some() => *field = Some(Field::Elevation),
        "time" if point.is_some() => *field = Some(Field::Time),
        "name" if point.is_some() => *field = Some(Field::Name),
        "name" if depth == 2 && gpx.routes.last().is_some_and(|r| r.points.is_empty()) => {
            *field = Some(Field::RouteName);
        }
        "name" if depth == 2 && gpx.tracks.last().is_some_and(|t| t.segments.is_empty()) => {
            *field = Some(Field::TrackName);
        }
        _ => {}
    }
    Ok(())
}

fn finish(
    gpx: &mut Gpx,
    name: &str,
    point: &mut Option<(GpxPoint, Owner)>,
    field: &mut Option<Field>,
    text: &str,
    row: u32,
) -> Result<()> {
    if let Some(current) = field.take() {
        let value = text.trim();
        match (current, point.as_mut()) {
            (Field::Elevation, Some((p, _))) => {
                let metres = number(Some(value), "ele", row)?;
                p.elevation = Some(Height::above_mean_sea_level(
                    Distance::from_metres(metres).map_err(|_| format_error("ele", row))?,
                ));
            }
            (Field::Time, Some((p, _))) => {
                p.time = Some(parse_time(value).ok_or_else(|| format_error("time", row))?);
            }
            (Field::Name, Some((p, _))) => p.name = Some(String::from(value)),
            (Field::RouteName, _) => {
                if let Some(route) = gpx.routes.last_mut() {
                    route.name = Some(String::from(value));
                }
            }
            (Field::TrackName, _) => {
                if let Some(track) = gpx.tracks.last_mut() {
                    track.name = Some(String::from(value));
                }
            }
            _ => {}
        }
        return Ok(());
    }
    if matches!(name, "wpt" | "rtept" | "trkpt") {
        if let Some((done, owner)) = point.take() {
            match owner {
                Owner::Waypoint => gpx.waypoints.push(done),
                Owner::Route => {
                    if let Some(route) = gpx.routes.last_mut() {
                        route.points.push(done);
                    }
                }
                Owner::Track => {
                    if let Some(track) = gpx.tracks.last_mut() {
                        if track.segments.is_empty() {
                            track.segments.push(Vec::new());
                        }
                        if let Some(segment) = track.segments.last_mut() {
                            segment.push(done);
                        }
                    }
                }
            }
        }
    }
    Ok(())
}

fn write_point(out: &mut String, tag: &str, point: &GpxPoint, indent: &str) {
    let _ = write!(
        out,
        "{indent}<{tag} lat=\"{:.9}\" lon=\"{:.9}\"",
        point.position.latitude().degrees(),
        point.position.longitude().degrees()
    );
    if point.elevation.is_none() && point.time.is_none() && point.name.is_none() {
        out.push_str("/>\n");
        return;
    }
    out.push('>');
    if let Some(elevation) = point.elevation {
        let _ = write!(out, "<ele>{}</ele>", elevation.value().metres());
    }
    if let Some(time) = point.time {
        let _ = write!(out, "<time>{}</time>", format_time(time));
    }
    if let Some(name) = &point.name {
        let _ = write!(out, "<name>{}</name>", escape(name));
    }
    let _ = writeln!(out, "</{tag}>");
}

/// `2009-01-17T20:27:37Z`, with the fraction of the second when there is one.
fn format_time(time: Instant<Utc>) -> String {
    let civil = time.civil();
    let mut out = format!(
        "{:04}-{:02}-{:02}T{:02}:{:02}:{:02}",
        civil.year, civil.month, civil.day, civil.hour, civil.minute, civil.second
    );
    if civil.nanos != 0 {
        let fraction = format!("{:09}", civil.nanos);
        let _ = write!(out, ".{}", fraction.trim_end_matches('0'));
    }
    out.push('Z');
    out
}

/// ISO 8601 as GPX uses it: `YYYY-MM-DDThh:mm:ss[.f…](Z|±hh[:mm])`; without a
/// zone, UTC.
fn parse_time(text: &str) -> Option<Instant<Utc>> {
    let bytes = text.as_bytes();
    let digits = |from: usize, count: usize| -> Option<u32> {
        let slice = bytes.get(from..from.checked_add(count)?)?;
        slice.iter().try_fold(0u32, |sum, &b| {
            b.is_ascii_digit().then(|| sum * 10 + u32::from(b - b'0'))
        })
    };
    let year = i32::try_from(digits(0, 4)?).ok()?;
    let separators = [(4, b'-'), (7, b'-'), (10, b'T'), (13, b':'), (16, b':')];
    for (at, expected) in separators {
        let found = *bytes.get(at)?;
        if !(found == expected || (at == 10 && found == b' ')) {
            return None;
        }
    }
    let field = |from: usize| digits(from, 2).and_then(|v| u8::try_from(v).ok());
    let (month, day, hour, minute, second) =
        (field(5)?, field(8)?, field(11)?, field(14)?, field(17)?);
    let mut at = 19;
    let mut nanos = 0u32;
    if bytes.get(at) == Some(&b'.') {
        at += 1;
        let mut scale = 100_000_000u32;
        while let Some(&b) = bytes.get(at).filter(|b| b.is_ascii_digit()) {
            nanos += u32::from(b - b'0') * scale;
            scale /= 10;
            at += 1;
        }
    }
    let offset_seconds: i64 = match bytes.get(at) {
        None | Some(b'Z' | b'z') => 0,
        Some(sign @ (b'+' | b'-')) => {
            let hours = i64::from(digits(at + 1, 2)?);
            let minutes = match bytes.get(at + 3) {
                Some(b':') => i64::from(digits(at + 4, 2)?),
                Some(_) => i64::from(digits(at + 3, 2)?),
                None => 0,
            };
            let total = hours * 3600 + minutes * 60;
            if *sign == b'+' {
                total
            } else {
                -total
            }
        }
        Some(_) => return None,
    };
    let local = Instant::<Utc>::from_civil(Civil {
        year,
        month,
        day,
        hour,
        minute,
        second: second.min(59),
        nanos,
    })
    .ok()?;
    let utc = if offset_seconds >= 0 {
        local.checked_sub(core::time::Duration::from_secs(
            offset_seconds.unsigned_abs(),
        ))
    } else {
        local.checked_add(core::time::Duration::from_secs(
            offset_seconds.unsigned_abs(),
        ))
    }?;
    // A leap second, 60, is the last instant of its minute.
    if second == 60 {
        return utc.checked_add(core::time::Duration::from_secs(1));
    }
    Some(utc)
}

fn attribute(attributes: &[(&str, String)], name: &str) -> Option<String> {
    attributes
        .iter()
        .find(|(key, _)| *key == name)
        .map(|(_, value)| value.clone())
}

fn number(text: Option<&str>, what: &'static str, row: u32) -> Result<f64> {
    text.and_then(|t| t.trim().parse::<f64>().ok())
        .filter(|value| value.is_finite())
        .ok_or_else(|| format_error(what, row))
}

fn format_error(what: &'static str, line: u32) -> LandError {
    LandError::Format { what, line }
}

/// The error with the line of `offset` in `text`.
fn at_line(error: LandError, text: &str, offset: usize) -> LandError {
    match error {
        LandError::Format { what, .. } => LandError::Format {
            what,
            line: row_of(text, offset),
        },
        other => other,
    }
}

fn row_of(text: &str, offset: usize) -> u32 {
    let before = text.get(..offset).unwrap_or("");
    u32::try_from(
        before
            .bytes()
            .filter(|&b| b == b'\n')
            .count()
            .saturating_add(1),
    )
    .unwrap_or(u32::MAX)
}

/// The five predefined entities and numeric references.
fn unescape(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    let mut rest = text;
    while let Some(amp) = rest.find('&') {
        out.push_str(rest.get(..amp).unwrap_or(""));
        let after = rest.get(amp..).unwrap_or("");
        let Some(semi) = after.find(';') else {
            out.push_str(after);
            return out;
        };
        let entity = after.get(1..semi).unwrap_or("");
        let decoded = match entity {
            "amp" => Some('&'),
            "lt" => Some('<'),
            "gt" => Some('>'),
            "quot" => Some('"'),
            "apos" => Some('\''),
            _ => entity
                .strip_prefix("#x")
                .or_else(|| entity.strip_prefix("#X"))
                .and_then(|hex| u32::from_str_radix(hex, 16).ok())
                .or_else(|| entity.strip_prefix('#').and_then(|dec| dec.parse().ok()))
                .and_then(char::from_u32)
                .filter(|&ch| xml_char(ch)),
        };
        match decoded {
            Some(ch) => out.push(ch),
            None => out.push_str(after.get(..=semi).unwrap_or("")),
        }
        rest = after.get(semi.saturating_add(1)..).unwrap_or("");
    }
    out.push_str(rest);
    out
}

/// Whether XML 1.0 allows the character: a reference to any other, `&#0;`
/// among them, stays text, as written.
fn xml_char(ch: char) -> bool {
    matches!(ch, '\u{9}' | '\u{A}' | '\u{D}' | '\u{20}'..='\u{D7FF}' | '\u{E000}'..='\u{FFFD}' | '\u{10000}'..)
}

fn escape(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    for ch in text.chars() {
        match ch {
            '&' => out.push_str("&amp;"),
            '<' => out.push_str("&lt;"),
            '>' => out.push_str("&gt;"),
            '"' => out.push_str("&quot;"),
            '\'' => out.push_str("&apos;"),
            _ => out.push(ch),
        }
    }
    out
}
