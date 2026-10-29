//! `GeoJSON` (RFC 7946): points, lines and polygons of WGS 84 positions, as
//! features with a name, read and written.
//!
//! Positions are `[longitude, latitude]` and an optional height, as the RFC
//! orders them; a polygon's outer ring runs anticlockwise and its holes
//! clockwise when written, and either way when read.

use std::string::String;
use std::vec::Vec;

use kinavis::route::Route;
use kinavis_kernel::{Latitude, Longitude, Position};
use serde_json::{json, Map, Value};

use crate::error::{LandError, Result};

/// A geometry of WGS 84 positions.
#[derive(Debug, Clone, PartialEq)]
pub enum Geometry {
    /// A point.
    Point(Position),
    /// A line through points.
    LineString(Vec<Position>),
    /// A polygon: its outer ring, then its holes; rings closed.
    Polygon(Vec<Vec<Position>>),
}

/// A geometry with a name.
#[derive(Debug, Clone, PartialEq)]
pub struct Feature {
    /// Geometry.
    pub geometry: Geometry,
    /// The `name` property.
    pub name: Option<String>,
}

impl Feature {
    /// A route as a line.
    #[must_use]
    pub fn route(route: &Route, name: Option<&str>) -> Self {
        Self {
            geometry: Geometry::LineString(route.waypoints().to_vec()),
            name: name.map(String::from),
        }
    }
}

/// Features as a `FeatureCollection`.
#[must_use]
pub fn to_geojson(features: &[Feature]) -> String {
    let collection = json!({
        "type": "FeatureCollection",
        "features": features.iter().map(feature_value).collect::<Vec<_>>(),
    });
    collection.to_string()
}

/// The features of a `GeoJSON` text: a `FeatureCollection`, a `Feature` or a
/// bare geometry; `Multi…` geometries and geometry collections are split
/// into their parts, each with the feature's name.
///
/// # Errors
///
/// [`LandError::Format`] for text that is not JSON, an unknown or missing
/// type, or a coordinate that is not a position on Earth.
pub fn parse_geojson(text: &str) -> Result<Vec<Feature>> {
    let value: Value = serde_json::from_str(text).map_err(|e| format_error("JSON", e.line()))?;
    let mut out = Vec::new();
    collect(&value, None, &mut out)?;
    Ok(out)
}

fn collect(value: &Value, name: Option<&str>, out: &mut Vec<Feature>) -> Result<()> {
    let kind = value
        .get("type")
        .and_then(Value::as_str)
        .ok_or_else(|| format_error("type", 0))?;
    match kind {
        "FeatureCollection" => {
            let features = value
                .get("features")
                .and_then(Value::as_array)
                .ok_or_else(|| format_error("features", 0))?;
            for feature in features {
                collect(feature, None, out)?;
            }
        }
        "Feature" => {
            let own = value
                .get("properties")
                .and_then(|p| p.get("name"))
                .and_then(Value::as_str);
            match value.get("geometry") {
                Some(Value::Null) | None => {}
                Some(geometry) => collect(geometry, own, out)?,
            }
        }
        "GeometryCollection" => {
            let parts = value
                .get("geometries")
                .and_then(Value::as_array)
                .ok_or_else(|| format_error("geometries", 0))?;
            for part in parts {
                collect(part, name, out)?;
            }
        }
        _ => {
            let coordinates = value
                .get("coordinates")
                .ok_or_else(|| format_error("coordinates", 0))?;
            let mut push = |geometry| {
                out.push(Feature {
                    geometry,
                    name: name.map(String::from),
                });
            };
            match kind {
                "Point" => push(Geometry::Point(position(coordinates)?)),
                "MultiPoint" => {
                    for p in array(coordinates)? {
                        push(Geometry::Point(position(p)?));
                    }
                }
                "LineString" => push(Geometry::LineString(line(coordinates)?)),
                "MultiLineString" => {
                    for l in array(coordinates)? {
                        push(Geometry::LineString(line(l)?));
                    }
                }
                "Polygon" => push(Geometry::Polygon(rings(coordinates)?)),
                "MultiPolygon" => {
                    for p in array(coordinates)? {
                        push(Geometry::Polygon(rings(p)?));
                    }
                }
                _ => return Err(format_error("geometry type", 0)),
            }
        }
    }
    Ok(())
}

fn feature_value(feature: &Feature) -> Value {
    let geometry = match &feature.geometry {
        Geometry::Point(p) => json!({"type": "Point", "coordinates": coordinate(*p)}),
        Geometry::LineString(points) => json!({
            "type": "LineString",
            "coordinates": points.iter().copied().map(coordinate).collect::<Vec<_>>(),
        }),
        Geometry::Polygon(rings) => json!({
            "type": "Polygon",
            "coordinates": rings
                .iter()
                .enumerate()
                .map(|(index, ring)| oriented(ring, index == 0))
                .collect::<Vec<_>>(),
        }),
    };
    let mut properties = Map::new();
    if let Some(name) = &feature.name {
        properties.insert(String::from("name"), Value::from(name.as_str()));
    }
    json!({"type": "Feature", "geometry": geometry, "properties": properties})
}

/// A ring closed and turned the way RFC 7946 asks: the outer ring
/// anticlockwise, holes clockwise.
fn oriented(ring: &[Position], outer: bool) -> Vec<Value> {
    let mut points: Vec<Position> = ring.to_vec();
    if points.first() != points.last() {
        if let Some(&first) = points.first() {
            points.push(first);
        }
    }
    let area: f64 = points
        .windows(2)
        .map(|pair| match pair {
            [a, b] => {
                a.longitude().degrees() * b.latitude().degrees()
                    - b.longitude().degrees() * a.latitude().degrees()
            }
            _ => 0.0,
        })
        .sum();
    if (area > 0.0) != outer {
        points.reverse();
    }
    points.into_iter().map(coordinate).collect()
}

fn coordinate(position: Position) -> Value {
    json!([
        position.longitude().degrees(),
        position.latitude().degrees()
    ])
}

fn array(value: &Value) -> Result<&Vec<Value>> {
    value
        .as_array()
        .ok_or_else(|| format_error("coordinates", 0))
}

fn position(value: &Value) -> Result<Position> {
    let pair = array(value)?;
    let (Some(lon), Some(lat)) = (
        pair.first().and_then(Value::as_f64),
        pair.get(1).and_then(Value::as_f64),
    ) else {
        return Err(format_error("position", 0));
    };
    Ok(Position::new(
        Latitude::from_degrees(lat).map_err(|_| format_error("latitude", 0))?,
        Longitude::from_degrees(lon).map_err(|_| format_error("longitude", 0))?,
    ))
}

fn line(value: &Value) -> Result<Vec<Position>> {
    array(value)?.iter().map(position).collect()
}

fn rings(value: &Value) -> Result<Vec<Vec<Position>>> {
    array(value)?.iter().map(line).collect()
}

fn format_error(what: &'static str, line: usize) -> LandError {
    LandError::Format {
        what,
        line: u32::try_from(line).unwrap_or(u32::MAX),
    }
}
