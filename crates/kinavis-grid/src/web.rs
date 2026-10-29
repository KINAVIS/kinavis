//! Web Mercator (EPSG:3857) and the XYZ tiles of web maps.
//!
//! Web Mercator is the Mercator projection of a *sphere* of radius equal to
//! the WGS 84 semi-major axis, applied to WGS 84 latitudes. It is not the
//! Mercator projection of the ellipsoid: it is not conformal there, its
//! northings are up to 43 km off the ellipsoidal Mercator, and no single
//! point scale exists. It is a display grid, not a survey grid.
//!
//! Tiles follow the OpenStreetMap scheme: at zoom `z` the square map
//! between ±[`WEB_MERCATOR_MAX_LATITUDE`] is cut into `2^z × 2^z` tiles of
//! [`TILE_PIXELS`] pixels, `x` eastward from 180° W, `y` southward from the
//! top edge. A quadkey names a tile by one base-4 digit per zoom level.

use core::fmt;
use core::str::FromStr;

use kinavis_kernel::error::{ensure_range, Excerpt, KernelError, Result};
use kinavis_kernel::{math, Distance, Latitude, Longitude, Position};

/// Radius of the Web Mercator sphere: the WGS 84 semi-major axis, m.
pub const WEB_MERCATOR_RADIUS: f64 = 6_378_137.0;

/// Latitude of the top and bottom edges of the square map, degrees:
/// `atan(sinh π)`, where the northing equals half the equator.
pub const WEB_MERCATOR_MAX_LATITUDE: f64 = 85.051_128_779_806_59;

/// Highest zoom level: tile indices and pixels stay exact in `u32` and
/// `f64`, and a pixel is under a millimetre.
pub const MAX_ZOOM: u8 = 30;

/// Pixels along a tile's side.
pub const TILE_PIXELS: u32 = 256;

/// Latitudes accepted beyond the edges, degrees, and drawn on them: the
/// limit as other implementations compute it in `f64`, 85.0511287798066, is
/// one unit in the last place above the correctly rounded one.
const LIMIT_SLACK: f64 = 1e-12;

/// Half the extent of the map, m: `π · a`.
const HALF_EXTENT: f64 = core::f64::consts::PI * WEB_MERCATOR_RADIUS;

/// A point on the Web Mercator grid, metres east and north of the
/// intersection of the equator and the prime meridian.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WebMercator {
    /// Easting, −π·a to π·a.
    pub x: Distance,
    /// Northing, −π·a to π·a within the square map.
    pub y: Distance,
}

impl WebMercator {
    /// Point of a WGS 84 position.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] beyond ±[`WEB_MERCATOR_MAX_LATITUDE`],
    /// give or take 10⁻¹²°.
    pub fn from_position(position: Position) -> Result<Self> {
        let limit = WEB_MERCATOR_MAX_LATITUDE + LIMIT_SLACK;
        ensure_range(
            "Web Mercator latitude",
            position.latitude().degrees(),
            -limit,
            limit,
        )?;
        Ok(Self::from_position_clamped(position))
    }

    /// Point of a WGS 84 position, latitudes beyond the map's edges drawn on
    /// them, as a map shows a point off its top.
    #[must_use]
    pub fn from_position_clamped(position: Position) -> Self {
        let latitude = position
            .latitude()
            .degrees()
            .clamp(-WEB_MERCATOR_MAX_LATITUDE, WEB_MERCATOR_MAX_LATITUDE);
        Self::from_degrees(latitude, position.longitude().degrees())
    }

    /// Point from its easting and northing.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] beyond ±π·a on either axis.
    pub fn new(x: Distance, y: Distance) -> Result<Self> {
        ensure_range("Web Mercator x", x.metres(), -HALF_EXTENT, HALF_EXTENT)?;
        ensure_range("Web Mercator y", y.metres(), -HALF_EXTENT, HALF_EXTENT)?;
        Ok(Self { x, y })
    }

    /// The WGS 84 position of the point; an easting of exactly π·a reads as
    /// 180° W.
    #[must_use]
    pub fn to_position(self) -> Position {
        let latitude = math::to_degrees(math::atan(math::sinh(
            self.y.metres() / WEB_MERCATOR_RADIUS,
        )));
        let longitude = math::to_degrees(self.x.metres() / WEB_MERCATOR_RADIUS);
        Position::new(
            Latitude::from_degrees_clamped(latitude),
            Longitude::from_degrees_wrapped(longitude),
        )
    }

    /// Fraction of the map's width and height, from the top left corner.
    fn unit(self) -> (f64, f64) {
        (
            f64::midpoint(self.x.metres() / HALF_EXTENT, 1.0),
            (1.0 - self.y.metres() / HALF_EXTENT) / 2.0,
        )
    }

    fn from_degrees(latitude: f64, longitude: f64) -> Self {
        let x = WEB_MERCATOR_RADIUS * math::to_radians(longitude);
        let y = WEB_MERCATOR_RADIUS * math::asinh(math::tan(math::to_radians(latitude)));
        Self {
            x: metres(x),
            y: metres(y.clamp(-HALF_EXTENT, HALF_EXTENT)),
        }
    }

    fn from_unit(u: f64, v: f64) -> Self {
        Self {
            x: metres((2.0 * u - 1.0) * HALF_EXTENT),
            y: metres((1.0 - 2.0 * v) * HALF_EXTENT),
        }
    }
}

/// Ground distance per pixel at a latitude and zoom, m: `2π·a·cos φ /
/// (256 · 2^z)`, on the sphere; the ellipsoid differs by up to 0.7 %.
///
/// # Errors
///
/// [`KernelError::OutOfRange`] for a zoom above [`MAX_ZOOM`].
pub fn metres_per_pixel(latitude: Latitude, zoom: u8) -> Result<Distance> {
    let tiles = tiles_across(zoom)?;
    let width = 2.0 * HALF_EXTENT * math::cos(latitude.radians());
    Distance::from_metres(width / (tiles * f64::from(TILE_PIXELS)))
}

/// A tile of the XYZ scheme.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct Tile {
    zoom: u8,
    x: u32,
    y: u32,
}

/// A point within a tile, in pixels from the tile's top left corner.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TilePoint {
    /// Tile.
    pub tile: Tile,
    /// Pixels rightward, 0 to [`TILE_PIXELS`].
    pub pixel_x: f64,
    /// Pixels downward, 0 to [`TILE_PIXELS`].
    pub pixel_y: f64,
}

/// Edges of a tile, degrees.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TileBounds {
    /// Western edge, from −180.
    pub west: f64,
    /// Southern edge.
    pub south: f64,
    /// Eastern edge, up to 180: the last column ends on the antimeridian.
    pub east: f64,
    /// Northern edge.
    pub north: f64,
}

impl Tile {
    /// Tile `x`, `y` at `zoom`.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] for a zoom above [`MAX_ZOOM`] or an index
    /// of `2^zoom` or more.
    pub fn new(zoom: u8, x: u32, y: u32) -> Result<Self> {
        let tiles = tiles_across(zoom)?;
        let last = tiles - 1.0;
        ensure_range("tile x", f64::from(x), 0.0, last)?;
        ensure_range("tile y", f64::from(y), 0.0, last)?;
        Ok(Self { zoom, x, y })
    }

    /// Zoom level.
    #[must_use]
    pub const fn zoom(self) -> u8 {
        self.zoom
    }

    /// Column, eastward from 180° W.
    #[must_use]
    pub const fn x(self) -> u32 {
        self.x
    }

    /// Row, southward from the top edge.
    #[must_use]
    pub const fn y(self) -> u32 {
        self.y
    }

    /// The tile holding a WGS 84 position at `zoom`, and where in it. A point
    /// on an edge belongs to the tile east or south of it, except on the
    /// bottom edge of the map.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] for a zoom above [`MAX_ZOOM`] or a
    /// latitude beyond ±[`WEB_MERCATOR_MAX_LATITUDE`].
    pub fn containing(position: Position, zoom: u8) -> Result<TilePoint> {
        Self::at(WebMercator::from_position(position)?, zoom)
    }

    /// The tile holding a Web Mercator point at `zoom`, and where in it.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] for a zoom above [`MAX_ZOOM`].
    pub fn at(point: WebMercator, zoom: u8) -> Result<TilePoint> {
        let tiles = tiles_across(zoom)?;
        let (u, v) = point.unit();
        let (x, pixel_x) = split(u * tiles, tiles);
        let (y, pixel_y) = split(v * tiles, tiles);
        Ok(TilePoint {
            tile: Self { zoom, x, y },
            pixel_x,
            pixel_y,
        })
    }

    /// Edges of the tile.
    #[must_use]
    pub fn bounds(self) -> TileBounds {
        let north_west = self.corner(0.0, 0.0);
        let south_east = self.corner(1.0, 1.0);
        TileBounds {
            west: north_west.0,
            south: south_east.1,
            east: south_east.0,
            north: north_west.1,
        }
    }

    /// The tile one zoom level out that holds this one; `None` at zoom 0.
    #[must_use]
    pub fn parent(self) -> Option<Self> {
        let zoom = self.zoom.checked_sub(1)?;
        Some(Self {
            zoom,
            x: self.x >> 1,
            y: self.y >> 1,
        })
    }

    /// The four tiles one zoom level in, in quadkey order: top left, top
    /// right, bottom left, bottom right; `None` at [`MAX_ZOOM`].
    #[must_use]
    pub fn children(self) -> Option<[Self; 4]> {
        if self.zoom >= MAX_ZOOM {
            return None;
        }
        let zoom = self.zoom.saturating_add(1);
        let (x, y) = (self.x << 1, self.y << 1);
        Some([(0, 0), (1, 0), (0, 1), (1, 1)].map(|(dx, dy)| Self {
            zoom,
            x: x | dx,
            y: y | dy,
        }))
    }

    /// Quadkey: per zoom level from the top, `x` bit plus twice the `y` bit.
    #[must_use]
    pub fn quadkey(self) -> Quadkey {
        let mut key = Quadkey {
            text: [0; MAX_ZOOM as usize],
            len: self.zoom,
        };
        let levels = (0..u32::from(self.zoom)).rev();
        for (slot, shift) in key.text.iter_mut().zip(levels) {
            let bit = |value: u32| value.checked_shr(shift).unwrap_or(0) & 1;
            let digit = bit(self.x) | (bit(self.y) << 1);
            *slot = b'0'.saturating_add(u8::try_from(digit).unwrap_or(0));
        }
        key
    }

    /// Longitude and latitude, degrees, of the point at fractions `(u, v)` of
    /// the tile from its top left corner; the longitude is exact at a corner.
    fn corner(self, u: f64, v: f64) -> (f64, f64) {
        let tiles = tiles_across(self.zoom).unwrap_or(1.0);
        let across = (f64::from(self.x) + u) / tiles;
        let down = (f64::from(self.y) + v) / tiles;
        let latitude = math::atan(math::sinh(core::f64::consts::PI * (1.0 - 2.0 * down)));
        (across * 360.0 - 180.0, math::to_degrees(latitude))
    }
}

impl TilePoint {
    /// The point as Web Mercator coordinates.
    #[must_use]
    pub fn to_web_mercator(self) -> WebMercator {
        let tiles = tiles_across(self.tile.zoom).unwrap_or(1.0);
        let size = f64::from(TILE_PIXELS);
        WebMercator::from_unit(
            (f64::from(self.tile.x) + self.pixel_x / size) / tiles,
            (f64::from(self.tile.y) + self.pixel_y / size) / tiles,
        )
    }

    /// The point's WGS 84 position.
    #[must_use]
    pub fn to_position(self) -> Position {
        self.to_web_mercator().to_position()
    }
}

/// A quadkey of up to [`MAX_ZOOM`] digits 0 to 3, stored inline; the empty
/// key is the single tile of zoom 0.
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub struct Quadkey {
    text: [u8; MAX_ZOOM as usize],
    len: u8,
}

impl Quadkey {
    /// The key as text.
    #[must_use]
    pub fn as_str(&self) -> &str {
        self.text
            .get(..usize::from(self.len))
            .and_then(|bytes| core::str::from_utf8(bytes).ok())
            .unwrap_or("")
    }

    /// The tile the key names.
    #[must_use]
    pub fn tile(&self) -> Tile {
        let mut tile = Tile {
            zoom: self.len,
            x: 0,
            y: 0,
        };
        for &digit in self.text.iter().take(usize::from(self.len)) {
            let value = u32::from(digit.saturating_sub(b'0'));
            tile.x = (tile.x << 1) | (value & 1);
            tile.y = (tile.y << 1) | (value >> 1);
        }
        tile
    }
}

impl FromStr for Quadkey {
    type Err = KernelError;

    /// Reads up to 30 digits 0 to 3.
    fn from_str(text: &str) -> Result<Self> {
        let bytes = text.as_bytes();
        let valid = bytes.len() <= usize::from(MAX_ZOOM)
            && bytes.iter().all(|byte| (b'0'..=b'3').contains(byte));
        if !valid {
            return Err(KernelError::Parse {
                what: "quadkey",
                input: Excerpt::new(text),
            });
        }
        let mut key = Self {
            text: [0; MAX_ZOOM as usize],
            len: u8::try_from(bytes.len()).unwrap_or(0),
        };
        for (slot, byte) in key.text.iter_mut().zip(bytes) {
            *slot = *byte;
        }
        Ok(key)
    }
}

impl fmt::Display for Quadkey {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

impl fmt::Debug for Quadkey {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Quadkey({:?})", self.as_str())
    }
}

/// `2^zoom` as `f64`.
fn tiles_across(zoom: u8) -> Result<f64> {
    ensure_range("zoom", f64::from(zoom), 0.0, f64::from(MAX_ZOOM))?;
    Ok(f64::from(1u32 << zoom))
}

/// A coordinate in tiles split into the whole tile, kept below `tiles`, and
/// the pixel within it.
fn split(value: f64, tiles: f64) -> (u32, f64) {
    let whole = math::trunc(value.clamp(0.0, tiles - 1.0));
    let index = u32::try_from(math::round_to_i32(whole)).unwrap_or(0);
    (index, (value - whole) * f64::from(TILE_PIXELS))
}

/// Metres as a [`Distance`]; every value here is finite.
fn metres(value: f64) -> Distance {
    Distance::from_metres(value).unwrap_or(Distance::ZERO)
}
