//! A terrain of many tiles, which the application loads and holds.

use kinavis_kernel::error::{KernelError, Result};
use kinavis_kernel::geodesy::ElevationModel;
use kinavis_kernel::{Height, Position};

use crate::hgt::{HgtTile, TileCorner};

/// Where the tiles of a terrain come from: a role the application fills —
/// files mapped into memory, a flash partition, embedded bytes.
pub trait TileSource {
    /// The tile at a corner, if held.
    fn tile(&self, corner: TileCorner) -> Option<HgtTile<'_>>;
}

/// A source behind a reference: a borrowed slice, a shared store.
impl<S: TileSource + ?Sized> TileSource for &S {
    fn tile(&self, corner: TileCorner) -> Option<HgtTile<'_>> {
        (**self).tile(corner)
    }
}

/// Tiles in a slice, found by their corners.
impl TileSource for [HgtTile<'_>] {
    fn tile(&self, corner: TileCorner) -> Option<HgtTile<'_>> {
        self.iter().find(|tile| tile.corner() == corner).copied()
    }
}

impl<const N: usize> TileSource for [HgtTile<'_>; N] {
    fn tile(&self, corner: TileCorner) -> Option<HgtTile<'_>> {
        self.as_slice().tile(corner)
    }
}

/// The terrain of a tile source: the elevation at a position from the tile
/// that holds it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Terrain<S> {
    source: S,
}

impl<S: TileSource> Terrain<S> {
    /// Terrain over a source.
    pub const fn new(source: S) -> Self {
        Self { source }
    }

    /// The source.
    pub const fn source(&self) -> &S {
        &self.source
    }
}

impl<S: TileSource> ElevationModel for Terrain<S> {
    fn elevation(&self, position: Position) -> Result<Height> {
        let corner = TileCorner::of(position);
        let tile = self
            .source
            .tile(corner)
            .ok_or(KernelError::OutsideValidity {
                data: "terrain tile at the position",
            })?;
        tile.elevation(position)
    }
}
