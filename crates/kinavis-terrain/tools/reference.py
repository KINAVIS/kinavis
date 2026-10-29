#!/usr/bin/env python3
"""Writes tests/data/srtm_reference.txt from GDAL, through rasterio.

For each SRTM tile in the directory given: random positions in the tile, its
corners and edges and points on the sample grid; the height GDAL interpolates
bilinearly there, on GDAL's own georeferencing of the file. GDAL resamples in
the band's type, so the tile is copied as float64 into memory first.
Columns: tile latitude longitude height.

    python3 -I tools/reference.py ~/projects/rust/kinavis-data/srtm

Requires rasterio (GDAL 3.12 in its wheel) and numpy.
"""

import random
import sys
from pathlib import Path

import numpy as np
import rasterio
from rasterio.enums import Resampling
from rasterio.io import MemoryFile
from rasterio.windows import Window

TILES = ["N43E042", "N55E037"]
rng = random.Random(20261029)


def heights(path, points):
    with rasterio.open(path) as source:
        band = source.read(1).astype("float64")
        profile = source.profile
        inverse = ~source.transform
    profile.update(dtype="float64", driver="GTiff", nodata=None)
    out = []
    with MemoryFile() as memory, memory.open(**profile) as copy:
        copy.write(band, 1)
        last = band.shape[0] - 1
        for lat, lon in points:
            column, row = inverse * (lon, lat)
            # Area coordinates to sample coordinates; the last sample has no
            # neighbour beyond, so the window there is one sample back.
            r, c = min(row - 0.5, last - 1e-9), min(column - 0.5, last - 1e-9)
            r, c = max(r, 0.0), max(c, 0.0)
            value = np.empty((1, 1), dtype="float64")
            copy.read(1, window=Window(c, r, 1, 1), out=value,
                      resampling=Resampling.bilinear)
            out.append(value[0, 0])
    return out


def main(directory):
    with open("tests/data/srtm_reference.txt", "w") as out:
        out.write("# tile latitude longitude height\n")
        for name in TILES:
            south, west = int(name[1:3]), int(name[4:7])
            points = [(south + rng.random(), west + rng.random()) for _ in range(4900)]
            points += [(south + i / 3600, west + j / 3600)
                       for i, j in [(0, 0), (3600, 3600), (0, 3600), (3600, 0), (1800, 1800),
                                    (1, 3599), (3599, 1), (1234, 2345)]]
            points += [(south + rng.random(), west + 1.0) for _ in range(46)]
            points += [(south + 1.0, west + rng.random()) for _ in range(46)]
            for (lat, lon), height in zip(points, heights(Path(directory) / f"{name}.hgt", points)):
                out.write(f"{name} {lat:.12f} {lon:.12f} {height:.9f}\n")


if __name__ == "__main__":
    main(sys.argv[1])
