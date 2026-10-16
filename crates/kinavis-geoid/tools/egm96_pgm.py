#!/usr/bin/env python3
"""Writes data/egm96-15.pgm from NGA's EGM96 15-minute geoid grid.

Source: us_nga_egm96_15.tif from PROJ-data (https://cdn.proj.org), NGA's
WW15MGH grid converted to GeoTIFF; public domain. Point registration, rows
from 90N to 90S, columns from 180W eastwards, metres, float32.

Output: the PGM layout read by GeographicLib's Geoid class and by
kinavis-geoid: 16-bit big-endian samples, N = offset + scale * sample, rows
from 90N, columns from 0E, no repeated 360E column.

    python3 -I tools/egm96_pgm.py path/to/us_nga_egm96_15.tif data/egm96-15.pgm

Requires numpy and tifffile.
"""

import hashlib
import sys

import numpy as np
import tifffile

SOURCE_SHA256 = "db493027562c9b004d7220fa881f5603adada4e1c5029b933fa7de4547b0e78d"
OFFSET = -108.0
SCALE = 0.003


def main(source: str, target: str) -> None:
    raw = open(source, "rb").read()
    digest = hashlib.sha256(raw).hexdigest()
    if digest != SOURCE_SHA256:
        sys.exit(f"{source}: SHA-256 {digest}, expected {SOURCE_SHA256}")
    grid = tifffile.imread(source).astype(np.float64)
    if grid.shape != (721, 1440):
        sys.exit(f"unexpected grid shape {grid.shape}")
    grid = np.roll(grid, -720, axis=1)
    samples = np.round((grid - OFFSET) / SCALE)
    if samples.min() < 0 or samples.max() > 65535:
        sys.exit("heights outside the 16-bit range of offset and scale")
    header = (
        "P5\n"
        "# Description WGS84 EGM96, 15-minute grid\n"
        "# Source NGA EGM96 WW15MGH (public domain) via PROJ-data us_nga_egm96_15.tif\n"
        f"# SourceSHA256 {SOURCE_SHA256}\n"
        f"# Offset {OFFSET:g}\n"
        f"# Scale {SCALE:g}\n"
        "# Origin 90N 0E\n"
        "1440 721\n"
        "65535\n"
    ).encode("ascii")
    body = samples.astype(">u2").tobytes()
    with open(target, "wb") as out:
        out.write(header + body)
    print(f"{target}: header {len(header)} bytes, "
          f"SHA-256 {hashlib.sha256(header + body).hexdigest()}")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2])
