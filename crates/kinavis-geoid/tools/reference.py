#!/usr/bin/env python3
"""Writes tests/data/egm96_reference.txt: geoid heights by two other
implementations at fixed and random points.

Columns: latitude, longitude (degrees); bilinear by PROJ on NGA's float grid
(vgridshift on us_nga_egm96_15.tif); bilinear and cubic by GeographicLib's
GeoidEval on data/egm96-15.pgm, printed to 0.1 mm.

    python3 -I tools/reference.py path/to/us_nga_egm96_15.tif

Requires pyproj and GeographicLib's GeoidEval on the PATH.
"""

import random
import shutil
import subprocess
import sys
import tempfile

from pyproj import Transformer

SPECIAL = [
    (0.0, 0.0), (90.0, 0.0), (-90.0, 0.0), (90.0, 123.4), (-90.0, -77.7),
    (89.9, 0.1), (-89.9, 179.9), (89.76, -45.0), (-89.76, 135.0),
    (0.0, -180.0), (0.0, 179.99), (10.0, 179.9), (10.0, -179.9),
    (50.76, -1.3), (8.0, 81.0), (-5.0, 140.0), (59.95, 30.3),
    (45.125, 7.25), (45.0, 7.0), (-0.125, -0.125),
]


def main(tif: str) -> None:
    proj = Transformer.from_pipeline(
        f"+proj=vgridshift +grids={tif} +multiplier=1"
    )
    rng = random.Random(20261008)
    points = SPECIAL + [
        (rng.uniform(-90.0, 90.0), rng.uniform(-180.0, 180.0)) for _ in range(2000)
    ]
    query = "".join(f"{latitude:.9f} {longitude:.9f}\n" for latitude, longitude in points)
    with tempfile.TemporaryDirectory() as directory:
        shutil.copy("data/egm96-15.pgm", directory)

        def geoid_eval(*flags: str) -> list[str]:
            return subprocess.run(
                ["GeoidEval", "-d", directory, "-n", "egm96-15", *flags],
                input=query, capture_output=True, text=True, check=True,
            ).stdout.split()

        bilinear = geoid_eval("-l")
        cubic = geoid_eval()
    with open("tests/data/egm96_reference.txt", "w") as out:
        out.write("# latitude longitude bilinear_proj bilinear_geoideval cubic_geoideval\n")
        for (latitude, longitude), by_bilinear, by_cubic in zip(points, bilinear, cubic):
            _, _, by_proj = proj.transform(longitude, latitude, 0.0)
            out.write(
                f"{latitude:.9f} {longitude:.9f} {by_proj:.6f} {by_bilinear} {by_cubic}\n"
            )


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    main(sys.argv[1])
