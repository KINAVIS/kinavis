#!/usr/bin/env python3
"""Writes the reference files in tests/data from two other implementations.

utm_ups_reference.txt — GeographicLib's GeoConvert: for random points over
the whole Earth, the standard UTM/UPS zone, easting and northing (-u), and
the meridian convergence and point scale (-c), to a micrometre.
Columns: latitude longitude zone hemisphere easting northing convergence
scale; zone 0 is UPS.

mgrs_reference.txt — GeoConvert again: the MGRS reference of random points
at precisions 0 to 11 (-m), and the centre of each referenced square read
back (-u). Columns: latitude longitude precision reference zone hemisphere
easting northing; zone 0 is UPS.

gauss_kruger_reference.txt — PROJ through pyproj, from each datum's
geographic CRS to its projected CRS, so no datum shift is involved: for
random points in each preset's zone, easting and northing.
Columns: preset zone latitude longitude easting northing.

    python3 -I tools/reference.py

Requires pyproj and GeographicLib's GeoConvert on the PATH.
"""

import random
import subprocess

from pyproj import Transformer

rng = random.Random(20261008)


def geoconvert(points, *flags, precision="6"):
    query = "".join(f"{lat:.12f} {lon:.12f}\n" for lat, lon in points)
    out = subprocess.run(["GeoConvert", *flags, "-p", precision], input=query,
                         capture_output=True, text=True, check=True).stdout
    return [line.split() for line in out.splitlines()]


def mgrs():
    points = [(rng.uniform(-90.0, 90.0), rng.uniform(-180.0, 180.0)) for _ in range(3000)]
    points += [(rng.uniform(72.0, 84.0), rng.uniform(0.0, 42.0)) for _ in range(200)]
    points += [(rng.uniform(56.0, 64.0), rng.uniform(0.0, 12.0)) for _ in range(200)]
    points += [(rng.uniform(-1.0, 1.0), rng.uniform(-180.0, 180.0)) for _ in range(200)]
    with open("tests/data/mgrs_reference.txt", "w") as out:
        out.write("# latitude longitude precision reference zone hemisphere easting northing\n")
        for precision in range(0, 12):
            chosen = points[precision::12] if precision > 5 else points[precision::6]
            references = [row[0] for row in geoconvert(chosen, "-m", precision=str(precision - 5))]
            query = "".join(f"{reference}\n" for reference in references)
            centres = subprocess.run(["GeoConvert", "-u", "-p", "6"], input=query,
                                     capture_output=True, text=True, check=True).stdout.splitlines()
            for (lat, lon), reference, centre in zip(chosen, references, centres):
                zone_hemisphere, easting, northing = centre.split()
                zone = zone_hemisphere[:-1] or "0"
                out.write(f"{lat:.12f} {lon:.12f} {precision} {reference} {zone} "
                          f"{zone_hemisphere[-1]} {easting} {northing}\n")


def utm_ups():
    points = [(rng.uniform(-90.0, 90.0), rng.uniform(-180.0, 180.0)) for _ in range(4000)]
    # Zone edges, the Norway and Svalbard exceptions, the band limits, poles.
    points += [(rng.uniform(56.0, 64.0), rng.uniform(0.0, 12.0)) for _ in range(300)]
    points += [(rng.uniform(72.0, 84.0), rng.uniform(0.0, 42.0)) for _ in range(300)]
    points += [(lat, lon) for lat in (-90.0, -80.0, -79.9999, 0.0, 83.9999, 84.0, 90.0)
               for lon in (-180.0, -3.0, 0.0, 3.0, 9.0, 21.0, 179.9999)]
    grid = geoconvert(points, "-u")
    distortion = geoconvert(points, "-c")
    with open("tests/data/utm_ups_reference.txt", "w") as out:
        out.write("# latitude longitude zone hemisphere easting northing convergence scale\n")
        for (lat, lon), coordinate, (gamma, k) in zip(points, grid, distortion):
            zone_hemisphere, easting, northing = coordinate
            zone = zone_hemisphere[:-1] or "0"
            hemisphere = zone_hemisphere[-1]
            out.write(f"{lat:.12f} {lon:.12f} {zone} {hemisphere} {easting} {northing} {gamma} {k}\n")


PRESETS = [
    # name, EPSG of the geographic CRS, EPSG of zone n, zone width, zones
    ("pulkovo_1942", 4284, lambda n: 28400 + n, 6, range(2, 33)),
    ("pulkovo_1995", 4200, lambda n: 20000 + n, 6, range(4, 33)),
    ("gsk_2011", 7683, lambda n: 20900 + n, 6, range(4, 33)),
    ("dhdn", 4314, lambda n: 31464 + n, 3, range(2, 6)),
    ("bng", 4277, lambda n: 27700, 0, [0]),
]


def gauss_kruger():
    with open("tests/data/gauss_kruger_reference.txt", "w") as out:
        out.write("# preset zone latitude longitude easting northing\n")
        for name, geographic, projected, width, zones in PRESETS:
            for _ in range(2000):
                zone = rng.choice(list(zones))
                if width == 0:
                    lat, lon = rng.uniform(49.9, 60.9), rng.uniform(-8.0, 2.0)
                else:
                    middle = width * zone - (3.0 if width == 6 else 0.0)
                    lat = rng.uniform(35.0, 82.0)
                    lon = middle + rng.uniform(-width / 2, width / 2)
                t = Transformer.from_crs(f"EPSG:{geographic}", f"EPSG:{projected(zone)}",
                                         always_xy=True)
                easting, northing = t.transform(lon, lat)
                out.write(f"{name} {zone} {lat:.12f} {lon:.12f} {easting:.6f} {northing:.6f}\n")


if __name__ == "__main__":
    utm_ups()
    mgrs()
    gauss_kruger()
