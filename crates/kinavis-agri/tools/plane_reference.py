#!/usr/bin/env python3
"""Writes tests/data/plane_reference.txt from PROJ through pyproj.

For field origins over the latitudes worked by machinery and heights from
sea level to a high plateau, the centred field plane is PROJ's transverse
Mercator with lat_0 and lon_0 on the origin, k_0 = (R + h)/R for the
Gaussian radius R = sqrt(MN) at the origin, and no false origin. For random points within 20 km of each origin: easting, northing,
meridian convergence and point scale (pyproj's Proj.get_factors, by numerical
differentiation: about 1e-10).
Columns: origin_latitude origin_longitude height latitude longitude easting
northing convergence scale.

    python3 -I tools/plane_reference.py

Requires pyproj.
"""

import math
import random

from pyproj import Geod, Proj

A = 6378137.0
F = 1 / 298.257223563
E2 = F * (2 - F)


def gaussian_radius(latitude):
    s = math.sin(math.radians(latitude))
    return A * math.sqrt(1 - E2) / (1 - E2 * s * s)

rng = random.Random(20261015)
geod = Geod(ellps="WGS84")

ORIGINS = [
    (0.5, -47.9, 1100.0),     # Brazilian cerrado
    (-34.6, -60.2, 40.0),     # Pampas
    (38.4, -98.7, 600.0),     # Kansas
    (45.0, 30.0, 120.0),      # Ukrainian steppe
    (51.98, 5.66, 10.0),      # Gelderland
    (52.26, 21.01, 110.0),    # Mazovia
    (55.75, 37.62, 180.0),    # Moscow region
    (59.9, 10.75, 90.0),      # Østlandet
    (64.2, -51.7, 30.0),      # southern Greenland
    (31.0, 91.1, 4600.0),     # Tibetan plateau
    (-43.5, 172.0, 0.0),      # Canterbury Plains
]

print(__doc__.splitlines()[0])
with open("tests/data/plane_reference.txt", "w") as out:
    out.write("# origin_latitude origin_longitude height latitude longitude "
              "easting northing convergence scale\n")
    for lat0, lon0, h in ORIGINS:
        radius = gaussian_radius(lat0)
        proj = Proj(proj="tmerc", lat_0=lat0, lon_0=lon0, k_0=(radius + h) / radius,
                    x_0=0, y_0=0, ellps="WGS84")
        for _ in range(60):
            azimuth = rng.uniform(0, 360)
            distance = 20000.0 * math.sqrt(rng.random()) * 0.999
            lon, lat, _ = geod.fwd(lon0, lat0, azimuth, distance)
            x, y = proj(lon, lat)
            factors = proj.get_factors(lon, lat)
            out.write(f"{lat0} {lon0} {h} {lat:.12f} {lon:.12f} {x:.6f} {y:.6f} "
                      f"{factors.meridian_convergence:.12f} {factors.meridional_scale:.15f}\n")
