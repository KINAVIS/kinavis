#!/usr/bin/env python3
"""Writes tests/fixtures/region_reference.txt from GEOS and CGAL.

Shapes: the parcels of at least a hectare (hedges and ditch banks are
registered too), by id, of a sample of the Dutch crop parcel register
(Basisregistratie Gewaspercelen, RVO through PDOK, CC0), in RD New
(EPSG:28992), a plane in metres. Parcels with holes become regions.

Per parcel, from GEOS through shapely: the rings (outer first), the area and
the perimeter, and where 1100 points fall — inside (I), on the boundary (B)
or outside (O). The points are not written: both sides draw them from
splitmix64 with the parcel's seed, uniform over the bounding box widened by a
tenth on each side.

Per inset distance (3, 12 and 24 m): CGAL's offset along the straight
skeleton (tools/skeleton_offset.cpp) — EMPTY, MULTI when the result falls
apart, else its area and where 300 more points fall (judged by GEOS). Not
GEOS's own buffer: GEOS simplifies the outline by a hundredth of the
distance before offsetting, which moves mitred corners by metres.

    c++ -std=c++17 -O2 -I/opt/homebrew/include -L/opt/homebrew/lib \\
        tools/skeleton_offset.cpp -lgmp -lmpfr -o skeleton_offset
    python3 -I tools/region_reference.py brp-sample.json ./skeleton_offset \\
        > tests/fixtures/region_reference.txt

The sample was fetched with

    curl -o brp-sample.json "https://service.pdok.nl/rvo/brpgewaspercelen/wfs/v1_0?request=GetFeature&service=WFS&version=2.0.0&typeNames=brpgewaspercelen:BrpGewas&count=300&outputFormat=application/json&bbox=150000,440000,160000,450000"

Requires shapely 2 and CGAL 6 (headers, with GMP and MPFR).
"""

import json
import subprocess
import sys

import shapely
from shapely.geometry import Point, shape

MASK = (1 << 64) - 1
POINTS = 1100
MIN_AREA = 10_000.0
INSET_POINTS = 300
INSETS = (3.0, 12.0, 24.0)


class SplitMix64:
    def __init__(self, seed):
        self.state = seed & MASK

    def next(self):
        self.state = (self.state + 0x9E3779B97F4A7C15) & MASK
        z = self.state
        z = ((z ^ (z >> 30)) * 0xBF58476D1CE4E5B9) & MASK
        z = ((z ^ (z >> 27)) * 0x94D049BB133111EB) & MASK
        return z ^ (z >> 31)

    def unit(self):
        return (self.next() >> 11) * 2.0**-53


def points(seed, count, bounds):
    west, south, east, north = bounds
    pad_x = (east - west) / 10
    pad_y = (north - south) / 10
    rng = SplitMix64(seed)
    for _ in range(count):
        x = west - pad_x + rng.unit() * (east - west + 2 * pad_x)
        y = south - pad_y + rng.unit() * (north - south + 2 * pad_y)
        yield Point(x, y)


def where(geometry, seed, count, bounds):
    out = []
    for p in points(seed, count, bounds):
        if geometry.contains(p):
            out.append("I")
        elif geometry.touches(p):
            out.append("B")
        else:
            out.append("O")
    return "".join(out)


def ring(coords):
    # Closing vertex dropped; the register stores millimetres.
    return " ".join(f"{x:.4f} {y:.4f}" for x, y in list(coords)[:-1])


def skeleton_offsets(rings, offset_tool):
    """CGAL offsets of one region at each distance, as lists of polygons."""
    query = "".join(
        "".join(f"ring {ring(r.coords)}\n" for r in rings) + f"offset {distance}\n"
        for distance in INSETS)
    out = iter(subprocess.run([offset_tool], input=query, capture_output=True,
                              text=True, check=True).stdout.splitlines())
    results = []
    for line in out:
        polygons = []
        for _ in range(int(line.split()[1])):
            holes = int(next(out).split()[3])
            parts = []
            for _ in range(holes + 1):
                values = list(map(float, next(out).split()[1:]))
                parts.append(list(zip(values[::2], values[1::2])))
            polygons.append(shape({"type": "Polygon", "coordinates": parts}))
        results.append(polygons)
    return results


def main():
    features = json.load(open(sys.argv[1]))["features"]
    features = [f for f in features if shape(f["geometry"]).area >= MIN_AREA]
    features.sort(key=lambda f: f["id"])
    print("# GEOS", ".".join(map(str, shapely.geos_version)), "via shapely",
          shapely.__version__, "; insets: CGAL straight skeleton")
    print("# BRP Gewaspercelen (RVO/PDOK, CC0), EPSG:28992; see tools/region_reference.py")
    for index, feature in enumerate(features):
        # Rounded first, so both sides see the same coordinates.
        raw = shape(feature["geometry"])
        geometry = shape({"type": "Polygon", "coordinates": [
            [(round(x, 4), round(y, 4)) for x, y in r.coords]
            for r in [raw.exterior, *raw.interiors]]})
        assert geometry.is_valid, feature["id"]
        bounds = geometry.bounds
        seed = 20261015 + index
        rings = [geometry.exterior, *geometry.interiors]
        print(f"parcel {index} {len(geometry.interiors)}")
        for r in rings:
            print("ring", ring(r.coords))
        print(f"area {geometry.area:.6f}")
        print(f"perimeter {geometry.length:.6f}")
        print(f"points {seed} {POINTS} {where(geometry, seed, POINTS, bounds)}")
        for distance, polygons in zip(INSETS, skeleton_offsets(rings, sys.argv[2])):
            if not polygons:
                print(f"inset {distance} EMPTY")
            elif len(polygons) > 1:
                print(f"inset {distance} MULTI")
            else:
                inset = polygons[0]
                inset_seed = seed * 31 + int(distance)
                print(f"inset {distance} AREA {inset.area:.6f} {inset_seed} {INSET_POINTS} "
                      f"{where(inset, inset_seed, INSET_POINTS, bounds)}")


main()
