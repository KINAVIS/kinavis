#!/usr/bin/env python3
"""Writes tests/data/gpsbabel_gpx.expected from gpxpy (Apache 2.0).

For every GPX file of GPSBabel's reference directory (GPL-2.0 data, fetched
at a pinned commit, never committed): what gpxpy reads in it — the number of
waypoints, routes, route points, tracks, segments and track points; the
sums of their latitudes and longitudes; the number and sum of elevations and
of times (seconds since 1970). Files gpxpy refuses are marked `refused`.
Columns: file wpt rte rtept trk trkseg trkpt lat lon ele ele_sum time time_sum.

    python3 -I tools/gpx_reference.py <gpsbabel checkout>
"""

import glob
import sys
from pathlib import Path

import gpxpy


def points(gpx):
    for p in gpx.waypoints:
        yield p
    for r in gpx.routes:
        yield from r.points
    for t in gpx.tracks:
        for s in t.segments:
            yield from s.points


def main(checkout):
    rows = []
    for path in sorted(glob.glob(str(Path(checkout) / "reference" / "*.gpx"))):
        name = Path(path).name
        try:
            with open(path, encoding="utf-8") as f:
                gpx = gpxpy.parse(f)
        except Exception as error:  # gpxpy's own errors and decoding
            rows.append(f"{name} refused")
            continue
        all_points = list(points(gpx))
        elevations = [p.elevation for p in all_points if p.elevation is not None]
        times = [p.time.timestamp() for p in all_points if p.time is not None]
        rows.append(" ".join(str(v) for v in [
            name, len(gpx.waypoints), len(gpx.routes), sum(len(r.points) for r in gpx.routes),
            len(gpx.tracks), sum(len(t.segments) for t in gpx.tracks),
            sum(len(s.points) for t in gpx.tracks for s in t.segments),
            f"{sum(p.latitude for p in all_points):.9f}", f"{sum(p.longitude for p in all_points):.9f}",
            len(elevations), f"{sum(elevations):.6f}", len(times), f"{sum(times):.3f}",
        ]))
    with open("tests/data/gpsbabel_gpx.expected", "w") as out:
        out.write("# file wpt rte rtept trk trkseg trkpt lat lon ele ele_sum time time_sum\n")
        out.write("\n".join(rows) + "\n")


if __name__ == "__main__":
    main(sys.argv[1])
