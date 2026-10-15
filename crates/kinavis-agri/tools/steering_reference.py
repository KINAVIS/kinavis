#!/usr/bin/env python3
"""Writes tests/data/steering_reference.txt from PythonRobotics.

The wheel angle of PythonRobotics' pure pursuit and Stanley controllers
(PathTracking/pure_pursuit and PathTracking/stanley_control) for random
machine states near a straight path and a circle, the paths sampled every
millimetre so that their nearest-point searches stand for the exact path.
Only the values are kept; none of their code is.

PythonRobotics' frame: x east, y north, yaw counter-clockwise from east,
a positive wheel angle turns left. Rows give the rear axle, yaw and speed in
that frame and the wheel angle as computed there.
Columns: controller path rear_x rear_y yaw speed delta. `path` is `line`
(through the origin, due north) or `circle` (radius 40 m about the origin,
clockwise).

Parameters as in the PythonRobotics sources: pure pursuit k = 0.1 s,
Lfc = 2.0 m, WB = 2.9 m; Stanley k = 0.5, L = 2.9 m.

    python3 -I tools/steering_reference.py PATH_TO_PYTHONROBOTICS

Requires numpy and matplotlib (imported by those modules), and a
checkout of https://github.com/AtsushiSakai/PythonRobotics.
"""

import math
import os
import random
import sys
import types

os.environ.setdefault("MPLBACKEND", "Agg")
sys.path.insert(0, sys.argv[1])
sys.path.insert(0, os.path.join(sys.argv[1], "PathTracking", "pure_pursuit"))
sys.path.insert(0, os.path.join(sys.argv[1], "PathTracking", "stanley_control"))

# utils.angle imports scipy's Rotation for a helper the controllers do not
# call; a stand-in keeps scipy, which some platforms cannot load, out.
_transform = types.ModuleType("scipy.spatial.transform")
_transform.Rotation = None
sys.modules.setdefault("scipy", types.ModuleType("scipy"))
sys.modules.setdefault("scipy.spatial", types.ModuleType("scipy.spatial"))
sys.modules["scipy.spatial.transform"] = _transform

import numpy as np  # noqa: E402
import pure_pursuit as pp  # noqa: E402
import stanley_control as st  # noqa: E402

RADIUS = 40.0
rng = random.Random(20261016)


def line_path():
    y = np.arange(-60.0, 200.0, 0.001)
    return np.zeros_like(y), y, np.full_like(y, math.pi / 2)


def circle_path():
    # Clockwise: the angle about the centre decreases.
    phi = np.arange(math.pi, -math.pi, -0.001 / RADIUS)
    return RADIUS * np.cos(phi), RADIUS * np.sin(phi), phi - math.pi / 2


def states(path):
    for _ in range(150):
        speed = rng.uniform(0.5, 6.0)
        error = rng.uniform(-1.5, 1.5)
        yaw_error = math.radians(rng.uniform(-15.0, 15.0))
        if path == "line":
            y = rng.uniform(0.0, 40.0)
            yield error, y, math.pi / 2 + yaw_error, speed
        else:
            phi = rng.uniform(0.0, math.pi / 2)
            r = RADIUS + error
            yield (r * math.cos(phi), r * math.sin(phi),
                   phi - math.pi / 2 + yaw_error, speed)


print(__doc__.splitlines()[0])
paths = {"line": line_path(), "circle": circle_path()}
with open("tests/data/steering_reference.txt", "w") as out:
    out.write("# controller path rear_x rear_y yaw speed delta\n")
    for name, (cx, cy, cyaw) in paths.items():
        course = pp.TargetCourse(list(cx), list(cy))
        for rear_x, rear_y, yaw, speed in states(name):
            # pure_pursuit.State takes the middle of the wheelbase.
            state = pp.State(x=rear_x + pp.WB / 2 * math.cos(yaw),
                             y=rear_y + pp.WB / 2 * math.sin(yaw), yaw=yaw, v=speed)
            course.old_nearest_point_index = None
            delta, _ = pp.pure_pursuit_steer_control(state, course, 0)
            out.write(f"pure_pursuit {name} {state.rear_x:.9f} {state.rear_y:.9f} "
                      f"{yaw:.12f} {speed:.9f} {float(delta):.12f}\n")
            state = st.State(x=rear_x, y=rear_y, yaw=yaw, v=speed)
            delta, _ = st.stanley_control(state, cx, cy, cyaw, 0)
            out.write(f"stanley {name} {rear_x:.9f} {rear_y:.9f} "
                      f"{yaw:.12f} {speed:.9f} {float(delta):.12f}\n")
