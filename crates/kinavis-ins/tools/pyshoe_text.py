#!/usr/bin/env python3
"""PyShoe's processed MAT files as text for the kinavis-ins tests.

For each trial, <name>.txt: a header line "# kind G_shoe shoe_err samples
truths" and then "imu t ax ay az wx wy wz shoe" (shoe: PyShoe's optimal SHOE
zero-velocity flag, 0 or 1) and "gt index x y z" lines; VICON truth at every
sample, hallway truth at the markers.

    python3 -I tools/pyshoe_text.py <pyshoe directory> <output directory>

Requires numpy and scipy.
"""

import glob
import sys
from pathlib import Path

import numpy as np
import scipy.io as sio


def write(out, kind, mat, truth_index):
    imu, ts = mat["imu"], mat["ts"].ravel()
    shoe = mat["zv_shoe_opt"].ravel() if "zv_shoe_opt" in mat else np.zeros(len(ts))
    g = float(mat["G_shoe_opt"].ravel()[0]) if "G_shoe_opt" in mat else 0.0
    err = float(mat["trial_error"].ravel()[0]) if "trial_error" in mat else -1.0
    gt = mat["gt"]
    with open(out, "w") as f:
        f.write(f"# {kind} {g:.6e} {err:.6f} {len(ts)} {len(gt)}\n")
        for t, row, z in zip(ts, imu, shoe):
            f.write("imu %.9f %s %d\n" % (t, " ".join("%.9e" % v for v in row), int(z)))
        for i, p in zip(truth_index, gt):
            f.write("gt %d %.9f %.9f %.9f\n" % (i, p[0], p[1], p[2]))


def main(source, target):
    source, target = Path(source), Path(target)
    target.mkdir(parents=True, exist_ok=True)
    for path in sorted(glob.glob(str(source / "data/vicon/processed/*.mat"))):
        mat = sio.loadmat(path)
        write(target / ("vicon-" + Path(path).stem + ".txt"), "vicon", mat, range(len(mat["gt"])))
    for path in sorted(glob.glob(str(source / "data/hallway/walk/*/*/processed_data.mat"))):
        mat = sio.loadmat(path)
        name = "hallway-" + Path(path).parent.parent.name + "-" + Path(path).parent.name
        write(target / (name + ".txt"), "hallway", mat, mat["gt_idx"].ravel())


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
