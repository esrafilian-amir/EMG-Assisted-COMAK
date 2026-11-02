"""Per-coordinate frame-to-frame jumps of the secondary coordinates of COMAK runs.

    python sec_jumps.py <run_dir> [<run_dir> ...]

Reads results/comak/*_values.sto, where rotations are in degrees and
translations in meters, and prints the largest jump between consecutive frames
(degrees or millimeters) and the range of motion of each secondary coordinate.
"""
import glob
import sys

import numpy as np

from sto import read_sto

SECONDARY = ("knee_add_r", "knee_rot_r", "knee_tx_r", "knee_ty_r", "knee_tz_r",
             "pf_flex_r", "pf_rot_r", "pf_tilt_r", "pf_tx_r", "pf_ty_r", "pf_tz_r")


def is_translation(name):
    return name.endswith(("_tx_r", "_ty_r", "_tz_r"))


def load(run_dir, window=None):
    L, D = read_sto(glob.glob(run_dir + "/JAM/*/results/comak/*_values.sto")[0])
    if window:
        t = D[:, 0]
        D = D[(t >= window[0]) & (t <= window[1])]
    return L, D


def table(run_dirs, window=None):
    data = {d: load(d, window) for d in run_dirs}
    names = [d.rstrip("/").split("/")[-2] for d in run_dirs]
    print("max jump between frames / range of motion   (deg for rotations, mm for translations)")
    print("%-12s " % "coordinate" + " ".join("%-22s" % n for n in names))
    for c in SECONDARY:
        row = []
        for d in run_dirs:
            L, D = data[d]
            q = D[:, L.index(c)] * (1000.0 if is_translation(c) else 1.0)
            row.append("%7.3f / %7.3f" % (np.abs(np.diff(q)).max(), q.max() - q.min()))
        print("%-12s " % c + " ".join("%-22s" % r for r in row))


if __name__ == "__main__":
    table(sys.argv[1:])
