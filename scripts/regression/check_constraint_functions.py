"""Check the secondary constraint functions written by COMAKInverseKinematicsTool.

    python check_constraint_functions.py <comak-inverse-kinematics results dir> \
        [--coupled /jointset/knee_r/knee_flex_r] [--npts 20]

Each SimmSpline knot vector must span exactly [min, max] of the swept coupled
coordinate in the sweep states file and contain npts points, and the knot values
must follow the sweep data. Exit code 1 if a check fails.
"""
import argparse
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from sto import read_sto


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("results_dir")
    ap.add_argument("--coupled", default="/jointset/knee_r/knee_flex_r")
    ap.add_argument("--npts", type=int, default=20)
    ap.add_argument("--tol", type=float, default=1e-5)
    a = ap.parse_args()

    d = Path(a.results_dir)
    sweep = next(d.glob("*_secondary_constraint_sweep_states.sto"))
    labels, data = read_sto(sweep)
    ind = data[:, labels.index(a.coupled + "/value")]
    ind_min, ind_max = ind.min(), ind.max()
    print(f"sweep {sweep.name}: {a.coupled} {ind_min:.6f} .. {ind_max:.6f} rad")

    root = ET.parse(d / "secondary_coordinate_constraint_functions.xml").getroot()
    failures = 0
    for spl in root.iter("SimmSpline"):
        name = spl.get("name")
        x = np.array(spl.find("x").text.split(), dtype=float)
        y = np.array(spl.find("y").text.split(), dtype=float)
        problems = []
        if len(x) != a.npts:
            problems.append(f"{len(x)} points, expected {a.npts}")
        if abs(x[0] - ind_min) > a.tol:
            problems.append(f"first knot {x[0]:.6f} != sweep min {ind_min:.6f}")
        if abs(x[-1] - ind_max) > a.tol:
            problems.append(f"last knot {x[-1]:.6f} != sweep max {ind_max:.6f}")
        col = labels.index(name + "/value") if (name + "/value") in labels else -1
        if col >= 0:
            ref = np.interp(x, ind, data[:, col])
            err = float(np.max(np.abs(ref - y)))
            if err > 1e-3:
                problems.append(f"knot values deviate from sweep by {err:.3g}")
        status = "FAIL" if problems else "ok  "
        failures += bool(problems)
        print(f"{status} {name}" + ("  " + "; ".join(problems) if problems else ""))
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
