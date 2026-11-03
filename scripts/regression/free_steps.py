"""Largest change per time step of the secondary coordinates in COMAK results.

    python free_steps.py <runs dir> <case> <folder> <prefix> [tag ...]

Reads <runs dir>/<tag>/<case>/JAM/<folder>/results/comak/<prefix>_values.sto and prints the
largest change between consecutive frames of every secondary coordinate (rad for rotations,
m for translations), the size of the largest change in the first tag with a factor 2 of the
reference IK step is not computed here (see the max_change table in the example scripts).
Use it on a run with max_change = 0 (no limit) to see how far the coordinates really move
and to set max_change for a new activity.
"""
import re
import sys

import numpy as np

from sto import read_sto

runs, case, folder, prefix, tags = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5:]
SEC = ("knee_", "pf_", "meniscus_")

table = {}
for tag in tags:
    path = f"{runs}/{tag}/{case}/JAM/{folder}/results/comak/{prefix}_values.sto"
    deg = "inDegrees=yes" in open(path).read()
    labels, data = read_sto(path)
    for i, n in enumerate(labels):
        if n.startswith(SEC) and n not in ("knee_flex_r", "knee_flex_l"):
            v = data[:, i]
            if deg and not re.search(r"_t[xyz]_", n + "_"):
                v = np.radians(v)
            table.setdefault(n, {})[tag] = float(np.abs(np.diff(v)).max())

print(f"{'coordinate':26s}" + "".join(f"{t:>14s}" for t in tags))
for n, d in table.items():
    print(f"{n:26s}" + "".join(f"{d.get(t, float('nan')):14.5f}" for t in tags))
