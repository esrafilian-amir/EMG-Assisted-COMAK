"""Roughness of the COMAK results over time and distance to a reference run.

    python rough.py <runs dir> <case> <folder> <prefix> tag1 tag2 ...

Roughness = RMS over time of the second difference (per frame^2) of a column, averaged over the
columns: muscle activations (unitless) and secondary coordinates (deg for rotations, mm for
translations as written in the .sto). The first tag is the reference for the distance columns.
"""
import sys
import numpy as np
from sto import read_sto

runs, case, folder, prefix, tags = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5:]
SEC = ("knee_", "pf_", "meniscus_")


def load(tag, name):
    return read_sto(f"{runs}/{tag}/{case}/JAM/{folder}/results/comak/{prefix}_{name}.sto")


def rough(d):
    dd = np.diff(d, n=2, axis=0)
    return float(np.sqrt((dd ** 2).mean(axis=0)).mean())


ref = {}
print(f"{'run':14s} {'act rough':>10s} {'act max step':>13s} {'sec rough':>10s} {'sec max step':>13s}"
      f" {'d act (max)':>12s} {'d sec (max)':>12s}")
for k, tag in enumerate(tags):
    ha, da = load(tag, "activations")
    hv, dv = load(tag, "values")
    ia = [i for i, n in enumerate(ha) if n.startswith("/forceset/")]
    iv = [i for i, n in enumerate(hv) if n.startswith(SEC) and n not in ("knee_flex_r", "knee_flex_l")]
    A, V = da[:, ia], dv[:, iv]
    if k == 0:
        ref = dict(A=A, V=V)
    n = min(len(A), len(ref["A"]))
    print(f"{tag:14s} {rough(A):10.5f} {np.abs(np.diff(A, axis=0)).max():13.4f} {rough(V):10.5f} "
          f"{np.abs(np.diff(V, axis=0)).max():13.4f} {np.abs(A[:n] - ref['A'][:n]).max():12.4f} "
          f"{np.abs(V[:n] - ref['V'][:n]).max():12.4f}")
