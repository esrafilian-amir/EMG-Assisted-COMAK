"""Summarize a COMAK run: convergence, jitter of the secondary coordinates and activation jumps.

    python stats.py <runs>/<tag>/<case> [more run dirs ...]
"""
import glob
import re
import sys

import numpy as np

from sto import read_sto

SECONDARY = ("knee_add_r", "knee_rot_r", "knee_tx_r", "knee_ty_r", "knee_tz_r",
             "pf_flex_r", "pf_rot_r", "pf_tilt_r", "pf_tx_r", "pf_ty_r", "pf_tz_r")


def stats(d):
    L, D = read_sto(glob.glob(d + "/JAM/*/results/comak/*_convergence.sto")[0])
    c = {l: D[:, i] for i, l in enumerate(L)}
    log = open(d + "/run.log", errors="replace").read()
    wall = float(re.search(r'"wall_seconds": ([0-9.]+)', open(d + "/meta.json").read()).group(1))
    Lv, Dv = read_sto(glob.glob(d + "/JAM/*/results/comak/*_values.sto")[0])
    sec = [i for i, l in enumerate(Lv) if l in SECONDARY]
    dt = float(np.diff(Dv[:, 0]).mean())
    q = Dv[:, sec]
    jerk = float(np.sqrt((np.diff(q, 3, axis=0) ** 2).mean()) / dt ** 3)
    jump = float(np.abs(np.diff(q, axis=0)).max())
    La, Da = read_sto(glob.glob(d + "/JAM/*/results/comak/*_activations.sto")[0])
    ajump = float(np.abs(np.diff(Da[:, 1:], axis=0)).max())
    return dict(conv=int(c["converged"].sum()), n=len(D), it=float(c["iterations"].mean()),
                itmax=int(c["iterations"].max()), udot=float(c["max_udot_error"].max()),
                fails=log.count("COMAK Optimization failed"), jerk=jerk, jump=jump,
                ajump=ajump, wall=wall), Da, Dv, sec


def line(name, s):
    return ("%-22s conv %2d/%-2d | iters %4.1f (max %2d) | worst udot %5.2f | ipopt fails %3d | "
            "sec jerk %.3g | max sec jump %.4f | max act jump %.3f | %4.0fs" %
            (name, s["conv"], s["n"], s["it"], s["itmax"], s["udot"], s["fails"],
             s["jerk"], s["jump"], s["ajump"], s["wall"]))


if __name__ == "__main__":
    for d in sys.argv[1:]:
        s = stats(d)[0]
        print(line(d.rstrip("/").split("/")[-2], s))
