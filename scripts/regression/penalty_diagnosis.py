"""Diagnose what the secondary-coordinate smoothness penalty does.

    python penalty_diagnosis.py <run_dir> [<run_dir> ...]

For each run (results/comak/*_values.sto: rotations in degrees, translations in m)
prints, over the secondary coordinates and per coordinate type:
  - the quantity that is penalized: RMS of the second difference (per frame^2),
  - the RMS of the first difference (frame to frame jumps),
  - the lag-1 autocorrelation of the second difference. A strongly negative value
    means the coordinate rings (overshoots and corrects) from frame to frame.
"""
import glob
import sys

import numpy as np

from sto import read_sto

ROT = ("knee_add_r", "knee_rot_r", "pf_flex_r", "pf_rot_r", "pf_tilt_r")
TRANS = ("knee_tx_r", "knee_ty_r", "knee_tz_r", "pf_tx_r", "pf_ty_r", "pf_tz_r")


def metrics(q):
    d1 = np.diff(q)
    d2 = np.diff(q, 2)
    ac = np.corrcoef(d2[:-1], d2[1:])[0, 1] if len(d2) > 3 and d2.std() > 0 else np.nan
    return np.sqrt((d1 ** 2).mean()), np.sqrt((d2 ** 2).mean()), ac


def main(run_dirs):
    print("%-14s | %-34s | %-34s" % ("run", "rotations (deg): 1st diff, 2nd diff, autocorr",
                                     "translations (mm): 1st diff, 2nd diff, autocorr"))
    for d in run_dirs:
        L, D = read_sto(glob.glob(d + "/JAM/*/results/comak/*_values.sto")[0])
        out = []
        for names, scale in ((ROT, 1.0), (TRANS, 1000.0)):
            m = np.array([metrics(D[:, L.index(n)] * scale) for n in names])
            out.append("%7.4f  %7.4f  %6.2f" % (m[:, 0].mean(), m[:, 1].mean(), np.nanmean(m[:, 2])))
        print("%-14s | %-34s | %-34s" % (d.rstrip("/").split("/")[-2], out[0], out[1]))


if __name__ == "__main__":
    main(sys.argv[1:])
