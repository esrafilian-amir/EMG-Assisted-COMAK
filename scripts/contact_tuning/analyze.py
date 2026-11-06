"""Compare the contact outputs of replay.py runs with a reference run.

    python analyze.py ref.json var1.json var2.json ...

Per contact and per quantity (force magnitude, max pressure, mean pressure, area):
  peak  = |peak(var) - peak(ref)| / peak(ref)                    (peak over the trial)
  rms   = rms(var - ref) / peak(ref)                             (whole trial)
  worst = max |var - ref| / peak(ref)                            (worst frame)
A setting passes at 5% when peak and rms of the force and of the max pressure are
both within 5% for every contact.
"""
import json
import sys

import numpy as np


def load(path):
    d = json.load(open(path))
    out = {}
    for name, c in d["contacts"].items():
        f = np.linalg.norm(np.array(c["force"]), axis=1)
        out[name] = {"force": f, "max_pressure": np.array(c["max_pressure"]),
                     "mean_pressure": np.array(c["mean_pressure"]),
                     "area": np.array(c["area"])}
    return d, out


def metrics(ref, var):
    r = {}
    for q in ("force", "max_pressure", "mean_pressure", "area"):
        a, b = ref[q], var[q]
        pk = np.max(a) if np.max(a) > 0 else 1.0
        r[q] = (abs(np.max(b) - np.max(a)) / pk,
                np.sqrt(np.mean((b - a) ** 2)) / pk,
                np.max(np.abs(b - a)) / pk)
    return r


def main():
    ref_d, ref = load(sys.argv[1])
    print("reference peaks:", {n: (round(float(v["force"].max()), 1),
                                   round(float(v["max_pressure"].max()) / 1e6, 2))
                               for n, v in ref.items()}, "(N, MPa)")
    hdr = "%-34s %6s | " % ("variant", "ok5%")
    for n in ref:
        hdr += "%s: Fpk Frms Fworst | Ppk Prms Pworst | Apk Arms || " % n
    print(hdr)
    for path in sys.argv[2:]:
        d, var = load(path)
        o = d["opts"]
        label = o.get("tag", path)
        ok = True
        row = ""
        for n in ref:
            if n not in var:
                row += "(missing) "
                ok = False
                continue
            m = metrics(ref[n], var[n])
            ok = ok and m["force"][0] <= 0.05 and m["force"][1] <= 0.05 and \
                m["max_pressure"][0] <= 0.05 and m["max_pressure"][1] <= 0.05
            row += "%s: %4.1f %4.1f %4.1f | %4.1f %4.1f %4.1f | %4.1f %4.1f || " % (
                n, 100 * m["force"][0], 100 * m["force"][1], 100 * m["force"][2],
                100 * m["max_pressure"][0], 100 * m["max_pressure"][1],
                100 * m["max_pressure"][2], 100 * m["area"][0], 100 * m["area"][1])
        print("%-34s %6s | %s (%.0fs)" % (label, "PASS" if ok else "fail", row, d["seconds"]))


main()
