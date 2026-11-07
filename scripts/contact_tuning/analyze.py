"""Compare the contact outputs of replay.py runs with a reference run.

    python analyze.py ref.json var1.json var2.json ... [--tol 0.05] [--cop-min 0.1]

Per contact, with the reference peak as the scale:
  Fpk / Frms : trial peak and RMS error of the contact force magnitude
  Ppk / Prms : same for the maximum pressure
  COP        : center of pressure distance between variant and reference (frames where the
               reference force is above cop-min * peak), as % of the contact patch
               diameter (2*sqrt(area/pi) of the reference): mean and max
A setting passes at the tolerance when Fpk, Frms, Ppk, Prms (and the COP mean) are all
within it for every contact. The 5% and 10% verdicts are both printed.
"""
import json
import sys

import numpy as np


def load(path):
    d = json.load(open(path))
    out = {}
    for name, c in d["contacts"].items():
        out[name] = {"force": np.linalg.norm(np.array(c["force"]), axis=1),
                     "max_pressure": np.array(c["max_pressure"]),
                     "mean_pressure": np.array(c["mean_pressure"]),
                     "area": np.array(c["area"]),
                     "cop": np.array(c["cop"]) if "cop" in c else None}
    return d, out


def metrics(ref, var, cop_min):
    r = {}
    for q in ("force", "max_pressure", "mean_pressure", "area"):
        a, b = ref[q], var[q]
        pk = np.max(a) if np.max(a) > 0 else 1.0
        r[q] = (abs(np.max(b) - np.max(a)) / pk, np.sqrt(np.mean((b - a) ** 2)) / pk)
    r["cop"] = (np.nan, np.nan)
    if ref["cop"] is not None and var["cop"] is not None:
        use = (ref["force"] > cop_min * ref["force"].max()) & (var["force"] > 0)
        if use.any():
            dist = np.linalg.norm(var["cop"][use] - ref["cop"][use], axis=1)
            diam = 2 * np.sqrt(np.maximum(ref["area"][use], 1e-12) / np.pi)
            rel = dist / diam
            r["cop"] = (rel.mean(), rel.max(), dist.mean() * 1000, dist.max() * 1000)
    return r


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    cop_min = 0.1
    for i, a in enumerate(sys.argv):
        if a == "--cop-min":
            cop_min = float(sys.argv[i + 1])
            args = [x for x in args if x != sys.argv[i + 1]]
    ref_d, ref = load(args[0])
    names = list(ref)
    print("reference peaks:", {n: (round(float(v["force"].max()), 1),
                                   round(float(v["max_pressure"].max()) / 1e6, 2))
                               for n, v in ref.items()}, "(N, MPa)")
    print("%-26s %5s %5s | per contact: Fpk Frms | Ppk Prms | COPmean COPmax (%% of patch) COPmean(mm)"
          % ("variant", "ok5", "ok10"))
    for path in args[1:]:
        d, var = load(path)
        label = d["opts"].get("tag", path)
        worst = 0.0
        row = ""
        for n in names:
            if n not in var:
                row += " (missing)"
                worst = 9
                continue
            m = metrics(ref[n], var[n], cop_min)
            vals = [m["force"][0], m["force"][1], m["max_pressure"][0], m["max_pressure"][1]]
            cop = m["cop"]
            if not np.isnan(cop[0]):
                vals.append(cop[0])
            worst = max(worst, max(vals))
            short = n.replace("_contact", "").replace("meniscus", "mn")[:14]
            row += " | %s %.1f %.1f | %.1f %.1f | %s" % (
                short, 100 * m["force"][0], 100 * m["force"][1], 100 * m["max_pressure"][0],
                100 * m["max_pressure"][1],
                ("%.1f %.1f %.2fmm" % (100 * cop[0], 100 * cop[1], cop[2])) if not np.isnan(cop[0]) else "n/a")
        print("%-26s %5s %5s%s" % (label, "yes" if worst <= 0.05 else "no",
                                   "yes" if worst <= 0.10 else "no", row))


main()
