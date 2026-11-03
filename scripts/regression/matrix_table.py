"""Table of a set of runs: convergence, activation roughness, muscle forces, knee contact forces.

    python matrix_table.py <runs dir> <case> <folder> <comak prefix> <jnt prefix> tag1 tag2 ...

The first tag is the reference for the force differences. Forces are the muscle forces in the COMAK
force file (N); the contact forces are the tibiofemoral and patellofemoral total contact forces of the
joint mechanics ForceReporter file (N, magnitude of the x, y, z components).
"""
import json
import sys
from pathlib import Path

import numpy as np

from sto import read_sto

runs, case, folder, cprefix, jprefix, tags = (
    sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5], sys.argv[6:])


def comak(tag, name):
    return read_sto(f"{runs}/{tag}/{case}/JAM/{folder}/results/comak/{cprefix}_{name}.sto")


def contact(tag):
    f = Path(f"{runs}/{tag}/{case}/JAM/{folder}/results/joint-mechanics/{jprefix}_ForceReporter_forces.sto")
    if not f.exists():
        return {}
    labels, d = read_sto(f)
    out = {}
    def vec(pat):
        cols = [labels.index(pat + c) for c in "xyz" if pat + c in labels]
        return d[:, cols] if len(cols) == 3 else None
    tf = vec("tf_contact.tibia_cartilage.total.contact_force_")
    if tf is None:   # medial and lateral compartments are separate contacts: add the two forces
        lat = vec("tf_contact_Lat.tibia_cartilage_Lat.total.contact_force_")
        med = vec("tf_contact_Med.tibia_cartilage_Med.total.contact_force_")
        if lat is not None and med is not None:
            tf = lat + med
    if tf is not None:
        out["TF"] = np.linalg.norm(tf, axis=1)
    pf = vec("pf_contact.patella_cartilage.total.contact_force_")
    if pf is not None:
        out["PF"] = np.linalg.norm(pf, axis=1)
    return out


def rough(a):
    dd = np.diff(a, n=2, axis=0)
    return float(np.sqrt((dd ** 2).mean(axis=0)).mean())


rows = []
ref = None
for tag in tags:
    meta = json.loads(Path(f"{runs}/{tag}/{case}/meta.json").read_text()) if Path(f"{runs}/{tag}/{case}/meta.json").exists() else {}
    try:
        lc, dc = comak(tag, "convergence")
        la, da = comak(tag, "activations")
        lf, df = comak(tag, "force")
    except Exception as e:
        rows.append((tag, "no results: %s" % type(e).__name__))
        continue
    conv = int(dc[:, lc.index("converged")].sum())
    iters = dc[:, lc.index("iterations")]
    idx = [i for i, n in enumerate(la) if n.startswith("/forceset/")]
    act = da[:, idx]
    fidx = [i for i, n in enumerate(lf) if n.startswith("/forceset/")]
    force = df[:, fidx]
    total = force.sum(axis=1)
    if ref is None:
        ref = dict(force=force, total=total)
    n = min(len(force), len(ref["force"]))
    ct = contact(tag)
    rows.append((tag, dict(
        frames="%d/%d" % (conv, len(dc)), iters=iters.sum(), mean_it=iters.mean(),
        wall=meta.get("wall_seconds", float("nan")), rough=rough(act),
        act_mean=act.mean(), total_peak=total.max(), total_mean=total.mean(),
        dforce=np.abs(force[:n] - ref["force"][:n]).max() if n else float("nan"),
        tf_peak=ct["TF"].max() if "TF" in ct else float("nan"),
        tf_mean=ct["TF"].mean() if "TF" in ct else float("nan"),
        pf_peak=ct["PF"].max() if "PF" in ct else float("nan"))))

print("| run | converged | iterations (mean) | wall (s) | activation roughness | mean activation | "
      "sum of muscle forces peak / mean (N) | max change of a muscle force vs first run (N) | "
      "tibiofemoral contact peak / mean (N) | patellofemoral contact peak (N) |")
print("|---|---|---|---|---|---|---|---|---|---|")
for tag, r in rows:
    if isinstance(r, str):
        print("| %s | %s |" % (tag, r))
        continue
    print("| %s | %s | %d (%.2f) | %.0f | %.4f | %.3f | %.0f / %.0f | %.0f | %.0f / %.0f | %.0f |" % (
        tag, r["frames"], r["iters"], r["mean_it"], r["wall"], r["rough"], r["act_mean"],
        r["total_peak"], r["total_mean"], r["dforce"], r["tf_peak"], r["tf_mean"], r["pf_peak"]))
