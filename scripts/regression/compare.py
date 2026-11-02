"""Compare the .sto/.mot results of two runs made by run_case.py.

    python compare.py <runA>/<case> <runB>/<case> [--abs 1e-6] [--rel 1e-6]

For every result file present in both runs, each column is compared row by row.
A column passes if max|a-b| <= abs_tol + rel_tol * max|a|. Files with different
numbers of rows are reported as failures. Exit code 1 if anything fails.
"""
import argparse
import sys
from pathlib import Path

import numpy as np

from sto import read_sto


def result_files(run):
    base = Path(run) / "JAM"
    return {p.relative_to(base).as_posix(): p
            for p in base.glob("*/results/**/*")
            if p.suffix in (".sto", ".mot")}


def compare_file(pa, pb, abs_tol, rel_tol):
    la, da = read_sto(pa)
    lb, db = read_sto(pb)
    if da.shape[0] != db.shape[0]:
        return [("<rows>", da.shape[0], db.shape[0], False)]
    out = []
    for j, lab in enumerate(la):
        if lab not in lb:
            out.append((lab, "missing in B", "", False))
            continue
        a, b = da[:, j], db[:, lb.index(lab)]
        diff = float(np.nanmax(np.abs(a - b))) if len(a) else 0.0
        tol = abs_tol + rel_tol * (float(np.nanmax(np.abs(a))) if len(a) else 0.0)
        out.append((lab, diff, tol, diff <= tol))
    for lab in lb:
        if lab not in la:
            out.append((lab, "", "missing in A", False))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("a")
    ap.add_argument("b")
    ap.add_argument("--abs", type=float, default=1e-6, dest="abs_tol")
    ap.add_argument("--rel", type=float, default=1e-6, dest="rel_tol")
    ap.add_argument("--worst", type=int, default=5,
                    help="show this many worst columns per failing file")
    a = ap.parse_args()

    fa, fb = result_files(a.a), result_files(a.b)
    failed = False
    for name in sorted(set(fa) | set(fb)):
        if name not in fa or name not in fb:
            print(f"FAIL {name}: only in {'A' if name in fa else 'B'}")
            failed = True
            continue
        rows = compare_file(fa[name], fb[name], a.abs_tol, a.rel_tol)
        bad = [r for r in rows if not r[3]]
        if not bad:
            worst = max((r[1] for r in rows if isinstance(r[1], float)), default=0.0)
            print(f"ok   {name}  (max diff {worst:.3g})")
            continue
        failed = True
        print(f"FAIL {name}: {len(bad)}/{len(rows)} columns outside tolerance")
        bad.sort(key=lambda r: r[1] if isinstance(r[1], float) else np.inf,
                 reverse=True)
        for lab, diff, tol, _ in bad[:a.worst]:
            print(f"       {lab}: diff {diff} tol {tol}")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
