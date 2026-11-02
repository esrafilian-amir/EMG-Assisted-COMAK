"""Print and save a compact summary of one run made by run_case.py.

    python summarize.py <runs>/<tag>/<case>

Writes <case>/summary.json with COMAK convergence statistics, per-column
ranges of the main result files and the wall time, small enough to commit.
"""
import json
import re
import sys
from pathlib import Path

import numpy as np

from sto import read_sto


def column_stats(path):
    labels, data = read_sto(path)
    return {lab: [float(data[:, j].min()), float(data[:, j].mean()),
                  float(data[:, j].max())]
            for j, lab in enumerate(labels) if j > 0 and len(data)}


def main(case_dir):
    case = Path(case_dir)
    summary = {"meta": json.loads((case / "meta.json").read_text())}

    for conv in case.glob("JAM/*/results/comak/*_convergence.sto"):
        labels, d = read_sto(conv)
        col = {lab: d[:, i] for i, lab in enumerate(labels)}
        summary["comak"] = {
            "frames": int(len(d)),
            "converged": int(col["converged"].sum()),
            "iterations_mean": float(col["iterations"].mean()),
            "iterations_max": int(col["iterations"].max()),
            "max_udot_error_max": float(col["max_udot_error"].max()),
            "max_udot_error_mean": float(col["max_udot_error"].mean()),
        }

    files = {}
    for p in sorted(case.glob("JAM/*/results/**/*")):
        if p.suffix in (".sto", ".mot") and p.stat().st_size < 50e6:
            files[p.relative_to(case / "JAM").as_posix()] = column_stats(p)
    summary["files"] = files

    log = (case / "run.log").read_text(errors="replace")
    summary["finished_in"] = re.findall(r"Finished in (.+)", log)

    out = case / "summary.json"
    out.write_text(json.dumps(summary, indent=1))
    print(json.dumps({k: v for k, v in summary.items() if k != "files"}, indent=1))
    print(f"{len(files)} result files summarized -> {out}")


if __name__ == "__main__":
    main(sys.argv[1])
