"""Run one JAM example case from a private copy so the repo stays clean.

The MATLAB example is copied into <out>/<tag>/JAM/<folder>/ with a junction to
the shared models folder, patched for headless/batch use (no visualizer, quiet
logging), and run with `matlab -batch` against the given OpenSim install.

    python run_case.py --install <install> --case walking --out <runs> --tag baseline
"""
import argparse
import json
import os
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
EXAMPLES = REPO / "Bindings" / "Java" / "Matlab" / "examples" / "JAM"
MATLAB = r"C:\Program Files\MATLAB\R2022a\bin\matlab.exe"

CASES = {
    "walking": ("exampleWalking", "exampleWalking.m", []),
    "walking_patella_constrained": (
        "exampleWalking", "exampleWalking_Patella_Constrained.m", []),
    # the shipped meniscus example has the IK and COMAK runs commented out
    "walking_meniscus": (
        "exampleWalkingMeniscus", "exampleWalkingMeniscus.m",
        [(r"^%\s*(comak_ik\.run\(\);)", r"\1"), (r"^%\s*(comak\.run\(\);)", r"\1")]),
    "emg": ("exampleWalking_EMG_Assisted", "exampleWalking_EMG_Assisted.m", []),
}

# Logging/visualization only; none of these change the numerical results.
COMMON_PATCHES = [
    (r"set_use_visualizer\(true\)", "set_use_visualizer(false)"),
    (r"set_verbose\(\d+\)", "set_verbose(0)"),
    (r"Logger\.setLevelString\('\w+'\)", "Logger.setLevelString('Info')"),
    (r"set_ipopt_diagnostics_level\(\d+\)", "set_ipopt_diagnostics_level(0)"),
    (r"set_write_vtp_files\(true\)", "set_write_vtp_files(false)"),
]


def git(*args):
    return subprocess.run(["git", "-C", str(REPO), *args], capture_output=True,
                          text=True).stdout.strip()


def make_link(link, target):
    if link.exists():
        return
    r = subprocess.run(["cmd", "/c", "mklink", "/J", str(link), str(target)],
                       capture_output=True, text=True)
    if r.returncode != 0:
        shutil.copytree(target, link)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--install", required=True)
    ap.add_argument("--case", required=True, choices=CASES)
    ap.add_argument("--out", required=True)
    ap.add_argument("--tag", required=True)
    ap.add_argument("--matlab", default=MATLAB)
    ap.add_argument("--insert-after", action="append", default=[],
                    metavar="REGEX::LINE",
                    help="add LINE after the first script line matching REGEX "
                         "(e.g. 'comak.set_settle_threshold::"
                         "comak.set_settle_plateau_tolerance(0.1);')")
    a = ap.parse_args()

    folder, script, extra = CASES[a.case]
    root = Path(a.out) / a.tag / a.case
    work = root / "JAM" / folder
    if work.exists():
        sys.exit(f"{work} already exists; use a new --tag")
    work.mkdir(parents=True)
    make_link(root / "JAM" / "models", EXAMPLES / "models")

    text = (EXAMPLES / folder / script).read_text()
    for pat, rep in COMMON_PATCHES + extra:
        text = re.sub(pat, rep, text, flags=re.M)
    for spec in a.insert_after:
        pat, line = spec.split("::", 1)
        text, n = re.subn(r"^(.*" + pat + r".*)$", r"\1\n" + line.replace("\\", r"\\"),
                          text, count=1, flags=re.M)
        if n != 1:
            sys.exit(f"--insert-after pattern not found: {pat}")
    (work / script).write_text(text)

    install = Path(a.install)
    jar = (install / "sdk" / "Java" / "org-opensim-modeling.jar").as_posix()
    binp = (install / "bin")
    # MATLAB's own classpath.txt and the machine PATH may point at another
    # OpenSim install. Put this install's jar first on the Java class path
    # (startup-folder javaclasspath.txt) and its bin first on PATH, so the jar
    # and every native library come from the same install.
    (work / "javaclasspath.txt").write_text("<before>\n" + jar + "\n")
    # the Java library path (where osimJavaJNI.dll is found) comes from MATLAB's
    # librarypath.txt, not PATH, so it needs the same treatment
    (work / "javalibrarypath.txt").write_text("<before>\n" + str(binp) + "\n")
    env = dict(os.environ)
    env["PATH"] = str(binp) + os.pathsep + env.get("PATH", "")
    wrapper = work / "run_wrapper.m"
    wrapper.write_text(
        "disp(['OpenSim jar in use: ' which('org.opensim.modeling.COMAKTool')]);\n"
        f"run('{(work / script).as_posix()}');\n")

    log = root / "run.log"
    t0 = time.time()
    with open(log, "w") as f:
        rc = subprocess.run(
            [a.matlab, "-batch", f"run('{wrapper.as_posix()}')"],
            cwd=work, env=env, stdout=f,
            stderr=subprocess.STDOUT).returncode
    wall = time.time() - t0

    meta = dict(case=a.case, tag=a.tag, install=str(install), returncode=rc,
                wall_seconds=round(wall, 1), git_head=git("rev-parse", "HEAD"),
                git_dirty=bool(git("status", "--porcelain")),
                patches=[p for p, _ in COMMON_PATCHES + extra],
                inserts=a.insert_after)
    (root / "meta.json").write_text(json.dumps(meta, indent=2))
    print(json.dumps(meta, indent=2))
    sys.exit(rc)


if __name__ == "__main__":
    main()
