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
    # examples without COMAK: forward simulation, joint mechanics, ligaments, contact
    "anterior_laxity": ("exampleAnteriorLaxity", "exampleAnteriorLaxity.m", []),
    "passive_flexion": ("examplePassiveFlexion", "examplePassiveFlexion.m", []),
    "passive_flexion_meniscus": (
        "examplePassiveFlexionMeniscus", "examplePassiveFlexionMeniscus.m", []),
    "sensitivity": (
        "exampleSensitivity", "exampleLigamentAndContactSensitivity.m", []),
    "visualize_kinematics": (
        "exampleVisualizeKinematics", "exampleVisualizeKinematics.m", []),
    # the shipped example has the four COMAK runs commented out
    "muscle_coordination": (
        "exampleWalkingMuscleCoordination", "exampleWalkingMuscleCoordination.m",
        [(r"^%\s*(comak_ik\.run\(\);)", r"\1"), (r"^%\s*(comak\.run\(\);)", r"\1"),
         (r"^%\s*(comak_muscle_weight\.run\(\);)", r"\1"),
         (r"^%\s*(comak_emg\.run\(\);)", r"\1")]),
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


def disable_muscle_in_model(text, folder, root, muscle, inertia_scales=(), model_subs=()):
    """Copy the model folder (Geometry linked, files copied) with `muscle` set
    to appliesForce=false and/or body inertias scaled, and point the script at
    the copy."""
    m = re.search(r"model_file\s*=\s*'([^']+)'", text)
    model_path = (EXAMPLES / folder / m.group(1)).resolve()
    models_root = (EXAMPLES / "models").resolve()
    rel_dir = model_path.parent.relative_to(models_root).as_posix()
    dest = root / "JAM" / "models_mod" / rel_dir
    dest.mkdir(parents=True, exist_ok=True)
    for item in model_path.parent.iterdir():
        if item.is_dir():
            make_link(dest / item.name, item)
        elif item == model_path:
            osim = item.read_text()
            if muscle:
                # muscles do not list appliesForce explicitly, so add it
                osim, n = re.subn(
                    r'(<[A-Za-z0-9]*Muscle[A-Za-z0-9]* name="' + re.escape(muscle) + r'">)',
                    "\\1\n<appliesForce>false</appliesForce>", osim, count=1)
                if n != 1:
                    sys.exit(f"muscle {muscle} not found in {item.name}")
            for spec in inertia_scales:
                body, factor = spec.split("=")
                factor = float(factor)

                def scale(m):
                    values = m.group(2).split()
                    scaled = [repr(float(v) * factor) for v in values[:3]]
                    return m.group(1) + " ".join(scaled + values[3:]) + m.group(3)

                osim, n = re.subn(
                    r'(<Body name="' + re.escape(body) + r'">.*?<inertia>)([^<]*)(</inertia>)',
                    scale, osim, count=1, flags=re.S)
                if n != 1:
                    sys.exit(f"body {body} not found in {item.name}")
            for spec in model_subs:
                pat, rep = spec.split("::", 1)
                osim, n = re.subn(pat, rep, osim, flags=re.S)
                if n == 0:
                    sys.exit(f"--model-sub pattern not found: {pat}")
            (dest / item.name).write_text(osim)
        else:
            shutil.copy2(item, dest / item.name)
    return text.replace(f"../models/{rel_dir}/", f"../models_mod/{rel_dir}/")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--install", required=True)
    ap.add_argument("--case", required=True, choices=CASES)
    ap.add_argument("--out", required=True)
    ap.add_argument("--tag", required=True)
    ap.add_argument("--matlab", default=MATLAB)
    ap.add_argument("--disable-muscle", metavar="NAME",
                    help="run with this muscle set to appliesForce=false, using "
                         "a modified copy of the model (Geometry is linked)")
    ap.add_argument("--scale-inertia", action="append", default=[],
                    metavar="BODY=FACTOR",
                    help="multiply the diagonal inertia of this body by FACTOR in a "
                         "modified copy of the model (Geometry is linked)")
    ap.add_argument("--model-sub", action="append", default=[], metavar="REGEX::REPL",
                    help="regex substitution applied to the text of the model file "
                         "(a modified copy is used; Geometry is linked)")
    ap.add_argument("--scale-max-change", type=float, metavar="FACTOR",
                    help="multiply every set_max_change(value) of the script by FACTOR")
    ap.add_argument("--insert-after", action="append", default=[],
                    metavar="REGEX::LINE",
                    help="add LINE after the first script line matching REGEX "
                         "(e.g. 'comak.set_settle_threshold::"
                         "comak.set_settle_plateau_tolerance(0.1);')")
    ap.add_argument("--reuse-ik", metavar="RUN_DIR",
                    help="copy results/comak-inverse-kinematics from this earlier "
                         "run (e.g. <runs>/baseline/walking) and skip the "
                         "COMAKInverseKinematicsTool run")
    ap.add_argument("--keep-ik-run", action="store_true",
                    help="with --reuse-ik, only copy the IK results (for the "
                         "constraint functions) and still run the IK tool")
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
    if a.scale_max_change is not None:
        text = re.sub(r"set_max_change\(([0-9.eE+-]+)\)",
                      lambda m: "set_max_change(%g)" % (float(m.group(1)) * a.scale_max_change),
                      text)
    if a.disable_muscle or a.scale_inertia or a.model_sub:
        text = disable_muscle_in_model(text, folder, root, a.disable_muscle,
                                       a.scale_inertia, a.model_sub)
    if a.reuse_ik:
        src = Path(a.reuse_ik) / "JAM" / folder / "results" / "comak-inverse-kinematics"
        shutil.copytree(src, work / "results" / "comak-inverse-kinematics")
        if not a.keep_ik_run:
            text, n = re.subn(r"^(\s*)(comak_ik\.run\(\);)", r"\1% \2", text,
                              flags=re.M)
            if n != 1:
                sys.exit("could not find comak_ik.run(); to skip")
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
    utilities = (install / "Resources" / "Code" / "Matlab" / "Utilities").as_posix()
    wrapper.write_text(
        # the MATLAB helper functions of the install (osimTableFromStruct, ...)
        f"addpath('{utilities}');\n"
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

    # The tools catch their own exceptions and log them as [error], so MATLAB can
    # exit with 0 although a tool did nothing. Count them.
    error_lines = [l.rstrip() for l in open(log, errors="replace")
                   if l.startswith("[error]") or l.startswith("[critical] Error")]

    meta = dict(case=a.case, tag=a.tag, install=str(install), returncode=rc,
                error_lines=len(error_lines), first_errors=error_lines[:3],
                wall_seconds=round(wall, 1), git_head=git("rev-parse", "HEAD"),
                git_dirty=bool(git("status", "--porcelain")),
                patches=[p for p, _ in COMMON_PATCHES + extra],
                inserts=a.insert_after, disabled_muscle=a.disable_muscle)
    (root / "meta.json").write_text(json.dumps(meta, indent=2))
    print(json.dumps(meta, indent=2))
    sys.exit(rc)


if __name__ == "__main__":
    main()
