"""Run the setup files written by an older install with a newer install.

The setup XMLs printed by an earlier run (inputs/*.xml of <src>) are copied to
the top of a fresh copy of that example folder and loaded with
COMAKInverseKinematicsTool(file) / COMAKTool(file). The old files do not have
the properties added since, so this checks that the defaults fill them in and
that the old values that equal an old default are upgraded (setup_version).

    python run_oldxml.py --install <install> --src <runs>/base2/walking \
        --folder exampleWalking --out <runs> --tag U_walking [--ik] [--frames 3]
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

MATLAB = r"C:\Program Files\MATLAB\R2022a\bin\matlab.exe"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--install", required=True)
    ap.add_argument("--src", required=True, help="run directory of the earlier run")
    ap.add_argument("--folder", required=True, help="example folder name")
    ap.add_argument("--out", required=True)
    ap.add_argument("--tag", required=True)
    ap.add_argument("--ik", action="store_true",
                    help="also run the old comak_inverse_kinematics_settings.xml")
    ap.add_argument("--frames", type=int, default=0,
                    help="stop COMAK after this many frames (0: as in the file)")
    ap.add_argument("--matlab", default=MATLAB)
    a = ap.parse_args()

    src = Path(a.src) / "JAM" / a.folder
    root = Path(a.out) / a.tag
    work = root / "JAM" / a.folder
    if root.exists():
        sys.exit(f"{root} exists; use a new --tag")
    work.mkdir(parents=True)
    subprocess.run(["cmd", "/c", "mklink", "/J", str(root / "JAM" / "models"),
                    str(Path(a.src) / "JAM" / "models")], capture_output=True)
    # inputs the settings refer to
    for sub in ("results/comak-inverse-kinematics",):
        if (src / sub).exists() and not a.ik:
            shutil.copytree(src / sub, work / sub)
    # the tools make one folder level only
    (work / "results").mkdir(exist_ok=True)
    ik_xml = src / "inputs" / "comak_inverse_kinematics_settings.xml"
    comak_xml = src / "inputs" / "comak_settings.xml"
    lines = []
    if a.ik:
        shutil.copy2(ik_xml, work / "old_ik.xml")
        lines.append("ik = COMAKInverseKinematicsTool('old_ik.xml');"
                     " disp('setup_version of IK file tool: '+string(ik.get_setup_version()));"
                     " ik.run();")
    shutil.copy2(comak_xml, work / "old_comak.xml")
    text = (work / "old_comak.xml").read_text()
    n_old = len(re.findall("<setup_version>", text))
    lines.append("c = COMAKTool('old_comak.xml');")
    if a.frames:
        m = re.search(r"<start_time>([^<]+)</start_time>", text)
        dt = re.search(r"<time_step>([^<]+)</time_step>", text)
        step = float(dt.group(1)) if dt and float(dt.group(1)) > 0 else 0.01
        lines.append("c.set_stop_time(%.6f);" % (float(m.group(1)) + step * (a.frames - 1)))
    lines.append("c.run();")
    lines.append("disp('setup_version after run: '+string(c.get_setup_version()));")

    install = Path(a.install)
    jar = (install / "sdk" / "Java" / "org-opensim-modeling.jar").as_posix()
    (work / "javaclasspath.txt").write_text("<before>\n" + jar + "\n")
    (work / "javalibrarypath.txt").write_text("<before>\n" + str(install / "bin") + "\n")
    env = dict(os.environ)
    env["PATH"] = str(install / "bin") + os.pathsep + env.get("PATH", "")
    wrapper = work / "run_old.m"
    wrapper.write_text(
        "import org.opensim.modeling.*;\n"
        f"addpath('{(install / 'Resources' / 'Code' / 'Matlab' / 'Utilities').as_posix()}');\n"
        "Logger.setLevelString('Info');\n" + "\n".join(lines) + "\n")
    log = root / "run.log"
    t0 = time.time()
    with open(log, "w") as f:
        rc = subprocess.run([a.matlab, "-batch", f"run('{wrapper.as_posix()}')"],
                            cwd=work, env=env, stdout=f,
                            stderr=subprocess.STDOUT).returncode
    wall = time.time() - t0
    errs = [l.rstrip() for l in open(log, errors="replace")
            if l.startswith("[error]") or l.startswith("[critical] Error")
            or "Error using" in l or "Undefined" in l]
    meta = dict(tag=a.tag, returncode=rc, error_lines=len(errs), first_errors=errs[:3],
                old_file_had_setup_version=bool(n_old), wall_seconds=round(wall, 1))
    (root / "meta.json").write_text(json.dumps(meta, indent=2))
    print(json.dumps(meta, indent=2))


if __name__ == "__main__":
    main()
