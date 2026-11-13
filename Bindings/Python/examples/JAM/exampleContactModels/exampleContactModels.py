#!/usr/bin/env python
"""Run the walking models (original and with the tuned contact settings) and report the contact results.

For every model:
  1. COMAK inverse kinematics + COMAK (the settings of the JAM example scripts, run unchanged apart from
     the paths) and the joint mechanics tool, which writes the contact vtp files (VTK PolyData: pressure,
     proximity, ..., and the thickness of the contact meshes) and the ForceReporter forces;
  2. the contact outputs of every Smith2018ArticularContactForce for every frame of the COMAK states
     (force, center of pressure, maximum and mean pressure, contact area) -> contact_results.csv;
  3. the thickness of the variable thickness contact meshes -> thickness_<mesh>.npz.
Then one pdf report with the contact forces, pressures, centers of pressure, contact areas, the thickness
of the cartilage and the comparison of the tuned models with the originals.

    python exampleContactModels.py [--out results_dir] [--models a,b] [--jobs 3] [--stage all|pipeline|analyze|report]

Results (default: the results folder next to this script), one folder per model:
  results/contact_report.pdf, results/summary.csv
  results/<model>/inputs, comak-inverse-kinematics, comak, joint-mechanics, contact-analysis, log

Models (see CASES): lenhart2015 and lenhart2015_SDF (walking, no menisci),
smith2019, smith2019_SDF and smith2019_SDF_nonlinear (walking with menisci).
OpenSim 4.6 with the JAM classes and its python package (opensim) must be importable.
"""
import argparse
import csv
import json
import os
import re
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
EXAMPLES = HERE.parent                      # the JAM examples folder
# the models of the JAM examples (the python examples use ../models; the shared copy is in the matlab examples)
MODELS = EXAMPLES / "models"
if not MODELS.exists():
    MODELS = HERE.parents[3] / "Java" / "Matlab" / "examples" / "JAM" / "models"

WALK = ("exampleWalking/exampleWalking.py", "healthy/lenhart2015/")
MEN = ("exampleWalkingMeniscus/exampleWalkingMeniscus.py", "healthy/smith2019/")
CASES = {
    "lenhart2015": dict(example=WALK[0], model=WALK[1] + "lenhart2015.osim",
                        title="Walking, lenhart2015 (original)", pair=None),
    "lenhart2015_SDF": dict(
        example=WALK[0], model=WALK[1] + "lenhart2015_SDF.osim",
        title="Walking, lenhart2015 (tuned contact)", pair="lenhart2015"),
    "smith2019": dict(example=MEN[0], model=MEN[1] + "smith2019.osim",
                      title="Walking with menisci, smith2019 (original, linear)", pair=None),
    "smith2019_SDF": dict(
        example=MEN[0], model=MEN[1] + "smith2019_SDF.osim",
        title="Walking with menisci, smith2019 (tuned contact, linear)", pair="smith2019"),
    "smith2019_SDF_nonlinear": dict(
        example=MEN[0], model=MEN[1] + "smith2019_SDF_nonlinear.osim",
        title="Walking with menisci, smith2019 (tuned contact, nonlinear)", pair="smith2019"),
}
FIELDS = ("force_x", "force_y", "force_z", "force_mag", "cop_x", "cop_y", "cop_z",
          "max_pressure", "mean_pressure", "area")


# --------------------------------------------------------------------------- pipeline
def prepare_script(key, outdir):
    """The example script with the model, the paths and the options of this run."""
    case = CASES[key]
    text = (EXAMPLES / case["example"]).read_text()

    def sub(pattern, repl, count=1, flags=re.M):
        nonlocal text
        text, n = re.subn(pattern, lambda m: repl, text, flags=flags)
        if n < count:
            raise RuntimeError("example script %s: pattern '%s' not found" % (case["example"], pattern))

    models = str(MODELS).replace("\\", "/")
    sub(r"^os\.chdir\(.*\)$", "os.chdir(r'%s')" % str(outdir).replace("\\", "/"))
    # the result folders of the example (./results/<x>) are subfolders of the folder of this model
    text = text.replace("'./results/", "'./").replace("'./results'", "'.'")
    sub(r"^model_file = .*$", "model_file = '%s/%s'" % (models, case["model"]))
    text = text.replace("'../models/", "'%s/" % models)
    text = re.sub(r"set_use_visualizer\(True\)", "set_use_visualizer(False)", text)
    text = re.sub(r"^# (comak_ik\.run\(\))", r"\1", text, flags=re.M)      # meniscus example
    text = re.sub(r"^# (comak\.run\(\))", r"\1", text, flags=re.M)
    # joint mechanics after COMAK: the contact vtp files with the thickness of the meshes
    jm = ("comak.set_run_joint_mechanics(True)\n"
          "_jm = comak.upd_joint_mechanics()\n"
          "_jm.set_contacts(0, 'all')\n"
          "_jm.set_contact_outputs(0, 'all')\n"
          "_jm.set_contact_mesh_properties(0, 'thickness')\n"
          "_jm.set_write_vtp_files(True)\n"
          "_jm.set_write_h5_file(False)\n"
          "_jm.set_results_directory('./joint-mechanics')\n"
          "comak.run()\n")
    sub(r"^comak\.run\(\)$", jm)
    (Path(outdir) / "log").mkdir(exist_ok=True)
    script = Path(outdir) / "log" / "run_pipeline.py"
    script.write_text(text)
    return script


def parse_times(logfile):
    """Seconds of the IK (with the sweep), the joint mechanics and COMAK (without the joint mechanics),
    from the Finished in lines of the log (the line after the '<tool> complete' line)."""
    last, found = None, {}
    with open(logfile, errors="replace") as f:
        for line in f:
            m = re.match(r"[info] (COMAKInverseKinematics|JointMechanicsTool|COMAK) complete", line)
            if m:
                last = m.group(1)
                continue
            m = re.match(r"[info] Finished in (d+) second", line)
            if m and last:
                found[last] = int(m.group(1))
                last = None
    jm = found.get("JointMechanicsTool", 0)
    out = {}
    if "COMAKInverseKinematics" in found:
        out["ik_s"] = found["COMAKInverseKinematics"]
    if "COMAK" in found:
        out["comak_s"] = found["COMAK"] - jm        # COMAK includes the joint mechanics
    if jm:
        out["jm_s"] = jm
    return out


def run_pipeline(key, out):
    outdir = Path(out) / key
    outdir.mkdir(parents=True, exist_ok=True)
    script = prepare_script(key, outdir)
    log = outdir / "log" / "pipeline.log"
    t0 = time.time()
    with open(log, "w") as f:
        rc = subprocess.call([sys.executable, str(script)], cwd=str(outdir), stdout=f,
                             stderr=subprocess.STDOUT)
    info = {"model": key, "return_code": rc, "wall_seconds": time.time() - t0}
    text = log.read_text(errors="replace")
    info.update(parse_times(log))
    info["all_frames_converged"] = "All frames converged" in text
    info["errors"] = len(re.findall(r"\[error\]", text))
    (outdir / "log" / "pipeline_info.json").write_text(json.dumps(info, indent=1))
    print("%-36s done in %.0f s (rc %d, converged %s)" % (
        key, info["wall_seconds"], rc, info["all_frames_converged"]), flush=True)
    return info


# --------------------------------------------------------------------------- analysis
def find_states(outdir):
    cands = sorted((Path(outdir) / "comak").glob("*_states.sto"))
    cands = [c for c in cands if "settle" not in c.name]
    if not cands:
        raise FileNotFoundError("no COMAK states file in %s/comak" % outdir)
    return cands[0]


def analyze(key, out):
    import opensim as osim
    osim.Logger.setLevelString("Error")
    outdir = Path(out) / key
    ana = outdir / "contact-analysis"
    ana.mkdir(exist_ok=True)
    model = osim.Model(str(MODELS / CASES[key]["model"]))
    contacts = []
    fs = model.getForceSet()
    for i in range(fs.getSize()):
        f = fs.get(i)
        if f.getConcreteClassName() == "Smith2018ArticularContactForce":
            contacts.append(osim.Smith2018ArticularContactForce.safeDownCast(f))
    model.initSystem()
    traj = osim.StatesTrajectory.createFromStatesStorage(
        model, osim.Storage(str(find_states(outdir))), True, True)

    rows, header = [], ["time"]
    for c in contacts:
        header += ["%s__%s" % (c.getName(), q) for q in FIELDS]
    for k in range(traj.getSize()):
        s = traj.get(k)
        model.realizeReport(s)
        row = [s.getTime()]
        for c in contacts:
            fv = osim.OutputVec3.safeDownCast(c.getOutput("casting_total_contact_force")).getValue(s)
            cv = osim.OutputVec3.safeDownCast(c.getOutput("casting_total_center_of_pressure")).getValue(s)
            f3 = [fv.get(0), fv.get(1), fv.get(2)]
            row += f3 + [float(np.linalg.norm(f3))] + [cv.get(0), cv.get(1), cv.get(2)]
            for name in ("casting_total_max_pressure", "casting_total_mean_pressure",
                         "casting_total_contact_area"):
                row.append(osim.OutputDouble.safeDownCast(c.getOutput(name)).getValue(s))
        rows.append(row)
    with open(ana / "contact_results.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)

    # contact settings of the model, for the report
    settings = {}
    for c in contacts:
        settings[c.getName()] = dict(
            method=c.get_contact_method(), formulation=c.get_elastic_foundation_formulation(),
            lumped=bool(c.get_use_lumped_contact_model()), max_proximity=c.get_max_proximity(),
            voxel=c.get_field_voxel_size(), taper=c.get_field_edge_taper_width())
    # thickness of the variable thickness meshes
    meshes = {}
    cg = model.getContactGeometrySet()
    for i in range(cg.getSize()):
        g = cg.get(i)
        if g.getConcreteClassName() != "Smith2018ContactMesh":
            continue
        m = osim.Smith2018ContactMesh.safeDownCast(g)
        n = m.getNumFaces()
        h = np.array([m.getTriangleThickness(j) for j in range(n)])
        cen = m.getTriangleCenters()
        xyz = np.array([[cen.get(j).get(0), cen.get(j).get(1), cen.get(j).get(2)] for j in range(n)])
        np.savez_compressed(ana / ("thickness_%s.npz" % m.getName()), thickness=h, centers=xyz)
        meshes[m.getName()] = dict(variable=bool(m.get_use_variable_thickness()),
                                   smoothing=int(m.get_thickness_smoothing_iterations()),
                                   faces=n, E=m.get_elastic_modulus(), constant=m.get_thickness())
    (ana / "model_settings.json").write_text(json.dumps(
        {"contacts": settings, "meshes": meshes}, indent=1))
    print("%-36s analysis: %d frames, %d contacts" % (key, len(rows), len(contacts)), flush=True)


# --------------------------------------------------------------------------- report
def load_run(out, key):
    d = Path(out) / key
    ana = d / "contact-analysis"
    r = {"key": key, "dir": d}
    csvf = ana / "contact_results.csv"
    if not csvf.exists():
        return None
    with open(csvf) as f:
        rd = csv.reader(f)
        head = next(rd)
        data = np.array([[float(x) for x in row] for row in rd])
    r["time"] = data[:, 0]
    r["contacts"] = {}
    for j, h in enumerate(head[1:], start=1):
        name, q = h.split("__")
        r["contacts"].setdefault(name, {})[q] = data[:, j]
    r["settings"] = json.loads((ana / "model_settings.json").read_text())
    r["info"] = json.loads((d / "log" / "pipeline_info.json").read_text()) \
        if (d / "log" / "pipeline_info.json").exists() else {}
    if (d / "log" / "pipeline.log").exists():                  # the times of the tools, from the log
        r["info"].update(parse_times(d / "log" / "pipeline.log"))
    r["thickness"] = {}
    for p in ana.glob("thickness_*.npz"):
        z = np.load(p)
        r["thickness"][p.stem[len("thickness_"):]] = (z["thickness"], z["centers"])
    return r


def report(out, keys, pdf_path, png_dir=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_pdf import PdfPages

    plt.rcParams.update({"font.size": 8, "axes.titlesize": 9, "axes.labelsize": 8,
                         "axes.spines.top": False, "axes.spines.right": False,
                         "axes.grid": True, "grid.alpha": 0.3, "legend.frameon": False})
    runs = {k: load_run(out, k) for k in keys}
    runs = {k: v for k, v in runs.items() if v is not None}
    colors = ["#1f77b4", "#d62728", "#2ca02c", "#9467bd", "#ff7f0e", "#8c564b"]
    A4 = (11.69, 8.27)

    def page_title(fig, title, sub=None):
        fig.suptitle(title, fontsize=12, fontweight="bold", x=0.02, ha="left", y=0.985)
        if sub:
            fig.text(0.02, 0.945, sub, fontsize=8, color="0.35")

    def fmt_method(s):
        c = list(s["contacts"].values())
        kinds = {(x["method"], x["formulation"], x["lumped"]) for x in c}
        return "; ".join("%s / %s%s" % (m, f, " lumped" if l else "") for m, f, l in sorted(kinds))

    npage = [0]

    with PdfPages(pdf_path) as pdf:
        def save(fig):
            pdf.savefig(fig)
            if png_dir:
                Path(png_dir).mkdir(parents=True, exist_ok=True)
                npage[0] += 1
                fig.savefig(Path(png_dir) / ("page_%02d.png" % npage[0]), dpi=80)

        # ---- summary
        fig = plt.figure(figsize=A4)
        page_title(fig, "Contact model report: walking trial (COMAK, joint mechanics)",
                   "Smith2018ArticularContactForce. Time window 1.28 to 1.91 s of the walking trial. "
                   "Generated by exampleContactModels.py")
        ax = fig.add_axes([0.02, 0.05, 0.96, 0.84])
        ax.axis("off")
        cols = ["model", "contact method / formulation", "smoothing", "IK + sweep (s)", "COMAK (s)",
                "converged", "tf peak\nforce (N)", "tf peak\npressure (MPa)", "pf peak\nforce (N)",
                "pf peak\npressure (MPa)"]
        cells = []
        for k, r in runs.items():
            c = r["contacts"]
            sm = sorted({m["smoothing"] for m in r["settings"]["meshes"].values() if m["variable"]})
            inf = r["info"]
            tf, pf = c.get("tf_contact"), c.get("pf_contact")
            cells.append([k, fmt_method(r["settings"]), ",".join(str(x) for x in sm) or "-",
                          inf.get("ik_s", ""), inf.get("comak_s", ""),
                          "yes" if r["info"].get("all_frames_converged") else "?",
                          "%.0f" % tf["force_mag"].max() if tf else "",
                          "%.2f" % (tf["max_pressure"].max() / 1e6) if tf else "",
                          "%.0f" % pf["force_mag"].max() if pf else "",
                          "%.2f" % (pf["max_pressure"].max() / 1e6) if pf else ""])
        tab = ax.table(cellText=cells, colLabels=cols, cellLoc="center",
                       colWidths=[0.20, 0.17, 0.06, 0.08, 0.07, 0.07, 0.08, 0.09, 0.08, 0.10],
                       bbox=[0.0, max(0.05, 0.95 - 0.07 * (len(cells) + 1)), 1.0, min(0.9, 0.07 * (len(cells) + 1))])
        tab.auto_set_font_size(False)
        tab.set_fontsize(6.5)
        for (i, j), cell in tab.get_celld().items():
            cell.set_linewidth(0.3)
            if i == 0:
                cell.set_facecolor("#e8eef5")
                cell.set_text_props(fontweight="bold")
        save(fig)
        plt.close(fig)

        # ---- per model pages: one row per contact
        for k, r in runs.items():
            names = list(r["contacts"])
            per_page = 3
            for p0 in range(0, len(names), per_page):
                chunk = names[p0:p0 + per_page]
                fig, axs = plt.subplots(len(chunk), 4, figsize=A4, squeeze=False)
                page_title(fig, CASES[k]["title"],
                           "contact force (components and magnitude), pressure, center of pressure, contact area"
                           + ("  (page %d)" % (p0 // per_page + 1) if len(names) > per_page else ""))
                for i, n in enumerate(chunk):
                    c, t = r["contacts"][n], r["time"]
                    a = axs[i][0]
                    for q, lab, col in (("force_x", "x", "#1f77b4"), ("force_y", "y", "#2ca02c"),
                                        ("force_z", "z", "#ff7f0e")):
                        a.plot(t, c[q], color=col, lw=0.8, label=lab)
                    a.plot(t, c["force_mag"], color="k", lw=1.4, label="|F|")
                    a.set_title("%s: contact force (N)" % n, loc="left")
                    a.legend(ncol=4, fontsize=6)
                    a = axs[i][1]
                    a.plot(t, c["max_pressure"] / 1e6, color="#d62728", lw=1.2, label="max")
                    a.plot(t, c["mean_pressure"] / 1e6, color="#1f77b4", lw=1.2, label="mean")
                    a.set_title("pressure (MPa)", loc="left")
                    a.legend(fontsize=6)
                    a = axs[i][2]
                    for q, lab, col in (("cop_x", "x", "#1f77b4"), ("cop_y", "y", "#2ca02c"),
                                        ("cop_z", "z", "#ff7f0e")):
                        a.plot(t, c[q] * 1000, color=col, lw=1.0, label=lab)
                    a.set_title("center of pressure (mm)", loc="left")
                    a.legend(ncol=3, fontsize=6)
                    a = axs[i][3]
                    a.plot(t, c["area"] * 1e6, color="0.2", lw=1.2)
                    a.set_title("contact area (mm$^2$)", loc="left")
                    if i == len(chunk) - 1:
                        for a in axs[i]:
                            a.set_xlabel("time (s)")
                fig.tight_layout(rect=[0, 0, 1, 0.93])
                save(fig)
                plt.close(fig)

        # ---- comparisons: tuned against original
        for k, r in runs.items():
            ref_key = CASES[k]["pair"]
            if not ref_key or ref_key not in runs:
                continue
            ref = runs[ref_key]
            names = [n for n in r["contacts"] if n in ref["contacts"]]
            fig, axs = plt.subplots(len(names), 3, figsize=A4, squeeze=False)
            page_title(fig, "%s against %s" % (k, ref_key),
                       "contact force magnitude, maximum pressure and center of pressure distance; "
                       "the deviations are relative to the trial peak of the original")
            rowtxt = []
            for i, n in enumerate(names):
                c, c0 = r["contacts"][n], ref["contacts"][n]
                t, t0 = r["time"], ref["time"]
                for j, (q, sc, unit, ttl) in enumerate((("force_mag", 1, "N", "force |F|"),
                                                        ("max_pressure", 1e-6, "MPa", "max pressure"))):
                    a = axs[i][j]
                    a.plot(t0, c0[q] * sc, color="0.4", lw=1.4, label="original")
                    a.plot(t, c[q] * sc, color="#d62728", lw=1.0, label="tuned")
                    a.set_title("%s: %s (%s)" % (n, ttl, unit), loc="left")
                    if i == 0 and j == 0:
                        a.legend(fontsize=6)
                cop0 = np.stack([c0["cop_x"], c0["cop_y"], c0["cop_z"]], 1)
                cop = np.stack([c["cop_x"], c["cop_y"], c["cop_z"]], 1)
                m = min(len(cop0), len(cop))
                use = (c0["force_mag"][:m] > 0.1 * c0["force_mag"].max())
                d = np.linalg.norm(cop[:m] - cop0[:m], axis=1) * 1000
                axs[i][2].plot(t[:m], d, color="#2ca02c", lw=1.0)
                axs[i][2].set_title("center of pressure distance (mm)", loc="left")
                fpk = c0["force_mag"].max() or 1.0
                ppk = c0["max_pressure"].max() or 1.0
                m2 = min(len(c["force_mag"]), len(c0["force_mag"]))
                rowtxt.append("%s: peak force %+.1f%%, RMS %.1f%%; peak max pressure %+.1f%%, RMS %.1f%%; "
                              "COP mean %.2f mm (frames above 10%% of peak force)" % (
                    n, 100 * (c["force_mag"].max() - fpk) / fpk,
                    100 * np.sqrt(np.mean((c["force_mag"][:m2] - c0["force_mag"][:m2]) ** 2)) / fpk,
                    100 * (c["max_pressure"].max() - ppk) / ppk,
                    100 * np.sqrt(np.mean((c["max_pressure"][:m2] - c0["max_pressure"][:m2]) ** 2)) / ppk,
                    d[use[:m]].mean() if use.any() else float("nan")))
            for a in axs[-1]:
                a.set_xlabel("time (s)")
            fig.tight_layout(rect=[0, 0.12, 1, 0.93])
            fig.text(0.02, 0.01, "\n".join(rowtxt), fontsize=6.5, va="bottom", family="monospace")
            save(fig)
            plt.close(fig)

        # ---- thickness
        def plane(xyz):
            c = xyz - xyz.mean(0)
            _, _, vt = np.linalg.svd(c, full_matrices=False)
            return c @ vt[0], c @ vt[1]

        for k, r in runs.items():
            ref_key = CASES[k]["pair"]
            ref = runs.get(ref_key) if ref_key else None
            meshes = [m for m, v in r["settings"]["meshes"].items() if v["variable"]]
            if not meshes:
                continue
            fig, axs = plt.subplots(len(meshes), 3, figsize=A4, squeeze=False)
            page_title(fig, "Cartilage thickness: %s" % CASES[k]["title"],
                       "thickness of the contact meshes (measured by a ray to the bone mesh; smoothing = "
                       "iterations of Laplacian smoothing over the neighbor triangles)")
            for i, mname in enumerate(meshes):
                h, xyz = r["thickness"][mname]
                x, y = plane(xyz)
                a = axs[i][0]
                sc = a.scatter(x * 1000, y * 1000, c=h * 1000, s=1.0, cmap="viridis", vmin=1, vmax=5,
                               rasterized=True)
                a.set_aspect("equal")
                a.set_title("%s, smoothing %d (mm)" % (mname, r["settings"]["meshes"][mname]["smoothing"]),
                            loc="left")
                a.grid(False)
                fig.colorbar(sc, ax=a, fraction=0.04)
                a = axs[i][1]
                if ref is not None and mname in ref["thickness"]:
                    h0, xyz0 = ref["thickness"][mname]
                    x0, y0 = plane(xyz0)
                    sc = a.scatter(x0 * 1000, y0 * 1000, c=h0 * 1000, s=1.0, cmap="viridis", vmin=1,
                                   vmax=5, rasterized=True)
                    a.set_title("%s, smoothing %d (original)" % (
                        mname, ref["settings"]["meshes"][mname]["smoothing"]), loc="left")
                    fig.colorbar(sc, ax=a, fraction=0.04)
                    a.set_aspect("equal")
                a.grid(False)
                if not (ref is not None and mname in ref["thickness"]):
                    a.axis("off")
                a = axs[i][2]
                bins = np.linspace(0.5, max(h.max(), 1e-3) * 1000 + 0.5, 60)
                a.hist(h * 1000, bins=bins, color="#d62728", alpha=0.6, label="this model")
                if ref is not None and mname in ref["thickness"]:
                    a.hist(ref["thickness"][mname][0] * 1000, bins=bins, histtype="step", color="0.2",
                           label="original")
                a.set_title("thickness histogram: mean %.2f, std %.2f mm" % (h.mean() * 1000, h.std() * 1000),
                            loc="left")
                a.set_xlabel("thickness (mm)")
                a.legend(fontsize=6)
            fig.tight_layout(rect=[0, 0, 1, 0.93])
            save(fig)
            plt.close(fig)

        # ---- notes
        fig = plt.figure(figsize=A4)
        page_title(fig, "Notes")
        fig.text(0.03, 0.88, "\n".join([
            "Contact force: total force of the casting mesh (tibia or patella cartilage; for the menisci the casting mesh of the contact) in N.",
            "Pressure: maximum and mean over the contacting triangles (MPa). Contact area in mm^2.",
            "Center of pressure: as reported by the contact output total_center_of_pressure (m, plotted in mm).",
            "Thickness: per triangle thickness of the variable thickness cartilage meshes; thickness_smoothing_iterations averages it",
            "  over the edge neighbors (graph Laplacian), which removes the triangle to triangle jumps of the measured thickness.",
            "distance_field: the depth of a casting triangle is read from a signed distance grid of the target mesh (see",
            "  scripts/contact_tuning/RESULTS.md for the comparison with the ray cast and the recommended settings).",
            "Files per model: <model>/comak (states, activations, forces), <model>/joint-mechanics (contact vtp files with pressure",
            "  and thickness for ParaView, forces .sto), <model>/contact-analysis (contact_results.csv, thickness_*.npz), <model>/inputs, <model>/log.",
        ]), fontsize=8, va="top", family="monospace")
        save(fig)
        plt.close(fig)
    print("report:", pdf_path, flush=True)


def write_summary(out, keys):
    """summary.csv: run times and the peak contact results of every model."""
    rows = []
    for k in keys:
        r = load_run(out, k)
        if r is None:
            continue
        inf = r["info"]
        row = {"model": k, "ik_and_sweep_s": inf.get("ik_s", ""), "joint_mechanics_s": inf.get("jm_s", ""),
               "comak_s": inf.get("comak_s", ""),
               "all_frames_converged": r["info"].get("all_frames_converged", "")}
        for n, c in r["contacts"].items():
            row[n + "_peak_force_N"] = "%.2f" % c["force_mag"].max()
            row[n + "_peak_max_pressure_MPa"] = "%.3f" % (c["max_pressure"].max() / 1e6)
            row[n + "_peak_mean_pressure_MPa"] = "%.3f" % (c["mean_pressure"].max() / 1e6)
            row[n + "_peak_area_mm2"] = "%.1f" % (c["area"].max() * 1e6)
        rows.append(row)
    cols = []
    for row in rows:
        for c in row:
            if c not in cols:
                cols.append(c)
    with open(Path(out) / "summary.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        w.writerows(rows)


# --------------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=str(HERE / "results"), help="results folder (default: results next to this script)")
    ap.add_argument("--models", default=",".join(CASES), help="comma separated, default all")
    ap.add_argument("--jobs", type=int, default=3, help="pipelines run in parallel")
    ap.add_argument("--stage", default="all", choices=["all", "pipeline", "analyze", "report"])
    ap.add_argument("--skip-existing", action="store_true",
                    help="do not rerun a pipeline whose COMAK states file exists")
    ap.add_argument("--png-dir", default=None, help="also write every page of the report as a png")
    ap.add_argument("--pdf", default=None, help="name of the pdf (default <out>/contact_report.pdf)")
    a = ap.parse_args()
    keys = [k.strip() for k in a.models.split(",") if k.strip()]
    for k in keys:
        if k not in CASES:
            sys.exit("unknown model %s (choose from %s)" % (k, ", ".join(CASES)))
    out = Path(a.out).resolve()
    out.mkdir(parents=True, exist_ok=True)

    if a.stage in ("all", "pipeline"):
        todo = [k for k in keys if not (a.skip_existing and
                                       list((out / k / "comak").glob("*_states.sto")))]
        with ThreadPoolExecutor(max_workers=max(1, a.jobs)) as ex:
            list(ex.map(lambda k: run_pipeline(k, out), todo))
    if a.stage in ("all", "analyze"):
        for k in keys:
            analyze(k, out)
    if a.stage in ("all", "report"):
        report(out, keys, a.pdf or str(out / "contact_report.pdf"), a.png_dir)
        write_summary(out, keys)


if __name__ == "__main__":
    main()
