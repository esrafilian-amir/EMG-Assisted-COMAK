"""Check COMAK's tendon compliance against the real compliant-tendon muscle model.

    python tendon_check.py <install> <model.osim> <comak results dir> <prefix> [frame step]

For frames of a finished COMAK run with use_tendon_compliance, the optimized coordinates and
activations are set in the muscle model with compliant tendons, the fiber-tendon equilibrium is
solved at zero fiber velocity (Model.equilibrateMuscles), and the tendon force of every muscle is
compared with (a) the muscle force COMAK reported and (b) the rigid tendon force for the same
activations (ignore_tendon_compliance true).
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from sto import read_sto

install, model_file, results, prefix = sys.argv[1:5]
step = int(sys.argv[5]) if len(sys.argv) > 5 else 8
os.environ["PATH"] = os.path.join(install, "bin") + os.pathsep + os.environ["PATH"]
sys.path.insert(0, os.path.join(install, "sdk", "Python"))
import opensim as osim

osim.Logger.setLevelString("Error")


def make(rigid):
    m = osim.Model(model_file)
    ms = m.getMuscles()
    for i in range(ms.getSize()):
        ms.get(i).set_ignore_tendon_compliance(rigid)
        ms.get(i).set_ignore_activation_dynamics(True)
    return m, m.initSystem()


compliant, cstate = make(False)
rigid, rstate = make(True)
cms, rms = compliant.getMuscles(), rigid.getMuscles()
names = [cms.get(i).getName() for i in range(cms.getSize())]

lv, dv = read_sto(f"{results}/{prefix}_values.sto")
la, da = read_sto(f"{results}/{prefix}_activations.sto")
lf, df = read_sto(f"{results}/{prefix}_force.sto")
in_deg = "inDegrees=yes" in open(f"{results}/{prefix}_values.sto").read()
act_cols = [la.index("/forceset/" + n) for n in names]
frc_cols = [lf.index("/forceset/" + n) for n in names]


def set_coords(model, s, row):
    for j, label in enumerate(lv):
        if j == 0:
            continue
        try:
            c = model.getCoordinateSet().get(label)
        except Exception:
            continue
        v = dv[row, j]
        if in_deg and c.getMotionType() == osim.Coordinate.Rotational:
            v = np.radians(v)
        c.setValue(s, v, False)
    model.assemble(s)
    model.realizePosition(s)


rows = list(range(0, len(dv), step))
err_c, err_r, shown = [], [], []
for row in rows:
    a = da[row, act_cols]
    rep = df[row, frc_cols]
    out = {}
    for tag, model, state, ms in (("compliant", compliant, cstate, cms), ("rigid", rigid, rstate, rms)):
        s = osim.State(state)
        set_coords(model, s, row)
        controls = osim.Vector(model.getNumControls(), 0.0)
        for k, n in enumerate(names):
            ms.get(n).addInControls(osim.Vector(1, float(a[k])), controls)
        model.setControls(s, controls)
        model.realizeVelocity(s)
        if tag == "compliant":
            model.equilibrateMuscles(s)
        model.realizeDynamics(s)
        out[tag] = np.array([ms.get(n).getTendonForce(s) if tag == "compliant" else ms.get(n).getActuation(s) for n in names])
    big = rep > 50
    if big.any():
        err_c.append(np.abs(out["compliant"] - rep)[big] / rep[big])
        err_r.append(np.abs(out["rigid"] - rep)[big] / rep[big])
    shown.append((dv[row, 0], np.abs(out["compliant"] - rep).max(), np.abs(out["rigid"] - rep).max(),
                  np.abs(out["compliant"] - out["rigid"]).max()))

print("frame time | max |actual compliant - COMAK| (N) | max |rigid - COMAK| (N) | max |compliant - rigid| (N)")
for t, a, b, c in shown:
    print("%9.3f | %8.1f | %8.1f | %8.1f" % (t, a, b, c))
ec, er = np.concatenate(err_c), np.concatenate(err_r)
print("muscles above 50 N over %d frames: relative difference to the COMAK force: compliant model median %.1f%% / 95th %.1f%%; rigid model median %.1f%% / 95th %.1f%%" % (
    len(rows), 100 * np.median(ec), 100 * np.percentile(ec, 95), 100 * np.median(er), 100 * np.percentile(er, 95)))
